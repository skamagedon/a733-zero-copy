/*
 * gstcedarzcdec.c - zero-copy Cedar H.264 decoder element for Allwinner A733
 *
 * The point of this element is to be the thing the vendor's gst-omx was
 * supposed to be. On A733 the vendor OMX component advertises
 * video/x-raw(memory:DMABuf) but cannot deliver it: it reports the proprietary
 * colour format 0x7f000002 through a malformed port-format enumeration, so
 * gst-omx aborts in negotiation. Everyone then falls back to
 * "omxh264dec disable-dma-feature=true ! videoconvert ! ximagesink", which
 * costs about 1.8 CPU cores for 1080p.
 *
 * This element talks to libvdecoder directly - the layer *underneath* the
 * broken OMX wrapper - and exports each decoded surface's dma_buf fd as
 * GstDmaBufMemory. So existing pipelines work unchanged:
 *
 *   gst-launch-1.0 filesrc location=clip.mp4 ! qtdemux ! h264parse \
 *       ! cedarzcdec ! kmssink
 *
 * No CPU pixel copy occurs anywhere in this element.
 *
 * Buffer lifetime is the subtle part. Cedar owns a small pool of surfaces and
 * recycles them, so a picture must not be returned while downstream still holds
 * it. Each output buffer therefore carries a weak reference: when GStreamer
 * finalises the buffer, ReturnPicture hands that surface back. The fd given to
 * the allocator is a dup(), because GstDmaBufMemory closes the fd it is given
 * and Cedar's original must stay valid for reuse.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/video/gstvideodecoder.h>
#include <gst/allocators/gstdmabuf.h>

#include <string.h>
#include <unistd.h>

#include "vdecoder.h"
#include "memoryAdapter.h"

/* Exported by libVE.so but absent from veInterface.h. Dispatches on engine
 * type: 0 = getVeAwOpsS (H.264/H.265), 1 = getVeVp9OpsS. Declaring it void
 * passes garbage and then crashes. */
#define VE_OPS_TYPE_AW 0
extern VeOpsS *GetVeOpsS (int type);

GST_DEBUG_CATEGORY_STATIC (cedarzcdec_debug);
#define GST_CAT_DEFAULT cedarzcdec_debug

#define GST_TYPE_CEDAR_ZC_DEC (gst_cedar_zc_dec_get_type ())
G_DECLARE_FINAL_TYPE (GstCedarZcDec, gst_cedar_zc_dec, GST, CEDAR_ZC_DEC,
    GstVideoDecoder)

struct _GstCedarZcDec
{
  GstVideoDecoder parent;

  VideoDecoder *dec;
  struct ScMemOpsS *memops;
  GstAllocator *dmabuf_alloc;
  GstVideoCodecState *input_state;

  /* Guards dec against the buffer-release callback, which runs on whatever
   * thread finalises a buffer - possibly after stop() has begun. */
  GMutex lock;
  gboolean shutting_down;
  gint outstanding;

  gboolean configured;
  gint width, height;           /* display size, after crop */
};

G_DEFINE_TYPE (GstCedarZcDec, gst_cedar_zc_dec, GST_TYPE_VIDEO_DECODER);

/* Byte-stream with access-unit alignment is required, not merely preferred:
 * libvdecoder must be handed whole access units. Feeding it arbitrary byte
 * runs makes it decode pictures that begin mid-slice, which it reports as
 * "the first slice of the frame is not 0" and which shows as tearing.
 * Requiring alignment=au here makes h264parse do that framing for us. */
static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK, GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("video/x-h264, "
        "stream-format = (string) byte-stream, "
        "alignment = (string) au"));

static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC, GST_PAD_ALWAYS,
    GST_STATIC_CAPS (GST_VIDEO_CAPS_MAKE_WITH_FEATURES
        (GST_CAPS_FEATURE_MEMORY_DMABUF, "NV21") "; "
        GST_VIDEO_CAPS_MAKE ("NV21")));

/* ---------------------------------------------------------------- release */

typedef struct
{
  GstCedarZcDec *self;
  VideoPicture *pic;
} FrameRelease;

static void
on_buffer_released (gpointer data, GstMiniObject * obj)
{
  FrameRelease *fr = data;
  GstCedarZcDec *self = fr->self;

  g_mutex_lock (&self->lock);
  if (!self->shutting_down && self->dec)
    ReturnPicture (self->dec, fr->pic);
  self->outstanding--;
  g_mutex_unlock (&self->lock);

  gst_object_unref (self);
  g_free (fr);
}

/* ------------------------------------------------------------- lifecycle */

static gboolean
gst_cedar_zc_dec_start (GstVideoDecoder * decoder)
{
  GstCedarZcDec *self = GST_CEDAR_ZC_DEC (decoder);

  self->memops = MemAdapterGetOpsS ();
  if (!self->memops) {
    GST_ERROR_OBJECT (self, "MemAdapterGetOpsS returned NULL");
    return FALSE;
  }
  /* open() is the initialiser. CdcMemSetup is an unimplemented hook that
   * fails, and CdcMemOpen2 dereferences the VE-ops self pointer it is given,
   * so passing NULL faults *after* it has opened /dev/dma_heap/system - which
   * looks like a permissions problem but is not. */
  if (CdcMemOpen (self->memops) != 0) {
    GST_ERROR_OBJECT (self, "could not open the Cedar memory adapter");
    return FALSE;
  }

  AddVDPlugin ();               /* lives in libvideoengine.so */

  self->dmabuf_alloc = gst_dmabuf_allocator_new ();
  self->shutting_down = FALSE;
  self->outstanding = 0;
  self->configured = FALSE;

  GST_INFO_OBJECT (self, "started");
  return TRUE;
}

static gboolean
gst_cedar_zc_dec_stop (GstVideoDecoder * decoder)
{
  GstCedarZcDec *self = GST_CEDAR_ZC_DEC (decoder);

  g_mutex_lock (&self->lock);
  self->shutting_down = TRUE;
  if (self->outstanding > 0) {
    /* Downstream still holds surfaces. Their release callbacks will now skip
     * ReturnPicture, which leaks those pool entries - acceptable at teardown,
     * and far better than returning them to a destroyed decoder. */
    GST_WARNING_OBJECT (self, "%d buffer(s) still held downstream at stop",
        self->outstanding);
  }
  if (self->dec) {
    DestroyVideoDecoder (self->dec);
    self->dec = NULL;
  }
  g_mutex_unlock (&self->lock);

  if (self->memops) {
    CdcMemClose (self->memops);
    self->memops = NULL;
  }
  g_clear_object (&self->dmabuf_alloc);
  if (self->input_state) {
    gst_video_codec_state_unref (self->input_state);
    self->input_state = NULL;
  }
  self->configured = FALSE;

  GST_INFO_OBJECT (self, "stopped");
  return TRUE;
}

static gboolean
gst_cedar_zc_dec_set_format (GstVideoDecoder * decoder,
    GstVideoCodecState * state)
{
  GstCedarZcDec *self = GST_CEDAR_ZC_DEC (decoder);
  VideoStreamInfo si;
  VConfig vc;

  if (self->input_state)
    gst_video_codec_state_unref (self->input_state);
  self->input_state = gst_video_codec_state_ref (state);

  g_mutex_lock (&self->lock);
  if (self->dec) {
    DestroyVideoDecoder (self->dec);
    self->dec = NULL;
  }

  self->dec = CreateVideoDecoder ();
  if (!self->dec) {
    g_mutex_unlock (&self->lock);
    GST_ERROR_OBJECT (self, "CreateVideoDecoder failed");
    return FALSE;
  }

  memset (&si, 0, sizeof (si));
  si.eCodecFormat = VIDEO_CODEC_FORMAT_H264;
  si.bIsFramePackage = 0;       /* Annex-B byte stream */

  memset (&vc, 0, sizeof (vc));
  vc.memops = self->memops;
  vc.veOpsS = GetVeOpsS (VE_OPS_TYPE_AW);
  vc.eOutputPixelFormat = PIXEL_FORMAT_NV21;
  vc.nFrameBufferNum = 8;
  vc.bDispErrorFrame = 1;
  vc.nDisplayHoldingFrameBufferNum = 2;

  if (InitializeVideoDecoder (self->dec, &si, &vc) != 0) {
    DestroyVideoDecoder (self->dec);
    self->dec = NULL;
    g_mutex_unlock (&self->lock);
    GST_ERROR_OBJECT (self, "InitializeVideoDecoder failed");
    return FALSE;
  }
  g_mutex_unlock (&self->lock);

  self->configured = FALSE;     /* output state set once we see a picture */
  GST_INFO_OBJECT (self, "decoder configured for H.264");
  return TRUE;
}

/* Output caps are set from the first decoded picture rather than the sink
 * caps, because the decoder is authoritative about the padded geometry and
 * the crop it wants applied. */
static gboolean
configure_output (GstCedarZcDec * self, VideoPicture * pic)
{
  GstVideoDecoder *decoder = GST_VIDEO_DECODER (self);
  GstVideoCodecState *out;
  gint w, h;

  w = pic->nRightOffset ? pic->nRightOffset - pic->nLeftOffset : pic->nWidth;
  h = pic->nBottomOffset ? pic->nBottomOffset - pic->nTopOffset : pic->nHeight;

  out = gst_video_decoder_set_output_state (decoder, GST_VIDEO_FORMAT_NV21,
      w, h, self->input_state);
  if (!out)
    return FALSE;

  /* Offer the DMABuf caps feature, but only if downstream actually wants it.
   *
   * Not every dma-buf-capable sink negotiates the feature. kmssink in 1.18,
   * for instance, advertises plain video/x-raw and decides whether to import
   * a dma_buf by inspecting the buffer's memory at render time. Forcing the
   * feature unconditionally therefore fails to negotiate with exactly the
   * sinks this element exists to feed. Either way the buffers we emit are
   * dma_buf-backed, so falling back to plain caps costs no copies - it just
   * stops advertising something the peer does not ask for. */
  {
    GstPad *srcpad = GST_VIDEO_DECODER_SRC_PAD (decoder);
    GstCaps *dma_caps, *peer;
    gboolean use_dmabuf;

    dma_caps = gst_video_info_to_caps (&out->info);
    gst_caps_set_features (dma_caps, 0,
        gst_caps_features_new (GST_CAPS_FEATURE_MEMORY_DMABUF, NULL));

    peer = gst_pad_peer_query_caps (srcpad, dma_caps);
    use_dmabuf = (peer && !gst_caps_is_empty (peer));
    if (peer)
      gst_caps_unref (peer);

    gst_caps_replace (&out->caps, NULL);
    if (use_dmabuf) {
      out->caps = dma_caps;
    } else {
      gst_caps_unref (dma_caps);
      out->caps = gst_video_info_to_caps (&out->info);
    }
    GST_INFO_OBJECT (self, "negotiating %s caps",
        use_dmabuf ? "video/x-raw(memory:DMABuf)" : "video/x-raw");
  }
  gst_video_codec_state_unref (out);

  if (!gst_video_decoder_negotiate (decoder)) {
    GST_ERROR_OBJECT (self, "failed to negotiate %dx%d NV21 DMABuf", w, h);
    return FALSE;
  }

  self->width = w;
  self->height = h;
  self->configured = TRUE;
  GST_INFO_OBJECT (self, "output %dx%d NV21 (buffer %dx%d, stride %d)",
      w, h, pic->nWidth, pic->nHeight, pic->nLineStride);
  return TRUE;
}

static GstBuffer *
wrap_picture (GstCedarZcDec * self, VideoPicture * pic)
{
  GstBuffer *buf;
  GstMemory *mem;
  FrameRelease *fr;
  gsize offsets[GST_VIDEO_MAX_PLANES] = { 0, };
  gint strides[GST_VIDEO_MAX_PLANES] = { 0, };
  gsize size;
  int dup_fd;

  /* GstDmaBufMemory closes the fd it is handed, and Cedar needs its own for
   * recycling, so hand over a duplicate. */
  dup_fd = dup (pic->nBufFd);
  if (dup_fd < 0) {
    GST_ERROR_OBJECT (self, "dup of dma_buf fd failed");
    return NULL;
  }

  size = pic->nBufSize > 0 ? (gsize) pic->nBufSize :
      (gsize) pic->nLineStride * pic->nHeight * 3 / 2;

  mem = gst_dmabuf_allocator_alloc (self->dmabuf_alloc, dup_fd, size);
  if (!mem) {
    close (dup_fd);
    return NULL;
  }

  buf = gst_buffer_new ();
  gst_buffer_append_memory (buf, mem);

  /* Plane layout is taken from the decoder's own pointers rather than assumed
   * from an alignment rule: pData1 - pData0 is the chroma offset, which
   * already accounts for the padded height. */
  strides[0] = pic->nLineStride;
  strides[1] = pic->nLineStride;
  offsets[0] = 0;
  offsets[1] = (gsize) (pic->pData1 - pic->pData0);

  gst_buffer_add_video_meta_full (buf, GST_VIDEO_FRAME_FLAG_NONE,
      GST_VIDEO_FORMAT_NV21, self->width, self->height, 2, offsets, strides);

  /* Hand the surface back only when downstream is finished with it. */
  fr = g_new0 (FrameRelease, 1);
  fr->self = gst_object_ref (self);
  fr->pic = pic;

  g_mutex_lock (&self->lock);
  self->outstanding++;
  g_mutex_unlock (&self->lock);

  gst_mini_object_weak_ref (GST_MINI_OBJECT (buf), on_buffer_released, fr);
  return buf;
}

static GstFlowReturn
gst_cedar_zc_dec_handle_frame (GstVideoDecoder * decoder,
    GstVideoCodecFrame * frame)
{
  GstCedarZcDec *self = GST_CEDAR_ZC_DEC (decoder);
  GstMapInfo map;
  VideoPicture *pic = NULL;
  char *buf = NULL, *ring = NULL;
  int bufsz = 0, ringsz = 0;
  int rounds = 0;

  if (!gst_buffer_map (frame->input_buffer, &map, GST_MAP_READ)) {
    gst_video_codec_frame_unref (frame);
    return GST_FLOW_ERROR;
  }

  g_mutex_lock (&self->lock);
  if (!self->dec) {
    g_mutex_unlock (&self->lock);
    gst_buffer_unmap (frame->input_buffer, &map);
    gst_video_codec_frame_unref (frame);
    return GST_FLOW_FLUSHING;
  }

  if (RequestVideoStreamBuffer (self->dec, (int) map.size, &buf, &bufsz,
          &ring, &ringsz, 0) == 0 && buf
      && (gsize) (bufsz + ringsz) >= map.size) {
    VideoStreamDataInfo di;

    /* The stream buffer is a ring; a request may straddle its wrap point. */
    memcpy (buf, map.data, (gsize) bufsz);
    if (ringsz > 0 && map.size > (gsize) bufsz)
      memcpy (ring, map.data + bufsz, map.size - bufsz);

    memset (&di, 0, sizeof (di));
    di.pData = buf;
    di.nLength = (int) map.size;
    di.bIsFirstPart = 1;
    di.bIsLastPart = 1;
    di.nPts = GST_CLOCK_TIME_IS_VALID (frame->pts) ?
        (int64_t) (frame->pts / GST_USECOND) : -1;
    di.bValid = 1;
    SubmitVideoStreamData (self->dec, &di, 0);
  } else {
    GST_WARNING_OBJECT (self, "no room in the Cedar stream buffer; dropping");
  }

  /* Decode is not guaranteed to yield a picture for every access unit, so
   * give it a bounded number of turns before returning empty-handed. */
  while (rounds++ < 4 && !pic) {
    DecodeVideoStream (self->dec, 0, 0, 0, 0);
    pic = RequestPicture (self->dec, 0);
  }
  g_mutex_unlock (&self->lock);

  gst_buffer_unmap (frame->input_buffer, &map);

  if (!pic) {
    /* Reordering or startup latency: nothing to emit yet. */
    gst_video_codec_frame_unref (frame);
    return GST_FLOW_OK;
  }

  if (pic->bEnableAfbcFlag) {
    GST_ELEMENT_ERROR (self, STREAM, DECODE, (NULL),
        ("decoder produced an AFBC-compressed surface, which needs a DRM "
            "modifier this element does not negotiate"));
    g_mutex_lock (&self->lock);
    if (self->dec)
      ReturnPicture (self->dec, pic);
    g_mutex_unlock (&self->lock);
    gst_video_codec_frame_unref (frame);
    return GST_FLOW_ERROR;
  }

  if (!self->configured && !configure_output (self, pic)) {
    g_mutex_lock (&self->lock);
    if (self->dec)
      ReturnPicture (self->dec, pic);
    g_mutex_unlock (&self->lock);
    gst_video_codec_frame_unref (frame);
    return GST_FLOW_NOT_NEGOTIATED;
  }

  frame->output_buffer = wrap_picture (self, pic);
  if (!frame->output_buffer) {
    g_mutex_lock (&self->lock);
    if (self->dec)
      ReturnPicture (self->dec, pic);
    g_mutex_unlock (&self->lock);
    gst_video_codec_frame_unref (frame);
    return GST_FLOW_ERROR;
  }

  return gst_video_decoder_finish_frame (decoder, frame);
}

static gboolean
gst_cedar_zc_dec_flush (GstVideoDecoder * decoder)
{
  GstCedarZcDec *self = GST_CEDAR_ZC_DEC (decoder);

  g_mutex_lock (&self->lock);
  if (self->dec)
    ResetVideoDecoder (self->dec);
  g_mutex_unlock (&self->lock);
  return TRUE;
}

/* ------------------------------------------------------------------ boiler */

static void
gst_cedar_zc_dec_finalize (GObject * object)
{
  GstCedarZcDec *self = GST_CEDAR_ZC_DEC (object);

  g_mutex_clear (&self->lock);
  G_OBJECT_CLASS (gst_cedar_zc_dec_parent_class)->finalize (object);
}

static void
gst_cedar_zc_dec_init (GstCedarZcDec * self)
{
  g_mutex_init (&self->lock);
  gst_video_decoder_set_packetized (GST_VIDEO_DECODER (self), TRUE);
  gst_video_decoder_set_needs_format (GST_VIDEO_DECODER (self), TRUE);
}

static void
gst_cedar_zc_dec_class_init (GstCedarZcDecClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstVideoDecoderClass *dec_class = GST_VIDEO_DECODER_CLASS (klass);

  gobject_class->finalize = gst_cedar_zc_dec_finalize;

  gst_element_class_add_static_pad_template (element_class, &sink_template);
  gst_element_class_add_static_pad_template (element_class, &src_template);
  gst_element_class_set_static_metadata (element_class,
      "Cedar zero-copy H.264 decoder", "Codec/Decoder/Video/Hardware",
      "Hardware H.264 decoding on Allwinner A733 with DMA-BUF output, "
      "bypassing the vendor OMX layer",
      "https://github.com/skamagedon/a733-zero-copy");

  dec_class->start = gst_cedar_zc_dec_start;
  dec_class->stop = gst_cedar_zc_dec_stop;
  dec_class->set_format = gst_cedar_zc_dec_set_format;
  dec_class->handle_frame = gst_cedar_zc_dec_handle_frame;
  dec_class->flush = gst_cedar_zc_dec_flush;
}

static gboolean
plugin_init (GstPlugin * plugin)
{
  GST_DEBUG_CATEGORY_INIT (cedarzcdec_debug, "cedarzcdec", 0,
      "Cedar zero-copy decoder");
  /* Rank PRIMARY+1 so this is chosen ahead of the vendor omxh264dec, whose
   * DMA-BUF negotiation is the thing this exists to replace. */
  return gst_element_register (plugin, "cedarzcdec", GST_RANK_PRIMARY + 1,
      GST_TYPE_CEDAR_ZC_DEC);
}

#ifndef PACKAGE
#define PACKAGE "cedarzc"
#endif
#ifndef VERSION
#define VERSION "0.1.0"
#endif

GST_PLUGIN_DEFINE (GST_VERSION_MAJOR, GST_VERSION_MINOR,
    cedarzc, "Zero-copy Cedar hardware decoding for Allwinner A733",
    plugin_init, VERSION, "LGPL", "a733-zero-copy",
    "https://github.com/skamagedon/a733-zero-copy")
