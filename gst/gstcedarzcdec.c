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
 * Output format is negotiated rather than hardcoded. Cedar honours a request
 * for NV12 or NV21 identically - same stride, same padded height, same
 * zero-copy import - so the element asks downstream which it wants and
 * configures the decoder accordingly, preferring NV12 when both are accepted.
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

/* How long to wait for room in the Cedar stream buffer before giving up on
 * an access unit.
 *
 * Waiting here IS the backpressure. handle_frame runs on the streaming
 * thread, so blocking in it is what tells upstream to slow down, which is
 * the whole mechanism a pull-rate source like filesrc relies on to not
 * outrun a hardware decoder.
 *
 * Bounded rather than indefinite so that a genuinely wedged decoder fails
 * as a stuttering stream instead of a hung pipeline. A second is far longer
 * than any legitimate decode and short enough that a live source recovers.
 */
#define SUBMIT_WAIT_US (1 * G_USEC_PER_SEC)

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

  /* Access units that could not be placed even after waiting. Counted
   * rather than merely logged, because losing one silently is how a
   * stream ends up decoding a fraction of its frames while every element
   * in the pipeline reports success. */
  guint64 dropped_aus;

  /* Output format chosen at set_format time and confirmed against what the
   * decoder actually delivers. Cedar honours the request for NV12, NV21 and
   * YV12 alike - same stride, same padded height, same zero-copy import - so
   * the choice is downstream's to make, not ours to hardcode. */
  int cedar_format;                 /* PIXEL_FORMAT_* handed to VConfig */
  GstVideoFormat gst_format;        /* matching GStreamer format */
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
        (GST_CAPS_FEATURE_MEMORY_DMABUF, "{ NV12, NV21 }") "; "
        GST_VIDEO_CAPS_MAKE ("{ NV12, NV21 }")));

/* ----------------------------------------------------------------- format */

static int
gst_to_cedar_format (GstVideoFormat f)
{
  switch (f) {
    case GST_VIDEO_FORMAT_NV12: return PIXEL_FORMAT_NV12;
    case GST_VIDEO_FORMAT_NV21: return PIXEL_FORMAT_NV21;
    default:                    return -1;
  }
}

static GstVideoFormat
cedar_to_gst_format (int f)
{
  switch (f) {
    case PIXEL_FORMAT_NV12: return GST_VIDEO_FORMAT_NV12;
    case PIXEL_FORMAT_NV21: return GST_VIDEO_FORMAT_NV21;
    default:                return GST_VIDEO_FORMAT_UNKNOWN;
  }
}

/* Ask downstream which of our formats it wants, before the decoder is created,
 * because eOutputPixelFormat has to be set at InitializeVideoDecoder time.
 *
 * NV12 is preferred when the peer accepts both: it is the more widely expected
 * format, and on this hardware it costs exactly the same as NV21. NV21 remains
 * the fallback because it is the combination this element was first proven
 * against. */
static GstVideoFormat
preferred_downstream_format (GstCedarZcDec * self)
{
  GstPad *srcpad = GST_VIDEO_DECODER_SRC_PAD (GST_VIDEO_DECODER (self));
  const GstVideoFormat order[] = { GST_VIDEO_FORMAT_NV12, GST_VIDEO_FORMAT_NV21 };
  GstCaps *peer;
  guint i;

  peer = gst_pad_peer_query_caps (srcpad, NULL);
  if (!peer || gst_caps_is_any (peer) || gst_caps_is_empty (peer)) {
    if (peer)
      gst_caps_unref (peer);
    return GST_VIDEO_FORMAT_NV12;
  }

  for (i = 0; i < G_N_ELEMENTS (order); i++) {
    const gchar *name = gst_video_format_to_string (order[i]);
    GstCaps *probe = gst_caps_new_simple ("video/x-raw",
        "format", G_TYPE_STRING, name, NULL);
    gboolean ok = gst_caps_can_intersect (peer, probe);

    if (!ok) {
      /* Try again carrying the DMABuf feature, since a sink may only list its
       * formats under that feature. */
      gst_caps_set_features (probe, 0,
          gst_caps_features_new (GST_CAPS_FEATURE_MEMORY_DMABUF, NULL));
      ok = gst_caps_can_intersect (peer, probe);
    }
    gst_caps_unref (probe);

    if (ok) {
      gst_caps_unref (peer);
      return order[i];
    }
  }

  gst_caps_unref (peer);
  return GST_VIDEO_FORMAT_NV12;
}

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

  if (self->dropped_aus)
    GST_ELEMENT_WARNING (self, STREAM, DECODE, (NULL),
        ("%" G_GUINT64_FORMAT " access unit(s) were dropped for want of room "
            "in the stream buffer; the decoded output is incomplete",
            self->dropped_aus));

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
  self->gst_format = preferred_downstream_format (self);
  self->cedar_format = gst_to_cedar_format (self->gst_format);
  if (self->cedar_format < 0) {
    self->gst_format = GST_VIDEO_FORMAT_NV12;
    self->cedar_format = PIXEL_FORMAT_NV12;
  }
  vc.eOutputPixelFormat = self->cedar_format;
  vc.nFrameBufferNum = 8;
  vc.bDispErrorFrame = 1;
  vc.nDisplayHoldingFrameBufferNum = 2;
  /* Slack for the decoder to work ahead in. Leaving this at zero deadlocks
   * the decoder outright on some perfectly ordinary streams: it decodes about
   * a dozen frames, then every DecodeVideoStream returns NO_FRAME_BUFFER
   * forever, whatever is downstream and however large nFrameBufferNum is.
   *
   * Measured on Big_Buck_Bunny_1080_10s_30MB.mp4, decoder straight to
   * fakesink so nothing held a surface:
   *
   *   nDecodeSmoothFrameBufferNum = 0    13 frames decoded, 744 NO_FRAME_BUFFER
   *   nDecodeSmoothFrameBufferNum = 3   283 frames decoded,   0 NO_FRAME_BUFFER
   *
   * Raising nFrameBufferNum does nothing for it; 4, 8, 16, 24 and 32 all stall
   * at the same 13 frames. It is this field specifically, and the vendor
   * library says so on every single init:
   *
   *   warning: the nDecodeSmoothFrameBufferNum is 0
   *
   * which is worth reading as an error rather than noise. */
  vc.nDecodeSmoothFrameBufferNum = 3;

  if (InitializeVideoDecoder (self->dec, &si, &vc) != 0) {
    DestroyVideoDecoder (self->dec);
    self->dec = NULL;
    g_mutex_unlock (&self->lock);
    GST_ERROR_OBJECT (self, "InitializeVideoDecoder failed");
    return FALSE;
  }
  g_mutex_unlock (&self->lock);

  self->configured = FALSE;     /* output state set once we see a picture */
  GST_INFO_OBJECT (self, "decoder configured for H.264, requesting %s output",
      gst_video_format_to_string (self->gst_format));
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
  GstVideoFormat delivered;
  gint w, h;

  w = pic->nRightOffset ? pic->nRightOffset - pic->nLeftOffset : pic->nWidth;
  h = pic->nBottomOffset ? pic->nBottomOffset - pic->nTopOffset : pic->nHeight;

  /* The delivered format is authoritative. We ask for one, and the decoder has
   * honoured that on this hardware, but reporting what actually arrived means a
   * silent override shows up as a caps mismatch rather than as wrong colours. */
  delivered = cedar_to_gst_format (pic->ePixelFormat);
  if (delivered == GST_VIDEO_FORMAT_UNKNOWN) {
    GST_ERROR_OBJECT (self,
        "decoder delivered ePixelFormat %d, which this element cannot map",
        pic->ePixelFormat);
    return FALSE;
  }
  if (delivered != self->gst_format) {
    GST_WARNING_OBJECT (self, "asked for %s but the decoder delivered %s",
        gst_video_format_to_string (self->gst_format),
        gst_video_format_to_string (delivered));
    self->gst_format = delivered;
  }

  out = gst_video_decoder_set_output_state (decoder, delivered,
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
  GST_INFO_OBJECT (self, "output %dx%d %s (buffer %dx%d, stride %d)",
      w, h, gst_video_format_to_string (delivered),
      pic->nWidth, pic->nHeight, pic->nLineStride);
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
      self->gst_format, self->width, self->height, 2, offsets, strides);

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

static void
return_picture (GstCedarZcDec * self, VideoPicture * pic)
{
  g_mutex_lock (&self->lock);
  if (self->dec)
    ReturnPicture (self->dec, pic);
  g_mutex_unlock (&self->lock);
}

/* Hand one decoded picture downstream, matched to the OLDEST frame still
 * waiting for output.
 *
 * The oldest frame is the right one; the frame currently going in is not. A
 * hardware decoder runs several frames behind its input, so the picture that
 * emerges while access unit N is submitted belongs to an earlier access unit.
 * Attaching it to N stamped every buffer a few frames late, and, worse,
 * gst_video_decoder_finish_frame() releases every frame OLDER than the one it
 * is handed. So each output quietly discarded the frames the decoder was still
 * working on, and the shortfall grew with the decoder's own latency. That is
 * where the missing frames went: 577 of 600 on one clip, 270 of 300 on
 * another, with nothing anywhere reporting a loss. */
static GstFlowReturn
emit_picture (GstCedarZcDec * self, GstVideoDecoder * decoder,
    VideoPicture * pic)
{
  GstVideoCodecFrame *out;
  GstBuffer *buf;

  if (pic->bEnableAfbcFlag) {
    GST_ELEMENT_ERROR (self, STREAM, DECODE, (NULL),
        ("decoder produced an AFBC-compressed surface, which needs a DRM "
            "modifier this element does not negotiate"));
    return_picture (self, pic);
    return GST_FLOW_ERROR;
  }

  if (!self->configured && !configure_output (self, pic)) {
    return_picture (self, pic);
    return GST_FLOW_NOT_NEGOTIATED;
  }

  buf = wrap_picture (self, pic);
  if (!buf) {
    return_picture (self, pic);
    return GST_FLOW_ERROR;
  }

  out = gst_video_decoder_get_oldest_frame (decoder);
  if (!out) {
    /* Nothing pending to carry it. Push it rather than drop it. */
    return gst_pad_push (GST_VIDEO_DECODER_SRC_PAD (decoder), buf);
  }

  out->output_buffer = buf;
  return gst_video_decoder_finish_frame (decoder, out);
}

/* Pull out everything still inside the decoder at end of stream.
 *
 * There was no drain at all before, so the pictures the decoder was holding
 * when the input ran out were simply lost. On a ten second clip that is a
 * visible piece of the ending, not a rounding error. */
static GstFlowReturn
gst_cedar_zc_dec_finish (GstVideoDecoder * decoder)
{
  GstCedarZcDec *self = GST_CEDAR_ZC_DEC (decoder);
  GstFlowReturn ret = GST_FLOW_OK;
  int empty = 0;

  while (empty < 8 && ret == GST_FLOW_OK) {
    VideoPicture *pic = NULL;

    g_mutex_lock (&self->lock);
    if (self->dec) {
      DecodeVideoStream (self->dec, 1 /* end of stream */, 0, 0, 0);
      pic = RequestPicture (self->dec, 0);
    }
    g_mutex_unlock (&self->lock);

    if (!pic) {
      empty++;
      continue;
    }
    empty = 0;
    ret = emit_picture (self, decoder, pic);
  }

  GST_INFO_OBJECT (self, "drained at end of stream");
  return ret;
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
  gboolean submitted = FALSE;
  gint64 deadline;

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

  /* Get this access unit into the stream buffer, waiting for room rather than
   * throwing it away.
   *
   * This used to drop on a full buffer and return GST_FLOW_OK, which is a
   * silent, unbounded loss of coded data: the pipeline reports success, every
   * element's stats look healthy, and the stream quietly decodes a fraction of
   * its frames because the dropped units carried references the rest depended
   * on. It never showed up on a live camera, where data arrives at the rate it
   * was captured, and it is immediate on anything that reads as fast as the
   * disk will go.
   *
   * Decoding is what empties the buffer, so decode; and surfaces come back
   * from downstream on another thread, so also leave the lock long enough for
   * that to happen. */
  deadline = g_get_monotonic_time () + SUBMIT_WAIT_US;
  for (;;) {
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
      submitted = TRUE;
      break;
    }

    DecodeVideoStream (self->dec, 0, 0, 0, 0);

    if (g_get_monotonic_time () >= deadline)
      break;

    g_mutex_unlock (&self->lock);
    g_usleep (1000);
    g_mutex_lock (&self->lock);
    if (!self->dec) {
      g_mutex_unlock (&self->lock);
      gst_buffer_unmap (frame->input_buffer, &map);
      gst_video_codec_frame_unref (frame);
      return GST_FLOW_FLUSHING;
    }
  }

  if (!submitted)
    self->dropped_aus++;

  /* Decode is not guaranteed to yield a picture for every access unit, so
   * give it a bounded number of turns before returning empty-handed. */
  while (rounds++ < 4 && !pic) {
    DecodeVideoStream (self->dec, 0, 0, 0, 0);
    pic = RequestPicture (self->dec, 0);
  }
  g_mutex_unlock (&self->lock);

  gst_buffer_unmap (frame->input_buffer, &map);

  if (!submitted) {
    /* On the bus for the first one, so it is visible without anybody having
     * thought to set GST_DEBUG beforehand. That it was only ever a
     * GST_WARNING is why this went unnoticed: at gst-launch's default debug
     * level the message does not print at all, so the failure looked like
     * nothing more than a short output file. */
    if (self->dropped_aus == 1)
      GST_ELEMENT_WARNING (self, STREAM, DECODE, (NULL),
          ("no room in the Cedar stream buffer after waiting %d ms; "
              "dropping coded data, so the output will be missing frames. "
              "The stream is arriving faster than this decoder can consume "
              "it and something upstream is not honouring backpressure.",
              (int) (SUBMIT_WAIT_US / 1000)));
    else
      GST_WARNING_OBJECT (self, "dropped access unit (%" G_GUINT64_FORMAT
          " so far)", self->dropped_aus);
  }

  /* This frame's ref is done with. Any picture the decoder produces is matched
   * to the oldest pending frame inside emit_picture, not to this one. */
  gst_video_codec_frame_unref (frame);

  if (!pic)
    return GST_FLOW_OK;         /* reordering or startup latency */

  return emit_picture (self, decoder, pic);
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
  dec_class->finish = gst_cedar_zc_dec_finish;
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
