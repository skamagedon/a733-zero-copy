/*
 * gstcedarzcenc.c - hardware H.264 encoder element for Allwinner A733
 *
 * The counterpart to cedarzcdec. The A733 silicon encodes fine - measured at
 * ~203 fps for 720p and ~98 fps for 1088p - but nothing exposed it to
 * GStreamer, so the only usable encoder on this board was x264, which manages
 * about 16 fps at 720p. That gap is what this element closes.
 *
 *   gst-launch-1.0 videotestsrc ! video/x-raw,format=NV12,width=1920,height=1080 \
 *       ! cedarzcenc bitrate=3000 ! h264parse ! flvmux ! rtmp2sink location=...
 *
 * Three A733 quirks are handled here, all documented in the project README:
 *
 * 1. Encode height must be a multiple of 16. A non-aligned height never
 *    completes a frame and the driver reports "wait interrupt overtime", which
 *    reads like a hardware fault and is not one. We encode at the aligned
 *    height and signal the real height through SPS frame cropping, so callers
 *    can hand us 1080 and get correct 1080 out.
 *
 * 2. Output is AVCC (4-byte big-endian length prefixes), not Annex-B, and
 *    bEncH264Nalu does not change that. We rewrite the prefixes to start codes.
 *
 * 3. The parameter sets cannot be retrieved. VENC_IndexParamH264SPSPPS returns
 *    a correctly shaped avcC record whose SPS and PPS payloads are 0xff filler,
 *    because nothing generates them. We synthesize both from the encoder's own
 *    fixed configuration. There is also no inline-emission option, so we
 *    prepend them ourselves ahead of every IDR - which is what a live RTMP or
 *    HLS consumer needs anyway.
 *
 * The encoder device node is root-only by default. Install the udev rule from
 * the README (KERNEL=="cedar_dev_ve2", GROUP="video", MODE="0660") and be in
 * the video group, or VideoEncCreate fails long before any encoding happens.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/video/gstvideoencoder.h>

#include <string.h>

#include "vencoder.h"
#include "memoryAdapter.h"

#define VE_OPS_TYPE_AW 0
extern VeOpsS *GetVeOpsS (int type);

GST_DEBUG_CATEGORY_STATIC (cedarzcenc_debug);
#define GST_CAT_DEFAULT cedarzcenc_debug

#define DEFAULT_BITRATE   4000    /* kbps */
#define DEFAULT_GOP       60      /* frames between IDRs */
#define DEFAULT_QP_MIN    10
#define DEFAULT_QP_MAX    45

enum
{
  PROP_0,
  PROP_BITRATE,
  PROP_GOP,
  PROP_QP_MIN,
  PROP_QP_MAX,
};

#define GST_TYPE_CEDAR_ZC_ENC (gst_cedar_zc_enc_get_type ())
G_DECLARE_FINAL_TYPE (GstCedarZcEnc, gst_cedar_zc_enc, GST, CEDAR_ZC_ENC,
    GstVideoEncoder)

struct _GstCedarZcEnc
{
  GstVideoEncoder parent;

  VideoEncoder *enc;
  struct ScMemOpsS *memops;
  GstVideoCodecState *input_state;

  gboolean configured;
  gint width;                   /* real, as negotiated */
  gint height;                  /* real, as negotiated */
  gint enc_height;              /* 16-aligned, what the hardware encodes */

  /* Synthesized once per configuration; prepended ahead of every IDR because
   * the encoder has no inline-emission option. */
  guint8 hdr[64];
  gsize hdr_len;

  guint bitrate;
  guint gop;
  guint qp_min;
  guint qp_max;
};

G_DEFINE_TYPE (GstCedarZcEnc, gst_cedar_zc_enc, GST_TYPE_VIDEO_ENCODER);

/* Cedar wants NV12 (VENC_PIXEL_YUV420SP). Anything else would need a CPU
 * convert, which defeats the point of using the hardware at all. */
static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK, GST_PAD_ALWAYS,
    GST_STATIC_CAPS (GST_VIDEO_CAPS_MAKE ("NV12")));

static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC, GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("video/x-h264, "
        "stream-format = (string) byte-stream, "
        "alignment = (string) au"));

/* --------------------------------------------------- parameter-set builder
 *
 * Lifted from tools/cedar-h264-encode-probe.c, where these values were pinned
 * by parsing 30 consecutive slice headers out of the encoder's own output and
 * confirmed by decoding at ~46 dB PSNR. They describe the encoder's default
 * configuration; changing profile or level means re-deriving them.
 */
struct bw
{
  guint8 b[64];
  int n;
};

static void
bw_u (struct bw *s, int n, guint v)
{
  int i;
  for (i = n - 1; i >= 0; i--) {
    if ((v >> i) & 1)
      s->b[s->n >> 3] |= 0x80 >> (s->n & 7);
    s->n++;
  }
}

static void
bw_ue (struct bw *s, guint v)
{
  guint t = ++v;
  int n = 0;
  while (t >>= 1)
    n++;
  bw_u (s, n, 0);
  bw_u (s, n + 1, v);
}

static void
bw_se (struct bw *s, int v)
{
  bw_ue (s, v > 0 ? (guint) (2 * v - 1) : (guint) (-2 * v));
}

static int
bw_finish (struct bw *s)
{
  bw_u (s, 1, 1);
  while (s->n & 7)
    bw_u (s, 1, 0);
  return s->n >> 3;
}

/* w/h are the REAL dimensions. The macroblock count rounds up to the aligned
 * height we actually encode, and the difference is emitted as frame cropping,
 * which is how a caller gets to ask for 1080 on hardware that needs 1088. */
static int
build_sps (struct bw *s, int w, int h, int profile, int level)
{
  int mbw = (w + 15) / 16, mbh = (h + 15) / 16;
  int cropr = (mbw * 16 - w) / 2;
  int cropb = (mbh * 16 - h) / 2;

  memset (s, 0, sizeof (*s));
  bw_u (s, 8, 0x67);
  bw_u (s, 8, (guint) profile);
  bw_u (s, 8, 0);
  bw_u (s, 8, (guint) level);
  bw_ue (s, 0);                 /* seq_parameter_set_id */
  bw_ue (s, 1);                 /* chroma_format_idc: 4:2:0 */
  bw_ue (s, 0);                 /* bit_depth_luma_minus8 */
  bw_ue (s, 0);                 /* bit_depth_chroma_minus8 */
  bw_u (s, 1, 0);               /* qpprime_y_zero_transform_bypass */
  bw_u (s, 1, 0);               /* seq_scaling_matrix_present */
  bw_ue (s, 8);                 /* log2_max_frame_num_minus4 -> 12 */
  bw_ue (s, 0);                 /* pic_order_cnt_type */
  bw_ue (s, 8);                 /* log2_max_poc_lsb_minus4 -> 12 */
  bw_ue (s, 2);                 /* max_num_ref_frames */
  bw_u (s, 1, 1);               /* gaps_in_frame_num_value_allowed */
  bw_ue (s, (guint) (mbw - 1));
  bw_ue (s, (guint) (mbh - 1));
  bw_u (s, 1, 1);               /* frame_mbs_only_flag */
  bw_u (s, 1, 1);               /* direct_8x8_inference_flag */
  bw_u (s, 1, (cropr || cropb) ? 1 : 0);
  if (cropr || cropb) {
    bw_ue (s, 0);
    bw_ue (s, (guint) cropr);
    bw_ue (s, 0);
    bw_ue (s, (guint) cropb);
  }
  bw_u (s, 1, 0);               /* vui_parameters_present_flag */
  return bw_finish (s);
}

static int
build_pps (struct bw *s)
{
  memset (s, 0, sizeof (*s));
  bw_u (s, 8, 0x68);
  bw_ue (s, 0);                 /* pic_parameter_set_id */
  bw_ue (s, 0);                 /* seq_parameter_set_id */
  bw_u (s, 1, 1);               /* entropy_coding_mode_flag: CABAC */
  bw_u (s, 1, 0);               /* bottom_field_pic_order_present */
  bw_ue (s, 0);                 /* num_slice_groups_minus1 */
  bw_ue (s, 0);                 /* num_ref_idx_l0_default_active_minus1 */
  bw_ue (s, 0);                 /* num_ref_idx_l1_default_active_minus1 */
  bw_u (s, 1, 0);               /* weighted_pred_flag */
  bw_u (s, 2, 0);               /* weighted_bipred_idc */
  bw_se (s, 0);                 /* pic_init_qp_minus26 */
  bw_se (s, 0);                 /* pic_init_qs_minus26 */
  bw_se (s, 0);                 /* chroma_qp_index_offset */
  bw_u (s, 1, 1);               /* deblocking_filter_control_present */
  bw_u (s, 1, 0);               /* constrained_intra_pred_flag */
  bw_u (s, 1, 0);               /* redundant_pic_cnt_present_flag */
  return bw_finish (s);
}

/* Walks the record rather than scanning it. The length fields are real data,
 * so a flat "is every byte 0xff" test wrongly reports a genuine record and
 * then takes the parse path. Only the payloads may be examined. */
static gboolean
avcc_is_filler (const guint8 * d, guint len)
{
  guint off = 5, i, k, count;

  if (len < 7 || d[0] != 1)
    return TRUE;

  count = d[off++] & 0x1f;
  for (i = 0; i < count; i++) {
    guint n;
    if (off + 2 > len)
      return TRUE;
    n = ((guint) d[off] << 8) | d[off + 1];
    off += 2;
    if (n == 0 || off + n > len)
      return TRUE;
    for (k = 0; k < n; k++)
      if (d[off + k] != 0xff)
        return FALSE;
    off += n;
  }

  if (off >= len)
    return TRUE;
  count = d[off++];
  for (i = 0; i < count; i++) {
    guint n;
    if (off + 2 > len)
      return TRUE;
    n = ((guint) d[off] << 8) | d[off + 1];
    off += 2;
    if (n == 0 || off + n > len)
      return TRUE;
    for (k = 0; k < n; k++)
      if (d[off + k] != 0xff)
        return FALSE;
    off += n;
  }
  return TRUE;
}

/* Fill self->hdr with Annex-B SPS+PPS, either parsed from a genuine avcC
 * record or synthesized when it is filler (the A733 case). */
static void
build_header (GstCedarZcEnc * self)
{
  static const guint8 sc[4] = { 0, 0, 0, 1 };
  VencHeaderData h;
  int profile = 100, level = 51;
  struct bw s;
  gsize o = 0;
  int n;

  memset (&h, 0, sizeof (h));
  self->hdr_len = 0;

  if (VideoEncGetParameter (self->enc, VENC_IndexParamH264SPSPPS, &h) == 0
      && h.pBuffer && h.nLength >= 4) {
    if (!avcc_is_filler (h.pBuffer, h.nLength)) {
      /* A future firmware might actually populate it. Take it if so. */
      guint off = 5, i, count;
      count = h.pBuffer[off++] & 0x1f;
      for (i = 0; i < count && off + 2 <= h.nLength; i++) {
        guint ln = ((guint) h.pBuffer[off] << 8) | h.pBuffer[off + 1];
        off += 2;
        if (off + ln > h.nLength || o + 4 + ln > sizeof (self->hdr))
          break;
        memcpy (self->hdr + o, sc, 4);
        o += 4;
        memcpy (self->hdr + o, h.pBuffer + off, ln);
        o += ln;
        off += ln;
      }
      if (off < h.nLength) {
        count = h.pBuffer[off++];
        for (i = 0; i < count && off + 2 <= h.nLength; i++) {
          guint ln = ((guint) h.pBuffer[off] << 8) | h.pBuffer[off + 1];
          off += 2;
          if (off + ln > h.nLength || o + 4 + ln > sizeof (self->hdr))
            break;
          memcpy (self->hdr + o, sc, 4);
          o += 4;
          memcpy (self->hdr + o, h.pBuffer + off, ln);
          o += ln;
          off += ln;
        }
      }
      self->hdr_len = o;
      GST_INFO_OBJECT (self, "parameter sets came from the encoder (%u bytes)",
          (guint) o);
      return;
    }
    profile = h.pBuffer[1];
    level = h.pBuffer[3];
  }

  n = build_sps (&s, self->width, self->height, profile, level);
  memcpy (self->hdr + o, sc, 4);
  o += 4;
  memcpy (self->hdr + o, s.b, (gsize) n);
  o += n;

  n = build_pps (&s);
  memcpy (self->hdr + o, sc, 4);
  o += 4;
  memcpy (self->hdr + o, s.b, (gsize) n);
  o += n;

  self->hdr_len = o;
  GST_INFO_OBJECT (self,
      "parameter sets are 0xff filler; synthesized SPS/PPS from profile %d "
      "level %d (%u bytes, crop for %dx%d in %dx%d)", profile, level,
      (guint) o, self->width, self->height, self->width, self->enc_height);
}

/* ------------------------------------------------------------- lifecycle */

static gboolean
gst_cedar_zc_enc_start (GstVideoEncoder * encoder)
{
  GstCedarZcEnc *self = GST_CEDAR_ZC_ENC (encoder);

  self->memops = MemAdapterGetOpsS ();
  if (!self->memops) {
    GST_ERROR_OBJECT (self, "MemAdapterGetOpsS returned NULL");
    return FALSE;
  }
  /* CdcMemOpen, not CdcMemSetup: the latter is an unimplemented hook. */
  if (CdcMemOpen (self->memops) != 0) {
    GST_ERROR_OBJECT (self, "could not open the Cedar memory adapter");
    return FALSE;
  }
  self->configured = FALSE;
  self->hdr_len = 0;
  GST_INFO_OBJECT (self, "started");
  return TRUE;
}

static void
teardown_encoder (GstCedarZcEnc * self)
{
  if (self->enc) {
    ReleaseAllocInputBuffer (self->enc);
    VideoEncUnInit (self->enc);
    VideoEncDestroy (self->enc);
    self->enc = NULL;
  }
  self->configured = FALSE;
}

static gboolean
gst_cedar_zc_enc_stop (GstVideoEncoder * encoder)
{
  GstCedarZcEnc *self = GST_CEDAR_ZC_ENC (encoder);

  teardown_encoder (self);
  if (self->memops) {
    CdcMemClose (self->memops);
    self->memops = NULL;
  }
  if (self->input_state) {
    gst_video_codec_state_unref (self->input_state);
    self->input_state = NULL;
  }
  GST_INFO_OBJECT (self, "stopped");
  return TRUE;
}

static gboolean
gst_cedar_zc_enc_set_format (GstVideoEncoder * encoder,
    GstVideoCodecState * state)
{
  GstCedarZcEnc *self = GST_CEDAR_ZC_ENC (encoder);
  GstVideoCodecState *out;
  VencBaseConfig cfg;
  VencAllocateBufferParam bufparam;
  VencH264Param h264;
  int fps;

  teardown_encoder (self);

  fps = GST_VIDEO_INFO_FPS_N (&state->info) > 0 ?
      GST_VIDEO_INFO_FPS_N (&state->info) /
      MAX (1, GST_VIDEO_INFO_FPS_D (&state->info)) : 30;

  if (self->input_state)
    gst_video_codec_state_unref (self->input_state);
  self->input_state = gst_video_codec_state_ref (state);

  self->width = GST_VIDEO_INFO_WIDTH (&state->info);
  self->height = GST_VIDEO_INFO_HEIGHT (&state->info);
  /* Quirk 1. Round up; the difference becomes SPS frame cropping. */
  self->enc_height = (self->height + 15) & ~15;
  if (self->enc_height != self->height)
    GST_INFO_OBJECT (self, "height %d is not a multiple of 16; encoding %d "
        "and cropping %d lines in the SPS", self->height, self->enc_height,
        self->enc_height - self->height);

  self->enc = VideoEncCreate (VENC_CODEC_H264);
  if (!self->enc) {
    GST_ELEMENT_ERROR (self, RESOURCE, OPEN_READ_WRITE,
        ("VideoEncCreate failed"),
        ("/dev/cedar_dev_ve2 is root-only by default. Install the udev rule "
            "KERNEL==\"cedar_dev_ve2\", GROUP=\"video\", MODE=\"0660\" and "
            "join the video group."));
    return FALSE;
  }

  memset (&cfg, 0, sizeof (cfg));
  cfg.nInputWidth = self->width;
  cfg.nInputHeight = self->enc_height;
  cfg.nDstWidth = self->width;
  cfg.nDstHeight = self->enc_height;
  cfg.nStride = self->width;
  cfg.eInputFormat = VENC_PIXEL_YUV420SP;       /* NV12 */
  cfg.memops = self->memops;
  cfg.veOpsS = GetVeOpsS (VE_OPS_TYPE_AW);
  cfg.pVeOpsSelf = NULL;
  cfg.bEncH264Nalu = 1;         /* does not actually give Annex-B; see quirk 2 */

  if (VideoEncInit (self->enc, &cfg) != 0) {
    GST_ELEMENT_ERROR (self, LIBRARY, INIT, ("VideoEncInit failed"),
        ("%dx%d (encode height %d)", self->width, self->height,
            self->enc_height));
    teardown_encoder (self);
    return FALSE;
  }

  /* Rate control. The A733 exposes the full set - CBR, VBR, AVBR, QPMAP,
   * FixQP - so we drive real CBR rather than constant quality, which is what
   * a fixed-bitrate uplink needs. */
  memset (&h264, 0, sizeof (h264));
  /* Profile and level must agree with what build_sps() writes, because the
   * parameter sets are synthesized rather than read back from the encoder. */
  h264.sProfileLevel.nProfile = VENC_H264ProfileHigh;
  h264.sProfileLevel.nLevel = VENC_H264Level51;
  /* Must be 1. build_pps() hardcodes entropy_coding_mode_flag = 1, and a PPS
   * claiming CABAC over CAVLC slice data does not decode at all. */
  h264.bEntropyCodingCABAC = 1;
  h264.nFramerate = fps;
  h264.nSrcFramerate = fps;
  h264.nBitrate = (int) (self->bitrate * 1000);
  h264.nMaxKeyInterval = (int) self->gop;
  h264.nCodingMode = VENC_FRAME_CODING;
  h264.sQPRange.nMinqp = (int) self->qp_min;
  h264.sQPRange.nMaxqp = (int) self->qp_max;
  h264.sQPRange.nMinPqp = (int) self->qp_min;
  h264.sQPRange.nMaxPqp = (int) self->qp_max;
  /* Real CBR, not constant quality: a fixed uplink cannot absorb VBR spikes.
   * eRcMode lives in the nested sRcParam, not on VencH264Param itself. */
  h264.sRcParam.eRcMode = AW_CBR;
  if (VideoEncSetParameter (self->enc, VENC_IndexParamH264Param, &h264) != 0)
    GST_WARNING_OBJECT (self, "VENC_IndexParamH264Param was rejected");

  memset (&bufparam, 0, sizeof (bufparam));
  bufparam.nBufferNum = 4;
  bufparam.nSizeY = self->width * self->enc_height;
  bufparam.nSizeC = self->width * self->enc_height / 2;
  if (AllocInputBuffer (self->enc, &bufparam) != 0) {
    GST_ELEMENT_ERROR (self, RESOURCE, NO_SPACE_LEFT,
        ("AllocInputBuffer failed"), (NULL));
    teardown_encoder (self);
    return FALSE;
  }

  build_header (self);

  out = gst_video_encoder_set_output_state (encoder,
      gst_caps_new_simple ("video/x-h264",
          "stream-format", G_TYPE_STRING, "byte-stream",
          "alignment", G_TYPE_STRING, "au", NULL), state);
  gst_video_codec_state_unref (out);

  self->configured = TRUE;
  GST_INFO_OBJECT (self, "configured %dx%d @ %d kbps, GOP %u",
      self->width, self->height, self->bitrate, self->gop);
  return gst_video_encoder_negotiate (encoder);
}

/* ------------------------------------------------------------ per frame */

/* Copies an NV12 frame into a Cedar input buffer, honouring the source stride,
 * which is rarely equal to the width. */
static void
copy_nv12 (GstCedarZcEnc * self, GstVideoFrame * vf, VencInputBuffer * in)
{
  const guint8 *sy = GST_VIDEO_FRAME_PLANE_DATA (vf, 0);
  const guint8 *sc = GST_VIDEO_FRAME_PLANE_DATA (vf, 1);
  gint sys = GST_VIDEO_FRAME_PLANE_STRIDE (vf, 0);
  gint scs = GST_VIDEO_FRAME_PLANE_STRIDE (vf, 1);
  guint8 *dy = (guint8 *) in->pAddrVirY;
  guint8 *dc = (guint8 *) in->pAddrVirC;
  gint w = self->width, h = self->height, i;

  for (i = 0; i < h; i++)
    memcpy (dy + (gsize) i * w, sy + (gsize) i * sys, w);
  /* Pad the alignment rows by repeating the last line rather than leaving them
   * uninitialised; garbage there costs bitrate for pixels nobody will see. */
  for (; i < self->enc_height; i++)
    memcpy (dy + (gsize) i * w, dy + (gsize) (h - 1) * w, w);

  for (i = 0; i < h / 2; i++)
    memcpy (dc + (gsize) i * w, sc + (gsize) i * scs, w);
  for (; i < self->enc_height / 2; i++)
    memcpy (dc + (gsize) i * w, dc + (gsize) (h / 2 - 1) * w, w);
}

/* AVCC (4-byte big-endian lengths) to Annex-B, in place of a copy loop that
 * would need a second buffer. Returns bytes written, and reports whether any
 * NAL was an IDR so the caller can mark the frame as a sync point. */
static gsize
avcc_to_annexb (const guint8 * src, gsize len, guint8 * dst, gboolean * is_idr)
{
  static const guint8 sc[4] = { 0, 0, 0, 1 };
  gsize off = 0, o = 0;

  *is_idr = FALSE;
  while (off + 4 <= len) {
    guint32 n = ((guint32) src[off] << 24) | ((guint32) src[off + 1] << 16) |
        ((guint32) src[off + 2] << 8) | (guint32) src[off + 3];
    off += 4;
    if (n == 0 || off + n > len)
      break;
    if ((src[off] & 0x1f) == 5)
      *is_idr = TRUE;
    memcpy (dst + o, sc, 4);
    o += 4;
    memcpy (dst + o, src + off, n);
    o += n;
    off += n;
  }
  return o;
}

static GstFlowReturn
gst_cedar_zc_enc_handle_frame (GstVideoEncoder * encoder,
    GstVideoCodecFrame * frame)
{
  GstCedarZcEnc *self = GST_CEDAR_ZC_ENC (encoder);
  GstVideoFrame vf;
  VencInputBuffer in;
  VencOutputBuffer ob;
  GstBuffer *outbuf;
  GstMapInfo map;
  gsize total, o;
  gboolean idr = FALSE;

  if (!self->configured) {
    gst_video_codec_frame_unref (frame);
    return GST_FLOW_NOT_NEGOTIATED;
  }

  if (!gst_video_frame_map (&vf, &self->input_state->info,
          frame->input_buffer, GST_MAP_READ)) {
    GST_ERROR_OBJECT (self, "could not map the input frame");
    gst_video_codec_frame_unref (frame);
    return GST_FLOW_ERROR;
  }

  memset (&in, 0, sizeof (in));
  if (GetOneAllocInputBuffer (self->enc, &in) != 0) {
    gst_video_frame_unmap (&vf);
    GST_ERROR_OBJECT (self, "GetOneAllocInputBuffer failed");
    gst_video_codec_frame_unref (frame);
    return GST_FLOW_ERROR;
  }

  copy_nv12 (self, &vf, &in);
  gst_video_frame_unmap (&vf);
  FlushCacheAllocInputBuffer (self->enc, &in);

  in.nPts = GST_CLOCK_TIME_IS_VALID (frame->pts) ?
      (long long) (frame->pts / GST_USECOND) : 0;

  if (AddOneInputBuffer (self->enc, &in) != 0) {
    GST_ERROR_OBJECT (self, "AddOneInputBuffer failed");
    gst_video_codec_frame_unref (frame);
    return GST_FLOW_ERROR;
  }

  if (VideoEncodeOneFrame (self->enc) != 0) {
    /* The classic symptom of a non-16-aligned height, which set_format should
     * have made impossible - so if it happens, say what it usually means. */
    GST_ELEMENT_ERROR (self, STREAM, ENCODE, ("VideoEncodeOneFrame failed"),
        ("usually a geometry problem: encode height %d, width %d",
            self->enc_height, self->width));
    AlreadyUsedInputBuffer (self->enc, &in);
    ReturnOneAllocInputBuffer (self->enc, &in);
    gst_video_codec_frame_unref (frame);
    return GST_FLOW_ERROR;
  }

  AlreadyUsedInputBuffer (self->enc, &in);
  ReturnOneAllocInputBuffer (self->enc, &in);

  if (ValidBitstreamFrameNum (self->enc) <= 0) {
    /* Nothing yet; the encoder is still filling. Not an error. */
    gst_video_codec_frame_unref (frame);
    return GST_FLOW_OK;
  }

  memset (&ob, 0, sizeof (ob));
  if (GetOneBitstreamFrame (self->enc, &ob) != 0) {
    GST_WARNING_OBJECT (self, "GetOneBitstreamFrame failed");
    gst_video_codec_frame_unref (frame);
    return GST_FLOW_OK;
  }

  /* Annex-B is the same size as AVCC (4-byte prefix either way), plus the
   * parameter sets when this turns out to be an IDR. */
  total = (gsize) ob.nSize0 + (gsize) ob.nSize1 + self->hdr_len;
  outbuf = gst_buffer_new_allocate (NULL, total, NULL);
  if (!outbuf) {
    FreeOneBitStreamFrame (self->enc, &ob);
    gst_video_codec_frame_unref (frame);
    return GST_FLOW_ERROR;
  }
  gst_buffer_map (outbuf, &map, GST_MAP_WRITE);

  o = self->hdr_len;            /* leave room; filled in once we know it is an IDR */
  if (ob.nSize0 && ob.pData0) {
    gboolean k = FALSE;
    o += avcc_to_annexb (ob.pData0, ob.nSize0, map.data + o, &k);
    idr |= k;
  }
  if (ob.nSize1 && ob.pData1) {
    gboolean k = FALSE;
    o += avcc_to_annexb (ob.pData1, ob.nSize1, map.data + o, &k);
    idr |= k;
  }

  if (idr && self->hdr_len) {
    memcpy (map.data, self->hdr, self->hdr_len);
  } else if (self->hdr_len) {
    /* Not an IDR: shuffle the slice data down over the reserved header space. */
    memmove (map.data, map.data + self->hdr_len, o - self->hdr_len);
    o -= self->hdr_len;
  }

  gst_buffer_unmap (outbuf, &map);
  gst_buffer_set_size (outbuf, o);
  FreeOneBitStreamFrame (self->enc, &ob);

  if (idr)
    GST_VIDEO_CODEC_FRAME_SET_SYNC_POINT (frame);
  else
    GST_BUFFER_FLAG_SET (outbuf, GST_BUFFER_FLAG_DELTA_UNIT);

  frame->output_buffer = outbuf;
  return gst_video_encoder_finish_frame (encoder, frame);
}

/* ------------------------------------------------------------ properties */

static void
gst_cedar_zc_enc_set_property (GObject * object, guint id, const GValue * v,
    GParamSpec * spec)
{
  GstCedarZcEnc *self = GST_CEDAR_ZC_ENC (object);

  switch (id) {
    case PROP_BITRATE:
      self->bitrate = g_value_get_uint (v);
      /* Live: nBitrate is not carried in the SPS or PPS, so changing it does
       * not invalidate the synthesized parameter sets. */
      if (self->enc) {
        int br = (int) (self->bitrate * 1000);
        VideoEncSetParameter (self->enc, VENC_IndexParamBitrate, &br);
      }
      break;
    case PROP_GOP:
      self->gop = g_value_get_uint (v);
      if (self->enc) {
        int g = (int) self->gop;
        VideoEncSetParameter (self->enc, VENC_IndexParamMaxKeyInterval, &g);
      }
      break;
    case PROP_QP_MIN:
      self->qp_min = g_value_get_uint (v);
      break;
    case PROP_QP_MAX:
      self->qp_max = g_value_get_uint (v);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, id, spec);
      break;
  }
}

static void
gst_cedar_zc_enc_get_property (GObject * object, guint id, GValue * v,
    GParamSpec * spec)
{
  GstCedarZcEnc *self = GST_CEDAR_ZC_ENC (object);

  switch (id) {
    case PROP_BITRATE:
      g_value_set_uint (v, self->bitrate);
      break;
    case PROP_GOP:
      g_value_set_uint (v, self->gop);
      break;
    case PROP_QP_MIN:
      g_value_set_uint (v, self->qp_min);
      break;
    case PROP_QP_MAX:
      g_value_set_uint (v, self->qp_max);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, id, spec);
      break;
  }
}

static void
gst_cedar_zc_enc_init (GstCedarZcEnc * self)
{
  self->bitrate = DEFAULT_BITRATE;
  self->gop = DEFAULT_GOP;
  self->qp_min = DEFAULT_QP_MIN;
  self->qp_max = DEFAULT_QP_MAX;
}

static void
gst_cedar_zc_enc_class_init (GstCedarZcEncClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstVideoEncoderClass *enc_class = GST_VIDEO_ENCODER_CLASS (klass);

  gobject_class->set_property = gst_cedar_zc_enc_set_property;
  gobject_class->get_property = gst_cedar_zc_enc_get_property;

  g_object_class_install_property (gobject_class, PROP_BITRATE,
      g_param_spec_uint ("bitrate", "Bitrate",
          "Target bitrate in kbps (CBR). Settable while running.",
          64, 40000, DEFAULT_BITRATE,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  g_object_class_install_property (gobject_class, PROP_GOP,
      g_param_spec_uint ("gop", "Keyframe interval",
          "Frames between IDRs. RTMP and HLS want 2 seconds worth.",
          1, 600, DEFAULT_GOP, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  g_object_class_install_property (gobject_class, PROP_QP_MIN,
      g_param_spec_uint ("qp-min", "Minimum QP", "Lower QP bound",
          0, 51, DEFAULT_QP_MIN, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  g_object_class_install_property (gobject_class, PROP_QP_MAX,
      g_param_spec_uint ("qp-max", "Maximum QP", "Upper QP bound",
          0, 51, DEFAULT_QP_MAX, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  gst_element_class_add_static_pad_template (element_class, &sink_template);
  gst_element_class_add_static_pad_template (element_class, &src_template);
  gst_element_class_set_static_metadata (element_class,
      "Cedar hardware H.264 encoder", "Codec/Encoder/Video/Hardware",
      "Hardware H.264 encoding on Allwinner A733, handling the 16-alignment "
      "rule, AVCC output and absent parameter sets",
      "https://github.com/skamagedon/a733-zero-copy");

  enc_class->start = gst_cedar_zc_enc_start;
  enc_class->stop = gst_cedar_zc_enc_stop;
  enc_class->set_format = gst_cedar_zc_enc_set_format;
  enc_class->handle_frame = gst_cedar_zc_enc_handle_frame;
}

static gboolean
plugin_init (GstPlugin * plugin)
{
  GST_DEBUG_CATEGORY_INIT (cedarzcenc_debug, "cedarzcenc", 0,
      "Cedar hardware encoder");
  /* Ranked above x264enc, which on this board manages about 16 fps at 720p
   * against this element's ~203. */
  return gst_element_register (plugin, "cedarzcenc", GST_RANK_PRIMARY + 1,
      GST_TYPE_CEDAR_ZC_ENC);
}

#ifndef PACKAGE
#define PACKAGE "cedarzcenc"
#endif
#ifndef VERSION
#define VERSION "0.1.0"
#endif

GST_PLUGIN_DEFINE (GST_VERSION_MAJOR, GST_VERSION_MINOR,
    cedarzcenc, "Hardware H.264 encoding for Allwinner A733",
    plugin_init, VERSION, "LGPL", "a733-zero-copy",
    "https://github.com/skamagedon/a733-zero-copy")
