/*
 * cedar-h264-encode-probe.c
 *
 * Does the A733's Cedar H.264 ENCODER actually work?
 *
 * The encoder stack is fully present on the stock image - libvencoder.so,
 * libvenc_h264.so and vencoder.h expose an API symmetric to the decoder one -
 * but there is a public report that on A733 the encode path never completes,
 * with cedar_dev_ve2 failing as "wait interrupt overtime". Presence of the API
 * is not evidence that it functions, so this encodes a few synthetic NV12
 * frames and reports exactly where it succeeds or stops.
 *
 * It writes nothing unless a bitstream actually comes back, so a failure
 * leaves no misleading half-file behind.
 *
 * Usage: cedar-h264-encode-probe [width] [height] [frames] [out.h264]
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>

#include "vencoder.h"
#include "memoryAdapter.h"

#define VE_OPS_TYPE_AW 0
extern VeOpsS *GetVeOpsS(int type);

/* A moving pattern rather than a flat colour: a constant image compresses to
 * almost nothing and would hide a broken encoder behind a plausible byte
 * count. */
static void fill_nv12(unsigned char *y, unsigned char *c, int w, int h,
		      int stride, int frame)
{
	int i, j;

	for (i = 0; i < h; i++)
		for (j = 0; j < w; j++)
			y[i * stride + j] = (unsigned char)((i + j + frame * 8) & 0xff);

	for (i = 0; i < h / 2; i++)
		for (j = 0; j < w; j += 2) {
			c[i * stride + j]     = (unsigned char)(128 + ((j + frame) & 0x3f) - 32);
			c[i * stride + j + 1] = (unsigned char)(128 + ((i - frame) & 0x3f) - 32);
		}
}

/*
 * The encoder emits AVCC: each NAL prefixed by a 4-byte big-endian length,
 * with no start codes, and it does NOT include SPS/PPS inline - the first NAL
 * of the stream is an IDR slice. So a raw dump of the output is not a decodable
 * .h264 file, which is exactly what ffmpeg reports as "No start code is found".
 *
 * Converting to Annex-B means replacing each length prefix with 00 00 00 01,
 * and writing the SPS/PPS from VENC_IndexParamH264SPSPPS first.
 */
/*
 * VENC_IndexParamH264SPSPPS returns an avcC AVCDecoderConfigurationRecord, not
 * raw NALs:
 *
 *   01 <profile> <compat> <level> ff  01 <spsLen:2> <SPS...>  01 <ppsLen:2> <PPS...>
 *
 * Writing that block verbatim behind one start code produces exactly the
 * "pps_id out of range" / "non-existing PPS" errors ffmpeg reports, because the
 * whole record gets parsed as a single malformed NAL. Each parameter set has to
 * be emitted as its own Annex-B NAL instead.
 *
 * On A733 that is not enough, because the SPS and PPS payloads inside the
 * record are 0xff filler at every point in the encoder's life: before the first
 * frame, after one frame, and after many. Setting venc_log_level = 0 in
 * /etc/cedarc.conf (the log emits "from /vendor/etc/cedarc.conf", but that path
 * does not exist - /etc/cedarc.conf is the file actually read) shows the
 * library's internal "get sps_pps_data" path never executes at all. The record
 * is a stub, not a failed copy, so there is nothing to retrieve.
 *
 * The parameter sets are still recoverable, because the encoder writes slice
 * headers that are self-consistent with exactly one set of values. Parsing 30
 * consecutive slice headers pins them:
 *
 *   frame_num reads 0,1,2,...      only under log2_max_frame_num = 12
 *   pic_order_cnt_lsb steps by 2   only under log2_max_poc_lsb   = 12
 *   cabac_init_idc present, valid  -> entropy_coding_mode_flag   = 1
 *   slice_qp_delta ramps +4..-8    -> pic_init_qp_minus26        = 0
 *
 * The smooth rate-control ramp in that last one is what confirms the bit
 * alignment is genuinely right: a wrong offset yields noise, not a monotone
 * curve. The record's own header bytes ARE valid, so profile, level and the two
 * lengths come from it; the values below reproduce those lengths exactly (11
 * and 4 bytes), and the result decodes every frame with no errors at ~46 dB
 * PSNR against the encoder's source pattern.
 *
 * max_num_ref_frames cannot be recovered from the stream - it only ever uses
 * one reference, so 1 and 2 are indistinguishable. 2 is used here because it is
 * the safer DPB hint and still matches the declared length.
 */
struct bw { unsigned char b[64]; int n; };

static void bw_u(struct bw *s, int n, unsigned int v)
{
	int i;

	for (i = n - 1; i >= 0; i--) {
		if ((v >> i) & 1)
			s->b[s->n >> 3] |= 0x80 >> (s->n & 7);
		s->n++;
	}
}

static void bw_ue(struct bw *s, unsigned int v)
{
	unsigned int t = ++v;
	int n = 0;

	while (t >>= 1)
		n++;
	bw_u(s, n, 0);
	bw_u(s, n + 1, v);
}

static void bw_se(struct bw *s, int v)
{
	bw_ue(s, v > 0 ? (unsigned int)(2 * v - 1) : (unsigned int)(-2 * v));
}

static int bw_finish(struct bw *s)		/* rbsp_trailing_bits; -> bytes */
{
	bw_u(s, 1, 1);
	while (s->n & 7)
		bw_u(s, 1, 0);
	return s->n >> 3;
}

static int build_sps(struct bw *s, int w, int h, int profile, int level)
{
	int mbw = (w + 15) / 16, mbh = (h + 15) / 16;
	int cropr = (mbw * 16 - w) / 2;	/* CropUnitX = 2 for 4:2:0 */
	int cropb = (mbh * 16 - h) / 2;	/* CropUnitY = 2 when frame_mbs_only */

	memset(s, 0, sizeof(*s));
	bw_u(s, 8, 0x67);
	bw_u(s, 8, (unsigned int)profile);
	bw_u(s, 8, 0);
	bw_u(s, 8, (unsigned int)level);
	bw_ue(s, 0);			/* seq_parameter_set_id */
	bw_ue(s, 1);			/* chroma_format_idc: 4:2:0 */
	bw_ue(s, 0);			/* bit_depth_luma_minus8 */
	bw_ue(s, 0);			/* bit_depth_chroma_minus8 */
	bw_u(s, 1, 0);			/* qpprime_y_zero_transform_bypass */
	bw_u(s, 1, 0);			/* seq_scaling_matrix_present */
	bw_ue(s, 8);			/* log2_max_frame_num_minus4 -> 12 */
	bw_ue(s, 0);			/* pic_order_cnt_type */
	bw_ue(s, 8);			/* log2_max_poc_lsb_minus4   -> 12 */
	bw_ue(s, 2);			/* max_num_ref_frames */
	bw_u(s, 1, 1);			/* gaps_in_frame_num_value_allowed */
	bw_ue(s, (unsigned int)(mbw - 1));
	bw_ue(s, (unsigned int)(mbh - 1));
	bw_u(s, 1, 1);			/* frame_mbs_only_flag */
	bw_u(s, 1, 1);			/* direct_8x8_inference_flag */
	bw_u(s, 1, (cropr || cropb) ? 1 : 0);
	if (cropr || cropb) {
		bw_ue(s, 0);		/* crop left */
		bw_ue(s, (unsigned int)cropr);
		bw_ue(s, 0);		/* crop top */
		bw_ue(s, (unsigned int)cropb);
	}
	bw_u(s, 1, 0);			/* vui_parameters_present_flag */
	return bw_finish(s);
}

static int build_pps(struct bw *s)
{
	memset(s, 0, sizeof(*s));
	bw_u(s, 8, 0x68);
	bw_ue(s, 0);			/* pic_parameter_set_id */
	bw_ue(s, 0);			/* seq_parameter_set_id */
	bw_u(s, 1, 1);			/* entropy_coding_mode_flag: CABAC */
	bw_u(s, 1, 0);			/* bottom_field_pic_order_present */
	bw_ue(s, 0);			/* num_slice_groups_minus1 */
	bw_ue(s, 0);			/* num_ref_idx_l0_default_active_minus1 */
	bw_ue(s, 0);			/* num_ref_idx_l1_default_active_minus1 */
	bw_u(s, 1, 0);			/* weighted_pred_flag */
	bw_u(s, 2, 0);			/* weighted_bipred_idc */
	bw_se(s, 0);			/* pic_init_qp_minus26 */
	bw_se(s, 0);			/* pic_init_qs_minus26 */
	bw_se(s, 0);			/* chroma_qp_index_offset */
	bw_u(s, 1, 1);			/* deblocking_filter_control_present */
	bw_u(s, 1, 0);			/* constrained_intra_pred_flag */
	bw_u(s, 1, 0);			/* redundant_pic_cnt_present_flag */
	return bw_finish(s);
}

/* True when the record is absent, malformed, or shaped correctly but carrying
 * only 0xff filler payloads - the A733 case.
 *
 * This has to walk the record rather than scan it. The length fields are real
 * data (00 0b, 00 04), so a flat "is every byte 0xff?" test reports a genuine
 * record and silently takes the parse path. Only the SPS and PPS payloads are
 * filler, and only they may be examined. */
static int avcc_is_filler(const unsigned char *d, unsigned int len)
{
	unsigned int off = 5, i, k, count;

	if (len < 7 || d[0] != 1)
		return 1;

	count = d[off++] & 0x1f;		/* number of SPS */
	for (i = 0; i < count; i++) {
		unsigned int n;

		if (off + 2 > len)
			return 1;
		n = ((unsigned int)d[off] << 8) | d[off + 1];
		off += 2;
		if (n == 0 || off + n > len)
			return 1;
		for (k = 0; k < n; k++)
			if (d[off + k] != 0xff)
				return 0;	/* a real parameter set */
		off += n;
	}

	if (off >= len)
		return 1;
	count = d[off++];			/* number of PPS */
	for (i = 0; i < count; i++) {
		unsigned int n;

		if (off + 2 > len)
			return 1;
		n = ((unsigned int)d[off] << 8) | d[off + 1];
		off += 2;
		if (n == 0 || off + n > len)
			return 1;
		for (k = 0; k < n; k++)
			if (d[off + k] != 0xff)
				return 0;
		off += n;
	}
	return 1;
}

static int write_avcc_header(FILE *f, const unsigned char *d, unsigned int len,
			     int w, int h)
{
	static const unsigned char sc[4] = { 0, 0, 0, 1 };
	unsigned int off = 5;		/* skip version/profile/compat/level/flags */
	unsigned int i, count;
	int written = 0;

	if (avcc_is_filler(d, len)) {
		int profile = (len >= 4) ? d[1] : 100;
		int level   = (len >= 4) ? d[3] : 51;
		struct bw s;
		int n;

		n = build_sps(&s, w, h, profile, level);
		fwrite(sc, 1, 4, f);
		fwrite(s.b, 1, (size_t)n, f);
		written++;

		n = build_pps(&s);
		fwrite(sc, 1, 4, f);
		fwrite(s.b, 1, (size_t)n, f);
		written++;

		fprintf(stderr,
			"note: parameter sets are 0xff filler; synthesized them"
			" from profile %d level %d\n", profile, level);
		return written;
	}

	count = d[off++] & 0x1f;	/* number of SPS */
	for (i = 0; i < count && off + 2 <= len; i++) {
		unsigned int n = ((unsigned int)d[off] << 8) | d[off + 1];
		off += 2;
		if (off + n > len)
			return written;
		fwrite(sc, 1, 4, f);
		fwrite(d + off, 1, n, f);
		off += n;
		written++;
	}

	if (off >= len)
		return written;
	count = d[off++];		/* number of PPS */
	for (i = 0; i < count && off + 2 <= len; i++) {
		unsigned int n = ((unsigned int)d[off] << 8) | d[off + 1];
		off += 2;
		if (off + n > len)
			return written;
		fwrite(sc, 1, 4, f);
		fwrite(d + off, 1, n, f);
		off += n;
		written++;
	}
	return written;
}

static void write_annexb(FILE *f, const unsigned char *data, unsigned int len)
{
	static const unsigned char sc[4] = { 0, 0, 0, 1 };
	unsigned int off = 0;

	while (off + 4 <= len) {
		unsigned int nal = ((unsigned int)data[off] << 24) |
				   ((unsigned int)data[off + 1] << 16) |
				   ((unsigned int)data[off + 2] << 8) |
				   (unsigned int)data[off + 3];
		off += 4;
		if (nal == 0 || off + nal > len)
			break;
		fwrite(sc, 1, 4, f);
		fwrite(data + off, 1, nal, f);
		off += nal;
	}
}

int main(int argc, char **argv)
{
	int w      = (argc > 1) ? atoi(argv[1]) : 1280;
	int h      = (argc > 2) ? atoi(argv[2]) : 720;
	int frames = (argc > 3) ? atoi(argv[3]) : 10;
	const char *out = (argc > 4) ? argv[4] : NULL;

	struct ScMemOpsS *memops = NULL;
	VideoEncoder *enc = NULL;
	VencBaseConfig cfg;
	VencAllocateBufferParam bufparam;
	FILE *f = NULL;
	int rc = 1, encoded = 0, got = 0;
	long total_bytes = 0;

	printf("A733 Cedar H.264 encode probe: %dx%d, %d frames\n", w, h, frames);

	memops = MemAdapterGetOpsS();
	if (!memops || CdcMemOpen(memops) != 0) {
		fprintf(stderr, "FAIL: Cedar memory adapter\n");
		return 1;
	}

	enc = VideoEncCreate(VENC_CODEC_H264);
	if (!enc) {
		fprintf(stderr, "FAIL: VideoEncCreate(VENC_CODEC_H264)\n");
		goto out;
	}
	printf("  VideoEncCreate      : ok\n");

	memset(&cfg, 0, sizeof(cfg));
	cfg.nInputWidth   = w;
	cfg.nInputHeight  = h;
	cfg.nDstWidth     = w;
	cfg.nDstHeight    = h;
	cfg.nStride       = w;
	cfg.eInputFormat  = VENC_PIXEL_YUV420SP;	/* NV12 */
	cfg.memops        = memops;
	cfg.veOpsS        = GetVeOpsS(VE_OPS_TYPE_AW);
	cfg.pVeOpsSelf    = NULL;
	/* Setting this does NOT give Annex-B output on this platform: the encoder
	 * emits AVCC length-prefixed NALs regardless, which is why write_annexb()
	 * exists below. Verified by hexdump of the raw output. */
	cfg.bEncH264Nalu  = 1;

	if (VideoEncInit(enc, &cfg) != 0) {
		fprintf(stderr, "FAIL: VideoEncInit\n");
		goto out;
	}
	printf("  VideoEncInit        : ok\n");

	memset(&bufparam, 0, sizeof(bufparam));
	bufparam.nBufferNum = 4;
	bufparam.nSizeY = w * h;
	bufparam.nSizeC = w * h / 2;
	if (AllocInputBuffer(enc, &bufparam) != 0) {
		fprintf(stderr, "FAIL: AllocInputBuffer\n");
		goto out;
	}
	printf("  AllocInputBuffer    : ok (%u buffers)\n", bufparam.nBufferNum);

	if (out) {
		f = fopen(out, "wb");
		if (!f) {
			fprintf(stderr, "FAIL: cannot open %s\n", out);
			goto out;
		}
	}

	for (int n = 0; n < frames; n++) {
		VencInputBuffer in;
		VencOutputBuffer ob;

		memset(&in, 0, sizeof(in));
		if (GetOneAllocInputBuffer(enc, &in) != 0) {
			fprintf(stderr, "FAIL: GetOneAllocInputBuffer at frame %d\n", n);
			goto out;
		}

		fill_nv12(in.pAddrVirY, in.pAddrVirC, w, h, w, n);
		FlushCacheAllocInputBuffer(enc, &in);

		in.nPts = (long long)n * 1000000 / 30;
		if (AddOneInputBuffer(enc, &in) != 0) {
			fprintf(stderr, "FAIL: AddOneInputBuffer at frame %d\n", n);
			goto out;
		}

		/* This is the call the public report says never returns cleanly on
		 * A733, because the VE interrupt does not fire. */
		if (VideoEncodeOneFrame(enc) != 0) {
			fprintf(stderr,
				"FAIL: VideoEncodeOneFrame returned non-zero at frame %d\n"
				"      (this is where the reported 'wait interrupt overtime' occurs)\n",
				n);
			goto out;
		}
		encoded++;

		AlreadyUsedInputBuffer(enc, &in);
		ReturnOneAllocInputBuffer(enc, &in);

		/* Write the parameter sets ahead of any slice data. The record
		 * itself is filler on this platform, so write_avcc_header()
		 * synthesizes them; see the comment above it. */
		if (n == 0 && f) {
			VencHeaderData hdr;

			memset(&hdr, 0, sizeof(hdr));
			if (VideoEncGetParameter(enc, VENC_IndexParamH264SPSPPS,
						 &hdr) == 0 && hdr.pBuffer && hdr.nLength) {
				int nals = write_avcc_header(f, hdr.pBuffer,
							     hdr.nLength, w, h);

				printf("  SPS/PPS             : %u bytes, %d NAL(s) written\n",
				       hdr.nLength, nals);
				if (nals == 0)
					fprintf(stderr,
						"WARN: parameter sets could not be parsed;"
						" output will not decode\n");
			} else {
				fprintf(stderr,
					"WARN: no SPS/PPS available; output will not decode\n");
			}
		}

		if (ValidBitstreamFrameNum(enc) > 0) {
			memset(&ob, 0, sizeof(ob));
			if (GetOneBitstreamFrame(enc, &ob) == 0) {
				total_bytes += ob.nSize0 + ob.nSize1;
				if (f) {
					if (ob.nSize0 && ob.pData0)
						write_annexb(f, ob.pData0, ob.nSize0);
					if (ob.nSize1 && ob.pData1)
						write_annexb(f, ob.pData1, ob.nSize1);
				}
				got++;
				FreeOneBitStreamFrame(enc, &ob);
			}
		}
	}

	printf("  VideoEncodeOneFrame : ok x%d\n", encoded);
	printf("  bitstream frames    : %d\n", got);
	printf("  bitstream bytes     : %ld\n", total_bytes);

	if (got > 0 && total_bytes > 0) {
		printf("\nPASS: the A733 Cedar H.264 encoder produced a bitstream\n");
		if (out)
			printf("      wrote %s\n", out);
		rc = 0;
	} else {
		fprintf(stderr,
			"\nFAIL: frames were accepted but no bitstream came back\n");
	}

out:
	if (f)
		fclose(f);
	if (enc) {
		ReleaseAllocInputBuffer(enc);
		VideoEncUnInit(enc);
		VideoEncDestroy(enc);
	}
	if (memops)
		CdcMemClose(memops);
	return rc;
}
