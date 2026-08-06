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
	cfg.bEncH264Nalu  = 1;				/* Annex-B output */

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

		if (ValidBitstreamFrameNum(enc) > 0) {
			memset(&ob, 0, sizeof(ob));
			if (GetOneBitstreamFrame(enc, &ob) == 0) {
				total_bytes += ob.nSize0 + ob.nSize1;
				if (f) {
					if (ob.nSize0 && ob.pData0)
						fwrite(ob.pData0, 1, ob.nSize0, f);
					if (ob.nSize1 && ob.pData1)
						fwrite(ob.pData1, 1, ob.nSize1, f);
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
