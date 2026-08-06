/*
 * sps-diag.c - where do the A733 encoder's SPS/PPS actually live?
 *
 * VideoEncGetParameter(VENC_IndexParamH264SPSPPS) returns a correctly shaped
 * avcC record whose payloads are 0xff filler. This tests four hypotheses in one
 * run rather than guessing one at a time:
 *
 *   A. VencHeaderData expects CALLER-SUPPLIED storage, and returning a struct
 *      zeroed by us yields the library's uninitialised scratch buffer.
 *   B. The parameter sets are delivered in the first output buffer's extra data
 *      pointers (pData1/pData2), which a pData0-only reader would never see.
 *   C. The record is populated only after more than one frame.
 *   D. Some other index (e.g. the H265 header index) is wired for H264 too.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "vencoder.h"
#include "memoryAdapter.h"

#define VE_OPS_TYPE_AW 0
extern VeOpsS *GetVeOpsS(int type);

static void dump(const char *tag, const unsigned char *d, unsigned int n)
{
	unsigned int i, show = n > 32 ? 32 : n;
	int allff = 1;

	if (!d || !n) {
		printf("    %-22s (none)\n", tag);
		return;
	}
	for (i = 0; i < n; i++)
		if (d[i] != 0xff) { allff = 0; break; }

	printf("    %-22s len=%-6u %s", tag, n, allff ? "[ALL 0xff] " : "");
	for (i = 0; i < show; i++)
		printf("%02x ", d[i]);
	printf("%s\n", n > show ? "..." : "");
}

static void probe_getparam(VideoEncoder *enc, const char *when)
{
	VencHeaderData hdr;
	unsigned char scratch[512];

	/* Hypothesis A-1: zeroed struct, library allocates. */
	memset(&hdr, 0, sizeof(hdr));
	if (VideoEncGetParameter(enc, VENC_IndexParamH264SPSPPS, &hdr) == 0)
		dump("SPSPPS lib-alloc", hdr.pBuffer, hdr.nLength);
	else
		printf("    SPSPPS lib-alloc       call failed\n");

	/* Hypothesis A-2: caller supplies the buffer and its capacity. */
	memset(scratch, 0xa5, sizeof(scratch));	/* poison, so fills are visible */
	memset(&hdr, 0, sizeof(hdr));
	hdr.pBuffer = scratch;
	hdr.nLength = sizeof(scratch);
	if (VideoEncGetParameter(enc, VENC_IndexParamH264SPSPPS, &hdr) == 0)
		dump("SPSPPS caller-buf", hdr.pBuffer, hdr.nLength > sizeof(scratch)
		     ? (unsigned int)sizeof(scratch) : hdr.nLength);
	else
		printf("    SPSPPS caller-buf      call failed\n");

	(void)when;
}

int main(int argc, char **argv)
{
	int w = (argc > 1) ? atoi(argv[1]) : 1280;
	int h = (argc > 2) ? atoi(argv[2]) : 720;
	struct ScMemOpsS *memops;
	VideoEncoder *enc;
	VencBaseConfig cfg;
	VencAllocateBufferParam bp;
	int n;

	memops = MemAdapterGetOpsS();
	if (!memops || CdcMemOpen(memops) != 0) {
		fprintf(stderr, "FAIL: memory adapter\n");
		return 1;
	}
	enc = VideoEncCreate(VENC_CODEC_H264);
	if (!enc) {
		fprintf(stderr, "FAIL: VideoEncCreate (need root for /dev/cedar_dev_ve2)\n");
		return 1;
	}

	memset(&cfg, 0, sizeof(cfg));
	cfg.nInputWidth = w;  cfg.nInputHeight = h;
	cfg.nDstWidth   = w;  cfg.nDstHeight   = h;
	cfg.nStride     = w;
	cfg.eInputFormat = VENC_PIXEL_YUV420SP;
	cfg.memops = memops;
	cfg.veOpsS = GetVeOpsS(VE_OPS_TYPE_AW);
	cfg.bEncH264Nalu = 1;

	if (VideoEncInit(enc, &cfg) != 0) {
		fprintf(stderr, "FAIL: VideoEncInit\n");
		return 1;
	}

	printf("== BEFORE any frame ==\n");
	probe_getparam(enc, "before");

	memset(&bp, 0, sizeof(bp));
	bp.nBufferNum = 4; bp.nSizeY = w * h; bp.nSizeC = w * h / 2;
	AllocInputBuffer(enc, &bp);

	for (n = 0; n < 3; n++) {
		VencInputBuffer in;
		VencOutputBuffer ob;

		memset(&in, 0, sizeof(in));
		if (GetOneAllocInputBuffer(enc, &in) != 0)
			break;
		memset(in.pAddrVirY, 0x40 + n * 8, (size_t)w * h);
		memset(in.pAddrVirC, 0x80, (size_t)w * h / 2);
		FlushCacheAllocInputBuffer(enc, &in);
		in.nPts = n * 33333;
		AddOneInputBuffer(enc, &in);
		if (VideoEncodeOneFrame(enc) != 0) {
			printf("encode failed at frame %d\n", n);
			break;
		}
		AlreadyUsedInputBuffer(enc, &in);
		ReturnOneAllocInputBuffer(enc, &in);

		printf("\n== AFTER frame %d ==\n", n);

		/* Hypothesis B: extra data pointers on the output buffer. */
		if (ValidBitstreamFrameNum(enc) > 0) {
			memset(&ob, 0, sizeof(ob));
			if (GetOneBitstreamFrame(enc, &ob) == 0) {
				printf("  output buffer:\n");
				dump("pData0", ob.pData0, ob.nSize0);
				dump("pData1", ob.pData1, ob.nSize1);
				dump("pData2", ob.pData2, ob.nSize2);
				FreeOneBitStreamFrame(enc, &ob);
			}
		}
		/* Hypothesis C: populated after N frames. */
		printf("  get-parameter:\n");
		probe_getparam(enc, "after");
	}

	ReleaseAllocInputBuffer(enc);
	VideoEncUnInit(enc);
	VideoEncDestroy(enc);
	CdcMemClose(memops);
	return 0;
}
