/*
 * cedar-dmabuf-drm-probe.c
 *
 * Purpose
 * -------
 * Prove (or disprove) the one link in the A733 zero-copy chain that the vendor
 * OMX/GstOMX path could not expose: that a *Cedar decoded frame* can be handed
 * to DRM as a DMA-BUF and scanned out without any CPU pixel copy.
 *
 * This deliberately bypasses OpenMAX and GStreamer. The vendor libOmxVdec.so
 * reports the proprietary colour format 0x7f000002 through a malformed port
 * format enumeration, so its GStreamer wrapper cannot negotiate the DMA-BUF
 * caps it advertises. libvdecoder.so is the layer *underneath* that wrapper and
 * publishes the decoded surface's DMA-BUF descriptor directly as
 * VideoPicture.nBufFd, so the broken negotiation is not on the path at all.
 *
 * Chain under test:
 *   H.264 Annex-B -> libvdecoder -> VideoPicture.nBufFd (dma_buf)
 *                 -> drmPrimeFDToHandle -> drmModeAddFB2WithModifiers
 *
 * The program never mmaps the decoded surface. Any pixel access would defeat
 * the property being tested, so its absence is part of the result.
 *
 * Usage: cedar-dmabuf-drm-probe <file.h264> [drm-card]
 * Exit:  0 = Cedar surface imported by DRM, 1 = failure (reason on stderr)
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

#include "vdecoder.h"
#include "memoryAdapter.h"

/* veInterface.h ships the VeOpsS type but not this accessor's prototype, even
 * though libVE.so exports it. Disassembly of libVE.so shows it dispatches on an
 * engine type: 0 -> getVeAwOpsS (H.264/H.265), 1 -> getVeVp9OpsS. Anything else
 * is rejected, so the argument must be passed explicitly. */
#define VE_OPS_TYPE_AW  0
#define VE_OPS_TYPE_VP9 1
extern VeOpsS *GetVeOpsS(int type);

#ifndef DMA_BUF_SET_NAME
#include <linux/dma-buf.h>
#endif

#define STAGE(x) fprintf(stderr, "[stage] " x "\n")

#define CHUNK_SIZE (512 * 1024)
#define MAX_DECODE_ROUNDS 600

static const char *pixfmt_name(int f)
{
	switch (f) {
	case PIXEL_FORMAT_YV12:		return "YV12";
	case PIXEL_FORMAT_NV21:		return "NV21";
	case PIXEL_FORMAT_NV12:		return "NV12";
	case PIXEL_FORMAT_YUV_MB32_420:	return "YUV_MB32_420 (tiled)";
	case PIXEL_FORMAT_P010_UV:	return "P010_UV";
	case PIXEL_FORMAT_P010_VU:	return "P010_VU";
	case PIXEL_FORMAT_YUV_PLANER_420: return "YUV_PLANER_420";
	default:			return "unknown";
	}
}

/* Only formats a DRM plane can scan out directly are mapped. A tiled or
 * compressed surface is a real result, not an error, but it needs a modifier
 * the display engine advertises, so it is reported rather than guessed at. */
static uint32_t drm_fourcc_for(int pixfmt, int *nplanes)
{
	switch (pixfmt) {
	case PIXEL_FORMAT_NV21:		*nplanes = 2; return DRM_FORMAT_NV21;
	case PIXEL_FORMAT_NV12:		*nplanes = 2; return DRM_FORMAT_NV12;
	case PIXEL_FORMAT_P010_UV:	*nplanes = 2; return DRM_FORMAT_P010;
	case PIXEL_FORMAT_YV12:		*nplanes = 3; return DRM_FORMAT_YVU420;
	case PIXEL_FORMAT_YUV_PLANER_420: *nplanes = 3; return DRM_FORMAT_YUV420;
	default:			*nplanes = 0; return 0;
	}
}

/* A dma_buf fd resolves through /proc/self/fd to a "dmabuf" link target. This
 * distinguishes a genuine exported buffer from an ordinary fd that merely
 * happens to be an integer in the struct. */
static int fd_is_dmabuf(int fd, char *out, size_t outsz)
{
	char link[64];
	ssize_t n;

	snprintf(link, sizeof(link), "/proc/self/fd/%d", fd);
	n = readlink(link, out, outsz - 1);
	if (n < 0)
		return 0;
	out[n] = '\0';
	return strstr(out, "dmabuf") != NULL;
}

int main(int argc, char **argv)
{
	const char *path = (argc > 1) ? argv[1] : NULL;
	const char *card = (argc > 2) ? argv[2] : "/dev/dri/card0";
	struct ScMemOpsS *memops = NULL;
	VideoDecoder *dec = NULL;
	VideoStreamInfo si;
	VConfig vc;
	VideoPicture *pic = NULL;
	FILE *f = NULL;
	unsigned char *chunk = NULL;
	int drm_fd = -1, rc = 1, rounds = 0, eos = 0, mem_open = 0;
	uint32_t handles[4] = {0}, pitches[4] = {0}, offsets[4] = {0}, fb = 0;
	uint64_t modifiers[4] = {0};
	uint32_t gem = 0, fourcc;
	int nplanes = 0;
	char linkbuf[256];

	if (!path) {
		fprintf(stderr, "usage: %s <file.h264> [drm-card]\n", argv[0]);
		return 1;
	}

	f = fopen(path, "rb");
	if (!f) {
		fprintf(stderr, "FAIL: open %s: %s\n", path, strerror(errno));
		return 1;
	}
	chunk = malloc(CHUNK_SIZE);
	if (!chunk) {
		fprintf(stderr, "FAIL: out of memory\n");
		goto out;
	}

	memops = MemAdapterGetOpsS();
	if (!memops) {
		fprintf(stderr, "FAIL: MemAdapterGetOpsS returned NULL\n");
		goto out;
	}
	/* open() is the allocator's initialiser; setup() is a separate hook this
	 * build does not implement. open2() also exists, but it dereferences the
	 * VE-ops self pointer it is handed, so it cannot be called before a VE
	 * context exists — passing NULL faults inside the allocator. */
	if (CdcMemOpen(memops) == 0)
		mem_open = 1;
	if (!mem_open) {
		fprintf(stderr, "FAIL: could not open Cedar memory adapter\n");
		goto out;
	}

	STAGE("mem adapter open");
	AddVDPlugin();
	STAGE("AddVDPlugin done");
	dec = CreateVideoDecoder();
	STAGE("CreateVideoDecoder returned");
	if (!dec) {
		fprintf(stderr, "FAIL: CreateVideoDecoder\n");
		goto out;
	}

	memset(&si, 0, sizeof(si));
	si.eCodecFormat = VIDEO_CODEC_FORMAT_H264;
	si.bIsFramePackage = 0;	/* Annex-B byte stream, not whole frames */

	memset(&vc, 0, sizeof(vc));
	vc.memops = memops;
	vc.veOpsS = GetVeOpsS(VE_OPS_TYPE_AW);
	vc.pVeOpsSelf = NULL;
	/* Ask for a linear semi-planar surface a DRM plane can scan out. If the
	 * decoder overrides this the actual value is reported below. */
	vc.eOutputPixelFormat = PIXEL_FORMAT_NV21;
	vc.nFrameBufferNum = 6;
	vc.bDispErrorFrame = 1;
	vc.bSupportPallocBufBeforeDecode = 1;

	STAGE("calling InitializeVideoDecoder");
	if (InitializeVideoDecoder(dec, &si, &vc) != 0) {
		fprintf(stderr, "FAIL: InitializeVideoDecoder\n");
		goto out;
	}

	STAGE("decoder initialised; feeding stream");

	/* Feed until the decoder produces its first picture. */
	while (rounds++ < MAX_DECODE_ROUNDS && !pic) {
		char *buf = NULL, *ring = NULL;
		int bufsz = 0, ringsz = 0;
		size_t got = 0;

		if (!eos) {
			got = fread(chunk, 1, CHUNK_SIZE, f);
			if (got == 0)
				eos = 1;
		}

		if (got > 0) {
			if (RequestVideoStreamBuffer(dec, (int)got, &buf, &bufsz,
						     &ring, &ringsz, 0) == 0 &&
			    buf && (size_t)(bufsz + ringsz) >= got) {
				VideoStreamDataInfo di;

				/* The stream buffer is a ring; a request may be
				 * split across its wrap point. */
				memcpy(buf, chunk, (size_t)bufsz);
				if (ringsz > 0 && got > (size_t)bufsz)
					memcpy(ring, chunk + bufsz, got - bufsz);

				memset(&di, 0, sizeof(di));
				di.pData = buf;
				di.nLength = (int)got;
				di.bIsFirstPart = 1;
				di.bIsLastPart = 1;
				di.nPts = -1;
				di.bValid = 1;
				SubmitVideoStreamData(dec, &di, 0);
			}
		}

		DecodeVideoStream(dec, eos, 0, 0, 0);
		pic = RequestPicture(dec, 0);

		if (!pic && eos && rounds > 8)
			break;
	}

	if (!pic) {
		fprintf(stderr, "FAIL: no picture decoded after %d rounds\n", rounds);
		goto out;
	}

	printf("== Cedar decoded picture ==\n");
	printf("  format      : %d (%s)\n", pic->ePixelFormat,
	       pixfmt_name(pic->ePixelFormat));
	printf("  size        : %dx%d  stride %d\n", pic->nWidth, pic->nHeight,
	       pic->nLineStride);
	printf("  crop        : top %d left %d bottom %d right %d\n",
	       pic->nTopOffset, pic->nLeftOffset, pic->nBottomOffset,
	       pic->nRightOffset);
	printf("  nBufFd      : %d\n", pic->nBufFd);
	printf("  nBufSize    : %d\n", pic->nBufSize);
	printf("  afbc        : %d (afbc size %d)\n", pic->bEnableAfbcFlag,
	       pic->nAfbcSize);
	printf("  10-bit      : %d\n", pic->b10BitPicFlag);

	if (pic->nBufFd < 0) {
		fprintf(stderr, "FAIL: decoder did not export a DMA-BUF fd\n");
		goto out;
	}
	if (!fd_is_dmabuf(pic->nBufFd, linkbuf, sizeof(linkbuf))) {
		fprintf(stderr, "FAIL: fd %d is not a dma_buf (resolves to %s)\n",
			pic->nBufFd, linkbuf);
		goto out;
	}
	printf("  fd resolves : %s  [genuine dma_buf]\n", linkbuf);

	if (pic->bEnableAfbcFlag) {
		fprintf(stderr,
			"FAIL: surface is AFBC-compressed; scanout needs a matching\n"
			"      DRM modifier from the display engine, not a linear import.\n");
		goto out;
	}

	fourcc = drm_fourcc_for(pic->ePixelFormat, &nplanes);
	if (!fourcc) {
		fprintf(stderr,
			"FAIL: pixel format %d (%s) has no direct linear DRM mapping\n",
			pic->ePixelFormat, pixfmt_name(pic->ePixelFormat));
		goto out;
	}

	drm_fd = open(card, O_RDWR | O_CLOEXEC);
	if (drm_fd < 0) {
		fprintf(stderr, "FAIL: open %s: %s\n", card, strerror(errno));
		goto out;
	}

	if (drmPrimeFDToHandle(drm_fd, pic->nBufFd, &gem) != 0) {
		fprintf(stderr, "FAIL: drmPrimeFDToHandle: %s\n", strerror(errno));
		goto out;
	}
	printf("  PRIME import: GEM handle %u\n", gem);

	/* Plane offsets are derived from the decoder's own plane pointers, which
	 * all address the same allocation. Deriving them avoids assuming an
	 * alignment rule the decoder may not follow. */
	handles[0] = gem;
	pitches[0] = (uint32_t)pic->nLineStride;
	offsets[0] = 0;
	if (nplanes >= 2) {
		handles[1] = gem;
		pitches[1] = (uint32_t)pic->nLineStride;
		offsets[1] = (uint32_t)(pic->pData1 - pic->pData0);
	}
	if (nplanes >= 3) {
		handles[2] = gem;
		pitches[2] = (uint32_t)(pic->nLineStride / 2);
		pitches[1] = (uint32_t)(pic->nLineStride / 2);
		offsets[2] = (uint32_t)(pic->pData2 - pic->pData0);
	}

	if (drmModeAddFB2WithModifiers(drm_fd, pic->nWidth, pic->nHeight, fourcc,
				       handles, pitches, offsets, modifiers,
				       &fb, 0) != 0) {
		fprintf(stderr, "FAIL: drmModeAddFB2WithModifiers: %s\n",
			strerror(errno));
		fprintf(stderr, "      fourcc %.4s planes %d pitch %u chroma off %u\n",
			(char *)&fourcc, nplanes, pitches[0], offsets[1]);
		goto out;
	}

	printf("\nPASS: Cedar decoded surface imported by DRM as framebuffer %u\n", fb);
	printf("      fourcc %.4s, %d plane(s), no CPU mapping performed.\n",
	       (char *)&fourcc, nplanes);
	rc = 0;

out:
	if (fb)
		drmModeRmFB(drm_fd, fb);
	if (drm_fd >= 0)
		close(drm_fd);
	if (pic && dec)
		ReturnPicture(dec, pic);
	if (dec)
		DestroyVideoDecoder(dec);
	if (mem_open)
		CdcMemClose(memops);
	free(chunk);
	if (f)
		fclose(f);
	return rc;
}
