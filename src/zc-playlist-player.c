/*
 * zc-playlist-player.c
 *
 * Zero-copy playlist player for Allwinner A733 (Orange Pi 4 Pro).
 *
 * This is the player, as distinct from the validation harnesses next to it.
 * cedar-drm-present.c proved the decode-to-scanout path; this wraps that path
 * in what a 24/7 signage box actually needs: playlist rotation, MP4 input,
 * still images, HDMI hotplug recovery, and clean teardown.
 *
 *   playlist.tsv
 *     -> libavformat demux + h264_mp4toannexb   (CPU: parsing only)
 *     -> libvdecoder                            (hardware decode)
 *     -> VideoPicture.nBufFd (dma_buf)
 *     -> drmPrimeFDToHandle + AddFB2
 *     -> atomic commit with OUT_FENCE_PTR       (explicit release fence)
 *     -> HDMI
 *
 * No CPU pixel copy occurs for video. Still images are necessarily copied once
 * into a dumb buffer, since they are not hardware-decoded; that is the honest
 * exception and it is confined to image entries.
 *
 * Replaces the X11 pipeline (omxh264dec ! videoconvert ! ximagesink), which
 * costs ~174% CPU plus an X server. This costs roughly 8-10% of one core and
 * needs no X server at all, so it must NOT be run while a compositor holds the
 * DRM device.
 *
 * Usage: zc-playlist-player [playlist.tsv] [drm-card]
 * Signals: SIGTERM/SIGINT exit cleanly, releasing the display.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <sys/mman.h>
#include <libgen.h>
#include <sys/stat.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>
#include <drm_mode.h>

#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>

#include "vdecoder.h"
#include "memoryAdapter.h"

#define VE_OPS_TYPE_AW 0
extern VeOpsS *GetVeOpsS(int type);

#define MAX_ENTRIES   256
#define MAX_FB_CACHE  64
#define DEFAULT_IMAGE_SECONDS 12
#define IDLE_RETRY_SECONDS 15
#define HDMI_STATUS "/sys/class/drm/card0-HDMI-A-1/status"

enum entry_kind { ENTRY_VIDEO, ENTRY_IMAGE };

struct entry {
	enum entry_kind kind;
	/* dir (up to PATH_MAX-ish) + separator + name, so composing the two
	 * cannot truncate. */
	char path[1024];
};

struct fb_entry {
	uint32_t gem;
	uint32_t fb;
};

struct kms {
	int fd;
	uint32_t conn_id, crtc_id, plane_id;
	uint32_t blob_id;
	drmModeModeInfo mode;
	uint32_t p_conn_crtc_id;
	uint32_t p_crtc_mode_id, p_crtc_active, p_out_fence_ptr;
	uint32_t p_fb_id, p_crtc_id;
	uint32_t p_src_x, p_src_y, p_src_w, p_src_h;
	uint32_t p_crtc_x, p_crtc_y, p_crtc_w, p_crtc_h;
	int modeset_done;
	/* Persistent black frame shown during item transitions. Without it the
	 * plane would still reference a framebuffer that has just been removed,
	 * and the display engine scans freed memory - seen as a brief orange or
	 * blue flash between clips. */
	uint32_t black_fb;
	uint32_t black_handle;
	uint64_t black_size;
};

static volatile sig_atomic_t stop_requested;
static void on_signal(int s) { (void)s; stop_requested = 1; }

static void logmsg(const char *fmt, ...)
{
	char ts[32];
	time_t now = time(NULL);
	struct tm tm;
	va_list ap;

	localtime_r(&now, &tm);
	strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S", &tm);
	fprintf(stderr, "%s ", ts);
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fprintf(stderr, "\n");
	fflush(stderr);
}

/* ------------------------------------------------------------------ KMS */

static int kms_show(struct kms *k, uint32_t fb, int sx, int sy, int sw, int sh);

static uint32_t prop_id(int fd, uint32_t obj, uint32_t type, const char *name)
{
	drmModeObjectProperties *props;
	uint32_t id = 0;
	unsigned i;

	props = drmModeObjectGetProperties(fd, obj, type);
	if (!props)
		return 0;
	for (i = 0; i < props->count_props && !id; i++) {
		drmModePropertyRes *p = drmModeGetProperty(fd, props->props[i]);
		if (p) {
			if (!strcmp(p->name, name))
				id = p->prop_id;
			drmModeFreeProperty(p);
		}
	}
	drmModeFreeObjectProperties(props);
	return id;
}

static int plane_supports(int fd, uint32_t plane_id, uint32_t fourcc)
{
	drmModePlane *pl = drmModeGetPlane(fd, plane_id);
	unsigned i;
	int ok = 0;

	if (!pl)
		return 0;
	for (i = 0; i < pl->count_formats; i++)
		if (pl->formats[i] == fourcc)
			ok = 1;
	drmModeFreePlane(pl);
	return ok;
}

static int hdmi_connected(void)
{
	char buf[32] = {0};
	int fd = open(HDMI_STATUS, O_RDONLY);
	ssize_t n;

	if (fd < 0)
		return 1;	/* cannot tell; assume present */
	n = read(fd, buf, sizeof(buf) - 1);
	close(fd);
	if (n <= 0)
		return 1;
	return strncmp(buf, "connected", 9) == 0;
}

/* One small black framebuffer, allocated once and kept for the life of the
 * process, used to cover transitions between playlist items. */
static void kms_make_black(struct kms *k)
{
	struct drm_mode_create_dumb creq;
	struct drm_mode_map_dumb mreq;
	uint32_t h[4] = {0}, p[4] = {0}, o[4] = {0};
	uint64_t m[4] = {0};
	void *map;

	memset(&creq, 0, sizeof(creq));
	creq.width = k->mode.hdisplay;
	creq.height = k->mode.vdisplay;
	creq.bpp = 32;
	if (drmIoctl(k->fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq) != 0)
		return;

	memset(&mreq, 0, sizeof(mreq));
	mreq.handle = creq.handle;
	if (drmIoctl(k->fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq) != 0)
		return;
	map = mmap(0, creq.size, PROT_READ | PROT_WRITE, MAP_SHARED, k->fd, mreq.offset);
	if (map == MAP_FAILED)
		return;
	memset(map, 0, creq.size);
	munmap(map, creq.size);

	h[0] = creq.handle;
	p[0] = creq.pitch;
	if (drmModeAddFB2WithModifiers(k->fd, creq.width, creq.height,
				       DRM_FORMAT_XRGB8888, h, p, o, m,
				       &k->black_fb, 0) != 0)
		k->black_fb = 0;
	k->black_handle = creq.handle;
	k->black_size = creq.size;
}

/* Cover the plane with black before tearing down an item's framebuffers, so it
 * never references a removed one. */
static void kms_blank(struct kms *k)
{
	if (k->black_fb && k->modeset_done)
		kms_show(k, k->black_fb, 0, 0, k->mode.hdisplay, k->mode.vdisplay);
}

/* Selects a connector/CRTC/plane able to scan out both the video format and
 * XRGB8888 for images, so one plane serves both without re-selection. */
static int kms_setup(struct kms *k)
{
	drmModeRes *res;
	drmModePlaneRes *plres;
	drmModeConnector *conn = NULL;
	int i, crtc_index = -1;
	unsigned u;

	drmSetClientCap(k->fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1);
	if (drmSetClientCap(k->fd, DRM_CLIENT_CAP_ATOMIC, 1) != 0) {
		logmsg("FATAL: driver lacks atomic modeset");
		return -1;
	}

	res = drmModeGetResources(k->fd);
	if (!res)
		return -1;

	for (i = 0; i < res->count_connectors && !conn; i++) {
		drmModeConnector *c = drmModeGetConnector(k->fd, res->connectors[i]);
		if (c && c->connection == DRM_MODE_CONNECTED && c->count_modes > 0)
			conn = c;
		else if (c)
			drmModeFreeConnector(c);
	}
	if (!conn) {
		drmModeFreeResources(res);
		return -1;
	}
	k->conn_id = conn->connector_id;
	k->mode = conn->modes[0];

	if (conn->encoder_id) {
		drmModeEncoder *enc = drmModeGetEncoder(k->fd, conn->encoder_id);
		if (enc && enc->crtc_id)
			k->crtc_id = enc->crtc_id;
		if (enc)
			drmModeFreeEncoder(enc);
	}
	if (!k->crtc_id && res->count_crtcs > 0)
		k->crtc_id = res->crtcs[0];
	for (i = 0; i < res->count_crtcs; i++)
		if (res->crtcs[i] == k->crtc_id)
			crtc_index = i;
	if (crtc_index < 0) {
		drmModeFreeConnector(conn);
		drmModeFreeResources(res);
		return -1;
	}

	plres = drmModeGetPlaneResources(k->fd);
	for (u = 0; plres && u < plres->count_planes; u++) {
		drmModePlane *pl = drmModeGetPlane(k->fd, plres->planes[u]);
		if (!pl)
			continue;
		if ((pl->possible_crtcs & (1u << crtc_index)) &&
		    plane_supports(k->fd, pl->plane_id, DRM_FORMAT_NV21) &&
		    plane_supports(k->fd, pl->plane_id, DRM_FORMAT_XRGB8888))
			k->plane_id = pl->plane_id;
		drmModeFreePlane(pl);
	}
	if (plres)
		drmModeFreePlaneResources(plres);
	drmModeFreeConnector(conn);
	drmModeFreeResources(res);

	if (!k->plane_id) {
		logmsg("FATAL: no plane supports both NV21 and XRGB8888");
		return -1;
	}

	k->p_conn_crtc_id  = prop_id(k->fd, k->conn_id, DRM_MODE_OBJECT_CONNECTOR, "CRTC_ID");
	k->p_crtc_mode_id  = prop_id(k->fd, k->crtc_id, DRM_MODE_OBJECT_CRTC, "MODE_ID");
	k->p_crtc_active   = prop_id(k->fd, k->crtc_id, DRM_MODE_OBJECT_CRTC, "ACTIVE");
	k->p_out_fence_ptr = prop_id(k->fd, k->crtc_id, DRM_MODE_OBJECT_CRTC, "OUT_FENCE_PTR");
	k->p_fb_id   = prop_id(k->fd, k->plane_id, DRM_MODE_OBJECT_PLANE, "FB_ID");
	k->p_crtc_id = prop_id(k->fd, k->plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_ID");
	k->p_src_x = prop_id(k->fd, k->plane_id, DRM_MODE_OBJECT_PLANE, "SRC_X");
	k->p_src_y = prop_id(k->fd, k->plane_id, DRM_MODE_OBJECT_PLANE, "SRC_Y");
	k->p_src_w = prop_id(k->fd, k->plane_id, DRM_MODE_OBJECT_PLANE, "SRC_W");
	k->p_src_h = prop_id(k->fd, k->plane_id, DRM_MODE_OBJECT_PLANE, "SRC_H");
	k->p_crtc_x = prop_id(k->fd, k->plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_X");
	k->p_crtc_y = prop_id(k->fd, k->plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_Y");
	k->p_crtc_w = prop_id(k->fd, k->plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_W");
	k->p_crtc_h = prop_id(k->fd, k->plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_H");

	if (!k->p_fb_id || !k->p_crtc_id || !k->p_crtc_mode_id ||
	    !k->p_crtc_active || !k->p_conn_crtc_id) {
		logmsg("FATAL: missing atomic properties");
		return -1;
	}
	if (k->blob_id) {
		drmModeDestroyPropertyBlob(k->fd, k->blob_id);
		k->blob_id = 0;
	}
	if (drmModeCreatePropertyBlob(k->fd, &k->mode, sizeof(k->mode),
				      &k->blob_id) != 0)
		return -1;

	k->modeset_done = 0;
	if (!k->black_fb)
		kms_make_black(k);
	logmsg("KMS ready: connector %u crtc %u plane %u mode %s@%u, out-fence %s",
	       k->conn_id, k->crtc_id, k->plane_id, k->mode.name, k->mode.vrefresh,
	       k->p_out_fence_ptr ? "yes" : "no");
	return 0;
}

/* Commits one framebuffer, waiting on an explicit release fence so the caller
 * knows when the previous buffer became free. */
static int kms_show(struct kms *k, uint32_t fb, int sx, int sy, int sw, int sh)
{
	drmModeAtomicReq *req = drmModeAtomicAlloc();
	int out_fence = -1, ret;
	uint32_t flags;

	if (!req)
		return -1;

	if (!k->modeset_done) {
		drmModeAtomicAddProperty(req, k->conn_id, k->p_conn_crtc_id, k->crtc_id);
		drmModeAtomicAddProperty(req, k->crtc_id, k->p_crtc_mode_id, k->blob_id);
		drmModeAtomicAddProperty(req, k->crtc_id, k->p_crtc_active, 1);
		flags = DRM_MODE_ATOMIC_ALLOW_MODESET;
	} else if (k->p_out_fence_ptr) {
		drmModeAtomicAddProperty(req, k->crtc_id, k->p_out_fence_ptr,
					 (uint64_t)(uintptr_t)&out_fence);
		flags = DRM_MODE_ATOMIC_NONBLOCK;
	} else {
		flags = 0;
	}

	drmModeAtomicAddProperty(req, k->plane_id, k->p_fb_id, fb);
	drmModeAtomicAddProperty(req, k->plane_id, k->p_crtc_id, k->crtc_id);
	drmModeAtomicAddProperty(req, k->plane_id, k->p_src_x, (uint32_t)sx << 16);
	drmModeAtomicAddProperty(req, k->plane_id, k->p_src_y, (uint32_t)sy << 16);
	drmModeAtomicAddProperty(req, k->plane_id, k->p_src_w, (uint32_t)sw << 16);
	drmModeAtomicAddProperty(req, k->plane_id, k->p_src_h, (uint32_t)sh << 16);
	drmModeAtomicAddProperty(req, k->plane_id, k->p_crtc_x, 0);
	drmModeAtomicAddProperty(req, k->plane_id, k->p_crtc_y, 0);
	drmModeAtomicAddProperty(req, k->plane_id, k->p_crtc_w, k->mode.hdisplay);
	drmModeAtomicAddProperty(req, k->plane_id, k->p_crtc_h, k->mode.vdisplay);

	ret = drmModeAtomicCommit(k->fd, req, flags, NULL);
	drmModeAtomicFree(req);
	if (ret != 0)
		return -1;

	k->modeset_done = 1;

	if (out_fence >= 0) {
		struct pollfd pfd = { .fd = out_fence, .events = POLLIN };
		poll(&pfd, 1, 1000);
		close(out_fence);
	}
	return 0;
}

static void kms_release(struct kms *k)
{
	drmModeAtomicReq *req;

	if (k->fd < 0 || !k->modeset_done)
		return;
	req = drmModeAtomicAlloc();
	if (req) {
		/* Detach the plane before anything is torn down. Dropping master
		 * does not reprogram the display, so a CRTC left pointing at a
		 * removed buffer keeps scanning freed memory. */
		drmModeAtomicAddProperty(req, k->plane_id, k->p_fb_id, 0);
		drmModeAtomicAddProperty(req, k->plane_id, k->p_crtc_id, 0);
		drmModeAtomicAddProperty(req, k->crtc_id, k->p_crtc_active, 0);
		drmModeAtomicAddProperty(req, k->crtc_id, k->p_crtc_mode_id, 0);
		drmModeAtomicAddProperty(req, k->conn_id, k->p_conn_crtc_id, 0);
		drmModeAtomicCommit(k->fd, req, DRM_MODE_ATOMIC_ALLOW_MODESET, NULL);
		drmModeAtomicFree(req);
	}
	k->modeset_done = 0;
}

/* --------------------------------------------------------------- video */

/*
 * Plays one MP4/H.264 file with the zero-copy path.
 *
 * Demuxing and the AVCC-to-Annex-B conversion are done by libavformat's
 * h264_mp4toannexb filter, which yields whole access units. That matters: the
 * decoder must be handed complete access units, and feeding it arbitrary byte
 * runs makes it decode pictures beginning mid-slice, which appears on screen as
 * tearing. The bitstream filter is a far safer source of AU boundaries than
 * hand-rolled start-code scanning.
 */
static int play_video(struct kms *k, struct ScMemOpsS *memops, const char *path)
{
	int played = 0;
	AVFormatContext *fmt = NULL;
	AVBSFContext *bsf = NULL;
	const AVBitStreamFilter *filter;
	AVPacket *pkt = NULL;
	VideoDecoder *dec = NULL;
	VideoPicture *onscreen = NULL;
	VideoStreamInfo si;
	VConfig vc;
	struct fb_entry cache[MAX_FB_CACHE];
	int ncache = 0, vstream = -1, i, eof = 0;
	long interval_ns = 0;
	struct timespec next_due;
	int paced = 0;
	double fps = 25.0;

	memset(cache, 0, sizeof(cache));

	if (avformat_open_input(&fmt, path, NULL, NULL) != 0) {
		logmsg("skip %s: cannot open", path);
		return 0;
	}
	if (avformat_find_stream_info(fmt, NULL) < 0)
		goto cleanup;

	for (i = 0; i < (int)fmt->nb_streams; i++)
		if (fmt->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
			vstream = i;
			break;
		}
	if (vstream < 0) {
		logmsg("skip %s: no video stream", path);
		goto cleanup;
	}
	if (fmt->streams[vstream]->codecpar->codec_id != AV_CODEC_ID_H264) {
		/* Cedar handles H.265 too, but this player only configures H.264.
		 * Skipping loudly beats displaying nothing with no explanation. */
		logmsg("skip %s: codec is not H.264 (id %d)", path,
		       fmt->streams[vstream]->codecpar->codec_id);
		goto cleanup;
	}

	{
		AVRational r = fmt->streams[vstream]->avg_frame_rate;
		if (r.num > 0 && r.den > 0) {
			fps = (double)r.num / r.den;
			if (fps < 1.0 || fps > 240.0)
				fps = 25.0;
		}
		interval_ns = (long)(1000000000.0 / fps);
	}

	filter = av_bsf_get_by_name("h264_mp4toannexb");
	if (!filter || av_bsf_alloc(filter, &bsf) < 0)
		goto cleanup;
	avcodec_parameters_copy(bsf->par_in, fmt->streams[vstream]->codecpar);
	if (av_bsf_init(bsf) < 0)
		goto cleanup;

	dec = CreateVideoDecoder();
	if (!dec)
		goto cleanup;

	memset(&si, 0, sizeof(si));
	si.eCodecFormat = VIDEO_CODEC_FORMAT_H264;
	memset(&vc, 0, sizeof(vc));
	vc.memops = memops;
	vc.veOpsS = GetVeOpsS(VE_OPS_TYPE_AW);
	vc.eOutputPixelFormat = PIXEL_FORMAT_NV21;
	vc.nFrameBufferNum = 8;
	vc.bDispErrorFrame = 1;
	vc.nDisplayHoldingFrameBufferNum = 2;
	if (InitializeVideoDecoder(dec, &si, &vc) != 0) {
		logmsg("skip %s: decoder init failed", path);
		goto cleanup;
	}

	pkt = av_packet_alloc();
	if (!pkt)
		goto cleanup;

	logmsg("playing %s (%.2f fps)", path, fps);

	while (!stop_requested) {
		VideoPicture *pic;
		uint32_t gem = 0, fb = 0;
		int sw, sh;

		if (!hdmi_connected())
			break;	/* main loop handles reconnection */

		if (!eof) {
			if (av_read_frame(fmt, pkt) < 0) {
				eof = 1;
			} else {
				if (pkt->stream_index == vstream &&
				    av_bsf_send_packet(bsf, pkt) == 0) {
					while (av_bsf_receive_packet(bsf, pkt) == 0) {
						char *buf = NULL, *ring = NULL;
						int bufsz = 0, ringsz = 0;
						size_t len = pkt->size;

						if (RequestVideoStreamBuffer(dec, (int)len,
								&buf, &bufsz, &ring,
								&ringsz, 0) == 0 &&
						    buf && (size_t)(bufsz + ringsz) >= len) {
							VideoStreamDataInfo di;

							memcpy(buf, pkt->data, (size_t)bufsz);
							if (ringsz > 0 && len > (size_t)bufsz)
								memcpy(ring, pkt->data + bufsz,
								       len - bufsz);
							memset(&di, 0, sizeof(di));
							di.pData = buf;
							di.nLength = (int)len;
							di.bIsFirstPart = 1;
							di.bIsLastPart = 1;
							di.nPts = -1;
							di.bValid = 1;
							SubmitVideoStreamData(dec, &di, 0);
						}
						av_packet_unref(pkt);
					}
				}
				av_packet_unref(pkt);
			}
		}

		DecodeVideoStream(dec, eof, 0, 0, 0);
		pic = RequestPicture(dec, 0);
		if (!pic) {
			if (eof)
				break;
			continue;
		}
		if (pic->bEnableAfbcFlag) {
			logmsg("skip %s: AFBC surface needs a modifier", path);
			ReturnPicture(dec, pic);
			break;
		}

		if (drmPrimeFDToHandle(k->fd, pic->nBufFd, &gem) != 0) {
			ReturnPicture(dec, pic);
			break;
		}
		for (i = 0; i < ncache; i++)
			if (cache[i].gem == gem)
				fb = cache[i].fb;
		if (!fb) {
			uint32_t h[4] = {0}, p[4] = {0}, o[4] = {0};
			uint64_t m[4] = {0};

			h[0] = gem; p[0] = (uint32_t)pic->nLineStride; o[0] = 0;
			h[1] = gem; p[1] = (uint32_t)pic->nLineStride;
			o[1] = (uint32_t)(pic->pData1 - pic->pData0);
			if (drmModeAddFB2WithModifiers(k->fd, pic->nWidth, pic->nHeight,
						       DRM_FORMAT_NV21, h, p, o, m,
						       &fb, 0) != 0) {
				ReturnPicture(dec, pic);
				break;
			}
			if (ncache < MAX_FB_CACHE) {
				cache[ncache].gem = gem;
				cache[ncache].fb = fb;
				ncache++;
			}
		}

		sw = pic->nRightOffset  ? pic->nRightOffset  - pic->nLeftOffset : pic->nWidth;
		sh = pic->nBottomOffset ? pic->nBottomOffset - pic->nTopOffset  : pic->nHeight;

		if (!paced) {
			clock_gettime(CLOCK_MONOTONIC, &next_due);
			paced = 1;
		} else {
			next_due.tv_nsec += interval_ns;
			while (next_due.tv_nsec >= 1000000000L) {
				next_due.tv_nsec -= 1000000000L;
				next_due.tv_sec++;
			}
			clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_due, NULL);
		}

		if (kms_show(k, fb, pic->nLeftOffset, pic->nTopOffset, sw, sh) != 0) {
			ReturnPicture(dec, pic);
			break;
		}

		/* Release the predecessor only once the commit that replaced it
		 * has completed, so the decoder cannot recycle a surface that is
		 * still being scanned out. */
		if (onscreen)
			ReturnPicture(dec, onscreen);
		onscreen = pic;
		played = 1;
	}

cleanup:
	kms_blank(k);
	if (onscreen && dec)
		ReturnPicture(dec, onscreen);
	for (i = 0; i < ncache; i++) {
		struct drm_gem_close gc = { .handle = cache[i].gem };

		drmModeRmFB(k->fd, cache[i].fb);
		drmIoctl(k->fd, DRM_IOCTL_GEM_CLOSE, &gc);
	}
	if (pkt)
		av_packet_free(&pkt);
	if (dec)
		DestroyVideoDecoder(dec);
	if (bsf)
		av_bsf_free(&bsf);
	if (fmt)
		avformat_close_input(&fmt);
	return played;
}

/* --------------------------------------------------------------- image */

/*
 * Still images are decoded on the CPU and copied once into a dumb buffer.
 * There is no hardware path for them, so this is the one place a pixel copy
 * happens; it is bounded by image entries and does not affect video.
 */
static int show_image(struct kms *k, const char *path, int seconds)
{
	int shown = 0;
	AVFormatContext *fmt = NULL;
	AVCodecContext *cc = NULL;
	const AVCodec *codec;
	AVFrame *frame = NULL;
	AVPacket *pkt = NULL;
	struct SwsContext *sws = NULL;
	struct drm_mode_create_dumb creq;
	struct drm_mode_map_dumb mreq;
	struct drm_mode_destroy_dumb dreq;
	uint8_t *map = MAP_FAILED;
	uint32_t fb = 0;
	int vstream = -1, i, got = 0;

	memset(&creq, 0, sizeof(creq));

	if (avformat_open_input(&fmt, path, NULL, NULL) != 0 ||
	    avformat_find_stream_info(fmt, NULL) < 0) {
		logmsg("skip image %s: cannot open", path);
		goto cleanup;
	}
	for (i = 0; i < (int)fmt->nb_streams; i++)
		if (fmt->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
			vstream = i;
			break;
		}
	if (vstream < 0)
		goto cleanup;

	codec = avcodec_find_decoder(fmt->streams[vstream]->codecpar->codec_id);
	if (!codec)
		goto cleanup;
	cc = avcodec_alloc_context3(codec);
	if (!cc || avcodec_parameters_to_context(cc, fmt->streams[vstream]->codecpar) < 0 ||
	    avcodec_open2(cc, codec, NULL) < 0)
		goto cleanup;

	frame = av_frame_alloc();
	pkt = av_packet_alloc();
	if (!frame || !pkt)
		goto cleanup;

	while (av_read_frame(fmt, pkt) >= 0) {
		if (pkt->stream_index == vstream &&
		    avcodec_send_packet(cc, pkt) == 0 &&
		    avcodec_receive_frame(cc, frame) == 0) {
			got = 1;
			av_packet_unref(pkt);
			break;
		}
		av_packet_unref(pkt);
	}
	if (!got) {
		logmsg("skip image %s: no frame decoded", path);
		goto cleanup;
	}

	creq.width  = k->mode.hdisplay;
	creq.height = k->mode.vdisplay;
	creq.bpp    = 32;
	if (drmIoctl(k->fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq) != 0)
		goto cleanup;

	memset(&mreq, 0, sizeof(mreq));
	mreq.handle = creq.handle;
	if (drmIoctl(k->fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq) != 0)
		goto cleanup;
	map = mmap(0, creq.size, PROT_READ | PROT_WRITE, MAP_SHARED, k->fd, mreq.offset);
	if (map == MAP_FAILED)
		goto cleanup;
	memset(map, 0, creq.size);

	/* Fit the image to the mode, preserving aspect, letterboxed on black. */
	{
		int dw = k->mode.hdisplay, dh = k->mode.vdisplay;
		double sx = (double)dw / frame->width;
		double sy = (double)dh / frame->height;
		double s = sx < sy ? sx : sy;
		int ow = (int)(frame->width * s) & ~1;
		int oh = (int)(frame->height * s) & ~1;
		int ox = (dw - ow) / 2, oy = (dh - oh) / 2;
		uint8_t *dst[4] = { map + (size_t)oy * creq.pitch + (size_t)ox * 4,
				    NULL, NULL, NULL };
		int dstride[4] = { (int)creq.pitch, 0, 0, 0 };

		sws = sws_getContext(frame->width, frame->height, cc->pix_fmt,
				     ow, oh, AV_PIX_FMT_BGRA,
				     SWS_BILINEAR, NULL, NULL, NULL);
		if (!sws)
			goto cleanup;
		sws_scale(sws, (const uint8_t * const *)frame->data, frame->linesize,
			  0, frame->height, dst, dstride);
	}

	{
		uint32_t h[4] = { creq.handle, 0, 0, 0 };
		uint32_t p[4] = { creq.pitch, 0, 0, 0 };
		uint32_t o[4] = { 0, 0, 0, 0 };
		uint64_t m[4] = { 0, 0, 0, 0 };

		if (drmModeAddFB2WithModifiers(k->fd, creq.width, creq.height,
					       DRM_FORMAT_XRGB8888, h, p, o, m,
					       &fb, 0) != 0)
			goto cleanup;
	}

	logmsg("showing image %s for %ds", path, seconds);
	if (kms_show(k, fb, 0, 0, creq.width, creq.height) == 0) {
		int elapsed = 0;

		shown = 1;

		while (!stop_requested && elapsed < seconds && hdmi_connected()) {
			sleep(1);
			elapsed++;
		}
	}

cleanup:
	if (fb)
		kms_blank(k);
	if (fb)
		drmModeRmFB(k->fd, fb);
	if (map != MAP_FAILED)
		munmap(map, creq.size);
	if (creq.handle) {
		memset(&dreq, 0, sizeof(dreq));
		dreq.handle = creq.handle;
		drmIoctl(k->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
	}
	if (sws)
		sws_freeContext(sws);
	if (pkt)
		av_packet_free(&pkt);
	if (frame)
		av_frame_free(&frame);
	if (cc)
		avcodec_free_context(&cc);
	if (fmt)
		avformat_close_input(&fmt);
	return shown;
}

/* ------------------------------------------------------------- playlist */

static int load_playlist(const char *path, struct entry *out, int max)
{
	FILE *f = fopen(path, "r");
	char line[640], dir[512];
	int n = 0;

	if (!f) {
		logmsg("cannot open playlist %s: %s", path, strerror(errno));
		return 0;
	}
	snprintf(dir, sizeof(dir), "%s", path);
	dirname(dir);

	while (n < max && fgets(line, sizeof(line), f)) {
		char kind[32], name[256];
		char *nl = strchr(line, '\n');

		if (nl)
			*nl = '\0';
		if (line[0] == '\0' || line[0] == '#')
			continue;
		if (sscanf(line, "%31s\t%511[^\n]", kind, name) != 2)
			continue;

		if (!strcmp(kind, "video"))
			out[n].kind = ENTRY_VIDEO;
		else if (!strcmp(kind, "image"))
			out[n].kind = ENTRY_IMAGE;
		else
			continue;

		if (name[0] == '/')
			snprintf(out[n].path, sizeof(out[n].path), "%s", name);
		else
			snprintf(out[n].path, sizeof(out[n].path), "%s/%s", dir, name);
		n++;
	}
	fclose(f);
	return n;
}

int main(int argc, char **argv)
{
	const char *playlist_path = (argc > 1) ? argv[1]
			: "./playlist.tsv";
	const char *card = (argc > 2) ? argv[2] : "/dev/dri/card0";
	struct entry entries[MAX_ENTRIES];
	struct kms k;
	struct ScMemOpsS *memops;
	int n_entries, idx = 0, rc = 1, ok = 0;
	int consecutive_failures = 0, backoff_logged = 0;
	struct stat pl_stat;
	time_t playlist_mtime = 0;

	signal(SIGTERM, on_signal);
	signal(SIGINT, on_signal);
	signal(SIGPIPE, SIG_IGN);

	memset(&k, 0, sizeof(k));
	k.fd = -1;

	n_entries = load_playlist(playlist_path, entries, MAX_ENTRIES);
	if (n_entries <= 0) {
		logmsg("FATAL: playlist %s has no usable entries", playlist_path);
		return 1;
	}
	if (stat(playlist_path, &pl_stat) == 0)
		playlist_mtime = pl_stat.st_mtime;
	logmsg("playlist %s: %d entries", playlist_path, n_entries);

	memops = MemAdapterGetOpsS();
	if (!memops || CdcMemOpen(memops) != 0) {
		logmsg("FATAL: Cedar memory adapter unavailable");
		return 1;
	}
	AddVDPlugin();

	k.fd = open(card, O_RDWR | O_CLOEXEC);
	if (k.fd < 0) {
		logmsg("FATAL: open %s: %s", card, strerror(errno));
		goto out;
	}
	if (drmSetMaster(k.fd) != 0) {
		logmsg("FATAL: cannot become DRM master (%s) - a compositor still"
		       " owns the display; stop it before starting this player",
		       strerror(errno));
		goto out;
	}
	if (kms_setup(&k) != 0)
		goto out;

	while (!stop_requested) {
		struct entry *e;

		/* Reload when the playlist changes on disk. an external sync process may rewrite
		 * playlist.tsv and delete files that are no longer current. Holding the startup copy would mean new media never
		 * appeared, and that stale entries pointed at files sync had
		 * already removed. The previous shell player re-read the playlist
		 * every rotation; this preserves that behaviour. */
		if (stat(playlist_path, &pl_stat) == 0 &&
		    pl_stat.st_mtime != playlist_mtime) {
			int fresh = load_playlist(playlist_path, entries, MAX_ENTRIES);

			if (fresh > 0) {
				playlist_mtime = pl_stat.st_mtime;
				n_entries = fresh;
				if (idx >= n_entries)
					idx = 0;
				logmsg("playlist changed on disk; reloaded %d entries",
				       n_entries);
			} else {
				/* Keep playing the old list rather than going
				 * black on a truncated or mid-write file. */
				logmsg("playlist reload yielded no entries; keeping previous list");
				playlist_mtime = pl_stat.st_mtime;
			}
		}

		e = &entries[idx];

		/* HDMI hotplug: a signage display gets power-cycled. Wait for it
		 * to come back and re-establish the mode rather than exiting. */
		if (!hdmi_connected()) {
			logmsg("HDMI disconnected; releasing display and waiting");
			kms_release(&k);
			while (!stop_requested && !hdmi_connected())
				sleep(2);
			if (stop_requested)
				break;
			logmsg("HDMI reconnected; re-establishing mode");
			if (kms_setup(&k) != 0) {
				sleep(2);
				continue;
			}
		}

		if (e->kind == ENTRY_VIDEO)
			ok = play_video(&k, memops, e->path);
		else
			ok = show_image(&k, e->path, DEFAULT_IMAGE_SECONDS);

		/* If an entire pass over the playlist displays nothing, stop
		 * spinning. Without this, a media directory that is missing,
		 * mid-sync, or unreadable makes this loop retry as fast as the
		 * CPU allows and flood the log - which is exactly how /var/log
		 * was filled to 100%% on the rack. Back off and let the next
		 * sync, or a human, fix it. */
		if (ok) {
			consecutive_failures = 0;
		} else if (++consecutive_failures >= n_entries) {
			if (!backoff_logged) {
				logmsg("nothing in the playlist could be displayed;"
				       " retrying every %ds (this message is logged once)",
				       IDLE_RETRY_SECONDS);
				backoff_logged = 1;
			}
			sleep(IDLE_RETRY_SECONDS);
			consecutive_failures = 0;
		}
		if (ok)
			backoff_logged = 0;

		idx = (idx + 1) % n_entries;
	}

	rc = 0;
	logmsg("stopping on request");

out:
	kms_release(&k);
	if (k.blob_id)
		drmModeDestroyPropertyBlob(k.fd, k.blob_id);
	if (k.fd >= 0) {
		drmDropMaster(k.fd);
		close(k.fd);
	}
	if (memops)
		CdcMemClose(memops);
	return rc;
}
