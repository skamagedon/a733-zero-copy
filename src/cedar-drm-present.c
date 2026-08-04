/*
 * cedar-drm-present.c
 *
 * Purpose
 * -------
 * The scanout gate. cedar-dmabuf-drm-probe.c proved that a Cedar decoded frame
 * can be imported by DRM as a framebuffer; this program proves the rest: that
 * those frames can actually be driven to HDMI, continuously, with correct
 * buffer recycling and no CPU pixel copy anywhere in the loop.
 *
 *   H.264 Annex-B -> libvdecoder -> VideoPicture.nBufFd (dma_buf)
 *                 -> drmPrimeFDToHandle -> drmModeAddFB2WithModifiers
 *                 -> atomic commit on a plane -> HDMI
 *                 -> ReturnPicture (only after the frame is off screen)
 *
 * Buffer lifetime is the part that is easy to get wrong. The decoder recycles a
 * small pool of surfaces, so a picture must not be returned while it is still
 * being scanned out.
 *
 * Two synchronisation modes are implemented, selectable so they can be compared:
 *
 *   fence (default) - the commit carries CRTC OUT_FENCE_PTR, and the kernel
 *     returns a sync_file descriptor that signals when that commit completed.
 *     Waiting on it is an explicit release fence: it states directly that the
 *     previous surface is free, rather than inferring it. Commits are issued
 *     non-blocking, so the fence is the only serialisation point.
 *
 *   event - the commit carries PAGE_FLIP_EVENT and the loop waits for the event
 *     before releasing the predecessor. Correct here, but it infers buffer
 *     lifetime from a display notification instead of stating it.
 *
 * There is deliberately no acquire fence. The Cedar ABI exposes none - no field
 * in VideoPicture, and no fence symbol anywhere in its headers - and none is
 * needed, because RequestPicture() returns only completed pictures, so decode
 * completion is already signalled by the call returning. The plane does carry
 * IN_FENCE_FD, so the display side could consume a producer fence if the
 * decoder ever exposed one.
 *
 * GEM handles are cached per imported buffer rather than created and closed per
 * frame: drmPrimeFDToHandle returns the same handle for the same underlying
 * dma_buf, so closing it per frame would tear down a mapping another
 * framebuffer still references.
 *
 * Requires DRM master, so no other compositor may hold the device.
 *
 * Usage: cedar-drm-present <file.h264> [seconds] [drm-card] [fps] [loop] [fence]
 *        fence: 1 = explicit OUT_FENCE_PTR release fence (default), 0 = page-flip event
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
#include <dirent.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

#include "vdecoder.h"
#include "memoryAdapter.h"

/* Exported by libVE.so but absent from veInterface.h. Dispatches on engine
 * type: 0 = getVeAwOpsS (H.264/H.265), 1 = getVeVp9OpsS. */
#define VE_OPS_TYPE_AW 0
extern VeOpsS *GetVeOpsS(int type);

#define CHUNK_SIZE (512 * 1024)
#define MAX_FB_CACHE 64

struct fb_entry {
	uint32_t gem;
	uint32_t fb;
};

struct kms {
	int fd;
	uint32_t conn_id, crtc_id, plane_id;
	uint32_t blob_id;
	drmModeModeInfo mode;
	/* property ids */
	uint32_t p_conn_crtc_id;
	uint32_t p_crtc_mode_id, p_crtc_active, p_out_fence_ptr;
	uint32_t p_fb_id, p_crtc_id;
	uint32_t p_src_x, p_src_y, p_src_w, p_src_h;
	uint32_t p_crtc_x, p_crtc_y, p_crtc_w, p_crtc_h;
};

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

/* Pick a connected connector, its CRTC, and a plane on that CRTC that can scan
 * out the decoder's format. An overlay plane is preferred so the primary plane
 * is left alone, but the primary is acceptable. */
static int kms_setup(struct kms *k, uint32_t fourcc)
{
	drmModeRes *res;
	drmModePlaneRes *plres;
	drmModeConnector *conn = NULL;
	int i, crtc_index = -1;
	unsigned u;

	drmSetClientCap(k->fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1);
	if (drmSetClientCap(k->fd, DRM_CLIENT_CAP_ATOMIC, 1) != 0) {
		fprintf(stderr, "FAIL: driver does not support atomic modeset\n");
		return -1;
	}

	res = drmModeGetResources(k->fd);
	if (!res) {
		fprintf(stderr, "FAIL: drmModeGetResources: %s\n", strerror(errno));
		return -1;
	}

	for (i = 0; i < res->count_connectors && !conn; i++) {
		drmModeConnector *c = drmModeGetConnector(k->fd, res->connectors[i]);
		if (c && c->connection == DRM_MODE_CONNECTED && c->count_modes > 0)
			conn = c;
		else if (c)
			drmModeFreeConnector(c);
	}
	if (!conn) {
		fprintf(stderr, "FAIL: no connected connector with modes\n");
		drmModeFreeResources(res);
		return -1;
	}
	k->conn_id = conn->connector_id;
	k->mode = conn->modes[0];	/* index 0 is the preferred mode */

	/* Use the encoder's current CRTC when there is one, else the first. */
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
		fprintf(stderr, "FAIL: could not resolve a CRTC\n");
		drmModeFreeConnector(conn);
		drmModeFreeResources(res);
		return -1;
	}

	plres = drmModeGetPlaneResources(k->fd);
	if (!plres) {
		fprintf(stderr, "FAIL: drmModeGetPlaneResources\n");
		drmModeFreeConnector(conn);
		drmModeFreeResources(res);
		return -1;
	}
	for (u = 0; u < plres->count_planes; u++) {
		drmModePlane *pl = drmModeGetPlane(k->fd, plres->planes[u]);
		if (!pl)
			continue;
		if ((pl->possible_crtcs & (1u << crtc_index)) &&
		    plane_supports(k->fd, pl->plane_id, fourcc)) {
			uint32_t t = prop_id(k->fd, pl->plane_id,
					     DRM_MODE_OBJECT_PLANE, "type");
			/* Prefer a non-primary plane, but take what is offered. */
			if (!k->plane_id || t)
				k->plane_id = pl->plane_id;
		}
		drmModeFreePlane(pl);
	}
	drmModeFreePlaneResources(plres);
	drmModeFreeConnector(conn);
	drmModeFreeResources(res);

	if (!k->plane_id) {
		fprintf(stderr, "FAIL: no plane on this CRTC supports %.4s\n",
			(char *)&fourcc);
		return -1;
	}

	k->p_conn_crtc_id = prop_id(k->fd, k->conn_id, DRM_MODE_OBJECT_CONNECTOR, "CRTC_ID");
	k->p_crtc_mode_id = prop_id(k->fd, k->crtc_id, DRM_MODE_OBJECT_CRTC, "MODE_ID");
	k->p_crtc_active  = prop_id(k->fd, k->crtc_id, DRM_MODE_OBJECT_CRTC, "ACTIVE");
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

	if (!k->p_fb_id || !k->p_crtc_id || !k->p_src_w || !k->p_crtc_w ||
	    !k->p_crtc_mode_id || !k->p_crtc_active || !k->p_conn_crtc_id) {
		fprintf(stderr, "FAIL: missing atomic properties\n");
		return -1;
	}

	if (drmModeCreatePropertyBlob(k->fd, &k->mode, sizeof(k->mode),
				      &k->blob_id) != 0) {
		fprintf(stderr, "FAIL: create mode blob: %s\n", strerror(errno));
		return -1;
	}

	printf("KMS: connector %u, crtc %u, plane %u, mode %s@%u\n",
	       k->conn_id, k->crtc_id, k->plane_id, k->mode.name,
	       k->mode.vrefresh);
	printf("     OUT_FENCE_PTR: %s\n",
	       k->p_out_fence_ptr ? "available" : "NOT available");
	return 0;
}

/* Find the next Annex-B start code at or after `i`. Returns its offset and sets
 * *sclen to 3 or 4, or returns n when there is none. */
static size_t next_start_code(const uint8_t *b, size_t n, size_t i, int *sclen)
{
	for (; i + 3 <= n; i++) {
		if (b[i] == 0 && b[i + 1] == 0) {
			if (b[i + 2] == 1) {
				*sclen = 3;
				return i;
			}
			if (i + 4 <= n && b[i + 2] == 0 && b[i + 3] == 1) {
				*sclen = 4;
				return i;
			}
		}
	}
	return n;
}

/*
 * Return the end offset of the access unit beginning at `start`.
 *
 * The decoder must be handed whole access units. Submitting arbitrary byte
 * chunks and flagging each as a complete frame makes it see slices that do not
 * begin a picture, which it reports as "the first slice of the frame is not 0"
 * and which shows on screen as tearing and artifacts.
 *
 * A new access unit starts at an access-unit delimiter, at a parameter set, or
 * at a VCL NAL whose first_mb_in_slice is 0. first_mb_in_slice is the leading
 * ue(v) of the slice header, so a set top bit means the value is 0.
 */
static size_t au_end(const uint8_t *b, size_t n, size_t start)
{
	size_t i = start;
	int first = 1;

	while (i < n) {
		int sclen = 0;
		size_t sc = next_start_code(b, n, i, &sclen);
		unsigned nal_type;

		if (sc >= n || sc + sclen + 1 >= n)
			return n;
		nal_type = b[sc + sclen] & 0x1f;

		if (!first) {
			if (nal_type == 9 || nal_type == 7 || nal_type == 8)
				return sc;
			if ((nal_type == 1 || nal_type == 5) &&
			    (b[sc + sclen + 1] & 0x80))
				return sc;
		}
		first = 0;
		i = sc + sclen + 1;
	}
	return n;
}

/* Number of descriptors this process holds. A zero-copy pipeline leaks here
 * first: every decoded surface arrives as a dma_buf fd, so a missing release
 * shows up as monotonic growth long before memory pressure appears. */
static int count_open_fds(void)
{
	DIR *d = opendir("/proc/self/fd");
	struct dirent *e;
	int n = 0;

	if (!d)
		return -1;
	while ((e = readdir(d)))
		if (e->d_name[0] != '.')
			n++;
	closedir(d);
	return n;
}

static long rss_kb(void)
{
	FILE *f = fopen("/proc/self/status", "r");
	char line[256];
	long kb = -1;

	if (!f)
		return -1;
	while (fgets(line, sizeof(line), f))
		if (sscanf(line, "VmRSS: %ld kB", &kb) == 1)
			break;
	fclose(f);
	return kb;
}

static void flip_handler(int fd, unsigned seq, unsigned tv_sec,
			 unsigned tv_usec, void *data)
{
	(void)fd; (void)seq; (void)tv_sec; (void)tv_usec;
	*(int *)data = 1;
}

int main(int argc, char **argv)
{
	const char *path = (argc > 1) ? argv[1] : NULL;
	int seconds = (argc > 2) ? atoi(argv[2]) : 10;
	const char *card = (argc > 3) ? argv[3] : "/dev/dri/card0";
	struct ScMemOpsS *memops = NULL;
	VideoDecoder *dec = NULL;
	VideoStreamInfo si;
	VConfig vc;
	VideoPicture *pic = NULL, *onscreen = NULL;
	struct kms k;
	struct fb_entry cache[MAX_FB_CACHE];
	int ncache = 0;
	FILE *f = NULL;
	uint8_t *es = NULL;
	size_t es_size = 0, es_pos = 0;
	long long au_index = 0;
	int rc = 1, eos = 0, mem_open = 0, first = 1;
	int frames = 0, imports = 0;
	uint32_t fourcc = DRM_FORMAT_NV21;
	struct timespec t0, now, next_due;
	drmEventContext evctx;
	double fps = (argc > 4) ? atof(argv[4]) : 0.0;
	int loop_input = (argc > 5) ? atoi(argv[5]) : 0;
	int use_fence = (argc > 6) ? atoi(argv[6]) : 1;
	long fence_waits = 0, fence_timeouts = 0, fence_missing = 0;
	int loops = 0;
	time_t last_report = 0;
	const long report_every = 60;
	long interval_ns = 0;

	if (!path) {
		fprintf(stderr, "usage: %s <file.h264> [seconds] [drm-card] [fps] [loop] [fence]\n", argv[0]);
		return 1;
	}

	memset(&k, 0, sizeof(k));
	k.fd = -1;
	memset(cache, 0, sizeof(cache));
	memset(&evctx, 0, sizeof(evctx));
	evctx.version = 2;
	evctx.page_flip_handler = flip_handler;

	f = fopen(path, "rb");
	if (!f) {
		fprintf(stderr, "FAIL: open %s: %s\n", path, strerror(errno));
		return 1;
	}
	/* The whole elementary stream is held in memory so access units can be
	 * delimited without straddling read boundaries. */
	fseek(f, 0, SEEK_END);
	es_size = (size_t)ftell(f);
	fseek(f, 0, SEEK_SET);
	es = malloc(es_size);
	if (!es || fread(es, 1, es_size, f) != es_size) {
		fprintf(stderr, "FAIL: could not read %s\n", path);
		goto out;
	}

	memops = MemAdapterGetOpsS();
	if (!memops || CdcMemOpen(memops) != 0) {
		fprintf(stderr, "FAIL: Cedar memory adapter\n");
		goto out;
	}
	mem_open = 1;

	AddVDPlugin();
	dec = CreateVideoDecoder();
	if (!dec) {
		fprintf(stderr, "FAIL: CreateVideoDecoder\n");
		goto out;
	}

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
		fprintf(stderr, "FAIL: InitializeVideoDecoder\n");
		goto out;
	}

	k.fd = open(card, O_RDWR | O_CLOEXEC);
	if (k.fd < 0) {
		fprintf(stderr, "FAIL: open %s: %s\n", card, strerror(errno));
		goto out;
	}
	if (drmSetMaster(k.fd) != 0) {
		fprintf(stderr,
			"FAIL: cannot become DRM master (%s).\n"
			"      Another compositor owns the display; stop it first.\n",
			strerror(errno));
		goto out;
	}
	if (kms_setup(&k, fourcc) != 0)
		goto out;

	clock_gettime(CLOCK_MONOTONIC, &t0);
	last_report = t0.tv_sec;

	while (1) {
		drmModeAtomicReq *req;
		uint32_t gem = 0, fb = 0;
		int i, flipped = 0, src_w, src_h;
		int out_fence = -1;

		clock_gettime(CLOCK_MONOTONIC, &now);
		if (now.tv_sec - t0.tv_sec >= seconds)
			break;

		/* Feed one whole access unit per submission. The stream is never
		 * rewound: seeking to offset 0 would splice mid-frame. Supply a
		 * clip long enough for the requested duration instead. */
		if (!eos) {
			/* Looping back to offset 0 is safe now that submissions are
			 * access-unit aligned: the start of the file is an AU
			 * boundary (SPS/PPS/IDR), so the decoder resumes from a
			 * clean random-access point. The earlier corruption came
			 * from rewinding mid-access-unit, not from looping itself. */
			if (es_pos >= es_size && loop_input) {
				es_pos = 0;
				loops++;
			}
			if (es_pos >= es_size) {
				eos = 1;
			} else {
				size_t end = au_end(es, es_size, es_pos);
				size_t len = end - es_pos;
				char *buf = NULL, *ring = NULL;
				int bufsz = 0, ringsz = 0;

				if (RequestVideoStreamBuffer(dec, (int)len, &buf,
							     &bufsz, &ring,
							     &ringsz, 0) == 0 &&
				    buf && (size_t)(bufsz + ringsz) >= len) {
					VideoStreamDataInfo di;

					memcpy(buf, es + es_pos, (size_t)bufsz);
					if (ringsz > 0 && len > (size_t)bufsz)
						memcpy(ring, es + es_pos + bufsz,
						       len - bufsz);
					memset(&di, 0, sizeof(di));
					di.pData = buf;
					di.nLength = (int)len;
					di.bIsFirstPart = 1;
					di.bIsLastPart = 1;
					di.nPts = au_index * 1000000LL / 30;
					di.bValid = 1;
					SubmitVideoStreamData(dec, &di, 0);
					es_pos = end;
					au_index++;
				}
			}
		}

		DecodeVideoStream(dec, eos, 0, 0, 0);
		pic = RequestPicture(dec, 0);
		if (!pic) {
			if (eos)
				break;	/* stream drained */
			continue;
		}

		if (pic->bEnableAfbcFlag) {
			fprintf(stderr, "FAIL: AFBC surface needs a modifier\n");
			goto out;
		}

		if (drmPrimeFDToHandle(k.fd, pic->nBufFd, &gem) != 0) {
			fprintf(stderr, "FAIL: PRIME import: %s\n", strerror(errno));
			goto out;
		}

		/* Reuse the framebuffer for a surface already seen. The decoder
		 * cycles a fixed pool, so this settles at pool size. */
		for (i = 0; i < ncache; i++)
			if (cache[i].gem == gem)
				fb = cache[i].fb;

		if (!fb) {
			uint32_t handles[4] = {0}, pitches[4] = {0};
			uint32_t offsets[4] = {0};
			uint64_t mods[4] = {0};

			handles[0] = gem;
			pitches[0] = (uint32_t)pic->nLineStride;
			offsets[0] = 0;
			handles[1] = gem;
			pitches[1] = (uint32_t)pic->nLineStride;
			offsets[1] = (uint32_t)(pic->pData1 - pic->pData0);

			if (drmModeAddFB2WithModifiers(k.fd, pic->nWidth,
						       pic->nHeight, fourcc,
						       handles, pitches, offsets,
						       mods, &fb, 0) != 0) {
				fprintf(stderr, "FAIL: AddFB2: %s\n",
					strerror(errno));
				goto out;
			}
			if (ncache < MAX_FB_CACHE) {
				cache[ncache].gem = gem;
				cache[ncache].fb = fb;
				ncache++;
			}
			imports++;
		}

		/* nBottomOffset/nRightOffset are absolute crop edges, not insets,
		 * so the padded 1088 height is cropped back to 1080 here. */
		src_w = pic->nRightOffset  ? pic->nRightOffset  - pic->nLeftOffset
					   : pic->nWidth;
		src_h = pic->nBottomOffset ? pic->nBottomOffset - pic->nTopOffset
					   : pic->nHeight;

		/* Pace to the source frame rate. Without this each decoded frame
		 * occupies exactly one vblank, so a 30 fps clip plays at the
		 * 60 Hz refresh - double speed, which reads as judder. The
		 * decoder reports nFrameRate scaled by 1000 on this platform. */
		if (interval_ns == 0) {
			if (fps <= 0.0 && pic->nFrameRate > 1000)
				fps = pic->nFrameRate / 1000.0;
			if (fps <= 0.0 || fps > 240.0)
				fps = 30.0;
			interval_ns = (long)(1000000000.0 / fps);
			printf("Pacing to %.2f fps (%ld ns/frame)\n", fps, interval_ns);
			clock_gettime(CLOCK_MONOTONIC, &next_due);
		} else {
			next_due.tv_nsec += interval_ns;
			while (next_due.tv_nsec >= 1000000000L) {
				next_due.tv_nsec -= 1000000000L;
				next_due.tv_sec++;
			}
			clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_due, NULL);
		}

		req = drmModeAtomicAlloc();
		if (!req)
			goto out;

		if (first) {
			drmModeAtomicAddProperty(req, k.conn_id, k.p_conn_crtc_id, k.crtc_id);
			drmModeAtomicAddProperty(req, k.crtc_id, k.p_crtc_mode_id, k.blob_id);
			drmModeAtomicAddProperty(req, k.crtc_id, k.p_crtc_active, 1);
		}
		drmModeAtomicAddProperty(req, k.plane_id, k.p_fb_id, fb);
		drmModeAtomicAddProperty(req, k.plane_id, k.p_crtc_id, k.crtc_id);
		drmModeAtomicAddProperty(req, k.plane_id, k.p_src_x, pic->nLeftOffset << 16);
		drmModeAtomicAddProperty(req, k.plane_id, k.p_src_y, pic->nTopOffset << 16);
		drmModeAtomicAddProperty(req, k.plane_id, k.p_src_w, (uint32_t)src_w << 16);
		drmModeAtomicAddProperty(req, k.plane_id, k.p_src_h, (uint32_t)src_h << 16);
		drmModeAtomicAddProperty(req, k.plane_id, k.p_crtc_x, 0);
		drmModeAtomicAddProperty(req, k.plane_id, k.p_crtc_y, 0);
		drmModeAtomicAddProperty(req, k.plane_id, k.p_crtc_w, k.mode.hdisplay);
		drmModeAtomicAddProperty(req, k.plane_id, k.p_crtc_h, k.mode.vdisplay);

		{
			uint32_t flags;
			int ret;

			if (first) {
				/* The initial modeset commits blocking: a modeset
				 * cannot be combined with NONBLOCK, and there is
				 * no predecessor to release yet. */
				flags = DRM_MODE_ATOMIC_ALLOW_MODESET;
			} else if (use_fence) {
				/* Explicit release fencing. The kernel writes a
				 * sync_file descriptor here that signals when this
				 * commit completes, which is exactly when the
				 * previously scanned-out surface becomes free.
				 * That is a real release fence rather than
				 * inferring the same moment from an event. */
				out_fence = -1;
				drmModeAtomicAddProperty(req, k.crtc_id,
							 k.p_out_fence_ptr,
							 (uint64_t)(uintptr_t)&out_fence);
				flags = DRM_MODE_ATOMIC_NONBLOCK;
			} else {
				flags = DRM_MODE_PAGE_FLIP_EVENT;
			}

			ret = drmModeAtomicCommit(k.fd, req, flags, &flipped);
			drmModeAtomicFree(req);
			if (ret != 0) {
				fprintf(stderr,
					"FAIL: atomic commit: %s (frame %d, %dx%d src %dx%d)\n",
					strerror(errno), frames, pic->nWidth,
					pic->nHeight, src_w, src_h);
				goto out;
			}
		}

		/* Do not release the previous picture until this commit is known
		 * complete; only then is that surface guaranteed off screen. */
		if (first) {
			/* No predecessor to release. */
		} else if (use_fence) {
			if (out_fence >= 0) {
				struct pollfd pfd = { .fd = out_fence,
						      .events = POLLIN };

				if (poll(&pfd, 1, 1000) <= 0)
					fence_timeouts++;
				close(out_fence);
				out_fence = -1;
				fence_waits++;
			} else {
				/* The kernel accepted the property but returned no
				 * fence. Counting this matters: continuing quietly
				 * would recycle buffers with no completion signal
				 * at all, which is worse than the event path. */
				fence_missing++;
			}
		} else {
			while (!flipped) {
				struct pollfd pfd = { .fd = k.fd, .events = POLLIN };
				if (poll(&pfd, 1, 1000) <= 0)
					break;
				drmHandleEvent(k.fd, &evctx);
			}
		}

		if (onscreen)
			ReturnPicture(dec, onscreen);
		onscreen = pic;
		pic = NULL;
		frames++;
		first = 0;

		if (frames == 1)
			printf("First frame on HDMI: %dx%d cropped to %dx%d, fb %u\n",
			       onscreen->nWidth, onscreen->nHeight, src_w, src_h, fb);

		/* Periodic leak telemetry. Open descriptors and imported surface
		 * count must both plateau; continued growth in either is the
		 * signature this soak exists to catch. */
		if (now.tv_sec - last_report >= report_every) {
			long elapsed = now.tv_sec - t0.tv_sec;

			printf("[%5lds] frames %-7d %.2f fps  imports %-3d  fds %-4d  rss %ldkB  loops %d\n",
			       elapsed, frames,
			       elapsed ? (double)frames / elapsed : 0.0,
			       imports, count_open_fds(), rss_kb(), loops);
			fflush(stdout);
			last_report = now.tv_sec;
		}
	}

	printf("\nPASS: %d frames scanned out over ~%ds\n", frames, seconds);
	printf("      %d distinct decoder surfaces imported (pool size)\n", imports);
	printf("      no CPU pixel mapping or copy at any point\n");
	if (use_fence)
		printf("      sync: explicit OUT_FENCE_PTR release fence"
		       " (%ld waits, %ld timeouts, %ld missing)\n",
		       fence_waits, fence_timeouts, fence_missing);
	else
		printf("      sync: page-flip event, no explicit fence\n");
	rc = (frames > 0) ? 0 : 1;

out:
	if (onscreen && dec)
		ReturnPicture(dec, onscreen);
	if (pic && dec)
		ReturnPicture(dec, pic);
	if (k.fd >= 0) {
		int i;

		/* Detach the plane before removing its framebuffers. Dropping
		 * master does not reprogram the display, so a CRTC left pointing
		 * at a removed buffer keeps scanning freed memory - which shows
		 * as a stuck solid-colour screen when no compositor is running
		 * to take over. Disabling the plane and the CRTC leaves the
		 * device in a state the next master can pick up cleanly. */
		if (!first && k.plane_id) {
			drmModeAtomicReq *req = drmModeAtomicAlloc();

			if (req) {
				drmModeAtomicAddProperty(req, k.plane_id, k.p_fb_id, 0);
				drmModeAtomicAddProperty(req, k.plane_id, k.p_crtc_id, 0);
				drmModeAtomicAddProperty(req, k.crtc_id, k.p_crtc_active, 0);
				drmModeAtomicAddProperty(req, k.crtc_id, k.p_crtc_mode_id, 0);
				drmModeAtomicAddProperty(req, k.conn_id, k.p_conn_crtc_id, 0);
				drmModeAtomicCommit(k.fd, req,
						    DRM_MODE_ATOMIC_ALLOW_MODESET,
						    NULL);
				drmModeAtomicFree(req);
			}
		}

		for (i = 0; i < ncache; i++)
			drmModeRmFB(k.fd, cache[i].fb);
		if (k.blob_id)
			drmModeDestroyPropertyBlob(k.fd, k.blob_id);
		drmDropMaster(k.fd);
		close(k.fd);
	}
	if (dec)
		DestroyVideoDecoder(dec);
	if (mem_open)
		CdcMemClose(memops);
	free(es);
	if (f)
		fclose(f);
	return rc;
}
