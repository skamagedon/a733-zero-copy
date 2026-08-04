/*
 * drm-plane-reset.c
 *
 * Repair tool. A presenter that exits while an overlay plane is still enabled
 * leaves that plane scanning out a buffer nobody owns any more. X only
 * programmes the primary plane when it starts, so it never clears a stale
 * overlay: the result is a frozen image sitting on top of the desktop that
 * restarting the display manager cannot remove.
 *
 * This detaches every non-primary plane and leaves the CRTCs alone, so the
 * primary plane (console, or X) becomes visible again.
 *
 * Needs DRM master, so stop the display manager before running it.
 *
 * Usage: drm-plane-reset [drm-card]
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <xf86drm.h>
#include <xf86drmMode.h>

static uint32_t prop_id(int fd, uint32_t obj, uint32_t type, const char *name,
			uint64_t *value)
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
			if (!strcmp(p->name, name)) {
				id = p->prop_id;
				if (value)
					*value = props->prop_values[i];
			}
			drmModeFreeProperty(p);
		}
	}
	drmModeFreeObjectProperties(props);
	return id;
}

int main(int argc, char **argv)
{
	const char *card = (argc > 1) ? argv[1] : "/dev/dri/card0";
	drmModePlaneRes *plres;
	drmModeAtomicReq *req;
	int fd, cleared = 0, rc = 1;
	unsigned i;

	fd = open(card, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		fprintf(stderr, "FAIL: open %s: %s\n", card, strerror(errno));
		return 1;
	}
	drmSetClientCap(fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1);
	if (drmSetClientCap(fd, DRM_CLIENT_CAP_ATOMIC, 1) != 0) {
		fprintf(stderr, "FAIL: no atomic support\n");
		goto out;
	}
	if (drmSetMaster(fd) != 0) {
		fprintf(stderr,
			"FAIL: cannot become DRM master (%s).\n"
			"      Stop the display manager first.\n", strerror(errno));
		goto out;
	}

	plres = drmModeGetPlaneResources(fd);
	if (!plres) {
		fprintf(stderr, "FAIL: drmModeGetPlaneResources\n");
		goto out;
	}

	req = drmModeAtomicAlloc();
	if (!req) {
		drmModeFreePlaneResources(plres);
		goto out;
	}

	for (i = 0; i < plres->count_planes; i++) {
		uint32_t pid = plres->planes[i];
		uint64_t type = 0, fb = 0;
		uint32_t p_type, p_fb, p_crtc;

		p_type = prop_id(fd, pid, DRM_MODE_OBJECT_PLANE, "type", &type);
		p_fb   = prop_id(fd, pid, DRM_MODE_OBJECT_PLANE, "FB_ID", &fb);
		p_crtc = prop_id(fd, pid, DRM_MODE_OBJECT_PLANE, "CRTC_ID", NULL);
		if (!p_type || !p_fb || !p_crtc)
			continue;

		/* DRM_PLANE_TYPE_PRIMARY is 1; leave it alone so the console or
		 * X remains visible. Overlay (0) and cursor (2) are cleared. */
		if (type == DRM_PLANE_TYPE_PRIMARY)
			continue;
		if (!fb)
			continue;	/* already detached */

		printf("clearing plane %u (type %llu, was fb %llu)\n", pid,
		       (unsigned long long)type, (unsigned long long)fb);
		drmModeAtomicAddProperty(req, pid, p_fb, 0);
		drmModeAtomicAddProperty(req, pid, p_crtc, 0);
		cleared++;
	}

	if (!cleared) {
		printf("no stale overlay/cursor planes found\n");
		rc = 0;
	} else if (drmModeAtomicCommit(fd, req, DRM_MODE_ATOMIC_ALLOW_MODESET,
				       NULL) != 0) {
		fprintf(stderr, "FAIL: atomic commit: %s\n", strerror(errno));
	} else {
		printf("cleared %d plane(s)\n", cleared);
		rc = 0;
	}

	drmModeAtomicFree(req);
	drmModeFreePlaneResources(plres);
	drmDropMaster(fd);
out:
	close(fd);
	return rc;
}
