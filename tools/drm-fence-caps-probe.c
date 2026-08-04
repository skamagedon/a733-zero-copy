/*
 * drm-fence-caps-probe.c
 *
 * Determine whether this driver can support explicit fence synchronisation.
 *
 * This cannot be answered with `modetest -p`: OUT_FENCE_PTR and IN_FENCE_FD are
 * flagged DRM_MODE_PROP_ATOMIC, and the kernel hides every such property from
 * clients that have not set DRM_CLIENT_CAP_ATOMIC. A plain modetest listing
 * omits them for the same reason it omits FB_ID and SRC_W, which demonstrably
 * do work. Enumerating with the cap set is the only sound test.
 *
 * OUT_FENCE_PTR (on the CRTC) yields a fence that signals when a commit has
 * completed, which is the release signal for the previously scanned-out buffer.
 * IN_FENCE_FD (on the plane) makes the kernel wait on a producer fence before
 * scanning out, which is the acquire side.
 *
 * Usage: drm-fence-caps-probe [drm-card]
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
#include <drm_fourcc.h>

static void dump_props(int fd, uint32_t obj, uint32_t type, const char *label,
		       const char *want_a, const char *want_b)
{
	drmModeObjectProperties *props;
	unsigned i;
	int found_a = 0, found_b = 0;

	props = drmModeObjectGetProperties(fd, obj, type);
	if (!props) {
		printf("%s %u: cannot read properties\n", label, obj);
		return;
	}

	printf("\n%s %u properties (%u):\n  ", label, obj, props->count_props);
	for (i = 0; i < props->count_props; i++) {
		drmModePropertyRes *p = drmModeGetProperty(fd, props->props[i]);
		if (!p)
			continue;
		printf("%s ", p->name);
		if (want_a && !strcmp(p->name, want_a))
			found_a = 1;
		if (want_b && !strcmp(p->name, want_b))
			found_b = 1;
		drmModeFreeProperty(p);
	}
	printf("\n");
	if (want_a)
		printf("  %-14s : %s\n", want_a, found_a ? "PRESENT" : "ABSENT");
	if (want_b)
		printf("  %-14s : %s\n", want_b, found_b ? "PRESENT" : "ABSENT");

	drmModeFreeObjectProperties(props);
}

int main(int argc, char **argv)
{
	const char *card = (argc > 1) ? argv[1] : "/dev/dri/card0";
	drmModeRes *res;
	drmModePlaneRes *plres;
	int fd;
	uint64_t cap = 0;
	unsigned u;

	fd = open(card, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		fprintf(stderr, "open %s: %s\n", card, strerror(errno));
		return 1;
	}

	drmSetClientCap(fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1);
	if (drmSetClientCap(fd, DRM_CLIENT_CAP_ATOMIC, 1) != 0) {
		fprintf(stderr, "driver does not support atomic\n");
		return 1;
	}
	printf("atomic client cap: set\n");

	/* Timeline semaphores and sync objects are a separate capability from
	 * the fence properties, but they indicate how much of the explicit
	 * synchronisation stack the driver carries. */
	if (drmGetCap(fd, DRM_CAP_SYNCOBJ, &cap) == 0)
		printf("DRM_CAP_SYNCOBJ         : %llu\n", (unsigned long long)cap);
	cap = 0;
	if (drmGetCap(fd, DRM_CAP_SYNCOBJ_TIMELINE, &cap) == 0)
		printf("DRM_CAP_SYNCOBJ_TIMELINE: %llu\n", (unsigned long long)cap);
	cap = 0;
	if (drmGetCap(fd, DRM_CAP_PRIME, &cap) == 0)
		printf("DRM_CAP_PRIME           : %llu (1=import 2=export 3=both)\n",
		       (unsigned long long)cap);

	res = drmModeGetResources(fd);
	if (res && res->count_crtcs > 0)
		dump_props(fd, res->crtcs[0], DRM_MODE_OBJECT_CRTC, "CRTC",
			   "OUT_FENCE_PTR", NULL);

	plres = drmModeGetPlaneResources(fd);
	if (plres) {
		for (u = 0; u < plres->count_planes; u++) {
			drmModePlane *pl = drmModeGetPlane(fd, plres->planes[u]);
			unsigned f;
			int nv21 = 0;

			if (!pl)
				continue;
			for (f = 0; f < pl->count_formats; f++)
				if (pl->formats[f] == DRM_FORMAT_NV21)
					nv21 = 1;
			if (nv21) {
				dump_props(fd, pl->plane_id, DRM_MODE_OBJECT_PLANE,
					   "PLANE(NV21-capable)", "IN_FENCE_FD",
					   "FB_ID");
				drmModeFreePlane(pl);
				break;
			}
			drmModeFreePlane(pl);
		}
		drmModeFreePlaneResources(plres);
	}
	if (res)
		drmModeFreeResources(res);
	close(fd);
	return 0;
}
