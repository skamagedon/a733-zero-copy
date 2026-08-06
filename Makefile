# Zero-copy hardware video playback on Allwinner A733 (sun60iw2).
#
# Build on the target board. The vendor Cedar libraries and their headers ship
# with the stock Orange Pi image; nothing here is redistributed.
#
#   make            build everything
#   make probe      minimal proof: Cedar frame -> DRM framebuffer
#   make present    reference player for a raw H.264 Annex-B stream
#   make player     playlist player (MP4 via libavformat, images, hotplug)
#   make tools      diagnostics: fence capability probe, plane reset
#   make gst        GStreamer element (libgstcedarzc.so)
#   make install-gst  install it into the GStreamer plugin dir
#
# The playlist player additionally needs libavformat/libavcodec/libswscale.

CC      ?= gcc
CFLAGS  ?= -O2 -Wall -Wextra -Wno-unused-parameter
DRM_CFLAGS := $(shell pkg-config --cflags libdrm 2>/dev/null || echo -I/usr/include/libdrm)
DRM_LIBS   := $(shell pkg-config --libs libdrm 2>/dev/null || echo -ldrm)

# Vendor Cedar stack, installed by the stock image.
CEDAR_LIBS := -lvdecoder -lMemAdapter -lVE -lvideoengine -lcdc_base

GST_MODULES := gstreamer-1.0 gstreamer-base-1.0 gstreamer-video-1.0 gstreamer-allocators-1.0
GST_CFLAGS  := $(shell pkg-config --cflags $(GST_MODULES) 2>/dev/null)
GST_LIBS    := $(shell pkg-config --libs $(GST_MODULES) 2>/dev/null)
GST_PLUGINDIR := $(shell pkg-config --variable=pluginsdir gstreamer-1.0 2>/dev/null)

AV_CFLAGS := $(shell pkg-config --cflags libavformat libavcodec libswscale 2>/dev/null)
AV_LIBS   := $(shell pkg-config --libs libavformat libavcodec libavutil libswscale 2>/dev/null)

BINDIR := build

BINS := $(BINDIR)/cedar-dmabuf-drm-probe \
        $(BINDIR)/cedar-drm-present \
        $(BINDIR)/zc-playlist-player \
        $(BINDIR)/drm-fence-caps-probe \
        $(BINDIR)/drm-plane-reset

.PHONY: all probe present player tools gst install-gst clean check

all: $(BINS)

probe:   $(BINDIR)/cedar-dmabuf-drm-probe
present: $(BINDIR)/cedar-drm-present
player:  $(BINDIR)/zc-playlist-player
tools:   $(BINDIR)/drm-fence-caps-probe $(BINDIR)/drm-plane-reset $(BINDIR)/cedar-h264-encode-probe
gst:     $(BINDIR)/libgstcedarzc.so

# Built as a shared object with -fPIC; GStreamer dlopen()s it from the
# plugin directory.
$(BINDIR)/libgstcedarzc.so: gst/gstcedarzcdec.c | $(BINDIR)
	$(CC) $(CFLAGS) -fPIC -shared $(GST_CFLAGS) -o $@ $< $(CEDAR_LIBS) $(GST_LIBS)

install-gst: $(BINDIR)/libgstcedarzc.so
	@test -n "$(GST_PLUGINDIR)" || { echo "cannot find the GStreamer plugin dir"; exit 1; }
	install -m 0644 $< $(DESTDIR)$(GST_PLUGINDIR)/libgstcedarzc.so
	@echo "installed; verify with: gst-inspect-1.0 cedarzcdec"

$(BINDIR):
	@mkdir -p $(BINDIR)

$(BINDIR)/cedar-dmabuf-drm-probe: src/cedar-dmabuf-drm-probe.c | $(BINDIR)
	$(CC) $(CFLAGS) $(DRM_CFLAGS) -o $@ $< $(CEDAR_LIBS) $(DRM_LIBS)

$(BINDIR)/cedar-drm-present: src/cedar-drm-present.c | $(BINDIR)
	$(CC) $(CFLAGS) $(DRM_CFLAGS) -o $@ $< $(CEDAR_LIBS) $(DRM_LIBS)

$(BINDIR)/zc-playlist-player: src/zc-playlist-player.c | $(BINDIR)
	$(CC) $(CFLAGS) $(DRM_CFLAGS) $(AV_CFLAGS) -o $@ $< $(CEDAR_LIBS) $(DRM_LIBS) $(AV_LIBS)

$(BINDIR)/drm-fence-caps-probe: tools/drm-fence-caps-probe.c | $(BINDIR)
	$(CC) $(CFLAGS) $(DRM_CFLAGS) -o $@ $< $(DRM_LIBS)

$(BINDIR)/drm-plane-reset: tools/drm-plane-reset.c | $(BINDIR)
	$(CC) $(CFLAGS) $(DRM_CFLAGS) -o $@ $< $(DRM_LIBS)

# Encoder probe: needs the venc libraries, not libdrm.
$(BINDIR)/cedar-h264-encode-probe: tools/cedar-h264-encode-probe.c | $(BINDIR)
	$(CC) $(CFLAGS) -o $@ $< -lvencoder -lMemAdapter -lVE -lcdc_base

# Fail early and legibly if the vendor stack is not what this expects.
check:
	@echo "== vendor headers =="
	@for h in vdecoder.h vbasetype.h veInterface.h sc_interface.h memoryAdapter.h; do \
		if [ -e /usr/include/$$h ]; then echo "  OK   /usr/include/$$h"; \
		else echo "  MISS /usr/include/$$h"; fi; done
	@echo "== vendor libraries =="
	@for l in libvdecoder libMemAdapter libVE libvideoengine libcdc_base; do \
		if ls /usr/lib/*/$$l.so >/dev/null 2>&1; then echo "  OK   $$l.so"; \
		else echo "  MISS $$l.so"; fi; done
	@echo "== dma-buf / DRM nodes =="
	@for d in /dev/dma_heap/system /dev/dri/card0 /dev/cedar_dev; do \
		if [ -e $$d ]; then echo "  OK   $$d"; else echo "  MISS $$d"; fi; done
	@echo "== group membership (need 'video' for /dev/dri and /dev/dma_heap) =="
	@id -nG | tr ' ' '\n' | grep -qx video && echo "  OK   in video group" \
		|| echo "  WARN not in video group"

clean:
	rm -rf $(BINDIR)
