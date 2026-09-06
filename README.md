# Zero-copy hardware video playback on Allwinner A733 (Orange Pi 4 Pro), without OMX

**TL;DR** — On the A733 (`sun60iw2`) the vendor GStreamer/OMX stack cannot hand
you DMA-BUFs, so everyone falls back to `videoconvert ! ximagesink` and burns
~1.8 CPU cores on 1080p. You don't have to. `libvdecoder.so` sits *underneath*
the broken OMX wrapper and hands you a real `dma_buf` file descriptor per
decoded frame, which imports straight into DRM/KMS. Same board, same clip:
**174% CPU → ~8%, and no X server at all.**

The same approach covers encoding: `cedarzcenc` reaches 52 fps at 1080p where
`x264enc` manages 6.7, which is what makes real-time transcode and
compositing possible on this board at all.

Tested on an Orange Pi 4 Pro (A733), Orange Pi 1.0.6 Bullseye, vendor kernel
`5.15.147-sun60iw2`. No kernel patches, no custom modules, no vendor SDK build.

---

## The problem

If you've tried hardware video on an A733 board you've probably found:

- No V4L2 decoder. There is no `/dev/video*` and no `/dev/media*`. Cedrus
  doesn't cover this SoC, so the usual mainline advice doesn't apply.
- `omxh264dec` *advertises* `video/x-raw(memory:DMABuf)` with NV21/YV12, and
  Cedar even logs `UseZeroCopyBuffer [1]`.
- But forcing those caps aborts inside `gst_omx_video_dec_negotiate` with
  `assertion failed: (l != NULL)`.

So the advertised zero-copy caps are not a usable API, and the practical
fallback becomes:

```
omxh264dec disable-dma-feature=true ! videoconvert ! ximagesink
```

which does CPU colour conversion plus X11 blitting. On my board that measured
**174% CPU across 10 threads, plus another 16.5% for the X server** — about 1.8
cores of an 8-core SoC to play one 1080p file.

## Why OMX fails

The vendor `libOmxVdec.so` *does* allocate its output surfaces from
`/dev/dma_heap/system` and does hold a distinct DMA-BUF fd for each one. The
failure is in the wrapper:

- OMX port 1 reports the proprietary colour format `0x7f000002`
  (`OMX_COLOR_FormatVendorStartUnused` is `0x7F000000`, so this is a vendor
  extension).
- Its port-format enumeration is malformed: it returns `OMX_ErrorNone` for
  out-of-range indices instead of terminating with `OMX_ErrorNoMore`.
- Vendor `gst-omx` therefore can't build a colour-format negotiation map, and
  aborts when you force DMA-BUF caps.

You can chase that for a long time. Don't. The fix is to stop using OMX.

## The fix: go underneath OMX

`libOmxVdec.so` is a wrapper over **`libvdecoder.so`**, and that lower library
is what actually owns the decode surfaces. Critically, the matching headers are
already installed on the stock image:

```
/usr/include/vdecoder.h  vbasetype.h  veInterface.h  sc_interface.h
                         memoryAdapter.h  typedef.h  cdc_config.h
```

Use those, not the public `allwinner-zh/media-codec` or CedarC repos — those are
older generations with different values.

`VideoPicture` in `vbasetype.h` contains:

```c
int nBufFd;   /* genuine dma_buf fd for this decoded frame */
```

That's the whole ballgame. The proprietary `0x7f000002` format and the broken
enumeration live *only* in the OMX wrapper, so going one layer down removes the
defect from the path entirely rather than working around it.

### The pipeline

```
H.264 Annex-B access units
  -> libvdecoder  (CreateVideoDecoder / InitializeVideoDecoder /
                   RequestVideoStreamBuffer / SubmitVideoStreamData /
                   DecodeVideoStream / RequestPicture)
  -> VideoPicture.nBufFd            (dma_buf, /dev/dma_heap/system)
  -> drmPrimeFDToHandle
  -> drmModeAddFB2WithModifiers     (NV21, linear)
  -> atomic commit on a DRM plane, with OUT_FENCE_PTR
  -> HDMI
  -> ReturnPicture                  (only after the commit completes)
```

Link against:

```sh
-lvdecoder -lMemAdapter -lVE -lvideoengine -lcdc_base -ldrm
```

Runs as a normal user in the `video` group. No root needed.

## Results

Same board, same 1080p H.264 clip:

| | vendor X11 path | zero-copy DRM |
| --- | --- | --- |
| player CPU | 174% (10 threads) | **~8%** (1–2 threads) |
| X server | +16.5% | **not needed** |
| system-wide | 22.2% of 8 cores | **1.5%** |
| load average | 1.80 | 0.50 |

Frame delivery is vsync-locked; a 1-hour soak held 90,000 frames at exactly
25.00 fps with the imported-surface count, descriptor count and RSS each taking
**one distinct value** for the whole run — no drift.

---

## The traps (this is the part that will save you days)

These cost me the most time. In rough order of nastiness:

**1. `GetVeOpsS` takes an argument, and has no prototype.**
`libVE.so` exports it but `veInterface.h` doesn't declare it. Disassembly shows
it dispatches on an engine type: `0` → `getVeAwOpsS` (H.264/H.265), `1` →
`getVeVp9OpsS`, anything else rejected. Declaring it `void` passes garbage and
then segfaults.

```c
extern VeOpsS *GetVeOpsS(int type);   /* 0 for H.264/H.265 */
```

**2. Use `CdcMemOpen`, not `CdcMemSetup`.**
`CdcMemSetup` is an unimplemented hook that just fails. And `CdcMemOpen2`
dereferences the VE-ops self pointer you hand it, so calling it with `NULL`
faults *after* it has already opened `/dev/dma_heap/system` — which makes it
look like a permissions problem when it isn't.

**3. `AddVDPlugin` lives in `libvideoengine.so`**, not `libvdecoder.so`. Miss it
and you get a link error naming a DSO you did remember to link.

**4. Feed whole access units.**
The decoder must be handed complete H.264 access units. Submitting arbitrary
byte chunks with `bIsFirstPart`/`bIsLastPart` both set makes it decode pictures
that begin mid-slice — you get `H264CheckNewFrame: the first slice of the frame
is not 0` in the log and visible tearing on screen. Easiest correct answer is
libavformat's `h264_mp4toannexb` bitstream filter, which yields exactly one AU
per packet.

**5. `modetest -p` hides the fence properties.**
`OUT_FENCE_PTR` and `IN_FENCE_FD` are flagged `DRM_MODE_PROP_ATOMIC`, and the
kernel hides those from any client that hasn't set `DRM_CLIENT_CAP_ATOMIC`. A
plain `modetest -p` listing omits them — and omits `FB_ID` and `SRC_W` too,
which is the tell that you're looking at a filtered list. Both properties exist
on this driver. Enumerate with the atomic cap set before concluding anything.

**6. Detach the plane before removing its framebuffer.**
Dropping DRM master does *not* reprogram the display. A CRTC left pointing at a
removed buffer keeps scanning freed memory, which shows as a stuck solid-colour
screen that survives restarting your compositor. Commit `FB_ID = 0` /
`CRTC_ID = 0` first. The same applies between clips: cover the transition with a
black framebuffer, or you get a brief flash of whatever was in that memory.

**7. Geometry.** `nHeight` is the padded height (1088 for 1080p);
`nBottomOffset`/`nRightOffset` are absolute crop *edges*, not insets. Derive the
chroma plane offset from the decoder's own `pData1 - pData0` rather than
assuming an alignment rule.

**8. Pace to the source frame rate.** Giving each decoded frame one vblank plays
30 fps content at the 60 Hz refresh — double speed, which reads as judder.

## On fences

The CRTC exposes `OUT_FENCE_PTR`, so you can get a proper release fence: commit
`NONBLOCK`, and the kernel hands back a `sync_file` fd that signals when that
commit completed — i.e. when the previous surface is genuinely free. Wait on it
before `ReturnPicture`. Over 7,476 commits I measured 0 timeouts and 0 missing
fences, with the descriptor count flat (close the fence fd — that's the one leak
this mode introduces).

There is **no acquire fence**, and none is needed: the Cedar ABI exposes no
producer fence anywhere in its headers, and `RequestPicture()` returns only
completed pictures, so decode completion is already signalled by the call
returning. The plane does carry `IN_FENCE_FD` if a future decoder ever exposes
one.

## H.264 encoding: the hardware encoder works, and its output can be made to decode

Four things trip people up here, and none of them is a hardware limitation.
Number 4 is the one that produces the widely reported `wait interrupt
overtime`, and it turns out to be an alignment rule rather than a defect.

**1. The encoder device is root-only.** The decoder node is world-accessible,
the encoder node is not:

```
crw-rw-rw-  /dev/cedar_dev       decoder
crw-------  /dev/cedar_dev_ve2   encoder
```

As a normal user, `VideoEncCreate()` fails long before any encoding happens:

```
ve2_env_init: open /dev/cedar_dev_ve2 faild, fd = -1
VeInitialize:  veEnvInit failed
VideoEncCreate: init ve ops failed
```

That is a *different* failure from the reported "wait interrupt overtime" - it
never reaches the interrupt path. For the failure that genuinely does produce
that message, see item 4 below. Run as root, or add a udev rule:

```
SUBSYSTEM=="cedar_ve2", KERNEL=="cedar_dev_ve2", GROUP="video", MODE="0660"
```

The subsystem is `cedar_ve2`, **not** `misc`. The driver registers its own
subsystem rather than sitting under `misc`, which is easy to assume and wrong:

```
P: /devices/virtual/cedar_ve2/cedar_dev_ve2
E: SUBSYSTEM=cedar_ve2
```

A rule matching `SUBSYSTEM=="misc"` loads without error and never fires, so the
node keeps its `crw------- root root` default and the only symptom is that
`VideoEncCreate()` still fails as a normal user. Matching on `KERNEL` alone
works too, since the name is unique. With the correct rule and membership of
`video`:

```
crw-rw---- 1 root video 237, 0 /dev/cedar_dev_ve2
```

With the device reachable, encoding runs: 30 frames of 1280x720 encoded in one
pass, 30 bitstream frames returned, 263 KB out. Sustained rates are in
[Encoder throughput](#encoder-throughput) below.

**2. The output is AVCC, not Annex-B.** Setting `bEncH264Nalu = 1` does not
change this. A raw dump of the encoder output starts:

```
00 00 2c 93   <- 4-byte big-endian NAL length (11411)
65            <- NAL type 5, an IDR slice
```

No start codes anywhere. Converting is simple: replace each 4-byte length
prefix with `00 00 00 01`. The slice data itself is genuine - after conversion
you get a correct IDR followed by P slices of plausible sizes.

**3. The parameter sets cannot be retrieved at all — they have to be
synthesized.** This is the part that actually blocks decoding, and it is worth
being precise about, because a byte count alone looks like success.

`VideoEncGetParameter(enc, VENC_IndexParamH264SPSPPS, &hdr)` returns 26 bytes
that are a *correctly shaped* avcC `AVCDecoderConfigurationRecord`:

```
01 64 00 33 ff  01 00 0b [11 bytes]  01 00 04 [4 bytes]
│  │     │      │  └ SPS length 11   └ 1 PPS, length 4
│  │     └ level 5.1
│  └ High profile
└ configurationVersion
```

Profile, level, `lengthSizeMinusOne` and both parameter-set *lengths* are all
plausible - but both *payloads* are `ff ff ff …` filler. That is still true when
the record is fetched after the first frame has been encoded, so it is not
simply a timing problem. `ffmpeg` therefore reports `non-existing PPS 0
referenced` and decodes nothing, even though the slice NALs are valid.

What has been ruled out, via `tools/cedar-sps-pps-diag.c`:

- **Caller-supplied storage.** Passing a poisoned 512-byte buffer in
  `VencHeaderData.pBuffer` with its capacity in `nLength` returns the same
  26-byte `0xff` record, so the API is not expecting the caller to provide room.
- **Extra output pointers.** `VencOutputBuffer.pData1` and `pData2` are empty on
  every frame; parameter sets are not riding along beside the slice data.
- **Needing more frames.** The record is byte-identical before encoding and
  after frames 0, 1 and 2.
- **A different index.** `VENC_IndexParamH264SPSPPS` is the only H.264 header
  index in `vencoder.h`; there is no inline-emission option.
- **`h264_save_sps_pps`.** Despite the name, disassembly shows it `fopen()`s a
  path - it is a debug dump, not the retrieval path.

And finally, **the record is a stub, not a failed copy.** `libvenc_h264.so`
contains the log string `get sps_pps_data: nLength=%d`, which looked like
evidence that an internal path knew a real length. It does not run. Setting
`venc_log_level = 0` in `/etc/cedarc.conf` and re-running never prints that
line, so nothing is generating the parameter sets in the first place. There is
nothing to retrieve, and no further point poking at the API.

A logging detail worth knowing while you test this: the runtime prints
`Set log level to 5 from /vendor/etc/cedarc.conf`, but **`/vendor` does not
exist on this image**. The file actually read is `/etc/cedarc.conf` — the line
immediately above it, `load conf file /etc/cedarc.conf ok!`, is the truthful
one. The path in that message is a hardcoded string. There is no `getenv` in
`libcdc_base.so`, so the level cannot be set from the environment, and the file
semantics are *"log will output if level >= log_level"* — **lower** the number
to see **more**.

### The fix: synthesize the parameter sets

The encoder does not tell you its configuration, but it cannot hide it either:
every slice header it writes is consistent with exactly one set of values.
Parsing 30 consecutive slice headers pins them.

| observation across 30 slices | forces |
|---|---|
| `frame_num` reads 0,1,2,… and resets at the IDR | `log2_max_frame_num = 12` |
| `pic_order_cnt_lsb` steps by exactly 2 | `log2_max_pic_order_cnt_lsb = 12` |
| `cabac_init_idc` present and valid | `entropy_coding_mode_flag = 1` |
| `slice_qp_delta` ramps smoothly +4 → −8 | `pic_init_qp_minus26 = 0` |

That last row is the one that tells you the bit alignment is genuinely right
rather than accidentally plausible: a wrong offset produces noise, not a
monotone rate-control curve.

Three more fields fall out of decode testing, because any other value breaks
CABAC context initialisation immediately at MB 0: `transform_8x8_mode_flag = 0`,
`constrained_intra_pred_flag = 0`, `pic_init_qp_minus26 = 0`.

`chroma_qp_index_offset` needs a different test, because a wrong value **decodes
without any error** and only degrades chroma silently. Scoring PSNR against the
probe's own deterministic source pattern settles it: `0` gives U 43.4 dB /
V 44.2 dB, and every other value in −7…+7 collapses by 5 to 25 dB.

For 1280×720 the result is:

```
SPS (11 bytes):  67 64 00 33 ac 13 12 e0 50 05 b9
PPS (4 bytes):   68 ee 3c 80
```

Both match the lengths the avcC record declares **exactly** — the SPS is 87 bits
plus the stop bit, filling 11 bytes with no padding. All 30 frames then decode
with zero `ffmpeg` errors at ~46 dB PSNR against the source.

Two honest caveats. `max_num_ref_frames` is **not** recoverable from the stream:
it only ever uses one reference, so 1 and 2 decode identically. 2 is used as the
safer DPB hint. And these values describe the encoder's *default* configuration
— change profile, level, GOP or rate control and they must be re-derived. Take
profile and level from the avcC record's header bytes, which are valid.

`tools/cedar-h264-encode-probe.c` implements all of this: it detects the filler
record, synthesizes the parameter sets (including frame cropping, so 1080p codes
as 1088 with `frame_crop_bottom_offset = 4`), and writes a decodable Annex-B
file. Build with `make tools`, run as root.

### 4. Encode height must be a multiple of 16

This is what actually produces `h264 encoder wait interrupt overtime`, the error
most often reported against this encoder. It is not a hardware limit, not a
permissions problem, and not a fault in the interrupt path: the encoder never
completes a frame whose height is not 16-aligned, and the driver then times out
waiting for an interrupt that is never coming. It fails at frame 0, every time,
in about a second.

Sweep at default settings, 60 frames each:

```
1280x720    h%16=0   OK
1920x720    h%16=0   OK
1600x896    h%16=0   OK
1920x1088   h%16=0   OK
1280x1080   h%16=8   FAIL at frame 0
1600x900    h%16=4   FAIL at frame 0
1920x1080   h%16=8   FAIL at frame 0
```

Perfect correlation, no exceptions. Every width tested was already 16-aligned,
so this documents the height constraint only.

So 1080p encodes fine as **1920x1088** with the crop signalled in the SPS, which
is the same padding the decode side already needs. Note that
`tools/cedar-h264-encode-probe.c` passes `argv[2]` straight into
`cfg.nInputHeight` and does not pad for you: call it with 1088, not 1080.

### Encoder throughput

Timed on an otherwise idle board against monotonic `/proc/uptime`, two frame
counts per resolution so that encoder init cancels out of the marginal rate:

| resolution | 60 frames | 360 frames | marginal | vs 30 fps | vs 60 fps |
|---|---|---|---|---|---|
| 1280x720  | 0.43 s | 1.91 s | **~203 fps** | 6.8x | 3.4x |
| 1920x1088 | 0.75 s | 3.82 s | **~98 fps**  | 3.3x | 1.6x |

Both figures are lower bounds on the encoder itself, because the probe also
generates its synthetic source frame on the CPU inside the timed loop. The A76
cluster stayed at 416 MHz under `ondemand` for the whole run (max 2002 MHz) and
the VE ran at its default 624 MHz, so neither was boosted to get these numbers.

Content is synthetic and the run is short, so this measures the fixed-function
pipeline rather than long-run behaviour under real video. Encode has not been
soak-tested; the one-hour soak in [Results](#results) was the decode path.

### Rate control: the full set is available

`VENC_RC_MODE` in `vencoder.h` is not QP-only:

```c
AW_CBR = 0    AW_VBR = 1    AW_AVBR = 2    AW_QPMAP = 3    AW_FIXQP = 4
```

with the surrounding controls you would want for streaming:

| control | index / struct |
|---|---|
| target bitrate (bps)  | `VENC_IndexParamBitrate`, `nBitrate` |
| ceiling and floor     | `VENC_IndexParamSetBitRateRange` -> `VencBitRateRange` |
| VBV buffer            | `VENC_IndexParamSetVbvSize`, `VENC_IndexParamVbvInfo` |
| keyframe interval     | `VENC_IndexParamMaxKeyInterval` |
| on-demand IDR         | `VENC_IndexParamForceKeyFrame` |
| QP bounds inside RC   | `VencQPRange` |
| per-frame size cap    | `VENC_IndexParamSuperFrameConfig` |
| overflow behaviour    | `VENC_IndexParamDropOverflowFrame`, `VENC_IndexParamFillingCbr` |

H.265 is present too (`libvenc_h265.so`, `VENC_IndexParamH265Param`), untested
here.

The probe leaves all of these at their defaults, since `VencBaseConfig` is
memset to zero, so the synthesized parameter sets above describe the *default*
configuration only. `nBitrate` does not appear in the SPS or PPS, so changing it
at runtime should not invalidate them, but that is an inference from where the
field lives rather than something tested here.

## Scope and caveats

- Tested **only** on A733 / `sun60iw2`, Orange Pi 1.0.6 Bullseye, kernel
  `5.15.147-sun60iw2`.
- The same `libcedarc` family ships on other Allwinner parts (A133, H616, H618,
  T527…) and the `VideoPicture.nBufFd` approach is likely to transfer, but I
  have not tested it there. Check your own `/usr/include/vbasetype.h` — field
  layout is guarded by `VE_SUPPORT_DECODER_LBC_MODE` and differs between builds.
- H.264 only in my implementation. Cedar handles H.265 too; it's a config
  change, not a redesign.
- Still images have no hardware path — they're a CPU decode into a dumb buffer.
  The zero-copy claim is about video.
- This needs DRM master, so it can't run under X or a Wayland compositor. It
  replaces them rather than coexisting.
- The vendor headers are Allwinner's and ship on the stock image. Don't
  redistribute them; just point people at `/usr/include`.

---

## Build and run

Build **on the board**. The vendor Cedar libraries and headers ship with the
stock Orange Pi image and are not redistributed here.

```sh
make check     # verify the vendor stack is present and you're in the video group
make           # build everything into build/
```

`make check` reports each required header, library and device node, so a
mismatched image fails legibly instead of as a link error.

### 1. Prove it works — `cedar-dmabuf-drm-probe`

Decodes one frame and imports it into DRM. Never mmaps the surface, so the
absence of any pixel access is part of the result.

```sh
./build/cedar-dmabuf-drm-probe clip.h264 /dev/dri/card0
```

```
format      : 5 (NV21)
size        : 1920x1088  stride 1920
nBufFd      : 9
fd resolves : /dmabuf:  [genuine dma_buf]
PASS: Cedar decoded surface imported by DRM as framebuffer 166
```

### 2. Reference player — `cedar-drm-present`

Plays a raw H.264 Annex-B stream to HDMI. Needs DRM master, so stop your
display manager first.

```sh
sudo systemctl stop lightdm
./build/cedar-drm-present clip.h264 30 /dev/dri/card0 25 1 1
#                         file  secs card          fps loop fence
```

`fence` selects synchronisation: `1` uses an explicit `OUT_FENCE_PTR` release
fence with non-blocking commits, `0` falls back to page-flip events. Both are
kept so the fence path is checkable rather than merely asserted.

Generate a test clip on a desktop machine (the board has no encoder):

```sh
ffmpeg -f lavfi -i testsrc2=size=1920x1080:rate=30:duration=60        -c:v libx264 -pix_fmt yuv420p -bsf:v h264_mp4toannexb -f h264 clip.h264
```

### 3. Playlist player — `zc-playlist-player`

Takes MP4 directly (demuxed with libavformat), rotates a playlist, shows
stills, survives HDMI hotplug, and reloads the playlist when it changes on
disk.

```sh
./build/zc-playlist-player playlist.tsv /dev/dri/card0
```

`playlist.tsv` is tab-separated, paths relative to the playlist file:

```
video	clip-one.mp4
image	slide.png
video	clip-two.mp4
```

### 4. GStreamer element — `cedarzcdec`

If you'd rather not adopt a dedicated player, this is the drop-in: a
`GstVideoDecoder` that talks to `libvdecoder` and hands downstream real
`GstDmaBufMemory`. It's what the vendor's `gst-omx` was supposed to be.

```sh
make gst
sudo make install-gst
gst-inspect-1.0 cedarzcdec
```

Then existing pipelines just work:

```sh
gst-launch-1.0 filesrc location=clip.mp4 ! qtdemux ! h264parse     ! cedarzcdec ! kmssink driver-name=sunxi-drm
```

Measured on the same board and clip as the standalone player:

| pipeline | CPU | threads | RSS |
| --- | --- | --- | --- |
| `omxh264dec disable-dma-feature=true ! videoconvert ! ximagesink` | 172–182% | 10 | 31.9 MB |
| `cedarzcdec ! kmssink` | **12–13%** | 4 | 13.0 MB |

It registers at `GST_RANK_PRIMARY + 1`, so `decodebin`/`playbin` pick it ahead
of `omxh264dec`.

The output format is negotiated, not hardcoded: the element advertises both
NV12 and NV21, asks downstream which it wants, and configures the decoder
accordingly. NV12 is preferred when the peer accepts either, since it is the
more widely expected format and costs exactly the same here. Forcing either
works:

```sh
... ! cedarzcdec ! "video/x-raw(memory:DMABuf),format=NV21" ! fakesink
... ! cedarzcdec ! "video/x-raw(memory:DMABuf),format=NV12" ! fakesink
```

The format the decoder actually delivers is treated as authoritative, so a
silent override would surface as a caps mismatch rather than as wrong colours.

Two implementation notes that may save you time if you write something similar:

- **`alignment=au` is required on the sink pad, not merely preferred.**
  `libvdecoder` must be handed whole access units; feeding it arbitrary byte
  runs makes it decode pictures starting mid-slice. Requiring `alignment=au`
  makes `h264parse` do that framing for you.
- **Don't force the `memory:DMABuf` caps feature.** `kmssink` (1.18) advertises
  plain `video/x-raw` and decides whether to import a `dma_buf` by inspecting
  the buffer's memory at render time. Forcing the feature fails to negotiate
  with exactly the sinks you want to feed. This element offers the feature,
  checks whether the peer accepts it, and falls back to plain caps — the
  buffers are `dma_buf`-backed either way, so nothing is copied.

### 5. GStreamer element — `cedarzcenc`

The encoder counterpart. The silicon always encoded fine, but nothing exposed
it to GStreamer, so the only usable encoder on this board was `x264enc` at
about 16 fps for 720p and 6.7 for 1080p. This element closes that gap.

```sh
make gst
sudo make install-gst
gst-inspect-1.0 cedarzcenc
```

```sh
gst-launch-1.0 videotestsrc ! video/x-raw,format=NV12,width=1920,height=1080     ! cedarzcenc bitrate=3000 gop=60 ! h264parse ! flvmux ! rtmp2sink location=...
```

Measured on A733, 1920x1080 NV12:

| encoder | rate at 1080p |
| --- | --- |
| `x264enc speed-preset=veryfast` | 6.7 fps |
| `cedarzcenc` | **52.4 fps** |

300 frames in 5.729 s, output `h264 High, 1920x1080, yuv420p`, decoding with
zero `ffmpeg` errors, keyframe spacing exactly 60 frames at `gop=60`, and
3.15 Mbps against `bitrate=3000`.

End to end against a live camera, both directions in hardware:

```sh
gst-launch-1.0 rtspsrc location=rtsp://... ! rtph264depay ! h264parse     ! cedarzcdec ! videoconvert ! video/x-raw,format=NV12     ! cedarzcenc bitrate=2500 gop=40 ! h264parse ! ...
```

held realtime 1080p for ten minutes at the requested bitrate.

The element hides all three encoder quirks documented above:

- **16-alignment is invisible to callers.** Ask for 1080 and it encodes 1088
  and emits the difference as SPS frame cropping, so the stream correctly
  reports 1080. No caller ever has to think about `wait interrupt overtime`.
- **AVCC is rewritten to Annex-B** on the way out.
- **Parameter sets are synthesized and prepended ahead of every IDR**, since
  they cannot be retrieved and there is no inline-emission option. That is also
  exactly what a live RTMP or HLS consumer needs.

Properties: `bitrate` (CBR kbps), `gop`, `qp-min`, `qp-max`. `bitrate` and
`gop` are settable while the pipeline is running, because neither appears in
the SPS or PPS and so changing them does not invalidate the synthesized
parameter sets.

Two things worth knowing if you modify it:

- **`bEntropyCodingCABAC` must be 1.** `build_pps()` writes
  `entropy_coding_mode_flag = 1`, and a PPS claiming CABAC over CAVLC slice
  data does not decode at all. The two have to be changed together.
- **`eRcMode` is not on `VencH264Param`.** It lives in the nested `sRcParam`,
  as does the VBR configuration. Setting it on the outer struct silently does
  nothing and you get the constant-quality default, which is wrong for a fixed
  uplink.

The input path is currently a `memcpy` into the encoder's own buffer, which is
most of the distance between this element's 52 fps and the 98 fps the raw probe
reaches at the same resolution. Importing DMA-BUFs instead would close it, and
would also let `cedarzcdec` feed `cedarzcenc` without a round trip through
system memory.

## Output pixel format: NV12, NV21, YV12 are all selectable

`VConfig.eOutputPixelFormat` is honoured by the decoder, so you are not stuck
with whatever the examples happen to use. Set it before
`InitializeVideoDecoder()`:

```c
vc.eOutputPixelFormat = PIXEL_FORMAT_NV12;   /* 6; NV21 is 5 */
```

Measured on A733 with an 8-bit H.264 1080p source, requesting each format and
reporting what `VideoPicture.ePixelFormat` actually came back as:

| requested | delivered | DRM PRIME import |
| --- | --- | --- |
| 5 `PIXEL_FORMAT_NV21` | 5 NV21 | works |
| 6 `PIXEL_FORMAT_NV12` | 6 NV12 | works |
| 4 `PIXEL_FORMAT_YV12` | 4 YV12 | works |
| 1 `PIXEL_FORMAT_YUV_PLANER_420` | 1 planar 420 | works |
| 22 `PIXEL_FORMAT_P010_UV` | 22 reported, but `nBufFd` is 0 | no dma_buf |

So NV12 costs nothing versus NV21 — same stride, same padded height, same
zero-copy import. Pick whichever your sink prefers rather than converting.

The P010 result needs a caveat: the test source was 8-bit, and a 10-bit output
from an 8-bit stream is not a meaningful request, so this does **not** show that
P010 is unsupported in general. It shows only that in that configuration the
decoder reports the format but hands back no descriptor. Retest with genuine
10-bit content before concluding anything about HDR/10-bit paths.

Reproduce with the probe, which takes the format as its third argument:

```sh
./build/cedar-dmabuf-drm-probe clip.h264 /dev/dri/card0 6    # ask for NV12
```

### Diagnostics

```sh
./build/drm-fence-caps-probe /dev/dri/card0   # does this driver expose fences?
./build/drm-plane-reset /dev/dri/card0        # recover a display left stuck
```

`drm-plane-reset` exists because a player that exits without detaching its plane
leaves the CRTC scanning a removed buffer — a stuck solid-colour screen that
restarting X does **not** clear.

## Licence

MIT, see `LICENSE`. The Allwinner headers and libraries this builds against are
not covered by it and are not included; they come with your board's image.

## Related

- Radxa A733 report of the same Cedar stack and the same `0x7f000002` behaviour:
  <https://forum.radxa.com/t/h-264-encode-decode-loopback-does-not-work/30733>
- Allwinner's public (older-generation) OMX/CedarX source, useful for
  understanding the shape of the API but **not** for its values:
  <https://github.com/allwinner-zh/media-codec>

## Operating contract

This box is part of the Tiki Productions rack estate. The access paths,
hardware findings and rules that apply across all of it live in
[project-law](https://github.com/skamagedon/project-law). **Read it first.**
It is the record of what was already learned the hard way, and answers that
sit in it have been rediscovered slowly more than once.
