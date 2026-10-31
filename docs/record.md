# pibridge-record: video recording (tools/record)

`pibridge-record` records what the headset shows into `~/Videos/pimax/pimax-YYYYmmdd-HHMMSS.mp4`. It is the port of
omarchyvr's `omarchyvr record` (`rust/omarchyvr-record`, design in omarchyvr's `docs/RECORD.md`), with the same
options, settings file format, ffmpeg command lines, file names and status texts (checked against the Rust binary,
see Tests). The user starts it with the Dashboard's **Record video** card (docs/app.md); the command line is the
engine behind the card.

```
pibridge-record start [--eyes left|both] [--fps 30|60|72] [--quality low|medium|high] [--scale F] [--max-height N]
                   [--codec h264|hevc] [--encoder auto|nvenc|cpu] [--dir DIR] [--audio on|off] [--mic on|off] [--save]
pibridge-record stop          # finishes the file (always playable) and prints where it is
pibridge-record status
pibridge-record start --brief / stop --brief   # one short line ("Recording", "Saved: name.mp4")
pibridge-record start --source auto|tap|layer|mirror  # auto (default): the Vulkan capture layer under SteamVR
```

Defaults: one eye, 30 fps, high quality, sound on, microphone off, at most 1080 lines per eye, h264, NVENC when
ffmpeg has it (x264 on the CPU otherwise; an NVENC that dies within 5 s falls back to the CPU, same file name).

## The sources, one recorder

The recorder (`pibridge-record run`) encodes frames and does not care where they came from; the session picks the
source (`--source auto`, the Dashboard card does the same):

- **SteamVR** (Lighthouse, inside-out, SteamVR's OpenXR runtime): the **Vulkan capture layer** (`--source layer`, the
  default under SteamVR), described below. The old GL mirror (`--source mirror`) is kept but is no longer the default
  (it crashes vrserver on NVIDIA/Vulkan).
- **A Monado compositor** (`--source tap`, chosen by `auto` while monado-service runs): the tap below, the Monado
  backend's path (docs/plans/MONADO-BACKEND.md). In the headset it starts and stops from PiBridge World's **Record**
  button and from the dashboard overlay over games (Headset page, **Record video**); the app's card works as well.

### The Vulkan capture layer (`--source layer`, the SteamVR default)

`tools/record-layer` is a Vulkan implicit layer (`libpibridge-record-layer.so`, manifest
`VK_LAYER_PIBRIDGE_record.json`) that the loader pulls into SteamVR's `vrcompositor`. There it watches every external
colour image a VR app shares with the compositor (the signal for "a frame bound for the HMD"), and, while recording is
active, captures the chosen view. The recorder's `--source layer` path (`tools/record/src/capture_client.cpp`,
`runLayer()`) is the client/encoder. Two files carry the data between them:

- the **control file** `$XDG_RUNTIME_DIR/pibridge/record-capture` (`active`, `view`, `fps`, `max_height`), written by
  `pibridge-record start`/`stop`, polled by the layer (`common/pibridge_record_capture_control.h`);
- the **socket** `$XDG_RUNTIME_DIR/pibridge/record-capture.sock` (`common/pibridge_record_capture_ipc.h`): the layer
  sends a `HelloMsg` with the exported image fd (`SCM_RIGHTS`) + metadata, then a `FrameMsg` per captured frame; the
  client acks each frame so the one reused image is never torn.

In the compositor, the layer (once per recording) creates an **exportable** `VkImage` (`VK_FORMAT_R8G8B8A8_UNORM`,
`OPTIMAL`, `TRANSFER_DST|TRANSFER_SRC`, `VkExternalMemoryImageCreateInfo{OPAQUE_FD}` + device-local
`VkExportMemoryAllocateInfo{OPAQUE_FD}`), scaled so its height is at most `max_height` (aspect kept; `both` lays the two
eye layers side by side). Per frame, throttled to `fps`, it records a command buffer on a **separate** command pool and
blits the selected view/layer(s) into it with its **own fence** — it never `vkQueueWaitIdle`s the compositor's queue
and never delays the app's own submit. If any Vulkan step fails it disables capture and logs why; VR runs on untouched
(a leaf, zero missed frames). The log is `$XDG_RUNTIME_DIR/pibridge/record-layer.log`.

The client imports the fd into its own Vulkan device on the same GPU (matched by `deviceUUID`), copies the shared image
into a host buffer and feeds that to the same ffmpeg encoder as the other sources — so NVENC (`h264_nvenc`) encodes it
on NVIDIA. (The planned zero-copy `OPAQUE_FD`→CUDA→NVENC and dma_buf→VAAPI paths are TODOs in `capture_client.cpp`;
they need the CUDA toolkit / `libva-dev`, which are not on the current build host.)

**On-hardware iteration (flagged in the source):** (1) the **source image's layout** before the blit — the layer
assumes `SHADER_READ_ONLY_OPTIMAL` and restores it; if the live headset shows corruption/validation errors, try
`GENERAL`/`PRESENT_SRC_KHR` (`submitBlit()` in `layer.cpp`). (2) **cross-process sync** — the handoff relies on the
blit's fence plus the socket ack round-trip (no ownership transfer); the spec-clean upgrade is a shared EXTERNAL
timeline semaphore. If colours look swapped, switch the capture format to `B8G8R8A8_UNORM` and the encoder's
`-pix_fmt` to `bgra`.

**Live-test prerequisite (manifest location):** the Steam Linux Runtime container does not see
`~/.local/share/vulkan/implicit_layer.d`, so for a Proton/SteamVR run the manifest must be copied to a system path, e.g.
`/usr/share/vulkan/implicit_layer.d/VK_LAYER_PIBRIDGE_record.json` (needs root; the app's pkexec step, not an agent).

### The GL mirror (`--source mirror`, opt-in)

- **SteamVR**: `pibridge-record-mirror`, started by the
  recorder. It is a `VRApplication_Background` OpenVR client (it never starts SteamVR; an Overlay client only when
  SteamVR answers `IsNotSceneApplication` to a background one) that takes `IVRCompositor::GetMirrorTextureGL` for
  one or both eyes in a headless EGL context (no window, surfaceless or a 1x1 pbuffer) on an NVIDIA EGL device
  first, then the other devices, then the default display (`PIBRIDGE_RECORD_EGL_DEVICE=N` or `=default` forces one).
  Live 2026-10-19: GLVND's default display was Mesa's on the NVIDIA card ("failed to create dri2 screen") and the
  mirror texture had no size in that context. The log names the EGL devices, the GL renderer, what
  `GetMirrorTextureGL` returned and the size found (GL_TEXTURE_2D, else GL_TEXTURE_RECTANGLE, else SteamVR's
  recommended render size when the texture name is known but reports no size). Per frame: `LockGLSharedTextureForAccess`, one scaling blit per eye into its own
  framebuffer (side by side, rows flipped to top-first; `PIBRIDGE_RECORD_MIRROR_FLIP=0` turns the flip off), Unlock,
  `glReadPixels` into one of three pixel buffers with a fence. It never waits: a busy readback or a full ring drops
  the frame and counts it. It ends on the recorder's stop, a recorder heartbeat older than 3 s, `VREvent_Quit`
  (acknowledged) or vrserver gone. `libopenvr_api.so`: `PIBRIDGE_OPENVR_LIB`, else
  `<runtime>/bin/linux64/libopenvr_api.so` from `~/.config/openvr/openvrpaths.vrpath` (as the app finds it), with
  dlopen; GL entry points from `eglGetProcAddress` (it links libEGL only). Its log: `<session>/record-mirror.log`;
  a start failure reaches the card as "SteamVR's mirror could not start: <why>".

### Monado's tap

The tap lives in our Monado fork's compositor (`src/xrt/compositor/main/comp_recorder.c`): it
copies the squashed eye images (before the lens distortion) into shared memory without ever waiting, and drops
frames instead. monado-service gets `PIMAX_RECORD=1` and `PIMAX_RECORD_SHM=<ring>`, the path pibridge-record uses too
(`$PIMAX_RECORD_SHM`, else `$XDG_RUNTIME_DIR/pimax-record.shm`; the tap's own default is the same, `PIMAX_RECORD=0`
removes the tap). Under SteamVR, SteamVR's compositor draws the headset image and a tap sees nothing
(`--source tap` under SteamVR is refused in plain words).

**Menus fill the frame.** A game's main menu is often a single flat panel (an OpenXR quad layer) and no 3D: in the
eye images it is a small rectangle in a ~100° view. So the tap looks at every frame's layers (all clients, merged):
when there is nothing but quads, it records the largest quad's own image instead, fitted into the whole video frame
with black around it, kept as wide as the panel is in the headset and converted to sRGB like the eye images. As soon as
a frame has 3D (a projection layer: the cockpit, PiBridge World) it records the eye images again. The video keeps its
size throughout. Falcon BMS's menu and its loading splash are such quad-only frames. monado.log notes each switch
(`recorder: now recording the quad layer ...` / `... the eye images`). `PIMAX_RECORD_QUAD=0` in monado-service's
environment (`rec_env=` in pibridge.conf) always records the eye images.

**Not known yet**: whether SteamVR's Vulkan compositor on Linux shares its mirror texture with OpenGL at all. If it
does not, `GetMirrorTextureGL` returns an error (most likely `VRCompositorError_SharedTexturesNotSupported`, 106,
or `IsNotSceneApplication`, 103) and the card shows "SteamVR's mirror could not start: SteamVR's mirror texture is
not available: SteamVR does not share its mirror texture with OpenGL here (...)". The fallback then is a follow-up:
capture SteamVR's VR View window through the KDE screencast portal (PipeWire).

## Files

| File | What |
|---|---|
| `~/.config/pibridge/record.toml` | settings, `key = value` (eyes, fps, quality, scale, max_height, codec, encoder, dir, audio, mic); omarchyvr's `~/.config/omarchyvr/record.toml` is read while PiBridge's does not exist. `--save` and the card write PiBridge's. |
| `<session>/record.status` | key=value status (state, pid, file, sizes, frames, drops, note), rewritten about every second |
| `<session>/record.events` | one JSON line per event (`{"event":"notice","t":...,"id":"record","text":...}`) |
| `<session>/record.pid`, `record.log`, `record-ffmpeg.log` | the recorder's pid (checked by comm `pibridge-record`), its log, ffmpeg's log |

`<session>` is `$XDG_RUNTIME_DIR/pibridge` (`PIBRIDGE_VR_SESSION_DIR` / `PIMAX_VR_SESSION_DIR` override it).
Environment: `PIBRIDGE_FFMPEG` (or `PIMAX_FFMPEG`), `PIBRIDGE_RECORD_BIN`, `PIBRIDGE_RECORD_NO_NOTIFY`,
`PIBRIDGE_RECORD_NO_MONADO_CHECK` (tests), `PIBRIDGE_PROC_ROOT` (tests).

## Reliability

The recorder is a leaf: it never touches Monado, it only reads the ring; the tap detaches when the recorder's
heartbeat is 3 s old. It runs at nice 10 with the lowest best-effort I/O priority; ffmpeg too, with
`PR_SET_PDEATHSIG`. The mp4 is fragmented, so it plays at any moment. Every way out (stop, Monado gone, less than
1 GiB free, encoder error) finishes the file and exits 0. A hung ffmpeg is killed after 20 s without progress (5 s
after a stop). Sound: the default output's monitor (`@DEFAULT_MONITOR@`), plus the microphone when switched on;
both are test-read for 0.3 s first, and a missing one only drops the sound, never the video.

## Tests

- `pibridge_record_tool` (46 cases): twins of every Rust unit test (settings, cli, encode, shm, status, pacer), the
  ring against the fork's real `comp_recorder_shm.h` (when `PIBRIDGE_MONADO_SRC_DIR` has it), and whole recordings of
  the real binary with an in-process fake tap and `tests/fake_ffmpeg.sh`: a normal take, the NVENC fallback, no
  sound output, Monado gone, no tap. The SteamVR mirror: the pump with a fake backend (rate and size, both eyes,
  drops instead of waits, the recorder or SteamVR gone, errors), the openvrpaths lookup, a whole `--source mirror`
  recording (the fake producer as the mirror), and the real `pibridge-record-mirror` with a fake `libopenvr_api.so`
  (`tests/fake_openvr.cpp`: SteamVR not running, no compositor interface; EGL is never reached).
- The app's card: `pibridge_video_record_tests` (words, record.toml, the commands, the tap's env) and the screenshots
  `09h-dashboard-recording`, `09i-dashboard-recording-saved`.

## Not ported / not done yet

- omarchyvr's GPU safety gate (omarchyvrd): PiBridge has none, NVENC is always allowed.
- `omarchyvr record tracking` and the voice commands (data for other headsets is collected under Windows,
  [HEADSET-DATA.md](HEADSET-DATA.md)).
- The SteamVR mirror has never run against a real SteamVR: to check live (SteamVR running, nothing else on the
  GPU): does `GetMirrorTextureGL` work with the Linux Vulkan compositor (else the error above, and the VR View +
  portal follow-up); Background vs Overlay client; the EGL display (desktop vs device); the image's orientation and
  colours (sRGB); the mirror's size (per eye, distorted or not); no missed SteamVR frames while recording; a SteamVR
  quit ends the mirror and the file plays; OpenXR games under SteamVR's runtime.
