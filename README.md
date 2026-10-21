<p align="center"><img src="docs/brand/pibridge-logo.png" alt="PiBridge logo: a cartoon fly with pibridgeed eyes and a VR headset" width="200"></p>

# PiBridge

**VR hub for Pimax headsets on Linux. Not affiliated with Pimax.**

PiBridge runs a Pimax headset on Linux the way Pimax Play does on Windows: one desktop app, plus two small user
services that wake the headset and run its eye tracker while SteamVR runs. The app shows the headset, the
controllers and base stations, the eye tracker and the quad views (foveated rendering) settings, and changes the
settings the services share. It never talks to the hardware itself. Quad views (foveated rendering) come from our
OpenXR layer [linux-vulkan-quadviews-openxr-layer](https://github.com/artscout/linux-vulkan-quadviews-openxr-layer)
(a separate repository); PiBridge only shows and changes its settings.

**Status: alpha.** It is tested on one PC with one headset (Pimax Crystal Super micro-OLED with the Lighthouse
faceplate, RTX 5090, Ubuntu 26.04). Expect rough edges.

## Why PiBridge exists

PiBridge is a stopgap and a demonstration. It shows Pimax that their headsets can work on Linux: the display, the
eye tracker, Lighthouse and inside-out tracking, the controllers. What we really want is proper Linux support
from Pimax itself, and we are waiting for it. Until then, PiBridge is a temporary solution for people who want to use
their Pimax headset on Linux today. If Pimax ships official Linux support, PiBridge has done its job, and
everything here is open (MIT) for them to use.

PiBridge was written with AI: the code, the tests, the reverse engineering analysis and the docs were made with
Anthropic's Claude (Claude Code), with a human testing every step on real hardware. It took a couple of days of real
time to get here, not counting the week of work that went into omarchyvr, which PiBridge builds on.

## Built on omarchyvr

PiBridge is based on [omarchyvr](https://github.com/laughtt/omarchyvr) by laughtt (José Mauricio Carpio Herrera) and
contributors, licensed MIT OR Apache-2.0 (used here under MIT, see [NOTICE](NOTICE)). From omarchyvr come:

- the reverse-engineered Pimax USB protocol: headset wake, keepalive, panel and video on, IMU frames, the HID
  allowlist and the device descriptions (`devices/`, unchanged);
- the Tobii eye chip's start and stream protocol;
- the gaze math and the calibration file format;
- the keepalive and eye tracker daemons.

PiBridge ported these from Rust to C++, with omarchyvr's tests as the specification (every ported test has a C++ twin).
The Crystal Super work (eye tracking under SteamVR, the profile upload, the Pimax-style calibration, a keepalive that
follows SteamVR) was done in our omarchyvr fork first and sent back to omarchyvr. New in PiBridge: the C++ port, the Qt
app, the SteamVR device status, the quad views page, and packaging for Linux only. We keep sending our hardware
findings back to omarchyvr.

Also credited: [CustomHeadsetOpenVR](https://github.com/sboys3/CustomHeadsetOpenVR) by sboys3, the SteamVR display
driver that PiBridge's setup uses (our Linux patches are in `patches/chovr/`), and mbucchia's
[Quad-Views-Foveated](https://github.com/mbucchia/Quad-Views-Foveated) for the quad views design (design only; no code
is taken from it).

The room setup (`pibridge-roomsetup`, the in-headset boundary editor, and the Room page) is a C++ port of
[ZoneTweak](https://github.com/killdano/ZoneTweak) by killdano (MIT, see [NOTICE](NOTICE)): its data model, editing
tools (corner posts, play area handles, fit and centre, warning distance, saved spaces, backups and restore) and its
pixel font, for SteamVR's chaperone file on Linux. ZoneTweak's set dressing (the prison, robots, sound) is not ported.

## Supported headsets

| Headset | Tracking | Display | Eye tracking | Status |
|---|---|---|---|---|
| Pimax Crystal Super (micro-OLED), Lighthouse faceplate | SteamVR Lighthouse: Index controllers, base stations | CustomHeadsetOpenVR + our patches; wake, keepalive, panel 72/90 Hz; headset audio | yes, with your own Tobii profile | **works** (one headset tested) |
| Pimax Crystal Super, inside-out (no faceplate) | SLAM on the headset cameras (Pimax Play's engine under Wine, or Basalt), controllers by their LEDs | SteamVR via our bundled Monado driver | through the Pimax driver in Monado | **experimental**: head tracking usable on a worn headset, controllers not checked live yet |
| Pimax Dream Air | | | | protocol and device description present (from omarchyvr); not tested with PiBridge |
| Other Pimax models | | | | not supported yet: data wanted, see [docs/HEADSET-DATA.md](docs/HEADSET-DATA.md) |

### Your headset is not supported yet?

You can help add it. [docs/HEADSET-DATA.md](docs/HEADSET-DATA.md) explains how to record your headset's data under
Windows: one guided script, about 15 minutes. You send the recording to us privately. We publish only the command
bytes we derive from it, never the recording.

## Features

| Feature | Status | Notes |
|---|---|---|
| Wake and keepalive | works | `pibridge-keepalived`, only while SteamVR runs; standby (green LED) after |
| Panel on, refresh rate | works | 72 or 90 Hz, switch in the app |
| Audio switch | works | the headset is the default output while SteamVR runs |
| Eye tracking to SteamVR | works | `pibridge-eyed`; needs the CustomHeadsetOpenVR patch (`XR_EXT_eye_gaze_interaction`) |
| Eye calibration | works (new) | `pibridge-eyecal`: the eye tracker's own calibration, as Pimax Play does it; one Windows session is still needed for the start list |
| Quad views page | works | the layer itself: [linux-vulkan-quadviews-openxr-layer](https://github.com/artscout/linux-vulkan-quadviews-openxr-layer) |
| SteamVR device status | works | controllers, battery, base stations |
| Apps page | works | Steam games and your own programs; own apps wait until SteamVR (or Monado) is ready |
| SteamVR dashboard panel | new, not tested in a headset yet | `pibridge-overlay`: an in-VR panel (Room setup, quick room setup, record / stop video), toggled from Dashboard → Quick settings; [docs/OVERLAY.md](docs/OVERLAY.md) |
| Room setup | new, not tested in a headset yet | `pibridge-roomsetup` (in the headset, port of ZoneTweak) and the Room page; SteamVR's chaperone file |
| Inside-out tracking (SLAM) | experimental | see [SLAM status](#slam-status) |

## SLAM status

**How we do it.** Inside-out runs the Pimax driver inside Monado, which SteamVR loads as its headset driver
(`driver_monado.so` in vrserver), so games start through SteamVR as usual — no separate runtime. Our Monado (the Pimax
Crystal Super driver linked against PiBridge's C++ `libpibridge_pimax.a`, no Rust) is **bundled in this repo** at
`third_party/monado/` (see [docs/INSTALL.md](docs/INSTALL.md) section 7). The headset's cameras and IMU feed one of two
interchangeable head-tracking engines, chosen on the Tracking page:

- **Pimax Play's own engine** — the default, for parity with Windows: `pimax_slam.pi.dll` run under Wine in a small
  host (`winbridge`) and reached through a VIT library, with the headset's factory calibration — the same tracking
  Pimax Play uses on Windows ([docs/WINBRIDGE.md](docs/WINBRIDGE.md)).
- **Basalt** — open source: Monado's Basalt visual-inertial tracker on the two lower cameras plus the IMU, with a
  config PiBridge generates from the headset calibration.

Controllers are found by their LEDs in the cameras (Pimax's LED engine in the same winbridge, or our
`pibridge-ctrltrack` + IMU fusion). The lens distortion/FOV is a selectable profile (Default / like-Pimax / the raw
Pimax compositor mesh).

Status on the Crystal Super (no faceplate), as of 2026-10-19:

- **Head tracking works on a worn headset** (recorded with the Development page's guided Record tracking, then checked
  offline): no tracking losses, 5-7 cm drift over a take, 3-5 mm/s creep standing still, about 1 degree tilt error.
  What made it usable: camera auto exposure with the Windows exposure and gain ranges (the images were too dark
  before), the factory accelerometer bias plus a measured gyro bias in Basalt's calibration, and a finer feature
  grid. The output layer's drift hold is off for the Crystal (it made the error worse: 273 mm instead of 74 mm).
  Notes: [docs/research/slam-imu-intrinsics.md](docs/research/slam-imu-intrinsics.md).
- **Controllers** are found by their LEDs in the cameras (`pibridge-ctrltrack`, 86-100 % of the frames in recordings) and
  fused with their IMU; not checked live in the headset yet.
- **From the app:** the Tracking page's START TRACKING runs SteamVR with Monado's driver (`driver_monado`) instead of
  CustomHeadsetOpenVR, so games start as usual (in progress). "Monado directly" (no SteamVR; own OpenXR apps,
  `pibridge-home` as a home scene) is on the Development page for debugging. Neither is tested in a headset yet.
- **No Rust left:** the Pimax libraries Monado links (headset, cameras, auto exposure, lenses, controllers, hands) are
  ported to C++ (`libpibridge_pimax.a`), byte for byte against the Rust originals; a Monado build linked with it gives
  identical tracking offline. The live A/B in the headset is next.
  Plan and results: [docs/research/rust-port-plan.md](docs/research/rust-port-plan.md).
- **Open:** no persistent world anchor yet (the room boundary can turn or shift between sessions), the gyro bias
  takes about 40 s to settle after a start, and longer walking takes are still to be recorded.

## Requirements

| What | Tested | Notes |
|---|---|---|
| GPU and driver | NVIDIA RTX 5090, open modules 595.99.02 | needs the FEC patch and an EDID override from `patches/nvidia/`; other GPUs untested |
| Distribution | Ubuntu 26.04, KDE Wayland | package names for Arch and Fedora are in the guide, not tried on hardware |
| SteamVR | Steam's SteamVR for Linux | PiBridge does not start or change SteamVR on its own |
| CustomHeadsetOpenVR | upstream `4f25826` + `patches/chovr/` | or the `linux-crystal-super` branch of artscout/CustomHeadsetOpenVR |
| Quad views layer | optional | [linux-vulkan-quadviews-openxr-layer](https://github.com/artscout/linux-vulkan-quadviews-openxr-layer) |
| Build | CMake 3.22+, Ninja, GCC 13+ or Clang 17+, Qt 6 Widgets, OpenXR headers | zlib and IBM Plex fonts optional |

## Quick start (Ubuntu)

```
sudo apt install cmake ninja-build g++ qt6-base-dev libopenxr-dev zlib1g-dev fonts-ibm-plex
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/.local
cmake --build build && ctest --test-dir build
cmake --install build && systemctl --user daemon-reload
sudo cp ~/.local/share/pibridge/udev/70-pibridge.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger
systemctl --user enable --now pibridge-keepalived pibridge-eyed
pibridge
```

That gives the app and the services. The Crystal Super also needs the NVIDIA patch, CustomHeadsetOpenVR, the sound
rule and (for eye tracking) your eye profile. **Full guide: [docs/INSTALL.md](docs/INSTALL.md)** (Arch and Fedora,
the Crystal Super steps, eye tracking, checks, uninstall).

## More

- [docs/USER-GUIDE.md](docs/USER-GUIDE.md): the overview — one app, one click, and what each page does.
- [docs/INSTALL.md](docs/INSTALL.md): install, set up, check, uninstall.
- [docs/CONVENTIONS.md](docs/CONVENTIONS.md): layout of the tree, rules, file formats.
- [docs/app.md](docs/app.md) and the other `docs/*.md`: one page per part.
- [docs/OVERLAY.md](docs/OVERLAY.md): the in-VR SteamVR dashboard panel (room setup, record / stop).
- [patches/README.md](patches/README.md): the setup patches, where they come from and their licences.

## Licence and trademarks

PiBridge is MIT licensed ([LICENSE](LICENSE)); third-party parts and their licences are listed in [NOTICE](NOTICE).
PiBridge is an independent project: it is not made, endorsed or supported by Pimax. Pimax, Crystal Super and Dream Air
are trademarks of their owners and are used here only to say which hardware PiBridge works with.
