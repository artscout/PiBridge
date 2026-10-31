<p align="center"><img src="docs/brand/pibridge-logo.png" alt="PiBridge logo: a cartoon fly with pibridgeed eyes and a VR headset" width="200"></p>

# PiBridge

**VR hub for Pimax headsets on Linux. Not affiliated with Pimax.**

PiBridge runs a Pimax headset on Linux the way Pimax Play does on Windows: one desktop app, plus two small user
services that wake the headset and run its eye tracker while VR runs. The app shows the headset, the controllers and
base stations, the eye tracker and the quad views (foveated rendering) settings, and changes the settings the services
share. It never talks to the hardware itself.

VR runs on one of two backends, chosen on the Dashboard:

- **SteamVR**, with CustomHeadsetOpenVR as the headset driver. This is the default and the most tested path.
- **Monado**: PiBridge's own Monado fork ("Monado PiBridge fork from OmarchyVR", in `third_party/monado`) as the
  OpenXR runtime. In the headset, **PiBridge World** is the home: a Godot world with the dashboard, your apps,
  passthrough and the settings. Over a game, the menu button opens PiBridge's dashboard with Recenter, Quick room setup
  and Close app.

Quad views (eye-tracked foveated rendering for Vulkan games) come from PiBridge's own OpenXR layer in `quadviews/`,
built and installed with the app.

**Status: alpha.** It is tested on one PC with two headsets (Pimax Crystal Super micro-OLED with the Lighthouse
faceplate, and the original Pimax Crystal in both its tracking modes; RTX 5090, Ubuntu 26.04). Expect rough edges.

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
app, the SteamVR device status, the quad views layer and page, the Monado backend with PiBridge World, and packaging
for Linux only. We keep sending our hardware
findings back to omarchyvr.

The Monado backend runs on [Monado](https://monado.freedesktop.org/) by Collabora and the Monado contributors (BSL-1.0):
PiBridge's fork in `third_party/monado` builds on omarchyvr's Monado fork and its Pimax driver. PiBridge World runs on
[Godot](https://godotengine.org/) (MIT), which you download yourself.

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
| Pimax Crystal Super, inside-out (no faceplate) | SLAM on the headset cameras (Pimax Play's engine under Wine, or Basalt), controllers by their LEDs | SteamVR via CustomHeadsetOpenVR + our patches (`pibridge-headtrack` feeds the poses), no Monado | yes | **experimental**: head tracking usable on a worn headset, controllers not checked live yet. On a USB 2.0 port or hub the tracking cameras come up as Motion-JPEG instead of YUYV (480 Mbps cannot carry the uncompressed stream), which loses the metadata block, so that path runs without device timestamps, auto-exposure and the controller cameras; a SuperSpeed port gives YUYV back. See `docs/TRACKING.md` |
| Pimax Crystal (original, "P3B"), Lighthouse (switched on by software, no faceplate swap) | SteamVR Lighthouse: Index controllers, base stations | CustomHeadsetOpenVR + our patches; 72/90/120 Hz; sound over DisplayPort, its microphone | yes (its Tobii chip, started by PiBridge) | **works** (one headset tested) |
| Pimax Crystal (original), its own inside-out | the headset tracks itself (head and Crystal controllers); PiBridge passes the poses to CustomHeadsetOpenVR, no Monado | CustomHeadsetOpenVR + our patches | yes | **works** (one headset tested): head, controllers with all buttons and the stick, battery. Not yet: controller vibration; the controllers are drawn as Quest 2 controllers. See [docs/INSTALL.md](docs/INSTALL.md) 5.5 |
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
| Monado backend | works (tested in a headset), newer than SteamVR | PiBridge's Monado fork: inside-out or Lighthouse tracking, eye gaze, quad views, its own rooms; [docs/monado.md](docs/monado.md), [docs/INSTALL.md](docs/INSTALL.md) 7.2 |
| PiBridge World | works (tested in a headset) | the Monado backend's home in the headset, a Godot 4 project (Godot is downloaded by you): dashboard, Apps tiles with Run and Edit, passthrough, backgrounds, quad views settings; it gives a running game its memory and comes back after; [docs/world.md](docs/world.md) |
| Dashboard over games (Monado) | works | the menu button over a game: settings and a Recording page, and along the bottom Record video / Stop recording, Quick room setup, Recenter, Close app; [docs/home.md](docs/home.md) |
| Recenter | works | holding the Pimax button or the dashboards' Recenter: forward where you look, for every app (also seated ones on Monado); a ring shows the hold |
| OpenXR runtime switch | works | the Dashboard points the system's active OpenXR runtime at SteamVR or Monado |
| Eye calibration | works (new) | `pibridge-eyecal`: the eye tracker's own calibration, as Pimax Play does it; one Windows session is still needed for the start list |
| Quad views | works | the layer in `quadviews/` (built with PiBridge) and the Performance page; under Monado the focus area follows the gaze through Monado's quad views |
| SteamVR device status | works | controllers, battery, base stations |
| Apps page | works | Steam games and your own programs; own apps wait until SteamVR is ready |
| SteamVR dashboard panel | works (tested in a headset) | `pibridge-overlay`: an in-VR panel with Room setup, quick room setup and one record button (START/STOP recording), toggled from Dashboard → Quick settings; [docs/OVERLAY.md](docs/OVERLAY.md) |
| Room setup | works (tested in a headset) | `pibridge-roomsetup` (in the headset, port of ZoneTweak) and the Room page; SteamVR's chaperone file |
| Video recording | works | `pibridge-record` into NVENC/CPU mp4: under SteamVR from a Vulkan capture layer (`tools/record-layer`) in SteamVR's compositor, under Monado from a tap in our Monado's compositor; Dashboard Record card, the overlays, PiBridge World; [docs/record.md](docs/record.md) |
| Inside-out tracking (SLAM) | experimental | see [SLAM status](#slam-status) |

## SLAM status

**How we do it.** One process, `pibridge-headtrack`, owns the headset, tracks the head and the controllers and writes
the poses to a shared memory file; CustomHeadsetOpenVR (with `patches/chovr/chovr-linux-headtrack.patch`) shows them in
SteamVR, so games start through SteamVR as usual, the same way as the original Crystal's own inside-out
([docs/pibridge-headtrack.md](docs/pibridge-headtrack.md)). On the Monado backend the same engines run inside
PiBridge's Monado instead ([docs/monado.md](docs/monado.md)). The headset's cameras and IMU feed one of
two interchangeable head-tracking engines, chosen on the Tracking page:

- **Pimax Play's own engine** — the default, for parity with Windows: `pimax_slam.pi.dll` run under Wine in a small
  host (`winbridge`) and reached through a VIT library, with the headset's factory calibration — the same tracking
  Pimax Play uses on Windows ([docs/WINBRIDGE.md](docs/WINBRIDGE.md)).
- **Basalt** — open source, the fallback: the Basalt visual-inertial tracker (loaded directly as a VIT library) on the
  two lower cameras plus the IMU, with a config PiBridge generates from the headset calibration.

Controllers are found by their LEDs in the cameras (Pimax's LED engine in the same winbridge, or our
`pibridge-ctrltrack` + IMU fusion). The lens distortion/FOV is a selectable profile on the Display page (Crystal Super
Oled native: the Pimax compositor geometry; Crystal Super Oled chovr: CustomHeadsetOpenVR's lens shape), both with
each eye's axis set by a true rotation.

Status on the Crystal Super (no faceplate), as of 2026-10-19:

- **Head tracking works on a worn headset** (Basalt, recorded and checked offline): no tracking losses, 5-7 cm drift over a take, 3-5 mm/s creep standing still, about 1 degree tilt error.
  What made it usable: camera auto exposure with the Windows exposure and gain ranges (the images were too dark
  before), the factory accelerometer bias plus a measured gyro bias in Basalt's calibration, and a finer feature
  grid. The output layer's drift hold is off for the Crystal (it made the error worse: 273 mm instead of 74 mm).
  Notes: [docs/research/slam-imu-intrinsics.md](docs/research/slam-imu-intrinsics.md).
- **Controllers** are found by their LEDs in the cameras (`pibridge-ctrltrack`, 86-100 % of the frames in recordings) and
  fused with their IMU; not checked live in the headset yet.
- **From the app:** START VR on the Dashboard (with Inside-out chosen) starts the camera broker, `pibridge-headtrack`
  and SteamVR; the Tracking page holds the settings.
- **No Rust:** omarchyvr's Pimax libraries (headset, cameras, auto exposure, lenses, controllers, hands) are ported to
  C++, byte for byte against the Rust originals (every ported test has a C++ twin); nothing Rust is built or shipped.
  Plan and results: [docs/research/rust-port-plan.md](docs/research/rust-port-plan.md).
- **Open:** no persistent world anchor yet (the room boundary can turn or shift between sessions), the gyro bias
  takes about 40 s to settle after a start, and longer walking takes are still to be recorded.

## Requirements

| What | Tested | Notes |
|---|---|---|
| GPU and driver | NVIDIA RTX 5090, open modules 615.71.09+ (or 595.99.02) | **615.71.09+ needs no patch** (the Blackwell DSC fix is upstream); 595.x needs the FEC patch from `patches/nvidia/`; every driver: the EDID override (`pibridge-headset-edid`, docs/INSTALL.md 5.2); other GPUs untested |
| Distribution | Ubuntu 26.04, KDE Wayland | package names for Arch and Fedora are in the guide, not tried on hardware |
| SteamVR | Steam's SteamVR for Linux | the SteamVR backend: PiBridge starts it with START VR; the Monado backend with the Lighthouse faceplate only loads its Lighthouse driver |
| CustomHeadsetOpenVR | upstream `4f25826` + `patches/chovr/` | or the `linux-crystal-super` branch of artscout/CustomHeadsetOpenVR |
| Build | CMake 3.22+, Ninja, GCC 13+ or Clang 17+, Qt 6 Widgets, OpenXR headers, Vulkan headers and glslang | libjpeg (the Motion-JPEG camera profile), zlib and IBM Plex fonts optional |
| Monado backend | optional | the `pibridge-monado` package, or from source: Monado's build dependencies, the fork is in `third_party/monado` (docs/INSTALL.md 7.2) |
| Godot 4.7 | optional, for PiBridge World | the official Linux build, fetched and checked by `pibridge-get-godot`; not shipped with PiBridge |

## Quick start (Ubuntu)

From the packages (`pibridge` and, for the Monado backend, `pibridge-monado`; built with `dpkg-buildpackage`,
[docs/PACKAGING.md](docs/PACKAGING.md)):
```
sudo apt install ./pibridge_2.0.2_amd64.deb ./pibridge-monado_2.0.2_amd64.deb
systemctl --user daemon-reload && systemctl --user start pibridge-keepalived pibridge-eyed
pibridge
```
The packages set up the udev rule, the services, the sound rule and Monado's realtime priority; what is left by hand
(EDID override, CustomHeadsetOpenVR, models, lens profile, eye profile) is listed in
[docs/INSTALL.md](docs/INSTALL.md#installing-from-the-package-ubuntu--debian).

From source:
```
sudo apt install cmake ninja-build g++ qt6-base-dev libopenxr-dev libvulkan-dev glslang-tools zlib1g-dev libjpeg-dev fonts-ibm-plex
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/.local
cmake --build build && ctest --test-dir build
cmake --install build && systemctl --user daemon-reload
sudo cp ~/.local/share/pibridge/udev/70-pibridge.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger
systemctl --user enable --now pibridge-keepalived pibridge-eyed
pibridge
```

That gives the app, the services and the quad views layer. The Crystal Super also needs the EDID override,
CustomHeadsetOpenVR, the sound rule and (for eye tracking) your eye profile. The Monado backend needs its build of
`third_party/monado` and Godot for PiBridge World. **Full guide: [docs/INSTALL.md](docs/INSTALL.md)** (Arch and Fedora,
the Crystal Super steps, eye tracking, the Monado backend, checks, uninstall).

## More

- [docs/USER-GUIDE.md](docs/USER-GUIDE.md): the overview — one app, one click, and what each page does.
- [docs/INSTALL.md](docs/INSTALL.md): install, set up, check, uninstall.
- [docs/CONVENTIONS.md](docs/CONVENTIONS.md): layout of the tree, rules, file formats.
- [docs/app.md](docs/app.md) and the other `docs/*.md`: one page per part.
- [docs/OVERLAY.md](docs/OVERLAY.md): the in-VR SteamVR dashboard panel (room setup, record / stop).
- [docs/monado.md](docs/monado.md): PiBridge's Monado fork; [docs/world.md](docs/world.md): PiBridge World;
  [docs/home.md](docs/home.md): the dashboard in the headset and over games.
- [patches/README.md](patches/README.md): the setup patches, where they come from and their licences.

## Licence and trademarks

PiBridge is MIT licensed ([LICENSE](LICENSE)); third-party parts and their licences are listed in [NOTICE](NOTICE).
PiBridge is an independent project: it is not made, endorsed or supported by Pimax. Pimax, Crystal Super and Dream Air
are trademarks of their owners and are used here only to say which hardware PiBridge works with.
