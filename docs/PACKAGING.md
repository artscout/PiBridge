# Packaging PiBridge as a Debian/Ubuntu .deb

PiBridge ships as two binary packages from one source: `pibridge` (installed under `/usr`) and `pibridge-monado`
(PiBridge's Monado fork, under `/usr/lib/pibridge/monado`). The `debian/` directory in the repo root drives them.
How a user installs them: [INSTALL.md](INSTALL.md#installing-from-the-package-ubuntu--debian).

## What's in the package

| Content | Path |
| --- | --- |
| App + engine binaries (`pibridge`, `pibridge-headtrack`, `pibridge-cam`, `pibridge-slam`, `pibridge-eyed`, `pibridge-ctrltrack`, `pibridge-roommap`, `pibridge-record`, `pibridge-overlay`, `pibridge-home`, `pibridge-roomsetup`, `pibridge-xroverlay`, ...) | `/usr/bin/` |
| `pibridge-get-godot` (fetches the official Godot 4.7.2, checked by its SHA-512; for source installs and the build machine) | `/usr/bin/` |
| PiBridge World exported into one executable (Godot's Linux release template with the project embedded; never stripped) | `/usr/lib/pibridge/pibridge-world` |
| Device descriptions (HID allowlist, `*.toml`) | `/usr/share/pibridge/devices/` |
| systemd **user** units (`pibridge-eyed.service`, `pibridge-keepalived.service`) | `/usr/lib/systemd/user/` |
| Vulkan video-capture record layer | `/usr/lib/<triplet>/pibridge/libpibridge-record-layer.so` |
| Record-layer implicit-layer manifest | `/usr/share/vulkan/implicit_layer.d/VK_LAYER_PIBRIDGE_record.json` |
| Desktop entry + icons | `/usr/share/applications/`, `/usr/share/icons/hicolor/` |
| Quad views OpenXR layer (`quadviews/`) and its implicit-layer manifest | `/usr/lib/<triplet>/qv-vulkan/`, `/usr/share/openxr/1/api_layers/implicit.d/` |
| PiBridge World's project (`world/`; run only with `rec_godot=`, the app takes `pibridge-world`) | `/usr/share/pibridge/world/` |
| Docs | `/usr/share/doc/pibridge/` |
| udev rule (canonical copy) | `/usr/share/pibridge/udev/70-pibridge.rules` (activated into `/lib/udev/rules.d/` by postinst) |
| The Crystal Super's WirePlumber rule (`patches/audio/`; `wireplumber` is a dependency) | `/usr/share/wireplumber/wireplumber.conf.d/51-pimax-crystal-super.conf` |
| The patches the manual steps use (`patches/`: CustomHeadsetOpenVR, NVIDIA, the sound rule) | `/usr/share/pibridge/patches/` |
| The EDID override scripts and the known EDIDs | `/usr/share/pibridge/edid/` |
| winbridge **sources** + build helper | `/usr/share/pibridge/winbridge-src/` (incl. `build-winbridge.sh`; not on `$PATH`) |

`pibridge-monado` (Depends: `pibridge` of the same version, `libcap2-bin`):

| Content | Path |
|---|---|
| `monado-service`, `monado-cli`, `monado-ctl`, `monado-gui` | `/usr/lib/pibridge/monado/bin/` |
| `libopenxr_monado.so`, `libmonado.so` | `/usr/lib/pibridge/monado/lib/` |
| The OpenXR runtime manifest (absolute library path) | `/usr/lib/pibridge/monado/share/openxr/1/openxr_monado.json` |

It is built from `third_party/monado` after PiBridge, linking the `libpibridge_pimax.a` of the same build, with the
switches of [INSTALL.md](INSTALL.md) 7.2.1 minus what PiBridge never uses (samples, tests, ONNX hand tracking,
Monado's SteamVR plugin, its systemd units, its headers). Its own prefix keeps it apart from a distribution's Monado;
the app looks there before `PATH`. Its postinst sets `cap_sys_nice=ep` on `monado-service` (realtime compositor
threads) on every configure, since an upgrade unpacks the binary without it. It is not the system's OpenXR runtime
by itself: the app's **VR backend** choice points `~/.config/openxr/1/active_runtime.json` at its manifest.

### Dependencies: system packages only

Everything that a distribution packages is a **system dependency**, pulled via
`${shlibs:Depends}` (Qt 6, the Vulkan loader, EGL/GL, zlib, and the OpenXR
loader *only if actually linked*) plus a few explicit packages:

- **OpenCV 4.10** — `libopencv-core410 | libopencv-core4.10` and the imgproc,
  imgcodecs, video, calib3d, features2d, dnn and flann siblings, listed
  explicitly in `debian/control`.
- **ffmpeg** — `pibridge-record` runs it (NVENC, x264 fallback).
- **wireplumber**, **pulseaudio-utils** — the sound rule is WirePlumber's; `pibridge-keepalived` switches the
  default output and input with `pactl`.
- **steam** (Recommends) — SteamVR is the runtime.
- **wine**, **mingw-w64** (Suggests) — only for the optional winbridge build.

Nothing third-party is bundled: inside-out tracking runs in `pibridge-headtrack` (our code) and SteamVR shows it
through CustomHeadsetOpenVR, which the user installs (`docs/INSTALL.md` 5.3). No ONNX Runtime is built or shipped.
PiBridge's Monado fork is ours (from source in this tree) and goes into `pibridge-monado`. Godot is not a
dependency: the world is exported at build time into `pibridge-world`, which is Godot's own release template (MIT,
godotengine.org) with our project inside.

### What the build machine needs beyond the Build-Depends

- **Godot 4.7.2** and its **export templates** (Ubuntu has no Godot 4 package): `pibridge-get-godot` (or
  `tools/get-godot/pibridge-get-godot`) puts the editor into `~/.local/share/pibridge/godot/`; `debian/rules` finds it
  there or in `~/gamedev/godot/`, or takes `PIBRIDGE_GODOT=<binary>`. The templates: download
  `Godot_v4.7.2-stable_export_templates.tpz` from the same release, check it against the release's `SHA512-SUMS.txt`,
  and unpack the Linux x86_64 ones where Godot looks for them:
  ```
  D=~/.local/share/godot/export_templates/4.7.2.stable; mkdir -p $D
  unzip -j Godot_v4.7.2-stable_export_templates.tpz templates/linux_release.x86_64 templates/linux_debug.x86_64 templates/version.txt -d $D
  ```
  Nothing is downloaded during the package build itself.

### No development folders in the package

The app's lookups end with the development machine's folders (`~/gamedev/pibridge-monado-build`, `~/gamedev/ext/...`,
`~/gamedev/godot`, ...) so an uninstalled checkout runs. `debian/rules` builds with `-DPIBRIDGE_DEV_PATHS=OFF`: the
packaged app looks only in the user's folders (`~/.config/pibridge`, `~/.local/share/pibridge/...`), `/usr` and
`PATH`.
`app/third_party/openvr/openvr.h` is vendored **build-time** headers only;
SteamVR's own `libopenvr_api.so` is dlopen'd at run time.

### Excluded on purpose

Each of these is a manual step for the user, listed in INSTALL.md's "Installing from the package":

- Pimax Play DLLs / the windata kit, Pimax's controller models: Pimax's files (INSTALL.md 7.1, world.md).
- `winbridge.exe`: built for the user by `pibridge-build-winbridge` from the shipped sources.
- The lens profiles and the camera calibration in `~/gamedev/ext/`: made from the user's own Pimax install.
- CustomHeadsetOpenVR (GPL-2, its own project; the patches are shipped), SteamVR and its `driver_lighthouse`
  (Valve's), the NVIDIA module and the EDID override (system changes, root).
- Basalt (built by the user).

## Building the .deb (this machine)

```sh
# install build-deps once:
sudo apt install cmake ninja-build g++ qt6-base-dev libopenxr-dev \
     libvulkan-dev libgl-dev libegl-dev zlib1g-dev libjpeg-dev debhelper

# plus the Monado fork's (debian/control lists them all):
sudo apt install pkg-config python3 glslang-tools libopencv-dev libeigen3-dev libusb-1.0-0-dev \
     libudev-dev libhidapi-dev libx11-dev libx11-xcb-dev libxxf86vm-dev libxrandr-dev libxcb-randr0-dev \
     libglvnd-dev libwayland-dev wayland-protocols libcjson-dev libsdl2-dev libbsd-dev libsystemd-dev libdbus-1-dev
# (or: sudo apt build-dep .)

# binary .debs (run heavy build nicely):
cd ~/gamedev/pibridge
DEB_BUILD_OPTIONS="parallel=8" nice -n 19 dpkg-buildpackage -us -uc -b
# -> ../pibridge_2.0.2_amd64.deb, ../pibridge-monado_2.0.2_amd64.deb
```

`debuild -b -us -uc` works the same. Inspect with `dpkg -c ../pibridge_*.deb ../pibridge-monado_*.deb`. The version
comes from `debian/changelog` (and `project(VERSION)` in `CMakeLists.txt`, which the app shows): bump both.

### Source package

The format is `3.0 (native)`, so a source package can be produced for others to
build on Debian/Ubuntu:

```sh
dpkg-buildpackage -us -uc -S     # -> ../pibridge_2.0.2.dsc + .tar.xz
```

(`debian/source/options` keeps the tarball lean — no `build*/`, `.git`, caches
or Windows build outputs.)

## Build from source / other distros

### PiBridge itself

System packages to build the app and engine binaries:

- **Ubuntu/Debian:**
  `apt install cmake ninja-build g++ qt6-base-dev libopenxr-dev zlib1g-dev libjpeg-dev libvulkan-dev libgl-dev libegl-dev libopencv-dev ffmpeg mingw-w64 wine`
- **Arch:**
  `pacman -S cmake ninja gcc qt6-base openxr zlib libjpeg-turbo vulkan-icd-loader vulkan-headers opencv ffmpeg mingw-w64-gcc wine`
- **Fedora:**
  `dnf install cmake ninja-build gcc-c++ qt6-qtbase-devel openxr-devel zlib-devel libjpeg-turbo-devel vulkan-loader-devel mesa-libGL-devel mesa-libEGL-devel opencv-devel ffmpeg mingw64-gcc-c++ wine`

```sh
cmake -S . -B build-deb -G Ninja -DCMAKE_INSTALL_PREFIX=/usr \
      -DPIBRIDGE_SYSTEMD_USER_UNIT_DIR=lib/systemd/user -DPIBRIDGE_BUILD_TESTS=OFF
nice -n 19 cmake --build build-deb -j8
```

Note: `ffmpeg`, Vulkan and OpenCV are **system packages** everywhere. `openxr`
is used for **headers** (the app links the loader only if present).

### Basalt (optional)

**Basalt** (`libbasalt.so`) is not a distribution package and is not shipped. It is
only needed for the open SLAM engine, the fallback (the default inside-out path
uses the Pimax engine via winbridge); build it separately and point the app at it
(`rec_basalt_lib`, `docs/INSTALL.md` 7.1).

## The optional inside-out (Pimax-engine) path: winbridge

winbridge is **our** code (shipped as source). The proprietary Pimax DLLs it
loads are **not** shipped. After installing the package, build it once:

```sh
pibridge-build-winbridge   # = sh /usr/share/pibridge/winbridge-src/build-winbridge.sh; needs mingw-w64, wine at run time
```

It compiles, from `/usr/share/pibridge/winbridge-src/`:

```
x86_64-w64-mingw32-g++ -O2 -static server.cpp -lws2_32 -o winbridge.exe
g++ -O2 -std=c++20 -fPIC -shared vit_winbridge.cpp -o libpibridge_vit_winbridge.so -pthread
```

and installs the two outputs into `~/.local/share/pibridge/winbridge/`, where
the app looks for them. Nothing starts it as a service: the app runs
`wine winbridge.exe` inside each inside-out tracking session (START VR)
and stops it with the session. See `docs/WINBRIDGE.md` for the full picture and
`docs/INSTALL.md` section 7 for the Pimax kit and the user steps.

## After install

`pibridge`'s `postinst` activates the udev rule (copies it to `/lib/udev/rules.d/` and reloads udev) and prints what
is left to do by hand. debhelper's `dh_installsystemduser` enables the two user units for every user (`systemctl
--global`): they start at the next login. `pibridge-monado`'s `postinst` sets `cap_sys_nice` on `monado-service`.
Everything else (settings, rooms, the Pimax kit, lens profiles, models) is per user, in the home folder.
