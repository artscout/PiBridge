# Packaging PiBridge as a Debian/Ubuntu .deb

PiBridge ships as a single binary package, `pibridge`, installed under `/usr`.
The `debian/` directory in the repo root drives it.

## What's in the package

| Content | Path |
| --- | --- |
| App + engine binaries (`pibridge`, `pibridge-headtrack`, `pibridge-cam`, `pibridge-slam`, `pibridge-eyed`, `pibridge-ctrltrack`, `pibridge-roommap`, `pibridge-record`, `pibridge-overlay`, `pibridge-home`, `pibridge-roomsetup`, ...) | `/usr/bin/` |
| Device descriptions (HID allowlist, `*.toml`) | `/usr/share/pibridge/devices/` |
| systemd **user** units (`pibridge-eyed.service`, `pibridge-keepalived.service`) | `/usr/lib/systemd/user/` |
| Vulkan video-capture record layer | `/usr/lib/<triplet>/pibridge/libpibridge-record-layer.so` |
| Record-layer implicit-layer manifest | `/usr/share/vulkan/implicit_layer.d/VK_LAYER_PIBRIDGE_record.json` |
| Desktop entry + icons | `/usr/share/applications/`, `/usr/share/icons/hicolor/` |
| Quad views OpenXR layer (`quadviews/`) and its implicit-layer manifest | `/usr/lib/<triplet>/qv-vulkan/`, `/usr/share/openxr/1/api_layers/implicit.d/` |
| PiBridge World (`world/`, the Monado backend's home; a Godot project, run by the user's Godot) | `/usr/share/pibridge/world/` |
| Docs | `/usr/share/doc/pibridge/` |
| udev rule (canonical copy) | `/usr/share/pibridge/udev/70-pibridge.rules` (activated into `/lib/udev/rules.d/` by postinst) |
| winbridge **sources** + build helper | `/usr/share/pibridge/winbridge-src/` (incl. `build-winbridge.sh`; not on `$PATH`) |

### Dependencies: system packages only

Everything that a distribution packages is a **system dependency**, pulled via
`${shlibs:Depends}` (Qt 6, the Vulkan loader, EGL/GL, zlib, and the OpenXR
loader *only if actually linked*) plus a few explicit packages:

- **OpenCV 4.10** — `libopencv-core410 | libopencv-core4.10` and the imgproc,
  imgcodecs, video, calib3d, features2d, dnn and flann siblings, listed
  explicitly in `debian/control`.
- **ffmpeg** — `pibridge-record` runs it (NVENC, x264 fallback).
- **steam** (Recommends) — SteamVR is the runtime.
- **wine**, **mingw-w64** (Suggests) — only for the optional winbridge build.

Nothing is bundled: inside-out tracking runs in `pibridge-headtrack` (our code)
and SteamVR shows it through CustomHeadsetOpenVR, which the user installs
(`docs/INSTALL.md` 5.3). No ONNX Runtime is built or shipped. PiBridge's Monado
fork is in the source tree (`third_party/monado`) but is not part of the
package yet: the Monado backend's user builds it (`docs/INSTALL.md` 7.2.1).
Godot is not a dependency either: the user downloads it for PiBridge World
(`docs/INSTALL.md` 7.2.2); without it the classic `pibridge-home` is the home.
`app/third_party/openvr/openvr.h` is vendored **build-time** headers only;
SteamVR's own `libopenvr_api.so` is dlopen'd at run time.

### Excluded on purpose

Pimax Play DLLs / the windata kit, `winbridge.exe` build
output, and the external lens/calibration data in `~/gamedev/ext/`. The Pimax
kit is a manual post-install step — see `docs/INSTALL.md` §7.

## Building the .deb (this machine)

```sh
# install build-deps once:
sudo apt install cmake ninja-build g++ qt6-base-dev libopenxr-dev \
     libvulkan-dev libgl-dev libegl-dev zlib1g-dev libjpeg-dev debhelper

# binary .deb (run heavy build nicely):
cd ~/gamedev/pibridge
DEB_BUILD_OPTIONS="parallel=8" nice -n 19 dpkg-buildpackage -us -uc -b
# -> ../pibridge_1.0.1_amd64.deb
```

`debuild -b -us -uc` works the same. Inspect with `dpkg -c ../pibridge_*.deb`.

### Source package

The format is `3.0 (native)`, so a source package can be produced for others to
build on Debian/Ubuntu:

```sh
dpkg-buildpackage -us -uc -S     # -> ../pibridge_1.0.1.dsc + .tar.xz
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

## After install: per-user setup

`postinst` only activates the udev rule (copies it to `/lib/udev/rules.d/` and
reloads udev) and prints a note. The rest is **per-user** and is done by the
app on first run: enabling the systemd *user* services. No system
`daemon-reload` is needed (the units are user units). Just launch **PiBridge**.
