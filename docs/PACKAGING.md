# Packaging PiBridge as a Debian/Ubuntu .deb

PiBridge ships as a single binary package, `pibridge`, installed under `/usr`.
The `debian/` directory in the repo root drives it.

## What's in the package

| Content | Path |
| --- | --- |
| App + engine binaries (`pibridge`, `pibridge-slam`, `pibridge-eyed`, `pibridge-ctrltrack`, `pibridge-roommap`, `pibridge-record`, `pibridge-overlay`, `pibridge-home`, `pibridge-roomsetup`, ...) | `/usr/bin/` |
| Device descriptions (HID allowlist, `*.toml`) | `/usr/share/pibridge/devices/` |
| systemd **user** units (`pibridge-eyed.service`, `pibridge-keepalived.service`) | `/usr/lib/systemd/user/` |
| Vulkan video-capture record layer | `/usr/lib/<triplet>/pibridge/libpibridge-record-layer.so` |
| Record-layer implicit-layer manifest | `/usr/share/vulkan/implicit_layer.d/VK_LAYER_PIBRIDGE_record.json` |
| Desktop entry + icons | `/usr/share/applications/`, `/usr/share/icons/hicolor/` |
| Docs | `/usr/share/doc/pibridge/` |
| udev rule (canonical copy) | `/usr/share/pibridge/udev/70-pibridge.rules` (activated into `/lib/udev/rules.d/` by postinst) |
| **Bundled** Monado SteamVR driver (prebuilt, stripped) | `/usr/lib/pibridge/steamvr-monado/` |
| **Bundled** ONNX Runtime | `/usr/lib/pibridge/libonnxruntime.so*` |
| winbridge **sources** + build helper | `/usr/share/pibridge/winbridge-src/` (incl. `build-winbridge.sh`; not on `$PATH`) |

### Dependencies: system first, bundle only the unpackaged

Everything that a distribution packages is a **system dependency**, pulled via
`${shlibs:Depends}` (Qt 6, the Vulkan loader, EGL/GL, zlib, and the OpenXR
loader *only if actually linked*) plus a few explicit packages:

- **OpenCV 4.10** — `libopencv-core410 | libopencv-core4.10` and the imgproc,
  imgcodecs, video, calib3d, features2d, dnn and flann siblings. These are
  listed explicitly because the one thing that links OpenCV is the *bundled*
  Monado driver blob, which `dh_shlibdeps` does not scan.
- **ffmpeg** — `pibridge-record` runs it (NVENC, x264 fallback).
- **steam** (Recommends) — SteamVR is the runtime.
- **wine**, **mingw-w64** (Suggests) — only for the optional winbridge build.

Only **two** things are bundled, because they are not distribution packages:

1. the **PiBridge Monado SteamVR driver** (our build), and
2. **libonnxruntime** (ONNX Runtime, required by the Pimax driver for Mercury
   hand tracking).

`monado-service` is **not** shipped (the SteamVR-driver path does not need it).
`app/third_party/openvr/openvr.h` is vendored **build-time** headers only;
SteamVR's own `libopenvr_api.so` is dlopen'd at run time.

### Excluded on purpose

Mercury hand models, Pimax Play DLLs / the windata kit, `winbridge.exe` build
output, and the external lens/calibration data in `~/gamedev/ext/`. The Pimax
kit is a manual post-install step — see `docs/INSTALL.md` §7.

## Building the .deb (this machine)

The binary package stages the **prebuilt** Monado driver from
`~/gamedev/pibridge-monado-build/steamvr-monado` and ONNX Runtime from
`~/gamedev/ovr-build/onnxruntime/lib`. Both paths are overridable in
`debian/rules` (`PIBRIDGE_MONADO_PREBUILT`, `PIBRIDGE_ONNX_LIBDIR`).

```sh
# install build-deps once:
sudo apt install cmake ninja-build g++ qt6-base-dev libopenxr-dev \
     libvulkan-dev libgl-dev libegl-dev zlib1g-dev libjpeg-dev patchelf debhelper

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

> **Honest caveat.** The native source package does **not** carry the prebuilt
> Monado driver or ONNX Runtime (they live outside the repo). A from-source
> build on another machine must build `third_party/monado/` itself and obtain
> ONNX Runtime separately (see below); the current `debian/rules` only stages
> the prebuilt blob and will fail with a clear message if it is absent. Wiring
> the from-source Monado build into `debian/rules` is a documented TODO there.

## Build from source / other distros

### 1. PiBridge itself (needs no bundled blob to compile)

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

### 2. The bundled Monado SteamVR driver

The Monado fork (with the Pimax Crystal Super driver) is vendored in
`third_party/monado/` — no clone needed. Build it out-of-tree with
`PIMAX_USE_PIBRIDGE=ON`; the exact CMake options and the cache-copy method are
in **`docs/monado-abi.md`**.

System build-deps Monado needs (names vary by distro): `libudev`/eudev,
`vulkan-headers` + the Vulkan loader, `glslang`/`glslc` (shader compiler),
`libusb-1.0`, `libxcb`/`wayland` dev, `eigen3`, and **OpenCV** (all system
packages).

Honest gaps — **NOT** distribution packages, obtain separately:

- **ONNX Runtime** — *required* by the Pimax driver (Mercury hand tracking).
  Download a release (`onnxruntime-linux-x64-*.tgz`) or build it, then point
  Monado's CMake at it:
  `-DONNXRuntime_ROOT_DIR=/path/to/onnxruntime`.
  There is **no** Debian/Ubuntu `onnxruntime` package; this is why the .deb
  bundles `libonnxruntime.so*` into `/usr/lib/pibridge/`.
- **Basalt** (`libbasalt.so`) — *optional*, only for the open SLAM engine
  (the default inside-out path uses the Pimax engine via winbridge, not Basalt).

After building the driver, install/stage it as `steamvr-monado/` and copy the
ONNX Runtime `.so` next to `driver_monado.so` (or set an rpath to it), exactly
as `debian/rules` does for the prebuilt blob.

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
`wine winbridge.exe` inside each inside-out tracking session (START TRACKING)
and stops it with the session. See `docs/WINBRIDGE.md` for the full picture and
`docs/INSTALL.md` section 7 for the Pimax kit and the user steps.

## After install: per-user setup

`postinst` only activates the udev rule (copies it to `/lib/udev/rules.d/` and
reloads udev) and prints a note. The rest is **per-user** and is done by the
app on first run: enabling the systemd *user* services and registering the
PiBridge Monado SteamVR driver in `openvrpaths.vrpath`. No system
`daemon-reload` is needed (the units are user units). Just launch **PiBridge**.
