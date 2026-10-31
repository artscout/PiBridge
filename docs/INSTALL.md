# Installing PiBridge

PiBridge is a VR hub for Pimax headsets on Linux: one app (`pibridge`) and two small user services (`pibridge-keepalived`
keeps the headset awake while VR runs, `pibridge-eyed` runs the eye tracker). Not affiliated with Pimax.

It runs VR on one of two backends, chosen on the Dashboard:

- **SteamVR** (the default): SteamVR with CustomHeadsetOpenVR as the headset driver (section 5).
- **Monado**: PiBridge's own Monado fork as the OpenXR runtime (section 7.2), with PiBridge World as the home in the
  headset (a Godot 4 project; Godot is needed for it).

This guide installs PiBridge for your user only (into `~/.local`). Root is needed for three things only, each said where
it comes: the udev rule, the NVIDIA module (Crystal Super) and the package installs.

Today's targets: the **Pimax Crystal Super with the Lighthouse faceplate** and the **original Pimax Crystal** (5.5) under
SteamVR, tested on Ubuntu 26.04 (KDE Wayland) with an RTX 5090. The steps for Arch and Fedora use the same tools; their package names are given, but they
have not been tried on real hardware yet.

**Another Pimax headset** (Crystal Light, Dream Air SE, 8KX, 5K family, ...): PiBridge only sends a headset
the commands its official Pimax client sends, so a model is added from a recording of that client. If you have one
and a Windows PC with the Pimax client, please record it with the guided kit in [`windows/`](../windows/) (about 15
minutes, read only towards the headset) and send us the bundle privately: [HEADSET-DATA.md](HEADSET-DATA.md).

## 1. Build dependencies

You need CMake 3.22 or newer, Ninja, a C++23 compiler with `std::expected` (GCC 13 or newer, or Clang 17 or newer),
the Qt 6 Widgets development files, the OpenXR headers, and the Vulkan headers with `glslangValidator` (the quad views
layer, section 7, is built with PiBridge). zlib (for reading the headset's calibration blob) and the IBM Plex fonts
(the app's look; it falls back to Noto/DejaVu) are optional.

Ubuntu / Debian:
```
sudo apt install cmake ninja-build g++ qt6-base-dev libopenxr-dev libvulkan-dev glslang-tools zlib1g-dev libjpeg-dev fonts-ibm-plex
```

Arch:
```
sudo pacman -S --needed cmake ninja gcc qt6-base openxr vulkan-headers vulkan-icd-loader glslang zlib libjpeg-turbo ttf-ibm-plex
```

Fedora:
```
sudo dnf install cmake ninja-build gcc-c++ qt6-qtbase-devel openxr-devel vulkan-headers vulkan-loader-devel glslang zlib-devel libjpeg-turbo-devel ibm-plex-sans-fonts ibm-plex-mono-fonts
```

The Monado backend (section 7.2, optional) needs Monado's own build dependencies and Godot 4; that section lists
them.

Check the compiler: `g++ --version` must say 13 or newer (Ubuntu 22.04 ships 11: install `g++-13` and add
`-DCMAKE_CXX_COMPILER=g++-13` to the configure command below).

Inside-out tracking (section 7, optional) needs more: **Wine** and, to build the Pimax engine host once (`pibridge-build-winbridge`), **MinGW-w64**
(`sudo apt install wine mingw-w64`; Arch `wine mingw-w64-gcc`; Fedora `wine mingw64-gcc-c++`). The open **Basalt**
engine needs `libbasalt.so` built separately (section 7.1). None of this is needed for the tested Lighthouse path.

## 2. Build, test and install

From the PiBridge source folder:
```
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/.local
cmake --build build
ctest --test-dir build
cmake --install build
systemctl --user daemon-reload
# refresh the menu and icon caches so the PiBridge entry shows its icon at once
kbuildsycoca6 2>/dev/null || true                      # KDE Plasma
gtk-update-icon-cache -q -t ~/.local/share/icons/hicolor 2>/dev/null || true   # GNOME and others
```

If the menu entry still has no icon in KDE, log out and in once (Plasma keeps its icon cache in memory).

`ctest` must report `100% tests passed`. The tests never touch a headset: they use fake devices. (Some app tests
switch to a German or Russian locale, when one is installed, to check number formatting.)

What `cmake --install` puts where (prefix `~/.local`):

| What | Where |
|---|---|
| The app and the tools: `pibridge`, `pibridge-keepalived`, `pibridge-eyed`, `pibridge-eye`, and the rest of the `pibridge-*` tools (room setup, recorder, the in-headset dashboard `pibridge-home`, the recenter ring `pibridge-xroverlay`, ...) | `~/.local/bin/` |
| The quad views layer and its OpenXR manifest (section 7) | `~/.local/lib/qv-vulkan/`, `~/.local/share/openxr/1/api_layers/implicit.d/` |
| PiBridge World, the Monado backend's home in the headset (a Godot project, section 7.2) | `~/.local/share/pibridge/world/` |
| The systemd user units `pibridge-keepalived.service`, `pibridge-eyed.service` (their `ExecStart` names `~/.local/bin`) | `~/.local/share/systemd/user/` (systemd reads it: it is `$XDG_DATA_HOME/systemd/user`) |
| The device descriptions (the allowlist of HID commands) | `~/.local/share/pibridge/devices/` |
| The udev rule, to copy by hand (step 3) | `~/.local/share/pibridge/udev/70-pibridge.rules` |
| Desktop entry and icon | `~/.local/share/applications/pibridge.desktop`, `~/.local/share/icons/hicolor/scalable/apps/pibridge.svg` |
| The app's help and this guide | `~/.local/share/doc/pibridge/` |

No library headers or static libraries are installed: the libraries are internal. If `~/.local/bin` is not on your
`PATH`, the desktop entry still works (it names the full path); for the terminal add it to `PATH`.

A system-wide install works the same with `-DCMAKE_INSTALL_PREFIX=/usr/local` and `sudo cmake --install build`; the
units then go to `/usr/local/share/systemd/user/`, also on systemd's user search path.

## 3. Access to the headset (root, once)

The headset's HID interfaces (vendor `34a4`) and the eye chip (`2104:0561`; `2104:0220` on the original Crystal) must be readable and writable by you.
The rule gives the logged-in user access (`uaccess`):
```
sudo cp ~/.local/share/pibridge/udev/70-pibridge.rules /etc/udev/rules.d/
sudo udevadm control --reload
sudo udevadm trigger
```
If you already have omarchyvr's `70-pimax.rules`, it gives the same access; keeping both does no harm.

## 4. Start the services

```
systemctl --user enable --now pibridge-keepalived pibridge-eyed
```

They replace the older services of the same job, and refuse to run next to them (`Conflicts=`): `omarchyvr-keepalived`
and `omarchyvr-eyed` (omarchyvr), and `pimax-keepalived` and `pimax-eyed` (PiBridge's earlier name, `pimax-linux`). If
you had those, switch them off first so they do not come back at the next login:
```
systemctl --user disable --now omarchyvr-keepalived omarchyvr-eyed pimax-keepalived pimax-eyed 2>/dev/null
systemctl --user enable --now pibridge-keepalived pibridge-eyed
```

Settings and eye files from the earlier name are picked up on their own: while `~/.config/pibridge/pibridge.conf` does not
exist, PiBridge reads `~/.config/pimax-linux/pimax-linux.conf`, and the eye start list, eye profile, calibration and
correction files are looked for in `~/.config/pibridge/`, then `~/.config/pimax-linux/`, then `~/.config/omarchyvr/`.
The first setting you change in the app writes `~/.config/pibridge/pibridge.conf`. To move everything at once instead:
```
mkdir -p ~/.config/pibridge
cp -n ~/.config/pimax-linux/* ~/.config/pibridge/ 2>/dev/null
[ -f ~/.config/pibridge/pimax-linux.conf ] && mv -n ~/.config/pibridge/pimax-linux.conf ~/.config/pibridge/pibridge.conf
```

Then start **PiBridge** from the application menu (or `pibridge`). With the defaults the services wait for a VR runtime,
SteamVR or PiBridge's Monado (`follow=vrserver,monado-service` in `pibridge.conf`). When one starts they wake the
headset, switch its panel on at 90 Hz, make it the default sound device and start the eye tracker. When it closes they
give everything back and the headset goes to standby (green LED).

### 4.1 The SteamVR dashboard panel (installed with the app)

`pibridge-overlay` is a small tab in the SteamVR dashboard with three buttons — **ROOM SETUP**, **QUICK ROOM SETUP**
and a video-record button (**START RECORDING** / **STOP RECORDING**) — so you can start room setup or recording from
inside VR without taking the headset off. It is built and installed together with the
app by `cmake --install` (the binary goes to `~/.local/bin/pibridge-overlay`, its manifest to
`~/.local/share/pibridge/pibridge.vrmanifest`); there is nothing extra to build.

You turn it on from the app, not from a terminal: the **SteamVR dashboard** switch in the **Dashboard → Quick settings**
(right under Passthrough). On, the app runs the tab as a background helper and the PiBridge tab appears in the SteamVR
dashboard the next time you open it; off, the app stops it. The choice is saved (`steamvr_dashboard` in
`pibridge.conf`), so a restart keeps it.

**No manual registration is needed.** The running helper creates the dashboard tab itself; the auto-launch
registration that would let SteamVR start it on its own is deliberately left off (leaving the app as the single owner
of the panel). Full detail is in [OVERLAY.md](OVERLAY.md).

This panel, and the in-headset room setup it launches, are tested in a headset (see [OVERLAY.md](OVERLAY.md)). You can
also start room setup from the Dashboard or the Room page.

### 4.2 Video recording (the Vulkan capture layer)

PiBridge records what the headset shows — the Dashboard **Record video** card, or the overlay's record button — with
`pibridge-record` and a small **Vulkan implicit layer** (`tools/record-layer`: `libpibridge-record-layer.so` and its
manifest `VK_LAYER_PIBRIDGE_record.json`). The loader pulls the layer into SteamVR's `vrcompositor`, where it copies the
game's rendered eye image and hands it to `pibridge-record` for NVENC (or CPU) mp4 encoding — no GL mirror, no missed
frames. Both are built and installed with the app (`cmake --install`): the `.so` goes to `~/.local/lib/pibridge/`, the
manifest to `~/.local/share/vulkan/implicit_layer.d/`.

**One root step for games under Proton/SteamVR:** the Steam Linux Runtime container does not see
`~/.local/share/vulkan/implicit_layer.d`, so the manifest must also sit in a system path
(`/usr/share/vulkan/implicit_layer.d/VK_LAYER_PIBRIDGE_record.json`) — a root step, done through the app's pkexec
prompt. Full detail (capture sources, encoders, options) is in [record.md](record.md).

Under the Monado backend no layer is involved: PiBridge's Monado has a capture tap in its compositor, and the same
`pibridge-record` encodes from it. In the headset, recording starts and stops from PiBridge World's **Record** button
or the dashboard over a game (Headset page, **Record video**).

## 5. Pimax Crystal Super (Lighthouse faceplate) and the original Crystal

Who does what in this setup:

| Part | Who |
|---|---|
| Head and controller tracking | SteamVR's own Lighthouse driver (the faceplate) |
| The headset display in SteamVR (render size, lens distortion, IPD) | CustomHeadsetOpenVR (sboys3) with our patches (5.3) |
| Wake, keepalive, panel on, sound switch | `pibridge-keepalived` |
| Eye tracking for OpenXR apps (`XR_EXT_eye_gaze_interaction`) | `pibridge-eyed` -> `/dev/shm/pibridge-eye-gaze` -> CustomHeadsetOpenVR |
| 7104x3840 at 90 Hz over DSC on RTX 50 | driver 615.71.09+ needs no patched module; older branches (595.x) do. The EDID override (5.2) is for every driver |
| Headset speakers and microphone | USB audio, with the WirePlumber rule (5.4) |

The files are in `patches/` of the source tree; `patches/README.md` says where each comes from and its licence.

### 5.1 Faceplate
Switch the Lighthouse faceplate on (its switch). The Valve devices (`28de:2300`, and `28de:2101` dongles) appear on
their own; pair the controllers in SteamVR as usual.

### 5.2 NVIDIA driver (root)
#### The EDID override (every Pimax headset, every driver version)
NVIDIA's driver needs an EDID override for the headset's output, made from the headset's own EDID:
- the "VR headset" flag (a Microsoft VSDB): without it the desktop extends onto the headset (the original Crystal
  shows the KDE desktop) instead of leaving it to SteamVR;
- 8 bits per colour: the Crystal Super's EDID says "colour depth undefined", and DSC refuses the 6 bpc NVIDIA then
  uses, so it gets no picture at all.

One script does it for the Crystal Super, the original Crystal and the Crystal Light (the Light untested). The headset
must be on. The Crystal Super is switched on by software: PiBridge running, or `pibridge-keepalived --no-follow
--video 90` in a terminal with the service stopped. The Crystal Light is switched on by software too (Pimax Play on
Windows), which PiBridge does not do for it yet. The original Crystal is switched on with its own power button:
```
/usr/share/pibridge/edid/pibridge-headset-edid status           # card1-DP-6 DP-4 desktop: no override yet
sudo /usr/share/pibridge/edid/pibridge-headset-edid apply        # then reboot
```
(From the source tree: `tools/headset-edid/pibridge-headset-edid`.) It finds the output itself and writes
`/etc/pibridge/pimax-edid.bin`, `/etc/pibridge/nvidia-modeset.conf` (`override.[<GPU>].<output>=...`) and
`/etc/modprobe.d/pibridge-nvidia.conf` (`options nvidia-modeset config_file=...`). The driver reads them once, when it
loads, so a reboot is needed; `status` then says `vr`. When nvidia-modeset loads from the initramfs, the script also
installs an initramfs hook and rebuilds it (Ubuntu/Debian initramfs-tools; on Arch / Fedora add the two files by hand:
`FILES=` in `/etc/mkinitcpio.conf`, `install_items+=` in a `/etc/dracut.conf.d/` file).

- The override belongs to the GPU output, not to the headset. Plugged into another DisplayPort, the headset needs
  `apply` and a reboot again. An overridden output counts as connected even when empty, and a monitor plugged into it
  would get the headset's EDID: `sudo pibridge-headset-edid forget DP-4` (or its DRM name, `card1-DP-6`) takes one
  output back, `remove` all of them.
- One override serves every Pimax: it carries the timings of all headsets the script knows
  (`/usr/share/pibridge/edid/known/`) and of each one it has seen here (`/var/lib/pibridge/edid/`), so swapping the
  Crystal Super and the original Crystal on the same output needs nothing more.
- Older installs under the omarchyvr names (`/etc/omarchyvr/`, `omarchyvr-nvidia.conf`, the `omarchyvr-edid` hook)
  are taken over by `apply` and removed.

#### The patched modules (driver branches older than 615.71.09, Crystal Super on RTX 50)
> **Driver 615.71.09 and newer (RTX 5090): no patch needed.** The Blackwell DSC/display fixes are upstream, so the
> stock open module drives the Crystal Super on its own (with the EDID override above). On Ubuntu just install the
> packaged open module (`sudo apt install linux-modules-nvidia-615-open-generic nvidia-dkms-615-open` or the `-open`
> metapackage): it is Canonical-signed, so it loads under Secure Boot with no MOK step. If you had the hand-built
> patched module from an older branch, `sudo /path/to/pibridge/patches/nvidia/blackwell/uninstall-nvidia.sh` removes
> it (and the override: run `apply` again afterwards), then reboot. **Never install patched modules of another
> version than the NVIDIA libraries:** the desktop loses its GPU; `install-nvidia.sh` refuses that.

Without the patch on an older branch the driver lists no mode for the headset ("DP Bandwidth check failed"). Why, and
what the patch changes: `patches/nvidia/README.md`. The install script is written for Ubuntu/Debian (initramfs-tools).
On an RTX 20/30/40 the Crystal Super would need the 2Heads1OR patches in `patches/nvidia/pre-blackwell/` instead
(untested; the table in `patches/nvidia/README.md` says which version has which patch).

1. Find your driver version and get the matching open module source:
   ```
   modinfo -F version nvidia          # e.g. 595.99.02
   git clone --depth 1 --branch 595.99.02 https://github.com/NVIDIA/open-gpu-kernel-modules
   cd open-gpu-kernel-modules
   git apply /path/to/pibridge/patches/nvidia/blackwell/595.99.02/0001-nvidia-dp-fec-reread.patch
   make modules -j8 SYSSRC=/lib/modules/$(uname -r)/build
   ```
   The five modules are then in `kernel-open/`: `nvidia.ko`, `nvidia-modeset.ko`, `nvidia-drm.ko`, `nvidia-uvm.ko`,
   `nvidia-peermem.ko`.
2. A signing key (Secure Boot). The install script checks the key's name, so keep it exactly (the name is from
   omarchyvr, so keys made for it work too):
   ```
   mkdir -p ~/crystal-nvidia/{modules,mok}
   cd ~/crystal-nvidia/mok
   openssl req -new -x509 -newkey rsa:2048 -nodes -days 36500 -outform DER \
       -keyout MOK.priv -out MOK.der -subj "/CN=omarchyvr local NVIDIA module signing/"
   sudo mokutil --import MOK.der
   ```
   Choose a password of digits only (MokManager uses a US keyboard layout), reboot and enroll the key in the blue
   MokManager screen.
3. Sign the modules and put them into the work folder:
   ```
   cp /path/to/open-gpu-kernel-modules/kernel-open/nvidia*.ko ~/crystal-nvidia/modules/
   for m in ~/crystal-nvidia/modules/*.ko; do
       /lib/modules/$(uname -r)/build/scripts/sign-file sha256 ~/crystal-nvidia/mok/MOK.priv ~/crystal-nvidia/mok/MOK.der "$m"
   done
   ```
4. Install, with the headset on (the script runs `pibridge-headset-edid apply` for the override), and reboot:
   ```
   sudo /path/to/pibridge/patches/nvidia/blackwell/install-nvidia.sh ~/crystal-nvidia
   ```
   It puts the modules into `/lib/modules/<kernel>/updates/pibridge-nvidia` and rebuilds the initramfs. `--no-edid`:
   modules only. `--debug` (nvidia-modeset `debug=1` at every boot) is for diagnosing a rejected mode only; running
   the script again without it removes `debug=1`.
   Undo: `sudo /path/to/pibridge/patches/nvidia/blackwell/uninstall-nvidia.sh`, then reboot. If a boot hangs, pick
   another kernel in GRUB's "Advanced options" and run `uninstall-nvidia.sh <the hung kernel's version>` there.

Arch and Fedora: steps 1 to 3 are the same; the script's initramfs step uses Ubuntu's `update-initramfs`. Install by
hand instead: the signed modules into `/lib/modules/$(uname -r)/updates/`, `depmod -a`, and the EDID override as
above. A driver update needs the module build and signing again.

### 5.3 CustomHeadsetOpenVR (the SteamVR display driver)
[CustomHeadsetOpenVR](https://github.com/sboys3/CustomHeadsetOpenVR) by sboys3, with PiBridge's Linux patches
(`patches/chovr/`): the direct USB connection (no Pimax runtime on Linux), the gaze from shared memory, the
"Pimax Builtin (from dump)" lens profile, and the original Crystal (its Lighthouse and its own inside-out, the
refresh rate). For the Crystal Super the branch with the first two is enough:
```
git clone --recurse-submodules --branch linux-crystal-super https://github.com/artscout/CustomHeadsetOpenVR.git
```
or apply them to upstream commit `4f25826` (in this order; inside-out of the Crystal Super / Dream Air needs all five):
```
git clone --recurse-submodules https://github.com/sboys3/CustomHeadsetOpenVR.git
cd CustomHeadsetOpenVR && git checkout 4f25826 && git submodule update --init --recursive
git apply /path/to/pibridge/patches/chovr/chovr-linux-direct-usb.patch
git apply /path/to/pibridge/patches/chovr/chovr-linux-eye-gaze.patch
git apply /path/to/pibridge/patches/chovr/chovr-linux-pimax-dump-profile.patch
git apply /path/to/pibridge/patches/chovr/chovr-linux-crystal.patch
git apply /path/to/pibridge/patches/chovr/chovr-linux-headtrack.patch
cd ..
```
(The changes are offered upstream.) Build it and register it with SteamVR:
```
cd CustomHeadsetOpenVR/CustomHeadsetOpenVR
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j8
~/.steam/steam/steamapps/common/SteamVR/bin/vrpathreg.sh adddriver "$PWD/CustomHeadsetOpenVR"
```
Optional — Pimax's own controller models (otherwise SteamVR draws them as Oculus Touch). SteamVR loads render models
from its OWN resources (it does not search an external driver's), so copy them there once from a Windows Pimax Play
install (personal use, not in the repo; repeat after a Pimax update, and after a SteamVR update that clears them):
```
SV=~/.local/share/Steam/steamapps/common/SteamVR/resources/rendermodels
cp -r "/mnt/c/Program Files/Pimax/SteamVRSupport/drivers/aapvr/resources/rendermodels/crystal_controller_left" \
      "/mnt/c/Program Files/Pimax/SteamVRSupport/drivers/aapvr/resources/rendermodels/crystal_controller_right" "$SV/"
```
The controller driver uses the Pimax model (`crystal_controller_left`/`right`) only when it finds them there (it logs
the chosen model and path); without them it keeps the Oculus Touch model, so a controller still shows.

Enable the headset in `~/.config/CustomHeadset/settings.json` (or with its GUI, `CustomHeadsetGUI`; PiBridge starts that
configurator from the Dashboard's **Headset driver settings** button once the driver is registered):
```
{"crystalSuperMicroOLED": {"enable": true, "distortionProfile": "Crystal Super micro-OLED like Pimax"}}
```
The "like Pimax" profile matched the lens best in our test; the driver's default is "Crystal Super Micro-OLED
Default".

### 5.4 Sound
The speakers are ALSA device 1 of the headset's card, and the stock profiles only look at device 0. The rule picks the
pro-audio profile and gives the devices plain names:
```
mkdir -p ~/.config/wireplumber/wireplumber.conf.d
cp /path/to/pibridge/patches/audio/51-pimax-crystal-super.conf ~/.config/wireplumber/wireplumber.conf.d/
systemctl --user restart wireplumber
```
The **Default audio output** switch in the app (on by default) makes the headset the default output while SteamVR
runs.

### 5.5 The original Pimax Crystal
The original Crystal (USB `34a4:0012`, "P3B") is a standalone headset with its own Snapdragon XR2: it is switched on
with its power button, runs on its battery, tracks itself (inside-out) and switches to Lighthouse tracking by a
command, with no faceplate swap. It needs the same parts as the Crystal Super (5.2 EDID override, 5.3
CustomHeadsetOpenVR with **all four** patches, the udev rule of section 3) and no NVIDIA module patch.

Who does what:

| Part | Who |
|---|---|
| Connecting when it is switched on, the tracking mode, switching it off | `pibridge-keepalived`: the frames Pimax Play sends (`screen_state`, `video 120`, `EnableLighthouse` / `LighthouseTakeEffect`, `shutDown`, `setTobiiEyeDeviceEnable`), byte for byte from a Windows recording |
| Head and controller poses in its own inside-out | the headset; `pibridge-keepalived` writes them to `/dev/shm/pibridge-crystal-pose`, CustomHeadsetOpenVR's SLAM headset hands them to SteamVR |
| Head and controllers in Lighthouse mode | SteamVR's Lighthouse driver (its built-in Lighthouse module, Index controllers) |
| Display | CustomHeadsetOpenVR (`crystalOG`, enabled by the app); 72, 90 or 120 Hz from the Dashboard |
| Eye tracking | its Tobii chip `2104:0220`, switched on by `pibridge-keepalived`, read by `pibridge-eyed` |
| Sound | over DisplayPort (the GPU's audio, the port named "Pimax"); its USB audio is the microphone |

Use:
1. Switch it on with its power button. `pibridge-keepalived` connects within a few seconds, switches it to the
   tracking mode chosen on the Dashboard, waits ~10 s for its DisplayPort to settle, then starts SteamVR and PiBridge.
2. The Dashboard shows its battery, **SWITCH OFF**, and the tracking it really runs (from its reports; "tracking lost"
   when its inside-out has no position). The IPD is set with the buttons on the headset; the slider follows them.
3. **Lighthouse / Inside-out** on the Dashboard switches the headset; a running SteamVR is closed and started again,
   since SteamVR picks its headset device at its start. The refresh rate (72 / 90 / 120 Hz) restarts it the same way.
4. **STOP VR** closes SteamVR and switches the headset off. **Reset SteamVR** restarts SteamVR.

Notes:
- The EDID override (5.2) is what keeps the desktop off the headset: without it KDE uses it as a monitor.
- Started before its DisplayPort settles, SteamVR gets only the 72 Hz mode and the headset never shows the picture
  (`pibridge-keepalived` logs `video link 1`); the wait above avoids that. `video link 2` means the panel shows the
  PC's picture.
- Its sound output exists only while SteamVR drives its display; `pibridge-keepalived` makes it the default output as
  soon as it appears.
- Controller models: Pimax's own (`crystal_controller_*`) when copied into the CHOVR driver's resources (step 5.3),
  else Oculus Touch.

On the **Monado backend** (section 7.2; NOT tried on the headset yet) the same daemon connects it, switches its mode
and switches it off, but starts only the PiBridge app (not SteamVR); **START VR** there runs Monado for it:
- Its own inside-out: the `pibridge_og` builder: its head and controllers from `/dev/shm/pibridge-crystal-pose` (the
  Pimax button held recenters, the menu button is the menu).
- Lighthouse: the `pibridge_lh` builder with its built-in Lighthouse module; Index controllers. VR starts once the
  module is on USB.
- A tracking switch (the Dashboard, or the headset's own) restarts the Monado session by itself: Monado's devices are
  fixed at its start, as SteamVR's headset is.
- Its lens profile: `crystal-og.conf` + `crystal-og-lut.bin` in `~/.local/share/pibridge/lens/` or
  `~/gamedev/ext/lens/` (`rec_lens_conf_og=` names another; the Crystal Super's `lens.conf` / `rec_lens_conf` are never
  used for it). Made from CustomHeadsetOpenVR's "Crystal OG Default" with its own code, offline
  (`~/gamedev/ext/lens/og/`: `chovr_profile.py`, `chovr_sample.cpp`, `og2lut.py`), each eye's axis turned outward by
  CHOVR's 5 deg; if the world looks too small or too large, turn the axis with
  `pibridge-lens turn og/crystal-og-lut-unturned.bin crystal-og-lut.bin -LEFT RIGHT` (the Crystal Super needed its own,
  4.4 to 4.7 deg).
- Its volume buttons change its own volume; the indicator shows it in the headset.
- Not yet: controller vibration, the floor height in
  inside-out (its origin is where its tracking started).

## 6. Eye tracking

The Tobii eye chip in the headset is started with the bytes of a start that Pimax Play made under Windows, with your
own Tobii calibration profile uploaded in place of the one in that capture. The chip then applies your calibration
itself, as under Windows. Today this needs **one Windows session** with Pimax Play:

1. Under Windows: calibrate the eye tracking in Pimax Play. Capture one cold start of Pimax Play with USBPcap (close
   Pimax Play, start the capture, start Pimax Play, wait until the eye tracking runs). omarchyvr's
   `windows/usbpcap-capture.cmd` and `windows/crystal-eye-files.ps1` do this and copy the files. Your profile is
   `C:\ProgramData\Tobii\Tobii Platform Runtime\XR5EYECHIP\<headset>\calibration.setpm`.
2. Under Linux, make the start list from the capture and take the profile from the calibration file:
   ```
   mkdir -p ~/.config/pibridge
   pibridge-eye frames capture.pcap -o ~/.config/pibridge/eye-start-crystal-super.txt
   pibridge-eye profile calibration.setpm -o ~/.config/pibridge/eye-profile-crystal-super.bin
   ```
   Or, for the profile, use **Import calibration from Windows...** in the app (Dashboard, Quick settings). Both files
   are private (they hold your headset's eye data): do not share them. After a new calibration under Windows only the
   profile has to be taken again.
3. Turn **Eye tracking** on in the app (or `systemctl --user restart pibridge-eyed`). With SteamVR running the app shows
   "Streaming" and a live gaze dot; OpenXR apps get `XR_EXT_eye_gaze_interaction` through CustomHeadsetOpenVR.
4. **Calibrate** in the app runs `pibridge-eyecal`: the eye tracker calibrates itself, as Pimax Play does it (a green dot
   travels to 9 targets, on black and then on white; A starts, at the end A again or B save and exit). The chip's new
   profile is saved (the previous one is kept as a backup) and uploaded at every start. **Recenter** (5 targets) removes
   a constant offset between the chip and SteamVR's view. The app stops `pibridge-eyed` while either runs, since the chip
   has one owner. Details: docs/eyecal.md.

With the calibration on the chip itself, the Windows step is needed only for the start list (step 1); a public start
list for the Crystal Super, so that a new headset needs no Windows session at all, is the next step.

## 7. Quad views (foveated rendering for Vulkan games)

Foveated rendering ("quad views") draws the middle of your view at full detail and the edges at lower detail, which
saves a lot of GPU work; with eye tracking running the sharp area follows your eyes. PiBridge's **Performance** page is
the control panel for it: the on/off switch, the presets, the resolutions, the gaze area and the per-game profiles
(written to `~/.config/qv-vulkan/settings.conf`).

The rendering is done by PiBridge's OpenXR API layer in `quadviews/` (formerly the separate
`linux-vulkan-quadviews-openxr-layer` project, now part of PiBridge). It is built, tested and installed with the rest
(section 2): the layer goes to `~/.local/lib/qv-vulkan/`, its manifest to
`~/.local/share/openxr/1/api_layers/implicit.d/`, the same manifest the Performance page renames to switch the layer on
and off. There is nothing else to install.

- **Under SteamVR** the layer renders the four views and composes them into stereo itself.
- **Under Monado**, which has quad views (`XR_VARJO_quad_views`), the layer moves the focus views with your gaze and
  Monado's compositor puts them in. PiBridge World has a **Quad views** page with the same settings.

If you had the old stand-alone layer installed into the same prefix, PiBridge's install replaces it (same paths).

### 7.1 Inside-out tracking (Crystal Super / Dream Air, optional)

Inside-out tracks the head in 6DoF from the headset's own cameras — no base stations, no faceplate. `pibridge-headtrack`
tracks the head and the controllers and CustomHeadsetOpenVR shows the poses in SteamVR, so games still start through
SteamVR as usual ([pibridge-headtrack.md](pibridge-headtrack.md)). It is **experimental**; Lighthouse (section 5) is
the tested path. Choose **Inside-out** on the Dashboard (or the Tracking page) and press **START VR**.

Common to both engines: CustomHeadsetOpenVR with all five patches, including `chovr-linux-headtrack.patch` (section
5.3); `pibridge-cam`, `pibridge-ctrltrack` and `pibridge-headtrack` (installed with the app, section 2); and the
headset's factory **camera calibration**
(`device_calibration_<serial>.xml`, copied below, or set as `rec_cam_calib`). Then pick one head-tracking engine on the
Tracking page (**Head tracking engine**):

**A) Pimax — the default, matches Pimax Play.** Pimax Play's own head-SLAM and controller-LED engine, run under Wine
(`winbridge`; the full design and rationale are in [WINBRIDGE.md](WINBRIDGE.md)). One-time setup:

1. Install Wine and MinGW-w64 (section 1).
2. Copy Pimax Play's files from a Windows PC into `~/.local/share/pibridge/windata/`: the `Program Files/Pimax/Runtime`
   DLLs, the SLAM kit (`voc_GEN_8X4.dbow`, `pimax_database.bin`), this headset's `device_calibration_<serial>.xml`, and
   the `H1/` controller models. The exact table and copy commands:
   [WINBRIDGE.md, "Copying the Pimax files"](WINBRIDGE.md#copying-the-pimax-files-once-and-again-after-a-pimax-play-update).
   These files are Pimax's — personal use, they stay on this computer and are never committed.
3. Build the engine host (`winbridge.exe`) and the VIT bridge (`libpibridge_vit_winbridge.so`) once. They are our
   code, shipped as source, because the `.exe` is built with MinGW on your machine.
   - **From the package (.deb):** run
     ```
     pibridge-build-winbridge
     ```
     It builds from `/usr/share/pibridge/winbridge-src/` and installs both files to
     `~/.local/share/pibridge/winbridge/`. Run it as your user, not with sudo: the result belongs to your home.
   - **From the source tree:**
     ```
     sh winbridge/build-winbridge.sh
     ```
     It does the same, from `winbridge/`.

   The app finds the two files in `~/.local/share/pibridge/winbridge/`, in `<prefix>/lib/pibridge/winbridge/`, or in
   the source tree's `winbridge/`; `rec_winbridge=` and `rec_vit_winbridge=` in `pibridge.conf` name other paths.
4. On the Tracking page choose **Head tracking engine: Pimax**; it tells you in plain words what is still missing
   (Wine, the kit, or the two files from step 3).
5. **There is nothing to start by hand, and no service for it.** winbridge is not a daemon and pibridge-keepalived
   does not start it. The app runs it inside the tracking session: at every **START VR** it starts
   `wine winbridge.exe` once the camera broker is up, before `pibridge-headtrack` and SteamVR, and stops it with the
   session. If it is still running from an earlier session it holds
   its port and the next start is refused; end it with `pkill -f winbridge.exe`.
   To check it ran, see `~/.local/state/pibridge/inside-out/<session>/logs/winbridge.log`. If the engine stops by
   itself the session goes on and the head falls back to 3DoF until the next start.
6. After a **Pimax Play update** on Windows, copy the kit again (step 2). winbridge itself needs rebuilding only after
   a PiBridge update that changes it (run step 3 again).

**B) Basalt — open source, the fallback.** The Basalt visual-inertial tracker on the two lower cameras plus the IMU,
loaded by `pibridge-headtrack` directly as a VIT library; the controllers come from `pibridge-ctrltrack`. Build
`libbasalt.so` separately (the Basalt fork with the VIT interface, from the Monado project) and put it where the app
finds it — `~/.local/lib/pibridge/`, `/usr/local/lib/`,
`/usr/lib/`, or set `rec_basalt_lib`. PiBridge generates the base Basalt config from the headset calibration at start;
the tunables (detection grid, camera count, accel/gyro bias, drift hold) are on the Tracking page, documented in
[TRACKING.md](TRACKING.md). Choose **Head tracking engine: Basalt**.

## 7.2 The Monado backend (optional)

PiBridge has two VR backends (the **VR backend** choice on the Dashboard; `vr_backend=steamvr|monado` in
`pibridge.conf`). SteamVR is the default. **Monado** is PiBridge's own Monado fork with the Pimax driver as the OpenXR
runtime ("Monado PiBridge fork from OmarchyVR", in `third_party/monado`; what it changes: [monado.md](monado.md)).

Under Monado:

- The headset renders through PiBridge's lens profile (`pibridge-lens crystal-profile`).
- It serves eye tracking (`XR_EXT_eye_gaze_interaction`, the gaze of pibridge-eyed) and quad views
  (`XR_VARJO_quad_views`).
- The head is tracked inside-out (Tracking mode Inside-out) or by the Lighthouse faceplate (Tracking mode Lighthouse:
  Valve's `driver_lighthouse` from your SteamVR install tracks the faceplate and the Lighthouse controllers; SteamVR
  itself does not run).
- Each backend keeps its own rooms (`~/.local/share/pibridge/rooms/<backend>`; rooms saved before the split are copied
  to the SteamVR ones once) and runtime files (`$XDG_RUNTIME_DIR/pibridge/<backend>/`).
- In the headset, **PiBridge World** is the home: a world around you, the dashboard on a panel, Apps, Passthrough, the
  settings ([world.md](world.md)). Over a game, the menu button opens PiBridge's dashboard as an overlay
  ([home.md](home.md)).

### 7.2.1 Build the Monado fork

The fork is in this repository (`third_party/monado`) and is built on its own, out of tree. Its Pimax driver links
PiBridge's `libpibridge_pimax.a` (monado-abi, [monado-abi.md](monado-abi.md)), so build PiBridge first (section 2).

Monado's own build dependencies (Ubuntu / Debian; the same list as Monado's README):
```
sudo apt install build-essential pkg-config python3 libeigen3-dev libusb-1.0-0-dev libudev-dev libhidapi-dev \
    libx11-dev libx11-xcb-dev libxxf86vm-dev libxrandr-dev libxcb-randr0-dev libvulkan-dev glslang-tools \
    libglvnd-dev libgl1-mesa-dev libegl1-mesa-dev libwayland-dev wayland-protocols libuvc-dev libv4l-dev \
    libcjson-dev libsdl2-dev libbsd-dev
```

Configure and build, from the PiBridge source folder. The app looks for the build in `~/gamedev/pibridge-monado-build`;
another folder works when you name its `monado-service` as `rec_monado_service=` in `pibridge.conf` (or choose it on
the Development page):
```
cmake --build build --target pibridge_pimax_archive
cmake -S third_party/monado -B ~/gamedev/pibridge-monado-build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DXRT_BUILD_DRIVER_PIMAX=ON -DXRT_BUILD_DRIVER_STEAMVR_LIGHTHOUSE=ON \
    -DPIMAX_USE_PIBRIDGE=ON -DPIMAX_PIBRIDGE_LIB=$PWD/build/monado-abi/libpibridge_pimax.a \
    -DPIMAX_PIBRIDGE_INCLUDE=$PWD/monado-abi/include
nice -n 19 ninja -C ~/gamedev/pibridge-monado-build
ctest --test-dir ~/gamedev/pibridge-monado-build
```

**Realtime priority (once after every build, root).** Monado's compositor wants `cap_sys_nice`, and every relink of
`monado-service` drops it. The Dashboard shows **Fix permissions** while the Monado backend is chosen and the
capability is missing: it sets it through a pkexec prompt.

### 7.2.2 Godot, for PiBridge World

PiBridge World (`world/` in the source tree, installed to `~/.local/share/pibridge/world/`) is a Godot 4 project, run
by Godot. PiBridge does not ship Godot. Download the official Linux build of **Godot 4.7** (standard, not .NET) from
<https://godotengine.org/download/linux/> and verify it against the published checksums, then put it where the app
finds it:

- `~/gamedev/godot/Godot_v4.7.2-stable_linux.x86_64` (any `Godot_v4*_linux.x86_64` in that folder; the newest wins), or
- `godot4` or `godot` on your `PATH`, or
- any path, named as `rec_godot=` in `pibridge.conf`.

No import step is needed: the project has no imported assets. Without Godot the app falls back to the classic home
(`pibridge-home`), which you can also choose on purpose with `vr_home=classic`.

The app runs the installed copy (`~/.local/share/pibridge/world/`) and uses the source tree's `world/` only when there
is none, so after changing anything in `world/` run `cmake --install build` again.

Never start the world by hand while SteamVR is the system's OpenXR runtime: Godot would start SteamVR. The app starts
it inside the Monado session with the right runtime. For a check without a headset:
```
~/gamedev/godot/Godot_v4.7.2-stable_linux.x86_64 --headless --xr-mode off --path ~/.local/share/pibridge/world -- --selftest
```

Optional, for the controllers (Crystal or Valve Index) and the base stations to look as in SteamVR: copy their render
models as described in [world.md](world.md#controller-and-base-station-models) (step 6 below).

### 7.2.3 Use

1. Choose **Monado** under **VR backend** on the Dashboard and press **START VR**. Monado starts, then PiBridge World
   in the headset.
2. Games started from the app's Apps page or from the world's Apps page run on Monado (`XR_RUNTIME_JSON`). Games
   started elsewhere (Steam) use the system's active OpenXR runtime: the Dashboard's **VR backend** choice also points
   `~/.config/openxr/1/active_runtime.json` at SteamVR or at the Monado build (a symlink; a foreign regular file is
   kept once as `active_runtime.json.pibridge-backup`). OpenVR-only games always need SteamVR. Windows games that
   Steam runs through Proton do not find Monado yet, even with OpenXR: Proton sets up its OpenXR bridge only through
   an OpenVR runtime. Run them on the SteamVR backend.
3. While a game runs, PiBridge World closes itself to leave the game its memory and comes back when the game ends.
   The menu button opens the dashboard over the game: settings, and in its bottom bar **Record video** / **Stop
   recording**, **Quick room setup**, **Recenter** and **Close app**.
4. **Recenter** (the dashboard, or holding the Pimax button; on Valve Index controllers the right system button, the
   left one is the menu) turns forward to where you look and puts the centre where you stand, for every app, also
   seated ones.
5. **Room setup** for Monado runs from the world's or the overlay's dashboard and saves into the Monado rooms only. In
   the Lighthouse mode run it once per room before anything else: until then Monado uses the raw Lighthouse space,
   whose floor is somewhere else (often above your head). The setup shows the base stations where they stand.
6. **Models** (optional): the controllers and the base stations are drawn with SteamVR's render models. Copy them into
   `~/.local/share/pibridge/windata/steamvr/rendermodels/`: `crystal_controller_left` / `_right` from the Pimax client
   on Windows, `valve_controller_knu_1_0_left` / `_right` from `<SteamVR>/drivers/indexcontroller/resources/rendermodels/`,
   `lh_basestation_valve_gen2` (and `lh_basestation_vive`) from `<SteamVR>/resources/rendermodels/`; the commands are in
   [world.md](world.md#controller-and-base-station-models). **Show base stations in VR** on the Dashboard's Tracking
   card turns the base stations on.
7. **The headset goes dark but the tracking goes on** (the app's view still follows your head): a game hung the GPU,
   and Monado gave the display back so that the hang cannot lock it for good. `journalctl -k` shows an NVIDIA `Xid`
   naming the game, and the session's `monado.log` says `Released the NVIDIA modeset permission`. The headset stays
   dark until the session restarts: press **STOP VR**, then **START VR**.

## 8. Check that it works

- [ ] `ctest --test-dir build` passed, and `ls ~/.local/share/systemd/user/` lists `pibridge-keepalived.service` and
      `pibridge-eyed.service`.
- [ ] `systemctl --user status pibridge-keepalived pibridge-eyed` shows both `active (running)`; no `omarchyvr-*` or
      `pimax-*` keepalive/eye service is running.
- [ ] PiBridge starts from the application menu, with the gem icon. Without SteamVR the Dashboard says the headset
      waits for SteamVR.
- [ ] Start SteamVR: the headset's LED changes, the Dashboard shows **SCREEN ON** and 90 Hz, the headset is the
      default sound output, the controllers and base stations are listed.
- [ ] The picture in the headset is sharp and the lens distortion right (CustomHeadsetOpenVR's profile).
- [ ] Eye tracking: the Dashboard shows **Streaming** with 2 eyes and the gaze dot follows your eyes.
- [ ] Close SteamVR: about 5 s later the sound goes back to your speakers and the headset goes to standby (green LED).
- [ ] Logs page: the service lines show no errors.
- [ ] Monado backend (if you use it): with **Monado** chosen, START VR brings up PiBridge World in the headset; a game
      started from Apps shows the controllers, and the menu button over it opens the dashboard.

If the Dashboard says "Service not running": `journalctl --user -u pibridge-keepalived -u pibridge-eyed -n 50` (the Logs
page shows the same). "Permission denied" on a hidraw or USB device means the udev rule (step 3) is missing.

## 9. Uninstall or go back

Stop the services and remove what `cmake --install` installed (it keeps a list):
```
systemctl --user disable --now pibridge-keepalived pibridge-eyed
xargs rm -f < build/install_manifest.txt
systemctl --user daemon-reload
```
Your settings stay in `~/.config/pibridge/` (remove the folder to forget them). The udev rule: `sudo rm
/etc/udev/rules.d/70-pibridge.rules`. The NVIDIA module: `sudo patches/nvidia/blackwell/uninstall-nvidia.sh`, then reboot. The
CustomHeadsetOpenVR driver: `vrpathreg.sh removedriver <its folder>`.

To go back to the earlier services, enable them again (they conflict with PiBridge's, so stop PiBridge's first):
```
systemctl --user disable --now pibridge-keepalived pibridge-eyed
systemctl --user enable --now pimax-keepalived pimax-eyed      # or omarchyvr-keepalived omarchyvr-eyed
```
They read their own files (`~/.config/pimax-linux/`, `~/.config/omarchyvr/`), which PiBridge never writes.

## Dependencies and where they come from

| What | Kind | Where it comes from |
|---|---|---|
| Qt 6 (Core, Widgets) | build and run time, system package | your distribution (`qt6-base-dev` / `qt6-base` / `qt6-qtbase-devel`) |
| OpenXR headers | build time, system package | your distribution (`libopenxr-dev` / `openxr` / `openxr-devel`); headers only, nothing is linked |
| zlib | optional, system package | your distribution; without it the headset calibration blob is not inflated |
| IBM Plex fonts | optional, system package | your distribution; fallbacks: Noto Sans, DejaVu |
| `openvr.h` | vendored header | `app/third_party/openvr/` (OpenVR SDK 2.15.6, Valve, BSD-3-Clause) |
| Basalt (`libbasalt.so`) | optional, built separately | the open inside-out engine (VIT interface); not shipped |
| `libopenvr_api.so` | loaded at run time with `dlopen` | your SteamVR install (the app finds it through `~/.config/openvr/openvrpaths.vrpath`); not linked, not shipped |
| Headset and eye chip protocols, gaze math, the two daemons, device descriptions | our code, ported | from omarchyvr (laughtt and contributors, MIT OR Apache-2.0, used as MIT; see NOTICE), Rust to C++ |
| Quad views settings code | our code, copied | from artscout-qv-vulkan-linux (MIT) |
| Quad views layer | our code, built with PiBridge | `quadviews/` (formerly linux-vulkan-quadviews-openxr-layer, MIT) |
| Vulkan headers, `glslangValidator` | build time, system package | your distribution; for the quad views layer |
| Monado | optional, in this tree, built separately | `third_party/monado`: PiBridge's fork of Monado (Collabora and contributors, BSL-1.0) with the Pimax driver from omarchyvr's fork; [monado.md](monado.md) |
| Godot 4.7 | optional, run time, downloaded by you | godotengine.org (MIT); runs PiBridge World (`world/`, our code) |
| `pimax-eyecal` | optional fallback, run time | omarchyvr; PiBridge ships its own `pibridge-eyecal` |
| CustomHeadsetOpenVR | separate SteamVR driver | sboys3 (GPL-2.0), with our patches from `patches/chovr/` |
| NVIDIA open kernel modules | separate, root | NVIDIA's open-gpu-kernel-modules (MIT/GPL-2.0), with our patch from `patches/nvidia/blackwell/` (RTX 50) or `patches/nvidia/pre-blackwell/` (RTX 20/30/40) |

PiBridge builds without network access: nothing is downloaded at build time.
