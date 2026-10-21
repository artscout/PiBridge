# Installing PiBridge

PiBridge is a VR hub for Pimax headsets on Linux: one app (`pibridge`) and two small user services (`pibridge-keepalived`
keeps the headset awake while SteamVR runs, `pibridge-eyed` runs the eye tracker). Not affiliated with Pimax.

This guide installs PiBridge for your user only (into `~/.local`). Root is needed for three things only, each said where
it comes: the udev rule, the NVIDIA module (Crystal Super) and the package installs.

Today's target: the **Pimax Crystal Super with the Lighthouse faceplate under SteamVR**, tested on Ubuntu 26.04 (KDE
Wayland) with an RTX 5090. The steps for Arch and Fedora use the same tools; their package names are given, but they
have not been tried on real hardware yet.

## 1. Build dependencies

You need CMake 3.22 or newer, Ninja, a C++23 compiler with `std::expected` (GCC 13 or newer, or Clang 17 or newer),
the Qt 6 Widgets development files and the OpenXR headers. zlib (for reading the headset's calibration blob) and the
IBM Plex fonts (the app's look; it falls back to Noto/DejaVu) are optional.

Ubuntu / Debian:
```
sudo apt install cmake ninja-build g++ qt6-base-dev libopenxr-dev zlib1g-dev fonts-ibm-plex
```

Arch:
```
sudo pacman -S --needed cmake ninja gcc qt6-base openxr zlib ttf-ibm-plex
```

Fedora:
```
sudo dnf install cmake ninja-build gcc-c++ qt6-qtbase-devel openxr-devel zlib-devel ibm-plex-sans-fonts ibm-plex-mono-fonts
```

Check the compiler: `g++ --version` must say 13 or newer (Ubuntu 22.04 ships 11: install `g++-13` and add
`-DCMAKE_CXX_COMPILER=g++-13` to the configure command below).

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
| The app and the tools: `pibridge`, `pibridge-keepalived`, `pibridge-eyed`, `pibridge-eye` | `~/.local/bin/` |
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

The headset's HID interfaces (vendor `34a4`) and the eye chip (`2104:0561`) must be readable and writable by you.
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

Then start **PiBridge** from the application menu (or `pibridge`). With the defaults the services wait for SteamVR: when
SteamVR starts they wake the headset, switch its panel on at 90 Hz, make it the default sound device and start the
eye tracker; when SteamVR closes they give everything back and the headset goes to standby (green LED).

### 4.1 The SteamVR dashboard panel (installed with the app)

`pibridge-overlay` is a small tab in the SteamVR dashboard with two buttons — **ROOM SETUP** and **QUICK ROOM SETUP**
— so you can start room setup from inside VR without taking the headset off. It is built and installed together with the
app by `cmake --install` (the binary goes to `~/.local/bin/pibridge-overlay`, its manifest to
`~/.local/share/pibridge/pibridge.vrmanifest`); there is nothing extra to build.

You turn it on from the app, not from a terminal: the **SteamVR dashboard** switch in the **Dashboard → Quick settings**
(right under Passthrough). On, the app runs the tab as a background helper and the PiBridge tab appears in the SteamVR
dashboard the next time you open it; off, the app stops it. The choice is saved (`steamvr_dashboard` in
`pibridge.conf`), so a restart keeps it.

**No manual registration is needed.** The running helper creates the dashboard tab itself; the auto-launch
registration that would let SteamVR start it on its own is deliberately left off (leaving the app as the single owner
of the panel). Full detail is in [OVERLAY.md](OVERLAY.md).

> This panel is new and has not been verified in a headset yet (see [OVERLAY.md](OVERLAY.md)). Until then, you can also
> start room setup from the Dashboard or the Room page.

## 5. Pimax Crystal Super (Lighthouse faceplate)

Who does what in this setup:

| Part | Who |
|---|---|
| Head and controller tracking | SteamVR's own Lighthouse driver (the faceplate) |
| The headset display in SteamVR (render size, lens distortion, IPD) | CustomHeadsetOpenVR (sboys3) with our patches (5.3) |
| Wake, keepalive, panel on, sound switch | `pibridge-keepalived` |
| Eye tracking for OpenXR apps (`XR_EXT_eye_gaze_interaction`) | `pibridge-eyed` -> `/dev/shm/omarchyvr-eye-gaze` -> CustomHeadsetOpenVR |
| 7104x3840 at 90 Hz over DSC on RTX 50 | driver 615.71.09+ works as-is; only older branches (595.x) need the NVIDIA patch + EDID override (5.2) |
| Headset speakers and microphone | USB audio, with the WirePlumber rule (5.4) |

The files are in `patches/` of the source tree; `patches/README.md` says where each comes from and its licence.

### 5.1 Faceplate
Switch the Lighthouse faceplate on (its switch). The Valve devices (`28de:2300`, and `28de:2101` dongles) appear on
their own; pair the controllers in SteamVR as usual.

### 5.2 NVIDIA driver (RTX 50 / Blackwell; root)
> **Driver 615.71.09 and newer (RTX 5090): no patch needed.** The Blackwell DSC/display fixes are upstream, so the
> stock open module drives the Crystal Super on its own. On Ubuntu just install the packaged open module
> (`sudo apt install linux-modules-nvidia-615-open-generic nvidia-dkms-615-open` or the `-open` metapackage) — it is
> Canonical-signed, so it loads under Secure Boot with no MOK step. If you had the hand-built patched module from an
> older branch, remove that override first (e.g. `sudo mv /lib/modules/$(uname -r)/updates/omarchyvr-nvidia
> ~/nvidia-override.bak && sudo depmod -a`) so the packaged 615 module takes over, then reboot. The hand-patched steps
> below are only for **older driver branches (e.g. 595.x)** that lack the upstream fix.

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
2. A signing key (Secure Boot). The install script checks the key's name, so keep it exactly:
   ```
   mkdir -p ~/crystal-nvidia/{modules,mok,edid}
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
4. The EDID override (the headset's EDID says "colour depth undefined", and DSC refuses the 6 bpc that NVIDIA then
   uses). The headset must be awake: stop the service and keep it awake by hand in a terminal while you do this:
   ```
   systemctl --user stop pibridge-keepalived
   pibridge-keepalived --no-follow --video 90        # leave it running; Ctrl+C when done
   ```
   In a second terminal, find the headset's connector (its EDID starts with the vendor bytes `42 d2`, "PVR") and make
   the override:
   ```
   for f in /sys/class/drm/card*-DP-*/edid; do [ "$(od -An -tx1 -j8 -N2 "$f" | tr -d ' ')" = 42d2 ] && echo "$f"; done
   python3 /path/to/pibridge/patches/nvidia/blackwell/make_crystal_edid.py /sys/class/drm/cardX-DP-N/edid ~/crystal-nvidia/edid/crystal-edid.bin
   ```
5. Install (headset still awake: the script reads the headset's output name from the driver) and reboot:
   ```
   sudo /path/to/pibridge/patches/nvidia/blackwell/install-nvidia.sh ~/crystal-nvidia
   ```
   Then stop the hand-started `pibridge-keepalived` (Ctrl+C), `systemctl --user start pibridge-keepalived`, and reboot.
   `--debug` (nvidia-modeset `debug=1` at every boot) is for diagnosing a rejected mode only; running the script
   again without it removes `debug=1`.
   Undo: `sudo /path/to/pibridge/patches/nvidia/blackwell/uninstall-nvidia.sh`, then reboot. If a boot hangs, pick another kernel
   in GRUB's "Advanced options" and run `uninstall-nvidia.sh <the hung kernel's version>` there.

Arch and Fedora: steps 1 to 4 are the same; the script's initramfs step uses Ubuntu's `update-initramfs`. Install by
hand instead: the signed modules into `/lib/modules/$(uname -r)/updates/`, `depmod -a`, the EDID file and
`options nvidia-modeset config_file=...` as the script writes them, and add both files to the initramfs
(`FILES=` in `/etc/mkinitcpio.conf` on Arch, `install_items+=` in a `/etc/dracut.conf.d/` file on Fedora). A driver
update needs the module build and signing again.

### 5.3 CustomHeadsetOpenVR (the SteamVR display driver)
[CustomHeadsetOpenVR](https://github.com/sboys3/CustomHeadsetOpenVR) by sboys3, with two Linux patches: the direct
USB connection (no Pimax runtime on Linux) and the gaze from shared memory. Either use the branch that has them:
```
git clone --recurse-submodules --branch linux-crystal-super https://github.com/artscout/CustomHeadsetOpenVR.git
```
or apply them to upstream commit `4f25826` (direct-usb first):
```
git clone --recurse-submodules https://github.com/sboys3/CustomHeadsetOpenVR.git
cd CustomHeadsetOpenVR && git checkout 4f25826 && git submodule update --init --recursive
git apply /path/to/pibridge/patches/chovr/chovr-linux-direct-usb.patch
git apply /path/to/pibridge/patches/chovr/chovr-linux-eye-gaze.patch
cd ..
```
(A pull request with both is pending upstream.) Build it and register it with SteamVR:
```
cd CustomHeadsetOpenVR/CustomHeadsetOpenVR
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j8
~/.steam/steam/steamapps/common/SteamVR/bin/vrpathreg.sh adddriver "$PWD/CustomHeadsetOpenVR"
```
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
(written to `~/.config/qv-vulkan/settings.conf`). The rendering itself is done by an OpenXR layer,
[linux-vulkan-quadviews-openxr-layer](https://github.com/artscout/linux-vulkan-quadviews-openxr-layer) (`qv-vulkan`).

Install the layer into the **same prefix** as PiBridge so everything lands together under `~/.local`:
```
git clone https://github.com/artscout/linux-vulkan-quadviews-openxr-layer.git
cd linux-vulkan-quadviews-openxr-layer
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/.local
cmake --build build
cmake --install build
```
That installs the layer to `~/.local/lib/qv-vulkan/` and its OpenXR manifest to
`~/.local/share/openxr/1/api_layers/implicit.d/` — the same manifest PiBridge's Performance page renames to switch the
layer on and off. It needs the Vulkan and OpenXR headers, `glslangValidator`, and (for the layer's own stand-alone
settings app, which PiBridge replaces) Qt 6 Widgets.

Once the layer is installed, you do **not** use its own settings app: the PiBridge **Performance** page drives it.
The on/off switch there is disabled until the layer is installed, and the **Device** page's Rendering card shows
whether it is on, off or not installed.

> **Planned: one install for the whole stack.** The layer is built from its own repository today; PiBridge only
> configures it. Bundling the layer into PiBridge's own build and install (so a single `cmake --install` lays down the
> app, the services, the dashboard panel *and* the foveated-rendering layer together) is **not wired up yet** — this
> is a TODO. For now, follow the two installs above.

### 7.1 Pimax's own tracking engines (inside-out, optional)

Tracking page, **Head tracking engine: Pimax** runs Pimax Play's own head SLAM and controller LED engine under Wine
(`winbridge`, docs/WINBRIDGE.md). Needs `wine` and, once (again after a Pimax Play update), Pimax Play's files copied
from the PC's Windows into `~/.local/share/pibridge/windata`: the table and commands are in
[WINBRIDGE.md, "Copying the Pimax files"](WINBRIDGE.md#copying-the-pimax-files-once-and-again-after-a-pimax-play-update).
The files are Pimax's: personal use, they stay on this computer.

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
| `libopenvr_api.so` | loaded at run time with `dlopen` | your SteamVR install (the app finds it through `~/.config/openvr/openvrpaths.vrpath`); not linked, not shipped |
| Headset and eye chip protocols, gaze math, the two daemons, device descriptions | our code, ported | from omarchyvr (laughtt and contributors, MIT OR Apache-2.0, used as MIT; see NOTICE), Rust to C++ |
| Quad views settings code | our code, copied | from artscout-qv-vulkan-linux (MIT) |
| `pimax-eyecal` | optional fallback, run time | omarchyvr; PiBridge ships its own `pibridge-eyecal` |
| CustomHeadsetOpenVR | separate SteamVR driver | sboys3 (GPL-2.0), with our patches from `patches/chovr/` |
| NVIDIA open kernel modules | separate, root | NVIDIA's open-gpu-kernel-modules (MIT/GPL-2.0), with our patch from `patches/nvidia/blackwell/` (RTX 50) or `patches/nvidia/pre-blackwell/` (RTX 20/30/40) |
| Quad views layer | separate OpenXR layer | linux-vulkan-quadviews-openxr-layer |

PiBridge builds without network access: nothing is downloaded at build time.
