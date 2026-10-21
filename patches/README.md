# Setup patches and helpers

Everything here is for the **Pimax Crystal Super with the Lighthouse faceplate under SteamVR**: PiBridge's own code does
not need any of it to build. The files are copies from [omarchyvr](https://github.com/laughtt/omarchyvr) (our
Crystal Super work there, commit `6142188`, 2026-10-18), with their content unchanged except where noted. How to use
them, step by step: [docs/INSTALL.md](../docs/INSTALL.md).

| Folder | What | From (omarchyvr) | Licence |
|---|---|---|---|
| `chovr/` | Two patches for the SteamVR display driver CustomHeadsetOpenVR, and the C header of the gaze file | `linux/crystal-super/` | patches: GPL-2.0 (CustomHeadsetOpenVR's licence, they change its code); header: MIT OR Apache-2.0 |
| `nvidia/` | NVIDIA open kernel module patch sets per GPU generation and driver version: `blackwell/` (RTX 50: the Crystal Super's FEC re-read, the EDID override maker, the install and uninstall scripts) and `pre-blackwell/` (RTX 20/30/40: the Dream Air's 2Heads1OR patches for 610.57.04, 615.78.08 and 595.99.02, an untested 2Heads1OR patch for the Crystal Super, the Arch re-apply script and pacman hooks) | `patches/nvidia/`, `patches/nvidia/blackwell/`, `linux/crystal-super/` | patches: MIT/GPL-2.0 dual, as NVIDIA's open-gpu-kernel-modules; scripts and notes: MIT (omarchyvr is MIT OR Apache-2.0, used here as MIT) |
| `audio/` | WirePlumber rule for the headset's USB audio | `linux/crystal-super/` | MIT (as above) |
| `monado/` | `monado-pimax-use-pibridge.patch`: the Monado CMake switch `PIMAX_USE_PIBRIDGE` as a standalone diff (to apply on a vanilla Monado `src/xrt/drivers/CMakeLists.txt`). Normally you do **not** need it: the full Monado source is bundled at [`third_party/monado/`](../third_party/monado) with this switch already applied. Build recipe: [docs/monado-abi.md](../docs/monado-abi.md), "The PiBridge-linked Monado" | new (our change to the Monado fork) | BSL-1.0 (Monado's licence) |

The udev rule is our own: [`udev/70-pibridge.rules`](../udev/70-pibridge.rules) (the three access lines of omarchyvr's
`70-pimax.rules`, without its hub power helper).

## chovr/: CustomHeadsetOpenVR on Linux

[CustomHeadsetOpenVR](https://github.com/sboys3/CustomHeadsetOpenVR) by sboys3 is the SteamVR driver that gives
SteamVR the headset's display (EDID vendor, render size, distortion, IPD). With the Lighthouse faceplate SteamVR's own
lighthouse driver does the tracking.

| File | What it does |
|---|---|
| `chovr-linux-direct-usb.patch` | Without the Pimax runtime (Linux), connect to the headset over USB before deciding on the Lighthouse shim |
| `chovr-linux-eye-gaze.patch` | Read the gaze from shared memory and serve SteamVR's eye tracking: OpenXR apps get `XR_EXT_eye_gaze_interaction`. The gaze file is now `/dev/shm/pibridge-eye-gaze` (written by `pibridge-eyed`); this unmodified patch still reads the old name `/dev/shm/omarchyvr-eye-gaze`, which `pibridge-eyed` symlinks to the new file for binary compatibility (or set `CUSTOM_HEADSET_EYE_GAZE_SHM`) |
| `omarchyvr_eye_gaze.h` | The layout of that file (136 bytes, magic "OVEG", seqlock) for C and C++ readers. It keeps its omarchyvr name: the driver patch, PiBridge's docs and other readers know it by that name, and PiBridge writes the file byte for byte as omarchyvr does |

Apply on upstream commit `4f25826` ("Filter cropping and 100% resolution for other headsets"), direct-usb first:

```
git clone --recurse-submodules https://github.com/sboys3/CustomHeadsetOpenVR.git
cd CustomHeadsetOpenVR
git checkout 4f25826 && git submodule update --init --recursive
git apply /path/to/pibridge/patches/chovr/chovr-linux-direct-usb.patch
git apply /path/to/pibridge/patches/chovr/chovr-linux-eye-gaze.patch
```

Or use the branch with both patches already applied: `linux-crystal-super` of
[artscout/CustomHeadsetOpenVR](https://github.com/artscout/CustomHeadsetOpenVR). A pull request to sboys3's repository
is pending; once it is merged, upstream works as it is.

## nvidia/: the display, per GPU generation

Details, the GPU x headset x driver table and what is tested: [nvidia/README.md](nvidia/README.md).

| Path | What it is |
|---|---|
| `nvidia/blackwell/595.99.02/0001-nvidia-dp-fec-reread.patch` | RTX 50 + Crystal Super: re-read the sink's FEC capability in the SST DSC check, and no NULL dereference there (tested, RTX 5090) |
| `nvidia/blackwell/make_crystal_edid.py` | Makes the EDID override from the headset's own EDID: 8 bpc instead of "undefined", plus a Microsoft VSDB (non-desktop VR headset) |
| `nvidia/blackwell/install-nvidia.sh`, `uninstall-nvidia.sh` | As root: install the patched, signed modules, the EDID override and an initramfs hook (Ubuntu/Debian initramfs-tools); back to the distribution's driver |
| `nvidia/pre-blackwell/<version>/0001`-`0003` | RTX 20/30/40 + Dream Air: 2Heads1OR viewport, flip-hang fixes, bounded EVO wait (610.57.04 tested by omarchyvr; 615.78.08 and 595.99.02 ports untested) |
| `nvidia/pre-blackwell/<version>/0004-nvidia-2h1or-vr-wider-than-5120.patch` | RTX 20/30/40 + Crystal Super: 2Heads1OR for a VR headset wider than 5120 (untested) |
| `nvidia/pre-blackwell/nvidia-patch-reapply.sh`, `*.hook`, `*-versions.txt` | omarchyvr's re-apply script and pacman hooks (Arch, nvidia-open-dkms) |

Changed from the omarchyvr copies: the patches are `git format-patch` files with a description header and numbered
names (`0001`-`0004`), one folder per driver version; the re-apply script uses those names and `610.57.04/` as its
default set; `595.99.02/` (the port) and `0004` are new; one comment path in `install-nvidia.sh`
(`patches/nvidia/blackwell`), and its `--debug` help. The install scripts still install under the names they had
(`/lib/modules/<kernel>/updates/omarchyvr-nvidia`, `/etc/omarchyvr/`, `/etc/modprobe.d/omarchyvr-nvidia.conf`) and
check that the modules are signed by a key named `omarchyvr local NVIDIA module signing`, so an install made from
omarchyvr and one made from here are the same and either uninstall script removes it.

## audio/: headset sound

`51-pimax-crystal-super.conf` goes to `~/.config/wireplumber/wireplumber.conf.d/`. The speakers are ALSA device 1 of
the headset's card and the stock analog profiles only look at device 0; the rule picks the pro-audio profile and
gives the nodes plain names.
