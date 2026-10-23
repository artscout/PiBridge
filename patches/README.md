# Setup patches and helpers

Everything here is for the **Pimax Crystal Super with the Lighthouse faceplate under SteamVR**: PiBridge's own code does
not need any of it to build. The files are copies from [omarchyvr](https://github.com/laughtt/omarchyvr) (our
Crystal Super work there, commit `6142188`, 2026-10-18), with their content unchanged except where noted. How to use
them, step by step: [docs/INSTALL.md](../docs/INSTALL.md).

| Folder | What | From (omarchyvr) | Licence |
|---|---|---|---|
| `chovr/` | Five patches for the SteamVR display driver CustomHeadsetOpenVR, and the C header of the gaze file | `linux/crystal-super/` | patches: GPL-2.0 (CustomHeadsetOpenVR's licence, they change its code); header: MIT OR Apache-2.0 |
| `nvidia/` | NVIDIA open kernel module patch sets per GPU generation and driver version: `blackwell/` (RTX 50: the Crystal Super's FEC re-read, the EDID override maker, the install and uninstall scripts) and `pre-blackwell/` (RTX 20/30/40: the Dream Air's 2Heads1OR patches for 610.57.04, 615.78.08 and 595.99.02, an untested 2Heads1OR patch for the Crystal Super, the Arch re-apply script and pacman hooks) | `patches/nvidia/`, `patches/nvidia/blackwell/`, `linux/crystal-super/` | patches: MIT/GPL-2.0 dual, as NVIDIA's open-gpu-kernel-modules; scripts and notes: MIT (omarchyvr is MIT OR Apache-2.0, used here as MIT) |
| `audio/` | WirePlumber rule for the headset's USB audio | `linux/crystal-super/` | MIT (as above) |

The udev rule is our own: [`udev/70-pibridge.rules`](../udev/70-pibridge.rules) (the three access lines of omarchyvr's
`70-pimax.rules`, without its hub power helper).

## chovr/: CustomHeadsetOpenVR on Linux

[CustomHeadsetOpenVR](https://github.com/sboys3/CustomHeadsetOpenVR) by sboys3 is the SteamVR driver that gives
SteamVR the headset's display (EDID vendor, render size, distortion, IPD). With the Lighthouse faceplate SteamVR's own
lighthouse driver does the tracking.

| File | What it does |
|---|---|
| `chovr-linux-direct-usb.patch` | Without the Pimax runtime (Linux), connect to the headset over USB before deciding on the Lighthouse shim |
| `chovr-linux-eye-gaze.patch` | Read the gaze from shared memory and serve SteamVR's eye tracking: OpenXR apps get `XR_EXT_eye_gaze_interaction`. The gaze file is `/dev/shm/pibridge-eye-gaze` (written by `pibridge-eyed`; `CUSTOM_HEADSET_EYE_GAZE_SHM` overrides the path) |
| `chovr-linux-pimax-dump-profile.patch` | The "Pimax Builtin (from dump)" distortion profile: Pimax's own lens distortion without the Pimax runtime, from a dump of its `pvr_getHmdDistortedUV` (`pimaxDumpPath`, `pimaxDumpSquareDomain`) |
| `chovr-linux-crystal.patch` | The original Pimax Crystal (P3B, `34A4:0012`): identified by its USB product id, its HID left to `pibridge-keepalived`; its own inside-out as CHOVR's SLAM headset with head and controller poses, buttons and battery from `/dev/shm/pibridge-crystal-pose` (written by `pibridge-keepalived`; `CUSTOM_HEADSET_POSE_SHM` overrides it), controllers as Touch; `refreshRate` per headset (72 / 90 / 120 Hz for the Crystal, also SteamVR's `preferredRefreshRate`) |
| `chovr-linux-headtrack.patch` | Inside-out of the Crystal Super / Dream Air ([pibridge-headtrack](../docs/pibridge-headtrack.md)): the pose file's source field (1 the original Crystal via `pibridge-keepalived`, 2 `pibridge-headtrack`) picks which headset the SLAM headset stands for; controller vibration goes to `pibridge-headtrack` (`$XDG_RUNTIME_DIR/pimax-haptic-request`, "SLOT AMP MS") |
| `pibridge_eye_gaze.h` | The layout of the gaze file (136 bytes, magic "OVEG", seqlock) for C and C++ readers |

Apply on upstream commit `4f25826` ("Filter cropping and 100% resolution for other headsets"), in this order:

```
git clone --recurse-submodules https://github.com/sboys3/CustomHeadsetOpenVR.git
cd CustomHeadsetOpenVR
git checkout 4f25826 && git submodule update --init --recursive
git apply /path/to/pibridge/patches/chovr/chovr-linux-direct-usb.patch
git apply /path/to/pibridge/patches/chovr/chovr-linux-eye-gaze.patch
git apply /path/to/pibridge/patches/chovr/chovr-linux-pimax-dump-profile.patch
git apply /path/to/pibridge/patches/chovr/chovr-linux-crystal.patch
git apply /path/to/pibridge/patches/chovr/chovr-linux-headtrack.patch
```

The branch `linux-crystal-super` of [artscout/CustomHeadsetOpenVR](https://github.com/artscout/CustomHeadsetOpenVR) has
the first two applied (enough for the Crystal Super with the Lighthouse faceplate); the original Crystal needs the
first four, inside-out on the Crystal Super or the Dream Air all five. The changes are offered to
sboys3; once upstream has them, it works as it is.

## nvidia/: the display, per GPU generation

Details, the GPU x headset x driver table and what is tested: [nvidia/README.md](nvidia/README.md).

| Path | What it is |
|---|---|
| `nvidia/blackwell/595.99.02/0001-nvidia-dp-fec-reread.patch` | RTX 50 + Crystal Super: re-read the sink's FEC capability in the SST DSC check, and no NULL dereference there (tested, RTX 5090) |
| `nvidia/blackwell/install-nvidia.sh`, `uninstall-nvidia.sh` | As root: install the patched, signed modules and the EDID override (`tools/headset-edid/pibridge-headset-edid`, which also works alone, for every driver); back to the distribution's driver |
| `nvidia/pre-blackwell/<version>/0001`-`0003` | RTX 20/30/40 + Dream Air: 2Heads1OR viewport, flip-hang fixes, bounded EVO wait (610.57.04 tested by omarchyvr; 615.78.08 and 595.99.02 ports untested) |
| `nvidia/pre-blackwell/<version>/0004-nvidia-2h1or-vr-wider-than-5120.patch` | RTX 20/30/40 + Crystal Super: 2Heads1OR for a VR headset wider than 5120 (untested) |
| `nvidia/pre-blackwell/nvidia-patch-reapply.sh`, `*.hook`, `*-versions.txt` | omarchyvr's re-apply script and pacman hooks (Arch, nvidia-open-dkms) |

Changed from the omarchyvr copies: the patches are `git format-patch` files with a description header and numbered
names (`0001`-`0004`), one folder per driver version; the re-apply script uses those names and `610.57.04/` as its
default set; `595.99.02/` (the port) and `0004` are new; one comment path in `install-nvidia.sh`
(`patches/nvidia/blackwell`), and its `--debug` help. The install scripts now use PiBridge's names
(`/lib/modules/<kernel>/updates/pibridge-nvidia`, `/etc/pibridge/`, `/etc/modprobe.d/pibridge-nvidia.conf`), check
that the modules match the installed NVIDIA libraries, leave the EDID override to `tools/headset-edid/`, and replace
or remove an install under the omarchyvr names; the modules must still be signed by a key named `omarchyvr local
NVIDIA module signing`, so keys enrolled for omarchyvr work.

## audio/: headset sound

`51-pimax-crystal-super.conf` goes to `~/.config/wireplumber/wireplumber.conf.d/`. The speakers are ALSA device 1 of
the headset's card and the stock analog profiles only look at device 0; the rule picks the pro-audio profile and
gives the nodes plain names.
