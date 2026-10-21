# NVIDIA open kernel module patches

Patches for NVIDIA's [open-gpu-kernel-modules](https://github.com/NVIDIA/open-gpu-kernel-modules), per GPU generation
and driver version, for the Pimax Crystal Super and the Pimax Dream Air. Each patch is a `git format-patch` file whose
header says what it changes, why, for which GPUs and headsets, and whether it was tested. Each version folder has a
`series` file with the order to apply them in. Every patch applies with `git apply` (no fuzz) to the pristine tag of
the open-gpu-kernel-modules release its folder is named after (checked 2026-10-19).

The two GPU generations need different patches: Blackwell (RTX 50) has no 2Heads1OR (merged heads,
`nvkms-evo4.c`: `supportsMergeMode = FALSE`); it drives the headsets' 7104x3840 DSC modes with one head. RTX 20/30/40
(Turing, Ampere, Ada: `nvEvoC5`/`nvEvoC6` in `nvkms-evo3.c`) use 2Heads1OR for them.

## Which patch for which setup

| GPU generation | Headset | Driver | Patches | Status |
|---|---|---|---|---|
| RTX 50 (Blackwell) | Crystal Super | 595.99.02 | `blackwell/595.99.02/0001-nvidia-dp-fec-reread.patch` | **tested**: RTX 5090 + Crystal Super Micro-OLED, Ubuntu 26.04, kernel 7.0, 2026-10-18 (with the EDID override, see below) |
| RTX 50 (Blackwell) | Crystal Super | **615.71.09+** | **none needed** | **tested 2026-10-21**: stock Ubuntu `linux-modules-nvidia-615-open` (615.71.09, Canonical-signed) drives the Crystal Super with a picture — the FEC/DSC fix is upstream, so no patch. Install the packaged `-open` module and remove any older hand-built override |
| RTX 50 (Blackwell) | Crystal Super | 610.57.04, 615.78.08 | the same file | applies unchanged (`git apply --check`); not built, not tested (615.71.09 already works without it) |
| RTX 50 (Blackwell) | Dream Air | any | none known | not tried |
| RTX 20/30/40 | Dream Air | 610.57.04 | `pre-blackwell/610.57.04/0001`-`0003` | **tested** by omarchyvr (its `tested-versions.txt`; RTX 40). The original header of `0002` was written before that hardware run and still says "not verified" |
| RTX 20/30/40 | Dream Air | 615.78.08 | `pre-blackwell/615.78.08/0001`-`0003` | ported by omarchyvr, compile-checked, **untested** on hardware |
| RTX 20/30/40 | Dream Air | 595.99.02 | `pre-blackwell/595.99.02/0001`-`0003` | ported here (fuzz and two API changes), built and loaded on an RTX 5090 only, where the code is never reached: **untested** |
| RTX 20/30/40 | Crystal Super | 595.99.02, 610.57.04, 615.78.08 | `pre-blackwell/<version>/0004-nvidia-2h1or-vr-wider-than-5120.patch`, on top of `0001`-`0003` | **untested**: never run on a GPU that uses 2Heads1OR; on 610/615 only checked to apply |

The patches:

| Patch | What it does |
|---|---|
| `0001-nvidia-dp-fec-reread.patch` (blackwell) | DP library: re-read the sink's FEC capability in the SST DSC check when the cached value says no, and do not dereference a missing native device there |
| `0001-nvidia-2h1or-viewport.patch` | Each merged head starts at its own section of the surface (else the left eye shows in both eyes); same as NVIDIA PR #1262 |
| `0002-nvidia-2h1or-flip-fix.patch` | Flip hangs on merged heads: sync objects on the primary head only (module parameter `merged_head_sync`), force-idle of all merged windows, DRM lease revoke for killed processes, diagnostics |
| `0003-nvidia-evo-makeroom-bounded-wait.patch` | Bounded wait in the EVO pushbuffer (a stuck display channel no longer hangs the desktop); by Rouan (rvdende), from PR #1262 |
| `0004-nvidia-2h1or-vr-wider-than-5120.patch` | Use 2Heads1OR for a VR headset wider than single-head DSC allows (5120), even when one head could drive its pixel clock (the Crystal Super's 7104x3840 at 2028 / 2564 MHz) |

## blackwell/: the Crystal Super on RTX 50

| File | What it does |
|---|---|
| `595.99.02/0001-nvidia-dp-fec-reread.patch`, `595.99.02/series` | The only driver change the Crystal Super needs on RTX 50 |
| `make_crystal_edid.py` | Makes the EDID override from the headset's own EDID: 8 bpc instead of "undefined", plus a Microsoft VSDB (non-desktop VR headset) |
| `install-nvidia.sh` | As root: installs the patched, signed modules, the EDID override and an initramfs hook (Ubuntu/Debian initramfs-tools) |
| `uninstall-nvidia.sh` | As root: back to the distribution's driver |

Why the patch is needed. The Crystal Super's only modes are 7104x3840 at 90 Hz (2564 MHz) and 72 Hz (2028 MHz). Its
link is DP 1.4 HBR3 x4 (no UHBR), so both need DSC (sink: DSC 1.2, max 8.0 bpp, FEC). Without the patch nvidia-drm
lists no mode for the headset ("Mode is rejected: DP Bandwidth check failed"), because:

1. The DP library reads the sink's FEC capability (DPCD 0x90) once, when it detects the sink. The headset's display
   bridge still NAKs AUX reads at that moment ("AuxChCtl Failing"), the cached answer stays "no FEC", and
   `compoundQueryAttachSSTIsDscPossible` refuses DSC for the whole session. The patch reads FEC again there.
2. The same function dereferenced the native device without a check. A mode can be validated before the DP library
   has found the sink (with an EDID override, during the boot-time fbdev probe): that was a NULL dereference. The
   patch adds the check.

A second fix is needed outside the driver: the headset's EDID says "colour depth undefined" (base block byte 20 =
0x85), NVKMS then validates at 6 bpc and DSC refuses 6 bpc. `make_crystal_edid.py` makes an EDID override with 8 bpc.

Left out on purpose: debug prints, the 2Heads1OR changes (never reached on Blackwell) and a DSC slice height module
parameter (240-line slices instead of the automatic choice changed nothing visible).

## pre-blackwell/: the Dream Air (and, untested, the Crystal Super) on RTX 20/30/40

| File | What it does |
|---|---|
| `<version>/0001`-`0004`, `<version>/series` | The patches for that driver version (`0004` is optional and untested) |
| `nvidia-patch-reapply.sh` | Applies `0001`-`0003` idempotently to `/usr/src/nvidia-<version>` (`--check`, `--verify`, `--rebuild`, `--allow-untested`, `--fallback`); a version without its own folder gets the `610.57.04/` set, and only with the opt-in |
| `tested-versions.txt`, `ported-versions.txt` | Versions the script patches without the opt-in (tested), and versions with a clean port that still need it |
| `nvidia-pimax-patch.hook`, `nvidia-pimax-verify.hook` | pacman hooks: re-apply before the dkms build, check `merged_head_sync` after it |

The script and the hooks are omarchyvr's, for **Arch Linux** (Omarchy): the `nvidia-open-dkms` package, dkms,
mkinitcpio or Limine. Changed here: the patch names (`0001`-`0003`) and the default set (`610.57.04/` instead of the
script's own folder). Its messages still name the omarchyvr app, and it reads `/etc/omarchyvr/nvidia.conf` and writes
`/var/lib/omarchyvr/nvidia-patch-status`. To use the hooks, copy this folder to `/usr/local/lib/omarchyvr/` and the
hooks to `/etc/pacman.d/hooks/65-nvidia-pimax-patch.hook` and `75-nvidia-pimax-verify.hook` (as root). `0004` is never
applied by the script. On other distributions apply the patches by hand (below).

## Apply and build

Distribution packages (Ubuntu's among them) ship nvidia-modeset's core as a prebuilt object, so build the open modules
from the tag that matches the installed driver (`modinfo -F version nvidia`):

```
git clone --depth 1 --branch 595.99.02 https://github.com/NVIDIA/open-gpu-kernel-modules
cd open-gpu-kernel-modules
for p in $(cat /path/to/pibridge/patches/nvidia/blackwell/595.99.02/series); do
    git apply /path/to/pibridge/patches/nvidia/blackwell/595.99.02/$p
done
nice -n 19 make modules -j8 SYSSRC=/lib/modules/$(uname -r)/build
```

(`git am` instead of `git apply` keeps the descriptions as commits; `patch -p1 < file` works too.) The five modules
are then in `kernel-open/`. Under Secure Boot sign them with an enrolled MOK key
(`/lib/modules/$(uname -r)/build/scripts/sign-file sha256 MOK.priv MOK.der <module>`). Step by step, with the key and
the EDID override: [docs/INSTALL.md](../../docs/INSTALL.md), section 5.2.

## Install and uninstall (Crystal Super on RTX 50, Ubuntu/Debian)

`sudo blackwell/install-nvidia.sh WORKDIR` (WORKDIR holds `modules/*.ko`, `mok/MOK.der`, `edid/crystal-edid.bin`;
the headset must be awake) puts the signed modules into `/lib/modules/<kernel>/updates/omarchyvr-nvidia`, the EDID
override into `/etc/omarchyvr/`, the module options into `/etc/modprobe.d/omarchyvr-nvidia.conf`, an initramfs hook
into `/etc/initramfs-tools/hooks/omarchyvr-edid`, and rebuilds the initramfs. Reboot afterwards.
`sudo blackwell/uninstall-nvidia.sh [kernel version]` removes all of it and rebuilds the initramfs; if a boot hangs,
pick another kernel in GRUB's "Advanced options" and run it there with the hung kernel's version. The names stay
omarchyvr's (`omarchyvr-nvidia`, signer `omarchyvr local NVIDIA module signing`), so an install made from omarchyvr
and one made from here are the same and either uninstall script removes it.

### The debug option

`--debug` adds `debug=1` to the nvidia-modeset options at every boot: NVKMS and the DP library then log their
decisions (mode validation, link training, DSC) to the kernel log. Use it only to diagnose a rejected mode, and do not
leave it on: it fills the kernel log and, with a tree that carries extra prints, the boot screen (tens of thousands of
lines), which slows the boot and buries real errors. To turn it off run `install-nvidia.sh` again without `--debug`:
it rewrites `/etc/modprobe.d/omarchyvr-nvidia.conf` without `debug=1` (or removes the file when no option is left)
and rebuilds the initramfs.

## Licence and credit

The NVIDIA open kernel modules are MIT / GPL-2.0 (dual); every patch here is offered under the same dual licence.
The pre-Blackwell patches, the re-apply script and the hooks come from [omarchyvr](https://github.com/laughtt/omarchyvr)
(laughtt and contributors, `patches/nvidia/`; omarchyvr is MIT OR Apache-2.0, used here as MIT); `0003` is by Rouan
(rvdende), posted in NVIDIA/open-gpu-kernel-modules PR #1262. The FEC patch, the 595.99.02 port and `0004` were made
for the Crystal Super work. The e-mail addresses in the patches' `From:` lines are placeholders (`.invalid`).
