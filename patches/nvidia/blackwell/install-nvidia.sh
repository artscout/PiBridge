#!/bin/bash
# PiBridge: the patched NVIDIA open modules for driver branches older than 615.71.09 (RTX 50, Ubuntu initramfs-tools),
# with the headset's EDID override, in one go:
#  1. the patched modules (patches/nvidia/blackwell, signed with an enrolled MOK key) into
#     /lib/modules/<kernel>/updates/pibridge-nvidia, which depmod prefers over the distribution's modules; they must be
#     the same driver version as the NVIDIA libraries installed, or the desktop loses its GPU;
#  2. the EDID override of the Pimax headset (tools/headset-edid/pibridge-headset-edid apply: 8 bpc, non-desktop VR
#     headset), read by nvidia-modeset through its config_file; only the headset's output is affected.
# 615.71.09 and newer need no patched modules: run only `sudo pibridge-headset-edid apply` (docs/INSTALL.md 5.2).
# Usage: sudo install-nvidia.sh WORKDIR [--no-edid] [--debug]
#   WORKDIR holds modules/*.ko (nvidia, nvidia-modeset, nvidia-drm, nvidia-uvm, nvidia-peermem, built for the running
#   kernel and signed) and mok/MOK.der (the enrolled key).
#   --no-edid: modules only; --debug: nvidia-modeset debug=1 at every boot (DP library log in the kernel log; for a
#   diagnosis only, do not leave it on: run the script again without --debug, which writes the options without it).
# The headset must be awake (pibridge-keepalived; the original Crystal: its power button) so its output shows in
# /proc/driver/nvidia-modeset/dpys. Reboot afterwards. Undo: uninstall-nvidia.sh. If a boot hangs: pick another kernel
# in GRUB's "Advanced options" and run uninstall-nvidia.sh <hung kernel version> there.
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo "run with sudo"; exit 1; }
[[ $# -ge 1 && -d $1 ]] || { echo "usage: install-nvidia.sh WORKDIR [--no-edid] [--debug]"; exit 1; }
HERE=$(cd "$1" && pwd)
shift
K=$(uname -r)
EDID=1 DEBUG=0
for a in "$@"; do
	case $a in
	--no-edid) EDID=0 ;;
	--debug) DEBUG=1 ;;
	*) echo "unknown option $a"; exit 1 ;;
	esac
done
# the EDID helper: next to this tree in the repository, or the packaged copy
EDID_TOOL=$(dirname "$(readlink -f "$0")")/../../../tools/headset-edid/pibridge-headset-edid
[[ -x $EDID_TOOL ]] || EDID_TOOL=/usr/share/pibridge/edid/pibridge-headset-edid
MODPROBE=/etc/modprobe.d/pibridge-nvidia.conf

# --- checks before anything is changed
# mokutil exits 1 even when the key is enrolled (it warns about the kernel keyring), so only its text counts
mok=$(mokutil --test-key "$HERE/mok/MOK.der" 2>&1 || true)
[[ $mok == *"is already enrolled"* ]] || { echo "the MOK key is not enrolled: stop"; exit 1; }
for m in nvidia nvidia-modeset nvidia-drm nvidia-uvm nvidia-peermem; do
	f="$HERE/modules/$m.ko"
	[[ -f $f ]] || { echo "missing $f"; exit 1; }
	[[ $(modinfo -F vermagic "$f" | cut -d' ' -f1) == "$K" ]] || { echo "$m.ko is not built for $K"; exit 1; }
	[[ $(modinfo -F signer "$f") == "omarchyvr local NVIDIA module signing" ]] || { echo "$m.ko is not signed"; exit 1; }
done
# the kernel modules and the user-space libraries must be one driver version (2026-10-22: 595.99.02 modules under
# 615.71.09 libraries were installed by mistake; the next boot would have had no GPU acceleration)
mod_ver=$(modinfo -F version "$HERE/modules/nvidia.ko")
lib_ver=$(ls /usr/lib/x86_64-linux-gnu/libnvidia-glcore.so.* 2>/dev/null | sed -n 's/.*\.so\.//p' | head -1)
if [[ -n $lib_ver && $mod_ver != "$lib_ver" ]]; then
	echo "the modules are driver $mod_ver, the installed NVIDIA libraries $lib_ver: stop"
	echo "(615.71.09 and newer need no patched modules: sudo $EDID_TOOL apply is all)"
	exit 1
fi
[[ $EDID == 0 || -x $EDID_TOOL ]] || { echo "missing $EDID_TOOL"; exit 1; }

# --- 1. modules (the omarchyvr-named folder of an older install goes)
rm -rf "/lib/modules/$K/updates/omarchyvr-nvidia"
D=/lib/modules/$K/updates/pibridge-nvidia
install -d "$D"
install -m 644 "$HERE"/modules/*.ko "$D/"
depmod -a "$K"
echo "modules: $(modinfo -k "$K" -n nvidia_modeset)"

# --- 2. EDID override (config, module option config_file, initramfs hook)
if [[ $EDID == 1 ]]; then
	"$EDID_TOOL" apply
fi
# --debug: debug=1 next to whatever options the EDID helper wrote
opts=$(sed -n 's/^options nvidia-modeset //p' "$MODPROBE" 2>/dev/null | tr ' ' '\n' | grep -v '^debug=' | tr '\n' ' ' || true)
[[ $DEBUG == 1 ]] && opts="$opts debug=1"
opts=$(echo $opts)
if [[ -n $opts ]]; then
	echo "options nvidia-modeset $opts" > "$MODPROBE"
else
	rm -f "$MODPROBE"
fi

# --- 3. initramfs (the modules, and the override when nvidia-modeset loads from it)
update-initramfs -u -k "$K"
# list once into a variable: `lsinitramfs | grep -q` would fail under pipefail when grep stops early (SIGPIPE)
initrd=$(lsinitramfs "/boot/initrd.img-$K")
if [[ $initrd == *"nvidia-modeset.ko"* && -f /etc/pibridge/nvidia-modeset.conf && $initrd != *"etc/pibridge/pimax-edid.bin"* ]]; then
	echo "nvidia-modeset loads from the initramfs, but the EDID is NOT in it"; exit 1
fi
[[ $initrd == *"updates/pibridge-nvidia/nvidia-modeset.ko"* ]] || echo "note: the patched modules are not in the initramfs (they load from disk)"
echo "installed for $K (EDID override: $EDID, debug: $DEBUG); reboot to use it"
