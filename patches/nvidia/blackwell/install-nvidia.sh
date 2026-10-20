#!/bin/bash
# omarchyvr: everything the Pimax Crystal Super needs from the NVIDIA driver (RTX 50, Ubuntu initramfs-tools), in one go:
#  1. the patched NVIDIA open modules (patches/nvidia/blackwell, signed with an enrolled MOK key) into
#     /lib/modules/<kernel>/updates/omarchyvr-nvidia, which depmod prefers over the distribution's modules;
#  2. the EDID override of the headset (make_crystal_edid.py: 8 bpc, non-desktop VR headset), read by nvidia-modeset
#     through its config_file; only the headset's output is affected;
#  3. an initramfs-tools hook, because the modules load from the initramfs and read the config there.
# Usage: sudo install-nvidia.sh WORKDIR [--no-edid] [--debug]
#   WORKDIR holds modules/*.ko (nvidia, nvidia-modeset, nvidia-drm, nvidia-uvm, nvidia-peermem, built for the running
#   kernel and signed), mok/MOK.der (the enrolled key) and edid/crystal-edid.bin.
#   --no-edid: modules only; --debug: nvidia-modeset debug=1 at every boot (DP library log in the kernel log; for a
#   diagnosis only, do not leave it on: run the script again without --debug, which writes the options without it).
# The headset must be awake (omarchyvr keepalived) so its output shows in /proc/driver/nvidia-modeset/dpys.
# Reboot afterwards. Undo: uninstall-nvidia.sh. If a boot hangs: pick another kernel in GRUB's "Advanced options"
# and run uninstall-nvidia.sh <hung kernel version> there.
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
[[ $EDID == 0 || -f "$HERE/edid/crystal-edid.bin" ]] || { echo "missing $HERE/edid/crystal-edid.bin"; exit 1; }
if [[ $EDID == 1 ]]; then
	# the override key: the GPU's PCI address and NVKMS's name of the headset's output (not the DRM name: on the
	# test PC the DRM connector DP-6 is NVKMS's DP-4)
	# nvidia-smi prints an 8-digit domain (00000000:01:00.0); the key wants dddd:bb:ss.f
	PCI=$(nvidia-smi --query-gpu=pci.bus_id --format=csv,noheader | head -1 | tr 'A-F' 'a-f')
	PCI=${PCI: -12}
	DPY=$(grep -o 'Pimax[^(]*(\(DP-[0-9]*\))' /proc/driver/nvidia-modeset/dpys | grep -o 'DP-[0-9]*' | head -1)
	[[ -n $PCI && -n $DPY ]] || { echo "cannot find the GPU or the headset's output (is the headset awake?)"; exit 1; }
	KEY="override.[$PCI].$DPY"
	echo "EDID override key: $KEY"
fi

# --- 1. modules
D=/lib/modules/$K/updates/omarchyvr-nvidia
install -d "$D"
install -m 644 "$HERE"/modules/*.ko "$D/"
depmod -a "$K"
echo "modules: $(modinfo -k "$K" -n nvidia_modeset)"

# --- 2. EDID override and module options
OPTS=""
if [[ $EDID == 1 ]]; then
	install -d /etc/omarchyvr
	install -m 644 "$HERE/edid/crystal-edid.bin" /etc/omarchyvr/crystal-edid.bin
	echo "$KEY=/etc/omarchyvr/crystal-edid.bin" > /etc/omarchyvr/nvidia-modeset.conf
	OPTS="config_file=/etc/omarchyvr/nvidia-modeset.conf"
fi
[[ $DEBUG == 1 ]] && OPTS="$OPTS debug=1"
if [[ -n $OPTS ]]; then
	echo "options nvidia-modeset ${OPTS# }" > /etc/modprobe.d/omarchyvr-nvidia.conf
else
	rm -f /etc/modprobe.d/omarchyvr-nvidia.conf
fi

# --- 3. initramfs
if [[ $EDID == 1 ]]; then
	cat > /etc/initramfs-tools/hooks/omarchyvr-edid <<'HOOK'
#!/bin/sh
# omarchyvr: nvidia-modeset loads from the initramfs and reads its config_file there.
PREREQ=""
prereqs() { echo "$PREREQ"; }
case "$1" in prereqs) prereqs; exit 0 ;; esac
mkdir -p "$DESTDIR/etc/omarchyvr"
cp /etc/omarchyvr/nvidia-modeset.conf /etc/omarchyvr/crystal-edid.bin "$DESTDIR/etc/omarchyvr/"
HOOK
	chmod 755 /etc/initramfs-tools/hooks/omarchyvr-edid
else
	rm -f /etc/initramfs-tools/hooks/omarchyvr-edid
fi
update-initramfs -u -k "$K"
# list once into a variable: `lsinitramfs | grep -q` would fail under pipefail when grep stops early (SIGPIPE)
initrd=$(lsinitramfs "/boot/initrd.img-$K")
if [[ $EDID == 1 && $initrd != *"etc/omarchyvr/crystal-edid.bin"* ]]; then
	echo "the EDID is NOT in the initramfs"; exit 1
fi
[[ $initrd == *"updates/omarchyvr-nvidia/nvidia-modeset.ko"* ]] || echo "note: the patched modules are not in the initramfs (they load from disk)"
echo "installed for $K (EDID override: $EDID, debug: $DEBUG); reboot to use it"
