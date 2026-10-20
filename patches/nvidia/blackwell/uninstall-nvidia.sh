#!/bin/bash
# omarchyvr: back to the stock Ubuntu NVIDIA driver: removes the patched modules, the EDID override, the module
# options and the initramfs hook (install-nvidia.sh), then rebuilds the initramfs. Run with sudo, then reboot.
# Usage: uninstall-nvidia.sh [kernel version]  (default: the running kernel; from a rescue boot give the hung kernel's version)
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo "run with sudo"; exit 1; }
K=${1:-$(uname -r)}
rm -rf "/lib/modules/$K/updates/omarchyvr-nvidia"
rm -f /etc/modprobe.d/omarchyvr-nvidia.conf /etc/initramfs-tools/hooks/omarchyvr-edid
rm -rf /etc/omarchyvr
depmod -a "$K"
update-initramfs -u -k "$K"
echo "stock NVIDIA driver restored for $K; reboot"
