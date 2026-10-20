#!/bin/sh
# SPDX-License-Identifier: MIT
# A fake sysfs tree for pimax_find_interfaces (no device is opened):
#   pimax_fake_sysfs.sh WORK [PID] [IFACE...]
# WORK/sys/class/hidraw/hidrawN/device -> WORK/sys/devices/usb1/1-1/1-1:1.I/0003:34A4:PID.000N (uevent HID_ID,
# bInterfaceNumber in the interface directory), for the interfaces given (default 0 1 2 3) of the headset with USB
# product id PID (default 0044), plus a keyboard and the display bridge (0043) that must be ignored.
set -eu
w=$1
pid=${2:-0044}
shift $(( $# >= 2 ? 2 : $# ))
ifaces=${*:-0 1 2 3}
cls=$w/sys/class/hidraw
mkdir -p "$cls" "$w/dev"
n=0
add() { # vid pid iface
    up=$(printf '%s' "$2" | tr a-f A-F)
    d=$w/sys/devices/usb1/1-$n/1-$n:1.$3
    mkdir -p "$d/0003:$1:$up.000$n"
    printf '%02x\n' "$3" > "$d/bInterfaceNumber"
    printf 'DRIVER=hid-generic\nHID_ID=0003:0000%s:0000%s\nHID_NAME=fake\n' "$1" "$up" > "$d/0003:$1:$up.000$n/uevent"
    mkdir -p "$cls/hidraw$n"
    ln -s "$d/0003:$1:$up.000$n" "$cls/hidraw$n/device"
    n=$((n + 1))
}
add 046D C31C 0
for i in $ifaces; do
    add 34A4 "$pid" "$i"
done
add 34A4 0043 0
