#!/bin/sh
# SPDX-License-Identifier: MIT
# omarchyvr's unmodified libpimax/tools/pimax-cli.c built against libpibridge_pimax.a: its paths without hardware (twin
# of pimax-capi tests/abi.rs unmodified_pimax_cli_non_hardware_paths). The headset search runs on an empty sysfs tree.
#   pimax_cli_check.sh PIMAX_CLI WORKDIR      (WORKDIR is created; it must not exist or be a scratch folder)
set -u
cli=$1
w=$2
mkdir -p "$w/sys/class/hidraw"
fail=0
"$cli" > "$w/out" 2> "$w/err"; rc=$?
{ [ $rc = 2 ] && grep -q "usage:" "$w/err"; } || { echo "no args: exit $rc"; cat "$w/err"; fail=1; }
"$cli" bogus > "$w/out" 2> "$w/err"; rc=$?
{ [ $rc = 2 ] && grep -q "unknown subcommand bogus" "$w/err"; } || { echo "bogus: exit $rc"; cat "$w/err"; fail=1; }
# no headset (hermetic): waits 10 s, then "headset not found", exit 1
PIMAX_SYSFS_ROOT=$w/sys PIBRIDGE_SYSFS_ROOT=$w/sys PIMAX_DEV_ROOT=$w/sys PIBRIDGE_DEV_ROOT=$w/sys "$cli" info \
    > "$w/out" 2> "$w/err"; rc=$?
{ [ $rc = 1 ] && grep -q "headset not found" "$w/err"; } || { echo "info: exit $rc"; cat "$w/err"; fail=1; }
[ $fail = 0 ] && echo "pimax-cli: usage, unknown subcommand and no headset as with the Rust library"
exit $fail
