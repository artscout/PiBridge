#!/bin/sh
# SPDX-License-Identifier: MIT
# Run one pimax-ctrltrack / pibridge-ctrltrack command with stderr merged into stdout (both tools write stdout line by
# line, so the order is the program order) and mask what differs between any two runs of the same tool: wall-clock
# durations, heartbeat and pose ages, writer pids, the LED-frame count of the last heartbeat (written every 0.1 s, so it
# depends on when the writer stopped), the panicking thread's id and location.
#   ct_run.sh EXE ARGS...
exe=$1
shift
tmp=$(mktemp)
"$exe" "$@" > "$tmp" 2>&1
st=$?
sed -E -e "s/^thread 'main' \([0-9]+\) panicked at .*:\$/thread 'main' panicked at X:/" \
    -e 's/(blobs|acquisition) in [0-9.]+ s/\1 in X s/' \
    -e 's/frame work [0-9.]+ ms/frame work X ms/' \
    -e 's/beat +[0-9]+ ms/beat X ms/' -e 's/age +[0-9]+ ms/age X ms/g' \
    -e 's/"pid":[0-9]+/"pid":P/' -e 's/"led_frames":[0-9]+/"led_frames":N/' -e 's/"age_s":[0-9.]+/"age_s":X/' "$tmp"
rm -f "$tmp"
exit $st
