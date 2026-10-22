#!/bin/sh
# SPDX-License-Identifier: MIT
# `live --source broker:<socket> --factory` as the session runs it: pibridge-ctrltrack against a pibridge-cam broker
# that serves a recording at its recorded pace (socket, XDG_RUNTIME_DIR and HOME in OUTDIR; the broker is this
# script's child, stopped by PID). Checked: it runs to --seconds and exits 0, the start lines name the broker, LED
# frames reach the tracker, and Monado's reader (SHM_DUMP, when built) sees a valid pose file with the heartbeat set.
#   live_broker.sh OUTDIR PIBRIDGE_CAM PIBRIDGE_CTRLTRACK PXCAM SHM_DUMP
set -u
out=$1 pibridge_cam=$2 exe=$3 pxcam=$4 dump=$5
[ -f "$pxcam" ] || { echo "skipped: $pxcam not present (lab data)"; exit 77; }
rm -rf "$out"
d=$out/run
mkdir -p "$d/xdg" "$d/home"
fails=0
sock=$out/b.sock
"$pibridge_cam" broker --source "$pxcam" --socket "$sock" --slots 8 2> "$d/broker.log" &
bpid=$!
i=0
while [ ! -S "$sock" ] && [ $i -lt 500 ]; do sleep 0.01; i=$((i + 1)); done
XDG_RUNTIME_DIR=$d/xdg HOME=$d/home "$exe" live --source "broker:$sock" --factory --seconds 6 --stats 2 \
    > "$d/out.txt" 2> "$d/err.txt"
echo "exit $?" >> "$d/out.txt"
kill "$bpid" 2> /dev/null
wait "$bpid" 2> /dev/null
rm -f "$sock"
grep -q '^exit 0$' "$d/out.txt" || { echo "FAIL: $(cat "$d/out.txt" "$d/err.txt" | tail -3)"; fails=$((fails + 1)); }
grep -q "source broker $sock" "$d/err.txt" || { echo "FAIL: the start lines do not name the broker"; fails=$((fails + 1)); }
n=$(grep -c '^live: [1-9][0-9]* LED frames' "$d/err.txt")
[ "$n" -ge 1 ] || { echo "FAIL: no LED frames reached the tracker"; fails=$((fails + 1)); }
if [ -x "$dump" ]; then
    "$dump" "$d/xdg/pimax-ctrltrack.pose" > "$d/pose.txt"
    grep -q '^version 1 seq even 1 beat 1 source_up 1 reopens 0$' "$d/pose.txt" ||
        { echo "FAIL: pose file: $(cat "$d/pose.txt")"; fails=$((fails + 1)); }
fi
echo "$(grep -c '^live: ' "$d/err.txt") stats lines, $(tail -1 "$d/pose.txt" 2> /dev/null)"
[ $fails = 0 ]
