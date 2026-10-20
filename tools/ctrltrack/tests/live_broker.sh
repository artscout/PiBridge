#!/bin/sh
# SPDX-License-Identifier: MIT
# `live --source broker:<socket> --factory` as the session runs it, for the Rust oracle and for pibridge-ctrltrack: each
# against its own pibridge-cam broker that serves a recording at its recorded pace (sockets, XDG_RUNTIME_DIR and HOME in
# OUTDIR; the brokers are this script's children, stopped by PID). Frames before the client connects are lost, so the
# poses are not compared; checked: both run to --seconds and exit 0, the same start lines (the source description
# names the broker and its recording), LED frames reach the tracker, and Monado's reader sees a valid pose file with
# the heartbeat set.
#   live_broker.sh OUTDIR PIBRIDGE_CAM RUST_CTRLTRACK PIBRIDGE_CTRLTRACK PXCAM SHM_DUMP
set -u
out=$1 pibridge_cam=$2 rust=$3 pibridge=$4 pxcam=$5 dump=$6
[ -x "$rust" ] || { echo "skipped: no Rust oracle at $rust"; exit 77; }
[ -f "$pxcam" ] || { echo "skipped: $pxcam not present (lab data)"; exit 77; }
rm -rf "$out"
mkdir -p "$out"
fails=0
for t in rust pibridge; do
    exe=$rust
    [ $t = pibridge ] && exe=$pibridge
    # the same paths for both tools (they appear in the start lines); the results are kept in OUTDIR/<tool>
    d=$out/run
    rm -rf "$d"
    mkdir -p "$d/xdg" "$d/home"
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
    head -3 "$d/err.txt" | sed -E 's/\(aim pitch.*//' > "$d/start.txt"
    [ -x "$dump" ] && "$dump" "$d/xdg/pimax-ctrltrack.pose" > "$d/pose.txt"
    grep -q '^exit 0$' "$d/out.txt" || { echo "FAIL $t: $(cat "$d/out.txt" "$d/err.txt" | tail -3)"; fails=$((fails + 1)); }
    n=$(grep -c '^live: [1-9][0-9]* LED frames' "$d/err.txt")
    [ "$n" -ge 1 ] || { echo "FAIL $t: no LED frames reached the tracker"; fails=$((fails + 1)); }
    if [ -x "$dump" ]; then
        grep -q '^version 1 seq even 1 beat 1 source_up 1 reopens 0$' "$d/pose.txt" ||
            { echo "FAIL $t: pose file: $(cat "$d/pose.txt")"; fails=$((fails + 1)); }
    fi
    echo "$t: $(grep -c '^live: ' "$d/err.txt") stats lines, $(tail -1 "$d/pose.txt" 2> /dev/null)"
    mv "$d" "$out/$t"
done
if cmp -s "$out/rust/start.txt" "$out/pibridge/start.txt"; then
    echo "start lines identical: $(tr '\n' ' ' < "$out/rust/start.txt" | cut -c1-200)"
else
    echo "FAIL start lines differ:"
    diff "$out/rust/start.txt" "$out/pibridge/start.txt"
    fails=$((fails + 1))
fi
[ $fails = 0 ]
