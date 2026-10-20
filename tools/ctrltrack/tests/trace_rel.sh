#!/bin/sh
# SPDX-License-Identifier: MIT
# Print a `live --trace` file with the frame times relative to the first line (a replay rebases the recorded times
# onto the clock at its start, so only the differences are the same between two runs).
[ -f "$1" ] || { echo "no trace $1"; exit 0; }
awk -F, 'NR == 1 { b = $1 } { printf "%.0f", $1 - b; for (i = 2; i <= NF; i++) printf ",%s", $i; print "" }' "$1"
