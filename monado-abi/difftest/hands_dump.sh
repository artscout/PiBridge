#!/bin/sh
# SPDX-License-Identifier: MIT
# Print the files the controller fuser wrote in a difftest work directory (hands.cmake --post): the calibration
# (default path in the scratch HOME, or the PIMAX_CTRL_IMU_CALIB copy) and the IMU logs.
w=$1
for f in "$w/home/.config/omarchyvr/ctrl-imu.json" "$w/calib.json" "$w/log-left.csv" "$w/log-right.csv"; do
    echo "== ${f#"$w"/}"
    if [ -f "$f" ]; then
        cat "$f"
    else
        echo none
    fi
done
ls -a "$w/home/.config/omarchyvr" 2>/dev/null
