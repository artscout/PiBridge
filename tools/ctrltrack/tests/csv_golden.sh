#!/bin/sh
# SPDX-License-Identifier: MIT
# `track PXCAM --calib XML --csv` (scratch HOME) must reproduce the recorded reference CSV byte for byte.
#   csv_golden.sh OUTDIR PIBRIDGE_CTRLTRACK PXCAM CALIB_XML GOLDEN_CSV
out=$1 exe=$2 px=$3 xml=$4 golden=$5
for f in "$px" "$xml" "$golden"; do
    [ -f "$f" ] || { echo "skipped: $f not present (lab data)"; exit 77; }
done
rm -rf "$out"
mkdir -p "$out/home"
HOME=$out/home "$exe" track "$px" --calib "$xml" --csv "$out/out.csv" > "$out/out.txt" 2>&1 || { tail -3 "$out/out.txt"; exit 1; }
if cmp -s "$out/out.csv" "$golden"; then
    echo "identical to $golden: $(wc -l < "$golden") lines, $(wc -c < "$golden") bytes"
    exit 0
fi
echo "DIFFERENT: $out/out.csv vs $golden"
diff "$out/out.csv" "$golden" | head -10
exit 1
