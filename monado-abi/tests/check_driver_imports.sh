#!/bin/sh
# SPDX-License-Identifier: MIT
# Every pimax_* function of the ABI headers that the Monado Pimax driver objects import is defined (T) in the archive
# (the Monado tree: third_party/monado, built in PIBRIDGE_MONADO_DRIVER_BUILD_DIR). Read only.
#   check_driver_imports.sh MONADO_BUILD_DIR ARCHIVE INCLUDE_DIR
# Skipped (77) without a Monado build.
set -u
objs=$1/src/xrt/drivers/CMakeFiles/drv_pimax.dir/pimax
ar=$2
inc=$3
if ! ls "$objs"/*.o > /dev/null 2>&1; then
    echo "skipped: no Monado Pimax driver objects in $objs"
    exit 77
fi
export LC_ALL=C
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
cat "$inc"/*.h | grep -o 'pimax_[a-z0-9_]*(' | tr -d '(' | sort -u > "$tmp/declared"
nm -u "$objs"/*.o | awk '{print $NF}' | grep '^pimax_' | sort -u > "$tmp/imported"
comm -12 "$tmp/declared" "$tmp/imported" > "$tmp/used"
nm -g --defined-only "$ar" | awk '$2 == "T" {print $3}' | sort -u > "$tmp/exported"
missing=$(comm -23 "$tmp/used" "$tmp/exported")
echo "the driver imports $(wc -l < "$tmp/used") functions of the ABI headers"
if [ -n "$missing" ]; then
    echo "MISSING from the archive:"
    echo "$missing"
    exit 1
fi
echo "all defined in $(basename "$ar")"
