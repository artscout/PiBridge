#!/bin/sh
# SPDX-License-Identifier: MIT
# Dump the files of a room-map directory for a difftest, wall-clock fields masked (roommap.toml created, floor.toml
# when, the map.json/map.html created), map.bin by hash.
#   dump_map.sh DIR
d=$1
for f in roommap.toml correction.csv floor.toml preview.txt room.json last-reloc.toml map.json; do
    if [ -f "$d/$f" ]; then
        echo "== $f"
        sed -E 's/unix [0-9]+/unix T/; s/^when = [0-9]+/when = T/' "$d/$f"
    fi
done
if [ -f "$d/map.html" ]; then
    echo "== map.html"
    sed -E 's/unix [0-9]+/unix T/' "$d/map.html" | sha256sum
fi
if [ -f "$d/map.bin" ]; then
    echo "== map.bin"
    sha256sum < "$d/map.bin"
fi
true
