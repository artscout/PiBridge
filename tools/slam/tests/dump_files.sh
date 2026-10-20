#!/bin/sh
# SPDX-License-Identifier: MIT
# Print every file under DIR (sorted, recursively) with its name, for the export difftests (run_difftest.sh --post).
dir=$1
[ -d "$dir" ] || { echo "(no $dir)"; exit 0; }
find "$dir" -type f | LC_ALL=C sort | while IFS= read -r f; do
    echo "== ${f#"$dir"/}"
    cat "$f"
    echo "<<end"
done
