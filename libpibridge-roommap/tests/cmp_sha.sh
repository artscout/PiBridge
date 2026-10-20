#!/bin/sh
# SPDX-License-Identifier: MIT
# Run a parity probe and compare the SHA-256 and line count of its output with the oracle's (GOLDEN.sha256: "<sha256>
# <lines>"), for outputs too large to keep in the tree; a mismatch keeps the output for a diff against the oracle's.
#   cmp_sha.sh PROBE GOLDEN.sha256 [ARGS...]
set -u
probe=$1 golden=$2
shift 2
out=${TMPDIR:-/tmp}/$(basename "$golden" .sha256).pibridge.txt
"$probe" "$@" > "$out" || { echo "probe failed"; exit 1; }
got="$(sha256sum < "$out" | cut -c1-64) $(wc -l < "$out")"
want=$(cat "$golden")
if [ "$got" = "$want" ]; then
    echo "identical: $got"
    rm -f "$out"
    exit 0
fi
echo "DIFFERENT: $got (oracle $want); output kept in $out"
exit 1
