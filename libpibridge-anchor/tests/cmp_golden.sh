#!/bin/sh
# SPDX-License-Identifier: MIT
# Run a parity probe and compare its output byte for byte with the oracle's golden output (gzip).
#   cmp_golden.sh PROBE GOLDEN.gz [ARGS...]
set -u
probe=$1 golden=$2
shift 2
out=${TMPDIR:-/tmp}/$(basename "$golden" .gz).$$
"$probe" "$@" > "$out.pibridge" || { echo "probe failed"; exit 1; }
gzip -dc "$golden" > "$out.rust"
if cmp -s "$out.rust" "$out.pibridge"; then
    echo "identical: $(wc -l < "$out.rust") lines"
    rm -f "$out.rust" "$out.pibridge"
    exit 0
fi
echo "DIFFERENT: $out.rust vs $out.pibridge"
diff "$out.rust" "$out.pibridge" | head -20
exit 1
