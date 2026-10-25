#!/bin/sh
# SPDX-License-Identifier: MIT
# A shared object that links the whole libpibridge_pimax.a must export only the C ABI (pimax_*): everything else is
# hidden, so the archive cannot clash with Monado's own C++ (Mercury, OpenCV).
#   check_symbols.sh SHARED_OBJECT
set -u
so=$1
all=$(nm -D --defined-only "$so" | awk '{ print $3 }' | sort)
bad=$(printf '%s\n' "$all" | grep -v '^pimax_' | grep -v '^$' || true)
n=$(printf '%s\n' "$all" | grep -c '^pimax_')
if [ -n "$bad" ]; then
    echo "exported symbols that are not pimax_*:"
    printf '%s\n' "$bad" | head -40
    exit 1
fi
echo "only the C ABI is exported: $n pimax_* symbols"
printf '%s\n' "$all"
