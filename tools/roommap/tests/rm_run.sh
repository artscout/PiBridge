#!/bin/sh
# SPDX-License-Identifier: MIT
# Run one omarchyvr-anchor / -room / -roommap or pibridge-* command with stderr merged into stdout and mask what
# differs between any two runs of the same tool: wall-clock durations, the age of the newest ring sample, the tool's
# own name in error lines (omarchyvr-X / pibridge-X), the panicking thread's id and location.
#   rm_run.sh EXE ARGS...
exe=$1
shift
tmp=$(mktemp)
"$exe" "$@" > "$tmp" 2>&1
st=$?
sed -E -e "s/^thread 'main' (\([0-9]+\) )?panicked at .*:\$/thread 'main' panicked at X:/" \
    -e 's/newest sample [0-9.]+ s ago/newest sample X s ago/' \
    -e 's/\(([0-9]+\.[0-9]) s\)$/(X s)/' -e 's/ in [0-9]+\.[0-9] s$/ in X s/' \
    -e 's/^(omarchyvr|pibridge)-(anchor|room|roommap)/TOOL-\2/' "$tmp"
rm -f "$tmp"
exit $st
