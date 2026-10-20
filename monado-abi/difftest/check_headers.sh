#!/bin/sh
# SPDX-License-Identifier: MIT
# The headers in monado-abi/include are verbatim copies of omarchyvr's at the pinned commit (spec.txt).
#   check_headers.sh INCLUDE_DIR            checksums against include/SHA256SUMS
#   check_headers.sh INCLUDE_DIR OMARCHYVR  each header against `git show <pin>:<path>` in the omarchyvr checkout;
#                                           exit 77 (skipped) without the checkout, and a note when the working tree
#                                           has moved past the pin (the uncommitted AE work, for example)
set -u
inc=$1
spec="$inc/../spec.txt"
if [ $# -lt 2 ]; then
    cd "$inc" && sha256sum --quiet -c SHA256SUMS && echo "headers match SHA256SUMS" && exit 0
    exit 1
fi
repo=$2
if [ ! -d "$repo/.git" ] && [ ! -f "$repo/.git" ]; then
    echo "skipped: no omarchyvr checkout at $repo"
    exit 77
fi
pin=$(awk '$1 == "commit" { print $2 }' "$spec")
rc=0
moved=""
while read -r file path; do
    case "$file" in ''|'#'*|commit) continue ;; esac
    if ! git -C "$repo" show "$pin:$path" | cmp -s - "$inc/$file"; then
        echo "DIFFERENT from $pin:$path: $file"
        rc=1
    fi
    if [ -f "$repo/$path" ] && ! cmp -s "$repo/$path" "$inc/$file"; then
        moved="$moved $path"
    fi
done < "$spec"
[ $rc = 0 ] && echo "all headers identical to omarchyvr $pin"
[ -n "$moved" ] && echo "note: the omarchyvr working tree has moved past the pin for:$moved (re-pin before porting that part)"
exit $rc
