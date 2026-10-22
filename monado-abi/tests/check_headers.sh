#!/bin/sh
# SPDX-License-Identifier: MIT
# The headers in monado-abi/include are the C ABI the Monado Pimax driver compiles against (spec.txt names their
# origin); they change only on purpose: checked against include/SHA256SUMS.
#   check_headers.sh INCLUDE_DIR
set -u
cd "$1" && sha256sum --quiet -c SHA256SUMS && echo "headers match SHA256SUMS" && exit 0
exit 1
