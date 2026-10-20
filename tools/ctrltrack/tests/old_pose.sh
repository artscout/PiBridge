#!/bin/sh
# SPDX-License-Identifier: MIT
# Write a 256-byte pose file as an older writer leaves it: magic, version 1, seq 0, pid 7, no poses, no heartbeat.
{ printf 'PXCT\001\000\000\000\000\000\000\000\007\000\000\000'; head -c 240 /dev/zero; } > "$1"
