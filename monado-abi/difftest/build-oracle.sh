#!/bin/sh
# SPDX-License-Identifier: MIT
# Build the Rust oracle (libpimax_cam.a, pimax-cam, libpimax_lens.a, pimax-lens, omarchyvr-slam, libpimax_hands.a, pimax-ctrltrack) from the pinned omarchyvr commit (spec.txt), without touching
# the omarchyvr checkout: the commit is exported with `git archive` into a scratch folder and built there.
#   build-oracle.sh OMARCHYVR_DIR CARGO_TARGET_DIR [SCRATCH]
# Default target in the CMake cache: <omarchyvr>/rust/target-port (PIBRIDGE_RUST_ORACLE_DIR = .../release).
# CPU only, nice -n 19, -j8, offline (the crates must already be in ~/.cargo).
set -eu
repo=$1
target=$2
scratch=${3:-$(mktemp -d)}
here=$(cd "$(dirname "$0")" && pwd)
pin=$(awk '$1 == "commit" { print $2 }' "$here/../spec.txt")
rm -rf "$scratch/oracle-src"
mkdir -p "$scratch/oracle-src"
git -C "$repo" archive "$pin" rust | tar -x -C "$scratch/oracle-src"
cd "$scratch/oracle-src/rust"
command -v cargo > /dev/null || PATH=$HOME/.cargo/bin:$PATH
CARGO_TARGET_DIR=$target nice -n 19 cargo build --release --offline --locked -j8 -p pimax-cam -p pimax-lens -p omarchyvr-slam -p pimax-hands -p pimax-ctrltrack -p pimax-capi
echo "oracle: $target/release/{libpimax_cam.a,pimax-cam,libpimax_lens.a,pimax-lens,omarchyvr-slam,libpimax_hands.a,pimax-ctrltrack,libpimax_capi.a} from omarchyvr $pin"
