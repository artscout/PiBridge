#!/bin/sh
# build-winbridge.sh: build winbridge from the sources next to this script.
#
# This script ships IN the winbridge sources dir (not on $PATH); run it from there:
#   sh /usr/share/pibridge/winbridge-src/build-winbridge.sh
#
# winbridge is OUR code (MIT). It is the inside-out (Pimax-engine) tracking path:
# Monado's Pimax driver --VIT--> libpibridge_vit_winbridge.so --TCP--> winbridge.exe
# (Wine) --> pimax_slam.pi.dll. The proprietary Pimax DLLs it loads are NOT shipped
# (personal-use kit, see docs/INSTALL.md); only our sources are.
#
# Needs: mingw-w64 (x86_64-w64-mingw32-g++) to build winbridge.exe, and g++ for the
# VIT bridge library. Wine is needed only at run time. This step is OPTIONAL: it is
# only for the inside-out / Pimax-engine tracking path.
set -e

# the sources are next to this script
SRC="$(cd "$(dirname "$0")" && pwd)"
OUT="${XDG_DATA_HOME:-$HOME/.local/share}/pibridge/winbridge"

if [ ! -f "$SRC/server.cpp" ]; then
    echo "winbridge sources not found at $SRC" >&2
    exit 1
fi
if ! command -v x86_64-w64-mingw32-g++ >/dev/null 2>&1; then
    echo "x86_64-w64-mingw32-g++ not found: install mingw-w64" >&2
    exit 1
fi

BUILD="$(mktemp -d)"
trap 'rm -rf "$BUILD"' EXIT
cp -a "$SRC"/. "$BUILD"/
cd "$BUILD"

echo "Building winbridge.exe (MinGW, static) ..."
x86_64-w64-mingw32-g++ -O2 -static server.cpp -lws2_32 -o winbridge.exe

echo "Building libpibridge_vit_winbridge.so ..."
g++ -O2 -std=c++20 -fPIC -shared vit_winbridge.cpp -o libpibridge_vit_winbridge.so -pthread

mkdir -p "$OUT"
cp -f winbridge.exe libpibridge_vit_winbridge.so "$OUT"/
echo "winbridge built and installed to $OUT"
