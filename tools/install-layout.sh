#!/usr/bin/env bash
# install-layout.sh: populate ONE install prefix with everything PiBridge needs, so the app finds its engine and data
# under <prefix>/{bin,lib/pibridge,share/pibridge,etc/pibridge} (app/src/core/paths.h). The live user settings stay in
# XDG (~/.config/pibridge) and are NEVER touched here.
#
# Run it BETWEEN VR sessions only. It refuses while a session is live, and it never edits SteamVR's openvrpaths.vrpath:
# it prints the external_drivers line for the user/parent to add. Idempotent: safe to re-run.
#
# Usage: tools/install-layout.sh [PREFIX]   (PREFIX also from $PIBRIDGE_PREFIX; default ~/.local)
#
# NON-REDISTRIBUTABLE -- personal use only, copied from the user's own machine, never committed or shared:
#   * the Mercury hand models (no licence),
#   * winbridge.exe (the Pimax SLAM engine) and libpibridge_vit_winbridge.so,
#   * the camera device_calibration_*.xml, and the Pimax kit (windata: dlls + slam vocabulary/calibration).
set -euo pipefail

# ---------------------------------------------------------------- a. refuse while VR is live
if pgrep -x vrserver >/dev/null || pgrep -x monado-service >/dev/null; then
    echo "error: a VR session is running (vrserver/monado-service). Close it first; install only between sessions." >&2
    exit 1
fi

# ---------------------------------------------------------------- b. prefix
PREFIX="${1:-${PIBRIDGE_PREFIX:-$HOME/.local}}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)" # the pibridge checkout (tools/..)
BUILD_DIR="$ROOT/build"
MONADO_BUILD="$HOME/gamedev/pibridge-monado-build"
MONADO_PREFIX="$PREFIX/lib/pibridge/monado"
LIBDIR="$PREFIX/lib/pibridge"
DATADIR="$PREFIX/share/pibridge"
ETCDIR="$PREFIX/etc/pibridge"
echo ">> install prefix: $PREFIX"

# ---------------------------------------------------------------- c. build + install the PiBridge workspace
# configure (idempotent) and build the whole workspace, then install the binaries, repo data, record layer, units,
# desktop entry, icons, docs and udev rule under $PREFIX.
echo ">> building the PiBridge workspace"
nice -n 19 cmake -S "$ROOT" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX"
nice -n 19 cmake --build "$BUILD_DIR" -j8
echo ">> installing the PiBridge workspace into $PREFIX"
cmake --install "$BUILD_DIR" --prefix "$PREFIX"

# ---------------------------------------------------------------- d. Monado engine
# The Monado fork is built out of tree (pibridge-monado-build). Install it under $PREFIX/lib/pibridge/monado only when
# that build is configured; this is NOT a rebuild (no relink, so cap_sys_nice is untouched). monado-service lands at
# $MONADO_PREFIX/bin and the SteamVR driver tree at $MONADO_PREFIX/<libdir>/pibridge/steamvr-monado (steamvr_drv's
# install rule, CMAKE_INSTALL_LIBDIR). We then point the canonical $LIBDIR/steamvr-monado at it.
if [ -f "$MONADO_BUILD/CMakeCache.txt" ]; then
    echo ">> installing Monado into $MONADO_PREFIX"
    cmake --install "$MONADO_BUILD" --prefix "$MONADO_PREFIX"
    # the libdir varies by distro (lib, lib64, lib/x86_64-linux-gnu): find the tree that was actually installed.
    svr="$(find "$MONADO_PREFIX" -type d -name steamvr-monado -print -quit 2>/dev/null || true)"
    canon="$LIBDIR/steamvr-monado"
    if [ -n "$svr" ]; then
        if [ "$svr" != "$canon" ]; then
            mkdir -p "$LIBDIR"
            rm -rf "$canon"
            # a relative symlink keeps the prefix relocatable; fall back to a copy if symlinks are unavailable.
            if ln -s "$(realpath --relative-to="$LIBDIR" "$svr")" "$canon" 2>/dev/null; then
                echo ">> linked $canon -> $svr"
            else
                cp -a "$svr" "$canon"
                echo ">> copied $svr -> $canon"
            fi
        fi
    else
        echo "warning: no steamvr-monado tree under $MONADO_PREFIX (is XRT_FEATURE_STEAMVR_PLUGIN on?)." >&2
    fi
else
    echo "warning: $MONADO_BUILD is not configured; skipping the Monado install." >&2
fi

# ---------------------------------------------------------------- e. external data (copied from the user's machine)
# Only copied when the source exists. These are the files the app looks up under $DATADIR / $LIBDIR first
# (app/src/core/rec_setup.cpp resolveRecSetup).
copy_glob() { # copy_glob <dest-dir> <src-glob...>
    local dest="$1"; shift
    local src any=0
    for src in "$@"; do
        [ -e "$src" ] || continue
        mkdir -p "$dest"
        cp -a "$src" "$dest/"
        any=1
    done
    [ "$any" = 1 ] && echo ">> data -> $dest"
    return 0
}
copy_glob "$DATADIR/lens"  "$HOME"/gamedev/ext/lens/*.conf "$HOME"/gamedev/ext/lens/*lut*.bin
copy_glob "$DATADIR/calib" "$HOME"/gamedev/ext/calib/*.xml
copy_glob "$DATADIR/slam"  "$HOME"/gamedev/ext/slam/pimax-basalt.toml
# Basalt live 6DoF library (optional).
copy_glob "$LIBDIR" "$HOME"/gamedev/ovr-build/basalt/lib/libbasalt.so
# NON-REDISTRIBUTABLE: the Pimax SLAM engine under Wine and Monado's VIT library for it (personal use only).
copy_glob "$LIBDIR/winbridge" \
    "$HOME"/gamedev/pibridge/winbridge/winbridge.exe \
    "$HOME"/gamedev/pibridge/winbridge/libpibridge_vit_winbridge.so

# ---------------------------------------------------------------- g. default settings seed (never overwrite the user's)
# The read-only template Conf::reload() falls back to when neither ~/.config/pibridge/pibridge.conf nor the legacy
# file exists (app/src/core/conf.cpp). Fully commented, so it imposes nothing: it only documents the keys.
if [ ! -e "$ETCDIR/pibridge.conf" ]; then
    mkdir -p "$ETCDIR"
    cat > "$ETCDIR/pibridge.conf" <<'CONF'
# pibridge default settings (docs/CONVENTIONS.md). Shipped read-only seed: the live file is
# ~/.config/pibridge/pibridge.conf, which the app writes. Every line here is a commented default; uncomment and change
# in the LIVE file (through the app), not here.
#
# refresh_hz=90          # 90 or 72
# eye_tracking=on
# audio_to_headset=on
# tracking_mode=inside-out
#
# Record-tracking paths (left blank = auto-detect under the install prefix, then the dev tree):
# rec_monado_service=
# rec_pimax_cam=
# rec_pimax_ctrltrack=
# rec_basalt_lib=
# rec_slam_config=
# rec_lens_conf=
# rec_cam_calib=
# rec_connector=          # e.g. DP-6
# rec_cam_device=         # e.g. /dev/video10
CONF
    echo ">> wrote default seed $ETCDIR/pibridge.conf"
else
    echo ">> default seed already present: $ETCDIR/pibridge.conf (left as is)"
fi

# ---------------------------------------------------------------- done: what the parent/user must register by hand
cat <<DONE

Install layout populated under $PREFIX.

The script does NOT edit SteamVR's ~/.config/openvr/openvrpaths.vrpath. Add the Monado SteamVR driver to its
"external_drivers" list by hand (between sessions):

    $LIBDIR/steamvr-monado

Non-redistributable files (hand models, winbridge.exe, device_calibration XML, the Pimax windata kit) are copied from
your existing locations only; they are never committed or redistributed.
DONE
