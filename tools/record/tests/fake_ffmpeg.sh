#!/bin/sh
# SPDX-License-Identifier: MIT
# A fake ffmpeg for pibridge-record's tests: never encodes, never opens the sound server.
#   -encoders                    prints $FAKE_FFMPEG_ENCODERS (default: h264_nvenc and libx264; FAKE_FFMPEG_HEVC=1 adds
#                                hevc_nvenc and libx265)
#   -f pulse ... -f null -       the sound probe: exit $FAKE_FFMPEG_PULSE_EXIT (default 0)
#   anything else (an encode)    appends its argv (one line) to $FAKE_FFMPEG_LOG and copies stdin into the last
#                                argument (the output file); with FAKE_FFMPEG_FAIL_NVENC=1 a *_nvenc encode exits 1 at once
for a in "$@"; do
    if [ "$a" = "-encoders" ]; then
        printf '%s\n' "${FAKE_FFMPEG_ENCODERS:- V....D h264_nvenc           NVIDIA NVENC H.264 encoder (codec h264)
 V....D libx264              libx264 H.264 / AVC}"
        [ "${FAKE_FFMPEG_HEVC:-0}" = 1 ] && printf '%s\n' " V....D hevc_nvenc           NVIDIA NVENC hevc encoder" \
            " V....D libx265              libx265 H.265 / HEVC"
        exit 0
    fi
done
case " $* " in
*" -f pulse "*" -f null "*)
    code=${FAKE_FFMPEG_PULSE_EXIT:-0}
    [ "$code" = 0 ] || echo "[in#0] Error opening input: Connection refused" >&2
    exit "$code" ;;
esac
[ -n "${FAKE_FFMPEG_LOG:-}" ] && echo "$*" >> "$FAKE_FFMPEG_LOG"
if [ "${FAKE_FFMPEG_FAIL_NVENC:-0}" = 1 ]; then
    case " $* " in *_nvenc*) echo "Cannot load libnvidia-encode.so.1" >&2; exit 1 ;; esac
fi
for last in "$@"; do :; done
exec cat > "$last"
