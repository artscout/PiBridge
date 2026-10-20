#!/bin/sh
# SPDX-License-Identifier: MIT
# Parity of pibridge-record with the Rust omarchyvr-record (the oracle) on everything deterministic: argument errors,
# the saved record.toml, the settings errors, the status lines, and for whole recordings (a fake producer in place
# of Monado's tap, tests/fake_ffmpeg.sh in place of ffmpeg) the ffmpeg command lines and the final status.
#   difftest.sh OUTDIR RUST_BIN PIBRIDGE_BIN FAKE_PRODUCER FAKE_FFMPEG
# The Rust texts are normalised for the names only (omarchyvr record -> pibridge-record, the "start a session" hint).
# Exit 77 when the oracle is missing.
set -u
out=$1 rust=$2 pibridge=$3 producer=$4 ffmpeg=$5
if [ ! -x "$rust" ]; then
    echo "skipped: no Rust oracle at $rust"
    exit 77
fi
rm -rf "$out"
mkdir -p "$out"
fails=0 cases=0

norm() {
    sed -e 's/omarchyvr record: /pibridge-record: /' -e 's/omarchyvr-record: /pibridge-record: /' \
        -e 's/(omarchyvr desk start)/("Monado directly" on the Development page, or SteamVR)/' \
        -e 's/omarchyvr record/pibridge-record/g' -e 's#cfg/pibridge/record.toml#cfg/omarchyvr/record.toml#g'
}

# one TAG BIN WORK ARGS...: run with every path in WORK; stdout+stderr (first line of a usage error) and the exit code
one() {
    tag=$1 bin=$2 w=$3
    shift 3
    mkdir -p "$w/session" "$w/run" "$w/cfg" "$w/home" "$w/proc" "$w/pimaxd" "$w/out"
    env -i PATH="$PATH" HOME="$w/home" XDG_RUNTIME_DIR="$w/run" XDG_CONFIG_HOME="$w/cfg" \
        PIMAX_VR_SESSION_DIR="$w/session" PIMAX_PROC_ROOT="$w/proc" PIMAXD_DIR="$w/pimaxd" \
        PIMAX_RECORD_NO_NOTIFY=1 PIMAX_FFMPEG="$ffmpeg" LC_ALL=C ${EXTRA_ENV:-} \
        "$bin" "$@" > "$w/stdout" 2> "$w/stderr"
    echo "exit $?"
    cat "$w/stdout"
    if grep -q '^usage:' "$w/stderr"; then head -n 1 "$w/stderr"; else cat "$w/stderr"; fi
}

compare() {
    name=$1
    cases=$((cases + 1))
    if cmp -s "$out/$name.rust" "$out/$name.pibridge"; then
        echo "ok   $name"
    else
        echo "FAIL $name"
        diff "$out/$name.rust" "$out/$name.pibridge" | head -20
        fails=$((fails + 1))
    fi
}

# case NAME PRE ARGS...: PRE is a shell snippet run in the work dir (as $W) before each run
case_() {
    name=$1 pre=$2
    shift 2
    for t in rust pibridge; do
        w="$out/$name.$t.work"
        rm -rf "$w"
        mkdir -p "$w/cfg/omarchyvr" "$w/cfg/pibridge" "$w/session"
        W=$w sh -c "$pre"
        bin=$rust
        [ $t = pibridge ] && bin=$pibridge
        { one $t "$bin" "$w" "$@" | sed "s#$w#@W@#g"; cat "$w/cfg/pibridge/record.toml" 2>/dev/null || cat "$w/cfg/omarchyvr/record.toml" 2>/dev/null; } |
            norm > "$out/$name.$t"
    done
    compare "$name"
}

# ---- arguments and settings
case_ usage-fps "" start --fps 45
case_ usage-novalue "" start --eyes
case_ usage-unknown "" start --colour red
case_ usage-positional "" start now
case_ usage-stop-opts "" stop --fps 30
case_ usage-command "" record
case_ usage-audio "" start --audio maybe
case_ usage-scale "" start --scale=0.05
case_ usage-height "" start --max-height 100
case_ save-all "" start --eyes both --fps 72 --quality low --scale 0.5 --max-height 1440 --codec hevc --encoder cpu \
    --dir /data/vr --audio off --mic on --save
case_ save-aliases "" start --eyes=sbs --quality med --codec h265 --encoder gpu --scale 1 --save
case_ save-defaults "" start --save
case_ settings-bad-file 'printf "fps = 30\nfsp = 60\n" > $W/cfg/omarchyvr/record.toml' start
case_ settings-bad-value 'printf "eyes = \"three\"\n" > $W/cfg/omarchyvr/record.toml' start
case_ settings-migrate 'printf "quality = \"medium\"\nfps = 60\n" > $W/cfg/omarchyvr/record.toml' start --save
case_ no-session "" start
case_ stop-idle "" stop
case_ brief-stop "" stop --brief

# ---- status lines from a status file
st() { printf '%s\n' "$@" > "$W/session/record.status"; }
case_ status-none "" status
case_ status-stale 'printf "state=recording\nfile=/v/a.mp4\npid=999999\n" > $W/session/record.status' status
case_ status-idle 'printf "state=idle\nfile=/v/pimax-1.mp4\nfps=30\nframes=1234\nnote=Monado stopped (or detached the tap)\n" > $W/session/record.status' status
case_ status-error 'printf "state=error\nnote=cannot run ffmpeg (install ffmpeg)\n" > $W/session/record.status' status
case_ status-stop-error 'printf "state=error\nnote=x\n" > $W/session/record.status' stop

# ---- whole recordings: ffmpeg command lines and the final status
rec() {
    name=$1
    shift
    for t in rust pibridge; do
        w="$out/$name.$t.work"
        rm -rf "$w"
        mkdir -p "$w/session" "$w/run" "$w/cfg" "$w/home" "$w/proc" "$w/pimaxd" "$w/out"
        bin=$rust
        [ $t = pibridge ] && bin=$pibridge
        env -i PATH="$PATH" HOME="$w/home" XDG_RUNTIME_DIR="$w/run" XDG_CONFIG_HOME="$w/cfg" \
            PIMAX_VR_SESSION_DIR="$w/session" PIMAX_PROC_ROOT="$w/proc" PIMAXD_DIR="$w/pimaxd" \
            PIMAX_RECORD_NO_NOTIFY=1 PIMAX_RECORD_NO_MONADO_CHECK=1 PIMAX_FFMPEG="$ffmpeg" \
            PIMAX_RECORD_SHM="$w/run/rec.shm" FAKE_FFMPEG_LOG="$w/ffmpeg-args" LC_ALL=C ${EXTRA_ENV:-} \
            "$bin" run --dir "$w/out" "$@" > "$w/recorder.log" 2>&1 &
        rpid=$!
        "$producer" "$w/run/rec.shm" 15 "$w/published" &
        ppid=$!
        i=0
        while [ ! -e "$w/published" ] && [ $i -lt 200 ]; do sleep 0.05; i=$((i + 1)); done
        sleep 0.5
        kill -INT $rpid 2> /dev/null
        wait $rpid
        code=$?
        wait $ppid
        {
            echo "exit $code"
            sed -e "s#$w#@W@#g" -e 's#pimax-[0-9]\{8\}-[0-9]\{6\}#pimax-STAMP#g' "$w/ffmpeg-args" 2>/dev/null
            grep -E '^(state|eyes|fps|quality|encoder|audio|width|height|note)=' "$w/session/record.status" |
                sed -e "s#$w#@W@#g"
            ls "$w/out" | sed 's#pimax-[0-9]\{8\}-[0-9]\{6\}#pimax-STAMP#'
        } | norm > "$out/$name.$t"
    done
    compare "$name"
}

rec rec-default --max-height 240
rec rec-hevc-missing --max-height 240 --codec hevc 2> /dev/null
EXTRA_ENV="FAKE_FFMPEG_HEVC=1" rec rec-both-hevc-low --max-height 240 --eyes both --codec hevc --quality low --fps 60
EXTRA_ENV="FAKE_FFMPEG_HEVC=1" rec rec-hevc-cpu-high --max-height 240 --codec hevc --encoder cpu
rec rec-cpu-mic --max-height 360 --encoder cpu --mic on --quality medium
rec rec-scale-noaudio --max-height 480 --scale 0.1 --audio off --fps 72
EXTRA_ENV="FAKE_FFMPEG_FAIL_NVENC=1" rec rec-nvenc-fallback --max-height 240 --audio off
EXTRA_ENV="FAKE_FFMPEG_PULSE_EXIT=1" rec rec-no-sound --max-height 240 --mic on

echo "$cases cases, $fails failed"
[ $fails -eq 0 ]
