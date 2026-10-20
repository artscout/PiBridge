#!/bin/sh
# SPDX-License-Identifier: MIT
# Run one command twice, with the Rust oracle and with PiBridge's build, and compare the standard output byte for byte.
#
#   run_difftest.sh NAME OUTDIR RUST_EXE PIBRIDGE_EXE [--need FILE]... [--home] [--pre CMD] [--post CMD] [--exit N]
#                   [--ignore REGEX] [--env K=V]... [--hash] -- ARGS...
#
# @W@ in ARGS and CMD is replaced by a fresh work directory (the same path for both runs, emptied in between).
# --need FILE: skip (exit 77) when FILE is missing (private lab data). --home: HOME=@W@/home (empty) for the run.
# --post CMD: run CMD after each run and append its output (e.g. a dump of a file the command wrote).
# --pre CMD: run CMD in the fresh work directory before each run (e.g. to write an input file).
# --exit N: the exit status the Rust run must have (default 0), so two runs that fail alike do not pass.
# --ignore REGEX: drop matching lines before comparing (values that depend on the absolute clock of the run, which
#   differs between any two runs, Rust against Rust included).
# --env K=V: set an environment variable for both runs (@W@ is replaced in V).
# --hash: compare (and show) only a SHA-256 and the size of each output, for outputs that carry private lab values
#   (a headset's own calibration); the full outputs stay in OUTDIR as NAME.TAG.raw for a local cmp.
# Exit 77 also when RUST_EXE does not exist (no oracle built: monado-abi/difftest/build-oracle.sh).
set -u
name=$1 out=$2 rust=$3 pibridge=$4
shift 4
home=0
post=""
pre=""
ignore=""
want_exit=0
envs=""
hash=0
while [ $# -gt 0 ] && [ "$1" != "--" ]; do
    case "$1" in
    --need)
        if [ ! -e "$2" ]; then
            echo "skipped: $2 not present (lab data)"
            exit 77
        fi
        shift 2 ;;
    --home) home=1; shift ;;
    --post) post=$2; shift 2 ;;
    --exit) want_exit=$2; shift 2 ;;
    --pre) pre=$2; shift 2 ;;
    --ignore) ignore=$2; shift 2 ;;
    --env) envs="$envs$2
"; shift 2 ;;
    --hash) hash=1; shift ;;
    *) echo "bad option $1"; exit 2 ;;
    esac
done
[ "${1:-}" = "--" ] && shift
if [ ! -x "$rust" ]; then
    echo "skipped: no Rust oracle at $rust (run monado-abi/difftest/build-oracle.sh)"
    exit 77
fi
mkdir -p "$out"
work="$out/$name.work"

one() {
    exe=$1 tag=$2
    rm -rf "$work"
    mkdir -p "$work/home"
    if [ -n "$pre" ]; then
        sh -c "$(printf '%s' "$pre" | sed "s|@W@|$work|g")" > /dev/null 2>&1
    fi
    # rebuild the argument list with @W@ replaced (arguments contain no newlines)
    set --
    if [ -n "$ARGLIST" ]; then
        printf '%s\n' "$ARGLIST" | sed "s|@W@|$work|g" > "$out/$name.args"
        while IFS= read -r a; do
            set -- "$@" "$a"
        done < "$out/$name.args"
    fi
    # the environment: --home and --env assignments (with @W@ replaced), through env(1)
    envset=""
    if [ $home = 1 ]; then
        envset="HOME=$work/home
"
    fi
    if [ -n "$envs" ]; then
        envset="$envset$(printf '%s' "$envs" | sed "s|@W@|$work|g")
"
    fi
    if [ -n "$envset" ]; then
        printf '%s' "$envset" | sed '/^$/d' > "$out/$name.env"
        set -- "$exe" "$@"
        while IFS= read -r kv; do
            set -- "$kv" "$@"
        done < "$out/$name.env"
        env "$@" > "$out/$name.$tag.txt" 2> "$out/$name.$tag.err"
    else
        "$exe" "$@" > "$out/$name.$tag.txt" 2> "$out/$name.$tag.err"
    fi
    status=$?
    echo "exit $status" >> "$out/$name.$tag.txt"
    eval "status_$tag=$status"
    if [ -n "$post" ]; then
        sh -c "$(printf '%s' "$post" | sed "s|@W@|$work|g")" >> "$out/$name.$tag.txt" 2>&1
    fi
}

ARGLIST=$(for a in "$@"; do printf '%s\n' "$a"; done)
one "$rust" rust
one "$pibridge" pibridge
if [ $hash = 1 ]; then
    for t in rust pibridge; do
        mv "$out/$name.$t.txt" "$out/$name.$t.raw"
        printf '%s %s bytes, %s lines\n' "$(sha256sum < "$out/$name.$t.raw" | cut -c1-64)" \
            "$(wc -c < "$out/$name.$t.raw")" "$(wc -l < "$out/$name.$t.raw")" > "$out/$name.$t.txt"
    done
fi
rm -rf "$work"
if [ "$status_rust" != "$want_exit" ]; then
    echo "the Rust run exited $status_rust, expected $want_exit:"
    tail -5 "$out/$name.rust.err" "$out/$name.rust.txt"
    exit 1
fi
if [ -n "$ignore" ]; then
    for t in rust pibridge; do
        grep -v -E "$ignore" "$out/$name.$t.txt" > "$out/$name.$t.cmp" || true
        mv "$out/$name.$t.cmp" "$out/$name.$t.txt"
    done
fi
if cmp -s "$out/$name.rust.txt" "$out/$name.pibridge.txt"; then
    echo "identical: $(wc -l < "$out/$name.rust.txt") lines, $(wc -c < "$out/$name.rust.txt") bytes ($out/$name.rust.txt)"
    exit 0
fi
echo "DIFFERENT: $out/$name.rust.txt vs $out/$name.pibridge.txt"
diff "$out/$name.rust.txt" "$out/$name.pibridge.txt" | head -40
exit 1
