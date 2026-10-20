#!/bin/sh
# SPDX-License-Identifier: MIT
# Broker cross tests (docs/research/rust-port-plan.md M2): PXCB v1 in both directions, fed from a recorded .pxcam.
#
#   broker_cross.sh OUTDIR RUST_PIMAX_CAM PIBRIDGE_CAM RUST_LIB PIBRIDGE_DIFFTEST RUST_DIFFTEST PXCAM
#
# For every pair (server x client) of {Rust pimax-cam broker, pibridge-cam broker} x {C client on libpimax_cam.a, C
# client on libpibridge_pimax.a}: every frame the client gets is byte-identical (tiles, metadata) to a direct replay of the
# file and keeps the recorded spacing. Also: the raw hello and ring header of both servers are the same bytes, a slow
# client skips frames without slowing the server, a second broker on a live socket is refused (both ways), a stale
# socket file is replaced, and broker-status of each tool reads the other's broker. Sockets live in OUTDIR; the
# brokers are this script's own children and are stopped by PID. Exit 77 without the oracle or the recording.
set -u
out=$1 rust_cam=$2 pibridge_cam=$3 rust_lib=$4 pibridge_diff=$5 rust_diff=$6 pxcam=$7
for f in "$rust_cam" "$rust_diff"; do
    if [ ! -x "$f" ]; then
        echo "skipped: no Rust oracle ($f; monado-abi/difftest/build-oracle.sh)"
        exit 77
    fi
done
if [ ! -f "$pxcam" ]; then
    echo "skipped: $pxcam not present (lab data)"
    exit 77
fi
: "$rust_lib"
rm -rf "$out"
mkdir -p "$out"
fails=0
ok() { echo "ok    $*"; }
bad() { echo "FAIL  $*"; fails=$((fails + 1)); }

wait_socket() { # $1 = path
    i=0
    while [ ! -S "$1" ] && [ $i -lt 500 ]; do
        sleep 0.01
        i=$((i + 1))
    done
    [ -S "$1" ]
}

tool() { [ "$1" = rust ] && echo "$rust_cam" || echo "$pibridge_cam"; }
diffexe() { [ "$1" = rust ] && echo "$rust_diff" || echo "$pibridge_diff"; }

# 1. every server with every client
for s in rust pibridge; do
    for c in rust pibridge; do
        sock="$out/$s-$c.sock"
        "$(tool $s)" broker --source "$pxcam" --socket "$sock" --slots 8 2> "$out/$s-$c.broker.log" &
        pid=$!
        if ! wait_socket "$sock"; then
            bad "$s broker did not open $sock"
            kill "$pid" 2> /dev/null
            continue
        fi
        if "$(diffexe $c)" client "$sock" "$pxcam" > "$out/$s-$c.client.txt" 2> "$out/$s-$c.client.err"; then
            ok "$s broker -> $c client: $(tail -1 "$out/$s-$c.client.err")"
        else
            bad "$s broker -> $c client: $(cat "$out/$s-$c.client.txt" "$out/$s-$c.client.err" | tail -3)"
        fi
        wait "$pid"
        rc=$?
        [ $rc = 0 ] || bad "$s broker exited $rc"
        [ -e "$sock" ] && bad "$s broker left its socket file"
    done
done

# 2. the raw hello and ring header (no library on the client side)
for s in rust pibridge; do
    sock="$out/hello-$s.sock"
    "$(tool $s)" broker --source "$pxcam" --socket "$sock" --slots 8 --loop 2> "$out/hello-$s.broker.log" &
    pid=$!
    wait_socket "$sock"
    "$pibridge_diff" hello "$sock" | sed "s|$sock|SOCKET|g" > "$out/hello-$s.txt"
    kill "$pid"
    wait "$pid" 2> /dev/null
done
if cmp -s "$out/hello-rust.txt" "$out/hello-pibridge.txt"; then
    ok "hello and ring header identical: $(head -2 "$out/hello-rust.txt" | tr '\n' ' ')"
else
    bad "hello differs:"
    diff "$out/hello-rust.txt" "$out/hello-pibridge.txt"
fi

# 3. a slow client skips frames, in order, and never slows the server down
for pair in "rust pibridge" "pibridge rust"; do
    set -- $pair
    sock="$out/slow-$1.sock"
    "$(tool "$1")" broker --source "$pxcam" --socket "$sock" --slots 4 2> "$out/slow-$1.broker.log" &
    pid=$!
    wait_socket "$sock"
    t0=$(date +%s%N)
    if "$(diffexe "$2")" client "$sock" "$pxcam" 40 > "$out/slow-$1.client.txt" 2> "$out/slow-$1.client.err"; then
        ok "slow $2 client on the $1 broker: $(tail -1 "$out/slow-$1.client.err")"
    else
        bad "slow $2 client on the $1 broker: $(tail -2 "$out/slow-$1.client.txt")"
    fi
    wait "$pid"
    t1=$(date +%s%N)
    # the recording lasts 2.4 s at its own pace; a server held up by the client would take ~6 s
    ms=$(((t1 - t0) / 1000000))
    if [ $ms -lt 5000 ]; then ok "the $1 broker kept its pace with a slow client ($ms ms)"; else bad "$1 broker slowed down: $ms ms"; fi
done

# 4. a second broker on a live socket is refused, both ways; broker-status of each tool reads the other's broker
for pair in "rust pibridge" "pibridge rust"; do
    set -- $pair
    sock="$out/dup-$1.sock"
    "$(tool "$1")" broker --source "$pxcam" --socket "$sock" --loop 2> "$out/dup-$1.broker.log" &
    pid=$!
    wait_socket "$sock"
    if "$(tool "$2")" broker --source "$pxcam" --socket "$sock" 2> "$out/dup-$2-second.log"; then
        bad "a $2 broker started on the live socket of a $1 broker"
    elif grep -q "a camera broker already serves" "$out/dup-$2-second.log"; then
        ok "a $2 broker refuses the live socket of a $1 broker"
    else
        bad "$2 broker failed for another reason: $(cat "$out/dup-$2-second.log")"
    fi
    if "$(tool "$2")" broker-status --socket "$sock" --json --window 0.3 > "$out/status-$2-on-$1.json"; then
        if grep -q '"camera_up":true' "$out/status-$2-on-$1.json"; then
            ok "$2 broker-status on the $1 broker: $(cat "$out/status-$2-on-$1.json" | sed "s|$out|OUT|g")"
        else
            bad "$2 broker-status: $(cat "$out/status-$2-on-$1.json")"
        fi
    else
        bad "$2 broker-status on the $1 broker exited non-zero"
    fi
    kill "$pid"
    wait "$pid" 2> /dev/null
done

# 5. no broker: both tools say the same
for t in rust pibridge; do
    "$(tool $t)" broker-status --socket "$out/none.sock" --json > "$out/status-none-$t.json"
    echo "exit $?" >> "$out/status-none-$t.json"
done
if cmp -s "$out/status-none-rust.json" "$out/status-none-pibridge.json"; then
    ok "broker-status without a broker: identical ($(head -1 "$out/status-none-rust.json" | sed "s|$out|OUT|g"))"
else
    bad "broker-status without a broker differs"
    diff "$out/status-none-rust.json" "$out/status-none-pibridge.json"
fi

# 6. a stale socket file is replaced
for t in rust pibridge; do
    sock="$out/stale-$t.sock"
    echo stale > "$sock"
    "$(tool $t)" broker --source "$pxcam" --socket "$sock" 2> "$out/stale-$t.broker.log" &
    pid=$!
    if wait_socket "$sock" && "$pibridge_diff" client "$sock" "$pxcam" > /dev/null 2>&1; then
        ok "$t broker replaced a stale socket file"
    else
        bad "$t broker and a stale socket file"
    fi
    wait "$pid"
done

[ $fails = 0 ] && echo "all broker cross tests passed" && exit 0
echo "$fails broker cross test(s) failed"
exit 1
