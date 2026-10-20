// SPDX-License-Identifier: MIT
// A fake Monado recorder tap for the tests: does what comp_recorder.c does on the ring (attach with the size
// pick_eye_extent gives a 3840x3552 eye, publish frames at the requested rate with CLOCK_MONOTONIC timestamps,
// heartbeat, detach on `stop`). No GPU.
//   pibridge_record_fakeproducer RING [FRAMES DONE_FILE]   (RING alone: endless, as pibridge-record-mirror is started)
// Waits up to 10 s for RING, publishes FRAMES frames, writes DONE_FILE, then keeps its heartbeat until the consumer
// asks it to stop (at most 30 s) and detaches.
#include "pibridge/record/shm.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <thread>

#include <unistd.h>

using namespace pibridge::record;
using namespace std::chrono_literals;

int main(int argc, char** argv) {
    if (argc != 4 && argc != 2) {
        std::fprintf(stderr, "usage: %s RING FRAMES DONE_FILE\n", argv[0]);
        return 2;
    }
    const std::filesystem::path ringPath = argv[1];
    const long frames = argc == 4 ? std::atol(argv[2]) : 1L << 40;
    const auto t0 = std::chrono::steady_clock::now();
    std::optional<Ring> ring;
    while (!ring) {
        if (auto r = Ring::open(ringPath)) {
            ring.emplace(std::move(*r));
            break;
        }
        if (std::chrono::steady_clock::now() - t0 > 10s) {
            std::fprintf(stderr, "fake producer: no ring at %s\n", ringPath.c_str());
            return 1;
        }
        std::this_thread::sleep_for(20ms);
    }
    const ShmHeader& h = ring->hdr();
    const auto ext = pickEyeExtent({3840, 3552}, h.max_height, h.scale_permille, h.eyes, h.slot_size);
    if (!ext) {
        return 1;
    }
    const std::uint32_t w = ext->first * h.eyes, ht = ext->second;
    ring->attachProducer(w, ht, "fake tap", static_cast<std::uint32_t>(getpid()));
    const std::uint64_t interval = 1'000'000'000ull / (h.fps == 0 ? 30 : h.fps);
    std::uint64_t ts = monotonicNs();
    long sent = 0;
    for (std::uint64_t seq = 0; sent < frames && !ring->stopRequested(); seq++) {
        ring->producerHeartbeat();
        if (ring->publish(w, ht, seq, ts, [&](std::span<std::uint8_t> b) {
                for (std::size_t i = 0; i < b.size(); i++) {
                    b[i] = static_cast<std::uint8_t>(seq + i);
                }
            })) {
            sent++;
            ts += interval;
        }
        std::this_thread::sleep_for(std::chrono::nanoseconds(interval / 4));
    }
    if (argc == 4) {
        std::ofstream(argv[3]) << sent << '\n';
    }
    const auto t1 = std::chrono::steady_clock::now();
    while (!ring->stopRequested() && std::chrono::steady_clock::now() - t1 < 30s) {
        ring->producerHeartbeat();
        std::this_thread::sleep_for(100ms);
    }
    ring->detachProducer();
    return 0;
}
