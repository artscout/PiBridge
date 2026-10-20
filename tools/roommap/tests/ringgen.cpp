// SPDX-License-Identifier: MIT
// Test helper: write a head ring like Monado's (anchor::ring_bytes).
//   ringgen OUT PID AGE_S N [STEP_MS]: N samples, the newest AGE_S seconds before now (CLOCK_MONOTONIC), STEP_MS apart
//   (default 4), every third without a position; N = 0 writes a ring without samples.
#include "pibridge/anchor/rustx.h"
#include "pibridge/anchor/world.h"
#include "pibridge/cam/io.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace pibridge;

int main(int argc, char** argv) {
    if (argc < 5) {
        std::fputs("usage: ringgen OUT PID AGE_S N [STEP_MS]\n", stderr);
        return 2;
    }
    const auto pid = static_cast<std::uint32_t>(std::strtoul(argv[2], nullptr, 10));
    const double age = std::strtod(argv[3], nullptr);
    const int n = std::atoi(argv[4]);
    const double step_ms = argc > 5 ? std::strtod(argv[5], nullptr) : 4.0;
    const std::uint64_t newest = anchor::rs::monotonic_ns() - static_cast<std::uint64_t>(age * 1e9);
    std::vector<anchor::RingSample> s;
    for (int i = 0; i < n; i++) {
        const double a = 0.01 * i;
        const std::uint64_t ts = newest - static_cast<std::uint64_t>((n - 1 - i) * step_ms * 1e6);
        s.push_back({ts, cam::geom::Pose{cam::geom::rodrigues({0.0, a, 0.0}), {0.002 * i, 1.6, -0.001 * i}}, i % 3 != 2});
    }
    if (auto w = cam::write_file(argv[1], anchor::ring_bytes(pid, s)); !w) {
        std::fprintf(stderr, "ringgen: %s\n", w.error().text().c_str());
        return 1;
    }
    return 0;
}
