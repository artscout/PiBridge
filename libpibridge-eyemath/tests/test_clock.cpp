// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/clock.rs tests (MIT OR Apache-2.0).
#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>

#include "pibridge/eyemath/clock.h"
#include "pibridge_test.h"

using namespace pibridge::eyemath;

namespace {

/// Tiny deterministic LCG for jitter.
struct Lcg {
    std::uint64_t s;
    double next() {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>(s >> 33) / static_cast<double>(1U << 31);
    }
};

} // namespace

TEST(tracks_offset_and_drift_through_jitter) {
    ClockMap m;
    Lcg rng{7};
    const std::int64_t host0 = 5'000'000'000; // host clock at device t=0
    const double drift = 40e-6;                // device crystal 40 ppm slow relative to host
    double worst = 0.0;
    for (std::int64_t i = 0; i < 120 * 120; i++) {
        const std::int64_t dev_us = 1'000'000 + i * 8300;
        const double true_host = static_cast<double>(host0) + (static_cast<double>(dev_us) * 1000.0) * (1.0 + drift);
        // 0.3..3 ms transport delay, rare 15 ms hiccup
        const double delay = 300'000.0 + rng.next() * 2'700'000.0 + (i % 500 == 0 ? 15e6 : 0.0);
        const auto arrival = static_cast<std::int64_t>(true_host + delay);
        const auto mapped = m.map(dev_us, arrival);
        if (i > 2000) {
            worst = std::fmax(worst, std::fabs(static_cast<double>(mapped) - true_host));
        }
    }
    // after warm-up the mapped time is within ~the minimum delay plus a bit of filter bias
    CHECK(worst < 2'500'000.0);
}

TEST(output_is_strictly_increasing_and_reset_on_epoch_change) {
    ClockMap m;
    const auto a = m.map(1'000'000, 10'000'000'000);
    const auto b = m.map(1'008'300, 10'008'000'000); // arrived "early": offset drops
    CHECK(b > a);
    // device restarted
    const auto c = m.map(5'000, 20'000'000'000);
    CHECK(c >= 20'000'000'000 - 1'000'000);
}

TEST(latency_allowance_is_subtracted) {
    ClockMap m;
    m.latency_ns = 2'000'000;
    CHECK(m.map(1'000, 1'000'000'000) == 1'000'000'000 - 2'000'000);
}

TEST(monotonic_clock_advances) {
    const auto a = monotonic_ns();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    CHECK(monotonic_ns() > a);
}

PIBRIDGE_TEST_MAIN()
