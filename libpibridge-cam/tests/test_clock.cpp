// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-cam/src/clock.rs (MIT OR Apache-2.0).
#include "pibridge/cam/clock.h"
#include "pibridge_test.h"

#include <cstdlib>
#include <vector>

using namespace pibridge::cam;

TEST(imu_mapping_unwraps) {
    // IMU: u32 0xffff_ff00 extended to 5_000_000_000_000, offset 1 s.
    const ImuClockRef r{0xffff'ff00u, 1, 5'000'000'000'000ull, 1'000'000'000};
    // camera frame 0x200 us later, after the u32 wrap; its high word differs from the IMU's notion entirely.
    const std::uint64_t cam = (7ull << 32) | 0x100;
    CHECK(imu_dev_to_mono_ns(cam, r) == (5'000'000'000'000ull + 0x200) * 1000 + 1'000'000'000);
    // 20 ms earlier than the reference
    const std::uint64_t cam2 = static_cast<std::uint32_t>(0xffff'ff00u - 20'000u);
    CHECK(imu_dev_to_mono_ns(cam2, r) == (5'000'000'000'000ull - 20'000) * 1000 + 1'000'000'000);
}

TEST(host_map_tracks_minimum_delay) {
    HostClockMap m;
    // true offset 10 s; arrivals delayed by 11.8 ms +- jitter
    const std::int64_t jitter[] = {3'000'000, 0, 4'000'000, 1'000'000, 2'500'000, 0, 3'900'000};
    std::vector<std::uint64_t> out;
    for (std::uint64_t i = 0; i < 2000; i++) {
        const std::uint64_t dev = 1'000'000 + i * 19'988;
        const std::int64_t host =
            static_cast<std::int64_t>(dev * 1000) + 10'000'000'000 + 11'800'000 + jitter[i % 7];
        out.push_back(m.map(dev, static_cast<std::uint64_t>(host)));
    }
    // after convergence the output equals device time + offset + minimum delay, jitter removed
    const std::uint64_t last = out.back();
    const std::uint64_t dev = 1'000'000 + 1999 * 19'988ull;
    const std::uint64_t ideal = dev * 1000 + 10'000'000'000ull + 11'800'000;
    CHECK(std::llabs(static_cast<long long>(last) - static_cast<long long>(ideal)) < 200'000);
    bool inc = true;
    for (std::size_t i = 1; i < out.size(); i++) {
        inc = inc && out[i] > out[i - 1];
    }
    CHECK(inc);
    // frame spacing stays ~19.988 ms (no jitter transferred)
    const std::uint64_t d = out[1999] - out[1998];
    CHECK(std::llabs(static_cast<long long>(d) - 19'988'000) < 50'000);
}

TEST(host_map_resets_on_jump) {
    HostClockMap m;
    const auto a = m.map(10'000'000, 50'000'000'000);
    const auto b = m.map(1'000, 51'000'000'000); // device time went back (replay loop)
    CHECK(m.resets == 1);
    CHECK(a == 50'000'000'000ull);
    CHECK(b == 51'000'000'000ull);
}

PIBRIDGE_TEST_MAIN()
