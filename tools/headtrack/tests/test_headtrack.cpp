// SPDX-License-Identifier: MIT
// pibridge-headtrack's own pieces: the IMU clock mapping, the 3DoF fallback, the pose file's source.
#include "headtrack.h"
#include "pibridge_test.h"

#include <cmath>
#include <fstream>
#include <iterator>
#include <unistd.h>

namespace ht = pibridge::headtrack;
namespace ps = pibridge::hmd::pose_shm;

TEST(imu_time_unwraps_and_follows_the_least_delayed_sample) {
    ht::ImuTime t;
    const std::int64_t base = 1'000'000'000'000;
    // device time 1 ms per sample, arriving 2 ms late, one 0.1 ms quick
    const std::int64_t a = t.map(1000, base + 2'000'000);
    const std::int64_t b = t.map(2000, base + 1'000'000 + 2'000'000);
    const std::int64_t c = t.map(3000, base + 2'000'000 + 1'900'000); // the quickest so far: the offset follows it
    CHECK(b - a == 1'000'000);
    CHECK(c - b == 900'000 || c - b == 1'000'000); // the quicker offset moves the mapping by up to 0.1 ms
    // the u32 device clock wraps: the mapped time goes on increasing by the real step
    ht::ImuTime w;
    const std::int64_t x = w.map(0xffffff00u, base);
    const std::int64_t y = w.map(0x00000100u, base + 512'000);
    CHECK(y - x == 512'000);
    const auto r = w.ref();
    CHECK(r.valid == 1 && r.dev_ts_u32 == 0x100u && r.dev_ext_us == 0x100000100ull);
}

TEST(orientation_starts_level_from_gravity_and_learns_the_gyro_bias) {
    ht::Orientation o;
    CHECK(!o.valid());
    // resting level (gravity along +y in the head's OpenXR axes), a gyro bias of 0.01 rad/s about y
    for (int i = 0; i < 1500; i++) {
        o.update(1'000'000LL * (i + 1), {0, 9.80665, 0}, {0, 0.01, 0});
    }
    CHECK(o.valid());
    const auto q = o.q();
    CHECK(std::abs(q[0]) < 0.02 && std::abs(q[2]) < 0.02); // no tilt
    CHECK(std::abs(o.gyro()[1]) < 1e-6);                     // the bias is learnt at rest
}

TEST(the_pose_file_says_who_writes_it) {
    const auto path = std::filesystem::temp_directory_path() / ("pibridge-test-headtrack-" + std::to_string(::getpid()));
    {
        auto p = ps::Publisher::create(path);
        CHECK(p.has_value());
        if (!p) {
            return;
        }
        ps::State s;
        s.source = ps::SOURCE_HEADTRACK;
        s.tracking = 3;
        s.devices[0].state = 1;
        s.devices[0].t_ns = 42;
        p->publish(s, 100);
    }
    std::ifstream in(path, std::ios::binary);
    const std::vector<std::uint8_t> f{std::istreambuf_iterator<char>(in), {}};
    const auto s = ps::decode(f);
    CHECK(s && s->source == ps::SOURCE_HEADTRACK && s->tracking == 3 && s->written_ns == 100 && s->devices[0].t_ns == 42);
    std::filesystem::remove(path);
}

PIBRIDGE_TEST_MAIN()
