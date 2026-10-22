// SPDX-License-Identifier: MIT
// The Crystal's pose file: the layout round trip, the clock mapping, the writer against a temporary file (never the
// default path).
#include "pibridge_test.h"

#include "pibridge/hmd/pose_shm.h"

#include <cmath>
#include <fstream>
#include <iterator>
#include <unistd.h>

namespace ps = pibridge::hmd::pose_shm;
namespace p3b = pibridge::hmd::p3b;

TEST(the_body_round_trips) {
    ps::State s;
    s.written_ns = 123456789;
    s.tracking = 3;
    s.devices[0] = {.t_ns = 111, .state = 1, .battery = 73, .position = {0.1f, 1.6f, -0.2f},
                    .orientation = {0, 0.7071068f, 0, 0.7071068f}, .linear_velocity = {1, 2, 3},
                    .angular_velocity = {4, 5, 6}, .buttons = 2, .axes = {}, .sample = 9};
    s.devices[2] = {.t_ns = 222, .state = 7, .battery = 50, .position = {0.3f, 1.0f, -0.4f},
                    .orientation = {0, 0, 0, 1}, .linear_velocity = {}, .angular_velocity = {}, .buttons = 0,
                    .axes = {0.5f, 0, 0, -1}, .sample = 4};
    std::vector<std::uint8_t> f(ps::SIZE, 0);
    const auto b = ps::body(s);
    std::copy(b.begin(), b.end(), f.begin() + 8);
    f[0] = f[320] = 2; // a consistent sequence number
    CHECK(ps::decode(f) == std::optional<ps::State>(s));
    f[0] = 3; // a write in progress
    CHECK(!ps::decode(f));
    f[0] = 4; // torn
    CHECK(!ps::decode(f));
}

TEST(the_headset_clock_maps_to_the_host_by_the_quickest_report) {
    ps::Clock c;
    CHECK(c.to_host(1'000'000, 5'000'000) == 5'000'000);
    CHECK(c.to_host(2'000'000, 6'500'000) == 6'000'000); // waited 0.5 ms longer: the first offset stays
    CHECK(c.to_host(3'000'000, 6'800'000) == 6'800'000); // quicker: the offset follows it
    CHECK(c.to_host(1'000, 9'000'000'000) == 9'000'000'000); // the headset restarted: start over
}

TEST(a_headset_clock_running_slow_does_not_age_the_poses) {
    // the headset's clock 1.4 % slower than the host's (about what the original Crystal showed), a report every
    // 1.5 ms with 1..3 ms in transit: after ten minutes the mapped time still sits right before the receive time
    ps::Clock c;
    std::int64_t worst = 0;
    for (std::int64_t i = 0; i < 400'000; i++) {
        const std::int64_t host = 1'000'000'000 + i * 1'500'000;
        const auto headset = static_cast<std::uint64_t>(i * 1'500'000 * 986 / 1000);
        const std::int64_t received = host + 1'000'000 + (i % 3) * 1'000'000;
        const std::int64_t age = received - c.to_host(headset, received);
        if (i > 1000) {
            worst = std::max(worst, age);
        }
    }
    CHECK(worst < 7'000'000); // the transit time (up to 3 ms) plus two windows' drift (2.8 ms), not seconds
}

TEST(the_head_angular_velocity_turns_into_the_tracking_space) {
    p3b::Head h;
    h.tracked = true;
    h.pose.orientation = {0, 0.7071068f, 0, 0.7071068f}; // 90 degrees about y
    h.angular_velocity = {1, 0, 0};                        // about the body's x
    const auto d = ps::from_head(h, 5, 1);
    // body x -> the headset's -z; z is the axis of the quarter turn to y up, so it stays
    CHECK(std::abs(d.angular_velocity[0]) < 1e-5f && std::abs(d.angular_velocity[2] + 1) < 1e-5f);
    CHECK(d.state == 1 && d.t_ns == 5 && d.sample == 1);
}

TEST(the_controllers_stay_as_they_come) {
    p3b::Controller c;
    c.side = 1;
    c.state = 7;
    c.pose.position = {0.426f, -0.119f, -0.421f};
    c.pose.orientation = {0.334f, -0.119f, 0.704f, 0.616f};
    const auto d = ps::from_controller(c, 5, 1);
    CHECK(d.position == c.pose.position && d.orientation == c.pose.orientation);
}

TEST(the_headset_x_up_space_becomes_y_up) {
    // the pose taken in the headset looking straight ahead (2026-10-22): its up axis along the headset's +x
    p3b::Head h;
    h.tracked = true;
    h.pose.position = {1.6f, 0.2f, -0.3f};
    h.pose.orientation = {-0.355f, -0.363f, -0.617f, 0.601f};
    const auto d = ps::from_head(h, 5, 1);
    CHECK(std::abs(d.position[0] + 0.2f) < 1e-5f && std::abs(d.position[1] - 1.6f) < 1e-5f &&
          std::abs(d.position[2] + 0.3f) < 1e-5f); // x up -> y up
    // the head's up (+y) and forward (-z) in the file's space: up is up, forward is level
    const auto& q = d.orientation;
    const auto turn = [&](float vx, float vy, float vz) {
        const float x = q[0], y = q[1], z = q[2], w = q[3];
        const float tx = 2 * (y * vz - z * vy), ty = 2 * (z * vx - x * vz), tz = 2 * (x * vy - y * vx);
        return std::array<float, 3>{vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx)};
    };
    CHECK(turn(0, 1, 0)[1] > 0.99f);
    CHECK(std::abs(turn(0, 0, -1)[1]) < 0.05f);
    CHECK(d.state == 1 && d.t_ns == 5 && d.sample == 1);
}

TEST(the_writer_publishes_every_report) {
    const auto path = std::filesystem::temp_directory_path() / ("pibridge-test-pose-" + std::to_string(::getpid()));
    {
        auto p = ps::Publisher::create(path);
        CHECK(p.has_value());
        if (!p) {
            return;
        }
        p3b::Head h;
        h.tracked = true;
        h.tracking = 3;
        h.battery = 80;
        h.pose.t_ns = 1'000'000;
        h.pose.position = {0, 1.7f, 0};
        h.pose.orientation = {0, 0, 0, 1};
        p->head(h, 50'000'000);
        p3b::Controller c;
        c.side = 1;
        c.state = 7;
        c.battery = 40;
        c.pose.t_ns = 1'500'000;
        c.pose.orientation = {0, 0, 0, 1};
        p->controller(c, 50'600'000);
        std::ifstream in(path, std::ios::binary);
        const std::vector<std::uint8_t> f{std::istreambuf_iterator<char>(in), {}};
        const auto s = ps::decode(f);
        CHECK(s.has_value());
        if (s) {
            CHECK(s->tracking == 3 && s->written_ns == 50'600'000);
            CHECK(s->devices[0].state == 1 && s->devices[0].battery == 80 && std::abs(s->devices[0].position[0] + 1.7f) < 1e-5f);
            CHECK(s->devices[0].t_ns == 50'000'000);
            CHECK(s->devices[2].state == 7 && s->devices[2].battery == 40 && s->devices[2].t_ns == 50'500'000);
            CHECK(s->devices[1].state == 0 && s->devices[1].t_ns == 0); // the left one never seen
        }
        p->gone(60'000'000);
    }
    std::ifstream in(path, std::ios::binary);
    const std::vector<std::uint8_t> f{std::istreambuf_iterator<char>(in), {}};
    const auto s = ps::decode(f);
    CHECK(s && s->devices[0].state == 0 && s->devices[2].state == 0);
    std::filesystem::remove(path);
}

PIBRIDGE_TEST_MAIN()
