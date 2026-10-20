// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/track.rs tests (MIT OR Apache-2.0).
#include <cmath>
#include <cstdlib>
#include <vector>

#include "pibridge/eyemath/track.h"
#include "pibridge_test.h"

using namespace pibridge::eyemath;

namespace {

Eye eye(Vec3 o, Vec3 d, bool valid) {
    Eye e;
    e.origin_mm = o;
    e.origin_valid = valid;
    e.direction = d;
    e.direction_valid = valid;
    e.pupil_diameter_mm = 3.0;
    return e;
}

/// The frame from the replay log: A (29.8, 4.1, -20.2) dir (0.326, -0.005, 0.945), B (-28.5, 2.6, -19.4) dir
/// (0.033, -0.025, 0.999).
GazeSample replay_frame() {
    GazeSample g;
    g.timestamp_us = 1000;
    g.eyes = {eye({29.8, 4.1, -20.2}, {0.326, -0.005, 0.945}, true),
              eye({-28.5, 2.6, -19.4}, {0.033, -0.025, 0.999}, true)};
    return g;
}

} // namespace

TEST(default_conversion_flips_z_only_and_a_is_right) {
    const auto s = convert(replay_frame(), FrameConv{}, 0, 5, 6);
    // right eye (slot 1) is device eye A
    CHECK_NEAR(s.eyes[1].origin[0], 0.0298, 1e-6);
    CHECK_NEAR(s.eyes[1].origin[2], 0.0202, 1e-6);
    CHECK(s.eyes[1].direction[2] < -0.9F); // forward must be -z
    CHECK_NEAR(s.eyes[0].origin[0], -0.0285, 1e-6);
    const auto g = *s.gaze;
    CHECK(g.eyes == 3);
    float l2 = 0.0F;
    for (float c : g.direction) {
        l2 += c * c;
    }
    CHECK_NEAR(std::sqrt(l2), 1.0, 1e-5);
    CHECK(g.direction[2] < -0.9F);
    CHECK_NEAR(g.origin[0], (0.0298 - 0.0285) / 2.0, 1e-6);
}

TEST(flip_x_swaps_the_eyes_and_negates_x) {
    FrameConv c;
    c.flip_x = true;
    const auto s = convert(replay_frame(), c, 0, 0, 0);
    CHECK_NEAR(s.eyes[0].origin[0], -0.0298, 1e-6); // A is now left, x negated
    CHECK(s.eyes[0].origin[0] < 0.0F && s.eyes[1].origin[0] > 0.0F); // left has -x, right +x
    CHECK_NEAR(s.eyes[0].direction[0], -0.326, 1e-3);
}

TEST(single_valid_eye_gives_its_own_ray) {
    auto g = replay_frame();
    g.eyes[1] = eye({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, false);
    const auto s = convert(g, FrameConv{}, 0, 0, 0);
    const auto gz = *s.gaze;
    CHECK(gz.eyes == 0b10);
    CHECK_NEAR(gz.origin[0], 0.0298, 1e-6);
    CHECK(!s.eyes[0].direction_valid);
}

TEST(no_valid_eye_no_gaze) {
    auto g = replay_frame();
    g.eyes[0] = eye({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, false);
    g.eyes[1] = eye({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, false);
    CHECK(!convert(g, FrameConv{}, 0, 0, 0).gaze.has_value());
}

TEST(origin_only_eye_does_not_contribute) {
    auto g = replay_frame();
    g.eyes[0].direction_valid = false;
    const auto s = convert(g, FrameConv{}, 0, 0, 0);
    CHECK(s.gaze->eyes == 0b01);
    CHECK(s.eyes[1].origin_valid && !s.eyes[1].direction_valid);
}

TEST(sample_at_interpolates_and_clamps) {
    const auto mk = [](std::int64_t t, double dx) {
        auto g = replay_frame();
        g.eyes[0].direction = {dx, 0.0, 1.0};
        g.eyes[1].direction = {dx, 0.0, 1.0};
        return convert(g, FrameConv{}, 0, t, t);
    };
    const std::vector<Sample> h{mk(1000, 0.0), mk(2000, 1.0)};
    CHECK(sample_at(h, 0)->host_ts_ns == 1000);
    CHECK(sample_at(h, 9999)->host_ts_ns == 2000);
    const auto m = *sample_at(h, 1500);
    CHECK(m.host_ts_ns == 1500);
    const auto d = m.gaze->direction;
    // halfway between straight ahead and 45 degrees
    CHECK(d[0] > 0.2F && d[0] < 0.5F);
    CHECK(!sample_at(std::span<const Sample>{}, 5).has_value());
}

// C++ only: from_env reads the PIBRIDGE_EYE_* variables of the table in track.h.
TEST(from_env_reads_the_variables) {
    ::unsetenv("PIBRIDGE_EYE_FLIP_X");
    ::unsetenv("PIBRIDGE_EYE_FLIP_Y");
    ::unsetenv("PIBRIDGE_EYE_FLIP_Z");
    ::unsetenv("PIBRIDGE_EYE_OFFSET_MM");
    ::unsetenv("PIBRIDGE_EYE_SWAP");
    CHECK(FrameConv::from_env() == FrameConv{});
    ::setenv("PIBRIDGE_EYE_FLIP_X", " yes ", 1);
    ::setenv("PIBRIDGE_EYE_FLIP_Z", "0", 1);
    ::setenv("PIBRIDGE_EYE_OFFSET_MM", "1.5, -2,3", 1);
    ::setenv("PIBRIDGE_EYE_SWAP", "on", 1);
    auto c = FrameConv::from_env();
    CHECK(c.flip_x && !c.flip_y && !c.flip_z && c.swap_eyes);
    CHECK((c.offset_mm == Vec3{1.5, -2.0, 3.0}));
    CHECK(c.slot(true) == 1); // flip_x makes A left, the swap makes it right again
    ::setenv("PIBRIDGE_EYE_OFFSET_MM", "1,2", 1); // not three numbers: the default stays
    c = FrameConv::from_env();
    CHECK((c.offset_mm == Vec3{0.0, 0.0, 0.0}));
    ::unsetenv("PIBRIDGE_EYE_FLIP_X");
    ::unsetenv("PIBRIDGE_EYE_FLIP_Z");
    ::unsetenv("PIBRIDGE_EYE_OFFSET_MM");
    ::unsetenv("PIBRIDGE_EYE_SWAP");
}

// C++ only: the PIMAX_EYE_* names (omarchyvr, pimax-eyecal) still work; the PIBRIDGE_EYE_* name wins over them.
TEST(from_env_accepts_the_older_names) {
    for (const char* v : {"PIBRIDGE_EYE_FLIP_X", "PIBRIDGE_EYE_FLIP_Y", "PIBRIDGE_EYE_FLIP_Z", "PIMAX_EYE_FLIP_X",
                          "PIMAX_EYE_FLIP_Y", "PIMAX_EYE_FLIP_Z"}) {
        ::unsetenv(v);
    }
    ::setenv("PIMAX_EYE_FLIP_X", "1", 1);
    ::setenv("PIMAX_EYE_FLIP_Y", "1", 1);
    ::setenv("PIBRIDGE_EYE_FLIP_Y", "0", 1);
    const auto c = FrameConv::from_env();
    CHECK(c.flip_x && !c.flip_y && c.flip_z);
    for (const char* v : {"PIBRIDGE_EYE_FLIP_Y", "PIMAX_EYE_FLIP_X", "PIMAX_EYE_FLIP_Y"}) {
        ::unsetenv(v);
    }
}

PIBRIDGE_TEST_MAIN()
