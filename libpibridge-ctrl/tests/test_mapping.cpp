// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-hands/src/mapping.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/mapping.h"
#include "pibridge_test.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

using namespace pibridge::ctrl;

namespace {

constexpr std::uint64_t MS = 1000000;

/// Drive a state with a constant input for `ms` milliseconds at 4 ms steps.
struct Rig {
    HandState s{Params{}};
    std::uint64_t t = 1000 * MS;
    Curls run(std::uint64_t ms, Input i) {
        Curls c = s.curls();
        for (std::uint64_t k = 0; k < ms / 4; k++) {
            t += 4 * MS;
            c = s.update(t, i);
        }
        return c;
    }
};

Input inp(float trigger, float grip, std::uint32_t fl) {
    Input i;
    i.trigger = trigger;
    i.grip = grip;
    i.flags = fl;
    return i;
}

Input stick(float x, std::uint32_t fl = 0) {
    Input i;
    i.stick_x = x;
    i.flags = fl;
    return i;
}

const Params P;

} // namespace

TEST(rest_pose_is_relaxed) {
    Rig r;
    const Curls c = r.run(500, Input{});
    CHECK(std::fabs(c.index - P.index_rest) < 1e-3f);
    for (float v : {c.little, c.ring, c.middle}) {
        CHECK(std::fabs(v - P.grip_rest) < 1e-3f);
    }
    CHECK(std::fabs(c.thumb) < 1e-3f);
    CHECK(r.s.thumb_state() == ThumbState::Rest);
}

TEST(trigger_drives_only_the_index) {
    Rig r;
    const Curls rest = r.run(300, Input{});
    const Curls full = r.run(600, inp(1.0f, 0.0f, 0));
    CHECK(std::fabs(full.index - P.index_full) < 1e-3f);
    CHECK(full.middle == rest.middle && full.ring == rest.ring && full.little == rest.little && full.thumb == rest.thumb);
}

TEST(grip_drives_only_the_three_lower_fingers) {
    Rig r;
    const Curls rest = r.run(300, Input{});
    const Curls full = r.run(600, inp(0.0f, 1.0f, 0));
    for (float v : {full.little, full.ring, full.middle}) {
        CHECK(std::fabs(v - 1.0f) < 1e-3f);
    }
    CHECK(full.index == rest.index && full.thumb == rest.thumb);
}

TEST(curl_is_monotonic_in_trigger_and_grip) {
    for (bool pull_trigger : {true, false}) {
        float prev = -1.0f;
        for (int k = 0; k <= 50; k++) {
            const float v = static_cast<float>(k) / 50.0f;
            Rig r;
            const float c = pull_trigger ? r.run(800, inp(v, 0.0f, 0)).index : r.run(800, inp(0.0f, v, 0)).middle;
            CHECK(c >= prev - 1e-4f);
            prev = c;
        }
    }
}

TEST(deadzone_keeps_noise_out) {
    Rig r;
    const Curls a = r.run(500, inp(0.0f, 0.0f, 0));
    const Curls b = r.run(500, inp(0.015f, 0.015f, 0));
    CHECK(a.index == b.index && a.middle == b.middle);
}

TEST(thumb_follows_touch_and_goes_back_up) {
    Rig r;
    r.run(200, Input{});
    Curls c = r.run(600, inp(0.0f, 0.0f, flags::TOUCH_BTN1));
    CHECK(std::fabs(c.thumb - P.thumb_btn1) < 0.01f);
    CHECK(r.s.thumb_state() == ThumbState::Btn1);
    c = r.run(600, Input{});
    CHECK(c.thumb < 0.01f);
    CHECK(r.s.thumb_state() == ThumbState::Rest);
}

TEST(thumb_places_differ_per_button) {
    const struct {
        std::uint32_t fl;
        ThumbState state;
        float want;
    } cases[] = {
        {flags::TOUCH_BTN1, ThumbState::Btn1, P.thumb_btn1},
        {flags::TOUCH_BTN2, ThumbState::Btn2, P.thumb_btn2},
        {flags::TOUCH_STICK, ThumbState::Stick, P.thumb_stick},
        {flags::CLICK_MENU, ThumbState::Menu, P.thumb_menu + P.thumb_click_extra},
    };
    for (const auto& k : cases) {
        Rig r;
        r.run(100, Input{});
        const Curls c = r.run(800, inp(0.0f, 0.0f, k.fl));
        CHECK(r.s.thumb_state() == k.state);
        CHECK(std::fabs(c.thumb - k.want) < 0.01f);
    }
}

TEST(click_presses_deeper_than_touch) {
    Rig r;
    r.run(100, Input{});
    const Curls touch = r.run(600, inp(0.0f, 0.0f, flags::TOUCH_BTN2));
    const Curls click = r.run(600, inp(0.0f, 0.0f, flags::TOUCH_BTN2 | flags::CLICK_BTN2));
    CHECK(click.thumb > touch.thumb + 0.1f);
    const Curls back = r.run(600, inp(0.0f, 0.0f, flags::TOUCH_BTN2));
    CHECK(std::fabs(back.thumb - touch.thumb) < 0.01f);
}

TEST(click_alone_counts_as_contact) {
    Rig r;
    r.run(100, Input{});
    r.run(200, inp(0.0f, 0.0f, flags::CLICK_BTN1));
    CHECK(r.s.thumb_state() == ThumbState::Btn1);
}

TEST(most_recent_contact_wins_and_older_one_comes_back) {
    Rig r;
    r.run(100, Input{});
    r.run(200, inp(0.0f, 0.0f, flags::TOUCH_BTN1));
    CHECK(r.s.thumb_state() == ThumbState::Btn1);
    r.run(200, inp(0.0f, 0.0f, flags::TOUCH_BTN1 | flags::TOUCH_STICK));
    CHECK(r.s.thumb_state() == ThumbState::Stick);
    r.run(200, inp(0.0f, 0.0f, flags::TOUCH_BTN1));
    CHECK(r.s.thumb_state() == ThumbState::Btn1);
    r.run(200, Input{});
    CHECK(r.s.thumb_state() == ThumbState::Rest);
}

TEST(short_touch_dropouts_are_debounced) {
    Rig r;
    r.run(100, Input{});
    r.run(300, inp(0.0f, 0.0f, flags::TOUCH_BTN1));
    r.run(100, Input{});
    CHECK(r.s.thumb_state() == ThumbState::Btn1);
    r.run(200, inp(0.0f, 0.0f, flags::TOUCH_BTN1));
    r.run(200, Input{});
    CHECK(r.s.thumb_state() == ThumbState::Rest);
}

TEST(touch_blips_while_squeezing_do_not_move_the_thumb) {
    Rig r;
    r.run(300, Input{});
    for (std::uint64_t blip_ms : {16, 56, 76, 96, 100}) {
        r.run(40, inp(1.0f, 0.0f, 0));
        const Curls c = r.run(blip_ms, inp(1.0f, 0.0f, flags::TOUCH_BTN2 | flags::TOUCH_STICK));
        CHECK(r.s.thumb_state() == ThumbState::Rest);
        const Curls c2 = r.run(300, Input{});
        CHECK(c.thumb < 0.05f && c2.thumb < 0.05f);
    }
}

TEST(a_real_touch_is_taken_over_after_the_engage_time) {
    Rig r;
    r.run(300, Input{});
    r.run(100, inp(0.0f, 0.0f, flags::TOUCH_BTN1));
    CHECK(r.s.thumb_state() == ThumbState::Rest);
    r.run(60, inp(0.0f, 0.0f, flags::TOUCH_BTN1));
    CHECK(r.s.thumb_state() == ThumbState::Btn1);
}

TEST(clicks_and_stick_deflection_skip_the_engage_time) {
    Rig r;
    r.run(300, Input{});
    r.run(8, inp(0.0f, 0.0f, flags::CLICK_BTN2));
    CHECK(r.s.thumb_state() == ThumbState::Btn2);
    r.run(400, Input{});
    CHECK(r.s.thumb_state() == ThumbState::Rest);
    r.run(8, stick(0.8f));
    CHECK(r.s.thumb_state() == ThumbState::Stick);
}

TEST(stick_deflection_is_not_a_press) {
    Rig r;
    r.run(300, Input{});
    const Curls deflected = r.run(600, stick(0.8f));
    CHECK(std::fabs(deflected.thumb - P.thumb_stick) < 0.01f);
    const Curls clicked = r.run(600, stick(0.8f, flags::CLICK_STICK));
    CHECK(std::fabs(clicked.thumb - (P.thumb_stick + P.thumb_click_extra)) < 0.01f);
}

TEST(flickering_touch_bits_are_bridged) {
    Rig r;
    r.run(300, Input{});
    r.run(300, inp(0.0f, 0.0f, flags::TOUCH_STICK));
    CHECK(r.s.thumb_state() == ThumbState::Stick);
    for (std::uint64_t gap_ms : {40, 64, 100, 140, 40, 120}) {
        r.run(gap_ms, Input{});
        CHECK(r.s.thumb_state() == ThumbState::Stick);
        r.run(60, inp(0.0f, 0.0f, flags::TOUCH_STICK));
    }
}

TEST(stick_deflection_implies_touch_with_hysteresis) {
    Rig r;
    r.run(100, Input{});
    r.run(200, stick(0.15f));
    CHECK(r.s.thumb_state() == ThumbState::Rest);
    r.run(200, stick(0.5f));
    CHECK(r.s.thumb_state() == ThumbState::Stick);
    r.run(200, stick(0.15f));
    CHECK(r.s.thumb_state() == ThumbState::Stick);
    r.run(300, stick(0.05f));
    CHECK(r.s.thumb_state() == ThumbState::Rest);
}

TEST(thumb_moves_smoothly_not_in_a_jump) {
    Rig r;
    r.run(100, Input{});
    float prev = r.s.curls().thumb;
    float max_step = 0.0f;
    std::optional<std::uint64_t> reached_at;
    for (std::uint64_t k = 1; k <= 200; k++) {
        r.t += 4 * MS;
        const Curls c = r.s.update(r.t, inp(0.0f, 0.0f, flags::TOUCH_STICK));
        max_step = std::fmax(max_step, std::fabs(c.thumb - prev));
        prev = c.thumb;
        if (!reached_at && c.thumb > 0.9f * P.thumb_stick) {
            reached_at = k * 4;
        }
    }
    CHECK(max_step < 0.08f);
    CHECK(reached_at && *reached_at >= 40 && *reached_at < 400);
}

TEST(long_silence_restarts_from_the_new_sample) {
    Rig r;
    r.run(300, inp(0.0f, 0.0f, flags::TOUCH_BTN1));
    r.t += 3000 * MS;
    const Curls c = r.s.update(r.t, inp(1.0f, 1.0f, flags::TOUCH_STICK));
    CHECK(std::fabs(c.index - P.index_full) < 1e-4f);
    CHECK(std::fabs(c.middle - 1.0f) < 1e-4f);
    CHECK(std::fabs(c.thumb - P.thumb_stick) < 1e-4f);
    CHECK(r.s.thumb_state() == ThumbState::Stick);
}

TEST(first_sample_has_no_ramp) {
    HandState s{Params{}};
    const Curls c = s.update(5000 * MS, inp(0.5f, 0.5f, 0));
    CHECK(std::fabs(c.index - lerp(P.index_rest, P.index_full, shape(0.5f, P.deadzone))) < 1e-5f);
}

TEST(backwards_or_repeated_timestamps_are_harmless) {
    Rig r;
    r.run(100, inp(0.5f, 0.5f, 0));
    const Curls before = r.s.curls();
    Curls c = r.s.update(r.t, inp(0.5f, 0.5f, 0));
    CHECK(c == before);
    c = r.s.update(r.t - 50 * MS, inp(0.5f, 0.5f, 0));
    CHECK(std::isfinite(c.index));
    r.t += 4 * MS;
    c = r.s.update(r.t, inp(1.0f, 1.0f, 0));
    CHECK(c.index >= before.index);
}

TEST(garbage_input_is_sanitised) {
    HandState s{Params{}};
    std::uint64_t t = 10 * MS;
    const float bad[] = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(),
                         -std::numeric_limits<float>::infinity(), -5.0f, 7.0f, std::numeric_limits<float>::max(),
                         std::numeric_limits<float>::lowest()};
    for (float b : bad) {
        t += 4 * MS;
        const Curls c = s.update(t, Input{b, b, b, b, UINT32_MAX});
        for (float v : c.to_array()) {
            CHECK(std::isfinite(v) && v >= 0.0f && v <= 1.0f);
        }
    }
}

TEST(reset_returns_to_rest) {
    Rig r;
    r.run(500, inp(1.0f, 1.0f, flags::TOUCH_STICK));
    r.s.reset();
    const Curls c = r.s.curls();
    CHECK(std::fabs(c.index - P.index_rest) < 1e-6f && c.thumb == 0.0f);
    CHECK(r.s.thumb_state() == ThumbState::Rest);
}

TEST(jitter_on_the_trigger_is_filtered) {
    Rig r;
    r.run(300, inp(0.5f, 0.0f, 0));
    float lo = std::numeric_limits<float>::max();
    float hi = std::numeric_limits<float>::lowest();
    std::uint32_t x = 0x2468ace1u;
    for (int i = 0; i < 500; i++) {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        const float noise = ((static_cast<float>(x) / static_cast<float>(UINT32_MAX)) - 0.5f) * 0.02f;
        r.t += 4 * MS;
        const Curls c = r.s.update(r.t, inp(0.5f + noise, 0.0f, 0));
        lo = std::fmin(lo, c.index);
        hi = std::fmax(hi, c.index);
    }
    CHECK(hi - lo < 0.006f);
}

TEST(fast_trigger_pull_is_followed_quickly) {
    Rig r;
    r.run(300, Input{});
    std::optional<std::uint64_t> reached;
    for (std::uint64_t k = 1; k <= 100; k++) {
        r.t += 4 * MS;
        const Curls c = r.s.update(r.t, inp(1.0f, 0.0f, 0));
        if (!reached && c.index > 0.9f * P.index_full) {
            reached = k * 4;
        }
    }
    CHECK(reached && *reached < 160);
}

TEST(joint_curls_match_the_index_knuckles_simulation) {
    const HandJoints all = to_joints(Curls{1.0f, 1.0f, 1.0f, 1.0f, 1.0f});
    const auto near = [](float a, float b) { return std::fabs(a - b) < 1e-4f; };
    const auto& i = all.index.joint_curls;
    CHECK(near(i[0], 0.0f) && near(i[1], 1.43f) && near(i[2], 1.1f) && near(i[3], 1.1f));
    for (const FingerJoints& f : {all.middle, all.ring, all.little}) {
        const auto& j = f.joint_curls;
        CHECK(near(j[0], 0.0f) && near(j[1], 1.859f) && near(j[2], 1.43f) && near(j[3], 0.572f));
    }
    const auto& t = all.thumb.joint_curls;
    CHECK(near(t[0], 1.0f) && near(t[1], 1.57f) && near(t[2], 1.4f) && near(t[3], 0.0f));
    CHECK(all.thumb.splay == -0.35f);
    CHECK(all.index.splay == SPLAY[0] && all.middle.splay == SPLAY[1] && all.ring.splay == SPLAY[2] &&
          all.little.splay == SPLAY[3]);
}

TEST(joint_curls_zero_is_an_open_hand_and_scales_linearly) {
    const HandJoints open = to_joints(Curls{});
    for (const FingerJoints& f : {open.little, open.ring, open.middle, open.index, open.thumb}) {
        CHECK((f.joint_curls == std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f}));
    }
    Curls h;
    h.index = 0.5f;
    Curls fu;
    fu.index = 1.0f;
    const HandJoints half = to_joints(h);
    const HandJoints full = to_joints(fu);
    for (int k = 0; k < 4; k++) {
        CHECK(std::fabs(half.index.joint_curls[k] * 2.0f - full.index.joint_curls[k]) < 1e-5f);
    }
}

TEST(joint_curls_clamp_out_of_range_input) {
    const HandJoints j = to_joints(Curls{5.0f, -1.0f, std::numeric_limits<float>::quiet_NaN(), 2.0f,
                                         std::numeric_limits<float>::infinity()});
    for (const FingerJoints& f : {j.little, j.ring, j.middle, j.index, j.thumb}) {
        for (float v : f.joint_curls) {
            CHECK(std::isfinite(v) && v >= 0.0f && v <= 2.0f);
        }
    }
    CHECK((j.ring.joint_curls == std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f}));
    CHECK((j.middle.joint_curls == std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f}));
}

PIBRIDGE_TEST_MAIN()
