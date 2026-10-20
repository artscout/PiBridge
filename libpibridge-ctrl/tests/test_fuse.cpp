// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-hands/src/fuse.rs (MIT OR Apache-2.0).
#include "pibridge/cam/rustfmt.h"
#include "pibridge/ctrl/fuse.h"
#include "pibridge/ctrl/rust.h"
#include "pibridge_test.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <numbers>
#include <string>
#include <unistd.h>
#include <vector>

using namespace pibridge::ctrl::fuse;
using pibridge::cam::rfmt::to_degrees;
using pibridge::cam::rfmt::to_radians;

namespace {

V3 vadd(V3 a, V3 b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
V3 vsub(V3 a, V3 b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
V3 vscale(V3 a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
double vnorm(V3 a) { return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); }

constexpr Q QID = {0.0, 0.0, 0.0, 1.0};

/// Simple LCG noise.
struct Rng {
    std::uint64_t s;
    double f() {
        s = s * 6364136223846793005ull + 1442695040888963407ull;
        return (static_cast<double>(s >> 11) / static_cast<double>(1ull << 53)) * 2.0 - 1.0;
    }
};

std::array<std::uint8_t, 64> report(std::uint32_t ts, std::array<std::int16_t, 3> g0,
                                    std::optional<std::array<std::int16_t, 3>> g1) {
    std::array<std::uint8_t, 64> r{};
    r[0] = 0x53;
    r[1] = 0x32;
    r[2] = 0x08;
    r[3] = 2;
    for (int k = 0; k < 4; k++) {
        r[8 + k] = static_cast<std::uint8_t>(ts >> (8 * k));
    }
    const auto be = [&](std::size_t o, std::int16_t v) {
        r[o] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(v) >> 8);
        r[o + 1] = static_cast<std::uint8_t>(v);
    };
    for (std::size_t k = 0; k < 3; k++) {
        be(22 + 2 * k, g0[k]);
        be(16 + 2 * k, 1000);
    }
    if (g1) {
        for (std::size_t k = 0; k < 3; k++) {
            be(36 + 2 * k, (*g1)[k]);
            be(30 + 2 * k, 1000);
        }
    }
    return r;
}

struct Event {
    std::uint64_t t;
    std::optional<std::array<std::uint8_t, 64>> r;
    std::optional<std::pair<P, P>> o;
};

/// Synthetic session: smooth rotation, gyro reports at 250 Hz x 2 samples, optical fixes at 25 Hz with noise.
/// True mapping: aim = (raw y, -raw z, -raw x) * 1/16.4 deg/s, bias (12, -7, 3) LSB, IMU 16 ms late.
std::vector<Event> synth(double secs) {
    std::vector<Event> ev;
    const double s = to_radians(1.0 / 16.4);
    const M3 c_true = {{{0.0, s, 0.0}, {0.0, 0.0, -s}, {-s, 0.0, 0.0}}};
    const M3 ci = *inv3(c_true);
    const V3 bias = {12.0, -7.0, 3.0};
    Rng rng{7};
    const auto omega = [](double t) -> V3 {
        return {3.0 * std::sin(1.3 * t) + 1.0 * std::cos(3.1 * t), 2.5 * std::sin(0.7 * t + 1.0), 2.0 * std::cos(2.3 * t)};
    };
    Q q = QID;
    const std::uint64_t t0 = 10000000000ull;
    const double dt = 0.002;
    const auto n = static_cast<std::size_t>(secs / dt);
    const std::uint64_t lat_ns = 16000000;
    std::optional<std::array<std::int16_t, 3>> pending;
    for (std::size_t k = 0; k < n; k++) {
        const double ts = static_cast<double>(k) * dt;
        const V3 w = omega(ts);
        q = qnorm(qmul(q, qexp(vscale(w, dt))));
        const std::uint64_t host = t0 + pibridge::cam::rfmt::f64_to_u64(ts * 1e9);
        const V3 raw = vadd(mv(ci, w), bias);
        std::array<std::int16_t, 3> g;
        for (int j = 0; j < 3; j++) {
            g[j] = pibridge::ctrl::rs::f64_to_i16(std::round(raw[j] + rng.f() * 2.0));
        }
        if (!pending) {
            pending = g;
        } else {
            const auto g0 = *pending;
            pending.reset();
            const std::uint32_t dev = pibridge::cam::rfmt::f64_to_u32((ts - dt) * 1e6) + 16000;
            ev.push_back({host + lat_ns + 1000000, report(dev, g0, g), std::nullopt});
        }
        if (k % 20 == 0) {
            const V3 noise = {rng.f() * 0.02, rng.f() * 0.02, rng.f() * 0.02};
            Q qn = qnorm(qmul(q, qexp(noise)));
            if ((k / 20) % 8 == 3) {
                const V3 flip = {rng.f() * 1.5, rng.f() * 1.5, rng.f() * 1.5};
                qn = qnorm(qmul(qn, qexp(flip)));
            }
            ev.push_back({host, std::nullopt, std::pair<P, P>{P{qn, {0.1, -0.1, -0.4}}, P::id()}});
        }
    }
    return ev;
}

Config legacy(Config c) {
    c.set_legacy_window();
    return c;
}

Config noimu() {
    Config c;
    c.use_imu = false;
    return c;
}

const char* fix_at(Fuser& fu, std::uint64_t t_ms, double x) {
    return fu.optical_with(t_ms * 1000000, P{QID, {x, 0.0, -0.4}}, P::id(), 0.5f, 10, std::nullopt);
}

std::array<std::int16_t, 3> raw_for(const Calib& c, V3 w_aim) {
    const M3 ci = *inv3(c.c);
    const V3 r = vadd(mv(ci, w_aim), c.bias);
    return {pibridge::ctrl::rs::f64_to_i16(std::round(r[0])), pibridge::ctrl::rs::f64_to_i16(std::round(r[1])),
            pibridge::ctrl::rs::f64_to_i16(std::round(r[2]))};
}

std::string tmp(const std::string& name) {
    const char* d = std::getenv("TMPDIR");
    return std::string(d ? d : "/tmp") + "/" + name + std::to_string(::getpid()) + ".json";
}

} // namespace

TEST(an_online_fit_never_replaces_a_better_one) {
    CHECK(fit_replaces(std::nullopt, 0.8));
    CHECK(fit_replaces(0.9, 0.95));
    CHECK(fit_replaces(0.9, 0.9));
    CHECK(fit_replaces(0.9, 0.89));
    CHECK(!fit_replaces(0.95, 0.8));
    CHECK(!fit_replaces(0.9, 0.87));
}

TEST(online_fit_recovers_scale_axes_bias_and_offset) {
    Fuser fu(0, Config{}, std::nullopt);
    auto events = synth(60.0);
    std::stable_sort(events.begin(), events.end(), [](const Event& a, const Event& b) { return a.t < b.t; });
    for (const Event& e : events) {
        if (e.r) {
            fu.report(e.t, e.r->data());
        }
        if (e.o) {
            fu.optical(e.t, e.o->first, e.o->second, 0.5f, 10);
        }
    }
    CHECK(fu.calib.has_value());
    if (!fu.calib) {
        return;
    }
    const Calib c = *fu.calib;
    CHECK(std::fabs(c.scale_dps_per_lsb() - 1.0 / 16.4) < 0.03 / 16.4);
    CHECK(c.axes() == "+y -z -x");
    CHECK(c.axis_misalignment_deg() < 2.0);
    CHECK(std::fabs(c.bias[0] - 12.0) < 5.0 && std::fabs(c.bias[1] + 7.0) < 5.0);
    CHECK(c.time_offset_ms == 16.0 || c.time_offset_ms == 32.0 || c.time_offset_ms == 0.0);
    CHECK(c.r2 > 0.95);
}

TEST(save_and_load_keep_both_hands) {
    const std::string p = tmp("pimax-ctrl-imu-");
    Calib c;
    c.c = {{{0.001, 0.0, 0.0}, {0.0, 0.0, -0.001}, {0.0, 0.001, 0.0}}};
    c.bias = {1.0, 2.0, 3.0};
    c.time_offset_ms = 16.0;
    c.pairs = 500;
    c.moving = 200;
    c.r2 = 0.93;
    CHECK(save_calib(p, 0, c));
    Calib c2 = c;
    c2.bias = {4.0, 5.0, 6.0};
    CHECK(save_calib(p, 1, c2));
    const auto l = load_calib(p, 0);
    const auto r = load_calib(p, 1);
    CHECK(l && r);
    if (l && r) {
        CHECK(std::fabs(l->c[1][2] + 0.001) < 1e-12 && std::fabs(l->bias[2] - 3.0) < 1e-9);
        CHECK(std::fabs(r->bias[0] - 4.0) < 1e-9 && r->pairs == 500 && std::fabs(r->time_offset_ms - 16.0) < 1e-9);
        CHECK(l->axes() == "+x -z +y");
    }
    ::unlink(p.c_str());
}

TEST(out_of_view_extrapolates_then_holds_relative_to_the_head_then_gives_up) {
    Config cfg = noimu();
    cfg.lost_s = 3.0;
    cfg.follow_yaw = 1.0;
    Fuser fu(1, cfg, std::nullopt);
    const P head = P::id();
    for (std::uint64_t k = 0; k < 10; k++) {
        fu.optical(1000000000 + k * 40000000, P{QID, {0.04 * static_cast<double>(k), 0.0, -0.4}}, head, 0.5f, 10);
    }
    const std::uint64_t tl = 1000000000 + 9 * 40000000;
    auto [s, p] = fu.get(tl + 50000000, head);
    CHECK(s == State::Optical);
    CHECK(p.p[0] > 0.36 + 0.02 && p.p[0] < 0.36 + 0.05);
    auto [s2, p_hold] = fu.get(tl + 300000000, head);
    CHECK(s2 == State::Hold);
    const double x_end = p_hold.p[0];
    const P turned{{0.0, std::sin(std::numbers::pi / 4), 0.0, std::cos(std::numbers::pi / 4)}, {0.0, 0.0, -0.3}};
    auto [s3, p2] = fu.get(tl + 600000000, turned);
    CHECK(s3 == State::Hold);
    CHECK(std::fabs(p2.p[0] + 0.4) < 1e-6 && std::fabs(p2.p[2] - (-x_end - 0.3)) < 1e-6);
    CHECK(fu.get(tl + 3500000000ull, head).first == State::None);
}

TEST(reacquire_blends_and_outliers_are_dropped) {
    Config cfg = noimu();
    cfg.blend_mps = 0.0;
    cfg.blend_dps = 0.0;
    Fuser fu(0, legacy(cfg), std::nullopt);
    const P head = P::id();
    fu.optical(1000000000, P{QID, {0.0, 0.0, -0.4}}, head, 0.5f, 10);
    fu.optical(1040000000, P{QID, {0.0, 0.0, -0.4}}, head, 0.5f, 10);
    fu.optical(1080000000, P{QID, {1.0, 0.0, -0.4}}, head, 0.5f, 10);
    CHECK(fu.stats.rejected_body == 1);
    CHECK(std::fabs(fu.get(1090000000, head).second.p[0]) < 1e-6);
    const std::uint64_t t = 2100000000;
    const std::uint64_t q0 = t + 60000000;
    fu.optical(t, P{QID, {0.2, 0.0, -0.4}}, head, 0.5f, 10);
    const P a = fu.get(q0, head).second;
    const P b = fu.get(q0 + 75000000, head).second;
    const P c = fu.get(q0 + 160000000, head).second;
    CHECK(std::fabs(a.p[0]) < 0.02);
    CHECK(b.p[0] > 0.07 && b.p[0] < 0.13);
    CHECK(std::fabs(c.p[0] - 0.2) < 0.01);
    CHECK(fu.stats.reacquires == 1);
}

TEST(body_model_never_merges_the_hands_and_keeps_a_held_hand_in_reach) {
    const P head = P::id();
    Fuser left(0, noimu(), std::nullopt);
    left.optical(1000000000, P{QID, {-0.15, -0.1, -0.35}}, head, 0.5f, 10);
    const V3 right_at = {0.12, -0.1, -0.35};
    for (std::uint64_t k = 1; k < 6; k++) {
        const std::string v = left.optical_with(1000000000 + k * 40000000, P{QID, {0.13, -0.1, -0.35}}, head, 0.5f, 10, right_at);
        CHECK(v == "merged");
    }
    const std::string v = left.optical_with(1300000000, P{QID, {0.45, -0.1, -0.3}}, head, 0.5f, 10, std::nullopt);
    CHECK(v == "side");
    auto [s, p] = left.get_with(2000000000, head, V3{-0.15, -0.1, -0.35});
    CHECK(s == State::Hold);
    const double d = vnorm(vsub(p.p, {-0.15, -0.1, -0.35}));
    CHECK(std::fabs(d - 0.08) < 1e-9);
    CHECK(p.p[0] < -0.15);
    const P far_head{QID, {0.0, 0.0, -2.0}};
    const P q = left.get_with(2100000000, far_head, std::nullopt).second;
    CHECK(vnorm(vsub(q.p, left.shoulder(far_head))) <= 0.75 + 1e-9);
}

TEST(a_150_ms_miss_stays_optical_with_the_new_window_and_flips_with_the_old) {
    for (auto [cfg, want] : {std::pair{noimu(), State::Optical}, std::pair{legacy(noimu()), State::Hold}}) {
        Fuser fu(0, cfg, std::nullopt);
        for (std::uint64_t k = 0; k < 10; k++) {
            fix_at(fu, 1000 + 40 * k, 0.0);
        }
        CHECK(fu.get((1000 + 360 + 150) * 1000000ull, P::id()).first == want);
    }
}

TEST(a_lone_fix_after_a_hold_is_not_believed_two_in_a_row_are) {
    Fuser fu(0, noimu(), std::nullopt);
    for (std::uint64_t k = 0; k < 10; k++) {
        fix_at(fu, 1000 + 40 * k, 0.0);
    }
    CHECK(std::string(fix_at(fu, 2400, 0.0)) == "confirm");
    CHECK(fu.get(2410000000, P::id()).first == State::Hold);
    CHECK(std::string(fix_at(fu, 2800, 0.0)) == "confirm");
    CHECK(std::string(fix_at(fu, 2840, 0.0)) == "ok");
    CHECK(fu.get(2850000000, P::id()).first == State::Optical);
    CHECK(std::string(fix_at(fu, 2880, 0.0)) == "ok");
    CHECK(std::string(fix_at(fu, 4000, 0.0)) == "confirm");
    CHECK(std::string(fix_at(fu, 4040, 0.3)) == "confirm");
}

TEST(the_kill_switch_restores_single_fix_reentry) {
    Fuser fu(0, legacy(noimu()), std::nullopt);
    for (std::uint64_t k = 0; k < 10; k++) {
        fix_at(fu, 1000 + 40 * k, 0.0);
    }
    CHECK(std::string(fix_at(fu, 2400, 0.0)) == "ok");
}

TEST(few_led_and_high_residual_fixes_get_less_weight_when_smoothing) {
    CHECK(Fuser::gain(4, 0.5f, true) < Fuser::gain(6, 0.5f, true));
    CHECK(Fuser::gain(12, 3.0f, true) < Fuser::gain(12, 2.0f, true));
    CHECK(Fuser::gain(12, 2.0f, true) < Fuser::gain(12, 1.0f, true));
    CHECK(Fuser::gain(12, 0.5f, false) == 0.85);
}

TEST(forearm_model_swings_the_held_controller_around_the_elbow) {
    const Calib cal = default_calib(0);
    Config base;
    base.forearm = V3{0.0, 0.0, -0.3};
    base.follow_yaw = 1.0;
    base.still_lock = false;
    const Config cfg = legacy(base);
    const P head = P::id();
    const auto drive = [&](Fuser& fu) {
        std::uint64_t t = 1000000000;
        std::uint32_t dev = 0;
        for (std::uint64_t k = 0; k < 1000; k++) {
            const V3 w = (t >= 1500000000 && t < 2500000000ull) ? V3{std::numbers::pi / 2, 0.0, 0.0} : V3{0.0, 0.0, 0.0};
            fu.sample(t, dev, raw_for(cal, w));
            if (k % 20 == 0 && t < 1500000000) {
                fu.optical(t, P{QID, {-0.1, -0.2, -0.4}}, head, 0.5f, 12);
            }
            t += 2000000;
            dev += 2000;
        }
    };
    Fuser fu(0, cfg, cal);
    drive(fu);
    auto [s, p] = fu.get(2600000000ull, head);
    CHECK(s == State::Hold);
    CHECK(std::fabs(p.p[1] - 0.1) < 0.03 && std::fabs(p.p[2] + 0.1) < 0.03);
    Config c2 = cfg;
    c2.forearm = std::nullopt;
    c2.still_lock = false;
    Fuser old(0, c2, cal);
    drive(old);
    const P q = old.get(2600000000ull, head).second;
    CHECK(vnorm(vsub(q.p, {-0.1, -0.2, -0.4})) < 0.01);
}

TEST(the_hold_follows_half_the_head_yaw_by_default) {
    const Config cfg = noimu();
    CHECK(cfg.follow_yaw == 0.5);
    Fuser fu(1, cfg, std::nullopt);
    const P head = P::id();
    for (std::uint64_t k = 0; k < 10; k++) {
        fu.optical(1000000000 + k * 40000000, P{QID, {0.0, 0.0, -0.4}}, head, 0.5f, 10);
    }
    (void)fu.get(1700000000, head);
    const double s = std::numbers::pi / 4;
    const P turned{{0.0, std::sin(s), 0.0, std::cos(s)}, {0.0, 0.0, 0.0}};
    auto [st, p] = fu.get(1800000000, turned);
    CHECK(st == State::Hold);
    const double a = to_degrees(std::atan2(-p.p[0], -p.p[2]));
    CHECK(std::fabs(a - 45.0) < 1.0);
}

TEST(a_big_reacquire_correction_slides_at_most_half_a_metre_per_second) {
    Fuser fu(0, legacy(noimu()), std::nullopt);
    const P head = P::id();
    fu.optical(1000000000, P{QID, {-0.1, 0.0, -0.4}}, head, 0.5f, 10);
    fu.optical(1040000000, P{QID, {-0.1, 0.0, -0.4}}, head, 0.5f, 10);
    (void)fu.get(1500000000, head);
    fu.optical(2000000000, P{QID, {-0.3, 0.0, -0.4}}, head, 0.5f, 10);
    const std::uint64_t q0 = 2050000000;
    const P a = fu.get(q0, head).second;
    const P b = fu.get(q0 + 200000000, head).second;
    const P c = fu.get(q0 + 410000000, head).second;
    CHECK(std::fabs(a.p[0] + 0.1) < 0.01);
    CHECK(std::fabs(b.p[0] + 0.2) < 0.015);
    CHECK(std::fabs(c.p[0] + 0.3) < 0.005);
}

TEST(built_in_calibration_matches_the_fitted_axes_and_scale) {
    for (auto [h, axes] : {std::pair<std::size_t, const char*>{0, "-z -x +y"}, {1, "+z +x +y"}}) {
        const Calib c = default_calib(h);
        CHECK(c.axes() == axes);
        CHECK(std::fabs(c.scale_dps_per_lsb() - 0.122) < 0.002);
        CHECK(c.axis_misalignment_deg() < 12.0);
    }
    const auto c = calib_or_default(tmp("pimax-ctrl-imu-none-"), 1);
    CHECK(c && c->c == default_calib(1).c);
}

TEST(a_flipped_few_led_solve_keeps_its_position_but_not_its_orientation) {
    Config cfg;
    cfg.use_imu = true;
    Fuser fu(0, legacy(cfg), std::nullopt);
    const P head = P::id();
    std::uint64_t t = 1000000000;
    std::uint32_t dev = 0;
    int k = 0;
    while (t < 2000000000) {
        fu.sample(t, dev, {0, 0, 0});
        if (k % 20 == 0 && t < 1500000000) {
            fu.optical(t, P{QID, {-0.1, -0.1, -0.35}}, head, 0.5f, 12);
        }
        t += 2000000;
        dev += 2000;
        k++;
    }
    const P flipped{{0.0, 1.0, 0.0, 0.0}, {-0.12, -0.1, -0.35}};
    CHECK(std::string(fu.optical_with(2000000000, flipped, head, 0.8f, 6, std::nullopt)) == "pos-only");
    (void)fu.get(2050000000, head);
    const P p = fu.get(2300000000, head).second;
    CHECK(to_degrees(qangle(p.q, QID)) < 1.0);
    CHECK(std::fabs(p.p[0] + 0.12) < 0.01);
    std::string last;
    for (std::uint64_t i = 0; i < 3; i++) {
        last = fu.optical_with(2440000000 + i * 40000000, flipped, head, 0.5f, 12, std::nullopt);
    }
    CHECK(last == "ok");
}

PIBRIDGE_TEST_MAIN()
