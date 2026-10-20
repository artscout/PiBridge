// SPDX-License-Identifier: MIT
// PiBridge's fuser improvements (docs/research/ctrl-tracking-improvements.md): inertial position, the hold kept past
// lost_s, the gravity check, the inertial jump gate. A synthetic controller moving at constant velocity.
#include "pibridge/ctrl/fuse.h"
#include "pibridge_test.h"

#include <cmath>

using namespace pibridge::ctrl::fuse;

namespace {

constexpr double LSB = 0.1207 * M_PI / 180.0;

Calib calib() {
    Calib c;
    c.c = {{{LSB, 0, 0}, {0, LSB, 0}, {0, 0, LSB}}};
    c.bias = {0, 0, 0};
    c.pairs = 1000;
    c.moving = 500;
    c.r2 = 0.95;
    return c;
}

/// Runs the hand along x at v m/s (level, no rotation); optical fixes every 33 ms until cut_ns, then none. Returns
/// the shown pose at t_ns.
std::pair<State, P> run(Config cfg, double v, std::uint64_t cut_ns, std::uint64_t t_ns, Fuser** keep = nullptr) {
    cfg.online_fit = false;
    cfg.use_imu = true;
    static Fuser* f = nullptr;
    delete f;
    f = new Fuser(1, cfg, calib());
    const P head = P::id();
    std::uint32_t dev = 0;
    std::uint64_t next_fix = 100'000'000;
    for (std::uint64_t t = 100'000'000; t <= t_ns; t += 2'000'000) {
        dev += 2000;
        // level and still: gravity only (1024 LSB = 1 g along +y)
        f->sample_acc(t, dev, {0, 0, 0}, std::array<std::int16_t, 3>{0, 1024, 0});
        if (t >= next_fix + 20'000'000 && next_fix < cut_ns) {
            const double x = 0.1 + v * static_cast<double>(next_fix) * 1e-9;
            f->optical(next_fix, P{{0, 0, 0, 1}, {x, -0.3, -0.35}}, head, 0.8f, 12);
            next_fix += 33'000'000;
        }
    }
    if (keep != nullptr) {
        *keep = f;
    }
    return f->get(t_ns, head);
}

} // namespace

TEST(defaults_are_rust) {
    const Config r;
    CHECK(r.ins_s == 0.0 && !r.tilt_fix && r.keep_tau_s == 0.0 && r.grav_check_deg == 0.0 && r.ins_jump_m == 0.0);
    const Config f = Config::pibridge();
    CHECK(f.ins_s == 0.0 && f.tilt_fix && f.keep_tau_s > 0.0 && f.grav_check_deg > 0.0 && f.ins_jump_m > 0.0);
}

TEST(kept_past_lost_s) {
    // 5 s after the last fix: the Rust fuser gives up (None, Monado's rest pose); PiBridge keeps a held pose gliding
    // towards the rest pose
    const auto [s0, p0] = run(Config{}, 0.0, 1'000'000'000, 6'500'000'000);
    CHECK(s0 == State::None);
    const auto [s1, p1] = run(Config::pibridge(), 0.0, 1'000'000'000, 6'500'000'000);
    CHECK(s1 == State::Hold);
    const auto [s2, p2] = run(Config::pibridge(), 0.0, 1'000'000'000, 30'000'000'000);
    CHECK(s2 == State::Hold);
    CHECK(std::fabs(p2.p[0] - 0.15) < 0.01 && std::fabs(p2.p[1] + 0.16) < 0.01 && std::fabs(p2.p[2] + 0.38) < 0.01);
    (void)p0;
    (void)p1;
}

Config with_ins() {
    Config c = Config::pibridge();
    c.ins_s = 0.6; // the inertial position (off by default since the real takes)
    return c;
}

TEST(still_hand_stays_put) {
    // the inertial position of a hand at rest does not drift away out of view
    const auto [s, p] = run(with_ins(), 0.0, 1'000'000'000, 1'500'000'000);
    CHECK(s == State::Hold);
    CHECK(std::fabs(p.p[0] - 0.1) < 0.01 && std::fabs(p.p[1] + 0.3) < 0.01 && std::fabs(p.p[2] + 0.35) < 0.01);
}

TEST(gravity_check_drops_a_tilted_fix) {
    Fuser* f = nullptr;
    run(Config::pibridge(), 0.0, 3'000'000'000, 2'000'000'000, &f);
    // the next fix says the controller is rolled 60 deg while the accelerometer reads level
    const double a = 60.0 * M_PI / 180.0;
    const Q roll = {0.0, 0.0, std::sin(a / 2), std::cos(a / 2)};
    const char* v = f->optical_with(1'990'000'000, P{roll, {0.1, -0.3, -0.35}}, P::id(), 0.8f, 12, std::nullopt);
    CHECK(std::string_view(v) == "tilt");
}

TEST(inertial_jump_gate) {
    // a fix 10 cm off the inertial prediction (the other hand) one frame after the last good one is dropped
    Fuser* f = nullptr;
    run(with_ins(), 0.0, 3'000'000'000, 2'000'000'000, &f);
    const char* v =
        f->optical_with(1'985'000'000, P{{0, 0, 0, 1}, {0.2, -0.3, -0.35}}, P::id(), 0.8f, 12, std::nullopt);
    CHECK(std::string_view(v) == "jump");
}

PIBRIDGE_TEST_MAIN()
