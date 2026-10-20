// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/filter.rs tests (MIT OR Apache-2.0).
#include <cmath>
#include <numbers>
#include <vector>

#include "pibridge/eyemath/calib.h"
#include "pibridge/eyemath/filter.h"
#include "pibridge/eyemath/rng.h"
#include "pibridge_test.h"

using namespace pibridge::eyemath;

namespace {
double rad(double deg) { return deg * (std::numbers::pi / 180.0); }
double deg(double r) { return r * (180.0 / std::numbers::pi); }
} // namespace

TEST(fixation_noise_is_reduced_about_five_times) {
    const FilterParams p;
    OneEuro f;
    Rng rng(99);
    const double sigma = rad(1.0);
    std::vector<double> raw;
    std::vector<double> out;
    for (int i = 0; i < 1200; i++) {
        const double t = i / 120.0;
        // the Rust array literal evaluates the two gauss() calls in order
        const double gx = rng.gauss();
        const double gy = rng.gauss();
        const Vec2 x{0.1 + sigma * gx, -0.05 + sigma * gy};
        const auto y = f.filter(p, t, x);
        if (i > 240) {
            raw.push_back(deg(std::sqrt((x[0] - 0.1) * (x[0] - 0.1) + (x[1] + 0.05) * (x[1] + 0.05))));
            out.push_back(deg(std::sqrt((y[0] - 0.1) * (y[0] - 0.1) + (y[1] + 0.05) * (y[1] + 0.05))));
        }
    }
    const double r = stats(raw).mean;
    const double o = stats(out).mean;
    CHECK(o < r / 4.0);
}

TEST(saccade_lands_without_lag_after_the_snap) {
    const FilterParams p;
    OneEuro f;
    const Vec2 a{0.0, 0.0};
    const Vec2 b{std::tan(rad(10.0)), 0.0};
    for (int i = 0; i < 120; i++) {
        f.filter(p, i / 120.0, a);
    }
    const double t0 = 1.0;
    const auto y1 = f.filter(p, t0, b);
    const auto y2 = f.filter(p, t0 + 1.0 / 120.0, b);
    CHECK(y1[0] < b[0]); // first sample far away is still filtered
    CHECK(y2 == b);      // second sample far away snaps
    // small moves (below the snap) are followed smoothly and converge within ~0.5 s
    const Vec2 c{b[0] + rad(1.0), 0.0};
    auto y = y2;
    for (int i = 2; i < 62; i++) {
        y = f.filter(p, t0 + i / 120.0, c);
    }
    CHECK(deg(std::fabs(y[0] - c[0])) < 0.1);
}

TEST(gaps_and_missing_gaze_restart_the_filter) {
    const FilterParams p;
    OneEuro f;
    f.filter(p, 0.0, {0.0, 0.0});
    f.filter(p, 1.0 / 120.0, {0.0, 0.0});
    const auto y = f.filter(p, 1.0, {0.2, 0.0});
    CHECK((y == Vec2{0.2, 0.0})); // a 1 s gap is a restart
    Sample s;
    filter_sample(p, f, s);
    CHECK(!s.gaze.has_value());
    FilterParams off = p;
    off.enabled = false;
    Sample s2;
    s2.gaze = Gaze{{0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, -1.0F}, 3};
    filter_sample(off, f, s2);
    CHECK((s2.gaze->direction == Vec3f{0.0F, 0.0F, -1.0F}));
}

PIBRIDGE_TEST_MAIN()
