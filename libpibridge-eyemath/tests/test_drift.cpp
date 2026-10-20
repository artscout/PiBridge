// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/drift.rs tests (MIT OR Apache-2.0).
#include <cmath>
#include <numbers>

#include "pibridge/eyemath/calib.h"
#include "pibridge/eyemath/drift.h"
#include "pibridge_test.h"

using namespace pibridge::eyemath;

namespace {
double rad(double deg) { return deg * (std::numbers::pi / 180.0); }
} // namespace

TEST(converges_rejects_outliers_and_is_bounded) {
    const DriftParams p;
    Drift d;
    const Tangent slip{std::tan(rad(1.5)), std::tan(rad(-0.8))};
    for (int i = 0; i < 20; i++) {
        const Tangent gaze{0.1 - slip.u, 0.05 - slip.v};
        CHECK(d.observe(p, 0, gaze, {0.1, 0.05}));
    }
    const auto uv = d.apply(0, {0.1 - slip.u, 0.05 - slip.v});
    CHECK(tangent_angle_deg(uv, {0.1, 0.05}) < 0.01);
    CHECK((d.offset[1] == Tangent{0.0, 0.0})); // the other eye is untouched
    // a glance elsewhere (10 deg) is not a drift
    CHECK(!d.observe(p, 0, {0.3, 0.05}, {0.1, 0.05}));
    CHECK(d.rejected[0] == 1);
    // bounded
    Drift d2;
    DriftParams big = p;
    big.max_residual_deg = 30.0;
    big.gain = 1.0;
    d2.observe(big, 1, {0.0, 0.0}, {0.2, 0.0});
    const double l =
        std::atan(std::sqrt(d2.offset[1].u * d2.offset[1].u + d2.offset[1].v * d2.offset[1].v)) * (180.0 / std::numbers::pi);
    CHECK_NEAR(l, p.max_offset_deg, 0.01);
}

PIBRIDGE_TEST_MAIN()
