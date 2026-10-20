// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-hands/src/touch.rs (MIT OR Apache-2.0).
#include "pibridge/cam/rustfmt.h"
#include "pibridge/ctrl/rest.h"
#include "pibridge/ctrl/touch.h"
#include "pibridge_test.h"

#include <cmath>

using namespace pibridge::ctrl;
using pibridge::cam::rfmt::to_degrees;
using pibridge::cam::rfmt::to_radians;

namespace {
bool close(V3d a, V3d b, double tol) {
    for (int i = 0; i < 3; i++) {
        if (!(std::fabs(a[i] - b[i]) < tol)) {
            return false;
        }
    }
    return true;
}
} // namespace

TEST(aim_to_grip_is_sixty_degrees_and_the_handle_offset) {
    const Pose t = touch::aim_to_grip();
    const V3d f = forward(t);
    CHECK(close(f, {0.0, std::sin(to_radians(60.0)), -std::cos(to_radians(60.0))}, 1e-9));
    CHECK(t.rotate({1.0, 0.0, 0.0})[0] > 1.0 - 1e-12);
    CHECK(close(t.p, {0.0, -0.0300, 0.0950}, 5e-4));
    const Pose r = touch::openxr_aim(false).inverse().compose(touch::openxr_grip(false));
    CHECK(r.angle_to(t) < 1e-12 && close(r.p, t.p, 1e-12));
}

TEST(steamvr_derived_aim_equals_the_driver_aim) {
    const double a[3] = {0.3, -0.5, 0.2};
    const double n = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    const double h = 0.7;
    const Pose aim{{a[0] / n * std::sin(h), a[1] / n * std::sin(h), a[2] / n * std::sin(h), std::cos(h)}, {0.1, 1.2, -0.4}};
    for (bool left : {true, false}) {
        const Pose grip = aim.compose(touch::aim_to_grip());
        const Pose raw = grip.compose(touch::openxr_grip(left).inverse());
        const Pose steam_aim = raw.compose(touch::openxr_aim(left));
        CHECK(steam_aim.angle_to(aim) < 1e-9);
        CHECK(close(steam_aim.p, aim.p, 1e-9));
    }
}

TEST(the_old_rule_was_fifteen_degrees_off_in_steamvr) {
    const Pose aim = Pose::identity();
    const Pose old_grip = aim.compose(Pose{touch::pitch(45.0), {0.0, 0.0, 0.0}});
    const Pose steam_aim = old_grip.compose(touch::openxr_grip(true).inverse()).compose(touch::openxr_aim(true));
    const double deg = to_degrees(steam_aim.angle_to(aim));
    CHECK(std::fabs(deg - 15.0) < 1e-6);
    const V3d d = steam_aim.p;
    CHECK(std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) > 0.09);
}

PIBRIDGE_TEST_MAIN()
