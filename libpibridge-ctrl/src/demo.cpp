// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-hands/src/demo.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/demo.h"

#include <cmath>

namespace pibridge::ctrl {

namespace {

/// 0 -> 1 -> 0 triangle over [start, start + len], 0 elsewhere.
float tri(double t, double start, double len) {
    if (t < start || t > start + len) {
        return 0.0f;
    }
    const double x = (t - start) / len;
    return static_cast<float>(1.0 - std::fabs(2.0 * x - 1.0));
}

/// f64::rem_euclid.
double rem_euclid(double a, double b) {
    const double r = std::fmod(a, b);
    return r < 0.0 ? r + std::fabs(b) : r;
}

} // namespace

Input demo_input(std::uint64_t now_ns, bool right) {
    const double t = static_cast<double>(now_ns) * 1e-9 + (right ? DEMO_RIGHT_OFFSET_S : 0.0);
    const double c = rem_euclid(t, DEMO_PERIOD_S);
    Input i;
    i.grip = tri(c, 1.0, 2.0);
    i.trigger = tri(c, 3.0, 2.0);
    if (c >= 5.0 && c < 6.0) {
        i.flags |= flags::TOUCH_BTN1;
    } else if (c >= 6.0 && c < 7.0) {
        i.flags |= flags::TOUCH_BTN2;
        if (c >= 6.5) {
            i.flags |= flags::CLICK_BTN2;
        }
    } else if (c >= 7.0 && c < 8.0) {
        i.flags |= flags::TOUCH_STICK;
        i.stick_x = 0.6f;
    }
    return i;
}

} // namespace pibridge::ctrl
