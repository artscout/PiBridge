// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-hands/src/touch.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/touch.h"

#include "pibridge/cam/rustfmt.h"

#include <cmath>

namespace pibridge::ctrl::touch {

std::array<double, 4> pitch(double deg) {
    const double h = pibridge::cam::rfmt::to_radians(deg) * 0.5;
    return {std::sin(h), 0.0, 0.0, std::cos(h)};
}

namespace {
V3d mirror_x(V3d p, bool left) {
    return left ? p : V3d{-p[0], p[1], p[2]};
}
} // namespace

Pose openxr_grip(bool left) {
    return Pose{pitch(OPENXR_GRIP_PITCH_DEG), mirror_x(OPENXR_GRIP_ORIGIN, left)};
}

Pose openxr_aim(bool left) {
    return Pose{pitch(OPENXR_AIM_PITCH_DEG), mirror_x(OPENXR_AIM_ORIGIN, left)};
}

Pose aim_to_grip() {
    return openxr_aim(true).inverse().compose(openxr_grip(true));
}

} // namespace pibridge::ctrl::touch
