// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-hands/src/rest.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/rest.h"

#include "pibridge/cam/rustfmt.h"

#include <cmath>
#include <vector>

namespace pibridge::ctrl {

namespace rf = pibridge::cam::rfmt;

namespace {

V3d sub(V3d a, V3d b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
V3d scale(V3d a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
V3d cross(V3d a, V3d b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
double len(V3d a) { return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); }

bool in(double x, double lo, double hi) { return lo <= x && x <= hi; }

/// Orientation whose -z looks along `dir`, +y as close to world up as possible.
Pose look_along(V3d dir, V3d at) {
    const double l = len(dir);
    if (l < 1e-9) {
        return Pose::from_position(at);
    }
    const V3d z = scale(dir, -1.0 / l);
    V3d x = cross({0.0, 1.0, 0.0}, z);
    if (len(x) < 1e-6) {
        x = {1.0, 0.0, 0.0};
    }
    x = scale(x, 1.0 / len(x));
    const V3d y = cross(z, x);
    return Pose::from_axes(x, y, z, at);
}

} // namespace

std::expected<Rest, std::string> Rest::parse(std::string_view s) {
    const std::string_view t = rf::trim(s);
    if (t.empty() || t == "desk" || t == "default") {
        return desk();
    }
    if (t == "legacy") {
        return legacy();
    }
    std::vector<double> v;
    std::size_t start = 0;
    for (;;) {
        const auto comma = t.find(',', start);
        const std::string_view part = t.substr(start, comma == std::string_view::npos ? std::string_view::npos : comma - start);
        double x = 0.0;
        if (!rf::parse_f64(rf::trim(part), x)) {
            return std::unexpected("PIMAX_CTRL_REST=" + rf::debug_str(t) + ": want desk, legacy or side,down,fwd[,pitch_deg]");
        }
        v.push_back(x);
        if (comma == std::string_view::npos) {
            break;
        }
        start = comma + 1;
    }
    Rest r{};
    if (v.size() == 3) {
        r = Rest{v[0], v[1], v[2], desk().grip_pitch_deg};
    } else if (v.size() == 4) {
        r = Rest{v[0], v[1], v[2], v[3]};
    } else {
        return std::unexpected("PIMAX_CTRL_REST=" + rf::debug_str(t) + ": want 3 or 4 numbers");
    }
    const auto sane = [](double x, double lo, double hi) { return std::isfinite(x) && in(x, lo, hi); };
    if (!(sane(r.side, -1.0, 1.0) && sane(r.down, -1.0, 1.5) && sane(r.fwd, -0.5, 1.5) &&
          sane(r.grip_pitch_deg, -180.0, 180.0))) {
        return std::unexpected("PIMAX_CTRL_REST=" + rf::debug_str(t) + ": out of range (|m| <= ~1, pitch in +-180)");
    }
    return r;
}

V3d Rest::position(Hand hand) const {
    const double s = hand == Hand::Left ? -side : side;
    return {s, -down, -fwd};
}

Pose Rest::aim_to_grip() const {
    const double h = rf::to_radians(grip_pitch_deg) * 0.5;
    return Pose{{std::sin(h), 0.0, 0.0, std::cos(h)}, {0.0, 0.0, 0.0}};
}

std::expected<AimMode, std::string> AimMode::parse(std::string_view s) {
    const std::string_view t = rf::trim(s);
    if (t.empty() || t == "level") {
        return level();
    }
    if (t == "head") {
        return head_at(DEFAULT_HEAD_DISTANCE);
    }
    if (t.starts_with("head:")) {
        double d = 0.0;
        if (rf::parse_f64(rf::trim(t.substr(5)), d) && std::isfinite(d) && in(d, 0.2, 20.0)) {
            return head_at(d);
        }
    }
    return std::unexpected("PIMAX_CTRL_AIM=" + rf::debug_str(t) + ": want level, head or head:<0.2..20 m>");
}

Pose yaw_of(std::array<double, 4> head_q) {
    const Pose head{head_q, {0.0, 0.0, 0.0}};
    const V3d f = head.rotate({0.0, 0.0, -1.0});
    if (std::fabs(f[0]) + std::fabs(f[2]) < 1e-3) {
        return Pose::identity();
    }
    const double yaw = std::atan2(-f[0], -f[2]);
    const double h = yaw * 0.5;
    return Pose{{0.0, std::sin(h), 0.0, std::cos(h)}, {0.0, 0.0, 0.0}};
}

std::pair<Pose, Pose> controller_poses(Hand hand, const Rest& rest, AimMode aim, std::array<double, 4> head_q) {
    const Pose yaw = yaw_of(head_q);
    const V3d pos = yaw.rotate(rest.position(hand));
    Pose aim_pose;
    if (!aim.head) {
        aim_pose = Pose{yaw.q, pos};
    } else {
        const Pose head{head_q, {0.0, 0.0, 0.0}};
        const V3d target = scale(head.rotate({0.0, 0.0, -1.0}), aim.distance);
        aim_pose = look_along(sub(target, pos), pos);
    }
    const Pose level{yaw.q, pos};
    const Pose grip = level.compose(rest.aim_to_grip());
    return {aim_pose, grip};
}

V3d forward(const Pose& p) {
    return p.rotate({0.0, 0.0, -1.0});
}

std::pair<double, double> view_angles(std::array<double, 4> head_q, V3d point) {
    const Pose head{head_q, {0.0, 0.0, 0.0}};
    const V3d v = head.inverse().rotate(point);
    const double fwd = -v[2];
    return {rf::to_degrees(std::atan2(v[0], fwd)), rf::to_degrees(std::atan2(-v[1], fwd))};
}

} // namespace pibridge::ctrl
