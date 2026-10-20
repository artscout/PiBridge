// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-hands/src/pose.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/pose.h"

#include "pibridge/cam/rustfmt.h"

#include <cmath>

namespace pibridge::ctrl {

namespace {

V3d add(V3d a, V3d b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
V3d sub(V3d a, V3d b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
V3d scale(V3d a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
double dot(V3d a, V3d b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
V3d cross(V3d a, V3d b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
V3d norm(V3d a) {
    const double l = std::sqrt(dot(a, a));
    return l > 1e-12 ? scale(a, 1.0 / l) : a;
}
V3d mirror_x(V3d v) { return {-v[0], v[1], v[2]}; }

} // namespace

V3d Pose::rotate(V3d v) const {
    const V3d u = {q[0], q[1], q[2]};
    const double w = q[3];
    const V3d t = scale(cross(u, v), 2.0);
    return add(add(v, scale(t, w)), cross(u, t));
}

Pose Pose::compose(const Pose& o) const {
    const double ax = q[0], ay = q[1], az = q[2], aw = q[3];
    const double bx = o.q[0], by = o.q[1], bz = o.q[2], bw = o.q[3];
    const std::array<double, 4> r = {
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    };
    const double n = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3]);
    return Pose{{r[0] / n, r[1] / n, r[2] / n, r[3] / n}, add(p, rotate(o.p))};
}

Pose Pose::inverse() const {
    const Pose qi{{-q[0], -q[1], -q[2], q[3]}, {0.0, 0.0, 0.0}};
    return Pose{qi.q, scale(qi.rotate(p), -1.0)};
}

Pose Pose::from_axes(V3d x, V3d y, V3d z, V3d p) {
    const double m[3][3] = {{x[0], y[0], z[0]}, {x[1], y[1], z[1]}, {x[2], y[2], z[2]}};
    const double tr = m[0][0] + m[1][1] + m[2][2];
    std::array<double, 4> q;
    if (tr > 0.0) {
        const double s = std::sqrt(tr + 1.0) * 2.0;
        q = {(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s};
    } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
        const double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
        q = {0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s};
    } else if (m[1][1] > m[2][2]) {
        const double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
        q = {(m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s};
    } else {
        const double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
        q = {(m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s};
    }
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    return Pose{{q[0] / n, q[1] / n, q[2] / n, q[3] / n}, p};
}

double Pose::angle_to(const Pose& other) const {
    double d = -0.0; // Iterator::sum of f64 starts at -0.0
    for (int i = 0; i < 4; i++) {
        d += q[i] * other.q[i];
    }
    return 2.0 * std::acos(pibridge::cam::rfmt::fmin(std::fabs(d), 1.0));
}

Pose grip_from_joints(const Pose& palm, V3d index_prox, V3d ring_prox, Hand hand) {
    const V3d origin = add(palm.p, palm.rotate({0.0, -GRIP_OFFSET_FROM_PALM_M, 0.0}));
    const V3d plus_z = norm(sub(ring_prox, index_prox));
    const double side = hand == Hand::Right ? 1.0 : -1.0;
    const V3d want_x = palm.rotate({0.0, side, 0.0});
    const V3d plus_x = norm(sub(want_x, scale(plus_z, dot(want_x, plus_z))));
    const V3d plus_y = cross(plus_z, plus_x);
    return Pose::from_axes(plus_x, plus_y, plus_z, origin);
}

Pose grip_in_wrist(Hand hand) {
    const auto m = [hand](V3d v) { return hand == Hand::Right ? mirror_x(v) : v; };
    return grip_from_joints(Pose::from_position(m(PALM_LEFT)), m(INDEX_PROXIMAL_LEFT), m(RING_PROXIMAL_LEFT), hand);
}

Pose grip_to_wrist(Hand hand) {
    return grip_in_wrist(hand).inverse();
}

} // namespace pibridge::ctrl
