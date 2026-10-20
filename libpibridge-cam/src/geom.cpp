// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/geom.rs (MIT OR Apache-2.0).
#include "pibridge/cam/geom.h"

#include "pibridge/cam/rustfmt.h"

#include <cmath>

namespace pibridge::cam::geom {

double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

double norm(const Vec3& a) {
    return std::sqrt(dot(a, a));
}

Vec3 normalize(const Vec3& a) {
    const double n = norm(a);
    if (n == 0.0) {
        return a;
    }
    return scale(a, 1.0 / n);
}

Vec3 scale(const Vec3& a, double s) {
    return {a[0] * s, a[1] * s, a[2] * s};
}

Vec3 add(const Vec3& a, const Vec3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

Vec3 sub(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

Vec3 mat_vec(const Mat3& m, const Vec3& v) {
    return {dot(m[0], v), dot(m[1], v), dot(m[2], v)};
}

Mat3 mat_mul(const Mat3& a, const Mat3& b) {
    Mat3 r{};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            r[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    return r;
}

Mat3 transpose(const Mat3& m) {
    return {{{m[0][0], m[1][0], m[2][0]}, {m[0][1], m[1][1], m[2][1]}, {m[0][2], m[1][2], m[2][2]}}};
}

Vec3 col(const Mat3& m, int j) {
    return {m[0][j], m[1][j], m[2][j]};
}

Mat3 from_cols(const Vec3& a, const Vec3& b, const Vec3& c) {
    return {{{a[0], b[0], c[0]}, {a[1], b[1], c[1]}, {a[2], b[2], c[2]}}};
}

double det(const Mat3& m) {
    return dot(m[0], cross(m[1], m[2]));
}

double orthonormality_error(const Mat3& m) {
    const Mat3 p = mat_mul(transpose(m), m);
    double e = 0.0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            e = rfmt::fmax(e, std::fabs(p[i][j] - (i == j ? 1.0 : 0.0)));
        }
    }
    return e;
}

Mat3 rodrigues(const Vec3& w) {
    const double th = norm(w);
    if (th < 1e-12) {
        return IDENTITY;
    }
    const Vec3 k = scale(w, 1.0 / th);
    const double s = std::sin(th);
    const double c = std::cos(th);
    const Mat3 kk{{{0.0, -k[2], k[1]}, {k[2], 0.0, -k[0]}, {-k[1], k[0], 0.0}}};
    const Mat3 kk2 = mat_mul(kk, kk);
    Mat3 r = IDENTITY;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            r[i][j] += s * kk[i][j] + (1.0 - c) * kk2[i][j];
        }
    }
    return r;
}

Mat3 rot_x(double deg) {
    const double a = rfmt::to_radians(deg);
    const double s = std::sin(a);
    const double c = std::cos(a);
    return {{{1.0, 0.0, 0.0}, {0.0, c, -s}, {0.0, s, c}}};
}

double angle_deg(const Vec3& a, const Vec3& b) {
    return rfmt::to_degrees(std::acos(rfmt::clamp(dot(normalize(a), normalize(b)), -1.0, 1.0)));
}

std::array<double, 4> quat_from_mat(const Mat3& m) {
    const double tr = m[0][0] + m[1][1] + m[2][2];
    double x = 0;
    double y = 0;
    double z = 0;
    double w = 0;
    if (tr > 0.0) {
        const double s = std::sqrt(tr + 1.0) * 2.0;
        w = 0.25 * s;
        x = (m[2][1] - m[1][2]) / s;
        y = (m[0][2] - m[2][0]) / s;
        z = (m[1][0] - m[0][1]) / s;
    } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
        const double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
        w = (m[2][1] - m[1][2]) / s;
        x = 0.25 * s;
        y = (m[0][1] + m[1][0]) / s;
        z = (m[0][2] + m[2][0]) / s;
    } else if (m[1][1] > m[2][2]) {
        const double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
        w = (m[0][2] - m[2][0]) / s;
        x = (m[0][1] + m[1][0]) / s;
        y = 0.25 * s;
        z = (m[1][2] + m[2][1]) / s;
    } else {
        const double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
        w = (m[1][0] - m[0][1]) / s;
        x = (m[0][2] + m[2][0]) / s;
        y = (m[1][2] + m[2][1]) / s;
        z = 0.25 * s;
    }
    const double n = std::sqrt(x * x + y * y + z * z + w * w);
    return {x / n, y / n, z / n, w / n};
}

Pose Pose::inverse() const {
    const Mat3 rt = transpose(r);
    return Pose{rt, scale(mat_vec(rt, t), -1.0)};
}

Pose Pose::compose(const Pose& other) const {
    return Pose{mat_mul(r, other.r), add(mat_vec(r, other.t), t)};
}

} // namespace pibridge::cam::geom
