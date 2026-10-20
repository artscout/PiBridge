// SPDX-License-Identifier: MIT
// Ported from ZoneTweak vmath.go and mat4.go (MIT, Copyright (c) 2026 killdano; see NOTICE).
#include "pibridge/room/vmath.h"

namespace pibridge::room {

Mat34 poseMatrix(double qx, double qy, double qz, double qw, Vec3 p) {
    Mat34 r;
    const double xx = qx * qx, yy = qy * qy, zz = qz * qz;
    const double xy = qx * qy, xz = qx * qz, yz = qy * qz;
    const double wx = qw * qx, wy = qw * qy, wz = qw * qz;
    r.m[0] = {1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy), p.x};
    r.m[1] = {2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx), p.y};
    r.m[2] = {2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy), p.z};
    return r;
}

Mat34 inv34(const Mat34& m) {
    Mat34 r;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            r.m[i][j] = m.m[j][i];
        }
    }
    for (int i = 0; i < 3; i++) {
        r.m[i][3] = -(r.m[i][0] * m.m[0][3] + r.m[i][1] * m.m[1][3] + r.m[i][2] * m.m[2][3]);
    }
    return r;
}

Mat34 mul34(const Mat34& a, const Mat34& b) {
    Mat34 r;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            double s = 0;
            for (int k = 0; k < 3; k++) {
                s += a.m[i][k] * b.m[k][j];
            }
            if (j == 3) {
                s += a.m[i][3];
            }
            r.m[i][j] = s;
        }
    }
    return r;
}

Mat4 mat4From34(const Mat34& m) {
    const auto f = [](double v) { return static_cast<float>(v); };
    return {f(m.m[0][0]), f(m.m[1][0]), f(m.m[2][0]), 0, f(m.m[0][1]), f(m.m[1][1]), f(m.m[2][1]), 0,
            f(m.m[0][2]), f(m.m[1][2]), f(m.m[2][2]), 0, f(m.m[0][3]), f(m.m[1][3]), f(m.m[2][3]), 1};
}

Mat4 mul4(const Mat4& a, const Mat4& b) {
    Mat4 r{};
    for (int c = 0; c < 4; c++) {
        for (int rr = 0; rr < 4; rr++) {
            float s = 0;
            for (int k = 0; k < 4; k++) {
                s += a[static_cast<std::size_t>(k * 4 + rr)] * b[static_cast<std::size_t>(c * 4 + k)];
            }
            r[static_cast<std::size_t>(c * 4 + rr)] = s;
        }
    }
    return r;
}

Mat4 projectionVulkan(double angleLeft, double angleRight, double angleUp, double angleDown, double nearZ, double farZ) {
    // OpenXR's view space: x right, y up, -z forward. Vulkan clip: y down, depth 0..1.
    const double l = std::tan(angleLeft);
    const double r = std::tan(angleRight);
    const double u = std::tan(angleUp);
    const double d = std::tan(angleDown);
    const double w = r - l;
    const double h = d - u; // y flipped
    Mat4 m{};
    m[0] = static_cast<float>(2 / w);
    m[5] = static_cast<float>(2 / h);
    m[8] = static_cast<float>((r + l) / w);
    m[9] = static_cast<float>((u + d) / h);
    m[10] = static_cast<float>(-farZ / (farZ - nearZ));
    m[11] = -1;
    m[14] = static_cast<float>(-(farZ * nearZ) / (farZ - nearZ));
    return m;
}

std::array<double, 4> transform4(const Mat4& m, Vec3 p) {
    std::array<double, 4> o{};
    for (int r = 0; r < 4; r++) {
        o[static_cast<std::size_t>(r)] = m[static_cast<std::size_t>(r)] * p.x + m[static_cast<std::size_t>(4 + r)] * p.y +
                                         m[static_cast<std::size_t>(8 + r)] * p.z + m[static_cast<std::size_t>(12 + r)];
    }
    return o;
}

} // namespace pibridge::room
