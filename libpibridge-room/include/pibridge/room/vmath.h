// SPDX-License-Identifier: MIT
// Ported from ZoneTweak vmath.go and mat4.go (MIT, Copyright (c) 2026 killdano; see NOTICE).
//
// 3D helpers for the in-headset editor: vectors, a 3x4 pose matrix as OpenVR's HmdMatrix34 (columns 0..2 are the
// x, y, z axes, column 3 the position; -z is forward), quaternions from OpenXR, and the per-eye matrices for Vulkan
// (clip y down, depth 0..1).
#pragma once

#include "pibridge/room/geom.h"

#include <array>
#include <cmath>

namespace pibridge::room {

struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3 operator+(Vec3 b) const { return {x + b.x, y + b.y, z + b.z}; }
    Vec3 operator-(Vec3 b) const { return {x - b.x, y - b.y, z - b.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    double dot(Vec3 b) const { return x * b.x + y * b.y + z * b.z; }
    Vec3 cross(Vec3 b) const { return {y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x}; }
    double length() const { return std::sqrt(dot(*this)); }
    Vec3 norm() const {
        const double l = length();
        return l < 1e-9 ? *this : *this * (1 / l);
    }
    friend bool operator==(const Vec3&, const Vec3&) = default;
};

/// A point on the floor plane at height y.
inline Vec3 fl(Pt2 p, double y) { return {p.x, y, p.z}; }
inline Pt2 flat(Vec3 v) { return {v.x, v.z}; }

struct Rgba {
    float r = 1, g = 1, b = 1, a = 1;
    Rgba alpha(float v) const { return {r, g, b, v}; }
};

struct Mat34 {
    std::array<std::array<double, 4>, 3> m{{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}}};
    Vec3 pos() const { return {m[0][3], m[1][3], m[2][3]}; }
    Vec3 axis(int i) const { return {m[0][i], m[1][i], m[2][i]}; }
    Vec3 xform(Vec3 p) const { return axis(0) * p.x + axis(1) * p.y + axis(2) * p.z + pos(); }
    static Mat34 identity() { return {}; }
};

/// Pose from an OpenXR quaternion (x, y, z, w) and position.
Mat34 poseMatrix(double qx, double qy, double qz, double qw, Vec3 p);
Mat34 inv34(const Mat34& m);
Mat34 mul34(const Mat34& a, const Mat34& b);

/// Column-major 4x4 (as GLSL reads it).
using Mat4 = std::array<float, 16>;
Mat4 mat4From34(const Mat34& m);
Mat4 mul4(const Mat4& a, const Mat4& b);
/// Vulkan projection from the four OpenXR fov angles (radians; left and down negative).
Mat4 projectionVulkan(double angleLeft, double angleRight, double angleUp, double angleDown, double nearZ, double farZ);
/// Apply a column-major 4x4 to a point (w = 1); returns clip coordinates.
std::array<double, 4> transform4(const Mat4& m, Vec3 p);

} // namespace pibridge::room
