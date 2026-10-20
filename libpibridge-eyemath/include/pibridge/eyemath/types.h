// SPDX-License-Identifier: MIT
// Small value types shared by the gaze math, and the error type of the fallible functions.
#pragma once

#include <array>
#include <string>

namespace pibridge::eyemath {

/// A 3D vector in double precision (millimetres, metres or a unit direction, as the field says).
using Vec3 = std::array<double, 3>;
/// A 3D vector in single precision (what the head-frame samples carry, like the Rust `[f32; 3]`).
using Vec3f = std::array<float, 3>;
/// A 2D vector in double precision.
using Vec2 = std::array<double, 2>;

/// A gaze tangent `(u, v) = (x / -z, y / -z)` of a head-frame direction (OpenXR axes, -z forward). The Rust code
/// uses a `(f64, f64)` tuple for it.
struct Tangent {
    double u = 0.0;
    double v = 0.0;
    friend bool operator==(const Tangent&, const Tangent&) = default;
};

/// Error of the parsing and loading functions: a message in plain words (the Rust code returns a `String`).
struct Error {
    std::string message;
};

} // namespace pibridge::eyemath
