// SPDX-License-Identifier: MIT
// The few Rust rules for f32 the lens code needs on top of pibridge/cam/rustfmt.h (which covers f64 and strings):
// `{}` of an f32 (shortest f32 digits, never exponential) and the saturating `as` casts from f32/f64 to integers.
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace pibridge::lens::fmt {

/// Rust `format!("{}", x)` of an f32: shortest digits that round-trip as f32, never exponential ("1", "0.01",
/// "-0", "NaN", "inf").
std::string display_f32(float x);

/// Rust `x as T` for a float x and an integer T: saturating, NaN -> 0, truncation toward zero.
template <class T>
T sat_cast(double x) {
    if (std::isnan(x)) {
        return 0;
    }
    constexpr double lo = static_cast<double>(std::numeric_limits<T>::min());
    // max() + 1 is a power of two, exact as a double
    constexpr double hi_excl = static_cast<double>(std::numeric_limits<T>::max() / 2 + 1) * 2.0;
    if (x <= lo) {
        return std::numeric_limits<T>::min();
    }
    if (x >= hi_excl) {
        return std::numeric_limits<T>::max();
    }
    return static_cast<T>(std::trunc(x));
}

/// Rust `f32::to_degrees` / `to_radians` (multiplication by the f32 constants of the Rust core library).
inline float to_degrees(float x) {
    constexpr float k = 57.2957795130823208767981548141051703f;
    return x * k;
}
inline float to_radians(float x) {
    constexpr float k = 3.14159265358979323846264338327950288f / 180.0f;
    return x * k;
}

/// Rust `f32::clamp` (NaN stays NaN).
inline float clampf(float x, float lo, float hi) {
    if (x < lo) {
        x = lo;
    }
    if (x > hi) {
        x = hi;
    }
    return x;
}

/// Rust `f32::powi`: compiler-rt's __powisf2 (square and multiply in f32).
inline float powi(float a, int b) {
    const bool recip = b < 0;
    float r = 1.0f;
    while (true) {
        if ((b & 1) != 0) {
            r *= a;
        }
        b /= 2;
        if (b == 0) {
            break;
        }
        a *= a;
    }
    return recip ? 1.0f / r : r;
}

} // namespace pibridge::lens::fmt
