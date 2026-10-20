// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack (MIT OR Apache-2.0): the few Rust semantics the tracker port needs on top of
// libpibridge-cam's rustfmt.h (saturating float -> integer casts, Rust's `max_by` tie rule).
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace pibridge::ctrl::track::rs {

/// Rust `x as i64` for an f64: truncation toward zero, saturating at the type's range, NaN -> 0.
inline std::int64_t f64_to_i64(double x) {
    if (std::isnan(x)) {
        return 0;
    }
    if (x <= -9223372036854775808.0) {
        return std::numeric_limits<std::int64_t>::min();
    }
    if (x >= 9223372036854775808.0) {
        return std::numeric_limits<std::int64_t>::max();
    }
    return static_cast<std::int64_t>(x);
}

/// Rust `x as usize` (64-bit) for an f64: saturating, negative and NaN -> 0.
inline std::size_t f64_to_usize(double x) {
    if (!(x > 0.0)) {
        return 0;
    }
    if (x >= 18446744073709551616.0) {
        return std::numeric_limits<std::size_t>::max();
    }
    return static_cast<std::size_t>(x);
}

/// Rust `f64::round` (half away from zero).
inline double round(double x) { return std::round(x); }

} // namespace pibridge::ctrl::track::rs
