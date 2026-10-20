// SPDX-License-Identifier: MIT
// The few Rust semantics the pimax-hands port needs beyond libpibridge-cam's rustfmt.h: f32 clamp / min / max, the
// float `as` casts, `{:e}` of an f64, Unicode White_Space for `char::is_whitespace`, `Path::with_extension`, and what
// a Rust panic inside an `extern "C"` function does (the process aborts).
#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::ctrl::rs {

/// A Rust panic inside the C ABI: Rust prints the message and aborts the process (a panic cannot unwind out of an
/// `extern "C"` function). The C++ port does the same where the Rust code would panic (NaN in a `partial_cmp().unwrap()`).
[[noreturn]] void panic(const char* what) noexcept;

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

/// Rust `f32::min` / `f32::max`: the other operand when one is NaN.
float fminf(float a, float b);
float fmaxf(float a, float b);

/// Rust `x as i16` for an f64 / f32: saturating, NaN -> 0.
std::int16_t f64_to_i16(double x);
/// Rust `x as i64` for an f64: saturating, NaN -> 0.
std::int64_t f64_to_i64(double x);
/// Rust `x as usize` for an f64.
std::size_t f64_to_usize(double x);

/// Rust `format!("{:e}", x)`: shortest round-trip digits in exponential form ("1e0", "2.67478985236365e-4",
/// "-6.4e-5", "0e0", "NaN", "inf").
std::string lower_exp(double x);

/// Rust `char::is_whitespace` for the UTF-8 character starting at s[i]: its byte length, 0 when it is not one.
std::size_t ws_len(std::string_view s, std::size_t i);

/// Rust `Path::with_extension` on a string path.
std::string with_extension(const std::string& path, std::string_view ext);

/// Rust `i64::abs` in a release build (wrapping: i64::MIN stays i64::MIN).
inline std::int64_t wabs(std::int64_t v) {
    return v < 0 ? static_cast<std::int64_t>(0 - static_cast<std::uint64_t>(v)) : v;
}

/// Wrapping i64 addition / subtraction (release-mode Rust arithmetic).
inline std::int64_t wadd(std::int64_t a, std::int64_t b) {
    return static_cast<std::int64_t>(static_cast<std::uint64_t>(a) + static_cast<std::uint64_t>(b));
}
inline std::int64_t wsub(std::int64_t a, std::int64_t b) {
    return static_cast<std::int64_t>(static_cast<std::uint64_t>(a) - static_cast<std::uint64_t>(b));
}

/// u64 saturating_sub / saturating_add.
inline std::uint64_t sat_sub(std::uint64_t a, std::uint64_t b) {
    return a > b ? a - b : 0;
}
inline std::uint64_t sat_add(std::uint64_t a, std::uint64_t b) {
    const std::uint64_t s = a + b;
    return s < a ? UINT64_MAX : s;
}

/// The std::process::id() of this process.
std::uint32_t process_id();

} // namespace pibridge::ctrl::rs
