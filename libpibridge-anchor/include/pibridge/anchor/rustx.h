// SPDX-License-Identifier: MIT
// The Rust semantics the room-anchor and room-map ports need on top of libpibridge-cam's rustfmt.h: `str::lines`,
// `split_once`, `trim_matches`, `Path::with_extension` / `with_file_name` for the plain file names these tools use,
// `powi` as LLVM expands it, `f64::total_cmp`, the Display texts of Rust's number parse errors, and what a panic in
// a Rust tool does (message on stderr, exit 101).
#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pibridge::anchor::rs {

/// `str::lines`: split at "\n" (a "\r\n" ending loses its "\r"); no empty last line after a final "\n".
std::vector<std::string_view> lines(std::string_view s);

/// `str::split_once(c)`.
std::optional<std::pair<std::string_view, std::string_view>> split_once(std::string_view s, char c);

/// `str::trim_matches(|c| set.contains(c))` for ASCII characters.
std::string_view trim_matches(std::string_view s, std::string_view set);

/// `str::trim_start_matches` / `trim_end_matches` for one ASCII character.
std::string_view trim_start_char(std::string_view s, char c);
std::string_view trim_end_char(std::string_view s, char c);

/// `Path::file_name` of a plain path ("a/b.c" -> "b.c"; trailing "/" ignored); "" when there is none.
std::string file_name(const std::string& path);

/// `Path::with_file_name` of a plain path ("d/x" -> "d/name", "x" -> "name").
std::string with_file_name(const std::string& path, std::string_view name);

/// `Path::with_extension` of a plain path: the part of the file name after its last '.' (not a leading one) is
/// replaced by `ext` ("" removes it with the dot).
std::string with_extension(const std::string& path, std::string_view ext);

/// `f64::powi(n)`: repeated squaring as LLVM expands it (and compiler-rt's `__powidf2`): r *= a for every set bit
/// from the lowest, a *= a in between; 1 / r for a negative exponent.
double powi(double a, int n);
/// `f32::powi(n)`.
float powif(float a, int n);

/// `f64::total_cmp`: -1, 0, 1 (-NaN < -inf < ... < -0 < +0 < ... < inf < NaN).
int total_cmp(double a, double b);
inline bool total_less(double a, double b) { return total_cmp(a, b) < 0; }

/// Display of `ParseFloatError` for `s` (call only when parsing failed): "cannot parse float from empty string" or
/// "invalid float literal".
std::string float_error(std::string_view s);

/// `s.parse::<u32>()`, with the Display of `ParseIntError` on failure.
std::optional<std::uint32_t> parse_u32(std::string_view s, std::string* err = nullptr);
/// `s.parse::<u64>()`.
std::optional<std::uint64_t> parse_u64(std::string_view s, std::string* err = nullptr);
/// `s.parse::<i32>()`.
std::optional<std::int32_t> parse_i32(std::string_view s, std::string* err = nullptr);
/// `s.parse::<i64>()`.
std::optional<std::int64_t> parse_i64(std::string_view s, std::string* err = nullptr);
/// `s.parse::<usize>()`.
std::optional<std::size_t> parse_usize(std::string_view s, std::string* err = nullptr);
/// `s.parse::<f64>()`, with the Display of `ParseFloatError` on failure.
std::optional<double> parse_f64(std::string_view s, std::string* err = nullptr);

/// A Rust panic in a tool: "thread 'main' panicked at <where>:\n<msg>\nnote: ..." on stderr, exit code 101.
[[noreturn]] void panic(std::string_view where, std::string_view msg);

/// `std::process::id()`.
std::uint32_t process_id();

/// CLOCK_MONOTONIC now (ns).
std::uint64_t monotonic_ns();

/// Rust `x as u8` / `as i32` / `as usize` / ... for an f32 or f64: saturating, NaN -> 0 (inline: hot loops).
namespace detail {
template <class T>
inline T sat_cast(double x) {
    if (x != x) {
        return 0;
    }
    constexpr double lo = static_cast<double>(std::numeric_limits<T>::min());
    constexpr double hi = static_cast<double>(std::numeric_limits<T>::max()) + 1.0;
    if (x <= lo) {
        return std::numeric_limits<T>::min();
    }
    if (x >= hi) {
        return std::numeric_limits<T>::max();
    }
    return static_cast<T>(x);
}
} // namespace detail
inline std::uint8_t to_u8(double x) { return detail::sat_cast<std::uint8_t>(x); }
inline std::int32_t to_i32(double x) { return detail::sat_cast<std::int32_t>(x); }
inline std::int64_t to_i64(double x) {
    if (x != x) {
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
inline std::size_t to_usize(double x) {
    if (!(x > 0.0)) {
        return 0;
    }
    if (x >= 18446744073709551616.0) {
        return std::numeric_limits<std::size_t>::max();
    }
    return static_cast<std::size_t>(x);
}
inline std::uint32_t to_u32(double x) { return detail::sat_cast<std::uint32_t>(x); }
inline std::uint64_t to_u64(double x) { return static_cast<std::uint64_t>(to_usize(x)); }
inline std::uint16_t to_u16(double x) { return detail::sat_cast<std::uint16_t>(x); }
inline std::int16_t to_i16(double x) { return detail::sat_cast<std::int16_t>(x); }
inline std::int8_t to_i8(double x) { return detail::sat_cast<std::int8_t>(x); }
/// `x as u8` for an f32 (the hot pixel path).
inline std::uint8_t to_u8f(float x) {
    if (!(x > 0.0f)) {
        return 0;
    }
    if (x >= 255.0f) {
        return 255;
    }
    return static_cast<std::uint8_t>(x);
}
/// `f32::round` (half away from zero) without a libm call for the usual range; std::round otherwise. The sign of a
/// zero result may differ (use it where the result is cast to an integer).
inline float roundf_fast(float x) {
    if (x >= 0.0f && x < 8388608.0f) {
        const float t = static_cast<float>(static_cast<std::int32_t>(x));
        return (x - t >= 0.5f) ? t + 1.0f : t;
    }
    if (x < 0.0f && x > -8388608.0f) {
        const float t = static_cast<float>(static_cast<std::int32_t>(x));
        return (t - x >= 0.5f) ? t - 1.0f : t;
    }
    return __builtin_roundf(x);
}
/// `f64::round` the same way (2^52 range).
inline double round_fast(double x) {
    if (x >= 0.0 && x < 4503599627370496.0) {
        const double t = static_cast<double>(static_cast<std::int64_t>(x));
        return (x - t >= 0.5) ? t + 1.0 : t;
    }
    if (x < 0.0 && x > -4503599627370496.0) {
        const double t = static_cast<double>(static_cast<std::int64_t>(x));
        return (t - x >= 0.5) ? t - 1.0 : t;
    }
    return __builtin_round(x);
}

} // namespace pibridge::anchor::rs
