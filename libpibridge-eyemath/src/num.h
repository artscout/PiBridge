// SPDX-License-Identifier: MIT
// Number formatting and parsing that behave like Rust's `format!("{x:.N}")` and `str::parse::<f64>()`, independent
// of the locale (std::to_chars / std::from_chars), plus the few Rust float semantics the port relies on
// (`f64::max`, `f64::clamp`, saturating `as` casts, `to_degrees`). Internal to libpibridge-eyemath.
#pragma once

#include <cmath>
#include <concepts>
#include <cstdint>
#include <expected>
#include <limits>
#include <numbers>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::eyemath::detail {

/// `format!("{x:.prec}")`: fixed notation with `prec` decimals, correctly rounded; "NaN", "inf", "-inf" as Rust.
std::string fmt_fixed(double x, int prec);

/// `s.parse::<f64>()`: the whole string must be a number (optional sign, digits, point, exponent, "inf",
/// "infinity", "nan" in any case). Out-of-range literals give +-inf or +-0 like Rust. The error text is Rust's.
std::expected<double, std::string> parse_f64(std::string_view s);

/// The whole file as text; the error is "<io error> (os error N)" like Rust's `io::Error`.
std::expected<std::string, std::string> read_text(const char* path);

/// Write `text` to `path` through a temporary file in the same directory and a rename (readers never see half a
/// file). Same error text as read_text.
std::expected<void, std::string> write_text_atomic(const char* path, std::string_view text);

/// Rust `str::trim` for the ASCII whitespace the files can hold.
std::string_view trim(std::string_view s);

/// Rust `str::lines`: split at '\n', drop one trailing '\r' per line, no empty last line after a final '\n'.
std::vector<std::string_view> lines(std::string_view s);

/// Rust `str::trim_matches('"')` followed by `trim`.
std::string unquote(std::string_view s);

/// Rust `f64::to_degrees` / `f64::to_radians` (same constant order, so the same rounding).
inline double to_degrees(double x) { return x * (180.0 / std::numbers::pi); }
inline double to_radians(double x) { return x * (std::numbers::pi / 180.0); }

/// Rust `f64::max` / `f64::min`: a NaN operand is ignored.
inline double rmax(double a, double b) { return std::fmax(a, b); }
inline double rmin(double a, double b) { return std::fmin(a, b); }

/// Rust `f64::clamp`: NaN stays NaN.
inline double rclamp(double x, double lo, double hi) {
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

/// Rust `x as I` for a float: saturating, NaN gives 0 (a plain C++ cast is undefined out of range).
template <std::integral I>
I sat_cast(double x) {
    if (std::isnan(x)) {
        return 0;
    }
    constexpr double lo = static_cast<double>(std::numeric_limits<I>::min());
    // 2^bits as a double: every value below it fits
    constexpr double hi = static_cast<double>(std::numeric_limits<I>::max()) + 1.0;
    if (x <= lo) {
        return std::numeric_limits<I>::min();
    }
    if (x >= hi) {
        return std::numeric_limits<I>::max();
    }
    return static_cast<I>(x);
}

/// Rust `a.wrapping_sub(b)` for i64 (release builds wrap; C++ signed overflow would be undefined).
inline std::int64_t wrapping_sub(std::int64_t a, std::int64_t b) {
    return static_cast<std::int64_t>(static_cast<std::uint64_t>(a) - static_cast<std::uint64_t>(b));
}

} // namespace pibridge::eyemath::detail
