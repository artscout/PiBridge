// SPDX-License-Identifier: MIT
// Text exactly as the Rust code prints it. The C ABI's error messages, the tool's output and the JSON files must be
// byte-identical to omarchyvr's pimax-cam (the difftest compares them), so the few Rust formatting rules the port
// needs are reproduced here: `{:?}` and `{}` of f64 (shortest round-trip digits), `{:.N}` with Rust's NaN spelling,
// `{:?}` of strings and arrays, and lossy UTF-8 decoding.
#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::cam::rfmt {

/// Rust `format!("{:?}", x)` of an f64: shortest digits that round-trip, decimal with at least one fraction digit
/// ("1.0", "0.0001") for 1e-4 <= |x| < 1e16 or 0, otherwise exponential ("1e16", "-4.10994e-5"); "NaN", "inf".
std::string debug_f64(double x);

/// Rust `format!("{}", x)` of an f64: shortest digits, never exponential ("3", "0.0000001", "-0"); "NaN", "inf".
std::string display_f64(double x);

/// Rust `format!("{:.prec}", x)` (with `{:+.prec}` when `plus`): exact decimal rounding (half to even, as
/// std::format and Rust both do); "NaN" / "inf" / "-inf" (NaN never gets a sign).
std::string fixed(double x, int prec, bool plus = false);

/// `fixed` right-aligned in `width` columns (Rust `{:width.prec}` for a number).
std::string fixed_w(double x, int width, int prec, bool plus = false);

/// Rust `format!("{:?}", s)` of a &str: quoted, with `\"`, `\\`, `\n`, `\r`, `\t`, `\0` and `\u{..}` for the other
/// ASCII control characters (non-ASCII is printed as is).
std::string debug_str(std::string_view s);

/// Rust `{:?}` of a slice of f64: "[a, b, c]".
std::string debug_f64s(std::span<const double> v);

/// Rust `{:?}` of a slice of u16: "[1, 2, 3, 4]".
std::string debug_u16s(std::span<const std::uint16_t> v);

/// `String::from_utf8_lossy`: every maximal invalid sequence becomes U+FFFD.
std::string utf8_lossy(std::string_view bytes);

/// True when `bytes` is valid UTF-8 (`std::str::from_utf8(..).is_ok()`).
bool utf8_valid(std::string_view bytes);

/// Rust `str::trim`: strips Unicode White_Space (the ASCII ones, U+0085, U+00A0, U+1680, U+2000..U+200A, U+2028,
/// U+2029, U+202F, U+205F, U+3000) at both ends.
std::string_view trim(std::string_view s);

/// Rust `str::split_whitespace`.
std::vector<std::string_view> split_whitespace(std::string_view s);

/// Rust `s.parse::<f64>()`: optional sign, digits with an optional point, optional exponent, or inf/infinity/nan
/// (any case). The whole string must match. false on error.
bool parse_f64(std::string_view s, double& out);

/// Rust `s.parse::<u32>()`: optional '+', decimal digits, no overflow.
bool parse_u32(std::string_view s, std::uint32_t& out);

/// Rust `s.parse::<u64>()`.
bool parse_u64(std::string_view s, std::uint64_t& out);

/// Rust `x as u32` / `x as u16` / `x as u64` / `x as usize` for an f64: saturating, NaN -> 0.
std::uint32_t f64_to_u32(double x);
std::uint64_t f64_to_u64(double x);

/// Rust `f64::to_radians` / `to_degrees` (multiplication by the folded constant, as the Rust core library does).
double to_radians(double deg);
double to_degrees(double rad);

/// Rust `f64::clamp` (NaN stays NaN).
double clamp(double x, double lo, double hi);

/// Rust `f64::min` / `f64::max`: the other operand when one is NaN.
double fmin(double a, double b);
double fmax(double a, double b);

} // namespace pibridge::cam::rfmt
