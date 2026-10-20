// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/main.rs (MIT OR Apache-2.0): what the commands share. The Rust tool
// ends with a panic (exit 101) on bad numbers and missing arguments; these helpers print the same message lines
// (the panic location differs: "panicked at pibridge-ctrltrack").
#pragma once

#include "pibridge/cam/io.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace pibridge::ctrltool {

/// Print a Rust-style panic message on stderr and exit 101.
[[noreturn]] void panic(std::string_view msg);

/// `a[i]` with Rust's index panic.
const std::string& at(std::span<const std::string> a, std::size_t i);

/// `s.parse::<usize>().unwrap()`.
std::size_t parse_usize(std::string_view s);
/// `s.parse::<f64>().unwrap()`.
double parse_f64(std::string_view s);
/// `s.parse::<f64>().ok()`.
std::optional<double> parse_f64_ok(std::string_view s);

/// Rust `{:?}` of an io::Error (`Os { code: 2, kind: NotFound, message: "..." }` / `Custom { kind: .., error: ".." }`).
std::string io_debug(const cam::IoError& e);

/// Rust `{}` of an io::Error.
inline std::string io_text(const cam::IoError& e) { return e.text(); }

/// One line on stderr.
void eprintln(std::string_view s);
/// One line on stdout (line-buffered like Rust's stdout).
void println(std::string_view s);

/// PNG of an RGB image with pimax-cam's encoder layout (every row filtered with Sub, zlib level 6; the deflate bytes
/// come from zlib instead of miniz_oxide, so only the decoded pixels match the Rust file).
cam::Result<void> write_png_rgb(const std::string& path, std::size_t width, std::size_t height,
                                std::span<const std::uint8_t> rgb);

/// std::thread::available_parallelism (cgroup CPU quota and the affinity mask), or 4.
std::size_t available_parallelism();

int cmd_track(std::span<const std::string> a);
int cmd_live(std::span<const std::string> a);
int cmd_show(std::span<const std::string> a);
int cmd_health(std::span<const std::string> a);

} // namespace pibridge::ctrltool
