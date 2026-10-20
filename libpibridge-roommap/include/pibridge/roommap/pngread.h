// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-room/src/pngread.rs (MIT OR Apache-2.0).
//
// Minimal PNG reader for the 8-bit greyscale frames of EuRoC recordings (non-interlaced; colour type 0, or 2/6
// converted to grey with (77 R + 150 G + 29 B) >> 8). zlib inflates (the Rust code uses miniz_oxide: the pixels are
// the same, only the text of an inflate error differs).
#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <string>
#include <vector>

namespace pibridge::roommap::pngread {

struct Grey {
    std::size_t w = 0;
    std::size_t h = 0;
    /// Row-major, w bytes per row.
    std::vector<std::uint8_t> data;
    bool operator==(const Grey&) const = default;
};

/// Decode PNG bytes.
std::expected<Grey, std::string> decode(std::span<const std::uint8_t> b);
/// Read a PNG file ("<path>: <error>" on failure).
std::expected<Grey, std::string> read(const std::string& path);

} // namespace pibridge::roommap::pngread
