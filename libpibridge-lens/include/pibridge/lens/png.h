// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-lens/src/png.rs (MIT OR Apache-2.0).
//
// Minimal PNG writer (8-bit grey / RGB / RGBA, no interlacing) for debug images, zlib for the deflate stream (the
// Rust code used miniz_oxide: the compressed bytes differ, the pixels are the same). Rows are filtered with None,
// Sub or Up, whichever has the smallest sum of absolute values.
#pragma once

#include <cstdint>
#include <expected>
#include <span>
#include <string>
#include <vector>

namespace pibridge::lens::png {

/// CRC-32 as used by PNG (ISO 3309), over `data`.
std::uint32_t crc32(std::span<const std::uint8_t> data);

/// Encode `data` (`width * height * channels` bytes, rows top to bottom) as a PNG file image. Throws
/// std::invalid_argument("pixel buffer size") on a wrong buffer size or channel count (the Rust code panics).
std::vector<std::uint8_t> encode(std::uint32_t width, std::uint32_t height, std::size_t channels,
                                 std::span<const std::uint8_t> data);
std::vector<std::uint8_t> encode_rgb8(std::uint32_t width, std::uint32_t height, std::span<const std::uint8_t> rgb);
/// std::fs::write of the encoded image; the error is the Rust io::Error text.
std::expected<void, std::string> write_rgb8(const std::string& path, std::uint32_t width, std::uint32_t height,
                                            std::span<const std::uint8_t> rgb);

/// Decoded image (only what this writer produces: 8-bit, filters 0..=2, no interlace).
struct Decoded {
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::size_t channels = 0;
    std::vector<std::uint8_t> data;
    bool operator==(const Decoded&) const = default;
};
std::expected<Decoded, std::string> decode(std::span<const std::uint8_t> png);

} // namespace pibridge::lens::png
