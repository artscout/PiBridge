// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-lens/src/font.rs (MIT OR Apache-2.0).
//
// A small 5x7 bitmap font (digits, upper case letters, common punctuation, the degree sign) for labels in debug
// images. Lower case is drawn as upper case; unknown characters are drawn as a box. Text is UTF-8 (one glyph per
// code point, as Rust's `chars()`).
#pragma once

#include <cstdint>
#include <functional>
#include <string_view>
#include <utility>

namespace pibridge::lens::font {

inline constexpr std::uint32_t GLYPH_W = 5;
inline constexpr std::uint32_t GLYPH_H = 7;

/// True if the pixel (`col` 0..5, `row` 0..7) of the glyph for code point `c` is set.
bool pixel(char32_t c, std::uint32_t col, std::uint32_t row);
/// Size in pixels of `text` drawn at `scale`: one pixel column of spacing between glyphs.
std::pair<std::uint32_t, std::uint32_t> text_size(std::string_view text, std::uint32_t scale);
/// Call `plot(x, y)` for every set pixel of `text` whose top-left corner is at (`x0`, `y0`).
void draw_text(std::string_view text, std::int32_t x0, std::int32_t y0, std::uint32_t scale,
               const std::function<void(std::int32_t, std::int32_t)>& plot);

namespace detail {
/// The code points of a UTF-8 string (invalid bytes as U+FFFD).
std::u32string chars(std::string_view s);
/// Every glyph (code point and its 7 rows), for the table tests.
struct Glyph {
    char32_t c;
    const char* rows[7];
};
extern const Glyph GLYPHS[];
extern const std::size_t GLYPH_COUNT;
} // namespace detail

} // namespace pibridge::lens::font
