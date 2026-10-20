// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-lenstest/src/hudquad.rs and pimax-lens/src/font.rs (MIT OR Apache-2.0).
//
// The head-locked text (instructions, results) as an `XrCompositionLayerQuad` in VIEW space. Text baked into each
// eye's image lands at the same image position in both eyes, which with canted displays are different directions:
// the wearer sees it double. A quad layer is placed by the compositor in both eyes consistently. This is the pure
// part: the 5x7 bitmap font, where the quad hangs, how big it is, and the texture (the text on a transparent
// background). The OpenXR/Vulkan plumbing is in xrapp.cpp.
#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pibridge::eyecal::hud {

// ---------------------------------------------------------------- the font (digits, upper case, punctuation)

inline constexpr std::uint32_t GLYPH_W = 5;
inline constexpr std::uint32_t GLYPH_H = 7;

/// True if the pixel (`col` 0..5, `row` 0..7) of the glyph for code point `c` is set. Lower case is drawn as upper
/// case; unknown characters are drawn as a box.
bool pixel(char32_t c, std::uint32_t col, std::uint32_t row);
/// The code points of a UTF-8 string (invalid bytes become U+FFFD).
std::u32string decode(std::string_view utf8);
/// Size in pixels of `text` drawn at `scale`: one pixel column of spacing between glyphs.
std::pair<std::uint32_t, std::uint32_t> text_size(std::string_view text, std::uint32_t scale);
/// The characters the font has (for the tests).
std::vector<char32_t> glyph_chars();

// ---------------------------------------------------------------- the quad

/// Quad texture size in pixels (1 mm per pixel at the default quad size).
inline constexpr std::uint32_t TEX_W = 2048;
inline constexpr std::uint32_t TEX_H = 1024;
/// Largest text scale (pixels per font pixel); smaller when the text does not fit.
inline constexpr std::uint32_t MAX_SCALE = 5;

/// Placement of the quad in VIEW space (metres; -z is forward).
struct QuadSpec {
    /// Distance straight ahead of the head.
    float distance = 2.0F;
    float width = 2.048F;
    float height = 1.024F;
    /// How far below the view centre the top edge of the quad is.
    float top_below = 0.25F;

    /// Centre of the quad in VIEW space (x right, y up, z backwards).
    std::array<float, 3> position() const { return {0.0F, -(top_below + 0.5F * height), -distance}; }
    /// Visual angle in degrees of `metres` on the quad, in the middle.
    float angle_deg(float metres) const;
    /// Visual angle of one font pixel at text `scale` for a texture of `tex_w` pixels across the quad.
    float font_pixel_deg(std::uint32_t scale, std::uint32_t tex_w) const;
};

/// 2 m ahead, 2.048 x 1.024 m, top edge 0.25 m (7 degrees) below the centre of view.
inline constexpr QuadSpec SPEC{};

/// Largest scale (at most MAX_SCALE) at which all `lines` fit in a `w` x `h` texture.
std::uint32_t fit_scale(std::span<const std::string> lines, std::uint32_t w, std::uint32_t h);

/// An RGB image of panel codes (bytes, rows top to bottom).
struct Image {
    std::uint32_t w = 0;
    std::uint32_t h = 0;
    std::vector<std::uint8_t> rgb;
    std::array<std::uint8_t, 3> get(std::uint32_t x, std::uint32_t y) const;
    /// Pixels that are not pure black.
    std::size_t lit() const;
};

/// The texture: the lines centred, top-anchored, on near-black backdrops; everything else exactly black (made
/// transparent by alpha_from_black).
Image render(std::span<const std::string> lines);

/// How the compositor treats the bytes of the swapchain format: an `*_SRGB` swapchain shows the stored code, an
/// `*_UNORM` one wants the decoded (linear) value.
enum class Encoding { Srgb, Linear };

/// Fill `out` (`w * h * 4` bytes) with the swapchain bytes: panel codes through `enc`, optionally BGRA order,
/// alpha 255.
void to_upload(const Image& img, Encoding enc, bool bgra, std::span<std::uint8_t> out);

/// Set the alpha of 4-byte pixels: 0 where the colour is exactly black, 255 elsewhere (works for RGBA and BGRA).
void alpha_from_black(std::span<std::uint8_t> px);

} // namespace pibridge::eyecal::hud
