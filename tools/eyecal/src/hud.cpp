// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-lenstest/src/hudquad.rs and pimax-lens/src/font.rs (MIT OR Apache-2.0).
#include "hud.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace pibridge::eyecal::hud {

namespace {

struct Glyph {
    char32_t c;
    std::array<const char*, 7> rows;
};

constexpr std::array<const char*, 7> BOX = {"#####", "#...#", "#...#", "#...#", "#...#", "#...#", "#####"};

// clang-format off
constexpr Glyph GLYPHS[] = {
    {' ', {".....", ".....", ".....", ".....", ".....", ".....", "....."}},
    {'A', {".###.", "#...#", "#...#", "#####", "#...#", "#...#", "#...#"}},
    {'B', {"####.", "#...#", "#...#", "####.", "#...#", "#...#", "####."}},
    {'C', {".###.", "#...#", "#....", "#....", "#....", "#...#", ".###."}},
    {'D', {"####.", "#...#", "#...#", "#...#", "#...#", "#...#", "####."}},
    {'E', {"#####", "#....", "#....", "####.", "#....", "#....", "#####"}},
    {'F', {"#####", "#....", "#....", "####.", "#....", "#....", "#...."}},
    {'G', {".###.", "#...#", "#....", "#.###", "#...#", "#...#", ".####"}},
    {'H', {"#...#", "#...#", "#...#", "#####", "#...#", "#...#", "#...#"}},
    {'I', {".###.", "..#..", "..#..", "..#..", "..#..", "..#..", ".###."}},
    {'J', {"..###", "...#.", "...#.", "...#.", "...#.", "#..#.", ".##.."}},
    {'K', {"#...#", "#..#.", "#.#..", "##...", "#.#..", "#..#.", "#...#"}},
    {'L', {"#....", "#....", "#....", "#....", "#....", "#....", "#####"}},
    {'M', {"#...#", "##.##", "#.#.#", "#.#.#", "#...#", "#...#", "#...#"}},
    {'N', {"#...#", "##..#", "#.#.#", "#..##", "#...#", "#...#", "#...#"}},
    {'O', {".###.", "#...#", "#...#", "#...#", "#...#", "#...#", ".###."}},
    {'P', {"####.", "#...#", "#...#", "####.", "#....", "#....", "#...."}},
    {'Q', {".###.", "#...#", "#...#", "#...#", "#.#.#", "#..#.", ".##.#"}},
    {'R', {"####.", "#...#", "#...#", "####.", "#.#..", "#..#.", "#...#"}},
    {'S', {".####", "#....", "#....", ".###.", "....#", "....#", "####."}},
    {'T', {"#####", "..#..", "..#..", "..#..", "..#..", "..#..", "..#.."}},
    {'U', {"#...#", "#...#", "#...#", "#...#", "#...#", "#...#", ".###."}},
    {'V', {"#...#", "#...#", "#...#", "#...#", "#...#", ".#.#.", "..#.."}},
    {'W', {"#...#", "#...#", "#...#", "#.#.#", "#.#.#", "##.##", "#...#"}},
    {'X', {"#...#", "#...#", ".#.#.", "..#..", ".#.#.", "#...#", "#...#"}},
    {'Y', {"#...#", "#...#", ".#.#.", "..#..", "..#..", "..#..", "..#.."}},
    {'Z', {"#####", "....#", "...#.", "..#..", ".#...", "#....", "#####"}},
    {'0', {".###.", "#...#", "#..##", "#.#.#", "##..#", "#...#", ".###."}},
    {'1', {"..#..", ".##..", "..#..", "..#..", "..#..", "..#..", ".###."}},
    {'2', {".###.", "#...#", "....#", "...#.", "..#..", ".#...", "#####"}},
    {'3', {"####.", "....#", "....#", ".###.", "....#", "....#", "####."}},
    {'4', {"...#.", "..##.", ".#.#.", "#..#.", "#####", "...#.", "...#."}},
    {'5', {"#####", "#....", "####.", "....#", "....#", "#...#", ".###."}},
    {'6', {".###.", "#....", "#....", "####.", "#...#", "#...#", ".###."}},
    {'7', {"#####", "....#", "...#.", "..#..", ".#...", ".#...", ".#..."}},
    {'8', {".###.", "#...#", "#...#", ".###.", "#...#", "#...#", ".###."}},
    {'9', {".###.", "#...#", "#...#", ".####", "....#", "....#", ".###."}},
    {'.', {".....", ".....", ".....", ".....", ".....", "..##.", "..##."}},
    {',', {".....", ".....", ".....", ".....", "..##.", "..#..", ".#..."}},
    {':', {".....", "..##.", "..##.", ".....", "..##.", "..##.", "....."}},
    {';', {".....", "..##.", "..##.", ".....", "..##.", "..#..", ".#..."}},
    {'-', {".....", ".....", ".....", "#####", ".....", ".....", "....."}},
    {'+', {".....", "..#..", "..#..", "#####", "..#..", "..#..", "....."}},
    {'=', {".....", ".....", "#####", ".....", "#####", ".....", "....."}},
    {'/', {"....#", "....#", "...#.", "..#..", ".#...", "#....", "#...."}},
    {'\\', {"#....", "#....", ".#...", "..#..", "...#.", "....#", "....#"}},
    {'%', {"##..#", "##..#", "...#.", "..#..", ".#...", "#..##", "#..##"}},
    {'(', {"...#.", "..#..", ".#...", ".#...", ".#...", "..#..", "...#."}},
    {')', {".#...", "..#..", "...#.", "...#.", "...#.", "..#..", ".#..."}},
    {'[', {".###.", ".#...", ".#...", ".#...", ".#...", ".#...", ".###."}},
    {']', {".###.", "...#.", "...#.", "...#.", "...#.", "...#.", ".###."}},
    {'_', {".....", ".....", ".....", ".....", ".....", ".....", "#####"}},
    {'\'', {"..#..", "..#..", ".#...", ".....", ".....", ".....", "....."}},
    {'"', {".#.#.", ".#.#.", ".....", ".....", ".....", ".....", "....."}},
    {'!', {"..#..", "..#..", "..#..", "..#..", "..#..", ".....", "..#.."}},
    {'?', {".###.", "#...#", "....#", "...#.", "..#..", ".....", "..#.."}},
    {'<', {"...#.", "..#..", ".#...", "#....", ".#...", "..#..", "...#."}},
    {'>', {".#...", "..#..", "...#.", "....#", "...#.", "..#..", ".#..."}},
    {'*', {".....", "#.#.#", ".###.", "#####", ".###.", "#.#.#", "....."}},
    {'#', {".#.#.", ".#.#.", "#####", ".#.#.", "#####", ".#.#.", ".#.#."}},
    {'|', {"..#..", "..#..", "..#..", "..#..", "..#..", "..#..", "..#.."}},
    {'^', {"..#..", ".#.#.", "#...#", ".....", ".....", ".....", "....."}},
    {'~', {".....", ".....", ".##.#", "#.##.", ".....", ".....", "....."}},
    {0xb0, {".##..", "#..#.", "#..#.", ".##..", ".....", ".....", "....."}},
};
// clang-format on

const std::array<const char*, 7>& rows(char32_t c) {
    if (c >= U'a' && c <= U'z') {
        c = c - U'a' + U'A';
    }
    for (const auto& g : GLYPHS) {
        if (g.c == c) {
            return g.rows;
        }
    }
    return BOX;
}

constexpr float HUD_FG = 0.72F;
constexpr std::array<float, 3> HUD_BG = {0.01F, 0.01F, 0.01F};

std::uint8_t to_byte(float c) { return static_cast<std::uint8_t>(std::clamp(c, 0.0F, 1.0F) * 255.0F + 0.5F); }

float srgb_to_linear(float c) { return c <= 0.04045F ? c / 12.92F : std::pow((c + 0.055F) / 1.055F, 2.4F); }

} // namespace

bool pixel(char32_t c, std::uint32_t col, std::uint32_t row) {
    if (col >= GLYPH_W || row >= GLYPH_H) {
        return false;
    }
    return rows(c)[row][col] == '#';
}

std::vector<char32_t> glyph_chars() {
    std::vector<char32_t> v;
    for (const auto& g : GLYPHS) {
        v.push_back(g.c);
    }
    return v;
}

std::u32string decode(std::string_view s) {
    std::u32string out;
    for (std::size_t i = 0; i < s.size();) {
        const auto b = static_cast<unsigned char>(s[i]);
        std::size_t n = b < 0x80 ? 1 : (b >> 5) == 6 ? 2 : (b >> 4) == 14 ? 3 : (b >> 3) == 30 ? 4 : 0;
        if (n == 0 || i + n > s.size()) {
            out.push_back(0xfffd);
            i++;
            continue;
        }
        char32_t c = n == 1 ? b : n == 2 ? (b & 0x1f) : n == 3 ? (b & 0x0f) : (b & 0x07);
        bool ok = true;
        for (std::size_t k = 1; k < n; k++) {
            const auto x = static_cast<unsigned char>(s[i + k]);
            ok = ok && (x >> 6) == 2;
            c = (c << 6) | (x & 0x3f);
        }
        out.push_back(ok ? c : 0xfffd);
        i += ok ? n : 1;
    }
    return out;
}

std::pair<std::uint32_t, std::uint32_t> text_size(std::string_view text, std::uint32_t scale) {
    const auto n = static_cast<std::uint32_t>(decode(text).size());
    if (n == 0) {
        return {0, 0};
    }
    return {(n * (GLYPH_W + 1) - 1) * scale, GLYPH_H * scale};
}

float QuadSpec::angle_deg(float metres) const {
    return 2.0F * std::atan(0.5F * metres / distance) * 180.0F / std::numbers::pi_v<float>;
}

float QuadSpec::font_pixel_deg(std::uint32_t scale, std::uint32_t tex_w) const {
    return angle_deg(width / static_cast<float>(tex_w) * static_cast<float>(scale));
}

std::uint32_t fit_scale(std::span<const std::string> lines, std::uint32_t w, std::uint32_t h) {
    for (std::uint32_t scale = MAX_SCALE; scale >= 1; scale--) {
        std::uint32_t widest = 0;
        for (const auto& l : lines) {
            widest = std::max(widest, text_size(l, scale).first);
        }
        const auto rows = static_cast<std::uint32_t>(lines.size()) * 10 * scale;
        if (widest + 4 * scale <= w && rows + 4 * scale <= h) {
            return scale;
        }
    }
    return 1;
}

std::array<std::uint8_t, 3> Image::get(std::uint32_t x, std::uint32_t y) const {
    const std::size_t i = (static_cast<std::size_t>(y) * w + x) * 3;
    return {rgb[i], rgb[i + 1], rgb[i + 2]};
}

std::size_t Image::lit() const {
    std::size_t n = 0;
    for (std::size_t i = 0; i + 2 < rgb.size(); i += 3) {
        n += (rgb[i] | rgb[i + 1] | rgb[i + 2]) != 0 ? 1 : 0;
    }
    return n;
}

Image render(std::span<const std::string> lines) {
    Image img{TEX_W, TEX_H, std::vector<std::uint8_t>(static_cast<std::size_t>(TEX_W) * TEX_H * 3, 0)};
    const auto set = [&](int x, int y, const std::array<std::uint8_t, 3>& q) {
        if (x >= 0 && y >= 0 && x < static_cast<int>(img.w) && y < static_cast<int>(img.h)) {
            const std::size_t i = (static_cast<std::size_t>(y) * img.w + static_cast<std::size_t>(x)) * 3;
            img.rgb[i] = q[0];
            img.rgb[i + 1] = q[1];
            img.rgb[i + 2] = q[2];
        }
    };
    const std::uint32_t scale = fit_scale(lines, TEX_W, TEX_H);
    const auto s = static_cast<int>(scale);
    const std::array<std::uint8_t, 3> fg = {to_byte(HUD_FG), to_byte(HUD_FG), to_byte(HUD_FG)};
    const std::array<std::uint8_t, 3> bg = {to_byte(HUD_BG[0]), to_byte(HUD_BG[1]), to_byte(HUD_BG[2])};
    for (std::size_t li = 0; li < lines.size(); li++) {
        const auto [tw, th] = text_size(lines[li], scale);
        const int x = (static_cast<int>(TEX_W) - static_cast<int>(tw)) / 2;
        const int y = 2 * s + static_cast<int>(li) * 10 * s;
        // the label: a backdrop with one font pixel of margin, then the text
        if (tw > 0) {
            for (int yy = y - s; yy < y + static_cast<int>(th) + s; yy++) {
                for (int xx = x - s; xx < x + static_cast<int>(tw) + s; xx++) {
                    set(xx, yy, bg);
                }
            }
        }
        const auto text = decode(lines[li]);
        for (std::size_t k = 0; k < text.size(); k++) {
            const int gx = x + static_cast<int>(k) * static_cast<int>(GLYPH_W + 1) * s;
            for (std::uint32_t row = 0; row < GLYPH_H; row++) {
                for (std::uint32_t col = 0; col < GLYPH_W; col++) {
                    if (!pixel(text[k], col, row)) {
                        continue;
                    }
                    for (int dy = 0; dy < s; dy++) {
                        for (int dx = 0; dx < s; dx++) {
                            set(gx + static_cast<int>(col) * s + dx, y + static_cast<int>(row) * s + dy, fg);
                        }
                    }
                }
            }
        }
    }
    return img;
}

void to_upload(const Image& img, Encoding enc, bool bgra, std::span<std::uint8_t> out) {
    std::array<std::uint8_t, 256> table{};
    for (int i = 0; i < 256; i++) {
        const float code = static_cast<float>(i) / 255.0F;
        table[static_cast<std::size_t>(i)] = enc == Encoding::Srgb ? to_byte(code) : to_byte(srgb_to_linear(code));
    }
    const std::size_t px = static_cast<std::size_t>(img.w) * img.h;
    for (std::size_t i = 0; i < px && 4 * i + 3 < out.size(); i++) {
        const std::uint8_t r = table[img.rgb[3 * i]];
        const std::uint8_t g = table[img.rgb[3 * i + 1]];
        const std::uint8_t b = table[img.rgb[3 * i + 2]];
        out[4 * i] = bgra ? b : r;
        out[4 * i + 1] = g;
        out[4 * i + 2] = bgra ? r : b;
        out[4 * i + 3] = 255;
    }
}

void alpha_from_black(std::span<std::uint8_t> px) {
    for (std::size_t i = 0; i + 3 < px.size(); i += 4) {
        px[i + 3] = px[i] == 0 && px[i + 1] == 0 && px[i + 2] == 0 ? 0 : 255;
    }
}

} // namespace pibridge::eyecal::hud
