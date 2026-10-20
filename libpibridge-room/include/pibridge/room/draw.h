// SPDX-License-Identifier: MIT
// Ported from ZoneTweak batch.go and font.go (MIT, Copyright (c) 2026 killdano; see NOTICE).
//
// What the in-headset view is made of: unlit coloured triangles and lines in stage space (one vertex = position +
// colour), and ZoneTweak's 5x7 pixel font drawn as small quads. The renderer only uploads these lists.
#pragma once

#include "pibridge/room/vmath.h"

#include <string>
#include <string_view>
#include <vector>

namespace pibridge::room {

struct Vertex {
    float x, y, z;
    float r, g, b, a;
};

struct Batch {
    std::vector<Vertex> tris;  // 3 per triangle
    std::vector<Vertex> lines; // 2 per line
    void clear() {
        tris.clear();
        lines.clear();
    }
    void vert(std::vector<Vertex>& into, Vec3 p, Rgba c) {
        into.push_back({static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z), c.r, c.g, c.b, c.a});
    }
    void line(Vec3 a, Vec3 b, Rgba c) {
        vert(lines, a, c);
        vert(lines, b, c);
    }
    void tri(Vec3 a, Vec3 b, Vec3 c, Rgba col) {
        vert(tris, a, col);
        vert(tris, b, col);
        vert(tris, c, col);
    }
    void quad(Vec3 a, Vec3 b, Vec3 c, Vec3 d, Rgba col) {
        tri(a, b, c, col);
        tri(a, c, d, col);
    }
};

/// 7 rows of 5 bits (bit 4 = left column) for an upper-case character; '?' for one the font lacks.
const std::array<unsigned char, 7>& glyph(char c);
/// Width in font pixels: 6 per character minus the last gap.
int textWidth(std::string_view s);
/// Text from `origin` (bottom left) along `right`, upwards along `up`, `px` metres per font pixel; upper case.
void drawText(Batch& b, std::string_view s, Vec3 origin, Vec3 right, Vec3 up, double px, Rgba col);
void drawTextCentered(Batch& b, std::string_view s, Vec3 center, Vec3 right, Vec3 up, double px, Rgba col);
/// Words in upper case wrapped to `width` characters.
std::vector<std::string> wrapText(std::string_view s, std::size_t width);

} // namespace pibridge::room
