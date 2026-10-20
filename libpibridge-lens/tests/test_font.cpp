// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-lens/src/font.rs (MIT OR Apache-2.0).
#include "pibridge/lens/font.h"
#include "pibridge_test.h"

#include <algorithm>
#include <cstring>
#include <set>
#include <vector>

using namespace pibridge::lens;

TEST(every_glyph_is_5_wide_7_tall_and_unique_char) {
    std::set<char32_t> seen;
    for (std::size_t g = 0; g < font::detail::GLYPH_COUNT; g++) {
        const auto& gl = font::detail::GLYPHS[g];
        CHECK(seen.insert(gl.c).second);
        for (const char* row : gl.rows) {
            CHECK(std::strlen(row) == 5);
            for (const char* p = row; *p != 0; p++) {
                CHECK(*p == '#' || *p == '.');
            }
        }
    }
    for (const char c : std::string_view("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 .,:-+=/%()[]_")) {
        CHECK(seen.count(static_cast<char32_t>(c)) == 1);
    }
}

TEST(letters_are_distinct_and_non_blank) {
    const auto sig = [](char32_t c) {
        std::vector<bool> v;
        for (std::uint32_t r = 0; r < 7; r++) {
            for (std::uint32_t k = 0; k < 5; k++) {
                v.push_back(font::pixel(c, k, r));
            }
        }
        return v;
    };
    const std::string all = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    for (std::size_t i = 0; i < all.size(); i++) {
        const auto a = sig(static_cast<char32_t>(all[i]));
        CHECK(std::any_of(a.begin(), a.end(), [](bool p) { return p; }));
        for (std::size_t j = i + 1; j < all.size(); j++) {
            CHECK(a != sig(static_cast<char32_t>(all[j])));
        }
    }
}

TEST(lower_case_maps_to_upper_and_unknown_is_a_box) {
    CHECK(font::pixel('a', 1, 0) == font::pixel('A', 1, 0));
    CHECK(font::pixel(0x263a, 0, 0) && font::pixel(0x263a, 4, 6) && !font::pixel(0x263a, 2, 3));
    CHECK(!font::pixel('A', 5, 0) && !font::pixel('A', 0, 7));
}

TEST(size_and_draw) {
    CHECK((font::text_size("", 3) == std::pair<std::uint32_t, std::uint32_t>{0, 0}));
    CHECK((font::text_size("A", 1) == std::pair<std::uint32_t, std::uint32_t>{5, 7}));
    CHECK((font::text_size("AB", 2) == std::pair<std::uint32_t, std::uint32_t>{22, 14}));
    // the degree sign is one glyph (one code point)
    CHECK((font::text_size("10\xc2\xb0", 1) == std::pair<std::uint32_t, std::uint32_t>{17, 7}));
    int n = 0;
    std::int32_t maxx = 0;
    std::int32_t maxy = 0;
    font::draw_text("1", 10, 20, 2, [&](std::int32_t x, std::int32_t y) {
        n++;
        maxx = std::max(maxx, x);
        maxy = std::max(maxy, y);
    });
    int lit = 0;
    for (std::uint32_t r = 0; r < 7; r++) {
        for (std::uint32_t c = 0; c < 5; c++) {
            lit += font::pixel('1', c, r) ? 1 : 0;
        }
    }
    CHECK(n == lit * 4);
    CHECK(maxx < 10 + 10 && maxy < 20 + 14);
}

PIBRIDGE_TEST_MAIN()
