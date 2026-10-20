// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-lenstest/src/hudquad.rs and pimax-lens/src/font.rs tests (MIT OR Apache-2.0).
#include "hud.h"

#include "pibridge_test.h"

#include <cmath>
#include <set>
#include <string>
#include <vector>

using namespace pibridge::eyecal::hud;

TEST(every_glyph_is_5_wide_7_tall_and_unique_char) {
    std::set<char32_t> seen;
    for (const char32_t c : glyph_chars()) {
        CHECK(seen.insert(c).second);
    }
    for (const char c : std::string("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 .,:-+=/%()[]_")) {
        CHECK(seen.contains(static_cast<char32_t>(c)));
    }
}

TEST(letters_are_distinct_and_non_blank) {
    const auto sig = [](char32_t c) {
        std::vector<bool> v;
        for (std::uint32_t r = 0; r < 7; r++) {
            for (std::uint32_t k = 0; k < 5; k++) {
                v.push_back(pixel(c, k, r));
            }
        }
        return v;
    };
    const std::u32string all = U"ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    for (std::size_t i = 0; i < all.size(); i++) {
        const auto a = sig(all[i]);
        bool any = false;
        for (const bool p : a) {
            any = any || p;
        }
        CHECK(any);
        for (std::size_t j = i + 1; j < all.size(); j++) {
            CHECK(a != sig(all[j]));
        }
    }
}

TEST(lower_case_maps_to_upper_and_unknown_is_a_box) {
    CHECK(pixel(U'a', 1, 0) == pixel(U'A', 1, 0));
    CHECK(pixel(U'☺', 0, 0) && pixel(U'☺', 4, 6) && !pixel(U'☺', 2, 3));
    CHECK(!pixel(U'A', 5, 0) && !pixel(U'A', 0, 7));
    // UTF-8: the degree sign is one character
    CHECK(decode("\xc2\xb0").size() == 1 && decode("\xc2\xb0")[0] == 0xb0);
    CHECK(pixel(0xb0, 1, 0));
}

TEST(size_and_draw) {
    CHECK(text_size("", 3) == (std::pair<std::uint32_t, std::uint32_t>{0, 0}));
    CHECK(text_size("A", 1) == (std::pair<std::uint32_t, std::uint32_t>{5, 7}));
    CHECK(text_size("AB", 2) == (std::pair<std::uint32_t, std::uint32_t>{22, 14}));
}

TEST(quad_hangs_ahead_and_slightly_below_the_centre) {
    const auto [x, y, z] = SPEC.position();
    CHECK(x == 0.0F);
    CHECK(z == -2.0F);
    CHECK(std::fabs(y - -(0.25F + 0.512F)) < 1e-6F);
    // the top edge is 0.25 m below the centre of view, the bottom edge well inside the lens FOV
    const float top = std::atan2(y + 0.5F * SPEC.height, -z) * 180.0F / 3.14159265F;
    const float bottom = std::atan2(y - 0.5F * SPEC.height, -z) * 180.0F / 3.14159265F;
    CHECK(std::fabs(top - -7.125F) < 0.01F);
    CHECK(bottom > -35.0F && bottom < top);
}

TEST(quad_size_is_legible_and_not_too_wide) {
    const float width = SPEC.angle_deg(SPEC.width);
    CHECK(width > 30.0F && width < 60.0F);
    // text at the largest scale: a character cell about 1 degree tall (7 font pixels)
    const float ch = 7.0F * SPEC.font_pixel_deg(MAX_SCALE, TEX_W);
    CHECK(ch > 0.8F && ch < 1.5F);
    // texture and quad have the same aspect ratio: square pixels
    CHECK(std::fabs(SPEC.width / static_cast<float>(TEX_W) - SPEC.height / static_cast<float>(TEX_H)) < 1e-6F);
}

TEST(scale_shrinks_to_fit) {
    const std::vector<std::string> shortl = {"HI"};
    CHECK(fit_scale(shortl, TEX_W, TEX_H) == MAX_SCALE);
    CHECK(fit_scale({}, TEX_W, TEX_H) == MAX_SCALE);
    // 64 characters (the wrap width) fit at scale 5 in 2048 px
    const std::vector<std::string> wrap = {std::string(64, 'A')};
    CHECK(fit_scale(wrap, TEX_W, TEX_H) == MAX_SCALE);
    const std::vector<std::string> wide = {std::string(80, 'A')};
    const auto s = fit_scale(wide, TEX_W, TEX_H);
    CHECK(s >= 1 && s < MAX_SCALE);
    std::vector<std::string> tall;
    for (int i = 0; i < 30; i++) {
        tall.push_back("L" + std::to_string(i));
    }
    const auto s2 = fit_scale(tall, TEX_W, TEX_H);
    CHECK(s2 * 10 * 30 + 4 * s2 <= TEX_H);
    // absurdly much text still returns the minimum
    const std::vector<std::string> huge(200, std::string(500, 'A'));
    CHECK(fit_scale(huge, TEX_W, TEX_H) == 1);
}

TEST(texture_has_text_centred_and_a_transparent_rest) {
    const std::vector<std::string> lines = {"HEAD LOCKED", "SECOND"};
    const auto img = render(lines);
    CHECK(img.w == TEX_W && img.h == TEX_H);
    const auto lit = img.lit();
    CHECK(lit > 100 && lit < TEX_W * TEX_H / 10);
    CHECK((img.get(0, 0) == std::array<std::uint8_t, 3>{0, 0, 0}));
    CHECK((img.get(TEX_W - 1, TEX_H - 1) == std::array<std::uint8_t, 3>{0, 0, 0}));
    std::uint32_t lo = TEX_W;
    std::uint32_t hi = 0;
    for (std::uint32_t x = 0; x < TEX_W; x++) {
        for (std::uint32_t y = 0; y < TEX_H; y++) {
            const auto c = img.get(x, y);
            if ((c[0] | c[1] | c[2]) != 0) {
                lo = std::min(lo, x);
                hi = std::max(hi, x);
                break;
            }
        }
    }
    const int mid = static_cast<int>(lo + hi) / 2;
    CHECK(std::abs(mid - static_cast<int>(TEX_W) / 2) < 8);
    CHECK(render({}).lit() == 0);
}

TEST(alpha_is_zero_only_on_pure_black) {
    std::array<std::uint8_t, 16> px = {0, 0, 0, 255, 3, 0, 0, 255, 0, 0, 9, 255, 40, 40, 40, 0};
    alpha_from_black(px);
    CHECK(px[3] == 0 && px[7] == 255 && px[11] == 255 && px[15] == 255);
    // the upload: sRGB keeps the code, UNORM stores the decoded value, BGRA swaps
    Image one{1, 1, {200, 100, 50}};
    std::array<std::uint8_t, 4> out{};
    to_upload(one, Encoding::Srgb, false, out);
    CHECK(out[0] == 200 && out[1] == 100 && out[2] == 50 && out[3] == 255);
    to_upload(one, Encoding::Srgb, true, out);
    CHECK(out[0] == 50 && out[2] == 200);
    to_upload(one, Encoding::Linear, false, out);
    CHECK(out[0] < 200 && out[0] > 100);
}

PIBRIDGE_TEST_MAIN()
