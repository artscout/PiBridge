// SPDX-License-Identifier: MIT
// The in-headset view lists and the software preview.
#include "editor_util.h"
#include "pibridge/room/raster.h"
#include "pibridge/room/scene.h"
#include "pibridge_test.h"

using namespace pibridge::room;
using edtest::Rig;

TEST(scene_has_walls_area_and_text) {
    Rig r;
    r.frame({0.3, 1.0, 0.3});
    Batch b;
    buildScene(*r.ed, b, r.now);
    CHECK(b.tris.size() % 3 == 0);
    CHECK(b.lines.size() % 2 == 0);
    CHECK(b.tris.size() > 1000); // walls, posts, lengths in the pixel font
    CHECK(b.lines.size() > 100);  // grid, edges, play area
    const std::size_t closed = b.tris.size();
    r.click({0.3, 1.0, 0.3}, bit::Menu);
    buildScene(*r.ed, b, r.now);
    CHECK(b.tris.size() > closed + 2000); // the panel with its labels
}

TEST(font) {
    CHECK(textWidth("AB") == 11);
    CHECK(textWidth("") == 0);
    CHECK(glyph('a') == glyph('A'));
    CHECK(glyph('~') == glyph('?'));
    Batch b;
    drawText(b, "I", {}, {1, 0, 0}, {0, 1, 0}, 1, {});
    CHECK(b.tris.size() == 6 * 11); // 'I' has 11 pixels
    CHECK(wrapText("one two three", 7).size() == 2);
    CHECK(wrapText("one two three", 7)[0] == "ONE TWO");
}

TEST(raster_preview) {
    Rig r;
    r.frame({0.3, 1.0, 0.3});
    Batch b;
    buildScene(*r.ed, b, r.now);
    const Mat4 vp = mul4(projectionVulkan(-0.9, 0.9, 0.7, -0.7, 0.05, 60), lookAt({0, 1.7, 3.5}, {0, 0.6, -0.5}));
    const Image img = rasterize(b, vp, 320, 200, {0, 0, 0, 1});
    CHECK(img.rgb.size() == 320 * 200 * 3);
    std::size_t lit = 0;
    for (const auto v : img.rgb) {
        lit += v > 20 ? 1 : 0;
    }
    CHECK(lit > 3000); // something is drawn
    // projection: a point straight ahead lands in the middle; up is up on screen (Vulkan y down)
    const Mat4 p = projectionVulkan(-0.8, 0.8, 0.8, -0.8, 0.1, 10);
    const auto c = transform4(p, {0, 0, -2});
    CHECK_NEAR(c[0] / c[3], 0, 1e-9);
    CHECK_NEAR(c[1] / c[3], 0, 1e-9);
    const auto up = transform4(p, {0, 1, -2});
    CHECK(up[1] / up[3] < 0);
    CHECK(c[2] / c[3] > 0 && c[2] / c[3] < 1);
}

PIBRIDGE_TEST_MAIN()
