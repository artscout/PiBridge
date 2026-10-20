// SPDX-License-Identifier: MIT
// Twin of the test in omarchyvr pimax-cam/src/camera.rs (MIT OR Apache-2.0).
#include "common.h"
#include "pibridge/cam/camera.h"
#include "pibridge_test.h"

using namespace pibridge::cam;
using pibridge::cam::testing::synth;
using pibridge::cam::testing::VecSource;

TEST(pipeline_counts_kinds_drops_and_times) {
    auto src = std::make_unique<VecSource>();
    std::uint32_t counter = 100;
    for (std::uint64_t i = 0; i < 20; i++) {
        if (i == 10) {
            counter += 2; // two frames lost
        }
        const bool tracking = counter % 2 == 0;
        const std::uint16_t off = tracking ? 278 : 10;
        src->frames.emplace_back(synth(tracking, counter, 5'000'000 + counter * 20'000ull, off),
                                 9'000'000'000ull + counter * 20'000'000ull + 11'000'000);
        counter++;
    }
    Camera cam(std::move(src));
    std::vector<FrameInfo> infos;
    for (;;) {
        auto f = cam.next_frame(0);
        CHECK(f.has_value());
        if (!*f) {
            break;
        }
        infos.push_back(**f);
    }
    CHECK(infos.size() == 20);
    CHECK(cam.stats.dropped == 2);
    CHECK(infos[10].dropped == 2);
    CHECK(cam.stats.tracking + cam.stats.led == 20);
    CHECK(cam.stats.row_mismatch == 0);
    CHECK(cam.stats.scanned_rows == 0);
    CHECK(cam.stats.kind_repeats == 0); // losing two frames keeps the tracking/LED alternation
    for (const auto& i : infos) {
        if (i.kind == static_cast<std::uint32_t>(FrameKind::Tracking)) {
            CHECK((i.y_offset == std::array<std::uint16_t, 4>{278, 278, 278, 278}));
            break;
        }
    }
    bool meta = true;
    bool inc = true;
    for (std::size_t k = 0; k < infos.size(); k++) {
        meta = meta && (infos[k].flags & FLAG_META_VALID) != 0;
        if (k > 0) {
            inc = inc && infos[k].mono_ts_ns > infos[k - 1].mono_ts_ns;
        }
    }
    CHECK(meta);
    CHECK(inc);
    // tiles come out with the right content
    const auto tile2 = *cam.tile(2);
    CHECK(tile2[0] == 1 + 100);
    CHECK(tile2[5 * TILE_W + 7] == 1 + 100 + 5);
    std::uint8_t small[10];
    const auto e = cam.copy_tile(4, small, 640);
    CHECK(!e && e.error().text() == "tile index out of range");
    const auto e2 = cam.copy_tile(0, small, 640);
    CHECK(!e2 && e2.error().text() == "destination too small");
}

PIBRIDGE_TEST_MAIN()
