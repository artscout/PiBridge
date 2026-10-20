// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-ctrltrack/src/blob.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/track/blob.h"
#include "pibridge_test.h"

#include <cmath>
#include <tuple>

using namespace pibridge::ctrl::track;

TEST(two_blobs_subpixel) {
    const std::size_t w = 64, h = 48;
    std::vector<std::uint8_t> img(w * h, 0);
    // symmetric 3x3 blob centred at (10, 20)
    for (const auto& [dy, dx, v] : {std::tuple{0, 0, 250}, std::tuple{-1, 0, 150}, std::tuple{1, 0, 150},
                                    std::tuple{0, -1, 150}, std::tuple{0, 1, 150}}) {
        img[static_cast<std::size_t>(20 + dy) * w + static_cast<std::size_t>(10 + dx)] = static_cast<std::uint8_t>(v);
    }
    // 2x1 blob: centre between 40 and 41 on row 5
    img[5 * w + 40] = 200;
    img[5 * w + 41] = 200;
    // a dim blob without a bright peak is rejected
    img[30 * w + 30] = 80;
    img[30 * w + 31] = 80;
    const auto b = detect(img, w, h, w, BlobParams{});
    CHECK(b.size() == 2);
    for (const Blob& x : b) {
        if (x.area == 5) {
            CHECK(std::fabs(x.x - 10.0) < 1e-9 && std::fabs(x.y - 20.0) < 1e-9);
        } else {
            CHECK(x.area == 2);
            CHECK(std::fabs(x.x - 40.5) < 1e-9 && std::fabs(x.y - 5.0) < 1e-9);
        }
    }
}

TEST(u_shape_merges) {
    const std::size_t w = 16, h = 16;
    std::vector<std::uint8_t> img(w * h, 0);
    for (std::size_t y = 2; y < 8; y++) {
        img[y * w + 2] = 200;
        img[y * w + 6] = 200;
    }
    for (std::size_t x = 2; x < 7; x++) {
        img[8 * w + x] = 200;
    }
    const auto b = detect(img, w, h, w, BlobParams{});
    CHECK(b.size() == 1);
    CHECK(!b.empty() && b[0].area == 17);
}

// not in the Rust file: the live parameters and their overrides (a bad value is ignored)
TEST(live_params_and_overrides) {
    CHECK(BlobParams::live().threshold == 80 && BlobParams::live().required_peak == 120);
    const std::vector<std::string> a{"live", "--thr", "+70", "--peak", "300"};
    const BlobParams p = BlobParams::from_args(a);
    CHECK(p.threshold == 70 && p.required_peak == 120);
}

PIBRIDGE_TEST_MAIN()
