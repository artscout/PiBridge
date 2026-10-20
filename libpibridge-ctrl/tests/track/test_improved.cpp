// SPDX-License-Identifier: MIT
// PiBridge's tracker improvements (docs/research/ctrl-tracking-improvements.md): core blobs, dim LEDs, the switches.
#include "pibridge/ctrl/parity.h"
#include "pibridge/ctrl/track/blob.h"
#include "pibridge/ctrl/track/tracker.h"
#include "pibridge_test.h"

#include <cmath>
#include <string_view>

using namespace pibridge::ctrl::track;

namespace {
/// A round saturated LED with a halo (a Gaussian, peak 255 clipped, scaled by k).
void led(std::vector<std::uint8_t>& img, std::size_t w, double cx, double cy, double k, double sigma) {
    const std::size_t h = img.size() / w;
    for (std::size_t y = 0; y < h; y++) {
        for (std::size_t x = 0; x < w; x++) {
            const double d2 = (static_cast<double>(x) - cx) * (static_cast<double>(x) - cx) +
                              (static_cast<double>(y) - cy) * (static_cast<double>(y) - cy);
            const double v = std::min(255.0, img[y * w + x] + k * 400.0 * std::exp(-d2 / (2.0 * sigma * sigma)));
            img[y * w + x] = static_cast<std::uint8_t>(v);
        }
    }
}
} // namespace

TEST(defaults_are_rust) {
    const BlobParams b;
    CHECK(!b.core && !b.dim);
    CHECK(!BlobParams::live().core && !BlobParams::live().dim);
    CHECK(BlobParams::improved().core && BlobParams::improved().dim);
    CHECK(BlobParams::improved().threshold == BlobParams::live().threshold);
    const TrackCfg t;
    CHECK(!t.slot_model && !t.imu_id && !t.use_hint);
    CHECK(TrackCfg::improved().slot_model && TrackCfg::improved().imu_id && !TrackCfg::improved().use_hint);
}

TEST(imu_identity_needs_samples) {
    // without IMU samples the hand-identity test refuses nothing; a controller that stopped reporting while the other
    // goes on is refused, and when both are quiet (no writer) nothing is refused
    Tracker tk(TrackCfg::improved());
    const pibridge::cam::geom::Pose p;
    CHECK(tk.imu_refuse(0, p, 1'000'000'000) == nullptr);
    ImuSample a;
    a.t = 1'000'000'000;
    tk.imu_push(0, a);
    a.t = 2'000'000'000;
    tk.imu_push(1, a);
    CHECK(std::string_view(tk.imu_refuse(0, p, 2'000'000'000) ? tk.imu_refuse(0, p, 2'000'000'000) : "") == "silent");
    CHECK(tk.imu_refuse(0, p, 9'000'000'000) == nullptr);
}

TEST(core_splits_merged_halos) {
    // two bright LEDs 10 px apart: their halos merge at the live threshold 80
    const std::size_t w = 64, h = 48;
    std::vector<std::uint8_t> img(w * h, 0);
    led(img, w, 25.0, 20.0, 2.0, 2.2);
    led(img, w, 35.0, 20.0, 2.0, 2.2);
    const auto merged = detect(img, w, h, w, BlobParams::live());
    CHECK(merged.size() == 1);
    const auto split = detect(img, w, h, w, BlobParams::improved());
    CHECK(split.size() == 2);
    for (const Blob& b : split) {
        CHECK(std::fabs(b.y - 20.0) < 0.05);
        CHECK(std::fabs(b.x - 25.0) < 0.5 || std::fabs(b.x - 35.0) < 0.5);
    }
}

TEST(core_keeps_a_single_led) {
    const std::size_t w = 64, h = 48;
    std::vector<std::uint8_t> img(w * h, 0);
    led(img, w, 30.3, 20.6, 1.5, 1.8);
    const auto a = detect(img, w, h, w, BlobParams::live());
    const auto b = detect(img, w, h, w, BlobParams::improved());
    CHECK(a.size() == 1 && b.size() == 1);
    CHECK(std::fabs(b[0].x - 30.3) < 0.1 && std::fabs(b[0].y - 20.6) < 0.1);
    // the area reported is the LED's whole size at the threshold
    CHECK(a[0].area == b[0].area);
}

TEST(dim_leds_are_found) {
    // an LED whose brightest pixel is 100 (< required peak 120): no blob in the Rust detection
    const std::size_t w = 64, h = 48;
    std::vector<std::uint8_t> img(w * h, 3);
    led(img, w, 20.0, 20.0, 0.24, 1.5);
    CHECK(detect(img, w, h, w, BlobParams::live()).empty());
    const auto b = detect(img, w, h, w, BlobParams::improved());
    CHECK(b.size() == 1 && std::fabs(b[0].x - 20.0) < 0.1);
    // noise below dim_min stays empty
    std::vector<std::uint8_t> dark(w * h, 0);
    dark[10 * w + 10] = 30;
    CHECK(detect(dark, w, h, w, BlobParams::improved()).empty());
}

PIBRIDGE_TEST_MAIN()
