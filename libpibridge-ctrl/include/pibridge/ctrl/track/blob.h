// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/blob.rs (MIT OR Apache-2.0).
//
// Blob detection on short-exposure LED tiles (640x480 grey).
//
// Threshold + 8-connected components (two-pass union-find), intensity-weighted centroid above the background.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace pibridge::ctrl::track {

/// One detected blob in tile pixel coordinates (x right, y down).
struct Blob {
    double x = 0;
    double y = 0;
    /// Pixel count above the threshold.
    std::uint32_t area = 0;
    /// Brightest pixel value.
    std::uint8_t peak = 0;
    /// Bounding box width and height.
    std::uint16_t w = 0;
    std::uint16_t h = 0;
    bool operator==(const Blob&) const = default;
};

/// Detection parameters.
struct BlobParams {
    /// Pixel threshold to be part of a blob.
    std::uint8_t threshold = 60;
    /// A blob must contain at least one pixel at or above this value.
    std::uint8_t required_peak = 120;
    std::uint32_t min_area = 2;
    std::uint32_t max_area = 600;
    /// Maximum bounding box side (rejects lamps and windows).
    std::uint16_t max_side = 40;
    /// PiBridge (not in the Rust tool): measure each blob on its core. A component found at `threshold` is re-cut at
    /// max(threshold, core_frac * peak): the centroid is weighted over the core only (a saturated LED's halo no longer
    /// pulls it), touching LEDs whose halos merged at `threshold` come out as separate blobs, and the area / side
    /// limits apply to the core (a near, bright LED is not thrown away for its halo). Off = the Rust detection.
    bool core = false;
    double core_frac = 0.5;
    /// PiBridge: dim LEDs. When no pixel of the image reaches `required_peak` but the brightest reaches `dim_min`, the
    /// threshold and the peak test are scaled down with the brightest pixel (threshold / 255 and required_peak / 255
    /// of it): far or turned-away LEDs, a darker LED exposure. Off = the Rust detection (no blobs at all then).
    bool dim = false;
    std::uint8_t dim_min = 40;

    /// The live tracker's detection (`live`, `lockrate`): blob pixels from 80 instead of 60. At the LED-frame
    /// exposure (40 / gain 62) the 60-level halo merges neighbouring LEDs and shifts centroids; 80 gives more
    /// separate LEDs per controller (ctrl-test.pxcam, in-use part: brute-force finds both controllers in 54.5 % of
    /// LED frames instead of 46.8 %; omarchyvr analysis/ctrl-lock-rate.md). The required peak stays 120, so dim LEDs
    /// are detected as before.
    static BlobParams live();
    /// `live()` with `--thr T` / `--peak P` overrides (a value that is not a u8 is ignored, as in Rust).
    static BlobParams from_args(std::span<const std::string> a);
    /// `live()` plus PiBridge's improvements (`core`); what `live` uses unless PIBRIDGE_CTRL_PARITY=1.
    static BlobParams improved();
};

/// Detect blobs in a `w`x`h` image with row stride `stride`.
std::vector<Blob> detect(std::span<const std::uint8_t> img, std::size_t w, std::size_t h, std::size_t stride,
                         const BlobParams& prm);

} // namespace pibridge::ctrl::track
