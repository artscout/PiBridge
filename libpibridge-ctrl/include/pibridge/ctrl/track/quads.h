// SPDX-License-Identifier: MIT
// PiBridge: the LED quad index, a geometric hash for acquiring a controller without a prior.
//
// The brute-force acquisition (solve.h `acquire`) tries every blob triple against every LED triple of the model with
// P3P. The index replaces that search by a lookup: four blobs close together in the image have a shape that, made
// independent of where the camera looks and of the distance (the bearings projected onto the plane perpendicular to
// their mean direction, then a similarity transform taking the base pair to (0,0) and (1,0)), names the four LEDs
// that can make it. The index holds that shape for every group of four neighbouring LEDs as the camera sees them from
// all around the controller and at several distances (only LEDs facing the camera), quantised into a hash map.
//
// Lookup of four blob bearings returns the LED quads whose stored shape is within the tolerance; each is checked with
// P3P on three of the pairs and the fourth as a test, then scored like the brute force.
#pragma once

#include "pibridge/cam/geom.h"
#include "pibridge/ctrl/track/model.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

namespace pibridge::ctrl::track {

struct QuadParams {
    /// Camera directions sampled on the sphere around the controller (Fibonacci) and the distances (m).
    std::size_t directions = 3000;
    std::vector<double> distances{0.15, 0.25, 0.4, 0.7, 1.2};
    /// An LED is in a view when its normal is within this angle of the direction to the camera.
    double facing_deg = 80.0;
    /// Neighbours per anchor LED that the stored quads are made of (anchor + 3 of them). The default stores every
    /// group of four LEDs in the view: in the image some LEDs are hidden (the body, a hand, the facing limit is
    /// generous), so a blob's nearest neighbours are often LEDs that are not its nearest in the model view.
    std::size_t knn = 32;
    /// Hash cell size of the shape coordinates.
    double cell = 0.05;
    /// Base pairs: every pair at least this fraction of the longest one is stored as a base (the lookup takes the
    /// longest), so noise that changes which pair is longest still finds the entry.
    double base_frac = 0.85;
};

/// A similarity-invariant shape of four directions with the base pair (a, b) and the other two (c, d):
/// gnomonic projection about the mean direction, then (z - z_a) / (z_b - z_a) of c and d as (x_c, y_c, x_d, y_d).
std::array<double, 4> quad_shape(const std::array<cam::geom::Vec3, 4>& dirs, std::size_t a, std::size_t b,
                                 std::size_t c, std::size_t d);

class QuadIndex {
public:
    struct Entry {
        std::array<std::uint8_t, 4> led{}; // in shape order: base a, base b, c, d
        std::array<float, 4> shape{};
    };
    /// A lookup result: blob slots (0..3 of the query, in shape order) and their LEDs.
    struct Hit {
        std::array<std::uint8_t, 4> slot{};
        std::array<std::uint8_t, 4> led{};
    };

    static std::shared_ptr<const QuadIndex> build(const LedModel& model, const QuadParams& p = {});

    /// The LED quads that can make the four bearings; `tol` is the shape tolerance (the caller derives it from the
    /// angular noise over the base length). Appends to `out`.
    void lookup(const std::array<cam::geom::Vec3, 4>& dirs, double tol, std::vector<Hit>& out) const;

    std::size_t entries() const { return entries_; }
    std::size_t cells() const { return map_.size(); }
    double build_ms() const { return build_ms_; }

private:
    QuadParams p_;
    std::unordered_map<std::uint64_t, std::vector<Entry>> map_;
    std::size_t entries_ = 0;
    double build_ms_ = 0;
    std::uint64_t key(const std::array<int, 4>& q) const;
};

} // namespace pibridge::ctrl::track
