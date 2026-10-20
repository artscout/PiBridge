// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-room/src/orb.rs (MIT OR Apache-2.0).
//
// ORB features for 8-bit grey images: FAST-9 corners on an image pyramid, intensity-centroid orientation, steered
// BRIEF (256-bit) descriptors and brute-force Hamming matching. Deterministic and single-threaded; written (in the
// Rust original) from the published descriptions (Rosten and Drummond: FAST; Calonder et al.: BRIEF; Rublee et al.:
// ORB), no OpenCV or ORB-SLAM code. Bit-identical to the Rust module (the build uses -ffp-contract=off and the f32
// libm calls the Rust code makes: sinf, cosf, atan2f).
//
// Pipeline: pyramid (bilinear, corner-aligned grid, 8-bit fixed-point weights) -> FAST-9 per grid cell (retry at the
// lower threshold in empty cells), 3x3 non-maximum suppression, Harris response (7x7 block of Sobel gradients) ->
// per-level budgets proportional to the area, best corner of every cell first, then by response, 3 px spacing, the
// shortfall topped up from other levels -> intensity-centroid angle (disc of radius 15, smoothed image) -> steered
// BRIEF on the 7x7 Gaussian (sigma 2) smoothed level image. The BRIEF pattern comes from a fixed-seed splitmix64
// (integer only); descriptors only compare with descriptors of this module (DESCRIPTOR_VERSION).
//
// Conventions: pixel centres at integer coordinates; Keypoint::angle in radians (-pi, pi], from +x towards +y;
// keypoint x / y in level-0 pixels (integers times scale^level); descriptor bit k = byte k / 8, bit k % 8, set when the
// first sample of pair k is darker than the second.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <tuple>
#include <utility>
#include <vector>

namespace pibridge::roommap::orb {

/// Version of the descriptor definition.
inline constexpr std::uint32_t DESCRIPTOR_VERSION = 1;

/// Borrowed grey image (row-major, `stride` bytes per row).
struct Image {
    std::size_t w = 0;
    std::size_t h = 0;
    std::size_t stride = 0;
    std::span<const std::uint8_t> data;

    /// A tightly packed image; invalid (is_valid false) when w or h is 0 or data is too short.
    static std::optional<Image> make(std::size_t w, std::size_t h, std::span<const std::uint8_t> data);
    /// w, h > 0, stride >= w, data holds (h - 1) * stride + w bytes (no overflow).
    bool is_valid() const;
};

/// Keypoint in level-0 pixel coordinates.
struct Keypoint {
    float x = 0;
    float y = 0;
    std::uint8_t level = 0;
    float angle = 0;
    float response = 0;
    bool operator==(const Keypoint&) const = default;
};

/// 256-bit binary descriptor.
using Descriptor = std::array<std::uint8_t, 32>;

struct OrbParams {
    std::size_t n_features = 500;
    std::size_t n_levels = 4;
    float scale = 1.2f;
    std::uint8_t fast_threshold = 20;
    std::uint8_t min_fast_threshold = 7;
    std::size_t grid_cols = 8;
    std::size_t grid_rows = 6;

    /// Size of a level-`level` pixel in level-0 pixels (scale clamped like extract does), as f32.
    float level_scale(std::uint8_t level) const;
    bool operator==(const OrbParams&) const = default;
};

/// Keypoint mask in level-0 pixel coordinates: false excludes a location. nullptr = the whole image.
using Mask = std::function<bool(float, float)>;

/// Detect + describe: at most n_features keypoints, ordered by level, then row, then column.
std::vector<std::pair<Keypoint, Descriptor>> extract(const Image& img, const Mask* mask, const OrbParams& p);

/// Hamming distance (0..256).
std::uint32_t hamming(const Descriptor& a, const Descriptor& b);

/// (index in a, index in b, distance).
using Match = std::tuple<std::size_t, std::size_t, std::uint32_t>;

/// Brute-force matching a -> b: mutual nearest neighbours, distance <= max_dist, best < ratio * second (strict; a
/// single candidate always passes); ordered by the index in a; ties to the lower index.
std::vector<Match> match_bf(std::span<const Descriptor> a, std::span<const Descriptor> b, std::uint32_t max_dist,
                            float ratio);
/// match_bf over the pairs `allow` lets through (called exactly once per pair, row-major).
std::vector<Match> match_gated(std::span<const Descriptor> a, std::span<const Descriptor> b, std::uint32_t max_dist,
                               float ratio, const std::function<bool(std::size_t, std::size_t)>& allow);

/// The internals the Rust unit tests reach (namespace detail: not an interface for other code).
namespace detail {

inline constexpr std::size_t BORDER = 22;
inline constexpr std::size_t MIN_LEVEL_DIM = 2 * BORDER + 16;
inline constexpr std::size_t IC_RADIUS = 15;
inline constexpr std::size_t MIN_KP_DIST2 = 9;
inline constexpr std::size_t MAX_LEVELS = 32;
inline constexpr std::size_t MAX_FEATURES = std::size_t{1} << 20;
inline constexpr std::int32_t MIN_PAIR_SEP2 = 16;

/// `[ax, ay, bx, by]` of the 256 comparisons.
const std::array<std::array<std::int8_t, 4>, 256>& pattern();
/// splitmix64 step: (new state, output).
std::pair<std::uint64_t, std::uint64_t> splitmix64(std::uint64_t state);

struct Cfg {
    std::size_t n_features;
    std::size_t n_levels;
    double scale;
    std::uint8_t thr;
    std::uint8_t min_thr;
    std::size_t cols;
    std::size_t rows;
    explicit Cfg(const OrbParams& p);
};

struct Geom {
    std::size_t w;
    std::size_t h;
    double scale;
};

std::vector<Geom> level_geometry(std::size_t img_w, std::size_t img_h, const Cfg& cfg);
std::vector<std::vector<std::uint8_t>> build_pyramid(const Image& img, const std::vector<Geom>& geom, double scale);
std::vector<Image> level_images(const Image& img, const std::vector<std::vector<std::uint8_t>>& bufs,
                                const std::vector<Geom>& geom);
std::vector<std::uint8_t> gaussian_blur(const Image& img);
std::uint8_t ring_score(const std::array<std::uint8_t, 16>& ring, std::uint8_t center, std::uint8_t thr);

struct Corner {
    std::size_t x;
    std::size_t y;
    std::uint8_t score;
};
struct Rect {
    std::size_t x0;
    std::size_t y0;
    std::size_t x1;
    std::size_t y1;
};
void fast_cell(const Image& img, Rect r, std::uint8_t thr, std::vector<std::uint8_t>& scratch, std::vector<Corner>& out);
float harris(const Image& img, std::size_t x, std::size_t y);
float ic_angle(std::span<const std::uint8_t> img, std::size_t stride, std::size_t x, std::size_t y);
Descriptor describe(std::span<const std::uint8_t> sm, std::size_t stride, std::size_t x, std::size_t y, float angle);

struct Cand {
    std::size_t x;
    std::size_t y;
    float resp;
    std::uint8_t level;
    std::uint16_t cell;
};
std::pair<std::vector<Cand>, std::vector<Cand>> select_level(const std::vector<Cand>& cands, std::size_t n_cells,
                                                             std::size_t budget);
void top_up(std::vector<std::vector<Cand>>& picked, std::vector<Cand> unused, std::size_t n_features);
std::vector<std::size_t> level_budgets(std::size_t n, const std::vector<Geom>& geom);

} // namespace detail

} // namespace pibridge::roommap::orb
