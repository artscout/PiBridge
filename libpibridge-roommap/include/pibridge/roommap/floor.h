// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/floor.rs (MIT OR Apache-2.0).
//
// Floor detection and the floor in Monado's spaces. Frames are gravity aligned (y up), so the floor is a plane y = h:
// from stereo / map points (height histogram -> horizontal layers -> gravity-constrained RANSAC plane per layer; the
// floor is the lowest layer with enough support, extent and a plausible head height above it), from a controller lying
// on the floor, or typed in. Stored in the map directory (floor.toml); Monado reports head = (0, 1.6, 0) + offset *
// slam with STAGE at y = 0, so floor_shift moves the floor to app y = -1.6. Follower keeps it there smoothly.
#pragma once

#include "pibridge/cam/geom.h"
#include "pibridge/roommap/map.h"

#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pibridge::roommap::floor {

using cam::geom::Vec3;

inline constexpr double MONADO_ORIGIN_HEIGHT = 1.6;

struct FloorParams {
    double bin_m = 0.01;
    double band_m = 0.04;
    double inlier_m = 0.02;
    double max_tilt_deg = 4.0;
    std::size_t min_support = 60;
    double min_rel_support = 0.08;
    double min_area_m2 = 0.4;
    double head_min_m = 0.4;
    double head_max_m = 2.1;
    std::size_t iters = 300;
};

struct Layer {
    double y = 0;
    std::size_t inliers = 0;
    double spread_m = 0;
    double sigma_m = 0;
    double tilt_deg = 0;
    double area_m2 = 0;
};

struct Floor {
    Layer layer;
    double head_above_m = 0;
    double confidence = 0;
    /// Every horizontal layer found, lowest first.
    std::vector<Layer> layers;
    /// Why lower layers were not taken.
    std::vector<std::string> rejected;
};

/// Horizontal layers of pts, lowest first.
std::vector<Layer> layers(std::span<const Vec3> pts, std::array<double, 2> at, const FloorParams& p);
/// The floor among the points: the lowest supported, extended layer 0.4-2.1 m below the median head.
std::expected<Floor, std::string> detect(std::span<const Vec3> pts, std::span<const Vec3> heads, const FloorParams& p);
/// Follow a known floor from `prior` (robust mean shift, at most gate_m from the prior), level, no tilt.
std::optional<Floor> track(std::span<const Vec3> pts, std::span<const Vec3> heads, double prior, double gate_m,
                           const FloorParams& p);
/// Map points within max_range of their keyframe, and the keyframe (head) positions.
std::pair<std::vector<Vec3>, std::vector<Vec3>> map_points(const map::RoomMap& map, double max_range);

enum class FloorSource { Points, Touch, Manual, Standing };
const char* source_name(FloorSource s);
std::optional<FloorSource> source_parse(std::string_view s);

/// <map>/floor.toml: the floor height in the room frame.
struct StoredFloor {
    double y = 0;
    FloorSource source = FloorSource::Points;
    double confidence = 0;
    double sigma_m = 0;
    std::size_t inliers = 0;
    double head_above_m = 0;
    std::uint64_t when = 0;

    std::string to_toml() const;
    static std::expected<StoredFloor, std::string> parse(std::string_view s);
    static std::expected<StoredFloor, std::string> load(const std::string& dir);
    std::expected<void, std::string> save(const std::string& dir) const;
};

/// The change of the offset's y that puts a floor at app y `floor_y_app` onto Monado's STAGE floor.
double floor_shift(double floor_y_app, double origin_height);
/// Monado's origin height: 1.6 m + XRT_TRACKING_ORIGIN_OFFSET_Y of the Monado process (/proc/<pid>/environ), else
/// of this process. Under SteamVR (the session pid is vrserver: driver_monado, whose tracking origin offset is 0,
/// "use steamvr room setup instead") it is SteamVR's own standing height (steamvr_stage, -raw_from_stage.y), or 0
/// when that is unknown. (PiBridge: the Rust tool assumed monado-service's 1.6 m everywhere.)
double origin_height(std::uint32_t monado_pid);

/// SteamVR's standing (STAGE) frame in its raw tracking space while the Pimax driver runs inside vrserver: the app
/// space Monado's anchor offset produces is SteamVR's raw space, and SteamVR shows the standing frame
/// (GetRawZeroPoseToStandingAbsoluteTrackingPose, from the universe's chaperone "standing" pose). PiBridge's app writes
/// it ($XDG_RUNTIME_DIR/pibridge/steamvr-stage, PIBRIDGE_STEAMVR_STAGE_FILE overrides):
///   "pibridge-steamvr-stage 1\nvrserver PID\nuniverse U\nstanding_from_raw m00 m01 m02 m03 m10 .. m23\n" (row major 3x4).
struct SteamVrStage {
    std::uint32_t pid = 0;      // the vrserver it belongs to
    std::uint64_t universe = 0;
    double yaw = 0;             // raw_from_stage: p_raw = Ry(yaw) p_stage + t
    Vec3 t{0.0, 0.0, 0.0};
    double tilt_deg = 0;        // how far the matrix is from a pure yaw (should be ~0)
};
std::string steamvr_stage_path();
std::optional<SteamVrStage> parse_steamvr_stage(std::string_view text);
/// The process is SteamVR's vrserver (/proc/<pid>/comm).
bool is_vrserver(std::uint32_t pid);
/// pibridge-headtrack (the head tracking without Monado; its comm is cut to 15 characters).
bool is_headtrack(std::uint32_t pid);
/// The stage of the session `pid` when it is a vrserver and the file names it; nullopt otherwise.
std::optional<SteamVrStage> steamvr_stage(std::uint32_t pid);
/// A controller lying still on the floor: (floor y, spread) or why not.
std::expected<std::pair<double, double>, std::string> touch_floor(std::span<const Vec3> origins, double bottom_m,
                                                                  double max_spread_m);
/// Combine a point estimate and a touch (the touch wins beyond gate_m, else precision weighted).
std::optional<std::pair<double, double>> combine(std::optional<std::pair<double, double>> points,
                                                 std::optional<std::pair<double, double>> touch, double gate_m);

/// Smooth, rate-limited vertical correction of the offset (continuous floor lock).
struct Follower {
    double tau_s = 10.0;
    double max_rate = 0.01;
    double outlier_m = 0.05;
    std::size_t confirm = 3;
    std::optional<double> target;
    std::optional<double> applied;
    std::vector<double> pending;

    void measure(double shift, double dt_s);
    std::optional<double> step(double dt_s);
};

/// Downsampled points for a top-down preview (<map>/preview.txt, cm).
std::string preview_text(std::span<const Vec3> pts, std::span<const Vec3> heads, std::optional<double> floor_y,
                         std::size_t max_points);

} // namespace pibridge::roommap::floor
