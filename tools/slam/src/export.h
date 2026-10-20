// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-slam/src/export.rs, euroc.rs (load part), imu.rs and the omarchyvr-config paths /
// devicedata helpers the export uses (MIT OR Apache-2.0).
//
// Basalt files for a Pimax headset: calibration (from the factory XML, libpibridge-cam slam.h), VIO config (a Basalt base
// config with our overrides) and the Monado SLAM_CONFIG toml with absolute paths and the GUI off.
#pragma once

#include "pibridge/cam/calib.h"
#include "pibridge/cam/json.h"
#include "pibridge/cam/slam.h"

#include <cstdint>
#include <expected>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pibridge::slamtool {

using cam::slam::Vec3;

/// Our changes to Basalt's generic (EuRoC) VIO config: 640x480 fisheye at 25 Hz instead of 752x480 pinhole at 20 Hz,
/// so a finer detection grid (more points per image); everything else stays at Basalt's tested values.
inline const std::vector<std::pair<std::string, double>> OVERRIDES = {
    {"config.optical_flow_detection_grid_size", 40.0}};

/// Basalt's generic EuRoC VIO config of the pinned Basalt commit (BSD-3-Clause, NOTICE), unmodified, embedded: the
/// last fallback, so a clean install without a Basalt source tree still starts 6DoF.
std::string_view builtin_base();

/// What the export records as the base when builtin_base() is used.
inline constexpr std::string_view BUILTIN_BASE_NAME = "built in: Basalt df6e970 data/euroc/euroc_config.json";

/// Candidate base configs, best first (the installed Basalt's copy before Basalt's own msdmo config); with none on
/// disk the export uses builtin_base().
std::vector<std::string> base_config_candidates();

/// Serialise a JSON value (pretty, 2 spaces; integral numbers without a fraction).
void to_json(const cam::json::Value& v, std::size_t indent, std::string& out);

/// PiBridge (not in the Rust code): the one Basalt setting that depends on the camera count. Basalt detects new corners
/// in cam0 and tracks them into every other camera; a camera beyond the stereo pair (the Crystal Super's upper side
/// pair, cameras 2 and 3) looks mostly where cam0 does not, so it only gets corners of its own with this on. Every
/// other per-camera value (intrinsics, resolution, T_imu_cam) is in the calibration.
inline constexpr std::string_view MULTICAM_NONOVERLAP_KEY = "config.optical_flow_detection_nonoverlap";

/// Base config text with OVERRIDES applied. Integer-valued keys keep integer formatting. With more than two cameras
/// MULTICAM_NONOVERLAP_KEY is also set to true (added when the base lacks it); with two the output is the Rust one.
std::expected<std::string, std::string> vio_config(std::string_view base, std::size_t cams = 2);

/// The Monado SLAM_CONFIG toml for Basalt.
std::string toml(const std::string& calib, const std::string& config, bool show_gui);

/// Files written by export_files.
struct Exported {
    std::string calib;
    std::string config;
    std::string toml;
    /// Base config used.
    std::string base;
};

/// Write everything into dir (absolute paths inside the toml).
std::expected<Exported, std::string> export_files(const std::string& dir, const cam::DeviceCalib& cal,
                                                  const cam::slam::SlamSetup& setup, const cam::slam::ImuNoise& imu,
                                                  const std::optional<std::string>& base, bool show_gui);

/*
 * omarchyvr-config: the few paths the export reads
 */

/// key = "value" (or key = value) from the text of paths.toml; comments and blank values are ignored.
std::optional<std::string> paths_value(std::string_view text, std::string_view key);
/// The source/build tree root: $PIMAX_SRC_DIR, else src_dir in ~/.config/omarchyvr/paths.toml, else <home>/src.
std::string src_dir(const std::string& home);
/// The installed set: lib_dir in paths.toml, else <home>/.local/lib/omarchyvr.
std::string lib_dir(const std::string& home);
/// The cache of the names before omarchyvr's rename (<cache>/pimax/gyro-bias); the driver still reads it.
std::string legacy_gyro_bias_path(const std::string& home,
                                  const std::function<std::optional<std::string>(std::string_view)>& env);
/// The gyro bias every start begins with when the PC has none measured yet: the typical value of the Dream Air
/// reference headset (deg/s, raw IMU axes, the driver's cache format; devices/dream-air/reference/gyro-bias).
std::string_view reference_gyro_bias();

/*
 * EuRoC datasets (euroc.rs) and IMU statistics (imu.rs)
 */

/// One IMU sample (EuRoC column order: gyro then accel).
struct ImuRow {
    std::int64_t ts = 0;
    Vec3 gyro{};
    Vec3 accel{};
};

/// Find the dataset root (the directory holding mav0): dir itself, or the newest dir/<child> with a mav0.
std::expected<std::string, std::string> find_root(const std::string& dir);
/// Parse imu0/data.csv text.
std::expected<std::vector<ImuRow>, std::string> parse_imu(std::string_view text);
/// Parse a camN/data.csv text into (timestamp, file name).
std::expected<std::vector<std::pair<std::int64_t, std::string>>, std::string> parse_cam(std::string_view text);

/// A loaded dataset (images are not read).
struct Dataset {
    std::string root;
    std::vector<ImuRow> imu;
    std::vector<std::vector<std::pair<std::int64_t, std::string>>> cams;
    /// Load imu0 and every camN found.
    static std::expected<Dataset, std::string> load(const std::string& dir);
};

/// Result of estimate.
struct NoiseEstimate {
    std::pair<std::int64_t, std::int64_t> window{};
    double rate_hz = 0;
    Vec3 gyro_std{};
    Vec3 accel_std{};
    Vec3 gyro_mean{};
    Vec3 accel_mean{};
    cam::slam::ImuNoise noise;
};

/// Find the stillest window_s seconds (smallest summed gyro variance) and derive a noise model. Fails when the
/// stillest window still looks like motion (gyro std above 0.02 rad/s on any axis).
std::expected<NoiseEstimate, std::string> estimate(const std::vector<ImuRow>& imu, double window_s, double inflate);

/// Gyro bias from the first still window_s of a recording (mean gyro, rad/s, recording = body frame), the way the
/// driver measures it at start-up: the first window whose per-axis standard deviation stays below 0.004 rad/s (gyro)
/// and 0.03 m/s^2 (accel). Returns (bias, window start ns) or nullopt when the headset never lay still.
std::optional<std::pair<Vec3, std::int64_t>> still_gyro_bias(const std::vector<ImuRow>& imu, double window_s);

} // namespace pibridge::slamtool
