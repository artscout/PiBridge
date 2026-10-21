// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/calib.rs (MIT OR Apache-2.0).
//
// Factory calibration (<DeviceConfiguration> XML served by the headset over IF0): four OV7251 fisheye cameras plus
// the camera/IMU extrinsic.
//
// Conventions (omarchyvr analysis/overnight/cam.md has the evidence):
// * Intrinsics: FISHEYE_4_PARAMETERS = OpenCV/Kannala-Brandt KB4 on the full 640x480 sensor image, OpenCV camera
//   axes (x right, y down, z forward). radial_distortion has 6 numbers; the first four are k1..k4.
// * <Rig> of camera i: X_i = R * X_0 + t (it maps camera-0 coordinates into camera i, "convention 2" of
//   camgeom.py). Camera 0 has the identity.
// * <SFConfig><Stateinit ombc tbc>: ombc is the rotation vector of R_bc, the rotation from camera-0 coordinates into
//   the body frame (X_b = R_bc X_c + tbc). With these two conventions the A-B baseline lies on body +X to 1e-2 and
//   the optical axes are symmetric about the body YZ plane, looking along -Z and 29 deg down, with the image upside
//   down. The body frame is therefore the OpenXR head frame (x right, y up, z back), i.e. the raw IMU axes with X
//   and Z negated, which is the Monado driver's PIMAX_AXIS_MODE=1.
#pragma once

#include "pibridge/cam/geom.h"
#include "pibridge/cam/kb4.h"
#include "pibridge/cam/xml.h"

#include <expected>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pibridge::cam {

/// The REFERENCE factory calibration of the Pimax Dream Air, shipped for everyone (omarchyvr user decision
/// 2026-10-17): omarchyvr's own headset's Windows calibration with the serial scrubbed (deviceUID="reference";
/// PiBridge devices/dream-air/reference/headset-calibration.xml, embedded at build time). Used only when no
/// calibration of the user's own headset is found.
std::string_view reference_xml();

/// deviceUID of reference_xml().
inline constexpr std::string_view REFERENCE_UID = "reference";

/// Where omarchyvr installs the reference with its data copy, relative to $HOME.
inline constexpr std::string_view REFERENCE_INSTALLED =
    ".local/share/omarchyvr/data/rust/pimax-device/devices/dream-air/reference/headset-calibration.xml";

/// The reference fallback is on unless PIMAX_NO_REFERENCE is set to something other than 0.
bool reference_allowed(const std::function<std::optional<std::string>(std::string_view)>& env);

/// The path reported for the reference: the installed copy when it exists, else a name that says it is built in.
std::string reference_path(const std::optional<std::string>& home);

/// Calibration of one camera.
struct CameraCalib {
    std::uint32_t id = 0;
    /// name attribute (trackingA, trackingB, Pi-controllerA, Pi-controllerB).
    std::string name;
    /// model attribute of <Calibration>.
    std::string model;
    Kb4 intr;
    /// <Rig rowMajorRotationMat>.
    geom::Mat3 rig_r = geom::IDENTITY;
    /// <Rig translation> (metres).
    geom::Vec3 rig_t{};
    /// <TimeAlignment delta> (seconds).
    double time_delta = 0;
    bool operator==(const CameraCalib&) const = default;
};

/// Camera-IMU terms from <SFConfig><Stateinit>.
struct ImuExtrinsic {
    /// Rotation vector of R_bc (camera 0 -> body).
    geom::Vec3 ombc{};
    /// Camera 0 origin in the body frame (metres).
    geom::Vec3 tbc{};
    /// delta: camera-IMU time offset (seconds). Not applied by this library.
    double delta_s = 0;
    /// All Stateinit attributes as raw strings (biases, scales, ...).
    std::vector<std::pair<std::string, std::string>> raw;
    bool operator==(const ImuExtrinsic&) const = default;
};

/// How to read <Rig>.
enum class RigConvention {
    /// X_i = R X_0 + t (default, verified).
    CamFromCam0,
    /// X_0 = R X_i + t (alternative, for hypothesis tests).
    Cam0FromCam,
};

/// "CamFromCam0" / "Cam0FromCam" (Rust Debug).
const char* convention_name(RigConvention c);

/// Error while reading the calibration; text() is the Rust Display.
struct CalibError {
    enum class Kind { Io, Xml, Format, NotFound };
    Kind kind = Kind::Format;
    std::string message;             ///< Io: the io error text; Xml: the XML error text; Format: the detail
    std::string path;                ///< Io: the file
    std::vector<std::string> tried;  ///< NotFound: the candidates
    std::string text() const;
};

/// The whole document.
struct DeviceCalib {
    std::string uid;
    std::string sensor;
    std::string date;
    std::string version;
    /// Cameras sorted by id.
    std::vector<CameraCalib> cameras;
    /// Camera-IMU extrinsic if present.
    std::optional<ImuExtrinsic> imu;

    /// Parse the XML text.
    static std::expected<DeviceCalib, CalibError> parse(std::string_view doc);
    /// Read and parse a file.
    static std::expected<DeviceCalib, CalibError> load(const std::string& path);
    /// Default search order: $PIMAX_CAM_CALIB, $PIMAX_CALIB, ~/.config/pibridge/headset-calibration.xml, then
    /// ~/.config/pimax-linux/headset-calibration.xml and ~/.config/omarchyvr/headset-calibration.xml (the last two
    /// only when present, docs/CONVENTIONS.md), <repo>/analysis/headset-calibration.xml (the installed data copy
    /// ~/.local/share/omarchyvr/data, else ~/omarchyvr; $PIMAX_REPO overrides).
    static std::vector<std::string> default_candidates();
    /// Load from path, or from the first existing default candidate, else the shipped reference (uid "reference";
    /// $PIMAX_NO_REFERENCE=1 turns that fallback off). The headset's own file always wins.
    static std::expected<std::pair<DeviceCalib, std::string>, CalibError> load_default(
        const std::optional<std::string>& path);
    /// load_default with explicit candidates.
    static std::expected<std::pair<DeviceCalib, std::string>, CalibError> load_first(
        const std::vector<std::string>& cands, bool reference, const std::optional<std::string>& home);

    /// True for the shipped reference.
    bool is_reference() const { return uid == REFERENCE_UID; }
    /// cam_i_from_cam0 under a rig convention.
    geom::Pose cam_from_cam0(std::size_t i, RigConvention conv) const;
    /// Stereo extrinsic b_from_a: X_b = R X_a + t (OpenCV stereoCalibrate R, T; Monado
    /// t_stereo_camera_calibration::camera_rotation/translation).
    geom::Pose stereo(std::size_t a, std::size_t b, RigConvention conv) const;
    /// body_from_cam0 from ombc/tbc (body = OpenXR head axes).
    std::optional<geom::Pose> body_from_cam0() const;
    /// body_from_cam_i.
    std::optional<geom::Pose> body_from_cam(std::size_t i, RigConvention conv) const;
    /// Human-readable geometry summary (centres, optical axes, baselines, FOV), the text of `pimax-cam calib`.
    std::string geometry_report(RigConvention conv) const;

    bool operator==(const DeviceCalib&) const = default;
};

} // namespace pibridge::cam
