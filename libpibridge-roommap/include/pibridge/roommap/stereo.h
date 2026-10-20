// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-room/src/stereo.rs (MIT OR Apache-2.0).
//
// The camera rig from the factory calibration, per-camera ORB features with bearings, and stereo triangulation of
// a camera pair into 3D points in the IMU (body) frame. Features are extracted on a virtual pinhole view of each
// fisheye camera: every camera of the rig gets the SAME virtual orientation (upright, looking forward and down by the
// mean pitch of the first two cameras, the stereo pair), so the two views of the pair overlap and a point looks alike
// wherever it falls in the fisheye. Dream Air: the lower pair (cams 0/1, 11 cm baseline, diverging 54 deg).
#pragma once

#include "pibridge/cam/calib.h"
#include "pibridge/cam/geom.h"
#include "pibridge/cam/kb4.h"
#include "pibridge/roommap/orb.h"

#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace pibridge::roommap::stereo {

using cam::geom::Mat3;
using cam::geom::Pose;
using cam::geom::Vec3;

/// A virtual pinhole view of a fisheye camera (same size as the fisheye, its centre focal length).
struct Pinhole {
    std::size_t w = 0;
    std::size_t h = 0;
    double f = 0;
    double cx = 0;
    double cy = 0;
    /// Virtual camera axes (OpenCV) -> physical camera axes.
    Mat3 cam_from_virt{};
    /// Fisheye pixel for each pinhole pixel (NaN = outside).
    std::vector<std::pair<float, float>> lut;

    static Pinhole make(const cam::Kb4& k, const Mat3& cam_from_virt);
    /// Upright virtual view looking forward with `pitch_rad` (negative = down) in the body frame (OpenXR axes).
    static Mat3 body_from_virt(double pitch_rad);
    /// Undistort a fisheye image (bilinear); pixels outside the fisheye are 0.
    std::vector<std::uint8_t> undistort(const orb::Image& img) const;
    /// Is the pinhole pixel covered by the fisheye?
    bool covered(float x, float y) const;
    /// Unit bearing (physical camera frame) of a pinhole pixel.
    Vec3 bearing(double x, double y) const;
};

/// One camera of the rig.
struct RigCam {
    /// Factory camera id.
    std::size_t id = 0;
    cam::Kb4 intr;
    /// body_from_cam (body = IMU frame, OpenXR axes; camera = OpenCV axes).
    Pose body_from_cam;
    Pinhole pinhole;
};

/// The cameras used for the room map, in rig order (the first two are the stereo pair).
struct Rig {
    std::vector<RigCam> cams;
    /// Rig of factory cameras `ids`.
    static std::expected<Rig, std::string> from_calib(const cam::DeviceCalib& cal, std::span<const std::size_t> ids);
    std::vector<Pose> body_from_cams() const;
};

/// Features of one image.
struct Features {
    std::vector<orb::Keypoint> kps;
    std::vector<orb::Descriptor> desc;
    /// Unit bearing in the camera frame per keypoint.
    std::vector<Vec3> bearings;
    /// The undistorted (virtual pinhole) image (for sub-pixel stereo).
    std::vector<std::uint8_t> img;
    std::size_t w = 0;
};

/// Low-light contrast stretch before ORB.
struct LowLight {
    std::uint8_t p99_below = 100;
    std::uint8_t target = 200;
    float max_gain = 6.0f;
    bool denoise = true;
    bool operator==(const LowLight&) const = default;
};
/// The default: only dark frames (p99 < 100) are stretched, up to 6x, p99 to 200, 3x3 box filter first.
inline constexpr LowLight LOW_LIGHT{100, 200, 6.0f, true};

/// The stretch (black level kept, the 99th percentile of the lit pixels to `target`); returns the gain (1 = none).
float low_light_stretch(std::span<std::uint8_t> img, std::size_t w, const LowLight& p);

/// ORB features of one fisheye camera image on its virtual pinhole image, with the low-light stretch.
Features features(const orb::Image& img, const RigCam& cam, const orb::OrbParams& p);
/// features with a chosen stretch (nullopt: off).
Features features_with(const orb::Image& img, const RigCam& cam, const orb::OrbParams& p,
                       const std::optional<LowLight>& low);

/// Sub-pixel x in image b of the 9x9 patch at (xa, ya) in image a (NCC along the row, +-2 px, parabola).
std::optional<double> refine_x(const Features& fa, const Features& fb, double xa, double ya, double xb, double yb);

struct StereoParams {
    double epi_deg = 1.0;
    double tri_deg = 0.5;
    double min_depth = 0.25;
    double max_depth = 8.0;
    std::uint32_t max_dist = 85;
    float ratio = 0.9f;
    bool operator==(const StereoParams&) const = default;
};

/// A triangulated point.
struct StereoPoint {
    Vec3 p_body{};
    orb::Descriptor desc{};
    double depth = 0;
    std::size_t ia = 0;
};

/// Triangulate matches between camera `a` and camera `b` of the rig.
std::vector<StereoPoint> triangulate(const Features& fa, const Features& fb, const RigCam& a, const RigCam& b,
                                     const StereoParams& p);

} // namespace pibridge::roommap::stereo
