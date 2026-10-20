// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/solve.rs (MIT OR Apache-2.0).
//
// Constellation solve: blobs -> LED identities -> pose, per camera and jointly over all four cameras.
//
// Acquisition is brute force without a prior: each blob plus two of its nearest neighbours is tried against every
// ordered triple of neighbouring, co-visible LEDs; Grunert P3P gives up to four poses; each pose is scored by how
// many other facing LEDs land on a blob (angular test on unit bearings, no distortion maths in the inner loop). The
// winner is refined with Levenberg-Marquardt on the pixel reprojection error (KB4 fisheye), re-matched and refined
// again. Tracking with a prior (project last pose, nearest blob, refine) skips the brute force.
#pragma once

#include "pibridge/cam/geom.h"
#include "pibridge/cam/kb4.h"
#include "pibridge/ctrl/track/model.h"

#include <array>
#include <cstddef>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace pibridge::ctrl::track {

/// One camera of the rig.
struct Cam {
    cam::Kb4 intr;
    /// `cam_from_body` (body = OpenXR head axes, from the factory calibration).
    cam::geom::Pose cam_from_body;
};

/// Blobs of one camera in one LED frame.
struct CamObs {
    std::vector<std::pair<double, double>> px;
    /// Unit bearings in the camera frame (nullopt outside the model's disc).
    std::vector<std::optional<cam::geom::Vec3>> bearing;
    static CamObs make(const Cam& cam, std::vector<std::pair<double, double>> px);
};

struct Params {
    /// Angular inlier tolerance for hypothesis scoring (radians).
    double tol_rad = 3.0 / 230.0;
    /// Pixel inlier tolerance after refinement.
    double tol_px = 2.5;
    /// Pixel tolerance when matching from a prior pose.
    double prior_tol_px = 12.0;
    /// An LED counts as visible when its normal is within this angle of the direction to the camera.
    double facing_deg = 80.0;
    double min_dist = 0.07;
    double max_dist = 1.5;
    /// Nearest neighbours per anchor blob used to form blob triples.
    std::size_t knn = 5;
    /// Minimum matched LEDs to accept a single-camera brute-force solve.
    std::size_t min_inliers = 5;
    /// Stop the brute-force search once a hypothesis matches this many LEDs.
    std::size_t good_enough = 9;
};

/// (led index, blob index) pairs.
using Matches = std::vector<std::pair<std::size_t, std::size_t>>;

/// A pose with its LED <-> blob matches in one camera.
struct CamSolve {
    std::size_t cam = 0;
    cam::geom::Pose cam_from_model;
    Matches matches;
    double rms_px = 0;
};

/// The LED's unit direction from the camera when it faces the camera (normal within the facing cone), else nullopt.
std::optional<cam::geom::Vec3> facing(const cam::geom::Pose& cam_from_model, const Led& led, double cos_lim);

/// Pixel-space matching from a pose (used after refinement and for priors).
Matches match_pixels(const cam::geom::Pose& pose, const LedModel& model, const Cam& cam, const CamObs& obs,
                     const std::vector<bool>& used, double cos_face, double tol_px);

/// A pixel observation for joint refinement: camera, model point, measured pixel.
struct Obs {
    const Cam* cam = nullptr;
    cam::geom::Vec3 p{};
    std::pair<double, double> px{};
};

/// Levenberg-Marquardt over `base_from_model`; each observation's camera sees `cam_from_base * base_from_model`
/// (`cam_from_base` = identity for a single-camera solve, `cam_from_body` for a joint one). Returns (pose, rms px).
std::pair<cam::geom::Pose, double> refine(const cam::geom::Pose& pose, std::span<const Obs> obs, bool joint,
                                          std::size_t iters);

/// Rotation vector (axis * angle, rad) of a rotation matrix.
cam::geom::Vec3 rotvec(const cam::geom::Mat3& r);

/// `refine` with an optional rotation prior `(r_prior, w)`: three extra residuals `w * rotvec(R r_prior^T)` (w in px
/// per rad), so 3-4 LED solves stay near the predicted orientation (the constrained few-LED track). The returned rms
/// is over the pixel residuals only.
std::pair<cam::geom::Pose, double> refine_with(const cam::geom::Pose& pose, std::span<const Obs> obs, bool joint,
                                               std::size_t iters,
                                               const std::optional<std::pair<cam::geom::Mat3, double>>& rot_prior);

/// Rotation matrix of a quaternion [x, y, z, w] (as `geom::quat_from_mat`), normalised first.
cam::geom::Mat3 quat_to_mat(const std::array<double, 4>& q);

/// Refine a single-camera pose on its matches, re-match in pixels and refine again.
std::optional<CamSolve> polish(const cam::geom::Pose& pose, const LedModel& model, const Cam& cam,
                               std::size_t cam_idx, const CamObs& obs, const std::vector<bool>& used,
                               const Params& prm, const Matches& matches);

/// Brute-force acquisition of one controller in one camera (no prior). `used` marks blobs already claimed.
std::optional<CamSolve> acquire(const LedModel& model, std::span<const std::array<std::size_t, 3>> triples,
                                const Cam& cam, std::size_t cam_idx, const CamObs& obs, const std::vector<bool>& used,
                                const Params& prm);

struct JointSolve {
    cam::geom::Pose body_from_model;
    /// Matches per camera.
    std::vector<Matches> matches;
    double rms_px = 0;
    std::size_t total() const;
    std::size_t cams() const;
};

/// Track from a prior `body_from_model`: match in every camera and refine jointly.
std::optional<JointSolve> joint_from_prior(const cam::geom::Pose& body_from_model, const LedModel& model,
                                           std::span<const Cam> cams, std::span<const CamObs> obs,
                                           std::span<const std::vector<bool>> used, double tol_px, double facing_deg);

} // namespace pibridge::ctrl::track
