// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/overlay.rs (MIT OR Apache-2.0).
//
// Learned calibration overlay: small corrections on top of the factory calibration, kept in a separate file
// (default ~/.config/omarchyvr/cam-calib-overlay.json) so the factory blob is never modified.
//
// Version 1 holds per-camera rotation corrections about the camera centre, the output of pimax-selfcal. Semantics
// (rig convention X_i = R X_0 + t, the verified one): for camera i with rotation vector w_i (radians, OpenCV camera
// axes) the corrected rig is R_i' = Exp(w_i) R_i, t_i' = Exp(w_i) t_i, i.e. every cam_i_from_X gains a left factor
// Exp(w_i) and the camera centre stays where it is. Camera-IMU terms (ombc, tbc), intrinsics and translations are
// unchanged.
//
// Safety: with_overlay refuses an overlay made for another headset (device UID or factory calibration date), with
// unknown camera ids or with a correction above max_angle_deg. Callers then fall back to the factory calibration.
// save keeps the previous overlay as <file>.prev for rollback. Nothing loads the overlay implicitly: callers opt in
// (pibridge-cam ... --overlay FILE).
#pragma once

#include "pibridge/cam/calib.h"
#include "pibridge/cam/geom.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/json.h"

#include <cstdint>
#include <expected>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace pibridge::cam::overlay {

/// format field of the file.
inline constexpr std::string_view FORMAT = "pimax-cam-calib-overlay";
/// Current version.
inline constexpr std::uint32_t VERSION = 1;
/// Default refusal limit for one camera's correction (degrees).
inline constexpr double DEFAULT_MAX_ANGLE_DEG = 3.0;

/// Rotation correction of one camera.
struct CameraRotation {
    std::size_t id = 0;
    /// Rotation vector w (radians, camera axes): cam_from_X' = Exp(w) cam_from_X.
    geom::Vec3 rotation_vector{};
    /// One-sigma uncertainty per component (radians), if the estimator gave one.
    std::optional<geom::Vec3> sigma;
    /// Correction angle in degrees.
    double angle_deg() const;
    bool operator==(const CameraRotation&) const = default;
};

/// Overlay errors; text() is the Rust Display.
struct OverlayError {
    enum class Kind { Io, Json, Format };
    Kind kind = Kind::Format;
    std::string message;
    std::string path;
    std::string text() const;
};

/// The overlay document.
struct CalibOverlay {
    /// Headset serial the overlay belongs to (deviceUID of the factory XML). Empty: any headset.
    std::string device_uid;
    /// calibration_date of the factory XML it was estimated against. Empty: not checked.
    std::string factory_date;
    /// Seconds since the Unix epoch when it was written.
    std::uint64_t created_unix = 0;
    /// Producer (tool, version, command).
    std::string tool;
    /// Camera held fixed by the estimator (the gauge).
    std::size_t reference_camera = 0;
    std::vector<CameraRotation> rotations;
    /// Numeric quality figures (free-form, for humans and sanity checks).
    std::vector<std::pair<std::string, double>> quality;
    std::string note;

    /// An empty overlay for calib (no corrections).
    static CalibOverlay for_calib(const DeviceCalib& calib, std::string_view tool);
    /// Rotation of camera id (zero if absent).
    geom::Vec3 rotation(std::size_t id) const;
    /// Largest correction angle (degrees).
    double max_angle_deg() const;
    /// Parse the JSON text.
    static std::expected<CalibOverlay, OverlayError> parse(std::string_view text);
    /// Serialise (stable, human-readable layout).
    std::string to_json() const;
    /// Read and parse a file.
    static std::expected<CalibOverlay, OverlayError> load(const std::string& path);
    /// Why this overlay must not be applied to calib (nullopt: it may be applied).
    std::optional<std::string> refusal(const DeviceCalib& calib, double max_angle_deg) const;
    bool operator==(const CalibOverlay&) const = default;
};

/// Copy of calib with the overlay's rotation corrections applied. Errors if refusal objects; calib is never
/// modified.
std::expected<DeviceCalib, CalibError> with_overlay(const DeviceCalib& calib, const CalibOverlay& ov,
                                                    double max_angle_deg);

/// Default overlay path: $PIMAX_CAM_OVERLAY, else $XDG_CONFIG_HOME/omarchyvr/cam-calib-overlay.json, else
/// ~/.config/omarchyvr/cam-calib-overlay.json.
std::optional<std::string> default_path();
/// <path>.prev.
std::string prev_path(const std::string& path);
/// Write atomically (temp file + rename). An existing file is kept as <path>.prev (one level of rollback).
Result<void> save(const CalibOverlay& ov, const std::string& path);
/// Restore <path>.prev as the overlay (the current one becomes the new .prev, so a second rollback undoes the
/// first). Errors if there is no previous overlay.
Result<void> rollback(const std::string& path);
/// Delete the overlay and its .prev copy (back to the factory calibration). Returns the files removed.
Result<std::vector<std::string>> clear(const std::string& path);

} // namespace pibridge::cam::overlay
