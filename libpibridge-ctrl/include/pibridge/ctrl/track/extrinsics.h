// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/extrinsics.rs (MIT OR Apache-2.0).
//
// Camera extrinsic corrections fitted on controller poses (`track --fit-extrinsics --save-extrinsics P`). Stores the
// full corrected `cam_from_body` of each refitted camera; `live`/`track` load them with `--extrinsics` (default
// `~/.config/omarchyvr/ctrl-cam-extrinsics.json` when it exists).
//
// The shipped Dream Air reference has the fit of the reference headset (reference_extrinsics_json()): cameras 1-3
// corrected by 0.46, 1.45 and 3.20 deg (mostly pitch). Without it the controllers come out a few degrees tilted.
// reference_for applies it only to a headset whose factory camera rig equals the reference's, i.e. the same headset;
// the user's own file always wins.
#pragma once

#include "pibridge/cam/calib.h"
#include "pibridge/cam/geom.h"
#include "pibridge/ctrl/track/model.h"

#include <cstddef>
#include <expected>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pibridge::ctrl::track::extrinsics {

/// (camera id, cam_from_body).
using CamPoses = std::vector<std::pair<std::size_t, cam::geom::Pose>>;

/// The camera extrinsics refit of the reference headset (serial scrubbed, `device_uid` "reference"): PiBridge
/// devices/dream-air/reference/ctrl-cam-extrinsics.json, embedded at build time.
std::string_view reference_json();

/// `~/.config/omarchyvr/ctrl-cam-extrinsics.json`.
std::string default_path();

/// Write the file (through a `.json.tmp` and a rename); the error is the Rust io::Error text.
std::expected<void, std::string> save(const std::string& path, std::string_view device_uid,
                                      std::span<const std::pair<std::size_t, cam::geom::Pose>> cams,
                                      std::string_view note);

std::expected<std::pair<std::string, CamPoses>, std::string> load(const std::string& path);

/// Parse the file text: (device_uid, [(camera id, cam_from_body)]).
std::expected<std::pair<std::string, CamPoses>, std::string> parse(std::string_view t);

/// True when `calib` has the same factory camera rig (every camera's body_from_cam: rig rotation and translation
/// plus the camera to IMU extrinsic) as the shipped reference.
bool same_rig_as_reference(const cam::DeviceCalib& calib);

/// The shipped reference extrinsics for `calib` when it is the reference headset (same factory rig) and the
/// reference is allowed (`PIMAX_NO_REFERENCE` unset or 0). nullopt for any other headset.
std::optional<CamPoses> reference_for(const cam::DeviceCalib& calib, const Env& env);

/// The corrections to use on top of the factory extrinsics of `calib`, and a note for the log: the user's own fit at
/// `path` (default default_path()) when it exists and belongs to this headset, else the shipped reference fit for
/// the reference headset, else none ("factory extrinsics"). Prints why a file was ignored to stderr.
std::pair<CamPoses, std::string> resolve(const cam::DeviceCalib& calib, const std::optional<std::string>& path,
                                         const Env& env);

} // namespace pibridge::ctrl::track::extrinsics
