// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/track.rs (MIT OR Apache-2.0): the part shared by `track`, `live` and the
// tracker (the rig, one LED frame's solve, the blob claims). The `track` command itself is in tools/ctrltrack.
#pragma once

#include "pibridge/cam/calib.h"
#include "pibridge/cam/geom.h"
#include "pibridge/ctrl/track/model.h"
#include "pibridge/ctrl/track/solve.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>
#include <string>
#include <tuple>
#include <vector>

namespace pibridge::ctrl::track {

/// The blobs of one LED frame: record number (or sequence), device time, per camera the blob centres.
struct LedFrame {
    std::size_t rec = 0;
    std::uint64_t ts_us = 0;
    std::vector<std::vector<std::pair<double, double>>> px;
};

/// One controller found in a frame.
struct Found {
    std::size_t model = 0;
    JointSolve joint;
    /// Joint fit of the other hand's model from the same start (L/R discrimination).
    std::optional<JointSolve> alt;
    /// Largest distance between a single-camera solve and the joint pose (m), for camera pairs.
    double cam_spread_m = 0;
    std::size_t single = 0;
};

/// Everything the solver needs: calibration, (corrected) cameras, LED models, hypothesis triples.
struct Rig {
    cam::DeviceCalib calib;
    std::vector<Cam> cams;
    std::array<LedModel, 2> models;
    std::vector<std::vector<std::array<std::size_t, 3>>> triples;
    Params prm;

    /// `--calib XML --left JSON --right JSON --extrinsics JSON|--factory`. The calibration and the LED models fall
    /// back to the shipped Dream Air reference when the user has none of their own (the log lines on stderr say which
    /// is used). The error is the plain reason (the tool adds "pimax-ctrltrack: " and exits 2).
    static std::expected<Rig, std::string> try_load(std::span<const std::string> a);
};

/// The value after the first `k` in `a` (Rust `arg`).
std::optional<std::string> arg(std::span<const std::string> a, std::string_view k);

/// A previous fix used as a prior: (model, body_from_model, ts).
using Prior = std::optional<std::tuple<std::size_t, cam::geom::Pose, std::uint64_t>>;

/// Solve one LED frame. With `prior`, track each previous controller first; acquisition is skipped. `pre` are fixes
/// already made this frame (their blobs are not used again). `parallel` runs the per-camera acquisition on one thread
/// per camera (the result is the same).
std::vector<Found> solve_frame(const LedFrame& f, std::span<const Cam> cams, const std::array<LedModel, 2>& models,
                               std::span<const std::vector<std::array<std::size_t, 3>>> triples, const Params& prm,
                               const std::vector<Prior>* prior, std::span<const Found> pre, bool parallel);

/// Mark the blobs of a joint solve as used.
void claim(std::vector<std::vector<bool>>& used, const JointSolve& j);

/// Two fixes closer than 6 cm are the same controller.
bool same_object(const Found& a, const Found& b);

} // namespace pibridge::ctrl::track
