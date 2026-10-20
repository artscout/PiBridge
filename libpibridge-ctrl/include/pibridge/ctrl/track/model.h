// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/model.rs (MIT OR Apache-2.0).
//
// Controller LED model from Pimax's `controller_params.json` (Windows `ProgramData/Pimax/slam/H1/<serial>/`).
//
// 15 `ModelPoints` per controller, each `[x, y, z, nx, ny, nz]` in metres in the controller ("device") frame, the
// unit LED normal, plus `ImuToDevice` (row-major 4x4) and `Hand`. Left and right are mirror images in x.
#pragma once

#include "pibridge/cam/geom.h"

#include <array>
#include <cstddef>
#include <expected>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pibridge::ctrl::track {

/// An environment lookup (std::env::var): nullopt when unset.
using Env = std::function<std::optional<std::string>(std::string_view)>;

/// The process environment (std::env::var: unset or not UTF-8 -> nullopt).
Env process_env();

/// The REFERENCE controller LED models of the Pimax Dream Air, shipped for everyone (omarchyvr user decision
/// 2026-10-17, "approve shipping calibrations"): Pimax's own `controller_params.json` of omarchyvr's Windows install,
/// the serial folders renamed to left/right in the order the tracker always used them. The per-serial files equal
/// Pimax's default `test_0`/`test_1` models (rounding only, no per-unit LED calibration), so the reference fits any
/// Dream Air controller pair. PiBridge devices/dream-air/reference/controllers/{left,right}.json, embedded at build time.
std::string_view reference_left();
/// See reference_left().
std::string_view reference_right();

/// Where the LED models in use came from: the user's own files (`files`), or the shipped reference (`files` empty).
struct ModelSource {
    std::optional<std::array<std::string, 2>> files;
    /// One plain line for the logs and the app.
    std::string describe() const;
    bool operator==(const ModelSource&) const = default;
};

struct Led {
    cam::geom::Vec3 p{};
    cam::geom::Vec3 n{};
};

struct LedModel {
    std::string hand;
    std::string serial;
    std::vector<Led> leds;
    /// `device_from_imu` (`ImuToDevice`).
    cam::geom::Pose imu_to_device;

    static std::expected<LedModel, std::string> parse(std::string_view text);
    static std::expected<LedModel, std::string> load(const std::string& path);

    /// The directories searched for the user's own model files, in order: `$PIMAX_CTRL_MODEL_DIR`, else
    /// `~/.config/omarchyvr/controllers` (where the app imports them), else `~/.local/share/omarchyvr/controller-models`,
    /// else the Windows dump in the repo (`<repo>/pimax-data/ProgramData-Pimax/slam/H1`; repo = `$PIMAX_REPO`, the
    /// installed data copy, or `~/omarchyvr`). Each holds `left.json` + `right.json` or one folder per controller
    /// serial with a `controller_params.json` (the Pimax `slam/H1/<serial>/` layout; the two folders are taken in name
    /// order, as the tracker has always used them).
    static std::vector<std::string> search_dirs(const Env& env);
    /// The user's own model files (left, right), if any: the first pair in search_dirs.
    static std::optional<std::array<std::string, 2>> own_paths(const Env& env);
    /// Default model files (left, right): the user's own; with none found the paths point at
    /// `~/.config/omarchyvr/controllers/{left,right}.json` so an error names where they go.
    static std::array<std::string, 2> default_paths();
    /// The models the tracker uses: `left`/`right` when given, else the user's own, else the shipped reference (unless
    /// `PIMAX_NO_REFERENCE=1`). An unreadable or broken own file is an error (never a crash).
    static std::expected<std::pair<std::array<LedModel, 2>, ModelSource>, std::string> resolve(
        const std::optional<std::string>& left, const std::optional<std::string>& right, const Env& env);
    /// The first two `<dir>/<serial>/controller_params.json` in folder-name order (or `left.json`/`right.json`).
    static std::optional<std::array<std::string, 2>> pair_in(const std::string& dir);

    /// Ordered LED triples that can be seen together and are close: used as P3P hypotheses.
    /// `max_dist` metres between any two, normals pairwise within `max_normal_deg`.
    std::vector<std::array<std::size_t, 3>> neighbour_triples(double max_dist, double max_normal_deg) const;
};

} // namespace pibridge::ctrl::track
