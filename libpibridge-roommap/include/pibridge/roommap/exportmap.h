// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/export.rs (MIT OR Apache-2.0).
//
// `export` / `view`: the room map as JSON and as one self-contained HTML5 page (data/viewer.html, plain WebGL/canvas,
// no network). Points of all keyframes are merged on a 2 cm grid (mean position, n = observations), keyframes carry
// their room-frame position and orientation, the floor and anchors come along. Room frame: y up, metres.
#pragma once

#include "pibridge/roommap/floor.h"
#include "pibridge/roommap/map.h"

#include <cstdint>
#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace pibridge::roommap::exportmap {

inline constexpr double CELL = 0.02;

/// Merged points: (x, y, z, observations), in order of first appearance.
std::vector<std::tuple<double, double, double, std::uint32_t>> merged_points(const map::RoomMap& map);
/// The map directory as JSON text (no stored floor: one detected from the map for the picture only).
std::expected<std::string, std::string> export_json(const std::string& dir, const std::string& name);
std::string map_json(const map::RoomMap& map, const std::string& name, const floor::StoredFloor* floor,
                     const std::optional<std::string>& meta_toml);
/// The self-contained page for a JSON export.
std::string html_page(const std::string& json, const std::string& name);
/// The embedded viewer page (generated from data/viewer.html).
std::string_view viewer_html();

} // namespace pibridge::roommap::exportmap
