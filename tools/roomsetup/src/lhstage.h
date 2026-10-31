// SPDX-License-Identifier: MIT
// The Lighthouse stage of the Monado backend (pibridge-roomsetup --lh-stage): where the play area's centre, forward and
// floor are in the Lighthouse space. steamvr_lh (PiBridge's Monado fork) applies it to the head and the controllers
// from LH_STAGE_FILE, re-read while Monado runs; the room setup writes it. SteamVR's chaperone is never involved: each
// backend keeps its own room (docs/plans/MONADO-BACKEND.md).
//
// The file: one line "px py pz qx qy qz qw" = stage_from_raw. The head the setup reads (the Pimax driver's head ring in
// this mode) is in the stage that was live then, so a new stage is the correction composed onto the old one.
#pragma once

#include "recenter.h"

#include "pibridge/room/draw.h"

#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::roomsetup {

using StagePose = std::array<double, 7>; // px py pz qx qy qz qw

inline constexpr StagePose kStageIdentity{0, 0, 0, 0, 0, 0, 1};

std::optional<StagePose> parseStage(std::string_view text);
std::string stageText(const StagePose& p);

/// The new stage_from_raw: the head of `r` (in the old stage) becomes the centre, its forward -Z, and the floor
/// r.floorStage (the controller touch), else head y - `eyeHeight`, else the old floor stays.
StagePose lhStageFromRecenter(const StagePose& oldStage, const RecenterResult& r, std::optional<double> eyeHeight);

/// $XDG_RUNTIME_DIR/pibridge/monado/lh-stage: the live file steamvr_lh reads (LH_STAGE_FILE).
std::string lhStageRuntimePath();
/// <room dir>/lh-stage: the saved one, copied to the live file at every Monado start.
std::string lhStageRoomPath(const std::string& roomDir);

/// The base stations where they stand, in the live stage: Monado's steamvr_lh writes
/// $XDG_RUNTIME_DIR/pibridge/monado/basestations, a line "<serial> px py pz qx qy qz qw" each, once a second.
std::string baseStationsPath();
/// The poses of that file; malformed lines are left out.
std::vector<room::Mat34> parseBaseStations(std::string_view text);
/// Each base station as a small wire box with a mark on its front (-Z, where it sweeps), so the room setup shows
/// where they are.
void drawBaseStations(room::Batch& b, const std::vector<room::Mat34>& stations);

} // namespace pibridge::roomsetup
