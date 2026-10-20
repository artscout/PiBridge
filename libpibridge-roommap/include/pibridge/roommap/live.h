// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/live.rs (MIT OR Apache-2.0).
//
// Live input: tracking frames from the camera broker (or a .pxcam) and the tracker pose of each frame from Monado's
// head ring (libpibridge-anchor; raw SLAM poses, before the offset). Read only: nothing is sent to the headset, Monado or
// the broker. The rig's stereo pair is tiles `tiles` of the frame (Dream Air: tiles 0 and 1).
#pragma once

#include "pibridge/roommap/pngread.h"
#include "pibridge/roommap/session.h"

#include <array>
#include <cstdint>
#include <expected>
#include <functional>
#include <string>

namespace pibridge::roommap::live {

struct LiveFrame {
    /// Host CLOCK_MONOTONIC (ns).
    std::int64_t ts_ns = 0;
    std::array<pngread::Grey, 2> imgs;
    cam::geom::Pose slam_from_body;
    std::uint32_t pid = 0;
};

struct LiveStats {
    std::size_t frames = 0;
    std::size_t posed = 0;
    std::uint32_t pid = 0;
};

/// Read frames from `spec` for `seconds`; `f` gets every frame with a pose and returns false to stop.
std::expected<LiveStats, std::string> capture(const std::string& spec, double seconds,
                                              const std::function<bool(LiveFrame&)>& f,
                                              std::array<std::size_t, 2> tiles = {0, 1});
/// The offset Monado applies now for `pid` (identity when there is none), as a yaw + translation.
session::Alignment current_offset(std::uint32_t pid);

} // namespace pibridge::roommap::live
