// SPDX-License-Identifier: MIT
// pibridge-home's OpenXR side (tools/xrkit for the session, the swapchains and the drawing): the session in STAGE space
// (LOCAL when there is none), the controllers' grip and aim poses (Touch, Index, simple controller bindings: what the
// Pimax driver in Monado offers), and the frame loop under the rules of loop.h. Writes home.json for the app.
#pragma once

#include "scene.h"

#include <atomic>
#include <expected>
#include <string>

namespace pibridge::home {

struct RunOpts {
    std::string roomsDir;
    unsigned resDiv = 1;
    std::string camera = "broker";
    // --passthrough-overlay: only the room through the cameras, as a Monado overlay over the running app (the Godot
    // world's Passthrough switch); no panel, no grid, no status file (the home's own one stays the session home's)
    bool passthroughOverlay = false;
    // --dashboard-overlay: the dashboard as a Monado overlay over a running game, opened and closed with the menu
    // button (not while PiBridge World is the main app: it has its own); the game's inputs are blocked while it is open
    bool dashboardOverlay = false;
};

/// Run until the runtime ends the session, Monado goes away or `stop` is set (SIGTERM / SIGINT).
std::expected<void, std::string> runHome(const RunOpts& opts, const std::atomic<bool>& stop);

} // namespace pibridge::home
