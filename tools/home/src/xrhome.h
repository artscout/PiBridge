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
};

/// Run until the runtime ends the session, Monado goes away or `stop` is set (SIGTERM / SIGINT).
std::expected<void, std::string> runHome(const RunOpts& opts, const std::atomic<bool>& stop);

} // namespace pibridge::home
