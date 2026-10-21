// SPDX-License-Identifier: MIT
// pibridge-overlay's OpenVR side (overlay.cpp keeps libopenvr_api.so and openvr.h to itself; main.cpp needs none of it).
#pragma once

#include "options.h"

#include <atomic>

namespace pibridge::overlay {

/// Run the dashboard launcher until `stop` is set. Waits in-process for SteamVR to come up, and exits the process on
/// VREvent_Quit (the app restarts it) rather than re-init without a shutdown. Returns 0 on a clean stop.
int run(const Options& o, std::atomic<bool>& stop);

/// Write the manifest to `o.manifest` and turn on SteamVR auto-launch for the dashboard overlay. SteamVR must be running.
int registerWithSteamVr(const Options& o);
/// Remove the manifest and turn off auto-launch.
int unregisterWithSteamVr(const Options& o);

} // namespace pibridge::overlay
