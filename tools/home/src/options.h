// SPDX-License-Identifier: MIT
// pibridge-home's command line.
#pragma once

#include <expected>
#include <string>
#include <vector>

namespace pibridge::home {

struct Options {
    bool help = false;
    bool selfTest = false;    // --selftest: the scene, the session rules and the status file, no runtime, no GPU
    std::string snapshotDir;  // --snapshot DIR (with --selftest): the view as PPM through the software renderer
    std::string roomsDir;     // --rooms DIR: instead of ~/.config/pibridge/rooms
    unsigned resDiv = 1;      // --res-div N: divide the eye size
    std::string camera = "broker"; // --camera SPEC: the passthrough's frames (broker, a .pxcam, "off")
    std::string panelPpm;     // --panel-ppm FILE (with --snapshot): a picture of the app window instead of the test card
    bool passthroughOverlay = false; // --passthrough-overlay: the cameras only, as an overlay over the app (Monado)
    bool dashboardOverlay = false;   // --dashboard-overlay: the dashboard over a running game, on the menu button (Monado)
};

std::expected<Options, std::string> parseArgs(const std::vector<std::string>& args);
std::string usage();

} // namespace pibridge::home
