// SPDX-License-Identifier: MIT
// pibridge-overlay's command line. The user never types these: the app's Play/Start and its SteamVR panel switch drive
// the binary, and the installer registers it with SteamVR (see docs/OVERLAY.md). The defaults come from the install
// paths at build time.
#pragma once

#include <expected>
#include <string>
#include <vector>

namespace pibridge::overlay {

struct Options {
    bool help = false;
    bool selfTest = false;      // --selftest: the pure parts, no runtime, no GPU
    bool printManifest = false; // --print-manifest: write the .vrmanifest JSON to stdout and exit
    bool doRegister = false;    // --register: write the manifest and turn on auto-launch in SteamVR (SteamVR must run)
    bool doUnregister = false;  // --unregister: remove the manifest and turn off auto-launch
    std::string icon;           // --icon PATH: the tab icon (default: the installed pibridge.png)
    std::string binary;         // --binary PATH: the overlay binary named in the manifest (default: the installed one)
    std::string manifest;       // --manifest PATH: where the .vrmanifest is written / read (default: the installed one)
    double width = 0.6;         // --width M: the small launcher panel's width in the dashboard (0.2 .. 3)
};

std::expected<Options, std::string> parseArgs(const std::vector<std::string>& args);
std::string usage();

} // namespace pibridge::overlay
