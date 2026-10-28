// SPDX-License-Identifier: MIT
// pibridge-overlay: the PiBridge buttons in the SteamVR dashboard bar -- Room Setup, Quick Room Setup, Record Video
// (docs/OVERLAY.md) -- and the in-game recenter ring. The app starts it; a click writes a request the app acts on.
#include "options.h"
#include "overlay.h"
#include "overlay_core.h"

#include <atomic>
#include <csignal>
#include <cstdio>
#include <string>
#include <vector>

#ifndef PIBRIDGE_OVERLAY_ICON
#define PIBRIDGE_OVERLAY_ICON ""
#endif
#ifndef PIBRIDGE_OVERLAY_BINARY
#define PIBRIDGE_OVERLAY_BINARY "pibridge-overlay"
#endif
#ifndef PIBRIDGE_OVERLAY_MANIFEST
#define PIBRIDGE_OVERLAY_MANIFEST "pibridge.vrmanifest"
#endif

namespace {

std::atomic<bool> g_stop{false};

extern "C" void onSignal(int) { g_stop.store(true); }

// A few no-runtime checks, run as the binary's self test (ctest and the app's preflight).
int runSelfTest() {
    using namespace pibridge;
    int bad = 0;
    const auto fail = [&](const char* what) {
        std::fprintf(stderr, "pibridge-overlay --selftest: %s\n", what);
        bad++;
    };
    using overlay::Tab;
    using overlay::TabState;
    const TabState idle{};
    // a page draws at its fixed size (a few hundred px, not 2048), an icon at its own
    if (overlay::drawPage(Tab::Room, idle, false).size() != static_cast<std::size_t>(overlay::kPanelW) * overlay::kPanelH * 4U ||
        overlay::drawIcon(Tab::Record, idle).size() != static_cast<std::size_t>(overlay::kIconSize) * overlay::kIconSize * 4U) {
        fail("drawPage / drawIcon size wrong");
    }
    if (!(overlay::kPanelW <= 1024 && overlay::kPanelH <= 1024)) {
        fail("panel is too big (must be small, not a full window)");
    }
    // hover lights the button; the recording state changes the record page and its bar icon
    if (overlay::drawPage(Tab::Room, idle, true) == overlay::drawPage(Tab::Room, idle, false)) {
        fail("hover does not change the page");
    }
    TabState rec;
    rec.recording = true;
    if (overlay::drawPage(Tab::Record, rec, false) == overlay::drawPage(Tab::Record, idle, false) ||
        overlay::drawIcon(Tab::Record, rec) == overlay::drawIcon(Tab::Record, idle)) {
        fail("recording state does not change the record page / icon");
    }
    // the laser hit test: GL space is bottom-left; the big button's centre, and a corner off it
    const float ph = static_cast<float>(overlay::kPanelH);
    if (!overlay::hitButton(260.0F, ph - 102.0F) || overlay::hitButton(5.0F, 5.0F)) {
        fail("hitButton wrong");
    }
    TabState busy;
    busy.roomSetupRunning = true;
    if (overlay::requestFor(Tab::Room, idle) != "room-setup" || overlay::requestFor(Tab::QuickRoom, idle) != "quick-room-setup" ||
        overlay::requestFor(Tab::Record, idle) != "record-start" || overlay::requestFor(Tab::Record, rec) != "record-stop" ||
        !overlay::requestFor(Tab::Room, busy).empty() || !overlay::requestFor(Tab::QuickRoom, busy).empty()) {
        fail("requestFor wrong");
    }
    const std::string m = overlay::manifestJson("pibridge.dashboard", "/bin/x", "/ico.png", "PiBridge");
    if (m.find("\"is_dashboard_overlay\": true") == std::string::npos || m.find("pibridge.dashboard") == std::string::npos) {
        fail("manifestJson missing fields");
    }
    if (bad == 0) {
        std::printf("pibridge-overlay --selftest: ok\n");
    }
    return bad == 0 ? 0 : 1;
}

} // namespace

int main(int argc, char** argv) {
    using namespace pibridge::overlay;
    const std::vector<std::string> args(argv + 1, argv + argc);
    auto o = parseArgs(args);
    if (!o) {
        std::fprintf(stderr, "pibridge-overlay: %s\n\n%s", o.error().c_str(), usage().c_str());
        return 2;
    }
    if (o->help) {
        std::fputs(usage().c_str(), stdout);
        return 0;
    }
    if (o->selfTest) {
        return runSelfTest();
    }
    // defaults from the install paths (the user never types these)
    if (o->icon.empty()) {
        o->icon = PIBRIDGE_OVERLAY_ICON;
    }
    if (o->binary.empty()) {
        o->binary = PIBRIDGE_OVERLAY_BINARY;
    }
    if (o->manifest.empty()) {
        o->manifest = PIBRIDGE_OVERLAY_MANIFEST;
    }
    if (o->printManifest) {
        std::fputs(manifestJson("pibridge.dashboard", o->binary, o->icon, "PiBridge").c_str(), stdout);
        return 0;
    }
    if (o->doRegister) {
        return registerWithSteamVr(*o);
    }
    if (o->doUnregister) {
        return unregisterWithSteamVr(*o);
    }
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    return run(*o, g_stop);
}
