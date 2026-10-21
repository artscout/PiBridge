// SPDX-License-Identifier: MIT
// pibridge-overlay: the PiBridge window as a SteamVR dashboard tab (docs/OVERLAY.md). SteamVR auto-launches it once it
// is registered; it mirrors the app's window from the shared frame and sends the controller laser back as mouse input.
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
    // the panel draws at its fixed size (a few hundred px, not 2048)
    if (overlay::drawPanel(overlay::Button::None).size() !=
        static_cast<std::size_t>(overlay::kPanelW) * overlay::kPanelH * 4U) {
        fail("drawPanel size wrong");
    }
    if (!(overlay::kPanelW <= 1024 && overlay::kPanelH <= 1024)) {
        fail("panel is too big (must be small, not a full window)");
    }
    // hover lights one button: the pixels differ from the resting panel
    if (overlay::drawPanel(overlay::Button::RoomSetup) == overlay::drawPanel(overlay::Button::None)) {
        fail("hover does not change the panel");
    }
    // the laser hit test: GL space is bottom-left, so a point high on the screen (large y) hits the top button.
    // Centres of the four stacked buttons (top to bottom).
    const float ph = static_cast<float>(overlay::kPanelH);
    if (overlay::hitTest(256.0F, ph - 60.0F) != overlay::Button::RoomSetup) {
        fail("hitTest ROOM SETUP wrong");
    }
    if (overlay::hitTest(256.0F, ph - 154.0F) != overlay::Button::QuickRoomSetup) {
        fail("hitTest QUICK ROOM SETUP wrong");
    }
    if (overlay::hitTest(256.0F, ph - 248.0F) != overlay::Button::RecordStart) {
        fail("hitTest RECORD wrong");
    }
    if (overlay::hitTest(256.0F, ph - 342.0F) != overlay::Button::RecordStop) {
        fail("hitTest STOP wrong");
    }
    if (overlay::hitTest(5.0F, 5.0F) != overlay::Button::None) {
        fail("hitTest off a button wrong");
    }
    if (overlay::requestFor(overlay::Button::RoomSetup) != "room-setup" ||
        overlay::requestFor(overlay::Button::QuickRoomSetup) != "quick-room-setup" ||
        overlay::requestFor(overlay::Button::RecordStart) != "record-start" ||
        overlay::requestFor(overlay::Button::RecordStop) != "record-stop" ||
        !overlay::requestFor(overlay::Button::None).empty()) {
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
