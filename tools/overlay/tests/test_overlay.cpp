// SPDX-License-Identifier: MIT
// pibridge-overlay without a runtime: the command line, the small launcher panel it draws (size, hover highlight), the
// laser hit test (GL bottom-left origin), the request lines and the SteamVR manifest text.
#include "options.h"
#include "overlay_core.h"

#include "pibridge_test.h"

using namespace pibridge;
using namespace pibridge::overlay;

TEST(options) {
    CHECK(parseArgs({})->width < 1.0 && !parseArgs({})->selfTest); // a small default panel
    CHECK(parseArgs({"--selftest"})->selfTest);
    CHECK(parseArgs({"--register"})->doRegister);
    CHECK(parseArgs({"--icon", "/a.png"})->icon == "/a.png");
    CHECK(parseArgs({"--width", "0.5"})->width == 0.5);
    CHECK(!parseArgs({"--width", "9"}));         // out of range
    CHECK(!parseArgs({"--icon"}));               // missing value
    CHECK(!parseArgs({"--register", "--unregister"}));
    CHECK(!parseArgs({"--nope"}));
}

TEST(panel_is_small) {
    // a few hundred pixels, never a 2048-square app window
    CHECK(kPanelW <= 1024 && kPanelH <= 1024);
    CHECK(drawPanel(Button::None).size() == static_cast<std::size_t>(kPanelW) * kPanelH * 4U);
    // every pixel opaque
    const auto p = drawPanel(Button::None);
    bool opaque = true;
    for (std::size_t i = 3; i < p.size(); i += 4) {
        opaque = opaque && p[i] == 255;
    }
    CHECK(opaque);
}

TEST(hover_changes_only_on_change) {
    const auto rest = drawPanel(Button::None);
    const auto hot0 = drawPanel(Button::RoomSetup);
    const auto hot1 = drawPanel(Button::QuickRoomSetup);
    CHECK(rest != hot0);
    CHECK(rest != hot1);
    CHECK(hot0 != hot1);
    CHECK(drawPanel(Button::RoomSetup) == hot0); // deterministic
}

TEST(hit_test) {
    // GL space: y grows up from the bottom, so a large y is near the top button. Centres of the four stacked buttons.
    const float w = static_cast<float>(kPanelW) / 2;
    const float h = static_cast<float>(kPanelH);
    CHECK(hitTest(w, h - 60.0F) == Button::RoomSetup);
    CHECK(hitTest(w, h - 154.0F) == Button::QuickRoomSetup);
    CHECK(hitTest(w, h - 248.0F) == Button::RecordStart);
    CHECK(hitTest(w, h - 342.0F) == Button::RecordStop);
    CHECK(hitTest(2.0F, 2.0F) == Button::None);                          // the bottom margin
    CHECK(hitTest(-1.0F, 10.0F) == Button::None);                        // off the panel
    CHECK(hitTest(static_cast<float>(kPanelW) + 5, 10.0F) == Button::None);
}

TEST(request_lines) {
    CHECK(requestFor(Button::RoomSetup) == "room-setup");
    CHECK(requestFor(Button::QuickRoomSetup) == "quick-room-setup");
    CHECK(requestFor(Button::RecordStart) == "record-start");
    CHECK(requestFor(Button::RecordStop) == "record-stop");
    CHECK(requestFor(Button::None).empty());
}

TEST(manifest) {
    const std::string m = manifestJson("pibridge.dashboard", "/usr/bin/pibridge-overlay", "/usr/share/x.png", "PiBridge");
    CHECK(m.find("\"app_key\": \"pibridge.dashboard\"") != std::string::npos);
    CHECK(m.find("\"is_dashboard_overlay\": true") != std::string::npos);
    CHECK(m.find("\"binary_path_linux\": \"/usr/bin/pibridge-overlay\"") != std::string::npos);
    CHECK(m.find("\"image_path\": \"/usr/share/x.png\"") != std::string::npos);
}

PIBRIDGE_TEST_MAIN();
