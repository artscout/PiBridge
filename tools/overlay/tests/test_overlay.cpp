// SPDX-License-Identifier: MIT
// pibridge-overlay without a runtime: the command line, the compact panel it draws (size, hover highlight, recording
// state), the laser hit test (GL bottom-left origin), the request lines and the SteamVR manifest text.
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
    CHECK(drawPanel(false, Hit{}).size() == static_cast<std::size_t>(kPanelW) * kPanelH * 4U);
    // every pixel opaque
    const auto p = drawPanel(false, Hit{});
    bool opaque = true;
    for (std::size_t i = 3; i < p.size(); i += 4) {
        opaque = opaque && p[i] == 255;
    }
    CHECK(opaque);
}

TEST(panel_reflects_state) {
    const auto rest = drawPanel(false, Hit{});
    CHECK(rest != drawPanel(false, Hit{Hit::Kind::Room}));      // a hovered button
    CHECK(rest != drawPanel(false, Hit{Hit::Kind::QuickRoom})); // another hovered button
    CHECK(rest != drawPanel(true, Hit{}));                      // START vs STOP RECORDING
    CHECK(drawPanel(false, Hit{}) == rest);                     // deterministic
}

TEST(hit_test) {
    // GL space: y grows up from the bottom, so a large y is near the top row. Centres of the three wide buttons.
    const float h = static_cast<float>(kPanelH);
    CHECK(hitTest(260.0F, h - 92.0F) == (Hit{Hit::Kind::Record}));
    CHECK(hitTest(260.0F, h - 226.0F) == (Hit{Hit::Kind::Room}));
    CHECK(hitTest(260.0F, h - 304.0F) == (Hit{Hit::Kind::QuickRoom}));
    CHECK(hitTest(5.0F, 5.0F) == Hit{});                                // the bottom margin
    CHECK(hitTest(-1.0F, 10.0F) == Hit{});                              // off the panel
    CHECK(hitTest(static_cast<float>(kPanelW) + 5, 10.0F) == Hit{});
}

TEST(request_lines) {
    CHECK(requestFor(Hit{Hit::Kind::Room}, false) == "room-setup");
    CHECK(requestFor(Hit{Hit::Kind::QuickRoom}, false) == "quick-room-setup");
    CHECK(requestFor(Hit{Hit::Kind::Record}, false) == "record-start"); // not recording -> start
    CHECK(requestFor(Hit{Hit::Kind::Record}, true) == "record-stop");   // recording -> stop
    CHECK(requestFor(Hit{}, false).empty());
}

TEST(manifest) {
    const std::string m = manifestJson("pibridge.dashboard", "/usr/bin/pibridge-overlay", "/usr/share/x.png", "PiBridge");
    CHECK(m.find("\"app_key\": \"pibridge.dashboard\"") != std::string::npos);
    CHECK(m.find("\"is_dashboard_overlay\": true") != std::string::npos);
    CHECK(m.find("\"binary_path_linux\": \"/usr/bin/pibridge-overlay\"") != std::string::npos);
    CHECK(m.find("\"image_path\": \"/usr/share/x.png\"") != std::string::npos);
}

PIBRIDGE_TEST_MAIN();
