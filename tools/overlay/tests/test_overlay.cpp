// SPDX-License-Identifier: MIT
// pibridge-overlay without a runtime: the command line, the three dashboard-bar buttons' pages and icons (size, hover
// highlight, recording / room-setup state), the laser hit test (GL bottom-left origin), the request lines (none while a
// room setup runs), the state parsing (capture control file, /proc cmdline), the inline-tabs setting and the manifest.
#include "options.h"
#include "overlay_core.h"

#include "pibridge_test.h"

#include <cstdio>
#include <cstdlib>
#include <string>

using namespace pibridge;
using namespace pibridge::overlay;

namespace {

TabState recording() {
    TabState s;
    s.recording = true;
    return s;
}
TabState roomBusy() {
    TabState s;
    s.roomSetupRunning = true;
    return s;
}

// PIBRIDGE_OVERLAY_DUMP=<dir>: write every page / icon state as a PPM (pages) and a PAM (icons, with alpha) to look at.
void dump(const std::string& name, const std::vector<std::uint8_t>& rgba, std::uint32_t w, std::uint32_t h) {
    const char* dir = std::getenv("PIBRIDGE_OVERLAY_DUMP");
    if (dir == nullptr || dir[0] == '\0') {
        return;
    }
    const bool alpha = w == kIconSize;
    const std::string path = std::string(dir) + "/" + name + (alpha ? ".pam" : ".ppm");
    FILE* f = std::fopen(path.c_str(), "wb");
    if (f == nullptr) {
        return;
    }
    if (alpha) {
        std::fprintf(f, "P7\nWIDTH %u\nHEIGHT %u\nDEPTH 4\nMAXVAL 255\nTUPLTYPE RGB_ALPHA\nENDHDR\n", w, h);
        std::fwrite(rgba.data(), 1, rgba.size(), f);
    } else {
        std::fprintf(f, "P6\n%u %u\n255\n", w, h);
        for (std::size_t i = 0; i < rgba.size(); i += 4) {
            std::fwrite(&rgba[i], 1, 3, f);
        }
    }
    std::fclose(f);
}

} // namespace

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

TEST(three_tabs) {
    // three dashboard overlays, three distinct keys; Room Setup keeps the old key SteamVR may remember
    CHECK(kTabs.size() == 3U);
    CHECK(std::string(tabKey(Tab::Room)) == "pibridge.dashboard");
    CHECK(std::string(tabKey(Tab::QuickRoom)) != tabKey(Tab::Room));
    CHECK(std::string(tabKey(Tab::Record)) != tabKey(Tab::QuickRoom));
    CHECK(tabTitle(Tab::Room, {}) == "Room Setup");
    CHECK(tabTitle(Tab::QuickRoom, {}) == "Quick Room Setup");
    CHECK(tabTitle(Tab::Record, {}) == "Record Video");
    CHECK(tabTitle(Tab::Record, recording()) == "Recording");
}

TEST(page_is_small) {
    // a few hundred pixels, never a 2048-square app window
    CHECK(kPanelW <= 1024 && kPanelH <= 1024);
    for (const Tab t : kTabs) {
        const auto p = drawPage(t, {}, false);
        CHECK(p.size() == static_cast<std::size_t>(kPanelW) * kPanelH * 4U);
        bool opaque = true; // every pixel opaque
        for (std::size_t i = 3; i < p.size(); i += 4) {
            opaque = opaque && p[i] == 255;
        }
        CHECK(opaque);
        CHECK(drawIcon(t, {}).size() == static_cast<std::size_t>(kIconSize) * kIconSize * 4U);
    }
}

TEST(page_reflects_state) {
    for (const Tab t : kTabs) {
        const auto rest = drawPage(t, {}, false);
        CHECK(rest != drawPage(t, {}, true)); // the hovered button
        CHECK(drawPage(t, {}, false) == rest); // deterministic
    }
    CHECK(drawPage(Tab::Record, {}, false) != drawPage(Tab::Record, recording(), false)); // START vs STOP RECORDING
    CHECK(drawPage(Tab::Room, {}, false) != drawPage(Tab::Room, roomBusy(), false));      // ROOM SETUP RUNNING
    // a disabled button does not light up under the laser
    CHECK(drawPage(Tab::Room, roomBusy(), true) == drawPage(Tab::Room, roomBusy(), false));
    // the pages differ from each other
    CHECK(drawPage(Tab::Room, {}, false) != drawPage(Tab::QuickRoom, {}, false));
}

TEST(icon_reflects_state) {
    // the bar icon: the record one shows the recording state, the room ones a running setup; transparent corners
    CHECK(drawIcon(Tab::Record, {}) != drawIcon(Tab::Record, recording()));
    CHECK(drawIcon(Tab::Room, {}) != drawIcon(Tab::Room, roomBusy()));
    CHECK(drawIcon(Tab::QuickRoom, {}) != drawIcon(Tab::QuickRoom, roomBusy()));
    CHECK(drawIcon(Tab::Room, recording()) == drawIcon(Tab::Room, {})); // recording does not touch the room icon
    for (const Tab t : kTabs) {
        CHECK(drawIcon(t, {})[3] == 0);
    }
    // the centre of the recording icon is red
    const auto r = drawIcon(Tab::Record, recording());
    const std::size_t c = (static_cast<std::size_t>(kIconSize / 2) * kIconSize + kIconSize / 2) * 4;
    CHECK(r[c] > 200 && r[c + 1] < 80 && r[c + 3] == 255);
}

TEST(labels) {
    CHECK(buttonLabel(Tab::Room, {}) == "START ROOM SETUP");
    CHECK(buttonLabel(Tab::QuickRoom, {}) == "START QUICK SETUP");
    CHECK(buttonLabel(Tab::Record, {}) == "START RECORDING");
    CHECK(buttonLabel(Tab::Record, recording()) == "STOP RECORDING");
    CHECK(buttonLabel(Tab::Room, roomBusy()) == "ROOM SETUP RUNNING");
    CHECK(buttonLabel(Tab::QuickRoom, roomBusy()) == "ROOM SETUP RUNNING");
    CHECK(statusLine(Tab::Record, recording()) == "RECORDING NOW");
    CHECK(statusLine(Tab::Record, {}) == "NOT RECORDING");
}

TEST(hit_test) {
    // GL space: y grows up from the bottom, so a large y is near the top row. The big button spans rows 54..150.
    const float h = static_cast<float>(kPanelH);
    CHECK(hitButton(260.0F, h - 102.0F));  // its centre
    CHECK(hitButton(30.0F, h - 60.0F));    // near its top-left corner
    CHECK(!hitButton(260.0F, h - 30.0F));  // the header above it
    CHECK(!hitButton(260.0F, h - 180.0F)); // the status line under it
    CHECK(!hitButton(5.0F, 5.0F));         // the bottom margin
    CHECK(!hitButton(-1.0F, 100.0F));      // off the page
    CHECK(!hitButton(static_cast<float>(kPanelW) + 5, 100.0F));
}

TEST(request_lines) {
    CHECK(requestFor(Tab::Room, {}) == "room-setup");
    CHECK(requestFor(Tab::QuickRoom, {}) == "quick-room-setup");
    CHECK(requestFor(Tab::Record, {}) == "record-start");          // not recording -> start
    CHECK(requestFor(Tab::Record, recording()) == "record-stop");  // recording -> stop
    // a room setup runs or was just asked for: no second one
    CHECK(requestFor(Tab::Room, roomBusy()).empty());
    CHECK(requestFor(Tab::QuickRoom, roomBusy()).empty());
    TabState pending;
    pending.roomPending = true;
    CHECK(requestFor(Tab::Room, pending).empty() && requestFor(Tab::QuickRoom, pending).empty());
    CHECK(requestFor(Tab::Record, pending) == "record-start"); // the room request does not block recording
    TabState recPending;
    recPending.recordPending = true;
    CHECK(requestFor(Tab::Record, recPending).empty());
    CHECK(requestFor(Tab::Room, recPending) == "room-setup");
    CHECK(requestFor(Tab::Record, roomBusy()) == "record-start"); // recording during a room setup is allowed
}

TEST(capture_control) {
    CHECK(!captureActive(""));
    CHECK(captureActive("active 1\nview both\nfps 60\n"));
    CHECK(!captureActive("active 0\nview both\n"));
    CHECK(!captureActive("view both\nfps 60\n"));
    CHECK(captureActive("view left\nactive 1"));
}

TEST(room_setup_cmdline) {
    using namespace std::string_literals;
    CHECK(isRoomSetupCmdline("/usr/bin/pibridge-roomsetup\0--map\0--room\0default\0--ring\0/r\0"s));
    CHECK(isRoomSetupCmdline("pibridge-roomsetup\0--backend\0steamvr\0--map\0--quick-direct\0"s));
    CHECK(!isRoomSetupCmdline("pibridge-roomsetup\0--vr-standing\0--keep-floor\0"s)); // headtrack's short commit
    CHECK(!isRoomSetupCmdline("pibridge-roomsetup\0--recenter\0--ring\0/r\0"s));
    CHECK(!isRoomSetupCmdline("/usr/bin/pibridge-roommap\0--map\0"s));
    CHECK(!isRoomSetupCmdline(""));
}

TEST(inline_tabs) {
    CHECK(inlineTabsToSet(0) == kInlineTabsWanted); // unset: SteamVR's default 2 folds our three
    CHECK(inlineTabsToSet(2) == kInlineTabsWanted);
    CHECK(inlineTabsToSet(kInlineTabsWanted) == 0); // enough: left alone
    CHECK(inlineTabsToSet(9) == 0);                 // never lowered
    CHECK(kInlineTabsWanted >= static_cast<int>(kTabs.size()));
}

TEST(dump_images) {
    const char* names[] = {"room", "quick", "record"};
    for (std::size_t i = 0; i < kTabs.size(); i++) {
        const Tab t = kTabs[i];
        TabState roomPend;
        roomPend.roomPending = true;
        TabState recPend = recording();
        recPend.recordPending = true;
        dump(std::string(names[i]) + "-idle", drawPage(t, {}, false), kPanelW, kPanelH);
        dump(std::string(names[i]) + "-hover", drawPage(t, {}, true), kPanelW, kPanelH);
        dump(std::string(names[i]) + "-recording", drawPage(t, recording(), false), kPanelW, kPanelH);
        dump(std::string(names[i]) + "-recording-hover", drawPage(t, recording(), true), kPanelW, kPanelH);
        dump(std::string(names[i]) + "-roombusy", drawPage(t, roomBusy(), true), kPanelW, kPanelH);
        dump(std::string(names[i]) + "-roompending", drawPage(t, roomPend, false), kPanelW, kPanelH);
        dump(std::string(names[i]) + "-stopping", drawPage(t, recPend, false), kPanelW, kPanelH);
        dump(std::string("icon-") + names[i] + "-idle", drawIcon(t, {}), kIconSize, kIconSize);
        dump(std::string("icon-") + names[i] + "-recording", drawIcon(t, recording()), kIconSize, kIconSize);
        dump(std::string("icon-") + names[i] + "-roombusy", drawIcon(t, roomBusy()), kIconSize, kIconSize);
    }
    CHECK(true);
}

TEST(manifest) {
    const std::string m = manifestJson("pibridge.dashboard", "/usr/bin/pibridge-overlay", "/usr/share/x.png", "PiBridge");
    CHECK(m.find("\"app_key\": \"pibridge.dashboard\"") != std::string::npos);
    CHECK(m.find("\"is_dashboard_overlay\": true") != std::string::npos);
    CHECK(m.find("\"binary_path_linux\": \"/usr/bin/pibridge-overlay\"") != std::string::npos);
    CHECK(m.find("\"image_path\": \"/usr/share/x.png\"") != std::string::npos);
}

PIBRIDGE_TEST_MAIN();
