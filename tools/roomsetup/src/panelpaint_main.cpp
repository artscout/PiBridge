// SPDX-License-Identifier: MIT
// pibridge-panelpaint: dump sample room-setup panels to PNG (no headset) so the look can be tuned.
//   pibridge-panelpaint [OUT_DIR] [PVRGUARDIAN_ASSET_DIR]
#include "panelpaint.h"

#include <cstdio>
#include <string>

using namespace pibridge::roomsetup::panel;

int main(int argc, char** argv) {
    const std::string dir = argc > 1 ? argv[1] : "/tmp";
    const std::string assets = argc > 2 ? argv[2] : "";
    Painter pt;
    if (!assets.empty()) {
        pt.setAssetDir(assets);
    }

    const int W = 900, H = 560;
    auto chip = [](const char* l, float x, bool hover, bool primary) {
        return Button{l, x, 0.785f, 0.19f, 0.12f, hover, primary};
    };

    Content floor;
    floor.title = "Floor";
    floor.lines = {"The grid shows the floor below your feet.", "Nudge it up / down with the thumbstick,", "then press NEXT."};
    floor.buttons = {chip("REPEAT", 0.29f, false, false), chip("NEXT", 0.52f, true, true)};
    bool ok = pt.savePng(floor, W, H, dir + "/panel-floor.png");

    Content scan;
    scan.title = "Room setup";
    scan.lines = {"Stand in the middle of your space.", "Look around slowly: left, right,", "behind you, and at the floor."};
    scan.scanning = true;
    scan.buttons = {chip("CANCEL", 0.40f, false, false)};
    ok = pt.savePng(scan, W, H, dir + "/panel-scan.png") && ok;

    Content mode;
    mode.title = "How do you set up?";
    mode.lines = {"QUICK: a circle around you.", "CUSTOM: draw your own edge."};
    mode.buttons = {chip("QUICK", 0.29f, false, true), chip("CUSTOM", 0.52f, true, false)};
    ok = pt.savePng(mode, W, H, dir + "/panel-mode.png") && ok;

    std::printf("%s panels to %s (panel-floor/scan/mode.png)\n", ok ? "wrote" : "FAILED writing", dir.c_str());
    return ok ? 0 : 1;
}
