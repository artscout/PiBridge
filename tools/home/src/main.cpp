// SPDX-License-Identifier: MIT
// pibridge-home: the home scene in the headset for PiBridge's Tracking session in "Monado directly" mode (docs/home.md).
// The app starts it once Monado runs, with XR_RUNTIME_JSON pointing at that Monado, and stops it with SIGTERM.
#include "options.h"
#include "selftest.h"

#ifdef PIBRIDGE_HOME_XR
#include "xrhome.h"
#endif
#ifdef PIBRIDGE_HOME_DASH
#include "dash_paint.h"
#endif

#include "pibridge/room/files.h"

#include <atomic>
#include <csignal>
#include <cstdio>
#include <string>
#include <vector>

namespace {

std::atomic<bool> g_stop{false};

void onHomeSignal(int) { g_stop.store(true); }

} // namespace

int main(int argc, char** argv) {
    using namespace pibridge::home;
    const std::vector<std::string> args(argv + 1, argv + argc);
    auto o = parseArgs(args);
    if (!o) {
        std::fprintf(stderr, "pibridge-home: %s\n\n%s", o.error().c_str(), usage().c_str());
        return 2;
    }
    if (o->help) {
        std::fputs(usage().c_str(), stdout);
        return 0;
    }
    if (o->selfTest) {
        DashPaint paint;
#ifdef PIBRIDGE_HOME_DASH
        // the painter only for the pictures: the checks run without Qt
        if (!o->snapshotDir.empty()) {
            paint = [](const dash::View& v) {
                static dash::Painter painter;
                return painter.picture(v);
            };
        }
#endif
        return runSelfTest(o->snapshotDir, o->panelPpm, paint);
    }
#ifdef PIBRIDGE_HOME_XR
    std::signal(SIGINT, onHomeSignal);
    std::signal(SIGTERM, onHomeSignal);
    RunOpts r;
    r.roomsDir = o->roomsDir.empty() ? pibridge::room::roomsDir(pibridge::room::Env::fromProcess()) : o->roomsDir;
    r.resDiv = o->resDiv;
    r.camera = o->camera;
    r.passthroughOverlay = o->passthroughOverlay;
    r.dashboardOverlay = o->dashboardOverlay;
    if (auto res = runHome(r, g_stop); !res) {
        std::fprintf(stderr, "pibridge-home: %s\n", res.error().c_str());
        return 1;
    }
    return 0;
#else
    (void)onHomeSignal;
    std::fprintf(stderr, "pibridge-home: built without OpenXR / Vulkan: only --selftest works\n");
    return 2;
#endif
}
