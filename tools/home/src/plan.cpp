// SPDX-License-Identifier: MIT
#include "plan.h"

namespace pibridge::home {

Plan plan(const bg::HomeSettings& s, const Have& h) {
    Plan p;
    switch (s.background.kind) {
    case bg::Kind::None: p.back = Back::Grid; break;
    case bg::Kind::Dusk: p.back = Back::Dusk; break;
    case bg::Kind::Picture:
        if (!h.pictureFile) {
            p.back = Back::Dusk;
            p.note = "The picture is not downloaded yet: download it on the Home page. Showing the dusk sky.";
        } else if (h.pictureFailed) {
            p.back = Back::Dusk;
            p.note = "The picture could not be read: download it again on the Home page. Showing the dusk sky.";
        } else {
            p.loadPicture = true;
            // the dusk sky while it loads
            p.back = h.pictureLoaded ? Back::Picture : Back::Dusk;
        }
        break;
    case bg::Kind::Passthrough:
        if (!h.cameras) {
            p.back = Back::Dusk;
            p.note = "No camera picture: the passthrough could not start. Showing the dusk sky.";
        } else {
            p.runFeed = true;
            p.back = h.cameraFrames ? Back::Passthrough : Back::Dusk;
            if (!h.cameraFrames) {
                p.note = "Waiting for the cameras ...";
            }
        }
        break;
    }
    p.grid = p.back == Back::Grid;
    const bool window = s.panel && h.appLinked;
    if (h.hidden) {
        p.window = p.dashboard = p.textPanel = false;
    } else if (h.dashboard) {
        // the dashboard unless the window was picked there and can be shown
        p.window = h.wantWindow && window;
        p.dashboard = !p.window;
        p.textPanel = false;
    } else {
        p.window = window;
        p.textPanel = !p.window;
    }
    return p;
}

const char* backName(Back b) {
    switch (b) {
    case Back::Passthrough: return "passthrough";
    case Back::Dusk: return "dusk";
    case Back::Picture: return "picture";
    case Back::Grid: break;
    }
    return "grid";
}

} // namespace pibridge::home
