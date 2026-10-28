// SPDX-License-Identifier: MIT
// What the VR home shows, from the settings and what is there (pure; xrhome.cpp follows it, the tests check it):
// the background (the room through the cameras, a picture, the built-in dusk, or the dark grid), and on the curved panel
// the dashboard (dash_view.h), the PiBridge window when picked there, or, with neither, the old text panel. The menu
// button hides the panel (Have::hidden) and brings it back. Every missing part falls back to a simpler one and says why in
// plain words (the status file's "background_note"); nothing stops the home.
#pragma once

#include "backgrounds.h"

#include <string>

namespace pibridge::home {

enum class Back { Grid, Passthrough, Dusk, Picture };

struct Have {
    bool pictureFile = false; // the chosen picture is downloaded
    bool pictureLoaded = false;
    bool pictureFailed = false; // it could not be read
    bool cameras = false;     // the passthrough rig and pipeline are there
    bool cameraFrames = false; // frames came in the last 2 s
    bool appLinked = false;   // the app's window picture is fresh
    bool dashboard = false;   // the dashboard can be drawn (the panel pass and the painter are there)
    bool wantWindow = false;  // the PiBridge window was picked on the dashboard
    bool hidden = false;      // the menu button hid the panel
};

struct Plan {
    Back back = Back::Grid;
    bool loadPicture = false; // start (or keep) loading the chosen picture
    bool runFeed = false;     // the camera feed runs
    bool grid = true;
    bool dashboard = false;   // the dashboard on the panel
    bool window = false;      // the PiBridge window on the panel
    bool textPanel = true;    // the fallback text panel
    std::string note;         // plain words when something fell back
};

Plan plan(const bg::HomeSettings& s, const Have& h);
const char* backName(Back b);

} // namespace pibridge::home
