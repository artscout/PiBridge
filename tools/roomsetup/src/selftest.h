// SPDX-License-Identifier: MIT
// pibridge-roomsetup --selftest: the whole editor on made-up files in a temporary folder, driven by scripted controller
// frames (move a corner, open the menu, unlock and fit the play area, trace a room by walking, save, save as a named
// room, reload). Optional snapshots of the views through the software renderer. No runtime, no GPU, nothing of the
// user's is read or written.
#pragma once

#include <string>

namespace pibridge::roomsetup {

/// 0 when every step passed. `snapshotDir` "" = no pictures. Steps are printed to stdout.
int runSelfTest(const std::string& snapshotDir);

/// The synthetic chaperone file the self test edits (a 4 x 3 m room, universe 1111111111, plus universe 2).
std::string selfTestVrchap();

} // namespace pibridge::roomsetup
