// SPDX-License-Identifier: MIT
// pibridge-home --selftest: the scene (grid, horizon, a synthetic saved room in a temporary folder, the panel, the
// controllers), the session rules (begin once, yield to a game, end and leave) and the status file, optionally a
// picture of the view through the software renderer. No runtime, no GPU, nothing of the user's is read or written.
#pragma once

#include <string>

namespace pibridge::home {

/// 0 when every step passed. `snapshotDir` "" = no picture. Steps are printed to stdout.
int runSelfTest(const std::string& snapshotDir, const std::string& panelPpm = {});

} // namespace pibridge::home
