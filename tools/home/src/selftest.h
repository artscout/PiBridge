// SPDX-License-Identifier: MIT
// pibridge-home --selftest: the scene (grid, horizon, a synthetic saved room in a temporary folder, the panel, the
// controllers), the session rules (begin once, yield to a game, end and leave) and the status file, optionally a
// picture of the view through the software renderer. The dashboard: its settings against fake receivers (a temporary
// pibridge.conf and runtime folder), its layout and clicks, and with `paint` (the binary built with Qt) its picture.
// No runtime, no GPU, nothing of the user's is read or written.
#pragma once

#include "dash_view.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace pibridge::home {

/// The dashboard's painter (dash_paint.h): a kW x kH BGRA picture of the view.
using DashPaint = std::function<std::vector<std::uint8_t>(const dash::View&)>;

/// 0 when every step passed. `snapshotDir` "" = no picture. Steps are printed to stdout.
int runSelfTest(const std::string& snapshotDir, const std::string& panelPpm = {}, const DashPaint& paint = {});

} // namespace pibridge::home
