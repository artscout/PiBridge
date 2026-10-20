// SPDX-License-Identifier: MIT
// The product name and tagline, in one place (window title, sidebar, About, the desktop entry says the same).
#pragma once

#include <QString>

namespace pibridge::ui {

inline QString productName() { return QStringLiteral("PiBridge"); }
inline QString tagline() { return QStringLiteral("VR hub for Pimax headsets on Linux. Not affiliated with Pimax."); }

} // namespace pibridge::ui
