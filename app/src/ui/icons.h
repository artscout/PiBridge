// SPDX-License-Identifier: MIT
// Our own simple line icons, drawn with QPainterPath (no icon fonts, no artwork from elsewhere).
#pragma once

#include <QColor>
#include <QIcon>

namespace pibridge::ui {

enum class Icon { Dashboard, Performance, Logs, About, Play, Restart, Power, Eye, Help, Sound, Tracking, Controller, Room };

// The icon in `normal` colour, and in `checked` colour for the On state (checked nav buttons).
QIcon lineIcon(Icon kind, const QColor& normal, const QColor& checked);
QIcon lineIcon(Icon kind, const QColor& c);

// The app icon: a pibridgeed gem outline in the accent colour on a dark rounded square, drawn at the usual sizes
// (the same drawing as app/data/pibridge.svg, which the desktop entry uses).
QIcon appIcon();
// The logo at `px` logical pixels (sharp on HiDPI).
QPixmap logoPixmap(int px);
// The gem outline alone, `px` square, for the sidebar.
QPixmap gemPixmap(int px, const QColor& c);

} // namespace pibridge::ui
