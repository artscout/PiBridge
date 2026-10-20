// SPDX-License-Identifier: MIT
// The look: near-black background, slightly lighter panels with a 1 px border, one indigo accent, light condensed
// UPPERCASE titles, letter-spaced UPPERCASE labels. One style sheet plus the fonts and colours the custom widgets use.
#pragma once

#include <QColor>
#include <QFont>
#include <QString>

class QApplication;

namespace pibridge::ui {

namespace color {
inline const QColor bg{0x0c, 0x0c, 0x0e};
inline const QColor nav{0x16, 0x16, 0x19};
inline const QColor panel{0x14, 0x14, 0x17};
inline const QColor border{0x2a, 0x2a, 0x31};
inline const QColor borderStrong{0x3a, 0x3a, 0x43};
inline const QColor text{0xe6, 0xe6, 0xec};
inline const QColor dim{0x9a, 0x9a, 0xa5};
inline const QColor faint{0x5c, 0x5c, 0x66};
inline const QColor accent{0x4a, 0x5a, 0xe8};
inline const QColor accentText{0x7a, 0x88, 0xff};
inline const QColor accentDark{0x1c, 0x22, 0x55};
inline const QColor amber{0xf2, 0xb0, 0x2e};
inline const QColor red{0xef, 0x5a, 0x55};
inline const QColor yellowDot{0xf5, 0xc5, 0x18};
} // namespace color

// Body text.
QFont bodyFont(int px = 13);
// Letter-spaced uppercase labels ("EYE TRACKING").
QFont labelFont(int px = 13, QFont::Weight w = QFont::Medium);
// Light condensed card and page titles ("QUICK SETTINGS").
QFont titleFont(int px = 22);
QFont monoFont(int px = 12);

QString styleSheet();
// Fonts, palette and style sheet for the whole app.
void applyTheme(QApplication& app);

} // namespace pibridge::ui
