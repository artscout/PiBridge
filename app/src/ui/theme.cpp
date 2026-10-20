// SPDX-License-Identifier: MIT
#include "theme.h"

#include <QApplication>
#include <QPalette>
#include <QStyleFactory>

namespace pibridge::ui {

namespace {
const QStringList kSans{QStringLiteral("IBM Plex Sans"), QStringLiteral("Noto Sans"), QStringLiteral("DejaVu Sans")};
const QStringList kCondensed{QStringLiteral("IBM Plex Sans Condensed"), QStringLiteral("Roboto Condensed"),
                             QStringLiteral("Noto Sans"), QStringLiteral("DejaVu Sans Condensed")};
} // namespace

QFont bodyFont(int px) {
    QFont f;
    f.setFamilies(kSans);
    f.setPixelSize(px);
    return f;
}

QFont labelFont(int px, QFont::Weight w) {
    QFont f = bodyFont(px);
    f.setWeight(w);
    f.setLetterSpacing(QFont::AbsoluteSpacing, 1.3);
    f.setCapitalization(QFont::AllUppercase);
    return f;
}

QFont titleFont(int px) {
    QFont f;
    f.setFamilies(kCondensed);
    f.setPixelSize(px);
    f.setWeight(QFont::Light);
    f.setCapitalization(QFont::AllUppercase);
    return f;
}

QFont monoFont(int px) {
    QFont f;
    f.setFamilies({QStringLiteral("IBM Plex Mono"), QStringLiteral("DejaVu Sans Mono"), QStringLiteral("monospace")});
    f.setPixelSize(px);
    return f;
}

QString styleSheet() {
    return QStringLiteral(R"(
QWidget { color: #e6e6ec; }
QMainWindow, #root, #page { background: #0c0c0e; }
#nav { background: #161619; border-right: 1px solid #222228; }
#navSep { background: #2a2a31; max-height: 1px; min-height: 1px; }
QPushButton#nav { text-align: left; padding: 9px 12px; border: 1px solid transparent; border-radius: 4px;
                  color: #c8c8d0; background: transparent; }
QPushButton#nav:hover { background: #1e1e23; }
QPushButton#nav:checked { background: #1c2255; border: 1px solid #4a5ae8; color: #ffffff; }
QFrame#card { background: #141417; border: 1px solid #2a2a31; border-radius: 3px; }
QFrame#banner { background: #141417; border: 1px solid #2a2a31; border-radius: 3px; }
QFrame#rule { background: #26262c; max-height: 1px; min-height: 1px; border: none; }
QFrame#statusbar { background: #111113; border-top: 1px solid #222228; }
QLabel { background: transparent; }
QLabel#dim { color: #9a9aa5; }
QLabel#error { color: #ef5a55; }
QLabel#accentText { color: #7a88ff; }
QPushButton#tool { background: transparent; border: 1px solid #3a3a43; border-radius: 2px; padding: 5px 10px;
                   color: #e8e8ee; }
QPushButton#tool:hover { border-color: #5b5b68; background: #18181c; }
QPushButton#tool:pressed { background: #202026; }
QPushButton#tool:disabled { color: #5c5c66; border-color: #26262c; }
QPushButton#accent { background: #26307a; border: 1px solid #4a5ae8; border-radius: 2px; padding: 5px 10px;
                     color: #e2e6ff; }
QPushButton#accent:hover { background: #2f3a92; }
QPushButton#accent:disabled { background: #1a1d33; border-color: #2c3260; color: #6c7090; }
QPushButton#link { background: transparent; border: none; color: #7a88ff; padding: 2px 4px; }
QPushButton#link:hover { color: #a3adff; }
QPushButton#link:disabled { color: #4d5170; }
QPushButton#seg { background: #111114; border: 1px solid #34343c; border-left: none; padding: 6px 14px;
                  color: #b8b8c2; border-radius: 0; }
QPushButton#seg[first="true"] { border-left: 1px solid #34343c; border-top-left-radius: 2px;
                                border-bottom-left-radius: 2px; }
QPushButton#seg:hover { color: #ffffff; }
QPushButton#seg:checked { background: #3a47c6; border-color: #4a5ae8; color: #ffffff; }
QPushButton#step { background: transparent; border: none; color: #6d7cff; padding: 0; min-width: 16px; }
QPushButton#step:hover { color: #a3adff; }
QSpinBox#valuebox { background: #0f0f12; border: 1px solid #2e2e36; border-radius: 2px; padding: 2px 4px;
                    color: #e6e6ec; min-width: 38px; }
QComboBox { background: #0f0f12; border: 1px solid #34343c; border-radius: 2px; padding: 4px 8px; color: #e6e6ec; }
QComboBox::drop-down { border: none; width: 18px; }
QComboBox QAbstractItemView { background: #18181c; border: 1px solid #34343c; selection-background-color: #3a47c6; }
QScrollArea { background: transparent; border: none; }
QScrollArea > QWidget > QWidget { background: transparent; }
QScrollBar:vertical { background: transparent; width: 8px; margin: 0; }
QScrollBar::handle:vertical { background: #2e2e36; border-radius: 4px; min-height: 30px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar:horizontal { background: transparent; height: 8px; }
QScrollBar::handle:horizontal { background: #2e2e36; border-radius: 4px; min-width: 30px; }
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }
QPlainTextEdit { background: #0e0e11; border: 1px solid #26262c; color: #c9c9d2; selection-background-color: #3a47c6; }
QToolTip { background: #202026; color: #e6e6ec; border: 1px solid #3a3a43; padding: 4px; }
QMessageBox, QInputDialog, QFileDialog { background: #141417; }
QLineEdit { background: #0f0f12; border: 1px solid #34343c; padding: 4px; color: #e6e6ec; }
)");
}

void applyTheme(QApplication& app) {
    app.setStyle(QStyleFactory::create(QStringLiteral("Fusion")));
    QPalette p;
    p.setColor(QPalette::Window, color::bg);
    p.setColor(QPalette::WindowText, color::text);
    p.setColor(QPalette::Base, QColor(0x0f, 0x0f, 0x12));
    p.setColor(QPalette::AlternateBase, color::panel);
    p.setColor(QPalette::Text, color::text);
    p.setColor(QPalette::Button, color::panel);
    p.setColor(QPalette::ButtonText, color::text);
    p.setColor(QPalette::Highlight, color::accent);
    p.setColor(QPalette::HighlightedText, Qt::white);
    p.setColor(QPalette::ToolTipBase, QColor(0x20, 0x20, 0x26));
    p.setColor(QPalette::ToolTipText, color::text);
    p.setColor(QPalette::PlaceholderText, color::faint);
    p.setColor(QPalette::Disabled, QPalette::Text, color::faint);
    p.setColor(QPalette::Disabled, QPalette::WindowText, color::faint);
    p.setColor(QPalette::Disabled, QPalette::ButtonText, color::faint);
    app.setPalette(p);
    app.setFont(bodyFont(13));
    app.setStyleSheet(styleSheet());
}

} // namespace pibridge::ui
