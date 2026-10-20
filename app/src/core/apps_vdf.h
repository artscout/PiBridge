// SPDX-License-Identifier: MIT
// A small reader of Valve's KeyValues text format (libraryfolders.vdf, appmanifest_*.acf):
//
//   "AppState"
//   {
//       "appid"     "620980"
//       "name"      "Beat Saber"
//       "InstalledDepots" { "620981" { "manifest" "4963..." } }
//   }
//
// Keys and values are quoted strings (\" \\ \n \t escapes) or bare words; a value is a string or a { block };
// "//" starts a comment to the end of the line; "#include"/"#base" lines and [$CONDITION] tags are skipped.
// The reader never throws and never loops: broken input gives what was read up to the break plus an error text.
// Keys are matched without regard to case (Steam writes "LastPlayed" and older files "lastplayed").
#pragma once

#include <QString>
#include <QStringView>
#include <vector>

namespace pibridge::app {

struct VdfNode {
    QString key;
    QString value;                 // for a leaf
    std::vector<VdfNode> children; // for a block
    bool block = false;

    // The first child with this key (case-insensitive), or nullptr.
    const VdfNode* child(QStringView name) const;
    // The value of the first leaf child with this key, or `def`.
    QString str(QStringView name, const QString& def = {}) const;
    // The same as a number; `def` when missing or not a number.
    qint64 num(QStringView name, qint64 def = 0) const;
};

struct VdfDoc {
    VdfNode root;  // a block whose children are the top-level entries
    QString error; // "" when the whole text was read
    bool ok() const { return error.isEmpty(); }
};

VdfDoc parseVdf(QStringView text);
// Reads the file (UTF-8); a missing file is an error "cannot read <path>".
VdfDoc readVdfFile(const QString& path);

} // namespace pibridge::app
