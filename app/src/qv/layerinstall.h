// SPDX-License-Identifier: MIT
// Copied from artscout-qv-vulkan-linux gui/src/layerinstall.h (MIT).
// Where the layer is installed and whether it is on, and the layer's status file of the last game.
#pragma once

#include <QDateTime>
#include <QMap>
#include <QString>

namespace qvgui {

struct Install {
    QString userManifest;   // ~/.local/share/openxr/1/api_layers/implicit.d/<name>.json (on) or .json.disabled (off)
    bool userInstalled = false;
    bool userEnabled = false;
    QString systemManifest; // a manifest outside the home directory (the app cannot switch it)
};

Install findInstall();
// Rename the user manifest to switch the layer on or off; false with `error` set when it failed.
bool setEnabled(const Install& i, bool on, QString& error);

// status.txt of the layer: key -> value, plus the time it was written.
struct Status {
    QMap<QString, QString> values;
    QDateTime written;
    bool valid() const { return !values.isEmpty(); }
};
Status readStatus();
QString logPath();

} // namespace qvgui
