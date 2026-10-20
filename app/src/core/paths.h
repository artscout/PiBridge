// SPDX-License-Identifier: MIT
// Where the app finds the daemons' files (docs/CONVENTIONS.md). Every path honours the XDG variables first, so tests
// and the screenshot mode point the whole app at a temporary directory by setting them in the process.
#pragma once

#include <QString>
#include <QStringList>

namespace pibridge::app {

// $XDG_CONFIG_HOME/pibridge (or ~/.config/pibridge). Not created here.
QString configDir();
// configDir()/pibridge.conf: the shared settings.
QString confPath();
// The settings file of PiBridge's earlier name ($XDG_CONFIG_HOME/pimax-linux/pimax-linux.conf): read while confPath()
// does not exist yet; never written.
QString legacyConfPath();
// The folders the eye start list and profile are looked for in, first match wins: configDir(), the folder of
// PiBridge's earlier name (pimax-linux), then omarchyvr's.
QStringList eyeFileDirs();
// $XDG_RUNTIME_DIR/pibridge (or /tmp/pibridge-<uid>): the status files. Read only; never created by the app.
QString statusDir();
// Where "Import calibration from Windows..." writes the eye profile.
QString eyeProfileOut();

} // namespace pibridge::app
