// SPDX-License-Identifier: MIT
// Where the app finds the daemons' files (docs/CONVENTIONS.md). Every path honours the XDG variables first, so tests
// and the screenshot mode point the whole app at a temporary directory by setting them in the process.
#pragma once

#include <QString>
#include <QStringList>

namespace pibridge::app {

// The install prefix PiBridge was installed under, derived from the running binary (<prefix>/bin/pibridge -> <prefix>),
// so the layout is relocatable. $PIBRIDGE_PREFIX overrides it (tests, a non-standard install); the fallback is ~/.local.
QString installPrefix();
// <prefix>/share/pibridge: the installed read-only data (lens/, calib/, slam/, home/, models/, devices/, vrmanifest).
QString dataDir();
// <prefix>/lib/pibridge: the installed plugins and engine (monado/, steamvr-monado/, libbasalt.so, the record layer,
// winbridge/).
QString libDir();
// <prefix>/etc/pibridge: the shipped default settings (a read-only seed; the live settings stay in configDir()).
QString defaultsDir();

// $XDG_CONFIG_HOME/pibridge (or ~/.config/pibridge). Not created here.
QString configDir();
// configDir()/pibridge.conf: the shared settings.
QString confPath();
// The settings file of PiBridge's earlier name ($XDG_CONFIG_HOME/pimax-linux/pimax-linux.conf): read while confPath()
// does not exist yet; never written.
QString legacyConfPath();
// defaultsDir()/pibridge.conf: the shipped default settings seed, read (never written) when neither confPath() nor
// legacyConfPath() exists yet.
QString defaultsConfPath();
// The folders the eye start list and profile are looked for in, first match wins: configDir(), the folder of
// PiBridge's earlier name (pimax-linux), then omarchyvr's.
QStringList eyeFileDirs();
// $XDG_RUNTIME_DIR/pibridge (or /tmp/pibridge-<uid>): the status files. Read only; never created by the app.
QString statusDir();
// Where "Import calibration from Windows..." writes the eye profile.
QString eyeProfileOut();

} // namespace pibridge::app
