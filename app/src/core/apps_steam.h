// SPDX-License-Identifier: MIT
// The installed Steam games, found by reading Steam's own files (read only, no network, Steam need not run):
//   <root>/steamapps/libraryfolders.vdf        the library folders ("path" of each entry; the root is always one)
//   <lib>/steamapps/appmanifest_<id>.acf       one per installed app: appid, name, installdir, StateFlags, LastPlayed
//   <root>/config/steamapps.vrmanifest         SteamVR's list of the Steam VR apps ("steam.app.<id>", JSON), plus
//                                              the other manifests named in <root>/config/appconfig.json
//   <root>/appcache/librarycache/...           the cover art Steam downloaded for its library view
// The root is $XDG_DATA_HOME/Steam (~/.local/share/Steam) or ~/.steam/steam / ~/.steam/root (symlinks to it);
// duplicates are dropped by their canonical path. Tools (SteamVR, Proton, the Steam Linux Runtime, the
// redistributables) are never listed.
#pragma once

#include <QHash>
#include <QSet>
#include <QString>
#include <QStringList>
#include <vector>

namespace pibridge::app {

struct SteamGame {
    QString appId;      // "620980"
    QString name;       // "Beat Saber"
    QString installDir; // full path of steamapps/common/<installdir>; may not exist
    qint64 lastPlayed = 0; // unix seconds, 0 = never (the later of the manifest and SteamVR's list)
    bool vr = false;       // SteamVR lists it as a VR app (or a VR overlay)
    QString cover;         // a cover image file, "" when Steam has none cached
    bool coverPortrait = false; // the cover is a 2:3 capsule (else a wide header)
};

struct SteamLibrary {
    bool found = false;   // a Steam root with steamapps/ exists
    bool vrKnown = false; // SteamVR's app list was read, so `vr` means something
    QString root;         // the Steam root used
    QStringList libraries; // the library folders read
    std::vector<SteamGame> games; // sorted: last played first, then by name
    QStringList errors;   // files that could not be read (plain words)
};

// The Steam roots to look at, in order, existing ones only, without duplicates (canonical paths).
// `home` and `dataHome` are for tests (defaults: $HOME and $XDG_DATA_HOME or ~/.local/share).
QStringList steamRoots(const QString& home = {}, const QString& dataHome = {});

// True for the appids and names that are tools, never games (SteamVR 250820, Proton, Steam Linux Runtime,
// Steamworks Common Redistributables, ...).
bool isSteamTool(const QString& appId, const QString& name);

// The app keys SteamVR lists (steamapps.vrmanifest JSON "applications"[]."app_key" and "last_played_time"),
// steam ones only: appid -> last played (0 when absent). `ok` = the file was read.
struct VrManifest {
    bool ok = false;
    QHash<QString, qint64> apps;
};
VrManifest readVrManifest(const QString& path);

// The best cover image for an app in <root>/appcache/librarycache: the portrait capsule first
// (<id>/library_600x900.jpg, <id>/<hash>/library_capsule.jpg, <id>_library_600x900.jpg), then the wide header
// (<id>/header.jpg, <id>/<hash>/library_header.jpg, <id>_header.jpg). "" when none is cached.
QString steamCover(const QString& root, const QString& appId, bool* portrait = nullptr);

// Read everything from one root (tests pass a temp dir).
SteamLibrary readSteamLibrary(const QString& root);
// Read from the first root of steamRoots() that has steamapps/; found = false without one.
SteamLibrary readSteamLibrary();

// The fake mode's library: three games with generated covers (cover = ""), no real paths.
SteamLibrary fakeSteamLibrary(qint64 nowSecs);

// "steam" "steam://rungameid/<id>" (started detached through the Runner).
QStringList steamRunArgs(const QString& appId);

// The appids of the Steam games running now: Steam starts each through its "reaper" helper with
// "SteamLaunch AppId=<id>" on the command line (/proc/<pid>/cmdline, read only). `procRoot` is for tests.
QSet<QString> runningSteamApps(const QString& procRoot = QStringLiteral("/proc"));

} // namespace pibridge::app
