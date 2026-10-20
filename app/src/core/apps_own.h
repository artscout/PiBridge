// SPDX-License-Identifier: MIT
// The user's own apps (Apps page, "Add app..."), stored in ~/.config/pibridge/apps.toml, one [[app]] table each:
//
//   [[app]]
//   id = "falcon-bms"                      stable key (from the name; also the log file name)
//   name = "Falcon BMS"
//   exec = "~/tools/bms-log.sh"            the program; a leading ~/ is the home directory
//   args = ["-xr"]
//   workdir = "/games/BMS"                 "" = the program's folder
//   env = ["DXVK_HUD=fps"]                 extra environment, KEY=value
//   icon = "~/Pictures/bms.png"            optional image file
//   quad_views = false                     false: DISABLE_XR_APILAYER_ARTSCOUT_QUAD_VIEWS_VULKAN=1 for this app
//   qv_profile = "Falcon BMS"              the Performance page profile to open ("" = the program's name)
//   last_played = 1792285852               unix seconds, written by the app on every start
//
// A small TOML subset: bare or quoted keys, basic "..." and literal '...' strings, true/false, integers, arrays of
// strings (also across lines), # comments. Everything the reader does not know (other keys, comments, other tables,
// sub-tables of an app, values in other TOML forms) is kept as written and written back unchanged, so the file can
// be edited by hand and by newer versions of the app. Writes are atomic (QSaveFile: temp file + rename).
#pragma once

#include <QDateTime>
#include <QString>
#include <QStringList>
#include <QStringView>
#include <optional>
#include <vector>

namespace pibridge::app {

struct OwnApp {
    QString id;
    QString name;
    QString exec;
    QStringList args;
    QString workdir;
    QStringList env; // "KEY=value"
    QString icon;
    bool quadViews = true;
    QString qvProfile;
    qint64 lastPlayed = 0;
    QStringList extra; // unknown lines of this [[app]] table (and its sub-tables), kept as written
};

// "Falcon BMS" -> "falcon-bms"; "" or only symbols -> "app".
QString appSlug(const QString& name);
// The profile name the Performance page is opened with: qvProfile, else the program's file name without suffix.
QString qvProfileName(const OwnApp& a);
// Plain-words problems that stop saving ("Give the app a name.", "Choose the program to start.",
// "Environment line 2 is not KEY=value."); empty when fine.
QStringList validateOwnApp(const OwnApp& a);

// The arguments as one line for the editor and back, shell-like: an argument with spaces, quotes or backslashes is
// put in double quotes with \" and \\ escaped; '...' groups too; "" is an empty argument.
QString joinArgs(const QStringList& args);
QStringList splitArgs(const QString& line);
// "Played today, 14:02", "Played yesterday", "Played 3 days ago", "Played 12 Oct 2026", "Never played" (English,
// local time of `now`).
QString lastPlayedWords(qint64 secs, const QDateTime& now);

// TOML string forms, exposed for the tests.
QString tomlQuote(const QString& s);
QString tomlStringArray(const QStringList& l);

class AppsDoc {
public:
    // Never fails: lines it cannot read are kept as unknown lines; `warnings` lists them in plain words.
    static AppsDoc parse(QStringView text, QStringList* warnings = nullptr);
    QString text() const;

    std::vector<OwnApp> apps() const;
    std::optional<OwnApp> find(const QString& id) const;
    // Replace the app with the same id, or append it. An empty id gets a fresh one from the name; an id taken by
    // another app gets a suffix ("-2"). Returns the id used.
    QString upsert(OwnApp a);
    bool remove(const QString& id);

private:
    QString uniqueId(const QString& wanted) const;

    struct Block {
        bool isApp = false;
        OwnApp app;      // isApp
        QStringList raw; // !isApp: a table of another kind, header included, as written
    };
    QStringList preamble_; // lines before the first table
    std::vector<Block> blocks_;
};

// $XDG_CONFIG_HOME/pibridge/apps.toml
QString appsConfigPath();
// $XDG_STATE_HOME/pibridge/apps (~/.local/state/pibridge/apps): the apps' log files
QString appsLogDir();

class AppsStore {
public:
    explicit AppsStore(QString path);
    const QString& path() const { return path_; }
    // Re-read (a missing file = no apps). Returns the reader's warnings.
    QStringList load();
    // Write atomically (creates the folder); false with `error` set when it could not.
    bool save(QString* error = nullptr);
    AppsDoc& doc() { return doc_; }
    const AppsDoc& doc() const { return doc_; }

private:
    QString path_;
    AppsDoc doc_;
};

// The fake mode's two own apps (no real paths).
std::vector<OwnApp> fakeOwnApps(qint64 nowSecs);

} // namespace pibridge::app
