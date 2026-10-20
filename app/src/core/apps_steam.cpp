// SPDX-License-Identifier: MIT
#include "apps_steam.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <algorithm>

#include "apps_vdf.h"

namespace pibridge::app {

QStringList steamRoots(const QString& home, const QString& dataHome) {
    const QString h = home.isEmpty() ? QDir::homePath() : home;
    QString data = dataHome;
    if (data.isEmpty()) {
        data = qEnvironmentVariable("XDG_DATA_HOME");
    }
    if (data.isEmpty()) {
        data = h + QStringLiteral("/.local/share");
    }
    const QStringList candidates{data + QStringLiteral("/Steam"), h + QStringLiteral("/.steam/steam"),
                                 h + QStringLiteral("/.steam/root"), h + QStringLiteral("/.local/share/Steam")};
    QStringList out;
    QSet<QString> seen;
    for (const QString& c : candidates) {
        const QFileInfo fi(c);
        if (!fi.isDir()) {
            continue;
        }
        const QString canon = fi.canonicalFilePath();
        if (canon.isEmpty() || seen.contains(canon)) {
            continue;
        }
        seen.insert(canon);
        out << canon;
    }
    return out;
}

bool isSteamTool(const QString& appId, const QString& name) {
    // appids of the tools Steam installs by itself (SteamVR, runtimes, Proton builds, redistributables)
    static const QSet<QString> ids{
        QStringLiteral("250820"),  // SteamVR
        QStringLiteral("228980"),  // Steamworks Common Redistributables
        QStringLiteral("1070560"), // Steam Linux Runtime 1.0 (scout)
        QStringLiteral("1391110"), // Steam Linux Runtime 2.0 (soldier)
        QStringLiteral("1628350"), // Steam Linux Runtime 3.0 (sniper)
        QStringLiteral("4183110"), // Steam Linux Runtime 4.0
        QStringLiteral("1493710"), // Proton Experimental
        QStringLiteral("2180100"), // Proton Hotfix
        QStringLiteral("1826330"), // Proton EasyAntiCheat Runtime
        QStringLiteral("1161040"), // Proton BattlEye Runtime
        QStringLiteral("858280"),  QStringLiteral("930400"),  QStringLiteral("961940"),  QStringLiteral("996510"),
        QStringLiteral("1054830"), QStringLiteral("1113280"), QStringLiteral("1245040"), QStringLiteral("1420170"),
        QStringLiteral("1580130"), QStringLiteral("1887720"), QStringLiteral("2348590"), QStringLiteral("2805730"),
        QStringLiteral("3658110"), // Proton 3.7 .. 10.0
    };
    if (ids.contains(appId)) {
        return true;
    }
    const QString n = name.trimmed();
    return n.startsWith(QLatin1String("Proton"), Qt::CaseInsensitive) ||
           n.startsWith(QLatin1String("Steam Linux Runtime"), Qt::CaseInsensitive) ||
           n.startsWith(QLatin1String("Steamworks Common"), Qt::CaseInsensitive) ||
           n.contains(QLatin1String("Redistributable"), Qt::CaseInsensitive) ||
           n.compare(QLatin1String("SteamVR"), Qt::CaseInsensitive) == 0;
}

VrManifest readVrManifest(const QString& path) {
    VrManifest m;
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        return m;
    }
    const QJsonDocument doc = QJsonDocument::fromJson(f.read(16 * 1024 * 1024));
    if (!doc.isObject()) {
        return m;
    }
    m.ok = true;
    static const QRegularExpression key(QStringLiteral("^steam\\.(?:app|overlay)\\.(\\d+)$"));
    for (const QJsonValue& v : doc.object().value(QStringLiteral("applications")).toArray()) {
        const QJsonObject a = v.toObject();
        const auto match = key.match(a.value(QStringLiteral("app_key")).toString());
        if (!match.hasMatch()) {
            continue;
        }
        // SteamVR writes the time as a string; accept a number too
        const QJsonValue t = a.value(QStringLiteral("last_played_time"));
        const qint64 played = t.isString() ? t.toString().toLongLong() : static_cast<qint64>(t.toDouble());
        qint64& slot = m.apps[match.captured(1)];
        slot = std::max(slot, played);
    }
    return m;
}

QString steamCover(const QString& root, const QString& appId, bool* portrait) {
    const QString cache = root + QStringLiteral("/appcache/librarycache");
    const QString dir = cache + QLatin1Char('/') + appId;
    // newer Steam keeps some images in hashed sub-folders (<id>/<sha1>/library_capsule.jpg)
    const auto inSub = [&dir](const QString& file) -> QString {
        const QDir d(dir);
        for (const QString& sub : d.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name)) {
            const QString p = dir + QLatin1Char('/') + sub + QLatin1Char('/') + file;
            if (QFileInfo(p).isFile()) {
                return p;
            }
        }
        return {};
    };
    const auto file = [](const QString& p) { return QFileInfo(p).isFile() ? p : QString(); };
    QString p;
    if (!(p = file(dir + QStringLiteral("/library_600x900.jpg"))).isEmpty() ||
        !(p = inSub(QStringLiteral("library_capsule.jpg"))).isEmpty() ||
        !(p = inSub(QStringLiteral("library_600x900.jpg"))).isEmpty() ||
        !(p = file(cache + QLatin1Char('/') + appId + QStringLiteral("_library_600x900.jpg"))).isEmpty()) {
        if (portrait) {
            *portrait = true;
        }
        return p;
    }
    if (portrait) {
        *portrait = false;
    }
    if (!(p = file(dir + QStringLiteral("/header.jpg"))).isEmpty() ||
        !(p = inSub(QStringLiteral("library_header.jpg"))).isEmpty() ||
        !(p = inSub(QStringLiteral("header.jpg"))).isEmpty() ||
        !(p = file(cache + QLatin1Char('/') + appId + QStringLiteral("_header.jpg"))).isEmpty()) {
        return p;
    }
    return {};
}

namespace {

// The library folders of a root: the root itself, then every "path" of libraryfolders.vdf (steamapps/ first, the
// older config/ copy as a fallback), existing ones only, without duplicates.
QStringList libraryFolders(const QString& root, QStringList& errors) {
    QStringList libs{root};
    QSet<QString> seen{QFileInfo(root).canonicalFilePath()};
    QString file = root + QStringLiteral("/steamapps/libraryfolders.vdf");
    if (!QFileInfo::exists(file)) {
        file = root + QStringLiteral("/config/libraryfolders.vdf");
    }
    if (!QFileInfo::exists(file)) {
        return libs;
    }
    const VdfDoc d = readVdfFile(file);
    if (!d.ok()) {
        errors << QStringLiteral("%1: %2").arg(QFileInfo(file).fileName(), d.error);
    }
    const VdfNode* top = d.root.child(u"libraryfolders");
    if (!top) {
        return libs;
    }
    for (const VdfNode& e : top->children) {
        // new format: "0" { "path" "..." }, the old one: "1" "/path"
        const QString path = e.block ? e.str(u"path") : e.value;
        if (path.isEmpty() || !QFileInfo(path + QStringLiteral("/steamapps")).isDir()) {
            continue;
        }
        const QString canon = QFileInfo(path).canonicalFilePath();
        if (seen.contains(canon)) {
            continue;
        }
        seen.insert(canon);
        libs << path;
    }
    return libs;
}

} // namespace

SteamLibrary readSteamLibrary(const QString& root) {
    SteamLibrary lib;
    lib.root = root;
    if (!QFileInfo(root + QStringLiteral("/steamapps")).isDir()) {
        return lib;
    }
    lib.found = true;
    lib.libraries = libraryFolders(root, lib.errors);

    // SteamVR's lists: steamapps.vrmanifest, plus every manifest appconfig.json names
    QStringList manifests{root + QStringLiteral("/config/steamapps.vrmanifest")};
    {
        QFile f(root + QStringLiteral("/config/appconfig.json"));
        if (f.open(QIODevice::ReadOnly)) {
            const QJsonDocument doc = QJsonDocument::fromJson(f.read(1024 * 1024));
            for (const QJsonValue& v : doc.object().value(QStringLiteral("manifest_paths")).toArray()) {
                const QString p = QDir::cleanPath(v.toString());
                if (!p.isEmpty() && !manifests.contains(p)) {
                    manifests << p;
                }
            }
        }
    }
    QHash<QString, qint64> vr;
    for (const QString& m : manifests) {
        const VrManifest vm = readVrManifest(m);
        if (!vm.ok) {
            continue;
        }
        lib.vrKnown = true;
        for (auto it = vm.apps.cbegin(); it != vm.apps.cend(); ++it) {
            vr[it.key()] = std::max(vr.value(it.key()), it.value());
        }
    }

    QSet<QString> seen;
    for (const QString& l : lib.libraries) {
        const QDir apps(l + QStringLiteral("/steamapps"));
        for (const QString& f : apps.entryList({QStringLiteral("appmanifest_*.acf")}, QDir::Files, QDir::Name)) {
            const VdfDoc d = readVdfFile(apps.filePath(f));
            const VdfNode* st = d.ok() ? d.root.child(u"AppState") : nullptr;
            if (!st) {
                lib.errors << QStringLiteral("%1: %2").arg(f, d.ok() ? QStringLiteral("no AppState") : d.error);
                continue;
            }
            SteamGame g;
            g.appId = st->str(u"appid");
            g.name = st->str(u"name");
            if (g.appId.isEmpty() || seen.contains(g.appId) || isSteamTool(g.appId, g.name)) {
                continue;
            }
            // StateFlags bit 4 = fully installed; a game being downloaded for the first time cannot start yet
            if ((st->num(u"StateFlags") & 4) == 0) {
                continue;
            }
            if (g.name.isEmpty()) {
                g.name = QStringLiteral("App %1").arg(g.appId);
            }
            seen.insert(g.appId);
            const QString dir = st->str(u"installdir");
            if (!dir.isEmpty()) {
                g.installDir = apps.filePath(QStringLiteral("common/") + dir);
            }
            g.lastPlayed = std::max(st->num(u"LastPlayed"), vr.value(g.appId));
            g.vr = vr.contains(g.appId);
            g.cover = steamCover(root, g.appId, &g.coverPortrait);
            lib.games.push_back(std::move(g));
        }
    }
    std::stable_sort(lib.games.begin(), lib.games.end(), [](const SteamGame& a, const SteamGame& b) {
        if (a.lastPlayed != b.lastPlayed) {
            return a.lastPlayed > b.lastPlayed;
        }
        return a.name.compare(b.name, Qt::CaseInsensitive) < 0;
    });
    return lib;
}

SteamLibrary readSteamLibrary() {
    for (const QString& r : steamRoots()) {
        if (QFileInfo(r + QStringLiteral("/steamapps")).isDir()) {
            return readSteamLibrary(r);
        }
    }
    return {};
}

SteamLibrary fakeSteamLibrary(qint64 now) {
    SteamLibrary lib;
    lib.found = true;
    lib.vrKnown = true;
    lib.root = QStringLiteral("/fake/Steam");
    lib.libraries = {lib.root};
    lib.games = {
        {QStringLiteral("900001"), QStringLiteral("Starlight Cockpit"), {}, now - 2 * 3600, true, {}, false},
        {QStringLiteral("900002"), QStringLiteral("Echo Canyon VR"), {}, now - 3 * 86400, true, {}, false},
        {QStringLiteral("900003"), QStringLiteral("Desk Puzzler"), {}, 0, false, {}, false},
    };
    return lib;
}

QStringList steamRunArgs(const QString& appId) {
    return {QStringLiteral("steam://rungameid/") + appId};
}

QSet<QString> runningSteamApps(const QString& procRoot) {
    QSet<QString> out;
    const QDir proc(procRoot);
    for (const QString& pid : proc.entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
        if (pid.isEmpty() || !pid.front().isDigit()) {
            continue;
        }
        QFile f(procRoot + QLatin1Char('/') + pid + QStringLiteral("/cmdline"));
        if (!f.open(QIODevice::ReadOnly)) {
            continue;
        }
        const QList<QByteArray> args = f.read(64 * 1024).split('\0');
        bool launch = false;
        for (const QByteArray& a : args) {
            if (a == "SteamLaunch") {
                launch = true;
            } else if (a == "--") {
                break; // the game's own arguments start here
            } else if (launch && a.startsWith("AppId=")) {
                const QByteArray id = a.mid(6);
                if (!id.isEmpty() && std::all_of(id.begin(), id.end(), [](char c) { return c >= '0' && c <= '9'; })) {
                    out.insert(QString::fromLatin1(id));
                }
            }
        }
    }
    return out;
}

} // namespace pibridge::app
