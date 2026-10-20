// SPDX-License-Identifier: MIT
#include "steamvr_drivers.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>

#include "pibridge/room/files.h"
#include "pibridge/room/json.h"
#include "steamvr.h"

namespace pibridge::app {

namespace room = pibridge::room;

namespace {

QString stateHome() {
    const QString x = qEnvironmentVariable("XDG_STATE_HOME");
    return x.isEmpty() ? QDir::homePath() + QStringLiteral("/.local/state") : x;
}

std::expected<room::Json, QString> readJson(const QString& path, bool* exists) {
    *exists = QFileInfo::exists(path);
    if (!*exists) {
        return room::Json::object();
    }
    auto text = room::readFile(path.toStdString());
    if (!text) {
        return std::unexpected(QStringLiteral("%1 cannot be read: %2").arg(path, QString::fromStdString(text.error())));
    }
    auto j = room::parseJson(*text);
    if (!j || !j->isObject()) {
        return std::unexpected(QStringLiteral("%1 is not a JSON object%2").arg(
            path, j ? QString() : QStringLiteral(" (%1)").arg(QString::fromStdString(j.error()))));
    }
    return *j;
}

std::expected<void, QString> writeJson(const QString& path, const room::Json& j) {
    if (auto w = room::writeFileAtomic(path.toStdString(), j.dump() + "\n"); !w) {
        return std::unexpected(QStringLiteral("%1 cannot be written: %2").arg(path, QString::fromStdString(w.error())));
    }
    return {};
}

bool samePath(const QString& a, const QString& b) {
    return QDir::cleanPath(a) == QDir::cleanPath(b);
}

QString section(const QString& driver) {
    return QStringLiteral("driver_") + driver;
}

// One steamvr.vrsettings key PiBridge changes.
struct KeyChange {
    QString section;
    QString key;
    bool hadSection = false;
    bool hadKey = false;
    QString previous; // the old value as JSON text (when hadKey)
    bool value = false;
};

struct Plan {
    room::Json vrpath;
    room::Json vrsettings;
    bool settingsExisted = true;
    bool listExisted = true;
    QString added;                           // the driver dir appended ("" when it was there)
    std::vector<std::pair<int, QString>> removed; // (index, path) of other "monado" driver dirs taken out
    std::vector<KeyChange> keys;
    QStringList words;
};

// The drivers registered with SteamVR by name: external_drivers entries and <runtime>/drivers/*.
QStringList registeredNames(const room::Json& vrpath, const QString& vrpathFile) {
    QStringList names;
    for (const room::Json& e : vrpath.get("external_drivers").items()) {
        const QString n = driverManifestName(QString::fromStdString(e.str()));
        if (!n.isEmpty()) {
            names << n;
        }
    }
    const QString rt = openVrRuntimeDir(vrpathFile);
    if (!rt.isEmpty()) {
        for (const QString& d : QDir(rt + QStringLiteral("/drivers")).entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
            const QString n = driverManifestName(rt + QStringLiteral("/drivers/") + d);
            names << (n.isEmpty() ? d : n);
        }
    }
    return names;
}

std::expected<Plan, QString> makePlan(const DriverSwitchFiles& f, const DriverPlan& p) {
    if (p.driverDir.isEmpty() || driverManifestName(p.driverDir).isEmpty()) {
        return std::unexpected(QStringLiteral("The Monado build has no SteamVR driver (steamvr-monado/ with "
                                              "driver.vrdrivermanifest)."));
    }
    Plan plan;
    bool vrpathExists = false;
    auto vp = readJson(f.vrpath, &vrpathExists);
    if (!vp) {
        return std::unexpected(vp.error());
    }
    if (!vrpathExists) {
        return std::unexpected(QStringLiteral("SteamVR is not set up: %1 does not exist. Start SteamVR once from Steam.")
                                   .arg(f.vrpath));
    }
    plan.vrpath = std::move(*vp);
    auto vs = readJson(f.vrsettings, &plan.settingsExisted);
    if (!vs) {
        return std::unexpected(vs.error());
    }
    plan.vrsettings = std::move(*vs);

    // external_drivers: ours in, another "monado" out
    plan.listExisted = plan.vrpath.has("external_drivers") && plan.vrpath.get("external_drivers").isArray();
    std::vector<room::Json> keep;
    bool ours = false;
    int idx = 0;
    for (const room::Json& e : plan.listExisted ? plan.vrpath.get("external_drivers").items() : std::vector<room::Json>{}) {
        const QString path = QString::fromStdString(e.str());
        if (samePath(path, p.driverDir)) {
            ours = true;
            keep.push_back(e);
        } else if (driverManifestName(path) == p.driverName) {
            plan.removed.emplace_back(idx, path);
            plan.words << QStringLiteral("openvrpaths.vrpath: take out %1 for the session (another driver named "
                                         "\"%2\")").arg(path, p.driverName);
        } else {
            keep.push_back(e);
        }
        idx++;
    }
    if (!ours) {
        keep.push_back(room::Json::string(p.driverDir.toStdString()));
        plan.added = p.driverDir;
        plan.words << QStringLiteral("openvrpaths.vrpath: add %1 to external_drivers").arg(p.driverDir);
    }
    if (!ours || !plan.removed.empty()) {
        plan.vrpath["external_drivers"] = room::Json::array(std::move(keep));
    }

    // the per-driver switches
    const QStringList registered = registeredNames(plan.vrpath, f.vrpath);
    const auto change = [&plan](const QString& driver, bool value) {
        KeyChange k;
        k.section = section(driver);
        k.key = QStringLiteral("enable");
        k.value = value;
        const std::string sec = k.section.toStdString();
        k.hadSection = plan.vrsettings.has(sec) && plan.vrsettings.get(sec).isObject();
        if (k.hadSection && plan.vrsettings.get(sec).has("enable")) {
            k.hadKey = true;
            k.previous = QString::fromStdString(plan.vrsettings.get(sec).get("enable").dump());
        }
        room::Json& s = plan.vrsettings[sec];
        if (!s.isObject()) {
            s = room::Json::object();
        }
        s["enable"] = room::Json::boolean(value);
        plan.words << QStringLiteral("steamvr.vrsettings: %1.enable = %2 (was %3)")
                          .arg(k.section, value ? QStringLiteral("true") : QStringLiteral("false"),
                               k.hadKey ? k.previous : QStringLiteral("not set"));
        plan.keys.push_back(std::move(k));
    };
    for (const QString& d : p.disable) {
        const std::string sec = section(d).toStdString();
        const bool off = plan.vrsettings.get(sec).has("enable") && !plan.vrsettings.get(sec).get("enable").boolean();
        if (registered.contains(d) && !off) {
            change(d, false);
        }
    }
    {
        const std::string sec = section(p.driverName).toStdString();
        if (plan.vrsettings.get(sec).has("enable") && !plan.vrsettings.get(sec).get("enable").boolean()) {
            change(p.driverName, true);
        }
    }
    return plan;
}

bool writeState(const QString& path, const QJsonObject& o) {
    QDir().mkpath(QFileInfo(path).absolutePath());
    QSaveFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
        return false;
    }
    f.write(QJsonDocument(o).toJson(QJsonDocument::Indented));
    return f.commit();
}

std::expected<void, QString> backup(const QString& path, const std::string& stamp) {
    if (!QFileInfo::exists(path)) {
        return {};
    }
    auto b = room::readFile(path.toStdString());
    if (!b) {
        return std::unexpected(QStringLiteral("%1 cannot be read for its backup.").arg(path));
    }
    if (auto w = room::writeFileAtomic(path.toStdString() + ".bak-" + stamp, *b); !w) {
        return std::unexpected(QStringLiteral("The backup of %1 cannot be written: %2").arg(path, QString::fromStdString(w.error())));
    }
    return {};
}

const QString kRunning = QStringLiteral("SteamVR is running: its driver settings can only be changed while it is "
                                        "closed (it writes them back when it closes).");

} // namespace

QString steamVrSettingsFileFor(const QString& vrpathFile, const QString& home) {
    QFile f(vrpathFile);
    if (f.open(QIODevice::ReadOnly)) {
        const QJsonArray cfg = QJsonDocument::fromJson(f.readAll()).object().value(QStringLiteral("config")).toArray();
        if (!cfg.isEmpty() && !cfg.first().toString().isEmpty()) {
            return QDir::cleanPath(cfg.first().toString()) + QStringLiteral("/steamvr.vrsettings");
        }
    }
    return home + QStringLiteral("/.local/share/Steam/config/steamvr.vrsettings");
}

QString steamVrDriverStateFile() {
    return stateHome() + QStringLiteral("/pibridge/steamvr-drivers.json");
}

QString driverManifestName(const QString& driverDir) {
    if (driverDir.isEmpty()) {
        return {};
    }
    QFile f(driverDir + QStringLiteral("/driver.vrdrivermanifest"));
    if (!f.open(QIODevice::ReadOnly)) {
        return {};
    }
    // SteamVR's manifests are JSON, sometimes with a trailing comma: the room parser and Qt are tried
    const QByteArray raw = f.readAll();
    const QJsonDocument d = QJsonDocument::fromJson(raw);
    if (d.isObject()) {
        return d.object().value(QStringLiteral("name")).toString();
    }
    if (auto j = room::parseJson(raw.toStdString()); j && j->isObject()) {
        return QString::fromStdString(j->get("name").str());
    }
    return {};
}

QStringList steamVrOnlyVars() {
    return {QStringLiteral("LD_PRELOAD"),           QStringLiteral("VALVE_SKIP_RUNTIME_SAFETY"),
            QStringLiteral("QT_QPA_PLATFORM"),      QStringLiteral("QT_PLUGIN_PATH"),
            QStringLiteral("LD_LIBRARY_PATH"),      QStringLiteral("PATH"),
            QStringLiteral("SteamAppId"),           QStringLiteral("SteamGameId"),
            QStringLiteral("SteamOverlayGameId"),   QStringLiteral("STEAMVR_VRENV"),
            QStringLiteral("STEAMVR_TOOLSDIR"),     QStringLiteral("VRCOMPOSITOR_LD_LIBRARY_PATH"),
            QStringLiteral("_STEAMVR_LOGGING_READY"), QStringLiteral("SRT_LAUNCHER_SERVICE_ALONGSIDE_STEAM"),
            QStringLiteral("STEAM_RUNTIME"),        QStringLiteral("PIMAX_STEAMVR_ENV_FILE")};
}

QStringList steamVrRestoredVars() {
    return {QStringLiteral("LD_PRELOAD"), QStringLiteral("QT_QPA_PLATFORM"), QStringLiteral("QT_PLUGIN_PATH"),
            QStringLiteral("LD_LIBRARY_PATH"), QStringLiteral("PATH")};
}

QString steamVrShimDir() {
    return stateHome() + QStringLiteral("/pibridge/steamvr-shim");
}

namespace {

QString shQuote(const QString& v) {
    QString q = v;
    q.replace(QLatin1Char('\''), QStringLiteral("'\\''"));
    return QLatin1Char('\'') + q + QLatin1Char('\'');
}

} // namespace

QString steamVrShimText(const QString& realProgram, const QStringList& userEnv) {
    QStringList parts{QStringLiteral("exec env")};
    for (const QString& v : steamVrOnlyVars()) {
        parts << QStringLiteral("-u ") + v;
    }
    const QStringList restored = steamVrRestoredVars();
    for (const QString& kv : userEnv) {
        const qsizetype eq = kv.indexOf(QLatin1Char('='));
        if (eq > 0 && restored.contains(kv.left(eq))) {
            parts << kv.left(eq) + QLatin1Char('=') + shQuote(kv.mid(eq + 1));
        }
    }
    parts << shQuote(realProgram) << QStringLiteral("\"$@\"");
    return QStringLiteral("#!/bin/sh\n# Written by PiBridge for SteamVR's PATH: what SteamVR opens starts without "
                          "SteamVR's variables.\n") +
           parts.join(QLatin1Char(' ')) + QLatin1Char('\n');
}

QString writeSteamVrShims(const QString& dir, const QStringList& userEnv,
                          const std::function<QString(const QString&)>& which) {
    QDir().mkpath(dir);
    bool any = false;
    for (const char* name : {"xdg-open", "steam", "gio", "gtk-launch"}) {
        const QString real = which ? which(QString::fromLatin1(name)) : QString();
        const QString path = dir + QLatin1Char('/') + QString::fromLatin1(name);
        if (real.isEmpty() || QFileInfo(real).absolutePath() == QFileInfo(path).absolutePath()) {
            QFile::remove(path);
            continue;
        }
        QSaveFile f(path);
        if (!f.open(QIODevice::WriteOnly)) {
            continue;
        }
        f.write(steamVrShimText(real, userEnv).toUtf8());
        if (f.commit()) {
            QFile::setPermissions(path, QFileDevice::ReadOwner | QFileDevice::WriteOwner | QFileDevice::ExeOwner);
            any = true;
        }
    }
    if (!any) {
        return {};
    }
    QString userPath;
    for (const QString& kv : userEnv) {
        if (kv.startsWith(QLatin1String("PATH="))) {
            userPath = kv.mid(5);
        }
    }
    return userPath.isEmpty() ? dir : dir + QLatin1Char(':') + userPath;
}

SteamVrDriverSwitch::SteamVrDriverSwitch(DriverSwitchFiles files, std::function<bool()> vrserverRunning,
                                         std::function<std::time_t()> now)
    : f_(std::move(files)), vrserverRunning_(std::move(vrserverRunning)),
      now_(now ? std::move(now) : std::function<std::time_t()>([] { return std::time(nullptr); })) {}

bool SteamVrDriverSwitch::pending() const {
    return QFileInfo::exists(f_.state);
}

std::expected<QStringList, QString> SteamVrDriverSwitch::preview(const DriverPlan& p) const {
    auto plan = makePlan(f_, p);
    if (!plan) {
        return std::unexpected(plan.error());
    }
    return plan->words;
}

std::expected<QStringList, QString> SteamVrDriverSwitch::apply(const DriverPlan& p) {
    if (vrserverRunning_ && vrserverRunning_()) {
        return std::unexpected(kRunning);
    }
    if (pending()) {
        return std::unexpected(QStringLiteral("SteamVR's driver settings of an earlier session were not put back yet "
                                              "(%1).").arg(f_.state));
    }
    auto plan = makePlan(f_, p);
    if (!plan) {
        return std::unexpected(plan.error());
    }
    if (plan->added.isEmpty() && plan->removed.empty() && plan->keys.empty()) {
        return QStringList{}; // already set up this way: nothing to change, nothing to put back
    }
    // the record first: a crash after it still restores (a restore of a change not made yet is harmless)
    QJsonObject st;
    st[QStringLiteral("format")] = QStringLiteral("pibridge-steamvr-drivers 1");
    st[QStringLiteral("applied")] = QDateTime::fromSecsSinceEpoch(now_()).toString(Qt::ISODate);
    st[QStringLiteral("vrpath")] = f_.vrpath;
    st[QStringLiteral("vrsettings")] = f_.vrsettings;
    st[QStringLiteral("vrsettings_created")] = !plan->settingsExisted;
    st[QStringLiteral("external_drivers_created")] = !plan->listExisted;
    st[QStringLiteral("external_added")] = plan->added;
    QJsonArray removed;
    for (const auto& [i, path] : plan->removed) {
        removed.append(QJsonObject{{QStringLiteral("index"), i}, {QStringLiteral("path"), path}});
    }
    st[QStringLiteral("external_removed")] = removed;
    QJsonArray keys;
    for (const KeyChange& k : plan->keys) {
        QJsonObject o{{QStringLiteral("section"), k.section}, {QStringLiteral("key"), k.key},
                      {QStringLiteral("had_section"), k.hadSection}, {QStringLiteral("had_key"), k.hadKey},
                      {QStringLiteral("set"), k.value}};
        if (k.hadKey) {
            o[QStringLiteral("previous")] = k.previous;
        }
        keys.append(o);
    }
    st[QStringLiteral("settings")] = keys;
    if (!writeState(f_.state, st)) {
        return std::unexpected(QStringLiteral("The state file %1 cannot be written; nothing was changed.").arg(f_.state));
    }
    const std::string stamp = room::backupStamp(now_());
    for (const QString& file : {f_.vrsettings, f_.vrpath}) {
        if (auto b = backup(file, stamp); !b) {
            QFile::remove(f_.state);
            return std::unexpected(b.error() + QStringLiteral(" Nothing was changed."));
        }
    }
    if (!plan->keys.empty()) {
        QDir().mkpath(QFileInfo(f_.vrsettings).absolutePath());
        if (auto w = writeJson(f_.vrsettings, plan->vrsettings); !w) {
            (void)restore();
            return std::unexpected(w.error());
        }
    }
    if (!plan->added.isEmpty() || !plan->removed.empty()) {
        if (auto w = writeJson(f_.vrpath, plan->vrpath); !w) {
            (void)restore();
            return std::unexpected(w.error());
        }
    }
    return plan->words;
}

std::expected<bool, QString> SteamVrDriverSwitch::restore() {
    if (!pending()) {
        return false;
    }
    if (vrserverRunning_ && vrserverRunning_()) {
        return std::unexpected(kRunning);
    }
    QFile sf(f_.state);
    if (!sf.open(QIODevice::ReadOnly)) {
        return std::unexpected(QStringLiteral("The state file %1 cannot be read.").arg(f_.state));
    }
    const QJsonObject st = QJsonDocument::fromJson(sf.readAll()).object();
    sf.close();
    if (st.value(QStringLiteral("format")).toString() != QLatin1String("pibridge-steamvr-drivers 1")) {
        return std::unexpected(QStringLiteral("The state file %1 is not PiBridge's; it was left alone.").arg(f_.state));
    }
    // the files the record names (a session applied with these)
    const QString vrpathFile = st.value(QStringLiteral("vrpath")).toString(f_.vrpath);
    const QString settingsFile = st.value(QStringLiteral("vrsettings")).toString(f_.vrsettings);

    // openvrpaths.vrpath
    bool exists = false;
    auto vp = readJson(vrpathFile, &exists);
    if (!vp) {
        return std::unexpected(vp.error());
    }
    if (exists) {
        room::Json& doc = *vp;
        std::vector<room::Json> list = doc.get("external_drivers").isArray() ? doc.get("external_drivers").items()
                                                                              : std::vector<room::Json>{};
        bool changed = false;
        const QString added = st.value(QStringLiteral("external_added")).toString();
        if (!added.isEmpty()) {
            const auto before = list.size();
            std::erase_if(list, [&](const room::Json& e) { return samePath(QString::fromStdString(e.str()), added); });
            changed = changed || list.size() != before;
        }
        for (const QJsonValue& v : st.value(QStringLiteral("external_removed")).toArray()) {
            const QString path = v.toObject().value(QStringLiteral("path")).toString();
            const bool there = std::any_of(list.begin(), list.end(), [&](const room::Json& e) {
                return samePath(QString::fromStdString(e.str()), path);
            });
            if (!path.isEmpty() && !there) {
                const auto at = std::min<size_t>(size_t(std::max(0, v.toObject().value(QStringLiteral("index")).toInt())), list.size());
                list.insert(list.begin() + static_cast<std::ptrdiff_t>(at), room::Json::string(path.toStdString()));
                changed = true;
            }
        }
        if (changed) {
            if (list.empty() && st.value(QStringLiteral("external_drivers_created")).toBool()) {
                doc.erase("external_drivers");
            } else {
                doc["external_drivers"] = room::Json::array(std::move(list));
            }
            if (auto w = writeJson(vrpathFile, doc); !w) {
                return std::unexpected(w.error());
            }
        }
    }

    // steamvr.vrsettings
    auto vs = readJson(settingsFile, &exists);
    if (!vs) {
        return std::unexpected(vs.error());
    }
    if (exists) {
        room::Json& doc = *vs;
        bool changed = false;
        for (const QJsonValue& v : st.value(QStringLiteral("settings")).toArray()) {
            const QJsonObject k = v.toObject();
            const std::string sec = k.value(QStringLiteral("section")).toString().toStdString();
            const std::string key = k.value(QStringLiteral("key")).toString().toStdString();
            if (sec.empty() || key.empty()) {
                continue;
            }
            if (k.value(QStringLiteral("had_key")).toBool()) {
                auto prev = room::parseJson(k.value(QStringLiteral("previous")).toString().toStdString());
                room::Json& s = doc[sec];
                if (!s.isObject()) {
                    s = room::Json::object();
                }
                if (prev && !(s.get(key) == *prev)) {
                    s[key] = *prev;
                    changed = true;
                }
            } else if (doc.get(sec).isObject() && doc.get(sec).has(key)) {
                doc[sec].erase(key);
                changed = true;
            }
            if (!k.value(QStringLiteral("had_section")).toBool() && doc.get(sec).isObject() &&
                doc.get(sec).members().empty()) {
                doc.erase(sec);
                changed = true;
            }
        }
        if (st.value(QStringLiteral("vrsettings_created")).toBool() && doc.members().empty()) {
            QFile::remove(settingsFile); // PiBridge made it for the session and nothing else is in it
        } else if (changed) {
            if (auto w = writeJson(settingsFile, doc); !w) {
                return std::unexpected(w.error());
            }
        }
    }
    if (!QFile::remove(f_.state)) {
        return std::unexpected(QStringLiteral("The state file %1 cannot be removed.").arg(f_.state));
    }
    return true;
}

} // namespace pibridge::app
