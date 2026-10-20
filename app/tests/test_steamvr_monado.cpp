// SPDX-License-Identifier: MIT
// Tests of START TRACKING with SteamVR + Monado (core/steamvr_drivers.h, core/io_session.h IoVariant::SteamVr) and of
// the Monado build choice (core/monado_build.h): SteamVR's driver switch on made-up openvrpaths.vrpath /
// steamvr.vrsettings files (exact, idempotent restore, refused while vrserver runs, the crash case), the shims, the
// session's start order, its PID-exact stop with fake signals, SteamVR closed by the user, the markers and the
// cap_sys_nice reading, and "Fix permissions" through a FakeRunner. Nothing is ever started or signalled; every file
// is in a temporary dir.
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <QThread>
#include <atomic>
#include <QtEndian>
#include <csignal>
#include <set>

#include "core/anchor_offset.h"
#include "core/conf.h"
#include "core/io_session.h"
#include "core/monado_build.h"
#include "core/rec_session.h"
#include "core/rec_setup.h"
#include "core/steamvr_drivers.h"
#include "pibridge/room/files.h"
#include "pibridge/room/json.h"
#include "pibridge_test.h"

using namespace pibridge::app;

namespace {

void writeAll(const QString& p, const QByteArray& b) {
    QDir().mkpath(QFileInfo(p).absolutePath());
    QFile f(p);
    if (f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        f.write(b);
    }
}

QByteArray readAll(const QString& p) {
    QFile f(p);
    return f.open(QIODevice::ReadOnly) ? f.readAll() : QByteArray();
}

struct Home {
    QTemporaryDir dir;
    Home() {
        for (const char* v : {"XDG_CONFIG_HOME", "XDG_CACHE_HOME", "XDG_STATE_HOME", "XDG_DATA_HOME", "XDG_RUNTIME_DIR"}) {
            const QString d = dir.path() + QLatin1Char('/') + QString::fromLatin1(v).toLower();
            QDir().mkpath(d);
            qputenv(v, QFile::encodeName(d));
        }
    }
    QString path(const QString& rel) const { return dir.path() + QLatin1Char('/') + rel; }
};

// A SteamVR set up as on the Crystal Super machine, with made-up paths: CHOVR registered, a driver_monado dir of
// another build registered too (a second "monado"), the files in SteamVR's own format (3-space indent).
struct SteamFiles {
    Home h;
    QString vrpath, vrsettings, state, ours, other, chovr;
    QByteArray vrpathText, settingsText;
    explicit SteamFiles(bool otherMonado = false, bool settingsFile = true) {
        vrpath = h.path(QStringLiteral("xdg_config_home/openvr/openvrpaths.vrpath"));
        vrsettings = h.path(QStringLiteral("steam/config/steamvr.vrsettings"));
        state = h.path(QStringLiteral("xdg_state_home/pibridge/steamvr-drivers.json"));
        ours = h.path(QStringLiteral("pibridge-monado-build/steamvr-monado"));
        other = h.path(QStringLiteral("omarchyvr/monado/steamvr-monado"));
        chovr = h.path(QStringLiteral("ext/CustomHeadsetOpenVR"));
        writeAll(ours + QStringLiteral("/driver.vrdrivermanifest"), "{\n\t\"alwaysActivate\": false,\n\t\"name\" : \"monado\"\n}\n");
        writeAll(other + QStringLiteral("/driver.vrdrivermanifest"), "{ \"name\" : \"monado\" }\n");
        writeAll(chovr + QStringLiteral("/driver.vrdrivermanifest"), "{ \"name\" : \"CustomHeadsetOpenVR\" }\n");
        QStringList ext{QStringLiteral("\"%1\"").arg(chovr)};
        if (otherMonado) {
            ext << QStringLiteral("\"%1\"").arg(other);
        }
        auto vp = pibridge::room::parseJson(
            QStringLiteral("{\"config\": [\"%1\"], \"external_drivers\": [%2], \"jsonid\": \"vrpathreg\", "
                           "\"log\": [\"/x/logs\"], \"runtime\": [\"%3\"], \"version\": 1}")
                .arg(QFileInfo(vrsettings).absolutePath(), ext.join(QStringLiteral(", ")), h.path(QStringLiteral("SteamVR")))
                .toStdString());
        vrpathText = QByteArray::fromStdString(vp->dump() + "\n");
        writeAll(vrpath, vrpathText);
        auto vs = pibridge::room::parseJson(
            "{\"collisionBounds\": {\"CollisionBoundsFadeDistance\": 0.5}, \"driver_CustomHeadsetOpenVR\": "
            "{\"hasBeenRun\": \"1.3.0\"}, \"steamvr\": {\"showMirrorView\": false, \"installID\": \"123\"}}");
        settingsText = QByteArray::fromStdString(vs->dump() + "\n");
        if (settingsFile) {
            writeAll(vrsettings, settingsText);
        }
    }
    DriverSwitchFiles files() const { return {vrpath, vrsettings, state}; }
    DriverPlan plan() const {
        DriverPlan p;
        p.driverDir = ours;
        return p;
    }
    pibridge::room::Json json(const QString& p) const { return *pibridge::room::parseJson(readAll(p).toStdString()); }
};

std::time_t fixedNow() {
    return 1792404000;
}

} // namespace

// ---------------------------------------------------------------- the driver switch

TEST(driver_switch_registers_monado_switches_chovr_off_and_restores_exactly) {
    SteamFiles f;
    bool vr = false;
    SteamVrDriverSwitch sw(f.files(), [&vr] { return vr; }, fixedNow);
    const auto pv = sw.preview(f.plan());
    CHECK(pv.has_value() && pv->size() == 2);
    CHECK(readAll(f.vrpath) == f.vrpathText && !sw.pending()); // a preview writes nothing
    const auto r = sw.apply(f.plan());
    CHECK(r.has_value() && *r == *pv);
    CHECK(sw.pending() && QFileInfo::exists(f.state));
    // openvrpaths.vrpath: ours appended, CHOVR kept; nothing else changed
    const auto vp = f.json(f.vrpath);
    CHECK(vp.get("external_drivers").size() == 2);
    CHECK(QString::fromStdString(vp.get("external_drivers").at(0).str()) == f.chovr);
    CHECK(QString::fromStdString(vp.get("external_drivers").at(1).str()) == f.ours);
    CHECK(vp.get("jsonid").str() == "vrpathreg" && vp.members().front().first == "config");
    // steamvr.vrsettings: only driver_CustomHeadsetOpenVR.enable, the section's other key stays
    const auto vs = f.json(f.vrsettings);
    CHECK(vs.get("driver_CustomHeadsetOpenVR").get("enable").isBool());
    CHECK(!vs.get("driver_CustomHeadsetOpenVR").get("enable").boolean());
    CHECK(vs.get("driver_CustomHeadsetOpenVR").get("hasBeenRun").str() == "1.3.0");
    CHECK(vs.get("collisionBounds").get("CollisionBoundsFadeDistance").num() == 0.5);
    CHECK(!vs.has("driver_monado")); // only touched when it says enable false
    // the backups of both files, taken before the change
    const QString stamp = QString::fromStdString(pibridge::room::backupStamp(fixedNow()));
    CHECK(readAll(f.vrpath + QStringLiteral(".bak-") + stamp) == f.vrpathText);
    CHECK(readAll(f.vrsettings + QStringLiteral(".bak-") + stamp) == f.settingsText);
    // the record says what was there before
    const QJsonObject st = QJsonDocument::fromJson(readAll(f.state)).object();
    CHECK(st.value(QStringLiteral("external_added")).toString() == f.ours);
    CHECK(st.value(QStringLiteral("settings")).toArray().size() == 1);
    CHECK(!st.value(QStringLiteral("settings")).toArray()[0].toObject().value(QStringLiteral("had_key")).toBool());
    // a second apply is refused until the restore
    CHECK(!sw.apply(f.plan()).has_value());
    // restore: byte for byte the files of before, the record gone
    const auto back = sw.restore();
    CHECK(back.has_value() && *back);
    CHECK(readAll(f.vrpath) == f.vrpathText);
    CHECK(readAll(f.vrsettings) == f.settingsText);
    CHECK(!sw.pending());
    // idempotent
    const auto again = sw.restore();
    CHECK(again.has_value() && !*again);
    CHECK(readAll(f.vrpath) == f.vrpathText && readAll(f.vrsettings) == f.settingsText);
}

TEST(driver_switch_never_writes_while_vrserver_runs) {
    SteamFiles f;
    bool vr = true;
    SteamVrDriverSwitch sw(f.files(), [&vr] { return vr; }, fixedNow);
    const auto r = sw.apply(f.plan());
    CHECK(!r.has_value() && r.error().contains(QStringLiteral("SteamVR is running")));
    CHECK(readAll(f.vrpath) == f.vrpathText && !sw.pending());
    vr = false;
    CHECK(sw.apply(f.plan()).has_value());
    vr = true; // SteamVR still runs at the end of the session: the restore waits
    const auto w = sw.restore();
    CHECK(!w.has_value() && sw.pending());
    vr = false;
    CHECK(sw.restore().value_or(false) && readAll(f.vrpath) == f.vrpathText);
}

TEST(driver_switch_restores_after_a_crash_and_keeps_steamvrs_own_changes) {
    SteamFiles f(true);
    {
        SteamVrDriverSwitch sw(f.files(), [] { return false; }, fixedNow);
        const auto r = sw.apply(f.plan());
        CHECK(r.has_value() && r->size() == 3); // ours in, the other "monado" out, CHOVR off
        const auto vp = f.json(f.vrpath);
        CHECK(vp.get("external_drivers").size() == 2);
        CHECK(QString::fromStdString(vp.get("external_drivers").at(1).str()) == f.ours);
    } // PiBridge crashes: the switch is gone, the state file stays
    // SteamVR ran and wrote its own things back (a new key in the CHOVR section, a new section)
    auto vs = f.json(f.vrsettings);
    vs["driver_CustomHeadsetOpenVR"]["lastRun"] = pibridge::room::Json::string("x");
    vs["dashboard"] = pibridge::room::Json::object();
    vs["dashboard"]["enableDashboard"] = pibridge::room::Json::boolean(true);
    writeAll(f.vrsettings, QByteArray::fromStdString(vs.dump() + "\n"));
    // PiBridge's next start
    SteamVrDriverSwitch next(f.files(), [] { return false; }, fixedNow);
    CHECK(next.pending());
    CHECK(next.restore().value_or(false));
    CHECK(readAll(f.vrpath) == f.vrpathText); // the other build's dir back at its place
    const auto after = f.json(f.vrsettings);
    CHECK(!after.get("driver_CustomHeadsetOpenVR").has("enable"));
    CHECK(after.get("driver_CustomHeadsetOpenVR").get("lastRun").str() == "x");
    CHECK(after.get("dashboard").get("enableDashboard").boolean());
}

TEST(driver_switch_previous_values_missing_files_and_nothing_to_do) {
    {
        // CHOVR explicitly on, driver_monado explicitly off: both come back as they were
        SteamFiles f;
        auto vs = f.json(f.vrsettings);
        vs["driver_CustomHeadsetOpenVR"]["enable"] = pibridge::room::Json::boolean(true);
        vs["driver_monado"] = pibridge::room::Json::object();
        vs["driver_monado"]["enable"] = pibridge::room::Json::boolean(false);
        const QByteArray before = QByteArray::fromStdString(vs.dump() + "\n");
        writeAll(f.vrsettings, before);
        SteamVrDriverSwitch sw(f.files(), [] { return false; }, fixedNow);
        CHECK(sw.apply(f.plan()).has_value());
        const auto on = f.json(f.vrsettings);
        CHECK(!on.get("driver_CustomHeadsetOpenVR").get("enable").boolean());
        CHECK(on.get("driver_monado").get("enable").boolean());
        CHECK(sw.restore().value_or(false));
        CHECK(readAll(f.vrsettings) == before);
    }
    {
        // no steamvr.vrsettings yet: made for the session, removed again
        SteamFiles f(false, false);
        SteamVrDriverSwitch sw(f.files(), [] { return false; }, fixedNow);
        CHECK(sw.apply(f.plan()).has_value());
        CHECK(QFileInfo::exists(f.vrsettings));
        CHECK(sw.restore().value_or(false));
        CHECK(!QFileInfo::exists(f.vrsettings));
    }
    {
        // already set up this way: nothing to change, no state file
        SteamFiles f;
        SteamVrDriverSwitch sw(f.files(), [] { return false; }, fixedNow);
        CHECK(sw.apply(f.plan()).has_value() && sw.restore().value_or(false));
        auto vp = f.json(f.vrpath);
        vp["external_drivers"].push(pibridge::room::Json::string(f.ours.toStdString()));
        writeAll(f.vrpath, QByteArray::fromStdString(vp.dump() + "\n"));
        auto vs = f.json(f.vrsettings);
        vs["driver_CustomHeadsetOpenVR"]["enable"] = pibridge::room::Json::boolean(false);
        writeAll(f.vrsettings, QByteArray::fromStdString(vs.dump() + "\n"));
        const auto r = sw.apply(f.plan());
        CHECK(r.has_value() && r->isEmpty() && !sw.pending());
    }
    {
        // errors in plain words, nothing written
        SteamFiles f;
        SteamVrDriverSwitch sw(f.files(), [] { return false; }, fixedNow);
        DriverPlan none;
        none.driverDir = f.h.path(QStringLiteral("nothing"));
        CHECK(!sw.apply(none).has_value() && !sw.pending());
        QFile::remove(f.vrpath);
        const auto r = sw.apply(f.plan());
        CHECK(!r.has_value() && r.error().contains(QStringLiteral("not set up")));
    }
    CHECK(driverManifestName(QStringLiteral("/nowhere")).isEmpty());
}

TEST(steamvr_settings_file_and_shims) {
    SteamFiles f;
    CHECK(steamVrSettingsFileFor(f.vrpath, QStringLiteral("/h")) == f.vrsettings);
    CHECK(steamVrSettingsFileFor(QStringLiteral("/none"), QStringLiteral("/h")) ==
          QStringLiteral("/h/.local/share/Steam/config/steamvr.vrsettings"));
    const QString text = steamVrShimText(QStringLiteral("/usr/bin/xdg-open"),
                                         {QStringLiteral("PATH=/usr/bin:/bin"), QStringLiteral("HOME=/h"),
                                          QStringLiteral("LD_PRELOAD=/it's.so")});
    CHECK(text.startsWith(QStringLiteral("#!/bin/sh\n")));
    CHECK(text.contains(QStringLiteral("-u SteamAppId")) && text.contains(QStringLiteral("-u LD_PRELOAD")));
    CHECK(text.contains(QStringLiteral("PATH='/usr/bin:/bin'")));     // the user's own value back
    CHECK(text.contains(QStringLiteral("LD_PRELOAD='/it'\\''s.so'"))); // quoted for sh
    CHECK(!text.contains(QStringLiteral("HOME=")));
    CHECK(text.trimmed().endsWith(QStringLiteral("'/usr/bin/xdg-open' \"$@\"")));
    const QString dir = f.h.path(QStringLiteral("shim"));
    const QString path = writeSteamVrShims(dir, {QStringLiteral("PATH=/usr/bin")}, [](const QString& n) {
        return n == QLatin1String("gio") ? QString() : QStringLiteral("/usr/bin/") + n;
    });
    CHECK(path == dir + QStringLiteral(":/usr/bin"));
    CHECK(QFileInfo(dir + QStringLiteral("/xdg-open")).isExecutable());
    CHECK(QFileInfo::exists(dir + QStringLiteral("/steam")) && !QFileInfo::exists(dir + QStringLiteral("/gio")));
    CHECK(writeSteamVrShims(dir, {}, [](const QString&) { return QString(); }).isEmpty());
}

// ---------------------------------------------------------------- the session

namespace {

// START TRACKING with SteamVR + Monado: fake processes, a fake SteamVR (vrserver, the headset, its processes), the
// real driver switch on made-up files.
struct SvRig {
    SteamFiles f;
    RecSetup setup;
    FakeRecProcesses* procs = nullptr;
    qint64 now = 0;
    bool vrserver = false; // SteamVR runs
    bool hmd = false;      // SteamVR reports the headset
    bool steam = true;     // Steam's client runs
    std::vector<SteamVrProc> svProcs; // SteamVR's processes (the ones started since the mark are this start's)
    std::set<int> alive;
    std::vector<std::pair<int, int>> sent;
    bool termKills = true;
    std::unique_ptr<IoSessionControl> c;
    QString shimPath = QStringLiteral("/shim:/usr/bin");

    SvRig() {
        setup = fakeRecSetup(f.h.path(QStringLiteral("recordings")));
        setup.io.mode = TrackingMode::InsideOut;
        setup.steamVrDriverDir = f.ours;
        setup.driverEnvFile = f.h.path(QStringLiteral("xdg_config_home/pibridge/steamvr-driver.env"));
        IoSessionControl::Deps d;
        d.variant = IoVariant::SteamVr;
        auto p = std::make_unique<FakeRecProcesses>();
        procs = p.get();
        d.procs = std::move(p);
        d.setup = [this] { return setup; };
        d.system = [this] {
            RecSystem s;
            s.vrserver = vrserver;
            s.steamClient = steam;
            return s;
        };
        d.probe.socketReady = [this](const QString&) { return now >= 500; };
        d.probe.imuBytes = [](const QString&) -> qint64 { return -1; };
        d.probe.readLog = [](const QString&) { return QString(); };
        d.probe.monoNs = [this] { return now * 1000000; };
        d.probe.steamVrUp = [this] { return vrserver && hmd; };
        d.nowMs = [this] { return now; };
        d.autoTick = false;
        d.vrserverRunning = [this] { return vrserver; };
        auto sw = std::make_shared<SteamVrDriverSwitch>(f.files(), [this] { return vrserver; }, fixedNow);
        d.applyDrivers = [sw](const DriverPlan& pl) { return sw->apply(pl); };
        d.previewDrivers = [sw](const DriverPlan& pl) { return sw->preview(pl); };
        d.restoreDrivers = [sw] { return sw->restore(); };
        d.driversPending = [sw] { return sw->pending(); };
        d.writeShims = [this] { return shimPath; };
        d.bootTicks = [] { return qint64(5000); };
        d.steamVrProcs = [this](qint64 since) {
            SteamVrScan scan;
            scan.procs = svProcs;
            std::vector<SteamVrProc> out;
            for (const SteamVrProc& sp : steamVrProcsSince(scan, since)) {
                if (alive.contains(sp.pid)) {
                    out.push_back(sp);
                }
            }
            return out;
        };
        d.procSignals.send = [this](int pid, int sig) {
            sent.emplace_back(pid, sig);
            if (sig == SIGKILL || termKills) {
                alive.erase(pid);
                if (pid == 9001) { // this start's vrserver
                    vrserver = false;
                    hmd = false;
                }
            }
            return 0;
        };
        d.procSignals.alive = [this](int pid, qint64) { return alive.contains(pid); };
        c = std::make_unique<IoSessionControl>(std::move(d));
    }
    // SteamVR comes up: vrserver and vrcompositor of this start, plus an old leftover that is not this start's
    void steamVrUp() {
        const auto proc = [](int pid, const char* name, qint64 ticks) {
            SteamVrProc p;
            p.pid = pid;
            p.name = QString::fromLatin1(name);
            p.startTicks = ticks;
            return p;
        };
        svProcs = {proc(900, "vrserver", 100), proc(9001, "vrserver", 5100), proc(9002, "vrcompositor", 5200)};
        alive = {900, 9001, 9002};
        vrserver = true;
        hmd = true;
    }
    void at(qint64 ms) {
        now = ms;
        c->tick();
    }
    QStringList names() const {
        QStringList n;
        for (const auto& sp : procs->started) {
            n << sp.name;
        }
        return n;
    }
};

bool has(const QStringList& l, const QString& s) {
    return l.contains(s);
}

} // namespace

TEST(steamvr_monado_readiness_in_plain_words) {
    SvRig r;
    CHECK(r.c->readiness().ready());
    r.vrserver = true;
    r.c->refresh();
    CHECK(r.c->readiness().blocking.join(QString()).contains(QStringLiteral("SteamVR is running. Close it first")));
    r.vrserver = false;
    r.steam = false;
    r.c->refresh();
    CHECK(r.c->readiness().blocking.join(QString()).contains(QStringLiteral("Steam is not running")));
    r.steam = true;
    r.setup.steamVrStartup.clear();
    r.setup.steamVrPreload.clear();
    r.c->refresh();
    CHECK(r.c->readiness().blocking.join(QString()).contains(QStringLiteral("SteamVR was not found")));
    CHECK(r.c->readiness().blocking.join(QString()).contains(QStringLiteral("pimax_rttime_preload.so is missing")));
    r.setup.steamVrDriverDir.clear();
    r.c->refresh();
    CHECK(r.c->readiness().blocking.join(QString()).contains(QStringLiteral("has no SteamVR driver")));
    r.setup = fakeRecSetup(r.f.h.path(QStringLiteral("recordings")));
    r.setup.io.mode = TrackingMode::InsideOut;
    r.setup.connector.clear(); // SteamVR's compositor picks the display
    r.setup.build.missing = {QStringLiteral("PIMAX_AE_MODEL")};
    r.c->refresh();
    CHECK(!r.c->readiness().blocking.join(QString()).contains(QStringLiteral("connector")));
    CHECK(r.c->readiness().blocking.join(QString()).contains(QStringLiteral("lacks a required marker (PIMAX_AE_MODEL)")));
    // the preview of what a start changes
    r.setup.build.missing.clear();
    r.setup.steamVrDriverDir = r.f.ours;
    r.c->refresh();
    CHECK(r.c->driverPreview().has_value() && r.c->driverPreview()->size() == 2);
}

TEST(steamvr_monado_starts_the_room_map_once_steamvr_is_up_and_stops_it_first) {
    SvRig r;
    r.setup.roomSaved = true;
    r.c->refresh();
    CHECK(r.c->start());
    CHECK(readAll(r.setup.driverEnvFile).contains("PIMAX_ANCHOR=1")); // driver_monado gets the anchor hook
    r.at(600);
    CHECK(!r.names().contains(QStringLiteral("roommap")));
    r.steamVrUp();
    r.at(1000);
    CHECK(r.c->state() == RecState::Recording);
    CHECK(r.names().back() == QStringLiteral("roommap"));
    const qint64 room = r.procs->pidOf(QStringLiteral("roommap"));
    r.c->stop();
    CHECK(!r.procs->signalled.empty() && r.procs->signalled[0].pid == room && r.procs->signalled[0].sig == SIGTERM);
    r.at(1000 + kTermWaitMs + kKillWaitMs + 200);
    CHECK(!r.c->active() && !r.c->driversPending());
    CHECK(r.names().count(QStringLiteral("roommap")) == 1);
}

TEST(steamvr_monado_starts_in_order_and_stops_pid_exactly_then_restores) {
    SvRig r;
    CHECK(r.c->start());
    // before anything runs: the driver file, the drivers switched
    CHECK(QString::fromUtf8(readAll(r.setup.driverEnvFile)) == steamvrDriverEnvText(r.c->setup()));
    CHECK(readAll(r.setup.driverEnvFile).contains("PIMAX_SLAM="));
    CHECK(r.c->driversPending());
    CHECK(r.f.json(r.f.vrpath).get("external_drivers").size() == 2);
    CHECK(r.names() == QStringList{QStringLiteral("broker")});
    r.at(600);
    CHECK((r.names() == QStringList{QStringLiteral("broker"), QStringLiteral("ctrltrack"), QStringLiteral("steamvr")}));
    const RecProcSpec& sv = r.procs->started[2];
    CHECK(sv.program == r.setup.steamVrStartup && sv.logPath.endsWith(QStringLiteral("/logs/steamvr.log")));
    CHECK(has(sv.env, QStringLiteral("VALVE_SKIP_RUNTIME_SAFETY=1")) && has(sv.env, QStringLiteral("QT_QPA_PLATFORM=xcb")));
    CHECK(has(sv.env, QStringLiteral("LD_PRELOAD=") + r.setup.steamVrPreload));
    CHECK(has(sv.env, QStringLiteral("PIMAX_STEAMVR_ENV_FILE=") + r.setup.driverEnvFile));
    CHECK(has(sv.env, QStringLiteral("PATH=/shim:/usr/bin")));
    CHECK(r.c->state() == RecState::Starting);
    // vrstartup.sh ends once it handed over: no failure
    r.procs->exit(r.procs->pidOf(QStringLiteral("steamvr")), 0);
    r.at(2000);
    CHECK(r.c->state() == RecState::Starting);
    r.steamVrUp();
    r.at(3000);
    CHECK(r.c->state() == RecState::Recording && r.c->active());
    CHECK(r.c->message().startsWith(QStringLiteral("Tracking runs in SteamVR.")));
    // Stop: SteamVR first (only this start's processes), then the tracker and the broker, then the drivers back
    const qint64 ctrl = r.procs->pidOf(QStringLiteral("ctrltrack"));
    const qint64 broker = r.procs->pidOf(QStringLiteral("broker"));
    r.c->stop();
    CHECK((r.sent == std::vector<std::pair<int, int>>{{9001, SIGTERM}, {9002, SIGTERM}}));
    CHECK(r.alive.contains(900)); // an older SteamVR process is not this start's: never signalled
    CHECK(r.procs->signalled.size() == 2);
    CHECK(r.procs->signalled[0].pid == ctrl && r.procs->signalled[1].pid == broker);
    CHECK(r.c->state() == RecState::Finished && !r.c->active());
    r.at(4000);
    CHECK(!r.c->driversPending());
    CHECK(readAll(r.f.vrpath) == r.f.vrpathText && readAll(r.f.vrsettings) == r.f.settingsText);
    CHECK(r.c->message().contains(QStringLiteral("as before again")));
}

TEST(steamvr_monado_closed_by_the_user_and_steamvr_ignoring_term) {
    {
        SvRig r;
        CHECK(r.c->start());
        r.at(600);
        r.steamVrUp();
        r.at(1000);
        CHECK(r.c->state() == RecState::Recording);
        // the user quits SteamVR: vrserver goes, vrcompositor stays behind
        r.alive.erase(9001);
        r.vrserver = false;
        r.at(1500);
        CHECK((r.sent == std::vector<std::pair<int, int>>{{9002, SIGTERM}}));
        r.at(1600);
        CHECK(r.c->state() == RecState::Finished);
        CHECK(r.c->message().startsWith(QStringLiteral("Stopped: SteamVR was closed.")));
        CHECK(!r.c->driversPending());
    }
    {
        SvRig r;
        r.termKills = false; // SteamVR ignores SIGTERM: SIGKILL after 5 s
        CHECK(r.c->start());
        r.at(600);
        r.steamVrUp();
        r.at(1000);
        r.c->stop();
        CHECK(r.c->state() == RecState::Stopping && r.c->message() == QStringLiteral("Closing SteamVR..."));
        CHECK(r.procs->signalled.empty()); // the tracker and the broker wait for SteamVR
        r.at(1000 + kTermWaitMs);
        CHECK(r.sent.size() == 4 && r.sent[2].second == SIGKILL && r.sent[3].second == SIGKILL);
        r.at(1000 + kTermWaitMs + 100);
        // then what the session started: the launcher (still there here), the tracker, the broker
        CHECK(r.procs->signalled.size() == 3 && r.c->state() == RecState::Finished);
        CHECK(r.procs->signalled[0].pid == r.procs->pidOf(QStringLiteral("steamvr")));
        CHECK(!r.c->driversPending());
    }
}

TEST(steamvr_monado_failures_close_steamvr_and_restore) {
    {
        // SteamVR never reports the headset: failed after 90 s, its processes closed, the drivers back
        SvRig r;
        CHECK(r.c->start());
        r.at(600);
        r.steamVrUp();
        r.hmd = false;
        r.at(600 + kSteamVrStartWaitMs);
        CHECK(r.c->state() == RecState::Failed || r.c->state() == RecState::Stopping);
        r.at(700 + kSteamVrStartWaitMs);
        CHECK(r.c->state() == RecState::Failed);
        CHECK(r.c->message().contains(QStringLiteral("did not come up")));
        CHECK(!r.alive.contains(9001) && !r.alive.contains(9002));
        CHECK(!r.c->driversPending());
    }
    {
        // the launcher hands over at once (code 255, live 2026-10-19): the session waits for SteamVR
        SvRig r;
        CHECK(r.c->start());
        r.at(600);
        r.procs->exit(r.procs->pidOf(QStringLiteral("steamvr")), 255);
        r.at(700);
        r.at(800);
        CHECK(r.c->state() == RecState::Starting);
        r.steamVrUp();
        r.at(1000);
        CHECK(r.c->state() == RecState::Recording);
        r.c->stop();
    }
    {
        // the start script crashes at once
        SvRig r;
        CHECK(r.c->start());
        r.at(600);
        r.procs->exit(r.procs->pidOf(QStringLiteral("steamvr")), 0, true);
        r.at(700);
        r.at(800);
        CHECK(r.c->state() == RecState::Failed && r.c->message().contains(QStringLiteral("start script ended")));
        CHECK(!r.c->driversPending());
    }
    {
        // the drivers cannot be switched: nothing starts
        SvRig r;
        QFile::remove(r.f.vrpath);
        CHECK(!r.c->start());
        CHECK(r.procs->started.empty() && r.c->state() == RecState::Failed);
        CHECK(r.c->message().contains(QStringLiteral("could not be switched")));
    }
    {
        // PiBridge closes during the session: SteamVR closed, the rest stopped, the drivers back
        SvRig r;
        CHECK(r.c->start());
        r.at(600);
        r.steamVrUp();
        r.at(1000);
        r.c->shutdown();
        CHECK(r.alive == std::set<int>{900});
        CHECK(!r.c->driversPending());
        CHECK(readAll(r.f.vrpath) == r.f.vrpathText);
    }
}

TEST(steamvr_monado_restore_waits_for_steamvr_and_runs_at_the_next_start) {
    SteamFiles f;
    {
        SteamVrDriverSwitch sw(f.files(), [] { return false; }, fixedNow);
        CHECK(sw.apply([&f] {
                  DriverPlan p;
                  p.driverDir = f.ours;
                  return p;
              }())
                  .has_value());
    } // a crash: the files stay switched
    // the next PiBridge: SteamVR (started by hand meanwhile) still runs, so the restore waits and blocks a start
    SvRig r; // only its made-up setup is used here
    bool vr = true;
    IoSessionControl::Deps d;
    d.variant = IoVariant::SteamVr;
    d.procs = std::make_unique<FakeRecProcesses>();
    d.setup = [&r] { return r.setup; };
    d.system = [&vr] {
        RecSystem s;
        s.vrserver = vr;
        return s;
    };
    d.nowMs = [] { return qint64(0); };
    d.autoTick = false;
    d.vrserverRunning = [&vr] { return vr; };
    auto sw = std::make_shared<SteamVrDriverSwitch>(f.files(), [&vr] { return vr; }, fixedNow);
    d.restoreDrivers = [sw] { return sw->restore(); };
    d.driversPending = [sw] { return sw->pending(); };
    IoSessionControl c(std::move(d));
    CHECK(c.driversPending());
    CHECK(c.readiness().blocking.join(QString()).contains(QStringLiteral("put back first")));
    vr = false;
    c.refresh();
    CHECK(!c.driversPending());
    CHECK(readAll(f.vrpath) == f.vrpathText && readAll(f.vrsettings) == f.settingsText);
}

TEST(steamvr_processes_of_one_start_and_boot_ticks) {
    SteamVrScan scan;
    SteamVrProc a, b, c;
    a.pid = 1;
    a.startTicks = 10;
    b.pid = 2;
    b.startTicks = 20;
    c.pid = 3;
    c.startTicks = 30;
    c.inRuntime = false; // holds a socket but is not SteamVR
    scan.procs = {a, b, c};
    const auto p = steamVrProcsSince(scan, 15);
    CHECK(p.size() == 1 && p[0].pid == 2);
    CHECK(steamVrProcsSince(scan, -1).empty()); // no mark: nothing is this start's
    Home h;
    writeAll(h.path(QStringLiteral("proc/uptime")), "123.45 999.00\n");
    CHECK(bootTicksNow(h.path(QStringLiteral("proc")), 100) == 12344);
    CHECK(bootTicksNow(h.path(QStringLiteral("none")), 100) == -1);
}

TEST(headset_card_words_while_monado_directly_runs) {
    CHECK(monadoHeadsetWords(RecState::Recording, {}).state == QStringLiteral("Monado: running"));
    CHECK(monadoHeadsetWords(RecState::Recording, {}).hint.contains(QStringLiteral("Development page")));
    CHECK(monadoHeadsetWords(RecState::Starting, QStringLiteral("Starting Monado...")).hint ==
          QStringLiteral("Starting Monado..."));
    CHECK(monadoHeadsetWords(RecState::Idle, {}).state.isEmpty());
    CHECK(monadoHeadsetWords(RecState::Finished, {}).state.isEmpty());
}

// ---------------------------------------------------------------- the Monado build

TEST(monado_build_markers_and_kind) {
    static const char kRust[] = "\x7f" "ELF\0\0\0\0PIMAX_SLAM\0PIMAX_ANCHOR\0PIMAX_EYE x PIMAX_CTRL_TRACK\0"
                                "pose held from the tracker\0PIMAX_AE_MODEL\0PIMAX_SLAM_DRIFT_HOLD\0";
    const QByteArray rust(kRust, sizeof kRust - 1);
    MonadoBuildInfo b = scanMonadoBinary(rust, QStringLiteral("/b/monado-service"));
    CHECK(b.ok() && b.kind == BuildKind::Rust && b.abiText.isEmpty());
    CHECK(buildSummary(b).startsWith(QStringLiteral("Rust:")));
    CHECK(buildSummary(b).endsWith(QStringLiteral("All 7 required markers present.")));
    CHECK(buildBlockingReason(b).isEmpty());
    QByteArray pibridge = rust;
    pibridge += QByteArray("pibridge_pimax_abi_check\0PIBRIDGE_PIMAX_ABI\0", 38);
    pibridge += QByteArray("PIBRIDGE_PIMAX_ABI 1 spec omarchyvr 5ffbcc5\0", 41);
    b = scanMonadoBinary(pibridge);
    CHECK(b.kind == BuildKind::PiBridge && b.abiText == QStringLiteral("PIBRIDGE_PIMAX_ABI 1 spec omarchyvr 5ffbcc5"));
    CHECK(buildKindWords(b.kind) == QStringLiteral("C++ (PiBridge)"));
    CHECK(buildSummary(b).startsWith(QStringLiteral("C++ (PiBridge): PIBRIDGE_PIMAX_ABI 1 spec omarchyvr 5ffbcc5.")));
    QByteArray old = rust;
    old.replace("PIMAX_AE_MODEL", "XXXXX_XX_XXXXX");
    old.replace("PIMAX_SLAM_DRIFT_HOLD", "XXXXXXXXXXXXXXXXXXXXX");
    b = scanMonadoBinary(old, QStringLiteral("/old/monado-service"));
    CHECK((b.missing == QStringList{QStringLiteral("PIMAX_AE_MODEL"), QStringLiteral("PIMAX_SLAM_DRIFT_HOLD")}));
    CHECK(buildBlockingReason(b).contains(QStringLiteral("/old/monado-service lacks required markers")));
    // the same through the file (streamed in chunks), and the setup's readiness
    Home h;
    const QString svc = h.path(QStringLiteral("build/src/xrt/targets/service/monado-service"));
    writeAll(svc, QByteArray(9 << 20, 'x') + old); // past the first 8 MB chunk
    b = monadoBuildInfo(svc);
    CHECK(b.checked && b.missing.size() == 2);
    CHECK(!monadoBuildInfo(h.path(QStringLiteral("none"))).checked);
    // the app's way: never on the calling thread; "checking" until the worker filled the cache
    const QString svc2 = h.path(QStringLiteral("b2/monado-service"));
    writeAll(svc2, QByteArray(3 << 20, 'y') + rust);
    std::atomic<bool> done{false};
    MonadoBuildInfo a = monadoBuildInfoAsync(svc2, [&done] { done = true; });
    CHECK(a.checking && !a.checked);
    CHECK(buildSummary(a) == QStringLiteral("Checking the build (markers)..."));
    CHECK(buildBlockingReason(a).contains(QStringLiteral("being checked")));
    for (int i = 0; i < 500 && !done; i++) {
        QThread::msleep(10);
    }
    a = monadoBuildInfoAsync(svc2);
    CHECK(done && a.checked && !a.checking && a.ok());
    RecSetup s = fakeRecSetup(h.path(QStringLiteral("rec")));
    s.build = b;
    s.monado.path = svc;
    CHECK(recReadiness(s, {}).blocking.join(QString()).contains(QStringLiteral("lacks required markers")));
}

TEST(monado_build_choices_and_driver_dir) {
    const auto c = monadoBuildChoices(QStringLiteral("/h"));
    CHECK(c.size() == 3 && c[0].id == QStringLiteral("auto") && c[0].service.isEmpty());
    CHECK(c[1].service == QStringLiteral("/h/gamedev/ovr-build/monado/src/xrt/targets/service/monado-service"));
    CHECK(c[2].service == QStringLiteral("/h/gamedev/pibridge-monado-build/src/xrt/targets/service/monado-service"));
    CHECK(monadoBuildChoiceOf({}, QStringLiteral("/h")) == QStringLiteral("auto"));
    CHECK(monadoBuildChoiceOf(QStringLiteral("~/gamedev/pibridge-monado-build/src/xrt/targets/service/monado-service"),
                              QStringLiteral("/h")) == QStringLiteral("pibridge"));
    CHECK(monadoBuildChoiceOf(c[1].service, QStringLiteral("/h")) == QStringLiteral("rust"));
    CHECK(monadoBuildChoiceOf(QStringLiteral("/opt/monado-service"), QStringLiteral("/h")) == QStringLiteral("other"));
    Home h;
    const QString svc = h.path(QStringLiteral("b/src/xrt/targets/service/monado-service"));
    writeAll(svc, "");
    CHECK(steamVrDriverDirFor(svc).isEmpty());
    writeAll(h.path(QStringLiteral("b/steamvr-monado/driver.vrdrivermanifest")), "{\"name\": \"monado\"}");
    CHECK(steamVrDriverDirFor(svc) == h.path(QStringLiteral("b/steamvr-monado")));
    // an installed omarchyvr build: <lib>/monado/bin/monado-service next to <lib>/monado/steamvr-monado
    const QString inst = h.path(QStringLiteral("lib/monado/bin/monado-service"));
    writeAll(inst, "");
    writeAll(h.path(QStringLiteral("lib/monado/steamvr-monado/driver.vrdrivermanifest")), "{\"name\": \"monado\"}");
    CHECK(steamVrDriverDirFor(inst) == h.path(QStringLiteral("lib/monado/steamvr-monado")));
}

TEST(cap_sys_nice_reading_and_fix_permissions) {
    const auto caps = [](quint32 magic, quint32 permitted, int size) {
        QByteArray x(size, '\0');
        qToLittleEndian(magic, x.data());
        qToLittleEndian(permitted, x.data() + 4);
        return x;
    };
    // cap_sys_nice=ep (VFS_CAP_REVISION_2 | effective)
    CapState c = parseFileCaps(caps(0x02000001u, 1u << 23, 20));
    CHECK(c.readable && c.permitted && c.effective && c.ok());
    CHECK(capWords(c, true) == QStringLiteral("Realtime priority allowed (cap_sys_nice=ep)."));
    c = parseFileCaps(caps(0x02000000u, 1u << 23, 20)); // =p only
    CHECK(c.permitted && !c.effective && !c.ok());
    c = parseFileCaps(caps(0x03000001u, 1u << 21, 24)); // revision 3, another capability
    CHECK(c.readable && !c.permitted);
    c = parseFileCaps(caps(0x01000001u, 1u << 23, 12)); // revision 1
    CHECK(c.ok());
    c = parseFileCaps({}); // no attribute
    CHECK(c.readable && !c.ok());
    CHECK(capWords(c, true).contains(QStringLiteral("cap_sys_nice missing")));
    CHECK(!parseFileCaps(caps(0x02000001u, 1u << 23, 12)).readable); // too short for its revision
    CHECK(!parseFileCaps(QByteArray("abc")).readable);
    CHECK(capWords(c, false) == QStringLiteral("monado-service was not found."));
    Home h;
    const QString svc = h.path(QStringLiteral("monado-service"));
    writeAll(svc, "x");
    CHECK(fileCaps(svc).readable && !fileCaps(svc).ok()); // a plain file has none
    // Fix permissions: only when asked, through the runner, pkexec setcap on that file
    FakeRunner runner;
    QString words;
    bool ok = false;
    runCapFix(runner, svc, [&](bool o, const QString& w) {
        ok = o;
        words = w;
    });
    CHECK(runner.calls.size() == 1);
    CHECK(runner.calls[0].line() == QStringLiteral("pkexec setcap cap_sys_nice=ep ") + svc && !runner.calls[0].detached);
    CHECK(ok && words.startsWith(QStringLiteral("Done")));
    runner.exitCode = 126;
    runCapFix(runner, svc, [&](bool o, const QString& w) {
        ok = o;
        words = w;
    });
    CHECK(!ok && words == QStringLiteral("The password prompt was closed; nothing was changed."));
    runner.exitCode = 1;
    runner.output = QStringLiteral("Failed to set capabilities");
    runCapFix(runner, svc, [&](bool o, const QString& w) {
        ok = o;
        words = w;
    });
    CHECK(!ok && words == QStringLiteral("setcap failed (code 1): Failed to set capabilities"));
    runCapFix(runner, h.path(QStringLiteral("none")), [&](bool o, const QString& w) {
        ok = o;
        words = w;
    });
    CHECK(runner.calls.size() == 3 && !ok); // a missing file: nothing is run
    // the setup's warning for monado-service starts, none for SteamVR + Monado
    RecSetup s = fakeRecSetup(h.path(QStringLiteral("rec")));
    s.caps = CapState{true, false, false};
    CHECK(recReadiness(s, {}).warnings.join(QString()).contains(QStringLiteral("Fix permissions")));
    CHECK(!recReadiness(s, {}, RecUse::SteamVr).warnings.join(QString()).contains(QStringLiteral("Fix permissions")));
    s.caps = CapState{true, true, true};
    CHECK(!recReadiness(s, {}).warnings.join(QString()).contains(QStringLiteral("Fix permissions")));
}

// ---------------------------------------------------------------- the daemons let go of the headset

TEST(daemons_pause_before_an_inside_out_start) {
    // the words of the status files
    const auto alive = [](qint64 p) { return p == 11 || p == 12; };
    CHECK(daemonsHolding({}, {}, alive).isEmpty()); // not running
    CHECK((daemonsHolding("{\"state\": \"idle\", \"pid\": 11}", "{\"state\": \"streaming\", \"pid\": 12}", alive) ==
           QStringList{QStringLiteral("the headset service (pibridge-keepalived)"), QStringLiteral("the eye tracker (pibridge-eyed)")}));
    CHECK(daemonsHolding("{\"state\": \"paused\", \"pid\": 11}", "{\"state\": \"idle\", \"pid\": 99}", alive).isEmpty());
    CHECK(insideOutMarkerText(4242, IoVariant::SteamVr) == QStringLiteral("pibridge-inside-out 1\nowner 4242\nvariant steamvr\n"));
    const auto m = parseInsideOutMarker(insideOutMarkerText(7, IoVariant::MonadoDirect).toLatin1());
    CHECK(m && m->first == 7 && m->second == QStringLiteral("monado"));
    CHECK(!parseInsideOutMarker("other\n"));

    // the session: the marker first, nothing started until both daemons say "paused"
    for (const bool steamvr : {true, false}) {
        SvRig r;
        std::vector<bool> markers;
        QStringList holding{QStringLiteral("the headset service (pibridge-keepalived)")};
        IoSessionControl::Deps d;
        d.variant = steamvr ? IoVariant::SteamVr : IoVariant::MonadoDirect;
        auto p = std::make_unique<FakeRecProcesses>();
        FakeRecProcesses* procs = p.get();
        d.procs = std::move(p);
        d.setup = [&r] { return r.setup; };
        d.system = [] { return RecSystem{}; };
        d.probe.socketReady = [](const QString&) { return false; };
        d.probe.imuBytes = [](const QString&) -> qint64 { return -1; };
        d.probe.readLog = [](const QString&) { return QString(); };
        d.probe.monoNs = [&r] { return r.now * 1000000; };
        d.nowMs = [&r] { return r.now; };
        d.autoTick = false;
        d.vrserverRunning = [] { return false; };
        d.marker = [&markers](bool on) {
            markers.push_back(on);
            return true;
        };
        d.daemonsHolding = [&holding] { return holding; };
        IoSessionControl c(std::move(d));
        markers.clear();
        CHECK(c.start());
        CHECK(markers == std::vector<bool>{true} || (markers.size() == 2 && markers[1])); // written before anything
        CHECK(c.active() && c.state() == RecState::Starting && procs->started.empty());
        CHECK(c.message().contains(QStringLiteral("let go of the headset")));
        CHECK(!QFileInfo::exists(r.setup.driverEnvFile) || !steamvr); // not even the driver file yet
        r.now = 1000;
        c.tick();
        CHECK(procs->started.empty());
        holding.clear(); // both report "paused"
        r.now = 1500;
        c.tick();
        CHECK(procs->started.size() == 1 && procs->started[0].name == QStringLiteral("broker"));
        // stop: the marker goes once everything ended
        c.stop();
        r.now = 2000;
        c.tick();
        c.tick();
        CHECK(!c.active() && !markers.empty() && !markers.back());
    }
    // a daemon that never lets go: a plain error after 15 s, the marker removed, nothing started
    {
        SvRig r;
        std::vector<bool> markers;
        IoSessionControl::Deps d;
        d.variant = IoVariant::MonadoDirect;
        auto p = std::make_unique<FakeRecProcesses>();
        FakeRecProcesses* procs = p.get();
        d.procs = std::move(p);
        d.setup = [&r] { return r.setup; };
        d.system = [] { return RecSystem{}; };
        d.nowMs = [&r] { return r.now; };
        d.autoTick = false;
        d.marker = [&markers](bool on) {
            markers.push_back(on);
            return true;
        };
        d.daemonsHolding = [] { return QStringList{QStringLiteral("the eye tracker (pibridge-eyed)")}; };
        IoSessionControl c(std::move(d));
        CHECK(c.start());
        r.now = kDaemonReleaseMs;
        c.tick();
        CHECK(!c.active() && c.state() == RecState::Failed && procs->started.empty());
        CHECK(c.message().startsWith(QStringLiteral("The eye tracker (pibridge-eyed) did not let go of the headset within 15 s")));
        CHECK(!markers.back());
    }
}

// ---------------------------------------------------------------- recenter (Monado's PIMAX_ANCHOR offset)

namespace {

// rotate v by the quaternion q (x y z w)
std::array<double, 3> rot(const std::array<double, 4>& q, const std::array<double, 3>& v) {
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    const std::array<double, 3> u{x, y, z};
    const auto cross = [](const std::array<double, 3>& a, const std::array<double, 3>& b) {
        return std::array<double, 3>{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    };
    const auto t = cross(u, v);
    const std::array<double, 3> t2{2 * t[0], 2 * t[1], 2 * t[2]};
    const auto c = cross(u, t2);
    return {v[0] + w * t2[0] + c[0], v[1] + w * t2[1] + c[1], v[2] + w * t2[2] + c[2]};
}

} // namespace

TEST(anchor_formats_are_omarchyvrs) {
    AnchorPose o;
    o.t = {0.1, -0.9, 0.2};
    o.q = {0, 0.0499792, 0, 0.9987503};
    // world.rs offset_text: {:.6} for the position, {:.7} for the quaternion
    CHECK(anchorOffsetText(4242, o) ==
          QStringLiteral("pimax-anchor-offset 1\npid 4242\npose 0.100000 -0.900000 0.200000 0.0000000 0.0499792 0.0000000 0.9987503\n"));
    const auto back = parseAnchorOffset(anchorOffsetText(4242, o).toLatin1());
    CHECK(back && back->first == 4242 && std::fabs(back->second.q[1] - 0.0499792) < 1e-9);
    CHECK(!parseAnchorOffset("pimax-anchor-offset 2\npid 1\npose 0 0 0 0 0 0 1\n"));
    CHECK(!parseAnchorOffset("pimax-anchor-offset 1\npid 1\npose 0 0 0\n"));
    // world.rs Lease::text and the app's schema 1 line
    AnchorLease l{QStringLiteral("roommap"), 1092498, 77, QStringLiteral("setup"), 2};
    CHECK(l.text() == QStringLiteral("roommap 1092498\nlease 2\nwriter 77\nrole setup\n"));
    const auto lp = AnchorLease::parse(l.text().toLatin1());
    CHECK(lp && lp->session == 1092498 && lp->writer == 77 && lp->role == QStringLiteral("setup") && lp->schema == 2);
    const auto old = AnchorLease::parse("anchor 42\n");
    CHECK(old && old->kind == QStringLiteral("anchor") && old->writer == 0 && old->schema == 1 && old->role.isEmpty());
    const auto newer = AnchorLease::parse("roommap 5\nlease 3\nwriter 9\nrole auto\n");
    CHECK(newer && newer->writer == 0 && newer->role.isEmpty());
    // world.rs ring_bytes layout
    QByteArray ring(kRingHdr + kRingSlots * kRingSlot, '\0');
    const auto put32 = [&ring](int off, quint32 v) { qToLittleEndian(v, ring.data() + off); };
    put32(0, kRingMagic);
    put32(4, 1);
    put32(12, 31337);
    qToLittleEndian<quint64>(900, ring.data() + kRingHdr);
    const float pose[7] = {0.25f, 1.6f, -0.5f, 0, 0, 0, 1};
    std::memcpy(ring.data() + kRingHdr + 8, pose, sizeof pose);
    put32(kRingHdr + 36, 1);
    const auto h = parseRingNewest(ring);
    CHECK(h && h->pid == 31337 && h->tsNs == 900 && std::fabs(h->pose.t[1] - 1.6) < 1e-6);
    put32(8, 5); // being written
    CHECK(!parseRingNewest(ring));
    // the result file of pibridge-roomsetup --recenter
    const auto r = parseRecenterResult("pibridge-recenter 1\npid 77\nhead 0.5 1.7 -1 0 0 0 1\nfloor 0.0400\n");
    CHECK(r && r->session == 77 && r->floorStage && std::fabs(*r->floorStage - 0.04) < 1e-12 && r->head.t[1] == 1.7);
    CHECK(parseRecenterResult("pibridge-recenter 1\npid 77\nhead 0 0 0 0 0 0 1\nfloor keep\n")->floorStage == std::nullopt);
    CHECK(!parseRecenterResult("pibridge-recenter 1\npid 77\nhead 0 0 0 0 0 0 1\nfloor 9\n"));
}

TEST(recenter_puts_the_head_at_the_centre_facing_forward) {
    AnchorPose head;
    head.t = {1.0, 1.7, 2.0};
    const double psi = -M_PI / 2; // facing +X
    head.q = {0, std::sin(psi / 2), 0, std::cos(psi / 2)};
    AnchorPose cur;
    cur.t = {0.3, -0.3, 0.1};
    for (const auto& [current, floor, wantY] :
         std::vector<std::tuple<std::optional<AnchorPose>, std::optional<double>, double>>{
             {std::nullopt, std::nullopt, 1.7}, {cur, std::nullopt, 1.4}, {cur, 0.04, 1.36}}) {
        const AnchorPose o = recenterOffset(head, current, floor);
        CHECK(std::fabs(o.q[0]) < 1e-12 && std::fabs(o.q[2]) < 1e-12); // yaw only: gravity stays
        const auto p = rot(o.q, head.t);
        const std::array<double, 3> app{p[0] + o.t[0], p[1] + o.t[1], p[2] + o.t[2]};
        CHECK(std::fabs(app[0]) < 1e-9 && std::fabs(app[2]) < 1e-9);
        CHECK(std::fabs(app[1] - wantY) < 1e-9);
        // the head's forward in the app world is -Z
        const auto fwd = rot(o.q, rot(head.q, {0, 0, -1}));
        CHECK(std::fabs(fwd[0]) < 1e-9 && std::fabs(fwd[2] + 1) < 1e-9);
    }
}

TEST(recenter_writer_takes_the_lease_and_never_ends_another_writer) {
    Home h;
    AnchorFiles f;
    f.offset = h.path(QStringLiteral("run/pimax-anchor.offset"));
    f.ring = h.path(QStringLiteral("run/pimax-anchor.head"));
    QDir().mkpath(h.path(QStringLiteral("run")));
    RecenterInput in;
    in.session = 500;
    in.head.t = {0.2, 1.6, -0.4};
    std::set<quint32> alive{500, 600};
    const auto live = [&alive](quint32 p) { return alive.contains(p); };
    auto w = writeRecenterOffset(f, in, 900, live);
    CHECK(w.has_value() && w->words.contains(QStringLiteral("the floor stays")));
    CHECK(readAll(f.lease()) == "roommap 500\nlease 2\nwriter 900\nrole setup\n");
    const auto off = parseAnchorOffset(readAll(f.offset));
    CHECK(off && off->first == 500 && std::fabs(off->second.t[0] + 0.2) < 1e-6 && std::fabs(off->second.t[2] - 0.4) < 1e-6);
    CHECK(!QFileInfo::exists(f.offset + QStringLiteral(".tmp")));
    // a second recenter with a floor touch builds on the current offset's height
    in.floorStage = 0.05;
    w = writeRecenterOffset(f, in, 900, live);
    CHECK(w.has_value() && std::fabs(w->offset.t[1] + 0.05) < 1e-9);
    // another live writer of this session (omarchyvr's room setup): refused, nothing changed, nobody signalled
    writeAll(f.lease(), "roommap 500\nlease 2\nwriter 600\nrole setup\n");
    const QByteArray before = readAll(f.offset);
    w = writeRecenterOffset(f, in, 900, live);
    CHECK(!w.has_value() && w.error().contains(QStringLiteral("the room setup (PID 600)")));
    CHECK(readAll(f.offset) == before && alive.contains(600));
    // that writer is gone: PiBridge takes the lease
    alive.erase(600);
    CHECK(writeRecenterOffset(f, in, 900, live).has_value());
    // the AprilTag placed this session: one of them per session
    writeAll(f.lease(), "anchor 500\nlease 2\nwriter 601\nrole tag\n");
    w = writeRecenterOffset(f, in, 900, live);
    CHECK(!w.has_value() && w.error().contains(QStringLiteral("AprilTag")));
    // a new Monado session: the old lease does not count
    in.session = 501;
    CHECK(writeRecenterOffset(f, in, 900, live).has_value());
    CHECK(readAll(f.lease()).startsWith("roommap 501\n"));
    in.session = 0;
    CHECK(!writeRecenterOffset(f, in, 900, live).has_value());
}

PIBRIDGE_TEST_MAIN()
