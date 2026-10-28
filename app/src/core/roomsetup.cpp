// SPDX-License-Identifier: MIT
#include "roomsetup.h"
#include "vr_backend.h"

#include <QDateTime>
#include <QLocale>
#include <cmath>
#include <ctime>
#include <filesystem>

#include "pibridge/room/geom.h"
#include "pibridge/room/json.h"

namespace pibridge::app {

namespace room = pibridge::room;

room::Env roomEnv(bool fake) {
    room::Env env = room::Env::fromProcess();
    if (fake) {
        const std::string data = qEnvironmentVariable("XDG_DATA_HOME").toStdString();
        env.home = data + "/fake-home"; // never the real ~/.steam
    }
    return env;
}

void writeFakeRoomFiles(const room::Env& env) {
    const std::string cfg = env.dataHome() + "/Steam/config";
    std::error_code ec;
    std::filesystem::create_directories(cfg, ec);
    // a 4.2 x 3.4 m room with one corner cut off (a wardrobe), standing origin 0.3 m off the middle
    const room::Poly a{{-2.1, -1.7}, {2.1, -1.7}, {2.1, 0.9}, {1.3, 1.7}, {-2.1, 1.7}};
    const room::Poly b = room::rectangle(3, 2);
    const auto uni = [](const room::Poly& p, const char* id, double w, double d, double x, double z, double yaw,
                        const char* time) {
        room::Json u = room::Json::object();
        u["collision_bounds"] = room::makeBounds(p, room::kDefaultHeight);
        u["play_area"] = room::Json::array({room::Json::number(w), room::Json::number(d)});
        room::Json seated = room::Json::object();
        seated["translation"] = room::Json::array({room::Json::number(0), room::Json::number(1.2), room::Json::number(0)});
        seated["yaw"] = room::Json::number(0);
        u["seated"] = seated;
        room::Json st = room::Json::object();
        st["translation"] = room::Json::array({room::Json::number(x), room::Json::number(0), room::Json::number(z)});
        st["yaw"] = room::Json::number(yaw);
        u["standing"] = st;
        u["time"] = room::Json::string(time);
        u["universeID"] = room::Json::string(id);
        return u;
    };
    room::Json doc = room::Json::object();
    doc["jsonid"] = room::Json::string("chaperone_info");
    doc["universes"] = room::Json::array({uni(a, "1111111111", 3.4, 2.6, 0.3, -0.2, 0.25, "Mon Oct 19 10:00:00 2026"),
                                          uni(b, kInsideOutUniverse, 2.5, 1.5, 0, 0, 0, "Sun Oct 18 09:00:00 2026")});
    doc["version"] = room::Json::number(5);
    (void)room::writeFileAtomic(cfg + "/chaperone_info.vrchap", doc.dump() + "\n");

    room::RoomLib lib(room::roomsDir(env));
    room::Room r1;
    r1.name = "Door open";
    r1.universe = "1111111111";
    r1.points = a;
    r1.standing = room::Standing{0.3, 0, -0.2, 0.25};
    r1.playArea = room::Size2{3.4, 2.6};
    (void)lib.save(r1, 1792404000); // 19 Oct 2026
    room::Room r2 = r1;
    r2.name = "Door closed";
    r2.points = {{-2.1, -1.7}, {2.1, -1.7}, {2.1, 1.7}, {-2.1, 1.7}};
    (void)lib.save(r2, 1792407600);
    lib.setActive("Door open");
}

Gate headsetGate(bool steamvrRunning, const QString& toolPath) {
    if (toolPath.isEmpty()) {
        return {false, QStringLiteral("pibridge-roomsetup is not installed (it comes with PiBridge; install PiBridge again).")};
    }
    if (!steamvrRunning) {
        return {false, QStringLiteral("Press START VR on the Dashboard first: the room setup runs in the headset, in the VR session.")};
    }
    return {true, {}};
}

Gate applyGate(bool steamvrRunning, bool haveFile) {
    if (!haveFile) {
        return {false, QStringLiteral("There is no chaperone file yet: run SteamVR's Room Setup once.")};
    }
    if (steamvrRunning) {
        return {false, QStringLiteral("Close SteamVR first: SteamVR keeps the boundary in memory and writes it back "
                                      "when it closes, so a change made here now would be lost. Or edit the room in "
                                      "the headset, which tells SteamVR itself.")};
    }
    return {true, {}};
}

QString activeUniverse(const SteamVrStatus& s) {
    if (s.state != VrLinkState::Connected) {
        return {};
    }
    for (const VrDevice& d : s.devices) {
        if (d.cls == VrClass::Hmd && d.universe != 0) {
            return QString::number(d.universe);
        }
    }
    return {};
}

namespace {

QString num(double v, int prec) {
    return QLocale(QLocale::English).toString(v, 'f', prec);
}

} // namespace

std::vector<UniverseRow> universeRows(const room::Store& st, const QString& active) {
    std::vector<UniverseRow> out;
    for (const auto& id : st.ids()) {
        const auto& u = st.universes().at(id);
        UniverseRow r;
        r.id = QString::fromStdString(id);
        r.active = r.id == active;
        r.pimax = u.pimax;
        r.monado = r.id == QLatin1String(kInsideOutUniverse);
        QStringList parts;
        parts << QStringLiteral("%1 corners").arg(u.points);
        if (auto doc = room::readJsonFile(u.source)) {
            if (const room::Json* e = room::findUniverse(*doc, id)) {
                const room::UniverseData d = room::universeData(*e);
                parts << QStringLiteral("%1 m²").arg(num(std::fabs(room::polyArea(d.bounds)), 1));
                parts << QStringLiteral("play area %1 x %2 m").arg(num(d.playArea.w, 1), num(d.playArea.h, 1));
            }
        }
        if (!u.time.empty()) {
            parts << QString::fromStdString(u.time);
        }
        r.detail = parts.join(QStringLiteral("  ·  "));
        out.push_back(std::move(r));
    }
    return out;
}

std::vector<RoomRow> roomRows(const room::RoomLib& lib) {
    std::vector<RoomRow> out;
    const QString act = QString::fromStdString(lib.active());
    const QLocale c(QLocale::English);
    for (const auto& r : lib.list()) {
        RoomRow row;
        row.name = QString::fromStdString(r.name);
        row.active = row.name.compare(act, Qt::CaseInsensitive) == 0;
        QStringList parts{QStringLiteral("universe %1").arg(QString::fromStdString(r.universe)),
                          QStringLiteral("%1 corners").arg(r.points.size())};
        const QDateTime t = QDateTime::fromString(QString::fromStdString(r.saved), Qt::ISODate);
        if (t.isValid()) {
            parts << QStringLiteral("saved %1").arg(c.toString(t, QStringLiteral("d MMM yyyy, HH:mm")));
        }
        row.detail = parts.join(QStringLiteral("  ·  "));
        out.push_back(std::move(row));
    }
    return out;
}

QString roomTargetUniverse(bool insideOut, const QString& active) {
    if (!active.isEmpty()) {
        return active;
    }
    return insideOut ? QString::fromLatin1(kInsideOutUniverse) : QString();
}

QString insideOutRoomNote() {
    return QStringLiteral("Inside-out: SteamVR keeps this boundary in universe 30, the one of CustomHeadsetOpenVR's "
                          "inside-out headset. Each start begins a new tracking world; the room map (below) "
                          "puts it back on your room: map the room once, then the floor, the centre and forward are "
                          "recognised at every start. The Lighthouse rooms are not changed.");
}

Gate saveRoomGate(const room::RoomLib& lib, const QString& name, const QString& universe) {
    const std::string file = lib.fileFor(name.trimmed().toStdString());
    for (const room::Room& r : lib.list()) {
        if (lib.fileFor(r.name) != file) {
            continue;
        }
        if (!universe.isEmpty() && QString::fromStdString(r.universe) != universe) {
            const bool monado = r.universe == kInsideOutUniverse;
            return {false, QStringLiteral("\"%1\" is the saved room of universe %2 (%3). Choose another name, for "
                                          "example \"%4\".")
                               .arg(QString::fromStdString(r.name), QString::fromStdString(r.universe),
                                    monado ? QStringLiteral("inside-out") : QStringLiteral("Lighthouse"),
                                    name.trimmed() + (universe == QLatin1String(kInsideOutUniverse)
                                                          ? QStringLiteral(" inside-out")
                                                          : QStringLiteral(" lighthouse")))};
        }
    }
    return {true, {}};
}

Gate showRoomGate(bool insideOut, const QString& roomUniverse, bool roomUniverseInFile) {
    if (insideOut && roomUniverse != QLatin1String(kInsideOutUniverse) && !roomUniverseInFile) {
        return {false, QStringLiteral("This room was saved for universe %1, which is not in SteamVR's chaperone file; "
                                      "it is not put on the inside-out universe 30.")
                           .arg(roomUniverse)};
    }
    return {true, {}};
}

Gate recenterGate(bool insideOut, bool steamvrRunning, bool anchorOn, const QString& toolPath) {
    if (!insideOut) {
        return {false, QStringLiteral("Recenter is for the inside-out mode; base stations keep the room by themselves.")};
    }
    if (toolPath.isEmpty()) {
        return {false, QStringLiteral("pibridge-roomsetup is not installed (it comes with PiBridge; install PiBridge again).")};
    }
    if (!anchorOn) {
        return {false, QStringLiteral("Turn Room anchor on (Tracking page, Head tracking), then start tracking again: "
                                      "the recenter moves the tracker's world through it.")};
    }
    if (!steamvrRunning) {
        return {false, QStringLiteral("Start tracking first (Tracking page): the recenter runs in the headset.")};
    }
    return {true, {}};
}

QStringList recenterArgs(const QString& out, const QString& ring) {
    return QStringList{QStringLiteral("--recenter"), QStringLiteral("--recenter-out"), out, QStringLiteral("--ring"), ring}
           << vrBackendArgs(currentVrBackend());
}

QString recenterReminder() {
    return QStringLiteral("With a mapped room (Room page, MAP THE ROOM) the floor, the centre and forward come back by "
                          "themselves after every start. To turn forward for this session only, hold the menu / Pimax "
                          "button for 1.5 s, after the room is recognised.");
}

std::expected<void, QString> addInsideOutUniverse(const QString& file, std::time_t now) {
    auto text = room::readFile(file.toStdString());
    if (!text) {
        return std::unexpected(QStringLiteral("%1 cannot be read.").arg(file));
    }
    auto doc = room::parseJson(*text);
    if (!doc || !doc->isObject()) {
        return std::unexpected(QStringLiteral("%1 is not a chaperone file.").arg(file));
    }
    if (room::findUniverse(*doc, kInsideOutUniverse) != nullptr) {
        return std::unexpected(QStringLiteral("Universe 30 is in %1 already.").arg(file));
    }
    room::Json u = room::Json::object();
    u["collision_bounds"] = room::makeBounds(room::rectangle(2, 2), room::kDefaultHeight);
    u["play_area"] = room::Json::array({room::Json::number(1.5), room::Json::number(1.5)});
    room::Json seated = room::Json::object();
    seated["translation"] = room::Json::array({room::Json::number(0), room::Json::number(0), room::Json::number(0)});
    seated["yaw"] = room::Json::number(0);
    u["seated"] = seated;
    room::Json st = room::Json::object();
    st["translation"] = room::Json::array({room::Json::number(0), room::Json::number(0), room::Json::number(0)});
    st["yaw"] = room::Json::number(0);
    u["standing"] = st;
    char buf[64];
    std::strftime(buf, sizeof buf, "%a %b %e %H:%M:%S %Y", std::localtime(&now));
    u["time"] = room::Json::string(buf);
    u["universeID"] = room::Json::string(kInsideOutUniverse);
    room::Json& list = (*doc)["universes"];
    if (!list.isArray()) {
        list = room::Json::array();
    }
    list.push(std::move(u));
    if (auto b = room::writeFileAtomic(file.toStdString() + ".bak-" + room::backupStamp(now), *text); !b) {
        return std::unexpected(QStringLiteral("The backup of %1 cannot be written.").arg(file));
    }
    if (auto w = room::writeFileAtomic(file.toStdString(), doc->dump() + "\n"); !w) {
        return std::unexpected(QStringLiteral("%1 cannot be written.").arg(file));
    }
    return {};
}

QStringList roomSetupArgs(const QString& universe) {
    if (universe.isEmpty()) {
        return {};
    }
    return {QStringLiteral("--universe"), universe};
}

bool launchRoomSetup(Runner& runner, const QString& toolPath, const QString& universe) {
    return runner.startDetached(toolPath, roomSetupArgs(universe));
}

} // namespace pibridge::app
