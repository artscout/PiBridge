// SPDX-License-Identifier: MIT
#include "roommap_link.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QtEndian>
#include <algorithm>
#include <cerrno>
#include <csignal>
#include <ctime>
#include <sys/types.h>

#include "anchor_offset.h"

namespace pibridge::app {

QString roomName(const Conf& conf) {
    const QString n = conf.value(roomkey::kRoom, QString()).trimmed();
    if (n.isEmpty() || n.contains(QLatin1Char('/')) || n == QLatin1String(".") || n == QLatin1String("..")) {
        return QString::fromLatin1(kDefaultRoom);
    }
    return n;
}

QString roommapBaseDir() {
    for (const char* n : {"PIBRIDGE_ROOMMAP_DIR", "PIMAX_ROOMMAP_DIR"}) {
        if (const QString v = qEnvironmentVariable(n); !v.isEmpty()) {
            return v;
        }
    }
    const QString home = qEnvironmentVariable("HOME", QStringLiteral("."));
    const QString pibridge = home + QStringLiteral("/.local/share/pibridge/roommap");
    const QString omarchyvr = home + QStringLiteral("/.local/share/omarchyvr/roommap");
    if (!QFileInfo(pibridge).isDir() && QFileInfo(omarchyvr).isDir()) {
        return omarchyvr;
    }
    return pibridge;
}

bool roomSaved(const QString& baseDir, const QString& name) {
    return !baseDir.isEmpty() && !name.isEmpty() && QFileInfo(baseDir + QLatin1Char('/') + name + QStringLiteral("/map.bin")).isFile();
}

QStringList roomSetupFastArgs(const QString& name, std::optional<double> eyeHeight) {
    QStringList a{QStringLiteral("setup"), QStringLiteral("fast"), QStringLiteral("--name"), name, QStringLiteral("--no-hud")};
    if (eyeHeight) {
        a << QStringLiteral("--eye-height") << QString::number(*eyeHeight, 'f', 3);
    }
    return a;
}

QStringList roomMapHeadsetArgs(const QString& name, const QString& roommapTool, const QString& ring,
                               std::optional<double> eyeHeight, bool chaperone) {
    QStringList a{QStringLiteral("--map"), QStringLiteral("--room"), name, QStringLiteral("--roommap"), roommapTool,
                  QStringLiteral("--ring"), ring};
    if (eyeHeight) {
        a << QStringLiteral("--eye-height") << QString::number(*eyeHeight, 'f', 3);
    }
    if (chaperone) {
        a << QStringLiteral("--chaperone");
    }
    return a;
}

std::optional<double> parseHeightM(const QString& text, double lo, double hi) {
    QString t = text.trimmed();
    t.replace(QLatin1Char(','), QLatin1Char('.'));
    if (t.endsWith(QLatin1Char('m'))) {
        t.chop(1);
        t = t.trimmed();
    }
    bool ok = false;
    const double v = t.toDouble(&ok);
    if (!ok || !(v >= lo && v <= hi)) {
        return std::nullopt;
    }
    return v;
}

std::optional<double> derivedEyeHeight(const QString& bodyText) {
    const auto b = parseHeightM(bodyText, 1.2, 2.3);
    if (!b) {
        return std::nullopt;
    }
    return *b * kEyeFromBody;
}

std::optional<double> eyeHeightOf(const QString& bodyText, const QString& eyeText) {
    if (const auto e = parseHeightM(eyeText, 1.0, 2.3)) {
        return e;
    }
    return derivedEyeHeight(bodyText);
}

std::optional<double> eyeHeightOf(const Conf& conf) {
    return eyeHeightOf(conf.value(roomkey::kBodyHeight, QString()), conf.value(roomkey::kEyeHeight, QString()));
}

QStringList floorHeightArgs(const QString& name, double eyeHeight, const QString& roommapTool, const QString& ring) {
    return {QStringLiteral("--floor-height"), QStringLiteral("--room"), name, QStringLiteral("--eye-height"),
            QString::number(eyeHeight, 'f', 3), QStringLiteral("--roommap"), roommapTool, QStringLiteral("--ring"), ring};
}

QStringList roomAutoArgs(const QString& name) {
    return {QStringLiteral("auto"), QStringLiteral("--name"), name, QStringLiteral("--wait"), QString::number(kRoomAutoWaitS)};
}

QString roomStatusPath() {
    const QString d = qEnvironmentVariable("XDG_RUNTIME_DIR", QStringLiteral("/tmp"));
    return d + QStringLiteral("/omarchyvr-room.status");
}

std::optional<RoomStatus> parseRoomStatus(const QByteArray& bytes) {
    RoomStatus s;
    bool any = false;
    for (const QString& raw : QString::fromUtf8(bytes).split(QLatin1Char('\n'))) {
        const QString l = raw.trimmed();
        if (l.isEmpty() || l.startsWith(QLatin1Char('#'))) {
            continue;
        }
        const qsizetype eq = l.indexOf(QLatin1Char('='));
        if (eq <= 0) {
            continue;
        }
        const QString k = l.left(eq).trimmed();
        QString v = l.mid(eq + 1).trimmed();
        if (v.size() >= 2 && v.startsWith(QLatin1Char('"')) && v.endsWith(QLatin1Char('"'))) {
            v = v.mid(1, v.size() - 2);
        }
        if (k == QLatin1String("state")) {
            s.state = v;
            any = true;
        } else if (k == QLatin1String("room")) {
            s.room = v;
        } else if (k == QLatin1String("pid")) {
            s.pid = v.toUInt();
        } else if (k == QLatin1String("text")) {
            s.text = v;
        } else if (k == QLatin1String("when")) {
            s.when = v.toLongLong();
        }
    }
    if (!any) {
        return std::nullopt;
    }
    return s;
}

RoomWords roomStatusWords(const std::optional<RoomStatus>& st, quint32 sessionPid, qint64 sinceUnix, bool saved,
                          bool autoRunning, const QString& name) {
    RoomWords w;
    const bool current = st && st->room == name &&
                         ((sessionPid != 0 && st->pid == sessionPid) || (sinceUnix > 0 && st->when >= sinceUnix));
    if (current) {
        w.detail = st->text;
        if (st->state == QLatin1String("ok")) {
            w.look = RoomLook::Recognised;
            w.headline = QStringLiteral("Room recognised");
        } else if (st->state == QLatin1String("searching")) {
            w.look = RoomLook::Searching;
            w.headline = QStringLiteral("Searching for the room");
        } else if (st->state == QLatin1String("failed")) {
            w.look = RoomLook::NotFound;
            w.headline = QStringLiteral("Room not found");
        } else {
            w.look = RoomLook::Other;
            w.headline = QStringLiteral("Room left as it is");
        }
        return w;
    }
    if (autoRunning) {
        w.look = RoomLook::Searching;
        w.headline = QStringLiteral("Searching for the room");
        w.detail = QStringLiteral("Waiting for the head tracking to start; then it looks for the room \"%1\" for up to %2 s.")
                       .arg(name)
                       .arg(kRoomAutoWaitS);
        return w;
    }
    if (!saved) {
        w.look = RoomLook::NoRoom;
        w.headline = QStringLiteral("No saved room");
        w.detail = QStringLiteral("Set up your playspace once (Room page, SETUP PLAYSPACE): then the floor and the room "
                                  "come back by themselves at every start.");
        return w;
    }
    w.look = RoomLook::Saved;
    w.headline = QStringLiteral("Room \"%1\" saved").arg(name);
    w.detail = QStringLiteral("It is looked for at every tracking start.");
    return w;
}

RingFeed readRingFeed() {
    RingFeed f;
    QFile file(AnchorFiles::defaults().ring);
    if (!file.open(QIODevice::ReadOnly)) {
        return f;
    }
    const QByteArray b = file.read(kRingHdr + kRingSlots * kRingSlot);
    if (b.size() >= kRingHdr) {
        f.pid = qFromLittleEndian<quint32>(b.constData() + 12);
    }
    f.alive = f.pid != 0 && (::kill(static_cast<pid_t>(f.pid), 0) == 0 || errno == EPERM);
    if (const auto h = parseRingNewest(b)) {
        timespec ts{};
        clock_gettime(CLOCK_MONOTONIC, &ts);
        const double now = double(ts.tv_sec) + double(ts.tv_nsec) * 1e-9;
        f.ageS = std::max(0.0, now - double(h->tsNs) * 1e-9);
    }
    return f;
}

RoomGate mapRoomGate(bool insideOut, const QString& tool, const RingFeed& feed, bool running) {
    if (!insideOut) {
        return {false, QStringLiteral("The room map is for the inside-out mode (Tracking page).")};
    }
    if (tool.isEmpty()) {
        return {false, QStringLiteral("pibridge-roommap was not found next to pibridge.")};
    }
    if (running) {
        return {false, QStringLiteral("The room is being mapped now.")};
    }
    if (feed.pid == 0 || !feed.alive) {
        return {false, QStringLiteral("No inside-out session runs. Press START TRACKING on the Tracking page, put the "
                                      "headset on, then map the room.")};
    }
    if (feed.ageS < 0 || feed.ageS > 2.0) {
        return {false, QStringLiteral("The session runs but sends no head poses yet. Put the headset on and wait until "
                                      "the head is tracked.")};
    }
    return {true, {}};
}

RoomGate floorHeightGate(bool insideOut, const QString& roommapTool, const QString& roomsetupTool, const RingFeed& feed,
                         std::optional<double> eyeHeight, bool recognised, bool running) {
    if (const RoomGate g = mapRoomHeadsetGate(insideOut, roommapTool, roomsetupTool, feed, running); !g.ok) {
        return g;
    }
    if (!eyeHeight) {
        return {false, QStringLiteral("Enter your body height (or your measured eye height) above first.")};
    }
    if (!recognised) {
        return {false, QStringLiteral("The room is not recognised in this session yet: wait for \"Room recognised\", or map "
                                      "the room.")};
    }
    return {true, {}};
}

RoomGate mapRoomHeadsetGate(bool insideOut, const QString& roommapTool, const QString& roomsetupTool, const RingFeed& feed,
                            bool running) {
    if (insideOut && !roommapTool.isEmpty() && roomsetupTool.isEmpty()) {
        return {false, QStringLiteral("pibridge-roomsetup was not found next to pibridge (it shows the room mapping in the "
                                      "headset; install PiBridge again).")};
    }
    return mapRoomGate(insideOut, roommapTool, feed, running);
}

QString floorWarning(const QByteArray& floorToml) {
    for (const QByteArray& raw : floorToml.split('\n')) {
        const QByteArray line = raw.trimmed();
        if (!line.startsWith("head_above_m")) {
            continue;
        }
        const int eq = line.indexOf('=');
        bool ok = false;
        const double above = eq < 0 ? 0.0 : line.mid(eq + 1).trimmed().toDouble(&ok);
        if (!ok || above >= kFloorMinBelowHeadM) {
            return {};
        }
        return QStringLiteral("The floor the map found is only %1 m below your head: that is probably a table or a bed, "
                              "not the floor. Enter your body height above and run SETUP PLAYSPACE again: the floor is "
                              "then taken from your height, not from the point cloud.")
            .arg(above, 0, 'f', 2);
    }
    return {};
}

} // namespace pibridge::app
