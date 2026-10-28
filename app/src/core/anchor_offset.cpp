// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-anchor/src/world.rs (MIT OR Apache-2.0).
#include "anchor_offset.h"
#include "vr_backend.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QtEndian>
#include <cmath>
#include <map>
#include <cstring>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace pibridge::app {

namespace {

QString runtimeDir() {
    const QString x = qEnvironmentVariable("XDG_RUNTIME_DIR");
    return x.isEmpty() ? QStringLiteral("/tmp") : x;
}

QByteArray readAll(const QString& p) {
    QFile f(p);
    return f.open(QIODevice::ReadOnly) ? f.readAll() : QByteArray();
}

// std::fs::write + rename, as the Rust writer does (the temporary name is part of the format's behaviour).
bool writeRename(const QString& tmp, const QString& path, const QByteArray& data) {
    QFile f(tmp);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate) || f.write(data) != data.size()) {
        return false;
    }
    f.close();
    return ::rename(QFile::encodeName(tmp).constData(), QFile::encodeName(path).constData()) == 0;
}

// flock on the lock file next to the offset (the same lock omarchyvr's writers take).
class Lock {
public:
    explicit Lock(const QString& path) {
        QDir().mkpath(QFileInfo(path).absolutePath());
        fd_ = ::open(QFile::encodeName(path).constData(), O_CREAT | O_WRONLY | O_CLOEXEC, 0644);
        if (fd_ >= 0 && ::flock(fd_, LOCK_EX) != 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }
    ~Lock() {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }
    bool ok() const { return fd_ >= 0; }

private:
    int fd_ = -1;
};

using M3 = std::array<std::array<double, 3>, 3>;

M3 quatToMat(const std::array<double, 4>& qq) {
    const double n = std::sqrt(qq[0] * qq[0] + qq[1] * qq[1] + qq[2] * qq[2] + qq[3] * qq[3]);
    const double x = qq[0] / n, y = qq[1] / n, z = qq[2] / n, w = qq[3] / n;
    return {{{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
             {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
             {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}}};
}

QString holderWords(const AnchorLease& l) {
    static const std::map<QString, QString> titles{{QStringLiteral("setup"), QStringLiteral("the room setup")},
                                                   {QStringLiteral("auto"), QStringLiteral("the automatic room placement")},
                                                   {QStringLiteral("follow"), QStringLiteral("the floor follow")},
                                                   {QStringLiteral("reloc"), QStringLiteral("Recognise room")},
                                                   {QStringLiteral("apply"), QStringLiteral("a saved room fix")},
                                                   {QStringLiteral("tag"), QStringLiteral("the AprilTag")}};
    if (const auto it = titles.find(l.role); it != titles.end()) {
        return QStringLiteral("%1 (PID %2)").arg(it->second).arg(l.writer);
    }
    return l.kind == QLatin1String("anchor") ? QStringLiteral("the AprilTag") : QStringLiteral("the room map");
}

} // namespace

AnchorFiles AnchorFiles::defaults() {
    AnchorFiles f;
    // the backend's files (core/vr_backend.h, libpibridge-anchor backend.h), unless named
    const QString dir = vrBackendRuntimeDir(currentVrBackend());
    const QString o = qEnvironmentVariable("PIMAX_ANCHOR_FILE");
    f.offset = o.isEmpty() ? dir + QStringLiteral("/anchor.offset") : o;
    const QString r = qEnvironmentVariable("PIMAX_ANCHOR_HEAD_FILE");
    f.ring = r.isEmpty() ? dir + QStringLiteral("/anchor.head") : r;
    return f;
}

QString AnchorFiles::lease() const {
    return QFileInfo(offset).absolutePath() + QStringLiteral("/omarchyvr-anchor.offset.owner");
}

QString AnchorFiles::lock() const {
    return QFileInfo(offset).absolutePath() + QStringLiteral("/omarchyvr-anchor.offset.lock");
}

QString anchorOffsetText(quint32 pid, const AnchorPose& o) {
    return QString::asprintf("pimax-anchor-offset 1\npid %u\npose %.6f %.6f %.6f %.7f %.7f %.7f %.7f\n", pid, o.t[0],
                             o.t[1], o.t[2], o.q[0], o.q[1], o.q[2], o.q[3]);
}

std::optional<std::pair<quint32, AnchorPose>> parseAnchorOffset(const QByteArray& text) {
    const QList<QByteArray> l = text.split('\n');
    if (l.size() < 3 || l[0].trimmed() != "pimax-anchor-offset 1" || !l[1].startsWith("pid ") || !l[2].startsWith("pose ")) {
        return std::nullopt;
    }
    bool ok = false;
    const quint32 pid = l[1].mid(4).trimmed().toUInt(&ok);
    if (!ok) {
        return std::nullopt;
    }
    const QList<QByteArray> v = l[2].mid(5).simplified().split(' ');
    if (v.size() != 7) {
        return std::nullopt;
    }
    AnchorPose p;
    for (int i = 0; i < 7; i++) {
        const double d = v[i].toDouble(&ok);
        if (!ok) {
            return std::nullopt;
        }
        (i < 3 ? p.t[size_t(i)] : p.q[size_t(i - 3)]) = d;
    }
    return std::pair{pid, p};
}

QString AnchorLease::text() const {
    QString t = QStringLiteral("%1 %2\nlease %3\nwriter %4\n").arg(kind).arg(session).arg(schema).arg(writer);
    if (!role.isEmpty()) {
        t += QStringLiteral("role %1\n").arg(role);
    }
    return t;
}

std::optional<AnchorLease> AnchorLease::parse(const QByteArray& s) {
    const QList<QByteArray> lines = s.split('\n');
    if (lines.isEmpty()) {
        return std::nullopt;
    }
    const QList<QByteArray> w = lines[0].simplified().split(' ');
    if (w.size() < 2) {
        return std::nullopt;
    }
    bool ok = false;
    AnchorLease l;
    l.kind = QString::fromLatin1(w[0]);
    l.session = w[1].toUInt(&ok);
    if (!ok) {
        return std::nullopt;
    }
    for (qsizetype i = 1; i < lines.size(); i++) {
        const QByteArray line = lines[i].trimmed();
        const qsizetype sp = line.indexOf(' ');
        if (sp < 0) {
            continue;
        }
        const QByteArray k = line.left(sp), v = line.mid(sp + 1).trimmed();
        if (k == "lease") {
            l.schema = v.toUInt(&ok);
            if (!ok) {
                return std::nullopt;
            }
        } else if (k == "writer") {
            l.writer = v.toUInt(&ok);
            if (!ok) {
                return std::nullopt;
            }
        } else if (k == "role") {
            static const QStringList roles{QStringLiteral("setup"), QStringLiteral("auto"), QStringLiteral("follow"),
                                           QStringLiteral("reloc"), QStringLiteral("apply"), QStringLiteral("tag")};
            l.role = roles.contains(QString::fromLatin1(v)) ? QString::fromLatin1(v) : QString();
        }
    }
    if (l.schema > 2) { // a newer writer's file: the session and kind only
        l.writer = 0;
        l.role.clear();
    }
    return l;
}

std::optional<RingHead> parseRingNewest(const QByteArray& b) {
    if (b.size() < kRingHdr + kRingSlots * kRingSlot) {
        return std::nullopt;
    }
    const auto u32 = [&b](int o) { return qFromLittleEndian<quint32>(b.constData() + o); };
    if (u32(0) != kRingMagic || u32(4) != 1 || (u32(8) & 1u) != 0) {
        return std::nullopt; // not a ring, another version, or being written
    }
    std::optional<RingHead> best;
    for (int i = 0; i < kRingSlots; i++) {
        const int o = kRingHdr + i * kRingSlot;
        const quint64 ts = qFromLittleEndian<quint64>(b.constData() + o);
        if (ts == 0 || (u32(o + 36) & 1u) == 0 || (best && best->tsNs >= ts)) {
            continue;
        }
        RingHead h;
        h.pid = u32(12);
        h.tsNs = ts;
        for (int k = 0; k < 3; k++) {
            float f;
            std::memcpy(&f, b.constData() + o + 8 + 4 * k, 4);
            h.pose.t[size_t(k)] = qFromLittleEndian(f);
        }
        for (int k = 0; k < 4; k++) {
            float f;
            std::memcpy(&f, b.constData() + o + 20 + 4 * k, 4);
            h.pose.q[size_t(k)] = qFromLittleEndian(f);
        }
        best = h;
    }
    return best;
}

AnchorPose recenterOffset(const AnchorPose& head, const std::optional<AnchorPose>& current,
                          const std::optional<double>& floorStage) {
    const M3 r = quatToMat(head.q);
    // the head's forward (-Z) in the SLAM world; its yaw about +Y (gravity stays)
    const double fx = -r[0][2], fz = -r[2][2];
    const double psi = std::atan2(-fx, -fz); // Ry(psi) * (0, 0, -1) = forward
    const double a = -psi;                   // the offset turns it back onto -Z
    AnchorPose o;
    o.q = {0, std::sin(a / 2), 0, std::cos(a / 2)};
    const double c = std::cos(a), s = std::sin(a);
    // Ry(a) * head position, horizontal part to the origin
    const double hx = c * head.t[0] + s * head.t[2];
    const double hz = -s * head.t[0] + c * head.t[2];
    o.t = {-hx, (current ? current->t[1] : 0.0) - floorStage.value_or(0.0), -hz};
    return o;
}

std::expected<RecenterWritten, QString> writeRecenterOffset(const AnchorFiles& f, const RecenterInput& in, quint32 me,
                                                            const std::function<bool(quint32)>& alive) {
    if (in.session == 0) {
        return std::unexpected(QStringLiteral("No Monado session to recenter (no head pose from the Pimax driver)."));
    }
    Lock lock(f.lock());
    if (!lock.ok()) {
        return std::unexpected(QStringLiteral("The room offset lock %1 cannot be taken.").arg(f.lock()));
    }
    const auto cur = AnchorLease::parse(readAll(f.lease()));
    const auto offset = parseAnchorOffset(readAll(f.offset));
    const bool written = offset && offset->first == in.session;
    const bool mine = cur && cur->session == in.session && cur->writer == me && cur->schema == 2;
    if (!mine && cur && cur->session == in.session) {
        if (cur->writer != 0 && cur->writer != me && alive && alive(cur->writer)) {
            return std::unexpected(QStringLiteral("%1 writes the room offset of this session; PiBridge leaves it alone. "
                                                  "Stop it first, then recenter again.")
                                       .arg(holderWords(*cur)));
        }
        if (written && cur->kind != QLatin1String("roommap")) {
            return std::unexpected(QStringLiteral("This session's room offset came from %1: only one of the room map "
                                                  "and the AprilTag per session.")
                                       .arg(holderWords(*cur)));
        }
    }
    if (!mine) {
        AnchorLease l;
        l.kind = QStringLiteral("roommap");
        l.session = in.session;
        l.writer = me;
        l.role = QStringLiteral("setup");
        l.schema = 2;
        if (!writeRename(f.lease() + QStringLiteral(".tmp"), f.lease(), l.text().toLatin1())) {
            return std::unexpected(QStringLiteral("The lease %1 cannot be written.").arg(f.lease()));
        }
    }
    RecenterWritten w;
    w.offset = recenterOffset(in.head, written ? std::optional<AnchorPose>(offset->second) : std::nullopt, in.floorStage);
    if (!writeRename(f.offset + QStringLiteral(".tmp"), f.offset, anchorOffsetText(in.session, w.offset).toLatin1())) {
        return std::unexpected(QStringLiteral("The room offset %1 cannot be written.").arg(f.offset));
    }
    w.words = in.floorStage
                  ? QStringLiteral("Recentered: the play area centre is where you stood, forward is where you looked, "
                                   "the floor moved by %1 cm.")
                        .arg(-*in.floorStage * 100.0, 0, 'f', 1)
                  : QStringLiteral("Recentered: the play area centre is where you stood, forward is where you looked; "
                                   "the floor stays.");
    return w;
}

std::optional<RecenterInput> parseRecenterResult(const QByteArray& text) {
    const QList<QByteArray> l = text.split('\n');
    if (l.size() < 4 || l[0].trimmed() != "pibridge-recenter 1" || !l[1].startsWith("pid ") || !l[2].startsWith("head ") ||
        !l[3].startsWith("floor ")) {
        return std::nullopt;
    }
    RecenterInput in;
    bool ok = false;
    in.session = l[1].mid(4).trimmed().toUInt(&ok);
    if (!ok) {
        return std::nullopt;
    }
    const auto off = parseAnchorOffset("pimax-anchor-offset 1\npid 1\npose " + l[2].mid(5) + "\n");
    if (!off) {
        return std::nullopt;
    }
    in.head = off->second;
    const QByteArray fl = l[3].mid(6).trimmed();
    if (fl != "keep") {
        const double y = fl.toDouble(&ok);
        if (!ok || std::fabs(y) > 1.5) {
            return std::nullopt;
        }
        in.floorStage = y;
    }
    return in;
}

} // namespace pibridge::app
