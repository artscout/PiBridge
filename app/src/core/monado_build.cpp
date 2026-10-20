// SPDX-License-Identifier: MIT
#include "monado_build.h"

#include <QByteArrayMatcher>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QMutex>
#include <QSet>
#include <thread>
#include <QtEndian>
#include <cerrno>
#include <sys/xattr.h>

namespace pibridge::app {

QStringList requiredMonadoMarkers() {
    return {QStringLiteral("PIMAX_SLAM"),       QStringLiteral("PIMAX_ANCHOR"),
            QStringLiteral("PIMAX_EYE"),        QStringLiteral("PIMAX_CTRL_TRACK"),
            QStringLiteral("held from the tracker"), QStringLiteral("PIMAX_AE_MODEL"),
            QStringLiteral("PIMAX_SLAM_DRIFT_HOLD")};
}

namespace {

// The printable text from `at` to the next NUL or non-printable byte (at most 200 bytes).
QString cString(const QByteArray& data, qsizetype at) {
    qsizetype end = at;
    while (end < data.size() && end - at < 200) {
        const auto c = static_cast<unsigned char>(data[end]);
        if (c < 0x20 || c > 0x7e) {
            break;
        }
        end++;
    }
    return QString::fromLatin1(data.mid(at, end - at)).trimmed();
}

struct Scanner {
    QStringList needles;
    std::vector<QByteArrayMatcher> matchers;
    std::vector<bool> found;
    QString abi;
    qsizetype keep = 0;
    Scanner() {
        needles = requiredMonadoMarkers();
        for (const QString& n : needles) {
            matchers.emplace_back(n.toLatin1());
            keep = std::max<qsizetype>(keep, n.size());
        }
        found.assign(needles.size(), false);
        keep = std::max<qsizetype>(keep, 220); // the ABI string after its marker
    }
    void feed(const QByteArray& chunk) {
        for (size_t i = 0; i < matchers.size(); i++) {
            if (!found[i] && matchers[i].indexIn(chunk) >= 0) {
                found[i] = true;
            }
        }
        if (abi.isEmpty()) {
            static const QByteArrayMatcher m{QByteArray(kPiBridgeAbiMarker)};
            // the marker as a whole string ("PIBRIDGE_PIMAX_ABI 1 spec ..."), not a symbol name containing it
            for (qsizetype at = m.indexIn(chunk); at >= 0; at = m.indexIn(chunk, at + 1)) {
                const QString s = cString(chunk, at);
                if (s.size() > qsizetype(sizeof(kPiBridgeAbiMarker)) && s.at(qsizetype(sizeof(kPiBridgeAbiMarker)) - 1) == QLatin1Char(' ')) {
                    abi = s;
                    break;
                }
                if (abi.isEmpty() && s == QLatin1String(kPiBridgeAbiMarker)) {
                    abi = s; // the bare marker still says PiBridge
                }
            }
        }
    }
    MonadoBuildInfo result(const QString& path) const {
        MonadoBuildInfo b;
        b.checked = true;
        b.path = path;
        b.abiText = abi;
        b.kind = abi.isEmpty() ? BuildKind::Rust : BuildKind::PiBridge;
        for (size_t i = 0; i < found.size(); i++) {
            if (!found[i]) {
                b.missing << needles[qsizetype(i)];
            }
        }
        return b;
    }
};

} // namespace

MonadoBuildInfo scanMonadoBinary(const QByteArray& data, const QString& path) {
    Scanner s;
    s.feed(data);
    return s.result(path);
}

namespace {

QMutex g_mutex;
QHash<QString, std::pair<QString, MonadoBuildInfo>> g_cache; // path -> (size:mtime, info)
QSet<QString> g_busy;                                         // paths a worker scans now

QString stampOf(const QFileInfo& fi) {
    return QStringLiteral("%1:%2").arg(fi.size()).arg(fi.lastModified().toMSecsSinceEpoch());
}

} // namespace

MonadoBuildInfo monadoBuildInfoAsync(const QString& monadoService, std::function<void()> done) {
    MonadoBuildInfo b;
    b.path = monadoService;
    const QFileInfo fi(monadoService);
    if (monadoService.isEmpty() || !fi.isFile()) {
        return monadoBuildInfo(monadoService); // nothing to read: at once
    }
    const QString key = fi.absoluteFilePath();
    {
        QMutexLocker l(&g_mutex);
        if (const auto it = g_cache.constFind(key); it != g_cache.constEnd() && it->first == stampOf(fi)) {
            return it->second;
        }
        b.checking = true;
        if (g_busy.contains(key)) {
            return b;
        }
        g_busy.insert(key);
    }
    std::thread([monadoService, key, done = std::move(done)] {
        (void)monadoBuildInfo(monadoService); // fills the cache
        {
            QMutexLocker l(&g_mutex);
            g_busy.remove(key);
        }
        if (done) {
            done();
        }
    }).detach();
    return b;
}

MonadoBuildInfo monadoBuildInfo(const QString& monadoService) {
    MonadoBuildInfo none;
    none.path = monadoService;
    if (monadoService.isEmpty()) {
        return none;
    }
    const QFileInfo fi(monadoService);
    if (!fi.isFile()) {
        none.error = QStringLiteral("%1 does not exist.").arg(monadoService);
        return none;
    }
    auto& mutex = g_mutex;
    auto& cache = g_cache;
    const QString stamp = stampOf(fi);
    {
        QMutexLocker l(&mutex);
        if (const auto it = cache.constFind(fi.absoluteFilePath()); it != cache.constEnd() && it->first == stamp) {
            return it->second;
        }
    }
    QFile f(monadoService);
    if (!f.open(QIODevice::ReadOnly)) {
        none.error = QStringLiteral("%1 cannot be read: %2").arg(monadoService, f.errorString());
        return none;
    }
    Scanner s;
    QByteArray tail;
    constexpr qint64 kChunk = 8 << 20;
    while (!f.atEnd()) {
        QByteArray chunk = tail + f.read(kChunk);
        if (chunk.size() == tail.size()) {
            break;
        }
        s.feed(chunk);
        tail = chunk.right(s.keep); // a marker across two chunks
    }
    const MonadoBuildInfo b = s.result(monadoService);
    QMutexLocker l(&mutex);
    cache.insert(fi.absoluteFilePath(), {stamp, b});
    return b;
}

QString buildKindWords(BuildKind k) {
    switch (k) {
    case BuildKind::PiBridge: return QStringLiteral("C++ (PiBridge)");
    case BuildKind::Rust: return QStringLiteral("Rust");
    case BuildKind::Unknown: break;
    }
    return QStringLiteral("unknown");
}

QString buildSummary(const MonadoBuildInfo& b) {
    if (b.checking) {
        return QStringLiteral("Checking the build (markers)...");
    }
    if (!b.checked) {
        return b.error.isEmpty() ? QStringLiteral("No monado-service selected.") : b.error;
    }
    QString s = b.kind == BuildKind::PiBridge
                    ? QStringLiteral("C++ (PiBridge): %1.").arg(b.abiText)
                    : QStringLiteral("Rust: links omarchyvr's Rust archives (no %1 marker).").arg(QLatin1String(kPiBridgeAbiMarker));
    if (b.missing.isEmpty()) {
        s += QStringLiteral(" All %1 required markers present.").arg(requiredMonadoMarkers().size());
    } else {
        s += QStringLiteral(" Missing markers: %1.").arg(b.missing.join(QStringLiteral(", ")));
    }
    return s;
}

QString buildBlockingReason(const MonadoBuildInfo& b) {
    if (b.checking) {
        return QStringLiteral("The Monado build is being checked (its markers); a moment.");
    }
    if (!b.checked || b.missing.isEmpty()) {
        return {};
    }
    return QStringLiteral("The Monado build %1 lacks %2 (%3). It was built without part of the Pimax driver; choose "
                          "the other build on the Development page or rebuild it with the cache options of the live "
                          "build.")
        .arg(b.path, b.missing.size() == 1 ? QStringLiteral("a required marker") : QStringLiteral("required markers"),
             b.missing.join(QStringLiteral(", ")));
}

// ---------------------------------------------------------------- the A/B choice

std::vector<MonadoBuildChoice> monadoBuildChoices(const QString& home) {
    const QString svc = QStringLiteral("/src/xrt/targets/service/monado-service");
    // The Rust-linked (omarchyvr) build was dropped (user 2026-10-20: we use our own C++ build now).
    return {{QStringLiteral("auto"), QStringLiteral("Auto"), QString()},
            {QStringLiteral("pibridge"), QStringLiteral("C++ (PiBridge)"), home + QStringLiteral("/gamedev/pibridge-monado-build") + svc}};
}

QString monadoBuildChoiceOf(const QString& confValue, const QString& home) {
    QString v = confValue.trimmed();
    if (v.isEmpty()) {
        return QStringLiteral("auto");
    }
    if (v.startsWith(QLatin1String("~/"))) {
        v = home + v.mid(1);
    }
    v = QDir::cleanPath(v);
    for (const MonadoBuildChoice& c : monadoBuildChoices(home)) {
        if (!c.service.isEmpty() && QDir::cleanPath(c.service) == v) {
            return c.id;
        }
    }
    return QStringLiteral("other");
}

QString steamVrDriverDirFor(const QString& monadoService) {
    if (monadoService.isEmpty()) {
        return {};
    }
    QDir d = QFileInfo(monadoService).absoluteDir();
    for (int i = 0; i <= 4; i++) {
        const QString p = d.filePath(QStringLiteral("steamvr-monado"));
        if (QFileInfo(p + QStringLiteral("/driver.vrdrivermanifest")).isFile()) {
            return QDir::cleanPath(p);
        }
        if (!d.cdUp()) {
            break;
        }
    }
    return {};
}

// ---------------------------------------------------------------- cap_sys_nice

CapState parseFileCaps(const QByteArray& x) {
    CapState c;
    c.readable = true;
    if (x.isEmpty()) {
        return c; // no capabilities at all
    }
    if (x.size() < 12) {
        c.readable = false;
        return c;
    }
    const auto u32 = [&x](int off) { return qFromLittleEndian<quint32>(x.constData() + off); };
    const quint32 magic = u32(0);
    const quint32 rev = magic & 0xFF000000u;
    const qsizetype need = rev == 0x01000000u ? 12 : rev == 0x02000000u ? 20 : rev == 0x03000000u ? 24 : -1;
    if (need < 0 || x.size() < need) {
        c.readable = false;
        return c;
    }
    const quint32 permitted0 = u32(4);
    c.permitted = (permitted0 >> kCapSysNice) & 1u;
    c.effective = (magic & 0x000001u) != 0; // VFS_CAP_FLAGS_EFFECTIVE
    return c;
}

CapState fileCaps(const QString& path) {
    const QByteArray p = QFile::encodeName(path);
    char buf[64];
    const ssize_t n = ::getxattr(p.constData(), "security.capability", buf, sizeof buf);
    if (n < 0) {
        CapState c;
        c.readable = errno == ENODATA; // no attribute: no capabilities
        return c;
    }
    return parseFileCaps(QByteArray(buf, static_cast<qsizetype>(n)));
}

QString capWords(const CapState& c, bool fileExists) {
    if (!fileExists) {
        return QStringLiteral("monado-service was not found.");
    }
    if (!c.readable) {
        return QStringLiteral("Its permissions could not be read.");
    }
    if (c.ok()) {
        return QStringLiteral("Realtime priority allowed (cap_sys_nice=ep).");
    }
    return QStringLiteral("No realtime priority (cap_sys_nice missing): Monado can judder when the PC is busy.");
}

QStringList capFixArgs(const QString& monadoService) {
    return {QStringLiteral("setcap"), QStringLiteral("cap_sys_nice=ep"), monadoService};
}

void runCapFix(Runner& runner, const QString& monadoService, std::function<void(bool ok, const QString& words)> done) {
    if (monadoService.isEmpty() || !QFileInfo(monadoService).isFile()) {
        if (done) {
            done(false, QStringLiteral("No monado-service to fix."));
        }
        return;
    }
    runner.run(QString::fromLatin1(kPkexec), capFixArgs(monadoService), [done](int code, const QString& out) {
        if (!done) {
            return;
        }
        if (code == 0) {
            done(true, QStringLiteral("Done: monado-service may use realtime priority now."));
        } else if (code == 126) {
            done(false, QStringLiteral("The password prompt was closed; nothing was changed."));
        } else if (code == 127) {
            done(false, QStringLiteral("Not allowed, or no password prompt is available (polkit agent)."));
        } else {
            const QString o = out.trimmed();
            done(false, QStringLiteral("setcap failed (code %1)%2").arg(code).arg(o.isEmpty() ? QStringLiteral(".")
                                                                                                : QStringLiteral(": ") + o.left(200)));
        }
    });
}

} // namespace pibridge::app
