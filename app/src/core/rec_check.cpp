// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-slam/src/euroc.rs (MIT OR Apache-2.0).
#include "rec_check.h"

#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QSet>
#include <algorithm>
#include <cmath>
#include <limits>

namespace pibridge::app {

namespace {

QString f(double v, int prec) {
    return QString::number(v, 'f', prec); // always a point, any locale
}

// (line number, trimmed text) of every line that is not empty and not a # comment
std::vector<std::pair<int, QByteArray>> dataLines(const QByteArray& text) {
    std::vector<std::pair<int, QByteArray>> out;
    int n = 0;
    for (const QByteArray& raw : text.split('\n')) {
        n++;
        const QByteArray l = raw.trimmed();
        if (!l.isEmpty() && !l.startsWith('#')) {
            out.emplace_back(n, l);
        }
    }
    return out;
}

double median(std::vector<double> v) {
    if (v.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

struct Intervals {
    double med = 0, max = 0;
    size_t gaps = 0, back = 0;
};

// median dt s, max dt s, gaps > 1.5 median, non-increasing
Intervals intervals(const std::vector<qint64>& ts) {
    std::vector<double> d;
    d.reserve(ts.size());
    for (size_t i = 1; i < ts.size(); i++) {
        d.push_back(double(ts[i] - ts[i - 1]) / 1e9);
    }
    Intervals r;
    for (double x : d) {
        if (x <= 0.0) {
            r.back++;
        }
        r.max = std::max(r.max, x);
    }
    r.med = median(d);
    for (double x : d) {
        if (x > 1.5 * r.med) {
            r.gaps++;
        }
    }
    return r;
}

QByteArray readAll(const QString& p, QString* error) {
    QFile file(p);
    if (!file.open(QIODevice::ReadOnly)) {
        if (error) {
            *error = QStringLiteral("%1: %2").arg(p, file.errorString());
        }
        return {};
    }
    return file.readAll();
}

bool isRecordingFolder(const QString& p) {
    const QDir d(p);
    if (QFileInfo(d.filePath(QString::fromLatin1(kStepsFile))).isFile()) {
        return true;
    }
    return !d.entryList({QStringLiteral("euroc_*")}, QDir::Dirs | QDir::NoDotAndDotDot).isEmpty();
}

qint64 folderBytes(const QString& p) {
    qint64 n = 0;
    QDirIterator it(p, QDir::Files | QDir::Hidden | QDir::NoSymLinks, QDirIterator::Subdirectories);
    while (it.hasNext()) {
        it.next();
        n += it.fileInfo().size();
    }
    return n;
}

RecordingInfo infoFromCheck(const QString& path, const QJsonObject& c) {
    RecordingInfo r;
    r.path = path;
    r.checked = true;
    r.ok = c.value(QStringLiteral("ok")).toBool();
    r.durationS = c.value(QStringLiteral("duration_s")).toDouble(-1);
    for (const QJsonValue& v : c.value(QStringLiteral("problems")).toArray()) {
        r.problems << v.toString();
    }
    r.summary = c.value(QStringLiteral("summary")).toString();
    r.dataset = c.value(QStringLiteral("dataset")).toString();
    r.imuFirstNs = c.value(QStringLiteral("imu_first_ns")).toInteger();
    r.imuLastNs = c.value(QStringLiteral("imu_last_ns")).toInteger();
    return r;
}

} // namespace

std::optional<QString> findEurocRoot(const QString& dir, QString* error) {
    const QDir d(dir);
    if (QFileInfo(d.filePath(QStringLiteral("mav0"))).isDir()) {
        return QDir::cleanPath(dir);
    }
    if (!d.exists()) {
        if (error) {
            *error = QStringLiteral("%1: no such folder").arg(dir);
        }
        return std::nullopt;
    }
    QStringList c;
    for (const QString& e : d.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name)) {
        if (QFileInfo(d.filePath(e + QStringLiteral("/mav0"))).isDir()) {
            c << e;
        }
    }
    if (c.isEmpty()) {
        if (error) {
            *error = QStringLiteral("%1: no mav0 directory (not a EuRoC dataset)").arg(dir);
        }
        return std::nullopt;
    }
    return QDir::cleanPath(d.filePath(c.last()));
}

std::optional<std::vector<ImuRow>> parseImuCsv(const QByteArray& text, QString* error) {
    std::vector<ImuRow> v;
    for (const auto& [n, l] : dataLines(text)) {
        const QList<QByteArray> fl = l.split(',');
        if (fl.size() < 7) {
            if (error) {
                *error = QStringLiteral("imu line %1: %2 fields, want 7").arg(n).arg(fl.size());
            }
            return std::nullopt;
        }
        bool ok = false;
        ImuRow r;
        r.ts = fl[0].trimmed().toLongLong(&ok);
        if (!ok) {
            if (error) {
                *error = QStringLiteral("imu line %1: bad timestamp \"%2\"").arg(n).arg(QString::fromLatin1(fl[0]));
            }
            return std::nullopt;
        }
        double x[6];
        for (int k = 0; k < 6; k++) {
            x[k] = fl[k + 1].trimmed().toDouble(&ok); // QByteArray::toDouble: C locale
            if (!ok) {
                if (error) {
                    *error = QStringLiteral("imu line %1: bad number \"%2\"").arg(n).arg(QString::fromLatin1(fl[k + 1]));
                }
                return std::nullopt;
            }
        }
        r.gyro = {x[0], x[1], x[2]};
        r.accel = {x[3], x[4], x[5]};
        v.push_back(r);
    }
    return v;
}

std::optional<std::vector<std::pair<qint64, QString>>> parseCamCsv(const QByteArray& text, QString* error) {
    std::vector<std::pair<qint64, QString>> v;
    for (const auto& [n, l] : dataLines(text)) {
        const qsizetype comma = l.indexOf(',');
        if (comma < 0) {
            if (error) {
                *error = QStringLiteral("cam line %1: want ts,filename").arg(n);
            }
            return std::nullopt;
        }
        bool ok = false;
        const qint64 ts = l.left(comma).trimmed().toLongLong(&ok);
        if (!ok) {
            if (error) {
                *error = QStringLiteral("cam line %1: bad timestamp \"%2\"").arg(n).arg(QString::fromLatin1(l.left(comma)));
            }
            return std::nullopt;
        }
        v.emplace_back(ts, QString::fromUtf8(l.mid(comma + 1).trimmed()));
    }
    return v;
}

std::optional<EurocDataset> loadEuroc(const QString& dir, QString* error) {
    const auto root = findEurocRoot(dir, error);
    if (!root) {
        return std::nullopt;
    }
    EurocDataset ds;
    ds.root = *root;
    QString err;
    const QByteArray imuText = readAll(*root + QStringLiteral("/mav0/imu0/data.csv"), &err);
    if (!err.isEmpty()) {
        if (error) {
            *error = err;
        }
        return std::nullopt;
    }
    auto imu = parseImuCsv(imuText, error);
    if (!imu) {
        return std::nullopt;
    }
    ds.imu = std::move(*imu);
    for (int i = 0;; i++) {
        const QString p = *root + QStringLiteral("/mav0/cam%1/data.csv").arg(i);
        if (!QFileInfo(p).isFile()) {
            break;
        }
        const QByteArray t = readAll(p, &err);
        if (!err.isEmpty()) {
            if (error) {
                *error = err;
            }
            return std::nullopt;
        }
        auto cam = parseCamCsv(t, error);
        if (!cam) {
            return std::nullopt;
        }
        ds.cams.push_back(std::move(*cam));
    }
    return ds;
}

EurocReport checkEuroc(const EurocDataset& ds, bool checkFiles) {
    EurocReport r;
    const auto w = [&r](const QString& s) { r.text += s + QLatin1Char('\n'); };
    w(QStringLiteral("dataset %1").arg(ds.root));
    QStringList problems;
    if (ds.imu.size() < 2) {
        problems << QStringLiteral("fewer than 2 IMU samples");
    } else {
        std::vector<qint64> ts;
        ts.reserve(ds.imu.size());
        for (const ImuRow& s : ds.imu) {
            ts.push_back(s.ts);
        }
        const Intervals iv = intervals(ts);
        const double span = double(ts.back() - ts.front()) / 1e9;
        r.imuSamples = qint64(ts.size());
        r.imuHz = 1.0 / iv.med;
        r.durationS = span;
        w(QStringLiteral("imu0: %1 samples over %2 s, %3 Hz (median dt %4 ms, max %5 ms), %6 gaps > 1.5x, %7 non-increasing")
              .arg(ts.size())
              .arg(f(span, 2), f(1.0 / iv.med, 1), f(iv.med * 1e3, 3), f(iv.max * 1e3, 2))
              .arg(iv.gaps)
              .arg(iv.back));
        if (iv.back > 0) {
            problems << QStringLiteral("imu0: %1 non-increasing timestamps").arg(iv.back);
        }
        if (iv.max > 0.02) {
            problems << QStringLiteral("imu0: a %1 ms gap (USB stall?)").arg(f(iv.max * 1e3, 1));
        }
        std::vector<double> g;
        g.reserve(ds.imu.size());
        for (const ImuRow& s : ds.imu) {
            g.push_back(std::sqrt(s.accel[0] * s.accel[0] + s.accel[1] * s.accel[1] + s.accel[2] * s.accel[2]));
        }
        const double gm = median(std::move(g));
        r.medianAccel = gm;
        w(QStringLiteral("imu0: median |accel| %1 m/s² (gravity 9.807)").arg(f(gm, 3)));
        if (std::fabs(gm - 9.807) > 0.5) {
            problems << QStringLiteral("imu0: median |accel| %1 m/s², scale wrong?").arg(f(gm, 2));
        }
    }
    const qint64 firstImu = ds.imu.empty() ? 0 : ds.imu.front().ts;
    const qint64 lastImu = ds.imu.empty() ? 0 : ds.imu.back().ts;
    r.imuFirstNs = firstImu;
    r.imuLastNs = lastImu;
    if (ds.cams.empty()) {
        problems << QStringLiteral("no camN/data.csv");
    }
    for (size_t i = 0; i < ds.cams.size(); i++) {
        const auto& c = ds.cams[i];
        r.camFrames.push_back(qint64(c.size()));
        std::vector<qint64> ts;
        ts.reserve(c.size());
        for (const auto& x : c) {
            ts.push_back(x.first);
        }
        if (ts.size() < 2) {
            problems << QStringLiteral("cam%1: fewer than 2 frames").arg(i);
            continue;
        }
        const Intervals iv = intervals(ts);
        const auto outside = std::count_if(ts.begin(), ts.end(), [&](qint64 t) { return t < firstImu || t > lastImu; });
        w(QStringLiteral("cam%1: %2 frames, %3 Hz (median dt %4 ms, max %5 ms), %6 gaps > 1.5x, %7 non-increasing, %8 "
                         "outside the IMU span")
              .arg(i)
              .arg(ts.size())
              .arg(f(1.0 / iv.med, 2), f(iv.med * 1e3, 2), f(iv.max * 1e3, 1))
              .arg(iv.gaps)
              .arg(iv.back)
              .arg(outside));
        if (iv.back > 0) {
            problems << QStringLiteral("cam%1: %2 non-increasing timestamps").arg(i).arg(iv.back);
        }
        if (outside > 2) {
            problems << QStringLiteral("cam%1: %2 frames outside the IMU time span (clock mismatch?)").arg(i).arg(outside);
        }
        if (checkFiles) {
            const QString dir = ds.root + QStringLiteral("/mav0/cam%1/data/").arg(i);
            const auto missing = std::count_if(c.begin(), c.end(), [&](const auto& x) {
                return !QFileInfo(dir + x.second).isFile();
            });
            if (missing > 0) {
                problems << QStringLiteral("cam%1: %2 image files missing (recording cut short?)").arg(i).arg(missing);
            }
        }
    }
    if (ds.cams.size() > 1) {
        QSet<qint64> base;
        for (const auto& x : ds.cams[0]) {
            base.insert(x.first);
        }
        for (size_t i = 1; i < ds.cams.size(); i++) {
            const auto& c = ds.cams[i];
            const auto common = std::count_if(c.begin(), c.end(), [&](const auto& x) { return base.contains(x.first); });
            w(QStringLiteral("cam%1: %2 of %3 timestamps shared with cam0 (synchronised stereo wants all)")
                  .arg(i)
                  .arg(common)
                  .arg(c.size()));
            if (size_t(common) * 10 < c.size() * 9) {
                problems << QStringLiteral("cam%1: only %2/%3 timestamps match cam0").arg(i).arg(common).arg(c.size());
            }
        }
    }
    for (const QString& p : problems) {
        w(QStringLiteral("PROBLEM: %1").arg(p));
    }
    if (problems.isEmpty()) {
        w(QStringLiteral("OK: usable for offline SLAM replay"));
    }
    r.problems = problems;
    return r;
}

bool writeEurocCsvs(const QString& root, const std::vector<ImuRow>& imu,
                    const std::vector<std::vector<std::pair<qint64, QString>>>& cams) {
    const auto put = [](const QString& p, const QByteArray& b) {
        QDir().mkpath(QFileInfo(p).absolutePath());
        QFile file(p);
        return file.open(QIODevice::WriteOnly | QIODevice::Truncate) && file.write(b) == b.size();
    };
    QByteArray s = "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],"
                   "a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\r\n";
    for (const ImuRow& r : imu) {
        s += QByteArray::number(r.ts);
        for (double x : {r.gyro[0], r.gyro[1], r.gyro[2], r.accel[0], r.accel[1], r.accel[2]}) {
            s += ',' + QByteArray::number(x, 'g', 17);
        }
        s += "\r\n";
    }
    if (!put(root + QStringLiteral("/mav0/imu0/data.csv"), s)) {
        return false;
    }
    for (size_t i = 0; i < cams.size(); i++) {
        QDir().mkpath(root + QStringLiteral("/mav0/cam%1/data").arg(i));
        QByteArray c = "#timestamp [ns],filename\r\n";
        for (const auto& [t, name] : cams[i]) {
            c += QByteArray::number(t) + ',' + name.toUtf8() + "\r\n";
        }
        if (!put(root + QStringLiteral("/mav0/cam%1/data.csv").arg(i), c)) {
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------- the recordings folder

RecordingInfo checkRecording(const QString& recording) {
    QJsonObject c;
    QString err;
    const auto ds = loadEuroc(recording, &err);
    if (!ds) {
        c.insert(QStringLiteral("ok"), false);
        c.insert(QStringLiteral("problems"), QJsonArray{err});
        c.insert(QStringLiteral("summary"), QStringLiteral("No usable dataset: %1").arg(err));
    } else {
        const EurocReport r = checkEuroc(*ds, true);
        c.insert(QStringLiteral("ok"), r.ok());
        c.insert(QStringLiteral("problems"), QJsonArray::fromStringList(r.problems));
        c.insert(QStringLiteral("summary"), r.ok() ? QStringLiteral("OK: usable")
                                                   : QStringLiteral("Problem: %1").arg(r.problems.first()));
        c.insert(QStringLiteral("text"), r.text);
        c.insert(QStringLiteral("dataset"), QFileInfo(ds->root).fileName());
        c.insert(QStringLiteral("imu_samples"), r.imuSamples);
        c.insert(QStringLiteral("imu_hz"), r.imuHz);
        c.insert(QStringLiteral("duration_s"), r.durationS);
        QJsonArray frames;
        for (qint64 n : r.camFrames) {
            frames.append(n);
        }
        c.insert(QStringLiteral("cam_frames"), frames);
        c.insert(QStringLiteral("median_accel"), r.medianAccel);
        c.insert(QStringLiteral("imu_first_ns"), r.imuFirstNs);
        c.insert(QStringLiteral("imu_last_ns"), r.imuLastNs);
    }
    c.insert(QStringLiteral("checked"), QDateTime::currentDateTime().toOffsetFromUtc(
                                                  QDateTime::currentDateTime().offsetFromUtc())
                                                  .toString(Qt::ISODate));
    QSaveFile out(recording + QLatin1Char('/') + QString::fromLatin1(kCheckFile));
    if (out.open(QIODevice::WriteOnly)) {
        out.write(QJsonDocument(c).toJson(QJsonDocument::Indented));
        out.commit();
    }
    RecordingInfo info = infoFromCheck(recording, c);
    info.name = QFileInfo(recording).fileName();
    return info;
}

std::vector<RecordingInfo> listRecordings(const QString& dir, bool checkMissing) {
    std::vector<RecordingInfo> out;
    const QDir d(dir);
    for (const QString& e : d.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name | QDir::Reversed)) {
        const QString p = d.filePath(e);
        if (!isRecordingFolder(p)) {
            continue;
        }
        RecordingInfo r;
        QFile cf(p + QLatin1Char('/') + QString::fromLatin1(kCheckFile));
        if (cf.open(QIODevice::ReadOnly)) {
            const QJsonDocument doc = QJsonDocument::fromJson(cf.readAll());
            if (doc.isObject()) {
                r = infoFromCheck(p, doc.object());
            }
        }
        if (!r.checked) {
            if (checkMissing) {
                r = checkRecording(p);
            } else {
                r.path = p;
                r.summary = QStringLiteral("Not checked yet");
            }
        }
        r.name = e;
        r.when = QDateTime::fromString(e, QStringLiteral("yyyyMMdd-HHmmss"));
        if (!r.when.isValid()) {
            r.when = QFileInfo(p).lastModified();
        }
        QFile sf(p + QLatin1Char('/') + QString::fromLatin1(kStepsFile));
        if (sf.open(QIODevice::ReadOnly)) {
            r.mode = QJsonDocument::fromJson(sf.readAll()).object().value(QStringLiteral("mode")).toString();
        }
        r.bytes = folderBytes(p);
        out.push_back(r);
    }
    std::stable_sort(out.begin(), out.end(), [](const RecordingInfo& a, const RecordingInfo& b) { return a.when > b.when; });
    return out;
}

bool deleteRecording(const QString& recordingsDir, const QString& recording, QString* error) {
    const QFileInfo fi(recording);
    const QString parent = QFileInfo(fi.absolutePath()).canonicalFilePath();
    const QString base = QFileInfo(recordingsDir).canonicalFilePath();
    const auto fail = [error](const QString& e) {
        if (error) {
            *error = e;
        }
        return false;
    };
    if (base.isEmpty() || parent != base || fi.isSymLink() || !fi.isDir()) {
        return fail(QStringLiteral("%1 is not a recording in %2").arg(recording, recordingsDir));
    }
    if (!isRecordingFolder(fi.absoluteFilePath())) {
        return fail(QStringLiteral("%1 does not look like a recording (no steps.json, no euroc_* folder)").arg(recording));
    }
    if (!QDir(fi.absoluteFilePath()).removeRecursively()) {
        return fail(QStringLiteral("Could not delete all of %1").arg(recording));
    }
    return true;
}

QString sizeWords(qint64 b) {
    if (b >= 1000LL * 1000 * 1000) {
        return QStringLiteral("%1 GB").arg(f(double(b) / 1e9, 1));
    }
    if (b >= 1000LL * 1000) {
        return QStringLiteral("%1 MB").arg(qRound64(double(b) / 1e6));
    }
    return QStringLiteral("%1 kB").arg(qMax<qint64>(b > 0 ? 1 : 0, qRound64(double(b) / 1e3)));
}

QString durationWords(double s) {
    if (s < 0) {
        return QStringLiteral("unknown length");
    }
    const qint64 t = qRound64(s);
    if (t < 60) {
        return QStringLiteral("%1 s").arg(t);
    }
    return QStringLiteral("%1 min %2 s").arg(t / 60).arg(t % 60, 2, 10, QLatin1Char('0'));
}

} // namespace pibridge::app
