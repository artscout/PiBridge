// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-slam/src/euroc.rs (MIT OR Apache-2.0).
// The check after a recording ("omarchyvr-slam check" in C++): EuRoC/ASL datasets as Monado's recorder writes them
// (mav0/imu0/data.csv, mav0/camN/data.csv + mav0/camN/data/<ts>.png): sample counts, rates, gaps, non-increasing
// timestamps, cam0/cam1 shared timestamps, camera frames outside the IMU span, median |accel|, missing images.
// And the list of past recordings in the recordings folder (one folder per recording, see docs/DEVELOPMENT.md).
#pragma once

#include <QDateTime>
#include <QString>
#include <QStringList>
#include <array>
#include <optional>
#include <utility>
#include <vector>

namespace pibridge::app {

struct ImuRow {
    qint64 ts = 0;               // ns
    std::array<double, 3> gyro{}; // rad/s
    std::array<double, 3> accel{}; // m/s^2
};

// The dataset root (the folder holding mav0): `dir` itself, or the last (by name) child with a mav0
// (<dir>/euroc_<datetime>/mav0). nullopt with `error` set when there is none.
std::optional<QString> findEurocRoot(const QString& dir, QString* error);
// imu0/data.csv text; nullopt with `error` ("imu line 3: 5 fields, want 7").
std::optional<std::vector<ImuRow>> parseImuCsv(const QByteArray& text, QString* error);
// camN/data.csv text: (timestamp, file name).
std::optional<std::vector<std::pair<qint64, QString>>> parseCamCsv(const QByteArray& text, QString* error);

struct EurocDataset {
    QString root;
    std::vector<ImuRow> imu;
    std::vector<std::vector<std::pair<qint64, QString>>> cams;
};
std::optional<EurocDataset> loadEuroc(const QString& dir, QString* error);

struct EurocReport {
    QString text;          // the lines omarchyvr-slam check prints
    QStringList problems;  // empty = usable
    qint64 imuSamples = 0;
    double imuHz = 0;
    double durationS = 0;  // the IMU span
    std::vector<qint64> camFrames;
    double medianAccel = 0;
    qint64 imuFirstNs = 0, imuLastNs = 0;
    bool ok() const { return problems.isEmpty(); }
};
// `checkFiles`: also look for every camN image file.
EurocReport checkEuroc(const EurocDataset& ds, bool checkFiles);

// Writes a dataset's CSV files (tests, the fake mode's made-up recordings), with Monado's headers and CRLF ends.
bool writeEurocCsvs(const QString& root, const std::vector<ImuRow>& imu,
                    const std::vector<std::vector<std::pair<qint64, QString>>>& cams);

// ---------------------------------------------------------------- the recordings folder

inline constexpr char kCheckFile[] = "check.json";
inline constexpr char kStepsFile[] = "steps.json";

struct RecordingInfo {
    QString path;       // <recordings>/<yyyyMMdd-HHmmss>
    QString name;       // the folder name
    QDateTime when;     // from the folder name, else its modification time
    qint64 bytes = 0;
    double durationS = -1;  // -1 unknown
    QString mode;       // from steps.json ("guided" | "free"), "" unknown
    bool checked = false;
    bool ok = false;
    QString summary;    // "OK: usable" or the first problem, in plain words
    QStringList problems;
    QString dataset;    // the euroc_<datetime> folder name ("" when none)
    qint64 imuFirstNs = 0, imuLastNs = 0;
};

// Loads the dataset under `recording`, checks it and writes <recording>/check.json; the result as an info.
RecordingInfo checkRecording(const QString& recording);
// Every recording folder (one with steps.json or a euroc_* child), newest first. A folder without check.json is
// checked now (and gets one), unless `checkMissing` is false.
std::vector<RecordingInfo> listRecordings(const QString& dir, bool checkMissing = true);
// Deletes a recording folder; refuses anything that is not a direct child of `recordingsDir` holding steps.json or
// a euroc_* folder.
bool deleteRecording(const QString& recordingsDir, const QString& recording, QString* error);
// "1.4 GB", "820 MB", "12 kB".
QString sizeWords(qint64 bytes);
// "2 min 05 s", "48 s".
QString durationWords(double seconds);

} // namespace pibridge::app
