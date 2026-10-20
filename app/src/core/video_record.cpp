// SPDX-License-Identifier: MIT
#include "video_record.h"

#include "pibridge/record/cli.h"
#include "pibridge/record/run.h"
#include "pibridge/record/shm.h"

#include <QFileInfo>

namespace pibridge::app {

namespace rec = pibridge::record;

namespace {
QString q(const std::string& s) { return QString::fromStdString(s); }

QString length(const rec::Status& st) {
    const double secs = st.seconds();
    const auto s = static_cast<qint64>(secs);
    return QStringLiteral("%1:%2").arg(s / 60).arg(s % 60, 2, 10, QLatin1Char('0'));
}
} // namespace

RecordCardModel recordCardModel(const RecordInputs& in) {
    RecordCardModel m;
    const rec::Status& st = in.status;
    if (!st.file.empty()) {
        m.lastFile = q(st.file);
        m.lastFolder = QFileInfo(m.lastFile).absolutePath();
    }
    if (in.recorderRunning) {
        m.canStop = true;
        if (st.state == "recording") {
            m.chip = QStringLiteral("REC");
            m.chipKind = ChipKind::Error; // the red REC
            m.headline = QStringLiteral("Recording %1").arg(length(st));
            m.detail = QStringLiteral("%1 eye%2, %3x%4 at %5 fps, %6 quality, %7%8%9.")
                           .arg(st.eyes == "both" ? QStringLiteral("Both") : QStringLiteral("One"),
                                st.eyes == "both" ? QStringLiteral("s") : QString())
                           .arg(st.width)
                           .arg(st.height)
                           .arg(st.fps)
                           .arg(q(st.quality), q(st.encoder),
                                st.audio.empty() ? QStringLiteral(", no sound") : QStringLiteral(", with sound"),
                                st.source == "mirror" ? QStringLiteral(", from SteamVR's mirror") : QString());
            const std::uint64_t dropped = st.dropped_tap + st.dropped_ring;
            if (dropped > 0) {
                m.detail += QStringLiteral(" %1 frames skipped by the recorder (VR itself never waits for it).").arg(dropped);
            }
        } else if (st.state == "finalizing") {
            m.chip = QStringLiteral("SAVING");
            m.chipKind = ChipKind::Warn;
            m.headline = QStringLiteral("Finishing the file");
            m.detail = QStringLiteral("Writing the end of %1.").arg(QFileInfo(m.lastFile).fileName());
            m.canStop = false;
        } else {
            m.chip = QStringLiteral("STARTING");
            m.chipKind = ChipKind::Warn;
            m.headline = QStringLiteral("Starting the recorder");
            m.detail = st.source == "mirror" ? QStringLiteral("Waiting for SteamVR's mirror image.")
                                             : QStringLiteral("Waiting for Monado's recorder tap.");
        }
        return m;
    }
    // the source follows the session (pibridge-record --source auto does the same): SteamVR's mirror while SteamVR
    // runs (also SteamVR + Monado, whose image SteamVR's compositor draws), else Monado's own tap
    if (in.steamvrRunning) {
        m.chip = QStringLiteral("READY");
        m.chipKind = ChipKind::Info;
        m.headline = QStringLiteral("Ready to record (SteamVR)");
        m.detail = QStringLiteral("Records SteamVR's mirror image (what the headset shows, before the lens distortion) "
                                  "into ~/Videos/pimax. VR keeps running if the recorder fails.");
        m.canStart = true;
    } else if (in.directRunning || in.monadoRunning) {
        m.chip = QStringLiteral("READY");
        m.chipKind = ChipKind::Info;
        m.headline = QStringLiteral("Ready to record (Monado)");
        m.detail = QStringLiteral("Records what the headset shows (before the lens distortion) into ~/Videos/pimax. "
                                  "VR keeps running if the recorder fails.");
        m.canStart = true;
    } else {
        m.chip = QStringLiteral("OFF");
        m.chipKind = ChipKind::Off;
        m.headline = QStringLiteral("No VR session");
        m.detail = QStringLiteral("Start SteamVR, or \"Monado directly\" on the Development page, then press Record.");
    }
    // the last result, in plain words
    if (st.state == "error" && !st.note.empty()) {
        m.detail = QStringLiteral("Last try: %1").arg(q(st.note));
        m.error = true;
    } else if (st.state == "idle" && !m.lastFile.isEmpty()) {
        m.detail += QStringLiteral(" Last: %1 (%2)%3")
                        .arg(QFileInfo(m.lastFile).fileName(), length(st),
                             st.note.empty() ? QString() : QStringLiteral(", ") + q(st.note));
    }
    return m;
}

RecordInputs readRecordInputs(bool directRunning, bool steamvrRunning) {
    RecordInputs in;
    const rec::Roots r = rec::Roots::fromEnv();
    in.status = rec::Status::load(rec::statusPath(r.session)).value_or(rec::Status{});
    in.recorderRunning = rec::runningPid(r).has_value();
    in.directRunning = directRunning;
    in.monadoRunning = rec::monadoRunning(r);
    in.steamvrRunning = steamvrRunning || rec::steamvrRunning(r);
    return in;
}

rec::Settings loadRecordSettings(QString* error) {
    const rec::Roots r = rec::Roots::fromEnv();
    auto s = rec::Settings::load(rec::settingsReadPath(r.config, r.legacyConfig));
    if (!s) {
        if (error) {
            *error = q(s.error());
        }
        return rec::Settings{};
    }
    return *s;
}

QString saveRecordSettings(const rec::Settings& s) { return q(s.save(rec::settingsPath(rec::Roots::fromEnv().config))); }

QString recordSettingsPath() { return q(rec::settingsPath(rec::Roots::fromEnv().config).string()); }

QString recordShmPath() { return q(rec::defaultShmPath(rec::Roots::fromEnv().runtime).string()); }

void startRecording(Runner& runner, const QString& tool, Done done) {
    runner.run(tool, {QStringLiteral("start")}, std::move(done));
}

void stopRecording(Runner& runner, const QString& tool, Done done) {
    runner.run(tool, {QStringLiteral("stop")}, std::move(done));
}

} // namespace pibridge::app
