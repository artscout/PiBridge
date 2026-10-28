// SPDX-License-Identifier: MIT
#include "overlay_control.h"

#include "conf.h"
#include "vr_backend.h"
#include "pibridge_status.h"

#include <QByteArray>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QStandardPaths>

namespace pibridge::app {

OverlayStatus overlayStatusWords(bool wanted, bool processRunning, bool steamvrRunning, const QString& lastError) {
    OverlayStatus s;
    if (!wanted) {
        s.run = OverlayRun::Off;
        s.words = QStringLiteral("Off: no PiBridge buttons in the SteamVR dashboard.");
        return s;
    }
    if (!processRunning) {
        s.run = OverlayRun::Failed;
        s.error = true;
        s.words = lastError.isEmpty() ? QStringLiteral("Not running.") : lastError;
        return s;
    }
    if (!steamvrRunning) {
        s.run = OverlayRun::WaitingForSteamVr;
        s.words = QStringLiteral("Waiting for SteamVR. The buttons appear in the dashboard when SteamVR starts.");
        return s;
    }
    s.run = OverlayRun::Running;
    s.words = QStringLiteral("Running: Room Setup, Quick Room Setup and Record Video are buttons in the SteamVR dashboard "
                             "bar. Point at one with the controller to set up the room or start/stop a recording.");
    return s;
}

ChipText overlayChip(const OverlayStatus& s) {
    switch (s.run) {
    case OverlayRun::Running:
        return {QStringLiteral("RUNNING"), ChipKind::Info};
    case OverlayRun::WaitingForSteamVr:
        return {QStringLiteral("WAITING"), ChipKind::Warn};
    case OverlayRun::Failed:
        return {QStringLiteral("NOT RUNNING"), ChipKind::Error};
    case OverlayRun::Off:
        break;
    }
    return {QStringLiteral("OFF"), ChipKind::Off};
}

QString findOverlayProgram(const QString& appDir, const std::function<QString(const QString&)>& which) {
    for (const QString& p : {appDir + QStringLiteral("/pibridge-overlay"),
                             appDir + QStringLiteral("/../tools/overlay/pibridge-overlay")}) {
        const QFileInfo f(p);
        if (f.isFile() && f.isExecutable()) {
            return QDir::cleanPath(f.absoluteFilePath());
        }
    }
    return which ? which(QStringLiteral("pibridge-overlay")) : QString();
}

std::optional<OverlayRequest> parseOverlayRequest(const QByteArray& text) {
    const QString line = QString::fromUtf8(text).split(QLatin1Char('\n')).value(0).trimmed();
    if (line == QLatin1String("quick-room-setup")) {
        return OverlayRequest{OverlayAction::RoomSetupQuick};
    }
    if (line == QLatin1String("room-setup")) {
        return OverlayRequest{OverlayAction::RoomSetupFull};
    }
    if (line == QLatin1String("record-start")) {
        return OverlayRequest{OverlayAction::RecordStart};
    }
    if (line == QLatin1String("record-stop")) {
        return OverlayRequest{OverlayAction::RecordStop};
    }
    // "record-set <key> <value>" (key = view | fps | max_height), e.g. "record-set view both"
    if (line.startsWith(QLatin1String("record-set "))) {
        const QStringList f = line.split(QLatin1Char(' '), Qt::SkipEmptyParts);
        if (f.size() == 3 && (f[1] == QLatin1String("view") || f[1] == QLatin1String("fps") ||
                              f[1] == QLatin1String("max_height"))) {
            return OverlayRequest{OverlayAction::RecordSet, f[1], f[2]};
        }
    }
    return std::nullopt;
}

OverlayControl::OverlayControl(Conf& conf, bool fake, QObject* parent) : QObject(parent), conf_(conf), fake_(fake) {
    program_ = fake_ ? QStringLiteral("/fake/bin/pibridge-overlay")
                     : findOverlayProgram(QCoreApplication::applicationDirPath(),
                                          [](const QString& n) { return QStandardPaths::findExecutable(n); });
    requestPath_ = QString::fromStdString((status::runtime_dir() / "overlay-request").string());
    QFile::remove(requestPath_); // drop any stale request from a previous run, so nothing launches at startup
    requestTimer_.setInterval(400);
    connect(&requestTimer_, &QTimer::timeout, this, &OverlayControl::pollRequest);
    if (QCoreApplication::instance() != nullptr) {
        requestTimer_.start(); // needs the app's event loop (tests construct us without one)
    }
    blocked_ = vrBackend(conf_) == VrBackend::Monado;
    follow();
}

void OverlayControl::setBlocked(bool blocked) {
    if (blocked == blocked_) {
        return;
    }
    blocked_ = blocked;
    if (blocked_) {
        stop();
    } else {
        follow();
    }
    emit changed();
}

OverlayControl::~OverlayControl() { stop(); }

void OverlayControl::pollRequest() {
    QFile f(requestPath_);
    if (!f.exists()) {
        return;
    }
    QByteArray text;
    if (f.open(QIODevice::ReadOnly)) {
        text = f.readAll();
        f.close();
    }
    QFile::remove(requestPath_); // consume it once
    if (const std::optional<OverlayRequest> a = parseOverlayRequest(text)) {
        switch (a->action) {
        case OverlayAction::RoomSetupFull:
            emit roomSetupRequested(false);
            break;
        case OverlayAction::RoomSetupQuick:
            emit roomSetupRequested(true);
            break;
        case OverlayAction::RecordStart:
            emit recordRequested(true);
            break;
        case OverlayAction::RecordStop:
            emit recordRequested(false);
            break;
        case OverlayAction::RecordSet:
            emit recordSettingChanged(a->key, a->value);
            break;
        }
    }
}

// Default ON: the overlay process runs with the app (it self-waits for SteamVR and reconnects), so the in-game
// recenter ring and the dashboard launcher are there without the user turning anything on. It is invisible until the
// SteamVR dashboard is opened or the recenter button is held, so running it always costs nothing on screen.
bool OverlayControl::wanted() const { return conf_.flag(kDashboardKey, true); }

bool OverlayControl::running() const {
    return fake_ ? fakeRunning_ : (proc_ && proc_->state() != QProcess::NotRunning);
}

OverlayStatus OverlayControl::status(bool steamvrRunning) const {
    if (blocked_) {
        OverlayStatus s;
        s.words = QStringLiteral("SteamVR backend only: on the Monado backend, PiBridge's own dashboard in the headset "
                                 "(the menu button) has the same buttons.");
        return s;
    }
    return overlayStatusWords(wanted(), running(), steamvrRunning, lastError_);
}

bool OverlayControl::setWanted(bool on) {
    const bool ok = conf_.set(kDashboardKey, on ? "on" : "off");
    if (on) {
        start();
    } else {
        stop();
    }
    emit changed();
    return ok;
}

void OverlayControl::follow() {
    if (wanted()) {
        start();
    }
}

void OverlayControl::start() {
    if (running() || blocked_) {
        return;
    }
    lastError_.clear();
    if (program_.isEmpty()) {
        lastError_ = QStringLiteral("pibridge-overlay was not found. Reinstall PiBridge.");
        emit changed();
        return;
    }
    if (fake_) {
        fakeRunning_ = true;
        emit changed();
        return;
    }
    proc_ = std::make_unique<QProcess>(this);
    proc_->setProgram(program_);
    proc_->setProcessChannelMode(QProcess::ForwardedChannels); // its own words go to the app's log, nothing read back
    connect(proc_.get(), &QProcess::started, this, [this] { emit changed(); });
    connect(proc_.get(), &QProcess::errorOccurred, this, [this](QProcess::ProcessError e) {
        if (e == QProcess::FailedToStart) {
            lastError_ = QStringLiteral("pibridge-overlay could not start.");
            emit changed();
        }
    });
    connect(proc_.get(), &QProcess::finished, this, [this](int code, QProcess::ExitStatus st) {
        // The overlay exits when SteamVR goes down (it never calls VR_ShutdownInternal; it just exits). While it is
        // still wanted and we did not ask it to stop, restart it shortly: it then waits for SteamVR to come back.
        if (!stopping_ && wanted()) {
            lastError_ = st == QProcess::CrashExit ? QStringLiteral("pibridge-overlay stopped; restarting.")
                                                   : QStringLiteral("pibridge-overlay exited (code %1); restarting.").arg(code);
            QTimer::singleShot(3000, this, [this] {
                if (wanted() && !running()) {
                    start();
                }
            });
        }
        emit changed();
    });
    proc_->start(QIODevice::ReadOnly);
}

void OverlayControl::stop() {
    if (fake_) {
        fakeRunning_ = false;
        emit changed();
        return;
    }
    stopping_ = true; // so the finished handler does not restart it
    if (proc_ && proc_->state() != QProcess::NotRunning) {
        proc_->terminate(); // SIGTERM: the overlay returns from its loop and exits (it never calls VR_ShutdownInternal)
        if (!proc_->waitForFinished(2000)) {
            proc_->kill();
            proc_->waitForFinished(500);
        }
    }
    proc_.reset();
    stopping_ = false;
    emit changed();
}

} // namespace pibridge::app
