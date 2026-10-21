// SPDX-License-Identifier: MIT
#include "overlay_control.h"

#include "conf.h"
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
        s.words = QStringLiteral("Off: no PiBridge tab in the SteamVR dashboard.");
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
        s.words = QStringLiteral("Waiting for SteamVR. The tab appears in the dashboard when SteamVR starts.");
        return s;
    }
    s.run = OverlayRun::Running;
    s.words = QStringLiteral("Running: a PiBridge tab is in the SteamVR dashboard. Point at it with the controller to "
                             "set up the room or start/stop a recording.");
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

std::optional<OverlayAction> parseOverlayRequest(const QByteArray& text) {
    const QString line = QString::fromUtf8(text).split(QLatin1Char('\n')).value(0).trimmed();
    if (line == QLatin1String("quick-room-setup")) {
        return OverlayAction::RoomSetupQuick;
    }
    if (line == QLatin1String("room-setup")) {
        return OverlayAction::RoomSetupFull;
    }
    if (line == QLatin1String("record-start")) {
        return OverlayAction::RecordStart;
    }
    if (line == QLatin1String("record-stop")) {
        return OverlayAction::RecordStop;
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
    follow();
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
    if (const std::optional<OverlayAction> a = parseOverlayRequest(text)) {
        switch (*a) {
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
        }
    }
}

bool OverlayControl::wanted() const { return conf_.flag(kDashboardKey, false); }

bool OverlayControl::running() const {
    return fake_ ? fakeRunning_ : (proc_ && proc_->state() != QProcess::NotRunning);
}

OverlayStatus OverlayControl::status(bool steamvrRunning) const {
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
    if (running()) {
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
