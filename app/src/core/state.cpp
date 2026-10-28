// SPDX-License-Identifier: MIT
#include "state.h"

#include <QDateTime>

#include "paths.h"
#include "system.h"

namespace pibridge::app {

QString vrRuntimeWords(const AppState& s) {
    if (s.steamvrRunning && s.monadoRunning) {
        return QStringLiteral("SteamVR and Monado running");
    }
    if (s.steamvrRunning) {
        return QStringLiteral("SteamVR running");
    }
    if (s.monadoRunning) {
        return QStringLiteral("Monado running");
    }
    return QStringLiteral("VR not running");
}

AppState fakeState(bool active) {
    AppState s;
    s.steamvrRunning = active;
    auto& k = s.keepalive;
    k.running = true;
    k.state = active ? QStringLiteral("active") : QStringLiteral("idle");
    k.model = QStringLiteral("Crystal Super");
    k.headsetPresent = true;
    k.panelOn = active;
    k.rateHz = 90;
    k.follow = QStringLiteral("vrserver");
    k.followRunning = active;
    k.audioOnHeadset = active;
    k.pid = 4242;
    auto& e = s.eyed;
    e.running = true;
    e.state = active ? QStringLiteral("streaming") : QStringLiteral("idle");
    e.profile = QStringLiteral("user");
    e.eyesSeen = active ? 2 : 0;
    e.samplesPerS = active ? 120 : 0;
    e.follow = QStringLiteral("vrserver");
    e.followRunning = active;
    e.pid = 4243;
    // The devices go through the same link as the real ones, with a scripted source.
    FakeSteamVrSource vr;
    vr.polls = {fakeVrDevices()};
    SteamVrLink link(vr, [] { return qint64(0); });
    s.steamvr = link.tick(active);
    return s;
}

GazeSnapshot fakeGaze() {
    GazeSnapshot g;
    g.flags = kGazeLeft | kGazeRight | kGazeCombined;
    g.direction[2][0] = 0.18f;
    g.direction[2][1] = 0.08f;
    g.direction[2][2] = -0.98f;
    g.state = 2;
    g.sample = 1;
    g.tNs = g.writtenNs = monotonicNs(); // fresh, so the dot shows as live
    return g;
}

StateSource::StateSource(bool fake, QObject* parent, std::unique_ptr<SteamVrSource> vr)
    : QObject(parent), fake_(fake) {
    if (fake_) {
        state_ = fakeState(true);
        return;
    }
    timer_.setInterval(1000);
    connect(&timer_, &QTimer::timeout, this, &StateSource::refresh);
    refresh();
    timer_.start();
    vrWorker_ = std::make_unique<SteamVrWorker>(
        vr ? std::move(vr) : makeOpenVrSource(), [this] { return vrserver_.load(); },
        [this](const SteamVrStatus& st) {
            // worker thread -> GUI thread; dropped by Qt if this object is gone (the worker is joined before)
            QMetaObject::invokeMethod(
                this,
                [this, st] {
                    // a tick that started before vrserver went away must not bring old devices back
                    state_.steamvr = state_.steamvrRunning ? st : SteamVrStatus{};
                    emit changed();
                },
                Qt::QueuedConnection);
        });
}

StateSource::~StateSource() {
    vrWorker_.reset(); // join the thread (and drop the SteamVR connection) before the members go
}

void StateSource::setFakeState(const AppState& s) {
    state_ = s;
    emit changed();
}

void StateSource::refresh() {
    if (fake_) {
        emit changed();
        return;
    }
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    const QString dir = statusDir();
    state_.keepalive = readKeepalive(dir + QStringLiteral("/keepalived.json"), now);
    state_.eyed = readEyed(dir + QStringLiteral("/eyed.json"), now);
    state_.steamvrRunning = processRunning(QStringLiteral("vrserver"));
    state_.monadoRunning = processRunning(QStringLiteral("monado-service"));
    vrserver_ = state_.steamvrRunning;
    if (!state_.steamvrRunning) {
        state_.steamvr = {}; // shown at once; the worker drops the connection on its next tick
    }
    emit changed();
}

std::optional<GazeSnapshot> StateSource::gaze() const {
    if (fake_) {
        return state_.eyed.state == QLatin1String("streaming") ? std::optional(fakeGaze()) : std::nullopt;
    }
    return readGaze();
}

} // namespace pibridge::app
