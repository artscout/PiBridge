// SPDX-License-Identifier: MIT
#include "steamvr_ready.h"

#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <chrono>

#include "apps_steam.h"
#include "state.h"
#include "system.h"

namespace pibridge::app {

namespace {

qint64 steadyMs() {
    return static_cast<qint64>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
}

const VrDevice* hmd(const SteamVrStatus& s) {
    for (const VrDevice& d : s.devices) {
        if (d.index == 0 && d.cls == VrClass::Hmd) {
            return &d;
        }
    }
    return nullptr;
}

// vrmonitor states of a start still in progress; the others (Ready, Standby, and the Shutdown / Off of a stale log
// from an earlier run) do not block, since steps 1-3 already show what runs now.
bool monitorBlocks(const QString& st) {
    return st == QLatin1String("Startup") || st == QLatin1String("Connecting") || st == QLatin1String("NotReady") ||
           st == QLatin1String("Restart");
}

} // namespace

Readiness evaluateReady(const ReadyInputs& in) {
    if (!in.installed || in.link.state == VrLinkState::NotInstalled) {
        return {ReadyStep::NotInstalled, QStringLiteral("SteamVR is not installed"), in.link.error};
    }
    if (!in.vrserverRunning) {
        return {ReadyStep::NoServer, QStringLiteral("SteamVR is not running"), {}};
    }
    if (in.link.state != VrLinkState::Connected) {
        return {ReadyStep::NoClients, QStringLiteral("Waiting for SteamVR to take clients..."), in.link.error};
    }
    const VrDevice* h = hmd(in.link);
    if (!h) {
        return {ReadyStep::NoHeadset, QStringLiteral("Waiting for SteamVR to see the headset..."), {}};
    }
    if (!h->poseValid || h->trackingResult != vrtrack::RunningOk) {
        return {ReadyStep::HeadsetNotTracked, QStringLiteral("Waiting for the headset to be tracked..."),
                vrTrackingWords(h->trackingResult)};
    }
    const bool compositor = in.link.compositorPid > 0 || (in.link.compositorPid < 0 && in.compositorProcess);
    if (!compositor) {
        return {ReadyStep::NoCompositor, QStringLiteral("Waiting for the SteamVR compositor..."),
                in.link.compositorPid < 0 ? QStringLiteral("no vrcompositor process") : QStringLiteral("not running")};
    }
    if (monitorBlocks(in.monitorState)) {
        return {ReadyStep::MonitorNotReady, QStringLiteral("Waiting for SteamVR to report Ready..."),
                QStringLiteral("SteamVR state: %1").arg(in.monitorState)};
    }
    return {ReadyStep::Settling, QStringLiteral("SteamVR is almost ready..."), {}};
}

QString readyFailureReason(const Readiness& r) {
    const auto with = [&r](const QString& s) { return r.detail.isEmpty() ? s : QStringLiteral("%1 (%2)").arg(s, r.detail); };
    switch (r.step) {
    case ReadyStep::NotInstalled: return QStringLiteral("SteamVR is not installed");
    case ReadyStep::NoServer: return QStringLiteral("SteamVR did not start");
    case ReadyStep::NoClients: return with(QStringLiteral("SteamVR started but does not answer"));
    case ReadyStep::NoHeadset: return QStringLiteral("SteamVR does not see the headset");
    case ReadyStep::HeadsetNotTracked: return with(QStringLiteral("SteamVR sees the headset but does not track it"));
    case ReadyStep::NoCompositor: return QStringLiteral("the SteamVR compositor did not start");
    case ReadyStep::MonitorNotReady: return with(QStringLiteral("SteamVR did not report Ready"));
    case ReadyStep::Settling: return QStringLiteral("SteamVR did not stay ready");
    case ReadyStep::Ready: break;
    }
    return {};
}

QString lastSystemState(QStringView text) {
    // "... Transition from 'SteamVRSystemState_Startup' to 'SteamVRSystemState_Connecting'."
    static const QString marker = QStringLiteral("to 'SteamVRSystemState_");
    const qsizetype at = text.lastIndexOf(marker);
    if (at < 0) {
        return {};
    }
    const qsizetype start = at + marker.size();
    const qsizetype end = text.indexOf(QLatin1Char('\''), start);
    if (end < 0 || end - start > 40) {
        return {};
    }
    return text.mid(start, end - start).toString();
}

QString lastSystemStateOfFile(const QString& path) {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        return {};
    }
    const qint64 tail = 64 * 1024;
    if (f.size() > tail) {
        f.seek(f.size() - tail);
    }
    return lastSystemState(QString::fromUtf8(f.read(tail)));
}

QString vrmonitorLogPath() {
    const QStringList roots = steamRoots();
    return roots.isEmpty() ? QString() : roots.first() + QStringLiteral("/logs/vrmonitor.txt");
}

ReadyInputs readyInputsFrom(const AppState& s, bool fake) {
    ReadyInputs in;
    in.vrserverRunning = s.steamvrRunning;
    in.link = s.steamvrRunning ? s.steamvr : SteamVrStatus{};
    if (fake) {
        in.installed = true;
        in.compositorProcess = s.steamvrRunning;
        in.monitorState = s.steamvrRunning ? QStringLiteral("Ready") : QString();
        return in;
    }
    in.installed = !openVrRuntimeDir(openVrPathsFile()).isEmpty();
    if (s.steamvrRunning) {
        if (in.link.compositorPid < 0 && in.link.state == VrLinkState::Connected) {
            in.compositorProcess = processRunning(QStringLiteral("vrcompositor"));
        }
        if (const QString log = vrmonitorLogPath(); !log.isEmpty()) {
            in.monitorState = lastSystemStateOfFile(log);
        }
    }
    return in;
}

// ---------------------------------------------------------------- tracker

ReadyTracker::ReadyTracker(Clock clock, qint64 settleMs) : clock_(std::move(clock)), settleMs_(settleMs) {
    current_ = evaluateReady(last_);
}

Readiness ReadyTracker::update(const ReadyInputs& in) {
    last_ = in;
    return recheck();
}

Readiness ReadyTracker::recheck() {
    Readiness r = evaluateReady(last_);
    const qint64 now = clock_();
    if (r.step != ReadyStep::Settling) {
        allHeldSince_.reset();
    } else {
        if (!allHeldSince_) {
            allHeldSince_ = now;
        }
        if (now - *allHeldSince_ >= settleMs_) {
            r = {ReadyStep::Ready, QStringLiteral("SteamVR is ready"), {}};
        }
    }
    current_ = r;
    return r;
}

// ---------------------------------------------------------------- Qt side

SteamVrReady::SteamVrReady(Clock clock, qint64 settleMs, QObject* parent)
    : QObject(parent), tracker_(clock ? std::move(clock) : Clock(&steadyMs), settleMs) {
    timer_.setInterval(250);
    connect(&timer_, &QTimer::timeout, this, &SteamVrReady::poll);
}

void SteamVrReady::setInputs(const ReadyInputs& in) {
    everReceived_ = true;
    const ReadyStep before = tracker_.current().step;
    tracker_.update(in);
    settleWaiters();
    if (tracker_.current().step != before) {
        emit changed();
    }
}

void SteamVrReady::poll() {
    const ReadyStep before = tracker_.current().step;
    tracker_.recheck();
    settleWaiters();
    if (tracker_.current().step != before) {
        emit changed();
    }
}

void SteamVrReady::waitReady(int timeoutMs, Callback cb) {
    if (!cb) {
        return;
    }
    tracker_.recheck();
    const Readiness& r = tracker_.current();
    if (r.ready()) {
        cb(true, {});
        return;
    }
    if (everReceived_ && r.step == ReadyStep::NotInstalled) {
        cb(false, readyFailureReason(r));
        return;
    }
    waiters_.push_back({tracker_.now() + timeoutMs, std::move(cb)});
    if (QCoreApplication::instance() && !timer_.isActive()) {
        timer_.start();
    }
}

void SteamVrReady::settleWaiters() {
    if (waiters_.empty()) {
        timer_.stop();
        return;
    }
    const Readiness r = tracker_.current();
    const qint64 now = tracker_.now();
    // take the ones that end now first: a callback may start a new wait
    std::vector<Waiter> done;
    std::vector<Waiter> keep;
    for (Waiter& w : waiters_) {
        if (r.ready() || r.step == ReadyStep::NotInstalled || now >= w.deadline) {
            done.push_back(std::move(w));
        } else {
            keep.push_back(std::move(w));
        }
    }
    waiters_ = std::move(keep);
    if (waiters_.empty()) {
        timer_.stop();
    }
    for (Waiter& w : done) {
        if (r.ready()) {
            w.cb(true, {});
        } else {
            w.cb(false, readyFailureReason(r));
        }
    }
}

void SteamVrReady::cancelAll(const QString& reason) {
    std::vector<Waiter> done = std::move(waiters_);
    waiters_.clear();
    timer_.stop();
    for (Waiter& w : done) {
        w.cb(false, reason);
    }
}

} // namespace pibridge::app
