// SPDX-License-Identifier: MIT
#include "service.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>

#include <cstdio>
#include <ctime>
#include <unistd.h>

namespace pibridge::desktopsvc {

QString tokenPath() {
    QString state = qEnvironmentVariable("XDG_STATE_HOME");
    if (state.isEmpty()) {
        state = QDir::homePath() + QStringLiteral("/.local/state");
    }
    return state + QStringLiteral("/pibridge/desktop-token");
}

QString readToken() {
    QFile f(tokenPath());
    return f.open(QIODevice::ReadOnly) ? QString::fromUtf8(f.readAll()).trimmed() : QString();
}

bool writeToken(const QString& token) {
    if (token.isEmpty()) {
        return false;
    }
    QDir().mkpath(QFileInfo(tokenPath()).absolutePath());
    QSaveFile f(tokenPath());
    if (!f.open(QIODevice::WriteOnly)) {
        return false;
    }
    f.setPermissions(QFileDevice::ReadOwner | QFileDevice::WriteOwner);
    f.write(token.toUtf8() + '\n');
    return f.commit();
}

Service::Service(desktop::Link& link, const RunOptions& opt) : link_(link), opt_(opt) {
    timer_.setInterval(8); // the pointer at ~120 Hz
    connect(&timer_, &QTimer::timeout, this, &Service::tick);
}

Service::~Service() { stopCapture("the service ends", desktop::State::Off); }

void Service::log(const std::string& s) {
    const std::time_t t = std::time(nullptr);
    char ts[16];
    std::strftime(ts, sizeof ts, "%H:%M:%S", std::localtime(&t));
    std::fprintf(stderr, "%s %s\n", ts, s.c_str());
}

void Service::start() {
    auto* h = link_.header();
    h->servicePid.store(static_cast<std::uint32_t>(::getpid()), std::memory_order_relaxed);
    link_.setState(desktop::State::Off, "waiting for the desktop to be opened in the headset");
    startedNs_ = lastClientNs_ = desktop::monoNs();
    lastBeat_ = h->clientBeatNs.load(); // a beat left from before is no one looking
    timer_.start();
    log("pibridge-desktop: ready");
}

void Service::onFrame(void* user, unsigned index, const std::uint8_t* data, std::uint32_t w, std::uint32_t h,
                      std::uint32_t stride, int format) {
    auto* s = static_cast<Service*>(user);
    if (index >= desktop::kMaxMonitors || (s->want_.load(std::memory_order_relaxed) & (1U << index)) == 0) {
        return; // nobody shows it: no copy
    }
    const std::uint64_t now = desktop::monoNs();
    const std::uint64_t gap = 1'000'000'000ULL / static_cast<std::uint64_t>(std::max(1, s->opt_.fps));
    if (now - s->lastFrameNs_[index].load(std::memory_order_relaxed) < gap * 9 / 10) {
        return;
    }
    s->lastFrameNs_[index].store(now, std::memory_order_relaxed);
    s->link_.writeFrame(index, data, w, h, stride, static_cast<desktop::Source>(format));
}

void Service::onError(void* user, unsigned index, const char* words) {
    auto* s = static_cast<Service*>(user);
    s->streamFailed_.store(true);
    s->log("pibridge-desktop: monitor " + std::to_string(index + 1) + ": " + (words != nullptr ? words : "stopped"));
}

void Service::startCapture() {
    const QString token = readToken();
    if (token.isEmpty()) {
        link_.setState(desktop::State::NeedsPermission,
                       "not allowed yet: on the desktop, PiBridge's Dashboard, Desktop in VR, Allow");
        retryNs_ = desktop::monoNs() + 3'000'000'000ULL; // the token may appear in a moment
        return;
    }
    link_.setState(desktop::State::Starting, "connecting to the desktop");
    opening_ = true;
    PortalResult r = portal_.open(token, 8000);
    opening_ = false;
    if (!r.restoreToken.isEmpty()) {
        writeToken(r.restoreToken); // a token is good once: the next start needs this new one
    }
    if (!r.ok) {
        const bool perm = r.denied;
        link_.setState(perm ? desktop::State::NeedsPermission : desktop::State::Failed,
                       (perm ? "not allowed: on the desktop, PiBridge's Dashboard, Desktop in VR, Allow. "
                             : "the desktop could not be shared: ") +
                           r.error.toStdString());
        log("pibridge-desktop: " + r.error.toStdString());
        retryNs_ = desktop::monoNs() + 10'000'000'000ULL;
        return;
    }
    streams_ = r.streams;
    std::vector<desktop::MonitorInfo> mons;
    std::vector<std::uint32_t> nodes;
    for (std::size_t i = 0; i < streams_.size() && i < desktop::kMaxMonitors; i++) {
        desktop::MonitorInfo m;
        m.index = static_cast<std::uint32_t>(i);
        m.x = streams_[i].x;
        m.y = streams_[i].y;
        m.logicalW = static_cast<std::uint32_t>(std::max(0, streams_[i].w));
        m.logicalH = static_cast<std::uint32_t>(std::max(0, streams_[i].h));
        m.name = "Monitor " + std::to_string(i + 1);
        mons.push_back(m);
        nodes.push_back(streams_[i].node);
    }
    layout_ = mons;
    link_.setLayout(mons);
    streamFailed_.store(false);
    for (auto& t : lastFrameNs_) {
        t.store(0);
    }
    char err[256] = {0};
    pw_ = pibridge_pw_start(r.pipewireFd, nodes.data(), static_cast<unsigned>(nodes.size()), &Service::onFrame,
                            &Service::onError, this, err, sizeof err);
    if (pw_ == nullptr) {
        portal_.close();
        link_.setState(desktop::State::Failed, std::string("the desktop's video could not be opened: ") + err);
        retryNs_ = desktop::monoNs() + 10'000'000'000ULL;
        return;
    }
    input_.reset();
    link_.setState(desktop::State::Running, std::to_string(mons.size()) + (mons.size() == 1 ? " monitor" : " monitors"));
    log("pibridge-desktop: sharing " + std::to_string(mons.size()) + " monitor(s)");
}

void Service::stopCapture(const std::string& why, desktop::State st) {
    if (pw_ != nullptr) {
        pibridge_pw_stop(pw_);
        pw_ = nullptr;
    }
    if (portal_.isOpen()) {
        // let go of held buttons before the session goes
        for (const Event& e : input_.update(desktop::Pointer{}, 0, layout_)) {
            if (e.kind == Event::Kind::Button) {
                portal_.pointerButton(e.button, false);
            }
        }
        portal_.close();
        log("pibridge-desktop: stopped (" + why + ")");
    }
    if (link_.isOpen()) {
        link_.setState(st, why);
    }
}

void Service::tick() {
    if (opening_) {
        return; // the portal's answer is being waited for (a local event loop runs this timer too)
    }
    const std::uint64_t now = desktop::monoNs();
    auto* h = link_.header();
    h->serviceBeatNs.store(now, std::memory_order_release);
    // the overlay looks while its beat moves: its own clock's value (PiBridge World's Godot has no CLOCK_MONOTONIC),
    // so it is its change that counts, timed with this clock
    const std::uint64_t beat = h->clientBeatNs.load(std::memory_order_acquire);
    if (beat != lastBeat_) {
        lastBeat_ = beat;
        beatSeenNs_ = beat != 0 ? now : 0;
    }
    const bool looking = beatSeenNs_ != 0 && now - beatSeenNs_ < desktop::kBeatTimeoutNs;
    const std::uint32_t want = looking ? h->want.load(std::memory_order_relaxed) : 0;
    want_.store(want, std::memory_order_relaxed);
    if (want != 0) {
        lastClientNs_ = now;
    }
    const bool capturing = pw_ != nullptr;
    if (capturing && (streamFailed_.load() || portal_.closedByPortal())) {
        stopCapture("the desktop stopped sharing", desktop::State::Failed);
        retryNs_ = now + 3'000'000'000ULL;
        return;
    }
    if (!capturing && want != 0 && now >= retryNs_) {
        startCapture();
        return;
    }
    if (capturing && want == 0 && now - lastClientNs_ > static_cast<std::uint64_t>(opt_.stopCaptureS) * 1'000'000'000ULL) {
        stopCapture("nobody looks at the desktop", desktop::State::Off);
    }
    if (capturing && looking) {
        const desktop::Pointer p = desktop::unpackPointer(h->pointer.load(std::memory_order_relaxed));
        const std::int64_t wheel = h->wheel.load(std::memory_order_relaxed);
        for (const Event& e : input_.update(p, wheel, link_.monitors())) {
            switch (e.kind) {
            case Event::Kind::Move:
                if (e.monitor < streams_.size()) {
                    portal_.pointerMotion(streams_[e.monitor].node, e.x, e.y);
                }
                break;
            case Event::Kind::Button: portal_.pointerButton(e.button, e.down); break;
            case Event::Kind::Wheel: portal_.pointerWheel(e.steps); break;
            }
        }
    } else if (capturing && !looking) {
        // the overlay went away: nothing stays pressed
        for (const Event& e : input_.update(desktop::Pointer{}, h->wheel.load(std::memory_order_relaxed), layout_)) {
            if (e.kind == Event::Kind::Button) {
                portal_.pointerButton(e.button, false);
            }
        }
    }
    if (opt_.idleExitS > 0 && pw_ == nullptr &&
        now - lastClientNs_ > static_cast<std::uint64_t>(opt_.idleExitS) * 1'000'000'000ULL) {
        log("pibridge-desktop: nobody has looked for " + std::to_string(opt_.idleExitS) + " s, ending");
        link_.setState(desktop::State::Off, "not running");
        h->serviceBeatNs.store(0, std::memory_order_release);
        QCoreApplication::quit();
    }
}

} // namespace pibridge::desktopsvc
