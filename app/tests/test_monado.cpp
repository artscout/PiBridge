// SPDX-License-Identifier: MIT
// The Dashboard's controllers from Monado (core/monado_devices.h): the link (wanted or not, no library, no socket,
// connect, poll, Monado going away, back off, another build), the card's words, the dlopen backend refusing what is
// not libmonado, and the worker thread with a fake source. No Monado is ever contacted.
#include <QFile>
#include <QTemporaryDir>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include "core/monado_devices.h"
#include "pibridge_test.h"

using namespace pibridge::app;

namespace {

struct LinkRig {
    qint64 now = 0;
    bool socket = true;
    FakeMonadoSource src;
    MonadoLink link{src, [this] { return now; }, [this](const QString&) { return socket; }};
    MonadoTarget on() const {
        MonadoTarget t;
        t.wanted = true;
        t.library = QStringLiteral("/m/libmonado.so");
        t.socketPath = QStringLiteral("/run/x/monado_comp_ipc");
        return t;
    }
};

} // namespace

TEST(link_off_no_library_and_no_socket) {
    LinkRig r;
    CHECK(r.link.tick({}).state == MndLinkState::Off);
    CHECK(r.src.libraries.isEmpty()); // nothing is tried while not wanted
    MonadoTarget t = r.on();
    t.library.clear();
    const MonadoStatus s = r.link.tick(t);
    CHECK(s.state == MndLinkState::NoLibrary && s.error.contains(QStringLiteral("libmonado.so was not found")));
    r.socket = false;
    CHECK(r.link.tick(r.on()).state == MndLinkState::NotRunning);
    CHECK(r.src.libraries.isEmpty());
}

TEST(link_connects_polls_and_handles_monado_going_away) {
    LinkRig r;
    r.src.polls = {fakeMonadoDevices()};
    MonadoStatus s = r.link.tick(r.on());
    CHECK(s.state == MndLinkState::Connected && s.devices.size() == 3);
    CHECK(r.src.libraries == QStringList{QStringLiteral("/m/libmonado.so")});
    r.now += 1000;
    CHECK(r.link.tick(r.on()).state == MndLinkState::Connected);
    CHECK(r.src.libraries.size() == 1); // one connection, polled
    // Monado's socket goes: let go at once
    r.socket = false;
    CHECK(r.link.tick(r.on()).state == MndLinkState::NotRunning);
    CHECK(r.src.disconnectCalls == 1 && !r.link.connected());
    r.socket = true;
    r.now += 1000;
    CHECK(r.link.tick(r.on()).state == MndLinkState::Connected); // back
    // a poll fails (Monado stopped answering): disconnect, wait kMndRetryMs
    r.src.polls = {std::nullopt, fakeMonadoDevices()};
    r.now += 1000;
    s = r.link.tick(r.on());
    CHECK(s.state == MndLinkState::Waiting && s.error == QStringLiteral("Monado stopped answering."));
    CHECK(!r.link.connected());
    const int tries = static_cast<int>(r.src.libraries.size());
    r.now += 1000;
    CHECK(r.link.tick(r.on()).state == MndLinkState::Waiting);
    CHECK(r.src.libraries.size() == tries); // backing off
    r.now += kMndRetryMs;
    CHECK(r.link.tick(r.on()).state == MndLinkState::Connected);
    // not wanted any more: disconnect
    const int before = r.src.disconnectCalls;
    CHECK(r.link.tick({}).state == MndLinkState::Off);
    CHECK(r.src.disconnectCalls == before + 1);
}

TEST(link_connect_failures_and_another_build) {
    LinkRig r;
    r.src.connects = {{MndConnect::Failed, QStringLiteral("Monado did not answer.")}, {MndConnect::Ok, {}}};
    MonadoStatus s = r.link.tick(r.on());
    CHECK(s.state == MndLinkState::Waiting && s.error == QStringLiteral("Monado did not answer."));
    r.now += kMndRetryMs;
    CHECK(r.link.tick(r.on()).state == MndLinkState::Connected);
    MonadoTarget other = r.on();
    other.library = QStringLiteral("/n/libmonado.so");
    r.now += 100;
    CHECK(r.link.tick(other).state == MndLinkState::Connected);
    CHECK(r.src.libraries.back() == QStringLiteral("/n/libmonado.so")); // reconnected for the other build
    LinkRig n;
    n.src.connects = {{MndConnect::NoLibrary, QStringLiteral("libmonado could not be loaded: x")}};
    CHECK(n.link.tick(n.on()).state == MndLinkState::NoLibrary);
    n.now += 10;
    s = n.link.tick(n.on());
    CHECK(s.state == MndLinkState::NoLibrary && s.error == QStringLiteral("libmonado could not be loaded: x"));
}

TEST(card_words) {
    MonadoStatus s;
    ControllersView v = monadoControllersView(s);
    CHECK(v.header.text == QStringLiteral("MONADO OFF"));
    CHECK(v.message == QStringLiteral("Start tracking (Tracking page) to see the controllers."));
    CHECK(!v.message.contains(QStringLiteral("SteamVR")));
    s.state = MndLinkState::NoLibrary;
    s.error = QStringLiteral("libmonado.so was not found in the Monado build next to monado-service.");
    v = monadoControllersView(s);
    CHECK(v.header.text == QStringLiteral("NO LIBMONADO"));
    CHECK(v.message == s.error + QStringLiteral(" The controllers cannot be read."));
    s.state = MndLinkState::Connected;
    s.error.clear();
    s.devices = fakeMonadoDevices();
    v = monadoControllersView(s);
    CHECK(v.header.text == QStringLiteral("MONADO: 1/2 ON") && v.header.kind == ChipKind::Warn);
    CHECK(v.left.present && v.left.chip.text == QStringLiteral("ON") && v.left.batteryPercent == 64 && v.left.charging);
    CHECK(v.left.batteryText == QStringLiteral("64 % charging"));
    CHECK(v.right.present && v.right.chip.text == QStringLiteral("OFF") && v.right.detail == QStringLiteral("Turn it on"));
    CHECK(v.right.batteryPercent == -1);
    CHECK(v.message.isEmpty() && v.extra.isEmpty()); // the headset is not listed as a controller
    for (MonadoDevice& d : s.devices) {
        d.batteryPresent = false;
    }
    v = monadoControllersView(s);
    CHECK(v.header.text == QStringLiteral("MONADO: 0/2 ON") && v.message == QStringLiteral("Turn the controllers on."));
    s.devices = {fakeMonadoDevices()[0]};
    v = monadoControllersView(s);
    CHECK(!v.left.present && !v.right.present && v.message == QStringLiteral("Turn the controllers on."));
    MonadoDevice nb = fakeMonadoDevices()[1];
    nb.batterySupported = false;
    MonadoDevice tracker = fakeMonadoDevices()[2];
    tracker.role = VrRole::None;
    tracker.name = QStringLiteral("Tracker");
    s.devices = {nb, tracker};
    v = monadoControllersView(s);
    CHECK(v.left.chip.text == QStringLiteral("LISTED") && v.header.text == QStringLiteral("MONADO: 1/2 ON"));
    CHECK(v.extra == QStringList{QStringLiteral("Tracker: off")});
    s.state = MndLinkState::Waiting;
    s.error = QStringLiteral("Monado did not answer.");
    CHECK(monadoControllersView(s).message == QStringLiteral("Connecting to Monado... (Monado did not answer.)"));
}

TEST(libmonado_backend_refuses_what_is_not_libmonado) {
    auto src = makeLibMonadoSource();
    CHECK(src->connect(QStringLiteral("/nonexistent/libmonado.so")).result == MndConnect::NoLibrary);
    QTemporaryDir t;
    const QString fake = t.path() + QStringLiteral("/libmonado.so");
    QFile f(fake);
    CHECK(f.open(QIODevice::WriteOnly));
    f.write("not an ELF file");
    f.close();
    const MndConnect c = src->connect(fake);
    CHECK(c.result == MndConnect::NoLibrary && c.error.startsWith(QStringLiteral("libmonado could not be loaded")));
    CHECK(!src->poll().has_value());
    src->disconnect(); // safe when not connected
}

TEST(worker_delivers_off_then_the_devices_on_its_thread) {
    std::mutex m;
    std::vector<MndLinkState> seen;
    std::atomic<bool> connected{false};
    auto f = std::make_unique<FakeMonadoSource>();
    f->polls = {fakeMonadoDevices()};
    {
        MonadoWorker w(std::move(f),
                       [&](const MonadoStatus& s) {
                           std::lock_guard lk(m);
                           seen.push_back(s.state);
                           if (s.state == MndLinkState::Connected && s.devices.size() == 3) {
                               connected = true;
                           }
                       },
                       50);
        MonadoTarget t;
        t.wanted = true;
        t.library = QStringLiteral("/fake/libmonado.so");
        w.setTarget(t);
        for (int i = 0; i < 200 && !connected; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } // joined here
    CHECK(connected.load());
    std::lock_guard lk(m);
    CHECK(!seen.empty());
}

PIBRIDGE_TEST_MAIN()
