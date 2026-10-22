// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax/tests/session.rs (MIT OR Apache-2.0): the session thread against the in-process fake.
#include "common.h"
#include "pibridge_test.h"

#include "pibridge/hmd/descriptor.h"
#include "pibridge/hmd/fake.h"
#include "pibridge/hmd/session.h"

#include <algorithm>
#include <atomic>
#include <iterator>
#include <mutex>
#include <thread>

using namespace pibridge::hmd;
using namespace std::chrono;
using Clock = steady_clock;
using testutil::bytes;

static constexpr milliseconds MS(long long v) { return milliseconds(v); }

static std::shared_ptr<const Allowlist> allow(const char* id = "dream-air") {
    const auto d = Descriptor::load(std::filesystem::path(PIBRIDGE_SOURCE_DEVICES_DIR) / (std::string(id) + ".toml"));
    if (!d) {
        std::abort();
    }
    return std::make_shared<const Allowlist>(d->allowlist);
}

static Config base(Config c = {}) {
    if (!c.allowlist) {
        c.allowlist = allow();
    }
    return c;
}

struct Started {
    std::unique_ptr<Session> s;
    std::shared_ptr<EventQueue> rx;
    fake::FakeDevice dev;
};

static Started start(Config cfg) {
    auto [be, dev] = fake::pair();
    auto r = Session::open_channel_with_backend(base(std::move(cfg)), std::move(be), 100'000, true);
    if (!r) {
        std::abort();
    }
    auto [s, rx] = std::move(*r);
    CHECK(s->wait_connected(MS(2000)).has_value());
    CHECK(dev.wait_host(MS(1000)));
    return Started{std::move(s), rx, dev};
}

/// Device-side thread that answers requests like the headset does.
static std::thread responder(fake::FakeDevice dev, std::atomic<bool>& stop, std::vector<Report>& seen) {
    return std::thread([dev, &stop, &seen] {
        while (!stop.load()) {
            const auto c = dev.recv_command(MS(20));
            if (!c) {
                continue;
            }
            seen.push_back(*c);
            if ((*c)[0] != 0x58) {
                continue;
            }
            if ((*c)[2] == 0x16) {
                // noise first: a 0x37 and an unrelated reply must not satisfy the request
                dev.send_report(0, bytes({0x4c, 0x0f, 0x37, 0x02, 0x48, 1, 2, 3, 4, 0x49, 0, 2, 1, 0, 8, 0x2d}));
                dev.send_report(0, bytes({0x4c, 0x04, 0x51, 0x00}));
                dev.send_report(0, bytes({0x4c, 0x1a, 0x16, (*c)[3], 5, 'H', '1', '2', '3', '4'}));
            } else if ((*c)[2] == 0x03) {
                dev.send_report(0, bytes({0x4c, 0x07, 0x03, (*c)[3], 0x00, 0x08, 0x2d, 0x6b}));
            }
        }
    });
}

template <class T>
static std::vector<T> only(const std::vector<Event>& evs) {
    std::vector<T> v;
    for (const auto& e : evs) {
        if (const T* x = std::get_if<T>(&e)) {
            v.push_back(*x);
        }
    }
    return v;
}

TEST(connect_sends_wake_then_keepalives_every_213ms) {
    auto st = start({});
    const auto t0 = Clock::now();
    std::vector<std::pair<Clock::duration, Report>> cmds;
    while (Clock::now() - t0 < MS(1100)) {
        if (const auto c = st.dev.recv_command(MS(50))) {
            cmds.emplace_back(Clock::now() - t0, *c);
        }
    }
    CHECK(!cmds.empty() && cmds[0].second == wake()); // first command is the wake
    std::vector<std::pair<Clock::duration, Report>> ka;
    for (const auto& c : cmds) {
        if (c.second[2] == 0x39) {
            ka.push_back(c);
        }
    }
    CHECK(ka.size() >= 5 && ka.size() <= 6);
    for (const auto& [t, c] : ka) {
        CHECK(c == keepalive(0));
        CHECK(c[1] == 4); // Windows / python wire form LL=4
    }
    for (std::size_t i = 1; i < ka.size(); i++) {
        const auto d = duration_cast<milliseconds>(ka[i].first - ka[i - 1].first).count();
        CHECK(std::abs(d - 213) < 40);
    }
    CHECK(st.s->stats().keepalives >= 5);
}

TEST(rate_byte_follows_handle_and_file) {
    {
        auto st = start({});
        st.s->set_video_rate(72);
        const auto seen = st.dev.drain_commands(MS(600));
        CHECK(std::any_of(seen.begin(), seen.end(), [](const Report& c) { return c == keepalive(72); }));
    }
    // file source (pimax_keepalived.py): re-read on every keepalive, missing = 0, garbage = 0
    const auto dir = testutil::tmpdir("ratefile");
    const auto f = dir / "pimax-rate";
    Config cfg;
    cfg.rate_file = f;
    auto st = start(cfg);
    const auto a = st.dev.drain_commands(MS(300));
    CHECK(std::all_of(a.begin(), a.end(), [](const Report& c) { return c[2] != 0x39 || c[4] == 0; }));
    std::ofstream(f) << "90\n";
    const auto b = st.dev.drain_commands(MS(500));
    CHECK(std::any_of(b.begin(), b.end(), [](const Report& c) { return c == keepalive(90); }));
    std::ofstream(f) << "junk";
    const auto c = st.dev.drain_commands(MS(500));
    CHECK(std::any_of(c.begin(), c.end(), [](const Report& x) { return x == keepalive(0); }));
    std::filesystem::remove_all(dir);
}

TEST(rate_file_off_sends_one_rate_zero_keepalive_then_silence) {
    // Pimax Play closed (Crystal Super capture, take shutdown): the last keepalive carries rate 0, then the host only
    // reads and the headset goes to its green standby
    const auto dir = testutil::tmpdir("ratefile-off");
    const auto f = dir / "pimax-rate";
    std::ofstream(f) << "90\n";
    Config cfg;
    cfg.rate_file = f;
    cfg.allowlist = allow("crystal-super");
    auto st = start(cfg);
    const auto a = st.dev.drain_commands(MS(500));
    CHECK(std::any_of(a.begin(), a.end(), [](const Report& c) { return c == keepalive(90); }));
    std::ofstream(f) << "off\n";
    const auto b = st.dev.drain_commands(MS(900));
    std::vector<Report> kab;
    for (const auto& c : b) {
        if (c[2] == 0x39) {
            kab.push_back(c);
        }
    }
    // maybe one more 90 before the file was seen, then exactly one rate-0 and nothing after it
    CHECK(!kab.empty() && kab.back() == keepalive(0));
    CHECK(std::count(kab.begin(), kab.end(), keepalive(0)) == 1);
    const auto quiet = st.dev.drain_commands(MS(600));
    CHECK(std::none_of(quiet.begin(), quiet.end(), [](const Report& c) { return c[2] == 0x39; }));
    CHECK(st.s->is_connected()); // still reading the headset
    // back on: keepalives at the rate again
    std::ofstream(f) << "90\n";
    const auto c = st.dev.drain_commands(MS(500));
    CHECK(std::any_of(c.begin(), c.end(), [](const Report& x) { return x == keepalive(90); }));
    std::filesystem::remove_all(dir);
}

TEST(passive_session_sends_nothing) {
    auto st = start(Config::passive());
    CHECK(st.dev.drain_commands(MS(600)).empty());
    CHECK(st.s->is_connected());
}

static Report synth_imu(std::uint32_t i) {
    Report r = bytes({0x53, 0x31, 0x08, 0x00});
    const std::uint32_t ts = 1000 + 2000 * i;
    for (int k = 0; k < 4; k++) {
        r[4 + static_cast<std::size_t>(k)] = static_cast<std::uint8_t>(ts >> (8 * k));
    }
    r[14] = 0x08; // accel x = 2048 = 1 g
    r[20] = static_cast<std::uint8_t>(i);
    return r;
}

static Report synth_ctrl(std::uint32_t i) {
    Report r = bytes({0x53, 0x32, 0x08, 0x03, 0x43, 128, 128, static_cast<std::uint8_t>(i)});
    r[16] = 0x04; // accel x = 1024
    r[46] = 99;
    return r;
}

TEST(imu_controller_status_log_events_and_drain) {
    auto st = start({});
    // the real captures when present (as in Rust), otherwise synthetic reports of the same kinds
    std::vector<Report> imu;
    std::vector<Report> ctl;
    if (const auto rec = testutil::load_rec("p83.rec.gz")) {
        for (std::size_t i = 0; i < 300; i++) {
            imu.push_back((*rec)[i].second);
        }
    } else {
        for (std::uint32_t i = 0; i < 300; i++) {
            imu.push_back(synth_imu(i));
        }
    }
    if (const auto rec = testutil::load_rec("p82.rec.gz")) {
        for (const auto& [t, b] : *rec) {
            if (b[3] == 3 && ctl.size() < 200) {
                ctl.push_back(b);
            }
        }
    } else {
        for (std::uint32_t i = 0; i < 200; i++) {
            ctl.push_back(synth_ctrl(i));
        }
    }
    for (const auto& r : imu) {
        CHECK(st.dev.send_report(2, r));
    }
    for (const auto& r : ctl) {
        CHECK(st.dev.send_report(1, r));
    }
    Report sr = bytes({0x4c, 0x0f, 0x37, 0x03, 0x48, 4, 0x88, 0x91, 2, 0x63, 0, 2, 1, 0, 8, 0x2e});
    const std::uint8_t second[] = {0x0f, 0x37, 0x02, 0x48, 0, 0, 0, 0, 0x49};
    std::copy(std::begin(second), std::end(second), sr.begin() + 17);
    CHECK(st.dev.send_report(0, sr));
    Report lg{};
    const std::string l1 = "[00:00:01] app: hello\r\npartial";
    std::copy(l1.begin(), l1.end(), lg.begin());
    CHECK(st.dev.send_report(3, lg));
    Report lg2{};
    const std::string l2 = " line\n";
    std::copy(l2.begin(), l2.end(), lg2.begin());
    CHECK(st.dev.send_report(3, lg2));
    std::this_thread::sleep_for(MS(300));
    const auto evs = st.rx->drain();
    const auto imus = only<ImuSample>(evs);
    CHECK(imus.size() == 600); // two samples per report
    const auto first = decode_imu(imu[0]).value();
    CHECK(imus.size() >= 2 && imus[0] == first[0] && imus[1] == first[1]);
    const auto ctrls = only<ControllerReport>(evs);
    CHECK(ctrls.size() == 200);
    CHECK(std::all_of(ctrls.begin(), ctrls.end(), [](const ControllerReport& c) { return c.slot == 3; }));
    const auto sts = only<ControllerStatus>(evs);
    CHECK(sts.size() == 2);
    CHECK(sts.size() == 2 && sts[0].slot == 3 && sts[0].battery == 0x63 && sts[1].slot == 2 && sts[1].battery == 0x49);
    const auto logs = only<LogLine>(evs);
    CHECK((logs == std::vector<LogLine>{{"[00:00:01] app: hello"}, {"partial line"}}));
    const auto states = only<State>(evs);
    CHECK(std::find(states.begin(), states.end(), State::Connected) != states.end());
    const auto s = st.s->stats();
    CHECK(s.imu_reports == 300 && s.controller_reports == 200 && s.status_messages == 2 && s.log_lines == 2);
}

TEST(request_reply_matching_and_timeout) {
    auto st = start({});
    std::atomic<bool> stop{false};
    std::vector<Report> seen;
    auto h = responder(st.dev, stop, seen);
    CHECK(st.s->read_serial(0, MS(1000)).value_or("") == "H1234");
    CHECK((st.s->read_version(2, MS(1000)).value_or(std::vector<std::uint8_t>{}) ==
           std::vector<std::uint8_t>{0x00, 0x08, 0x2d, 0x6b}));
    // no responder for 0x50 (an allowed read): times out, and the next request still works
    const auto t = Clock::now();
    const auto r = st.s->request(cmd::INFO, 0, {}, MS(150));
    CHECK(!r && r.error().code == Errc::Timeout);
    CHECK(Clock::now() - t >= MS(140) && Clock::now() - t < MS(400));
    CHECK(st.s->read_serial(0, MS(1000)).value_or("") == "H1234");
    const std::vector<std::uint8_t> big(61, 0);
    const auto tl = st.s->send(0x16, 0, big);
    CHECK(!tl && tl.error().code == Errc::TooLong);
    stop.store(true);
    h.join();
    CHECK(std::any_of(seen.begin(), seen.end(), [](const Report& c) {
        return c[0] == 0x58 && c[1] == 0x04 && c[2] == 0x16 && c[3] == 0x00 && c[4] == 0x1e;
    }));
}

TEST(reports_outside_the_allowlist_are_refused_and_never_written) {
    // the Rust timeout case used opcode 0x2c (the visor probe), which the allowlist refuses: nothing reaches the device
    auto st = start({});
    (void)st.dev.drain_commands(MS(50));
    const auto r = st.s->request(0x2c, 0, {}, MS(150));
    CHECK(!r && r.error().code == Errc::Refused);
    const auto d = st.s->send(0x19, 0, {}); // DeviceCheckResult: denied
    CHECK(!d && d.error().code == Errc::Refused && d.error().message().find("DeviceCheckResult") != std::string::npos);
    st.s->set_video_rate(60); // not a rate the allowlist knows: the keepalive is refused, not sent
    const auto seen = st.dev.drain_commands(MS(500));
    CHECK(std::none_of(seen.begin(), seen.end(), [](const Report& c) { return c[2] == 0x2c || c[2] == 0x19; }));
    CHECK(std::none_of(seen.begin(), seen.end(), [](const Report& c) { return c[2] == 0x39 && c[4] == 60; }));
    CHECK(st.s->stats().refused >= 3);
}

TEST(no_description_means_nothing_is_sent) {
    auto [be, dev] = fake::pair();
    Config cfg;
    cfg.devices_dirs = {testutil::tmpdir("no-devices")}; // an empty folder: no description to load
    auto r = Session::open_with_backend(cfg, std::move(be), nullptr);
    CHECK(r.has_value());
    auto& s = *r;
    CHECK(s->wait_connected(MS(2000)).has_value());
    CHECK(dev.drain_commands(MS(500)).empty());
    const auto w = s->send_report(wake());
    CHECK(!w && w.error().code == Errc::Refused);
}

TEST(the_model_of_the_link_selects_its_description) {
    auto [be, dev] = fake::pair();
    dev.set_model(CRYSTAL_SUPER);
    Config cfg;
    cfg.devices_dirs = {PIBRIDGE_SOURCE_DEVICES_DIR};
    auto r = Session::open_with_backend(cfg, std::move(be), nullptr);
    auto& s = *r;
    CHECK(s->wait_connected(MS(2000)).has_value());
    CHECK(s->model() && s->model()->id == "crystal-super");
    const auto seen = dev.drain_commands(MS(300));
    CHECK(!seen.empty() && seen[0] == wake());
    // the Crystal Super lists the IPD motor (2026-10-22) and the haptic command (2026-10-19), but not the eye-module
    // power off (captured, not enabled)
    CHECK(s->send_report(*build_command(0x5c, 0x05, std::vector<std::uint8_t>{0x01, 0xaf, 0x18})).has_value());
    CHECK(s->send_report(haptic_command(3, 25, 300)).has_value());
    const auto off = s->send_report(*build_command(0x3c, 0x05, std::vector<std::uint8_t>{0x00, 0x00}));
    CHECK(!off && off.error().code == Errc::Refused);
}

TEST(calibration_over_the_session_with_a_lost_reply) {
#if PIBRIDGE_HAVE_ZLIB
    const auto xs = testutil::read_text(testutil::data_dir() / "synthetic-headset-calibration.xml").value_or("");
    const std::vector<std::uint8_t> xml(xs.begin(), xs.end());
    const auto blob = calib::build_blob(xml);
    auto st = start({});
    std::atomic<bool> stop{false};
    auto d2 = st.dev;
    std::thread h([&] {
        bool dropped_once = false;
        while (!stop.load()) {
            const auto c = d2.recv_command(MS(20));
            if (!c) {
                continue;
            }
            if ((*c)[2] == 0x0a && (*c)[4] == 1) {
                const auto addr = static_cast<std::uint16_t>((*c)[5] | ((*c)[6] << 8));
                if (addr == 0x100 && !dropped_once) {
                    dropped_once = true; // lose the first reply: the host must retry
                    continue;
                }
                d2.send_report(0, calib::make_block_reply(blob, addr));
            }
        }
    });
    const auto got = st.s->read_calibration_blob(MS(300));
    CHECK(got.has_value());
    CHECK(calib::inflate_blob(got.value_or(std::vector<std::uint8_t>{})).value_or(std::vector<std::uint8_t>{}) == xml);
    stop.store(true);
    h.join();
#else
    std::fprintf(stderr, "  skipped: needs zlib\n");
#endif
}

TEST(unplug_and_reconnect) {
    auto st = start({});
    CHECK(st.s->stats().connects == 1);
    st.dev.unplug();
    const auto t = Clock::now();
    while (st.s->is_connected() && Clock::now() - t < MS(2000)) {
        std::this_thread::sleep_for(MS(5));
    }
    CHECK(!st.s->is_connected()); // session notices the hang-up
    const auto a = st.s->send(0x03, 0, {});
    CHECK(!a && a.error().code == Errc::NotConnected);
    const auto b = st.s->request(0x03, 0, {}, MS(100));
    CHECK(!b && b.error().code == Errc::NotConnected);
    st.dev.plug();
    CHECK(st.s->wait_connected(MS(3000)).has_value());
    CHECK(st.dev.wait_host(MS(500)));
    CHECK(st.s->stats().connects == 2);
    const auto first = st.dev.recv_command(MS(500));
    CHECK(first && *first == wake()); // wake is repeated after a reconnect
    std::this_thread::sleep_for(MS(50));
    const auto states = only<State>(st.rx->drain());
    CHECK((states == std::vector<State>{State::Connected, State::Disconnected, State::Connected}));
}

TEST(pending_request_is_aborted_when_the_device_vanishes) {
    auto st = start({});
    auto d = st.dev;
    std::thread u([d] {
        std::this_thread::sleep_for(MS(100));
        d.unplug();
    });
    const auto t = Clock::now();
    const auto r = st.s->request(cmd::INFO, 0, {}, MS(3000));
    CHECK(!r && r.error().code == Errc::NotConnected);
    CHECK(Clock::now() - t < MS(1500));
    u.join();
}

TEST(auto_attach_initial_burst_and_reattach_on_0x37) {
    Config cfg;
    cfg.auto_attach = true;
    auto st = start(cfg);
    const auto t0 = Clock::now();
    std::vector<std::pair<Clock::duration, Report>> seen;
    // announce slot 2 as awake-but-silent shortly after the first burst
    std::this_thread::sleep_for(MS(100));
    CHECK(st.dev.send_report(0, bytes({0x4c, 0x0f, 0x37, 0x02, 0x48, 4, 0x88, 0x91, 2, 0x63, 0, 2, 1, 0, 8, 0x2e})));
    while (Clock::now() - t0 < MS(3400)) {
        if (const auto c = st.dev.recv_command(MS(50))) {
            seen.emplace_back(Clock::now() - t0, *c);
        }
    }
    std::vector<std::pair<Clock::duration, Report>> attach;
    for (const auto& x : seen) {
        const auto c = x.second[2];
        if (c == 0x16 || c == 0x50 || c == 0x0a || c == 0x03) {
            attach.push_back(x);
        }
    }
    const auto by_slot = [&](std::uint8_t slot) {
        return std::count_if(attach.begin(), attach.end(), [&](const auto& x) { return x.second[3] == slot; });
    };
    // initial: 4 commands x 2 slots; then one re-burst for slot 2 only (>= 2.5 s after the first)
    CHECK(by_slot(2) == 8);
    CHECK(by_slot(3) == 4);
    std::vector<std::pair<Clock::duration, Report>> s2;
    for (const auto& x : attach) {
        if (x.second[3] == 2) {
            s2.push_back(x);
        }
    }
    if (s2.size() == 8) {
        CHECK(s2[0].second[2] == 0x16);
        CHECK(s2[4].second[2] == 0x16);
        CHECK(s2[4].first - s2[0].first >= MS(2400)); // retry rate limited to 2.5 s
        // spacing inside the (live) retry burst is about 10 ms
        CHECK(s2[7].first - s2[4].first >= MS(25) && s2[7].first - s2[4].first < MS(120));
    }
    CHECK(st.s->stats().auto_attaches == 1);
}

TEST(no_reattach_while_reports_flow) {
    Config cfg;
    cfg.auto_attach = true;
    auto st = start(cfg);
    Report rep = synth_ctrl(0);
    rep[3] = 2;
    if (const auto rec = testutil::load_rec("p82.rec.gz")) {
        for (const auto& [t, b] : *rec) {
            if (b[3] == 2) {
                rep = b;
                break;
            }
        }
    }
    const Report sr = bytes({0x4c, 0x0f, 0x37, 0x02, 0x48, 4, 0x88, 0x91, 2, 0x63, 0, 2, 1, 0, 8, 0x2e});
    const auto t0 = Clock::now();
    while (Clock::now() - t0 < MS(3200)) {
        st.dev.send_report(1, rep); // slot 2 alive
        st.dev.send_report(0, sr);  // stray heartbeat
        std::this_thread::sleep_for(MS(20));
    }
    CHECK(st.s->stats().auto_attaches == 0);
}

TEST(video_prepare_wire_and_rate) {
    auto st = start({});
    CHECK(st.s->video_prepare(72).has_value());
    const auto seen = st.dev.drain_commands(MS(500));
    const auto pos = [&](std::initializer_list<std::uint8_t> p) {
        for (std::size_t i = 0; i < seen.size(); i++) {
            if (std::equal(p.begin(), p.end(), seen[i].begin())) {
                return static_cast<long>(i);
            }
        }
        return -1L;
    };
    const long a = pos({0x58, 0x05, 0x5b, 0x05, 0x02, 0x02});
    const long b = pos({0x58, 0x04, 0x29, 0x00, 0x01});
    const long c = pos({0x58, 0x0d, 0x1a, 0x05, 0x02, 0x48});
    const long d = pos({0x58, 0x0d, 0x1a, 0x05, 0x05, 0x00});
    CHECK(a >= 0 && a < b && b < c && c < d);
    CHECK(std::any_of(seen.begin(), seen.end(), [](const Report& x) { return x == keepalive(72); }));
}

TEST(haptic_goes_out_on_the_session_thread_rate_limited) {
    auto st = start({});
    (void)st.dev.drain_commands(MS(50));
    const auto haptics_in = [](const std::vector<Report>& v) {
        std::vector<Report> h;
        std::copy_if(v.begin(), v.end(), std::back_inserter(h), [](const Report& c) { return c[2] == 0x07; });
        return h;
    };
    // Pimax Play's test pulse (right, 25 %, 300 ms): one command, the captured bytes
    CHECK(st.s->haptic(SLOT_RIGHT, haptics::Pulse{25, 300}));
    auto h = haptics_in(st.dev.drain_commands(MS(100)));
    CHECK(h.size() == 1);
    const std::uint8_t pulse[] = {0x58, 0x09, 0x07, 0x03, 0x19, 0x2c, 0x01, 0, 0, 0};
    CHECK(!h.empty() && std::equal(std::begin(pulse), std::end(pulse), h[0].begin()));
    // a burst of 20 requests within ~1 ms: far fewer commands, the last one is the newest value
    const auto t0 = Clock::now();
    for (std::uint8_t k = 0; k < 20; k++) {
        st.s->haptic(SLOT_LEFT, haptics::Pulse{static_cast<std::uint8_t>(80 + k), 12});
    }
    h = haptics_in(st.dev.drain_commands(MS(100)));
    CHECK(Clock::now() - t0 >= MS(8));
    CHECK(!h.empty() && h.size() <= 3);
    const std::uint8_t last[] = {0x58, 0x09, 0x07, 0x02, 99, 12, 0};
    CHECK(!h.empty() && std::equal(std::begin(last), std::end(last), h.back().begin()));
    CHECK(!st.s->haptic(5, haptics::Pulse{1, 1})); // unknown slot
    CHECK(st.s->haptic_counts().first >= 2);
}

/// Without `haptic` in the connected model's description the pulse is refused by the gate: nothing reaches the
/// device, the refusal is counted and logged.
TEST(haptic_outside_the_allowlist_is_refused) {
    auto a = std::make_shared<Allowlist>(*allow());
    std::erase_if(a->commands, [](const CommandSpec& c) { return c.id == "haptic"; });
    auto lines = std::make_shared<std::vector<std::string>>();
    auto mu = std::make_shared<std::mutex>();
    Config c;
    c.allowlist = a;
    c.log = [lines, mu](std::string_view l) {
        std::lock_guard g(*mu);
        lines->emplace_back(l);
    };
    auto st = start(std::move(c));
    (void)st.dev.drain_commands(MS(50));
    const auto before = st.s->stats().refused;
    CHECK(st.s->haptic(SLOT_RIGHT, haptics::Pulse{25, 300}));
    const auto seen = st.dev.drain_commands(MS(100));
    CHECK(std::none_of(seen.begin(), seen.end(), [](const Report& r) { return r[2] == 0x07; }));
    CHECK(st.s->stats().refused == before + 1);
    std::lock_guard g(*mu);
    CHECK(std::any_of(lines->begin(), lines->end(),
                      [](const std::string& l) { return l.starts_with("haptic write: refused"); }));
}

TEST(close_is_prompt_and_drop_joins) {
    {
        auto st = start({});
        const auto t = Clock::now();
        st.s->close();
        CHECK(Clock::now() - t < MS(100));
    }
    // never connects: still stops promptly
    auto [be, dev] = fake::pair();
    dev.unplug();
    auto s = Session::open_with_backend(base(), std::move(be), std::make_unique<NoopHandler>());
    const auto w = (*s)->wait_connected(MS(100));
    CHECK(!w && w.error().code == Errc::Timeout);
    const auto t = Clock::now();
    s->reset();
    CHECK(Clock::now() - t < MS(100));
}

static std::vector<Clock::duration> wakes_in(const fake::FakeDevice& dev, long long ms) {
    const auto t0 = Clock::now();
    std::vector<Clock::duration> v;
    while (Clock::now() - t0 < MS(ms)) {
        if (const auto c = dev.recv_command(MS(20)); c && *c == wake()) {
            v.push_back(Clock::now() - t0);
        }
    }
    return v;
}

TEST(unacked_wake_is_sent_once_more_after_half_a_second) {
    // after a headset restart the first wake is ignored: wake at 0, again at ~0.5 s, no third
    auto st = start({});
    const auto w = wakes_in(st.dev, 1600);
    CHECK(w.size() == 2);
    CHECK(w.size() == 2 && w[1] >= MS(450) && w[1] <= MS(900));
}

TEST(acked_wake_is_not_repeated) {
    auto st = start({});
    CHECK(st.dev.send_report(0, bytes({0x4c, 0x04, 0x1a, 0x05, 0x01, 0x07})));
    CHECK(wakes_in(st.dev, 1200).size() == 1); // only the first wake
}

PIBRIDGE_TEST_MAIN()
