// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/live.rs tests (MIT OR Apache-2.0).
//
// The live tracker against the in-process FakeChip, which streams eyemath-encoded gaze (live::fake_payload, the Rust
// FakeChip::sample). The Rust tests that need the private full start list run here on the public base list, which
// has the same subscribe (0x4c4), calibration (0x456) and 0x547 steps. Nothing opens a device node: RealDevice is
// only tried on temporary sysfs trees, and HOME points at a temporary directory so no user file is read.
#include "pibridge/eye/live.h"

#include "pibridge/eye/fake.h"
#include "pibridge/eye/frames.h"

#include "pibridge_test.h"
#include "testutil.h"

#include <atomic>
#include <cstdlib>
#include <limits>
#include <mutex>
#include <numbers>
#include <thread>

using namespace pibridge::eye;
using namespace pibridge::eye::live;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace {

// Before any test: no user calibration, correction or environment can leak in.
const fs::path g_home = [] {
    const auto d = testutil::temp_dir("live-home");
    ::setenv("HOME", d.c_str(), 1);
    for (const char* v : {"PIBRIDGE_EYE_CALIBRATION", "PIBRIDGE_EYE_CORRECTION", "PIBRIDGE_EYE_FRAMES", "PIBRIDGE_EYE_PROFILE",
                          "PIBRIDGE_EYE_FLIP_X", "PIBRIDGE_EYE_FLIP_Y", "PIBRIDGE_EYE_FLIP_Z", "PIBRIDGE_EYE_SWAP",
                          "PIBRIDGE_EYE_OFFSET_MM", "PIBRIDGE_EYE_CAL", "PIBRIDGE_EYE_SKIP_AUTH", "PIBRIDGE_EYE_LATENCY_MS",
                          "PIBRIDGE_EYE_USB_RESET", "PIBRIDGE_EYE_POWER_ON", "PIBRIDGE_EYE_USB", "PIBRIDGE_EYE_FILTER"}) {
        ::unsetenv(v);
        ::unsetenv(("PIMAX_" + std::string(v + 6)).c_str()); // the older names are read too (pibridge_env.h)
    }
    return d;
}();
const struct Cleanup {
    ~Cleanup() {
        std::error_code ec;
        fs::remove_all(g_home, ec);
    }
} g_cleanup;

Config fast_cfg() {
    Config c;
    c.pace = false;
    c.retry = 20ms;
    c.history = 64;
    return c;
}

fake::FakeChip chip(std::optional<std::uint32_t> start_on = session::OP_SUBSCRIBE) {
    fake::FakeChip c;
    c.payload = fake_payload;
    c.start_on = start_on;
    return c;
}

std::vector<ttp::Message> sent_messages(const fake::FakeChip& c) {
    ttp::StreamDecoder dec(ttp::Direction::Out);
    std::vector<ttp::Message> out;
    for (const auto& s : c.sent) {
        if (const auto* b = std::get_if<fake::SentBulkOut>(&s)) {
            for (auto& m : dec.push(b->data)) {
                if (m) {
                    out.push_back(std::move(*m));
                }
            }
        }
    }
    return out;
}

std::vector<std::uint32_t> sent_ops(const fake::FakeChip& c) {
    std::vector<std::uint32_t> ops;
    for (const auto& m : sent_messages(c)) {
        ops.push_back(m.header.id);
    }
    return ops;
}

bool has(const std::vector<std::uint32_t>& v, std::uint32_t x) { return std::ranges::find(v, x) != v.end(); }

/// The first `n` samples of a link on a fresh fake chip that streams after the subscribe.
std::vector<pibridge::eyemath::Sample> first_samples(const Config& cfg, std::size_t n) {
    std::vector<pibridge::eyemath::Sample> got;
    std::stop_source stop;
    Pipeline pipe(cfg, [&](const pibridge::eyemath::Sample& s) {
        got.push_back(s);
        if (got.size() >= n) {
            stop.request_stop();
        }
    });
    auto c = chip();
    CHECK(run_link(c, cfg, stop.get_token(), pipe, {}, {}).has_value());
    return got;
}

template <class F>
bool wait_for(F f, Duration max) {
    const auto t0 = Clock::now();
    while (!f() && Clock::now() - t0 < max) {
        std::this_thread::sleep_for(10ms);
    }
    return f();
}

FnDevice::OpenFn streaming_chips() {
    return [](const Config&, std::stop_token) -> std::expected<std::unique_ptr<Transport>, std::string> {
        return std::make_unique<fake::FakeChip>(chip());
    };
}

void quiet(std::string_view) {}

} // namespace

TEST(link_inits_streams_and_unsubscribes_without_cal) {
    const auto cfg = fast_cfg();
    std::vector<pibridge::eyemath::Sample> v;
    std::stop_source stop;
    Pipeline pipe(cfg, [&](const pibridge::eyemath::Sample& s) {
        v.push_back(s);
        if (v.size() >= 40) {
            stop.request_stop();
        }
    });
    auto c = chip();
    std::vector<State> states;
    CHECK(run_link(c, cfg, stop.get_token(), pipe, [&](State s) { states.push_back(s); }, {}).has_value());
    const auto ops = sent_ops(c);
    CHECK(has(ops, 0x4c4));
    CHECK(!has(ops, 0x456)); // calibration must not be sent by default
    CHECK(!ops.empty() && ops.back() == 0x4ce); // unsubscribe last
    for (const auto denied : frames::DENIED_OPS) {
        CHECK(!has(ops, denied));
    }
    CHECK(!c.streaming); // fake stops streaming on 0x4ce
    CHECK(!states.empty() && states.front() == State::Connecting);
    CHECK(std::ranges::find(states, State::Streaming) != states.end());
    CHECK(v.size() >= 40);
    for (std::size_t i = 1; i < v.size(); i++) {
        CHECK(v[i].host_ts_ns > v[i - 1].host_ts_ns && v[i].seq == v[i - 1].seq + 1);
    }
    // fake: eye A tracked only, A is the right eye by default
    CHECK(v[0].eyes[1].direction_valid && !v[0].eyes[0].direction_valid);
    CHECK(v[0].gaze && v[0].gaze->eyes == 0b10);
}

TEST(with_cal_sends_the_captured_blob_only) {
    auto cfg = fast_cfg();
    cfg.with_cal = true;
    std::stop_source stop;
    Pipeline pipe(cfg, [&](const pibridge::eyemath::Sample& s) {
        if (s.seq >= 5) {
            stop.request_stop();
        }
    });
    auto c = chip(frames::CAL_OP); // starts streaming after 0x456
    CHECK(run_link(c, cfg, stop.get_token(), pipe, {}, {}).has_value());
    const auto ops = sent_ops(c);
    CHECK(has(ops, 0x456));
    CHECK(!ops.empty() && ops.back() == 0x4ce);
}

TEST(already_streaming_chip_is_adopted_not_reinitialised) {
    const auto cfg = fast_cfg();
    std::stop_source stop;
    Pipeline pipe(cfg, [&](const pibridge::eyemath::Sample& s) {
        if (s.seq >= 5) {
            stop.request_stop();
        }
    });
    auto c = chip();
    c.streaming = true;
    std::vector<std::string> log;
    CHECK(run_link(c, cfg, stop.get_token(), pipe, {}, [&](std::string_view m) { log.emplace_back(m); }).has_value());
    CHECK(c.sent.empty()); // nothing may be sent to a session we did not start
    CHECK(!log.empty() && log.front().starts_with("gaze already streaming"));
}

TEST(silent_chip_is_a_stall_error) {
    auto cfg = fast_cfg();
    cfg.stall = 100ms;
    std::stop_source stop;
    Pipeline pipe(cfg, {});
    auto c = chip(std::nullopt); // never streams
    // the first-gaze wait is 10 s; check that a dead sequence step does not hang: silence the subscribe op
    c.silent.push_back(0x4c4);
    const auto t0 = Clock::now();
    std::expected<void, LinkError> r;
    {
        std::jthread h([&] { r = run_link(c, cfg, stop.get_token(), pipe, {}, {}); });
        std::this_thread::sleep_for(500ms);
        stop.request_stop();
    }
    // stopped by the flag while waiting for the response: clean exit, bounded time
    CHECK(r.has_value());
    CHECK(Clock::now() - t0 < 5s);
}

TEST(tracker_thread_publishes_and_stops_cleanly) {
    auto tr = Tracker::start_with(fast_cfg(), streaming_chips(), quiet);
    CHECK(wait_for([&] { return tr->latest().has_value(); }, 10s));
    const auto l = tr->latest();
    CHECK(l.has_value());
    CHECK(tr->state() == State::Streaming);
    // interpolation at a time between two history entries
    std::this_thread::sleep_for(100ms);
    const auto l2 = tr->latest();
    const std::int64_t mid = (l->host_ts_ns + l2->host_ts_ns) / 2;
    CHECK(tr->sample_at(mid)->host_ts_ns == mid);
    CHECK(tr->sample_at(std::numeric_limits<std::int64_t>::max())->seq == tr->latest()->seq);
    CHECK(tr->stats().samples > 0);
    tr->stop();
    CHECK(tr->state() == State::Stopped);
}

TEST(pipeline_applies_the_configured_correction_to_every_sample) {
    const auto first_gaze = [](std::optional<pibridge::eyemath::Correction> corr) {
        auto cfg = fast_cfg();
        cfg.correction = std::move(corr);
        const auto v = first_samples(cfg, 3);
        CHECK(!v.empty() && v[0].gaze.has_value()); // the fake chip tracks eye A
        return v.empty() || !v[0].gaze ? pibridge::eyemath::Gaze{} : *v[0].gaze;
    };
    const auto raw = first_gaze(std::nullopt);
    auto c = pibridge::eyemath::Correction::identity();
    c.cx = {0.1, 1.0, 0.0, 0.0, 0.0, 0.0}; // shift u by 0.1
    const auto fixed = first_gaze(c);
    const auto d = [](const pibridge::eyemath::Vec3f& f) { return pibridge::eyemath::Vec3{f[0], f[1], f[2]}; };
    const auto rt = pibridge::eyemath::tangent_of(d(raw.direction));
    const auto ft = pibridge::eyemath::tangent_of(d(fixed.direction));
    CHECK(rt && ft);
    CHECK(std::fabs(ft->u - (rt->u + 0.1)) < 1e-4 && std::fabs(ft->v - rt->v) < 1e-4);
    CHECK(raw.origin == fixed.origin);
}

TEST(pipeline_applies_the_per_eye_calibration_and_its_conversion) {
    const auto raw = first_samples(fast_cfg(), 3);
    // a calibration measured with FLIP_X: eye A becomes the LEFT eye, and its map shifts u by +0.05
    pibridge::eyemath::FrameConv fx;
    fx.flip_x = true;
    auto cal = pibridge::eyemath::Calibration::identity(fx, 0.064);
    cal.eyes[0].map.cx[0] = 0.05;
    cal.filter.enabled = false;
    auto cfg = fast_cfg();
    cfg.calibration = cal;
    // an old correction file is ignored when a calibration is present
    auto old = pibridge::eyemath::Correction::identity();
    old.cx[0] = 1.0;
    cfg.correction = old;
    const auto out = first_samples(cfg, 3);
    CHECK(!raw.empty() && !out.empty());
    const auto& r = raw[0].eyes[1]; // default conversion: A is the right eye
    const auto& o = out[0].eyes[0]; // calibration conversion: A is the left eye
    CHECK(o.direction_valid && !out[0].eyes[1].direction_valid);
    const auto rt = pibridge::eyemath::tangent_of({-double(r.direction[0]), double(r.direction[1]), double(r.direction[2])});
    const auto ot = pibridge::eyemath::tangent_of({double(o.direction[0]), double(o.direction[1]), double(o.direction[2])});
    CHECK(rt && ot);
    CHECK(std::fabs(ot->u - (rt->u + 0.05)) < 1e-5 && std::fabs(ot->v - rt->v) < 1e-5);
    CHECK((o.origin == pibridge::eyemath::Vec3f{-0.032F, 0.0F, 0.0F}));
    CHECK(out[0].gaze.has_value());
    CHECK(out[0].gaze->eyes == 0b01);
    CHECK((out[0].gaze->origin == pibridge::eyemath::Vec3f{0.0F, 0.0F, 0.0F}));
}

TEST(drift_hook_moves_the_calibrated_gaze_towards_the_target) {
    auto cal = pibridge::eyemath::Calibration::identity(pibridge::eyemath::FrameConv{}, 0.064);
    cal.filter.enabled = false;
    auto cfg = fast_cfg();
    cfg.calibration = cal;
    auto tr = Tracker::start_with(cfg, streaming_chips(), quiet);
    CHECK(wait_for([&] { return tr->latest().has_value(); }, 10s));
    const auto s = *tr->latest();
    CHECK(s.gaze.has_value());
    const auto& g = *s.gaze;
    // the wearer was looking 2 degrees to the right of where the gaze is
    const auto t = pibridge::eyemath::tangent_of({double(g.direction[0]), double(g.direction[1]), double(g.direction[2])});
    CHECK(t.has_value());
    const auto truth = pibridge::eyemath::dir_of(t->u + std::tan(2.0 * std::numbers::pi / 180.0), t->v);
    pibridge::eyemath::DriftParams p;
    p.gain = 1.0;
    const auto used = tr->observe_target(s.host_ts_ns, truth, p);
    CHECK(used && *used == 1);
    const auto d = tr->drift();
    CHECK(d.offset[1].u > 0.03 && d.offset[1].u < 0.04);
    CHECK((d.offset[0] == pibridge::eyemath::Tangent{0.0, 0.0}));
    tr->stop();
    // without a calibration the hook refuses
    auto none = Tracker::start_with(
        fast_cfg(),
        [](const Config&, std::stop_token) -> std::expected<std::unique_ptr<Transport>, std::string> {
            return std::unexpected(std::string("none"));
        },
        quiet);
    CHECK(!none->observe_target(0, {0.0, 0.0, -1.0}, p).has_value());
    none->stop();
}

TEST(recovery_resets_after_two_silent_failures_and_spaces_resets) {
    Config cfg;
    cfg.reset_min_interval = 60s;
    Recovery r;
    const auto t0 = Clock::now();
    CHECK(!r.after_link(Failure::NoResponse, false, t0, cfg));
    CHECK(r.after_link(Failure::NoResponse, false, t0, cfg));
    CHECK(r.resets == 1);
    // two more silent failures within the minimum interval: no second reset yet
    CHECK(!r.after_link(Failure::Stall, false, t0 + 5s, cfg));
    CHECK(!r.after_link(Failure::NoResponse, false, t0 + 10s, cfg));
    CHECK(r.after_link(Failure::NoResponse, false, t0 + 70s, cfg));
    // an I/O error (device gone) or a clean stop clears the count
    Recovery r2;
    CHECK(!r2.after_link(Failure::NoResponse, false, t0, cfg));
    CHECK(!r2.after_link(Failure::Io, false, t0, cfg));
    CHECK(!r2.after_link(Failure::NoResponse, false, t0, cfg));
    CHECK(!r2.after_link(std::nullopt, true, t0, cfg));
    CHECK(!r2.after_link(Failure::NoResponse, false, t0, cfg));
    // a link that streamed and then stalled counts once
    Recovery r3;
    CHECK(!r3.after_link(Failure::Stall, true, t0, cfg));
    CHECK(r3.after_link(Failure::Stall, true, t0, cfg));
    // disabled
    Config off;
    off.usb_reset = false;
    Recovery r4;
    for (int i = 0; i < 5; i++) {
        CHECK(!r4.after_link(Failure::NoResponse, false, t0, off));
    }
}

namespace {

/// A chip that ignores 0x547 (the 14 Oct wedge) until it is USB-reset.
struct WedgedUntilReset final : Device {
    std::shared_ptr<std::atomic<int>> resets = std::make_shared<std::atomic<int>>(0);
    std::shared_ptr<std::atomic<int>> opens = std::make_shared<std::atomic<int>>(0);
    std::shared_ptr<std::atomic<int>> silents = std::make_shared<std::atomic<int>>(0);
    std::expected<std::unique_ptr<Transport>, std::string> open(const Config&, std::stop_token) override {
        opens->fetch_add(1);
        auto c = std::make_unique<fake::FakeChip>(chip());
        if (resets->load() == 0) {
            c->silent.push_back(0x547);
        }
        return c;
    }
    std::expected<std::string, std::string> reset(const Config&) override {
        resets->fetch_add(1);
        return std::string("reset");
    }
    std::optional<std::string> silent(const Config&, Clock::time_point) override {
        silents->fetch_add(1);
        return std::nullopt;
    }
};

} // namespace

TEST(silent_chip_is_usb_reset_and_streams_again) {
    auto cfg = fast_cfg();
    cfg.response_wait = 150ms;
    cfg.reset_min_interval = Duration::zero();
    auto dev = std::make_unique<WedgedUntilReset>();
    const auto resets = dev->resets;
    const auto opens = dev->opens;
    const auto silents = dev->silents;
    auto tr = Tracker::start(cfg, std::move(dev), quiet);
    CHECK(wait_for([&] { return tr->latest().has_value(); }, 20s)); // streams after the reset
    CHECK(resets->load() == 1); // exactly one reset
    CHECK(opens->load() >= 3);  // two silent attempts, then the good one
    CHECK(silents->load() >= 2); // the device heard of each silent link (eye-module enable chance)
    const auto st = tr->stats();
    CHECK(st.resets == 1);
    CHECK(st.reconnects >= 2);
    // the samples arrive in order and none is lost between polls
    const auto all = tr->samples_since(std::nullopt);
    CHECK(!all.empty());
    const auto last = all.back().seq;
    std::this_thread::sleep_for(100ms);
    const auto more = tr->samples_since(last);
    for (std::size_t i = 0; i < more.size(); i++) {
        CHECK(more[i].seq > last);
        if (i > 0) {
            CHECK(more[i].seq == more[i - 1].seq + 1);
        }
    }
    tr->stop();
}

TEST(tracker_reconnects_when_the_device_goes_away) {
    auto opens = std::make_shared<std::atomic<int>>(0);
    auto tr = Tracker::start_with(
        fast_cfg(),
        [opens](const Config&, std::stop_token) -> std::expected<std::unique_ptr<Transport>, std::string> {
            if (opens->fetch_add(1) == 0) {
                return std::unexpected(std::string("no EyeChip yet"));
            }
            return std::make_unique<fake::FakeChip>(chip());
        },
        quiet);
    CHECK(wait_for([&] { return tr->latest().has_value(); }, 10s));
    CHECK(opens->load() >= 2);
    tr->stop();
}

// ---------------------------------------------------------------------------------------------- C++ only

TEST(a_bad_status_ends_the_link_with_status) {
    auto cfg = fast_cfg();
    Pipeline pipe(cfg, {});
    auto c = chip();
    c.status_override[0x532] = 7;
    std::stop_source stop;
    const auto r = run_link(c, cfg, stop.get_token(), pipe, {}, {});
    CHECK(!r.has_value() && r.error().kind == Failure::Status);
    CHECK(!r.has_value() && r.error().msg.find("status 7") != std::string::npos);
    CHECK(!has(sent_ops(c), 0x4ce)); // never subscribed, so no unsubscribe
}

TEST(a_silent_request_is_no_response) {
    auto cfg = fast_cfg();
    cfg.response_wait = 100ms;
    Pipeline pipe(cfg, {});
    auto c = chip();
    c.silent.push_back(0x547);
    std::stop_source stop;
    const auto r = run_link(c, cfg, stop.get_token(), pipe, {}, {});
    CHECK(!r.has_value() && r.error().kind == Failure::NoResponse);
    CHECK(is_silent(Failure::NoResponse) && is_silent(Failure::Stall) && !is_silent(Failure::Io));
}

TEST(the_users_profile_replaces_the_lists_upload_and_unpins_the_guard) {
    auto steps = frames::parse(frames::base_list());
    CHECK(steps.has_value());
    std::optional<Bytes> body;
    for (const auto& st : *steps) {
        if (const auto* t = std::get_if<frames::Ttp>(&st.action); t && t->op == frames::CAL_OP) {
            const auto m = frames::ttp_info(t->frames);
            CHECK(m.has_value());
            if (const auto b = frames::profile_body(m->payload)) {
                body = Bytes(b->begin(), b->end());
            }
        }
    }
    CHECK(body.has_value() && frames::check_profile(*body).has_value());
    // another user's profile: same layout, other bytes (the pinned guard would refuse it)
    body->back() ^= 0x5a;
    auto cfg = fast_cfg();
    cfg.profile = std::make_shared<const Bytes>(*body);
    std::stop_source stop;
    Pipeline pipe(cfg, [&](const pibridge::eyemath::Sample& s) {
        if (s.seq >= 3) {
            stop.request_stop();
        }
    });
    auto c = chip(frames::CAL_OP);
    std::vector<std::string> log;
    CHECK(run_link(c, cfg, stop.get_token(), pipe, {}, [&](std::string_view m) { log.emplace_back(m); }).has_value());
    bool uploaded = false;
    for (const auto& m : sent_messages(c)) {
        if (m.header.id == frames::CAL_OP) {
            uploaded = m.payload == frames::profile_payload(*body);
        }
    }
    CHECK(uploaded);
    CHECK(std::ranges::any_of(log, [](const std::string& l) { return l.starts_with("eye profile: uploading"); }));
}

TEST(an_own_start_list_is_used_and_its_upload_accepted) {
    auto cfg = fast_cfg();
    cfg.frames = std::make_shared<const std::string>(frames::base_list());
    cfg.with_cal = true;
    std::stop_source stop;
    Pipeline pipe(cfg, [&](const pibridge::eyemath::Sample& s) {
        if (s.seq >= 3) {
            stop.request_stop();
        }
    });
    auto c = chip(frames::CAL_OP);
    CHECK(run_link(c, cfg, stop.get_token(), pipe, {}, {}).has_value());
    CHECK(has(sent_ops(c), 0x456));
    // a list that is not a list stops the link before anything is sent
    cfg.frames = std::make_shared<const std::string>("STEP 1 | X | nonsense\nBULK_OUT ep=0x05 zz\n");
    auto c2 = chip();
    const auto r = run_link(c2, cfg, stop.get_token(), pipe, {}, {});
    CHECK(!r.has_value() && r.error().kind == Failure::Other);
    CHECK(c2.sent.empty());
}

TEST(config_from_env_reads_the_eye_variables) {
    const auto dir = testutil::temp_dir("live-env");
    const auto list = dir / "eye-start.txt";
    testutil::write_bytes(list, std::span(reinterpret_cast<const std::uint8_t*>(frames::base_list().data()),
                                          frames::base_list().size()));
    ::setenv("PIBRIDGE_EYE_FRAMES", list.c_str(), 1);
    ::setenv("PIBRIDGE_EYE_PROFILE", (dir / "missing.bin").c_str(), 1);
    ::setenv("PIBRIDGE_EYE_POWER_ON", "0", 1);
    ::setenv("PIBRIDGE_EYE_USB_RESET", "off", 1);
    ::setenv("PIBRIDGE_EYE_LATENCY_MS", " 1.5 ", 1);
    ::setenv("PIBRIDGE_EYE_CAL", "yes", 1);
    ::setenv("PIBRIDGE_EYE_FLIP_X", "1", 1);
    ::setenv("PIBRIDGE_EYE_FILTER", "one-euro", 1);
    std::vector<std::string> log;
    const auto c = Config::from_env([&](std::string_view m) { log.emplace_back(m); });
    CHECK(c.frames && *c.frames == frames::base_list());
    CHECK(!c.profile);  // a missing profile is skipped quietly
    CHECK(log.empty()); // no calibration under the temporary HOME, nothing wrong
    CHECK(!c.power_on && !c.usb_reset && c.with_cal && !c.skip_auth);
    CHECK(c.latency_ns == 1'500'000);
    CHECK(c.conv.flip_x && c.conv.flip_z);
    CHECK(c.filter && c.filter->enabled);
    CHECK(!c.calibration && c.calibration_path.empty());
    // a broken profile is reported and left out
    const auto bad = dir / "bad.bin";
    testutil::write_bytes(bad, std::vector<std::uint8_t>{1, 2, 3});
    ::setenv("PIBRIDGE_EYE_PROFILE", bad.c_str(), 1);
    ::setenv("PIBRIDGE_EYE_FILTER", "none", 1);
    log.clear();
    const auto c2 = Config::from_env([&](std::string_view m) { log.emplace_back(m); });
    CHECK(!c2.profile && log.size() == 1 && log[0].starts_with("PIBRIDGE_EYE_PROFILE"));
    CHECK(c2.filter && !c2.filter->enabled);
    for (const char* v : {"PIBRIDGE_EYE_FRAMES", "PIBRIDGE_EYE_PROFILE", "PIBRIDGE_EYE_POWER_ON", "PIBRIDGE_EYE_USB_RESET",
                          "PIBRIDGE_EYE_LATENCY_MS", "PIBRIDGE_EYE_CAL", "PIBRIDGE_EYE_FLIP_X", "PIBRIDGE_EYE_FILTER"}) {
        ::unsetenv(v);
    }
    // defaults
    const auto d = Config::from_env(quiet);
    CHECK(d.power_on && d.usb_reset && !d.with_cal && d.skip_vendor_write && !d.frames && !d.filter);
    CHECK(d.stall == 3s && d.retry == 2s && d.timeout == 2s && d.response_wait == 3s && d.history == 256);
    CHECK(d.reset_after == 2 && d.reset_min_interval == 60s && d.pace);
    fs::remove_all(dir);
}

TEST(a_calibration_file_under_home_is_loaded_with_its_path) {
    auto cal = pibridge::eyemath::Calibration::identity(pibridge::eyemath::FrameConv{}, 0.064);
    cal.conv.flip_x = true;
    const auto p = g_home / ".config/pibridge/eye-calibration.toml";
    fs::create_directories(p.parent_path());
    CHECK(cal.save(p).has_value());
    std::vector<std::string> log;
    const auto c = Config::from_env([&](std::string_view m) { log.emplace_back(m); });
    CHECK(c.calibration.has_value() && c.calibration_path == p);
    CHECK(c.conv.flip_x); // the calibration's conversion wins
    CHECK(log.size() == 1 && log[0].find("per-eye calibration") == 0 &&
          log[0].find("instead of the PIBRIDGE_EYE_FLIP_*") != std::string::npos);
    fs::remove(p);
}

TEST(to_shm_carries_the_published_fields) {
    const auto v = first_samples(fast_cfg(), 2);
    CHECK(!v.empty());
    const auto o = to_shm(v[0]);
    CHECK(o.seq == v[0].seq && o.host_ts_ns == v[0].host_ts_ns);
    CHECK(o.eyes[1].direction_valid && o.eyes[1].origin_valid && !o.eyes[0].direction_valid);
    CHECK(o.eyes[1].direction == v[0].eyes[1].direction && o.eyes[1].origin == v[0].eyes[1].origin);
    CHECK(o.gaze && o.gaze->direction == v[0].gaze->direction);
    CHECK(state_name(State::Streaming) == "Streaming" && static_cast<int>(State::Stopped) == 4);
}

namespace {

void put(const fs::path& p, const std::string& s) {
    fs::create_directories(p.parent_path());
    testutil::write_bytes(p, std::span(reinterpret_cast<const std::uint8_t*>(s.data()), s.size()));
}

void usb_device(const fs::path& root, const char* name, const char* vid, const char* pid, const char* devnum) {
    put(root / name / "idVendor", std::string(vid) + "\n");
    put(root / name / "idProduct", std::string(pid) + "\n");
    put(root / name / "busnum", "3\n");
    put(root / name / "devnum", std::string(devnum) + "\n");
}

struct RecordingHid final : power::HeadsetCommands {
    std::vector<std::string> sent;
    bool fail = false;
    Result<void> send(std::string_view id) override {
        sent.emplace_back(id);
        if (fail) {
            return std::unexpected(Error::make(Errc::not_found, "no hidraw node for the headset"));
        }
        return {};
    }
};

} // namespace

TEST(real_device_powers_the_eye_module_once_per_enumeration) {
    const auto dir = testutil::temp_dir("live-sysfs");
    const auto usb = dir / "usb";
    fs::create_directories(usb);
    auto hid = std::make_shared<RecordingHid>();
    RealDevice dev(hid, usb, dir / "dev");
    const Config cfg = fast_cfg();
    const auto t0 = Clock::now();
    // no headset: nothing to do
    CHECK(!dev.absent(cfg, t0));
    CHECK(!dev.open(cfg, {}).has_value());
    CHECK(dev.open(cfg, {}).error() == "no EyeChip (2104:0561) on the USB bus");
    usb_device(usb, "3-1", "34a4", "0040", "9");
    CHECK(!dev.absent(cfg, t0));       // the 5 s grace starts
    CHECK(!dev.absent(cfg, t0 + 3s));
    const auto m = dev.absent(cfg, t0 + 6s);
    CHECK(m && m->find("headset 3-1 devnum 9 present") != std::string::npos);
    CHECK(hid->sent == std::vector<std::string>{"eye_power_on"});
    CHECK(!dev.absent(cfg, t0 + 20s)); // once per enumeration
    CHECK(hid->sent.size() == 1);
    // switched off: never
    auto off = cfg;
    off.power_on = false;
    RealDevice dev2(hid, usb, dir / "dev");
    CHECK(!dev2.absent(off, t0) && !dev2.absent(off, t0 + 10s));
    // no command sink: never
    RealDevice dev3(nullptr, usb, dir / "dev");
    CHECK(!dev3.absent(cfg, t0) && !dev3.absent(cfg, t0 + 10s) && !dev3.silent(cfg, t0));
    CHECK(hid->sent.size() == 1);
    fs::remove_all(dir);
}

TEST(real_device_sends_the_enable_after_a_silent_link_at_most_every_5_s) {
    const auto dir = testutil::temp_dir("live-silent");
    const auto usb = dir / "usb";
    fs::create_directories(usb);
    auto hid = std::make_shared<RecordingHid>();
    RealDevice dev(hid, usb, dir / "dev");
    const Config cfg = fast_cfg();
    const auto t0 = Clock::now();
    CHECK(!dev.silent(cfg, t0)); // no headset
    usb_device(usb, "3-1", "34a4", "0040", "9");
    const auto m = dev.silent(cfg, t0);
    CHECK(m && m->starts_with("chip silent (headset 3-1 devnum 9)"));
    CHECK(!dev.silent(cfg, t0 + 2s));
    CHECK(dev.silent(cfg, t0 + 6s).has_value());
    CHECK(hid->sent.size() == 2);
    hid->fail = true;
    const auto e = dev.silent(cfg, t0 + 12s);
    CHECK(e && e->starts_with("eye module enable failed: no hidraw node"));
    // a fixed usbfs node (PIBRIDGE_EYE_USB) means no power control
    auto fixed = cfg;
    fixed.usb_device = dir / "dev/003/005";
    CHECK(!dev.silent(fixed, t0 + 30s));
    fs::remove_all(dir);
}

TEST(real_device_does_not_take_a_chip_a_kernel_driver_owns) {
    const auto dir = testutil::temp_dir("live-driver");
    const auto usb = dir / "usb";
    usb_device(usb, "3-2", "2104", "0561", "12");
    fs::create_directories(usb / "3-2/3-2:1.0");
    fs::create_directories(dir / "drivers/usbhid");
    fs::create_directory_symlink(dir / "drivers/usbhid", usb / "3-2/3-2:1.0/driver");
    RealDevice dev(nullptr, usb, dir / "dev");
    const auto r = dev.open(fast_cfg(), {});
    CHECK(!r.has_value() && r.error() == "kernel driver 'usbhid' is bound to the EyeChip interface; not detaching it");
    // without the driver the node is opened: here a temporary path that does not exist
    fs::remove(usb / "3-2/3-2:1.0/driver");
    const auto r2 = dev.open(fast_cfg(), {});
    CHECK(!r2.has_value() && r2.error().starts_with("open " + (dir / "dev/003/012").string()));
    fs::remove_all(dir);
}

// ------------------------------------------------------------------------------------------------ original Crystal

// One gaze notification of the original Crystal's tracker 2104:0220 as captured (Windows, take ipd-buttons of
// 2026-10-22, device address 97, 20:29:24.386, headset worn): a whole 1242-byte IN transfer, both eyes tracked.
constexpr std::string_view P3B_GAZE_HEX =
    "01000000da0400000000005300000000000000000000050500000000000004ba0000050000000400230bb8050000000400020bb90200"
    "0000040000000106000000080000000002ee1f76050000000400020bb9020000000400000002050000000400031f420300000004001a"
    "a8ed0300000004fffaaa0f0300000004ffe2e1a2050000000400020bb902000000040000000302000000040000000105000000040002"
    "0bb9020000000400000004050000000400031f42030000000400002d840300000004ffffeef303000000040000fb5705000000040002"
    "0bb9020000000400000005020000000400000001050000000400020bb902000000040000000603000000040004607705000000040002"
    "0bb9020000000400000007030000000400010000050000000400020bb9020000000400000010050000000400021f4004000000080000"
    "020e58c400000400000008000002f0e5d00000050000000400020bb9020000000400000017050000000400021f430300000004000093"
    "a603000000040000b9cf050000000400020bb9020000000400000016020000000400000001050000000400020bb90200000004000000"
    "1e020000000400000001050000000400020bb902000000040000001f020000000400000000050000000400020bb90200000004000000"
    "08050000000400031f420300000004ffe420d30300000004fffb28ba0300000004ffe3fd5a050000000400020bb90200000004000000"
    "09020000000400000001050000000400020bb902000000040000000a050000000400031f420300000004ffffe1750300000004000003"
    "f103000000040000fe23050000000400020bb902000000040000000b020000000400000001050000000400020bb90200000004000000"
    "0c030000000400047a5d050000000400020bb902000000040000000d030000000400010000050000000400020bb90200000004000000"
    "11050000000400021f40040000000800000214035400000400000008000002d428a40000050000000400020bb9020000000400000019"
    "050000000400021f4303000000040000742e03000000040000b16b050000000400020bb9020000000400000018020000000400000001"
    "050000000400020bb9020000000400000020020000000400000001050000000400020bb9020000000400000021020000000400000000"
    "050000000400020bb902000000040000000e0200000004000fffff050000000400020bb902000000040000000f020000000400001589"
    "050000000400020bb9020000000400000012050000000400031f4203000000040000078b0300000004fffff96003000000040000ffcd"
    "050000000400020bb9020000000400000013020000000400000001050000000400020bb9020000000400000014050000000400031f42"
    "0300000004ffff5f200300000004fffae9720300000004ffe36f9c050000000400020bb9020000000400000015020000000400000001"
    "050000000400020bb902000000040000001a020000000400000000050000000400020bb902000000040000001b020000000400000000"
    "050000000400020bb902000000040000001d0300000004ffff0000050000000400020bb902000000040000001c020000000400000000"
    "050000000400020bb90200000004000000220300000004ffff0000050000000400020bb9020000000400000023020000000400000000";

TEST(p3b_gaze_notification_of_the_capture_decodes_with_both_eyes) {
    Bytes b;
    for (std::size_t i = 0; i + 1 < P3B_GAZE_HEX.size(); i += 2) {
        b.push_back(static_cast<std::uint8_t>(std::stoi(std::string(P3B_GAZE_HEX.substr(i, 2)), nullptr, 16)));
    }
    CHECK(b.size() == 1242);
    std::vector<pibridge::eyemath::Sample> got;
    Pipeline pipe(fast_cfg(), [&](const pibridge::eyemath::Sample& s) { got.push_back(s); });
    CHECK(pipe.push(b).empty());
    CHECK(pipe.parse_errors == 0 && pipe.framing_errors == 0 && got.size() == 1);
    if (got.size() != 1) {
        return;
    }
    // default conversion: z flipped, mm to m, device eye A (x > 0) is the right eye
    const auto& s = got[0];
    const auto near = [](float x, double want) { return std::fabs(x - want) < 1e-4; };
    const auto& r = s.eyes[1];
    const auto& l = s.eyes[0];
    CHECK(r.origin_valid && r.direction_valid && l.origin_valid && l.direction_valid);
    CHECK(near(r.origin[0], 0.026665) && near(r.origin[1], -0.005331) && near(r.origin[2], 0.029120));
    CHECK(near(r.direction[0], 0.177795) && near(r.direction[1], -0.066605) && near(r.direction[2], -0.981796));
    CHECK(near(l.origin[0], -0.027872) && near(l.direction[0], -0.119308) && near(l.direction[2], -0.992722));
    CHECK(std::fabs(r.pupil_mm - 4.3768f) < 1e-3f);
    CHECK(r.openness == 1.0f && l.openness == 1.0f); // provided by this chip (-1 on the Dream Air)
    CHECK(s.gaze && s.gaze->eyes == 0b11);
    CHECK(s.ipd_mm == -1.0f); // field 34 is -1 here (to_shm / the shared memory carry 0: unknown)
    CHECK(s.device_ts_us == 0x2ee1f76);
}

TEST(a_p3b_chip_runs_on_its_own_start_list) {
    auto cfg = fast_cfg();
    cfg.frames = std::make_shared<const std::string>(frames::base_list()); // a 2104:0561 list (e.g. the Super's file)
    cfg.profile = std::make_shared<const Bytes>(Bytes(64, 1));
    cfg.with_cal = true;
    const auto p = for_chip(cfg, replay::EYE_PID_P3B);
    CHECK(p.frames && *p.frames == frames::p3b_list() && !p.profile && !p.with_cal);
    const auto same = for_chip(cfg, replay::EYE_PID);
    CHECK(same.frames == cfg.frames && same.profile == cfg.profile && same.with_cal);

    // a link on it: exactly the captured requests, the subscribe, the lens configuration, then the unsubscribe
    std::stop_source stop;
    std::size_t n = 0;
    Pipeline pipe(p, [&](const pibridge::eyemath::Sample&) {
        if (++n >= 20) {
            stop.request_stop();
        }
    });
    auto c = chip();
    CHECK(run_link(c, p, stop.get_token(), pipe, {}, {}).has_value());
    const std::vector<std::uint32_t> want = {0x3e8, 0x640, 0x58c, 0x532, 0x546, 0x547, 0x5d2, 0x686,
                                             0x4b0, 0x596, 0x578, 0x672, 0x4c4, 0x5be, 0x4ce};
    CHECK(sent_ops(c) == want);
    CHECK(n >= 20);
    std::size_t controls = 0;
    for (const auto& x : c.sent) {
        if (const auto* ctl = std::get_if<fake::SentControl>(&x)) {
            controls++;
            CHECK(ctl->setup[0] != 0x41 || ctl->setup[1] != 48); // the vendor write stays skipped by default
        }
    }
    CHECK(controls == 11); // 0xc1/48, 9 x 0xc1/70, 0x41/65
}

TEST(the_tracker_uses_the_p3b_list_for_a_2104_0220) {
    auto cfg = fast_cfg();
    std::mutex mu;
    std::vector<std::string> log;
    std::shared_ptr<fake::FakeChip> last;
    auto tr = Tracker::start_with(
        cfg,
        [&](const Config&, std::stop_token) -> std::expected<std::unique_ptr<Transport>, std::string> {
            return std::make_unique<fake::FakeChip>(chip());
        },
        [&](std::string_view m) {
            std::lock_guard l(mu);
            log.emplace_back(m);
        },
        replay::EYE_PID_P3B);
    CHECK(wait_for([&] { return tr->state() == State::Streaming; }, 5s));
    tr->stop();
    std::lock_guard l(mu);
    CHECK(std::ranges::find(log, "original Crystal eye tracker (2104:0220): its own captured start list") != log.end());
}

TEST(real_device_takes_a_2104_0220_only_when_asked) {
    const auto dir = testutil::temp_dir("live-p3b");
    const auto usb = dir / "usb";
    usb_device(usb, "3-4", "2104", "0220", "12");
    RealDevice dev(nullptr, usb, dir / "dev");
    auto cfg = fast_cfg();
    const auto r = dev.open(cfg, {});
    CHECK(!r.has_value() && r.error() == "no EyeChip (2104:0561) on the USB bus");
    cfg.p3b = true;
    // found: the node is opened (here a temporary path that does not exist)
    const auto r2 = dev.open(cfg, {});
    CHECK(!r2.has_value() && r2.error().starts_with("open " + (dir / "dev/003/012").string()));
    CHECK(dev.pid() == replay::EYE_PID_P3B);
    // a 2104:0561 is preferred
    usb_device(usb, "3-2", "2104", "0561", "13");
    const auto r3 = dev.open(cfg, {});
    CHECK(!r3.has_value() && r3.error().starts_with("open " + (dir / "dev/003/013").string()));
    CHECK(dev.pid() == replay::EYE_PID);
    fs::remove_all(usb / "3-2");
    fs::remove_all(usb / "3-4");
    CHECK(dev.open(cfg, {}).error() == "no eye chip (2104:0561 or 2104:0220) on the USB bus");
    fs::remove_all(dir);
}

TEST(the_tracker_logs_each_line_once_and_keeps_the_last) {
    std::vector<std::string> log;
    std::mutex mu;
    auto tr = Tracker::start_with(
        fast_cfg(),
        [](const Config&, std::stop_token) -> std::expected<std::unique_ptr<Transport>, std::string> {
            return std::unexpected(std::string("no EyeChip (2104:0561) on the USB bus"));
        },
        [&](std::string_view m) {
            std::lock_guard l(mu);
            log.emplace_back(m);
        });
    std::this_thread::sleep_for(150ms);
    CHECK(tr->state() == State::Searching);
    tr->stop();
    std::lock_guard l(mu);
    CHECK(log.size() == 1);
    CHECK(tr->last_message() == "no EyeChip (2104:0561) on the USB bus");
}

PIBRIDGE_TEST_MAIN()
