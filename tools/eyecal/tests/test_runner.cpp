// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/xrapp.rs tests (MIT OR Apache-2.0): the keys, the raw CSV and the gaze pose's
// forward direction. New: the whole on-chip calibration through the Runner on a live tracker over FakeChip (the
// procedure with short timings, real threads, the profile and calibration files in a temporary folder): saved and
// kept with B at the end, put back when the run ends without it.
#include "runner.h"

#include "report.h"

#include "pibridge/eye/fake.h"
#include "pibridge/eye/frames.h"
#include "pibridge/eye/session.h"
#include "pibridge/eyemath/calib.h"
#include "pibridge/eyemath/clock.h"

#include "pibridge_test.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>

#include <unistd.h>

using namespace pibridge::eyecal;
using namespace std::chrono_literals;
namespace fs = std::filesystem;
namespace cal = pibridge::eye::calibration;
namespace live = pibridge::eye::live;
using pibridge::eye::Bytes;

namespace {

fs::path temp_dir(const std::string& name) {
    const auto d = fs::temp_directory_path() / ("pibridge-eyecal-" + name + "-" + std::to_string(::getpid()));
    fs::remove_all(d);
    fs::create_directories(d);
    return d;
}

std::string read(const fs::path& p) {
    std::ifstream f(p, std::ios::binary);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

Bytes body(std::size_t n, std::uint8_t fill) {
    Bytes b(n, fill);
    const auto lead = static_cast<std::uint32_t>(n - 168);
    for (int i = 0; i < 4; i++) {
        b[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(lead >> (8 * i));
    }
    return b;
}

struct Fwd final : pibridge::eye::Transport {
    std::shared_ptr<pibridge::eye::fake::FakeChip> c;
    pibridge::eye::Result<Bytes> control(const pibridge::eye::Setup& s, pibridge::eye::ByteView o, pibridge::eye::Duration t) override {
        return c->control(s, o, t);
    }
    pibridge::eye::Result<void> bulk_out(std::uint8_t ep, pibridge::eye::ByteView d, pibridge::eye::Duration t) override {
        return c->bulk_out(ep, d, t);
    }
    pibridge::eye::Result<std::optional<Bytes>> bulk_in(std::uint8_t ep, std::size_t m, pibridge::eye::Duration t) override {
        return c->bulk_in(ep, m, t);
    }
};

struct Rig {
    fs::path dir;
    std::shared_ptr<pibridge::eye::fake::FakeChip> fc = std::make_shared<pibridge::eye::fake::FakeChip>();
    std::shared_ptr<cal::Link> link = std::make_shared<cal::Link>();
    Bytes profile = body(33887, 0x42);
    std::vector<std::string> log;
    std::unique_ptr<Runner> runner;

    explicit Rig(const std::string& name) : dir(temp_dir(name)) {
        fc->payload = live::fake_payload;
        fc->start_on = pibridge::eye::session::OP_SUBSCRIBE;
        fc->reply_payload[cal::OP_DOWNLOAD] = pibridge::eye::frames::profile_payload(profile);
        fc->in_max_body = 16384 - 8;
        fs::create_directories(dir / "config");
        std::ofstream(dir / "config" / "eye-profile-crystal-super.bin") << "the old profile";
        Settings s = Settings{}.chip();
        s.attract_s = 0.05;
        s.min_hold_s = 0.05;
        s.max_hold_s = 0.2;
        s.collect_s = 0.3;
        RunFiles files{dir / "results", "20261019-120000", dir / "config" / "eye-calibration.toml",
                       dir / "config" / "eye-profile-crystal-super.bin", true};
        auto chip = fc;
        auto l = link;
        Runner::StartTracker start = [chip, l](const pibridge::eyemath::FrameConv& conv) {
            live::Config cfg;
            cfg.pace = false;
            cfg.retry = 20ms;
            cfg.history = 1024;
            cfg.conv = conv;
            auto inner = std::make_unique<live::FnDevice>(
                [chip](const live::Config&, std::stop_token) -> std::expected<std::unique_ptr<pibridge::eye::Transport>, std::string> {
                    auto f = std::make_unique<Fwd>();
                    f->c = chip;
                    return f;
                });
            return live::Tracker::start(cfg, std::make_unique<ChannelDevice>(std::move(inner), l), [](std::string_view) {});
        };
        runner = std::make_unique<Runner>(Procedure(s), files, link, start, [this](const std::string& m) { log.push_back(m); });
        runner->begin();
    }
    ~Rig() {
        runner.reset();
        fs::remove_all(dir);
    }

    /// Frames until `done` or the time is up; A once the chip is ready.
    template <class F>
    bool drive(F done, std::chrono::seconds max, std::vector<Cmd> at_start = {Cmd::Next}) {
        const auto t0 = std::chrono::steady_clock::now();
        const std::array<Vec3, 2> eyes = {Vec3{-0.032, 0.0, 0.0}, Vec3{0.032, 0.0, 0.0}};
        while (std::chrono::steady_clock::now() - t0 < max) {
            std::vector<Cmd> cmds;
            if (!at_start.empty() && runner->procedure().chip_ready()) {
                cmds = at_start;
                at_start.clear();
            }
            const double now = static_cast<double>(pibridge::eyemath::monotonic_ns()) * 1e-9;
            runner->step(now, std::array<double, 4>{0.0, 0.0, 0.0, 1.0}, eyes, std::nullopt, cmds);
            if (done()) {
                return true;
            }
            std::this_thread::sleep_for(8ms);
        }
        return false;
    }

    std::vector<std::uint32_t> cal_ops() const {
        pibridge::eye::ttp::StreamDecoder dec(pibridge::eye::ttp::Direction::Out);
        std::vector<std::uint32_t> ops;
        for (const auto& s : fc->sent) {
            if (const auto* b = std::get_if<pibridge::eye::fake::SentBulkOut>(&s)) {
                for (auto& m : dec.push(b->data)) {
                    if (m && cal::is_calibration_op(m->header.id)) {
                        ops.push_back(m->header.id);
                    }
                }
            }
        }
        return ops;
    }
};

} // namespace

TEST(forward_of_identity_and_yaw) {
    const auto f = forward_of({0.0, 0.0, 0.0, 1.0});
    CHECK(std::fabs(f[2] + 1.0) < 1e-9);
    const double h = std::sqrt(0.5);
    const auto y = forward_of({0.0, -h, 0.0, h});
    CHECK(std::fabs(y[0] - 1.0) < 1e-6);
    const auto p = forward_of({h, 0.0, 0.0, h});
    CHECK(std::fabs(p[1] - 1.0) < 1e-6);
}

TEST(keys_and_csv) {
    CHECK(key_command(' ') == std::optional<std::optional<Cmd>>(Cmd::Next));
    CHECK(key_command('y') == std::optional<std::optional<Cmd>>(Cmd::Accept));
    CHECK(key_command('b') == std::optional<std::optional<Cmd>>(Cmd::Redo));
    CHECK(key_command('q') == std::optional<std::optional<Cmd>>(std::optional<Cmd>{}));
    CHECK(!key_command('z').has_value());
    const RawSample s{1.5, {}};
    const auto row = raw_csv_row(s, "Intro", std::nullopt);
    CHECK(std::ranges::count(row, ',') == std::ranges::count(raw_csv_header(), ','));
    RawSample t{2.0, {RawEye{{0.0, 0.0, -1.0}, {-0.03, 0.0, 0.02}, 3.25}, std::nullopt}};
    const auto r2 = raw_csv_row(t, "Chip:0:+0,+0:Hold", std::array<double, 4>{0.0, 0.0, 0.0, 1.0});
    CHECK(r2.starts_with("2.000000,Chip:0:+0,+0:Hold,1,0.000000,0.000000,-1.000000,-0.03000,0.00000,0.02000,3.250,0,"));
}

TEST(a_chip_calibration_through_the_runner_is_saved_and_kept_with_b) {
    Rig rig("runner-keep");
    const bool ended = rig.drive([&] { return rig.runner->procedure().phase.is(Phase::Kind::Results); }, 60s);
    CHECK(ended);
    const auto& p = rig.runner->procedure();
    CHECK(!p.chip_failed && p.chip_profile.has_value());
    const fs::path prof = rig.dir / "config" / "eye-profile-crystal-super.bin";
    // the new profile is in place (not kept yet), the old one is backed up
    CHECK(read(prof) == std::string(rig.profile.begin(), rig.profile.end()));
    CHECK(read(rig.dir / "config" / "eye-profile-crystal-super.bin.bak-20261019-120000") == "the old profile");
    // the chip got the captured sequence
    CHECK(rig.cal_ops() == (std::vector<std::uint32_t>{0x3f2, 0x407, 0x407, 0x407, 0x407, 0x407, 0x42e, 0x44c, 0x3fc, 0x3f2, 0x407,
                                                       0x407, 0x407, 0x407, 0x42e, 0x44c, 0x3fc, 0x44c}));
    // B: save, back to the check grid (the profile is kept at once, the host map written)
    rig.runner->step(static_cast<double>(pibridge::eyemath::monotonic_ns()) * 1e-9, std::nullopt, {Vec3{-0.032, 0.0, 0.0}, Vec3{0.032, 0.0, 0.0}}, std::nullopt, {Cmd::Redo});
    CHECK(!rig.runner->quit_requested() && rig.runner->procedure().phase.is(Phase::Kind::GridSaved));
    CHECK(rig.runner->profile_store().kept() && fs::exists(rig.dir / "config" / "eye-calibration.toml"));
    CHECK(rig.runner->procedure().s.view.has_value()); // the grid's gaze goes through the new host map
    // B in the grid: exit
    rig.runner->step(static_cast<double>(pibridge::eyemath::monotonic_ns()) * 1e-9, std::nullopt, {Vec3{-0.032, 0.0, 0.0}, Vec3{0.032, 0.0, 0.0}}, std::nullopt, {Cmd::Redo});
    CHECK(rig.runner->quit_requested());
    rig.runner->finish();
    CHECK(rig.runner->profile_store().kept());
    CHECK(read(prof) == std::string(rig.profile.begin(), rig.profile.end()));
    const auto c = pibridge::eyemath::Calibration::load(rig.dir / "config" / "eye-calibration.toml");
    CHECK(c.has_value() && c->eyes[0].map.is_identity() && c->eyes[1].map.is_identity());
    CHECK(fs::exists(rig.dir / "results" / "eyecal-20261019-120000.json"));
    CHECK(fs::exists(rig.dir / "results" / "eyecal-20261019-120000-raw.csv"));
    CHECK(std::ranges::any_of(rig.log, [](const std::string& l) { return l.find("the new profile stays") != std::string::npos; }));
}

TEST(a_chip_calibration_left_without_b_puts_the_old_profile_back) {
    Rig rig("runner-back");
    const bool saved = rig.drive([&] { return rig.runner->profile_store().saved(); }, 60s);
    CHECK(saved);
    rig.runner->finish(); // e.g. the headset's session ended
    CHECK(read(rig.dir / "config" / "eye-profile-crystal-super.bin") == "the old profile");
    CHECK(!fs::exists(rig.dir / "config" / "eye-calibration.toml"));
    CHECK(std::ranges::any_of(rig.log, [](const std::string& l) { return l.starts_with("not saved: the previous profile is back"); }));
}

TEST(stopping_mid_run_finishes_the_open_pass) {
    Rig rig("runner-stop");
    // into the first pass (the first target answered), then B
    const bool in_pass = rig.drive([&] { return rig.runner->procedure().chip_res.size() >= 1; }, 30s);
    CHECK(in_pass);
    rig.runner->step(static_cast<double>(pibridge::eyemath::monotonic_ns()) * 1e-9, std::nullopt, {Vec3{-0.032, 0.0, 0.0}, Vec3{0.032, 0.0, 0.0}}, std::nullopt, {Cmd::Redo});
    CHECK(!rig.runner->quit_requested() && rig.runner->procedure().phase.is(Phase::Kind::Grid)); // back to the grid
    // the finish of the open pass is answered while the grid shows; then B exits
    const bool answered = rig.drive([&] { return rig.runner->procedure().chip_pending() == 0; }, 10s, {});
    CHECK(answered);
    rig.runner->step(static_cast<double>(pibridge::eyemath::monotonic_ns()) * 1e-9, std::nullopt, {Vec3{-0.032, 0.0, 0.0}, Vec3{0.032, 0.0, 0.0}}, std::nullopt, {Cmd::Redo});
    CHECK(rig.runner->quit_requested());
    rig.runner->finish();
    const auto ops = rig.cal_ops();
    CHECK(!ops.empty() && ops.back() == cal::OP_FINISH);
    CHECK(std::ranges::count(ops, cal::OP_DOWNLOAD) == 0);
    CHECK(read(rig.dir / "config" / "eye-profile-crystal-super.bin") == "the old profile");
}

PIBRIDGE_TEST_MAIN()
