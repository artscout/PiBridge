// SPDX-License-Identifier: MIT
// The tool's side of the chip: the profile file (0600, backup, a bad profile keeps the old file, rollback, keep), the
// one-owner check on temporary /proc trees, and the driver and channel device on a live tracker over FakeChip. No
// device node is opened and no file of the user's is touched (temporary folders only).
#include "chip.h"

#include "pibridge/eye/fake.h"
#include "pibridge/eye/frames.h"
#include "pibridge/eye/session.h"

#include "pibridge_test.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>

#include <sys/stat.h>
#include <unistd.h>

using namespace pibridge::eyecal;
using namespace std::chrono_literals;
using namespace std::string_literals;
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

void write(const fs::path& p, const std::string& s) {
    fs::create_directories(p.parent_path());
    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    f << s;
}

unsigned mode_of(const fs::path& p) {
    struct stat st{};
    ::stat(p.c_str(), &st);
    return st.st_mode & 0777;
}

Bytes body(std::size_t n, std::uint8_t fill) {
    Bytes b(n, fill);
    const auto lead = static_cast<std::uint32_t>(n - 168);
    for (int i = 0; i < 4; i++) {
        b[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(lead >> (8 * i));
    }
    return b;
}

std::string str(const Bytes& b) { return std::string(b.begin(), b.end()); }

} // namespace

TEST(the_new_profile_is_saved_privately_with_a_backup_and_put_back_when_not_kept) {
    const auto d = temp_dir("profile");
    const auto p = d / "pibridge" / "eye-profile-crystal-super.bin";
    write(p, "old profile");
    ::chmod(p.c_str(), 0644);
    ProfileStore s(p);
    const Bytes a = body(1000, 1);
    auto r = s.save(a, "20261019-120000");
    CHECK(r.has_value() && r->has_value());
    const fs::path bak = d / "pibridge" / "eye-profile-crystal-super.bin.bak-20261019-120000";
    CHECK(r && *r && **r == bak);
    CHECK(read(p) == str(a) && mode_of(p) == 0600);
    CHECK(read(bak) == "old profile" && mode_of(bak) == 0600);
    // a second save of the same run (calibrate again) replaces the new one, the backup stays the original
    const Bytes b2 = body(1200, 2);
    r = s.save(b2, "20261019-120500");
    CHECK(r.has_value() && !r->has_value());
    CHECK(read(p) == str(b2) && read(bak) == "old profile");
    // not kept: the old one comes back (the copy stays)
    auto rb = s.rollback();
    CHECK(rb.has_value() && read(p) == "old profile" && fs::exists(bak));
    CHECK(!s.saved());
    // kept: rollback does nothing
    ProfileStore k(p);
    CHECK(k.save(a, "20261019-130000").has_value());
    k.keep();
    CHECK(k.rollback().has_value() && read(p) == str(a));
    // no temporary file is left behind
    for (const auto& e : fs::directory_iterator(d / "pibridge")) {
        CHECK(e.path().filename().string().find(".tmp-") == std::string::npos);
    }
    fs::remove_all(d);
}

TEST(after_a_kept_save_a_new_calibration_rolls_back_to_the_saved_profile) {
    // the check grid's flow: save (B), calibrate again from the grid (A), leave without saving: the profile of the
    // save comes back, not the one from before the tool
    const auto d = temp_dir("profile-again");
    const auto p = d / "eye-profile-crystal-super.bin";
    write(p, "old profile");
    ProfileStore s(p);
    const Bytes a = body(1000, 1);
    const Bytes b2 = body(1200, 2);
    CHECK(s.save(a, "20261019-120000").has_value());
    s.keep();
    CHECK(s.kept() && read(p) == str(a));
    auto r = s.save(b2, "20261019-120000");
    CHECK(r.has_value() && r->has_value() && !s.kept() && s.saved());
    const fs::path first = d / "eye-profile-crystal-super.bin.bak-20261019-120000";
    const fs::path second = d / "eye-profile-crystal-super.bin.bak-20261019-120000-1";
    CHECK(r && *r && **r == second);
    CHECK(read(first) == "old profile" && read(second) == str(a) && read(p) == str(b2));
    auto rb = s.rollback();
    CHECK(rb.has_value() && read(p) == str(a) && !s.saved());
    // saved and kept again: the second calibration stays
    ProfileStore k(p);
    CHECK(k.save(a, "t").has_value());
    k.keep();
    CHECK(k.save(b2, "t").has_value());
    k.keep();
    CHECK(k.rollback().has_value() && read(p) == str(b2));
    fs::remove_all(d);
}

TEST(a_bad_profile_keeps_the_old_file_and_a_new_file_is_removed_on_rollback) {
    const auto d = temp_dir("profile-bad");
    const auto p = d / "eye-profile-crystal-super.bin";
    write(p, "old");
    ProfileStore s(p);
    Bytes bad = body(1000, 1);
    bad[0] ^= 1;
    auto r = s.save(bad, "x");
    CHECK(!r.has_value() && r.error().find("not valid") != std::string::npos);
    CHECK(read(p) == "old" && !s.saved());
    CHECK(fs::directory_iterator(d) != fs::directory_iterator() && std::distance(fs::directory_iterator(d), {}) == 1);
    // there was no profile before: rollback removes the new one
    const auto q = d / "new" / "eye-profile-crystal-super.bin";
    ProfileStore n(q);
    CHECK(n.save(body(500, 3), "y").has_value());
    CHECK(fs::exists(q) && !n.backup().has_value());
    CHECK(n.rollback().has_value() && !fs::exists(q));
    fs::remove_all(d);
}

TEST(the_chip_owner_is_found_by_name) {
    const auto d = temp_dir("proc");
    CHECK(!chip_owner(d).has_value());
    write(d / "101" / "comm", "bash\n");
    write(d / "self" / "comm", "pibridge-eyed\n"); // not a pid directory
    CHECK(!chip_owner(d).has_value());
    write(d / "202" / "comm", "pibridge-eyed\n");
    CHECK(chip_owner(d) == std::optional<std::string>("pibridge-eyed"));
    fs::remove_all(d / "202");
    write(d / "303" / "comm", "omarchyvr-eyed\n");
    CHECK(chip_owner(d) == std::optional<std::string>("omarchyvr-eyed"));
    fs::remove_all(d / "303");
    write(d / "404" / "comm", "omarchyvr\n");
    write(d / "404" / "cmdline", "/home/x/.local/bin/omarchyvr\0keepalived\0"s);
    CHECK(!chip_owner(d).has_value());
    write(d / "404" / "cmdline", "/home/x/.local/bin/omarchyvr\0eyed\0--follow\0vrserver\0"s);
    CHECK(chip_owner(d) == std::optional<std::string>("omarchyvr eyed"));
    CHECK(owner_message("pibridge-eyed").find("stop pibridge-eyed first") != std::string::npos);
    fs::remove_all(d);
}

TEST(the_driver_carries_requests_over_a_live_tracker_session) {
    auto fc = std::make_shared<pibridge::eye::fake::FakeChip>();
    fc->payload = live::fake_payload;
    fc->start_on = pibridge::eye::session::OP_SUBSCRIBE;
    fc->reply_payload[cal::OP_DOWNLOAD] = pibridge::eye::frames::profile_payload(body(2000, 9));
    auto link = std::make_shared<cal::Link>();
    live::Config cfg;
    cfg.pace = false;
    cfg.retry = 20ms;
    // the channel device wraps whatever the inner device opens
    auto inner = std::make_unique<live::FnDevice>(
        [fc](const live::Config&, std::stop_token) -> std::expected<std::unique_ptr<pibridge::eye::Transport>, std::string> {
            // a transport that forwards to the shared fake (the device hands out a fresh one per link)
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
            auto f = std::make_unique<Fwd>();
            f->c = fc;
            return f;
        });
    ChipDriver drv(link);
    CHECK(!drv.ready());
    // before a link: a request fails at once and comes back through poll
    drv.request(cal::Step{cal::OP_START, std::nullopt, 1});
    auto r0 = drv.poll();
    CHECK(r0.has_value() && !r0->second.has_value() && r0->first == 0);
    auto tr = live::Tracker::start(cfg, std::make_unique<ChannelDevice>(std::move(inner), link), [](std::string_view) {});
    const auto t0 = std::chrono::steady_clock::now();
    while (!drv.ready() && std::chrono::steady_clock::now() - t0 < 5s) {
        drv.follow(tr->state());
        std::this_thread::sleep_for(10ms);
    }
    CHECK(drv.ready());
    // the whole sequence through the driver, in order
    cal::Run run;
    while (const cal::Step* s = run.current()) {
        drv.request(*s);
        std::optional<ChipDriver::Result> r;
        const auto t1 = std::chrono::steady_clock::now();
        while (!(r = drv.poll()) && std::chrono::steady_clock::now() - t1 < 3s) {
            std::this_thread::sleep_for(5ms);
        }
        CHECK(r.has_value() && r->second.has_value());
        if (!r || !r->second) {
            break;
        }
        CHECK(run.answer(r->first, *r->second).has_value());
    }
    CHECK(run.done() && run.profile() && *run.profile() == body(2000, 9));
    CHECK(drv.outstanding() == 0);
    tr->stop();
    drv.follow(live::State::Stopped);
    CHECK(!drv.ready());
}

PIBRIDGE_TEST_MAIN()
