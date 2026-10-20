// SPDX-License-Identifier: MIT
// Controller pairing and unpairing (pairing.h): log verdicts, the request file, the exact frames, both allowlists,
// and the frames over a session against the in-process fake headset. Nothing touches a real device.
#include "common.h"
#include "pibridge_test.h"

#include "pibridge/hmd/descriptor.h"
#include "pibridge/hmd/fake.h"
#include "pibridge/hmd/pairing.h"
#include "pibridge/hmd/session.h"

#include <fstream>
#include <unistd.h>

using namespace pibridge::hmd;
using namespace pibridge::hmd::pairing;
using namespace std::chrono;
using testutil::bytes;

static std::shared_ptr<const Allowlist> allow(const char* id) {
    const auto d = Descriptor::load(std::filesystem::path(PIBRIDGE_SOURCE_DEVICES_DIR) / (std::string(id) + ".toml"));
    if (!d) {
        std::abort();
    }
    return std::make_shared<const Allowlist>(d->allowlist);
}

static std::string checked(const Allowlist& al, const Report& r) {
    const auto c = al.check(0, r);
    return c ? (*c)->id : "<refused>";
}

TEST(log_lines_give_the_verdict) {
    CHECK(parse_log("[I] eSpiCmdStartPair slot 2") == LogEvent::Started);
    CHECK(parse_log("Pairing device = 2") == LogEvent::Started);
    CHECK(parse_log("pair finished") == LogEvent::Success);
    CHECK(parse_log("radio_pair_success_process ok") == LogEvent::Success);
    CHECK(parse_log("radio_pair_cancel_or_timeout_process") == LogEvent::TimeoutOrCancel);
    CHECK(!parse_log("imu ok"));
}

TEST(sides_and_slots) {
    CHECK(slot_of("left") == std::uint8_t{2});
    CHECK(slot_of("right") == std::uint8_t{3});
    CHECK(!slot_of("both"));
    CHECK(side_of(2) == "left" && side_of(3) == "right");
}

TEST(request_lines_round_trip_and_junk_is_refused) {
    const Request r{42, Op::Unpair, 3};
    CHECK(format_request(r) == "42 unpair right\n");
    CHECK(parse_request(format_request(r)) == r);
    CHECK(parse_request("7 pair left") == (Request{7, Op::Pair, 2}));
    for (const char* bad : {"", "pair left", "0 pair left", "x pair left", "1 pair middle", "1 reset left",
                            "1 pair left extra", "-1 pair left"}) {
        CHECK(!parse_request(bad));
    }
}

TEST(take_request_reads_and_removes) {
    const auto d = std::filesystem::temp_directory_path() / ("pibridge-test-pairing-" + std::to_string(::getpid()));
    std::filesystem::create_directories(d);
    const auto p = d / REQUEST_FILE;
    CHECK((take_request(p).value_or(Request{}) == std::optional<Request>{}));
    std::ofstream(p) << "5 pair right\n";
    const auto t = take_request(p);
    CHECK(t.has_value() && *t == (Request{5, Op::Pair, 3}));
    CHECK(!std::filesystem::exists(p));
    std::ofstream(p) << "junk";
    CHECK(!take_request(p).has_value());
    CHECK(!std::filesystem::exists(p));
    std::filesystem::remove_all(d);
}

TEST(frames_are_the_captured_bytes) {
    CHECK(start_pair(2) == bytes({0x58, 0x04, 0x04, 0x00, 0x02}));
    CHECK(start_pair(3) == bytes({0x58, 0x04, 0x04, 0x00, 0x03}));
    CHECK(unpair(2) == bytes({0x58, 0x03, 0x05, 0x02}));
    CHECK(unpair(3) == bytes({0x58, 0x03, 0x05, 0x03}));
}

TEST(both_models_allow_pair_and_unpair_for_slots_2_and_3_only) {
    for (const char* id : {"dream-air", "crystal-super"}) {
        const auto al = allow(id);
        for (const std::uint8_t s : {std::uint8_t{2}, std::uint8_t{3}}) {
            CHECK(checked(*al, start_pair(s)) == "start_pair");
            CHECK(checked(*al, unpair(s)) == "unpair");
        }
        for (const std::uint8_t s : {std::uint8_t{0}, std::uint8_t{1}, std::uint8_t{4}}) {
            CHECK(checked(*al, start_pair(s)) == "<refused>");
            CHECK(checked(*al, unpair(s)) == "<refused>");
        }
        // StartPair goes out at most once per 30 s through one gate (never retried)
        std::vector<std::uint8_t> out;
        Gate g(*al, 0, vector_writer(out));
        CHECK(g.send(start_pair(2)).has_value());
        const auto again = g.send(start_pair(2));
        CHECK(!again && again.error().refusal() && again.error().refusal()->kind == Refused::Kind::TooSoon);
        CHECK(out.size() == 65);
        const auto spec = al->get("start_pair");
        CHECK(spec && spec->policy == Policy::OncePerRequest && !spec->approved.empty());
    }
}

TEST(crystal_session_sends_pair_and_gets_the_unpair_reply) {
    auto [be, dev] = fake::pair();
    dev.set_model(CRYSTAL_SUPER);
    Config cfg;
    cfg.allowlist = allow("crystal-super");
    cfg.keepalive = false;
    auto s = Session::open_with_backend(std::move(cfg), std::move(be), std::make_unique<NoopHandler>());
    CHECK(s.has_value());
    if (!s) {
        return;
    }
    CHECK((*s)->wait_connected(milliseconds(2000)).has_value());
    CHECK(dev.wait_host(milliseconds(1000)));
    (void)dev.drain_commands(milliseconds(100)); // the wake
    CHECK((*s)->send_report(start_pair(3)).has_value());
    const auto c = dev.recv_command(milliseconds(500));
    CHECK(c && *c == start_pair(3));
    std::thread reply([dev] {
        if (const auto u = dev.recv_command(milliseconds(1000)); u && (*u)[2] == 0x05) {
            dev.send_report(0, bytes({0x4c, 0x04, 0x05, (*u)[3], 0x01, 0x07}));
        }
    });
    const auto r = (*s)->request_with(unpair(2), milliseconds(1000), [](const Report& d) {
        return d[2] == cmd::UNPAIR && d[3] == 2;
    });
    reply.join();
    CHECK(r.has_value() && (*r)[4] == 0x01);
}

PIBRIDGE_TEST_MAIN()
