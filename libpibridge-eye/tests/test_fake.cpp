// SPDX-License-Identifier: MIT
// Tests of the fake chip itself (omarchyvr pimax-eye/src/fake.rs has none of its own; its behaviour is relied on by
// replay_fake.rs and live.rs, and checked here directly).
#include "pibridge/eye/fake.h"

#include "pibridge_test.h"
#include "testutil.h"

using namespace pibridge::eye;
using namespace pibridge::eye::fake;
using namespace std::chrono_literals;

namespace {
std::optional<ttp::Message> one_message(FakeChip& chip) {
    auto got = chip.bulk_in(0x83, 16384, 10ms);
    if (!got || !*got) {
        return std::nullopt;
    }
    ttp::StreamDecoder dec(ttp::Direction::In);
    auto r = dec.push(**got);
    if (r.size() != 1 || !r[0]) {
        return std::nullopt;
    }
    return *r[0];
}
} // namespace

TEST(answers_every_request_with_its_seq_and_status_1) {
    FakeChip chip;
    const auto req = testutil::request(7, 0x3e8, 2);
    CHECK(chip.bulk_out(0x05, testutil::concat(req.to_frames(ttp::Direction::Out, 4087)), 10ms).has_value());
    const auto m = one_message(chip);
    CHECK(m && m->header.kind == ttp::Kind::Response && m->header.seq == 7 && m->header.id == 0x3e8);
    CHECK(m->header.status == 1);
    CHECK(chip.sent.size() == 1);
}

TEST(status_override_silent_and_stream_control) {
    FakeChip chip;
    chip.status_override[0x640] = 9;
    chip.silent.push_back(0x532);
    chip.start_on = 0x4c4;
    const auto send = [&](std::uint32_t seq, std::uint32_t op) {
        CHECK(chip.bulk_out(0x05, testutil::request(seq, op, 0).to_frames(ttp::Direction::Out, 4087)[0], 10ms)
                  .has_value());
    };
    send(1, 0x640);
    auto m = one_message(chip);
    CHECK(m && m->header.status == 9);
    send(2, 0x532);
    CHECK(!one_message(chip)); // silent: nothing queued, not streaming
    send(3, 0x4c4);
    m = one_message(chip);
    CHECK(m && m->header.kind == ttp::Kind::Response);
    CHECK(chip.streaming);
    m = one_message(chip);
    CHECK(m && m->header.kind == ttp::Kind::Notification && m->header.id == ttp::STREAM_GAZE);
    CHECK(chip.emitted == 1);
    send(4, 0x4ce);
    CHECK(!chip.streaming);
}

TEST(control_in_returns_at_most_8_bytes_and_everything_is_recorded) {
    FakeChip chip;
    const auto r = chip.control({0xc1, 48, 0, 0, 0, 0, 0, 2}, {}, 10ms);
    CHECK(r && r->size() == 8);
    const auto o = chip.control({0x41, 65, 0, 0, 0, 0, 0, 0}, {}, 10ms);
    CHECK(o && o->empty());
    CHECK(chip.sent.size() == 2);
    CHECK(std::holds_alternative<SentControl>(chip.sent[0]));
}

TEST(garbage_on_the_out_pipe_is_invalid_data) {
    FakeChip chip;
    const auto r = chip.bulk_out(0x05, Bytes{1, 0, 0, 0, 8, 0, 0, 0}, 10ms);
    CHECK(!r && r.error().code == Errc::invalid_data);
}

TEST(the_payload_hook_shapes_the_notifications) {
    FakeChip chip;
    chip.streaming = true;
    std::vector<std::int64_t> ts;
    chip.payload = [&](std::uint32_t, std::int64_t t) {
        ts.push_back(t);
        return Bytes{0, 0, 9};
    };
    const auto a = one_message(chip);
    const auto b = one_message(chip);
    CHECK(a && b && a->payload == (Bytes{0, 0, 9}));
    CHECK(ts.size() == 2 && ts[1] - ts[0] == NOMINAL_PERIOD_US);
}

PIBRIDGE_TEST_MAIN()
