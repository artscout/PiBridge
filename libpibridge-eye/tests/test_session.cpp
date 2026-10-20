// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/session.rs tests and tests/replay_fake.rs (MIT OR Apache-2.0).
// The replay_fake.rs cases that drive the real cold-start capture need private lab data and are skipped; the same
// behaviour is checked here on a plan made from the public base list.
#include "pibridge/eye/session.h"

#include "pibridge/eye/fake.h"
#include "pibridge/eye/frames.h"

#include "pibridge_test.h"
#include "testutil.h"

using namespace pibridge::eye;
using namespace pibridge::eye::session;
using namespace testutil;
using namespace std::chrono_literals;

namespace {

ReplayOptions quick() {
    return ReplayOptions{false, 300ms};
}

// The public base list as a capture of it would give (every byte complete).
replay::Plan base_plan() {
    const auto base = frames::parse(frames::base_list()).value();
    replay::Plan plan;
    plan.device = 7;
    for (std::size_t i = 0; i < base.size(); i++) {
        const auto& st = base[i];
        replay::StepKind kind;
        if (std::holds_alternative<frames::Enumeration>(st.action)) {
            continue;
        } else if (const auto* c = std::get_if<frames::Control>(&st.action)) {
            if ((c->setup[0] & 0x80) == 0) {
                kind = replay::ControlOut{c->setup, c->data};
            } else {
                kind = replay::ControlIn{c->setup};
            }
        } else {
            const auto& t = std::get<frames::Ttp>(st.action);
            kind = replay::Request{t.seq, t.op, 0, t.frames, replay::Expect{1, 0}};
        }
        plan.steps.push_back(replay::Step{i, static_cast<double>(i), std::move(kind), 0, 1});
    }
    return plan;
}

// What a faithful replay must send: control transfers and bulk frames of the plan, in order, nothing else.
std::vector<fake::Sent> expected_sent(const replay::Plan& plan, const std::vector<std::size_t>& skip) {
    std::vector<fake::Sent> v;
    for (std::size_t i = 0; i < plan.steps.size(); i++) {
        if (std::ranges::find(skip, i) != skip.end()) {
            continue;
        }
        const auto& k = plan.steps[i].kind;
        if (const auto* c = std::get_if<replay::ControlOut>(&k)) {
            v.emplace_back(fake::SentControl{c->setup, c->data});
        } else if (const auto* ci = std::get_if<replay::ControlIn>(&k)) {
            v.emplace_back(fake::SentControl{ci->setup, {}});
        } else {
            for (const auto& f : std::get<replay::Request>(k).frames) {
                v.emplace_back(fake::SentBulkOut{replay::EP_BULK_OUT, f});
            }
        }
    }
    return v;
}

std::size_t bulk_count(const fake::FakeChip& chip) {
    return static_cast<std::size_t>(
        std::ranges::count_if(chip.sent, [](const fake::Sent& s) { return std::holds_alternative<fake::SentBulkOut>(s); }));
}

// Transport failing at the k-th control or bulk OUT operation.
class Flaky final : public Transport {
public:
    fake::FakeChip inner;
    std::size_t left = 0;

    Result<Bytes> control(const Setup& s, ByteView o, Duration t) override {
        if (auto e = tick(); !e) {
            return std::unexpected(e.error());
        }
        return inner.control(s, o, t);
    }
    Result<void> bulk_out(std::uint8_t ep, ByteView d, Duration t) override {
        if (auto e = tick(); !e) {
            return e;
        }
        return inner.bulk_out(ep, d, t);
    }
    Result<std::optional<Bytes>> bulk_in(std::uint8_t ep, std::size_t m, Duration t) override {
        return inner.bulk_in(ep, m, t);
    }

private:
    Result<void> tick() {
        if (left == 0) {
            return std::unexpected(Error::os(32)); // EPIPE
        }
        left--;
        return {};
    }
};

} // namespace

TEST(captured_unsubscribe_is_byte_exact_with_the_capture) {
    // hex of the OUT transfer at t=10.5906 s of 20261013-124248-poweron-hub2.pcap, taken with an independent script
    const std::string hex = "0000000023000000000000510000001a00000000000004ce000000000000000b0000020000000400000505";
    Bytes want;
    for (std::size_t i = 0; i < hex.size(); i += 2) {
        want.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
    }
    CHECK(std::ranges::equal(CAPTURED_UNSUBSCRIBE_GAZE, want));
}

TEST(captured_unsubscribe_is_a_valid_request) {
    const auto w = ttp::Wrapper::parse(CAPTURED_UNSUBSCRIBE_GAZE);
    CHECK(w && w->dir == ttp::Direction::Out);
    CHECK(w->frame_len() == CAPTURED_UNSUBSCRIBE_GAZE.size());
    ttp::StreamDecoder dec(ttp::Direction::Out);
    auto r = dec.push(CAPTURED_UNSUBSCRIBE_GAZE);
    CHECK(r.size() == 1 && r[0].has_value());
    const auto& h = r[0]->header;
    CHECK(h.kind == ttp::Kind::Request && h.seq == 26 && h.id == OP_UNSUBSCRIBE && h.payload_len == 11);
    const auto items = r[0]->items().collect();
    CHECK(items.size() == 1);
    CHECK(items[0]->tag == tlv::tag::INT32);
    CHECK(std::ranges::equal(items[0]->value, Bytes{0, 0, 5, 5}));
}

TEST(full_replay_sends_exactly_the_plan_then_the_stream_starts) {
    skip_lab_data("full_replay_sends_exactly_the_plan_then_the_stream_starts");
}
TEST(incomplete_capture_is_refused_before_anything_is_sent) {
    skip_lab_data("incomplete_capture_is_refused_before_anything_is_sent");
}
TEST(allow_incomplete_skips_exactly_the_cut_steps) { skip_lab_data("allow_incomplete_skips_exactly_the_cut_steps"); }
TEST(an_unexpected_status_stops_the_replay_at_once) {
    skip_lab_data("an_unexpected_status_stops_the_replay_at_once");
}
TEST(a_silent_chip_gives_no_response_error_without_retry) {
    skip_lab_data("a_silent_chip_gives_no_response_error_without_retry");
}
TEST(a_usb_error_stops_the_replay_and_names_the_step) {
    skip_lab_data("a_usb_error_stops_the_replay_and_names_the_step");
}

TEST(base_replay_sends_exactly_the_plan_then_the_stream_starts) {
    const auto plan = base_plan();
    CHECK(plan.is_complete());
    fake::FakeChip chip;
    auto dec = in_decoder();
    std::vector<std::string> log;
    const auto rep = run_plan(chip, plan, quick(), dec, [&](std::string_view l) { log.emplace_back(l); });
    CHECK(rep.has_value());
    CHECK(rep->sent == 23);
    CHECK(rep->skipped.empty());
    CHECK(rep->exchanges.size() == 19);
    CHECK(std::ranges::all_of(rep->exchanges, [](const Exchange& e) { return e.status == 1; }));
    CHECK(rep->control_in.size() == 2);
    CHECK(chip.sent == expected_sent(plan, {})); // nothing but the plan may reach the device
    CHECK(chip.streaming);                       // the profile upload (0x456) starts the stream in the fake
    // 200 ms of stream: ~24 notifications at 120.5 Hz
    std::optional<Duration> first;
    const auto s = read_stream(chip, dec, 200ms, [&](const ttp::Message&, Duration at) -> std::expected<void, std::string> {
        if (!first) {
            first = at;
        }
        return {};
    });
    CHECK(s.has_value());
    CHECK(s->parse_errors == 0 && s->framing_errors == 0);
    CHECK(s->samples >= 15 && s->samples <= 26);
    CHECK(s->first_sample_after && *s->first_sample_after < 60ms);
    const auto hz = s->host_rate_hz();
    CHECK(hz && *hz > 95.0 && *hz < 130.0);
    CHECK(std::ranges::any_of(log, [](const std::string& l) { return l.find("op 0x456") != std::string::npos; }));
}

TEST(an_incomplete_plan_is_refused_before_anything_is_sent) {
    auto plan = base_plan();
    plan.steps[3].missing_bytes = 5;
    plan.steps.back().missing_bytes = 7;
    fake::FakeChip chip;
    auto dec = in_decoder();
    const auto r = run_plan(chip, plan, quick(), dec);
    CHECK(!r && r.error().kind == ReplayError::Kind::Incomplete);
    CHECK(r.error().incomplete.size() == 2);
    CHECK(r.error().incomplete[0].step == 3 && r.error().incomplete[0].missing == 5);
    CHECK(r.error().incomplete[1].what.find("op=0x456") != std::string::npos);
    CHECK(chip.sent.empty()); // a refused plan must not touch the device
    // allow_incomplete skips exactly the cut steps; without the profile upload the fake does not stream
    fake::FakeChip chip2;
    auto dec2 = in_decoder();
    auto opts = quick();
    opts.allow_incomplete = true;
    const auto rep = run_plan(chip2, plan, opts, dec2);
    CHECK(rep.has_value());
    CHECK(rep->skipped == (std::vector<std::size_t>{3, plan.steps.size() - 1}));
    CHECK(rep->sent == plan.steps.size() - 2);
    CHECK(chip2.sent == expected_sent(plan, {3, plan.steps.size() - 1}));
    CHECK(!chip2.streaming);
}

TEST(an_unexpected_status_on_the_base_plan_stops_at_once) {
    const auto plan = base_plan();
    fake::FakeChip chip;
    chip.status_override[0x852] = 7;
    auto dec = in_decoder();
    const auto r = run_plan(chip, plan, quick(), dec);
    CHECK(!r && r.error().kind == ReplayError::Kind::BadStatus);
    CHECK(r.error().seq == 5 && r.error().opcode == 0x852 && r.error().got == 7 && r.error().expected == 1);
    // requests seq 1, 2 and the failing 5 were sent, nothing after it
    CHECK(bulk_count(chip) == 3);
}

TEST(a_silent_chip_gives_no_response_without_retry) {
    const auto plan = base_plan();
    fake::FakeChip chip;
    chip.silent.push_back(0x640);
    auto dec = in_decoder();
    auto opts = quick();
    opts.timeout = 120ms;
    const auto t0 = Clock::now();
    const auto r = run_plan(chip, plan, opts, dec);
    CHECK(!r && r.error().kind == ReplayError::Kind::NoResponse);
    CHECK(r.error().seq == 2 && r.error().opcode == 0x640);
    CHECK(Clock::now() - t0 < 2s);
    const auto sends = std::ranges::count_if(chip.sent, [](const fake::Sent& s) {
        const auto* b = std::get_if<fake::SentBulkOut>(&s);
        return b != nullptr && b->data.size() > 23 && b->data[23] == 0x40 && b->data[22] == 0x06;
    });
    CHECK(sends == 1); // the request must be sent once, never retried
    CHECK(r.error().to_string().find("no response to request seq 2 opcode 0x640") != std::string::npos);
}

TEST(a_usb_error_on_the_base_plan_stops_and_names_the_step) {
    const auto plan = base_plan();
    Flaky chip;
    chip.left = 6;
    auto dec = in_decoder();
    const auto r = run_plan(chip, plan, quick(), dec);
    CHECK(!r && r.error().kind == ReplayError::Kind::Io);
    CHECK(r.error().step == 6);
    CHECK(r.error().io.os_errno == 32);
    CHECK(chip.inner.sent.size() == 6);
}

TEST(an_empty_plan_is_refused) {
    fake::FakeChip chip;
    auto dec = in_decoder();
    const auto r = run_plan(chip, replay::Plan{}, quick(), dec);
    CHECK(!r && r.error().kind == ReplayError::Kind::EmptyPlan);
}

TEST(attaching_to_a_running_stream_needs_no_traffic_to_the_device) {
    fake::FakeChip chip;
    chip.streaming = true;
    auto dec = in_decoder();
    const auto s = read_stream(chip, dec, 100ms);
    CHECK(s.has_value());
    CHECK(s->samples >= 8);
    CHECK(chip.sent.empty());
}

TEST(a_chip_that_never_streams_is_reported_as_such) {
    fake::FakeChip chip;
    auto dec = in_decoder();
    const auto s = read_stream(chip, dec, 60ms);
    CHECK(s.has_value());
    CHECK(s->samples == 0);
    CHECK(s->timeouts > 0 && !s->first_sample_after && !s->host_rate_hz());
}

TEST(decode_errors_are_counted_not_fatal) {
    fake::FakeChip chip;
    chip.streaming = true;
    auto dec = in_decoder();
    int n = 0;
    const auto s = read_stream(chip, dec, 60ms, [&](const ttp::Message&, Duration) -> std::expected<void, std::string> {
        if (n++ % 2 == 0) {
            return std::unexpected(std::string("bad"));
        }
        return {};
    });
    CHECK(s.has_value() && s->parse_errors > 0 && s->samples > 0);
    CHECK(s->first_parse_error == std::optional<std::string>("bad"));
}

PIBRIDGE_TEST_MAIN()
