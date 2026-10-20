// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/replay.rs tests (MIT OR Apache-2.0).
#include "pibridge/eye/replay.h"

#include "pibridge_test.h"
#include "testutil.h"

using namespace pibridge::eye::replay;
using namespace testutil;

namespace {

std::vector<UsbPacket> session(bool truncate_big) {
    std::vector<UsbPacket> v;
    double t = 1.0;
    const auto tick = [&] {
        t += 0.001;
        return t;
    };
    v.push_back(ctrl(tick(), {0x41, 0x30, 0, 0, 0, 0, 4, 0}, {1, 2, 3, 4}));
    v.push_back(ctrl(tick(), {0xc1, 0x46, 1, 0, 0, 0, 8, 0}, {}));
    v.push_back(ctrl(tick(), {0x80, 0x06, 0, 3, 9, 4, 0x34, 0}, {})); // standard: ignored
    const auto r1 = request(1, 0x3e8, 3);
    for (const auto& f : r1.to_frames(Direction::Out, 4087)) {
        v.push_back(bulk_out(tick(), f, f.size()));
    }
    v.push_back(bulk_in(tick(), response(1, 0x3e8).to_frames(Direction::In, 4087)[0]));
    // big message: 3 frames of 100 body bytes
    const auto big = request(2, 0x456, 60);
    const auto frames = big.to_frames(Direction::Out, 100);
    CHECK(frames.size() > 2);
    for (const auto& f : frames) {
        if (truncate_big) {
            v.push_back(bulk_out(tick(), std::span<const std::uint8_t>(f).first(std::min<std::size_t>(40, f.size())),
                                 f.size()));
        } else {
            v.push_back(bulk_out(tick(), f, f.size()));
        }
    }
    v.push_back(bulk_in(tick(), response(2, 0x456).to_frames(Direction::In, 4087)[0]));
    v.push_back(bulk_in(tick(), notification().to_frames(Direction::In, 4087)[0]));
    // after the stream start: must not be in the plan
    const auto late = request(3, 0x44c, 0);
    v.push_back(bulk_out(tick(), late.to_frames(Direction::Out, 4087)[0], late.body().size() + 8));
    return v;
}

} // namespace

TEST(complete_session_gives_complete_plan) {
    const auto pk = session(false);
    CHECK(find_eye_devices(pk) == std::vector<std::uint16_t>{30});
    const auto plan = build_plan(30, pk);
    CHECK(plan.steps.size() == 4);
    CHECK(plan.is_complete());
    CHECK(plan.requests().size() == 2);
    CHECK(plan.orphan_frames == 0);
    CHECK(plan.stream_start.has_value());
    const auto* r = std::get_if<Request>(&plan.steps[3].kind);
    CHECK(r != nullptr);
    if (r != nullptr) {
        CHECK(r->seq == 2 && r->opcode == 0x456);
        CHECK(r->frames.size() > 2);
        CHECK(r->expect == std::optional<Expect>(Expect{1, 0}));
    }
    CHECK(std::holds_alternative<ControlOut>(plan.steps[0].kind));
    CHECK(std::holds_alternative<ControlIn>(plan.steps[1].kind));
    CHECK(plan.auth_steps().empty());
}

TEST(truncated_frames_make_the_plan_incomplete) {
    const auto pk = session(true);
    const auto plan = build_plan(30, pk);
    CHECK(plan.steps.size() == 4);
    CHECK(!plan.is_complete());
    const auto inc = plan.incomplete_steps();
    CHECK(inc.size() == 1);
    CHECK(inc[0]->missing_bytes > 0 && inc[0]->total_bytes > inc[0]->missing_bytes);
    CHECK(plan.missing_bytes() == inc[0]->missing_bytes);
    CHECK(plan.describe().find("INCOMPLETE") != std::string::npos);
    // all frames of the multi-frame message were grouped even though the capture cut them
    const auto* r = std::get_if<Request>(&inc[0]->kind);
    CHECK(r != nullptr && r->frames.size() > 2);
}

TEST(empty_input_gives_an_empty_incomplete_plan) {
    const auto plan = build_plan(30, {});
    CHECK(!plan.is_complete());
    CHECK(!plan.stream_start);
}

TEST(auth_steps_are_flagged) {
    std::vector<UsbPacket> pk;
    const std::uint32_t ops[] = {0x76c, 0x777, 0x4c4};
    for (std::uint32_t i = 0; i < 3; i++) {
        const auto m = request(i + 1, ops[i], 1);
        const auto f = m.to_frames(Direction::Out, 4087);
        pk.push_back(bulk_out(1.0 + i * 0.001, f[0], f[0].size()));
    }
    CHECK(build_plan(30, pk).auth_steps().size() == 2);
}

TEST(foreign_devices_are_not_detected) {
    const std::vector<UsbPacket> p = {
        UsbPacket::build(0.0, 2, 9, 0x05, Transfer::Bulk, false, std::nullopt, Bytes(64, 1), 64)};
    CHECK(find_eye_devices(p).empty());
}

// From tests/props.rs plan_builder_is_total_and_consistent, with a fixed pseudo-random sample.
TEST(plan_builder_is_total_and_consistent) {
    std::uint32_t x = 777;
    const auto rnd = [&] {
        x = x * 1664525u + 1013904223u;
        return x >> 24;
    };
    for (int round = 0; round < 100; round++) {
        std::vector<UsbPacket> pk;
        for (std::uint32_t i = 0; i < rnd() % 60; i++) {
            Bytes d(rnd() % 80);
            for (auto& c : d) {
                c = static_cast<std::uint8_t>(rnd());
            }
            if (rnd() % 3 == 0 && d.size() >= 8) {
                d[0] = 0;
                d[1] = d[2] = d[3] = 0;
            }
            const std::uint8_t eps[] = {0x05, 0x83, 0x00, 0x80};
            const Transfer trs[] = {Transfer::Bulk, Transfer::Control, Transfer::Interrupt};
            const auto tr = trs[rnd() % 3];
            pk.push_back(UsbPacket::build(i * 0.001, 2, 30, eps[rnd() % 4], tr, rnd() % 2 == 0,
                                          tr == Transfer::Control ? std::optional<std::uint8_t>(0) : std::nullopt, d,
                                          static_cast<std::uint32_t>(d.size() + rnd() % 3)));
        }
        const auto plan = build_plan(30, pk);
        std::uint64_t missing = 0;
        for (const auto& s : plan.steps) {
            CHECK(s.missing_bytes <= s.total_bytes);
            missing += s.missing_bytes;
        }
        CHECK(missing == plan.missing_bytes());
        CHECK(plan.is_complete() == (!plan.steps.empty() && missing == 0));
        (void)plan.describe();
    }
}

PIBRIDGE_TEST_MAIN()
