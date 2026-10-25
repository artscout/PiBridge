// SPDX-License-Identifier: MIT
// pibridge-roomsetup --recenter: the head ring reader (the layout of omarchyvr world.rs ring_bytes), the steps of the
// flow driven by scripted controller frames, the result text, the options. No runtime, no files of Monado.
#include "pibridge_test.h"
#include "options.h"
#include "recenter.h"

#include <cstring>

using namespace pibridge;
using namespace pibridge::roomsetup;

namespace {

// omarchyvr world.rs ring_bytes(pid, samples): the reference writer layout
std::string ringBytes(std::uint32_t pid, const std::vector<std::tuple<std::uint64_t, std::array<float, 7>, bool>>& samples,
                      std::uint32_t seq = 2) {
    std::string b(16 + 128 * 40, '\0');
    const auto put = [&b](std::size_t o, const void* p, std::size_t n) { std::memcpy(b.data() + o, p, n); };
    const std::uint32_t magic = 0x434E4150, version = 1;
    put(0, &magic, 4);
    put(4, &version, 4);
    put(8, &seq, 4);
    put(12, &pid, 4);
    for (std::size_t i = 0; i < samples.size(); i++) {
        const auto& [ts, p, valid] = samples[i];
        const std::size_t o = 16 + i * 40;
        put(o, &ts, 8);
        put(o + 8, p.data(), 28);
        const std::uint32_t f = valid ? 1 : 0;
        put(o + 36, &f, 4);
    }
    return b;
}

room::FrameInput frame(std::uint64_t left, std::uint64_t right, double rightY = 0.02) {
    room::FrameInput in;
    in.headValid = true;
    in.head.m = {{{1, 0, 0, 0}, {0, 1, 0, 1.7}, {0, 0, 1, 0}}};
    in.hands[0].valid = true;
    in.hands[0].buttons = left;
    in.hands[1].valid = true;
    in.hands[1].buttons = right;
    in.hands[1].grip.m[1][3] = rightY;
    return in;
}

constexpr std::uint64_t kTrig = 1ULL << room::bit::Trigger;
constexpr std::uint64_t kMenu = 1ULL << room::bit::Menu;

} // namespace

TEST(ring_newest_valid_sample) {
    const std::string b = ringBytes(4242, {{100, {1, 1.6f, 2, 0, 0, 0, 1}, true},
                                           {300, {0.5f, 1.7f, -1, 0, 0.3826834f, 0, 0.9238795f}, true},
                                           {400, {9, 9, 9, 0, 0, 0, 1}, false}}); // newest but not valid
    const auto s = parseRing(b);
    CHECK(s && s->pid == 4242 && s->tsNs == 300);
    CHECK(std::fabs(s->pose[0] - 0.5) < 1e-6 && std::fabs(s->pose[4] - 0.3826834) < 1e-6);
    CHECK(!parseRing(ringBytes(1, {{100, {}, true}}, 3)));  // odd seq: being written
    CHECK(!parseRing(b.substr(0, 100)));                     // too short
    CHECK(!parseRing(ringBytes(1, {})));                     // nothing valid
    std::string bad = b;
    bad[0] = 0;
    CHECK(!parseRing(bad));
    CHECK(!readRing("/nonexistent/pimax-anchor.head"));
}

TEST(flow_centre_then_floor_touch_or_keep) {
    const std::string ring = ringBytes(77, {{5, {0.5f, 1.7f, -1, 0, 0, 0, 1}, true}});
    const auto reader = [&ring] { return parseRing(ring); };
    {
        RecenterFlow f;
        CHECK(f.step() == RecenterFlow::Step::Centre && f.lines().front() == "RECENTER");
        f.frame(frame(0, 0), 0.0, reader);
        f.frame(frame(0, kTrig), 0.0, reader); // the trigger goes down: the hold ring starts filling
        CHECK(f.step() == RecenterFlow::Step::Centre); // not yet: the ring has not filled
        room::Batch mid;
        f.build(mid, 0.4); // mid-hold: the ring wedge is drawn (triangles beyond the prompt text)
        CHECK(!mid.tris.empty());
        f.frame(frame(0, kTrig), RecenterFlow::kHoldS + 0.01, reader); // held past kHoldS: the recenter fires
        CHECK(f.step() == RecenterFlow::Step::Floor);
        f.frame(frame(0, kTrig), RecenterFlow::kHoldS + 0.02, reader); // still held: no second action
        CHECK(f.step() == RecenterFlow::Step::Floor);
        f.frame(frame(0, 0), 2.0, reader);
        f.frame(frame(0, kTrig, 0.04), 2.1, reader); // the floor touch, 4 cm above the stage floor
        CHECK(f.finished() && f.result() && f.result()->head.pid == 77);
        CHECK(f.result()->floorStage && std::fabs(*f.result()->floorStage - 0.04) < 1e-9);
        CHECK(recenterResultText(*f.result()) ==
              "pibridge-recenter 1\npid 77\nhead 0.500000 1.700000 -1.000000 0.0000000 0.0000000 0.0000000 1.0000000\nfloor 0.0400\n");
        room::Batch b;
        f.build(b, 2.1);
        CHECK(!b.tris.empty());
    }
    {
        RecenterFlow f;
        f.frame(frame(kTrig, 0), 0.0, reader);                        // left-hand trigger: start the hold
        f.frame(frame(kTrig, 0), RecenterFlow::kHoldS + 0.01, reader); // fills -> Floor
        CHECK(f.step() == RecenterFlow::Step::Floor);
        f.frame(frame(0, 0), 2.0, reader);
        f.frame(frame(kMenu, 0), 2.1, reader); // keep the floor
        CHECK(f.finished() && !f.result()->floorStage);
        CHECK(recenterResultText(*f.result()).ends_with("floor keep\n"));
    }
    {
        RecenterFlow f; // no ring: Monado's anchor is off
        f.frame(frame(0, kTrig), 0.0, [] { return std::optional<RingSample>(); });
        CHECK(f.step() == RecenterFlow::Step::Centre && f.lines().back().starts_with("NO HEAD POSE"));
        f.frame(frame(0, 0), 0.1, reader);
        f.frame(frame(0, kTrig), 0.2, reader);                       // start the hold with a valid ring
        f.frame(frame(0, kTrig), 0.2 + RecenterFlow::kHoldS + 0.01, reader); // fills -> Floor
        CHECK(f.step() == RecenterFlow::Step::Floor && !f.lines().back().starts_with("NO HEAD POSE"));
        f.frame(frame(0, 0), 3.0, reader);
        f.frame(frame(0, kTrig, 1.9), 3.1, reader); // a controller held high: not the floor
        CHECK(f.step() == RecenterFlow::Step::Floor && f.lines().back().starts_with("THAT IS NOT THE FLOOR"));
    }
}

TEST(floor_touch_is_checked_against_the_head) {
    // live 2026-10-19: the stage floor was 0.8 m off, the head at stage y -0.3, the real floor at -1.8: refused before
    CHECK(recenterFloorPlausible(true, -0.3, -1.8));
    CHECK(!recenterFloorPlausible(true, -0.3, -0.5));  // only 0.2 m below the head: a hand held up
    CHECK(!recenterFloorPlausible(true, 1.7, -1.0));   // 2.7 m below: not plausible
    CHECK(recenterFloorPlausible(true, 1.7, 0.03));
    CHECK(recenterFloorPlausible(false, 0, 0.1) && !recenterFloorPlausible(false, 0, -1.8));
}

TEST(recenter_options) {
    auto o = parseArgs({"--recenter", "--recenter-out", "/r/out.txt", "--ring", "/r/ring"});
    CHECK(o && o->run == Run::Recenter && o->recenterOut == "/r/out.txt" && o->ringFile == "/r/ring");
    CHECK(!parseArgs({"--recenter"}).has_value());
    CHECK(usage().find("--recenter") != std::string::npos);
}

PIBRIDGE_TEST_MAIN()
