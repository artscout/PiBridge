// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/frames.rs tests (MIT OR Apache-2.0).
#include "pibridge/eye/frames.h"

#include "pibridge/eye/fake.h"

#include "pibridge_test.h"
#include "testutil.h"

using namespace pibridge::eye;
using namespace pibridge::eye::frames;
using namespace testutil;
using namespace std::chrono_literals;

namespace {

std::vector<FStep> base_steps() {
    auto s = parse(base_list());
    CHECK(s.has_value());
    return s.value_or(std::vector<FStep>{});
}

// The omarchyvr public tree's `data/replay-frames.txt` is a comment-only stand-in (the real list is private lab data):
// it parses to no steps. The guard test below runs on it as the Rust test does.
constexpr std::string_view DATA_STUB =
    "# Replay frame list of the eye chip cold start: NOT DISTRIBUTED.\n"
    "# With this empty list, eye tracking cannot start; the tests that need the real list skip.\n";

std::vector<Bytes> cal_frames(const std::vector<FStep>& steps) {
    for (const auto& s : steps) {
        if (const auto* t = std::get_if<Ttp>(&s.action); t != nullptr && t->op == CAL_OP) {
            return t->frames;
        }
    }
    CHECK(false); // a 0x456 step
    return {};
}

std::vector<std::uint32_t> ops_of(const std::vector<FStep>& s) {
    std::vector<std::uint32_t> ops;
    for (const auto& x : s) {
        if (const auto* t = std::get_if<Ttp>(&x.action)) {
            ops.push_back(t->op);
        }
    }
    return ops;
}

} // namespace

TEST(list_has_26_steps_and_29_urbs) { skip_lab_data("list_has_26_steps_and_29_urbs"); }
TEST(opcodes_in_order_match_the_transcript) { skip_lab_data("opcodes_in_order_match_the_transcript"); }
TEST(hello_frame_is_byte_exact) { skip_lab_data("hello_frame_is_byte_exact"); }
TEST(multi_urb_blob_reassembles) { skip_lab_data("multi_urb_blob_reassembles"); }
TEST(auth_answer_step_has_the_captured_answer) { skip_lab_data("auth_answer_step_has_the_captured_answer"); }
TEST(select_applies_skips) { skip_lab_data("select_applies_skips"); }
TEST(base_frames_are_byte_identical_to_the_private_list) {
    skip_lab_data("base_frames_are_byte_identical_to_the_private_list");
}

TEST(the_compiled_in_base_list_is_the_test_data_file) {
    CHECK(base_list() == read_text(std::filesystem::path(PIBRIDGE_EYE_DATA_DIR) / "base-frames.txt"));
}

TEST(a_list_made_from_a_plan_parses_back_to_the_same_transfers) {
    // a plan with the public base list's transfers, as a capture of it would give
    const auto base = base_steps();
    replay::Plan plan;
    plan.device = 7;
    plan.stream_start = 9.0;
    for (std::size_t i = 0; i < base.size(); i++) {
        const auto& st = base[i];
        replay::StepKind kind;
        if (std::holds_alternative<Enumeration>(st.action)) {
            continue;
        } else if (const auto* c = std::get_if<Control>(&st.action)) {
            if ((c->setup[0] & 0x80) == 0) {
                kind = replay::ControlOut{c->setup, c->data};
            } else {
                kind = replay::ControlIn{c->setup};
            }
        } else {
            const auto& t = std::get<Ttp>(st.action);
            kind = replay::Request{t.seq, t.op, 0, t.frames, std::nullopt};
        }
        plan.steps.push_back(replay::Step{i, static_cast<double>(i), std::move(kind), 0, 1});
    }
    auto text = from_plan(plan, "test", false);
    CHECK(text.has_value());
    auto back = parse(*text);
    CHECK(back.has_value());
    std::vector<Action> want, got;
    for (const auto& s : base) {
        if (!std::holds_alternative<Enumeration>(s.action)) {
            want.push_back(s.action);
        }
    }
    for (const auto& s : *back) {
        got.push_back(s.action);
    }
    CHECK(got == want);
    // the made list is accepted by the gate with its own profile upload
    CHECK(Guard::make_with(*back, false).has_value());
    // a cut capture is refused
    plan.steps[0].missing_bytes = 3;
    CHECK(!from_plan(plan, "test", false));
}

TEST(base_list_parses_and_has_no_auth_ops) {
    const auto s = base_steps();
    CHECK(s.size() == 24); // 26 steps of the cold start minus the 2 auth steps
    CHECK(std::ranges::all_of(s, [](const FStep& x) { return x.cls != "AUTH"; }));
    const auto ops = ops_of(s);
    CHECK(std::ranges::all_of(ops, [](std::uint32_t o) { return !is_auth(o) && !is_denied(o); }));
    const std::vector<std::uint32_t> want = {0x3e8, 0x640, 0x852, 0x77b, 0x532, 0x58c, 0x546, 0x547, 0x5d2, 0x5cd,
                                             0x4b0, 0x596, 0x578, 0x686, 0x67c, 0x5be, 0x672, 0x4c4, 0x456};
    CHECK(ops == want);
    // skipping auth changes nothing (it is absent); enumeration is never sent
    CHECK(select(s, Skips{}).size() == 23);
    CHECK(select(s, Skips{.auth = true}).size() == 23);
}

TEST(the_compiled_in_p3b_list_is_the_data_file) {
    CHECK(p3b_list() == read_text(std::filesystem::path(PIBRIDGE_EYE_DATA_DIR) / "p3b-frames.txt"));
}

TEST(p3b_list_is_the_captured_start_of_the_original_crystals_tracker) {
    // Pimax Play's start of 2104:0220 (capture 20261022-202727-crystal-cold-hub2.pcap, device address 97, 20:28:37.9):
    // 4 vendor control transfers, 12 reads, 8 clock reads, the gaze subscribe, the lens configuration
    const auto p = parse(p3b_list());
    CHECK(p.has_value());
    const auto s = p.value_or(std::vector<FStep>{});
    CHECK(s.size() == 26);
    const auto ops = ops_of(s);
    const std::vector<std::uint32_t> want = {0x3e8, 0x640, 0x58c, 0x532, 0x546, 0x547, 0x5d2,
                                             0x686, 0x4b0, 0x596, 0x578, 0x672, 0x4c4, 0x5be};
    CHECK(ops == want);
    CHECK(std::ranges::all_of(ops, [](std::uint32_t o) { return !is_auth(o) && !is_denied(o) && o != CAL_OP; }));
    // byte exact against the capture: the hello, the subscribe and the lens configuration
    const auto frame_of = [&](std::uint32_t op) {
        for (const auto& x : s) {
            if (const auto* t = std::get_if<Ttp>(&x.action); t != nullptr && t->op == op && t->frames.size() == 1) {
                return hex(t->frames[0]);
            }
        }
        return std::string();
    };
    CHECK(frame_of(0x3e8) == "0000000047000000000000510000000100000000000003e8000000000000002f000017000000280000000900010000"
                             "0001000100010002000100030001000400010005000100060001000700010008");
    CHECK(frame_of(0x4c4) == "000000002c000000000000510000000d00000000000004c4000000000000001400000200000004000005051700"
                             "00000400000000");
    CHECK(frame_of(0x5be) == "000000007a000000000000510000000e00000000000005be00000000000000620000050000000400031f41040000"
                             "000800007860008000000400000008000000000000000004000000080000000000000000050000000400031f41"
                             "0400000008ffff879fff8000000400000008000000000000000004000000080000000000000000");
    // the vendor control transfers are those of 2104:0561 (the write bReq 48 is skipped by default, as there)
    const auto* c0 = std::get_if<Control>(&s[0].action);
    CHECK(c0 != nullptr && c0->setup[0] == 0x41 && c0->setup[1] == 48 &&
          hex(c0->data) == "010000000000000001000000040000000000000000000000");
    CHECK(select(s, Skips{.vendor_write = true}).size() == 25);
    CHECK(Guard::make_with(s, false).has_value());
}

TEST(base_calibration_step_matches_the_pins) {
    const auto s = base_steps();
    const auto* t = std::get_if<Ttp>(&s.back().action);
    CHECK(t != nullptr && t->op == CAL_OP);
    const auto m = ttp_info(t->frames);
    CHECK(m.has_value());
    CHECK(m->header.payload_len == CAL_PAYLOAD_LEN);
    CHECK(m->payload.size() == CAL_PAYLOAD_LEN);
    CHECK(fnv1a(m->payload) == CAL_PAYLOAD_FNV);
    // the guard accepts the list (it pins the blob) and rejects a changed blob
    CHECK(Guard::make(s).has_value());
    auto bad = s;
    auto& frames = std::get<Ttp>(bad.back().action).frames;
    frames.back().back() ^= 1;
    CHECK(!Guard::make(bad));
}

TEST(list_picks_the_base_list_unless_the_full_one_is_linked) {
    const auto [text, kind] = list();
    CHECK(kind == ListKind::Base); // no private list is ever compiled into this tree
    CHECK(text == base_list());
}

TEST(guard_refuses_foreign_and_denied) {
    const auto s = parse(DATA_STUB);
    CHECK(s.has_value() && s->empty());
    const auto g = Guard::make(*s);
    CHECK(g.has_value());
    fake::FakeChip chip;
    // a hand-built 0x3f2 start-calibration request
    const auto m = ttp::Message::make(ttp::Kind::Request, 99, 0, 0x3f2, {});
    const auto f = m.to_frames(ttp::Direction::Out, 4087).front();
    const auto e = g->bulk_out(chip, f, 10ms);
    CHECK(!e && e.error().code == Errc::permission_denied);
    // a vendor write that is not in the list
    const auto c = g->control(chip, {0x41, 49, 0, 0, 0, 0, 1, 0}, Bytes{1}, 10ms);
    CHECK(!c && c.error().code == Errc::permission_denied);
    CHECK(chip.sent.empty());
}

TEST(guard_passes_listed_transfers_and_refuses_denied_even_when_allowed) {
    const auto s = base_steps();
    auto g = Guard::make(s);
    CHECK(g.has_value());
    fake::FakeChip chip;
    const auto& hello = std::get<Ttp>(s[5].action);
    CHECK(hello.op == 0x3e8);
    CHECK(g->bulk_out(chip, hello.frames[0], 10ms).has_value());
    const auto& ctl = std::get<Control>(s[1].action);
    CHECK(g->control(chip, ctl.setup, ctl.data, 10ms).has_value());
    // one changed byte is refused
    Bytes changed = hello.frames[0];
    CHECK(changed.size() == 79);
    if (!changed.empty()) {
        changed[changed.size() - 1] ^= 1;
    }
    CHECK(!g->bulk_out(chip, changed, 10ms));
    // the same control with other data is refused
    Bytes other = ctl.data;
    other[0] ^= 1;
    CHECK(!g->control(chip, ctl.setup, other, 10ms));
    // allow_frame cannot open a denied opcode
    const auto cal = ttp::Message::make(ttp::Kind::Request, 1, 0, 0x407, {}).to_frames(ttp::Direction::Out, 4087)[0];
    g->allow_frame(cal);
    const auto e = g->bulk_out(chip, cal, 10ms);
    CHECK(!e && e.error().code == Errc::permission_denied);
    CHECK(chip.sent.size() == 2);
}

TEST(guard_rejects_list_with_denied_op) {
    const auto m = ttp::Message::make(ttp::Kind::Request, 1, 0, 0x407, {});
    const auto f = m.to_frames(ttp::Direction::Out, 4087).front();
    const std::vector<FStep> st = {FStep{1, "R", "", std::nullopt, Ttp{1, 0x407, {f}}}};
    CHECK(!Guard::make(st));
}

TEST(chunker_handles_split_in_stream) {
    // a gaze-like frame split over two URBs, then a response in the second URB's tail
    const auto m = ttp::Message::make(ttp::Kind::Response, 3, 1, 0x777, {});
    const auto f = m.to_frames(ttp::Direction::In, 4087).front();
    ttp::StreamDecoder dec(ttp::Direction::In);
    const std::span<const std::uint8_t> all(f);
    CHECK(dec.push(all.first(10)).empty());
    const auto r = dec.push(all.subspan(10));
    CHECK(r.size() == 1);
    CHECK(r[0]->header.id == 0x777);
}

TEST(the_captured_profile_upload_is_rebuilt_byte_for_byte_from_its_body) {
    auto base = base_steps();
    const auto captured = cal_frames(base);
    const auto m = ttp_info(captured);
    CHECK(m.has_value());
    const auto pb = profile_body(m->payload); // the captured upload has the profile layout
    CHECK(pb.has_value());
    const Bytes body(pb->begin(), pb->end());
    CHECK(check_profile(body).has_value());
    CHECK(with_profile(base, body).has_value());
    CHECK(cal_frames(base) == captured);
    // and the list still passes the pinned guard
    CHECK(Guard::make(base).has_value());
}

TEST(another_profile_replaces_the_upload_and_needs_the_unpinned_guard) {
    auto base = base_steps();
    Bytes body(37401, 0x5a);
    const std::uint32_t lead = 37401u - 168;
    for (int i = 0; i < 4; i++) {
        body[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(lead >> (8 * i));
    }
    Bytes setpm;
    for (const std::uint32_t w : {4u, 1u, 37405u, 0x12440846u}) {
        for (int i = 0; i < 4; i++) {
            setpm.push_back(static_cast<std::uint8_t>(w >> (8 * i)));
        }
    }
    setpm.insert(setpm.end(), body.begin(), body.end());
    CHECK(profile_from_setpm(setpm) == body);
    CHECK(with_profile(base, body).has_value());
    const auto frames = cal_frames(base);
    // 24 + 37 412 bytes in 4087-byte bodies, as Windows splits it
    CHECK(frames.size() == 10);
    CHECK(std::all_of(frames.begin(), frames.begin() + 9, [](const Bytes& f) { return f.size() == 4095; }));
    const auto m = ttp_info(frames);
    CHECK(m.has_value() && m->header.payload_len == 37412);
    const auto pb = profile_body(m->payload);
    CHECK(pb.has_value() && std::ranges::equal(*pb, body));
    CHECK(!Guard::make(base));
    CHECK(Guard::make_with(base, false).has_value());
}

TEST(a_broken_setpm_or_profile_is_refused) {
    CHECK(!profile_from_setpm(Bytes{4, 0, 0, 0}));
    Bytes f(16 + 400, 0);
    f[0] = 4;
    f[4] = 1;
    f[8] = static_cast<std::uint8_t>(404 & 0xff);
    f[9] = static_cast<std::uint8_t>(404 >> 8);
    // body does not start with its length
    CHECK(!profile_from_setpm(f));
    const std::uint32_t lead = 400 - 168;
    f[16] = static_cast<std::uint8_t>(lead);
    f[17] = static_cast<std::uint8_t>(lead >> 8);
    CHECK(profile_from_setpm(f).has_value());
    f[8] = 0;
    CHECK(!profile_from_setpm(f));
    CHECK(!check_profile(Bytes(100, 0)));
    auto steps = base_steps();
    std::erase_if(steps, [](const FStep& s) {
        const auto* t = std::get_if<Ttp>(&s.action);
        return t != nullptr && t->op == CAL_OP;
    });
    CHECK(!with_profile(steps, std::span<const std::uint8_t>(f).subspan(16)));
}

TEST(parse_refuses_broken_lists) {
    CHECK(!parse("STEP x | R | standard: x\n"));
    CHECK(!parse("STEP 1 R standard\n"));
    CHECK(!parse("  BULK_OUT ep=0x05 len=1 00\n"));
    CHECK(!parse("STEP 2 | C | CTRL OUT bmRT=0x41 bReq=48 wValue=0 wIndex=0 wLength=2 data=01\n"));
    CHECK(!parse("STEP 3 | C | seq=1 op=0x3e9 x\n  BULK_OUT ep=0x05 len=32 "
                 "0000000018000000000000510000000200000000000006400000000000000000\n"));
    const auto ok = parse("STEP 3 | R | seq=2 op=0x640 get (t=1.5)\n  BULK_OUT ep=0x05 len=32 "
                          "0000000018000000000000510000000200000000000006400000000000000000\n");
    CHECK(ok.has_value() && ok->size() == 1 && (*ok)[0].t == std::optional<double>(1.5));
}

PIBRIDGE_TEST_MAIN()
