// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/gaze.rs tests (MIT OR Apache-2.0).
#include <algorithm>
#include <format>
#include <string>
#include <string_view>
#include <cmath>
#include <numbers>
#include <numeric>
#include <vector>

#include "pibridge/eyemath/gaze.h"
#include "pibridge_test.h"
#include "tlv_lite.h"

using namespace pibridge::eyemath;
namespace d = pibridge::eyemath::detail;

namespace {

/// Canonical sample with eye A tracked.
GazeSample sample() {
    GazeSample s;
    s.timestamp_us = 50'376'465;
    s.frame_counter = 5412;
    s.frame.field_14 = 0xfffff;
    s.frame.field_34 = fixed16_16_to_f64(0x0039'5a85);
    s.frame.field_35 = 1;
    auto& a = s.eyes[0];
    a.origin_mm = {30.75, -4.5, -27.25};
    a.origin_valid = true;
    a.direction = {0.3125, 0.140625, 0.9375};
    a.direction_valid = true;
    a.pupil_diameter_mm = 3.5;
    a.pupil_pos = {0.5, 0.625};
    a.pupil_pos_valid = true;
    a.flags = {1, 0};
    s.eyes[1].flags = {1, 0};
    return s;
}

std::span<const std::uint8_t> head(const std::vector<std::uint8_t>& p, std::size_t n) { return {p.data(), n}; }

std::vector<std::uint8_t> with_prefix(const std::vector<std::uint8_t>& items) {
    std::vector<std::uint8_t> p{0, 0};
    p.insert(p.end(), items.begin(), items.end());
    return p;
}

} // namespace

TEST(wire_order_covers_each_field_once) {
    std::vector<std::uint8_t> seen(WIRE_ORDER.begin(), WIRE_ORDER.end());
    std::ranges::sort(seen);
    std::vector<std::uint8_t> want(35);
    std::iota(want.begin(), want.end(), std::uint8_t{1});
    CHECK(seen == want);
}

TEST(encode_then_parse_is_identity) {
    auto s = sample();
    const auto p = encode_payload(s);
    const auto back = parse_payload(p);
    CHECK(back.has_value());
    if (!back) {
        return;
    }
    s.fields = back->fields;
    s.eyes[0].complete = true;
    s.eyes[1].complete = true;
    CHECK(*back == s);
    CHECK(back->is_complete());
    CHECK(encode_payload(*back) == p); // bytes must survive a decode/encode cycle
}

TEST(truncation_keeps_complete_elements_only) {
    const auto p = encode_payload(sample());
    const auto full = *parse_payload(p);
    for (std::size_t cut = 0; cut < p.size(); cut++) {
        const auto r = parse_payload_lenient(head(p, cut));
        if (r) {
            const auto& s = *r;
            CHECK(s.truncated);
            CHECK(!parse_payload(head(p, cut)).has_value());
            // every decoded field equals the full decode
            if (s.has(field::TIMESTAMP)) {
                CHECK(s.timestamp_us == full.timestamp_us);
            }
            if (s.eyes[0].complete) {
                CHECK(s.eyes[0].origin_mm == full.eyes[0].origin_mm);
            }
            CHECK((s.fields & ~full.fields) == 0);
        } else {
            const auto k = r.error().kind;
            CHECK(k == GazeError::Kind::TooShort || k == GazeError::Kind::NotGaze);
        }
    }
    CHECK(!parse_payload_lenient(p)->truncated);
}

TEST(strict_rejects_trailing_items_and_short_containers) {
    const auto p = encode_payload(sample());
    // an item after the announced elements
    auto extra = p;
    d::push_i32(extra, 1);
    const auto e1 = parse_payload(extra);
    CHECK(!e1 && e1.error().kind == GazeError::Kind::Trailing);
    // container announces one element more than present
    auto more = p;
    const std::array<std::uint8_t, 5> hdr{5, 0, 0, 0, 4};
    const auto it = std::search(more.begin(), more.end(), hdr.begin(), hdr.end());
    CHECK(it != more.end());
    const auto pos = static_cast<std::size_t>(it - more.begin());
    const std::uint32_t w = (36U << 16) | TYPE_GAZE;
    more[pos + 5] = static_cast<std::uint8_t>(w >> 24);
    more[pos + 6] = static_cast<std::uint8_t>(w >> 16);
    more[pos + 7] = static_cast<std::uint8_t>(w >> 8);
    more[pos + 8] = static_cast<std::uint8_t>(w);
    GazeError want;
    want.kind = GazeError::Kind::Truncated;
    want.found = 35;
    want.announced = 36;
    const auto e2 = parse_payload(more);
    CHECK(!e2 && e2.error() == want);
    const auto lenient = parse_payload_lenient(more);
    CHECK(lenient && lenient->truncated && lenient->is_complete());
    // wrong value type for a known field: the timestamp as int32
    std::vector<std::uint8_t> wrong;
    d::push_compound(wrong, 1, TYPE_GAZE);
    d::push_compound(wrong, 2, TYPE_FIELD);
    d::push_i32(wrong, 1);
    d::push_i32(wrong, 5);
    const auto e3 = parse_payload(with_prefix(wrong));
    CHECK(!e3 && e3.error().kind == GazeError::Kind::BadField && e3.error().field == 1);
}

TEST(unknown_fields_are_counted_not_fatal) {
    std::vector<std::uint8_t> items;
    d::push_compound(items, 2, TYPE_GAZE);
    d::push_compound(items, 2, TYPE_FIELD);
    d::push_i32(items, 1);
    d::push_i64(items, 77);
    d::push_compound(items, 2, TYPE_FIELD);
    d::push_i32(items, 99);
    d::push_i32(items, 5);
    const auto s = parse_payload(with_prefix(items));
    CHECK(s.has_value());
    CHECK(s->timestamp_us == 77 && s->unknown_fields == 1);
    CHECK(!s->is_complete());
}

TEST(non_gaze_payloads_are_rejected) {
    GazeError too_short;
    too_short.kind = GazeError::Kind::TooShort;
    GazeError not_gaze;
    not_gaze.kind = GazeError::Kind::NotGaze;
    const std::vector<std::uint8_t> empty;
    CHECK(parse_payload(empty).error() == too_short);
    const std::vector<std::uint8_t> two{0, 0};
    CHECK(parse_payload(two).error() == too_short);
    std::vector<std::uint8_t> items;
    d::push_i32(items, 3);
    CHECK(parse_payload(with_prefix(items)).error() == not_gaze);
}

TEST(eye_helpers) {
    const auto s = sample();
    CHECK(s.eyes[0].state() == EyeState::Tracked);
    CHECK(s.eyes[1].state() == EyeState::Lost);
    const auto yp = *s.eyes[0].yaw_pitch_deg();
    CHECK_NEAR(yp.first, std::atan2(0.3125, 0.9375) * (180.0 / std::numbers::pi), 1e-12);
    CHECK(yp.second > 0.0);
    CHECK(s.eyes[0].pupil_diameter() == std::optional<double>(3.5));
    CHECK(!s.eyes[1].pupil_diameter().has_value());
    CHECK(!s.eyes[0].openness().has_value());
    CHECK(!s.eyes[1].ray().has_value());
    CHECK(s.eyes[0].ray().has_value());
}

TEST(stats_detect_gaps_and_rate) {
    GazeStats st;
    auto s = sample();
    for (std::uint32_t i = 0; i < 100; i++) {
        s.timestamp_us += NOMINAL_PERIOD_US;
        s.frame_counter = 5412 + i;
        if (i == 50) {
            s.timestamp_us += 2 * NOMINAL_PERIOD_US;
            s.frame_counter = 5412 + i + 2;
        }
        if (i > 50) {
            s.frame_counter = 5412 + i + 2;
        }
        st.add(s);
    }
    CHECK(st.samples == 100);
    CHECK(st.counter_gaps == 2);
    CHECK(st.timestamp_gaps == 1);
    CHECK(st.timestamp_regressions == 0);
    const auto hz = *st.device_rate_hz();
    CHECK_NEAR(hz, 1e6 / 8300.0 * 99.0 / 101.0, 0.5);
    CHECK((st.states[0] == std::array<std::uint64_t, 3>{0, 0, 100}));
    CHECK((st.states[1] == std::array<std::uint64_t, 3>{100, 0, 0}));
}

TEST(line_format_mentions_both_eyes) {
    const auto decoded = *parse_payload(encode_payload(sample()));
    const auto l = format_line(decoded);
    CHECK(l.find("A: org(") != std::string::npos && l.find("B: invalid") != std::string::npos);
}

// C++ only: the error messages are those of the Rust Display impl.
TEST(error_messages_match_rust) {
    GazeError e;
    e.kind = GazeError::Kind::Truncated;
    e.found = 35;
    e.announced = 36;
    CHECK(e.message() == "payload truncated: 35 of 36 elements");
    std::vector<std::uint8_t> wrong;
    d::push_compound(wrong, 1, TYPE_GAZE);
    d::push_compound(wrong, 2, TYPE_FIELD);
    d::push_i32(wrong, 1);
    d::push_i32(wrong, 5);
    const auto r = parse_payload(with_prefix(wrong));
    CHECK(!r && r.error().message() == "field 1: value type does not match the field");
}

// C++ only: the bytes and the printed line of the Rust encode_payload / format_line for the canonical sample
// (printed by the omarchyvr code), so the fake chip of libpibridge-eye sends what the Rust fake sends.
constexpr std::string_view RUST_SAMPLE_HEX =
    "0000050000000400230bb8050000000400020bb90200000004000000010600000008000000000300af11050000000400020bb90200000004"
    "00000002050000000400031f420300000004001ec0000300000004fffb80000300000004ffe4c000050000000400020bb902000000040000"
    "0003020000000400000001050000000400020bb9020000000400000004050000000400031f42030000000400005000030000000400002400"
    "03000000040000f000050000000400020bb9020000000400000005020000000400000001050000000400020bb90200000004000000060300"
    "00000400038000050000000400020bb90200000004000000070300000004ffff0000050000000400020bb902000000040000001005000000"
    "0400021f400400000008fffffc00000000000400000008fffffc0000000000050000000400020bb902000000040000001705000000040002"
    "1f4303000000040000800003000000040000a000050000000400020bb9020000000400000016020000000400000001050000000400020bb9"
    "02000000040000001e020000000400000001050000000400020bb902000000040000001f020000000400000000050000000400020bb90200"
    "00000400000008050000000400031f42030000000400000000030000000400000000030000000400000000050000000400020bb902000000"
    "0400000009020000000400000000050000000400020bb902000000040000000a050000000400031f42030000000400000000030000000400"
    "000000030000000400000000050000000400020bb902000000040000000b020000000400000000050000000400020bb90200000004000000"
    "0c0300000004ffff0000050000000400020bb902000000040000000d0300000004ffff0000050000000400020bb902000000040000001105"
    "0000000400021f400400000008fffffc00000000000400000008fffffc0000000000050000000400020bb902000000040000001905000000"
    "0400021f430300000004ffff00000300000004ffff0000050000000400020bb9020000000400000018020000000400000000050000000400"
    "020bb9020000000400000020020000000400000001050000000400020bb9020000000400000021020000000400000000050000000400020b"
    "b902000000040000000e0200000004000fffff050000000400020bb902000000040000000f020000000400001524050000000400020bb902"
    "0000000400000012050000000400031f420300000004ffff00000300000004ffff00000300000004ffff0000050000000400020bb9020000"
    "000400000013020000000400000000050000000400020bb9020000000400000014050000000400031f420300000004ffff00000300000004"
    "ffff00000300000004ffff0000050000000400020bb9020000000400000015020000000400000000050000000400020bb902000000040000"
    "001a020000000400000000050000000400020bb902000000040000001b020000000400000000050000000400020bb902000000040000001d"
    "0300000004ffff0000050000000400020bb902000000040000001c020000000400000000050000000400020bb90200000004000000220300"
    "00000400395a85050000000400020bb9020000000400000023020000000400000001";
constexpr std::string_view RUST_SAMPLE_LINE =
    "ts=    50376465us ctr=  5412 | A: org( +30.8,  -4.5, -27.2) dir(+0.312,+0.141,+0.938) yaw +18.4 pitch  +8.1 pupil 3.50mm | B: invalid";

TEST(encoding_matches_the_rust_bytes) {
    const auto p = encode_payload(sample());
    CHECK(p.size() == 1210);
    std::string hex;
    for (auto b : p) {
        hex += std::format("{:02x}", b);
    }
    CHECK(hex == RUST_SAMPLE_HEX);
    CHECK(format_line(*parse_payload(p)) == RUST_SAMPLE_LINE);
}

PIBRIDGE_TEST_MAIN()
