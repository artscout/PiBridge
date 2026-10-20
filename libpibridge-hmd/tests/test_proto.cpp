// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-proto/tests/{framing,decoders,props}.rs and src/model.rs tests (MIT OR Apache-2.0).
#include "common.h"
#include "pibridge_test.h"

#include "pibridge/hmd/proto.h"

#include <bit>
#include <cmath>
#include <random>

using namespace pibridge::hmd;
using testutil::bytes;
using testutil::hex;

static std::string hexn(const Report& r, std::size_t n) { return hex(std::span(r.data(), n)); }
static bool zeros_from(const Report& r, std::size_t from) {
    return std::all_of(r.begin() + static_cast<std::ptrdiff_t>(from), r.end(), [](std::uint8_t b) { return b == 0; });
}

// ------------------------------------------------------------------------------------------------ model.rs

TEST(ids_are_unique_and_found) {
    for (const auto& m : MODELS) {
        CHECK(model_by_pid(m.pid) && *model_by_pid(m.pid) == m);
        CHECK(model_by_bridge_pid(m.pid_bridge) && *model_by_bridge_pid(m.pid_bridge) == m);
        CHECK(model_by_pid(m.pid_bridge) == nullptr); // a bridge id is not a headset
    }
    CHECK(model_by_pid(0x0044)->id == "dream-air");
    CHECK(model_by_pid(0x0040)->id == "crystal-super");
    CHECK(model_by_pid(0x0042) == nullptr);
}

// ------------------------------------------------------------------------------------------------ framing.rs

TEST(wake_and_keepalive_bytes_match_python) {
    const Report w = wake();
    CHECK(hexn(w, 8) == "58 06 1a 05 01 00 00 00");
    CHECK(zeros_from(w, 8));
    for (const std::uint8_t rate : std::initializer_list<std::uint8_t>{0, 72, 90, 255}) {
        const Report k = keepalive(rate);
        CHECK(k[0] == 0x58 && k[1] == 0x04 && k[2] == 0x39 && k[3] == 0x00 && k[4] == rate && k[5] == 0x00);
        CHECK(zeros_from(k, 6));
    }
}

TEST(write_buffer_has_report_id_zero) {
    const auto b = to_write_buf(wake());
    CHECK(b.size() == 65);
    CHECK(b[0] == 0);
    CHECK(b[1] == 0x58);
}

TEST(build_command_ll_is_len_plus_three) {
    for (std::size_t n = 0; n <= MAX_DATA; n++) {
        const std::vector<std::uint8_t> data(n, 0xAB);
        const auto r = build_command(0x16, 2, data);
        CHECK(r.has_value());
        CHECK((*r)[0] == 0x58);
        CHECK((*r)[1] == n + 3);
        CHECK((*r)[2] == 0x16 && (*r)[3] == 2);
        CHECK(std::equal(data.begin(), data.end(), r->begin() + 4));
        CHECK(zeros_from(*r, 4 + n));
    }
    const std::vector<std::uint8_t> big(MAX_DATA + 1, 0);
    CHECK(build_command(1, 0, big).error() == FrameError::TooLong);
}

TEST(video_prepare_matches_python_steps) {
    const auto c = video_prepare_commands(72);
    CHECK(hexn(c[0], 6) == "58 05 5b 05 02 02");
    CHECK(hexn(c[1], 5) == "58 04 29 00 01");
    CHECK(hexn(c[2], 6) == "58 0d 1a 05 02 48");
    CHECK(hexn(c[3], 6) == "58 0d 1a 05 05 00");
    CHECK(zeros_from(c[2], 14));
}

TEST(attach_burst_matches_python) {
    for (const std::uint8_t slot : std::initializer_list<std::uint8_t>{2, 3}) {
        const auto c = attach_commands(slot);
        char s[8];
        std::snprintf(s, sizeof s, "%02x", slot);
        CHECK(hexn(c[0], 5) == std::string("58 04 16 ") + s + " 1e");
        CHECK(hexn(c[1], 4) == std::string("58 03 50 ") + s);
        CHECK(hexn(c[2], 8) == std::string("58 07 0a ") + s + " 00 00 00 10");
        CHECK(hexn(c[3], 4) == std::string("58 03 03 ") + s);
    }
}

TEST(calib_request_matches_python) {
    const Report r = calib::read_block_request(0x0530);
    CHECK(hexn(r, 8) == "58 07 0a 00 01 30 05 10");
}

TEST(reply_parsing) {
    Report r = bytes({0x4c, 0x09, 0x16, 0x00, 0x04, 'A', 'B', 'C'});
    const auto p = parse_reply(r);
    CHECK(p && p->cmd == 0x16 && p->opt == 0);
    CHECK(p->data.size() == 6);
    const auto s = parse_serial(p->data.subspan(0, 4));
    CHECK(std::string(s.begin(), s.end()) == "ABC");
    CHECK(parse_serial({}).empty());
    const std::uint8_t one[] = {9, 'x'};
    const auto s2 = parse_serial(one);
    CHECK(std::string(s2.begin(), s2.end()) == "x");
    r[0] = 0x53;
    CHECK(!parse_reply(r));
    // LL clamps: 0xff -> 60 data bytes, 0..2 -> empty
    r[0] = 0x4c;
    r[1] = 0xff;
    CHECK(parse_reply(r)->data.size() == 60);
    r[1] = 1;
    CHECK(parse_reply(r)->data.empty());
}

TEST(haptic_bytes_match_the_windows_capture) {
    const Report r = haptic_command(SLOT_RIGHT, 25, 300);
    CHECK(hexn(r, 10) == "58 09 07 03 19 2c 01 00 00 00");
    CHECK(zeros_from(r, 7));
    const Report l = haptic_command(SLOT_LEFT, 99, 11);
    CHECK(hexn(l, 10) == "58 09 07 02 63 0b 00 00 00 00");
    CHECK(zeros_from(l, 7));
    CHECK(cmd::HAPTIC == 0x07);
    // clamps: AMP 1..100 (never 0 on the wire), DUR >= 1
    const Report a = haptic_command(3, 0, 0);
    CHECK(a[4] == 1 && a[5] == 1 && a[6] == 0);
    const Report b = haptic_command(3, 250, 65535);
    CHECK(b[4] == 100 && b[5] == 0xff && b[6] == 0xff);
}

TEST(haptic_openxr_mapping) {
    using P = std::pair<std::uint8_t, std::uint16_t>;
    CHECK(haptic_from_openxr(0.25f, 300'000'000) == P(25, 300));
    CHECK(haptic_from_openxr(1.0f, -1) == P(100, 12)); // XR_MIN_HAPTIC_DURATION -> 12 ms like games
    CHECK(haptic_from_openxr(0.5f, 0) == P(50, 12));
    CHECK(haptic_from_openxr(0.001f, 1) == P(1, 1)); // tiny amplitude still buzzes, 1 ns -> 1 ms
    CHECK(haptic_from_openxr(2.0f, 11'200'000) == P(100, 12));
    CHECK(haptic_from_openxr(0.3f, 1'000'000'000'000) == P(30, 65535));
    CHECK(!haptic_from_openxr(0.0f, 300'000'000)); // amplitude 0 = stop: nothing is sent
    CHECK(!haptic_from_openxr(std::nanf(""), 300'000'000));
}

TEST(video_prepare_90_matches_the_windows_capture) {
    const auto c = video_prepare_commands(90);
    CHECK(hexn(c[2], 17) == "58 0d 1a 05 02 5a 00 00 00 00 00 00 00 00 00 00 00");
    CHECK(hexn(c[3], 17) == "58 0d 1a 05 05 00 00 00 00 00 00 00 00 00 00 00 00");
    CHECK(hexn(video_prepare_commands(72)[2], 6) == "58 0d 1a 05 02 48");
    CHECK(hexn(keepalive(90), 6) == "58 04 39 00 5a 00");
}

TEST(round_pos_matches_std_round) {
    float x = 0.0f;
    int bad = 0;
    while (x <= 100.0f) {
        if (round_pos(x) != static_cast<std::uint32_t>(std::round(x))) {
            bad++;
        }
        x += 1e-4f;
    }
    for (std::uint32_t k = 0; k <= 200; k++) {
        const float h = static_cast<float>(k) + 0.5f;
        const std::uint32_t hb = std::bit_cast<std::uint32_t>(h);
        for (const float v : {h, std::bit_cast<float>(hb - 1), std::bit_cast<float>(hb + 1), static_cast<float>(k)}) {
            if (round_pos(v) != static_cast<std::uint32_t>(std::round(v))) {
                bad++;
            }
        }
    }
    CHECK(bad == 0);
}

// ------------------------------------------------------------------------------------------------ decoders.rs (hand vectors)

static void put_be16(Report& r, std::size_t at, std::int16_t v) {
    r[at] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(v) >> 8);
    r[at + 1] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(v) & 0xff);
}
static void put_le(Report& r, std::size_t at, std::uint32_t v, std::size_t n) {
    for (std::size_t i = 0; i < n; i++) {
        r[at + i] = static_cast<std::uint8_t>(v >> (8 * i));
    }
}

TEST(imu_hand_vector) {
    Report r = bytes({0x53, 0x31, 0x08, 0x00});
    put_le(r, 4, 1'000'000, 4);
    put_be16(r, 14, 2048);
    put_be16(r, 16, -1024);
    put_be16(r, 20, 164);
    put_be16(r, 22, -164);
    put_be16(r, 24, 16);
    put_le(r, 45, 6590, 2);
    const auto s = decode_imu(r);
    CHECK(s.has_value());
    CHECK((*s)[0].ts_us == 1'000'000);
    CHECK((*s)[1].ts_us == 1'001'000);
    CHECK(((*s)[0].accel_g == std::array<float, 3>{1.0f, -0.5f, 0.0f}));
    CHECK((*s)[0].gyro_dps[0] == 10.0f);
    CHECK((*s)[0].gyro_dps[1] == -10.0f);
    CHECK(std::fabs((*s)[0].ipd_mm - 65.9f) < 1e-4f);
    CHECK((*s)[0].sample_in_report == 0 && (*s)[1].sample_in_report == 1);
    r[1] = 0x32;
    CHECK(!decode_imu(r));
}

TEST(controller_hand_vector) {
    Report r = bytes({0x53, 0x32, 0x08, 0x02, 0x43});
    r[5] = 255;
    r[6] = 1;
    r[7] = 200;
    put_le(r, 8, 123456, 4);
    put_be16(r, 16, 1024);
    r[44] = 99;
    r[46] = 0xda; // 90 % charging
    r[47] = 0x0b; // B, A, stick
    r[48] = 1;    // menu
    r[49] = 0x19; // touch A, stick, hand on
    auto c = decode_controller(r);
    CHECK(c.has_value());
    CHECK(c->stick_x == 127 && c->stick_y == -127);
    CHECK(c->trigger == 200 && c->grip == 99);
    CHECK(c->battery == 90 && c->charging);
    CHECK(c->buttons == (BTN_B | BTN_A | BTN_STICK | BTN_MENU));
    CHECK(c->ts_us == 123456);
    CHECK((c->raw_accel == std::array<std::int16_t, 3>{1024, 0, 0}));
    CHECK((c->pressed() == std::vector<std::string_view>{"B", "A", "stick", "menu"}));
    const TouchBits t = c->touch_bits();
    CHECK(t.a && !t.b && t.stick && t.hand_on && t.thumb());
    // the same byte on the right controller decodes the same (bit 4 is "hand on" on both hands)
    r[3] = 3;
    CHECK(decode_controller(r)->touch_bits() == t);
    // bits 0 and 1 rise together for either face button
    r[49] = 0x02;
    CHECK(decode_controller(r)->touch_bits().thumb());
    r[49] = 0x00;
    CHECK(!decode_controller(r)->touch_bits().hand_on);
    r[49] = 0x19;
    // other slots / magic rejected
    r[3] = 4;
    CHECK(!decode_controller(r));
    r[3] = 2;
    r[2] = 0x09;
    CHECK(!decode_controller(r));
    // second sample used when the first is all zero
    Report q = bytes({0x53, 0x32, 0x08, 0x03, 0x43});
    put_be16(q, 30, -5);
    CHECK(decode_controller(q)->raw_accel[0] == -5);
}

static Report from_hex(const std::string& h) {
    Report d{};
    for (std::size_t i = 0; i + 1 < h.size() && i / 2 < d.size(); i += 2) {
        d[i / 2] = static_cast<std::uint8_t>(std::stoi(h.substr(i, 2), nullptr, 16));
    }
    return d;
}

TEST(status_0x37_two_per_packet_and_python_golden) {
    const Report d = from_hex(std::string("4c0f370248e8b69102490002010008") + "2d" + "c70f37034846a791026300020100082e38");
    const auto s = parse_status(d);
    CHECK(s[0] && s[1]);
    CHECK(s[0]->slot == 2 && s[0]->battery == 0x49 && s[0]->fw[0] == 0x08 && s[0]->fw[1] == 0x2d);
    CHECK(s[1]->slot == 3 && s[1]->battery == 0x63 && s[1]->fw[0] == 0x08 && s[1]->fw[1] == 0x2e);
    Report one = bytes({0x4c, 0x0f, 0x37, 0x03, 0x48, 4, 0x88, 0x91, 2, 0x63, 0, 2, 1, 0, 8, 0x2e});
    const auto s1 = parse_status(one);
    CHECK(s1[0] && !s1[1]);
    const Report ka = bytes({0x4c, 0x04, 0x39, 0x00, 0x01});
    const auto s2 = parse_status(ka);
    CHECK(!s2[0] && !s2[1]);
    one[0] = 0x53;
    const auto s3 = parse_status(one);
    CHECK(!s3[0] && !s3[1]);

    // every 0x37 packet seen in the cold-start capture agrees with the python parser
    const auto g = testutil::read_text(testutil::data_dir() / "status37_golden.txt");
    CHECK(g.has_value());
    std::istringstream in(g.value_or(""));
    std::string line;
    int n = 0;
    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        const auto sp = line.find(' ');
        const Report p = from_hex(line.substr(0, sp));
        const std::string exp = sp == std::string::npos ? "" : line.substr(sp + 1);
        std::string got;
        for (const auto& x : parse_status(p)) {
            if (x) {
                if (!got.empty()) {
                    got += ';';
                }
                got += std::to_string(x->slot) + ":" + std::to_string(x->battery);
            }
        }
        CHECK(got == exp);
        n++;
    }
    CHECK(n >= 20);
}

TEST(log_splitter_edge_cases) {
    LineSplitter sp;
    std::vector<std::string> out;
    const auto emit = [&](std::string_view l) { out.emplace_back(l); };
    const std::string a = "ab\r\ncd";
    sp.push(std::span(reinterpret_cast<const std::uint8_t*>(a.data()), a.size()), emit);
    const char b[] = "ef\n\n\0\0gh\n";
    sp.push(std::span(reinterpret_cast<const std::uint8_t*>(b), sizeof b - 1), emit);
    CHECK((out == std::vector<std::string>{"ab", "cdef", "gh"}));
    // overlong line is flushed at 511 bytes without losing the next byte
    std::vector<std::size_t> out2;
    const std::vector<std::uint8_t> lng(1100, 'x');
    sp.push(lng, [&](std::string_view l) { out2.push_back(l.size()); });
    CHECK((out2 == std::vector<std::size_t>{511, 511}));
}

// The private-lab calibration tests of decoders.rs run here on the public synthetic document instead.
TEST(calibration_round_trip_and_errors) {
#if PIBRIDGE_HAVE_ZLIB
    const auto xs = testutil::read_text(testutil::data_dir() / "synthetic-headset-calibration.xml");
    CHECK(xs.has_value());
    const std::vector<std::uint8_t> xml(xs->begin(), xs->end());
    const auto blob = calib::build_blob(xml);
    CHECK(blob.size() == calib::TOTAL);
    // through the wire format, in a shuffled order, with duplicates
    calib::Assembler asm_;
    std::vector<std::uint16_t> addrs;
    for (std::uint16_t i = 0; i < calib::BLOCKS; i++) {
        addrs.push_back(static_cast<std::uint16_t>(i * 16));
    }
    std::reverse(addrs.begin(), addrs.end());
    addrs.insert(addrs.end(), {0, 16, 0x530});
    for (const auto a : addrs) {
        CHECK(asm_.feed(calib::make_block_reply(blob, a)));
    }
    CHECK(asm_.complete());
    CHECK(!asm_.next_missing());
    CHECK(asm_.document().value_or(std::vector<std::uint8_t>{}) == xml);
    // missing block
    calib::Assembler part;
    for (std::uint16_t i = 1; i < calib::BLOCKS; i++) {
        part.feed(calib::make_block_reply(blob, static_cast<std::uint16_t>(i * 16)));
    }
    CHECK(part.next_missing() == std::optional<std::uint16_t>(0));
    CHECK(part.document().error() == calib::CalibError::Incomplete);
    // corrupt / truncated
    const std::uint8_t tiny[] = {1, 2, 3};
    CHECK(calib::inflate_blob(tiny).error() == calib::CalibError::Truncated);
    auto bad = blob;
    bad[0] = bad[1] = bad[2] = bad[3] = 0xff;
    CHECK(calib::inflate_blob(bad).error() == calib::CalibError::Truncated);
    bad = blob;
    bad[20] ^= 0xff;
    bad[21] ^= 0xff;
    CHECK(calib::inflate_blob(bad).error() == calib::CalibError::Inflate);
    // non-block replies are ignored, addresses outside the range too
    Report r = calib::make_block_reply(blob, 0x10);
    r[0] = 0x53;
    CHECK(!calib::Assembler().feed(r));
    CHECK(!calib::Assembler().feed(calib::make_block_reply(blob, 0x540)));
    CHECK(!calib::Assembler().feed(calib::make_block_reply(blob, 0x11)));
#else
    std::fprintf(stderr, "  skipped: needs zlib\n");
#endif
}

// ------------------------------------------------------------------------------------------------ props.rs
// proptest twins: the same properties over a fixed-seed random sample of 20000 reports each.

static Report random_report(std::mt19937& g) {
    Report r{};
    for (auto& b : r) {
        b = static_cast<std::uint8_t>(g());
    }
    return r;
}

/// Reports with a valid magic so the decoders actually run.
static Report structured(std::mt19937& g) {
    Report r = random_report(g);
    const auto k = g() % 6;
    switch (k) {
    case 0: r[0] = 0x53, r[1] = 0x31, r[2] = 0x08, r[3] = 0x00; break;
    case 1: r[0] = 0x53, r[1] = 0x32, r[2] = 0x08, r[3] = static_cast<std::uint8_t>(2 + (r[3] & 1)); break;
    case 2: r[0] = 0x4c, r[1] = 0x0f, r[2] = 0x37; break;
    case 3: r[0] = 0x4c; break;
    default: break;
    }
    return r;
}

TEST(props_no_decoder_crashes_and_magic_rules) {
    std::mt19937 g(12345);
    int bad = 0;
    for (int i = 0; i < 20000; i++) {
        const Report r = (i & 1) ? random_report(g) : structured(g);
        (void)parse_status(r);
        (void)calib::parse_block_reply(r);
        calib::Assembler a;
        (void)a.feed(r);
        (void)parse_serial(r);
        bad += decode_imu(r).has_value() != (r[0] == 0x53 && r[1] == 0x31);
        bad += decode_controller(r).has_value() != (r[0] == 0x53 && r[1] == 0x32 && r[2] == 0x08 && (r[3] == 2 || r[3] == 3));
        bad += parse_reply(r).has_value() != (r[0] == 0x4c);
        if (const auto c = decode_controller(r)) {
            bad += c->battery > 127;
            bad += (c->battery | (c->charging ? 0x80 : 0)) != r[46];
            bad += c->stick_x < -128 || c->stick_x > 127 || c->stick_y < -128 || c->stick_y > 127;
            bad += (c->buttons & ~(BTN_A | BTN_B | BTN_STICK | BTN_MENU)) != 0;
            bad += c->raw != r;
        }
        if (const auto s = decode_imu(r)) {
            bad += (*s)[1].ts_us != (*s)[0].ts_us + 1000u;
            for (std::size_t k = 0; k < 3; k++) {
                bad += (*s)[0].accel_g[k] != static_cast<float>((*s)[0].raw_accel[k]) / 2048.0f;
            }
        }
    }
    CHECK(bad == 0);
}

TEST(props_command_round_trip) {
    std::mt19937 g(777);
    int bad = 0;
    for (int i = 0; i < 20000; i++) {
        const auto c = static_cast<std::uint8_t>(g());
        const auto opt = static_cast<std::uint8_t>(g());
        std::vector<std::uint8_t> data(g() % 61);
        for (auto& b : data) {
            b = static_cast<std::uint8_t>(g());
        }
        Report r = build_command(c, opt, data).value();
        bad += r[0] != 0x58;
        r[0] = 0x4c;
        const auto p = parse_reply(r);
        bad += !p || p->cmd != c || p->opt != opt || !std::equal(p->data.begin(), p->data.end(), data.begin(), data.end());
    }
    CHECK(bad == 0);
}

TEST(props_line_splitter_bounds_and_inflate_never_crashes) {
    std::mt19937 g(99);
    int bad = 0;
    for (int i = 0; i < 2000; i++) {
        LineSplitter sp;
        const auto chunks = g() % 40;
        for (std::uint32_t k = 0; k < chunks; k++) {
            std::vector<std::uint8_t> c(g() % 64);
            for (auto& b : c) {
                b = static_cast<std::uint8_t>(g());
            }
            sp.push(c, [&](std::string_view l) {
                bad += l.empty() || l.size() >= 512 || l.find('\n') != std::string_view::npos ||
                       l.find('\0') != std::string_view::npos;
            });
        }
        std::vector<std::uint8_t> blob(g() % 2000);
        for (auto& b : blob) {
            b = static_cast<std::uint8_t>(g());
        }
        (void)calib::inflate_blob(blob);
    }
    CHECK(bad == 0);
}

PIBRIDGE_TEST_MAIN()
