// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-device/tests/allowlist.rs (MIT OR Apache-2.0), against devices/dream-air.toml and
// devices/crystal-super.toml: approved commands pass, everything else is refused before a byte is written.
#include "common.h"
#include "pibridge_test.h"

#include "pibridge/hmd/descriptor.h"
#include "pibridge/hmd/ipd.h"
#include "pibridge/hmd/proto.h"

#include <map>
#include <tuple>

using namespace pibridge::hmd;
using testutil::pad;
using K = Refused::Kind;

static const Descriptor& load(const char* id) {
    static std::map<std::string, Descriptor> cache;
    auto it = cache.find(id);
    if (it == cache.end()) {
        auto d = Descriptor::load(std::filesystem::path(PIBRIDGE_SOURCE_DEVICES_DIR) / (std::string(id) + ".toml"));
        if (!d) {
            std::fprintf(stderr, "%s: %s\n", id, d.error().msg.c_str());
            std::abort();
        }
        it = cache.emplace(id, std::move(*d)).first;
    }
    return it->second;
}
static const Descriptor& dream_air() { return load("dream-air"); }
static const Descriptor& crystal_super() { return load("crystal-super"); }

static std::vector<std::uint8_t> vec(const Report& r) { return {r.begin(), r.end()}; }

/// Send through a gate that writes into a vector: must be refused, and nothing written.
static Refused refused(const Descriptor& d, const std::vector<std::uint8_t>& r) {
    std::vector<std::uint8_t> out;
    Gate g(d.allowlist, 0, vector_writer(out));
    const auto e = g.send(r);
    CHECK(!e.has_value());
    CHECK(out.empty()); // bytes written for a refused report
    if (e || !e.error().refused()) {
        return Refused{K::NotListed, "<not refused>"};
    }
    return *e.error().refusal();
}

static std::string checked_id(const Allowlist& al, std::uint8_t iface, const std::vector<std::uint8_t>& r) {
    const auto c = al.check(iface, r);
    return c ? (*c)->id : "<" + c.error().message() + ">";
}

TEST(both_descriptions_are_real_and_detected) {
    const std::vector<Descriptor> all{dream_air(), crystal_super()};
    CHECK(all[0].id == "dream-air" && !all[0].is_mock());
    CHECK(all[1].id == "crystal-super" && !all[1].is_mock());
    auto m = detect(all, 0x34a4, 0x0044);
    CHECK(m.kind == Detected::Kind::Module && m.desc->id == "dream-air");
    m = detect(all, 0x34a4, 0x0040);
    CHECK(m.kind == Detected::Kind::Module && m.desc->id == "crystal-super");
    CHECK(detect(all, 0x34a4, 0x0043).kind == Detected::Kind::None); // bridge MCU is not the headset
    CHECK(detect(all, 0x34a4, 0x0041).kind == Detected::Kind::None);
    CHECK(detect(all, 0x1234, 0x0044).kind == Detected::Kind::None);
    // the description files match the model table
    for (const auto& md : MODELS) {
        const auto d = load_descriptor(md, {PIBRIDGE_SOURCE_DEVICES_DIR});
        CHECK(d.has_value() && d->matches_usb(VID, md.pid));
    }
}

TEST(ipd_motor_matches_pimax_eye_and_is_clamped) {
    const auto& m = dream_air();
    CHECK(m.ipd_report(60.0f).value() == pad({0x58, 0x06, 0x5c, 0x05, 0x01, 0x70, 0x17}));
    CHECK(m.ipd_report(65.9f).value() == pad({0x58, 0x06, 0x5c, 0x05, 0x01, 0xbe, 0x19}));
    // clamped 58..72
    const auto lo = m.ipd_report(40.0f).value();
    CHECK(lo[5] == (5800 & 0xff) && lo[6] == (5800 >> 8));
    const auto hi = m.ipd_report(99.0f).value();
    CHECK(hi[5] == (7200 & 0xff) && hi[6] == (7200 >> 8));
    CHECK(!m.ipd_report(std::nanf("")));
    // a hand-built out-of-range report is refused at the send layer
    const auto e = refused(m, pad({0x58, 0x06, 0x5c, 0x05, 0x01, 0x88, 0x13})); // 50.00 mm
    CHECK(e.kind == K::OutOfRange && e.param == "ipd" && e.value == 5000);
}

TEST(start_pair_only_for_slots_2_and_3) {
    const auto& m = dream_air();
    CHECK(m.pair_report(Hand::Left).value() == pad({0x58, 0x04, 0x04, 0x00, 0x02}));
    CHECK(m.pair_report(Hand::Right).value() == pad({0x58, 0x04, 0x04, 0x00, 0x03}));
    CHECK(refused(m, pad({0x58, 0x04, 0x04, 0x00, 0x04})).kind == K::OutOfRange);
    CHECK(refused(m, pad({0x58, 0x04, 0x04, 0x01, 0x02})).kind == K::NotListed);
}

TEST(camera_ae_windows_frame_passes_and_bad_gain_does_not) {
    const auto& al = dream_air().allowlist;
    const auto ae = [&](std::array<std::uint32_t, 4> e, std::array<std::uint32_t, 4> g, std::uint32_t le, std::uint32_t lg) {
        return al.build("camera_ae", {{"e0", e[0]}, {"e1", e[1]}, {"e2", e[2]}, {"e3", e[3]}, {"g0", g[0]}, {"g1", g[1]},
                                      {"g2", g[2]}, {"g3", g[3]}, {"led_e", le}, {"led_g", lg}});
    };
    const auto want = pad({0x58, 0x19, 0x51, 0x00, 0x01, 0x32, 0x47, 0x0c, 0x47, 0x0c, 0x57, 0x08, 0x57,
                           0x08, 0x0d, 0x00, 0x0d, 0x00, 0x0d, 0x00, 0x0d, 0x00, 0x28, 0x00, 0x3e, 0x00});
    CHECK(ae({0x0c47, 0x0c47, 0x0857, 0x0857}, {13, 13, 13, 13}, 40, 62).value() == want);
    CHECK(!ae({0x0c47, 0x0c47, 0x0857, 0x0857}, {0, 13, 13, 13}, 40, 62));
    CHECK(!ae({10410, 3000, 3000, 3000}, {13, 13, 13, 13}, 40, 62)); // power-on exposure is not a Windows value
}

TEST(session_commands_match_pimax_proto) {
    CHECK(dream_air().keepalive_report(72).value() == pad({0x58, 0x04, 0x39, 0x00, 0x48, 0x00}));
    CHECK(!dream_air().keepalive_report(60));
    const auto& al = dream_air().allowlist;
    const std::vector<std::pair<std::vector<std::uint8_t>, std::string>> cases = {
        {pad({0x58, 0x06, 0x1a, 0x05, 0x01, 0x00, 0x00}), "wake"},
        {pad({0x58, 0x05, 0x5b, 0x05, 0x02, 0x02}), "app_unlock"},
        {pad({0x58, 0x04, 0x29, 0x00, 0x01}), "panel_on"},
        {pad({0x58, 0x0d, 0x1a, 0x05, 0x02, 72}), "announce_refresh"},
        {pad({0x58, 0x0d, 0x1a, 0x05, 0x05}), "announce_gpu"},
        {pad({0x58, 0x04, 0x1a, 0x05, 0x04}), "stop"},
        {pad({0x58, 0x04, 0x16, 0x00, 0x1e}), "read_serial"},
        {pad({0x58, 0x03, 0x50, 0x02}), "read_info"},
        {pad({0x58, 0x03, 0x03, 0x03}), "read_version"},
        {pad({0x58, 0x07, 0x0a, 0x02, 0x00, 0x00, 0x00, 0x10}), "read_doc_controller"},
        {pad({0x58, 0x07, 0x0a, 0x00, 0x01, 0x10, 0x00, 0x10}), "read_calib_block"},
        {pad({0x58, 0x05, 0x3c, 0x05, 0x00, 0x01}), "eye_power_on"},
    };
    for (const auto& [r, id] : cases) {
        CHECK(checked_id(al, 0, r) == id);
        // the hidraw write buffer form (report id 0 prefix) is accepted too
        std::vector<std::uint8_t> w{0};
        w.insert(w.end(), r.begin(), r.end());
        CHECK(checked_id(al, 0, w) == id);
    }
}

TEST(everything_else_is_refused) {
    const auto& m = dream_air();
    // explicitly denied, with the reason
    const auto d = refused(m, pad({0x58, 0x03, 0x19, 0x00}));
    CHECK(d.kind == K::Denied && d.why.find("DeviceCheckResult") != std::string::npos);
    CHECK(refused(m, pad({0x58, 0x05, 0x3c, 0x05, 0x00, 0x00})).kind == K::Denied);
    CHECK(refused(m, pad({0x58, 0x10, 0x55, 0x00})).kind == K::Denied);
    // never approved: visor probe, other 0x5b blobs, unknown opcodes
    CHECK(refused(m, pad({0x58, 0x04, 0x2c, 0x00, 0x00})).kind == K::NotListed);
    CHECK(refused(m, pad({0x58, 0x05, 0x5b, 0x05, 0x01, 0x02})).kind == K::NotListed);
    CHECK(refused(m, pad({0xff})).kind == K::NotListed);
    // trailing garbage after an approved pattern
    auto r = pad({0x58, 0x06, 0x1a, 0x05, 0x01, 0x00, 0x00});
    r[40] = 1;
    CHECK(refused(m, r).kind == K::Padding);
    // wrong length
    const auto l = refused(m, {0x58, 0x06, 0x1a, 0x05, 0x01, 0x00, 0x00});
    CHECK(l.kind == K::Length && l.got == 7);
    // wrong interface
    const auto& al = m.allowlist;
    const auto w = al.check(2, pad({0x58, 0x06, 0x1a, 0x05, 0x01, 0x00, 0x00}));
    CHECK(!w && w.error().kind == K::NotListed);
    // unknown ids
    const auto u = al.build("flash_firmware");
    CHECK(!u && u.error().kind == K::UnknownCommand);
}

TEST(gate_writes_one_report_and_rate_limits_once_per_request_commands) {
    const auto& m = dream_air();
    std::vector<std::uint8_t> out;
    Gate g(m.allowlist, 0, vector_writer(out));
    const auto r = m.pair_report(Hand::Left).value();
    CHECK(g.send(r).value_or("") == "start_pair");
    const auto again = g.send(r);
    CHECK(!again && again.error().refusal() && again.error().refusal()->kind == K::TooSoon);
    CHECK(out.size() == 65);
    CHECK((std::vector<std::uint8_t>(out.begin(), out.begin() + 6) == std::vector<std::uint8_t>{0x00, 0x58, 0x04, 0x04, 0x00, 0x02}));
}

/// The approved Windows-capture commands (2026-10-15), built by the stack's own builders, pass the allowlist;
/// anything outside the captured ranges does not.
TEST(windows_capture_commands_pass_and_match_the_stack_builders) {
    const auto& m = dream_air();
    const auto& al = m.allowlist;
    const auto cap = pad({0x58, 0x09, 0x07, 0x03, 0x19, 0x2c, 0x01});
    CHECK(vec(haptic_command(3, 25, 300)) == cap);
    CHECK(checked_id(al, 0, cap) == "haptic");
    CHECK(checked_id(al, 0, vec(haptic_command(2, 100, 65535))) == "haptic");
    CHECK(checked_id(al, 0, vec(haptic_command(2, 1, 12))) == "haptic");
    CHECK(refused(m, vec(haptic_command(4, 25, 300))).kind == K::OutOfRange);
    CHECK(refused(m, pad({0x58, 0x09, 0x07, 0x03, 101, 0x2c, 0x01})).kind == K::OutOfRange);
    CHECK(refused(m, pad({0x58, 0x09, 0x07, 0x03, 0, 0x2c, 0x01})).kind == K::OutOfRange);
    // unpair: left `58 03 05 02`, right `58 03 05 03`
    for (const std::uint8_t slot : std::initializer_list<std::uint8_t>{2, 3}) {
        const auto c = pad({0x58, 0x03, 0x05, slot});
        CHECK(vec(unpair(slot)) == c);
        CHECK(checked_id(al, 0, c) == "unpair");
    }
    CHECK(refused(m, pad({0x58, 0x03, 0x05, 0x04})).kind == K::OutOfRange);
    CHECK(refused(m, pad({0x58, 0x03, 0x05, 0x00})).kind == K::OutOfRange);
    // restart: `58 04 28 00 00`; nothing else with opcode 0x28 passes
    const auto rs = pad({0x58, 0x04, 0x28, 0x00, 0x00});
    CHECK(vec(sys_reset()) == rs);
    CHECK(checked_id(al, 0, rs) == "sys_reset");
    CHECK(refused(m, pad({0x58, 0x04, 0x28, 0x00, 0x01})).kind == K::NotListed);
    CHECK(refused(m, pad({0x58, 0x04, 0x28, 0x01, 0x00})).kind == K::NotListed);
    // set IPD: take 6 `58 06 5c 05 01 a8 16` = 58.00 mm
    const auto ipd = pad({0x58, 0x06, 0x5c, 0x05, 0x01, 0xa8, 0x16});
    CHECK(m.ipd_report(58.0f).value() == ipd);
    CHECK(checked_id(al, 0, ipd) == "ipd_motor");
}

TEST(restart_and_unpair_are_once_per_request) {
    const auto& m = dream_air();
    std::vector<std::uint8_t> out;
    Gate g(m.allowlist, 0, vector_writer(out));
    CHECK(g.send(sys_reset()).value_or("") == "sys_reset");
    auto e = g.send(sys_reset());
    CHECK(!e && e.error().refusal() && e.error().refusal()->kind == K::TooSoon);
    CHECK(g.send(unpair(2)).value_or("") == "unpair");
    e = g.send(unpair(2));
    CHECK(!e && e.error().refusal() && e.error().refusal()->kind == K::TooSoon);
}

TEST(crystal_super_allows_only_its_captured_commands) {
    const auto& m = crystal_super();
    const auto& al = m.allowlist;
    // the cold-start frames from the 2026-10-17 capture build and pass the gate
    const std::vector<std::tuple<std::string, ParamValues, std::vector<std::uint8_t>>> cases = {
        {"wake", {}, {0x58, 0x06, 0x1a, 0x05, 0x01, 0x00, 0x00}},
        {"keepalive", {{"rate", 0x5a}}, {0x58, 0x04, 0x39, 0x00, 0x5a, 0x00}},
        {"announce_refresh", {{"fps", 90}}, {0x58, 0x0d, 0x1a, 0x05, 0x02, 0x5a}},
        {"read_module_type", {}, {0x58, 0x04, 0x3a, 0x05, 0x01}},
        {"eye_power_on", {}, {0x58, 0x05, 0x3c, 0x05, 0x00, 0x01}},
        {"keepalive", {{"rate", 0x00}}, {0x58, 0x04, 0x39, 0x00, 0x00, 0x00}},
    };
    for (const auto& [id, vals, want] : cases) {
        const auto r = al.build(id, vals);
        CHECK(r.has_value());
        if (!r) {
            continue;
        }
        CHECK(std::equal(want.begin(), want.end(), r->begin()));
        std::vector<std::uint8_t> out;
        Gate g(al, 0, vector_writer(out));
        CHECK(g.send(*r).has_value());
        CHECK(out.size() == 65); // report id 0 + the 64-byte report
    }
    // the IPD motor is enabled (user, 2026-10-22), within the motor range; the eye-module power off is captured but
    // not enabled yet (camera exposure: its own entry with the Crystal's Windows ranges, so an all-zero exposure is
    // out of range)
    CHECK(m.ipd_report(63.19f).has_value());
    CHECK(refused(m, pad({0x58, 0x06, 0x5c, 0x05, 0x01, 0x88, 0x13})).kind == K::OutOfRange); // 50.00 mm
    CHECK(m.pair_report(Hand::Left).has_value()); // pairing: the Dream Air frame (user, 2026-10-19)
    CHECK(refused(m, pad({0x58, 0x05, 0x3c, 0x05, 0x00, 0x00})).kind == K::NotListed);
    CHECK(refused(m, pad({0x58, 0x19, 0x51, 0x00, 0x01, 0x32})).kind == K::OutOfRange);
    // controller vibration (user, 2026-10-19): the captured test pulse of take slam passes, AMP 0 and slot 0 do not
    {
        std::vector<std::uint8_t> out;
        Gate g(al, 0, vector_writer(out));
        CHECK(g.send(haptic_command(3, 25, 300)).value_or("") == "haptic");
        CHECK(out.size() == 65);
        const std::uint8_t pulse[] = {0, 0x58, 0x09, 0x07, 0x03, 0x19, 0x2c, 0x01, 0, 0, 0};
        CHECK(std::equal(std::begin(pulse), std::end(pulse), out.begin()));
    }
    CHECK(refused(m, pad({0x58, 0x09, 0x07, 0x03, 0x00, 0x2c, 0x01})).kind == K::OutOfRange);
    CHECK(refused(m, pad({0x58, 0x09, 0x07, 0x00, 0x19, 0x2c, 0x01})).kind == K::OutOfRange);
    CHECK(refused(m, pad({0x58, 0x04, 0x39, 0x00, 0x3c, 0x00})).kind == K::OutOfRange);
}

/// Every frame the session and keepalived send passes both descriptions (the Crystal Super at 90 Hz, the Dream Air
/// at 72 and 90 Hz).
TEST(session_frames_pass_both_descriptions) {
    for (const auto* d : {&dream_air(), &crystal_super()}) {
        const auto& al = d->allowlist;
        CHECK(checked_id(al, 0, vec(wake())) == "wake");
        for (const std::uint8_t r : std::initializer_list<std::uint8_t>{0, 72, 90}) {
            CHECK(checked_id(al, 0, vec(keepalive(r))) == "keepalive");
        }
        for (const std::uint8_t fps : std::initializer_list<std::uint8_t>{72, 90}) {
            const auto v = video_prepare_commands(fps);
            CHECK(checked_id(al, 0, vec(v[0])) == "app_unlock");
            CHECK(checked_id(al, 0, vec(v[1])) == "panel_on");
            CHECK(checked_id(al, 0, vec(v[2])) == "announce_refresh");
            CHECK(checked_id(al, 0, vec(v[3])) == "announce_gpu");
        }
        for (const std::uint8_t slot : std::initializer_list<std::uint8_t>{2, 3}) {
            const auto a = attach_commands(slot);
            CHECK(checked_id(al, 0, vec(a[0])) == "read_serial");
            CHECK(checked_id(al, 0, vec(a[1])) == "read_info");
            CHECK(checked_id(al, 0, vec(a[2])) == "read_doc_controller");
            CHECK(checked_id(al, 0, vec(a[3])) == "read_version");
        }
        CHECK(checked_id(al, 0, vec(calib::read_block_request(0x0530))) == "read_calib_block");
        for (const std::uint8_t slot : std::initializer_list<std::uint8_t>{2, 3}) {
            CHECK(checked_id(al, 0, vec(haptic_command(slot, 1, 1))) == "haptic");
            CHECK(checked_id(al, 0, vec(haptic_command(slot, 100, 65535))) == "haptic");
        }
        CHECK(checked_id(al, 0, vec(keepalive(60))).starts_with("<"));
    }
}

TEST(descriptor_rules_are_enforced) {
    const std::string base = "[device]\nid = \"x\"\nname = \"X\"\nvendor = \"v\"\nstatus = \"mock\"\nusb_vid = 1\n"
                             "[display]\n[tracking]\n[controllers]\n[eye_tracking]\n[ipd]\n[cameras]\n[audio]\n"
                             "[calibration]\n[onboarding]\nsteps = [\"detect\"]\n";
    CHECK(Descriptor::parse(base).has_value());
    // a mock may not send anything
    CHECK(!Descriptor::parse(base + "[[hid]]\nid = \"a\"\nbytes = \"58\"\nsource = \"s\"\n"));
    // unknown onboarding step, unknown array table, missing table
    std::string s = base;
    s.replace(s.find("\"detect\""), 8, "\"nope\"");
    CHECK(!Descriptor::parse(s));
    CHECK(!Descriptor::parse(base + "[[what]]\n"));
    s = base;
    s.erase(s.find("[audio]\n"), 8);
    CHECK(!Descriptor::parse(s));
    // a state-changing command needs `approved`; a field needs a constraint
    std::string real = base;
    real.replace(real.find("\"mock\""), 6, "\"real\"");
    real.replace(real.find("usb_vid = 1\n"), 12, "usb_vid = 1\nusb_pids = [2]\n");
    CHECK(Descriptor::parse(real).has_value());
    CHECK(!Descriptor::parse(real + "[[hid]]\nid = \"a\"\nbytes = \"58\"\nsource = \"s\"\npolicy = \"once_per_request\"\n"));
    CHECK(!Descriptor::parse(real + "[[hid]]\nid = \"a\"\nbytes = \"58 {x:u8}\"\nsource = \"s\"\n"));
    CHECK(!Descriptor::parse(real + "[[hid]]\nid = \"a\"\nbytes = \"58\"\nparams = [\"x=1\"]\nsource = \"s\"\n"));
    CHECK(!Descriptor::parse(real + "[[hid]]\nid = \"a\"\nbytes = \"58 {x:u32}\"\nparams = [\"x=1\"]\nsource = \"s\"\n"));
    CHECK(Descriptor::parse(real + "[[hid]]\nid = \"a\"\nbytes = \"58 {x:u16le}\"\nparams = [\"x=1..0x10\"]\nsource = \"s\"\n").has_value());
    CHECK(!Descriptor::parse(real + "[[hid]]\nid = \"a\"\nbytes = \"58 {x:u8}\"\nparams = [\"x=5..1\"]\nsource = \"s\"\n"));
}

TEST(ipd_motor_frame_passes_the_crystal_super_gate) {
    // the frame of the 2026-10-17 capture take ipd: 63.19 mm = 58 06 5c 05 01 af 18
    const auto f = ipd_motor(63.19f);
    CHECK((std::vector<std::uint8_t>(f.begin(), f.end()) == pad({0x58, 0x06, 0x5c, 0x05, 0x01, 0xaf, 0x18})));
    CHECK(ipd_motor(40.0f) == ipd_motor(58.0f)); // clamped to the motor range
    const auto& m = load("crystal-super");
    CHECK(m.caps.ipd_motor);
    std::vector<std::uint8_t> out;
    Gate g(m.allowlist, 0, vector_writer(out));
    CHECK(g.send(ipd_motor(66.0f)).value_or("") == "ipd_motor");
    CHECK(ipd::parse_request("63.5\n") == std::optional<float>(63.5f));
    CHECK(!ipd::parse_request("abc"));
    CHECK(!ipd::parse_request("200"));
}

PIBRIDGE_TEST_MAIN()
