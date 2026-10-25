// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-hands/src/ffi.rs, src/inhand_ffi.rs and tests/c_abi.rs (MIT OR Apache-2.0),
// through the archive (libpibridge_pimax.a) the way the Monado driver links it. PIMAX_CTRL_REST / PIMAX_CTRL_AIM are read
// once per process, so the configuration cases run in forked children.
#include "pimax_ctrlfuse.h"
#include "pimax_hands.h"
#include "pimax_inhand.h"

#include "pibridge_test.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <unistd.h>

namespace {

constexpr std::uint64_t MS = 1000000;

/// pimax_hands_controller_describe in a child with PIMAX_CTRL_REST / PIMAX_CTRL_AIM set (nullptr = unset).
std::pair<int, std::string> describe_with(const char* rest, const char* aim) {
    int fd[2];
    if (::pipe(fd) != 0) {
        return {-1, ""};
    }
    const pid_t pid = ::fork();
    if (pid == 0) {
        ::close(fd[0]);
        if (rest) {
            ::setenv("PIMAX_CTRL_REST", rest, 1);
        } else {
            ::unsetenv("PIMAX_CTRL_REST");
        }
        if (aim) {
            ::setenv("PIMAX_CTRL_AIM", aim, 1);
        } else {
            ::unsetenv("PIMAX_CTRL_AIM");
        }
        char buf[512];
        const int r = pimax_hands_controller_describe(buf, sizeof buf);
        const std::string out = std::to_string(r) + "|" + buf;
        if (::write(fd[1], out.data(), out.size()) < 0) {
            ::_exit(2);
        }
        ::_exit(0);
    }
    ::close(fd[1]);
    std::string s;
    char b[256];
    ssize_t n;
    while ((n = ::read(fd[0], b, sizeof b)) > 0) {
        s.append(b, static_cast<std::size_t>(n));
    }
    ::close(fd[0]);
    int st = 0;
    ::waitpid(pid, &st, 0);
    const auto bar = s.find('|');
    if (bar == std::string::npos) {
        return {-1, s};
    }
    return {std::atoi(s.substr(0, bar).c_str()), s.substr(bar + 1)};
}

bool pose_eq(const pimax_hands_pose& a, const pimax_hands_pose& b) {
    return std::memcmp(&a, &b, sizeof a) == 0;
}

} // namespace

// ---------- ffi.rs ----------

TEST(controller_config_defaults_overrides_and_errors) {
    const auto d = describe_with(nullptr, nullptr);
    CHECK(d.first == 0);
    CHECK(d.second == "rest 0.15 m side, 0.16 m down, 0.38 m forward, grip pitch 45 deg; aim level (yaw only)");
    const auto l = describe_with("legacy", "head:2");
    CHECK(l.first == 0);
    CHECK(l.second.find("(legacy)") != std::string::npos && l.second.find("2.00 m") != std::string::npos);
    const auto bad = describe_with("nope", "sideways");
    CHECK(bad.first == 1);
    CHECK(bad.second.find("rest 0.15 m side") == 0 && bad.second.find("aim level (yaw only)") != std::string::npos);
    CHECK(bad.second.find("INVALID SETTING IGNORED: PIMAX_CTRL_REST=\"nope\": want desk, legacy or "
                          "side,down,fwd[,pitch_deg]; PIMAX_CTRL_AIM=\"sideways\": want level, head or "
                          "head:<0.2..20 m>") != std::string::npos);
}

TEST(controller_poses_ffi_matches_rust_and_tolerates_null) {
    pimax_hands_controller_poses(1, nullptr, nullptr, nullptr);
    pimax_hands_aim_to_grip(nullptr);
    pimax_hands_tracked_aim_to_grip(nullptr);
    pimax_hands_pose t{};
    pimax_hands_tracked_aim_to_grip(&t);
    CHECK(std::fabs(t.orientation[0] - 0.5f) < 1e-6f && std::fabs(t.position[2] - 0.095f) < 5e-4f);
    CHECK(pimax_hands_controller_describe(nullptr, 0) <= 1);
    pimax_hands_pose a{}, g{}, a2{}, g2{};
    const float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // degenerate quaternion -> identity
    pimax_hands_controller_poses(0, zero, &a, &g);
    pimax_hands_controller_poses(0, nullptr, &a2, &g2);
    CHECK(pose_eq(a, a2) && pose_eq(g, g2));
    char buf[8];
    std::memset(buf, 'x', sizeof buf);
    pimax_hands_controller_describe(buf, sizeof buf);
    CHECK(buf[7] == 0); // truncated and terminated
}

TEST(lifecycle_and_null_arguments) {
    CHECK(pimax_hands_abi_version() == PIMAX_HANDS_ABI_VERSION);
    pimax_hands_destroy(nullptr);
    pimax_hands_reset(nullptr);
    pimax_hands_update(nullptr, 1, nullptr);
    pimax_hands_out out{};
    pimax_hands_get(nullptr, &out);
    pimax_hands_out zero{};
    CHECK(std::memcmp(&out, &zero, sizeof out) == 0);
    pimax_hands_grip_to_wrist(0, nullptr);
    pimax_hands_demo_input(0, 0, nullptr);
    pimax_hands* h = pimax_hands_create();
    CHECK(h != nullptr);
    pimax_hands_update(h, 1, nullptr);
    pimax_hands_get(h, nullptr);
    pimax_hands_reset(h);
    pimax_hands_destroy(h);
}

TEST(update_and_get_round_trip) {
    pimax_hands* h = pimax_hands_create();
    pimax_hands_input i{1.0f, 1.0f, 0.0f, 0.0f, PIMAX_HANDS_TOUCH_STICK};
    std::uint64_t t = 100 * MS;
    for (int k = 0; k < 200; k++) {
        t += 4 * MS;
        pimax_hands_update(h, t, &i);
    }
    pimax_hands_out out{};
    pimax_hands_get(h, &out);
    CHECK(std::fabs(out.curl[3] - 0.85f) < 0.01f);
    CHECK(std::fabs(out.curl[2] - 1.0f) < 0.01f && out.curl[0] == out.curl[2] && out.curl[1] == out.curl[2]);
    CHECK(std::fabs(out.curl[4] - 0.55f) < 0.02f);
    CHECK(out.thumb_state == 3);
    CHECK(std::fabs(out.middle.joint_curls[1] - out.curl[2] * 1.859f) < 1e-3f);
    CHECK(std::fabs(out.index.joint_curls[1] - out.curl[3] * 1.43f) < 1e-3f);
    CHECK(std::fabs(out.thumb.joint_curls[1] - out.curl[4] * 1.57f) < 1e-3f);
    i.flags = 0;
    for (int k = 0; k < 200; k++) {
        t += 4 * MS;
        pimax_hands_update(h, t, &i);
    }
    pimax_hands_get(h, &out);
    CHECK(out.thumb_state == 0);
    pimax_hands_destroy(h);
}

TEST(grip_to_wrist_matches_the_rust_api) {
    const double want[2][3] = {{-0.0289, 0.0629, 0.0146}, {0.0289, 0.0629, 0.0146}};
    for (int right = 0; right < 2; right++) {
        pimax_hands_pose p{};
        pimax_hands_grip_to_wrist(right, &p);
        for (int k = 0; k < 3; k++) {
            CHECK(std::fabs(p.position[k] - want[right][k]) < 0.0005);
        }
        float n = 0.0f;
        for (float c : p.orientation) {
            n += c * c;
        }
        CHECK(std::fabs(n - 1.0f) < 1e-5f);
    }
}

TEST(demo_input_matches_the_rust_api) {
    const struct {
        std::uint64_t ns;
        int right;
        float trigger, grip;
        std::uint32_t flags;
    } cases[] = {
        {0, 0, 0.0f, 0.0f, 0},
        {2000 * MS, 0, 0.0f, 1.0f, 0},
        {4000 * MS, 1, 0.0f, 0.0f, PIMAX_HANDS_TOUCH_BTN2 | PIMAX_HANDS_CLICK_BTN2},
        {6700 * MS, 0, 0.0f, 0.0f, PIMAX_HANDS_TOUCH_BTN2 | PIMAX_HANDS_CLICK_BTN2},
        {7500 * MS, 1, 0.0f, 0.0f, 0},
    };
    for (const auto& c : cases) {
        pimax_hands_input o{};
        pimax_hands_demo_input(c.ns, c.right, &o);
        CHECK(std::fabs(o.trigger - c.trigger) < 1e-6f && std::fabs(o.grip - c.grip) < 1e-6f && o.flags == c.flags);
    }
}

// ---------- inhand_ffi.rs ----------

TEST(inhand_null_is_harmless) {
    pimax_inhand_report(nullptr, 0, nullptr, 0, 0, 0, 0, 0, 0);
    CHECK(pimax_inhand_held(nullptr, 0) == 0);
    pimax_inhand_destroy(nullptr);
    CHECK(pimax_inhand_describe(nullptr, 0, nullptr, 0) == 0);
}

TEST(a_button_press_shows_the_controller) {
    pimax_inhand* h = pimax_inhand_create(0);
    const std::uint8_t raw[64] = {};
    pimax_inhand_report(h, 1000, raw, 0, 0, 0, 0, 0, 0);
    CHECK(pimax_inhand_held(h, 2000) == 0);
    pimax_inhand_report(h, 4000000, raw, 0, 0, 0, 0, 0x02, 0);
    CHECK(pimax_inhand_held(h, 5000000) == 1);
    char b[64];
    const std::size_t n = pimax_inhand_describe(h, 5000000, b, sizeof b);
    CHECK(n == 12 && std::string(b) == "held (input)");
    pimax_inhand_destroy(h);
}

TEST(mode_names_round_trip) {
    CHECK(std::string(pimax_inhand_mode_name(PIMAX_INHAND_MODE_AUTO)) == "auto");
    CHECK(std::string(pimax_inhand_mode_name(PIMAX_INHAND_MODE_ALWAYS)) == "always");
    CHECK(std::string(pimax_inhand_mode_name(PIMAX_INHAND_MODE_OFF)) == "off");
    CHECK(std::string(pimax_inhand_mode_name(17)) == "auto");
    // the names parse back (through the run-time request file)
    const char* d = std::getenv("XDG_RUNTIME_DIR");
    const std::string req = std::string(d ? d : "/tmp") + "/pimax-ctrl-inhand-mode";
    for (int m : {PIMAX_INHAND_MODE_AUTO, PIMAX_INHAND_MODE_ALWAYS, PIMAX_INHAND_MODE_OFF}) {
        std::ofstream(req) << pimax_inhand_mode_name(m);
        CHECK(pimax_inhand_poll_mode() == m);
        CHECK(::access(req.c_str(), F_OK) != 0);
    }
    CHECK(pimax_inhand_poll_mode() == -1);
}

// ---------- tests/c_abi.rs ----------

TEST(struct_layout_matches_the_c_header) {
    CHECK(sizeof(pimax_hands_input) == 20 && alignof(pimax_hands_input) == 4);
    CHECK(offsetof(pimax_hands_input, trigger) == 0 && offsetof(pimax_hands_input, grip) == 4 &&
          offsetof(pimax_hands_input, stick_x) == 8 && offsetof(pimax_hands_input, stick_y) == 12 &&
          offsetof(pimax_hands_input, flags) == 16);
    CHECK(sizeof(pimax_hands_finger) == 20 && offsetof(pimax_hands_finger, joint_curls) == 4);
    CHECK(sizeof(pimax_hands_out) == 124 && offsetof(pimax_hands_out, ring) == 20 &&
          offsetof(pimax_hands_out, thumb) == 80 && offsetof(pimax_hands_out, curl) == 100 &&
          offsetof(pimax_hands_out, thumb_state) == 120);
    CHECK(sizeof(pimax_hands_pose) == 28 && offsetof(pimax_hands_pose, position) == 16);
    CHECK(PIMAX_HANDS_ABI_VERSION == 1 && PIMAX_HANDS_TOUCH_BTN1 == 0x1 && PIMAX_HANDS_TOUCH_BTN2 == 0x2 &&
          PIMAX_HANDS_TOUCH_STICK == 0x4 && PIMAX_HANDS_CLICK_BTN1 == 0x8 && PIMAX_HANDS_CLICK_BTN2 == 0x10 &&
          PIMAX_HANDS_CLICK_STICK == 0x20 && PIMAX_HANDS_CLICK_MENU == 0x40);
    CHECK(sizeof(pimax_ctrlfuse_info) == 8);
}

TEST(c_program_drives_a_hand_through_the_staticlib) {
    // the steps of pimax-hands' tests/c/session.c in short: the rest pose, a squeeze, the report path.
    pimax_hands* h = pimax_hands_create();
    pimax_hands_out out;
    std::memset(&out, 0xff, sizeof out);
    pimax_hands_get(h, &out);
    CHECK(out.thumb_state == PIMAX_HANDS_THUMB_REST && out.curl[3] < 0.2f && out.curl[2] < 0.4f);
    std::uint64_t t = 5000 * MS;
    for (int i = 0; i < 300; i++) {
        t += 4 * MS;
        pimax_hands_update_report(h, t, 255, 255, 0, 0, 0x02, 0x01);
    }
    pimax_hands_get(h, &out);
    CHECK(out.thumb_state == PIMAX_HANDS_THUMB_BTN1 && out.curl[4] > 0.55f);
    pimax_hands_destroy(h);
}

TEST(header_declares_every_exported_function) {
    std::ifstream f(PIBRIDGE_HANDS_HEADER);
    std::stringstream ss;
    ss << f.rdbuf();
    const std::string hdr = ss.str();
    for (const char* fn : {"pimax_hands_abi_version", "pimax_hands_create", "pimax_hands_destroy", "pimax_hands_reset",
                           "pimax_hands_update", "pimax_hands_update_report", "pimax_hands_get",
                           "pimax_hands_grip_to_wrist", "pimax_hands_demo_input"}) {
        CHECK(hdr.find(std::string(fn) + "(") != std::string::npos);
    }
    // the archive exports them: they link (monado_abi_symbols checks that only pimax_* is exported)
    CHECK(&pimax_hands_create != nullptr && &pimax_hands_update != nullptr && &pimax_hands_get != nullptr &&
          &pimax_hands_grip_to_wrist != nullptr);
}

PIBRIDGE_TEST_MAIN()
