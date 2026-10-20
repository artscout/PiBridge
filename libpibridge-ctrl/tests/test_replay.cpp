// SPDX-License-Identifier: MIT
// Twins of omarchyvr pimax-hands/tests/replay.rs (MIT OR Apache-2.0): the REAL button-by-button capture
// (rust/fixtures/ctrl_buttons_hub2.rec.gz, p82.rec.gz and ctrl_marks.json in the omarchyvr checkout, read only;
// skipped without them), decoded with libpibridge-hmd's pimax-proto port.
#include "pibridge/ctrl/mapping.h"
#include "pibridge/ctrl/report.h"
#include "pibridge/hmd/proto.h"
#include "pibridge_test.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <unistd.h>
#include <vector>
#include <zlib.h>

using namespace pibridge::ctrl;
namespace hmd = pibridge::hmd;

namespace {

std::string fixtures() {
    return PIBRIDGE_CTRL_FIXTURES;
}

std::vector<std::pair<double, hmd::Report>> load_rec(const std::string& name) {
    std::vector<std::pair<double, hmd::Report>> out;
    gzFile f = gzopen((fixtures() + "/" + name).c_str(), "rb");
    if (f == nullptr) {
        return out;
    }
    unsigned char rec[72];
    while (gzread(f, rec, sizeof rec) == static_cast<int>(sizeof rec)) {
        double t;
        std::memcpy(&t, rec, 8);
        hmd::Report b{};
        std::memcpy(b.data(), rec + 8, 64);
        out.emplace_back(t, b);
    }
    gzclose(f);
    return out;
}

std::map<std::string, double> marks() {
    std::map<std::string, double> m;
    FILE* f = std::fopen((fixtures() + "/ctrl_marks.json").c_str(), "r");
    if (f == nullptr) {
        return m;
    }
    char line[256];
    while (std::fgets(line, sizeof line, f) != nullptr) {
        std::string l = line;
        while (!l.empty() && (l.back() == '\n' || l.back() == ' ' || l.back() == ',')) {
            l.pop_back();
        }
        const auto p = l.rfind(": ");
        if (p == std::string::npos) {
            continue;
        }
        std::string k = l.substr(0, p);
        while (!k.empty() && (k.front() == ' ' || k.front() == '"')) {
            k.erase(0, 1);
        }
        while (!k.empty() && k.back() == '"') {
            k.pop_back();
        }
        m[k] = std::strtod(l.c_str() + p + 2, nullptr);
    }
    std::fclose(f);
    return m;
}

Report as_report(const hmd::ControllerReport& c) {
    return Report{c.trigger, c.grip, c.stick_x, c.stick_y, c.buttons, c.touch};
}

struct Sample {
    double t;
    std::array<float, 5> curls;
    ThumbState thumb;
    float trigger, grip;
};

std::vector<Sample> replay(std::uint8_t slot) {
    const auto rows = load_rec("ctrl_buttons_hub2.rec.gz");
    const double t0 = rows[0].first;
    HandState st{Params{}};
    std::vector<Sample> out;
    for (const auto& [t, b] : rows) {
        if (b[3] != slot) {
            continue;
        }
        const auto c = hmd::decode_controller(b);
        CHECK(c.has_value());
        const Input inp = input_from_report(as_report(*c));
        const double ns = (t - t0) * 1e9;
        const auto curls = st.update(ns <= 0 ? 0 : static_cast<std::uint64_t>(ns), inp).to_array();
        out.push_back({t - t0, curls, st.thumb_state(), inp.trigger, inp.grip});
    }
    return out;
}

const Sample* first(const std::vector<Sample>& s, auto pred) {
    for (const Sample& x : s) {
        if (pred(x)) {
            return &x;
        }
    }
    return nullptr;
}

} // namespace

TEST(bit_constants_equal_the_protocol_crate) {
    CHECK(report::BTN_A == hmd::BTN_A && report::BTN_B == hmd::BTN_B && report::BTN_STICK == hmd::BTN_STICK &&
          report::BTN_MENU == hmd::BTN_MENU);
    const auto rows = load_rec("ctrl_buttons_hub2.rec.gz");
    int checked = 0;
    for (std::size_t i = 0; i < rows.size(); i += 7) {
        const auto c = hmd::decode_controller(rows[i].second);
        if (!c) {
            continue;
        }
        const auto tb = c->touch_bits();
        const std::uint32_t f = input_from_report(as_report(*c)).flags;
        CHECK(tb.a == ((f & flags::TOUCH_BTN1) != 0));
        CHECK(tb.b == ((f & flags::TOUCH_BTN2) != 0));
        CHECK(tb.stick == ((f & flags::TOUCH_STICK) != 0));
        CHECK(((c->buttons & hmd::BTN_A) != 0) == ((f & flags::CLICK_BTN1) != 0));
        CHECK(((c->buttons & hmd::BTN_B) != 0) == ((f & flags::CLICK_BTN2) != 0));
        CHECK(((c->buttons & hmd::BTN_STICK) != 0) == ((f & flags::CLICK_STICK) != 0));
        CHECK(((c->buttons & hmd::BTN_MENU) != 0) == ((f & flags::CLICK_MENU) != 0));
        checked++;
    }
    CHECK(checked > 10000);
}

TEST(left_controller_every_control_moves_the_right_finger) {
    const auto s = replay(2);
    const auto m = marks();
    const auto rows = load_rec("ctrl_buttons_hub2.rec.gz");
    const double t0 = rows[0].first;
    const auto mark = [&](const char* k) { return m.at(k) - t0; };
    const Params p;
    for (std::size_t i = 0; i < 200; i++) {
        CHECK(s[i].thumb == ThumbState::Btn1);
        CHECK(std::fabs(s[i].curls[4] - p.thumb_btn1) < 0.03f);
    }
    const Sample* trig = first(s, [](const Sample& x) { return x.trigger > 0.8f; });
    CHECK(trig != nullptr);
    CHECK(trig->t >= mark("l trigger") - 6.5 && trig->t <= mark("l trigger") + 0.5);
    float peak = 0.0f, lower = 0.0f;
    for (const Sample& x : s) {
        if (std::fabs(x.t - trig->t) < 2.0) {
            peak = std::fmax(peak, x.curls[3]);
        }
        if (std::fabs(x.t - trig->t) < 0.3 && x.grip < 0.1f) {
            lower = std::fmax(lower, x.curls[2]);
        }
    }
    CHECK(peak > 0.75f);
    CHECK(lower < 0.4f);
    const Sample* grip = first(s, [](const Sample& x) { return x.grip > 0.5f; });
    CHECK(grip != nullptr);
    CHECK(grip->t >= mark("l grip") - 6.5 && grip->t <= mark("l grip") + 2.0);
    float fist = 0.0f, idx = 0.0f;
    for (const Sample& x : s) {
        if (std::fabs(x.t - grip->t) < 2.5) {
            fist = std::fmax(fist, x.curls[2]);
        }
        if (std::fabs(x.t - grip->t) < 0.3 && x.trigger < 0.1f) {
            idx = std::fmax(idx, x.curls[3]);
        }
        for (float c : x.curls) {
            CHECK(c >= 0.0f && c <= 1.0f);
        }
    }
    CHECK(fist > 0.95f);
    CHECK(idx < 0.3f);
    for (auto [at, after] : {std::pair{trig->t, 1.0}, std::pair{grip->t, 0.9}}) {
        std::size_t n = 0;
        for (const Sample& x : s) {
            if (x.t >= at - 0.15 && x.t <= at + after) {
                n++;
                CHECK(x.thumb == ThumbState::Rest && x.curls[4] < 0.05f);
            }
        }
        CHECK(n > 100);
    }
    const auto thumb_at = [&](const char* key, ThumbState state, double lead, double slack) {
        const Sample* hit = first(s, [&](const Sample& x) {
            return x.t >= mark(key) - lead && x.t <= mark(key) + slack && x.thumb == state && x.curls[4] > 0.35f;
        });
        CHECK(hit != nullptr);
    };
    thumb_at("l a", ThumbState::Btn1, 6.5, 1.0);
    thumb_at("l b", ThumbState::Btn2, 6.5, 1.0);
    thumb_at("l sick click", ThumbState::Stick, 6.5, 1.0);
    thumb_at("l menu", ThumbState::Menu, 6.5, 1.0);
    thumb_at("l stick up", ThumbState::Stick, 6.5, 1.5);
    thumb_at("l stck right", ThumbState::Stick, 6.5, 1.5);
    CHECK(first(s, [&](const Sample& x) { return x.thumb == ThumbState::Btn1 && x.curls[4] > p.thumb_btn1 + 0.04f; }));
    int checked = 0;
    std::size_t k = 0;
    for (const auto& [t, b] : rows) {
        if (b[3] != 2) {
            continue;
        }
        if (k >= s.size()) {
            break;
        }
        const auto c = hmd::decode_controller(b);
        const double mag = std::hypot(static_cast<double>(c->stick_x), static_cast<double>(c->stick_y)) / 127.0;
        if (mag > 0.3 && s[k].t > 5.0) {
            CHECK(s[k].thumb == ThumbState::Stick);
            checked++;
        }
        k++;
    }
    CHECK(checked > 500);
}

TEST(right_controller_trigger_and_grip_of_the_first_round) {
    const auto s = replay(3);
    CHECK(s.size() > 40000);
    const Params p;
    for (const Sample& x : s) {
        CHECK(x.thumb == ThumbState::Rest);
        CHECK(std::fabs(x.curls[3] - p.index_rest) < 0.02f && std::fabs(x.curls[2] - p.grip_rest) < 0.02f);
    }
}

TEST(the_p82_capture_with_squeezes_on_both_controllers) {
    const auto rows = load_rec("p82.rec.gz");
    const double t0 = rows[0].first;
    HandState st[2] = {HandState{Params{}}, HandState{Params{}}};
    float mx[2][5] = {};
    std::size_t n[2] = {0, 0};
    for (const auto& [t, b] : rows) {
        const auto c = hmd::decode_controller(b);
        if (!c) {
            continue;
        }
        const int k = c->slot == 2 ? 0 : (c->slot == 3 ? 1 : -1);
        if (k < 0) {
            continue;
        }
        const double ns = (t - t0) * 1e9;
        const auto curls = st[k].update(ns <= 0 ? 0 : static_cast<std::uint64_t>(ns), input_from_report(as_report(*c))).to_array();
        n[k]++;
        for (int i = 0; i < 5; i++) {
            mx[k][i] = std::fmax(mx[k][i], curls[i]);
            CHECK(curls[i] >= 0.0f && curls[i] <= 1.0f);
        }
    }
    CHECK(n[0] > 1000 && n[1] > 1000);
    CHECK(std::fmax(mx[0][3], mx[1][3]) > 0.7f && std::fmax(mx[0][2], mx[1][2]) > 0.7f);
}

int main(int argc, char** argv) {
    if (::access((fixtures() + "/ctrl_buttons_hub2.rec.gz").c_str(), R_OK) != 0) {
        std::printf("skipped: no pimax-hands fixtures at %s\n", PIBRIDGE_CTRL_FIXTURES);
        return 77;
    }
    return pibridge::test::run_all(argc, argv);
}
