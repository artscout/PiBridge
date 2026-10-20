// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-proto/tests/{decoders,replay}.rs (MIT OR Apache-2.0): the decoders on real captures.
// The big captures come from PIBRIDGE_FIXTURES_DIR (an omarchyvr checkout); without it those tests say "skipped".
#include "common.h"
#include "pibridge_test.h"

#include "pibridge/hmd/proto.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>

using namespace pibridge::hmd;

TEST(imu_p83_real_capture) {
    const auto rows = testutil::load_rec("p83.rec.gz");
    if (!rows) {
        return;
    }
    CHECK(rows->size() > 60'000);
    std::optional<std::uint32_t> prev;
    std::size_t n = 0;
    double sum_g = 0;
    int bad = 0;
    for (const auto& [t, b] : *rows) {
        const auto s = decode_imu(b);
        if (!s) {
            bad++; // every p83 record is an IMU report
            continue;
        }
        if (prev) {
            const std::uint32_t d = (*s)[0].ts_us - *prev;
            bad += !(d >= 1 && d < 5000); // timestamps advance by about 2 ms
        }
        prev = (*s)[0].ts_us;
        bad += (*s)[1].ts_us != (*s)[0].ts_us + 1000;
        const auto& a = (*s)[0].accel_g;
        sum_g += std::sqrt(static_cast<double>(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]));
        n++;
    }
    CHECK(bad == 0);
    const double mean = sum_g / static_cast<double>(n);
    CHECK(mean >= 0.5 && mean < 2.0);
}

/// C++ decode == python `pimax_controllers.decode()` on every controller report we have.
TEST(controller_equals_python_on_all_captures) {
    auto rows = testutil::load_rec("p82.rec.gz");
    const auto more = testutil::load_rec("ctrl_buttons_hub2.rec.gz");
    const auto gp = testutil::fixture("ctrl_golden.csv.gz");
    if (!rows || !more || !gp) {
        return;
    }
    rows->insert(rows->end(), more->begin(), more->end());
    const auto graw = testutil::gunzip(*gp);
    CHECK(graw.has_value());
    std::istringstream in(std::string(graw->begin(), graw->end()));
    std::vector<std::vector<long long>> gold;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        std::vector<long long> v;
        std::stringstream ls(line);
        std::string x;
        while (std::getline(ls, x, ',')) {
            v.push_back(std::stoll(x));
        }
        gold.push_back(std::move(v));
    }
    std::vector<ControllerReport> reps;
    for (const auto& [t, b] : *rows) {
        if (auto c = decode_controller(b)) {
            reps.push_back(*c);
        }
    }
    CHECK(reps.size() == gold.size());
    CHECK(reps.size() > 100'000);
    std::size_t bad = 0;
    for (std::size_t i = 0; i < std::min(reps.size(), gold.size()); i++) {
        const auto& c = reps[i];
        const long long bm = ((c.buttons & BTN_B) != 0) | (((c.buttons & BTN_A) != 0) << 1) |
                             (((c.buttons & BTN_STICK) != 0) << 2) | (((c.buttons & BTN_MENU) != 0) << 3);
        const TouchBits t = c.touch_bits();
        // the golden was made with the old decoder, where bit 4 rested at 1 on the left and 0 on the right
        const bool rest4 = c.slot == SLOT_LEFT;
        const bool t4_old = t.hand_on != rest4;
        const long long tm = t.a | (t.b << 1) | (t.stick << 2) | (t4_old << 3);
        const std::vector<long long> mine = {c.slot,         c.ts_us,        c.stick_x,      c.stick_y,
                                             c.trigger,      c.grip,         c.battery,      c.charging,
                                             bm,             tm,             c.raw_accel[0], c.raw_accel[1],
                                             c.raw_accel[2], c.raw_gyro[0],  c.raw_gyro[1],  c.raw_gyro[2]};
        if (mine != gold[i] && bad++ < 3) {
            std::fprintf(stderr, "  report #%zu differs\n", i);
        }
    }
    CHECK(bad == 0);
}

TEST(log_lines_from_real_firmware_capture) {
    const auto chunks = testutil::load_rec("if3_log_coldstart.rec.gz");
    if (!chunks) {
        return;
    }
    const auto strim = [](std::string s) {
        const auto b = s.find_first_not_of(" \t\r\n\v\f");
        if (b == std::string::npos) {
            return std::string{};
        }
        return s.substr(b, s.find_last_not_of(" \t\r\n\v\f") - b + 1);
    };
    // reference: python pimax_logtail.py (rstrip NULs per chunk, split on \n, strip) plus libpibridge-hmd's rule of
    // dropping CR inside a line
    std::string buf;
    std::vector<std::string> want;
    for (const auto& [t, c] : *chunks) {
        std::size_t end = c.size();
        while (end > 0 && c[end - 1] == 0) {
            end--;
        }
        for (std::size_t i = 0; i < end; i++) {
            if (c[i] != '\r') {
                buf += static_cast<char>(c[i]);
            }
        }
        std::size_t p = 0;
        while ((p = buf.find('\n')) != std::string::npos) {
            const std::string l = strim(buf.substr(0, p + 1));
            buf.erase(0, p + 1);
            if (!l.empty()) {
                want.push_back(l);
            }
        }
    }
    LineSplitter sp;
    std::vector<std::string> got;
    for (const auto& [t, c] : *chunks) {
        sp.push(c, [&](std::string_view l) {
            const std::string s = strim(std::string(l));
            if (!s.empty()) {
                got.push_back(s);
            }
        });
    }
    CHECK(want.size() > 50);
    CHECK(got == want);
    CHECK(std::any_of(got.begin(), got.end(), [](const std::string& l) { return l.find("request_panel_on") != std::string::npos; }));
}

// ------------------------------------------------------------------------------------------------ replay.rs

static std::map<std::string, double> marks() {
    std::map<std::string, double> m;
    const auto p = testutil::fixture("ctrl_marks.json");
    if (!p) {
        return m;
    }
    std::istringstream in(testutil::read_text(*p).value_or(""));
    std::string l;
    while (std::getline(in, l)) {
        while (!l.empty() && (l.back() == ',' || l.back() == ' ' || l.back() == '\r')) {
            l.pop_back();
        }
        const auto sep = l.rfind(": ");
        if (sep == std::string::npos) {
            continue;
        }
        std::string k = l.substr(0, sep);
        k.erase(0, k.find_first_not_of(" \t"));
        if (k.size() >= 2 && k.front() == '"' && k.back() == '"') {
            k = k.substr(1, k.size() - 2);
        }
        try {
            m[k] = std::stod(l.substr(sep + 2));
        } catch (...) {
        }
    }
    return m;
}

using TimedCtrl = std::vector<std::pair<double, ControllerReport>>;

TEST(left_controller_buttons_capture) {
    const auto rows = testutil::load_rec("ctrl_buttons_hub2.rec.gz");
    const auto m = marks();
    if (!rows || m.empty()) {
        return;
    }
    CHECK(rows->size() > 80'000);
    const double t0 = (*rows)[0].first;
    TimedCtrl left;
    for (const auto& [t, b] : *rows) {
        if (b[3] == 2) {
            left.emplace_back(t - t0, decode_controller(b).value());
        }
    }
    CHECK(m.size() == 10);
    const auto mk = [&](const std::string& k) { return m.at(k) - t0; };
    constexpr double LEAD = 6.5;
    using Pred = std::function<bool(const ControllerReport&)>;
    struct Row {
        const char* name;
        const char* mark;
        double slack;
        Pred pred;
    };
    const std::vector<Row> table = {
        {"trigger", "l trigger", 0.5, [](const ControllerReport& c) { return c.trigger > 200; }},
        {"grip", "l grip", 2.0, [](const ControllerReport& c) { return c.grip > 100; }},
        {"A", "l a", 0.5, [](const ControllerReport& c) { return (c.buttons & BTN_A) != 0; }},
        {"B", "l b", 0.5, [](const ControllerReport& c) { return (c.buttons & BTN_B) != 0; }},
        {"stick", "l sick click", 0.5, [](const ControllerReport& c) { return (c.buttons & BTN_STICK) != 0; }},
        {"menu", "l menu", 0.5, [](const ControllerReport& c) { return (c.buttons & BTN_MENU) != 0; }},
        {"up", "l stick up", 0.5, [](const ControllerReport& c) { return c.stick_y > 100; }},
        {"down", "l stick down", 0.5, [](const ControllerReport& c) { return c.stick_y < -100; }},
        {"right", "l stck right", 0.5, [](const ControllerReport& c) { return c.stick_x > 100; }},
        {"left", "l sticvk left", 0.5, [](const ControllerReport& c) { return c.stick_x < -100; }},
    };
    std::vector<std::pair<std::string, double>> first;
    for (const auto& row : table) {
        const auto it = std::find_if(left.begin(), left.end(), [&](const auto& x) { return row.pred(x.second); });
        CHECK(it != left.end());
        if (it == left.end()) {
            return;
        }
        const double t = it->first;
        const double mt = mk(row.mark);
        CHECK(mt - LEAD <= t && t <= mt + row.slack);
        first.emplace_back(row.name, t);
    }
    // activation order == marker order
    auto by_time = first;
    std::stable_sort(by_time.begin(), by_time.end(), [](const auto& a, const auto& b) { return a.second < b.second; });
    for (std::size_t i = 0; i < table.size(); i++) {
        CHECK(by_time[i].first == table[i].name);
    }
    const auto t_of = [&](const std::string& n) {
        return std::find_if(first.begin(), first.end(), [&](const auto& x) { return x.first == n; })->second;
    };
    // single-button edges are exclusive
    for (const auto& [name, bit] : std::vector<std::pair<std::string, std::uint32_t>>{{"A", BTN_A}, {"B", BTN_B}, {"menu", BTN_MENU}}) {
        const double t = t_of(name);
        std::uint32_t near = 0;
        for (const auto& [tt, c] : left) {
            if (tt >= t - 0.2 && tt <= t + 0.1) {
                near |= c.buttons;
            }
        }
        CHECK(near == bit);
    }
    const auto touch = [&](double t, double span, const std::function<bool(TouchBits)>& f) {
        return std::any_of(left.begin(), left.end(), [&](const auto& x) {
            return x.first >= t - span && x.first <= t + span && f(x.second.touch_bits());
        });
    };
    CHECK(touch(t_of("A") + 0.05, 0.05, [](TouchBits b) { return b.a; }));
    CHECK(touch(t_of("B") + 0.05, 0.05, [](TouchBits b) { return b.b; }));
    CHECK(touch(t_of("stick") + 0.5, 0.6, [](TouchBits b) { return b.stick; }));
    CHECK(touch(t_of("up") + 1.0, 1.0, [](TouchBits b) { return b.stick; }));
    CHECK(std::all_of(left.begin(), left.end(), [](const auto& x) { return x.second.touch_bits().hand_on; }));
    // analog sanity
    std::uint8_t max_trig = 0, max_grip = 0;
    std::int32_t min_x = 1000, max_y = -1000;
    for (const auto& [t, c] : left) {
        max_trig = std::max(max_trig, c.trigger);
        max_grip = std::max(max_grip, c.grip);
        min_x = std::min(min_x, c.stick_x);
        max_y = std::max(max_y, c.stick_y);
    }
    CHECK(max_trig == 255);
    CHECK(max_grip == 255);
    CHECK(min_x <= -127);
    CHECK(max_y == 127);
    CHECK(std::all_of(left.begin(), left.end(), [](const auto& x) {
        return x.first >= 20.0 || (std::abs(x.second.stick_x) < 4 && std::abs(x.second.stick_y) < 4);
    }));
    double sum = 0;
    for (std::size_t i = 0; i < 200; i++) {
        double q = 0;
        for (const auto a : left[i].second.raw_accel) {
            q += static_cast<double>(a) * static_cast<double>(a);
        }
        sum += std::sqrt(q) / 1024.0;
    }
    const double mean = sum / 200.0;
    CHECK(0.9 < mean && mean < 1.1);
    bool mono = true;
    for (std::size_t i = 1; i < left.size(); i++) {
        const std::uint32_t d = left[i].second.ts_us - left[i - 1].second.ts_us;
        mono = mono && d > 0 && d < 20'000;
    }
    CHECK(mono);
    CHECK(std::all_of(left.begin(), left.end(), [](const auto& x) { return x.second.battery == 72 && !x.second.charging; }));
}

TEST(right_controller_idle_and_round1) {
    const auto rows = testutil::load_rec("ctrl_buttons_hub2.rec.gz");
    const auto p82 = testutil::load_rec("p82.rec.gz");
    if (!rows || !p82) {
        return;
    }
    std::vector<ControllerReport> right;
    for (const auto& [t, b] : *rows) {
        if (b[3] == 3) {
            right.push_back(decode_controller(b).value());
        }
    }
    CHECK(right.size() > 40'000);
    CHECK(std::all_of(right.begin(), right.end(), [](const ControllerReport& c) {
        return c.buttons == 0 && c.touch_bits() == TouchBits{} && c.trigger == 0 && c.grip == 0 && c.stick_x == 0 &&
               c.stick_y == 0;
    }));
    CHECK(std::all_of(right.begin(), right.end(), [](const ControllerReport& c) { return c.battery == 99; }));

    TimedCtrl r1;
    for (const auto& [t, b] : *p82) {
        if (b[3] == 3) {
            r1.emplace_back(t, decode_controller(b).value());
        }
    }
    const auto win = [&](double a, double b) {
        std::vector<ControllerReport> v;
        for (const auto& [t, c] : r1) {
            if (t >= a && t <= b) {
                v.push_back(c);
            }
        }
        return v;
    };
    const auto grip = win(72.8, 73.4);
    CHECK(!grip.empty());
    if (grip.empty()) {
        return;
    }
    const auto mg = std::max_element(grip.begin(), grip.end(), [](auto& a, auto& b) { return a.grip < b.grip; })->grip;
    CHECK(mg > 150 && grip[0].grip < 20);
    const auto trig = win(75.4, 75.8);
    const auto mt = std::max_element(trig.begin(), trig.end(), [](auto& a, auto& b) { return a.trigger < b.trigger; })->trigger;
    CHECK(mt == 255);
    CHECK(trig[0].trigger < 30);
    const auto w = win(75.5, 75.7);
    const auto my = std::max_element(w.begin(), w.end(), [](auto& a, auto& b) { return a.stick_y < b.stick_y; })->stick_y;
    CHECK(my >= 10);
    CHECK(std::all_of(r1.begin(), r1.end(), [](const auto& x) { return x.second.buttons == 0; }));
    CHECK(std::any_of(r1.begin(), r1.end(), [](const auto& x) { return x.second.touch_bits().hand_on; }));
    CHECK(std::all_of(r1.begin(), r1.end(), [](const auto& x) { return x.first >= 70.0 || !x.second.touch_bits().hand_on; }));
}

PIBRIDGE_TEST_MAIN()
