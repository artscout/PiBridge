// SPDX-License-Identifier: MIT
// Twins of omarchyvr pimax-hands/tests/inhand_fixtures.rs (MIT OR Apache-2.0): the in-hand detector on the REAL
// left-controller IMU log of the session 2026-10-16-1818 (rust/fixtures/inhand_*.csv.gz in the omarchyvr checkout,
// read only; skipped without it).
#include "pibridge/ctrl/inhand.h"
#include "pibridge/ctrl/report.h"
#include "pibridge_test.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <tuple>
#include <unistd.h>
#include <vector>
#include <zlib.h>

using namespace pibridge::ctrl;
using namespace pibridge::ctrl::inhand;

namespace {

struct Sample {
    std::uint64_t t;
    std::array<std::int16_t, 3> g, a;
};

std::string fixture(const std::string& name) {
    return std::string(PIBRIDGE_CTRL_FIXTURES) + "/inhand_" + name + ".csv.gz";
}

std::vector<Sample> load(const std::string& name) {
    std::vector<Sample> out;
    gzFile f = gzopen(fixture(name).c_str(), "rb");
    if (f == nullptr) {
        return out;
    }
    char line[256];
    std::uint64_t t = 1000000000ull;
    while (gzgets(f, line, sizeof line) != nullptr) {
        long long v[7] = {};
        char* p = line;
        for (long long& x : v) {
            x = std::strtoll(p, &p, 10);
            if (*p == ',') {
                p++;
            }
        }
        t += static_cast<std::uint64_t>(v[0]) * 1000;
        const auto i = [&](int k) { return static_cast<std::int16_t>(v[k]); };
        out.push_back({t, {i(1), i(2), i(3)}, {i(4), i(5), i(6)}});
    }
    gzclose(f);
    return out;
}

using Tr = std::tuple<double, bool, Why>;

std::vector<Tr> run(InHand& d, const std::vector<Sample>& s) {
    const std::uint64_t t0 = s[0].t;
    std::vector<Tr> out;
    bool held = d.is_held();
    for (const Sample& x : s) {
        d.sample(x.t, x.g, x.a);
        if (d.held(x.t) != held) {
            held = !held;
            out.emplace_back(static_cast<double>(x.t - t0) * 1e-9, held, d.last_change().first);
        }
    }
    return out;
}

InHand picked_up(std::uint64_t t) {
    InHand d{Config{}};
    d.input(t, 0, 0, 0, 0, 0, 0);
    d.input(t + 1, 0, 0, 0, 0, report::BTN_A, 0);
    CHECK(d.is_held());
    return d;
}

bool have() {
    return ::access(fixture("desk").c_str(), R_OK) == 0;
}

} // namespace

TEST(lying_on_the_desk_never_shows) {
    for (const char* name : {"desk", "desk_bump"}) {
        InHand d;
        CHECK(run(d, load(name)).empty());
    }
}

TEST(held_still_on_the_desk_is_put_down_within_6_s) {
    const auto s = load("desk");
    InHand d = picked_up(s[0].t);
    const auto tr = run(d, s);
    CHECK(tr.size() == 1);
    if (!tr.empty()) {
        const auto [at, held, why] = tr[0];
        CHECK(!held && why == Why::Still && at >= 5.0 && at < 6.5);
    }
}

TEST(picking_up_shows_within_250_ms) {
    InHand d;
    const auto tr = run(d, load("pickup"));
    CHECK(!tr.empty());
    if (!tr.empty()) {
        const auto [at, held, why] = tr[0];
        CHECK(held && why == Why::Motion);
        CHECK(at >= 6.45 && at < 6.75);
    }
    CHECK(tr.size() == 1);
}

TEST(quiet_in_the_hand_is_never_put_down) {
    const auto s = load("held");
    InHand d = picked_up(s[0].t);
    CHECK(run(d, s).empty());
}

TEST(put_down_and_picked_up_again) {
    const auto s = load("putdown");
    InHand d = picked_up(s[0].t);
    const auto tr = run(d, s);
    CHECK(tr.size() == 2);
    if (tr.size() == 2) {
        CHECK(!std::get<1>(tr[0]) && std::get<2>(tr[0]) == Why::Still && std::get<0>(tr[0]) >= 12.5 &&
              std::get<0>(tr[0]) < 14.5);
        CHECK(std::get<1>(tr[1]) && std::get<2>(tr[1]) == Why::Motion && std::get<0>(tr[1]) >= 21.9 &&
              std::get<0>(tr[1]) < 22.6);
    }
}

TEST(typing_on_the_desk_stays_down) {
    auto s = load("desk");
    const std::uint64_t t0 = s[0].t;
    std::uint64_t seed = 12345;
    const auto rnd = [&](std::int32_t m) {
        seed = seed * 6364136223846793005ull + 1442695040888963407ull;
        return (static_cast<std::int32_t>(seed >> 33) % (2 * m + 1)) - m;
    };
    std::uint64_t next_key = t0 + 100000000;
    std::uint64_t key_end = 0;
    for (Sample& x : s) {
        if (x.t >= next_key) {
            key_end = x.t + 10000000;
            next_key = x.t + 120000000 + static_cast<std::uint64_t>(rnd(40) + 40) * 1000000;
        }
        if (x.t < key_end) {
            for (int k = 0; k < 3; k++) {
                x.g[k] = static_cast<std::int16_t>(x.g[k] + rnd(15));
                x.a[k] = static_cast<std::int16_t>(x.a[k] + rnd(40));
            }
        }
    }
    InHand d;
    CHECK(run(d, s).empty());
}

TEST(no_reports_puts_it_down) {
    const auto s = load("held");
    InHand d = picked_up(s[0].t);
    const std::vector<Sample> half(s.begin(), s.begin() + static_cast<std::ptrdiff_t>(s.size() / 2));
    run(d, half);
    const std::uint64_t last = half.back().t;
    CHECK(d.held(last + 1000000000));
    CHECK(!d.held(last + 2000000000));
    CHECK(d.last_change().first == Why::NoReports);
}

int main(int argc, char** argv) {
    if (!have()) {
        std::printf("skipped: no pimax-hands fixtures at %s\n", PIBRIDGE_CTRL_FIXTURES);
        return 77;
    }
    return pibridge::test::run_all(argc, argv);
}
