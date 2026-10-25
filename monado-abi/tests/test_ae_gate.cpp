// SPDX-License-Identifier: MIT
// The auto-exposure payloads against PiBridge's HID allowlist gate (devices/*.toml `camera_ae`), M3:
// * the twin of omarchyvr pimax-device/tests/allowlist.rs crystal_super_camera_ae_uses_its_own_windows_ranges
//   (MIT OR Apache-2.0): Windows' per-pair ranges of the Crystal capture pass, anything outside them does not, the
//   descriptor and the AE tables agree, the controller's payloads from dark to bright pass;
// * a property test: for random rooms (dark to sunlit, windows, blobs, an upper pair of its own, random start
//   settings) and every PIMAX_CAM_AE_* override (alone, extreme, NaN/inf, garbage, and random combinations), no target
//   the Crystal Super controller computes leaves Windows' pair ranges, and every payload it hands out for sending
//   passes the gate as `camera_ae` (one 64-byte report, nothing else on the wire).
// Uses libpibridge-cam's ae (the code inside libpibridge_pimax.a) and libpibridge-hmd's gate directly.
#include "pibridge/cam/ae.h"
#include "pibridge/hmd/descriptor.h"
#include "pibridge_test.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

using namespace pibridge::cam::ae;
using namespace pibridge::hmd;

using LEV = std::vector<pibridge::cam::ae::LightEvent>;
using P16 = std::pair<std::uint16_t, std::uint16_t>;
using P32 = std::pair<std::uint32_t, std::uint32_t>;
using A4 = std::array<std::uint16_t, 4>;
using V16 = std::vector<std::uint16_t>;

namespace {

const Descriptor& load(const char* id) {
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

std::vector<std::uint8_t> report(const AeSetting& s) {
    std::vector<std::uint8_t> b{0x58, 0x19, 0x51, 0x00};
    const auto p = payload(s);
    b.insert(b.end(), p.begin(), p.end());
    b.resize(64, 0);
    return b;
}

ParamValues vals(const AeSetting& s) {
    return {{"e0", s.exposure[0]}, {"e1", s.exposure[1]}, {"e2", s.exposure[2]}, {"e3", s.exposure[3]},
            {"g0", s.gain[0]},     {"g1", s.gain[1]},     {"g2", s.gain[2]},     {"g3", s.gain[3]},
            {"led_e", s.led_exposure}, {"led_g", s.led_gain}};
}

std::string checked(const Allowlist& al, const std::vector<std::uint8_t>& r) {
    const auto c = al.check(0, r);
    return c ? (*c)->id : "<" + c.error().message() + ">";
}

bool out_of_range(const Allowlist& al, const std::vector<std::uint8_t>& r) {
    const auto c = al.check(0, r);
    return !c && c.error().kind == Refused::Kind::OutOfRange;
}

std::pair<std::uint32_t, std::uint32_t> range(const Allowlist& al, const char* id, const char* name) {
    const CommandSpec* c = al.get(id);
    for (const auto& [n, con] : c->params) {
        if (n == name) {
            return con.is_range ? std::pair{con.lo, con.hi} : std::pair{con.set.front(), con.set.back()};
        }
    }
    return {0, 0};
}

} // namespace

TEST(crystal_super_camera_ae_uses_its_own_windows_ranges) {
    const Allowlist& al = load("crystal-super").allowlist;
    // The first command of take cold2 (2200/13) and the brightest Windows sent (lower 10410/449, upper 8941/439).
    std::vector<std::uint8_t> first{0x58, 0x19, 0x51, 0x00, 0x01, 0x32, 0x98, 0x08, 0x98, 0x08, 0x98, 0x08, 0x98,
                                    0x08, 0x0d, 0x00, 0x0d, 0x00, 0x0d, 0x00, 0x0d, 0x00, 0x28, 0x00, 0x3e, 0x00};
    first.resize(64, 0);
    CHECK(checked(al, first) == "camera_ae");
    const AeSetting top{{10410, 10410, 8941, 8941}, {449, 449, 439, 439}, 40, 62};
    const auto r = al.build("camera_ae", vals(top));
    CHECK(r.has_value());
    if (r) {
        const auto p = payload(top);
        CHECK(std::equal(p.begin(), p.end(), r->begin() + 4));
        std::vector<std::uint8_t> out;
        Gate g(al, 0, vector_writer(out));
        CHECK(g.send(*r).value_or("") == "camera_ae");
        CHECK(out.size() == 65);
    }
    // Outside: the upper pair at the lower pair's top, below 2200, gain 450, another LED pair.
    std::vector<AeSetting> bad(6, top);
    bad[0].exposure = {10410, 10410, 10410, 10410};
    bad[1].gain = {449, 449, 440, 440};
    bad[2].exposure = {2199, 2199, 3000, 3000};
    bad[3].gain = {450, 450, 13, 13};
    bad[4].led_exposure = 25;
    bad[5].led_gain = 35;
    for (const AeSetting& b : bad) {
        CHECK(out_of_range(al, report(b)));
        CHECK(!model_allows(AeModel::CrystalSuper, b));
    }
    auto b = first;
    b[5] = 0x3c; // 01 3c instead of 01 32
    const auto c = al.check(0, b);
    CHECK(!c && c.error().kind == Refused::Kind::NotListed);
    // The descriptor and the AE tables agree on the ranges.
    const char* names[2][2] = {{"e0", "g0"}, {"e2", "g2"}};
    for (std::size_t k = 0; k < 2; k++) {
        const PairRange pr = CRYSTAL_PAIR_RANGES[k];
        CHECK(range(al, "camera_ae", names[k][0]) == P32(pr.e_min, pr.e_max));
        CHECK(range(al, "camera_ae", names[k][1]) == P32(pr.g_min, pr.g_max));
    }
    // Every setting of the Crystal's high table, in both pairs where the range allows it, builds and passes.
    for (const auto& [e, gn] : CRYSTAL_HIGH) {
        const bool fits = e <= 8941 && gn <= 439;
        const AeSetting s{{e, e, fits ? e : std::uint16_t(8941), fits ? e : std::uint16_t(8941)},
                          {gn, gn, fits ? gn : std::uint16_t(439), fits ? gn : std::uint16_t(439)}, 40, 62};
        CHECK(al.build("camera_ae", vals(s)).has_value());
    }
    // The controller itself: from the firmware default through dark and bright frames.
    AutoExposure ae(AeParams::for_model(AeModel::CrystalSuper));
    AeSetting s{{3500, 3500, 3500, 3500}, {60, 60, 60, 60}, 40, 62};
    std::uint64_t t = 0;
    for (const auto& [mean, frames] : std::vector<std::pair<double, int>>{
             {2.0, 200}, {250.0, 200}, {64.0, 50}, {10.0, 300}, {40.0, 100}, {200.0, 1000}}) {
        for (int i = 0; i < frames; i++) {
            TileStats st;
            st.mean = st.trimmed_mean = st.p50 = st.p95 = mean;
            st.sat_frac = mean > 240.0 ? 0.6 : 0.0;
            st.dark_frac = mean < 8.0 ? 0.9 : 0.0;
            st.n = 1000;
            const auto d = ae.update(s, {st, st}, t);
            const AeSetting tg = *ae.target();
            CHECK(checked(al, report(tg)) == "camera_ae");
            if (d.send) {
                s = tg;
            }
            t += 40'000'000;
        }
    }
    CHECK(s.exposure[0] == 2200 && s.gain[0] == 13);
    // The Dream Air keeps its own (unchanged) ranges: the Crystal's high settings are refused there.
    const Allowlist& da = load("dream-air").allowlist;
    CHECK(!da.build("camera_ae", vals(AeSetting{{5641, 5641, 5641, 5641}, {215, 215, 215, 215}, 40, 62})));
    CHECK(da.build("camera_ae", vals(AeSetting{{2200, 2200, 2200, 2200}, {13, 13, 13, 13}, 40, 62})).has_value());
}

namespace {

std::uint64_t g_lcg = 1;
double rnd() {
    g_lcg = g_lcg * 6364136223846793005ull + 1442695040888963407ull;
    return static_cast<double>(g_lcg >> 11) / 9007199254740992.0;
}

/// One tile of a simulated sensor (texture, black lens corners, a window, a blob). Only the pixels the controller
/// samples (its 4-pixel grid from (2, 2); the step is not an override) are rendered.
std::vector<std::uint8_t> render(double rad, double window, bool blob, std::uint16_t e, std::uint16_t g) {
    std::vector<std::uint8_t> img(640 * 480);
    const double c = static_cast<double>(e) * static_cast<double>(g) / (2000.0 * 13.0);
    const int x0 = static_cast<int>(640.0 * (1.0 - window));
    for (int y = 2; y < 480; y += 4) {
        for (int x = 2; x < 640; x += 4) {
            const double dx = (x - 319.5) / 320.0, dy = (y - 239.5) / 240.0;
            double v = rad * (0.6 + 0.4 * ((x / 16 + y / 16) % 2)) * c;
            if (dx * dx + dy * dy > 1.0) {
                v = 0.0;
            } else if (x >= x0) {
                v = 16.0 * rad * c;
            }
            if (blob && (x - 500) * (x - 500) + (y - 100) * (y - 100) < 900) {
                v = 1e4;
            }
            v = std::round(v);
            img[static_cast<std::size_t>(y * 640 + x)] = static_cast<std::uint8_t>(!(v > 0.0) ? 0.0 : v > 255.0 ? 255.0 : v);
        }
    }
    return img;
}

struct Totals {
    long runs = 0, panics = 0, sends = 0, targets = 0;
};

/// A closed loop of `frames` tracking frames in changing random rooms; every target and every sent payload checked.
void closed_loop(const Allowlist& al, int frames, Totals& tot) {
    AeParams p;
    try {
        p = AeParams::from_env_model(AeModel::CrystalSuper);
    } catch (const std::logic_error&) {
        // The Rust code panics here (f64::clamp with crossed bounds): pimax_cam_ae_create returns NULL, nothing is
        // ever sent.
        tot.panics++;
        return;
    }
    tot.runs++;
    AutoExposure ae(p);
    const double r0 = rnd();
    const std::uint16_t e0 = r0 < 0.5 ? 3500 : static_cast<std::uint16_t>(1 + rnd() * 20000);
    const std::uint16_t g0 = r0 < 0.5 ? 60 : static_cast<std::uint16_t>(rnd() * 1000);
    AeSetting applied{{e0, e0, e0, e0}, {g0, g0, g0, g0}, 40, 62};
    std::vector<std::pair<int, AeSetting>> pipe;
    double rad = 30.0, upper = 1.0, window = 0.0;
    bool blob = false;
    int seg_end = 0;
    std::uint64_t t = static_cast<std::uint64_t>(rnd() * 1e12);
    for (int i = 0; i < frames; i++) {
        if (i >= seg_end) {
            rad = std::exp(std::log(0.03) + rnd() * (std::log(3000.0) - std::log(0.03)));
            upper = 0.3 + rnd() * 1.2;
            window = rnd() < 0.3 ? rnd() * 0.6 : 0.0;
            blob = rnd() < 0.3;
            seg_end = i + 25 * (1 + static_cast<int>(rnd() * 8.0));
        }
        while (!pipe.empty() && pipe.front().first <= i) {
            applied = pipe.front().second;
            pipe.erase(pipe.begin());
        }
        const auto lo = render(rad, window, blob, applied.exposure[0], applied.gain[0]);
        const auto up = render(rad * upper, window * 0.5, false, applied.exposure[2], applied.gain[2]);
        const pibridge::cam::TileView vl{lo, 640, 0, 0, 640, 480}, vu{up, 640, 0, 0, 640, 480};
        const auto st = ae.frame_stats({vl, vl, vu, vu}, Mask{});
        const auto d = ae.update(applied, st, t);
        const AeSetting tg = *ae.target();
        if (!model_allows(AeModel::CrystalSuper, tg)) {
            tot.targets++;
            std::fprintf(stderr, "target outside the ranges: %u/%u %u/%u\n", tg.exposure[0], tg.gain[0], tg.exposure[2],
                         tg.gain[2]);
        }
        // What pimax_cam_ae_update hands out: send only when allowed; the driver's report goes through the gate.
        if (d.send && model_allows(AeModel::CrystalSuper, tg)) {
            tot.sends++;
            std::vector<std::uint8_t> out;
            Gate g(al, 0, vector_writer(out));
            const auto sent = g.send(report(tg));
            CHECK(sent.value_or("") == "camera_ae");
            CHECK(out.size() == 65);
            pipe.emplace_back(i + 2, tg);
        }
        t += 40'000'000ULL + static_cast<std::uint64_t>(rnd() * 4e6);
        if (rnd() < 0.002) {
            t += 3'000'000'000ULL; // a camera restart
        }
    }
}

const char* const ENV_NAMES[] = {"PIMAX_CAM_AE_TARGET", "PIMAX_CAM_AE_EXP_MIN",   "PIMAX_CAM_AE_EXP_MAX",
                                 "PIMAX_CAM_AE_GAIN_MIN", "PIMAX_CAM_AE_GAIN_MAX", "PIMAX_CAM_AE_PERIOD_MS",
                                 "PIMAX_CAM_AE_K",       "PIMAX_CAM_AE_DEADBAND", "PIMAX_CAM_AE_UPPER",
                                 "PIMAX_CAM_AE_QUANTIZE", "PIMAX_CAM_AE_EXTEND",  "PIMAX_CAM_AE_LAW"};
const char* const ENV_VALUES[] = {"0",    "1",    "-5",    "13",  "100",   "449",  "450",  "900",    "2199",
                                  "2200", "4108", "8941",  "8942", "10410", "10411", "65535", "1e9",  "-1e9",
                                  "nan",  "inf",  "-inf",  "0.5", " 64 ",  "x",     "",     "old",  "v1"};

void clear_env() {
    for (const char* n : ENV_NAMES) {
        ::unsetenv(n);
    }
}

} // namespace

TEST(crystal_payloads_never_leave_the_windows_ranges_for_any_override) {
    const Allowlist& al = load("crystal-super").allowlist;
    Totals tot;
    clear_env();
    g_lcg = 20261019;
    // the defaults, many rooms
    for (int k = 0; k < 6; k++) {
        closed_loop(al, 600, tot);
    }
    // every override alone, every value
    for (const char* n : ENV_NAMES) {
        for (const char* v : ENV_VALUES) {
            clear_env();
            ::setenv(n, v, 1);
            closed_loop(al, 120, tot);
        }
    }
    // random combinations of three to twelve overrides
    for (int k = 0; k < 150; k++) {
        clear_env();
        const int m = 3 + static_cast<int>(rnd() * 10.0);
        for (int j = 0; j < m; j++) {
            ::setenv(ENV_NAMES[static_cast<std::size_t>(rnd() * 12.0)],
                     ENV_VALUES[static_cast<std::size_t>(rnd() * (sizeof ENV_VALUES / sizeof ENV_VALUES[0]))], 1);
        }
        closed_loop(al, 150, tot);
    }
    clear_env();
    std::printf("%ld runs (%ld overrides that panic in Rust too: no controller), %ld payloads sent through the gate, "
                "%ld targets outside the ranges\n",
                tot.runs, tot.panics, tot.sends, tot.targets);
    CHECK(tot.targets == 0);
    CHECK(tot.sends > 1000);
}

PIBRIDGE_TEST_MAIN()
