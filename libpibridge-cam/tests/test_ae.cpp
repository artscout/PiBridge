// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-cam/src/ae.rs and the text tests of pimax-cam/src/capi_ae.rs (MIT OR
// Apache-2.0): the same cases and expected values.
#include "pibridge/cam/ae.h"
#include "pibridge_test.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <string>
#include <unistd.h>
#include <vector>

using namespace pibridge::cam;
using namespace pibridge::cam::ae;

using LEV = std::vector<pibridge::cam::ae::LightEvent>;
using P16 = std::pair<std::uint16_t, std::uint16_t>;
using P32 = std::pair<std::uint32_t, std::uint32_t>;
using A4 = std::array<std::uint16_t, 4>;
using V16 = std::vector<std::uint16_t>;

namespace {

AeSetting setting(std::uint16_t e, std::uint16_t g) {
    return AeSetting{{e, e, e, e}, {g, g, g, g}, 40, 62};
}

TileStats ts(double trimmed_mean, double sat_frac = 0.0) {
    TileStats s;
    s.trimmed_mean = trimmed_mean;
    s.sat_frac = sat_frac;
    s.n = 1000;
    return s;
}

/// A simulated sensor: pixel = clamp(radiance * e * g * c), with a textured scene and a bright light blob.
std::vector<std::uint8_t> render(double radiance, std::uint16_t e, std::uint16_t g, bool blob) {
    std::vector<std::uint8_t> img(640 * 480);
    const double c = 1.0 / (2000.0 * 13.0);
    for (int y = 0; y < 480; y++) {
        for (int x = 0; x < 640; x++) {
            const double tex = 0.6 + 0.4 * static_cast<double>((x / 16 + y / 16) % 2) + 0.1 * std::sin(x * 0.1);
            double v = radiance * tex * static_cast<double>(e) * static_cast<double>(g) * c;
            if (blob && (x - 500) * (x - 500) + (y - 100) * (y - 100) < 900) {
                v = 1e4;
            }
            v = std::round(v);
            img[static_cast<std::size_t>(y) * 640 + static_cast<std::size_t>(x)] =
                static_cast<std::uint8_t>(v < 0.0 ? 0.0 : (v > 255.0 ? 255.0 : v));
        }
    }
    return img;
}

TileView view(const std::vector<std::uint8_t>& d) {
    return TileView{d, 640, 0, 0, 640, 480};
}

std::array<TileStats, 2> stats_of(const AutoExposure& ae, const std::vector<std::uint8_t>& a,
                                  const std::vector<std::uint8_t>& b) {
    return ae.frame_stats({view(a), view(b), view(a), view(b)}, Mask{});
}

AeSetting power_on() {
    return setting(POWER_ON_EXPOSURE, POWER_ON_GAIN);
}

AeSetting start_bright() {
    return setting(2000, 13);
}

using Pipe = std::vector<std::pair<std::size_t, AeSetting>>;

void land(Pipe& pipe, std::size_t i, AeSetting& applied) {
    while (!pipe.empty() && pipe.front().first <= i) {
        applied = pipe.front().second;
        pipe.erase(pipe.begin());
    }
}

constexpr std::uint64_t S = 1'000'000'000;

/// Closed-loop run in a scene of constant radiance (2-frame latency, 40 ms tracking period).
void run_scene(AutoExposure& ae, AeSetting& applied, Pipe& pipeline, double radiance, std::size_t frames,
               std::uint64_t& t, std::vector<std::pair<std::uint64_t, LightEvent>>& events) {
    // Commands still in flight from the previous call land at once.
    if (!pipeline.empty()) {
        applied = pipeline.back().second;
        pipeline.clear();
    }
    for (std::size_t i = 0; i < frames; i++) {
        land(pipeline, i, applied);
        const auto img = render(radiance, applied.exposure[0], applied.gain[0], false);
        const auto st = stats_of(ae, img, img);
        const auto d = ae.update(applied, st, t);
        for (const auto& ev : ae.take_events()) {
            events.emplace_back(t, ev);
        }
        if (d.send) {
            pipeline.emplace_back(i + 2, *ae.target());
        }
        const AeSetting s = *ae.target();
        CHECK(s.led_exposure == 40 && s.led_gain == 62);
        if (s.exposure[0] > 4108) {
            CHECK(s.gain[0] == POWER_ON_GAIN);
        }
        CHECK(static_cast<std::uint64_t>(s.exposure[0]) * 1000 < 40'000'000);
        CHECK(s.exposure[0] <= 10410 && s.gain[0] <= 449);
        t += 40'000'000;
    }
}

/// The room with a window (20x brighter) over the right frac of the width.
std::vector<std::uint8_t> render_window(double radiance, std::uint16_t e, std::uint16_t g, double frac) {
    auto img = render(radiance, e, g, false);
    const std::size_t x0 = static_cast<std::size_t>(640.0 * (1.0 - frac));
    const double c = 1.0 / (2000.0 * 13.0);
    for (std::size_t y = 0; y < 480; y++) {
        for (std::size_t x = x0; x < 640; x++) {
            const double v = std::round(20.0 * radiance * 0.8 * static_cast<double>(e) * static_cast<double>(g) * c);
            img[y * 640 + x] = static_cast<std::uint8_t>(v < 0.0 ? 0.0 : (v > 255.0 ? 255.0 : v));
        }
    }
    return img;
}

struct LawRun {
    std::vector<std::uint32_t> per_s;
    std::vector<std::tuple<std::uint16_t, std::uint16_t, double, double>> log;
    AutoExposure ae;
};

LawRun run_law(const AeParams& p, std::size_t secs,
               const std::function<std::vector<std::uint8_t>(std::uint16_t, std::uint16_t)>& scene, AeSetting start) {
    LawRun r{std::vector<std::uint32_t>(secs, 0), {}, AutoExposure(p)};
    AeSetting applied = start;
    Pipe pipe;
    std::uint64_t t = 0;
    std::optional<AeSetting> prev;
    for (std::size_t i = 0; i < secs * 25; i++) {
        land(pipe, i, applied);
        const auto img = scene(applied.exposure[0], applied.gain[0]);
        const auto st = stats_of(r.ae, img, img);
        const auto d = r.ae.update(applied, st, t);
        if (d.send) {
            pipe.emplace_back(i + 2, *r.ae.target());
        }
        const AeSetting tg = *r.ae.target();
        if (prev && *prev != tg) {
            r.per_s[i / 25] += 1;
        }
        prev = tg;
        r.log.emplace_back(applied.exposure[0], applied.gain[0], st[0].trimmed_mean, st[0].sat_frac);
        t += 40'000'000;
    }
    return r;
}

bool all_le(const std::vector<std::uint32_t>& v, std::size_t from, std::uint32_t lim) {
    return std::all_of(v.begin() + static_cast<std::ptrdiff_t>(from), v.end(), [&](std::uint32_t c) { return c <= lim; });
}

void tick(const LightParams& lp, LightGov& g, std::uint64_t& t, std::vector<LightEvent>& ev, double secs, double mean,
          bool at, bool settled) {
    const auto n = static_cast<std::uint64_t>(std::round(secs * 25.0));
    for (std::uint64_t i = 0; i < n; i++) {
        const auto e = g.step(lp, t, mean, at, settled, 2);
        ev.insert(ev.end(), e.begin(), e.end());
        t += 40'000'000;
    }
}

AeParams crystal() {
    return AeParams::for_model(AeModel::CrystalSuper);
}

bool contains(std::span<const std::uint16_t> s, std::uint16_t v) {
    return std::find(s.begin(), s.end(), v) != s.end();
}

bool in_high(std::uint16_t e, std::uint16_t g) {
    return std::find(CRYSTAL_HIGH.begin(), CRYSTAL_HIGH.end(), std::pair{e, g}) != CRYSTAL_HIGH.end();
}

} // namespace

TEST(payload_matches_the_windows_frame) {
    const AeSetting s{{0x0c47, 0x0c47, 0x0857, 0x0857}, {13, 13, 13, 13}, 40, 62};
    const auto p = payload(s);
    const std::array<std::uint8_t, 22> want{0x01, 0x32, 0x47, 0x0c, 0x47, 0x0c, 0x57, 0x08, 0x57, 0x08, 0x0d,
                                            0x00, 0x0d, 0x00, 0x0d, 0x00, 0x0d, 0x00, 0x28, 0x00, 0x3e, 0x00};
    CHECK(p == want);
    std::vector<std::uint8_t> report{0x58, 0x19, 0x51, 0x00};
    report.insert(report.end(), p.begin(), p.end());
    CHECK(parse_payload(report) == s);
    CHECK(parse_payload(p) == s);
    const std::uint8_t other[] = {0x58, 0x04, 0x39, 0x00, 0x48};
    CHECK(!parse_payload(other));
}

TEST(histogram_stats) {
    Histogram h;
    h.bins[10] = 50;
    h.bins[100] = 48;
    h.bins[255] = 2;
    const auto s = h.stats(0.98, 250);
    CHECK(s.n == 100);
    CHECK(std::fabs(s.mean - (500.0 + 4800.0 + 510.0) / 100.0) < 1e-9);
    CHECK(std::fabs(s.trimmed_mean - (500.0 + 4800.0) / 98.0) < 1e-9);
    CHECK(s.p50 == 10.0);
    CHECK(std::fabs(s.sat_frac - 0.02) < 1e-12);
    CHECK(h.quantile(0.51) == 100.0);
}

TEST(split_prefers_exposure) {
    const AeParams p;
    CHECK(p.split(2200.0 * 13.0) == P16(2200, 13));
    CHECK(p.split(3150.0 * 13.0) == P16(3143, 13));
    const auto [e, g] = p.split(4500.0 * 13.0);
    CHECK(g == 15);
    CHECK(e <= p.exposure_max && std::fabs(e * 15.0 - 4500.0 * 13.0) / (4500.0 * 13.0) < 0.035);
    CHECK(p.split(1.0) == std::pair(p.exposure_min, p.gain_min));
    CHECK(p.split(1e12) == std::pair(p.exposure_max, p.gain_max));
    AeParams q;
    q.quantize = false;
    CHECK(q.split(2000.0 * 13.0) == P16(2000, 13));
    for (const double w : {1682.0, 1733.0, 1785.0, 2011.0, 2478.0, 3051.0, 3988.0, 4108.0}) {
        CHECK(ladder(w) == w);
    }
}

TEST(converges_from_power_on_with_latency) {
    for (const auto& [radiance, blob] : std::vector<std::pair<double, bool>>{{90.0, true}, {30.0, false}, {300.0, false}, {6.0, false}}) {
        AutoExposure ae{AeParams{}};
        AeSetting applied = power_on();
        Pipe pipeline;
        std::vector<double> means;
        std::uint64_t t = 0;
        for (std::size_t i = 0; i < 150; i++) {
            land(pipeline, i, applied);
            const auto img = render(radiance, applied.exposure[0], applied.gain[0], blob);
            const auto st = stats_of(ae, img, img);
            means.push_back(st[0].trimmed_mean);
            const auto d = ae.update(applied, st, t);
            if (d.send) {
                pipeline.emplace_back(i + 2, *ae.target());
            }
            const AeSetting s = *ae.target();
            CHECK(s.exposure[0] == s.exposure[1]);
            CHECK(s.gain[2] == s.gain[3]);
            CHECK(s.exposure[0] <= ae.p.exposure_max && s.gain[0] <= ae.p.gain_max);
            t += 40'000'000;
        }
        const bool at_caps = (applied.exposure[0] == ae.p.exposure_max && applied.gain[0] == ae.p.gain_max) ||
                             (applied.exposure[0] == ae.p.exposure_min && applied.gain[0] == ae.p.gain_min) ||
                             applied.exposure[0] == POWER_ON_EXPOSURE;
        auto in_band = [&](double m) { return std::fabs(std::log(m / ae.p.target)) < 0.25; };
        const auto it = std::find_if(means.begin(), means.end(), in_band);
        CHECK(at_caps || (it != means.end() && it - means.begin() < 80));
        const auto [lo, hi] = std::minmax_element(means.end() - 20, means.end());
        CHECK(*hi - *lo < 1.0);
        CHECK(at_caps || in_band(means.back()));
    }
}

TEST(deadband_and_period) {
    AutoExposure ae{AeParams{}};
    const AeSetting s = setting(2000, 13);
    const TileStats st = ts(66.0);
    auto d = ae.update(s, {st, st}, 0);
    CHECK(d.send);
    CHECK(ae.target()->exposure == (A4{2000, 2000, 2000, 2000}));
    d = ae.update(s, {st, st}, 40'000'000);
    CHECK(!d.send && !d.changed);
    d = ae.update(s, {st, st}, 80'000'000);
    CHECK(d.send && !d.changed);
    const TileStats dark = ts(35.0);
    d = ae.update(s, {dark, st}, 120'000'000);
    CHECK(d.changed && d.err[0] > 0.0 && d.err[1] == 0.0);
    const AeSetting t = *ae.target();
    CHECK(t.exposure[0] > 2000 && t.exposure[0] == t.exposure[1]);
    CHECK(t.exposure[2] == 2000);
    const TileStats sat = ts(95.0, 0.2);
    {
        const auto [e, g, err] = ae.step_pair(2000, 13, sat);
        (void)g;
        CHECK(err < 0.0 && e < 2000);
    }
    const TileStats black = ts(7.0);
    CHECK(std::get<0>(ae.step_pair(POWER_ON_EXPOSURE, POWER_ON_GAIN, black)) == POWER_ON_EXPOSURE);
    const TileStats bright = ts(240.0, 0.6);
    const auto [e, g, err] = ae.step_pair(POWER_ON_EXPOSURE, POWER_ON_GAIN, bright);
    (void)err;
    CHECK(e <= ae.p.exposure_max && static_cast<double>(e) * g < static_cast<double>(POWER_ON_EXPOSURE) * POWER_ON_GAIN);
}

TEST(extends_gain_first_then_exposure_and_retracts) {
    AutoExposure ae{AeParams{}};
    AeSetting applied = setting(4108, 215);
    Pipe pipe;
    std::vector<std::pair<std::uint64_t, LightEvent>> ev;
    std::uint64_t t = 0;
    run_scene(ae, applied, pipe, 0.3, 900, t, ev);
    std::vector<std::pair<std::uint64_t, LightEvent>> ext;
    for (const auto& e : ev) {
        if (e.second.kind == LightEvent::Extended) {
            ext.push_back(e);
        }
    }
    CHECK(ext.size() == 2);
    if (ext.size() == 2) {
        CHECK(ext[0].second == (LightEvent{LightEvent::Extended, 1}));
        CHECK(ext[1].second == (LightEvent{LightEvent::Extended, 2}));
        CHECK(ext[0].first >= 5 * S && ext[0].first < 8 * S);
        CHECK(ext[1].first - ext[0].first >= 5 * S);
    }
    CHECK(ae.light.stage == 2);
    CHECK(ae.p.exposure_max == 10410 && ae.p.gain_max == 449);
    CHECK(applied.exposure[0] > 4108 || applied.gain[0] == 449);
    CHECK(!ae.light.low);
    CHECK(std::none_of(ev.begin(), ev.end(), [](const auto& e) { return e.second.kind == LightEvent::LowRaised; }));
    run_scene(ae, applied, pipe, 30.0, 600, t, ev);
    CHECK(ae.light.stage == 0);
    CHECK(std::any_of(ev.begin(), ev.end(), [](const auto& e) { return e.second.kind == LightEvent::Retracted; }));
    CHECK(ae.p.exposure_max == 4108 && ae.p.gain_max == 215);
    CHECK(applied.exposure[0] <= 4108 && applied.gain[0] <= 215);
}

TEST(gain_goes_to_the_extended_cap_before_exposure_passes_the_normal_one) {
    AeParams p;
    p.exposure_max = 10410;
    p.gain_max = 449;
    auto [e, g] = p.split(4108.0 * 300.0);
    CHECK(g == 449);
    CHECK(e <= 4108 && std::fabs(e * 449.0 - 4108.0 * 300.0) / (4108.0 * 300.0) < 0.035);
    std::tie(e, g) = p.split(3000.0 * 13.0);
    CHECK(e == static_cast<std::uint16_t>(ladder(3000.0)) && g == 13);
    std::tie(e, g) = p.split(4108.0 * 215.0);
    CHECK(e <= 4108 && g <= 449);
    std::tie(e, g) = p.split(6000.0 * 449.0);
    CHECK(g == 449);
    CHECK(e > 4108 && e <= 10410);
    CHECK(p.split(1e12) == P16(10410, 449));
}

TEST(governor_timers_and_hysteresis) {
    const LightParams lp;
    LightGov g;
    std::uint64_t t = 0;
    std::vector<LightEvent> all;
    tick(lp, g, t, all, 4.0, 10.0, true, false);
    CHECK(g.stage == 0);
    tick(lp, g, t, all, 10.0, 10.0, false, false);
    CHECK(g.stage == 0);
    tick(lp, g, t, all, 6.0, 10.0, true, false);
    CHECK(g.stage == 1);
    tick(lp, g, t, all, 6.0, 10.0, true, false);
    CHECK(g.stage == 2);
    CHECK(all == (LEV{{LightEvent::Extended, 1}, {LightEvent::Extended, 2}}));
    tick(lp, g, t, all, 5.0, 90.0, false, false);
    tick(lp, g, t, all, 1.0, 30.0, false, false);
    CHECK(g.stage == 2);
    tick(lp, g, t, all, 9.0, 90.0, false, false);
    CHECK(g.stage == 0);
    CHECK(!all.empty() && all.back() == (LightEvent{LightEvent::Retracted, 0}));
    LightGov g2;
    t = 0;
    tick(lp, g2, t, all, 4.0, 10.0, true, false);
    t += 3 * S;
    tick(lp, g2, t, all, 4.0, 10.0, true, false);
    CHECK(g2.stage == 0);
}

TEST(low_light_warning_raises_and_clears) {
    const LightParams lp;
    LightGov g;
    g.stage = 2;
    std::uint64_t t = 0;
    std::vector<LightEvent> ev;
    tick(lp, g, t, ev, 9.0, 12.0, true, false);
    CHECK(!g.low);
    tick(lp, g, t, ev, 1.5, 12.0, true, false);
    CHECK(g.low);
    tick(lp, g, t, ev, 10.0, 20.0, true, false);
    CHECK(g.low);
    tick(lp, g, t, ev, 2.0, 40.0, true, false);
    CHECK(g.low);
    tick(lp, g, t, ev, 1.5, 40.0, true, false);
    CHECK(!g.low);
    CHECK(ev == (LEV{{LightEvent::LowRaised, 0}, {LightEvent::LowCleared, 0}}));
    LightGov g2;
    g2.stage = 2;
    ev.clear();
    tick(lp, g2, t, ev, 20.0, 5.0, false, false);
    CHECK(!g2.low);
    LightGov g3;
    for (int i = 0; i < 11 * 25; i++) {
        const auto e = g3.step(lp, t, 8.0, true, false, 0);
        ev.insert(ev.end(), e.begin(), e.end());
        t += 40'000'000;
    }
    CHECK(g3.low && g3.stage == 0);
}

TEST(closed_loop_warning_in_a_very_dark_room_and_clear_when_lit) {
    AutoExposure ae{AeParams{}};
    AeSetting applied = power_on();
    Pipe pipe;
    std::vector<std::pair<std::uint64_t, LightEvent>> ev;
    std::uint64_t t = 0;
    run_scene(ae, applied, pipe, 0.08, 1100, t, ev);
    CHECK(ae.light.stage == 2);
    CHECK(ae.light.low);
    const auto r = std::find_if(ev.begin(), ev.end(), [](const auto& e) { return e.second.kind == LightEvent::LowRaised; });
    CHECK(r != ev.end());
    if (r != ev.end()) {
        CHECK(r->first >= 10 * S);
    }
    run_scene(ae, applied, pipe, 20.0, 400, t, ev);
    CHECK(!ae.light.low);
    CHECK(std::any_of(ev.begin(), ev.end(), [](const auto& e) { return e.second.kind == LightEvent::LowCleared; }));
}

TEST(extension_is_off_with_the_env_switch_or_when_the_caps_are_already_the_largest) {
    AeParams off;
    off.light.enabled = false;
    CHECK(AutoExposure(off).top_stage() == 0);
    AeParams big;
    big.exposure_max = 10410;
    big.gain_max = 449;
    CHECK(AutoExposure(big).top_stage() == 0);
    CHECK(AutoExposure(AeParams{}).top_stage() == 2);
    const AutoExposure ae{AeParams{}};
    CHECK(ae.caps(0) == P16(4108, 215));
    CHECK(ae.caps(1) == P16(4108, 449));
    CHECK(ae.caps(2) == P16(10410, 449));
}

TEST(window_scene_is_steady_at_or_above_the_floor) {
    auto scene = [](std::uint16_t e, std::uint16_t g) { return render_window(80.0, e, g, 0.22); };
    const auto r = run_law(AeParams{}, 40, scene, start_bright());
    CHECK(all_le(r.per_s, 15, 2));
    CHECK(std::all_of(r.log.begin(), r.log.end(), [](const auto& l) { return std::get<0>(l) >= 1140; }));
    CHECK(all_le(r.per_s, 0, 5));
    CHECK(!r.ae.light.clip_low);
    const double m = std::get<2>(r.log.back());
    CHECK(m > 40.0);
    const auto old = run_law(AeParams::old_law(), 40, scene, start_bright());
    CHECK(std::get<0>(old.log.back()) < 1140 || std::get<2>(old.log.back()) < m);
}

TEST(big_window_opens_the_low_stage_after_three_seconds_and_never_goes_to_100) {
    auto scene = [](std::uint16_t e, std::uint16_t g) { return render_window(80.0, e, g, 0.45); };
    const auto r = run_law(AeParams{}, 40, scene, start_bright());
    CHECK(r.ae.light.clip_low);
    std::uint16_t lowest = 65535;
    for (const auto& l : r.log) {
        lowest = std::min(lowest, std::get<0>(l));
    }
    CHECK(lowest >= 300 && lowest < 1140);
    const auto first_low = std::find_if(r.log.begin(), r.log.end(), [](const auto& l) { return std::get<0>(l) < 1140; });
    const auto first_floor = std::find_if(r.log.begin(), r.log.end(), [](const auto& l) { return std::get<0>(l) <= 1197; });
    CHECK(first_low != r.log.end() && first_floor != r.log.end());
    CHECK(first_low - r.log.begin() >= (first_floor - r.log.begin()) + 3 * 25);
    CHECK(all_le(r.per_s, 20, 2));
}

TEST(low_stage_returns_when_the_light_changes) {
    AutoExposure ae{AeParams{}};
    AeSetting applied = start_bright();
    std::uint64_t t = 0;
    Pipe pipe;
    for (std::size_t i = 0; i < 1000; i++) {
        land(pipe, i, applied);
        const auto img = i < 500 ? render_window(80.0, applied.exposure[0], applied.gain[0], 0.45)
                                 : render(5.0, applied.exposure[0], applied.gain[0], false);
        const auto st = stats_of(ae, img, img);
        if (ae.update(applied, st, t).send) {
            pipe.emplace_back(i + 2, *ae.target());
        }
        if (i == 499) {
            CHECK(ae.light.clip_low);
        }
        t += 40'000'000;
    }
    CHECK(!ae.light.clip_low);
    CHECK(ae.p.exposure_min == 1140);
}

TEST(dark_room_goes_up_without_oscillation) {
    auto scene = [](std::uint16_t e, std::uint16_t g) { return render(6.0, e, g, false); };
    const auto r = run_law(AeParams{}, 40, scene, start_bright());
    CHECK(all_le(r.per_s, 20, 1));
    const auto [e, g, m, sat] = r.log.back();
    (void)sat;
    CHECK(static_cast<double>(e) * g > 2000.0 * 13.0 * 2.0 && m > 30.0);
}

TEST(steady_room_does_not_move_and_floor_holds) {
    auto scene = [](std::uint16_t e, std::uint16_t g) { return render(80.0, e, g, false); };
    const auto r = run_law(AeParams{}, 30, scene, setting(2000, 13));
    CHECK(all_le(r.per_s, 10, 0));
    const AeSetting t = *r.ae.target();
    CHECK(t.led_exposure == 40 && t.led_gain == 62);
    const AeParams p;
    CHECK(p.exposure_min == 1140);
    CHECK(p.split(1.0).first == 1140);
    CHECK(p.sat_max_frac == 0.20 && p.deadband >= 0.15);
    AutoExposure ae(p);
    const TileStats allsat = ts(250.0, 0.9);
    AeSetting s = start_bright();
    for (std::uint64_t i = 0; i < 40; i++) {
        ae.update(s, {allsat, allsat}, i * 300'000'000);
        s = *ae.target();
        CHECK(s.exposure[0] >= 1140 || ae.light.clip_low);
        CHECK(s.exposure[0] >= 300);
    }
}

TEST(too_bright_rule_only_above_twenty_percent_and_rate_limited) {
    const AutoExposure ae0{AeParams{}};
    const TileStats ok = ts(64.0, 0.19);
    CHECK(ae0.error(ok) == 0.0);
    TileStats bad = ok;
    bad.sat_frac = 0.21;
    CHECK(ae0.error(bad) < 0.0);
    AutoExposure ae{AeParams{}};
    const TileStats dark = ts(20.0);
    AeSetting s = start_bright();
    int changes = 0;
    std::optional<AeSetting> last = ae.target();
    for (std::uint64_t i = 0; i < 100; i++) {
        ae.update(s, {dark, dark}, i * 40'000'000);
        const auto t = ae.target();
        if (i > 0 && t != last) {
            changes++;
        }
        last = t;
        s = *t;
    }
    CHECK(changes <= 5 * 4 + 1);
    CHECK(changes >= 10);
}

TEST(old_law_stays_reachable) {
    const AeParams o = AeParams::old_law();
    CHECK(o.exposure_min == 100 && o.sat_max_frac == 0.05 && o.deadband == 0.08 && o.law == AeLaw::V1 &&
          o.min_change_ns == 0);
    CHECK(AeParams{}.law == AeLaw::V2);
    ::setenv("PIMAX_CAM_AE_LAW", "old", 1);
    const AeParams e = AeParams::from_env();
    ::unsetenv("PIMAX_CAM_AE_LAW");
    CHECK(e.law == AeLaw::V1);
    CHECK(e.exposure_min == 100);
    CHECK(AeParams::from_env().law == AeLaw::V2);
}

TEST(metric_leaves_out_corners_and_clipped_cells) {
    std::vector<std::uint8_t> d(640 * 480, 80);
    for (std::size_t y = 0; y < 480; y++) {
        for (std::size_t x = 0; x < 640; x++) {
            const double dx = (static_cast<double>(x) - 319.5) / 320.0;
            const double dy = (static_cast<double>(y) - 239.5) / 240.0;
            if (dx * dx + dy * dy > 1.0) {
                d[y * 640 + x] = 0;
            }
            if (x >= 512) {
                d[y * 640 + x] = 255;
            }
        }
    }
    auto m = scene_metric_grey(d, 640, 480);
    CHECK(std::fabs(m.mean - 80.0) < 1.0);
    CHECK(m.sat > 0.1 && m.sat < 0.3);
    double plain = 0.0;
    for (const std::uint8_t v : d) {
        plain += v;
    }
    plain /= static_cast<double>(d.size());
    CHECK(std::fabs(plain - 80.0) > 10.0);
    m = scene_metric_grey(std::vector<std::uint8_t>(640 * 480, 255), 640, 480);
    CHECK(m.mean > 250.0 && m.sat > 0.99);
    m = scene_metric_grey(std::vector<std::uint8_t>(640 * 480, 6), 640, 480);
    CHECK(std::fabs(m.mean - 6.0) < 0.5 && m.sat == 0.0 && m.dark > 0.99);
    CHECK(scene_metric_grey({}, 0, 0) == SceneMetric{});
}

TEST(good_frame_definition) {
    auto m = [](double mean, double sat) { return SceneMetric{mean, sat, 0.0, 100}; };
    CHECK(is_good_frame(m(60.0, 0.1), false, std::nullopt, false));
    CHECK(is_good_frame(m(30.0, 0.20), false, GOOD_QUIET_NS, false));
    CHECK(is_good_frame(m(190.0, 0.0), false, std::nullopt, false));
    CHECK(!is_good_frame(m(29.9, 0.0), false, std::nullopt, false));
    CHECK(!is_good_frame(m(190.1, 0.0), false, std::nullopt, false));
    CHECK(!is_good_frame(m(60.0, 0.21), false, std::nullopt, false));
    CHECK(!is_good_frame(m(60.0, 0.1), true, std::nullopt, false));
    CHECK(!is_good_frame(m(60.0, 0.1), false, GOOD_QUIET_NS - 1, false));
    CHECK(!is_good_frame(m(60.0, 0.1), false, std::nullopt, true));
    CHECK(!is_good_frame(SceneMetric{}, false, std::nullopt, false));
    StepWatch w;
    const SceneMetric ok = m(60.0, 0.0);
    std::uint64_t t = 0;
    const std::uint64_t step = 40'000'000;
    CHECK(w.good(ok, 2000, 13, t, false));
    t += step;
    CHECK(w.good(ok, 2000, 13, t, false));
    t += step;
    CHECK(!w.good(ok, 2600, 13, t, false));
    const std::uint64_t t_big = t;
    t += step;
    CHECK(!w.good(ok, 2600, 13, t, false));
    t = t_big + GOOD_QUIET_NS;
    CHECK(w.good(ok, 2600, 13, t, false));
    t += step;
    CHECK(!w.good(ok, 2680, 13, t, false));
    t += step;
    CHECK(w.good(ok, 2680, 13, t, false));
}

TEST(dream_air_defaults_are_unchanged) {
    CHECK(AeParams::for_model(AeModel::DreamAir) == AeParams{});
    CHECK(AeParams::old_law_for(AeModel::DreamAir) == AeParams::old_law());
    const AeParams p;
    CHECK(p.exposure_min == 1140 && p.exposure_max == 4108 && p.gain_min == 13 && p.gain_max == 215 &&
          p.exposure_soft == 4108);
    CHECK(p.model == AeModel::DreamAir);
    CHECK(!model_pair_ranges(AeModel::DreamAir));
    CHECK(model_power_on(AeModel::DreamAir) == P16(10410, 449));
    for (const auto& [x, want] : std::vector<std::pair<double, std::pair<std::uint16_t, std::uint16_t>>>{
             {1.0, {1140, 13}}, {2200.0 * 13.0, {2200, 13}}, {3150.0 * 13.0, {3143, 13}}, {4108.0 * 13.0, {4108, 13}},
             {1e12, {4108, 215}}}) {
        CHECK(p.split(x) == want);
        CHECK(p.split_pair(1, x) == want);
    }
    const AutoExposure ae{AeParams{}};
    const auto [e, g] = ae.caps(2);
    AeParams q;
    q.exposure_max = e;
    q.gain_max = g;
    CHECK(q.split(1e12) == P16(10410, 449));
    CHECK(q.split(4108.0 * 300.0).second == 449);
}

TEST(crystal_tables_are_windows_values) {
    CHECK(std::is_sorted(CRYSTAL_EXPOSURE_LADDER.begin(), CRYSTAL_EXPOSURE_LADDER.end(), std::less_equal<>()) &&
          std::adjacent_find(CRYSTAL_EXPOSURE_LADDER.begin(), CRYSTAL_EXPOSURE_LADDER.end()) == CRYSTAL_EXPOSURE_LADDER.end());
    for (std::size_t i = 1; i < CRYSTAL_GAIN_CODES.size(); i++) {
        CHECK(CRYSTAL_GAIN_CODES[i - 1] < CRYSTAL_GAIN_CODES[i]);
    }
    for (std::size_t i = 1; i < CRYSTAL_HIGH.size(); i++) {
        CHECK(static_cast<std::uint32_t>(CRYSTAL_HIGH[i - 1].first) * CRYSTAL_HIGH[i - 1].second <
              static_cast<std::uint32_t>(CRYSTAL_HIGH[i].first) * CRYSTAL_HIGH[i].second);
    }
    for (const std::uint16_t v : CRYSTAL_EXPOSURE_LADDER) {
        CHECK(contains(EXPOSURE_LADDER, v));
    }
    for (const std::uint16_t e : A4{2200, 2875, 2962, 3758}) {
        CHECK(contains(CRYSTAL_EXPOSURE_LADDER, e));
    }
    CHECK(contains(CRYSTAL_EXPOSURE_LADDER, 3988));
    CHECK(CRYSTAL_HIGH.front() == P16(4194, 215));
    CHECK(CRYSTAL_HIGH.back() == P16(10410, 449));
    CHECK(CRYSTAL_GAIN_CODES.back() == 215);
    double prev = 4108.0 * 215.0;
    for (const auto& [e, g] : CRYSTAL_HIGH) {
        const double x = static_cast<double>(e) * g;
        CHECK(x / prev < 1.085);
        prev = x;
    }
    CHECK(model_power_on(AeModel::CrystalSuper) == P16(3500, 60));
}

TEST(crystal_split_follows_windows) {
    const AeParams p = crystal();
    CHECK(p.exposure_min == 2200 && p.exposure_max == 10410 && p.gain_min == 13 && p.gain_max == 449);
    using P = std::pair<std::uint16_t, std::uint16_t>;
    CHECK(p.split(1.0) == P(2200, 13));
    CHECK(p.split(2200.0 * 13.0) == P(2200, 13));
    CHECK(p.split(2875.0 * 13.0) == P(2875, 13));
    CHECK(p.split(4108.0 * 13.0) == P(4108, 13));
    {
        const auto [e, g] = p.split(3900.0 * 49.0);
        CHECK(contains(CRYSTAL_GAIN_CODES, g) && g <= 49 && e <= 4108);
    }
    for (const auto& [e, g] : CRYSTAL_HIGH) {
        const double x = static_cast<double>(e) * g;
        CHECK(p.split(x) == P(e, g));
        if (e <= 8941 && g <= 439) {
            CHECK(p.split_pair(1, x) == P(e, g));
        }
    }
    CHECK(p.split(1e12) == P(10410, 449));
    CHECK(p.split_pair(1, 1e12) == P(8941, 439));
    CHECK(p.split(4150.0 * 215.0).second == 215);
    double x = 1000.0;
    while (x < 1e8) {
        for (std::size_t k = 0; k < 2; k++) {
            const auto [e, g] = p.split_pair(k, x);
            const PairRange r = CRYSTAL_PAIR_RANGES[k];
            CHECK(r.e_min <= e && e <= r.e_max && r.g_min <= g && g <= r.g_max);
            const bool on_ladder = g < 215 && contains(CRYSTAL_GAIN_CODES, g) && e <= 4108;
            const bool gain13 = g == 13 && contains(CRYSTAL_EXPOSURE_LADDER, e);
            const bool top = g == 215 && e <= 4108 && contains(CRYSTAL_EXPOSURE_LADDER, e);
            CHECK(on_ladder || gain13 || top || in_high(e, g));
        }
        x *= 1.01;
    }
}

TEST(crystal_overrides_stay_in_the_windows_range) {
    ::setenv("PIMAX_CAM_AE_EXP_MIN", "100", 1);
    ::setenv("PIMAX_CAM_AE_GAIN_MAX", "900", 1);
    const AeParams c = AeParams::from_env_model(AeModel::CrystalSuper);
    const AeParams d = AeParams::from_env_model(AeModel::DreamAir);
    ::unsetenv("PIMAX_CAM_AE_EXP_MIN");
    ::unsetenv("PIMAX_CAM_AE_GAIN_MAX");
    CHECK(c.exposure_min == 2200 && c.gain_max == 449);
    CHECK(d.exposure_min == 100 && d.gain_max == 449);
    const AeParams o = AeParams::old_law_for(AeModel::CrystalSuper);
    CHECK(o.exposure_min == 2200 && o.law == AeLaw::V1 && o.model == AeModel::CrystalSuper);
    const AutoExposure ae(crystal());
    CHECK(ae.top_stage() == 0);
    CHECK(!ae.p.light.clip_enabled);
}

TEST(crystal_closed_loop_stays_in_range) {
    for (const auto& [radiance, blob] : std::vector<std::pair<double, bool>>{
             {90.0, true}, {30.0, false}, {300.0, false}, {6.0, false}, {0.5, false}, {3000.0, true}}) {
        AutoExposure ae(crystal());
        AeSetting applied = setting(CRYSTAL_POWER_ON.first, CRYSTAL_POWER_ON.second);
        Pipe pipeline;
        std::uint64_t t = 0;
        std::vector<double> means;
        for (std::size_t i = 0; i < 250; i++) {
            land(pipeline, i, applied);
            const auto lo = render(radiance, applied.exposure[0], applied.gain[0], blob);
            const auto up = render(radiance * 0.6, applied.exposure[2], applied.gain[2], false);
            const auto st = ae.frame_stats({view(lo), view(lo), view(up), view(up)}, Mask{});
            means.push_back(st[0].trimmed_mean);
            const auto d = ae.update(applied, st, t);
            const AeSetting tg = *ae.target();
            CHECK(model_allows(AeModel::CrystalSuper, tg));
            CHECK(tg.exposure[0] == tg.exposure[1] && tg.exposure[2] == tg.exposure[3]);
            CHECK(tg.gain[0] == tg.gain[1] && tg.gain[2] == tg.gain[3]);
            const auto pl = payload(tg);
            CHECK(pl[0] == AE_HEAD[0] && pl[1] == AE_HEAD[1]);
            if (d.send) {
                pipeline.emplace_back(i + 2, tg);
            }
            t += 40'000'000;
        }
        const double m = means.back();
        const std::uint16_t e = applied.exposure[0];
        const std::uint16_t g = applied.gain[0];
        const bool pinned_low = e == 2200 && g == 13;
        const bool pinned_high = e == 10410 && g == 449;
        CHECK(std::fabs(std::log(m / 64.0)) < 0.3 || (pinned_low && m > 64.0) || (pinned_high && m < 64.0));
    }
}

TEST(crystal_fit_and_allows) {
    const AeParams p = crystal();
    using P = std::pair<std::uint16_t, std::uint16_t>;
    CHECK(p.fit_pair(1, 10410, 449) == P(8941, 439));
    CHECK(p.fit_pair(0, 10410, 449) == P(10410, 449));
    CHECK(p.fit_pair(0, 3500, 60) == P(3500, 60));
    CHECK(p.fit_pair(0, 1000, 13) == P(2200, 13));
    const AeSetting ok{{10410, 10410, 8941, 8941}, {449, 449, 439, 439}, 40, 62};
    CHECK(model_allows(AeModel::CrystalSuper, ok));
    AeSetting b = ok;
    b.exposure = {10410, 10410, 10410, 10410};
    CHECK(!model_allows(AeModel::CrystalSuper, b));
    b = ok;
    b.gain = {449, 449, 440, 440};
    CHECK(!model_allows(AeModel::CrystalSuper, b));
    b = ok;
    b.exposure = {2199, 2199, 3000, 3000};
    CHECK(!model_allows(AeModel::CrystalSuper, b));
    b = ok;
    b.led_exposure = 41;
    CHECK(!model_allows(AeModel::CrystalSuper, b));
    b = ok;
    b.exposure = {1, 1, 1, 1};
    CHECK(model_allows(AeModel::DreamAir, b));
    const std::uint8_t first_b[] = {0x58, 0x19, 0x51, 0x00, 0x01, 0x32, 0x98, 0x08, 0x98, 0x08, 0x98, 0x08, 0x98,
                                    0x08, 0x0d, 0x00, 0x0d, 0x00, 0x0d, 0x00, 0x0d, 0x00, 0x28, 0x00, 0x3e, 0x00};
    const auto first = parse_payload(first_b);
    CHECK(first && first->exposure == (A4{2200, 2200, 2200, 2200}));
    CHECK(first && model_allows(AeModel::CrystalSuper, *first));
}

TEST(model_detection) {
    const std::uint16_t a[] = {0x0040, 0x0041, 0x0014};
    CHECK(model_from_pids(a).first == AeModel::CrystalSuper);
    const std::uint16_t b[] = {0x0044, 0x0043, 0x0014};
    CHECK(model_from_pids(b).first == AeModel::DreamAir);
    CHECK(model_from_pids({}).first == AeModel::DreamAir);
    const std::uint16_t c[] = {0x0040, 0x0044};
    CHECK(model_from_pids(c).first == AeModel::DreamAir);
    for (const AeModel m : {AeModel::DreamAir, AeModel::CrystalSuper}) {
        CHECK(model_parse(model_id(m)) == m);
    }
    CHECK(model_parse("Crystal_Super") == AeModel::CrystalSuper);
    CHECK(!model_parse("8kx"));
    CHECK(model_by_pid(0x0040) == AeModel::CrystalSuper);
    CHECK(model_by_pid(0x0044) == AeModel::DreamAir);
    const char* tmp = std::getenv("TMPDIR");
    const std::filesystem::path dir =
        std::filesystem::path(tmp ? tmp : "/tmp") / ("pimax-ae-sysfs-" + std::to_string(::getpid()));
    for (const auto& [n, v, p] : std::vector<std::tuple<const char*, const char*, const char*>>{
             {"1-1", "34a4", "0040"}, {"1-2", "34a4", "0014"}, {"1-3", "046d", "0040"}}) {
        std::filesystem::create_directories(dir / n);
        std::ofstream(dir / n / "idVendor") << v << "\n";
        std::ofstream(dir / n / "idProduct") << p << "\n";
    }
    auto pids = usb_pids_sysfs(dir.string());
    std::sort(pids.begin(), pids.end());
    CHECK(pids == (V16{0x0014, 0x0040}));
    CHECK(model_from_pids(pids).first == AeModel::CrystalSuper);
    std::filesystem::remove_all(dir);
}

// capi_ae.rs (the texts; the ABI tests are in monado-abi/tests/test_ae_capi.cpp)

TEST(create_for_a_named_model_texts) {
    const AutoExposure c(AeParams::for_model(AeModel::CrystalSuper));
    const std::string l = model_line(c, "USB 34a4:0040");
    CHECK(l.starts_with("PIMAX_AE_MODEL: crystal-super (USB 34a4:0040)"));
    CHECK(l.find("upper pair 2200..8941 us / gain 13..439") != std::string::npos);
    CHECK(start_line(c).starts_with("PIMAX_AE_EXTENDED: off"));
    const AutoExposure d{AeParams{}};
    CHECK(model_line(d, "x") == "PIMAX_AE_MODEL: dream-air (x); exposure 1140..4108 us, gain 13..215");
}

TEST(marker_and_toast_text) {
    const AutoExposure ae{AeParams{}};
    CHECK(start_line(ae).starts_with("PIMAX_AE_EXTENDED: armed"));
    CHECK(event_line(LightEvent{LightEvent::LowRaised, 0}, ae).find("too little infrared light") != std::string::npos);
    const std::string m = toast_message(LOW_LIGHT_TEXT);
    CHECK(m.find("\"messageType\":1") != std::string::npos && m.find("infrared") != std::string::npos);
    CHECK(toast_message("a\"b").find("a\\\"b") != std::string::npos);
}

PIBRIDGE_TEST_MAIN()
