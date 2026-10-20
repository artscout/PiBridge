// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/ae.rs and the texts of pimax-cam/src/capi_ae.rs (MIT OR Apache-2.0).
//
// Bit parity with the Rust code: the same operation order, f64::max/min/clamp/round/ln/exp/powf semantics (NaN
// included: rf::fmax/fmin, std::round = half away from zero, Rust's `as u16` saturating with NaN -> 0), and the panics
// of f64::clamp / Ord::clamp with crossed bounds as exceptions (the C ABI turns them into an error return, as the
// Rust catch_unwind guard does).
#include "pibridge/cam/ae.h"

#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"

#include <arpa/inet.h>
#include <dirent.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdio>
#include <format>
#include <limits>
#include <stdexcept>

namespace pibridge::cam::ae {

namespace rf = pibridge::cam::rfmt;

namespace {

/// Rust f64::clamp: panics unless min <= max (NaN bounds included); a NaN value stays NaN.
double rclamp(double x, double lo, double hi) {
    if (!(lo <= hi)) {
        throw std::logic_error("min > max, or either was NaN");
    }
    if (x < lo) {
        x = lo;
    }
    if (x > hi) {
        x = hi;
    }
    return x;
}

/// Rust Ord::clamp for u16: panics unless min <= max.
std::uint16_t uclamp(std::uint16_t x, std::uint16_t lo, std::uint16_t hi) {
    if (lo > hi) {
        throw std::logic_error("assertion failed: min <= max");
    }
    return x < lo ? lo : (x > hi ? hi : x);
}

/// Rust `x as u16` for an f64: saturating, NaN -> 0.
std::uint16_t as_u16(double x) {
    if (std::isnan(x) || x <= 0.0) {
        return 0;
    }
    if (x >= 65535.0) {
        return 65535;
    }
    return static_cast<std::uint16_t>(x);
}

/// Rust f64::total_cmp: a < b.
bool total_less(double a, double b) {
    auto key = [](double x) {
        std::int64_t i = std::bit_cast<std::int64_t>(x);
        i ^= static_cast<std::int64_t>(static_cast<std::uint64_t>(i >> 63) >> 1);
        return i;
    };
    return key(a) < key(b);
}

std::uint64_t sat_sub(std::uint64_t a, std::uint64_t b) {
    return a >= b ? a - b : 0;
}

/// fn held: true once cond has held for ns (the timer starts at the first true frame, resets on false).
bool held(std::optional<std::uint64_t>& t, std::uint64_t now_ns, bool cond, std::uint64_t ns) {
    if (!cond) {
        t.reset();
        return false;
    }
    if (!t) {
        t = now_ns;
    }
    return sat_sub(now_ns, *t) >= ns;
}

/// env_f64: std::env::var(name).ok().and_then(|v| v.trim().parse().ok())
std::optional<double> env_f64(const char* name) {
    const auto v = env_utf8(name);
    if (!v) {
        return std::nullopt;
    }
    double d = 0.0;
    if (!rf::parse_f64(rf::trim(*v), d)) {
        return std::nullopt;
    }
    return d;
}

std::string ascii_lower(std::string_view s) {
    std::string o(s);
    for (char& c : o) {
        if (c >= 'A' && c <= 'Z') {
            c = static_cast<char>(c - 'A' + 'a');
        }
    }
    return o;
}

/// u16::from_str_radix(s, 16)
std::optional<std::uint16_t> parse_hex_u16(std::string_view s) {
    if (!s.empty() && s[0] == '+') {
        s.remove_prefix(1);
    }
    if (s.empty()) {
        return std::nullopt;
    }
    std::uint32_t v = 0;
    for (const char c : s) {
        int d = 0;
        if (c >= '0' && c <= '9') {
            d = c - '0';
        } else if (c >= 'a' && c <= 'f') {
            d = c - 'a' + 10;
        } else if (c >= 'A' && c <= 'F') {
            d = c - 'A' + 10;
        } else {
            return std::nullopt;
        }
        v = v * 16 + static_cast<std::uint32_t>(d);
        if (v > 0xffff) {
            return std::nullopt;
        }
    }
    return static_cast<std::uint16_t>(v);
}

} // namespace

double ladder(double e) {
    return ladder_in(EXPOSURE_LADDER, e);
}

double ladder_down(double e) {
    return ladder_down_in(EXPOSURE_LADDER, e);
}

double ladder_in(std::span<const std::uint16_t> table, double e) {
    const double lo = static_cast<double>(table[0]);
    const double hi = static_cast<double>(table[table.size() - 1]);
    if (e < lo || e > hi) {
        const double k = std::round(std::log(rf::fmax(e, 1.0) / LADDER_ANCHOR) / std::log(LADDER_RATIO));
        return std::round(LADDER_ANCHOR * std::pow(LADDER_RATIO, k));
    }
    double best = lo;
    for (const std::uint16_t v : table) {
        if (std::fabs(static_cast<double>(v) - e) < std::fabs(best - e)) {
            best = static_cast<double>(v);
        }
    }
    return best;
}

double ladder_down_in(std::span<const std::uint16_t> table, double e) {
    for (auto it = table.rbegin(); it != table.rend(); ++it) {
        if (static_cast<double>(*it) <= e) {
            return static_cast<double>(*it);
        }
    }
    const double k = std::floor(std::log(rf::fmax(e, 1.0) / LADDER_ANCHOR) / std::log(LADDER_RATIO) + 1e-9);
    return std::round(LADDER_ANCHOR * std::pow(LADDER_RATIO, k));
}

// ---- the model ----

const char* model_id(AeModel m) {
    return m == AeModel::DreamAir ? "dream-air" : "crystal-super";
}

std::optional<AeModel> model_parse(std::string_view s) {
    std::string v = ascii_lower(rf::trim(s));
    std::replace(v.begin(), v.end(), '_', '-');
    std::replace(v.begin(), v.end(), ' ', '-');
    if (v == "dream-air" || v == "dreamair" || v == "0044" || v == "0x0044") {
        return AeModel::DreamAir;
    }
    if (v == "crystal-super" || v == "crystalsuper" || v == "crystal" || v == "0040" || v == "0x0040") {
        return AeModel::CrystalSuper;
    }
    return std::nullopt;
}

std::optional<AeModel> model_by_pid(std::uint16_t pid) {
    switch (pid) {
    case 0x0044:
        return AeModel::DreamAir;
    case 0x0040:
        return AeModel::CrystalSuper;
    default:
        return std::nullopt;
    }
}

std::span<const std::uint16_t> model_ladder(AeModel m) {
    if (m == AeModel::DreamAir) {
        return EXPOSURE_LADDER;
    }
    return CRYSTAL_EXPOSURE_LADDER;
}

std::span<const std::uint16_t> model_gain_codes(AeModel m) {
    if (m == AeModel::DreamAir) {
        return GAIN_CODES;
    }
    return CRYSTAL_GAIN_CODES;
}

std::span<const std::pair<std::uint16_t, std::uint16_t>> model_high(AeModel m) {
    if (m == AeModel::DreamAir) {
        return {};
    }
    return CRYSTAL_HIGH;
}

std::pair<std::uint16_t, std::uint16_t> model_power_on(AeModel m) {
    if (m == AeModel::DreamAir) {
        return {POWER_ON_EXPOSURE, POWER_ON_GAIN};
    }
    return CRYSTAL_POWER_ON;
}

std::optional<std::array<PairRange, 2>> model_pair_ranges(AeModel m) {
    if (m == AeModel::DreamAir) {
        return std::nullopt;
    }
    return CRYSTAL_PAIR_RANGES;
}

bool model_allows(AeModel m, const AeSetting& s) {
    const auto r = model_pair_ranges(m);
    if (!r) {
        return true;
    }
    auto ok = [&](std::size_t c, const PairRange& p) {
        return p.e_min <= s.exposure[c] && s.exposure[c] <= p.e_max && p.g_min <= s.gain[c] && s.gain[c] <= p.g_max;
    };
    return ok(0, (*r)[0]) && ok(1, (*r)[0]) && ok(2, (*r)[1]) && ok(3, (*r)[1]) && s.led_exposure == 40 &&
           s.led_gain == 62;
}

std::pair<AeModel, std::string> model_detect() {
    if (const auto v = env_utf8("PIMAX_CAM_AE_MODEL")) {
        if (const auto m = model_parse(*v)) {
            return {*m, "PIMAX_CAM_AE_MODEL"};
        }
        if (!rf::trim(*v).empty()) {
            std::fprintf(stderr, "PIMAX_CAM_AE_MODEL=%s unknown (dream-air or crystal-super); looking at USB\n",
                         v->c_str());
        }
    }
    const auto pids = usb_pids_sysfs("/sys/bus/usb/devices");
    const auto [m, why] = model_from_pids(pids);
    return {m, why};
}

std::vector<std::uint16_t> usb_pids_sysfs(const std::string& dir) {
    std::vector<std::uint16_t> out;
    DIR* d = ::opendir(dir.c_str());
    if (d == nullptr) {
        return out;
    }
    while (const dirent* e = ::readdir(d)) {
        const std::string n = e->d_name;
        if (n == "." || n == "..") {
            continue;
        }
        const std::string p = path_join(dir, n);
        auto rd = [&](const char* f) -> std::optional<std::string> {
            auto s = read_to_string(path_join(p, f));
            if (!s) {
                return std::nullopt;
            }
            return ascii_lower(rf::trim(*s));
        };
        if (rd("idVendor") == std::optional<std::string>("34a4")) {
            if (const auto s = rd("idProduct")) {
                if (const auto pid = parse_hex_u16(*s)) {
                    out.push_back(*pid);
                }
            }
        }
    }
    ::closedir(d);
    return out;
}

std::pair<AeModel, const char*> model_from_pids(std::span<const std::uint16_t> pids) {
    const bool crystal = std::find(pids.begin(), pids.end(), 0x0040) != pids.end();
    const bool dream = std::find(pids.begin(), pids.end(), 0x0044) != pids.end();
    if (crystal && !dream) {
        return {AeModel::CrystalSuper, "USB 34a4:0040"};
    }
    if (!crystal && dream) {
        return {AeModel::DreamAir, "USB 34a4:0044"};
    }
    if (crystal && dream) {
        return {AeModel::DreamAir, "both headsets on USB, Dream Air kept"};
    }
    return {AeModel::DreamAir, "no headset on USB, Dream Air default"};
}

// ---- setting and payload ----

AeSetting AeSetting::from_info(const FrameInfo& fi, std::uint16_t led_exposure, std::uint16_t led_gain) {
    return AeSetting{fi.exposure, fi.gain, led_exposure, led_gain};
}

std::array<std::uint8_t, AE_PAYLOAD_LEN> payload(const AeSetting& s) {
    std::array<std::uint8_t, AE_PAYLOAD_LEN> p{};
    p[0] = AE_HEAD[0];
    p[1] = AE_HEAD[1];
    const std::array<std::uint16_t, 10> words{s.exposure[0], s.exposure[1], s.exposure[2], s.exposure[3], s.gain[0],
                                              s.gain[1],     s.gain[2],     s.gain[3],     s.led_exposure, s.led_gain};
    for (std::size_t i = 0; i < words.size(); i++) {
        p[2 + 2 * i] = static_cast<std::uint8_t>(words[i] & 0xff);
        p[3 + 2 * i] = static_cast<std::uint8_t>(words[i] >> 8);
    }
    return p;
}

std::optional<AeSetting> parse_payload(std::span<const std::uint8_t> b) {
    std::span<const std::uint8_t> p = b;
    if (b.size() >= 4 && b[0] == 0x58 && b[1] == 0x19 && b[2] == AE_CMD && b[3] == AE_OPT) {
        p = b.subspan(4);
    }
    if (p.size() < AE_PAYLOAD_LEN || p[0] != AE_HEAD[0] || p[1] != AE_HEAD[1]) {
        return std::nullopt;
    }
    auto w = [&](std::size_t i) {
        return static_cast<std::uint16_t>(p[2 + 2 * i] | (p[3 + 2 * i] << 8));
    };
    return AeSetting{{w(0), w(1), w(2), w(3)}, {w(4), w(5), w(6), w(7)}, w(8), w(9)};
}

// ---- statistics ----

std::uint64_t Histogram::total() const {
    std::uint64_t n = 0;
    for (const std::uint32_t c : bins) {
        n += c;
    }
    return n;
}

void Histogram::merge(const Histogram& o) {
    for (std::size_t i = 0; i < bins.size(); i++) {
        bins[i] += o.bins[i];
    }
}

double Histogram::quantile(double q) const {
    const std::uint64_t n = total();
    if (n == 0) {
        return 0.0;
    }
    const std::uint64_t want = rf::f64_to_u64(rf::fmax(std::ceil(rclamp(q, 0.0, 1.0) * static_cast<double>(n)), 1.0));
    std::uint64_t acc = 0;
    for (std::size_t v = 0; v < bins.size(); v++) {
        acc += bins[v];
        if (acc >= want) {
            return static_cast<double>(v);
        }
    }
    return 255.0;
}

TileStats Histogram::stats(double trim, std::uint8_t sat_level) const {
    const std::uint64_t n = total();
    if (n == 0) {
        return TileStats{};
    }
    // `as usize` of the quantile (0..255, never NaN)
    const std::size_t cut = static_cast<std::size_t>(rf::f64_to_u64(quantile(trim)));
    double sum = 0.0;
    double tsum = 0.0;
    std::uint64_t tn = 0;
    std::uint64_t sat = 0;
    std::uint64_t dark = 0;
    for (std::size_t v = 0; v < bins.size(); v++) {
        const std::uint64_t c = bins[v];
        sum += static_cast<double>(v * c);
        if (v <= cut) {
            tsum += static_cast<double>(v * c);
            tn += c;
        }
        if (v >= sat_level) {
            sat += c;
        }
        if (v <= 8) {
            dark += c;
        }
    }
    const double nf = static_cast<double>(n);
    return TileStats{sum / nf,
                     tn > 0 ? tsum / static_cast<double>(tn) : 0.0,
                     quantile(0.5),
                     quantile(0.95),
                     static_cast<double>(sat) / nf,
                     static_cast<double>(dark) / nf,
                     static_cast<std::uint32_t>(n)};
}

Histogram tile_histogram(const TileView& t, std::size_t step, const Mask& mask) {
    Histogram h;
    step = std::max<std::size_t>(step, 1);
    for (std::size_t y = step / 2; y < t.h; y += step) {
        for (std::size_t x = step / 2; x < t.w; x += step) {
            if (mask.inside(static_cast<double>(x), static_cast<double>(y))) {
                h.bins[px(t, x, y)] += 1;
            }
        }
    }
    return h;
}

SceneMetricHist scene_metric_hist(std::span<const TileView> tiles, std::size_t step, const Mask& mask,
                                  std::uint8_t sat_level) {
    step = std::max<std::size_t>(step, 1);
    Histogram all;
    Histogram kept;
    std::uint64_t sat = 0;
    std::uint64_t dark = 0;
    struct Cell {
        std::uint32_t sum = 0, n = 0, clipped = 0;
    };
    for (const TileView& t : tiles) {
        const std::size_t cw = (t.w + METER_CELL - 1) / METER_CELL;
        const std::size_t ch = (t.h + METER_CELL - 1) / METER_CELL;
        std::vector<Cell> cells(cw * ch);
        std::vector<std::pair<std::uint16_t, std::uint8_t>> samples;
        for (std::size_t y = step / 2; y < t.h; y += step) {
            for (std::size_t x = step / 2; x < t.w; x += step) {
                if (mask.inside(static_cast<double>(x), static_cast<double>(y))) {
                    const std::uint8_t v = px(t, x, y);
                    const std::size_t ci = (y / METER_CELL) * cw + x / METER_CELL;
                    Cell& c = cells.at(ci);
                    c.sum += v;
                    c.n += 1;
                    if (v >= sat_level) {
                        c.clipped += 1;
                        sat += 1;
                    }
                    if (v <= 8) {
                        dark += 1;
                    }
                    all.bins[v] += 1;
                    // `ci as u16`, as the Rust code keeps it
                    samples.emplace_back(static_cast<std::uint16_t>(ci), v);
                }
            }
        }
        for (const auto& [ci, v] : samples) {
            const Cell c = cells.at(ci);
            if (static_cast<double>(c.clipped) <= METER_CELL_CLIP * static_cast<double>(c.n)) {
                kept.bins[v] += 1;
            }
        }
    }
    const std::uint64_t n = all.total();
    const Histogram& src = kept.total() > 0 ? kept : all;
    double m = 0.0;
    if (src.total() > 0) {
        double s = -0.0;
        for (std::size_t v = 0; v < src.bins.size(); v++) {
            s += static_cast<double>(static_cast<std::uint64_t>(v) * static_cast<std::uint64_t>(src.bins[v]));
        }
        m = s / static_cast<double>(src.total());
    }
    const double nf = static_cast<double>(std::max<std::uint64_t>(n, 1));
    return SceneMetricHist{SceneMetric{m, static_cast<double>(sat) / nf, static_cast<double>(dark) / nf,
                                       static_cast<std::uint32_t>(n)},
                           all, kept};
}

SceneMetric scene_metric(std::span<const TileView> tiles, std::size_t step, const Mask& mask, std::uint8_t sat_level) {
    return scene_metric_hist(tiles, step, mask, sat_level).metric;
}

SceneMetric scene_metric_grey(std::span<const std::uint8_t> data, std::size_t w, std::size_t h) {
    if (w == 0 || h == 0 || data.size() < w * h) {
        return SceneMetric{};
    }
    const Mask d;
    const Mask mask{(static_cast<double>(w) - 1.0) / 2.0, (static_cast<double>(h) - 1.0) / 2.0,
                    static_cast<double>(w) / 2.0 * (d.rx / 320.0), static_cast<double>(h) / 2.0 * (d.ry / 240.0)};
    const TileView t{data, w, 0, 0, w, h};
    return scene_metric(std::span<const TileView>(&t, 1), 2, mask, 250);
}

bool is_good_frame(const SceneMetric& m, bool step_landed, std::optional<std::uint64_t> since_big_step_ns,
                   bool settling) {
    return m.n > 0 && GOOD_MEAN_MIN <= m.mean && m.mean <= GOOD_MEAN_MAX && m.sat <= GOOD_SAT_MAX && !step_landed &&
           (!since_big_step_ns || *since_big_step_ns >= GOOD_QUIET_NS) && !settling;
}

std::pair<bool, std::optional<std::uint64_t>> StepWatch::push(std::uint16_t exposure, std::uint16_t gain,
                                                              std::uint64_t now_ns) {
    bool landed = false;
    if (prev_) {
        const auto [pe, pg] = *prev_;
        auto d = [](std::uint16_t a, std::uint16_t b) {
            return std::fabs(static_cast<double>(a) - static_cast<double>(b)) >
                   STEP_LANDED_FRAC * static_cast<double>(std::max<std::uint16_t>(b, 1));
        };
        landed = d(exposure, pe) || d(gain, pg);
        const double a = static_cast<double>(exposure) * static_cast<double>(gain);
        const double b = static_cast<double>(pe) * static_cast<double>(pg);
        const double r = rf::fmax(a, 1.0) / rf::fmax(b, 1.0);
        if (r > BIG_STEP_RATIO || r < 1.0 / BIG_STEP_RATIO) {
            big_at_ = now_ns;
        }
    }
    prev_ = std::pair{exposure, gain};
    std::optional<std::uint64_t> since;
    if (big_at_) {
        since = sat_sub(now_ns, *big_at_);
    }
    return {landed, since};
}

bool StepWatch::good(const SceneMetric& m, std::uint16_t exposure, std::uint16_t gain, std::uint64_t now_ns,
                     bool settling) {
    const auto [landed, since] = push(exposure, gain, now_ns);
    return is_good_frame(m, landed, since, settling);
}

// ---- parameters ----

AeParams AeParams::for_model(AeModel model) {
    if (model == AeModel::DreamAir) {
        return AeParams{};
    }
    const PairRange lo = CRYSTAL_PAIR_RANGES[0];
    AeParams p;
    p.exposure_min = lo.e_min;
    p.exposure_max = lo.e_max;
    p.gain_min = lo.g_min;
    p.gain_max = lo.g_max;
    p.exposure_soft = 4108;
    p.light.ext_exposure = lo.e_max;
    p.light.ext_gain = lo.g_max;
    p.light.clip_enabled = false;
    p.model = model;
    return p;
}

AeParams AeParams::old_law() {
    AeParams p;
    p.deadband = 0.08;
    p.exposure_min = 100;
    p.sat_max_frac = 0.05;
    p.law = AeLaw::V1;
    p.min_change_ns = 0;
    p.sat_release = 0.05;
    p.light.clip_enabled = false;
    return p;
}

AeParams AeParams::old_law_for(AeModel model) {
    if (model == AeModel::DreamAir) {
        return old_law();
    }
    const AeParams o = old_law();
    AeParams p = for_model(model);
    p.deadband = o.deadband;
    p.sat_max_frac = o.sat_max_frac;
    p.law = o.law;
    p.min_change_ns = o.min_change_ns;
    p.sat_release = o.sat_release;
    return p;
}

AeParams AeParams::from_env() {
    return from_env_model(AeModel::DreamAir);
}

AeParams AeParams::from_env_model(AeModel model) {
    bool old = false;
    if (const auto v = env_utf8("PIMAX_CAM_AE_LAW")) {
        const std::string l = ascii_lower(rf::trim(*v));
        old = l == "old" || l == "v1" || l == "off" || l == "0";
    }
    AeParams p = old ? old_law_for(model) : for_model(model);
    // (exposure-cap floor, exposure-floor floor, gain floor, ceilings) for the overrides.
    double emax_lo = 100.0, emin_lo = 1.0, g_lo = 1.0, e_hi = 10410.0, g_hi = 449.0;
    if (const auto r = model_pair_ranges(model)) {
        const PairRange x = (*r)[0];
        emax_lo = x.e_min;
        emin_lo = x.e_min;
        g_lo = x.g_min;
        e_hi = x.e_max;
        g_hi = x.g_max;
    }
    if (const auto v = env_f64("PIMAX_CAM_AE_TARGET")) {
        p.target = rclamp(*v, 20.0, 200.0);
    }
    if (const auto v = env_f64("PIMAX_CAM_AE_EXP_MAX")) {
        p.exposure_max = as_u16(rclamp(*v, emax_lo, e_hi));
    }
    if (const auto v = env_f64("PIMAX_CAM_AE_EXP_MIN")) {
        p.exposure_min = as_u16(rclamp(*v, emin_lo, static_cast<double>(p.exposure_max)));
    }
    if (const auto v = env_f64("PIMAX_CAM_AE_GAIN_MAX")) {
        p.gain_max = as_u16(rclamp(*v, g_lo, g_hi));
    }
    if (const auto v = env_f64("PIMAX_CAM_AE_GAIN_MIN")) {
        p.gain_min = as_u16(rclamp(*v, g_lo, static_cast<double>(p.gain_max)));
    }
    if (const auto v = env_f64("PIMAX_CAM_AE_PERIOD_MS")) {
        p.period_ns = rf::f64_to_u64(rclamp(*v, 40.0, 2000.0) * 1e6);
    }
    if (const auto v = env_f64("PIMAX_CAM_AE_K")) {
        p.k = rclamp(*v, 0.1, 1.0);
    }
    if (const auto v = env_f64("PIMAX_CAM_AE_DEADBAND")) {
        p.deadband = rclamp(*v, 0.0, 1.0);
    }
    if (const auto v = env_f64("PIMAX_CAM_AE_UPPER")) {
        p.upper = *v != 0.0;
    }
    if (const auto v = env_f64("PIMAX_CAM_AE_QUANTIZE")) {
        p.quantize = *v != 0.0;
    }
    if (const auto v = env_f64("PIMAX_CAM_AE_EXTEND")) {
        p.light.enabled = *v != 0.0;
    }
    p.exposure_soft = std::min(p.exposure_max, p.exposure_soft);
    // An explicit exposure cap from the environment is also the soft one.
    if (env_f64("PIMAX_CAM_AE_EXP_MAX")) {
        p.exposure_soft = p.exposure_max;
    }
    return p;
}

std::array<std::uint16_t, 4> AeParams::pair_caps(std::size_t pk) const {
    std::uint16_t elo = exposure_min, ehi = exposure_max, glo = gain_min, ghi = gain_max;
    if (const auto r = model_pair_ranges(model)) {
        const PairRange x = (*r)[std::min<std::size_t>(pk, 1)];
        elo = uclamp(elo, x.e_min, x.e_max);
        ehi = uclamp(ehi, elo, x.e_max);
        glo = uclamp(glo, x.g_min, x.g_max);
        ghi = uclamp(ghi, glo, x.g_max);
    }
    return {elo, std::max(ehi, elo), glo, std::max(ghi, glo)};
}

std::pair<std::uint16_t, std::uint16_t> AeParams::split(double product) const {
    return split_pair(0, product);
}

std::pair<std::uint16_t, std::uint16_t> AeParams::split_pair(std::size_t pk, double product) const {
    const auto [elo, ehi, glo, ghi] = pair_caps(pk);
    auto clamp = [&](double e, double g) -> std::pair<std::uint16_t, std::uint16_t> {
        return {as_u16(rclamp(std::round(e), elo, ehi)), as_u16(rclamp(std::round(g), glo, ghi))};
    };
    const double g_min = static_cast<double>(std::max<std::uint16_t>(glo, 1));
    const double e_lo = elo;
    const double e_hi = ehi;
    const double e_soft = rclamp(static_cast<double>(exposure_soft), e_lo, e_hi);
    if (!quantize) {
        double e = rclamp(product / g_min, e_lo, e_soft);
        const double g = rclamp(product / rf::fmax(e, 1.0), g_min, static_cast<double>(ghi));
        // Gain exhausted: the exposure may go above the soft cap (extended range).
        e = product / g > e ? product / g : e;
        return clamp(e, g);
    }
    const auto table = model_ladder(model);
    if (product / g_min <= e_soft) {
        const double e = rclamp(ladder_in(table, product / g_min), e_lo, e_soft);
        return clamp(e, g_min);
    }
    const auto high = model_high(model);
    std::vector<std::uint16_t> codes;
    for (const std::uint16_t g : model_gain_codes(model)) {
        if (g >= glo && g <= ghi) {
            codes.push_back(g);
        }
    }
    // The power-on gain (Windows' 449) is the one code above the ladder; only when the cap allows it.
    if (high.empty() && ghi >= POWER_ON_GAIN && glo <= POWER_ON_GAIN) {
        codes.push_back(POWER_ON_GAIN);
    }
    const double need = product / e_soft;
    for (const std::uint16_t g : codes) {
        if (static_cast<double>(g) >= need) {
            const double e = rclamp(ladder_down_in(table, product / static_cast<double>(g)), e_lo, e_soft);
            return clamp(e, g);
        }
    }
    if (!high.empty()) {
        // The top of the gain stage and every higher setting inside the caps: the nearest in log brightness.
        std::vector<std::pair<std::uint16_t, std::uint16_t>> cand;
        if (!codes.empty()) {
            cand.push_back(clamp(ladder_down_in(table, e_soft), codes.back()));
        }
        for (const auto& [e, g] : high) {
            if (e >= elo && e <= ehi && g >= glo && g <= ghi) {
                cand.emplace_back(e, g);
            }
        }
        auto dist = [&](const std::pair<std::uint16_t, std::uint16_t>& c) {
            return std::fabs(std::log(rf::fmax(static_cast<double>(c.first) * static_cast<double>(c.second), 1.0) /
                                      rf::fmax(product, 1.0)));
        };
        if (!cand.empty()) {
            std::size_t best = 0;
            double bd = dist(cand[0]);
            for (std::size_t i = 1; i < cand.size(); i++) {
                const double di = dist(cand[i]);
                if (total_less(di, bd)) {
                    best = i;
                    bd = di;
                }
            }
            return cand[best];
        }
    }
    // Every gain code is used up: the exposure grows past the soft cap (exposure last), up to exposure_max.
    const double g = static_cast<double>(codes.empty() ? glo : codes.back());
    const double e = rclamp(ladder_in(table, product / g), e_lo, e_hi);
    return clamp(e, g);
}

std::pair<std::uint16_t, std::uint16_t> AeParams::fit_pair(std::size_t pk, std::uint16_t e, std::uint16_t g) const {
    const auto [elo, ehi, glo, ghi] = pair_caps(pk);
    if (!model_pair_ranges(model) || (elo <= e && e <= ehi && glo <= g && g <= ghi)) {
        return {e, g};
    }
    return split_pair(pk, static_cast<double>(std::max<std::uint16_t>(e, 1)) *
                             static_cast<double>(std::max<std::uint16_t>(g, 1)));
}

// ---- the low-light governor ----

std::vector<LightEvent> LightGov::step(const LightParams& lp, std::uint64_t now_ns, double mean, bool at_ceiling,
                                       bool settled, std::uint8_t top) {
    std::vector<LightEvent> ev;
    // A gap in the tracking frames (camera restart) makes the timers stale.
    if (last_ns_ && sat_sub(now_ns, *last_ns_) > 2'000'000'000ULL) {
        starved_.reset();
        easy_.reset();
        dark_.reset();
        clear_.reset();
    }
    last_ns_ = now_ns;
    const bool starving = stage < top && at_ceiling && mean < lp.extend_mean;
    if (held(starved_, now_ns, starving, lp.extend_ns)) {
        stage = static_cast<std::uint8_t>(stage + 1);
        starved_.reset();
        ev.push_back(LightEvent{LightEvent::Extended, stage});
    }
    const bool easy = stage > 0 && (mean > lp.retract_mean || (settled && mean >= lp.settled_mean));
    if (held(easy_, now_ns, easy, lp.retract_ns)) {
        stage = 0;
        easy_.reset();
        starved_.reset();
        ev.push_back(LightEvent{LightEvent::Retracted, 0});
    }
    // The warning: the full range is in use (or none exists) and the image is still dark.
    const bool full = stage >= top;
    if (!low) {
        if (held(dark_, now_ns, full && at_ceiling && mean < lp.low_mean, lp.low_ns)) {
            low = true;
            dark_.reset();
            clear_.reset();
            ev.push_back(LightEvent{LightEvent::LowRaised, 0});
        }
    } else if (held(clear_, now_ns, mean > lp.clear_mean || !full, lp.clear_ns)) {
        low = false;
        clear_.reset();
        dark_.reset();
        ev.push_back(LightEvent{LightEvent::LowCleared, 0});
    }
    return ev;
}

bool LightGov::step_clip(const LightParams& lp, std::uint64_t now_ns, bool at_floor, bool free) {
    if (!lp.clip_enabled) {
        return std::exchange(clip_low, false);
    }
    if (!clip_low) {
        if (held(clipping_, now_ns, at_floor, lp.clip_ns)) {
            clip_low = true;
            clipping_.reset();
            unneeded_.reset();
            return true;
        }
    } else if (held(unneeded_, now_ns, free, lp.clip_back_ns)) {
        clip_low = false;
        unneeded_.reset();
        clipping_.reset();
        return true;
    }
    return false;
}

// ---- the controller ----

AutoExposure::AutoExposure(AeParams params)
    : p(std::move(params)), base_exposure_max_(p.exposure_max), base_gain_max_(p.gain_max),
      base_exposure_min_(p.exposure_min) {}

std::uint8_t AutoExposure::top_stage() const {
    const LightParams& l = p.light;
    return l.enabled && (base_exposure_max_ < l.ext_exposure || base_gain_max_ < l.ext_gain) ? 2 : 0;
}

std::pair<std::uint16_t, std::uint16_t> AutoExposure::caps(std::uint8_t stage) const {
    const LightParams& l = p.light;
    switch (stage) {
    case 0:
        return {base_exposure_max_, base_gain_max_};
    case 1:
        return {base_exposure_max_, std::max(base_gain_max_, l.ext_gain)};
    default:
        return {std::max(base_exposure_max_, l.ext_exposure), std::max(base_gain_max_, l.ext_gain)};
    }
}

std::vector<LightEvent> AutoExposure::take_events() {
    return std::exchange(events_, {});
}

void AutoExposure::govern(const AeSetting& setting, const std::array<TileStats, 2>& stats, std::uint64_t now_ns) {
    const std::size_t np = p.upper ? 2 : 1;
    double sum = -0.0;
    for (std::size_t k = 0; k < np; k++) {
        sum += stats[k].trimmed_mean;
    }
    const double mean = sum / static_cast<double>(np);
    const auto [ce, cg] = caps(light.stage);
    const auto [ne, ng] = caps(0);
    const auto ranges = model_pair_ranges(p.model);
    bool at_ceiling = true;
    for (std::size_t k = 0; k < np && at_ceiling; k++) {
        const std::size_t c = p.pairs[k][0];
        // A model with Windows ranges per pair: the pair's own top (the Crystal's upper pair ends at 8941 / 439).
        std::uint16_t pe = ce, pg = cg;
        if (ranges) {
            pe = std::min(ce, (*ranges)[k].e_max);
            pg = std::min(cg, (*ranges)[k].g_max);
        }
        at_ceiling = setting.gain[c] >= pg && static_cast<double>(setting.exposure[c]) >= 0.97 * static_cast<double>(pe);
    }
    // "settled": every controlled pair needs less than half of the normal ceiling product.
    const double half = 0.5 * static_cast<double>(ne) * static_cast<double>(ng);
    bool settled = true;
    for (std::size_t k = 0; k < np && settled; k++) {
        const std::size_t c = p.pairs[k][0];
        settled = static_cast<double>(setting.exposure[c]) * static_cast<double>(setting.gain[c]) <= half;
    }
    const std::uint8_t top = top_stage();
    const std::uint8_t before = light.stage;
    const auto ev = light.step(p.light, now_ns, mean, at_ceiling, settled, top);
    if (light.stage != before) {
        const auto [e, g] = caps(light.stage);
        p.exposure_max = e;
        p.gain_max = g;
    }
    events_.insert(events_.end(), ev.begin(), ev.end());
    // The low exposure floor (law V2): a window in view that clips more than 30 % for 3 s at the normal floor.
    const double floor = static_cast<double>(base_exposure_min_) * 1.05;
    bool at_floor = false;
    bool free = true;
    for (std::size_t k = 0; k < np; k++) {
        const double e = static_cast<double>(setting.exposure[p.pairs[k][0]]);
        at_floor = at_floor || (stats[k].sat_frac > p.light.clip_frac && e <= floor);
    }
    for (std::size_t k = 0; k < np && free; k++) {
        free = static_cast<double>(setting.exposure[p.pairs[k][0]]) > floor;
    }
    const LightParams lp = p.light;
    if (light.step_clip(lp, now_ns, at_floor, free)) {
        p.exposure_min = light.clip_low ? std::min(EXPOSURE_LOW_STAGE, base_exposure_min_) : base_exposure_min_;
    }
}

double AutoExposure::error(const TileStats& s) const {
    if (s.n == 0) {
        return 0.0;
    }
    const double m = rf::fmax(s.trimmed_mean, 1.0);
    double err = std::log(p.target / m);
    if (s.sat_frac > p.sat_max_frac) {
        err = rf::fmin(err, -p.sat_pull);
    }
    if (std::fabs(err) < p.deadband) {
        return 0.0;
    }
    return err;
}

std::tuple<std::uint16_t, std::uint16_t, double> AutoExposure::step_pair(std::uint16_t e, std::uint16_t g,
                                                                         const TileStats& s) const {
    return step_pair_with(0, e, g, s, false);
}

std::tuple<std::uint16_t, std::uint16_t, double> AutoExposure::step_pair_k(std::size_t k, std::uint16_t e,
                                                                           std::uint16_t g, const TileStats& s) const {
    return step_pair_with(k, e, g, s, false);
}

std::tuple<std::uint16_t, std::uint16_t, double> AutoExposure::step_pair_with(std::size_t k, std::uint16_t e,
                                                                              std::uint16_t g, const TileStats& s,
                                                                              bool no_brighter) const {
    double err = error(s);
    if (no_brighter && err > 0.0) {
        err = 0.0;
    }
    if (err == 0.0) {
        return {e, g, 0.0};
    }
    const bool far = s.sat_frac > 0.5 || s.dark_frac > 0.8 || s.trimmed_mean > 240.0 || s.trimmed_mean < 8.0;
    const double lim = std::log(rf::fmax(far ? p.max_step_far : p.max_step, 1.0));
    const double r = std::exp(rclamp(p.k * err, -lim, lim));
    const double base = static_cast<double>(std::max<std::uint16_t>(e, 1)) * static_cast<double>(std::max<std::uint16_t>(g, 1));
    const auto [ne, ng] = p.split_pair(k, base * r);
    const double nw = static_cast<double>(ne) * static_cast<double>(ng);
    // The caps can make the split darker than the frame's own setting (e.g. the power-on 10410/449 in a dark room):
    // never step against the error.
    if ((err > 0.0 && nw < base) || (err < 0.0 && nw > base)) {
        return {e, g, err};
    }
    return {ne, ng, err};
}

AeDecision AutoExposure::update(const AeSetting& setting, std::array<TileStats, 2> stats, std::uint64_t now_ns) {
    frames += 1;
    govern(setting, stats, now_ns);
    const std::optional<AeSetting> prev = target_;
    AeSetting next = prev.value_or(AeSetting{setting.exposure, setting.gain, p.led_exposure, p.led_gain});
    std::array<double, 2> errs{0.0, 0.0};
    for (std::size_t k = 0; k < p.pairs.size(); k++) {
        if (k == 1 && !p.upper) {
            continue;
        }
        const std::size_t a = p.pairs[k][0];
        const std::size_t b = p.pairs[k][1];
        // The pair runs on one setting; take the frame's value of the first camera.
        if (p.law == AeLaw::V2) {
            // Hysteresis of the too-bright rule: once it fired, no brightening until the clip is below the release
            // level (no limit cycle at the edge of the threshold).
            if (stats[k].sat_frac > p.sat_max_frac) {
                sat_latch_[k] = true;
            } else if (stats[k].sat_frac < p.sat_release) {
                sat_latch_[k] = false;
            }
        }
        const bool latched = sat_latch_[k];
        auto [e, g, err] = step_pair_with(k, setting.exposure[a], setting.gain[a], stats[k], latched);
        // A model with Windows ranges per pair (Crystal Super): the result always lies inside them, even when the
        // frame's own setting (the firmware default) or a no-step answer does not.
        std::tie(e, g) = p.fit_pair(k, e, g);
        // Rate limit: at most one change per pair per min_change_ns (about 5 changes/s).
        if (err != 0.0 && prev) {
            if (last_change_ns_[k] && sat_sub(now_ns, *last_change_ns_[k]) < p.min_change_ns) {
                err = 0.0;
            } else {
                last_change_ns_[k] = now_ns;
            }
        }
        errs[k] = err;
        if (err != 0.0 || !prev) {
            // Outside the deadband (or first frame): the new value is relative to what this frame used.
            next.exposure[a] = e;
            next.exposure[b] = e;
            next.gain[a] = g;
            next.gain[b] = g;
        }
    }
    if (model_pair_ranges(p.model)) {
        // Also a pair that is not controlled (upper off) goes out inside Windows' ranges, as one equal pair.
        for (std::size_t k = 0; k < p.pairs.size(); k++) {
            const std::size_t a = p.pairs[k][0];
            const std::size_t b = p.pairs[k][1];
            const auto [e, g] = p.fit_pair(k, next.exposure[a], next.gain[a]);
            next.exposure[a] = e;
            next.exposure[b] = e;
            next.gain[a] = g;
            next.gain[b] = g;
        }
    }
    next.led_exposure = p.led_exposure;
    next.led_gain = p.led_gain;
    const bool changed = prev != std::optional<AeSetting>(next);
    if (changed) {
        changes += 1;
    }
    target_ = next;
    const bool due = !last_send_ns_ || sat_sub(now_ns, *last_send_ns_) >= p.period_ns;
    const bool send = due || (changed && !prev);
    if (send) {
        last_send_ns_ = now_ns;
        sends += 1;
    }
    return AeDecision{stats, errs, changed, send};
}

std::array<TileStats, 2> AutoExposure::frame_stats(const std::array<TileView, 4>& tiles, const Mask& mask) const {
    std::array<TileStats, 2> out{};
    for (std::size_t k = 0; k < p.pairs.size(); k++) {
        const auto& pair = p.pairs[k];
        if (p.law == AeLaw::V2) {
            // Masked mean of the unclipped cells is the AE input (in trimmed_mean); the rest describes the whole
            // masked view.
            const std::array<TileView, 2> t{tiles[pair[0]], tiles[pair[1]]};
            const auto mh = scene_metric_hist(t, p.step, mask, p.sat_level);
            TileStats st = mh.all.stats(p.trim, p.sat_level);
            st.trimmed_mean = mh.metric.mean;
            st.sat_frac = mh.metric.sat;
            st.dark_frac = mh.metric.dark;
            out[k] = st;
            continue;
        }
        Histogram h;
        for (const std::size_t t : pair) {
            h.merge(tile_histogram(tiles[t], p.step, mask));
        }
        out[k] = h.stats(p.trim, p.sat_level);
    }
    return out;
}

// ---- texts ----

std::string model_line(const AutoExposure& ae, std::string_view why) {
    const AeParams& p = ae.p;
    std::string ranges;
    if (const auto r = model_pair_ranges(p.model)) {
        const auto& a = (*r)[0];
        const auto& b = (*r)[1];
        ranges = std::format(
            "Windows ranges enforced: lower pair {}..{} us / gain {}..{}, upper pair {}..{} us / gain {}..{}", a.e_min,
            a.e_max, a.g_min, a.g_max, b.e_min, b.e_max, b.g_min, b.g_max);
    } else {
        ranges = std::format("exposure {}..{} us, gain {}..{}", p.exposure_min, p.exposure_max, p.gain_min, p.gain_max);
    }
    return std::format("PIMAX_AE_MODEL: {} ({}); {}", model_id(p.model), why, ranges);
}

std::string start_line(const AutoExposure& ae) {
    if (ae.top_stage() == 0) {
        return "PIMAX_AE_EXTENDED: off (extended range disabled or the normal caps are already the largest)";
    }
    const auto [e, g] = ae.caps(0);
    const auto [xe, xg] = ae.caps(2);
    return std::format("PIMAX_AE_EXTENDED: armed, normal {} us / gain {}, extended up to {} us / gain {} (gain first, "
                       "exposure last)",
                       e, g, xe, xg);
}

std::string event_line(const LightEvent& ev, const AutoExposure& ae) {
    const auto [e, g] = ae.caps(ae.light.stage);
    switch (ev.kind) {
    case LightEvent::Extended:
        return std::format("PIMAX_AE_EXTENDED: range extended to stage {} (caps {} us / gain {})", ev.stage, e, g);
    case LightEvent::Retracted:
        return std::format("PIMAX_AE_EXTENDED: back to the normal range (caps {} us / gain {})", e, g);
    case LightEvent::LowRaised:
        return std::format("PIMAX_AE_LIGHT_LOW: {}", LOW_LIGHT_TEXT);
    case LightEvent::LowCleared:
        break;
    }
    return "PIMAX_AE_LIGHT_LOW: cleared, the cameras see enough light again";
}

std::string toast_message(std::string_view text) {
    std::string q;
    for (const char c : text) {
        if (c == '"') {
            q += "\\\"";
        } else if (c == '\\') {
            q += "\\\\";
        } else if (static_cast<unsigned char>(c) < 0x20) {
            q += ' ';
        } else {
            q += c;
        }
    }
    return std::format(R"({{"messageType":1,"title":"Pimax","content":"{}","timeout":8.0,"volume":-1.0}})", q);
}

void toast(std::string_view text) {
    const int s = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (s < 0) {
        return;
    }
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = 0;
    if (::bind(s, reinterpret_cast<const sockaddr*>(&a), sizeof a) == 0) {
        const std::string m = toast_message(text);
        a.sin_port = htons(42069);
        (void)::sendto(s, m.data(), m.size(), 0, reinterpret_cast<const sockaddr*>(&a), sizeof a);
    }
    ::close(s);
}

} // namespace pibridge::cam::ae
