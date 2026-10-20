// SPDX-License-Identifier: MIT
//
// pibridge-ctrlbench --fuse: the controller fuser (libpibridge-ctrl fuse.h) against a known trajectory.
//
// No recording here has the controller IMU next to the LED frames, so the IMU is synthesised: the truth is the
// tracked trajectory of each hand (the tracker's fixes, smoothed over five fixes, interpolated), the gyro its body
// angular rate and the accelerometer its specific force, at 500 Hz in the fuser's raw units, with the noise of a real
// controller on the desk (omarchyvr fixture inhand_desk: gyro sigma 1.1 LSB, accel 1.2 LSB), a gyro bias error the
// fuser does not know (--bias-err, deg/s per axis), an accel bias of 5 LSB and the 3.6 % accel scale error seen on the
// desk. The optical fixes are the tracker's own (noisy) fixes, handed to the fuser 30 ms after their frame time (the
// live latency). The head is the world origin (the recordings' poses are head-relative).
//
// For each hand and each cut length D (0.5, 1, 2, 5 s) the optical fixes are cut for D seconds at every 0.25 s step
// where the truth covers 1 s before, the cut and 0.7 s after; the shown pose (Fuser::get at 90 Hz) is compared with
// the truth: position / orientation error at fixed times into the cut, how often the fuser gives up (State::None:
// Monado then shows the rest pose), and at relock the largest frame-to-frame step of the shown pose beyond the truth's
// own step (the jump the user sees) and the error 0.1 / 0.3 s after the cut.
#include "bench_fuse.h"

#include "pibridge/ctrl/fuse.h"
#include "pibridge/ctrl/track/solve.h"

#include <algorithm>
#include <deque>
#include <memory>
#include <tuple>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <map>
#include <optional>
#include <random>
#include <string>

namespace pibridge::ctrlbench {

namespace g = cam::geom;
namespace fu = ctrl::fuse;

namespace {

using V3 = fu::V3;
using Q = fu::Q;

Q to_q(const g::Mat3& r) {
    const auto q = g::quat_from_mat(r);
    return {q[0], q[1], q[2], q[3]};
}

struct Seg {
    std::vector<std::uint64_t> t;
    std::vector<V3> p;
    std::vector<Q> q;
    std::vector<std::size_t> fix; // index into the hand's fixes
    std::pair<V3, Q> at(std::uint64_t x) const {
        std::size_t k = static_cast<std::size_t>(std::upper_bound(t.begin(), t.end(), x) - t.begin());
        k = std::clamp<std::size_t>(k, 1, t.size() - 1);
        const double f = std::clamp(static_cast<double>(static_cast<std::int64_t>(x - t[k - 1])) /
                                        static_cast<double>(t[k] - t[k - 1]),
                                    0.0, 1.0);
        V3 pp;
        Q q0 = q[k - 1];
        Q q1 = q[k];
        if (q0[0] * q1[0] + q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3] < 0) {
            q1 = {-q1[0], -q1[1], -q1[2], -q1[3]};
        }
        Q qq;
        for (int i = 0; i < 3; i++) {
            pp[i] = p[k - 1][i] + (p[k][i] - p[k - 1][i]) * f;
        }
        for (int i = 0; i < 4; i++) {
            qq[i] = q0[i] + (q1[i] - q0[i]) * f;
        }
        return {pp, fu::qnorm(qq)};
    }
};

std::vector<Seg> segments(const std::vector<FixIn>& fx) {
    std::vector<Seg> out;
    Seg cur;
    const auto flush = [&] {
        if (cur.t.size() >= 10) {
            // smooth: binomial weights over five fixes
            const std::array<double, 5> w{1, 4, 6, 4, 1};
            Seg s = cur;
            for (std::size_t i = 0; i < cur.t.size(); i++) {
                V3 p{0, 0, 0};
                Q q{0, 0, 0, 0};
                double sw = 0;
                for (int d = -2; d <= 2; d++) {
                    const long j = static_cast<long>(i) + d;
                    if (j < 0 || j >= static_cast<long>(cur.t.size())) {
                        continue;
                    }
                    const double ww = w[static_cast<std::size_t>(d + 2)];
                    Q qj = cur.q[static_cast<std::size_t>(j)];
                    const Q& qi = cur.q[i];
                    if (qj[0] * qi[0] + qj[1] * qi[1] + qj[2] * qi[2] + qj[3] * qi[3] < 0) {
                        qj = {-qj[0], -qj[1], -qj[2], -qj[3]};
                    }
                    for (int k = 0; k < 3; k++) {
                        p[static_cast<std::size_t>(k)] += ww * cur.p[static_cast<std::size_t>(j)][static_cast<std::size_t>(k)];
                    }
                    for (int k = 0; k < 4; k++) {
                        q[static_cast<std::size_t>(k)] += ww * qj[static_cast<std::size_t>(k)];
                    }
                    sw += ww;
                }
                s.p[i] = {p[0] / sw, p[1] / sw, p[2] / sw};
                s.q[i] = fu::qnorm(q);
            }
            out.push_back(std::move(s));
        }
        cur = Seg{};
    };
    for (std::size_t i = 0; i < fx.size(); i++) {
        static const std::uint64_t gap_ns = std::getenv("BENCH_SEG_GAP_MS") ? std::stoull(std::getenv("BENCH_SEG_GAP_MS")) * 1000000ull : 70000000ull;
        if (!cur.t.empty() && fx[i].ns - cur.t.back() > gap_ns) {
            flush();
        }
        cur.t.push_back(fx[i].ns);
        cur.p.push_back(fx[i].body_from_model.t);
        cur.q.push_back(to_q(fx[i].body_from_model.r));
        cur.fix.push_back(i);
    }
    flush();
    return out;
}

double pctl(std::vector<double> v, double q) {
    if (v.empty()) {
        return NAN;
    }
    std::sort(v.begin(), v.end());
    return v[static_cast<std::size_t>(std::round(static_cast<double>(v.size() - 1) * q))];
}

V3 scale3(V3 a, double k) { return {a[0] * k, a[1] * k, a[2] * k}; }

double dist(V3 a, V3 b) { return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2])); }

constexpr double GYRO_LSB_RAD = 0.1207 * M_PI / 180.0;
constexpr double ACC_SCALE_ERR = -0.036;

struct RunOut {
    std::map<int, std::vector<double>> pos_mm, rot_deg; // key: ms into the cut
    int queries_in_cut = 0, none_in_cut = 0;
    double relock_jump_mm = NAN;
    double relock_jump_deg = NAN;
    double err_after_100 = NAN, err_after_300 = NAN;
    std::vector<double> tracked_err_mm; // error while tracked (before the cut)
    int swaps = 0, swaps_taken = 0;     // other-hand fixes injected, and taken by the fuser
};

/// One fuser run over [t0, t2] of one segment with the optical fixes cut in [c0, c1).
RunOut run_one(const Seg& s, const std::vector<FixIn>& fixes, const std::vector<FixIn>& other, double swap_p,
               std::size_t hand, std::uint64_t t0, std::uint64_t c0,
               std::uint64_t c1, std::uint64_t t2, const fu::Config& cfg, std::mt19937& rng, double bias_err_dps) {
    RunOut out;
    std::normal_distribution<double> ng(0.0, 1.1);
    std::normal_distribution<double> na(0.0, 1.2);
    std::uniform_int_distribution<int> sign(0, 1);
    const V3 bias{-4.0, 0.5, -3.0};
    V3 berr;
    V3 abias;
    for (std::size_t k = 0; k < 3; k++) {
        berr[k] = (sign(rng) ? 1.0 : -1.0) * bias_err_dps / 0.1207;
        abias[k] = (sign(rng) ? 1.0 : -1.0) * 5.0;
    }
    fu::Calib cal;
    cal.c = {{{GYRO_LSB_RAD, 0, 0}, {0, GYRO_LSB_RAD, 0}, {0, 0, GYRO_LSB_RAD}}};
    cal.bias = {bias[0] + berr[0], bias[1] + berr[1], bias[2] + berr[2]};
    cal.pairs = 1000;
    cal.moving = 500;
    cal.r2 = 0.95;
    fu::Config c = cfg;
    c.online_fit = false;
    c.use_imu = true;
    fu::Fuser fz(hand, c, cal);
    const fu::P head = fu::P::id();
    const std::uint64_t dt_imu = 2'000'000;
    const std::uint64_t lat = 30'000'000;
    const std::uint64_t dt_q = 11'111'111;
    std::size_t next_fix = 0;
    while (next_fix < s.t.size() && s.t[next_fix] < t0) {
        next_fix++;
    }
    std::uint64_t next_q = t0 + 500'000'000; // queries after a warm-up
    std::optional<fu::P> last_shown;
    std::optional<std::pair<V3, Q>> last_truth;
    std::uint32_t dev = 1000;
    bool after_cut_first = true;
    double max_jump = 0, max_jump_deg = 0;
    const double h = 0.05;
    for (std::uint64_t t = t0; t <= t2; t += dt_imu) {
        dev += 2000;
        // IMU sample at t
        const auto [p, q] = s.at(t);
        const auto q1 = s.at(t + dt_imu).second;
        const V3 w = scale3(fu::qlog(fu::qmul(fu::qconj(q), q1)), 1.0 / (static_cast<double>(dt_imu) * 1e-9));
        const std::uint64_t hn = static_cast<std::uint64_t>(h * 1e9);
        const V3 pa = s.at(t > s.t.front() + hn ? t - hn : s.t.front()).first;
        const V3 pb = s.at(t + hn).first;
        V3 acc_w;
        for (std::size_t k = 0; k < 3; k++) {
            acc_w[k] = (pa[k] - 2.0 * p[k] + pb[k]) / (h * h);
        }
        acc_w[1] += fu::ACCEL_G; // specific force: a - g, g = -9.81 y
        const V3 f_b = fu::qrot(fu::qconj(q), acc_w);
        std::array<std::int16_t, 3> rg{};
        std::array<std::int16_t, 3> ra{};
        for (std::size_t k = 0; k < 3; k++) {
            rg[k] = static_cast<std::int16_t>(std::lround(w[k] / GYRO_LSB_RAD + bias[k] + ng(rng)));
            ra[k] = static_cast<std::int16_t>(
                std::lround(f_b[k] / fu::ACCEL_G * fu::ACCEL_LSB_PER_G * (1.0 + ACC_SCALE_ERR) + abias[k] + na(rng)));
        }
        fz.sample_acc(t, dev, rg, ra);
        // optical fixes that have arrived by t (frame time + latency), outside the cut
        while (next_fix < s.t.size() && s.t[next_fix] + lat <= t) {
            const std::uint64_t ft = s.t[next_fix];
            if (ft < c0 || ft >= c1) {
                const FixIn* fp = &fixes[s.fix[next_fix]];
                bool swapped = false;
                if (swap_p > 0.0 && std::uniform_real_distribution<double>(0.0, 1.0)(rng) < swap_p) {
                    // a left/right swap: this frame's fix of the other hand
                    for (const FixIn& o : other) {
                        if (o.ns + 5'000'000 > ft && o.ns < ft + 5'000'000) {
                            fp = &o;
                            swapped = true;
                            break;
                        }
                    }
                }
                const FixIn& fi = *fp;
                if (std::getenv("BENCH_TRACE") != nullptr && ft + 300'000'000 >= c1 && ft <= c1 + 400'000'000) {
                    std::printf("  fix t-c1 %+.3f leds %zu at %.4f %.4f %.4f\n",
                                static_cast<double>(static_cast<std::int64_t>(ft - c1)) * 1e-9, fi.leds,
                                fi.body_from_model.t[0], fi.body_from_model.t[1], fi.body_from_model.t[2]);
                }
                const char* verdict = fz.optical_with(ft, fu::P{to_q(fi.body_from_model.r), fi.body_from_model.t}, head,
                           static_cast<float>(fi.rms_px), static_cast<std::uint32_t>(fi.leds), std::nullopt);
                if (std::getenv("BENCH_TRACE") != nullptr && ft + 300'000'000 >= c1 && ft <= c1 + 400'000'000) {
                    std::printf("    verdict %s\n", verdict);
                }
                if (swapped) {
                    out.swaps++;
                    const std::string_view v(verdict);
                    out.swaps_taken += v == "ok" || v == "pos-only";
                }
            }
            next_fix++;
        }
        // display queries at 90 Hz (pose for "now")
        if (t >= next_q) {
            next_q += dt_q;
            auto [st, sh] = fz.get(t, head);
            if (st == fu::State::None) {
                // what Monado shows then: the rest pose (head-yaw frame)
                sh.p = {hand == 0 ? -0.15 : 0.15, -0.16, -0.38};
            }
            const auto truth = s.at(t);
            const bool in_cut = t >= c0 && t < c1;
            if (in_cut) {
                out.queries_in_cut++;
                out.none_in_cut += st == fu::State::None;
            }
            if (t < c0 && st != fu::State::None) {
                out.tracked_err_mm.push_back(dist(sh.p, truth.first) * 1000.0);
            }
            if (t >= c1 && last_shown && last_truth) {
                // the relock window: steps beyond the truth's own
                const double step = dist(sh.p, last_shown->p) - dist(truth.first, last_truth->first);
                const double step_deg = fu::qangle(sh.q, last_shown->q) * 180.0 / M_PI -
                                        fu::qangle(truth.second, last_truth->second) * 180.0 / M_PI;
                max_jump = std::max(max_jump, step * 1000.0);
                max_jump_deg = std::max(max_jump_deg, step_deg);
                (void)after_cut_first;
            }
            const double since_end = static_cast<double>(static_cast<std::int64_t>(t - c1)) * 1e-9;
            if (t >= c1 && std::isnan(out.err_after_100) && since_end >= 0.1) {
                out.err_after_100 = dist(sh.p, truth.first) * 1000.0;
            }
            if (t >= c1 && std::isnan(out.err_after_300) && since_end >= 0.3) {
                out.err_after_300 = dist(sh.p, truth.first) * 1000.0;
            }
            if (std::getenv("BENCH_TRACE") != nullptr && t + 300'000'000 >= c1 && t <= c1 + 400'000'000) {
                std::printf("  trace t-c1 %+.3f st %u shown %.4f %.4f %.4f truth %.4f %.4f %.4f err %.1f mm\n",
                            static_cast<double>(static_cast<std::int64_t>(t - c1)) * 1e-9, static_cast<unsigned>(st),
                            sh.p[0], sh.p[1], sh.p[2], truth.first[0], truth.first[1], truth.first[2],
                            dist(sh.p, truth.first) * 1000.0);
            }
            last_shown = sh;
            last_truth = truth;
        }
        // errors at fixed times into the cut
        for (const int ms : {100, 250, 500, 1000, 2000, 4500}) {
            const std::uint64_t tt = c0 + static_cast<std::uint64_t>(ms) * 1'000'000;
            if (!(tt >= t && tt < t + dt_imu && tt < c1)) {
                continue;
            }
            const auto [st, sh] = fz.get(t, head);
            const auto truth = s.at(t);
            if (st == fu::State::None) {
                // Monado shows the rest pose (head-yaw frame): the error to it
                const V3 rest{hand == 0 ? -0.15 : 0.15, -0.16, -0.38};
                out.pos_mm[ms].push_back(dist(rest, truth.first) * 1000.0);
            } else {
                out.pos_mm[ms].push_back(dist(sh.p, truth.first) * 1000.0);
                out.rot_deg[ms].push_back(fu::qangle(sh.q, truth.second) * 180.0 / M_PI);
            }
        }
    }
    out.relock_jump_mm = max_jump;
    out.relock_jump_deg = max_jump_deg;
    return out;
}

} // namespace

void fuse_bench(const FuseBenchIn& in) {
    fu::Config cfg = in.parity ? fu::Config{} : fu::Config::from_env();
    std::mt19937 rng(static_cast<std::mt19937::result_type>(in.seed));
    std::printf("fuse (%s): synthetic IMU (gyro bias error %.2f deg/s per axis), optical latency 30 ms\n",
                in.parity ? "Rust fuser" : "PiBridge fuser", in.bias_err_dps);
    const char* only = std::getenv("BENCH_D");
    int run_no = 0;
    const int want_run = std::getenv("BENCH_RUN") ? std::atoi(std::getenv("BENCH_RUN")) : -1;
    for (const double D : {0.5, 1.0, 2.0, 5.0}) {
        if (only != nullptr && std::atof(only) != D) {
            continue;
        }
        std::map<int, std::vector<double>> pos, rot;
        std::vector<double> jump, jump_deg, e100, e300, tracked;
        int q_cut = 0, q_none = 0, runs = 0, swaps = 0, swaps_taken = 0;
        for (std::size_t hand = 0; hand < 2; hand++) {
            std::vector<FixIn> fx;
            std::vector<FixIn> ofx;
            for (const FixIn& f : in.fixes) {
                (f.slot == hand ? fx : ofx).push_back(f);
            }
            for (const Seg& s : segments(fx)) {
                const std::uint64_t pre = 1'000'000'000;
                const std::uint64_t post = 700'000'000;
                const std::uint64_t d = static_cast<std::uint64_t>(D * 1e9);
                for (std::uint64_t c0 = s.t.front() + pre; c0 + d + post <= s.t.back(); c0 += 250'000'000) {
                    if (want_run >= 0 && run_no++ != want_run) {
                        continue;
                    }
                    if (std::getenv("BENCH_TRACE") != nullptr) {
                        std::printf("run hand %zu cut at %.2f s\n", hand, static_cast<double>(c0 - s.t.front()) * 1e-9);
                    }
                    const RunOut r = run_one(s, fx, ofx, in.swap_p, hand, c0 - pre, c0, c0 + d, c0 + d + post, cfg, rng,
                                             in.bias_err_dps);
                    runs++;
                    if (std::getenv("BENCH_TRACE") != nullptr) {
                        std::printf("  relock jump %.1f mm\n", r.relock_jump_mm);
                    }
                    for (const auto& [k, v] : r.pos_mm) {
                        pos[k].insert(pos[k].end(), v.begin(), v.end());
                    }
                    for (const auto& [k, v] : r.rot_deg) {
                        rot[k].insert(rot[k].end(), v.begin(), v.end());
                    }
                    q_cut += r.queries_in_cut;
                    swaps += r.swaps;
                    swaps_taken += r.swaps_taken;
                    q_none += r.none_in_cut;
                    jump.push_back(r.relock_jump_mm);
                    jump_deg.push_back(r.relock_jump_deg);
                    if (!std::isnan(r.err_after_100)) {
                        e100.push_back(r.err_after_100);
                    }
                    if (!std::isnan(r.err_after_300)) {
                        e300.push_back(r.err_after_300);
                    }
                    tracked.insert(tracked.end(), r.tracked_err_mm.begin(), r.tracked_err_mm.end());
                }
            }
        }
        std::string line;
        for (const auto& [k, v] : pos) {
            line += std::format(" | {:.2f} s: {:.0f}/{:.0f} mm {:.1f}/{:.1f} deg", k / 1000.0, pctl(v, 0.5),
                                pctl(v, 0.9), pctl(rot[k], 0.5), pctl(rot[k], 0.9));
        }
        std::printf("cut %.1f s, %d runs: error p50/p90%s | none %.0f %% | relock jump p50/p90 %.1f/%.1f mm %.1f/%.1f deg, "
                    "error 0.1 s after %.0f/%.0f mm, 0.3 s after %.0f/%.0f mm | tracked error p50 %.1f mm | other-hand fixes "
                    "taken %d of %d\n",
                    D, runs, line.c_str(), q_cut ? 100.0 * q_none / q_cut : 0.0, pctl(jump, 0.5), pctl(jump, 0.9),
                    pctl(jump_deg, 0.5), pctl(jump_deg, 0.9), pctl(e100, 0.5), pctl(e100, 0.9), pctl(e300, 0.5),
                    pctl(e300, 0.9), pctl(tracked, 0.5), swaps_taken, swaps);
    }
}


struct HintSim::Impl {
    struct Hand {
        std::vector<Seg> segs;
        std::unique_ptr<fu::Fuser> f;
        std::uint64_t t = 0;      // IMU time reached
        std::uint32_t dev = 1000;
        V3 bias{-4.0, 0.5, -3.0};
        V3 abias{};
        std::deque<std::tuple<std::uint64_t, fu::P, std::size_t, double>> pending;
    };
    std::array<Hand, 2> h;
    std::mt19937 rng;
    std::normal_distribution<double> ng{0.0, 1.1};
    std::normal_distribution<double> na{0.0, 1.2};
    const Seg* seg_at(std::size_t s, std::uint64_t t) const {
        for (const Seg& g : h[s].segs) {
            if (t >= g.t.front() && t <= g.t.back()) {
                return &g;
            }
        }
        return nullptr;
    }
};

HintSim::HintSim(const std::vector<FixIn>& reference, unsigned long seed, double bias_err_dps) : d_(new Impl) {
    d_->rng.seed(static_cast<std::mt19937::result_type>(seed));
    std::uniform_int_distribution<int> sign(0, 1);
    for (std::size_t s = 0; s < 2; s++) {
        std::vector<FixIn> fx;
        for (const FixIn& f : reference) {
            if (f.slot == s) {
                fx.push_back(f);
            }
        }
        auto& H = d_->h[s];
        H.segs = segments(fx);
        V3 berr;
        for (std::size_t k = 0; k < 3; k++) {
            berr[k] = (sign(d_->rng) ? 1.0 : -1.0) * bias_err_dps / 0.1207;
            H.abias[k] = (sign(d_->rng) ? 1.0 : -1.0) * 5.0;
        }
        fu::Calib cal;
        cal.c = {{{GYRO_LSB_RAD, 0, 0}, {0, GYRO_LSB_RAD, 0}, {0, 0, GYRO_LSB_RAD}}};
        cal.bias = {H.bias[0] + berr[0], H.bias[1] + berr[1], H.bias[2] + berr[2]};
        cal.pairs = 1000;
        cal.moving = 500;
        cal.r2 = 0.95;
        fu::Config c = fu::Config::pibridge();
        c.online_fit = false;
        c.use_imu = true;
        H.f = std::make_unique<fu::Fuser>(s, c, cal);
    }
}

HintSim::~HintSim() { delete d_; }

std::optional<g::Vec3> HintSim::truth(std::size_t slot, std::uint64_t t) const {
    if (const Seg* g = d_->seg_at(slot, t)) {
        return g->at(t).first;
    }
    return std::nullopt;
}

void HintSim::fix(std::size_t slot, std::uint64_t frame_ns, const g::Pose& p, std::size_t leds, double rms_px) {
    d_->h[slot].pending.emplace_back(frame_ns, fu::P{to_q(p.r), p.t}, leds, rms_px);
}

std::array<HintSim::Out, 2> HintSim::at(std::uint64_t frame_ns) {
    std::array<Out, 2> out{};
    const std::uint64_t now = frame_ns + 20'000'000;
    const std::uint64_t dt_imu = 2'000'000;
    const double hh = 0.05;
    const fu::P head = fu::P::id();
    for (std::size_t s = 0; s < 2; s++) {
        auto& H = d_->h[s];
        if (H.t == 0) {
            H.t = frame_ns > 1'000'000'000 ? frame_ns - 1'000'000'000 : 0;
        }
        for (std::uint64_t t = H.t + dt_imu; t <= now; t += dt_imu) {
            H.t = t;
            H.dev += 2000;
            const Seg* sg = d_->seg_at(s, t);
            if (sg == nullptr || t + dt_imu > sg->t.back()) {
                continue; // no truth: no IMU sample (the hint is invalid there anyway)
            }
            const auto [p, q] = sg->at(t);
            const auto q1 = sg->at(t + dt_imu).second;
            const V3 w = scale3(fu::qlog(fu::qmul(fu::qconj(q), q1)), 1.0 / (static_cast<double>(dt_imu) * 1e-9));
            const std::uint64_t hn = static_cast<std::uint64_t>(hh * 1e9);
            const V3 pa = sg->at(t > sg->t.front() + hn ? t - hn : sg->t.front()).first;
            const V3 pb = sg->at(t + hn).first;
            V3 acc_w;
            for (std::size_t k = 0; k < 3; k++) {
                acc_w[k] = (pa[k] - 2.0 * p[k] + pb[k]) / (hh * hh);
            }
            acc_w[1] += fu::ACCEL_G;
            const V3 f_b = fu::qrot(fu::qconj(q), acc_w);
            std::array<std::int16_t, 3> rg{};
            std::array<std::int16_t, 3> ra{};
            for (std::size_t k = 0; k < 3; k++) {
                rg[k] = static_cast<std::int16_t>(std::lround(w[k] / GYRO_LSB_RAD + H.bias[k] + d_->ng(d_->rng)));
                ra[k] = static_cast<std::int16_t>(std::lround(f_b[k] / fu::ACCEL_G * fu::ACCEL_LSB_PER_G *
                                                                  (1.0 + ACC_SCALE_ERR) +
                                                              H.abias[k] + d_->na(d_->rng)));
            }
            H.f->sample_acc(t, H.dev, rg, ra);
            while (!H.pending.empty() && std::get<0>(H.pending.front()) + 30'000'000 <= t) {
                const auto& [ft, pose, leds, rms] = H.pending.front();
                H.f->optical(ft, pose, head, static_cast<float>(rms), static_cast<std::uint32_t>(leds));
                H.pending.pop_front();
            }
        }
        const Seg* sg = d_->seg_at(s, frame_ns);
        if (sg == nullptr) {
            continue;
        }
        const auto st = H.f->get(frame_ns, head).first;
        const auto hp = st == fu::State::None ? std::nullopt : H.f->hint_pose(frame_ns);
        if (!hp) {
            continue;
        }
        const fu::P p = *hp;
        Out o;
        o.valid = true;
        o.state = static_cast<std::uint32_t>(st);
        const auto fx = H.f->fresh_fix(frame_ns, 1e9);
        (void)fx;
        o.fix_age_s = H.f->last_fix_age(frame_ns);
        o.body_from_model = g::Pose{ctrl::track::quat_to_mat({p.q[0], p.q[1], p.q[2], p.q[3]}), p.p};
        out[s] = o;
    }
    return out;
}

} // namespace pibridge::ctrlbench

// ---------- the real IMU log (PIMAX_CTRL_IMU_LOG) ----------

namespace pibridge::ctrlbench {

namespace {

struct LogLine {
    char kind = 0;
    std::uint64_t t = 0;
    std::uint32_t dev = 0;
    std::array<std::int16_t, 3> g{};
    std::optional<std::array<std::int16_t, 3>> a;
    fu::P w;
    fu::P head;
    float rms = 0;
    std::uint32_t leds = 0;
    std::string verdict;
};

std::vector<std::string> split(const std::string& l) {
    std::vector<std::string> out;
    std::size_t at = 0;
    for (;;) {
        const std::size_t c = l.find(',', at);
        out.push_back(l.substr(at, c == std::string::npos ? std::string::npos : c - at));
        if (c == std::string::npos) {
            return out;
        }
        at = c + 1;
    }
}

fu::P pose_at(const std::vector<std::string>& f, std::size_t i) {
    return fu::P{{std::stod(f[i + 3]), std::stod(f[i + 4]), std::stod(f[i + 5]), std::stod(f[i + 6])},
                 {std::stod(f[i]), std::stod(f[i + 1]), std::stod(f[i + 2])}};
}

} // namespace

bool read_imu_log(const std::string& path, std::size_t& hand, std::vector<LogLine>& out, std::string& err);

bool read_imu_log(const std::string& path, std::size_t& hand, std::vector<LogLine>& out, std::string& err) {
    std::FILE* fp = std::fopen(path.c_str(), "r");
    if (fp == nullptr) {
        err = "cannot open " + path;
        return false;
    }
    hand = path.find("right") != std::string::npos ? 1 : 0;
    char buf[1024];
    std::size_t n = 0;
    while (std::fgets(buf, sizeof buf, fp) != nullptr) {
        n++;
        std::string l(buf);
        while (!l.empty() && (l.back() == '\n' || l.back() == '\r')) {
            l.pop_back();
        }
        if (l.empty()) {
            continue;
        }
        if (l[0] == '#') {
            if (const auto k = l.find("hand="); k != std::string::npos) {
                hand = l.compare(k + 5, 5, "right") == 0 ? 1 : 0;
            }
            continue;
        }
        const auto f = split(l);
        try {
            LogLine x;
            x.kind = l[0];
            x.t = std::stoull(f.at(1));
            if (x.kind == 'I' && (f.size() == 6 || f.size() == 9)) {
                x.dev = static_cast<std::uint32_t>(std::stoul(f[2]));
                for (std::size_t k = 0; k < 3; k++) {
                    x.g[k] = static_cast<std::int16_t>(std::stoi(f[3 + k]));
                }
                if (f.size() == 9) {
                    std::array<std::int16_t, 3> a{};
                    for (std::size_t k = 0; k < 3; k++) {
                        a[k] = static_cast<std::int16_t>(std::stoi(f[6 + k]));
                    }
                    x.a = a;
                }
            } else if (x.kind == 'O' && f.size() == 19) {
                x.w = pose_at(f, 2);
                x.head = pose_at(f, 9);
                x.rms = std::stof(f[16]);
                x.leds = static_cast<std::uint32_t>(std::stoul(f[17]));
                x.verdict = f[18];
            } else if (x.kind == 'P' && f.size() >= 17) {
                x.dev = static_cast<std::uint32_t>(std::stoul(f[2])); // the logged state
                x.w = pose_at(f, 3);
                x.head = pose_at(f, 10);
            } else {
                continue;
            }
            out.push_back(std::move(x));
        } catch (...) {
            err = std::format("{}: line {} is not a pimax-ctrlfuse log line", path, n);
            std::fclose(fp);
            return false;
        }
    }
    std::fclose(fp);
    return true;
}

int imu_log_bench(const std::string& path, const std::string& calib_json, bool parity) {
    std::size_t hand = 0;
    std::vector<LogLine> lines;
    std::string err;
    if (!read_imu_log(path, hand, lines, err)) {
        std::fprintf(stderr, "%s\n", err.c_str());
        return 1;
    }
    fu::Config cfg = parity ? fu::Config{} : fu::Config::from_env();
    cfg.hint_path.clear();
    std::optional<fu::Calib> cal = calib_json.empty() ? std::optional<fu::Calib>(fu::default_calib(hand))
                                                      : fu::load_calib(calib_json, hand);
    if (!cal) {
        cal = fu::default_calib(hand);
    }
    std::size_t ni = 0, no = 0, np = 0;
    for (const auto& l : lines) {
        ni += l.kind == 'I';
        no += l.kind == 'O';
        np += l.kind == 'P';
    }
    std::printf("imu log %s: hand %s, %zu IMU samples, %zu optical fixes, %zu shown poses; fuser %s\n", path.c_str(),
                hand == 0 ? "left" : "right", ni, no, np, parity ? "Rust" : "PiBridge");
    // truth: good optical fixes (taken by the logging fuser, 8+ LEDs)
    std::vector<std::pair<std::uint64_t, fu::P>> truth;
    for (const auto& l : lines) {
        if (l.kind == 'O' && l.verdict == "ok" && l.leds >= 8) {
            truth.emplace_back(l.t, l.w);
        }
    }
    std::sort(truth.begin(), truth.end(), [](const auto& x, const auto& y) { return x.first < y.first; });
    const auto truth_at = [&](std::uint64_t t) -> std::optional<V3> {
        const auto it = std::lower_bound(truth.begin(), truth.end(), t,
                                         [](const auto& e, std::uint64_t x) { return e.first < x; });
        if (it == truth.begin() || it == truth.end()) {
            return std::nullopt;
        }
        const auto& b = *it;
        const auto& a = *(it - 1);
        if (b.first - a.first > 100'000'000) {
            return std::nullopt;
        }
        const double f = static_cast<double>(t - a.first) / static_cast<double>(b.first - a.first);
        return V3{a.second.p[0] + (b.second.p[0] - a.second.p[0]) * f, a.second.p[1] + (b.second.p[1] - a.second.p[1]) * f,
                  a.second.p[2] + (b.second.p[2] - a.second.p[2]) * f};
    };
    // 1. the whole log replayed without cuts: how often the replay's verdicts agree with the logged ones, and the
    // overshoot after hard stops (the shown pose past where the hand came to rest, along the motion)
    {
        fu::Fuser fz(hand, cfg, cal);
        std::size_t same = 0, n = 0;
        std::vector<std::pair<std::uint64_t, V3>> shown, logged;
        for (const auto& l : lines) {
            if (l.kind == 'I') {
                fz.sample_acc(l.t, l.dev, l.g, l.a);
            } else if (l.kind == 'O') {
                const std::string v = fz.optical_with(l.t, l.w, l.head, l.rms, l.leds, std::nullopt);
                same += v == l.verdict;
                n++;
            } else {
                const auto [st, p] = fz.get(l.t, l.head);
                if (st != fu::State::None) {
                    shown.emplace_back(l.t, p.p);
                }
                logged.emplace_back(l.t, l.w.p);
            }
        }
        std::printf("replay: %zu of %zu verdicts as logged (all of them with the logging fuser's config and "
                    "calibration)\n",
                    same, n);
        std::vector<std::pair<std::uint64_t, V3>> good;
        for (const auto& l : lines) {
            if (l.kind == 'O' && (l.verdict == "ok" || l.verdict == "pos-only")) {
                good.emplace_back(l.t, l.w.p);
            }
        }
        std::sort(good.begin(), good.end(), [](const auto& x, const auto& y) { return x.first < y.first; });
        std::string ov_r, ov_l;
        std::uint64_t last_stop = 0;
        for (std::size_t i = 3; i + 12 < good.size(); i++) {
            const auto& [t0, p0] = good[i - 3];
            const auto& [t1, p1] = good[i];
            const auto& [t2, p2] = good[i + 3];
            if (t1 - t0 > 150'000'000 || t2 - t1 > 150'000'000) {
                continue;
            }
            const double vb = dist(p0, p1) / (static_cast<double>(t1 - t0) * 1e-9);
            const double va = dist(p1, p2) / (static_cast<double>(t2 - t1) * 1e-9);
            if (vb < 0.6 || va > 0.12 || (last_stop != 0 && t1 - last_stop < 1'000'000'000)) {
                continue;
            }
            last_stop = t1;
            V3 d;
            V3 rest{0, 0, 0};
            for (std::size_t k = 0; k < 3; k++) {
                d[k] = (p1[k] - p0[k]) / dist(p0, p1);
            }
            for (std::size_t j = i + 3; j < i + 12; j++) {
                for (std::size_t k = 0; k < 3; k++) {
                    rest[k] += good[j].second[k] / 9.0;
                }
            }
            const auto over = [&](const std::vector<std::pair<std::uint64_t, V3>>& v) {
                double m = -1e9;
                for (const auto& [t, p] : v) {
                    if (t >= t1 && t <= t1 + 600'000'000) {
                        m = std::max(m, (p[0] - rest[0]) * d[0] + (p[1] - rest[1]) * d[1] + (p[2] - rest[2]) * d[2]);
                    }
                }
                return m * 1000.0;
            };
            ov_r += std::format(" {:.0f}", over(shown));
            ov_l += std::format(" {:.0f}", over(logged));
        }
        std::printf("overshoot after hard stops (mm, per stop): replay%s | as shown live%s\n", ov_r.c_str(),
                    ov_l.c_str());
    }
    // 2. cuts where the hand was well tracked
    if (truth.size() < 20) {
        std::printf("too few good fixes (8+ LEDs) for cuts\n");
        return 0;
    }
    for (const double D : {0.5, 1.0, 2.0, 5.0}) {
        std::map<int, std::vector<double>> pos;
        std::vector<double> jump, e300;
        int runs = 0, none = 0, q = 0;
        const std::uint64_t d = static_cast<std::uint64_t>(D * 1e9);
        for (std::uint64_t c0 = truth.front().first + 3'000'000'000; c0 + d + 500'000'000 <= truth.back().first;
             c0 += 500'000'000) {
            // dense truth over the cut and 0.5 s after (every 100 ms)
            bool dense = true;
            for (std::uint64_t t = c0; t < c0 + d + 500'000'000 && dense; t += 50'000'000) {
                dense = truth_at(t).has_value();
            }
            if (!dense) {
                continue;
            }
            runs++;
            const std::uint64_t c1 = c0 + d;
            fu::Fuser fz(hand, cfg, cal);
            std::optional<fu::P> last;
            std::optional<V3> last_truth;
            double max_jump = 0;
            double err300 = NAN;
            std::array<bool, 6> got{};
            const std::array<int, 6> marks{100, 250, 500, 1000, 2000, 4500};
            for (const auto& l : lines) {
                if (l.t + 5'000'000'000 < c0) {
                    if (l.kind == 'I') {
                        fz.sample_acc(l.t, l.dev, l.g, l.a); // warm-up (bias, gyro ring)
                    }
                    continue;
                }
                if (l.t > c1 + 1'000'000'000) {
                    break;
                }
                if (l.kind == 'I') {
                    fz.sample_acc(l.t, l.dev, l.g, l.a);
                } else if (l.kind == 'O') {
                    if (l.t < c0 || l.t >= c1) {
                        fz.optical_with(l.t, l.w, l.head, l.rms, l.leds, std::nullopt);
                    }
                } else {
                    auto [st, sh] = fz.get(l.t, l.head);
                    const auto tr = truth_at(l.t);
                    if (l.t >= c0 && l.t < c1) {
                        q++;
                        none += st == fu::State::None;
                        for (std::size_t k = 0; k < marks.size(); k++) {
                            const std::uint64_t tt = c0 + static_cast<std::uint64_t>(marks[k]) * 1'000'000;
                            if (!got[k] && tt < c1 && l.t >= tt && tr && st != fu::State::None) {
                                got[k] = true;
                                pos[marks[k]].push_back(dist(sh.p, *tr) * 1000.0);
                            }
                        }
                    }
                    if (l.t >= c1 && st != fu::State::None && last && tr && last_truth) {
                        max_jump = std::max(max_jump, (dist(sh.p, last->p) - dist(*tr, *last_truth)) * 1000.0);
                    }
                    if (l.t >= c1 + 300'000'000 && std::isnan(err300) && tr && st != fu::State::None) {
                        err300 = dist(sh.p, *tr) * 1000.0;
                    }
                    last = st == fu::State::None ? std::nullopt : std::optional<fu::P>(sh);
                    last_truth = tr;
                }
            }
            jump.push_back(max_jump);
            if (!std::isnan(err300)) {
                e300.push_back(err300);
            }
        }
        std::string line;
        for (const auto& [k, v] : pos) {
            line += std::format(" | {:.2f} s: {:.0f}/{:.0f} mm", k / 1000.0, pctl(v, 0.5), pctl(v, 0.9));
        }
        std::printf("cut %.1f s, %d runs: error p50/p90%s | none %.0f %% | relock jump p50/p90 %.1f/%.1f mm | error "
                    "0.3 s after %.0f/%.0f mm\n",
                    D, runs, line.c_str(), q ? 100.0 * none / q : 0.0, pctl(jump, 0.5), pctl(jump, 0.9),
                    pctl(e300, 0.5), pctl(e300, 0.9));
    }
    return 0;
}

int imu_log_selftest(const std::string& dir) {
    // a hand moving on a circle (r 0.1 m, 0.5 Hz) and turning, 20 s; the Rust fuser logs it; the replay must give
    // every logged verdict back and finite errors in the cuts
    const std::string log = dir + "/selftest-imu-log-left.csv";
    const std::string calj = dir + "/selftest-ctrl-imu.json";
    fu::Calib cal;
    cal.c = {{{GYRO_LSB_RAD, 0, 0}, {0, GYRO_LSB_RAD, 0}, {0, 0, GYRO_LSB_RAD}}};
    cal.bias = {-4.0, 0.5, -3.0};
    cal.pairs = 1000;
    cal.moving = 500;
    cal.r2 = 0.95;
    if (!fu::save_calib(calj, 0, cal)) {
        std::fprintf(stderr, "selftest: cannot write %s\n", calj.c_str());
        return 1;
    }
    {
        fu::Config c;
        fu::Fuser fz(0, c, fu::load_calib(calj, 0));
        if (!fz.start_log(log, 10.0)) {
            std::fprintf(stderr, "selftest: cannot write %s\n", log.c_str());
            return 1;
        }
        std::mt19937 rng(7);
        std::normal_distribution<double> n(0.0, 1.0);
        const fu::P head{{0, 0, 0, 1}, {0.0, 1.6, 0.0}};
        const auto pose = [&](double s) {
            const double ph = 2.0 * M_PI * 0.5 * s;
            const V3 p{0.1 * std::cos(ph) + 0.15, 1.3 + 0.1 * std::sin(ph), -0.35};
            const double yaw = 0.3 * std::sin(ph);
            return fu::P{{0.0, std::sin(yaw / 2), 0.0, std::cos(yaw / 2)}, p};
        };
        std::uint32_t dev = 1000;
        for (std::uint64_t t = 1'000'000'000; t < 21'000'000'000; t += 2'000'000) {
            dev += 2000;
            const double s = static_cast<double>(t) * 1e-9;
            const fu::P a = pose(s);
            const fu::P b = pose(s + 0.002);
            const V3 w = scale3(fu::qlog(fu::qmul(fu::qconj(a.q), b.q)), 1.0 / 0.002);
            const double ph = 2.0 * M_PI * 0.5 * s;
            const double om = 2.0 * M_PI * 0.5;
            const V3 acc{-0.1 * om * om * std::cos(ph), fu::ACCEL_G - 0.1 * om * om * std::sin(ph), 0.0};
            const V3 fb = fu::qrot(fu::qconj(a.q), acc);
            std::array<std::int16_t, 3> rg{};
            std::array<std::int16_t, 3> ra{};
            for (std::size_t k = 0; k < 3; k++) {
                rg[k] = static_cast<std::int16_t>(std::lround(w[k] / GYRO_LSB_RAD + cal.bias[k] + n(rng)));
                ra[k] = static_cast<std::int16_t>(std::lround(fb[k] / fu::ACCEL_G * fu::ACCEL_LSB_PER_G + n(rng)));
            }
            fz.sample_acc(t, dev, rg, ra);
            if (t % 34'000'000 == 0) {
                const std::uint64_t ft = t - 30'000'000;
                fu::P o = pose(static_cast<double>(ft) * 1e-9);
                for (std::size_t k = 0; k < 3; k++) {
                    o.p[k] += 0.001 * n(rng);
                }
                fz.optical_with(ft, o, head, 0.8f, 12, std::nullopt);
            }
            if (t % 20'000'000 == 0) {
                fz.get(t, head);
            }
        }
        fz.flush_log();
    }
    return imu_log_bench(log, calj, true);
}

} // namespace pibridge::ctrlbench

namespace pibridge::ctrlbench {

struct ImuFeed::Impl {
    std::array<std::vector<LogLine>, 2> lines;
    std::array<std::size_t, 2> at{0, 0};
    std::array<std::unique_ptr<fu::Fuser>, 2> f;
    std::array<std::uint64_t, 2> pushed{0, 0};
    std::array<std::array<double, 3>, 2> acc{};
    std::array<std::uint32_t, 2> accn{0, 0};
    std::vector<std::pair<std::uint64_t, fu::P>> heads;
    bool ok = false;
    fu::P head_at(std::uint64_t t) const {
        if (heads.empty()) {
            return fu::P::id();
        }
        const auto it = std::lower_bound(heads.begin(), heads.end(), t,
                                         [](const auto& e, std::uint64_t x) { return e.first < x; });
        if (it == heads.end()) {
            return heads.back().second;
        }
        if (it == heads.begin()) {
            return it->second;
        }
        return (t - (it - 1)->first < it->first - t) ? (it - 1)->second : it->second;
    }
};

ImuFeed::ImuFeed(const std::string& left, const std::string& right, const std::string& calib_json) : d_(new Impl) {
    d_->ok = true;
    const std::array<std::string, 2> paths{left, right};
    for (std::size_t s = 0; s < 2; s++) {
        std::size_t hand = s;
        std::string err;
        std::vector<LogLine> all;
        if (!read_imu_log(paths[s], hand, all, err)) {
            std::fprintf(stderr, "%s\n", err.c_str());
            d_->ok = false;
            continue;
        }
        for (auto& l : all) {
            if (l.kind == 'I') {
                d_->lines[s].push_back(std::move(l));
            } else {
                d_->heads.emplace_back(l.t, l.head);
            }
        }
        fu::Config c = fu::Config::pibridge();
        c.online_fit = false;
        c.imu_state = true;
        std::optional<fu::Calib> cal = calib_json.empty() ? std::nullopt : fu::load_calib(calib_json, s);
        d_->f[s] = std::make_unique<fu::Fuser>(s, c, cal ? cal : std::optional<fu::Calib>(fu::default_calib(s)));
    }
    std::sort(d_->heads.begin(), d_->heads.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
}

ImuFeed::~ImuFeed() { delete d_; }


bool ImuFeed::ok() const { return d_->ok; }

std::array<std::vector<ImuFeed::Sample>, 2> ImuFeed::advance(std::uint64_t t) {
    std::array<std::vector<Sample>, 2> out;
    for (std::size_t s = 0; s < 2; s++) {
        auto& L = d_->lines[s];
        auto& i = d_->at[s];
        if (!d_->f[s]) {
            continue;
        }
        while (i < L.size() && L[i].t <= t) {
            d_->f[s]->sample_acc(L[i].t, L[i].dev, L[i].g, L[i].a);
            if (L[i].a && d_->f[s]->calib) {
                const auto& a = *L[i].a;
                const V3 f = d_->f[s]->calib->accel({static_cast<double>(a[0]), static_cast<double>(a[1]),
                                                     static_cast<double>(a[2])});
                d_->acc[s] = {d_->acc[s][0] + f[0], d_->acc[s][1] + f[1], d_->acc[s][2] + f[2]};
                d_->accn[s] += 1;
            }
            const std::uint64_t lt = L[i].t;
            i++;
            if (lt >= d_->pushed[s] + 10'000'000) {
                d_->pushed[s] = lt;
                if (const auto st = d_->f[s]->imu_state(lt)) {
                    const double n = std::max(1.0, static_cast<double>(d_->accn[s]));
                    const fu::P hd = d_->head_at(lt);
                    out[s].push_back(Sample{lt, st->q, st->up, st->up_ok && st->calibrated, st->still_s,
                                            {d_->acc[s][0] / n, d_->acc[s][1] / n, d_->acc[s][2] / n}, hd.q, hd.p});
                }
                d_->acc[s] = {0.0, 0.0, 0.0};
                d_->accn[s] = 0;
            }
        }
    }
    return out;
}

} // namespace pibridge::ctrlbench
