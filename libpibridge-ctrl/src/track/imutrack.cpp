// SPDX-License-Identifier: MIT
// The IMU-led controller tracker (imutrack.h).
#include "pibridge/ctrl/track/imutrack.h"

#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace pibridge::ctrl::track {

namespace g = cam::geom;
namespace rf = cam::rfmt;

namespace {

constexpr double G = 9.80665;
constexpr std::size_t N = 15; // error state: dtheta (world), dp, dv, dbg (aim), dba (world)

using MatN = std::array<double, N * N>;

double& at(MatN& m, std::size_t i, std::size_t j) { return m[i * N + j]; }
double at(const MatN& m, std::size_t i, std::size_t j) { return m[i * N + j]; }

MatN mul(const MatN& a, const MatN& b) {
    MatN c{};
    for (std::size_t i = 0; i < N; i++) {
        for (std::size_t k = 0; k < N; k++) {
            const double x = at(a, i, k);
            if (x == 0.0) {
                continue;
            }
            for (std::size_t j = 0; j < N; j++) {
                c[i * N + j] += x * at(b, k, j);
            }
        }
    }
    return c;
}

MatN transpose(const MatN& a) {
    MatN t{};
    for (std::size_t i = 0; i < N; i++) {
        for (std::size_t j = 0; j < N; j++) {
            t[j * N + i] = at(a, i, j);
        }
    }
    return t;
}

MatN eye() {
    MatN m{};
    for (std::size_t i = 0; i < N; i++) {
        m[i * N + i] = 1.0;
    }
    return m;
}

void put3(MatN& m, std::size_t r, std::size_t c, const g::Mat3& b, double k) {
    for (std::size_t i = 0; i < 3; i++) {
        for (std::size_t j = 0; j < 3; j++) {
            at(m, r + i, c + j) = b[i][j] * k;
        }
    }
}

g::Mat3 skew(const g::Vec3& v) { return {{{0.0, -v[2], v[1]}, {v[2], 0.0, -v[0]}, {-v[1], v[0], 0.0}}}; }

/// Inverse of a small symmetric positive matrix (Gauss-Jordan); false when singular.
template <std::size_t M>
bool invert(std::array<double, M * M>& a) {
    std::array<double, M * M> inv{};
    for (std::size_t i = 0; i < M; i++) {
        inv[i * M + i] = 1.0;
    }
    for (std::size_t c = 0; c < M; c++) {
        std::size_t piv = c;
        for (std::size_t r = c + 1; r < M; r++) {
            if (std::fabs(a[r * M + c]) > std::fabs(a[piv * M + c])) {
                piv = r;
            }
        }
        if (std::fabs(a[piv * M + c]) < 1e-15) {
            return false;
        }
        for (std::size_t j = 0; j < M; j++) {
            std::swap(a[c * M + j], a[piv * M + j]);
            std::swap(inv[c * M + j], inv[piv * M + j]);
        }
        const double d = a[c * M + c];
        for (std::size_t j = 0; j < M; j++) {
            a[c * M + j] /= d;
            inv[c * M + j] /= d;
        }
        for (std::size_t r = 0; r < M; r++) {
            if (r == c) {
                continue;
            }
            const double f = a[r * M + c];
            for (std::size_t j = 0; j < M; j++) {
                a[r * M + j] -= f * a[c * M + j];
                inv[r * M + j] -= f * inv[c * M + j];
            }
        }
    }
    a = inv;
    return true;
}

std::vector<Obs> obs_list(const Rig& rig, const LedModel& model, const std::vector<CamObs>& obs,
                          const std::vector<Matches>& per_cam) {
    std::vector<Obs> o;
    for (std::size_t ci = 0; ci < per_cam.size(); ci++) {
        for (const auto& [l, b] : per_cam[ci]) {
            o.push_back(Obs{&rig.cams[ci], model.leds[l].p, obs[ci].px[b]});
        }
    }
    return o;
}

std::size_t count(const std::vector<Matches>& m) {
    std::size_t t = 0;
    for (const auto& x : m) {
        t += x.size();
    }
    return t;
}

std::uint64_t sat_sub(std::uint64_t a, std::uint64_t b) { return a > b ? a - b : 0; }

double angle_deg(const g::Mat3& a, const g::Mat3& b) {
    return rf::to_degrees(g::norm(rotvec(g::mat_mul(a, g::transpose(b)))));
}

} // namespace

ImuTracker::ImuTracker(ImuTrackCfg c, TrackCfg fallback) : cfg(c), fallback_(fallback) {}

void ImuTracker::imu_push(std::size_t slot, const ImuIn& s) {
    for (const auto& row : s.qg) {
        for (const double x : row) {
            if (!std::isfinite(x)) {
                return;
            }
        }
    }
    for (const double x : s.f_aim) {
        if (!std::isfinite(x)) {
            return;
        }
    }
    auto& d = imu_[slot];
    if (!d.empty() && s.t <= d.back().t) {
        return;
    }
    d.push_back(s);
    if (s.world_from_head.t != g::Vec3{0.0, 0.0, 0.0} || s.world_from_head.r != g::IDENTITY) {
        head_seen_ = s.t;
    }
    // the same IMU for the per-frame tracker's hand identity (used when this one falls back)
    ImuSample is;
    is.t = s.t;
    is.qg = s.qg;
    is.up_aim = s.up_aim;
    is.up_ok = s.up_ok;
    is.up_body = g::mat_vec(g::transpose(s.world_from_head.r), {0.0, 1.0, 0.0});
    is.still_s = s.still_s;
    fallback_.imu_push(slot, is);
    while (!d.empty() && s.t - d.front().t > 3'000'000'000ull) {
        d.pop_front();
    }
}

const ImuIn* ImuTracker::imu_at(std::size_t slot, std::uint64_t t) const {
    const auto& d = imu_[slot];
    const auto it = std::lower_bound(d.begin(), d.end(), t, [](const ImuIn& e, std::uint64_t x) { return e.t < x; });
    const ImuIn* best = nullptr;
    std::uint64_t bd = 30'000'000;
    for (auto j = it == d.begin() ? it : it - 1; j != d.end() && j <= it; ++j) {
        const std::uint64_t dd = j->t > t ? j->t - t : t - j->t;
        if (dd <= bd) {
            bd = dd;
            best = &*j;
        }
    }
    return best;
}

bool ImuTracker::imu_live(std::size_t slot, std::uint64_t t) const {
    return !imu_[slot].empty() && sat_sub(t, imu_[slot].back().t) < 500'000'000 &&
           imu_[slot].back().t + 500'000'000 > t;
}

void ImuTracker::step_state(Ctl& c, double dt, const g::Mat3& dm, const g::Vec3& f_model, double still_s) {
    const g::Mat3 rd = g::mat_mul(c.r, device_from_aim);
    // orientation: the gyro's turn (model frame), less the residual bias
    const g::Vec3 bias_m = g::mat_vec(device_from_aim, c.bg);
    c.r = g::mat_mul(g::mat_mul(c.r, dm), g::rodrigues(g::scale(bias_m, -dt)));
    // position: the specific force in the world, less gravity and the accelerometer bias
    const g::Vec3 fw = g::mat_vec(c.r, f_model);
    const g::Vec3 a = cfg.use_accel ? g::sub(g::sub(fw, g::scale(c.up_w, G)), c.ba) : g::Vec3{0.0, 0.0, 0.0};
    c.p = g::add(c.p, g::add(g::scale(c.v, dt), g::scale(a, 0.5 * dt * dt)));
    c.v = g::add(c.v, g::scale(a, dt));
    const double age = static_cast<double>(sat_sub(c.t, c.last_update)) * 1e-9;
    double damp = age > cfg.free_s ? std::exp(-dt / cfg.tau_v) : 1.0;
    if (still_s > 0.2) {
        damp *= std::exp(-dt / 0.05); // a quiet gyro: the controller is not moving
    }
    c.v = g::scale(c.v, damp);
    // covariance: F P F^T + Q
    MatN F = eye();
    put3(F, 0, 9, rd, -dt);
    put3(F, 3, 6, g::IDENTITY, dt);
    put3(F, 6, 0, skew(fw), -dt);
    put3(F, 6, 12, g::IDENTITY, -dt);
    MatN P = mul(mul(F, c.P), transpose(F));
    const double turn = g::norm(rotvec(dm));
    const double sg2 = 0.003 * 0.003 * dt + std::pow(0.08 * turn, 2); // gyro noise + its scale / axis error
    const double sa2 = cfg.sigma_a * cfg.sigma_a * dt;
    for (std::size_t i = 0; i < 3; i++) {
        at(P, i, i) += sg2;
        at(P, 6 + i, 6 + i) += sa2;
        at(P, 9 + i, 9 + i) += 2e-4 * 2e-4 * dt;
        at(P, 12 + i, 12 + i) += 0.02 * 0.02 * dt;
    }
    c.P = P;
}

void ImuTracker::propagate(std::size_t slot, std::uint64_t t) {
    Ctl& c = c_[slot];
    const auto& d = imu_[slot];
    for (const ImuIn& s : d) {
        if (s.t <= c.t) {
            continue;
        }
        const bool partial = s.t > t;
        const double frac = partial ? static_cast<double>(t - c.t) / static_cast<double>(s.t - c.t) : 1.0;
        if (frac <= 0.0) {
            break;
        }
        const std::uint64_t to = partial ? t : s.t;
        if (c.has_ref) {
            g::Mat3 da = g::mat_mul(g::transpose(c.qg_ref), s.qg);
            if (partial) {
                da = g::rodrigues(g::scale(rotvec(da), frac));
            }
            const g::Mat3 dm = g::mat_mul(g::mat_mul(device_from_aim, da), g::transpose(device_from_aim));
            const double dt = static_cast<double>(to - c.t) * 1e-9;
            if (c.has_state && dt > 0.0 && dt < 0.1) {
                step_state(c, dt, dm, g::mat_vec(device_from_aim, s.f_aim), s.still_s);
            } else if (c.has_state) {
                c.r = g::mat_mul(c.r, dm);
            }
            // the reference: the sample itself (products of products lose orthonormality), part of it when partial
            c.qg_ref = partial ? g::mat_mul(c.qg_ref, da) : s.qg;
        } else {
            c.qg_ref = s.qg;
            c.has_ref = true;
        }
        c.t = to;
        if (partial) {
            break;
        }
    }
    if (c.t < t && !c.has_ref) {
        c.t = t;
    }
}

bool ImuTracker::update(Ctl& c, const g::Pose& m, double sig_p, double sig_r, bool use_rot) {
    // z = (rotation error, position error); H selects dtheta and dp
    std::array<double, 6> z{};
    const g::Vec3 dth = rotvec(g::mat_mul(m.r, g::transpose(c.r)));
    const g::Vec3 dp = g::sub(m.t, c.p);
    for (std::size_t i = 0; i < 3; i++) {
        z[i] = use_rot ? dth[i] : 0.0;
        z[3 + i] = dp[i];
    }
    std::array<double, 36> S{};
    for (std::size_t i = 0; i < 6; i++) {
        for (std::size_t j = 0; j < 6; j++) {
            S[i * 6 + j] = at(c.P, i, j);
        }
    }
    for (std::size_t i = 0; i < 3; i++) {
        S[i * 6 + i] += use_rot ? sig_r * sig_r : 1e6;
        S[(3 + i) * 6 + 3 + i] += sig_p * sig_p;
    }
    std::array<double, 36> Si = S;
    if (!invert<6>(Si)) {
        return false;
    }
    double d2 = 0.0;
    for (std::size_t i = 0; i < 6; i++) {
        for (std::size_t j = 0; j < 6; j++) {
            d2 += z[i] * Si[i * 6 + j] * z[j];
        }
    }
    if (d2 > cfg.gate_chi2) {
        return false;
    }
    // K = P H^T S^-1 (15 x 6)
    std::array<double, N * 6> K{};
    for (std::size_t i = 0; i < N; i++) {
        for (std::size_t j = 0; j < 6; j++) {
            double s = 0.0;
            for (std::size_t k = 0; k < 6; k++) {
                s += at(c.P, i, k) * Si[k * 6 + j];
            }
            K[i * 6 + j] = s;
        }
    }
    std::array<double, N> dx{};
    for (std::size_t i = 0; i < N; i++) {
        for (std::size_t j = 0; j < 6; j++) {
            dx[i] += K[i * 6 + j] * z[j];
        }
    }
    c.r = g::mat_mul(g::rodrigues({dx[0], dx[1], dx[2]}), c.r);
    for (std::size_t i = 0; i < 3; i++) {
        c.p[i] += dx[3 + i];
        c.v[i] += dx[6 + i];
        c.bg[i] += dx[9 + i];
        c.ba[i] += dx[12 + i];
    }
    // P = (I - K H) P
    MatN P = c.P;
    for (std::size_t i = 0; i < N; i++) {
        for (std::size_t j = 0; j < N; j++) {
            double s = 0.0;
            for (std::size_t k = 0; k < 6; k++) {
                s += K[i * 6 + k] * at(c.P, k, j);
            }
            P[i * N + j] -= s;
        }
    }
    c.P = P;
    return true;
}

std::vector<Fix> ImuTracker::step(const Rig& rig, const LedFrame& lf, std::uint64_t frame_ns) {
    frame_no_ += 1;
    fallback_.device_from_aim = device_from_aim;
    // no IMU for either controller: the per-frame tracker
    if ((!imu_live(0, frame_ns) && !imu_live(1, frame_ns)) || (cfg.use_accel && sat_sub(frame_ns, head_seen_) > 1'000'000'000)) {
        counts.fallback_frames += 1;
        fallback_.wanted = wanted;
        return fallback_.step(rig, lf, frame_ns);
    }
    Stats& st = fallback_.stats;
    st.led_frames += 1;
    for (std::size_t s = 0; s < 2; s++) {
        propagate(s, frame_ns);
    }
    std::vector<CamObs> obs;
    for (std::size_t ci = 0; ci < rig.cams.size() && ci < lf.px.size(); ci++) {
        obs.push_back(CamObs::make(rig.cams[ci], lf.px[ci]));
    }
    std::vector<std::vector<bool>> used;
    for (const CamObs& o : obs) {
        used.emplace_back(o.px.size(), false);
    }
    const double cos_face = std::cos(rf::to_radians(rig.prm.facing_deg));
    std::vector<Fix> out;
    std::vector<Found> pre;
    const auto head_at = [&](std::size_t s) {
        const ImuIn* im = imu_at(s, frame_ns);
        return im != nullptr ? im->world_from_head : g::Pose{};
    };
    const auto publish = [&](std::size_t s, Ctl& c, Found f, bool few) {
        const g::Pose wh = head_at(s);
        const g::Pose hw = wh.inverse();
        Fix fx{s, std::move(f), few, g::Pose{g::mat_mul(hw.r, c.r), hw.apply(c.p)}};
        st.hand[s] += 1;
        if (few) {
            st.few[s] += 1;
        }
        last_fix_ns_ = frame_ns;
        out.push_back(std::move(fx));
    };

    // 1. controllers with a state: correspondences from the prediction (the tighter prediction first)
    std::array<std::size_t, 2> order{0, 1};
    const auto spread = [&](std::size_t s) { return at(c_[s].P, 3, 3) + at(c_[s].P, 4, 4) + at(c_[s].P, 5, 5); };
    if (spread(1) < spread(0)) {
        order = {1, 0};
    }
    for (const std::size_t s : order) {
        Ctl& c = c_[s];
        if (!wanted[s] || c.status == Status::Lost || !c.has_state) {
            continue;
        }
        st.tries += 1;
        const LedModel& model = rig.models[s];
        const g::Pose wh = head_at(s);
        const g::Pose hw = wh.inverse();
        const g::Pose pred{g::mat_mul(hw.r, c.r), hw.apply(c.p)};
        const double sp = std::sqrt(spread(s) / 3.0);
        const double sr = std::sqrt((at(c.P, 0, 0) + at(c.P, 1, 1) + at(c.P, 2, 2)) / 3.0);
        const double gate = std::clamp(3.0 * 230.0 * (sp + 0.07 * sr) / 0.4 + 3.0, cfg.gate_min_px, cfg.gate_max_px);
        const std::optional<std::pair<g::Mat3, double>> prior =
            std::pair{pred.r, std::clamp(1.0 / std::max(sr, 1e-3), 30.0, 1500.0)};
        bool ok = false;
        // a tight window first (the prediction is usually within a few mm), the covariance's window after
        for (const double win : {std::min(gate, 10.0), gate}) {
        if (ok) {
            break;
        }
        std::vector<Matches> m = match_all(pred, rig, model, obs, used, cos_face, win);
        if (std::getenv("CT_IMUTRACK_DEBUG") != nullptr) {
            std::fprintf(stderr, "itk %zu slot %zu status %d pred (%.3f %.3f %.3f) v (%.2f %.2f %.2f) gate %.1f sp %.4f sr %.4f matches %zu\n",
                         static_cast<std::size_t>(frame_no_), s, static_cast<int>(c.status), pred.t[0], pred.t[1], pred.t[2],
                         c.v[0], c.v[1], c.v[2], gate, sp, sr, count(m));
        }
        if (count(m) < cfg.min_leds) {
            counts.miss_window += 1;
        }
        if (count(m) >= cfg.min_leds) {
            g::Pose p1 = refine_with(pred, obs_list(rig, model, obs, m), true, 20, prior).first;
            std::vector<Matches> m2 = match_all(p1, rig, model, obs, used, cos_face, rig.prm.tol_px * 2.0);
            const std::size_t n2 = count(m2);
            if (n2 < cfg.min_leds) {
                counts.miss_refined += 1;
            }
            if (n2 >= cfg.min_leds) {
                auto [p2, rms] = refine_with(p1, obs_list(rig, model, obs, m2), true, 20, prior);
                bool free_rot = false;
                if (n2 >= 5) {
                    // enough LEDs for a free orientation: the optics measure it
                    const auto [p3, rms3] = refine(p2, obs_list(rig, model, obs, m2), true, 20);
                    if (rms3 <= rig.prm.tol_px && angle_deg(p3.r, pred.r) < 45.0) {
                        p2 = p3;
                        rms = rms3;
                        free_rot = true;
                    }
                }
                if (std::getenv("CT_IMUTRACK_DEBUG") != nullptr) {
                    std::fprintf(stderr, "   n2 %zu rms %.2f free %d dpos %.1f mm drot %.1f deg\n", n2, rms, free_rot ? 1 : 0,
                                 g::norm(g::sub(p2.t, pred.t)) * 1000.0, angle_deg(p2.r, pred.r));
                }
                if (rms > (n2 == 3 ? 1.2 : rig.prm.tol_px)) {
                    counts.miss_rms += 1;
                }
                if (rms <= (n2 == 3 ? 1.2 : rig.prm.tol_px)) {
                    const g::Pose meas{g::mat_mul(wh.r, p2.r), wh.apply(p2.t)};
                    const double sigp = n2 >= 8 ? cfg.sigma_p[0] : n2 >= 5 ? cfg.sigma_p[1] : cfg.sigma_p[2];
                    const double sigr = rf::to_radians(n2 >= 8 ? 2.5 : 5.0);
                    if (update(c, meas, sigp, sigr, free_rot)) {
                        ok = true;
                        if (const ImuIn* im = imu_at(s, frame_ns); im != nullptr && im->up_ok && free_rot && n2 >= 8) {
                            const g::Vec3 u = g::mat_vec(g::mat_mul(meas.r, device_from_aim), im->up_aim);
                            const double k = std::max(0.02, 1.0 / static_cast<double>(c.up_n + 1));
                            c.up_w = g::normalize(g::add(g::scale(c.up_w, 1.0 - k), g::scale(u, k)));
                            c.up_n += 1;
                        }
                        counts.updates += 1;
                        c.last_update = frame_ns;
                        c.misses = 0;
                        c.streak += 1;
                        JointSolve j{p2, std::move(m2), rms};
                        claim(used, j);
                        Found f{s, std::move(j), std::nullopt, 0.0, 0};
                        pre.push_back(f);
                        if (c.status == Status::Tentative && c.streak >= cfg.confirm) {
                            c.status = Status::Tracking;
                        }
                        if (c.status == Status::Tracking) {
                            publish(s, c, std::move(f), n2 < 5);
                        }
                    } else {
                        counts.gate_rejects += 1;
                    }
                }
            }
        }
        }
        if (!ok) {
            counts.misses += 1;
            c.misses += 1;
            c.streak = 0;
            if (c.status == Status::Tentative) {
                c.status = Status::Lost;
                counts.tentative_dropped += 1;
            } else if (c.misses >= cfg.miss_lost) {
                c.status = Status::Lost;
            }
        }
    }

    // 2. lost controllers: the brute-force search, every hypothesis checked against the controller's IMU
    std::array<bool, 2> need{false, false};
    for (std::size_t s = 0; s < 2; s++) {
        const ImuIn* im = imu_at(s, frame_ns);
        bool updated = false;
        for (const Fix& f : out) {
            updated = updated || f.slot == s;
        }
        // lost, or not found from the prediction this frame (a tentative track is found from it or dropped)
        need[s] = wanted[s] && c_[s].status != Status::Tentative && !updated && c_[s].last_update != frame_ns &&
                  imu_live(s, frame_ns) && im != nullptr && im->still_s < cfg.imu_parked_s;
    }
    const bool idle = out.empty() && sat_sub(frame_ns, last_fix_ns_) > 2'000'000'000;
    if ((need[0] || need[1]) && (!idle || frame_no_ % 3 == 0)) {
        const std::vector<Found> got = solve_frame(lf, rig.cams, rig.models, rig.triples, rig.prm, nullptr, pre, true);
        st.searches += 1;
        struct Pick {
            std::size_t cand;
            JointSolve j;
            double err;
        };
        std::array<std::optional<Pick>, 2> best;
        for (std::size_t s = 0; s < 2; s++) {
            if (!need[s]) {
                continue;
            }
            const ImuIn* im = imu_at(s, frame_ns);
            const g::Pose wh = im->world_from_head;
            for (std::size_t k = 0; k < got.size(); k++) {
                const Found& gf = got[k];
                const std::optional<JointSolve> j =
                    gf.model == s ? std::optional<JointSolve>(gf.joint) : gf.alt;
                if (!j || j->total() < 5 || j->rms_px > rig.prm.tol_px) {
                    continue;
                }
                const g::Mat3 rw = g::mat_mul(wh.r, j->body_from_model.r);
                double err = 0.0;
                bool known = false;
                if (im->up_ok && c_[s].up_n >= 20) {
                    const g::Vec3 up = g::mat_vec(g::mat_mul(rw, device_from_aim), im->up_aim);
                    const double tilt = g::angle_deg(up, c_[s].up_w);
                    if (tilt > cfg.acq_tilt_deg) {
                        counts.acq_refused += 1;
                        continue;
                    }
                    err = tilt;
                    known = true;
                }
                if (c_[s].has_state && sat_sub(frame_ns, c_[s].last_update) < 10'000'000'000ull) {
                    const double e = angle_deg(rw, c_[s].r);
                    if (e > cfg.acq_rot_deg) {
                        counts.acq_refused += 1;
                        continue;
                    }
                    // a tracked controller missed by its prediction: also near where it should be
                    const double age = static_cast<double>(sat_sub(frame_ns, c_[s].last_update)) * 1e-9;
                    if (c_[s].status == Status::Tracking &&
                        g::norm(g::sub(wh.apply(j->body_from_model.t), c_[s].p)) > 0.15 + 1.0 * age) {
                        counts.acq_refused += 1;
                        continue;
                    }
                    err = std::max(err, e);
                    known = true;
                }
                // without an IMU reference yet the acquisition is tentative: the filter, turned by this
                // controller's gyro, must then agree with the next fits (a mirror fit or the other hand does not)
                (void)known;
                if (!best[s] || j->total() > best[s]->j.total() ||
                    (j->total() == best[s]->j.total() && err < best[s]->err)) {
                    best[s] = Pick{k, *j, err};
                }
            }
        }
        if (best[0] && best[1] && best[0]->cand == best[1]->cand) {
            // one controller's LEDs: the hand whose IMU agrees better takes it
            best[best[0]->err <= best[1]->err ? 1 : 0].reset();
        }
        for (std::size_t s = 0; s < 2; s++) {
            if (!best[s]) {
                continue;
            }
            Ctl& c = c_[s];
            const ImuIn* im = imu_at(s, frame_ns);
            const g::Pose wh = im->world_from_head;
            const JointSolve& j = best[s]->j;
            const bool known_orient = c.has_state && sat_sub(frame_ns, c.last_update) < 10'000'000'000ull;
            if (c.status == Status::Tracking && known_orient) {
                // a tracked controller the prediction missed: the search's fit is a measurement, not a restart
                const g::Pose meas{g::mat_mul(wh.r, j.body_from_model.r), wh.apply(j.body_from_model.t)};
                const std::size_t n = j.total();
                const double sigp = n >= 8 ? cfg.sigma_p[0] : n >= 5 ? cfg.sigma_p[1] : cfg.sigma_p[2];
                if (update(c, meas, sigp, rf::to_radians(n >= 8 ? 2.5 : 5.0), true)) {
                    counts.updates += 1;
                    c.last_update = frame_ns;
                    c.misses = 0;
                    c.streak += 1;
                    publish(s, c, Found{s, j, std::nullopt, 0.0, 0}, false);
                    continue;
                }
                counts.gate_rejects += 1;
                if (c.misses < cfg.miss_lost) {
                    continue;
                }
            }
            c.r = g::mat_mul(wh.r, j.body_from_model.r);
            c.p = wh.apply(j.body_from_model.t);
            c.v = {0.0, 0.0, 0.0};
            const std::array<double, 3> keep_bg{at(c.P, 9, 9), at(c.P, 10, 10), at(c.P, 11, 11)};
            const std::array<double, 3> keep_ba{at(c.P, 12, 12), at(c.P, 13, 13), at(c.P, 14, 14)};
            c.P = MatN{};
            for (std::size_t i = 0; i < 3; i++) {
                at(c.P, i, i) = std::pow(rf::to_radians(6.0), 2);
                at(c.P, 3 + i, 3 + i) = 0.005 * 0.005;
                at(c.P, 6 + i, 6 + i) = 0.3 * 0.3;
                at(c.P, 9 + i, 9 + i) = c.has_state ? keep_bg[i] : 0.01 * 0.01;
                at(c.P, 12 + i, 12 + i) = c.has_state ? keep_ba[i] : 0.3 * 0.3;
            }
            if (c.up_n < 20 && im->up_ok) {
                // gravity from this first fit and the accelerometer (learned better from the next fits)
                c.up_w = g::normalize(g::mat_vec(g::mat_mul(c.r, device_from_aim), im->up_aim));
            }
            c.has_state = true;
            c.t = frame_ns;
            c.last_update = frame_ns;
            c.misses = 0;
            c.streak = 1;
            c.status = known_orient ? Status::Tracking : Status::Tentative;
            if (std::getenv("CT_IMUTRACK_DEBUG") != nullptr) {
                std::fprintf(stderr, "itk %zu slot %zu ACQ at (%.3f %.3f %.3f) leds %zu err %.1f\n",
                             static_cast<std::size_t>(frame_no_), s, j.body_from_model.t[0], j.body_from_model.t[1],
                             j.body_from_model.t[2], j.total(), best[s]->err);
            }
            counts.acquisitions += 1;
            Found f{s, j, std::nullopt, 0.0, 0};
            if (c.status == Status::Tracking) {
                publish(s, c, std::move(f), false);
            }
        }
    }
    return out;
}

} // namespace pibridge::ctrl::track
