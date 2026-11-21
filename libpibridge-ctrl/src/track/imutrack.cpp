// SPDX-License-Identifier: MIT
// The IMU-led controller tracker (imutrack.h).
#include "pibridge/ctrl/track/imutrack.h"
#include "pibridge/ctrl/track/trackutil.h"

#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <string_view>

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

} // namespace

ImuTracker::ImuTracker(ImuTrackCfg c, TrackCfg fallback) : cfg(c), fallback_(fallback) {}

namespace {
/// How long after its last update a lost controller's gyro-carried orientation still constrains its search (CT_KNOWN_S,
/// s; the bench forces cold acquisitions with a short one).
std::uint64_t known_ns() {
    static const std::uint64_t v = [] {
        const char* e = std::getenv("CT_KNOWN_S");
        return static_cast<std::uint64_t>((e != nullptr ? std::atof(e) : 10.0) * 1e9);
    }();
    return v;
}
} // namespace

void ImuTracker::imu_push(std::size_t slot, const ImuIn& s_in) {
    static const double shift_s = [this] {
        const char* e = std::getenv("CT_IMU_SHIFT_MS");
        return e != nullptr ? std::atof(e) * 1e-3 : cfg.imu_shift_s;
    }();
    ImuIn s = s_in;
    if (shift_s != 0.0) {
        const std::int64_t sh = static_cast<std::int64_t>(shift_s * 1e9);
        s.t = static_cast<std::uint64_t>(std::max<std::int64_t>(0, static_cast<std::int64_t>(s.t) + sh));
    }
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
    // PiBridge: the accelerometer's scale and axis errors, and the orientation error that rotates it into the world,
    // grow with the acceleration: a Beat Saber swing (50-100 m/s^2) is off by several m/s^2, which the fixed sigma_a
    // made a gate reject of the right fit (CT_SIGMA_A_REL, fraction of |a|)
    static const double rel = [] {
        const char* e = std::getenv("CT_SIGMA_A_REL");
        // off: on beat1 (Beat Saber, Pimax as truth) 0.1 tracked the left hand 95.6 % instead of 99.2 %
        return e != nullptr ? std::atof(e) : 0.0;
    }();
    const double sa = cfg.sigma_a + rel * g::norm(a);
    const double sa2 = sa * sa * dt;
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
                if (twin_[slot]) {
                    const g::Vec3 fm = g::mat_vec(device_from_aim, s.f_aim);
                    const g::Mat3 rt = g::mat_mul(c.r, g::transpose(twin_[slot]->t.r));
                    auto& fr = force_[slot];
                    fr.push_back(ForceRow{to, g::mat_vec(c.r, fm), g::mat_vec(rt, fm)});
                    while (!fr.empty() && fr.front().t + 1'500'000'000ull < to) {
                        fr.pop_front();
                    }
                }
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

namespace {

/// A velocity at t from the optical positions within +-80 ms (least squares line); nullopt with fewer than 3.
std::optional<g::Vec3> velocity_at(const std::deque<std::pair<std::uint64_t, g::Vec3>>& o, std::uint64_t t) {
    double st = 0, stt = 0, n = 0;
    g::Vec3 sp{0, 0, 0}, stp{0, 0, 0};
    for (const auto& [ti, p] : o) {
        const double dt = (static_cast<double>(ti) - static_cast<double>(t)) * 1e-9;
        if (std::abs(dt) > 0.08) {
            continue;
        }
        n += 1;
        st += dt;
        stt += dt * dt;
        sp = g::add(sp, p);
        stp = g::add(stp, g::scale(p, dt));
    }
    const double den = n * stt - st * st;
    if (n < 3 || den < 1e-9) {
        return std::nullopt;
    }
    return g::scale(g::sub(g::scale(stp, n), g::scale(sp, st)), 1.0 / den);
}

} // namespace

std::optional<g::Pose> ImuTracker::p1p_hypothesis(const Rig& rig, const LedModel& model, const std::vector<CamObs>& obs,
                                                  const std::vector<std::vector<bool>>& used, double cos_face,
                                                  const g::Pose& pred, double radius_px) const {
    static const bool on = [] {
        const char* e = std::getenv("CT_P1P");
        return !(e != nullptr && std::string_view(e) == "0");
    }();
    static const double max_shift = [] {
        const char* e = std::getenv("CT_P1P_M");
        // Pimax's 5 cm; ctrl-imu-take2: misses 65 -> 45, jumps > 8 cm 7 -> 1, worst frames 2.5 -> 0.7 ms
        return e != nullptr ? std::atof(e) : 0.05;
    }();
    if (!on) {
        return std::nullopt;
    }
    constexpr double kTight = 4.0; // px: the window a hypothesis is scored in
    struct Pair {
        double d;
        std::size_t cam, led, blob;
    };
    std::vector<Pair> pairs;
    for (std::size_t ci = 0; ci < rig.cams.size() && ci < obs.size(); ci++) {
        const g::Pose cfm = rig.cams[ci].cam_from_body.compose(pred);
        for (std::size_t l = 0; l < model.leds.size(); l++) {
            if (!facing(cfm, model.leds[l], cos_face)) {
                continue;
            }
            const auto uv = rig.cams[ci].intr.project(cfm.apply(model.leds[l].p));
            if (!uv) {
                continue;
            }
            for (std::size_t b = 0; b < obs[ci].px.size(); b++) {
                if (used[ci][b] || !obs[ci].bearing[b]) {
                    continue;
                }
                const double du = uv->first - obs[ci].px[b].first, dv = uv->second - obs[ci].px[b].second;
                const double d = std::sqrt(du * du + dv * dv);
                if (d <= radius_px && d > kTight) {
                    pairs.push_back({d, ci, l, b});
                }
            }
        }
    }
    if (pairs.empty()) {
        return std::nullopt;
    }
    std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) { return a.d < b.d; });
    if (pairs.size() > 30) {
        pairs.resize(30);
    }
    const std::size_t base = count(match_all(pred, rig, model, obs, used, cos_face, kTight));
    std::optional<g::Pose> best;
    std::size_t best_n = base;
    double best_shift = 1e9;
    for (const Pair& p : pairs) {
        const Cam& cam = rig.cams[p.cam];
        const g::Pose cfm = cam.cam_from_body.compose(pred);
        const g::Vec3 led_c = cfm.apply(model.leds[p.led].p);
        const g::Vec3 ray = *obs[p.cam].bearing[p.blob];
        const double s = g::dot(ray, led_c);
        if (s <= 0.02) {
            continue;
        }
        const g::Vec3 shift_c = g::sub(g::scale(ray, s), led_c);
        if (g::norm(shift_c) > max_shift) {
            continue;
        }
        g::Pose h = pred;
        h.t = g::add(pred.t, g::mat_vec(g::transpose(cam.cam_from_body.r), shift_c));
        const std::size_t n = count(match_all(h, rig, model, obs, used, cos_face, kTight));
        const double sh = g::norm(shift_c);
        if (n > best_n || (best && n == best_n && sh < best_shift)) {
            best = h;
            best_n = n;
            best_shift = sh;
        }
    }
    if (!best || best_n < 2) {
        return std::nullopt;
    }
    return best;
}

double ImuTracker::hyp_err(std::size_t slot, const Hyp& h, bool& still) const {
    still = false;
    const auto& d = imu_[slot];
    const std::size_t n = h.f.size();
    if (n < 3 || d.empty()) {
        return -1.0;
    }
    const g::Vec3 up = c_[slot].up_w;
    double sum = 0.0;
    std::size_t k_n = 0;
    double tilt_max = 0.0;
    bool all_up = true;
    const std::size_t from = n > 6 ? n - 6 : 0;
    for (std::size_t k = from; k + 2 < n; k++) {
        const HypFrame& f0 = h.f[k];
        const HypFrame& f1 = h.f[k + 1];
        const HypFrame& f2 = h.f[k + 2];
        const double dt0 = static_cast<double>(f1.t - f0.t) * 1e-9;
        const double dt1 = static_cast<double>(f2.t - f1.t) * 1e-9;
        if (dt0 <= 1e-3 || dt1 <= 1e-3) {
            continue;
        }
        const g::Vec3 va = g::scale(g::sub(f1.p, f0.p), 1.0 / dt0);
        const g::Vec3 vb = g::scale(g::sub(f2.p, f1.p), 1.0 / dt1);
        const std::uint64_t ta = f0.t + (f1.t - f0.t) / 2;
        const std::uint64_t tb = f1.t + (f2.t - f1.t) / 2;
        // the specific force in the world under this hypothesis' orientation, carried by the gyro from the middle frame
        g::Vec3 dv{0.0, 0.0, 0.0};
        double covered = 0.0;
        std::uint64_t prev = 0;
        for (const ImuIn& s : d) {
            if (s.t <= ta) {
                prev = s.t;
                continue;
            }
            if (prev == 0) {
                prev = s.t;
                continue;
            }
            const std::uint64_t a = std::max(prev, ta);
            const std::uint64_t b = std::min(s.t, tb);
            prev = s.t;
            if (b > a) {
                const double dt = static_cast<double>(b - a) * 1e-9;
                const g::Mat3 da = g::mat_mul(g::transpose(f1.qg), s.qg);
                const g::Mat3 r = g::mat_mul(f1.r, g::mat_mul(g::mat_mul(device_from_aim, da), g::transpose(device_from_aim)));
                const g::Vec3 fw = g::mat_vec(r, g::mat_vec(device_from_aim, s.f_aim));
                dv = g::add(dv, g::scale(g::sub(fw, g::scale(up, G)), dt));
                covered += dt;
            }
            if (s.t >= tb) {
                break;
            }
        }
        const double span = static_cast<double>(tb - ta) * 1e-9;
        if (covered < 0.8 * span) {
            continue;
        }
        sum += g::norm(g::sub(g::sub(vb, va), dv));
        k_n += 1;
    }
    for (std::size_t k = from; k < n; k++) {
        const ImuIn* im = imu_at(slot, h.f[k].t);
        if (im == nullptr || !im->up_ok || im->still_s < 0.05) {
            all_up = false;
            continue;
        }
        const g::Vec3 u = g::mat_vec(g::mat_mul(h.f[k].r, device_from_aim), im->up_aim);
        tilt_max = std::max(tilt_max, g::angle_deg(u, up));
    }
    if (k_n == 0) {
        return -1.0;
    }
    double e = sum / static_cast<double>(k_n);
    if (all_up) {
        // still: the accelerometer's gravity is the sharper test (Pimax: within 0.3 rad of up in every frame); as m/s
        // over a frame pair, 20 deg ~ the motion threshold
        still = true;
        e = std::max(e, tilt_max / 20.0 * 0.5);
    }
    return e;
}

std::optional<std::pair<ImuTracker::HypFrame, g::Vec3>> ImuTracker::hyp_step(std::size_t slot, std::uint64_t frame_ns,
                                                                            const std::vector<HypFrame>& cands) {
    static const double thr = [] {
        const char* e = std::getenv("CT_HYP_M");
        return e != nullptr ? std::atof(e) : 0.5;
    }();
    static const double gyro_rad = [] {
        const char* e = std::getenv("CT_HYP_GYRO");
        return e != nullptr ? std::atof(e) : 0.3;
    }();
    static const bool dbg = std::getenv("CT_HYP_DEBUG") != nullptr;
    // frames in a row without an agreeing candidate that drop a hypothesis (Pimax: 2; ctrl-imu-take2 cold masks with 4:
    // relock p95 960 -> 560 ms, no wrong relocks: the search does not find the controller every frame)
    static const std::uint32_t hyp_miss = [] {
        const char* e = std::getenv("CT_HYP_MISS");
        return static_cast<std::uint32_t>(e != nullptr ? std::atoi(e) : 4);
    }();
    auto& hs = hyps_[slot];
    // stale hypotheses (the controller was not searched for; a parked one is searched every 5th frame)
    std::erase_if(hs, [&](const Hyp& h) { return h.f.empty() || h.f.back().t + 500'000'000ull < frame_ns; });
    const ImuIn* im = imu_at(slot, frame_ns);
    if (im == nullptr) {
        return std::nullopt;
    }
    std::vector<bool> taken(cands.size(), false);
    for (Hyp& h : hs) {
        const HypFrame& l = h.f.back();
        if (l.t >= frame_ns) {
            continue;
        }
        const g::Mat3 da = g::mat_mul(g::transpose(l.qg), im->qg);
        const g::Mat3 rp = g::mat_mul(l.r, g::mat_mul(g::mat_mul(device_from_aim, da), g::transpose(device_from_aim)));
        const double dt = static_cast<double>(frame_ns - l.t) * 1e-9;
        std::optional<std::size_t> best;
        double be = 1e9;
        for (std::size_t k = 0; k < cands.size(); k++) {
            const double e = rf::to_radians(angle_deg(rp, cands[k].r));
            if (e < gyro_rad && g::norm(g::sub(cands[k].p, l.p)) < 0.15 + 4.0 * dt && e < be) {
                be = e;
                best = k;
            }
        }
        if (best) {
            h.f.push_back(cands[*best]);
            if (h.f.size() > 12) {
                h.f.erase(h.f.begin());
            }
            h.misses = 0;
            taken[*best] = true;
        } else if (++h.misses >= hyp_miss) {
            h.f.clear(); // erased below
            counts.hyp_gyro_drops += 1;
        }
    }
    std::erase_if(hs, [](const Hyp& h) { return h.f.empty(); });
    for (std::size_t k = 0; k < cands.size(); k++) {
        if (!taken[k] && hs.size() < 16) {
            hs.push_back(Hyp{{cands[k]}, 0});
            counts.hyp_started += 1;
        }
    }
    // the tests: the hypotheses extended this frame with 3+ frames
    struct Score {
        std::size_t i;
        double e;
        bool still;
    };
    std::vector<Score> sc;
    for (std::size_t i = 0; i < hs.size(); i++) {
        if (hs[i].f.back().t != frame_ns || hs[i].f.size() < 3) {
            continue;
        }
        bool still = false;
        const double e = hyp_err(slot, hs[i], still);
        if (e >= 0.0) {
            sc.push_back({i, e, still});
        }
    }
    std::sort(sc.begin(), sc.end(), [](const Score& a, const Score& b) { return a.e < b.e; });
    if (dbg) {
        for (const Score& x : sc) {
            const HypFrame& f = hs[x.i].f.back();
            std::fprintf(stderr, "hyp slot %zu t %.3f frames %zu err %.2f still %d at (%.3f %.3f %.3f) leds %zu\n", slot,
                         static_cast<double>(frame_ns) * 1e-9, hs[x.i].f.size(), x.e, x.still ? 1 : 0, f.p[0], f.p[1],
                         f.p[2], f.j.total());
        }
    }
    if (sc.empty()) {
        return std::nullopt;
    }
    const Score& b = sc.front();
    if (b.e > thr) {
        counts.hyp_motion_fail += 1;
        return std::nullopt;
    }
    // a rival on the same blobs (the twin) that explains the IMU nearly as well: wait for more motion, up to 12 frames
    const HypFrame& bf = hs[b.i].f.back();
    for (std::size_t k = 1; k < sc.size(); k++) {
        const HypFrame& of = hs[sc[k].i].f.back();
        if (g::norm(g::sub(of.p, bf.p)) < 0.08 && sc[k].e < 1.6 * b.e + 0.05 && hs[b.i].f.size() < 12) {
            counts.hyp_ambiguous += 1;
            return std::nullopt;
        }
    }
    const Hyp& h = hs[b.i];
    // the hand: the turns between the hypothesis' fits against each controller's gyro (magnitudes: frame-free). When the
    // other controller's gyro explains them better, these are its LEDs (or the hands move alike: wait)
    if (imu_live(1 - slot, frame_ns)) {
        double own = 0.0, oth = 0.0, turn = 0.0;
        std::size_t n = 0;
        for (std::size_t k = 0; k + 1 < h.f.size(); k++) {
            const ImuIn* a = imu_at(1 - slot, h.f[k].t);
            const ImuIn* b2 = imu_at(1 - slot, h.f[k + 1].t);
            if (a == nullptr || b2 == nullptr) {
                continue;
            }
            const double opt = g::norm(rotvec(g::mat_mul(g::transpose(h.f[k].r), h.f[k + 1].r)));
            const double go = g::norm(rotvec(g::mat_mul(g::transpose(h.f[k].qg), h.f[k + 1].qg)));
            const double gt = g::norm(rotvec(g::mat_mul(g::transpose(a->qg), b2->qg)));
            own += std::abs(opt - go);
            oth += std::abs(opt - gt);
            turn += opt;
            n += 1;
        }
        static const bool hand_dbg = std::getenv("CT_HYP_DEBUG") != nullptr;
        if (hand_dbg && n > 0) {
            std::fprintf(stderr, "hyp slot %zu hand test: turn %.3f own %.3f other %.3f rad\n", slot, turn, own, oth);
        }
        // only a clear case: the own gyro off by more than the fits' noise (~1 deg a pair) and the other's much closer
        if (n > 0 && own / static_cast<double>(n) > 0.03 && oth < 0.6 * own) {
            counts.hyp_other_hand += 1;
            return std::nullopt;
        }
    }
    const HypFrame& f1 = h.f[h.f.size() - 2];
    const double dt = static_cast<double>(bf.t - f1.t) * 1e-9;
    const g::Vec3 v = dt > 1e-3 ? g::scale(g::sub(bf.p, f1.p), 1.0 / dt) : g::Vec3{0.0, 0.0, 0.0};
    std::pair<HypFrame, g::Vec3> out{bf, v};
    counts.hyp_accepted += 1;
    if (dbg) {
        const Ctl& oc = c_[1 - slot];
        std::fprintf(stderr, "hyp slot %zu ACCEPT t %.3f frames %zu err %.2f still %d leds %zu | other: status %d, %.0f mm away, "
                             "updated %.0f ms ago | rivals %zu\n",
                     slot, static_cast<double>(frame_ns) * 1e-9, h.f.size(), b.e, b.still ? 1 : 0, bf.j.total(),
                     static_cast<int>(oc.status), g::norm(g::sub(oc.p, bf.p)) * 1000.0,
                     static_cast<double>(sat_sub(frame_ns, oc.last_update)) * 1e-6, sc.size() - 1);
    }
    hs.clear();
    return out;
}

bool ImuTracker::led_updates(const Rig& rig, const LedModel& model, const std::vector<CamObs>& obs, std::size_t slot,
                             std::uint64_t frame_ns, const std::vector<Matches>& m, std::vector<std::vector<bool>>& used,
                             std::vector<Fix>& out) {
    static const bool on = [] {
        const char* e = std::getenv("CT_LED_UPDATE");
        return !(e != nullptr && std::string_view(e) == "0");
    }();
    std::size_t avail = 0;
    for (const auto& mc : m) {
        avail += mc.size();
    }
    static const std::size_t min_avail = [] {
        const char* e = std::getenv("CT_LED_MIN");
        return e != nullptr ? static_cast<std::size_t>(std::atoi(e)) : std::size_t{2};
    }();
    if (!on || avail < min_avail) {
        return false;
    }
    Ctl& c = c_[slot];
    const ImuIn* im = imu_at(slot, frame_ns);
    const g::Pose wh = im != nullptr ? im->world_from_head : g::Pose{};
    const g::Pose hw = wh.inverse();
    // the angular noise of one blob: ~1.5 px at 230 px/rad, more at speed (blur) as the pose fit's tolerance
    const double sig = (1.5 + 0.5 * g::norm(c.v)) / 230.0;
    std::size_t used_n = 0;
    double sum2 = 0.0;
    Matches taken_all;
    std::vector<Matches> taken(m.size());
    for (std::size_t ci = 0; ci < m.size() && ci < obs.size(); ci++) {
        const g::Pose cw = rig.cams[ci].cam_from_body.compose(hw); // cam_from_world
        for (const auto& [l, b] : m[ci]) {
            if (b >= obs[ci].bearing.size() || !obs[ci].bearing[b]) {
                continue;
            }
            const g::Vec3 bo = *obs[ci].bearing[b];
            const g::Vec3 rx = g::mat_vec(c.r, model.leds[l].p);
            const g::Vec3 pc = cw.apply(g::add(c.p, rx));
            const double d = g::norm(pc);
            if (pc[2] <= 0.0 || d < 1e-3) {
                continue;
            }
            const g::Vec3 bp = g::scale(pc, 1.0 / d);
            const g::Vec3 h0 = std::abs(bp[0]) < 0.9 ? g::Vec3{1, 0, 0} : g::Vec3{0, 1, 0};
            const g::Vec3 u = g::normalize(g::cross(h0, bp));
            const g::Vec3 w = g::cross(bp, u);
            const double z[2] = {g::dot(u, bo), g::dot(w, bo)};
            // H = (1/d) [u; w] R_cw [ -[R x]x | I | 0 ... ]: the rotation (left perturbation) and the position
            double J[2][3];
            for (int k = 0; k < 3; k++) {
                const g::Vec3 col{cw.r[0][k], cw.r[1][k], cw.r[2][k]};
                J[0][k] = g::dot(u, col) / d;
                J[1][k] = g::dot(w, col) / d;
            }
            double H[2][N] = {};
            for (int r = 0; r < 2; r++) {
                // the LED moves by d(R x) = -[R x]x dtheta (left perturbation): a row a^T of J times -[rx]x is rx x a
                const g::Vec3 arow{J[r][0], J[r][1], J[r][2]};
                const g::Vec3 hr = g::cross(rx, arow);
                for (int k = 0; k < 3; k++) {
                    H[r][k] = hr[k];
                    H[r][3 + k] = arow[k];
                }
            }
            // S = H P H^T + sig^2 I (2 x 2)
            double PH[N][2] = {};
            for (std::size_t i = 0; i < N; i++) {
                for (int r = 0; r < 2; r++) {
                    double sacc = 0.0;
                    for (std::size_t k = 0; k < 6; k++) {
                        sacc += at(c.P, i, k) * H[r][k];
                    }
                    PH[i][r] = sacc;
                }
            }
            double S[2][2];
            for (int r = 0; r < 2; r++) {
                for (int q = 0; q < 2; q++) {
                    double sacc = 0.0;
                    for (std::size_t k = 0; k < 6; k++) {
                        sacc += H[r][k] * PH[k][q];
                    }
                    S[r][q] = sacc + (r == q ? sig * sig : 0.0);
                }
            }
            const double det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
            if (det <= 1e-30) {
                continue;
            }
            const double Si[2][2] = {{S[1][1] / det, -S[0][1] / det}, {-S[1][0] / det, S[0][0] / det}};
            const double d2 = z[0] * (Si[0][0] * z[0] + Si[0][1] * z[1]) + z[1] * (Si[1][0] * z[0] + Si[1][1] * z[1]);
            if (d2 > 9.21) {
                continue; // 99 % for 2 dof: this blob is not that LED
            }
            double dx[N] = {};
            double K[N][2];
            for (std::size_t i = 0; i < N; i++) {
                K[i][0] = PH[i][0] * Si[0][0] + PH[i][1] * Si[1][0];
                K[i][1] = PH[i][0] * Si[0][1] + PH[i][1] * Si[1][1];
                dx[i] = K[i][0] * z[0] + K[i][1] * z[1];
            }
            c.r = g::mat_mul(g::rodrigues({dx[0], dx[1], dx[2]}), c.r);
            for (std::size_t i = 0; i < 3; i++) {
                c.p[i] += dx[3 + i];
                c.v[i] += dx[6 + i];
                c.bg[i] += dx[9 + i];
                c.ba[i] += dx[12 + i];
            }
            MatN P = c.P;
            for (std::size_t i = 0; i < N; i++) {
                for (std::size_t j = 0; j < N; j++) {
                    P[i * N + j] -= K[i][0] * (H[0][0] * at(c.P, 0, j) + H[0][1] * at(c.P, 1, j) + H[0][2] * at(c.P, 2, j) +
                                               H[0][3] * at(c.P, 3, j) + H[0][4] * at(c.P, 4, j) + H[0][5] * at(c.P, 5, j)) +
                                    K[i][1] * (H[1][0] * at(c.P, 0, j) + H[1][1] * at(c.P, 1, j) + H[1][2] * at(c.P, 2, j) +
                                               H[1][3] * at(c.P, 3, j) + H[1][4] * at(c.P, 4, j) + H[1][5] * at(c.P, 5, j));
                }
            }
            c.P = P;
            used_n++;
            sum2 += (z[0] * z[0] + z[1] * z[1]);
            taken[ci].emplace_back(l, b);
        }
    }
    static const std::size_t min_n = [] {
        const char* e = std::getenv("CT_LED_MIN");
        return e != nullptr ? static_cast<std::size_t>(std::atoi(e)) : std::size_t{2};
    }();
    if (used_n < min_n) {
        // not enough to keep it tracked: the corrections stay (they agreed with the prediction), the frame is a miss
        return false;
    }
    counts.led_updates += 1;
    counts.led_points += used_n;
    c.last_update = frame_ns;
                        c.p_fixed = c.p;
    c.misses = 0;
    // the filter's pose as a few-LED fix (the fuser keeps showing the hand instead of holding it)
    const g::Pose body{g::mat_mul(hw.r, c.r), hw.apply(c.p)};
    JointSolve j{body, std::move(taken), std::sqrt(sum2 / static_cast<double>(used_n)) * 230.0};
    claim(used, j);
    Fix fx{slot, Found{slot, std::move(j), std::nullopt, 0.0, 0}, true, body, g::mat_vec(hw.r, c.v)};
    fallback_.stats.hand[slot] += 1;
    fallback_.stats.few[slot] += 1;
    last_fix_ns_ = frame_ns;
    out.push_back(std::move(fx));
    return true;
}

void ImuTracker::twin_reset(std::size_t slot) {
    orient_bad_[slot] = 0;
    force_[slot].clear();
    opt_[slot].clear();
    twin_run_[slot] = 0;
}

void ImuTracker::twin_check(std::size_t slot, std::uint64_t frame_ns) {
    auto& o = opt_[slot];
    while (!o.empty() && o.front().first + 1'500'000'000ull < frame_ns) {
        o.pop_front();
    }
    if (!twin_[slot] || frame_ns < twin_last_[slot] + 100'000'000ull) {
        return;
    }
    // the window: velocity at t1 = 80 ms ago (the fits on both sides) and at t0 = 0.4 s before
    const std::uint64_t t1 = frame_ns - 80'000'000ull, t0 = t1 - 400'000'000ull;
    const auto v0 = velocity_at(o, t0);
    const auto v1 = velocity_at(o, t1);
    if (!v0 || !v1) {
        return;
    }
    // the accelerometer's change of velocity under each orientation (gravity: the world's up, the head pose is level)
    g::Vec3 ic{0, 0, 0}, it{0, 0, 0};
    std::uint64_t prev = 0, covered = 0;
    for (const ForceRow& f : force_[slot]) {
        if (f.t <= t0) {
            prev = f.t;
            continue;
        }
        if (f.t > t1) {
            break;
        }
        if (prev == 0) {
            prev = f.t;
            continue;
        }
        const double dt = static_cast<double>(f.t - prev) * 1e-9;
        covered += f.t - prev;
        prev = f.t;
        const g::Vec3 gw{0.0, G, 0.0};
        ic = g::add(ic, g::scale(g::sub(f.cur, gw), dt));
        it = g::add(it, g::scale(g::sub(f.twin, gw), dt));
    }
    if (covered < 300'000'000ull) {
        return; // the IMU does not cover the window
    }
    twin_last_[slot] = frame_ns;
    counts.twin_checks += 1;
    const g::Vec3 dv = g::sub(*v1, *v0);
    const double ec = g::norm(g::sub(dv, ic)), et = g::norm(g::sub(dv, it));
    counts.accel_err_sum += ec;
    counts.accel_err_n += 1;
    // neither orientation explains the motion (right: ~0.25 m/s, at most ~1.5 on hard swings): the track is wrong in
    // some other way; drop it so the controller is found again under the gravity test
    orient_bad_[slot] = (ec > 2.5 && et > 2.5) ? orient_bad_[slot] + 1 : 0;
    if (orient_bad_[slot] >= 5) {
        Ctl& c = c_[slot];
        c.has_state = false;
        c.status = Status::Lost;
        c.misses = cfg.miss_lost;
        orient_bad_[slot] = 0;
        counts.orient_resets += 1;
        twin_reset(slot);
        return;
    }
    if (et < 0.5 * ec && ec - et > 1.0) {
        counts.twin_votes += 1;
        twin_run_[slot] += 1;
    } else if (ec < 0.5 * et) {
        twin_run_[slot] = 0;
    }
    if (std::getenv("CT_TWIN_DEBUG") != nullptr) {
        std::fprintf(stderr, "twin %zu: dv %.2f cur %.2f twin %.2f run %u\n", slot, g::norm(dv), ec, et, twin_run_[slot]);
    }
    if (twin_run_[slot] < 3) {
        return;
    }
    // switch to the twin: W' = W o T^-1 (the same LEDs, the other identity); the model origin moves with it
    Ctl& c = c_[slot];
    const g::Pose& t = twin_[slot]->t;
    const g::Mat3 rti = g::transpose(t.r);
    c.p = g::add(c.p, g::mat_vec(c.r, g::scale(g::mat_vec(rti, t.t), -1.0)));
    c.r = g::mat_mul(c.r, rti);
    c.up_n = 0; // the learnt up came from the wrong orientation
    counts.twin_flips += 1;
    twin_reset(slot);
}

bool ImuTracker::known(std::size_t s, std::uint64_t t) const {
    const Ctl& c = c_[s];
    if (!c.has_state || sat_sub(t, c.last_update) >= known_ns()) {
        return false;
    }
    // Pimax: a lost controller over 1 m from the head is searched for from scratch (CT_KNOWN_FAR=0: by its orientation)
    static const bool far_rule = [] {
        const char* e = std::getenv("CT_KNOWN_FAR");
        // on: without it 2 of 17 masked relocks on ctrl-imu-take2 failed (the filter's orientation drifts with its position)
        return !(e != nullptr && std::string_view(e) == "0");
    }();
    const ImuIn* im = imu_at(s, t);
    // from where the last update put it (the lost state's position is IMU-only and runs off within half a second on a
    // swing: live, a crossing then cost a cold search of several seconds)
    return !far_rule || c.status == Status::Tracking || im == nullptr ||
           g::norm(g::sub(c.p_fixed, im->world_from_head.t)) <= 1.0;
}

std::vector<Fix> ImuTracker::step(const Rig& rig, const LedFrame& lf, std::uint64_t frame_ns) {
    frame_no_ += 1;
    if (!twin_found_) {
        twin_found_ = true;
        for (std::size_t m = 0; m < 2 && m < rig.models.size(); m++) {
            if (const auto tw = model_twin(rig.models[m])) {
                twin_[m] = Twin{tw->first, tw->second};
                const g::Mat3& r = tw->first.r;
                std::fprintf(stderr, "imutrack: %s model's twin: turned %.0f deg, LEDs within %.1f mm rms (checked by the "
                                     "accelerometer)\n", rig.models[m].hand.c_str(),
                             rf::to_degrees(std::acos(std::clamp((r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0, -1.0, 1.0))),
                             tw->second * 1000.0);
            }
        }
    }
    fallback_.device_from_aim = device_from_aim;
    // no IMU for either controller: the per-frame tracker
    if ((!imu_live(0, frame_ns) && !imu_live(1, frame_ns)) || (cfg.use_accel && sat_sub(frame_ns, head_seen_) > 1'000'000'000)) {
        counts.fallback_frames += 1;
        fallback_.wanted = wanted;
        return fallback_.step(rig, lf, frame_ns);
    }
    Stats& st = fallback_.stats;
    st.led_frames += 1;
    // with a head pose the world is level (y up): gravity is known, nothing to learn. The learnt up (for takes without
    // a head pose) followed whatever orientation the fits had, a wrong one included, and then kept the tracker wrong.
    static const bool learn_up = [] {
        const char* e = std::getenv("CT_LEARN_UP");
        return e != nullptr && std::string_view(e) == "1";
    }();
    if (!learn_up && sat_sub(frame_ns, head_seen_) < 1'000'000'000) {
        for (Ctl& c : c_) {
            c.up_w = {0.0, 1.0, 0.0};
            c.up_n = std::max<std::uint32_t>(c.up_n, 20);
        }
    }
    for (std::size_t s = 0; s < 2; s++) {
        propagate(s, frame_ns);
    }
    std::vector<CamObs> obs;
    for (std::size_t ci = 0; ci < rig.cams.size(); ci++) { // one per rig camera (empty when the frame has fewer)
        obs.push_back(CamObs::make(rig.cams[ci], ci < lf.px.size() ? lf.px[ci] : std::vector<std::pair<double, double>>{}));
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
        // out of reach (Pimax: over 1 m from the head is lost): a diverged filter, not a hand
        if (g::norm(hw.apply(c.p)) > 1.2) {
            counts.far_drops += 1;
            c.status = Status::Lost;
            c.misses = cfg.miss_lost;
            return;
        }
        Fix fx{s, std::move(f), few, g::Pose{g::mat_mul(hw.r, c.r), hw.apply(c.p)}, g::mat_vec(hw.r, c.v)};
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
    // a controller whose pose fit failed tries its few-LED updates after both controllers' fits claimed their blobs
    // (so the other controller's LEDs are not taken for its own)
    struct Deferred {
        std::size_t s;
        g::Pose pred;
        double win;
    };
    std::vector<Deferred> deferred;
    // contested blobs (Pimax's DoubleCtrlTracking): with both controllers predicted, a blob near the LEDs of both goes to
    // the one whose projected LED is nearer; the other matches as if it were taken (CT_CONTEST=0: first come first
    // served, the tighter prediction first)
    static const bool contest = [] {
        const char* e = std::getenv("CT_CONTEST");
        return !(e != nullptr && std::string_view(e) == "0");
    }();
    std::array<std::vector<std::vector<bool>>, 2> theirs;
    {
        std::array<std::optional<g::Pose>, 2> pr;
        for (std::size_t s = 0; s < 2; s++) {
            const Ctl& c = c_[s];
            if (wanted[s] && c.status != Status::Lost && c.has_state) {
                const g::Pose hw = head_at(s).inverse();
                pr[s] = g::Pose{g::mat_mul(hw.r, c.r), hw.apply(c.p)};
            }
        }
        for (std::size_t s = 0; s < 2; s++) {
            theirs[s].resize(obs.size());
            for (std::size_t ci = 0; ci < obs.size(); ci++) {
                theirs[s][ci].assign(obs[ci].px.size(), false);
            }
        }
        if (contest && pr[0] && pr[1]) {
            for (std::size_t ci = 0; ci < obs.size() && ci < rig.cams.size(); ci++) {
                std::array<std::vector<std::pair<double, double>>, 2> uv;
                for (std::size_t s = 0; s < 2; s++) {
                    const g::Pose cfm = rig.cams[ci].cam_from_body.compose(*pr[s]);
                    for (const Led& led : rig.models[s].leds) {
                        if (facing(cfm, led, cos_face)) {
                            if (const auto p = rig.cams[ci].intr.project(cfm.apply(led.p))) {
                                uv[s].push_back(*p);
                            }
                        }
                    }
                }
                if (uv[0].empty() || uv[1].empty()) {
                    continue;
                }
                for (std::size_t b = 0; b < obs[ci].px.size(); b++) {
                    const auto [bu, bv] = obs[ci].px[b];
                    std::array<double, 2> d{1e9, 1e9};
                    for (std::size_t s = 0; s < 2; s++) {
                        for (const auto& [u, v] : uv[s]) {
                            d[s] = std::min(d[s], std::hypot(u - bu, v - bv));
                        }
                    }
                    for (std::size_t s = 0; s < 2; s++) {
                        // the other's when its LED is nearer and close (a blob far from both is nobody's yet)
                        if (d[1 - s] < 20.0 && d[1 - s] + 1.0 < d[s]) {
                            theirs[s][ci][b] = true;
                            counts.contested += d[s] < 20.0 ? 1 : 0;
                        }
                    }
                }
            }
        }
    }
    const auto own_view = [&](std::size_t s) {
        std::vector<std::vector<bool>> u = used;
        for (std::size_t ci = 0; ci < u.size() && ci < theirs[s].size(); ci++) {
            for (std::size_t b = 0; b < u[ci].size() && b < theirs[s][ci].size(); b++) {
                u[ci][b] = u[ci][b] || theirs[s][ci][b];
            }
        }
        return u;
    };
    const auto miss = [&](std::size_t s) {
        Ctl& c = c_[s];
        counts.misses += 1;
        c.misses += 1;
        c.streak = 0;
        if (c.status == Status::Tentative) {
            c.status = Status::Lost;
            counts.tentative_dropped += 1;
        } else if (c.misses >= cfg.miss_lost) {
            c.status = Status::Lost;
        }
    };
    for (const std::size_t s : order) {
        Ctl& c = c_[s];
        if (!wanted[s] || c.status == Status::Lost || !c.has_state) {
            continue;
        }
        st.tries += 1;
        const LedModel& model = rig.models[s];
        const g::Pose wh = head_at(s);
        const g::Pose hw = wh.inverse();
        g::Pose pred{g::mat_mul(hw.r, c.r), hw.apply(c.p)};
        const double sp = std::sqrt(spread(s) / 3.0);
        const double sr = std::sqrt((at(c.P, 0, 0) + at(c.P, 1, 1) + at(c.P, 2, 2)) / 3.0);
        const double gate = std::clamp(3.0 * 230.0 * (sp + 0.07 * sr) / 0.4 + 3.0, cfg.gate_min_px, cfg.gate_max_px);
        const std::optional<std::pair<g::Mat3, double>> prior =
            std::pair{pred.r, std::clamp(1.0 / std::max(sr, 1e-3), 30.0, 1500.0)};
        bool ok = false;
        // a tight window first (the prediction is usually within a few mm), the covariance's window after
        // CT_WIN3=1: then the widest window (a fast swing: the prediction off by more than the covariance says)
        static const bool win3 = [] {
            const char* e = std::getenv("CT_WIN3");
            return e != nullptr && std::string_view(e) == "1";
        }();
        const double win_wide = win3 ? cfg.gate_max_px : gate;
        // the fit's rms allowed 1 px more per m/s of the controller (blur, rolling shutter); CT_RMS_SPEED=K to try others
        static const double rms_speed = [] {
            const char* e = std::getenv("CT_RMS_SPEED");
            // ctrl-imu-take2: 1 px per m/s -38 % misses with no more jumps; 2 doubles the jumps
            return e != nullptr ? std::atof(e) : 1.0;
        }();
        // PiBridge (Pimax's gate by distance: 5 px to 0.4 m, 3 px to 0.7 m, 1.5 px beyond): a controller near the
        // cameras turns the model's mm errors into more pixels; live, held still at ~0.3 m its 18-LED fits failed the
        // fixed 2.5 px and few-LED updates let it wander by centimetres. CT_NEAR_TOL=0: the fixed tolerance.
        static const bool near_tol = [] {
            const char* e = std::getenv("CT_NEAR_TOL");
            return !(e != nullptr && std::string_view(e) == "0");
        }();
        const double dist = g::norm(pred.t);
        const double near_k = near_tol ? (dist < 0.25 ? 2.4 : dist < 0.4 ? 1.6 : dist < 0.55 ? 1.2 : 1.0) : 1.0;
        const double tol_fit = rig.prm.tol_px * near_k + rms_speed * g::norm(c.v);
        // Pimax's tracking hypotheses (P1P): the gyro's rotation is right on a fast swing, the position is not. Each
        // near (LED, blob) pair moves the predicted position so that LED lands on that blob (the nearest point of its
        // ray, at most 5 cm); the hypothesis matching the most LEDs in a tight window replaces the prediction.
        const std::vector<std::vector<bool>> mine = own_view(s);
        if (const auto hp = p1p_hypothesis(rig, model, obs, mine, cos_face, pred, c.streak < 5 ? 300.0 : 150.0)) {
            pred = *hp;
            counts.p1p_moves += 1;
        }
        std::vector<Matches> last_m; // the widest window's correspondences (the LED updates of a missed frame)
        for (const double win : {std::min(gate, 10.0), gate, win_wide}) {
        if (ok) {
            break;
        }
        std::vector<Matches> m = match_all(pred, rig, model, obs, mine, cos_face, win);
        last_m = m;
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
            std::vector<Matches> m2 = match_all(p1, rig, model, obs, mine, cos_face, rig.prm.tol_px * 2.0);
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
                if (rms > (n2 == 3 ? 1.2 : tol_fit)) {
                    counts.miss_rms += 1;
                }
                if (rms <= (n2 == 3 ? 1.2 : tol_fit)) {
                    const g::Pose meas{g::mat_mul(wh.r, p2.r), wh.apply(p2.t)};
                    // Pimax's fast-stop: enters after 3 frames with the gyro under 0.3 rad/s (over 0.1 s) and the
                    // speed under 0.12 m/s, leaves over 0.5 rad/s or 0.18 m/s; while still, the fixes count CT_QUIET
                    // (3) times less, so their noise does not shake a hand held still (Pimax: keep-static and
                    // zero-velocity factors). Off by default (CT_QUIET=3 to try): on still1 the left hand's tracking
                    // fell 75 -> 61 % (the stiffer filter lags the start of a motion); the fuser smooths a still hand
                    // instead (Config::smooth_still_s)
                    static const double quiet_k = [] {
                        const char* e = std::getenv("CT_QUIET");
                        return e != nullptr ? std::atof(e) : 1.0;
                    }();
                    {
                        double rate = 1e9;
                        if (const ImuIn* a = imu_at(s, frame_ns > 100'000'000 ? frame_ns - 100'000'000 : 0);
                            a != nullptr) {
                            if (const ImuIn* b = imu_at(s, frame_ns); b != nullptr && b->t > a->t) {
                                rate = g::norm(rotvec(g::mat_mul(g::transpose(a->qg), b->qg))) /
                                       (static_cast<double>(b->t - a->t) * 1e-9);
                            }
                        }
                        const double speed = g::norm(c.v);
                        if (rate < 0.3 && speed < 0.12) {
                            c.quiet_run += 1;
                        } else {
                            c.quiet_run = 0;
                        }
                        if (c.quiet && (rate > 0.5 || speed > 0.18)) {
                            c.quiet = false;
                        } else if (!c.quiet && c.quiet_run >= 3) {
                            c.quiet = true;
                        }
                    }
                    const double qk = c.quiet && quiet_k > 1.0 ? quiet_k : 1.0;
                    counts.quiet_updates += qk > 1.0 ? 1 : 0;
                    const double sigp = qk * (n2 >= 8 ? cfg.sigma_p[0] : n2 >= 5 ? cfg.sigma_p[1] : cfg.sigma_p[2]);
                    // CT_SIGR_SCALE: the optical orientation's weight against the gyro (sigma x scale)
                    static const double sigr_scale = [] {
                        const char* e = std::getenv("CT_SIGR_SCALE");
                        return e != nullptr ? std::atof(e) : 1.0;
                    }();
                    const double sigr = rf::to_radians(n2 >= 8 ? 2.5 : 5.0) * sigr_scale * qk;
                    bool upd = update(c, meas, sigp, sigr, free_rot);
                    // a strong fit the filter's gate refuses (a fast swing the IMU prediction missed): Pimax has no
                    // gate after the solve, it trusts the optics. The position and velocity start over from it. Off unless
                    // CT_REANCHOR=1: on ctrl-imu-take2 the refused fits were wrong ones (jumps > 8 cm 1 -> 31)
                    static const bool reanchor = [] {
                        const char* e = std::getenv("CT_REANCHOR");
                        return e != nullptr && std::string_view(e) == "1";
                    }();
                    if (!upd && reanchor && n2 >= 6 && free_rot && angle_deg(p2.r, pred.r) < 30.0) {
                        for (std::size_t i = 3; i < 9; i++) {
                            for (std::size_t k = 0; k < 15; k++) {
                                at(c.P, i, k) = 0.0;
                                at(c.P, k, i) = 0.0;
                            }
                        }
                        for (std::size_t i = 0; i < 3; i++) {
                            at(c.P, 3 + i, 3 + i) = 0.1 * 0.1;
                            at(c.P, 6 + i, 6 + i) = 2.0 * 2.0;
                            at(c.P, i, i) += std::pow(rf::to_radians(10.0), 2);
                        }
                        upd = update(c, meas, sigp, sigr, free_rot);
                        counts.reanchors += upd ? 1 : 0;
                    }
                    if (upd) {
                        ok = true;
                        if (const ImuIn* im = imu_at(s, frame_ns); im != nullptr && im->up_ok && free_rot && n2 >= 8) {
                            const g::Vec3 u = g::mat_vec(g::mat_mul(meas.r, device_from_aim), im->up_aim);
                            const double k = std::max(0.02, 1.0 / static_cast<double>(c.up_n + 1));
                            c.up_w = g::normalize(g::add(g::scale(c.up_w, 1.0 - k), g::scale(u, k)));
                            c.up_n += 1;
                        }
                        counts.updates += 1;
                        c.last_update = frame_ns;
                        c.p_fixed = c.p;
                        c.misses = 0;
                        c.streak += 1;
                        opt_[s].emplace_back(frame_ns, meas.t);
                        twin_check(s, frame_ns);
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
        static const double led_win = [] {
            const char* e = std::getenv("CT_LED_WIN");
            // ctrl-imu-take2: 6 px, at least 2 LEDs: misses 175 -> 65, tracked +2 %, jumps as before
            return e != nullptr ? std::atof(e) : 6.0;
        }();
        if (ok) {
            c.led_run = 0;
        } else if (c.status == Status::Tracking) {
            deferred.push_back({s, pred, std::min(gate, led_win)});
        } else {
            miss(s);
        }
    }
    static const std::uint32_t led_run_max = [] {
        const char* e = std::getenv("CT_LED_RUN");
        return static_cast<std::uint32_t>(e != nullptr ? std::atoi(e) : 15);
    }();
    for (const Deferred& df : deferred) {
        Ctl& c = c_[df.s];
        const LedModel& model = rig.models[df.s];
        if (c.led_run >= led_run_max) {
            counts.led_run_cut += 1;
            miss(df.s);
            continue;
        }
        // not on the other controller's LEDs either (its pose after this frame's update, projected; a few LEDs of the
        // other hand fit any prediction near it)
        std::vector<std::vector<bool>> view = own_view(df.s);
        if (const Ctl& oc = c_[1 - df.s]; oc.status == Status::Tracking && oc.has_state &&
                                       sat_sub(frame_ns, oc.last_update) < 200'000'000ull) {
            const g::Pose hw = head_at(1 - df.s).inverse();
            const g::Pose ob{g::mat_mul(hw.r, oc.r), hw.apply(oc.p)};
            for (std::size_t ci = 0; ci < obs.size() && ci < rig.cams.size(); ci++) {
                const g::Pose cfm = rig.cams[ci].cam_from_body.compose(ob);
                for (const Led& led : rig.models[1 - df.s].leds) {
                    if (!facing(cfm, led, cos_face)) {
                        continue;
                    }
                    const auto uv = rig.cams[ci].intr.project(cfm.apply(led.p));
                    if (!uv) {
                        continue;
                    }
                    for (std::size_t b = 0; b < obs[ci].px.size(); b++) {
                        if (std::hypot(uv->first - obs[ci].px[b].first, uv->second - obs[ci].px[b].second) < 4.0) {
                            view[ci][b] = true;
                        }
                    }
                }
            }
        }
        if (led_updates(rig, model, obs, df.s, frame_ns, match_all(df.pred, rig, model, obs, view, cos_face, df.win),
                        used, out)) {
            c.led_run += 1; // a few LEDs near the prediction corrected the filter: still tracked (a fast swing)
        } else {
            miss(df.s);
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
                  imu_live(s, frame_ns) && im != nullptr && (im->still_s < cfg.imu_parked_s || frame_no_ % 5 == 0);
        // (a parked controller, its gyro quiet for imu_parked_s, is searched for every 5th frame: Pimax keeps finding a
        // controller lying in view, and one held very still is not lost for good)
        if (wanted[s] && c_[s].status == Status::Lost && im != nullptr && im->still_s >= cfg.imu_parked_s) {
            counts.parked[s] += 1;
        }
    }
    const bool idle = out.empty() && sat_sub(frame_ns, last_fix_ns_) > 2'000'000'000;
    if ((need[0] || need[1]) && (!idle || frame_no_ % 3 == 0)) {
        // what each controller's IMU says about its orientation: the acquisition keeps only agreeing hypotheses (the
        // same tests as the candidates' below, applied before the best one is picked)
        std::array<std::optional<OrientPrior>, 2> orient{};
        for (std::size_t s = 0; s < 2; s++) {
            const ImuIn* im = imu_at(s, frame_ns);
            if (!need[s] || im == nullptr) {
                continue;
            }
            const g::Mat3 hw_r = g::transpose(im->world_from_head.r);
            OrientPrior op;
            op.tilt_deg = cfg.acq_tilt_deg;
            op.rot_deg = cfg.acq_rot_deg;
            if (im->up_ok) {
                // the learnt up once there is one; before it the world's (the head pose is gravity-aligned), loosely:
                // enough to tell a fit from its twin turned half a turn (the LED ring is nearly symmetric)
                const bool learnt = c_[s].up_n >= 20;
                op.up = std::pair{g::mat_vec(device_from_aim, im->up_aim),
                                  g::mat_vec(hw_r, learnt ? c_[s].up_w : g::Vec3{0.0, 1.0, 0.0})};
                static const double cold = [this] {
                    const char* e = std::getenv("CT_TILT_COLD");
                    return e != nullptr ? std::atof(e) : cfg.acq_tilt_cold_deg;
                }();
                op.tilt_deg = learnt ? cfg.acq_tilt_deg : cold;
            }
            if (known(s, frame_ns)) {
                op.r = g::mat_mul(hw_r, c_[s].r);
                if (twin_[s]) {
                    op.r_twin = g::mat_mul(hw_r, g::mat_mul(c_[s].r, g::transpose(twin_[s]->t.r)));
                }
            }
            if (op.up || op.r) {
                orient[s] = op;
            }
        }
        const std::vector<Found> got =
            solve_frame(lf, rig.cams, rig.models, rig.triples, rig.prm, nullptr, pre, true, &orient, &need);
        st.searches += 1;
        struct Pick {
            std::size_t cand;
            JointSolve j;
            double err;
        };
        std::array<std::optional<Pick>, 2> best;
        // Pimax's ControllerDetect for a controller with no orientation to go by (CT_HYP=0: the single-frame pick)
        static const bool hyp_on = [] {
            const char* e = std::getenv("CT_HYP");
            return !(e != nullptr && std::string_view(e) == "0");
        }();
        std::array<bool, 2> cold{false, false};
        std::array<std::vector<HypFrame>, 2> cold_c;
        std::array<std::optional<g::Vec3>, 2> hyp_v;
        // a candidate's twin (the LED ring turned ~180 deg onto itself), refit: the IMU must tell them apart
        const auto twin_of = [&](std::size_t s, const JointSolve& j) -> std::optional<JointSolve> {
            if (!twin_[s]) {
                return std::nullopt;
            }
            const LedModel& model = rig.models[s];
            const g::Mat3 rti = g::transpose(twin_[s]->t.r);
            g::Pose p{g::mat_mul(j.body_from_model.r, rti),
                      g::sub(j.body_from_model.t, g::mat_vec(g::mat_mul(j.body_from_model.r, rti), twin_[s]->t.t))};
            for (const double win : {rig.prm.tol_px * 4.0, rig.prm.tol_px * 2.0}) {
                const std::vector<Matches> m = match_all(p, rig, model, obs, used, cos_face, win);
                if (count(m) < 5) {
                    return std::nullopt;
                }
                const auto [p2, rms] = refine(p, obs_list(rig, model, obs, m), true, 20);
                p = p2;
                if (win == rig.prm.tol_px * 2.0) {
                    std::vector<Matches> m2 = match_all(p, rig, model, obs, used, cos_face, rig.prm.tol_px * 2.0);
                    const auto [p3, rms3] = refine(p, obs_list(rig, model, obs, m2), true, 20);
                    if (count(m2) < 5 || rms3 > rig.prm.tol_px) {
                        return std::nullopt;
                    }
                    return JointSolve{p3, std::move(m2), rms3};
                }
            }
            return std::nullopt;
        };
        for (std::size_t s = 0; s < 2; s++) {
            if (!need[s]) {
                continue;
            }
            const ImuIn* im = imu_at(s, frame_ns);
            cold[s] = hyp_on && !(known(s, frame_ns));
            const g::Pose wh = im->world_from_head;
            for (std::size_t k = 0; k < got.size(); k++) {
                const Found& gf = got[k];
                const std::optional<JointSolve> j =
                    gf.model == s ? std::optional<JointSolve>(gf.joint) : gf.alt;
                if (!j || j->total() < 5 || j->rms_px > rig.prm.tol_px) {
                    continue;
                }
                counts.acq_cands[s] += 1;
                // not where the other controller was found this frame: its leftover LEDs fit this hand's (mirror)
                // model too
                bool overlap = false;
                for (const Found& p : pre) {
                    overlap = overlap || (p.model != s && g::norm(g::sub(p.joint.body_from_model.t,
                                                                         j->body_from_model.t)) < cfg.acq_apart_m);
                }
                if (overlap) {
                    counts.acq_refused += 1;
                    counts.acq_overlap += 1;
                    continue;
                }
                // nor on the other controller's LEDs: its state (tracked, maybe not fitted this frame) projected, a
                // third of this fit's blobs within 4 px of them refuses it (Pimax: a tracked controller claims its blobs
                // first, the search gets the rest)
                if (const Ctl& oc = c_[1 - s]; oc.status == Status::Tracking && oc.has_state &&
                                               sat_sub(frame_ns, oc.last_update) < 500'000'000ull) {
                    const g::Pose ob{g::mat_mul(g::transpose(wh.r), oc.r), wh.inverse().apply(oc.p)};
                    const LedModel& om = rig.models[1 - s];
                    std::size_t on = 0;
                    for (std::size_t ci = 0; ci < j->matches.size() && ci < obs.size(); ci++) {
                        const g::Pose cfm = rig.cams[ci].cam_from_body.compose(ob);
                        for (const auto& [l, b] : j->matches[ci]) {
                            (void)l;
                            const auto [bu, bv] = obs[ci].px[b];
                            for (const Led& led : om.leds) {
                                if (!facing(cfm, led, cos_face)) {
                                    continue;
                                }
                                const auto uv = rig.cams[ci].intr.project(cfm.apply(led.p));
                                if (uv && std::hypot(uv->first - bu, uv->second - bv) < 4.0) {
                                    on += 1;
                                    break;
                                }
                            }
                        }
                    }
                    if (3 * on >= j->total() ||
                        g::norm(g::sub(oc.p, wh.apply(j->body_from_model.t))) < cfg.acq_apart_m) {
                        counts.acq_refused += 1;
                        counts.acq_on_other += 1;
                        continue;
                    }
                }
                const g::Mat3 rw = g::mat_mul(wh.r, j->body_from_model.r);
                double err = 0.0;
                bool constrained = false;
                if (im->up_ok && c_[s].up_n >= 20) {
                    const g::Vec3 up = g::mat_vec(g::mat_mul(rw, device_from_aim), im->up_aim);
                    const double tilt = g::angle_deg(up, c_[s].up_w);
                    if (tilt > cfg.acq_tilt_deg) {
                        counts.acq_refused += 1;
                        counts.acq_tilt += 1;
                        continue;
                    }
                    err = tilt;
                    constrained = true;
                }
                if (cold[s]) {
                    const auto add = [&](const JointSolve& js) {
                        const g::Mat3 r = g::mat_mul(wh.r, js.body_from_model.r);
                        const g::Vec3 p = wh.apply(js.body_from_model.t);
                        for (const HypFrame& o : cold_c[s]) {
                            if (g::norm(g::sub(o.p, p)) < 0.03 && angle_deg(o.r, r) < 10.0) {
                                return;
                            }
                        }
                        cold_c[s].push_back(HypFrame{frame_ns, r, p, im->qg, js});
                    };
                    add(*j);
                    if (const auto tw = twin_of(s, *j)) {
                        add(*tw);
                    }
                    continue;
                }
                if (known(s, frame_ns)) {
                    double e = angle_deg(rw, c_[s].r);
                    if (twin_[s]) {
                        // the twin of the gyro's orientation is as good: the gyro cannot tell them apart
                        e = std::min(e, angle_deg(rw, g::mat_mul(c_[s].r, g::transpose(twin_[s]->t.r))));
                    }
                    if (e > cfg.acq_rot_deg) {
                        counts.acq_refused += 1;
                        counts.acq_rot += 1;
                        continue;
                    }
                    // a tracked controller missed by its prediction: also near where it should be
                    const double age = static_cast<double>(sat_sub(frame_ns, c_[s].last_update)) * 1e-9;
                    // (not for a strong fit, nor after 3 misses: the filter's position is what drifted then; live a
                    // swing lost the right hand for 5 s on this test)
                    static const int far_loose = [] {
                        const char* e = std::getenv("CT_FAR_LOOSE");
                        return e != nullptr ? std::atoi(e) : 1;
                    }();
                    // 1: a strong fit (7+ LEDs), 2: after 3 misses, 4: both together, 0: never
                    const bool exempt = ((far_loose & 1) != 0 && j->total() >= 7) ||
                                        ((far_loose & 4) != 0 && j->total() >= 7 && c_[s].misses >= 3);
                    bool drifted = (far_loose & 2) != 0 && c_[s].misses >= 3;
                    // ... and never next to the other controller (its LEDs fit this hand's model well enough: "doubles")
                    bool clear = true;
                    if (const Ctl& oc = c_[1 - s]; oc.has_state && oc.status == Status::Tracking) {
                        clear = g::norm(g::sub(oc.p, wh.apply(j->body_from_model.t))) > 0.15;
                    }
                    const bool loosen = clear && (exempt || drifted);
                    drifted = loosen;
                    if (c_[s].status == Status::Tracking && !drifted &&
                        g::norm(g::sub(wh.apply(j->body_from_model.t), c_[s].p)) > 0.15 + 1.0 * age) {
                        counts.acq_refused += 1;
                        counts.acq_far += 1;
                        continue;
                    }
                    err = std::max(err, e);
                    constrained = true;
                }
                // without an IMU reference yet the acquisition is tentative: the filter, turned by this
                // controller's gyro, must then agree with the next fits (a mirror fit or the other hand does not)
                (void)constrained;
                if (!best[s] || j->total() > best[s]->j.total() ||
                    (j->total() == best[s]->j.total() && err < best[s]->err)) {
                    best[s] = Pick{k, *j, err};
                }
            }
        }
        for (std::size_t s = 0; s < 2; s++) {
            if (!cold[s]) {
                continue;
            }
            if (const auto acc = hyp_step(s, frame_ns, cold_c[s])) {
                const g::Pose hw = imu_at(s, frame_ns)->world_from_head.inverse();
                bool clash = false;
                if (s == 1 && hyp_v[0] && best[0]) {
                    clash = g::norm(g::sub(best[0]->j.body_from_model.t, acc->first.j.body_from_model.t)) < cfg.acq_apart_m;
                }
                if (!clash) {
                    best[s] = Pick{1000 + s, acc->first.j, 0.0};
                    hyp_v[s] = acc->second;
                }
                (void)hw;
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
            const bool known_orient = known(s, frame_ns);
            if (c.status == Status::Tracking && known_orient) {
                // a tracked controller the prediction missed: the search's fit is a measurement, not a restart
                const g::Pose meas{g::mat_mul(wh.r, j.body_from_model.r), wh.apply(j.body_from_model.t)};
                const std::size_t n = j.total();
                const double sigp = n >= 8 ? cfg.sigma_p[0] : n >= 5 ? cfg.sigma_p[1] : cfg.sigma_p[2];
                if (update(c, meas, sigp, rf::to_radians(n >= 8 ? 2.5 : 5.0), true)) {
                    counts.updates += 1;
                    opt_[s].emplace_back(frame_ns, meas.t);
                    twin_check(s, frame_ns);
                    c.last_update = frame_ns;
                        c.p_fixed = c.p;
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
            c.v = hyp_v[s] ? *hyp_v[s] : g::Vec3{0.0, 0.0, 0.0};
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
            twin_reset(s);
            if (std::getenv("CT_TWIN_FORCE") != nullptr && twin_[s]) {
                // test: start from the twin (the check must switch back)
                const g::Mat3 rti = g::transpose(twin_[s]->t.r);
                c.p = g::add(c.p, g::mat_vec(c.r, g::scale(g::mat_vec(rti, twin_[s]->t.t), -1.0)));
                c.r = g::mat_mul(c.r, rti);
            }
            c.t = frame_ns;
            c.last_update = frame_ns;
                        c.p_fixed = c.p;
            c.misses = 0;
            c.streak = 1;
            c.status = known_orient || hyp_v[s] ? Status::Tracking : Status::Tentative;
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

std::string ImuTracker::describe_slots() const {
    std::string o;
    for (std::size_t s = 0; s < 2; s++) {
        const Ctl& c = c_[s];
        static const char* names[] = {"lost", "tentative", "tracking"};
        o += std::format("{}{} {} up_n {} misses {}", s == 0 ? "" : "; ", s == 0 ? "left" : "right",
                         names[static_cast<int>(c.status)], c.up_n, c.misses);
    }
    return o;
}

} // namespace pibridge::ctrl::track
