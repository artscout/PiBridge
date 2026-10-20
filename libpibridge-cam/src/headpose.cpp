// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/headpose.rs (MIT OR Apache-2.0).
#include "pibridge/cam/headpose.h"

#include "pibridge/cam/calib.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <format>
#include <mutex>

namespace pibridge::cam::headpose {

namespace rf = pibridge::cam::rfmt;
using slam::qconj;
using slam::qmul;
using slam::qnorm;
using slam::qrot;
using slam::QUAT_ID;
using slam::TrackState;

namespace scene {
namespace {
std::atomic<std::uint64_t> g_packed{0};

std::uint64_t now_ms() {
    // OnceLock<Instant>: the first call fixes the base (+1 so that 0 means "never published").
    static const auto base = std::chrono::steady_clock::now();
    const auto d = std::chrono::steady_clock::now() - base;
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(d).count()) + 1;
}
} // namespace

void publish(double mean) {
    if (!std::isfinite(mean)) {
        return;
    }
    const std::uint64_t m = rf::f64_to_u64(rf::clamp(mean, 0.0, 255.0) * 100.0);
    g_packed.store((now_ms() << 32) | (m & 0xffff'ffffULL), std::memory_order_relaxed);
}

std::optional<double> latest(std::uint64_t max_age_ms) {
    const std::uint64_t v = g_packed.load(std::memory_order_relaxed);
    const std::uint64_t ts = v >> 32;
    if (ts == 0) {
        return std::nullopt;
    }
    const std::uint64_t now = now_ms();
    if ((now >= ts ? now - ts : 0) > max_age_ms) {
        return std::nullopt;
    }
    return static_cast<double>(v & 0xffff'ffffULL) / 100.0;
}
} // namespace scene

bool parity_mode() {
    const auto v = env_utf8("PIBRIDGE_HEAD_PARITY");
    if (!v) {
        return false;
    }
    const std::string_view t = rf::trim(*v);
    return !t.empty() && t != "0" && t != "off" && t != "no";
}

Quat qexp(const Vec3& w) {
    const double th = geom::norm(w);
    if (th < 1e-12) {
        return qnorm({w[0] * 0.5, w[1] * 0.5, w[2] * 0.5, 1.0});
    }
    const double s = std::sin(th * 0.5) / th;
    return {w[0] * s, w[1] * s, w[2] * s, std::cos(th * 0.5)};
}

Vec3 qlog(const Quat& q0) {
    Quat q = qnorm(q0);
    if (q[3] < 0.0) {
        q = {-q[0], -q[1], -q[2], -q[3]};
    }
    const Vec3 v{q[0], q[1], q[2]};
    const double s = geom::norm(v);
    if (s < 1e-12) {
        return geom::scale(v, 2.0);
    }
    return geom::scale(v, 2.0 * std::atan2(s, q[3]) / s);
}

/*
 * Predictor
 */

Predictor::Predictor(PredictorParams pp) : p(pp) {}

double Predictor::noise_level() const {
    if (!p.fuse_auto) {
        return 0.0;
    }
    const double vel = (vel_noise_n < 5 || p.noise_hi <= p.noise_lo)
                           ? 0.0
                           : rf::clamp((std::sqrt(vel_noise_ms) - p.noise_lo) / (p.noise_hi - p.noise_lo), 0.0, 1.0);
    return rf::fmax(vel, rf::clamp(scene_dark, 0.0, 1.0));
}

std::array<double, 5> Predictor::fuse_weights(double fast) const {
    const double n = noise_level();
    const double k = 1.0 - 0.5 * n * (1.0 - rf::clamp(fast, 0.0, 1.0));
    return {p.fuse_alpha * k, p.fuse_alpha_y * k, p.fuse_beta * k, p.fuse_vel_w * (1.0 - 0.8 * n), 1.0};
}

std::optional<ImuWindow> Predictor::imu_window(std::int64_t window_ns) const {
    if (imu_.empty()) {
        return std::nullopt;
    }
    const ImuSample& last = imu_.back();
    const std::int64_t from = last.ts_ns - window_ns;
    if (imu_.front().ts_ns > from + window_ns / 10) {
        return std::nullopt; // window not covered yet
    }
    std::size_t n = 0;
    Vec3 sg{}, sg2{}, sa{}, sa2{};
    double gmax = 0.0;
    for (auto it = imu_.rbegin(); it != imu_.rend() && it->ts_ns >= from; ++it) {
        const ImuSample& s = *it;
        n += 1;
        for (int k = 0; k < 3; k++) {
            sg[k] += s.gyro[k];
            sg2[k] += s.gyro[k] * s.gyro[k];
            sa[k] += s.accel[k];
            sa2[k] += s.accel[k] * s.accel[k];
        }
        gmax = rf::fmax(gmax, geom::norm(geom::sub(s.gyro, gyro_bias)));
    }
    if (n == 0) {
        return std::nullopt;
    }
    const double nf = static_cast<double>(n);
    auto sd = [nf](double s, double s2) { return std::sqrt(rf::fmax(s2 / nf - (s / nf) * (s / nf), 0.0)); };
    ImuWindow w;
    w.n = n;
    w.gyro_mean = geom::scale(sg, 1.0 / nf);
    w.gyro_std = {sd(sg[0], sg2[0]), sd(sg[1], sg2[1]), sd(sg[2], sg2[2])};
    w.accel_mean = geom::scale(sa, 1.0 / nf);
    w.accel_std = {sd(sa[0], sa2[0]), sd(sa[1], sa2[1]), sd(sa[2], sa2[2])};
    w.gyro_max = gmax;
    return w;
}

void Predictor::push_imu(const ImuSample& s) {
    if (!imu_.empty() && s.ts_ns <= imu_.back().ts_ns) {
        return;
    }
    imu_.push_back(s);
    while (imu_.size() > std::max<std::size_t>(p.imu_cap, 16)) {
        imu_.pop_front();
    }
}

bool Predictor::imu_still(std::int64_t window_ns, double gyro_std, double accel_std) const {
    const auto w = imu_window(window_ns);
    if (!w || w->n < 10) {
        return false;
    }
    for (int k = 0; k < 3; k++) {
        if (!(w->gyro_std[k] < gyro_std && w->accel_std[k] < accel_std)) {
            return false;
        }
    }
    return true;
}

std::optional<std::int64_t> Predictor::last_imu_ns() const {
    if (imu_.empty()) {
        return std::nullopt;
    }
    return imu_.back().ts_ns;
}

void Predictor::clear_vio() {
    vio_.clear();
    last_raw_.reset();
}

void Predictor::learn_vel_noise(const BodyState& raw) {
    if (!last_raw_) {
        return;
    }
    const BodyState prev = *last_raw_;
    const double dt = static_cast<double>(raw.ts_ns - prev.ts_ns) / 1e9;
    if (!(dt >= 0.005 && dt < 0.15) || imu_.empty() || imu_.back().ts_ns < raw.ts_ns) {
        return;
    }
    const auto [pr, w_unused] = propagate(prev, raw.ts_ns);
    (void)w_unused;
    const double dv = geom::norm(geom::sub(raw.v, pr.v));
    const double k = rf::fmin(dt / rf::fmax(p.noise_tau_s, 1e-3), 1.0);
    if (vel_noise_n == 0) {
        vel_noise_ms = dv * dv;
    } else {
        vel_noise_ms += (dv * dv - vel_noise_ms) * k;
    }
    vel_noise_n += 1;
}

bool Predictor::push_vio(BodyState st, bool learn_bias) {
    if (!vio_.empty()) {
        const BodyState prev = vio_.back();
        if (st.ts_ns <= prev.ts_ns) {
            return false;
        }
        const double dt = static_cast<double>(st.ts_ns - prev.ts_ns) / 1e9;
        if (learn_bias && p.bias_tau_s > 0.0 && dt >= 0.01 && dt <= 0.2) {
            if (const auto raw = gyro_delta_raw(prev.ts_ns, st.ts_ns)) {
                // raw = d_true * exp(b dt)  =>  b = log(d_vio^-1 raw) / dt
                const Quat d_vio = qmul(qconj(qnorm(prev.q)), qnorm(st.q));
                const Vec3 b = geom::scale(qlog(qmul(qconj(d_vio), *raw)), 1.0 / dt);
                if (geom::norm(b) < p.bias_max) {
                    const double a = rf::fmin(dt / p.bias_tau_s, 1.0);
                    gyro_bias = geom::add(gyro_bias, geom::scale(geom::sub(b, gyro_bias), a));
                    bias_samples += 1;
                }
            }
        }
    }
    st.q = qnorm(st.q);
    if (p.fuse_auto && !fuse_hold && !lying) {
        learn_vel_noise(st);
    }
    if (!fuse_hold && !lying) {
        last_raw_ = st;
    } else {
        last_raw_.reset();
    }
    if (p.fuse_alpha > 0.0 && !fuse_hold && !vio_.empty()) {
        const BodyState prev = vio_.back();
        const double dt = static_cast<double>(st.ts_ns - prev.ts_ns) / 1e9;
        const auto [pr, w_unused] = propagate(prev, st.ts_ns);
        (void)w_unused;
        const Vec3 r = geom::sub(st.p, pr.p);
        if (dt > 0.0 && dt < 0.25 && geom::norm(r) < p.fuse_reset_m) {
            const double f =
                p.fuse_fast_speed > 0.0 ? rf::fmin(geom::norm(pr.v) / p.fuse_fast_speed, 1.0) : 0.0;
            const auto wts = fuse_weights(f);
            const double a = wts[0];
            const double ay = wts[1];
            const double b = wts[2] / dt;
            double w = wts[3];
            const double w_fast = wts[4];
            w += rf::fmax(w_fast - w, 0.0) * f;
            st.p = geom::add(pr.p, {a * r[0], ay * r[1], a * r[2]});
            st.v = geom::add(geom::add(pr.v, geom::scale(r, b)), geom::scale(geom::sub(st.v, pr.v), w));
        }
    }
    vio_.push_back(st);
    while (vio_.size() > std::max<std::size_t>(p.vio_cap, 2)) {
        vio_.pop_front();
    }
    return true;
}

std::optional<BodyState> Predictor::latest() const {
    if (vio_.empty()) {
        return std::nullopt;
    }
    return vio_.back();
}

std::optional<BodyState> Predictor::base_at(std::int64_t t) const {
    const auto it = std::partition_point(vio_.begin(), vio_.end(), [t](const BodyState& s) { return s.ts_ns <= t; });
    if (it == vio_.begin()) {
        if (vio_.empty()) {
            return std::nullopt;
        }
        return vio_.front();
    }
    return *(it - 1);
}

std::optional<Quat> Predictor::gyro_delta_raw(std::int64_t t0, std::int64_t t1) const {
    if (imu_.empty()) {
        return std::nullopt;
    }
    const ImuSample& first = imu_.front();
    if (t0 < first.ts_ns - 2'000'000 || imu_.back().ts_ns < t1) {
        return std::nullopt;
    }
    Quat q = QUAT_ID;
    std::int64_t t = t0;
    const auto i0 = std::partition_point(imu_.begin(), imu_.end(), [t0](const ImuSample& s) { return s.ts_ns <= t0; });
    for (auto it = i0; it != imu_.end(); ++it) {
        const ImuSample& s = *it;
        const std::int64_t end = std::min(s.ts_ns, t1);
        if (end > t) {
            q = qmul(q, qexp(geom::scale(s.gyro, static_cast<double>(end - t) / 1e9)));
            t = end;
        }
        if (s.ts_ns >= t1) {
            break;
        }
    }
    return qnorm(q);
}

std::optional<Quat> Predictor::gyro_delta(std::int64_t t0, std::int64_t t1) const {
    const auto raw = gyro_delta_raw(t0, t1);
    if (!raw) {
        return std::nullopt;
    }
    const double dt = static_cast<double>(t1 - t0) / 1e9;
    return qmul(*raw, qexp(geom::scale(gyro_bias, -dt)));
}

std::pair<BodyState, Vec3> Predictor::propagate(const BodyState& base, std::int64_t t) const {
    BodyState st = base;
    const Vec3 bias = gyro_bias;
    Vec3 w_last = imu_.empty() ? Vec3{0.0, 0.0, 0.0} : geom::sub(imu_.back().gyro, bias);
    if (t <= base.ts_ns) {
        return {st, w_last};
    }
    const std::int64_t bt = base.ts_ns;
    const auto i0 = std::partition_point(imu_.begin(), imu_.end(), [bt](const ImuSample& s) { return s.ts_ns <= bt; });
    for (auto it = i0; it != imu_.end(); ++it) {
        const ImuSample& s = *it;
        if (st.ts_ns >= t) {
            break;
        }
        const std::int64_t end = std::min(s.ts_ns, t);
        const double dt = static_cast<double>(end - st.ts_ns) / 1e9;
        const Vec3 w = geom::sub(s.gyro, bias);
        const Vec3 a = (p.use_accel && !lying) ? geom::add(qrot(st.q, s.accel), G_WORLD) : Vec3{0.0, 0.0, 0.0};
        st.p = geom::add(st.p, geom::add(geom::scale(st.v, dt), geom::scale(a, 0.5 * dt * dt)));
        st.v = geom::add(st.v, geom::scale(a, dt));
        st.q = qnorm(qmul(st.q, qexp(geom::scale(w, dt))));
        st.ts_ns = end;
        w_last = w;
    }
    if (st.ts_ns < t) {
        const double dt = static_cast<double>(t - st.ts_ns) / 1e9;
        st.p = geom::add(st.p, geom::scale(st.v, dt));
        Vec3 w_ext = w_last;
        if (p.ang_accel_k > 0.0 && !imu_.empty()) {
            // PiBridge: rate of the last 4 ms and angular acceleration between it and the 4 ms ending 20 ms earlier.
            const std::int64_t te = imu_.back().ts_ns;
            Vec3 sa{}, sb{};
            int na = 0, nb = 0;
            for (auto it = imu_.rbegin(); it != imu_.rend() && it->ts_ns > te - 24'000'000; ++it) {
                if (it->ts_ns > te - 4'000'000) {
                    sa = geom::add(sa, it->gyro);
                    na++;
                } else if (it->ts_ns <= te - 20'000'000) {
                    sb = geom::add(sb, it->gyro);
                    nb++;
                }
            }
            if (na > 0 && nb > 0) {
                const Vec3 wa = geom::sub(geom::scale(sa, 1.0 / na), bias);
                const Vec3 wb = geom::sub(geom::scale(sb, 1.0 / nb), bias);
                const Vec3 alpha = geom::scale(geom::sub(wa, wb), 1.0 / 0.020);
                // the rate at the middle of the extrapolated stretch (the window centre is 2 ms before te)
                const double h = rf::fmin(dt, p.ang_accel_max_s);
                w_ext = geom::add(wa, geom::scale(alpha, p.ang_accel_k * (0.5 * h + 0.002) * h / dt));
            }
        }
        st.q = qnorm(qmul(st.q, qexp(geom::scale(w_ext, dt))));
        st.ts_ns = t;
    }
    return {st, w_last};
}

std::optional<std::pair<BodyState, Vec3>> Predictor::state_at(std::int64_t t) const {
    const auto b = base_at(t);
    if (!b) {
        return std::nullopt;
    }
    return propagate(*b, t);
}

/*
 * HeadParams
 */

HeadParams::HeadParams()
    : w_min(rf::to_radians(3.0)), snap_rot(rf::to_radians(60.0)), tilt_rate(rf::to_radians(0.5)),
      tilt_max(rf::to_radians(5.0)) {}

namespace {

// std::env::var(name).ok().and_then(|v| v.trim().parse().ok())
std::optional<double> env_f64(const char* name) {
    const auto v = env_utf8(name);
    if (!v) {
        return std::nullopt;
    }
    double x = 0;
    if (!rf::parse_f64(rf::trim(*v), x)) {
        return std::nullopt;
    }
    return x;
}

} // namespace

namespace {

// The calibration-derived starting values of the PiBridge additions (HeadParams::seed_gyro_bias, accel_bias).
void load_biases(HeadParams& p, bool seed_gyro, bool accel) {
    std::string src;
    if (seed_gyro) {
        if (const auto g = slam::gyro_bias_cache()) {
            p.seed_gyro_bias = *g;
            src += "gyro bias seeded from the driver cache";
        } else {
            src += "no gyro bias cache";
        }
    }
    if (accel) {
        std::optional<Vec3> a;
        std::string from;
        if (const auto path = slam::slam_config_calib_path()) {
            if (const auto text = read_to_string(*path)) {
                a = slam::calib_json_biases(*text).accel;
                from = "the SLAM_CONFIG calibration";
            }
        }
        if (!a) {
            if (const auto c = DeviceCalib::load_default(std::nullopt); c && !c->first.is_reference()) {
                a = slam::factory_accel_bias(c->first);
                from = "the factory aBias";
            }
        }
        if (!src.empty()) {
            src += ", ";
        }
        if (a) {
            p.accel_bias = *a;
            src += "accel bias from " + from;
        } else {
            src += "no accel bias";
        }
    }
    p.bias_source = src;
}

} // namespace

HeadParams HeadParams::from_env() {
    HeadParams p;
    const bool parity = parity_mode();
    if (!parity) {
        // PiBridge default: no drift hold. On the Crystal Super it holds slow steps and leans as drift (worn take
        // 2026-10-19 07:04: mark error 264-333 mm with it, 64-70 mm without); the IMU cannot tell a creep of a few
        // mm/s from slow head motion, so a correct hold does not exist at that level (head-tracking-improvements.md).
        // PIMAX_SLAM_DRIFT_HOLD=1 (the app's switch) still turns it on.
        p.drift.on = false;
    }
    if (auto v = env_f64("PIMAX_SLAM_SMOOTH")) {
        p.smooth = *v != 0.0;
    }
    if (auto v0 = env_f64("PIMAX_SLAM_TAU_MS")) {
        const double v = rf::clamp(*v0, 5.0, 2000.0) / 1000.0;
        p.tau_pos_s = v;
        p.tau_rot_s = v * 2.0 / 3.0;
    }
    if (auto v = env_f64("PIMAX_SLAM_RESNAP")) {
        p.resnap = *v != 0.0;
    }
    // Output smoothing (2026-10-15, analysis/slam-drift.md "Corrections while worn"): fused position and slow decay at
    // rest are on by default; each can be switched off for A/B.
    if (auto raw = env_utf8("PIMAX_SLAM_FUSE")) {
        const std::string v(rf::trim(*raw));
        if (v == "0" || v == "off") {
            p.fuse = {0.0, 0.0, 0.0, 0.0, 0.0};
        } else {
            std::vector<double> f;
            std::size_t start = 0;
            for (;;) {
                const std::size_t c = v.find(',', start);
                const std::string_view part =
                    std::string_view(v).substr(start, c == std::string::npos ? std::string::npos : c - start);
                double x = 0;
                if (rf::parse_f64(rf::trim(part), x)) {
                    f.push_back(x);
                }
                if (c == std::string::npos) {
                    break;
                }
                start = c + 1;
            }
            if (f.size() == 5 && f[0] >= 0.0 && f[0] <= 1.0) {
                p.fuse = {f[0], f[1], f[2], f[3], f[4]};
            }
        }
    }
    if (auto v = env_f64("PIMAX_FUSE_AUTO")) {
        p.fuse_auto = *v != 0.0;
    }
    if (auto v = env_f64("PIMAX_HEAD_TILT")) {
        p.tilt_tau_s = *v <= 0.0 ? 0.0 : rf::clamp(*v, 100.0, 60'000.0) / 1000.0;
    }
    if (auto v = env_f64("PIMAX_SLAM_TAU_REST_MS")) {
        p.tau_rest_s = rf::clamp(*v, 0.0, 5000.0) / 1000.0;
    }
    if (auto v = env_f64("PIMAX_SLAM_BIG_CORR_MM")) {
        p.big_corr_m = rf::clamp(*v, 1.0, 1000.0) / 1000.0;
    }
    if (auto v = env_f64("PIMAX_SLAM_DRIFT_HOLD")) {
        p.drift.on = *v != 0.0;
        if (*v == 2.0) {
            // PiBridge experiment: hold only while the head is quiet (DriftParams::quiet_gyro_std).
            p.drift.quiet_gyro_std = rf::to_radians(1.3);
            p.drift.quiet_accel_std = 0.09;
        }
    }
    if (auto v = env_f64("PIMAX_SLAM_FREEZE")) {
        p.freeze = *v != 0.0;
    }
    if (auto v = env_f64("PIMAX_SLAM_STILL_REZERO")) {
        p.still.rezero = *v != 0.0;
    }
    if (auto v = env_f64("PIMAX_SLAM_STILL")) {
        p.still.hold = *v > 0.0;
        if (*v > 0.0 && *v != 1.0) {
            p.still.reset_m = rf::clamp(*v, 0.5, 100.0) / 100.0;
        }
    }
    // PiBridge additions: on by default, off in parity mode; the switch wins either way.
    if (!parity) {
        p.predict_accel = 0.75;
    }
    // PiBridge (A/B only): how long the headset must be desk-quiet before the still state (Rust: 0.5 s). Longer was
    // measured worse on the desk take (the dark tracker runs away before the hold starts) and worn heads never pass
    // the 0.2 deg/s test anyway (head-tracking-improvements.md, "Still re-zero while worn").
    if (auto v = env_f64("PIMAX_SLAM_STILL_ENTER_MS")) {
        p.still_enter_window_ns = static_cast<std::int64_t>(rf::clamp(*v, 200.0, 10'000.0) * 1e6);
    }
    if (auto v = env_f64("PIMAX_SLAM_PREDICT_ACCEL")) {
        p.predict_accel = rf::clamp(*v, 0.0, 1.5);
    }
    if (auto v = env_f64("PIMAX_SLAM_STILL_BACKOFF")) {
        // 0 = a re-zero every second (Rust); N = up to 2^N s between re-zeros while still (1..8).
        p.still.backoff_max = *v <= 0.0 ? 0u : static_cast<std::uint32_t>(rf::clamp(std::round(*v), 1.0, 8.0));
    }
    bool seed = !parity;
    if (auto v = env_f64("PIMAX_SLAM_SEED_BIAS")) {
        seed = *v != 0.0;
    }
    bool accel = !parity;
    if (auto v = env_f64("PIMAX_SLAM_ACCEL_BIAS")) {
        accel = *v != 0.0;
    }
    if (seed || accel) {
        load_biases(p, seed, accel);
    }
    return p;
}

std::string HeadParams::describe() const {
    const std::string tilt = tilt_tau_s > 0.0 ? std::format("on (tau {} s, <= {} deg/s)", rf::fixed(tilt_tau_s, 1),
                                                            rf::fixed(rf::to_degrees(tilt_rate), 2))
                                              : std::string("off");
    const std::string autow = (fuse_auto && fuse[0] > 0.0)
                                  ? std::format("on (velocity weight {}..{})", rf::fixed(fuse[3] * 0.2, 2),
                                                rf::fixed(fuse[3], 2))
                                  : std::string("off");
    return std::format("6DoF output: PIMAX_HEAD_TILT gravity tilt correction {}; PIMAX_FUSE_AUTO fusion weights {}; "
                       "still = desk-quiet ({} deg/s, {} s); PIMAX_SLAM_DRIFT_HOLD drift hold {}",
                       tilt, autow, rf::fixed(rf::to_degrees(still_gyro_std), 2),
                       rf::fixed(static_cast<double>(still_enter_window_ns) / 1e9, 1),
                       drift.on ? "on (unexplained tracker translation held)" : "off") +
           pibridge_describe();
}

std::string HeadParams::pibridge_describe() const {
    std::string f = bias_source;
    auto add = [&f](const std::string& x) { f += (f.empty() ? "" : ", ") + x; };
    if (predict_accel > 0.0) {
        add(std::format("rotation predicted with {} of the angular acceleration", rf::fixed(predict_accel, 2)));
    }
    if (still.backoff_max > 0) {
        add(std::format("still re-zero back-off up to {} s",
                        rf::fixed(static_cast<double>(still.reset_every_ns << still.backoff_max) / 1e9, 0)));
    }
    return f.empty() ? std::string() : "; pibridge: " + f;
}

/*
 * HeadTracker
 */

namespace {
PredictorParams predictor_params(const HeadParams& p) {
    PredictorParams pp;
    pp.fuse_alpha = p.fuse[0];
    pp.fuse_alpha_y = p.fuse[1];
    pp.fuse_beta = p.fuse[2];
    pp.fuse_vel_w = p.fuse[3];
    pp.fuse_fast_speed = p.fuse[4];
    pp.fuse_auto = p.fuse_auto;
    pp.ang_accel_k = p.predict_accel;
    return pp;
}

slam::Supervisor make_supervisor(const slam::SupervisorParams& sp, const HeadParams& p, const Vec3& head_in_imu) {
    slam::Supervisor sup(sp, head_in_imu);
    if (p.resnap) {
        sup.set_resnap(slam::Resnap{p.resnap_yaw_deg, p.resnap_speed, p.resnap_min_m});
    }
    sup.set_still_params(p.still);
    return sup;
}
} // namespace

HeadTracker::HeadTracker(slam::SupervisorParams sp, HeadParams hp, Vec3 head_in_imu)
    : sup(make_supervisor(sp, hp, head_in_imu)), pred(predictor_params(hp)), p(std::move(hp)),
      drift(p.drift, head_in_imu), head_in_imu_(head_in_imu) {
    out_ = sup.update(0, std::nullopt);
    if (p.seed_gyro_bias) {
        pred.gyro_bias = *p.seed_gyro_bias;
    }
}

void HeadTracker::push_imu(const ImuSample& s0) {
    ImuSample s = s0;
    if (p.accel_bias != Vec3{0.0, 0.0, 0.0}) {
        s.accel = geom::sub(s.accel, p.accel_bias);
    }
    pred.push_imu(s);
    drift.push_imu(s, pred.gyro_bias);
}

slam::TrackOutput HeadTracker::update(std::int64_t now_ns, std::optional<VioSample> s) {
    const bool tracking_before = sup.state() == TrackState::Tracking;
    const bool was_still = still_;
    still_ = p.still.hold && still_test();
    const bool still = still_;
    if (still && !was_still) {
        // Entered after still_enter_window_ns of quiet: hold the head where it was when that began.
        const std::int64_t from = pred.last_imu_ns().value_or(0) - p.still_enter_window_ns;
        for (auto it = head_hist_.rbegin(); it != head_hist_.rend(); ++it) {
            if (it->first <= from) {
                sup.set_still_anchor(it->second);
                break;
            }
        }
    }
    sup.set_still(still);
    pred.lying = still;
    if (const auto w = pred.imu_window(100'000'000)) {
        noise_sel_.update(w->gyro_max);
    }
    std::optional<slam::TrackSample> ts;
    if (s) {
        ts = slam::TrackSample{static_cast<std::uint64_t>(std::max<std::int64_t>(s->ts_ns, 0)), s->pos, s->q,
                               s->q3dof};
    }
    sup.set_tilt(tilt_);
    slam::TrackOutput o = sup.update(static_cast<std::uint64_t>(std::max<std::int64_t>(now_ns, 0)), ts);
    if ((o.events & slam::events::RESET_WANTED) != 0) {
        pred.clear_vio();
        head_hist_.clear();
        drift.reset();
        // A fresh tracker world starts gravity aligned (Basalt initialises its attitude from the accelerometer): its
        // correction starts from zero when it takes over. Until then the held attitude carries the old correction.
        tilt_reset_ = true;
    }
    if (tilt_reset_ && o.state == TrackState::Tracking) {
        tilt_reset_ = false;
        tilt_ = QUAT_ID;
        tilt_ns_.reset();
    }
    if (s) {
        const VioSample v = *s;
        if (head_hist_.empty() || v.ts_ns > head_hist_.back().first) {
            head_hist_.emplace_back(v.ts_ns, geom::add(v.pos, qrot(qnorm(v.q), head_in_imu_)));
            while (head_hist_.size() > 150) {
                head_hist_.pop_front();
            }
        }
        const bool learn = p.learn_bias && tracking_before && o.state == TrackState::Tracking;
        pred.fuse_hold = still || !tracking_before;
        // Lying still: whatever velocity a landmark-less tracker integrated is not real.
        pred.push_vio(BodyState{v.ts_ns, v.pos, v.q, still ? Vec3{0.0, 0.0, 0.0} : v.vel}, learn);
        if (tracking_before && o.state == TrackState::Tracking) {
            bool loud = false;
            if (p.drift.quiet_gyro_std > 0.0) {
                // PiBridge quiet-only hold: a moving head (a step, a lean) is never held.
                const auto w = pred.imu_window(1'000'000'000);
                loud = !w || w->n < 100;
                for (int k = 0; !loud && k < 3; k++) {
                    loud = w->gyro_std[k] >= p.drift.quiet_gyro_std || w->accel_std[k] >= p.drift.quiet_accel_std;
                }
            }
            if (still || loud) {
                drift.reset();
            } else if (const auto b = pred.latest()) {
                const std::int64_t at = std::max(pred.last_imu_ns().value_or(b->ts_ns), b->ts_ns);
                const Quat q_now = pred.propagate(*b, at).first.q;
                if (const auto d = drift.on_pose(*b, q_now, sup.world_from_slam().second)) {
                    sup.shift_world(*d);
                    o.world_from_slam = sup.world_from_slam();
                }
            }
            out_ = o;
            update_tilt(v);
        } else {
            drift.reset();
        }
    }
    out_ = o;
    return o;
}

bool HeadTracker::still_test() const {
    if (still_) {
        return pred.imu_still(p.still_window_ns, p.still_exit_gyro_std, p.still_exit_accel_std);
    }
    const auto w = pred.imu_window(p.still_enter_window_ns);
    if (!w) {
        return false;
    }
    bool quiet = w->n >= 10;
    for (int k = 0; quiet && k < 3; k++) {
        quiet = w->gyro_std[k] < p.still_gyro_std && w->accel_std[k] < p.still_accel_std;
    }
    if (!quiet) {
        return false;
    }
    // 250 tracker poses = 10 s of learning: before that the bias estimate may still be far off.
    if (p.still_bias_tol > 0.0 && pred.bias_samples >= 250) {
        const double off = geom::norm(geom::sub(w->gyro_mean, pred.gyro_bias));
        if (off > p.still_bias_tol) {
            return false;
        }
    }
    return true;
}

void HeadTracker::update_tilt(const VioSample& v) {
    if (p.tilt_tau_s <= 0.0) {
        return;
    }
    double dt = 0.0;
    if (tilt_ns_) {
        if (v.ts_ns > *tilt_ns_) {
            dt = rf::fmin(static_cast<double>(v.ts_ns - *tilt_ns_) / 1e9, 0.2);
        } else {
            return;
        }
    }
    tilt_ns_ = v.ts_ns;
    const auto w = pred.imu_window(20'000'000);
    if (!w) {
        return;
    }
    const double an = geom::norm(w->accel_mean);
    if (w->n < 5 || std::fabs(an - GRAVITY) > p.tilt_g_tol || w->gyro_max > p.tilt_gyro_max) {
        tilt_skipped += 1;
        return;
    }
    const Quat o = qmul(tilt_, qmul(out_.world_from_slam.second, qnorm(v.q)));
    const Vec3 up_w = qrot(o, geom::scale(w->accel_mean, 1.0 / an));
    const Vec3 e = geom::cross(up_w, {0.0, 1.0, 0.0});
    const double s = geom::norm(e);
    const double ang = std::atan2(s, up_w[1]);
    if (ang > p.tilt_max) {
        tilt_skipped += 1;
        return;
    }
    if (s < 1e-9) {
        return;
    }
    const double step = rf::fmin(ang * (1.0 - std::exp(-dt / p.tilt_tau_s)), p.tilt_rate * dt);
    const Quat next = qnorm(qmul(qexp(geom::scale(e, step / s)), tilt_));
    // Keep it a pure swing about a horizontal axis (drop any twist about world up: yaw is the tracker's).
    const Quat twist = qnorm({0.0, next[1], 0.0, next[3]});
    Quat t = qnorm(qmul(next, qconj(twist)));
    const double mag = geom::norm(qlog(t));
    if (mag > p.tilt_max) {
        t = qexp(geom::scale(qlog(t), p.tilt_max / mag));
    }
    tilt_ = t;
    tilt_updates += 1;
}

void HeadTracker::set_scene_mean(std::optional<double> mean) {
    if (mean && std::isfinite(*mean)) {
        pred.scene_dark = rf::clamp((SCENE_DARK_FROM - *mean) / (SCENE_DARK_FROM - SCENE_DARK_FULL), 0.0, 1.0);
    } else {
        pred.scene_dark = 0.0;
    }
}

HeadTracker::Config HeadTracker::config() const {
    return Config{out_.state, out_.world_from_slam.first, out_.world_from_slam.second, out_.fix3dof, out_.held_pos,
                  pred.latest()};
}

Rel HeadTracker::eval(const Config& c, std::int64_t t, const Rel& rel3dof) const {
    if (c.state == TrackState::Tracking && c.base) {
        BodyState b = *c.base;
        if (t < b.ts_ns) {
            b = pred.base_at(t).value_or(b);
        }
        const auto [st, w_body] = pred.propagate(b, t);
        const Vec3 r = qrot(st.q, head_in_imu_);
        const Vec3 head_slam = geom::add(st.p, r);
        const Vec3 w_slam = qrot(st.q, w_body);
        const Vec3 v_head = geom::add(st.v, geom::cross(w_slam, r));
        Rel o;
        o.pos = geom::add(qrot(c.w_q, head_slam), c.w_t);
        o.q = qnorm(qmul(tilt_, qmul(c.w_q, st.q)));
        o.lin_vel = qrot(c.w_q, v_head);
        o.ang_vel = qrot(tilt_, qrot(c.w_q, w_slam));
        o.flags = flags::ALL;
        return o;
    }
    std::uint32_t f = rel3dof.flags & ~(flags::POSITION_TRACKED | flags::LINEAR_VELOCITY_VALID);
    if (c.state == TrackState::Lost) {
        f |= flags::POSITION_VALID;
    }
    Rel o;
    o.pos = c.held;
    o.q = qnorm(qmul(c.fix3dof, rel3dof.q));
    o.lin_vel = {0.0, 0.0, 0.0};
    o.ang_vel = qrot(c.fix3dof, rel3dof.ang_vel);
    o.flags = f;
    return o;
}

void HeadTracker::decay(std::int64_t now_ns) {
    if (!corr_ns_) {
        corr_ns_ = now_ns;
        return;
    }
    const std::int64_t t0 = *corr_ns_;
    if (now_ns <= t0) {
        return;
    }
    const double dt = static_cast<double>(now_ns - t0) / 1e9;
    corr_ns_ = now_ns;
    auto shrink = [dt](const Vec3& c, double tau, double limit) -> Vec3 {
        const double n = geom::norm(c);
        if (n < 1e-12) {
            return {0.0, 0.0, 0.0};
        }
        const double want = n * (1.0 - std::exp(-dt / rf::fmax(tau, 1e-3)));
        const double step = rf::fmin(want, limit * dt);
        return geom::scale(c, (n - step) / n);
    };
    double vl = p.v_min + p.k_v * speed_;
    double wl = p.w_min + p.k_w * ang_speed_;
    double tau = p.tau_pos_s;
    double tau_r = p.tau_rot_s;
    if (p.tau_rest_s > tau && p.rest_speed > 0.0) {
        const double rest = rf::clamp(1.0 - speed_ / p.rest_speed, 0.0, 1.0);
        tau += (p.tau_rest_s - tau) * rest;
    }
    if (still_) {
        // Desk-quiet: nobody wears it (a worn head is never this quiet), so a correction (the hold going back to
        // where the quiet began, a re-zero takeover) is taken in at once instead of sliding on into the moment it is
        // picked up again.
        tau = rf::fmin(tau, 0.05);
        tau_r = rf::fmin(tau_r, 0.05);
        vl = rf::fmax(vl, 5.0);
        wl = rf::fmax(wl, 1.0);
    }
    corr_p_ = shrink(corr_p_, tau, vl);
    corr_r_ = shrink(corr_r_, tau_r, wl);
}

Rel HeadTracker::head(std::int64_t now_ns, std::int64_t at_ns, const Rel& rel3dof) {
    decay(now_ns);
    const Config cur = config();
    const std::optional<Config> prev = emitted_;
    Rel raw = eval(cur, at_ns, rel3dof);
    // Continuity: when the configuration changed since the last output, put the difference at this query time into
    // the correction. Past queries (before the newest tracker pose) wait for a current one.
    const bool current_query = !cur.base || at_ns >= cur.base->ts_ns;
    if (prev != std::optional<Config>(cur) && current_query) {
        if (prev && p.smooth) {
            const Config& pc = *prev;
            const Rel old = eval(pc, at_ns, rel3dof);
            const Vec3 old_out_p = geom::add(old.pos, corr_p_);
            const Quat old_out_q = qmul(qexp(corr_r_), old.q);
            const Vec3 np = geom::sub(old_out_p, raw.pos);
            const Vec3 nr = qlog(qmul(old_out_q, qconj(raw.q)));
            if (geom::norm(np) > p.snap_pos || geom::norm(nr) > p.snap_rot) {
                corr_p_ = {0.0, 0.0, 0.0};
                corr_r_ = {0.0, 0.0, 0.0};
                corr.snapped += 1;
            } else {
                CorrEvent ev;
                ev.ts_ns = now_ns;
                ev.step = geom::sub(np, corr_p_);
                ev.offset = np;
                ev.rot = geom::norm(nr);
                ev.speed = speed_;
                ev.ang_speed = ang_speed_;
                ev.pose_only = pc.state == cur.state && pc.w_t == cur.w_t && pc.w_q == cur.w_q &&
                               pc.fix3dof == cur.fix3dof && pc.held == cur.held;
                corr_p_ = np;
                corr_r_ = nr;
                corr.blended += 1;
                corr.max_pos = rf::fmax(corr.max_pos, geom::norm(np));
                corr.max_rot = rf::fmax(corr.max_rot, geom::norm(nr));
                if (geom::norm(geom::sub(ev.step, {0.0, 0.0, 0.0})) >= p.big_corr_m) {
                    corr.big += 1;
                    const double h = std::sqrt(ev.step[0] * ev.step[0] + ev.step[2] * ev.step[2]);
                    if (std::fabs(ev.step[1]) > h) {
                        corr.big_vertical += 1;
                    }
                    if (big_q_.size() >= 64) {
                        big_q_.pop_front();
                    }
                    big_q_.push_back(ev);
                }
                if (corr_log) {
                    corr_log->push_back(ev);
                }
            }
        }
        emitted_ = cur;
    }
    if (!p.smooth) {
        corr_p_ = {0.0, 0.0, 0.0};
        corr_r_ = {0.0, 0.0, 0.0};
    }
    // Positions only carry a correction when the output has a position at all.
    raw.pos = geom::add(raw.pos, corr_p_);
    raw.q = qnorm(qmul(qexp(corr_r_), raw.q));
    if (current_query) {
        freeze_step(raw);
    } else if (frozen_) {
        raw = *frozen_;
    }
    if (current_query) {
        speed_ = geom::norm(raw.lin_vel);
        ang_speed_ = geom::norm(raw.ang_vel);
    }
    return raw;
}

void HeadTracker::freeze_step(Rel& raw) {
    const bool was = freeze_on_;
    if (!p.freeze || !still_) {
        freeze_on_ = false;
    } else if (const auto w = pred.imu_window(p.freeze_window_ns)) {
        const double lim = was ? p.freeze_gyro_off : p.freeze_gyro_on;
        freeze_on_ = w->gyro_max < lim;
    }
    if (freeze_on_) {
        Rel live = raw;
        live.lin_vel = {0.0, 0.0, 0.0};
        live.ang_vel = {0.0, 0.0, 0.0};
        Rel f = live;
        if (frozen_ && geom::norm(qlog(qmul(raw.q, qconj(frozen_->q)))) <= p.freeze_deadband_rad &&
            geom::norm(geom::sub(raw.pos, frozen_->pos)) <= p.freeze_deadband_m) {
            f = *frozen_;
        }
        frozen_ = f;
        raw = f;
        freeze_frames += 1;
    } else if (frozen_) {
        const Rel f = *frozen_;
        frozen_.reset();
        // Released: the frozen pose stays this query's output; the correction carries the difference.
        const Vec3 uncorr_p = geom::sub(raw.pos, corr_p_);
        const Quat uncorr_q = qmul(qexp(geom::scale(corr_r_, -1.0)), raw.q);
        corr_p_ = geom::sub(f.pos, uncorr_p);
        corr_r_ = qlog(qmul(f.q, qconj(uncorr_q)));
        if (geom::norm(corr_p_) > p.snap_pos || geom::norm(corr_r_) > p.snap_rot) {
            corr_p_ = {0.0, 0.0, 0.0};
            corr_r_ = {0.0, 0.0, 0.0};
        } else {
            raw = f;
        }
    }
}

std::optional<CorrEvent> HeadTracker::take_big() {
    if (big_q_.empty()) {
        return std::nullopt;
    }
    CorrEvent e = big_q_.front();
    big_q_.pop_front();
    return e;
}

double HeadTracker::room_yaw_deg() const {
    return rf::to_degrees(slam::yaw_of(out_.world_from_slam.second));
}

} // namespace pibridge::cam::headpose
