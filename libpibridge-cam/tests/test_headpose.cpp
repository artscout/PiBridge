// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-cam/src/headpose.rs (MIT OR Apache-2.0). The two #[ignore] replays of a
// session's head-imu.csv (replay_head_imu_log, replay_drift_hold) are analysis tools, not checks; the recorded
// replay is covered by the difftest (monado_abi_diff_slam_replay_*).
#include "pibridge/cam/headpose.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge_test.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>

using namespace pibridge::cam;
using namespace pibridge::cam::headpose;
using slam::qangle_deg;
using slam::qconj;
using slam::qmul;
using slam::qnorm;
using slam::qrot;
using slam::qyaw;
using slam::QUAT_ID;
using slam::SupervisorParams;
using slam::TrackState;
namespace rf = pibridge::cam::rfmt;
namespace ev = pibridge::cam::slam::events;

namespace {

constexpr double PI = 3.14159265358979323846264338327950288;

std::int64_t round_ns(double s) {
    return static_cast<std::int64_t>(std::round(s * 1e9));
}

SupervisorParams acquire3() {
    SupervisorParams p;
    p.acquire_poses = 3;
    return p;
}

/// Deterministic standard normal (Irwin-Hall from an LCG).
double gauss(std::uint64_t& seed) {
    double s = 0.0;
    for (int i = 0; i < 12; i++) {
        seed = seed * 6364136223846793005ull + 1442695040888963407ull;
        s += static_cast<double>(seed >> 11) / static_cast<double>(1ull << 53);
    }
    return s - 6.0;
}

/// IMU at rest for secs from t0: bias + white noise (rad/s, m/s^2 sd per axis) + a breathing / pulse sway (rad/s
/// amplitude), the way a worn head sits.
std::vector<ImuSample> rest_trace(double t0, double secs, double noise_g, double noise_a, double sway,
                                  std::uint64_t& seed) {
    const Vec3 bias{-0.0043, 0.0138, -0.0059}; // the Dream Air's ~0.85 deg/s
    const auto n = static_cast<std::int64_t>(secs * 1000.0);
    std::vector<ImuSample> v;
    for (std::int64_t i = 0; i < n; i++) {
        const double t = t0 + static_cast<double>(i) * 0.001;
        const double sw = sway * (std::sin(2.0 * PI * 0.25 * t) + 0.4 * std::sin(2.0 * PI * 1.2 * t));
        ImuSample s;
        s.ts_ns = round_ns(t);
        s.gyro[0] = bias[0] + sw + noise_g * gauss(seed);
        s.gyro[1] = bias[1] + 0.5 * sw + noise_g * gauss(seed);
        s.gyro[2] = bias[2] + noise_g * gauss(seed);
        s.accel[0] = noise_a * gauss(seed);
        s.accel[1] = GRAVITY + noise_a * gauss(seed);
        s.accel[2] = noise_a * gauss(seed);
        v.push_back(s);
    }
    return v;
}

std::pair<bool, std::uint64_t> still_after(HeadParams p, const std::vector<ImuSample>& trace,
                                           std::optional<Vec3> bias, std::size_t count = SIZE_MAX) {
    HeadTracker h(SupervisorParams{}, std::move(p), {0.0, 0.0, 0.0});
    if (bias) {
        h.pred.gyro_bias = *bias;
        h.pred.bias_samples = 1000;
    }
    std::uint64_t entered = 0;
    for (std::size_t i = 0; i < std::min(count, trace.size()); i++) {
        h.push_imu(trace[i]);
        if (i % 11 == 0) {
            h.update(trace[i].ts_ns, std::nullopt);
            entered += h.is_still() ? 1 : 0;
        }
    }
    return {h.is_still(), entered};
}

Rel rel3(const Quat& q) {
    Rel r;
    r.q = q;
    r.flags = flags::ORIENTATION_VALID | flags::ORIENTATION_TRACKED;
    return r;
}

/*
 * Tilt runs
 */

Quat tilt_true_q(double t, bool moving) {
    const double yaw = moving ? 0.05 * t + 0.02 * std::sin(3.0 * t) : 0.0;
    return qmul(qyaw(yaw), {std::sin(0.1), 0.0, 0.0, std::cos(0.1)});
}

double tilt_yaw_rate(double t, bool moving) {
    return moving ? 0.05 + 0.06 * std::cos(3.0 * t) : 0.0;
}

struct TiltRun {
    std::vector<std::pair<double, Rel>> out;
    std::vector<Quat> tilts;
    std::unique_ptr<HeadTracker> h;
};

TiltRun run_tilt(HeadParams p, double secs, bool moving, const std::function<std::pair<Quat, Vec3>(double, bool)>& err) {
    auto h = std::make_unique<HeadTracker>(acquire3(), std::move(p), Vec3{0.0, -0.02, 0.07});
    std::uint64_t seed = 11;
    TiltRun r;
    bool reset = false;
    std::int64_t i = 0;
    while (static_cast<double>(i) * 0.001 < secs) {
        const double t = static_cast<double>(i) * 0.001;
        const std::int64_t now = i * 1'000'000;
        const Quat q = tilt_true_q(t, moving);
        const Vec3 wr{0.0, tilt_yaw_rate(t, moving), 0.0};
        ImuSample s;
        s.ts_ns = now;
        s.gyro = geom::add(qrot(qconj(q), wr), {0.0004 * gauss(seed), 0.0, 0.0});
        s.accel = geom::add(qrot(qconj(q), {0.0, GRAVITY, 0.0}), {0.01 * gauss(seed), 0.0, 0.0});
        h->push_imu(s);
        std::optional<VioSample> vio;
        if (i % 40 == 0 && i > 0) {
            const auto [e, off] = err(t, reset);
            vio = VioSample{now, off, qmul(e, q), {0.0, 0.0, 0.0}, q};
        }
        const auto o = h->update(now, vio);
        if ((o.events & ev::RESET_WANTED) != 0) {
            reset = true;
        }
        if (vio) {
            r.tilts.push_back(h->tilt());
        }
        if (i % 11 == 0) {
            Rel rr = rel3(q);
            rr.ang_vel = wr;
            r.out.emplace_back(t, h->head(now, now, rr));
        }
        i += 1;
    }
    r.h = std::move(h);
    return r;
}

/// Angle (deg) between the output's up and the true up.
double up_err_deg(double t, const Rel& r, bool moving) {
    const Vec3 up_true = qrot(qconj(tilt_true_q(t, moving)), {0.0, 1.0, 0.0});
    const Vec3 up_out = qrot(qconj(r.q), {0.0, 1.0, 0.0});
    return rf::to_degrees(std::acos(rf::clamp(geom::dot(up_true, up_out), -1.0, 1.0)));
}

bool tracked(const Rel& r) {
    return (r.flags & flags::POSITION_TRACKED) != 0;
}

/*
 * The walk (truth) and the full loop
 */

struct Truth {
    Vec3 p, v, w, a;
    Quat q;
};

/// Smooth head motion: slow walk on a circle, head yaw/pitch oscillation. World y-up.
Truth truth(double t) {
    const double r = 0.8;
    const double wc = 0.35; // rad/s around the circle
    // Walking bobs the head: 1.5 cm at the step rate (1.8 Hz), about 1.9 m/s^2.
    const double ws = 2.0 * PI * 1.8;
    const double bob = 0.015;
    Truth x;
    x.p = {r * std::sin(wc * t), 1.6 + 0.03 * std::sin(1.3 * t) + bob * std::sin(ws * t), r * std::cos(wc * t) - r};
    x.v = {r * wc * std::cos(wc * t), 0.03 * 1.3 * std::cos(1.3 * t) + bob * ws * std::cos(ws * t),
           -r * wc * std::sin(wc * t)};
    x.a = {-r * wc * wc * std::sin(wc * t), -0.03 * 1.69 * std::sin(1.3 * t) - bob * ws * ws * std::sin(ws * t),
           -r * wc * wc * std::cos(wc * t)};
    // yaw(t) = 0.6 sin(0.9 t) + 0.2 t ; pitch(t) = 0.25 sin(1.7 t)
    const double yaw = 0.6 * std::sin(0.9 * t) + 0.2 * t;
    const double pitch = 0.25 * std::sin(1.7 * t);
    const Quat qy = qyaw(yaw);
    const Quat qp{std::sin(pitch / 2.0), 0.0, 0.0, std::cos(pitch / 2.0)};
    x.q = qmul(qy, qp);
    // angular velocity (world) = yaw_rate * Y + Ry(yaw) * pitch_rate * X
    const double yr = 0.54 * std::cos(0.9 * t) + 0.2;
    const double pr = 0.425 * std::cos(1.7 * t);
    x.w = geom::add({0.0, yr, 0.0}, qrot(qy, {pr, 0.0, 0.0}));
    return x;
}

ImuSample imu_at(double t, const Vec3& bias) {
    const Truth x = truth(t);
    ImuSample s;
    s.ts_ns = round_ns(t);
    s.gyro = geom::add(qrot(qconj(x.q), x.w), bias);
    s.accel = qrot(qconj(x.q), geom::sub(x.a, G_WORLD));
    return s;
}

using Out = std::vector<std::pair<double, Rel>>;
using VioFn = std::function<std::optional<std::tuple<Vec3, Quat, Vec3>>(double)>;

/// Full loop: IMU 1 kHz, VIO 25 Hz delivered 25 ms late with noise, queries at 90 Hz for display time (now + 20 ms).
struct Sim {
    std::unique_ptr<HeadTracker> h;
    std::uint64_t rng = 0x1234'5678;

    explicit Sim(HeadParams p) : h(std::make_unique<HeadTracker>(acquire3(), std::move(p), Vec3{0.0, -0.02, 0.07})) {}

    double gaussian() {
        // Irwin-Hall approximation from splitmix64.
        double s = 0.0;
        for (int i = 0; i < 12; i++) {
            rng = rng + 0x9E37'79B9'7F4A'7C15ull;
            std::uint64_t z = rng;
            z = (z ^ (z >> 30)) * 0xBF58'476D'1CE4'E5B9ull;
            z = (z ^ (z >> 27)) * 0x94D0'49BB'1331'11EBull;
            z ^= z >> 31;
            s += static_cast<double>(z >> 11) / static_cast<double>(1ull << 53);
        }
        return s - 6.0;
    }

    void run(double t0, double t1, double noise_m, const VioFn& vio, Out& out) {
        double t = t0;
        double next_vio = t0;
        std::vector<std::pair<double, VioSample>> pending;
        double next_q = t0;
        while (t < t1) {
            const std::int64_t now = round_ns(t);
            h->push_imu(imu_at(t, {0.0, 0.0, 0.0}));
            if (t >= next_vio) {
                if (const auto v = vio(t)) {
                    const auto [p, q, vel] = *v;
                    const double nx = gaussian() * noise_m;
                    const double ny = gaussian() * noise_m;
                    const double nz = gaussian() * noise_m;
                    const Quat q3 = truth(t).q;
                    pending.emplace_back(t + 0.025, VioSample{now, geom::add(p, {nx, ny, nz}), q, vel, q3});
                }
                next_vio += 0.04;
            }
            const auto it = std::find_if(pending.begin(), pending.end(), [t](const auto& x) { return x.first <= t; });
            if (it != pending.end()) {
                const VioSample s = it->second;
                pending.erase(it);
                h->update(now, s);
            } else {
                h->update(now, std::nullopt);
            }
            if (t >= next_q) {
                const double at = t + 0.020;
                const Truth x = truth(at);
                Rel r = rel3(x.q);
                r.ang_vel = x.w;
                const Rel o = h->head(now, round_ns(at), r);
                out.emplace_back(at, o);
                next_q += 1.0 / 90.0;
            }
            t += 0.001;
        }
    }
};

std::pair<Vec3, Quat> head_truth(double t, const Vec3& head) {
    const Truth x = truth(t);
    return {geom::add(x.p, qrot(x.q, head)), x.q};
}

/// Tracker world = truth yawed and offset (as Basalt's own world would be).
std::tuple<Vec3, Quat, Vec3> slam_of(double t, double yaw, const Vec3& off) {
    const Truth x = truth(t);
    const Quat wq = qyaw(yaw);
    return {geom::add(qrot(wq, x.p), off), qmul(wq, x.q), qrot(wq, x.v)};
}

/// First tracked output at or after t_ref.
std::pair<double, Rel> ref_at(const Out& out, double t_ref) {
    for (const auto& o : out) {
        if (o.first >= t_ref && tracked(o.second)) {
            return o;
        }
    }
    return out.back();
}

/// Where the output should be at t given a reference output.
Vec3 expected(const std::pair<double, Rel>& rf0, double t, const Vec3& head, const Vec3& extra) {
    const auto [p0, q0] = head_truth(rf0.first, head);
    const auto [pt, qt] = head_truth(t, head);
    (void)qt;
    const Quat wq = qyaw(slam::yaw_of(qmul(rf0.second.q, qconj(q0))));
    return geom::add(rf0.second.pos, qrot(wq, geom::add(geom::sub(pt, p0), extra)));
}

/// Error of the outputs against truth after aligning the first tracked output: (max position, max rotation, jitter).
std::tuple<double, double, double> errors(const Out& out, double from, const Vec3& head) {
    std::vector<const std::pair<double, Rel>*> tr;
    for (const auto& o : out) {
        if (o.first >= from && tracked(o.second)) {
            tr.push_back(&o);
        }
    }
    const auto& [t0, r0] = *tr[0];
    const auto [p0, q0] = head_truth(t0, head);
    // world_from_truth: yaw + offset mapping truth to the output frame.
    const Quat wq = qyaw(slam::yaw_of(qmul(r0.q, qconj(q0))));
    double max_p = 0.0, max_r = 0.0, jit = 0.0;
    std::optional<Vec3> prev;
    for (const auto* o : tr) {
        const auto [pt, qt] = head_truth(o->first, head);
        const Vec3 pe = geom::add(qrot(wq, geom::sub(pt, p0)), r0.pos);
        const Vec3 e = geom::sub(o->second.pos, pe);
        max_p = rf::fmax(max_p, geom::norm(e));
        max_r = rf::fmax(max_r, qangle_deg(qmul(qconj(o->second.q), qmul(wq, qt))));
        if (prev) {
            jit = rf::fmax(jit, geom::norm(geom::sub(e, *prev)));
        }
        prev = e;
    }
    return {max_p, max_r, jit};
}

/*
 * The drift governor body
 */

constexpr Vec3 GOV_HEAD{0.049, -0.015, 0.030};
constexpr Vec3 TRUE_NECK_IN_HEAD{0.0, -0.11, 0.095};

using NeckFn = std::function<std::tuple<Vec3, Vec3, Vec3>(double)>;
using YawFn = std::function<std::tuple<double, double, double>(double)>;

struct Body {
    NeckFn neck;
    YawFn yaw;
    /// IMU position, orientation, velocity, angular velocity, acceleration (world).
    void imu(double t, Vec3& p, Quat& q, Vec3& v, Vec3& w, Vec3& a) const {
        const auto [n, nv, na] = neck(t);
        const auto [th, thd, thdd] = yaw(t);
        q = qmul(qyaw(th), {std::sin(-0.15 / 2.0), 0.0, 0.0, std::cos(-0.15 / 2.0)});
        w = {0.0, thd, 0.0};
        const Vec3 al{0.0, thdd, 0.0};
        const Vec3 r = qrot(q, geom::scale(geom::add(GOV_HEAD, TRUE_NECK_IN_HEAD), -1.0));
        p = geom::add(n, r);
        v = geom::add(nv, geom::cross(w, r));
        a = geom::add(na, geom::add(geom::cross(al, r), geom::cross(w, geom::cross(w, r))));
    }
    Vec3 head(double t) const {
        Vec3 p, v, w, a;
        Quat q;
        imu(t, p, q, v, w, a);
        return geom::add(p, qrot(q, GOV_HEAD));
    }
};

using GovOut = std::vector<std::tuple<double, Vec3, Vec3>>;

/// Run body for secs with a tracker that slides by drift(t) (world m), IMU noise like a worn head, tracker noise 1 mm,
/// poses 25 Hz delivered 25 ms late. Returns (time, output head, true head) per 90 Hz query once tracking.
std::pair<GovOut, std::unique_ptr<HeadTracker>> gov_run(HeadParams p, const Body& body, double secs,
                                                       const std::function<Vec3(double)>& drift) {
    auto h = std::make_unique<HeadTracker>(acquire3(), std::move(p), GOV_HEAD);
    std::uint64_t seed = 21;
    std::deque<std::pair<double, VioSample>> pending;
    double next_vio = 0.0, next_q = 0.0;
    GovOut out;
    std::int64_t i = 0;
    auto n3 = [](std::uint64_t& s, double k) {
        const double x = k * gauss(s);
        const double y = k * gauss(s);
        const double z = k * gauss(s);
        return Vec3{x, y, z};
    };
    while (static_cast<double>(i) * 0.001 < secs) {
        const double t = static_cast<double>(i) * 0.001;
        const std::int64_t now = i * 1'000'000;
        Vec3 p0, v0, w, a;
        Quat q;
        body.imu(t, p0, q, v0, w, a);
        ImuSample s;
        s.ts_ns = now;
        const Vec3 gn = n3(seed, 0.002);
        s.gyro = geom::add(geom::add(qrot(qconj(q), w), gn),
                           {0.008 * std::sin(9.0 * t), 0.007 * std::sin(6.3 * t + 1.0), 0.006 * std::sin(11.0 * t)});
        const Vec3 an = n3(seed, 0.02);
        s.accel = geom::add(qrot(qconj(q), geom::sub(a, G_WORLD)), an);
        h->push_imu(s);
        if (t >= next_vio) {
            Vec3 pp, vv, ww, aa;
            Quat qq;
            body.imu(t, pp, qq, vv, ww, aa);
            const Vec3 d = drift(t);
            const Vec3 dv = geom::scale(geom::sub(drift(t + 0.01), drift(t - 0.01)), 50.0);
            const Vec3 pn = n3(seed, 0.001);
            pending.emplace_back(t + 0.025, VioSample{now, geom::add(geom::add(pp, d), pn), qq, geom::add(vv, dv), qq});
            next_vio += 0.04;
        }
        std::optional<VioSample> sv;
        if (!pending.empty() && pending.front().first <= t) {
            sv = pending.front().second;
            pending.pop_front();
        }
        h->update(now, sv);
        if (t >= next_q) {
            Rel r = rel3(q);
            r.ang_vel = w;
            const Rel o = h->head(now, now, r);
            if (h->output().state == TrackState::Tracking) {
                out.emplace_back(t, o.pos, body.head(t));
            }
            next_q += 1.0 / 90.0;
        }
        i += 1;
    }
    return {std::move(out), std::move(h)};
}

/// Output and true head displacement between t0 and t1 (s).
std::pair<Vec3, Vec3> gov_disp(const GovOut& out, double t0, double t1) {
    auto at = [&](double t) {
        const auto* best = &out[0];
        for (const auto& o : out) {
            if (std::fabs(std::get<0>(o) - t) < std::fabs(std::get<0>(*best) - t)) {
                best = &o;
            }
        }
        return *best;
    };
    const auto a = at(t0);
    const auto b = at(t1);
    return {geom::sub(std::get<1>(b), std::get<1>(a)), geom::sub(std::get<2>(b), std::get<2>(a))};
}

HeadParams drift_off() {
    HeadParams p;
    p.drift.on = false;
    return p;
}

/// Raised-cosine move of d (m, along dir) from t0 over dur s: position, velocity, acceleration.
std::tuple<Vec3, Vec3, Vec3> smooth_move(double t, double t0, double dur, double d, const Vec3& dir) {
    double x = 0.0, v = 0.0, a = 0.0;
    if (t <= t0) {
    } else if (t >= t0 + dur) {
        x = d;
    } else {
        const double u = PI * (t - t0) / dur;
        x = 0.5 * d * (1.0 - std::cos(u));
        v = 0.5 * d * (PI / dur) * std::sin(u);
        a = 0.5 * d * ((PI / dur) * (PI / dur)) * std::cos(u);
    }
    return {geom::scale(dir, x), geom::scale(dir, v), geom::scale(dir, a)};
}

NeckFn seated() {
    // breathing sway: 4 mm at 0.25 Hz
    return [](double t) {
        const double w = 2.0 * PI * 0.25;
        return std::tuple<Vec3, Vec3, Vec3>{{0.004 * std::sin(w * t), 1.2, 0.0},
                                            {0.004 * w * std::cos(w * t), 0.0, 0.0},
                                            {-0.004 * w * w * std::sin(w * t), 0.0, 0.0}};
    };
}

YawFn no_turn() {
    return [](double t) {
        return std::tuple<double, double, double>{0.3 + 0.02 * std::sin(0.7 * t), 0.014 * std::cos(0.7 * t),
                                                  -0.0098 * std::sin(0.7 * t)};
    };
}

YawFn fast_turns() {
    // looking left and right: +-60 deg at 0.5 Hz (peak 3.3 rad/s, 190 deg/s)
    return [](double t) {
        const double w = 2.0 * PI * 0.5;
        const double a = rf::to_radians(60.0);
        return std::tuple<double, double, double>{a * std::sin(w * t), a * w * std::cos(w * t),
                                                  -a * w * w * std::sin(w * t)};
    };
}

NeckFn lean(double t0, double dur, double d, Vec3 dir, double y) {
    return [=](double t) {
        const auto [p, v, a] = smooth_move(t, t0, dur, d, dir);
        return std::tuple<Vec3, Vec3, Vec3>{geom::add(p, {0.0, y, 0.0}), v, a};
    };
}

/// Desk-still run: 3DoF orientation that shimmers by shim rad, then a pickup at pick_s.
std::pair<std::unique_ptr<HeadTracker>, Out> freeze_run(HeadParams p, double shim, double pick_s, double secs) {
    auto h = std::make_unique<HeadTracker>(SupervisorParams{}, std::move(p), Vec3{0.0, 0.0, 0.0});
    std::uint64_t seed = 5;
    Out out;
    for (std::int64_t i = 0; i < static_cast<std::int64_t>(secs * 1000.0); i++) {
        const double t = static_cast<double>(i) * 0.001;
        const double w = t >= pick_s ? 0.5 : 0.0;
        ImuSample s;
        s.ts_ns = i * 1'000'000;
        s.gyro[0] = 0.0004 * gauss(seed) + w;
        s.gyro[1] = 0.0004 * gauss(seed);
        s.gyro[2] = 0.0004 * gauss(seed);
        s.accel[0] = 0.005 * gauss(seed);
        s.accel[1] = GRAVITY + 0.005 * gauss(seed);
        s.accel[2] = 0.005 * gauss(seed);
        h->push_imu(s);
        h->update(i * 1'000'000, std::nullopt);
        if (i % 5 == 0) {
            const double a = shim * std::sin(t * 40.0);
            Rel r = rel3({0.0, std::sin(a / 2.0), 0.0, std::cos(a / 2.0)});
            r.ang_vel = {0.0, shim * 40.0 * std::cos(t * 40.0), 0.0};
            out.emplace_back(t, h->head(i * 1'000'000, i * 1'000'000, r));
        }
    }
    return {std::move(h), std::move(out)};
}

double angle_between(const Quat& a, const Quat& b) {
    return rf::to_degrees(geom::norm(qlog(qmul(a, qconj(b)))));
}

} // namespace

TEST(still_state_has_hysteresis) {
    HeadTracker h(SupervisorParams{}, HeadParams{}, {0.0, 0.0, 0.0});
    auto feed = [&](std::int64_t from, std::int64_t n, double amp) {
        for (std::int64_t i = from; i < from + n; i++) {
            const double w = i % 2 == 0 ? amp : -amp;
            ImuSample s;
            s.ts_ns = i * 1'000'000;
            s.gyro = {0.013, 0.0, 0.0};
            s.accel = {w, 9.8, 0.0};
            h.push_imu(s);
        }
        h.update((from + n) * 1'000'000, std::nullopt);
        return h.is_still();
    };
    CHECK(!feed(0, 300, 0.01));    // 0.3 s of desk quiet is not enough yet
    CHECK(feed(300, 700, 0.01));   // desk quiet enters
    CHECK(feed(1000, 600, 0.1));   // 0.1 m/s^2 vibration keeps it
    CHECK(!feed(1600, 600, 0.3));  // real motion ends it
    CHECK(!feed(2200, 2500, 0.1)); // 0.1 m/s^2 does not enter
}

TEST(worn_head_at_rest_never_counts_as_still_a_desk_does) {
    std::uint64_t seed = 7;
    HeadParams old;
    old.still_enter_window_ns = 500'000'000;
    old.still_gyro_std = 0.006;
    old.still_accel_std = 0.06;
    old.still_bias_tol = 0.0;
    // Worn, sitting still: 0.3 deg/s gyro std, 0.02 m/s^2.
    const auto worn = rest_trace(0.0, 8.0, 0.0053, 0.02, 0.0, seed);
    const auto [st_old, n_old] = still_after(old, worn, std::nullopt);
    (void)st_old;
    const auto [st, n_new] = still_after(HeadParams{}, worn, std::nullopt);
    CHECK(n_old > 0); // the old 0.34 deg/s gate took a worn head for still
    CHECK(!st && n_new == 0);
    // On the desk: bare sensor noise (0.057 deg/s, 0.012 m/s^2) -> still after 0.5 s.
    const auto desk = rest_trace(0.0, 4.0, 0.001, 0.012, 0.0, seed);
    CHECK(!still_after(HeadParams{}, desk, std::nullopt, 400).first); // not before 0.5 s
    CHECK(still_after(HeadParams{}, desk, std::nullopt).first);       // desk enters
    // Flat but turning slowly (0.4 deg/s over the bias): not still once the bias is known.
    auto turn = desk;
    for (auto& s : turn) {
        s.gyro[1] += 0.007;
    }
    const Vec3 bias{-0.0043, 0.0138, -0.0059};
    CHECK(!still_after(HeadParams{}, turn, bias).first); // slow steady turn
    CHECK(still_after(HeadParams{}, desk, bias).first);  // desk with a known bias
}

TEST(tilted_tracker_attitude_converges_to_gravity_yaw_untouched_rate_limited) {
    // 1.5 deg pitch + 0.8 deg roll of tracker attitude error (what Basalt showed in dark runs).
    const Quat err = qmul(qexp({rf::to_radians(1.5), 0.0, 0.0}), qexp({0.0, 0.0, rf::to_radians(0.8)}));
    auto e = [&](double, bool) { return std::pair<Quat, Vec3>{err, {0.3, 1.6, -0.2}}; };
    const auto on = run_tilt(HeadParams{}, 12.0, true, e);
    HeadParams poff;
    poff.tilt_tau_s = 0.0;
    const auto off = run_tilt(poff, 12.0, true, e);
    double t_first = 0;
    for (const auto& o : on.out) {
        if (tracked(o.second)) {
            t_first = o.first;
            break;
        }
    }
    const auto [t_last, last] = on.out.back();
    // The acquisition blends from the 3DoF attitude: compare at 1 s, when that has decayed.
    std::pair<double, Rel> r1{};
    for (const auto& o : off.out) {
        if (o.first > t_first + 1.0 && tracked(o.second)) {
            r1 = o;
            break;
        }
    }
    const double e_first = up_err_deg(r1.first, r1.second, true);
    const double e_last = up_err_deg(t_last, last, true);
    const double e_off = up_err_deg(t_last, off.out.back().second, true);
    std::fprintf(stderr, "tilt error: start %.3f deg, after 12 s %.3f deg (no correction %.3f deg), %llu updates\n",
                 e_first, e_last, e_off, static_cast<unsigned long long>(on.h->tilt_updates));
    CHECK(on.h->sup.resets == 0);
    CHECK(e_first > 1.4 && e_off > 1.4);
    CHECK(e_last < 0.1);
    // Yaw and position are the tracker's.
    for (std::size_t k = 0; k < std::min(on.out.size(), off.out.size()); k++) {
        const Rel& a = on.out[k].second;
        const Rel& b = off.out[k].second;
        if (!tracked(a)) {
            continue;
        }
        const Vec3 d = qlog(qmul(a.q, qconj(b.q)));
        CHECK(rf::to_degrees(std::fabs(d[1])) < 0.01);
        CHECK(geom::norm(geom::sub(a.pos, b.pos)) < 1e-9);
    }
    // Never faster than 0.5 deg/s: <= 0.02 deg per 40 ms pose.
    double max_step = 0.0;
    for (std::size_t k = 1; k < on.tilts.size(); k++) {
        max_step = rf::fmax(max_step, qangle_deg(qmul(on.tilts[k], qconj(on.tilts[k - 1]))));
    }
    CHECK(max_step <= 0.0201);
    CHECK(on.h->tilt_updates > 200);
    // Output frame to frame: no jump beyond the true motion.
    for (std::size_t k = 1; k < on.out.size(); k++) {
        const auto& [t0, a] = on.out[k - 1];
        const auto& [t1, b] = on.out[k];
        if (t0 > t_first + 0.5 && tracked(a)) {
            const double tr = qangle_deg(qmul(qconj(tilt_true_q(t0, true)), tilt_true_q(t1, true)));
            const double extra = qangle_deg(qmul(qconj(a.q), b.q)) - tr;
            CHECK(extra < 0.03);
        }
    }
}

TEST(tilt_correction_is_held_through_a_still_rezero) {
    // Tracker tilted 1.2 deg and drifting (no landmarks) while the headset lies still: it is re-zeroed; the fresh
    // tracker world starts gravity aligned. The output attitude must not swing at either end.
    const Quat err = qexp({rf::to_radians(1.2), 0.0, 0.0});
    auto e = [&](double t, bool reset) {
        if (reset) {
            return std::pair<Quat, Vec3>{QUAT_ID, {5.0, 0.0, 5.0}};
        }
        const double d = t > 8.0 ? 0.5 * 0.07 * (t - 8.0) * (t - 8.0) : 0.0;
        return std::pair<Quat, Vec3>{err, {d, 0.0, 0.0}};
    };
    const auto r = run_tilt(HeadParams{}, 14.0, false, e);
    CHECK(r.h->sup.still_resets == 1);
    CHECK(r.h->sup.losses == 0);
    const auto [t_last, last] = r.out.back();
    const double e_last = up_err_deg(t_last, last, false);
    double max_jump = 0.0;
    for (std::size_t k = 1; k < r.out.size(); k++) {
        const auto& [t0, a] = r.out[k - 1];
        const auto& [t1, b] = r.out[k];
        if (t0 < 2.0) {
            continue;
        }
        const double tr = qangle_deg(qmul(qconj(tilt_true_q(t0, false)), tilt_true_q(t1, false)));
        max_jump = rf::fmax(max_jump, qangle_deg(qmul(qconj(a.q), b.q)) - tr);
    }
    std::fprintf(stderr, "re-zero with tilt: max extra rotation per frame %.4f deg, end tilt error %.3f deg\n",
                 max_jump, e_last);
    CHECK(max_jump < 0.05);
    CHECK(e_last < 0.15);
}

TEST(noisy_tracker_velocity_lowers_the_fusion_weights) {
    const HeadParams p;
    auto mk = [&]() {
        PredictorParams pp;
        pp.fuse_alpha = p.fuse[0];
        pp.fuse_alpha_y = p.fuse[1];
        pp.fuse_beta = p.fuse[2];
        pp.fuse_vel_w = p.fuse[3];
        pp.fuse_fast_speed = p.fuse[4];
        pp.fuse_auto = true;
        return Predictor(pp);
    };
    auto run = [](Predictor& pr, double vel_noise) {
        std::uint64_t seed = 3;
        double next_vio = 0.1;
        double t = 0.0;
        while (t < 10.0) {
            pr.push_imu(imu_at(t, {0.0, 0.0, 0.0}));
            if (t >= next_vio) {
                const Truth x = truth(t);
                const double n0 = gauss(seed);
                const double n1 = gauss(seed);
                const double n2 = gauss(seed);
                pr.push_vio(BodyState{round_ns(t), x.p, x.q, geom::add(x.v, geom::scale({n0, n1, n2}, vel_noise))},
                            false);
                next_vio += 0.04;
            }
            t += 0.001;
        }
    };
    Predictor clean = mk();
    run(clean, 0.01);
    Predictor noisy = mk();
    run(noisy, 0.25);
    const auto wc = clean.fuse_weights(0.0);
    const auto wn = noisy.fuse_weights(0.0);
    CHECK(clean.noise_level() < 0.1);
    CHECK(std::fabs(wc[3] - 0.5) < 0.05 && std::fabs(wc[0] - 0.15) < 0.01);
    CHECK(noisy.noise_level() > 0.9);
    CHECK(std::fabs(wn[3] - 0.1) < 0.02 && wn[0] < 0.08 && wn[4] == 1.0);
    // In fast motion the position pull stays (no lag), only the velocity weight is lowered.
    CHECK(std::fabs(noisy.fuse_weights(1.0)[0] - 0.15) < 1e-9);
    // A dark image lowers them as well, even with a clean velocity.
    HeadTracker h(SupervisorParams{}, HeadParams{}, {0.0, 0.0, 0.0});
    h.pred = clean;
    h.set_scene_mean(40.0);
    CHECK(h.pred.fuse_weights(0.0)[3] > 0.45);
    h.set_scene_mean(12.0);
    CHECK(std::fabs(h.pred.fuse_weights(0.0)[3] - 0.1) < 0.01);
    h.set_scene_mean(std::nullopt);
    CHECK(h.pred.fuse_weights(0.0)[3] > 0.45);
    // Auto off = the configured weights whatever the noise.
    noisy.p.fuse_auto = false;
    CHECK(noisy.fuse_weights(0.0)[3] == 0.5);
}

TEST(noisy_tracker_velocity_jitters_less_with_auto_weights) {
    // The test walk (0.3 m/s) counted as slow motion (fast speed 2 m/s): where auto lowers the weights.
    const Vec3 head{0.0, -0.02, 0.07};
    std::vector<std::tuple<double, double, double>> res;
    for (const bool autow : {false, true}) {
        HeadParams p;
        p.fuse_auto = autow;
        p.fuse = {0.15, 0.075, 0.06, 0.5, 2.0};
        Sim sim(p);
        std::uint64_t seed = 5;
        VioFn vio = [&](double t) -> std::optional<std::tuple<Vec3, Quat, Vec3>> {
            auto s = slam_of(t, 0.4, {0.5, 0.0, -1.0});
            for (int k = 0; k < 3; k++) {
                std::get<2>(s)[static_cast<std::size_t>(k)] += 0.3 * gauss(seed);
            }
            return s;
        };
        Out out;
        sim.run(0.0, 10.0, 0.002, vio, out);
        res.push_back(errors(out, 2.0, head));
    }
    CHECK(std::get<2>(res[1]) < std::get<2>(res[0]));
}

TEST(describe_names_the_build_markers) {
    const std::string d = HeadParams{}.describe();
    CHECK(d.find("PIMAX_HEAD_TILT") != std::string::npos && d.find("PIMAX_FUSE_AUTO") != std::string::npos);
    CHECK(d.find("on (tau 1.5 s") != std::string::npos && d.find("0.20 deg/s, 0.5 s") != std::string::npos);
}

TEST(imu_still_detection) {
    Predictor p(PredictorParams{});
    std::uint64_t seed = 1;
    auto rnd = [&]() {
        seed = seed * 6364136223846793005ull + 1442695040888963407ull;
        return static_cast<double>(seed >> 11) / static_cast<double>(1ull << 53) - 0.5;
    };
    // 1 s lying still (gyro bias + 0.001 rad/s noise, accel 0.01 m/s^2), then 1 s of head sway.
    for (std::int64_t i = 0; i < 2000; i++) {
        const double amp = i < 1000 ? 1.0 : 20.0;
        ImuSample s;
        s.ts_ns = i * 1'000'000;
        s.gyro[0] = 0.013 + 0.003 * amp * rnd();
        s.gyro[1] = -0.004 + 0.003 * amp * rnd();
        s.gyro[2] = 0.003 * amp * rnd();
        s.accel[0] = 0.03 * amp * rnd();
        s.accel[1] = 9.8 + 0.03 * amp * rnd();
        s.accel[2] = 0.03 * amp * rnd();
        p.push_imu(s);
        if (i == 999) {
            CHECK(p.imu_still(500'000'000, 0.004, 0.03)); // still
        }
        if (i == 300) {
            CHECK(!p.imu_still(500'000'000, 0.004, 0.03)); // window not covered yet
        }
    }
    CHECK(!p.imu_still(500'000'000, 0.004, 0.03)); // moving
}

TEST(exp_log_roundtrip) {
    for (const Vec3& w : {Vec3{0.0, 0.0, 0.0}, Vec3{0.1, -0.2, 0.3}, Vec3{2.0, 1.0, -1.5}, Vec3{1e-9, 0.0, 0.0}}) {
        const Vec3 back = qlog(qexp(w));
        CHECK(geom::norm(geom::sub(back, w)) < 1e-9);
    }
}

TEST(predictor_tracks_true_motion_from_imu) {
    Predictor pr(PredictorParams{});
    for (int i = 0; i < 3000; i++) {
        pr.push_imu(imu_at(static_cast<double>(i) * 0.001, {0.0, 0.0, 0.0}));
    }
    // Base at 2.0 s from truth, predict 0..60 ms ahead within the IMU span and 20 ms past it.
    const Truth x = truth(2.0);
    const BodyState base{2'000'000'000, x.p, x.q, x.v};
    for (const double dt_ms : {10.0, 40.0, 60.0}) {
        const double t = 2.0 + dt_ms / 1000.0;
        const auto [st, w] = pr.propagate(base, static_cast<std::int64_t>(t * 1e9));
        (void)w;
        const Truth y = truth(t);
        const double ep = geom::norm(geom::sub(st.p, y.p));
        const double er = qangle_deg(qmul(qconj(st.q), y.q));
        CHECK(ep < 2e-4 && er < 0.02);
    }
    const auto [st, w] = pr.propagate(base, 3'020'000'000 - 1'000'000'000 + 1'000'000'000);
    (void)w;
    CHECK(st.ts_ns == 3'020'000'000);
}

TEST(gyro_bias_is_learned_from_the_tracker) {
    const Vec3 bias{0.01, -0.02, 0.015}; // ~1 deg/s
    PredictorParams pp;
    pp.bias_tau_s = 2.0;
    Predictor pr(pp);
    double t = 0.0;
    double next_vio = 0.1;
    while (t < 30.0) {
        pr.push_imu(imu_at(t, bias));
        if (t >= next_vio) {
            const Truth x = truth(t);
            pr.push_vio(BodyState{round_ns(t), x.p, x.q, x.v}, true);
            next_vio += 0.04;
        }
        t += 0.001;
    }
    const double e = geom::norm(geom::sub(pr.gyro_bias, bias));
    CHECK(e < 0.1 * geom::norm(bias));
    CHECK(pr.bias_samples > 500);
}

TEST(noisy_tracker_is_smoothed_without_lag) {
    const Vec3 head{0.0, -0.02, 0.07};
    Out raw_out, sm_out;
    for (const bool smooth : {false, true}) {
        HeadParams p;
        p.smooth = smooth;
        Sim sim(p);
        VioFn vio = [](double t) -> std::optional<std::tuple<Vec3, Quat, Vec3>> {
            return slam_of(t, 0.7, {1.0, 0.0, -2.0});
        };
        sim.run(0.0, 8.0, 0.002, vio, smooth ? sm_out : raw_out);
        CHECK(sim.h->output().state == TrackState::Tracking);
    }
    const auto [rp, rr, rj] = errors(raw_out, 1.0, head);
    (void)rr;
    const auto [sp, sr, sj] = errors(sm_out, 1.0, head);
    std::fprintf(stderr, "noise 2 mm: raw max err %.4f m, max step err %.4f; smoothed %.4f m, %.4f, rot %.3f deg\n",
                 rp, rj, sp, sj, sr);
    CHECK(sj < 0.6 * rj);
    CHECK(sp < 0.008);
    CHECK(sr < 0.5);
}

TEST(tracker_correction_slides_instead_of_jumping) {
    const Vec3 head{0.0, -0.02, 0.07};
    Sim sim(HeadParams{});
    Out out;
    // At 3 s the tracker corrects itself by 6 cm (e.g. after re-observing old features).
    VioFn vio = [](double t) -> std::optional<std::tuple<Vec3, Quat, Vec3>> {
        auto s = slam_of(t, 0.0, {0.0, 0.0, 0.0});
        if (t >= 3.0) {
            std::get<0>(s)[0] += 0.06;
        }
        return s;
    };
    sim.run(0.0, 6.0, 0.0, vio, out);
    // Largest step between consecutive outputs beyond the true motion: bounded by the rate limit.
    double max_step = 0.0;
    for (std::size_t k = 1; k < out.size(); k++) {
        const auto& [t0, a] = out[k - 1];
        const auto& [t1, b] = out[k];
        if (!tracked(a) || t0 < 1.0) {
            continue;
        }
        const auto [p0, q0] = head_truth(t0, head);
        const auto [p1, q1] = head_truth(t1, head);
        (void)q0;
        (void)q1;
        const double extra = geom::norm(geom::sub(geom::sub(b.pos, a.pos), geom::sub(p1, p0)));
        max_step = rf::fmax(max_step, extra);
    }
    CHECK(max_step < 0.006);
    // The output ends on the corrected track (the shift is in the tracker world, so it stays).
    const auto [t_end, last] = out.back();
    const Vec3 want = expected(ref_at(out, 2.5), t_end, head, {0.06, 0.0, 0.0});
    CHECK(geom::norm(geom::sub(last.pos, want)) < 0.003);
    // With the fused position (default) the 6 cm arrive over several poses, each a small step.
    CHECK(sim.h->corr.blended > 0 && sim.h->corr.max_pos < 0.05);
}

TEST(loss_then_same_world_resnaps_the_room) {
    const Vec3 head{0.0, -0.02, 0.07};
    Sim sim(HeadParams{});
    Out out;
    // Tracker silent from 3.0 to 4.5 s (head keeps walking ~0.4 m), then back in the SAME world.
    VioFn vio = [](double t) -> std::optional<std::tuple<Vec3, Quat, Vec3>> {
        if (t >= 3.0 && t < 4.5) {
            return std::nullopt;
        }
        return slam_of(t, 0.3, {0.5, 0.0, 0.5});
    };
    sim.run(0.0, 9.0, 0.0, vio, out);
    CHECK(sim.h->sup.losses == 1);
    CHECK(sim.h->sup.acquisitions == 2);
    CHECK(std::string(sim.h->output().reason).find("restored") != std::string::npos);
    // After the slide the output is back on the true room track (as if never lost).
    const auto [t_end, last] = out.back();
    const Vec3 want = expected(ref_at(out, 2.5), t_end, head, {0.0, 0.0, 0.0});
    CHECK(geom::norm(geom::sub(last.pos, want)) < 0.01);
    // No visible jump at loss or re-acquisition.
    for (std::size_t k = 1; k < out.size(); k++) {
        const auto& [t0, a] = out[k - 1];
        const auto& [t1, b] = out[k];
        if (t0 < 1.0) {
            continue;
        }
        const double dq = qangle_deg(qmul(qconj(a.q), b.q));
        const auto qa = head_truth(t0, head).second;
        const auto qb = head_truth(t1, head).second;
        const double true_dq = qangle_deg(qmul(qconj(qa), qb));
        CHECK(std::fabs(dq - true_dq) < 0.5);
        CHECK(geom::norm(geom::sub(b.pos, a.pos)) < 0.012);
    }
}

TEST(reset_world_realigns_continuously) {
    Sim sim(HeadParams{});
    Out out;
    // Tracker comes back in a NEW world (other yaw and origin): no re-snap, continuous re-alignment.
    VioFn vio = [](double t) -> std::optional<std::tuple<Vec3, Quat, Vec3>> {
        if (t >= 3.0 && t < 4.0) {
            return std::nullopt;
        }
        if (t < 3.0) {
            return slam_of(t, 0.0, {0.0, 0.0, 0.0});
        }
        return slam_of(t, 1.2, {3.0, 0.0, 1.0});
    };
    sim.run(0.0, 7.0, 0.0, vio, out);
    CHECK(sim.h->sup.acquisitions == 2);
    CHECK(std::string(sim.h->output().reason).find("restored") == std::string::npos);
    for (std::size_t k = 1; k < out.size(); k++) {
        if (out[k - 1].first < 1.0) {
            continue;
        }
        CHECK(geom::norm(geom::sub(out[k].second.pos, out[k - 1].second.pos)) < 0.012);
    }
}

TEST(drift_hold_keeps_a_quiet_seated_head_in_place) {
    const Body body{seated(), no_turn()};
    // Basalt's slide of 2026-10-18 12:05:31: 2.6 cm/s sideways, a little up.
    auto drift = [](double t) {
        return t > 5.0 ? Vec3{0.026 * (t - 5.0), 0.004 * (t - 5.0), 0.0} : Vec3{0.0, 0.0, 0.0};
    };
    auto [on, h] = gov_run(HeadParams{}, body, 26.0, drift);
    auto [off, h2] = gov_run(drift_off(), body, 26.0, drift);
    (void)h2;
    const auto [d_on, d_true] = gov_disp(on, 6.0, 26.0);
    const auto [d_off, d_true2] = gov_disp(off, 6.0, 26.0);
    (void)d_true2;
    const double e_on = geom::norm(geom::sub(d_on, d_true));
    const double e_off = geom::norm(geom::sub(d_off, d_true));
    std::fprintf(stderr, "quiet seated: output slid %.1f cm with the hold, %.1f cm without; held %.2f m\n",
                 e_on * 100.0, e_off * 100.0, h->drift.stats.held_m);
    CHECK(e_off > 0.45);
    CHECK(e_on < 0.03);
    CHECK(h->drift.stats.held_m > 0.4);
}

TEST(drift_hold_bounds_a_seated_head_turning_fast_and_keeps_the_arc) {
    const Body body{seated(), fast_turns()};
    // 1.2 m right and 0.84 m up in 70 s, as in the 12:00:28 fast turns: in bursts while turning.
    auto drift = [](double t) {
        const double s = t > 4.0 ? t - 4.0 + 0.3 * std::sin(2.0 * PI * 0.5 * t) : 0.0;
        return Vec3{0.017 * s, 0.012 * s, 0.0};
    };
    auto [on, h] = gov_run(HeadParams{}, body, 34.0, drift);
    auto [off, h2] = gov_run(drift_off(), body, 34.0, drift);
    (void)h;
    (void)h2;
    const auto [d_on, d_true] = gov_disp(on, 4.0, 34.0);
    const auto [d_off, d_true2] = gov_disp(off, 4.0, 34.0);
    (void)d_true2;
    const double e_on = geom::norm(geom::sub(d_on, d_true));
    const double e_off = geom::norm(geom::sub(d_off, d_true));
    // The head still swings on its arc around the neck.
    double zo_max = std::numeric_limits<double>::lowest(), zo_min = std::numeric_limits<double>::max();
    double zt_max = zo_max, zt_min = zo_min;
    for (const auto& o : on) {
        if (std::get<0>(o) > 30.0) {
            zo_max = rf::fmax(zo_max, std::get<1>(o)[2]);
            zo_min = rf::fmin(zo_min, std::get<1>(o)[2]);
            zt_max = rf::fmax(zt_max, std::get<2>(o)[2]);
            zt_min = rf::fmin(zt_min, std::get<2>(o)[2]);
        }
    }
    const double swing_out = zo_max - zo_min;
    const double swing_true = zt_max - zt_min;
    std::fprintf(stderr, "fast turns: output slid %.1f cm with the hold, %.1f cm without; arc %.1f cm (true %.1f)\n",
                 e_on * 100.0, e_off * 100.0, swing_out * 100.0, swing_true * 100.0);
    CHECK(e_off > 0.55);
    CHECK(e_on < 0.08);
    CHECK(std::fabs(swing_out - swing_true) < 0.03);
}

TEST(drift_hold_lets_real_leans_walks_and_standing_up_through) {
    struct Case {
        const char* name;
        NeckFn neck;
        double t0, t1, keep;
    };
    const double ws = 2.0 * PI * 1.8;
    std::vector<Case> cases = {
        {"lean forward 20 cm in 1 s", lean(3.0, 1.0, 0.20, {0.0, 0.0, -1.0}, 1.2), 3.0, 5.0, 0.9},
        {"lean sideways 25 cm in 1.5 s", lean(3.0, 1.5, 0.25, {1.0, 0.0, 0.0}, 1.2), 3.0, 5.5, 0.85},
        {"stand up 45 cm in 1.2 s", lean(3.0, 1.2, 0.45, {0.0, 1.0, 0.0}, 1.2), 3.0, 5.0, 0.9},
        {"walk 2.1 m in 3 s (head bob)",
         [ws](double t) {
             const auto [p, v, a] = smooth_move(t, 3.0, 3.0, 2.1, {0.0, 0.0, -1.0});
             const bool on = t >= 3.0 && t < 6.0;
             const double b = on ? 0.015 : 0.0;
             return std::tuple<Vec3, Vec3, Vec3>{geom::add(p, {0.0, 1.6 + b * std::sin(ws * t), 0.0}),
                                                 geom::add(v, {0.0, b * ws * std::cos(ws * t), 0.0}),
                                                 geom::add(a, {0.0, -b * ws * ws * std::sin(ws * t), 0.0})};
         },
         2.5, 7.0, 0.95},
    };
    for (const auto& c : cases) {
        const Body body{c.neck, no_turn()};
        auto [out, h] = gov_run(HeadParams{}, body, c.t1 + 0.5, [](double) { return Vec3{0.0, 0.0, 0.0}; });
        const auto [d_out, d_true] = gov_disp(out, c.t0, c.t1);
        const double kept = geom::dot(d_out, d_true) / geom::dot(d_true, d_true);
        std::fprintf(stderr, "%s: kept %.0f %%, held %.1f cm\n", c.name, kept * 100.0, h->drift.stats.held_m * 100.0);
        CHECK(kept > c.keep && kept < 1.05);
    }
    // A slow lean (10 cm in 2 s) is the hard case: report how much passes.
    const Body body{lean(3.0, 2.0, 0.10, {0.0, 0.0, -1.0}, 1.2), no_turn()};
    auto [out, h] = gov_run(HeadParams{}, body, 6.0, [](double) { return Vec3{0.0, 0.0, 0.0}; });
    (void)h;
    const auto [d_out, d_true] = gov_disp(out, 3.0, 5.5);
    std::fprintf(stderr, "slow lean 10 cm in 2 s: output moved %.1f cm of %.1f cm\n", geom::norm(d_out) * 100.0,
                 geom::norm(d_true) * 100.0);
}

TEST(exact_freeze_holds_position_and_orientation_while_still) {
    auto [h, out] = freeze_run(HeadParams{}, 0.0003, 99.0, 3.0);
    CHECK(h->is_frozen() && h->freeze_frames > 100);
    std::vector<Rel> late;
    for (const auto& o : out) {
        if (o.first > 1.5) {
            late.push_back(o.second);
        }
    }
    const Rel first = late[0];
    for (const auto& r : late) {
        CHECK(r.q == first.q); // orientation is one fixed value
        CHECK(r.pos == first.pos);
        CHECK(r.ang_vel == (Vec3{0.0, 0.0, 0.0}));
        CHECK(r.lin_vel == (Vec3{0.0, 0.0, 0.0}));
    }
    // Switched off, the same input shimmers.
    HeadParams pf;
    pf.freeze = false;
    auto [h2, out2] = freeze_run(pf, 0.0003, 99.0, 3.0);
    CHECK(!h2->is_frozen());
    std::vector<Rel> late2;
    for (const auto& o : out2) {
        if (o.first > 1.5) {
            late2.push_back(o.second);
        }
    }
    bool shimmers = false;
    for (const auto& r : late2) {
        shimmers = shimmers || r.q != late2[0].q;
    }
    CHECK(shimmers);
}

TEST(exact_freeze_ends_on_pickup_without_a_jump) {
    auto [h, out] = freeze_run(HeadParams{}, 0.0003, 2.0, 2.6);
    CHECK(!h->is_frozen()); // a pickup (0.5 rad/s) ends the freeze
    for (std::size_t k = 1; k < out.size(); k++) {
        if (out[k - 1].first <= 1.9) {
            continue;
        }
        const double d = angle_between(out[k].second.q, out[k - 1].second.q);
        CHECK(d < 1.0);
        CHECK(geom::norm(geom::sub(out[k].second.pos, out[k - 1].second.pos)) < 0.01);
    }
}

TEST(noise_regime_follows_the_gyro_rate) {
    auto [h, o] = freeze_run(HeadParams{}, 0.0, 99.0, 1.0);
    (void)o;
    CHECK(h->noise_regime() == slam::NoiseRegime::Still);
    auto [h2, o2] = freeze_run(HeadParams{}, 0.0, 0.5, 1.0);
    (void)o2;
    CHECK(h2->noise_regime() == slam::NoiseRegime::Moving);
}

PIBRIDGE_TEST_MAIN()
