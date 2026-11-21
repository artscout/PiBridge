// SPDX-License-Identifier: MIT
// PiBridge: the sliding-window estimator (window.h) on a synthetic controller: a known trajectory, its IMU steps and
// noisy LED pixels; the window must follow it closely and stay put when the controller is still.
#include "pibridge/ctrl/track/window.h"
#include "pibridge_test.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>

using namespace pibridge::ctrl::track;
namespace g = pibridge::cam::geom;

namespace {

constexpr double G = 9.80665;

Cam make_cam(const g::Vec3& t_body_cam, double yaw_deg) {
    Cam c;
    c.intr = pibridge::cam::Kb4::make(640, 480, 234.7, 234.7, 320.0, 240.0, {0.06, -0.06, 0.03, -0.007}, {});
    // body_from_cam: the camera at t_body_cam looking along -z of the body turned by yaw
    const double a = yaw_deg * M_PI / 180.0;
    const g::Mat3 ry{{{std::cos(a), 0.0, std::sin(a)}, {0.0, 1.0, 0.0}, {-std::sin(a), 0.0, std::cos(a)}}};
    // camera z forward = body -z: rotate 180 deg about y first
    const g::Mat3 flip{{{-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, -1.0}}};
    const g::Pose body_from_cam{g::mat_mul(ry, flip), t_body_cam};
    c.cam_from_body = body_from_cam.inverse();
    return c;
}

/// 12 LEDs on a ring of radius 4 cm around the device y axis, normals outward.
std::vector<std::pair<g::Vec3, g::Vec3>> ring() {
    std::vector<std::pair<g::Vec3, g::Vec3>> l;
    for (int i = 0; i < 12; i++) {
        const double a = i * 2.0 * M_PI / 12.0;
        const g::Vec3 n{std::cos(a), 0.0, std::sin(a)};
        l.emplace_back(g::Vec3{0.04 * n[0], 0.01 * (i % 2), 0.04 * n[2]}, n);
    }
    return l;
}

struct Sim {
    std::vector<Cam> cams;
    std::vector<std::pair<g::Vec3, g::Vec3>> leds = ring();
    std::mt19937 rng{7};
    double px_noise = 0.5;
    g::Pose body_from_world{g::IDENTITY, {0.0, 0.0, 0.0}}; // the head at the origin

    Sim() {
        cams.push_back(make_cam({-0.05, 0.0, 0.0}, 0.0));
        cams.push_back(make_cam({0.05, 0.0, 0.0}, 0.0));
    }
    /// truth at t: a circle of radius 0.2 in front of the head at 1 Hz (or still when omega 0), turning about y
    void truth(double t, double omega, g::Mat3& r, g::Vec3& p, g::Vec3& v, g::Vec3& a, g::Vec3& w) const {
        const double ph = omega * t;
        p = {0.2 * std::cos(ph), -0.2 + 0.2 * std::sin(ph), -0.6};
        v = {-0.2 * omega * std::sin(ph), 0.2 * omega * std::cos(ph), 0.0};
        a = {-0.2 * omega * omega * std::cos(ph), -0.2 * omega * omega * std::sin(ph), 0.0};
        const double yaw = 0.5 * std::sin(ph);
        r = {{{std::cos(yaw), 0.0, std::sin(yaw)}, {0.0, 1.0, 0.0}, {-std::sin(yaw), 0.0, std::cos(yaw)}}};
        w = {0.0, 0.5 * omega * std::cos(ph), 0.0}; // world = device y axis here
    }
    std::vector<ImuStep> imu(double t0, double t1, double omega) const {
        std::vector<ImuStep> out;
        const int n = 10;
        for (int k = 0; k < n; k++) {
            const double ta = t0 + (t1 - t0) * k / n, tb = t0 + (t1 - t0) * (k + 1) / n;
            g::Mat3 ra, rb;
            g::Vec3 p, v, a, w, pb, vb, ab, wb;
            truth(ta, omega, ra, p, v, a, w);
            truth(tb, omega, rb, pb, vb, ab, wb);
            const g::Vec3 am = g::scale(g::add(a, ab), 0.5);
            // specific force in the device frame: R^T (a - g), g = (0, -G, 0)
            const g::Mat3 rm = ra;
            ImuStep s;
            s.dt = tb - ta;
            s.dtheta = rotvec(g::mat_mul(g::transpose(ra), rb));
            s.f = g::mat_vec(g::transpose(rm), g::sub(am, g::Vec3{0.0, -G, 0.0}));
            out.push_back(s);
        }
        return out;
    }
    std::vector<VisObs> see(const g::Mat3& r, const g::Vec3& p) {
        std::vector<VisObs> o;
        std::normal_distribution<double> nd(0.0, px_noise);
        for (const Cam& c : cams) {
            for (const auto& [lp, ln] : leds) {
                const g::Vec3 pw = g::add(g::mat_vec(r, lp), p);
                const g::Vec3 pb = body_from_world.apply(pw);
                const g::Vec3 pc = c.cam_from_body.apply(pb);
                const g::Vec3 nc = g::mat_vec(c.cam_from_body.r, g::mat_vec(r, ln));
                if (pc[2] <= 0.05 || g::dot(nc, g::scale(pc, -1.0 / g::norm(pc))) < 0.2) {
                    continue;
                }
                const auto uv = c.intr.project(pc);
                if (!uv) {
                    continue;
                }
                o.push_back(VisObs{&c, lp, uv->first + nd(rng), uv->second + nd(rng)});
            }
        }
        return o;
    }
};

} // namespace

TEST(imu_factor_is_zero_at_the_truth) {
    Sim sim;
    const double omega = 2.0 * M_PI * 0.5;
    Window w;
    WinState a, b;
    g::Vec3 acc, wv;
    sim.truth(1.0, omega, a.r, a.p, a.v, acc, wv);
    sim.truth(1.04, omega, b.r, b.p, b.v, acc, wv);
    const auto [r, pd] = w.imu_residual_debug(a, b, sim.imu(1.0, 1.04, omega));
    std::fprintf(stderr, "imu residual at truth: theta (%.2e %.2e %.2e) v (%.2e %.2e %.2e) p (%.2e %.2e %.2e)\n", r[0], r[1],
                 r[2], r[3], r[4], r[5], r[6], r[7], r[8]);
    std::fprintf(stderr, "preint sigma: theta %.2e v %.2e p %.2e\n", std::sqrt(pd[0]), std::sqrt(pd[3]), std::sqrt(pd[6]));
    for (int i = 0; i < 3; i++) {
        CHECK(std::fabs(r[i]) < 1e-4);
        CHECK(std::fabs(r[3 + i]) < 2e-3);
        CHECK(std::fabs(r[6 + i]) < 1e-4);
    }
}

TEST(visual_only_window_converges) {
    // no IMU steps (no IMU link): the pixels alone must place a still controller to well under a millimetre
    Sim sim;
    sim.px_noise = 0.0;
    Window w;
    g::Mat3 r;
    g::Vec3 p, v, a, wv;
    sim.truth(0.0, 0.0, r, p, v, a, wv);
    WinState s0;
    s0.r = r;
    s0.p = g::add(p, g::Vec3{0.01, -0.01, 0.02});
    w.reset(s0);
    for (int k = 1; k <= 12; k++) {
        w.push(static_cast<std::uint64_t>(k * 0.04 * 1e9), sim.body_from_world, sim.see(r, p), {}, false, false, false);
    }
    const double e = g::norm(g::sub(w.newest().p, p));
    std::fprintf(stderr, "visual only: error %.3f mm after 12 frames, rms %.3f px\n", 1000.0 * e, w.last_vis_rms_px);
    CHECK(e < 0.0005);
}

TEST(follows_a_moving_controller) {
    Sim sim;
    const double omega = 2.0 * M_PI * 0.5; // 0.5 Hz circle: ~0.6 m/s, 2 m/s^2
    Window w;
    g::Mat3 r;
    g::Vec3 p, v, a, wv;
    sim.truth(0.0, omega, r, p, v, a, wv);
    WinState s0;
    s0.t = 0;
    s0.r = r;
    s0.p = g::add(p, g::Vec3{0.003, -0.002, 0.004});
    s0.v = v;
    w.reset(s0);
    double worst = 0.0, sum = 0.0;
    int n = 0;
    for (int k = 1; k <= 60; k++) {
        const double t = k * 0.04;
        const std::uint64_t tn = static_cast<std::uint64_t>(t * 1e9);
        sim.truth(t, omega, r, p, v, a, wv);
        w.push(tn, sim.body_from_world, sim.see(r, p), sim.imu(t - 0.04, t, omega), false, false, false);
        const double e = g::norm(g::sub(w.newest().p, p));
        if (k > 5) {
            worst = std::max(worst, e);
            sum += e;
            n++;
        }
    }
    std::fprintf(stderr, "moving: position error mean %.2f mm worst %.2f mm, frames %zu, rms %.2f px\n",
                 1000.0 * sum / n, 1000.0 * worst, w.frames(), w.last_vis_rms_px);
    CHECK(w.frames() == 10);
    CHECK(sum / n < 0.004);
    CHECK(worst < 0.012);
    CHECK(w.last_vis_rms_px < 1.5);
}

TEST(a_still_controller_stays_put) {
    Sim sim;
    Window w;
    if (const char* e = std::getenv("WT_MAXF")) {
        w.cfg.max_frames = static_cast<std::size_t>(std::atoi(e));
    }
    const bool stat = std::getenv("WT_NOSTAT") == nullptr;
    if (const char* e = std::getenv("WT_NOISE")) {
        sim.px_noise = std::atof(e);
    }
    if (const char* e = std::getenv("WT_SA")) {
        w.cfg.sigma_a *= std::atof(e);
        w.cfg.sigma_g *= std::atof(e);
    }
    if (const char* e = std::getenv("WT_HUB")) {
        w.cfg.huber = std::atof(e);
    }
    const bool no_imu = std::getenv("WT_NOIMU") != nullptr;
    g::Mat3 r;
    g::Vec3 p, v, a, wv;
    sim.truth(0.0, 0.0, r, p, v, a, wv);
    WinState s0;
    s0.r = r;
    s0.p = p;
    w.reset(s0);
    double worst = 0.0;
    std::vector<g::Vec3> ps;
    for (int k = 1; k <= 60; k++) {
        const double t = k * 0.04;
        const std::uint64_t tn = static_cast<std::uint64_t>(t * 1e9);
        // still: stationary factors on from frame 10
        w.push(tn, sim.body_from_world, sim.see(r, p), no_imu ? std::vector<ImuStep>{} : sim.imu(t - 0.04, t, 0.0),
               stat && k > 10, stat && k > 10, false);
        if (std::getenv("WT_TRACE") != nullptr) {
            const auto& nw = w.newest();
            std::fprintf(stderr, "k %d err %.3f mm v %.4f frames %zu cost %.2f rms %.3f accepted %zu\n", k,
                         1000.0 * g::norm(g::sub(nw.p, p)), g::norm(nw.v), w.frames(), w.last_cost, w.last_vis_rms_px,
                         w.last_accepted);
        }
        if (k > 20) {
            worst = std::max(worst, g::norm(g::sub(w.newest().p, p)));
            ps.push_back(w.newest().p);
        }
    }
    double jitter = 0.0;
    for (std::size_t i = 1; i < ps.size(); i++) {
        jitter = std::max(jitter, g::norm(g::sub(ps[i], ps[i - 1])));
    }
    // the same without the IMU (each frame on its own pixels): the window must average the depth noise of this
    // weakly conditioned scene (a 4 cm ring at 0.6 m, two cameras 10 cm apart) well below that
    Window w1;
    w1.reset(s0);
    sim.rng.seed(7);
    double worst1 = 0.0, jitter1 = 0.0;
    std::optional<g::Vec3> prev;
    for (int k = 1; k <= 60; k++) {
        w1.push(static_cast<std::uint64_t>(k * 0.04 * 1e9), sim.body_from_world, sim.see(r, p), {}, false, false, false);
        if (k > 20) {
            worst1 = std::max(worst1, g::norm(g::sub(w1.newest().p, p)));
            if (prev) {
                jitter1 = std::max(jitter1, g::norm(g::sub(w1.newest().p, *prev)));
            }
            prev = w1.newest().p;
        }
    }
    std::fprintf(stderr, "still: worst error %.2f mm, largest frame-to-frame step %.3f mm (pixels alone: %.2f / %.3f mm)\n",
                 1000.0 * worst, 1000.0 * jitter, 1000.0 * worst1, 1000.0 * jitter1);
    CHECK(worst < 0.5 * worst1);
    CHECK(jitter < 0.25 * jitter1);
    CHECK(worst < 0.008);
}

PIBRIDGE_TEST_MAIN()
