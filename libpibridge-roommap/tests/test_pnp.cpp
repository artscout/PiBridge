// SPDX-License-Identifier: MIT
// Twins of omarchyvr-room pnp.rs `tests` (21 tests; the #[ignore] `ransac_many_seeds` stress run is left out). The
// Rust tests use the lab copy of the Dream Air calibration when present, else the default search (the reference);
// these use the shipped reference (devices/dream-air/reference, PIBRIDGE_REFERENCE_XML).
#include "pibridge/cam/calib.h"
#include "pibridge/cam/slam.h"
#include "pibridge/roommap/pnp.h"
#include "pibridge_test.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <string>
#include <vector>

using namespace pibridge;
using namespace pibridge::roommap::pnp;
using namespace pibridge::roommap::pnp::detail;
namespace g = cam::geom;
namespace sl = cam::slam;

namespace {

constexpr double PI = std::numbers::pi;
const double ROOM[3] = {6.0, 3.0, 6.0};
constexpr double NOISE_PX = 0.5;

Mat3 orthonormalize(const Mat3& m) {
    auto col = [&](int j) { return Vec3{m[0][j], m[1][j], m[2][j]}; };
    const Vec3 c0 = g::normalize(col(0));
    const Vec3 c1 = g::normalize(g::sub(col(1), g::scale(c0, g::dot(c0, col(1)))));
    const Vec3 c2 = g::cross(c0, c1);
    return {{{c0[0], c1[0], c2[0]}, {c0[1], c1[1], c2[1]}, {c0[2], c1[2], c2[2]}}};
}

struct Rig {
    cam::DeviceCalib cal;
    std::vector<Pose> body_from_cam;
};

Rig rig() {
    auto cal = cam::DeviceCalib::load(PIBRIDGE_REFERENCE_XML);
    Rig r{*cal, {}};
    for (std::size_t i = 0; i < 4; i++) {
        auto p = sl::t_imu_cam(r.cal, i);
        r.body_from_cam.push_back(Pose{orthonormalize(p->r), p->t});
    }
    return r;
}

struct Truth {
    Pose4 pose;
    Mat3 r_tilt;
    Pose map_from_body() const { return pose.to_pose(r_tilt); }
};

double uni(SplitMix64& rng, double lo, double hi) {
    return lo + (hi - lo) * rng.unit();
}

double gauss(SplitMix64& rng) {
    const double u1 = 1.0 - rng.unit();
    const double u2 = rng.unit();
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * PI * u2);
}

Truth random_truth(SplitMix64& rng) {
    const double yaw = uni(rng, -PI, PI);
    const double tilt = uni(rng, 0.0, 30.0 * PI / 180.0);
    const double az = uni(rng, 0.0, 2.0 * PI);
    const Mat3 r_tilt = g::rodrigues(g::scale({std::cos(az), 0.0, std::sin(az)}, tilt));
    const double t0 = uni(rng, -2.5, 2.5);
    const double t1 = uni(rng, 1.0, 2.0);
    const double t2 = uni(rng, -2.5, 2.5);
    return Truth{Pose4{yaw, {t0, t1, t2}}, r_tilt};
}

Vec3 room_point(SplitMix64& rng) {
    const double x = uni(rng, -ROOM[0] / 2.0, ROOM[0] / 2.0);
    const double y = uni(rng, 0.0, ROOM[1]);
    const double z = uni(rng, -ROOM[2] / 2.0, ROOM[2] / 2.0);
    return {x, y, z};
}

Vec3 camera_point(const Rig& rig, const Truth& truth, std::size_t cam, const Vec3& x) {
    return rig.body_from_cam[cam].inverse().apply(truth.map_from_body().inverse().apply(x));
}

Corr exact(const Rig& rig, const Truth& truth, std::size_t cam, const Vec3& x) {
    return Corr{cam, g::normalize(camera_point(rig, truth, cam, x)), x};
}

std::optional<Corr> observe(SplitMix64& rng, const Rig& rig, const Truth& truth, std::size_t cam, const Vec3& x,
                            double noise_px) {
    const Vec3 xc = camera_point(rig, truth, cam, x);
    if (!(xc[2] >= 0.3 && xc[2] <= 6.0)) {
        return std::nullopt;
    }
    const auto& k = rig.cal.cameras[cam].intr;
    const auto uv = k.project(xc);
    if (!uv) {
        return std::nullopt;
    }
    const auto [u, v] = *uv;
    if (u < 0.0 || v < 0.0 || u >= static_cast<double>(k.width) || v >= static_cast<double>(k.height)) {
        return std::nullopt;
    }
    const double nu = u + noise_px * gauss(rng);
    const double nv = v + noise_px * gauss(rng);
    const auto bearing = k.unproject(nu, nv);
    if (!bearing) {
        return std::nullopt;
    }
    if (noise_px == 0.0 && g::norm(g::cross(*bearing, g::normalize(xc))) > 1e-10) {
        return std::nullopt;
    }
    return Corr{cam, *bearing, x};
}

Corr visible(SplitMix64& rng, const Rig& rig, const Truth& truth, const std::vector<std::size_t>& cams, double noise) {
    for (int i = 0; i < 1000000; i++) {
        const std::size_t cam = cams[rng.below(cams.size())];
        const Vec3 x = room_point(rng);
        if (auto c = observe(rng, rig, truth, cam, x, noise)) {
            return *c;
        }
    }
    std::abort();
}

Corr outlier(SplitMix64& rng, const Rig& rig, const Truth& truth, const std::vector<std::size_t>& cams) {
    for (;;) {
        const std::size_t cam = cams[rng.below(cams.size())];
        const auto& k = rig.cal.cameras[cam].intr;
        const double u = uni(rng, 0.0, static_cast<double>(k.width));
        const double v = uni(rng, 0.0, static_cast<double>(k.height));
        const auto bearing = k.unproject(u, v);
        if (!bearing) {
            continue;
        }
        const Corr c{cam, *bearing, room_point(rng)};
        if (residual_deg(c, rig.body_from_cam, truth.r_tilt, truth.pose) > 3.0) {
            return c;
        }
    }
}

Corr peripheral(SplitMix64& rng, const Rig& rig, const Truth& truth) {
    for (int i = 0; i < 1000000; i++) {
        const std::size_t cam = rng.below(4);
        const auto& k = rig.cal.cameras[cam].intr;
        const double u = uni(rng, 0.0, static_cast<double>(k.width));
        const double v = uni(rng, 0.0, static_cast<double>(k.height));
        const auto f = k.unproject(u, v);
        if (!f) {
            continue;
        }
        if ((*f)[2] > -0.005) {
            continue;
        }
        const Vec3 xc = g::scale(*f, uni(rng, 0.5, 4.0));
        const Vec3 x = truth.map_from_body().apply(rig.body_from_cam[cam].apply(xc));
        return Corr{cam, *f, x};
    }
    std::abort();
}

double yaw_err_deg(double a, double b) {
    return std::fabs(wrap_pi(a - b)) * 180.0 / PI;
}

double pos_err(const Vec3& a, const Vec3& b) {
    return g::norm(g::sub(a, b));
}

bool is_truth(const Pose4& s, const Truth& truth, double yaw_tol_deg, double t_tol) {
    return yaw_err_deg(s.yaw, truth.pose.yaw) < yaw_tol_deg && pos_err(s.t, truth.pose.t) < t_tol;
}

Corr with_cam(Corr c, std::size_t cam) {
    c.cam = cam;
    return c;
}
Corr with_bearing(Corr c, const Vec3& b) {
    c.bearing = b;
    return c;
}
Corr with_point(Corr c, const Vec3& p) {
    c.point = p;
    return c;
}

void assert_valid_solution(const Rig& rig, const Mat3& r_tilt, const Pose4& s, const std::array<const Corr*, 2>& pair) {
    const Pose pose = s.to_pose(r_tilt);
    for (const Corr* c : pair) {
        const double r = residual_deg(*c, rig.body_from_cam, r_tilt, s);
        CHECK(r < 1e-6);
        const Vec3 xc = rig.body_from_cam[c->cam].inverse().apply(pose.inverse().apply(c->point));
        CHECK(g::dot(xc, c->bearing) > 0.0);
    }
}

bool pair_is_degenerate(const Rig& rig, const Truth& truth, const Corr& a, const Corr& b) {
    const auto ra = *Ray::make(a, rig.body_from_cam, truth.r_tilt);
    const auto rb = *Ray::make(b, rig.body_from_cam, truth.r_tilt);
    const Vec3 n = g::cross(ra.d, rb.d);
    const Vec3 dx = g::sub(a.point, b.point);
    const Vec3 m = g::normalize(n);
    const double horiz = std::sqrt(m[0] * m[0] + m[2] * m[2]) * std::sqrt(dx[0] * dx[0] + dx[2] * dx[2]);
    const double gamma = g::dot(m, g::sub(ra.p, rb.p)) - m[1] * dx[1];
    const double ratio = horiz > 0.0 ? std::fabs(gamma / horiz) : INFINITY;
    return g::norm(n) < 1e-2 || g::norm(dx) < 5e-2 || horiz < 2e-2 || ratio > 1.0 - 1e-5;
}

void check_p2_exact(const std::vector<std::size_t>& cams, std::uint64_t seed, std::size_t trials) {
    const Rig r = rig();
    SplitMix64 rng{seed};
    std::size_t done = 0;
    while (done < trials) {
        const Truth truth = random_truth(rng);
        const Corr a = visible(rng, r, truth, cams, 0.0);
        const Corr b = visible(rng, r, truth, cams, 0.0);
        if (pair_is_degenerate(r, truth, a, b)) {
            continue;
        }
        const auto sols = solve_p2({a, b}, r.body_from_cam, truth.r_tilt);
        CHECK(!sols.empty() && sols.size() <= 2);
        double best = INFINITY;
        double by = 0, bt = 0;
        for (const auto& s : sols) {
            assert_valid_solution(r, truth.r_tilt, s, {&a, &b});
            const double ye = yaw_err_deg(s.yaw, truth.pose.yaw) * PI / 180.0;
            const double te = pos_err(s.t, truth.pose.t);
            if (ye + te < best) {
                best = ye + te;
                by = ye;
                bt = te;
            }
        }
        CHECK(by < 1e-6 && bt < 1e-6);
        done++;
    }
}

std::pair<std::vector<Corr>, std::vector<bool>> scene(SplitMix64& rng, const Rig& rig, const Truth& truth,
                                                      const std::vector<std::size_t>& cams, std::size_t n_in,
                                                      std::size_t n_out, double noise) {
    std::vector<std::pair<Corr, bool>> v;
    for (std::size_t i = 0; i < n_in; i++) {
        v.emplace_back(visible(rng, rig, truth, cams, noise), true);
    }
    for (std::size_t i = 0; i < n_out; i++) {
        v.emplace_back(outlier(rng, rig, truth, cams), false);
    }
    for (std::size_t i = v.size(); i-- > 1;) {
        std::swap(v[i], v[rng.below(i + 1)]);
    }
    std::vector<Corr> c;
    std::vector<bool> b;
    for (auto& [x, y] : v) {
        c.push_back(x);
        b.push_back(y);
    }
    return {c, b};
}

std::pair<std::vector<Corr>, std::vector<bool>> landmark_scene(SplitMix64& rng, const Rig& rig, const Truth& truth,
                                                               std::size_t n_in, std::size_t n_out) {
    std::vector<Corr> corrs;
    std::vector<bool> is_in;
    while (is_in.size() < 2 * n_in) {
        const Vec3 x = room_point(rng);
        auto a = observe(rng, rig, truth, 0, x, NOISE_PX);
        auto b = observe(rng, rig, truth, 1, x, NOISE_PX);
        if (a && b) {
            corrs.push_back(*a);
            corrs.push_back(*b);
            is_in.push_back(true);
            is_in.push_back(true);
        }
    }
    for (std::size_t i = 0; i < n_out; i++) {
        const Corr a = outlier(rng, rig, truth, {0});
        const Corr b = outlier(rng, rig, truth, {1});
        corrs.push_back(a);
        corrs.push_back(with_point(b, a.point));
        is_in.push_back(false);
        is_in.push_back(false);
    }
    return {corrs, is_in};
}

struct Trial {
    double yaw_err, t_err, recall;
    bool ok;
};

Trial evaluate(const Rig& rig, const Truth& truth, const std::vector<Corr>& corrs, const std::vector<bool>& is_in,
               const PnpParams& p) {
    const std::size_t n_in = static_cast<std::size_t>(std::count(is_in.begin(), is_in.end(), true));
    const auto r = ransac(corrs, rig.body_from_cam, truth.r_tilt, p);
    if (!r) {
        return Trial{INFINITY, INFINITY, 0.0, false};
    }
    for (std::size_t i = 1; i < r->inliers.size(); i++) {
        CHECK(r->inliers[i - 1] < r->inliers[i]);
    }
    double sum2 = 0;
    for (std::size_t i = 0; i < corrs.size(); i++) {
        const double res = residual_deg(corrs[i], rig.body_from_cam, truth.r_tilt, r->pose);
        if (std::binary_search(r->inliers.begin(), r->inliers.end(), i)) {
            CHECK(res <= p.thresh_deg + 1e-9);
            sum2 += res * res;
        } else {
            CHECK(res > p.thresh_deg - 1e-9);
        }
    }
    CHECK(std::fabs(r->rms_deg - std::sqrt(sum2 / static_cast<double>(r->inliers.size()))) < 1e-9);
    CHECK(r->iterations >= 1 && r->iterations <= p.max_iters);
    std::size_t hits = 0;
    for (std::size_t i : r->inliers) {
        hits += is_in[i];
    }
    const double ye = yaw_err_deg(r->pose.yaw, truth.pose.yaw);
    const double te = pos_err(r->pose.t, truth.pose.t);
    const double recall = static_cast<double>(hits) / static_cast<double>(n_in);
    return Trial{ye, te, recall, ye < 0.3 && te < 0.02 && recall >= 0.9};
}

std::size_t run_ransac_trials(const std::vector<std::size_t>& cams, std::size_t n_in, std::size_t n_out,
                              std::uint64_t first_seed, std::size_t trials) {
    const Rig r = rig();
    std::size_t ok = 0;
    for (std::uint64_t k = 0; k < trials; k++) {
        SplitMix64 rng{first_seed + k};
        const Truth truth = random_truth(rng);
        const auto [corrs, is_in] = scene(rng, r, truth, cams, n_in, n_out, NOISE_PX);
        PnpParams p;
        p.seed = first_seed + k;
        ok += evaluate(r, truth, corrs, is_in, p).ok;
    }
    std::printf("ransac %zu inliers + %zu outliers: %zu/%zu ok\n", n_in, n_out, ok, trials);
    return ok;
}

} // namespace

TEST(splitmix_reference_values) {
    SplitMix64 r{0};
    CHECK(r.next_u64() == 0xE220A8397B1DCDAFull);
    CHECK(r.next_u64() == 0x6E789E6AA1B965F4ull);
    SplitMix64 r7{7};
    for (std::size_t n : {1u, 2u, 3u, 250u}) {
        for (int i = 0; i < 1000; i++) {
            CHECK(r7.below(n) < n);
        }
    }
    const double u = r7.unit();
    CHECK(u >= 0.0 && u < 1.0);
}

TEST(default_params_are_the_documented_ones) {
    const PnpParams p;
    CHECK(p.thresh_deg == 1.0 && p.max_iters == 300 && p.confidence == 0.999 && p.min_inliers == 15);
}

TEST(to_pose_matches_quaternion_helpers) {
    SplitMix64 rng{11};
    for (int i = 0; i < 20; i++) {
        const Truth truth = random_truth(rng);
        const Pose p = truth.map_from_body();
        const Mat3 m = sl::mat_of(sl::qmul(sl::qyaw(truth.pose.yaw), sl::quat_of(truth.r_tilt)));
        for (int a = 0; a < 3; a++) {
            for (int b = 0; b < 3; b++) {
                CHECK(std::fabs(p.r[a][b] - m[a][b]) < 1e-12);
            }
        }
        CHECK(g::orthonormality_error(p.r) < 1e-12);
        CHECK(p.t == truth.pose.t);
        CHECK(g::mat_vec(p.r, {0.0, 1.0, 0.0})[1] >= std::cos(30.0 * PI / 180.0) - 1e-9);
    }
}

TEST(p2_noise_free_recovers_the_pose_stereo_pair) {
    check_p2_exact({0, 1}, 1, 200);
}

TEST(p2_noise_free_recovers_the_pose_four_cameras) {
    check_p2_exact({0, 1, 2, 3}, 2, 200);
}

TEST(p2_same_camera_pairs_work) {
    const Rig r = rig();
    SplitMix64 rng{3};
    int done = 0;
    while (done < 100) {
        const Truth truth = random_truth(rng);
        const std::size_t cam = rng.below(4);
        const Corr a = visible(rng, r, truth, {cam}, 0.0);
        const Corr b = visible(rng, r, truth, {cam}, 0.0);
        if (pair_is_degenerate(r, truth, a, b)) {
            continue;
        }
        const auto sols = solve_p2({a, b}, r.body_from_cam, truth.r_tilt);
        CHECK(std::any_of(sols.begin(), sols.end(), [&](const Pose4& s) { return is_truth(s, truth, 1e-4, 1e-6); }));
        done++;
    }
}

TEST(p2_accepts_rays_beyond_90_degrees) {
    const Rig r = rig();
    SplitMix64 rng{9};
    int done = 0;
    while (done < 60) {
        const Truth truth = random_truth(rng);
        const Corr a = peripheral(rng, r, truth);
        const Corr b = visible(rng, r, truth, {0, 1, 2, 3}, 0.0);
        if (pair_is_degenerate(r, truth, a, b)) {
            continue;
        }
        CHECK(a.bearing[2] < 0.0);
        const auto sols = solve_p2({a, b}, r.body_from_cam, truth.r_tilt);
        CHECK(std::any_of(sols.begin(), sols.end(), [&](const Pose4& s) { return is_truth(s, truth, 1e-4, 1e-6); }));
        done++;
    }
}

TEST(p2_degenerate_inputs_give_no_solution_and_do_not_panic) {
    const Rig r = rig();
    SplitMix64 rng{5};
    const Truth truth = random_truth(rng);
    const auto& bc = r.body_from_cam;
    const Mat3& rt = truth.r_tilt;
    const Corr a = visible(rng, r, truth, {0, 1}, 0.0);
    const Corr b = visible(rng, r, truth, {0, 1}, 0.0);
    CHECK(!solve_p2({a, b}, bc, rt).empty());
    CHECK(solve_p2({a, a}, bc, rt).empty());
    CHECK(solve_p2({a, exact(r, truth, a.cam, a.point)}, bc, rt).empty());
    CHECK(solve_p2({a, exact(r, truth, 1 - a.cam, a.point)}, bc, rt).empty());
    const Corr par = with_point(a, g::add(a.point, {0.3, 0.2, -0.4}));
    CHECK(solve_p2({a, par}, bc, rt).empty());
    const Corr low = exact(r, truth, 0, a.point);
    const Corr high = exact(r, truth, 0, g::add(a.point, {0.0, 0.8, 0.0}));
    CHECK(solve_p2({low, high}, bc, rt).empty());
    auto level = [&](double az, const Vec3& x) {
        const Vec3 down_the_ray = g::mat_vec(g::transpose(rt), {std::cos(az), 0.0, std::sin(az)});
        return Corr{0, g::mat_vec(g::transpose(bc[0].r), down_the_ray), x};
    };
    CHECK(solve_p2({level(0.3, a.point), level(1.1, b.point)}, bc, rt).empty());
    const Corr unusable[] = {with_cam(a, 9), with_bearing(a, {0.0, 0.0, 0.0}), with_bearing(a, {NAN, 0.0, 1.0}),
                             with_point(a, {INFINITY, 0.0, 0.0})};
    for (const Corr& bad : unusable) {
        CHECK(solve_p2({bad, b}, bc, rt).empty());
        CHECK(solve_p2({b, bad}, bc, rt).empty());
    }
    CHECK(solve_p2({a, b}, std::span<const Pose>{}, rt).empty());
    const Corr scaled = with_bearing(a, g::scale(a.bearing, 3.7));
    const auto s1 = solve_p2({a, b}, bc, rt);
    const auto s2 = solve_p2({scaled, b}, bc, rt);
    CHECK(s1.size() == s2.size());
    for (std::size_t i = 0; i < s1.size() && i < s2.size(); i++) {
        CHECK(yaw_err_deg(s1[i].yaw, s2[i].yaw) < 1e-9 && pos_err(s1[i].t, s2[i].t) < 1e-9);
    }
}

TEST(p2_random_pairs_obey_the_invariants) {
    const Rig r = rig();
    SplitMix64 rng{8};
    std::size_t hist[3] = {0, 0, 0};
    for (int i = 0; i < 3000; i++) {
        const Truth truth = random_truth(rng);
        const Corr a = outlier(rng, r, truth, {0, 1, 2, 3});
        const Corr b = outlier(rng, r, truth, {0, 1, 2, 3});
        const auto sols = solve_p2({a, b}, r.body_from_cam, truth.r_tilt);
        CHECK(sols.size() <= 2);
        hist[sols.size()]++;
        for (const auto& s : sols) {
            CHECK(std::isfinite(s.yaw) && std::fabs(s.yaw) <= PI + 1e-12);
            assert_valid_solution(r, truth.r_tilt, s, {&a, &b});
        }
    }
    std::printf("solve_p2 random pairs: %zu with 0, %zu with 1, %zu with 2 solutions\n", hist[0], hist[1], hist[2]);
    CHECK(hist[0] > 100 && hist[1] + hist[2] > 100);
}

TEST(p2_rejects_points_behind_the_camera) {
    const Rig r = rig();
    SplitMix64 rng{6};
    int checked = 0;
    while (checked < 100) {
        const Truth truth = random_truth(rng);
        const Corr a = visible(rng, r, truth, {0, 1}, 0.0);
        const Corr b = visible(rng, r, truth, {0, 1}, 0.0);
        if (pair_is_degenerate(r, truth, a, b)) {
            continue;
        }
        const Corr bf = with_bearing(b, g::scale(b.bearing, -1.0));
        for (const auto& s : solve_p2({a, bf}, r.body_from_cam, truth.r_tilt)) {
            assert_valid_solution(r, truth.r_tilt, s, {&a, &bf});
            CHECK(!is_truth(s, truth, 1e-4, 1e-6));
        }
        checked++;
    }
}

TEST(ransac_150_inliers_40_percent_outliers_stereo_pair) {
    CHECK(run_ransac_trials({0, 1}, 150, 100, 1000, 50) >= 48);
}

TEST(ransac_150_inliers_40_percent_outliers_four_cameras) {
    CHECK(run_ransac_trials({0, 1, 2, 3}, 150, 100, 2000, 50) >= 48);
}

TEST(ransac_landmarks_seen_by_both_cameras) {
    const Rig r = rig();
    std::size_t ok = 0;
    for (std::uint64_t k = 0; k < 30; k++) {
        SplitMix64 rng{4000 + k};
        const Truth truth = random_truth(rng);
        const auto [corrs, is_in] = landmark_scene(rng, r, truth, 75, 50);
        PnpParams p;
        p.seed = k;
        ok += evaluate(r, truth, corrs, is_in, p).ok;
    }
    CHECK(ok >= 29);
}

TEST(ransac_survives_70_percent_outliers) {
    CHECK(run_ransac_trials({0, 1, 2, 3}, 100, 233, 3000, 30) >= 28);
}

TEST(ransac_is_deterministic_per_seed) {
    const Rig r = rig();
    SplitMix64 rng{77};
    const Truth truth = random_truth(rng);
    const auto [corrs, is_in] = scene(rng, r, truth, {0, 1}, 120, 60, NOISE_PX);
    (void)is_in;
    const PnpParams p;
    const auto r1 = ransac(corrs, r.body_from_cam, truth.r_tilt, p);
    CHECK(r1 && *r1 == *ransac(corrs, r.body_from_cam, truth.r_tilt, p));
    PnpParams p1;
    p1.seed = 1;
    const auto r3 = ransac(corrs, r.body_from_cam, truth.r_tilt, p1);
    CHECK(r3 && yaw_err_deg(r1->pose.yaw, r3->pose.yaw) < 0.1 && pos_err(r1->pose.t, r3->pose.t) < 0.01);
}

TEST(ransac_degenerate_inputs_give_none) {
    const Rig r = rig();
    SplitMix64 rng{51};
    const Truth truth = random_truth(rng);
    const PnpParams p;
    const auto& bc = r.body_from_cam;
    const Mat3& rt = truth.r_tilt;
    const auto few = scene(rng, r, truth, {0, 1}, p.min_inliers - 1, 0, NOISE_PX).first;
    CHECK(!ransac(few, bc, rt, p));
    CHECK(!ransac(std::span<const Corr>{}, bc, rt, p));
    const auto just = scene(rng, r, truth, {0, 1}, p.min_inliers, 0, NOISE_PX).first;
    const auto rj = ransac(just, bc, rt, p);
    CHECK(rj && rj->inliers.size() == p.min_inliers);
    const auto junk = scene(rng, r, truth, {0, 1, 2, 3}, 0, 120, NOISE_PX).first;
    CHECK(!ransac(junk, bc, rt, p));
    auto mixed = scene(rng, r, truth, {0, 1}, 60, 20, NOISE_PX).first;
    const std::size_t n_valid = mixed.size();
    mixed.push_back(with_cam(mixed[0], 12));
    mixed.push_back(with_bearing(mixed[1], {NAN, NAN, NAN}));
    mixed.push_back(with_point(mixed[2], {NAN, NAN, NAN}));
    const auto rm = ransac(mixed, bc, rt, p);
    CHECK(rm.has_value());
    if (rm) {
        CHECK(std::all_of(rm->inliers.begin(), rm->inliers.end(), [&](std::size_t i) { return i < n_valid; }));
        CHECK(rm->inliers.size() >= 55);
    }
    const std::vector<Corr> same(40, mixed[0]);
    CHECK(!ransac(same, bc, rt, p));
    CHECK(!ransac(mixed, std::span<const Pose>{}, rt, p));
    auto with = [&](auto f) {
        PnpParams q;
        f(q);
        return q;
    };
    CHECK(!ransac(mixed, bc, rt, with([](PnpParams& q) { q.thresh_deg = 0.0; })));
    CHECK(!ransac(mixed, bc, rt, with([](PnpParams& q) { q.thresh_deg = NAN; })));
    CHECK(!ransac(mixed, bc, rt, with([](PnpParams& q) { q.max_iters = 0; })));
    CHECK(!ransac(mixed, bc, rt, with([](PnpParams& q) { q.min_inliers = 70; })));
    if (auto one = ransac(mixed, bc, rt, with([](PnpParams& q) { q.max_iters = 1; }))) {
        CHECK(one->iterations == 1);
    }
    for (double confidence : {static_cast<double>(NAN), -1.0, 0.0, 1.0, 2.0}) {
        CHECK(ransac(mixed, bc, rt, with([&](PnpParams& q) { q.confidence = confidence; })).has_value());
    }
}

TEST(refine_brings_a_perturbed_pose_to_the_noise_level) {
    const Rig r = rig();
    const std::pair<std::vector<std::size_t>, std::uint64_t> cases[] = {{{0, 1}, 21}, {{0, 1, 2, 3}, 22}};
    for (const auto& [cams, seed] : cases) {
        SplitMix64 rng{seed};
        const Truth truth = random_truth(rng);
        const auto corrs = scene(rng, r, truth, cams, 120, 0, NOISE_PX).first;
        std::vector<std::size_t> idx(corrs.size());
        for (std::size_t i = 0; i < idx.size(); i++) {
            idx[i] = i;
        }
        const double d0 = uni(rng, -1.0, 1.0);
        const double d1 = uni(rng, -1.0, 1.0);
        const double d2 = uni(rng, -1.0, 1.0);
        const Vec3 dir = g::normalize({d0, d1, d2});
        const Pose4 init{truth.pose.yaw + 2.0 * PI / 180.0, g::add(truth.pose.t, g::scale(dir, 0.10))};
        auto rms_of = [&](const Pose4& p) {
            double s = 0;
            for (const auto& c : corrs) {
                const double e = residual_deg(c, r.body_from_cam, truth.r_tilt, p);
                s += e * e;
            }
            return std::sqrt(s / static_cast<double>(corrs.size()));
        };
        const double rms_init = rms_of(init), rms_truth = rms_of(truth.pose);
        const auto [pose, rms] = refine(corrs, idx, r.body_from_cam, truth.r_tilt, init, 1.0);
        const double ye = yaw_err_deg(pose.yaw, truth.pose.yaw), te = pos_err(pose.t, truth.pose.t);
        std::printf("refine: rms %.3f -> %.4f deg (truth %.4f); yaw err %.4f deg, t err %.4f m\n", rms_init, rms,
                    rms_truth, ye, te);
        CHECK(rms_init > 5.0 * rms_truth);
        CHECK(std::fabs(rms_of(pose) - rms) < 1e-9);
        CHECK(rms <= rms_truth * (1.0 + 1e-9));
        CHECK(std::fabs(rms - rms_truth) < 0.05 * rms_truth);
        CHECK(ye < 0.1 && te < 0.01);
        CHECK(rms > 0.03 && rms < 0.3);
        CHECK(std::fabs(pose.yaw - wrap_pi(pose.yaw)) < 1e-12);
    }
}

TEST(refine_converges_exactly_on_noise_free_data_and_tolerates_gross_outliers) {
    const Rig r = rig();
    SplitMix64 rng{31};
    const Truth truth = random_truth(rng);
    const auto& bc = r.body_from_cam;
    const Mat3& rt = truth.r_tilt;
    const auto corrs = scene(rng, r, truth, {0, 1, 2, 3}, 100, 0, 0.0).first;
    std::vector<std::size_t> all(corrs.size());
    for (std::size_t i = 0; i < all.size(); i++) {
        all[i] = i;
    }
    const Pose4 init{truth.pose.yaw - 0.15, g::add(truth.pose.t, {0.2, -0.1, 0.15})};
    const auto [pose, rms] = refine(corrs, all, bc, rt, init, 1.0);
    CHECK(rms < 1e-6 && is_truth(pose, truth, 1e-7, 1e-8));
    for (double huber : {0.0, -3.0, static_cast<double>(NAN)}) {
        CHECK(is_truth(refine(corrs, all, bc, rt, init, huber).first, truth, 1e-7, 1e-8));
    }
    const auto outliers = scene(rng, r, truth, {0, 1, 2, 3}, 0, 6, 0.0).first;
    std::vector<Corr> mixed = corrs;
    mixed.insert(mixed.end(), outliers.begin(), outliers.end());
    std::vector<std::size_t> idx(mixed.size());
    for (std::size_t i = 0; i < idx.size(); i++) {
        idx[i] = i;
    }
    CHECK(is_truth(refine(mixed, idx, bc, rt, init, 1.0).first, truth, 0.5, 0.1));
    const std::vector<std::size_t> bad_idx = {0, 1, 2, 3, 4, 5, 999999};
    CHECK(std::isfinite(refine(corrs, bad_idx, bc, rt, init, 1.0).second));
    const std::vector<Corr> invalid = {with_cam(corrs[0], 7)};
    const std::vector<std::size_t> zero = {0};
    auto [p0, r0] = refine(invalid, zero, bc, rt, init, 1.0);
    CHECK(p0 == init && std::isnan(r0));
    auto [p1, r1] = refine(corrs, std::span<const std::size_t>{}, bc, rt, init, 1.0);
    CHECK(p1 == init && std::isnan(r1));
    Pose4 nan = init;
    nan.yaw = NAN;
    CHECK(std::isnan(refine(corrs, all, bc, rt, nan, 1.0).first.yaw));
}

TEST(refine_recovers_from_a_large_initial_error) {
    const Rig r = rig();
    SplitMix64 rng{33};
    int ok = 0;
    for (int i = 0; i < 20; i++) {
        const Truth truth = random_truth(rng);
        const auto corrs = scene(rng, r, truth, {0, 1, 2, 3}, 100, 0, NOISE_PX).first;
        std::vector<std::size_t> idx(corrs.size());
        for (std::size_t k = 0; k < idx.size(); k++) {
            idx[k] = k;
        }
        const Pose4 init{truth.pose.yaw + 20.0 * PI / 180.0, g::add(truth.pose.t, {0.3, -0.2, 0.3})};
        ok += is_truth(refine(corrs, idx, r.body_from_cam, truth.r_tilt, init, 1.0).first, truth, 0.2, 0.02);
    }
    std::printf("refine from 20 deg / 0.47 m off: %d/20 converged\n", ok);
    CHECK(ok >= 18);
}

TEST(residual_deg_basics) {
    const Rig r = rig();
    SplitMix64 rng{41};
    const Truth truth = random_truth(rng);
    const auto& bc = r.body_from_cam;
    const Mat3& rt = truth.r_tilt;
    const Corr c = visible(rng, r, truth, {0, 1, 2, 3}, 0.0);
    CHECK(residual_deg(c, bc, rt, truth.pose) < 1e-8);
    Pose4 off = truth.pose;
    off.yaw += PI / 180.0;
    const double res = residual_deg(c, bc, rt, off);
    CHECK(res > 1e-3 && res < 60.0);
    const Corr back = with_bearing(c, g::scale(c.bearing, -1.0));
    CHECK(std::fabs(residual_deg(back, bc, rt, truth.pose) - 180.0) < 1e-6);
    CHECK(std::isinf(residual_deg(with_cam(c, 4), bc, rt, truth.pose)));
    CHECK(std::isinf(residual_deg(with_bearing(c, {0.0, 0.0, 0.0}), bc, rt, truth.pose)));
    const Vec3 cam_centre = truth.map_from_body().apply(bc[c.cam].t);
    CHECK(std::isinf(residual_deg(with_point(c, cam_centre), bc, rt, truth.pose)));
}

TEST(jacobian_matches_finite_differences) {
    const Ray ray{{0.05, 0.02, -0.03}, g::normalize({0.2, -0.3, 0.9}), {1.0, 1.5, 2.5}};
    const Pose4 pose{0.4, {0.3, -0.1, 0.2}};
    auto resid = [&](const Pose4& q) { return linearize(ray, std::cos(q.yaw), std::sin(q.yaw), q.t)->r; };
    const auto lin = *linearize(ray, std::cos(pose.yaw), std::sin(pose.yaw), pose.t);
    CHECK(lin.cos > 0.0);
    const double h = 1e-6;
    for (int k = 0; k < 4; k++) {
        Pose4 hi = pose, lo = pose;
        if (k == 0) {
            hi.yaw += h;
            lo.yaw -= h;
        } else {
            hi.t[k - 1] += h;
            lo.t[k - 1] -= h;
        }
        const Vec3 rh = resid(hi), rl = resid(lo);
        for (int row = 0; row < 3; row++) {
            CHECK(std::fabs((rh[row] - rl[row]) / (2.0 * h) - lin.j[row][k]) < 1e-7);
        }
    }
}

TEST(solve4_solves_and_flags_singular_systems) {
    const std::array<std::array<double, 4>, 4> a = {{{4.0, 1.0, 0.0, 0.0}, {1.0, 3.0, 1.0, 0.0}, {0.0, 1.0, 2.0, 1.0},
                                                     {0.0, 0.0, 1.0, 5.0}}};
    const std::array<double, 4> x = {1.0, -2.0, 0.5, 3.0};
    std::array<double, 4> b{};
    for (int i = 0; i < 4; i++) {
        double s = -0.0;
        for (int j = 0; j < 4; j++) {
            s += a[i][j] * x[j];
        }
        b[i] = s;
    }
    const auto s = solve4(a, b);
    CHECK(s.has_value());
    for (int i = 0; i < 4; i++) {
        CHECK(std::fabs((*s)[i] - x[i]) < 1e-12);
    }
    CHECK(!solve4({}, {1.0, 1.0, 1.0, 1.0}));
    std::array<std::array<double, 4>, 4> nan;
    for (auto& r : nan) {
        r.fill(NAN);
    }
    CHECK(!solve4(nan, {1.0, 1.0, 1.0, 1.0}));
}

TEST(adaptive_iteration_count) {
    CHECK(iterations_needed(1.0, 0.999) == 1);
    CHECK(iterations_needed(0.0, 0.999) == SIZE_MAX);
    const std::size_t n = iterations_needed(0.6, 0.999);
    CHECK(n >= 15 && n <= 17);
    CHECK(iterations_needed(0.3, 0.999) > n);
    CHECK(iterations_needed(0.6, NAN) >= 1);
}

PIBRIDGE_TEST_MAIN()
