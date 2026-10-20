// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-room/src/pnp.rs (MIT OR Apache-2.0). See pnp.h.
#include "pibridge/roommap/pnp.h"

#include "pibridge/anchor/rustx.h"
#include "pibridge/cam/rustfmt.h"

#include <cmath>
#include <limits>
#include <numbers>

namespace pibridge::roommap::pnp {

namespace g = cam::geom;
namespace rf = cam::rfmt;
namespace rs = anchor::rs;
using namespace detail;

namespace {

constexpr double MIN_RAY_SIN = 1e-6;
constexpr double MIN_YAW_LEVER = 1e-6;
constexpr double MIN_POINT_SEP2 = 1e-12;
constexpr double MIN_DEPTH = 1e-6;
constexpr double TANGENT_TOL = 1e-9;
constexpr std::size_t LM_MAX_ITERS = 60;
constexpr std::size_t LM_MAX_RETRIES = 14;
constexpr double PI = std::numbers::pi;
constexpr double FRAC_PI_2 = std::numbers::pi / 2.0;

Mat3 rot_y_cs(double c, double s) {
    return {{{c, 0.0, s}, {0.0, 1.0, 0.0}, {-s, 0.0, c}}};
}

bool finite3(const Vec3& v) {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

Vec3 offset(const Ray& r, double c, double s, const Vec3& t) {
    const Vec3 y = g::sub(r.x, t);
    return {c * y[0] - s * y[2] - r.p[0], y[1] - r.p[1], s * y[0] + c * y[2] - r.p[2]};
}

std::optional<double> cos_angle(const Ray& r, double c, double s, const Vec3& t) {
    const Vec3 v = offset(r, c, s, t);
    const double n = g::norm(v);
    if (n > 1e-12) {
        return g::dot(r.d, v) / n;
    }
    return std::nullopt;
}

double angle(const Ray& r, double c, double s, const Vec3& t) {
    const Vec3 v = offset(r, c, s, t);
    const double n = g::norm(v);
    if (std::isnan(n) || n <= 1e-12) {
        return std::numeric_limits<double>::infinity();
    }
    return std::atan2(g::norm(g::cross(r.d, v)), g::dot(r.d, v));
}

void build_rays(std::span<const Corr> corrs, const std::vector<std::size_t>* idx, std::size_t n_all,
                std::span<const Pose> body_from_cam, const Mat3& r_tilt, std::vector<Ray>& rays,
                std::vector<std::size_t>& ids) {
    const std::size_t n = idx != nullptr ? idx->size() : n_all;
    for (std::size_t k = 0; k < n; k++) {
        const std::size_t i = idx != nullptr ? (*idx)[k] : k;
        if (i >= corrs.size()) {
            continue;
        }
        if (auto r = Ray::make(corrs[i], body_from_cam, r_tilt)) {
            rays.push_back(*r);
            ids.push_back(i);
        }
    }
}

std::optional<Pose4> complete(const Ray& a, const Ray& b, double nn2, double c, double s) {
    const Vec3 dx = g::sub(a.x, b.x);
    const Vec3 dp = g::sub(a.p, b.p);
    const Vec3 w{c * dx[0] - s * dx[2] - dp[0], dx[1] - dp[1], s * dx[0] + c * dx[2] - dp[2]};
    const double k = g::dot(a.d, b.d);
    const double wa = g::dot(a.d, w);
    const double wb = g::dot(b.d, w);
    const double la = (wa - k * wb) / nn2;
    const double lb = (k * wa - wb) / nn2;
    if (std::isnan(la) || std::isnan(lb) || la <= MIN_DEPTH || lb <= MIN_DEPTH) {
        return std::nullopt;
    }
    const Mat3 ry = rot_y_cs(c, s);
    const Vec3 ta = g::sub(a.x, g::mat_vec(ry, g::add(a.p, g::scale(a.d, la))));
    const Vec3 tb = g::sub(b.x, g::mat_vec(ry, g::add(b.p, g::scale(b.d, lb))));
    const Vec3 t = g::scale(g::add(ta, tb), 0.5);
    if (!finite3(t)) {
        return std::nullopt;
    }
    return Pose4{std::atan2(s, c), t};
}

std::size_t solve_rays(const Ray& a, const Ray& b, std::array<Pose4, 2>& out) {
    const Vec3 n = g::cross(a.d, b.d);
    const double nn2 = g::dot(n, n);
    if (std::isnan(nn2) || nn2 <= MIN_RAY_SIN * MIN_RAY_SIN) {
        return 0;
    }
    const Vec3 dx = g::sub(a.x, b.x);
    if (std::isnan(g::dot(dx, dx)) || g::dot(dx, dx) <= MIN_POINT_SEP2) {
        return 0;
    }
    const Vec3 dp = g::sub(a.p, b.p);
    const Vec3 m = g::scale(n, 1.0 / std::sqrt(nn2));
    const double alpha = m[0] * dx[0] + m[2] * dx[2];
    const double beta = m[2] * dx[0] - m[0] * dx[2];
    const double gamma = g::dot(m, dp) - m[1] * dx[1];
    const double r2 = alpha * alpha + beta * beta;
    if (std::isnan(r2) || r2 <= MIN_YAW_LEVER * MIN_YAW_LEVER || std::isnan(gamma)) {
        return 0;
    }
    const double ratio2 = gamma * gamma / r2;
    if (ratio2 > 1.0 + TANGENT_TOL) {
        return 0;
    }
    const double h = std::sqrt(rf::fmax(1.0 - ratio2, 0.0));
    const double r = std::sqrt(r2);
    const double c0 = gamma * alpha / r2;
    const double s0 = gamma * beta / r2;
    const double lx = -beta / r;
    const double ly = alpha / r;
    std::size_t count = 0;
    const double both[2] = {1.0, -1.0};
    const double zero[1] = {0.0};
    const bool tangent = h < 1e-12;
    const double* signs = tangent ? zero : both;
    const std::size_t ns = tangent ? 1 : 2;
    for (std::size_t k = 0; k < ns; k++) {
        const double sg = signs[k];
        const double cy = c0 + sg * h * lx;
        const double sy = s0 + sg * h * ly;
        const double nrm = std::hypot(cy, sy);
        if (auto pose = complete(a, b, nn2, cy / nrm, sy / nrm)) {
            out[count++] = *pose;
        }
    }
    return count;
}

struct Score {
    std::size_t count;
    double cost;
    bool better_than(const Score& o) const { return cost < o.cost || (cost == o.cost && count > o.count); }
};

Score score(const std::vector<Ray>& rays, const Pose4& pose, double cos_thr) {
    const double s = std::sin(pose.yaw);
    const double c = std::cos(pose.yaw);
    const double cap = 1.0 - cos_thr;
    Score sc{0, 0.0};
    for (const Ray& r : rays) {
        const auto ca = cos_angle(r, c, s, pose.t);
        const double e = ca ? 1.0 - *ca : 2.0;
        if (e <= cap) {
            sc.count++;
        }
        sc.cost += rf::fmin(e, cap);
    }
    return sc;
}

std::vector<std::size_t> inlier_ids(const std::vector<Ray>& rays, const Pose4& pose, double cos_thr) {
    const double s = std::sin(pose.yaw);
    const double c = std::cos(pose.yaw);
    std::vector<std::size_t> out;
    for (std::size_t i = 0; i < rays.size(); i++) {
        const auto ca = cos_angle(rays[i], c, s, pose.t);
        if (ca && *ca >= cos_thr) {
            out.push_back(i);
        }
    }
    return out;
}

double huber(double rn, double delta) {
    if (rn <= delta) {
        return 0.5 * rn * rn;
    }
    return delta * (rn - 0.5 * delta);
}

std::optional<double> linearize_cost(const Ray& ray, double c, double s, const Vec3& t) {
    const Vec3 v = offset(ray, c, s, t);
    const double nv = g::norm(v);
    if (std::isnan(nv) || nv <= 1e-9 || g::dot(ray.d, v) <= 0.0) {
        return std::nullopt;
    }
    return rf::fmin(g::norm(g::cross(ray.d, v)) / nv, 1.0);
}

double robust_cost(const std::vector<Ray>& rays, const Pose4& pose, double delta) {
    const double s = std::sin(pose.yaw);
    const double c = std::cos(pose.yaw);
    double sum = -0.0;
    for (const Ray& ray : rays) {
        const auto rn = linearize_cost(ray, c, s, pose.t);
        sum += rn ? huber(*rn, delta) : huber(1.0, delta);
    }
    return sum;
}

void normal_equations(const std::vector<Ray>& rays, const Pose4& pose, double delta,
                      std::array<std::array<double, 4>, 4>& h, std::array<double, 4>& gv) {
    const double s = std::sin(pose.yaw);
    const double c = std::cos(pose.yaw);
    h = {};
    gv = {};
    for (const Ray& ray : rays) {
        const auto l = linearize(ray, c, s, pose.t);
        if (!l) {
            continue;
        }
        if (l->cos <= 0.0) {
            continue;
        }
        const double rn = g::norm(l->r);
        const double w = rn <= delta ? 1.0 : delta / rn;
        for (std::size_t row = 0; row < 3; row++) {
            const auto& jr = l->j[row];
            const double rr = l->r[row];
            for (std::size_t a = 0; a < 4; a++) {
                gv[a] += w * jr[a] * rr;
                for (std::size_t b = 0; b < 4; b++) {
                    h[a][b] += w * jr[a] * jr[b];
                }
            }
        }
    }
}

Pose4 refine_rays(const std::vector<Ray>& rays, const Pose4& init, double huber_rad) {
    Pose4 pose = init;
    if (rays.empty() || !std::isfinite(pose.yaw) || !finite3(pose.t)) {
        return pose;
    }
    const double delta =
        huber_rad > 0.0 ? std::sin(rf::fmin(huber_rad, FRAC_PI_2)) : std::numeric_limits<double>::infinity();
    double cost = robust_cost(rays, pose, delta);
    double mu = 1e-3;
    for (std::size_t it = 0; it < LM_MAX_ITERS; it++) {
        std::array<std::array<double, 4>, 4> h;
        std::array<double, 4> gv;
        normal_equations(rays, pose, delta, h, gv);
        bool accepted = false;
        Pose4 cand;
        double c2 = 0;
        std::array<double, 4> step{};
        for (std::size_t k = 0; k < LM_MAX_RETRIES; k++) {
            auto a = h;
            for (std::size_t i = 0; i < 4; i++) {
                a[i][i] += mu * rf::fmax(h[i][i], 1e-9);
            }
            const auto st = solve4(a, {-gv[0], -gv[1], -gv[2], -gv[3]});
            if (!st) {
                mu *= 10.0;
                continue;
            }
            const Pose4 cd{pose.yaw + (*st)[0], {pose.t[0] + (*st)[1], pose.t[1] + (*st)[2], pose.t[2] + (*st)[3]}};
            const double cc = robust_cost(rays, cd, delta);
            if (cc <= cost) {
                accepted = true;
                cand = cd;
                c2 = cc;
                step = *st;
                mu = rf::fmax(mu * 0.3, 1e-12);
                break;
            }
            mu *= 4.0;
        }
        if (!accepted) {
            break;
        }
        const double decrease = cost - c2;
        pose = cand;
        cost = c2;
        const double step_size = rf::fmax(std::fabs(step[0]), g::norm({step[1], step[2], step[3]}));
        if (step_size < 1e-11 || decrease <= 1e-15 * cost) {
            break;
        }
    }
    pose.yaw = wrap_pi(pose.yaw);
    return pose;
}

std::pair<Pose4, Score> local_opt(const std::vector<Ray>& rays, const Pose4& start, double cos_thr, double huber_rad) {
    Pose4 pose = start;
    Score best = score(rays, pose, cos_thr);
    for (int k = 0; k < 2; k++) {
        std::vector<Ray> sel;
        for (std::size_t i : inlier_ids(rays, pose, cos_thr)) {
            sel.push_back(rays[i]);
        }
        if (sel.size() < 3) {
            break;
        }
        const Pose4 cand = refine_rays(sel, pose, huber_rad);
        const Score sc = score(rays, cand, cos_thr);
        if (!sc.better_than(best)) {
            break;
        }
        pose = cand;
        best = sc;
    }
    return {pose, best};
}

} // namespace

namespace detail {

std::uint64_t SplitMix64::next_u64() {
    s += 0x9E3779B97F4A7C15ull;
    std::uint64_t z = s;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

__extension__ typedef unsigned __int128 u128;

std::size_t SplitMix64::below(std::size_t n) {
    return static_cast<std::size_t>((static_cast<u128>(next_u64()) * n) >> 64);
}

double SplitMix64::unit() {
    return static_cast<double>(next_u64() >> 11) * (1.0 / 9007199254740992.0);
}

double wrap_pi(double a) {
    double x = std::fmod(a, 2.0 * PI);
    if (x > PI) {
        x -= 2.0 * PI;
    } else if (x <= -PI) {
        x += 2.0 * PI;
    }
    return x;
}

std::optional<Ray> Ray::make(const Corr& c, std::span<const Pose> body_from_cam, const Mat3& r_tilt) {
    if (c.cam >= body_from_cam.size()) {
        return std::nullopt;
    }
    const Pose& b = body_from_cam[c.cam];
    if (!finite3(c.bearing) || !finite3(c.point)) {
        return std::nullopt;
    }
    const Vec3 d = g::mat_vec(r_tilt, g::mat_vec(b.r, c.bearing));
    const double n = g::norm(d);
    const Vec3 p = g::mat_vec(r_tilt, b.t);
    if (!std::isfinite(n) || n < 1e-12 || !finite3(p)) {
        return std::nullopt;
    }
    return Ray{p, g::scale(d, 1.0 / n), c.point};
}

std::optional<Lin> linearize(const Ray& ray, double c, double s, const Vec3& t) {
    const Vec3 y = g::sub(ray.x, t);
    const Vec3 v = offset(ray, c, s, t);
    const double nv = g::norm(v);
    if (std::isnan(nv) || nv <= 1e-9) {
        return std::nullopt;
    }
    const Vec3 vh = g::scale(v, 1.0 / nv);
    const Vec3 dv[4] = {
        {-s * y[0] - c * y[2], 0.0, c * y[0] - s * y[2]},
        {-c, 0.0, -s},
        {0.0, -1.0, 0.0},
        {s, 0.0, -c},
    };
    Lin l{};
    for (std::size_t k = 0; k < 4; k++) {
        const Vec3 dvh = g::scale(g::sub(dv[k], g::scale(vh, g::dot(vh, dv[k]))), 1.0 / nv);
        const Vec3 col = g::cross(ray.d, dvh);
        for (std::size_t row = 0; row < 3; row++) {
            l.j[row][k] = col[row];
        }
    }
    l.r = g::cross(ray.d, vh);
    l.cos = g::dot(ray.d, vh);
    return l;
}

std::optional<std::array<double, 4>> solve4(std::array<std::array<double, 4>, 4> a, std::array<double, 4> b) {
    for (std::size_t col = 0; col < 4; col++) {
        std::size_t piv = col;
        for (std::size_t row = col + 1; row < 4; row++) {
            if (std::fabs(a[row][col]) > std::fabs(a[piv][col])) {
                piv = row;
            }
        }
        const double p = a[piv][col];
        if (std::isnan(p) || std::fabs(p) <= 1e-300) {
            return std::nullopt;
        }
        std::swap(a[col], a[piv]);
        std::swap(b[col], b[piv]);
        for (std::size_t row = col + 1; row < 4; row++) {
            const double f = a[row][col] / a[col][col];
            const auto pivot_row = a[col];
            for (std::size_t k = col; k < 4; k++) {
                a[row][k] -= f * pivot_row[k];
            }
            b[row] -= f * b[col];
        }
    }
    std::array<double, 4> x{};
    for (std::size_t r = 4; r-- > 0;) {
        double tail = -0.0;
        for (std::size_t k = r + 1; k < 4; k++) {
            tail += a[r][k] * x[k];
        }
        x[r] = (b[r] - tail) / a[r][r];
    }
    for (double v : x) {
        if (!std::isfinite(v)) {
            return std::nullopt;
        }
    }
    return x;
}

std::size_t iterations_needed(double w, double confidence) {
    const double conf = (confidence > 0.0 && confidence < 1.0) ? confidence : 0.999;
    const double w2 = rf::clamp(w * w, 0.0, 1.0);
    if (w2 >= 1.0 - 1e-12) {
        return 1;
    }
    if (w2 <= 1e-12) {
        return std::numeric_limits<std::size_t>::max();
    }
    const double k = std::ceil(std::log(1.0 - conf) / std::log1p(-w2));
    if (std::isfinite(k) && k >= 1.0) {
        return rs::to_usize(k);
    }
    return std::numeric_limits<std::size_t>::max();
}

} // namespace detail

Pose Pose4::to_pose(const Mat3& r_tilt) const {
    return Pose{g::mat_mul(rot_y(yaw), r_tilt), t};
}

Mat3 rot_y(double yaw) {
    return rot_y_cs(std::cos(yaw), std::sin(yaw));
}

std::vector<Pose4> solve_p2(const std::array<Corr, 2>& c, std::span<const Pose> body_from_cam, const Mat3& r_tilt) {
    const auto a = Ray::make(c[0], body_from_cam, r_tilt);
    const auto b = Ray::make(c[1], body_from_cam, r_tilt);
    if (!a || !b) {
        return {};
    }
    std::array<Pose4, 2> sols{};
    const std::size_t n = solve_rays(*a, *b, sols);
    return std::vector<Pose4>(sols.begin(), sols.begin() + static_cast<std::ptrdiff_t>(n));
}

std::optional<PnpResult> ransac(std::span<const Corr> corrs, std::span<const Pose> body_from_cam, const Mat3& r_tilt,
                                const PnpParams& p) {
    const std::size_t min_inliers = std::max<std::size_t>(p.min_inliers, 2);
    if (!(std::isfinite(p.thresh_deg) && p.thresh_deg > 0.0) || p.max_iters == 0) {
        return std::nullopt;
    }
    std::vector<Ray> rays;
    std::vector<std::size_t> ids;
    build_rays(corrs, nullptr, corrs.size(), body_from_cam, r_tilt, rays, ids);
    const std::size_t m = rays.size();
    if (m < min_inliers) {
        return std::nullopt;
    }
    const double thresh_rad = rf::to_radians(rf::fmin(p.thresh_deg, 89.0));
    const double cos_thr = std::cos(thresh_rad);
    SplitMix64 rng{p.seed};
    std::optional<std::pair<Pose4, Score>> best;
    std::optional<Score> best_raw;
    std::size_t needed = p.max_iters;
    std::size_t iterations = 0;
    while (iterations < std::min(needed, p.max_iters)) {
        iterations++;
        const std::size_t i = rng.below(m);
        std::size_t j = rng.below(m - 1);
        if (j >= i) {
            j++;
        }
        std::array<Pose4, 2> sols{};
        const std::size_t ns = solve_rays(rays[i], rays[j], sols);
        for (std::size_t k = 0; k < ns; k++) {
            const Score raw = score(rays, sols[k], cos_thr);
            if (best_raw && !raw.better_than(*best_raw)) {
                continue;
            }
            best_raw = raw;
            const auto [pose, polished] = local_opt(rays, sols[k], cos_thr, thresh_rad);
            if (best && !polished.better_than(best->second)) {
                continue;
            }
            needed = iterations_needed(static_cast<double>(polished.count) / static_cast<double>(m), p.confidence);
            best = std::pair{pose, polished};
        }
    }
    if (!best) {
        return std::nullopt;
    }
    Pose4 pose = best->first;
    double cost = score(rays, pose, cos_thr).cost;
    for (int k = 0; k < 2; k++) {
        std::vector<Ray> sel;
        for (std::size_t i : inlier_ids(rays, pose, cos_thr)) {
            sel.push_back(rays[i]);
        }
        if (sel.size() < 2) {
            break;
        }
        const Pose4 cand = refine_rays(sel, pose, thresh_rad);
        const double c2 = score(rays, cand, cos_thr).cost;
        if (c2 <= cost) {
            pose = cand;
            cost = c2;
        }
    }
    const auto inl = inlier_ids(rays, pose, cos_thr);
    if (inl.size() < min_inliers) {
        return std::nullopt;
    }
    const double s = std::sin(pose.yaw);
    const double c = std::cos(pose.yaw);
    double sum2 = -0.0;
    for (std::size_t i : inl) {
        const double a = angle(rays[i], c, s, pose.t);
        sum2 += a * a;
    }
    PnpResult res;
    res.pose = pose;
    res.rms_deg = rf::to_degrees(std::sqrt(sum2 / static_cast<double>(inl.size())));
    for (std::size_t i : inl) {
        res.inliers.push_back(ids[i]);
    }
    res.iterations = iterations;
    return res;
}

double residual_deg(const Corr& c, std::span<const Pose> body_from_cam, const Mat3& r_tilt, const Pose4& pose) {
    if (auto r = Ray::make(c, body_from_cam, r_tilt)) {
        return rf::to_degrees(angle(*r, std::cos(pose.yaw), std::sin(pose.yaw), pose.t));
    }
    return std::numeric_limits<double>::infinity();
}

std::pair<Pose4, double> refine(std::span<const Corr> corrs, std::span<const std::size_t> idx,
                                std::span<const Pose> body_from_cam, const Mat3& r_tilt, const Pose4& init,
                                double huber_deg) {
    std::vector<Ray> rays;
    std::vector<std::size_t> ids;
    const std::vector<std::size_t> iv(idx.begin(), idx.end());
    build_rays(corrs, &iv, 0, body_from_cam, r_tilt, rays, ids);
    const double huber_rad = (std::isfinite(huber_deg) && huber_deg > 0.0) ? rf::to_radians(huber_deg) : 0.0;
    const Pose4 pose = refine_rays(rays, init, huber_rad);
    const double s = std::sin(pose.yaw);
    const double c = std::cos(pose.yaw);
    double s2 = 0.0;
    std::size_t n = 0;
    for (const Ray& r : rays) {
        const double a = angle(r, c, s, pose.t);
        if (std::isfinite(a)) {
            s2 = s2 + a * a;
            n++;
        }
    }
    const double rms = n == 0 ? std::numeric_limits<double>::quiet_NaN()
                              : rf::to_degrees(std::sqrt(s2 / static_cast<double>(n)));
    return {pose, rms};
}

} // namespace pibridge::roommap::pnp
