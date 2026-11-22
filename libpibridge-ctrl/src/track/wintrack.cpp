// SPDX-License-Identifier: MIT
// PiBridge: the controller tracker after Pimax's LED engine (wintrack.h).
#include "pibridge/ctrl/track/wintrack.h"

#include "pibridge/cam/rustfmt.h"
#include "pibridge/ctrl/track/p3p.h"
#include "pibridge/ctrl/track/trackutil.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <format>
#include <string_view>

namespace pibridge::ctrl::track {

namespace g = cam::geom;
namespace rf = cam::rfmt;

namespace {

constexpr double G = 9.80665;

/// WT_DEBUG: a per-frame trace on stderr (the bench).
bool wt_debug() {
    static const bool d = std::getenv("WT_DEBUG") != nullptr;
    return d;
}

bool env_flag(const char* name, bool def) {
    const char* e = std::getenv(name);
    if (e == nullptr) {
        return def;
    }
    return std::string_view(e) != "0";
}

/// Pimax's inlier gate by the nearest LED's distance to the camera.
double gate_px(double d_min) {
    if (d_min < 0.05) {
        return 60.0;
    }
    if (d_min < 0.1) {
        return 10.0;
    }
    if (d_min <= 0.4) {
        return 5.0;
    }
    if (d_min <= 0.7) {
        return 3.0;
    }
    return 1.5;
}

/// The LED projected from `cam_from_model` if visible (z > 1 mm, normal within `cos_vis` of the direction to the
/// camera, inside the image with a 15 px margin).
std::optional<std::pair<double, double>> project_visible(const Cam& cam, const g::Pose& cam_from_model, const Led& led,
                                                         double cos_vis, g::Vec3* pc_out = nullptr) {
    const g::Vec3 pc = cam_from_model.apply(led.p);
    if (pc[2] <= 0.001) {
        return std::nullopt;
    }
    const g::Vec3 nc = g::mat_vec(cam_from_model.r, led.n);
    const g::Vec3 to_cam = g::scale(pc, -1.0 / g::norm(pc));
    if (g::dot(nc, to_cam) < cos_vis) {
        return std::nullopt;
    }
    const auto uv = cam.intr.project(pc);
    if (!uv) {
        return std::nullopt;
    }
    if (uv->first < -15.0 || uv->second < -15.0 || uv->first > cam.intr.width + 15.0 ||
        uv->second > cam.intr.height + 15.0) {
        return std::nullopt;
    }
    if (pc_out != nullptr) {
        *pc_out = pc;
    }
    return uv;
}

} // namespace

WinTracker::WinTracker(WinTrackCfg c, TrackCfg fallback) : cfg(c), fallback_(fallback) {
    // offline tuning knobs (the bench)
    if (const char* e = std::getenv("WT_REVERT")) {
        cfg.revert_rms_px = std::atof(e);
    }
    if (const char* e = std::getenv("WT_RELOCK")) {
        cfg.min_leds_relock = static_cast<std::size_t>(std::atoi(e));
    }
    if (const char* e = std::getenv("WT_P3P_DEG")) {
        cfg.p3p_max_deg = std::atof(e);
    }
    if (const char* e = std::getenv("WT_GATE_SCALE")) {
        cfg.gate_scale = std::atof(e);
    }
    if (const char* e = std::getenv("WT_RELOCK_FRAMES")) {
        cfg.relock_frames = static_cast<std::uint32_t>(std::atoi(e));
    }
    if (const char* e = std::getenv("WT_FAR_LAST")) {
        cfg.far_from_last = std::string_view(e) != "0";
    }
    if (const char* e = std::getenv("WT_ACC_HOLD")) {
        cfg.acc_hold = std::string_view(e) != "0";
    }
    if (const char* e = std::getenv("WT_FAR_M")) {
        cfg.far_m = std::atof(e);
    }
    if (const char* e = std::getenv("WT_STRONG")) {
        cfg.relock_strong = static_cast<std::size_t>(std::atoi(e));
    }
    if (const char* e = std::getenv("WT_HYP_FEW_RAD")) {
        cfg.hyp_gyro_rad_few = std::atof(e);
    }
    if (const char* e = std::getenv("WT_WARM_S")) {
        cfg.warm_s = std::atof(e);
    }
    if (const char* e = std::getenv("WT_RELOCK_BOX")) {
        cfg.relock_box_frac = std::atof(e);
    }
    if (const char* e = std::getenv("WT_REFIT")) {
        cfg.refit_turned = std::string_view(e) != "0";
    }
    if (const char* e = std::getenv("WT_TENTATIVE")) {
        cfg.tentative_below = static_cast<std::size_t>(std::atoi(e));
        cfg.tentative_one_cam = cfg.tentative_below > 0;
    }
    if (const char* e = std::getenv("WT_TENTATIVE_RMS")) {
        cfg.tentative_rms_px = std::atof(e);
    }
    if (const char* e = std::getenv("WT_TENTATIVE_MIN")) {
        cfg.tentative_min_leds = static_cast<std::size_t>(std::atoi(e));
    }
    if (const char* e = std::getenv("WT_TENTATIVE_ONECAM")) {
        cfg.tentative_one_cam = std::string_view(e) != "0";
    }
    if (const char* e = std::getenv("WT_WARM_ROT")) {
        // "base,per_s"; "0": the flat 30 deg
        cfg.warm_rot_deg = std::atof(e);
        cfg.warm_rot_deg_per_s = 0.0;
        if (const char* k = std::strchr(e, ',')) {
            cfg.warm_rot_deg_per_s = std::atof(k + 1);
        }
        if (cfg.warm_rot_deg <= 0.0) {
            cfg.warm_rot_deg = 30.0; // off: the flat 30 for one-camera fits too
            cfg.warm_rot_deg_per_s = 0.0;
        }
    }
    for (Hand& h : h_) {
        h.win = Window(cfg.window);
    }
}

void WinTracker::imu_push(std::size_t slot, const ImuIn& s) {
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
    ImuIn in = s;
    if (cfg.acc_hold && in.f_aim == g::Vec3{0.0, 0.0, 0.0}) {
        // PiBridge: the hint writes f_aim = 0 when no accelerometer sample came with the IMU sample (live 1-2.5 % of
        // samples; the recordings' logs never do); taken as a measurement it is a free fall (9.8 m/s^2 for a step)
        // and the window absorbed it into the biases and the velocity. The previous sample's force is held instead.
        counts.imu_no_acc += 1;
        in.f_aim = d.empty() ? g::Vec3{0.0, G, 0.0} : d.back().f_aim;
    }
    d.push_back(in);
    if (s.world_from_head.t != g::Vec3{0.0, 0.0, 0.0} || s.world_from_head.r != g::IDENTITY) {
        head_seen_ = s.t;
    }
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

const ImuIn* WinTracker::imu_at(std::size_t slot, std::uint64_t t) const {
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

bool WinTracker::imu_live(std::size_t slot, std::uint64_t t) const {
    return !imu_[slot].empty() && sat_sub(t, imu_[slot].back().t) < 500'000'000 &&
           imu_[slot].back().t + 500'000'000 > t;
}

std::vector<ImuStep> WinTracker::steps(std::size_t slot, std::uint64_t t0, std::uint64_t t1, bool* gap) const {
    std::vector<ImuStep> out;
    if (gap != nullptr) {
        *gap = false;
    }
    const auto& d = imu_[slot];
    if (d.size() < 2 || t1 <= t0) {
        return out;
    }
    const g::Mat3& D = device_from_aim;
    const g::Mat3 Dt = g::transpose(D);
    std::uint64_t covered_to = t0;
    g::Vec3 last_rate{0.0, 0.0, 0.0};
    g::Vec3 last_f{0.0, G, 0.0};
    for (std::size_t i = 1; i < d.size(); i++) {
        const ImuIn& a = d[i - 1];
        const ImuIn& b = d[i];
        if (b.t <= t0) {
            continue;
        }
        if (a.t >= t1) {
            break;
        }
        const std::uint64_t lo = std::max(a.t, t0), hi = std::min(b.t, t1);
        if (hi <= lo || b.t <= a.t) {
            continue;
        }
        const double full = static_cast<double>(b.t - a.t) * 1e-9;
        const double part = static_cast<double>(hi - lo) * 1e-9;
        const g::Vec3 dth_full = rotvec(g::mat_mul(g::mat_mul(D, g::mat_mul(g::transpose(a.qg), b.qg)), Dt));
        const g::Vec3 f = g::mat_vec(D, b.f_aim);
        ImuStep s;
        s.dt = part;
        s.dtheta = g::scale(dth_full, part / full);
        s.f = f;
        out.push_back(s);
        last_rate = g::scale(dth_full, 1.0 / full);
        last_f = f;
        covered_to = hi;
    }
    if (covered_to < t1) {
        // the IMU has not reached the frame yet: hold the last rates (Pimax waits up to 12 ms instead)
        const double dt = static_cast<double>(t1 - covered_to) * 1e-9;
        if (dt < 0.05) {
            ImuStep s;
            s.dt = dt;
            s.dtheta = g::scale(last_rate, dt);
            s.f = last_f;
            out.push_back(s);
            if (gap != nullptr) {
                *gap = true;
            }
        }
    }
    return out;
}

WinTracker::ImuStats WinTracker::imu_stats(std::size_t slot, std::uint64_t t, const g::Vec3& bg,
                                           const g::Vec3& ba) const {
    ImuStats st;
    const auto& d = imu_[slot];
    if (d.size() < 3) {
        return st;
    }
    const g::Mat3& D = device_from_aim;
    const g::Mat3 Dt = g::transpose(D);
    std::vector<double> gn, an;
    double max_dev = 0.0;
    for (std::size_t i = d.size() - 1; i >= 1 && gn.size() < 16; i--) {
        const ImuIn& a = d[i - 1];
        const ImuIn& b = d[i];
        if (b.t > t + 20'000'000) {
            continue;
        }
        const double dt = static_cast<double>(b.t - a.t) * 1e-9;
        if (dt <= 0.0) {
            continue;
        }
        const g::Vec3 w = g::sub(g::scale(rotvec(g::mat_mul(g::mat_mul(D, g::mat_mul(g::transpose(a.qg), b.qg)), Dt)), 1.0 / dt), bg);
        const g::Vec3 acc = g::sub(g::mat_vec(D, b.f_aim), ba);
        gn.push_back(g::norm(w));
        an.push_back(g::norm(acc));
        max_dev = std::max(max_dev, std::abs(g::norm(acc) - G));
        if (i == 1) {
            break;
        }
    }
    if (gn.size() < 2) {
        return st;
    }
    const auto mean = [](const std::vector<double>& v) {
        double s = 0;
        for (const double x : v) {
            s += x;
        }
        return s / static_cast<double>(v.size());
    };
    const auto sd = [&](const std::vector<double>& v, double m) {
        double s = 0;
        for (const double x : v) {
            s += (x - m) * (x - m);
        }
        return std::sqrt(s / static_cast<double>(v.size()));
    };
    st.ok = true;
    st.mean_g = mean(gn);
    st.std_g = sd(gn, st.mean_g);
    st.max_g = *std::max_element(gn.begin(), gn.end());
    st.mean_a = mean(an);
    st.std_a = sd(an, st.mean_a);
    st.max_dev = max_dev;
    return st;
}

// ---------------------------------------------------------------- matching (Pimax's matcher)

std::optional<WinTracker::Match> WinTracker::match(const Rig& rig, std::size_t slot, const WinState& pred,
                                                   const g::Pose& bfw, const std::vector<CamObs>& obs,
                                                   const std::vector<std::vector<bool>>& used) {
    const LedModel& model = rig.models[slot];
    Hand& hd = h_[slot];
    const double cos_vis = std::cos(rf::to_radians(cfg.visible_deg));
    const g::Pose pred_body{g::mat_mul(bfw.r, pred.r), bfw.apply(pred.p)};
    const double radius = hd.frames_tracked < cfg.first_frames ? cfg.radius_first
                          : hd.last_valid < 4                 ? cfg.radius_few
                                                              : cfg.radius;
    // candidate pairs from the prediction, per camera
    struct Pair {
        double d;
        std::size_t cam, led, blob;
        g::Vec3 pc; // predicted LED in the camera frame
    };
    std::vector<Pair> pairs;
    std::size_t blobs_total = 0;
    for (std::size_t ci = 0; ci < rig.cams.size() && ci < obs.size(); ci++) {
        blobs_total += obs[ci].px.size();
        const g::Pose cfm = rig.cams[ci].cam_from_body.compose(pred_body);
        for (std::size_t l = 0; l < model.leds.size(); l++) {
            g::Vec3 pc;
            const auto uv = project_visible(rig.cams[ci], cfm, model.leds[l], cos_vis, &pc);
            if (!uv) {
                continue;
            }
            for (std::size_t b = 0; b < obs[ci].px.size(); b++) {
                if (used[ci][b] || !obs[ci].bearing[b]) {
                    continue;
                }
                const double d = std::hypot(uv->first - obs[ci].px[b].first, uv->second - obs[ci].px[b].second);
                if (d <= radius) {
                    pairs.push_back({d, ci, l, b, pc});
                }
            }
        }
    }
    if (pairs.empty()) {
        counts.miss_no_pairs += 1;
        if (wt_debug()) {
            std::fprintf(stderr, "wt %llu slot %zu MISS no pairs (radius %.0f, blobs %zu)\n",
                         static_cast<unsigned long long>(frame_no_), slot, radius, blobs_total);
        }
        return std::nullopt;
    }
    std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) { return a.d < b.d; });
    // rough matches: greedy one-to-one over the sorted pairs
    std::vector<Pair> rough;
    {
        std::vector<std::vector<bool>> bused(obs.size());
        for (std::size_t ci = 0; ci < obs.size(); ci++) {
            bused[ci].assign(obs[ci].px.size(), false);
        }
        std::vector<std::vector<bool>> lused(obs.size(), std::vector<bool>(model.leds.size(), false));
        for (const Pair& p : pairs) {
            if (bused[p.cam][p.blob] || lused[p.cam][p.led]) {
                continue;
            }
            bused[p.cam][p.blob] = true;
            lused[p.cam][p.led] = true;
            rough.push_back(p);
            if (rough.size() >= 30) {
                break;
            }
        }
    }
    // hypotheses: body_from_model poses
    struct Hyp {
        g::Pose pose;
        int kind;
    };
    std::vector<Hyp> hyps;
    hyps.push_back({pred_body, 0});
    // P3P on triples of rough matches of one camera (the first 6 per camera)
    for (std::size_t ci = 0; ci < rig.cams.size(); ci++) {
        std::vector<const Pair*> rc;
        for (const Pair& p : rough) {
            if (p.cam == ci && rc.size() < 6) {
                rc.push_back(&p);
            }
        }
        if (rc.size() < 3) {
            continue;
        }
        const g::Pose body_from_cam = rig.cams[ci].cam_from_body.inverse();
        for (std::size_t a = 0; a < rc.size() && hyps.size() < cfg.max_hyp; a++) {
            for (std::size_t b = a + 1; b < rc.size() && hyps.size() < cfg.max_hyp; b++) {
                for (std::size_t c = b + 1; c < rc.size() && hyps.size() < cfg.max_hyp; c++) {
                    const std::array<g::Vec3, 3> y{*obs[ci].bearing[rc[a]->blob], *obs[ci].bearing[rc[b]->blob],
                                                   *obs[ci].bearing[rc[c]->blob]};
                    const std::array<g::Vec3, 3> x{model.leds[rc[a]->led].p, model.leds[rc[b]->led].p,
                                                   model.leds[rc[c]->led].p};
                    for (const g::Pose& cfm : p3p(y, x)) {
                        if (cfm.t[2] <= 0.0 || g::norm(cfm.t) > 2.0) {
                            continue;
                        }
                        const g::Pose bfm = body_from_cam.compose(cfm);
                        if (g::norm(g::sub(bfm.t, pred_body.t)) > cfg.p3p_max_m ||
                            angle_deg(bfm.r, pred_body.r) > cfg.p3p_max_deg) {
                            continue;
                        }
                        hyps.push_back({bfm, 1});
                        counts.hyp_p3p += 1;
                    }
                }
            }
        }
    }
    // P2P: the predicted rotation, the translation from two rough matches (least squares on the two rays)
    if (blobs_total < 46) {
        for (std::size_t a = 0; a < rough.size() && hyps.size() < cfg.max_hyp; a++) {
            for (std::size_t b = a + 1; b < rough.size() && hyps.size() < cfg.max_hyp; b++) {
                // (I - r r^T)(u_k + Rcb t) = 0, u_k = Rcb R pl + tcb
                double A[6][3], rhs[6];
                for (int q = 0; q < 2; q++) {
                    const Pair& p = q == 0 ? rough[a] : rough[b];
                    const Cam& cam = rig.cams[p.cam];
                    const g::Vec3 r = *obs[p.cam].bearing[p.blob];
                    const g::Vec3 u = g::add(g::mat_vec(cam.cam_from_body.r, g::mat_vec(pred_body.r, model.leds[p.led].p)),
                                             cam.cam_from_body.t);
                    for (int i = 0; i < 3; i++) {
                        for (int j = 0; j < 3; j++) {
                            const double Pij = (i == j ? 1.0 : 0.0) - r[i] * r[j];
                            A[3 * q + i][j] = 0.0;
                            for (int k = 0; k < 3; k++) {
                                A[3 * q + i][j] += Pij * 0.0; // placeholder for clarity
                            }
                        }
                    }
                    // A_rows = (I - r r^T) Rcb ; rhs = -(I - r r^T) u
                    for (int i = 0; i < 3; i++) {
                        double ru = 0.0;
                        for (int k = 0; k < 3; k++) {
                            ru += r[k] * u[k];
                        }
                        rhs[3 * q + i] = -(u[i] - r[i] * ru);
                        for (int j = 0; j < 3; j++) {
                            double s = 0.0;
                            for (int k = 0; k < 3; k++) {
                                const double Pik = (i == k ? 1.0 : 0.0) - r[i] * r[k];
                                s += Pik * cam.cam_from_body.r[k][j];
                            }
                            A[3 * q + i][j] = s;
                        }
                    }
                }
                // normal equations 3x3
                g::Mat3 N{};
                g::Vec3 nb{0.0, 0.0, 0.0};
                for (int i = 0; i < 6; i++) {
                    for (int j = 0; j < 3; j++) {
                        nb[j] += A[i][j] * rhs[i];
                        for (int k = 0; k < 3; k++) {
                            N[j][k] += A[i][j] * A[i][k];
                        }
                    }
                }
                const double det = g::det(N);
                if (std::abs(det) < 1e-9) {
                    continue;
                }
                g::Mat3 inv{};
                inv[0] = g::scale(g::cross(N[1], N[2]), 1.0 / det);
                inv[1] = g::scale(g::cross(N[2], N[0]), 1.0 / det);
                inv[2] = g::scale(g::cross(N[0], N[1]), 1.0 / det);
                const g::Mat3 invT = g::transpose(inv);
                const g::Vec3 t = g::mat_vec(invT, nb);
                if (!std::isfinite(t[0]) || g::norm(g::sub(t, pred_body.t)) > cfg.p2p_max_m) {
                    continue;
                }
                hyps.push_back({g::Pose{pred_body.r, t}, 2});
                counts.hyp_p2p += 1;
            }
        }
    }
    // P1P: the predicted rotation, the translation moving one LED onto its blob's ray (the ray's nearest point)
    for (const Pair& p : rough) {
        if (hyps.size() >= cfg.max_hyp) {
            break;
        }
        const Cam& cam = rig.cams[p.cam];
        const g::Vec3 ray = *obs[p.cam].bearing[p.blob];
        const double s = g::dot(ray, p.pc);
        if (s <= 0.02) {
            continue;
        }
        const g::Vec3 shift_c = g::sub(g::scale(ray, s), p.pc);
        if (g::norm(shift_c) > cfg.p1p_max_m) {
            continue;
        }
        const g::Vec3 shift_b = g::mat_vec(g::transpose(cam.cam_from_body.r), shift_c);
        hyps.push_back({g::Pose{pred_body.r, g::add(pred_body.t, shift_b)}, 3});
        counts.hyp_p1p += 1;
    }
    counts.hyp_total += hyps.size();
    // score every hypothesis: greedy one-to-one within the distance gate, in all cameras
    std::vector<Match> scored;
    scored.reserve(hyps.size());
    for (const Hyp& h : hyps) {
        Match m;
        m.kind = h.kind;
        m.body_pose = h.pose;
        m.per_cam.assign(obs.size(), {});
        m.dt = g::norm(g::sub(h.pose.t, pred_body.t));
        m.dr = rf::to_radians(angle_deg(h.pose.r, pred_body.r));
        struct C3 {
            double d;
            std::size_t cam, led, blob;
        };
        std::vector<C3> cand;
        for (std::size_t ci = 0; ci < rig.cams.size() && ci < obs.size(); ci++) {
            const g::Pose cfm = rig.cams[ci].cam_from_body.compose(h.pose);
            double d_min = 1e9;
            std::vector<std::pair<std::size_t, std::pair<double, double>>> proj;
            for (std::size_t l = 0; l < model.leds.size(); l++) {
                g::Vec3 pc;
                const auto uv = project_visible(rig.cams[ci], cfm, model.leds[l], cos_vis, &pc);
                if (!uv) {
                    continue;
                }
                d_min = std::min(d_min, g::norm(pc));
                proj.emplace_back(l, *uv);
            }
            const double gate = gate_px(d_min) * cfg.gate_scale;
            for (const auto& [l, uv] : proj) {
                for (std::size_t b = 0; b < obs[ci].px.size(); b++) {
                    if (used[ci][b]) {
                        continue;
                    }
                    const double d = std::hypot(uv.first - obs[ci].px[b].first, uv.second - obs[ci].px[b].second);
                    if (d <= gate) {
                        cand.push_back({d, ci, l, b});
                    }
                }
            }
        }
        std::sort(cand.begin(), cand.end(), [](const C3& a, const C3& b) { return a.d < b.d; });
        std::vector<std::vector<bool>> bused(obs.size());
        for (std::size_t ci = 0; ci < obs.size(); ci++) {
            bused[ci].assign(obs[ci].px.size(), false);
        }
        std::vector<std::vector<bool>> lused(obs.size(), std::vector<bool>(model.leds.size(), false));
        for (const C3& c : cand) {
            if (bused[c.cam][c.blob] || lused[c.cam][c.led]) {
                continue;
            }
            bused[c.cam][c.blob] = true;
            lused[c.cam][c.led] = true;
            m.per_cam[c.cam].emplace_back(c.led, c.blob);
            m.count += 1;
            m.err += c.d;
        }
        scored.push_back(std::move(m));
    }
    if (scored.empty()) {
        return std::nullopt;
    }
    // the choice: most inliers, then the error, then nearness to the prediction
    std::size_t best = 0;
    for (std::size_t i = 1; i < scored.size(); i++) {
        const Match& a = scored[i];
        const Match& b = scored[best];
        if (a.count > b.count || (a.count == b.count && (a.err < b.err || (a.err == b.err && (a.dt < b.dt || (a.dt == b.dt && a.dr < b.dr)))))) {
            best = i;
        }
    }
    const Match& bm = scored[best];
    if (bm.dt > 0.09 && bm.dr > 0.1) {
        // far from the prediction: a near alternative with nearly as many inliers wins
        const std::size_t need = bm.count >= 10 ? bm.count - 3 : bm.count >= 2 ? bm.count - 2 : 0;
        const double dt_lim = std::clamp(0.5 * bm.dt, 0.015, 0.035);
        const double dr_lim = std::clamp(0.5 * bm.dr, 0.02, 0.05);
        std::optional<std::size_t> alt;
        double alt_score = 1e18;
        for (std::size_t i = 0; i < scored.size(); i++) {
            const Match& a = scored[i];
            if (i == best || a.count < need || a.dt > dt_lim || a.dr > dr_lim) {
                continue;
            }
            const double sc = static_cast<double>(bm.count - a.count) + a.err + 80.0 * a.dt + 20.0 * a.dr;
            if (sc < alt_score) {
                alt_score = sc;
                alt = i;
            }
        }
        if (alt) {
            best = *alt;
            counts.pick_alt += 1;
        }
    }
    Match out = scored[best];
    out.pred_count = scored[0].count;
    out.pred_err = scored[0].err;
    // the blobs the hypothesis should explain: free blobs inside the box of its projected LEDs (15 px margin), per camera
    for (std::size_t ci = 0; ci < rig.cams.size() && ci < obs.size(); ci++) {
        const g::Pose cfm = rig.cams[ci].cam_from_body.compose(out.body_pose);
        double u0 = 1e9, v0 = 1e9, u1 = -1e9, v1 = -1e9;
        std::size_t n = 0;
        for (const Led& led : model.leds) {
            if (const auto uv = project_visible(rig.cams[ci], cfm, led, cos_vis)) {
                u0 = std::min(u0, uv->first);
                v0 = std::min(v0, uv->second);
                u1 = std::max(u1, uv->first);
                v1 = std::max(v1, uv->second);
                n += 1;
            }
        }
        out.expected += n;
        if (n == 0) {
            continue;
        }
        for (std::size_t b = 0; b < obs[ci].px.size(); b++) {
            const auto& [u, v] = obs[ci].px[b];
            if (!used[ci][b] && u >= u0 - 15.0 && u <= u1 + 15.0 && v >= v0 - 15.0 && v <= v1 + 15.0) {
                out.box_blobs += 1;
            }
        }
    }
    if (out.count < cfg.min_leds) {
        counts.miss_one += 1;
        if (wt_debug()) {
            std::fprintf(stderr, "wt %llu slot %zu MISS one (pairs %zu rough %zu hyps %zu best %zu kind %d pred %zu/%.1f)\n",
                         static_cast<unsigned long long>(frame_no_), slot, pairs.size(), rough.size(), hyps.size(),
                         out.count, out.kind, out.pred_count, out.pred_err);
        }
        return std::nullopt;
    }
    switch (out.kind) {
    case 0: counts.pick_pred += 1; break;
    case 1: counts.pick_p3p += 1; break;
    case 2: counts.pick_p2p += 1; break;
    default: counts.pick_p1p += 1; break;
    }
    return out;
}

// ---------------------------------------------------------------- acquisition hypotheses (imutrack's port)

double WinTracker::hyp_err(std::size_t slot, const Hyp& h, bool& still) const {
    still = false;
    const auto& d = imu_[slot];
    const std::size_t n = h.f.size();
    if (n < 3 || d.empty()) {
        return -1.0;
    }
    const g::Vec3 up = h_[slot].up_w;
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
        g::Vec3 dv{0.0, 0.0, 0.0};
        double covered = 0.0;
        std::uint64_t prev = 0;
        // from the last sample at or before ta (the deque is sorted)
        auto it = std::lower_bound(d.begin(), d.end(), ta, [](const ImuIn& e, std::uint64_t x) { return e.t < x; });
        if (it != d.begin()) {
            --it;
        }
        for (; it != d.end(); ++it) {
            const ImuIn& s = *it;
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
        still = true;
        e = std::max(e, tilt_max / 20.0 * 0.5);
    }
    return e;
}

std::optional<std::pair<WinTracker::HypFrame, g::Vec3>> WinTracker::hyp_step(std::size_t slot, std::uint64_t frame_ns,
                                                                            const std::vector<HypFrame>& cands) {
    constexpr double thr = 0.5;
    constexpr std::uint32_t hyp_miss = 4;
    auto& hs = hyps_[slot];
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
        double dbg_e = 1e9, dbg_d = 1e9;
        for (std::size_t k = 0; k < cands.size(); k++) {
            const double e = rf::to_radians(angle_deg(rp, cands[k].r));
            const double dd = g::norm(g::sub(cands[k].p, l.p));
            if (e < dbg_e) {
                dbg_e = e;
                dbg_d = dd;
            }
            const double gyro_rad = cands[k].j.total() >= 8 ? cfg.hyp_gyro_rad : cfg.hyp_gyro_rad_few;
            if (e < gyro_rad && dd < 0.15 + 4.0 * dt && e < be) {
                be = e;
                best = k;
            }
        }
        if (wt_debug() && !cands.empty()) {
            std::fprintf(stderr, "wt %llu slot %zu hypchain len %zu misses %u dt %.2f nearest e %.1f deg d %.3f%s\n",
                         static_cast<unsigned long long>(frame_no_), slot, h.f.size(), h.misses, dt,
                         rf::to_degrees(dbg_e), dbg_d, best ? " EXT" : "");
        }
        if (best) {
            h.f.push_back(cands[*best]);
            if (h.f.size() > 12) {
                h.f.erase(h.f.begin());
            }
            h.misses = 0;
            taken[*best] = true;
        } else if (++h.misses >= hyp_miss) {
            h.f.clear();
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
    if (wt_debug()) {
        std::size_t with3 = 0;
        for (const Hyp& h : hs) {
            with3 += h.f.size() >= 3 && h.f.back().t == frame_ns ? 1 : 0;
        }
        std::fprintf(stderr, "wt %llu slot %zu hyp cands %zu hyps %zu current3 %zu scored %zu best e %.2f%s\n",
                     static_cast<unsigned long long>(frame_no_), slot, cands.size(), hs.size(), with3, sc.size(),
                     sc.empty() ? -1.0 : sc.front().e, sc.empty() ? "" : sc.front().still ? " still" : "");
    }
    if (sc.empty()) {
        return std::nullopt;
    }
    const Score& b = sc.front();
    if (b.e > thr) {
        counts.hyp_motion_fail += 1;
        return std::nullopt;
    }
    const HypFrame& bf = hs[b.i].f.back();
    for (std::size_t k = 1; k < sc.size(); k++) {
        const HypFrame& of = hs[sc[k].i].f.back();
        if (g::norm(g::sub(of.p, bf.p)) < 0.08 && sc[k].e < 1.6 * b.e + 0.05 && hs[b.i].f.size() < 12) {
            if (wt_debug()) {
                std::fprintf(stderr, "wt %llu slot %zu hyp WAIT twin e %.2f vs %.2f frames %zu\n",
                             static_cast<unsigned long long>(frame_no_), slot, sc[k].e, b.e, hs[b.i].f.size());
            }
            return std::nullopt; // the twin explains the IMU as well: wait for more motion
        }
    }
    const Hyp& h = hs[b.i];
    if (imu_live(1 - slot, frame_ns)) {
        double own = 0.0, oth = 0.0;
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
            n += 1;
        }
        if (n > 0 && own / static_cast<double>(n) > 0.03 && oth < 0.6 * own) {
            counts.hyp_other_hand += 1;
            if (wt_debug()) {
                std::fprintf(stderr, "wt %llu slot %zu hyp OTHER HAND own %.3f oth %.3f\n",
                             static_cast<unsigned long long>(frame_no_), slot, own / n, oth / n);
            }
            return std::nullopt;
        }
    }
    const HypFrame& f1 = h.f[h.f.size() - 2];
    const double dt = static_cast<double>(bf.t - f1.t) * 1e-9;
    const g::Vec3 v = dt > 1e-3 ? g::scale(g::sub(bf.p, f1.p), 1.0 / dt) : g::Vec3{0.0, 0.0, 0.0};
    std::pair<HypFrame, g::Vec3> out{bf, v};
    counts.hyp_accepted += 1;
    hs.clear();
    return out;
}

// ---------------------------------------------------------------- tentative relocks

namespace {
template <class PerCam>
std::size_t cams_seen(const PerCam& per_cam) {
    std::size_t n = 0;
    for (const auto& m : per_cam) {
        n += m.empty() ? 0 : 1;
    }
    return n;
}
} // namespace

void WinTracker::tent_save(Hand& hd, const Window& win_before) {
    hd.tentative = true;
    hd.tent_win = win_before;
    hd.tent_mode = hd.mode;
    hd.tent_lost_t = hd.lost_t;
    hd.tent_lost_count = hd.lost_count;
    hd.tent_lost_threshold = hd.lost_threshold;
    hd.tent_last_update = hd.last_update;
    hd.tent_good_p = hd.good_p;
    hd.tent_last_valid = hd.last_valid;
    hd.tent_frames_tracked = hd.frames_tracked;
    counts.relocks_tentative += 1;
}

void WinTracker::tent_revert(Hand& hd) {
    hd.win = *hd.tent_win;
    hd.mode = hd.tent_mode;
    hd.lost_t = hd.tent_lost_t;
    hd.lost_count = hd.tent_lost_count + 1; // the relock frame counts as lost
    hd.lost_threshold = hd.tent_lost_threshold;
    hd.last_update = hd.tent_last_update;
    hd.good_p = hd.tent_good_p;
    hd.last_valid = hd.tent_last_valid;
    hd.frames_tracked = hd.tent_frames_tracked;
    hd.tentative = false;
    hd.tent_win.reset();
    counts.relocks_reverted += 1;
}

// ---------------------------------------------------------------- the frame

std::vector<Fix> WinTracker::step(const Rig& rig, const LedFrame& lf, std::uint64_t frame_ns) {
    const auto t_step = std::chrono::steady_clock::now();
    struct StepTimer {
        Counts& c;
        std::chrono::steady_clock::time_point t0;
        ~StepTimer() {
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            c.step_ms += ms;
            c.step_max_ms = std::max(c.step_max_ms, ms);
        }
    } step_timer{counts, t_step};
    frame_no_ += 1;
    if (!twin_found_) {
        twin_found_ = true;
        for (std::size_t m = 0; m < 2 && m < rig.models.size(); m++) {
            twin_[m] = model_twin(rig.models[m]);
        }
    }
    fallback_.device_from_aim = device_from_aim;
    if ((!imu_live(0, frame_ns) && !imu_live(1, frame_ns)) || sat_sub(frame_ns, head_seen_) > 1'000'000'000) {
        counts.fallback_frames += 1;
        fallback_.wanted = wanted;
        return fallback_.step(rig, lf, frame_ns);
    }
    Stats& st = fallback_.stats;
    st.led_frames += 1;
    static const bool dbg = std::getenv("WT_DEBUG") != nullptr;

    std::vector<CamObs> obs;
    for (std::size_t ci = 0; ci < rig.cams.size(); ci++) { // one per rig camera (empty when the frame has fewer)
        obs.push_back(CamObs::make(rig.cams[ci], ci < lf.px.size() ? lf.px[ci] : std::vector<std::pair<double, double>>{}));
    }
    std::vector<std::vector<bool>> used;
    for (const CamObs& o : obs) {
        used.emplace_back(o.px.size(), false);
    }
    std::vector<Fix> out;
    std::vector<Found> pre;
    const auto head_at = [&](std::size_t s) {
        const ImuIn* im = imu_at(s, frame_ns);
        return im != nullptr ? im->world_from_head : g::Pose{};
    };

    // 1. the controllers with a window: Pimax's matcher from the IMU prediction, both before either claims blobs
    struct Pending {
        std::size_t s;
        WinState pred;
        g::Pose bfw;
        std::vector<ImuStep> steps;
        bool gap;
        std::optional<Match> m;
    };
    std::vector<Pending> pend;
    for (std::size_t s = 0; s < 2; s++) {
        Hand& hd = h_[s];
        if (!wanted[s] || hd.win.empty() || (hd.mode != Mode::Tracking && hd.mode != Mode::Lost) ||
            !imu_live(s, frame_ns)) {
            continue;
        }
        st.tries += 1;
        Pending p;
        p.s = s;
        p.bfw = head_at(s).inverse();
        p.steps = steps(s, hd.win.newest().t, frame_ns, &p.gap);
        p.pred = hd.win.predict(p.steps, frame_ns);
        const auto t0 = std::chrono::steady_clock::now();
        p.m = match(rig, s, p.pred, p.bfw, obs, used);
        counts.match_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        if (p.gap) {
            counts.imu_gaps += 1;
        }
        pend.push_back(std::move(p));
    }
    // contested blobs: to the controller whose LED lands nearer (Pimax: the nearer controller)
    if (pend.size() == 2 && pend[0].m && pend[1].m) {
        for (std::size_t ci = 0; ci < obs.size(); ci++) {
            auto& ma = pend[0].m->per_cam[ci];
            auto& mb = pend[1].m->per_cam[ci];
            for (auto ia = ma.begin(); ia != ma.end();) {
                auto ib = std::find_if(mb.begin(), mb.end(), [&](const auto& q) { return q.second == ia->second; });
                if (ib == mb.end()) {
                    ++ia;
                    continue;
                }
                counts.contested += 1;
                // distances of the blob to each controller's projected LED
                const auto dist = [&](const Pending& p, std::size_t led) {
                    const g::Pose cfm = rig.cams[ci].cam_from_body.compose(p.m->body_pose);
                    const auto uv = rig.cams[ci].intr.project(cfm.apply(rig.models[p.s].leds[led].p));
                    return uv ? std::hypot(uv->first - obs[ci].px[ia->second].first, uv->second - obs[ci].px[ia->second].second) : 1e9;
                };
                if (dist(pend[0], ia->first) <= dist(pend[1], ib->first)) {
                    mb.erase(ib);
                    pend[1].m->count -= 1;
                    ++ia;
                } else {
                    ia = ma.erase(ia);
                    pend[0].m->count -= 1;
                }
            }
        }
        for (Pending& p : pend) {
            if (p.m && p.m->count < cfg.min_leds) {
                p.m.reset();
            }
        }
    }
    for (Pending& p : pend) {
        Hand& hd = h_[p.s];
        const std::size_t s = p.s;
        const LedModel& model = rig.models[s];
        if (p.m && hd.mode == Mode::Lost) {
            // a relock needs min_leds_relock inliers; beyond relock_frames also the box rule: the match leaves at most
            // relock_box_frac of the free blobs where the controller would be unexplained
            const bool late = hd.lost_count >= cfg.relock_frames;
            const bool box_ok = cfg.relock_box_frac > 0.0 && p.m->box_blobs > 0 &&
                                static_cast<double>(p.m->box_blobs - std::min(p.m->box_blobs, p.m->count)) <=
                                    cfg.relock_box_frac * static_cast<double>(p.m->box_blobs);
            if (p.m->count < cfg.min_leds_relock || (late && !box_ok)) {
                counts.relock_short += 1;
                if (dbg) {
                    std::fprintf(stderr, "wt %llu slot %zu MISS relock short (n %zu lost %u box %zu expected %zu)\n",
                                 static_cast<unsigned long long>(frame_no_), s, p.m->count, hd.lost_count, p.m->box_blobs,
                                 p.m->expected);
                }
                p.m.reset();
            } else if (late) {
                counts.relock_late += 1;
            }
        }
        if (hd.tentative && p.m && p.m->count < cfg.tentative_min_leds) {
            p.m.reset(); // a weaker update does not confirm a tentative relock
        }
        if (!p.m && hd.tentative) {
            // the tentative relock did not hold: back to the lost state it came from, the search goes on
            if (dbg) {
                std::fprintf(stderr, "wt %llu slot %zu TENTATIVE reverted t %llu (lost %u before the relock)\n",
                             static_cast<unsigned long long>(frame_no_), s, static_cast<unsigned long long>(frame_ns),
                             hd.tent_lost_count);
            }
            tent_revert(hd);
            counts.misses += 1;
            continue;
        }
        const bool was_lost = hd.mode == Mode::Lost;
        if (!p.m) {
            counts.misses += 1;
            if (dbg) {
                std::fprintf(stderr, "wt %llu slot %zu miss t %llu mode %d lost %u pred (%.3f %.3f %.3f) |v| %.2f steps %zu%s\n",
                             static_cast<unsigned long long>(frame_no_), s, static_cast<unsigned long long>(frame_ns),
                             static_cast<int>(hd.mode), hd.lost_count, p.pred.p[0], p.pred.p[1], p.pred.p[2],
                             g::norm(p.pred.v), p.steps.size(), p.gap ? " GAP" : "");
            }
            if (hd.mode == Mode::Tracking) {
                hd.mode = Mode::Lost;
                hd.lost_t = frame_ns;
                hd.lost_count = 0;
            }
            continue;
        }
        // IMU stats, fast-stop (Pimax)
        const WinState& nw = hd.win.newest();
        const ImuStats is = imu_stats(s, frame_ns, nw.bg, nw.ba);
        bool strict = false, loose = false;
        if (is.ok) {
            const bool motion = is.mean_g > 1.0 || is.max_g > 1.5 || is.std_g > 0.35 || std::abs(is.mean_a - G) > 1.5 ||
                                is.std_a > 0.9 || is.max_dev > 3.0;
            loose = is.mean_g < 0.5 && is.std_g < 0.25 && std::abs(is.mean_a - G) < 0.8 && is.std_a < 0.65 &&
                    is.max_g < 1.0 && is.max_dev < 2.5;
            strict = is.mean_g < 0.15 && is.std_g < 0.08 && std::abs(is.mean_a - G) < 0.35 && is.std_a < 0.3 &&
                     is.max_g < 0.35 && is.max_dev < 1.0;
            const double speed = g::norm(p.pred.v);
            double rate = 0.0;
            if (hd.win.frames() >= 2) {
                const auto& fr = hd.win.frames_debug();
                const auto& a = fr[fr.size() - 2].x;
                const auto& b = fr[fr.size() - 1].x;
                const double dt = static_cast<double>(b.t - a.t) * 1e-9;
                rate = dt > 1e-3 ? g::norm(rotvec(g::mat_mul(g::transpose(a.r), b.r))) / dt : 0.0;
            }
            if (motion) {
                hd.fast_stop = false;
                hd.motion_cooldown = 12;
                hd.quiet_frames = 0;
                hd.slow_frames = 0;
            } else {
                hd.quiet_frames += loose ? 1 : 0;
                hd.slow_frames = speed < 0.12 && rate < 0.5 ? hd.slow_frames + 1 : 0;
                if (!hd.fast_stop && hd.motion_cooldown >= 1 && hd.quiet_frames >= 2 && hd.slow_frames >= 3) {
                    hd.fast_stop = true;
                }
                if (hd.fast_stop && (speed > 0.18 || rate > 0.8 || !loose)) {
                    hd.fast_stop = false;
                }
            }
            hd.motion_cooldown = std::max(0, hd.motion_cooldown - 1);
            if (is.mean_a > cfg.fast_a) {
                hd.fast = true;
                hd.slow_a_frames = 0;
            } else if (++hd.slow_a_frames > 10) {
                hd.fast = false;
            }
        }
        const bool still_strict = hd.fast_stop && strict;
        const bool still_loose = hd.fast_stop && loose;
        counts.still_strict += still_strict ? 1 : 0;
        counts.still_loose += still_loose ? 1 : 0;
        counts.fast_stop_frames += hd.fast_stop ? 1 : 0;
        // the window update
        std::vector<VisObs> vo;
        for (std::size_t ci = 0; ci < p.m->per_cam.size(); ci++) {
            for (const auto& [l, b] : p.m->per_cam[ci]) {
                vo.push_back(VisObs{&rig.cams[ci], model.leds[l].p, obs[ci].px[b].first, obs[ci].px[b].second});
            }
        }
        const auto t1 = std::chrono::steady_clock::now();
        const Window backup = hd.win;
        hd.win.push(frame_ns, p.bfw, std::move(vo), p.steps, still_strict, still_loose, hd.fast);
        counts.push_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t1).count();
        if (cfg.revert_rms_px > 0.0 && p.m->count >= 4 && hd.win.last_vis_rms_px > cfg.revert_rms_px) {
            if (dbg) {
                const WinState& nx = hd.win.newest();
                std::fprintf(stderr, "wt %llu slot %zu REVERTED t %llu n %zu (pred %zu err %.1f) kind %d dt %.3f dr %.1f rms %.2f cost %.1f v (%.2f %.2f %.2f) ba (%.2f %.2f %.2f) steps %zu%s\n",
                             static_cast<unsigned long long>(frame_no_), s, static_cast<unsigned long long>(frame_ns), p.m->count,
                             p.m->pred_count, p.m->pred_err, p.m->kind, p.m->dt, rf::to_degrees(p.m->dr), hd.win.last_vis_rms_px,
                             hd.win.last_cost, nx.v[0], nx.v[1], nx.v[2], nx.ba[0], nx.ba[1], nx.ba[2], p.steps.size(),
                             p.gap ? " GAP" : "");
            }
            hd.win = backup;
            counts.reverted += 1;
            counts.misses += 1;
            if (hd.mode == Mode::Tracking) {
                hd.mode = Mode::Lost;
                hd.lost_t = frame_ns;
                hd.lost_count = 0;
            }
            continue;
        }
        counts.updates += 1;
        counts.iter_sum += static_cast<double>(hd.win.last_iterations);
        counts.vis_rms_sum += hd.win.last_vis_rms_px;
        counts.vis_rms_n += 1;
        counts.inliers_sum += p.m->count;
        bool hold_fix = false;
        if (hd.tentative && hd.win.last_vis_rms_px > cfg.tentative_rms_px) {
            // the update holds the relock only loosely: back to the lost state
            if (dbg) {
                std::fprintf(stderr, "wt %llu slot %zu TENTATIVE reverted t %llu (rms %.2f with n %zu)\n",
                             static_cast<unsigned long long>(frame_no_), s, static_cast<unsigned long long>(frame_ns),
                             hd.win.last_vis_rms_px, p.m->count);
            }
            tent_revert(hd);
            counts.misses += 1;
            continue;
        }
        if (hd.tentative) {
            // held by this update: the relock stands
            hd.tentative = false;
            hd.tent_win.reset();
            counts.relocks_confirmed += 1;
            if (dbg) {
                std::fprintf(stderr, "wt %llu slot %zu TENTATIVE confirmed n %zu\n", static_cast<unsigned long long>(frame_no_), s, p.m->count);
            }
        } else if (was_lost && cfg.tentative_below > 0 &&
                   (p.m->count < cfg.tentative_below || (cfg.tentative_one_cam && cams_seen(p.m->per_cam) < 2))) {
            // the matcher's relock from few LEDs: held back until the next frame holds it
            tent_save(hd, backup);
            hold_fix = true;
            if (dbg) {
                std::fprintf(stderr, "wt %llu slot %zu TENTATIVE relock (matcher) n %zu lost %u\n",
                             static_cast<unsigned long long>(frame_no_), s, p.m->count, hd.lost_count);
            }
        }
        hd.last_update = frame_ns;
        hd.frames_tracked += 1;
        hd.last_valid = p.m->count;
        hd.mode = Mode::Tracking;
        if (p.m->count >= 5) {
            hd.good_p = hd.win.newest().p;
        }
        // the lost threshold for the next loss
        hd.lost_threshold = cfg.lost_default;
        if (p.m->count < 4) {
            hd.lost_threshold = cfg.lost_weak;
        }
        if (hd.frames_tracked <= cfg.first_frames) {
            hd.lost_threshold = std::min(hd.lost_threshold, cfg.lost_acquired);
        }
        // the state in the body frame: the fix
        const WinState& x = hd.win.newest();
        const g::Pose body{g::mat_mul(p.bfw.r, x.r), p.bfw.apply(x.p)};
        {
            // an LED within 5 cm of camera 0 or 1: Pimax's threshold 0; the controllers close in an image: 15
            for (std::size_t ci = 0; ci < 2 && ci < rig.cams.size(); ci++) {
                const g::Pose cfm = rig.cams[ci].cam_from_body.compose(body);
                for (const Led& led : model.leds) {
                    if (g::norm(cfm.apply(led.p)) < 0.05) {
                        hd.lost_threshold = 0;
                    }
                }
            }
        }
        // the learnt up for the acquisition tilt test (the controller's accelerometer in the world)
        if (const ImuIn* im = imu_at(s, frame_ns); im != nullptr && im->up_ok && p.m->count >= 8) {
            const g::Vec3 u = g::mat_vec(g::mat_mul(x.r, device_from_aim), im->up_aim);
            const double k = std::max(0.02, 1.0 / static_cast<double>(hd.up_n + 1));
            hd.up_w = g::normalize(g::add(g::scale(hd.up_w, 1.0 - k), g::scale(u, k)));
            hd.up_n += 1;
        }
        JointSolve j{body, p.m->per_cam, hd.win.last_vis_rms_px};
        claim(used, j);
        Found f{s, j, std::nullopt, 0.0, 0};
        pre.push_back(f);
        if (!hold_fix) {
            Fix fx{s, f, p.m->count < 5, body, g::mat_vec(p.bfw.r, x.v)};
            st.hand[s] += 1;
            if (fx.few) {
                st.few[s] += 1;
            }
            last_fix_ns_ = frame_ns;
            out.push_back(std::move(fx));
        }
        if (dbg) {
            std::fprintf(stderr, "wt %llu slot %zu upd t %llu n %zu (pred %zu err %.1f) kind %d dt %.3f dr %.1f rms %.2f cost %.1f v (%.2f %.2f %.2f) ba (%.2f %.2f %.2f) bg (%.3f %.3f %.3f) steps %zu%s\n",
                         static_cast<unsigned long long>(frame_no_), s, static_cast<unsigned long long>(frame_ns), p.m->count, p.m->pred_count, p.m->pred_err, p.m->kind,
                         p.m->dt, rf::to_degrees(p.m->dr), hd.win.last_vis_rms_px, hd.win.last_cost, x.v[0], x.v[1], x.v[2],
                         x.ba[0], x.ba[1], x.ba[2], x.bg[0], x.bg[1], x.bg[2], p.steps.size(), hd.fast_stop ? " still" : "");
        }
    }
    // the two controllers close in an image: the weak threshold for both
    if (pend.size() == 2 && pend[0].m && pend[1].m) {
        for (std::size_t ci = 0; ci < rig.cams.size(); ci++) {
            const auto ca = rig.cams[ci].intr.project(rig.cams[ci].cam_from_body.apply(pend[0].m->body_pose.t));
            const auto cb = rig.cams[ci].intr.project(rig.cams[ci].cam_from_body.apply(pend[1].m->body_pose.t));
            if (ca && cb && std::hypot(ca->first - cb->first, ca->second - cb->second) < cfg.close_px) {
                h_[0].lost_threshold = std::min(h_[0].lost_threshold, cfg.lost_weak);
                h_[1].lost_threshold = std::min(h_[1].lost_threshold, cfg.lost_weak);
            }
        }
    }

    // 2. lost and initing controllers
    std::array<bool, 2> need{false, false};
    std::array<bool, 2> cold{false, false};
    for (std::size_t s = 0; s < 2; s++) {
        Hand& hd = h_[s];
        if (!wanted[s] || !imu_live(s, frame_ns) || hd.last_update == frame_ns) {
            continue;
        }
        if (hd.mode == Mode::None) {
            hd.mode = Mode::Initing;
        }
        if (hd.mode == Mode::Lost) {
            counts.lost_frames += 1;
            const WinState pr = cfg.far_from_last ? hd.win.newest()
                                                  : hd.win.predict(steps(s, hd.win.newest().t, frame_ns, nullptr), frame_ns);
            const g::Vec3 head_p = head_at(s).t;
            const double budget_s = std::min(cfg.lost_max_s, hd.lost_threshold * 0.0333);
            const double since = static_cast<double>(sat_sub(frame_ns, hd.lost_t)) * 1e-9;
            if (g::norm(g::sub(pr.p, head_p)) > cfg.far_m) {
                hd.mode = Mode::Initing;
                counts.far_resets += 1;
                if (dbg) {
                    std::fprintf(stderr, "wt %llu slot %zu FAR reset t %llu dist %.2f |v| %.2f lost %u since %.2f\n",
                                 static_cast<unsigned long long>(frame_no_), s, static_cast<unsigned long long>(frame_ns),
                                 g::norm(g::sub(pr.p, head_p)), g::norm(pr.v), hd.lost_count, since);
                }
            } else if (since > budget_s || hd.lost_count >= hd.lost_threshold) {
                hd.mode = Mode::Initing;
                counts.budget_resets += 1;
                if (dbg) {
                    std::fprintf(stderr, "wt %llu slot %zu BUDGET reset t %llu lost %u thr %u since %.2f\n",
                                 static_cast<unsigned long long>(frame_no_), s, static_cast<unsigned long long>(frame_ns),
                                 hd.lost_count, hd.lost_threshold, since);
                }
            } else {
                hd.lost_count += 1;
            }
        }
        if (hd.mode == Mode::Initing) {
            counts.initing_frames += 1;
        }
        const ImuIn* im = imu_at(s, frame_ns);
        need[s] = im != nullptr && (hd.mode == Mode::Lost || hd.mode == Mode::Initing) &&
                  (im->still_s < 5.0 || frame_no_ % 5 == 0);
        cold[s] = hd.mode == Mode::Initing;
    }
    const bool idle = out.empty() && sat_sub(frame_ns, last_fix_ns_) > 2'000'000'000;
    if ((need[0] || need[1]) && (!idle || frame_no_ % 3 == 0)) {
        std::array<std::optional<OrientPrior>, 2> orient{};
        std::array<std::optional<WinState>, 2> pred{};
        for (std::size_t s = 0; s < 2; s++) {
            Hand& hd = h_[s];
            const ImuIn* im = imu_at(s, frame_ns);
            if (!need[s] || im == nullptr) {
                continue;
            }
            const g::Mat3 hw_r = g::transpose(im->world_from_head.r);
            OrientPrior op;
            op.tilt_deg = 20.0;
            op.rot_deg = 30.0;
            if (im->up_ok) {
                const bool learnt = hd.up_n >= 20;
                op.up = std::pair{g::mat_vec(device_from_aim, im->up_aim),
                                  g::mat_vec(hw_r, learnt ? hd.up_w : g::Vec3{0.0, 1.0, 0.0})};
                op.tilt_deg = learnt ? 20.0 : 75.0;
            }
            // warm: the gyro-carried orientation is known (imutrack's known()): updated within warm_s, and the last
            // update's place within the far rule of the head (the IMU-only prediction may have run off since; a
            // controller updated far away is searched for from scratch). Lost or Initing alike.
            const bool warm = !hd.win.empty() && hd.last_update != 0 &&
                              static_cast<double>(sat_sub(frame_ns, hd.last_update)) * 1e-9 < cfg.warm_s &&
                              g::norm(g::sub(hd.good_p.value_or(hd.win.newest().p), im->world_from_head.t)) <= cfg.far_m;
            if (warm) {
                // the gyro-carried orientation (good for seconds) and the predicted position constrain the search
                pred[s] = hd.win.predict(steps(s, hd.win.newest().t, frame_ns, nullptr), frame_ns);
                op.r = g::mat_mul(hw_r, pred[s]->r);
                if (twin_[s]) {
                    op.r_twin = g::mat_mul(hw_r, g::mat_mul(pred[s]->r, g::transpose(twin_[s]->first.r)));
                }
                cold[s] = false;
            } else {
                cold[s] = true; // lost too long ago for the gyro to vouch: a cold acquisition
            }
            if (op.up || op.r) {
                orient[s] = op;
            }
        }
        const auto t2 = std::chrono::steady_clock::now();
        const std::vector<Found> got =
            solve_frame(lf, rig.cams, rig.models, rig.triples, rig.prm, nullptr, pre, true, &orient, &need);
        counts.search_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t2).count();
        st.searches += 1;
        std::array<std::vector<HypFrame>, 2> cold_c;
        for (std::size_t s = 0; s < 2; s++) {
            if (!need[s]) {
                continue;
            }
            Hand& hd = h_[s];
            const ImuIn* im = imu_at(s, frame_ns);
            const g::Pose wh = im->world_from_head;
            const g::Pose bfw = wh.inverse();
            std::optional<JointSolve> best;
            double best_d = 1e9;
            bool best_strong = false;
            for (const Found& gf : got) {
                const std::optional<JointSolve> j = gf.model == s ? std::optional<JointSolve>(gf.joint) : gf.alt;
                if (!j || j->total() < 5 || j->rms_px > rig.prm.tol_px) {
                    continue;
                }
                // not on the other controller
                bool overlap = false;
                for (const Found& p : pre) {
                    overlap = overlap || (p.model != s && g::norm(g::sub(p.joint.body_from_model.t, j->body_from_model.t)) < 0.08);
                }
                if (overlap) {
                    continue;
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
                    // the twin, refit
                    if (twin_[s]) {
                        const LedModel& model = rig.models[s];
                        const g::Mat3 rti = g::transpose(twin_[s]->first.r);
                        g::Pose p{g::mat_mul(j->body_from_model.r, rti),
                                  g::sub(j->body_from_model.t, g::mat_vec(g::mat_mul(j->body_from_model.r, rti), twin_[s]->first.t))};
                        const double cos_face = std::cos(rf::to_radians(rig.prm.facing_deg));
                        std::vector<Matches> m = match_all(p, rig, model, obs, used, cos_face, rig.prm.tol_px * 4.0);
                        if (count(m) >= 5) {
                            p = refine(p, obs_list(rig, model, obs, m), true, 20).first;
                            std::vector<Matches> m2 = match_all(p, rig, model, obs, used, cos_face, rig.prm.tol_px * 2.0);
                            const auto [p3, rms3] = refine(p, obs_list(rig, model, obs, m2), true, 20);
                            if (count(m2) >= 5 && rms3 <= rig.prm.tol_px) {
                                add(JointSolve{p3, std::move(m2), rms3});
                            }
                        }
                    }
                    continue;
                }
                // warm: the fit must agree with the gyro-carried orientation, and lie near the prediction unless it
                // is strong (relock_strong LEDs) and clear of the other controller
                std::optional<JointSolve> jj = j;
                const g::Mat3 rw = g::mat_mul(wh.r, j->body_from_model.r);
                double e = angle_deg(rw, pred[s]->r);
                const double age = static_cast<double>(sat_sub(frame_ns, hd.last_update)) * 1e-9;
                const char* why = nullptr;
                if (e > 30.0 && cfg.refit_turned) {
                    // the fit's orientation disagrees with the gyro's: the LED assignment turned half a turn onto the
                    // ring's twin, or a few-LED fit of a far controller (they scatter 20-40 deg on the live dumps).
                    // Refit from the gyro's orientation at the fit's place (12 / 5 / 2.5 px windows, as the tracker's
                    // prior path); the LEDs decide whether that holds, and the result must agree with the gyro.
                    const bool twin = twin_[s] && angle_deg(rw, g::mat_mul(pred[s]->r, g::transpose(twin_[s]->first.r))) < e;
                    g::Pose start{g::mat_mul(g::transpose(wh.r), pred[s]->r), j->body_from_model.t};
                    std::optional<JointSolve> r;
                    for (const double tol : {rig.prm.prior_tol_px, rig.prm.tol_px * 2.0, rig.prm.tol_px}) {
                        r = joint_from_prior(start, rig.models[s], rig.cams, obs, used, tol, rig.prm.facing_deg);
                        if (!r) {
                            break;
                        }
                        start = r->body_from_model;
                    }
                    if (r && r->total() >= 5 && r->rms_px <= rig.prm.tol_px &&
                        angle_deg(g::mat_mul(wh.r, r->body_from_model.r), pred[s]->r) <= 30.0) {
                        jj = *r;
                        e = angle_deg(g::mat_mul(wh.r, jj->body_from_model.r), pred[s]->r);
                        counts.relocks_twin += twin ? 1 : 0;
                        counts.relocks_refit += 1;
                    } else {
                        why = twin ? "twin refit" : "turned";
                    }
                } else if (e > 30.0) {
                    why = "turned";
                }
                const g::Vec3 pw = why == nullptr ? wh.apply(jj->body_from_model.t) : g::Vec3{0.0, 0.0, 0.0};
                const double d = why == nullptr ? g::norm(g::sub(pw, pred[s]->p)) : 0.0;
                if (why == nullptr && im->up_ok && hd.up_n >= 20) {
                    const g::Vec3 u = g::mat_vec(g::mat_mul(g::mat_mul(wh.r, jj->body_from_model.r), device_from_aim), im->up_aim);
                    if (g::angle_deg(u, hd.up_w) > 20.0) {
                        why = "tilt";
                    }
                }
                // the other controller: its predicted place (tracked or lost, updated within 0.5 s)
                double d_other = 1e9;
                if (const Hand& oh = h_[1 - s];
                    why == nullptr && !oh.win.empty() && (oh.mode == Mode::Tracking || oh.mode == Mode::Lost) &&
                    sat_sub(frame_ns, oh.last_update) < 500'000'000ull) {
                    const WinState op = oh.win.predict(steps(1 - s, oh.win.newest().t, frame_ns, nullptr), frame_ns);
                    d_other = g::norm(g::sub(pw, op.p));
                    if (d_other < 0.08) {
                        why = "on the other controller";
                    }
                }
                // the position gate holds while the prediction is credible: fresh, and not run off past the far rule
                // (on the live dumps the IMU-only prediction stood 5-80 m away after 1-5 s: the window's accelerometer
                // bias of several m/s^2 integrated). A strong fit, or any fit of this hand's own model while the
                // prediction is not credible, passes on the orientation and the other-controller clearance alone.
                // (Letting any own-model fit through while the prediction is not credible was tried on win2: 5-LED
                // fits 0.8-1.1 m above the head were accepted, hand 1 lost 32 -> 43 s and 6 more >15 cm episodes.)
                const bool strong = why == nullptr && gf.model == s && d_other > 0.15 && jj->total() >= cfg.relock_strong;
                if (why == nullptr && jj->cams() < 2 &&
                    e > std::min(30.0, cfg.warm_rot_deg + cfg.warm_rot_deg_per_s * age)) {
                    why = "turned for the age (one camera)"; // the gyro carries the orientation to degrees per second
                }
                if (why == nullptr && d > 0.15 + 1.0 * age && !strong) {
                    why = "far from the prediction";
                }
                if (dbg) {
                    std::fprintf(stderr, "wt %llu slot %zu warmfit n %zu rms %.2f e %.1f deg d %.3f age %.2f other %.2f%s%s\n",
                                 static_cast<unsigned long long>(frame_no_), s, jj ? jj->total() : j->total(),
                                 jj ? jj->rms_px : j->rms_px, e, d, age, d_other, why != nullptr ? " REFUSED " : "",
                                 why != nullptr ? why : "");
                }
                if (why != nullptr) {
                    counts.relock_refused += 1;
                    continue;
                }
                // the best: the most LEDs, then the nearest to the prediction
                if (!best || jj->total() > best->total() || (jj->total() == best->total() && d < best_d)) {
                    best_d = d;
                    best = *jj;
                    best_strong = strong && d > 0.15 + 1.0 * age;
                }
            }
            if (dbg) {
                std::fprintf(stderr, "wt %llu slot %zu search t %llu %s found %zu cands %zu best %s hyps %zu\n",
                             static_cast<unsigned long long>(frame_no_), s, static_cast<unsigned long long>(frame_ns),
                             cold[s] ? "cold" : "warm", got.size(), cold_c[s].size(), best ? "yes" : "no", hyps_[s].size());
            }
            std::optional<WinState> start;
            std::optional<JointSolve> start_j;
            if (cold[s]) {
                const auto t3 = std::chrono::steady_clock::now();
                const auto acc = hyp_step(s, frame_ns, cold_c[s]);
                counts.hyp_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t3).count();
                if (acc) {
                    WinState x;
                    x.t = frame_ns;
                    x.r = acc->first.r;
                    x.p = acc->first.p;
                    x.v = acc->second;
                    if (!hd.win.empty()) {
                        x.bg = hd.win.newest().bg;
                        x.ba = hd.win.newest().ba;
                        if (const double nb = g::norm(x.ba); nb > 1.0) {
                            x.ba = g::scale(x.ba, 1.0 / nb); // within what a bias can be (see the relock below)
                        }
                    }
                    start = x;
                    start_j = acc->first.j;
                    counts.acquisitions += 1;
                    if (hd.up_n < 20 && im->up_ok) {
                        hd.up_w = g::normalize(g::mat_vec(g::mat_mul(x.r, device_from_aim), im->up_aim));
                    }
                }
            } else if (best) {
                WinState x;
                x.t = frame_ns;
                x.r = g::mat_mul(wh.r, best->body_from_model.r);
                x.p = wh.apply(best->body_from_model.t);
                // the prediction's velocity only while fresh and plausible: after 2-3 LED updates the window's
                // velocity ran to 5-40 m/s on the live dumps, and carried into the restart it threw the prediction
                // past the far rule within a frame (relock, far reset, relock, ...). The first frame's velocity prior
                // (init_v) lets the window find the true one from the next fits.
                const double age = static_cast<double>(sat_sub(frame_ns, hd.last_update)) * 1e-9;
                x.v = age <= 0.2 && g::norm(pred[s]->v) < 6.0 ? pred[s]->v : g::Vec3{0.0, 0.0, 0.0};
                x.bg = hd.win.newest().bg;
                // the accelerometer bias within what a bias can be (the window's ran to 8-9 m/s^2 in fast motion)
                x.ba = hd.win.newest().ba;
                if (const double nb = g::norm(x.ba); nb > 1.0) {
                    x.ba = g::scale(x.ba, 1.0 / nb);
                }
                start = x;
                start_j = *best;
                counts.relocks += 1;
                counts.relocks_initing += hd.mode == Mode::Initing ? 1 : 0;
                counts.relocks_strong += best_strong ? 1 : 0;
            }
            if (start) {
                // a warm relock from few LEDs is tentative: the next frame's matcher update must hold it
                const bool tentative = !cold[s] && cfg.tentative_below > 0 &&
                                       (start_j->total() < cfg.tentative_below ||
                                        (cfg.tentative_one_cam && start_j->cams() < 2));
                if (tentative) {
                    tent_save(hd, hd.win);
                }
                hd.win.reset(*start);
                hd.mode = Mode::Tracking;
                hd.last_update = frame_ns;
                hd.good_p = start->p;
                hd.frames_tracked = 0;
                hd.last_valid = start_j->total();
                hd.lost_threshold = cfg.lost_acquired;
                hd.fast_stop = false;
                claim(used, *start_j);
                Found f{s, *start_j, std::nullopt, 0.0, 0};
                pre.push_back(f);
                if (!tentative) {
                    Fix fx{s, f, false, start_j->body_from_model, g::mat_vec(bfw.r, start->v)};
                    st.hand[s] += 1;
                    last_fix_ns_ = frame_ns;
                    out.push_back(std::move(fx));
                }
                if (dbg) {
                    std::fprintf(stderr, "wt %llu slot %zu %s leds %zu at (%.3f %.3f %.3f)%s\n",
                                 static_cast<unsigned long long>(frame_no_), s, cold[s] ? "ACQUIRED" : "relocked",
                                 start_j->total(), start->p[0], start->p[1], start->p[2], tentative ? " TENTATIVE" : "");
                }
            }
        }
    }
    return out;
}

std::string WinTracker::describe_slots() const {
    std::string o;
    static const char* names[] = {"none", "initing", "tracking", "lost"};
    for (std::size_t s = 0; s < 2; s++) {
        const Hand& h = h_[s];
        o += std::format("{}{} {} frames {} window {} thr {}{}", s == 0 ? "" : "; ", s == 0 ? "left" : "right",
                         names[static_cast<int>(h.mode)], h.frames_tracked, h.win.frames(), h.lost_threshold,
                         h.fast_stop ? " still" : "");
    }
    return o;
}

} // namespace pibridge::ctrl::track
