// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/tracker.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/track/tracker.h"

#include "pibridge/cam/frame.h"
#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace pibridge::ctrl::track {

namespace g = cam::geom;
namespace rf = cam::rfmt;

namespace {

constexpr std::uint64_t STALE_NS = 300'000'000;
constexpr double MAX_EXTRAP_S = 0.12;
constexpr double MAX_V = 3.0;
constexpr double MAX_W = 10.0;

std::uint64_t sat_sub(std::uint64_t a, std::uint64_t b) { return a > b ? a - b : 0; }

g::Vec3 cap(const g::Vec3& v, double m) {
    const double n = g::norm(v);
    if (n > m) {
        return g::scale(v, m / n);
    }
    return v;
}

std::vector<Obs> obs_of(const Rig& rig, const LedModel& model, const std::vector<CamObs>& obs,
                        const std::vector<Matches>& per_cam) {
    std::vector<Obs> o;
    for (std::size_t ci = 0; ci < per_cam.size(); ci++) {
        for (const auto& [l, b] : per_cam[ci]) {
            o.push_back(Obs{&rig.cams[ci], model.leds[l].p, obs[ci].px[b]});
        }
    }
    return o;
}

std::size_t total_of(const std::vector<Matches>& m) {
    std::size_t t = 0;
    for (const auto& x : m) {
        t += x.size();
    }
    return t;
}

/// The assignment before the body model (A/B: `--no-body`): two new poses by x, one into the only free slot, else by
/// side / shape label.
std::vector<std::pair<std::size_t, std::size_t>> legacy_assign(const std::vector<Cand>& cands,
                                                               const std::array<Slot, 2>& slots) {
    std::vector<std::size_t> free;
    for (std::size_t s = 0; s < 2; s++) {
        if (!slots[s].tracked_now) {
            free.push_back(s);
        }
    }
    std::vector<std::size_t> idx(cands.size());
    for (std::size_t i = 0; i < idx.size(); i++) {
        idx[i] = i;
    }
    std::stable_sort(idx.begin(), idx.end(), [&](std::size_t a, std::size_t b) { return cands[a].p[0] < cands[b].p[0]; });
    if (idx.size() == 2 && free.size() == 2) {
        return {{idx[0], 0}, {idx[1], 1}};
    }
    std::vector<std::pair<std::size_t, std::size_t>> out;
    for (const std::size_t c : idx) {
        std::size_t slot = 0;
        if (free.empty()) {
            break;
        }
        if (free.size() == 1) {
            slot = free[0];
        } else {
            const double x = cands[c].p[0];
            slot = x < -0.08 ? 0 : x > 0.08 ? 1 : cands[c].model;
        }
        std::erase(free, slot);
        out.emplace_back(c, slot);
    }
    return out;
}

} // namespace

TrackCfg TrackCfg::baseline() {
    TrackCfg c;
    c.predict = false;
    c.wide_gate_px = 0.0;
    c.few_leds = 5;
    c.few_streak_max = 0;
    c.rot_w = 0.0;
    c.body = true;
    c.fail_limit = 0;
    c.far_m = std::numeric_limits<double>::infinity();
    c.far_leds = 4;
    return c;
}

TrackCfg TrackCfg::improved() {
    TrackCfg c;
    c.slot_model = true;
    c.imu_id = true;
    // the fuser's hint (use_hint) was not better on the recordings with the synthetic IMU: off until a live take
    // with the real IMU says otherwise (PIBRIDGE_CTRL_TRACK_HINT=1 in live)
    return c;
}

bool cold_start_ok(const TrackCfg& cfg, double dist_m, std::size_t leds) {
    return dist_m <= cfg.far_m || leds >= cfg.far_leds;
}

bool FailCount::frame(std::size_t slot, bool ok, std::uint32_t limit) {
    if (ok || limit == 0) {
        n[slot] = 0;
        return false;
    }
    n[slot] += 1;
    if (n[slot] >= limit) {
        n[slot] = 0;
        return true;
    }
    return false;
}

Tracker::Tracker(TrackCfg c) : cfg(c) {}

std::optional<std::tuple<std::size_t, g::Pose, std::uint64_t>> Tracker::last(std::size_t slot) const {
    if (!prev_[slot]) {
        return std::nullopt;
    }
    return std::tuple{prev_[slot]->model, prev_[slot]->pose, prev_[slot]->ns};
}

g::Pose Tracker::predicted(const SlotState& st, std::uint64_t frame_ns) const {
    const double dt = rf::fmin(static_cast<double>(sat_sub(frame_ns, st.ns)) * 1e-9, MAX_EXTRAP_S);
    if (cfg.predict && st.vel) {
        const auto& [v, w] = *st.vel;
        return g::Pose{g::mat_mul(st.pose.r, g::rodrigues(g::scale(w, dt))), g::add(st.pose.t, g::scale(v, dt))};
    }
    return st.pose;
}

std::optional<std::pair<Found, bool>> Tracker::track_slot(const Rig& rig, const std::vector<CamObs>& obs,
                                                          const std::vector<std::vector<bool>>& used,
                                                          const SlotState& st, std::uint64_t frame_ns,
                                                          const Hint* h) const {
    const Params& prm = rig.prm;
    const LedModel& model = rig.models[st.model];
    // with a hint: the fuser's pose (IMU orientation, inertial position) is the prediction
    const g::Pose pred = h != nullptr ? h->body_from_model : predicted(st, frame_ns);
    const double dt = static_cast<double>(sat_sub(frame_ns, st.ns)) * 1e-9;
    const double speed = st.vel ? g::norm(st.vel->first) : 0.0;
    std::vector<double> gates{prm.prior_tol_px};
    if (cfg.wide_gate_px > prm.prior_tol_px && (speed > 0.3 || dt > 0.06)) {
        gates.push_back(cfg.wide_gate_px);
    }
    std::vector<g::Pose> starts{pred};
    if (h != nullptr) {
        starts.push_back(predicted(st, frame_ns));
    } else if (cfg.predict && st.vel) {
        starts.push_back(st.pose);
    }
    const auto found = [&](JointSolve j) { return Found{st.model, std::move(j), std::nullopt, 0.0, 0}; };
    const double cos_face = std::cos(rf::to_radians(prm.facing_deg));
    std::optional<JointSolve> best_few;
    for (const g::Pose& start : starts) {
        for (const double gate : gates) {
            const std::vector<Matches> per_cam = match_all(start, rig, model, obs, used, cos_face, gate);
            const std::size_t total = total_of(per_cam);
            if (total >= 4) {
                // the original path: free 6DoF refine, rematch at the normal tolerance, 5+ LEDs
                const g::Pose p1 = refine(start, obs_of(rig, model, obs, per_cam), true, 25).first;
                std::vector<Matches> m2 = match_all(p1, rig, model, obs, used, cos_face, prm.tol_px);
                const std::size_t n2 = total_of(m2);
                if (n2 >= 4) {
                    const auto [p2, rms] = refine(p1, obs_of(rig, model, obs, m2), true, 25);
                    if (n2 >= 5 && rms <= prm.tol_px) {
                        return std::pair{found(JointSolve{p2, std::move(m2), rms}), false};
                    }
                }
            }
            const std::size_t few_min = h != nullptr ? cfg.hint_few_leds : cfg.few_leds;
            if (few_min < 5 && !best_few && total >= few_min && st.few_streak < cfg.few_streak_max) {
                best_few = few_led(rig, model, obs, used, pred, per_cam, dt, cos_face, h);
            }
        }
    }
    if (best_few) {
        return std::pair{found(std::move(*best_few)), true};
    }
    return std::nullopt;
}

std::optional<JointSolve> Tracker::few_led(const Rig& rig, const LedModel& model, const std::vector<CamObs>& obs,
                                           const std::vector<std::vector<bool>>& used, const g::Pose& pred,
                                           const std::vector<Matches>& per_cam, double dt, double cos_face,
                                           const Hint* h) const {
    const Params& prm = rig.prm;
    const std::optional<std::pair<g::Mat3, double>> prior = std::pair{pred.r, h != nullptr ? cfg.hint_rot_w : cfg.rot_w};
    const g::Pose p1 = refine_with(pred, obs_of(rig, model, obs, per_cam), true, 20, prior).first;
    std::vector<Matches> m2 = match_all(p1, rig, model, obs, used, cos_face, prm.tol_px * 1.5);
    const std::size_t n2 = total_of(m2);
    const std::size_t few_min = h != nullptr ? cfg.hint_few_leds : cfg.few_leds;
    if (n2 < few_min) {
        return std::nullopt;
    }
    if (h != nullptr && n2 < 3) {
        // two LEDs: the orientation is the gyro's, the position has 4 residuals for 3 unknowns: keep it close
        const auto [p2, rms] = refine_with(p1, obs_of(rig, model, obs, m2), true, 20, prior);
        const double moved = g::norm(g::sub(p2.t, pred.t));
        if (rms <= 1.0 && moved <= 0.02 + 0.5 * dt) {
            return JointSolve{p2, std::move(m2), rms};
        }
        return std::nullopt;
    }
    const auto [p2, rms] = refine_with(p1, obs_of(rig, model, obs, m2), true, 20, prior);
    const double moved = g::norm(g::sub(p2.t, pred.t));
    const double turned = g::norm(rotvec(g::mat_mul(p2.r, g::transpose(pred.r))));
    // 3 LEDs leave little redundancy: demand a tighter fit
    const double rms_max = n2 == 3 ? 1.2 : 1.8;
    if (rms <= rms_max && moved <= 0.03 + 0.75 * dt && turned <= rf::to_radians(20.0)) {
        return JointSolve{p2, std::move(m2), rms};
    }
    return std::nullopt;
}

std::vector<Fix> Tracker::step(const Rig& rig, const LedFrame& lf, std::uint64_t frame_ns) {
    frame_no_ += 1;
    stats.led_frames += 1;
    for (auto& p : prev_) {
        if (p && sat_sub(frame_ns, p->ns) > STALE_NS) {
            p.reset();
        }
    }
    std::vector<CamObs> obs;
    for (std::size_t ci = 0; ci < rig.cams.size() && ci < lf.px.size(); ci++) {
        obs.push_back(CamObs::make(rig.cams[ci], lf.px[ci]));
    }
    std::vector<std::vector<bool>> used;
    for (const CamObs& o : obs) {
        used.emplace_back(o.px.size(), false);
    }
    struct T {
        std::size_t slot;
        Found f;
        bool few;
        /// fitted with the gyro's orientation (imu_refit): not an anchor for the identity test
        bool refit = false;
    };
    std::vector<T> tracked;
    for (std::size_t slot = 0; slot < 2; slot++) {
        if (!wanted[slot]) {
            prev_[slot].reset();
            continue;
        }
        if (!prev_[slot]) {
            continue;
        }
        const SlotState st = *prev_[slot];
        stats.tries += 1;
        // the hint only replaces the tracker's own prediction once that is stale (a gap of 2+ LED frames): fresh
        // constant-velocity priors from the last fix were better on the recordings
        const bool stale = sat_sub(frame_ns, st.ns) > static_cast<std::uint64_t>(cfg.hint_stale_s * 1e9);
        const Hint* h = cfg.use_hint && hint[slot] && stale ? &*hint[slot] : nullptr;
        auto r = track_slot(rig, obs, used, st, frame_ns, h);
        bool refit_now = false;
        if (r && imu_refuse(slot, r->first.joint.body_from_model, frame_ns) != nullptr) {
            // the IMU says this is not the hand (a mirror fit, the other hand): fit again with the gyro's orientation
            stats.imu_refused += 1;
            auto j = cfg.imu_refit ? imu_refit(rig, obs, used, slot, r->first.joint.body_from_model.t, frame_ns)
                                   : std::nullopt;
            if (j) {
                r->first.joint = std::move(*j);
                r->second = false;
                refit_now = true;
            } else {
                r.reset();
            }
        }
        if (fails_.frame(slot, r.has_value(), cfg.fail_limit)) {
            // lost for good: back to init (cold start; the rotation stays for the gyro to carry)
            prev_[slot].reset();
            last_seen_[slot].reset();
            stats.init_resets += 1;
        }
        if (r) {
            claim(used, r->first.joint);
            tracked.push_back(T{slot, std::move(r->first), r->second, refit_now});
        }
    }
    // PiBridge: a slot without a prior relocks from the fuser's hint (the IMU carried it while the LEDs were away)
    if (cfg.use_hint) {
        for (std::size_t slot = 0; slot < 2; slot++) {
            if (!wanted[slot] || prev_[slot] || !hint[slot] || hint[slot]->fix_age_s > cfg.hint_max_age_s) {
                continue;
            }
            bool has = false;
            for (const T& t : tracked) {
                has = has || t.slot == slot;
            }
            if (has) {
                continue;
            }
            const std::size_t model = slot;
            // an old prior: the wide gate is tried too
            const SlotState st{model, hint[slot]->body_from_model, frame_ns > 100'000'000 ? frame_ns - 100'000'000 : 0,
                               std::nullopt, 0};
            auto r = track_slot(rig, obs, used, st, frame_ns, &*hint[slot]);
            if (r) {
                stats.hint_relocks += 1;
                claim(used, r->first.joint);
                r->first.model = model;
                tracked.push_back(T{slot, std::move(r->first), r->second});
            }
        }
    }
    const Body& body = body_;
    const auto last_seen = last_seen_;
    if (cfg.body) {
        std::erase_if(tracked, [&](const T& t) {
            const g::Vec3 p = t.f.joint.body_from_model.t;
            bool jump_ok = true;
            if (last_seen[t.slot]) {
                const auto& [lp, ns] = *last_seen[t.slot];
                const double age = static_cast<double>(sat_sub(frame_ns, ns)) * 1e-9;
                jump_ok = age >= body.memory_s || g::norm(g::sub(p, lp)) <= 0.12 + body.max_speed * age;
            }
            return !(jump_ok && body.implausible(t.slot, p) == nullptr);
        });
        if (tracked.size() == 2) {
            const std::array<V3, 2> now{tracked[0].f.joint.body_from_model.t, tracked[1].f.joint.body_from_model.t};
            std::array<std::optional<V3>, 2> last;
            for (std::size_t i = 0; i < 2; i++) {
                if (last_seen[tracked[i].slot]) {
                    last[i] = last_seen[tracked[i].slot]->first;
                }
            }
            if (const auto k = body.collapsed(now, last)) {
                tracked.erase(tracked.begin() + static_cast<std::ptrdiff_t>(*k));
                stats.collapsed += 1;
            }
        }
    }
    if (!tracked.empty()) {
        last_fix_ns_ = frame_ns;
    }
    // something missing: brute-force search (about 10 ms over 4 threads); with nothing in view for 2 s, only every
    // third LED frame (CPU)
    const bool idle = tracked.empty() && sat_sub(frame_ns, last_fix_ns_) > 2'000'000'000;
    std::vector<std::size_t> searched;
    bool missing = false;
    for (std::size_t s = 0; s < 2; s++) {
        bool has = false;
        for (const T& t : tracked) {
            has = has || t.slot == s;
        }
        missing = missing || (wanted[s] && !has);
    }
    if (missing && (!idle || frame_no_ % 3 == 0)) {
        std::vector<Found> pre;
        for (const T& t : tracked) {
            pre.push_back(t.f);
        }
        const std::vector<Found> got =
            solve_frame(lf, rig.cams, rig.models, rig.triples, rig.prm, nullptr, pre, true);
        stats.searches += 1;
        std::vector<Cand> cands;
        for (const Found& gf : got) {
            cands.push_back(Cand{gf.joint.body_from_model.t, gf.model});
        }
        std::array<Slot, 2> slots{};
        for (std::size_t s = 0; s < 2; s++) {
            if (last_seen_[s]) {
                slots[s].last = std::pair{last_seen_[s]->first,
                                          static_cast<double>(sat_sub(frame_ns, last_seen_[s]->second)) * 1e-9};
            }
            for (const T& t : tracked) {
                if (t.slot == s) {
                    slots[s].tracked_now = t.f.joint.body_from_model.t;
                    break;
                }
            }
        }
        const auto assigned = cfg.body ? body_.assign(cands, slots) : legacy_assign(cands, slots);
        stats.unassigned += static_cast<std::uint64_t>(got.size() - assigned.size());
        for (const auto& [ci, slot] : assigned) {
            if (!wanted[slot]) {
                continue;
            }
            Found gf = got[ci];
            if (cfg.slot_model && gf.model != slot && slot < rig.models.size()) {
                // the slot's own model from the candidate's pose: its alt fit, or a two-pass match from the pose
                std::optional<JointSolve> best;
                const auto ok = [&](const JointSolve& j) { return j.total() >= 5 && j.rms_px <= rig.prm.tol_px; };
                if (gf.alt && ok(*gf.alt)) {
                    best = *gf.alt;
                }
                if (const auto j1 = joint_from_prior(gf.joint.body_from_model, rig.models[slot], rig.cams, obs, used,
                                                     rig.prm.tol_px * 2.0, rig.prm.facing_deg)) {
                    if (auto j2 = joint_from_prior(j1->body_from_model, rig.models[slot], rig.cams, obs, used,
                                                   rig.prm.tol_px, rig.prm.facing_deg);
                        j2 && ok(*j2) &&
                        (!best || j2->total() > best->total() ||
                         (j2->total() == best->total() && j2->rms_px < best->rms_px))) {
                        best = std::move(*j2);
                    }
                }
                if (!best) {
                    // a fresh single-camera acquisition with the slot's model on the candidate's own blobs (and
                    // their neighbours), then the two-pass joint fit
                    for (std::size_t c = 0; c < gf.joint.matches.size() && c < obs.size(); c++) {
                        if (gf.joint.matches[c].size() < 3) {
                            continue;
                        }
                        std::vector<bool> u(obs[c].px.size(), true);
                        for (std::size_t b = 0; b < u.size(); b++) {
                            for (const auto& [l, mb] : gf.joint.matches[c]) {
                                const double du = obs[c].px[b].first - obs[c].px[mb].first;
                                const double dv = obs[c].px[b].second - obs[c].px[mb].second;
                                if (du * du + dv * dv < 60.0 * 60.0) {
                                    u[b] = used[c][b];
                                }
                            }
                        }
                        const auto cs = acquire(rig.models[slot], rig.triples[slot], rig.cams[c], c, obs[c], u, rig.prm);
                        if (!cs) {
                            continue;
                        }
                        const g::Pose start = rig.cams[c].cam_from_body.inverse().compose(cs->cam_from_model);
                        if (const auto j1 = joint_from_prior(start, rig.models[slot], rig.cams, obs, used,
                                                             rig.prm.tol_px * 2.0, rig.prm.facing_deg)) {
                            if (auto j2 = joint_from_prior(j1->body_from_model, rig.models[slot], rig.cams, obs, used,
                                                           rig.prm.tol_px, rig.prm.facing_deg);
                                j2 && ok(*j2) && (!best || j2->total() > best->total())) {
                                best = std::move(*j2);
                            }
                        }
                    }
                }
                if (!best) {
                    stats.model_dropped += 1;
                    continue;
                }
                stats.model_refit += 1;
                gf = Found{slot, std::move(*best), std::nullopt, gf.cam_spread_m, gf.single};
            }
            if (cfg.use_hint && hint[slot] && hint[1 - slot] && hint[slot]->fix_age_s <= cfg.hint_max_age_s) {
                // left / right: a search fix that turns like the other hand's IMU and not like this one's is the
                // other hand
                const auto ang = [&](const Hint& h) {
                    return rf::to_degrees(g::norm(
                        rotvec(g::mat_mul(gf.joint.body_from_model.r, g::transpose(h.body_from_model.r)))));
                };
                const double mine = ang(*hint[slot]);
                const double theirs = ang(*hint[1 - slot]);
                if (mine > cfg.hint_lr_deg && theirs < 0.5 * cfg.hint_lr_deg) {
                    stats.hint_refused += 1;
                    continue;
                }
            }
            bool search_refit = false;
            if (const char* why = imu_refuse(slot, gf.joint.body_from_model, frame_ns)) {
                stats.imu_refused += 1;
                std::optional<JointSolve> j;
                if (cfg.imu_refit && (std::string_view(why) == "turn" || std::string_view(why) == "tilt")) {
                    j = imu_refit(rig, obs, used, slot, gf.joint.body_from_model.t, frame_ns);
                }
                if (!j) {
                    continue;
                }
                gf.joint = std::move(*j);
                search_refit = true;
            }
            if (!prev_[slot] && !cold_start_ok(cfg, g::norm(gf.joint.body_from_model.t), gf.joint.total())) {
                stats.far_refused += 1;
                continue;
            }
            searched.push_back(slot);
            tracked.push_back(T{slot, gf, false, search_refit});
        }
    }
    std::vector<Fix> out;
    for (T& t : tracked) {
        const std::size_t slot = t.slot;
        const g::Pose p = t.f.joint.body_from_model;
        const std::optional<SlotState> old = prev_[slot];
        std::optional<std::pair<g::Vec3, g::Vec3>> vel;
        const bool was_searched = std::find(searched.begin(), searched.end(), slot) != searched.end();
        if (old && !was_searched && old->model == t.f.model) {
            const double dt = static_cast<double>(sat_sub(frame_ns, old->ns)) * 1e-9;
            if (dt > 0.005 && dt < 0.13) {
                const g::Vec3 v = g::scale(g::sub(p.t, old->pose.t), 1.0 / dt);
                g::Vec3 w = g::scale(rotvec(g::mat_mul(g::transpose(old->pose.r), p.r)), 1.0 / dt);
                const auto [v0, w0] = old->vel ? *old->vel : std::pair{v, w};
                // few-LED rotations follow the prior: keep (decay) the old rate rather than measure one
                w = t.few ? g::scale(w0, 0.5) : g::scale(g::add(w, w0), 0.5);
                vel = std::pair{cap(g::scale(g::add(v, v0), 0.5), MAX_V), cap(w, MAX_W)};
            }
        }
        const std::uint32_t few_streak = t.few ? (old ? old->few_streak : 0) + 1 : 0;
        prev_[slot] = SlotState{t.f.model, p, frame_ns, vel, few_streak};
        last_seen_[slot] = std::pair{p.t, frame_ns};
        last_rot_[slot] = std::pair{p.r, frame_ns};
        last_pos_[slot] = std::pair{p.t, frame_ns};
        if (!t.refit) {
            anchor_[slot] = std::pair{p.r, frame_ns};
        }
        fails_.clear(slot);
        stats.hand[slot] += 1;
        if (t.few) {
            stats.few[slot] += 1;
        }
        out.push_back(Fix{slot, std::move(t.f), t.few, std::nullopt});
    }
    return out;
}

void Tracker::imu_push(std::size_t slot, const ImuSample& s) {
    auto& d = imu_[slot];
    if (!d.empty() && s.t <= d.back().t) {
        return;
    }
    d.push_back(s);
    while (!d.empty() && s.t - d.front().t > 3'000'000'000ull) {
        d.pop_front();
    }
}

const ImuSample* Tracker::imu_near(std::size_t slot, std::uint64_t t) const {
    const auto& d = imu_[slot];
    if (d.empty()) {
        return nullptr;
    }
    const auto it = std::lower_bound(d.begin(), d.end(), t, [](const ImuSample& e, std::uint64_t x) { return e.t < x; });
    const ImuSample* best = nullptr;
    std::uint64_t bd = 25'000'000;
    for (auto j = it == d.begin() ? it : it - 1; j != d.end() && j <= it; ++j) {
        const std::uint64_t dd = j->t > t ? j->t - t : t - j->t;
        if (dd <= bd) {
            bd = dd;
            best = &*j;
        }
    }
    return best;
}

std::optional<std::pair<char, double>> Tracker::imu_error(std::size_t slot, const g::Mat3& r, std::uint64_t t) const {
    const ImuSample* now = imu_near(slot, t);
    if (now == nullptr) {
        return std::nullopt;
    }
    if (anchor_[slot]) {
        const auto& [rl, tl] = *anchor_[slot];
        const double age = static_cast<double>(sat_sub(t, tl)) * 1e-9;
        const ImuSample* then = imu_near(slot, tl);
        if (then != nullptr && age > 0.0 && age <= 1.5) {
            // the turn since the last fix, from the gyro (aim frame) and in the model frame from the optics
            const g::Mat3 dg = g::mat_mul(g::transpose(then->qg), now->qg);
            const g::Mat3 pred = g::mat_mul(g::mat_mul(device_from_aim, dg), g::transpose(device_from_aim));
            const g::Mat3 opt = g::mat_mul(g::transpose(rl), r);
            return std::pair{'r', rf::to_degrees(g::norm(rotvec(g::mat_mul(g::transpose(pred), opt))))};
        }
    }
    if (now->up_ok) {
        const g::Vec3 up = g::mat_vec(g::mat_mul(r, device_from_aim), now->up_aim);
        return std::pair{'t', g::angle_deg(up, now->up_body)};
    }
    return std::nullopt;
}

const char* Tracker::imu_refuse(std::size_t slot, const g::Pose& p, std::uint64_t t) const {
    if (!cfg.imu_id || imu_[slot].empty()) {
        return nullptr;
    }
    if (sat_sub(t, imu_[slot].back().t) > 500'000'000) {
        // the controller's IMU stopped reporting while the other's goes on: it is off or out of radio range; when
        // both are quiet, the writer (Monado) is gone and the IMU says nothing
        const auto& o = imu_[1 - slot];
        return !o.empty() && sat_sub(t, o.back().t) <= 500'000'000 ? "silent" : nullptr;
    }
    const ImuSample* now = imu_near(slot, t);
    if (now != nullptr && now->still_s > 0.3 && last_pos_[slot] && last_rot_[slot] &&
        sat_sub(t, last_pos_[slot]->second) < 2'000'000'000) {
        // quiet gyro: the controller has not moved since its last fix
        const double moved = g::norm(g::sub(p.t, last_pos_[slot]->first));
        const double turned = rf::to_degrees(g::norm(rotvec(g::mat_mul(g::transpose(last_rot_[slot]->first), p.r))));
        if (moved > 0.03 || turned > 6.0) {
            return "rest";
        }
    }
    if (now != nullptr && now->still_s > cfg.imu_rest_s &&
        (!last_pos_[slot] || sat_sub(t, last_pos_[slot]->second) > 500'000'000)) {
        return "parked"; // at rest for long and not tracked: a fix here follows the other hand
    }
    if (const auto e = imu_error(slot, p.r, t)) {
        if (std::getenv("CT_IMU_DEBUG") != nullptr) {
            std::fprintf(stderr, "imudbg t %.3f slot %zu %c %.1f still %.2f age %.3f\n", static_cast<double>(t) * 1e-9, slot,
                         e->first, e->second, now ? now->still_s : -1.0,
                         last_rot_[slot] ? static_cast<double>(sat_sub(t, last_rot_[slot]->second)) * 1e-9 : -1.0);
        }
        if (e->first == 'r' ? e->second > cfg.imu_rot_deg : e->second > cfg.imu_tilt_deg) {
            return e->first == 'r' ? "turn" : "tilt";
        }
    }
    return nullptr;
}

std::optional<g::Mat3> Tracker::imu_predict(std::size_t slot, std::uint64_t t) const {
    if (!anchor_[slot]) {
        return std::nullopt;
    }
    const auto& [rl, tl] = *anchor_[slot];
    const ImuSample* now = imu_near(slot, t);
    const ImuSample* then = imu_near(slot, tl);
    if (now == nullptr || then == nullptr || sat_sub(t, tl) > 1'500'000'000) {
        return std::nullopt;
    }
    const g::Mat3 dg = g::mat_mul(g::transpose(then->qg), now->qg);
    return g::mat_mul(rl, g::mat_mul(g::mat_mul(device_from_aim, dg), g::transpose(device_from_aim)));
}

std::optional<JointSolve> Tracker::imu_refit(const Rig& rig, const std::vector<CamObs>& obs,
                                             const std::vector<std::vector<bool>>& used, std::size_t slot,
                                             const g::Vec3& at, std::uint64_t t) const {
    const auto rp = imu_predict(slot, t);
    if (!rp) {
        return std::nullopt;
    }
    const LedModel& model = rig.models[slot];
    const double cos_face = std::cos(rf::to_radians(rig.prm.facing_deg));
    const std::optional<std::pair<g::Mat3, double>> prior = std::pair{*rp, cfg.hint_rot_w};
    g::Pose p{*rp, at};
    for (const double tol : {rig.prm.prior_tol_px, rig.prm.tol_px * 2.0}) {
        const std::vector<Matches> m = match_all(p, rig, model, obs, used, cos_face, tol);
        if (total_of(m) < 3) {
            return std::nullopt;
        }
        p = refine_with(p, obs_of(rig, model, obs, m), true, 20, prior).first;
    }
    std::vector<Matches> m = match_all(p, rig, model, obs, used, cos_face, rig.prm.tol_px);
    if (total_of(m) < 4) {
        return std::nullopt;
    }
    const auto [p2, rms] = refine_with(p, obs_of(rig, model, obs, m), true, 20, prior);
    if (rms > rig.prm.tol_px || imu_refuse(slot, p2, t) != nullptr) {
        return std::nullopt;
    }
    return JointSolve{p2, std::move(m), rms};
}

std::vector<Matches> match_all(const g::Pose& pose, const Rig& rig, const LedModel& model,
                               const std::vector<CamObs>& obs, const std::vector<std::vector<bool>>& used,
                               double cos_face, double tol_px) {
    std::vector<Matches> out;
    for (std::size_t ci = 0; ci < rig.cams.size(); ci++) {
        const Cam& c = rig.cams[ci];
        const g::Pose cfm = c.cam_from_body.compose(pose);
        out.push_back(match_pixels(cfm, model, c, obs[ci], used[ci], cos_face, tol_px));
    }
    return out;
}

std::pair<std::size_t, std::size_t> visibility(const Rig& rig, std::size_t model_i, const g::Pose& pose,
                                               const std::vector<std::vector<std::pair<double, double>>>& px,
                                               double near_px) {
    const LedModel& model = rig.models[model_i];
    const double cos_face = std::cos(rf::to_radians(rig.prm.facing_deg));
    std::size_t inview = 0;
    std::size_t near = 0;
    for (std::size_t ci = 0; ci < rig.cams.size(); ci++) {
        const Cam& c = rig.cams[ci];
        const g::Pose cfm = c.cam_from_body.compose(pose);
        std::vector<std::pair<double, double>> proj;
        for (const Led& led : model.leds) {
            const g::Vec3 pc = cfm.apply(led.p);
            if (pc[2] <= 0.0) {
                continue;
            }
            const g::Vec3 n = g::mat_vec(cfm.r, led.n);
            if (g::dot(n, g::scale(g::normalize(pc), -1.0)) < cos_face) {
                continue;
            }
            if (const auto uv = c.intr.project(pc)) {
                const auto [u, v] = *uv;
                if (u >= 0.0 && v >= 0.0 && u < static_cast<double>(cam::TILE_W) &&
                    v < static_cast<double>(cam::TILE_H)) {
                    proj.emplace_back(u, v);
                }
            }
        }
        inview += proj.size();
        for (const auto& b : px[ci]) {
            for (const auto& q : proj) {
                const double du = q.first - b.first;
                const double dv = q.second - b.second;
                if (du * du + dv * dv <= near_px * near_px) {
                    near++;
                    break;
                }
            }
        }
    }
    return {inview, near};
}

} // namespace pibridge::ctrl::track
