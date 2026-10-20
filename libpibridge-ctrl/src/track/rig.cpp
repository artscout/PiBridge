// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/track.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/track/rig.h"

#include "pibridge/cam/frame.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/ctrl/track/extrinsics.h"

#include <algorithm>
#include <cstdio>
#include <format>
#include <thread>

namespace pibridge::ctrl::track {

namespace g = cam::geom;
namespace rf = cam::rfmt;

std::optional<std::string> arg(std::span<const std::string> a, std::string_view k) {
    for (std::size_t i = 0; i < a.size(); i++) {
        if (a[i] == k) {
            if (i + 1 < a.size()) {
                return a[i + 1];
            }
            return std::nullopt;
        }
    }
    return std::nullopt;
}

std::expected<Rig, std::string> Rig::try_load(std::span<const std::string> a) {
    auto loaded = cam::DeviceCalib::load_default(arg(a, "--calib"));
    if (!loaded) {
        return std::unexpected(std::format("headset calibration: {}", loaded.error().text()));
    }
    Rig rig;
    rig.calib = std::move(loaded->first);
    const std::string cpath = rf::utf8_lossy(loaded->second);
    if (rig.calib.cameras.size() < cam::TILE_COUNT) {
        return std::unexpected(std::format("headset calibration {}: {} cameras, the tracker needs {}", cpath,
                                           rig.calib.cameras.size(), cam::TILE_COUNT));
    }
    const Env env = process_env();
    auto models = LedModel::resolve(arg(a, "--left"), arg(a, "--right"), env);
    if (!models) {
        return std::unexpected(models.error());
    }
    rig.models = std::move(models->first);
    const ModelSource source = std::move(models->second);
    for (std::size_t i = 0; i < cam::TILE_COUNT; i++) {
        const auto body_from_cam = rig.calib.body_from_cam(i, cam::RigConvention::CamFromCam0);
        if (!body_from_cam) {
            return std::unexpected(
                std::format("headset calibration {}: no camera/IMU extrinsic (ombc/tbc)", cpath));
        }
        rig.cams.push_back(Cam{rig.calib.cameras[i].intr, body_from_cam->inverse()});
    }
    std::string ext_note = "factory extrinsics";
    if (std::find(a.begin(), a.end(), "--factory") == a.end()) {
        // the user's own fit, else the shipped fit for the reference headset (its factory extrinsics of cams 1-3 are
        // off by up to 3.2 deg, which tilts the controllers)
        auto [v, note] = extrinsics::resolve(rig.calib, arg(a, "--extrinsics"), env);
        for (const auto& [id, pose] : v) {
            if (id < rig.cams.size()) {
                rig.cams[id].cam_from_body = pose;
            }
        }
        ext_note = std::move(note);
    }
    for (const LedModel& m : rig.models) {
        rig.triples.push_back(m.neighbour_triples(0.045, 110.0));
    }
    std::fprintf(stderr, "%s\n",
                 std::format("calibration {}{} ({}) | models {} ({} LEDs) / {} ({} LEDs) | LED triples {} / {}", cpath,
                             rig.calib.is_reference() ? " (the shipped Dream Air reference)" : "", ext_note,
                             rig.models[0].hand, rig.models[0].leds.size(), rig.models[1].hand,
                             rig.models[1].leds.size(), rig.triples[0].size(), rig.triples[1].size())
                     .c_str());
    std::fprintf(stderr, "controller models: %s\n", source.describe().c_str());
    return rig;
}

void claim(std::vector<std::vector<bool>>& used, const JointSolve& j) {
    for (std::size_t ci = 0; ci < j.matches.size(); ci++) {
        for (const auto& [l, b] : j.matches[ci]) {
            used[ci][b] = true;
        }
    }
}

bool same_object(const Found& a, const Found& b) {
    return g::norm(g::sub(a.joint.body_from_model.t, b.joint.body_from_model.t)) < 0.06;
}

namespace {

/// Rust `{:?}` of Option<(usize, usize, f64, [f64; 3])> (the CT_DEBUG line).
std::string debug_best(const std::optional<std::pair<std::size_t, CamSolve>>& best) {
    if (!best) {
        return "None";
    }
    const auto& [m, s] = *best;
    return std::format("Some(({}, {}, {}, {}))", m, s.matches.size(), rf::debug_f64(s.rms_px),
                       rf::debug_f64s(s.cam_from_model.t));
}

} // namespace

std::vector<Found> solve_frame(const LedFrame& f, std::span<const Cam> cams, const std::array<LedModel, 2>& models,
                               std::span<const std::vector<std::array<std::size_t, 3>>> triples, const Params& prm,
                               const std::vector<Prior>* prior, std::span<const Found> pre, bool parallel) {
    std::vector<CamObs> obs;
    obs.reserve(cams.size());
    for (std::size_t ci = 0; ci < cams.size() && ci < f.px.size(); ci++) {
        obs.push_back(CamObs::make(cams[ci], f.px[ci]));
    }
    std::vector<std::vector<bool>> used;
    for (const CamObs& o : obs) {
        used.emplace_back(o.px.size(), false);
    }
    for (const Found& p : pre) {
        claim(used, p.joint);
    }
    const auto accept = [&](const JointSolve& j) { return j.total() >= 5 && j.rms_px <= prm.tol_px; };
    std::vector<Found> out;
    if (prior != nullptr) {
        for (const Prior& pr : *prior) {
            if (!pr) {
                continue;
            }
            const auto& [mi, pose, ts] = *pr;
            (void)ts;
            const auto j1 = joint_from_prior(pose, models[mi], cams, obs, used, prm.prior_tol_px, prm.facing_deg);
            if (!j1) {
                continue;
            }
            // tighten: rematch at the normal tolerance from the refined pose
            auto j = joint_from_prior(j1->body_from_model, models[mi], cams, obs, used, prm.tol_px, prm.facing_deg);
            if (!j) {
                continue;
            }
            if (accept(*j)) {
                claim(used, *j);
                out.push_back(Found{mi, std::move(*j), std::nullopt, 0.0, 0});
            }
        }
        return out;
    }
    // single-camera acquisition: per camera, the better model first, then the other on the remaining blobs
    const bool debug = cam::env_os("CT_DEBUG").has_value();
    const auto per_cam = [&](std::size_t ci) {
        const Cam& cam = cams[ci];
        std::vector<bool> u = used[ci];
        std::vector<std::pair<std::size_t, CamSolve>> got;
        for (int round = 0; round < 2; round++) {
            std::optional<std::pair<std::size_t, CamSolve>> best;
            for (std::size_t mi = 0; mi < models.size(); mi++) {
                auto s = acquire(models[mi], triples[mi], cam, ci, obs[ci], u, prm);
                if (!s) {
                    continue;
                }
                const bool better = !best || s->matches.size() > best->second.matches.size() ||
                                    (s->matches.size() == best->second.matches.size() &&
                                     s->rms_px < best->second.rms_px);
                if (better) {
                    best = std::pair{mi, std::move(*s)};
                }
            }
            if (debug) {
                std::fprintf(stderr, "%s\n",
                             std::format("  cam{} round {}: blobs {} -> {}", ci, round, obs[ci].px.size(),
                                         debug_best(best))
                                 .c_str());
            }
            if (!best) {
                break;
            }
            for (const auto& [l, b] : best->second.matches) {
                u[b] = true;
            }
            got.push_back(std::move(*best));
        }
        return got;
    };
    std::vector<std::vector<std::pair<std::size_t, CamSolve>>> per(cams.size());
    if (parallel) {
        std::vector<std::thread> hs;
        hs.reserve(cams.size());
        for (std::size_t ci = 0; ci < cams.size(); ci++) {
            hs.emplace_back([&, ci] { per[ci] = per_cam(ci); });
        }
        for (std::thread& h : hs) {
            h.join();
        }
    } else {
        for (std::size_t ci = 0; ci < cams.size(); ci++) {
            per[ci] = per_cam(ci);
        }
    }
    std::vector<std::pair<std::size_t, CamSolve>> cands;
    for (auto& v : per) {
        for (auto& c : v) {
            cands.push_back(std::move(c));
        }
    }
    std::stable_sort(cands.begin(), cands.end(), [](const auto& a, const auto& b) {
        if (a.second.matches.size() != b.second.matches.size()) {
            return a.second.matches.size() > b.second.matches.size();
        }
        return a.second.rms_px < b.second.rms_px;
    });
    const auto body = [&](const CamSolve& c) { return cams[c.cam].cam_from_body.inverse().compose(c.cam_from_model); };
    const auto two_pass = [&](const g::Pose& start, std::size_t mi) -> std::optional<JointSolve> {
        const auto j = joint_from_prior(start, models[mi], cams, obs, used, prm.tol_px * 2.0, prm.facing_deg);
        if (!j) {
            return std::nullopt;
        }
        return joint_from_prior(j->body_from_model, models[mi], cams, obs, used, prm.tol_px, prm.facing_deg);
    };
    for (const auto& [mi, c] : cands) {
        if (out.size() + pre.size() >= 2) {
            break;
        }
        const g::Pose start = body(c);
        bool near = false;
        for (const Found& o : out) {
            near = near || g::norm(g::sub(o.joint.body_from_model.t, start.t)) < 0.06;
        }
        if (near) {
            continue;
        }
        auto j = two_pass(start, mi);
        if (!j || !accept(*j)) {
            continue;
        }
        const std::size_t other = 1 - mi;
        auto alt = two_pass(start, other);
        // the same controller's single-camera solves from other cameras
        double spread = 0.0;
        std::size_t single = 0;
        for (const auto& [mj, c2] : cands) {
            const g::Pose b2 = body(c2);
            if (mj == mi && g::norm(g::sub(b2.t, j->body_from_model.t)) < 0.06) {
                single += 1;
                spread = rf::fmax(spread, g::norm(g::sub(b2.t, j->body_from_model.t)));
            }
        }
        claim(used, *j);
        out.push_back(Found{mi, std::move(*j), std::move(alt), spread, single});
    }
    return out;
}

} // namespace pibridge::ctrl::track
