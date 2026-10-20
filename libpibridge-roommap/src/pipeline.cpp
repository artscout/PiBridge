// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/pipeline.rs (MIT OR Apache-2.0). See pipeline.h.
#include "pibridge/roommap/pipeline.h"

#include <algorithm>
#include <optional>
#include <thread>

namespace pibridge::roommap::pipeline {

namespace g = cam::geom;

namespace {

struct Job {
    std::int64_t ts;
    Pose pose;
    const rec::FrameRef* f;
};

std::vector<Job> jobs_of(const rec::Recording& rec, std::size_t every, const std::optional<Pose>& pre) {
    std::vector<Job> jobs;
    for (std::size_t k = 0; k < rec.frames.size(); k++) {
        if (k % std::max<std::size_t>(every, 1) != 0) {
            continue;
        }
        const auto& f = rec.frames[k];
        if (auto p = rec::pose_at(rec.traj, f.ts_ns)) {
            jobs.push_back(Job{f.ts_ns, pre ? pre->compose(*p) : *p, &f});
        }
    }
    return jobs;
}

/// Run `work` over chunks of `jobs` on `threads` workers; results in chunk order; the last failing chunk's error.
template <class T, class F>
std::expected<std::vector<T>, std::string> parallel(const std::vector<Job>& jobs, std::size_t threads, F work) {
    const std::size_t n = jobs.size();
    const std::size_t nt = std::clamp<std::size_t>(threads, 1, 64);
    const std::size_t chunk = std::max<std::size_t>((n + nt - 1) / nt, 1);
    const std::size_t parts = (n + chunk - 1) / chunk;
    std::vector<std::expected<std::vector<T>, std::string>> res(parts);
    {
        std::vector<std::jthread> hs;
        for (std::size_t c = 0; c < parts; c++) {
            hs.emplace_back([&, c] {
                std::vector<T> v;
                for (std::size_t i = c * chunk; i < std::min(n, (c + 1) * chunk); i++) {
                    auto r = work(jobs[i], v);
                    if (!r) {
                        res[c] = std::unexpected(r.error());
                        return;
                    }
                }
                res[c] = std::move(v);
            });
        }
    }
    std::vector<T> out;
    std::optional<std::string> err;
    for (auto& r : res) {
        if (r) {
            for (auto& x : *r) {
                out.push_back(std::move(x));
            }
        } else {
            err = r.error();
        }
    }
    if (err) {
        return std::unexpected(*err);
    }
    return out;
}

} // namespace

session::SessionParams build_params(double kf_dist, double kf_angle_deg) {
    session::SessionParams p;
    p.kf_min_dist = kf_dist;
    p.kf_min_angle_deg = kf_angle_deg;
    p.kf_min_points = 40;
    return p;
}

std::vector<stereo::Features> frame_features(const stereo::Rig& rig, const pngread::Grey& a, const pngread::Grey& b,
                                             const orb::OrbParams& orb) {
    std::vector<stereo::Features> v;
    const pngread::Grey* imgs[2] = {&a, &b};
    for (std::size_t i = 0; i < 2 && i < rig.cams.size(); i++) {
        v.push_back(stereo::features(orb::Image{imgs[i]->w, imgs[i]->h, imgs[i]->w, imgs[i]->data}, rig.cams[i], orb));
    }
    return v;
}

std::expected<std::pair<map::RoomMap, BuildStats>, std::string> build_from_recording(
    const rec::Recording& rec, const stereo::Rig& rig, map::RoomMap map, const session::SessionParams& sp,
    std::size_t every, const session::Alignment& room_from_slam) {
    session::RoomSession s(std::move(map), rig, sp);
    BuildStats st;
    for (std::size_t k = 0; k < rec.frames.size(); k++) {
        if (k % std::max<std::size_t>(every, 1) != 0) {
            continue;
        }
        const auto& f = rec.frames[k];
        const auto sb = rec::pose_at(rec.traj, f.ts_ns);
        if (!sb) {
            st.no_pose++;
            continue;
        }
        auto a = pngread::read(f.cam0);
        if (!a) {
            return std::unexpected(a.error());
        }
        auto b = pngread::read(f.cam1);
        if (!b) {
            return std::unexpected(b.error());
        }
        const orb::Image v[2] = {orb::Image{a->w, a->h, a->w, a->data}, orb::Image{b->w, b->h, b->w, b->data}};
        s.on_frame(f.ts_ns, v, *sb, room_from_slam, true);
        st.frames++;
    }
    return std::pair{std::move(s).finish(), st};
}

std::expected<std::pair<std::vector<align::FrameObs>, std::size_t>, std::string> observe_recording(
    const rec::Recording& rec, const map::RoomMap& map, const stereo::Rig& rig, std::size_t every,
    const orb::OrbParams& orb, const align::ObserveParams& op, std::size_t threads) {
    const auto jobs = jobs_of(rec, every, std::nullopt);
    auto out = parallel<align::FrameObs>(jobs, threads, [&](const Job& j, std::vector<align::FrameObs>& v) -> std::expected<void, std::string> {
        auto a = pngread::read(j.f->cam0);
        if (!a) {
            return std::unexpected(a.error());
        }
        auto b = pngread::read(j.f->cam1);
        if (!b) {
            return std::unexpected(b.error());
        }
        const auto feats = frame_features(rig, *a, *b, orb);
        if (auto o = align::observe(map, rig, feats, j.ts, j.pose, op)) {
            v.push_back(std::move(*o));
        }
        return {};
    });
    if (!out) {
        return std::unexpected(out.error());
    }
    std::stable_sort(out->begin(), out->end(),
                     [](const align::FrameObs& x, const align::FrameObs& y) { return x.ts_ns < y.ts_ns; });
    return std::pair{std::move(*out), jobs.size()};
}

std::expected<std::vector<refine::MapFrame>, std::string> map_frames(const rec::Recording& rec, const stereo::Rig& rig,
                                                                     std::size_t every, const orb::OrbParams& orb,
                                                                     const session::Alignment& room_from_slam,
                                                                     std::size_t threads) {
    const auto jobs = jobs_of(rec, every, room_from_slam.pose());
    auto out = parallel<refine::MapFrame>(jobs, threads, [&](const Job& j, std::vector<refine::MapFrame>& v) -> std::expected<void, std::string> {
        auto a = pngread::read(j.f->cam0);
        if (!a) {
            return std::unexpected(a.error());
        }
        auto b = pngread::read(j.f->cam1);
        if (!b) {
            return std::unexpected(b.error());
        }
        v.push_back(refine::MapFrame::make(j.ts, j.pose, frame_features(rig, *a, *b, orb), rig));
        return {};
    });
    if (!out) {
        return std::unexpected(out.error());
    }
    std::stable_sort(out->begin(), out->end(),
                     [](const refine::MapFrame& x, const refine::MapFrame& y) { return x.ts_ns < y.ts_ns; });
    return std::move(*out);
}

std::vector<Vec3> frame_points(const stereo::Rig& rig, const std::vector<stereo::Features>& feats,
                               const Pose& slam_from_body, double max_range) {
    const stereo::StereoParams sp;
    std::vector<Vec3> out;
    for (const auto& p : stereo::triangulate(feats[0], feats[1], rig.cams[0], rig.cams[1], sp)) {
        if (g::norm(p.p_body) <= max_range) {
            out.push_back(slam_from_body.apply(p.p_body));
        }
    }
    return out;
}

std::expected<std::vector<PointFrame>, std::string> recording_points(const rec::Recording& rec, const stereo::Rig& rig,
                                                                     std::size_t every, double max_range,
                                                                     std::size_t threads) {
    const orb::OrbParams orb;
    const auto jobs = jobs_of(rec, every, std::nullopt);
    auto out = parallel<PointFrame>(jobs, threads, [&](const Job& j, std::vector<PointFrame>& v) -> std::expected<void, std::string> {
        auto a = pngread::read(j.f->cam0);
        if (!a) {
            return std::unexpected(a.error());
        }
        auto b = pngread::read(j.f->cam1);
        if (!b) {
            return std::unexpected(b.error());
        }
        const auto feats = frame_features(rig, *a, *b, orb);
        v.push_back(PointFrame{j.ts, j.pose, frame_points(rig, feats, j.pose, max_range)});
        return {};
    });
    if (!out) {
        return std::unexpected(out.error());
    }
    std::stable_sort(out->begin(), out->end(), [](const PointFrame& x, const PointFrame& y) { return x.ts_ns < y.ts_ns; });
    return std::move(*out);
}

} // namespace pibridge::roommap::pipeline
