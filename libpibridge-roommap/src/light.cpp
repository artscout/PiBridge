// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/light.rs (MIT OR Apache-2.0). See light.h.
#include "pibridge/roommap/light.h"

#include "pibridge/anchor/rustx.h"
#include "pibridge/cam/ae.h"
#include "pibridge/cam/frame.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/cam/source.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace pibridge::roommap::light {

namespace g = cam::geom;
namespace rf = cam::rfmt;
namespace rs = anchor::rs;

namespace {

std::pair<double, double> shared_metric(const pngread::Grey& a, const pngread::Grey& b) {
    const cam::ae::SceneMetric ms[2] = {cam::ae::scene_metric_grey(a.data, a.w, a.h),
                                        cam::ae::scene_metric_grey(b.data, b.w, b.h)};
    double n = -0.0;
    for (const auto& m : ms) {
        n += static_cast<double>(m.n);
    }
    n = rf::fmax(n, 1.0);
    double mean = -0.0, sat = -0.0;
    for (const auto& m : ms) {
        mean += m.mean * static_cast<double>(m.n);
    }
    for (const auto& m : ms) {
        sat += m.sat * static_cast<double>(m.n);
    }
    return {mean / n, sat / n};
}

template <class T>
T median_of(std::vector<T> v) {
    if (v.empty()) {
        return T{};
    }
    // sort_by(partial_cmp, Equal for NaN): stable
    std::stable_sort(v.begin(), v.end(), [](const T& x, const T& y) { return x < y; });
    return v[v.size() / 2];
}

} // namespace

FrameQuality measure(const stereo::Rig& rig, const pngread::Grey& a, const pngread::Grey& b, const orb::OrbParams& orb) {
    return measure_with(rig, a, b, orb, stereo::LOW_LIGHT);
}

FrameQuality measure_with(const stereo::Rig& rig, const pngread::Grey& a, const pngread::Grey& b,
                          const orb::OrbParams& orb, const std::optional<stereo::LowLight>& low) {
    const pngread::Grey* imgs[2] = {&a, &b};
    std::vector<stereo::Features> feats;
    for (std::size_t i = 0; i < 2 && i < rig.cams.size(); i++) {
        feats.push_back(
            stereo::features_with(orb::Image{imgs[i]->w, imgs[i]->h, imgs[i]->w, imgs[i]->data}, rig.cams[i], orb, low));
    }
    FrameQuality q;
    q.stereo = stereo::triangulate(feats[0], feats[1], rig.cams[0], rig.cams[1], stereo::StereoParams{}).size();
    std::tie(q.mean, q.sat) = shared_metric(a, b);
    q.corners = {feats[0].kps.size(), feats[1].kps.size()};
    return q;
}

const char* advice_debug(Advice a) {
    switch (a) {
    case Advice::Good: return "Good";
    case Advice::TooDark: return "TooDark";
    case Advice::TooPlain: return "TooPlain";
    }
    return "";
}

double dark_mean() {
    return cam::ae::GOOD_MEAN_MIN;
}

Advice advice(const FrameQuality& q, std::size_t need) {
    if (q.stereo >= need) {
        return Advice::Good;
    }
    if (q.mean < dark_mean()) {
        return Advice::TooDark;
    }
    return Advice::TooPlain;
}

const char* advice_text(Advice a) {
    switch (a) {
    case Advice::Good: return "Good: keep looking around slowly.";
    case Advice::TooDark: return "Too dark for the cameras: turn on the room lights.";
    case Advice::TooPlain: return "Too little detail here: look at furniture, shelves or the desk, not a blank wall.";
    }
    return "";
}

LightReport summarise(std::span<const FrameQuality> qs, std::size_t need) {
    LightReport r;
    r.frames = qs.size();
    std::vector<double> means;
    std::vector<std::size_t> c0, c1, st;
    for (const auto& q : qs) {
        means.push_back(q.mean);
        c0.push_back(q.corners[0]);
        c1.push_back(q.corners[1]);
        st.push_back(q.stereo);
        r.good += q.stereo >= need;
    }
    r.mean = median_of(means);
    r.corners = {median_of(c0), median_of(c1)};
    r.stereo = median_of(st);
    if (!qs.empty()) {
        r.exposure = qs.back().exposure;
        r.gain = qs.back().gain;
    }
    return r;
}

void darken(pngread::Grey& gr, float scale, float noise, std::uint64_t seed) {
    std::uint64_t x = seed | 1;
    for (auto& v : gr.data) {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        const float u = static_cast<float>(x >> 40) / static_cast<float>(1ull << 24) * 2.0f - 1.0f;
        float r = std::round(static_cast<float>(v) * scale + u * noise);
        // f32::clamp (NaN stays NaN), then `as u8`
        if (r < 0.0f) {
            r = 0.0f;
        }
        if (r > 255.0f) {
            r = 255.0f;
        }
        v = rs::to_u8f(r);
    }
}

std::expected<std::vector<FrameQuality>, std::string> measure_source(const std::string& spec, double seconds,
                                                                     const MeasureOpts& o, const stereo::Rig& rig,
                                                                     const orb::OrbParams& orb) {
    cam::ReplayOptions ro;
    auto src = cam::open_source(spec, 0, ro);
    if (!src) {
        return std::unexpected("cannot open source " + rf::debug_str(spec) + ": " + src.error().text());
    }
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<FrameQuality> out;
    std::size_t k = 0;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < seconds) {
        auto fr = (*src)->next(500);
        if (!fr) {
            return std::unexpected("source: " + fr.error().text());
        }
        if (!*fr) {
            if ((*src)->finished()) {
                break;
            }
            continue;
        }
        const auto& f = **fr;
        const auto m = cam::Meta::parse(f.data.subspan(0, cam::META_LEN));
        if (!m || m->kind() != cam::FrameKind::Tracking) {
            continue;
        }
        k++;
        if (k % std::max<std::size_t>(o.every, 1) != 0) {
            continue;
        }
        const auto rows = cam::tile_rows(&*m, f.data, f.stride);
        pngread::Grey imgs[2];
        for (std::size_t ci = 0; ci < 2; ci++) {
            imgs[ci].w = cam::TILE_W;
            imgs[ci].h = cam::TILE_H;
            imgs[ci].data.assign(cam::TILE_W * cam::TILE_H, 0);
            cam::copy_tile(f.data, f.stride, ci, rows.y[ci], imgs[ci].data, cam::TILE_W);
        }
        if (o.darken) {
            for (std::size_t i = 0; i < 2; i++) {
                darken(imgs[i], o.darken->first, o.darken->second, (static_cast<std::uint64_t>(k) << 1) | i);
            }
        }
        FrameQuality q = measure_with(rig, imgs[0], imgs[1], orb, o.low);
        q.exposure = m->exposure[0];
        q.gain = m->gain[0];
        out.push_back(q);
    }
    return out;
}

FrameQuality quality_of(const stereo::Rig& rig, const std::vector<stereo::Features>& feats, const pngread::Grey& a,
                        const pngread::Grey& b) {
    FrameQuality q;
    q.stereo = stereo::triangulate(feats[0], feats[1], rig.cams[0], rig.cams[1], stereo::StereoParams{}).size();
    std::tie(q.mean, q.sat) = shared_metric(a, b);
    q.corners = {feats[0].kps.size(), feats[1].kps.size()};
    return q;
}

std::string progress_text(const MapProgress& p) {
    const char* what = (p.advice == Advice::Good && p.move_more)
                           ? "Turn your head further: left, right, behind you and down at the floor."
                           : advice_text(p.advice);
    return std::to_string(p.keyframes) + " views mapped, " + std::to_string(p.quality.stereo) + " points in view. " + what;
}

bool moved(std::span<const cam::geom::Pose> poses, double min_deg, double min_m) {
    if (poses.empty()) {
        return false;
    }
    const auto& first = poses.front();
    const g::Vec3 f0 = g::mat_vec(first.r, {0.0, 0.0, -1.0});
    for (const auto& p : poses) {
        if (g::angle_deg(f0, g::mat_vec(p.r, {0.0, 0.0, -1.0})) > min_deg || g::norm(g::sub(p.t, first.t)) > min_m) {
            return true;
        }
    }
    return false;
}

} // namespace pibridge::roommap::light
