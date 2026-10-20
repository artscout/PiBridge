// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-room/src/stereo.rs (MIT OR Apache-2.0). See stereo.h.
#include "pibridge/roommap/stereo.h"

#include "pibridge/anchor/rustx.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/cam/slam.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace pibridge::roommap::stereo {

namespace g = cam::geom;
namespace rf = cam::rfmt;
namespace rs = anchor::rs;

Pinhole Pinhole::make(const cam::Kb4& k, const Mat3& cam_from_virt) {
    Pinhole ph;
    ph.w = static_cast<std::size_t>(k.width);
    ph.h = static_cast<std::size_t>(k.height);
    ph.f = k.fx;
    ph.cx = (static_cast<double>(ph.w) - 1.0) / 2.0;
    ph.cy = (static_cast<double>(ph.h) - 1.0) / 2.0;
    ph.cam_from_virt = cam_from_virt;
    ph.lut.reserve(ph.w * ph.h);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (std::size_t y = 0; y < ph.h; y++) {
        for (std::size_t x = 0; x < ph.w; x++) {
            const Vec3 ray = g::mat_vec(cam_from_virt, {(static_cast<double>(x) - ph.cx) / ph.f,
                                                        (static_cast<double>(y) - ph.cy) / ph.f, 1.0});
            const auto uv = k.project(ray);
            if (uv && uv->first >= 0.0 && uv->second >= 0.0 && uv->first < static_cast<double>(ph.w) - 1.0 &&
                uv->second < static_cast<double>(ph.h) - 1.0) {
                ph.lut.emplace_back(static_cast<float>(uv->first), static_cast<float>(uv->second));
            } else {
                ph.lut.emplace_back(nan, nan);
            }
        }
    }
    return ph;
}

Mat3 Pinhole::body_from_virt(double pitch_rad) {
    const Vec3 z{0.0, std::sin(pitch_rad), -std::cos(pitch_rad)};
    const Vec3 x{1.0, 0.0, 0.0};
    const Vec3 y = g::cross(z, x);
    return g::from_cols(x, y, z);
}

std::vector<std::uint8_t> Pinhole::undistort(const orb::Image& img) const {
    std::vector<std::uint8_t> out(w * h, 0);
    for (std::size_t i = 0; i < out.size(); i++) {
        const auto [u, v] = lut[i];
        if (!std::isfinite(u) || rs::to_usize(u) + 1 >= img.w || rs::to_usize(v) + 1 >= img.h) {
            continue;
        }
        const std::size_t x0 = rs::to_usize(u);
        const std::size_t y0 = rs::to_usize(v);
        const float fx = u - static_cast<float>(x0);
        const float fy = v - static_cast<float>(y0);
        auto p = [&](std::size_t x, std::size_t y) { return static_cast<float>(img.data[y * img.stride + x]); };
        const float top = p(x0, y0) * (1.0f - fx) + p(x0 + 1, y0) * fx;
        const float bot = p(x0, y0 + 1) * (1.0f - fx) + p(x0 + 1, y0 + 1) * fx;
        out[i] = rs::to_u8f(rs::roundf_fast(top * (1.0f - fy) + bot * fy));
    }
    return out;
}

bool Pinhole::covered(float x, float y) const {
    const std::int64_t xi = rs::to_i64(static_cast<double>(std::round(x)));
    const std::int64_t yi = rs::to_i64(static_cast<double>(std::round(y)));
    return xi >= 0 && yi >= 0 && static_cast<std::size_t>(xi) < w && static_cast<std::size_t>(yi) < h &&
           std::isfinite(lut[static_cast<std::size_t>(yi) * w + static_cast<std::size_t>(xi)].first);
}

Vec3 Pinhole::bearing(double x, double y) const {
    return g::normalize(g::mat_vec(cam_from_virt, {(x - cx) / f, (y - cy) / f, 1.0}));
}

std::expected<Rig, std::string> Rig::from_calib(const cam::DeviceCalib& cal, std::span<const std::size_t> ids) {
    std::vector<Pose> poses;
    for (std::size_t i : ids) {
        auto p = cam::slam::t_imu_cam(cal, i);
        if (!p) {
            return std::unexpected("camera " + std::to_string(i) + ": no camera-IMU extrinsic in the calibration");
        }
        poses.push_back(*p);
    }
    const std::size_t n = std::clamp<std::size_t>(poses.size(), 1, 2);
    double sum = -0.0;
    for (std::size_t k = 0; k < poses.size() && k < n; k++) {
        const Vec3 a = g::col(poses[k].r, 2);
        sum += std::atan2(a[1], std::sqrt(a[0] * a[0] + a[2] * a[2]));
    }
    const double pitch = sum / static_cast<double>(n);
    const Mat3 bv = Pinhole::body_from_virt(pitch);
    Rig rig;
    for (std::size_t k = 0; k < ids.size(); k++) {
        RigCam c;
        c.id = ids[k];
        c.intr = cal.cameras[ids[k]].intr;
        c.body_from_cam = poses[k];
        const Mat3 cam_from_virt = g::mat_mul(g::transpose(poses[k].r), bv);
        c.pinhole = Pinhole::make(c.intr, cam_from_virt);
        rig.cams.push_back(std::move(c));
    }
    if (rig.cams.empty()) {
        return std::unexpected("empty rig");
    }
    return rig;
}

std::vector<Pose> Rig::body_from_cams() const {
    std::vector<Pose> v;
    v.reserve(cams.size());
    for (const auto& c : cams) {
        v.push_back(c.body_from_cam);
    }
    return v;
}

namespace {

/// f32::min(255.0) of a rounded value.
inline float min255(float v) {
    return (v != v) ? 255.0f : (v < 255.0f ? v : 255.0f); // f32::min(255.0): NaN gives 255
}

/// 3x3 box filter times `g` (rounded, clamped); border pixels and 0 pixels (outside the fisheye) only scaled, so the
/// black mask edge stays where it is.
void box3(std::span<std::uint8_t> img, std::size_t w, float gain) {
    const std::size_t h = img.size() / w;
    const std::vector<std::uint8_t> src(img.begin(), img.end());
    for (std::size_t y = 0; y < h; y++) {
        for (std::size_t x = 0; x < w; x++) {
            const std::size_t i = y * w + x;
            float v;
            if (src[i] == 0 || x == 0 || y == 0 || x + 1 == w || y + 1 == h) {
                v = static_cast<float>(src[i]);
            } else {
                std::uint32_t s = 0;
                for (std::size_t yy = y - 1; yy <= y + 1; yy++) {
                    for (std::size_t xx = x - 1; xx <= x + 1; xx++) {
                        s += src[yy * w + xx];
                    }
                }
                v = static_cast<float>(s) / 9.0f;
            }
            img[i] = rs::to_u8f(min255(rs::roundf_fast(v * gain)));
        }
    }
}

std::optional<double> sample(const std::vector<std::uint8_t>& img, std::size_t w, double x, double y) {
    const std::size_t h = img.size() / std::max<std::size_t>(w, 1);
    if (x < 0.0 || y < 0.0 || x >= static_cast<double>(w - 1) || y >= static_cast<double>(h - 1)) {
        return std::nullopt;
    }
    const std::size_t x0 = rs::to_usize(x);
    const std::size_t y0 = rs::to_usize(y);
    const double fx = x - static_cast<double>(x0);
    const double fy = y - static_cast<double>(y0);
    auto p = [&](std::size_t xx, std::size_t yy) { return static_cast<double>(img[yy * w + xx]); };
    const double top = p(x0, y0) * (1.0 - fx) + p(x0 + 1, y0) * fx;
    const double bot = p(x0, y0 + 1) * (1.0 - fx) + p(x0 + 1, y0 + 1) * fx;
    return top * (1.0 - fy) + bot * fy;
}

/// Closest points of two rays oa + s da, ob + u db: (s, u).
std::optional<std::pair<double, double>> ray_params(const Vec3& oa, const Vec3& da, const Vec3& ob, const Vec3& db) {
    const Vec3 w = g::sub(oa, ob);
    const double a = g::dot(da, da), b = g::dot(da, db), c = g::dot(db, db);
    const double d = g::dot(da, w), e = g::dot(db, w);
    const double den = a * c - b * b;
    if (std::fabs(den) < 1e-12) {
        return std::nullopt;
    }
    return std::pair{(b * e - c * d) / den, (a * e - b * d) / den};
}

} // namespace

float low_light_stretch(std::span<std::uint8_t> img, std::size_t w, const LowLight& p) {
    std::array<std::uint32_t, 256> hist{};
    for (std::uint8_t v : img) {
        hist[v]++;
    }
    std::uint32_t n = 0;
    for (std::size_t v = 1; v < 256; v++) {
        n += hist[v];
    }
    if (n == 0) {
        return 1.0f;
    }
    const std::uint32_t want = rs::to_u32(std::ceil(static_cast<double>(n) * 0.99));
    std::uint32_t acc = 0;
    std::size_t p99 = 255;
    for (std::size_t v = 1; v < 256; v++) {
        acc += hist[v];
        if (acc >= want) {
            p99 = v;
            break;
        }
    }
    if (p99 >= p.p99_below) {
        return 1.0f;
    }
    const float gain = std::fmin(static_cast<float>(p.target) / static_cast<float>(std::max<std::size_t>(p99, 1)),
                                 p.max_gain);
    if (gain <= 1.0f) {
        return 1.0f;
    }
    if (p.denoise && w >= 3 && img.size() >= 3 * w) {
        box3(img, w, gain);
    } else {
        std::array<std::uint8_t, 256> lut{};
        for (std::size_t v = 0; v < 256; v++) {
            lut[v] = rs::to_u8f(min255(rs::roundf_fast(static_cast<float>(v) * gain)));
        }
        for (auto& v : img) {
            v = lut[v];
        }
    }
    return gain;
}

Features features(const orb::Image& img, const RigCam& cam, const orb::OrbParams& p) {
    return features_with(img, cam, p, LOW_LIGHT);
}

Features features_with(const orb::Image& img, const RigCam& cam, const orb::OrbParams& p,
                       const std::optional<LowLight>& low) {
    const Pinhole& ph = cam.pinhole;
    std::vector<std::uint8_t> und = ph.undistort(img);
    if (low) {
        low_light_stretch(und, ph.w, *low);
    }
    const orb::Image view{ph.w, ph.h, ph.w, und};
    const float m = 24.0f;
    const orb::Mask mask = [&](float x, float y) {
        return ph.covered(x - m, y) && ph.covered(x + m, y) && ph.covered(x, y - m) && ph.covered(x, y + m);
    };
    const auto raw = orb::extract(view, &mask, p);
    Features f;
    f.w = ph.w;
    f.kps.reserve(raw.size());
    f.desc.reserve(raw.size());
    f.bearings.reserve(raw.size());
    for (const auto& [k, d] : raw) {
        f.kps.push_back(k);
        f.desc.push_back(d);
        f.bearings.push_back(ph.bearing(static_cast<double>(k.x), static_cast<double>(k.y)));
    }
    f.img = std::move(und);
    return f;
}

std::optional<double> refine_x(const Features& fa, const Features& fb, double xa, double ya, double xb, double yb) {
    constexpr int R = 4;
    std::array<double, 81> pa{};
    std::size_t n = 0;
    for (int dy = -R; dy <= R; dy++) {
        for (int dx = -R; dx <= R; dx++) {
            const auto v = sample(fa.img, fa.w, xa + dx, ya + dy);
            if (!v) {
                return std::nullopt;
            }
            pa[n++] = *v;
        }
    }
    double sa = -0.0;
    for (double v : pa) {
        sa += v;
    }
    const double ma = sa / 81.0;
    double va = -0.0;
    for (double v : pa) {
        va += (v - ma) * (v - ma);
    }
    if (va < 1e-6) {
        return std::nullopt;
    }
    std::array<double, 17> scores{};
    for (int k = -8; k <= 8; k++) {
        const double off = static_cast<double>(k) * 0.25;
        std::array<double, 81> pb{};
        std::size_t m = 0;
        for (int dy = -R; dy <= R; dy++) {
            for (int dx = -R; dx <= R; dx++) {
                const auto v = sample(fb.img, fb.w, xb + off + dx, yb + dy);
                if (!v) {
                    return std::nullopt;
                }
                pb[m++] = *v;
            }
        }
        double sb = -0.0;
        for (double v : pb) {
            sb += v;
        }
        const double mb = sb / 81.0;
        double vb = -0.0;
        for (double v : pb) {
            vb += (v - mb) * (v - mb);
        }
        double cov = -0.0;
        for (std::size_t i = 0; i < 81; i++) {
            cov += (pa[i] - ma) * (pb[i] - mb);
        }
        scores[static_cast<std::size_t>(k + 8)] = vb < 1e-6 ? -1.0 : cov / std::sqrt(va * vb);
    }
    // max_by: the last of equal maxima
    std::size_t bi = 0;
    for (std::size_t i = 1; i < scores.size(); i++) {
        if (scores[i] >= scores[bi]) {
            bi = i;
        }
    }
    const double best = scores[bi];
    if (best < 0.8 || bi == 0 || bi == scores.size() - 1) {
        return std::nullopt;
    }
    const double l = scores[bi - 1], c = best, r = scores[bi + 1];
    const double den = l - 2.0 * c + r;
    const double frac = std::fabs(den) > 1e-12 ? 0.5 * (l - r) / den : 0.0;
    return xb + ((static_cast<double>(bi) - 8.0) + rf::clamp(frac, -0.5, 0.5)) * 0.25;
}

std::vector<StereoPoint> triangulate(const Features& fa, const Features& fb, const RigCam& a, const RigCam& b,
                                     const StereoParams& p) {
    const Vec3 oa = a.body_from_cam.t;
    const Vec3 ob = b.body_from_cam.t;
    std::vector<Vec3> da, db, normals;
    da.reserve(fa.bearings.size());
    db.reserve(fb.bearings.size());
    for (const Vec3& f : fa.bearings) {
        da.push_back(g::mat_vec(a.body_from_cam.r, f));
    }
    for (const Vec3& f : fb.bearings) {
        db.push_back(g::mat_vec(b.body_from_cam.r, f));
    }
    const Vec3 base = g::sub(ob, oa);
    normals.reserve(da.size());
    for (const Vec3& d : da) {
        normals.push_back(g::normalize(g::cross(base, d)));
    }
    const double sin_epi = std::sin(rf::to_radians(p.epi_deg));
    const auto allow = [&](std::size_t i, std::size_t j) {
        return std::fabs(g::dot(normals[i], db[j])) < sin_epi && g::dot(da[i], db[j]) > 0.3;
    };
    const auto m = orb::match_gated(fa.desc, fb.desc, p.max_dist, p.ratio, allow);
    const double cos_tri = std::cos(rf::to_radians(p.tri_deg));
    std::vector<StereoPoint> out;
    for (const auto& [i, j, d] : m) {
        (void)d;
        const orb::Keypoint& ka = fa.kps[i];
        const orb::Keypoint& kb = fb.kps[j];
        const auto xb = refine_x(fa, fb, static_cast<double>(ka.x), static_cast<double>(ka.y),
                                 static_cast<double>(kb.x), static_cast<double>(kb.y));
        if (!xb) {
            continue;
        }
        const Vec3 dbj = g::mat_vec(b.body_from_cam.r, b.pinhole.bearing(*xb, static_cast<double>(kb.y)));
        const auto su = ray_params(oa, da[i], ob, dbj);
        if (!su) {
            continue;
        }
        const auto [s, u] = *su;
        if (s < p.min_depth || s > p.max_depth || u <= 0.0) {
            continue;
        }
        const Vec3 x = g::scale(g::add(g::add(oa, g::scale(da[i], s)), g::add(ob, g::scale(dbj, u))), 0.5);
        const Vec3 ra = g::normalize(g::sub(x, oa));
        const Vec3 rb = g::normalize(g::sub(x, ob));
        if (g::dot(ra, da[i]) < cos_tri || g::dot(rb, dbj) < cos_tri) {
            continue;
        }
        out.push_back(StereoPoint{x, fa.desc[i], s, i});
    }
    return out;
}

} // namespace pibridge::roommap::stereo
