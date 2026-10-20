// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-lens/src/mask.rs (MIT OR Apache-2.0).
#include "pibridge/lens/mask.h"

#include "pibridge/lens/fmt.h"

#include <cmath>
#include <limits>

namespace pibridge::lens {

namespace {

bool unit(float x) {
    return 0.0f <= x && x <= 1.0f;
}

// Is panel point (xp, yp) (half-heights from the optical centre) visible: inside the rendered FOV and not masked?
bool visible(const Lens& lens, std::size_t view, float xp, float yp) {
    const EyeConfig& e = lens.eye(view);
    const float a = lens.config().aspect;
    const float u = 0.5f + (xp + e.cx) / (2.0f * a);
    const float v = 0.5f - (yp + e.cy) / 2.0f;
    if (!unit(u) || !unit(v)) {
        return false;
    }
    if (lens.hide_amount(view, u, v) > 0.0f) {
        return false;
    }
    const Uv uv = lens.compute(view, u, v)[1];
    return unit(uv[0]) && unit(uv[1]);
}

// Distance along a tangent-plane direction from the origin to the rectangle of the rendered FOV.
float rect_distance(const Lens& lens, std::size_t view, float dx, float dy) {
    const auto [tl, tr, tu, td] = lens.fov_tan(view);
    float t = std::numeric_limits<float>::infinity();
    if (dx > 1e-9f) {
        t = std::fmin(t, tr / dx);
    } else if (dx < -1e-9f) {
        t = std::fmin(t, tl / -dx);
    }
    if (dy > 1e-9f) {
        t = std::fmin(t, tu / dy);
    } else if (dy < -1e-9f) {
        t = std::fmin(t, td / -dy);
    }
    return t;
}

std::array<float, 2> flip(std::array<float, 2> p) {
    return {p[0], -p[1]};
}

} // namespace

std::optional<MaskKind> mask_kind_from_i32(int v) {
    switch (v) {
    case 0:
        return MaskKind::HiddenTriangleMesh;
    case 1:
        return MaskKind::VisibleTriangleMesh;
    case 2:
        return MaskKind::LineLoop;
    default:
        return std::nullopt;
    }
}

std::optional<std::vector<std::array<float, 2>>> outline_tangent(const Lens& lens, std::size_t view, std::size_t n) {
    if (!visible(lens, view, 0.0f, 0.0f)) {
        return std::nullopt;
    }
    constexpr float TAU = 6.28318530717958647692528676655900577f;
    std::vector<std::array<float, 2>> pts;
    pts.reserve(n);
    for (std::size_t i = 0; i < n; i++) {
        const float phi = TAU * static_cast<float>(i) / static_cast<float>(n);
        const float dx = std::cos(phi);
        const float dy = std::sin(phi);
        float lo = 0.0f;
        float hi = 4.0f;
        for (int k = 0; k < 40; k++) {
            const float mid = 0.5f * (lo + hi);
            if (visible(lens, view, mid * dx, mid * dy)) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        const EyeConfig& e = lens.eye(view);
        const float a = lens.config().aspect;
        // panel point of the boundary -> (u, v) -> green tangent
        const float xp = lo * dx;
        const float yp = lo * dy;
        const float u = 0.5f + (xp + e.cx) / (2.0f * a);
        const float v = 0.5f - (yp + e.cy) / 2.0f;
        const Uv t = lens.tangent(view, 1, u, v);
        // keep it inside the FOV rectangle despite rounding
        const auto [tl, tr, tu, td] = lens.fov_tan(view);
        pts.push_back({fmt::clampf(t[0], -tl, tr), fmt::clampf(t[1], -td, tu)});
    }
    return pts;
}

std::optional<MaskMesh> visibility_mesh(const Lens& lens, std::size_t view, MaskKind kind, std::size_t n) {
    if (!lens.config().enabled || !lens.has_mask(view) || n < 8) {
        return std::nullopt;
    }
    const auto poly = outline_tangent(lens, view, n);
    if (!poly) {
        return std::nullopt;
    }
    MaskMesh m;
    for (const auto& p : *poly) {
        m.vertices.push_back(flip(p));
    }
    const auto nn = static_cast<std::uint32_t>(n);
    switch (kind) {
    case MaskKind::LineLoop:
        for (std::uint32_t i = 0; i < nn; i++) {
            m.indices.push_back(i);
        }
        break;
    case MaskKind::VisibleTriangleMesh:
        m.vertices.push_back({0.0f, 0.0f});
        m.indices.reserve(n * 3);
        for (std::uint32_t i = 0; i < nn; i++) {
            m.indices.insert(m.indices.end(), {nn, i, (i + 1) % nn});
        }
        break;
    case MaskKind::HiddenTriangleMesh:
        for (const auto& p : *poly) {
            const float len = std::hypot(p[0], p[1]);
            float dx = 1.0f;
            float dy = 0.0f;
            if (len > 1e-9f) {
                dx = p[0] / len;
                dy = p[1] / len;
            }
            const float d = rect_distance(lens, view, dx, dy);
            m.vertices.push_back(flip({dx * d, dy * d}));
        }
        m.indices.reserve(n * 6);
        for (std::uint32_t i = 0; i < nn; i++) {
            const std::uint32_t j = (i + 1) % nn;
            m.indices.insert(m.indices.end(), {i, nn + i, nn + j, i, nn + j, j});
        }
        break;
    }
    return m;
}

float mesh_area(const MaskMesh& m) {
    // Rust `sum()` of f32 starts at -0.0
    float s = -0.0f;
    for (std::size_t t = 0; t + 3 <= m.indices.size(); t += 3) {
        const auto& a = m.vertices[m.indices[t]];
        const auto& b = m.vertices[m.indices[t + 1]];
        const auto& c = m.vertices[m.indices[t + 2]];
        s += 0.5f * std::fabs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]));
    }
    return s;
}

} // namespace pibridge::lens
