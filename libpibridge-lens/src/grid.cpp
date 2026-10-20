// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-lens/src/grid.rs (MIT OR Apache-2.0).
#include "pibridge/lens/grid.h"

#include "pibridge/cam/rustfmt.h"
#include "pibridge/lens/fmt.h"
#include "pibridge/lens/font.h"

#include <algorithm>
#include <cmath>
#include <format>

namespace pibridge::lens::grid {

namespace rf = cam::rfmt;

GridOptions GridOptions::for_panel(std::uint32_t div) {
    div = std::max<std::uint32_t>(div, 1);
    GridOptions o;
    o.width = 3840 / div;
    o.height = 3552 / div;
    o.divisions = 10;
    o.rings_deg = {10.0f, 20.0f, 30.0f, 40.0f, 50.0f};
    return o;
}

namespace {

struct Field {
    std::size_t w;
    std::size_t h;
    std::vector<float> v;
    Field(std::size_t w_, std::size_t h_) : w(w_), h(h_), v(w_ * h_, 0.0f) {}
    float at(std::size_t x, std::size_t y) const { return v[y * w + x]; }
    // Central-difference gradient magnitude per pixel.
    float grad(std::size_t x, std::size_t y) const {
        const std::size_t x0 = x > 0 ? x - 1 : 0;
        const std::size_t x1 = std::min(x + 1, w - 1);
        const std::size_t y0 = y > 0 ? y - 1 : 0;
        const std::size_t y1 = std::min(y + 1, h - 1);
        const float gx = (at(x1, y) - at(x0, y)) / static_cast<float>(std::max<std::size_t>(x1 - x0, 1));
        const float gy = (at(x, y1) - at(x, y0)) / static_cast<float>(std::max<std::size_t>(y1 - y0, 1));
        return std::sqrt(gx * gx + gy * gy);
    }
};

using Rgb = std::array<std::uint8_t, 3>;

std::uint8_t to_u8(float x) {
    return fmt::sat_cast<std::uint8_t>(fmt::clampf(std::round(x), 0.0f, 255.0f));
}

void blend(std::uint8_t* px, Rgb col, float a) {
    for (std::size_t i = 0; i < 3; i++) {
        const float p = px[i];
        px[i] = to_u8(p + (static_cast<float>(col[i]) - p) * a);
    }
}

void add_channel(std::uint8_t* px, std::size_t c, float a) {
    const float v = static_cast<float>(px[c]) + 255.0f * a;
    px[c] = to_u8(v);
}

bool unit(float x) {
    return 0.0f <= x && x <= 1.0f;
}

} // namespace

namespace detail {

float iso_cover(float value, float grad, float step, float halfw) {
    const float q = value / step;
    const float frac = std::fabs(q - std::round(q)) * step;
    const float d = frac / std::fmax(grad, 1e-9f);
    return fmt::clampf(halfw + 0.5f - d, 0.0f, 1.0f);
}

} // namespace detail

std::vector<std::uint8_t> render(const Lens& lens, std::size_t view, const GridOptions& opt) {
    using detail::iso_cover;
    const std::size_t w = opt.width;
    const std::size_t h = opt.height;
    const auto n = static_cast<float>(std::max<std::uint32_t>(opt.divisions, 1));
    const float aspect = lens.config().aspect;
    std::vector<Field> uf(3, Field(w, h));
    std::vector<Field> vf(3, Field(w, h));
    Field tx(w, h);
    Field ty(w, h);
    Field ang(w, h);
    std::vector<float> hide(w * h, 0.0f);
    Field sdist(w, h);
    const bool has_mask = lens.eye(view).has_mask();
    for (std::size_t y = 0; y < h; y++) {
        for (std::size_t x = 0; x < w; x++) {
            const float u = (static_cast<float>(x) + 0.5f) / static_cast<float>(w);
            const float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(h);
            const Uv3 uv = lens.compute(view, u, v);
            const std::size_t i = y * w + x;
            for (std::size_t c = 0; c < 3; c++) {
                uf[c].v[i] = uv[c][0];
                vf[c].v[i] = uv[c][1];
            }
            // tangent / angle from the unmasked green mapping (the mask pushes UV out of range)
            const Uv t = lens.tangent(view, 1, u, v);
            tx.v[i] = t[0];
            ty.v[i] = t[1];
            ang.v[i] = fmt::to_degrees(std::atan(std::hypot(t[0], t[1])));
            if (has_mask) {
                hide[i] = lens.hide_amount(view, u, v);
                const float s = lens.mask_distance(view, u, v);
                sdist.v[i] = std::isfinite(s) ? s : 1e3f;
            }
        }
    }

    std::vector<std::uint8_t> img(w * h * 3, 0);
    for (std::size_t y = 0; y < h; y++) {
        for (std::size_t x = 0; x < w; x++) {
            const std::size_t i = y * w + x;
            std::uint8_t* px = img.data() + i * 3;
            const bool g_in = unit(uf[1].v[i]) && unit(vf[1].v[i]);
            const Rgb base = hide[i] >= 0.5f ? Rgb{5, 5, 6} : g_in ? Rgb{26, 26, 32} : Rgb{12, 12, 16};
            std::copy(base.begin(), base.end(), px);
            if (hide[i] >= 0.5f) {
                continue;
            }
            // UV iso-lines, one colour channel each
            for (std::size_t c = 0; c < 3; c++) {
                for (int which = 0; which < 2; which++) {
                    const Field& f = which == 0 ? uf[c] : vf[c];
                    const float val = f.at(x, y);
                    if (!(-0.0005f <= val && val <= 1.0005f)) {
                        continue;
                    }
                    const float g = f.grad(x, y);
                    const float a = iso_cover(val, g, 1.0f / n, 0.5f);
                    if (a > 0.0f) {
                        const bool edge = (val < 0.01f || val > 0.99f) && which < 2;
                        add_channel(px, c, a * (edge ? 1.0f : 0.6f));
                    }
                }
            }
            // rings at known angles
            for (std::size_t ri = 0; ri < opt.rings_deg.size(); ri++) {
                const float a = iso_cover(ang.at(x, y) - opt.rings_deg[ri], ang.grad(x, y), 1e6f, 1.0f);
                if (a > 0.0f) {
                    blend(px, RING_COLORS[ri % RING_COLORS.size()], a);
                }
            }
            // optical axis
            const float ax = std::fmax(iso_cover(tx.at(x, y), tx.grad(x, y), 1e6f, 1.0f),
                                       iso_cover(ty.at(x, y), ty.grad(x, y), 1e6f, 1.0f));
            if (ax > 0.0f && ang.at(x, y) < 70.0f) {
                blend(px, {255, 128, 0}, ax * 0.9f);
            }
            // visibility mask boundary
            if (has_mask) {
                const float a = iso_cover(sdist.at(x, y), sdist.grad(x, y), 1e6f, 0.75f);
                if (a > 0.0f) {
                    blend(px, {255, 90, 160}, a);
                }
            }
        }
    }
    // panel outline
    const auto grey = [&](std::size_t x, std::size_t y) {
        std::uint8_t* p = img.data() + (y * w + x) * 3;
        p[0] = 150;
        p[1] = 150;
        p[2] = 150;
    };
    for (std::size_t x = 0; x < w; x++) {
        for (const std::size_t y : {std::size_t{0}, std::size_t{1}, h - 1, h >= 2 ? h - 2 : 0}) {
            grey(x, y);
        }
    }
    for (std::size_t y = 0; y < h; y++) {
        for (const std::size_t x : {std::size_t{0}, std::size_t{1}, w - 1, w >= 2 ? w - 2 : 0}) {
            grey(x, y);
        }
    }
    // labels: ring angle on the horizontal axis through the optical centre, title in the corner
    const auto scale = static_cast<std::uint32_t>(std::max<std::size_t>(h / 300, 1));
    const EyeConfig& e = lens.eye(view);
    const auto put = [&](std::string_view text, std::int32_t x0, std::int32_t y0, Rgb col) {
        font::draw_text(text, x0, y0, scale, [&](std::int32_t qx, std::int32_t qy) {
            if (qx >= 0 && qy >= 0 && static_cast<std::size_t>(qx) < w && static_cast<std::size_t>(qy) < h) {
                std::uint8_t* p = img.data() + (static_cast<std::size_t>(qy) * w + static_cast<std::size_t>(qx)) * 3;
                std::copy(col.begin(), col.end(), p);
            }
        });
    };
    const auto ys = fmt::sat_cast<std::int32_t>((0.5f - e.cy / 2.0f) * static_cast<float>(h));
    for (std::size_t ri = 0; ri < opt.rings_deg.size(); ri++) {
        const float deg = opt.rings_deg[ri];
        if (const auto r = lens.radius_for_angle(view, 1, deg)) {
            const float xp = e.cx + *r;
            const auto xs = fmt::sat_cast<std::int32_t>((0.5f + xp / (2.0f * aspect)) * static_cast<float>(w));
            if (xs > 0 && static_cast<std::size_t>(xs) < w) {
                const std::string label = rf::fixed(deg, 0) + "\xc2\xb0";
                const auto [tw, th] = font::text_size(label, scale);
                (void)th;
                put(label, xs - static_cast<std::int32_t>(tw) / 2, ys - 9 * static_cast<std::int32_t>(scale),
                    RING_COLORS[ri % RING_COLORS.size()]);
            }
        }
    }
    const std::string title =
        std::format("{} EYE  UV LINES EVERY {}", view == 0 ? "LEFT" : "RIGHT", rf::fixed(1.0f / n, 2));
    put(title, 6, 6, {220, 220, 220});
    put("R G B LINES = CHANNELS  ORANGE = AXIS", 6, 6 + 9 * static_cast<std::int32_t>(scale), {180, 180, 180});
    return img;
}

std::string summary(const Lens& lens, std::size_t view) {
    const EyeConfig& e = lens.eye(view);
    const LensConfig& cfg = lens.config();
    std::string s;
    s += std::format("eye {} ({}): distortion {}, rendered FOV L{} R{} U{} D{} deg, optical centre ({}, {}) "
                     "half-heights\n",
                     view, view == 0 ? "left" : "right", cfg.enabled ? "on" : "OFF", rf::fixed(e.fov[0], 1),
                     rf::fixed(e.fov[1], 1), rf::fixed(e.fov[2], 1), rf::fixed(e.fov[3], 1), rf::fixed(e.cx, 4, true),
                     rf::fixed(e.cy, 4, true));
    s += "  panel radius where the green channel shows a ray at angle off the optical axis (half-heights, px of 3552 "
         "tall):\n";
    for (const float deg : {10.0f, 20.0f, 30.0f, 40.0f, 50.0f}) {
        if (const auto r = lens.radius_for_angle(view, 1, deg)) {
            const float rr = lens.radius_for_angle(view, 0, deg).value_or(*r);
            const float rb = lens.radius_for_angle(view, 2, deg).value_or(*r);
            s += std::format("    {} deg: r = {} ({} px)   red {} px, blue {} px vs green\n", rf::fixed_w(deg, 4, 0),
                             rf::fixed(*r, 4), rf::fixed(*r * HALF_HEIGHT_PX, 0),
                             rf::fixed((rr - *r) * HALF_HEIGHT_PX, 1, true),
                             rf::fixed((rb - *r) * HALF_HEIGHT_PX, 1, true));
        } else {
            s += std::format("    {} deg: not reached\n", rf::fixed_w(deg, 4, 0));
        }
    }
    const float a = cfg.aspect;
    s += "  tangent (green) at the panel edges and corners; 'in FOV' = inside the rendered image:\n";
    struct P {
        const char* name;
        float u;
        float v;
    };
    for (const P& p : {P{"left  edge ", 0.0f, 0.5f}, P{"right edge ", 1.0f, 0.5f}, P{"top   edge ", 0.5f, 0.0f},
                       P{"bottom edge", 0.5f, 1.0f}, P{"corner TL  ", 0.0f, 0.0f}, P{"corner TR  ", 1.0f, 0.0f},
                       P{"corner BL  ", 0.0f, 1.0f}, P{"corner BR  ", 1.0f, 1.0f}}) {
        const Uv t = lens.tangent(view, 1, p.u, p.v);
        const Uv uv = lens.tangent_to_uv(view, t);
        const float ang = fmt::to_degrees(std::atan(std::hypot(t[0], t[1])));
        const bool inside = unit(uv[0]) && unit(uv[1]);
        s += std::format("    {} panel ({},{}): {} deg off axis, image uv ({},{}) {}\n", p.name,
                         rf::fixed((p.u - 0.5f) * 2.0f * a - e.cx, 3, true),
                         rf::fixed((0.5f - p.v) * 2.0f - e.cy, 3, true), rf::fixed_w(ang, 5, 1), rf::fixed(uv[0], 3),
                         rf::fixed(uv[1], 3), inside ? "in FOV" : "OUTSIDE FOV (black)");
    }
    std::uint32_t in_fov = 0;
    std::uint32_t masked = 0;
    const std::uint32_t total = 200u * 200;
    for (int j = 0; j < 200; j++) {
        for (int i = 0; i < 200; i++) {
            const float u = (static_cast<float>(i) + 0.5f) / 200.0f;
            const float v = (static_cast<float>(j) + 0.5f) / 200.0f;
            const Uv g = lens.compute(view, u, v)[1];
            if (unit(g[0]) && unit(g[1])) {
                in_fov++;
            }
            if (lens.hide_amount(view, u, v) >= 0.5f) {
                masked++;
            }
        }
    }
    s += std::format("  {}% of the panel shows image content (rest is black: outside the rendered FOV or masked); {}% "
                     "hidden by the visibility mask\n",
                     rf::fixed(100.0 * double{static_cast<double>(in_fov)} / static_cast<double>(total), 1),
                     rf::fixed(100.0 * static_cast<double>(masked) / static_cast<double>(total), 1));
    std::vector<std::string> folds;
    const char* names[3] = {"red", "green", "blue"};
    for (std::size_t c = 0; c < 3; c++) {
        if (const auto r = lens.first_fold(view, c, 1.6f)) {
            folds.push_back(std::format("{}: r={}", names[c], rf::fixed(*r, 3)));
        }
    }
    if (folds.empty()) {
        s += std::format("  radial curve is monotonic out to r = 1.6 for all channels (panel corner is r = {})\n",
                         rf::fixed(std::sqrt(a * a + 1.0f), 3));
    } else {
        std::string j;
        for (std::size_t i = 0; i < folds.size(); i++) {
            if (i > 0) {
                j += ", ";
            }
            j += folds[i];
        }
        s += std::format("  WARNING: radial curve folds back (image mirrors) at {}\n", j);
    }
    if (e.fit_range > 0.0f) {
        s += std::format("  fit_range {}: beyond this radius the curve continues as its tangent line\n",
                         rf::fixed(e.fit_range, 3));
    }
    if (e.has_mask()) {
        using fmt::display_f32;
        s += std::format("  visibility mask: {} a={} b={} n={} blackout L{} R{} T{} B{} feather {}\n",
                         mask_shape_name(e.mask.shape), display_f32(e.mask.a), display_f32(e.mask.b),
                         display_f32(e.mask.n), display_f32(e.blackout[0]), display_f32(e.blackout[1]),
                         display_f32(e.blackout[2]), display_f32(e.blackout[3]), display_f32(e.mask_feather));
    }
    return s;
}

} // namespace pibridge::lens::grid
