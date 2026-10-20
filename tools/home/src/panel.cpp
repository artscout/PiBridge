// SPDX-License-Identifier: MIT
#include "panel.h"

#include <algorithm>
#include <cmath>

namespace pibridge::home::win {

room::Vec3 Geometry::at(double u, double v) const {
    const double th = (u - 0.5) * 2 * halfArc();
    const double y = centerY + (0.5 - v) * height;
    room::Vec3 p = axis - toward * (radius * std::cos(th)) + right * (radius * std::sin(th));
    p.y = y;
    return p;
}

void fitAspect(Geometry& g, std::uint32_t w, std::uint32_t h) {
    if (w == 0 || h == 0) {
        return;
    }
    g.height = std::min(kMaxHeight, g.width * static_cast<double>(h) / static_cast<double>(w));
}

Geometry place(const room::Mat34& head, double floorY, std::uint32_t w, std::uint32_t h) {
    Geometry g;
    room::Vec3 fwd = head.axis(2) * -1.0;
    fwd.y = 0;
    if (fwd.length() < 1e-6) {
        fwd = {0, 0, -1};
    }
    fwd = fwd.norm();
    g.toward = fwd * -1.0;
    g.right = room::Vec3{0, 1, 0}.cross(g.toward).norm(); // up x toward = right
    g.axis = head.pos();
    g.axis.y = 0;
    g.centerY = std::clamp(head.pos().y - 0.15, floorY + 0.9, floorY + 2.0);
    fitAspect(g, w == 0 ? 16 : w, h == 0 ? 10 : h);
    return g;
}

std::optional<Hit> hit(const Geometry& g, room::Vec3 o, room::Vec3 d) {
    const room::Vec3 q = o - g.axis;
    const double a = q.dot(g.right), b = q.dot(g.toward);
    const double da = d.dot(g.right), db = d.dot(g.toward);
    const double A = da * da + db * db;
    if (A < 1e-12) {
        return std::nullopt;
    }
    const double B = 2 * (a * da + b * db);
    const double C = a * a + b * b - g.radius * g.radius;
    const double disc = B * B - 4 * A * C;
    if (disc < 0) {
        return std::nullopt;
    }
    const double sq = std::sqrt(disc);
    // the far root: from inside the cylinder that is the panel's side in front of you
    for (const double t : {(-B + sq) / (2 * A), (-B - sq) / (2 * A)}) {
        if (t <= 1e-4) {
            continue;
        }
        const double pa = a + da * t, pb = b + db * t;
        const double th = std::atan2(pa, -pb);
        const double y = o.y + d.y * t;
        const double u = th / (2 * g.halfArc()) + 0.5;
        const double v = 0.5 - (y - g.centerY) / g.height;
        if (u >= 0 && u <= 1 && v >= 0 && v <= 1) {
            return Hit{u, v, t};
        }
    }
    return std::nullopt;
}

std::vector<MeshVertex> mesh(const Geometry& g, int segments) {
    std::vector<MeshVertex> m;
    segments = std::clamp(segments, 1, 120);
    const auto vtx = [&](double u, double v) {
        const room::Vec3 p = g.at(u, v);
        return MeshVertex{static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z), static_cast<float>(u),
                          static_cast<float>(v)};
    };
    for (int i = 0; i < segments; i++) {
        const double u0 = static_cast<double>(i) / segments, u1 = static_cast<double>(i + 1) / segments;
        const MeshVertex a = vtx(u0, 0), b = vtx(u1, 0), c = vtx(u1, 1), d = vtx(u0, 1);
        m.insert(m.end(), {a, b, c, a, c, d});
    }
    return m;
}

void Pointer::update(const std::optional<Hit>& h, double trigger, double stickY, double dt, std::uint32_t w, std::uint32_t hgt) {
    if (!pressed_ && trigger > 0.6 && h) {
        pressed_ = true;
    } else if (pressed_ && trigger < 0.4) {
        pressed_ = false;
    }
    if (h && w > 0 && hgt > 0) {
        state_.on = true;
        state_.x = static_cast<std::uint32_t>(std::clamp(h->u * w, 0.0, w - 1.0));
        state_.y = static_cast<std::uint32_t>(std::clamp(h->v * hgt, 0.0, hgt - 1.0));
    } else if (!pressed_) {
        state_.on = false; // a drag keeps its last place on the window until the release
    }
    state_.buttons = pressed_ ? 1U : 0U;
    if (state_.on && std::fabs(stickY) > 0.2) {
        const double s = (std::fabs(stickY) - 0.2) / 0.8 * (stickY > 0 ? 1 : -1);
        wheelAcc_ += s * 8.0 * 120.0 * dt;
        const auto whole = static_cast<std::int64_t>(wheelAcc_);
        wheel_ += whole;
        wheelAcc_ -= static_cast<double>(whole);
    }
}

void addRay(room::Batch& b, room::Vec3 o, room::Vec3 d, const std::optional<Hit>& h, const Geometry& g, bool pressed) {
    const double len = h ? h->t : 3.0;
    const room::Rgba c = pressed ? room::Rgba{0.55F, 0.85F, 1.0F, 1.0F} : room::Rgba{0.85F, 0.9F, 1.0F, 0.8F};
    b.line(o, o + d * len, c);
    if (h) {
        // a small cross on the panel, a little in front of it
        const room::Vec3 p = g.at(h->u, h->v) + (g.axis - g.at(h->u, h->v)).norm() * 0.004;
        const room::Vec3 up{0, 1, 0};
        const room::Vec3 rt = (g.axis - p).cross(up).norm();
        const double s = pressed ? 0.012 : 0.018;
        b.line(p - rt * s, p + rt * s, c);
        b.line(p - up * s, p + up * s, c);
    }
}

void drawOnto(room::Image& img, const room::Mat34& e, double tanL, double tanR, double tanU, double tanD, const Geometry& g,
              const std::vector<std::uint8_t>& bgra, std::uint32_t w, std::uint32_t h, double opacity) {
    if (w == 0 || h == 0 || bgra.size() < static_cast<std::size_t>(w) * h * 4) {
        return;
    }
    for (int y = 0; y < img.h; y++) {
        for (int x = 0; x < img.w; x++) {
            const double tx = tanL + (tanR - tanL) * (x + 0.5) / img.w, ty = tanU + (tanD - tanU) * (y + 0.5) / img.h;
            const room::Vec3 d = (e.axis(0) * tx + e.axis(1) * ty + e.axis(2) * -1.0).norm();
            const auto hh = hit(g, e.pos(), d);
            if (!hh) {
                continue;
            }
            const auto px = static_cast<std::size_t>(std::clamp(hh->u * w, 0.0, w - 1.0));
            const auto py = static_cast<std::size_t>(std::clamp(hh->v * h, 0.0, h - 1.0));
            const std::uint8_t* s = &bgra[(py * w + px) * 4];
            std::uint8_t* o = &img.rgb[(static_cast<std::size_t>(y) * img.w + x) * 3];
            const std::array<std::uint8_t, 3> rgb{s[2], s[1], s[0]};
            for (int k = 0; k < 3; k++) {
                o[k] = static_cast<std::uint8_t>(std::lround(o[k] * (1 - opacity) + rgb[static_cast<std::size_t>(k)] * opacity));
            }
        }
    }
}

std::vector<std::uint8_t> testCard(std::uint32_t w, std::uint32_t h) {
    std::vector<std::uint8_t> p(static_cast<std::size_t>(w) * h * 4);
    const auto put = [&](std::uint32_t x, std::uint32_t y, std::uint8_t r, std::uint8_t g, std::uint8_t b) {
        std::uint8_t* q = &p[(static_cast<std::size_t>(y) * w + x) * 4];
        q[0] = b;
        q[1] = g;
        q[2] = r;
        q[3] = 255;
    };
    for (std::uint32_t y = 0; y < h; y++) {
        for (std::uint32_t x = 0; x < w; x++) {
            const bool nav = x < w * 15 / 100;
            const bool bar = y > h - h / 26;
            const bool card = !nav && !bar && (x - w * 15 / 100) % (w / 3) > 24 && y % (h / 3) > 24 && y > h / 10;
            if (bar) {
                put(x, y, 18, 22, 30);
            } else if (nav) {
                put(x, y, 20, 24, 34);
            } else if (card) {
                put(x, y, 34, 40, 54);
            } else {
                put(x, y, 26, 30, 40);
            }
        }
    }
    return p;
}

} // namespace pibridge::home::win
