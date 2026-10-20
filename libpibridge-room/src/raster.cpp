// SPDX-License-Identifier: MIT
#include "pibridge/room/raster.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace pibridge::room {

namespace {

struct P {
    double x, y;
    bool ok;
};

P toScreen(const Mat4& m, const Vertex& v, int w, int h) {
    const auto c = transform4(m, {v.x, v.y, v.z});
    if (c[3] <= 0.02) {
        return {0, 0, false}; // behind the eye: the whole primitive is dropped (good enough for a preview)
    }
    return {(c[0] / c[3] * 0.5 + 0.5) * w, (c[1] / c[3] * 0.5 + 0.5) * h, true};
}

void blend(Image& img, int x, int y, const Vertex& v) {
    if (x < 0 || y < 0 || x >= img.w || y >= img.h) {
        return;
    }
    const auto i = static_cast<std::size_t>((y * img.w + x) * 3);
    const float a = std::clamp(v.a, 0.0F, 1.0F);
    const float c[3] = {v.r, v.g, v.b};
    for (std::size_t k = 0; k < 3; k++) {
        const float dst = img.rgb[i + k] / 255.0F;
        const float out = dst * (1 - a) + std::clamp(c[k], 0.0F, 1.0F) * a;
        img.rgb[i + k] = static_cast<std::uint8_t>(std::lround(out * 255));
    }
}

} // namespace

Image rasterize(const Batch& b, const Mat4& m, int w, int h, Rgba bg) {
    Image img{w, h, std::vector<std::uint8_t>(static_cast<std::size_t>(w * h * 3))};
    for (std::size_t i = 0; i < img.rgb.size(); i += 3) {
        img.rgb[i] = static_cast<std::uint8_t>(bg.r * 255);
        img.rgb[i + 1] = static_cast<std::uint8_t>(bg.g * 255);
        img.rgb[i + 2] = static_cast<std::uint8_t>(bg.b * 255);
    }
    rasterizeOnto(img, b, m);
    return img;
}

void rasterizeOnto(Image& img, const Batch& b, const Mat4& m) {
    const int w = img.w;
    const int h = img.h;
    for (std::size_t t = 0; t + 2 < b.tris.size(); t += 3) {
        const P a = toScreen(m, b.tris[t], w, h);
        const P c = toScreen(m, b.tris[t + 1], w, h);
        const P d = toScreen(m, b.tris[t + 2], w, h);
        if (!a.ok || !c.ok || !d.ok) {
            continue;
        }
        const double area = (c.x - a.x) * (d.y - a.y) - (c.y - a.y) * (d.x - a.x);
        if (std::fabs(area) < 1e-9) {
            continue;
        }
        const int x0 = std::max(0, static_cast<int>(std::floor(std::min({a.x, c.x, d.x}))));
        const int x1 = std::min(w - 1, static_cast<int>(std::ceil(std::max({a.x, c.x, d.x}))));
        const int y0 = std::max(0, static_cast<int>(std::floor(std::min({a.y, c.y, d.y}))));
        const int y1 = std::min(h - 1, static_cast<int>(std::ceil(std::max({a.y, c.y, d.y}))));
        for (int y = y0; y <= y1; y++) {
            for (int x = x0; x <= x1; x++) {
                const double px = x + 0.5;
                const double py = y + 0.5;
                const double w0 = ((c.x - px) * (d.y - py) - (c.y - py) * (d.x - px)) / area;
                const double w1 = ((d.x - px) * (a.y - py) - (d.y - py) * (a.x - px)) / area;
                const double w2 = 1 - w0 - w1;
                if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
                    blend(img, x, y, b.tris[t]);
                }
            }
        }
    }
    for (std::size_t l = 0; l + 1 < b.lines.size(); l += 2) {
        const P a = toScreen(m, b.lines[l], w, h);
        const P c = toScreen(m, b.lines[l + 1], w, h);
        if (!a.ok || !c.ok) {
            continue;
        }
        const int n = static_cast<int>(std::max(std::fabs(c.x - a.x), std::fabs(c.y - a.y))) + 1;
        if (n > 4 * (w + h)) {
            continue;
        }
        for (int k = 0; k <= n; k++) {
            const double s = static_cast<double>(k) / n;
            blend(img, static_cast<int>(a.x + (c.x - a.x) * s), static_cast<int>(a.y + (c.y - a.y) * s), b.lines[l]);
        }
    }
}

bool writePpm(const Image& img, const std::string& path) {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (f == nullptr) {
        return false;
    }
    std::fprintf(f, "P6\n%d %d\n255\n", img.w, img.h);
    const bool ok = std::fwrite(img.rgb.data(), 1, img.rgb.size(), f) == img.rgb.size();
    return std::fclose(f) == 0 && ok;
}

Mat4 lookAt(Vec3 eye, Vec3 at) {
    const Vec3 f = (at - eye).norm();
    Vec3 r = f.cross({0, 1, 0});
    if (r.length() < 1e-6) {
        r = {1, 0, 0};
    }
    r = r.norm();
    const Vec3 u = r.cross(f);
    Mat34 pose; // eye -> stage: x = r, y = u, z = -f
    pose.m[0] = {r.x, u.x, -f.x, eye.x};
    pose.m[1] = {r.y, u.y, -f.y, eye.y};
    pose.m[2] = {r.z, u.z, -f.z, eye.z};
    return mat4From34(inv34(pose));
}

} // namespace pibridge::room
