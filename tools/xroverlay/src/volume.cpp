// SPDX-License-Identifier: MIT
#include "volume.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <sys/stat.h>

namespace pibridge::xroverlay {

namespace {

// straight-alpha "over" of a colour with coverage a onto the pixel
void blend(std::uint8_t* p, double r, double g, double b, double a) {
    if (a <= 0.0) {
        return;
    }
    const double da = p[3] / 255.0;
    const double oa = a + da * (1.0 - a);
    const auto mix = [&](std::uint8_t d, double s) {
        return static_cast<std::uint8_t>(std::lround((s * a + d * da * (1.0 - a)) / oa));
    };
    p[0] = mix(p[0], r);
    p[1] = mix(p[1], g);
    p[2] = mix(p[2], b);
    p[3] = static_cast<std::uint8_t>(std::lround(oa * 255.0));
}

// coverage of a rounded rectangle (x0..x1, y0..y1, radius rad) at a pixel centre, antialiased over 1 px
double roundRect(double x, double y, double x0, double y0, double x1, double y1, double rad) {
    const double cx = std::clamp(x, x0 + rad, x1 - rad);
    const double cy = std::clamp(y, y0 + rad, y1 - rad);
    const double d = std::hypot(x - cx, y - cy) - rad;
    return std::clamp(0.5 - d, 0.0, 1.0);
}

} // namespace

int volumeFrame(int percent) {
    return std::clamp((std::clamp(percent, 0, 100) + 2) / 5, 0, kVolFrames - 1);
}

std::vector<std::uint8_t> drawVolumeSheet() {
    std::vector<std::uint8_t> px(static_cast<std::size_t>(kVolW) * kVolH * kVolFrames * 4, 0);
    const double h = kVolH;
    const double barX0 = 120, barX1 = kVolW - 36.0, barY0 = h / 2 - 9, barY1 = h / 2 + 9;
    for (int f = 0; f < kVolFrames; f++) {
        const double level = f / static_cast<double>(kVolFrames - 1);
        const double fillX1 = barX0 + (barX1 - barX0) * level;
        for (int y = 0; y < static_cast<int>(kVolH); y++) {
            for (int x = 0; x < static_cast<int>(kVolW); x++) {
                std::uint8_t* p = &px[((static_cast<std::size_t>(f) * kVolH + y) * kVolW + x) * 4];
                const double fx = x + 0.5, fy = y + 0.5;
                // the dark pill behind everything
                blend(p, 18, 22, 30, 0.82 * roundRect(fx, fy, 2, 2, kVolW - 2.0, h - 2, h / 2 - 2));
                // the speaker: a box and a cone, then sound waves growing with the level
                blend(p, 235, 242, 255, roundRect(fx, fy, 34, h / 2 - 11, 48, h / 2 + 11, 2));
                if (fx >= 46 && fx <= 66) {
                    const double half = 11 + (fx - 46) * (25.0 - 11.0) / 20.0;
                    blend(p, 235, 242, 255, std::clamp(half - std::fabs(fy - h / 2) + 0.5, 0.0, 1.0));
                }
                const double rr = std::hypot(fx - 60, fy - h / 2);
                const bool right = fx > 70;
                for (int w = 0; w < 2 && right; w++) {
                    const double radius = 20.0 + 12.0 * w;
                    const bool on = level > (w == 0 ? 0.0 : 0.5);
                    const double ring = std::clamp(2.5 - std::fabs(rr - radius), 0.0, 1.0);
                    if (std::fabs(fy - h / 2) < radius * 0.75) {
                        blend(p, 235, 242, 255, ring * (on ? 1.0 : 0.25));
                    }
                }
                // the bar: the track, then the level
                blend(p, 90, 100, 120, 0.6 * roundRect(fx, fy, barX0, barY0, barX1, barY1, 9));
                if (level > 0.0) {
                    blend(p, 110, 200, 255, roundRect(fx, fy, barX0, barY0, std::max(fillX1, barX0 + 18), barY1, 9));
                }
            }
        }
    }
    return px;
}

std::string volumeHudPath() {
    const char* rt = std::getenv("XDG_RUNTIME_DIR");
    return std::string(rt != nullptr && *rt != '\0' ? rt : "/tmp") + "/pibridge/volume-hud";
}

std::optional<int> volumeToShow(const std::string& path) {
    struct stat st {};
    if (::stat(path.c_str(), &st) != 0) {
        return std::nullopt;
    }
    struct timespec now {};
    clock_gettime(CLOCK_REALTIME, &now);
    const double age = static_cast<double>(now.tv_sec - st.st_mtim.tv_sec) + (now.tv_nsec - st.st_mtim.tv_nsec) / 1e9;
    if (age < 0.0 || age > kVolShowS) {
        return std::nullopt;
    }
    std::ifstream f(path);
    int pct = -1;
    if (!(f >> pct) || pct < 0) {
        return std::nullopt;
    }
    return std::min(pct, 100);
}

} // namespace pibridge::xroverlay
