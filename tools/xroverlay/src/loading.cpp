// SPDX-License-Identifier: MIT
#include "loading.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <sys/stat.h>
#include <ctime>

namespace pibridge::xroverlay {

namespace {
constexpr double kPi = 3.14159265358979323846;
} // namespace

double dotLevel(int frame, int dot) {
    // a smooth bump that walks across the three dots once per cycle, a third of a cycle apart
    const double t = static_cast<double>(((frame % kDotsFrames) + kDotsFrames) % kDotsFrames) / kDotsFrames;
    const double ph = t - static_cast<double>(dot) / 3.0;
    const double c = std::cos(2.0 * kPi * ph);
    const double bump = std::pow(std::max(0.0, c), 3.0);
    return 0.25 + 0.75 * bump;
}

int dotsFrameAt(double t) {
    const double c = std::fmod(std::max(0.0, t), kDotsCycleS) / kDotsCycleS;
    return std::clamp(static_cast<int>(c * kDotsFrames), 0, kDotsFrames - 1);
}

std::vector<std::uint8_t> drawDotsSheet() {
    std::vector<std::uint8_t> px(static_cast<std::size_t>(kDotsW) * kDotsH * kDotsFrames * 4, 0);
    const double cy = kDotsH / 2.0;
    const double gap = kDotsW / 4.0;
    for (int f = 0; f < kDotsFrames; f++) {
        for (int d = 0; d < 3; d++) {
            const double lv = dotLevel(f, d);
            const double r = 18.0 + 10.0 * lv; // grows with its brightness
            const double cx = gap * (d + 1);
            // soft white-blue, antialiased edge (1.5 px ramp), a faint glow around it
            const int x0 = std::max(0, static_cast<int>(cx - r - 24)), x1 = std::min<int>(kDotsW - 1, static_cast<int>(cx + r + 24));
            const int y0 = std::max(0, static_cast<int>(cy - r - 24)), y1 = std::min<int>(kDotsH - 1, static_cast<int>(cy + r + 24));
            for (int y = y0; y <= y1; y++) {
                for (int x = x0; x <= x1; x++) {
                    const double dist = std::hypot(x + 0.5 - cx, y + 0.5 - cy);
                    const double core = std::clamp((r - dist) / 1.5 + 0.5, 0.0, 1.0);
                    const double glow = 0.35 * lv * std::exp(-std::pow(std::max(0.0, dist - r) / 9.0, 2.0));
                    const double a = std::clamp(core * (0.45 + 0.55 * lv) + glow * (1.0 - core), 0.0, 1.0);
                    if (a <= 0.0) {
                        continue;
                    }
                    std::uint8_t* p = &px[((static_cast<std::size_t>(f) * kDotsH + y) * kDotsW + x) * 4];
                    const double na = std::max(p[3] / 255.0, a);
                    p[0] = static_cast<std::uint8_t>(std::lround(200 + 30 * lv));
                    p[1] = static_cast<std::uint8_t>(std::lround(225 + 20 * lv));
                    p[2] = 255;
                    p[3] = static_cast<std::uint8_t>(std::lround(na * 255));
                }
            }
        }
    }
    return px;
}

std::string loadingRequestPath() {
    const char* rt = std::getenv("XDG_RUNTIME_DIR");
    return std::string(rt != nullptr && *rt != '\0' ? rt : "/tmp") + "/pibridge/loading";
}

bool loadingWanted(const std::string& path) {
    struct stat st {};
    if (::stat(path.c_str(), &st) != 0) {
        return false;
    }
    return std::difftime(std::time(nullptr), st.st_mtime) < kLoadingMaxAgeS;
}

} // namespace pibridge::xroverlay
