// SPDX-License-Identifier: MIT
// The loading dots: the pulse walks across the dots, the sheet is antialiased, the request file ages out.
#include "../src/loading.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <sys/stat.h>
#include <utime.h>
#include <unistd.h>
#include <ctime>

using namespace pibridge::xroverlay;

static int g_fail = 0;
#define CHECK(c)                                                                                                      \
    do {                                                                                                              \
        if (!(c)) {                                                                                                   \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);                                                  \
            g_fail++;                                                                                                 \
        }                                                                                                             \
    } while (0)

int main() {
    // the brightest dot moves left to right over a cycle
    CHECK(dotLevel(0, 0) > dotLevel(0, 1) && dotLevel(0, 0) > dotLevel(0, 2));
    CHECK(dotLevel(kDotsFrames / 3, 1) > dotLevel(kDotsFrames / 3, 0));
    CHECK(dotLevel(2 * kDotsFrames / 3, 2) > dotLevel(2 * kDotsFrames / 3, 1));
    for (int f = 0; f < kDotsFrames; f++) {
        for (int d = 0; d < 3; d++) {
            CHECK(dotLevel(f, d) >= 0.25 - 1e-9 && dotLevel(f, d) <= 1.0 + 1e-9);
        }
    }
    CHECK(dotsFrameAt(0) == 0);
    CHECK(dotsFrameAt(kDotsCycleS * 0.5) == kDotsFrames / 2);
    CHECK(std::abs(dotsFrameAt(kDotsCycleS * 1.5) - kDotsFrames / 2) <= 1); // the next cycle, the same place
    CHECK(dotsFrameAt(kDotsCycleS - 1e-6) == kDotsFrames - 1);

    // the sheet: size, transparent corners, an opaque dot centre, partial alpha at the edge (antialiased)
    const auto px = drawDotsSheet();
    CHECK(px.size() == static_cast<std::size_t>(kDotsW) * kDotsH * kDotsFrames * 4);
    const auto alpha = [&](int f, int x, int y) { return px[((static_cast<std::size_t>(f) * kDotsH + y) * kDotsW + x) * 4 + 3]; };
    CHECK(alpha(0, 0, 0) == 0 && alpha(0, kDotsW - 1, kDotsH - 1) == 0);
    CHECK(alpha(0, kDotsW / 4, kDotsH / 2) > 240);
    int partial = 0;
    for (int x = 0; x < static_cast<int>(kDotsW); x++) {
        const int a = alpha(0, x, kDotsH / 2);
        partial += a > 0 && a < 200;
    }
    CHECK(partial >= 6);

    // the request file: missing, fresh, stale
    const std::string p = (std::filesystem::temp_directory_path() / ("pibridge-loading-test-" + std::to_string(::getpid()))).string();
    std::filesystem::remove(p);
    CHECK(!loadingWanted(p));
    std::ofstream(p) << "1\n";
    CHECK(loadingWanted(p));
    utimbuf old{std::time(nullptr) - 60, std::time(nullptr) - 60};
    ::utime(p.c_str(), &old);
    CHECK(!loadingWanted(p));
    std::filesystem::remove(p);

    std::printf(g_fail == 0 ? "loading: ok\n" : "loading: %d failed\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
