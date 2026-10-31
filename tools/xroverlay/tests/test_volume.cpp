// SPDX-License-Identifier: MIT
// The volume indicator: the frame per percent, the bar grows with it, the file is shown only while fresh.
#include "../src/volume.h"

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <sys/stat.h>
#include <unistd.h>
#include <utime.h>

using namespace pibridge::xroverlay;

static int g_fail = 0;
#define CHECK(c)                                                                                                      \
    do {                                                                                                              \
        if (!(c)) {                                                                                                   \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);                                                  \
            g_fail++;                                                                                                 \
        }                                                                                                             \
    } while (0)

// how many lit bar pixels a frame has on its middle row
static int barPixels(const std::vector<std::uint8_t>& px, int frame) {
    int n = 0;
    const std::size_t y = static_cast<std::size_t>(frame) * kVolH + kVolH / 2;
    for (std::uint32_t x = 130; x < kVolW - 40; x++) {
        const std::uint8_t* p = &px[(y * kVolW + x) * 4];
        n += p[2] > 200 && p[0] < 160 ? 1 : 0; // the level's blue, not the grey track
    }
    return n;
}

int main() {
    CHECK(volumeFrame(0) == 0 && volumeFrame(100) == kVolFrames - 1 && volumeFrame(50) == 10);
    CHECK(volumeFrame(-5) == 0 && volumeFrame(140) == kVolFrames - 1 && volumeFrame(47) == 9 && volumeFrame(48) == 10);
    const auto px = drawVolumeSheet();
    CHECK(px.size() == static_cast<std::size_t>(kVolW) * kVolH * kVolFrames * 4);
    CHECK(barPixels(px, 0) == 0);
    CHECK(barPixels(px, 5) > 0 && barPixels(px, 10) > barPixels(px, 5) && barPixels(px, 20) > barPixels(px, 10));
    // the corners stay transparent (a pill, not a box)
    CHECK(px[3] == 0);

    const char* tmp = std::getenv("TMPDIR");
    const std::string path = std::string(tmp != nullptr ? tmp : "/tmp") + "/pibridge-volume-hud-test-" +
                             std::to_string(::getpid());
    CHECK(!volumeToShow(path)); // no file
    std::ofstream(path) << "35\n";
    CHECK(volumeToShow(path) == 35);
    struct utimbuf old {std::time(nullptr) - 10, std::time(nullptr) - 10};
    ::utime(path.c_str(), &old);
    CHECK(!volumeToShow(path)); // too old: not shown
    std::filesystem::remove(path);
    std::printf("xroverlay volume: %s\n", g_fail == 0 ? "ok" : "FAILED");
    return g_fail == 0 ? 0 : 1;
}
