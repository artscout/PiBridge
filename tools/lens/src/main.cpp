// SPDX-License-Identifier: MIT
// pibridge-lens: the lens files of the Monado Pimax driver.
//   pibridge-lens crystal-profile IN.bin OUTDIR   the Crystal Super profile: IN (the like-Pimax LUT, made from the
//                                                 user's own Pimax install) turned by +-atan(0.0819) per eye, written as
//                                                 OUTDIR/crystal-pimax-native-lut.bin + crystal-pimax-native.conf
//   pibridge-lens turn IN.bin OUT.bin LEFT_DEG RIGHT_DEG   any yaw turn of a LUT (A/B variants)
//   pibridge-lens info FILE.bin                   the grid, serial and FOV port of a LUT
#include "pibridge/cam/io.h"
#include "pibridge/lens/lut.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

using pibridge::lens::Lut;

constexpr double kCrystalAxisTan = 0.0819; // the eye axis of the Crystal Super, verified in the headset 2026-10-22

int usage() {
    std::fprintf(stderr, "usage: pibridge-lens crystal-profile IN.bin OUTDIR\n"
                         "       pibridge-lens turn IN.bin OUT.bin LEFT_DEG RIGHT_DEG\n"
                         "       pibridge-lens info FILE.bin\n");
    return 2;
}

bool write_lut(const Lut& l, const std::string& path) {
    const auto b = l.to_bytes();
    const std::string s(reinterpret_cast<const char*>(b.data()), b.size());
    if (auto w = pibridge::cam::write_file(path, s); !w) {
        std::fprintf(stderr, "pibridge-lens: %s: %s\n", path.c_str(), w.error().text().c_str());
        return false;
    }
    return true;
}

void info(const Lut& l, const std::string& what) {
    std::printf("%s: grid %zu, serial \"%s\"\n", what.c_str(), l.n, l.serial.c_str());
    for (std::size_t e = 0; e < 2; e++) {
        const auto f = l.fov_deg(e);
        std::printf("  eye %zu: FOV left %.2f right %.2f up %.2f down %.2f deg\n", e, f[0], f[1], f[2], f[3]);
    }
}

} // namespace

int main(int argc, char** argv) {
    const std::vector<std::string> a(argv + 1, argv + argc);
    if (a.empty()) {
        return usage();
    }
    const auto load = [](const std::string& p) {
        auto l = Lut::load(p);
        if (!l) {
            std::fprintf(stderr, "pibridge-lens: %s\n", l.error().c_str());
        }
        return l;
    };
    if (a[0] == "info" && a.size() == 2) {
        const auto l = load(a[1]);
        if (!l) {
            return 1;
        }
        info(*l, a[1]);
        return 0;
    }
    if (a[0] == "turn" && a.size() == 5) {
        const auto l = load(a[1]);
        if (!l) {
            return 1;
        }
        const double ld = std::strtod(a[3].c_str(), nullptr), rd = std::strtod(a[4].c_str(), nullptr);
        const auto t = l->rotated_yaw({ld * M_PI / 180.0, rd * M_PI / 180.0});
        if (!write_lut(t, a[2])) {
            return 1;
        }
        info(t, a[2]);
        return 0;
    }
    if (a[0] == "crystal-profile" && a.size() == 3) {
        const auto l = load(a[1]);
        if (!l) {
            return 1;
        }
        const double th = std::atan(kCrystalAxisTan);
        const auto t = l->rotated_yaw({th, -th});
        const std::string lut = a[2] + "/crystal-pimax-native-lut.bin";
        const std::string conf = a[2] + "/crystal-pimax-native.conf";
        if (!write_lut(t, lut)) {
            return 1;
        }
        const std::string text =
            "# Crystal Super Oled native (pibridge-lens crystal-profile): the like-Pimax lens geometry with each eye's axis\n"
            "# set by a true (projective) rotation of atan(0.0819) = 4.68 deg, mirrored per eye. Verified in the headset\n"
            "# 2026-10-22: correct world scale and straight lines on head turns.\n"
            "cant_deg = 0\n"
            "lut = crystal-pimax-native-lut.bin\n"
            "lut_margin = 0\n";
        if (auto w = pibridge::cam::write_file(conf, text); !w) {
            std::fprintf(stderr, "pibridge-lens: %s: %s\n", conf.c_str(), w.error().text().c_str());
            return 1;
        }
        info(t, lut);
        std::printf("wrote %s\n", conf.c_str());
        return 0;
    }
    return usage();
}
