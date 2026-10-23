// SPDX-License-Identifier: MIT
// pibridge-headtrack: the Crystal Super's / Dream Air's inside-out head tracking without Monado (headtrack.h).
#include "headtrack.h"

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

namespace {

const char* USAGE =
    "usage: pibridge-headtrack [--engine pimax|basalt] [--vit LIB] [--slam-config FILE] [--camera SPEC]\n"
    "                          [--cams 0,1,2,3] [--fps 72|90]\n"
    "                          [--calib XML] [--pose-file PATH] [--cam-offset-us N]\n"
    "  --vit          the VIT SLAM library (default $PIBRIDGE_HEADTRACK_VIT, else\n"
    "                 ~/.local/share/pibridge/winbridge/libpibridge_vit_winbridge.so: Pimax's engine, winbridge.exe running)\n"
    "  --engine       pimax: Pimax Play's engine, its pose as is, controllers by its LED engine (default);\n"
    "                 basalt: Basalt + PiBridge's output layer, controllers from pibridge-ctrltrack\n"
    "  --slam-config  SLAM_CONFIG for engines that read one (Basalt)\n"
    "  --camera       camera source (default \"broker\": the camera broker's socket)\n"
    "  --cams         tiles handed to the engine (default 0,1,2,3)\n"
    "  --fps          panel rate (default 90)\n"
    "  --calib        factory calibration XML (default: the usual search)\n"
    "  --pose-file    default /dev/shm/pibridge-crystal-pose (what CustomHeadsetOpenVR reads)\n";

void say(std::string_view m) {
    std::time_t t = std::time(nullptr);
    char ts[16];
    std::strftime(ts, sizeof ts, "%H:%M:%S", std::localtime(&t));
    std::printf("%s %.*s\n", ts, static_cast<int>(m.size()), m.data());
    std::fflush(stdout);
}

std::vector<std::uint32_t> parse_cams(const std::string& s) {
    std::vector<std::uint32_t> out;
    for (std::size_t i = 0; i < s.size();) {
        const std::size_t j = s.find(',', i);
        out.push_back(static_cast<std::uint32_t>(std::stoul(s.substr(i, j - i))));
        i = j == std::string::npos ? s.size() : j + 1;
    }
    return out;
}

} // namespace

int main(int argc, char** argv) {
    pibridge::headtrack::Options o;
    if (const char* e = std::getenv("PIBRIDGE_HEADTRACK_VIT"); e && *e) {
        o.vit_library = e;
    } else if (const char* h = std::getenv("HOME")) {
        o.vit_library = std::string(h) + "/.local/share/pibridge/winbridge/libpibridge_vit_winbridge.so";
    }
    try {
        for (int i = 1; i < argc; i++) {
            const std::string a = argv[i];
            auto val = [&]() -> std::string {
                if (i + 1 >= argc) {
                    throw std::invalid_argument(a + " needs a value");
                }
                return argv[++i];
            };
            if (a == "--vit") {
                o.vit_library = val();
            } else if (a == "--slam-config") {
                o.slam_config = val();
            } else if (a == "--engine") {
                o.engine = val();
                if (o.engine != "pimax" && o.engine != "basalt") {
                    throw std::invalid_argument("--engine pimax|basalt");
                }
                o.exposure_mid = o.engine == "basalt";
            } else if (a == "--camera") {
                o.camera = val();
            } else if (a == "--cams") {
                o.cams = parse_cams(val());
            } else if (a == "--fps") {
                o.fps = static_cast<std::uint8_t>(std::stoi(val()));
            } else if (a == "--calib") {
                o.calibration = val();
            } else if (a == "--pose-file") {
                o.pose_file = val();
            } else if (a == "--cam-offset-us") {
                o.cam_offset_ns = std::stoll(val()) * 1000;
            } else if (a == "-h" || a == "--help") {
                std::fputs(USAGE, stdout);
                return 0;
            } else {
                throw std::invalid_argument("unknown option " + a);
            }
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n%s", e.what(), USAGE);
        return 2;
    }
    if (o.cams.empty() || o.fps == 0) {
        std::fputs(USAGE, stderr);
        return 2;
    }
    // SIGTERM / SIGINT end it: blocked here, waited for below (before any thread starts, so all inherit the mask)
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGTERM);
    sigaddset(&set, SIGINT);
    pthread_sigmask(SIG_BLOCK, &set, nullptr);

    pibridge::headtrack::Daemon d(o, say);
    if (!d.start()) {
        d.stop();
        return 1;
    }
    int sig = 0;
    sigwait(&set, &sig);
    say("stopping");
    d.stop();
    return 0;
}
