// SPDX-License-Identifier: MIT
// pibridge-headtrack-replay: a head shadow take (shadow_take.h: the frame sets and IMU samples the shadow engine was
// fed in a live session) through a VIT engine (Basalt) again, offline, with any SLAM_CONFIG, writing the poses in the
// head shadow log's format (src 1). With --ref the live log's driving-engine rows (src 0) are copied in front, so the
// result is compared exactly like a live log (head_drift2.py): Basalt with another configuration against Pimax on
// the same session. docs/pibridge-headtrack.md "Head shadow mode".
#include "shadow_take.h"
#include "vit_host.h"

#include "pibridge/cam/imulog.h"
#include "pibridge/cam/io.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

namespace {

using pibridge::headtrack::EnginePose;
using pibridge::headtrack::ShadowTakeFrame;
using pibridge::headtrack::ShadowTakeReader;
using pibridge::headtrack::VitHost;

constexpr const char* USAGE =
    "usage: pibridge-headtrack-replay TAKE.bin IMU.csv --vit LIB [--slam-config FILE] [--out CSV] [--ref LOG.csv]\n"
    "                                 [--imu-offset-ns N] [--cam-offset-ns N] [--lead-ns N] [--max-lag-ms N] [--no-reset]\n"
    "  TAKE.bin, IMU.csv  head-shadow-*-take.bin and head-shadow-*-imu.csv of a live session (PIBRIDGE_HEAD_SHADOW_TAKE=1)\n"
    "  --vit LIB          the VIT library (Basalt: libbasalt.so), --slam-config its SLAM_CONFIG toml\n"
    "  --out CSV          the poses in the head shadow log format, src 1 (default TAKE with -replay.csv)\n"
    "  --ref LOG.csv      copy the src 0 rows (the driving engine) of this live log in front, for head_drift2.py\n"
    "  --imu-offset-ns N  add N to every IMU time (a camera-IMU time offset experiment); --cam-offset-ns N to every frame\n"
    "  --lead-ns N        push a frame set once the IMU reaches its time + N (default: the take's own arrival lead)\n"
    "  --max-lag-ms N     wait for the engine when its poses fall more than N ms behind the frames (default 400)\n"
    "  --no-reset         do not start the engine over when it diverges (the live mode does: 3 m/s, 0.5 m, 50 m)\n";

std::int64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

struct Args {
    std::vector<std::string> v;
    bool flag(const char* n) {
        auto it = std::find(v.begin(), v.end(), n);
        if (it == v.end()) {
            return false;
        }
        v.erase(it);
        return true;
    }
    std::string opt(const char* n, const std::string& def = "") {
        auto it = std::find(v.begin(), v.end(), n);
        if (it == v.end() || it + 1 == v.end()) {
            return def;
        }
        std::string val = *(it + 1);
        v.erase(it, it + 2);
        return val;
    }
};

} // namespace

int main(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; i++) {
        a.v.emplace_back(argv[i]);
    }
    const std::string vit = a.opt("--vit");
    const std::string config = a.opt("--slam-config");
    std::string out = a.opt("--out");
    const std::string ref = a.opt("--ref");
    const std::int64_t imu_off = std::atoll(a.opt("--imu-offset-ns", "0").c_str());
    const std::int64_t cam_off = std::atoll(a.opt("--cam-offset-ns", "0").c_str());
    const std::string lead_s = a.opt("--lead-ns");
    const std::int64_t max_lag = std::atoll(a.opt("--max-lag-ms", "400").c_str()) * 1'000'000;
    const bool no_reset = a.flag("--no-reset");
    if (a.v.size() != 2 || vit.empty()) {
        std::fputs(USAGE, stderr);
        return 2;
    }
    const std::string take_path = a.v[0], imu_path = a.v[1];
    if (out.empty()) {
        out = take_path.ends_with(".bin") ? take_path.substr(0, take_path.size() - 4) + "-replay.csv" : take_path + "-replay.csv";
    }

    // the IMU
    auto text = pibridge::cam::read_to_string(imu_path);
    if (!text) {
        std::fprintf(stderr, "%s: %s\n", imu_path.c_str(), text.error().text().c_str());
        return 1;
    }
    pibridge::cam::imulog::HeadImu imu = pibridge::cam::imulog::parse(*text);
    text->clear();
    text->shrink_to_fit();
    std::sort(imu.imu.begin(), imu.imu.end(), [](const auto& x, const auto& y) { return x.ts < y.ts; });
    if (imu.imu.empty()) {
        std::fprintf(stderr, "%s: no IMU samples\n", imu_path.c_str());
        return 1;
    }

    // the take; its arrival lead (how far the IMU had run ahead of a frame set when it was pushed)
    std::string err;
    auto take = ShadowTakeReader::open(take_path, &err);
    if (!take) {
        std::fprintf(stderr, "%s\n", err.c_str());
        return 1;
    }
    std::int64_t lead = 20'000'000;
    if (!lead_s.empty()) {
        lead = std::atoll(lead_s.c_str());
    } else {
        std::vector<std::int64_t> leads;
        ShadowTakeFrame f;
        while (leads.size() < 2000 && take->next(f, &err)) {
            if (f.arrival_ns != 0) {
                leads.push_back(f.arrival_ns - f.ts_ns);
            }
        }
        if (!leads.empty()) {
            std::nth_element(leads.begin(), leads.begin() + static_cast<std::ptrdiff_t>(leads.size() / 2), leads.end());
            lead = leads[leads.size() / 2];
        }
        take->rewind();
    }

    // the output: the reference's src 0 rows first
    std::FILE* o = std::fopen(out.c_str(), "w");
    if (o == nullptr) {
        std::fprintf(stderr, "%s: %s\n", out.c_str(), std::strerror(errno));
        return 1;
    }
    std::fputs("t_ns,src,px,py,pz,qx,qy,qz,qw,vx,vy,vz,event\n", o);
    std::uint64_t ref_rows = 0;
    if (!ref.empty()) {
        std::ifstream r(ref);
        std::string line;
        while (std::getline(r, line)) {
            const std::size_t c = line.find(',');
            if (c != std::string::npos && line.compare(c, 3, ",0,") == 0) {
                std::fputs(line.c_str(), o);
                std::fputc('\n', o);
                ref_rows++;
            }
        }
    }

    auto host = VitHost::open(vit, take->tiles, config, &err);
    if (!host) {
        std::fprintf(stderr, "%s\n", err.c_str());
        return 1;
    }
    std::printf("replay %s (%u x %ux%u) + %zu IMU samples through %s%s%s\n  lead %.1f ms, imu offset %lld ns, cam offset %lld ns, out %s (%llu reference rows)\n",
                take_path.c_str(), take->tiles, take->w, take->h, imu.imu.size(), vit.c_str(),
                config.empty() ? "" : ", config ", config.c_str(), static_cast<double>(lead) / 1e6, static_cast<long long>(imu_off),
                static_cast<long long>(cam_off), out.c_str(), static_cast<unsigned long long>(ref_rows));

    const std::int64_t t_start = now_ns();
    std::size_t ii = 0;
    std::uint64_t frames = 0, pushed_imu = 0, poses = 0, resets = 0, last_logged = 0;
    std::int64_t last_pose_ts = 0, first_ts = 0, last_ts = 0, last_reset_ts = 0;
    float prev_p[3] = {0, 0, 0};
    bool have_prev = false;
    ShadowTakeFrame f;
    auto drain = [&](std::int64_t now_take) {
        for (const EnginePose& e : host->poses()) {
            poses++;
            last_pose_ts = e.ts_ns;
            if (e.ts_ns - static_cast<std::int64_t>(last_logged) >= 5'000'000 || last_logged == 0) { // as the live log
                last_logged = static_cast<std::uint64_t>(e.ts_ns);
                std::fprintf(o, "%lld,1,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,\n",
                             static_cast<long long>(e.ts_ns), e.p[0], e.p[1], e.p[2], e.q[0], e.q[1], e.q[2], e.q[3],
                             e.v[0], e.v[1], e.v[2]);
            }
            const double v = std::sqrt(e.v[0] * e.v[0] + e.v[1] * e.v[1] + e.v[2] * e.v[2]);
            const double r = std::sqrt(e.p[0] * e.p[0] + e.p[1] * e.p[1] + e.p[2] * e.p[2]);
            const double dx = e.p[0] - prev_p[0], dy = e.p[1] - prev_p[1], dz = e.p[2] - prev_p[2];
            const bool diverged = v > 3.0 || r > 50.0 || (have_prev && std::sqrt(dx * dx + dy * dy + dz * dz) > 0.5);
            std::copy(e.p, e.p + 3, prev_p);
            have_prev = true;
            if (diverged && !no_reset && now_take - last_reset_ts > 1'000'000'000) {
                host->reset();
                resets++;
                last_reset_ts = now_take;
                have_prev = false;
                std::fprintf(o, "%lld,1,,,,,,,,,,,diverged\n", static_cast<long long>(now_take));
                std::printf("  %.1f s: diverged, started over\n", static_cast<double>(now_take - first_ts) / 1e9);
            }
        }
    };
    while (take->next(f, &err)) {
        const std::int64_t fts = f.ts_ns + cam_off;
        if (first_ts == 0) {
            first_ts = fts;
        }
        last_ts = fts;
        // the IMU up to the frame's arrival (as live: the samples that had come when the frame set was pushed)
        while (ii < imu.imu.size() && imu.imu[ii].ts + imu_off <= fts + lead) {
            const auto& s = imu.imu[ii++];
            const float ac[3] = {static_cast<float>(s.accel[0]), static_cast<float>(s.accel[1]), static_cast<float>(s.accel[2])};
            const float gy[3] = {static_cast<float>(s.gyro[0]), static_cast<float>(s.gyro[1]), static_cast<float>(s.gyro[2])};
            host->push_imu(s.ts + imu_off, ac, gy);
            pushed_imu++;
        }
        std::vector<const std::uint8_t*> tiles(take->tiles);
        for (std::uint32_t t = 0; t < take->tiles; t++) {
            tiles[t] = f.data.data() + static_cast<std::size_t>(t) * take->w * take->h;
        }
        host->push_frames(fts, tiles, take->w, take->h);
        frames++;
        drain(fts);
        // do not run away from the engine: its queues are bounded (the IMU one blocks the pusher) and a replay that
        // floods the frame queue measures memory, not tracking
        const std::int64_t wait_start = now_ns();
        while (poses > 0 && last_pose_ts < fts - max_lag && now_ns() - wait_start < 2'000'000'000) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            drain(fts);
        }
        if (frames % 1500 == 0) {
            std::printf("  %.0f s of the take, %llu poses, %llu resets, %.1fx real time\n", static_cast<double>(fts - first_ts) / 1e9,
                        static_cast<unsigned long long>(poses), static_cast<unsigned long long>(resets),
                        static_cast<double>(fts - first_ts) / static_cast<double>(std::max<std::int64_t>(now_ns() - t_start, 1)));
            std::fflush(stdout);
        }
    }
    if (!err.empty()) {
        std::fprintf(stderr, "take: %s after %llu frame sets\n", err.c_str(), static_cast<unsigned long long>(frames));
    }
    // the rest of the IMU within the last frame's reach, and the engine's last poses
    while (ii < imu.imu.size() && imu.imu[ii].ts + imu_off <= last_ts + lead + 100'000'000) {
        const auto& s = imu.imu[ii++];
        const float ac[3] = {static_cast<float>(s.accel[0]), static_cast<float>(s.accel[1]), static_cast<float>(s.accel[2])};
        const float gy[3] = {static_cast<float>(s.gyro[0]), static_cast<float>(s.gyro[1]), static_cast<float>(s.gyro[2])};
        host->push_imu(s.ts + imu_off, ac, gy);
        pushed_imu++;
    }
    const std::int64_t end_wait = now_ns();
    while (now_ns() - end_wait < 1'500'000'000) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        drain(last_ts);
    }
    std::fclose(o);
    const double secs = static_cast<double>(now_ns() - t_start) / 1e9;
    std::printf("done: %llu frame sets (%.1f s of the take%s), %llu IMU samples pushed (%llu held by the gate), %llu poses, %llu resets, %.0f s (%.1fx real time)\n",
                static_cast<unsigned long long>(frames), static_cast<double>(last_ts - first_ts) / 1e9, take->truncated ? ", truncated tail" : "",
                static_cast<unsigned long long>(pushed_imu), static_cast<unsigned long long>(host->imu_held()),
                static_cast<unsigned long long>(poses), static_cast<unsigned long long>(resets), secs,
                static_cast<double>(last_ts - first_ts) / 1e9 / std::max(secs, 1e-3));
    host.reset();
    return 0;
}
