// SPDX-License-Identifier: MIT
//
// pibridge-posetap: a read-only probe on the pose shared memory (pibridge/hmd/pose_shm.h,
// /dev/shm/pibridge-crystal-pose). It samples the file fast and reports, for the HEAD device, how the pose actually
// reaches the SteamVR driver:
//
//   - update rate   : how often devices[0] changes (its `sample` number) -> the true output rate CHOVR forwards.
//                     Basalt's output layer runs at the loop rate (~1 kHz); the raw Pimax-engine passthrough changes
//                     only on a new SLAM frame (~camera rate). That difference is what this tool is for.
//   - age           : written_ns - t_ns, how old the sample was when headtrack published it (pipeline latency carried
//                     into SteamVR's prediction via poseTimeOffset).
//   - gap / jitter  : host time between successive head updates (mean, stddev, max).
//   - d_ori / d_pos : the step the orientation and position take between successive updates -- large steps at a low
//                     rate are the "snaps" SteamVR has to bridge by constant-velocity prediction (the suspected judder).
//   - wvel          : angular-velocity magnitude, to correlate big steps with fast head turns.
//
// Nothing is written and no behaviour changes: safe to run against a live session.
//
//   pibridge-posetap [--file PATH] [--secs N] [--hz POLL] [--raw]
//
// --file  the pose file (default: pose_shm::default_path()).
// --secs  stop after N seconds (default: run until Ctrl-C).
// --hz    poll rate (default 2000).
// --raw   also print one line per head update (t_ns, age, step), not only the per-second summary.

#include "pibridge/hmd/pose_shm.h"

#include <array>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <optional>
#include <string>

#include <fcntl.h>
#include <unistd.h>

namespace ps = pibridge::hmd::pose_shm;

namespace {

std::atomic<bool> g_run{true};
void on_sigint(int) { g_run.store(false); }

std::int64_t mono_ns() {
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return std::int64_t{ts.tv_sec} * 1'000'000'000 + ts.tv_nsec;
}

double norm3(const std::array<float, 3>& v) {
    return std::sqrt(double(v[0]) * v[0] + double(v[1]) * v[1] + double(v[2]) * v[2]);
}

// angle between two unit quaternions, degrees (sign-insensitive: the short way round)
double quat_step_deg(const std::array<float, 4>& a, const std::array<float, 4>& b) {
    double dot = double(a[0]) * b[0] + double(a[1]) * b[1] + double(a[2]) * b[2] + double(a[3]) * b[3];
    dot = std::fabs(dot);
    if (dot > 1.0) {
        dot = 1.0;
    }
    return 2.0 * std::acos(dot) * 180.0 / M_PI;
}

double dist_mm(const std::array<float, 3>& a, const std::array<float, 3>& b) {
    const double dx = double(a[0]) - b[0], dy = double(a[1]) - b[1], dz = double(a[2]) - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz) * 1000.0;
}

// running min / mean / max / variance (Welford) over a window
struct Stat {
    long n = 0;
    double sum = 0, min = 0, max = 0, m2 = 0, mean = 0;
    void add(double x) {
        if (n == 0) {
            min = max = x;
        } else {
            min = std::fmin(min, x);
            max = std::fmax(max, x);
        }
        ++n;
        const double d = x - mean;
        mean += d / double(n);
        m2 += d * (x - mean);
        sum += x;
    }
    double sd() const { return n > 1 ? std::sqrt(m2 / double(n - 1)) : 0.0; }
    void reset() { *this = Stat{}; }
};

const char* track_word(std::uint32_t t) {
    switch (t) {
    case 0: return "none";
    case 1: return "3dof";
    case 3: return "6dof";
    default: return "?";
    }
}

} // namespace

int main(int argc, char** argv) {
    std::string file;
    double secs = 0;     // 0 = until Ctrl-C
    double poll_hz = 2000;
    bool raw = false;
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        if (a == "--file" && i + 1 < argc) {
            file = argv[++i];
        } else if (a == "--secs" && i + 1 < argc) {
            secs = std::strtod(argv[++i], nullptr);
        } else if (a == "--hz" && i + 1 < argc) {
            poll_hz = std::strtod(argv[++i], nullptr);
        } else if (a == "--raw") {
            raw = true;
        } else {
            std::fprintf(stderr, "usage: pibridge-posetap [--file PATH] [--secs N] [--hz POLL] [--raw]\n");
            return 2;
        }
    }
    if (file.empty()) {
        file = ps::default_path().string();
    }
    if (poll_hz < 1) {
        poll_hz = 1;
    }

    const int fd = ::open(file.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        std::fprintf(stderr, "pibridge-posetap: cannot open %s: %s\n", file.c_str(), std::strerror(errno));
        return 1;
    }
    std::signal(SIGINT, on_sigint);
    std::signal(SIGTERM, on_sigint);
    std::fprintf(stderr, "pibridge-posetap: %s, poll %.0f Hz%s%s\n", file.c_str(), poll_hz,
                 secs > 0 ? (", " + std::to_string(secs) + " s").c_str() : ", until Ctrl-C", raw ? ", raw" : "");

    const std::int64_t poll_ns = std::int64_t(1e9 / poll_hz);
    const std::int64_t start = mono_ns();
    std::int64_t window_start = start;

    std::optional<ps::State> prev;      // the last head update we counted (by sample)
    std::int64_t prev_update_host = 0;  // host time of that update

    // per-second window and whole-run totals
    Stat w_age, w_gap, w_ori, w_pos, w_wvel, r_age, r_gap, r_ori, r_pos, r_wvel;
    long w_updates = 0, r_updates = 0;
    long w_polls = 0, r_polls = 0, torn = 0;
    std::uint32_t last_track = 0xffffffff, last_state = 0xffffffff;

    auto flush = [&](std::int64_t now, bool final) {
        const double dt = double(now - window_start) / 1e9;
        const double rate = dt > 0 ? double(w_updates) / dt : 0;
        std::printf("t=%5.1f  %-4s/st%u  upd %6.1f Hz (%ld)  age %.0f/%.0f/%.0f ms  gap %.1f±%.1f (max %.0f) ms  "
                    "d_ori %.2f/%.2f deg  d_pos %.1f/%.1f mm  wvel %.0f deg/s  polls %ld torn %ld\n",
                    double(now - start) / 1e9, track_word(last_track), last_state, rate, w_updates,
                    w_age.min, w_age.mean, w_age.max, w_gap.mean, w_gap.sd(), w_gap.max, w_ori.mean, w_ori.max,
                    w_pos.mean, w_pos.max, w_wvel.mean, w_polls, torn);
        std::fflush(stdout);
        if (final) {
            return;
        }
        window_start = now;
        w_updates = 0;
        w_polls = 0;
        torn = 0;
        w_age.reset();
        w_gap.reset();
        w_ori.reset();
        w_pos.reset();
        w_wvel.reset();
    };

    while (g_run.load()) {
        const std::int64_t now = mono_ns();
        if (secs > 0 && double(now - start) / 1e9 >= secs) {
            break;
        }

        std::array<std::uint8_t, ps::SIZE> buf{};
        const ssize_t got = ::pread(fd, buf.data(), buf.size(), 0);
        ++w_polls;
        ++r_polls;
        if (got == ssize_t(buf.size())) {
            if (auto st = ps::decode(buf)) {
                const ps::Device& h = st->devices[0];
                const bool fresh = !prev || prev->devices[0].sample != h.sample;
                if (fresh && h.state != 0) {
                    const double age = double(st->written_ns - h.t_ns) / 1e6; // ms
                    const double wvel = norm3(h.angular_velocity) * 180.0 / M_PI;
                    double gap = 0, dori = 0, dpos = 0;
                    if (prev && prev->devices[0].state != 0) {
                        gap = double(now - prev_update_host) / 1e6; // ms, host-observed interval
                        dori = quat_step_deg(prev->devices[0].orientation, h.orientation);
                        dpos = dist_mm(prev->devices[0].position, h.position);
                    }
                    w_age.add(age);
                    r_age.add(age);
                    w_wvel.add(wvel);
                    r_wvel.add(wvel);
                    if (prev && prev->devices[0].state != 0) {
                        w_gap.add(gap);
                        r_gap.add(gap);
                        w_ori.add(dori);
                        r_ori.add(dori);
                        w_pos.add(dpos);
                        r_pos.add(dpos);
                    }
                    ++w_updates;
                    ++r_updates;
                    if (raw) {
                        std::printf("  upd t_ns=%lld age=%.1f ms gap=%.1f ms d_ori=%.3f deg d_pos=%.2f mm wvel=%.0f\n",
                                    (long long)h.t_ns, age, gap, dori, dpos, wvel);
                    }
                    prev = st;
                    prev_update_host = now;
                }
                last_track = st->tracking;
                last_state = h.state;
            } else {
                ++torn;
            }
        }

        if (now - window_start >= 1'000'000'000) {
            flush(now, false);
        }

        timespec req{};
        req.tv_sec = poll_ns / 1'000'000'000;
        req.tv_nsec = poll_ns % 1'000'000'000;
        ::nanosleep(&req, nullptr);
    }

    const std::int64_t end = mono_ns();
    flush(end, true);
    const double run = double(end - start) / 1e9;
    std::printf("---- summary over %.1f s ----\n", run);
    std::printf("head updates : %ld  (%.1f Hz)\n", r_updates, run > 0 ? double(r_updates) / run : 0);
    std::printf("pose age     : %.0f / %.0f / %.0f ms (min/mean/max)\n", r_age.min, r_age.mean, r_age.max);
    std::printf("update gap   : %.1f ms mean, %.1f ms sd, %.0f ms max\n", r_gap.mean, r_gap.sd(), r_gap.max);
    std::printf("ori step     : %.3f deg mean, %.2f deg max\n", r_ori.mean, r_ori.max);
    std::printf("pos step     : %.2f mm mean, %.1f mm max\n", r_pos.mean, r_pos.max);
    std::printf("ang velocity : %.0f deg/s mean, %.0f deg/s max\n", r_wvel.mean, r_wvel.max);
    std::printf("polls        : %ld\n", r_polls);
    ::close(fd);
    return 0;
}
