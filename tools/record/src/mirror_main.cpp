// SPDX-License-Identifier: MIT
// pibridge-record-mirror RING: the SteamVR source of pibridge-record (include/pibridge/record/mirror.h). Started by
// `pibridge-record run --source mirror` after it created RING; it plays the part of Monado's tap on that ring.
// Exit 0: a clean end (asked, SteamVR quit, the recorder gone); 3: could not start (the last line says why); 4: an
// error while recording.
#include "pibridge/record/mirror.h"
#include "pibridge/record/proc.h"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <thread>

#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

namespace {
std::atomic<bool> g_stop{false};
extern "C" void onSignal(int) { g_stop.store(true); }
} // namespace

int main(int argc, char** argv) {
    using namespace pibridge::record;
    if (argc != 2) {
        std::fprintf(stderr, "usage: pibridge-record-mirror RING (started by pibridge-record run --source mirror)\n");
        return 2;
    }
    setpriority(PRIO_PROCESS, 0, 10);
    syscall(SYS_ioprio_set, 1, 0, (2 << 13) | 7);
    struct sigaction sa{};
    sa.sa_handler = onSignal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    signal(SIGPIPE, SIG_IGN);

    const std::filesystem::path ringPath = argv[1];
    std::optional<Ring> ring;
    const auto t0 = std::chrono::steady_clock::now();
    while (!ring) {
        if (auto r = Ring::open(ringPath)) {
            ring.emplace(std::move(*r));
        } else if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(5) || g_stop.load()) {
            std::fprintf(stderr, "pibridge-record-mirror: the recorder's ring %s is not there (%s)\n", ringPath.c_str(),
                         r.error().c_str());
            return 3;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
    const std::filesystem::path proc = procRoot();
    if (!anyWithComm(proc, "vrserver")) {
        std::fprintf(stderr, "pibridge-record-mirror: SteamVR is not running\n");
        return 3;
    }
    auto backend = makeOpenVrMirror(openVrLibrary());
    MirrorOptions opt;
    opt.steamvrAlive = [&] { return anyWithComm(proc, "vrserver"); };
    const MirrorResult r = runMirror(*ring, *backend, g_stop, opt);
    backend.reset(); // the textures, the GL context and the OpenVR connection go before the exit
    std::fprintf(stderr, "pibridge-record-mirror: %s\n", r.reason.c_str());
    return r.code;
}
