// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-tools/src/eyed.rs (MIT OR Apache-2.0).
// pibridge-eyed: signals and the clock around eyed::Daemon. SIGTERM/SIGINT stop it cleanly (unsubscribe, "no gaze",
// status file removed); SIGHUP re-reads pibridge.conf (the command line still overrides it).
#include "eyed.h"

#include "pibridge_pause.h"

#include "pibridge/hmd/sys.h"

#include <array>
#include <csignal>
#include <cstdio>

#include <poll.h>
#include <sys/signalfd.h>
#include <unistd.h>

using namespace pibridge;
using namespace pibridge::hmd;

namespace {

void warn_settings(const Settings& s, const std::filesystem::path& p) {
    for (const auto& w : s.warnings) {
        std::printf("%s %s: %s\n", local_hms().c_str(), p.c_str(), w.c_str());
    }
    std::fflush(stdout);
}

} // namespace

int main(int argc, char** argv) {
    const std::vector<std::string> args(argv + 1, argv + argc);
    const auto settings_path = eyed::settings_path_of(args);
    Settings settings = Settings::load_or_legacy(settings_path);
    auto opts = eyed::parse_args(args, settings, eyed::ConfigDirs::from_env());
    if (!opts) {
        if (opts.error() == "help") {
            std::fputs(eyed::usage().c_str(), stdout);
            return 0;
        }
        std::fprintf(stderr, "pibridge-eyed: %s\n%s", opts.error().c_str(), eyed::usage().c_str());
        return 2;
    }

    // block the signals before the tracker thread starts (it inherits the mask); they arrive on a signalfd
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGTERM);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGHUP);
    pthread_sigmask(SIG_BLOCK, &set, nullptr);
    Fd sfd(::signalfd(-1, &set, SFD_NONBLOCK | SFD_CLOEXEC));
    if (!sfd) {
        std::perror("pibridge-eyed: signalfd");
        return 1;
    }

    eyed::Daemon d(*opts, eyed::Deps{});
    warn_settings(settings, settings_path);
    // an inside-out session owns the eye chip (common/pibridge_pause.h): start paused, so the chip is never opened
    if (pibridge::pause::active_marker()) {
        d.set_paused(true, eyed::Clock::now());
    }
    if (auto r = d.start(eyed::Clock::now()); !r) {
        std::fprintf(stderr, "pibridge-eyed: %s\n", r.error().c_str());
        return 1;
    }
    bool run = true;
    auto last_marker = eyed::Clock::time_point{};
    while (run) {
        // the Rust loop polls every 2 ms: a new sample reaches the gaze file within that
        std::array<pollfd, 1> pf{pollin(sfd.get())};
        (void)ppoll(pf, std::chrono::milliseconds(2));
        if (pf[0].revents & POLLIN) {
            signalfd_siginfo si{};
            while (::read(sfd.get(), &si, sizeof si) == static_cast<ssize_t>(sizeof si)) {
                if (si.ssi_signo == SIGHUP) {
                    settings = Settings::load_or_legacy(settings_path);
                    warn_settings(settings, settings_path);
                    if (auto o = eyed::parse_args(args, settings, eyed::ConfigDirs::from_env())) {
                        d.reload(*o, eyed::Clock::now());
                    } else {
                        std::printf("%s settings not applied: %s\n", local_hms().c_str(), o.error().c_str());
                    }
                } else {
                    run = false;
                }
            }
        }
        if (run) {
            const auto now = eyed::Clock::now();
            // the marker is checked twice a second (the loop runs every 2 ms)
            if (now - last_marker >= std::chrono::milliseconds(500)) {
                last_marker = now;
                d.set_paused(pibridge::pause::active_marker().has_value(), now);
            }
            d.tick(now);
        }
    }
    d.stop();
    return 0;
}
