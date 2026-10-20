// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-tools/src/keepalived.rs (MIT OR Apache-2.0).
// pibridge-keepalived: signals and the clock around keepalived::Daemon. SIGTERM/SIGINT stop it cleanly (idle report,
// audio back); SIGHUP re-reads pibridge.conf (the command line still overrides it).
#include "keepalived.h"

#include "pibridge_pause.h"

#include <cerrno>
#include <csignal>
#include <cstdio>

#include <poll.h>
#include <sys/signalfd.h>
#include <unistd.h>

using namespace pibridge;
using namespace pibridge::hmd;

int main(int argc, char** argv) {
    const std::vector<std::string> args(argv + 1, argv + argc);
    const auto settings_path = keepalived::settings_path_of(args);
    Settings settings = Settings::load_or_legacy(settings_path);
    auto opts = keepalived::parse_args(args, settings);
    if (!opts) {
        if (opts.error() == "help") {
            std::fputs(keepalived::usage().c_str(), stdout);
            return 0;
        }
        std::fprintf(stderr, "pibridge-keepalived: %s\n%s", opts.error().c_str(), keepalived::usage().c_str());
        return 2;
    }

    // block the signals before the session thread starts (it inherits the mask); they arrive on a signalfd
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGTERM);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGHUP);
    pthread_sigmask(SIG_BLOCK, &set, nullptr);
    Fd sfd(::signalfd(-1, &set, SFD_NONBLOCK | SFD_CLOEXEC));
    if (!sfd) {
        std::perror("pibridge-keepalived: signalfd");
        return 1;
    }

    keepalived::Daemon d(*opts, keepalived::Deps{});
    for (const auto& w : settings.warnings) {
        std::printf("%s %s: %s\n", local_hms().c_str(), settings_path.c_str(), w.c_str());
    }
    // an inside-out session owns the headset (common/pibridge_pause.h): wait without opening it
    if (pibridge::pause::active_marker()) {
        d.pause(keepalived::Clock::now());
    } else if (auto r = d.start(std::make_unique<HidrawBackend>(), keepalived::Clock::now()); !r) {
        std::fprintf(stderr, "pibridge-keepalived: %s\n", r.error().c_str());
        return 1;
    }
    bool run = true;
    while (run) {
        std::array<pollfd, 2> pf{pollin(sfd.get()), pollin(d.wake_fd())};
        (void)ppoll(pf, std::chrono::milliseconds(500));
        if (pf[0].revents & POLLIN) {
            signalfd_siginfo si{};
            while (::read(sfd.get(), &si, sizeof si) == static_cast<ssize_t>(sizeof si)) {
                if (si.ssi_signo == SIGHUP) {
                    settings = Settings::load_or_legacy(settings_path);
                    for (const auto& w : settings.warnings) {
                        std::printf("%s %s: %s\n", local_hms().c_str(), settings_path.c_str(), w.c_str());
                    }
                    if (auto o = keepalived::parse_args(args, settings)) {
                        d.reload(*o, keepalived::Clock::now());
                    } else {
                        std::printf("%s settings not applied: %s\n", local_hms().c_str(), o.error().c_str());
                    }
                } else {
                    run = false;
                }
            }
        }
        if (run) {
            const auto now = keepalived::Clock::now();
            const bool marker = pibridge::pause::active_marker().has_value();
            if (marker && !d.paused()) {
                d.pause(now);
            } else if (!marker && d.paused()) {
                if (auto r = d.resume(std::make_unique<HidrawBackend>(), now); !r) {
                    std::fprintf(stderr, "pibridge-keepalived: %s\n", r.error().c_str());
                    return 1;
                }
            }
            d.tick(now);
        }
    }
    d.stop();
    return 0;
}
