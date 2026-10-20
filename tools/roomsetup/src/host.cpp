// SPDX-License-Identifier: MIT
#include "host.h"

#include "pibridge/room/files.h"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <format>
#include <poll.h>
#include <spawn.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

extern char** environ;

namespace pibridge::roomsetup {

HelperResult runSelf(const std::vector<std::string>& args, int timeoutMs) {
    int fds[2];
    if (::pipe(fds) != 0) {
        return {};
    }
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_adddup2(&fa, fds[1], 1);
    posix_spawn_file_actions_addclose(&fa, fds[0]);
    std::vector<std::string> argv{"pibridge-roomsetup"};
    argv.insert(argv.end(), args.begin(), args.end());
    std::vector<char*> cargv;
    for (auto& a : argv) {
        cargv.push_back(a.data());
    }
    cargv.push_back(nullptr);
    pid_t pid = 0;
    const int rc = posix_spawn(&pid, "/proc/self/exe", &fa, nullptr, cargv.data(), environ);
    posix_spawn_file_actions_destroy(&fa);
    ::close(fds[1]);
    if (rc != 0) {
        ::close(fds[0]);
        return {};
    }
    HelperResult r;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    char buf[256];
    bool open = true;
    while (open) {
        const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
        if (left.count() <= 0) {
            break;
        }
        pollfd p{fds[0], POLLIN, 0};
        if (::poll(&p, 1, static_cast<int>(left.count())) <= 0) {
            break;
        }
        const ssize_t n = ::read(fds[0], buf, sizeof buf);
        if (n <= 0) {
            open = false;
        } else {
            r.out.append(buf, static_cast<std::size_t>(n));
        }
    }
    ::close(fds[0]);
    int status = 0;
    if (open) { // timed out: stop our own child, by its pid
        ::kill(pid, SIGKILL);
        ::waitpid(pid, &status, 0);
        return {-1, r.out};
    }
    for (int i = 0; i < 50; i++) { // it closed its output: it ends now
        if (::waitpid(pid, &status, WNOHANG) == pid) {
            r.code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
            return r;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ::kill(pid, SIGKILL);
    ::waitpid(pid, &status, 0);
    return {-1, r.out};
}

std::string currentUniverse(const HelperRunner& run) {
    const HelperResult r = run({"--vr-universe"});
    if (r.code != 0) {
        return {};
    }
    std::string s = r.out;
    while (!s.empty() && (s.back() == '\n' || s.back() == ' ')) {
        s.pop_back();
    }
    for (const char c : s) {
        if (c < '0' || c > '9') {
            return {};
        }
    }
    return s;
}

std::string reloadAfterSave(const HelperRunner& run, bool steamvr) {
    if (!steamvr) {
        return "The runtime reads it at its next start";
    }
    const HelperResult r = run({"--vr-reload"});
    return r.code == 0 ? "SteamVR reloaded it" : "Restart SteamVR to see it";
}

ToolZoneControl::ToolZoneControl(std::string settingsFile, std::string stateDir, bool steamvr, HelperRunner run,
                                 std::function<std::time_t()> clock)
    : file_(std::move(settingsFile), stateDir, std::move(clock)), stateDir_(std::move(stateDir)), steamvr_(steamvr),
      run_(std::move(run)) {}

room::ZoneValues ToolZoneControl::values() { return live_ ? *live_ : file_.values(); }

std::expected<double, std::string> ToolZoneControl::setWarnDistance(std::optional<double> metres) {
    if (!steamvr_) {
        return file_.setWarnDistance(metres);
    }
    room::ZoneValues v = values();
    std::string arg = "fade=default";
    if (metres) {
        const double m = room::clampWarnDistance(*metres);
        arg = std::format("fade={}", m);
        v.fade = m;
    } else {
        v.fade.reset();
    }
    if (run_({"--vr-zone", arg}).code != 0) {
        return std::unexpected(std::string("SteamVR did not take the setting"));
    }
    live_ = v;
    return v.warnDistance();
}

std::expected<void, std::string> ToolZoneControl::setZoneOn(bool on) {
    if (!steamvr_) {
        return file_.setZoneOn(on);
    }
    room::ZoneValues v = values();
    std::string arg;
    if (!on) {
        if (v.alpha && *v.alpha > 0) {
            std::error_code ec;
            std::filesystem::create_directories(stateDir_, ec);
            (void)room::writeFileAtomic(stateDir_ + "/zone-alpha.txt", std::to_string(*v.alpha));
        }
        arg = "alpha=0";
        v.alpha = 0;
    } else {
        std::optional<int> restore;
        if (auto t = room::readFile(stateDir_ + "/zone-alpha.txt")) {
            int n = 0;
            if (std::sscanf(t->c_str(), "%d", &n) == 1 && n > 0) {
                restore = n;
            }
        }
        arg = restore ? std::format("alpha={}", *restore) : std::string("alpha=default");
        v.alpha = restore;
    }
    if (run_({"--vr-zone", arg}).code != 0) {
        return std::unexpected(std::string("SteamVR did not take the setting"));
    }
    live_ = v;
    return {};
}

} // namespace pibridge::roomsetup
