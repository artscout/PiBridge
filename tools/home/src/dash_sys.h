// SPDX-License-Identifier: MIT
// The dashboard's real sinks (dash_settings.h Sinks): files written atomically, the headset volume through pactl on a
// thread of its own (a pactl call takes milliseconds: never on the frame loop), and the room setup started detached
// (its own session, so a stop of the home does not take it down; its output goes to the home's log).
#pragma once

#include "dash_settings.h"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace pibridge::home::dash {

/// pibridge.conf (bg::confPath()), $XDG_RUNTIME_DIR (or /tmp), $XDG_RUNTIME_DIR/pibridge, and pibridge-roomsetup: next
/// to this program, then the build tree's tools/roomsetup, then PATH ("" when none).
Paths defaultPaths();

std::optional<std::string> readFile(const std::filesystem::path& p);
/// Temporary file + rename; the folder is made when missing.
bool writeFileAtomic(const std::filesystem::path& p, const std::string& text);
/// posix_spawn with POSIX_SPAWN_SETSID, the process's environment plus `env` ("KEY=VALUE", replacing a KEY there).
std::optional<long> spawnDetached(const std::vector<std::string>& argv, const std::vector<std::string>& env);
/// The exit code once `pid` (a child of ours) has ended; nullopt while it runs. -1 for a signal.
std::optional<int> reap(long pid);
/// `program --help`'s output (stdout and stderr), at most 3 s.
std::optional<std::string> usageOf(const std::string& program);

/// The headset volume, read every 2 s and set on request, on its own thread.
class VolumeWorker {
public:
    VolumeWorker();
    ~VolumeWorker();
    VolumeWorker(const VolumeWorker&) = delete;
    VolumeWorker& operator=(const VolumeWorker&) = delete;
    std::optional<int> current() const;
    /// Asks for `percent` (the thread sets it); false while no headset output is known.
    bool set(int percent);

private:
    void run();
    mutable std::mutex mu_;
    std::condition_variable cv_;
    std::optional<int> current_;
    std::optional<int> target_;
    bool stop_ = false;
    std::thread th_;
};

/// The Sinks over the functions above and `volume`.
Sinks realSinks(VolumeWorker& volume);

} // namespace pibridge::home::dash
