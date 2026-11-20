// SPDX-License-Identifier: MIT
// The controller shadow mode's system log: once a second, the scheduler statistics of the threads that track (Pimax's
// LED engine under Wine, pibridge-ctrltrack, headtrack, the camera broker, SteamVR) and the machine's load, pressure,
// temperatures and clocks, so a slowdown of the engine can be told apart as "computes more" or "waits more".
#pragma once

#include <atomic>
#include <cstdio>
#include <string>
#include <thread>

namespace pibridge::headtrack {

/// `<path>`: a CSV `t_ns,kind,proc,pid,tid,thread,cpu_ms,wait_ms,slices,value`, one row per busy thread per second
/// (kind `thread`: CPU time, run-queue wait and timeslices in that second, from `/proc/PID/task/TID/schedstat`) and
/// one per machine reading (kind `load`, `psi_cpu`, `temp`, `freq_mean`, `freq_max`, `gpu_busy`: `value`) and one per
/// watched process (kind `rss`: resident memory in MB).
class SysMon {
public:
    SysMon() = default;
    SysMon(const SysMon&) = delete;
    SysMon& operator=(const SysMon&) = delete;
    ~SysMon() { stop(); }

    /// Opens the file and starts the thread; false when the file cannot be written.
    bool start(const std::string& path);
    void stop();
    std::uint64_t rows() const { return rows_; }

private:
    void loop();
    std::FILE* f_ = nullptr;
    std::thread th_;
    std::atomic<bool> run_{false};
    std::uint64_t rows_ = 0;
};

} // namespace pibridge::headtrack
