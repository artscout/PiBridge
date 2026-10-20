// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-tools/src/follow.rs (MIT OR Apache-2.0).
//
// `--follow PROCESS`: the headset is busy only while a VR runtime runs (SteamVR's `vrserver`), as Pimax Play keeps it
// busy only while it runs. Follow says when to wake it and when to let it sleep: active as soon as the process is
// there, idle once it has been gone for IDLE_AFTER (SteamVR restarts its server on its own).
#pragma once

#include <chrono>
#include <filesystem>
#include <optional>
#include <string_view>

namespace pibridge::hmd {

/// How long the process must be gone before the headset goes idle.
inline constexpr std::chrono::seconds IDLE_AFTER{5};

/// Whether a process with this name (/proc/<pid>/comm, at most 15 characters) runs.
bool process_running(std::string_view name);
/// process_running over another proc root (tests).
bool process_running_in(const std::filesystem::path& proc_root, std::string_view name);

/// The debounced state.
class Follow {
public:
    enum class Change {
        /// The runtime started: wake the headset.
        Active,
        /// The runtime has been gone for IDLE_AFTER: let it sleep.
        Idle,
    };
    using Clock = std::chrono::steady_clock;

    /// Start in the state the process is in now (no change reported for it).
    explicit Follow(bool running) : active_(running) {}
    /// Whether the headset should be busy now.
    bool active() const { return active_; }
    /// Feed one observation; returns the change it causes.
    std::optional<Change> step(Clock::time_point now, bool running);

private:
    bool active_;
    std::optional<Clock::time_point> gone_since_;
};

} // namespace pibridge::hmd
