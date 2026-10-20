// SPDX-License-Identifier: MIT
// Ported from the frame loop of omarchyvr pimax-eyecal/src/xrapp.rs (MIT OR Apache-2.0), without OpenXR: the raw
// tracker, the procedure, its effects (files, chip requests, the tracker restart) and the clean end. xrapp.cpp calls
// step() once per rendered frame; the tests drive it with FakeChip trackers.
#pragma once

#include "chip.h"
#include "collect.h"
#include "procedure.h"

#include "pibridge/eye/calibration.h"
#include "pibridge/eye/live.h"

#include <array>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::eyecal {

/// Where the run's files go.
struct RunFiles {
    /// Results JSON and raw CSV folder (report::results_dir()).
    std::filesystem::path results_dir;
    /// `YYYYMMDD-HHMMSS` of the run (file names, backups).
    std::string stamp;
    /// The host map (eye-calibration.toml).
    std::filesystem::path calibration;
    /// The eye profile eyed uploads (the chip's new profile replaces it).
    std::filesystem::path profile;
    /// Write every raw sample to `eyecal-<stamp>-raw.csv`.
    bool raw_log = true;
};

/// What a terminal key means: a command, nullopt inside for quit, nullopt outside for nothing.
std::optional<std::optional<Cmd>> key_command(char c);

/// The forward direction (0, 0, -1) rotated by the quaternion (x, y, z, w).
Vec3 forward_of(const std::array<double, 4>& q);

/// The raw CSV: header and one row.
std::string raw_csv_header();
std::string raw_csv_row(const RawSample& s, std::string_view tag, const std::optional<std::array<double, 4>>& head);

class Runner {
public:
    using StartTracker = std::function<std::unique_ptr<pibridge::eye::live::Tracker>(const pibridge::eyemath::FrameConv&)>;
    using Print = std::function<void(const std::string&)>;
    /// Start (true) or end (false) the end-to-end check through the runtime; false when it cannot.
    using Service = std::function<bool(bool start)>;

    /// `link` is the calibration link the tracker's chip channels attach to (ChannelDevice).
    Runner(Procedure proc, RunFiles files, std::shared_ptr<pibridge::eye::calibration::Link> link, StartTracker start,
           Print print, Service service = {});
    ~Runner();
    Runner(const Runner&) = delete;
    /// Start the raw tracker (after the OpenXR session exists, so a runtime that fails to start leaves the chip
    /// alone).
    void begin();
    /// The end-to-end check hook (set by the OpenXR app once its gaze action exists).
    void set_service(Service s) { service_ = std::move(s); }
    Runner& operator=(const Runner&) = delete;

    /// One frame: the new samples, the head pose, the eye positions, the runtime's gaze (end-to-end check), the
    /// wearer's commands.
    void step(double now, const std::optional<std::array<double, 4>>& head, const std::array<Vec3, 2>& eye_pos,
              const std::optional<Vec3>& service, const std::vector<Cmd>& cmds);
    /// The procedure asked to quit.
    bool quit_requested() const { return quit_; }
    /// The end: finish an open chip pass (and wait for its answer, at most 1.5 s), stop the tracker, put the previous
    /// profile back unless the new one was kept, flush the raw log. Called by the destructor when not called before.
    void finish();

    Procedure& procedure() { return proc_; }
    const Procedure& procedure() const { return proc_; }
    /// The tracker's state ("Streaming", ...), or "Stopped" during the end-to-end check.
    std::string tracker_state() const;
    /// Head-locked lines to show above the procedure's (the tracker when it does not stream).
    std::vector<std::string> hud_prefix() const;
    const ProfileStore& profile_store() const { return store_; }

private:
    void handle(std::vector<Effect> fx);

    Procedure proc_;
    RunFiles files_;
    std::shared_ptr<pibridge::eye::calibration::Link> link_;
    StartTracker start_;
    Print print_;
    Service service_;
    std::unique_ptr<pibridge::eye::live::Tracker> tracker_;
    ChipDriver driver_;
    ProfileStore store_;
    std::optional<std::ofstream> raw_;
    std::optional<std::uint64_t> last_seq_;
    std::optional<pibridge::eye::live::State> last_state_;
    std::optional<std::array<double, 4>> head_;
    bool quit_ = false;
    bool finished_ = false;
};

} // namespace pibridge::eyecal
