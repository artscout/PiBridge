// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/imulog.rs (MIT OR Apache-2.0).
//
// Head IMU log for offline replays (PIMAX_HEAD_IMU_LOG=<file>): every IMU sample the 6DoF output gets and every
// tracker pose time, so a "Record tracking" take (cameras.pxcam + this log) can be replayed through Basalt
// (`omarchyvr-slam take-euroc`). Monado owns the headset's HID interface, so this is the only place the head IMU of a
// running session can be saved.
//
// A leaf: the IMU thread only puts a small message into a bounded queue (8192, never blocks: a full queue drops); a
// writer thread owns the file. If the file cannot be written the log stops and nothing else changes. It writes for
// the first PIMAX_HEAD_IMU_LOG_MIN minutes (default 10) and afterwards only while
// $XDG_RUNTIME_DIR/omarchyvr-imu-log.on exists (the tracking recorder creates it for the length of a take).
//
// Every start keeps the earlier logs (rotate): the previous one becomes head-imu.<YYYYmmdd-HHMMSS>.csv (its own last
// write time, local) in keep_dir (~/.local/state/omarchyvr/head-imu/, on disk: the session dir is tmpfs) and
// head-imu.csv.prev in the session dir links to it; the newest KEEP old logs are kept.
//
// Format (text, one line each):
// * I,ts_ns,ax,ay,az,gx,gy,gz: IMU frame, m/s^2 and rad/s, CLOCK_MONOTONIC ns, exactly as fed to the tracker;
// * V,ts_ns,px,py,pz,qx,qy,qz,qw: a tracker pose; ts_ns is its camera frame time on the same clock, which ties the
//   camera's device clock to the IMU's.
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::cam::imulog {

/// Header line of the file.
inline constexpr std::string_view HEADER =
    "# pimax-head-imu v1: I,ts_ns,ax,ay,az,gx,gy,gz (IMU frame, m/s2, rad/s) | V,ts_ns,px,py,pz,qx,qy,qz,qw (tracker "
    "pose at its camera frame time); CLOCK_MONOTONIC ns";

/// Old logs kept next to the current one.
inline constexpr std::size_t KEEP = 3;

/// File that keeps the log writing after the first minutes (created by `omarchyvr record tracking`).
std::optional<std::string> flag_path();

struct Channel;

/// The sending side (owned by the head tracker). Dropping it closes the queue; the writer thread then writes what is
/// left, flushes and ends on its own (it is not joined, as in the Rust code).
class HeadImuLog {
public:
    /// From PIMAX_HEAD_IMU_LOG (and PIMAX_HEAD_IMU_LOG_MIN); nullptr when unset or the file cannot be created.
    static std::unique_ptr<HeadImuLog> from_env();
    /// Log to path: always for minutes, then while flag exists. An earlier log at path is kept (rotate).
    static std::unique_ptr<HeadImuLog> start(const std::string& path, double minutes, std::optional<std::string> flag);
    /// start with the directory for the kept logs (nullopt: they stay next to path). The earlier log is renamed at
    /// once (same directory, instant); the copy to keep runs on a thread of its own.
    static std::unique_ptr<HeadImuLog> start_keeping(const std::string& path, double minutes,
                                                     std::optional<std::string> flag, std::optional<std::string> keep);
    ~HeadImuLog();
    HeadImuLog(const HeadImuLog&) = delete;
    HeadImuLog& operator=(const HeadImuLog&) = delete;

    /// One IMU sample (never blocks).
    void imu(std::int64_t ts_ns, const std::array<double, 3>& accel, const std::array<double, 3>& gyro);
    /// One tracker pose (never blocks).
    void vio(std::int64_t ts_ns, const std::array<double, 3>& pos, const std::array<double, 4>& q);

    /// Messages dropped because the writer fell behind.
    std::uint64_t dropped = 0;

private:
    explicit HeadImuLog(std::shared_ptr<Channel> ch) : ch_(std::move(ch)) {}
    std::shared_ptr<Channel> ch_;
};

/// Where the kept logs go: $XDG_STATE_HOME/omarchyvr/head-imu, else $HOME/.local/state/omarchyvr/head-imu (nullopt
/// without either: they stay next to the log).
std::optional<std::string> keep_dir(const std::function<std::optional<std::string>(std::string_view)>& env);

/// Keep an existing log at path (dir/name.csv) now: move it to keep_dir/name.<stamp>.csv, point dir/name.csv.prev at
/// it (a symlink), and delete all but the newest keep stamped copies in keep_dir. Returns the kept file. Errors leave
/// the old log in dir (renamed to its stamp when possible).
std::optional<std::string> rotate(const std::string& path, const std::string& keep_dir, std::size_t keep);

/// A parsed head IMU log.
struct HeadImu {
    struct Imu {
        std::int64_t ts = 0;
        std::array<double, 3> accel{};
        std::array<double, 3> gyro{};
    };
    /// (ts ns, accel m/s^2, gyro rad/s)
    std::vector<Imu> imu;
    /// Tracker pose times (ns).
    std::vector<std::int64_t> vio_ts;
};

/// Read a log (unreadable lines are skipped: a take's slice can start or end inside a line).
HeadImu parse(std::string_view text);

} // namespace pibridge::cam::imulog
