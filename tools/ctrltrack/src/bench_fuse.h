// SPDX-License-Identifier: MIT
// pibridge-ctrlbench --fuse: the controller fuser on a synthetic IMU made from tracked trajectories (bench.cpp).
#pragma once

#include "pibridge/cam/geom.h"

#include <cstddef>
#include <cstdint>
#include <array>
#include <optional>
#include <string>
#include <vector>

namespace pibridge::ctrlbench {

struct FixIn {
    std::size_t slot;
    std::uint64_t ns;
    cam::geom::Pose body_from_model;
    std::size_t leds;
    double rms_px;
};

struct FuseBenchIn {
    std::vector<FixIn> fixes;
    bool parity = false;
    unsigned long seed = 1;
    /// Gyro bias left after the fuser's own estimate (deg/s per axis, random sign).
    double bias_err_dps = 0.3;
    /// Probability that a fix handed to the fuser is the other hand's (a left/right swap).
    double swap_p = 0.0;
};

void fuse_bench(const FuseBenchIn& in);

/// The fuser of both hands in closed loop with the tracker (pibridge-ctrlbench --hint): a synthetic IMU from the truth
/// (the reference run's fixes, smoothed), the tracker's own fixes handed over 30 ms after their frame, and the hint
/// (the fuser's pose at an LED frame's time) back to the tracker. No hint where the truth has no data (the IMU cannot
/// be made there).
/// The fuser on a real IMU log (PIMAX_CTRL_IMU_LOG, `# pimax-ctrlfuse log v1` lines I / O / P): the replay's
/// verdicts against the logged ones, then cuts of the optical fixes where the hand was well tracked (the logged 8+ LED
/// fixes are the truth). `calib_json` is the ctrl-imu.json the session used (empty = the built-in calibration).
int imu_log_bench(const std::string& path, const std::string& calib_json, bool parity);
/// Writes a synthetic log with the Rust fuser into `dir` and replays it (exit 0 when every verdict comes back).
int imu_log_selftest(const std::string& dir);

/// The controllers' real IMU logs (PIMAX_CTRL_IMU_LOG, both hands) replayed into a fuser per hand for the tracker's
/// hand identity (pibridge-ctrlbench --imu): what the fuser would write into the hint file at time t.
class ImuFeed {
public:
    ImuFeed(const std::string& left, const std::string& right, const std::string& calib_json);
    ~ImuFeed();
    ImuFeed(const ImuFeed&) = delete;
    ImuFeed& operator=(const ImuFeed&) = delete;
    bool ok() const;
    struct Sample {
        std::uint64_t t;
        std::array<double, 4> qg;
        std::array<double, 3> up;
        bool up_ok;
        double still_s;
        /// Mean specific force since the previous sample (aim frame, m/s^2).
        std::array<double, 3> f_aim;
        /// The head pose at t (world_from_head, from the log's O / P lines; identity when the log has none).
        std::array<double, 4> head_q;
        std::array<double, 3> head_p;
    };
    /// IMU of both hands up to t; the samples every 10 ms since the last call.
    std::array<std::vector<Sample>, 2> advance(std::uint64_t t);

private:
    struct Impl;
    Impl* d_;
};

class HintSim {
public:
    HintSim(const std::vector<FixIn>& reference, unsigned long seed, double bias_err_dps);
    ~HintSim();
    HintSim(const HintSim&) = delete;
    HintSim& operator=(const HintSim&) = delete;
    struct Out {
        bool valid = false;
        cam::geom::Pose body_from_model;
        double fix_age_s = 0.0;
        std::uint32_t state = 0;
    };
    /// Advance to the tracker's processing of the LED frame at frame_ns (IMU up to frame_ns + 20 ms, fixes that have
    /// arrived by then) and return the fuser's pose of each hand at frame_ns.
    std::array<Out, 2> at(std::uint64_t frame_ns);
    /// A fix the tracker made (given to the fuser 30 ms after frame_ns).
    void fix(std::size_t slot, std::uint64_t frame_ns, const cam::geom::Pose& body_from_model, std::size_t leds,
             double rms_px);
    /// Truth position of a hand at t (smoothed reference), if covered.
    std::optional<cam::geom::Vec3> truth(std::size_t slot, std::uint64_t t) const;

private:
    struct Impl;
    Impl* d_;
};

} // namespace pibridge::ctrlbench
