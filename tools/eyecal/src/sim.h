// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/sim.rs (MIT OR Apache-2.0); the simulated chip of the on-chip mode is new.
//
// A simulated wearer and uncalibrated tracker, to run the whole procedure without a headset (`--simulate`, tests).
//
// The wearer looks at the first mark of the scene (the target) after a reaction time, with a 40 ms saccade, small
// fixation drift and blinks. The "tracker" reports each eye with what the uncalibrated EyeChip shows on Linux: a gain
// of about 0.6, per-eye offsets of several degrees (the eyes seem to diverge by ~16 degrees), a compressed upper field,
// 0.7 degrees of noise per axis and sample, dropped samples, and the eyelid artefact at blink edges.
//
// In the on-chip mode a simulated chip answers the calibration requests with the captured delays (0x407 165 ms,
// 0x42e 500 ms, 0x44c 80 ms) and a synthetic profile; after its first 0x42e it reports the gaze as a calibrated chip
// does: right, apart from a constant per-eye offset (SimParams::chip_offset_deg, like the 9-14 degree offset between
// the chip's frame and SteamVR's view seen on 2026-10-18).
#pragma once

#include "procedure.h"

#include "pibridge/eye/calibration.h"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace pibridge::eyecal {

/// Simulation settings.
struct SimParams {
    /// Noise per axis and sample, degrees.
    double noise_deg = 0.7;
    /// Probability that a sample of one eye is dropped.
    double drop = 0.03;
    /// Mean time between blinks, seconds.
    double blink_every_s = 4.0;
    /// Reaction time to a new target, seconds.
    double latency_s = 0.22;
    /// Seed.
    std::uint64_t seed = 1;
    /// Head turn (degrees) at this time (seconds), once: tests the head-motion check.
    std::optional<std::pair<double, double>> head_turn;
    /// The wearer looks this many degrees beside one calibration target (index): tests the outlier handling.
    std::optional<std::pair<std::size_t, double>> glance;
    /// The calibrated chip's constant offset per eye, degrees (u right, v up).
    std::array<Tangent, 2> chip_offset_deg = {Tangent{0.5, -3.0}, Tangent{-0.4, -2.5}};
    /// The simulated chip refuses this opcode (status 0).
    std::optional<std::uint32_t> chip_refuse;
    /// Stop at the end screen without pressing anything (tests of the end screen). Otherwise the simulation saves:
    /// B in the grid flows (the chip mode, --recenter), which goes back to the check grid, where it looks for 1 s and
    /// presses B again (exit); Accept (menu / Y) in the host modes.
    bool stop_at_end = false;
};

/// What the uncalibrated chip makes of a true per-eye tangent.
Tangent distort(std::size_t eye, Tangent t);

/// One request the simulated chip got.
struct SimRequest {
    pibridge::eye::calibration::Step step;
    std::uint32_t seq = 0;
    /// Time it was sent.
    double t = 0.0;
    /// Where the wearer's eyes pointed then (head-frame point).
    Vec3 gaze{0.0, 0.0, -2.0};
};

/// Result of a simulated run.
struct SimOutcome {
    /// Everything the procedure printed.
    std::vector<std::string> log;
    /// The procedure at the end.
    Procedure procedure;
    /// The calibration file text, if it was saved.
    std::optional<std::string> saved;
    /// The on-chip mode: the requests the chip got, in order.
    std::vector<SimRequest> chip_requests;
    /// The new profile the procedure handed over (Effect::SaveProfile), and whether it was kept.
    std::optional<pibridge::eye::Bytes> profile;
    bool profile_kept = false;
    /// The procedure asked to quit.
    bool quit = false;
    /// The grid flows: the check grid's gaze dot (head-frame direction) after the save, just before the exit, and
    /// where the wearer looked then (head-frame point).
    std::optional<Vec3> grid_gaze;
    Vec3 grid_look{0.0, 0.0, -2.0};
};

/// The synthetic profile body the simulated chip downloads (33 887 bytes, the size of the capture's last download).
pibridge::eye::Bytes sim_profile_body();

/// Run a full calibration + validation (+ pursuit if `proc.s.pursuit`) with a simulated wearer; presses A at the
/// start (from the check grid in the grid flows) and saves at the end (SimParams::stop_at_end).
SimOutcome run(Procedure proc, const SimParams& p);

} // namespace pibridge::eyecal
