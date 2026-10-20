// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/source.rs (MIT OR Apache-2.0).
//
// Frame sources: live V4L2 (v4l2.h), a recording (record.h) or a running camera broker (broker.h).
#pragma once

#include "pibridge/cam/io.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace pibridge::cam {

/// One raw frame as delivered by the device: FRAME_H rows of stride bytes (grey interpretation).
struct RawFrame {
    /// Frame bytes; valid until the source advances.
    std::span<const std::uint8_t> data;
    /// Row stride in bytes (2560 for this device).
    std::size_t stride = 0;
    /// Host CLOCK_MONOTONIC time (V4L2 buffer timestamp, or rebased replay time), ns.
    std::uint64_t host_ts_ns = 0;
    /// Driver sequence number.
    std::uint32_t sequence = 0;
};

/// Something that yields raw frames.
class Source {
public:
    virtual ~Source() = default;
    /// Move to the next frame. false on timeout (live) or end of file (replay without loop).
    virtual Result<bool> advance(int timeout_ms) = 0;
    /// The current frame (after a successful advance).
    virtual std::optional<RawFrame> current() const = 0;
    /// One-line description for logs.
    virtual std::string describe() const = 0;
    /// True for a replay that has reached its end.
    virtual bool finished() const { return false; }
    /// Buffers the driver flagged as corrupt (skipped).
    virtual std::uint64_t corrupt_frames() const { return 0; }
    /// True for a recording.
    virtual bool is_replay() const { return false; }
    /// Changes whenever the frames start coming from a reopened device or a new broker (the device clock and the
    /// frame counter may have restarted). 0 for sources that never reopen.
    virtual std::uint64_t generation() const { return 0; }
    /// False while the source is reopening its camera (or the broker reports its camera down).
    virtual bool up() const { return true; }
    /// Advance and return the frame.
    Result<std::optional<RawFrame>> next(int timeout_ms) {
        auto a = advance(timeout_ms);
        if (!a) {
            return fail(a.error());
        }
        if (*a) {
            return current();
        }
        return std::optional<RawFrame>{};
    }
};

/// Replay options.
struct ReplayOptions {
    /// Restart at the end.
    bool looping = false;
    /// Pace frames with their recorded spacing (otherwise as fast as asked).
    bool realtime = false;
};

/// Open a source from a string: empty or "auto" = autodetect the camera, "/dev/..." = that device, "broker" or
/// "broker:<socket>" = the frames of a running camera broker (broker.h), anything else = a recording file. fps = 0
/// keeps the driver default.
///
/// A live source (the camera, or a broker serving the camera) comes back wrapped in Resilient (resilient.h): after
/// an unplug, a stall or a broker restart it reopens by itself instead of failing (PIMAX_CAM_RECONNECT=0 turns that
/// off). The first open must succeed: it still fails fast.
Result<std::unique_ptr<Source>> open_source(std::string_view spec, std::uint32_t fps, ReplayOptions replay);

/// True for "broker" / "broker:<socket>".
bool is_broker_spec(std::string_view spec);

/// open_source without the reopen wrapper.
Result<std::unique_ptr<Source>> open_once(std::string_view spec, std::uint32_t fps, ReplayOptions replay);

} // namespace pibridge::cam
