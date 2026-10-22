// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/camera.rs (MIT OR Apache-2.0).
//
// The capture pipeline used by the C ABI and the tool: source -> metadata -> tile rows -> timestamps -> stats, with
// optional raw recording of every frame.
#pragma once

#include "pibridge/cam/clock.h"
#include "pibridge/cam/frame.h"
#include "pibridge/cam/record.h"
#include "pibridge/cam/source.h"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>

namespace pibridge::cam {

/// Frame info flag: metadata parsed and plausible.
inline constexpr std::uint32_t FLAG_META_VALID = 1;
/// Frame info flag: at least one tile row was found by scanning (metadata offset unusable).
inline constexpr std::uint32_t FLAG_ROWS_SCANNED = 2;
/// Frame info flag: frame comes from a recording.
inline constexpr std::uint32_t FLAG_REPLAY = 4;
/// Frame info flag: the stream carries no device timestamp for this frame (the Motion-JPEG profile of the Crystal
/// Super: its metadata block does not survive JPEG). dev_ts_us and counter are then 0 and mono_ts_ns is the host
/// CLOCK_MONOTONIC time of the frame.
inline constexpr std::uint32_t FLAG_NO_DEV_TIME = 8;

/// A Motion-JPEG frame whose tiles start above this row comes from the short-exposure (controller-LED) window. The
/// firmware runs its two exposure programs alternately and they read out from different rows: the raw stream's
/// metadata reports 278 for the tracking window and 10 for the LED one (pimax_cam.h), while a Crystal Super MJPEG
/// stream shows 96 and 9, so the two are told apart by which part of the sensor they start in.
inline constexpr std::uint16_t LED_WINDOW_MAX_ROW = 48;
/// Per-frame result (the layout of the C struct pimax_cam_frame_info).
struct FrameInfo {
    /// Host CLOCK_MONOTONIC ns when the driver saw the frame (V4L2 timestamp; rebased for replays).
    std::uint64_t host_ts_ns = 0;
    /// Device timestamp (MCU us, IMU clock).
    std::uint64_t dev_ts_us = 0;
    /// Device time mapped to CLOCK_MONOTONIC by the minimum-delay filter (fallback without IMU reference).
    std::uint64_t mono_ts_ns = 0;
    std::uint32_t counter = 0;
    std::uint32_t sequence = 0;
    /// Frames missing just before this one (counter gap minus one).
    std::uint32_t dropped = 0;
    /// FrameKind as u32.
    std::uint32_t kind = 0;
    std::uint32_t frame_type = 0;
    std::uint32_t stream_index = 0;
    std::array<std::uint16_t, 4> exposure{};
    std::array<std::uint16_t, 4> gain{};
    /// Tile start rows actually used.
    std::array<std::uint16_t, 4> y_offset{};
    std::uint32_t tile_width = 0;
    std::uint32_t tile_height = 0;
    /// FLAG_*.
    std::uint32_t flags = 0;
    std::uint32_t reserved = 0;
    bool operator==(const FrameInfo&) const = default;
};

/// Running counters.
struct Stats {
    std::uint64_t frames = 0;
    std::uint64_t tracking = 0;
    std::uint64_t led = 0;
    /// Frames with unrecognised metadata.
    std::uint64_t unknown = 0;
    /// Sum of counter gaps (lost frames).
    std::uint64_t dropped = 0;
    /// Frames whose tile rows had to be scanned.
    std::uint64_t scanned_rows = 0;
    /// Frames where a metadata row offset did not match the zero/non-zero boundary (diagnostic).
    std::uint64_t row_mismatch = 0;
    /// Frames whose kind did not alternate (tracking after tracking or LED after LED).
    std::uint64_t kind_repeats = 0;
    /// Frames the recorder could not queue (disk too slow).
    std::uint64_t record_dropped = 0;
    /// Times the source came back from a reopened camera or a new broker (clock and counter tracking restarted).
    std::uint64_t reopens = 0;
};

/// A borrowed view of one tile of the current frame (no copy; for statistics such as auto-exposure).
struct TileView {
    std::span<const std::uint8_t> data;
    std::size_t stride = 0;
    std::size_t x0 = 0;
    std::size_t y0 = 0;
    std::size_t w = 0;
    std::size_t h = 0;
};

/// Capture pipeline.
class Camera {
public:
    /// Wrap a source.
    explicit Camera(std::unique_ptr<Source> src);
    /// Open by spec (see open_source).
    static Result<std::unique_ptr<Camera>> open(std::string_view spec, std::uint32_t fps, ReplayOptions replay);

    std::string describe() const { return src_->describe(); }
    const Source& source() const { return *src_; }

    /// Start recording every following frame to path (zlib: per-frame compression level, nullopt = raw).
    Result<void> start_recording(const std::string& path, std::optional<std::uint8_t> zlib);
    /// Stop recording; returns (frames, bytes) or nullopt when not recording.
    Result<std::optional<std::pair<std::uint64_t, std::uint64_t>>> stop_recording();
    bool recording() const { return recorder_ != nullptr; }

    /// Wait for the next frame. nullopt: timeout / end of a replay.
    Result<std::optional<FrameInfo>> next_frame(int timeout_ms);
    const FrameInfo& info() const { return info_; }

    /// Copy tile t of the current frame into dst (row pitch dst_stride >= 640, 480 rows).
    Result<void> copy_tile(std::size_t t, std::span<std::uint8_t> dst, std::size_t dst_stride) const;
    /// Borrowed view of tile t of the current frame.
    Result<TileView> tile_view(std::size_t t) const;
    /// Current tile as an owned 640x480 buffer.
    Result<std::vector<std::uint8_t>> tile(std::size_t t) const;
    /// The current raw frame bytes.
    std::optional<std::span<const std::uint8_t>> raw() const;
    std::uint64_t corrupt_frames() const { return src_->corrupt_frames(); }
    /// Current host-device clock offset of the fallback mapping.
    std::int64_t clock_offset_ns() const { return clock_.offset_ns(); }

    /// Check every frame's row offsets against the picture (costs ~4 row scans per frame).
    bool verify_rows = true;
    Stats stats;

private:
    std::unique_ptr<Source> src_;
    HostClockMap clock_;
    std::unique_ptr<AsyncRecorder> recorder_;
    std::optional<TileRows> rows_;
    FrameInfo info_;
    std::optional<std::uint32_t> last_counter_;
    FrameKind last_kind_ = FrameKind::Unknown;
    std::uint64_t generation_ = 0;
};

} // namespace pibridge::cam
