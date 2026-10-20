// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/v4l2.rs (MIT OR Apache-2.0).
//
// Minimal V4L2 capture (raw ioctls, mmap streaming, nothing besides libc and <linux/videodev2.h>) for the camera
// node of 34a4:0014. Opening sends nothing to the headset: it is a plain UVC stream.
#pragma once

#include "pibridge/cam/source.h"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace pibridge::cam {

/// USB ids of the camera.
inline constexpr std::string_view CAM_VID = "34a4";
inline constexpr std::string_view CAM_PID = "0014";
/// V4L2_PIX_FMT_YUYV.
inline constexpr std::uint32_t PIX_FMT_YUYV = 0x5659'5559;

/// Static facts about an opened device.
struct DeviceInfo {
    std::string path;
    std::string driver;
    std::string card;
    std::string bus;
    /// Negotiated width x height (in YUYV pixels).
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t bytesperline = 0;
    std::uint32_t sizeimage = 0;
    /// Negotiated frame interval as fps (0 if unknown).
    double fps = 0;
    /// Frame intervals offered for YUYV at this size (fps).
    std::vector<double> offered_fps;
};

/// One /dev/videoN candidate for pick_node.
struct NodeEntry {
    std::string path;
    /// USB vendor id (lower case hex), empty if unknown.
    std::string vid;
    std::string pid;
    /// V4L2 capability bits (device_caps when present); nullopt if they could not be read.
    std::optional<std::uint32_t> caps;
    /// index from sysfs (0 = first node of the interface), nullopt if unknown.
    std::optional<std::uint32_t> index;
};

/// Pure selection: of the camera's nodes (USB 34a4:0014) pick the video-capture + streaming one (not the UVC
/// metadata node); when caps are unknown fall back to sysfs index 0. Lowest node number wins.
std::optional<std::string> pick_node(const std::vector<NodeEntry>& entries);

/// Find /dev/videoN of the camera (USB 34a4:0014, capture capable). nullopt when not plugged in. Reads sysfs and
/// issues QUERYCAP on the candidate nodes (that does not claim the device).
std::optional<std::string> find_device();

/// An open, streaming capture device.
class V4l2Capture final : public Source {
public:
    /// Open path, set YUYV 1280x832, request fps (0 = keep the driver default), map nbufs buffers, start streaming.
    static Result<std::unique_ptr<V4l2Capture>> open(const std::string& path, std::uint32_t fps, std::uint32_t nbufs);
    ~V4l2Capture() override;

    Result<bool> advance(int timeout_ms) override;
    std::optional<RawFrame> current() const override;
    std::string describe() const override;
    std::uint64_t corrupt_frames() const override { return corrupt; }

    /// What was negotiated.
    DeviceInfo info;
    /// Buffers dequeued with V4L2_BUF_FLAG_ERROR or short bytesused (skipped).
    std::uint64_t corrupt = 0;
    /// Frames whose timestamp was not CLOCK_MONOTONIC (should stay 0).
    std::uint64_t non_monotonic_ts = 0;

private:
    struct Mapping {
        void* ptr = nullptr;
        std::size_t len = 0;
    };
    struct Held {
        std::uint32_t index = 0;
        std::size_t len = 0;
        std::uint64_t host_ts_ns = 0;
        std::uint32_t sequence = 0;
    };
    V4l2Capture() = default;
    Result<void> queue(std::uint32_t index);
    Result<void> requeue_held();

    int fd_ = -1;
    std::vector<Mapping> maps_;
    std::optional<Held> held_;
    bool streaming_ = false;
};

} // namespace pibridge::cam
