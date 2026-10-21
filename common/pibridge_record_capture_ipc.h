// SPDX-License-Identifier: MIT
// The IPC protocol between PiBridge's Vulkan capture layer (tools/record-layer, the SERVER, loaded inside SteamVR's
// vrcompositor) and the recorder/encoder (tools/record `pibridge-record run --source layer`, the CLIENT).
//
// Transport: an AF_UNIX SOCK_STREAM socket at `$XDG_RUNTIME_DIR/pibridge/record-capture.sock`. The layer binds and
// listens; the client connects. Everything is little-endian host layout (both ends are the same machine, same ABI).
//
// Handshake and flow:
//   1. The client connects. As soon as the layer has created its exportable capture image (on the first active
//      frame), it sends ONE HelloMsg whose ancillary data carries the image's exported fd (SCM_RIGHTS). The fd is a
//      Vulkan external-memory handle (VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT): the client imports it into its
//      own VkDeviceMemory (same physical GPU, matched by deviceUUID) and binds an identically-created VkImage.
//   2. Per captured frame the layer sends a FrameMsg (no fd): the shared image now holds frame `seq`.
//   3. After it has finished reading the shared image for that frame, the client sends ONE ack byte (kAckByte). The
//      layer never overwrites the shared image until every connected client has acked the previous frame, so the one
//      reused image is never torn. A client that never acks (or a slow one) simply makes the layer skip frames — the
//      layer NEVER blocks the compositor (zero missed VR frames; the capture is a leaf).
//
// This header is deliberately free of <vulkan/vulkan.h>: the recorder's CLI (which only writes the control file) and
// the layer both include it. The few Vulkan enum values the two ends must agree on are repeated here as constants.
#pragma once

#include <cstdint>

namespace pibridge::reclayer {

// The socket's basename inside `$XDG_RUNTIME_DIR/pibridge/` (see capture_control.h for the control file).
inline constexpr char kCaptureSockName[] = "record-capture.sock";

inline constexpr std::uint32_t kCaptureMagic = 0x50434150u;   // "PCAP"
inline constexpr std::uint32_t kCaptureVersion = 1u;

// Message discriminator (the first word after magic/version is always the type).
enum class MsgType : std::uint32_t { Hello = 1, Frame = 2, Bye = 3 };

// The client's one-byte acknowledgement ("I have read the shared image for the last frame; you may reuse it").
inline constexpr std::uint8_t kAckByte = 0x01u;

// --- the fixed contract for the shared image (both ends create/import it with exactly these) ----------------------
// VK_FORMAT_R8G8B8A8_UNORM (37): the readback bytes are then R,G,B,A in memory, which is ffmpeg's `-pix_fmt rgba`.
inline constexpr std::int32_t kCaptureVkFormat = 37;
// VK_IMAGE_USAGE_TRANSFER_SRC_BIT (0x1) | VK_IMAGE_USAGE_TRANSFER_DST_BIT (0x2): the layer blits INTO it, the client
// copies OUT of it.
inline constexpr std::uint32_t kCaptureVkImageUsage = 0x1u | 0x2u;
// VK_IMAGE_TILING_OPTIMAL (0).
inline constexpr std::uint32_t kCaptureVkImageTiling = 0u;
// VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT (0x1).
inline constexpr std::uint32_t kCaptureHandleTypeOpaqueFd = 0x1u;

// Sent once, with the exported fd as SCM_RIGHTS ancillary data.
struct HelloMsg {
    std::uint32_t magic = kCaptureMagic;
    std::uint32_t version = kCaptureVersion;
    std::uint32_t type = static_cast<std::uint32_t>(MsgType::Hello);
    std::uint32_t width = 0;  // the exportable image's extent (already scaled to <= max_height, side-by-side for both)
    std::uint32_t height = 0;
    std::int32_t vkFormat = kCaptureVkFormat;
    std::uint32_t vkImageUsage = kCaptureVkImageUsage;
    std::uint32_t vkImageTiling = kCaptureVkImageTiling;
    std::uint32_t fps = 0;           // the pace the layer blits at (from the control file); the client paces to it too
    std::uint32_t handleType = kCaptureHandleTypeOpaqueFd;
    std::uint64_t allocationSize = 0;   // VkMemoryRequirements.size the layer allocated (the client imports this many bytes)
    std::uint32_t memoryTypeIndex = 0;  // the device-local memory type the layer chose (the client binds the same)
    std::uint32_t bytesPerRow = 0;      // 0 for OPTIMAL tiling (not host-knowable; the client copies via Vulkan)
    std::uint64_t drmFormatModifier = UINT64_MAX; // UINT64_MAX = none/unknown (OPAQUE_FD carries no DRM modifier)
    std::uint32_t queueFamilyIndex = 0; // the layer's queue family (for the EXTERNAL ownership transfer, if used)
    std::uint32_t _pad = 0;
    std::uint8_t deviceUUID[16] = {};   // VkPhysicalDeviceIDProperties.deviceUUID: the client imports on this GPU
};

// Sent per captured frame (no ancillary fd: the one shared image is reused).
struct FrameMsg {
    std::uint32_t magic = kCaptureMagic;
    std::uint32_t version = kCaptureVersion;
    std::uint32_t type = static_cast<std::uint32_t>(MsgType::Frame);
    std::uint32_t _pad = 0;
    std::uint64_t seq = 0;
    std::uint64_t timestamp_ns = 0; // CLOCK_MONOTONIC, for the recorder's pacer (constant frame-rate output)
};

} // namespace pibridge::reclayer
