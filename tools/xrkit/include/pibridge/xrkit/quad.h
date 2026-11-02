// SPDX-License-Identifier: MIT
// A picture from the CPU as an OpenXR quad layer of its own (quad.cpp): its own swapchain (B8G8R8A8 sRGB when the
// runtime has it, else R8G8B8A8 sRGB with the bytes swapped), filled by a buffer copy only when a new picture comes;
// between pictures the runtime keeps showing the last one released. The runtime samples it straight onto the eyes, so
// a desktop's text stays as sharp as the headset can show (no second resampling through an eye image).
#pragma once

#include "pibridge/xrkit/xrkit.h"

#include <cstddef>
#include <cstdint>
#include <expected>
#include <string>

namespace pibridge::xrkit {

class QuadImage {
public:
    QuadImage() = default;
    QuadImage(const QuadImage&) = delete;
    QuadImage& operator=(const QuadImage&) = delete;
    ~QuadImage() { destroy(); }

    /// Copy a BGRA picture (rows `stride` bytes apart) into the next swapchain image and release it; the swapchain is
    /// made (again) when the size changes.
    std::expected<void, std::string> upload(Session& x, const std::uint8_t* bgra, std::uint32_t w, std::uint32_t h,
                                            std::size_t stride);
    /// A picture has been released (the layer can be submitted).
    bool ready() const { return released_; }
    std::uint32_t width() const { return w_; }
    std::uint32_t height() const { return h_; }
    /// The layer: `pose` (the quad's centre, facing +Z towards the viewer) in `space`, `sizeM` metres.
    XrCompositionLayerQuad layer(XrSpace space, const XrPosef& pose, XrExtent2Df sizeM) const;
    void destroy();

private:
    std::expected<void, std::string> make(Session& x, std::uint32_t w, std::uint32_t h);

    VkDevice device_ = VK_NULL_HANDLE;
    XrSwapchain sc_ = XR_NULL_HANDLE;
    std::vector<VkImage> images_;
    bool swapRB_ = false; // the swapchain is RGBA: the BGRA bytes are swapped on the way
    std::uint32_t w_ = 0, h_ = 0;
    VkBuffer staging_ = VK_NULL_HANDLE;
    VkDeviceMemory smem_ = VK_NULL_HANDLE;
    void* sptr_ = nullptr;
    VkCommandPool pool_ = VK_NULL_HANDLE;
    VkCommandBuffer cmd_ = VK_NULL_HANDLE;
    VkFence fence_ = VK_NULL_HANDLE;
    bool released_ = false;
};

} // namespace pibridge::xrkit
