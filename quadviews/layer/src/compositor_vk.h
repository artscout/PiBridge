// SPDX-License-Identifier: MIT
// The Vulkan side: composes a peripheral and a focus image into one eye image on the game's own device and queue.
//
// It runs inside xrEndFrame, where OpenXR gives the runtime (and so the layers) the use of the queue the game
// passed in its graphics binding. Queue order makes the game's rendering visible to us and ours to the runtime;
// the pipeline barriers cover the memory side. The game's images are in COLOR_ATTACHMENT_OPTIMAL when it has
// released them (XR_KHR_vulkan_enable) and are left in that layout again.
#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

#include <vulkan/vulkan.h>

namespace qv {

struct ImageRef {
    VkImage image = VK_NULL_HANDLE;
    uint32_t layer = 0;
    VkFormat format = VK_FORMAT_UNDEFINED;
    uint32_t width = 0, height = 0; // the whole image
};

// The push constants of compose.frag (7 x vec4).
struct ComposeParams {
    std::array<float, 4> focusRect;
    std::array<float, 4> periSrc;
    std::array<float, 4> focusSrc;
    std::array<float, 4> periTexel;
    std::array<float, 4> focusTexel;
    std::array<float, 4> params;
    std::array<float, 4> gaze;
};
static_assert(sizeof(ComposeParams) == 112);

struct EyeJob {
    ImageRef peripheral, focus, target;
    ComposeParams params;
};

class Compositor {
  public:
    Compositor() = default;
    Compositor(const Compositor&) = delete;
    Compositor& operator=(const Compositor&) = delete;
    ~Compositor();

    bool init(VkInstance instance, VkPhysicalDevice physicalDevice, VkDevice device, uint32_t queueFamily,
              uint32_t queueIndex);
    bool ready() const { return device_ != VK_NULL_HANDLE; }

    // Record and submit the composition of the jobs (one per eye). False when anything failed (logged).
    bool compose(const EyeJob* jobs, uint32_t count);

    // The game destroyed a swapchain: drop the views made for its images.
    void forgetImage(VkImage image);

    // GPU time of the last finished composition, milliseconds (0 when not measured).
    double lastGpuMs() const { return lastGpuMs_; }

  private:
    struct Pipe {
        VkRenderPass renderPass = VK_NULL_HANDLE;
        VkPipeline pipeline = VK_NULL_HANDLE;
    };
    struct Slot {
        VkCommandBuffer cmd = VK_NULL_HANDLE;
        VkFence fence = VK_NULL_HANDLE;
        std::array<VkDescriptorSet, 2> sets{};
        bool submitted = false;
    };
    using ViewKey = std::tuple<VkImage, uint32_t, VkFormat>;

    Pipe* pipeFor(VkFormat format);
    VkImageView viewFor(const ImageRef& r);
    VkFramebuffer framebufferFor(const ImageRef& r, VkRenderPass rp);
    void destroy();

    VkDevice device_ = VK_NULL_HANDLE;
    VkQueue queue_ = VK_NULL_HANDLE;
    VkCommandPool pool_ = VK_NULL_HANDLE;
    VkSampler sampler_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout setLayout_ = VK_NULL_HANDLE;
    VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
    VkDescriptorPool descriptorPool_ = VK_NULL_HANDLE;
    VkShaderModule vert_ = VK_NULL_HANDLE, frag_ = VK_NULL_HANDLE;
    VkQueryPool queries_ = VK_NULL_HANDLE;
    double timestampPeriodNs_ = 0;
    std::map<VkFormat, Pipe> pipes_;
    std::map<ViewKey, VkImageView> views_;
    std::map<std::pair<VkImageView, VkRenderPass>, std::pair<VkFramebuffer, VkImage>> framebuffers_;
    static constexpr uint32_t kSlots = 3;
    std::array<Slot, kSlots> slots_{};
    uint32_t next_ = 0;
    uint32_t maxEyes_ = 2;
    double lastGpuMs_ = 0;
};

} // namespace qv
