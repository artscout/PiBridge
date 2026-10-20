// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/xrapp.rs and src/gpu.rs (MIT OR Apache-2.0).
#include "xrapp.h"

#include "hud.h"
#include "marks.h"

#include "pibridge/eyemath/clock.h"

#include <vulkan/vulkan.h>
#define XR_USE_GRAPHICS_API_VULKAN
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include "marks_frag.h"
#include "marks_vert.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <format>
#include <mutex>
#include <thread>
#include <vector>

#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace pibridge::eyecal {

namespace {

using namespace std::chrono_literals;

/// A zeroed Vulkan / OpenXR structure with its type set (no missing-initializer noise).
template <class T>
T vk_make(VkStructureType t) {
    T v{};
    v.sType = t;
    return v;
}
template <class T>
T xr_make(XrStructureType t) {
    T v{};
    v.type = t;
    return v;
}

constexpr XrViewConfigurationType VIEW_TYPE = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
constexpr XrPosef IDENTITY_POSE = {{0.0F, 0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 0.0F}};

std::string xr_err(const char* what, XrResult r) { return std::format("{}: XrResult {}", what, static_cast<int>(r)); }
std::string vk_err(const char* what, VkResult r) { return std::format("{}: VkResult {}", what, static_cast<int>(r)); }

#define PIBRIDGE_XR(call, what)                                                                                       \
    do {                                                                                                           \
        const XrResult pibridge_r = (call);                                                                           \
        if (XR_FAILED(pibridge_r)) {                                                                                  \
            return std::unexpected(xr_err(what, pibridge_r));                                                         \
        }                                                                                                          \
    } while (0)
#define PIBRIDGE_VK(call, what)                                                                                       \
    do {                                                                                                           \
        const VkResult pibridge_r = (call);                                                                           \
        if (pibridge_r != VK_SUCCESS) {                                                                               \
            return std::unexpected(vk_err(what, pibridge_r));                                                         \
        }                                                                                                          \
    } while (0)

// ------------------------------------------------------------------------------------------------ terminal keys

/// Raw terminal on stdin while the app runs (keys without Enter); restored at the end.
class RawTerminal {
public:
    RawTerminal() {
        if (::isatty(0) == 0 || ::tcgetattr(0, &saved_) != 0) {
            return;
        }
        termios t = saved_;
        t.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        t.c_cc[VMIN] = 1;
        t.c_cc[VTIME] = 0;
        active_ = ::tcsetattr(0, TCSANOW, &t) == 0;
    }
    ~RawTerminal() {
        if (active_) {
            ::tcsetattr(0, TCSANOW, &saved_);
        }
    }
    RawTerminal(const RawTerminal&) = delete;
    RawTerminal& operator=(const RawTerminal&) = delete;

private:
    termios saved_{};
    bool active_ = false;
};

/// Reads stdin on its own thread (poll with a timeout, so the stop token ends it).
class Keyboard {
public:
    Keyboard()
        : th_([this](std::stop_token st) {
              while (!st.stop_requested()) {
                  pollfd p{0, POLLIN, 0};
                  if (::poll(&p, 1, 100) <= 0) {
                      continue;
                  }
                  char c = 0;
                  if (::read(0, &c, 1) != 1) {
                      return; // stdin closed (started from the app)
                  }
                  std::lock_guard l(mu_);
                  keys_.push_back(c);
              }
          }) {}
    std::vector<char> take() {
        std::lock_guard l(mu_);
        std::vector<char> v(keys_.begin(), keys_.end());
        keys_.clear();
        return v;
    }

private:
    std::mutex mu_;
    std::deque<char> keys_;
    std::jthread th_;
};

// ------------------------------------------------------------------------------------------------ Vulkan

std::optional<std::uint32_t> memory_type(const VkPhysicalDeviceMemoryProperties& p, std::uint32_t bits,
                                         VkMemoryPropertyFlags flags) {
    for (std::uint32_t i = 0; i < p.memoryTypeCount; i++) {
        if ((bits & (1U << i)) != 0 && (p.memoryTypes[i].propertyFlags & flags) == flags) {
            return i;
        }
    }
    return std::nullopt;
}

constexpr VkImageSubresourceRange COLOR_RANGE = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};

/// The marks pass: pipeline, one uniform buffer and descriptor set per eye, a framebuffer per swapchain image.
struct MarksGpu {
    VkDevice device = VK_NULL_HANDLE;
    VkQueue queue = VK_NULL_HANDLE;
    VkCommandPool pool = VK_NULL_HANDLE;
    VkCommandBuffer cmd = VK_NULL_HANDLE;
    VkFence fence = VK_NULL_HANDLE;
    VkRenderPass render_pass = VK_NULL_HANDLE;
    VkDescriptorSetLayout set_layout = VK_NULL_HANDLE;
    VkPipelineLayout layout = VK_NULL_HANDLE;
    VkPipeline pipeline = VK_NULL_HANDLE;
    VkDescriptorPool desc_pool = VK_NULL_HANDLE;
    std::array<VkDescriptorSet, 2> sets{};
    struct Ubo {
        VkBuffer buf = VK_NULL_HANDLE;
        VkDeviceMemory mem = VK_NULL_HANDLE;
        void* ptr = nullptr;
    };
    std::array<Ubo, 2> ubo{};
    std::array<std::vector<std::pair<VkImageView, VkFramebuffer>>, 2> targets;
    std::array<std::uint32_t, 2> size{};

    std::expected<void, std::string> create(VkDevice dev, VkQueue q, std::uint32_t qf,
                                            const VkPhysicalDeviceMemoryProperties& mem_props, VkFormat format,
                                            std::array<std::uint32_t, 2> sz, const std::array<std::vector<VkImage>, 2>& images) {
        device = dev;
        queue = q;
        size = sz;
        VkShaderModuleCreateInfo vsi{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO, nullptr, 0, sizeof(marks_vert), marks_vert};
        VkShaderModuleCreateInfo fsi{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO, nullptr, 0, sizeof(marks_frag), marks_frag};
        VkShaderModule vs = VK_NULL_HANDLE;
        VkShaderModule fs = VK_NULL_HANDLE;
        PIBRIDGE_VK(vkCreateShaderModule(device, &vsi, nullptr, &vs), "vertex shader");
        PIBRIDGE_VK(vkCreateShaderModule(device, &fsi, nullptr, &fs), "fragment shader");
        VkDescriptorSetLayoutBinding binding{0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr};
        VkDescriptorSetLayoutCreateInfo dsl{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO, nullptr, 0, 1, &binding};
        PIBRIDGE_VK(vkCreateDescriptorSetLayout(device, &dsl, nullptr, &set_layout), "descriptor set layout");
        VkPipelineLayoutCreateInfo pli{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO, nullptr, 0, 1, &set_layout, 0, nullptr};
        PIBRIDGE_VK(vkCreatePipelineLayout(device, &pli, nullptr, &layout), "pipeline layout");
        VkAttachmentDescription att{0,
                                    format,
                                    VK_SAMPLE_COUNT_1_BIT,
                                    VK_ATTACHMENT_LOAD_OP_DONT_CARE,
                                    VK_ATTACHMENT_STORE_OP_STORE,
                                    VK_ATTACHMENT_LOAD_OP_DONT_CARE,
                                    VK_ATTACHMENT_STORE_OP_DONT_CARE,
                                    VK_IMAGE_LAYOUT_UNDEFINED,
                                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
        VkAttachmentReference ref{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
        VkSubpassDescription sub{};
        sub.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        sub.colorAttachmentCount = 1;
        sub.pColorAttachments = &ref;
        VkSubpassDependency dep{VK_SUBPASS_EXTERNAL,
                                0,
                                VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
                                VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                                VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT,
                                VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
                                0};
        VkRenderPassCreateInfo rpi{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO, nullptr, 0, 1, &att, 1, &sub, 1, &dep};
        PIBRIDGE_VK(vkCreateRenderPass(device, &rpi, nullptr, &render_pass), "render pass");
        VkPipelineShaderStageCreateInfo stages[2] = {
            {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_VERTEX_BIT, vs, "main", nullptr},
            {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_FRAGMENT_BIT, fs, "main", nullptr}};
        auto vin = vk_make<VkPipelineVertexInputStateCreateInfo>(VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO);
        auto ia = vk_make<VkPipelineInputAssemblyStateCreateInfo>(VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO);
        ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        VkViewport viewport{0.0F, 0.0F, static_cast<float>(size[0]), static_cast<float>(size[1]), 0.0F, 1.0F};
        VkRect2D scissor{{0, 0}, {size[0], size[1]}};
        auto vp = vk_make<VkPipelineViewportStateCreateInfo>(VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO);
        vp.viewportCount = 1;
        vp.pViewports = &viewport;
        vp.scissorCount = 1;
        vp.pScissors = &scissor;
        auto rs = vk_make<VkPipelineRasterizationStateCreateInfo>(VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO);
        rs.polygonMode = VK_POLYGON_MODE_FILL;
        rs.cullMode = VK_CULL_MODE_NONE;
        rs.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        rs.lineWidth = 1.0F;
        auto ms = vk_make<VkPipelineMultisampleStateCreateInfo>(VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO);
        ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        VkPipelineColorBlendAttachmentState blend_att{};
        blend_att.colorWriteMask =
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        auto blend = vk_make<VkPipelineColorBlendStateCreateInfo>(VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO);
        blend.attachmentCount = 1;
        blend.pAttachments = &blend_att;
        auto gpi = vk_make<VkGraphicsPipelineCreateInfo>(VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO);
        gpi.stageCount = 2;
        gpi.pStages = stages;
        gpi.pVertexInputState = &vin;
        gpi.pInputAssemblyState = &ia;
        gpi.pViewportState = &vp;
        gpi.pRasterizationState = &rs;
        gpi.pMultisampleState = &ms;
        gpi.pColorBlendState = &blend;
        gpi.layout = layout;
        gpi.renderPass = render_pass;
        gpi.subpass = 0;
        const VkResult pr = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &gpi, nullptr, &pipeline);
        vkDestroyShaderModule(device, vs, nullptr);
        vkDestroyShaderModule(device, fs, nullptr);
        if (pr != VK_SUCCESS) {
            return std::unexpected(vk_err("graphics pipeline", pr));
        }
        VkDescriptorPoolSize ps{VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 2};
        VkDescriptorPoolCreateInfo dpi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO, nullptr, 0, 2, 1, &ps};
        PIBRIDGE_VK(vkCreateDescriptorPool(device, &dpi, nullptr, &desc_pool), "descriptor pool");
        const std::array<VkDescriptorSetLayout, 2> layouts = {set_layout, set_layout};
        VkDescriptorSetAllocateInfo dsa{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO, nullptr, desc_pool, 2, layouts.data()};
        PIBRIDGE_VK(vkAllocateDescriptorSets(device, &dsa, sets.data()), "descriptor sets");
        const VkDeviceSize len = BLOCK_FLOATS * sizeof(float);
        for (std::size_t e = 0; e < 2; e++) {
            auto bi = vk_make<VkBufferCreateInfo>(VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO);
            bi.size = len;
            bi.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
            bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
            PIBRIDGE_VK(vkCreateBuffer(device, &bi, nullptr, &ubo[e].buf), "uniform buffer");
            VkMemoryRequirements req;
            vkGetBufferMemoryRequirements(device, ubo[e].buf, &req);
            const auto mt = memory_type(mem_props, req.memoryTypeBits,
                                        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
            if (!mt) {
                return std::unexpected(std::string("no host visible and coherent memory for the uniform buffer"));
            }
            VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO, nullptr, req.size, *mt};
            PIBRIDGE_VK(vkAllocateMemory(device, &mai, nullptr, &ubo[e].mem), "uniform memory");
            PIBRIDGE_VK(vkBindBufferMemory(device, ubo[e].buf, ubo[e].mem, 0), "uniform bind");
            PIBRIDGE_VK(vkMapMemory(device, ubo[e].mem, 0, VK_WHOLE_SIZE, 0, &ubo[e].ptr), "uniform map");
            VkDescriptorBufferInfo info{ubo[e].buf, 0, len};
            auto w = vk_make<VkWriteDescriptorSet>(VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET);
            w.dstSet = sets[e];
            w.dstBinding = 0;
            w.descriptorCount = 1;
            w.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            w.pBufferInfo = &info;
            vkUpdateDescriptorSets(device, 1, &w, 0, nullptr);
        }
        VkCommandPoolCreateInfo cpi{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO, nullptr,
                                    VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT, qf};
        PIBRIDGE_VK(vkCreateCommandPool(device, &cpi, nullptr, &pool), "command pool");
        VkCommandBufferAllocateInfo cba{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO, nullptr, pool,
                                        VK_COMMAND_BUFFER_LEVEL_PRIMARY, 1};
        PIBRIDGE_VK(vkAllocateCommandBuffers(device, &cba, &cmd), "command buffer");
        auto fci = vk_make<VkFenceCreateInfo>(VK_STRUCTURE_TYPE_FENCE_CREATE_INFO);
        PIBRIDGE_VK(vkCreateFence(device, &fci, nullptr, &fence), "fence");
        for (std::size_t e = 0; e < 2; e++) {
            for (const VkImage img : images[e]) {
                auto vi = vk_make<VkImageViewCreateInfo>(VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO);
                vi.image = img;
                vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
                vi.format = format;
                vi.subresourceRange = COLOR_RANGE;
                VkImageView view = VK_NULL_HANDLE;
                PIBRIDGE_VK(vkCreateImageView(device, &vi, nullptr, &view), "swapchain image view");
                auto fbi = vk_make<VkFramebufferCreateInfo>(VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO);
                fbi.renderPass = render_pass;
                fbi.attachmentCount = 1;
                fbi.pAttachments = &view;
                fbi.width = size[0];
                fbi.height = size[1];
                fbi.layers = 1;
                VkFramebuffer fb = VK_NULL_HANDLE;
                PIBRIDGE_VK(vkCreateFramebuffer(device, &fbi, nullptr, &fb), "framebuffer");
                targets[e].emplace_back(view, fb);
            }
        }
        return {};
    }

    /// Draw `b` into swapchain image `idx` of `eye` and wait for the GPU.
    std::expected<void, std::string> draw(std::size_t eye, std::size_t idx, const Block& b) {
        std::memcpy(ubo[eye].ptr, b.data(), sizeof(float) * BLOCK_FLOATS);
        PIBRIDGE_VK(vkResetCommandBuffer(cmd, 0), "reset");
        VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO, nullptr,
                                    VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT, nullptr};
        PIBRIDGE_VK(vkBeginCommandBuffer(cmd, &bi), "begin");
        auto rbi = vk_make<VkRenderPassBeginInfo>(VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO);
        rbi.renderPass = render_pass;
        rbi.framebuffer = targets[eye][idx].second;
        rbi.renderArea = {{0, 0}, {size[0], size[1]}};
        vkCmdBeginRenderPass(cmd, &rbi, VK_SUBPASS_CONTENTS_INLINE);
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, 1, &sets[eye], 0, nullptr);
        vkCmdDraw(cmd, 3, 1, 0, 0);
        vkCmdEndRenderPass(cmd);
        PIBRIDGE_VK(vkEndCommandBuffer(cmd), "end");
        auto si = vk_make<VkSubmitInfo>(VK_STRUCTURE_TYPE_SUBMIT_INFO);
        si.commandBufferCount = 1;
        si.pCommandBuffers = &cmd;
        PIBRIDGE_VK(vkQueueSubmit(queue, 1, &si, fence), "submit");
        PIBRIDGE_VK(vkWaitForFences(device, 1, &fence, VK_TRUE, UINT64_MAX), "wait");
        PIBRIDGE_VK(vkResetFences(device, 1, &fence), "reset fence");
        return {};
    }

    void destroy() {
        if (device == VK_NULL_HANDLE) {
            return;
        }
        for (auto& t : targets) {
            for (auto& [view, fb] : t) {
                vkDestroyFramebuffer(device, fb, nullptr);
                vkDestroyImageView(device, view, nullptr);
            }
            t.clear();
        }
        for (auto& u : ubo) {
            if (u.mem != VK_NULL_HANDLE) {
                vkUnmapMemory(device, u.mem);
                vkFreeMemory(device, u.mem, nullptr);
            }
            if (u.buf != VK_NULL_HANDLE) {
                vkDestroyBuffer(device, u.buf, nullptr);
            }
            u = {};
        }
        vkDestroyDescriptorPool(device, desc_pool, nullptr);
        vkDestroyPipeline(device, pipeline, nullptr);
        vkDestroyPipelineLayout(device, layout, nullptr);
        vkDestroyDescriptorSetLayout(device, set_layout, nullptr);
        vkDestroyRenderPass(device, render_pass, nullptr);
        vkDestroyFence(device, fence, nullptr);
        vkDestroyCommandPool(device, pool, nullptr);
        device = VK_NULL_HANDLE;
    }
};

/// The head-locked text: its own swapchain, a staging buffer and a copy command.
struct HudQuad {
    VkDevice device = VK_NULL_HANDLE;
    VkQueue queue = VK_NULL_HANDLE;
    XrSwapchain swapchain = XR_NULL_HANDLE;
    std::vector<VkImage> images;
    std::vector<std::uint64_t> uploaded;
    VkBuffer staging = VK_NULL_HANDLE;
    VkDeviceMemory staging_mem = VK_NULL_HANDLE;
    std::uint8_t* staging_ptr = nullptr;
    std::size_t staging_len = 0;
    VkCommandPool pool = VK_NULL_HANDLE;
    VkCommandBuffer cmd = VK_NULL_HANDLE;
    VkFence fence = VK_NULL_HANDLE;

    /// Copy the staging buffer into `image` and leave it in COLOR_ATTACHMENT_OPTIMAL.
    std::expected<void, std::string> copy(VkImage image) {
        PIBRIDGE_VK(vkResetCommandBuffer(cmd, 0), "reset");
        VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO, nullptr,
                                    VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT, nullptr};
        PIBRIDGE_VK(vkBeginCommandBuffer(cmd, &bi), "begin");
        auto to_dst = vk_make<VkImageMemoryBarrier>(VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER);
        to_dst.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        to_dst.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        to_dst.image = image;
        to_dst.subresourceRange = COLOR_RANGE;
        to_dst.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        to_dst.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        to_dst.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0,
                             nullptr, 1, &to_dst);
        VkBufferImageCopy region{};
        region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        region.imageExtent = {hud::TEX_W, hud::TEX_H, 1};
        vkCmdCopyBufferToImage(cmd, staging, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
        VkImageMemoryBarrier to_color = to_dst;
        to_color.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        to_color.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        to_color.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        to_color.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0,
                             nullptr, 1, &to_color);
        PIBRIDGE_VK(vkEndCommandBuffer(cmd), "end");
        auto si = vk_make<VkSubmitInfo>(VK_STRUCTURE_TYPE_SUBMIT_INFO);
        si.commandBufferCount = 1;
        si.pCommandBuffers = &cmd;
        PIBRIDGE_VK(vkQueueSubmit(queue, 1, &si, fence), "submit");
        PIBRIDGE_VK(vkWaitForFences(device, 1, &fence, VK_TRUE, UINT64_MAX), "wait");
        PIBRIDGE_VK(vkResetFences(device, 1, &fence), "reset fence");
        return {};
    }

    void destroy() {
        if (device == VK_NULL_HANDLE) {
            return;
        }
        if (staging_mem != VK_NULL_HANDLE) {
            vkUnmapMemory(device, staging_mem);
            vkFreeMemory(device, staging_mem, nullptr);
        }
        vkDestroyBuffer(device, staging, nullptr);
        vkDestroyFence(device, fence, nullptr);
        vkDestroyCommandPool(device, pool, nullptr);
        device = VK_NULL_HANDLE;
    }
};

constexpr std::int64_t FMT_R8G8B8A8_SRGB = VK_FORMAT_R8G8B8A8_SRGB;
constexpr std::int64_t FMT_B8G8R8A8_SRGB = VK_FORMAT_B8G8R8A8_SRGB;
constexpr std::int64_t FMT_R8G8B8A8_UNORM = VK_FORMAT_R8G8B8A8_UNORM;
constexpr std::int64_t FMT_B8G8R8A8_UNORM = VK_FORMAT_B8G8R8A8_UNORM;

std::optional<std::int64_t> choose_format(const std::vector<std::int64_t>& offered) {
    for (const auto f : {FMT_R8G8B8A8_SRGB, FMT_B8G8R8A8_SRGB, FMT_R8G8B8A8_UNORM, FMT_B8G8R8A8_UNORM}) {
        if (std::ranges::find(offered, f) != offered.end()) {
            return f;
        }
    }
    return std::nullopt;
}

const char* format_name(std::int64_t f) {
    switch (f) {
    case FMT_R8G8B8A8_SRGB: return "R8G8B8A8_SRGB";
    case FMT_B8G8R8A8_SRGB: return "B8G8R8A8_SRGB";
    case FMT_R8G8B8A8_UNORM: return "R8G8B8A8_UNORM";
    case FMT_B8G8R8A8_UNORM: return "B8G8R8A8_UNORM";
    default: return "other";
    }
}

// ------------------------------------------------------------------------------------------------ the app

/// Every handle of the session, destroyed in reverse order.
struct Xr {
    XrInstance instance = XR_NULL_HANDLE;
    XrSystemId system = XR_NULL_SYSTEM_ID;
    XrSession session = XR_NULL_HANDLE;
    XrSpace view_space = XR_NULL_HANDLE;
    XrSpace local_space = XR_NULL_HANDLE;
    VkInstance vk_instance = VK_NULL_HANDLE;
    VkPhysicalDevice pd = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    VkQueue queue = VK_NULL_HANDLE;
    std::uint32_t qf = 0;
    VkPhysicalDeviceMemoryProperties mem_props{};
    std::array<XrSwapchain, 2> eye_sc{XR_NULL_HANDLE, XR_NULL_HANDLE};
    std::array<std::vector<VkImage>, 2> eye_images;
    MarksGpu gpu;
    HudQuad hud;
    // input
    XrActionSet set = XR_NULL_HANDLE;
    std::array<XrPath, 2> hands{XR_NULL_PATH, XR_NULL_PATH};
    XrAction next = XR_NULL_HANDLE;
    XrAction redo = XR_NULL_HANDLE;
    XrAction menu = XR_NULL_HANDLE;
    XrAction trigger = XR_NULL_HANDLE;
    std::array<bool, 4> prev{};
    // the runtime's eye gaze (end-to-end check)
    XrActionSet gaze_set = XR_NULL_HANDLE;
    XrAction gaze_action = XR_NULL_HANDLE;
    XrSpace gaze_space = XR_NULL_HANDLE;

    ~Xr() {
        if (device != VK_NULL_HANDLE) {
            vkDeviceWaitIdle(device);
        }
        gpu.destroy();
        hud.destroy();
        if (gaze_space != XR_NULL_HANDLE) {
            xrDestroySpace(gaze_space);
        }
        if (hud.swapchain != XR_NULL_HANDLE) {
            xrDestroySwapchain(hud.swapchain);
        }
        for (auto sc : eye_sc) {
            if (sc != XR_NULL_HANDLE) {
                xrDestroySwapchain(sc);
            }
        }
        if (local_space != XR_NULL_HANDLE) {
            xrDestroySpace(local_space);
        }
        if (view_space != XR_NULL_HANDLE) {
            xrDestroySpace(view_space);
        }
        if (session != XR_NULL_HANDLE) {
            xrDestroySession(session);
        }
        if (device != VK_NULL_HANDLE) {
            vkDestroyDevice(device, nullptr);
        }
        if (vk_instance != VK_NULL_HANDLE) {
            vkDestroyInstance(vk_instance, nullptr);
        }
        if (instance != XR_NULL_HANDLE) {
            xrDestroyInstance(instance);
        }
    }

    template <class F>
    std::expected<F, std::string> proc(const char* name) const {
        PFN_xrVoidFunction f = nullptr;
        const XrResult r = xrGetInstanceProcAddr(instance, name, &f);
        if (XR_FAILED(r) || f == nullptr) {
            return std::unexpected(xr_err(name, r));
        }
        return reinterpret_cast<F>(f);
    }

    std::expected<XrPath, std::string> path(const char* s) const {
        XrPath p = XR_NULL_PATH;
        PIBRIDGE_XR(xrStringToPath(instance, s, &p), s);
        return p;
    }
};

std::expected<XrAction, std::string> make_action(XrActionSet set, const char* name, XrActionType type,
                                                 std::span<const XrPath> sub) {
    auto ai = xr_make<XrActionCreateInfo>(XR_TYPE_ACTION_CREATE_INFO);
    std::snprintf(ai.actionName, sizeof ai.actionName, "%s", name);
    std::snprintf(ai.localizedActionName, sizeof ai.localizedActionName, "%s", name);
    ai.actionType = type;
    ai.countSubactionPaths = static_cast<std::uint32_t>(sub.size());
    ai.subactionPaths = sub.empty() ? nullptr : sub.data();
    XrAction a = XR_NULL_HANDLE;
    PIBRIDGE_XR(xrCreateAction(set, &ai, &a), name);
    return a;
}

std::expected<void, std::string> suggest(const Xr& x, const char* profile,
                                         const std::vector<std::pair<XrAction, const char*>>& b) {
    std::vector<XrActionSuggestedBinding> v;
    for (const auto& [a, p] : b) {
        auto path = x.path(p);
        if (!path) {
            return std::unexpected(path.error());
        }
        v.push_back({a, *path});
    }
    auto prof = x.path(profile);
    if (!prof) {
        return std::unexpected(prof.error());
    }
    auto s = xr_make<XrInteractionProfileSuggestedBinding>(XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING);
    s.interactionProfile = *prof;
    s.countSuggestedBindings = static_cast<std::uint32_t>(v.size());
    s.suggestedBindings = v.data();
    PIBRIDGE_XR(xrSuggestInteractionProfileBindings(x.instance, &s), profile);
    return {};
}

/// Controller buttons (A next, B back, menu save, trigger start).
std::expected<void, std::string> make_controls(Xr& x) {
    auto si = xr_make<XrActionSetCreateInfo>(XR_TYPE_ACTION_SET_CREATE_INFO);
    std::snprintf(si.actionSetName, sizeof si.actionSetName, "eyecal");
    std::snprintf(si.localizedActionSetName, sizeof si.localizedActionSetName, "eyecal");
    PIBRIDGE_XR(xrCreateActionSet(x.instance, &si, &x.set), "action set");
    auto l = x.path("/user/hand/left");
    auto r = x.path("/user/hand/right");
    if (!l || !r) {
        return std::unexpected(std::string("hand paths"));
    }
    x.hands = {*l, *r};
    auto next = make_action(x.set, "next", XR_ACTION_TYPE_BOOLEAN_INPUT, x.hands);
    auto redo = make_action(x.set, "redo", XR_ACTION_TYPE_BOOLEAN_INPUT, x.hands);
    auto menu = make_action(x.set, "save", XR_ACTION_TYPE_BOOLEAN_INPUT, x.hands);
    auto trig = make_action(x.set, "trigger", XR_ACTION_TYPE_FLOAT_INPUT, x.hands);
    if (!next || !redo || !menu || !trig) {
        return std::unexpected(!next ? next.error() : !redo ? redo.error() : !menu ? menu.error() : trig.error());
    }
    x.next = *next;
    x.redo = *redo;
    x.menu = *menu;
    x.trigger = *trig;
    // Oculus Touch (Monado and most runtimes): A/X next, B/Y back, menu save, the triggers
    if (auto s = suggest(x, "/interaction_profiles/oculus/touch_controller",
                         {{x.next, "/user/hand/left/input/x/click"},
                          {x.next, "/user/hand/right/input/a/click"},
                          {x.redo, "/user/hand/left/input/y/click"},
                          {x.redo, "/user/hand/right/input/b/click"},
                          {x.menu, "/user/hand/left/input/menu/click"},
                          {x.trigger, "/user/hand/left/input/trigger/value"},
                          {x.trigger, "/user/hand/right/input/trigger/value"}});
        !s) {
        std::printf("Touch controller bindings not taken: %s\n", s.error().c_str());
    }
    // Valve Index controllers (SteamVR with the Lighthouse setup): A start / next, B stop / save / exit, the triggers
    if (auto s = suggest(x, "/interaction_profiles/valve/index_controller",
                         {{x.next, "/user/hand/left/input/a/click"},
                          {x.next, "/user/hand/right/input/a/click"},
                          {x.redo, "/user/hand/left/input/b/click"},
                          {x.redo, "/user/hand/right/input/b/click"},
                          {x.trigger, "/user/hand/left/input/trigger/value"},
                          {x.trigger, "/user/hand/right/input/trigger/value"}});
        !s) {
        std::printf("Index controller bindings not taken: %s\n", s.error().c_str());
    }
    return {};
}

/// The runtime's eye gaze action (its space is only created for the end-to-end check).
std::expected<void, std::string> make_gaze(Xr& x) {
    auto si = xr_make<XrActionSetCreateInfo>(XR_TYPE_ACTION_SET_CREATE_INFO);
    std::snprintf(si.actionSetName, sizeof si.actionSetName, "gaze");
    std::snprintf(si.localizedActionSetName, sizeof si.localizedActionSetName, "gaze");
    PIBRIDGE_XR(xrCreateActionSet(x.instance, &si, &x.gaze_set), "gaze set");
    auto a = make_action(x.gaze_set, "gaze_pose", XR_ACTION_TYPE_POSE_INPUT, {});
    if (!a) {
        return std::unexpected(a.error());
    }
    x.gaze_action = *a;
    return suggest(x, "/interaction_profiles/ext/eye_gaze_interaction", {{x.gaze_action, "/user/eyes_ext/input/gaze_ext/pose"}});
}

/// Commands from button presses since the last call (edges only).
std::vector<Cmd> poll_controls(Xr& x) {
    std::vector<XrActiveActionSet> active = {{x.set, XR_NULL_PATH}};
    if (x.gaze_set != XR_NULL_HANDLE) {
        active.push_back({x.gaze_set, XR_NULL_PATH});
    }
    auto sync = xr_make<XrActionsSyncInfo>(XR_TYPE_ACTIONS_SYNC_INFO);
    sync.countActiveActionSets = static_cast<std::uint32_t>(active.size());
    sync.activeActionSets = active.data();
    if (XR_FAILED(xrSyncActions(x.session, &sync))) {
        return {};
    }
    std::array<bool, 4> now{};
    for (const XrPath h : x.hands) {
        const auto boolean = [&](XrAction a) {
            XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO, nullptr, a, h};
            auto st = xr_make<XrActionStateBoolean>(XR_TYPE_ACTION_STATE_BOOLEAN);
            return XR_SUCCEEDED(xrGetActionStateBoolean(x.session, &gi, &st)) && st.currentState == XR_TRUE;
        };
        now[0] = now[0] || boolean(x.next);
        now[1] = now[1] || boolean(x.redo);
        now[2] = now[2] || boolean(x.menu);
        XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO, nullptr, x.trigger, h};
        auto st = xr_make<XrActionStateFloat>(XR_TYPE_ACTION_STATE_FLOAT);
        now[3] = now[3] || (XR_SUCCEEDED(xrGetActionStateFloat(x.session, &gi, &st)) && st.currentState > 0.7F);
    }
    std::vector<Cmd> out;
    const std::array<Cmd, 4> cmds = {Cmd::Next, Cmd::Redo, Cmd::Accept, Cmd::Start};
    for (std::size_t i = 0; i < 4; i++) {
        if (now[i] && !x.prev[i]) {
            out.push_back(cmds[i]);
        }
    }
    x.prev = now;
    return out;
}

std::expected<void, std::string> setup(Xr& x, const XrOpts& opts, bool& gaze_ext, std::int64_t& format,
                                       std::array<std::uint32_t, 2>& size) {
    std::uint32_t n = 0;
    PIBRIDGE_XR(xrEnumerateInstanceExtensionProperties(nullptr, 0, &n, nullptr),
             "cannot load the OpenXR runtime (is SteamVR running and the active OpenXR runtime?)");
    std::vector<XrExtensionProperties> props(n, xr_make<XrExtensionProperties>(XR_TYPE_EXTENSION_PROPERTIES));
    PIBRIDGE_XR(xrEnumerateInstanceExtensionProperties(nullptr, n, &n, props.data()), "extensions");
    const auto has = [&](const char* e) {
        return std::ranges::any_of(props, [&](const XrExtensionProperties& p) { return std::strcmp(p.extensionName, e) == 0; });
    };
    if (!has(XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME)) {
        return std::unexpected(std::string("the OpenXR runtime does not offer XR_KHR_vulkan_enable2"));
    }
    gaze_ext = has(XR_EXT_EYE_GAZE_INTERACTION_EXTENSION_NAME);
    std::vector<const char*> exts = {XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME};
    if (gaze_ext) {
        exts.push_back(XR_EXT_EYE_GAZE_INTERACTION_EXTENSION_NAME);
    }
    auto ici = xr_make<XrInstanceCreateInfo>(XR_TYPE_INSTANCE_CREATE_INFO);
    std::snprintf(ici.applicationInfo.applicationName, sizeof ici.applicationInfo.applicationName, "pibridge-eyecal");
    std::snprintf(ici.applicationInfo.engineName, sizeof ici.applicationInfo.engineName, "pibridge-eyecal");
    ici.applicationInfo.applicationVersion = 1;
    ici.applicationInfo.engineVersion = 1;
    ici.applicationInfo.apiVersion = XR_MAKE_VERSION(1, 0, 0);
    ici.enabledExtensionCount = static_cast<std::uint32_t>(exts.size());
    ici.enabledExtensionNames = exts.data();
    PIBRIDGE_XR(xrCreateInstance(&ici, &x.instance), "xrCreateInstance (is SteamVR running?)");
    XrSystemGetInfo sgi{XR_TYPE_SYSTEM_GET_INFO, nullptr, XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY};
    PIBRIDGE_XR(xrGetSystem(x.instance, &sgi, &x.system), "no head mounted display");
    auto reqs_fn = x.proc<PFN_xrGetVulkanGraphicsRequirements2KHR>("xrGetVulkanGraphicsRequirements2KHR");
    auto inst_fn = x.proc<PFN_xrCreateVulkanInstanceKHR>("xrCreateVulkanInstanceKHR");
    auto pd_fn = x.proc<PFN_xrGetVulkanGraphicsDevice2KHR>("xrGetVulkanGraphicsDevice2KHR");
    auto dev_fn = x.proc<PFN_xrCreateVulkanDeviceKHR>("xrCreateVulkanDeviceKHR");
    if (!reqs_fn || !inst_fn || !pd_fn || !dev_fn) {
        return std::unexpected(std::string("the runtime lacks the XR_KHR_vulkan_enable2 functions"));
    }
    auto reqs = xr_make<XrGraphicsRequirementsVulkan2KHR>(XR_TYPE_GRAPHICS_REQUIREMENTS_VULKAN2_KHR);
    PIBRIDGE_XR((*reqs_fn)(x.instance, x.system, &reqs), "graphics requirements");
    if (XR_MAKE_VERSION(1, 1, 0) < reqs.minApiVersionSupported) {
        return std::unexpected(std::string("the runtime wants a newer Vulkan than 1.1"));
    }
    auto app = vk_make<VkApplicationInfo>(VK_STRUCTURE_TYPE_APPLICATION_INFO);
    app.pApplicationName = "pibridge-eyecal";
    app.apiVersion = VK_MAKE_API_VERSION(0, 1, 1, 0);
    auto vci = vk_make<VkInstanceCreateInfo>(VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO);
    vci.pApplicationInfo = &app;
    auto xvi = xr_make<XrVulkanInstanceCreateInfoKHR>(XR_TYPE_VULKAN_INSTANCE_CREATE_INFO_KHR);
    xvi.systemId = x.system;
    xvi.pfnGetInstanceProcAddr = &vkGetInstanceProcAddr;
    xvi.vulkanCreateInfo = &vci;
    VkResult vr = VK_SUCCESS;
    PIBRIDGE_XR((*inst_fn)(x.instance, &xvi, &x.vk_instance, &vr), "xrCreateVulkanInstanceKHR");
    PIBRIDGE_VK(vr, "vkCreateInstance");
    auto gdi = xr_make<XrVulkanGraphicsDeviceGetInfoKHR>(XR_TYPE_VULKAN_GRAPHICS_DEVICE_GET_INFO_KHR);
    gdi.systemId = x.system;
    gdi.vulkanInstance = x.vk_instance;
    PIBRIDGE_XR((*pd_fn)(x.instance, &gdi, &x.pd), "xrGetVulkanGraphicsDevice2KHR");
    std::uint32_t nq = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(x.pd, &nq, nullptr);
    std::vector<VkQueueFamilyProperties> qs(nq);
    vkGetPhysicalDeviceQueueFamilyProperties(x.pd, &nq, qs.data());
    bool found = false;
    for (std::uint32_t i = 0; i < nq; i++) {
        if ((qs[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0) {
            x.qf = i;
            found = true;
            break;
        }
    }
    if (!found) {
        return std::unexpected(std::string("no graphics queue"));
    }
    const float prio = 1.0F;
    VkDeviceQueueCreateInfo qi{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO, nullptr, 0, x.qf, 1, &prio};
    auto dci = vk_make<VkDeviceCreateInfo>(VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO);
    dci.queueCreateInfoCount = 1;
    dci.pQueueCreateInfos = &qi;
    auto xdi = xr_make<XrVulkanDeviceCreateInfoKHR>(XR_TYPE_VULKAN_DEVICE_CREATE_INFO_KHR);
    xdi.systemId = x.system;
    xdi.pfnGetInstanceProcAddr = &vkGetInstanceProcAddr;
    xdi.vulkanPhysicalDevice = x.pd;
    xdi.vulkanCreateInfo = &dci;
    PIBRIDGE_XR((*dev_fn)(x.instance, &xdi, &x.device, &vr), "xrCreateVulkanDeviceKHR");
    PIBRIDGE_VK(vr, "vkCreateDevice");
    vkGetDeviceQueue(x.device, x.qf, 0, &x.queue);
    vkGetPhysicalDeviceMemoryProperties(x.pd, &x.mem_props);
    auto binding = xr_make<XrGraphicsBindingVulkan2KHR>(XR_TYPE_GRAPHICS_BINDING_VULKAN2_KHR);
    binding.instance = x.vk_instance;
    binding.physicalDevice = x.pd;
    binding.device = x.device;
    binding.queueFamilyIndex = x.qf;
    binding.queueIndex = 0;
    XrSessionCreateInfo sci{XR_TYPE_SESSION_CREATE_INFO, &binding, 0, x.system};
    PIBRIDGE_XR(xrCreateSession(x.instance, &sci, &x.session), "xrCreateSession");
    XrReferenceSpaceCreateInfo rsi{XR_TYPE_REFERENCE_SPACE_CREATE_INFO, nullptr, XR_REFERENCE_SPACE_TYPE_VIEW, IDENTITY_POSE};
    PIBRIDGE_XR(xrCreateReferenceSpace(x.session, &rsi, &x.view_space), "view space");
    rsi.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    if (XR_FAILED(xrCreateReferenceSpace(x.session, &rsi, &x.local_space))) {
        x.local_space = XR_NULL_HANDLE; // no head motion check then
    }
    if (auto c = make_controls(x); !c) {
        return std::unexpected(c.error());
    }
    if (gaze_ext && !make_gaze(x)) {
        x.gaze_set = XR_NULL_HANDLE;
    }
    std::vector<XrActionSet> sets = {x.set};
    if (x.gaze_set != XR_NULL_HANDLE) {
        sets.push_back(x.gaze_set);
    }
    auto att = xr_make<XrSessionActionSetsAttachInfo>(XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO);
    att.countActionSets = static_cast<std::uint32_t>(sets.size());
    att.actionSets = sets.data();
    PIBRIDGE_XR(xrAttachSessionActionSets(x.session, &att), "attach action sets");
    std::uint32_t nv = 0;
    PIBRIDGE_XR(xrEnumerateViewConfigurationViews(x.instance, x.system, VIEW_TYPE, 0, &nv, nullptr), "views");
    std::vector<XrViewConfigurationView> views(nv, xr_make<XrViewConfigurationView>(XR_TYPE_VIEW_CONFIGURATION_VIEW));
    PIBRIDGE_XR(xrEnumerateViewConfigurationViews(x.instance, x.system, VIEW_TYPE, nv, &nv, views.data()), "views");
    if (nv != 2) {
        return std::unexpected(std::format("expected 2 views, got {}", nv));
    }
    const std::uint32_t div = std::max<std::uint32_t>(opts.res_div, 1);
    size = {std::max<std::uint32_t>(views[0].recommendedImageRectWidth / div, 16),
            std::max<std::uint32_t>(views[0].recommendedImageRectHeight / div, 16)};
    std::uint32_t nf = 0;
    PIBRIDGE_XR(xrEnumerateSwapchainFormats(x.session, 0, &nf, nullptr), "swapchain formats");
    std::vector<std::int64_t> offered(nf);
    PIBRIDGE_XR(xrEnumerateSwapchainFormats(x.session, nf, &nf, offered.data()), "swapchain formats");
    const auto f = choose_format(offered);
    if (!f) {
        return std::unexpected(std::string("no 8-bit RGBA/BGRA swapchain format"));
    }
    format = *f;
    auto ip = xr_make<XrInstanceProperties>(XR_TYPE_INSTANCE_PROPERTIES);
    (void)xrGetInstanceProperties(x.instance, &ip);
    std::printf("render %ux%u per eye (%s), runtime %s\n", size[0], size[1], format_name(format), ip.runtimeName);
    const auto swapchain = [&](std::uint32_t w, std::uint32_t h, XrSwapchainUsageFlags usage, XrSwapchain& sc,
                               std::vector<VkImage>& imgs) -> std::expected<void, std::string> {
        auto ci = xr_make<XrSwapchainCreateInfo>(XR_TYPE_SWAPCHAIN_CREATE_INFO);
        ci.usageFlags = usage;
        ci.format = format;
        ci.sampleCount = 1;
        ci.width = w;
        ci.height = h;
        ci.faceCount = 1;
        ci.arraySize = 1;
        ci.mipCount = 1;
        PIBRIDGE_XR(xrCreateSwapchain(x.session, &ci, &sc), "swapchain");
        std::uint32_t ni = 0;
        PIBRIDGE_XR(xrEnumerateSwapchainImages(sc, 0, &ni, nullptr), "swapchain images");
        std::vector<XrSwapchainImageVulkan2KHR> v(ni, xr_make<XrSwapchainImageVulkan2KHR>(XR_TYPE_SWAPCHAIN_IMAGE_VULKAN2_KHR));
        PIBRIDGE_XR(xrEnumerateSwapchainImages(sc, ni, &ni, reinterpret_cast<XrSwapchainImageBaseHeader*>(v.data())),
                 "swapchain images");
        for (const auto& i : v) {
            imgs.push_back(i.image);
        }
        return {};
    };
    for (std::size_t e = 0; e < 2; e++) {
        if (auto r = swapchain(size[0], size[1], XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT,
                               x.eye_sc[e], x.eye_images[e]);
            !r) {
            return std::unexpected("eye " + r.error());
        }
    }
    if (auto r = x.gpu.create(x.device, x.queue, x.qf, x.mem_props, static_cast<VkFormat>(format), size, x.eye_images); !r) {
        return std::unexpected(r.error());
    }
    // the text quad
    auto& h = x.hud;
    h.device = x.device;
    h.queue = x.queue;
    if (auto r = swapchain(hud::TEX_W, hud::TEX_H,
                           XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT |
                               XR_SWAPCHAIN_USAGE_SAMPLED_BIT,
                           h.swapchain, h.images);
        !r) {
        return std::unexpected("text quad " + r.error());
    }
    h.uploaded.assign(h.images.size(), 0);
    h.staging_len = static_cast<std::size_t>(hud::TEX_W) * hud::TEX_H * 4;
    auto bi = vk_make<VkBufferCreateInfo>(VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO);
    bi.size = h.staging_len;
    bi.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    PIBRIDGE_VK(vkCreateBuffer(x.device, &bi, nullptr, &h.staging), "text staging");
    VkMemoryRequirements req;
    vkGetBufferMemoryRequirements(x.device, h.staging, &req);
    const auto mt = memory_type(x.mem_props, req.memoryTypeBits,
                                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    if (!mt) {
        return std::unexpected(std::string("no host visible memory for the text"));
    }
    VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO, nullptr, req.size, *mt};
    PIBRIDGE_VK(vkAllocateMemory(x.device, &mai, nullptr, &h.staging_mem), "text memory");
    PIBRIDGE_VK(vkBindBufferMemory(x.device, h.staging, h.staging_mem, 0), "text bind");
    void* ptr = nullptr;
    PIBRIDGE_VK(vkMapMemory(x.device, h.staging_mem, 0, VK_WHOLE_SIZE, 0, &ptr), "text map");
    h.staging_ptr = static_cast<std::uint8_t*>(ptr);
    VkCommandPoolCreateInfo cpi{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO, nullptr,
                                VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT, x.qf};
    PIBRIDGE_VK(vkCreateCommandPool(x.device, &cpi, nullptr, &h.pool), "command pool");
    VkCommandBufferAllocateInfo cba{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO, nullptr, h.pool,
                                    VK_COMMAND_BUFFER_LEVEL_PRIMARY, 1};
    PIBRIDGE_VK(vkAllocateCommandBuffers(x.device, &cba, &h.cmd), "command buffer");
    auto fci = vk_make<VkFenceCreateInfo>(VK_STRUCTURE_TYPE_FENCE_CREATE_INFO);
    PIBRIDGE_VK(vkCreateFence(x.device, &fci, nullptr, &h.fence), "fence");
    return {};
}

Vec3 forward_q(const XrQuaternionf& q) { return forward_of({q.x, q.y, q.z, q.w}); }

} // namespace

std::expected<void, std::string> run_xr(Runner& runner, const XrOpts& opts, const std::atomic<bool>& stop) {
    Xr x;
    bool gaze_ext = false;
    std::int64_t format = 0;
    std::array<std::uint32_t, 2> size{};
    if (auto r = setup(x, opts, gaze_ext, format, size); !r) {
        return std::unexpected(r.error());
    }
    bool service = false;
    runner.set_service([&x, &service](bool start) {
        if (!start) {
            if (x.gaze_space != XR_NULL_HANDLE) {
                xrDestroySpace(x.gaze_space); // the runtime lets go of the eye tracker
                x.gaze_space = XR_NULL_HANDLE;
            }
            service = false;
            return true;
        }
        if (x.gaze_action == XR_NULL_HANDLE) {
            return false;
        }
        XrActionSpaceCreateInfo asi{XR_TYPE_ACTION_SPACE_CREATE_INFO, nullptr, x.gaze_action, XR_NULL_PATH, IDENTITY_POSE};
        if (XR_FAILED(xrCreateActionSpace(x.session, &asi, &x.gaze_space))) {
            return false;
        }
        service = true;
        return true;
    });
    const RawTerminal term;
    Keyboard keys;
    std::printf("keys: space/a next  b back/stop  y save  p pursuit  c cursor  s runtime check  q quit   (controllers: A "
                "next, B back / stop / save and exit, trigger start)\n");
    std::fflush(stdout);
    runner.begin();
    const bool srgb = format == FMT_R8G8B8A8_SRGB || format == FMT_B8G8R8A8_SRGB;
    const bool bgra = format == FMT_B8G8R8A8_SRGB || format == FMT_B8G8R8A8_UNORM;
    bool running = false;
    std::optional<std::chrono::steady_clock::time_point> exit_at;
    std::optional<std::vector<std::string>> hud_sig;
    std::uint64_t hud_gen = 0;
    std::uint64_t frames = 0;
    std::uint64_t slow = 0;
    const auto t_start = std::chrono::steady_clock::now();
    const auto request_exit = [&] {
        if (!exit_at) {
            (void)xrRequestExitSession(x.session);
            exit_at = std::chrono::steady_clock::now();
        }
    };
    while (true) {
        if (stop.load() || runner.quit_requested()) {
            request_exit();
        }
        auto ev = xr_make<XrEventDataBuffer>(XR_TYPE_EVENT_DATA_BUFFER);
        bool leave = false;
        while (xrPollEvent(x.instance, &ev) == XR_SUCCESS) {
            if (ev.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
                const auto& e = *reinterpret_cast<const XrEventDataSessionStateChanged*>(&ev);
                if (e.state == XR_SESSION_STATE_READY) {
                    XrSessionBeginInfo bi{XR_TYPE_SESSION_BEGIN_INFO, nullptr, VIEW_TYPE};
                    PIBRIDGE_XR(xrBeginSession(x.session, &bi), "xrBeginSession");
                    running = true;
                } else if (e.state == XR_SESSION_STATE_STOPPING) {
                    PIBRIDGE_XR(xrEndSession(x.session), "xrEndSession");
                    running = false;
                } else if (e.state == XR_SESSION_STATE_EXITING || e.state == XR_SESSION_STATE_LOSS_PENDING) {
                    leave = true;
                }
            } else if (ev.type == XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING) {
                leave = true;
            }
            ev = xr_make<XrEventDataBuffer>(XR_TYPE_EVENT_DATA_BUFFER);
        }
        if (leave) {
            break;
        }
        if (!running) {
            if (exit_at && std::chrono::steady_clock::now() - *exit_at > 3s) {
                break;
            }
            std::this_thread::sleep_for(20ms);
            continue;
        }
        auto fwi = xr_make<XrFrameWaitInfo>(XR_TYPE_FRAME_WAIT_INFO);
        auto fs = xr_make<XrFrameState>(XR_TYPE_FRAME_STATE);
        PIBRIDGE_XR(xrWaitFrame(x.session, &fwi, &fs), "xrWaitFrame");
        auto fbi = xr_make<XrFrameBeginInfo>(XR_TYPE_FRAME_BEGIN_INFO);
        PIBRIDGE_XR(xrBeginFrame(x.session, &fbi), "xrBeginFrame");
        const auto t_frame = std::chrono::steady_clock::now();
        XrViewLocateInfo vli{XR_TYPE_VIEW_LOCATE_INFO, nullptr, VIEW_TYPE, fs.predictedDisplayTime, x.view_space};
        auto vs = xr_make<XrViewState>(XR_TYPE_VIEW_STATE);
        std::array<XrView, 2> xv{xr_make<XrView>(XR_TYPE_VIEW), xr_make<XrView>(XR_TYPE_VIEW)};
        std::uint32_t nv = 0;
        PIBRIDGE_XR(xrLocateViews(x.session, &vli, &vs, 2, &nv, xv.data()), "xrLocateViews");
        std::optional<std::array<double, 4>> head;
        if (x.local_space != XR_NULL_HANDLE) {
            auto loc = xr_make<XrSpaceLocation>(XR_TYPE_SPACE_LOCATION);
            if (XR_SUCCEEDED(xrLocateSpace(x.view_space, x.local_space, fs.predictedDisplayTime, &loc)) &&
                (loc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0) {
                const auto& q = loc.pose.orientation;
                head = std::array<double, 4>{q.x, q.y, q.z, q.w};
            }
        }
        std::array<Vec3, 2> eye_pos{};
        for (std::size_t i = 0; i < 2; i++) {
            const auto& p = xv[i].pose.position;
            eye_pos[i] = {p.x, p.y, p.z};
        }
        // input
        std::vector<Cmd> cmds = poll_controls(x);
        for (const char k : keys.take()) {
            const char c = static_cast<char>(k >= 'A' && k <= 'Z' ? k - 'A' + 'a' : k);
            if (const auto m = key_command(c)) {
                if (*m) {
                    cmds.push_back(**m);
                } else {
                    request_exit();
                }
            }
        }
        std::optional<Vec3> service_dir;
        if (service && x.gaze_space != XR_NULL_HANDLE) {
            auto loc = xr_make<XrSpaceLocation>(XR_TYPE_SPACE_LOCATION);
            if (XR_SUCCEEDED(xrLocateSpace(x.gaze_space, x.view_space, fs.predictedDisplayTime, &loc)) &&
                (loc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0) {
                service_dir = forward_q(loc.pose.orientation);
            }
        }
        const double now = static_cast<double>(pibridge::eyemath::monotonic_ns()) * 1e-9;
        runner.step(now, head, eye_pos, service_dir, cmds);
        // draw
        const Scene scene = runner.procedure().scene();
        std::vector<std::string> hud_lines = runner.hud_prefix();
        hud_lines.insert(hud_lines.end(), scene.hud.begin(), scene.hud.end());
        std::vector<XrCompositionLayerBaseHeader*> layers;
        std::array<XrCompositionLayerProjectionView, 2> pv{};
        auto proj = xr_make<XrCompositionLayerProjection>(XR_TYPE_COMPOSITION_LAYER_PROJECTION);
        auto quad = xr_make<XrCompositionLayerQuad>(XR_TYPE_COMPOSITION_LAYER_QUAD);
        if (fs.shouldRender == XR_TRUE) {
            for (std::size_t e = 0; e < 2; e++) {
                const auto& q = xv[e].pose.orientation;
                const auto& f = xv[e].fov;
                EyeView v;
                v.rot = quat_to_mat({q.x, q.y, q.z, q.w});
                v.pos = {xv[e].pose.position.x, xv[e].pose.position.y, xv[e].pose.position.z};
                v.tan = {std::tan(-f.angleLeft), std::tan(f.angleRight), std::tan(f.angleUp), std::tan(-f.angleDown)};
                v.size = size;
                const Block b = block(scene, v);
                std::uint32_t idx = 0;
                auto ai = xr_make<XrSwapchainImageAcquireInfo>(XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO);
                PIBRIDGE_XR(xrAcquireSwapchainImage(x.eye_sc[e], &ai, &idx), "acquire");
                XrSwapchainImageWaitInfo wi{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO, nullptr, XR_INFINITE_DURATION};
                PIBRIDGE_XR(xrWaitSwapchainImage(x.eye_sc[e], &wi), "wait image");
                if (auto r = x.gpu.draw(e, idx, b); !r) {
                    return std::unexpected(r.error());
                }
                auto ri = xr_make<XrSwapchainImageReleaseInfo>(XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO);
                PIBRIDGE_XR(xrReleaseSwapchainImage(x.eye_sc[e], &ri), "release");
                pv[e] = xr_make<XrCompositionLayerProjectionView>(XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW);
                pv[e].pose = xv[e].pose;
                pv[e].fov = xv[e].fov;
                pv[e].subImage.swapchain = x.eye_sc[e];
                pv[e].subImage.imageRect = {{0, 0}, {static_cast<std::int32_t>(size[0]), static_cast<std::int32_t>(size[1])}};
                pv[e].subImage.imageArrayIndex = 0;
            }
            proj.space = x.view_space;
            proj.viewCount = 2;
            proj.views = pv.data();
            layers.push_back(reinterpret_cast<XrCompositionLayerBaseHeader*>(&proj));
            if (!hud_lines.empty()) {
                auto& h = x.hud;
                if (hud_sig != hud_lines) {
                    const auto img = hud::render(hud_lines);
                    std::span<std::uint8_t> out(h.staging_ptr, h.staging_len);
                    hud::to_upload(img, srgb ? hud::Encoding::Srgb : hud::Encoding::Linear, bgra, out);
                    hud::alpha_from_black(out);
                    hud_sig = hud_lines;
                    hud_gen++;
                }
                std::uint32_t idx = 0;
                auto ai = xr_make<XrSwapchainImageAcquireInfo>(XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO);
                PIBRIDGE_XR(xrAcquireSwapchainImage(h.swapchain, &ai, &idx), "acquire text");
                XrSwapchainImageWaitInfo wi{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO, nullptr, XR_INFINITE_DURATION};
                PIBRIDGE_XR(xrWaitSwapchainImage(h.swapchain, &wi), "wait text");
                if (h.uploaded[idx] != hud_gen) {
                    if (auto r = h.copy(h.images[idx]); !r) {
                        return std::unexpected(r.error());
                    }
                    h.uploaded[idx] = hud_gen;
                }
                auto ri = xr_make<XrSwapchainImageReleaseInfo>(XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO);
                PIBRIDGE_XR(xrReleaseSwapchainImage(h.swapchain, &ri), "release text");
                const auto pos = hud::SPEC.position();
                quad.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;
                quad.space = x.view_space;
                quad.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
                quad.pose = {{0.0F, 0.0F, 0.0F, 1.0F}, {pos[0], pos[1], pos[2]}};
                quad.size = {hud::SPEC.width, hud::SPEC.height};
                quad.subImage.swapchain = h.swapchain;
                quad.subImage.imageRect = {{0, 0}, {static_cast<std::int32_t>(hud::TEX_W), static_cast<std::int32_t>(hud::TEX_H)}};
                layers.push_back(reinterpret_cast<XrCompositionLayerBaseHeader*>(&quad));
            }
        }
        auto fei = xr_make<XrFrameEndInfo>(XR_TYPE_FRAME_END_INFO);
        fei.displayTime = fs.predictedDisplayTime;
        fei.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
        fei.layerCount = static_cast<std::uint32_t>(layers.size());
        fei.layers = layers.data();
        PIBRIDGE_XR(xrEndFrame(x.session, &fei), "xrEndFrame");
        frames++;
        if (std::chrono::steady_clock::now() - t_frame > 8ms) {
            slow++;
        }
        if (frames % 3600 == 0) {
            std::printf("%llu frames in %.0f s, %llu took more than 8 ms of CPU+GPU\n", static_cast<unsigned long long>(frames),
                        std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count(),
                        static_cast<unsigned long long>(slow));
            std::fflush(stdout);
        }
    }
    runner.finish();
    std::printf("%llu frames, %llu slow\n", static_cast<unsigned long long>(frames), static_cast<unsigned long long>(slow));
    return {};
}

} // namespace pibridge::eyecal
