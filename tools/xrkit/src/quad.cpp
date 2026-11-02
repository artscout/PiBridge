// SPDX-License-Identifier: MIT
#include "pibridge/xrkit/quad.h"

#include <cstring>
#include <vector>

namespace pibridge::xrkit {

namespace {

std::optional<std::uint32_t> hostType(const VkPhysicalDeviceMemoryProperties& p, std::uint32_t bits) {
    const VkMemoryPropertyFlags want = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    for (std::uint32_t i = 0; i < p.memoryTypeCount; i++) {
        if ((bits & (1U << i)) != 0 && (p.memoryTypes[i].propertyFlags & want) == want) {
            return i;
        }
    }
    return std::nullopt;
}

} // namespace

void QuadImage::destroy() {
    if (device_ != VK_NULL_HANDLE) {
        vkDeviceWaitIdle(device_);
        if (fence_ != VK_NULL_HANDLE) {
            vkDestroyFence(device_, fence_, nullptr);
        }
        if (pool_ != VK_NULL_HANDLE) {
            vkDestroyCommandPool(device_, pool_, nullptr);
        }
        if (staging_ != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, staging_, nullptr);
        }
        if (smem_ != VK_NULL_HANDLE) {
            vkFreeMemory(device_, smem_, nullptr);
        }
    }
    if (sc_ != XR_NULL_HANDLE) {
        xrDestroySwapchain(sc_);
    }
    fence_ = VK_NULL_HANDLE;
    pool_ = VK_NULL_HANDLE;
    cmd_ = VK_NULL_HANDLE;
    staging_ = VK_NULL_HANDLE;
    smem_ = VK_NULL_HANDLE;
    sptr_ = nullptr;
    sc_ = XR_NULL_HANDLE;
    images_.clear();
    w_ = h_ = 0;
    released_ = false;
}

std::expected<void, std::string> QuadImage::make(Session& x, std::uint32_t w, std::uint32_t h) {
    destroy();
    device_ = x.device;
    std::uint32_t nf = 0;
    PIBRIDGE_XR(xrEnumerateSwapchainFormats(x.session, 0, &nf, nullptr), "quad formats");
    std::vector<std::int64_t> offered(nf);
    PIBRIDGE_XR(xrEnumerateSwapchainFormats(x.session, nf, &nf, offered.data()), "quad formats");
    std::int64_t format = 0;
    for (const std::int64_t f : offered) {
        if (f == VK_FORMAT_B8G8R8A8_SRGB) {
            format = f;
            swapRB_ = false;
            break;
        }
        if (f == VK_FORMAT_R8G8B8A8_SRGB && format == 0) {
            format = f;
            swapRB_ = true;
        }
    }
    if (format == 0) {
        return std::unexpected(std::string("no 8-bit sRGB swapchain format for a quad"));
    }
    auto ci = xr_make<XrSwapchainCreateInfo>(XR_TYPE_SWAPCHAIN_CREATE_INFO);
    ci.usageFlags = XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT | XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;
    ci.format = format;
    ci.sampleCount = 1;
    ci.width = w;
    ci.height = h;
    ci.faceCount = 1;
    ci.arraySize = 1;
    ci.mipCount = 1;
    PIBRIDGE_XR(xrCreateSwapchain(x.session, &ci, &sc_), "quad swapchain");
    std::uint32_t ni = 0;
    PIBRIDGE_XR(xrEnumerateSwapchainImages(sc_, 0, &ni, nullptr), "quad images");
    std::vector<XrSwapchainImageVulkan2KHR> v(ni, xr_make<XrSwapchainImageVulkan2KHR>(XR_TYPE_SWAPCHAIN_IMAGE_VULKAN2_KHR));
    PIBRIDGE_XR(xrEnumerateSwapchainImages(sc_, ni, &ni, reinterpret_cast<XrSwapchainImageBaseHeader*>(v.data())), "quad images");
    for (const auto& i : v) {
        images_.push_back(i.image);
    }
    // the staging buffer (host visible), a command buffer and its fence
    auto bi = vk_make<VkBufferCreateInfo>(VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO);
    bi.size = static_cast<VkDeviceSize>(w) * h * 4;
    bi.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    PIBRIDGE_VK(vkCreateBuffer(device_, &bi, nullptr, &staging_), "quad staging");
    VkMemoryRequirements req;
    vkGetBufferMemoryRequirements(device_, staging_, &req);
    const auto mt = hostType(x.memProps, req.memoryTypeBits);
    if (!mt) {
        return std::unexpected(std::string("no host visible memory for a quad"));
    }
    VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO, nullptr, req.size, *mt};
    PIBRIDGE_VK(vkAllocateMemory(device_, &mai, nullptr, &smem_), "quad staging memory");
    PIBRIDGE_VK(vkBindBufferMemory(device_, staging_, smem_, 0), "quad staging bind");
    PIBRIDGE_VK(vkMapMemory(device_, smem_, 0, VK_WHOLE_SIZE, 0, &sptr_), "quad staging map");
    auto pci = vk_make<VkCommandPoolCreateInfo>(VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO);
    pci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    pci.queueFamilyIndex = x.qf;
    PIBRIDGE_VK(vkCreateCommandPool(device_, &pci, nullptr, &pool_), "quad command pool");
    auto cai = vk_make<VkCommandBufferAllocateInfo>(VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO);
    cai.commandPool = pool_;
    cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cai.commandBufferCount = 1;
    PIBRIDGE_VK(vkAllocateCommandBuffers(device_, &cai, &cmd_), "quad command buffer");
    auto fci = vk_make<VkFenceCreateInfo>(VK_STRUCTURE_TYPE_FENCE_CREATE_INFO);
    PIBRIDGE_VK(vkCreateFence(device_, &fci, nullptr, &fence_), "quad fence");
    w_ = w;
    h_ = h;
    return {};
}

std::expected<void, std::string> QuadImage::upload(Session& x, const std::uint8_t* bgra, std::uint32_t w, std::uint32_t h,
                                                   std::size_t stride) {
    if (w == 0 || h == 0) {
        return std::unexpected(std::string("an empty picture"));
    }
    if (sc_ == XR_NULL_HANDLE || w != w_ || h != h_) {
        if (auto r = make(x, w, h); !r) {
            destroy();
            return r;
        }
    }
    auto* dst = static_cast<std::uint8_t*>(sptr_);
    const std::size_t row = static_cast<std::size_t>(w) * 4;
    for (std::uint32_t y = 0; y < h; y++) {
        std::uint8_t* d = dst + y * row;
        std::memcpy(d, bgra + y * stride, row);
        if (swapRB_) {
            for (std::uint32_t i = 0; i < w; i++) {
                std::swap(d[i * 4], d[i * 4 + 2]);
            }
        }
    }
    std::uint32_t idx = 0;
    auto ai = xr_make<XrSwapchainImageAcquireInfo>(XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO);
    PIBRIDGE_XR(xrAcquireSwapchainImage(sc_, &ai, &idx), "quad acquire");
    XrSwapchainImageWaitInfo wi{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO, nullptr, XR_INFINITE_DURATION};
    PIBRIDGE_XR(xrWaitSwapchainImage(sc_, &wi), "quad wait");
    vkResetCommandBuffer(cmd_, 0);
    auto bbi = vk_make<VkCommandBufferBeginInfo>(VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO);
    bbi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    PIBRIDGE_VK(vkBeginCommandBuffer(cmd_, &bbi), "quad begin");
    auto toDst = vk_make<VkImageMemoryBarrier>(VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER);
    toDst.srcAccessMask = 0;
    toDst.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    toDst.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED; // the whole image is written
    toDst.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    toDst.srcQueueFamilyIndex = toDst.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    toDst.image = images_[idx];
    toDst.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    vkCmdPipelineBarrier(cmd_, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr,
                         1, &toDst);
    VkBufferImageCopy copy{};
    copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    copy.imageExtent = {w, h, 1};
    vkCmdCopyBufferToImage(cmd_, staging_, images_[idx], VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
    // back to the layout the runtime takes a released colour-attachment image in
    auto toColor = toDst;
    toColor.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    toColor.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT;
    toColor.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    toColor.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    vkCmdPipelineBarrier(cmd_, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0, nullptr, 1,
                         &toColor);
    PIBRIDGE_VK(vkEndCommandBuffer(cmd_), "quad end");
    auto si = vk_make<VkSubmitInfo>(VK_STRUCTURE_TYPE_SUBMIT_INFO);
    si.commandBufferCount = 1;
    si.pCommandBuffers = &cmd_;
    PIBRIDGE_VK(vkResetFences(device_, 1, &fence_), "quad fence reset");
    PIBRIDGE_VK(vkQueueSubmit(x.queue, 1, &si, fence_), "quad submit");
    PIBRIDGE_VK(vkWaitForFences(device_, 1, &fence_, VK_TRUE, 1'000'000'000ULL), "quad copy"); // the staging buffer is reused
    auto ri = xr_make<XrSwapchainImageReleaseInfo>(XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO);
    PIBRIDGE_XR(xrReleaseSwapchainImage(sc_, &ri), "quad release");
    released_ = true;
    return {};
}

XrCompositionLayerQuad QuadImage::layer(XrSpace space, const XrPosef& pose, XrExtent2Df sizeM) const {
    auto q = xr_make<XrCompositionLayerQuad>(XR_TYPE_COMPOSITION_LAYER_QUAD);
    q.layerFlags = 0;
    q.space = space;
    q.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
    q.subImage.swapchain = sc_;
    q.subImage.imageRect = {{0, 0}, {static_cast<std::int32_t>(w_), static_cast<std::int32_t>(h_)}};
    q.subImage.imageArrayIndex = 0;
    q.pose = pose;
    q.size = sizeM;
    return q;
}

} // namespace pibridge::xrkit
