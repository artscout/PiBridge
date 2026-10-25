// SPDX-License-Identifier: MIT
// The Vulkan / OpenXR plumbing of pibridge-roomsetup (which follows tools/eyecal/src/xrapp.cpp: session, swapchains,
// frame loop), moved here unchanged so pibridge-home uses the same code.
#include "pibridge/xrkit/xrkit.h"

#include "passthrough_frag.h"
#include "passthrough_vert.h"
#include "scene_frag.h"
#include "scene_vert.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <tuple>

namespace pibridge::xrkit {

namespace {

constexpr VkImageSubresourceRange kColorRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};

std::optional<std::uint32_t> memoryType(const VkPhysicalDeviceMemoryProperties& p, std::uint32_t bits,
                                        VkMemoryPropertyFlags flags) {
    for (std::uint32_t i = 0; i < p.memoryTypeCount; i++) {
        if ((bits & (1U << i)) != 0 && (p.memoryTypes[i].propertyFlags & flags) == flags) {
            return i;
        }
    }
    return std::nullopt;
}

constexpr std::int64_t kR8G8B8A8Srgb = VK_FORMAT_R8G8B8A8_SRGB;
constexpr std::int64_t kB8G8R8A8Srgb = VK_FORMAT_B8G8R8A8_SRGB;
constexpr std::int64_t kR8G8B8A8Unorm = VK_FORMAT_R8G8B8A8_UNORM;
constexpr std::int64_t kB8G8R8A8Unorm = VK_FORMAT_B8G8R8A8_UNORM;

} // namespace

std::optional<std::int64_t> chooseFormat(const std::vector<std::int64_t>& offered) {
    for (const auto f : {kR8G8B8A8Srgb, kB8G8R8A8Srgb, kR8G8B8A8Unorm, kB8G8R8A8Unorm}) {
        if (std::ranges::find(offered, f) != offered.end()) {
            return f;
        }
    }
    return std::nullopt;
}

// ---------------------------------------------------------------- SceneGpu

std::expected<VkPipeline, std::string> SceneGpu::pipeline(VkPrimitiveTopology topo, VkShaderModule vs, VkShaderModule fs) {
    VkPipelineShaderStageCreateInfo stages[2] = {
        {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_VERTEX_BIT, vs, "main", nullptr},
        {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_FRAGMENT_BIT, fs, "main", nullptr}};
    const VkVertexInputBindingDescription bind{0, sizeof(room::Vertex), VK_VERTEX_INPUT_RATE_VERTEX};
    const std::array<VkVertexInputAttributeDescription, 2> attrs{{{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},
                                                                  {1, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 12}}};
    auto vin = vk_make<VkPipelineVertexInputStateCreateInfo>(VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO);
    vin.vertexBindingDescriptionCount = 1;
    vin.pVertexBindingDescriptions = &bind;
    vin.vertexAttributeDescriptionCount = static_cast<std::uint32_t>(attrs.size());
    vin.pVertexAttributeDescriptions = attrs.data();
    auto ia = vk_make<VkPipelineInputAssemblyStateCreateInfo>(VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO);
    ia.topology = topo;
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
    VkPipelineColorBlendAttachmentState ba{};
    ba.blendEnable = VK_TRUE;
    ba.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    ba.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    ba.colorBlendOp = VK_BLEND_OP_ADD;
    ba.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    ba.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    ba.alphaBlendOp = VK_BLEND_OP_ADD;
    ba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    auto blend = vk_make<VkPipelineColorBlendStateCreateInfo>(VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO);
    blend.attachmentCount = 1;
    blend.pAttachments = &ba;
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
    gpi.renderPass = renderPass;
    VkPipeline p = VK_NULL_HANDLE;
    PIBRIDGE_VK(vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &gpi, nullptr, &p), "graphics pipeline");
    return p;
}

std::expected<void, std::string> SceneGpu::create(VkDevice dev, VkQueue q, std::uint32_t qf,
                                              const VkPhysicalDeviceMemoryProperties& mp, VkFormat format,
                                              std::array<std::uint32_t, 2> sz,
                                              const std::array<std::vector<VkImage>, 2>& images) {
    device = dev;
    queue = q;
    memProps = mp;
    size = sz;
    VkShaderModuleCreateInfo vsi{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO, nullptr, 0, sizeof(scene_vert), scene_vert};
    VkShaderModuleCreateInfo fsi{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO, nullptr, 0, sizeof(scene_frag), scene_frag};
    VkShaderModule vs = VK_NULL_HANDLE;
    VkShaderModule fs = VK_NULL_HANDLE;
    PIBRIDGE_VK(vkCreateShaderModule(device, &vsi, nullptr, &vs), "vertex shader");
    PIBRIDGE_VK(vkCreateShaderModule(device, &fsi, nullptr, &fs), "fragment shader");
    const VkPushConstantRange pcr{VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(Push)};
    VkPipelineLayoutCreateInfo pli{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO, nullptr, 0, 0, nullptr, 1, &pcr};
    PIBRIDGE_VK(vkCreatePipelineLayout(device, &pli, nullptr, &layout), "pipeline layout");
    VkAttachmentDescription att{0,
                                format,
                                VK_SAMPLE_COUNT_1_BIT,
                                VK_ATTACHMENT_LOAD_OP_CLEAR,
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
    PIBRIDGE_VK(vkCreateRenderPass(device, &rpi, nullptr, &renderPass), "render pass");
    auto t = pipeline(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, vs, fs);
    auto l = pipeline(VK_PRIMITIVE_TOPOLOGY_LINE_LIST, vs, fs);
    vkDestroyShaderModule(device, vs, nullptr);
    vkDestroyShaderModule(device, fs, nullptr);
    if (!t || !l) {
        return std::unexpected(!t ? t.error() : l.error());
    }
    tris = *t;
    lines = *l;
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
            vi.subresourceRange = kColorRange;
            VkImageView view = VK_NULL_HANDLE;
            PIBRIDGE_VK(vkCreateImageView(device, &vi, nullptr, &view), "swapchain image view");
            auto fbi = vk_make<VkFramebufferCreateInfo>(VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO);
            fbi.renderPass = renderPass;
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

std::expected<void, std::string> SceneGpu::reserve(std::size_t n) {
    if (n <= vcap) {
        return {};
    }
    if (vbuf != VK_NULL_HANDLE) {
        vkUnmapMemory(device, vmem);
        vkFreeMemory(device, vmem, nullptr);
        vkDestroyBuffer(device, vbuf, nullptr);
        vbuf = VK_NULL_HANDLE;
    }
    vcap = std::max<std::size_t>(n * 3 / 2, 65536);
    auto bi = vk_make<VkBufferCreateInfo>(VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO);
    bi.size = vcap * sizeof(room::Vertex);
    bi.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    PIBRIDGE_VK(vkCreateBuffer(device, &bi, nullptr, &vbuf), "vertex buffer");
    VkMemoryRequirements req;
    vkGetBufferMemoryRequirements(device, vbuf, &req);
    const auto mt = memoryType(memProps, req.memoryTypeBits,
                               VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    if (!mt) {
        return std::unexpected(std::string("no host visible memory for the vertices"));
    }
    VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO, nullptr, req.size, *mt};
    PIBRIDGE_VK(vkAllocateMemory(device, &mai, nullptr, &vmem), "vertex memory");
    PIBRIDGE_VK(vkBindBufferMemory(device, vbuf, vmem, 0), "vertex bind");
    PIBRIDGE_VK(vkMapMemory(device, vmem, 0, VK_WHOLE_SIZE, 0, &vptr), "vertex map");
    return {};
}

std::expected<void, std::string> SceneGpu::upload(const room::Batch& b, const room::Batch* overlay) {
    const std::size_t no = overlay ? overlay->tris.size() + overlay->lines.size() : 0;
    if (auto r = reserve(b.tris.size() + b.lines.size() + no); !r) {
        return r;
    }
    auto* dst = static_cast<room::Vertex*>(vptr);
    std::ranges::copy(b.tris, dst);
    std::ranges::copy(b.lines, dst + b.tris.size());
    overlayTris = overlay ? static_cast<std::uint32_t>(overlay->tris.size()) : 0;
    overlayLines = overlay ? static_cast<std::uint32_t>(overlay->lines.size()) : 0;
    if (overlay) {
        auto* o = dst + b.tris.size() + b.lines.size();
        std::ranges::copy(overlay->tris, o);
        std::ranges::copy(overlay->lines, o + overlay->tris.size());
    }
    return {};
}

std::expected<void, std::string> SceneGpu::draw(std::size_t eye, std::size_t idx, const Push& push, std::uint32_t nTris,
                                            std::uint32_t nLines) {
    PIBRIDGE_VK(vkResetCommandBuffer(cmd, 0), "reset");
    VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO, nullptr,
                                VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT, nullptr};
    PIBRIDGE_VK(vkBeginCommandBuffer(cmd, &bi), "begin");
    pt.recordUpload(cmd); // a new camera frame: copied once, before the first eye's pass
    sky.recordUpload(cmd);
    panel.recordUpload(cmd);
    VkClearValue clear{};
    clear.color = {{background[0], background[1], background[2], background[3]}};
    auto rbi = vk_make<VkRenderPassBeginInfo>(VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO);
    rbi.renderPass = renderPass;
    rbi.framebuffer = targets[eye][idx].second;
    rbi.renderArea = {{0, 0}, {size[0], size[1]}};
    rbi.clearValueCount = 1;
    rbi.pClearValues = &clear;
    vkCmdBeginRenderPass(cmd, &rbi, VK_SUBPASS_CONTENTS_INLINE);
    if (eye < 2 && sky.ready() && sky.eyeOn[eye] && sky.haveImage) {
        sky.recordDraw(cmd, eye);
    } else {
        pt.recordDraw(cmd, eye);
    }
    if (eye < 2) {
        pt.eyeOn[eye] = false;
        sky.eyeOn[eye] = false;
    }
    const auto lists = [&](std::uint32_t first, std::uint32_t nt, std::uint32_t nl) {
        if (vbuf == VK_NULL_HANDLE || nt + nl == 0) {
            return;
        }
        const VkDeviceSize off = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &vbuf, &off);
        vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(Push), &push);
        if (nt > 0) {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, tris);
            vkCmdDraw(cmd, nt, 1, first, 0);
        }
        if (nl > 0) {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, lines);
            vkCmdDraw(cmd, nl, 1, first + nt, 0);
        }
    };
    lists(0, nTris, nLines);
    if (eye < 2 && panel.ready() && panel.eyeOn[eye]) {
        panel.recordDraw(cmd, eye);
        panel.eyeOn[eye] = false;
    }
    lists(nTris + nLines, overlayTris, overlayLines);
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

void SceneGpu::destroy() {
    if (device == VK_NULL_HANDLE) {
        return;
    }
    pt.destroy();
    sky.destroy();
    panel.destroy();
    for (auto& t : targets) {
        for (auto& [view, fb] : t) {
            vkDestroyFramebuffer(device, fb, nullptr);
            vkDestroyImageView(device, view, nullptr);
        }
        t.clear();
    }
    if (vbuf != VK_NULL_HANDLE) {
        vkUnmapMemory(device, vmem);
        vkFreeMemory(device, vmem, nullptr);
        vkDestroyBuffer(device, vbuf, nullptr);
    }
    vkDestroyPipeline(device, tris, nullptr);
    vkDestroyPipeline(device, lines, nullptr);
    vkDestroyPipelineLayout(device, layout, nullptr);
    vkDestroyRenderPass(device, renderPass, nullptr);
    vkDestroyFence(device, fence, nullptr);
    vkDestroyCommandPool(device, pool, nullptr);
    device = VK_NULL_HANDLE;
}

// ---------------------------------------------------------------- PassthroughGpu

namespace {

std::expected<std::pair<VkBuffer, VkDeviceMemory>, std::string> hostBuffer(VkDevice device, const VkPhysicalDeviceMemoryProperties& mp,
                                                                          VkDeviceSize size, VkBufferUsageFlags usage) {
    auto bi = vk_make<VkBufferCreateInfo>(VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO);
    bi.size = size;
    bi.usage = usage;
    bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    VkBuffer b = VK_NULL_HANDLE;
    PIBRIDGE_VK(vkCreateBuffer(device, &bi, nullptr, &b), "passthrough buffer");
    VkMemoryRequirements req;
    vkGetBufferMemoryRequirements(device, b, &req);
    const auto mt = memoryType(mp, req.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    if (!mt) {
        vkDestroyBuffer(device, b, nullptr);
        return std::unexpected(std::string("no host visible memory for the passthrough"));
    }
    VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO, nullptr, req.size, *mt};
    VkDeviceMemory m = VK_NULL_HANDLE;
    PIBRIDGE_VK(vkAllocateMemory(device, &mai, nullptr, &m), "passthrough buffer memory");
    PIBRIDGE_VK(vkBindBufferMemory(device, b, m, 0), "passthrough buffer bind");
    return std::pair{b, m};
}

} // namespace

std::expected<void, std::string> PassthroughGpu::create(VkDevice dev, const VkPhysicalDeviceMemoryProperties& mp, VkRenderPass rp,
                                                        std::array<std::uint32_t, 2> size, VkDeviceSize uboAlign) {
    device = dev;
    uStride = std::max<VkDeviceSize>(1024, ((sizeof(pt::Uniform) + uboAlign - 1) / std::max<VkDeviceSize>(uboAlign, 1)) * uboAlign);
    constexpr VkDeviceSize kImg = static_cast<VkDeviceSize>(pt::kW) * pt::kH;
    auto st = hostBuffer(device, mp, pt::kCams * kImg, VK_BUFFER_USAGE_TRANSFER_SRC_BIT);
    if (!st) {
        return std::unexpected(st.error());
    }
    std::tie(staging, smem) = *st;
    PIBRIDGE_VK(vkMapMemory(device, smem, 0, VK_WHOLE_SIZE, 0, &sptr), "passthrough staging map");
    auto ub = hostBuffer(device, mp, 2 * uStride, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
    if (!ub) {
        return std::unexpected(ub.error());
    }
    std::tie(ubuf, umem) = *ub;
    void* up = nullptr;
    PIBRIDGE_VK(vkMapMemory(device, umem, 0, VK_WHOLE_SIZE, 0, &up), "passthrough uniform map");
    uptr = static_cast<std::uint8_t*>(up);
    {
        auto ii = vk_make<VkImageCreateInfo>(VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO);
        ii.imageType = VK_IMAGE_TYPE_2D;
        ii.format = VK_FORMAT_R8_UNORM;
        ii.extent = {static_cast<std::uint32_t>(pt::kW), static_cast<std::uint32_t>(pt::kH), 1};
        ii.mipLevels = 1;
        ii.arrayLayers = static_cast<std::uint32_t>(pt::kCams);
        ii.samples = VK_SAMPLE_COUNT_1_BIT;
        ii.tiling = VK_IMAGE_TILING_OPTIMAL;
        ii.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        ii.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        ii.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        PIBRIDGE_VK(vkCreateImage(device, &ii, nullptr, &img), "passthrough image");
        VkMemoryRequirements req;
        vkGetImageMemoryRequirements(device, img, &req);
        auto mt = memoryType(mp, req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        if (!mt) {
            mt = memoryType(mp, req.memoryTypeBits, 0);
        }
        if (!mt) {
            return std::unexpected(std::string("no memory for the passthrough images"));
        }
        VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO, nullptr, req.size, *mt};
        PIBRIDGE_VK(vkAllocateMemory(device, &mai, nullptr, &imem), "passthrough image memory");
        PIBRIDGE_VK(vkBindImageMemory(device, img, imem, 0), "passthrough image bind");
        auto vi = vk_make<VkImageViewCreateInfo>(VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO);
        vi.image = img;
        vi.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
        vi.format = VK_FORMAT_R8_UNORM;
        vi.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, static_cast<std::uint32_t>(pt::kCams)};
        PIBRIDGE_VK(vkCreateImageView(device, &vi, nullptr, &view), "passthrough image view");
    }
    auto si = vk_make<VkSamplerCreateInfo>(VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO);
    si.magFilter = VK_FILTER_LINEAR;
    si.minFilter = VK_FILTER_LINEAR;
    si.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    si.addressModeU = si.addressModeV = si.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    si.maxLod = 0.0F;
    PIBRIDGE_VK(vkCreateSampler(device, &si, nullptr, &sampler), "passthrough sampler");
    const std::array<VkDescriptorSetLayoutBinding, 2> b{{
        {0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
        {1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
    }};
    auto dli = vk_make<VkDescriptorSetLayoutCreateInfo>(VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO);
    dli.bindingCount = static_cast<std::uint32_t>(b.size());
    dli.pBindings = b.data();
    PIBRIDGE_VK(vkCreateDescriptorSetLayout(device, &dli, nullptr, &dsl), "passthrough set layout");
    const std::array<VkDescriptorPoolSize, 2> ps{{{VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 2}, {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 2}}};
    auto dpi = vk_make<VkDescriptorPoolCreateInfo>(VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO);
    dpi.maxSets = 2;
    dpi.poolSizeCount = static_cast<std::uint32_t>(ps.size());
    dpi.pPoolSizes = ps.data();
    PIBRIDGE_VK(vkCreateDescriptorPool(device, &dpi, nullptr, &pool), "passthrough descriptor pool");
    const std::array<VkDescriptorSetLayout, 2> layouts{dsl, dsl};
    auto dai = vk_make<VkDescriptorSetAllocateInfo>(VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO);
    dai.descriptorPool = pool;
    dai.descriptorSetCount = 2;
    dai.pSetLayouts = layouts.data();
    PIBRIDGE_VK(vkAllocateDescriptorSets(device, &dai, sets.data()), "passthrough descriptor sets");
    for (std::size_t e = 0; e < 2; e++) {
        const VkDescriptorBufferInfo bi{ubuf, e * uStride, sizeof(pt::Uniform)};
        const VkDescriptorImageInfo ii{sampler, view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        std::array<VkWriteDescriptorSet, 2> w{};
        for (std::uint32_t k = 0; k < 2; k++) {
            w[k] = vk_make<VkWriteDescriptorSet>(VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET);
            w[k].dstSet = sets[e];
            w[k].dstBinding = k;
            w[k].descriptorCount = 1;
            w[k].descriptorType = k == 0 ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER : VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        }
        w[0].pBufferInfo = &bi;
        w[1].pImageInfo = &ii;
        vkUpdateDescriptorSets(device, static_cast<std::uint32_t>(w.size()), w.data(), 0, nullptr);
    }
    VkPipelineLayoutCreateInfo pli{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO, nullptr, 0, 1, &dsl, 0, nullptr};
    PIBRIDGE_VK(vkCreatePipelineLayout(device, &pli, nullptr, &layout), "passthrough pipeline layout");
    VkShaderModuleCreateInfo vsi{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO, nullptr, 0, sizeof(passthrough_vert), passthrough_vert};
    VkShaderModuleCreateInfo fsi{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO, nullptr, 0, sizeof(passthrough_frag), passthrough_frag};
    VkShaderModule vs = VK_NULL_HANDLE;
    VkShaderModule fs = VK_NULL_HANDLE;
    PIBRIDGE_VK(vkCreateShaderModule(device, &vsi, nullptr, &vs), "passthrough vertex shader");
    PIBRIDGE_VK(vkCreateShaderModule(device, &fsi, nullptr, &fs), "passthrough fragment shader");
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
    rs.lineWidth = 1.0F;
    auto ms = vk_make<VkPipelineMultisampleStateCreateInfo>(VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO);
    ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    VkPipelineColorBlendAttachmentState ba{};
    ba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    auto blend = vk_make<VkPipelineColorBlendStateCreateInfo>(VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO);
    blend.attachmentCount = 1;
    blend.pAttachments = &ba;
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
    gpi.renderPass = rp;
    const VkResult r = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &gpi, nullptr, &pipe);
    vkDestroyShaderModule(device, vs, nullptr);
    vkDestroyShaderModule(device, fs, nullptr);
    PIBRIDGE_VK(r, "passthrough pipeline");
    return {};
}

void PassthroughGpu::setFrame(const pt::Frame& f) {
    constexpr std::size_t kImg = static_cast<std::size_t>(pt::kW) * pt::kH;
    if (sptr == nullptr || f.n == 0) {
        return;
    }
    auto* dst = static_cast<std::uint8_t*>(sptr);
    std::uint32_t n = 0;
    for (std::size_t s = 0; s < std::min(f.n, pt::kCams); s++) {
        if (f.img[s].size() != kImg) {
            return;
        }
        std::memcpy(dst + s * kImg, f.img[s].data(), kImg);
        n++;
    }
    layers = n;
    dirty = true;
}

void PassthroughGpu::setEye(std::size_t eye, const pt::Uniform& u) {
    if (uptr != nullptr && eye < 2) {
        std::memcpy(uptr + eye * uStride, &u, sizeof u);
        eyeOn[eye] = true;
    }
}

void PassthroughGpu::recordUpload(VkCommandBuffer cmd) {
    if (!dirty || !ready() || layers == 0) {
        return;
    }
    // every layer goes to TRANSFER_DST and back (the first time all four, so none is left UNDEFINED)
    auto b = vk_make<VkImageMemoryBarrier>(VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER);
    b.srcAccessMask = initialised ? VK_ACCESS_SHADER_READ_BIT : 0;
    b.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    b.oldLayout = initialised ? VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL : VK_IMAGE_LAYOUT_UNDEFINED;
    b.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b.image = img;
    b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, static_cast<std::uint32_t>(pt::kCams)};
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                         VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &b);
    VkBufferImageCopy r{};
    r.bufferOffset = 0;
    r.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, layers}; // tightly packed layers, slot order
    r.imageExtent = {static_cast<std::uint32_t>(pt::kW), static_cast<std::uint32_t>(pt::kH), 1};
    vkCmdCopyBufferToImage(cmd, staging, img, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &r);
    b.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    b.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    b.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    b.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1, &b);
    dirty = false;
    initialised = true;
    haveImage = true;
}

void PassthroughGpu::recordDraw(VkCommandBuffer cmd, std::size_t eye) {
    if (!ready() || !haveImage || eye > 1 || !eyeOn[eye]) {
        return;
    }
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, 1, &sets[eye], 0, nullptr);
    vkCmdDraw(cmd, 3, 1, 0, 0);
}

void PassthroughGpu::destroy() {
    if (device == VK_NULL_HANDLE) {
        return;
    }
    vkDestroyPipeline(device, pipe, nullptr);
    vkDestroyPipelineLayout(device, layout, nullptr);
    vkDestroyDescriptorPool(device, pool, nullptr);
    vkDestroyDescriptorSetLayout(device, dsl, nullptr);
    vkDestroySampler(device, sampler, nullptr);
    vkDestroyImageView(device, view, nullptr);
    vkDestroyImage(device, img, nullptr);
    vkFreeMemory(device, imem, nullptr);
    if (smem != VK_NULL_HANDLE) {
        vkUnmapMemory(device, smem);
    }
    vkDestroyBuffer(device, staging, nullptr);
    vkFreeMemory(device, smem, nullptr);
    if (umem != VK_NULL_HANDLE) {
        vkUnmapMemory(device, umem);
    }
    vkDestroyBuffer(device, ubuf, nullptr);
    vkFreeMemory(device, umem, nullptr);
    *this = PassthroughGpu{};
}

// ---------------------------------------------------------------- Session

Session::~Session() {
    if (device != VK_NULL_HANDLE) {
        vkDeviceWaitIdle(device);
    }
    gpu.destroy();
    for (auto sc : eyeSc) {
        if (sc != XR_NULL_HANDLE) {
            xrDestroySwapchain(sc);
        }
    }
    for (auto& h : hands) {
        if (h.grip != XR_NULL_HANDLE) {
            xrDestroySpace(h.grip);
        }
        if (h.aim != XR_NULL_HANDLE) {
            xrDestroySpace(h.aim);
        }
    }
    if (stage != XR_NULL_HANDLE) {
        xrDestroySpace(stage);
    }
    if (view != XR_NULL_HANDLE) {
        xrDestroySpace(view);
    }
    if (session != XR_NULL_HANDLE) {
        xrDestroySession(session);
    }
    if (device != VK_NULL_HANDLE) {
        vkDestroyDevice(device, nullptr);
    }
    if (vkInstance != VK_NULL_HANDLE) {
        vkDestroyInstance(vkInstance, nullptr);
    }
    if (instance != XR_NULL_HANDLE) {
        xrDestroyInstance(instance);
    }
}

XrPath Session::path(const std::string& s) const {
    XrPath p = XR_NULL_PATH;
    (void)xrStringToPath(instance, s.c_str(), &p);
    return p;
}

bool Session::srgb() const {
    return format == kR8G8B8A8Srgb || format == kB8G8R8A8Srgb;
}

std::expected<XrAction, std::string> makeAction(XrActionSet set, const char* name, const char* label, XrActionType type,
                                                std::span<const XrPath> sub) {
    auto ai = xr_make<XrActionCreateInfo>(XR_TYPE_ACTION_CREATE_INFO);
    std::snprintf(ai.actionName, sizeof ai.actionName, "%s", name);
    std::snprintf(ai.localizedActionName, sizeof ai.localizedActionName, "%s", label);
    ai.actionType = type;
    ai.countSubactionPaths = static_cast<std::uint32_t>(sub.size());
    ai.subactionPaths = sub.data();
    XrAction a = XR_NULL_HANDLE;
    PIBRIDGE_XR(xrCreateAction(set, &ai, &a), name);
    return a;
}

std::expected<void, std::string> openSession(Session& x, const OpenOpts& opts) {
    std::uint32_t n = 0;
    const std::string noRuntime = "cannot load the OpenXR runtime (" + opts.runtimeHint + ")";
    PIBRIDGE_XR(xrEnumerateInstanceExtensionProperties(nullptr, 0, &n, nullptr), noRuntime.c_str());
    std::vector<XrExtensionProperties> props(n, xr_make<XrExtensionProperties>(XR_TYPE_EXTENSION_PROPERTIES));
    PIBRIDGE_XR(xrEnumerateInstanceExtensionProperties(nullptr, n, &n, props.data()), "extensions");
    if (!std::ranges::any_of(props, [](const XrExtensionProperties& p) {
            return std::strcmp(p.extensionName, XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME) == 0;
        })) {
        return std::unexpected(std::string("the OpenXR runtime does not offer XR_KHR_vulkan_enable2"));
    }
    const bool timespec = std::ranges::any_of(props, [](const XrExtensionProperties& p) {
        return std::strcmp(p.extensionName, XR_KHR_CONVERT_TIMESPEC_TIME_EXTENSION_NAME) == 0;
    });
    // Optional: the runtime's own controller render model as a glTF (XR_MSFT_controller_model). We only enable it so a
    // caller can probe what the runtime hands back; the drawing still goes through OpenVR render models for now.
    const bool ctrlModel = std::ranges::any_of(props, [](const XrExtensionProperties& p) {
        return std::strcmp(p.extensionName, XR_MSFT_CONTROLLER_MODEL_EXTENSION_NAME) == 0;
    });
    std::vector<const char*> exts = {XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME};
    if (timespec) {
        exts.push_back(XR_KHR_CONVERT_TIMESPEC_TIME_EXTENSION_NAME);
    }
    if (ctrlModel) {
        exts.push_back(XR_MSFT_CONTROLLER_MODEL_EXTENSION_NAME);
    }
    auto ici = xr_make<XrInstanceCreateInfo>(XR_TYPE_INSTANCE_CREATE_INFO);
    std::snprintf(ici.applicationInfo.applicationName, sizeof ici.applicationInfo.applicationName, "%s", opts.appName.c_str());
    std::snprintf(ici.applicationInfo.engineName, sizeof ici.applicationInfo.engineName, "%s", opts.appName.c_str());
    ici.applicationInfo.applicationVersion = 1;
    ici.applicationInfo.engineVersion = 1;
    ici.applicationInfo.apiVersion = XR_MAKE_VERSION(1, 0, 0);
    ici.enabledExtensionCount = static_cast<std::uint32_t>(exts.size());
    ici.enabledExtensionNames = exts.data();
    const std::string noInstance = "xrCreateInstance (" + opts.runtimeHint + ")";
    PIBRIDGE_XR(xrCreateInstance(&ici, &x.instance), noInstance.c_str());
    x.timespec = timespec;
    x.controllerModelExt = ctrlModel;
    auto ip = xr_make<XrInstanceProperties>(XR_TYPE_INSTANCE_PROPERTIES);
    if (XR_SUCCEEDED(xrGetInstanceProperties(x.instance, &ip))) {
        x.runtimeName = ip.runtimeName;
    }
    XrSystemGetInfo sgi{XR_TYPE_SYSTEM_GET_INFO, nullptr, XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY};
    PIBRIDGE_XR(xrGetSystem(x.instance, &sgi, &x.system), "no head mounted display");
    auto reqsFn = x.proc<PFN_xrGetVulkanGraphicsRequirements2KHR>("xrGetVulkanGraphicsRequirements2KHR");
    auto instFn = x.proc<PFN_xrCreateVulkanInstanceKHR>("xrCreateVulkanInstanceKHR");
    auto pdFn = x.proc<PFN_xrGetVulkanGraphicsDevice2KHR>("xrGetVulkanGraphicsDevice2KHR");
    auto devFn = x.proc<PFN_xrCreateVulkanDeviceKHR>("xrCreateVulkanDeviceKHR");
    if (!reqsFn || !instFn || !pdFn || !devFn) {
        return std::unexpected(std::string("the runtime lacks the XR_KHR_vulkan_enable2 functions"));
    }
    auto reqs = xr_make<XrGraphicsRequirementsVulkan2KHR>(XR_TYPE_GRAPHICS_REQUIREMENTS_VULKAN2_KHR);
    PIBRIDGE_XR((*reqsFn)(x.instance, x.system, &reqs), "graphics requirements");
    if (XR_MAKE_VERSION(1, 1, 0) < reqs.minApiVersionSupported) {
        return std::unexpected(std::string("the runtime wants a newer Vulkan than 1.1"));
    }
    auto app = vk_make<VkApplicationInfo>(VK_STRUCTURE_TYPE_APPLICATION_INFO);
    app.pApplicationName = opts.appName.c_str();
    app.apiVersion = VK_MAKE_API_VERSION(0, 1, 1, 0);
    auto vci = vk_make<VkInstanceCreateInfo>(VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO);
    vci.pApplicationInfo = &app;
    auto xvi = xr_make<XrVulkanInstanceCreateInfoKHR>(XR_TYPE_VULKAN_INSTANCE_CREATE_INFO_KHR);
    xvi.systemId = x.system;
    xvi.pfnGetInstanceProcAddr = &vkGetInstanceProcAddr;
    xvi.vulkanCreateInfo = &vci;
    VkResult vr = VK_SUCCESS;
    PIBRIDGE_XR((*instFn)(x.instance, &xvi, &x.vkInstance, &vr), "xrCreateVulkanInstanceKHR");
    PIBRIDGE_VK(vr, "vkCreateInstance");
    auto gdi = xr_make<XrVulkanGraphicsDeviceGetInfoKHR>(XR_TYPE_VULKAN_GRAPHICS_DEVICE_GET_INFO_KHR);
    gdi.systemId = x.system;
    gdi.vulkanInstance = x.vkInstance;
    PIBRIDGE_XR((*pdFn)(x.instance, &gdi, &x.pd), "xrGetVulkanGraphicsDevice2KHR");
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
    PIBRIDGE_XR((*devFn)(x.instance, &xdi, &x.device, &vr), "xrCreateVulkanDeviceKHR");
    PIBRIDGE_VK(vr, "vkCreateDevice");
    vkGetDeviceQueue(x.device, x.qf, 0, &x.queue);
    vkGetPhysicalDeviceMemoryProperties(x.pd, &x.memProps);
    auto binding = xr_make<XrGraphicsBindingVulkan2KHR>(XR_TYPE_GRAPHICS_BINDING_VULKAN2_KHR);
    binding.instance = x.vkInstance;
    binding.physicalDevice = x.pd;
    binding.device = x.device;
    binding.queueFamilyIndex = x.qf;
    binding.queueIndex = 0;
    XrSessionCreateInfo sci{XR_TYPE_SESSION_CREATE_INFO, &binding, 0, x.system};
    PIBRIDGE_XR(xrCreateSession(x.instance, &sci, &x.session), "xrCreateSession");
    XrReferenceSpaceCreateInfo rsi{XR_TYPE_REFERENCE_SPACE_CREATE_INFO, nullptr, XR_REFERENCE_SPACE_TYPE_STAGE, kIdentity};
    if (XR_FAILED(xrCreateReferenceSpace(x.session, &rsi, &x.stage))) {
        rsi.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL; // ZoneTweak's fallback
        x.stageType = XR_REFERENCE_SPACE_TYPE_LOCAL;
        PIBRIDGE_XR(xrCreateReferenceSpace(x.session, &rsi, &x.stage), "stage space");
    }
    rsi.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    PIBRIDGE_XR(xrCreateReferenceSpace(x.session, &rsi, &x.view), "view space");
    return {};
}

std::expected<void, std::string> createSwapchains(Session& x, const OpenOpts& opts) {
    std::uint32_t nv = 0;
    PIBRIDGE_XR(xrEnumerateViewConfigurationViews(x.instance, x.system, kViewType, 0, &nv, nullptr), "views");
    std::vector<XrViewConfigurationView> views(nv, xr_make<XrViewConfigurationView>(XR_TYPE_VIEW_CONFIGURATION_VIEW));
    PIBRIDGE_XR(xrEnumerateViewConfigurationViews(x.instance, x.system, kViewType, nv, &nv, views.data()), "views");
    if (nv != 2) {
        return std::unexpected(std::format("expected 2 views, got {}", nv));
    }
    // ZoneTweak caps the eye size at 3200 px: lines and pixel text need no more
    std::uint32_t w = views[0].recommendedImageRectWidth / std::max(opts.resDiv, 1U);
    std::uint32_t h = views[0].recommendedImageRectHeight / std::max(opts.resDiv, 1U);
    const std::uint32_t cap = std::max(opts.maxEye, 64U);
    if (const std::uint32_t m = std::max(w, h); m > cap) {
        w = w * cap / m;
        h = h * cap / m;
    }
    x.size = {std::max(w, 16U), std::max(h, 16U)};
    auto& size = x.size;
    std::uint32_t nf = 0;
    PIBRIDGE_XR(xrEnumerateSwapchainFormats(x.session, 0, &nf, nullptr), "swapchain formats");
    std::vector<std::int64_t> offered(nf);
    PIBRIDGE_XR(xrEnumerateSwapchainFormats(x.session, nf, &nf, offered.data()), "swapchain formats");
    const auto f = chooseFormat(offered);
    if (!f) {
        return std::unexpected(std::string("no 8-bit RGBA/BGRA swapchain format"));
    }
    x.format = *f;
    const std::int64_t format = x.format;
    std::printf("render %ux%u per eye, runtime %s, space %s\n", size[0], size[1], x.runtimeName.c_str(),
                x.stageType == XR_REFERENCE_SPACE_TYPE_STAGE ? "STAGE" : "LOCAL");
    for (std::size_t e = 0; e < 2; e++) {
        auto ci = xr_make<XrSwapchainCreateInfo>(XR_TYPE_SWAPCHAIN_CREATE_INFO);
        ci.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
        ci.format = format;
        ci.sampleCount = 1;
        ci.width = size[0];
        ci.height = size[1];
        ci.faceCount = 1;
        ci.arraySize = 1;
        ci.mipCount = 1;
        PIBRIDGE_XR(xrCreateSwapchain(x.session, &ci, &x.eyeSc[e]), "swapchain");
        std::uint32_t ni = 0;
        PIBRIDGE_XR(xrEnumerateSwapchainImages(x.eyeSc[e], 0, &ni, nullptr), "swapchain images");
        std::vector<XrSwapchainImageVulkan2KHR> v(ni, xr_make<XrSwapchainImageVulkan2KHR>(XR_TYPE_SWAPCHAIN_IMAGE_VULKAN2_KHR));
        PIBRIDGE_XR(xrEnumerateSwapchainImages(x.eyeSc[e], ni, &ni, reinterpret_cast<XrSwapchainImageBaseHeader*>(v.data())),
                 "swapchain images");
        for (const auto& i : v) {
            x.eyeImages[e].push_back(i.image);
        }
    }
    return x.gpu.create(x.device, x.queue, x.qf, x.memProps, static_cast<VkFormat>(format), size, x.eyeImages);
}

std::expected<void, std::string> enablePassthrough(Session& x) {
    if (x.gpu.pt.ready()) {
        return {};
    }
    VkPhysicalDeviceProperties props{};
    vkGetPhysicalDeviceProperties(x.pd, &props);
    auto r = x.gpu.pt.create(x.device, x.memProps, x.gpu.renderPass, x.size, props.limits.minUniformBufferOffsetAlignment);
    if (!r) {
        x.gpu.pt.destroy();
    }
    return r;
}

std::expected<void, std::string> enableSky(Session& x) {
    if (x.gpu.sky.ready()) {
        return {};
    }
    auto r = x.gpu.sky.create(x.device, x.memProps, x.gpu.renderPass, x.size, TexPass::Kind::Sky, VK_FORMAT_R8G8B8A8_SRGB);
    if (!r) {
        x.gpu.sky.destroy();
    }
    return r;
}

std::expected<void, std::string> enablePanel(Session& x) {
    if (x.gpu.panel.ready()) {
        return {};
    }
    auto r = x.gpu.panel.create(x.device, x.memProps, x.gpu.renderPass, x.size, TexPass::Kind::Mesh, VK_FORMAT_B8G8R8A8_SRGB);
    if (!r) {
        x.gpu.panel.destroy();
    }
    return r;
}

std::optional<XrTime> xrTimeOfMonotonic(const Session& x, std::uint64_t ns) {
    if (!x.timespec) {
        return std::nullopt;
    }
    static PFN_xrConvertTimespecTimeToTimeKHR fn = nullptr;
    if (fn == nullptr) {
        auto f = x.proc<PFN_xrConvertTimespecTimeToTimeKHR>("xrConvertTimespecTimeToTimeKHR");
        if (!f) {
            return std::nullopt;
        }
        fn = *f;
    }
    timespec ts{static_cast<std::time_t>(ns / 1000000000ULL), static_cast<long>(ns % 1000000000ULL)};
    XrTime t = 0;
    if (XR_FAILED(fn(x.instance, &ts, &t))) {
        return std::nullopt;
    }
    return t;
}

std::optional<std::uint64_t> monotonicOfXrTime(const Session& x, XrTime t) {
    if (!x.timespec) {
        return std::nullopt;
    }
    static PFN_xrConvertTimeToTimespecTimeKHR fn = nullptr;
    if (fn == nullptr) {
        auto f = x.proc<PFN_xrConvertTimeToTimespecTimeKHR>("xrConvertTimeToTimespecTimeKHR");
        if (!f) {
            return std::nullopt;
        }
        fn = *f;
    }
    timespec ts{};
    if (XR_FAILED(fn(x.instance, t, &ts)) || ts.tv_sec < 0) {
        return std::nullopt;
    }
    return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<std::uint64_t>(ts.tv_nsec);
}

room::Mat34 toMat(const XrPosef& p) {
    return room::poseMatrix(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w,
                            {p.position.x, p.position.y, p.position.z});
}

bool locate(XrSpace space, XrSpace base, XrTime t, room::Mat34& out, bool& tracked) {
    auto loc = xr_make<XrSpaceLocation>(XR_TYPE_SPACE_LOCATION);
    tracked = false;
    if (XR_FAILED(xrLocateSpace(space, base, t, &loc))) {
        return false;
    }
    const XrSpaceLocationFlags need = XR_SPACE_LOCATION_ORIENTATION_VALID_BIT | XR_SPACE_LOCATION_POSITION_VALID_BIT;
    if ((loc.locationFlags & need) != need) {
        return false;
    }
    tracked = (loc.locationFlags & XR_SPACE_LOCATION_POSITION_TRACKED_BIT) != 0;
    out = toMat(loc.pose);
    return true;
}

bool locate(XrSpace space, XrSpace base, XrTime t, room::Mat34& out) {
    auto loc = xr_make<XrSpaceLocation>(XR_TYPE_SPACE_LOCATION);
    if (XR_FAILED(xrLocateSpace(space, base, t, &loc))) {
        return false;
    }
    const XrSpaceLocationFlags need = XR_SPACE_LOCATION_ORIENTATION_VALID_BIT | XR_SPACE_LOCATION_POSITION_VALID_BIT;
    if ((loc.locationFlags & need) != need) {
        return false;
    }
    out = toMat(loc.pose);
    return true;
}

std::expected<bool, std::string> renderStereo(Session& x, const room::Batch& batch, XrTime t, double nearZ, double farZ,
                                              ProjectionLayer& out, const PassthroughHook& passthrough, const Extras* extras) {
    XrViewLocateInfo vli{XR_TYPE_VIEW_LOCATE_INFO, nullptr, kViewType, t, x.stage};
    auto vs = xr_make<XrViewState>(XR_TYPE_VIEW_STATE);
    std::array<XrView, 2> xv{xr_make<XrView>(XR_TYPE_VIEW), xr_make<XrView>(XR_TYPE_VIEW)};
    std::uint32_t nv = 0;
    const bool ok = XR_SUCCEEDED(xrLocateViews(x.session, &vli, &vs, 2, &nv, xv.data())) && nv == 2 &&
                    (vs.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) != 0;
    if (!ok) {
        return false;
    }
    if (auto r = x.gpu.upload(batch, extras ? extras->overlay : nullptr); !r) {
        return std::unexpected(r.error());
    }
    const bool srgb = x.srgb();
    for (std::size_t e = 0; e < 2; e++) {
        const auto& f = xv[e].fov;
        const room::Mat4 viewM = room::mat4From34(room::inv34(toMat(xv[e].pose)));
        Push push{room::mul4(room::projectionVulkan(f.angleLeft, f.angleRight, f.angleUp, f.angleDown, nearZ, farZ), viewM),
                  {srgb ? 1.0F : 0.0F, 0, 0, 0}};
        const pt::EyeView ev = pt::eyeView(toMat(xv[e].pose), f.angleLeft, f.angleRight, f.angleUp, f.angleDown);
        bool skyOn = false;
        if (extras && extras->sky && x.gpu.sky.ready()) {
            if (const auto sp = extras->sky(e, ev)) {
                x.gpu.sky.setSkyEye(e, *sp);
                skyOn = true;
            }
        }
        if (!skyOn && passthrough && x.gpu.pt.ready()) {
            if (const auto u = passthrough(e, ev)) {
                x.gpu.pt.setEye(e, *u);
            }
        }
        if (extras && extras->panel && x.gpu.panel.ready()) {
            Push pp = push;
            pp.flags[1] = extras->panelOpacity;
            x.gpu.panel.setMeshEye(e, pp);
        }
        std::uint32_t idx = 0;
        auto ai = xr_make<XrSwapchainImageAcquireInfo>(XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO);
        PIBRIDGE_XR(xrAcquireSwapchainImage(x.eyeSc[e], &ai, &idx), "acquire");
        XrSwapchainImageWaitInfo wi{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO, nullptr, XR_INFINITE_DURATION};
        PIBRIDGE_XR(xrWaitSwapchainImage(x.eyeSc[e], &wi), "wait image");
        if (auto r = x.gpu.draw(e, idx, push, static_cast<std::uint32_t>(batch.tris.size()),
                                static_cast<std::uint32_t>(batch.lines.size()));
            !r) {
            return std::unexpected(r.error());
        }
        auto ri = xr_make<XrSwapchainImageReleaseInfo>(XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO);
        PIBRIDGE_XR(xrReleaseSwapchainImage(x.eyeSc[e], &ri), "release");
        out.views[e] = xr_make<XrCompositionLayerProjectionView>(XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW);
        out.views[e].pose = xv[e].pose;
        out.views[e].fov = xv[e].fov;
        out.views[e].subImage.swapchain = x.eyeSc[e];
        out.views[e].subImage.imageRect = {{0, 0}, {static_cast<std::int32_t>(x.size[0]), static_cast<std::int32_t>(x.size[1])}};
    }
    out.proj = xr_make<XrCompositionLayerProjection>(XR_TYPE_COMPOSITION_LAYER_PROJECTION);
    out.proj.space = x.stage;
    out.proj.viewCount = 2;
    out.proj.views = out.views.data();
    return true;
}

} // namespace pibridge::xrkit
