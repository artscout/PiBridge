// SPDX-License-Identifier: MIT
// xrkit's textured passes (xrkit.h TexPass): pibridge-home's equirect sky and its curved PiBridge-window panel.
#include "pibridge/xrkit/xrkit.h"

#include "panel_frag.h"
#include "panel_vert.h"
#include "sky_frag.h"
#include "sky_vert.h"

#include <algorithm>
#include <cstring>
#include <tuple>

namespace pibridge::xrkit {

namespace {

std::optional<std::uint32_t> memType(const VkPhysicalDeviceMemoryProperties& p, std::uint32_t bits, VkMemoryPropertyFlags flags) {
    for (std::uint32_t i = 0; i < p.memoryTypeCount; i++) {
        if ((bits & (1U << i)) != 0 && (p.memoryTypes[i].propertyFlags & flags) == flags) {
            return i;
        }
    }
    return std::nullopt;
}

std::expected<std::pair<VkBuffer, VkDeviceMemory>, std::string> hostBuf(VkDevice device, const VkPhysicalDeviceMemoryProperties& mp,
                                                                       VkDeviceSize size, VkBufferUsageFlags usage) {
    auto bi = vk_make<VkBufferCreateInfo>(VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO);
    bi.size = size;
    bi.usage = usage;
    bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    VkBuffer b = VK_NULL_HANDLE;
    PIBRIDGE_VK(vkCreateBuffer(device, &bi, nullptr, &b), "texture buffer");
    VkMemoryRequirements req;
    vkGetBufferMemoryRequirements(device, b, &req);
    const auto mt = memType(mp, req.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    if (!mt) {
        vkDestroyBuffer(device, b, nullptr);
        return std::unexpected(std::string("no host visible memory for a texture"));
    }
    VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO, nullptr, req.size, *mt};
    VkDeviceMemory m = VK_NULL_HANDLE;
    if (const VkResult r = vkAllocateMemory(device, &mai, nullptr, &m); r != VK_SUCCESS) {
        vkDestroyBuffer(device, b, nullptr);
        return std::unexpected(vkErr("texture buffer memory", r));
    }
    PIBRIDGE_VK(vkBindBufferMemory(device, b, m, 0), "texture buffer bind");
    return std::pair{b, m};
}

} // namespace

std::expected<void, std::string> TexPass::create(VkDevice dev, const VkPhysicalDeviceMemoryProperties& mp, VkRenderPass rp,
                                                 std::array<std::uint32_t, 2> size, Kind k, VkFormat fmt) {
    device = dev;
    memProps = mp;
    kind = k;
    format = fmt;
    auto si = vk_make<VkSamplerCreateInfo>(VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO);
    si.magFilter = VK_FILTER_LINEAR;
    si.minFilter = VK_FILTER_LINEAR;
    si.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    // the sky wraps around in longitude; the panel is clamped
    si.addressModeU = kind == Kind::Sky ? VK_SAMPLER_ADDRESS_MODE_REPEAT : VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    si.addressModeV = si.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    si.maxLod = 0.0F;
    PIBRIDGE_VK(vkCreateSampler(device, &si, nullptr, &sampler), "texture sampler");
    const VkDescriptorSetLayoutBinding b{0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr};
    auto dli = vk_make<VkDescriptorSetLayoutCreateInfo>(VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO);
    dli.bindingCount = 1;
    dli.pBindings = &b;
    PIBRIDGE_VK(vkCreateDescriptorSetLayout(device, &dli, nullptr, &dsl), "texture set layout");
    const VkDescriptorPoolSize ps{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1};
    auto dpi = vk_make<VkDescriptorPoolCreateInfo>(VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO);
    dpi.maxSets = 1;
    dpi.poolSizeCount = 1;
    dpi.pPoolSizes = &ps;
    PIBRIDGE_VK(vkCreateDescriptorPool(device, &dpi, nullptr, &pool), "texture descriptor pool");
    auto dai = vk_make<VkDescriptorSetAllocateInfo>(VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO);
    dai.descriptorPool = pool;
    dai.descriptorSetCount = 1;
    dai.pSetLayouts = &dsl;
    PIBRIDGE_VK(vkAllocateDescriptorSets(device, &dai, &set), "texture descriptor set");
    const std::uint32_t pushSize = kind == Kind::Sky ? sizeof(SkyPush) : sizeof(Push);
    const VkPushConstantRange pcr{VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, pushSize};
    VkPipelineLayoutCreateInfo pli{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO, nullptr, 0, 1, &dsl, 1, &pcr};
    PIBRIDGE_VK(vkCreatePipelineLayout(device, &pli, nullptr, &layout), "texture pipeline layout");

    VkShaderModuleCreateInfo vsi{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO, nullptr, 0, 0, nullptr};
    VkShaderModuleCreateInfo fsi = vsi;
    if (kind == Kind::Sky) {
        vsi.codeSize = sizeof(sky_vert);
        vsi.pCode = sky_vert;
        fsi.codeSize = sizeof(sky_frag);
        fsi.pCode = sky_frag;
    } else {
        vsi.codeSize = sizeof(panel_vert);
        vsi.pCode = panel_vert;
        fsi.codeSize = sizeof(panel_frag);
        fsi.pCode = panel_frag;
    }
    VkShaderModule vs = VK_NULL_HANDLE;
    VkShaderModule fs = VK_NULL_HANDLE;
    PIBRIDGE_VK(vkCreateShaderModule(device, &vsi, nullptr, &vs), "texture vertex shader");
    PIBRIDGE_VK(vkCreateShaderModule(device, &fsi, nullptr, &fs), "texture fragment shader");
    VkPipelineShaderStageCreateInfo stages[2] = {
        {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_VERTEX_BIT, vs, "main", nullptr},
        {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_FRAGMENT_BIT, fs, "main", nullptr}};
    const VkVertexInputBindingDescription bind{0, sizeof(TexVertex), VK_VERTEX_INPUT_RATE_VERTEX};
    const std::array<VkVertexInputAttributeDescription, 2> attrs{{{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},
                                                                  {1, 0, VK_FORMAT_R32G32_SFLOAT, 12}}};
    auto vin = vk_make<VkPipelineVertexInputStateCreateInfo>(VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO);
    if (kind == Kind::Mesh) {
        vin.vertexBindingDescriptionCount = 1;
        vin.pVertexBindingDescriptions = &bind;
        vin.vertexAttributeDescriptionCount = static_cast<std::uint32_t>(attrs.size());
        vin.pVertexAttributeDescriptions = attrs.data();
    }
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
    ba.blendEnable = kind == Kind::Mesh ? VK_TRUE : VK_FALSE;
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
    gpi.renderPass = rp;
    const VkResult r = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &gpi, nullptr, &pipe);
    vkDestroyShaderModule(device, vs, nullptr);
    vkDestroyShaderModule(device, fs, nullptr);
    PIBRIDGE_VK(r, "texture pipeline");
    if (kind == Kind::Mesh) {
        auto vb = hostBuf(device, memProps, sizeof(TexVertex) * kMaxVerts, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
        if (!vb) {
            return std::unexpected(vb.error());
        }
        std::tie(vbuf, vmem) = *vb;
        PIBRIDGE_VK(vkMapMemory(device, vmem, 0, VK_WHOLE_SIZE, 0, &vptr), "panel vertex map");
    }
    return {};
}

void TexPass::freeImage() {
    if (device == VK_NULL_HANDLE) {
        return;
    }
    vkDestroyImageView(device, view, nullptr);
    vkDestroyImage(device, img, nullptr);
    vkFreeMemory(device, imem, nullptr);
    if (smem != VK_NULL_HANDLE) {
        vkUnmapMemory(device, smem);
    }
    vkDestroyBuffer(device, staging, nullptr);
    vkFreeMemory(device, smem, nullptr);
    view = VK_NULL_HANDLE;
    img = VK_NULL_HANDLE;
    imem = VK_NULL_HANDLE;
    staging = VK_NULL_HANDLE;
    smem = VK_NULL_HANDLE;
    sptr = nullptr;
    w = h = 0;
    initialised = haveImage = dirty = false;
}

std::expected<void, std::string> TexPass::setPixels(const std::uint8_t* src, std::uint32_t width, std::uint32_t height,
                                                    std::size_t stride) {
    if (!ready() || src == nullptr || width == 0 || height == 0) {
        return {};
    }
    if (width != w || height != h) {
        freeImage();
        auto st = hostBuf(device, memProps, static_cast<VkDeviceSize>(width) * height * 4, VK_BUFFER_USAGE_TRANSFER_SRC_BIT);
        if (!st) {
            return std::unexpected(st.error());
        }
        std::tie(staging, smem) = *st;
        PIBRIDGE_VK(vkMapMemory(device, smem, 0, VK_WHOLE_SIZE, 0, &sptr), "texture staging map");
        auto ii = vk_make<VkImageCreateInfo>(VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO);
        ii.imageType = VK_IMAGE_TYPE_2D;
        ii.format = format;
        ii.extent = {width, height, 1};
        ii.mipLevels = 1;
        ii.arrayLayers = 1;
        ii.samples = VK_SAMPLE_COUNT_1_BIT;
        ii.tiling = VK_IMAGE_TILING_OPTIMAL;
        ii.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        ii.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        ii.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        PIBRIDGE_VK(vkCreateImage(device, &ii, nullptr, &img), "texture image");
        VkMemoryRequirements req;
        vkGetImageMemoryRequirements(device, img, &req);
        auto mt = memType(memProps, req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        if (!mt) {
            mt = memType(memProps, req.memoryTypeBits, 0);
        }
        if (!mt) {
            return std::unexpected(std::string("no memory for a texture"));
        }
        VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO, nullptr, req.size, *mt};
        PIBRIDGE_VK(vkAllocateMemory(device, &mai, nullptr, &imem), "texture image memory");
        PIBRIDGE_VK(vkBindImageMemory(device, img, imem, 0), "texture image bind");
        auto vi = vk_make<VkImageViewCreateInfo>(VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO);
        vi.image = img;
        vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
        vi.format = format;
        vi.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        PIBRIDGE_VK(vkCreateImageView(device, &vi, nullptr, &view), "texture image view");
        const VkDescriptorImageInfo info{sampler, view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        auto wr = vk_make<VkWriteDescriptorSet>(VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET);
        wr.dstSet = set;
        wr.dstBinding = 0;
        wr.descriptorCount = 1;
        wr.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        wr.pImageInfo = &info;
        vkUpdateDescriptorSets(device, 1, &wr, 0, nullptr);
        w = width;
        h = height;
    }
    auto* dst = static_cast<std::uint8_t*>(sptr);
    const std::size_t row = static_cast<std::size_t>(width) * 4;
    if (stride == row) {
        std::memcpy(dst, src, row * height);
    } else {
        for (std::uint32_t y = 0; y < height; y++) {
            std::memcpy(dst + y * row, src + y * stride, row);
        }
    }
    dirty = true;
    return {};
}

void TexPass::setMesh(const std::vector<TexVertex>& v) {
    if (vptr == nullptr) {
        return;
    }
    vcount = static_cast<std::uint32_t>(std::min<std::size_t>(v.size() / 3 * 3, kMaxVerts));
    std::memcpy(vptr, v.data(), vcount * sizeof(TexVertex));
}

void TexPass::setSkyEye(std::size_t eye, const SkyPush& p) {
    if (eye < 2) {
        sky[eye] = p;
        eyeOn[eye] = true;
    }
}

void TexPass::setMeshEye(std::size_t eye, const Push& p) {
    if (eye < 2) {
        mesh[eye] = p;
        eyeOn[eye] = true;
    }
}

void TexPass::recordUpload(VkCommandBuffer cmd) {
    if (!dirty || !ready() || img == VK_NULL_HANDLE) {
        return;
    }
    auto b = vk_make<VkImageMemoryBarrier>(VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER);
    b.srcAccessMask = initialised ? VK_ACCESS_SHADER_READ_BIT : 0;
    b.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    b.oldLayout = initialised ? VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL : VK_IMAGE_LAYOUT_UNDEFINED;
    b.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b.image = img;
    b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                         VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &b);
    VkBufferImageCopy r{};
    r.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    r.imageExtent = {w, h, 1};
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

void TexPass::recordDraw(VkCommandBuffer cmd, std::size_t eye) {
    if (!ready() || !haveImage || eye > 1 || !eyeOn[eye]) {
        return;
    }
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, 1, &set, 0, nullptr);
    const VkShaderStageFlags st = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    if (kind == Kind::Sky) {
        vkCmdPushConstants(cmd, layout, st, 0, sizeof(SkyPush), &sky[eye]);
        vkCmdDraw(cmd, 3, 1, 0, 0);
    } else if (vcount > 0) {
        vkCmdPushConstants(cmd, layout, st, 0, sizeof(Push), &mesh[eye]);
        const VkDeviceSize off = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &vbuf, &off);
        vkCmdDraw(cmd, vcount, 1, 0, 0);
    }
}

void TexPass::destroy() {
    if (device == VK_NULL_HANDLE) {
        return;
    }
    freeImage();
    vkDestroyPipeline(device, pipe, nullptr);
    vkDestroyPipelineLayout(device, layout, nullptr);
    vkDestroyDescriptorPool(device, pool, nullptr);
    vkDestroyDescriptorSetLayout(device, dsl, nullptr);
    vkDestroySampler(device, sampler, nullptr);
    if (vmem != VK_NULL_HANDLE) {
        vkUnmapMemory(device, vmem);
    }
    vkDestroyBuffer(device, vbuf, nullptr);
    vkFreeMemory(device, vmem, nullptr);
    *this = TexPass{};
}

} // namespace pibridge::xrkit
