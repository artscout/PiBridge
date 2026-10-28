// SPDX-License-Identifier: MIT
#include "compositor_vk.h"

#include <algorithm>
#include <set>

#include "log.h"

// SPIR-V made at build time by glslangValidator --vn (see layer/CMakeLists.txt).
#include "compose_frag.h"
#include "compose_vert.h"

namespace qv {

namespace {

const char* vkName(VkResult r) {
    switch (r) {
    case VK_SUCCESS: return "VK_SUCCESS";
    case VK_TIMEOUT: return "VK_TIMEOUT";
    case VK_NOT_READY: return "VK_NOT_READY";
    case VK_ERROR_OUT_OF_HOST_MEMORY: return "VK_ERROR_OUT_OF_HOST_MEMORY";
    case VK_ERROR_OUT_OF_DEVICE_MEMORY: return "VK_ERROR_OUT_OF_DEVICE_MEMORY";
    case VK_ERROR_DEVICE_LOST: return "VK_ERROR_DEVICE_LOST";
    case VK_ERROR_FORMAT_NOT_SUPPORTED: return "VK_ERROR_FORMAT_NOT_SUPPORTED";
    default: return "VkResult";
    }
}

VkShaderModule makeModule(VkDevice d, const uint32_t* code, size_t bytes) {
    VkShaderModuleCreateInfo ci{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    ci.codeSize = bytes;
    ci.pCode = code;
    VkShaderModule m = VK_NULL_HANDLE;
    if (vkCreateShaderModule(d, &ci, nullptr, &m) != VK_SUCCESS) {
        return VK_NULL_HANDLE;
    }
    return m;
}

} // namespace

Compositor::~Compositor() {
    destroy();
}

bool Compositor::init(VkInstance, VkPhysicalDevice physicalDevice, VkDevice device, uint32_t queueFamily,
                      uint32_t queueIndex) {
    destroy();
    device_ = device;
    vkGetDeviceQueue(device_, queueFamily, queueIndex, &queue_);

    VkPhysicalDeviceProperties props{};
    vkGetPhysicalDeviceProperties(physicalDevice, &props);
    timestampPeriodNs_ = props.limits.timestampComputeAndGraphics ? props.limits.timestampPeriod : 0.0;
    logf("Vulkan device: %s (queue family %u, index %u)", props.deviceName, queueFamily, queueIndex);

    const auto fail = [this] {
        destroy();
        return false;
    };
#define VK_CHECK(call)                                                                                               \
    do {                                                                                                               \
        const VkResult r_ = (call);                                                                                    \
        if (r_ != VK_SUCCESS) {                                                                                        \
            logf("Vulkan: %s failed: %s (%d)", #call, vkName(r_), r_);                                                 \
            return fail();                                                                                             \
        }                                                                                                              \
    } while (0)

    VkCommandPoolCreateInfo pci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    pci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    pci.queueFamilyIndex = queueFamily;
    VK_CHECK(vkCreateCommandPool(device_, &pci, nullptr, &pool_));

    VkSamplerCreateInfo sci{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
    sci.magFilter = sci.minFilter = VK_FILTER_LINEAR;
    sci.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    sci.addressModeU = sci.addressModeV = sci.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sci.maxLod = 0.f;
    VK_CHECK(vkCreateSampler(device_, &sci, nullptr, &sampler_));

    VkDescriptorSetLayoutBinding b[2]{};
    for (uint32_t i = 0; i < 2; i++) {
        b[i].binding = i;
        b[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        b[i].descriptorCount = 1;
        b[i].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        b[i].pImmutableSamplers = &sampler_;
    }
    VkDescriptorSetLayoutCreateInfo dlci{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    dlci.bindingCount = 2;
    dlci.pBindings = b;
    VK_CHECK(vkCreateDescriptorSetLayout(device_, &dlci, nullptr, &setLayout_));

    VkPushConstantRange pcr{VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(ComposeParams)};
    VkPipelineLayoutCreateInfo plci{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    plci.setLayoutCount = 1;
    plci.pSetLayouts = &setLayout_;
    plci.pushConstantRangeCount = 1;
    plci.pPushConstantRanges = &pcr;
    VK_CHECK(vkCreatePipelineLayout(device_, &plci, nullptr, &pipelineLayout_));

    VkDescriptorPoolSize ps{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 2 * maxEyes_ * kSlots};
    VkDescriptorPoolCreateInfo dpci{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    dpci.maxSets = maxEyes_ * kSlots;
    dpci.poolSizeCount = 1;
    dpci.pPoolSizes = &ps;
    VK_CHECK(vkCreateDescriptorPool(device_, &dpci, nullptr, &descriptorPool_));

    vert_ = makeModule(device_, compose_vert, sizeof(compose_vert));
    frag_ = makeModule(device_, compose_frag, sizeof(compose_frag));
    if (!vert_ || !frag_) {
        logf("Vulkan: shader modules failed");
        return fail();
    }

    if (timestampPeriodNs_ > 0) {
        VkQueryPoolCreateInfo qci{VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO};
        qci.queryType = VK_QUERY_TYPE_TIMESTAMP;
        qci.queryCount = 2 * kSlots;
        if (vkCreateQueryPool(device_, &qci, nullptr, &queries_) != VK_SUCCESS) {
            queries_ = VK_NULL_HANDLE;
        }
    }

    for (Slot& s : slots_) {
        VkCommandBufferAllocateInfo cai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        cai.commandPool = pool_;
        cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        cai.commandBufferCount = 1;
        VK_CHECK(vkAllocateCommandBuffers(device_, &cai, &s.cmd));
        VkFenceCreateInfo fci{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        VK_CHECK(vkCreateFence(device_, &fci, nullptr, &s.fence));
        VkDescriptorSetLayout layouts[2] = {setLayout_, setLayout_};
        VkDescriptorSetAllocateInfo dai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        dai.descriptorPool = descriptorPool_;
        dai.descriptorSetCount = maxEyes_;
        dai.pSetLayouts = layouts;
        VK_CHECK(vkAllocateDescriptorSets(device_, &dai, s.sets.data()));
    }
    return true;
#undef VK_CHECK
}

void Compositor::destroy() {
    if (!device_) {
        return;
    }
    for (Slot& s : slots_) {
        if (s.fence) {
            if (s.submitted) {
                vkWaitForFences(device_, 1, &s.fence, VK_TRUE, 1'000'000'000);
            }
            vkDestroyFence(device_, s.fence, nullptr);
        }
        s = Slot{};
    }
    for (auto& [k, fb] : framebuffers_) {
        vkDestroyFramebuffer(device_, fb.first, nullptr);
    }
    framebuffers_.clear();
    for (auto& [k, v] : views_) {
        vkDestroyImageView(device_, v, nullptr);
    }
    views_.clear();
    for (auto& [f, p] : pipes_) {
        vkDestroyPipeline(device_, p.pipeline, nullptr);
        vkDestroyRenderPass(device_, p.renderPass, nullptr);
    }
    pipes_.clear();
    if (queries_) vkDestroyQueryPool(device_, queries_, nullptr);
    if (vert_) vkDestroyShaderModule(device_, vert_, nullptr);
    if (frag_) vkDestroyShaderModule(device_, frag_, nullptr);
    if (descriptorPool_) vkDestroyDescriptorPool(device_, descriptorPool_, nullptr);
    if (pipelineLayout_) vkDestroyPipelineLayout(device_, pipelineLayout_, nullptr);
    if (setLayout_) vkDestroyDescriptorSetLayout(device_, setLayout_, nullptr);
    if (sampler_) vkDestroySampler(device_, sampler_, nullptr);
    if (pool_) vkDestroyCommandPool(device_, pool_, nullptr);
    queries_ = VK_NULL_HANDLE;
    vert_ = frag_ = VK_NULL_HANDLE;
    descriptorPool_ = VK_NULL_HANDLE;
    pipelineLayout_ = VK_NULL_HANDLE;
    setLayout_ = VK_NULL_HANDLE;
    sampler_ = VK_NULL_HANDLE;
    pool_ = VK_NULL_HANDLE;
    queue_ = VK_NULL_HANDLE;
    device_ = VK_NULL_HANDLE;
}

Compositor::Pipe* Compositor::pipeFor(VkFormat format) {
    if (auto it = pipes_.find(format); it != pipes_.end()) {
        return &it->second;
    }
    Pipe p;
    VkAttachmentDescription att{};
    att.format = format;
    att.samples = VK_SAMPLE_COUNT_1_BIT;
    att.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE; // every pixel is written
    att.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    att.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    att.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    att.initialLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL; // as acquired from the runtime
    att.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;   // as the runtime wants it back
    VkAttachmentReference ref{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkSubpassDescription sub{};
    sub.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    sub.colorAttachmentCount = 1;
    sub.pColorAttachments = &ref;
    VkSubpassDependency deps[2]{};
    deps[0].srcSubpass = VK_SUBPASS_EXTERNAL;
    deps[0].dstSubpass = 0;
    deps[0].srcStageMask = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT;
    deps[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    deps[0].srcAccessMask = VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT;
    deps[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    deps[1].srcSubpass = 0;
    deps[1].dstSubpass = VK_SUBPASS_EXTERNAL;
    deps[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    deps[1].dstStageMask = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT;
    deps[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    deps[1].dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
    VkRenderPassCreateInfo rpci{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
    rpci.attachmentCount = 1;
    rpci.pAttachments = &att;
    rpci.subpassCount = 1;
    rpci.pSubpasses = &sub;
    rpci.dependencyCount = 2;
    rpci.pDependencies = deps;
    if (vkCreateRenderPass(device_, &rpci, nullptr, &p.renderPass) != VK_SUCCESS) {
        logf("Vulkan: render pass for format %d failed", format);
        return nullptr;
    }

    VkPipelineShaderStageCreateInfo stages[2]{};
    stages[0].sType = stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = vert_;
    stages[0].pName = "main";
    stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = frag_;
    stages[1].pName = "main";
    VkPipelineVertexInputStateCreateInfo vi{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    VkPipelineViewportStateCreateInfo vp{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    vp.viewportCount = 1;
    vp.scissorCount = 1;
    VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rs.polygonMode = VK_POLYGON_MODE_FILL;
    rs.cullMode = VK_CULL_MODE_NONE;
    rs.lineWidth = 1.f;
    VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    VkPipelineColorBlendAttachmentState cba{};
    cba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT |
                         VK_COLOR_COMPONENT_A_BIT;
    VkPipelineColorBlendStateCreateInfo cb{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    cb.attachmentCount = 1;
    cb.pAttachments = &cba;
    VkDynamicState dyn[2] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo ds{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    ds.dynamicStateCount = 2;
    ds.pDynamicStates = dyn;
    VkGraphicsPipelineCreateInfo gci{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    gci.stageCount = 2;
    gci.pStages = stages;
    gci.pVertexInputState = &vi;
    gci.pInputAssemblyState = &ia;
    gci.pViewportState = &vp;
    gci.pRasterizationState = &rs;
    gci.pMultisampleState = &ms;
    gci.pColorBlendState = &cb;
    gci.pDynamicState = &ds;
    gci.layout = pipelineLayout_;
    gci.renderPass = p.renderPass;
    if (vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &gci, nullptr, &p.pipeline) != VK_SUCCESS) {
        logf("Vulkan: pipeline for format %d failed", format);
        vkDestroyRenderPass(device_, p.renderPass, nullptr);
        return nullptr;
    }
    logf("Vulkan: pipeline for format %d", format);
    return &pipes_.emplace(format, p).first->second;
}

VkImageView Compositor::viewFor(const ImageRef& r) {
    const ViewKey key{r.image, r.layer, r.format};
    if (auto it = views_.find(key); it != views_.end()) {
        return it->second;
    }
    VkImageViewCreateInfo ci{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    ci.image = r.image;
    ci.viewType = VK_IMAGE_VIEW_TYPE_2D;
    ci.format = r.format;
    ci.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, r.layer, 1};
    VkImageView v = VK_NULL_HANDLE;
    if (vkCreateImageView(device_, &ci, nullptr, &v) != VK_SUCCESS) {
        logf("Vulkan: image view (format %d, layer %u) failed", r.format, r.layer);
        return VK_NULL_HANDLE;
    }
    views_.emplace(key, v);
    return v;
}

VkFramebuffer Compositor::framebufferFor(const ImageRef& r, VkRenderPass rp) {
    const VkImageView view = viewFor(r);
    if (!view) {
        return VK_NULL_HANDLE;
    }
    const auto key = std::make_pair(view, rp);
    if (auto it = framebuffers_.find(key); it != framebuffers_.end()) {
        return it->second.first;
    }
    VkFramebufferCreateInfo ci{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
    ci.renderPass = rp;
    ci.attachmentCount = 1;
    ci.pAttachments = &view;
    ci.width = r.width;
    ci.height = r.height;
    ci.layers = 1;
    VkFramebuffer fb = VK_NULL_HANDLE;
    if (vkCreateFramebuffer(device_, &ci, nullptr, &fb) != VK_SUCCESS) {
        logf("Vulkan: framebuffer %ux%u failed", r.width, r.height);
        return VK_NULL_HANDLE;
    }
    framebuffers_.emplace(key, std::make_pair(fb, r.image));
    return fb;
}

void Compositor::forgetImage(VkImage image) {
    if (!device_) {
        return;
    }
    // The views may still be used by a submitted composition.
    for (Slot& s : slots_) {
        if (s.submitted) {
            vkWaitForFences(device_, 1, &s.fence, VK_TRUE, 1'000'000'000);
        }
    }
    for (auto it = framebuffers_.begin(); it != framebuffers_.end();) {
        if (it->second.second == image) {
            vkDestroyFramebuffer(device_, it->second.first, nullptr);
            it = framebuffers_.erase(it);
        } else {
            ++it;
        }
    }
    for (auto it = views_.begin(); it != views_.end();) {
        if (std::get<0>(it->first) == image) {
            vkDestroyImageView(device_, it->second, nullptr);
            it = views_.erase(it);
        } else {
            ++it;
        }
    }
}

bool Compositor::compose(const EyeJob* jobs, uint32_t count) {
    if (!device_ || count == 0 || count > maxEyes_) {
        return false;
    }
#define VK_CHECK_F(call)                                                                                               \
    do {                                                                                                               \
        const VkResult r_ = (call);                                                                                    \
        if (r_ != VK_SUCCESS) {                                                                                        \
            logf("Vulkan: %s failed: %s (%d)", #call, vkName(r_), r_);                                                 \
            return false;                                                                                              \
        }                                                                                                              \
    } while (0)

    const uint32_t si = next_;
    Slot& s = slots_[si];
    if (s.submitted) {
        // Three frames back: long finished in practice.
        VK_CHECK_F(vkWaitForFences(device_, 1, &s.fence, VK_TRUE, 1'000'000'000));
        if (queries_) {
            uint64_t ts[2] = {};
            if (vkGetQueryPoolResults(device_, queries_, 2 * si, 2, sizeof(ts), ts, sizeof(uint64_t),
                                      VK_QUERY_RESULT_64_BIT) == VK_SUCCESS &&
                ts[1] >= ts[0]) {
                lastGpuMs_ = double(ts[1] - ts[0]) * timestampPeriodNs_ * 1e-6;
            }
        }
        s.submitted = false;
    }
    VK_CHECK_F(vkResetFences(device_, 1, &s.fence));

    // Descriptors of this slot (its previous use has finished).
    for (uint32_t e = 0; e < count; e++) {
        const VkImageView pv = viewFor(jobs[e].peripheral);
        const VkImageView fv = viewFor(jobs[e].focus);
        if (!pv || !fv) {
            return false;
        }
        VkDescriptorImageInfo ii[2] = {{VK_NULL_HANDLE, pv, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL},
                                       {VK_NULL_HANDLE, fv, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL}};
        VkWriteDescriptorSet w[2]{};
        for (uint32_t i = 0; i < 2; i++) {
            w[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            w[i].dstSet = s.sets[e];
            w[i].dstBinding = i;
            w[i].descriptorCount = 1;
            w[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            w[i].pImageInfo = &ii[i];
        }
        vkUpdateDescriptorSets(device_, 2, w, 0, nullptr);
    }

    VK_CHECK_F(vkResetCommandBuffer(s.cmd, 0));
    VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    VK_CHECK_F(vkBeginCommandBuffer(s.cmd, &bi));
    if (queries_) {
        vkCmdResetQueryPool(s.cmd, queries_, 2 * si, 2);
        vkCmdWriteTimestamp(s.cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, queries_, 2 * si);
    }

    // The game's images (each image layer once): attachment -> sampled, and back after.
    std::set<std::pair<VkImage, uint32_t>> sources;
    for (uint32_t e = 0; e < count; e++) {
        sources.insert({jobs[e].peripheral.image, jobs[e].peripheral.layer});
        sources.insert({jobs[e].focus.image, jobs[e].focus.layer});
    }
    std::vector<VkImageMemoryBarrier> toRead, toAttachment;
    for (const auto& [img, layer] : sources) {
        VkImageMemoryBarrier b{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.image = img;
        b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, layer, 1};
        b.srcAccessMask = VK_ACCESS_MEMORY_WRITE_BIT;
        b.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        b.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        b.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        toRead.push_back(b);
        b.srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
        b.dstAccessMask = VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT;
        b.oldLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        b.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        toAttachment.push_back(b);
    }
    vkCmdPipelineBarrier(s.cmd, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0,
                         nullptr, 0, nullptr, uint32_t(toRead.size()), toRead.data());

    for (uint32_t e = 0; e < count; e++) {
        const EyeJob& j = jobs[e];
        Pipe* p = pipeFor(j.target.format);
        if (!p) {
            vkEndCommandBuffer(s.cmd);
            return false;
        }
        const VkFramebuffer fb = framebufferFor(j.target, p->renderPass);
        if (!fb) {
            vkEndCommandBuffer(s.cmd);
            return false;
        }
        VkRenderPassBeginInfo rbi{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
        rbi.renderPass = p->renderPass;
        rbi.framebuffer = fb;
        rbi.renderArea = {{0, 0}, {j.target.width, j.target.height}};
        vkCmdBeginRenderPass(s.cmd, &rbi, VK_SUBPASS_CONTENTS_INLINE);
        vkCmdBindPipeline(s.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, p->pipeline);
        const VkViewport vpt{0.f, 0.f, float(j.target.width), float(j.target.height), 0.f, 1.f};
        vkCmdSetViewport(s.cmd, 0, 1, &vpt);
        vkCmdSetScissor(s.cmd, 0, 1, &rbi.renderArea);
        vkCmdBindDescriptorSets(s.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout_, 0, 1, &s.sets[e], 0,
                                nullptr);
        vkCmdPushConstants(s.cmd, pipelineLayout_, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(ComposeParams),
                           &j.params);
        vkCmdDraw(s.cmd, 3, 1, 0, 0);
        vkCmdEndRenderPass(s.cmd);
    }

    vkCmdPipelineBarrier(s.cmd, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0,
                         nullptr, 0, nullptr, uint32_t(toAttachment.size()), toAttachment.data());
    if (queries_) {
        vkCmdWriteTimestamp(s.cmd, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, queries_, 2 * si + 1);
    }
    VK_CHECK_F(vkEndCommandBuffer(s.cmd));

    VkSubmitInfo sub{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    sub.commandBufferCount = 1;
    sub.pCommandBuffers = &s.cmd;
    VK_CHECK_F(vkQueueSubmit(queue_, 1, &sub, s.fence));
    s.submitted = true;
    next_ = (next_ + 1) % kSlots;
    return true;
#undef VK_CHECK_F
}

} // namespace qv
