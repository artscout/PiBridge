// SPDX-License-Identifier: MIT
// pibridge-xroverlay: the Monado backend's overlay (XR_EXTX_overlay), composited by Monado over whatever app runs - a
// game, the home. It shows the volume indicator after a volume button press (volume.h), the loading dots, and the recenter ring: while the Pimax button is held, the Pimax driver writes the hold
// progress 0..1 to $XDG_RUNTIME_DIR/pibridge/recenter-progress (as pibridge-headtrack does under SteamVR, where
// pibridge-overlay draws the same ring); a ring filling to completion floats 0.7 m in front of the head.
//
// The ring's frames are one sprite sheet (pibridge-overlay's drawRingSheet) in a static swapchain, uploaded once; a frame
// is a quad layer showing one cell (subImage.imageRect), so nothing is drawn or uploaded per frame.
#include "overlay_core.h"
#include "pibridge/xrkit/xrkit.h"
#include "loading.h"
#include "volume.h"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace pibridge;

using pibridge::overlay::drawRingSheet;
using pibridge::overlay::kRingFrames;
using pibridge::overlay::kRingGrid;
using pibridge::overlay::kRingSize;

volatile std::sig_atomic_t g_stop = 0;
void onSignal(int) {
    g_stop = 1;
}

int fail(const std::string& why) {
    std::fprintf(stderr, "pibridge-xroverlay: %s\n", why.c_str());
    return 1;
}

double readProgress() {
    const char* rt = std::getenv("XDG_RUNTIME_DIR");
    std::ifstream f(std::string(rt != nullptr && *rt != '\0' ? rt : "/tmp") + "/pibridge/recenter-progress");
    double p = 0.0;
    if (!(f >> p)) {
        return 0.0;
    }
    return p;
}

// One-time upload of RGBA pixels into a swapchain image (TRANSFER, then COLOR_ATTACHMENT_OPTIMAL for the runtime).
std::expected<void, std::string> upload(xrkit::Session& x, VkImage img, const std::vector<std::uint8_t>& px, std::uint32_t w, std::uint32_t h) {
    VkBuffer buf = VK_NULL_HANDLE;
    VkDeviceMemory mem = VK_NULL_HANDLE;
    auto bci = xrkit::vk_make<VkBufferCreateInfo>(VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO);
    bci.size = px.size();
    bci.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    if (vkCreateBuffer(x.device, &bci, nullptr, &buf) != VK_SUCCESS) {
        return std::unexpected(std::string("staging buffer"));
    }
    VkMemoryRequirements req{};
    vkGetBufferMemoryRequirements(x.device, buf, &req);
    std::uint32_t type = UINT32_MAX;
    for (std::uint32_t i = 0; i < x.memProps.memoryTypeCount; i++) {
        const auto want = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
        if ((req.memoryTypeBits & (1u << i)) != 0 && (x.memProps.memoryTypes[i].propertyFlags & want) == want) {
            type = i;
            break;
        }
    }
    auto mai = xrkit::vk_make<VkMemoryAllocateInfo>(VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO);
    mai.allocationSize = req.size;
    mai.memoryTypeIndex = type;
    if (type == UINT32_MAX || vkAllocateMemory(x.device, &mai, nullptr, &mem) != VK_SUCCESS) {
        vkDestroyBuffer(x.device, buf, nullptr);
        return std::unexpected(std::string("staging memory"));
    }
    vkBindBufferMemory(x.device, buf, mem, 0);
    void* map = nullptr;
    vkMapMemory(x.device, mem, 0, px.size(), 0, &map);
    std::memcpy(map, px.data(), px.size());
    vkUnmapMemory(x.device, mem);

    VkCommandPool pool = VK_NULL_HANDLE;
    auto pci = xrkit::vk_make<VkCommandPoolCreateInfo>(VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO);
    pci.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
    pci.queueFamilyIndex = x.qf;
    vkCreateCommandPool(x.device, &pci, nullptr, &pool);
    VkCommandBuffer cb = VK_NULL_HANDLE;
    auto cai = xrkit::vk_make<VkCommandBufferAllocateInfo>(VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO);
    cai.commandPool = pool;
    cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cai.commandBufferCount = 1;
    vkAllocateCommandBuffers(x.device, &cai, &cb);
    auto bi = xrkit::vk_make<VkCommandBufferBeginInfo>(VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO);
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cb, &bi);
    const auto barrier = [&](VkImageLayout from, VkImageLayout to, VkAccessFlags src, VkAccessFlags dst,
                             VkPipelineStageFlags s0, VkPipelineStageFlags s1) {
        auto b = xrkit::vk_make<VkImageMemoryBarrier>(VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER);
        b.srcAccessMask = src;
        b.dstAccessMask = dst;
        b.oldLayout = from;
        b.newLayout = to;
        b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.image = img;
        b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        vkCmdPipelineBarrier(cb, s0, s1, 0, 0, nullptr, 0, nullptr, 1, &b);
    };
    barrier(VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 0, VK_ACCESS_TRANSFER_WRITE_BIT,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT);
    VkBufferImageCopy copy{};
    copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    copy.imageExtent = {w, h, 1};
    vkCmdCopyBufferToImage(cb, buf, img, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
    barrier(VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_ACCESS_TRANSFER_WRITE_BIT,
            VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT);
    vkEndCommandBuffer(cb);
    auto si = xrkit::vk_make<VkSubmitInfo>(VK_STRUCTURE_TYPE_SUBMIT_INFO);
    si.commandBufferCount = 1;
    si.pCommandBuffers = &cb;
    vkQueueSubmit(x.queue, 1, &si, VK_NULL_HANDLE);
    vkQueueWaitIdle(x.queue);
    vkDestroyCommandPool(x.device, pool, nullptr);
    vkDestroyBuffer(x.device, buf, nullptr);
    vkFreeMemory(x.device, mem, nullptr);
    return {};
}

} // namespace

int main(int argc, char** argv) {
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        if (a == "--help" || a == "-h") {
            std::printf("pibridge-xroverlay: the Monado backend's overlay over every app (XR_EXTX_overlay): the recenter\n"
                        "ring while the Pimax button is held ($XDG_RUNTIME_DIR/pibridge/recenter-progress) and the loading\n"
                        "dots while PiBridge World starts again after a game ($XDG_RUNTIME_DIR/pibridge/loading).\n"
                        "PiBridge's Monado session starts it with XR_RUNTIME_JSON set; it ends on SIGTERM.\n\n"
                        "  pibridge-xroverlay          run (needs a running Monado)\n"
                        "  pibridge-xroverlay --help\n");
            return 0;
        }
        std::fprintf(stderr, "pibridge-xroverlay: unknown argument %s (see --help)\n", a.c_str());
        return 2;
    }
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    xrkit::Session x;
    xrkit::OpenOpts o;
    o.appName = "pibridge-xroverlay";
    o.runtimeHint = "is Monado (the Monado backend's session) running?";
    o.overlay = true;
    o.overlayPlacement = 10; // over the home's and any other overlay
    if (auto r = xrkit::openSession(x, o); !r) {
        return fail(r.error());
    }

    // the sprite sheets: RGBA straight alpha, sRGB values, each in a static swapchain uploaded once
    std::uint32_t n = 0;
    xrEnumerateSwapchainFormats(x.session, 0, &n, nullptr);
    std::vector<std::int64_t> formats(n);
    xrEnumerateSwapchainFormats(x.session, n, &n, formats.data());
    const auto fmt = xrkit::chooseFormat(formats);
    if (!fmt) {
        return fail("no RGBA/BGRA swapchain format");
    }
    const bool bgra = *fmt == VK_FORMAT_B8G8R8A8_SRGB || *fmt == VK_FORMAT_B8G8R8A8_UNORM;
    const auto staticSwapchain = [&](std::vector<std::uint8_t> px, std::uint32_t w, std::uint32_t h,
                                     const char* what) -> std::expected<XrSwapchain, std::string> {
        if (bgra) {
            for (std::size_t i = 0; i + 3 < px.size(); i += 4) {
                std::swap(px[i], px[i + 2]);
            }
        }
        auto sci = xrkit::xr_make<XrSwapchainCreateInfo>(XR_TYPE_SWAPCHAIN_CREATE_INFO);
        sci.createFlags = XR_SWAPCHAIN_CREATE_STATIC_IMAGE_BIT;
        sci.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
        sci.format = *fmt;
        sci.sampleCount = 1;
        sci.width = w;
        sci.height = h;
        sci.faceCount = 1;
        sci.arraySize = 1;
        sci.mipCount = 1;
        XrSwapchain sw = XR_NULL_HANDLE;
        if (XR_FAILED(xrCreateSwapchain(x.session, &sci, &sw))) {
            return std::unexpected(std::string("xrCreateSwapchain (") + what + ")");
        }
        std::uint32_t k = 0;
        xrEnumerateSwapchainImages(sw, 0, &k, nullptr);
        std::vector<XrSwapchainImageVulkan2KHR> imgs(k, xrkit::xr_make<XrSwapchainImageVulkan2KHR>(XR_TYPE_SWAPCHAIN_IMAGE_VULKAN2_KHR));
        xrEnumerateSwapchainImages(sw, k, &k, reinterpret_cast<XrSwapchainImageBaseHeader*>(imgs.data()));
        std::uint32_t idx = 0;
        auto ai = xrkit::xr_make<XrSwapchainImageAcquireInfo>(XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO);
        xrAcquireSwapchainImage(sw, &ai, &idx);
        auto wi = xrkit::xr_make<XrSwapchainImageWaitInfo>(XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO);
        wi.timeout = XR_INFINITE_DURATION;
        xrWaitSwapchainImage(sw, &wi);
        if (auto r = upload(x, imgs[idx].image, px, w, h); !r) {
            return std::unexpected(r.error());
        }
        auto ri = xrkit::xr_make<XrSwapchainImageReleaseInfo>(XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO);
        xrReleaseSwapchainImage(sw, &ri);
        return sw;
    };
    const std::uint32_t side = kRingSize * static_cast<std::uint32_t>(kRingGrid);
    const auto ringSc = staticSwapchain(drawRingSheet(), side, side, "static ring sheet");
    if (!ringSc) {
        return fail(ringSc.error());
    }
    const XrSwapchain sc = *ringSc;
    namespace xo = pibridge::xroverlay;
    const auto dotsSc = staticSwapchain(xo::drawDotsSheet(), xo::kDotsW, xo::kDotsH * xo::kDotsFrames, "loading dots");
    if (!dotsSc) {
        return fail(dotsSc.error());
    }
    const XrSwapchain dots = *dotsSc;
    const auto volSc = staticSwapchain(xo::drawVolumeSheet(), xo::kVolW, xo::kVolH * xo::kVolFrames, "volume");
    if (!volSc) {
        return fail(volSc.error());
    }
    const XrSwapchain vol = *volSc;
    const std::string volumePath = xo::volumeHudPath();
    std::optional<int> volume;
    const std::string loadingPath = xo::loadingRequestPath();
    std::printf("pibridge-xroverlay: overlay session on %s; the recenter ring is ready\n", x.runtimeName.c_str());

    bool running = false;
    bool loading = false;
    auto loadingCheckedAt = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    const auto t0 = std::chrono::steady_clock::now();
    bool shown = false;
    int lowPolls = 0;
    int frame = 0;
    while (g_stop == 0) {
        auto ev = xrkit::xr_make<XrEventDataBuffer>(XR_TYPE_EVENT_DATA_BUFFER);
        while (xrPollEvent(x.instance, &ev) == XR_SUCCESS) {
            if (ev.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
                const auto& sc2 = *reinterpret_cast<XrEventDataSessionStateChanged*>(&ev);
                if (sc2.state == XR_SESSION_STATE_READY) {
                    auto sbi = xrkit::xr_make<XrSessionBeginInfo>(XR_TYPE_SESSION_BEGIN_INFO);
                    sbi.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
                    running = XR_SUCCEEDED(xrBeginSession(x.session, &sbi));
                } else if (sc2.state == XR_SESSION_STATE_STOPPING) {
                    xrEndSession(x.session);
                    running = false;
                } else if (sc2.state == XR_SESSION_STATE_EXITING || sc2.state == XR_SESSION_STATE_LOSS_PENDING) {
                    g_stop = 1;
                }
            } else if (ev.type == XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING) {
                g_stop = 1;
            }
            ev = xrkit::xr_make<XrEventDataBuffer>(XR_TYPE_EVENT_DATA_BUFFER);
        }
        if (!running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        auto fs = xrkit::xr_make<XrFrameState>(XR_TYPE_FRAME_STATE);
        auto fwi = xrkit::xr_make<XrFrameWaitInfo>(XR_TYPE_FRAME_WAIT_INFO);
        if (XR_FAILED(xrWaitFrame(x.session, &fwi, &fs))) {
            continue;
        }
        auto fbi = xrkit::xr_make<XrFrameBeginInfo>(XR_TYPE_FRAME_BEGIN_INFO);
        xrBeginFrame(x.session, &fbi);
        // the hold progress; hidden only after two "no hold" reads in a row (a transient read never blinks it)
        const double p = readProgress();
        if (p > 0.01) {
            frame = static_cast<int>(p * (kRingFrames - 1) + 0.5);
            frame = frame < 0 ? 0 : (frame >= kRingFrames ? kRingFrames - 1 : frame);
            shown = true;
            lowPolls = 0;
        } else if (shown && ++lowPolls >= 2) {
            shown = false;
        }
        // the loading dots while the dashboard overlay says the home is on its way (checked 4 times a second)
        const auto now = std::chrono::steady_clock::now();
        if (now - loadingCheckedAt >= std::chrono::milliseconds(250)) {
            loadingCheckedAt = now;
            const bool want = xo::loadingWanted(loadingPath);
            if (want != loading) {
                std::printf("pibridge-xroverlay: loading dots %s\n", want ? "on" : "off");
                std::fflush(stdout);
            }
            loading = want;
        }
        // the volume indicator for a moment after a volume button (keepalived writes the new value)
        volume = xo::volumeToShow(volumePath);
        auto d = xrkit::xr_make<XrCompositionLayerQuad>(XR_TYPE_COMPOSITION_LAYER_QUAD);
        d.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT | XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT;
        d.space = x.view;
        d.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
        d.subImage.swapchain = dots;
        const int df = xo::dotsFrameAt(std::chrono::duration<double>(now - t0).count());
        d.subImage.imageRect = {{0, df * static_cast<std::int32_t>(xo::kDotsH)},
                                {static_cast<std::int32_t>(xo::kDotsW), static_cast<std::int32_t>(xo::kDotsH)}};
        d.pose = {{0.0F, 0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, -1.2F}};
        d.size = {0.27F, 0.09F};
        auto q = xrkit::xr_make<XrCompositionLayerQuad>(XR_TYPE_COMPOSITION_LAYER_QUAD);
        q.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT | XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT;
        q.space = x.view; // fixed in front of the head
        q.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
        q.subImage.swapchain = sc;
        const std::int32_t cell = static_cast<std::int32_t>(kRingSize);
        q.subImage.imageRect = {{(frame % kRingGrid) * cell, (frame / kRingGrid) * cell}, {cell, cell}};
        q.pose = {{0.0F, 0.0F, 0.0F, 1.0F}, {0.0F, -0.03F, -0.7F}};
        q.size = {0.22F, 0.22F};
        auto v = xrkit::xr_make<XrCompositionLayerQuad>(XR_TYPE_COMPOSITION_LAYER_QUAD);
        v.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT | XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT;
        v.space = x.view; // low in front of the head, out of the way
        v.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
        v.subImage.swapchain = vol;
        v.subImage.imageRect = {{0, xo::volumeFrame(volume.value_or(0)) * static_cast<std::int32_t>(xo::kVolH)},
                                {static_cast<std::int32_t>(xo::kVolW), static_cast<std::int32_t>(xo::kVolH)}};
        v.pose = {{0.0F, 0.0F, 0.0F, 1.0F}, {0.0F, -0.22F, -0.9F}};
        v.size = {0.32F, 0.06F};
        const XrCompositionLayerBaseHeader* layers[3] = {};
        std::uint32_t layerCount = 0;
        if (fs.shouldRender && loading) {
            layers[layerCount++] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&d);
        }
        if (fs.shouldRender && volume) {
            layers[layerCount++] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&v);
        }
        if (fs.shouldRender && shown) {
            layers[layerCount++] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&q); // the ring on top
        }
        auto fei = xrkit::xr_make<XrFrameEndInfo>(XR_TYPE_FRAME_END_INFO);
        fei.displayTime = fs.predictedDisplayTime;
        fei.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
        fei.layerCount = layerCount;
        fei.layers = layers;
        xrEndFrame(x.session, &fei);
    }
    if (running) {
        xrRequestExitSession(x.session);
    }
    xrDestroySwapchain(sc);
    xrDestroySwapchain(dots);
    xrDestroySwapchain(vol);
    return 0;
}
