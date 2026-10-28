// SPDX-License-Identifier: MIT
// A tiny OpenXR runtime for the end-to-end test: one stereo HMD with eye tracking (a gaze the test scripts), Vulkan
// swapchains backed by real images on the test's device, and a read-back of every submitted projection view.
//
// It checks what a strict runtime checks and the layer could get wrong: the frame's swapchains must be released,
// the latest suggestion for an interaction profile replaces earlier ones (so a layer that overwrote the game's gaze
// binding would leave the game's action inactive), views must be located before end frame.
// clang-format off
#include <vulkan/vulkan.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <openxr/openxr_loader_negotiation.h>
// clang-format on

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#define FAKE_EXPORT extern "C" __attribute__((visibility("default")))

namespace {

constexpr uint32_t kRec = 1000, kMax = 4000;

struct Image {
    VkImage image = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
};

struct SwapchainData {
    XrSwapchainCreateInfo info{};
    std::vector<Image> images;
    uint32_t next = 0;
    std::deque<uint32_t> acquired;
    uint32_t lastReleased = 0;
    bool released = false;
};

struct Captured {
    uint32_t w = 0, h = 0;
    std::vector<uint32_t> px;
    XrFovf fov{};
    XrPosef pose{};
};

struct State {
    std::mutex m;
    // the view configuration xrBeginSession took (0: none yet) and the xrLocateViews of another type refused, as
    // Monado refuses them (oxr_xrLocateViews: "is not view configuration passed into xrBeginSession")
    XrViewConfigurationType begun{};
    uint32_t locateRejects = 0;
    // Vulkan of the session.
    VkPhysicalDevice phys = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    VkQueue queue = VK_NULL_HANDLE;
    uint32_t queueFamily = 0;
    VkCommandPool pool = VK_NULL_HANDLE;

    uint64_t nextHandle = 100;
    std::map<uint64_t, SwapchainData> swapchains;
    std::map<uint64_t, std::string> paths;   // path -> string
    std::map<uint64_t, uint64_t> actionSpaces; // space -> action
    std::set<uint64_t> refSpaces;
    std::map<uint64_t, std::vector<uint64_t>> suggested; // profile path -> actions bound
    std::set<uint64_t> attachedSets;
    std::map<uint64_t, uint64_t> actionSet; // action -> set
    bool attached = false;
    XrTime time = 1'000'000'000;
    bool gazeValid = false;
    float gazeYaw = 0, gazePitch = 0;
    std::vector<Captured> frame;
    uint32_t frames = 0;
    std::vector<uint64_t> createdUsage;
    std::string lastError;
};

State& S() {
    static State s;
    return s;
}

template <typename H>
H newHandle() {
    return reinterpret_cast<H>(S().nextHandle++);
}

uint64_t pathOf(const std::string& str) {
    State& s = S();
    for (auto& [p, v] : s.paths) {
        if (v == str) return p;
    }
    const uint64_t p = s.nextHandle++;
    s.paths[p] = str;
    return p;
}

XrFovf eyeFov(uint32_t eye) {
    const float d = float(M_PI) / 180.f;
    // Asymmetric like a real headset (more field outward), mirrored for the right eye.
    return eye == 0 ? XrFovf{-50 * d, 42 * d, 45 * d, -48 * d} : XrFovf{-42 * d, 50 * d, 45 * d, -48 * d};
}

XrPosef eyePose(uint32_t eye) {
    return {{0, 0, 0, 1}, {eye == 0 ? -0.032f : 0.032f, 0, 0}};
}

void oneShot(State& s, const std::function<void(VkCommandBuffer)>& rec);

uint32_t memType(State& s, uint32_t bits, VkMemoryPropertyFlags want) {
    VkPhysicalDeviceMemoryProperties mp;
    vkGetPhysicalDeviceMemoryProperties(s.phys, &mp);
    for (uint32_t i = 0; i < mp.memoryTypeCount; i++) {
        if ((bits & (1u << i)) && (mp.memoryTypes[i].propertyFlags & want) == want) return i;
    }
    return 0;
}

void oneShot(State& s, const std::function<void(VkCommandBuffer)>& rec) {
    VkCommandBufferAllocateInfo ai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    ai.commandPool = s.pool;
    ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    ai.commandBufferCount = 1;
    VkCommandBuffer cb;
    vkAllocateCommandBuffers(s.device, &ai, &cb);
    VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    vkBeginCommandBuffer(cb, &bi);
    rec(cb);
    vkEndCommandBuffer(cb);
    VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    si.commandBufferCount = 1;
    si.pCommandBuffers = &cb;
    vkQueueSubmit(s.queue, 1, &si, VK_NULL_HANDLE);
    vkQueueWaitIdle(s.queue);
    vkFreeCommandBuffers(s.device, s.pool, 1, &cb);
}

VkImageMemoryBarrier barrier(VkImage img, uint32_t layer, VkImageLayout from, VkImageLayout to) {
    VkImageMemoryBarrier b{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
    b.srcAccessMask = VK_ACCESS_MEMORY_WRITE_BIT;
    b.dstAccessMask = VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT;
    b.oldLayout = from;
    b.newLayout = to;
    b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b.image = img;
    b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, layer, 1};
    return b;
}

Captured readBack(State& s, const SwapchainData& sc, const XrCompositionLayerProjectionView& v) {
    Captured c;
    c.w = uint32_t(v.subImage.imageRect.extent.width);
    c.h = uint32_t(v.subImage.imageRect.extent.height);
    c.fov = v.fov;
    c.pose = v.pose;
    const VkDeviceSize bytes = VkDeviceSize(c.w) * c.h * 4;
    VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bci.size = bytes;
    bci.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    VkBuffer buf;
    vkCreateBuffer(s.device, &bci, nullptr, &buf);
    VkMemoryRequirements mr;
    vkGetBufferMemoryRequirements(s.device, buf, &mr);
    VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    mai.allocationSize = mr.size;
    mai.memoryTypeIndex =
        memType(s, mr.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    VkDeviceMemory mem;
    vkAllocateMemory(s.device, &mai, nullptr, &mem);
    vkBindBufferMemory(s.device, buf, mem, 0);
    const VkImage img = sc.images.at(sc.lastReleased).image;
    const uint32_t layer = v.subImage.imageArrayIndex;
    oneShot(s, [&](VkCommandBuffer cb) {
        auto b = barrier(img, layer, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
        vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0,
                             nullptr, 1, &b);
        VkBufferImageCopy r{};
        r.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, layer, 1};
        r.imageOffset = {v.subImage.imageRect.offset.x, v.subImage.imageRect.offset.y, 0};
        r.imageExtent = {c.w, c.h, 1};
        vkCmdCopyImageToBuffer(cb, img, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, buf, 1, &r);
        b = barrier(img, layer, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
        vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0,
                             nullptr, 1, &b);
    });
    void* p = nullptr;
    vkMapMemory(s.device, mem, 0, bytes, 0, &p);
    c.px.assign(static_cast<uint32_t*>(p), static_cast<uint32_t*>(p) + size_t(c.w) * c.h);
    vkUnmapMemory(s.device, mem);
    vkDestroyBuffer(s.device, buf, nullptr);
    vkFreeMemory(s.device, mem, nullptr);
    return c;
}

// ---- the OpenXR functions ------------------------------------------------------------------------------------------

const char* kExts[] = {XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME, XR_EXT_EYE_GAZE_INTERACTION_EXTENSION_NAME,
                       XR_KHR_VISIBILITY_MASK_EXTENSION_NAME, XR_VARJO_QUAD_VIEWS_EXTENSION_NAME};

// QV_FAKE_NATIVE_QUAD=1: a runtime with quad views of its own and no foveated rendering (as Monado): the quad view
// configuration with focus views of half the eye's field, centred. Without it, the last extension is not offered.
bool nativeQuad() {
    const char* e = std::getenv("QV_FAKE_NATIVE_QUAD");
    return e != nullptr && e[0] == '1';
}
size_t extCount() {
    return std::size(kExts) - (nativeQuad() ? 0 : 1);
}
bool quadType(XrViewConfigurationType t) {
    return nativeQuad() && t == XR_VIEW_CONFIGURATION_TYPE_PRIMARY_QUAD_VARJO;
}

XrResult XRAPI_CALL EnumerateInstanceExtensionProperties(const char* layer, uint32_t cap, uint32_t* n,
                                                         XrExtensionProperties* p) {
    if (layer) return XR_ERROR_API_LAYER_NOT_PRESENT;
    *n = uint32_t(extCount());
    if (cap == 0) return XR_SUCCESS;
    if (cap < *n) return XR_ERROR_SIZE_INSUFFICIENT;
    for (uint32_t i = 0; i < *n; i++) {
        std::strcpy(p[i].extensionName, kExts[i]);
        p[i].extensionVersion = 1;
    }
    return XR_SUCCESS;
}

XrResult XRAPI_CALL CreateInstance(const XrInstanceCreateInfo* ci, XrInstance* inst) {
    for (uint32_t i = 0; i < ci->enabledExtensionCount; i++) {
        bool known = false;
        for (size_t k = 0; k < extCount(); k++) known = known || std::strcmp(kExts[k], ci->enabledExtensionNames[i]) == 0;
        if (!known) {
            S().lastError = std::string("unknown extension ") + ci->enabledExtensionNames[i];
            return XR_ERROR_EXTENSION_NOT_PRESENT;
        }
    }
    *inst = newHandle<XrInstance>();
    return XR_SUCCESS;
}
XrResult XRAPI_CALL DestroyInstance(XrInstance) {
    State& s = S();
    s.suggested.clear();
    s.attached = false;
    return XR_SUCCESS;
}
XrResult XRAPI_CALL GetInstanceProperties(XrInstance, XrInstanceProperties* p) {
    std::strcpy(p->runtimeName, "qv fake runtime");
    p->runtimeVersion = XR_MAKE_VERSION(0, 1, 0);
    return XR_SUCCESS;
}
XrResult XRAPI_CALL GetSystem(XrInstance, const XrSystemGetInfo*, XrSystemId* id) {
    *id = 1;
    return XR_SUCCESS;
}
XrResult XRAPI_CALL GetSystemProperties(XrInstance, XrSystemId, XrSystemProperties* p) {
    std::strcpy(p->systemName, "qv fake HMD");
    p->graphicsProperties.maxSwapchainImageWidth = kMax;
    p->graphicsProperties.maxSwapchainImageHeight = kMax;
    p->graphicsProperties.maxLayerCount = 16;
    for (auto* b = reinterpret_cast<XrBaseOutStructure*>(p->next); b; b = b->next) {
        if (b->type == XR_TYPE_SYSTEM_EYE_GAZE_INTERACTION_PROPERTIES_EXT) {
            reinterpret_cast<XrSystemEyeGazeInteractionPropertiesEXT*>(b)->supportsEyeGazeInteraction = XR_TRUE;
        }
    }
    return XR_SUCCESS;
}
XrResult XRAPI_CALL EnumerateViewConfigurations(XrInstance, XrSystemId, uint32_t cap, uint32_t* n,
                                                XrViewConfigurationType* t) {
    *n = nativeQuad() ? 2 : 1;
    if (cap == 0) return XR_SUCCESS;
    t[0] = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    if (nativeQuad() && cap > 1) t[1] = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_QUAD_VARJO;
    return XR_SUCCESS;
}
XrResult XRAPI_CALL GetViewConfigurationProperties(XrInstance, XrSystemId, XrViewConfigurationType t,
                                                   XrViewConfigurationProperties* p) {
    if (t != XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO && !quadType(t)) return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
    p->viewConfigurationType = t;
    p->fovMutable = XR_TRUE;
    return XR_SUCCESS;
}
XrResult XRAPI_CALL EnumerateViewConfigurationViews(XrInstance, XrSystemId, XrViewConfigurationType t, uint32_t cap,
                                                    uint32_t* n, XrViewConfigurationView* v) {
    if (t != XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO && !quadType(t)) return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
    const uint32_t views = quadType(t) ? 4 : 2;
    *n = views;
    if (cap == 0) return XR_SUCCESS;
    if (cap < views) return XR_ERROR_SIZE_INSUFFICIENT;
    for (uint32_t i = 0; i < views; i++) {
        v[i].recommendedImageRectWidth = v[i].recommendedImageRectHeight = kRec;
        v[i].maxImageRectWidth = v[i].maxImageRectHeight = kMax;
        v[i].recommendedSwapchainSampleCount = v[i].maxSwapchainSampleCount = 1;
    }
    return XR_SUCCESS;
}
XrResult XRAPI_CALL EnumerateEnvironmentBlendModes(XrInstance, XrSystemId, XrViewConfigurationType t, uint32_t cap,
                                                   uint32_t* n, XrEnvironmentBlendMode* m) {
    if (t != XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO && !quadType(t)) return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
    *n = 1;
    if (cap) m[0] = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    return XR_SUCCESS;
}
XrResult XRAPI_CALL GetVulkanGraphicsRequirements2KHR(XrInstance, XrSystemId, XrGraphicsRequirementsVulkanKHR* r) {
    r->minApiVersionSupported = XR_MAKE_VERSION(1, 0, 0);
    r->maxApiVersionSupported = XR_MAKE_VERSION(1, 4, 0);
    return XR_SUCCESS;
}
XrResult XRAPI_CALL CreateSession(XrInstance, const XrSessionCreateInfo* ci, XrSession* s) {
    State& st = S();
    for (auto* b = reinterpret_cast<const XrBaseInStructure*>(ci->next); b; b = b->next) {
        if (b->type == XR_TYPE_GRAPHICS_BINDING_VULKAN_KHR) {
            const auto* vk = reinterpret_cast<const XrGraphicsBindingVulkanKHR*>(b);
            st.phys = vk->physicalDevice;
            st.device = vk->device;
            st.queueFamily = vk->queueFamilyIndex;
            vkGetDeviceQueue(st.device, vk->queueFamilyIndex, vk->queueIndex, &st.queue);
            VkCommandPoolCreateInfo pci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
            pci.queueFamilyIndex = st.queueFamily;
            vkCreateCommandPool(st.device, &pci, nullptr, &st.pool);
        }
    }
    if (!st.device) return XR_ERROR_GRAPHICS_DEVICE_INVALID;
    *s = newHandle<XrSession>();
    return XR_SUCCESS;
}
XrResult XRAPI_CALL DestroySession(XrSession) {
    S().begun = {};
    State& s = S();
    vkDeviceWaitIdle(s.device);
    for (auto& [h, sc] : s.swapchains) {
        for (auto& i : sc.images) {
            vkDestroyImage(s.device, i.image, nullptr);
            vkFreeMemory(s.device, i.memory, nullptr);
        }
    }
    s.swapchains.clear();
    vkDestroyCommandPool(s.device, s.pool, nullptr);
    s.pool = VK_NULL_HANDLE;
    s.device = VK_NULL_HANDLE;
    s.attached = false;
    s.attachedSets.clear();
    return XR_SUCCESS;
}
XrResult XRAPI_CALL BeginSession(XrSession, const XrSessionBeginInfo* bi) {
    if (bi->primaryViewConfigurationType == XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO ||
        quadType(bi->primaryViewConfigurationType)) {
        S().begun = bi->primaryViewConfigurationType;
        return XR_SUCCESS;
    }
    return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
}
XrResult XRAPI_CALL EndSession(XrSession) {
    return XR_SUCCESS;
}
XrResult XRAPI_CALL WaitFrame(XrSession, const XrFrameWaitInfo*, XrFrameState* fs) {
    State& s = S();
    s.time += 11'111'111;
    fs->predictedDisplayTime = s.time;
    fs->predictedDisplayPeriod = 11'111'111;
    fs->shouldRender = XR_TRUE;
    return XR_SUCCESS;
}
XrResult XRAPI_CALL BeginFrame(XrSession, const XrFrameBeginInfo*) {
    return XR_SUCCESS;
}
XrResult XRAPI_CALL EndFrame(XrSession, const XrFrameEndInfo* fi) {
    State& s = S();
    std::lock_guard lock(s.m);
    s.frame.clear();
    for (uint32_t i = 0; i < fi->layerCount; i++) {
        if (fi->layers[i]->type != XR_TYPE_COMPOSITION_LAYER_PROJECTION) continue;
        const auto* p = reinterpret_cast<const XrCompositionLayerProjection*>(fi->layers[i]);
        if (p->viewCount != 2) {
            s.lastError = "projection layer with " + std::to_string(p->viewCount) + " views";
            return XR_ERROR_VALIDATION_FAILURE;
        }
        for (uint32_t v = 0; v < 2; v++) {
            auto it = s.swapchains.find(uint64_t(p->views[v].subImage.swapchain));
            if (it == s.swapchains.end()) {
                s.lastError = "unknown swapchain in a layer";
                return XR_ERROR_HANDLE_INVALID;
            }
            if (!it->second.acquired.empty() || !it->second.released) {
                s.lastError = "a swapchain of the frame is not released";
                return XR_ERROR_LAYER_INVALID;
            }
            s.frame.push_back(readBack(s, it->second, p->views[v]));
        }
    }
    s.frames++;
    return XR_SUCCESS;
}
XrResult XRAPI_CALL LocateViews(XrSession, const XrViewLocateInfo* li, XrViewState* vs, uint32_t cap, uint32_t* n,
                                XrView* v) {
    if (li->viewConfigurationType != XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO && !quadType(li->viewConfigurationType)) {
        return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
    }
    if (S().begun != XrViewConfigurationType{} && li->viewConfigurationType != S().begun) {
        S().locateRejects++;
        return XR_ERROR_VALIDATION_FAILURE;
    }
    const uint32_t views = quadType(li->viewConfigurationType) ? 4 : 2;
    *n = views;
    if (cap == 0) return XR_SUCCESS;
    if (cap < views) return XR_ERROR_SIZE_INSUFFICIENT;
    for (uint32_t i = 2; i < views; i++) {
        // the runtime's own focus views: half the eye's field, centred (Monado's emulation)
        const XrFovf f = eyeFov(i - 2);
        v[i].pose = eyePose(i - 2);
        v[i].fov = {f.angleLeft / 2, f.angleRight / 2, f.angleUp / 2, f.angleDown / 2};
    }
    vs->viewStateFlags = XR_VIEW_STATE_ORIENTATION_VALID_BIT | XR_VIEW_STATE_POSITION_VALID_BIT |
                         XR_VIEW_STATE_ORIENTATION_TRACKED_BIT | XR_VIEW_STATE_POSITION_TRACKED_BIT;
    for (uint32_t i = 0; i < 2; i++) {
        v[i].pose = eyePose(i); // the head sits at the origin of every space
        v[i].fov = eyeFov(i);
    }
    return XR_SUCCESS;
}
XrResult XRAPI_CALL CreateReferenceSpace(XrSession, const XrReferenceSpaceCreateInfo*, XrSpace* sp) {
    *sp = newHandle<XrSpace>();
    S().refSpaces.insert(uint64_t(*sp));
    return XR_SUCCESS;
}
XrResult XRAPI_CALL DestroySpace(XrSpace sp) {
    S().refSpaces.erase(uint64_t(sp));
    S().actionSpaces.erase(uint64_t(sp));
    return XR_SUCCESS;
}
bool actionActive(State& s, uint64_t action) {
    const uint64_t gp = pathOf("/interaction_profiles/ext/eye_gaze_interaction");
    auto it = s.suggested.find(gp);
    if (!s.attached || it == s.suggested.end()) return false;
    bool bound = false;
    for (uint64_t a : it->second) bound = bound || a == action;
    return bound && s.attachedSets.count(s.actionSet[action]) && s.gazeValid;
}
XrResult XRAPI_CALL LocateSpace(XrSpace sp, XrSpace, XrTime, XrSpaceLocation* loc) {
    State& s = S();
    auto it = s.actionSpaces.find(uint64_t(sp));
    loc->pose = {{0, 0, 0, 1}, {0, 0, 0}};
    loc->locationFlags = XR_SPACE_LOCATION_ORIENTATION_VALID_BIT | XR_SPACE_LOCATION_POSITION_VALID_BIT;
    if (it != s.actionSpaces.end()) {
        if (!actionActive(s, it->second)) {
            loc->locationFlags = 0;
            return XR_SUCCESS;
        }
        // yaw right = rotation about -y; pitch up = rotation about +x
        const float y = -s.gazeYaw * float(M_PI) / 360.f, p = s.gazePitch * float(M_PI) / 360.f;
        const XrQuaternionf qy{0, std::sin(y), 0, std::cos(y)}, qp{std::sin(p), 0, 0, std::cos(p)};
        loc->pose.orientation = {qy.w * qp.x + qy.x * qp.w + qy.y * qp.z - qy.z * qp.y,
                                 qy.w * qp.y - qy.x * qp.z + qy.y * qp.w + qy.z * qp.x,
                                 qy.w * qp.z + qy.x * qp.y - qy.y * qp.x + qy.z * qp.w,
                                 qy.w * qp.w - qy.x * qp.x - qy.y * qp.y - qy.z * qp.z};
        loc->locationFlags |= XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
    }
    return XR_SUCCESS;
}
XrResult XRAPI_CALL EnumerateSwapchainFormats(XrSession, uint32_t cap, uint32_t* n, int64_t* f) {
    *n = 2;
    if (cap == 0) return XR_SUCCESS;
    f[0] = VK_FORMAT_R8G8B8A8_UNORM;
    f[1] = VK_FORMAT_R8G8B8A8_SRGB;
    return XR_SUCCESS;
}
XrResult XRAPI_CALL CreateSwapchain(XrSession, const XrSwapchainCreateInfo* ci, XrSwapchain* sc) {
    State& s = S();
    SwapchainData d;
    d.info = *ci;
    s.createdUsage.push_back(ci->usageFlags);
    VkImageUsageFlags usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    if (ci->usageFlags & XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT) usage |= VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    if (ci->usageFlags & XR_SWAPCHAIN_USAGE_SAMPLED_BIT) usage |= VK_IMAGE_USAGE_SAMPLED_BIT;
    for (int i = 0; i < 3; i++) {
        VkImageCreateInfo ici{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
        ici.imageType = VK_IMAGE_TYPE_2D;
        ici.format = VkFormat(ci->format);
        ici.extent = {ci->width, ci->height, 1};
        ici.mipLevels = 1;
        ici.arrayLayers = ci->arraySize;
        ici.samples = VK_SAMPLE_COUNT_1_BIT;
        ici.tiling = VK_IMAGE_TILING_OPTIMAL;
        ici.usage = usage;
        Image im;
        if (vkCreateImage(s.device, &ici, nullptr, &im.image) != VK_SUCCESS) return XR_ERROR_RUNTIME_FAILURE;
        VkMemoryRequirements mr;
        vkGetImageMemoryRequirements(s.device, im.image, &mr);
        VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        mai.allocationSize = mr.size;
        mai.memoryTypeIndex = memType(s, mr.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        vkAllocateMemory(s.device, &mai, nullptr, &im.memory);
        vkBindImageMemory(s.device, im.image, im.memory, 0);
        oneShot(s, [&](VkCommandBuffer cb) {
            for (uint32_t l = 0; l < ci->arraySize; l++) {
                auto b = barrier(im.image, l, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
                vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0,
                                     nullptr, 0, nullptr, 1, &b);
            }
        });
        d.images.push_back(im);
    }
    *sc = newHandle<XrSwapchain>();
    s.swapchains[uint64_t(*sc)] = std::move(d);
    return XR_SUCCESS;
}
XrResult XRAPI_CALL DestroySwapchain(XrSwapchain sc) {
    State& s = S();
    auto it = s.swapchains.find(uint64_t(sc));
    if (it == s.swapchains.end()) return XR_ERROR_HANDLE_INVALID;
    vkDeviceWaitIdle(s.device);
    for (auto& i : it->second.images) {
        vkDestroyImage(s.device, i.image, nullptr);
        vkFreeMemory(s.device, i.memory, nullptr);
    }
    s.swapchains.erase(it);
    return XR_SUCCESS;
}
XrResult XRAPI_CALL EnumerateSwapchainImages(XrSwapchain sc, uint32_t cap, uint32_t* n, XrSwapchainImageBaseHeader* p) {
    auto& d = S().swapchains.at(uint64_t(sc));
    *n = uint32_t(d.images.size());
    if (cap == 0) return XR_SUCCESS;
    if (cap < *n) return XR_ERROR_SIZE_INSUFFICIENT;
    auto* v = reinterpret_cast<XrSwapchainImageVulkanKHR*>(p);
    for (uint32_t i = 0; i < *n; i++) v[i].image = d.images[i].image;
    return XR_SUCCESS;
}
XrResult XRAPI_CALL AcquireSwapchainImage(XrSwapchain sc, const XrSwapchainImageAcquireInfo*, uint32_t* i) {
    auto& d = S().swapchains.at(uint64_t(sc));
    if (d.acquired.size() >= d.images.size()) return XR_ERROR_CALL_ORDER_INVALID;
    *i = d.next;
    d.acquired.push_back(d.next);
    d.next = (d.next + 1) % d.images.size();
    return XR_SUCCESS;
}
XrResult XRAPI_CALL WaitSwapchainImage(XrSwapchain, const XrSwapchainImageWaitInfo*) {
    return XR_SUCCESS;
}
XrResult XRAPI_CALL ReleaseSwapchainImage(XrSwapchain sc, const XrSwapchainImageReleaseInfo*) {
    auto& d = S().swapchains.at(uint64_t(sc));
    if (d.acquired.empty()) return XR_ERROR_CALL_ORDER_INVALID;
    d.lastReleased = d.acquired.front();
    d.acquired.pop_front();
    d.released = true;
    return XR_SUCCESS;
}
XrResult XRAPI_CALL StringToPath(XrInstance, const char* str, XrPath* p) {
    *p = pathOf(str);
    return XR_SUCCESS;
}
XrResult XRAPI_CALL PathToString(XrInstance, XrPath p, uint32_t cap, uint32_t* n, char* buf) {
    const std::string& s = S().paths[p];
    *n = uint32_t(s.size() + 1);
    if (cap == 0) return XR_SUCCESS;
    if (cap < *n) return XR_ERROR_SIZE_INSUFFICIENT;
    std::memcpy(buf, s.c_str(), *n);
    return XR_SUCCESS;
}
XrResult XRAPI_CALL CreateActionSet(XrInstance, const XrActionSetCreateInfo*, XrActionSet* a) {
    *a = newHandle<XrActionSet>();
    return XR_SUCCESS;
}
XrResult XRAPI_CALL DestroyActionSet(XrActionSet) {
    return XR_SUCCESS;
}
XrResult XRAPI_CALL CreateAction(XrActionSet set, const XrActionCreateInfo*, XrAction* a) {
    if (S().attachedSets.count(uint64_t(set))) return XR_ERROR_ACTIONSETS_ALREADY_ATTACHED;
    *a = newHandle<XrAction>();
    S().actionSet[uint64_t(*a)] = uint64_t(set);
    return XR_SUCCESS;
}
XrResult XRAPI_CALL CreateActionSpace(XrSession, const XrActionSpaceCreateInfo* ci, XrSpace* sp) {
    *sp = newHandle<XrSpace>();
    S().actionSpaces[uint64_t(*sp)] = uint64_t(ci->action);
    return XR_SUCCESS;
}
XrResult XRAPI_CALL SuggestInteractionProfileBindings(XrInstance, const XrInteractionProfileSuggestedBinding* s) {
    State& st = S();
    if (st.attached) return XR_ERROR_ACTIONSETS_ALREADY_ATTACHED;
    std::vector<uint64_t> acts;
    for (uint32_t i = 0; i < s->countSuggestedBindings; i++) acts.push_back(uint64_t(s->suggestedBindings[i].action));
    st.suggested[s->interactionProfile] = acts; // replaces the earlier suggestion for this profile
    return XR_SUCCESS;
}
XrResult XRAPI_CALL AttachSessionActionSets(XrSession, const XrSessionActionSetsAttachInfo* a) {
    State& s = S();
    if (s.attached) return XR_ERROR_ACTIONSETS_ALREADY_ATTACHED;
    for (uint32_t i = 0; i < a->countActionSets; i++) s.attachedSets.insert(uint64_t(a->actionSets[i]));
    s.attached = true;
    return XR_SUCCESS;
}
XrResult XRAPI_CALL SyncActions(XrSession, const XrActionsSyncInfo*) {
    return S().attached ? XR_SUCCESS : XR_ERROR_ACTIONSET_NOT_ATTACHED;
}
XrResult XRAPI_CALL GetActionStatePose(XrSession, const XrActionStateGetInfo* gi, XrActionStatePose* st) {
    st->isActive = actionActive(S(), uint64_t(gi->action)) ? XR_TRUE : XR_FALSE;
    return XR_SUCCESS;
}
XrResult XRAPI_CALL PollEvent(XrInstance, XrEventDataBuffer*) {
    return XR_EVENT_UNAVAILABLE;
}
XrResult XRAPI_CALL GetVisibilityMaskKHR(XrSession, XrViewConfigurationType t, uint32_t, XrVisibilityMaskTypeKHR,
                                         XrVisibilityMaskKHR* m) {
    if (t != XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO) return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
    m->vertexCountOutput = 3;
    m->indexCountOutput = 3;
    return XR_SUCCESS;
}

XrResult XRAPI_CALL GetInstanceProcAddr(XrInstance, const char* name, PFN_xrVoidFunction* fn) {
#define F(n)                                                                                                           \
    if (std::strcmp(name, "xr" #n) == 0) {                                                                             \
        *fn = reinterpret_cast<PFN_xrVoidFunction>(n);                                                                 \
        return XR_SUCCESS;                                                                                             \
    }
    F(GetInstanceProcAddr) F(EnumerateInstanceExtensionProperties) F(CreateInstance) F(DestroyInstance)
    F(GetInstanceProperties) F(GetSystem) F(GetSystemProperties) F(EnumerateViewConfigurations)
    F(GetViewConfigurationProperties) F(EnumerateViewConfigurationViews) F(EnumerateEnvironmentBlendModes)
    F(GetVulkanGraphicsRequirements2KHR) F(CreateSession) F(DestroySession) F(BeginSession) F(EndSession)
    F(WaitFrame) F(BeginFrame) F(EndFrame) F(LocateViews) F(CreateReferenceSpace) F(DestroySpace) F(LocateSpace)
    F(EnumerateSwapchainFormats) F(CreateSwapchain) F(DestroySwapchain) F(EnumerateSwapchainImages)
    F(AcquireSwapchainImage) F(WaitSwapchainImage) F(ReleaseSwapchainImage) F(StringToPath) F(PathToString)
    F(CreateActionSet) F(DestroyActionSet) F(CreateAction) F(CreateActionSpace) F(SuggestInteractionProfileBindings)
    F(AttachSessionActionSets) F(SyncActions) F(GetActionStatePose) F(PollEvent) F(GetVisibilityMaskKHR)
#undef F
    *fn = nullptr;
    return XR_ERROR_FUNCTION_UNSUPPORTED;
}

} // namespace

FAKE_EXPORT XrResult xrNegotiateLoaderRuntimeInterface(const XrNegotiateLoaderInfo* li, XrNegotiateRuntimeRequest* r) {
    if (!li || !r || li->minInterfaceVersion > XR_CURRENT_LOADER_RUNTIME_VERSION) return XR_ERROR_INITIALIZATION_FAILED;
    r->runtimeInterfaceVersion = XR_CURRENT_LOADER_RUNTIME_VERSION;
    r->runtimeApiVersion = XR_MAKE_VERSION(1, 0, 0);
    r->getInstanceProcAddr = GetInstanceProcAddr;
    return XR_SUCCESS;
}

// The test's side channel.
FAKE_EXPORT uint32_t fake_locate_rejects() {
    return S().locateRejects;
}
FAKE_EXPORT void fake_set_gaze(int valid, float yawDeg, float pitchDeg) {
    S().gazeValid = valid != 0;
    S().gazeYaw = yawDeg;
    S().gazePitch = pitchDeg;
}
FAKE_EXPORT uint32_t fake_frame_views() {
    return uint32_t(S().frame.size());
}
FAKE_EXPORT const uint32_t* fake_frame_view(uint32_t i, uint32_t* w, uint32_t* h, XrFovf* fov) {
    const Captured& c = S().frame.at(i);
    *w = c.w;
    *h = c.h;
    *fov = c.fov;
    return c.px.data();
}
FAKE_EXPORT uint32_t fake_frames() {
    return S().frames;
}
FAKE_EXPORT const char* fake_last_error() {
    return S().lastError.c_str();
}
FAKE_EXPORT uint32_t fake_created_usage(uint32_t i) {
    return i < S().createdUsage.size() ? uint32_t(S().createdUsage[i]) : 0;
}
FAKE_EXPORT uint32_t fake_created_count() {
    return uint32_t(S().createdUsage.size());
}
FAKE_EXPORT void fake_reset_created() {
    S().createdUsage.clear();
}
