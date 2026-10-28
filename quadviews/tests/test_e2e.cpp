// SPDX-License-Identifier: MIT
// End to end: a game that uses quad views (like Falcon BMS: Vulkan, XR_VARJO_quad_views + foveated rendering, its
// own eye gaze action), the real OpenXR loader, the layer from the build tree, and the fake runtime. The runtime reads
// back what reaches it; the test checks the composed pixels against the geometry.
//
//   qv_test_e2e <fake runtime .so> <layer manifest .json>
//
// Runs on any Vulkan device; ctest points VK_DRIVER_FILES at lavapipe (CPU), so it never touches the GPU.
// clang-format off
#include <vulkan/vulkan.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
// clang-format on

#include <dlfcn.h>
#include <unistd.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "check.h"

namespace fs = std::filesystem;

namespace {

std::string g_runtime, g_manifest;

#define XR_OK(call)                                                                                                    \
    do {                                                                                                               \
        const XrResult r_ = (call);                                                                                    \
        if (XR_FAILED(r_)) {                                                                                           \
            std::printf("  FAIL %s:%d: %s = %d (%s)\n", __FILE__, __LINE__, #call, r_, fakeError());                   \
            qvtest::failures()++;                                                                                      \
            return;                                                                                                    \
        }                                                                                                              \
    } while (0)

void* fakeLib() {
    static void* h = dlopen(g_runtime.c_str(), RTLD_NOW | RTLD_NOLOAD);
    return h;
}
template <typename F>
F fakeFn(const char* name) {
    return reinterpret_cast<F>(dlsym(fakeLib(), name));
}
const char* fakeError() {
    auto f = fakeLib() ? fakeFn<const char* (*)()>("fake_last_error") : nullptr;
    return f ? f() : "";
}

void setupEnvironment() {
    const fs::path tmp = fs::temp_directory_path() / ("qv-vulkan-e2e-" + std::to_string(getpid()));
    fs::remove_all(tmp);
    const fs::path layers = tmp / "data/openxr/1/api_layers/implicit.d";
    fs::create_directories(layers);
    fs::copy_file(g_manifest, layers / fs::path(g_manifest).filename());
    fs::create_directories(tmp / "empty");
    fs::create_directories(tmp / "config/qv-vulkan");
    {
        std::ofstream o(tmp / "runtime.json");
        o << "{\"file_format_version\":\"1.0.0\",\"runtime\":{\"name\":\"qv fake\",\"library_path\":\"" << g_runtime
          << "\"}}\n";
    }
    {
        std::ofstream o(tmp / "config/qv-vulkan/settings.conf");
        o << "peripheral_density=0.4\nfocus_density=1.0\nfixed_width=0.5\nfixed_height=0.45\ngaze_width=0.35\n"
             "gaze_height=0.35\ntransition=0.2\nedge_opacity=0\nsharpen=0.6\nperipheral_filter=bicubic\ngaze_filter=on\n";
    }
    setenv("XR_RUNTIME_JSON", (tmp / "runtime.json").c_str(), 1);
    setenv("XDG_DATA_HOME", (tmp / "data").c_str(), 1);
    setenv("XDG_DATA_DIRS", (tmp / "empty").c_str(), 1);
    setenv("XDG_CONFIG_DIRS", (tmp / "empty").c_str(), 1);
    setenv("XDG_CONFIG_HOME", (tmp / "config").c_str(), 1);
    setenv("XDG_STATE_HOME", (tmp / "state").c_str(), 1);
    std::printf("environment in %s\n", tmp.c_str());
}

struct Vk {
    VkInstance instance = VK_NULL_HANDLE;
    VkPhysicalDevice phys = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    uint32_t family = 0;
    VkQueue queue = VK_NULL_HANDLE;
    VkCommandPool pool = VK_NULL_HANDLE;

    bool init() {
        VkApplicationInfo ai{VK_STRUCTURE_TYPE_APPLICATION_INFO};
        ai.apiVersion = VK_API_VERSION_1_1;
        VkInstanceCreateInfo ci{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
        ci.pApplicationInfo = &ai;
        if (vkCreateInstance(&ci, nullptr, &instance) != VK_SUCCESS) return false;
        uint32_t n = 0;
        vkEnumeratePhysicalDevices(instance, &n, nullptr);
        if (!n) return false;
        std::vector<VkPhysicalDevice> pds(n);
        vkEnumeratePhysicalDevices(instance, &n, pds.data());
        phys = pds[0];
        VkPhysicalDeviceProperties p;
        vkGetPhysicalDeviceProperties(phys, &p);
        std::printf("Vulkan device: %s\n", p.deviceName);
        vkGetPhysicalDeviceQueueFamilyProperties(phys, &n, nullptr);
        std::vector<VkQueueFamilyProperties> qf(n);
        vkGetPhysicalDeviceQueueFamilyProperties(phys, &n, qf.data());
        for (uint32_t i = 0; i < n; i++) {
            if (qf[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
                family = i;
                break;
            }
        }
        const float prio = 1.f;
        VkDeviceQueueCreateInfo qci{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        qci.queueFamilyIndex = family;
        qci.queueCount = 1;
        qci.pQueuePriorities = &prio;
        VkDeviceCreateInfo dci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
        dci.queueCreateInfoCount = 1;
        dci.pQueueCreateInfos = &qci;
        if (vkCreateDevice(phys, &dci, nullptr, &device) != VK_SUCCESS) return false;
        vkGetDeviceQueue(device, family, 0, &queue);
        VkCommandPoolCreateInfo pci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        pci.queueFamilyIndex = family;
        vkCreateCommandPool(device, &pci, nullptr, &pool);
        return true;
    }

    // Fill one swapchain image with a colour (as a game's render pass would) and leave it in COLOR_ATTACHMENT.
    void fill(VkImage img, uint32_t w, uint32_t h, uint32_t rgba) {
        const VkDeviceSize bytes = VkDeviceSize(w) * h * 4;
        VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bci.size = bytes;
        bci.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
        VkBuffer buf;
        vkCreateBuffer(device, &bci, nullptr, &buf);
        VkMemoryRequirements mr;
        vkGetBufferMemoryRequirements(device, buf, &mr);
        VkPhysicalDeviceMemoryProperties mp;
        vkGetPhysicalDeviceMemoryProperties(phys, &mp);
        uint32_t type = 0;
        for (uint32_t i = 0; i < mp.memoryTypeCount; i++) {
            const auto want = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
            if ((mr.memoryTypeBits & (1u << i)) && (mp.memoryTypes[i].propertyFlags & want) == want) {
                type = i;
                break;
            }
        }
        VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        mai.allocationSize = mr.size;
        mai.memoryTypeIndex = type;
        VkDeviceMemory mem;
        vkAllocateMemory(device, &mai, nullptr, &mem);
        vkBindBufferMemory(device, buf, mem, 0);
        void* p;
        vkMapMemory(device, mem, 0, bytes, 0, &p);
        std::fill_n(static_cast<uint32_t*>(p), size_t(w) * h, rgba);
        vkUnmapMemory(device, mem);

        VkCommandBufferAllocateInfo cai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        cai.commandPool = pool;
        cai.commandBufferCount = 1;
        VkCommandBuffer cb;
        vkAllocateCommandBuffers(device, &cai, &cb);
        VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        vkBeginCommandBuffer(cb, &bi);
        VkImageMemoryBarrier b{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        b.srcAccessMask = VK_ACCESS_MEMORY_READ_BIT;
        b.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        b.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        b.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.image = img;
        b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0,
                             nullptr, 1, &b);
        VkBufferImageCopy r{};
        r.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        r.imageExtent = {w, h, 1};
        vkCmdCopyBufferToImage(cb, buf, img, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &r);
        b.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        b.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        b.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        b.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0,
                             nullptr, 1, &b);
        vkEndCommandBuffer(cb);
        VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        si.commandBufferCount = 1;
        si.pCommandBuffers = &cb;
        vkQueueSubmit(queue, 1, &si, VK_NULL_HANDLE);
        vkQueueWaitIdle(queue);
        vkFreeCommandBuffers(device, pool, 1, &cb);
        vkDestroyBuffer(device, buf, nullptr);
        vkFreeMemory(device, mem, nullptr);
    }

    void destroy() {
        if (device) {
            vkDestroyCommandPool(device, pool, nullptr);
            vkDestroyDevice(device, nullptr);
        }
        if (instance) vkDestroyInstance(instance, nullptr);
    }
};

constexpr uint32_t kRed = 0xff0000c8;    // R 200 (RGBA8 in memory: R G B A)
constexpr uint32_t kGreen = 0xff00c800;  // G 200
constexpr uint32_t kBlue = 0xffc80000;   // B 200
constexpr uint32_t kYellow = 0xff00c8c8; // R 200 G 200

int ch(uint32_t px, int c) {
    return int((px >> (8 * c)) & 0xff);
}

// The focus window of `focus` inside `outer` in image pixels: x0, y0, x1, y1.
std::array<double, 4> focusRectPx(const XrFovf& outer, const XrFovf& focus, uint32_t w, uint32_t h) {
    const double l = std::tan(outer.angleLeft), r = std::tan(outer.angleRight);
    const double u = std::tan(outer.angleUp), d = std::tan(outer.angleDown);
    return {(std::tan(focus.angleLeft) - l) / (r - l) * w, (u - std::tan(focus.angleUp)) / (u - d) * h,
            (std::tan(focus.angleRight) - l) / (r - l) * w, (u - std::tan(focus.angleDown)) / (u - d) * h};
}

double centreTan(float a, float b) {
    return 0.5 * (std::tan(a) + std::tan(b));
}

struct App {
    Vk vk;
    XrInstance instance = XR_NULL_HANDLE;
    XrSystemId system = XR_NULL_SYSTEM_ID;
    XrSession session = XR_NULL_HANDLE;
    XrSpace local = XR_NULL_HANDLE;
    XrActionSet set = XR_NULL_HANDLE;
    XrAction gaze = XR_NULL_HANDLE;
    XrSpace gazeSpace = XR_NULL_HANDLE;
    std::array<XrSwapchain, 4> sc{};
    std::array<std::vector<VkImage>, 4> images;
    std::array<XrViewConfigurationView, 4> cfg{};
};

void runQuad(VkFormat format) {
    App a;
    std::printf("  format %d\n", format);
    CHECK(a.vk.init());
    const char* exts[] = {XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME, XR_VARJO_QUAD_VIEWS_EXTENSION_NAME,
                          XR_VARJO_FOVEATED_RENDERING_EXTENSION_NAME, XR_EXT_EYE_GAZE_INTERACTION_EXTENSION_NAME};
    XrInstanceCreateInfo ici{XR_TYPE_INSTANCE_CREATE_INFO};
    std::strcpy(ici.applicationInfo.applicationName, "qv e2e");
    ici.applicationInfo.apiVersion = XR_MAKE_VERSION(1, 0, 0);
    ici.enabledExtensionCount = 4;
    ici.enabledExtensionNames = exts;
    XR_OK(xrCreateInstance(&ici, &a.instance));
    CHECK(fakeLib() != nullptr);
    auto setGaze = fakeFn<void (*)(int, float, float)>("fake_set_gaze");
    setGaze(0, 0, 0);

    XrSystemGetInfo sgi{XR_TYPE_SYSTEM_GET_INFO};
    sgi.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    XR_OK(xrGetSystem(a.instance, &sgi, &a.system));
    XrSystemFoveatedRenderingPropertiesVARJO fov{XR_TYPE_SYSTEM_FOVEATED_RENDERING_PROPERTIES_VARJO};
    XrSystemEyeGazeInteractionPropertiesEXT eg{XR_TYPE_SYSTEM_EYE_GAZE_INTERACTION_PROPERTIES_EXT, &fov};
    XrSystemProperties sp{XR_TYPE_SYSTEM_PROPERTIES, &eg};
    XR_OK(xrGetSystemProperties(a.instance, a.system, &sp));
    CHECK(fov.supportsFoveatedRendering == XR_TRUE);
    CHECK(eg.supportsEyeGazeInteraction == XR_TRUE);

    uint32_t n = 0;
    XrViewConfigurationType types[4];
    XR_OK(xrEnumerateViewConfigurations(a.instance, a.system, 4, &n, types));
    CHECK(n == 2 && types[0] == XR_VIEW_CONFIGURATION_TYPE_PRIMARY_QUAD_VARJO &&
          types[1] == XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO);

    std::array<XrFoveatedViewConfigurationViewVARJO, 4> fcv{};
    for (uint32_t i = 0; i < 4; i++) {
        fcv[i] = {XR_TYPE_FOVEATED_VIEW_CONFIGURATION_VIEW_VARJO, nullptr, XR_TRUE};
        a.cfg[i] = {XR_TYPE_VIEW_CONFIGURATION_VIEW, &fcv[i]};
    }
    XR_OK(xrEnumerateViewConfigurationViews(a.instance, a.system, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_QUAD_VARJO, 4, &n,
                                            a.cfg.data()));
    CHECK(n == 4);
    CHECK(a.cfg[0].recommendedImageRectWidth == 400 && a.cfg[1].recommendedImageRectHeight == 400);
    CHECK(a.cfg[2].recommendedImageRectWidth == 350 && a.cfg[3].recommendedImageRectHeight == 350);
    CHECK(a.cfg[2].next == &fcv[2]); // the game's chain is kept

    PFN_xrGetVulkanGraphicsRequirements2KHR req = nullptr;
    XR_OK(xrGetInstanceProcAddr(a.instance, "xrGetVulkanGraphicsRequirements2KHR",
                                reinterpret_cast<PFN_xrVoidFunction*>(&req)));
    XrGraphicsRequirementsVulkanKHR gr{XR_TYPE_GRAPHICS_REQUIREMENTS_VULKAN_KHR};
    XR_OK(req(a.instance, a.system, &gr));

    // The game's own gaze action, as Falcon BMS does it.
    XrActionSetCreateInfo asi{XR_TYPE_ACTION_SET_CREATE_INFO};
    std::strcpy(asi.actionSetName, "game");
    std::strcpy(asi.localizedActionSetName, "Game");
    XR_OK(xrCreateActionSet(a.instance, &asi, &a.set));
    XrActionCreateInfo aci{XR_TYPE_ACTION_CREATE_INFO};
    std::strcpy(aci.actionName, "eye_gaze");
    std::strcpy(aci.localizedActionName, "Eye Gaze");
    aci.actionType = XR_ACTION_TYPE_POSE_INPUT;
    XR_OK(xrCreateAction(a.set, &aci, &a.gaze));
    XrActionSuggestedBinding b{a.gaze};
    XR_OK(xrStringToPath(a.instance, "/user/eyes_ext/input/gaze_ext/pose", &b.binding));
    XrInteractionProfileSuggestedBinding sb{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
    XR_OK(xrStringToPath(a.instance, "/interaction_profiles/ext/eye_gaze_interaction", &sb.interactionProfile));
    sb.countSuggestedBindings = 1;
    sb.suggestedBindings = &b;
    XR_OK(xrSuggestInteractionProfileBindings(a.instance, &sb));

    XrGraphicsBindingVulkanKHR gb{XR_TYPE_GRAPHICS_BINDING_VULKAN_KHR};
    gb.instance = a.vk.instance;
    gb.physicalDevice = a.vk.phys;
    gb.device = a.vk.device;
    gb.queueFamilyIndex = a.vk.family;
    XrSessionCreateInfo sci{XR_TYPE_SESSION_CREATE_INFO, &gb};
    sci.systemId = a.system;
    XR_OK(xrCreateSession(a.instance, &sci, &a.session));
    XrReferenceSpaceCreateInfo rsi{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    rsi.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    rsi.poseInReferenceSpace.orientation.w = 1;
    XR_OK(xrCreateReferenceSpace(a.session, &rsi, &a.local));
    XrActionSpaceCreateInfo gsi{XR_TYPE_ACTION_SPACE_CREATE_INFO};
    gsi.action = a.gaze;
    gsi.poseInActionSpace.orientation.w = 1;
    XR_OK(xrCreateActionSpace(a.session, &gsi, &a.gazeSpace));

    auto resetCreated = fakeFn<void (*)()>("fake_reset_created");
    auto createdUsage = fakeFn<uint32_t (*)(uint32_t)>("fake_created_usage");
    resetCreated();
    for (uint32_t i = 0; i < 4; i++) {
        XrSwapchainCreateInfo ci{XR_TYPE_SWAPCHAIN_CREATE_INFO};
        ci.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT;
        ci.format = format;
        ci.sampleCount = 1;
        ci.width = a.cfg[i].recommendedImageRectWidth;
        ci.height = a.cfg[i].recommendedImageRectHeight;
        ci.faceCount = ci.arraySize = ci.mipCount = 1;
        XR_OK(xrCreateSwapchain(a.session, &ci, &a.sc[i]));
        CHECK(createdUsage(i) & XR_SWAPCHAIN_USAGE_SAMPLED_BIT); // the layer samples the game's views
        uint32_t k = 0;
        xrEnumerateSwapchainImages(a.sc[i], 0, &k, nullptr);
        std::vector<XrSwapchainImageVulkanKHR> imgs(k, {XR_TYPE_SWAPCHAIN_IMAGE_VULKAN_KHR});
        XR_OK(xrEnumerateSwapchainImages(a.sc[i], k, &k, reinterpret_cast<XrSwapchainImageBaseHeader*>(imgs.data())));
        for (auto& im : imgs) a.images[i].push_back(im.image);
    }

    XrSessionBeginInfo sbi{XR_TYPE_SESSION_BEGIN_INFO};
    sbi.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_QUAD_VARJO;
    XR_OK(xrBeginSession(a.session, &sbi));
    XrSessionActionSetsAttachInfo att{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
    att.countActionSets = 1;
    att.actionSets = &a.set;
    XR_OK(xrAttachSessionActionSets(a.session, &att));

    const uint32_t colours[4] = {kRed, kBlue, kGreen, kYellow};
    auto frameViews = fakeFn<uint32_t (*)()>("fake_frame_views");
    auto frameView = fakeFn<const uint32_t* (*)(uint32_t, uint32_t*, uint32_t*, XrFovf*)>("fake_frame_view");

    for (int f = 0; f < 10; f++) {
        const bool withGaze = f >= 5;
        setGaze(withGaze ? 1 : 0, 12.f, 5.f); // 12 degrees right, 5 up
        XrFrameState fs{XR_TYPE_FRAME_STATE};
        XR_OK(xrWaitFrame(a.session, nullptr, &fs));
        XR_OK(xrBeginFrame(a.session, nullptr));
        XrActiveActionSet active{a.set, XR_NULL_PATH};
        XrActionsSyncInfo si{XR_TYPE_ACTIONS_SYNC_INFO};
        si.countActiveActionSets = 1;
        si.activeActionSets = &active;
        XR_OK(xrSyncActions(a.session, &si));
        XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO};
        gi.action = a.gaze;
        XrActionStatePose ps{XR_TYPE_ACTION_STATE_POSE};
        XR_OK(xrGetActionStatePose(a.session, &gi, &ps));
        CHECK(bool(ps.isActive) == withGaze); // the game's own gaze binding survived the layer's

        XrViewLocateFoveatedRenderingVARJO fl{XR_TYPE_VIEW_LOCATE_FOVEATED_RENDERING_VARJO, nullptr, XR_TRUE};
        XrViewLocateInfo li{XR_TYPE_VIEW_LOCATE_INFO, &fl};
        li.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_QUAD_VARJO;
        li.displayTime = fs.predictedDisplayTime;
        li.space = a.local;
        XrViewState vs{XR_TYPE_VIEW_STATE};
        std::array<XrView, 4> views;
        views.fill({XR_TYPE_VIEW});
        XR_OK(xrLocateViews(a.session, &li, &vs, 4, &n, views.data()));
        CHECK(n == 4);

        std::array<XrCompositionLayerProjectionView, 4> pv{};
        for (uint32_t i = 0; i < 4; i++) {
            uint32_t idx = 0;
            XR_OK(xrAcquireSwapchainImage(a.sc[i], nullptr, &idx));
            XrSwapchainImageWaitInfo wi{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
            wi.timeout = XR_INFINITE_DURATION;
            XR_OK(xrWaitSwapchainImage(a.sc[i], &wi));
            a.vk.fill(a.images[i][idx], a.cfg[i].recommendedImageRectWidth, a.cfg[i].recommendedImageRectHeight,
                      colours[i]);
            XR_OK(xrReleaseSwapchainImage(a.sc[i], nullptr));
            pv[i] = {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW};
            pv[i].pose = views[i].pose;
            pv[i].fov = views[i].fov;
            pv[i].subImage.swapchain = a.sc[i];
            pv[i].subImage.imageRect = {{0, 0},
                                        {int32_t(a.cfg[i].recommendedImageRectWidth),
                                         int32_t(a.cfg[i].recommendedImageRectHeight)}};
        }
        XrCompositionLayerProjection layer{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
        layer.space = a.local;
        layer.viewCount = 4;
        layer.views = pv.data();
        const XrCompositionLayerBaseHeader* layers[] = {reinterpret_cast<XrCompositionLayerBaseHeader*>(&layer)};
        XrFrameEndInfo fe{XR_TYPE_FRAME_END_INFO};
        fe.displayTime = fs.predictedDisplayTime;
        fe.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
        fe.layerCount = 1;
        fe.layers = layers;
        XR_OK(xrEndFrame(a.session, &fe));

        if (f != 4 && f != 9) {
            continue;
        }
        const char* what = withGaze ? "gaze" : "fixed";
        // The focus field: centred on the eye's view of a point 2 m ahead (fixed) or along the gaze.
        for (uint32_t e = 0; e < 2; e++) {
            const XrFovf& o = views[e].fov;
            const XrFovf& fo = views[2 + e].fov;
            const double share = withGaze ? 0.35 : 0.5;
            const double spanO = std::tan(o.angleRight) - std::tan(o.angleLeft);
            const double spanF = std::tan(fo.angleRight) - std::tan(fo.angleLeft);
            CHECK_NEAR(spanF / spanO, share, 1e-4);
            // The gaze direction of the fake (pitch, then yaw) and the point 2 m along it, seen from this eye.
            const double ex = e == 0 ? -0.032 : 0.032;
            const double yaw = withGaze ? 12 * M_PI / 180 : 0, pitch = withGaze ? 5 * M_PI / 180 : 0;
            const double dx = std::cos(pitch) * std::sin(yaw), dy = std::sin(pitch),
                         dz = -std::cos(pitch) * std::cos(yaw);
            const double wantX = (2 * dx - ex) / (-2 * dz), wantY = (2 * dy) / (-2 * dz);
            CHECK_NEAR(centreTan(fo.angleLeft, fo.angleRight), wantX, 2e-3);
            CHECK_NEAR(centreTan(fo.angleDown, fo.angleUp), wantY, 2e-3);
            std::printf("  %s eye %u: focus centre tan %.4f (want %.4f), span %.3f of the field\n", what, e,
                        centreTan(fo.angleLeft, fo.angleRight), wantX, spanF / spanO);
        }

        // What reached the runtime: two composed images of the full field.
        CHECK(frameViews() == 2);
        for (uint32_t e = 0; e < 2 && frameViews() == 2; e++) {
            uint32_t w = 0, h = 0;
            XrFovf ofov{};
            const uint32_t* px = frameView(e, &w, &h, &ofov);
            CHECK(w == 1000 && h == 1000);
            CHECK_NEAR(ofov.angleLeft, views[e].fov.angleLeft, 1e-6);
            const auto r = focusRectPx(views[e].fov, views[2 + e].fov, w, h);
            const uint32_t periph = e == 0 ? kRed : kBlue, focus = e == 0 ? kGreen : kYellow;
            const auto at = [&](double x, double y) { return px[size_t(y) * w + size_t(x)]; };
            const uint32_t centre = at((r[0] + r[2]) / 2, (r[1] + r[3]) / 2);
            const uint32_t corner = at(3, 3);
            CHECK(std::abs(ch(centre, 0) - ch(focus, 0)) <= 2 && std::abs(ch(centre, 1) - ch(focus, 1)) <= 2 &&
                  std::abs(ch(centre, 2) - ch(focus, 2)) <= 2);
            CHECK(std::abs(ch(corner, 0) - ch(periph, 0)) <= 2 && std::abs(ch(corner, 2) - ch(periph, 2)) <= 2);
            // The blend: half way at 10 % of the focus width inside its edge (transition 0.2, smoothstep).
            const double y = (r[1] + r[3]) / 2;
            const int c = 1; // green rises in both eyes (green focus over red, yellow over blue)
            // In linear light: an sRGB image is blended after decoding, as it should be.
            const bool srgb = format == VK_FORMAT_R8G8B8A8_SRGB;
            const auto lin = [&](int v) {
                const double x = v / 255.0;
                return srgb ? (x <= 0.04045 ? x / 12.92 : std::pow((x + 0.055) / 1.055, 2.4)) : x;
            };
            const int lo = ch(periph, c), hi = ch(focus, c);
            double half = -1;
            for (uint32_t x = uint32_t(std::max(0.0, r[0] - 5)); x < uint32_t(r[2]); x++) {
                if (lin(ch(at(x, y), c)) >= (lin(lo) + lin(hi)) / 2) {
                    half = x + 0.5;
                    break;
                }
            }
            const double want = r[0] + 0.1 * (r[2] - r[0]);
            CHECK_NEAR(half, want, 2.0);
            CHECK(ch(at(r[0] - 3, y), c) <= lo + 2); // just outside: the wide view only
            std::printf("  %s eye %u: focus %.1f..%.1f x %.1f..%.1f px, 50%% blend at x %.1f (want %.1f)\n", what, e,
                        r[0], r[2], r[1], r[3], half, want);
        }
    }

    XR_OK(xrEndSession(a.session));
    XR_OK(xrDestroySession(a.session));
    XR_OK(xrDestroyInstance(a.instance));
    a.vk.destroy();
}

// A game without quad views: the layer stays out of the way.
void runStereo() {
    const char* exts[] = {XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME};
    XrInstanceCreateInfo ici{XR_TYPE_INSTANCE_CREATE_INFO};
    std::strcpy(ici.applicationInfo.applicationName, "qv e2e stereo");
    ici.applicationInfo.apiVersion = XR_MAKE_VERSION(1, 0, 0);
    ici.enabledExtensionCount = 1;
    ici.enabledExtensionNames = exts;
    XrInstance inst = XR_NULL_HANDLE;
    XR_OK(xrCreateInstance(&ici, &inst));
    XrSystemGetInfo sgi{XR_TYPE_SYSTEM_GET_INFO};
    sgi.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    XrSystemId sys;
    XR_OK(xrGetSystem(inst, &sgi, &sys));
    uint32_t n = 0;
    XrViewConfigurationType types[4];
    XR_OK(xrEnumerateViewConfigurations(inst, sys, 4, &n, types));
    CHECK(n == 1 && types[0] == XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO);
    XR_OK(xrDestroyInstance(inst));
}

// Native mode (a runtime with quad views of its own, as Monado): the four views stay the runtime's; the layer only
// moves the focus views onto the gaze and answers foveated rendering.
void runNative() {
    ::setenv("QV_FAKE_NATIVE_QUAD", "1", 1);
    App a;
    CHECK(a.vk.init());
    const char* exts[] = {XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME, XR_VARJO_QUAD_VIEWS_EXTENSION_NAME,
                          XR_VARJO_FOVEATED_RENDERING_EXTENSION_NAME};
    XrInstanceCreateInfo ici{XR_TYPE_INSTANCE_CREATE_INFO};
    std::strcpy(ici.applicationInfo.applicationName, "qv e2e native");
    ici.applicationInfo.apiVersion = XR_MAKE_VERSION(1, 0, 0);
    ici.enabledExtensionCount = 3;
    ici.enabledExtensionNames = exts;
    XR_OK(xrCreateInstance(&ici, &a.instance));
    auto setGaze = fakeFn<void (*)(int, float, float)>("fake_set_gaze");
    setGaze(0, 0, 0);
    XrSystemGetInfo sgi{XR_TYPE_SYSTEM_GET_INFO};
    sgi.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    XR_OK(xrGetSystem(a.instance, &sgi, &a.system));
    XrSystemFoveatedRenderingPropertiesVARJO fov{XR_TYPE_SYSTEM_FOVEATED_RENDERING_PROPERTIES_VARJO};
    XrSystemProperties sp{XR_TYPE_SYSTEM_PROPERTIES, &fov};
    XR_OK(xrGetSystemProperties(a.instance, a.system, &sp));
    CHECK(fov.supportsFoveatedRendering == XR_TRUE); // answered by the layer: the runtime has eye gaze
    uint32_t n = 0;
    XrViewConfigurationType types[4];
    XR_OK(xrEnumerateViewConfigurations(a.instance, a.system, 4, &n, types));
    CHECK(n == 2); // the runtime's own list, not the layer's
    for (uint32_t i = 0; i < 4; i++) {
        a.cfg[i] = {XR_TYPE_VIEW_CONFIGURATION_VIEW};
    }
    XR_OK(xrEnumerateViewConfigurationViews(a.instance, a.system, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_QUAD_VARJO, 4, &n,
                                            a.cfg.data()));
    CHECK(n == 4);
    CHECK(a.cfg[0].recommendedImageRectWidth == 400 && a.cfg[2].recommendedImageRectWidth == 350); // the settings' sizes
    PFN_xrGetVulkanGraphicsRequirements2KHR req = nullptr;
    XR_OK(xrGetInstanceProcAddr(a.instance, "xrGetVulkanGraphicsRequirements2KHR",
                                reinterpret_cast<PFN_xrVoidFunction*>(&req)));
    XrGraphicsRequirementsVulkanKHR gr{XR_TYPE_GRAPHICS_REQUIREMENTS_VULKAN_KHR};
    XR_OK(req(a.instance, a.system, &gr));
    XrGraphicsBindingVulkanKHR gb{XR_TYPE_GRAPHICS_BINDING_VULKAN_KHR};
    gb.instance = a.vk.instance;
    gb.physicalDevice = a.vk.phys;
    gb.device = a.vk.device;
    gb.queueFamilyIndex = a.vk.family;
    XrSessionCreateInfo sci{XR_TYPE_SESSION_CREATE_INFO, &gb};
    sci.systemId = a.system;
    XR_OK(xrCreateSession(a.instance, &sci, &a.session));
    XrReferenceSpaceCreateInfo rsi{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    rsi.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    rsi.poseInReferenceSpace.orientation.w = 1;
    XR_OK(xrCreateReferenceSpace(a.session, &rsi, &a.local));
    XrSessionBeginInfo sbi{XR_TYPE_SESSION_BEGIN_INFO};
    sbi.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_QUAD_VARJO;
    XR_OK(xrBeginSession(a.session, &sbi)); // the runtime itself took quad views
    // no action set of the game: the layer attaches its gaze after 100 frames, as for games without controllers
    float centreFixed = 0, centreGaze = 0;
    for (int f = 0; f < 130; f++) {
        setGaze(f >= 110 ? 1 : 0, 12.f, 0.f); // 12 degrees right from frame 110
        XrFrameState fs{XR_TYPE_FRAME_STATE};
        XR_OK(xrWaitFrame(a.session, nullptr, &fs));
        XR_OK(xrBeginFrame(a.session, nullptr));
        XrViewLocateInfo li{XR_TYPE_VIEW_LOCATE_INFO};
        li.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_QUAD_VARJO;
        li.displayTime = fs.predictedDisplayTime;
        li.space = a.local;
        XrViewState vs{XR_TYPE_VIEW_STATE};
        std::array<XrView, 4> v{};
        for (auto& x : v) x = {XR_TYPE_VIEW};
        XR_OK(xrLocateViews(a.session, &li, &vs, 4, &n, v.data()));
        CHECK(n == 4);
        const float c = (std::tan(v[2].fov.angleLeft) + std::tan(v[2].fov.angleRight)) / 2;
        if (f == 105) centreFixed = c;
        if (f == 129) centreGaze = c;
        XrFrameEndInfo fe{XR_TYPE_FRAME_END_INFO};
        fe.displayTime = fs.predictedDisplayTime;
        fe.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
        XR_OK(xrEndFrame(a.session, &fe));
    }
    std::printf("  native: focus centre tangent fixed %.3f, gaze 12 deg right %.3f\n", centreFixed, centreGaze);
    // the layer's own eye locate uses the session's type: the runtime refused none (BMS: one refusal per frame)
    CHECK(fakeFn<uint32_t (*)()>("fake_locate_rejects")() == 0);
    CHECK(std::fabs(centreFixed) < 0.08f); // centred (the eye's own canting aside) without a gaze
    CHECK(centreGaze > centreFixed + 0.12f); // follows the gaze to the right (tan 12 deg = 0.21)
    xrDestroySession(a.session);
    xrDestroyInstance(a.instance);
    ::unsetenv("QV_FAKE_NATIVE_QUAD");
}

} // namespace

TEST(quad_views_compose_end_to_end) {
    runQuad(VK_FORMAT_R8G8B8A8_UNORM);
}

// Falcon BMS picks an sRGB format: sampling decodes, the composed image encodes, the colours come out unchanged.
TEST(quad_views_compose_srgb) {
    runQuad(VK_FORMAT_R8G8B8A8_SRGB);
}

TEST(stereo_game_passes_through) {
    runStereo();
}

TEST(native_quad_views_focus_follows_the_gaze) {
    runNative();
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::printf("usage: %s <fake runtime .so> <layer manifest .json>\n", argv[0]);
        return 2;
    }
    g_runtime = fs::absolute(argv[1]).string();
    g_manifest = fs::absolute(argv[2]).string();
    setupEnvironment();
    for (auto& c : qvtest::cases()) {
        const int before = qvtest::failures();
        c.fn();
        std::printf("%s %s\n", qvtest::failures() == before ? "ok  " : "FAIL", c.name);
    }
    return qvtest::failures() ? 1 : 0;
}
