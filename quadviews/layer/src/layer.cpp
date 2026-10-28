// SPDX-License-Identifier: MIT
// The OpenXR API layer: quad views (XR_VARJO_quad_views + XR_VARJO_foveated_rendering) on top of a stereo runtime,
// for Vulkan apps.
//
// What the game sees: a fourth view configuration, PRIMARY_QUAD_VARJO, with two wide views at a low pixel density
// and two focus views at full density over a part of each eye's field. With eye tracking (the runtime's
// XR_EXT_eye_gaze_interaction) the focus follows the gaze; without it, it sits in the middle of the field.
//
// What the runtime sees: an ordinary stereo session. In xrEndFrame the two views of each eye are composed into one
// image per eye (compositor_vk.cpp), which is submitted in a projection layer with the wide view's pose and field.
//
// The design follows Quad-Views-Foveated by Matthieu Bucchianeri (MIT, D3D11 only) and borrows ideas from VectorXR
// (keep the focus size at the edge of the field, a diagnostic overlay); see README.md for the differences.
// clang-format off
#include <vulkan/vulkan.h> // before openxr_platform.h (its Vulkan structs)
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <openxr/openxr_loader_negotiation.h>
// clang-format on

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "compositor_vk.h"
#include "fovmath.h"
#include "gaze.h"
#include "log.h"
#include "settings.h"

#define QV_EXPORT extern "C" __attribute__((visibility("default")))

namespace qv {
namespace {

constexpr const char* kLayerName = "XR_APILAYER_ARTSCOUT_quad_views_vulkan";
constexpr XrViewConfigurationType kQuad = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_QUAD_VARJO;
constexpr XrViewConfigurationType kStereo = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
// The gaze is turned into a point this far ahead (metres) and projected into each eye.
constexpr float kGazeDistance = 2.0f;

const char* xrName(XrResult r) {
    switch (r) {
    case XR_SUCCESS: return "XR_SUCCESS";
    case XR_ERROR_VALIDATION_FAILURE: return "XR_ERROR_VALIDATION_FAILURE";
    case XR_ERROR_RUNTIME_FAILURE: return "XR_ERROR_RUNTIME_FAILURE";
    case XR_ERROR_HANDLE_INVALID: return "XR_ERROR_HANDLE_INVALID";
    case XR_ERROR_EXTENSION_NOT_PRESENT: return "XR_ERROR_EXTENSION_NOT_PRESENT";
    case XR_ERROR_FUNCTION_UNSUPPORTED: return "XR_ERROR_FUNCTION_UNSUPPORTED";
    case XR_ERROR_SESSION_NOT_RUNNING: return "XR_ERROR_SESSION_NOT_RUNNING";
    case XR_ERROR_ACTIONSETS_ALREADY_ATTACHED: return "XR_ERROR_ACTIONSETS_ALREADY_ATTACHED";
    case XR_TIMEOUT_EXPIRED: return "XR_TIMEOUT_EXPIRED";
    default: return "XrResult";
    }
}

template <typename T>
T* findInChain(void* next, XrStructureType type) {
    for (auto* p = reinterpret_cast<XrBaseOutStructure*>(next); p; p = p->next) {
        if (p->type == type) {
            return reinterpret_cast<T*>(p);
        }
    }
    return nullptr;
}

template <typename T>
const T* findInChain(const void* next, XrStructureType type) {
    for (auto* p = reinterpret_cast<const XrBaseInStructure*>(next); p; p = p->next) {
        if (p->type == type) {
            return reinterpret_cast<const T*>(p);
        }
    }
    return nullptr;
}

// The functions of the next layer (or the runtime).
struct Next {
    PFN_xrGetInstanceProcAddr getInstanceProcAddr = nullptr;
#define QV_NEXT_FNS(X)                                                                                                 \
    X(xrDestroyInstance)                                                                                               \
    X(xrGetInstanceProperties)                                                                                         \
    X(xrGetSystem)                                                                                                     \
    X(xrGetSystemProperties)                                                                                           \
    X(xrEnumerateViewConfigurations)                                                                                   \
    X(xrGetViewConfigurationProperties)                                                                                \
    X(xrEnumerateViewConfigurationViews)                                                                               \
    X(xrEnumerateEnvironmentBlendModes)                                                                                \
    X(xrCreateSession)                                                                                                 \
    X(xrDestroySession)                                                                                                \
    X(xrBeginSession)                                                                                                  \
    X(xrBeginFrame)                                                                                                    \
    X(xrEndFrame)                                                                                                      \
    X(xrLocateViews)                                                                                                   \
    X(xrCreateSwapchain)                                                                                               \
    X(xrDestroySwapchain)                                                                                              \
    X(xrEnumerateSwapchainImages)                                                                                      \
    X(xrAcquireSwapchainImage)                                                                                         \
    X(xrWaitSwapchainImage)                                                                                            \
    X(xrReleaseSwapchainImage)                                                                                         \
    X(xrCreateReferenceSpace)                                                                                          \
    X(xrDestroySpace)                                                                                                  \
    X(xrLocateSpace)                                                                                                   \
    X(xrPollEvent)                                                                                                     \
    X(xrStringToPath)                                                                                                  \
    X(xrCreateActionSet)                                                                                               \
    X(xrCreateAction)                                                                                                  \
    X(xrCreateActionSpace)                                                                                             \
    X(xrSuggestInteractionProfileBindings)                                                                             \
    X(xrAttachSessionActionSets)                                                                                       \
    X(xrSyncActions)                                                                                                   \
    X(xrGetActionStatePose)                                                                                            \
    X(xrGetVisibilityMaskKHR)
#define QV_DECLARE(name) PFN_##name name = nullptr;
    QV_NEXT_FNS(QV_DECLARE)
#undef QV_DECLARE

    void load(XrInstance instance) {
#define QV_LOAD(name)                                                                                                  \
    if (XR_FAILED(getInstanceProcAddr(instance, #name, reinterpret_cast<PFN_xrVoidFunction*>(&name)))) {               \
        name = nullptr;                                                                                                \
    }
        QV_NEXT_FNS(QV_LOAD)
#undef QV_LOAD
    }
};

struct SwapchainState {
    XrSwapchainCreateInfo info{};
    std::vector<VkImage> images;
    std::deque<uint32_t> acquired;
    uint32_t lastReleased = 0;
    bool releasePending = false; // the game released it; the real release waits until we read it in xrEndFrame
};

struct Output {
    XrSwapchain swapchain = XR_NULL_HANDLE;
    VkFormat format = VK_FORMAT_UNDEFINED;
    Size size{};
    std::vector<VkImage> images;
};

class Layer {
  public:
    std::mutex m;
    Next next;
    XrInstance instance = XR_NULL_HANDLE;
    bool bypass = true;
    // Native: the runtime has quad views itself (Monado) but no foveated rendering. The game's four views go to the
    // runtime unchanged (its compositor puts the focus views in, at the field each one was rendered with); the layer
    // only places the focus views: xrLocateViews hands out focus fields centred on the gaze, the sizes follow the
    // settings, and XR_VARJO_foveated_rendering is answered here.
    bool native = false;
    bool foveatedRequested = false;
    bool eyeGazeEnabled = false; // XR_EXT_eye_gaze_interaction is on in the runtime (we or the game asked for it)
    std::string runtimeName;
    AppId app;
    Settings settings;
    std::filesystem::path settingsFile;
    std::filesystem::file_time_type settingsTime{};
    std::chrono::steady_clock::time_point lastSettingsCheck{};

    XrSystemId systemId = XR_NULL_SYSTEM_ID;
    std::string systemName;
    bool systemEyeGaze = false;
    Size recommended{}, maxSize{};

    // Session.
    XrSession session = XR_NULL_HANDLE;
    bool vulkan = false;
    bool useQuad = false;
    std::unique_ptr<Compositor> compositor;
    XrSpace viewSpace = XR_NULL_HANDLE;
    XrActionSet gazeSet = XR_NULL_HANDLE;
    XrAction gazeAction = XR_NULL_HANDLE;
    XrSpace gazeSpace = XR_NULL_HANDLE;
    XrPath gazeProfile = XR_NULL_PATH, gazePosePath = XR_NULL_PATH;
    bool gameSuggestedGaze = false, attached = false, gameSyncedThisFrame = false;
    uint64_t frames = 0;
    std::set<XrSpace> combinedEyeSpaces;
    std::unordered_map<XrSwapchain, SwapchainState> swapchains;
    std::array<Output, 2> outputs{};
    std::map<XrTime, std::array<XrFovf, 2>> focusFovs;
    GazeTrack gazeTrack;
    bool gazeLive = false;
    std::array<std::array<float, 2>, 2> gazeUv{}; // the focus centre per eye, image uv (debug overlay)
    bool loggedFovMismatch = false, loggedComposeFailure = false;
    std::chrono::steady_clock::time_point lastStatus{};
    // The last session's numbers, kept for the status after it ended.
    Size lastPeripheral{}, lastFocus{}, lastCanvas{};
    double lastGpuMs = 0;
    bool sessionEnded = false;

    bool trackerAvailable() const {
        return eyeGazeEnabled && systemEyeGaze && settings.tracking == Tracking::Eye;
    }

    void reloadSettingsIfChanged(bool force = false) {
        const auto now = std::chrono::steady_clock::now();
        if (!force && now - lastSettingsCheck < std::chrono::milliseconds(500)) {
            return;
        }
        lastSettingsCheck = now;
        std::error_code ec;
        const auto t = std::filesystem::last_write_time(settingsFile, ec);
        if (!force && (ec || t == settingsTime)) {
            return;
        }
        settingsTime = ec ? std::filesystem::file_time_type{} : t;
        const Settings s = loadSettings(settingsFile, app);
        if (force || !(s == settings)) {
            settings = s;
            std::string one = settings.toLines();
            std::replace(one.begin(), one.end(), '\n', ' ');
            logf("Settings%s: %s", force ? "" : " changed", one.c_str());
        }
    }

    void writeStatus(bool now = false) {
        const auto t = std::chrono::steady_clock::now();
        if (!now && t - lastStatus < std::chrono::seconds(1)) {
            return;
        }
        lastStatus = t;
        const auto dir = stateDir();
        if (dir.empty()) {
            return;
        }
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        const auto tmp = dir / "status.txt.tmp";
        {
            std::ofstream o(tmp, std::ios::trunc);
            const auto wall = std::chrono::duration_cast<std::chrono::seconds>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count();
            o << "time=" << wall << "\n"
              << "app=" << app.name << "\n"
              << "exe=" << app.exe << "\n"
              << "runtime=" << runtimeName << "\n"
              << "system=" << systemName << "\n"
              << "session=" << (session ? (useQuad ? "quad" : "stereo") : (sessionEnded ? "ended" : "none")) << "\n"
              << "graphics=" << (vulkan ? "vulkan" : "other") << "\n"
              << "eye_tracking=" << (trackerAvailable() ? (gazeLive ? "live" : "lost") : "off") << "\n"
              << "recommended=" << recommended.w << "x" << recommended.h << "\n"
              << "peripheral=" << lastPeripheral.w << "x" << lastPeripheral.h << "\n"
              << "focus=" << lastFocus.w << "x" << lastFocus.h << "\n"
              << "canvas=" << lastCanvas.w << "x" << lastCanvas.h << "\n"
              << "compose_gpu_ms=" << lastGpuMs << "\n"
              << "frames=" << frames << "\n";
        }
        std::filesystem::rename(tmp, dir / "status.txt", ec);
    }

    void resetSession() {
        for (Output& o : outputs) {
            if (o.swapchain && next.xrDestroySwapchain) {
                next.xrDestroySwapchain(o.swapchain);
            }
            o = Output{};
        }
        compositor.reset();
        if (gazeSpace && next.xrDestroySpace) next.xrDestroySpace(gazeSpace);
        if (viewSpace && next.xrDestroySpace) next.xrDestroySpace(viewSpace);
        gazeSpace = viewSpace = XR_NULL_HANDLE;
        session = XR_NULL_HANDLE;
        vulkan = useQuad = false;
        attached = gameSyncedThisFrame = false;
        frames = 0;
        combinedEyeSpaces.clear();
        swapchains.clear();
        focusFovs.clear();
        gazeTrack = GazeTrack{};
        gazeLive = false;
    }

    // Our action for the gaze (created once per instance, before the game attaches its action sets).
    void ensureGazeAction() {
        if (gazeSet || !eyeGazeEnabled || !next.xrCreateActionSet) {
            return;
        }
        next.xrStringToPath(instance, "/interaction_profiles/ext/eye_gaze_interaction", &gazeProfile);
        next.xrStringToPath(instance, "/user/eyes_ext/input/gaze_ext/pose", &gazePosePath);
        XrActionSetCreateInfo si{XR_TYPE_ACTION_SET_CREATE_INFO};
        std::strcpy(si.actionSetName, "qv_vulkan_eye_tracker");
        std::strcpy(si.localizedActionSetName, "Quad views eye tracker");
        if (XR_FAILED(next.xrCreateActionSet(instance, &si, &gazeSet))) {
            logf("Eye tracking: could not create the action set");
            gazeSet = XR_NULL_HANDLE;
            return;
        }
        XrActionCreateInfo ai{XR_TYPE_ACTION_CREATE_INFO};
        std::strcpy(ai.actionName, "qv_vulkan_gaze");
        std::strcpy(ai.localizedActionName, "Quad views gaze");
        ai.actionType = XR_ACTION_TYPE_POSE_INPUT;
        if (XR_FAILED(next.xrCreateAction(gazeSet, &ai, &gazeAction))) {
            logf("Eye tracking: could not create the action");
            gazeAction = XR_NULL_HANDLE;
        }
    }

    void suggestOurGazeOnly() {
        XrActionSuggestedBinding b{gazeAction, gazePosePath};
        XrInteractionProfileSuggestedBinding s{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
        s.interactionProfile = gazeProfile;
        s.countSuggestedBindings = 1;
        s.suggestedBindings = &b;
        const XrResult r = next.xrSuggestInteractionProfileBindings(instance, &s);
        if (XR_FAILED(r)) {
            logf("Eye tracking: suggesting the gaze binding failed: %s", xrName(r));
        }
    }

    // The gaze as a tangent in view space at `time`, unfiltered; nothing when the tracker has none.
    std::optional<GazeTrack::Dir> readGaze(XrTime time) {
        if (!gazeAction || !attached || !gazeSpace) {
            return std::nullopt;
        }
        XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO};
        gi.action = gazeAction;
        XrActionStatePose st{XR_TYPE_ACTION_STATE_POSE};
        if (XR_FAILED(next.xrGetActionStatePose(session, &gi, &st)) || !st.isActive) {
            return std::nullopt;
        }
        XrSpaceLocation loc{XR_TYPE_SPACE_LOCATION};
        if (XR_FAILED(next.xrLocateSpace(gazeSpace, viewSpace, time, &loc)) ||
            !(loc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)) {
            return std::nullopt;
        }
        const XrVector3f d = rotate(loc.pose.orientation, {0.f, 0.f, -1.f});
        if (!(d.z < -1e-3f)) {
            return std::nullopt;
        }
        return GazeTrack::Dir{d.x / -d.z, d.y / -d.z};
    }

    // The two focus fields for the located stereo views.
    void computeFocus(const XrViewLocateInfo* info, XrView* views, bool foveated) {
        std::optional<GazeTrack::Dir> gaze;
        if (foveated && trackerAvailable()) {
            gaze = gazeTrack.update(info->displayTime * 1e-9, readGaze(info->displayTime), settings.gazeFilter,
                                    settings.gazeHoldMs);
        }
        gazeLive = gaze.has_value();

        // The eyes in view space (for the parallax of the gaze point and the canted displays). In native mode the
        // runtime's session runs the quad configuration, and Monado refuses any other type in xrLocateViews
        // (XR_ERROR_VALIDATION_FAILURE every frame, and the focus fell back to the plain gaze tangent): locate with
        // the session's type, room for its four views, and use the first two (the eyes).
        XrView eyes[4]{{XR_TYPE_VIEW}, {XR_TYPE_VIEW}, {XR_TYPE_VIEW}, {XR_TYPE_VIEW}};
        bool haveEyes = false;
        if (viewSpace) {
            XrViewLocateInfo li = *info;
            li.next = nullptr;
            li.viewConfigurationType = native ? info->viewConfigurationType : kStereo;
            li.space = viewSpace;
            XrViewState vs{XR_TYPE_VIEW_STATE};
            uint32_t n = 0;
            const uint32_t want = native ? 4u : 2u;
            haveEyes = XR_SUCCEEDED(next.xrLocateViews(session, &li, &vs, want, &n, eyes)) && n >= 2 &&
                       (vs.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT);
        }

        XrVector3f point{0.f, 0.f, -kGazeDistance};
        if (gaze) {
            const float n = std::sqrt(float(gaze->x * gaze->x + gaze->y * gaze->y + 1.0));
            point = {float(gaze->x) / n * kGazeDistance, float(gaze->y) / n * kGazeDistance, -kGazeDistance / n};
        }
        const auto& share = gaze ? settings.gazeSize : settings.fixedSize;
        std::array<XrFovf, 2> fovs{};
        for (uint32_t e = 0; e < 2; e++) {
            const Tan base = toTan(views[e].fov);
            float cx = 0.f, cy = 0.f;
            if (!(haveEyes && eyeTangent(eyes[e].pose, point, cx, cy))) {
                cx = gaze ? float(gaze->x) : 0.f;
                cy = gaze ? float(gaze->y) : 0.f;
            }
            cx = addAngle(cx, settings.offsetDeg[0]);
            cy = addAngle(cy, settings.offsetDeg[1]);
            const Tan win = focusWindow(base, cx, cy, share[0], share[1]);
            fovs[e] = toFov(win);
            views[2 + e].pose = views[e].pose;
            views[2 + e].fov = fovs[e];
            gazeUv[e] = {(cx - base.left) / (base.right - base.left), (base.up - cy) / (base.up - base.down)};
        }
        focusFovs[info->displayTime] = fovs;
    }

    bool ensureOutput(uint32_t eye, VkFormat format, const XrSwapchainCreateInfo& like) {
        Output& o = outputs[eye];
        Size want = scaledSize(recommended.w ? recommended : Size{like.width, like.height}, maxSize,
                               recommended.w ? settings.focusDensity : 1.f);
        if (o.swapchain && o.format == format && o.size == want) {
            return true;
        }
        if (o.swapchain) {
            for (VkImage img : o.images) compositor->forgetImage(img);
            next.xrDestroySwapchain(o.swapchain);
            o = Output{};
        }
        XrSwapchainCreateInfo ci{XR_TYPE_SWAPCHAIN_CREATE_INFO};
        ci.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
        ci.format = format;
        ci.sampleCount = 1;
        ci.width = want.w;
        ci.height = want.h;
        ci.faceCount = 1;
        ci.arraySize = 1;
        ci.mipCount = 1;
        XrResult r = next.xrCreateSwapchain(session, &ci, &o.swapchain);
        if (XR_FAILED(r)) {
            logf("Composed image: xrCreateSwapchain %ux%u format %d failed: %s", want.w, want.h, format, xrName(r));
            o = Output{};
            return false;
        }
        uint32_t n = 0;
        next.xrEnumerateSwapchainImages(o.swapchain, 0, &n, nullptr);
        std::vector<XrSwapchainImageVulkanKHR> imgs(n, {XR_TYPE_SWAPCHAIN_IMAGE_VULKAN_KHR});
        next.xrEnumerateSwapchainImages(o.swapchain, n, &n, reinterpret_cast<XrSwapchainImageBaseHeader*>(imgs.data()));
        for (const auto& i : imgs) o.images.push_back(i.image);
        o.format = format;
        o.size = want;
        logf("Composed image for eye %u: %ux%u, format %d, %u images", eye, want.w, want.h, format, n);
        return true;
    }

    SwapchainState* stateOf(XrSwapchain sc) {
        auto it = swapchains.find(sc);
        if (it == swapchains.end()) {
            return nullptr;
        }
        if (it->second.images.empty()) {
            uint32_t n = 0;
            next.xrEnumerateSwapchainImages(sc, 0, &n, nullptr);
            std::vector<XrSwapchainImageVulkanKHR> imgs(n, {XR_TYPE_SWAPCHAIN_IMAGE_VULKAN_KHR});
            next.xrEnumerateSwapchainImages(sc, n, &n, reinterpret_cast<XrSwapchainImageBaseHeader*>(imgs.data()));
            for (const auto& i : imgs) it->second.images.push_back(i.image);
        }
        return &it->second;
    }

    static ImageRef refOf(const SwapchainState& s, const XrSwapchainSubImage& sub) {
        ImageRef r;
        r.image = s.images.at(s.lastReleased);
        r.layer = sub.imageArrayIndex;
        r.format = VkFormat(s.info.format);
        r.width = s.info.width;
        r.height = s.info.height;
        return r;
    }

    static std::array<float, 4> srcRect(const SwapchainState& s, const XrRect2Di& rc) {
        return {float(rc.offset.x) / s.info.width, float(rc.offset.y) / s.info.height,
                float(rc.extent.width) / s.info.width, float(rc.extent.height) / s.info.height};
    }

    // Compose the two eyes of a quad projection layer; on success `outViews` are the two composed views.
    bool composeLayer(const XrCompositionLayerProjection* proj, XrTime displayTime,
                      std::array<XrCompositionLayerProjectionView, 2>& outViews) {
        if (!compositor || !compositor->ready()) {
            return false;
        }
        std::array<EyeJob, 2> jobs{};
        std::array<uint32_t, 2> outIndex{};
        const auto fovIt = focusFovs.find(displayTime);
        for (uint32_t e = 0; e < 2; e++) {
            const XrCompositionLayerProjectionView& pv = proj->views[e];
            const XrCompositionLayerProjectionView& fv = proj->views[2 + e];
            SwapchainState* ps = stateOf(pv.subImage.swapchain);
            SwapchainState* fs = stateOf(fv.subImage.swapchain);
            if (!ps || !fs || ps->images.empty() || fs->images.empty()) {
                logf("Compose: unknown swapchain in the quad layer");
                return false;
            }
            if (ps->info.sampleCount > 1 || fs->info.sampleCount > 1) {
                logf("Compose: multisampled swapchains are not supported");
                return false;
            }
            // The submitted field is the one the image was drawn with (Falcon BMS snaps the located field to its
            // texel grid before drawing and submits the snapped one); the located field only stands in for a
            // submit that carries no usable field.
            XrFovf focusFov = fv.fov;
            const bool submittedOk =
                fv.fov.angleRight > fv.fov.angleLeft && fv.fov.angleUp > fv.fov.angleDown;
            if (fovIt != focusFovs.end()) {
                const XrFovf& ours = fovIt->second[e];
                if (!submittedOk) {
                    focusFov = ours;
                } else if (!loggedFovMismatch && std::fabs(ours.angleLeft - fv.fov.angleLeft) +
                                                         std::fabs(ours.angleUp - fv.fov.angleUp) >
                                                     1e-3f) {
                    logf("The game submits other focus fields than it located; using the submitted ones");
                    loggedFovMismatch = true;
                }
            }
            if (!ensureOutput(e, VkFormat(ps->info.format), ps->info)) {
                return false;
            }
            Output& o = outputs[e];
            XrSwapchainImageAcquireInfo ai{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
            if (XR_FAILED(next.xrAcquireSwapchainImage(o.swapchain, &ai, &outIndex[e]))) {
                return false;
            }
            XrSwapchainImageWaitInfo wi{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
            wi.timeout = 100'000'000;
            if (XR_FAILED(next.xrWaitSwapchainImage(o.swapchain, &wi))) {
                XrSwapchainImageReleaseInfo ri{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
                next.xrReleaseSwapchainImage(o.swapchain, &ri);
                return false;
            }

            EyeJob& j = jobs[e];
            j.peripheral = refOf(*ps, pv.subImage);
            j.focus = refOf(*fs, fv.subImage);
            j.target = {o.images.at(outIndex[e]), 0, o.format, o.size.w, o.size.h};
            ComposeParams& p = j.params;
            p.focusRect = uvRect(toTan(pv.fov), toTan(focusFov));
            p.periSrc = srcRect(*ps, pv.subImage.imageRect);
            p.focusSrc = srcRect(*fs, fv.subImage.imageRect);
            p.periTexel = {1.f / ps->info.width, 1.f / ps->info.height, float(ps->info.width),
                           float(ps->info.height)};
            p.focusTexel = {1.f / fs->info.width, 1.f / fs->info.height, float(fs->info.width),
                            float(fs->info.height)};
            p.params = {settings.transition, settings.sharpen,
                        settings.peripheralFilter == Filter::Bicubic ? 1.f : 0.f, settings.debugOverlay ? 1.f : 0.f};
            p.gaze = {gazeUv[e][0], gazeUv[e][1], gazeLive ? 1.f : 0.f, settings.edgeOpacity};
            lastPeripheral = {uint32_t(pv.subImage.imageRect.extent.width),
                              uint32_t(pv.subImage.imageRect.extent.height)};
            lastFocus = {uint32_t(fv.subImage.imageRect.extent.width), uint32_t(fv.subImage.imageRect.extent.height)};

            outViews[e] = pv;
            outViews[e].subImage.swapchain = o.swapchain;
            outViews[e].subImage.imageArrayIndex = 0;
            outViews[e].subImage.imageRect = {{0, 0}, {int32_t(o.size.w), int32_t(o.size.h)}};
        }
        const bool ok = compositor->compose(jobs.data(), 2);
        lastCanvas = outputs[0].size;
        lastGpuMs = compositor->lastGpuMs();
        for (uint32_t e = 0; e < 2; e++) {
            XrSwapchainImageReleaseInfo ri{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
            next.xrReleaseSwapchainImage(outputs[e].swapchain, &ri);
        }
        return ok;
    }

    void releasePending() {
        for (auto& [sc, s] : swapchains) {
            if (s.releasePending) {
                XrSwapchainImageReleaseInfo ri{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
                next.xrReleaseSwapchainImage(sc, &ri);
                s.releasePending = false;
            }
        }
    }
};

Layer* g = nullptr;

Layer& L() {
    if (!g) {
        g = new Layer(); // never freed: the loader may call into the layer until the process ends
    }
    return *g;
}

// ---- hooks ---------------------------------------------------------------------------------------------------------

XRAPI_ATTR XrResult XRAPI_CALL qvDestroyInstance(XrInstance instance) {
    Layer& l = L();
    std::lock_guard lock(l.m);
    const XrResult r = l.next.xrDestroyInstance(instance);
    if (instance == l.instance) {
        l.resetSession();
        l.instance = XR_NULL_HANDLE;
        l.gazeSet = XR_NULL_HANDLE;
        l.gazeAction = XR_NULL_HANDLE;
        l.systemId = XR_NULL_SYSTEM_ID;
        l.bypass = true;
        l.native = false;
        logf("Instance destroyed");
    }
    return r;
}

XRAPI_ATTR XrResult XRAPI_CALL qvGetSystem(XrInstance instance, const XrSystemGetInfo* info, XrSystemId* id) {
    Layer& l = L();
    std::lock_guard lock(l.m);
    const XrResult r = l.next.xrGetSystem(instance, info, id);
    if (XR_SUCCEEDED(r) && info->formFactor == XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY && *id != l.systemId) {
        l.systemId = *id;
        XrSystemEyeGazeInteractionPropertiesEXT eg{XR_TYPE_SYSTEM_EYE_GAZE_INTERACTION_PROPERTIES_EXT};
        XrSystemProperties sp{XR_TYPE_SYSTEM_PROPERTIES};
        if (l.eyeGazeEnabled) {
            sp.next = &eg;
        }
        if (XR_SUCCEEDED(l.next.xrGetSystemProperties(instance, *id, &sp))) {
            l.systemName = sp.systemName;
            l.systemEyeGaze = l.eyeGazeEnabled && eg.supportsEyeGazeInteraction;
        }
        logf("System: %s, eye tracking %s", l.systemName.c_str(), l.systemEyeGaze ? "available" : "not available");
        l.ensureGazeAction();
    }
    return r;
}

XRAPI_ATTR XrResult XRAPI_CALL qvGetSystemProperties(XrInstance instance, XrSystemId id, XrSystemProperties* p) {
    Layer& l = L();
    std::lock_guard lock(l.m);
    const XrResult r = l.next.xrGetSystemProperties(instance, id, p);
    if (XR_SUCCEEDED(r) && l.foveatedRequested) {
        if (auto* f = findInChain<XrSystemFoveatedRenderingPropertiesVARJO>(
                p->next, XR_TYPE_SYSTEM_FOVEATED_RENDERING_PROPERTIES_VARJO)) {
            f->supportsFoveatedRendering = l.trackerAvailable() ? XR_TRUE : XR_FALSE;
        }
    }
    return r;
}

XRAPI_ATTR XrResult XRAPI_CALL qvEnumerateViewConfigurations(XrInstance instance, XrSystemId id, uint32_t cap,
                                                             uint32_t* count, XrViewConfigurationType* types) {
    Layer& l = L();
    std::lock_guard lock(l.m);
    // Quad views first: the game asked for the extension, so it prefers them.
    if (cap == 0) {
        const XrResult r = l.next.xrEnumerateViewConfigurations(instance, id, 0, count, nullptr);
        if (XR_SUCCEEDED(r)) (*count)++;
        return r;
    }
    const XrResult r = l.next.xrEnumerateViewConfigurations(instance, id, cap - 1, count, types + 1);
    if (r == XR_ERROR_SIZE_INSUFFICIENT) {
        (*count)++;
        return r;
    }
    if (XR_SUCCEEDED(r)) {
        types[0] = kQuad;
        (*count)++;
    }
    return r;
}

XRAPI_ATTR XrResult XRAPI_CALL qvGetViewConfigurationProperties(XrInstance instance, XrSystemId id,
                                                                XrViewConfigurationType type,
                                                                XrViewConfigurationProperties* p) {
    Layer& l = L();
    std::lock_guard lock(l.m);
    const XrResult r = l.next.xrGetViewConfigurationProperties(instance, id, type == kQuad ? kStereo : type, p);
    if (XR_SUCCEEDED(r) && type == kQuad) {
        p->viewConfigurationType = kQuad;
    }
    return r;
}

XRAPI_ATTR XrResult XRAPI_CALL qvEnumerateViewConfigurationViews(XrInstance instance, XrSystemId id,
                                                                 XrViewConfigurationType type, uint32_t cap,
                                                                 uint32_t* count, XrViewConfigurationView* views) {
    Layer& l = L();
    std::lock_guard lock(l.m);
    if (type != kQuad) {
        return l.next.xrEnumerateViewConfigurationViews(instance, id, type, cap, count, views);
    }
    if (l.native) {
        // the runtime's four views, sized by the settings: the wide views at the peripheral density, the focus
        // views at the focus density over their share of the eye's field
        const XrResult nr = l.next.xrEnumerateViewConfigurationViews(instance, id, type, cap, count, views);
        if (XR_FAILED(nr) || cap < 4 || !views) {
            return nr;
        }
        l.reloadSettingsIfChanged();
        const bool fov = l.trackerAvailable();
        const Size rec{views[0].recommendedImageRectWidth, views[0].recommendedImageRectHeight};
        const Size max{views[0].maxImageRectWidth, views[0].maxImageRectHeight};
        l.recommended = rec;
        l.maxSize = max;
        const auto& share = fov ? l.settings.gazeSize : l.settings.fixedSize;
        for (uint32_t i = 0; i < 4; i++) {
            const Size sz = i < 2 ? scaledSize(rec, max, l.settings.peripheralDensity)
                                  : scaledSize(rec, max, l.settings.focusDensity, share[0], share[1]);
            views[i].recommendedImageRectWidth = sz.w;
            views[i].recommendedImageRectHeight = sz.h;
        }
        logf("Views (native): %ux%u -> peripheral %ux%u, focus %ux%u (%s)", rec.w, rec.h,
             views[0].recommendedImageRectWidth, views[0].recommendedImageRectHeight,
             views[2].recommendedImageRectWidth, views[2].recommendedImageRectHeight,
             fov ? "follows the gaze" : "fixed");
        return nr;
    }
    *count = 4;
    if (cap == 0) {
        return XR_SUCCESS;
    }
    if (cap < 4) {
        return XR_ERROR_SIZE_INSUFFICIENT;
    }
    XrViewConfigurationView stereo[2]{{XR_TYPE_VIEW_CONFIGURATION_VIEW}, {XR_TYPE_VIEW_CONFIGURATION_VIEW}};
    uint32_t n = 0;
    const XrResult r = l.next.xrEnumerateViewConfigurationViews(instance, id, kStereo, 2, &n, stereo);
    if (XR_FAILED(r)) {
        return r;
    }
    *count = 4;
    l.reloadSettingsIfChanged();

    // Foveated sizes when the tracker is there and the game asks for them (XrFoveatedViewConfigurationViewVARJO on
    // any view); a game that does not chain the struct gets them too.
    bool chained = false, asked = false;
    for (uint32_t i = 0; i < 4; i++) {
        if (views[i].type != XR_TYPE_VIEW_CONFIGURATION_VIEW) {
            return XR_ERROR_VALIDATION_FAILURE;
        }
        if (auto* f = findInChain<XrFoveatedViewConfigurationViewVARJO>(
                views[i].next, XR_TYPE_FOVEATED_VIEW_CONFIGURATION_VIEW_VARJO)) {
            chained = true;
            asked = asked || f->foveatedRenderingActive;
        }
    }
    const bool foveated = l.trackerAvailable() && (!chained || asked);

    const Size rec{stereo[0].recommendedImageRectWidth, stereo[0].recommendedImageRectHeight};
    const Size max{stereo[0].maxImageRectWidth, stereo[0].maxImageRectHeight};
    l.recommended = rec;
    l.maxSize = max;
    const auto& share = foveated ? l.settings.gazeSize : l.settings.fixedSize;
    for (uint32_t i = 0; i < 4; i++) {
        void* keepNext = views[i].next;
        views[i] = stereo[i % 2];
        views[i].next = keepNext;
        const Size s = i < 2 ? scaledSize(rec, max, l.settings.peripheralDensity)
                             : scaledSize(rec, max, l.settings.focusDensity, share[0], share[1]);
        views[i].recommendedImageRectWidth = s.w;
        views[i].recommendedImageRectHeight = s.h;
    }
    const Budget b = budget(rec, l.settings, foveated);
    logf("Views: stereo %ux%u -> peripheral %ux%u, focus %ux%u (%s); %.1f%% fewer pixels", rec.w, rec.h,
         views[0].recommendedImageRectWidth, views[0].recommendedImageRectHeight, views[2].recommendedImageRectWidth,
         views[2].recommendedImageRectHeight, foveated ? "follows the gaze" : "fixed", b.saving * 100.0);
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL qvEnumerateEnvironmentBlendModes(XrInstance instance, XrSystemId id,
                                                                XrViewConfigurationType type, uint32_t cap,
                                                                uint32_t* count, XrEnvironmentBlendMode* modes) {
    Layer& l = L();
    return l.next.xrEnumerateEnvironmentBlendModes(instance, id, type == kQuad ? kStereo : type, cap, count, modes);
}

XRAPI_ATTR XrResult XRAPI_CALL qvCreateSession(XrInstance instance, const XrSessionCreateInfo* info,
                                               XrSession* session) {
    Layer& l = L();
    std::lock_guard lock(l.m);
    const XrResult r = l.next.xrCreateSession(instance, info, session);
    if (XR_FAILED(r)) {
        return r;
    }
    l.resetSession();
    l.session = *session;
    l.sessionEnded = false;
    l.lastPeripheral = l.lastFocus = l.lastCanvas = Size{};
    l.lastGpuMs = 0;
    if (l.native) {
        // the runtime composes: no compositor here
    } else if (const auto* vk = findInChain<XrGraphicsBindingVulkanKHR>(info->next, XR_TYPE_GRAPHICS_BINDING_VULKAN_KHR)) {
        l.vulkan = true;
        l.compositor = std::make_unique<Compositor>();
        if (!l.compositor->init(vk->instance, vk->physicalDevice, vk->device, vk->queueFamilyIndex, vk->queueIndex)) {
            logf("Session: the Vulkan compositor could not start; quad views are off for this session");
            l.compositor.reset();
        }
    } else {
        logf("Session: not Vulkan; quad views are off for this session");
    }
    XrReferenceSpaceCreateInfo vs{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    vs.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    vs.poseInReferenceSpace.orientation.w = 1.f;
    l.next.xrCreateReferenceSpace(*session, &vs, &l.viewSpace);
    l.ensureGazeAction();
    if (l.gazeAction) {
        XrActionSpaceCreateInfo as{XR_TYPE_ACTION_SPACE_CREATE_INFO};
        as.action = l.gazeAction;
        as.poseInActionSpace.orientation.w = 1.f;
        l.next.xrCreateActionSpace(*session, &as, &l.gazeSpace);
    }
    l.writeStatus(true);
    return r;
}

XRAPI_ATTR XrResult XRAPI_CALL qvDestroySession(XrSession session) {
    Layer& l = L();
    std::lock_guard lock(l.m);
    if (session == l.session) {
        l.resetSession();
        l.sessionEnded = true;
        l.writeStatus(true);
    }
    return l.next.xrDestroySession(session);
}

XRAPI_ATTR XrResult XRAPI_CALL qvBeginSession(XrSession session, const XrSessionBeginInfo* info) {
    Layer& l = L();
    std::lock_guard lock(l.m);
    XrSessionBeginInfo bi = *info;
    if (l.native) {
        l.useQuad = session == l.session && info->primaryViewConfigurationType == kQuad;
        logf(l.useQuad ? "Begin session: quad views (the runtime's), the focus placed by the layer"
                       : "Begin session: the game uses stereo; the layer only passes through");
        const XrResult nr = l.next.xrBeginSession(session, info);
        if (XR_FAILED(nr)) {
            l.useQuad = false;
        }
        l.writeStatus(true);
        return nr;
    }
    if (session == l.session && info->primaryViewConfigurationType == kQuad) {
        if (!l.compositor) {
            logf("Begin session: quad views asked for, but this session cannot compose (not Vulkan)");
            return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
        }
        bi.primaryViewConfigurationType = kStereo;
        l.useQuad = true;
        logf("Begin session: quad views");
    } else if (session == l.session) {
        logf("Begin session: the game uses stereo; the layer only passes through");
    }
    const XrResult r = l.next.xrBeginSession(session, &bi);
    if (XR_FAILED(r)) {
        l.useQuad = false;
    }
    l.writeStatus(true);
    return r;
}

XRAPI_ATTR XrResult XRAPI_CALL qvSuggestInteractionProfileBindings(
    XrInstance instance, const XrInteractionProfileSuggestedBinding* s) {
    Layer& l = L();
    std::lock_guard lock(l.m);
    l.ensureGazeAction();
    if (!l.gazeAction || s->interactionProfile != l.gazeProfile) {
        return l.next.xrSuggestInteractionProfileBindings(instance, s);
    }
    // The game suggests its own gaze binding: add ours, else one would replace the other.
    std::vector<XrActionSuggestedBinding> b(s->suggestedBindings, s->suggestedBindings + s->countSuggestedBindings);
    b.push_back({l.gazeAction, l.gazePosePath});
    XrInteractionProfileSuggestedBinding c = *s;
    c.suggestedBindings = b.data();
    c.countSuggestedBindings = uint32_t(b.size());
    const XrResult r = l.next.xrSuggestInteractionProfileBindings(instance, &c);
    if (XR_SUCCEEDED(r)) {
        l.gameSuggestedGaze = true;
    }
    return r;
}

XRAPI_ATTR XrResult XRAPI_CALL qvAttachSessionActionSets(XrSession session, const XrSessionActionSetsAttachInfo* a) {
    Layer& l = L();
    std::lock_guard lock(l.m);
    if (session != l.session || !l.gazeSet) {
        return l.next.xrAttachSessionActionSets(session, a);
    }
    if (!l.gameSuggestedGaze) {
        l.suggestOurGazeOnly();
    }
    std::vector<XrActionSet> sets(a->actionSets, a->actionSets + a->countActionSets);
    sets.push_back(l.gazeSet);
    XrSessionActionSetsAttachInfo c = *a;
    c.actionSets = sets.data();
    c.countActionSets = uint32_t(sets.size());
    const XrResult r = l.next.xrAttachSessionActionSets(session, &c);
    if (XR_SUCCEEDED(r)) {
        l.attached = true;
    } else {
        logf("Attach action sets with the gaze set failed: %s", xrName(r));
    }
    return r;
}

XRAPI_ATTR XrResult XRAPI_CALL qvSyncActions(XrSession session, const XrActionsSyncInfo* s) {
    Layer& l = L();
    std::lock_guard lock(l.m);
    if (session != l.session || !l.gazeSet || !l.attached) {
        return l.next.xrSyncActions(session, s);
    }
    std::vector<XrActiveActionSet> sets(s->activeActionSets, s->activeActionSets + s->countActiveActionSets);
    sets.push_back({l.gazeSet, XR_NULL_PATH});
    XrActionsSyncInfo c = *s;
    c.activeActionSets = sets.data();
    c.countActiveActionSets = uint32_t(sets.size());
    l.gameSyncedThisFrame = true;
    return l.next.xrSyncActions(session, &c);
}

XRAPI_ATTR XrResult XRAPI_CALL qvBeginFrame(XrSession session, const XrFrameBeginInfo* info) {
    Layer& l = L();
    std::lock_guard lock(l.m);
    const XrResult r = l.next.xrBeginFrame(session, info);
    if (XR_SUCCEEDED(r) && session == l.session && l.native) {
        l.frames++; // native mode: xrEndFrame is the runtime's, so the frames are counted here
    }
    if (XR_SUCCEEDED(r) && session == l.session && l.useQuad && l.gazeSet && l.trackerAvailable()) {
        // Games without controllers may never attach or sync actions: give them 100 frames, then do it ourselves.
        if (!l.attached && l.frames > 100) {
            l.suggestOurGazeOnly();
            XrSessionActionSetsAttachInfo a{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
            a.countActionSets = 1;
            a.actionSets = &l.gazeSet;
            const XrResult ar = l.next.xrAttachSessionActionSets(session, &a);
            l.attached = XR_SUCCEEDED(ar);
            logf("Attached the gaze action set ourselves: %s", xrName(ar));
        }
        if (l.attached && !l.gameSyncedThisFrame && l.frames > 100) {
            XrActiveActionSet as{l.gazeSet, XR_NULL_PATH};
            XrActionsSyncInfo si{XR_TYPE_ACTIONS_SYNC_INFO};
            si.countActiveActionSets = 1;
            si.activeActionSets = &as;
            l.next.xrSyncActions(session, &si);
        }
        l.gameSyncedThisFrame = false;
    }
    return r;
}

XRAPI_ATTR XrResult XRAPI_CALL qvLocateViews(XrSession session, const XrViewLocateInfo* info, XrViewState* state,
                                             uint32_t cap, uint32_t* count, XrView* views) {
    Layer& l = L();
    std::lock_guard lock(l.m);
    if (session != l.session || info->viewConfigurationType != kQuad) {
        return l.next.xrLocateViews(session, info, state, cap, count, views);
    }
    if (l.native) {
        // the runtime's four views; the focus views (2, 3) replaced by the fields centred on the gaze
        const XrResult nr = l.next.xrLocateViews(session, info, state, cap, count, views);
        if (XR_FAILED(nr) || cap < 4 || *count < 4) {
            return nr;
        }
        l.reloadSettingsIfChanged();
        bool fov = l.trackerAvailable();
        if (const auto* f = findInChain<XrViewLocateFoveatedRenderingVARJO>(
                info->next, XR_TYPE_VIEW_LOCATE_FOVEATED_RENDERING_VARJO)) {
            fov = fov && f->foveatedRenderingActive;
        }
        if (state->viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) {
            l.computeFocus(info, views, fov);
        }
        return nr;
    }
    if (!l.useQuad) {
        return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
    }
    *count = 4;
    if (cap == 0) {
        return XR_SUCCESS;
    }
    if (cap < 4) {
        return XR_ERROR_SIZE_INSUFFICIENT;
    }
    for (uint32_t i = 0; i < 4; i++) {
        if (views[i].type != XR_TYPE_VIEW) {
            return XR_ERROR_VALIDATION_FAILURE;
        }
    }
    XrViewLocateInfo li = *info;
    li.viewConfigurationType = kStereo;
    uint32_t n = 0;
    const XrResult r = l.next.xrLocateViews(session, &li, state, 2, &n, views);
    if (XR_FAILED(r)) {
        return r;
    }
    *count = 4;
    l.reloadSettingsIfChanged();
    bool foveated = l.trackerAvailable();
    if (const auto* f = findInChain<XrViewLocateFoveatedRenderingVARJO>(info->next,
                                                                       XR_TYPE_VIEW_LOCATE_FOVEATED_RENDERING_VARJO)) {
        foveated = foveated && f->foveatedRenderingActive;
    }
    if (state->viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) {
        l.computeFocus(info, views, foveated);
    } else {
        for (uint32_t e = 0; e < 2; e++) {
            views[2 + e].pose = views[e].pose;
            views[2 + e].fov = views[e].fov;
        }
    }
    return r;
}

XRAPI_ATTR XrResult XRAPI_CALL qvCreateSwapchain(XrSession session, const XrSwapchainCreateInfo* info,
                                                 XrSwapchain* sc) {
    Layer& l = L();
    std::lock_guard lock(l.m);
    XrSwapchainCreateInfo c = *info;
    if (session == l.session && l.vulkan && (c.usageFlags & XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT)) {
        c.usageFlags |= XR_SWAPCHAIN_USAGE_SAMPLED_BIT; // we sample the game's views
    }
    const XrResult r = l.next.xrCreateSwapchain(session, &c, sc);
    if (XR_SUCCEEDED(r) && session == l.session) {
        SwapchainState s;
        s.info = c;
        s.info.next = nullptr;
        l.swapchains[*sc] = std::move(s);
    }
    return r;
}

XRAPI_ATTR XrResult XRAPI_CALL qvDestroySwapchain(XrSwapchain sc) {
    Layer& l = L();
    std::lock_guard lock(l.m);
    if (auto it = l.swapchains.find(sc); it != l.swapchains.end()) {
        if (l.compositor) {
            for (VkImage img : it->second.images) l.compositor->forgetImage(img);
        }
        l.swapchains.erase(it);
    }
    return l.next.xrDestroySwapchain(sc);
}

XRAPI_ATTR XrResult XRAPI_CALL qvAcquireSwapchainImage(XrSwapchain sc, const XrSwapchainImageAcquireInfo* info,
                                                       uint32_t* index) {
    Layer& l = L();
    std::lock_guard lock(l.m);
    auto it = l.swapchains.find(sc);
    if (it != l.swapchains.end() && it->second.releasePending) {
        // The game acquires again before ending the frame: release the image we held first.
        XrSwapchainImageReleaseInfo ri{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
        l.next.xrReleaseSwapchainImage(sc, &ri);
        it->second.releasePending = false;
    }
    const XrResult r = l.next.xrAcquireSwapchainImage(sc, info, index);
    if (XR_SUCCEEDED(r) && it != l.swapchains.end()) {
        it->second.acquired.push_back(*index);
    }
    return r;
}

XRAPI_ATTR XrResult XRAPI_CALL qvReleaseSwapchainImage(XrSwapchain sc, const XrSwapchainImageReleaseInfo* info) {
    Layer& l = L();
    std::lock_guard lock(l.m);
    auto it = l.swapchains.find(sc);
    if (it == l.swapchains.end()) {
        return l.next.xrReleaseSwapchainImage(sc, info);
    }
    SwapchainState& s = it->second;
    if (s.acquired.empty()) {
        return XR_ERROR_CALL_ORDER_INVALID;
    }
    if (l.useQuad) {
        // Hold the image until xrEndFrame has read it: after a real release the runtime owns it.
        if (s.releasePending) {
            l.next.xrReleaseSwapchainImage(sc, info);
        }
        s.releasePending = true;
        s.lastReleased = s.acquired.front();
        s.acquired.pop_front();
        return XR_SUCCESS;
    }
    const XrResult r = l.next.xrReleaseSwapchainImage(sc, info);
    if (XR_SUCCEEDED(r)) {
        s.lastReleased = s.acquired.front();
        s.acquired.pop_front();
    }
    return r;
}

XRAPI_ATTR XrResult XRAPI_CALL qvEndFrame(XrSession session, const XrFrameEndInfo* info) {
    Layer& l = L();
    std::unique_lock lock(l.m);
    if (session != l.session || !l.useQuad) {
        lock.unlock();
        return l.next.xrEndFrame(session, info);
    }
    l.frames++;
    std::vector<XrCompositionLayerProjection> projs;
    std::vector<std::array<XrCompositionLayerProjectionView, 2>> projViews;
    std::vector<const XrCompositionLayerBaseHeader*> layers;
    projs.reserve(info->layerCount);
    projViews.reserve(info->layerCount);
    for (uint32_t i = 0; i < info->layerCount; i++) {
        const XrCompositionLayerBaseHeader* h = info->layers[i];
        if (!h) {
            l.releasePending();
            return XR_ERROR_LAYER_INVALID;
        }
        if (h->type != XR_TYPE_COMPOSITION_LAYER_PROJECTION) {
            layers.push_back(h);
            continue;
        }
        const auto* proj = reinterpret_cast<const XrCompositionLayerProjection*>(h);
        if (proj->viewCount != 4) {
            layers.push_back(h);
            continue;
        }
        projViews.emplace_back();
        if (!l.composeLayer(proj, info->displayTime, projViews.back())) {
            if (!l.loggedComposeFailure) {
                logf("Compose failed; submitting the wide views alone");
                l.loggedComposeFailure = true;
            }
            projViews.back() = {proj->views[0], proj->views[1]};
        }
        projs.push_back(*proj);
        projs.back().viewCount = 2;
        projs.back().views = projViews.back().data();
        layers.push_back(reinterpret_cast<const XrCompositionLayerBaseHeader*>(&projs.back()));
    }
    // Our GPU work is queued: now the runtime may have the game's images.
    l.releasePending();
    while (!l.focusFovs.empty() && l.focusFovs.begin()->first < info->displayTime - 1'000'000'000) {
        l.focusFovs.erase(l.focusFovs.begin());
    }
    l.writeStatus();
    XrFrameEndInfo c = *info;
    c.layers = layers.data();
    c.layerCount = uint32_t(layers.size());
    lock.unlock(); // the runtime may block here; locating views on another thread must not wait for it
    return l.next.xrEndFrame(session, &c);
}

XRAPI_ATTR XrResult XRAPI_CALL qvGetVisibilityMaskKHR(XrSession session, XrViewConfigurationType type,
                                                      uint32_t viewIndex, XrVisibilityMaskTypeKHR maskType,
                                                      XrVisibilityMaskKHR* mask) {
    Layer& l = L();
    if (!l.next.xrGetVisibilityMaskKHR) {
        return XR_ERROR_FUNCTION_UNSUPPORTED;
    }
    if (type != kQuad) {
        return l.next.xrGetVisibilityMaskKHR(session, type, viewIndex, maskType, mask);
    }
    if (viewIndex >= 4) {
        return XR_ERROR_VALIDATION_FAILURE;
    }
    if (viewIndex >= 2) {
        // The focus views are inside the visible area: no mask.
        mask->vertexCountOutput = 0;
        mask->indexCountOutput = 0;
        return XR_SUCCESS;
    }
    return l.next.xrGetVisibilityMaskKHR(session, kStereo, viewIndex, maskType, mask);
}

XRAPI_ATTR XrResult XRAPI_CALL qvPollEvent(XrInstance instance, XrEventDataBuffer* ev) {
    Layer& l = L();
    const XrResult r = l.next.xrPollEvent(instance, ev);
    if (r == XR_SUCCESS && ev->type == XR_TYPE_EVENT_DATA_VISIBILITY_MASK_CHANGED_KHR) {
        auto* e = reinterpret_cast<XrEventDataVisibilityMaskChangedKHR*>(ev);
        if (e->viewConfigurationType == kStereo && l.useQuad) {
            e->viewConfigurationType = kQuad;
        }
    }
    return r;
}

XRAPI_ATTR XrResult XRAPI_CALL qvCreateReferenceSpace(XrSession session, const XrReferenceSpaceCreateInfo* info,
                                                      XrSpace* space) {
    Layer& l = L();
    std::lock_guard lock(l.m);
    if (info->referenceSpaceType != XR_REFERENCE_SPACE_TYPE_COMBINED_EYE_VARJO || !l.foveatedRequested) {
        return l.next.xrCreateReferenceSpace(session, info, space);
    }
    // XR_VARJO_foveated_rendering's gaze space: a stand-in the layer answers itself (tracked = gaze available).
    XrReferenceSpaceCreateInfo c = *info;
    c.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    const XrResult r = l.next.xrCreateReferenceSpace(session, &c, space);
    if (XR_SUCCEEDED(r)) {
        l.combinedEyeSpaces.insert(*space);
    }
    return r;
}

XRAPI_ATTR XrResult XRAPI_CALL qvDestroySpace(XrSpace space) {
    Layer& l = L();
    {
        std::lock_guard lock(l.m);
        l.combinedEyeSpaces.erase(space);
    }
    return l.next.xrDestroySpace(space);
}

XRAPI_ATTR XrResult XRAPI_CALL qvLocateSpace(XrSpace space, XrSpace base, XrTime time, XrSpaceLocation* loc) {
    Layer& l = L();
    {
        std::lock_guard lock(l.m);
        if (l.combinedEyeSpaces.count(space)) {
            loc->pose = {{0.f, 0.f, 0.f, 1.f}, {0.f, 0.f, 0.f}};
            loc->locationFlags = l.gazeLive ? (XR_SPACE_LOCATION_ORIENTATION_VALID_BIT |
                                               XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT)
                                            : 0;
            return XR_SUCCESS;
        }
    }
    return l.next.xrLocateSpace(space, base, time, loc);
}

struct Hook {
    const char* name;
    PFN_xrVoidFunction fn;
};

#define QV_HOOK(name, fn) Hook{#name, reinterpret_cast<PFN_xrVoidFunction>(fn)}
const Hook kHooks[] = {
    QV_HOOK(xrDestroyInstance, qvDestroyInstance),
    QV_HOOK(xrGetSystem, qvGetSystem),
    QV_HOOK(xrGetSystemProperties, qvGetSystemProperties),
    QV_HOOK(xrEnumerateViewConfigurations, qvEnumerateViewConfigurations),
    QV_HOOK(xrGetViewConfigurationProperties, qvGetViewConfigurationProperties),
    QV_HOOK(xrEnumerateViewConfigurationViews, qvEnumerateViewConfigurationViews),
    QV_HOOK(xrEnumerateEnvironmentBlendModes, qvEnumerateEnvironmentBlendModes),
    QV_HOOK(xrCreateSession, qvCreateSession),
    QV_HOOK(xrDestroySession, qvDestroySession),
    QV_HOOK(xrBeginSession, qvBeginSession),
    QV_HOOK(xrSuggestInteractionProfileBindings, qvSuggestInteractionProfileBindings),
    QV_HOOK(xrAttachSessionActionSets, qvAttachSessionActionSets),
    QV_HOOK(xrSyncActions, qvSyncActions),
    QV_HOOK(xrBeginFrame, qvBeginFrame),
    QV_HOOK(xrEndFrame, qvEndFrame),
    QV_HOOK(xrLocateViews, qvLocateViews),
    QV_HOOK(xrCreateSwapchain, qvCreateSwapchain),
    QV_HOOK(xrDestroySwapchain, qvDestroySwapchain),
    QV_HOOK(xrAcquireSwapchainImage, qvAcquireSwapchainImage),
    QV_HOOK(xrReleaseSwapchainImage, qvReleaseSwapchainImage),
    QV_HOOK(xrGetVisibilityMaskKHR, qvGetVisibilityMaskKHR),
    QV_HOOK(xrPollEvent, qvPollEvent),
    QV_HOOK(xrCreateReferenceSpace, qvCreateReferenceSpace),
    QV_HOOK(xrDestroySpace, qvDestroySpace),
    QV_HOOK(xrLocateSpace, qvLocateSpace),
};
#undef QV_HOOK

XRAPI_ATTR XrResult XRAPI_CALL qvGetInstanceProcAddr(XrInstance instance, const char* name,
                                                     PFN_xrVoidFunction* fn) {
    Layer& l = L();
    if (!l.next.getInstanceProcAddr) {
        *fn = nullptr;
        return XR_ERROR_HANDLE_INVALID;
    }
    const XrResult r = l.next.getInstanceProcAddr(instance, name, fn);
    if (l.bypass || XR_FAILED(r) || !*fn) {
        return r; // also: no hook for a function the next layer does not have (an extension that is off)
    }
    if (l.native) {
        // native mode: only what reads the gaze and places the focus views; frames, swapchains, view configurations
        // and masks go straight to the runtime
        static const char* const kNative[] = {
            "xrDestroyInstance", "xrGetSystem", "xrGetSystemProperties", "xrEnumerateViewConfigurationViews",
            "xrCreateSession", "xrDestroySession", "xrBeginSession", "xrSuggestInteractionProfileBindings",
            "xrAttachSessionActionSets", "xrSyncActions", "xrBeginFrame", "xrLocateViews"};
        if (std::none_of(std::begin(kNative), std::end(kNative), [&](const char* n) { return std::strcmp(n, name) == 0; })) {
            return r;
        }
    }
    for (const Hook& h : kHooks) {
        if (std::strcmp(h.name, name) == 0) {
            *fn = h.fn;
            break;
        }
    }
    return r;
}

std::vector<XrExtensionProperties> runtimeExtensions(PFN_xrGetInstanceProcAddr gipa) {
    PFN_xrEnumerateInstanceExtensionProperties e = nullptr;
    if (XR_FAILED(gipa(XR_NULL_HANDLE, "xrEnumerateInstanceExtensionProperties",
                       reinterpret_cast<PFN_xrVoidFunction*>(&e))) ||
        !e) {
        return {};
    }
    uint32_t n = 0;
    if (XR_FAILED(e(nullptr, 0, &n, nullptr))) {
        return {};
    }
    std::vector<XrExtensionProperties> v(n, {XR_TYPE_EXTENSION_PROPERTIES});
    if (XR_FAILED(e(nullptr, n, &n, v.data()))) {
        return {};
    }
    v.resize(n);
    return v;
}

XRAPI_ATTR XrResult XRAPI_CALL qvCreateApiLayerInstance(const XrInstanceCreateInfo* info,
                                                        const XrApiLayerCreateInfo* layerInfo, XrInstance* instance) {
    if (!info || !layerInfo || !layerInfo->nextInfo || std::strcmp(layerInfo->nextInfo->layerName, kLayerName) != 0) {
        return XR_ERROR_INITIALIZATION_FAILED;
    }
    Layer& l = L();
    std::lock_guard lock(l.m);
    logOpen();
    l.next = Next{};
    l.next.getInstanceProcAddr = layerInfo->nextInfo->nextGetInstanceProcAddr;
    XrApiLayerCreateInfo nextLayerInfo = *layerInfo;
    nextLayerInfo.nextInfo = layerInfo->nextInfo->next;
    const auto createNext = layerInfo->nextInfo->nextCreateApiLayerInstance;

    bool quad = false, gameEyeGaze = false, vulkanApi = false;
    std::vector<const char*> exts;
    for (uint32_t i = 0; i < info->enabledExtensionCount; i++) {
        const std::string e = info->enabledExtensionNames[i];
        if (e == XR_VARJO_QUAD_VIEWS_EXTENSION_NAME) {
            quad = true;
            continue;
        }
        if (e == XR_VARJO_FOVEATED_RENDERING_EXTENSION_NAME) {
            l.foveatedRequested = true;
            continue;
        }
        gameEyeGaze = gameEyeGaze || e == XR_EXT_EYE_GAZE_INTERACTION_EXTENSION_NAME;
        vulkanApi = vulkanApi || e == XR_KHR_VULKAN_ENABLE_EXTENSION_NAME || e == XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME;
        exts.push_back(info->enabledExtensionNames[i]);
    }
    l.app = AppId::current(info->applicationInfo.applicationName);
    logf("qv-vulkan %s: app \"%s\" (%s), quad views %s, foveated %s, Vulkan %s", QV_VERSION, l.app.name.c_str(),
         l.app.exe.c_str(), quad ? "asked" : "not asked", l.foveatedRequested ? "asked" : "not asked",
         vulkanApi ? "yes" : "no");

    const auto rt = runtimeExtensions(l.next.getInstanceProcAddr);
    const auto has = [&](const char* name) {
        return std::any_of(rt.begin(), rt.end(), [&](const auto& p) { return std::strcmp(p.extensionName, name) == 0; });
    };
    if (quad && has(XR_VARJO_QUAD_VIEWS_EXTENSION_NAME) && !has(XR_VARJO_FOVEATED_RENDERING_EXTENSION_NAME)) {
        // Native mode (Monado): quad views stay the runtime's; the gaze is ours to read.
        exts.push_back(XR_VARJO_QUAD_VIEWS_EXTENSION_NAME);
        const bool addGaze = !gameEyeGaze && has(XR_EXT_EYE_GAZE_INTERACTION_EXTENSION_NAME);
        if (addGaze) {
            exts.push_back(XR_EXT_EYE_GAZE_INTERACTION_EXTENSION_NAME);
        }
        XrInstanceCreateInfo c = *info;
        c.enabledExtensionNames = exts.data();
        c.enabledExtensionCount = uint32_t(exts.size());
        XrResult r = createNext(&c, &nextLayerInfo, instance);
        if (XR_FAILED(r) && addGaze) {
            exts.pop_back();
            c.enabledExtensionCount = uint32_t(exts.size());
            r = createNext(&c, &nextLayerInfo, instance);
        } else {
            l.eyeGazeEnabled = gameEyeGaze || addGaze;
        }
        if (XR_FAILED(r)) {
            logf("xrCreateInstance failed below the layer (native quad views): %s", xrName(r));
            return r;
        }
        l.instance = *instance;
        l.bypass = false;
        l.native = true;
        l.next.load(*instance);
        if (l.next.xrGetInstanceProperties) {
            XrInstanceProperties ip{XR_TYPE_INSTANCE_PROPERTIES};
            if (XR_SUCCEEDED(l.next.xrGetInstanceProperties(*instance, &ip))) {
                l.runtimeName = ip.runtimeName;
            }
        }
        l.settingsFile = settingsPath();
        l.reloadSettingsIfChanged(true);
        logf("Runtime: %s has quad views itself: native mode (the focus follows the gaze); eye gaze extension %s",
             l.runtimeName.c_str(), l.eyeGazeEnabled ? "on" : "off");
        return r;
    }
    if (!quad || has(XR_VARJO_QUAD_VIEWS_EXTENSION_NAME)) {
        // Nothing to do: the game does not use quad views, or the runtime has them itself (and foveation too).
        logf(quad ? "The runtime has quad views itself: passing through" : "Passing through");
        l.bypass = true;
        l.foveatedRequested = false;
        const XrResult r = createNext(info, &nextLayerInfo, instance);
        l.instance = XR_SUCCEEDED(r) ? *instance : XR_NULL_HANDLE;
        return r;
    }
    const bool addEyeGaze = !gameEyeGaze && has(XR_EXT_EYE_GAZE_INTERACTION_EXTENSION_NAME);
    if (addEyeGaze) {
        exts.push_back(XR_EXT_EYE_GAZE_INTERACTION_EXTENSION_NAME);
    }
    XrInstanceCreateInfo c = *info;
    c.enabledExtensionNames = exts.data();
    c.enabledExtensionCount = uint32_t(exts.size());
    XrResult r = createNext(&c, &nextLayerInfo, instance);
    if (XR_FAILED(r) && addEyeGaze) {
        exts.pop_back();
        c.enabledExtensionCount = uint32_t(exts.size());
        r = createNext(&c, &nextLayerInfo, instance);
    } else {
        l.eyeGazeEnabled = gameEyeGaze || addEyeGaze;
    }
    if (XR_FAILED(r)) {
        logf("xrCreateInstance failed below the layer: %s", xrName(r));
        return r;
    }
    l.instance = *instance;
    l.bypass = false;
    l.next.load(*instance);
    if (l.next.xrGetInstanceProperties) {
        XrInstanceProperties ip{XR_TYPE_INSTANCE_PROPERTIES};
        if (XR_SUCCEEDED(l.next.xrGetInstanceProperties(*instance, &ip))) {
            l.runtimeName = ip.runtimeName;
        }
    }
    l.settingsFile = settingsPath();
    l.reloadSettingsIfChanged(true);
    logf("Runtime: %s; eye gaze extension %s", l.runtimeName.c_str(), l.eyeGazeEnabled ? "on" : "off");
    if (!vulkanApi) {
        logf("The game does not use Vulkan: quad views will be refused at xrBeginSession");
    }
    return r;
}

} // namespace
} // namespace qv

QV_EXPORT XrResult XRAPI_CALL xrNegotiateLoaderApiLayerInterface(const XrNegotiateLoaderInfo* loaderInfo,
                                                                 const char* layerName,
                                                                 XrNegotiateApiLayerRequest* request) {
    if (!loaderInfo || !request || loaderInfo->structType != XR_LOADER_INTERFACE_STRUCT_LOADER_INFO ||
        request->structType != XR_LOADER_INTERFACE_STRUCT_API_LAYER_REQUEST ||
        (layerName && std::strcmp(layerName, qv::kLayerName) != 0)) {
        return XR_ERROR_INITIALIZATION_FAILED;
    }
    if (loaderInfo->minInterfaceVersion > XR_CURRENT_LOADER_API_LAYER_VERSION ||
        loaderInfo->maxInterfaceVersion < XR_CURRENT_LOADER_API_LAYER_VERSION) {
        return XR_ERROR_INITIALIZATION_FAILED;
    }
    request->layerInterfaceVersion = XR_CURRENT_LOADER_API_LAYER_VERSION;
    request->layerApiVersion = XR_MAKE_VERSION(1, 0, 0);
    request->getInstanceProcAddr = qv::qvGetInstanceProcAddr;
    request->createApiLayerInstance = qv::qvCreateApiLayerInstance;
    return XR_SUCCESS;
}
