// SPDX-License-Identifier: MIT
// The OpenXR (XR_KHR_vulkan_enable2) + Vulkan plumbing that pibridge-roomsetup and pibridge-home share: the instance, the
// Vulkan device the runtime asks for, the session with its STAGE space (LOCAL when the runtime has no stage) and
// VIEW space, one swapchain per eye, and two small pipelines that draw libpibridge-room's scene lists (unlit coloured
// triangles and lines, pibridge/room/draw.h) into both eyes. The input bindings, the event handling and what is drawn
// stay in each tool. Internal: a static library, nothing is installed.
//
// Order of use: openSession() -> the tool's action set, its attach and hand spaces -> createSwapchains() -> per frame
// xrWaitFrame / xrBeginFrame -> renderStereo() -> xrEndFrame. Session's destructor frees everything in reverse order.
#pragma once

#include "pibridge/room/draw.h"
#include "pibridge/room/vmath.h"
#include "pibridge/xrkit/passthrough.h"

#include <time.h>
#include <vulkan/vulkan.h>
#define XR_USE_GRAPHICS_API_VULKAN
#define XR_USE_TIMESPEC
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <array>
#include <cstdint>
#include <expected>
#include <format>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace pibridge::xrkit {

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

inline constexpr XrViewConfigurationType kViewType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
inline constexpr XrPosef kIdentity = {{0.0F, 0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 0.0F}};

inline std::string xrErr(const char* what, XrResult r) { return std::format("{}: XrResult {}", what, static_cast<int>(r)); }
inline std::string vkErr(const char* what, VkResult r) { return std::format("{}: VkResult {}", what, static_cast<int>(r)); }

#define PIBRIDGE_XR(call, what)                                                                                       \
    do {                                                                                                           \
        const XrResult pibridge_r = (call);                                                                           \
        if (XR_FAILED(pibridge_r)) {                                                                                  \
            return std::unexpected(::pibridge::xrkit::xrErr(what, pibridge_r));                                          \
        }                                                                                                          \
    } while (0)
#define PIBRIDGE_VK(call, what)                                                                                       \
    do {                                                                                                           \
        const VkResult pibridge_r = (call);                                                                           \
        if (pibridge_r != VK_SUCCESS) {                                                                               \
            return std::unexpected(::pibridge::xrkit::vkErr(what, pibridge_r));                                          \
        }                                                                                                          \
    } while (0)

/// What the vertex shader gets: the eye's view-projection and flags (x: 1 for an sRGB swapchain).
struct Push {
    room::Mat4 viewProj;
    std::array<float, 4> flags;
};

/// The passthrough background (passthrough.h): one R8 640x480 image of 4 layers (one per camera slot) filled from a
/// host-visible staging buffer when a new camera frame came, a sampler, one uniform block per eye and a full-screen
/// pipeline drawn first in the scene's render pass. One copy of up to 1.2 MB per camera frame (25 Hz) and one fragment
/// pass per eye.
struct PassthroughGpu {
    VkDevice device = VK_NULL_HANDLE;
    VkDescriptorSetLayout dsl = VK_NULL_HANDLE;
    VkPipelineLayout layout = VK_NULL_HANDLE;
    VkPipeline pipe = VK_NULL_HANDLE;
    VkDescriptorPool pool = VK_NULL_HANDLE;
    std::array<VkDescriptorSet, 2> sets{VK_NULL_HANDLE, VK_NULL_HANDLE};
    VkImage img = VK_NULL_HANDLE;
    VkDeviceMemory imem = VK_NULL_HANDLE;
    VkImageView view = VK_NULL_HANDLE;
    VkSampler sampler = VK_NULL_HANDLE;
    VkBuffer staging = VK_NULL_HANDLE;
    VkDeviceMemory smem = VK_NULL_HANDLE;
    void* sptr = nullptr;
    VkBuffer ubuf = VK_NULL_HANDLE;
    VkDeviceMemory umem = VK_NULL_HANDLE;
    std::uint8_t* uptr = nullptr;
    VkDeviceSize uStride = 1024;
    std::uint32_t layers = 0; // layers in the staging buffer to copy
    bool dirty = false;     // the staging buffer holds a frame not copied yet
    bool initialised = false; // the image has been written once (its layout is SHADER_READ_ONLY)
    bool haveImage = false;
    std::array<bool, 2> eyeOn{false, false}; // draw it in this eye's next draw

    std::expected<void, std::string> create(VkDevice dev, const VkPhysicalDeviceMemoryProperties& mp, VkRenderPass rp,
                                            std::array<std::uint32_t, 2> size, VkDeviceSize uboAlign);
    bool ready() const { return pipe != VK_NULL_HANDLE; }
    /// Copy a camera frame into the staging buffer (the GPU is idle between draws: every draw waits for its fence).
    void setFrame(const pt::Frame& f);
    void setEye(std::size_t eye, const pt::Uniform& u);
    void recordUpload(VkCommandBuffer cmd);
    void recordDraw(VkCommandBuffer cmd, std::size_t eye);
    void destroy();
};

/// The sky's per-eye push block (sky.frag): the eye's rotation, its fov and the look.
struct SkyPush {
    room::Mat4 stageFromEye;
    std::array<float, 4> tan{};  // left, right, up, down
    std::array<float, 4> misc{}; // 1 = sRGB swapchain, yaw (rad), brightness, 0
};
/// A vertex of the panel mesh (stage metres, texture coordinates 0..1 with v down).
struct TexVertex {
    float x, y, z, u, v;
};

/// A textured pass (texpass.cpp): one sRGB image (RGBA for the sky, BGRA for the panel: the bytes as given) filled
/// from a host-visible staging buffer, a sampler, and either a full-screen equirect pipeline (Sky, push SkyPush) or a
/// mesh pipeline (Mesh, push Push, alpha-blended). The image is made at the first setPixels and made again only when
/// the size changes; the copy goes in with the next draw (the GPU is idle between draws: every draw waits for its
/// fence).
struct TexPass {
    enum class Kind { Sky, Mesh };
    Kind kind = Kind::Sky;
    VkDevice device = VK_NULL_HANDLE;
    VkPhysicalDeviceMemoryProperties memProps{};
    VkFormat format = VK_FORMAT_R8G8B8A8_SRGB;
    VkDescriptorSetLayout dsl = VK_NULL_HANDLE;
    VkPipelineLayout layout = VK_NULL_HANDLE;
    VkPipeline pipe = VK_NULL_HANDLE;
    VkDescriptorPool pool = VK_NULL_HANDLE;
    VkDescriptorSet set = VK_NULL_HANDLE;
    VkSampler sampler = VK_NULL_HANDLE;
    VkImage img = VK_NULL_HANDLE;
    VkDeviceMemory imem = VK_NULL_HANDLE;
    VkImageView view = VK_NULL_HANDLE;
    VkBuffer staging = VK_NULL_HANDLE;
    VkDeviceMemory smem = VK_NULL_HANDLE;
    void* sptr = nullptr;
    VkBuffer vbuf = VK_NULL_HANDLE; // Mesh: the vertices
    VkDeviceMemory vmem = VK_NULL_HANDLE;
    void* vptr = nullptr;
    std::uint32_t vcount = 0;
    static constexpr std::uint32_t kMaxVerts = 6 * 128;
    std::uint32_t w = 0, h = 0;
    bool dirty = false;
    bool initialised = false;
    bool haveImage = false;
    std::array<bool, 2> eyeOn{false, false};
    std::array<SkyPush, 2> sky{};
    std::array<Push, 2> mesh{};

    std::expected<void, std::string> create(VkDevice dev, const VkPhysicalDeviceMemoryProperties& mp, VkRenderPass rp,
                                            std::array<std::uint32_t, 2> size, Kind k, VkFormat fmt);
    bool ready() const { return pipe != VK_NULL_HANDLE; }
    /// Copy `width` x `height` pixels of 4 bytes (rows `stride` bytes apart) for the next draw.
    std::expected<void, std::string> setPixels(const std::uint8_t* src, std::uint32_t width, std::uint32_t height,
                                               std::size_t stride);
    /// Mesh: the triangles (3 vertices each, at most kMaxVerts).
    void setMesh(const std::vector<TexVertex>& v);
    void setSkyEye(std::size_t eye, const SkyPush& p);
    void setMeshEye(std::size_t eye, const Push& p);
    void recordUpload(VkCommandBuffer cmd);
    void recordDraw(VkCommandBuffer cmd, std::size_t eye);
    void destroy();

private:
    void freeImage();
};

/// Two pipelines (triangles, lines) over one host-visible vertex buffer; a framebuffer per swapchain image.
struct SceneGpu {
    VkDevice device = VK_NULL_HANDLE;
    VkQueue queue = VK_NULL_HANDLE;
    VkPhysicalDeviceMemoryProperties memProps{};
    VkCommandPool pool = VK_NULL_HANDLE;
    VkCommandBuffer cmd = VK_NULL_HANDLE;
    VkFence fence = VK_NULL_HANDLE;
    VkRenderPass renderPass = VK_NULL_HANDLE;
    VkPipelineLayout layout = VK_NULL_HANDLE;
    VkPipeline tris = VK_NULL_HANDLE;
    VkPipeline lines = VK_NULL_HANDLE;
    VkBuffer vbuf = VK_NULL_HANDLE;
    VkDeviceMemory vmem = VK_NULL_HANDLE;
    void* vptr = nullptr;
    std::size_t vcap = 0; // vertices
    std::array<std::vector<std::pair<VkImageView, VkFramebuffer>>, 2> targets;
    std::array<std::uint32_t, 2> size{};
    std::array<float, 4> background{0.012F, 0.016F, 0.016F, 1.0F}; // the clear colour, as given to Vulkan
    PassthroughGpu pt; // made by enablePassthrough(); drawn first when on for the eye
    TexPass sky;       // made by enableSky(); drawn first (instead of the passthrough) when on for the eye
    TexPass panel;     // made by enablePanel(); drawn after the scene, before the overlay lists
    std::uint32_t overlayTris = 0, overlayLines = 0; // the overlay's vertices, after the scene's in the buffer

    std::expected<void, std::string> create(VkDevice dev, VkQueue q, std::uint32_t qf, const VkPhysicalDeviceMemoryProperties& mp,
                                            VkFormat format, std::array<std::uint32_t, 2> sz,
                                            const std::array<std::vector<VkImage>, 2>& images);
    /// Room for `n` vertices (the GPU is idle between draws: every draw waits for its fence).
    std::expected<void, std::string> reserve(std::size_t n);
    std::expected<void, std::string> upload(const room::Batch& b, const room::Batch* overlay = nullptr);
    std::expected<void, std::string> draw(std::size_t eye, std::size_t idx, const Push& push, std::uint32_t nTris,
                                          std::uint32_t nLines);
    void destroy();

private:
    std::expected<VkPipeline, std::string> pipeline(VkPrimitiveTopology topo, VkShaderModule vs, VkShaderModule fs);
};

struct Hand {
    XrPath path = XR_NULL_PATH;
    XrSpace grip = XR_NULL_HANDLE;
    XrSpace aim = XR_NULL_HANDLE;
};

/// Every handle of the session, destroyed in reverse order.
struct Session {
    XrInstance instance = XR_NULL_HANDLE;
    XrSystemId system = XR_NULL_SYSTEM_ID;
    XrSession session = XR_NULL_HANDLE;
    XrSpace stage = XR_NULL_HANDLE;
    XrReferenceSpaceType stageType = XR_REFERENCE_SPACE_TYPE_STAGE;
    XrSpace view = XR_NULL_HANDLE;
    VkInstance vkInstance = VK_NULL_HANDLE;
    VkPhysicalDevice pd = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    VkQueue queue = VK_NULL_HANDLE;
    std::uint32_t qf = 0;
    VkPhysicalDeviceMemoryProperties memProps{};
    std::array<XrSwapchain, 2> eyeSc{XR_NULL_HANDLE, XR_NULL_HANDLE};
    std::array<std::vector<VkImage>, 2> eyeImages;
    std::int64_t format = 0;
    std::array<std::uint32_t, 2> size{};
    SceneGpu gpu;
    XrActionSet set = XR_NULL_HANDLE; // the tool's (its actions go with the instance)
    std::array<Hand, 2> hands;        // the tool fills them (left, right)
    std::string runtimeName;
    bool timespec = false;            // XR_KHR_convert_timespec_time is enabled

    Session() = default;
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
    ~Session();

    template <class F>
    std::expected<F, std::string> proc(const char* name) const {
        PFN_xrVoidFunction f = nullptr;
        const XrResult r = xrGetInstanceProcAddr(instance, name, &f);
        if (XR_FAILED(r) || f == nullptr) {
            return std::unexpected(xrErr(name, r));
        }
        return reinterpret_cast<F>(f);
    }
    XrPath path(const std::string& s) const;
    /// The swapchain format is sRGB (the shader then linearises the sRGB palette).
    bool srgb() const;
};

struct OpenOpts {
    std::string appName = "pibridge";
    std::string runtimeHint = "is SteamVR running?"; // added to the instance errors
    unsigned resDiv = 1;                             // divide the recommended eye size
    std::uint32_t maxEye = 3200;                     // cap of the longer side (lines and pixel text need no more)
};

/// The instance (XR_KHR_vulkan_enable2), the Vulkan instance and device through the runtime, the session, the STAGE
/// space (LOCAL when the runtime has none; `stageType` says which) and the VIEW space.
std::expected<void, std::string> openSession(Session& x, const OpenOpts& opts);
/// The eye size (recommended / resDiv, capped), an 8-bit RGBA/BGRA format (sRGB first), one swapchain per eye and the
/// pipelines. Prints the size, the runtime and the space.
std::expected<void, std::string> createSwapchains(Session& x, const OpenOpts& opts);

/// The passthrough pipeline (after createSwapchains). Frames go in with x.gpu.pt.setFrame; renderStereo draws it in
/// each eye for which its `passthrough` hook returns a uniform block.
std::expected<void, std::string> enablePassthrough(Session& x);
/// The equirect background pass (after createSwapchains); pictures go in with x.gpu.sky.setPixels (RGBA, sRGB).
std::expected<void, std::string> enableSky(Session& x);
/// The textured panel pass (after createSwapchains); pictures go in with x.gpu.panel.setPixels (BGRA, sRGB), the
/// mesh with x.gpu.panel.setMesh.
std::expected<void, std::string> enablePanel(Session& x);

/// CLOCK_MONOTONIC ns -> XrTime through XR_KHR_convert_timespec_time (openSession enables it when the runtime offers
/// it); nullopt without it.
std::optional<XrTime> xrTimeOfMonotonic(const Session& x, std::uint64_t ns);
/// XrTime -> CLOCK_MONOTONIC ns, the other way.
std::optional<std::uint64_t> monotonicOfXrTime(const Session& x, XrTime t);

/// The first of the formats pibridge's renderers take (R8G8B8A8/B8G8R8A8, sRGB first).
std::optional<std::int64_t> chooseFormat(const std::vector<std::int64_t>& offered);

std::expected<XrAction, std::string> makeAction(XrActionSet set, const char* name, const char* label, XrActionType type,
                                                std::span<const XrPath> sub);
room::Mat34 toMat(const XrPosef& p);
/// The pose of `space` in `base` at `t`, when position and orientation are both valid.
bool locate(XrSpace space, XrSpace base, XrTime t, room::Mat34& out);
/// The same; `tracked`: the position is tracked now (XR_SPACE_LOCATION_POSITION_TRACKED_BIT), not a held one.
bool locate(XrSpace space, XrSpace base, XrTime t, room::Mat34& out, bool& tracked);

/// A projection layer over both eyes, ready for xrEndFrame (the views point into it: keep it alive until then).
struct ProjectionLayer {
    XrCompositionLayerProjection proj = xr_make<XrCompositionLayerProjection>(XR_TYPE_COMPOSITION_LAYER_PROJECTION);
    std::array<XrCompositionLayerProjectionView, 2> views{};
    XrCompositionLayerBaseHeader* header() { return reinterpret_cast<XrCompositionLayerBaseHeader*>(&proj); }
};

/// Locate both eyes at `t` in the stage space, upload `batch` and draw it into each eye's next swapchain image
/// (near/far in metres). True with `out` filled when the views were valid and drawn; false when there is nothing to
/// show this frame (the views were not valid).
/// `passthrough` (optional): per eye (0 left, 1 right) and its view at `t`, the passthrough block to draw first, or
/// nullopt for none.
using PassthroughHook = std::function<std::optional<pt::Uniform>(std::size_t eye, const pt::EyeView& view)>;
/// More for a frame (pibridge-home): `sky` per eye (the sky pass is drawn instead of the passthrough when it returns a
/// block), `panel` (draw the panel mesh with the eye's view-projection, opacity `panelOpacity`), `overlay` (lists
/// drawn last, over the panel: the pointer).
struct Extras {
    std::function<std::optional<SkyPush>(std::size_t eye, const pt::EyeView& view)> sky;
    bool panel = false;
    float panelOpacity = 1.0F;
    const room::Batch* overlay = nullptr;
};
std::expected<bool, std::string> renderStereo(Session& x, const room::Batch& batch, XrTime t, double nearZ, double farZ,
                                              ProjectionLayer& out, const PassthroughHook& passthrough = {},
                                              const Extras* extras = nullptr);

} // namespace pibridge::xrkit
