// SPDX-License-Identifier: MIT
// pibridge-record-layer: a Vulkan implicit layer for PiBridge's video recording ([memory] pibridge-video-capture-plan,
// docs/record.md). SteamVR's GL mirror texture (GetMirrorTextureGL) crashes vrserver on NVIDIA/Vulkan, and a Linux
// OpenXR API layer never loads into Proton/Windows games. A Vulkan layer DOES: Proton games render through the host
// Linux Vulkan loader (DXVK/VKD3D), so this layer sees their work too, on any desktop (X11 or Wayland), without the
// compositor/portal.
//
// The eye images a VR app hands to SteamVR's compositor are Vulkan colour images created with EXTERNAL MEMORY
// (VkExternalMemoryImageCreateInfo, shared by opaque-fd / dma_buf). That is the reliable, Proton-proof signal for "a
// frame bound for the HMD" — not a guess. With quad views the final per-eye image is the quad-views layer's composed
// output, which is what gets shared, so recording captures the foveated image actually shown.
//
//   M1-vk: a LOGGER of every external colour image (its size/layers/format), to confirm which images are the eyes.
//   M2  : in the compositor, SELECT the capture target (the game's eye buffer = the largest arrayLayers>=2 colour
//         image) and which eye(s) from the control file's `view` (left | right | both).
//   M2b (this file): CAPTURE. In the device that holds the target, create ONCE an exportable (OPAQUE_FD) VkImage
//         scaled to the control file's max_height, and per frame (throttled to fps) blit the selected view/layer(s)
//         into it on a SEPARATE command pool + fence (never vkQueueWaitIdle on the compositor's queue). A unix-socket
//         server at $XDG_RUNTIME_DIR/pibridge/record-capture.sock hands the exported fd (SCM_RIGHTS) + metadata to the
//         recorder (`pibridge-record run --source layer`, the client/encoder) and signals each captured frame. If any
//         Vulkan step fails the capture disables itself and the VR session runs on untouched (a leaf: zero missed
//         frames). Every step is logged to record-layer.log so a live run shows exactly what happened.
//
// The blit's source-image layout and its cross-queue / cross-process synchronisation are the parts that need
// on-hardware iteration; the best guess and its fallback are noted at each barrier.
#include <vulkan/vulkan.h>
#include <vulkan/vk_layer.h>

#include "pibridge_record_capture_control.h"
#include "pibridge_record_capture_ipc.h"

#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <system_error>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#define PRL_EXPORT extern "C" __attribute__((visibility("default")))

namespace pibridge::reclayer {
namespace {

// ---- logging to $XDG_RUNTIME_DIR/pibridge/record-layer.log (best effort; a layer never fails an app over a log) ----
std::mutex g_log_mutex;
FILE* g_log = nullptr;

void log_open() {
    std::lock_guard lock(g_log_mutex);
    if (g_log != nullptr) {
        return;
    }
    const char* rt = std::getenv("XDG_RUNTIME_DIR");
    std::string dir = (rt != nullptr && rt[0] != '\0') ? std::string(rt) + "/pibridge" : std::string("/tmp");
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    const std::string path = dir + "/record-layer.log";
    g_log = std::fopen(path.c_str(), "ae"); // append: several processes may load the layer
}

void logf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void logf(const char* fmt, ...) {
    std::lock_guard lock(g_log_mutex);
    if (g_log == nullptr) {
        return;
    }
    std::va_list ap;
    va_start(ap, fmt);
    std::vfprintf(g_log, fmt, ap);
    va_end(ap);
    std::fputc('\n', g_log);
    std::fflush(g_log);
}

std::uint64_t now_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ull + static_cast<std::uint64_t>(ts.tv_nsec);
}

// The short name of the process this layer is loaded into (/proc/self/comm), to tell the game from the SteamVR
// compositor / cef / vrwebhelper in the log.
const char* proc_name() {
    static const std::string name = [] {
        std::string n;
        if (FILE* f = std::fopen("/proc/self/comm", "re")) {
            char buf[256];
            if (std::fgets(buf, sizeof(buf), f) != nullptr) {
                n = buf;
                if (!n.empty() && n.back() == '\n') {
                    n.pop_back();
                }
            }
            std::fclose(f);
        }
        return n.empty() ? std::string("?") : n;
    }();
    return name.c_str();
}

// Is this process the SteamVR compositor? (That is where the eye buffer and the composited output live — see
// [memory] pibridge-video-capture-plan.) Capture only happens here.
bool in_compositor() {
    static const bool yes = std::strcmp(proc_name(), "vrcompositor") == 0;
    return yes;
}

// ---- the recording control: $XDG_RUNTIME_DIR/pibridge/record-capture (the app/CLI writes it; we poll it) ----------
// Lines: "active 1" (0 = off), "view left|right|both", "fps N", "max_height N". While off, pass-through.
enum class CaptureView { LeftEye, RightEye, BothEyes };
struct Control {
    bool active = false;
    CaptureView view = CaptureView::LeftEye;
    std::uint32_t fps = 60;
    std::uint32_t max_height = 1080;
};

Control read_control() {
    Control c;
    std::ifstream f(captureControlPath());
    if (!f) {
        return c;
    }
    std::string key;
    std::string val;
    while (f >> key >> val) {
        if (key == "active") {
            c.active = (val == "1" || val == "on" || val == "true");
        } else if (key == "view") {
            if (val == "right") {
                c.view = CaptureView::RightEye;
            } else if (val == "both") {
                c.view = CaptureView::BothEyes;
            } else {
                c.view = CaptureView::LeftEye;
            }
        } else if (key == "fps") {
            const unsigned long v = std::strtoul(val.c_str(), nullptr, 10);
            if (v >= 1 && v <= 240) {
                c.fps = static_cast<std::uint32_t>(v);
            }
        } else if (key == "max_height") {
            const unsigned long v = std::strtoul(val.c_str(), nullptr, 10);
            if (v >= 64 && v <= 4320) {
                c.max_height = static_cast<std::uint32_t>(v);
            }
        }
    }
    return c;
}

// ---- the capture server: one AF_UNIX socket for this process; hands out the fd + metadata, signals each frame ------
// Non-blocking: a stuck client can only make the capture skip frames, never block the compositor. Flow control is one
// bit per client ("ready for the next frame"), flipped off when we send FrameMsg and back on by the client's ack byte.
class CaptureServer {
public:
    void start() {
        std::lock_guard lock(mtx_);
        if (listen_fd_ >= 0 || started_) {
            return;
        }
        started_ = true;
        const std::string path = captureSocketPath();
        std::error_code ec;
        std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
        ::unlink(path.c_str());
        const int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd < 0) {
            logf("[vrcompositor] capture server: socket() failed: %s", std::strerror(errno));
            return;
        }
        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
        if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 || ::listen(fd, 4) != 0) {
            logf("[vrcompositor] capture server: bind/listen %s failed: %s", path.c_str(), std::strerror(errno));
            ::close(fd);
            return;
        }
        listen_fd_ = fd;
        quit_.store(false);
        thread_ = std::thread([this] { loop(); });
        logf("[vrcompositor] capture server listening at %s", path.c_str());
    }

    ~CaptureServer() { shutdown(); }

    void shutdown() {
        quit_.store(true);
        if (thread_.joinable()) {
            thread_.join();
        }
        std::lock_guard lock(mtx_);
        for (auto& c : clients_) {
            ::close(c.fd);
        }
        clients_.clear();
        if (listen_fd_ >= 0) {
            ::close(listen_fd_);
            listen_fd_ = -1;
        }
        if (image_fd_ >= 0) {
            ::close(image_fd_);
            image_fd_ = -1;
        }
    }

    // The exportable image exists: remember its fd + metadata and push HELLO to every connected client. The server
    // keeps its own dup of the fd; the caller still owns the one it passed.
    void setImage(const HelloMsg& h, int fd) {
        std::lock_guard lock(mtx_);
        if (image_fd_ >= 0) {
            ::close(image_fd_);
        }
        image_fd_ = ::dup(fd);
        hello_ = h;
        have_image_ = true;
        for (auto& c : clients_) {
            sendHello(c);
        }
        logf("[vrcompositor] capture server: image ready %ux%u, %zu client(s) told", h.width, h.height, clients_.size());
    }

    bool hasClient() {
        std::lock_guard lock(mtx_);
        return !clients_.empty();
    }

    // Every connected client has acknowledged the previous frame (so the one shared image may be reused).
    bool allReady() {
        std::lock_guard lock(mtx_);
        if (clients_.empty()) {
            return false;
        }
        for (const auto& c : clients_) {
            if (!c.ready) {
                return false;
            }
        }
        return true;
    }

    // Broadcast "the shared image now holds frame `seq`". Marks each client not-ready until it acks.
    void postFrame(std::uint64_t seq, std::uint64_t ts) {
        FrameMsg m;
        m.seq = seq;
        m.timestamp_ns = ts;
        std::lock_guard lock(mtx_);
        for (auto it = clients_.begin(); it != clients_.end();) {
            const ssize_t n = ::send(it->fd, &m, sizeof(m), MSG_DONTWAIT | MSG_NOSIGNAL);
            if (n == static_cast<ssize_t>(sizeof(m))) {
                it->ready = false;
                ++it;
            } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                ++it; // client is behind; leave it (it will catch up or be dropped on the next error)
            } else {
                ::close(it->fd);
                it = clients_.erase(it);
            }
        }
    }

private:
    struct Client {
        int fd = -1;
        bool ready = true; // ready to receive the next frame (no outstanding unacked frame)
    };

    void sendHello(Client& c) {
        // sendmsg with the fd as SCM_RIGHTS ancillary data.
        iovec io{};
        io.iov_base = &hello_;
        io.iov_len = sizeof(hello_);
        char cbuf[CMSG_SPACE(sizeof(int))];
        std::memset(cbuf, 0, sizeof(cbuf));
        msghdr msg{};
        msg.msg_iov = &io;
        msg.msg_iovlen = 1;
        msg.msg_control = cbuf;
        msg.msg_controllen = sizeof(cbuf);
        cmsghdr* cm = CMSG_FIRSTHDR(&msg);
        cm->cmsg_level = SOL_SOCKET;
        cm->cmsg_type = SCM_RIGHTS;
        cm->cmsg_len = CMSG_LEN(sizeof(int));
        std::memcpy(CMSG_DATA(cm), &image_fd_, sizeof(int));
        const ssize_t n = ::sendmsg(c.fd, &msg, MSG_NOSIGNAL);
        if (n < 0) {
            logf("[vrcompositor] capture server: sendmsg(HELLO) failed: %s", std::strerror(errno));
        }
        c.ready = true;
    }

    void loop() {
        while (!quit_.load()) {
            std::vector<pollfd> pfds;
            {
                std::lock_guard lock(mtx_);
                pfds.push_back(pollfd{listen_fd_, POLLIN, 0});
                for (const auto& c : clients_) {
                    pfds.push_back(pollfd{c.fd, POLLIN, 0});
                }
            }
            const int pr = ::poll(pfds.data(), pfds.size(), 200);
            if (pr <= 0) {
                continue;
            }
            std::lock_guard lock(mtx_);
            if ((pfds[0].revents & POLLIN) != 0) {
                for (;;) {
                    const int cfd = ::accept4(listen_fd_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
                    if (cfd < 0) {
                        break;
                    }
                    clients_.push_back(Client{cfd, true});
                    logf("[vrcompositor] capture server: client connected (fd %d)", cfd);
                    if (have_image_) {
                        sendHello(clients_.back());
                    }
                }
            }
            // client events: ack bytes (ready again) or hang-up
            std::size_t i = 1;
            for (auto it = clients_.begin(); it != clients_.end(); ++i) {
                const short re = (i < pfds.size()) ? pfds[i].revents : static_cast<short>(0);
                if ((re & (POLLIN | POLLHUP | POLLERR)) == 0) {
                    ++it;
                    continue;
                }
                std::uint8_t buf[64];
                const ssize_t n = ::recv(it->fd, buf, sizeof(buf), MSG_DONTWAIT);
                if (n > 0) {
                    it->ready = true; // any byte(s) from the client = "I have read the last frame, send the next"
                    ++it;
                } else if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
                    logf("[vrcompositor] capture server: client (fd %d) gone", it->fd);
                    ::close(it->fd);
                    it = clients_.erase(it);
                } else {
                    ++it;
                }
            }
        }
    }

    std::mutex mtx_;
    std::atomic<bool> quit_{false};
    bool started_ = false;
    int listen_fd_ = -1;
    std::thread thread_;
    std::vector<Client> clients_;
    bool have_image_ = false;
    HelloMsg hello_;
    int image_fd_ = -1;
};

CaptureServer& server() {
    static CaptureServer s;
    return s;
}

// ---- the loader's dispatch key: the first pointer of a dispatchable handle ---------------------------------------
void* disp_key(void* handle) { return *reinterpret_cast<void**>(handle); }

struct InstanceData {
    VkInstance handle = VK_NULL_HANDLE;
    PFN_vkGetInstanceProcAddr gipa = nullptr;
    PFN_vkDestroyInstance destroyInstance = nullptr;
    PFN_vkGetPhysicalDeviceMemoryProperties getMemProps = nullptr;
    PFN_vkGetPhysicalDeviceProperties2 getProps2 = nullptr;
};

struct SwapchainInfo {
    uint32_t w = 0, h = 0, arrayLayers = 0, sampleCount = 0;
    int64_t format = 0;
    VkImageUsageFlags usage = 0;
};

// All the device entry points the capture path uses, plus the exportable image and its one reused command buffer.
// Lives on the heap (shared_ptr in DeviceData) so the compositor thread can do Vulkan work under cap->mtx without
// holding the global map lock.
struct CaptureCtx {
    VkDevice device = VK_NULL_HANDLE;
    VkPhysicalDeviceMemoryProperties memProps{};
    std::uint8_t deviceUUID[16] = {};

    PFN_vkCreateImage createImage = nullptr;
    PFN_vkDestroyImage destroyImage = nullptr;
    PFN_vkGetImageMemoryRequirements getImageMemReq = nullptr;
    PFN_vkAllocateMemory allocateMemory = nullptr;
    PFN_vkFreeMemory freeMemory = nullptr;
    PFN_vkBindImageMemory bindImageMemory = nullptr;
    PFN_vkGetMemoryFdKHR getMemoryFd = nullptr;
    PFN_vkCreateCommandPool createCommandPool = nullptr;
    PFN_vkDestroyCommandPool destroyCommandPool = nullptr;
    PFN_vkAllocateCommandBuffers allocateCommandBuffers = nullptr;
    PFN_vkBeginCommandBuffer beginCommandBuffer = nullptr;
    PFN_vkEndCommandBuffer endCommandBuffer = nullptr;
    PFN_vkResetCommandBuffer resetCommandBuffer = nullptr;
    PFN_vkCmdPipelineBarrier cmdPipelineBarrier = nullptr;
    PFN_vkCmdBlitImage cmdBlitImage = nullptr;
    PFN_vkCreateFence createFence = nullptr;
    PFN_vkDestroyFence destroyFence = nullptr;
    PFN_vkResetFences resetFences = nullptr;
    PFN_vkGetFenceStatus getFenceStatus = nullptr;
    PFN_vkQueueSubmit queueSubmit = nullptr;

    std::mutex mtx;
    bool failed = false;
    bool haveResources = false;
    // the parameters the current resources were built for (rebuilt if they change)
    CaptureView view = CaptureView::LeftEye;
    uint32_t srcW = 0, srcH = 0, srcLayers = 0;
    uint32_t dstW = 0, dstH = 0;
    uint32_t poolFamily = UINT32_MAX;

    VkImage dstImage = VK_NULL_HANDLE;
    VkDeviceMemory dstMem = VK_NULL_HANDLE;
    int exportFd = -1;
    VkCommandPool pool = VK_NULL_HANDLE;
    VkCommandBuffer cmd = VK_NULL_HANDLE;
    VkFence fence = VK_NULL_HANDLE;

    bool pending = false;     // a blit is in flight (fence not yet signalled)
    std::uint64_t seq = 0;
    std::uint64_t pendingTs = 0;
    std::uint64_t lastBlitNs = 0;
    std::uint64_t capturedLogged = 0; // throttle the per-frame "blit submitted" log to the first few

    void destroyResources() {
        if (fence != VK_NULL_HANDLE) {
            destroyFence(device, fence, nullptr);
            fence = VK_NULL_HANDLE;
        }
        if (pool != VK_NULL_HANDLE) {
            destroyCommandPool(device, pool, nullptr); // frees cmd
            pool = VK_NULL_HANDLE;
            cmd = VK_NULL_HANDLE;
        }
        if (dstImage != VK_NULL_HANDLE) {
            destroyImage(device, dstImage, nullptr);
            dstImage = VK_NULL_HANDLE;
        }
        if (dstMem != VK_NULL_HANDLE) {
            freeMemory(device, dstMem, nullptr);
            dstMem = VK_NULL_HANDLE;
        }
        if (exportFd >= 0) {
            ::close(exportFd); // the server kept its own dup; this one is ours
            exportFd = -1;
        }
        haveResources = false;
        pending = false;
    }

    ~CaptureCtx() { destroyResources(); }
};

struct DeviceData {
    PFN_vkGetDeviceProcAddr gdpa = nullptr;
    PFN_vkDestroyDevice destroyDevice = nullptr;
    PFN_vkCreateImage createImage = nullptr;
    PFN_vkDestroyImage destroyImage = nullptr;
    PFN_vkGetMemoryFdKHR getMemoryFd = nullptr;
    PFN_vkQueueSubmit queueSubmit = nullptr;
    PFN_vkGetDeviceQueue getDeviceQueue = nullptr;
    std::set<std::tuple<uint32_t, uint32_t, uint32_t, int>> logged;    // (w,h,layers,format) already logged
    std::unordered_map<VkImage, SwapchainInfo> images;                // every external colour image, for selection
    std::unordered_map<VkQueue, uint32_t> queueFamily;                // VkQueue -> its family (from vkGetDeviceQueue)
    uint32_t primaryFamily = 0;                                       // queueCreateInfos[0], a fallback
    std::tuple<uint32_t, uint32_t, uint32_t, int> lastPicked{0, 0, 0, 0}; // last capture target logged (avoid spam)
    std::shared_ptr<CaptureCtx> cap;
};

std::mutex g_mutex;
std::unordered_map<void*, InstanceData> g_instances;
std::unordered_map<void*, DeviceData> g_devices;

InstanceData instance_data(void* h) {
    std::lock_guard lock(g_mutex);
    return g_instances[disp_key(h)];
}

// ---- helpers to walk the loader's link-info chain ----------------------------------------------------------------
VkLayerInstanceCreateInfo* instanceChainInfo(const VkInstanceCreateInfo* ci) {
    auto* p = static_cast<VkLayerInstanceCreateInfo*>(const_cast<void*>(ci->pNext));
    while (p != nullptr &&
           !(p->sType == VK_STRUCTURE_TYPE_LOADER_INSTANCE_CREATE_INFO && p->function == VK_LAYER_LINK_INFO)) {
        p = static_cast<VkLayerInstanceCreateInfo*>(const_cast<void*>(p->pNext));
    }
    return p;
}

VkLayerDeviceCreateInfo* deviceChainInfo(const VkDeviceCreateInfo* ci) {
    auto* p = static_cast<VkLayerDeviceCreateInfo*>(const_cast<void*>(ci->pNext));
    while (p != nullptr &&
           !(p->sType == VK_STRUCTURE_TYPE_LOADER_DEVICE_CREATE_INFO && p->function == VK_LAYER_LINK_INFO)) {
        p = static_cast<VkLayerDeviceCreateInfo*>(const_cast<void*>(p->pNext));
    }
    return p;
}

// forward decls of the hooks the loader can query
VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL prlGetInstanceProcAddr(VkInstance, const char*);
VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL prlGetDeviceProcAddr(VkDevice, const char*);

// ---- instance create / destroy -----------------------------------------------------------------------------------
VKAPI_ATTR VkResult VKAPI_CALL prlCreateInstance(const VkInstanceCreateInfo* pCreateInfo,
                                                 const VkAllocationCallbacks* pAllocator, VkInstance* pInstance) {
    log_open();
    VkLayerInstanceCreateInfo* link = instanceChainInfo(pCreateInfo);
    if (link == nullptr || link->u.pLayerInfo == nullptr) {
        return VK_ERROR_INITIALIZATION_FAILED;
    }
    const PFN_vkGetInstanceProcAddr gipa = link->u.pLayerInfo->pfnNextGetInstanceProcAddr;
    link->u.pLayerInfo = link->u.pLayerInfo->pNext; // advance the chain for the next layer
    auto createInstance = reinterpret_cast<PFN_vkCreateInstance>(gipa(VK_NULL_HANDLE, "vkCreateInstance"));
    if (createInstance == nullptr) {
        return VK_ERROR_INITIALIZATION_FAILED;
    }
    const VkResult r = createInstance(pCreateInfo, pAllocator, pInstance);
    if (r != VK_SUCCESS) {
        return r;
    }
    InstanceData d;
    d.handle = *pInstance;
    d.gipa = gipa;
    d.destroyInstance = reinterpret_cast<PFN_vkDestroyInstance>(gipa(*pInstance, "vkDestroyInstance"));
    d.getMemProps = reinterpret_cast<PFN_vkGetPhysicalDeviceMemoryProperties>(
        gipa(*pInstance, "vkGetPhysicalDeviceMemoryProperties"));
    d.getProps2 = reinterpret_cast<PFN_vkGetPhysicalDeviceProperties2>(gipa(*pInstance, "vkGetPhysicalDeviceProperties2"));
    if (d.getProps2 == nullptr) {
        d.getProps2 =
            reinterpret_cast<PFN_vkGetPhysicalDeviceProperties2>(gipa(*pInstance, "vkGetPhysicalDeviceProperties2KHR"));
    }
    {
        std::lock_guard lock(g_mutex);
        g_instances[disp_key(*pInstance)] = d;
    }
    const char* app = (pCreateInfo->pApplicationInfo != nullptr && pCreateInfo->pApplicationInfo->pApplicationName != nullptr)
                          ? pCreateInfo->pApplicationInfo->pApplicationName
                          : "(unnamed)";
    logf("pibridge-record-layer (M2b capture) [%s]: vkCreateInstance, app \"%s\"", proc_name(), app);
    if (in_compositor()) {
        server().start(); // the capture server lives only in vrcompositor; a client may connect any time
    }
    return r;
}

VKAPI_ATTR void VKAPI_CALL prlDestroyInstance(VkInstance instance, const VkAllocationCallbacks* pAllocator) {
    InstanceData d = instance_data(instance);
    if (d.destroyInstance != nullptr) {
        d.destroyInstance(instance, pAllocator);
    }
    std::lock_guard lock(g_mutex);
    g_instances.erase(disp_key(instance));
}

// ---- device create / destroy -------------------------------------------------------------------------------------
VKAPI_ATTR VkResult VKAPI_CALL prlCreateDevice(VkPhysicalDevice physicalDevice, const VkDeviceCreateInfo* pCreateInfo,
                                               const VkAllocationCallbacks* pAllocator, VkDevice* pDevice) {
    VkLayerDeviceCreateInfo* link = deviceChainInfo(pCreateInfo);
    if (link == nullptr || link->u.pLayerInfo == nullptr) {
        return VK_ERROR_INITIALIZATION_FAILED;
    }
    const PFN_vkGetInstanceProcAddr gipa = link->u.pLayerInfo->pfnNextGetInstanceProcAddr;
    const PFN_vkGetDeviceProcAddr gdpa = link->u.pLayerInfo->pfnNextGetDeviceProcAddr;
    link->u.pLayerInfo = link->u.pLayerInfo->pNext; // advance
    auto createDevice = reinterpret_cast<PFN_vkCreateDevice>(gipa(VK_NULL_HANDLE, "vkCreateDevice"));
    if (createDevice == nullptr) {
        return VK_ERROR_INITIALIZATION_FAILED;
    }
    const VkResult r = createDevice(physicalDevice, pCreateInfo, pAllocator, pDevice);
    if (r != VK_SUCCESS) {
        return r;
    }
    DeviceData d;
    d.gdpa = gdpa;
    const auto load = [&](const char* n) { return gdpa(*pDevice, n); };
    d.destroyDevice = reinterpret_cast<PFN_vkDestroyDevice>(load("vkDestroyDevice"));
    d.createImage = reinterpret_cast<PFN_vkCreateImage>(load("vkCreateImage"));
    d.destroyImage = reinterpret_cast<PFN_vkDestroyImage>(load("vkDestroyImage"));
    d.getMemoryFd = reinterpret_cast<PFN_vkGetMemoryFdKHR>(load("vkGetMemoryFdKHR"));
    d.queueSubmit = reinterpret_cast<PFN_vkQueueSubmit>(load("vkQueueSubmit"));
    d.getDeviceQueue = reinterpret_cast<PFN_vkGetDeviceQueue>(load("vkGetDeviceQueue"));
    if (pCreateInfo->queueCreateInfoCount > 0 && pCreateInfo->pQueueCreateInfos != nullptr) {
        d.primaryFamily = pCreateInfo->pQueueCreateInfos[0].queueFamilyIndex;
    }

    // The capture context: only the compositor needs it, and only when vkGetMemoryFdKHR exists (external memory).
    if (in_compositor() && d.getMemoryFd != nullptr) {
        auto cap = std::make_shared<CaptureCtx>();
        cap->device = *pDevice;
        cap->createImage = d.createImage;
        cap->destroyImage = d.destroyImage;
        cap->getImageMemReq = reinterpret_cast<PFN_vkGetImageMemoryRequirements>(load("vkGetImageMemoryRequirements"));
        cap->allocateMemory = reinterpret_cast<PFN_vkAllocateMemory>(load("vkAllocateMemory"));
        cap->freeMemory = reinterpret_cast<PFN_vkFreeMemory>(load("vkFreeMemory"));
        cap->bindImageMemory = reinterpret_cast<PFN_vkBindImageMemory>(load("vkBindImageMemory"));
        cap->getMemoryFd = d.getMemoryFd;
        cap->createCommandPool = reinterpret_cast<PFN_vkCreateCommandPool>(load("vkCreateCommandPool"));
        cap->destroyCommandPool = reinterpret_cast<PFN_vkDestroyCommandPool>(load("vkDestroyCommandPool"));
        cap->allocateCommandBuffers = reinterpret_cast<PFN_vkAllocateCommandBuffers>(load("vkAllocateCommandBuffers"));
        cap->beginCommandBuffer = reinterpret_cast<PFN_vkBeginCommandBuffer>(load("vkBeginCommandBuffer"));
        cap->endCommandBuffer = reinterpret_cast<PFN_vkEndCommandBuffer>(load("vkEndCommandBuffer"));
        cap->resetCommandBuffer = reinterpret_cast<PFN_vkResetCommandBuffer>(load("vkResetCommandBuffer"));
        cap->cmdPipelineBarrier = reinterpret_cast<PFN_vkCmdPipelineBarrier>(load("vkCmdPipelineBarrier"));
        cap->cmdBlitImage = reinterpret_cast<PFN_vkCmdBlitImage>(load("vkCmdBlitImage"));
        cap->createFence = reinterpret_cast<PFN_vkCreateFence>(load("vkCreateFence"));
        cap->destroyFence = reinterpret_cast<PFN_vkDestroyFence>(load("vkDestroyFence"));
        cap->resetFences = reinterpret_cast<PFN_vkResetFences>(load("vkResetFences"));
        cap->getFenceStatus = reinterpret_cast<PFN_vkGetFenceStatus>(load("vkGetFenceStatus"));
        cap->queueSubmit = d.queueSubmit;

        InstanceData inst = instance_data(physicalDevice);
        if (inst.getMemProps != nullptr) {
            inst.getMemProps(physicalDevice, &cap->memProps);
        }
        if (inst.getProps2 != nullptr) {
            VkPhysicalDeviceIDProperties idp{};
            idp.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ID_PROPERTIES;
            VkPhysicalDeviceProperties2 p2{};
            p2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
            p2.pNext = &idp;
            inst.getProps2(physicalDevice, &p2);
            std::memcpy(cap->deviceUUID, idp.deviceUUID, sizeof(cap->deviceUUID));
        }
        // All the capture entry points must be present; otherwise capture stays disabled (VR runs on).
        const bool ok = cap->getImageMemReq && cap->allocateMemory && cap->freeMemory && cap->bindImageMemory &&
                        cap->getMemoryFd && cap->createCommandPool && cap->destroyCommandPool &&
                        cap->allocateCommandBuffers && cap->beginCommandBuffer && cap->endCommandBuffer &&
                        cap->resetCommandBuffer && cap->cmdPipelineBarrier && cap->cmdBlitImage && cap->createFence &&
                        cap->destroyFence && cap->resetFences && cap->getFenceStatus && cap->queueSubmit &&
                        inst.getMemProps != nullptr;
        cap->failed = !ok;
        if (!ok) {
            logf("[vrcompositor] capture disabled: a required entry point or memory property is missing");
        }
        d.cap = cap;
    }

    {
        std::lock_guard lock(g_mutex);
        g_devices[disp_key(*pDevice)] = std::move(d);
    }
    logf("vkCreateDevice: device ready [%s] (vkGetMemoryFdKHR %s, capture %s)", proc_name(),
         gdpa(*pDevice, "vkGetMemoryFdKHR") != nullptr ? "present" : "absent",
         in_compositor() && gdpa(*pDevice, "vkGetMemoryFdKHR") != nullptr ? "armed" : "off");
    return r;
}

VKAPI_ATTR void VKAPI_CALL prlDestroyDevice(VkDevice device, const VkAllocationCallbacks* pAllocator) {
    PFN_vkDestroyDevice destroy = nullptr;
    std::shared_ptr<CaptureCtx> cap;
    {
        std::lock_guard lock(g_mutex);
        auto it = g_devices.find(disp_key(device));
        if (it != g_devices.end()) {
            destroy = it->second.destroyDevice;
            cap = it->second.cap;
        }
    }
    if (cap) {
        std::lock_guard lock(cap->mtx);
        cap->destroyResources();
    }
    if (destroy != nullptr) {
        destroy(device, pAllocator);
    }
    std::lock_guard lock(g_mutex);
    g_devices.erase(disp_key(device));
}

VKAPI_ATTR void VKAPI_CALL prlGetDeviceQueue(VkDevice device, uint32_t queueFamilyIndex, uint32_t queueIndex,
                                             VkQueue* pQueue) {
    PFN_vkGetDeviceQueue down = nullptr;
    {
        std::lock_guard lock(g_mutex);
        auto it = g_devices.find(disp_key(device));
        if (it != g_devices.end()) {
            down = it->second.getDeviceQueue;
        }
    }
    if (down == nullptr) {
        return;
    }
    down(device, queueFamilyIndex, queueIndex, pQueue);
    if (pQueue != nullptr && *pQueue != VK_NULL_HANDLE) {
        std::lock_guard lock(g_mutex);
        auto it = g_devices.find(disp_key(device));
        if (it != g_devices.end()) {
            it->second.queueFamily[*pQueue] = queueFamilyIndex;
        }
    }
}

// ---- observe external colour images (eye / compositor candidates) ------------------------------------------------
VKAPI_ATTR VkResult VKAPI_CALL prlCreateImage(VkDevice device, const VkImageCreateInfo* pCreateInfo,
                                              const VkAllocationCallbacks* pAllocator, VkImage* pImage) {
    PFN_vkCreateImage down = nullptr;
    {
        std::lock_guard lock(g_mutex);
        auto it = g_devices.find(disp_key(device));
        if (it != g_devices.end()) {
            down = it->second.createImage;
        }
    }
    if (down == nullptr) {
        return VK_ERROR_INITIALIZATION_FAILED;
    }
    const VkResult r = down(device, pCreateInfo, pAllocator, pImage);
    if (r != VK_SUCCESS || pCreateInfo == nullptr) {
        return r;
    }
    VkExternalMemoryHandleTypeFlags handleTypes = 0;
    for (const auto* p = static_cast<const VkBaseInStructure*>(pCreateInfo->pNext); p != nullptr; p = p->pNext) {
        if (p->sType == VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO) {
            handleTypes = reinterpret_cast<const VkExternalMemoryImageCreateInfo*>(p)->handleTypes;
        }
    }
    const bool external = handleTypes != 0;
    const bool color = (pCreateInfo->usage & VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT) != 0;
    if (external && color) {
        const auto k = std::make_tuple(pCreateInfo->extent.width, pCreateInfo->extent.height,
                                       pCreateInfo->arrayLayers, static_cast<int>(pCreateInfo->format));
        SwapchainInfo si{pCreateInfo->extent.width, pCreateInfo->extent.height, pCreateInfo->arrayLayers,
                         static_cast<uint32_t>(pCreateInfo->samples), static_cast<int64_t>(pCreateInfo->format),
                         pCreateInfo->usage};
        bool fresh = false;
        {
            std::lock_guard lock(g_mutex);
            auto it = g_devices.find(disp_key(device));
            if (it != g_devices.end()) {
                fresh = it->second.logged.insert(k).second;
                it->second.images[*pImage] = si; // remember every external colour image, for capture selection
            }
        }
        if (fresh) {
            logf("[%s] external image: %ux%u arrayLayers %u samples %u format %d usage 0x%x handleTypes 0x%x%s",
                 proc_name(), pCreateInfo->extent.width, pCreateInfo->extent.height, pCreateInfo->arrayLayers,
                 static_cast<unsigned>(pCreateInfo->samples), static_cast<int>(pCreateInfo->format),
                 static_cast<unsigned>(pCreateInfo->usage), static_cast<unsigned>(handleTypes),
                 pCreateInfo->arrayLayers >= 2 ? "  <- stereo: likely the eye buffer (layer 0 = left, 1 = right)" : "");
        }
    }
    return r;
}

VKAPI_ATTR void VKAPI_CALL prlDestroyImage(VkDevice device, VkImage image, const VkAllocationCallbacks* pAllocator) {
    PFN_vkDestroyImage down = nullptr;
    {
        std::lock_guard lock(g_mutex);
        auto it = g_devices.find(disp_key(device));
        if (it != g_devices.end()) {
            down = it->second.destroyImage;
            it->second.images.erase(image);
        }
    }
    if (down != nullptr) {
        down(device, image, pAllocator);
    }
}

// ---- M2b capture helpers -----------------------------------------------------------------------------------------
// Choose a device-local memory type that satisfies the requirement bitmask.
int chooseMemoryType(const VkPhysicalDeviceMemoryProperties& mp, uint32_t typeBits) {
    for (uint32_t i = 0; i < mp.memoryTypeCount; i++) {
        if ((typeBits & (1u << i)) != 0 &&
            (mp.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) != 0) {
            return static_cast<int>(i);
        }
    }
    // fall back to any allowed type
    for (uint32_t i = 0; i < mp.memoryTypeCount; i++) {
        if ((typeBits & (1u << i)) != 0) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

uint32_t evenDown(uint32_t v) { return v >= 2 ? (v & ~1u) : 2u; }

// Build the exportable image + command buffer + fence for the current (view, src dims, max_height). Returns true on
// success; on any failure logs and leaves cap->failed set so capture never retries this session.
bool buildResources(CaptureCtx& cap, CaptureView view, const SwapchainInfo& src, uint32_t maxH, uint32_t family,
                    uint32_t fps) {
    cap.destroyResources();
    cap.view = view;
    cap.srcW = src.w;
    cap.srcH = src.h;
    cap.srcLayers = src.arrayLayers;
    cap.poolFamily = family;

    const uint32_t srcH = src.h > 0 ? src.h : 1;
    const uint32_t targetH = (maxH > 0 && maxH < srcH) ? maxH : srcH;
    const double scale = static_cast<double>(targetH) / static_cast<double>(srcH);
    const uint32_t perEyeW = evenDown(static_cast<uint32_t>(static_cast<double>(src.w) * scale + 0.5));
    const uint32_t dstH = evenDown(targetH);
    const uint32_t dstW = view == CaptureView::BothEyes ? evenDown(perEyeW * 2) : perEyeW;
    cap.dstW = dstW;
    cap.dstH = dstH;

    VkExternalMemoryImageCreateInfo ext{};
    ext.sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO;
    ext.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT;
    VkImageCreateInfo ici{};
    ici.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    ici.pNext = &ext;
    ici.imageType = VK_IMAGE_TYPE_2D;
    ici.format = static_cast<VkFormat>(kCaptureVkFormat);
    ici.extent = VkExtent3D{dstW, dstH, 1};
    ici.mipLevels = 1;
    ici.arrayLayers = 1;
    ici.samples = VK_SAMPLE_COUNT_1_BIT;
    ici.tiling = VK_IMAGE_TILING_OPTIMAL;
    ici.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    ici.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    if (cap.createImage(cap.device, &ici, nullptr, &cap.dstImage) != VK_SUCCESS) {
        logf("[vrcompositor] capture: vkCreateImage(%ux%u) failed", dstW, dstH);
        cap.failed = true;
        return false;
    }
    VkMemoryRequirements mr{};
    cap.getImageMemReq(cap.device, cap.dstImage, &mr);
    const int memType = chooseMemoryType(cap.memProps, mr.memoryTypeBits);
    if (memType < 0) {
        logf("[vrcompositor] capture: no device-local memory type");
        cap.failed = true;
        return false;
    }
    VkExportMemoryAllocateInfo exp{};
    exp.sType = VK_STRUCTURE_TYPE_EXPORT_MEMORY_ALLOCATE_INFO;
    exp.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT;
    VkMemoryAllocateInfo mai{};
    mai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    mai.pNext = &exp;
    mai.allocationSize = mr.size;
    mai.memoryTypeIndex = static_cast<uint32_t>(memType);
    if (cap.allocateMemory(cap.device, &mai, nullptr, &cap.dstMem) != VK_SUCCESS) {
        logf("[vrcompositor] capture: vkAllocateMemory(%llu) failed", static_cast<unsigned long long>(mr.size));
        cap.failed = true;
        return false;
    }
    if (cap.bindImageMemory(cap.device, cap.dstImage, cap.dstMem, 0) != VK_SUCCESS) {
        logf("[vrcompositor] capture: vkBindImageMemory failed");
        cap.failed = true;
        return false;
    }
    VkMemoryGetFdInfoKHR gfi{};
    gfi.sType = VK_STRUCTURE_TYPE_MEMORY_GET_FD_INFO_KHR;
    gfi.memory = cap.dstMem;
    gfi.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT;
    if (cap.getMemoryFd(cap.device, &gfi, &cap.exportFd) != VK_SUCCESS || cap.exportFd < 0) {
        logf("[vrcompositor] capture: vkGetMemoryFdKHR failed");
        cap.failed = true;
        return false;
    }
    VkCommandPoolCreateInfo pci{};
    pci.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    pci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    pci.queueFamilyIndex = family;
    if (cap.createCommandPool(cap.device, &pci, nullptr, &cap.pool) != VK_SUCCESS) {
        logf("[vrcompositor] capture: vkCreateCommandPool(family %u) failed", family);
        cap.failed = true;
        return false;
    }
    VkCommandBufferAllocateInfo cbi{};
    cbi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    cbi.commandPool = cap.pool;
    cbi.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cbi.commandBufferCount = 1;
    if (cap.allocateCommandBuffers(cap.device, &cbi, &cap.cmd) != VK_SUCCESS) {
        logf("[vrcompositor] capture: vkAllocateCommandBuffers failed");
        cap.failed = true;
        return false;
    }
    VkFenceCreateInfo fci{};
    fci.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    if (cap.createFence(cap.device, &fci, nullptr, &cap.fence) != VK_SUCCESS) {
        logf("[vrcompositor] capture: vkCreateFence failed");
        cap.failed = true;
        return false;
    }

    HelloMsg h;
    h.width = dstW;
    h.height = dstH;
    h.fps = fps;
    h.allocationSize = mr.size;
    h.memoryTypeIndex = static_cast<uint32_t>(memType);
    h.queueFamilyIndex = family;
    std::memcpy(h.deviceUUID, cap.deviceUUID, sizeof(h.deviceUUID));
    server().setImage(h, cap.exportFd);

    cap.haveResources = true;
    logf("[vrcompositor] capture: exportable image %ux%u created, fd exported, %llu bytes (src %ux%u layers %u, view %d)",
         dstW, dstH, static_cast<unsigned long long>(mr.size), src.w, src.h, src.arrayLayers, static_cast<int>(view));
    return true;
}

// Record + submit one blit of the selected view/layer(s) into the exportable image, with its own fence. Submitted to
// `queue` (the queue the app just used) so it is ordered after the app's work on that queue, and we never wait on the
// fence here (the compositor is never stalled). Returns true if a blit was submitted.
bool submitBlit(CaptureCtx& cap, VkQueue queue, VkImage srcImage, CaptureView view) {
    if (cap.resetCommandBuffer(cap.cmd, 0) != VK_SUCCESS) {
        return false;
    }
    VkCommandBufferBeginInfo bi{};
    bi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    if (cap.beginCommandBuffer(cap.cmd, &bi) != VK_SUCCESS) {
        return false;
    }

    const uint32_t srcLayerCount = view == CaptureView::BothEyes ? 2u : 1u;
    const uint32_t srcBaseLayer = view == CaptureView::RightEye ? 1u : 0u;

    // ON-HARDWARE ITERATION #1: the source image's current layout. Best guess: the compositor leaves a frame bound
    // for the HMD in SHADER_READ_ONLY_OPTIMAL. We transition it to TRANSFER_SRC and back. If this proves wrong on the
    // live headset (validation/corruption), the fallbacks to try are GENERAL or PRESENT_SRC_KHR as oldLayout, or an
    // oldLayout of UNDEFINED only if a torn frame is acceptable (UNDEFINED discards contents on some drivers).
    VkImageMemoryBarrier srcToTransfer{};
    srcToTransfer.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    srcToTransfer.srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
    srcToTransfer.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
    srcToTransfer.oldLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    srcToTransfer.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    srcToTransfer.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    srcToTransfer.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    srcToTransfer.image = srcImage;
    srcToTransfer.subresourceRange = VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, srcBaseLayer, srcLayerCount};

    VkImageMemoryBarrier dstToTransfer{};
    dstToTransfer.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    dstToTransfer.srcAccessMask = 0;
    dstToTransfer.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    dstToTransfer.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    dstToTransfer.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    dstToTransfer.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    dstToTransfer.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    dstToTransfer.image = cap.dstImage;
    dstToTransfer.subresourceRange = VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};

    VkImageMemoryBarrier pre[2] = {srcToTransfer, dstToTransfer};
    cap.cmdPipelineBarrier(cap.cmd, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0,
                           nullptr, 2, pre);

    VkImageBlit regions[2]{};
    const int32_t sw = static_cast<int32_t>(cap.srcW);
    const int32_t sh = static_cast<int32_t>(cap.srcH);
    const int32_t perEyeW = static_cast<int32_t>(view == CaptureView::BothEyes ? cap.dstW / 2 : cap.dstW);
    const int32_t dh = static_cast<int32_t>(cap.dstH);
    uint32_t regionCount = 0;
    const auto addRegion = [&](uint32_t srcLayer, int32_t dstX0, int32_t dstX1) {
        VkImageBlit& b = regions[regionCount++];
        b.srcSubresource = VkImageSubresourceLayers{VK_IMAGE_ASPECT_COLOR_BIT, 0, srcLayer, 1};
        b.srcOffsets[0] = VkOffset3D{0, 0, 0};
        b.srcOffsets[1] = VkOffset3D{sw, sh, 1};
        b.dstSubresource = VkImageSubresourceLayers{VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        b.dstOffsets[0] = VkOffset3D{dstX0, 0, 0};
        b.dstOffsets[1] = VkOffset3D{dstX1, dh, 1};
    };
    if (view == CaptureView::BothEyes) {
        addRegion(0, 0, perEyeW);
        addRegion(1, perEyeW, perEyeW * 2);
    } else {
        addRegion(srcBaseLayer, 0, perEyeW);
    }
    cap.cmdBlitImage(cap.cmd, srcImage, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, cap.dstImage,
                     VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, regionCount, regions, VK_FILTER_LINEAR);

    // dst: TRANSFER_DST -> TRANSFER_SRC, left in that layout for the client to copy out of. ON-HARDWARE ITERATION #2:
    // cross-PROCESS synchronisation. We do NOT do a queue-family ownership transfer (families IGNORED): the client is
    // a different VkDevice, and the happens-before is carried coarsely by this blit's fence (the layer signals the
    // client only after the fence) plus the socket ack round-trip (the layer never reuses the image until the client
    // acked). On the single-queue-family RTX 5090 this is reliable; the spec-clean upgrade (flagged TODO) is a shared
    // EXTERNAL (opaque-fd) timeline semaphore signalled by this submit and waited on by the client, instead of the
    // fence+ack. Fallback if the client reads garbage anyway: make the exportable image LINEAR + host-visible and have
    // the client mmap the dma_buf directly (no second Vulkan device), at the cost of a slower blit target.
    VkImageMemoryBarrier dstRelease{};
    dstRelease.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    dstRelease.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    dstRelease.dstAccessMask = 0;
    dstRelease.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    dstRelease.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    dstRelease.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    dstRelease.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    dstRelease.image = cap.dstImage;
    dstRelease.subresourceRange = VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};

    // src: TRANSFER_SRC -> back to SHADER_READ_ONLY so the compositor reads it unchanged.
    VkImageMemoryBarrier srcBack = srcToTransfer;
    srcBack.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
    srcBack.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    srcBack.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    srcBack.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkImageMemoryBarrier post[2] = {dstRelease, srcBack};
    cap.cmdPipelineBarrier(cap.cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0,
                           nullptr, 2, post);

    if (cap.endCommandBuffer(cap.cmd) != VK_SUCCESS) {
        return false;
    }
    VkSubmitInfo si{};
    si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    si.commandBufferCount = 1;
    si.pCommandBuffers = &cap.cmd;
    const VkResult sr = cap.queueSubmit(queue, 1, &si, cap.fence);
    if (sr != VK_SUCCESS) {
        logf("[vrcompositor] capture: vkQueueSubmit(blit) = %d", static_cast<int>(sr));
        return false;
    }
    return true;
}

// Called from prlQueueSubmit AFTER the app's own submit was forwarded. Advances the pending blit, and starts a new one
// when due. Never blocks the compositor.
void maybeCapture(const std::shared_ptr<CaptureCtx>& capPtr, VkQueue queue, uint32_t family, const Control& ctl,
                  VkImage srcImage, const SwapchainInfo& srcInfo) {
    if (!capPtr) {
        return;
    }
    CaptureCtx& cap = *capPtr;
    std::lock_guard lock(cap.mtx);
    if (cap.failed) {
        return;
    }
    // Retire a pending blit: when its fence is signalled the shared image holds the frame; tell the clients.
    if (cap.pending) {
        if (cap.getFenceStatus(cap.device, cap.fence) == VK_SUCCESS) {
            cap.resetFences(cap.device, 1, &cap.fence);
            cap.pending = false;
            server().postFrame(cap.seq, cap.pendingTs);
        } else {
            return; // still in flight
        }
    }
    if (!server().hasClient() || !server().allReady()) {
        return; // no consumer, or it has not read the previous frame yet
    }
    const uint32_t fps = ctl.fps > 0 ? ctl.fps : 60;
    const std::uint64_t now = now_ns();
    if (cap.lastBlitNs != 0 && now - cap.lastBlitNs < 1000000000ull / fps) {
        return; // pace to fps
    }
    // (re)build the exportable image if the parameters changed since last time
    if (!cap.haveResources || cap.view != ctl.view || cap.srcW != srcInfo.w || cap.srcH != srcInfo.h ||
        cap.poolFamily != family) {
        if (!buildResources(cap, ctl.view, srcInfo, ctl.max_height, family, fps)) {
            return;
        }
    }
    if (!submitBlit(cap, queue, srcImage, ctl.view)) {
        // one failed submit is not fatal; but a reset is needed before the next try (the fence may be unsignalled)
        cap.resetFences(cap.device, 1, &cap.fence);
        return;
    }
    cap.pending = true;
    cap.seq++;
    cap.pendingTs = now;
    cap.lastBlitNs = now;
    if (cap.capturedLogged < 3) {
        cap.capturedLogged++;
        logf("[vrcompositor] capture: blit submitted (seq %llu, %ux%u)", static_cast<unsigned long long>(cap.seq),
             cap.dstW, cap.dstH);
    }
}

// ---- the submit hook: forward the app's work, then (in the compositor) capture the selected view -----------------
VKAPI_ATTR VkResult VKAPI_CALL prlQueueSubmit(VkQueue queue, uint32_t submitCount, const VkSubmitInfo* pSubmits,
                                              VkFence fence) {
    PFN_vkQueueSubmit down = nullptr;
    std::shared_ptr<CaptureCtx> cap;
    uint32_t family = 0;
    {
        std::lock_guard lock(g_mutex);
        auto it = g_devices.find(disp_key(queue));
        if (it != g_devices.end()) {
            down = it->second.queueSubmit;
            cap = it->second.cap;
            auto qf = it->second.queueFamily.find(queue);
            family = qf != it->second.queueFamily.end() ? qf->second : it->second.primaryFamily;
        }
    }
    if (down == nullptr) {
        return VK_ERROR_INITIALIZATION_FAILED;
    }
    // Forward the app's submit FIRST: the capture must never delay the compositor's own frame.
    const VkResult r = down(queue, submitCount, pSubmits, fence);

    if (in_compositor() && cap) {
        static std::atomic<std::uint64_t> submits{0};
        const std::uint64_t i = submits.fetch_add(1);
        // read the control only now and then (it is a file poll); keep the last one between polls
        static std::mutex ctlMtx;
        static Control lastCtl;
        Control ctl;
        if (i % 60 == 0) {
            ctl = read_control();
            std::lock_guard lk(ctlMtx);
            lastCtl = ctl;
        } else {
            std::lock_guard lk(ctlMtx);
            ctl = lastCtl;
        }
        if (ctl.active) {
            // pick the target image for the chosen view: the game's eye buffer = the largest arrayLayers>=2 colour
            // image. Only the device that holds it captures.
            VkImage bestImg = VK_NULL_HANDLE;
            SwapchainInfo bestInfo;
            std::uint64_t bestArea = 0;
            {
                std::lock_guard lock(g_mutex);
                auto it = g_devices.find(disp_key(queue));
                if (it != g_devices.end()) {
                    for (const auto& [img, info] : it->second.images) {
                        if (info.arrayLayers < 2) {
                            continue;
                        }
                        const std::uint64_t area = static_cast<std::uint64_t>(info.w) * info.h;
                        if (area > bestArea) {
                            bestArea = area;
                            bestImg = img;
                            bestInfo = info;
                        }
                    }
                    if (bestImg != VK_NULL_HANDLE) {
                        const auto k = std::make_tuple(bestInfo.w, bestInfo.h, bestInfo.arrayLayers,
                                                       static_cast<int>(bestInfo.format));
                        if (k != it->second.lastPicked) {
                            it->second.lastPicked = k;
                            const char* vn = ctl.view == CaptureView::LeftEye    ? "left"
                                             : ctl.view == CaptureView::RightEye ? "right"
                                             : ctl.view == CaptureView::BothEyes ? "both"
                                                                                 : "compositor";
                            logf("[vrcompositor] CAPTURE target view=%s: image %ux%u arrayLayers %u format %lld", vn,
                                 bestInfo.w, bestInfo.h, bestInfo.arrayLayers, static_cast<long long>(bestInfo.format));
                        }
                    }
                }
            }
            if (bestImg != VK_NULL_HANDLE) {
                maybeCapture(cap, queue, family, ctl, bestImg, bestInfo);
            }
        }
    }
    return r;
}

// ---- proc-addr dispatch ------------------------------------------------------------------------------------------
struct Hook {
    const char* name;
    PFN_vkVoidFunction fn;
};
#define PRL_HOOK(n, f) Hook{#n, reinterpret_cast<PFN_vkVoidFunction>(f)}
const Hook kHooks[] = {
    PRL_HOOK(vkGetInstanceProcAddr, prlGetInstanceProcAddr),
    PRL_HOOK(vkCreateInstance, prlCreateInstance),
    PRL_HOOK(vkDestroyInstance, prlDestroyInstance),
    PRL_HOOK(vkCreateDevice, prlCreateDevice),
    PRL_HOOK(vkGetDeviceProcAddr, prlGetDeviceProcAddr),
    PRL_HOOK(vkDestroyDevice, prlDestroyDevice),
    PRL_HOOK(vkGetDeviceQueue, prlGetDeviceQueue),
    PRL_HOOK(vkCreateImage, prlCreateImage),
    PRL_HOOK(vkDestroyImage, prlDestroyImage),
    PRL_HOOK(vkQueueSubmit, prlQueueSubmit),
};
#undef PRL_HOOK

PFN_vkVoidFunction hookFor(const char* name) {
    for (const Hook& h : kHooks) {
        if (std::strcmp(h.name, name) == 0) {
            return h.fn;
        }
    }
    return nullptr;
}

VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL prlGetInstanceProcAddr(VkInstance instance, const char* name) {
    if (PFN_vkVoidFunction h = hookFor(name); h != nullptr) {
        return h;
    }
    if (instance == VK_NULL_HANDLE) {
        return nullptr;
    }
    const InstanceData d = instance_data(instance);
    return d.gipa != nullptr ? d.gipa(instance, name) : nullptr;
}

VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL prlGetDeviceProcAddr(VkDevice device, const char* name) {
    if (PFN_vkVoidFunction h = hookFor(name); h != nullptr) {
        return h;
    }
    PFN_vkGetDeviceProcAddr gdpa = nullptr;
    {
        std::lock_guard lock(g_mutex);
        auto it = g_devices.find(disp_key(device));
        if (it != g_devices.end()) {
            gdpa = it->second.gdpa;
        }
    }
    return gdpa != nullptr ? gdpa(device, name) : nullptr;
}

} // namespace
} // namespace pibridge::reclayer

PRL_EXPORT VKAPI_ATTR VkResult VKAPI_CALL vkNegotiateLoaderLayerInterfaceVersion(VkNegotiateLayerInterface* pv) {
    using namespace pibridge::reclayer;
    if (pv == nullptr || pv->sType != LAYER_NEGOTIATE_INTERFACE_STRUCT) {
        return VK_ERROR_INITIALIZATION_FAILED;
    }
    if (pv->loaderLayerInterfaceVersion > CURRENT_LOADER_LAYER_INTERFACE_VERSION) {
        pv->loaderLayerInterfaceVersion = CURRENT_LOADER_LAYER_INTERFACE_VERSION;
    }
    pv->pfnGetInstanceProcAddr = prlGetInstanceProcAddr;
    pv->pfnGetDeviceProcAddr = prlGetDeviceProcAddr;
    pv->pfnGetPhysicalDeviceProcAddr = nullptr;
    return VK_SUCCESS;
}
