// SPDX-License-Identifier: MIT
// The encoder for `pibridge-record run --source layer` (M3): the CLIENT of PiBridge's Vulkan capture layer
// (tools/record-layer, the SERVER inside SteamVR's vrcompositor). It connects to the layer's unix socket, receives the
// exported eye-image fd + metadata (pibridge_record_capture_ipc.h), imports the fd into its own Vulkan device on the
// same GPU, and per frame reads the shared image back into a CPU RGBA buffer, which it feeds to the SAME ffmpeg
// pipeline the tap/mirror path uses (encode.cpp) — so NVENC (h264_nvenc / hevc_nvenc) encodes it on NVIDIA, VAAPI or
// libx264/libx265 otherwise, exactly as record.toml's `encoder` asks.
//
// Vendor paths (EncoderPref):
//   - NVIDIA: the readback buffer is handed to ffmpeg's h264_nvenc/hevc_nvenc (GPU-encoded). The planned zero-copy
//     optimisation — import the OPAQUE_FD into CUDA (cudaImportExternalMemory) and feed NVENC's CUDA hw_frames_ctx
//     with no readback — is a TODO below (needs the CUDA toolkit, absent on this build host; marked for on-hardware
//     iteration). The readback path already uses the NVENC encoder, just not zero-copy.
//   - AMD/Intel: ffmpeg h264_vaapi/hevc_vaapi fed the readback buffer. The zero-copy dma_buf->VAAPI import is a TODO
//     (needs libva-dev, absent here).
//   - Any GPU: the readback buffer + libx264/libx265 on the CPU (the always-available fallback).
//
// Reliability: a leaf. If the layer is not loaded, capture is not active, or any Vulkan/import step fails, this exits
// cleanly with a plain-words status; the VR session is never touched (it does not even run in vrcompositor).
#include "pibridge/record/run.h"

#include "pibridge/record/encode.h"
#include "pibridge/record/proc.h"
#include "pibridge/record/status.h"
#include "pibridge_env.h"
#include "pibridge_record_capture_control.h"
#include "pibridge_record_capture_ipc.h"

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <format>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <sys/resource.h>
#include <sys/syscall.h>

#ifdef PIBRIDGE_HAVE_VULKAN
#include <vulkan/vulkan.h>
#endif

namespace pibridge::record {

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace {

std::atomic<bool> g_stop{false};
extern "C" void onLayerSignal(int) { g_stop.store(true); }

std::uint64_t nowNsMono() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ull + static_cast<std::uint64_t>(ts.tv_nsec);
}

void beNiceLayer() {
    setpriority(PRIO_PROCESS, 0, 10);
    syscall(SYS_ioprio_set, 1, 0, (2 << 13) | 7);
}

// Read exactly n bytes with a deadline; false on EOF/error/timeout.
bool recvAll(int fd, void* buf, std::size_t n, std::chrono::milliseconds timeout) {
    auto* p = static_cast<std::uint8_t*>(buf);
    std::size_t got = 0;
    const auto t0 = std::chrono::steady_clock::now();
    while (got < n) {
        const auto left =
            timeout - std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
        if (left <= 0ms) {
            return false;
        }
        pollfd pf{fd, POLLIN, 0};
        const int pr = ::poll(&pf, 1, static_cast<int>(left.count()));
        if (pr < 0 && errno == EINTR) {
            continue;
        }
        if (pr <= 0) {
            return false;
        }
        const ssize_t r = ::recv(fd, p + got, n - got, 0);
        if (r <= 0) {
            if (r < 0 && errno == EINTR) {
                continue;
            }
            return false;
        }
        got += static_cast<std::size_t>(r);
    }
    return true;
}

// Receive the HELLO message and its SCM_RIGHTS fd. Returns the fd (>=0) or -1.
int recvHello(int sock, reclayer::HelloMsg& out, std::chrono::milliseconds timeout) {
    const auto t0 = std::chrono::steady_clock::now();
    for (;;) {
        const auto left =
            timeout - std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
        if (left <= 0ms) {
            return -1;
        }
        pollfd pf{sock, POLLIN, 0};
        const int pr = ::poll(&pf, 1, static_cast<int>(left.count()));
        if (pr < 0 && errno == EINTR) {
            continue;
        }
        if (pr <= 0) {
            return -1;
        }
        iovec io{};
        io.iov_base = &out;
        io.iov_len = sizeof(out);
        char cbuf[CMSG_SPACE(sizeof(int))];
        std::memset(cbuf, 0, sizeof(cbuf));
        msghdr msg{};
        msg.msg_iov = &io;
        msg.msg_iovlen = 1;
        msg.msg_control = cbuf;
        msg.msg_controllen = sizeof(cbuf);
        const ssize_t n = ::recvmsg(sock, &msg, 0);
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n < static_cast<ssize_t>(sizeof(out))) {
            return -1;
        }
        if (out.magic != reclayer::kCaptureMagic || out.version != reclayer::kCaptureVersion ||
            out.type != static_cast<std::uint32_t>(reclayer::MsgType::Hello)) {
            return -1;
        }
        for (cmsghdr* cm = CMSG_FIRSTHDR(&msg); cm != nullptr; cm = CMSG_NXTHDR(&msg, cm)) {
            if (cm->cmsg_level == SOL_SOCKET && cm->cmsg_type == SCM_RIGHTS) {
                int fd = -1;
                std::memcpy(&fd, CMSG_DATA(cm), sizeof(int));
                return fd;
            }
        }
        return -1;
    }
}

// Connect to the capture socket, retrying until `timeout`. -1 on failure.
int connectCapture(const std::string& path, std::chrono::milliseconds timeout) {
    const auto t0 = std::chrono::steady_clock::now();
    for (;;) {
        const int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
        if (fd < 0) {
            return -1;
        }
        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
        if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0) {
            return fd;
        }
        ::close(fd);
        if (std::chrono::steady_clock::now() - t0 > timeout || g_stop.load()) {
            return -1;
        }
        std::this_thread::sleep_for(200ms);
    }
}

} // namespace

#ifdef PIBRIDGE_HAVE_VULKAN
namespace {

// The consumer's Vulkan device: imports the layer's OPAQUE_FD image and reads it back to a host buffer.
class CaptureClient {
public:
    ~CaptureClient() { cleanup(); }

    // Connect, receive HELLO+fd, import into Vulkan, set up the readback buffer. Empty on success, else why.
    std::string open(const std::string& sockPath, std::chrono::milliseconds timeout) {
        sock_ = connectCapture(sockPath, timeout);
        if (sock_ < 0) {
            return "the capture layer is not reachable (is SteamVR running with the pibridge record layer loaded?)";
        }
        imageFd_ = recvHello(sock_, hello_, timeout);
        if (imageFd_ < 0) {
            return "the capture layer did not send a shared image (recording may not be active, or the layer is not "
                   "loaded in vrcompositor)";
        }
        if (std::string e = initVulkan(); !e.empty()) {
            return e;
        }
        return {};
    }

    std::uint32_t width() const { return hello_.width; }
    std::uint32_t height() const { return hello_.height; }

    // Block (up to timeout) for the next frame, read it back into rgba (width*height*4), ack. 1 ok, 0 timeout, -1 gone.
    int nextFrame(std::vector<std::uint8_t>& rgba, std::uint64_t& ts, std::chrono::milliseconds timeout) {
        reclayer::FrameMsg fm{};
        if (!recvAll(sock_, &fm, sizeof(fm), timeout)) {
            // distinguish timeout (socket still open) from a closed socket
            pollfd pf{sock_, POLLIN, 0};
            if (::poll(&pf, 1, 0) > 0 && (pf.revents & (POLLHUP | POLLERR)) != 0) {
                return -1;
            }
            return 0;
        }
        if (fm.magic != reclayer::kCaptureMagic || fm.type != static_cast<std::uint32_t>(reclayer::MsgType::Frame)) {
            return -1;
        }
        ts = fm.timestamp_ns;
        if (std::string e = readback(rgba); !e.empty()) {
            std::fprintf(stderr, "pibridge-record: capture readback: %s\n", e.c_str());
            return -1;
        }
        const std::uint8_t ack = reclayer::kAckByte;
        (void)!::send(sock_, &ack, 1, MSG_NOSIGNAL);
        return 1;
    }

private:
    std::string initVulkan() {
        VkApplicationInfo ai{};
        ai.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
        ai.pApplicationName = "pibridge-record-capture";
        ai.apiVersion = VK_API_VERSION_1_1;
        VkInstanceCreateInfo ici{};
        ici.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
        ici.pApplicationInfo = &ai;
        if (vkCreateInstance(&ici, nullptr, &inst_) != VK_SUCCESS) {
            return "cannot create a Vulkan instance";
        }
        std::uint32_t count = 0;
        vkEnumeratePhysicalDevices(inst_, &count, nullptr);
        std::vector<VkPhysicalDevice> devs(count);
        vkEnumeratePhysicalDevices(inst_, &count, devs.data());
        for (VkPhysicalDevice d : devs) {
            VkPhysicalDeviceIDProperties idp{};
            idp.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ID_PROPERTIES;
            VkPhysicalDeviceProperties2 p2{};
            p2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
            p2.pNext = &idp;
            vkGetPhysicalDeviceProperties2(d, &p2);
            if (std::memcmp(idp.deviceUUID, hello_.deviceUUID, sizeof(idp.deviceUUID)) == 0) {
                phys_ = d;
                break;
            }
        }
        if (phys_ == VK_NULL_HANDLE && !devs.empty()) {
            phys_ = devs[0]; // the layer's GPU was not matched by UUID; best effort on the first device
        }
        if (phys_ == VK_NULL_HANDLE) {
            return "no Vulkan device to import the shared image into";
        }
        // a queue family that can do transfers (graphics implies transfer)
        std::uint32_t qf = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(phys_, &qf, nullptr);
        std::vector<VkQueueFamilyProperties> qfs(qf);
        vkGetPhysicalDeviceQueueFamilyProperties(phys_, &qf, qfs.data());
        family_ = UINT32_MAX;
        for (std::uint32_t i = 0; i < qfs.size(); i++) {
            if ((qfs[i].queueFlags & (VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_TRANSFER_BIT | VK_QUEUE_COMPUTE_BIT)) != 0) {
                family_ = i;
                break;
            }
        }
        if (family_ == UINT32_MAX) {
            return "no transfer-capable queue family";
        }
        const float prio = 1.0f;
        VkDeviceQueueCreateInfo qci{};
        qci.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        qci.queueFamilyIndex = family_;
        qci.queueCount = 1;
        qci.pQueuePriorities = &prio;
        // VK_KHR_external_memory is core in 1.1; only the fd extension stays an extension. Enabling the promoted one
        // can be rejected by some drivers, so enable just VK_KHR_external_memory_fd.
        const char* exts[] = {VK_KHR_EXTERNAL_MEMORY_FD_EXTENSION_NAME};
        VkDeviceCreateInfo dci{};
        dci.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
        dci.queueCreateInfoCount = 1;
        dci.pQueueCreateInfos = &qci;
        dci.enabledExtensionCount = 1;
        dci.ppEnabledExtensionNames = exts;
        if (vkCreateDevice(phys_, &dci, nullptr, &dev_) != VK_SUCCESS) {
            return "cannot create a Vulkan device with VK_KHR_external_memory_fd";
        }
        vkGetDeviceQueue(dev_, family_, 0, &queue_);

        // Import the shared image: an identically-created VkImage bound to imported memory.
        VkExternalMemoryImageCreateInfo ext{};
        ext.sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO;
        ext.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT;
        VkImageCreateInfo imgci{};
        imgci.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        imgci.pNext = &ext;
        imgci.imageType = VK_IMAGE_TYPE_2D;
        imgci.format = static_cast<VkFormat>(hello_.vkFormat);
        imgci.extent = VkExtent3D{hello_.width, hello_.height, 1};
        imgci.mipLevels = 1;
        imgci.arrayLayers = 1;
        imgci.samples = VK_SAMPLE_COUNT_1_BIT;
        imgci.tiling = static_cast<VkImageTiling>(hello_.vkImageTiling);
        imgci.usage = hello_.vkImageUsage;
        imgci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        imgci.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        if (vkCreateImage(dev_, &imgci, nullptr, &img_) != VK_SUCCESS) {
            return "cannot create the import image";
        }
        VkImportMemoryFdInfoKHR imp{};
        imp.sType = VK_STRUCTURE_TYPE_IMPORT_MEMORY_FD_INFO_KHR;
        imp.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT;
        imp.fd = imageFd_; // ownership passes to Vulkan on success
        VkMemoryAllocateInfo mai{};
        mai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        mai.pNext = &imp;
        mai.allocationSize = hello_.allocationSize;
        mai.memoryTypeIndex = hello_.memoryTypeIndex;
        if (vkAllocateMemory(dev_, &mai, nullptr, &mem_) != VK_SUCCESS) {
            return "cannot import the shared memory (fd)";
        }
        imageFd_ = -1; // consumed by vkAllocateMemory
        if (vkBindImageMemory(dev_, img_, mem_, 0) != VK_SUCCESS) {
            return "cannot bind the imported image";
        }

        // The host-visible readback buffer (tightly packed RGBA).
        bufSize_ = static_cast<VkDeviceSize>(hello_.width) * hello_.height * 4u;
        VkBufferCreateInfo bci{};
        bci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bci.size = bufSize_;
        bci.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
        bci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        if (vkCreateBuffer(dev_, &bci, nullptr, &buf_) != VK_SUCCESS) {
            return "cannot create the readback buffer";
        }
        VkMemoryRequirements mr{};
        vkGetBufferMemoryRequirements(dev_, buf_, &mr);
        VkPhysicalDeviceMemoryProperties mp{};
        vkGetPhysicalDeviceMemoryProperties(phys_, &mp);
        int hostType = -1;
        for (std::uint32_t i = 0; i < mp.memoryTypeCount; i++) {
            const bool allowed = (mr.memoryTypeBits & (1u << i)) != 0;
            const auto want = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
            if (allowed && (mp.memoryTypes[i].propertyFlags & want) == want) {
                hostType = static_cast<int>(i);
                break;
            }
        }
        if (hostType < 0) {
            return "no host-visible coherent memory for the readback buffer";
        }
        VkMemoryAllocateInfo bmi{};
        bmi.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        bmi.allocationSize = mr.size;
        bmi.memoryTypeIndex = static_cast<std::uint32_t>(hostType);
        if (vkAllocateMemory(dev_, &bmi, nullptr, &bufMem_) != VK_SUCCESS) {
            return "cannot allocate the readback buffer memory";
        }
        vkBindBufferMemory(dev_, buf_, bufMem_, 0);
        if (vkMapMemory(dev_, bufMem_, 0, bufSize_, 0, &bufMap_) != VK_SUCCESS) {
            return "cannot map the readback buffer";
        }

        VkCommandPoolCreateInfo pci{};
        pci.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        pci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        pci.queueFamilyIndex = family_;
        if (vkCreateCommandPool(dev_, &pci, nullptr, &pool_) != VK_SUCCESS) {
            return "cannot create the command pool";
        }
        VkCommandBufferAllocateInfo cbi{};
        cbi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        cbi.commandPool = pool_;
        cbi.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        cbi.commandBufferCount = 1;
        vkAllocateCommandBuffers(dev_, &cbi, &cmd_);
        VkFenceCreateInfo fci{};
        fci.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        vkCreateFence(dev_, &fci, nullptr, &fence_);
        return {};
    }

    std::string readback(std::vector<std::uint8_t>& rgba) {
        vkResetCommandBuffer(cmd_, 0);
        VkCommandBufferBeginInfo bi{};
        bi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        vkBeginCommandBuffer(cmd_, &bi);
        // The layer left the shared image in TRANSFER_SRC_OPTIMAL. Make its writes visible to our transfer read. No
        // queue-family ownership transfer (IGNORED): the happens-before is the layer's fence + socket ack round-trip
        // (see tools/record-layer/src/layer.cpp, iteration #2). ON-HARDWARE: if frames are stale/garbage, the fix is a
        // shared EXTERNAL timeline semaphore from the layer's blit, waited on here, in place of the ack round-trip.
        VkImageMemoryBarrier acq{};
        acq.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        acq.srcAccessMask = 0;
        acq.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        acq.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        acq.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        acq.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        acq.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        acq.image = img_;
        acq.subresourceRange = VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        vkCmdPipelineBarrier(cmd_, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0,
                             nullptr, 1, &acq);
        VkBufferImageCopy region{};
        region.bufferOffset = 0;
        region.bufferRowLength = 0;   // tightly packed
        region.bufferImageHeight = 0;
        region.imageSubresource = VkImageSubresourceLayers{VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        region.imageOffset = VkOffset3D{0, 0, 0};
        region.imageExtent = VkExtent3D{hello_.width, hello_.height, 1};
        vkCmdCopyImageToBuffer(cmd_, img_, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, buf_, 1, &region);
        vkEndCommandBuffer(cmd_);
        VkSubmitInfo si{};
        si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        si.commandBufferCount = 1;
        si.pCommandBuffers = &cmd_;
        vkResetFences(dev_, 1, &fence_);
        if (vkQueueSubmit(queue_, 1, &si, fence_) != VK_SUCCESS) {
            return "vkQueueSubmit(copy) failed";
        }
        if (vkWaitForFences(dev_, 1, &fence_, VK_TRUE, 1000000000ull) != VK_SUCCESS) {
            return "readback timed out";
        }
        rgba.resize(static_cast<std::size_t>(bufSize_));
        std::memcpy(rgba.data(), bufMap_, static_cast<std::size_t>(bufSize_)); // coherent: no invalidate needed
        return {};
    }

    void cleanup() {
        if (dev_ != VK_NULL_HANDLE) {
            vkDeviceWaitIdle(dev_);
            if (fence_ != VK_NULL_HANDLE) {
                vkDestroyFence(dev_, fence_, nullptr);
            }
            if (pool_ != VK_NULL_HANDLE) {
                vkDestroyCommandPool(dev_, pool_, nullptr);
            }
            if (bufMap_ != nullptr) {
                vkUnmapMemory(dev_, bufMem_);
            }
            if (buf_ != VK_NULL_HANDLE) {
                vkDestroyBuffer(dev_, buf_, nullptr);
            }
            if (bufMem_ != VK_NULL_HANDLE) {
                vkFreeMemory(dev_, bufMem_, nullptr);
            }
            if (img_ != VK_NULL_HANDLE) {
                vkDestroyImage(dev_, img_, nullptr);
            }
            if (mem_ != VK_NULL_HANDLE) {
                vkFreeMemory(dev_, mem_, nullptr);
            }
            vkDestroyDevice(dev_, nullptr);
        }
        if (inst_ != VK_NULL_HANDLE) {
            vkDestroyInstance(inst_, nullptr);
        }
        if (imageFd_ >= 0) {
            ::close(imageFd_);
        }
        if (sock_ >= 0) {
            ::close(sock_);
        }
    }

    int sock_ = -1;
    int imageFd_ = -1;
    reclayer::HelloMsg hello_{};

    VkInstance inst_ = VK_NULL_HANDLE;
    VkPhysicalDevice phys_ = VK_NULL_HANDLE;
    VkDevice dev_ = VK_NULL_HANDLE;
    VkQueue queue_ = VK_NULL_HANDLE;
    std::uint32_t family_ = 0;
    VkImage img_ = VK_NULL_HANDLE;
    VkDeviceMemory mem_ = VK_NULL_HANDLE;
    VkBuffer buf_ = VK_NULL_HANDLE;
    VkDeviceMemory bufMem_ = VK_NULL_HANDLE;
    VkDeviceSize bufSize_ = 0;
    void* bufMap_ = nullptr;
    VkCommandPool pool_ = VK_NULL_HANDLE;
    VkCommandBuffer cmd_ = VK_NULL_HANDLE;
    VkFence fence_ = VK_NULL_HANDLE;
};

} // namespace

int runLayer(const Settings& s, const Roots& r) {
    beNiceLayer();
    struct sigaction sa{};
    sa.sa_handler = onLayerSignal;
    sa.sa_flags = SA_RESTART;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    signal(SIGPIPE, SIG_IGN);

    std::error_code ec;
    fs::create_directories(r.session, ec);
    const fs::path stPath = statusPath(r.session);
    Status st;
    st.state = "starting";
    st.pid = static_cast<std::uint32_t>(getpid());
    st.started_unix = unixNow();
    st.eyes = s.view == View::BothEyes ? "both" : "left";
    st.fps = s.fps;
    st.quality = name(s.quality);
    st.source = "layer";
    const auto save = [&] {
        st.updated_unix = unixNow();
        st.save(stPath);
    };
    const auto fail = [&](const std::string& why) {
        st.state = "error";
        st.note = why;
        save();
        event(r.session, "critical", why, true);
        return 0;
    };
    save();

    if (pibridge::env("RECORD_NO_MONADO_CHECK") == nullptr && !steamvrRunning(r)) {
        return fail("no VR session (SteamVR is not running): the Vulkan capture layer runs inside SteamVR");
    }
    const fs::path ffmpeg = ffmpegPath();
    const auto avail = Available::probe(ffmpeg);
    if (!avail) {
        return fail(std::format("cannot run {} (install ffmpeg)", ffmpeg.string()));
    }
    auto planned = plan(s.encoder, s.codec, *avail, true);
    if (!planned) {
        return fail(planned.error());
    }
    std::vector<Encoder> encs = *planned;
    const fs::path dir = s.outDir(r.home);
    fs::create_directories(dir, ec);
    if (ec) {
        return fail(std::format("cannot create {}: {}", dir.string(), ec.message()));
    }
    if (const auto f = freeBytes(dir); f && *f < kStartFree) {
        return fail(std::format("less than 2 GiB free in {}", dir.string()));
    }

    CaptureClient client;
    if (std::string e = client.open(reclayer::captureSocketPath(), 15s); !e.empty()) {
        return fail(e);
    }
    const std::uint32_t w = client.width();
    const std::uint32_t h = client.height();
    if (w == 0 || h == 0) {
        return fail("the capture layer reported a zero-sized image");
    }

    // Sound, like the tap/mirror path: a missing output or mic never stops the video.
    Settings es = s;
    std::string audioNote;
    if (es.audio) {
        if (const auto e = probeAudio(ffmpeg, kMonitorSrc)) {
            es.audio = false;
            es.mic = false;
            audioNote = std::format("recording without sound: the audio output cannot be read ({})", *e);
        } else if (es.mic) {
            if (const auto m = probeAudio(ffmpeg, kMicSrc)) {
                es.mic = false;
                audioNote = std::format("the microphone cannot be read, recording the output sound only ({})", *m);
            }
        }
    }

    const fs::path out = uniquePath(dir, fileName(static_cast<std::int64_t>(unixNow())));
    const fs::path ffLog = r.session / "record-ffmpeg.log";
    std::size_t encI = 0;
    const auto spawn = [&](Encoder e) { return Ffmpeg::spawn(ffmpeg, e, es, w, h, out, ffLog); };
    auto first = spawn(encs[0]);
    if (!first) {
        return fail(std::format("cannot start ffmpeg: {}", first.error()));
    }
    std::optional<Ffmpeg> ff(std::move(*first));
    st.state = "recording";
    st.file = out.string();
    st.encoder = encoderName(encs[0], s.codec);
    st.audio = !es.audio ? "" : es.mic ? "output+mic" : "output";
    st.width = w;
    st.height = h;
    save();
    if (!audioNote.empty()) {
        event(r.session, "notice", audioNote, true);
    }
    event(r.session, "notice", std::format("recording to {} (Vulkan capture layer)", out.string()), true);

    Pacer pacer(s.fps);
    std::string reason = "stopped";
    bool critical = false;
    auto lastStatus = std::chrono::steady_clock::now();
    std::vector<std::uint8_t> frame;
    std::uint64_t idleStart = 0;
    for (;;) {
        if (g_stop.load()) {
            break;
        }
        std::uint64_t ts = 0;
        const int got = client.nextFrame(frame, ts, 1000ms);
        if (got < 0) {
            reason = "the capture layer stopped (SteamVR or the compositor ended)";
            break;
        }
        if (got == 0) {
            // no frame within a second: the app may be idle (menus) — not an error for a while
            if (idleStart == 0) {
                idleStart = nowNsMono();
            } else if (nowNsMono() - idleStart > 30000000000ull) {
                reason = "no frames for 30 s (the capture did not produce images; is a VR app rendering?)";
                break;
            }
        } else {
            idleStart = 0;
        }

        if (std::optional<std::string> how = ff->exited(); how) {
            if (std::chrono::steady_clock::now() - ff->started < 5s && encI + 1 < encs.size()) {
                encI++;
                fs::remove(out, ec);
                event(r.session, "notice",
                      std::format("{} failed ({}), falling back to {}", encoderName(ff->encoder, s.codec), *how,
                                  encoderName(encs[encI], s.codec)),
                      false);
                auto next = spawn(encs[encI]);
                if (next) {
                    ff.emplace(std::move(*next));
                    st.encoder = encoderName(encs[encI], s.codec);
                    pacer = Pacer(s.fps);
                    continue;
                }
                reason = std::format("cannot start ffmpeg: {}", next.error());
                critical = true;
                break;
            }
            reason = std::format("the encoder stopped ({}; see {})", *how, ffLog.string());
            critical = true;
            break;
        }

        if (got == 1) {
            const std::uint64_t n = pacer.copies(ts);
            for (std::uint64_t i = 0; i < n; i++) {
                if (!ff->write(frame)) {
                    reason = "the encoder closed its input";
                    critical = true;
                    break;
                }
            }
            if (critical) {
                break;
            }
        }

        if (std::chrono::steady_clock::now() - lastStatus >= 1s) {
            lastStatus = std::chrono::steady_clock::now();
            if (const auto f = freeBytes(dir); f && *f < kStopFree) {
                reason = std::format("the disk is almost full (less than 1 GiB free in {})", dir.string());
                critical = true;
                break;
            }
            st.frames = ff->frames;
            st.duplicated = pacer.duplicated;
            save();
        }
    }

    st.state = "finalizing";
    save();
    const std::uint64_t frames = ff->frames;
    const std::string how = ff->finish(15s);
    st.frames = frames;
    st.duplicated = pacer.duplicated;
    const auto size = fs::file_size(out, ec);
    const double secs = double(frames) / double(s.fps);
    std::string text = std::format("saved {} ({:.0f} s, {:.1f} MB; {} repeated){}", out.string(), secs,
                                   double(ec ? 0 : size) / 1e6, pacer.duplicated,
                                   how == "ok" ? std::string() : "; encoder: " + how);
    if (reason != "stopped") {
        text = reason + ": " + text;
    }
    if (frames == 0) {
        fs::remove(out, ec);
        text = reason + ": nothing recorded";
        st.file.clear();
    }
    st.state = critical ? "error" : "idle";
    st.note = reason == "stopped" ? std::string() : reason;
    save();
    event(r.session, critical ? "critical" : "notice", text, true);
    return 0;
}

} // namespace pibridge::record

#else // !PIBRIDGE_HAVE_VULKAN

namespace pibridge::record {
int runLayer(const Settings&, const Roots& r) {
    Status st;
    st.state = "error";
    st.pid = static_cast<std::uint32_t>(getpid());
    st.source = "layer";
    st.note = "this build has no Vulkan capture support (the record layer path is unavailable)";
    st.started_unix = unixNow();
    st.updated_unix = st.started_unix;
    st.save(statusPath(r.session));
    event(r.session, "critical", st.note, true);
    return 0;
}
} // namespace pibridge::record

#endif
