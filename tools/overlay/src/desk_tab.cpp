// SPDX-License-Identifier: MIT
#include <vulkan/vulkan.h>

#include "desk_tab.h"

#include "dash_paint.h"
#include "dash_settings.h"
#include "dash_view.h"
#include "frame_layout.h"
#include "overlay_core.h"
#include "pibridge_desktop.h"
#include "pibridge_record_capture_control.h"
#include "pibridge_status.h"

#include <QImage>
#include <QPainter>
#include <QPainterPath>

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <format>
#include <fstream>
#include <iterator>
#include <sstream>
#include <vector>

namespace pibridge::overlay {

namespace dash = pibridge::home::dash;
namespace frame = pibridge::home::frame;

namespace {

using Clock = std::chrono::steady_clock;

template <class T>
T vk(VkStructureType t) {
    T v{};
    v.sType = t;
    return v;
}
constexpr VkFormat kFormat = VK_FORMAT_B8G8R8A8_SRGB; // the desktop's bytes and the bar's: BGRA, sRGB

std::vector<std::string> splitWords(const std::string& s) {
    std::istringstream in(s);
    std::vector<std::string> out;
    for (std::string w; in >> w;) {
        out.push_back(w);
    }
    return out;
}

std::optional<std::uint32_t> memType(const VkPhysicalDeviceMemoryProperties& p, std::uint32_t bits, VkMemoryPropertyFlags f) {
    for (std::uint32_t i = 0; i < p.memoryTypeCount; i++) {
        if ((bits & (1U << i)) != 0 && (p.memoryTypes[i].propertyFlags & f) == f) {
            return i;
        }
    }
    return std::nullopt;
}

// The tab's icon in SteamVR's dashboard bar: a monitor, drawn in code (128 x 128 RGBA, transparent around it).
std::vector<std::uint8_t> monitorIcon() {
    QImage img(static_cast<int>(kIconSize), static_cast<int>(kIconSize), QImage::Format_RGBA8888);
    img.fill(Qt::transparent);
    QPainter p(&img);
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(QPen(QColor(234, 240, 248), 9, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.drawRoundedRect(QRectF(12, 18, 104, 68), 8, 8);
    p.drawLine(QPointF(64, 86), QPointF(64, 106));
    p.drawLine(QPointF(40, 110), QPointF(88, 110));
    p.end();
    const auto* b = img.constBits();
    return std::vector<std::uint8_t>(b, b + static_cast<std::size_t>(img.sizeInBytes()));
}

bool processAlive(const char* comm15) {
    std::error_code ec;
    for (const auto& e : std::filesystem::directory_iterator("/proc", ec)) {
        const std::string n = e.path().filename().string();
        if (n.empty() || !std::isdigit(static_cast<unsigned char>(n[0]))) {
            continue;
        }
        std::ifstream f(e.path() / "comm");
        std::string c;
        std::getline(f, c);
        if (c == comm15) {
            return true;
        }
    }
    return false;
}

void writeRuntimeFile(const char* name, const std::string& text) {
    const auto dir = status::runtime_dir();
    const auto tmp = dir / (std::string(".") + name + ".tmp");
    {
        std::ofstream f(tmp, std::ios::trunc);
        f << text << '\n';
    }
    std::error_code ec;
    std::filesystem::rename(tmp, dir / name, ec);
}

struct Image {
    VkImage img = VK_NULL_HANDLE;
    VkDeviceMemory mem = VK_NULL_HANDLE;
    std::uint32_t w = 0, h = 0;
    VkImageLayout layout = VK_IMAGE_LAYOUT_UNDEFINED;
};
struct Buffer {
    VkBuffer buf = VK_NULL_HANDLE;
    VkDeviceMemory mem = VK_NULL_HANDLE;
    void* ptr = nullptr;
    VkDeviceSize size = 0;
};

} // namespace

struct DeskTab::Impl {
    vr::IVROverlay* ov = nullptr;
    vr::IVRSystem* sys = nullptr;
    vr::IVRCompositor* comp = nullptr;
    vr::VROverlayHandle_t main = vr::k_ulOverlayHandleInvalid; // the tab: the bar
    vr::VROverlayHandle_t thumb = vr::k_ulOverlayHandleInvalid;
    std::array<vr::VROverlayHandle_t, desktop::kMaxMonitors> monOv{}; // the monitors, in the room
    std::vector<std::uint8_t> icon;

    // ---- the desktop
    desktop::Link link;
    std::string tool = desktop::toolPath();
    bool on = false;          // the monitors are shown (the bar's Desktop: a second press hides them)
    std::uint32_t mask = 0;
    std::vector<desktop::MonitorInfo> mons;
    room::Mat34 head;         // where the head was when they were placed
    double zoom = 1.0, distance = 0.0;
    std::string uiScale = "Medium";
    std::array<std::uint64_t, desktop::kMaxMonitors> seq{};
    std::vector<std::uint8_t> pix;
    desktop::Pointer pointer;
    std::uint32_t buttons = 0;
    std::int64_t wheel = 0;
    double wheelAcc = 0;
    Clock::time_point startedAt{};
    Clock::time_point saveAt{};
    bool saveDue = false;

    // ---- the bar
    dash::Painter painter;
    std::string painted;
    std::string over;
    std::string downOn;
    std::string message;
    bool recording = false;
    bool headtrack = false;
    Clock::time_point stateAt{};

    // ---- Vulkan
    bool gpuTried = false, gpuOk = false;
    VkInstance inst = VK_NULL_HANDLE;
    VkPhysicalDevice pd = VK_NULL_HANDLE;
    VkDevice dev = VK_NULL_HANDLE;
    VkQueue queue = VK_NULL_HANDLE;
    std::uint32_t qf = 0;
    VkPhysicalDeviceMemoryProperties mp{};
    VkCommandPool pool = VK_NULL_HANDLE;
    VkCommandBuffer cmd = VK_NULL_HANDLE;
    VkFence fence = VK_NULL_HANDLE;
    // two images a monitor and two for the bar: SteamVR may still read the last one handed over
    std::array<std::array<Image, 2>, desktop::kMaxMonitors> monImg;
    std::array<int, desktop::kMaxMonitors> monCur{};
    std::array<Buffer, desktop::kMaxMonitors> monBuf;
    std::array<Image, 2> barImg;
    int barCur = 0;
    Buffer barBuf;

    ~Impl() { destroyGpu(); }

    // ---------------------------------------------------------------- Vulkan

    bool initGpu(std::string& err) {
        gpuTried = true;
        std::vector<std::string> ie;
        if (comp != nullptr) {
            char buf[2048] = {0};
            comp->GetVulkanInstanceExtensionsRequired(buf, sizeof buf);
            ie = splitWords(buf);
        }
        std::vector<const char*> iep;
        for (const auto& e : ie) {
            iep.push_back(e.c_str());
        }
        VkApplicationInfo ai{VK_STRUCTURE_TYPE_APPLICATION_INFO, nullptr, "pibridge-overlay", 1, "pibridge", 1, VK_API_VERSION_1_1};
        auto ici = vk<VkInstanceCreateInfo>(VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO);
        ici.pApplicationInfo = &ai;
        ici.enabledExtensionCount = static_cast<std::uint32_t>(iep.size());
        ici.ppEnabledExtensionNames = iep.data();
        if (vkCreateInstance(&ici, nullptr, &inst) != VK_SUCCESS) {
            err = "no Vulkan instance";
            return false;
        }
        std::uint64_t out = 0;
        if (sys != nullptr) {
            sys->GetOutputDevice(&out, vr::TextureType_Vulkan, reinterpret_cast<VkInstance_T*>(inst));
        }
        pd = reinterpret_cast<VkPhysicalDevice>(out);
        if (pd == VK_NULL_HANDLE) {
            std::uint32_t n = 0;
            vkEnumeratePhysicalDevices(inst, &n, nullptr);
            std::vector<VkPhysicalDevice> v(n);
            vkEnumeratePhysicalDevices(inst, &n, v.data());
            if (v.empty()) {
                err = "no Vulkan device";
                return false;
            }
            pd = v[0];
        }
        vkGetPhysicalDeviceMemoryProperties(pd, &mp);
        std::uint32_t nq = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(pd, &nq, nullptr);
        std::vector<VkQueueFamilyProperties> qs(nq);
        vkGetPhysicalDeviceQueueFamilyProperties(pd, &nq, qs.data());
        bool found = false;
        for (std::uint32_t i = 0; i < nq; i++) {
            if ((qs[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0) {
                qf = i;
                found = true;
                break;
            }
        }
        if (!found) {
            err = "no graphics queue";
            return false;
        }
        std::vector<std::string> de;
        if (comp != nullptr) {
            char buf[2048] = {0};
            comp->GetVulkanDeviceExtensionsRequired(reinterpret_cast<VkPhysicalDevice_T*>(pd), buf, sizeof buf);
            de = splitWords(buf);
        }
        std::vector<const char*> dep;
        for (const auto& e : de) {
            dep.push_back(e.c_str());
        }
        const float prio = 1.0F;
        VkDeviceQueueCreateInfo qci{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO, nullptr, 0, qf, 1, &prio};
        auto dci = vk<VkDeviceCreateInfo>(VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO);
        dci.queueCreateInfoCount = 1;
        dci.pQueueCreateInfos = &qci;
        dci.enabledExtensionCount = static_cast<std::uint32_t>(dep.size());
        dci.ppEnabledExtensionNames = dep.data();
        if (vkCreateDevice(pd, &dci, nullptr, &dev) != VK_SUCCESS) {
            err = "no Vulkan device (its extensions)";
            return false;
        }
        vkGetDeviceQueue(dev, qf, 0, &queue);
        VkCommandPoolCreateInfo pci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO, nullptr,
                                    VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT, qf};
        vkCreateCommandPool(dev, &pci, nullptr, &pool);
        VkCommandBufferAllocateInfo cai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO, nullptr, pool,
                                        VK_COMMAND_BUFFER_LEVEL_PRIMARY, 1};
        vkAllocateCommandBuffers(dev, &cai, &cmd);
        auto fci = vk<VkFenceCreateInfo>(VK_STRUCTURE_TYPE_FENCE_CREATE_INFO);
        vkCreateFence(dev, &fci, nullptr, &fence);
        gpuOk = true;
        return true;
    }

    void freeImage(Image& i) {
        if (i.img != VK_NULL_HANDLE) {
            vkDestroyImage(dev, i.img, nullptr);
        }
        if (i.mem != VK_NULL_HANDLE) {
            vkFreeMemory(dev, i.mem, nullptr);
        }
        i = Image{};
    }
    void freeBuffer(Buffer& b) {
        if (b.buf != VK_NULL_HANDLE) {
            vkDestroyBuffer(dev, b.buf, nullptr);
        }
        if (b.mem != VK_NULL_HANDLE) {
            vkFreeMemory(dev, b.mem, nullptr);
        }
        b = Buffer{};
    }
    void destroyGpu() {
        if (dev != VK_NULL_HANDLE) {
            vkDeviceWaitIdle(dev);
            for (auto& two : monImg) {
                for (auto& i : two) {
                    freeImage(i);
                }
            }
            for (auto& b : monBuf) {
                freeBuffer(b);
            }
            for (auto& i : barImg) {
                freeImage(i);
            }
            freeBuffer(barBuf);
            if (fence != VK_NULL_HANDLE) {
                vkDestroyFence(dev, fence, nullptr);
            }
            if (pool != VK_NULL_HANDLE) {
                vkDestroyCommandPool(dev, pool, nullptr);
            }
            vkDestroyDevice(dev, nullptr);
        }
        if (inst != VK_NULL_HANDLE) {
            vkDestroyInstance(inst, nullptr);
        }
        dev = VK_NULL_HANDLE;
        inst = VK_NULL_HANDLE;
    }

    bool makeImage(Image& i, std::uint32_t w, std::uint32_t h) {
        if (i.img != VK_NULL_HANDLE && i.w == w && i.h == h) {
            return true;
        }
        vkDeviceWaitIdle(dev);
        freeImage(i);
        auto ci = vk<VkImageCreateInfo>(VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO);
        ci.imageType = VK_IMAGE_TYPE_2D;
        ci.format = kFormat;
        ci.extent = {w, h, 1};
        ci.mipLevels = 1;
        ci.arrayLayers = 1;
        ci.samples = VK_SAMPLE_COUNT_1_BIT;
        ci.tiling = VK_IMAGE_TILING_OPTIMAL;
        ci.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        ci.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        if (vkCreateImage(dev, &ci, nullptr, &i.img) != VK_SUCCESS) {
            return false;
        }
        VkMemoryRequirements req;
        vkGetImageMemoryRequirements(dev, i.img, &req);
        const auto t = memType(mp, req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO, nullptr, req.size, t.value_or(0)};
        if (!t || vkAllocateMemory(dev, &mai, nullptr, &i.mem) != VK_SUCCESS || vkBindImageMemory(dev, i.img, i.mem, 0) != VK_SUCCESS) {
            freeImage(i);
            return false;
        }
        i.w = w;
        i.h = h;
        i.layout = VK_IMAGE_LAYOUT_UNDEFINED;
        return true;
    }
    bool makeBuffer(Buffer& b, VkDeviceSize size) {
        if (b.buf != VK_NULL_HANDLE && b.size >= size) {
            return true;
        }
        vkDeviceWaitIdle(dev);
        freeBuffer(b);
        auto bi = vk<VkBufferCreateInfo>(VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO);
        bi.size = size;
        bi.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
        bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        if (vkCreateBuffer(dev, &bi, nullptr, &b.buf) != VK_SUCCESS) {
            return false;
        }
        VkMemoryRequirements req;
        vkGetBufferMemoryRequirements(dev, b.buf, &req);
        const auto t = memType(mp, req.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO, nullptr, req.size, t.value_or(0)};
        if (!t || vkAllocateMemory(dev, &mai, nullptr, &b.mem) != VK_SUCCESS || vkBindBufferMemory(dev, b.buf, b.mem, 0) != VK_SUCCESS ||
            vkMapMemory(dev, b.mem, 0, VK_WHOLE_SIZE, 0, &b.ptr) != VK_SUCCESS) {
            freeBuffer(b);
            return false;
        }
        b.size = size;
        return true;
    }

    void barrier(Image& i, VkImageLayout to) {
        auto b = vk<VkImageMemoryBarrier>(VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER);
        b.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT | VK_ACCESS_TRANSFER_READ_BIT;
        b.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT | VK_ACCESS_TRANSFER_READ_BIT;
        b.oldLayout = i.layout;
        b.newLayout = to;
        b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.image = i.img;
        b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &b);
        i.layout = to;
    }
    void copyIn(Image& i, const Buffer& b) {
        barrier(i, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        VkBufferImageCopy c{};
        c.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        c.imageExtent = {i.w, i.h, 1};
        vkCmdCopyBufferToImage(cmd, b.buf, i.img, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &c);
        barrier(i, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
    }

    /// Copy `b` into `img` (made at w x h), ready for SteamVR (TRANSFER_SRC), and hand it to overlay `h`.
    void present(vr::VROverlayHandle_t h, Image& img, const Buffer& b, std::uint32_t w, std::uint32_t hgt) {
        if (!makeImage(img, w, hgt)) {
            return;
        }
        vkResetCommandBuffer(cmd, 0);
        auto bbi = vk<VkCommandBufferBeginInfo>(VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO);
        bbi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        vkBeginCommandBuffer(cmd, &bbi);
        copyIn(img, b); // ends in TRANSFER_SRC_OPTIMAL: what SteamVR takes a Vulkan texture in
        vkEndCommandBuffer(cmd);
        auto si = vk<VkSubmitInfo>(VK_STRUCTURE_TYPE_SUBMIT_INFO);
        si.commandBufferCount = 1;
        si.pCommandBuffers = &cmd;
        vkResetFences(dev, 1, &fence);
        if (vkQueueSubmit(queue, 1, &si, fence) != VK_SUCCESS) {
            return;
        }
        vkWaitForFences(dev, 1, &fence, VK_TRUE, 1'000'000'000ULL);
        vr::VRVulkanTextureData_t vt{};
        vt.m_nImage = reinterpret_cast<std::uint64_t>(img.img);
        vt.m_pDevice = reinterpret_cast<VkDevice_T*>(dev);
        vt.m_pPhysicalDevice = reinterpret_cast<VkPhysicalDevice_T*>(pd);
        vt.m_pInstance = reinterpret_cast<VkInstance_T*>(inst);
        vt.m_pQueue = reinterpret_cast<VkQueue_T*>(queue);
        vt.m_nQueueFamilyIndex = qf;
        vt.m_nWidth = img.w;
        vt.m_nHeight = img.h;
        vt.m_nFormat = kFormat;
        vt.m_nSampleCount = 1;
        vr::Texture_t tex{&vt, vr::TextureType_Vulkan, vr::ColorSpace_Auto};
        ov->SetOverlayTexture(h, &tex);
    }

    // ---------------------------------------------------------------- settings (pibridge.conf, shared with the world)

    static std::string confPath() {
        const char* x = std::getenv("XDG_CONFIG_HOME");
        const char* home = std::getenv("HOME");
        const std::string base = x != nullptr && *x != '\0' ? x : std::string(home != nullptr ? home : "") + "/.config";
        return base + "/pibridge/pibridge.conf";
    }
    static std::string confValue(const std::string& text, const std::string& key) {
        std::istringstream in(text);
        std::string line, v;
        while (std::getline(in, line)) {
            const auto eq = line.find('=');
            if (line.empty() || line[0] == '#' || eq == std::string::npos) {
                continue;
            }
            std::string k = line.substr(0, eq);
            k.erase(k.find_last_not_of(" \t") + 1);
            if (k == key) {
                v = line.substr(eq + 1);
                v = v.substr(0, v.find('#'));
                v.erase(0, v.find_first_not_of(" \t"));
                v.erase(v.find_last_not_of(" \t") + 1);
            }
        }
        return v;
    }
    void loadSettings() {
        std::ifstream f(confPath());
        const std::string text((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        const auto num = [&](const char* k, double def) {
            // from_chars: a dot whatever the locale (the world and saveSettings write one)
            const std::string v = confValue(text, k);
            double d = def;
            const auto r = std::from_chars(v.data(), v.data() + v.size(), d);
            return r.ec == std::errc() ? d : def;
        };
        zoom = std::clamp(num("desktop_zoom", 1.0), frame::kZoomMin, frame::kZoomMax);
        distance = std::clamp(num("desktop_distance", 0.0), frame::kDistanceMin, frame::kDistanceMax);
        if (const std::string u = confValue(text, "world_ui_scale"); !u.empty()) {
            uiScale = u;
        }
    }
    void saveSettings() {
        std::ifstream f(confPath());
        std::string text((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        // std::format: a dot whatever the locale (Qt sets the user's, which may write "1,25")
        text = dash::confSet(dash::confSet(text, "desktop_zoom", std::format("{:.2f}", zoom)), "desktop_distance",
                             std::format("{:.2f}", distance));
        const std::string tmp = confPath() + ".tmp-overlay";
        {
            std::ofstream o(tmp, std::ios::trunc);
            o << text;
        }
        std::error_code ec;
        std::filesystem::rename(tmp, confPath(), ec);
    }

    // ---------------------------------------------------------------- the monitors in the room

    static room::Mat34 toMat(const vr::HmdMatrix34_t& m) {
        room::Mat34 r;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                r.m[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = m.m[i][j];
            }
        }
        return r;
    }
    static vr::HmdMatrix34_t toHmd(const room::Mat34& r) {
        vr::HmdMatrix34_t m{};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                m.m[i][j] = static_cast<float>(r.m[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
            }
        }
        return m;
    }
    bool headNow() {
        vr::TrackedDevicePose_t p{};
        if (sys == nullptr) {
            return false;
        }
        sys->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0.0F, &p, 1);
        if (!p.bPoseIsValid) {
            return false;
        }
        head = toMat(p.mDeviceToAbsoluteTracking);
        return true;
    }

    vr::VROverlayHandle_t monitorOverlay(std::uint32_t i) {
        if (monOv[i] != vr::k_ulOverlayHandleInvalid) {
            return monOv[i];
        }
        const std::string key = "pibridge.desktop.monitor" + std::to_string(i + 1);
        const std::string name = "Desktop " + std::to_string(i + 1);
        if (ov->CreateOverlay(key.c_str(), name.c_str(), &monOv[i]) != vr::VROverlayError_None) {
            monOv[i] = vr::k_ulOverlayHandleInvalid;
            return monOv[i];
        }
        ov->SetOverlayInputMethod(monOv[i], vr::VROverlayInputMethod_Mouse);
        // the laser works on them also outside the dashboard, and they stay in the dashboard too
        ov->SetOverlayFlag(monOv[i], vr::VROverlayFlags_MakeOverlaysInteractiveIfVisible, true);
        ov->SetOverlayFlag(monOv[i], vr::VROverlayFlags_SendVRDiscreteScrollEvents, true);
        ov->SetOverlayFlag(monOv[i], vr::VROverlayFlags_VisibleInDashboard, true);
        return monOv[i];
    }

    /// Place the shown monitors (the frame at `head`), show them, hide the rest.
    void placeMonitors() {
        std::vector<frame::Monitor> fm;
        for (const auto& m : mons) {
            fm.push_back({m.index, m.x, m.y, m.logicalW, m.logicalH});
        }
        std::array<bool, desktop::kMaxMonitors> used{};
        if (on) {
            for (const frame::Placed& p : frame::place(head, fm, mask, zoom, distance, uiScale)) {
                const vr::VROverlayHandle_t h = monitorOverlay(p.monitor);
                if (h == vr::k_ulOverlayHandleInvalid) {
                    continue;
                }
                used[p.monitor] = true;
                const vr::HmdMatrix34_t t = toHmd(p.pose);
                ov->SetOverlayTransformAbsolute(h, vr::TrackingUniverseStanding, &t);
                ov->SetOverlayWidthInMeters(h, static_cast<float>(p.w));
                for (const auto& m : mons) {
                    if (m.index == p.monitor && m.width > 0 && m.height > 0) {
                        const vr::HmdVector2_t scale{{static_cast<float>(m.width), static_cast<float>(m.height)}};
                        ov->SetOverlayMouseScale(h, &scale);
                    }
                }
                if (seq[p.monitor] != 0) {
                    ov->ShowOverlay(h); // once it has a picture
                }
            }
        }
        for (std::uint32_t i = 0; i < desktop::kMaxMonitors; i++) {
            if (!used[i] && monOv[i] != vr::k_ulOverlayHandleInvalid) {
                ov->HideOverlay(monOv[i]);
            }
        }
    }

    std::uint32_t allMask() const {
        std::uint32_t m = 0;
        for (const auto& mi : mons) {
            m |= 1U << mi.index;
        }
        return m;
    }

    // ---------------------------------------------------------------- the bar (the tab's page)

    dash::DeskBar deskBar(const std::string& words) const {
        dash::DeskBar db;
        db.monitors = static_cast<std::uint32_t>(mons.size());
        db.mask = mask;
        db.words = words;
        db.dashShown = false;
        db.toggleShown = true;
        db.on = on;
        db.zoomShown = on;
        db.zoom = zoom;
        db.distance = distance;
        return db;
    }

    dash::Header header() {
        if (Clock::now() >= stateAt) {
            stateAt = Clock::now() + std::chrono::seconds(1);
            std::ifstream f(reclayer::captureControlPath());
            const std::string text((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
            recording = captureActive(text) && processAlive("pibridge-record");
            headtrack = processAlive("pibridge-headtra"); // the inside-out tracker recenters (pibridge-headtrack, cut to 15)
        }
        dash::Header h;
        h.overGame = true;
        h.message = message;
        h.recordAvailable = true;
        h.recording = recording;
        h.roomQuickAvailable = true;
        h.recenterAvailable = headtrack;
        h.closeShown = false;
        return h;
    }

    std::string words() const {
        if (!on) {
            return message.empty() ? "Desktop: press Desktop to show your monitors in front of you." : message;
        }
        if (tool.empty()) {
            return "pibridge-desktop is not installed.";
        }
        const bool up = link.isOpen() && desktop::fresh(link.header()->serviceBeatNs.load(), desktop::monoNs());
        if (!up) {
            return "Starting the desktop...";
        }
        if (link.state() == desktop::State::Running) {
            return message.empty() ? "Desktop: " + link.text() + ". The laser is the mouse." : message;
        }
        return "Desktop: " + link.text();
    }

    void paintBar() {
        dash::View v = dash::deskLayout(header(), deskBar(words()));
        v.hover = over;
        v.pressed = downOn;
        std::string sig = dash::signature(v);
        if (sig == painted || !gpuOk) {
            return;
        }
        int stride = 0;
        const std::uint8_t* px = painter.paint(v, stride);
        if (!makeBuffer(barBuf, static_cast<VkDeviceSize>(dash::kW) * dash::kBarH * 4)) {
            return;
        }
        auto* dst = static_cast<std::uint8_t*>(barBuf.ptr);
        for (int y = 0; y < dash::kBarH; y++) {
            std::memcpy(dst + static_cast<std::size_t>(y) * dash::kW * 4, px + static_cast<std::size_t>(y) * stride,
                        static_cast<std::size_t>(dash::kW) * 4);
        }
        barCur ^= 1;
        present(main, barImg[static_cast<std::size_t>(barCur)], barBuf, dash::kW, dash::kBarH);
        painted = std::move(sig);
    }

    std::string barHit(double x, double yUp) {
        const dash::View v = dash::deskLayout(header(), deskBar(""));
        return dash::hit(v, x, dash::kBarH - yUp).value_or("");
    }

    void setOn(bool want) {
        on = want;
        message.clear();
        if (on) {
            loadSettings();
            if (!headNow()) {
                message = "No head pose from SteamVR: the monitors go where the room's centre is.";
            }
        } else {
            pointer = {};
            buttons = 0;
        }
        placeMonitors();
    }

    void click(const std::string& id) {
        const auto stepped = [](double v, double step, double lo, double hi) {
            return std::clamp(std::round((v + step) / std::fabs(step)) * std::fabs(step), lo, hi);
        };
        if (id == "desk:toggle") {
            setOn(!on);
        } else if (id == "desk:all") {
            mask = allMask();
            placeMonitors();
        } else if (id.starts_with("desk:mon:")) {
            mask = 1U << static_cast<std::uint32_t>(std::stoul(id.substr(9)));
            placeMonitors();
        } else if (id.starts_with("desk:zoom:") || id.starts_with("desk:dist:")) {
            const bool plus = id.back() == '+';
            if (id.starts_with("desk:zoom:")) {
                zoom = stepped(zoom, plus ? frame::kZoomStep : -frame::kZoomStep, frame::kZoomMin, frame::kZoomMax);
            } else {
                distance = stepped(distance, plus ? frame::kDistanceStep : -frame::kDistanceStep, frame::kDistanceMin,
                                   frame::kDistanceMax);
            }
            placeMonitors();
            saveDue = true;
            saveAt = Clock::now() + std::chrono::seconds(1);
        } else if (id == "view:record") {
            writeRuntimeFile(kRequestFileName, recording ? kRequestRecordStop : kRequestRecordStart);
            recording = !recording; // at once; the next state check confirms it
            stateAt = Clock::now() + std::chrono::seconds(2);
            message = recording ? "Recording what you see." : "Stopping the recording.";
        } else if (id == "view:roomquick") {
            writeRuntimeFile(kRequestFileName, kRequestQuickRoomSetup);
            message = "Quick room setup starts.";
        } else if (id == "view:recenter") {
            writeRuntimeFile("recenter-request", std::to_string(std::time(nullptr)));
            message = "Recentered: forward is where you look.";
        }
    }

    void barMouse(double x, double yUp, bool down, bool move, int button) {
        const std::string hit = barHit(x, yUp);
        if (move) {
            over = hit;
            return;
        }
        if (down) {
            downOn = hit;
        } else {
            if (!downOn.empty() && downOn == hit && button == 0) {
                click(hit);
            }
            downOn.clear();
        }
    }

    // SteamVR's laser on monitor `i` (its picture's pixels, y from the bottom)
    void monitorMouse(std::uint32_t i, double x, double yUp, int button, bool down, bool move) {
        std::uint32_t w = 0, h = 0;
        for (const auto& m : mons) {
            if (m.index == i) {
                w = m.width;
                h = m.height;
            }
        }
        if (w == 0 || h == 0) {
            return;
        }
        pointer.on = true;
        pointer.monitor = i;
        pointer.x = static_cast<std::uint32_t>(std::clamp(x, 0.0, w - 1.0));
        pointer.y = static_cast<std::uint32_t>(std::clamp(h - yUp, 0.0, h - 1.0));
        if (!move) {
            const std::uint32_t bit = button == 0 ? 1U : button == 1 ? 2U : 4U;
            buttons = down ? (buttons | bit) : (buttons & ~bit);
        }
    }
};

DeskTab::DeskTab(vr::IVROverlay* ov, vr::IVRSystem* sys, vr::IVRCompositor* comp) : d_(std::make_unique<Impl>()) {
    d_->ov = ov;
    d_->sys = sys;
    d_->comp = comp;
}

DeskTab::~DeskTab() {
    for (const auto h : d_->monOv) {
        if (h != vr::k_ulOverlayHandleInvalid) {
            d_->ov->HideOverlay(h);
        }
    }
}

bool DeskTab::create() {
    auto& d = *d_;
    const vr::EVROverlayError e = d.ov->CreateDashboardOverlay("pibridge.desktop", "Desktop", &d.main, &d.thumb);
    if (e != vr::VROverlayError_None) {
        err_ = std::string("CreateDashboardOverlay pibridge.desktop: ") + d.ov->GetOverlayErrorNameFromEnum(e);
        return false;
    }
    d.monOv.fill(vr::k_ulOverlayHandleInvalid);
    d.ov->SetOverlayInputMethod(d.main, vr::VROverlayInputMethod_Mouse);
    d.ov->SetOverlayWidthInMeters(d.main, 2.0F);
    const vr::HmdVector2_t scale{{static_cast<float>(dash::kW), static_cast<float>(dash::kBarH)}};
    d.ov->SetOverlayMouseScale(d.main, &scale);
    d.icon = monitorIcon();
    d.ov->SetOverlayRaw(d.thumb, d.icon.data(), kIconSize, kIconSize, 4);
    d.loadSettings();
    return true;
}

bool DeskTab::shown() const { return d_->on || d_->ov->IsOverlayVisible(d_->main); }

bool DeskTab::update() {
    auto& d = *d_;
    vr::VREvent_t e{};
    while (d.ov->PollNextOverlayEvent(d.main, &e, sizeof(e))) {
        switch (e.eventType) {
        case vr::VREvent_MouseMove: d.barMouse(e.data.mouse.x, e.data.mouse.y, false, true, 0); break;
        case vr::VREvent_MouseButtonDown:
        case vr::VREvent_MouseButtonUp:
            d.barMouse(e.data.mouse.x, e.data.mouse.y, e.eventType == vr::VREvent_MouseButtonDown, false,
                       e.data.mouse.button == vr::VRMouseButton_Left ? 0 : 1);
            break;
        case vr::VREvent_FocusLeave:
            d.over.clear();
            d.downOn.clear();
            break;
        case vr::VREvent_Quit: return false;
        default: break;
        }
    }
    for (std::uint32_t i = 0; i < desktop::kMaxMonitors; i++) {
        if (d.monOv[i] == vr::k_ulOverlayHandleInvalid) {
            continue;
        }
        while (d.ov->PollNextOverlayEvent(d.monOv[i], &e, sizeof(e))) {
            switch (e.eventType) {
            case vr::VREvent_MouseMove: d.monitorMouse(i, e.data.mouse.x, e.data.mouse.y, 0, false, true); break;
            case vr::VREvent_MouseButtonDown:
            case vr::VREvent_MouseButtonUp: {
                const int b = e.data.mouse.button == vr::VRMouseButton_Right ? 1 : e.data.mouse.button == vr::VRMouseButton_Middle ? 2 : 0;
                d.monitorMouse(i, e.data.mouse.x, e.data.mouse.y, b, e.eventType == vr::VREvent_MouseButtonDown, false);
                break;
            }
            case vr::VREvent_ScrollDiscrete:
                d.wheelAcc += e.data.scroll.ydelta * 120.0;
                d.wheel += static_cast<std::int64_t>(d.wheelAcc);
                d.wheelAcc -= static_cast<double>(static_cast<std::int64_t>(d.wheelAcc));
                break;
            case vr::VREvent_FocusLeave:
                if (d.pointer.monitor == i) {
                    d.pointer.on = false;
                    d.buttons = 0;
                }
                break;
            default: break;
            }
        }
    }
    if (!shown()) {
        return true;
    }
    if (!d.gpuTried) {
        std::string why;
        if (!d.initGpu(why)) {
            std::fprintf(stderr, "pibridge-overlay: the desktop has no GPU texture: %s\n", why.c_str());
        }
    }
    if (d.saveDue && Clock::now() >= d.saveAt) {
        d.saveDue = false;
        d.saveSettings();
    }
    if (d.on && (d.link.isOpen() || d.link.open())) {
        const auto now = Clock::now();
        const bool up = desktop::fresh(d.link.header()->serviceBeatNs.load(), desktop::monoNs());
        if (!up && !d.tool.empty() && now - d.startedAt > std::chrono::seconds(5)) {
            d.startedAt = now;
            desktop::startService(d.tool);
        }
        const auto mons = up ? d.link.monitors() : std::vector<desktop::MonitorInfo>{};
        bool changed = mons.size() != d.mons.size();
        for (std::size_t i = 0; i < mons.size() && !changed; i++) {
            changed = mons[i].x != d.mons[i].x || mons[i].y != d.mons[i].y || mons[i].logicalW != d.mons[i].logicalW ||
                      mons[i].logicalH != d.mons[i].logicalH || mons[i].width != d.mons[i].width ||
                      mons[i].height != d.mons[i].height;
        }
        if (up && changed) {
            d.mons = mons;
            if ((d.mask & d.allMask()) == 0) {
                d.mask = d.allMask();
            }
            d.placeMonitors();
        }
        desktop::Pointer p = d.pointer;
        p.buttons = p.on ? d.buttons : 0;
        d.link.clientUpdate(d.mask != 0 ? d.mask : desktop::kWantAll, p, d.wheel); // all until the monitors are known
        if (d.gpuOk) {
            for (std::uint32_t i = 0; i < desktop::kMaxMonitors; i++) {
                if ((d.mask & (1U << i)) == 0) {
                    continue;
                }
                std::uint32_t w = 0, h = 0;
                if (const auto sq = d.link.readFrame(i, d.seq[i], d.pix, w, h)) {
                    const bool first = d.seq[i] == 0;
                    d.seq[i] = *sq;
                    const vr::VROverlayHandle_t ovh = d.monitorOverlay(i);
                    if (ovh != vr::k_ulOverlayHandleInvalid &&
                        d.makeBuffer(d.monBuf[i], static_cast<VkDeviceSize>(w) * h * 4)) {
                        std::memcpy(d.monBuf[i].ptr, d.pix.data(), d.pix.size());
                        d.monCur[i] ^= 1;
                        d.present(ovh, d.monImg[i][static_cast<std::size_t>(d.monCur[i])], d.monBuf[i], w, h);
                        if (first) {
                            d.placeMonitors(); // shown once it has a picture
                        }
                    }
                }
            }
        }
    }
    d.paintBar();
    return true;
}

} // namespace pibridge::overlay
