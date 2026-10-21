// SPDX-License-Identifier: MIT
#include "overlay.h"

#include "overlay_core.h"

#include "pibridge/room/files.h"
#include "pibridge/room/json.h"
#include "pibridge_status.h"

#include <dlfcn.h>
#include <openvr.h>

#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace pibridge::overlay {

namespace {

constexpr const char* kAppKey = "pibridge.dashboard";
constexpr const char* kName = "PiBridge";

using InitInternal2Fn = std::uint32_t (*)(vr::EVRInitError*, vr::EVRApplicationType, const char*);
using GetGenericInterfaceFn = void* (*)(const char*, vr::EVRInitError*);
using IsInterfaceVersionValidFn = bool (*)(const char*);

template <typename F>
F sym(void* lib, const char* name) {
    return reinterpret_cast<F>(dlsym(lib, name));
}

// libopenvr_api.so loaded by hand (as the Monado driver tree and tools/roomsetup do). Crash-free rule: it NEVER calls
// VR_ShutdownInternal (that is the crash class the room tool hit); the process just exits and the OS cleans up. So VR is
// initialised at most once per process: run() exits on VREvent_Quit and the app restarts it.
class Client {
public:
    explicit Client(vr::EVRApplicationType type) {
        const auto env = room::Env::fromProcess();
        const auto t = room::readFile(env.configHome() + "/openvr/openvrpaths.vrpath");
        if (!t) {
            err_ = "SteamVR not found (no openvrpaths.vrpath)";
            return;
        }
        const auto j = room::parseJson(*t);
        if (!j || j->get("runtime").size() == 0) {
            err_ = "no SteamVR runtime in openvrpaths.vrpath";
            return;
        }
        const std::string lib = j->get("runtime").at(0).str() + "/bin/linux64/libopenvr_api.so";
        if (!std::filesystem::exists(lib)) {
            err_ = lib + " is missing";
            return;
        }
        lib_ = dlopen(lib.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (lib_ == nullptr) {
            err_ = "could not load " + lib;
            return;
        }
        init_ = sym<InitInternal2Fn>(lib_, "VR_InitInternal2");
        generic_ = sym<GetGenericInterfaceFn>(lib_, "VR_GetGenericInterface");
        valid_ = sym<IsInterfaceVersionValidFn>(lib_, "VR_IsInterfaceVersionValid");
        if (init_ == nullptr || generic_ == nullptr || valid_ == nullptr) {
            err_ = lib + " lacks the OpenVR entry points";
            return;
        }
        vr::EVRInitError e = vr::VRInitError_None;
        init_(&e, type, nullptr);
        if (e != vr::VRInitError_None) {
            err_ = e == vr::VRInitError_Init_NoServerForBackgroundApp
                       ? "SteamVR is not running"
                       : "SteamVR refused the connection (" + std::to_string(static_cast<int>(e)) + ")";
            return;
        }
        live_ = true;
    }
    // No VR_ShutdownInternal on purpose (crash-free). The dlopen handle is left as the process exits right after.
    ~Client() = default;
    Client(const Client&) = delete;
    Client& operator=(const Client&) = delete;

    template <class T>
    T* iface(const char* version) {
        if (!live_ || !valid_(version)) {
            return nullptr;
        }
        vr::EVRInitError e = vr::VRInitError_None;
        void* p = generic_(version, &e);
        return e == vr::VRInitError_None ? static_cast<T*>(p) : nullptr;
    }
    bool live() const { return live_; }
    const std::string& error() const { return err_; }

private:
    void* lib_ = nullptr;
    InitInternal2Fn init_ = nullptr;
    GetGenericInterfaceFn generic_ = nullptr;
    IsInterfaceVersionValidFn valid_ = nullptr;
    bool live_ = false;
    std::string err_;
};

void sleepMs(std::atomic<bool>& stop, int ms) {
    for (int left = ms; left > 0 && !stop.load(); left -= 25) {
        const timespec ts{0, static_cast<long>(left < 25 ? left : 25) * 1'000'000L};
        nanosleep(&ts, nullptr);
    }
}

// Write "<line>\n" atomically to $XDG_RUNTIME_DIR/pibridge/<kRequestFileName>; the app reads and launches the setup.
void writeRequest(const std::string& line) {
    const std::filesystem::path dir = status::runtime_dir();
    const std::filesystem::path path = dir / kRequestFileName;
    const std::filesystem::path tmp = dir / (std::string(".") + kRequestFileName + ".tmp");
    {
        std::ofstream f(tmp, std::ios::trunc);
        if (!f) {
            return;
        }
        f << line << '\n';
    }
    std::error_code ec;
    std::filesystem::rename(tmp, path, ec);
}

// The small launcher panel: a dashboard overlay with two buttons, uploaded ONCE and re-uploaded only when the hovered
// button changes. No app-window embed, no vrpanel.shm, no per-frame upload.
class Launcher {
public:
    Launcher(vr::IVROverlay* ov, const Options& o) : ov_(ov), o_(o) {}

    bool create() {
        const vr::EVROverlayError e = ov_->CreateDashboardOverlay(kAppKey, kName, &main_, &thumb_);
        if (e != vr::VROverlayError_None) {
            err_ = std::string("CreateDashboardOverlay: ") + ov_->GetOverlayErrorNameFromEnum(e);
            return false;
        }
        if (!o_.icon.empty() && std::filesystem::exists(o_.icon)) {
            ov_->SetOverlayFromFile(thumb_, o_.icon.c_str()); // the dashboard tab icon
        }
        ov_->SetOverlayWidthInMeters(main_, static_cast<float>(o_.width)); // small, fixed (~0.6 m)
        ov_->SetOverlayInputMethod(main_, vr::VROverlayInputMethod_Mouse);
        const vr::HmdVector2_t scale{{static_cast<float>(kPanelW), static_cast<float>(kPanelH)}};
        ov_->SetOverlayMouseScale(main_, &scale);
        redraw(); // the one upload
        return true;
    }

    /// Returns false on VREvent_Quit (SteamVR going down: exit; the app restarts us).
    bool poll() {
        vr::VREvent_t e{};
        while (ov_->PollNextOverlayEvent(main_, &e, sizeof(e))) {
            switch (e.eventType) {
            case vr::VREvent_MouseMove: {
                const Button b = hitTest(e.data.mouse.x, e.data.mouse.y);
                if (b != hover_) {
                    hover_ = b;
                    redraw(); // only on a hover change
                }
                break;
            }
            case vr::VREvent_MouseButtonUp:
                if (e.data.mouse.button == vr::VRMouseButton_Left) {
                    const std::string req = requestFor(hitTest(e.data.mouse.x, e.data.mouse.y));
                    if (!req.empty()) {
                        writeRequest(req);
                    }
                }
                break;
            case vr::VREvent_FocusLeave:
                if (hover_ != Button::None) {
                    hover_ = Button::None;
                    redraw();
                }
                break;
            case vr::VREvent_Quit:
                return false;
            default:
                break;
            }
        }
        return true;
    }

    const std::string& error() const { return err_; }

private:
    void redraw() {
        rgba_ = drawPanel(hover_);
        ov_->SetOverlayRaw(main_, rgba_.data(), kPanelW, kPanelH, 4);
    }

    vr::IVROverlay* ov_;
    Options o_;
    vr::VROverlayHandle_t main_ = vr::k_ulOverlayHandleInvalid;
    vr::VROverlayHandle_t thumb_ = vr::k_ulOverlayHandleInvalid;
    Button hover_ = Button::None;
    std::vector<std::uint8_t> rgba_;
    std::string err_;
};

} // namespace

int run(const Options& o, std::atomic<bool>& stop) {
    bool said = false;
    while (!stop.load()) {
        Client c(vr::VRApplication_Background); // never starts SteamVR; initialised at most once (no shutdown, see above)
        if (!c.live()) {
            if (!said) {
                std::fprintf(stderr, "pibridge-overlay: %s; waiting for SteamVR\n", c.error().c_str());
                said = true;
            }
            sleepMs(stop, 3000);
            continue; // keep waiting in-process; VR was not initialised, so there is nothing to shut down
        }
        auto* ov = c.iface<vr::IVROverlay>(vr::IVROverlay_Version);
        if (ov == nullptr) {
            std::fprintf(stderr, "pibridge-overlay: SteamVR has no IVROverlay (%s)\n", vr::IVROverlay_Version);
            return 1;
        }
        Launcher app(ov, o);
        if (!app.create()) {
            std::fprintf(stderr, "pibridge-overlay: %s\n", app.error().c_str());
            return 1;
        }
        std::fprintf(stderr, "pibridge-overlay: dashboard launcher up\n");
        while (!stop.load()) {
            if (!app.poll()) {
                // SteamVR is going down: exit cleanly (no shutdown). The app restarts us, and the next process waits.
                return 0;
            }
            sleepMs(stop, 50);
        }
        return 0; // stop requested
    }
    return 0;
}

int registerWithSteamVr(const Options& o) {
    const std::string json = manifestJson(kAppKey, o.binary, o.icon, kName);
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(o.manifest).parent_path(), ec);
    {
        std::ofstream f(o.manifest, std::ios::trunc);
        if (!f) {
            std::fprintf(stderr, "pibridge-overlay: cannot write %s\n", o.manifest.c_str());
            return 1;
        }
        f << json;
        if (!f) {
            std::fprintf(stderr, "pibridge-overlay: cannot write %s\n", o.manifest.c_str());
            return 1;
        }
    }
    Client c(vr::VRApplication_Background);
    if (!c.live()) {
        std::fprintf(stderr, "pibridge-overlay: %s (start SteamVR, then register)\n", c.error().c_str());
        return 1;
    }
    auto* apps = c.iface<vr::IVRApplications>(vr::IVRApplications_Version);
    if (apps == nullptr) {
        std::fprintf(stderr, "pibridge-overlay: SteamVR has no IVRApplications\n");
        return 1;
    }
    const vr::EVRApplicationError e = apps->AddApplicationManifest(o.manifest.c_str(), false);
    if (e != vr::VRApplicationError_None) {
        std::fprintf(stderr, "pibridge-overlay: AddApplicationManifest: %s\n", apps->GetApplicationsErrorNameFromEnum(e));
        return 1;
    }
    apps->SetApplicationAutoLaunch(kAppKey, true);
    std::printf("registered %s (manifest %s, auto-launch on)\n", kAppKey, o.manifest.c_str());
    return 0;
}

int unregisterWithSteamVr(const Options& o) {
    Client c(vr::VRApplication_Background);
    if (!c.live()) {
        std::fprintf(stderr, "pibridge-overlay: %s (start SteamVR, then unregister)\n", c.error().c_str());
        return 1;
    }
    auto* apps = c.iface<vr::IVRApplications>(vr::IVRApplications_Version);
    if (apps == nullptr) {
        std::fprintf(stderr, "pibridge-overlay: SteamVR has no IVRApplications\n");
        return 1;
    }
    apps->SetApplicationAutoLaunch(kAppKey, false);
    apps->RemoveApplicationManifest(o.manifest.c_str());
    std::printf("unregistered %s\n", kAppKey);
    return 0;
}

} // namespace pibridge::overlay
