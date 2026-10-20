// SPDX-License-Identifier: MIT
#include "openvr_helper.h"

#include "pibridge/room/files.h"
#include "pibridge/room/json.h"
#include "pibridge/room/zone.h"

#include <dlfcn.h>
#include <openvr.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <optional>
#include <string>

namespace pibridge::roomsetup {

namespace {

using InitInternal2Fn = uint32_t (*)(vr::EVRInitError*, vr::EVRApplicationType, const char*);
using ShutdownInternalFn = void (*)();
using GetGenericInterfaceFn = void* (*)(const char*, vr::EVRInitError*);
using IsInterfaceVersionValidFn = bool (*)(const char*);

template <typename F>
F sym(void* lib, const char* name) {
    return reinterpret_cast<F>(dlsym(lib, name));
}

/// The background client; every call fails quietly when SteamVR is not there.
class Client {
public:
    Client() {
        const auto env = room::Env::fromProcess();
        const auto t = room::readFile(env.configHome() + "/openvr/openvrpaths.vrpath");
        if (!t) {
            err_ = "no openvrpaths.vrpath";
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
        shutdown_ = sym<ShutdownInternalFn>(lib_, "VR_ShutdownInternal");
        generic_ = sym<GetGenericInterfaceFn>(lib_, "VR_GetGenericInterface");
        valid_ = sym<IsInterfaceVersionValidFn>(lib_, "VR_IsInterfaceVersionValid");
        if (init_ == nullptr || shutdown_ == nullptr || generic_ == nullptr || valid_ == nullptr) {
            err_ = lib + " lacks the OpenVR functions";
            return;
        }
        vr::EVRInitError e = vr::VRInitError_None;
        init_(&e, vr::VRApplication_Background, nullptr);
        if (e != vr::VRInitError_None) {
            err_ = e == vr::VRInitError_Init_NoServerForBackgroundApp ? "SteamVR is not running"
                                                                        : "SteamVR refused the connection (" +
                                                                              std::to_string(static_cast<int>(e)) + ")";
            return;
        }
        live_ = true;
    }
    ~Client() {
        if (live_) {
            shutdown_();
        }
        // the library stays loaded: the process ends right after
    }
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
    const std::string& error() const { return err_; }

private:
    void* lib_ = nullptr;
    InitInternal2Fn init_ = nullptr;
    ShutdownInternalFn shutdown_ = nullptr;
    GetGenericInterfaceFn generic_ = nullptr;
    IsInterfaceVersionValidFn valid_ = nullptr;
    bool live_ = false;
    std::string err_;
};

int fail(const Client& c, const char* what) {
    std::fprintf(stderr, "pibridge-roomsetup: %s: %s\n", what, c.error().empty() ? "not available" : c.error().c_str());
    return 1;
}

} // namespace

int vrUniverseMain() {
    Client c;
    auto* sys = c.iface<vr::IVRSystem>(vr::IVRSystem_Version);
    if (sys == nullptr) {
        return fail(c, "universe");
    }
    vr::ETrackedPropertyError pe = vr::TrackedProp_Success;
    const uint64_t id = sys->GetUint64TrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_CurrentUniverseId_Uint64, &pe);
    if (pe != vr::TrackedProp_Success) {
        std::fprintf(stderr, "pibridge-roomsetup: the headset reports no universe\n");
        return 1;
    }
    std::printf("%llu\n", static_cast<unsigned long long>(id));
    return 0;
}

int vrReloadMain() {
    Client c;
    auto* setup = c.iface<vr::IVRChaperoneSetup>(vr::IVRChaperoneSetup_Version);
    if (setup == nullptr) {
        return fail(c, "reload");
    }
    setup->ReloadFromDisk(vr::EChaperoneConfigFile_Live);
    std::printf("reloaded\n");
    return 0;
}

int vrStandingMain(std::optional<double> eyeHeight, std::optional<double> radius, bool seated) {
    if (!seated && !eyeHeight) {
        std::fprintf(stderr, "pibridge-roomsetup: --vr-standing needs --eye-height M (standing); seated needs none\n");
        return 1;
    }
    Client c;
    auto* sys = c.iface<vr::IVRSystem>(vr::IVRSystem_Version);
    auto* setup = c.iface<vr::IVRChaperoneSetup>(vr::IVRChaperoneSetup_Version);
    if (sys == nullptr || setup == nullptr) {
        return fail(c, "standing");
    }
    // The live head pose in SteamVR's RAW tracking space (what the Monado driver reports; the chaperone standing pose
    // is expressed in this space). A just-started background client may have no pose on the first call, so poll ~2 s.
    std::array<vr::TrackedDevicePose_t, vr::k_unMaxTrackedDeviceCount> poses{};
    const vr::HmdMatrix34_t* head = nullptr;
    for (int i = 0; i < 100 && head == nullptr; i++) {
        sys->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0.0F, poses.data(),
                                             vr::k_unMaxTrackedDeviceCount);
        const vr::TrackedDevicePose_t& p = poses[vr::k_unTrackedDeviceIndex_Hmd];
        if (p.bPoseIsValid && p.eTrackingResult == vr::TrackingResult_Running_OK) {
            head = &p.mDeviceToAbsoluteTracking;
            break;
        }
        const timespec ts{0, 20'000'000};
        nanosleep(&ts, nullptr);
    }
    if (head == nullptr) {
        std::fprintf(stderr, "pibridge-roomsetup: no valid head pose from SteamVR (put the headset on and stand still)\n");
        return 1;
    }
    const double hx = head->m[0][3], hy = head->m[1][3], hz = head->m[2][3];
    // head forward = -Z axis of the head rotation; the zero frame's -Z must match it (yaw only, on the floor)
    const double fx = -static_cast<double>(head->m[0][2]), fz = -static_cast<double>(head->m[2][2]);
    const double yaw = std::atan2(-fx, -fz); // forward = (-sin yaw, 0, -cos yaw), the room-frame convention
    // standing: origin on the floor under the head (head y - eye height). seated: origin at the head (recenter the
    // seated pose where you sit), no floor drop. Rotated by yaw about +Y (Ry = [[c,0,s],[0,1,0],[-s,0,c]]).
    const double originY = seated ? hy : (hy - *eyeHeight);
    const double cs = std::cos(yaw), sn = std::sin(yaw);
    vr::HmdMatrix34_t m{};
    m.m[0][0] = static_cast<float>(cs); m.m[0][1] = 0; m.m[0][2] = static_cast<float>(sn); m.m[0][3] = static_cast<float>(hx);
    m.m[1][0] = 0;                      m.m[1][1] = 1; m.m[1][2] = 0;                       m.m[1][3] = static_cast<float>(originY);
    m.m[2][0] = static_cast<float>(-sn);m.m[2][1] = 0; m.m[2][2] = static_cast<float>(cs); m.m[2][3] = static_cast<float>(hz);
    std::printf("%s: raw head (%.3f, %.3f, %.3f), forward yaw %.1f deg; origin y %.3f%s; writing chaperone zero pose\n",
                seated ? "seated" : "standing", hx, hy, hz, yaw * 180.0 / M_PI, originY,
                seated ? "" : (" (floor, eye " + std::to_string(*eyeHeight) + " m)").c_str());
    setup->RevertWorkingCopy(); // start from the live config so the existing collision bounds are kept
    if (seated) {
        setup->SetWorkingSeatedZeroPoseToRawTrackingPose(&m);
    } else {
        setup->SetWorkingStandingZeroPoseToRawTrackingPose(&m);
    }
    // Quick room setup: a circle of `radius` m around the play centre as the boundary (the PVRGuardian "quick setup"
    // = a circle around you; the full setup traces the room). The perimeter is in the standing frame, origin = centre.
    if (radius && *radius >= 0.3 && *radius <= 5.0) {
        constexpr int kN = 32;
        std::array<vr::HmdVector2_t, kN> ring{};
        for (int i = 0; i < kN; i++) {
            const double a = 2.0 * M_PI * i / kN;
            ring[static_cast<size_t>(i)].v[0] = static_cast<float>(*radius * std::cos(a));
            ring[static_cast<size_t>(i)].v[1] = static_cast<float>(*radius * std::sin(a));
        }
        setup->SetWorkingPerimeter(ring.data(), kN);
        std::printf("quick room: a %.2f m circle around the centre as the boundary\n", *radius);
    }
    if (!setup->CommitWorkingCopy(vr::EChaperoneConfigFile_Live)) {
        std::fprintf(stderr, "pibridge-roomsetup: SteamVR refused the chaperone commit\n");
        return 1;
    }
    std::printf("%s zero pose committed: centre where you %s, forward where you looked%s\n", seated ? "seated" : "standing",
                seated ? "sit" : "stand", radius ? ", circle boundary set" : "");
    return 0;
}

int vrZoneMain(const ZoneArg& z) {
    Client c;
    auto* s = c.iface<vr::IVRSettings>(vr::IVRSettings_Version);
    if (s == nullptr) {
        return fail(c, "zone");
    }
    vr::EVRSettingsError e = vr::VRSettingsError_None;
    const char* key = z.key == ZoneArg::Fade ? room::kKeyFade : room::kKeyAlpha;
    if (!z.value) {
        s->RemoveKeyInSection(room::kZoneSection, key, &e);
    } else if (z.key == ZoneArg::Fade) {
        s->SetFloat(room::kZoneSection, key, static_cast<float>(*z.value), &e);
    } else {
        s->SetInt32(room::kZoneSection, key, static_cast<int32_t>(*z.value), &e);
    }
    if (e != vr::VRSettingsError_None) {
        std::fprintf(stderr, "pibridge-roomsetup: SteamVR refused the setting (%d)\n", static_cast<int>(e));
        return 1;
    }
    std::printf("set\n");
    return 0;
}

} // namespace pibridge::roomsetup
