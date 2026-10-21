// SPDX-License-Identifier: MIT
#include "rendermodel.h"

#include "pibridge/room/files.h"
#include "pibridge/room/json.h"

#include <dlfcn.h>
#include <openvr.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace pibridge::roomsetup {

namespace {

using InitInternal2Fn = std::uint32_t (*)(vr::EVRInitError*, vr::EVRApplicationType, const char*);
using ShutdownInternalFn = void (*)();
using GetGenericInterfaceFn = void* (*)(const char*, vr::EVRInitError*);
using IsInterfaceVersionValidFn = bool (*)(const char*);

template <typename F>
F sym(void* lib, const char* name) {
    return reinterpret_cast<F>(dlsym(lib, name));
}

/// One controller's mesh in its local render-model space: triangles (3 positions each) and their flat normals.
struct Mesh {
    std::vector<std::array<room::Vec3, 3>> tris;
    std::vector<room::Vec3> normals;
};

/// Build a Mesh from an OpenVR RenderModel_t (positions in metres, 16-bit indices).
Mesh convert(const vr::RenderModel_t& m) {
    Mesh out;
    out.tris.reserve(m.unTriangleCount);
    out.normals.reserve(m.unTriangleCount);
    const auto vpos = [&](std::uint16_t i) {
        const auto& v = m.rVertexData[i].vPosition;
        return room::Vec3{v.v[0], v.v[1], v.v[2]};
    };
    for (std::uint32_t t = 0; t < m.unTriangleCount; t++) {
        const room::Vec3 a = vpos(m.rIndexData[3 * t]);
        const room::Vec3 b = vpos(m.rIndexData[3 * t + 1]);
        const room::Vec3 c = vpos(m.rIndexData[3 * t + 2]);
        out.tris.push_back({a, b, c});
        out.normals.push_back((b - a).cross(c - a).norm());
    }
    return out;
}

} // namespace

struct ControllerModels::Impl {
    void* lib = nullptr;
    ShutdownInternalFn shutdown = nullptr;
    bool live = false;
    vr::IVRSystem* sys = nullptr;
    vr::IVRRenderModels* rm = nullptr;
    struct Hand {
        std::string name; // the render-model name, once the controller is known
        bool ready = false;
        bool failed = false; // give up for this hand (keep the box proxy)
        Mesh mesh;
    };
    std::array<Hand, 2> hands;

    Impl() {
        const auto env = room::Env::fromProcess();
        const auto t = room::readFile(env.configHome() + "/openvr/openvrpaths.vrpath");
        if (!t) {
            return;
        }
        const auto j = room::parseJson(*t);
        if (!j || j->get("runtime").size() == 0) {
            return;
        }
        const std::string path = j->get("runtime").at(0).str() + "/bin/linux64/libopenvr_api.so";
        if (!std::filesystem::exists(path)) {
            return;
        }
        lib = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (lib == nullptr) {
            return;
        }
        auto init = sym<InitInternal2Fn>(lib, "VR_InitInternal2");
        shutdown = sym<ShutdownInternalFn>(lib, "VR_ShutdownInternal");
        auto generic = sym<GetGenericInterfaceFn>(lib, "VR_GetGenericInterface");
        auto valid = sym<IsInterfaceVersionValidFn>(lib, "VR_IsInterfaceVersionValid");
        if (init == nullptr || shutdown == nullptr || generic == nullptr || valid == nullptr) {
            return;
        }
        vr::EVRInitError e = vr::VRInitError_None;
        init(&e, vr::VRApplication_Background, nullptr);
        if (e != vr::VRInitError_None) {
            std::printf("controller models: no OpenVR background client (%d): drawing the box proxy\n",
                        static_cast<int>(e));
            return;
        }
        const auto get = [&](const char* ver) -> void* {
            if (!valid(ver)) {
                return nullptr;
            }
            vr::EVRInitError ge = vr::VRInitError_None;
            void* p = generic(ver, &ge);
            return ge == vr::VRInitError_None ? p : nullptr;
        };
        sys = static_cast<vr::IVRSystem*>(get(vr::IVRSystem_Version));
        rm = static_cast<vr::IVRRenderModels*>(get(vr::IVRRenderModels_Version));
        live = sys != nullptr && rm != nullptr;
        if (live) {
            std::printf("controller models: OpenVR render models ready\n");
        }
    }

    ~Impl() {
        // Do NOT call VR_ShutdownInternal here. This OpenVR background client lives inside the OpenXR (--map) process,
        // which is also an active SteamVR scene session; shutting the OpenVR client down at process teardown tore down
        // shared SteamVR state and crashed the OpenXR session teardown (the frontend "fell out" after the room setup
        // finished, so the chaperone write never ran). We leak the client instead: the process exits immediately after
        // runMapXr returns, and the OS closes the connection. `shutdown` is intentionally unused.
    }

    void poll() {
        if (!live) {
            return;
        }
        for (int h = 0; h < 2; h++) {
            Hand& hd = hands[static_cast<std::size_t>(h)];
            if (hd.ready || hd.failed) {
                continue;
            }
            if (hd.name.empty()) {
                const vr::ETrackedControllerRole role =
                    h == 0 ? vr::TrackedControllerRole_LeftHand : vr::TrackedControllerRole_RightHand;
                const vr::TrackedDeviceIndex_t idx = sys->GetTrackedDeviceIndexForControllerRole(role);
                if (idx == vr::k_unTrackedDeviceIndexInvalid) {
                    continue; // the role is not assigned yet; try again next frame
                }
                char buf[vr::k_unMaxPropertyStringSize];
                vr::ETrackedPropertyError pe = vr::TrackedProp_Success;
                const std::uint32_t n =
                    sys->GetStringTrackedDeviceProperty(idx, vr::Prop_RenderModelName_String, buf, sizeof buf, &pe);
                if (pe != vr::TrackedProp_Success || n == 0) {
                    continue;
                }
                hd.name = buf;
            }
            vr::RenderModel_t* model = nullptr;
            const vr::EVRRenderModelError e = rm->LoadRenderModel_Async(hd.name.c_str(), &model);
            if (e == vr::VRRenderModelError_Loading) {
                continue; // still loading; poll again next frame
            }
            if (e != vr::VRRenderModelError_None || model == nullptr) {
                std::printf("controller models: hand %d (%s) did not load (%d): box proxy\n", h, hd.name.c_str(),
                            static_cast<int>(e));
                hd.failed = true;
                continue;
            }
            hd.mesh = convert(*model);
            rm->FreeRenderModel(model);
            hd.ready = true;
            std::printf("controller models: hand %d = %s (%zu triangles)\n", h, hd.name.c_str(), hd.mesh.tris.size());
        }
    }
};

ControllerModels::ControllerModels() : impl_(std::make_unique<Impl>()) {}
ControllerModels::~ControllerModels() = default;

bool ControllerModels::live() const { return impl_->live; }
void ControllerModels::poll() { impl_->poll(); }
bool ControllerModels::ready(int h) const {
    return h >= 0 && h < 2 && impl_->hands[static_cast<std::size_t>(h)].ready;
}

bool ControllerModels::draw(room::Batch& b, int h, const room::Mat34& grip, room::Rgba tint) const {
    if (!ready(h)) {
        return false;
    }
    const Mesh& mesh = impl_->hands[static_cast<std::size_t>(h)].mesh;
    // a fixed light in the stage; our renderer is unlit, so flat-shade each triangle by its world normal for depth
    const room::Vec3 light = room::Vec3{0.4, 1.0, 0.35}.norm();
    for (std::size_t i = 0; i < mesh.tris.size(); i++) {
        const auto& tr = mesh.tris[i];
        const room::Vec3 a = grip.xform(tr[0]), c = grip.xform(tr[1]), d = grip.xform(tr[2]);
        const room::Vec3 ln = mesh.normals[i];
        const room::Vec3 wn = grip.axis(0) * ln.x + grip.axis(1) * ln.y + grip.axis(2) * ln.z; // rotate the normal
        const float sh = static_cast<float>(std::clamp(0.4 + 0.6 * std::fabs(wn.dot(light)), 0.0, 1.0));
        b.tri(a, c, d, room::Rgba{tint.r * sh, tint.g * sh, tint.b * sh, tint.a});
    }
    return true;
}

} // namespace pibridge::roomsetup
