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

/// An OpenVR HmdMatrix34_t (row-major 3x4) as our Mat34.
room::Mat34 fromHmd34(const vr::HmdMatrix34_t& h) {
    room::Mat34 o;
    for (std::size_t r = 0; r < 3; r++) {
        for (std::size_t c = 0; c < 4; c++) {
            o.m[r][c] = h.m[r][c];
        }
    }
    return o;
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
            // Component models (Pimax's crystal_controller_*): the geometry is in the components (body + buttons), not
            // at the base name, so LoadRenderModel of the base returns nothing. Load each component's render model and
            // append -- the parts are already in the controller frame. A model with no components loads by its name.
            Mesh combined;
            bool anyLoading = false, loadedSomething = false;
            // `xf` places the loaded part into the controller frame. For a standalone model it is identity; for a
            // component it is the component's mTrackingToComponentRenderModel (Pimax's crystal_* parts are authored
            // already in that frame, so their transform is identity, but a stock model -- the Oculus Touch fallback --
            // authors each part at its own origin, so without this its parts pile up at the origin as a grey mess).
            const auto loadOne = [&](const char* rmName, const room::Mat34& xf) {
                vr::RenderModel_t* model = nullptr;
                const vr::EVRRenderModelError e = rm->LoadRenderModel_Async(rmName, &model);
                if (e == vr::VRRenderModelError_Loading) {
                    anyLoading = true;
                    return;
                }
                if (e != vr::VRRenderModelError_None || model == nullptr) {
                    return;
                }
                const Mesh m = convert(*model);
                rm->FreeRenderModel(model);
                for (const auto& tr : m.tris) {
                    const room::Vec3 a = xf.xform(tr[0]), b = xf.xform(tr[1]), c = xf.xform(tr[2]);
                    combined.tris.push_back({a, b, c});
                    combined.normals.push_back((b - a).cross(c - a).norm());
                }
                loadedSomething = true;
            };
            const std::uint32_t nComp = rm->GetComponentCount(hd.name.c_str());
            if (nComp == 0) {
                loadOne(hd.name.c_str(), room::Mat34::identity());
            } else {
                for (std::uint32_t ci = 0; ci < nComp; ci++) {
                    char comp[vr::k_unMaxPropertyStringSize];
                    if (rm->GetComponentName(hd.name.c_str(), ci, comp, sizeof comp) == 0) {
                        continue;
                    }
                    char crm[vr::k_unMaxPropertyStringSize];
                    if (rm->GetComponentRenderModelName(hd.name.c_str(), comp, crm, sizeof crm) == 0) {
                        continue; // a pose-only component (tip / base / openxr_aim): no geometry
                    }
                    // the part's pose in the controller frame, and whether it shows in the rest pose
                    room::Mat34 xf = room::Mat34::identity();
                    vr::RenderModel_ControllerMode_State_t mode{};
                    vr::VRControllerState_t cs{};
                    vr::RenderModel_ComponentState_t st{};
                    if (rm->GetComponentState(hd.name.c_str(), comp, &cs, &mode, &st)) {
                        if ((st.uProperties & vr::VRComponentProperty_IsVisible) == 0) {
                            continue;
                        }
                        xf = fromHmd34(st.mTrackingToComponentRenderModel);
                    }
                    loadOne(crm, xf);
                }
            }
            if (anyLoading) {
                continue; // some part is still loading; try again next frame
            }
            if (!loadedSomething) {
                std::printf("controller models: hand %d (%s) did not load (%u components): box proxy\n", h,
                            hd.name.c_str(), nComp);
                hd.failed = true;
                continue;
            }
            hd.mesh = std::move(combined);
            hd.ready = true;
            std::printf("controller models: hand %d = %s (%u components, %zu triangles)\n", h, hd.name.c_str(), nComp,
                        hd.mesh.tris.size());
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
