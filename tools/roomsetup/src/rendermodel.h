// SPDX-License-Identifier: MIT
// The real SteamVR controller render models in the --map process (room setup), drawn over the passthrough at the
// controller grip pose. An OpenVR *background* client (dlopen of libopenvr_api.so, the same pattern as
// openvr_helper.cpp) loads IVRRenderModels; the meshes are converted to room::Batch triangles once and flat-shaded
// each frame (our renderer is unlit, so there is no texture pipeline). Everything degrades to a no-op when SteamVR or
// the models are unavailable, so the caller keeps its box-proxy + laser fallback. OpenVR is hidden behind a pimpl so
// this header stays free of openvr.h (mapflow and the tests never see it).
#pragma once

#include "pibridge/room/draw.h"
#include "pibridge/room/vmath.h"

#include <memory>

namespace pibridge::roomsetup {

class ControllerModels {
public:
    ControllerModels();
    ~ControllerModels();
    ControllerModels(const ControllerModels&) = delete;
    ControllerModels& operator=(const ControllerModels&) = delete;

    /// Whether an OpenVR background client is connected (else draw() is always false and the caller uses the box).
    bool live() const;
    /// Pump the async render-model loads; cheap once loaded. Call once per frame.
    void poll();
    /// Whether hand `h` (0 = left, 1 = right) has a mesh ready to draw.
    bool ready(int h) const;
    /// Append hand `h`'s flat-shaded mesh at `grip` (stage space) into `b`, tinted by `tint`. Returns false (nothing
    /// drawn) when no mesh is ready, so the caller draws its box proxy instead.
    bool draw(room::Batch& b, int h, const room::Mat34& grip, room::Rgba tint) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace pibridge::roomsetup
