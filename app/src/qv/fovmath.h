// SPDX-License-Identifier: MIT
// Copied from artscout-qv-vulkan-linux layer/src/fovmath.h (MIT).
// The geometry of quad views, all in tangent space (x right, y up, looking down -z).
//
// A view's field is the tangent rectangle [left, right] x [down, up]. Pixels of a projection view are uniform in
// tangent space, so a focus window that covers a share s of the eye's tangent span needs s times its pixels, and
// the mapping between the composed image and the focus image is affine: no matrices on the GPU.
#pragma once

#include <array>
#include <cstdint>

#include <openxr/openxr.h>

#include "settings.h"

namespace qv {

struct Tan {
    float left, right, down, up; // tangents: left < right, down < up
};

Tan toTan(const XrFovf& fov);
XrFovf toFov(const Tan& t);

// A window of share (sx, sy) of `base`'s tangent span, centred at (cx, cy), moved back inside `base` when it would
// stick out (it keeps its size, unlike clamping, so the focus does not shrink at the edge of the field).
Tan focusWindow(const Tan& base, float cx, float cy, float sx, float sy);

// The tangent of `pointView` (view space) seen from an eye whose pose in view space is `eye`; false when the point is
// behind the eye.
bool eyeTangent(const XrPosef& eye, const XrVector3f& pointView, float& tx, float& ty);

// tan(atan(t) + deg).
float addAngle(float t, float deg);

// The focus window `inner` in the uv of the composed image covering `outer`: {u0, v0, u1, v1}, v pointing down
// (image rows), so v0 is at `inner.up`.
std::array<float, 4> uvRect(const Tan& outer, const Tan& inner);

struct Size {
    uint32_t w, h;
    bool operator==(const Size&) const = default;
};

// rec * density * share, rounded to even pixels, at least 16, at most `max`.
Size scaledSize(Size rec, Size max, float density, float shareX = 1.f, float shareY = 1.f);

// The pixels rendered per frame (both eyes) with plain stereo and with quad views.
struct Budget {
    uint64_t stereoPixels;
    uint64_t quadPixels;
    double saving; // 1 - quad / stereo
};
Budget budget(Size rec, const Settings& s, bool foveated);

// Rotate v by the inverse of q.
XrVector3f rotateInv(const XrQuaternionf& q, const XrVector3f& v);
XrVector3f rotate(const XrQuaternionf& q, const XrVector3f& v);

} // namespace qv
