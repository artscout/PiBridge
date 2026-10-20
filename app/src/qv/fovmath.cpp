// SPDX-License-Identifier: MIT
// Copied from artscout-qv-vulkan-linux layer/src/fovmath.cpp (MIT).
#include "fovmath.h"

#include <algorithm>
#include <cmath>

namespace qv {

Tan toTan(const XrFovf& f) {
    return {std::tan(f.angleLeft), std::tan(f.angleRight), std::tan(f.angleDown), std::tan(f.angleUp)};
}

XrFovf toFov(const Tan& t) {
    return {std::atan(t.left), std::atan(t.right), std::atan(t.up), std::atan(t.down)};
}

namespace {

// [lo, hi] of width share*(bhi-blo) centred at c, moved inside [blo, bhi].
void window1d(float blo, float bhi, float c, float share, float& lo, float& hi) {
    const float span = bhi - blo;
    const float w = std::clamp(share, 0.f, 1.f) * span;
    lo = c - w / 2;
    hi = c + w / 2;
    if (lo < blo) {
        hi += blo - lo;
        lo = blo;
    }
    if (hi > bhi) {
        lo -= hi - bhi;
        hi = bhi;
    }
    lo = std::max(lo, blo);
}

} // namespace

Tan focusWindow(const Tan& base, float cx, float cy, float sx, float sy) {
    Tan t{};
    window1d(base.left, base.right, cx, sx, t.left, t.right);
    window1d(base.down, base.up, cy, sy, t.down, t.up);
    return t;
}

XrVector3f rotate(const XrQuaternionf& q, const XrVector3f& v) {
    // v + 2w(u x v) + 2u x (u x v), u = (x, y, z)
    const float cx = q.y * v.z - q.z * v.y, cy = q.z * v.x - q.x * v.z, cz = q.x * v.y - q.y * v.x;
    const float dx = q.y * cz - q.z * cy, dy = q.z * cx - q.x * cz, dz = q.x * cy - q.y * cx;
    return {v.x + 2 * (q.w * cx + dx), v.y + 2 * (q.w * cy + dy), v.z + 2 * (q.w * cz + dz)};
}

XrVector3f rotateInv(const XrQuaternionf& q, const XrVector3f& v) {
    return rotate({-q.x, -q.y, -q.z, q.w}, v);
}

bool eyeTangent(const XrPosef& eye, const XrVector3f& p, float& tx, float& ty) {
    const XrVector3f d{p.x - eye.position.x, p.y - eye.position.y, p.z - eye.position.z};
    const XrVector3f l = rotateInv(eye.orientation, d);
    if (!(l.z < -1e-6f)) {
        return false;
    }
    tx = l.x / -l.z;
    ty = l.y / -l.z;
    return std::isfinite(tx) && std::isfinite(ty);
}

float addAngle(float t, float deg) {
    if (deg == 0.f) {
        return t;
    }
    const float a = std::clamp(std::atan(t) + deg * 3.14159265f / 180.f, -1.5f, 1.5f);
    return std::tan(a);
}

std::array<float, 4> uvRect(const Tan& o, const Tan& i) {
    const float w = o.right - o.left, h = o.up - o.down;
    return {(i.left - o.left) / w, (o.up - i.up) / h, (i.right - o.left) / w, (o.up - i.down) / h};
}

Size scaledSize(Size rec, Size max, float density, float shareX, float shareY) {
    const auto one = [](uint32_t r, uint32_t m, float k) {
        uint32_t v = static_cast<uint32_t>(std::lround(double(r) * k / 2.0)) * 2;
        v = std::max<uint32_t>(v, 16);
        if (m > 0) {
            v = std::min(v, m);
        }
        return v;
    };
    return {one(rec.w, max.w, density * shareX), one(rec.h, max.h, density * shareY)};
}

Budget budget(Size rec, const Settings& s, bool foveated) {
    const auto& share = foveated ? s.gazeSize : s.fixedSize;
    const Size p = scaledSize(rec, {}, s.peripheralDensity);
    const Size f = scaledSize(rec, {}, s.focusDensity, share[0], share[1]);
    Budget b{};
    b.stereoPixels = 2ull * rec.w * rec.h;
    b.quadPixels = 2ull * (uint64_t(p.w) * p.h + uint64_t(f.w) * f.h);
    b.saving = b.stereoPixels ? 1.0 - double(b.quadPixels) / double(b.stereoPixels) : 0.0;
    return b;
}

} // namespace qv
