// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/drift.rs (MIT OR Apache-2.0).
#include "pibridge/eyemath/drift.h"

#include <cmath>

#include "pibridge/eyemath/calib.h"
#include "pibridge/eyemath/correction.h"
#include "num.h"

namespace pibridge::eyemath {

bool Drift::observe(const DriftParams& p, std::size_t eye, Tangent gaze, Tangent truth) {
    const auto cur = offset[eye];
    const Tangent now{gaze.u + cur.u, gaze.v + cur.v};
    if (tangent_angle_deg(now, truth) > p.max_residual_deg) {
        rejected[eye]++;
        return false;
    }
    const Tangent want{truth.u - gaze.u, truth.v - gaze.v};
    Tangent o{cur.u + p.gain * (want.u - cur.u), cur.v + p.gain * (want.v - cur.v)};
    const double lim = std::tan(detail::to_radians(p.max_offset_deg));
    const double l = std::sqrt(o.u * o.u + o.v * o.v);
    if (l > lim) {
        o = {o.u * lim / l, o.v * lim / l};
    }
    offset[eye] = o;
    accepted[eye]++;
    return true;
}

Tangent Drift::apply(std::size_t eye, Tangent t) const { return {t.u + offset[eye].u, t.v + offset[eye].v}; }

Vec3 Drift::apply_dir(std::size_t eye, Tangent t) const {
    const auto a = apply(eye, t);
    return dir_of(a.u, a.v);
}

} // namespace pibridge::eyemath
