// SPDX-License-Identifier: MIT
// Ported from ZoneTweak filter.go (MIT, Copyright (c) 2026 killdano; see NOTICE).
//
// The One Euro filter (Casiez et al. 2012) that steadies the laser pointer's floor point: slow, careful moves are
// smoothed, fast moves still follow quickly.
#pragma once

#include "pibridge/room/geom.h"

#include <cmath>
#include <numbers>

namespace pibridge::room {

struct OneEuro {
    double minCutoff = 0.7;
    double beta = 1.5;
    double dCutoff = 1.0;
    double x = 0;
    double dx = 0;
    bool init = false;

    static double alpha(double cutoff, double dt) {
        const double tau = 1 / (2 * std::numbers::pi * cutoff);
        return 1 / (1 + tau / dt);
    }
    double step(double v, double dt) {
        if (!init || dt <= 0) {
            x = v;
            dx = 0;
            init = true;
            return v;
        }
        const double d = (v - x) / dt;
        dx += alpha(dCutoff, dt) * (d - dx);
        x += alpha(minCutoff + beta * std::fabs(dx), dt) * (v - x);
        return x;
    }
};

struct PointFilter {
    OneEuro fx;
    OneEuro fz;
    Pt2 step(Pt2 c, double dt) { return {fx.step(c.x, dt), fz.step(c.z, dt)}; }
    void reset() { fx.init = fz.init = false; }
};

} // namespace pibridge::room
