// SPDX-License-Identifier: MIT
// Smoothing of the gaze for the focus placement: a One-Euro filter (Casiez et al. 2012) on the gaze tangent, with a
// saccade snap, and a short hold of the last gaze when it drops out (blinks).
//
// One-Euro is a first-order low-pass whose cutoff rises with the speed: fc = minCutoff + beta * |speed| (speed in
// degrees per second, itself low-passed at dCutoff). At fixation it removes the sample noise that would make the
// focus edge shimmer; during a saccade it opens up. The speed estimate needs a few samples to rise, so a saccade
// would still be followed with some lag: the snap removes it (raw gaze more than snapDeg from the output for two
// samples in a row: the eye has landed elsewhere, jump there). Plain exponential smoothing lags every saccade.
#pragma once

#include <cmath>
#include <cstdint>
#include <optional>

namespace qv {

struct OneEuroParams {
    double minCutoffHz = 1.0;
    double beta = 0.03; // per degree per second
    double dCutoffHz = 2.0;
    double snapDeg = 3.0;
};

class OneEuro {
  public:
    // Samples further apart than this restart the filter.
    static constexpr double kMaxGapS = 0.1;

    // Filter the tangent (x, y) at time t (seconds).
    void filter(const OneEuroParams& p, double t, double& x, double& y) {
        if (!has_) {
            restart(t, x, y);
            return;
        }
        const double dt = t - t_;
        if (!(dt > 1e-5) || dt > kMaxGapS) {
            restart(t, x, y);
            return;
        }
        t_ = t;
        const double offDeg = std::atan(std::hypot(x - x_, y - y_)) * 180.0 / M_PI;
        if (p.snapDeg > 0 && offDeg > p.snapDeg) {
            if (++far_ >= 2) {
                restart(t, x, y);
                return;
            }
        } else {
            far_ = 0;
        }
        const double ad = alpha(p.dCutoffHz, dt);
        dx_ += ad * ((x - x_) / dt - dx_);
        dy_ += ad * ((y - y_) / dt - dy_);
        const double speedDps = std::hypot(dx_, dy_) * 180.0 / M_PI;
        const double a = alpha(p.minCutoffHz + p.beta * speedDps, dt);
        x_ += a * (x - x_);
        y_ += a * (y - y_);
        x = x_;
        y = y_;
    }

    void reset() { has_ = false; }

  private:
    static double alpha(double cutoffHz, double dt) {
        const double tau = 1.0 / (2.0 * M_PI * std::max(cutoffHz, 1e-6));
        return 1.0 / (1.0 + tau / dt);
    }
    void restart(double t, double x, double y) {
        has_ = true;
        t_ = t;
        x_ = x;
        y_ = y;
        dx_ = dy_ = 0;
        far_ = 0;
    }

    bool has_ = false;
    double t_ = 0, x_ = 0, y_ = 0, dx_ = 0, dy_ = 0;
    uint32_t far_ = 0;
};

// The gaze the focus uses: filtered when valid, the last one for holdMs after it drops out, then none.
class GazeTrack {
  public:
    struct Dir {
        double x, y; // tangent in view space (x right, y up, looking down -z)
    };

    // A new reading at time t (seconds); `raw` empty when the tracker has no gaze.
    std::optional<Dir> update(double t, std::optional<Dir> raw, bool smooth, uint32_t holdMs,
                              const OneEuroParams& p = {}) {
        if (raw) {
            Dir d = *raw;
            if (smooth) {
                filter_.filter(p, t, d.x, d.y);
            } else {
                filter_.reset();
            }
            last_ = d;
            lastT_ = t;
            return d;
        }
        filter_.reset();
        if (last_ && t - lastT_ <= holdMs / 1000.0) {
            return last_;
        }
        last_.reset();
        return std::nullopt;
    }

  private:
    OneEuro filter_;
    std::optional<Dir> last_;
    double lastT_ = 0;
};

} // namespace qv
