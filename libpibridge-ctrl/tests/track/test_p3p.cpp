// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-ctrltrack/src/p3p.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/track/p3p.h"
#include "pibridge_test.h"

#include <algorithm>
#include <cfloat>
#include <cmath>

using namespace pibridge::ctrl::track;
namespace g = pibridge::cam::geom;

TEST(quartic_known_roots) {
    // (x-1)(x+2)(x-3)(x-0.5)
    const double r[] = {1.0, -2.0, 3.0, 0.5};
    std::vector<double> c{1.0};
    for (const double ri : r) {
        std::vector<double> n(c.size() + 1, 0.0);
        for (std::size_t i = 0; i < c.size(); i++) {
            n[i] += c[i];
            n[i + 1] -= c[i] * ri;
        }
        c = n;
    }
    auto got = quartic_real_roots({c[0], c[1], c[2], c[3], c[4]});
    std::sort(got.begin(), got.end());
    const double want[] = {-2.0, 0.5, 1.0, 3.0};
    for (std::size_t i = 0; i < got.size() && i < 4; i++) {
        CHECK(std::fabs(got[i] - want[i]) < 1e-9);
    }
}

TEST(recovers_pose) {
    const std::array<g::Vec3, 3> x{g::Vec3{0.03, -0.01, -0.016}, g::Vec3{0.029, 0.001, -0.024},
                                   g::Vec3{0.044, 0.019, -0.010}};
    const g::Pose truth{g::mat_mul(g::rodrigues({0.3, -0.5, 0.2}), g::rot_x(20.0)), {0.05, -0.1, 0.45}};
    std::array<g::Vec3, 3> y{};
    for (int i = 0; i < 3; i++) {
        y[i] = g::normalize(truth.apply(x[i]));
    }
    const auto sols = p3p(y, x);
    CHECK(!sols.empty());
    double best = DBL_MAX;
    for (const g::Pose& s : sols) {
        double e = g::norm(g::sub(s.t, truth.t));
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                e += std::fabs(s.r[i][j] - truth.r[i][j]);
            }
        }
        best = std::min(best, e);
    }
    CHECK(best < 1e-6);
}

PIBRIDGE_TEST_MAIN()
