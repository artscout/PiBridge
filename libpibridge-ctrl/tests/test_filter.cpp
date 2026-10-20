// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-hands/src/filter.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/filter.h"
#include "pibridge_test.h"

#include <cmath>
#include <cstdint>
#include <algorithm>
#include <limits>

using namespace pibridge::ctrl;

namespace {

/// Deterministic pseudo noise in [-1, 1] (xorshift).
struct Noise {
    std::uint32_t s;
    float next() {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        return (static_cast<float>(s) / static_cast<float>(UINT32_MAX)) * 2.0f - 1.0f;
    }
};

constexpr float NaN = std::numeric_limits<float>::quiet_NaN();
constexpr float INF = std::numeric_limits<float>::infinity();

} // namespace

TEST(one_euro_first_sample_passes_through) {
    OneEuro f(1.0f, 0.0f, 1.0f);
    CHECK(!f.value());
    CHECK(f.update(0.7f, 0.004f) == 0.7f);
    CHECK(f.value() == 0.7f);
}

TEST(one_euro_constant_input_is_a_fixed_point) {
    OneEuro f(2.0f, 5.0f, 1.0f);
    f.update(0.4f, 0.004f);
    for (int i = 0; i < 1000; i++) {
        CHECK(std::fabs(f.update(0.4f, 0.004f) - 0.4f) < 1e-6f);
    }
    CHECK(std::fabs(f.speed()) < 1e-6f);
}

TEST(one_euro_attenuates_jitter_at_rest) {
    OneEuro f(1.5f, 0.0f, 1.0f);
    Noise n{0x12345678};
    double raw_sq = 0.0, out_sq = 0.0;
    f.update(0.5f, 0.004f);
    for (int i = 0; i < 4000; i++) {
        const float noise = 0.02f * n.next();
        const float y = f.update(0.5f + noise, 0.004f);
        if (i > 500) {
            raw_sq += static_cast<double>(noise) * static_cast<double>(noise);
            out_sq += static_cast<double>(y - 0.5f) * static_cast<double>(y - 0.5f);
        }
    }
    CHECK(out_sq < raw_sq * 0.2);
}

TEST(one_euro_speed_coefficient_reduces_lag_on_fast_moves) {
    const auto reach = [](float beta) {
        OneEuro f(1.0f, beta, 1.0f);
        f.update(0.0f, 0.004f);
        float t = 0.0f;
        for (;;) {
            t += 0.004f;
            const float x = std::fmin(t / 0.1f, 1.0f);
            const float y = f.update(x, 0.004f);
            if (y >= 0.9f || t > 5.0f) {
                return t;
            }
        }
    };
    CHECK(reach(20.0f) < reach(0.0f) * 0.6f);
}

TEST(one_euro_never_leaves_the_input_range) {
    OneEuro f(1.0f, 30.0f, 1.0f);
    Noise n{99};
    f.update(0.0f, 0.004f);
    for (int i = 0; i < 20000; i++) {
        const float x = std::clamp(n.next() * 0.5f + 0.5f, 0.0f, 1.0f);
        const float y = f.update(x, 0.004f);
        CHECK(y >= 0.0f && y <= 1.0f);
    }
}

TEST(one_euro_ignores_garbage) {
    OneEuro f(1.0f, 1.0f, 1.0f);
    f.update(0.3f, 0.004f);
    CHECK(f.update(NaN, 0.004f) == 0.3f);
    CHECK(f.update(INF, 0.004f) == 0.3f);
    CHECK(f.update(0.9f, 0.0f) == 0.3f);
    CHECK(f.update(0.9f, -1.0f) == 0.3f);
    CHECK(f.update(0.9f, NaN) == 0.3f);
    CHECK(f.update(0.9f, 0.004f) > 0.3f);
}

TEST(one_euro_is_roughly_independent_of_the_sample_rate) {
    const auto run = [](float dt) {
        OneEuro f(2.0f, 3.0f, 1.0f);
        f.update(0.0f, dt);
        const auto steps = static_cast<std::size_t>(0.3f / dt);
        float y = 0.0f;
        for (std::size_t i = 0; i < steps; i++) {
            y = f.update(1.0f, dt);
        }
        return y;
    };
    CHECK(std::fabs(run(0.004f) - run(0.0111f)) < 0.1f);
}

TEST(smooth_damp_converges_without_overshoot) {
    SmoothDamp s(0.0f, 0.08f, 100.0f);
    float prev = 0.0f;
    for (int i = 0; i < 500; i++) {
        const float v = s.update(1.0f, 0.004f);
        CHECK(v >= prev - 1e-6f && v <= 1.0f + 1e-6f);
        prev = v;
    }
    CHECK(std::fabs(s.value() - 1.0f) < 1e-3f);
}

TEST(smooth_damp_goes_down_as_well) {
    SmoothDamp s(1.0f, 0.08f, 100.0f);
    for (int i = 0; i < 500; i++) {
        const float v = s.update(0.0f, 0.004f);
        CHECK(v >= -1e-6f && v <= 1.0f + 1e-6f);
    }
    CHECK(s.value() < 1e-3f);
}

TEST(smooth_damp_respects_the_speed_limit) {
    SmoothDamp s(0.0f, 0.05f, 2.0f);
    float prev = 0.0f;
    for (int i = 0; i < 200; i++) {
        const float v = s.update(1.0f, 0.004f);
        CHECK((v - prev) / 0.004f <= 2.0f * 1.2f);
        prev = v;
    }
}

TEST(smooth_damp_reaches_the_target_in_about_the_smooth_time) {
    SmoothDamp s(0.0f, 0.1f, 100.0f);
    float t = 0.0f;
    while (s.update(1.0f, 0.001f) < 0.95f) {
        t += 0.001f;
        CHECK(t < 1.0f);
        if (t >= 1.0f) {
            break;
        }
    }
    CHECK(t >= 0.08f && t < 0.4f);
}

TEST(smooth_damp_snap_and_garbage) {
    SmoothDamp s(0.0f, 0.1f, 10.0f);
    s.snap(0.6f);
    CHECK(s.value() == 0.6f);
    CHECK(s.update(NaN, 0.01f) == 0.6f);
    CHECK(s.update(1.0f, 0.0f) == 0.6f);
    CHECK(s.update(1.0f, -0.1f) == 0.6f);
    s.snap(NaN);
    CHECK(s.value() == 0.6f);
}

PIBRIDGE_TEST_MAIN()
