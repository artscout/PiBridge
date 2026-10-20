// SPDX-License-Identifier: MIT
// Twins of omarchyvr pimax-hands/tests/properties.rs (MIT OR Apache-2.0): whatever the controllers send, the hand
// stays sane. proptest's random cases become fixed-seed loops with the same value ranges (400 cases each).
#include "pibridge/ctrl/mapping.h"
#include "pibridge_test.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

using namespace pibridge::ctrl;

namespace {

constexpr std::uint64_t MS = 1000000;
constexpr int CASES = 400;

struct Rng {
    std::uint64_t s;
    std::uint64_t next() {
        s = s * 6364136223846793005ull + 1442695040888963407ull;
        return s >> 11;
    }
    std::uint64_t below(std::uint64_t n) { return n ? next() % n : 0; }
    std::uint64_t range(std::uint64_t lo, std::uint64_t hi) { return lo + below(hi - lo); } // [lo, hi)
    float f(float lo, float hi) { return lo + (hi - lo) * static_cast<float>(next() >> 29) / 16777216.0f; }
    std::uint64_t any64() { return (next() << 11) ^ next(); }
};

/// Floats including the nasty ones (8:1:1:1:1:1 like the proptest strategy).
float any_f32(Rng& r) {
    switch (r.below(13)) {
    case 8: return std::numeric_limits<float>::quiet_NaN();
    case 9: return std::numeric_limits<float>::infinity();
    case 10: return -std::numeric_limits<float>::infinity();
    case 11: return std::numeric_limits<float>::max();
    case 12: return std::numeric_limits<float>::lowest();
    default: return r.f(-1.5f, 1.5f);
    }
}

Input any_input(Rng& r) {
    return Input{any_f32(r), any_f32(r), any_f32(r), any_f32(r), static_cast<std::uint32_t>(r.next())};
}

Input clean_input(Rng& r) {
    return Input{r.f(0.0f, 1.0f), r.f(0.0f, 1.0f), r.f(-1.0f, 1.0f), r.f(-1.0f, 1.0f),
                 static_cast<std::uint32_t>(r.below(flags::ALL + 1))};
}

bool in01(float v) {
    return std::isfinite(v) && v >= 0.0f && v <= 1.0f;
}

} // namespace

TEST(output_is_always_finite_and_in_range) {
    Rng r{1};
    for (int c = 0; c < CASES; c++) {
        HandState s{Params{}};
        std::uint64_t t = r.below(UINT64_MAX / 4);
        const std::uint64_t n = r.range(1, 200);
        for (std::uint64_t k = 0; k < n; k++) {
            const std::uint64_t dt_ms = r.below(400);
            const Input i = any_input(r);
            const std::uint64_t add = dt_ms * MS;
            t = t + add < t ? UINT64_MAX : t + add;
            const Curls cu = s.update(t, i);
            for (float v : cu.to_array()) {
                CHECK(in01(v));
            }
            const HandJoints j = to_joints(cu);
            for (const FingerJoints& f : {j.little, j.ring, j.middle, j.index, j.thumb}) {
                CHECK(std::isfinite(f.splay));
                for (float v : f.joint_curls) {
                    CHECK(std::isfinite(v) && v >= 0.0f && v <= 2.0f);
                }
            }
        }
    }
}

TEST(timestamps_may_go_backwards_without_trouble) {
    Rng r{2};
    for (int c = 0; c < CASES; c++) {
        HandState s{Params{}};
        const Input i = clean_input(r);
        const std::uint64_t n = r.range(1, 100);
        for (std::uint64_t k = 0; k < n; k++) {
            for (float v : s.update(r.any64(), i).to_array()) {
                CHECK(in01(v));
            }
        }
    }
}

TEST(deterministic_and_shift_invariant) {
    Rng r{3};
    for (int c = 0; c < CASES; c++) {
        HandState a{Params{}};
        HandState b{Params{}};
        const std::uint64_t shift = r.below(1000000000000ull);
        std::uint64_t ta = 5 * MS;
        std::uint64_t tb = 5 * MS + shift;
        const std::uint64_t n = r.range(1, 150);
        for (std::uint64_t k = 0; k < n; k++) {
            const std::uint64_t dt_ms = r.range(1, 30);
            const Input i = clean_input(r);
            ta += dt_ms * MS;
            tb += dt_ms * MS;
            CHECK(a.update(ta, i) == b.update(tb, i));
        }
    }
}

TEST(steady_state_forgets_the_history) {
    Rng r{4};
    for (int c = 0; c < CASES; c++) {
        Input hold;
        hold.trigger = r.f(0.0f, 1.0f);
        hold.grip = r.f(0.0f, 1.0f);
        HandState a{Params{}};
        std::uint64_t t = 10 * MS;
        const std::uint64_t n = r.below(80);
        for (std::uint64_t k = 0; k < n; k++) {
            t += r.range(1, 30) * MS;
            a.update(t, clean_input(r));
        }
        HandState b{Params{}};
        std::uint64_t tb = 10 * MS;
        Curls ca = a.curls(), cb = b.curls();
        for (int k = 0; k < 600; k++) {
            t += 4 * MS;
            tb += 4 * MS;
            ca = a.update(t, hold);
            cb = b.update(tb, hold);
        }
        const auto x = ca.to_array();
        const auto y = cb.to_array();
        for (int k = 0; k < 5; k++) {
            CHECK(std::fabs(x[k] - y[k]) < 0.02f);
        }
    }
}

TEST(more_trigger_never_means_less_index_curl) {
    Rng r{5};
    const auto settle = [](float v) {
        HandState s{Params{}};
        Curls c = s.curls();
        for (std::uint64_t k = 1; k <= 300; k++) {
            Input i;
            i.trigger = v;
            c = s.update(k * 4 * MS, i);
        }
        return c;
    };
    for (int c = 0; c < CASES / 4; c++) {
        const float a = r.f(0.0f, 1.0f);
        const float b = r.f(0.0f, 1.0f);
        const float lo = a <= b ? a : b;
        const float hi = a <= b ? b : a;
        CHECK(settle(lo).index <= settle(hi).index + 1e-4f);
        CHECK(settle(lo).thumb == settle(hi).thumb);
    }
}

TEST(joint_curls_grow_with_curl) {
    Rng r{6};
    for (int c = 0; c < CASES; c++) {
        const float c1 = r.f(0.0f, 1.0f);
        const float c2 = r.f(0.0f, 1.0f);
        const float lo = c1 <= c2 ? c1 : c2;
        const float hi = c1 <= c2 ? c2 : c1;
        const HandJoints a = to_joints(Curls{lo, lo, lo, lo, lo});
        const HandJoints b = to_joints(Curls{hi, hi, hi, hi, hi});
        const FingerJoints* x[5] = {&a.little, &a.ring, &a.middle, &a.index, &a.thumb};
        const FingerJoints* y[5] = {&b.little, &b.ring, &b.middle, &b.index, &b.thumb};
        for (int f = 0; f < 5; f++) {
            for (int k = 0; k < 4; k++) {
                CHECK(x[f]->joint_curls[k] <= y[f]->joint_curls[k] + 1e-6f);
            }
        }
    }
}

PIBRIDGE_TEST_MAIN()
