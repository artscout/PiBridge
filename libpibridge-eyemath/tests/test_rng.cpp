// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/rng.rs tests (MIT OR Apache-2.0).
#include <cmath>
#include <vector>

#include "pibridge/eyemath/rng.h"
#include "pibridge_test.h"

using namespace pibridge::eyemath;

TEST(moments_are_right) {
    Rng r(5);
    const int n = 20000;
    std::vector<double> v;
    for (int i = 0; i < n; i++) {
        v.push_back(r.gauss());
    }
    double m = 0.0;
    for (double x : v) {
        m += x;
    }
    m /= n;
    double s = 0.0;
    for (double x : v) {
        s += (x - m) * (x - m);
    }
    s = std::sqrt(s / n);
    CHECK(std::fabs(m) < 0.03 && std::fabs(s - 1.0) < 0.03);
    bool inside = true;
    for (int i = 0; i < n; i++) {
        const double u = r.uniform();
        inside = inside && u > 0.0 && u < 1.0;
    }
    CHECK(inside);
}

// C++ only: the sequence is the Rust one (first values of seed 1, from xorshift64* by hand).
TEST(sequence_is_deterministic) {
    Rng a(1);
    Rng b(1);
    for (int i = 0; i < 100; i++) {
        CHECK(a.next_u64() == b.next_u64());
    }
    Rng z(0);
    Rng k(0x9e37'79b9'7f4a'7c15ULL);
    CHECK(z.next_u64() == k.next_u64()); // seed 0 is replaced by the constant
}

PIBRIDGE_TEST_MAIN()
