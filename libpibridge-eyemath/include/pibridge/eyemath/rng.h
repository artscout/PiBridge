// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/rng.rs (MIT OR Apache-2.0).
// A tiny deterministic random source (xorshift64*) with a normal deviate, for simulations and tests (no dependency).
// The same seed gives the same numbers as the Rust code.
#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>

namespace pibridge::eyemath {

/// xorshift64* generator.
class Rng {
public:
    /// New generator; the seed 0 is replaced by a constant.
    explicit Rng(std::uint64_t seed) : s_(seed == 0 ? 0x9e37'79b9'7f4a'7c15ULL : seed) {}

    /// Next 64 random bits.
    std::uint64_t next_u64() {
        std::uint64_t x = s_;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        s_ = x;
        return x * 0x2545'f491'4f6c'dd1dULL;
    }

    /// Uniform in (0, 1).
    double uniform() {
        return (static_cast<double>(next_u64() >> 11) + 0.5) / static_cast<double>(std::uint64_t{1} << 53);
    }

    /// Standard normal (Box-Muller).
    double gauss() {
        const double a = uniform();
        const double b = uniform();
        return std::sqrt(-2.0 * std::log(a)) * std::cos(2.0 * std::numbers::pi * b);
    }

private:
    std::uint64_t s_;
};

} // namespace pibridge::eyemath
