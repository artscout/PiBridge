// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/clock.rs (MIT OR Apache-2.0).
#include "pibridge/eyemath/clock.h"

#include <ctime>

#include "num.h"

namespace pibridge::eyemath {

std::int64_t monotonic_ns() {
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::int64_t>(ts.tv_sec) * 1'000'000'000 + static_cast<std::int64_t>(ts.tv_nsec);
}

void ClockMap::reset() {
    const auto lat = latency_ns;
    *this = ClockMap(alpha_);
    latency_ns = lat;
}

std::int64_t ClockMap::map(std::int64_t dev_us, std::int64_t arrival_ns) {
    constexpr auto none = std::numeric_limits<std::int64_t>::min();
    // device clock went backwards by more than a second: new epoch
    if (last_dev_us_ != none && dev_us < detail::wrapping_sub(last_dev_us_, 1'000'000)) {
        reset();
    }
    last_dev_us_ = dev_us;
    const double x = static_cast<double>(arrival_ns) - static_cast<double>(dev_us) * 1000.0;
    double o = x;
    if (offset_ns_ && !(x < *offset_ns_)) {
        o = *offset_ns_ + alpha_ * (x - *offset_ns_);
    }
    offset_ns_ = o;
    std::int64_t out =
        detail::wrapping_sub(detail::sat_cast<std::int64_t>(static_cast<double>(dev_us) * 1000.0 + o), latency_ns);
    if (out <= last_out_ns_) {
        out = last_out_ns_ + 1000;
    }
    last_out_ns_ = out;
    return out;
}

} // namespace pibridge::eyemath
