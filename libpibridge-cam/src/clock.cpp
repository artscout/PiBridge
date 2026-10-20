// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/clock.rs (MIT OR Apache-2.0).
#include "pibridge/cam/clock.h"

#include <ctime>
#include <limits>

namespace pibridge::cam {

namespace {

__extension__ typedef __int128 i128;

// Rust's release-mode i64 arithmetic wraps; do it on u64 to stay defined.
std::int64_t wadd(std::int64_t a, std::int64_t b) {
    return static_cast<std::int64_t>(static_cast<std::uint64_t>(a) + static_cast<std::uint64_t>(b));
}

std::int64_t wsub(std::int64_t a, std::int64_t b) {
    return static_cast<std::int64_t>(static_cast<std::uint64_t>(a) - static_cast<std::uint64_t>(b));
}

std::int64_t wmul(std::int64_t a, std::int64_t b) {
    return static_cast<std::int64_t>(static_cast<std::uint64_t>(a) * static_cast<std::uint64_t>(b));
}

} // namespace

std::uint64_t imu_dev_to_mono_ns(std::uint64_t dev_ts_us, const ImuClockRef& r) {
    const auto delta =
        static_cast<std::int64_t>(static_cast<std::int32_t>(static_cast<std::uint32_t>(dev_ts_us) - r.dev_ts_u32));
    const std::int64_t ext = wadd(static_cast<std::int64_t>(r.dev_ext_us), delta);
    const i128 ns = static_cast<i128>(ext) * 1000 + static_cast<i128>(r.offset_ns);
    if (ns < 0) {
        return 0;
    }
    if (ns > static_cast<i128>(std::numeric_limits<std::uint64_t>::max())) {
        return std::numeric_limits<std::uint64_t>::max();
    }
    return static_cast<std::uint64_t>(ns);
}

std::uint64_t HostClockMap::map(std::uint64_t dev_us, std::uint64_t host_ns) {
    const bool jump = have_ && (dev_us < last_dev_us_ || dev_us - last_dev_us_ > 2'000'000);
    if (jump) {
        resets++;
    }
    const std::int64_t cand = wsub(static_cast<std::int64_t>(host_ns), wmul(static_cast<std::int64_t>(dev_us), 1000));
    if (!have_ || jump) {
        have_ = true;
        offset_ns_ = cand;
    } else if (cand < offset_ns_) {
        offset_ns_ = cand;
    } else {
        offset_ns_ = wadd(offset_ns_, wsub(cand, offset_ns_) >> RELAX);
    }
    last_dev_us_ = dev_us;
    const std::int64_t v = wadd(wmul(static_cast<std::int64_t>(dev_us), 1000), offset_ns_);
    std::uint64_t t = v < 0 ? 0 : static_cast<std::uint64_t>(v);
    if (!jump && t <= last_out_ns_) {
        t = last_out_ns_ + 1;
    }
    last_out_ns_ = t;
    return t;
}

std::uint64_t monotonic_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000'000ull + static_cast<std::uint64_t>(ts.tv_nsec);
}

} // namespace pibridge::cam
