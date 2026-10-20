// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/source.rs (MIT OR Apache-2.0).
#include "pibridge/cam/source.h"

#include "pibridge/cam/broker.h"
#include "pibridge/cam/record.h"
#include "pibridge/cam/resilient.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/cam/v4l2.h"

#include <cerrno>
#include <format>

namespace pibridge::cam {

bool is_broker_spec(std::string_view spec) {
    const std::string_view s = rfmt::trim(spec);
    if (!s.starts_with("broker")) {
        return false;
    }
    const std::string_view r = s.substr(6);
    return r.empty() || r.front() == ':';
}

Result<std::unique_ptr<Source>> open_source(std::string_view spec, std::uint32_t fps, ReplayOptions replay) {
    auto first = open_once(spec, fps, replay);
    if (!first) {
        return first;
    }
    if ((*first)->is_replay() || !reconnect_enabled()) {
        return first;
    }
    const bool broker = is_broker_spec(spec);
    Policy policy = broker ? Policy::broker() : Policy::camera();
    const char* label = broker ? "camera broker" : "camera";
    std::string s(spec);
    Opener reopen = [s, fps, replay]() { return open_once(s, fps, replay); };
    return std::unique_ptr<Source>(new Resilient(std::move(*first), std::move(reopen), policy, label));
}

Result<std::unique_ptr<Source>> open_once(std::string_view spec_in, std::uint32_t fps, ReplayOptions replay) {
    const std::string_view spec = rfmt::trim(spec_in);
    if (spec.starts_with("broker")) {
        std::string_view rest = spec.substr(6);
        if (rest.empty() || rest.front() == ':') {
            while (!rest.empty() && rest.front() == ':') {
                rest.remove_prefix(1);
            }
            const std::string path = rest.empty() ? broker::default_socket() : std::string(rest);
            auto b = broker::BrokerSource::connect(path);
            if (!b) {
                return fail(b.error());
            }
            return std::unique_ptr<Source>(std::move(*b));
        }
    }
    if (spec.empty() || spec == "auto") {
        const auto dev = find_device();
        if (!dev) {
            return fail(
                IoError::custom(ErrorKind::NotFound, "camera 34a4:0014 not found under /sys/class/video4linux"));
        }
        auto c = V4l2Capture::open(*dev, fps, 6);
        if (!c) {
            const char* hint = c.error().os == EBUSY
                                   ? " (busy: another process is streaming it, e.g. a running omarchyvr-passthrough)"
                                   : "";
            return fail(IoError::custom(c.error().kind,
                                        std::format("{}: {}{}", rfmt::utf8_lossy(*dev), c.error().text(), hint)));
        }
        return std::unique_ptr<Source>(std::move(*c));
    }
    if (spec.starts_with("/dev/")) {
        auto c = V4l2Capture::open(std::string(spec), fps, 6);
        if (!c) {
            return fail(c.error());
        }
        return std::unique_ptr<Source>(std::move(*c));
    }
    auto r = Replay::open(std::string(spec), replay);
    if (!r) {
        return fail(r.error());
    }
    return std::unique_ptr<Source>(std::move(*r));
}

} // namespace pibridge::cam
