// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-capi/src/eye.rs (MIT OR Apache-2.0).
// The eye gaze C ABI (include/pimax_eye.h) over libpibridge-eye's live tracker. Thin, as in Rust: everything is in
// pibridge::eye::live and libpibridge-eyemath. The tracker reads the PIMAX_EYE_* environment (PIBRIDGE_EYE_* first) once at
// start. Its one HID command, the captured eye-module enable (`eye_power_on`), goes through the connected model's
// allowlist gate (pimax_internal.h) to the headset's interface 0, opened for that one write as the Rust
// power::send_enable does. For the Crystal Super the eye runs through pibridge-eyed (one owner of the chip): Monado is
// started without PIMAX_EYE there, so nothing here runs.
#include "pimax_eye.h"

#include "pimax_internal.h"

#include "pibridge/eye/live.h"
#include "pibridge/eye/power.h"
#include "pibridge/eyemath/clock.h"
#include "pibridge/eyemath/drift.h"
#include "pibridge/hmd/discovery.h"
#include "pibridge/hmd/sys.h"

#include <cerrno>
#include <cstdio>
#include <memory>
#include <string>

#include <unistd.h>

#define PIBRIDGE_EYE_ABI extern "C" __attribute__((visibility("default")))

namespace live = pibridge::eye::live;
namespace hmd = pibridge::hmd;

struct pimax_eye {
    std::unique_ptr<live::Tracker> tracker;
};

namespace {

/// A newest sample older than this makes latest / sample_at report PIMAX_EYE_STALE.
constexpr std::int64_t STALE_NS = 300'000'000;

// power::HeadsetCommands over libpibridge-hmd: the headset on hidraw now, its allowlist, one gated write.
class GatedCommands final : public pibridge::eye::power::HeadsetCommands {
public:
    pibridge::eye::Result<void> send(std::string_view id) override {
        using pibridge::eye::Errc;
        using E = pibridge::eye::Error;
        const auto roots = hmd::discovery::Roots::from_env();
        const auto found = hmd::discovery::find_headset(roots);
        if (!found || found->model == nullptr) {
            return std::unexpected(E::make(Errc::not_found, "no Pimax headset (34a4:0040 or 34a4:0044) on hidraw"));
        }
        const auto allow = pibridge::abi::device_allowlist(*found->model);
        if (!allow) {
            return std::unexpected(E::make(Errc::not_found, "no device description: " + allow.error().msg));
        }
        const hmd::CommandSpec* spec = (*allow)->get(id);
        if (spec == nullptr) {
            return std::unexpected(E::make(Errc::permission_denied, "'" + std::string(id) + "' is not in the allowlist"));
        }
        const auto report = (*allow)->build(id);
        if (!report) {
            return std::unexpected(E::make(Errc::permission_denied, report.error().message()));
        }
        std::optional<std::filesystem::path> node;
        if (spec->interface == 0) {
            node = pibridge::eye::power::headset_command_node(roots.hidraw_class, roots.dev);
        }
        if (!node && spec->interface < found->nodes.size()) {
            node = found->nodes[spec->interface];
        }
        if (!node) {
            return std::unexpected(E::make(Errc::not_found, "no hidraw node for the command interface"));
        }
        auto fd = hmd::hid::open(*node, true);
        if (!fd) {
            return std::unexpected(E::make(Errc::io, "open " + node->string() + ": " + fd.error().message()));
        }
        const int raw = fd->get();
        hmd::Gate gate(**allow, spec->interface, [raw](std::span<const std::uint8_t> b) -> long {
            const auto n = ::write(raw, b.data(), b.size());
            return n < 0 ? -static_cast<long>(errno) : static_cast<long>(n);
        });
        if (auto r = gate.send(*report); !r) {
            if (const auto* ref = r.error().refusal()) {
                return std::unexpected(E::make(Errc::permission_denied, ref->message()));
            }
            return std::unexpected(E::os(std::get<int>(r.error().e), "write " + node->string()));
        }
        return {};
    }
};

pimax_eye_sample to_c(const pibridge::eyemath::Sample& s) {
    pimax_eye_sample o{};
    o.seq = s.seq;
    o.device_ts_us = s.device_ts_us;
    o.host_ts_ns = s.host_ts_ns;
    if (s.gaze) {
        o.flags |= PIMAX_EYE_GAZE_VALID;
        for (int k = 0; k < 3; k++) {
            o.origin[k] = s.gaze->origin[k];
            o.direction[k] = s.gaze->direction[k];
        }
    }
    for (int i = 0; i < 2; i++) {
        for (int k = 0; k < 3; k++) {
            o.eye_origin[i][k] = s.eyes[i].origin[k];
            o.eye_direction[i][k] = s.eyes[i].direction[k];
        }
        o.pupil_mm[i] = s.eyes[i].pupil_mm;
        if (s.eyes[i].origin_valid && s.eyes[i].direction_valid) {
            o.flags |= i == 0 ? PIMAX_EYE_LEFT_VALID : PIMAX_EYE_RIGHT_VALID;
        }
    }
    return o;
}

int fill(const std::optional<pibridge::eyemath::Sample>& newest, const std::optional<pibridge::eyemath::Sample>& picked,
         pimax_eye_sample* out) {
    if (!newest || !picked) {
        return PIMAX_EYE_NO_SAMPLE;
    }
    *out = to_c(*picked);
    // staleness is judged against the real clock, by the newest sample's arrival
    return pibridge::eyemath::monotonic_ns() - newest->arrival_ns > STALE_NS ? PIMAX_EYE_STALE : PIMAX_EYE_OK;
}

} // namespace

PIBRIDGE_EYE_ABI uint32_t pimax_eye_abi_version(void) { return PIMAX_EYE_ABI_VERSION; }

PIBRIDGE_EYE_ABI pimax_eye* pimax_eye_start(uint32_t flags) {
    try {
        auto cfg = live::Config::from_env();
        cfg.with_cal = cfg.with_cal || (flags & PIMAX_EYE_START_WITH_CAL) != 0;
        auto t = live::Tracker::start(std::move(cfg),
                                      std::make_unique<live::RealDevice>(std::make_shared<GatedCommands>()));
        if (!t) {
            return nullptr;
        }
        return new pimax_eye{std::move(t)};
    } catch (...) {
        return nullptr;
    }
}

PIBRIDGE_EYE_ABI void pimax_eye_stop(pimax_eye* h) {
    if (h == nullptr) {
        return;
    }
    try {
        h->tracker.reset(); // joins the thread, which unsubscribes and releases the interface
    } catch (...) {
    }
    delete h;
}

PIBRIDGE_EYE_ABI int pimax_eye_state(const pimax_eye* h) {
    if (h == nullptr || !h->tracker) {
        return static_cast<int>(live::State::Stopped);
    }
    return static_cast<int>(h->tracker->state());
}

PIBRIDGE_EYE_ABI int pimax_eye_latest(const pimax_eye* h, pimax_eye_sample* out) {
    if (h == nullptr || !h->tracker || out == nullptr) {
        return PIMAX_EYE_BAD_ARG;
    }
    try {
        const auto l = h->tracker->latest();
        return fill(l, l, out);
    } catch (...) {
        return PIMAX_EYE_NO_SAMPLE;
    }
}

PIBRIDGE_EYE_ABI int pimax_eye_sample_at(const pimax_eye* h, int64_t at_ns, pimax_eye_sample* out) {
    if (h == nullptr || !h->tracker || out == nullptr) {
        return PIMAX_EYE_BAD_ARG;
    }
    try {
        return fill(h->tracker->latest(), h->tracker->sample_at(at_ns), out);
    } catch (...) {
        return PIMAX_EYE_NO_SAMPLE;
    }
}

/// Drift-correction hook: the wearer was looking along dir (head frame, unit) at at_ns. The number of eyes that used
/// the observation (0..2), PIMAX_EYE_BAD_ARG for NULL pointers, PIMAX_EYE_NO_SAMPLE without a calibration or a sample
/// at that time.
PIBRIDGE_EYE_ABI int pimax_eye_observe_target(const pimax_eye* h, int64_t at_ns, const float dir[3]) {
    if (h == nullptr || !h->tracker || dir == nullptr) {
        return PIMAX_EYE_BAD_ARG;
    }
    try {
        const pibridge::eyemath::Vec3 d{static_cast<double>(dir[0]), static_cast<double>(dir[1]),
                                     static_cast<double>(dir[2])};
        const auto n = h->tracker->observe_target(at_ns, d, pibridge::eyemath::DriftParams{});
        return n ? static_cast<int>(*n) : PIMAX_EYE_NO_SAMPLE;
    } catch (...) {
        return PIMAX_EYE_NO_SAMPLE;
    }
}
