// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/capi_ae.rs (MIT OR Apache-2.0).
//
// The "Auto-exposure" section of include/pimax_cam.h over libpibridge-cam's ae (M3): pimax_cam_ae_create,
// pimax_cam_ae_create_model, pimax_cam_ae_destroy, pimax_cam_ae_update.
//
// The driver calls pimax_cam_ae_update after every frame; when it returns 1 the driver may send out->payload as HID
// command 0x51 option 0 (58 19 51 00 + payload). Whether it really sends is the driver's decision (PIMAX_CAM_AE=1;
// "dry" = compute and log only). Nothing here touches the device. For a model with Windows ranges (the Crystal
// Super) a setting outside them is never handed out ("not sent" line); PiBridge's allowlist gate (`camera_ae`) checks
// the report again. Same stderr lines as the Rust library: "PIMAX_AE_MODEL: ..." and "PIMAX_AE_EXTENDED: ..." at
// create (build markers), the governor's events, the low-light toast to WayVR (127.0.0.1:42069).
//
// Every update also publishes the darker pair's mean to the head tracker (headpose::scene, PIMAX_FUSE_AUTO).
#include "pimax_cam.h"

#include "capi_common.h"

#include "pibridge/cam/ae.h"
#include "pibridge/cam/camera.h"
#include "pibridge/cam/headpose.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <format>
#include <string>

namespace ae = pibridge::cam::ae;
namespace rf = pibridge::cam::rfmt;

// The handle (the header only declares it).
struct pimax_cam_ae {
    ae::AutoExposure ae;
    ae::Mask mask;
    std::uint64_t refused = 0;
};

using namespace pibridge::abi;

namespace {

// The out struct as the Rust #[repr(C)] CAeOut (capi_ae.rs out_layout_is_stable).
static_assert(sizeof(pimax_cam_ae_out) == 8 + 24 + 24 + 8 + 8 + 4 + 8 + 8 + 8 + 8 + 8 + 4);
static_assert(offsetof(pimax_cam_ae_out, payload) == 32);
static_assert(offsetof(pimax_cam_ae_out, mean) == 92);
static_assert(PIMAX_CAM_AE_PAYLOAD_LEN == ae::AE_PAYLOAD_LEN);
static_assert(PIMAX_CAM_AE_CMD == ae::AE_CMD && PIMAX_CAM_AE_OPT == ae::AE_OPT);

pimax_cam_ae* create(ae::AeModel model, const std::string& why) {
    ae::AutoExposure a(ae::AeParams::from_env_model(model));
    // Once-at-start markers (also in the binary: the build check greps for them).
    std::fprintf(stderr, "%s\n", ae::model_line(a, why).c_str());
    std::fprintf(stderr, "%s\n", ae::start_line(a).c_str());
    return new pimax_cam_ae{std::move(a), ae::Mask{}, 0};
}

/// Log the event; on a raised warning also show it in the headset (WayVR's XSOverlay-format UDP listener on
/// 127.0.0.1:42069; nothing listens in a SteamVR game session, then only the log and the app show it).
void report(const ae::LightEvent& ev, const ae::AutoExposure& a) {
    std::fprintf(stderr, "%s\n", ae::event_line(ev, a).c_str());
    if (ev.kind == ae::LightEvent::LowRaised) {
        ae::toast(ae::LOW_LIGHT_TEXT);
    }
}

} // namespace

PIBRIDGE_ABI pimax_cam_ae* pimax_cam_ae_create(void) {
    return guard<pimax_cam_ae*>(nullptr, []() -> std::expected<pimax_cam_ae*, Err> {
        const auto [m, why] = ae::model_detect();
        return create(m, why);
    });
}

PIBRIDGE_ABI pimax_cam_ae* pimax_cam_ae_create_model(const char* model) {
    return guard<pimax_cam_ae*>(nullptr, [&]() -> std::expected<pimax_cam_ae*, Err> {
        const std::string name = model == nullptr ? std::string() : rf::utf8_lossy(model);
        std::optional<ae::AeModel> env;
        if (const auto v = pibridge::cam::env_utf8("PIMAX_CAM_AE_MODEL")) {
            env = ae::model_parse(*v);
        }
        if (rf::trim(name).empty() || env) {
            const auto [m, why] = ae::model_detect();
            return create(m, why);
        }
        const auto m = ae::model_parse(name);
        if (!m) {
            return err(std::format("unknown headset model {} (dream-air or crystal-super)", rf::debug_str(name)));
        }
        return create(*m, "the driver");
    });
}

PIBRIDGE_ABI void pimax_cam_ae_destroy(pimax_cam_ae* a) {
    try {
        delete a;
    } catch (...) {
    }
}

PIBRIDGE_ABI int pimax_cam_ae_update(pimax_cam_ae* a, const pimax_cam* cam, uint64_t now_ns, pimax_cam_ae_out* out) {
    return guard<int>(-1, [&]() -> std::expected<int, Err> {
        if (a == nullptr) {
            return err("null controller");
        }
        if (cam == nullptr) {
            return err("null camera");
        }
        if (out == nullptr) {
            return err("null output");
        }
        const pibridge::cam::FrameInfo fi = cam->cam->info();
        // Only live tracking frames with metadata drive the loop (a replay cannot be controlled).
        if (fi.kind != static_cast<std::uint32_t>(pibridge::cam::FrameKind::Tracking) ||
            (fi.flags & pibridge::cam::FLAG_META_VALID) == 0 || (fi.flags & pibridge::cam::FLAG_REPLAY) != 0) {
            return 0;
        }
        std::array<pibridge::cam::TileView, 4> tiles{};
        for (std::size_t t = 0; t < 4; t++) {
            auto v = cam->cam->tile_view(t);
            if (!v) {
                return err(v.error().text());
            }
            tiles[t] = *v;
        }
        const ae::AeSetting setting = ae::AeSetting::from_info(fi, a->ae.p.led_exposure, a->ae.p.led_gain);
        const auto stats = a->ae.frame_stats(tiles, a->mask);
        const ae::AeDecision d = a->ae.update(setting, stats, now_ns);
        for (const auto& ev : a->ae.take_events()) {
            report(ev, a->ae);
        }
        const ae::AeSetting tgt = a->ae.target().value_or(setting);
        // Last guard: a model with Windows ranges (Crystal Super) never hands out a setting outside them.
        const bool allowed = ae::model_allows(a->ae.p.model, tgt);
        if (!allowed) {
            a->refused += 1;
            if (a->refused == 1 || a->refused % 1000 == 0) {
                std::fprintf(stderr, "PIMAX_AE_MODEL: %s setting %s/%s outside the Windows ranges, not sent (%llu so far)\n",
                             ae::model_id(a->ae.p.model), rf::debug_u16s(tgt.exposure).c_str(),
                             rf::debug_u16s(tgt.gain).c_str(), static_cast<unsigned long long>(a->refused));
            }
        }
        const bool send = d.send && allowed;
        pimax_cam_ae_out o{};
        o.send = send ? 1 : 0;
        o.changed = d.changed ? 1 : 0;
        o.frames = a->ae.frames;
        o.sends = a->ae.sends;
        o.changes = a->ae.changes;
        const auto pl = ae::payload(tgt);
        std::memcpy(o.payload, pl.data(), pl.size());
        for (std::size_t i = 0; i < 4; i++) {
            o.exposure[i] = tgt.exposure[i];
            o.gain[i] = tgt.gain[i];
            o.frame_exposure[i] = fi.exposure[i];
            o.frame_gain[i] = fi.gain[i];
        }
        o.led_exposure = tgt.led_exposure;
        o.led_gain = tgt.led_gain;
        for (std::size_t k = 0; k < 2; k++) {
            o.mean[k] = static_cast<float>(d.stats[k].trimmed_mean);
            o.sat[k] = static_cast<float>(d.stats[k].sat_frac);
            o.err[k] = static_cast<float>(d.err[k]);
        }
        o.light = static_cast<std::uint32_t>(a->ae.light.stage) | (static_cast<std::uint32_t>(a->ae.light.low) << 2);
        // The head tracker lowers its fusion weights in a dark image (headpose, PIMAX_FUSE_AUTO).
        pibridge::cam::headpose::scene::publish(static_cast<double>(std::fmin(o.mean[0], o.mean[1])));
        *out = o;
        return send ? 1 : 0;
    });
}
