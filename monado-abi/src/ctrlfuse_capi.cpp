// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-hands/src/fuse_ffi.rs (MIT OR Apache-2.0).
// The pimax_ctrlfuse_* C ABI (include/pimax_ctrlfuse.h) over libpibridge-ctrl's fuser. Every function tolerates NULL.
// One object per hand; all calls are serialised by an internal mutex (reports come from the session thread, queries
// from the compositor). The other hand (body model) is shared like the Rust Arc<Mutex<Fuser>> and never locked while
// this hand's lock is held.
#include "pimax_ctrlfuse.h"

#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/ctrl/fuse.h"

#include <algorithm>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>

#define PIBRIDGE_CTRLFUSE_ABI extern "C" __attribute__((visibility("default")))

namespace fu = pibridge::ctrl::fuse;
namespace rf = pibridge::cam::rfmt;

namespace {
struct Locked {
    std::mutex m;
    fu::Fuser f;
    Locked(std::size_t hand, fu::Config cfg, std::optional<fu::Calib> c) : f(hand, cfg, c) {}
};
} // namespace

struct PimaxCtrlFuse {
    std::shared_ptr<Locked> f;
    std::mutex other_m;
    std::shared_ptr<Locked> other_f;
    std::string path;

    std::shared_ptr<Locked> other() {
        std::lock_guard lk(other_m);
        return other_f;
    }
};

namespace {

fu::P to_p(const pimax_hands_pose& p) {
    const auto& o = p.orientation;
    return fu::P{fu::qnorm({o[0], o[1], o[2], o[3]}), {p.position[0], p.position[1], p.position[2]}};
}

pimax_hands_pose from_p(const fu::P& p) {
    pimax_hands_pose o;
    for (int k = 0; k < 4; k++) {
        o.orientation[k] = static_cast<float>(p.q[k]);
    }
    for (int k = 0; k < 3; k++) {
        o.position[k] = static_cast<float>(p.p[k]);
    }
    return o;
}

} // namespace

PIBRIDGE_CTRLFUSE_ABI pimax_ctrlfuse* pimax_ctrlfuse_create(int hand) {
    try {
        const std::size_t hd = hand == 0 ? 0 : 1;
        std::string path = fu::calib_path();
        auto l = std::make_shared<Locked>(hd, fu::Config::from_env(), fu::calib_or_default(path, hd));
        if (const auto prefix = pibridge::cam::env_os("PIMAX_CTRL_IMU_LOG"); prefix && !prefix->empty()) {
            std::string p = pibridge::cam::rfmt::utf8_valid(*prefix) ? *prefix : std::string();
            p += hd == 0 ? "-left.csv" : "-right.csv";
            double minutes = 10.0;
            if (const auto v = pibridge::cam::env_utf8("PIMAX_CTRL_IMU_LOG_MIN")) {
                double x = 0.0;
                if (rf::parse_f64(*v, x)) {
                    minutes = x;
                }
            }
            l->f.start_log(p, minutes);
        }
        auto* h = new PimaxCtrlFuse();
        h->f = std::move(l);
        h->path = std::move(path);
        return h;
    } catch (...) {
        return nullptr;
    }
}

PIBRIDGE_CTRLFUSE_ABI void pimax_ctrlfuse_destroy(pimax_ctrlfuse* h) {
    if (h == nullptr) {
        return;
    }
    try {
        std::lock_guard lk(h->f->m);
        h->f->f.maybe_save(UINT64_MAX, h->path, 0.0);
        h->f->f.flush_log();
    } catch (...) {
    }
    delete h;
}

PIBRIDGE_CTRLFUSE_ABI void pimax_ctrlfuse_report(pimax_ctrlfuse* h, uint64_t host_ns, const uint8_t* raw) {
    if (h == nullptr || raw == nullptr) {
        return;
    }
    try {
        std::lock_guard lk(h->f->m);
        fu::Fuser& f = h->f->f;
        f.report(host_ns, raw);
        if (f.stats.imu_samples % 2000 == 0) {
            f.maybe_save(host_ns, h->path, 30.0);
            f.flush_log();
        }
    } catch (...) {
    }
}

PIBRIDGE_CTRLFUSE_ABI void pimax_ctrlfuse_optical(pimax_ctrlfuse* h, uint64_t frame_ns, const pimax_hands_pose* world_from_aim,
                                               const pimax_hands_pose* world_from_head, float rms_px, uint32_t leds) {
    if (h == nullptr || world_from_aim == nullptr || world_from_head == nullptr) {
        return;
    }
    try {
        std::optional<fu::V3> other;
        if (const auto o = h->other()) {
            std::lock_guard lk(o->m);
            other = o->f.fresh_fix(frame_ns, 0.1);
        }
        std::lock_guard lk(h->f->m);
        h->f->f.optical_with(frame_ns, to_p(*world_from_aim), to_p(*world_from_head), rms_px, leds, other);
    } catch (...) {
    }
}

PIBRIDGE_CTRLFUSE_ABI void pimax_ctrlfuse_link(pimax_ctrlfuse* a, pimax_ctrlfuse* b) {
    if (a == nullptr || b == nullptr || a == b) {
        return;
    }
    {
        std::lock_guard lk(a->other_m);
        a->other_f = b->f;
    }
    {
        std::lock_guard lk(b->other_m);
        b->other_f = a->f;
    }
}

PIBRIDGE_CTRLFUSE_ABI int pimax_ctrlfuse_get(pimax_ctrlfuse* h, uint64_t at_ns, const pimax_hands_pose* world_from_head,
                                          pimax_hands_pose* out, pimax_ctrlfuse_info* info) {
    if (h == nullptr || world_from_head == nullptr || out == nullptr) {
        return 0;
    }
    try {
        const fu::P head = to_p(*world_from_head);
        std::optional<fu::V3> other;
        if (const auto o = h->other()) {
            std::lock_guard lk(o->m);
            other = o->f.peek(at_ns, head);
        }
        fu::State s;
        fu::P p;
        bool gyro;
        {
            std::lock_guard lk(h->f->m);
            std::tie(s, p) = h->f->f.get_with(at_ns, head, other);
            gyro = h->f->f.gyro_active();
        }
        *out = from_p(p);
        if (info != nullptr) {
            info->state = static_cast<uint32_t>(s);
            info->gyro = (gyro && s != fu::State::None) ? 1u : 0u;
        }
        return static_cast<int>(s);
    } catch (...) {
        return 0;
    }
}

PIBRIDGE_CTRLFUSE_ABI size_t pimax_ctrlfuse_describe(pimax_ctrlfuse* h, char* buf, size_t len) {
    if (h == nullptr || buf == nullptr || len == 0) {
        return 0;
    }
    try {
        std::string s;
        {
            std::lock_guard lk(h->f->m);
            const fu::Fuser& f = h->f->f;
            std::string cal;
            if (f.calib) {
                const fu::Calib& c = *f.calib;
                cal = "gyro calibrated (" + rf::fixed(c.scale_dps_per_lsb(), 4) + " deg/s per LSB, axes " + c.axes() +
                      ", offset " + rf::display_f64(c.time_offset_ms) + " ms, " + std::to_string(c.pairs) +
                      " pairs, r2 " + rf::fixed(c.r2, 3) + ")";
            } else {
                cal = "gyro not calibrated yet (learning from the optical poses)";
            }
            s = cal + "; gyro " + (f.gyro_active() ? "on" : "off") + "; " + std::to_string(f.stats.fixes) +
                " fixes, " + std::to_string(f.stats.rejected_body) + " dropped by the body model, " +
                std::to_string(f.stats.rejected) + " by the prediction, " + std::to_string(f.stats.reacquires) +
                " reacquires, " + std::to_string(f.stats.fit_updates) + " fit updates, out of view kept " +
                rf::fixed(f.cfg.lost_s, 1) + " s";
        }
        const std::size_t n = std::min(s.size(), len - 1);
        std::memcpy(buf, s.data(), n);
        buf[n] = 0;
        return n;
    } catch (...) {
        return 0;
    }
}
