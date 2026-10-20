// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-hands/src/inhand.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/inhand.h"

#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/ctrl/fuse.h"
#include "pibridge/ctrl/report.h"
#include "pibridge/ctrl/rust.h"

#include <cmath>
#include <cstdlib>
#include <string>

namespace pibridge::ctrl::inhand {

namespace rf = pibridge::cam::rfmt;

Config Config::from_env() {
    Config c;
    const auto v = pibridge::cam::env_utf8("PIMAX_CTRL_INHAND_DOWN_S");
    double s = 0.0;
    if (v && rf::parse_f64(rf::trim(*v), s) && 1.0 <= s && s <= 60.0) {
        const double win_s = static_cast<double>(c.block_ms) * static_cast<double>(c.window_blocks) / 1000.0;
        c.down_windows = rs::f64_to_usize(rf::fmax(std::round(s / win_s), 2.0));
        c.down_still = rs::f64_to_usize(std::ceil(static_cast<double>(c.down_windows) * 0.8));
        c.input_hold_s = s;
    }
    return c;
}

std::optional<Mode> parse_mode(std::string_view s) {
    std::string t(rf::trim(s));
    for (char& ch : t) {
        if (ch >= 'A' && ch <= 'Z') {
            ch = static_cast<char>(ch - 'A' + 'a');
        }
    }
    if (t == "auto" || t == "when-held" || t == "when_held" || t == "held" || t == "picked-up") {
        return Mode::Auto;
    }
    if (t == "always" || t == "on" || t == "1") {
        return Mode::Always;
    }
    if (t == "off" || t == "never" || t == "0") {
        return Mode::Off;
    }
    return std::nullopt;
}

const char* mode_name(Mode m) {
    switch (m) {
    case Mode::Auto: return "auto";
    case Mode::Always: return "always";
    case Mode::Off: return "off";
    }
    return "auto";
}

const char* why_name(Why w) {
    switch (w) {
    case Why::Start: return "start";
    case Why::Motion: return "motion";
    case Why::Tilt: return "tilt";
    case Why::Input: return "input";
    case Why::Still: return "still";
    case Why::NoReports: return "no reports";
    }
    return "start";
}

namespace {

using V3 = std::array<double, 3>;

double std3(const std::vector<std::pair<V3, V3>>& v, bool gyro) {
    const double n = static_cast<double>(v.empty() ? 1 : v.size());
    double out = 0.0;
    for (int k = 0; k < 3; k++) {
        const auto x = [&](const std::pair<V3, V3>& e) { return gyro ? e.first[k] : e.second[k]; };
        double s = -0.0;
        for (const auto& e : v) {
            s += x(e);
        }
        const double m = s / n;
        double s2 = -0.0;
        for (const auto& e : v) {
            const double d = x(e) - m;
            s2 += d * d;
        }
        out = rf::fmax(out, std::sqrt(s2 / n));
    }
    return out;
}

double angle_deg(V3 a, V3 b) {
    const double na = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    const double nb = std::sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
    if (na < 1e-6 || nb < 1e-6) {
        return 0.0;
    }
    const double c = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (na * nb);
    return rf::to_degrees(std::acos(rf::clamp(c, -1.0, 1.0)));
}

std::int32_t iabs(std::int32_t v) {
    return v < 0 ? static_cast<std::int32_t>(0u - static_cast<std::uint32_t>(v)) : v;
}

} // namespace

InHand::InHand(Config cfg_) : cfg(cfg_) {
    win_.reserve(cfg.window_blocks);
}

void InHand::set(bool held, Why why, std::uint64_t t) {
    if (held_ == held) {
        return;
    }
    held_ = held;
    why_ = why;
    changed_ns_ = t;
    move_run_ = 0;
    tilt_run_ = 0;
    ring_.clear();
    if (held) {
        counts.first++;
    } else {
        counts.second++;
    }
}

void InHand::input(std::uint64_t t_ns, std::uint8_t trigger, std::uint8_t grip, std::int32_t stick_x,
                   std::int32_t stick_y, std::uint32_t buttons, std::uint8_t touch) {
    report_seen(t_ns);
    const detail::Inputs now{trigger, grip, stick_x, stick_y, buttons,
                             static_cast<std::uint8_t>(touch & (report::TOUCH_A | report::TOUCH_B | report::TOUCH_STICK))};
    const std::optional<detail::Inputs> prev = inputs_;
    inputs_ = now;
    if (!prev) {
        return; // first report: nothing to compare with
    }
    const detail::Inputs old = *prev;
    const std::int32_t step = cfg.analog_step;
    // (Rust i32 arithmetic in a release build wraps; the inputs are small enough not to.)
    const bool changed = now.buttons != old.buttons || now.touch != old.touch ||
                         iabs(now.trigger - old.trigger) >= step || iabs(now.grip - old.grip) >= step ||
                         iabs(static_cast<std::int32_t>(static_cast<std::uint32_t>(now.stick_x) -
                                                        static_cast<std::uint32_t>(old.stick_x))) >= step ||
                         iabs(static_cast<std::int32_t>(static_cast<std::uint32_t>(now.stick_y) -
                                                        static_cast<std::uint32_t>(old.stick_y))) >= step;
    if (changed) {
        last_input_ns_ = t_ns;
        set(true, Why::Input, t_ns);
    } else if (step > 1) {
        // slow analog drift: compare with the value at the last counted change
        detail::Inputs keep = now;
        keep.trigger = old.trigger;
        keep.grip = old.grip;
        keep.stick_x = old.stick_x;
        keep.stick_y = old.stick_y;
        inputs_ = keep;
    }
}

void InHand::sample(std::uint64_t t_ns, std::array<std::int16_t, 3> g, std::array<std::int16_t, 3> a) {
    report_seen(t_ns);
    if (!blk_start_) {
        blk_start_ = t_ns;
    }
    const std::uint64_t start = *blk_start_;
    if (t_ns >= start + cfg.block_ms * 1000000u || t_ns < start) {
        if (blk_n_ > 0) {
            const double n = static_cast<double>(blk_n_);
            const V3 gm = {blk_g_[0] / n, blk_g_[1] / n, blk_g_[2] / n};
            const V3 am = {blk_a_[0] / n, blk_a_[1] / n, blk_a_[2] / n};
            block(t_ns, gm, am);
        }
        blk_start_ = t_ns;
        blk_n_ = 0;
        blk_g_ = {0.0, 0.0, 0.0};
        blk_a_ = {0.0, 0.0, 0.0};
    }
    blk_n_++;
    for (int k = 0; k < 3; k++) {
        blk_g_[k] += static_cast<double>(g[k]);
        blk_a_[k] += static_cast<double>(a[k]);
    }
}

void InHand::block(std::uint64_t t, V3 g, V3 a) {
    if (!bias_) {
        bias_ = g;
    }
    const V3 bias = *bias_;
    if (!rest_a_) {
        rest_a_ = a;
    }
    const V3 rest = *rest_a_;
    win_.emplace_back(g, a);
    if (win_.size() >= cfg.window_blocks) {
        const double gs = std3(win_, true);
        const double as_ = std3(win_, false);
        const bool still = gs < cfg.still_lsb && as_ < cfg.still_lsb;
        if (still) {
            const double n = static_cast<double>(win_.size());
            V3 gm = {0.0, 0.0, 0.0};
            V3 am = {0.0, 0.0, 0.0};
            for (const auto& [wg, wa] : win_) {
                for (int k = 0; k < 3; k++) {
                    gm[k] += wg[k] / n;
                    am[k] += wa[k] / n;
                }
            }
            const V3 b = bias_ ? *bias_ : gm;
            bias_ = V3{0.8 * b[0] + 0.2 * gm[0], 0.8 * b[1] + 0.2 * gm[1], 0.8 * b[2] + 0.2 * gm[2]};
            if (!held_) {
                rest_a_ = am;
            }
        }
        win_.clear();
        if (ring_.size() == cfg.down_windows) {
            ring_.pop_front();
        }
        ring_.emplace_back(still, gs);
        if (held_ && ring_.size() == cfg.down_windows) {
            std::size_t n_still = 0;
            bool calm = true;
            for (const auto& w : ring_) {
                n_still += w.first ? 1 : 0;
                calm = calm && w.second < cfg.motion_lsb;
            }
            const bool quiet_input =
                !last_input_ns_ || static_cast<double>(rs::sat_sub(t, *last_input_ns_)) * 1e-9 >= cfg.input_hold_s;
            if (n_still >= cfg.down_still && calm && quiet_input) {
                set(false, Why::Still, t);
                rest_a_ = a;
            }
        }
    }
    if (!held_) {
        double mo = 0.0;
        for (int k = 0; k < 3; k++) {
            mo = rf::fmax(mo, std::fabs(g[k] - bias[k]));
        }
        move_run_ = mo > cfg.move_lsb ? move_run_ + 1 : 0;
        tilt_run_ = angle_deg(a, rest) > cfg.tilt_deg ? tilt_run_ + 1 : 0;
        if (move_run_ >= cfg.move_blocks) {
            set(true, Why::Motion, t);
        } else if (tilt_run_ >= cfg.tilt_blocks) {
            set(true, Why::Tilt, t);
        }
    }
}

void InHand::report(std::uint64_t host_ns, const std::uint8_t* raw, std::uint8_t trigger, std::uint8_t grip,
                    std::int32_t stick_x, std::int32_t stick_y, std::uint32_t buttons, std::uint8_t touch) {
    const auto gy = fuse::report_gyro(raw);
    const auto ac = fuse::report_accel(raw);
    for (std::size_t k = 0; k < gy.n; k++) {
        sample(rs::sat_sub(host_ns, static_cast<std::uint64_t>(gy.n - 1 - k) * 2000000u), gy.g[k], ac[k]);
    }
    input(host_ns, trigger, grip, stick_x, stick_y, buttons, touch);
}

bool InHand::held(std::uint64_t now_ns) {
    if (held_) {
        const bool silent =
            !last_report_ns_ || static_cast<double>(rs::sat_sub(now_ns, *last_report_ns_)) * 1e-9 > cfg.no_report_s;
        if (silent) {
            set(false, Why::NoReports, now_ns);
        }
    }
    return held_;
}

} // namespace pibridge::ctrl::inhand
