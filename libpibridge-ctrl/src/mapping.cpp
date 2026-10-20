// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-hands/src/mapping.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/mapping.h"

#include "pibridge/ctrl/rust.h"

#include <bit>
#include <cmath>

namespace pibridge::ctrl {

namespace {

/// Longest step fed to the filters, seconds (a stalled thread must not make them jump).
constexpr float k_max_dt_s = 0.1f;

constexpr std::size_t BTN1 = 0;
constexpr std::size_t BTN2 = 1;
constexpr std::size_t STICK = 2;
constexpr std::size_t MENU = 3;

float clamp01(float x) {
    return std::isfinite(x) ? rs::clampf(x, 0.0f, 1.0f) : 0.0f;
}

/// Replace NaN/inf/out-of-range values.
Input sanitize(const Input& i) {
    const auto s = [](float v) { return std::isfinite(v) ? rs::clampf(v, -1.0f, 1.0f) : 0.0f; };
    return Input{clamp01(i.trigger), clamp01(i.grip), s(i.stick_x), s(i.stick_y), i.flags & flags::ALL};
}

struct Evidence {
    bool touch;
    bool firm;
    bool pressed;
};

void contact_update(detail::Contact& c, Evidence e, float dt, double now, float engage, float hold, bool immediate) {
    c.clicked = e.pressed;
    if (e.firm || (e.touch && immediate)) {
        if (!c.active) {
            c.active = true;
            c.since = now;
        }
        c.idle = 0.0f;
        c.pending = 0.0f;
    } else if (e.touch) {
        c.idle = 0.0f;
        if (c.active) {
            return;
        }
        c.pending += dt;
        if (c.pending >= engage) {
            c.active = true;
            c.since = now - static_cast<double>(c.pending);
        }
    } else {
        c.pending = 0.0f;
        if (c.active) {
            c.idle += dt;
            if (c.idle >= hold) {
                c.active = false;
                c.idle = 0.0f;
            }
        }
    }
}

/// f64::total_cmp: -1, 0, 1.
int total_cmp(double a, double b) {
    auto key = [](double x) {
        auto i = std::bit_cast<std::int64_t>(x);
        i ^= static_cast<std::int64_t>(static_cast<std::uint64_t>(i >> 63) >> 1);
        return i;
    };
    const auto ka = key(a);
    const auto kb = key(b);
    return ka < kb ? -1 : (ka > kb ? 1 : 0);
}

ThumbState from_index(std::size_t i) {
    switch (i) {
    case 0: return ThumbState::Btn1;
    case 1: return ThumbState::Btn2;
    case 2: return ThumbState::Stick;
    case 3: return ThumbState::Menu;
    default: return ThumbState::Rest;
    }
}

} // namespace

float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

float shape(float x, float deadzone) {
    const float dz = rs::clampf(deadzone, 0.0f, 0.5f);
    return rs::clampf((x - dz) / (1.0f - dz), 0.0f, 1.0f);
}

HandState::HandState(Params params)
    : params_(params), trigger_f_(params.trigger_min_cutoff, params.trigger_beta, params.d_cutoff),
      grip_f_(params.grip_min_cutoff, params.grip_beta, params.d_cutoff),
      thumb_sd_(params.thumb_rest, params.thumb_smooth_time_s, params.thumb_max_speed) {
    curls_ = compose(0.0f, 0.0f);
}

void HandState::reset() {
    *this = HandState(params_);
}

ThumbState HandState::thumb_state() const {
    return selected_ ? from_index(*selected_) : ThumbState::Rest;
}

Curls HandState::update(std::uint64_t now_ns, const Input& input) {
    const Input inp = sanitize(input);
    std::optional<double> gap_s;
    if (last_ns_) {
        gap_s = now_ns > *last_ns_ ? static_cast<double>(now_ns - *last_ns_) * 1e-9 : 0.0;
    }
    if (!gap_s || *gap_s > static_cast<double>(params_.reset_gap_s)) {
        restart(inp);
    } else {
        step(inp, rs::fminf(static_cast<float>(*gap_s), k_max_dt_s));
    }
    if (!(last_ns_ && now_ns <= *last_ns_)) {
        last_ns_ = now_ns;
    }
    return curls_;
}

void HandState::restart(const Input& inp) {
    trigger_f_.reset();
    grip_f_.reset();
    const float t = trigger_f_.update(inp.trigger, 0.0f);
    const float g = grip_f_.update(inp.grip, 0.0f);
    contacts_ = {};
    stick_deflected_ = false;
    clock_ = 0.0;
    update_contacts(inp, 0.0f, true);
    thumb_sd_.snap(thumb_target());
    curls_ = compose(t, g);
}

void HandState::step(const Input& inp, float dt) {
    clock_ += static_cast<double>(dt);
    const float t = trigger_f_.update(inp.trigger, dt);
    const float g = grip_f_.update(inp.grip, dt);
    update_contacts(inp, dt, false);
    thumb_sd_.update(thumb_target(), dt);
    curls_ = compose(t, g);
}

void HandState::update_contacts(const Input& inp, float dt, bool immediate) {
    const float mag = std::hypot(inp.stick_x, inp.stick_y);
    stick_deflected_ = stick_deflected_ ? mag > params_.stick_off : mag > params_.stick_on;
    const std::uint32_t f = inp.flags;
    const auto ev = [f](std::uint32_t touch, std::uint32_t click, bool deflected) {
        return Evidence{(f & touch) != 0, (f & click) != 0 || deflected, (f & click) != 0};
    };
    const Evidence evidence[4] = {
        ev(flags::TOUCH_BTN1, flags::CLICK_BTN1, false),
        ev(flags::TOUCH_BTN2, flags::CLICK_BTN2, false),
        ev(flags::TOUCH_STICK, flags::CLICK_STICK, stick_deflected_),
        ev(0, flags::CLICK_MENU, false),
    };
    for (std::size_t i = 0; i < 4; i++) {
        contact_update(contacts_[i], evidence[i], dt, clock_, params_.engage_s, params_.release_hold_s, immediate);
    }
    // The most recently touched place wins (max_by: the last of equal maxima; equal times: lower index wins).
    std::optional<std::size_t> best;
    for (std::size_t i = 0; i < 4; i++) {
        if (!contacts_[i].active) {
            continue;
        }
        if (!best) {
            best = i;
            continue;
        }
        int c = total_cmp(contacts_[*best].since, contacts_[i].since);
        if (c == 0) {
            c = i < *best ? -1 : (i > *best ? 1 : 0); // then(b.cmp(&a)) with a = best, b = i
        }
        if (c != 1) {
            best = i;
        }
    }
    selected_ = best;
}

float HandState::thumb_target() const {
    const Params& p = params_;
    if (!selected_) {
        return p.thumb_rest;
    }
    const std::size_t i = *selected_;
    float base = p.thumb_rest;
    switch (i) {
    case BTN1: base = p.thumb_btn1; break;
    case BTN2: base = p.thumb_btn2; break;
    case STICK: base = p.thumb_stick; break;
    case MENU: base = p.thumb_menu; break;
    default: break;
    }
    const float extra = contacts_[i].clicked ? p.thumb_click_extra : 0.0f;
    return rs::clampf(base + extra, 0.0f, 1.0f);
}

Curls HandState::compose(float trigger, float grip) const {
    const Params& p = params_;
    const float t = shape(trigger, p.deadzone);
    const float g = shape(grip, p.deadzone);
    const float index = rs::clampf(lerp(p.index_rest, p.index_full, t), 0.0f, 1.0f);
    const float lower = rs::clampf(lerp(p.grip_rest, p.grip_full, g), 0.0f, 1.0f);
    return Curls{lower, lower, lower, index, rs::clampf(thumb_sd_.value(), 0.0f, 1.0f)};
}

HandJoints to_joints(const Curls& c) {
    const float idx = clamp01(c.index);
    FingerJoints index{SPLAY[0], {0.0f, idx * 1.1f * 1.3f, idx * 1.1f, idx * 1.1f}};
    const auto other = [](float curl, float splay) {
        const float v = clamp01(curl) * 1.1f * 1.3f;
        return FingerJoints{splay, {0.0f, v * 1.3f, v, v * 0.4f}};
    };
    const float th = clamp01(c.thumb);
    return HandJoints{other(c.little, SPLAY[3]), other(c.ring, SPLAY[2]), other(c.middle, SPLAY[1]), index,
                      FingerJoints{THUMB_SPLAY, {th, th * 1.57f, th * 1.4f, 0.0f}}};
}

} // namespace pibridge::ctrl
