// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-hands/src/ffi.rs (MIT OR Apache-2.0).
// The pimax_hands_* C ABI (include/pimax_hands.h) over libpibridge-ctrl. Every function tolerates NULL; nothing throws
// out of the ABI (an allocation failure in create returns NULL, the Rust library would abort).
#include "pimax_hands.h"

#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/ctrl/demo.h"
#include "pibridge/ctrl/mapping.h"
#include "pibridge/ctrl/pose.h"
#include "pibridge/ctrl/report.h"
#include "pibridge/ctrl/rest.h"
#include "pibridge/ctrl/touch.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <optional>
#include <string>

#define PIBRIDGE_HANDS_ABI extern "C" __attribute__((visibility("default")))

namespace ctrl = pibridge::ctrl;
namespace rf = pibridge::cam::rfmt;

struct pimax_hands {
    ctrl::HandState state{ctrl::Params{}};
};

static_assert(sizeof(pimax_hands_input) == 20 && offsetof(pimax_hands_input, flags) == 16);
static_assert(sizeof(pimax_hands_finger) == 20);
static_assert(sizeof(pimax_hands_out) == 5 * 20 + 20 + 4);
static_assert(sizeof(pimax_hands_pose) == 28);

namespace {

pimax_hands_finger finger(const ctrl::FingerJoints& f) {
    pimax_hands_finger o;
    o.splay = f.splay;
    for (int k = 0; k < 4; k++) {
        o.joint_curls[k] = f.joint_curls[k];
    }
    return o;
}

pimax_hands_pose c_pose(const ctrl::Pose& p) {
    pimax_hands_pose o;
    for (int k = 0; k < 4; k++) {
        o.orientation[k] = static_cast<float>(p.q[k]);
    }
    for (int k = 0; k < 3; k++) {
        o.position[k] = static_cast<float>(p.p[k]);
    }
    return o;
}

/// Placement of the synthesized controllers, read once from PIMAX_CTRL_REST and PIMAX_CTRL_AIM.
struct ControllerConfig {
    ctrl::Rest rest = ctrl::Rest::desk();
    ctrl::AimMode aim = ctrl::AimMode::level();
    std::optional<std::string> error;
};

ControllerConfig config_from(const std::optional<std::string>& rest, const std::optional<std::string>& aim) {
    ControllerConfig c;
    std::string errs;
    const auto push = [&](const std::string& e) {
        if (!errs.empty()) {
            errs += "; ";
        }
        errs += e;
    };
    if (rest) {
        if (auto r = ctrl::Rest::parse(*rest)) {
            c.rest = *r;
        } else {
            push(r.error());
        }
    }
    if (aim) {
        if (auto a = ctrl::AimMode::parse(*aim)) {
            c.aim = *a;
        } else {
            push(a.error());
        }
    }
    if (!errs.empty()) {
        c.error = errs;
    }
    return c;
}

const ControllerConfig& controller_config() {
    static const ControllerConfig cfg =
        config_from(pibridge::cam::env_utf8("PIMAX_CTRL_REST"), pibridge::cam::env_utf8("PIMAX_CTRL_AIM"));
    return cfg;
}

std::string describe(const ControllerConfig& c) {
    const std::string aim = !c.aim.head ? std::string("level (yaw only)")
                                        : "head (rays meet " + rf::fixed(c.aim.distance, 2) + " m ahead of the view)";
    std::string s = "rest " + rf::fixed(c.rest.side, 2) + " m side, " + rf::fixed(c.rest.down, 2) + " m down, " +
                    rf::fixed(c.rest.fwd, 2) + " m forward, grip pitch " + rf::fixed(c.rest.grip_pitch_deg, 0) + " deg" +
                    (c.rest == ctrl::Rest::legacy() ? " (legacy)" : "") + "; aim " + aim;
    if (c.error) {
        s += "; INVALID SETTING IGNORED: " + *c.error;
    }
    return s;
}

} // namespace

PIBRIDGE_HANDS_ABI uint32_t pimax_hands_abi_version(void) {
    return PIMAX_HANDS_ABI_VERSION;
}

PIBRIDGE_HANDS_ABI pimax_hands* pimax_hands_create(void) {
    try {
        return new pimax_hands();
    } catch (...) {
        return nullptr;
    }
}

PIBRIDGE_HANDS_ABI void pimax_hands_destroy(pimax_hands* h) {
    delete h;
}

PIBRIDGE_HANDS_ABI void pimax_hands_reset(pimax_hands* h) {
    if (h != nullptr) {
        h->state.reset();
    }
}

PIBRIDGE_HANDS_ABI void pimax_hands_update(pimax_hands* h, uint64_t now_ns, const pimax_hands_input* input) {
    if (h == nullptr || input == nullptr) {
        return;
    }
    h->state.update(now_ns, ctrl::Input{input->trigger, input->grip, input->stick_x, input->stick_y, input->flags});
}

PIBRIDGE_HANDS_ABI void pimax_hands_update_report(pimax_hands* h, uint64_t now_ns, uint8_t trigger, uint8_t grip,
                                               int stick_x, int stick_y, uint32_t buttons, uint8_t touch) {
    if (h == nullptr) {
        return;
    }
    h->state.update(now_ns, ctrl::input_from_report(ctrl::Report{trigger, grip, stick_x, stick_y, buttons, touch}));
}

PIBRIDGE_HANDS_ABI void pimax_hands_get(const pimax_hands* h, pimax_hands_out* out) {
    if (h == nullptr || out == nullptr) {
        return;
    }
    const ctrl::Curls c = h->state.curls();
    const ctrl::HandJoints j = ctrl::to_joints(c);
    pimax_hands_out o;
    o.little = finger(j.little);
    o.ring = finger(j.ring);
    o.middle = finger(j.middle);
    o.index = finger(j.index);
    o.thumb = finger(j.thumb);
    const auto a = c.to_array();
    for (int k = 0; k < 5; k++) {
        o.curl[k] = a[k];
    }
    o.thumb_state = static_cast<uint32_t>(h->state.thumb_state());
    *out = o;
}

PIBRIDGE_HANDS_ABI void pimax_hands_grip_to_wrist(int is_right, pimax_hands_pose* out) {
    if (out == nullptr) {
        return;
    }
    *out = c_pose(ctrl::grip_to_wrist(is_right != 0 ? ctrl::Hand::Right : ctrl::Hand::Left));
}

PIBRIDGE_HANDS_ABI void pimax_hands_demo_input(uint64_t now_ns, int is_right, pimax_hands_input* out) {
    if (out == nullptr) {
        return;
    }
    const ctrl::Input i = ctrl::demo_input(now_ns, is_right != 0);
    *out = pimax_hands_input{i.trigger, i.grip, i.stick_x, i.stick_y, i.flags};
}

PIBRIDGE_HANDS_ABI void pimax_hands_controller_poses(int is_right, const float* head_orientation, pimax_hands_pose* aim,
                                                  pimax_hands_pose* grip) {
    try {
        std::array<double, 4> q = {0.0, 0.0, 0.0, 1.0};
        if (head_orientation != nullptr) {
            const std::array<double, 4> h = {head_orientation[0], head_orientation[1], head_orientation[2],
                                             head_orientation[3]};
            const double n = std::sqrt(h[0] * h[0] + h[1] * h[1] + h[2] * h[2] + h[3] * h[3]);
            if (std::isfinite(n) && n > 1e-6) {
                q = {h[0] / n, h[1] / n, h[2] / n, h[3] / n};
            }
        }
        const ControllerConfig& c = controller_config();
        const auto [a, g] =
            ctrl::controller_poses(is_right != 0 ? ctrl::Hand::Right : ctrl::Hand::Left, c.rest, c.aim, q);
        if (aim != nullptr) {
            *aim = c_pose(a);
        }
        if (grip != nullptr) {
            *grip = c_pose(g);
        }
    } catch (...) {
    }
}

PIBRIDGE_HANDS_ABI void pimax_hands_aim_to_grip(pimax_hands_pose* out) {
    try {
        if (out != nullptr) {
            *out = c_pose(controller_config().rest.aim_to_grip());
        }
    } catch (...) {
    }
}

PIBRIDGE_HANDS_ABI void pimax_hands_tracked_aim_to_grip(pimax_hands_pose* out) {
    if (out != nullptr) {
        *out = c_pose(ctrl::touch::aim_to_grip());
    }
}

PIBRIDGE_HANDS_ABI int pimax_hands_controller_describe(char* buf, size_t len) {
    try {
        const ControllerConfig& c = controller_config();
        if (buf != nullptr && len > 0) {
            const std::string d = describe(c);
            const std::size_t n = std::min(d.size(), len - 1);
            std::memcpy(buf, d.data(), n);
            buf[n] = 0;
        }
        return c.error ? 1 : 0;
    } catch (...) {
        return 0;
    }
}
