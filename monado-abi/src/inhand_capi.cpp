// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-hands/src/inhand_ffi.rs (MIT OR Apache-2.0).
// The pimax_inhand_* C ABI (include/pimax_inhand.h) over libpibridge-ctrl's in-hand detector. Every function tolerates
// NULL; one object per controller, calls serialised by an internal mutex. Status file $XDG_RUNTIME_DIR/pimax-ctrl-inhand
// (pid, mode, left/right held|down|shown|off and why), run-time request $XDG_RUNTIME_DIR/pimax-ctrl-inhand-mode.
#include "pimax_inhand.h"

#include "pibridge/cam/io.h"
#include "pibridge/ctrl/inhand.h"
#include "pibridge/ctrl/rust.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <unistd.h>
#include <utility>

#define PIBRIDGE_INHAND_ABI extern "C" __attribute__((visibility("default")))

namespace ih = pibridge::ctrl::inhand;
namespace io = pibridge::cam;

struct PimaxInHand {
    std::mutex m;
    ih::InHand d;
    explicit PimaxInHand(ih::Config c) : d(c) {}
};

namespace {

int mode_to_c(ih::Mode m) {
    switch (m) {
    case ih::Mode::Auto: return PIMAX_INHAND_MODE_AUTO;
    case ih::Mode::Always: return PIMAX_INHAND_MODE_ALWAYS;
    case ih::Mode::Off: return PIMAX_INHAND_MODE_OFF;
    }
    return PIMAX_INHAND_MODE_AUTO;
}

ih::Mode mode_from_c(int m) {
    switch (m) {
    case PIMAX_INHAND_MODE_ALWAYS: return ih::Mode::Always;
    case PIMAX_INHAND_MODE_OFF: return ih::Mode::Off;
    default: return ih::Mode::Auto;
    }
}

std::string runtime_dir() {
    const auto v = io::env_os("XDG_RUNTIME_DIR");
    return v && !v->empty() ? *v : std::string("/tmp");
}

std::string status_path() {
    return io::path_join(runtime_dir(), "pimax-ctrl-inhand");
}

std::string request_path() {
    return io::path_join(runtime_dir(), "pimax-ctrl-inhand-mode");
}

std::pair<std::string, std::string> state_line(PimaxInHand* h, ih::Mode mode, uint64_t now_ns) {
    if (mode == ih::Mode::Off) {
        return {"off", "switched off"};
    }
    if (h == nullptr) {
        return {"down", "no controller"};
    }
    std::lock_guard lk(h->m);
    const bool held = h->d.held(now_ns);
    const std::string why = ih::why_name(h->d.last_change().first);
    if (mode == ih::Mode::Always) {
        return {held ? "held" : "shown", why};
    }
    return {held ? "held" : "down", why};
}

} // namespace

PIBRIDGE_INHAND_ABI int pimax_inhand_mode_from_env(int default_mode) {
    try {
        const auto v = io::env_utf8("PIMAX_CTRL_INHAND");
        if (v) {
            if (const auto m = ih::parse_mode(*v)) {
                return mode_to_c(*m);
            }
        }
    } catch (...) {
    }
    return default_mode;
}

PIBRIDGE_INHAND_ABI const char* pimax_inhand_mode_name(int mode) {
    return ih::mode_name(mode_from_c(mode));
}

PIBRIDGE_INHAND_ABI pimax_inhand* pimax_inhand_create(int /*hand*/) {
    try {
        return new PimaxInHand(ih::Config::from_env());
    } catch (...) {
        return nullptr;
    }
}

PIBRIDGE_INHAND_ABI void pimax_inhand_destroy(pimax_inhand* h) {
    delete h;
}

PIBRIDGE_INHAND_ABI void pimax_inhand_report(pimax_inhand* h, uint64_t host_ns, const uint8_t* raw, uint8_t trigger,
                                          uint8_t grip, int stick_x, int stick_y, uint32_t buttons, uint8_t touch) {
    if (h == nullptr || raw == nullptr) {
        return;
    }
    try {
        std::lock_guard lk(h->m);
        h->d.report(host_ns, raw, trigger, grip, stick_x, stick_y, buttons, touch);
    } catch (...) {
    }
}

PIBRIDGE_INHAND_ABI int pimax_inhand_held(pimax_inhand* h, uint64_t now_ns) {
    if (h == nullptr) {
        return 0;
    }
    std::lock_guard lk(h->m);
    return h->d.held(now_ns) ? 1 : 0;
}

PIBRIDGE_INHAND_ABI int pimax_inhand_publish(int mode, pimax_inhand* left, pimax_inhand* right, uint64_t now_ns) {
    try {
        static std::mutex last_m;
        static std::string last;
        const ih::Mode m = mode_from_c(mode);
        const auto [l, lw] = state_line(left, m, now_ns);
        const auto [r, rw] = state_line(right, m, now_ns);
        const std::string text = "pid=" + std::to_string(pibridge::ctrl::rs::process_id()) + "\nmode=" + ih::mode_name(m) +
                                 "\nleft=" + l + "\nleft_why=" + lw + "\nright=" + r + "\nright_why=" + rw + "\n";
        std::lock_guard lk(last_m);
        if (last == text) {
            return 0;
        }
        const std::string path = status_path();
        const std::string tmp = pibridge::ctrl::rs::with_extension(path, "tmp");
        if (io::write_file(tmp, text) && ::rename(tmp.c_str(), path.c_str()) == 0) {
            last = text;
            return 1;
        }
    } catch (...) {
    }
    return 0;
}

PIBRIDGE_INHAND_ABI int pimax_inhand_poll_mode(void) {
    try {
        const std::string p = request_path();
        const auto s = io::read_to_string(p);
        if (!s) {
            return -1;
        }
        ::unlink(p.c_str());
        if (const auto m = ih::parse_mode(*s)) {
            return mode_to_c(*m);
        }
    } catch (...) {
    }
    return -1;
}

PIBRIDGE_INHAND_ABI void pimax_inhand_clear_request(void) {
    try {
        ::unlink(request_path().c_str());
    } catch (...) {
    }
}

PIBRIDGE_INHAND_ABI size_t pimax_inhand_describe(pimax_inhand* h, uint64_t now_ns, char* buf, size_t len) {
    if (buf == nullptr || len == 0) {
        return 0;
    }
    try {
        const auto [s, w] = state_line(h, ih::Mode::Auto, now_ns);
        const std::string line = s + " (" + w + ")";
        const std::size_t n = std::min(line.size(), len - 1);
        std::memcpy(buf, line.data(), n);
        buf[n] = 0;
        return n;
    } catch (...) {
        return 0;
    }
}
