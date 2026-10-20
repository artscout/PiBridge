// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-capi/src/lib.rs (MIT OR Apache-2.0).
// The libpimax C ABI (include/pimax.h) over pibridge::hmd::Session: same symbol names, struct layouts, callback
// signatures and return conventions (0 / length / -errno) as the Rust drop-in, plus the extension
// pimax_open_with_fds (tests, fake devices) and, PiBridge only, pimax_open_with_fds_pid (the same with the model the
// fake headset claims to be).
//
// The one deliberate difference: every report goes through the HID allowlist gate of the connected model
// (devices/<id>.toml, docs/libpibridge-hmd.md). The descriptions are taken from the PiBridge devices folders
// (PIBRIDGE_DEVICES_DIR, the installed copy) and otherwise from the copies compiled into this archive; a report the
// description does not list is not written and the call returns -EPERM. Refusals and session problems are logged to
// stderr as "libpimax: ..." (each distinct line once in a row).
#include "pimax.h"

#include "pibridge/cam/rustfmt.h"
#include "pibridge/hmd/descriptor.h"
#include "pibridge/hmd/haptics.h"
#include "pibridge/hmd/proto.h"
#include "pibridge/hmd/session.h"

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>

#include <fcntl.h>
#include <unistd.h>

#define PIBRIDGE_PIMAX_ABI extern "C" __attribute__((visibility("default")))

namespace hmd = pibridge::hmd;

#include "pimax_internal.h"

struct pimax {
    std::unique_ptr<hmd::Session> session;
    /// Set when pimax_close starts: the session thread then stops calling into the application, so a callback that
    /// needs a lock the closing thread holds (or state it is tearing down) cannot stall or race the join.
    std::shared_ptr<std::atomic<bool>> closing;
};

namespace {

pimax_imu_sample to_imu(const hmd::ImuSample& s) {
    pimax_imu_sample o{};
    o.ts_us = s.ts_us;
    for (int i = 0; i < 3; i++) {
        o.raw_accel[i] = s.raw_accel[i];
        o.raw_gyro[i] = s.raw_gyro[i];
        o.accel_g[i] = s.accel_g[i];
        o.gyro_dps[i] = s.gyro_dps[i];
    }
    o.ipd_mm = s.ipd_mm;
    o.sample_in_report = s.sample_in_report;
    return o;
}

pimax_controller_report to_ctl(const hmd::ControllerReport& c) {
    pimax_controller_report o{};
    o.slot = c.slot;
    o.ts_us = c.ts_us;
    o.stick_x = c.stick_x;
    o.stick_y = c.stick_y;
    o.trigger = c.trigger;
    o.grip = c.grip;
    o.battery = c.battery;
    o.buttons = c.buttons;
    o.touch = c.touch;
    for (int i = 0; i < 3; i++) {
        o.raw_accel[i] = c.raw_accel[i];
        o.raw_gyro[i] = c.raw_gyro[i];
    }
    std::memcpy(o.raw, c.raw.data(), PIMAX_REPORT_SIZE);
    return o;
}

class CHandler final : public hmd::Handler {
public:
    CHandler(const pimax_callbacks* cb, std::shared_ptr<std::atomic<bool>> closing) : closing_(std::move(closing)) {
        if (cb != nullptr) {
            cb_ = *cb;
        }
    }
    void on_state(hmd::State s) override {
        if (!closing() && cb_.on_state) {
            cb_.on_state(cb_.user, s == hmd::State::Connected ? PIMAX_CONNECTED : PIMAX_DISCONNECTED);
        }
    }
    void on_imu(const hmd::ImuSample& s) override {
        if (!closing() && cb_.on_imu) {
            const auto c = to_imu(s);
            cb_.on_imu(cb_.user, &c);
        }
    }
    void on_controller(const hmd::ControllerReport& r) override {
        if (!closing() && cb_.on_controller) {
            const auto c = to_ctl(r);
            cb_.on_controller(cb_.user, &c);
        }
    }
    void on_status(const hmd::ControllerStatus& s) override {
        if (!closing() && cb_.on_controller_status) {
            pimax_controller_status c{};
            c.slot = s.slot;
            c.battery = s.battery;
            c.fw[0] = s.fw[0];
            c.fw[1] = s.fw[1];
            std::memcpy(c.raw, s.raw.data(), PIMAX_REPORT_SIZE);
            cb_.on_controller_status(cb_.user, &c);
        }
    }
    void on_log(std::string_view line) override {
        if (!closing() && cb_.on_log) {
            // the Rust session hands over the line as UTF-8 (invalid bytes replaced), the ABI cuts it at 511 bytes
            const std::string s = pibridge::cam::rfmt::utf8_lossy(line);
            char buf[513] = {};
            const std::size_t n = std::min<std::size_t>(s.size(), 511);
            std::memcpy(buf, s.data(), n);
            cb_.on_log(cb_.user, buf);
        }
    }
    void on_reply(const hmd::Report& r) override {
        if (!closing() && cb_.on_reply) {
            cb_.on_reply(cb_.user, r.data());
        }
    }

private:
    bool closing() const { return closing_->load(std::memory_order_acquire); }
    pimax_callbacks cb_{};
    std::shared_ptr<std::atomic<bool>> closing_;
};

void set_nonblock(int fd) {
    const int fl = ::fcntl(fd, F_GETFL);
    if (fl >= 0) {
        ::fcntl(fd, F_SETFL, fl | O_NONBLOCK);
    }
}

// stderr, each distinct line once in a row (a refused stream would otherwise repeat the same line at 125 Hz).
void log_line(std::string_view l) {
    static std::mutex mu;
    static std::string last;
    std::lock_guard g(mu);
    if (l == last) {
        return;
    }
    last.assign(l);
    std::fprintf(stderr, "libpimax: %.*s\n", static_cast<int>(l.size()), l.data());
}

} // namespace

// The allowlist of the connected model: the PiBridge devices folders first, then the copy compiled into the archive.
std::expected<std::shared_ptr<const hmd::Allowlist>, hmd::SpecError> pibridge::abi::device_allowlist(const hmd::Model& m) {
    auto a = hmd::load_allowlist(m, hmd::devices_dirs());
    if (a) {
        return a;
    }
    const auto text = pibridge::abi::embedded_device_toml(m.id);
    if (text.empty()) {
        return a;
    }
    auto d = hmd::Descriptor::parse(text);
    if (!d) {
        return std::unexpected(hmd::SpecError{"built-in " + std::string(m.id) + ".toml: " + d.error().msg});
    }
    return std::make_shared<const hmd::Allowlist>(d->allowlist);
}

namespace {

// Config::default() of the Rust session (wake, keepalive, rate from the handle, no auto attach) + the gate.
hmd::Config config() {
    hmd::Config c;
    c.allowlist_for = pibridge::abi::device_allowlist;
    c.log = log_line;
    return c;
}

pimax_t* boxed(hmd::Result<std::unique_ptr<hmd::Session>> s, std::shared_ptr<std::atomic<bool>> closing) {
    if (!s) {
        errno = s.error().os_errno != 0 ? s.error().os_errno : ENOMEM;
        return nullptr;
    }
    return new pimax{std::move(*s), std::move(closing)};
}

class FdBackend final : public hmd::Backend {
public:
    explicit FdBackend(hmd::Link l) : link_(std::move(l)) {}
    hmd::Result<std::optional<hmd::Link>> connect() override {
        if (!link_) {
            return std::optional<hmd::Link>{};
        }
        auto l = std::move(link_);
        link_.reset();
        return l;
    }

private:
    std::optional<hmd::Link> link_;
};

pimax_t* open_fds(const pimax_callbacks* cb, const int* read_fds, int ctl_write_fd, const hmd::Model& model) {
    hmd::Link link;
    for (std::size_t i = 0; i < PIMAX_NUM_IF; i++) {
        const int fd = read_fds[i];
        if (fd >= 0) {
            set_nonblock(fd);
            link.reads[i] = hmd::Fd(fd);
        }
    }
    set_nonblock(ctl_write_fd);
    link.ctl = hmd::Fd(ctl_write_fd);
    link.model = &model;
    auto closing = std::make_shared<std::atomic<bool>>(false);
    return boxed(hmd::Session::open_with_backend(config(), std::make_unique<FdBackend>(std::move(link)),
                                                 std::make_unique<CHandler>(cb, closing)),
                 closing);
}

std::span<const std::uint8_t> data_slice(const void* data, std::size_t len) {
    if (data == nullptr || len == 0) {
        return {};
    }
    return {static_cast<const std::uint8_t*>(data), len};
}

int timeout_ms_of(int t) { return t < 0 ? 0 : t; }

// No exception leaves the ABI (Rust: panic = abort is not used, errors are -errno); a NULL handle is refused
// instead of crashing (the Rust library dereferences it).
template <class F>
int guarded(pimax_t* h, F&& f) noexcept {
    if (h == nullptr || !h->session) {
        return -EINVAL;
    }
    try {
        return f(*h->session);
    } catch (...) {
        return -EIO;
    }
}

} // namespace

/// Fill paths[n] with "/dev/hidrawN" for USB interface n. Returns a bitmask of the interfaces found.
PIBRIDGE_PIMAX_ABI int pimax_find_interfaces(char paths[PIMAX_NUM_IF][PIMAX_PATH_MAX]) {
    try {
        const auto found = hmd::discovery::find_interfaces(hmd::discovery::Roots::from_env());
        int mask = 0;
        for (std::size_t i = 0; i < found.size() && i < PIMAX_NUM_IF; i++) {
            if (!found[i]) {
                continue;
            }
            mask |= 1 << i;
            if (paths == nullptr) {
                continue;
            }
            const std::string& p = found[i]->native();
            const std::size_t n = std::min<std::size_t>(p.size(), PIMAX_PATH_MAX - 1);
            std::memcpy(paths[i], p.data(), n);
            paths[i][n] = '\0';
        }
        return mask;
    } catch (...) {
        return 0;
    }
}

/// Start the session thread. cb may be NULL. Returns NULL (errno set) on failure.
PIBRIDGE_PIMAX_ABI pimax_t* pimax_open(const pimax_callbacks* cb) {
    try {
        auto closing = std::make_shared<std::atomic<bool>>(false);
        return boxed(hmd::Session::open(config(), std::make_unique<CHandler>(cb, closing)), closing);
    } catch (...) {
        errno = ENOMEM;
        return nullptr;
    }
}

/// Not part of pimax.h: run a session on already-open descriptors (tests, fake devices). read_fds[n] is the
/// descriptor to read for interface n (negative = absent), ctl_write_fd receives commands. The library takes ownership
/// of all descriptors and makes them non-blocking. The link is a Dream Air (its allowlist).
PIBRIDGE_PIMAX_ABI pimax_t* pimax_open_with_fds(const pimax_callbacks* cb, const int* read_fds, int ctl_write_fd) {
    try {
        return open_fds(cb, read_fds, ctl_write_fd, hmd::DREAM_AIR);
    } catch (...) {
        errno = ENOMEM;
        return nullptr;
    }
}

/// PiBridge only: pimax_open_with_fds for the model with USB product id `pid` (0x0044 Dream Air, 0x0040 Crystal Super),
/// so a test can run the session against that model's allowlist. NULL with errno ENODEV for an unknown pid.
PIBRIDGE_PIMAX_ABI pimax_t* pimax_open_with_fds_pid(const pimax_callbacks* cb, const int* read_fds, int ctl_write_fd,
                                                 uint16_t pid) {
    try {
        const hmd::Model* m = hmd::model_by_pid(pid);
        if (m == nullptr) {
            errno = ENODEV;
            return nullptr;
        }
        return open_fds(cb, read_fds, ctl_write_fd, *m);
    } catch (...) {
        errno = ENOMEM;
        return nullptr;
    }
}

PIBRIDGE_PIMAX_ABI void pimax_close(pimax_t* h) {
    if (h == nullptr) {
        return;
    }
    h->closing->store(true, std::memory_order_release);
    try {
        h->session.reset(); // wakes the session thread (eventfd in the ppoll set) and joins it
    } catch (...) {
    }
    delete h;
}

PIBRIDGE_PIMAX_ABI int pimax_is_connected(pimax_t* h) {
    return h != nullptr && h->session && h->session->is_connected() ? 1 : 0;
}

/// 0 on success, -ETIMEDOUT otherwise.
PIBRIDGE_PIMAX_ABI int pimax_wait_connected(pimax_t* h, int timeout_ms) {
    return guarded(h, [&](hmd::Session& s) {
        return s.wait_connected(std::chrono::milliseconds(timeout_ms_of(timeout_ms))) ? 0 : -ETIMEDOUT;
    });
}

PIBRIDGE_PIMAX_ABI void pimax_set_video_rate(pimax_t* h, uint8_t fps) {
    (void)guarded(h, [&](hmd::Session& s) {
        s.set_video_rate(fps);
        return 0;
    });
}

PIBRIDGE_PIMAX_ABI int pimax_send(pimax_t* h, uint8_t cmd, uint8_t opt, const void* data, size_t len) {
    return guarded(h, [&](hmd::Session& s) {
        const auto r = s.send(cmd, opt, data_slice(data, len));
        return r ? 0 : r.error().to_errno();
    });
}

PIBRIDGE_PIMAX_ABI int pimax_request(pimax_t* h, uint8_t cmd, uint8_t opt, const void* data, size_t len,
                                  uint8_t* reply, size_t reply_cap, int timeout_ms) {
    return guarded(h, [&](hmd::Session& s) {
        const auto d = s.request(cmd, opt, data_slice(data, len),
                                 std::chrono::milliseconds(timeout_ms_of(timeout_ms)));
        if (!d) {
            return d.error().to_errno();
        }
        if (reply != nullptr && reply_cap > 0) {
            std::memcpy(reply, d->data(), std::min(d->size(), reply_cap));
        }
        return static_cast<int>(d->size());
    });
}

PIBRIDGE_PIMAX_ABI int pimax_read_serial(pimax_t* h, uint8_t opt, char* out, size_t cap, int timeout_ms) {
    std::uint8_t r[PIMAX_REPORT_SIZE] = {};
    const std::uint8_t arg[1] = {0x1e};
    const int n = pimax_request(h, hmd::cmd::SERIAL, opt, arg, 1, r, sizeof r, timeout_ms);
    if (n < 0) {
        return n;
    }
    const auto nn = static_cast<std::size_t>(n);
    std::size_t sl = nn > 0 ? r[0] : 0;
    const std::size_t room = nn > 0 ? nn - 1 : 0; // saturating_sub(1)
    if (sl > room) {
        sl = room;
    }
    if (sl >= cap) {
        sl = cap > 0 ? cap - 1 : 0;
    }
    if (cap > 0 && out != nullptr) {
        std::memcpy(out, r + 1, sl);
        out[sl] = '\0';
    }
    return static_cast<int>(sl);
}

PIBRIDGE_PIMAX_ABI int pimax_read_version(pimax_t* h, uint8_t opt, uint8_t* out, size_t cap, int timeout_ms) {
    return pimax_request(h, hmd::cmd::VERSION, opt, nullptr, 0, out, cap, timeout_ms);
}

PIBRIDGE_PIMAX_ABI int pimax_controller_attach(pimax_t* h, uint8_t slot) {
    return guarded(h, [&](hmd::Session& s) {
        const auto r = s.controller_attach(slot);
        return r ? 0 : r.error().to_errno();
    });
}

PIBRIDGE_PIMAX_ABI int pimax_video_prepare(pimax_t* h, uint8_t fps) {
    return guarded(h, [&](hmd::Session& s) {
        const auto r = s.video_prepare(fps);
        return r ? 0 : r.error().to_errno();
    });
}

/// Controller vibration from an OpenXR haptic value (amplitude 0..1, duration ns, -1 = minimum; frequency is not
/// used by the hardware). Queued for the session thread: `58 09 07 SS AMP DUR` at most once per 8 ms per hand,
/// newest wins. 0 when queued, 1 for a stop (amplitude 0: nothing is sent, a waiting pulse is dropped), -EINVAL for a
/// slot other than 2/3.
PIBRIDGE_PIMAX_ABI int pimax_haptic(pimax_t* h, uint8_t slot, float amplitude, int64_t duration_ns) {
    return guarded(h, [&](hmd::Session& s) {
        const auto pulse = hmd::haptic_from_openxr(amplitude, duration_ns);
        if (!s.haptic(slot, pulse)) {
            return -EINVAL;
        }
        return pulse ? 0 : 1;
    });
}

/// Take a test pulse request ($XDG_RUNTIME_DIR/pimax-haptic-request) and queue it. The slot (2/3) when one was sent,
/// 0 when there was none, -EINVAL for a malformed request (removed).
PIBRIDGE_PIMAX_ABI int pimax_haptic_poll_request(pimax_t* h) {
    return guarded(h, [&](hmd::Session& s) {
        const auto r = hmd::haptics::take_request(hmd::haptics::request_path());
        if (!r) {
            return -EINVAL;
        }
        if (!*r) {
            return 0;
        }
        const auto [slot, amp, ms] = **r;
        s.haptic(slot, hmd::haptics::Pulse{amp, ms});
        return static_cast<int>(slot);
    });
}

/// Vibration commands handed to the gate and requests replaced before they went out (either pointer may be NULL).
PIBRIDGE_PIMAX_ABI void pimax_haptic_counts(pimax_t* h, uint64_t* sent, uint64_t* replaced) {
    (void)guarded(h, [&](hmd::Session& s) {
        const auto [a, b] = s.haptic_counts();
        if (sent != nullptr) {
            *sent = a;
        }
        if (replaced != nullptr) {
            *replaced = b;
        }
        return 0;
    });
}

/// 0 if the buffer is not an IMU report. out receives two samples.
PIBRIDGE_PIMAX_ABI int pimax_decode_imu(const uint8_t r[PIMAX_REPORT_SIZE], pimax_imu_sample out[2]) {
    if (r == nullptr || out == nullptr) {
        return 0;
    }
    hmd::Report rep;
    std::memcpy(rep.data(), r, PIMAX_REPORT_SIZE);
    const auto s = hmd::decode_imu(rep);
    if (!s) {
        return 0;
    }
    out[0] = to_imu((*s)[0]);
    out[1] = to_imu((*s)[1]);
    return 1;
}

/// 0 if the buffer is not a controller report.
PIBRIDGE_PIMAX_ABI int pimax_decode_controller(const uint8_t r[PIMAX_REPORT_SIZE], pimax_controller_report* out) {
    if (r == nullptr || out == nullptr) {
        return 0;
    }
    hmd::Report rep;
    std::memcpy(rep.data(), r, PIMAX_REPORT_SIZE);
    const auto c = hmd::decode_controller(rep);
    if (!c) {
        return 0;
    }
    *out = to_ctl(*c);
    return 1;
}
