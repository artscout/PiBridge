// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax/src/session.rs (MIT OR Apache-2.0).
//
// The Session: one thread that wakes the headset, sends the 213 ms keepalive, keeps IF1-IF3 open and drained (the
// firmware detaches when nobody reads them), reconnects automatically, matches command replies and delivers IMU /
// controller / status / log events through a Handler (or a bounded EventQueue).
//
// Every report the session writes goes through the HID allowlist gate of the connected model (devices/<id>.toml):
// a report that is not listed is refused (Errc::Refused) and nothing reaches the device. Without a loadable
// description nothing at all is sent.
//
// No async runtime: ppoll(2) on at most five fds.
#pragma once

#include "pibridge/hmd/allowlist.h"
#include "pibridge/hmd/discovery.h"
#include "pibridge/hmd/haptics.h"
#include "pibridge/hmd/proto.h"
#include "pibridge/hmd/sys.h"

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <variant>
#include <vector>

namespace pibridge::hmd {

/// Connection state reported to Handler::on_state.
enum class State {
    /// No session.
    Disconnected,
    /// Woken up, keepalive running.
    Connected,
};

/// Callbacks, all run on the session thread: keep them short and never call Session::request from one.
class Handler {
public:
    virtual ~Handler() = default;
    virtual void on_state(State) {}
    /// One IMU sample (two per report).
    virtual void on_imu(const ImuSample&) {}
    virtual void on_controller(const ControllerReport&) {}
    /// Unsolicited controller heartbeat (0x37).
    virtual void on_status(const ControllerStatus&) {}
    /// One firmware log line (IF3).
    virtual void on_log(std::string_view) {}
    /// Every report read from IF0.
    virtual void on_reply(const Report&) {}
    /// A report from IF1 or IF2 that is neither IMU nor controller data (`len` bytes of it are real).
    virtual void on_other(std::size_t, const Report&, std::size_t) {}
    /// A link ended: how long it lived and how many keepalives it sent (called after on_state(Disconnected)).
    virtual void on_link_closed(std::chrono::nanoseconds, std::uint64_t) {}
    /// The automatic logic (Config::auto_attach) scheduled an attach burst for `slot` (initial or retry).
    virtual void on_attach(std::uint8_t, bool) {}
};

/// Ignores everything (the session still drains and answers requests).
class NoopHandler : public Handler {};

// ------------------------------------------------------------------------------------------------ events (channel)

struct LogLine {
    std::string text;
    bool operator==(const LogLine&) const = default;
};
struct AttachEvent {
    std::uint8_t slot = 0;
    /// True = retry after a 0x37 heartbeat.
    bool retry = false;
    bool operator==(const AttachEvent&) const = default;
};
/// One event of the channel interface; a `Report` is an IF0 report (only when requested).
using Event = std::variant<State, ImuSample, ControllerReport, ControllerStatus, LogLine, Report, AttachEvent>;

/// Bounded, thread-safe event queue: events are dropped (and counted) when it is full.
class EventQueue {
public:
    explicit EventQueue(std::size_t capacity) : cap_(capacity) {}
    /// False when the queue was full (the event is dropped).
    bool push(Event e);
    std::optional<Event> try_pop();
    /// Wait up to `timeout` for one event.
    std::optional<Event> pop(std::chrono::milliseconds timeout);
    /// Everything queued now.
    std::vector<Event> drain();
    std::uint64_t dropped() const;

private:
    mutable std::mutex mu_;
    std::condition_variable cv_;
    std::deque<Event> q_;
    std::size_t cap_;
    std::uint64_t dropped_ = 0;
};

/// A Handler that feeds an EventQueue (`replies`: also every IF0 report).
std::unique_ptr<Handler> queue_handler(std::shared_ptr<EventQueue> q, bool replies);

// ------------------------------------------------------------------------------------------------ config

/// Session options. The defaults equal the behaviour of the C libpibridge-hmd.
struct Config {
    /// Send the wake `58 06 1a 05 01 00 00` after connecting.
    bool wake = true;
    /// Send keepalives (disable for passive read-only sessions).
    bool keepalive = true;
    /// Keepalive period.
    std::chrono::milliseconds period{KEEPALIVE_MS};
    /// Source of the rate byte. nullopt: the value of Session::set_video_rate (libpibridge-hmd behaviour). A path: re-read
    /// the file on every keepalive; missing or invalid = 0 (`$XDG_RUNTIME_DIR/pimax-rate`). The value `off`: one
    /// keepalive with rate 0, then none until the file changes, which is what Pimax Play does when it is closed
    /// (Crystal Super Windows capture of 2026-10-17, take shutdown: the headset goes to its green standby).
    std::optional<std::filesystem::path> rate_file;
    /// Attach both controllers at connect and re-attach when a 0x37 heartbeat shows one awake but silent.
    bool auto_attach = false;
    /// The allowlist every write must pass. Null: load the connected model's description from `devices_dirs`
    /// (empty = pibridge::hmd::devices_dirs()) at every connect.
    std::shared_ptr<const Allowlist> allowlist;
    std::vector<std::filesystem::path> devices_dirs;
    /// Used instead of `devices_dirs` when set (and `allowlist` is null): the allowlist of the connected model, at
    /// every connect (monado-abi: the installed descriptions, else the copies compiled into the archive).
    std::function<std::expected<std::shared_ptr<const Allowlist>, SpecError>(const Model&)> allowlist_for;
    /// Log lines (plain words); null = silent.
    std::function<void(std::string_view)> log;

    /// Read-only: no wake, no keepalive. The headset must be kept alive by someone else.
    static Config passive();
};

/// Counters, readable at any time.
struct Stats {
    std::uint64_t imu_reports = 0;
    std::uint64_t controller_reports = 0;
    std::uint64_t status_messages = 0;
    std::uint64_t log_lines = 0;
    std::uint64_t keepalives = 0;
    std::uint64_t connects = 0;
    /// Controller attach bursts scheduled by the automatic logic (retries).
    std::uint64_t auto_attaches = 0;
    /// Events dropped because a queue was full.
    std::uint64_t dropped_events = 0;
    /// Writes the allowlist refused.
    std::uint64_t refused = 0;
};

// ------------------------------------------------------------------------------------------------ backends

/// An open link to the device.
struct Link {
    /// Read side of each interface (IF0 included: replies).
    std::array<Fd, NUM_IF> reads;
    /// Write side of IF0.
    Fd ctl;
    /// The model on the other end (selects the allowlist).
    const Model* model = &DREAM_AIR;
};

/// Source of device links; the real one is HidrawBackend, tests use pibridge::hmd::fake.
class Backend {
public:
    virtual ~Backend() = default;
    /// Try to open the device. nullopt = not present (the session polls every 50 ms).
    virtual Result<std::optional<Link>> connect() = 0;
};

/// hidraw via sysfs (needs IF0 and IF2).
class HidrawBackend : public Backend {
public:
    HidrawBackend();
    explicit HidrawBackend(discovery::Roots roots) : roots_(std::move(roots)) {}
    Result<std::optional<Link>> connect() override;

private:
    discovery::Roots roots_;
};

/// A headset restart ignores the first wake: without a `4c .. 1a` reply this long after the wake, send it once more.
inline constexpr std::chrono::milliseconds WAKE_RESEND_AFTER{500};

// ------------------------------------------------------------------------------------------------ session

class Session {
public:
    using Matcher = std::function<bool(const Report&)>;

    /// Start a session on the real hidraw devices.
    static Result<std::unique_ptr<Session>> open(Config cfg, std::unique_ptr<Handler> handler);
    /// Start a session with a custom backend.
    static Result<std::unique_ptr<Session>> open_with_backend(Config cfg, std::unique_ptr<Backend> backend,
                                                              std::unique_ptr<Handler> handler);
    /// Events through a bounded queue (dropped when full).
    static Result<std::pair<std::unique_ptr<Session>, std::shared_ptr<EventQueue>>>
    open_channel_with_backend(Config cfg, std::unique_ptr<Backend> backend, std::size_t capacity, bool replies);

    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
    /// Stops the thread and waits for it.
    ~Session();

    /// True while a device session is up.
    bool is_connected() const;
    /// Block until connected (Errc::Timeout).
    Result<void> wait_connected(std::chrono::milliseconds timeout) const;
    /// Video rate carried by the keepalive (without a rate file); 0 = none.
    void set_video_rate(std::uint8_t fps);
    Stats stats() const;
    /// Model of the current (or last) link; nullptr before the first connect.
    const Model* model() const;
    /// Why the last connect attempt failed, in plain words ("" once connected).
    std::string last_error() const;

    /// Fire and forget `58 LL cmd opt data`.
    Result<void> send(std::uint8_t cmd, std::uint8_t opt, std::span<const std::uint8_t> data);
    /// Send a prebuilt report.
    Result<void> send_report(const Report& r);
    /// Send and wait for the `4c LL cmd opt ..` reply; returns its data without the check byte.
    Result<std::vector<std::uint8_t>> request(std::uint8_t cmd, std::uint8_t opt, std::span<const std::uint8_t> data,
                                              std::chrono::milliseconds timeout);
    /// Send `report` and wait for the first IF0 `4c` report accepted by `matcher`.
    Result<Report> request_with(const Report& report, std::chrono::milliseconds timeout, Matcher matcher);
    /// Serial number (cmd 0x16) of the headset (opt 0) or a controller (opt = slot).
    Result<std::string> read_serial(std::uint8_t opt, std::chrono::milliseconds timeout);
    /// Version bytes (cmd 0x03).
    Result<std::vector<std::uint8_t>> read_version(std::uint8_t opt, std::chrono::milliseconds timeout);
    /// The 1344-byte calibration blob over IF0 (84 reads, up to 3 tries each). calib::inflate_blob gives the XML.
    Result<std::vector<std::uint8_t>> read_calibration_blob(std::chrono::milliseconds per_try);
    /// Per-slot attach burst (0x16, 0x50, 0x0a, 0x03), 10 ms apart, blocking about 30 ms.
    Result<void> controller_attach(std::uint8_t slot);
    /// Switch the panel on: `5b(02 02)`, `29(01)`, `1a(02 FPS)`, `1a(05 00)`, and set the keepalive rate.
    Result<void> video_prepare(std::uint8_t fps);
    /// Vibrate a controller (`slot` 2 left / 3 right, pulse = amplitude percent 1..100, duration ms): queued for the
    /// session thread, which writes `58 09 07 SS AMP DUR` (through the gate, like every write) at most once per 8 ms
    /// per hand; a newer request replaces a waiting one. nullopt drops a waiting pulse (a stop sends nothing, as on
    /// Windows). Never blocks on the device. False for an unknown slot.
    bool haptic(std::uint8_t slot, std::optional<haptics::Pulse> pulse);
    /// Vibration commands handed to the gate so far, and requests replaced before they went out.
    std::pair<std::uint64_t, std::uint64_t> haptic_counts() const;

    /// Stop the thread and wait for it (also done by the destructor).
    void close();

    struct Shared;

private:
    Session() = default;
    std::shared_ptr<Shared> sh_;
    std::jthread thread_;
};

} // namespace pibridge::hmd
