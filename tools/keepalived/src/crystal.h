// SPDX-License-Identifier: MIT
// The original Pimax Crystal (P3B, USB 34A4:0012) in pibridge-keepalived. Unlike the Crystal Super the PC does not wake
// it: it is switched on with its own button, boots (standalone XR2) and comes up on USB. This waits for it, then does
// what Pimax Play does on connect (captured 2026-10-22): screen on, keep the runtime, PC video at 120 Hz; switches the
// tracking to the one pibridge.conf asks for (crystal_tracking: the Lighthouse faceplate or its own inside-out), polls
// the connection every 10 s, and starts SteamVR (crystal_autostart_steamvr). Only the captured frames of
// pibridge/hmd/p3b.h are ever written.
#pragma once

#include "pibridge/hmd/p3b.h"

#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::keepalived {

namespace p3b = pibridge::hmd::p3b;

/// The open headset: writes frames, hands back the RPC replies read in the background.
class P3bLink {
public:
    virtual ~P3bLink() = default;
    virtual bool write(const p3b::Report& r) = 0;
    /// Replies read since the last call (oldest first).
    virtual std::vector<p3b::Rpc> take_replies() = 0;
    /// False once the device is gone (read or write failed).
    virtual bool alive() const = 0;
    /// The newest head report (battery, IPD, tracking mode), if one came yet.
    virtual std::optional<p3b::Head> last_head() const = 0;
};

/// The hidraw link: a reader thread drains the 1024-byte reports (the tracking stream too, ~1270/s) and keeps the
/// RPC replies; writes go out as report number 0 + 1024 bytes (the device has no report ids).
std::unique_ptr<P3bLink> open_hidraw(const std::filesystem::path& node, std::string* error);

/// Start `argv` detached (double fork, its own session, stdio to /dev/null, never waited for); false when it could not
/// be started (the exec failing in the grandchild is not seen).
bool start_detached(const std::vector<std::string>& argv);
/// `argv` run through `systemd-run --user --scope` when there is one: out of the calling service's cgroup.
std::vector<std::string> in_own_scope(std::vector<std::string> argv);

/// Sends `sig` to SteamVR's own processes (their executable under .../SteamVR/bin/), not to the games.
void signal_steamvr_processes(int sig, const std::filesystem::path& proc = "/proc");

/// True when a Valve Lighthouse receiver (LHR, USB 28de:2300: the headset's Lighthouse module) is on USB.
bool lighthouse_module_present(const std::filesystem::path& usb_devices = "/sys/bus/usb/devices");

class Crystal {
public:
    using Clock = std::chrono::steady_clock;
    struct Deps {
        std::function<std::optional<std::filesystem::path>()> find;
        std::function<std::unique_ptr<P3bLink>(const std::filesystem::path&, std::string*)> open;
        std::function<bool(std::string_view)> process_running;
        /// Starts SteamVR (detached).
        std::function<bool()> start_steamvr;
        /// Starts the PiBridge app (detached), with SteamVR, when it does not run.
        std::function<bool()> start_app;
        /// Signals SteamVR's processes (SIGTERM to close it for a tracking change, SIGKILL when it does not close).
        std::function<void(int)> signal_steamvr;
        /// The headset's Lighthouse module is on USB; SteamVR finds a Lighthouse headset only when started after it
        /// (empty: not waited for).
        std::function<bool()> lighthouse_module;
        std::function<void(std::string_view)> say;
        /// The app's request file ("shutdown"): taken (removed) and done while the headset is connected.
        std::filesystem::path request;
    };
    struct Settings {
        std::string tracking = "lighthouse"; // lighthouse | slam | keep
        bool autostart_steamvr = true;
    };

    explicit Crystal(Deps deps);
    /// New settings; a changed tracking while the headset is connected switches it now.
    void set(Settings s);
    void tick(Clock::time_point now);

    bool present() const { return link_ != nullptr; }
    /// "", "connecting", "switching", "ready"
    std::string_view stage() const;
    /// How the headset tracks right now: "1" Lighthouse, "0" its own inside-out, from its live head reports (byte 113,
    /// flags of its inside-out: 1 orientation, 2 position; none in Lighthouse mode);
    /// before the first report, its persist.sys.pmx.lighthouse.enable as read ("" = not known yet).
    std::string lighthouse() const;
    /// Its own inside-out has the position right now (flag 2 and head report 0x31). False in Lighthouse mode, where the
    /// head report carries no pose of its own.
    bool tracked() const;
    /// The headset's battery in percent, -1 = not known.
    int battery() const;
    /// The live lens IPD in mm x 100 (0 = not known).
    int ipd_x100() const;

private:
    enum class Step { None, Connect, Query, Decide, TakeEffect, Ready };
    void send(p3b::Cmd c);
    void restart_steamvr(Clock::time_point now);
    void replies();
    void reset();
    void requests();

    Deps deps_;
    Settings settings_;
    std::unique_ptr<P3bLink> link_;
    Step step_ = Step::None;
    Clock::time_point at_{};          // when the current step is due
    Clock::time_point next_find_{};
    Clock::time_point next_poll_{};
    Clock::time_point settle_{};      // SteamVR waits for the Lighthouse module to come up
    std::string lh_enable_, lh_support_;
    Clock::time_point lhr_since_{};
    std::string video_;               // the headset's video link state (piPcSwicthIoctl reply), "" not reported
    Clock::time_point video_settled_{}; // SteamVR not before: the DisplayPort replug after video 120 is over
    enum class Restart { None, Close, Closing, Killed };
    Restart restart_ = Restart::None;   // SteamVR closed for a tracking change, started again by the Ready step
    Clock::time_point restart_at_{};   // the Lighthouse module on USB since (epoch: not seen)
    bool steamvr_started_ = false;

    /// In Lighthouse mode: the Lighthouse module has been on USB for a moment, or it never came (then SteamVR starts
    /// anyway after LHR_TIMEOUT, with a note).
    bool lighthouse_module_ready(Clock::time_point now);
};

} // namespace pibridge::keepalived
