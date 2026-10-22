// SPDX-License-Identifier: MIT
#include "crystal.h"

#include "pibridge/hmd/pose_shm.h"

#include <atomic>
#include <cmath>
#include <fstream>
#include <cerrno>
#include <csignal>
#include <mutex>
#include <thread>

#include <cstring>

#include <fcntl.h>
#include <time.h>
#include <sys/wait.h>
#include <unistd.h>

namespace pibridge::keepalived {

using namespace std::chrono_literals;
namespace pose_shm = pibridge::hmd::pose_shm;

namespace {

/// Pimax Play's timing (capture 2026-10-22): the connect frames ~0.5 s after the HID opens, the property reads then,
/// TakeEffect 8 s after EnableLighthouse, getConnectState every 10 s. The Lighthouse module (Watchman) was on USB 2 s
/// after EnableLighthouse and the base stations bootstrapped ~18 s later; SteamVR is started 5 s after TakeEffect.
constexpr auto CONNECT_DELAY = 2s;
constexpr auto QUERY_DELAY = 1s;
constexpr auto REPLY_WAIT = 3s;
constexpr auto TAKE_EFFECT_DELAY = 8s;
constexpr auto SETTLE = 5s;
// 2026-10-22: with the Lighthouse mode kept from the last session the LHR came up 7 s after the headset, 1 s after
// SteamVR had looked for it: SteamVR then found no headset at all.
constexpr auto LHR_SETTLE = 3s;
// `video 120` makes the headset replug its DisplayPort. SteamVR started during that (2 s after it, 2026-10-22) found
// only the 72 Hz mode (the ones that need DSC not there yet) and the headset never took the picture ("video link 1");
// started later, 90 Hz and "video link 2". On Windows the link came up ~8 s after video 120.
constexpr auto VIDEO_SETTLE = 10s;
constexpr auto STEAMVR_CLOSE_WAIT = 10s;
constexpr auto LHR_TIMEOUT = 30s;
constexpr auto POLL = 10s;
constexpr auto FIND_EVERY = 1s;

std::string read_trimmed(const std::filesystem::path& p) {
    std::string s;
    std::getline(std::ifstream(p), s);
    return s;
}

std::int64_t monotonic_ns() {
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return std::int64_t{ts.tv_sec} * 1'000'000'000 + ts.tv_nsec;
}

class Hidraw : public P3bLink {
public:
    explicit Hidraw(int fd)
        : fd_(fd), poses_(pose_shm::Publisher::create(pose_shm::default_path())), reader_([this] { run(); }) {}
    ~Hidraw() override {
        stop_ = true;
        ::close(fd_); // unblocks the read
        if (reader_.joinable()) {
            reader_.join();
        }
        if (poses_) {
            poses_->gone(monotonic_ns()); // the SteamVR driver sees no pose at once, not a frozen one
        }
    }
    bool write(const p3b::Report& r) override {
        std::uint8_t b[1 + p3b::REPORT_SIZE];
        b[0] = 0; // no report ids
        std::copy(r.begin(), r.end(), b + 1);
        const auto n = ::write(fd_, b, sizeof b);
        if (n != static_cast<ssize_t>(sizeof b)) {
            alive_ = false;
            return false;
        }
        return true;
    }
    std::vector<p3b::Rpc> take_replies() override {
        std::lock_guard g(mu_);
        return std::exchange(replies_, {});
    }
    bool alive() const override { return alive_; }
    std::optional<p3b::Head> last_head() const override {
        std::lock_guard g(mu_);
        return head_;
    }

private:
    void run() {
        p3b::Report r;
        while (!stop_) {
            const auto n = ::read(fd_, r.data(), r.size());
            if (n < 0 && errno == EINTR) {
                continue;
            }
            if (n <= 0) {
                alive_ = false;
                return;
            }
            // the head report: kept (the newest) for the battery, the IPD and the tracking mode; head and controllers
            // go to the pose file for the SteamVR driver (the headset's own inside-out tracking)
            const auto data = std::span(r.data(), static_cast<std::size_t>(n));
            if ((r[0] & 0x0f) == 1 && n >= 140) {
                if (auto h = p3b::decode_head(data)) {
                    if (poses_) {
                        poses_->head(*h, monotonic_ns());
                    }
                    std::lock_guard g(mu_);
                    head_ = *h;
                }
                continue;
            }
            if (((r[0] & 0x0f) == 2 || (r[0] & 0x0f) == 3) && r[0] >= 0x40 && n >= 62) {
                if (auto c = p3b::decode_controller(data); c && poses_) {
                    poses_->controller(*c, monotonic_ns());
                }
                continue;
            }
            // the RPC replies only; the tracking stream is not an RPC frame (decode refuses it)
            if (r[0] == p3b::type::COMMAND || r[0] == p3b::type::CALL || r[0] == p3b::type::GETPROP) {
                if (auto d = p3b::decode(std::span(r.data(), static_cast<std::size_t>(n)))) {
                    std::lock_guard g(mu_);
                    if (replies_.size() < 64) {
                        replies_.push_back(std::move(*d));
                    }
                }
            }
        }
    }
    int fd_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> alive_{true};
    mutable std::mutex mu_;
    std::vector<p3b::Rpc> replies_;
    std::optional<p3b::Head> head_;
    std::optional<pose_shm::Publisher> poses_; // only the reader thread writes it (and the destructor after it)
    std::thread reader_;
};

} // namespace

std::unique_ptr<P3bLink> open_hidraw(const std::filesystem::path& node, std::string* error) {
    const int fd = ::open(node.c_str(), O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        if (error) {
            *error = node.string() + ": " + std::strerror(errno);
        }
        return nullptr;
    }
    return std::make_unique<Hidraw>(fd);
}

void signal_steamvr_processes(int sig, const std::filesystem::path& proc) {
    std::error_code ec;
    for (const auto& d : std::filesystem::directory_iterator(proc, ec)) {
        const std::string pid = d.path().filename().string();
        if (pid.empty() || pid.find_first_not_of("0123456789") != std::string::npos) {
            continue;
        }
        std::error_code e;
        const std::string exe = std::filesystem::read_symlink(d.path() / "exe", e).string();
        // SteamVR's own programs (vrmonitor, vrserver, vrcompositor, vrwebhelper, ...), not the games
        if (!e && exe.find("/SteamVR/bin/") != std::string::npos) {
            ::kill(std::stoi(pid), sig);
        }
    }
}

bool lighthouse_module_present(const std::filesystem::path& usb_devices) {
    std::error_code ec;
    for (const auto& d : std::filesystem::directory_iterator(usb_devices, ec)) {
        if (read_trimmed(d.path() / "idVendor") == "28de" && read_trimmed(d.path() / "idProduct") == "2300") {
            return true;
        }
    }
    return false;
}

std::vector<std::string> in_own_scope(std::vector<std::string> argv) {
    // a child of a systemd service stays in the service's cgroup: a reload signal to the unit, its restart or its stop
    // would reach SteamVR and the app too (2026-10-22: SIGHUP went to PiBridge). systemd-run --scope moves it out.
    for (const char* p : {"/usr/bin/systemd-run", "/bin/systemd-run"}) {
        if (::access(p, X_OK) == 0) {
            std::vector<std::string> out{p, "--user", "--scope", "--quiet", "--"};
            out.insert(out.end(), argv.begin(), argv.end());
            return out;
        }
    }
    return argv;
}

bool start_detached(const std::vector<std::string>& argv) {
    if (argv.empty()) {
        return false;
    }
    std::vector<std::string> store = argv;
    std::vector<char*> av;
    for (auto& a : store) {
        av.push_back(a.data());
    }
    av.push_back(nullptr);
    const pid_t pid = ::fork();
    if (pid < 0) {
        return false;
    }
    if (pid == 0) {
        // only async-signal-safe calls from here on (the daemon has threads)
        ::setsid();
        const pid_t g = ::fork();
        if (g != 0) {
            ::_exit(g < 0 ? 1 : 0);
        }
        const int null = ::open("/dev/null", O_RDWR);
        if (null >= 0) {
            ::dup2(null, 0);
            ::dup2(null, 1);
            ::dup2(null, 2);
        }
        ::execvp(av[0], av.data());
        ::_exit(127);
    }
    int st = 0;
    while (::waitpid(pid, &st, 0) < 0 && errno == EINTR) {
    }
    return WIFEXITED(st) && WEXITSTATUS(st) == 0;
}

Crystal::Crystal(Deps deps) : deps_(std::move(deps)) {}

void Crystal::set(Settings s) {
    const bool changed = s.tracking != settings_.tracking;
    settings_ = std::move(s);
    if (changed && link_ && (step_ == Step::Ready || step_ == Step::Decide)) {
        deps_.say("crystal: tracking wanted now: " + settings_.tracking);
        lh_enable_.clear(); // read it again, then switch
        step_ = Step::Query;
        at_ = {};
        // SteamVR picks its headset device once, at its start (Lighthouse: the Lighthouse module; inside-out:
        // CustomHeadsetOpenVR's SLAM headset): it is closed now and started again once the headset has switched
        if (deps_.process_running && deps_.process_running("vrserver") && deps_.signal_steamvr) {
            restart_ = Restart::Close;
        }
    }
}

void Crystal::restart_steamvr(Clock::time_point now) {
    switch (restart_) {
    case Restart::None: break;
    case Restart::Close:
        deps_.say("crystal: closing SteamVR to start it again (it reads its headset device and refresh rate once, at "
                  "its start)");
        deps_.signal_steamvr(SIGTERM);
        restart_ = Restart::Closing;
        restart_at_ = now;
        break;
    case Restart::Closing:
    case Restart::Killed:
        if (!deps_.process_running("vrserver")) {
            deps_.say("crystal: SteamVR closed");
            restart_ = Restart::None;
            steamvr_started_ = false; // the Ready step starts it again (after the Lighthouse module / the settle time)
            lhr_since_ = {};
        } else if (restart_ == Restart::Closing && now >= restart_at_ + STEAMVR_CLOSE_WAIT) {
            deps_.say("crystal: SteamVR did not close in time: killing it");
            deps_.signal_steamvr(SIGKILL);
            restart_ = Restart::Killed;
        }
        break;
    }
}

std::string Crystal::lighthouse() const {
    if (link_) {
        if (const auto h = link_->last_head()) {
            return h->tracking != 0 ? "0" : "1"; // flags of its own inside-out; none in Lighthouse mode
        }
    }
    return lh_enable_;
}

bool Crystal::tracked() const {
    if (!link_) {
        return false;
    }
    const auto h = link_->last_head();
    return h && (h->tracking & 2) != 0 && h->tracked; // its inside-out has the position, not just the orientation
}

int Crystal::battery() const {
    if (!link_) {
        return -1;
    }
    const auto h = link_->last_head();
    return h ? h->battery : -1;
}

int Crystal::ipd_x100() const {
    if (!link_) {
        return 0;
    }
    const auto h = link_->last_head();
    return h && h->ipd_mm >= 40.0f && h->ipd_mm <= 90.0f ? static_cast<int>(std::lround(h->ipd_mm * 100.0f)) : 0;
}

std::string_view Crystal::stage() const {
    switch (step_) {
    case Step::None: return "";
    case Step::Connect:
    case Step::Query: return "connecting";
    case Step::Decide:
    case Step::TakeEffect: return "switching";
    case Step::Ready: return "ready";
    }
    return "";
}

void Crystal::send(p3b::Cmd c) {
    const auto r = p3b::encode(p3b::command(c));
    if (r && link_->write(*r)) {
        deps_.say("crystal: sent " + std::string(p3b::name_of(c)));
    } else {
        deps_.say("crystal: could not send " + std::string(p3b::name_of(c)));
    }
}

void Crystal::reset() {
    link_.reset();
    step_ = Step::None;
    lh_enable_.clear();
    lh_support_.clear();
    steamvr_started_ = false;
    lhr_since_ = {};
    video_.clear();
    restart_ = Restart::None;
}

bool Crystal::lighthouse_module_ready(Clock::time_point now) {
    if (!deps_.lighthouse_module) {
        return true;
    }
    if (!deps_.lighthouse_module()) {
        lhr_since_ = {};
        if (now < settle_ + LHR_TIMEOUT) {
            return false;
        }
        deps_.say("crystal: the headset's Lighthouse module (USB 28de:2300) did not show up; starting SteamVR anyway");
        return true;
    }
    if (lhr_since_ == Clock::time_point{}) {
        lhr_since_ = now;
    }
    return now >= lhr_since_ + LHR_SETTLE;
}

void Crystal::replies() {
    for (const auto& r : link_->take_replies()) {
        if (r.name == "persist.sys.pmx.lighthouse.enable") {
            lh_enable_ = r.value;
        } else if (r.name == "persist.sys.pmx.lighthouse.support") {
            lh_support_ = r.value;
        }
        if (r.name == "piPcSwicthIoctl") {
            // the video link as the headset sees it (to video 120 and every getConnectState): "1" waiting for the
            // PC's picture, "2" the panel shows it (Windows 2026-10-22: "2" with PiService's "panel report ok");
            // other replies are the encrypted connection status
            if ((r.value == "1" || r.value == "2" || r.value == "0") && r.value != video_) {
                video_ = r.value;
                deps_.say("crystal: video link " + video_ +
                          (video_ == "2" ? " (the panel shows the PC's picture)" : " (no PC picture on the panel)"));
            }
            continue;
        }
        deps_.say("crystal: reply " + r.name + (r.value.empty() ? "" : " = " + r.value.substr(0, 40)));
    }
}

void Crystal::requests() {
    if (deps_.request.empty()) {
        return;
    }
    std::error_code ec;
    if (!std::filesystem::exists(deps_.request, ec)) {
        return;
    }
    std::string what;
    {
        std::ifstream f(deps_.request);
        std::getline(f, what);
    }
    std::filesystem::remove(deps_.request, ec);
    if (what == "shutdown") {
        deps_.say("crystal: switching the headset off (app request)");
        send(p3b::Cmd::Shutdown);
    } else if (what == "restart-steamvr") {
        // the app changed something SteamVR reads at its start only (the Crystal's refresh rate)
        if (deps_.process_running && deps_.process_running("vrserver") && deps_.signal_steamvr && restart_ == Restart::None) {
            restart_ = Restart::Close;
        }
    } else {
        deps_.say("crystal: unknown request \"" + what + "\"");
    }
}

void Crystal::tick(Clock::time_point now) {
    if (!link_) {
        if (now < next_find_) {
            return;
        }
        next_find_ = now + FIND_EVERY;
        const auto node = deps_.find ? deps_.find() : std::nullopt;
        if (!node) {
            return;
        }
        std::string err;
        link_ = deps_.open(*node, &err);
        if (!link_) {
            deps_.say("crystal: found on " + node->string() + " but cannot open it: " + err);
            next_find_ = now + 5s;
            return;
        }
        deps_.say("crystal: original Pimax Crystal (P3B) on " + node->string() + ", tracking wanted: " +
                  settings_.tracking);
        step_ = Step::Connect;
        at_ = now + CONNECT_DELAY;
        next_poll_ = now + POLL;
        return;
    }
    if (!link_->alive()) {
        deps_.say("crystal: headset gone (switched off or unplugged)");
        reset();
        next_find_ = now + FIND_EVERY;
        return;
    }
    replies();
    requests();
    restart_steamvr(now);
    if (step_ != Step::Connect && now >= next_poll_) {
        next_poll_ = now + POLL;
        send(p3b::Cmd::ConnectState);
    }
    switch (step_) {
    case Step::None: break;
    case Step::Connect:
        if (now >= at_) {
            send(p3b::Cmd::ScreenOn);
            send(p3b::Cmd::KeepRuntime);
            send(p3b::Cmd::PcVideo120);
            video_settled_ = now + VIDEO_SETTLE;
            step_ = Step::Query;
            at_ = now + QUERY_DELAY;
        }
        break;
    case Step::Query:
        if (now >= at_) {
            send(p3b::Cmd::GetLighthouseSupport);
            send(p3b::Cmd::GetLighthouseEnable);
            if (lh_support_.empty()) { // once per connection (not again on a tracking change): the eye tracker on USB
                send(p3b::Cmd::TobiiEnable);
            }
            step_ = Step::Decide;
            at_ = now + REPLY_WAIT;
        }
        break;
    case Step::Decide:
        if (lh_enable_.empty() && now < at_) {
            break; // wait for the reply
        }
        if (settings_.tracking == "lighthouse" && lh_enable_ != "1") {
            if (lh_support_ == "0") {
                deps_.say("crystal: the headset reports no Lighthouse support; tracking stays inside-out");
                step_ = Step::Ready;
                settle_ = now;
            } else {
                send(p3b::Cmd::LighthouseOn);
                step_ = Step::TakeEffect;
                at_ = now + TAKE_EFFECT_DELAY;
            }
        } else if (settings_.tracking == "slam" && lh_enable_ == "1") {
            send(p3b::Cmd::LighthouseOff);
            step_ = Step::Ready;
            settle_ = now + SETTLE;
        } else {
            step_ = Step::Ready;
            settle_ = now;
        }
        break;
    case Step::TakeEffect:
        if (now >= at_) {
            send(p3b::Cmd::LighthouseTakeEffect);
            send(p3b::Cmd::GetLighthouseEnable);
            step_ = Step::Ready;
            settle_ = now + SETTLE;
        }
        break;
    case Step::Ready:
        if (!steamvr_started_ && settings_.autostart_steamvr && restart_ == Restart::None) {
            const bool slam = settings_.tracking == "slam" || (settings_.tracking == "keep" && lh_enable_ != "1");
            // watched from the first Ready tick: the module may come up during the settle time
            if (!(slam || lighthouse_module_ready(now)) || now < settle_ || now < video_settled_) {
                break;
            }
            steamvr_started_ = true; // once per connection, whatever happens
            // inside-out: CustomHeadsetOpenVR's SLAM headset reads the poses this daemon writes to shared memory
            if (deps_.process_running && deps_.process_running("vrserver")) {
                deps_.say("crystal: SteamVR already runs");
            } else if (deps_.start_steamvr && deps_.start_steamvr()) {
                deps_.say("crystal: starting SteamVR");
            } else {
                deps_.say("crystal: could not start SteamVR (steam not found?)");
            }
            if (deps_.start_app && !(deps_.process_running && deps_.process_running("pibridge"))) {
                deps_.say(deps_.start_app() ? "crystal: starting PiBridge" : "crystal: could not start PiBridge");
            }
        }
        break;
    }
}

} // namespace pibridge::keepalived
