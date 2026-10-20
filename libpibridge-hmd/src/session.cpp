// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax/src/session.rs (MIT OR Apache-2.0).
#include "pibridge/hmd/session.h"

#include "pibridge/hmd/descriptor.h"
#include "pibridge/hmd/discovery.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <charconv>
#include <cstring>
#include <fstream>
#include <sstream>

#include <unistd.h>

namespace pibridge::hmd {

using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;

// ------------------------------------------------------------------------------------------------ event queue

bool EventQueue::push(Event e) {
    {
        std::lock_guard g(mu_);
        if (q_.size() >= cap_) {
            dropped_++;
            return false;
        }
        q_.push_back(std::move(e));
    }
    cv_.notify_one();
    return true;
}

std::optional<Event> EventQueue::try_pop() {
    std::lock_guard g(mu_);
    if (q_.empty()) {
        return std::nullopt;
    }
    Event e = std::move(q_.front());
    q_.pop_front();
    return e;
}

std::optional<Event> EventQueue::pop(milliseconds timeout) {
    std::unique_lock g(mu_);
    if (!cv_.wait_for(g, timeout, [&] { return !q_.empty(); })) {
        return std::nullopt;
    }
    Event e = std::move(q_.front());
    q_.pop_front();
    return e;
}

std::vector<Event> EventQueue::drain() {
    std::lock_guard g(mu_);
    std::vector<Event> v(std::make_move_iterator(q_.begin()), std::make_move_iterator(q_.end()));
    q_.clear();
    return v;
}

std::uint64_t EventQueue::dropped() const {
    std::lock_guard g(mu_);
    return dropped_;
}

namespace {

class QueueHandler : public Handler {
public:
    QueueHandler(std::shared_ptr<EventQueue> q, bool replies) : q_(std::move(q)), replies_(replies) {}
    void on_state(State s) override { q_->push(s); }
    void on_imu(const ImuSample& s) override { q_->push(s); }
    void on_controller(const ControllerReport& r) override { q_->push(r); }
    void on_status(const ControllerStatus& s) override { q_->push(s); }
    void on_log(std::string_view l) override { q_->push(LogLine{std::string(l)}); }
    void on_reply(const Report& r) override {
        if (replies_) {
            q_->push(r);
        }
    }
    void on_attach(std::uint8_t slot, bool retry) override { q_->push(AttachEvent{slot, retry}); }

private:
    std::shared_ptr<EventQueue> q_;
    bool replies_;
};

} // namespace

std::unique_ptr<Handler> queue_handler(std::shared_ptr<EventQueue> q, bool replies) {
    return std::make_unique<QueueHandler>(std::move(q), replies);
}

Config Config::passive() {
    Config c;
    c.wake = false;
    c.keepalive = false;
    return c;
}

// ------------------------------------------------------------------------------------------------ hidraw backend

HidrawBackend::HidrawBackend() : roots_(discovery::Roots::from_env()) {}

Result<std::optional<Link>> HidrawBackend::connect() {
    const auto found = discovery::find_headset(roots_);
    if (!found || !found->nodes[0] || !found->nodes[2]) {
        return std::optional<Link>{}; // need IF0 and IF2
    }
    Link link;
    link.model = found->model;
    for (std::size_t i = 0; i < NUM_IF; i++) {
        if (!found->nodes[i]) {
            continue;
        }
        auto f = hid::open(*found->nodes[i], i == 0);
        if (f) {
            link.reads[i] = std::move(*f);
        } else if (i == 0 || i == 2) {
            return std::unexpected(f.error());
        }
        // a missing IF1/IF3 only costs its reports
    }
    auto ctl = link.reads[0].try_clone();
    if (!ctl) {
        return std::unexpected(ctl.error());
    }
    link.ctl = std::move(*ctl);
    return std::optional<Link>(std::move(link));
}

// ------------------------------------------------------------------------------------------------ shared state

namespace {

struct Pending {
    Session::Matcher matcher;
    std::optional<Report> done;
    bool aborted = false;
};

struct Counters {
    std::atomic<std::uint64_t> imu{0}, ctrl{0}, status{0}, log{0}, ka{0}, connects{0}, attaches{0}, refused{0};
};

} // namespace

struct Session::Shared {
    Config cfg;
    std::atomic<bool> stop{false};
    std::atomic<bool> connected{false};
    std::atomic<std::uint8_t> rate{0};
    Fd waker;
    Counters counters;
    std::function<std::uint64_t()> dropped;

    std::mutex mu; // guards everything below and is the mutex of `cv`
    std::condition_variable cv;
    Fd ctl;
    std::optional<Pending> pending;
    std::shared_ptr<const Allowlist> allow;
    std::unique_ptr<Gate> gate;
    std::string allow_error;
    const Model* model = nullptr;
    std::string last_error;

    std::mutex req_lock; // serialises requests

    /// Controller vibration waiting for the session thread (at most one command per hand per 8 ms, newest wins).
    mutable std::mutex haptic_mu;
    haptics::Limiter haptic;

    void log(const std::string& m) const {
        if (cfg.log) {
            cfg.log(m);
        }
    }
    void wake() const { eventfd_signal(waker.get()); }

    // Every write goes through the gate of the connected model.
    Result<void> write_ctl_locked(const Report& r) {
        if (!ctl) {
            return std::unexpected(Error{Errc::NotConnected, 0, {}});
        }
        if (!gate) {
            counters.refused++;
            return std::unexpected(Error{Errc::Refused, 0,
                                         "refused: no device description, nothing is sent (" + allow_error + ")"});
        }
        const auto sent = gate->send(r);
        if (sent) {
            return {};
        }
        if (const auto* ref = sent.error().refusal()) {
            counters.refused++;
            return std::unexpected(Error{Errc::Refused, 0, ref->message()});
        }
        const int e = std::get<int>(sent.error().e);
        if (hid::is_gone(e)) {
            return std::unexpected(Error{Errc::NotConnected, 0, {}});
        }
        return std::unexpected(io_error(e));
    }

    Result<void> write_ctl(const Report& r) {
        std::lock_guard g(mu);
        return write_ctl_locked(r);
    }
};

// ------------------------------------------------------------------------------------------------ session thread

namespace {

using Sh = Session::Shared;

void sleep_interruptible(Sh& sh, Clock::duration d) {
    if (sh.stop.load()) {
        return;
    }
    pollfd p = pollin(sh.waker.get());
    (void)ppoll(std::span(&p, 1), d);
    eventfd_drain(sh.waker.get());
}

void set_state(Sh& sh, Handler& h, State s) {
    const auto publish = [&] {
        std::lock_guard g(sh.mu);
        sh.connected.store(s == State::Connected);
        sh.cv.notify_all();
    };
    if (s == State::Connected) {
        // Handler first: when wait_connected returns (or is_connected reads true) the application has already been
        // told on_state(Connected), so it never reads a stale state right after waiting.
        h.on_state(s);
        publish();
    } else {
        publish();
        h.on_state(s);
    }
}

constexpr milliseconds STALE_AFTER{1500};
constexpr milliseconds RETRY_MIN{2500};

struct Attach {
    /// The wake is acked (any `4c .. 1a ..` reply seen on IF0).
    bool wake_acked = false;
    /// When to send the wake once more if it is still not acked.
    std::optional<Clock::time_point> wake_resend_at;
    std::array<std::optional<Clock::time_point>, 4> last_rx{};
    std::array<std::optional<Clock::time_point>, 4> seen37{};
    std::array<std::optional<Clock::time_point>, 4> tried{};
};

using Sched = std::deque<std::pair<Clock::time_point, Report>>;

void schedule_burst(Sched& q, Clock::time_point at, std::uint8_t slot) {
    const auto cmds = attach_commands(slot);
    for (std::size_t i = 0; i < cmds.size(); i++) {
        q.emplace_back(at + milliseconds(10 * i), cmds[i]);
    }
}

std::string trim(std::string s) {
    const auto b = s.find_first_not_of(" \t\r\n\v\f");
    if (b == std::string::npos) {
        return {};
    }
    const auto e = s.find_last_not_of(" \t\r\n\v\f");
    return s.substr(b, e - b + 1);
}

// The rate byte from the rate file: nullopt = `off`; missing or invalid = 0; `as u8` truncation like Rust.
std::optional<std::uint8_t> read_rate_file(const std::filesystem::path& p) {
    std::ifstream f(p);
    if (!f) {
        return 0;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    const std::string t = trim(ss.str());
    if (t == "off") {
        return std::nullopt;
    }
    std::string_view num = t;
    if (num.starts_with('+')) {
        num.remove_prefix(1);
    }
    std::int64_t v = 0;
    const auto [ptr, ec] = std::from_chars(num.data(), num.data() + num.size(), v);
    if (num.empty() || num.starts_with('+') || ec != std::errc{} || ptr != num.data() + num.size()) {
        return 0;
    }
    return static_cast<std::uint8_t>(v);
}

void teardown(Sh& sh, Handler& h, bool was_connected_hint) {
    {
        std::lock_guard hg(sh.haptic_mu);
        sh.haptic.clear();
    }
    {
        std::lock_guard g(sh.mu);
        sh.ctl.reset();
        sh.gate.reset();
        if (sh.pending) {
            sh.pending->aborted = true;
        }
        sh.cv.notify_all();
    }
    if (was_connected_hint && sh.connected.load()) {
        set_state(sh, h, State::Disconnected);
    }
}

void dispatch_if0(Sh& sh, Handler& h, const Report& d, Attach& att) {
    h.on_reply(d);
    if (d[0] != 0x4c) {
        return;
    }
    if (d[2] == cmd::ANNOUNCE) {
        att.wake_acked = true;
    }
    const auto st = parse_status(d);
    if (st[0] || st[1]) {
        for (const auto& s : st) {
            if (!s) {
                continue;
            }
            sh.counters.status++;
            if (sh.cfg.auto_attach) {
                const auto now = Clock::now();
                const auto& lr = att.last_rx[s->slot];
                if (!lr || now - *lr > STALE_AFTER) {
                    att.seen37[s->slot] = now; // awake but not attached
                }
            }
            h.on_status(*s);
        }
        if (st[0]) {
            return;
        }
    }
    std::lock_guard g(sh.mu);
    if (sh.pending && !sh.pending->done && sh.pending->matcher(d)) {
        sh.pending->done = d;
        sh.cv.notify_all();
    }
}

void dispatch(Sh& sh, Handler& h, std::size_t idx, const Report& d, std::size_t len, LineSplitter& log_split,
              Attach& att) {
    if (len < 4) {
        return;
    }
    // hidraw delivers whole reports; anything shorter was zero padded by the read buffer
    if (idx == 3) {
        log_split.push(std::span(d.data(), len), [&](std::string_view l) {
            sh.counters.log++;
            h.on_log(l);
        });
    } else if (idx == 0) {
        dispatch_if0(sh, h, d, att);
    } else if (const auto s = decode_imu(d)) {
        sh.counters.imu++;
        h.on_imu((*s)[0]);
        h.on_imu((*s)[1]);
    } else if (const auto c = decode_controller(d)) {
        sh.counters.ctrl++;
        att.last_rx[c->slot] = Clock::now();
        h.on_controller(*c);
    }
}

void run_link(Sh& sh, Link link, Handler& h) {
    const Config& cfg = sh.cfg;
    {
        std::lock_guard g(sh.mu);
        sh.ctl = std::move(link.ctl);
        sh.model = link.model;
        sh.last_error.clear();
        // the allowlist of this model; a new link is a new enumeration, so the min_interval history starts over
        if (cfg.allowlist) {
            sh.allow = cfg.allowlist;
        } else {
            auto a = cfg.allowlist_for
                         ? cfg.allowlist_for(*link.model)
                         : load_allowlist(*link.model, cfg.devices_dirs.empty() ? devices_dirs() : cfg.devices_dirs);
            if (a) {
                sh.allow = *a;
                sh.allow_error.clear();
            } else {
                sh.allow.reset();
                sh.allow_error = a.error().msg;
            }
        }
        sh.gate.reset();
        if (sh.allow) {
            const int fd = sh.ctl.get();
            sh.gate = std::make_unique<Gate>(*sh.allow, std::uint8_t{0}, [fd](std::span<const std::uint8_t> b) -> long {
                const auto n = ::write(fd, b.data(), b.size());
                return n < 0 ? -static_cast<long>(errno) : static_cast<long>(n);
            });
        }
    }
    if (!sh.gate) {
        sh.log("no device description for " + std::string(link.model->name) + ": nothing is sent (" + sh.allow_error +
               ")");
    }
    sh.counters.connects++;
    if (cfg.wake) {
        if (auto w = sh.write_ctl(wake()); !w) {
            sh.log("wake failed: " + w.error().message());
            if (w.error().code != Errc::Refused) {
                teardown(sh, h, false);
                return;
            }
        }
    }
    set_state(sh, h, State::Connected);

    LineSplitter log_split;
    Sched sched;
    Attach att;
    if (cfg.wake) {
        att.wake_resend_at = Clock::now() + WAKE_RESEND_AFTER;
    }
    const auto start = Clock::now();
    if (cfg.auto_attach) {
        for (const std::uint8_t slot : {SLOT_LEFT, SLOT_RIGHT}) {
            schedule_burst(sched, start, slot);
            att.tried[slot] = start;
            h.on_attach(slot, false);
        }
    }
    auto next_ka = start;
    std::uint64_t ka_sent = 0;
    bool off_sent = false; // `off` in the rate file: the closing rate-0 keepalive went out
    bool ka_refusal_logged = false;
    std::array<pollfd, NUM_IF + 1> pf{};
    std::array<std::size_t, NUM_IF + 1> map{};
    bool gone = false;

    while (!sh.stop.load()) {
        const auto now = Clock::now();
        if (cfg.keepalive && now >= next_ka) {
            next_ka += cfg.period;
            if (next_ka < now) {
                next_ka = now + cfg.period;
            }
            std::optional<std::uint8_t> rate;
            if (cfg.rate_file) {
                rate = read_rate_file(*cfg.rate_file);
            } else {
                rate = sh.rate.load();
            }
            std::optional<std::uint8_t> send;
            if (rate) {
                off_sent = false;
                send = rate;
            } else if (!off_sent) {
                off_sent = true;
                send = 0;
            }
            if (send) {
                const auto w = sh.write_ctl(keepalive(*send));
                if (w) {
                    ka_sent++;
                    sh.counters.ka++;
                } else if (w.error().code == Errc::NotConnected) {
                    break;
                } else if (w.error().code == Errc::Refused) {
                    if (!ka_refusal_logged) {
                        sh.log("keepalive with rate " + std::to_string(*send) + " not sent: " + w.error().message());
                        ka_refusal_logged = true;
                    }
                } else if (w.error().os_errno != EAGAIN) {
                    sh.log("keepalive write: " + w.error().message());
                }
            }
        }
        if (att.wake_resend_at) {
            if (att.wake_acked) {
                att.wake_resend_at.reset();
            } else if (now >= *att.wake_resend_at) {
                att.wake_resend_at.reset();
                sh.log("wake not acked after " + std::to_string(WAKE_RESEND_AFTER.count()) +
                       " ms: sending it once more");
                if (auto w = sh.write_ctl(wake()); !w && w.error().code == Errc::NotConnected) {
                    break;
                }
            }
        }
        bool lost = false;
        while (!sched.empty() && sched.front().first <= now) {
            const Report c = sched.front().second;
            sched.pop_front();
            if (auto w = sh.write_ctl(c); !w && w.error().code == Errc::NotConnected) {
                lost = true;
                break;
            }
        }
        if (lost) {
            break;
        }
        std::vector<haptics::Due> due;
        {
            std::lock_guard hg(sh.haptic_mu);
            due = sh.haptic.take_due(now);
        }
        for (const auto& [slot, amp, ms] : due) {
            if (auto w = sh.write_ctl(haptic_command(slot, amp, ms)); !w) {
                if (w.error().code == Errc::NotConnected) {
                    lost = true;
                    break;
                }
                sh.log("haptic write: " + w.error().message());
            }
        }
        if (lost) {
            break;
        }
        if (cfg.auto_attach) {
            for (const std::uint8_t slot : {SLOT_LEFT, SLOT_RIGHT}) {
                const auto& lr = att.last_rx[slot];
                const auto& seen = att.seen37[slot];
                const auto& tried = att.tried[slot];
                const bool stale = !lr || now - *lr > STALE_AFTER;
                const bool awake = (seen && tried && *seen > *tried) || (seen && !tried);
                const bool rate_ok = !tried || now - *tried >= RETRY_MIN;
                if (awake && stale && rate_ok) {
                    sh.log("controller slot " + std::to_string(slot) +
                           " awake (0x37) but silent: re-sending attach burst");
                    schedule_burst(sched, now, slot);
                    att.tried[slot] = now;
                    sh.counters.attaches++;
                    h.on_attach(slot, true);
                }
            }
        }

        // poll: waker + interfaces
        std::size_t n = 0;
        pf[n] = pollin(sh.waker.get());
        map[n] = SIZE_MAX;
        n++;
        for (std::size_t i = 0; i < NUM_IF; i++) {
            if (link.reads[i]) {
                pf[n] = pollin(link.reads[i].get());
                map[n] = i;
                n++;
            }
        }
        auto wake_at = cfg.keepalive ? next_ka : now + std::chrono::seconds(1);
        if (!sched.empty()) {
            wake_at = std::min(wake_at, sched.front().first);
        }
        if (att.wake_resend_at) {
            wake_at = std::min(wake_at, *att.wake_resend_at);
        }
        {
            std::lock_guard hg(sh.haptic_mu);
            if (const auto t = sh.haptic.next_due()) {
                wake_at = std::min(wake_at, *t);
            }
        }
        const auto wait = std::max(wake_at - Clock::now(), Clock::duration::zero());
        if (!ppoll(std::span(pf.data(), n), std::chrono::duration_cast<std::chrono::nanoseconds>(wait))) {
            break;
        }
        if (pf[0].revents != 0) {
            eventfd_drain(sh.waker.get());
        }
        for (std::size_t k = 1; k < n; k++) {
            const short re = pf[k].revents;
            const std::size_t idx = map[k];
            if (re & POLLIN) {
                // One read per ready fd and cycle: a second read would only return EAGAIN almost every time (reports
                // arrive one by one) and cost a syscall; level-triggered ppoll hands back anything still queued.
                Report buf{};
                const auto len = ::read(link.reads[idx].get(), buf.data(), buf.size());
                if (len > 0) {
                    dispatch(sh, h, idx, buf, static_cast<std::size_t>(len), log_split, att);
                } else if (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                    gone = true;
                }
            }
            if (re & (POLLERR | POLLHUP | POLLNVAL)) {
                gone = true;
            }
        }
        if (gone) {
            break;
        }
    }
    teardown(sh, h, true);
    h.on_link_closed(Clock::now() - start, ka_sent);
}

void thread_main(const std::stop_token& st, std::shared_ptr<Sh> shp, std::unique_ptr<Backend> backend,
                 std::unique_ptr<Handler> handler) {
    Sh& sh = *shp;
    bool warned = false;
    while (!sh.stop.load() && !st.stop_requested()) {
        auto r = backend->connect();
        if (r && *r) {
            warned = false;
            run_link(sh, std::move(**r), *handler);
            sleep_interruptible(sh, milliseconds(100));
        } else if (r) {
            sleep_interruptible(sh, milliseconds(50));
        } else {
            const bool perm = r.error().os_errno == EACCES || r.error().os_errno == EPERM;
            const std::string msg = "cannot open the headset: " + r.error().message() +
                                    (perm ? " (udev rule 70-pibridge.rules missing?)" : "");
            {
                std::lock_guard g(sh.mu);
                sh.last_error = msg;
            }
            if (!warned) {
                sh.log(msg);
                warned = true;
            }
            sleep_interruptible(sh, perm ? std::chrono::seconds(2) : std::chrono::seconds(1));
        }
    }
}

} // namespace

// ------------------------------------------------------------------------------------------------ public handle

Result<std::unique_ptr<Session>> Session::open(Config cfg, std::unique_ptr<Handler> handler) {
    return open_with_backend(std::move(cfg), std::make_unique<HidrawBackend>(), std::move(handler));
}

Result<std::unique_ptr<Session>> Session::open_with_backend(Config cfg, std::unique_ptr<Backend> backend,
                                                            std::unique_ptr<Handler> handler) {
    auto w = eventfd();
    if (!w) {
        return std::unexpected(w.error());
    }
    std::unique_ptr<Session> s(new Session());
    s->sh_ = std::make_shared<Shared>();
    s->sh_->cfg = std::move(cfg);
    s->sh_->waker = std::move(*w);
    if (!handler) {
        handler = std::make_unique<NoopHandler>();
    }
    s->thread_ = std::jthread(thread_main, s->sh_, std::move(backend), std::move(handler));
    return s;
}

Result<std::pair<std::unique_ptr<Session>, std::shared_ptr<EventQueue>>>
Session::open_channel_with_backend(Config cfg, std::unique_ptr<Backend> backend, std::size_t capacity, bool replies) {
    auto q = std::make_shared<EventQueue>(capacity);
    auto s = open_with_backend(std::move(cfg), std::move(backend), queue_handler(q, replies));
    if (!s) {
        return std::unexpected(s.error());
    }
    (*s)->sh_->dropped = [q] { return q->dropped(); };
    return std::pair{std::move(*s), q};
}

Session::~Session() { close(); }

void Session::close() {
    if (!sh_) {
        return;
    }
    sh_->stop.store(true);
    sh_->wake();
    if (thread_.joinable()) {
        thread_.request_stop();
        thread_.join();
    }
}

bool Session::is_connected() const { return sh_->connected.load(); }

Result<void> Session::wait_connected(milliseconds timeout) const {
    std::unique_lock g(sh_->mu);
    if (!sh_->cv.wait_for(g, timeout, [&] { return sh_->connected.load(); })) {
        return std::unexpected(Error{Errc::Timeout, 0, {}});
    }
    return {};
}

void Session::set_video_rate(std::uint8_t fps) { sh_->rate.store(fps); }

Stats Session::stats() const {
    const auto& c = sh_->counters;
    Stats s;
    s.imu_reports = c.imu.load();
    s.controller_reports = c.ctrl.load();
    s.status_messages = c.status.load();
    s.log_lines = c.log.load();
    s.keepalives = c.ka.load();
    s.connects = c.connects.load();
    s.auto_attaches = c.attaches.load();
    s.refused = c.refused.load();
    s.dropped_events = sh_->dropped ? sh_->dropped() : 0;
    return s;
}

const Model* Session::model() const {
    std::lock_guard g(sh_->mu);
    return sh_->model;
}

std::string Session::last_error() const {
    std::lock_guard g(sh_->mu);
    return sh_->last_error;
}

Result<void> Session::send(std::uint8_t c, std::uint8_t opt, std::span<const std::uint8_t> data) {
    const auto r = build_command(c, opt, data);
    if (!r) {
        return std::unexpected(Error{Errc::TooLong, 0, {}});
    }
    return sh_->write_ctl(*r);
}

Result<void> Session::send_report(const Report& r) { return sh_->write_ctl(r); }

Result<std::vector<std::uint8_t>> Session::request(std::uint8_t c, std::uint8_t opt,
                                                   std::span<const std::uint8_t> data, milliseconds timeout) {
    const auto r = build_command(c, opt, data);
    if (!r) {
        return std::unexpected(Error{Errc::TooLong, 0, {}});
    }
    auto rep = request_with(*r, timeout, [c, opt](const Report& d) { return d[2] == c && d[3] == opt; });
    if (!rep) {
        return std::unexpected(rep.error());
    }
    const auto p = parse_reply(*rep);
    if (!p) {
        return std::vector<std::uint8_t>{};
    }
    return std::vector<std::uint8_t>(p->data.begin(), p->data.end());
}

Result<Report> Session::request_with(const Report& report, milliseconds timeout, Matcher matcher) {
    Shared& sh = *sh_;
    std::lock_guard serial(sh.req_lock);
    std::unique_lock g(sh.mu);
    sh.pending = Pending{std::move(matcher), std::nullopt, false};
    if (auto w = sh.write_ctl_locked(report); !w) {
        sh.pending.reset();
        return std::unexpected(w.error());
    }
    const auto deadline = Clock::now() + timeout;
    Result<Report> res = std::unexpected(Error{Errc::Timeout, 0, {}});
    while (true) {
        if (!sh.pending || sh.pending->aborted) {
            res = std::unexpected(Error{Errc::NotConnected, 0, {}});
            break;
        }
        if (sh.pending->done) {
            res = *sh.pending->done;
            break;
        }
        if (Clock::now() >= deadline) {
            break;
        }
        sh.cv.wait_until(g, deadline);
    }
    sh.pending.reset();
    return res;
}

Result<std::string> Session::read_serial(std::uint8_t opt, milliseconds timeout) {
    const std::uint8_t d[] = {0x1e};
    auto r = request(cmd::SERIAL, opt, d, timeout);
    if (!r) {
        return std::unexpected(r.error());
    }
    const auto s = parse_serial(*r);
    // String::from_utf8_lossy: the serial is ASCII; anything else is replaced
    std::string out;
    for (const auto b : s) {
        out += b < 0x80 ? static_cast<char>(b) : '?';
    }
    return out;
}

Result<std::vector<std::uint8_t>> Session::read_version(std::uint8_t opt, milliseconds timeout) {
    return request(cmd::VERSION, opt, {}, timeout);
}

Result<std::vector<std::uint8_t>> Session::read_calibration_blob(milliseconds per_try) {
    calib::Assembler asm_;
    for (std::size_t a = 0; a < calib::TOTAL; a += calib::BLOCK) {
        const auto addr = static_cast<std::uint16_t>(a);
        const Report req = calib::read_block_request(addr);
        bool ok = false;
        for (int t = 0; t < 3; t++) {
            auto r = request_with(req, per_try, [addr](const Report& d) {
                const auto b = calib::parse_block_reply(d);
                return b && b->first == addr;
            });
            if (r) {
                asm_.feed(*r);
                ok = true;
                break;
            }
            if (r.error().code != Errc::Timeout) {
                return std::unexpected(r.error());
            }
        }
        if (!ok) {
            return std::unexpected(Error{Errc::Timeout, 0, {}});
        }
    }
    return std::vector<std::uint8_t>(asm_.blob().begin(), asm_.blob().end());
}

Result<void> Session::controller_attach(std::uint8_t slot) {
    const auto cmds = attach_commands(slot);
    for (std::size_t i = 0; i < cmds.size(); i++) {
        if (auto w = sh_->write_ctl(cmds[i]); !w) {
            return w;
        }
        if (i < 3) {
            std::this_thread::sleep_for(milliseconds(10));
        }
    }
    return {};
}

Result<void> Session::video_prepare(std::uint8_t fps) {
    set_video_rate(fps);
    for (const auto& c : video_prepare_commands(fps)) {
        if (auto w = sh_->write_ctl(c); !w) {
            return w;
        }
    }
    return {};
}

bool Session::haptic(std::uint8_t slot, std::optional<haptics::Pulse> pulse) {
    bool ok = false;
    {
        std::lock_guard g(sh_->haptic_mu);
        ok = sh_->haptic.offer(slot, pulse);
    }
    if (ok && pulse) {
        sh_->wake();
    }
    return ok;
}

std::pair<std::uint64_t, std::uint64_t> Session::haptic_counts() const {
    std::lock_guard g(sh_->haptic_mu);
    return {sh_->haptic.sent, sh_->haptic.replaced};
}

} // namespace pibridge::hmd
