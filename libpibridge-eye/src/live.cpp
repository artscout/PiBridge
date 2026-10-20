// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/live.rs (MIT OR Apache-2.0).
#include "pibridge/eye/live.h"
#include "pibridge_env.h"

#include "pibridge/eye/frames.h"
#include "pibridge/eye/usbfs.h"

#include "pibridge/eyemath/clock.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <fstream>
#include <iterator>
#include <sstream>

namespace pibridge::eye::live {

namespace fs = std::filesystem;

std::string_view state_name(State s) {
    switch (s) {
    case State::Searching: return "Searching";
    case State::Connecting: return "Connecting";
    case State::Streaming: return "Streaming";
    case State::Reconnecting: return "Reconnecting";
    case State::Stopped: return "Stopped";
    }
    return "Searching";
}

namespace {

void stderr_log(std::string_view m) {
    std::fprintf(stderr, "pibridge-eye: %.*s\n", static_cast<int>(m.size()), m.data());
}

std::optional<std::string> env(const char* n) {
    const char* v = pibridge::env(n);
    if (v == nullptr) {
        return std::nullopt;
    }
    return std::string(v);
}

std::string_view trim(std::string_view s) {
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t' || s.front() == '\n' || s.front() == '\r')) {
        s.remove_prefix(1);
    }
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\n' || s.back() == '\r')) {
        s.remove_suffix(1);
    }
    return s;
}

bool env_on(const char* n) {
    const auto v = env(n);
    if (!v) {
        return false;
    }
    const auto t = trim(*v);
    return t == "1" || t == "true" || t == "yes" || t == "on";
}

bool env_off(const char* n) {
    const auto v = env(n);
    if (!v) {
        return false;
    }
    const auto t = trim(*v);
    return t == "0" || t == "false" || t == "no" || t == "off";
}

// Whole file; nullopt with errno-like text on failure. `missing` is set when the file does not exist.
std::expected<std::string, std::string> read_file(const fs::path& p, bool& missing) {
    missing = false;
    std::error_code ec;
    if (!fs::exists(p, ec)) {
        missing = true;
        return std::unexpected(std::string("No such file or directory"));
    }
    std::ifstream f(p, std::ios::binary);
    if (!f) {
        return std::unexpected(std::string("cannot open the file"));
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    if (f.bad()) {
        return std::unexpected(std::string("read error"));
    }
    return ss.str();
}

std::string conv_text(const pibridge::eyemath::FrameConv& c) {
    return std::format("flip x/y/z {}/{}/{}, swap {}, offset ({}, {}, {}) mm", int(c.flip_x), int(c.flip_y),
                       int(c.flip_z), int(c.swap_eyes), c.offset_mm[0], c.offset_mm[1], c.offset_mm[2]);
}

void sleep_stoppable(Duration d, const std::stop_token& stop) {
    const auto end = Clock::now() + d;
    while (!stop.stop_requested()) {
        const auto now = Clock::now();
        if (now >= end) {
            break;
        }
        std::this_thread::sleep_for(std::min<Duration>(end - now, 50ms));
    }
}

double secs(Duration d) { return std::chrono::duration<double>(d).count(); }

} // namespace

// ------------------------------------------------------------------------------------------------ Config

Config Config::from_env(const LogFn& log_in) {
    const LogFn log = log_in ? log_in : LogFn(stderr_log);
    Config c;
    c.conv = pibridge::eyemath::FrameConv::from_env();
    c.with_cal = env_on("EYE_CAL");
    c.skip_auth = env_on("EYE_SKIP_AUTH");
    c.usb_reset = !env_off("EYE_USB_RESET");
    c.power_on = !env_off("EYE_POWER_ON");
    if (const auto v = env("EYE_LATENCY_MS")) {
        const auto t = trim(*v);
        double ms = 0;
        const auto r = std::from_chars(t.data(), t.data() + t.size(), ms);
        if (r.ec == std::errc{} && r.ptr == t.data() + t.size()) {
            c.latency_ns = static_cast<std::int64_t>(ms * 1e6);
        }
    }
    if (const auto v = env("EYE_USB")) {
        c.usb_device = fs::path(*v);
    }
    if (const auto p = env("EYE_FRAMES")) {
        bool missing = false;
        if (auto t = read_file(*p, missing)) {
            c.frames = std::make_shared<const std::string>(std::move(*t));
        } else {
            log(std::format("{} {}: {}", pibridge::env_name("EYE_FRAMES"), *p, t.error()));
        }
    }
    if (const auto p = env("EYE_PROFILE")) {
        bool missing = false;
        if (auto t = read_file(*p, missing)) {
            Bytes b(t->begin(), t->end());
            if (auto ok = frames::check_profile(b)) {
                c.profile = std::make_shared<const Bytes>(std::move(b));
            } else {
                log(std::format("{} {}: {}", pibridge::env_name("EYE_PROFILE"), *p, ok.error()));
            }
        } else if (!missing) {
            log(std::format("{} {}: {}", pibridge::env_name("EYE_PROFILE"), *p, t.error()));
        }
    }
    if (auto sel = pibridge::eyemath::Calibration::from_env(); !sel) {
        log("per-eye calibration not loaded (raw gaze): " + sel.error().message);
    } else if (*sel) {
        const auto& lc = **sel;
        const auto env_conv = c.conv;
        c.conv = lc.cal.conv;
        std::string extra;
        if (!(env_conv == lc.cal.conv)) {
            extra = std::format("; using its frame conversion ({}) instead of the PIBRIDGE_EYE_FLIP_* environment ({})",
                                conv_text(lc.cal.conv), conv_text(env_conv));
        }
        log(std::format("per-eye calibration {} ({} / {} model, filter {}){}", lc.path.string(),
                        lc.cal.eyes[0].map.model, lc.cal.eyes[1].map.model,
                        lc.cal.filter.enabled ? "one-euro" : "off", extra));
        c.calibration = lc.cal;
        c.calibration_path = lc.path;
    }
    if (!c.calibration) {
        if (auto corr = pibridge::eyemath::Correction::from_env(); !corr) {
            log("gaze correction not loaded: " + corr.error().message);
        } else if (*corr) {
            std::string m = std::format("gaze correction active ({} model)", (*corr)->model);
            if (const auto w = (*corr)->flip_warning(c.conv)) {
                m += "\npibridge-eye: warning: " + *w;
            }
            log(m);
            c.correction = **corr;
        }
    }
    if (const auto f = env("EYE_FILTER")) {
        const auto t = trim(*f);
        if (t == "none" || t == "off" || t == "0") {
            pibridge::eyemath::FilterParams p;
            p.enabled = false;
            c.filter = p;
        } else if (t == "one-euro" || t == "on" || t == "1") {
            pibridge::eyemath::FilterParams p = c.calibration ? c.calibration->filter : pibridge::eyemath::FilterParams{};
            p.enabled = true;
            c.filter = p;
        }
    }
    return c;
}

// ------------------------------------------------------------------------------------------------ Pipeline

Pipeline::Pipeline(const Config& cfg, Sink sink)
    : now(pibridge::eyemath::monotonic_ns), dec_(session::in_decoder()), sink_(std::move(sink)) {
    clock_.latency_ns = cfg.latency_ns;
    if (cfg.filter) {
        filter_ = *cfg.filter;
    } else if (cfg.calibration) {
        filter_ = cfg.calibration->filter;
    } else {
        filter_ = pibridge::eyemath::FilterParams{};
        filter_.enabled = false;
    }
    conv_ = cfg.calibration ? cfg.calibration->conv : cfg.conv;
    if (!cfg.calibration) {
        correction_ = cfg.correction;
    }
    calibration_ = cfg.calibration;
}

void Pipeline::reset_link() {
    dec_ = session::in_decoder();
    clock_.reset();
    one_euro_.reset();
}

std::vector<ttp::Message> Pipeline::push(ByteView bytes) {
    const std::int64_t arrival = now();
    std::vector<ttp::Message> rest;
    for (auto& r : dec_.push(bytes)) {
        if (!r) {
            framing_errors++;
            continue;
        }
        ttp::Message& m = *r;
        if (m.header.kind != ttp::Kind::Notification || m.header.id != ttp::STREAM_GAZE) {
            rest.push_back(std::move(m));
            continue;
        }
        const auto g = pibridge::eyemath::parse_payload(m.payload);
        if (!g) {
            parse_errors++;
            continue;
        }
        const std::int64_t host = clock_.map(g->timestamp_us, arrival);
        pibridge::eyemath::Sample s = pibridge::eyemath::convert(*g, conv_, samples, host, arrival);
        if (calibration_) {
            std::array<pibridge::eyemath::Tangent, 2> extra{};
            if (drift_) {
                std::lock_guard l(drift_->mu);
                extra = drift_->drift.offset;
            }
            calibration_->apply_sample_with(s, combine_, extra);
        } else if (correction_) {
            correction_->apply_sample(s);
        }
        pibridge::eyemath::filter_sample(filter_, one_euro_, s);
        samples++;
        if (sink_) {
            sink_(s);
        }
    }
    return rest;
}

// ------------------------------------------------------------------------------------------------ Recovery

bool is_silent(Failure f) { return f == Failure::NoResponse || f == Failure::Stall; }

bool Recovery::after_link(std::optional<Failure> failure, bool streamed, Clock::time_point now, const Config& cfg) {
    // a link that streamed proves the chip was fine, unless it ended because the stream stopped
    if (streamed && failure != Failure::Stall) {
        silent_ = 0;
    }
    if (failure && is_silent(*failure)) {
        silent_++;
    } else {
        silent_ = 0;
        return false;
    }
    const bool spaced = !last_reset_ || now - *last_reset_ >= cfg.reset_min_interval;
    if (cfg.usb_reset && silent_ >= std::max<std::uint32_t>(cfg.reset_after, 1) && spaced) {
        silent_ = 0;
        last_reset_ = now;
        resets++;
        return true;
    }
    return false;
}

// ------------------------------------------------------------------------------------------------ Device

std::expected<std::string, std::string> Device::reset(const Config&) {
    return std::unexpected(std::string("USB reset not supported by this device"));
}

std::optional<std::string> Device::absent(const Config&, Clock::time_point) { return std::nullopt; }

std::optional<std::string> Device::silent(const Config&, Clock::time_point) { return std::nullopt; }

RealDevice::RealDevice(std::shared_ptr<power::HeadsetCommands> hid, fs::path usb_root, fs::path dev_usb_root)
    : hid_(std::move(hid)), usb_root_(std::move(usb_root)), dev_usb_root_(std::move(dev_usb_root)) {}

std::expected<fs::path, std::string> RealDevice::chip_node(const Config& cfg) const {
    if (cfg.usb_device) {
        return *cfg.usb_device;
    }
    for (const auto& d : usbfs::scan_sysfs(usb_root_, dev_usb_root_)) {
        if (d.vid != replay::EYE_VID || d.pid != replay::EYE_PID) {
            continue;
        }
        if (const auto drv = d.interface_driver()) {
            return std::unexpected(
                std::format("kernel driver '{}' is bound to the EyeChip interface; not detaching it", *drv));
        }
        return d.devnode;
    }
    return std::unexpected(std::string("no EyeChip (2104:0561) on the USB bus"));
}

std::expected<std::unique_ptr<Transport>, std::string> RealDevice::open(const Config& cfg, std::stop_token) {
    const auto node = chip_node(cfg);
    if (!node) {
        return std::unexpected(node.error());
    }
    auto dev = usbfs::UsbFs::open(*node);
    if (!dev) {
        return std::unexpected(std::format("open {}: {}", node->string(), dev.error().to_string()));
    }
    if (auto c = dev->claim_interface(0); !c) {
        return std::unexpected(std::format("claim interface 0 of {}: {}", node->string(), c.error().to_string()));
    }
    return std::make_unique<usbfs::UsbFs>(std::move(*dev));
}

std::expected<std::string, std::string> RealDevice::reset(const Config& cfg) {
    const auto node = chip_node(cfg);
    if (!node) {
        return std::unexpected(node.error());
    }
    if (auto r = usbfs::reset_node(*node); !r) {
        return std::unexpected(std::format("{}: {}", node->string(), r.error().to_string()));
    }
    return std::format("chip silent: host-side USB port reset of 2104:0561 ({}); it re-enumerates like a replug",
                       node->string());
}

std::optional<std::string> RealDevice::silent(const Config& cfg, Clock::time_point now) {
    if (!cfg.power_on || cfg.usb_device || !hid_) {
        return std::nullopt;
    }
    if (silent_enable_at_ && now - *silent_enable_at_ < SILENT_ENABLE_INTERVAL) {
        return std::nullopt;
    }
    const auto h = power::headset(usb_root_);
    if (!h) {
        return std::nullopt;
    }
    silent_enable_at_ = now;
    if (auto r = power::send_enable(*hid_); !r) {
        return "eye module enable failed: " + r.error().to_string();
    }
    return std::format("chip silent (headset {} devnum {}): sent the captured eye-module enable 58 05 3c 05 00 01 "
                       "again (the module goes off when the headset is taken off)",
                       h->name, h->devnum);
}

std::optional<std::string> RealDevice::absent(const Config& cfg, Clock::time_point now) {
    if (!cfg.power_on || cfg.usb_device || !hid_) {
        return std::nullopt;
    }
    const auto g = gate_.step(now, power::headset(usb_root_), power::eye_chip_present(usb_root_));
    if (g.kind != power::Gate::Kind::Send) {
        return std::nullopt;
    }
    if (auto r = power::send_enable(*hid_); !r) {
        return "eye module enable failed: " + r.error().to_string();
    }
    return std::format("eye module off (headset {} devnum {} present, no 2104:0561 for 5 s): sent the captured "
                       "eye-module enable 58 05 3c 05 00 01 (once for this headset enumeration)",
                       g.target.name, g.target.devnum);
}

// ------------------------------------------------------------------------------------------------ the link

namespace {

LinkError link_error(Failure k, std::string msg) { return LinkError{k, std::move(msg)}; }

std::expected<void, LinkError> send_sequence(Transport& t, const Config& cfg, const std::vector<const frames::FStep*>& sel,
                                             const frames::Guard& guard, const std::stop_token& stop, Pipeline& pipe,
                                             bool& subscribed, const LogFn& log) {
    std::optional<double> prev_t;
    auto prev_sent = Clock::now();
    for (const frames::FStep* st : sel) {
        if (stop.stop_requested()) {
            return {};
        }
        if (const auto* c = std::get_if<frames::Control>(&st->action)) {
            if (auto r = guard.control(t, c->setup, c->data, cfg.timeout); !r) {
                return std::unexpected(link_error(Failure::Io, std::format("step {}: {}", st->n, r.error().to_string())));
            }
            continue;
        }
        const auto* req = std::get_if<frames::Ttp>(&st->action);
        if (req == nullptr) {
            continue; // enumeration: never selected
        }
        if (cfg.pace && prev_t && st->t) {
            const double gap = std::clamp(*st->t - *prev_t, 0.0, 1.0);
            const auto want = std::chrono::duration_cast<Duration>(std::chrono::duration<double>(gap));
            const auto done = Clock::now() - prev_sent;
            if (want > done) {
                sleep_stoppable(want - done, stop);
            }
        }
        prev_sent = Clock::now();
        prev_t = st->t;
        for (const auto& f : req->frames) {
            if (auto r = guard.bulk_out(t, f, cfg.timeout); !r) {
                return std::unexpected(link_error(
                    Failure::Io, std::format("step {} ({}): {}", st->n, ttp::opcode_name(req->op), r.error().to_string())));
            }
        }
        if (req->op == session::OP_SUBSCRIBE) {
            subscribed = true;
        }
        const auto sent = Clock::now();
        const Duration wait = req->op == frames::CAL_OP ? std::max<Duration>(cfg.response_wait, 10s) : cfg.response_wait;
        std::optional<ttp::Message> resp;
        while (!resp) {
            if (stop.stop_requested()) {
                return {};
            }
            if (Clock::now() - sent > wait) {
                return std::unexpected(link_error(
                    Failure::NoResponse, std::format("no response to seq {} op {:#x} ({}) within {:.1f} s", req->seq,
                                                     req->op, ttp::opcode_name(req->op), secs(wait))));
            }
            auto b = t.bulk_in(0x83, session::IN_BUFFER, 200ms);
            if (!b) {
                return std::unexpected(link_error(Failure::Io, "IN read: " + b.error().to_string()));
            }
            if (!*b) {
                continue;
            }
            for (auto& m : pipe.push(**b)) {
                if (m.header.kind == ttp::Kind::Response && m.header.seq == req->seq) {
                    resp = std::move(m);
                    break;
                }
            }
        }
        if (resp->header.status != 1 && !frames::is_auth(req->op)) {
            return std::unexpected(link_error(
                Failure::Status, std::format("op {:#x} ({}) answered with status {} (captured: 1)", req->op,
                                             ttp::opcode_name(req->op), resp->header.status)));
        }
        if (resp->header.status != 1 && log) {
            log(std::format("auth op {:#x} answered with status {} (expected, not needed)", req->op,
                            resp->header.status));
        }
    }
    return {};
}

std::expected<void, LinkError> read_loop(Transport& t, const Config& cfg, const std::stop_token& stop, Pipeline& pipe,
                                         const std::function<void(State)>& state) {
    auto last_gaze = Clock::now();
    auto last_count = pipe.samples;
    bool announced = false;
    // the first gaze can take a while after the calibration upload
    const Duration first_wait = std::max<Duration>(cfg.stall, 10s);
    while (!stop.stop_requested()) {
        auto b = t.bulk_in(0x83, session::IN_BUFFER, 100ms);
        if (!b) {
            return std::unexpected(link_error(Failure::Io, "IN read: " + b.error().to_string()));
        }
        if (*b) {
            pipe.push(**b);
        }
        if (pipe.samples != last_count) {
            last_count = pipe.samples;
            last_gaze = Clock::now();
            if (!announced) {
                announced = true;
                if (state) {
                    state(State::Streaming);
                }
            }
        } else if (Clock::now() - last_gaze > (announced ? cfg.stall : first_wait)) {
            return std::unexpected(
                link_error(Failure::Stall, std::format("no gaze for {:.1f} s", secs(Clock::now() - last_gaze))));
        }
    }
    return {};
}

} // namespace

std::expected<void, LinkError> run_link(Transport& t, const Config& cfg, std::stop_token stop, Pipeline& pipe,
                                        const std::function<void(State)>& state, const LogFn& log) {
    const std::string* own = cfg.frames.get();
    auto steps = frames::parse(own != nullptr ? std::string_view(*own) : frames::list().first);
    if (!steps) {
        return std::unexpected(link_error(Failure::Other, steps.error()));
    }
    if (cfg.profile) {
        if (auto r = frames::with_profile(*steps, *cfg.profile); !r) {
            return std::unexpected(link_error(Failure::Other, r.error()));
        }
        if (log) {
            log(std::format("eye profile: uploading the user's profile ({} bytes)", cfg.profile->size()));
        }
    }
    frames::Skips skips;
    skips.auth = cfg.skip_auth;
    skips.cal = !cfg.with_cal && !cfg.profile;
    skips.vendor_write = cfg.skip_vendor_write;
    const auto sel = frames::select(*steps, skips);
    auto guard = frames::Guard::make_with(*steps, own == nullptr && !cfg.profile);
    if (!guard) {
        return std::unexpected(link_error(Failure::Other, guard.error()));
    }
    guard->allow_frame(session::CAPTURED_UNSUBSCRIBE_GAZE);
    pipe.reset_link();
    if (state) {
        state(State::Connecting);
    }

    // Whatever the chip has pending; gaze already flowing means a previous session is still subscribed: adopt it.
    const auto before = pipe.samples;
    const auto until = Clock::now() + 300ms;
    while (Clock::now() < until && !stop.stop_requested()) {
        auto b = t.bulk_in(0x83, session::IN_BUFFER, 100ms);
        if (!b) {
            return std::unexpected(link_error(Failure::Io, "initial IN read: " + b.error().to_string()));
        }
        if (*b) {
            pipe.push(**b);
        }
    }
    const bool adopted = pipe.samples > before;
    bool subscribed = false;
    std::expected<void, LinkError> result;
    if (adopted) {
        if (log) {
            log("gaze already streaming (earlier session); not sending the init, will not unsubscribe");
        }
    } else {
        result = send_sequence(t, cfg, sel, *guard, stop, pipe, subscribed, log);
    }
    if (result) {
        result = read_loop(t, cfg, stop, pipe, state);
    }
    if (subscribed) {
        // best effort; on a dead device bulk_out fails at once
        if (guard->bulk_out(t, session::CAPTURED_UNSUBSCRIBE_GAZE, 500ms)) {
            const auto end = Clock::now() + 300ms;
            while (Clock::now() < end) {
                auto b = t.bulk_in(0x83, session::IN_BUFFER, 50ms);
                if (!b) {
                    break;
                }
                if (*b) {
                    pipe.push(**b);
                }
            }
            if (log) {
                log("unsubscribed (0x4ce)");
            }
        }
    }
    return result;
}

// ------------------------------------------------------------------------------------------------ Tracker

Tracker::Tracker(Config cfg, LogFn log)
    : cfg_(std::move(cfg)), log_(log ? std::move(log) : LogFn(stderr_log)), cap_(std::max<std::size_t>(cfg_.history, 2)) {}

std::unique_ptr<Tracker> Tracker::start(Config cfg, std::unique_ptr<Device> dev, LogFn log) {
    std::unique_ptr<Tracker> tr(new Tracker(std::move(cfg), std::move(log)));
    Tracker* self = tr.get();
    tr->thread_ = std::jthread([self, d = std::move(dev)](std::stop_token st) mutable { self->run(st, std::move(d)); });
    return tr;
}

std::unique_ptr<Tracker> Tracker::start_with(Config cfg, FnDevice::OpenFn open, LogFn log) {
    return start(std::move(cfg), std::make_unique<FnDevice>(std::move(open)), std::move(log));
}

Tracker::~Tracker() { stop(); }

void Tracker::stop() {
    if (thread_.joinable()) {
        thread_.request_stop();
        thread_.join();
    }
}

void Tracker::run(std::stop_token stop, std::unique_ptr<Device> dev) {
    Pipeline pipe(cfg_, [this](const pibridge::eyemath::Sample& s) {
        samples_.fetch_add(1, std::memory_order_relaxed);
        std::lock_guard l(hist_mu_);
        if (hist_.size() >= cap_) {
            hist_.pop_front();
        }
        hist_.push_back(s);
    });
    pipe.set_drift(drift_);
    Duration backoff = cfg_.retry;
    const auto say = [this](std::string_view m) {
        {
            std::lock_guard l(msg_mu_);
            if (m == last_msg_) {
                return;
            }
            last_msg_ = std::string(m);
        }
        log_(m);
    };
    const auto grow = [](Duration b) { return std::min<Duration>(b * 2, 10s); };
    Recovery recovery;
    while (!stop.stop_requested()) {
        set(State::Searching);
        auto t = dev->open(cfg_, stop);
        if (!t) {
            say(t.error());
            if (auto m = dev->absent(cfg_, Clock::now())) {
                say(*m);
                backoff = cfg_.retry;
            }
            sleep_stoppable(backoff, stop);
            backoff = grow(backoff);
            continue;
        }
        bool streamed = false;
        auto r = run_link(
            **t, cfg_, stop, pipe,
            [&](State s) {
                streamed |= s == State::Streaming;
                set(s);
            },
            say);
        t->reset(); // releases the interface
        if (streamed) {
            backoff = cfg_.retry;
        }
        const std::optional<Failure> failure = r ? std::nullopt : std::optional<Failure>(r.error().kind);
        if (failure && is_silent(*failure)) {
            if (auto m = dev->silent(cfg_, Clock::now())) {
                say(*m);
            }
        }
        const bool reset = recovery.after_link(failure, streamed, Clock::now(), cfg_);
        if (!r) {
            say("link lost: " + r.error().msg + "; retrying");
            reconnects_.fetch_add(1, std::memory_order_relaxed);
            set(State::Reconnecting);
            if (reset && !stop.stop_requested()) {
                resets_.fetch_add(1, std::memory_order_relaxed);
                if (auto m = dev->reset(cfg_)) {
                    say(*m);
                } else {
                    say("USB reset failed: " + m.error());
                }
                // re-enumeration takes about a second; then try at once
                sleep_stoppable(1500ms, stop);
                backoff = cfg_.retry;
            } else {
                sleep_stoppable(backoff, stop);
                backoff = grow(backoff);
            }
        }
    }
    set(State::Stopped);
}

TrackerStats Tracker::stats() const {
    return TrackerStats{samples_.load(std::memory_order_relaxed), reconnects_.load(std::memory_order_relaxed),
                        resets_.load(std::memory_order_relaxed)};
}

std::optional<pibridge::eyemath::Sample> Tracker::latest() const {
    std::lock_guard l(hist_mu_);
    if (hist_.empty()) {
        return std::nullopt;
    }
    return hist_.back();
}

std::vector<pibridge::eyemath::Sample> Tracker::samples_since(std::optional<std::uint64_t> after) const {
    std::lock_guard l(hist_mu_);
    std::vector<pibridge::eyemath::Sample> out;
    for (const auto& s : hist_) {
        if (!after || s.seq > *after) {
            out.push_back(s);
        }
    }
    return out;
}

std::optional<pibridge::eyemath::Sample> Tracker::sample_at(std::int64_t t_ns) const {
    std::vector<pibridge::eyemath::Sample> v;
    {
        std::lock_guard l(hist_mu_);
        v.assign(hist_.begin(), hist_.end());
    }
    return pibridge::eyemath::sample_at(v, t_ns);
}

std::expected<std::size_t, std::string> Tracker::observe_target(std::int64_t at_ns, const pibridge::eyemath::Vec3& dir,
                                                                const pibridge::eyemath::DriftParams& p) {
    if (!cfg_.calibration) {
        return std::unexpected(std::string("no per-eye calibration loaded: drift correction needs one"));
    }
    const auto& cal = *cfg_.calibration;
    const auto s = sample_at(at_ns);
    if (!s) {
        return std::unexpected(std::string("no gaze sample"));
    }
    pibridge::eyemath::Vec3 mid{};
    pibridge::eyemath::Vec3 target{};
    for (std::size_t k = 0; k < 3; k++) {
        mid[k] = 0.5 * (cal.eyes[0].origin_m[k] + cal.eyes[1].origin_m[k]);
        target[k] = mid[k] + cal.combine.depth_m * dir[k];
    }
    std::lock_guard l(drift_->mu);
    auto& d = drift_->drift;
    std::size_t used = 0;
    for (std::size_t i = 0; i < 2; i++) {
        const auto& e = s->eyes[i];
        if (!(e.origin_valid && e.direction_valid)) {
            continue;
        }
        const auto& o = cal.eyes[i].origin_m;
        const pibridge::eyemath::Vec3 want{target[0] - o[0], target[1] - o[1], target[2] - o[2]};
        const pibridge::eyemath::Vec3 got{e.direction[0], e.direction[1], e.direction[2]};
        const auto w = pibridge::eyemath::tangent_of(want);
        const auto g = pibridge::eyemath::tangent_of(got);
        if (w && g) {
            // the published direction already holds the running offset: observe without it
            const auto cur = d.offset[i];
            if (d.observe(p, i, pibridge::eyemath::Tangent{g->u - cur.u, g->v - cur.v}, *w)) {
                used++;
            }
        }
    }
    return used;
}

pibridge::eyemath::Drift Tracker::drift() const {
    std::lock_guard l(drift_->mu);
    return drift_->drift;
}

std::string Tracker::last_message() const {
    std::lock_guard l(msg_mu_);
    return last_msg_;
}

// ------------------------------------------------------------------------------------------------ helpers

shm::Sample to_shm(const pibridge::eyemath::Sample& s) {
    shm::Sample o;
    o.seq = s.seq;
    o.host_ts_ns = s.host_ts_ns;
    for (std::size_t i = 0; i < 2; i++) {
        o.eyes[i].origin = s.eyes[i].origin;
        o.eyes[i].direction = s.eyes[i].direction;
        o.eyes[i].origin_valid = s.eyes[i].origin_valid;
        o.eyes[i].direction_valid = s.eyes[i].direction_valid;
    }
    if (s.gaze) {
        o.gaze = shm::CombinedRay{s.gaze->origin, s.gaze->direction};
    }
    o.ipd_mm = s.ipd_mm;
    return o;
}

pibridge::eyemath::GazeSample fake_sample(std::uint32_t n, std::int64_t timestamp_us) {
    const double ph = static_cast<double>(n) * 0.02;
    pibridge::eyemath::Vec3 dir{0.2 * std::sin(ph), 0.1 * std::cos(ph), 1.0};
    const double norm = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
    for (auto& c : dir) {
        c /= norm;
    }
    pibridge::eyemath::Eye a;
    a.origin_mm = {31.0, -2.0, -22.0};
    a.origin_valid = true;
    a.direction = dir;
    a.direction_valid = true;
    a.pupil_diameter_mm = 3.25;
    a.pupil_pos = {0.5, 0.5};
    a.pupil_pos_valid = true;
    a.flags = {1, 0};
    pibridge::eyemath::Eye b;
    b.flags = {1, 0};
    pibridge::eyemath::GazeSample g;
    g.timestamp_us = timestamp_us;
    g.frame_counter = 1000 + n;
    g.eyes = {a, b};
    g.frame.field_14 = 0xfffff;
    g.frame.field_35 = 1;
    return g;
}

Bytes fake_payload(std::uint32_t n, std::int64_t timestamp_us) {
    return pibridge::eyemath::encode_payload(fake_sample(n, timestamp_us));
}

} // namespace pibridge::eye::live
