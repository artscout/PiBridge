// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/resilient.rs (MIT OR Apache-2.0).
#include "pibridge/cam/resilient.h"

#include "pibridge/cam/rustfmt.h"

#include <cstdio>
#include <format>
#include <thread>

namespace pibridge::cam {

namespace {

double secs(Clock::duration d) {
    return std::chrono::duration<double>(d).count();
}

Clock::duration from_secs_f64(double s) {
    // Duration::from_secs_f64 (a value too big for a Duration would panic in Rust: cap it at a year)
    if (s > 3.15e7) {
        s = 3.15e7;
    }
    return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(s));
}

Clock::duration sat_since(Clock::time_point later, Clock::time_point earlier) {
    return later > earlier ? later - earlier : Clock::duration::zero();
}

} // namespace

Policy Policy::camera() {
    double stall = 5.0;
    if (auto v = env_utf8("PIMAX_CAM_STALL_S")) {
        double x = 0;
        if (rfmt::parse_f64(rfmt::trim(*v), x)) {
            stall = x;
        }
    }
    Policy p;
    if (stall > 0.0) {
        p.stall = from_secs_f64(stall);
    }
    p.backoff_min = std::chrono::milliseconds(250);
    p.backoff_max = std::chrono::seconds(2);
    p.log_every = std::chrono::seconds(30);
    return p;
}

Policy Policy::broker() {
    Policy p;
    p.backoff_min = std::chrono::milliseconds(250);
    p.backoff_max = std::chrono::seconds(1);
    p.log_every = std::chrono::seconds(30);
    return p;
}

Clock::duration Policy::backoff(std::uint32_t failures) const {
    const std::uint32_t shift = std::min<std::uint32_t>(failures > 0 ? failures - 1 : 0, 16);
    const auto d = backoff_min * (1u << shift);
    return std::min(d, backoff_max);
}

bool reconnect_enabled() {
    const auto v = env_utf8("PIMAX_CAM_RECONNECT");
    if (!v) {
        return true;
    }
    const auto t = rfmt::trim(*v);
    return !(t == "0" || t == "off" || t == "no" || t == "false");
}

Resilient::Resilient(std::unique_ptr<Source> first, Opener open, Policy policy, std::string label)
    : open_(std::move(open)), policy_(policy), label_(std::move(label)) {
    const auto now = Clock::now();
    desc_ = first->describe();
    inner_ = std::move(first);
    health_.up = true;
    health_.opens = 1;
    last_frame_ = now;
    next_try_ = now;
}

Health Resilient::health() const {
    Health h = health_;
    h.up = inner_ != nullptr;
    return h;
}

void Resilient::say(Clock::time_point now, const std::string& msg) {
    last_log_ = now;
    if (log_) {
        std::fprintf(stderr, "pimax-cam: %s\n", msg.c_str());
    }
}

void Resilient::lose(Clock::time_point now, std::string why, bool stall) {
    if (inner_) {
        corrupt_base_ += inner_->corrupt_frames();
        inner_.reset();
    }
    health_.losses++;
    if (stall) {
        health_.stalls++;
    }
    if (!framed_) {
        // reopened but never delivered: back off like a failed open
        if (health_.failures != UINT32_MAX) {
            health_.failures++;
        }
    }
    const bool first = !down_since_.has_value();
    if (!down_since_) {
        down_since_ = now;
    }
    next_try_ = now + policy_.backoff(std::max<std::uint32_t>(health_.failures, 1));
    const bool quiet = last_log_ && sat_since(now, *last_log_) < policy_.log_every;
    if (first || !quiet) {
        say(now, std::format("{} lost ({}); reopening", label_, why));
    }
    last_err_ = std::move(why);
}

void Resilient::try_open(Clock::time_point now) {
    auto s = open_();
    if (s) {
        const double down = down_since_ ? secs(sat_since(now, *down_since_)) : 0.0;
        desc_ = (*s)->describe();
        inner_ = std::move(*s);
        health_.opens++;
        framed_ = false;
        last_frame_ = now;
        say(now, std::format("{} reopened after {} s ({} failed tries): {}", label_, rfmt::fixed(down, 1),
                             health_.failures, desc_));
        return;
    }
    if (health_.failures != UINT32_MAX) {
        health_.failures++;
    }
    next_try_ = now + policy_.backoff(health_.failures);
    const bool due = !last_log_ || sat_since(now, *last_log_) >= policy_.log_every;
    if (due) {
        const double down = down_since_ ? secs(sat_since(now, *down_since_)) : 0.0;
        say(now, std::format("{} still gone after {} s ({} tries): {}", label_, rfmt::fixed(down, 0),
                             health_.failures, s.error().text()));
    }
    last_err_ = s.error().text();
}

Result<bool> Resilient::advance(int timeout_ms) {
    std::optional<Clock::time_point> deadline;
    if (timeout_ms >= 0) {
        deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
    }
    for (;;) {
        auto now = Clock::now();
        if (!inner_ && now >= next_try_) {
            try_open(now);
        }
        if (!inner_) {
            const auto wake = deadline ? std::min(*deadline, next_try_) : next_try_;
            std::this_thread::sleep_for(sat_since(wake, now));
            if (deadline && Clock::now() >= *deadline) {
                return false;
            }
            continue;
        }
        // waiting forever: still come back now and then so a stall can be noticed
        int left = 500;
        if (deadline) {
            const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(sat_since(*deadline, now)).count();
            left = static_cast<int>(std::min<long long>(ms, INT32_MAX));
        }
        Source& src = *inner_;
        auto r = src.advance(left);
        if (r && *r) {
            last_frame_ = Clock::now();
            if (!framed_) {
                framed_ = true;
                health_.failures = 0;
                down_since_.reset();
            }
            return true;
        }
        if (r) {
            if (src.finished()) {
                return false;
            }
            now = Clock::now();
            if (policy_.stall && !src.is_replay() && sat_since(now, last_frame_) >= *policy_.stall) {
                lose(now, std::format("no frame for {} s", rfmt::fixed(secs(*policy_.stall), 1)), true);
            }
            if (deadline) {
                return false;
            }
            continue;
        }
        if (src.is_replay()) {
            return fail(r.error()); // the end of a replay broker: not ours to hide
        }
        lose(Clock::now(), r.error().text(), false);
        if (deadline && Clock::now() >= *deadline) {
            return false;
        }
    }
}

std::optional<RawFrame> Resilient::current() const {
    return inner_ ? inner_->current() : std::nullopt;
}

std::string Resilient::describe() const {
    if (inner_) {
        return inner_->describe();
    }
    return std::format("{} (reopening: {})", desc_, last_err_);
}

bool Resilient::finished() const {
    return inner_ && inner_->finished();
}

std::uint64_t Resilient::corrupt_frames() const {
    return corrupt_base_ + (inner_ ? inner_->corrupt_frames() : 0);
}

bool Resilient::is_replay() const {
    return inner_ && inner_->is_replay();
}

std::uint64_t Resilient::generation() const {
    const std::uint64_t inner = inner_ ? inner_->generation() : 0;
    const std::uint64_t opens = health_.opens > 0 ? health_.opens - 1 : 0;
    return (opens << 32) | (inner & 0xffff'ffffull);
}

bool Resilient::up() const {
    return inner_ && inner_->up();
}

} // namespace pibridge::cam
