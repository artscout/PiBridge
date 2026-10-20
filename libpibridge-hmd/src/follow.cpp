// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-tools/src/follow.rs (MIT OR Apache-2.0).
#include "pibridge/hmd/follow.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>

namespace pibridge::hmd {
namespace fs = std::filesystem;

bool process_running(std::string_view name) { return process_running_in("/proc", name); }

bool process_running_in(const fs::path& proc_root, std::string_view name) {
    // comm holds at most 15 bytes; Rust takes 15 characters (the same for the ASCII names used here)
    const std::string want(name.substr(0, std::min<std::size_t>(name.size(), 15)));
    std::error_code ec;
    for (fs::directory_iterator it(proc_root, ec), end; !ec && it != end; it.increment(ec)) {
        const std::string n = it->path().filename().string();
        if (n.empty() || !std::all_of(n.begin(), n.end(), [](char c) { return c >= '0' && c <= '9'; })) {
            continue;
        }
        std::ifstream f(it->path() / "comm");
        if (!f) {
            continue;
        }
        std::stringstream ss;
        ss << f.rdbuf();
        std::string c = ss.str();
        while (!c.empty() && (c.back() == '\n' || c.back() == ' ' || c.back() == '\t' || c.back() == '\r')) {
            c.pop_back();
        }
        if (c == want) {
            return true;
        }
    }
    return false;
}

std::optional<Follow::Change> Follow::step(Clock::time_point now, bool running) {
    if (running) {
        gone_since_.reset();
        if (!active_) {
            active_ = true;
            return Change::Active;
        }
        return std::nullopt;
    }
    if (!active_) {
        return std::nullopt;
    }
    if (!gone_since_) {
        gone_since_ = now;
    }
    if (now - *gone_since_ >= IDLE_AFTER) {
        active_ = false;
        gone_since_.reset();
        return Change::Idle;
    }
    return std::nullopt;
}

} // namespace pibridge::hmd
