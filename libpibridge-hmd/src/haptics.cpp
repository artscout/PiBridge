// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax/src/haptics.rs (MIT OR Apache-2.0).
#include "pibridge/hmd/haptics.h"

#include <cstddef>

#include <unistd.h>

#include <sys/un.h>

#include <sys/socket.h>

#include "pibridge/hmd/proto.h"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>

namespace pibridge::hmd::haptics {

namespace {

std::optional<std::size_t> idx(std::uint8_t slot) {
    if (slot == SLOT_LEFT) {
        return 0;
    }
    if (slot == SLOT_RIGHT) {
        return 1;
    }
    return std::nullopt;
}

// One UTF-8 scalar at `s[i]`: its code point and length; nullopt for a malformed sequence (the file is then not
// valid UTF-8, which Rust's read_to_string refuses).
std::optional<std::pair<char32_t, std::size_t>> utf8_at(std::string_view s, std::size_t i) {
    const auto b = static_cast<unsigned char>(s[i]);
    if (b < 0x80) {
        return std::pair{static_cast<char32_t>(b), std::size_t{1}};
    }
    std::size_t n = 0;
    char32_t cp = 0;
    char32_t min = 0;
    if ((b & 0xe0) == 0xc0) {
        n = 2, cp = b & 0x1f, min = 0x80;
    } else if ((b & 0xf0) == 0xe0) {
        n = 3, cp = b & 0x0f, min = 0x800;
    } else if ((b & 0xf8) == 0xf0) {
        n = 4, cp = b & 0x07, min = 0x10000;
    } else {
        return std::nullopt;
    }
    if (i + n > s.size()) {
        return std::nullopt;
    }
    for (std::size_t k = 1; k < n; k++) {
        const auto c = static_cast<unsigned char>(s[i + k]);
        if ((c & 0xc0) != 0x80) {
            return std::nullopt;
        }
        cp = (cp << 6) | (c & 0x3f);
    }
    if (cp < min || cp > 0x10ffff || (cp >= 0xd800 && cp <= 0xdfff)) {
        return std::nullopt;
    }
    return std::pair{cp, n};
}

// char::is_whitespace (Unicode White_Space).
bool is_ws(char32_t c) {
    return (c >= 0x09 && c <= 0x0d) || c == 0x20 || c == 0x85 || c == 0xa0 || c == 0x1680 ||
           (c >= 0x2000 && c <= 0x200a) || c == 0x2028 || c == 0x2029 || c == 0x202f || c == 0x205f || c == 0x3000;
}

// str::split_whitespace on valid UTF-8.
std::vector<std::string_view> split_ws(std::string_view s) {
    std::vector<std::string_view> out;
    std::size_t i = 0, start = std::string_view::npos;
    while (i < s.size()) {
        const auto u = utf8_at(s, i);
        const auto [cp, n] = u ? *u : std::pair{char32_t{0xfffd}, std::size_t{1}};
        if (is_ws(cp)) {
            if (start != std::string_view::npos) {
                out.push_back(s.substr(start, i - start));
                start = std::string_view::npos;
            }
        } else if (start == std::string_view::npos) {
            start = i;
        }
        i += n;
    }
    if (start != std::string_view::npos) {
        out.push_back(s.substr(start));
    }
    return out;
}

// `str::parse::<uN>()`: optional '+', then at least one ASCII digit, no overflow.
template <class T>
std::optional<T> parse_unsigned(std::string_view s) {
    if (!s.empty() && s[0] == '+') {
        s.remove_prefix(1);
    }
    if (s.empty()) {
        return std::nullopt;
    }
    std::uint64_t v = 0;
    for (const char c : s) {
        if (c < '0' || c > '9') {
            return std::nullopt;
        }
        v = v * 10 + static_cast<std::uint64_t>(c - '0');
        if (v > static_cast<std::uint64_t>(static_cast<T>(-1))) {
            return std::nullopt;
        }
    }
    return static_cast<T>(v);
}

bool valid_utf8(std::string_view s) {
    for (std::size_t i = 0; i < s.size();) {
        const auto u = utf8_at(s, i);
        if (!u) {
            return false;
        }
        i += u->second;
    }
    return true;
}

} // namespace

bool Limiter::offer(std::uint8_t slot, std::optional<Pulse> pulse) {
    const auto i = idx(slot);
    if (!i) {
        return false;
    }
    Slot& s = slots_[*i];
    if (s.pending && pulse) {
        replaced++;
    }
    s.pending = pulse;
    return true;
}

std::vector<Due> Limiter::take_due(Clock::time_point now) {
    std::vector<Due> out;
    const std::uint8_t ids[2] = {SLOT_LEFT, SLOT_RIGHT};
    for (std::size_t i = 0; i < 2; i++) {
        Slot& s = slots_[i];
        // saturating_duration_since: a send time after `now` counts as 0 elapsed
        const bool ready = !s.last_sent || (now > *s.last_sent ? now - *s.last_sent : Clock::duration::zero()) >= MIN_GAP;
        if (ready && s.pending) {
            const auto [amp, ms] = *s.pending;
            s.pending.reset();
            s.last_sent = now;
            sent++;
            out.emplace_back(ids[i], amp, ms);
        }
    }
    return out;
}

std::optional<Clock::time_point> Limiter::next_due() const {
    std::optional<Clock::time_point> best;
    for (const auto& s : slots_) {
        if (!s.pending) {
            continue;
        }
        const auto t = s.last_sent ? *s.last_sent + MIN_GAP : Clock::now();
        if (!best || t < *best) {
            best = t;
        }
    }
    return best;
}

void Limiter::clear() { slots_ = {}; }

std::filesystem::path request_path() {
    const char* x = std::getenv("XDG_RUNTIME_DIR");
    return std::filesystem::path(x != nullptr ? x : "/tmp") / REQUEST_FILE;
}

std::string format_request(std::uint8_t slot, std::uint8_t amp, std::uint16_t ms) {
    return std::to_string(slot) + " " + std::to_string(amp) + " " + std::to_string(ms) + "\n";
}

std::optional<Due> parse_request(std::string_view s) {
    const auto t = split_ws(s);
    if (t.size() < 3) {
        return std::nullopt;
    }
    const auto slot = parse_unsigned<std::uint8_t>(t[0]);
    const auto amp = parse_unsigned<std::uint8_t>(t[1]);
    const auto ms = parse_unsigned<std::uint16_t>(t[2]);
    if (!slot || !amp || !ms) {
        return std::nullopt;
    }
    if (!idx(*slot) || *amp < 1 || *amp > 100 || *ms < 1 || t.size() != 3) {
        return std::nullopt;
    }
    return Due{*slot, *amp, *ms};
}

std::expected<std::optional<Due>, std::string> take_request(const std::filesystem::path& path) {
    std::error_code ec;
    if (!std::filesystem::exists(path, ec)) {
        if (!ec || ec == std::errc::no_such_file_or_directory) {
            return std::optional<Due>{};
        }
        return std::unexpected(ec.message());
    }
    if (std::filesystem::is_directory(path, ec)) {
        return std::unexpected(std::string("Is a directory (os error 21)")); // read_to_string fails, nothing removed
    }
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        const int e = errno;
        if (e == ENOENT) {
            return std::optional<Due>{}; // taken by someone else in between
        }
        return std::unexpected(std::string(std::strerror(e)));
    }
    const std::string s((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (!valid_utf8(s)) {
        return std::unexpected(std::string("stream did not contain valid UTF-8")); // not removed, as in Rust
    }
    std::filesystem::remove(path, ec);
    if (auto r = parse_request(s)) {
        return std::optional<Due>(*r);
    }
    return std::unexpected("bad haptic request \"" + s + "\"");
}

std::string socket_name() {
    return "pibridge-haptic-" + std::to_string(::getuid());
}

Receiver::Receiver() {
    const int fd = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        return;
    }
    sockaddr_un a{};
    a.sun_family = AF_UNIX;
    const std::string n = socket_name();
    std::memcpy(a.sun_path + 1, n.data(), n.size()); // abstract: sun_path[0] = 0
    const auto len = static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + 1 + n.size());
    if (::bind(fd, reinterpret_cast<const sockaddr*>(&a), len) != 0) {
        ::close(fd);
        return;
    }
    fd_ = fd;
}

Receiver::~Receiver() {
    if (fd_ >= 0) {
        ::close(fd_);
    }
}

std::optional<Due> Receiver::next() {
    if (fd_ < 0) {
        return std::nullopt;
    }
    char buf[64];
    for (;;) {
        const ssize_t n = ::recv(fd_, buf, sizeof buf - 1, 0);
        if (n < 0) {
            return std::nullopt; // EAGAIN: nothing waits
        }
        if (auto d = parse_request(std::string_view(buf, static_cast<std::size_t>(n)))) {
            return d;
        }
    }
}

} // namespace pibridge::hmd::haptics
