// SPDX-License-Identifier: MIT
// Rust semantics for the room-anchor and room-map ports (see rustx.h).
#include "pibridge/anchor/rustx.h"

#include "pibridge/cam/rustfmt.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <limits>
#include <unistd.h>

namespace pibridge::anchor::rs {

std::vector<std::string_view> lines(std::string_view s) {
    std::vector<std::string_view> out;
    std::size_t i = 0;
    while (i < s.size()) {
        const std::size_t nl = s.find('\n', i);
        if (nl == std::string_view::npos) {
            out.push_back(s.substr(i));
            break;
        }
        std::string_view l = s.substr(i, nl - i);
        if (!l.empty() && l.back() == '\r') {
            l.remove_suffix(1);
        }
        out.push_back(l);
        i = nl + 1;
    }
    return out;
}

std::optional<std::pair<std::string_view, std::string_view>> split_once(std::string_view s, char c) {
    const std::size_t p = s.find(c);
    if (p == std::string_view::npos) {
        return std::nullopt;
    }
    return std::pair{s.substr(0, p), s.substr(p + 1)};
}

std::string_view trim_matches(std::string_view s, std::string_view set) {
    while (!s.empty() && set.find(s.front()) != std::string_view::npos) {
        s.remove_prefix(1);
    }
    while (!s.empty() && set.find(s.back()) != std::string_view::npos) {
        s.remove_suffix(1);
    }
    return s;
}

std::string_view trim_start_char(std::string_view s, char c) {
    while (!s.empty() && s.front() == c) {
        s.remove_prefix(1);
    }
    return s;
}

std::string_view trim_end_char(std::string_view s, char c) {
    while (!s.empty() && s.back() == c) {
        s.remove_suffix(1);
    }
    return s;
}

namespace {

/// [start, end) of the file name in `path`.
std::pair<std::size_t, std::size_t> name_span(const std::string& path) {
    std::size_t end = path.size();
    while (end > 0 && path[end - 1] == '/') {
        end--;
    }
    const std::size_t slash = end == 0 ? std::string::npos : path.rfind('/', end - 1);
    const std::size_t start = slash == std::string::npos ? 0 : slash + 1;
    return {start, end};
}

} // namespace

std::string file_name(const std::string& path) {
    const auto [a, b] = name_span(path);
    const std::string n = path.substr(a, b - a);
    if (n == "." || n == "..") {
        return "";
    }
    return n;
}

std::string with_file_name(const std::string& path, std::string_view name) {
    const auto [a, b] = name_span(path);
    if (b == 0) {
        return std::string(name);
    }
    return path.substr(0, a) + std::string(name);
}

std::string with_extension(const std::string& path, std::string_view ext) {
    const auto [a, b] = name_span(path);
    std::string name = path.substr(a, b - a);
    if (name.empty() || name == "." || name == "..") {
        return path;
    }
    std::size_t dot = name.rfind('.');
    std::string stem = (dot == std::string::npos || dot == 0) ? name : name.substr(0, dot);
    std::string out = path.substr(0, a) + stem;
    if (!ext.empty()) {
        out += '.';
        out += ext;
    }
    return out;
}

double powi(double a, int n) {
    const bool recip = n < 0;
    unsigned b = recip ? 0u - static_cast<unsigned>(n) : static_cast<unsigned>(n);
    double r = 1.0;
    bool first = true;
    for (;;) {
        if (b & 1u) {
            r = first ? a : r * a;
            first = false;
        }
        b >>= 1;
        if (b == 0) {
            break;
        }
        a *= a;
    }
    return recip ? 1.0 / r : r;
}

float powif(float a, int n) {
    const bool recip = n < 0;
    unsigned b = recip ? 0u - static_cast<unsigned>(n) : static_cast<unsigned>(n);
    float r = 1.0f;
    bool first = true;
    for (;;) {
        if (b & 1u) {
            r = first ? a : r * a;
            first = false;
        }
        b >>= 1;
        if (b == 0) {
            break;
        }
        a *= a;
    }
    return recip ? 1.0f / r : r;
}

int total_cmp(double a, double b) {
    std::int64_t x;
    std::int64_t y;
    std::memcpy(&x, &a, 8);
    std::memcpy(&y, &b, 8);
    x ^= static_cast<std::int64_t>(static_cast<std::uint64_t>(x >> 63) >> 1);
    y ^= static_cast<std::int64_t>(static_cast<std::uint64_t>(y >> 63) >> 1);
    return x < y ? -1 : (x > y ? 1 : 0);
}

std::string float_error(std::string_view s) {
    return s.empty() ? "cannot parse float from empty string" : "invalid float literal";
}

namespace {

template <class T>
std::optional<T> parse_uint(std::string_view s, std::string* err) {
    auto fail = [&](const char* m) -> std::optional<T> {
        if (err != nullptr) {
            *err = m;
        }
        return std::nullopt;
    };
    if (s.empty()) {
        return fail("cannot parse integer from empty string");
    }
    std::string_view d = s;
    if (d.front() == '+') {
        d.remove_prefix(1);
        if (d.empty()) {
            return fail("invalid digit found in string");
        }
    }
    T v = 0;
    bool over = false;
    for (char c : d) {
        if (c < '0' || c > '9') {
            return fail("invalid digit found in string");
        }
        const T dig = static_cast<T>(c - '0');
        if (!over && (v > (std::numeric_limits<T>::max() - dig) / 10)) {
            over = true;
        }
        if (!over) {
            v = static_cast<T>(v * 10 + dig);
        }
    }
    if (over) {
        return fail("number too large to fit in target type");
    }
    return v;
}

} // namespace

std::optional<std::uint32_t> parse_u32(std::string_view s, std::string* err) {
    return parse_uint<std::uint32_t>(s, err);
}

std::optional<std::uint64_t> parse_u64(std::string_view s, std::string* err) {
    return parse_uint<std::uint64_t>(s, err);
}

std::optional<std::size_t> parse_usize(std::string_view s, std::string* err) {
    return parse_uint<std::size_t>(s, err);
}

std::optional<std::int32_t> parse_i32(std::string_view s, std::string* err) {
    auto fail = [&](const char* m) -> std::optional<std::int32_t> {
        if (err != nullptr) {
            *err = m;
        }
        return std::nullopt;
    };
    if (s.empty()) {
        return fail("cannot parse integer from empty string");
    }
    bool neg = false;
    std::string_view d = s;
    if (d.front() == '+' || d.front() == '-') {
        neg = d.front() == '-';
        d.remove_prefix(1);
        if (d.empty()) {
            return fail("invalid digit found in string");
        }
    }
    std::int64_t v = 0;
    bool over = false;
    for (char c : d) {
        if (c < '0' || c > '9') {
            return fail("invalid digit found in string");
        }
        if (!over) {
            v = v * 10 + (c - '0');
            if (v > 2147483648LL) {
                over = true;
            }
        }
    }
    if (over || (!neg && v > 2147483647LL)) {
        return fail(neg ? "number too small to fit in target type" : "number too large to fit in target type");
    }
    return static_cast<std::int32_t>(neg ? -v : v);
}

std::optional<std::int64_t> parse_i64(std::string_view s, std::string* err) {
    auto fail = [&](const char* m) -> std::optional<std::int64_t> {
        if (err != nullptr) {
            *err = m;
        }
        return std::nullopt;
    };
    if (s.empty()) {
        return fail("cannot parse integer from empty string");
    }
    bool neg = false;
    std::string_view d = s;
    if (d.front() == '+' || d.front() == '-') {
        neg = d.front() == '-';
        d.remove_prefix(1);
        if (d.empty()) {
            return fail("invalid digit found in string");
        }
    }
    std::uint64_t v = 0;
    bool over = false;
    const std::uint64_t lim = neg ? 9223372036854775808ull : 9223372036854775807ull;
    for (char c : d) {
        if (c < '0' || c > '9') {
            return fail("invalid digit found in string");
        }
        const std::uint64_t dig = static_cast<std::uint64_t>(c - '0');
        if (!over && v > (lim - dig) / 10) {
            over = true;
        }
        if (!over) {
            v = v * 10 + dig;
        }
    }
    if (over) {
        return fail(neg ? "number too small to fit in target type" : "number too large to fit in target type");
    }
    return neg ? static_cast<std::int64_t>(0 - v) : static_cast<std::int64_t>(v);
}

std::optional<double> parse_f64(std::string_view s, std::string* err) {
    double v = 0;
    if (cam::rfmt::parse_f64(s, v)) {
        return v;
    }
    if (err != nullptr) {
        *err = float_error(s);
    }
    return std::nullopt;
}

void panic(std::string_view where, std::string_view msg) {
    std::fflush(stdout);
    std::fprintf(stderr, "\nthread 'main' panicked at %.*s:\n%.*s\nnote: run with `RUST_BACKTRACE=1` environment variable to display a backtrace\n",
                 static_cast<int>(where.size()), where.data(), static_cast<int>(msg.size()), msg.data());
    std::_Exit(101);
}

std::uint32_t process_id() {
    return static_cast<std::uint32_t>(::getpid());
}

std::uint64_t monotonic_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ull + static_cast<std::uint64_t>(ts.tv_nsec);
}

} // namespace pibridge::anchor::rs
