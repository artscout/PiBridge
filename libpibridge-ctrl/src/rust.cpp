// SPDX-License-Identifier: MIT
// Rust semantics the pimax-hands port needs (see rust.h).
#include "pibridge/ctrl/rust.h"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <unistd.h>

namespace pibridge::ctrl::rs {

void panic(const char* what) noexcept {
    std::fprintf(stderr, "panicked: %s\npanic in a function that cannot unwind\n", what);
    std::abort();
}

float fminf(float a, float b) {
    return std::fmin(a, b);
}

float fmaxf(float a, float b) {
    return std::fmax(a, b);
}

std::int16_t f64_to_i16(double x) {
    if (std::isnan(x)) {
        return 0;
    }
    if (x <= -32768.0) {
        return INT16_MIN;
    }
    if (x >= 32767.0) {
        return INT16_MAX;
    }
    return static_cast<std::int16_t>(x);
}

std::int64_t f64_to_i64(double x) {
    if (std::isnan(x)) {
        return 0;
    }
    if (x <= -9223372036854775808.0) {
        return INT64_MIN;
    }
    if (x >= 9223372036854775808.0) {
        return INT64_MAX;
    }
    return static_cast<std::int64_t>(x);
}

std::size_t f64_to_usize(double x) {
    if (std::isnan(x) || x <= 0.0) {
        return 0;
    }
    if (x >= 18446744073709551615.0) {
        return std::numeric_limits<std::size_t>::max();
    }
    return static_cast<std::size_t>(x);
}

std::string lower_exp(double x) {
    if (std::isnan(x)) {
        return "NaN";
    }
    if (std::isinf(x)) {
        return x < 0 ? "-inf" : "inf";
    }
    if (x == 0.0) {
        return std::signbit(x) ? "-0e0" : "0e0";
    }
    char buf[64];
    const auto r = std::to_chars(buf, buf + sizeof buf, x, std::chars_format::scientific);
    std::string s(buf, r.ptr);
    const auto e = s.find('e');
    std::string mant = s.substr(0, e);
    int exp = std::atoi(s.c_str() + e + 1);
    return mant + "e" + std::to_string(exp);
}

std::size_t ws_len(std::string_view s, std::size_t i) {
    const auto b = [&](std::size_t k) { return k < s.size() ? static_cast<unsigned char>(s[k]) : 0u; };
    const unsigned c = b(i);
    if (c == ' ' || (c >= 0x09 && c <= 0x0d)) {
        return 1;
    }
    if (c == 0xc2 && (b(i + 1) == 0x85 || b(i + 1) == 0xa0)) {
        return 2;
    }
    if (c == 0xe1 && b(i + 1) == 0x9a && b(i + 2) == 0x80) {
        return 3; // U+1680
    }
    if (c == 0xe2 && b(i + 1) == 0x80) {
        const unsigned d = b(i + 2);
        if ((d >= 0x80 && d <= 0x8a) || d == 0xa8 || d == 0xa9 || d == 0xaf) {
            return 3; // U+2000..U+200A, U+2028, U+2029, U+202F
        }
    }
    if (c == 0xe2 && b(i + 1) == 0x81 && b(i + 2) == 0x9f) {
        return 3; // U+205F
    }
    if (c == 0xe3 && b(i + 1) == 0x80 && b(i + 2) == 0x80) {
        return 3; // U+3000
    }
    return 0;
}

std::string with_extension(const std::string& path, std::string_view ext) {
    // The file name: the last component, trailing separators and "." components dropped (Path::components).
    std::size_t end = path.size();
    for (;;) {
        while (end > 0 && path[end - 1] == '/') {
            end--;
        }
        if (end >= 2 && path[end - 1] == '.' && path[end - 2] == '/') {
            end -= 1;
            continue;
        }
        if (end == 1 && path[0] == '.' && path.size() > 1) {
            break; // a leading "./" stays a component (CurDir), not a file name
        }
        break;
    }
    std::size_t start = path.rfind('/', end == 0 ? 0 : end - 1);
    start = (start == std::string::npos || end == 0) ? 0 : start + 1;
    if (end == 0 && !path.empty() && path[0] == '/') {
        return path; // "/": no file name
    }
    const std::string_view name(path.data() + start, end - start);
    if (name.empty() || name == ".." || name == ".") {
        return path; // no file name: unchanged
    }
    // file_stem: up to the last '.', unless the only '.' is the first byte
    std::size_t stem_len = name.size();
    const auto dot = name.rfind('.');
    if (dot != std::string_view::npos && dot != 0) {
        stem_len = dot;
    }
    std::string out = path.substr(0, start + stem_len);
    if (!ext.empty()) {
        out += '.';
        out += ext;
    }
    return out;
}

std::uint32_t process_id() {
    return static_cast<std::uint32_t>(::getpid());
}

} // namespace pibridge::ctrl::rs
