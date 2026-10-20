// SPDX-License-Identifier: MIT
// Rust's formatting rules for the values pimax-cam prints (see rustfmt.h).
#include "pibridge/cam/rustfmt.h"

#include <charconv>
#include <clocale>
#include <cmath>
#include <cstdlib>
#include <format>
#include <limits>
#include <locale.h>

namespace pibridge::cam::rfmt {

namespace {

// Shortest round-trip digits of |x| (finite, non-zero) and the decimal exponent of the first digit:
// x = 0.d1d2d3... * 10^(exp10 + 1), i.e. d1.d2d3 * 10^exp10.
void shortest_digits(double x, std::string& digits, int& exp10) {
    char buf[64];
    const auto r = std::to_chars(buf, buf + sizeof buf, std::fabs(x), std::chars_format::scientific);
    const std::string_view s(buf, static_cast<std::size_t>(r.ptr - buf));
    const std::size_t e = s.find('e');
    digits.clear();
    for (std::size_t i = 0; i < e; i++) {
        if (s[i] != '.') {
            digits.push_back(s[i]);
        }
    }
    exp10 = std::atoi(std::string(s.substr(e + 1)).c_str());
}

std::string non_finite(double x) {
    if (std::isnan(x)) {
        return "NaN";
    }
    return x < 0 ? "-inf" : "inf";
}

// Decimal notation of the shortest digits with at least `min_frac` fraction digits.
std::string decimal(double x, int min_frac) {
    if (x == 0.0) {
        std::string s = std::signbit(x) ? "-0" : "0";
        if (min_frac > 0) {
            s += '.';
            s.append(static_cast<std::size_t>(min_frac), '0');
        }
        return s;
    }
    std::string d;
    int e = 0;
    shortest_digits(x, d, e);
    std::string out = std::signbit(x) ? "-" : "";
    const int n = static_cast<int>(d.size());
    std::string intpart;
    std::string frac;
    if (e >= 0) {
        if (n <= e + 1) {
            intpart = d + std::string(static_cast<std::size_t>(e + 1 - n), '0');
        } else {
            intpart = d.substr(0, static_cast<std::size_t>(e + 1));
            frac = d.substr(static_cast<std::size_t>(e + 1));
        }
    } else {
        intpart = "0";
        frac = std::string(static_cast<std::size_t>(-e - 1), '0') + d;
    }
    if (static_cast<int>(frac.size()) < min_frac) {
        frac.append(static_cast<std::size_t>(min_frac - static_cast<int>(frac.size())), '0');
    }
    out += intpart;
    if (!frac.empty()) {
        out += '.';
        out += frac;
    }
    return out;
}

const locale_t& c_locale() {
    static const locale_t loc = newlocale(LC_ALL_MASK, "C", static_cast<locale_t>(nullptr));
    return loc;
}

bool ieq(std::string_view a, std::string_view b) {
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); i++) {
        char c = a[i];
        if (c >= 'A' && c <= 'Z') {
            c = static_cast<char>(c - 'A' + 'a');
        }
        if (c != b[i]) {
            return false;
        }
    }
    return true;
}

} // namespace

std::string debug_f64(double x) {
    if (!std::isfinite(x)) {
        return non_finite(x);
    }
    const double a = std::fabs(x);
    if (a != 0.0 && (a >= 1e16 || a < 1e-4)) {
        std::string d;
        int e = 0;
        shortest_digits(x, d, e);
        std::string out = std::signbit(x) ? "-" : "";
        out += d[0];
        if (d.size() > 1) {
            out += '.';
            out += d.substr(1);
        }
        out += 'e';
        out += std::to_string(e);
        return out;
    }
    return decimal(x, 1);
}

std::string display_f64(double x) {
    if (!std::isfinite(x)) {
        return non_finite(x);
    }
    return decimal(x, 0);
}

std::string fixed(double x, int prec, bool plus) {
    if (std::isnan(x)) {
        return "NaN";
    }
    if (std::isinf(x)) {
        return x < 0 ? "-inf" : (plus ? "+inf" : "inf");
    }
    return plus ? std::format("{:+.{}f}", x, prec) : std::format("{:.{}f}", x, prec);
}

std::string fixed_w(double x, int width, int prec, bool plus) {
    return std::format("{:>{}}", fixed(x, prec, plus), width);
}

namespace {
#include "rustfmt_escape.inc"

bool rust_escapes(char32_t c) {
    std::size_t lo = 0;
    std::size_t hi = sizeof k_rust_escaped / sizeof k_rust_escaped[0];
    while (lo < hi) {
        const std::size_t mid = (lo + hi) / 2;
        if (k_rust_escaped[mid][1] < c) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo < sizeof k_rust_escaped / sizeof k_rust_escaped[0] && k_rust_escaped[lo][0] <= c;
}

// Length of the UTF-8 sequence at s[i] and its code point; 0 when it is not a valid sequence.
std::size_t utf8_at(std::string_view s, std::size_t i, char32_t& cp) {
    const auto b0 = static_cast<unsigned char>(s[i]);
    std::size_t n = 0;
    if (b0 < 0x80) {
        cp = b0;
        return 1;
    }
    if ((b0 & 0xe0) == 0xc0) {
        n = 2;
        cp = b0 & 0x1f;
    } else if ((b0 & 0xf0) == 0xe0) {
        n = 3;
        cp = b0 & 0x0f;
    } else if ((b0 & 0xf8) == 0xf0) {
        n = 4;
        cp = b0 & 0x07;
    } else {
        return 0;
    }
    if (i + n > s.size()) {
        return 0;
    }
    for (std::size_t k = 1; k < n; k++) {
        const auto b = static_cast<unsigned char>(s[i + k]);
        if ((b & 0xc0) != 0x80) {
            return 0;
        }
        cp = (cp << 6) | (b & 0x3f);
    }
    return n;
}
} // namespace

std::string debug_str(std::string_view s) {
    std::string o = "\"";
    for (std::size_t i = 0; i < s.size();) {
        const char ch = s[i];
        switch (ch) {
        case '"': o += "\\\""; i++; continue;
        case '\\': o += "\\\\"; i++; continue;
        case '\n': o += "\\n"; i++; continue;
        case '\r': o += "\\r"; i++; continue;
        case '\t': o += "\\t"; i++; continue;
        case 0: o += "\\0"; i++; continue;
        default: break;
        }
        char32_t cp = 0;
        const std::size_t n = utf8_at(s, i, cp);
        if (n == 0) {
            // not UTF-8 (a Rust &str never is): the byte as it is
            o += ch;
            i++;
            continue;
        }
        if (rust_escapes(cp)) {
            o += std::format("\\u{{{:x}}}", static_cast<std::uint32_t>(cp));
        } else {
            o.append(s.substr(i, n));
        }
        i += n;
    }
    o += '"';
    return o;
}

std::string debug_f64s(std::span<const double> v) {
    std::string o = "[";
    for (std::size_t i = 0; i < v.size(); i++) {
        if (i > 0) {
            o += ", ";
        }
        o += debug_f64(v[i]);
    }
    return o + "]";
}

std::string debug_u16s(std::span<const std::uint16_t> v) {
    std::string o = "[";
    for (std::size_t i = 0; i < v.size(); i++) {
        if (i > 0) {
            o += ", ";
        }
        o += std::to_string(v[i]);
    }
    return o + "]";
}

namespace {

// Length of the valid UTF-8 sequence at s[i], or 0 when invalid; `bad` = how many bytes the maximal invalid prefix
// spans (the unit replaced by one U+FFFD, as Rust's Utf8Chunks does).
std::size_t utf8_seq(std::string_view s, std::size_t i, std::size_t& bad) {
    const auto b = [&](std::size_t k) { return static_cast<unsigned char>(s[k]); };
    const unsigned char c = b(i);
    bad = 1;
    if (c < 0x80) {
        return 1;
    }
    std::size_t need = 0;
    unsigned char lo = 0x80;
    unsigned char hi = 0xbf;
    if (c >= 0xc2 && c <= 0xdf) {
        need = 1;
    } else if (c >= 0xe0 && c <= 0xef) {
        need = 2;
        if (c == 0xe0) {
            lo = 0xa0;
        } else if (c == 0xed) {
            hi = 0x9f;
        }
    } else if (c >= 0xf0 && c <= 0xf4) {
        need = 3;
        if (c == 0xf0) {
            lo = 0x90;
        } else if (c == 0xf4) {
            hi = 0x8f;
        }
    } else {
        return 0;
    }
    for (std::size_t k = 1; k <= need; k++) {
        if (i + k >= s.size()) {
            bad = k;
            return 0;
        }
        const unsigned char d = b(i + k);
        const unsigned char l = k == 1 ? lo : 0x80;
        const unsigned char h = k == 1 ? hi : 0xbf;
        if (d < l || d > h) {
            bad = k;
            return 0;
        }
    }
    return need + 1;
}

} // namespace

std::string utf8_lossy(std::string_view s) {
    std::string o;
    o.reserve(s.size());
    std::size_t i = 0;
    while (i < s.size()) {
        std::size_t bad = 1;
        const std::size_t n = utf8_seq(s, i, bad);
        if (n == 0) {
            o += "\xef\xbf\xbd";
            i += bad;
        } else {
            o.append(s.substr(i, n));
            i += n;
        }
    }
    return o;
}

bool utf8_valid(std::string_view s) {
    std::size_t i = 0;
    while (i < s.size()) {
        std::size_t bad = 1;
        const std::size_t n = utf8_seq(s, i, bad);
        if (n == 0) {
            return false;
        }
        i += n;
    }
    return true;
}

namespace {

// Byte length of the Unicode White_Space character at s[i], 0 when s[i] does not start one.
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

// Length of the White_Space character that ends at s[end-1], 0 if none.
std::size_t ws_len_back(std::string_view s, std::size_t end) {
    for (std::size_t n = 1; n <= 3 && n <= end; n++) {
        if (ws_len(s, end - n) == n) {
            return n;
        }
    }
    return 0;
}

} // namespace

std::string_view trim(std::string_view s) {
    std::size_t a = 0;
    while (a < s.size()) {
        const std::size_t n = ws_len(s, a);
        if (n == 0) {
            break;
        }
        a += n;
    }
    std::size_t e = s.size();
    while (e > a) {
        const std::size_t n = ws_len_back(s, e);
        if (n == 0) {
            break;
        }
        e -= n;
    }
    return s.substr(a, e - a);
}

std::vector<std::string_view> split_whitespace(std::string_view s) {
    std::vector<std::string_view> out;
    std::size_t i = 0;
    while (i < s.size()) {
        std::size_t n = ws_len(s, i);
        if (n > 0) {
            i += n;
            continue;
        }
        const std::size_t start = i;
        while (i < s.size() && ws_len(s, i) == 0) {
            i++;
        }
        out.push_back(s.substr(start, i - start));
    }
    return out;
}

bool parse_f64(std::string_view s, double& out) {
    bool neg = false;
    std::string_view t = s;
    if (!t.empty() && (t[0] == '+' || t[0] == '-')) {
        neg = t[0] == '-';
        t.remove_prefix(1);
    }
    if (t.empty()) {
        return false;
    }
    if (ieq(t, "inf") || ieq(t, "infinity")) {
        out = neg ? -std::numeric_limits<double>::infinity() : std::numeric_limits<double>::infinity();
        return true;
    }
    if (ieq(t, "nan")) {
        out = std::numeric_limits<double>::quiet_NaN();
        return true;
    }
    // digits* [. digits*] [e [+-] digits+], at least one mantissa digit
    std::size_t i = 0;
    std::size_t mant = 0;
    while (i < t.size() && t[i] >= '0' && t[i] <= '9') {
        i++;
        mant++;
    }
    if (i < t.size() && t[i] == '.') {
        i++;
        while (i < t.size() && t[i] >= '0' && t[i] <= '9') {
            i++;
            mant++;
        }
    }
    if (mant == 0) {
        return false;
    }
    if (i < t.size() && (t[i] == 'e' || t[i] == 'E')) {
        i++;
        if (i < t.size() && (t[i] == '+' || t[i] == '-')) {
            i++;
        }
        std::size_t ed = 0;
        while (i < t.size() && t[i] >= '0' && t[i] <= '9') {
            i++;
            ed++;
        }
        if (ed == 0) {
            return false;
        }
    }
    if (i != t.size()) {
        return false;
    }
    // correctly rounded conversion; overflow gives inf and underflow 0, like Rust
    const std::string z(t);
    double v = strtod_l(z.c_str(), nullptr, c_locale());
    out = neg ? -v : v;
    return true;
}

bool parse_u64(std::string_view s, std::uint64_t& out) {
    if (!s.empty() && s[0] == '+') {
        s.remove_prefix(1);
    }
    if (s.empty()) {
        return false;
    }
    std::uint64_t v = 0;
    for (const char c : s) {
        if (c < '0' || c > '9') {
            return false;
        }
        const auto d = static_cast<std::uint64_t>(c - '0');
        if (v > (std::numeric_limits<std::uint64_t>::max() - d) / 10) {
            return false;
        }
        v = v * 10 + d;
    }
    out = v;
    return true;
}

bool parse_u32(std::string_view s, std::uint32_t& out) {
    std::uint64_t v = 0;
    if (!parse_u64(s, v) || v > std::numeric_limits<std::uint32_t>::max()) {
        return false;
    }
    out = static_cast<std::uint32_t>(v);
    return true;
}

std::uint32_t f64_to_u32(double x) {
    if (std::isnan(x) || x <= 0.0) {
        return 0;
    }
    if (x >= 4294967295.0) {
        return std::numeric_limits<std::uint32_t>::max();
    }
    return static_cast<std::uint32_t>(x);
}

std::uint64_t f64_to_u64(double x) {
    if (std::isnan(x) || x <= 0.0) {
        return 0;
    }
    if (x >= 18446744073709551615.0) {
        return std::numeric_limits<std::uint64_t>::max();
    }
    return static_cast<std::uint64_t>(x);
}

double to_radians(double deg) {
    constexpr double rads_per_deg = 3.14159265358979323846264338327950288 / 180.0;
    return deg * rads_per_deg;
}

double to_degrees(double rad) {
    constexpr double degs_per_rad = 180.0 / 3.14159265358979323846264338327950288;
    return rad * degs_per_rad;
}

double clamp(double x, double lo, double hi) {
    if (x < lo) {
        x = lo;
    }
    if (x > hi) {
        x = hi;
    }
    return x;
}

double fmin(double a, double b) {
    return std::fmin(a, b);
}

double fmax(double a, double b) {
    return std::fmax(a, b);
}

} // namespace pibridge::cam::rfmt
