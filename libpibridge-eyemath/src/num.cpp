// SPDX-License-Identifier: MIT
#include "num.h"

#include <array>
#include <cerrno>
#include <charconv>
#include <cstdio>
#include <cstring>
#include <format>
#include <system_error>

#include <fcntl.h>
#include <unistd.h>

namespace pibridge::eyemath::detail {

std::string fmt_fixed(double x, int prec) {
    if (std::isnan(x)) {
        return "NaN";
    }
    if (std::isinf(x)) {
        return x > 0 ? "inf" : "-inf";
    }
    // the largest double has 309 integer digits; plus sign, point and the decimals
    std::array<char, 400> buf{};
    const auto r = std::to_chars(buf.data(), buf.data() + buf.size(), x, std::chars_format::fixed, prec);
    if (r.ec != std::errc{}) {
        return "NaN";
    }
    return std::string(buf.data(), r.ptr);
}

namespace {

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

bool is_digit(char c) { return c >= '0' && c <= '9'; }

// A decimal literal in Rust's grammar: digits [ '.' digits ] [ (e|E) [sign] digits ], at least one mantissa digit.
// from_chars accepts the same set (no hex in chars_format::general), but the check keeps its extensions out.
bool is_decimal(std::string_view s) {
    std::size_t i = 0;
    std::size_t mant = 0;
    while (i < s.size() && is_digit(s[i])) {
        i++;
        mant++;
    }
    if (i < s.size() && s[i] == '.') {
        i++;
        while (i < s.size() && is_digit(s[i])) {
            i++;
            mant++;
        }
    }
    if (mant == 0) {
        return false;
    }
    if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
        i++;
        if (i < s.size() && (s[i] == '+' || s[i] == '-')) {
            i++;
        }
        std::size_t exp = 0;
        while (i < s.size() && is_digit(s[i])) {
            i++;
            exp++;
        }
        if (exp == 0) {
            return false;
        }
    }
    return i == s.size();
}

// Decimal exponent of the first significant digit (roughly log10 of the magnitude) of a valid decimal literal; used
// only to tell overflow from underflow when from_chars reports out of range.
long magnitude(std::string_view s) {
    long int_digits = 0;
    long lead_frac_zeros = 0;
    bool seen_nonzero = false;
    std::size_t i = 0;
    while (i < s.size() && is_digit(s[i])) {
        if (s[i] != '0' || seen_nonzero) {
            seen_nonzero = true;
            int_digits++;
        }
        i++;
    }
    if (i < s.size() && s[i] == '.') {
        i++;
        while (i < s.size() && is_digit(s[i])) {
            if (!seen_nonzero) {
                if (s[i] == '0') {
                    lead_frac_zeros++;
                } else {
                    seen_nonzero = true;
                }
            }
            i++;
        }
    }
    long exp = 0;
    if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
        i++;
        bool neg = false;
        if (i < s.size() && (s[i] == '+' || s[i] == '-')) {
            neg = s[i] == '-';
            i++;
        }
        while (i < s.size() && is_digit(s[i])) {
            if (exp < 100000) {
                exp = exp * 10 + (s[i] - '0');
            }
            i++;
        }
        if (neg) {
            exp = -exp;
        }
    }
    return int_digits > 0 ? int_digits + exp : exp - lead_frac_zeros;
}

} // namespace

std::expected<double, std::string> parse_f64(std::string_view s) {
    if (s.empty()) {
        return std::unexpected(std::string("cannot parse float from empty string"));
    }
    const std::string invalid = "invalid float literal";
    bool neg = false;
    std::string_view body = s;
    if (body.front() == '+' || body.front() == '-') {
        neg = body.front() == '-';
        body.remove_prefix(1);
    }
    if (ieq(body, "inf") || ieq(body, "infinity")) {
        return neg ? -std::numeric_limits<double>::infinity() : std::numeric_limits<double>::infinity();
    }
    if (ieq(body, "nan")) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (!is_decimal(body)) {
        return std::unexpected(invalid);
    }
    double v = 0.0;
    const auto r = std::from_chars(body.data(), body.data() + body.size(), v, std::chars_format::general);
    if (r.ec == std::errc::result_out_of_range) {
        v = magnitude(body) > 0 ? std::numeric_limits<double>::infinity() : 0.0;
    } else if (r.ec != std::errc{} || r.ptr != body.data() + body.size()) {
        return std::unexpected(invalid);
    }
    return neg ? -v : v;
}

namespace {

std::string io_error(int err) { return std::format("{} (os error {})", std::strerror(err), err); }

} // namespace

std::expected<std::string, std::string> read_text(const char* path) {
    const int fd = ::open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        return std::unexpected(io_error(errno));
    }
    std::string out;
    std::array<char, 8192> buf{};
    while (true) {
        const ssize_t n = ::read(fd, buf.data(), buf.size());
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            const int err = errno;
            ::close(fd);
            return std::unexpected(io_error(err));
        }
        if (n == 0) {
            break;
        }
        out.append(buf.data(), static_cast<std::size_t>(n));
    }
    ::close(fd);
    return out;
}

std::expected<void, std::string> write_text_atomic(const char* path, std::string_view text) {
    const std::string tmp = std::format("{}.tmp-{}", path, ::getpid());
    const int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (fd < 0) {
        return std::unexpected(io_error(errno));
    }
    std::size_t done = 0;
    while (done < text.size()) {
        const ssize_t n = ::write(fd, text.data() + done, text.size() - done);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            const int err = errno;
            ::close(fd);
            ::unlink(tmp.c_str());
            return std::unexpected(io_error(err));
        }
        done += static_cast<std::size_t>(n);
    }
    int err = ::fsync(fd) == 0 ? 0 : errno;
    if (::close(fd) != 0 && err == 0) {
        err = errno;
    }
    if (err != 0) {
        ::unlink(tmp.c_str());
        return std::unexpected(io_error(err));
    }
    if (::rename(tmp.c_str(), path) != 0) {
        err = errno;
        ::unlink(tmp.c_str());
        return std::unexpected(io_error(err));
    }
    return {};
}

std::string_view trim(std::string_view s) {
    constexpr std::string_view ws = " \t\n\r\v\f";
    const auto b = s.find_first_not_of(ws);
    if (b == std::string_view::npos) {
        return {};
    }
    const auto e = s.find_last_not_of(ws);
    return s.substr(b, e - b + 1);
}

std::vector<std::string_view> lines(std::string_view s) {
    std::vector<std::string_view> out;
    std::size_t pos = 0;
    while (pos < s.size()) {
        auto nl = s.find('\n', pos);
        std::string_view l = nl == std::string_view::npos ? s.substr(pos) : s.substr(pos, nl - pos);
        if (!l.empty() && l.back() == '\r') {
            l.remove_suffix(1);
        }
        out.push_back(l);
        if (nl == std::string_view::npos) {
            break;
        }
        pos = nl + 1;
    }
    return out;
}

std::string unquote(std::string_view s) {
    while (!s.empty() && s.front() == '"') {
        s.remove_prefix(1);
    }
    while (!s.empty() && s.back() == '"') {
        s.remove_suffix(1);
    }
    return std::string(trim(s));
}

} // namespace pibridge::eyemath::detail
