// SPDX-License-Identifier: MIT
// Internal helpers for reading small sysfs attribute files.
#pragma once

#include <charconv>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>

namespace pibridge::eye::detail {

/// The file's text with surrounding whitespace removed, nullopt when it cannot be read.
inline std::optional<std::string> read_trim(const std::filesystem::path& p) {
    std::error_code ec;
    if (std::filesystem::is_directory(p, ec)) {
        return std::nullopt;
    }
    std::ifstream f(p);
    if (!f) {
        return std::nullopt;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    if (f.bad()) {
        return std::nullopt;
    }
    std::string s = ss.str();
    const auto b = s.find_first_not_of(" \t\r\n\v\f");
    if (b == std::string::npos) {
        return std::string();
    }
    const auto e = s.find_last_not_of(" \t\r\n\v\f");
    return s.substr(b, e - b + 1);
}

/// The whole string as an unsigned number in `base`.
template <class T>
std::optional<T> parse_uint(const std::optional<std::string>& s, int base) {
    if (!s || s->empty()) {
        return std::nullopt;
    }
    T v{};
    const auto r = std::from_chars(s->data(), s->data() + s->size(), v, base);
    if (r.ec != std::errc{} || r.ptr != s->data() + s->size()) {
        return std::nullopt;
    }
    return v;
}

} // namespace pibridge::eye::detail
