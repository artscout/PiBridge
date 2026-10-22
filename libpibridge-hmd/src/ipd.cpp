// SPDX-License-Identifier: MIT
#include "pibridge/hmd/ipd.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <system_error>

#include <unistd.h>

namespace pibridge::hmd::ipd {

std::filesystem::path request_path() {
    std::filesystem::path d;
    if (const char* x = std::getenv("XDG_RUNTIME_DIR"); x && *x) {
        d = std::filesystem::path(x) / "pibridge";
    } else {
        d = std::filesystem::path("/tmp") / ("pibridge-" + std::to_string(::getuid()));
    }
    return d / REQUEST_FILE;
}

std::optional<float> parse_request(std::string_view text) {
    std::string s(text);
    char* end = nullptr;
    const float v = std::strtof(s.c_str(), &end);
    if (end == s.c_str()) {
        return std::nullopt;
    }
    for (; *end != '\0'; ++end) {
        if (*end != ' ' && *end != '\t' && *end != '\r' && *end != '\n') {
            return std::nullopt;
        }
    }
    if (!std::isfinite(v) || v < 40.0f || v > 90.0f) {
        return std::nullopt;
    }
    return v;
}

std::expected<std::optional<float>, std::string> take_request(const std::filesystem::path& path) {
    // rename first: the request is ours alone, and a half-written file cannot be read (the app renames into place)
    const std::filesystem::path mine = path.string() + ".taken." + std::to_string(::getpid());
    std::error_code ec;
    std::filesystem::rename(path, mine, ec);
    if (ec) {
        if (ec == std::errc::no_such_file_or_directory) {
            return std::optional<float>{};
        }
        return std::unexpected(ec.message());
    }
    std::string s;
    {
        std::ifstream f(mine, std::ios::binary);
        s.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    }
    std::filesystem::remove(mine, ec);
    if (s.size() > 64) {
        return std::unexpected(std::string("IPD request too long"));
    }
    if (const auto v = parse_request(s)) {
        return std::optional<float>(*v);
    }
    return std::unexpected("bad IPD request \"" + s + "\"");
}

} // namespace pibridge::hmd::ipd
