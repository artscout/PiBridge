// SPDX-License-Identifier: MIT
// Status files of the daemons, read by the app: $XDG_RUNTIME_DIR/pibridge/<name>.json, one flat JSON object,
// written atomically (temporary file + rename) so a reader never sees half a file. See docs/CONVENTIONS.md.
#pragma once

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pibridge::status {

// $XDG_RUNTIME_DIR/pibridge (or /tmp/pibridge-<uid> without it); created on demand.
inline std::filesystem::path runtime_dir() {
    std::filesystem::path d;
    if (const char* x = std::getenv("XDG_RUNTIME_DIR"); x && *x) {
        d = std::filesystem::path(x) / "pibridge";
    } else {
        d = std::filesystem::path("/tmp") / ("pibridge-" + std::to_string(::getuid()));
    }
    std::error_code ec;
    std::filesystem::create_directories(d, ec);
    return d;
}

inline std::string json_escape(std::string_view s) {
    std::string o;
    o.reserve(s.size() + 2);
    for (const char c : s) {
        switch (c) {
        case '"': o += "\\\""; break;
        case '\\': o += "\\\\"; break;
        case '\n': o += "\\n"; break;
        case '\r': o += "\\r"; break;
        case '\t': o += "\\t"; break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                char b[8];
                std::snprintf(b, sizeof b, "\\u%04x", c);
                o += b;
            } else {
                o += c;
            }
        }
    }
    return o;
}

// One flat JSON object built field by field; values are written as given (str() quotes and escapes).
class StatusWriter {
public:
    StatusWriter& str(std::string_view key, std::string_view v) {
        return raw(key, "\"" + json_escape(v) + "\"");
    }
    StatusWriter& num(std::string_view key, double v) {
        char b[64];
        std::snprintf(b, sizeof b, "%.6g", v);
        return raw(key, b);
    }
    StatusWriter& integer(std::string_view key, long long v) { return raw(key, std::to_string(v)); }
    StatusWriter& boolean(std::string_view key, bool v) { return raw(key, v ? "true" : "false"); }
    StatusWriter& raw(std::string_view key, std::string v) {
        fields_.emplace_back(std::string(key), std::move(v));
        return *this;
    }
    std::string text() const {
        std::string o = "{";
        for (size_t i = 0; i < fields_.size(); i++) {
            o += (i ? ",\n  \"" : "\n  \"") + json_escape(fields_[i].first) + "\": " + fields_[i].second;
        }
        return o + "\n}\n";
    }
    // Writes <runtime_dir>/<name>.json; adds "updated_unix_ms". False on any I/O error (the daemon goes on).
    bool write(std::string_view name) {
        const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        integer("updated_unix_ms", now);
        const auto dir = runtime_dir();
        const auto path = dir / (std::string(name) + ".json");
        const auto tmp = dir / ("." + std::string(name) + ".json.tmp");
        {
            std::ofstream f(tmp, std::ios::trunc);
            if (!f) {
                return false;
            }
            f << text();
            if (!f) {
                return false;
            }
        }
        std::error_code ec;
        std::filesystem::rename(tmp, path, ec);
        return !ec;
    }

private:
    std::vector<std::pair<std::string, std::string>> fields_;
};

} // namespace pibridge::status
