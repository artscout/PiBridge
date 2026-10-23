// SPDX-License-Identifier: MIT
#include "pibridge/hmd/settings.h"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include <pwd.h>
#include <unistd.h>

namespace pibridge::hmd {
namespace fs = std::filesystem;

namespace {

std::string_view trim(std::string_view s) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) {
        s.remove_prefix(1);
    }
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) {
        s.remove_suffix(1);
    }
    return s;
}

std::string lower(std::string_view s) {
    std::string o(s);
    std::transform(o.begin(), o.end(), o.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return o;
}

// An integer, or a number with a point that is a whole number ("90.0"); locale independent.
bool parse_int(std::string_view v, int& out) {
    const char* b = v.data();
    const char* e = v.data() + v.size();
    int i = 0;
    if (auto [p, ec] = std::from_chars(b, e, i); ec == std::errc{} && p == e && !v.empty()) {
        out = i;
        return true;
    }
    double d = 0;
    if (auto [p, ec] = std::from_chars(b, e, d); ec == std::errc{} && p == e && !v.empty() && std::isfinite(d) &&
                                                 d == std::floor(d) && std::fabs(d) < 1e9) {
        out = static_cast<int>(d);
        return true;
    }
    return false;
}

fs::path home_dir() {
    if (const char* h = std::getenv("HOME"); h && *h) {
        return h;
    }
    if (const passwd* pw = ::getpwuid(::getuid()); pw && pw->pw_dir) {
        return pw->pw_dir;
    }
    return "/";
}

} // namespace

bool parse_bool(std::string_view v, bool& ok) {
    const std::string l = lower(trim(v));
    ok = true;
    if (l == "on" || l == "1" || l == "true" || l == "yes") {
        return true;
    }
    if (l == "off" || l == "0" || l == "false" || l == "no") {
        return false;
    }
    ok = false;
    return false;
}

Settings Settings::parse(std::string_view text) {
    Settings s;
    std::size_t n = 0;
    while (!text.empty()) {
        const auto nl = text.find('\n');
        const std::string_view raw = text.substr(0, nl);
        text = nl == std::string_view::npos ? std::string_view{} : text.substr(nl + 1);
        n++;
        const std::string_view line = trim(raw);
        if (line.empty() || line.front() == '#') {
            continue;
        }
        const auto eq = line.find('=');
        const std::string where = "line " + std::to_string(n) + ": ";
        if (eq == std::string_view::npos) {
            s.warnings.push_back(where + "no '=' in \"" + std::string(line) + "\", ignored");
            continue;
        }
        const std::string key(trim(line.substr(0, eq)));
        const std::string_view val = trim(line.substr(eq + 1));
        const auto boolean = [&](bool& field) {
            bool ok = false;
            const bool b = parse_bool(val, ok);
            if (ok) {
                field = b;
            } else {
                s.warnings.push_back(where + key + "=" + std::string(val) + " is not on or off, kept " +
                                     (field ? "on" : "off"));
            }
        };
        if (key == "follow") {
            s.follow = std::string(val);
        } else if (key == "refresh_hz") {
            int hz = 0;
            if (parse_int(val, hz) && (hz == 72 || hz == 90)) {
                s.refresh_hz = hz;
            } else {
                s.warnings.push_back(where + "refresh_hz=" + std::string(val) + " is not 72 or 90, kept " +
                                     std::to_string(s.refresh_hz));
            }
        } else if (key == "audio_to_headset") {
            boolean(s.audio_to_headset);
        } else if (key == "mic_to_headset") {
            boolean(s.mic_to_headset);
        } else if (key == "eye_tracking") {
            boolean(s.eye_tracking);
        } else if (key == "eye_start_list") {
            s.eye_start_list = std::string(val);
        } else if (key == "eye_profile") {
            s.eye_profile = std::string(val);
        } else if (key == "eye_flip_x") {
            boolean(s.eye_flip_x);
        } else if (key == "crystal_tracking") {
            if (val == "lighthouse" || val == "slam" || val == "keep") {
                s.crystal_tracking = std::string(val);
            } else {
                s.warnings.push_back(where + "crystal_tracking=" + std::string(val) +
                                     " is not lighthouse, slam or keep, kept " + s.crystal_tracking);
            }
        } else if (key == "crystal_autostart_steamvr") {
            boolean(s.crystal_autostart_steamvr);
        }
        // unknown keys are ignored (other parts and newer apps add their own)
    }
    return s;
}

Settings Settings::load(const fs::path& path) {
    std::error_code ec;
    if (!fs::exists(path, ec)) {
        return Settings{};
    }
    std::ifstream f(path);
    if (!f) {
        Settings s;
        s.warnings.push_back(path.string() + ": cannot read, using the defaults");
        return s;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    return parse(ss.str());
}

fs::path Settings::config_dir() {
    if (const char* x = std::getenv("XDG_CONFIG_HOME"); x && *x) {
        return fs::path(x) / "pibridge";
    }
    return home_dir() / ".config" / "pibridge";
}

fs::path Settings::default_path() { return config_dir() / "pibridge.conf"; }

fs::path Settings::legacy_path() { return config_dir().parent_path() / "pimax-linux" / "pimax-linux.conf"; }

Settings Settings::load_or_legacy(const fs::path& path) {
    std::error_code ec;
    if (path == default_path() && !fs::exists(path, ec)) {
        if (const auto old = legacy_path(); fs::exists(old, ec)) {
            return load(old);
        }
    }
    return load(path);
}

} // namespace pibridge::hmd
