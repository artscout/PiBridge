// SPDX-License-Identifier: MIT
#include "settings.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace qv {

namespace {

std::string trim(const std::string& s) {
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) {
        return {};
    }
    const auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

// Locale-independent both ways: the settings app runs in the user's locale (a decimal comma under ru_RU, where
// printf wrote "0,13" that strtof in the game's "C" locale then refused), the game in whatever locale it set.
// A comma is read as the point, so files written by the old app still load.
bool parseFloat(const std::string& v, float lo, float hi, float& out) {
    std::string t = v;
    std::replace(t.begin(), t.end(), ',', '.');
    float x = 0.f;
    const char* b = t.data();
    const char* e = t.data() + t.size();
    if (!t.empty() && *b == '+') {
        ++b;
    }
    const auto r = std::from_chars(b, e, x);
    if (t.empty() || r.ec != std::errc() || r.ptr != e || !std::isfinite(x)) {
        return false;
    }
    out = std::clamp(x, lo, hi);
    return true;
}

bool parseOnOff(const std::string& v, bool& out) {
    if (v == "on" || v == "1" || v == "true" || v == "yes") {
        out = true;
        return true;
    }
    if (v == "off" || v == "0" || v == "false" || v == "no") {
        out = false;
        return true;
    }
    return false;
}

std::string fmt(float x, int digits) {
    char buf[32];
    const auto r = std::to_chars(buf, buf + sizeof(buf), x, std::chars_format::fixed, digits);
    return std::string(buf, r.ptr);
}

std::filesystem::path xdgDir(const char* var, const char* fallbackBelowHome) {
    if (const char* x = std::getenv(var); x && *x) {
        return std::filesystem::path(x) / "qv-vulkan";
    }
    if (const char* h = std::getenv("HOME"); h && *h) {
        return std::filesystem::path(h) / fallbackBelowHome / "qv-vulkan";
    }
    return {};
}

} // namespace

bool Settings::set(const std::string& rawKey, const std::string& rawValue) {
    const std::string key = trim(rawKey);
    const std::string v = trim(rawValue);
    if (key == "peripheral_density") return parseFloat(v, 0.05f, 1.5f, peripheralDensity);
    if (key == "focus_density") return parseFloat(v, 0.5f, 2.0f, focusDensity);
    if (key == "fixed_width") return parseFloat(v, 0.1f, 0.9f, fixedSize[0]);
    if (key == "fixed_height") return parseFloat(v, 0.1f, 0.9f, fixedSize[1]);
    if (key == "gaze_width") return parseFloat(v, 0.1f, 0.9f, gazeSize[0]);
    if (key == "gaze_height") return parseFloat(v, 0.1f, 0.9f, gazeSize[1]);
    if (key == "offset_x_deg") return parseFloat(v, -30.f, 30.f, offsetDeg[0]);
    if (key == "offset_y_deg") return parseFloat(v, -30.f, 30.f, offsetDeg[1]);
    if (key == "transition") return parseFloat(v, 0.f, 0.5f, transition);
    if (key == "edge_opacity") return parseFloat(v, 0.f, 1.f, edgeOpacity);
    if (key == "sharpen") return parseFloat(v, 0.f, 1.f, sharpen);
    if (key == "gaze_filter") return parseOnOff(v, gazeFilter);
    if (key == "debug_overlay") return parseOnOff(v, debugOverlay);
    if (key == "tracking") {
        if (v == "eye") tracking = Tracking::Eye;
        else if (v == "fixed") tracking = Tracking::Fixed;
        else return false;
        return true;
    }
    if (key == "peripheral_filter") {
        if (v == "bicubic") peripheralFilter = Filter::Bicubic;
        else if (v == "bilinear") peripheralFilter = Filter::Bilinear;
        else return false;
        return true;
    }
    if (key == "gaze_hold_ms") {
        char* end = nullptr;
        const unsigned long x = std::strtoul(v.c_str(), &end, 10);
        if (v.empty() || *end != '\0' || v[0] == '-') {
            return false;
        }
        gazeHoldMs = static_cast<uint32_t>(std::min(x, 5000ul));
        return true;
    }
    return false;
}

std::string Settings::toLines() const {
    std::ostringstream o;
    o << "peripheral_density=" << fmt(peripheralDensity, 2) << "\n"
      << "focus_density=" << fmt(focusDensity, 2) << "\n"
      << "tracking=" << (tracking == Tracking::Eye ? "eye" : "fixed") << "\n"
      << "fixed_width=" << fmt(fixedSize[0], 2) << "\n"
      << "fixed_height=" << fmt(fixedSize[1], 2) << "\n"
      << "gaze_width=" << fmt(gazeSize[0], 2) << "\n"
      << "gaze_height=" << fmt(gazeSize[1], 2) << "\n"
      << "offset_x_deg=" << fmt(offsetDeg[0], 1) << "\n"
      << "offset_y_deg=" << fmt(offsetDeg[1], 1) << "\n"
      << "transition=" << fmt(transition, 2) << "\n"
      << "edge_opacity=" << fmt(edgeOpacity, 2) << "\n"
      << "sharpen=" << fmt(sharpen, 2) << "\n"
      << "peripheral_filter=" << (peripheralFilter == Filter::Bicubic ? "bicubic" : "bilinear") << "\n"
      << "gaze_filter=" << (gazeFilter ? "on" : "off") << "\n"
      << "gaze_hold_ms=" << gazeHoldMs << "\n"
      << "debug_overlay=" << (debugOverlay ? "on" : "off") << "\n";
    return o.str();
}

AppId AppId::current(const std::string& applicationName) {
    AppId id;
    id.name = applicationName;
    std::ifstream f("/proc/self/comm");
    std::getline(f, id.exe);
    id.exe = trim(id.exe);
    return id;
}

Settings parseSettings(const std::string& text, const AppId& app) {
    Settings s;
    bool active = true;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        line = trim(line.substr(0, line.find('#')));
        if (line.empty()) {
            continue;
        }
        if (line.front() == '[' && line.back() == ']') {
            const std::string sec = line.substr(1, line.size() - 2);
            const auto matches = [&](const std::string& prefix, const std::string& value) {
                if (sec.rfind(prefix, 0) != 0) return false;
                const std::string needle = sec.substr(prefix.size());
                return !needle.empty() && value.find(needle) != std::string::npos;
            };
            active = matches("app:", app.name) || matches("exe:", app.exe);
            continue;
        }
        const auto eq = line.find('=');
        if (active && eq != std::string::npos) {
            s.set(line.substr(0, eq), line.substr(eq + 1));
        }
    }
    return s;
}

Settings loadSettings(const std::filesystem::path& path, const AppId& app) {
    std::ifstream f(path);
    if (!f) {
        return {};
    }
    std::stringstream ss;
    ss << f.rdbuf();
    return parseSettings(ss.str(), app);
}

std::string replaceGlobal(const std::string& oldText, const Settings& s) {
    std::string out = "# Quad views (qv-vulkan) settings. Written by qv-vulkan-settings; see README.md.\n";
    out += s.toLines();
    bool inSection = false;
    std::istringstream in(oldText);
    std::string line;
    while (std::getline(in, line)) {
        const std::string t = trim(line);
        if (!t.empty() && t.front() == '[') {
            inSection = true;
        }
        if (inSection) {
            out += line + "\n";
        }
    }
    return out;
}

bool saveSettings(const std::filesystem::path& path, const Settings& s) {
    std::string old;
    if (std::ifstream f(path); f) {
        std::stringstream ss;
        ss << f.rdbuf();
        old = ss.str();
    }
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
    const auto tmp = std::filesystem::path(path.string() + ".tmp");
    {
        std::ofstream o(tmp, std::ios::trunc);
        if (!o) {
            return false;
        }
        o << replaceGlobal(old, s);
        if (!o.flush()) {
            return false;
        }
    }
    std::filesystem::rename(tmp, path, ec);
    return !ec;
}

namespace {

// The section name of a header line ("[app:X]" -> "app:X"), or empty.
std::string headerOf(const std::string& rawLine) {
    const std::string t = trim(rawLine.substr(0, rawLine.find('#')));
    if (t.size() > 2 && t.front() == '[' && t.back() == ']') {
        return t.substr(1, t.size() - 2);
    }
    return {};
}

std::vector<std::string> lines(const std::string& text) {
    std::vector<std::string> v;
    std::istringstream in(text);
    std::string l;
    while (std::getline(in, l)) {
        v.push_back(l);
    }
    return v;
}

// [begin, end) of the section's lines including its header; begin == end when missing.
std::pair<size_t, size_t> sectionRange(const std::vector<std::string>& v, const std::string& section) {
    for (size_t i = 0; i < v.size(); i++) {
        if (headerOf(v[i]) == section) {
            size_t j = i + 1;
            while (j < v.size() && headerOf(v[j]).empty()) {
                j++;
            }
            return {i, j};
        }
    }
    return {v.size(), v.size()};
}

std::string join(const std::vector<std::string>& v) {
    std::string o;
    for (const auto& l : v) {
        o += l + "\n";
    }
    return o;
}

} // namespace

std::vector<std::string> sectionNames(const std::string& text) {
    std::vector<std::string> out;
    for (const auto& l : lines(text)) {
        const std::string h = headerOf(l);
        if ((h.rfind("app:", 0) == 0 || h.rfind("exe:", 0) == 0) && h.size() > 4 &&
            std::find(out.begin(), out.end(), h) == out.end()) {
            out.push_back(h);
        }
    }
    return out;
}

Settings parseSection(const std::string& text, const std::string& section) {
    Settings s;
    std::string current;
    for (const auto& raw : lines(text)) {
        if (const std::string h = headerOf(raw); !h.empty()) {
            current = h;
            continue;
        }
        if (!current.empty() && current != section) {
            continue;
        }
        const std::string l = trim(raw.substr(0, raw.find('#')));
        if (const auto eq = l.find('='); eq != std::string::npos) {
            s.set(l.substr(0, eq), l.substr(eq + 1));
        }
    }
    return s;
}

std::string replaceSection(const std::string& text, const std::string& section, const Settings& s) {
    std::vector<std::string> v = lines(text);
    const auto [b, e] = sectionRange(v, section);
    std::vector<std::string> body{"[" + section + "]"};
    for (const auto& l : lines(s.toLines())) {
        body.push_back(l);
    }
    if (b == e) {
        if (!v.empty() && !trim(v.back()).empty()) {
            v.push_back("");
        }
        v.insert(v.end(), body.begin(), body.end());
    } else {
        v.erase(v.begin() + long(b), v.begin() + long(e));
        v.insert(v.begin() + long(b), body.begin(), body.end());
    }
    return join(v);
}

std::string removeSection(const std::string& text, const std::string& section) {
    std::vector<std::string> v = lines(text);
    const auto [b, e] = sectionRange(v, section);
    v.erase(v.begin() + long(b), v.begin() + long(e));
    return join(v);
}

std::filesystem::path settingsPath() {
    const auto d = xdgDir("XDG_CONFIG_HOME", ".config");
    return d.empty() ? d : d / "settings.conf";
}

std::filesystem::path stateDir() {
    return xdgDir("XDG_STATE_HOME", ".local/state");
}

} // namespace qv
