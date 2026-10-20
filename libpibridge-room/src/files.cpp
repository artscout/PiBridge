// SPDX-License-Identifier: MIT
#include "pibridge/room/files.h"

#include "pibridge/room/json.h"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace pibridge::room {

namespace {

std::string envOr(const char* name) {
    const char* v = std::getenv(name);
    return v != nullptr ? std::string(v) : std::string();
}

bool isDir(const std::string& p) {
    std::error_code ec;
    return fs::is_directory(p, ec);
}

std::string clean(const std::string& p) {
    std::error_code ec;
    fs::path c = fs::weakly_canonical(p, ec);
    if (ec) {
        c = fs::path(p).lexically_normal();
    }
    std::string s = c.string();
    while (s.size() > 1 && s.back() == '/') {
        s.pop_back();
    }
    return s;
}

} // namespace

Env Env::fromProcess() { return {envOr("HOME"), envOr("XDG_CONFIG_HOME"), envOr("XDG_DATA_HOME")}; }

std::string Env::configHome() const { return xdgConfigHome.empty() ? home + "/.config" : xdgConfigHome; }

std::string Env::dataHome() const { return xdgDataHome.empty() ? home + "/.local/share" : xdgDataHome; }

std::vector<std::string> steamConfigDirs(const Env& env) {
    std::vector<std::string> dirs;
    if (auto t = readFile(env.configHome() + "/openvr/openvrpaths.vrpath")) {
        if (auto j = parseJson(*t)) {
            for (const Json& c : j->get("config").items()) {
                if (c.isString()) {
                    dirs.push_back(c.str());
                }
            }
        }
    }
    dirs.push_back(env.dataHome() + "/Steam/config");
    if (!env.home.empty()) {
        dirs.push_back(env.home + "/.steam/steam/config");
    }
    std::vector<std::string> out;
    for (const auto& d : dirs) {
        if (!isDir(d)) {
            continue;
        }
        const std::string c = clean(d);
        if (std::ranges::find(out, c) == out.end()) {
            out.push_back(c);
        }
    }
    return out;
}

std::string roomsDir(const Env& env) { return env.configHome() + "/pibridge/rooms"; }

std::expected<std::string, std::string> readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        return std::unexpected(std::format("cannot read {}", path));
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

std::expected<void, std::string> writeFileAtomic(const std::string& path, const std::string& data) {
    const std::string tmp = path + ".tmp";
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        if (!f) {
            return std::unexpected(std::format("cannot write {}", path));
        }
        f.write(data.data(), static_cast<std::streamsize>(data.size()));
        f.flush();
        if (!f) {
            std::error_code ec;
            fs::remove(tmp, ec);
            return std::unexpected(std::format("cannot write {}", path));
        }
    }
    std::error_code ec;
    fs::rename(tmp, path, ec);
    if (ec) {
        fs::remove(tmp, ec);
        return std::unexpected(std::format("cannot replace {}", path));
    }
    return {};
}

std::string backupStamp(std::time_t t) {
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof buf, "%Y%m%d-%H%M%S", &tm);
    return buf;
}

std::vector<std::string> backupsOf(const std::string& path) {
    const fs::path p(path);
    const std::string prefix = p.filename().string() + ".bak-";
    std::vector<std::string> out;
    std::error_code ec;
    for (const auto& e : fs::directory_iterator(p.parent_path().empty() ? fs::path(".") : p.parent_path(), ec)) {
        const std::string n = e.path().filename().string();
        if (n.starts_with(prefix) && !n.ends_with(".tmp")) {
            out.push_back(e.path().string());
        }
    }
    std::ranges::sort(out, [](const std::string& a, const std::string& b) {
        return a.substr(a.rfind(".bak-")) < b.substr(b.rfind(".bak-"));
    });
    return out;
}

} // namespace pibridge::room
