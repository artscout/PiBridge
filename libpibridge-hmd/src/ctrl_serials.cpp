// SPDX-License-Identifier: MIT
#include "pibridge/hmd/ctrl_serials.h"

#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <system_error>

namespace pibridge::hmd::ctrl_serials {

namespace {

const char* key(int slot) {
    switch (slot) {
    case 0: return "headset";
    case 2: return "left";
    case 3: return "right";
    case 5: return "optics";
    default: return nullptr;
    }
}

std::map<std::string, std::string> read_all() {
    std::map<std::string, std::string> m;
    std::ifstream f(path());
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream l(line);
        std::string k, v;
        if (l >> k >> v) {
            m[k] = v;
        }
    }
    return m;
}

} // namespace

std::filesystem::path path() {
    if (const char* s = std::getenv("XDG_STATE_HOME"); s != nullptr && *s != '\0') {
        return std::filesystem::path(s) / "pibridge" / "controller-serials";
    }
    const char* home = std::getenv("HOME");
    return std::filesystem::path(home != nullptr ? home : "/tmp") / ".local" / "state" / "pibridge" / "controller-serials";
}

bool store(int slot, const std::string& serial) {
    const char* k = key(slot);
    if (k == nullptr || serial.empty() || serial.find_first_of(" \t\n") != std::string::npos) {
        return false;
    }
    auto m = read_all();
    if (m[k] == serial) {
        return true;
    }
    m[k] = serial;
    const auto p = path();
    std::error_code ec;
    std::filesystem::create_directories(p.parent_path(), ec);
    const auto tmp = p.string() + ".tmp";
    {
        std::ofstream f(tmp, std::ios::trunc);
        for (const auto& [kk, v] : m) {
            f << kk << ' ' << v << '\n';
        }
        if (!f) {
            return false;
        }
    }
    std::filesystem::rename(tmp, p, ec);
    return !ec;
}

std::optional<std::string> load(int slot) {
    const char* k = key(slot);
    if (k == nullptr) {
        return std::nullopt;
    }
    const auto m = read_all();
    const auto it = m.find(k);
    return it == m.end() ? std::nullopt : std::optional<std::string>(it->second);
}

} // namespace pibridge::hmd::ctrl_serials
