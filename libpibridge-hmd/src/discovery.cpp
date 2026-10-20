// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax/src/discovery.rs (MIT OR Apache-2.0).
#include "pibridge/hmd/discovery.h"
#include "pibridge_env.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <vector>

namespace pibridge::hmd::discovery {
namespace fs = std::filesystem;

namespace {

std::optional<std::string> read_file(const fs::path& p) {
    std::ifstream f(p);
    if (!f) {
        return std::nullopt;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

std::string trim(std::string s) {
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) {
        return {};
    }
    const auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

std::string hid_id(std::uint16_t vid, std::uint16_t pid) {
    char b[48];
    std::snprintf(b, sizeof b, "HID_ID=0003:%08X:%08X", vid, pid);
    return b;
}

std::vector<std::string> entries(const Roots& roots) {
    std::vector<std::string> v;
    std::error_code ec;
    for (fs::directory_iterator it(roots.hidraw_class, ec), end; !ec && it != end; it.increment(ec)) {
        const std::string n = it->path().filename().string();
        if (n.starts_with("hidraw")) {
            v.push_back(n);
        }
    }
    std::sort(v.begin(), v.end());
    return v;
}

Interfaces interfaces_of(const Roots& roots, const std::vector<std::string>& names, std::uint16_t pid) {
    Interfaces out{};
    const std::string want = hid_id(VID, pid);
    for (const auto& name : names) {
        const fs::path dev = roots.hidraw_class / name / "device";
        const auto uevent = read_file(dev / "uevent");
        if (!uevent || uevent->find(want) == std::string::npos) {
            continue;
        }
        std::error_code ec;
        const fs::path real = fs::canonical(dev, ec);
        if (ec || !real.has_parent_path()) {
            continue;
        }
        const auto num = read_file(real.parent_path() / "bInterfaceNumber");
        if (!num) {
            continue;
        }
        const std::string t = trim(*num);
        char* end = nullptr;
        const unsigned long n = std::strtoul(t.c_str(), &end, 16);
        if (t.empty() || end == nullptr || *end != '\0') {
            continue;
        }
        if (n < NUM_IF) {
            out[n] = roots.dev / name;
        }
    }
    return out;
}

} // namespace

Roots Roots::from_env() {
    Roots r;
    if (const char* s = pibridge::env("SYSFS_ROOT"); s && *s) {
        r.hidraw_class = fs::path(s) / "class/hidraw";
    }
    if (const char* d = pibridge::env("DEV_ROOT"); d && *d) {
        r.dev = d;
    }
    return r;
}

std::uint8_t mask(const Interfaces& found) {
    std::uint8_t m = 0;
    for (std::size_t i = 0; i < found.size(); i++) {
        if (found[i]) {
            m = static_cast<std::uint8_t>(m | (1u << i));
        }
    }
    return m;
}

std::optional<Found> find_headset(const Roots& roots) {
    const auto names = entries(roots);
    for (const auto& m : MODELS) {
        auto f = interfaces_of(roots, names, m.pid);
        if (mask(f) != 0) {
            return Found{&m, std::move(f)};
        }
    }
    return std::nullopt;
}

Interfaces find_interfaces(const Roots& roots) {
    auto f = find_headset(roots);
    return f ? f->nodes : Interfaces{};
}

std::optional<fs::path> find_bridge_log(const Roots& roots) {
    const auto names = entries(roots);
    for (const auto& m : MODELS) {
        const std::string want = hid_id(VID, m.pid_bridge);
        for (const auto& name : names) {
            const auto u = read_file(roots.hidraw_class / name / "device/uevent");
            if (u && u->find(want) != std::string::npos) {
                return roots.dev / name;
            }
        }
    }
    return std::nullopt;
}

std::optional<std::string> usb_devnum(const fs::path& usb_devices) {
    std::error_code ec;
    for (fs::directory_iterator it(usb_devices, ec), end; !ec && it != end; it.increment(ec)) {
        const fs::path p = it->path();
        const auto rd = [&](const char* f) -> std::optional<std::string> {
            auto s = read_file(p / f);
            if (!s) {
                return std::nullopt;
            }
            return trim(*s);
        };
        const auto vid = rd("idVendor");
        const auto pid = rd("idProduct");
        if (vid != "34a4" || !pid) {
            continue;
        }
        char* end2 = nullptr;
        const unsigned long v = std::strtoul(pid->c_str(), &end2, 16);
        if (pid->empty() || *end2 != '\0' || v > 0xffff) {
            continue;
        }
        if (model_by_pid(static_cast<std::uint16_t>(v))) {
            return rd("devnum");
        }
    }
    return std::nullopt;
}

} // namespace pibridge::hmd::discovery
