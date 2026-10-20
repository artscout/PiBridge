// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/power.rs (MIT OR Apache-2.0).
#include "pibridge/eye/power.h"

#include "sysfs.h"

#include <algorithm>
#include <format>

namespace pibridge::eye::power {

namespace fs = std::filesystem;

std::array<std::uint8_t, 64> enable_report() {
    std::array<std::uint8_t, 64> r{};
    const std::uint8_t head[6] = {0x58, 0x05, 0x3c, 0x05, 0x00, 0x01};
    std::copy(std::begin(head), std::end(head), r.begin());
    return r;
}

Result<void> send_enable(HeadsetCommands& hid) {
    return hid.send(EYE_POWER_COMMAND);
}

std::vector<UsbId> find_usb(const fs::path& root, std::uint16_t vid, std::uint16_t pid) {
    const std::string v = std::format("{:04x}", vid);
    const std::string p = std::format("{:04x}", pid);
    std::vector<UsbId> out;
    std::error_code ec;
    fs::directory_iterator it(root, ec);
    if (ec) {
        return out;
    }
    for (const auto& e : it) {
        const fs::path d = e.path();
        if (detail::read_trim(d / "idVendor") == v && detail::read_trim(d / "idProduct") == p) {
            const auto devnum = detail::parse_uint<std::uint32_t>(detail::read_trim(d / "devnum"), 10);
            out.push_back(UsbId{e.path().filename().string(), devnum.value_or(0)});
        }
    }
    std::ranges::sort(out, [](const UsbId& a, const UsbId& b) { return a.name < b.name; });
    return out;
}

std::optional<UsbId> headset(const fs::path& root) {
    for (const auto pid : HEADSET_PIDS) {
        auto v = find_usb(root, 0x34a4, pid);
        if (!v.empty()) {
            return v.front();
        }
    }
    return std::nullopt;
}

bool eye_chip_present(const fs::path& root) {
    return !find_usb(root, 0x2104, 0x0561).empty();
}

std::optional<fs::path> headset_command_node(const fs::path& hidraw_class, const fs::path& dev) {
    std::vector<std::string> names;
    std::error_code ec;
    fs::directory_iterator it(hidraw_class, ec);
    if (ec) {
        return std::nullopt;
    }
    for (const auto& e : it) {
        auto n = e.path().filename().string();
        if (n.starts_with("hidraw")) {
            names.push_back(std::move(n));
        }
    }
    std::ranges::sort(names);
    for (const auto& n : names) {
        const fs::path d = hidraw_class / n / "device";
        const auto uevent = detail::read_trim(d / "uevent");
        if (!uevent) {
            continue;
        }
        const bool ours = std::ranges::any_of(HEADSET_PIDS, [&](std::uint16_t pid) {
            return uevent->find(std::format("HID_ID=0003:000034A4:0000{:04X}", pid)) != std::string::npos;
        });
        if (!ours) {
            continue;
        }
        std::error_code cec;
        const fs::path real = fs::canonical(d, cec);
        if (cec) {
            continue;
        }
        if (detail::read_trim(real.parent_path() / "bInterfaceNumber") == std::string("00")) {
            return dev / n;
        }
    }
    return std::nullopt;
}

Gate EnableGate::step(Clock::time_point now, const std::optional<UsbId>& h, bool chip_present) {
    if (!h) {
        absent_since_.reset();
        headset_seen_.reset();
        return Gate::idle();
    }
    if (headset_seen_ != h) {
        // a new enumeration of the headset: the chip's absence is counted from now
        headset_seen_ = h;
        absent_since_.reset();
    }
    if (chip_present) {
        absent_since_.reset();
        return Gate::idle();
    }
    if (!absent_since_) {
        absent_since_ = now;
    }
    const auto since = *absent_since_;
    if (sent_for_ == h) {
        return Gate::already_sent();
    }
    if (now - since <= ABSENT_BEFORE_ENABLE) {
        return Gate::waiting();
    }
    sent_for_ = h;
    return Gate::send(*h);
}

} // namespace pibridge::eye::power
