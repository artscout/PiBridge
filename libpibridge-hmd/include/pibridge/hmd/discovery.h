// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax/src/discovery.rs (MIT OR Apache-2.0).
//
// sysfs discovery of the headset interfaces: every /sys/class/hidraw/hidrawN whose device uevent carries
// HID_ID=0003:000034A4:0000PPPP, mapped to its USB interface number (bInterfaceNumber of the parent directory).
// Reads sysfs only; never opens a device node.
#pragma once

#include "pibridge/hmd/proto.h"

#include <array>
#include <filesystem>
#include <optional>
#include <string>

namespace pibridge::hmd::discovery {

/// Where to look. Defaults to the real /sys/class/hidraw and /dev; tests point them at a temporary tree.
struct Roots {
    /// Directory holding the hidrawN entries.
    std::filesystem::path hidraw_class = "/sys/class/hidraw";
    /// Directory holding the device nodes.
    std::filesystem::path dev = "/dev";
    /// Defaults plus the PIBRIDGE_SYSFS_ROOT (replaces /sys) and PIBRIDGE_DEV_ROOT (replaces /dev) environment overrides.
    static Roots from_env();
};

using Interfaces = std::array<std::optional<std::filesystem::path>, NUM_IF>;

struct Found {
    const Model* model = nullptr;
    Interfaces nodes;
};

/// Device node of each interface 0..3 of the headset (the first model of MODELS that is present).
Interfaces find_interfaces(const Roots& roots);
/// The headset model on the bus and the node of each of its interfaces.
std::optional<Found> find_headset(const Roots& roots);
/// Bitmask of the interfaces found.
std::uint8_t mask(const Interfaces& found);
/// hidraw node of the display bridge MCU (ASCII log).
std::optional<std::filesystem::path> find_bridge_log(const Roots& roots);
/// USB device number of the headset, from /sys/bus/usb/devices (for "did it re-enumerate" diagnostics).
std::optional<std::string> usb_devnum(const std::filesystem::path& usb_devices = "/sys/bus/usb/devices");

} // namespace pibridge::hmd::discovery
