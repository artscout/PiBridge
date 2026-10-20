// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/power.rs (MIT OR Apache-2.0).
//
// Powering the eye-tracker module: the headset MCU (34a4:0044) switches the Tobii EyeChip (2104:0561) on when the host
// sends one HID report. Windows sends exactly this report (`analysis/init-sequence.md`, poweron capture 20261013,
// t = 61.645 s, log `eSpiCmdeyetkctrl`, reply `4c 04 3c 05 00 00`); the chip enumerates about 1.6 s later. Without
// it, a headset that was power-cycled under Linux leaves the eye module off (seen 14 Oct 13:56).
//
// Only the enable report is ever sent. The disable variant (`.. 00 00`) was never captured and is not in this code.
// Nothing here writes settings or flash: it is the same one-report request Pimax's service sends.
//
// The Crystal Super (34a4:0040) takes the same report on its interface 0: Pimax Play sent exactly these bytes in the
// Crystal Super capture (take cold2 of 2026-10-17, t = 27.543 s).
//
// This library does not write HID itself: the report goes out through HeadsetCommands, an interface that sends one
// named command of the device's allowlist (`devices/*.toml`, id `eye_power_on`). Phase 2 implements it with libpibridge-hmd.
//
// EnableGate is the rate limit used by the live tracker: at most one report per enumeration of the headset
// (identified by its USB device number), and only when the headset is present and the EyeChip has been absent for
// more than ABSENT_BEFORE_ENABLE.
#pragma once

#include "pibridge/eye/error.h"
#include "pibridge/eye/transport.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::eye::power {

/// The captured eye-module enable request, as a 64-byte HID output report (`58 05 3c 05 00 01`, zero padded). Kept
/// as the reference the allowlist entry `eye_power_on` must match; this library never writes it to a device itself.
std::array<std::uint8_t, 64> enable_report();

/// Id of the enable command in the device allowlist (`devices/dream-air.toml`, `devices/crystal-super.toml`).
inline constexpr std::string_view EYE_POWER_COMMAND = "eye_power_on";

/// The EyeChip must have been missing this long (with the headset present) before the enable is sent.
inline constexpr std::chrono::seconds ABSENT_BEFORE_ENABLE{5};

/// Sends one named, allowlisted HID command to the headset command interface (interface 0). Implemented in phase 2
/// over libpibridge-hmd's allowlist gate; tests use a recording fake. Nothing else in this library talks to the headset.
class HeadsetCommands {
public:
    virtual ~HeadsetCommands() = default;
    /// Send the allowlist entry `id` once. Unknown ids must be refused by the implementation.
    virtual Result<void> send(std::string_view id) = 0;
};

/// Send the eye-module enable (`eye_power_on`) through `hid`.
Result<void> send_enable(HeadsetCommands& hid);

/// A USB device seen in sysfs.
struct UsbId {
    /// sysfs name, e.g. `1-5.2.4.1`.
    std::string name;
    /// `devnum` (changes on every enumeration).
    std::uint32_t devnum = 0;
    bool operator==(const UsbId&) const = default;
};

/// Devices with this VID:PID under `root` (normally `/sys/bus/usb/devices`), sorted by name.
std::vector<UsbId> find_usb(const std::filesystem::path& root, std::uint16_t vid, std::uint16_t pid);

/// Headset HID chips whose interface 0 takes the eye-module enable: the Dream Air (0044), the Crystal Super (0040).
inline constexpr std::array<std::uint16_t, 2> HEADSET_PIDS = {0x0044, 0x0040};

/// The headset (34a4:0044 or 34a4:0040), if enumerated.
std::optional<UsbId> headset(const std::filesystem::path& root);

/// Whether an EyeChip (2104:0561) is enumerated.
bool eye_chip_present(const std::filesystem::path& root);

/// hidraw node of interface 0 (the command channel) of the headset (34a4:0044 or 0040): `hidraw_class` is normally
/// `/sys/class/hidraw`, `dev` is `/dev`. Reads sysfs only; nothing is opened.
std::optional<std::filesystem::path> headset_command_node(const std::filesystem::path& hidraw_class,
                                                          const std::filesystem::path& dev);

/// What EnableGate::step() decided.
struct Gate {
    enum class Kind {
        Idle,        ///< the EyeChip is there, or the headset is not: nothing to do
        Waiting,     ///< the chip has been absent for less than ABSENT_BEFORE_ENABLE
        Send,        ///< send the enable now, to this headset enumeration (`target`)
        AlreadySent, ///< already sent for this headset enumeration
    };
    Kind kind = Kind::Idle;
    UsbId target;

    static Gate idle() { return {}; }
    static Gate waiting() { return {Kind::Waiting, {}}; }
    static Gate send(UsbId id) { return {Kind::Send, std::move(id)}; }
    static Gate already_sent() { return {Kind::AlreadySent, {}}; }
    bool operator==(const Gate&) const = default;
};

/// Rate limit of the automatic enable (see the file comment).
class EnableGate {
public:
    /// Decide from the current bus state; Send is returned at most once per headset enumeration.
    Gate step(Clock::time_point now, const std::optional<UsbId>& headset, bool chip_present);

private:
    std::optional<Clock::time_point> absent_since_;
    std::optional<UsbId> sent_for_;
    std::optional<UsbId> headset_seen_;
};

} // namespace pibridge::eye::power
