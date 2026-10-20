// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-device/src/{descriptor,module}.rs and devices/dream_air.rs (MIT OR Apache-2.0).
//
// The device description: what a headset is and can do, read from `devices/<id>.toml` (verbatim copies of the
// omarchyvr `device.toml` files). It carries the HID allowlist every report must pass.
#pragma once

#include "pibridge/hmd/allowlist.h"
#include "pibridge/hmd/proto.h"

#include <expected>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace pibridge::hmd {

/// How far a capability works with this stack.
enum class Support { Works, Experimental, None, Unknown };
std::string_view support_label(Support s);

/// Real (driven by the stack) or mock (placeholder).
enum class DeviceStatus { Real, Mock };

/// The onboarding step ids a description may list (pimax-device/src/onboarding.rs).
bool is_onboarding_step(std::string_view id);

struct Capabilities {
    Support display = Support::Unknown;
    /// Combined mode (w, h).
    std::optional<std::pair<std::uint32_t, std::uint32_t>> mode;
    std::vector<std::uint32_t> refresh_hz;
    std::optional<std::string> edid_vendor;
    std::optional<std::uint16_t> edid_product;
    std::vector<std::string> display_needs;
    Support tracking = Support::Unknown;
    /// "slam", "lighthouse", "inside_out", "imu3dof", ...
    std::string head_tracking;
    Support controllers = Support::Unknown;
    std::vector<std::uint8_t> controller_slots;
    /// "start_pair", "unknown", ...
    std::string pairing;
    Support eye_tracking = Support::Unknown;
    std::optional<std::pair<std::uint16_t, std::uint16_t>> eye_usb;
    Support ipd = Support::Unknown;
    bool ipd_motor = false;
    std::optional<std::pair<float, float>> ipd_range_mm;
    Support cameras = Support::Unknown;
    std::uint32_t camera_count = 0;
    Support audio = Support::Unknown;
    std::vector<std::string> audio_sink_match;
    std::vector<std::string> audio_source_match;
};

/// Controller hand.
enum class Hand { Left, Right };
std::uint8_t hand_slot(Hand h);

struct Descriptor {
    std::string id;
    std::string name;
    std::string vendor;
    DeviceStatus status = DeviceStatus::Mock;
    std::uint16_t usb_vid = 0;
    /// PIDs that identify this model (empty for mocks).
    std::vector<std::uint16_t> usb_pids;
    std::vector<std::uint16_t> usb_pid_candidates;
    std::uint8_t command_interface = 0;
    Capabilities caps;
    std::vector<std::string> calibration_paths;
    std::vector<std::string> onboarding;
    Allowlist allowlist;

    /// Parse a description (the text of a `.toml`).
    static std::expected<Descriptor, SpecError> parse(std::string_view src);
    /// Read and parse a file.
    static std::expected<Descriptor, SpecError> load(const std::filesystem::path& path);

    bool matches_usb(std::uint16_t vid, std::uint16_t pid) const;
    bool is_mock() const { return status == DeviceStatus::Mock; }

    /// Lens IPD motor report for `mm`, clamped to the description's range. Refused when the model has no motor.
    std::expected<std::vector<std::uint8_t>, Refused> ipd_report(float mm) const;
    /// Controller pairing report for one hand (`start_pair`); refused when the model does not list it.
    std::expected<std::vector<std::uint8_t>, Refused> pair_report(Hand hand) const;
    /// Keepalive for a refresh rate (0 = stop announcing).
    std::expected<std::vector<std::uint8_t>, Refused> keepalive_report(std::uint8_t hz) const;

private:
    std::expected<void, SpecError> validate() const;
};

/// Where the device descriptions live, first hit wins: $PIBRIDGE_DEVICES_DIR, the installed data dir
/// (<prefix>/share/pibridge/devices, compiled in), and ../share/pibridge/devices next to the executable.
std::vector<std::filesystem::path> devices_dirs();
/// Load `<dir>/<model.id>.toml` from the first of `dirs` that has it.
std::expected<Descriptor, SpecError> load_descriptor(const Model& model,
                                                     const std::vector<std::filesystem::path>& dirs = devices_dirs());
/// Load the allowlist of a model (shared, immutable); the error says what is missing.
std::expected<std::shared_ptr<const Allowlist>, SpecError>
load_allowlist(const Model& model, const std::vector<std::filesystem::path>& dirs = devices_dirs());

/// Result of matching a USB id against descriptions.
struct Detected {
    enum class Kind { Module, Possibly, None } kind = Kind::None;
    const Descriptor* desc = nullptr;
};
Detected detect(const std::vector<Descriptor>& descs, std::uint16_t vid, std::uint16_t pid);

} // namespace pibridge::hmd
