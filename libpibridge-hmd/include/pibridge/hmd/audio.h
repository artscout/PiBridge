// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-tools/src/audio.rs (MIT OR Apache-2.0).
//
// While the VR runtime runs, the headset's USB audio is the default output and input (PipeWire, through `pactl`);
// when it is gone, the previous defaults come back, unless the user picked another device in the meantime. The
// headset's nodes are found by name prefix (no serial in here). The original Crystal plays over DisplayPort instead:
// its port on the GPU audio card is found by its monitor name, and the card's profile is switched to it (and back).
// The pactl runner is injectable for tests.
#pragma once

#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pibridge::hmd::audio {

/// Name prefix of the headset's output node.
inline constexpr std::string_view SINK_PREFIX = "alsa_output.usb-Pimax_Corporation_";
/// Name prefix of the headset's input node.
inline constexpr std::string_view SOURCE_PREFIX = "alsa_input.usb-Pimax_Corporation_";
/// Name prefix of the original Crystal's input node (34A4:0012, "PIMAX Pimax_Crystal"): its USB audio is the
/// microphone only, its sound goes over DisplayPort (DpPort).
inline constexpr std::string_view CRYSTAL_SOURCE_PREFIX = "alsa_input.usb-PIMAX_";
/// The monitor name (ELD) of a Pimax headset on a GPU's HDMI / DisplayPort audio.
inline constexpr std::string_view DP_MONITOR_NAME = "Pimax";

/// The original Crystal's sound output: the GPU audio card's port whose monitor is a Pimax.
struct DpPort {
    std::string card;           // "alsa_card.pci-0000_01_00.1"
    std::string profile;        // the port's stereo profile, "output:hdmi-stereo-extra2"
    std::string active_profile; // the card's profile now
    /// The sink the profile gives: "alsa_output.pci-0000_01_00.1.hdmi-stereo-extra2".
    std::string sink() const;
    bool operator==(const DpPort&) const = default;
};
/// From `pactl list cards` (C locale): the first port whose device.product.name is DP_MONITOR_NAME, with the first of
/// its profiles; nullopt when there is none.
std::optional<DpPort> parse_dp_port(std::string_view list_cards);

/// Runs `pactl <args>`: its stdout (trimmed) on success, nullopt when it fails or cannot start.
using Runner = std::function<std::optional<std::string>(const std::vector<std::string>& args)>;
/// The real runner: posix_spawnp("pactl") in the C locale (its text is parsed), stdout captured, stderr discarded.
std::optional<std::string> run_pactl(const std::vector<std::string>& args);

/// The first node of `pactl list short sinks|sources` whose name starts with `prefix` (monitors left out).
std::optional<std::string> pick(std::string_view list, std::string_view prefix);

/// What was the default before the headset took over.
struct Before {
    std::optional<std::string> sink;
    std::optional<std::string> source;
    /// What the headset was set to (restored only while these are still the defaults).
    std::optional<std::string> our_sink;
    std::optional<std::string> our_source;
    /// The GPU audio card switched to the headset's DisplayPort profile, and its profile before (given back while the
    /// card still has ours).
    std::optional<std::string> card;
    std::optional<std::string> card_profile;
    std::optional<std::string> our_card_profile;
    bool operator==(const Before&) const = default;
};

/// Make the headset the default output and input (the original Crystal's output: its DisplayPort port's profile
/// first); a log line and what to restore later.
std::pair<std::string, Before> to_headset(const Runner& pactl = run_pactl);
/// The headset's output node (`pactl list short sinks`; the original Crystal: its DisplayPort sink, when its profile is
/// active), nullopt when there is none.
std::optional<std::string> headset_sink(const Runner& pactl = run_pactl);
/// A sink's volume as pactl takes it back ("42%", the first channel of `pactl get-sink-volume`); nullopt on failure.
std::optional<std::string> sink_volume(const std::string& sink, const Runner& pactl = run_pactl);
/// `pactl set-sink-volume <sink> <volume>`; false on failure.
bool set_sink_volume(const std::string& sink, const std::string& volume, const Runner& pactl = run_pactl);
/// The first "NN%" in pactl's volume text, nullopt when there is none.
std::optional<std::string> parse_volume(std::string_view text);

/// A volume button change, from the firmware log lines on IF3: "<info> app: vol up 1" (pressed), "... vol up 0"
/// (released), the same with "vol down".
struct VolumeButton {
    int dir = 0;          // +1 up, -1 down, 0 = not a volume button line
    bool pressed = false; // true: pressed, false: released
    bool operator==(const VolumeButton&) const = default;
};
VolumeButton volume_button(std::string_view log_line);
/// One volume step of the buttons, in percent.
inline constexpr int VOLUME_STEP = 1;
/// `pactl set-sink-volume` to the current volume +- `steps` * VOLUME_STEP, kept within 0..100 %; the new volume
/// ("45%"), nullopt when the volume could not be read or set.
std::optional<std::string> step_sink_volume(const std::string& sink, int steps, const Runner& pactl = run_pactl);

/// Give the defaults back (each only if the headset is still the default, i.e. the user did not change it; the card
/// profile first, so the previous sink exists again).
std::string restore(const Before& b, const Runner& pactl = run_pactl);

} // namespace pibridge::hmd::audio
