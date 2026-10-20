// SPDX-License-Identifier: MIT
// Copied from artscout-qv-vulkan-linux layer/src/settings.h (MIT).
// The settings file, shared by the layer and the settings app:
//   $XDG_CONFIG_HOME/qv-vulkan/settings.conf   (default ~/.config/qv-vulkan/settings.conf)
//
//   peripheral_density=0.40   pixels of the wide views, per pixel the runtime recommends (0.05 .. 1.5)
//   focus_density=1.00        pixels of the focus views and of the composed image (0.5 .. 2.0)
//   tracking=eye              eye: the focus follows the gaze when the runtime has eye tracking, else fixed | fixed
//   fixed_width=0.50          size of the focus without eye tracking, share of the eye's field (tangent span)
//   fixed_height=0.45
//   gaze_width=0.35           size of the focus that follows the gaze
//   gaze_height=0.35
//   offset_x_deg=0            move the focus right (+) or left (-), degrees, both eyes
//   offset_y_deg=0            move the focus up (+) or down (-)
//   transition=0.20           blend band at the focus edge, share of the focus size on each side (0 .. 0.5)
//   edge_opacity=0.50         share of the focus already at the outer edge of the band (0 = fades from the
//                             periphery, 1 = hard edge; Pimax: "focus transition opacity" 0.5)
//   sharpen=0.60              adaptive sharpening of the focus (0 off .. 1)
//   peripheral_filter=bicubic upscaling of the wide views: bicubic | bilinear
//   gaze_filter=on            One-Euro smoothing of the gaze with a saccade snap
//   gaze_hold_ms=600          keep the last gaze this long when it drops out (blinks)
//   debug_overlay=off         tint the focus area, outline the blend band, mark the gaze
//
//   [app:Falcon BMS]          lines below apply to apps whose OpenXR application name contains this text
//   [exe:Falcon_BMS]          ... or whose process name contains this text
//
// Unknown keys and bad values are ignored (the default stays), so a hand-edited file never breaks a game.
// The layer reads the file when the game creates its OpenXR instance and again whenever it changes, so the
// settings app acts on a running game (except the two densities: the game sizes its swapchains once).
#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace qv {

enum class Tracking { Eye, Fixed };
enum class Filter { Bicubic, Bilinear };

struct Settings {
    float peripheralDensity = 0.4f;
    float focusDensity = 1.0f;
    Tracking tracking = Tracking::Eye;
    std::array<float, 2> fixedSize{0.5f, 0.45f};
    std::array<float, 2> gazeSize{0.35f, 0.35f};
    std::array<float, 2> offsetDeg{0.f, 0.f};
    float transition = 0.2f;
    float edgeOpacity = 0.5f;
    float sharpen = 0.6f;
    Filter peripheralFilter = Filter::Bicubic;
    bool gazeFilter = true;
    uint32_t gazeHoldMs = 600;
    bool debugOverlay = false;

    bool operator==(const Settings&) const = default;

    // Apply one key=value pair; false when the key or the value is not understood.
    bool set(const std::string& key, const std::string& value);
    // The global key=value lines of these settings.
    std::string toLines() const;
};

// Which app the settings are resolved for.
struct AppId {
    std::string name; // XrApplicationInfo::applicationName
    std::string exe;  // /proc/self/comm
    static AppId current(const std::string& applicationName);
};

// The settings for `app`: the global lines, then the matching [app:]/[exe:] sections in file order.
Settings parseSettings(const std::string& text, const AppId& app);
// Read the file; missing or unreadable: the defaults.
Settings loadSettings(const std::filesystem::path& path, const AppId& app);
// The file text with its global part replaced and every [app:]/[exe:] section kept as it was.
std::string replaceGlobal(const std::string& oldText, const Settings& s);
// Write the global part atomically (temp file + rename), keeping the per-app sections.
bool saveSettings(const std::filesystem::path& path, const Settings& s);

// Per-game profiles: the [app:NAME] / [exe:NAME] sections of the file ("app:Falcon BMS", "exe:Falcon_BMS").
std::vector<std::string> sectionNames(const std::string& text);
// The settings a profile gives: the global lines, then that section's lines.
Settings parseSection(const std::string& text, const std::string& section);
// The text with the section's lines replaced by the full settings `s` (the section is added when missing).
std::string replaceSection(const std::string& text, const std::string& section, const Settings& s);
// The text without the section.
std::string removeSection(const std::string& text, const std::string& section);

// $XDG_CONFIG_HOME/qv-vulkan/settings.conf (or ~/.config/...); empty when neither is set.
std::filesystem::path settingsPath();
// $XDG_STATE_HOME/qv-vulkan (or ~/.local/state/qv-vulkan): the log and the status file.
std::filesystem::path stateDir();

} // namespace qv
