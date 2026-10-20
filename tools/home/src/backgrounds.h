// SPDX-License-Identifier: MIT
// The VR home's backgrounds (docs/home.md "Backgrounds"): the choice in pibridge.conf, the curated list of free 360-degree
// pictures and where they live. Shared by pibridge-home (which shows the choice) and the app (Home page: the thumbnails,
// the download, the switch). Pure C++, no Qt, no network: the app downloads.
//
// Choice (`home_background` in pibridge.conf, docs/CONVENTIONS.md):
//   passthrough   the room through the headset's cameras (default; the same as "Off" for a picture)
//   none          the dark home: floor grid and horizon only
//   dusk          a built-in picture drawn in code (no file, always there)
//   <id>          a picture of the list below, once downloaded: <dir>/<id>.ppm (equirect 2:1, 8-bit RGB)
// The pictures are never in git: they are fetched on first use into ~/.local/share/pibridge/backgrounds (the list's
// pictures are all Poly Haven CC0, which would allow bundling, but 8K panoramas are 10-30 MB each).
#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::home::bg {

struct Picture {
    std::string id;      // Poly Haven asset id
    std::string theme;   // flowers, ocean, sunset, mountains, forest, desert, night, city, lake
    std::string title;
    std::string author;
    std::string licence; // all CC0
};
/// The curated list (from omarchyvr's rust/omarchyvr-backgrounds/sources.toml and docs/BACKGROUNDS.md: Poly Haven only,
/// CC0; Wikimedia's CC BY / BY-SA pictures need a credit and are left out).
const std::vector<Picture>& catalogue();
const Picture* find(std::string_view id);
/// Poly Haven's API entry that names the tonemapped 8K JPG (`tonemapped.url`) and the usual direct URL of that file.
std::string apiUrl(std::string_view id);
std::string directJpgUrl(std::string_view id);
std::string pageUrl(std::string_view id);

enum class Kind { Passthrough, None, Dusk, Picture };
struct Choice {
    Kind kind = Kind::Passthrough;
    std::string id; // Picture
    friend bool operator==(const Choice&, const Choice&) = default;
};
/// The value of home_background (empty or unknown: passthrough).
Choice parseChoice(std::string_view v);
std::string choiceValue(const Choice& c);

/// $XDG_DATA_HOME/pibridge/backgrounds (or ~/.local/share/pibridge/backgrounds).
std::filesystem::path libraryDir();
/// The picture pibridge-home loads: <dir>/<id>.ppm; the app's thumbnail: <dir>/thumbs/<id>.png.
std::filesystem::path picturePath(const std::filesystem::path& dir, std::string_view id);
std::filesystem::path thumbPath(const std::filesystem::path& dir, std::string_view id);
bool downloaded(const std::filesystem::path& dir, std::string_view id);

// ---- pibridge.conf (the keys pibridge-home reads; the app writes them through its Conf)
inline constexpr char kKeyBackground[] = "home_background";
inline constexpr char kKeyPanel[] = "home_panel"; // the PiBridge window in the home: on (default) / off
/// $XDG_CONFIG_HOME/pibridge/pibridge.conf (or ~/.config/pibridge/pibridge.conf).
std::filesystem::path confPath();
/// The value of the last live `key=` line of `text` (the daemons' rules: trimmed, '#' lines ignored).
std::optional<std::string> confValue(std::string_view text, std::string_view key);
bool confFlag(std::string_view text, std::string_view key, bool def);

struct HomeSettings {
    Choice background;
    bool panel = true;
    friend bool operator==(const HomeSettings&, const HomeSettings&) = default;
};
HomeSettings parseSettings(std::string_view confText);
/// Read the file (missing: the defaults).
HomeSettings loadSettings(const std::filesystem::path& conf);

} // namespace pibridge::home::bg
