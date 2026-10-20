// SPDX-License-Identifier: MIT
#include "backgrounds.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace pibridge::home::bg {

namespace fs = std::filesystem;

const std::vector<Picture>& catalogue() {
    static const std::vector<Picture> k{
        {"wildflower_field", "flowers", "Wildflower Field", "Alexander Scholten", "CC0"},
        {"sunflowers", "flowers", "Sunflowers", "Sergej Majboroda", "CC0"},
        {"spiaggia_di_mondello", "ocean", "Beach", "Andreas Mischok", "CC0"},
        {"blouberg_sunrise_2", "ocean", "Blouberg Sunrise", "Greg Zaal, Rico Cilliers", "CC0"},
        {"venice_sunset", "sunset", "Venice Sunset", "Greg Zaal", "CC0"},
        {"belfast_sunset", "sunset", "Belfast Sunset", "Dimitrios Savva, Greg Zaal", "CC0"},
        {"alps_field", "mountains", "Alps Field", "Andreas Mischok", "CC0"},
        {"table_mountain_1_puresky", "mountains", "Table Mountain", "Greg Zaal, Jarod Guest", "CC0"},
        {"autumn_forest_04", "forest", "Autumn Forest", "Alexander Scholten", "CC0"},
        {"birchwood", "forest", "Birchwood", "Sergej Majboroda", "CC0"},
        {"lakeside_sunrise", "lake", "Lakeside Sunrise", "Greg Zaal, Jarod Guest", "CC0"},
        {"goegap", "desert", "Goegap", "Greg Zaal", "CC0"},
        {"kloppenheim_02_puresky", "night", "Starry Night", "Greg Zaal, Jarod Guest", "CC0"},
        {"rogland_moonlit_night", "night", "Moonlit Night", "Greg Zaal", "CC0"},
        {"shanghai_bund", "city", "Shanghai Bund", "Greg Zaal", "CC0"},
        {"golden_bay", "city", "Golden Bay", "Greg Zaal", "CC0"},
    };
    return k;
}

const Picture* find(std::string_view id) {
    const auto& c = catalogue();
    const auto it = std::ranges::find_if(c, [&](const Picture& p) { return p.id == id; });
    return it == c.end() ? nullptr : &*it;
}

std::string apiUrl(std::string_view id) { return "https://api.polyhaven.com/files/" + std::string(id); }
std::string directJpgUrl(std::string_view id) {
    return "https://dl.polyhaven.org/file/ph-assets/HDRIs/extra/Tonemapped%20JPG/" + std::string(id) + ".jpg";
}
std::string pageUrl(std::string_view id) { return "https://polyhaven.com/a/" + std::string(id); }

Choice parseChoice(std::string_view v) {
    if (v == "none" || v == "off-dark") {
        return {Kind::None, {}};
    }
    if (v == "dusk") {
        return {Kind::Dusk, {}};
    }
    if (!v.empty() && v != "passthrough" && v != "off" && find(v) != nullptr) {
        return {Kind::Picture, std::string(v)};
    }
    return {Kind::Passthrough, {}};
}

std::string choiceValue(const Choice& c) {
    switch (c.kind) {
    case Kind::None: return "none";
    case Kind::Dusk: return "dusk";
    case Kind::Picture: return c.id;
    case Kind::Passthrough: break;
    }
    return "passthrough";
}

namespace {
fs::path envDir(const char* var, const char* homeSub) {
    if (const char* x = std::getenv(var); x && *x) {
        return fs::path(x);
    }
    const char* h = std::getenv("HOME");
    return fs::path(h && *h ? h : "/tmp") / homeSub;
}

std::string_view trim(std::string_view s) {
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) {
        s.remove_prefix(1);
    }
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r')) {
        s.remove_suffix(1);
    }
    return s;
}
} // namespace

fs::path libraryDir() { return envDir("XDG_DATA_HOME", ".local/share") / "pibridge" / "backgrounds"; }
fs::path picturePath(const fs::path& dir, std::string_view id) { return dir / (std::string(id) + ".ppm"); }
fs::path thumbPath(const fs::path& dir, std::string_view id) { return dir / "thumbs" / (std::string(id) + ".png"); }
bool downloaded(const fs::path& dir, std::string_view id) {
    std::error_code ec;
    return fs::is_regular_file(picturePath(dir, id), ec) && fs::file_size(picturePath(dir, id), ec) > 1024;
}

fs::path confPath() { return envDir("XDG_CONFIG_HOME", ".config") / "pibridge" / "pibridge.conf"; }

std::optional<std::string> confValue(std::string_view text, std::string_view key) {
    std::optional<std::string> out;
    std::size_t pos = 0;
    while (pos <= text.size()) {
        const std::size_t nl = text.find('\n', pos);
        const std::string_view line = trim(text.substr(pos, nl == std::string_view::npos ? std::string_view::npos : nl - pos));
        pos = nl == std::string_view::npos ? text.size() + 1 : nl + 1;
        if (line.empty() || line.front() == '#') {
            continue;
        }
        const std::size_t eq = line.find('=');
        if (eq == std::string_view::npos || trim(line.substr(0, eq)) != key) {
            continue;
        }
        out = std::string(trim(line.substr(eq + 1)));
    }
    return out;
}

bool confFlag(std::string_view text, std::string_view key, bool def) {
    const auto v = confValue(text, key);
    if (!v) {
        return def;
    }
    std::string l = *v;
    std::ranges::transform(l, l.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (l == "on" || l == "1" || l == "true" || l == "yes") {
        return true;
    }
    if (l == "off" || l == "0" || l == "false" || l == "no") {
        return false;
    }
    return def;
}

HomeSettings parseSettings(std::string_view t) {
    HomeSettings s;
    s.background = parseChoice(confValue(t, kKeyBackground).value_or(""));
    s.panel = confFlag(t, kKeyPanel, true);
    return s;
}

HomeSettings loadSettings(const fs::path& conf) {
    std::ifstream in(conf, std::ios::binary);
    if (!in) {
        return {};
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return parseSettings(ss.str());
}

} // namespace pibridge::home::bg
