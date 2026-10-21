// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-record/src/settings.rs (MIT OR Apache-2.0).
#include "pibridge/record/settings.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <ctime>
#include <format>
#include <fstream>
#include <sstream>
#include <system_error>

namespace pibridge::record {

namespace fs = std::filesystem;

std::string_view name(Eyes e) { return e == Eyes::Left ? "left" : "both"; }
std::string_view name(View v) {
    switch (v) {
    case View::LeftEye: return "left";
    case View::RightEye: return "right";
    case View::BothEyes: return "both";
    }
    return "left";
}
std::string_view name(Quality q) {
    switch (q) {
    case Quality::Low: return "low";
    case Quality::Medium: return "medium";
    case Quality::High: return "high";
    }
    return "high";
}
std::string_view name(Codec c) { return c == Codec::H264 ? "h264" : "hevc"; }
std::string_view name(EncoderPref p) {
    switch (p) {
    case EncoderPref::Auto: return "auto";
    case EncoderPref::Nvenc: return "nvenc";
    case EncoderPref::Vaapi: return "vaapi";
    case EncoderPref::Cpu: return "cpu";
    }
    return "auto";
}

std::optional<Eyes> parseEyes(std::string_view s) {
    if (s == "left" || s == "one" || s == "1") {
        return Eyes::Left;
    }
    if (s == "both" || s == "two" || s == "2" || s == "sbs") {
        return Eyes::Both;
    }
    return std::nullopt;
}
std::optional<View> parseView(std::string_view s) {
    if (s == "left" || s == "left-eye") {
        return View::LeftEye;
    }
    if (s == "right" || s == "right-eye") {
        return View::RightEye;
    }
    if (s == "both" || s == "both-eyes" || s == "sbs") {
        return View::BothEyes;
    }
    return std::nullopt;
}
std::optional<Quality> parseQuality(std::string_view s) {
    if (s == "low") {
        return Quality::Low;
    }
    if (s == "medium" || s == "med") {
        return Quality::Medium;
    }
    if (s == "high") {
        return Quality::High;
    }
    return std::nullopt;
}
std::optional<Codec> parseCodec(std::string_view s) {
    if (s == "h264" || s == "avc") {
        return Codec::H264;
    }
    if (s == "hevc" || s == "h265") {
        return Codec::Hevc;
    }
    return std::nullopt;
}
std::optional<EncoderPref> parseEncoder(std::string_view s) {
    if (s == "auto") {
        return EncoderPref::Auto;
    }
    if (s == "nvenc" || s == "gpu") {
        return EncoderPref::Nvenc;
    }
    if (s == "vaapi" || s == "va") {
        return EncoderPref::Vaapi;
    }
    if (s == "cpu" || s == "x264" || s == "x265" || s == "software") {
        return EncoderPref::Cpu;
    }
    return std::nullopt;
}

namespace {

std::string_view trim(std::string_view s) {
    const auto ws = [](char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\v' || c == '\f'; };
    while (!s.empty() && ws(s.front())) {
        s.remove_prefix(1);
    }
    while (!s.empty() && ws(s.back())) {
        s.remove_suffix(1);
    }
    return s;
}

std::string_view trimQuotes(std::string_view s) {
    while (!s.empty() && s.front() == '"') {
        s.remove_prefix(1);
    }
    while (!s.empty() && s.back() == '"') {
        s.remove_suffix(1);
    }
    return s;
}

std::optional<bool> parseBool(std::string_view v) {
    if (v == "on" || v == "true" || v == "yes" || v == "1") {
        return true;
    }
    if (v == "off" || v == "false" || v == "no" || v == "0") {
        return false;
    }
    return std::nullopt;
}

std::string bad(std::string_view k, std::string_view v, std::string_view want) {
    return std::format("{} = {}: expected {}", k, v, want);
}

const char* onoff(bool b) { return b ? "on" : "off"; }

} // namespace

std::optional<std::uint64_t> parseU64(std::string_view s) {
    if (!s.empty() && s.front() == '+') {
        s.remove_prefix(1);
    }
    if (s.empty() || s.front() == '-' || s.front() == '+') {
        return std::nullopt;
    }
    std::uint64_t v = 0;
    const auto [p, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
    if (ec != std::errc{} || p != s.data() + s.size()) {
        return std::nullopt;
    }
    return v;
}

namespace {
template <typename F>
std::optional<F> parseFloat(std::string_view s) {
    bool neg = false;
    if (!s.empty() && (s.front() == '+' || s.front() == '-')) {
        neg = s.front() == '-';
        s.remove_prefix(1);
    }
    if (s.empty() || s.front() == '+' || s.front() == '-') {
        return std::nullopt;
    }
    std::string low(s);
    std::ranges::transform(low, low.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (low == "inf" || low == "infinity") {
        return neg ? -INFINITY : INFINITY;
    }
    if (low == "nan") {
        return static_cast<F>(NAN);
    }
    F v = 0;
    const auto [p, ec] = std::from_chars(s.data(), s.data() + s.size(), v, std::chars_format::general);
    if (ec != std::errc{} || p != s.data() + s.size()) {
        return std::nullopt;
    }
    return neg ? -v : v;
}
} // namespace

std::optional<float> parseF32(std::string_view s) { return parseFloat<float>(s); }
std::optional<double> parseF64(std::string_view s) { return parseFloat<double>(s); }

std::string formatF32(float f) {
    // shortest round trip, fixed notation (Rust's Display never uses an exponent)
    char buf[64];
    const auto [p, ec] = std::to_chars(buf, buf + sizeof buf, f, std::chars_format::fixed);
    if (ec != std::errc{}) {
        return "0";
    }
    std::string s(buf, p);
    if (s == "-0") {
        return "-0";
    }
    return s;
}

std::expected<void, std::string> Settings::set(std::string_view key, std::string_view value) {
    const std::string_view v = trimQuotes(trim(value));
    std::string k(trim(key));
    std::ranges::replace(k, '-', '_');
    if (k == "eyes") {
        const auto e = parseEyes(v);
        if (!e) {
            return std::unexpected(bad(key, v, "left or both"));
        }
        eyes = *e;
    } else if (k == "view") {
        const auto vw = parseView(v);
        if (!vw) {
            return std::unexpected(bad(key, v, "left, right or both"));
        }
        view = *vw;
    } else if (k == "fps") {
        const auto n = parseU64(v);
        if (!n || *n > UINT32_MAX || std::ranges::find(kFpsChoices, static_cast<std::uint32_t>(*n)) == kFpsChoices.end()) {
            return std::unexpected(bad(key, v, "30, 60 or 72"));
        }
        fps = static_cast<std::uint32_t>(*n);
    } else if (k == "quality") {
        const auto q = parseQuality(v);
        if (!q) {
            return std::unexpected(bad(key, v, "low, medium or high"));
        }
        quality = *q;
    } else if (k == "scale") {
        const auto f = parseF32(v);
        if (!f || !(*f == 0.0f || (*f >= 0.1f && *f <= 1.0f))) {
            return std::unexpected(bad(key, v, "0 (auto) or 0.1 to 1.0"));
        }
        scale = *f;
    } else if (k == "max_height") {
        const auto n = parseU64(v);
        if (!n || *n < 240 || *n > 2160) {
            return std::unexpected(bad(key, v, "240 to 2160"));
        }
        max_height = static_cast<std::uint32_t>(*n);
    } else if (k == "codec") {
        const auto c = parseCodec(v);
        if (!c) {
            return std::unexpected(bad(key, v, "h264 or hevc"));
        }
        codec = *c;
    } else if (k == "encoder") {
        const auto e = parseEncoder(v);
        if (!e) {
            return std::unexpected(bad(key, v, "auto, nvenc or cpu"));
        }
        encoder = *e;
    } else if (k == "audio" || k == "mic") {
        const auto b = parseBool(v);
        if (!b) {
            return std::unexpected(bad(key, v, "on or off"));
        }
        (k == "audio" ? audio : mic) = *b;
    } else if (k == "dir") {
        if (v.empty()) {
            return std::unexpected(bad(key, v, "a directory"));
        }
        dir = std::string(v);
    } else {
        return std::unexpected(std::format("unknown setting {}", k));
    }
    return {};
}

std::expected<Settings, std::string> Settings::parse(std::string_view text) {
    Settings s;
    bool hasAudio = false;
    std::size_t i = 0;
    std::size_t start = 0;
    // Rust's str::lines(): split at '\n', a trailing '\r' is dropped, no empty last line
    while (start < text.size()) {
        std::size_t end = text.find('\n', start);
        std::string_view raw = text.substr(start, end == std::string_view::npos ? std::string_view::npos : end - start);
        start = end == std::string_view::npos ? text.size() : end + 1;
        if (!raw.empty() && raw.back() == '\r') {
            raw.remove_suffix(1);
        }
        i++;
        const std::string_view line = trim(raw.substr(0, raw.find('#')));
        if (line.empty() || line.front() == '[') {
            continue;
        }
        const std::size_t eq = line.find('=');
        if (eq == std::string_view::npos) {
            return std::unexpected(std::format("line {}: expected key = value", i));
        }
        const std::string_view k = line.substr(0, eq);
        const std::string_view v = line.substr(eq + 1);
        hasAudio = hasAudio || trim(k) == "audio";
        if (auto r = s.set(k, v); !r) {
            return std::unexpected(std::format("line {}: {}", i, r.error()));
        }
    }
    // Migration (2026-10-16, omarchyvr): a file from before audio existed never chose "medium" on purpose (the app
    // wrote every key), so it takes the new default, high quality. From then on the file has `audio` and is kept.
    if (!hasAudio && s.quality == Quality::Medium) {
        s.quality = Quality::High;
    }
    return s;
}

std::string Settings::toToml() const {
    return std::format(
        "# pibridge-record settings (pibridge-record --help). Written by `pibridge-record start --save` and the app.\n"
        "eyes = \"{}\"        # left (one eye, lens-undistorted) | both (side by side)\n"
        "view = \"{}\"      # left | right | both (the game's own render, one eye or both side by side)\n"
        "fps = {}             # 30 | 60 | 72\n"
        "quality = \"{}\"   # low | medium | high\n"
        "scale = {}            # 0 = fit max_height; 0.1 to 1.0 = fraction of the eye's resolution\n"
        "max_height = {}\n"
        "codec = \"{}\"        # h264 | hevc\n"
        "encoder = \"{}\"      # auto (NVENC/VAAPI by GPU, else CPU) | nvenc | vaapi | cpu\n"
        "dir = \"{}\"\n"
        "audio = \"{}\"       # on (what you hear) | off\n"
        "mic = \"{}\"         # on = also your microphone, mixed in | off\n",
        name(eyes), name(view), fps, name(quality), formatF32(scale), max_height, name(codec), name(encoder), dir,
        onoff(audio), onoff(mic));
}

std::expected<Settings, std::string> Settings::load(const fs::path& p) {
    std::ifstream f(p, std::ios::binary);
    if (!f) {
        std::error_code ec;
        if (!fs::exists(p, ec)) {
            return Settings{};
        }
        return std::unexpected(std::format("{}: cannot read the file", p.string()));
    }
    std::stringstream ss;
    ss << f.rdbuf();
    auto s = parse(ss.str());
    if (!s) {
        return std::unexpected(std::format("{}: {}", p.string(), s.error()));
    }
    return s;
}

std::string Settings::save(const fs::path& p) const {
    std::error_code ec;
    if (p.has_parent_path()) {
        fs::create_directories(p.parent_path(), ec);
        if (ec) {
            return ec.message();
        }
    }
    fs::path tmp = p;
    tmp.replace_extension("toml.tmp");
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        if (!f) {
            return "cannot write " + tmp.string();
        }
        f << toToml();
        if (!f.flush()) {
            return "cannot write " + tmp.string();
        }
    }
    fs::rename(tmp, p, ec);
    return ec ? ec.message() : std::string{};
}

fs::path Settings::outDir(const fs::path& home) const {
    if (dir.starts_with("~/")) {
        return home / dir.substr(2);
    }
    if (dir == "~") {
        return home;
    }
    return fs::path(dir);
}

std::pair<std::uint32_t, std::uint32_t> Settings::request() const {
    return {max_height, static_cast<std::uint32_t>(std::round(scale * 1000.0f))};
}

std::vector<std::string> Settings::toArgs() const {
    return {"--eyes",  std::string(name(eyes)),    "--fps",        std::to_string(fps),
            "--quality", std::string(name(quality)), "--scale",    formatF32(scale),
            "--max-height", std::to_string(max_height), "--codec", std::string(name(codec)),
            "--encoder", std::string(name(encoder)), "--dir",       dir,
            "--audio",   onoff(audio),               "--mic",        onoff(mic)};
}

fs::path settingsPath(const fs::path& configDir) { return configDir / "record.toml"; }

fs::path settingsReadPath(const fs::path& configDir, const fs::path& legacyDir) {
    const fs::path mine = settingsPath(configDir);
    std::error_code ec;
    if (!fs::exists(mine, ec) && !legacyDir.empty() && fs::exists(settingsPath(legacyDir), ec)) {
        return settingsPath(legacyDir);
    }
    return mine;
}

std::string fileName(std::int64_t unix) {
    std::tm tm{};
    const std::time_t t = static_cast<std::time_t>(unix);
    localtime_r(&t, &tm);
    return std::format("pimax-{:04}{:02}{:02}-{:02}{:02}{:02}.mp4", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                       tm.tm_hour, tm.tm_min, tm.tm_sec);
}

} // namespace pibridge::record
