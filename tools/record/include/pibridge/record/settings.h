// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-record/src/settings.rs (MIT OR Apache-2.0).
// Recording settings: `~/.config/pibridge/record.toml` (a plain `key = value` file; omarchyvr's
// `~/.config/omarchyvr/record.toml` is read while PiBridge's does not exist), overridden per recording by the command
// line (`--eyes both --fps 60 ...`).
#pragma once

#include <array>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pibridge::record {

enum class Eyes { Left, Both };
// What to record (the Dashboard's selector; distinct from `Source` in run.h, which is how frames are grabbed).
// LeftEye/RightEye/BothEyes = the game's own stereo render (clean, no SteamVR overlays or lens distortion). The Vulkan
// capture layer (tools/record-layer) picks the image by this. (SteamVR's composited output is not captured.)
enum class View { LeftEye, RightEye, BothEyes };
enum class Quality { Low, Medium, High };
enum class Codec { H264, Hevc };
// Which encoder ffmpeg uses: Auto = NVENC when ffmpeg has it, else the CPU (libx264 / libx265, niced).
enum class EncoderPref { Auto, Nvenc, Vaapi, Cpu };

std::string_view name(Eyes e);
std::string_view name(View v);
std::string_view name(Quality q);
std::string_view name(Codec c);
std::string_view name(EncoderPref p);
std::optional<Eyes> parseEyes(std::string_view s);
std::optional<View> parseView(std::string_view s);
std::optional<Quality> parseQuality(std::string_view s);
std::optional<Codec> parseCodec(std::string_view s);
std::optional<EncoderPref> parseEncoder(std::string_view s);
inline std::uint32_t eyeCount(Eyes e) { return e == Eyes::Both ? 2 : 1; }

// Frame rates offered (the panel runs at 72 or 90 Hz; the tap paces to the request).
inline constexpr std::array<std::uint32_t, 3> kFpsChoices{30, 60, 72};

struct Settings {
    Eyes eyes = Eyes::Left;
    // What to record (Dashboard selector): the game's left/right/both eye render, or SteamVR's composited output.
    View view = View::LeftEye;
    std::uint32_t fps = 30;
    Quality quality = Quality::High;
    // 0 = fit max_height; else a fraction (0.1 to 1.0) of the eye's resolution, still capped by max_height.
    float scale = 0.0f;
    std::uint32_t max_height = 1080;
    Codec codec = Codec::H264;
    EncoderPref encoder = EncoderPref::Auto;
    // Output directory (`~` is expanded).
    std::string dir = "~/Videos/pimax";
    // Record what the user hears (the default output's monitor) into the video.
    bool audio = true;
    // Also mix in the default microphone (only with audio).
    bool mic = false;

    bool operator==(const Settings&) const = default;

    // Apply one `key = value` (from the file or a `--key value` flag). A rejected value changes nothing.
    std::expected<void, std::string> set(std::string_view key, std::string_view value);
    // Parse the file's text. Comments (`#`), blank lines and `[section]` headers are ignored; unknown keys are an
    // error (a typo must not silently record at the wrong settings).
    static std::expected<Settings, std::string> parse(std::string_view text);
    std::string toToml() const;
    // The settings file, or the defaults if it does not exist.
    static std::expected<Settings, std::string> load(const std::filesystem::path& p);
    // Write atomically (temp file + rename). Empty on success, else why.
    std::string save(const std::filesystem::path& p) const;
    // `dir` with `~` expanded.
    std::filesystem::path outDir(const std::filesystem::path& home) const;
    // Per-eye output height the tap is asked for: (max_height, scale_permille).
    std::pair<std::uint32_t, std::uint32_t> request() const;
    // The command-line form (what `start` passes to `run`, so a respawn records with the same settings).
    std::vector<std::string> toArgs() const;
};

// `<config dir>/record.toml`.
std::filesystem::path settingsPath(const std::filesystem::path& configDir);
// The file to read: PiBridge's, else omarchyvr's (`<omarchyvr config dir>/record.toml`) when only that one exists.
std::filesystem::path settingsReadPath(const std::filesystem::path& configDir, const std::filesystem::path& legacyDir);
// `pimax-YYYYmmdd-HHMMSS.mp4` in local time (the name omarchyvr uses, so both tools fill the same folder alike).
std::string fileName(std::int64_t unix);

// Rust's `str::parse::<u32>()` / `<f32>()`: an optional '+', then the number, nothing else.
std::optional<std::uint64_t> parseU64(std::string_view s);
std::optional<float> parseF32(std::string_view s);
std::optional<double> parseF64(std::string_view s);
// Rust's `f32` Display (shortest round-trip form, no exponent for ordinary values: 0, 0.5, 0.75).
std::string formatF32(float f);

} // namespace pibridge::record
