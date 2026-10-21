// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-lens/src/config.rs (MIT OR Apache-2.0).
//
// `lens.conf` parsing. The grammar is the one of the C parser the Rust code replaced (`pimax_lens.c` of the Monado
// `omarchyvr` branch): INI-like, `#` and `;` start comments, `[common]`/`[both]` (or no section) apply to both eyes,
// `[left]`/`[right]` override one eye, later lines win, bad lines are reported and skipped. Every key of the old file
// format keeps its meaning; the newer keys are optional and default to "no change".
//
// Keys (per eye or common): distortion (on/off, global), cant_deg (deg, global, PIMAX_CANT_DEG overrides), fov
// (L R U D half-angles), center (x y, half-display-heights, +x right, +y up), base / base_r|g|b, scale,
// k / k_r|g|b (k1 k2 k3 [k4 k5]), ca_r|g|b, shift_r|g|b (dx dy), fit_range, blackout (w or L R U D),
// mask (off / ellipse a b / rect a b / superellipse a b n), mask_feather, lut (FILE | none), lut_margin, lut_interp.
#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::lens {

/// Colour channels: 0 = red, 1 = green, 2 = blue.
inline constexpr std::size_t CHANNELS = 3;
/// Largest accepted |cant_deg|.
inline constexpr float MAX_CANT_DEG = 20.0f;
/// Number of polynomial coefficients per channel (k1 .. k5, multiplying r^2, r^4, .. r^10).
inline constexpr std::size_t K_TERMS = 5;

/// How a LUT grid is interpolated (lut.rs `Interp`).
enum class Interp { Bilinear, Bicubic };
/// Rust `{:?}`: "Bilinear" / "Bicubic".
const char* interp_name(Interp i);

/// Shape of the visibility mask.
enum class MaskShape { Off, Ellipse, Rect, Superellipse };
/// Rust `{:?}`: "Off", "Ellipse", "Rect", "Superellipse".
const char* mask_shape_name(MaskShape s);

/// Visibility mask around the optical centre: `(|x/a|^n + |y/b|^n)^(1/n) <= 1` is visible.
struct Mask {
    MaskShape shape = MaskShape::Off;
    float a = 0.0f;
    float b = 0.0f;
    float n = 2.0f;
    static constexpr Mask off() { return Mask{}; }
    bool operator==(const Mask&) const = default;
};

using Poly = std::array<float, K_TERMS>;

/// Lens parameters of one eye.
struct EyeConfig {
    /// Half-angles in degrees of the rendered (client) FOV: left, right, up, down (all positive).
    std::array<float, 4> fov{};
    /// Optical centre on the panel in units of half the display height, +x right, +y up.
    float cx = 0.0f;
    float cy = 0.0f;
    /// Tangent of the ray angle per unit panel radius at the centre, per channel.
    std::array<float, CHANNELS> base{};
    /// Overall multiplier on ray tangents.
    float scale = 1.0f;
    /// Polynomial per channel: tan = base * scale * r * (1 + k1 r^2 + k2 r^4 + ...).
    std::array<Poly, CHANNELS> k{};
    /// Per-channel radial divisor (lateral chromatic aberration).
    std::array<float, CHANNELS> ca{};
    /// Per-channel optical centre shift relative to (cx, cy).
    std::array<std::array<float, 2>, CHANNELS> shift{};
    /// Radius beyond which the radial curve is replaced by its tangent line; 0 = never.
    float fit_range = 0.0f;
    /// Black margins at the panel edges: left, right, top, bottom (half-display-heights).
    std::array<float, 4> blackout{};
    Mask mask{};
    /// Width of the transition band outside the black-out boundary.
    float mask_feather = 0.0f;
    /// `lut = FILE` (relative to the config file); nullopt = radial model.
    std::optional<std::string> lut;
    /// `lut_margin = DEG`: widen the rendered FOV by this much on every side of the LUT's FOV port.
    float lut_margin = 0.0f;
    Interp lut_interp = Interp::Bicubic;

    /// Built-in defaults: fitted to the "Dream Air like Pimax" angle table.
    static EyeConfig defaults(std::size_t eye);
    /// True when any of the optional visibility features is set.
    bool has_mask() const;
    bool operator==(const EyeConfig&) const = default;
};

/// Parsed configuration for both eyes.
struct LensConfig {
    /// false: identity mesh (no distortion); the FOV still applies.
    bool enabled = true;
    /// Display width / height as the client sees it (3840/3552).
    float aspect = 1.0f;
    /// Outward cant of each display in degrees (view rotation about +Y: left +, right -).
    float cant_deg = 0.0f;
    std::array<EyeConfig, 2> eye{};

    static LensConfig defaults(float aspect);
    bool operator==(const LensConfig&) const = default;
};

/// A problem found while parsing; parsing continues after it.
struct Warning {
    std::size_t line = 0;
    std::string message;
    bool operator==(const Warning&) const = default;
};

/// The environment as the Rust `&dyn Fn(&str) -> Option<String>`.
using Env = std::function<std::optional<std::string>(std::string_view)>;

/// Apply the text of a lens.conf to `cfg`. `name` is only used in the warnings (usually the file path).
std::vector<Warning> parse_into(LensConfig& cfg, std::string_view text, std::string_view name);
/// Defaults plus the text of a lens.conf (warnings name "lens.conf").
std::pair<LensConfig, std::vector<Warning>> parse(std::string_view text, float aspect);

/// `PIMAX_LENS_CONF`, else `$XDG_CONFIG_HOME/pibridge/lens.conf`, else `$HOME/.config/pibridge/lens.conf`
/// (`./.config/...` without HOME).
std::string config_path(const Env& env);
/// `PIMAX_CANT_DEG=<deg>`: nullopt if unset/blank; `ok` false with the raw text if unparsable or out of range.
struct CantOverride {
    bool ok = false;
    float value = 0.0f;
    std::string raw;
};
std::optional<CantOverride> cant_override(const Env& env);
/// The shipped reference profile is used unless `PIMAX_NO_REFERENCE` is set to something other than `0`.
bool reference_allowed(const Env& env);
/// `PIMAX_LENS_OFF` non-empty and not starting with '0'.
bool forced_off(const Env& env);
/// The process environment (`std::env::var(..).ok()`: unset or not UTF-8 = nullopt).
std::optional<std::string> process_env(std::string_view name);

namespace detail {
/// `strtof`-like scan of a decimal float at the start of `s` (leading white space allowed): value and bytes used.
std::optional<std::pair<float, std::size_t>> scan_float(std::string_view s);
/// Between `min` and `max` floats separated by white space and/or commas.
std::optional<std::vector<float>> floats(std::string_view s, std::size_t min, std::size_t max);
} // namespace detail

} // namespace pibridge::lens
