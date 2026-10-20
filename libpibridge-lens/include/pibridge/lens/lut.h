// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-lens/src/lut.rs (MIT OR Apache-2.0).
//
// Exact distortion from Pimax's own samples ("LUT mode"). `pimax-lens make-lut <dump.json> --out FILE` stores, per
// eye and colour channel, the grid of the PVR distortion dump as ray tangents (x right, y up); lens.conf
// `lut = FILE` makes the Lens sample it instead of the radial polynomial.
//
// File format (little endian, version 1):
//   0   8   magic "PIMAXLUT"
//   8   u32 version (1)
//   12  u32 grid size n
//   16  u32 channels (3)    20 u32 eyes (2)
//   24  32  serial, NUL padded
//   56  per eye (2 x 36 bytes): u32 orientation (index into ORIENTS), f32 panel width, f32 panel height (px),
//       f32 FOV port tangents up, down, left, right, f32 reserved x2 (0)
//   128 data: eye, channel (R G B), then n*n points row-major (panel v outer, panel u inner) of f32 tx, ty
//   end u32 CRC32 of everything before it
// The data is in the dump's own panel order; `orientation` says how the panel axes map to the user-facing display,
// and from_bytes resamples (an exact permutation of grid nodes) into the user-display grid that sample()
// interpolates.
#pragma once

#include "pibridge/lens/config.h"

#include <array>
#include <cstdint>
#include <expected>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace pibridge::lens {

struct Dump; // pvrdump.h

/// A signed permutation matrix (pvrfit.rs `Dm`).
using Dm = std::array<std::array<double, 2>, 2>;
/// The 8 symmetries of the square, with a description of what the user-facing axes are in panel axes
/// (pvrfit.rs `ORIENTS`).
struct Orient {
    Dm d;
    const char* desc;
};
extern const std::array<Orient, 8> ORIENTS;

inline constexpr char LUT_MAGIC[8] = {'P', 'I', 'M', 'A', 'X', 'L', 'U', 'T'};
inline constexpr std::uint32_t LUT_VERSION = 1;
/// Orientation of the left / right eye panel: left user x = panel v, user y = -panel u; right user x = -panel v,
/// user y = panel u.
inline constexpr std::uint32_t ORIENT_LEFT = 3;
inline constexpr std::uint32_t ORIENT_RIGHT = 1;

/// Per-eye metadata of a LUT file.
struct EyeMeta {
    std::uint32_t orient = 0;
    std::array<float, 2> panel{};
    /// FOV port tangents: up, down, left, right.
    std::array<float, 4> fov_tan{};
    bool operator==(const EyeMeta&) const = default;
};

using Tan = std::array<float, 2>;

class Lut {
public:
    std::string serial;
    std::size_t n = 0;

    /// Build from a parsed PVR dump (orientation of the Dream Air: left 3, right 1).
    static std::expected<Lut, std::string> from_dump(const Dump& dump);
    static std::expected<Lut, std::string> from_bytes(std::span<const std::uint8_t> b);
    /// Read and parse a file; errors are prefixed with the path.
    static std::expected<Lut, std::string> load(const std::string& path);

    const EyeMeta& meta(std::size_t eye) const { return eyes_[eye].meta; }
    /// FOV port half-angles in degrees: left, right, up, down.
    std::array<float, 4> fov_deg(std::size_t eye) const;
    std::vector<std::uint8_t> to_bytes() const;
    /// Ray tangent (x right, y up) that channel `c` shows at display point (u, v) of `eye` (v = 0 at the top).
    /// Points outside [0,1] are clamped to the grid.
    Tan sample(std::size_t eye, std::size_t c, float u, float v, Interp interp) const;

    /// The user-display grid / the stored panel-order data (tests).
    const std::vector<Tan>& data(std::size_t eye, std::size_t c) const { return eyes_[eye].data[c]; }
    const std::vector<Tan>& raw(std::size_t eye, std::size_t c) const { return eyes_[eye].raw[c]; }

private:
    struct EyeLut {
        EyeMeta meta;
        std::array<std::vector<Tan>, CHANNELS> data;
        std::array<std::vector<Tan>, CHANNELS> raw;
        EyeLut(EyeMeta m, std::array<std::vector<Tan>, CHANNELS> r, std::size_t n);
        EyeLut() = default;
    };
    std::array<EyeLut, 2> eyes_;
};

/// Panel UV -> user-facing display (u, v), v down, for orientation matrix `d`.
std::pair<double, double> panel_to_display(const Dm& d, double pu, double pv);

/// Error statistics of a LUT against the dump's radial lines.
struct LineError {
    std::size_t samples = 0;
    double rms_deg = 0.0;
    double max_deg = 0.0;
    /// Where the maximum is: eye, line name, channel, position along the line 0..1.
    std::size_t worst_eye = 0;
    std::string worst_line;
    std::size_t worst_channel = 0;
    double worst_pos = 0.0;
};

/// Compare `interp` sampling of `lut` with the radial lines of `dump`, visible samples only (output in [0,1]).
LineError line_error(const Lut& lut, const Dump& dump, Interp interp);

namespace detail {
std::array<float, 4> catmull_rom(float t);
double ray_angle_deg(std::array<double, 2> a, std::array<double, 2> b);
} // namespace detail

} // namespace pibridge::lens
