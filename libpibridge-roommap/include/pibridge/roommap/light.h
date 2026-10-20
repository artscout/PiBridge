// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/light.rs (MIT OR Apache-2.0).
//
// Image quality for mapping: is the room bright and textured enough, and how many stereo points does a frame give
// (the auto exposure's masked brightness metric, ORB corners per camera, stereo points), for `light` and the live
// mapping feedback ("turn on the lights" / "look at the furniture"); no head pose needed.
#pragma once

#include "pibridge/roommap/pngread.h"
#include "pibridge/roommap/stereo.h"

#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace pibridge::roommap::light {

struct FrameQuality {
    double mean = 0;
    double sat = 0;
    std::array<std::size_t, 2> corners{};
    std::size_t stereo = 0;
    std::uint16_t exposure = 0;
    std::uint16_t gain = 0;
};

FrameQuality measure(const stereo::Rig& rig, const pngread::Grey& a, const pngread::Grey& b, const orb::OrbParams& orb);
FrameQuality measure_with(const stereo::Rig& rig, const pngread::Grey& a, const pngread::Grey& b,
                          const orb::OrbParams& orb, const std::optional<stereo::LowLight>& low);

enum class Advice { Good, TooDark, TooPlain };
/// Rust Debug of the advice ("Good", "TooDark", "TooPlain").
const char* advice_debug(Advice a);

/// Masked mean grey level below which a frame set counts as dark (the "good frame" lower bound, 30).
double dark_mean();
Advice advice(const FrameQuality& q, std::size_t need);
const char* advice_text(Advice a);

struct LightReport {
    std::size_t frames = 0;
    double mean = 0;
    std::array<std::size_t, 2> corners{};
    std::size_t stereo = 0;
    std::size_t good = 0;
    std::uint16_t exposure = 0;
    std::uint16_t gain = 0;
};

LightReport summarise(std::span<const FrameQuality> qs, std::size_t need);
/// Simulated darker room: grey * scale plus deterministic noise of +-noise grey levels.
void darken(pngread::Grey& g, float scale, float noise, std::uint64_t seed);

struct MeasureOpts {
    std::size_t every = 3;
    std::optional<stereo::LowLight> low = stereo::LOW_LIGHT;
    std::optional<std::pair<float, float>> darken;
};

/// Measure every `every`-th tracking frame set of `spec` (`broker` or a .pxcam) for `seconds` (a recording: to its end).
std::expected<std::vector<FrameQuality>, std::string> measure_source(const std::string& spec, double seconds,
                                                                     const MeasureOpts& o, const stereo::Rig& rig,
                                                                     const orb::OrbParams& orb);
/// Quality from features already extracted.
FrameQuality quality_of(const stereo::Rig& rig, const std::vector<stereo::Features>& feats, const pngread::Grey& a,
                        const pngread::Grey& b);

struct MapProgress {
    double elapsed_s = 0;
    std::size_t keyframes = 0;
    FrameQuality quality;
    Advice advice = Advice::Good;
    bool move_more = false;
};

std::string progress_text(const MapProgress& p);
/// Has the head moved: poses span more than min_deg of view direction or min_m of position.
bool moved(std::span<const cam::geom::Pose> poses, double min_deg, double min_m);

} // namespace pibridge::roommap::light
