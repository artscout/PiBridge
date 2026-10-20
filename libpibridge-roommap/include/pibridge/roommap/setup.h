// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/setup.rs (MIT OR Apache-2.0).
//
// Room setup: the play area (boundary), its centre and forward direction, stored with the room map (room-setup.toml),
// so every session puts the same floor, centre and forward on the same real place. Setup frame: origin = the play
// area's centre on the floor, -Z = forward, y up; Monado's offset = setup_from_room * floor shift * room_from_slam
// (final_offset). Geometry: boundary clean-up, the largest centred rectangle inside it (play_area), the free area
// around a standing user from the map's obstacle points (free_area), SteamVR's collision walls (walls). Guided steps
// (confirm_forward, trace_boundary) run against the Guide interface: prompts out, head and controller samples in.
#pragma once

#include "pibridge/roommap/session.h"

#include <array>
#include <cstdint>
#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace pibridge::roommap::setup {

using session::Alignment;
/// A point on the floor plane: (x, z) in metres.
using P2 = std::array<double, 2>;

inline constexpr double WALL_HEIGHT_M = 2.43;
inline constexpr double DEFAULT_HALF_M = 1.0;
inline constexpr char FILE[] = "room-setup.toml";

enum class SetupMode { Fast, Full };
const char* mode_name(SetupMode m);
std::optional<SetupMode> mode_parse(std::string_view s);

struct RoomSetup {
    std::string label;
    SetupMode mode = SetupMode::Fast;
    P2 centre{};
    double yaw = 0;
    std::vector<P2> boundary;
    std::array<double, 2> play_area{};
    double head_m = 0;
    std::uint64_t when = 0;

    Alignment room_from_setup() const;
    Alignment setup_from_room() const;
    std::vector<P2> boundary_setup() const;
    double area() const;
    std::string describe() const;
    std::string to_toml() const;
    static std::expected<RoomSetup, std::string> parse(std::string_view s);
    static std::expected<RoomSetup, std::string> load(const std::string& dir);
    std::expected<void, std::string> save(const std::string& dir) const;
};

P2 to_setup(P2 centre, double yaw, P2 p);
P2 to_room(P2 centre, double yaw, P2 p);
/// The heading (rotation about +y turning -Z into it) of a horizontal direction (x, z).
double heading(double dir_x, double dir_z);
Alignment final_offset(const Alignment& room_from_slam, double floor_dy, const RoomSetup* setup);
double area(const std::vector<P2>& p);
double perimeter(const std::vector<P2>& p);
double dist(P2 a, P2 b);
bool inside(const std::vector<P2>& poly, P2 q);
bool segments_cross(P2 a, P2 b, P2 c, P2 d);
bool self_intersects(const std::vector<P2>& p);
std::vector<P2> convex_hull(const std::vector<P2>& pts);
std::expected<std::vector<P2>, std::string> clean_boundary(const std::vector<P2>& trace, double tol);
bool rect_inside(const std::vector<P2>& poly, double hx, double hz);
std::array<double, 2> play_area(const std::vector<P2>& poly);
std::vector<P2> free_area(const std::vector<P2>& obstacles, std::optional<std::array<double, 4>> extent, double max_half);
std::pair<std::vector<P2>, std::optional<std::array<double, 4>>> obstacles_from_points(
    const std::vector<std::array<double, 3>>& points, double floor_y, P2 centre, double yaw);
std::vector<std::array<std::array<double, 3>, 4>> walls(const std::vector<P2>& poly, double height);

struct HandSample {
    std::optional<std::array<double, 3>> pos;
    double value = 0;
    bool active = false;
};

struct Sample {
    double t = 0;
    /// Head position (room frame) and heading.
    std::optional<std::pair<std::array<double, 3>, double>> head;
    std::array<HandSample, 2> hands{};
};

class Guide {
public:
    virtual ~Guide() = default;
    virtual void say(const std::string& title, const std::string& text, std::optional<float> progress) = 0;
    virtual Sample sample() = 0;
};

/// Centre and forward: (centre x z, heading, head height y in the room frame).
std::expected<std::tuple<P2, double, double>, std::string> confirm_forward(Guide& g, double timeout_s);
/// Trace the boundary: the traced points (room frame, x z).
std::expected<std::vector<P2>, std::string> trace_boundary(Guide& g, double timeout_s);

} // namespace pibridge::roommap::setup
