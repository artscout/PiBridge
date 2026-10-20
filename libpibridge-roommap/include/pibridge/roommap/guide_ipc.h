// SPDX-License-Identifier: MIT
// PiBridge (not in the Rust code): the in-headset room setup's link between `pibridge-roommap setup fast --guide DIR` (the
// work: mapping, floor, free area, the room fix and the offset) and `pibridge-roomsetup --map` (the headset: passthrough,
// the steps, the floor grid, the boundary, the controller). Two small text files in DIR, each written to a .tmp file
// and renamed:
//
//   DIR/state   pibridge-roommap -> headset, rewritten at every change:
//     pibridge-roommap-guide 1
//     pid N                         pibridge-roommap's pid
//     seq N                         +1 at every rewrite
//     step mode|wear|light|map|floor-confirm|floor-touch|boundary|forward|done|failed
//     title TEXT / text TEXT        the prompt (one line each)
//     progress F                    0..1 (optional)
//     keyframes N / points N        mapping feedback (views mapped, stereo points in view)
//     conv YAW TX TY TZ             room_from_raw (Monado's raw SLAM frame -> the map's room frame), once known
//     floor Y                       the floor in the room frame, once known
//     boundary X Z X Z ...          the proposed boundary (room frame, x z pairs)
//
//   DIR/answer  headset -> pibridge-roommap, one answer at a time:
//     pibridge-roomsetup-guide 1
//     pid N                         the headset tool's pid (pibridge-roommap stops when it is gone)
//     id N                          +1 per answer; pibridge-roommap takes each id once
//     step STEP                     the step answered (an answer for another step is ignored)
//     answer yes | touch Y | height Y EYE | path X Z X Z ... | forward CX CZ YAW HEADY | cancel   (room frame;
//            height: the floor Y from the head standing upright minus the user's eye height EYE)
//            | mode M (1 = full, 0 = quick) | back | repeat | next   (the panel's mode menu and step buttons)
#pragma once

#include <array>
#include <cstdint>
#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::roommap::guide {

using P2 = std::array<double, 2>;

inline constexpr char STATE_FILE[] = "state";
inline constexpr char ANSWER_FILE[] = "answer";

/// room_from_raw as a yaw about +y and a translation (p_room = Ry(yaw) p_raw + t), session::Alignment's form.
struct Conv {
    double yaw = 0;
    std::array<double, 3> t{};
    std::array<double, 3> apply(const std::array<double, 3>& p) const;
    std::array<double, 3> inverse_apply(const std::array<double, 3>& p) const;
    bool operator==(const Conv&) const = default;
};

struct State {
    std::uint32_t pid = 0;
    std::uint64_t seq = 0;
    std::string step;
    std::string title;
    std::string text;
    std::optional<double> progress;
    std::size_t keyframes = 0;
    std::size_t points = 0;
    std::optional<Conv> conv;
    std::optional<double> floor;
    std::vector<P2> boundary;
    bool operator==(const State&) const = default;
};

std::string format_state(const State& s);
std::expected<State, std::string> parse_state(std::string_view text);

struct Answer {
    std::uint32_t pid = 0;
    std::uint64_t id = 0;
    std::string step;
    std::string kind;          // yes | touch | path | forward | cancel
    std::vector<double> nums;  // touch: Y; path: x z ...; forward: cx cz yaw head_y
    bool operator==(const Answer&) const = default;
};

std::string format_answer(const Answer& a);
std::expected<Answer, std::string> parse_answer(std::string_view text);

/// Write `text` to DIR/name through DIR/name.tmp and a rename.
std::expected<void, std::string> write_atomic(const std::string& dir, const char* name, const std::string& text);
/// Read DIR/name (nullopt: missing or unreadable).
std::optional<std::string> read_file(const std::string& dir, const char* name);

} // namespace pibridge::roommap::guide
