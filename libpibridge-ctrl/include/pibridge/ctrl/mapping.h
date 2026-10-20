// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-hands/src/mapping.rs (MIT OR Apache-2.0).
// Controller inputs -> finger curls: index on the trigger, middle/ring/little on the grip, the thumb on whichever of
// A/X, B/Y, the stick or the menu button it touches (debounced state machine + critically damped smoother). A pure
// function of the inputs and their timestamps.
#pragma once

#include "pibridge/ctrl/filter.h"

#include <array>
#include <cstdint>
#include <optional>

namespace pibridge::ctrl {

/// Bits of Input::flags (same values as include/pimax_hands.h).
namespace flags {
inline constexpr std::uint32_t TOUCH_BTN1 = 1u << 0;
inline constexpr std::uint32_t TOUCH_BTN2 = 1u << 1;
inline constexpr std::uint32_t TOUCH_STICK = 1u << 2;
inline constexpr std::uint32_t CLICK_BTN1 = 1u << 3;
inline constexpr std::uint32_t CLICK_BTN2 = 1u << 4;
inline constexpr std::uint32_t CLICK_STICK = 1u << 5;
inline constexpr std::uint32_t CLICK_MENU = 1u << 6;
inline constexpr std::uint32_t ALL = 0x7f;
} // namespace flags

/// One controller report, already decoded. Analog values are 0..1 (sticks -1..1).
struct Input {
    float trigger = 0.0f;
    float grip = 0.0f;
    float stick_x = 0.0f;
    float stick_y = 0.0f;
    std::uint32_t flags = 0;
    bool operator==(const Input&) const = default;
};

/// Where the thumb rests.
enum class ThumbState : std::uint32_t { Rest = 0, Btn1 = 1, Btn2 = 2, Stick = 3, Menu = 4 };

/// Tuning. All curls are normalised: 0 = finger straight, 1 = fully closed fist.
struct Params {
    float trigger_min_cutoff = 3.0f;
    float trigger_beta = 6.0f;
    float grip_min_cutoff = 2.0f;
    float grip_beta = 4.0f;
    float d_cutoff = 1.0f;
    float deadzone = 0.02f;
    float index_rest = 0.10f;
    float index_full = 0.85f;
    float grip_rest = 0.25f;
    float grip_full = 1.0f;
    float thumb_rest = 0.0f;
    float thumb_btn1 = 0.45f;
    float thumb_btn2 = 0.50f;
    float thumb_stick = 0.55f;
    float thumb_menu = 0.50f;
    float thumb_click_extra = 0.15f;
    float engage_s = 0.12f;
    float release_hold_s = 0.15f;
    float stick_on = 0.20f;
    float stick_off = 0.12f;
    float thumb_smooth_time_s = 0.07f;
    float thumb_max_speed = 8.0f;
    float reset_gap_s = 0.5f;
};

/// Normalised curl of each finger.
struct Curls {
    float little = 0.0f;
    float ring = 0.0f;
    float middle = 0.0f;
    float index = 0.0f;
    float thumb = 0.0f;
    /// [little, ring, middle, index, thumb], the order of Monado's u_hand_tracking_values.
    std::array<float, 5> to_array() const { return {little, ring, middle, index, thumb}; }
    bool operator==(const Curls&) const = default;
};

/// Remove the dead zone and rescale so full range is preserved.
float shape(float x, float deadzone);
float lerp(float a, float b, float t);

namespace detail {
struct Contact {
    bool active = false;
    double since = 0.0;
    float idle = 0.0f;
    float pending = 0.0f;
    bool clicked = false;
};
} // namespace detail

/// Filtered state of one hand.
class HandState {
public:
    explicit HandState(Params params);
    /// Back to the relaxed rest pose; the next update starts from its input directly.
    void reset();
    const Params& params() const { return params_; }
    Curls curls() const { return curls_; }
    ThumbState thumb_state() const;
    /// Feed one report taken at `now_ns` (any monotonic clock); returns the new curls.
    Curls update(std::uint64_t now_ns, const Input& input);

private:
    void restart(const Input& inp);
    void step(const Input& inp, float dt);
    void update_contacts(const Input& inp, float dt, bool immediate);
    float thumb_target() const;
    Curls compose(float trigger, float grip) const;

    Params params_;
    OneEuro trigger_f_;
    OneEuro grip_f_;
    SmoothDamp thumb_sd_;
    std::array<detail::Contact, 4> contacts_{};
    bool stick_deflected_ = false;
    double clock_ = 0.0;
    std::optional<std::uint64_t> last_ns_;
    std::optional<std::size_t> selected_;
    Curls curls_;
};

/// Joint curls of one finger in the layout of Monado's struct u_hand_tracking_finger_value.
struct FingerJoints {
    float splay = 0.0f;
    std::array<float, 4> joint_curls{};
    bool operator==(const FingerJoints&) const = default;
};

/// All five fingers in the order of struct u_hand_tracking_values.
struct HandJoints {
    FingerJoints little, ring, middle, index, thumb;
};

/// Natural resting splay of index, middle, ring, little.
inline constexpr std::array<float, 4> SPLAY = {-0.01f, 0.0f, 0.01f, 0.02f};
/// Thumb splay used for Index controllers in Monado.
inline constexpr float THUMB_SPLAY = -0.35f;

/// Spread one normalised curl over the joints of a finger like u_hand_sim_simulate_for_valve_index_knuckles.
HandJoints to_joints(const Curls& c);

} // namespace pibridge::ctrl
