// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-hands/src/inhand.rs (MIT OR Apache-2.0).
// Is a controller in the hand or lying somewhere? (PIMAX_CTRL_INHAND, omarchyvr docs/CONTROLLERS-INHAND.md.) From
// the controller's own IMU and inputs: 20 ms block means, 0.5 s stillness windows; "down" after 8 of 10 still windows
// with no motion and no input change for 5 s, or no report for 1.5 s; "held" after 120 ms of gyro motion above
// 3.7 deg/s, 60 ms tilted more than 10 deg, or any input change. Thresholds tuned on the session 2026-10-16-1818.
#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

namespace pibridge::ctrl::inhand {

/// Thresholds; the defaults hold the values tuned on the recording.
struct Config {
    std::uint64_t block_ms = 20;
    std::size_t window_blocks = 25;
    double still_lsb = 2.5;
    double motion_lsb = 8.0;
    std::size_t down_windows = 10;
    std::size_t down_still = 8;
    double move_lsb = 30.0;
    std::uint32_t move_blocks = 6;
    double tilt_deg = 10.0;
    std::uint32_t tilt_blocks = 3;
    double no_report_s = 1.5;
    double input_hold_s = 5.0;
    std::int32_t analog_step = 24;

    /// Defaults, PIMAX_CTRL_INHAND_DOWN_S (seconds of stillness before a put-down, 1..60) on top.
    static Config from_env();
    bool operator==(const Config&) const = default;
};

/// What the switch in the app says (PIMAX_CTRL_INHAND).
enum class Mode { Auto, Always, Off };

/// auto / when-held / held, always / on, off / never (trimmed, ASCII case-insensitive).
std::optional<Mode> parse_mode(std::string_view s);
const char* mode_name(Mode m);

/// Why the state last changed.
enum class Why { Start, Motion, Tilt, Input, Still, NoReports };
const char* why_name(Why w);

namespace detail {
struct Inputs {
    std::int32_t trigger = 0, grip = 0, stick_x = 0, stick_y = 0;
    std::uint32_t buttons = 0;
    std::uint8_t touch = 0;
    bool operator==(const Inputs&) const = default;
};
} // namespace detail

/// One controller's detector.
class InHand {
public:
    explicit InHand(Config cfg = Config{});

    /// A report arrived at t_ns (keeps the no-report timeout away).
    void report_seen(std::uint64_t t_ns) { last_report_ns_ = t_ns; }
    /// The inputs of a report: trigger and grip 0..255, stick -128..127, buttons (PIMAX_BTN_*), touch (byte 49).
    void input(std::uint64_t t_ns, std::uint8_t trigger, std::uint8_t grip, std::int32_t stick_x, std::int32_t stick_y,
               std::uint32_t buttons, std::uint8_t touch);
    /// One IMU sample (raw LSB) at t_ns.
    void sample(std::uint64_t t_ns, std::array<std::int16_t, 3> g, std::array<std::int16_t, 3> a);
    /// One raw 64-byte controller report plus its decoded inputs, received at host_ns.
    void report(std::uint64_t host_ns, const std::uint8_t* raw64, std::uint8_t trigger, std::uint8_t grip,
                std::int32_t stick_x, std::int32_t stick_y, std::uint32_t buttons, std::uint8_t touch);
    /// Held at now_ns? (No report for no_report_s: down.)
    bool held(std::uint64_t now_ns);
    bool is_held() const { return held_; }
    std::pair<Why, std::uint64_t> last_change() const { return {why_, changed_ns_}; }

    Config cfg;
    /// Transitions so far (held, down).
    std::pair<std::uint32_t, std::uint32_t> counts{0, 0};

private:
    using V3 = std::array<double, 3>;
    void set(bool held, Why why, std::uint64_t t);
    void block(std::uint64_t t, V3 g, V3 a);

    bool held_ = false;
    Why why_ = Why::Start;
    std::uint64_t changed_ns_ = 0;
    std::optional<std::uint64_t> last_report_ns_;
    std::optional<std::uint64_t> last_input_ns_;
    std::optional<detail::Inputs> inputs_;
    std::optional<std::uint64_t> blk_start_;
    std::uint32_t blk_n_ = 0;
    V3 blk_g_{0.0, 0.0, 0.0};
    V3 blk_a_{0.0, 0.0, 0.0};
    std::vector<std::pair<V3, V3>> win_;
    std::deque<std::pair<bool, double>> ring_;
    std::optional<V3> bias_;
    std::optional<V3> rest_a_;
    std::uint32_t move_run_ = 0;
    std::uint32_t tilt_run_ = 0;
};

} // namespace pibridge::ctrl::inhand
