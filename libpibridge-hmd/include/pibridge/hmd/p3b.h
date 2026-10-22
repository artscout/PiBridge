// SPDX-License-Identifier: MIT
// The original Pimax Crystal ("Pimax P3B", standalone Snapdragon XR2, USB 34A4:0012). It does not speak the
// Dream Air / Crystal Super protocol: one HID interface (USB interface 2) with 1024-byte input and output reports and no
// report ids, carrying plaintext RPC frames
//   [0] type (0x0c command, 0x1c call, 0x3b property read), [1..4] u32 LE name length, [5..8] u32 LE value length,
//   then the name, then the value, zero padded to 1024 bytes.
// The headset tracks itself (SLAM on the XR2) and is switched on by its own power button; the PC only answers.
// Only the frames Pimax Play sends were captured (Windows capture 2026-10-22, G:/omarchyvr-windows/
// CRYSTAL-P3B-2026-10-22.md, crystal-session-20261022-*): `command()` returns exactly those bytes and nothing else is
// ever built for writing.
#pragma once

#include "pibridge/hmd/discovery.h"

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace pibridge::hmd::p3b {

inline constexpr std::uint16_t VID = 0x34A4;
inline constexpr std::uint16_t PID_CRYSTAL = 0x0012;
inline constexpr std::size_t REPORT_SIZE = 1024;
using Report = std::array<std::uint8_t, REPORT_SIZE>;

namespace type {
inline constexpr std::uint8_t COMMAND = 0x0c;
inline constexpr std::uint8_t CALL = 0x1c;
inline constexpr std::uint8_t GETPROP = 0x3b;
} // namespace type

/// One RPC frame.
struct Rpc {
    std::uint8_t type = 0;
    std::string name;
    std::string value;
    bool operator==(const Rpc&) const = default;
};

/// The frame for `r`, zero padded (nullopt when it does not fit a report).
std::optional<Report> encode(const Rpc& r);
/// An RPC frame read back (a reply or a request); nullopt for anything else, e.g. the tracking stream. The reply's
/// value is cut at the first zero byte: the headset reuses its buffer, so stale bytes follow a short value.
std::optional<Rpc> decode(std::span<const std::uint8_t> data);

/// The commands PiBridge may send, each the exact frame of the capture.
enum class Cmd {
    ScreenOn,            // 0c "setprop screen_state 1"                      (connect, 20:28:24.231)
    KeepRuntime,         // 0c "setprop control_signal_restart_runtime 0"    (connect, 20:28:24.231)
    PcVideo120,          // 1c "piPcSwicthIoctl" "video 120"                 (connect, 20:28:24.694)
    ConnectState,        // 1c "piPcSwicthIoctl" "getConnectState"           (every 10 s while connected)
    GetLighthouseEnable, // 3b "persist.sys.pmx.lighthouse.enable"           (reply value "0" / "1")
    GetLighthouseSupport,// 3b "persist.sys.pmx.lighthouse.support"
    LighthouseOn,        // 0c "EnableLighthouse" "1"                        (20:30:32.491)
    LighthouseOff,       // 0c "EnableLighthouse" "0"                        (20:33:02.292)
    LighthouseTakeEffect,// 0c "LighthouseTakeEffect" "1"  (~8 s after LighthouseOn; none after Off) (20:30:40.493)
    Shutdown,            // 1c "shutDown" (reply "true"; the headset powers off) (20:39:27.260)
    TobiiEnable,         // 1c "setTobiiEyeDeviceEnable" "1" (reply "true"; connect, after the getters, 20:28:34.819):
                         // the eye tracker 2104:0220 comes up on USB 1.6 s later. No disable was ever sent.
};
const Rpc& command(Cmd c);
/// The frame of `c` as captured, hex (for the tests and the log).
std::string_view captured_hex(Cmd c);
std::string_view name_of(Cmd c);

/// The tracking stream (input reports, decoded 2026-10-22 from the Windows capture; one sample per report, plain
/// little endian). Byte 0: low nibble = device (1 head, 2 left, 3 right controller), high nibble = state.
struct Pose {
    std::uint64_t t_ns = 0;            // sample time, ns since the headset booted ([1])
    std::array<float, 3> position{};   // m ([9])
    std::array<float, 4> orientation{}; // quaternion x, y, z, w ([21])
};
/// Head report 0x31 (tracked) / 0x01 (no pose yet), 140 bytes used, ~667/s.
struct Head {
    bool tracked = false;               // byte 0 high nibble 3
    Pose pose;
    std::array<float, 3> linear_velocity{};  // m/s, world ([49])
    std::array<float, 3> angular_velocity{}; // rad/s, body frame ([61])
    int battery = -1;                   // % ([109])
    int tracking = 0;                   // its own inside-out: flags 1 orientation, 2 position (3 both); 0 = Lighthouse ([113])
    float ipd_mm = 0;                   // live lens IPD, float16 ([122])
    int volume = -1;                    // 0..100 ([132])
    int buttons = 0;                    // 2 = volume up, 1 = volume down ([138])
};
/// Controller report 0x42/0x52/0x72 (left), 0x43/0x53/0x73 (right), 62 bytes used, ~333/s. State 4 = connected without
/// a pose, 5 = pose from the IMU only (position frozen), 7 = optically tracked.
struct Controller {
    int side = 0;                       // 0 left, 1 right
    int state = 0;                      // 4, 5 or 7
    Pose pose;
    std::array<float, 2> stick{};       // x right, y up, -1..1 ([55], [56], signed bytes)
    float trigger = 0;                  // 0..1 ([57])
    float grip = 0;                     // 0..1 ([58])
    int buttons = 0;                    // pressed, BUTTON_* ([59])
    int touches = 0;                    // touched, TOUCH_* ([60])
    int battery = -1;                   // % ([61])
};
/// Controller buttons ([59]), the same on both sides (2026-10-22, each pressed in turn in the headset).
inline constexpr int BUTTON_A = 0x01;     // X on the left
inline constexpr int BUTTON_B = 0x02;     // Y on the left
inline constexpr int BUTTON_MENU = 0x04;
inline constexpr int BUTTON_STICK = 0x08; // the stick pressed
inline constexpr int BUTTON_PIMAX = 0x40;
/// Controller touch sensors ([60]).
inline constexpr int TOUCH_A = 0x01;
inline constexpr int TOUCH_B = 0x02;
inline constexpr int TOUCH_STICK = 0x08;
inline constexpr int TOUCH_TRIGGER = 0x10;
inline constexpr int TOUCH_GRIP = 0x20;
inline constexpr int TOUCH_PIMAX = 0x40;
std::optional<Head> decode_head(std::span<const std::uint8_t> data);
std::optional<Controller> decode_controller(std::span<const std::uint8_t> data);
/// IEEE half to float.
float half_to_float(std::uint16_t h);

/// hidraw node of the Crystal (HID_ID 0003:000034A4:00000012), sysfs only.
std::optional<std::filesystem::path> find_hidraw(const discovery::Roots& roots);

} // namespace pibridge::hmd::p3b
