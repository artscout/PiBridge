// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-proto/src/{lib,frame,model,imu,status,controller,log,calib}.rs (MIT OR Apache-2.0).
//
// Wire protocol of the Pimax Dream Air (USB 34a4:0044) and Crystal Super (34a4:0040) over HID reports.
// Pure functions on 64-byte reports: no I/O, no allocation in the hot decoders.
//
// Host to device: `58 LL CMD OPT DATA..` (LL = data length + 3), zero padded to 64 bytes.
// Device to host on IF0: `4c LL CMD OPT DATA.. CK`; IF2 carries IMU reports `53 31 08 00 ..`,
// IF1 (and sometimes others) controller reports `53 32 08 SS 43 ..`, IF3 an ASCII firmware log.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <functional>
#include <optional>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

namespace pibridge::hmd {

/// Size of every HID report.
inline constexpr std::size_t REPORT_SIZE = 64;
/// One 64-byte HID report.
using Report = std::array<std::uint8_t, REPORT_SIZE>;

/// USB vendor id of the headset.
inline constexpr std::uint16_t VID = 0x34a4;
/// Number of USB interfaces (0..3).
inline constexpr std::size_t NUM_IF = 4;
/// Payload bytes of one command (64 - 4 header).
inline constexpr std::size_t MAX_DATA = 60;
/// Keepalive period of the host, in milliseconds.
inline constexpr std::uint64_t KEEPALIVE_MS = 213;
/// Controller slots.
inline constexpr std::uint8_t SLOT_LEFT = 2;
inline constexpr std::uint8_t SLOT_RIGHT = 3;

/// Controller button bits (`ControllerReport::buttons`).
inline constexpr std::uint32_t BTN_B = 0x01;
inline constexpr std::uint32_t BTN_A = 0x02;
inline constexpr std::uint32_t BTN_STICK = 0x08;
/// Byte 48 bit 0, remapped.
inline constexpr std::uint32_t BTN_MENU = 0x10;
/// PiBridge: the Menu button (byte 47 bit 2; live 2026-10-19, both hands). BTN_MENU above (byte 48 bit 0, the Rust
/// name) is in fact the Pimax button. Not decoded by the Rust libpimax: off with PIBRIDGE_CTRL_PARITY=1.
inline constexpr std::uint32_t BTN_APP_MENU = 0x04;
/// Raw touch bit for the thumbstick (byte 49).
inline constexpr std::uint8_t TOUCH_STICK = 0x08;

/// Command ids used by the host.
namespace cmd {
inline constexpr std::uint8_t VERSION = 0x03;
/// Controller vibration `58 09 07 SS AMP DUR_lo DUR_hi 00 00 00` (no reply; Windows capture 2026-10-15).
inline constexpr std::uint8_t HAPTIC = 0x07;
/// Calibration document read.
inline constexpr std::uint8_t READ_DOC = 0x0a;
inline constexpr std::uint8_t SERIAL = 0x16;
/// Wake / announce rate.
inline constexpr std::uint8_t ANNOUNCE = 0x1a;
/// Controller pairing, eSpiCmdStartPair `58 04 04 00 SS` (ack `4c 04 04 00 01 07`; the result is in the IF3 log).
inline constexpr std::uint8_t START_PAIR = 0x04;
/// Controller unpair `58 03 05 SS` (reply `4c 04 05 SS 01 07`).
inline constexpr std::uint8_t UNPAIR = 0x05;
/// Headset restart, eSpiCmdSysReset `58 04 28 00 00` (reply `4c 04 28 00 01 07`).
inline constexpr std::uint8_t SYS_RESET = 0x28;
inline constexpr std::uint8_t PANEL_ON = 0x29;
/// Controller status heartbeat (device to host).
inline constexpr std::uint8_t STATUS = 0x37;
inline constexpr std::uint8_t KEEPALIVE = 0x39;
/// Headset-local info (also part of the controller attach burst).
inline constexpr std::uint8_t INFO = 0x50;
inline constexpr std::uint8_t APP_UNLOCK = 0x5b;
/// Lens IPD motor `58 06 5c 05 01 <mm*100 LE>` (reply `4c 04 5c 05 01 07`).
inline constexpr std::uint8_t IPD = 0x5c;
} // namespace cmd

// ------------------------------------------------------------------------------------------------ models

/// One headset model, told apart by the USB product id of the HID chip. The Crystal Super answers the same `58 ..`
/// frames on the same four interfaces as the Dream Air; only the ids, the default video rate and the optics differ.
struct Model {
    /// Device description id (`devices/<id>.toml`).
    std::string_view id;
    /// Name shown to the user.
    std::string_view name;
    /// Product id of the HID chip (IF0..IF3).
    std::uint16_t pid;
    /// Product id of the display bridge MCU (ASCII log).
    std::uint16_t pid_bridge;
    /// Video rate Pimax Play announces at a cold start (`58 0d 1a 05 02 FPS`, keepalive rate byte).
    std::uint8_t default_fps;
    bool operator==(const Model&) const = default;
};

inline constexpr Model DREAM_AIR{"dream-air", "Pimax Dream Air", 0x0044, 0x0043, 72};
/// Windows capture 2026-10-17, take cold2: keepalive `58 04 39 00 5a 00`.
inline constexpr Model CRYSTAL_SUPER{"crystal-super", "Pimax Crystal Super", 0x0040, 0x0041, 90};
/// Every supported model, the Dream Air first.
inline constexpr std::array<Model, 2> MODELS{DREAM_AIR, CRYSTAL_SUPER};

/// The model with this HID-chip product id (nullptr when unknown).
const Model* model_by_pid(std::uint16_t pid);
/// The model whose display bridge has this product id.
const Model* model_by_bridge_pid(std::uint16_t pid);

// ------------------------------------------------------------------------------------------------ frames

enum class FrameError { TooLong };

/// Build `58 LL cmd opt data`, `LL = len + 3`, zero padded to 64 bytes.
std::expected<Report, FrameError> build_command(std::uint8_t cmd, std::uint8_t opt,
                                                std::span<const std::uint8_t> data);
/// Prefix the unnumbered report id 0, as required by a hidraw `write()`.
std::array<std::uint8_t, REPORT_SIZE + 1> to_write_buf(const Report& r);
/// Wake / "PC alive": `58 06 1a 05 01 00 00`.
Report wake();
/// Headset restart: `58 04 28 00 00`.
Report sys_reset();
/// Controller unpair: `58 03 05 SS`.
Report unpair(std::uint8_t slot);
/// Controller pairing (StartPair): `58 04 04 00 SS`.
Report start_pair(std::uint8_t slot);
/// The lens IPD motor to `mm` (clamped to 58..72 mm; the allowlist checks the model's range).
Report ipd_motor(float mm);
/// Keepalive in the exact wire form Windows uses: `58 04 39 00 RR 00` (LL = 4: the trailing zero is the high byte of
/// the u16 rate, outside LL).
Report keepalive(std::uint8_t rate);
/// Video preparation: `5b(02 02)`, `29(01)`, `1a(02 FPS)`, `1a(05 00)`.
std::array<Report, 4> video_prepare_commands(std::uint8_t fps);
/// Per-slot controller attach burst: `16(1e)`, `50`, `0a(00 00 00 10)`, `03`. The host spaces them by 10 ms.
std::array<Report, 4> attach_commands(std::uint8_t slot);

/// Duration games use for a streamed pulse (11-12 ms, re-sent every 8-10 ms).
inline constexpr std::uint16_t HAPTIC_MIN_MS = 12;
/// Controller vibration `58 09 07 SS AMP DUR_lo DUR_hi 00 00 00`; amp clamped to 1..100, duration at least 1 ms.
Report haptic_command(std::uint8_t slot, std::uint8_t amp, std::uint16_t dur_ms);
/// Round half away from zero for 0 <= x < 2^23 (bit-for-bit `f32::round` there).
std::uint32_t round_pos(float x);
/// OpenXR `XrHapticVibration` (amplitude 0..1, duration ns, -1 = minimum) to (amp %, ms); nullopt = stop.
std::optional<std::pair<std::uint8_t, std::uint16_t>> haptic_from_openxr(float amplitude, std::int64_t duration_ns);

/// A parsed `4c LL cmd opt data.. CK` reply (data points into the report).
struct Reply {
    std::uint8_t cmd;
    std::uint8_t opt;
    /// Payload without the trailing check byte (`LL - 3` bytes, clamped to the report).
    std::span<const std::uint8_t> data;
};
/// Parse a report from IF0. nullopt when it lacks the `4c` magic.
std::optional<Reply> parse_reply(const Report& r);
/// Serial reply data (`<len> <ASCII serial> [tail]`) to the ASCII serial bytes.
std::span<const std::uint8_t> parse_serial(std::span<const std::uint8_t> data);
/// True for controller slots 2 and 3.
bool is_slot(std::uint8_t s);

// ------------------------------------------------------------------------------------------------ IMU (IF2)

/// One IMU sample (a report holds two, 1 ms apart): `53 31 08 00 | ts u32 LE @4 | samples @12, @28 | ipd @45`.
struct ImuSample {
    /// Device clock, microseconds; the second sample is `ts + 1000`.
    std::uint32_t ts_us = 0;
    std::array<std::int16_t, 3> raw_accel{};
    std::array<std::int16_t, 3> raw_gyro{};
    /// `raw / 2048`.
    std::array<float, 3> accel_g{};
    /// `raw / 16.4`.
    std::array<float, 3> gyro_dps{};
    /// Live lens IPD in mm (bytes 45-46, u16 LE / 100).
    float ipd_mm = 0;
    /// 0 or 1.
    int sample_in_report = 0;
    bool operator==(const ImuSample&) const = default;
};
std::optional<std::array<ImuSample, 2>> decode_imu(const Report& r);

// ------------------------------------------------------------------------------------------------ 0x37 heartbeat

/// Unsolicited controller heartbeat on IF0: `4c 0f 37 SS 48 <u32> <batt%> 00 02 01 00 08 <fw>`; one packet may carry
/// two messages (the second at byte 16).
struct ControllerStatus {
    std::uint8_t slot = 0;
    std::uint8_t battery = 0;
    std::array<std::uint8_t, 2> fw{};
    Report raw{};
    bool operator==(const ControllerStatus&) const = default;
};
std::array<std::optional<ControllerStatus>, 2> parse_status(const Report& r);

// ------------------------------------------------------------------------------------------------ controller

/// Capacitive touch flags from byte 49 (corrected by the Windows redo take 5).
struct TouchBits {
    /// Bit 0. Bits 0 and 1 rise together for either face button: use thumb().
    bool a = false;
    /// Bit 1.
    bool b = false;
    /// Bit 3: thumb on the stick.
    bool stick = false;
    /// Bit 4: hand on the controller, on BOTH hands.
    bool hand_on = false;
    bool thumb() const { return a || b; }
    bool operator==(const TouchBits&) const = default;
};

/// Controller report: `53 32 08 SS 43 | sx sy trig | ts @8 | s0 @16 | s1 @30 | grip @44 | batt @46 | btn @47,48 |
/// touch @49`.
struct ControllerReport {
    std::uint8_t slot = 0;
    std::uint32_t ts_us = 0;
    /// -128..127, right positive.
    std::int32_t stick_x = 0;
    /// -128..127, up positive.
    std::int32_t stick_y = 0;
    std::uint8_t trigger = 0;
    std::uint8_t grip = 0;
    /// Percent: `byte46 & 0x7f`.
    std::uint8_t battery = 0;
    bool charging = false;
    /// `BTN_*` bits.
    std::uint32_t buttons = 0;
    /// Raw byte 49.
    std::uint8_t touch = 0;
    std::array<std::int16_t, 3> raw_accel{};
    std::array<std::int16_t, 3> raw_gyro{};
    Report raw{};

    TouchBits touch_bits() const;
    /// Names of the pressed buttons in `B, A, stick, menu` order.
    std::vector<std::string_view> pressed() const;
    bool operator==(const ControllerReport&) const = default;
};
std::optional<ControllerReport> decode_controller(const Report& r);

// ------------------------------------------------------------------------------------------------ firmware log (IF3)

/// Incremental line splitter. NUL and CR are dropped; a line is flushed on LF or when the buffer is full.
class LineSplitter {
public:
    static constexpr std::size_t MAX_LINE = 512;
    void reset() { len_ = 0; }
    /// Feed a chunk, calling `emit` once per non-empty line (bytes, no newline).
    void push(std::span<const std::uint8_t> data, const std::function<void(std::string_view)>& emit);

private:
    std::array<char, MAX_LINE> buf_{};
    std::size_t len_ = 0;
};

// ------------------------------------------------------------------------------------------------ calibration

/// On-device calibration document: 84 reads (`58 07 0a 00 01 AL AH 10`, addresses 0x0000..0x0530 step 0x10), each
/// answered by `4c 17 0a 00 01 AL AH 10 <16 B> CK`. The blob is `u32 LE zlib_len | 12 B | zlib stream | padding`.
namespace calib {
inline constexpr std::size_t BLOCK = 16;
inline constexpr std::size_t TOTAL = 0x540;
inline constexpr std::size_t BLOCKS = TOTAL / BLOCK;
inline constexpr std::size_t HEADER = 16;

enum class CalibError { Truncated, Inflate, Incomplete };

Report read_block_request(std::uint16_t addr);
std::optional<std::pair<std::uint16_t, std::array<std::uint8_t, BLOCK>>> parse_block_reply(const Report& r);
/// `blob[16 .. 16 + u32le(blob[0..4])]` inflated as zlib (limit 16 MiB). Needs zlib (Inflate error without it).
std::expected<std::vector<std::uint8_t>, CalibError> inflate_blob(std::span<const std::uint8_t> blob);
/// A device-style blob from an XML document (tests and tools). Empty without zlib.
std::vector<std::uint8_t> build_blob(std::span<const std::uint8_t> xml);
/// The reply the device would send for block `addr` of `blob` (fakes and tests).
Report make_block_reply(std::span<const std::uint8_t> blob, std::uint16_t addr);

/// Collects block replies in any order (and duplicates) into the blob.
class Assembler {
public:
    /// Feed any IF0 report; true if it was a block with a valid address.
    bool feed(const Report& r);
    bool complete() const;
    /// First block address not yet received.
    std::optional<std::uint16_t> next_missing() const;
    const std::array<std::uint8_t, TOTAL>& blob() const { return buf_; }
    std::expected<std::vector<std::uint8_t>, CalibError> document() const;

private:
    std::array<std::uint8_t, TOTAL> buf_{};
    std::array<bool, BLOCKS> got_{};
};
} // namespace calib

} // namespace pibridge::hmd
