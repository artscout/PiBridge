// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-proto/src/{frame,model,imu,status,controller,log,calib}.rs (MIT OR Apache-2.0).
#include "pibridge/hmd/proto.h"

#include <algorithm>
#include <cstdlib>
#include <string_view>
#include <cmath>
#include <cstring>
#include <limits>

#if PIBRIDGE_HAVE_ZLIB
#include <zlib.h>
#endif

namespace pibridge::hmd {

namespace {

std::int16_t be16(const std::uint8_t* p) {
    return static_cast<std::int16_t>(static_cast<std::uint16_t>((p[0] << 8) | p[1]));
}

std::uint32_t le32(const std::uint8_t* p) {
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

// All callers pass short constant payloads.
Report fixed(std::uint8_t c, std::uint8_t opt, std::initializer_list<std::uint8_t> data) {
    const std::vector<std::uint8_t> d(data);
    return build_command(c, opt, d).value_or(Report{});
}

Report fixed(std::uint8_t c, std::uint8_t opt, std::span<const std::uint8_t> data) {
    return build_command(c, opt, data).value_or(Report{});
}

} // namespace

const Model* model_by_pid(std::uint16_t pid) {
    for (const auto& m : MODELS) {
        if (m.pid == pid) {
            return &m;
        }
    }
    return nullptr;
}

const Model* model_by_bridge_pid(std::uint16_t pid) {
    for (const auto& m : MODELS) {
        if (m.pid_bridge == pid) {
            return &m;
        }
    }
    return nullptr;
}

// ------------------------------------------------------------------------------------------------ frames

std::expected<Report, FrameError> build_command(std::uint8_t c, std::uint8_t opt, std::span<const std::uint8_t> data) {
    if (data.size() > MAX_DATA) {
        return std::unexpected(FrameError::TooLong);
    }
    Report r{};
    r[0] = 0x58;
    r[1] = static_cast<std::uint8_t>(data.size() + 3);
    r[2] = c;
    r[3] = opt;
    std::copy(data.begin(), data.end(), r.begin() + 4);
    return r;
}

std::array<std::uint8_t, REPORT_SIZE + 1> to_write_buf(const Report& r) {
    std::array<std::uint8_t, REPORT_SIZE + 1> b{};
    std::copy(r.begin(), r.end(), b.begin() + 1);
    return b;
}

Report wake() { return fixed(cmd::ANNOUNCE, 0x05, {0x01, 0x00, 0x00}); }

Report sys_reset() { return fixed(cmd::SYS_RESET, 0x00, {0x00}); }

Report unpair(std::uint8_t slot) { return fixed(cmd::UNPAIR, slot, std::span<const std::uint8_t>{}); }

Report start_pair(std::uint8_t slot) { return fixed(cmd::START_PAIR, 0x00, {slot}); }

Report keepalive(std::uint8_t rate) {
    Report r = fixed(cmd::KEEPALIVE, 0x00, {rate});
    r[5] = 0x00;
    return r;
}

std::array<Report, 4> video_prepare_commands(std::uint8_t fps) {
    std::array<std::uint8_t, 10> d1a{};
    d1a[0] = 0x02;
    d1a[1] = fps;
    std::array<std::uint8_t, 10> d1a_gpu{};
    d1a_gpu[0] = 0x05;
    return {fixed(cmd::APP_UNLOCK, 0x05, {0x02, 0x02}), fixed(cmd::PANEL_ON, 0x00, {0x01}),
            fixed(cmd::ANNOUNCE, 0x05, d1a), fixed(cmd::ANNOUNCE, 0x05, d1a_gpu)};
}

std::array<Report, 4> attach_commands(std::uint8_t slot) {
    return {fixed(cmd::SERIAL, slot, {0x1e}), fixed(cmd::INFO, slot, std::span<const std::uint8_t>{}),
            fixed(cmd::READ_DOC, slot, {0, 0, 0, 0x10}), fixed(cmd::VERSION, slot, std::span<const std::uint8_t>{})};
}

Report haptic_command(std::uint8_t slot, std::uint8_t amp, std::uint16_t dur_ms) {
    const std::uint16_t d = std::max<std::uint16_t>(dur_ms, 1);
    return fixed(cmd::HAPTIC, slot,
                 {std::clamp<std::uint8_t>(amp, 1, 100), static_cast<std::uint8_t>(d & 0xff),
                  static_cast<std::uint8_t>(d >> 8), 0, 0, 0});
}

std::uint32_t round_pos(float x) {
    const auto t = static_cast<std::uint32_t>(x);
    return x - static_cast<float>(t) >= 0.5f ? t + 1 : t;
}

std::optional<std::pair<std::uint8_t, std::uint16_t>> haptic_from_openxr(float amplitude, std::int64_t duration_ns) {
    if (std::isnan(amplitude) || amplitude <= 0.0f) {
        return std::nullopt;
    }
    // `as u8` saturates in Rust; the value is at most 100 here anyway
    const auto amp = static_cast<std::uint8_t>(
        std::clamp<std::uint32_t>(round_pos(std::min(amplitude, 1.0f) * 100.0f), 1, 100));
    std::uint16_t ms = HAPTIC_MIN_MS;
    if (duration_ns > 0) {
        ms = static_cast<std::uint16_t>(
            std::clamp<std::int64_t>((duration_ns + 999'999) / 1'000'000, 1, std::numeric_limits<std::uint16_t>::max()));
    }
    return std::pair{amp, ms};
}

std::optional<Reply> parse_reply(const Report& r) {
    if (r[0] != 0x4c) {
        return std::nullopt;
    }
    const std::size_t ll = r[1];
    const std::size_t dlen = std::min<std::size_t>(ll >= 3 ? ll - 3 : 0, REPORT_SIZE - 4);
    return Reply{r[2], r[3], std::span<const std::uint8_t>(r.data() + 4, dlen)};
}

std::span<const std::uint8_t> parse_serial(std::span<const std::uint8_t> data) {
    if (data.empty()) {
        return {};
    }
    const std::size_t n = std::min<std::size_t>(data[0], data.size() - 1);
    return data.subspan(1, n);
}

bool is_slot(std::uint8_t s) { return s == SLOT_LEFT || s == SLOT_RIGHT; }

// ------------------------------------------------------------------------------------------------ IMU

std::optional<std::array<ImuSample, 2>> decode_imu(const Report& r) {
    if (r[0] != 0x53 || r[1] != 0x31) {
        return std::nullopt;
    }
    const std::uint32_t ts = le32(&r[4]);
    const float ipd = static_cast<float>(static_cast<std::uint16_t>(r[45] | (r[46] << 8))) / 100.0f;
    std::array<ImuSample, 2> out{};
    for (std::size_t i = 0; i < 2; i++) {
        auto& o = out[i];
        const std::uint8_t* s = &r[12 + 16 * i + 2];
        for (std::size_t k = 0; k < 3; k++) {
            o.raw_accel[k] = be16(s + 2 * k);
            o.raw_gyro[k] = be16(s + 6 + 2 * k);
            o.accel_g[k] = static_cast<float>(o.raw_accel[k]) / 2048.0f;
            o.gyro_dps[k] = static_cast<float>(o.raw_gyro[k]) / 16.4f;
        }
        o.ts_us = ts + 1000u * static_cast<std::uint32_t>(i);
        o.ipd_mm = ipd;
        o.sample_in_report = static_cast<int>(i);
    }
    return out;
}

// ------------------------------------------------------------------------------------------------ status

std::array<std::optional<ControllerStatus>, 2> parse_status(const Report& r) {
    std::array<std::optional<ControllerStatus>, 2> out{};
    if (r[0] != 0x4c) {
        return out;
    }
    const std::size_t offs[2] = {0, 16};
    for (std::size_t i = 0; i < 2; i++) {
        const std::size_t off = offs[i];
        if (r[off + 2] == 0x37 && is_slot(r[off + 3])) {
            out[i] = ControllerStatus{r[off + 3], r[off + 9], {r[off + 14], r[off + 15]}, r};
        }
    }
    return out;
}

// ------------------------------------------------------------------------------------------------ controller

TouchBits ControllerReport::touch_bits() const {
    return TouchBits{(touch & 0x01) != 0, (touch & 0x02) != 0, (touch & 0x08) != 0, (touch & 0x10) != 0};
}

std::vector<std::string_view> ControllerReport::pressed() const {
    std::vector<std::string_view> v;
    const std::pair<std::uint32_t, std::string_view> names[] = {
        {BTN_B, "B"}, {BTN_A, "A"}, {BTN_APP_MENU, "menu-key"}, {BTN_STICK, "stick"}, {BTN_MENU, "menu"}};
    for (const auto& [m, n] : names) {
        if (buttons & m) {
            v.push_back(n);
        }
    }
    return v;
}

std::optional<ControllerReport> decode_controller(const Report& r) {
    if (r[0] != 0x53 || r[1] != 0x32 || r[2] != 0x08) {
        return std::nullopt;
    }
    if (r[3] != SLOT_LEFT && r[3] != SLOT_RIGHT) {
        return std::nullopt;
    }
    // the second sample is zero when the packet holds one
    const bool s0_nonzero = std::any_of(r.begin() + 16, r.begin() + 28, [](std::uint8_t b) { return b != 0; });
    const std::uint8_t* s = s0_nonzero ? &r[16] : &r[30];
    ControllerReport c;
    for (std::size_t k = 0; k < 3; k++) {
        c.raw_accel[k] = be16(s + 2 * k);
        c.raw_gyro[k] = be16(s + 6 + 2 * k);
    }
    // the Menu button (byte 47 bit 2) is PiBridge's: the Rust decoder masks it (PIBRIDGE_CTRL_PARITY=1 keeps that)
    static const bool parity = [] {
        const char* v = std::getenv("PIBRIDGE_CTRL_PARITY");
        return v != nullptr && (std::string_view(v) == "1" || std::string_view(v) == "on" || std::string_view(v) == "true");
    }();
    c.buttons = static_cast<std::uint32_t>(r[47]) & (BTN_B | BTN_A | BTN_STICK | (parity ? 0u : BTN_APP_MENU));
    if (r[48] & 1) {
        c.buttons |= BTN_MENU;
    }
    c.slot = r[3];
    c.ts_us = le32(&r[8]);
    c.stick_x = static_cast<std::int32_t>(r[5]) - 128;
    c.stick_y = static_cast<std::int32_t>(r[6]) - 128;
    c.trigger = r[7];
    c.grip = r[44];
    c.battery = r[46] & 0x7f;
    c.charging = (r[46] & 0x80) != 0;
    c.touch = r[49];
    c.raw = r;
    return c;
}

// ------------------------------------------------------------------------------------------------ log

void LineSplitter::push(std::span<const std::uint8_t> data, const std::function<void(std::string_view)>& emit) {
    for (const std::uint8_t c : data) {
        if (c == 0 || c == '\r') {
            continue;
        }
        if (c == '\n' || len_ >= MAX_LINE - 1) {
            if (len_ > 0) {
                emit(std::string_view(buf_.data(), len_));
            }
            len_ = 0;
            if (c == '\n') {
                continue;
            }
        }
        buf_[len_++] = static_cast<char>(c);
    }
}

// ------------------------------------------------------------------------------------------------ calibration

namespace calib {

Report read_block_request(std::uint16_t addr) {
    return fixed(cmd::READ_DOC, 0x00,
                 {0x01, static_cast<std::uint8_t>(addr & 0xff), static_cast<std::uint8_t>(addr >> 8), 0x10});
}

std::optional<std::pair<std::uint16_t, std::array<std::uint8_t, BLOCK>>> parse_block_reply(const Report& r) {
    if (r[0] != 0x4c || r[2] != cmd::READ_DOC || r[4] != 0x01) {
        return std::nullopt;
    }
    const auto addr = static_cast<std::uint16_t>(r[5] | (r[6] << 8));
    std::array<std::uint8_t, BLOCK> d{};
    std::copy(r.begin() + 8, r.begin() + 8 + BLOCK, d.begin());
    return std::pair{addr, d};
}

bool Assembler::feed(const Report& r) {
    const auto b = parse_block_reply(r);
    if (!b || b->first >= TOTAL || b->first % BLOCK != 0) {
        return false;
    }
    const std::size_t i = b->first / BLOCK;
    std::copy(b->second.begin(), b->second.end(), buf_.begin() + static_cast<std::ptrdiff_t>(i * BLOCK));
    got_[i] = true;
    return true;
}

bool Assembler::complete() const {
    return std::all_of(got_.begin(), got_.end(), [](bool g) { return g; });
}

std::optional<std::uint16_t> Assembler::next_missing() const {
    for (std::size_t i = 0; i < BLOCKS; i++) {
        if (!got_[i]) {
            return static_cast<std::uint16_t>(i * BLOCK);
        }
    }
    return std::nullopt;
}

std::expected<std::vector<std::uint8_t>, CalibError> Assembler::document() const {
    if (!complete()) {
        return std::unexpected(CalibError::Incomplete);
    }
    return inflate_blob(buf_);
}

std::expected<std::vector<std::uint8_t>, CalibError> inflate_blob(std::span<const std::uint8_t> blob) {
    if (blob.size() < HEADER) {
        return std::unexpected(CalibError::Truncated);
    }
    const std::size_t n = le32(blob.data());
    if (n > blob.size() - HEADER) {
        return std::unexpected(CalibError::Truncated);
    }
#if PIBRIDGE_HAVE_ZLIB
    constexpr std::size_t LIMIT = 16u << 20;
    z_stream z{};
    if (inflateInit(&z) != Z_OK) {
        return std::unexpected(CalibError::Inflate);
    }
    std::vector<std::uint8_t> out;
    std::array<std::uint8_t, 16384> chunk{};
    z.next_in = const_cast<Bytef*>(blob.data() + HEADER);
    z.avail_in = static_cast<uInt>(n);
    int rc = Z_OK;
    while (rc == Z_OK) {
        z.next_out = chunk.data();
        z.avail_out = static_cast<uInt>(chunk.size());
        rc = inflate(&z, Z_NO_FLUSH);
        if (rc != Z_OK && rc != Z_STREAM_END) {
            break;
        }
        out.insert(out.end(), chunk.begin(), chunk.begin() + static_cast<std::ptrdiff_t>(chunk.size() - z.avail_out));
        if (out.size() > LIMIT) {
            rc = Z_DATA_ERROR;
            break;
        }
        if (rc == Z_OK && z.avail_in == 0 && z.avail_out != 0) {
            rc = Z_BUF_ERROR; // input ended before the stream did
        }
    }
    inflateEnd(&z);
    if (rc != Z_STREAM_END) {
        return std::unexpected(CalibError::Inflate);
    }
    return out;
#else
    return std::unexpected(CalibError::Inflate);
#endif
}

std::vector<std::uint8_t> build_blob(std::span<const std::uint8_t> xml) {
#if PIBRIDGE_HAVE_ZLIB
    uLongf zlen = compressBound(static_cast<uLong>(xml.size()));
    std::vector<std::uint8_t> z(zlen);
    if (compress2(z.data(), &zlen, xml.data(), static_cast<uLong>(xml.size()), 6) != Z_OK) {
        return {};
    }
    z.resize(zlen);
    std::vector<std::uint8_t> b;
    const auto n = static_cast<std::uint32_t>(z.size());
    for (int i = 0; i < 4; i++) {
        b.push_back(static_cast<std::uint8_t>(n >> (8 * i)));
    }
    b.resize(HEADER, 0);
    b.insert(b.end(), z.begin(), z.end());
    if (b.size() < TOTAL) {
        b.resize(TOTAL, 0);
    }
    return b;
#else
    (void)xml;
    return {};
#endif
}

Report make_block_reply(std::span<const std::uint8_t> blob, std::uint16_t addr) {
    Report r{};
    r[0] = 0x4c;
    r[1] = 0x17;
    r[2] = cmd::READ_DOC;
    r[4] = 0x01;
    r[5] = static_cast<std::uint8_t>(addr & 0xff);
    r[6] = static_cast<std::uint8_t>(addr >> 8);
    r[7] = 0x10;
    for (std::size_t i = 0; i < BLOCK; i++) {
        const std::size_t a = addr + i;
        r[8 + i] = a < blob.size() ? blob[a] : 0;
    }
    return r;
}

} // namespace calib
} // namespace pibridge::hmd
