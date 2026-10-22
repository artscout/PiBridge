// SPDX-License-Identifier: MIT
#include "pibridge/hmd/p3b.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <system_error>

namespace pibridge::hmd::p3b {

namespace {

std::uint32_t rd_u32(std::span<const std::uint8_t> d, std::size_t o) {
    return std::uint32_t(d[o]) | std::uint32_t(d[o + 1]) << 8 | std::uint32_t(d[o + 2]) << 16 |
           std::uint32_t(d[o + 3]) << 24;
}

void wr_u32(Report& r, std::size_t o, std::uint32_t v) {
    for (int i = 0; i < 4; i++) {
        r[o + i] = static_cast<std::uint8_t>(v >> (8 * i));
    }
}

bool printable(std::string_view s) {
    for (const char c : s) {
        if (c < 0x20 || c > 0x7e) {
            return false;
        }
    }
    return !s.empty();
}

struct Entry {
    Cmd cmd;
    std::string_view name;
    Rpc rpc;
    std::string_view hex;
};

// name, the frame, and the captured bytes (used length; the rest of the 1024-byte report is zero)
const Entry kTable[] = {
    {Cmd::ScreenOn, "screen_on", {type::COMMAND, "setprop screen_state 1", ""},
     "0c160000000000000073657470726f702073637265656e5f73746174652031"},
    {Cmd::KeepRuntime, "keep_runtime", {type::COMMAND, "setprop control_signal_restart_runtime 0", ""},
     "0c280000000000000073657470726f7020636f6e74726f6c5f7369676e616c5f726573746172745f72756e74696d652030"},
    {Cmd::PcVideo120, "pc_video_120", {type::CALL, "piPcSwicthIoctl", "video 120"},
     "1c0f0000000900000070695063537769637468496f63746c766964656f20313230"},
    {Cmd::ConnectState, "connect_state", {type::CALL, "piPcSwicthIoctl", "getConnectState"},
     "1c0f0000000f00000070695063537769637468496f63746c676574436f6e6e6563745374617465"},
    {Cmd::GetLighthouseEnable, "get_lighthouse_enable", {type::GETPROP, "persist.sys.pmx.lighthouse.enable", ""},
     "3b2100000000000000706572736973742e7379732e706d782e6c69676874686f7573652e656e61626c65"},
    {Cmd::GetLighthouseSupport, "get_lighthouse_support", {type::GETPROP, "persist.sys.pmx.lighthouse.support", ""},
     "3b2200000000000000706572736973742e7379732e706d782e6c69676874686f7573652e737570706f7274"},
    {Cmd::LighthouseOn, "lighthouse_on", {type::COMMAND, "EnableLighthouse", "1"},
     "0c1000000001000000456e61626c654c69676874686f75736531"},
    {Cmd::LighthouseOff, "lighthouse_off", {type::COMMAND, "EnableLighthouse", "0"},
     "0c1000000001000000456e61626c654c69676874686f75736530"},
    {Cmd::LighthouseTakeEffect, "lighthouse_take_effect", {type::COMMAND, "LighthouseTakeEffect", "1"},
     "0c14000000010000004c69676874686f75736554616b6545666665637431"},
    {Cmd::Shutdown, "shutdown", {type::CALL, "shutDown", ""}, "1c080000000000000073687574446f776e"},
    {Cmd::TobiiEnable, "tobii_enable", {type::CALL, "setTobiiEyeDeviceEnable", "1"},
     "1c1700000001000000736574546f626969457965446576696365456e61626c6531"},
};

const Entry& entry(Cmd c) {
    for (const auto& e : kTable) {
        if (e.cmd == c) {
            return e;
        }
    }
    return kTable[0]; // unreachable: every Cmd is in the table (test checked)
}

} // namespace

std::optional<Report> encode(const Rpc& r) {
    if (9 + r.name.size() + r.value.size() > REPORT_SIZE) {
        return std::nullopt;
    }
    Report out{};
    out[0] = r.type;
    wr_u32(out, 1, static_cast<std::uint32_t>(r.name.size()));
    wr_u32(out, 5, static_cast<std::uint32_t>(r.value.size()));
    std::size_t o = 9;
    for (const char c : r.name) {
        out[o++] = static_cast<std::uint8_t>(c);
    }
    for (const char c : r.value) {
        out[o++] = static_cast<std::uint8_t>(c);
    }
    return out;
}

std::optional<Rpc> decode(std::span<const std::uint8_t> d) {
    if (d.size() < 9) {
        return std::nullopt;
    }
    const std::uint32_t nl = rd_u32(d, 1);
    const std::uint32_t vl = rd_u32(d, 5);
    if (nl == 0 || nl > 256 || vl > REPORT_SIZE || 9 + std::size_t(nl) > d.size()) {
        return std::nullopt;
    }
    Rpc r;
    r.type = d[0];
    r.name.assign(reinterpret_cast<const char*>(d.data() + 9), nl);
    if (!printable(r.name)) {
        return std::nullopt;
    }
    const std::size_t vend = std::min<std::size_t>(9 + std::size_t(nl) + vl, d.size());
    for (std::size_t i = 9 + nl; i < vend && d[i] != 0; i++) {
        r.value.push_back(static_cast<char>(d[i]));
    }
    return r;
}

float half_to_float(std::uint16_t h) {
    const int e = (h >> 10) & 0x1f;
    const int m = h & 0x3ff;
    const float sign = (h & 0x8000) ? -1.0f : 1.0f;
    if (e == 0) {
        return sign * std::ldexp(static_cast<float>(m), -24);
    }
    if (e == 31) {
        return m ? NAN : sign * INFINITY;
    }
    return sign * std::ldexp(static_cast<float>(m | 0x400), e - 25);
}

namespace {

float rd_f32(std::span<const std::uint8_t> d, std::size_t o) {
    float f;
    std::memcpy(&f, d.data() + o, 4);
    return f;
}

std::uint64_t rd_u64(std::span<const std::uint8_t> d, std::size_t o) {
    std::uint64_t v = 0;
    for (int i = 7; i >= 0; i--) {
        v = v << 8 | d[o + i];
    }
    return v;
}

Pose rd_pose(std::span<const std::uint8_t> d) {
    Pose p;
    p.t_ns = rd_u64(d, 1);
    for (int i = 0; i < 3; i++) {
        p.position[i] = rd_f32(d, 9 + 4 * i);
    }
    for (int i = 0; i < 4; i++) {
        p.orientation[i] = rd_f32(d, 21 + 4 * i);
    }
    return p;
}

} // namespace

std::optional<Head> decode_head(std::span<const std::uint8_t> d) {
    if (d.size() < 140 || (d[0] & 0x0f) != 1 || ((d[0] >> 4) != 3 && (d[0] >> 4) != 0)) {
        return std::nullopt;
    }
    Head h;
    h.tracked = (d[0] >> 4) == 3;
    h.pose = rd_pose(d);
    for (int i = 0; i < 3; i++) {
        h.linear_velocity[i] = rd_f32(d, 49 + 4 * i);
        h.angular_velocity[i] = rd_f32(d, 61 + 4 * i);
    }
    h.battery = d[109] <= 100 ? d[109] : -1;
    h.tracking = d[113];
    h.ipd_mm = half_to_float(static_cast<std::uint16_t>(d[122] | d[123] << 8));
    h.volume = d[132] <= 100 ? d[132] : -1;
    h.buttons = d[138];
    return h;
}

std::optional<Controller> decode_controller(std::span<const std::uint8_t> d) {
    const int dev = d.empty() ? 0 : (d[0] & 0x0f);
    const int st = d.empty() ? 0 : (d[0] >> 4);
    if (d.size() < 62 || (dev != 2 && dev != 3) || (st != 4 && st != 5 && st != 7)) {
        return std::nullopt;
    }
    Controller c;
    c.side = dev - 2;
    c.state = st;
    c.pose = rd_pose(d);
    c.stick = {static_cast<std::int8_t>(d[55]) / 127.0f, static_cast<std::int8_t>(d[56]) / 127.0f};
    c.trigger = d[57] / 255.0f;
    c.grip = d[58] / 255.0f;
    c.buttons = d[59];
    c.touches = d[60];
    c.battery = d[61] <= 100 ? d[61] : -1;
    return c;
}

const Rpc& command(Cmd c) { return entry(c).rpc; }
std::string_view captured_hex(Cmd c) { return entry(c).hex; }
std::string_view name_of(Cmd c) { return entry(c).name; }

std::optional<std::filesystem::path> find_hidraw(const discovery::Roots& roots) {
    char want[48];
    std::snprintf(want, sizeof want, "HID_ID=0003:%08X:%08X", VID, PID_CRYSTAL);
    std::error_code ec;
    for (std::filesystem::directory_iterator it(roots.hidraw_class, ec), end; !ec && it != end; it.increment(ec)) {
        const std::string n = it->path().filename().string();
        if (!n.starts_with("hidraw")) {
            continue;
        }
        std::ifstream f(roots.hidraw_class / n / "device/uevent");
        const std::string u((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        if (u.find(want) != std::string::npos) {
            return roots.dev / n;
        }
    }
    return std::nullopt;
}

} // namespace pibridge::hmd::p3b
