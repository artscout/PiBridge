// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/replay.rs testutil (MIT OR Apache-2.0): packet and message builders, plus small
// helpers shared by the libpibridge-eye tests.
#pragma once

#include "pibridge/eye/fake.h"
#include "pibridge/eye/pcap.h"
#include "pibridge/eye/replay.h"
#include "pibridge/eye/tlv.h"
#include "pibridge/eye/ttp.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

namespace testutil {

using pibridge::eye::Bytes;
using pibridge::eye::Setup;
using pibridge::eye::pcap::Transfer;
using pibridge::eye::pcap::UsbPacket;
using pibridge::eye::ttp::Direction;
using pibridge::eye::ttp::Kind;
using pibridge::eye::ttp::Message;

inline UsbPacket bulk_out(double t, const Bytes& frame, std::size_t real) {
    return UsbPacket::build(t, 2, 30, pibridge::eye::replay::EP_BULK_OUT, Transfer::Bulk, false, std::nullopt, frame,
                            static_cast<std::uint32_t>(real));
}

inline UsbPacket bulk_out(double t, std::span<const std::uint8_t> frame, std::size_t real) {
    return UsbPacket::build(t, 2, 30, pibridge::eye::replay::EP_BULK_OUT, Transfer::Bulk, false, std::nullopt, frame,
                            static_cast<std::uint32_t>(real));
}

inline UsbPacket bulk_in(double t, const Bytes& frame) {
    return UsbPacket::build(t, 2, 30, pibridge::eye::replay::EP_BULK_IN, Transfer::Bulk, true, std::nullopt, frame,
                            static_cast<std::uint32_t>(frame.size()));
}

inline UsbPacket ctrl(double t, const Setup& setup, const Bytes& out) {
    Bytes d(setup.begin(), setup.end());
    d.insert(d.end(), out.begin(), out.end());
    const std::uint8_t ep = (setup[0] & 0x80) != 0 ? 0x80 : 0x00;
    return UsbPacket::build(t, 2, 30, ep, Transfer::Control, false, std::uint8_t{0}, d,
                            static_cast<std::uint32_t>(d.size()));
}

inline Message request(std::uint32_t seq, std::uint32_t op, std::size_t n_ints) {
    Bytes items;
    for (std::size_t i = 0; i < n_ints; i++) {
        pibridge::eye::tlv::push_i32(items, static_cast<std::int32_t>(i));
    }
    return Message::make(Kind::Request, seq, 0, op, items);
}

inline Message response(std::uint32_t seq, std::uint32_t op) {
    return Message::make(Kind::Response, seq, 1, op, {});
}

inline Message notification() {
    Bytes items;
    pibridge::eye::tlv::push_compound(items, 0, pibridge::eye::fake::TYPE_GAZE);
    return Message::make(Kind::Notification, 0, 0, pibridge::eye::ttp::STREAM_GAZE, items);
}

inline Bytes concat(const std::vector<Bytes>& v) {
    Bytes out;
    for (const auto& f : v) {
        out.insert(out.end(), f.begin(), f.end());
    }
    return out;
}

inline std::string read_text(const std::filesystem::path& p) {
    std::ifstream f(p, std::ios::binary);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

inline Bytes read_bytes(const std::filesystem::path& p) {
    const auto s = read_text(p);
    return Bytes(s.begin(), s.end());
}

inline void write_bytes(const std::filesystem::path& p, std::span<const std::uint8_t> b) {
    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    f.write(reinterpret_cast<const char*>(b.data()), static_cast<std::streamsize>(b.size()));
}

/// A fresh temporary directory for one test (never the shared-memory path of the live daemon).
inline std::filesystem::path temp_dir(const std::string& name) {
    const auto d = std::filesystem::temp_directory_path() / ("pibridge-eye-" + name + "-" + std::to_string(::getpid()));
    std::filesystem::remove_all(d);
    std::filesystem::create_directories(d);
    return d;
}

/// The Rust tests marked `cfg_attr(not(lab_data), ignore)` need private lab data that never enters this tree.
inline void skip_lab_data(const char* name) {
    std::printf("skip %s: needs the private lab data (real calibration, room recording or eye transcript)\n", name);
}

} // namespace testutil
