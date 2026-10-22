// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/broker.rs (MIT OR Apache-2.0).
//
// Camera frame broker: one process streams the camera, any number of local processes read every frame.
//
// V4L2 lets one process stream /dev/video0, but 6DoF head tracking (Monado), controller tracking (LED frames) and
// hand tracking all want the same four cameras. `pibridge-cam broker` (or omarchyvr's `pimax-cam broker`) opens the
// Source once and publishes each frame into a shared-memory ring; a client uses the source spec "broker" (or
// "broker:<socket>") and gets a BrokerSource, an ordinary Source.
//
// Protocol PXCB v1, byte-compatible with the Rust broker in both directions (Unix SOCK_SEQPACKET socket, default
// $XDG_RUNTIME_DIR/omarchyvr-cam-broker.sock, else /tmp/omarchyvr-cam-broker-<uid>.sock):
// * on connect the server sends one hello datagram (Hello) with the ring's memfd attached (SCM_RIGHTS):
//   magic "PXCB", then u32 LE version 1, slots, slot_bytes, data_bytes, flags (FLAG_REPLAY = 1, FLAG_PROFILE_JPEG2 = 2),
//   desc_len, then desc (<= 200 bytes);
// * then one 8-byte datagram per published frame: its index k (u64 LE); the frame sits in slot k % slots.
//
// Ring: a 4096-byte header page ("PXCB", u32 version at 4; u64 at 64 = newest complete index + 1, 72 = frames
// published, 80 = camera generation, 88 = camera state 0 unknown / 1 streaming / 2 reopening, 96 = heartbeat
// CLOCK_MONOTONIC ns), then the slots. Slot header (64 bytes): u64 seqlock at 0 (2k+1 while frame k is written, 2k+2
// once complete), u64 host ns at 8, u32 sequence at 16, u32 length at 20, u32 stride at 24, u64 k at 32; the frame
// at 64. A client that falls more than `slots` frames behind skips frames; nobody ever waits for a client
// (non-blocking sends: a full client socket just loses that notice). A second broker on the same socket is refused
// and a stale socket file is replaced.
#pragma once

#include "pibridge/cam/source.h"

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <tuple>
#include <vector>

namespace pibridge::cam::broker {

/// First bytes of the hello datagram and of the ring header.
inline constexpr char MAGIC[4] = {'P', 'X', 'C', 'B'};
/// Protocol / ring layout version.
inline constexpr std::uint32_t VERSION = 1;
/// Default number of slots: 8 frames = 160 ms at 50 fps.
inline constexpr std::uint32_t DEFAULT_SLOTS = 8;
/// Hello flag: the server's source is a recording.
inline constexpr std::uint32_t FLAG_REPLAY = 1;
/// Hello flag: the server's frames are the Motion-JPEG two-tile profile (source.h StreamProfile::JpegTiles2), so
/// clients read them the same way the broker does.
inline constexpr std::uint32_t FLAG_PROFILE_JPEG2 = 2;
/// Longest source description in a hello.
inline constexpr std::size_t MAX_DESC = 200;

/// Default socket: $XDG_RUNTIME_DIR/omarchyvr-cam-broker.sock (or /tmp/omarchyvr-cam-broker-<uid>.sock).
std::string default_socket();

/// What the server tells a new client.
struct Hello {
    std::uint32_t slots = 0;
    /// Bytes per slot (header + data, page multiple).
    std::uint32_t slot_bytes = 0;
    /// Largest frame a slot holds.
    std::uint32_t data_bytes = 0;
    /// FLAG_* bits.
    std::uint32_t flags = 0;
    /// The server's source, for logs.
    std::string desc;

    /// Wire form: magic, version, slots, slot_bytes, data_bytes, flags, desc_len (u32 LE), desc.
    std::vector<std::uint8_t> encode() const;
    /// Parse and validate a hello datagram.
    static Result<Hello> decode(std::span<const std::uint8_t> b);
    std::size_t ring_len() const;
    bool operator==(const Hello&) const = default;
};

/// Metadata of a slot read back by a client.
struct SlotInfo {
    std::uint64_t index = 0;
    std::uint64_t host_ts_ns = 0;
    std::uint32_t sequence = 0;
    std::size_t len = 0;
    std::size_t stride = 0;
};

/// The shared ring (memfd + mapping). The server maps it read-write, clients read-only.
class Ring {
public:
    /// Server side: a new memfd ring with slots slots of up to data_bytes each.
    static Result<std::unique_ptr<Ring>> create(std::uint32_t slots, std::size_t data_bytes, std::uint32_t flags,
                                                std::string_view desc);
    /// Client side: map a ring received from the server (read-only), after checking its size. Takes fd.
    static Result<std::unique_ptr<Ring>> open(int fd, Hello layout);
    ~Ring();
    Ring(const Ring&) = delete;
    Ring& operator=(const Ring&) = delete;

    const Hello& layout() const { return layout_; }
    /// The memfd (sent to clients).
    int fd() const { return fd_; }
    /// Newest complete frame index, if any.
    std::optional<std::uint64_t> latest() const;
    /// Frames published so far.
    std::uint64_t published() const;
    /// Server: record the camera state for clients and probes (up, its reopen generation) and the loop heartbeat.
    void set_camera(bool up, std::uint64_t generation, std::uint64_t now_ns) const;
    /// The server's camera state: (up or nullopt from a broker without the field, generation, heartbeat ns).
    std::tuple<std::optional<bool>, std::uint64_t, std::uint64_t> camera() const;
    /// Server: write frame k into its slot. Frames bigger than the slot are refused.
    Result<void> publish(std::uint64_t k, const RawFrame& f) const;
    /// Client: copy frame k into out (which must hold data_bytes). nullopt when the slot no longer (or not yet) holds
    /// frame k, or was overwritten during the copy.
    std::optional<SlotInfo> read(std::uint64_t k, std::span<std::uint8_t> out) const;

private:
    Ring() = default;
    std::uint64_t* word(std::size_t offset) const;
    std::size_t slot_offset(std::uint64_t k) const;

    std::uint8_t* ptr_ = nullptr;
    std::size_t len_ = 0;
    Hello layout_;
    int fd_ = -1;
};

/// Options of serve.
struct ServeOptions {
    std::string socket = default_socket();
    std::uint32_t slots = DEFAULT_SLOTS;
    /// How often on_status is called.
    std::chrono::steady_clock::duration status_every = std::chrono::seconds(5);
};

/// Counters of a running / finished broker.
struct ServeStats {
    std::uint64_t frames = 0;
    std::size_t clients = 0;
    std::uint64_t clients_total = 0;
    /// Frame notices not delivered because a client's socket was full (that client skips those frames).
    std::uint64_t notices_dropped = 0;
    /// Source read timeouts (no frame within 100 ms; live camera only).
    std::uint64_t timeouts = 0;
    /// The camera is streaming (false while it is being reopened).
    bool camera_up = false;
    std::uint64_t camera_reopens = 0;
    bool operator==(const ServeStats&) const = default;
};

/// Serve frames from src until stop is set, the source ends (replay without loop) or fails. The socket file is
/// removed on return. on_status is called every status_every with the counters (and at once when the camera goes
/// down or comes back).
Result<ServeStats> serve(std::unique_ptr<Source> src, const ServeOptions& opts, const std::atomic<bool>& stop,
                         const std::function<void(const ServeStats&)>& on_status);

/// A Source that reads frames from a running broker.
class BrokerSource final : public Source {
public:
    /// Connect to the broker at path and map its ring.
    static Result<std::unique_ptr<BrokerSource>> connect(const std::string& path);
    ~BrokerSource() override;

    /// Frames this client missed (it fell more than the ring behind, or its notices were dropped).
    std::uint64_t skipped() const { return skipped_; }
    const Hello& hello() const { return ring_->layout(); }
    std::uint64_t published() const { return ring_->published(); }
    std::tuple<std::optional<bool>, std::uint64_t, std::uint64_t> camera() const { return ring_->camera(); }

    Result<bool> advance(int timeout_ms) override;
    std::optional<RawFrame> current() const override;
    std::string describe() const override;
    bool finished() const override { return eof_ && is_replay(); }
    bool is_replay() const override { return (ring_->layout().flags & FLAG_REPLAY) != 0; }
    StreamProfile profile() const override {
        return (ring_->layout().flags & FLAG_PROFILE_JPEG2) != 0 ? StreamProfile::JpegTiles2
                                                                 : StreamProfile::GreyTiles4;
    }
    std::uint64_t generation() const override { return std::get<1>(ring_->camera()); }
    bool up() const override { return std::get<0>(ring_->camera()) != std::optional<bool>(false); }

private:
    BrokerSource() = default;
    Result<std::optional<std::uint64_t>> recv_notice();

    int sock_ = -1;
    std::unique_ptr<Ring> ring_;
    std::string path_;
    std::vector<std::uint8_t> buf_;
    std::optional<SlotInfo> cur_;
    std::uint64_t skipped_ = 0;
    bool eof_ = false;
};

/// What probe saw.
struct Probe {
    std::string desc;
    std::optional<bool> camera_up;
    std::uint64_t camera_reopens = 0;
    std::uint64_t published = 0;
    double fps = 0;
    /// Age of the server's loop heartbeat at the end of the window, s (nullopt: older broker).
    std::optional<double> heartbeat_age_s;
    /// One JSON object (the Rust text).
    std::string to_json() const;
};

/// Health probe of a running broker: connect (read-only client), count frames over window, read the camera state.
/// Sends nothing to the headset; the server just sees one short-lived client.
Result<Probe> probe(const std::string& path, std::chrono::steady_clock::duration window);

namespace detail {
/// Bind and listen (non-blocking); a stale socket file is replaced, a live one refused (AddrInUse). Returns the fd.
Result<int> listen(const std::string& path);
} // namespace detail

} // namespace pibridge::cam::broker
