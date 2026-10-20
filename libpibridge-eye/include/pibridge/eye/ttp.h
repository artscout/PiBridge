// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/ttp.rs (MIT OR Apache-2.0).
//
// TTP framing of the EyeChip bulk pipe (OUT EP 0x05, IN EP 0x83).
//
//   frame   = wrapper(8) | body
//   wrapper = u32 LE dir (0 = host->device, 1 = device->host) | u32 LE len
//             OUT: len = body length          (0x47 for a 79-byte transfer)
//             IN : len = whole frame length   (0x4da for a 1242-byte transfer)
//   message = header(24, big endian) | payload(payload_len)
//   header  = magic | seq | status | id | aux | payload_len
//             magic 0x51 request, 0x52 response, 0x53 notification
//             id    opcode (request/response) or stream id (notification, 0x0505 = gaze)
//             status 0 in requests, 1 in the responses seen so far
//   payload = 00 00 | TLV items          (empty, without the 2 zero bytes, when payload_len is 0)
//
// A message normally fits one frame. Larger ones are split into several frames, each with its own wrapper (OUT:
// 4087-byte bodies for the 34 612-byte profile upload; IN: the 0x44c reply is 35 + 34 437 + 164 bytes of body). Only the
// first frame carries the 24-byte header; the receiver counts payload bytes until `payload_len`. The wrapper `len`
// semantics differ between the directions; this was checked on every bulk transfer of the eight captures that contain
// the EyeChip (2 960 OUT and 44 000+ IN frames, no exception except the 16 KiB read buffers of the Windows stack in
// `poweron`, where the USB transfer is larger than the frame).
#pragma once

#include "pibridge/eye/tlv.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::eye::ttp {

/// Names of the request opcodes seen in the captures. Derived from payload shapes and timing, not from documentation:
/// treat them as labels, not as protocol truth. A trailing `?` marks a weaker guess.
std::string_view opcode_name(std::uint32_t op);

/// Wrapper size.
inline constexpr std::size_t WRAPPER_LEN = 8;
/// Header size.
inline constexpr std::size_t HEADER_LEN = 24;
/// Largest message payload accepted by the stream decoder (the biggest seen is 34 612 bytes).
inline constexpr std::size_t MAX_PAYLOAD_LEN = std::size_t{1} << 20;
/// Stream id of the 120 Hz gaze notifications.
inline constexpr std::uint32_t STREAM_GAZE = 0x0505;

/// Magic of requests.
inline constexpr std::uint32_t MAGIC_REQUEST = 0x51;
/// Magic of responses.
inline constexpr std::uint32_t MAGIC_RESPONSE = 0x52;
/// Magic of notifications.
inline constexpr std::uint32_t MAGIC_NOTIFICATION = 0x53;

/// Bulk direction as written in the wrapper.
enum class Direction : std::uint8_t {
    Out, ///< host to device (wire value 0)
    In,  ///< device to host (wire value 1)
};

/// Wire value.
std::uint32_t wire(Direction d);
/// From the wire value.
std::optional<Direction> direction_from_wire(std::uint32_t w);

/// Framing errors.
struct FrameError {
    enum class Kind {
        Short,          ///< not enough bytes for the structure (`needed`, `available`)
        BadDirection,   ///< wrapper direction word (`value`) is neither 0 nor 1
        WrongDirection, ///< wrapper direction differs from the decoder's (`dir` = the decoder's direction)
        BadLength,      ///< wrapper length (`value`) impossible (IN frames include the 8 wrapper bytes) or too large
        BadMagic,       ///< header magic (`value`) is not 0x51, 0x52 or 0x53
        TooLarge,       ///< header announces a payload (`value`) larger than MAX_PAYLOAD_LEN
    };
    Kind kind = Kind::Short;
    std::uint32_t value = 0;
    std::size_t needed = 0;
    std::size_t available = 0;
    Direction dir = Direction::Out;

    static FrameError short_(std::size_t needed, std::size_t available) {
        return {Kind::Short, 0, needed, available, Direction::Out};
    }
    static FrameError bad_direction(std::uint32_t d) { return {Kind::BadDirection, d, 0, 0, Direction::Out}; }
    static FrameError wrong_direction(Direction d) { return {Kind::WrongDirection, 0, 0, 0, d}; }
    static FrameError bad_length(std::uint32_t l) { return {Kind::BadLength, l, 0, 0, Direction::Out}; }
    static FrameError bad_magic(std::uint32_t m) { return {Kind::BadMagic, m, 0, 0, Direction::Out}; }
    static FrameError too_large(std::uint32_t l) { return {Kind::TooLarge, l, 0, 0, Direction::Out}; }

    bool operator==(const FrameError&) const = default;
    std::string to_string() const;
};

/// The 8-byte wrapper in front of every bulk transfer.
struct Wrapper {
    /// Direction word.
    Direction dir = Direction::Out;
    /// Length word (see the file comment for the direction-dependent meaning).
    std::uint32_t len = 0;

    /// Parse the first 8 bytes.
    static std::expected<Wrapper, FrameError> parse(std::span<const std::uint8_t> b);
    /// Wrapper for a frame carrying `body_len` body bytes.
    static Wrapper for_body(Direction dir, std::size_t body_len);
    /// Bytes after the wrapper.
    std::size_t body_len() const;
    /// Whole frame length on the bus, wrapper included.
    std::size_t frame_len() const;
    /// Wire bytes.
    std::array<std::uint8_t, WRAPPER_LEN> encode() const;

    bool operator==(const Wrapper&) const = default;
};

/// Message class from the header magic.
enum class Kind : std::uint8_t {
    Request,      ///< 0x51
    Response,     ///< 0x52
    Notification, ///< 0x53
};

/// Header magic of this kind.
std::uint32_t magic(Kind k);
/// Kind from a header magic.
std::optional<Kind> kind_from_magic(std::uint32_t m);

/// The 24-byte big-endian message header.
struct Header {
    /// Message class.
    Kind kind = Kind::Request;
    /// Sequence number: counts requests from 1; responses repeat it; notifications use 0.
    std::uint32_t seq = 0;
    /// 0 in requests; 1 in every response seen so far; 0 in notifications.
    std::uint32_t status = 0;
    /// Opcode (request/response) or stream id (notification).
    std::uint32_t id = 0;
    /// Word at offset 16: 0 everywhere except the responses to the calibration ops 0x407/0x42e (a handle-like value).
    std::uint32_t aux = 0;
    /// Payload length announced by the header (includes the 2-byte prefix).
    std::uint32_t payload_len = 0;

    /// Parse the first 24 bytes of a frame body.
    static std::expected<Header, FrameError> parse(std::span<const std::uint8_t> b);
    /// Wire bytes.
    std::array<std::uint8_t, HEADER_LEN> encode() const;

    bool operator==(const Header&) const = default;
};

/// A complete message.
struct Message {
    /// Header.
    Header header;
    /// Payload bytes: the 2-byte prefix and the TLV items. Empty when `payload_len` is 0.
    std::vector<std::uint8_t> payload;

    /// Build a message from TLV item bytes (adds the 2-byte prefix when there are items).
    static Message make(Kind kind, std::uint32_t seq, std::uint32_t status, std::uint32_t id,
                        std::span<const std::uint8_t> items);
    /// Header followed by payload.
    std::vector<std::uint8_t> body() const;
    /// The TLV items after the 2-byte prefix (a view: the message must outlive it).
    tlv::Items items() const;
    /// Wire bytes of the message split into frames of at most `max_body` body bytes (at least 1).
    std::vector<std::vector<std::uint8_t>> to_frames(Direction dir, std::size_t max_body) const;

    bool operator==(const Message&) const = default;
};

/// Incremental decoder for one direction of the bulk pipe. Feed it the bytes of consecutive USB transfers (any
/// chunking).
///
/// On a framing error the buffered bytes and any half-assembled message are dropped and the error is reported once;
/// decoding continues with the next bytes.
class StreamDecoder {
public:
    explicit StreamDecoder(Direction dir) : dir_(dir) {}

    /// Bytes buffered but not yet part of a complete frame.
    std::size_t buffered() const { return buf_.size(); }

    /// Feed bytes; returns the messages completed by them and the errors met.
    std::vector<std::expected<Message, FrameError>> push(std::span<const std::uint8_t> data);

private:
    struct Partial {
        Header header;
        std::vector<std::uint8_t> payload;
        std::size_t want = 0;
    };

    void reset();
    std::optional<FrameError> feed_body(std::span<const std::uint8_t> body,
                                        std::vector<std::expected<Message, FrameError>>& out);

    Direction dir_;
    std::vector<std::uint8_t> buf_;
    std::vector<std::uint8_t> hdr_;
    std::optional<Partial> partial_;
};

/// The start of a message found in one captured USB transfer, possibly cut by the capture snap length.
struct CapturedStart {
    /// Header.
    Header header;
    /// Payload bytes present in the transfer (a prefix of the real payload).
    std::vector<std::uint8_t> payload;
    /// Whether `payload` is the whole payload announced by the header.
    bool complete = false;

    bool operator==(const CapturedStart&) const = default;
};

/// Look for a message start at the beginning of a captured transfer. Returns nullopt for continuation transfers (no
/// valid wrapper of the expected direction or no valid header).
std::optional<CapturedStart> captured_start(Direction dir, std::span<const std::uint8_t> transfer);

} // namespace pibridge::eye::ttp
