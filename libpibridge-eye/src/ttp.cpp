// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/ttp.rs (MIT OR Apache-2.0).
#include "pibridge/eye/ttp.h"

#include "bytes.h"

#include <algorithm>
#include <format>
#include <limits>

namespace pibridge::eye::ttp {

std::string_view opcode_name(std::uint32_t op) {
    switch (op) {
    case 0x3e8: return "hello / capability list?";
    case 0x640: return "get (three zero ints)?";
    case 0x76c: return "authentication step 1 (reply carries a 256-byte blob)";
    case 0x777: return "authentication step 2 (16-byte answer + 4-byte tag)";
    case 0x77b: return "authentication finish?";
    case 0x852: return "configuration upload (2771-byte payload)";
    case 0x532: return "get platform properties";
    case 0x58c: return "get ids (serial, platform, firmware)";
    case 0x546:
    case 0x547:
    case 0x5d2: return "get settings dump?";
    case 0x5cd: return "get (one int)?";
    case 0x4b0: return "get feature list";
    case 0x596:
    case 0x578: return "get table (zeros)?";
    case 0x686: return "get supported gaze rates";
    case 0x67c: return "set gaze rate";
    case 0x672: return "get gaze rate";
    case 0x5be: return "set eye offsets (+IPD/2,0,0,-IPD/2,0,0 mm)";
    case 0x4c4: return "subscribe stream";
    case 0x4ce: return "unsubscribe stream";
    case 0x456: return "upload user profile 'human' (34612-byte payload)";
    case 0x44c: return "download user profile";
    case 0x3f2: return "calibration: status/start?";
    case 0x407: return "calibration: add target point?";
    case 0x42e: return "calibration: compute?";
    case 0x3fc: return "calibration: finish?";
    default: return "unknown";
    }
}

std::string FrameError::to_string() const {
    switch (kind) {
    case Kind::Short: return std::format("need {} bytes, have {}", needed, available);
    case Kind::BadDirection: return std::format("bad wrapper direction word {:#x}", value);
    case Kind::WrongDirection:
        return std::format("unexpected wrapper direction {}", dir == Direction::Out ? "Out" : "In");
    case Kind::BadLength: return std::format("bad wrapper length {:#x}", value);
    case Kind::BadMagic: return std::format("bad TTP magic {:#x}", value);
    case Kind::TooLarge: return std::format("payload length {} exceeds the limit", value);
    }
    return "framing error";
}

std::uint32_t wire(Direction d) {
    return d == Direction::Out ? 0 : 1;
}

std::optional<Direction> direction_from_wire(std::uint32_t w) {
    switch (w) {
    case 0: return Direction::Out;
    case 1: return Direction::In;
    default: return std::nullopt;
    }
}

std::expected<Wrapper, FrameError> Wrapper::parse(std::span<const std::uint8_t> b) {
    if (b.size() < WRAPPER_LEN) {
        return std::unexpected(FrameError::short_(WRAPPER_LEN, b.size()));
    }
    const std::uint32_t d = detail::le32(b, 0);
    const std::uint32_t len = detail::le32(b, 4);
    const auto dir = direction_from_wire(d);
    if (!dir) {
        return std::unexpected(FrameError::bad_direction(d));
    }
    if (*dir == Direction::In && len < WRAPPER_LEN) {
        return std::unexpected(FrameError::bad_length(len));
    }
    return Wrapper{*dir, len};
}

Wrapper Wrapper::for_body(Direction dir, std::size_t body_len) {
    const std::size_t len = dir == Direction::Out ? body_len : body_len + WRAPPER_LEN;
    return Wrapper{dir, len > std::numeric_limits<std::uint32_t>::max() ? std::numeric_limits<std::uint32_t>::max()
                                                                         : static_cast<std::uint32_t>(len)};
}

std::size_t Wrapper::body_len() const {
    if (dir == Direction::Out) {
        return len;
    }
    return len < WRAPPER_LEN ? 0 : len - WRAPPER_LEN;
}

std::size_t Wrapper::frame_len() const {
    return body_len() + WRAPPER_LEN;
}

std::array<std::uint8_t, WRAPPER_LEN> Wrapper::encode() const {
    std::array<std::uint8_t, WRAPPER_LEN> o{};
    detail::store_le32(o.data(), wire(dir));
    detail::store_le32(o.data() + 4, len);
    return o;
}

std::uint32_t magic(Kind k) {
    switch (k) {
    case Kind::Request: return MAGIC_REQUEST;
    case Kind::Response: return MAGIC_RESPONSE;
    case Kind::Notification: return MAGIC_NOTIFICATION;
    }
    return 0;
}

std::optional<Kind> kind_from_magic(std::uint32_t m) {
    switch (m) {
    case MAGIC_REQUEST: return Kind::Request;
    case MAGIC_RESPONSE: return Kind::Response;
    case MAGIC_NOTIFICATION: return Kind::Notification;
    default: return std::nullopt;
    }
}

std::expected<Header, FrameError> Header::parse(std::span<const std::uint8_t> b) {
    if (b.size() < HEADER_LEN) {
        return std::unexpected(FrameError::short_(HEADER_LEN, b.size()));
    }
    const auto w = [&](std::size_t i) { return detail::be32(b, i); };
    const auto kind = kind_from_magic(w(0));
    if (!kind) {
        return std::unexpected(FrameError::bad_magic(w(0)));
    }
    return Header{*kind, w(4), w(8), w(12), w(16), w(20)};
}

std::array<std::uint8_t, HEADER_LEN> Header::encode() const {
    std::array<std::uint8_t, HEADER_LEN> o{};
    const std::uint32_t words[6] = {magic(kind), seq, status, id, aux, payload_len};
    for (std::size_t i = 0; i < 6; i++) {
        detail::store_be32(o.data() + i * 4, words[i]);
    }
    return o;
}

Message Message::make(Kind kind, std::uint32_t seq, std::uint32_t status, std::uint32_t id,
                      std::span<const std::uint8_t> items) {
    Message m;
    if (!items.empty()) {
        m.payload = {0, 0};
        m.payload.insert(m.payload.end(), items.begin(), items.end());
    }
    m.header = Header{kind, seq, status, id, 0, static_cast<std::uint32_t>(m.payload.size())};
    return m;
}

std::vector<std::uint8_t> Message::body() const {
    std::vector<std::uint8_t> b;
    b.reserve(HEADER_LEN + payload.size());
    const auto h = header.encode();
    b.insert(b.end(), h.begin(), h.end());
    b.insert(b.end(), payload.begin(), payload.end());
    return b;
}

tlv::Items Message::items() const {
    if (payload.size() < 2) {
        return tlv::Items(std::span<const std::uint8_t>{});
    }
    return tlv::Items(std::span<const std::uint8_t>(payload).subspan(2));
}

std::vector<std::vector<std::uint8_t>> Message::to_frames(Direction dir, std::size_t max_body) const {
    const auto b = body();
    const std::size_t step = std::max<std::size_t>(max_body, 1);
    std::vector<std::vector<std::uint8_t>> out;
    for (std::size_t off = 0; off < b.size(); off += step) {
        const std::size_t n = std::min(step, b.size() - off);
        const auto w = Wrapper::for_body(dir, n).encode();
        std::vector<std::uint8_t> f(w.begin(), w.end());
        f.insert(f.end(), b.begin() + static_cast<std::ptrdiff_t>(off),
                 b.begin() + static_cast<std::ptrdiff_t>(off + n));
        out.push_back(std::move(f));
    }
    return out;
}

void StreamDecoder::reset() {
    buf_.clear();
    hdr_.clear();
    partial_.reset();
}

std::vector<std::expected<Message, FrameError>> StreamDecoder::push(std::span<const std::uint8_t> data) {
    std::vector<std::expected<Message, FrameError>> out;
    buf_.insert(buf_.end(), data.begin(), data.end());
    while (buf_.size() >= WRAPPER_LEN) {
        auto w = Wrapper::parse(buf_);
        if (!w) {
            out.push_back(std::unexpected(w.error()));
            reset();
            break;
        }
        if (w->dir != dir_) {
            out.push_back(std::unexpected(FrameError::wrong_direction(dir_)));
            reset();
            break;
        }
        const std::size_t total = w->frame_len();
        if (total > MAX_PAYLOAD_LEN + HEADER_LEN + WRAPPER_LEN) {
            out.push_back(std::unexpected(FrameError::bad_length(w->len)));
            reset();
            break;
        }
        if (buf_.size() < total) {
            break;
        }
        std::vector<std::uint8_t> body(buf_.begin() + WRAPPER_LEN, buf_.begin() + static_cast<std::ptrdiff_t>(total));
        buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(total));
        if (auto e = feed_body(body, out)) {
            out.push_back(std::unexpected(*e));
            reset();
            break;
        }
    }
    return out;
}

std::optional<FrameError> StreamDecoder::feed_body(std::span<const std::uint8_t> body,
                                                   std::vector<std::expected<Message, FrameError>>& out) {
    while (!body.empty()) {
        if (partial_) {
            Partial& p = *partial_;
            const std::size_t take = std::min(p.want - p.payload.size(), body.size());
            p.payload.insert(p.payload.end(), body.begin(), body.begin() + static_cast<std::ptrdiff_t>(take));
            body = body.subspan(take);
            if (p.payload.size() == p.want) {
                out.push_back(Message{p.header, std::move(p.payload)});
                partial_.reset();
            }
            continue;
        }
        // The header normally sits whole in the first frame, but accept one split over several frames.
        const std::size_t take = std::min(HEADER_LEN - hdr_.size(), body.size());
        hdr_.insert(hdr_.end(), body.begin(), body.begin() + static_cast<std::ptrdiff_t>(take));
        body = body.subspan(take);
        if (hdr_.size() < HEADER_LEN) {
            break;
        }
        auto header = Header::parse(hdr_);
        if (!header) {
            return header.error();
        }
        hdr_.clear();
        const std::size_t want = header->payload_len;
        if (want > MAX_PAYLOAD_LEN) {
            return FrameError::too_large(header->payload_len);
        }
        if (want == 0) {
            out.push_back(Message{*header, {}});
        } else {
            Partial p{*header, {}, want};
            p.payload.reserve(want);
            partial_ = std::move(p);
        }
    }
    return std::nullopt;
}

std::optional<CapturedStart> captured_start(Direction dir, std::span<const std::uint8_t> transfer) {
    const auto w = Wrapper::parse(transfer);
    if (!w || w->dir != dir) {
        return std::nullopt;
    }
    const auto body = transfer.subspan(WRAPPER_LEN);
    const auto header = Header::parse(body);
    if (!header) {
        return std::nullopt;
    }
    const auto rest = body.subspan(HEADER_LEN);
    const std::size_t want = header->payload_len;
    // The frame may end before the payload does (multi-frame message) or the capture may have cut the transfer.
    const std::size_t in_frame = w->body_len() < HEADER_LEN ? 0 : w->body_len() - HEADER_LEN;
    const std::size_t take = std::min({want, rest.size(), in_frame});
    return CapturedStart{*header, std::vector<std::uint8_t>(rest.begin(), rest.begin() + static_cast<std::ptrdiff_t>(take)),
                         take == want};
}

} // namespace pibridge::eye::ttp
