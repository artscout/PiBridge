// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/pcap.rs (MIT OR Apache-2.0).
#include "pibridge/eye/pcap.h"

#include "bytes.h"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <format>
#include <fstream>
#include <sstream>

namespace pibridge::eye::pcap {

namespace {
constexpr std::array<std::uint8_t, 4> MAGIC_US = {0xd4, 0xc3, 0xb2, 0xa1};
constexpr std::array<std::uint8_t, 4> MAGIC_NS = {0x4d, 0x3c, 0xb2, 0xa1};
constexpr std::size_t MAX_RECORD = std::size_t{1} << 26;

PcapError io_error(std::string m) {
    return PcapError{PcapError::Kind::Io, std::move(m), {}, 0, 0};
}

// Read up to n bytes; returns how many arrived (less only at the end of the stream).
std::size_t read_some(std::istream& in, std::uint8_t* p, std::size_t n) {
    in.read(reinterpret_cast<char*>(p), static_cast<std::streamsize>(n));
    return static_cast<std::size_t>(in.gcount());
}
} // namespace

std::string PcapError::to_string() const {
    switch (kind) {
    case Kind::Io: return std::format("I/O error: {}", message);
    case Kind::BadMagic:
        return std::format("not a little-endian pcap file (magic [{:02x}, {:02x}, {:02x}, {:02x}])", magic[0],
                           magic[1], magic[2], magic[3]);
    case Kind::BadLinkType: return std::format("pcap link type {}, expected 249 (USBPcap)", link);
    case Kind::Truncated: return "file ends inside a record";
    case Kind::BadRecord: return std::format("record {}: {}", index, message);
    }
    return "pcap error";
}

std::span<const std::uint8_t> UsbPacket::data() const {
    return std::span<const std::uint8_t>(raw_).subspan(header_len_);
}

std::expected<UsbPacket, PcapError> UsbPacket::parse(std::vector<std::uint8_t> raw, std::uint64_t index,
                                                     std::uint32_t ts_sec, std::uint32_t ts_frac,
                                                     std::uint32_t orig_len, double time) {
    const auto bad = [&](const char* reason) {
        return std::unexpected(PcapError{PcapError::Kind::BadRecord, reason, {}, 0, index});
    };
    if (raw.size() < USBPCAP_HEADER_LEN) {
        return bad("shorter than the USBPcap header");
    }
    const std::span<const std::uint8_t> r(raw);
    // Isochronous packets carry one descriptor per frame in the header, which a small snap length cuts: such a record
    // has no data bytes at all.
    std::size_t header_len = detail::le16(r, 0);
    if (header_len < USBPCAP_HEADER_LEN) {
        return bad("header length out of range");
    }
    header_len = std::min(header_len, raw.size());
    UsbPacket p;
    p.index = index;
    p.time = time;
    p.ts_sec = ts_sec;
    p.ts_frac = ts_frac;
    p.orig_len = orig_len;
    p.irp_id = detail::le64(r, 2);
    p.status = detail::le32(r, 10);
    p.function = detail::le16(r, 14);
    p.info = raw[16];
    p.bus = detail::le16(r, 17);
    p.device = detail::le16(r, 19);
    p.endpoint = raw[21];
    p.transfer = static_cast<Transfer>(raw[22]);
    p.data_len = detail::le32(r, 23);
    if (p.transfer == Transfer::Control && header_len > USBPCAP_HEADER_LEN) {
        p.stage = raw[USBPCAP_HEADER_LEN];
    }
    p.header_len_ = header_len;
    p.raw_ = std::move(raw);
    return p;
}

UsbPacket UsbPacket::build(double time, std::uint16_t bus, std::uint16_t device, std::uint8_t endpoint,
                           Transfer transfer, bool completion, std::optional<std::uint8_t> stage,
                           std::span<const std::uint8_t> data, std::uint32_t data_len) {
    const std::size_t header_len = stage ? USBPCAP_HEADER_LEN + 1 : USBPCAP_HEADER_LEN;
    // Written at fixed offsets into a sized buffer (the USBPcap header layout parse() reads): growing the vector with
    // push_back after reserve() trips GCC 15's -Wfree-nonheap-object false positive at -O2.
    std::vector<std::uint8_t> raw(header_len, 0);
    const auto put = [&raw](std::size_t at, std::uint64_t v, std::size_t n) {
        for (std::size_t i = 0; i < n; i++) {
            raw[at + i] = static_cast<std::uint8_t>(v >> (8 * i));
        }
    };
    put(0, header_len, 2);   // header length; irp id (8), status (4), function (2) stay 0
    raw[16] = completion ? 1 : 0;
    put(17, bus, 2);
    put(19, device, 2);
    raw[21] = endpoint;
    raw[22] = static_cast<std::uint8_t>(transfer);
    put(23, data_len, 4);
    if (stage) {
        raw[USBPCAP_HEADER_LEN] = *stage;
    }
    raw.insert(raw.end(), data.begin(), data.end());
    UsbPacket p;
    p.time = time;
    p.ts_sec = static_cast<std::uint32_t>(std::trunc(time));
    p.ts_frac = static_cast<std::uint32_t>(std::round((time - std::trunc(time)) * 1e6));
    p.orig_len = static_cast<std::uint32_t>(raw.size());
    p.info = completion ? 1 : 0;
    p.bus = bus;
    p.device = device;
    p.endpoint = endpoint;
    p.transfer = transfer;
    p.data_len = data_len;
    p.stage = stage;
    p.header_len_ = header_len;
    p.raw_ = std::move(raw);
    return p;
}

PcapReader::PcapReader(std::unique_ptr<std::istream> in) : in_(std::move(in)) {}
PcapReader::PcapReader(PcapReader&&) noexcept = default;
PcapReader& PcapReader::operator=(PcapReader&&) noexcept = default;
PcapReader::~PcapReader() = default;

std::optional<PcapError> PcapReader::read_header() {
    std::array<std::uint8_t, 24> h{};
    if (read_some(*in_, h.data(), h.size()) != h.size()) {
        return io_error("failed to fill whole buffer");
    }
    const std::array<std::uint8_t, 4> m = {h[0], h[1], h[2], h[3]};
    if (m == MAGIC_US) {
        nanos_ = false;
    } else if (m == MAGIC_NS) {
        nanos_ = true;
    } else {
        return PcapError{PcapError::Kind::BadMagic, {}, m, 0, 0};
    }
    snaplen_ = detail::le32(h, 16);
    const std::uint32_t link = detail::le32(h, 20);
    if (link != LINKTYPE_USBPCAP) {
        return PcapError{PcapError::Kind::BadLinkType, {}, {}, link, 0};
    }
    return std::nullopt;
}

std::expected<PcapReader, PcapError> PcapReader::open(const std::filesystem::path& path) {
    auto f = std::make_unique<std::ifstream>(path, std::ios::binary);
    if (!*f) {
        const int e = errno;
        return std::unexpected(io_error(std::format("{}: {}", path.string(), std::strerror(e != 0 ? e : ENOENT))));
    }
    PcapReader r(std::move(f));
    if (auto e = r.read_header()) {
        return std::unexpected(*e);
    }
    return r;
}

std::expected<PcapReader, PcapError> PcapReader::from_bytes(std::span<const std::uint8_t> bytes) {
    auto s = std::make_unique<std::istringstream>(
        std::string(reinterpret_cast<const char*>(bytes.data()), bytes.size()), std::ios::binary);
    PcapReader r(std::move(s));
    if (auto e = r.read_header()) {
        return std::unexpected(*e);
    }
    return r;
}

std::expected<std::optional<UsbPacket>, PcapError> PcapReader::next_packet() {
    std::array<std::uint8_t, 16> rh{};
    const std::size_t got = read_some(*in_, rh.data(), rh.size());
    if (got < rh.size()) {
        if (got == 0 && in_->eof()) {
            return std::optional<UsbPacket>{};
        }
        if (in_->bad()) {
            return std::unexpected(io_error("read failed"));
        }
        return std::unexpected(PcapError{PcapError::Kind::Truncated, {}, {}, 0, 0});
    }
    const std::uint32_t ts_sec = detail::le32(rh, 0);
    const std::uint32_t ts_frac = detail::le32(rh, 4);
    const std::size_t incl = detail::le32(rh, 8);
    const std::uint32_t orig = detail::le32(rh, 12);
    index_++;
    if (incl > MAX_RECORD) {
        return std::unexpected(PcapError{PcapError::Kind::BadRecord, "record length out of range", {}, 0, index_});
    }
    std::vector<std::uint8_t> body(incl);
    if (read_some(*in_, body.data(), incl) != incl) {
        if (in_->bad()) {
            return std::unexpected(io_error("read failed"));
        }
        return std::unexpected(PcapError{PcapError::Kind::Truncated, {}, {}, 0, 0});
    }
    const double div = nanos_ ? 1e9 : 1e6;
    const double abs = static_cast<double>(ts_sec) + static_cast<double>(ts_frac) / div;
    if (!t0_) {
        t0_ = abs;
    }
    auto p = UsbPacket::parse(std::move(body), index_, ts_sec, ts_frac, orig, abs - *t0_);
    if (!p) {
        return std::unexpected(p.error());
    }
    return std::optional<UsbPacket>(std::move(*p));
}

std::expected<std::vector<UsbPacket>, PcapError> PcapReader::read_all() {
    std::vector<UsbPacket> out;
    for (;;) {
        auto p = next_packet();
        if (!p) {
            return std::unexpected(p.error());
        }
        if (!*p) {
            return out;
        }
        out.push_back(std::move(**p));
    }
}

PcapWriter::PcapWriter(std::uint32_t snaplen) {
    out_.insert(out_.end(), MAGIC_US.begin(), MAGIC_US.end());
    detail::put_le16(out_, 2);
    detail::put_le16(out_, 4);
    detail::put_le32(out_, 0); // thiszone
    detail::put_le32(out_, 0); // sigfigs
    detail::put_le32(out_, snaplen);
    detail::put_le32(out_, LINKTYPE_USBPCAP);
}

void PcapWriter::write_packet(const UsbPacket& p) {
    detail::put_le32(out_, p.ts_sec);
    detail::put_le32(out_, p.ts_frac);
    detail::put_le32(out_, static_cast<std::uint32_t>(p.raw().size()));
    detail::put_le32(out_, p.orig_len);
    out_.insert(out_.end(), p.raw().begin(), p.raw().end());
}

std::expected<void, PcapError> PcapWriter::save(const std::filesystem::path& path) const {
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f.write(reinterpret_cast<const char*>(out_.data()), static_cast<std::streamsize>(out_.size()));
    if (!f) {
        return std::unexpected(io_error(std::format("{}: write failed", path.string())));
    }
    return {};
}

} // namespace pibridge::eye::pcap
