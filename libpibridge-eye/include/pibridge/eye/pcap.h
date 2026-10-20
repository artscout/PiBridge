// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/pcap.rs (MIT OR Apache-2.0).
//
// Reader and writer for USBPcap captures (classic pcap, link type 249), as recorded by `capture.ps1` on Windows.
//
// USBPcap packet header (little endian, 27 bytes, 28 for control transfers which add the stage byte):
// `u16 header_len | u64 irp_id | u32 status | u16 function | u8 info | u16 bus | u16 device | u8 endpoint |
//  u8 transfer | u32 data_len [| u8 stage]`.
// `info` bit 0 set = completion (device to host direction of the IRP). `data_len` is the real transfer length; the
// captured bytes after the header may be shorter (snap length).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <iosfwd>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace pibridge::eye::pcap {

/// pcap link type of USBPcap.
inline constexpr std::uint32_t LINKTYPE_USBPCAP = 249;
/// Size of the fixed USBPcap header part.
inline constexpr std::size_t USBPCAP_HEADER_LEN = 27;

/// Pcap reading errors.
struct PcapError {
    enum class Kind {
        Io,          ///< I/O error (`message`)
        BadMagic,    ///< not a little-endian classic pcap file (`magic`)
        BadLinkType, ///< link type (`link`) other than USBPcap
        Truncated,   ///< the file ends inside a record
        BadRecord,   ///< record `index` (1-based) is malformed (`message` says why)
    };
    Kind kind = Kind::Io;
    std::string message;
    std::array<std::uint8_t, 4> magic{};
    std::uint32_t link = 0;
    std::uint64_t index = 0;

    std::string to_string() const;
};

/// USB transfer type (the wire byte; values other than 0-3 are kept as they are).
enum class Transfer : std::uint8_t {
    Isochronous = 0,
    Interrupt = 1,
    Control = 2,
    Bulk = 3,
};

/// One captured USB packet (an IRP submission or completion).
class UsbPacket {
public:
    /// 1-based record number in the file.
    std::uint64_t index = 0;
    /// Seconds since the first packet of the file.
    double time = 0;
    /// pcap record timestamp, seconds part.
    std::uint32_t ts_sec = 0;
    /// pcap record timestamp, microsecond or nanosecond part.
    std::uint32_t ts_frac = 0;
    /// pcap `orig_len`.
    std::uint32_t orig_len = 0;
    /// IRP id.
    std::uint64_t irp_id = 0;
    /// USBD status of the completion.
    std::uint32_t status = 0;
    /// URB function.
    std::uint16_t function = 0;
    /// Info byte; bit 0 = completion.
    std::uint8_t info = 0;
    /// Bus number.
    std::uint16_t bus = 0;
    /// Device address on that bus (re-assigned on every enumeration).
    std::uint16_t device = 0;
    /// Endpoint address including the direction bit (0x80 = IN).
    std::uint8_t endpoint = 0;
    /// Transfer type.
    Transfer transfer = Transfer::Bulk;
    /// Real length of the transfer data (may exceed the captured bytes).
    std::uint32_t data_len = 0;
    /// Control stage (0 setup, 1 data, 2 status, 3 complete); control transfers only.
    std::optional<std::uint8_t> stage;

    /// Bytes after the USBPcap header, as captured.
    std::span<const std::uint8_t> data() const;
    /// The whole captured packet, header included (what a pcap record contains).
    std::span<const std::uint8_t> raw() const { return raw_; }
    /// Completion (device to host direction of the IRP).
    bool is_completion() const { return (info & 1) != 0; }
    /// Endpoint direction is IN.
    bool is_in() const { return (endpoint & 0x80) != 0; }
    /// The capture holds fewer data bytes than the transfer length.
    bool is_truncated() const { return data().size() < data_len; }

    /// Parse the USBPcap header of a record body.
    static std::expected<UsbPacket, PcapError> parse(std::vector<std::uint8_t> raw, std::uint64_t index,
                                                     std::uint32_t ts_sec, std::uint32_t ts_frac,
                                                     std::uint32_t orig_len, double time);

    /// Build a packet from its parts (tests, fake devices, fixture slicing).
    static UsbPacket build(double time, std::uint16_t bus, std::uint16_t device, std::uint8_t endpoint,
                           Transfer transfer, bool completion, std::optional<std::uint8_t> stage,
                           std::span<const std::uint8_t> data, std::uint32_t data_len);

private:
    std::size_t header_len_ = USBPCAP_HEADER_LEN;
    std::vector<std::uint8_t> raw_;
};

/// Streaming reader over a USBPcap file.
class PcapReader {
public:
    /// Open a file and check the global header.
    static std::expected<PcapReader, PcapError> open(const std::filesystem::path& path);
    /// Read from bytes in memory (tests).
    static std::expected<PcapReader, PcapError> from_bytes(std::span<const std::uint8_t> bytes);

    PcapReader(PcapReader&&) noexcept;
    PcapReader& operator=(PcapReader&&) noexcept;
    ~PcapReader();

    /// Snap length of the capture (512 for most of ours, 65535 for `full-packets`).
    std::uint32_t snaplen() const { return snaplen_; }

    /// Next packet, nullopt at a clean end of file.
    std::expected<std::optional<UsbPacket>, PcapError> next_packet();

    /// Every remaining packet, or the first error.
    std::expected<std::vector<UsbPacket>, PcapError> read_all();

private:
    explicit PcapReader(std::unique_ptr<std::istream> in);
    std::optional<PcapError> read_header();

    std::unique_ptr<std::istream> in_;
    bool nanos_ = false;
    std::uint32_t snaplen_ = 0;
    std::uint64_t index_ = 0;
    std::optional<double> t0_;
};

/// Writer producing the same record layout (microsecond timestamps).
class PcapWriter {
public:
    /// Start a capture (the global header is buffered at once).
    explicit PcapWriter(std::uint32_t snaplen);
    /// Append a packet with its original timestamps (byte-exact for microsecond captures).
    void write_packet(const UsbPacket& p);
    /// The bytes written so far.
    const std::vector<std::uint8_t>& bytes() const { return out_; }
    /// Write the bytes to a file.
    std::expected<void, PcapError> save(const std::filesystem::path& path) const;

private:
    std::vector<std::uint8_t> out_;
};

} // namespace pibridge::eye::pcap
