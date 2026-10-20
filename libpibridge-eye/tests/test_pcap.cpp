// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/pcap.rs tests (MIT OR Apache-2.0).
#include "pibridge/eye/pcap.h"

#include "pibridge_test.h"
#include "testutil.h"

#include <cmath>

using namespace pibridge::eye::pcap;
using B = std::vector<std::uint8_t>;

namespace {
UsbPacket pk(double t, std::uint16_t dev, std::uint8_t ep, Transfer tr, bool comp, std::optional<std::uint8_t> stage,
             const B& data) {
    return UsbPacket::build(t, 2, dev, ep, tr, comp, stage, data, static_cast<std::uint32_t>(data.size()));
}

B header(std::uint32_t snaplen, std::uint32_t link) {
    B h = {0xd4, 0xc3, 0xb2, 0xa1};
    h.resize(16, 0);
    for (int s = 0; s < 32; s += 8) {
        h.push_back(static_cast<std::uint8_t>(snaplen >> s));
    }
    for (int s = 0; s < 32; s += 8) {
        h.push_back(static_cast<std::uint8_t>(link >> s));
    }
    return h;
}
} // namespace

TEST(write_then_read_roundtrip) {
    const std::vector<UsbPacket> pkts = {
        pk(1000.5, 30, 0x05, Transfer::Bulk, false, std::nullopt, {1, 2, 3}),
        pk(1000.75, 30, 0x83, Transfer::Bulk, true, std::nullopt, B(40, 9)),
        pk(1001.25, 30, 0x80, Transfer::Control, false, 0, {0xc1, 0x46, 1, 0, 0, 0, 8, 0}),
    };
    PcapWriter w(512);
    for (const auto& p : pkts) {
        w.write_packet(p);
    }
    auto r = PcapReader::from_bytes(w.bytes());
    CHECK(r.has_value());
    CHECK(r->snaplen() == 512);
    auto got = r->read_all();
    CHECK(got.has_value() && got->size() == 3);
    for (std::size_t i = 0; i < 3; i++) {
        const auto& a = pkts[i];
        const auto& b = (*got)[i];
        CHECK(std::ranges::equal(a.raw(), b.raw()));
        CHECK(std::ranges::equal(a.data(), b.data()));
        CHECK(a.device == b.device && a.endpoint == b.endpoint && a.transfer == b.transfer && a.stage == b.stage);
    }
    CHECK(std::fabs((*got)[1].time - 0.25) < 1e-6 && (*got)[0].time == 0.0);
    CHECK((*got)[2].stage == std::optional<std::uint8_t>(0));
    CHECK((*got)[1].is_completion() && (*got)[1].is_in() && !(*got)[0].is_in());
}

TEST(truncation_is_visible) {
    const auto p = UsbPacket::build(0.0, 2, 30, 0x05, Transfer::Bulk, false, std::nullopt, B(485, 0), 2803);
    CHECK(p.is_truncated());
    const auto q = pk(0.0, 30, 0x05, Transfer::Bulk, false, std::nullopt, B(20, 0));
    CHECK(!q.is_truncated());
}

TEST(rejects_foreign_files) {
    CHECK(PcapReader::from_bytes(B(24, 0)).error().kind == PcapError::Kind::BadMagic);
    const B h = header(65535, 1);
    const auto e = PcapReader::from_bytes(h);
    CHECK(!e && e.error().kind == PcapError::Kind::BadLinkType && e.error().link == 1);
    CHECK(!PcapReader::from_bytes(std::span<const std::uint8_t>(h).first(10)));
}

TEST(truncated_file_and_bad_records) {
    PcapWriter w(512);
    w.write_packet(pk(0.0, 30, 0x83, Transfer::Bulk, true, std::nullopt, B(100, 7)));
    const B& bytes = w.bytes();
    for (const std::size_t cut : {bytes.size() - 1, bytes.size() - 100, std::size_t{24 + 5}}) {
        auto r = PcapReader::from_bytes(std::span<const std::uint8_t>(bytes).first(cut));
        CHECK(r.has_value());
        const auto p = r->next_packet();
        CHECK(!p && p.error().kind == PcapError::Kind::Truncated);
    }
    // record shorter than a USBPcap header
    B h = header(512, LINKTYPE_USBPCAP);
    const B record = {0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0};
    h.insert(h.end(), record.begin(), record.end());
    auto r = PcapReader::from_bytes(h);
    const auto p = r->next_packet();
    CHECK(!p && p.error().kind == PcapError::Kind::BadRecord && p.error().index == 1);
}

TEST(isochronous_record_with_a_cut_header_is_accepted) {
    // header_len says 27 + 12 + 12*10 but only 40 bytes were captured
    B raw(40, 0);
    raw[0] = 159;
    raw[16] = 1;
    raw[22] = 0; // isochronous
    const std::uint32_t len = 1'600'000;
    for (int i = 0; i < 4; i++) {
        raw[static_cast<std::size_t>(23 + i)] = static_cast<std::uint8_t>(len >> (8 * i));
    }
    const auto p = UsbPacket::parse(raw, 1, 0, 0, 1'600'000, 0.0);
    CHECK(p.has_value());
    CHECK(p->transfer == Transfer::Isochronous);
    CHECK(p->data().empty());
    CHECK(p->is_truncated());
    // but a header shorter than the fixed part is still an error
    B raw2(40, 0);
    raw2[0] = 20;
    CHECK(!UsbPacket::parse(raw2, 2, 0, 0, 40, 0.0));
}

TEST(arbitrary_garbage_never_panics) {
    std::uint32_t x = 0x12345678u;
    for (std::size_t len = 0; len < 300; len++) {
        B b = header(512, LINKTYPE_USBPCAP);
        for (std::size_t i = 0; i < len; i++) {
            x = x * 1'664'525u + 1'013'904'223u;
            b.push_back(static_cast<std::uint8_t>(x >> 24));
        }
        if (auto r = PcapReader::from_bytes(b)) {
            for (int guard = 0; guard < 1000; guard++) {
                auto p = r->next_packet();
                if (!p || !*p) {
                    break;
                }
            }
        }
    }
    CHECK(true);
}

TEST(missing_file_is_an_io_error) {
    const auto r = PcapReader::open("/nonexistent/file.pcap");
    CHECK(!r && r.error().kind == PcapError::Kind::Io);
}

PIBRIDGE_TEST_MAIN()
