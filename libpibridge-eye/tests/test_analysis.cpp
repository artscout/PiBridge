// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/analysis.rs tests (MIT OR Apache-2.0), without the gaze decoding checks (eye
// states, device-clock rate, CSV), which belong to libpibridge-eyemath.
#include "pibridge/eye/analysis.h"

#include "pibridge_test.h"
#include "testutil.h"

using namespace pibridge::eye;
using namespace pibridge::eye::analysis;
using namespace testutil;

namespace {

UsbPacket gaze_frame(std::uint32_t n, std::optional<std::size_t> cut) {
    auto m = Message::make(Kind::Notification, 0, 0, ttp::STREAM_GAZE, {});
    // a 1218-byte payload, the size of the real gaze notifications (1242-byte frames)
    m.payload = Bytes(1218, static_cast<std::uint8_t>(n));
    m.payload[0] = m.payload[1] = 0;
    m.header.payload_len = static_cast<std::uint32_t>(m.payload.size());
    const Bytes f = concat(m.to_frames(Direction::In, std::size_t{1} << 20));
    const double t = 2.0 + n * 0.0083;
    const auto data = cut ? std::span<const std::uint8_t>(f).first(*cut) : std::span<const std::uint8_t>(f);
    return UsbPacket::build(t, 2, 30, replay::EP_BULK_IN, Transfer::Bulk, true, std::nullopt, data,
                            static_cast<std::uint32_t>(f.size()));
}

} // namespace

TEST(analysis_of_a_synthetic_capture) {
    const auto req = request(1, 0x4c4, 1).to_frames(Direction::Out, 4087)[0];
    std::vector<UsbPacket> pk = {bulk_out(1.0, req, req.size())};
    pk.push_back(bulk_in(1.001, response(1, 0x4c4).to_frames(Direction::In, 4087)[0]));
    for (std::uint32_t n = 0; n < 50; n++) {
        pk.push_back(gaze_frame(n, n % 10 == 9 ? std::optional<std::size_t>(485) : std::nullopt));
    }
    // a stale 16 KiB read buffer: USB length larger than the frame
    auto over = gaze_frame(99, std::nullopt);
    const Bytes od(over.data().begin(), over.data().end());
    pk.push_back(UsbPacket::build(3.0, 2, 30, replay::EP_BULK_IN, Transfer::Bulk, true, std::nullopt, od, 16384));
    const auto dir = temp_dir("analysis");
    const auto path = dir / "synthetic.usbpcap";
    pcap::PcapWriter w(65535);
    for (const auto& p : pk) {
        w.write_packet(p);
    }
    CHECK(w.save(path).has_value());
    const auto r = analyze_file(path, std::nullopt);
    CHECK(r.has_value());
    CHECK(r->snaplen == 65535);
    CHECK(r->devices.size() == 1);
    const auto& d = r->devices[0];
    CHECK(d.device == 30);
    CHECK(d.gaze.frames == 50);
    CHECK(d.gaze.cut_frames == 5);
    CHECK(d.gaze.parse_errors == 0);
    CHECK(d.gaze.oversized == 1);
    const auto rate = d.gaze.capture_rate_hz();
    CHECK(rate && std::fabs(*rate - 1.0 / 0.0083) < 0.5);
    CHECK(std::ranges::any_of(d.catalog, [](const CatalogEntry& e) { return e.id == 0x4c4 && e.kind == Kind::Request; }));
    CHECK(std::ranges::any_of(d.catalog, [](const CatalogEntry& e) { return e.id == 0x4c4 && e.kind == Kind::Response; }));
    CHECK(d.plan.requests().size() == 1);
    const auto text = format_report(*r, true);
    CHECK(text.find("gaze stream: 50 notifications") != std::string::npos);
    CHECK(text.find("1 oversized completions ignored") != std::string::npos);
    // explicit device filter
    const auto r7 = analyze_file(path, std::uint16_t{7});
    CHECK(r7 && r7->devices.size() == 1 && r7->devices[0].gaze.frames == 0);
    // a gaze check hook counts undecodable notifications
    const auto rc = analyze_file(path, std::nullopt, [](std::span<const std::uint8_t>, bool complete)
                                                         -> std::expected<void, std::string> {
        if (!complete) {
            return std::unexpected(std::string("cut"));
        }
        return {};
    });
    CHECK(rc && rc->devices[0].gaze.parse_errors == 5 && rc->devices[0].gaze.first_error == std::optional<std::string>("cut"));
    std::filesystem::remove_all(dir);
}

TEST(a_missing_capture_is_an_error) {
    CHECK(!analyze_file("/nonexistent/file.pcap", std::nullopt));
}

PIBRIDGE_TEST_MAIN()
