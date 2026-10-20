// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/analysis.rs (MIT OR Apache-2.0).
#include "pibridge/eye/analysis.h"

#include <algorithm>
#include <format>

namespace pibridge::eye::analysis {

using pcap::Transfer;
using pcap::UsbPacket;
using replay::EP_BULK_IN;
using replay::EP_BULK_OUT;
using ttp::Direction;
using ttp::Kind;

std::optional<double> GazeSummary::capture_rate_hz() const {
    if (!span) {
        return std::nullopt;
    }
    const auto [a, b] = *span;
    if (frames > 1 && b > a) {
        return static_cast<double>(frames - 1) / (b - a);
    }
    return std::nullopt;
}

DeviceReport analyze_device(std::uint16_t device, std::span<const UsbPacket> packets, const GazeCheck& check) {
    DeviceReport r;
    r.device = device;
    r.packets = packets.size();
    for (const auto& p : packets) {
        if (p.transfer != Transfer::Bulk || p.data_len == 0) {
            continue;
        }
        Direction dir;
        if (p.endpoint == EP_BULK_OUT && !p.is_completion()) {
            dir = Direction::Out;
        } else if (p.endpoint == EP_BULK_IN && p.is_completion()) {
            dir = Direction::In;
        } else {
            continue;
        }
        const auto start = ttp::captured_start(dir, p.data());
        if (!start) {
            continue;
        }
        const auto& h = start->header;
        if (dir == Direction::In && h.kind == Kind::Notification && h.id == ttp::STREAM_GAZE) {
            auto& g = r.gaze;
            const auto w = ttp::Wrapper::parse(p.data());
            if (w && p.data_len != w->frame_len()) {
                g.oversized++;
                continue;
            }
            g.frames++;
            std::expected<void, std::string> ok;
            if (check) {
                ok = check(start->payload, start->complete);
            }
            if (ok) {
                g.cut_frames += start->complete ? 0 : 1;
                g.span = g.span ? std::pair{g.span->first, p.time} : std::pair{p.time, p.time};
            } else {
                g.parse_errors++;
                if (!g.first_error) {
                    g.first_error = ok.error();
                }
            }
            continue;
        }
        auto e = std::ranges::find_if(r.catalog, [&](const CatalogEntry& c) {
            return c.dir == dir && c.kind == h.kind && c.id == h.id;
        });
        if (e != r.catalog.end()) {
            e->count++;
            e->payload_len = {std::min(e->payload_len.first, h.payload_len),
                              std::max(e->payload_len.second, h.payload_len)};
            e->cut += start->complete ? 0 : 1;
        } else {
            r.catalog.push_back(CatalogEntry{dir, h.kind, h.id, 1, p.time, h.status, {h.payload_len, h.payload_len},
                                             start->complete ? 0u : 1u});
        }
    }
    r.plan = replay::build_plan(device, packets);
    return r;
}

std::expected<CaptureReport, pcap::PcapError> analyze_file(const std::filesystem::path& path,
                                                           std::optional<std::uint16_t> device,
                                                           const GazeCheck& check) {
    auto reader = pcap::PcapReader::open(path);
    if (!reader) {
        return std::unexpected(reader.error());
    }
    CaptureReport rep;
    rep.snaplen = reader->snaplen();
    replay::DeviceFinder finder;
    for (;;) {
        auto p = reader->next_packet();
        if (!p) {
            return std::unexpected(p.error());
        }
        if (!*p) {
            break;
        }
        rep.packets++;
        rep.duration = (*p)->time;
        if (!device) {
            finder.observe(**p);
        }
    }
    const std::vector<std::uint16_t> wanted = device ? std::vector<std::uint16_t>{*device} : finder.devices();
    std::vector<std::vector<UsbPacket>> kept(wanted.size());
    auto again = pcap::PcapReader::open(path);
    if (!again) {
        return std::unexpected(again.error());
    }
    for (;;) {
        auto p = again->next_packet();
        if (!p) {
            return std::unexpected(p.error());
        }
        if (!*p) {
            break;
        }
        const auto it = std::ranges::find(wanted, (*p)->device);
        if (it != wanted.end()) {
            kept[static_cast<std::size_t>(it - wanted.begin())].push_back(std::move(**p));
        }
    }
    for (std::size_t i = 0; i < wanted.size(); i++) {
        rep.devices.push_back(analyze_device(wanted[i], kept[i], check));
    }
    return rep;
}

std::string format_report(const CaptureReport& r, bool verbose_plan) {
    std::string o = std::format("snap length {} | {} packets | {:.1} s | {} TTP device(s)\n", r.snaplen, r.packets,
                                r.duration, r.devices.size());
    for (const auto& d : r.devices) {
        o += std::format("\n== device address {} ({} packets)\n", d.device, d.packets);
        if (!d.catalog.empty()) {
            o += "TTP messages (gaze notifications listed separately):\n";
            for (const auto& e : d.catalog) {
                const char* dir = e.dir == Direction::Out ? "OUT" : "IN ";
                const char* kind = e.kind == Kind::Request ? "req " : e.kind == Kind::Response ? "resp" : "note";
                const std::string cut = e.cut > 0 ? std::format(" (cut x{})", e.cut) : std::string();
                const std::string_view name =
                    e.dir == Direction::Out || e.kind == Kind::Response ? ttp::opcode_name(e.id) : "";
                o += std::format("  t={:>8.3} {} {} id={:#06x} x{:<4} status={} payload={}..{}{} {}\n", e.first_time,
                                 dir, kind, e.id, e.count, e.status, e.payload_len.first, e.payload_len.second, cut,
                                 name);
            }
        }
        const auto& g = d.gaze;
        if (g.frames > 0) {
            o += std::format("gaze stream: {} notifications ({} cut by the snap length, {} undecodable)\n", g.frames,
                             g.cut_frames, g.parse_errors);
            if (g.first_error) {
                o += std::format("  first decode error: {}\n", *g.first_error);
            }
            if (g.oversized > 0) {
                o += std::format("  {} oversized completions ignored (USB length differs from the frame length: "
                                 "stale 16 KiB read buffers)\n",
                                 g.oversized);
            }
            const auto rate = g.capture_rate_hz();
            o += std::format("  rate: {} by capture clock (per-eye states and the device-clock rate need the gaze "
                             "decoder, not in this tool yet)\n",
                             rate ? std::format("{:.2} Hz", *rate) : std::string("-"));
        }
        const auto& plan = d.plan;
        o += std::format("replay plan: {} steps, {} TTP requests, {} bytes missing -> {}{}\n", plan.steps.size(),
                         plan.requests().size(), plan.missing_bytes(),
                         plan.is_complete() ? "COMPLETE" : "NOT replayable verbatim",
                         !plan.stream_start && !plan.steps.empty()
                             ? " (but no gaze stream starts in this capture, so it is not a session start)"
                             : "");
        if (!plan.auth_steps().empty()) {
            o += "  contains the authentication exchange (0x76c/0x777/0x77b): the answers are bound to a device "
                 "challenge and may be refused on replay\n";
        }
        if (verbose_plan) {
            o += plan.describe();
        } else {
            for (const auto* s : plan.incomplete_steps()) {
                o += std::format("  incomplete: {} ({} of {} bytes missing)\n", s->label(), s->missing_bytes,
                                 s->total_bytes);
            }
        }
    }
    return o;
}

} // namespace pibridge::eye::analysis
