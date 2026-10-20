// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/replay.rs (MIT OR Apache-2.0).
#include "pibridge/eye/replay.h"

#include "pibridge/eye/ttp.h"

#include "bytes.h"

#include <format>

namespace pibridge::eye::replay {

using pcap::Transfer;
using pcap::UsbPacket;
using ttp::captured_start;
using ttp::Direction;

namespace {
std::uint16_t setup16(const Setup& s, std::size_t i) {
    return static_cast<std::uint16_t>(s[i] | (s[i + 1] << 8));
}
} // namespace

std::string Step::label() const {
    if (const auto* c = std::get_if<ControlOut>(&kind)) {
        return std::format("control OUT  bm={:#04x} req={:#04x} wValue={:#06x} wLength={}", c->setup[0], c->setup[1],
                           setup16(c->setup, 2), setup16(c->setup, 6));
    }
    if (const auto* c = std::get_if<ControlIn>(&kind)) {
        return std::format("control IN   bm={:#04x} req={:#04x} wValue={:#06x} wLength={}", c->setup[0], c->setup[1],
                           setup16(c->setup, 2), setup16(c->setup, 6));
    }
    const auto& r = std::get<Request>(kind);
    return std::format("TTP request  seq={:<3} op={:#05x} payload={:<6} frames={} {}", r.seq, r.opcode, r.payload_len,
                       r.frames.size(), ttp::opcode_name(r.opcode));
}

bool Plan::is_complete() const {
    if (steps.empty()) {
        return false;
    }
    for (const auto& s : steps) {
        if (!s.is_complete()) {
            return false;
        }
    }
    return true;
}

std::vector<const Step*> Plan::incomplete_steps() const {
    std::vector<const Step*> v;
    for (const auto& s : steps) {
        if (!s.is_complete()) {
            v.push_back(&s);
        }
    }
    return v;
}

std::uint64_t Plan::missing_bytes() const {
    std::uint64_t n = 0;
    for (const auto& s : steps) {
        n += s.missing_bytes;
    }
    return n;
}

std::vector<const Step*> Plan::requests() const {
    std::vector<const Step*> v;
    for (const auto& s : steps) {
        if (std::holds_alternative<Request>(s.kind)) {
            v.push_back(&s);
        }
    }
    return v;
}

std::vector<const Step*> Plan::auth_steps() const {
    std::vector<const Step*> v;
    for (const auto& s : steps) {
        if (const auto* r = std::get_if<Request>(&s.kind)) {
            if (r->opcode == 0x76c || r->opcode == 0x777 || r->opcode == 0x77b) {
                v.push_back(&s);
            }
        }
    }
    return v;
}

std::string Plan::describe() const {
    std::string out = std::format("device address {} | {} steps ({} TTP requests) | stream starts at {}\n", device,
                                  steps.size(), requests().size(),
                                  stream_start ? std::format("t={:.4}s", *stream_start) : std::string("never"));
    for (std::size_t i = 0; i < steps.size(); i++) {
        const Step& s = steps[i];
        const std::string state =
            s.is_complete() ? std::string("complete")
                            : std::format("INCOMPLETE: {} of {} bytes missing", s.missing_bytes, s.total_bytes);
        out += std::format("{:>3} t={:>9.4} rec={:<7} {:<95} {}\n", i, s.time, s.packet_index, s.label(), state);
    }
    return out;
}

void DeviceFinder::observe(const UsbPacket& p) {
    if (p.transfer != Transfer::Bulk || p.data_len == 0) {
        return;
    }
    const bool hit =
        (p.endpoint == EP_BULK_OUT && !p.is_completion() && captured_start(Direction::Out, p.data()).has_value()) ||
        (p.endpoint == EP_BULK_IN && p.is_completion() && captured_start(Direction::In, p.data()).has_value());
    if (hit) {
        found_.insert(p.device);
    }
}

std::vector<std::uint16_t> DeviceFinder::devices() const {
    return {found_.begin(), found_.end()};
}

std::vector<std::uint16_t> find_eye_devices(std::span<const UsbPacket> packets) {
    DeviceFinder f;
    for (const auto& p : packets) {
        f.observe(p);
    }
    return f.devices();
}

Plan build_plan(std::uint16_t device, std::span<const UsbPacket> packets) {
    struct Resp {
        std::size_t pos;
        std::uint32_t seq;
        Expect expect;
    };
    std::vector<Resp> responses;
    for (std::size_t pos = 0; pos < packets.size(); pos++) {
        const auto& p = packets[pos];
        if (p.transfer == Transfer::Bulk && p.endpoint == EP_BULK_IN && p.is_completion() && p.data_len > 0) {
            if (auto s = captured_start(Direction::In, p.data()); s && s->header.kind == ttp::Kind::Response) {
                responses.push_back({pos, s->header.seq, Expect{s->header.status, s->header.payload_len}});
            }
        }
    }

    Plan plan;
    plan.device = device;
    std::vector<std::size_t> step_pos;
    std::optional<std::pair<std::size_t, std::size_t>> open; // (step index, body bytes still expected)

    for (std::size_t pos = 0; pos < packets.size(); pos++) {
        const auto& p = packets[pos];
        if (p.transfer == Transfer::Control && p.stage == std::optional<std::uint8_t>(0) && !p.is_completion()) {
            const auto d = p.data();
            if (d.size() < 8 || (d[0] & 0x60) != 0x40) {
                continue; // short, or not a vendor request
            }
            Setup setup{};
            std::copy_n(d.begin(), 8, setup.begin());
            const std::size_t wlen = setup16(setup, 6);
            if ((d[0] & 0x80) != 0) {
                plan.steps.push_back(Step{p.index, p.time, ControlIn{setup}, 0, 8});
            } else {
                Bytes data(d.begin() + 8, d.end());
                const std::uint64_t missing = wlen > data.size() ? wlen - data.size() : 0;
                plan.steps.push_back(Step{p.index, p.time, ControlOut{setup, std::move(data)}, missing, 8 + wlen});
            }
            step_pos.push_back(pos);
        } else if (p.transfer == Transfer::Bulk && p.endpoint == EP_BULK_OUT && !p.is_completion() &&
                   p.data_len > 0) {
            const auto d = p.data();
            const std::uint64_t real = p.data_len;
            const std::uint64_t cut = real > d.size() ? real - d.size() : 0;
            if (open) {
                const auto [idx, remaining] = *open;
                const auto w = ttp::Wrapper::parse(d);
                const std::size_t body =
                    w ? w->body_len() : (p.data_len > 8 ? static_cast<std::size_t>(p.data_len) - 8 : 0);
                if (auto* r = std::get_if<Request>(&plan.steps[idx].kind)) {
                    r->frames.emplace_back(d.begin(), d.end());
                }
                plan.steps[idx].missing_bytes += cut;
                plan.steps[idx].total_bytes += real;
                const std::size_t left = remaining > body ? remaining - body : 0;
                open = left > 0 ? std::optional(std::pair{idx, left}) : std::nullopt;
            } else if (auto start = captured_start(Direction::Out, d)) {
                const auto w = ttp::Wrapper::parse(d);
                const std::size_t first_body = w ? w->body_len() : 0;
                const std::size_t total_body = ttp::HEADER_LEN + start->header.payload_len;
                const std::size_t left = total_body > first_body ? total_body - first_body : 0;
                Request r{start->header.seq, start->header.id, start->header.payload_len, {Bytes(d.begin(), d.end())},
                          std::nullopt};
                plan.steps.push_back(Step{p.index, p.time, std::move(r), cut, real});
                step_pos.push_back(pos);
                open = left > 0 ? std::optional(std::pair{plan.steps.size() - 1, left}) : std::nullopt;
            } else {
                plan.orphan_frames++;
            }
        } else if (p.transfer == Transfer::Bulk && p.endpoint == EP_BULK_IN && p.is_completion() && p.data_len > 0) {
            if (auto s = captured_start(Direction::In, p.data());
                s && s->header.kind == ttp::Kind::Notification && s->header.id == ttp::STREAM_GAZE) {
                plan.stream_start = p.time;
                break;
            }
        }
    }

    for (std::size_t i = 0; i < plan.steps.size(); i++) {
        if (auto* r = std::get_if<Request>(&plan.steps[i].kind)) {
            for (const auto& resp : responses) {
                if (resp.pos > step_pos[i] && resp.seq == r->seq) {
                    r->expect = resp.expect;
                    break;
                }
            }
        }
    }
    return plan;
}

} // namespace pibridge::eye::replay
