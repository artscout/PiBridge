// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/fake.rs (MIT OR Apache-2.0).
#include "pibridge/eye/fake.h"

#include "pibridge/eye/tlv.h"

#include <algorithm>
#include <thread>

namespace pibridge::eye::fake {

using namespace std::chrono_literals;

Bytes minimal_gaze_payload(std::uint32_t, std::int64_t) {
    Bytes items;
    tlv::push_compound(items, 0, TYPE_GAZE);
    Bytes p = {0, 0};
    p.insert(p.end(), items.begin(), items.end());
    return p;
}

FakeChip::FakeChip() = default;

Bytes FakeChip::notification() {
    Bytes body = payload ? payload(emitted, next_ts_) : Bytes{};
    emitted++;
    next_ts_ += NOMINAL_PERIOD_US;
    auto m = ttp::Message::make(ttp::Kind::Notification, 0, 0, ttp::STREAM_GAZE, {});
    m.payload = std::move(body);
    m.header.payload_len = static_cast<std::uint32_t>(m.payload.size());
    Bytes out;
    for (auto& f : m.to_frames(ttp::Direction::In, std::size_t{1} << 20)) {
        out.insert(out.end(), f.begin(), f.end());
    }
    return out;
}

void FakeChip::answer(const ttp::Message& req) {
    if (std::ranges::find(silent, req.header.id) != silent.end()) {
        return;
    }
    const auto it = status_override.find(req.header.id);
    const std::uint32_t status = it != status_override.end() ? it->second : 1;
    auto resp = ttp::Message::make(ttp::Kind::Response, req.header.seq, status, req.header.id, {});
    if (const auto p = reply_payload.find(req.header.id); p != reply_payload.end()) {
        resp.payload = p->second;
        resp.header.payload_len = static_cast<std::uint32_t>(resp.payload.size());
    }
    if (const auto a = reply_aux.find(req.header.id); a != reply_aux.end()) {
        resp.header.aux = a->second;
    }
    // one IN transfer per frame, as the chip sends a long answer
    for (auto& f : resp.to_frames(ttp::Direction::In, in_max_body)) {
        queue_.push_back(std::move(f));
    }
    if (req.header.id == 0x4ce) {
        streaming = false;
        // pending gaze frames go away with the subscription
        std::erase_if(queue_, [](const Bytes& f) { return f.size() >= 1000; });
    }
    if (start_on == req.header.id) {
        streaming = true;
    }
}

Result<Bytes> FakeChip::control(const Setup& setup, ByteView out, Duration) {
    sent.emplace_back(SentControl{setup, Bytes(out.begin(), out.end())});
    if ((setup[0] & 0x80) != 0) {
        const std::size_t w_length = static_cast<std::size_t>(setup[6] | (setup[7] << 8));
        return Bytes(std::min<std::size_t>(w_length, 8), 0);
    }
    return Bytes{};
}

Result<void> FakeChip::bulk_out(std::uint8_t ep, ByteView data, Duration) {
    sent.emplace_back(SentBulkOut{ep, Bytes(data.begin(), data.end())});
    for (auto& m : inbound_.push(data)) {
        if (!m) {
            return std::unexpected(Error::make(Errc::invalid_data, m.error().to_string()));
        }
        if (m->header.kind == ttp::Kind::Request) {
            answer(*m);
        }
    }
    return {};
}

Result<std::optional<Bytes>> FakeChip::bulk_in(std::uint8_t, std::size_t, Duration timeout) {
    if (!queue_.empty()) {
        Bytes f = std::move(queue_.front());
        queue_.pop_front();
        return std::optional<Bytes>(std::move(f));
    }
    if (streaming) {
        std::this_thread::sleep_for(std::min(period, timeout));
        return std::optional<Bytes>(notification());
    }
    std::this_thread::sleep_for(std::min<Duration>(timeout, 5ms));
    return std::optional<Bytes>{};
}

} // namespace pibridge::eye::fake
