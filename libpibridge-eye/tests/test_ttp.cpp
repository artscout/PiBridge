// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/ttp.rs tests (MIT OR Apache-2.0).
#include "pibridge/eye/ttp.h"

#include "pibridge_test.h"

#include <cstdint>
#include <expected>
#include <string>
#include <vector>

using namespace pibridge::eye::ttp;
using namespace pibridge::eye;
using B = std::vector<std::uint8_t>;
using R = std::expected<Message, FrameError>;

namespace {

Message sample_message(std::uint32_t seq, std::size_t n_items) {
    B items;
    for (std::size_t i = 0; i < n_items; i++) {
        tlv::push_i32(items, static_cast<std::int32_t>(i));
    }
    return Message::make(Kind::Request, seq, 0, 0x4c4, items);
}

B hex(const std::string& s) {
    B out;
    for (std::size_t i = 0; i + 1 < s.size(); i += 2) {
        out.push_back(static_cast<std::uint8_t>(std::stoul(s.substr(i, 2), nullptr, 16)));
    }
    return out;
}

B concat(const std::vector<B>& v) {
    B out;
    for (const auto& f : v) {
        out.insert(out.end(), f.begin(), f.end());
    }
    return out;
}

} // namespace

TEST(wrapper_semantics_match_the_captures) {
    // OUT 79-byte transfer: len = 0x47; IN 1242-byte transfer: len = 0x4da
    const auto out = Wrapper::parse(B{0, 0, 0, 0, 0x47, 0, 0, 0});
    CHECK(out && out->dir == Direction::Out && out->body_len() == 71 && out->frame_len() == 79);
    const auto inn = Wrapper::parse(B{1, 0, 0, 0, 0xda, 0x04, 0, 0});
    CHECK(inn && inn->dir == Direction::In && inn->body_len() == 1234 && inn->frame_len() == 1242);
    CHECK((inn->encode() == std::array<std::uint8_t, 8>{1, 0, 0, 0, 0xda, 4, 0, 0}));
    CHECK(Wrapper::for_body(Direction::In, 1234) == *inn);
    CHECK(Wrapper::for_body(Direction::Out, 71) == *out);
}

TEST(wrapper_rejects_garbage) {
    CHECK(Wrapper::parse(B{2, 0, 0, 0, 8, 0, 0, 0}).error() == FrameError::bad_direction(2));
    CHECK(Wrapper::parse(B{1, 0, 0, 0, 7, 0, 0, 0}).error() == FrameError::bad_length(7));
    CHECK(Wrapper::parse(B(7, 0)).error().kind == FrameError::Kind::Short);
}

TEST(real_request_header_roundtrip) {
    // seq 20, opcode 0x4c4 from the cold-start capture: 0x4c4 (stream 0x0505, empty option array)
    const B wire = hex("000000002c000000000000510000001400000000000004c4000000000000001400000200000004000005051700000004"
                       "00000000");
    const auto w = Wrapper::parse(wire);
    CHECK(w && w->frame_len() == wire.size());
    const auto h = Header::parse(std::span<const std::uint8_t>(wire).subspan(WRAPPER_LEN));
    CHECK(h && *h == (Header{Kind::Request, 20, 0, 0x4c4, 0, 20}));
    const auto enc = h->encode();
    CHECK(std::ranges::equal(enc, std::span<const std::uint8_t>(wire).subspan(WRAPPER_LEN, HEADER_LEN)));
    StreamDecoder dec(Direction::Out);
    const auto msgs = dec.push(wire);
    CHECK(msgs.size() == 1);
    const auto& m = *msgs[0];
    auto items = m.items().collect();
    CHECK(items.size() == 2);
    CHECK(items[0]->tag == tlv::tag::INT32 && items[1]->tag == tlv::tag::ARRAY);
}

TEST(multi_frame_message_is_reassembled) {
    const auto m = sample_message(9, 40);
    for (const auto dir : {Direction::Out, Direction::In}) {
        const B wire = concat(m.to_frames(dir, 17)); // header split over two frames
        StreamDecoder dec(dir);
        const auto got = dec.push(wire);
        CHECK(got == std::vector<R>{m});
        CHECK(dec.buffered() == 0);
    }
}

TEST(byte_by_byte_feed_gives_the_same_messages) {
    std::vector<Message> msgs;
    for (std::uint32_t i = 0; i < 5; i++) {
        msgs.push_back(sample_message(i, i * 3));
    }
    B wire;
    for (const auto& m : msgs) {
        const B f = concat(m.to_frames(Direction::In, 30));
        wire.insert(wire.end(), f.begin(), f.end());
    }
    StreamDecoder dec(Direction::In);
    std::vector<Message> got;
    for (const auto b : wire) {
        for (auto& r : dec.push(std::span<const std::uint8_t>(&b, 1))) {
            CHECK(r.has_value());
            got.push_back(*r);
        }
    }
    CHECK(got == msgs);
}

TEST(two_messages_in_one_frame_are_split) {
    const auto a = sample_message(1, 2);
    const auto b = sample_message(2, 3);
    B body = a.body();
    const B bb = b.body();
    body.insert(body.end(), bb.begin(), bb.end());
    const auto w = Wrapper::for_body(Direction::In, body.size()).encode();
    B wire(w.begin(), w.end());
    wire.insert(wire.end(), body.begin(), body.end());
    StreamDecoder dec(Direction::In);
    CHECK(dec.push(wire) == (std::vector<R>{a, b}));
}

TEST(errors_reset_and_decoding_continues) {
    const auto good = sample_message(3, 1);
    StreamDecoder dec(Direction::In);
    // wrong direction wrapper
    CHECK(dec.push(B{0, 0, 0, 0, 8, 0, 0, 0}) == std::vector<R>{std::unexpected(FrameError::wrong_direction(Direction::In))});
    // bad magic
    auto w = Wrapper::for_body(Direction::In, 24).encode();
    B bad(w.begin(), w.end());
    bad.resize(bad.size() + 24, 0);
    CHECK(dec.push(bad) == std::vector<R>{std::unexpected(FrameError::bad_magic(0))});
    // header larger than the limit
    Header h = good.header;
    h.payload_len = static_cast<std::uint32_t>(MAX_PAYLOAD_LEN + 1);
    B big(w.begin(), w.end());
    const auto he = h.encode();
    big.insert(big.end(), he.begin(), he.end());
    CHECK(dec.push(big) == std::vector<R>{std::unexpected(FrameError::too_large(h.payload_len))});
    // and a good one afterwards
    CHECK(dec.push(concat(good.to_frames(Direction::In, 1000))) == std::vector<R>{good});
}

TEST(captured_start_handles_truncation_and_continuations) {
    const auto m = sample_message(4, 200); // payload 1802 bytes
    const B wire = concat(m.to_frames(Direction::In, 100'000));
    const auto cut = std::span<const std::uint8_t>(wire).first(485);
    const auto s = captured_start(Direction::In, cut);
    CHECK(s && s->header == m.header && !s->complete);
    CHECK(std::ranges::equal(s->payload, std::span<const std::uint8_t>(m.payload).first(485 - WRAPPER_LEN - HEADER_LEN)));
    const auto full = captured_start(Direction::In, wire);
    CHECK(full && full->complete && full->payload == m.payload);
    // multi-frame: the first frame holds the header and 11 payload bytes only
    const auto frames = m.to_frames(Direction::In, HEADER_LEN + 11);
    const auto first = captured_start(Direction::In, frames[0]);
    CHECK(first && !first->complete && first->payload.size() == 11);
    // a continuation frame does not start with a header
    CHECK(!captured_start(Direction::In, frames[1]));
    CHECK(!captured_start(Direction::Out, wire));
}

// From tests/props.rs (decoder_is_total_and_bounded_on_garbage, decoder_recovers_after_a_reported_error,
// captured_start_is_total, valid streams with any chunking), with a fixed pseudo-random sample instead of proptest.
TEST(decoder_is_total_on_garbage_and_recovers) {
    std::uint32_t x = 12345;
    const auto rnd = [&] {
        x = x * 1664525u + 1013904223u;
        return x >> 24;
    };
    for (int round = 0; round < 200; round++) {
        B garbage(rnd() + 1);
        for (auto& c : garbage) {
            c = static_cast<std::uint8_t>(rnd());
        }
        for (const auto dir : {Direction::In, Direction::Out}) {
            (void)captured_start(dir, garbage);
            StreamDecoder dec(dir);
            (void)dec.push(garbage);
            CHECK(dec.buffered() <= MAX_PAYLOAD_LEN + HEADER_LEN + WRAPPER_LEN);
        }
        // a valid message in random chunks decodes to itself
        const auto m = sample_message(static_cast<std::uint32_t>(round), rnd() % 30);
        const B wire = concat(m.to_frames(Direction::In, 1 + rnd() % 200));
        StreamDecoder dec(Direction::In);
        std::vector<Message> got;
        std::size_t off = 0;
        while (off < wire.size()) {
            const std::size_t n = std::min<std::size_t>(wire.size() - off, 1 + rnd() % 64);
            for (auto& r : dec.push(std::span<const std::uint8_t>(wire).subspan(off, n))) {
                CHECK(r.has_value());
                got.push_back(*r);
            }
            off += n;
        }
        CHECK(got == std::vector<Message>{m});
    }
}

PIBRIDGE_TEST_MAIN()
