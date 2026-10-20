// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax/src/haptics.rs tests (MIT OR Apache-2.0): the vibration limiter and the request file.
#include "pibridge_test.h"

#include "pibridge/hmd/haptics.h"

#include <fstream>

#include <unistd.h>

using namespace pibridge::hmd::haptics;
using namespace std::chrono;

static std::vector<Due> none() { return {}; }

TEST(first_pulse_goes_out_at_once) {
    Limiter l;
    const auto t = Clock::now();
    CHECK((l.offer(3, Pulse{25, 300})));
    CHECK((l.take_due(t) == std::vector<Due>{Due{3, 25, 300}}));
    CHECK(l.take_due(t) == none()); // sent once
    CHECK(!l.next_due());
}

TEST(at_most_one_per_8_ms_per_hand_newest_wins) {
    Limiter l;
    const auto t0 = Clock::now();
    l.offer(2, Pulse{99, 12});
    CHECK((l.take_due(t0) == std::vector<Due>{Due{2, 99, 12}}));
    // three more within 3 ms: only the newest survives, and only after the 8 ms gap
    l.offer(2, Pulse{97, 12});
    l.offer(2, Pulse{94, 12});
    l.offer(2, Pulse{92, 11});
    CHECK(l.replaced == 2);
    CHECK(l.take_due(t0 + milliseconds(3)) == none());
    CHECK(l.take_due(t0 + milliseconds(7)) == none());
    CHECK(l.next_due() == t0 + MIN_GAP);
    CHECK((l.take_due(t0 + MIN_GAP) == std::vector<Due>{Due{2, 92, 11}}));
    CHECK(l.sent == 2);
}

TEST(hands_are_independent) {
    Limiter l;
    const auto t0 = Clock::now();
    l.offer(2, Pulse{50, 12});
    CHECK((l.take_due(t0) == std::vector<Due>{Due{2, 50, 12}}));
    l.offer(3, Pulse{60, 12});
    CHECK((l.take_due(t0 + milliseconds(1)) == std::vector<Due>{Due{3, 60, 12}})); // right is not held by the left
    l.offer(2, Pulse{51, 12});
    l.offer(3, Pulse{61, 12});
    CHECK((l.take_due(t0 + milliseconds(8)) == std::vector<Due>{Due{2, 51, 12}}));
    CHECK((l.take_due(t0 + milliseconds(9)) == std::vector<Due>{Due{3, 61, 12}}));
}

TEST(stop_drops_a_waiting_pulse_and_unknown_slots_are_refused) {
    Limiter l;
    const auto t0 = Clock::now();
    l.offer(3, Pulse{40, 12});
    l.take_due(t0);
    l.offer(3, Pulse{41, 12});
    l.offer(3, std::nullopt);
    CHECK(l.take_due(t0 + milliseconds(20)) == none());
    CHECK((!l.offer(0, Pulse{1, 1}) && !l.offer(4, Pulse{1, 1})));
    l.offer(2, Pulse{10, 10});
    l.clear();
    CHECK(l.take_due(t0) == none());
}

TEST(game_stream_every_2_ms_comes_out_every_8_ms) {
    // a runtime that calls every 2 ms for 100 ms: 13 commands (t = 0, 8, ..., 96), each the newest value
    Limiter l;
    const auto t0 = Clock::now();
    std::vector<std::pair<long, std::uint8_t>> out;
    for (long k = 0; k < 50; k++) {
        const auto t = t0 + milliseconds(2 * k);
        l.offer(3, Pulse{static_cast<std::uint8_t>(100 - k), 12});
        for (const auto& d : l.take_due(t)) {
            out.emplace_back(2 * k, std::get<1>(d));
        }
    }
    CHECK(out.size() == 13);
    for (std::size_t i = 1; i < out.size(); i++) {
        CHECK(out[i].first - out[i - 1].first >= 8);
    }
    CHECK((out[1] == std::pair<long, std::uint8_t>{8, 96})); // the newest amplitude at the time it goes out
}

TEST(request_round_trip_and_take) {
    CHECK(format_request(3, 25, 300) == "3 25 300\n");
    CHECK((parse_request("3 25 300\n") == Due{3, 25, 300}));
    CHECK((parse_request("2 100 65535") == Due{2, 100, 65535}));
    for (const char* bad : {"", "3 25", "1 25 300", "3 0 300", "3 101 300", "3 25 0", "3 25 300 x", "a b c"}) {
        CHECK(!parse_request(bad));
    }
    // Rust str::parse: a leading '+' is accepted, '-' and overflow are not; split_whitespace knows Unicode spaces
    CHECK((parse_request("+3 +25 +300") == Due{3, 25, 300}));
    CHECK(!parse_request("3 -25 300") && !parse_request("3 25 65536") && !parse_request("3 256 300"));
    CHECK((parse_request("\xe3\x80\x80" "2\xc2\xa0" "40 200\xe2\x80\xa9") == Due{2, 40, 200}));
    const char* tmp = std::getenv("TMPDIR");
    const auto d = std::filesystem::path(tmp != nullptr ? tmp : "/tmp") /
                   ("pibridge-haptic-test-" + std::to_string(::getpid()));
    std::filesystem::remove(d);
    CHECK((take_request(d).value_or(Due{0, 0, 0}) == std::optional<Due>{}));
    std::ofstream(d) << format_request(2, 40, 200);
    CHECK((take_request(d).value_or(std::nullopt) == std::optional<Due>(Due{2, 40, 200})));
    CHECK(!std::filesystem::exists(d)); // consumed
    std::ofstream(d) << "junk";
    CHECK((!take_request(d).has_value()));
    CHECK(!std::filesystem::exists(d)); // a bad request is removed too
    // not UTF-8: read_to_string fails, the file stays (Rust behaviour)
    std::ofstream(d) << "3 25 \xff";
    CHECK((!take_request(d).has_value()));
    CHECK(std::filesystem::exists(d));
    std::filesystem::remove(d);
}

PIBRIDGE_TEST_MAIN()
