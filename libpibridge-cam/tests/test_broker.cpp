// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-cam/src/broker.rs (MIT OR Apache-2.0), plus the PXCB v1 wire layout byte by
// byte. Sockets live in $TMPDIR; no camera is involved (synthetic sources).
#include "common.h"
#include "pibridge/cam/broker.h"
#include "pibridge/cam/resilient.h"
#include "pibridge_test.h"

#include <atomic>
#include <cstring>
#include <future>
#include <set>
#include <thread>
#include <sys/mman.h>
#include <unistd.h>

using namespace pibridge::cam;
using namespace pibridge::cam::broker;
using namespace std::chrono_literals;
using pibridge::cam::testing::tmp;

namespace {

std::uint8_t pattern(std::uint64_t k, std::size_t i) {
    return static_cast<std::uint8_t>((k * 7 + i) % 251);
}

/// Synthetic camera: n frames of len bytes, byte i of frame k = (k * 7 + i) % 251.
class Fake : public Source {
public:
    Fake(std::uint64_t n_, std::size_t len_, int period_ms) : n(n_), len(len_), buf(len_, 0), period(period_ms) {}
    std::uint64_t k = 0;
    std::uint64_t n;
    std::size_t len;
    std::vector<std::uint8_t> buf;
    std::chrono::milliseconds period;
    bool replay = true;

    Result<bool> advance(int) override {
        if (k >= n) {
            return false;
        }
        std::this_thread::sleep_for(period);
        for (std::size_t i = 0; i < buf.size(); i++) {
            buf[i] = pattern(k, i);
        }
        k++;
        return true;
    }
    std::optional<RawFrame> current() const override {
        if (k == 0) {
            return std::nullopt;
        }
        return RawFrame{std::span(buf).first(len), 2560, 1000 + k, static_cast<std::uint32_t>(k - 1)};
    }
    std::string describe() const override { return "fake camera"; }
    bool finished() const override { return k >= n; }
    bool is_replay() const override { return replay; }
};

/// A live camera that errors (unplug) after fail_after frames.
class Flaky final : public Source {
public:
    Flaky(std::uint64_t n, std::uint64_t fail_after_) : inner(n, 1000, 4), fail_after(fail_after_) { inner.replay = false; }
    Fake inner;
    std::uint64_t fail_after;
    Result<bool> advance(int t) override {
        if (inner.k >= fail_after) {
            return fail(IoError::other("camera device error (unplugged?)"));
        }
        return inner.advance(t);
    }
    std::optional<RawFrame> current() const override { return inner.current(); }
    std::string describe() const override { return "flaky camera"; }
};

std::unique_ptr<Source> live(std::uint64_t n, std::uint64_t fail_after) {
    return std::make_unique<Flaky>(n, fail_after);
}

std::string sock_path(const std::string& name) {
    std::string p = tmp("pxcb-" + name + ".sock");
    if (p.size() >= 100) {
        p = "/tmp/pibridge-pxcb-" + std::to_string(::getpid()) + "-" + name + ".sock";
    }
    return p;
}

ServeOptions opts_for(const std::string& path, std::uint32_t slots) {
    ServeOptions o;
    o.socket = path;
    o.slots = slots;
    o.status_every = 60s;
    return o;
}

std::unique_ptr<BrokerSource> connect_wait(const std::string& path) {
    const auto t0 = std::chrono::steady_clock::now();
    for (;;) {
        auto c = BrokerSource::connect(path);
        if (c) {
            return std::move(*c);
        }
        if (std::chrono::steady_clock::now() - t0 > 5s) {
            std::fprintf(stderr, "%s\n", c.error().text().c_str());
            return nullptr;
        }
        std::this_thread::sleep_for(5ms);
    }
}

std::unique_ptr<Source> connect_retry(const std::string& path) {
    const auto t0 = std::chrono::steady_clock::now();
    for (;;) {
        auto c = open_source("broker:" + path, 0, ReplayOptions{});
        if (c) {
            return std::move(*c);
        }
        if (std::chrono::steady_clock::now() - t0 > 5s) {
            std::fprintf(stderr, "%s\n", c.error().text().c_str());
            return nullptr;
        }
        std::this_thread::sleep_for(5ms);
    }
}

} // namespace

TEST(hello_roundtrip_and_rejects) {
    Hello h;
    h.slots = 8;
    h.slot_bytes = 4096 * 521;
    h.data_bytes = static_cast<std::uint32_t>(FRAME_BYTES);
    h.flags = FLAG_REPLAY;
    h.desc = std::string(300, 'x');
    const auto d = *Hello::decode(h.encode());
    CHECK(d.desc.size() == MAX_DESC);
    CHECK(d.slots == 8 && d.flags == FLAG_REPLAY);
    auto b = h.encode();
    b[0] = 'X';
    CHECK(!Hello::decode(b));
    b = h.encode();
    b[4] = 9; // version
    const auto v = Hello::decode(b);
    CHECK(!v && v.error().text() == "camera broker hello: version 9 (this client speaks 1)");
    const auto enc = h.encode();
    CHECK(!Hello::decode(std::span(enc).first(10)));
    Hello bad = h;
    bad.slot_bytes = 100;
    CHECK(!Hello::decode(bad.encode()));
}

// PXCB v1 byte for byte: what a Rust client expects from a PiBridge broker and the other way round.
TEST(wire_layout_is_pxcb_v1) {
    Hello h;
    h.slots = 8;
    h.slot_bytes = 2134016; // (64 + 2129920) rounded up to pages
    h.data_bytes = 2129920;
    h.flags = 1;
    h.desc = "d\xc3\xa9";
    const std::vector<std::uint8_t> want = {'P', 'X', 'C', 'B', 1, 0, 0, 0, 8, 0, 0, 0, 0x00, 0x90, 0x20, 0x00,
                                            0x00, 0x80, 0x20, 0x00, 1, 0, 0, 0, 3, 0, 0, 0, 'd', 0xc3, 0xa9};
    CHECK(h.encode() == want);
    // a description cut at 200 bytes never splits a UTF-8 character
    h.desc = std::string(199, 'a') + "\xc3\xa9";
    CHECK(h.encode().size() == 28 + 199);
    // the ring the server creates: that slot size, magic + version in the header page
    auto ring = *Ring::create(8, FRAME_BYTES, 0, "t");
    CHECK(ring->layout().slot_bytes == 2134016);
    CHECK(ring->layout().ring_len() == 4096 + 8ull * 2134016);
    std::vector<std::uint8_t> d(100, 0);
    for (std::size_t i = 0; i < d.size(); i++) {
        d[i] = static_cast<std::uint8_t>(i);
    }
    CHECK(ring->publish(9, RawFrame{d, 2560, 0x1122334455667788ull, 0xaabbccdd}).has_value());
    const auto* p = static_cast<const std::uint8_t*>(::mmap(nullptr, ring->layout().ring_len(), PROT_READ, MAP_SHARED,
                                                            ring->fd(), 0));
    CHECK(std::memcmp(p, "PXCB\x01\x00\x00\x00", 8) == 0);
    const auto u64 = [&](std::size_t o) {
        std::uint64_t v = 0;
        std::memcpy(&v, p + o, 8);
        return v;
    };
    const auto u32 = [&](std::size_t o) {
        std::uint32_t v = 0;
        std::memcpy(&v, p + o, 4);
        return v;
    };
    CHECK(u64(64) == 10); // newest complete index + 1
    CHECK(u64(72) == 1);  // published
    const std::size_t slot = 4096 + (9 % 8) * 2134016;
    CHECK(u64(slot) == 2 * 9 + 2); // seqlock
    CHECK(u64(slot + 8) == 0x1122334455667788ull);
    CHECK(u32(slot + 16) == 0xaabbccdd);
    CHECK(u32(slot + 20) == 100);
    CHECK(u32(slot + 24) == 2560);
    CHECK(u64(slot + 32) == 9);
    CHECK(std::memcmp(p + slot + 64, d.data(), d.size()) == 0);
    ring->set_camera(false, 7, 123);
    CHECK(u64(80) == 7 && u64(88) == 2 && u64(96) == 123);
    ::munmap(const_cast<std::uint8_t*>(p), ring->layout().ring_len());
}

TEST(ring_publish_read_overwrite) {
    auto ring = *Ring::create(3, 1000, 0, "t");
    std::vector<std::uint8_t> out(1000, 0);
    CHECK(!ring->latest());
    CHECK(!ring->read(0, out));
    std::vector<std::uint8_t> data(1000);
    for (std::size_t i = 0; i < 1000; i++) {
        data[i] = pattern(5, i);
    }
    for (std::uint64_t k = 0; k < 5; k++) {
        CHECK(ring->publish(k, RawFrame{std::span(data).first(900), 2560, 42 + k, static_cast<std::uint32_t>(k)})
                  .has_value());
    }
    CHECK(ring->latest() == std::optional<std::uint64_t>(4) && ring->published() == 5);
    // 3 slots: frames 2, 3, 4 are there, 0 and 1 were overwritten
    CHECK(!ring->read(1, out));
    const auto i = *ring->read(4, out);
    CHECK(i.index == 4 && i.len == 900 && i.host_ts_ns == 46 && i.sequence == 4 && i.stride == 2560);
    CHECK(std::equal(out.begin(), out.begin() + 900, data.begin()));
    CHECK(!ring->read(5, out)); // not published yet
    std::vector<std::uint8_t> big(2000, 0);
    const auto e = ring->publish(5, RawFrame{big, 2560, 0, 0});
    CHECK(!e && e.error().text() == "frame of 2000 B does not fit a 1000 B slot");
}

TEST(client_maps_the_servers_ring_read_only) {
    auto ring = *Ring::create(2, 100, FLAG_REPLAY, "d");
    auto client = *Ring::open(::dup(ring->fd()), ring->layout());
    std::uint8_t d[50];
    std::memset(d, 9, sizeof d);
    CHECK(ring->publish(0, RawFrame{d, 10, 7, 1}).has_value());
    std::uint8_t out[100];
    const auto i = *client->read(0, out);
    CHECK(i.len == 50 && i.host_ts_ns == 7);
    CHECK(std::memcmp(out, d, 50) == 0);
}

TEST(end_to_end_two_clients_get_every_frame_and_the_end) {
    const std::string path = sock_path("e2e");
    std::atomic<bool> stop{false};
    const std::uint64_t n = 40;
    auto server = std::async(std::launch::async, [&] {
        return serve(std::make_unique<Fake>(n, 5000, 15), opts_for(path, 8), stop, [](const ServeStats&) {});
    });
    std::vector<std::unique_ptr<BrokerSource>> clients;
    for (int i = 0; i < 2; i++) {
        clients.push_back(connect_wait(path));
        CHECK(clients.back() != nullptr);
    }
    for (const auto& c : clients) {
        CHECK(c->is_replay() && c->describe().find("fake camera") != std::string::npos);
    }
    // both clients read at the same time, like passthrough and Monado would
    std::vector<std::future<std::vector<std::uint64_t>>> readers;
    std::atomic<int> bad{0};
    for (auto& c : clients) {
        readers.push_back(std::async(std::launch::async, [&bad, c = c.get()] {
            std::vector<std::uint64_t> seen;
            for (;;) {
                auto a = c->advance(2000);
                if (a && *a) {
                    const auto f = *c->current();
                    const std::uint64_t k = f.sequence;
                    bool ok = f.data.size() == 5000 && f.host_ts_ns == 1001 + k;
                    for (std::size_t i = 0; i < f.data.size() && ok; i++) {
                        ok = f.data[i] == pattern(k, i);
                    }
                    if (!ok) {
                        bad++;
                    }
                    seen.push_back(k);
                } else if (a) {
                    bad++; // timeout
                    return seen;
                } else {
                    if (a.error().kind != ErrorKind::ConnectionAborted || !c->finished()) {
                        bad++;
                    }
                    return seen;
                }
            }
        }));
    }
    std::vector<std::vector<std::uint64_t>> seen;
    for (auto& r : readers) {
        seen.push_back(r.get());
    }
    const auto st = server.get();
    CHECK(st.has_value() && st->frames == n);
    CHECK(bad.load() == 0);
    for (const auto& s : seen) {
        // in order, no duplicates; the first frames may predate the connection
        for (std::size_t i = 1; i < s.size(); i++) {
            CHECK(s[i] > s[i - 1]);
        }
        CHECK(!s.empty() && s.back() == n - 1);
        CHECK(s.size() + 10 >= n);
    }
    CHECK(!path_exists(path)); // socket removed at exit
}

TEST(a_slow_client_skips_but_never_blocks_the_server) {
    const std::string path = sock_path("slow");
    std::atomic<bool> stop{false};
    const std::uint64_t n = 60;
    auto server = std::async(std::launch::async, [&] {
        const auto t0 = std::chrono::steady_clock::now();
        auto r = serve(std::make_unique<Fake>(n, 1000, 5), opts_for(path, 4), stop, [](const ServeStats&) {});
        return std::pair{r, std::chrono::steady_clock::now() - t0};
    });
    auto c = connect_wait(path);
    CHECK(c != nullptr);
    std::vector<std::uint32_t> got;
    for (;;) {
        auto a = c->advance(2000);
        if (!a || !*a) {
            break;
        }
        got.push_back(c->current()->sequence);
        std::this_thread::sleep_for(40ms); // 8x slower than the camera
    }
    const auto [r, took] = server.get();
    CHECK(r.has_value() && r->frames == n);
    CHECK(took < std::chrono::milliseconds(n * 5 + 1500)); // the server was not slowed down
    for (std::size_t i = 1; i < got.size(); i++) {
        CHECK(got[i] > got[i - 1]);
    }
    CHECK(c->skipped() > 0 && got.size() < n / 2);
}

TEST(second_broker_on_the_same_socket_is_refused_and_stale_files_are_replaced) {
    const std::string path = sock_path("dup");
    {
        std::FILE* f = std::fopen(path.c_str(), "wb");
        std::fputs("stale", f);
        std::fclose(f);
    }
    const auto l = detail::listen(path); // stale file replaced
    CHECK(l.has_value());
    const auto e = detail::listen(path);
    CHECK(!e && e.error().kind == ErrorKind::AddrInUse);
    CHECK(e.error().text() == "a camera broker already serves " + path);
    ::close(*l);
    ::unlink(path.c_str());
    const auto c = BrokerSource::connect(path);
    CHECK(!c && c.error().text().find("pimax-cam broker") != std::string::npos);
}

TEST(broker_reopens_a_dropped_camera_and_clients_keep_reading) {
    const std::string path = sock_path("reopen");
    std::atomic<bool> stop{false};
    auto server = std::async(std::launch::async, [&] {
        // the camera drops after 10 frames, is absent for 10 tries (~200 ms), then streams again
        auto tries = std::make_shared<int>(0);
        Opener open = [tries]() -> Result<std::unique_ptr<Source>> {
            (*tries)++;
            if (*tries <= 10) {
                return fail(IoError::custom(ErrorKind::NotFound, "camera 34a4:0014 not found"));
            }
            return live(UINT64_MAX, UINT64_MAX);
        };
        Policy policy;
        policy.stall = 5s;
        policy.backoff_min = 10ms;
        policy.backoff_max = 20ms;
        policy.log_every = 30s;
        auto src = std::make_unique<Resilient>(live(UINT64_MAX, 10), open, policy, "camera");
        src->quiet();
        int downs = 0;
        auto st = serve(std::move(src), opts_for(path, 8), stop, [&](const ServeStats& s) { downs += !s.camera_up; });
        return std::pair{st, downs};
    });
    auto c = connect_retry(path);
    CHECK(c != nullptr);
    const auto g0 = c->generation();
    int seen = 0;
    std::set<std::uint64_t> gens;
    const auto t0 = std::chrono::steady_clock::now();
    bool err = false;
    while (seen < 40 && std::chrono::steady_clock::now() - t0 < 5s) {
        auto a = c->advance(100);
        if (!a) {
            err = true; // a live broker client never errors
            break;
        }
        if (*a) {
            seen++;
            gens.insert(c->generation());
        }
    }
    CHECK(!err);
    CHECK(seen >= 40); // frames kept coming after the drop
    CHECK(gens.size() >= 2 && c->generation() != g0);
    const auto p = *probe(path, 100ms);
    CHECK(p.camera_up == std::optional<bool>(true));
    CHECK(p.camera_reopens == 1);
    CHECK(p.fps > 10.0);
    CHECK(*p.heartbeat_age_s < 1.0);
    CHECK(p.to_json().find("\"camera_up\":true") != std::string::npos);
    stop.store(true);
    const auto [st, downs] = server.get();
    CHECK(st.has_value() && st->camera_reopens == 1 && st->camera_up && downs == 1);
}

TEST(client_reconnects_to_a_restarted_broker) {
    const std::string path = sock_path("restart");
    const auto run = [&](std::atomic<bool>& stop) {
        return std::async(std::launch::async, [&path, &stop] {
            return serve(live(UINT64_MAX, UINT64_MAX), opts_for(path, 8), stop, [](const ServeStats&) {});
        });
    };
    std::atomic<bool> stop1{false};
    auto b1 = run(stop1);
    auto c = connect_retry(path);
    CHECK(c != nullptr);
    int n = 0;
    while (n < 5) {
        n += *c->advance(1000) ? 1 : 0;
    }
    const auto g1 = c->generation();
    stop1.store(true);
    CHECK(b1.get().has_value());
    // broker gone: after the frames still queued, the client only sees timeouts, no error
    int quiet = 0;
    bool err = false;
    while (quiet < 5) {
        auto a = c->advance(20);
        if (!a) {
            err = true;
            break;
        }
        quiet = *a ? 0 : quiet + 1;
    }
    CHECK(!err);
    CHECK(!c->up());
    std::atomic<bool> stop2{false};
    auto b2 = run(stop2);
    const auto t0 = std::chrono::steady_clock::now();
    int got = 0;
    while (got < 5 && std::chrono::steady_clock::now() - t0 < 5s) {
        auto a = c->advance(100);
        got += a && *a ? 1 : 0;
    }
    CHECK(got == 5); // frames from the new broker
    CHECK(c->generation() != g1);
    CHECK(c->up());
    stop2.store(true);
    CHECK(b2.get().has_value());
}

TEST(replay_broker_end_is_still_an_end_for_wrapped_clients) {
    const std::string path = sock_path("replay-end");
    std::atomic<bool> stop{false};
    auto server = std::async(std::launch::async, [&] {
        return serve(std::make_unique<Fake>(30, 1000, 10), opts_for(path, 8), stop, [](const ServeStats&) {});
    });
    auto c = connect_retry(path);
    CHECK(c != nullptr && c->is_replay());
    const auto t0 = std::chrono::steady_clock::now();
    std::optional<IoError> end;
    while (std::chrono::steady_clock::now() - t0 < 5s) {
        auto a = c->advance(1000);
        if (!a) {
            end = a.error();
            break;
        }
    }
    CHECK(end && end->kind == ErrorKind::ConnectionAborted);
    CHECK(c->finished());
    CHECK(server.get().has_value());
}

PIBRIDGE_TEST_MAIN()
