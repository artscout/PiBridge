// SPDX-License-Identifier: MIT
// The other side of the files: omarchyvr's Monado hook pimax_anchor.h itself (tests/monado_side.c, built from the
// checkout) writes the head ring, which this library reads, and reads the offset file this library writes.
// Skipped (77) without the checkout.
#include "pibridge/anchor/rustx.h"
#include "pibridge/anchor/world.h"
#include "pibridge_test.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <filesystem>
#include <thread>

using namespace pibridge;
namespace g = cam::geom;

#ifdef PIBRIDGE_HAVE_MONADO_ANCHOR
extern "C" {
void pibridge_monado_anchor_query(std::int64_t at_ns, const float p[7], std::uint32_t flags, float out[7]);
int pibridge_monado_anchor_state(float offset[7], float applied[7]);
void pibridge_monado_clock_shift(std::int64_t ns);
}

TEST(monado_ring_and_our_offset) {
    const char* t = std::getenv("TMPDIR");
    const std::string d = std::string(t != nullptr ? t : "/tmp") + "/anchor-monado-" + std::to_string(anchor::rs::process_id());
    std::filesystem::remove_all(d);
    std::filesystem::create_directories(d);
    const std::string off = d + "/pimax-anchor.offset";
    const std::string head = d + "/pimax-anchor.head";
    ::setenv("PIMAX_ANCHOR", "1", 1);
    ::setenv("PIMAX_ANCHOR_FILE", off.c_str(), 1);
    ::setenv("PIMAX_ANCHOR_HEAD_FILE", head.c_str(), 1);
    CHECK(anchor::offset_path() == off && anchor::ring_path() == head);
    // a stale offset of an earlier session: Monado deletes it at its start
    CHECK(cam::write_file(off, "pimax-anchor-offset 1\npid 1\npose 0 0 0 0 0 0 1\n").has_value());

    // Monado records 140 head queries 4 ms apart (valid orientation; every third one without a position)
    const std::uint32_t ori = 1u << 0, pos = 1u << 1;
    const std::uint64_t t0 = anchor::rs::monotonic_ns();
    float out[7];
    for (int i = 0; i < 140; i++) {
        const double a = 0.01 * i;
        const float p[7] = {static_cast<float>(0.002 * i), 1.6f, static_cast<float>(-0.001 * i), 0.0f,
                            static_cast<float>(std::sin(a / 2)), 0.0f, static_cast<float>(std::cos(a / 2))};
        pibridge_monado_anchor_query(static_cast<std::int64_t>(t0 + 4000000ull * i), p, i % 3 == 2 ? ori : ori | pos, out);
        // no offset yet: the pose passes unchanged
        CHECK(out[0] == p[0] && out[1] == p[1] && out[6] == p[6]);
    }
    CHECK(!cam::path_exists(off));
    const auto r = anchor::HeadRing::read(head);
    CHECK(r.has_value());
    if (!r) {
        return;
    }
    CHECK(r->pid == anchor::rs::process_id());
    CHECK(r->samples.size() == 128);
    // the newest 128 samples, in time order, the floats exactly as Monado stored them
    const auto& last = r->samples.back();
    CHECK(last.ts_ns == t0 + 4000000ull * 139);
    CHECK(last.pose.t[0] == static_cast<double>(static_cast<float>(0.002 * 139)));
    CHECK(last.valid == (139 % 3 != 2));
    CHECK(r->samples.front().ts_ns == t0 + 4000000ull * 12);
    // the feed: this process is not a monado-service, so it is named as another program's
    const auto f = anchor::check_feed(r, {anchor::rs::process_id()}, t0 + 4000000ull * 140, anchor::pid_alive);
    CHECK(f.kind == anchor::Feed::Kind::Live);

    // our offset for this "Monado" (pid = this process), with the lease
    CHECK(anchor::take_lease(off, anchor::rs::process_id(), anchor::Role::Apply).has_value());
    const g::Pose o{g::rodrigues({0.0, 0.7, 0.0}), {0.25, -1.4, 0.6}};
    CHECK(anchor::write_offset(off, anchor::rs::process_id(), o).has_value());
    pibridge_monado_clock_shift(300000000); // past the hook's 250 ms re-read interval
    const float p[7] = {0.1f, 1.6f, -0.2f, 0.0f, 0.0f, 0.0f, 1.0f};
    pibridge_monado_anchor_query(static_cast<std::int64_t>(t0 + 4000000ull * 141), p, ori | pos, out);
    float so[7], sa[7];
    const int st = pibridge_monado_anchor_state(so, sa);
    CHECK(st == 7); // enabled, offset parsed, applied
    const auto [tt, q] = anchor::pq(o);
    // Monado parses the text with sscanf("%f"): the 6/7-decimal text, as floats
    for (int i = 0; i < 3; i++) {
        CHECK(std::abs(so[i] - tt[i]) < 1e-6);
    }
    for (int i = 0; i < 4; i++) {
        CHECK(std::abs(so[3 + i] - q[i]) < 1e-6);
        CHECK(sa[3 + i] == so[3 + i]); // the first offset of a session is placed at once
    }
    // app_from_slam = offset * slam
    const g::Vec3 want = o.apply({0.1, 1.6, -0.2});
    for (int i = 0; i < 3; i++) {
        CHECK(std::abs(out[i] - want[i]) < 1e-5);
    }
    // an offset for another pid is ignored (the hook drops it). The hook sees a change by the file's mtime, which
    // moves in kernel clock ticks: wait one.
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    CHECK(anchor::take_lease_as(off, 12345, 99, anchor::Role::Apply, [](std::uint32_t) { return false; },
                                [](std::uint32_t) -> std::expected<void, std::string> { return {}; })
              .has_value());
    CHECK(anchor::write_offset_as(off, 12345, 99, o).has_value());
    pibridge_monado_clock_shift(300000000);
    pibridge_monado_anchor_query(static_cast<std::int64_t>(t0 + 4000000ull * 142), p, ori | pos, out);
    CHECK((pibridge_monado_anchor_state(so, sa) & 2) == 0);
    std::filesystem::remove_all(d);
}
#else
int main() {
    std::puts("skipped: no omarchyvr checkout (forks/monado pimax_anchor.h)");
    return 77;
}
#define PIBRIDGE_NO_MAIN
#endif

#ifndef PIBRIDGE_NO_MAIN
PIBRIDGE_TEST_MAIN()
#endif
