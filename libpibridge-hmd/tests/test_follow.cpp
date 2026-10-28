// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-tools/src/follow.rs tests (MIT OR Apache-2.0).
#include "common.h"
#include "pibridge_test.h"

#include "pibridge/hmd/follow.h"

using namespace pibridge::hmd;
using namespace std::chrono;
using C = Follow::Change;

TEST(wakes_at_once_and_sleeps_after_the_grace) {
    const auto t0 = Follow::Clock::now();
    Follow f(false);
    CHECK(f.step(t0, false) == std::nullopt);
    CHECK(f.step(t0, true) == C::Active);
    CHECK(f.step(t0 + seconds(1), true) == std::nullopt);
    // a short restart of the server keeps it awake
    CHECK(f.step(t0 + seconds(2), false) == std::nullopt);
    CHECK(f.step(t0 + seconds(4), true) == std::nullopt);
    CHECK(f.step(t0 + seconds(5), false) == std::nullopt);
    CHECK(f.step(t0 + seconds(9), false) == std::nullopt);
    CHECK(f.step(t0 + seconds(10), false) == C::Idle);
    CHECK(!f.active());
    CHECK(f.step(t0 + seconds(20), false) == std::nullopt);
}

TEST(finds_a_process_by_comm) {
    const auto root = testutil::tmpdir("follow");
    std::filesystem::create_directories(root / "1234");
    std::ofstream(root / "1234/comm") << "vrserver\n";
    std::filesystem::create_directories(root / "self");
    std::ofstream(root / "self/comm") << "vrserver\n"; // not a pid: ignored
    CHECK(process_running_in(root, "vrserver"));
    CHECK(!process_running_in(root, "vrcompositor"));
    std::filesystem::remove_all(root / "1234");
    CHECK(!process_running_in(root, "vrserver"));
    // names longer than comm's 15 characters match their first 15
    std::filesystem::create_directories(root / "77");
    std::ofstream(root / "77/comm") << "a-very-long-pro\n";
    CHECK(process_running_in(root, "a-very-long-process-name"));
    // a list: either runtime (SteamVR's vrserver or Monado's monado-service)
    std::filesystem::create_directories(root / "88");
    std::ofstream(root / "88/comm") << "monado-service\n";
    CHECK(process_running_in(root, "vrserver,monado-service"));
    CHECK(process_running_in(root, "vrserver, monado-service"));
    CHECK(!process_running_in(root, "vrserver,vrcompositor"));
    CHECK(!process_running_in(root, ","));
    std::filesystem::remove_all(root);
}

PIBRIDGE_TEST_MAIN()
