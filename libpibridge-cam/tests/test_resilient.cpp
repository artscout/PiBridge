// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-cam/src/resilient.rs (MIT OR Apache-2.0).
#include "pibridge/cam/resilient.h"
#include "pibridge_test.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

using namespace pibridge::cam;
using namespace std::chrono_literals;

namespace {

enum class Step { Frame, Timeout, Fail };

class FakeCam final : public Source {
public:
    FakeCam(std::vector<Step> s, std::uint32_t id_) : script(std::move(s)), id(id_), buf(4, 0) {}
    std::vector<Step> script;
    std::size_t pos = 0;
    std::uint32_t id;
    std::uint32_t k = 0;
    std::vector<std::uint8_t> buf;

    Result<bool> advance(int t) override {
        const Step s = pos < script.size() ? script[pos] : Step::Timeout;
        pos++;
        switch (s) {
        case Step::Frame: k++; return true;
        case Step::Timeout: std::this_thread::sleep_for(std::chrono::milliseconds(std::clamp(t, 0, 5))); return false;
        case Step::Fail: return fail(IoError::other("camera device error (unplugged?)"));
        }
        return false;
    }
    std::optional<RawFrame> current() const override { return RawFrame{buf, 4, k, id * 1000 + k}; }
    std::string describe() const override { return "fake cam #" + std::to_string(id); }
    std::uint64_t corrupt_frames() const override { return 1; }
};

/// An opener that fails `absent` times, then hands out cams with the given scripts.
std::pair<Opener, std::shared_ptr<std::atomic<std::uint32_t>>> opener(std::uint32_t absent,
                                                                       std::vector<std::vector<Step>> scripts) {
    auto calls = std::make_shared<std::atomic<std::uint32_t>>(0);
    auto st = std::make_shared<std::pair<std::vector<std::vector<Step>>, std::size_t>>(std::move(scripts), 0);
    auto id = std::make_shared<std::uint32_t>(1);
    Opener o = [calls, st, id, absent]() -> Result<std::unique_ptr<Source>> {
        const std::uint32_t n = calls->fetch_add(1);
        if (n < absent) {
            return fail(IoError::custom(ErrorKind::NotFound, "camera 34a4:0014 not found"));
        }
        (*id)++;
        std::vector<Step> script;
        if (st->second < st->first.size()) {
            script = st->first[st->second++];
        }
        return std::unique_ptr<Source>(new FakeCam(std::move(script), *id));
    };
    return {o, calls};
}

std::unique_ptr<Source> cam(std::vector<Step> script) {
    return std::make_unique<FakeCam>(std::move(script), 1);
}

Policy fast() {
    Policy p;
    p.stall = 60ms;
    p.backoff_min = 5ms;
    p.backoff_max = 20ms;
    p.log_every = 30s;
    return p;
}

/// Advance until a frame arrives or `limit` passes; returns the frame's sequence.
std::optional<std::uint32_t> next_frame(Resilient& r, Clock::duration limit) {
    const auto t0 = Clock::now();
    while (Clock::now() - t0 < limit) {
        auto a = r.advance(10);
        if (!a) {
            std::fprintf(stderr, "resilient source returned an error: %s\n", a.error().text().c_str());
            return std::nullopt;
        }
        if (*a) {
            return r.current()->sequence;
        }
    }
    return std::nullopt;
}

} // namespace

TEST(backoff_doubles_and_caps) {
    Policy p;
    p.backoff_min = 250ms;
    p.backoff_max = 2s;
    p.log_every = 30s;
    std::vector<long long> v;
    for (std::uint32_t n = 1; n <= 6; n++) {
        v.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(p.backoff(n)).count());
    }
    CHECK((v == std::vector<long long>{250, 500, 1000, 2000, 2000, 2000}));
    CHECK(p.backoff(0) == Clock::duration(250ms));
    CHECK(p.backoff(UINT32_MAX) == Clock::duration(2s));
}

TEST(error_then_absent_then_back_reopens_and_bumps_the_generation) {
    // frame, then an unplug error; the device is absent for 4 tries, then returns
    auto [o, calls] = opener(4, {{Step::Frame, Step::Frame}});
    Resilient r(cam({Step::Frame, Step::Fail}), o, fast(), "camera");
    r.quiet();
    const auto g0 = r.generation();
    CHECK(next_frame(r, 1s) == std::optional<std::uint32_t>(1001));
    // the error is hidden: only timeouts until the camera is back
    const auto back = next_frame(r, 2s);
    CHECK(back == std::optional<std::uint32_t>(2001)); // first frame of the reopened camera
    CHECK(calls->load() == 5);                          // 4 failed opens + 1 good
    const auto h = r.health();
    CHECK(h.up && h.opens == 2 && h.losses == 1 && h.stalls == 0 && h.failures == 0);
    CHECK(r.generation() != g0);
    CHECK(r.up());
    CHECK(r.corrupt_frames() == 2); // counts of the dropped source are kept
}

TEST(stall_reopens_a_camera_that_stops_sending) {
    auto [o, calls] = opener(0, {{Step::Frame}});
    Resilient r(cam({Step::Frame}), o, fast(), "camera");
    r.quiet();
    CHECK(next_frame(r, 1s) == std::optional<std::uint32_t>(1001));
    const auto t0 = Clock::now();
    CHECK(next_frame(r, 2s) == std::optional<std::uint32_t>(2001));
    CHECK(Clock::now() - t0 >= 60ms); // waited for the stall time
    CHECK(calls->load() == 1);
    CHECK(r.health().stalls == 1);
}

TEST(a_dark_camera_backs_off_instead_of_spinning) {
    // every reopen succeeds but never delivers a frame (headset asleep): failures grow, so the retry slows down
    auto [o, calls] = opener(0, std::vector<std::vector<Step>>(100));
    Resilient r(cam({}), o, fast(), "camera");
    r.quiet();
    const auto t0 = Clock::now();
    while (Clock::now() - t0 < 500ms) {
        auto a = r.advance(10);
        CHECK(a.has_value() && !*a);
    }
    const auto n = calls->load();
    // stall 60 ms + backoff up to 20 ms: at most ~8 reopens in 500 ms, at least 3
    CHECK(n >= 3 && n <= 9);
    CHECK(r.health().failures >= 3);
}

TEST(respects_the_callers_timeout_while_down) {
    auto [o, calls] = opener(UINT32_MAX, {});
    (void)calls;
    Policy p = fast();
    p.backoff_min = 5s;
    p.backoff_max = 5s;
    Resilient r(cam({Step::Fail}), o, p, "camera");
    r.quiet();
    const auto t0 = Clock::now();
    CHECK(!*r.advance(50));
    CHECK(!*r.advance(50));
    const auto took = Clock::now() - t0;
    CHECK(took >= 90ms && took < 400ms);
    CHECK(!r.up() && !r.current());
    CHECK(r.describe().find("reopening") != std::string::npos);
    CHECK(!r.finished());
}

TEST(broker_policy_does_not_stall_out) {
    auto [o, calls] = opener(0, {});
    Resilient r(cam({Step::Frame}), o, Policy::broker(), "camera broker");
    r.quiet();
    CHECK(next_frame(r, 1s).has_value());
    const auto t0 = Clock::now();
    while (Clock::now() - t0 < 150ms) {
        CHECK(!*r.advance(10));
    }
    CHECK(calls->load() == 0); // a quiet broker is not reconnected
}

PIBRIDGE_TEST_MAIN()
