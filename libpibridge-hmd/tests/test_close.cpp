// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax/tests/close.rs (MIT OR Apache-2.0): closing a session returns promptly in every state.
#include "common.h"
#include "pibridge_test.h"

#include "pibridge/hmd/descriptor.h"
#include "pibridge/hmd/fake.h"
#include "pibridge/hmd/session.h"

#include <future>
#include <mutex>
#include <thread>

using namespace pibridge::hmd;
using namespace std::chrono;
using Clock = steady_clock;

static Config cfg() {
    Config c;
    c.devices_dirs = {PIBRIDGE_SOURCE_DEVICES_DIR};
    return c;
}

class Locker : public Handler {
public:
    explicit Locker(std::shared_ptr<std::mutex> m) : m_(std::move(m)) {}
    void on_imu(const ImuSample&) override {
        std::lock_guard g(*m_);
        n_++;
    }

private:
    std::shared_ptr<std::mutex> m_;
    std::uint64_t n_ = 0;
};

static Report imu_report() {
    Report r{};
    r[0] = 0x11;
    return r;
}

TEST(close_while_connected_and_flooded) {
    auto [be, dev] = fake::pair();
    auto m = std::make_shared<std::mutex>();
    auto s = Session::open_with_backend(cfg(), std::move(be), std::make_unique<Locker>(m));
    CHECK((*s)->wait_connected(seconds(2)).has_value());
    const auto t = Clock::now();
    for (int i = 0; i < 200; i++) {
        (void)dev.send_report(1, imu_report());
    }
    (*s)->close();
    CHECK(Clock::now() - t < milliseconds(500));
}

TEST(close_while_not_connected) {
    auto [be, dev] = fake::pair();
    dev.unplug();
    auto s = Session::open_with_backend(cfg(), std::move(be), std::make_unique<Locker>(std::make_shared<std::mutex>()));
    std::this_thread::sleep_for(milliseconds(100));
    const auto t = Clock::now();
    s->reset();
    CHECK(Clock::now() - t < milliseconds(500));
}

/// The closer does not hold the callback's mutex (Monado's destroy order): a callback blocked briefly must not stall
/// close.
TEST(close_with_callback_contending) {
    auto [be, dev] = fake::pair();
    auto m = std::make_shared<std::mutex>();
    auto s = Session::open_with_backend(cfg(), std::move(be), std::make_unique<Locker>(m));
    CHECK((*s)->wait_connected(seconds(2)).has_value());
    std::promise<void> held;
    std::thread h([m, &held] {
        std::lock_guard g(*m);
        held.set_value();
        std::this_thread::sleep_for(milliseconds(200));
    });
    held.get_future().wait();
    (void)dev.send_report(1, imu_report());
    std::this_thread::sleep_for(milliseconds(50));
    const auto t = Clock::now();
    (*s)->close();
    h.join();
    CHECK(Clock::now() - t < milliseconds(600));
}

PIBRIDGE_TEST_MAIN()
