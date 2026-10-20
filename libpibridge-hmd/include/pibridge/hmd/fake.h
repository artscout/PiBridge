// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax/src/fake.rs (MIT OR Apache-2.0).
//
// In-process fake headset: seqpacket socketpairs behave like hidraw (message boundaries, poll, hang-up on unplug).
// Used by the tests; never touches real hardware.
#pragma once

#include "pibridge/hmd/session.h"

#include <chrono>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace pibridge::hmd::fake {

struct FakeState;

/// The "device side" handle of a fake headset (copyable; all copies are the same device).
class FakeDevice {
public:
    explicit FakeDevice(std::shared_ptr<FakeState> st) : st_(std::move(st)) {}
    /// Number of links the host has opened.
    std::uint64_t connects() const;
    /// Simulate unplugging: the host sees hang-up on every interface and the backend finds nothing.
    void unplug() const;
    /// Plug it back in; the next connect attempt succeeds.
    void plug() const;
    /// The model the next link reports (default: the Dream Air).
    void set_model(const Model& m) const;
    /// Wait until the host has connected (device ends exist).
    bool wait_host(std::chrono::milliseconds timeout) const;
    /// Deliver a report on interface `iface` (0..3) as if the device had sent it. False when not connected.
    bool send_report(std::size_t iface, const Report& r) const;
    /// Next command the host wrote to IF0 (report id stripped), waiting up to `timeout`.
    std::optional<Report> recv_command(std::chrono::milliseconds timeout) const;
    /// Every command written until `timeout` expires.
    std::vector<Report> drain_commands(std::chrono::milliseconds timeout) const;

private:
    std::shared_ptr<FakeState> st_;
};

/// The Backend half: give it to Session::open_with_backend.
class FakeBackend : public Backend {
public:
    explicit FakeBackend(std::shared_ptr<FakeState> st) : st_(std::move(st)) {}
    Result<std::optional<Link>> connect() override;

private:
    std::shared_ptr<FakeState> st_;
};

/// A plugged-in fake headset and its backend.
std::pair<std::unique_ptr<FakeBackend>, FakeDevice> pair();

} // namespace pibridge::hmd::fake
