// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax/src/fake.rs (MIT OR Apache-2.0).
#include "pibridge/hmd/fake.h"

#include <mutex>
#include <thread>

#include <unistd.h>

namespace pibridge::hmd::fake {

using Clock = std::chrono::steady_clock;

struct Ends {
    std::array<Fd, NUM_IF> if_dev;
    Fd ctl_dev;
};

struct FakeState {
    std::mutex mu;
    bool plugged = true;
    std::optional<Ends> ends;
    std::uint64_t connects = 0;
    const Model* model = &DREAM_AIR;
};

std::pair<std::unique_ptr<FakeBackend>, FakeDevice> pair() {
    auto st = std::make_shared<FakeState>();
    return {std::make_unique<FakeBackend>(st), FakeDevice(st)};
}

Result<std::optional<Link>> FakeBackend::connect() {
    std::lock_guard g(st_->mu);
    if (!st_->plugged) {
        return std::optional<Link>{};
    }
    Link link;
    Ends ends;
    for (std::size_t i = 0; i < NUM_IF; i++) {
        auto p = seqpacket_pair();
        if (!p) {
            return std::unexpected(p.error());
        }
        link.reads[i] = std::move(p->first);
        ends.if_dev[i] = std::move(p->second);
    }
    auto c = seqpacket_pair();
    if (!c) {
        return std::unexpected(c.error());
    }
    link.ctl = std::move(c->first);
    ends.ctl_dev = std::move(c->second);
    link.model = st_->model;
    st_->ends = std::move(ends);
    st_->connects++;
    return std::optional<Link>(std::move(link));
}

std::uint64_t FakeDevice::connects() const {
    std::lock_guard g(st_->mu);
    return st_->connects;
}

void FakeDevice::unplug() const {
    std::lock_guard g(st_->mu);
    st_->plugged = false;
    st_->ends.reset();
}

void FakeDevice::plug() const {
    std::lock_guard g(st_->mu);
    st_->plugged = true;
}

void FakeDevice::set_model(const Model& m) const {
    std::lock_guard g(st_->mu);
    st_->model = &m;
}

bool FakeDevice::wait_host(std::chrono::milliseconds timeout) const {
    const auto end = Clock::now() + timeout;
    while (Clock::now() < end) {
        {
            std::lock_guard g(st_->mu);
            if (st_->ends) {
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
}

bool FakeDevice::send_report(std::size_t iface, const Report& r) const {
    std::lock_guard g(st_->mu);
    if (!st_->ends || iface >= NUM_IF) {
        return false;
    }
    return ::write(st_->ends->if_dev[iface].get(), r.data(), r.size()) == static_cast<ssize_t>(r.size());
}

std::optional<Report> FakeDevice::recv_command(std::chrono::milliseconds timeout) const {
    const auto end = Clock::now() + timeout;
    while (true) {
        {
            std::lock_guard g(st_->mu);
            if (st_->ends) {
                std::array<std::uint8_t, REPORT_SIZE + 1> buf{};
                const auto n = ::read(st_->ends->ctl_dev.get(), buf.data(), buf.size());
                if (n == static_cast<ssize_t>(REPORT_SIZE + 1)) {
                    Report r{};
                    std::copy(buf.begin() + 1, buf.end(), r.begin());
                    return r;
                }
                if (n > 0) {
                    continue; // a short message: skip it (0 = the host closed its end: wait like for nothing)
                }
            }
        }
        const auto now = Clock::now();
        if (now >= end) {
            return std::nullopt;
        }
        std::this_thread::sleep_for(std::min<Clock::duration>(std::chrono::milliseconds(1), end - now));
    }
}

std::vector<Report> FakeDevice::drain_commands(std::chrono::milliseconds timeout) const {
    const auto end = Clock::now() + timeout;
    std::vector<Report> v;
    while (true) {
        const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(end - Clock::now());
        auto r = recv_command(std::max(left, std::chrono::milliseconds(0)));
        if (!r) {
            break;
        }
        v.push_back(*r);
    }
    return v;
}

} // namespace pibridge::hmd::fake
