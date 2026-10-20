// SPDX-License-Identifier: MIT
// The fuser -> tracker hint file (pibridge/ctrl/hint.h): layout, ring, interpolation, and the fuser writing it.
#include "pibridge/ctrl/fuse.h"
#include "pibridge/ctrl/hint.h"
#include "pibridge_test.h"

#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>

using namespace pibridge::ctrl;

namespace {
std::string tmp(const char* name) {
    const char* d = std::getenv("TMPDIR");
    return std::string(d != nullptr ? d : "/tmp") + "/" + name;
}
} // namespace

TEST(layout_and_ring) {
    CHECK(hint::SIZE == 5168);
    const std::string p = tmp("pibridge-ctrl-hint-test");
    std::unique_ptr<hint::Writer> w(hint::Writer::open(p));
    CHECK(w != nullptr);
    std::unique_ptr<hint::Reader> r(hint::Reader::open(p));
    CHECK(r != nullptr && r->live());
    CHECK(r->read(0).empty());
    for (std::uint64_t k = 0; k < 20; k++) {
        hint::Entry e;
        e.t_ns = 1000 + k * 10;
        e.state = 2;
        e.flags = 1;
        e.pos = {static_cast<float>(k), 0.0f, 0.0f};
        w->write(1, e);
    }
    const auto ring = r->read(1);
    CHECK(ring.size() == hint::RING);
    CHECK(ring.front().t_ns == 1040 && ring.back().t_ns == 1190);
    CHECK(r->read(0).empty());
    // between two entries: linear
    const auto m = hint::at(ring, 1105);
    CHECK(m && std::fabs(m->pos[0] - 10.5f) < 1e-4f);
    // before the ring: none; a little after: the newest
    CHECK(!hint::at(ring, 1000));
    const auto a = hint::at(ring, 1190 + 1000);
    CHECK(a && a->pos[0] == 19.0f);
    CHECK(!hint::at(ring, 1190 + 100'000'000));
}

TEST(carried_on_by_the_rate) {
    std::vector<hint::Entry> ring(1);
    ring[0].t_ns = 1'000'000'000;
    ring[0].state = 1;
    ring[0].w = {0.0f, 0.0f, 1.0f}; // 1 rad/s about z
    const auto e = hint::at(ring, 1'050'000'000);
    CHECK(e.has_value());
    const double ang = 2.0 * std::atan2(e->quat[2], e->quat[3]);
    CHECK(std::fabs(ang - 0.05) < 1e-4);
}

TEST(fuser_writes_it) {
    const std::string p = tmp("pibridge-ctrl-hint-fuser");
    fuse::Config c = fuse::Config::pibridge();
    c.hint_path = p;
    c.online_fit = false;
    c.use_imu = true;
    fuse::Calib cal;
    const double lsb = 0.1207 * M_PI / 180.0;
    cal.c = {{{lsb, 0, 0}, {0, lsb, 0}, {0, 0, lsb}}};
    fuse::Fuser f(0, c, cal);
    const fuse::P head{{0.0, 0.0, 0.0, 1.0}, {0.0, 1.6, 0.0}};
    std::uint32_t dev = 0;
    for (std::uint64_t t = 100'000'000; t < 400'000'000; t += 2'000'000) {
        dev += 2000;
        f.sample_acc(t, dev, {0, 0, 0}, std::array<std::int16_t, 3>{0, 1024, 0});
        if (t % 34'000'000 == 0) {
            f.optical(t - 20'000'000, fuse::P{{0, 0, 0, 1}, {0.1, 1.3, -0.3}}, head, 0.8f, 12);
        }
        f.get(t, head);
    }
    std::unique_ptr<hint::Reader> r(hint::Reader::open(p));
    CHECK(r != nullptr && r->live());
    const auto ring = r->read(0);
    CHECK(!ring.empty());
    // head-relative: the world fix (0.1, 1.3, -0.3) with the head at (0, 1.6, 0)
    CHECK(ring.back().state != 0 && (ring.back().flags & 1u) != 0);
    CHECK(std::fabs(ring.back().pos[0] - 0.1f) < 0.01f && std::fabs(ring.back().pos[1] + 0.3f) < 0.01f &&
          std::fabs(ring.back().pos[2] + 0.3f) < 0.01f);
    CHECK(r->read(1).empty());
}

PIBRIDGE_TEST_MAIN()
