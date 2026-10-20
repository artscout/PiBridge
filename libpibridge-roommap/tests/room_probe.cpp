// SPDX-License-Identifier: MIT
// Parity probe of the room half of libpibridge-roommap: the same cases as the scratch oracle program that linked
// omarchyvr-room at the pinned commit. `room CALIB`: the BRIEF pattern, the rig's virtual pinholes (LUT hash), the
// synthetic room's ray tables and renders, undistorted images, raw ORB on them, features (low-light stretch, mask,
// bearings) and stereo points for four poses; every float as its bits. The output must equal the oracle's
// (tests/golden/room_probe.sha256: its SHA-256 and size).
#include "pibridge/cam/calib.h"
#include "pibridge/cam/slam.h"
#include "pibridge/roommap/orb.h"
#include "pibridge/roommap/stereo.h"
#include "pibridge/roommap/synth.h"

#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace pibridge;
using namespace pibridge::roommap;
namespace g = cam::geom;
namespace sl = cam::slam;

namespace {

std::string hx(double x) {
    std::uint64_t b;
    std::memcpy(&b, &x, 8);
    char buf[24];
    std::snprintf(buf, sizeof buf, "%016" PRIx64, b);
    return buf;
}

std::string hf(float x) {
    std::uint32_t b;
    std::memcpy(&b, &x, 4);
    char buf[16];
    std::snprintf(buf, sizeof buf, "%08x", b);
    return buf;
}

std::string hp(const g::Pose& p) {
    std::string s;
    for (const auto& r : p.r) {
        for (double x : r) {
            s += hx(x) + " ";
        }
    }
    for (double x : p.t) {
        s += hx(x) + " ";
    }
    s.pop_back();
    return s;
}

std::string hd(const orb::Descriptor& d) {
    std::string s;
    char buf[4];
    for (auto b : d) {
        std::snprintf(buf, sizeof buf, "%02x", b);
        s += buf;
    }
    return s;
}

void fnv(std::uint64_t& h, const void* p, std::size_t n) {
    const auto* b = static_cast<const unsigned char*>(p);
    for (std::size_t i = 0; i < n; i++) {
        h = (h ^ b[i]) * 0x100000001B3ull;
    }
}

int room_cases(const char* calib) {
    auto cal = cam::DeviceCalib::load(calib);
    if (!cal) {
        std::fprintf(stderr, "%s\n", cal.error().text().c_str());
        return 1;
    }
    const std::size_t ids[2] = {0, 1};
    auto rig = stereo::Rig::from_calib(*cal, ids);
    if (!rig) {
        return 1;
    }
    std::uint64_t h = 0xCBF29CE484222325ull;
    for (const auto& p : orb::detail::pattern()) {
        fnv(h, p.data(), 4);
    }
    std::printf("pattern %016" PRIx64 "\n", h);
    std::vector<synth::RayTable> rays;
    for (std::size_t ci = 0; ci < rig->cams.size(); ci++) {
        const auto& c = rig->cams[ci];
        const auto& ph = c.pinhole;
        std::string cfv;
        for (const auto& r : ph.cam_from_virt) {
            for (double x : r) {
                cfv += hx(x) + " ";
            }
        }
        cfv.pop_back();
        std::printf("cam %zu id %zu w %zu h %zu f %s cx %s cy %s cfv %s\n", ci, c.id, ph.w, ph.h, hx(ph.f).c_str(),
                    hx(ph.cx).c_str(), hx(ph.cy).c_str(), cfv.c_str());
        std::printf("  bfc %s\n", hp(c.body_from_cam).c_str());
        h = 0xCBF29CE484222325ull;
        for (const auto& [u, v] : ph.lut) {
            fnv(h, &u, 4);
            fnv(h, &v, 4);
        }
        std::printf("  lut %016" PRIx64 "\n", h);
        rays.push_back(synth::RayTable::make(c.intr));
        h = 0xCBF29CE484222325ull;
        for (const auto& r4 : rays.back().rays) {
            for (const auto& r : r4) {
                fnv(h, r.data(), 12);
            }
        }
        std::printf("  rays %016" PRIx64 "\n", h);
    }
    const synth::Room room;
    for (int k = 0; k < 4; k++) {
        const double a = k * 1.3;
        const sl::Quat q = sl::qmul(sl::qyaw(a + 0.5), {std::sin(-0.1), 0.0, 0.0, std::cos(-0.1)});
        const g::Pose rb{sl::mat_of(q), {0.9 * std::sin(a), 1.55, 0.9 * std::cos(a)}};
        std::vector<stereo::Features> fs;
        for (std::size_t ci = 0; ci < rig->cams.size(); ci++) {
            const auto im = room.render_rays(rays[ci], rb.compose(rig->cams[ci].body_from_cam));
            h = 0xCBF29CE484222325ull;
            fnv(h, im.data(), im.size());
            std::printf("frame %d cam %zu img %016" PRIx64 "\n", k, ci, h);
            const orb::Image img{640, 480, 640, im};
            const auto und = rig->cams[ci].pinhole.undistort(img);
            h = 0xCBF29CE484222325ull;
            fnv(h, und.data(), und.size());
            std::printf("  und %016" PRIx64 "\n", h);
            const auto raw = orb::extract(orb::Image{640, 480, 640, und}, nullptr, orb::OrbParams{});
            std::printf("  orb-plain %zu\n", raw.size());
            for (const auto& [kp, d] : raw) {
                std::printf("    %s %s %u %s %s %s\n", hf(kp.x).c_str(), hf(kp.y).c_str(), kp.level, hf(kp.angle).c_str(),
                            hf(kp.response).c_str(), hd(d).c_str());
            }
            auto f = stereo::features(img, rig->cams[ci], orb::OrbParams{});
            std::printf("  feats %zu\n", f.kps.size());
            for (std::size_t i = 0; i < f.kps.size(); i++) {
                const auto& kp = f.kps[i];
                std::printf("    %s %s %u %s %s %s %s %s %s\n", hf(kp.x).c_str(), hf(kp.y).c_str(), kp.level,
                            hf(kp.angle).c_str(), hf(kp.response).c_str(), hd(f.desc[i]).c_str(),
                            hx(f.bearings[i][0]).c_str(), hx(f.bearings[i][1]).c_str(), hx(f.bearings[i][2]).c_str());
            }
            fs.push_back(std::move(f));
        }
        const auto pts = stereo::triangulate(fs[0], fs[1], rig->cams[0], rig->cams[1], stereo::StereoParams{});
        std::printf("  stereo %zu\n", pts.size());
        for (const auto& p : pts) {
            std::printf("    %zu %s %s %s %s %s\n", p.ia, hx(p.depth).c_str(), hx(p.p_body[0]).c_str(),
                        hx(p.p_body[1]).c_str(), hx(p.p_body[2]).c_str(), hd(p.desc).c_str());
        }
    }
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    if (argc >= 2 && std::string(argv[1]) == "trace") {
        const synth::Room room;
        std::uint64_t st = 5;
        auto next = [&]() {
            st += 0x9E3779B97F4A7C15ull;
            std::uint64_t z = st;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
            return z ^ (z >> 31);
        };
        auto sym = [&]() { return 2.0 * (static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0)) - 1.0; };
        for (int i = 0; i < 2000; i++) {
            const double o0 = sym() * 2.0;
            const double o1 = 1.0 + sym();
            const double o2 = sym() * 2.0;
            const double d0 = sym(), d1 = sym(), d2 = sym();
            const auto t = room.trace({o0, o1, o2}, g::normalize({d0, d1, d2}));
            if (t) {
                std::printf("%s %s\n", hx(t->first).c_str(), hx(t->second).c_str());
            } else {
                std::printf("none\n");
            }
        }
        return 0;
    }
    if (argc >= 3 && std::string(argv[1]) == "room") {
        return room_cases(argv[2]);
    }
    std::fputs("usage: room_probe room CALIB\n", stderr);
    return 2;
}
