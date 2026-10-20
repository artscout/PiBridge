// SPDX-License-Identifier: MIT
// The parity probe of libpibridge-anchor: the same deterministic cases as the scratch oracle program that linked
// omarchyvr-anchor at the pinned commit (offset text and parser, lease parser and text, ring parser and `at`, the ring
// writer, compute_offset, interp, pose_error, robust_mean, decide_lease, check_feed, anchor.toml), every number as the
// bits of its f64. Its output must equal tests/golden/anchor_probe.txt.gz (written by the oracle) byte for byte.
#include "pibridge/anchor/config.h"
#include "pibridge/anchor/world.h"
#include "pibridge/cam/rustfmt.h"

#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace pibridge;
using anchor::Pose;
namespace g = cam::geom;
namespace rf = cam::rfmt;

namespace {

struct Sm {
    std::uint64_t s;
    std::uint64_t next() {
        s += 0x9E3779B97F4A7C15ull;
        std::uint64_t z = s;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    double unit() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); }
    double sym() { return 2.0 * unit() - 1.0; }
    Pose pose(double rot, double tr) {
        g::Vec3 w{};
        for (auto& x : w) {
            x = rot * sym();
        }
        Pose p;
        p.r = g::rodrigues(w);
        for (auto& x : p.t) {
            x = tr * sym();
        }
        return p;
    }
};

std::string hx(double x) {
    std::uint64_t b;
    std::memcpy(&b, &x, 8);
    char buf[24];
    std::snprintf(buf, sizeof buf, "%016" PRIx64, b);
    return buf;
}

std::string hp(const Pose& p) {
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

std::string feed_debug(const anchor::Feed& f) {
    using K = anchor::Feed::Kind;
    switch (f.kind) {
    case K::Missing: return "Missing";
    case K::NoMonado: return "NoMonado { pid: " + std::to_string(f.pid) + " }";
    case K::Foreign:
        return "Foreign { pid: " + std::to_string(f.pid) + ", monado: " + std::to_string(f.monado) +
               ", alive: " + (f.alive ? "true" : "false") + " }";
    case K::Stale:
        return "Stale { pid: " + std::to_string(f.pid) + ", age_s: " +
               (f.age_s ? "Some(" + rf::debug_f64(*f.age_s) + ")" : std::string("None")) + " }";
    case K::Live: return "Live { pid: " + std::to_string(f.pid) + " }";
    }
    return "";
}

void put(std::string& b, std::size_t o, const void* v, std::size_t n) {
    std::memcpy(b.data() + o, v, n);
}

} // namespace

int main() {
    Sm rng{1};
    for (int i = 0; i < 300; i++) {
        const double rot = i % 3 == 0 ? 0.001 : 3.0;
        const Pose p = rng.pose(rot, i % 5 == 0 ? 1e-7 : 4.0);
        const auto pid = static_cast<std::uint32_t>(rng.next() % 5000000);
        const std::string t = anchor::offset_text(pid, p);
        std::printf("offset %d %s", i, t.c_str());
        if (auto r = anchor::parse_offset(t)) {
            std::printf("  parsed %u %s\n", r->first, hp(r->second).c_str());
        } else {
            std::printf("  parsed none\n");
        }
        const auto [tt, qq] = anchor::pq(p);
        std::printf("  pq %s %s %s %s %s %s %s\n", hx(tt[0]).c_str(), hx(tt[1]).c_str(), hx(tt[2]).c_str(),
                    hx(qq[0]).c_str(), hx(qq[1]).c_str(), hx(qq[2]).c_str(), hx(qq[3]).c_str());
    }
    const char* texts[] = {
        "pimax-anchor-offset 1\npid 42\npose 1 2 3 0 0 0 1\n",
        "pimax-anchor-offset 1\r\npid 42\r\npose 1 2 3 0 0 0 1\r\n",
        "  pimax-anchor-offset 1  \npid  +42 \npose 1e0 -2.5 3. 0.1 0.2 0.3 0.9",
        "pimax-anchor-offset 1\npid 42\npose 1 2 3 0 0 0\n",
        "pimax-anchor-offset 1\npid 42\npose 1 2 3 0 0 0 1 5\n",
        "pimax-anchor-offset 2\npid 42\npose 1 2 3 0 0 0 1\n",
        "pimax-anchor-offset 1\npid -4\npose 1 2 3 0 0 0 1\n",
        "pimax-anchor-offset 1\npid 4294967296\npose 1 2 3 0 0 0 1\n",
        "pimax-anchor-offset 1\n pid 42\npose 1 2 3 0 0 0 1\n",
        "pimax-anchor-offset 1\npid 42\npose  1 2 3 0 0 0 nan\n",
        "pimax-anchor-offset 1\npid 42\npose inf 2 3 0 0 0 1\n",
        "pimax-anchor-offset 1\npid 42\npose 1 2 3 0 0 0 0\n",
        "pimax-anchor-offset 1\npid 42\npose\t1\t2\t3\t0\t0\t0\t1\n",
        "pimax-anchor-offset 1\npid 42\n",
        "",
        "pimax-anchor-offset 1",
        "pimax-anchor-offset 1\npid 42\npose 1 2 3 0 0 0 1\ntrailing junk\n",
        "pimax-anchor-offset 1\rpid 42\npose 1 2 3 0 0 0 1\n",
        "pimax-anchor-offset 1\npid 42\npose 1,5 2 3 0 0 0 1\n",
        "pimax-anchor-offset 1\npid 42\npose +1 .5 -.5 0 0 0 1\n",
    };
    for (std::size_t i = 0; i < std::size(texts); i++) {
        if (auto r = anchor::parse_offset(texts[i])) {
            std::printf("parse %zu %u %s\n", i, r->first, hp(r->second).c_str());
        } else {
            std::printf("parse %zu none\n", i);
        }
    }
    const char* ls[] = {
        "roommap 1092498\nlease 2\nwriter 1093007\nrole setup\n",
        "anchor 42\n",
        "anchor 42",
        "roommap 7\nlease 3\nwriter 9\nrole auto\n",
        "roommap 7\nlease x\n",
        "roommap 7\nwriter -1\n",
        "roommap 7\nrole nonsense\nwriter 5\nlease 2\n",
        "roommap\n",
        "",
        "  roommap   7   extra\n  lease   2  \n writer 3\nrole  tag \n",
        "roommap 7\nlease 1\nwriter 3\nrole follow\nlease 2\n",
        "foo 99\r\nlease 2\r\nwriter 4\r\nrole reloc\r\n",
        "roommap 7\nleasex 2\nwriter3\n",
        "roommap 4294967295\nlease 2\nwriter 4294967295\nrole apply\n",
    };
    for (std::size_t i = 0; i < std::size(ls); i++) {
        if (auto l = anchor::Lease::parse(ls[i])) {
            const std::string role = l->role ? "Some(" + rf::debug_str(anchor::role_name(*l->role)) + ")" : "None";
            std::printf("lease %zu %s %u %u %s %u | %s\n", i, l->kind.c_str(), l->session, l->writer, role.c_str(),
                        l->schema, rf::debug_str(l->text()).c_str());
        } else {
            std::printf("lease %zu none\n", i);
        }
    }
    for (std::size_t i = 0; i < 40; i++) {
        std::string b(anchor::RING_SIZE + (i % 3) * 7, '\0');
        const std::uint32_t magic = i == 3 ? 0x1234 : anchor::RING_MAGIC;
        const std::uint32_t ver = i == 4 ? 2 : 1;
        put(b, 0, &magic, 4);
        put(b, 4, &ver, 4);
        const std::uint32_t seq = static_cast<std::uint32_t>(rng.next() % 1000) * 2;
        put(b, 8, &seq, 4);
        const std::uint32_t pid = static_cast<std::uint32_t>(rng.next() % 4000000);
        put(b, 12, &pid, 4);
        const std::size_t n = rng.next() % 140;
        const std::uint64_t base = 100000000000ull + rng.next() % 1000000000;
        for (std::size_t k = 0; k < std::min(n, anchor::RING_SLOTS); k++) {
            const std::size_t slot = (k * 7 + i) % anchor::RING_SLOTS;
            const std::size_t o = anchor::RING_HDR + slot * anchor::RING_SLOT;
            std::uint64_t ts;
            switch (rng.next() % 10) {
            case 0: ts = 0; break;
            case 1: ts = base; break;
            default: ts = base + rng.next() % 2000000000; break;
            }
            put(b, o, &ts, 8);
            for (std::size_t c = 0; c < 7; c++) {
                float v = static_cast<float>(rng.sym() * 3.0);
                if (rng.next() % 50 == 0) {
                    v = NAN;
                }
                if (rng.next() % 80 == 0) {
                    v = INFINITY;
                }
                put(b, o + 8 + 4 * c, &v, 4);
            }
            const std::uint32_t fl = static_cast<std::uint32_t>(rng.next() % 4);
            put(b, o + 36, &fl, 4);
        }
        const std::size_t cut = i == 5 ? anchor::RING_SIZE - 1 : b.size();
        auto r = anchor::HeadRing::parse(std::string_view(b).substr(0, cut));
        if (!r) {
            std::printf("ring %zu none\n", i);
            continue;
        }
        std::printf("ring %zu pid %u n %zu newest %" PRIu64 "\n", i, r->pid, r->samples.size(), r->newest_ns());
        std::uint64_t h = 0xCBF29CE484222325ull;
        for (const auto& s : r->samples) {
            const std::string line = std::to_string(s.ts_ns) + (s.valid ? " true " : " false ") + hp(s.pose) + "\n";
            for (unsigned char c : line) {
                h = (h ^ c) * 0x100000001B3ull;
            }
        }
        std::printf("  samples %016" PRIx64 "\n", h);
        for (int q = 0; q < 12; q++) {
            const std::uint64_t t = base + rng.next() % 2200000000ull - 100000000ull;
            const std::uint64_t gaps[] = {0, 1, 3000000, 100000000, 400000000, UINT64_MAX / 2};
            const std::uint64_t gap = gaps[rng.next() % 6];
            if (auto p = r->at(t, gap)) {
                std::printf("  at %" PRIu64 " %" PRIu64 " %s\n", t, gap, hp(*p).c_str());
            } else {
                std::printf("  at %" PRIu64 " %" PRIu64 " none\n", t, gap);
            }
        }
    }
    for (int i = 0; i < 10; i++) {
        const std::size_t n = rng.next() % 150;
        std::vector<anchor::RingSample> s;
        for (std::size_t k = 0; k < n; k++) {
            const std::uint64_t ts = 1 + k * 3000000ull + rng.next() % 1000;
            const Pose p = rng.pose(3.0, 2.0);
            const bool v = rng.next() % 2 == 0;
            s.push_back({ts, p, v});
        }
        const std::string b = anchor::ring_bytes(static_cast<std::uint32_t>(rng.next() % 99999), s);
        std::uint64_t h = 0xCBF29CE484222325ull;
        for (unsigned char c : b) {
            h = (h ^ c) * 0x100000001B3ull;
        }
        std::printf("ringbytes %d %zu %016" PRIx64 "\n", i, b.size(), h);
    }
    for (int i = 0; i < 60; i++) {
        const Pose a = rng.pose(3.0, 3.0);
        const Pose b = rng.pose(3.0, 3.0);
        std::printf("co %d %s | %s\n", i, hp(anchor::compute_offset(a, b, true)).c_str(),
                    hp(anchor::compute_offset(a, b, false)).c_str());
        // Rust evaluates the whole array literal: the random draw happens every time
        const double fs[6] = {0.0, 1.0, 0.5, rng.unit(), -0.2, 1.3};
        const double f = fs[i % 6];
        const Pose b2 = i % 4 == 0 ? Pose{g::mat_mul(a.r, g::rodrigues({1e-4, 0.0, 0.0})), b.t} : b;
        std::printf("ip %d %s\n", i, hp(anchor::interp(a, b2, f)).c_str());
        const auto [dt, dr] = anchor::pose_error(a, b);
        std::printf("pe %d %s %s\n", i, hx(dt).c_str(), hx(dr).c_str());
    }
    for (int i = 0; i < 30; i++) {
        const std::size_t n = rng.next() % 25;
        const Pose c = rng.pose(3.0, 1.0);
        std::vector<Pose> v;
        for (std::size_t k = 0; k < n; k++) {
            const double rot = rng.next() % 5 == 0 ? 2.0 : 0.02;
            const double tr = rng.next() % 4 == 0 ? 0.5 : 0.01;
            const Pose d = rng.pose(rot, tr);
            v.push_back(c.compose(d));
        }
        if (auto m = anchor::robust_mean(v, 0.02 + 0.01 * static_cast<double>(i % 3), 3.0)) {
            const auto& [mp, k, st, sr] = *m;
            std::printf("rm %d %s %zu %s %s\n", i, hp(mp).c_str(), k, hx(st).c_str(), hx(sr).c_str());
        } else {
            std::printf("rm %d none\n", i);
        }
    }
    using anchor::Role;
    const Role roles[] = {Role::Setup, Role::Auto, Role::Follow, Role::Reloc, Role::Apply, Role::Tag};
    const std::vector<std::optional<anchor::Lease>> curs = {
        std::nullopt,
        anchor::Lease::parse("roommap 9\n"),
        anchor::Lease::parse("anchor 9\n"),
        anchor::Lease::parse("roommap 9\nlease 2\nwriter 200\nrole auto\n"),
        anchor::Lease::parse("roommap 9\nlease 2\nwriter 300\nrole follow\n"),
        anchor::Lease::parse("anchor 9\nlease 2\nwriter 300\nrole tag\n"),
        anchor::Lease::parse("roommap 9\nlease 2\nwriter 0\nrole setup\n"),
        anchor::Lease::parse("roommap 10\nlease 2\nwriter 300\nrole setup\n"),
        anchor::Lease::parse("roommap 9\nlease 7\nwriter 300\nrole setup\n"),
        anchor::Lease::parse("roommap 9\nlease 2\nwriter 400\nrole reloc\n"),
    };
    for (std::size_t ci = 0; ci < curs.size(); ci++) {
        for (std::uint32_t me : {200u, 400u}) {
            for (Role r : roles) {
                for (bool written : {false, true}) {
                    for (int alive_set = 0; alive_set < 2; alive_set++) {
                        auto alive = [&](std::uint32_t p) { return alive_set == 1 && (p == 200 || p == 300); };
                        const auto d = anchor::decide_lease(curs[ci], 9, me, r, written, alive);
                        std::string ds;
                        if (d) {
                            ds = *d ? "Ok(Some(" + std::to_string(**d) + "))" : "Ok(None)";
                        } else {
                            ds = "Err(" + rf::debug_str(d.error()) + ")";
                        }
                        std::printf("dl %zu %u %s %s %d %s\n", ci, me, anchor::role_name(r), written ? "true" : "false",
                                    alive_set, ds.c_str());
                    }
                }
            }
        }
    }
    auto mk = [](std::uint32_t pid, std::uint64_t newest) {
        Pose p;
        p.r = g::rodrigues({0.0, 0.0, 0.0});
        p.t = {0.0, 1.6, 0.0};
        std::vector<anchor::RingSample> s;
        if (newest != 0) {
            s = {{newest - 3000000, p, true}, {newest, p, true}};
        }
        return anchor::HeadRing::parse(anchor::ring_bytes(pid, s));
    };
    const std::vector<std::optional<anchor::HeadRing>> rings = {mk(43262, 100000000000ull), mk(182552, 100000000000ull),
                                                                mk(43262, 0), std::nullopt};
    const std::vector<std::vector<std::uint32_t>> mons = {{43262}, {}, {1, 43262}, {7}};
    for (std::size_t ri = 0; ri < rings.size(); ri++) {
        for (std::size_t mi = 0; mi < mons.size(); mi++) {
            for (std::uint64_t now : {1ull, 100000000000ull, 101000000000ull, 105000000001ull, 172000000000ull,
                                      99000000000ull}) {
                for (bool al : {false, true}) {
                    const auto f = anchor::check_feed(rings[ri], mons[mi], now, [&](std::uint32_t) { return al; });
                    const auto lp = f.live_pid();
                    std::printf("feed %zu %zu %" PRIu64 " %s %s %s | %s\n", ri, mi, now, al ? "true" : "false",
                                feed_debug(f).c_str(), lp ? ("Some(" + std::to_string(*lp) + ")").c_str() : "None",
                                f.plain().c_str());
                }
            }
        }
    }
    const char* cfgs[] = {
        "[tag]\nid = 3\nsize_m = 0.15\n[reference]\npos = [1, 2, 3]\nquat = [0, 0, 0, 1]\nsaved = \"unix 5\"\n",
        "# c\n[tag]\nfamily = \"tag36h11\"\nid = 0\nsize_m = 0.2 # x\n\n[reference]\npos = [0.1, -0.2, 0.3]\nquat = [0.1, 0.2, 0.3, 0.9]\nsaved = \"x\"\n[[window]]\nname = \"term\"\npos = [1,2,3]\nquat = [0,0,0,1]\n[[window]]\nname = \"bad\"\npos = [1,2]\nquat = [0,0,0,1]\n",
        "[tag]\nid = x\n",
        "[tag]\nid = 1\nsize_m = 1e\n",
        "[tag]\nid = 1\nsize_m = 1\n[reference]\npos = [1, 2, 3]\n",
        "[tag]\nid = 1\nsize_m = 1\n[reference]\npos = [1, , 3]\nquat=[0,0,0,1]\n",
        "[tag]\nid = 1\nsize_m = 1\n[reference]\npos = 1, 2, 3\nquat=[0,0,0,1,2]\n",
        "garbage\n",
        "[tag]\nid = -1\n",
        "[tag]\nsize_m = 0.15\nid = 99999999999\n",
        "[[window]]\nname = \"a\"\n[tag]\nid = 1\nsize_m = 1\n[reference]\npos = [0,0,0]\nquat=[0,0,0,2]\n[window]\nname=\"b\"\npos=[1,1,1]\nquat=[1,0,0,0]\n",
        "",
        "[tag]\nid = 1\nsize_m = 1\n[reference]\npos = []\nquat=[0,0,0,1]\n",
    };
    for (std::size_t i = 0; i < std::size(cfgs); i++) {
        auto a = anchor::Anchor::parse(cfgs[i]);
        if (a) {
            std::printf("cfg %zu %u %s %s %s %zu | %s\n", i, a->id, hx(a->size_m).c_str(), hp(a->reference).c_str(),
                        rf::debug_str(a->saved).c_str(), a->windows.size(), rf::debug_str(a->to_toml()).c_str());
        } else {
            std::printf("cfg %zu err %s\n", i, a.error().c_str());
        }
    }
    return 0;
}
