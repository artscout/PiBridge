// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-anchor/src/main.rs (MIT OR Apache-2.0): `status` and `clear`. The AprilTag commands
// (`tag`, `detect`, `calibrate`, `apply`) need the AprilTag detector (apriltag C library) and are not ported: they
// exit 2 with a message (docs/pibridge-roommap.md).
#include "pibridge/anchor/config.h"
#include "pibridge/anchor/rustx.h"
#include "pibridge/anchor/world.h"
#include "pibridge/cam/rustfmt.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <unistd.h>

using namespace pibridge;
namespace g = cam::geom;
namespace rf = cam::rfmt;

namespace {

[[noreturn]] void usage() {
    std::fputs("pibridge-anchor: room anchor (AprilTag tag36h11 on the desk), port of omarchyvr-anchor\n"
               "\n"
               "  pibridge-anchor status             config, head ring, offset file\n"
               "  pibridge-anchor clear              remove the offset (identity)\n"
               "\n"
               "not ported (they need the AprilTag detector): tag, detect, calibrate, apply; use omarchyvr-anchor\n",
               stderr);
    std::exit(2);
}

std::string describe(const g::Pose& p) {
    const g::Vec3 n = g::mat_vec(p.r, {0.0, 0.0, 1.0});
    return "pos (" + rf::fixed(p.t[0], 3, true) + " " + rf::fixed(p.t[1], 3, true) + " " + rf::fixed(p.t[2], 3, true) +
           ") m, tag normal (" + rf::fixed(n[0], 2, true) + " " + rf::fixed(n[1], 2, true) + " " +
           rf::fixed(n[2], 2, true) + ")";
}

void cmd_status() {
    if (auto c = anchor::Anchor::load()) {
        std::printf("config %s: tag36h11 id %u size %s mm, %zu saved windows, saved %s\n  reference %s\n",
                    anchor::anchor_config_path().c_str(), c->id, rf::fixed(c->size_m * 1e3, 0).c_str(),
                    c->windows.size(), c->saved.c_str(), describe(c->reference).c_str());
    } else {
        std::printf("config: %s\n", c.error().c_str());
    }
    if (auto r = anchor::HeadRing::read(anchor::ring_path())) {
        const std::uint64_t now = anchor::rs::monotonic_ns();
        const double age = static_cast<double>(now > r->newest_ns() ? now - r->newest_ns() : 0) / 1e9;
        std::printf("head ring %s: pid %u, newest sample %s s ago\n", anchor::ring_path().c_str(), r->pid,
                    rf::fixed(age, 1).c_str());
        std::printf("  %s\n", anchor::feed_now().plain().c_str());
    } else {
        std::printf("head ring %s: absent (Monado not running with PIMAX_ANCHOR=1)\n", anchor::ring_path().c_str());
    }
    if (auto o = anchor::read_offset(anchor::offset_path())) {
        std::printf("offset file: pid %u, %s\n", o->first, describe(o->second).c_str());
    } else {
        std::printf("offset file: none (identity)\n");
    }
}

void cmd_clear() {
    const std::string p = anchor::offset_path();
    if (::unlink(p.c_str()) == 0) {
        std::printf("offset removed (%s)\n", p.c_str());
    } else {
        std::printf("nothing to remove: %s\n", cam::IoError::last_os().text().c_str());
    }
}

} // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0); // line by line, as Rust's stdout
    if (argc < 2) {
        usage();
    }
    const std::string c = argv[1];
    if (c == "status") {
        cmd_status();
    } else if (c == "clear") {
        cmd_clear();
    } else if (c == "tag" || c == "detect" || c == "calibrate" || c == "apply") {
        std::fprintf(stderr, "pibridge-anchor %s: not ported (needs the AprilTag detector); use omarchyvr-anchor\n", c.c_str());
        return 2;
    } else {
        usage();
    }
    std::fflush(stdout);
    return 0;
}
