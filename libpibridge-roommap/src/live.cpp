// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/live.rs (MIT OR Apache-2.0). See live.h.
#include "pibridge/roommap/live.h"

#include "pibridge/anchor/world.h"
#include "pibridge/cam/frame.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/cam/source.h"
#include "pibridge/roommap/map.h"

#include <chrono>
#include <cstdio>

namespace pibridge::roommap::live {

namespace rf = cam::rfmt;

std::expected<LiveStats, std::string> capture(const std::string& spec, double seconds,
                                              const std::function<bool(LiveFrame&)>& f, std::array<std::size_t, 2> tiles) {
    cam::ReplayOptions ro;
    auto src = cam::open_source(spec, 0, ro);
    if (!src) {
        return std::unexpected("cannot open source " + rf::debug_str(spec) + ": " + src.error().text());
    }
    std::fprintf(stderr, "source %s\n", (*src)->describe().c_str());
    const std::string ring_path = anchor::ring_path();
    const auto t0 = std::chrono::steady_clock::now();
    LiveStats st;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < seconds) {
        auto fr = (*src)->next(500);
        if (!fr) {
            return std::unexpected("source: " + fr.error().text());
        }
        if (!*fr) {
            if ((*src)->finished()) {
                break;
            }
            continue;
        }
        const auto& raw = **fr;
        const auto m = cam::Meta::parse(raw.data.subspan(0, cam::META_LEN));
        if (!m || m->kind() != cam::FrameKind::Tracking) {
            continue;
        }
        st.frames++;
        const std::uint64_t ts = raw.host_ts_ns;
        const auto rows = cam::tile_rows(&*m, raw.data, raw.stride);
        LiveFrame lf;
        for (std::size_t ci = 0; ci < 2; ci++) {
            lf.imgs[ci].w = cam::TILE_W;
            lf.imgs[ci].h = cam::TILE_H;
            lf.imgs[ci].data.assign(cam::TILE_W * cam::TILE_H, 0);
            cam::copy_tile(raw.data, raw.stride, tiles[ci], rows.y[tiles[ci]], lf.imgs[ci].data, cam::TILE_W);
        }
        const auto ring = anchor::HeadRing::read(ring_path);
        if (!ring) {
            continue;
        }
        st.pid = ring->pid;
        const auto pose = ring->at(ts, 100000000);
        if (!pose) {
            continue;
        }
        st.posed++;
        lf.ts_ns = static_cast<std::int64_t>(ts);
        lf.slam_from_body = *pose;
        lf.pid = ring->pid;
        if (!f(lf)) {
            break;
        }
    }
    return st;
}

session::Alignment current_offset(std::uint32_t pid) {
    if (const auto o = anchor::read_offset(anchor::offset_path()); o && o->first == pid && pid != 0) {
        return session::Alignment{map::yaw_of_mat(o->second.r), o->second.t};
    }
    return session::Alignment{};
}

} // namespace pibridge::roommap::live
