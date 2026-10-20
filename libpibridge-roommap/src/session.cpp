// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-room/src/session.rs (MIT OR Apache-2.0). See session.h.
#include "pibridge/roommap/session.h"

#include "pibridge/cam/rustfmt.h"
#include "pibridge/cam/slam.h"

#include <cmath>
#include <numbers>

namespace pibridge::roommap::session {

namespace g = cam::geom;
namespace rf = cam::rfmt;
namespace sl = cam::slam;

namespace {

double wrap(double a) {
    constexpr double PI = std::numbers::pi;
    while (a > PI) {
        a -= 2.0 * PI;
    }
    while (a < -PI) {
        a += 2.0 * PI;
    }
    return a;
}

} // namespace

Pose Alignment::pose() const {
    return Pose{sl::mat_of(sl::qyaw(yaw)), t};
}

bool Alignment::close(const Alignment& o, double m, double deg) const {
    return g::norm(g::sub(t, o.t)) <= m && rf::to_degrees(std::fabs(wrap(yaw - o.yaw))) <= deg;
}

RoomSession::RoomSession(map::RoomMap m, stereo::Rig rig, SessionParams prm)
    : map(std::move(m)), p(std::move(prm)), rig_(std::move(rig)), had_map_(!map.keyframes.empty()) {}

RoomSession RoomSession::with_alignment(map::RoomMap m, stereo::Rig rig, SessionParams prm,
                                        const Alignment& room_from_slam) {
    RoomSession s(std::move(m), std::move(rig), std::move(prm));
    s.aligned_ = room_from_slam;
    s.had_map_ = false;
    return s;
}

std::vector<Event> RoomSession::on_frame(std::int64_t ts_ns, std::span<const orb::Image> imgs,
                                         const Pose& slam_from_body, const Alignment& world_from_slam, bool tracking) {
    std::vector<Event> ev;
    if (!first_ns_) {
        first_ns_ = ts_ns;
    }
    if (!tracking || imgs.size() < 2 || rig_.cams.size() < 2) {
        return ev;
    }
    if (!had_map_ && !aligned_) {
        aligned_ = world_from_slam;
    }
    auto due = [&](std::int64_t every, std::optional<std::int64_t> last) { return !last || ts_ns - *last >= every; };
    const bool searching = !aligned_;
    const bool want_reloc =
        had_map_ && (searching ? due(p.search_every_ns, last_try_ns_) : due(p.refine_every_ns, last_try_ns_));
    std::optional<Pose> room_from_body_now;
    if (aligned_) {
        room_from_body_now = aligned_->pose().compose(slam_from_body);
    }
    const bool want_kf =
        room_from_body_now && map.keyframes.size() < p.max_keyframes &&
        map.wants_keyframe_in(*room_from_body_now, p.kf_min_dist, p.kf_min_angle_deg,
                              p.kf_per_session ? std::optional<std::uint32_t>(map.sessions) : std::nullopt);
    if (!want_reloc && !want_kf) {
        return ev;
    }
    std::vector<stereo::Features> feats;
    for (std::size_t i = 0; i < imgs.size() && i < rig_.cams.size(); i++) {
        feats.push_back(stereo::features(imgs[i], rig_.cams[i], p.orb));
    }
    if (want_reloc) {
        last_try_ns_ = ts_ns;
        stats.attempts++;
        const g::Mat3 tilt = map::tilt_of(sl::quat_of(slam_from_body.r));
        std::optional<std::pair<Vec3, double>> prior;
        if (room_from_body_now) {
            prior = std::pair{room_from_body_now->t, p.refine_radius};
        }
        if (auto r = map.relocalize(feats, rig_, tilt, prior, p.reloc)) {
            stats.found++;
            const auto [yaw, t] = map::room_from_slam(*r, slam_from_body);
            const Alignment cand{yaw, t};
            std::erase_if(candidates_, [&](const Alignment& c) { return !c.close(cand, p.agree_m, p.agree_deg); });
            candidates_.push_back(cand);
            if (candidates_.size() >= std::max<std::size_t>(p.agree, 1)) {
                const double n = static_cast<double>(candidates_.size());
                Vec3 mean_t{0.0, 0.0, 0.0};
                for (const auto& c : candidates_) {
                    mean_t = g::add(mean_t, g::scale(c.t, 1.0 / n));
                }
                double sum = -0.0;
                for (const auto& c : candidates_) {
                    sum += wrap(c.yaw - cand.yaw);
                }
                const double mean_yaw = cand.yaw + sum / n;
                const Alignment a{wrap(mean_yaw), mean_t};
                if (!aligned_) {
                    aligned_ = a;
                    stats.first_fix_ns = ts_ns - first_ns_.value_or(ts_ns);
                    ev.push_back(Event{Event::Kind::Relocalized, a, 0});
                } else {
                    const Alignment cur = *aligned_;
                    const double dm = g::norm(g::sub(a.t, cur.t));
                    const double dd = rf::to_degrees(std::fabs(wrap(a.yaw - cur.yaw)));
                    if (dm >= p.refine_min_m || dd >= p.refine_min_deg) {
                        aligned_ = a;
                        stats.refined++;
                        ev.push_back(Event{Event::Kind::Refined, a, 0});
                    }
                }
                candidates_.clear();
            } else if (aligned_) {
                const Alignment cur = *aligned_;
                const double dm = g::norm(g::sub(cand.t, cur.t));
                if (dm < p.refine_big_m && rf::to_degrees(std::fabs(wrap(cand.yaw - cur.yaw))) < 5.0) {
                    const double dd = rf::to_degrees(std::fabs(wrap(cand.yaw - cur.yaw)));
                    if (dm >= p.refine_min_m || dd >= p.refine_min_deg) {
                        aligned_ = cand;
                        stats.refined++;
                        ev.push_back(Event{Event::Kind::Refined, cand, 0});
                    }
                    candidates_.clear();
                }
            }
        }
    }
    if (want_kf && aligned_) {
        const auto pts = stereo::triangulate(feats[0], feats[1], rig_.cams[0], rig_.cams[1], p.stereo);
        if (pts.size() >= p.kf_min_points) {
            const Pose rb = aligned_->pose().compose(slam_from_body);
            const std::uint32_t id = map.add_keyframe(ts_ns, rb, pts);
            stats.keyframes++;
            ev.push_back(Event{Event::Kind::Keyframe, {}, id});
        }
    }
    return ev;
}

map::RoomMap RoomSession::finish() && {
    map.sessions += 1;
    return std::move(map);
}

} // namespace pibridge::roommap::session
