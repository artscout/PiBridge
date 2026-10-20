// SPDX-License-Identifier: MIT
#include "pibridge/xrkit/ptsynth.h"

#include "pibridge/cam/slam.h"

#include <cmath>

namespace pibridge::xrkit::ptsynth {

std::expected<pt::Rig, std::string> referenceRig(bool upper) {
    auto cal = cam::DeviceCalib::parse(cam::reference_xml());
    if (!cal) {
        return std::unexpected(cal.error().text());
    }
    const auto h = cam::slam::default_head_in_imu(*cal);
    return pt::makeRig(*cal, {h[0], h[1], h[2]}, upper);
}

cam::geom::Pose geomPose(const room::Mat34& m) {
    cam::geom::Pose p;
    for (std::size_t r = 0; r < 3; r++) {
        for (std::size_t c = 0; c < 3; c++) {
            p.r[r][c] = m.m[r][c];
        }
        p.t[r] = m.m[r][3];
    }
    return p;
}

room::Mat34 headPose(room::Vec3 p, double heading, double pitch) {
    // yaw about +y, then pitch about the head's x
    const double cy = std::cos(heading), sy = std::sin(heading), cp = std::cos(pitch), sp = std::sin(pitch);
    room::Mat34 yaw, pit;
    yaw.m = {{{cy, 0, sy, 0}, {0, 1, 0, 0}, {-sy, 0, cy, 0}}};
    pit.m = {{{1, 0, 0, 0}, {0, cp, -sp, 0}, {0, sp, cp, 0}}};
    room::Mat34 m = room::mul34(yaw, pit);
    m.m[0][3] = p.x;
    m.m[1][3] = p.y;
    m.m[2][3] = p.z;
    return m;
}

pt::Frame synthFrame(const roommap::synth::Room& room, const pt::Rig& rig, const room::Mat34& stageFromView, std::uint64_t seq) {
    pt::Frame f;
    for (std::size_t c = 0; c < rig.n; c++) {
        f.img[c] = room.render(rig.cam[c].kb, geomPose(room::mul34(stageFromView, rig.cam[c].viewFromCam)));
        f.means[c] = pt::meanOf(f.img[c]);
    }
    f.n = rig.n;
    f.seq = seq;
    f.mean = rig.n >= 2 ? 0.5F * (f.means[0] + f.means[1]) : f.means[0];
    return f;
}

pt::RoomShape roomShape(const roommap::synth::Room& room) {
    const auto& lo = room.shell.lo;
    const auto& hi = room.shell.hi;
    pt::RoomShape s;
    s.outline = {{lo[0], lo[2]}, {hi[0], lo[2]}, {hi[0], hi[2]}, {lo[0], hi[2]}};
    s.ceilingY = hi[1];
    return s;
}

} // namespace pibridge::xrkit::ptsynth
