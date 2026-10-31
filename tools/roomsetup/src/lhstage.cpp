// SPDX-License-Identifier: MIT
#include "lhstage.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <format>

namespace pibridge::roomsetup {

namespace {

using Q = std::array<double, 4>; // x y z w
using V = std::array<double, 3>;

Q qmul(const Q& a, const Q& b) {
    return {a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1], a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
            a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3], a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
}

V qrot(const Q& q, const V& v) {
    const Q p{v[0], v[1], v[2], 0};
    const Q c{-q[0], -q[1], -q[2], q[3]};
    const Q r = qmul(qmul(q, p), c);
    return {r[0], r[1], r[2]};
}

Q qnorm(Q q) {
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n < 1e-12) {
        return {0, 0, 0, 1};
    }
    for (double& x : q) {
        x /= n;
    }
    return q;
}

struct P {
    Q q;
    V t;
};

P of(const StagePose& s) {
    return {qnorm({s[3], s[4], s[5], s[6]}), {s[0], s[1], s[2]}};
}

StagePose to(const P& p) {
    return {p.t[0], p.t[1], p.t[2], p.q[0], p.q[1], p.q[2], p.q[3]};
}

// a * b (apply b, then a)
P compose(const P& a, const P& b) {
    const V r = qrot(a.q, b.t);
    return {qnorm(qmul(a.q, b.q)), {r[0] + a.t[0], r[1] + a.t[1], r[2] + a.t[2]}};
}

P inverse(const P& p) {
    const Q c{-p.q[0], -p.q[1], -p.q[2], p.q[3]};
    const V r = qrot(c, p.t);
    return {c, {-r[0], -r[1], -r[2]}};
}

} // namespace

std::optional<StagePose> parseStage(std::string_view text) {
    StagePose p{};
    const std::string s(text);
    if (std::sscanf(s.c_str(), "%lf %lf %lf %lf %lf %lf %lf", &p[0], &p[1], &p[2], &p[3], &p[4], &p[5], &p[6]) != 7) {
        return std::nullopt;
    }
    for (const double v : p) {
        if (!std::isfinite(v)) {
            return std::nullopt;
        }
    }
    return to(of(p)); // normalised
}

std::string stageText(const StagePose& p) {
    return std::format("{:.6f} {:.6f} {:.6f} {:.7f} {:.7f} {:.7f} {:.7f}\n", p[0], p[1], p[2], p[3], p[4], p[5], p[6]);
}

StagePose lhStageFromRecenter(const StagePose& oldStage, const RecenterResult& r, std::optional<double> eyeHeight) {
    const auto& h = r.head.pose;
    const Q hq = qnorm({h[3], h[4], h[5], h[6]});
    const V f = qrot(hq, {0, 0, -1});
    const double yaw = std::atan2(-f[0], -f[2]); // forward = (-sin yaw, 0, -cos yaw)
    double floorY = 0.0;                         // keep: the old stage's floor
    if (r.floorStage) {
        floorY = *r.floorStage;
    } else if (eyeHeight) {
        floorY = h[1] - *eyeHeight;
    }
    // the new stage's origin and axes in the old stage: on the floor under the head, -Z where the head looks
    const P newInOld{{0.0, std::sin(yaw / 2.0), 0.0, std::cos(yaw / 2.0)}, {h[0], floorY, h[2]}};
    return to(compose(inverse(newInOld), of(oldStage)));
}

std::string lhStageRuntimePath() {
    const char* rt = std::getenv("XDG_RUNTIME_DIR");
    return std::string(rt != nullptr && *rt != '\0' ? rt : "/tmp") + "/pibridge/monado/lh-stage";
}

std::string lhStageRoomPath(const std::string& roomDir) {
    return roomDir + "/lh-stage";
}

std::string baseStationsPath() {
    const char* rt = std::getenv("XDG_RUNTIME_DIR");
    return std::string(rt != nullptr && *rt != '\0' ? rt : "/tmp") + "/pibridge/monado/basestations";
}

std::vector<room::Mat34> parseBaseStations(std::string_view text) {
    std::vector<room::Mat34> out;
    std::size_t at = 0;
    while (at < text.size()) {
        std::size_t end = text.find('\n', at);
        if (end == std::string_view::npos) {
            end = text.size();
        }
        const std::string line(text.substr(at, end - at));
        at = end + 1;
        char serial[64];
        double v[7];
        if (std::sscanf(line.c_str(), "%63s %lf %lf %lf %lf %lf %lf %lf", serial, &v[0], &v[1], &v[2], &v[3], &v[4],
                        &v[5], &v[6]) != 8) {
            continue;
        }
        const double n = std::sqrt(v[3] * v[3] + v[4] * v[4] + v[5] * v[5] + v[6] * v[6]);
        if (!(n > 0.5)) {
            continue;
        }
        out.push_back(room::poseMatrix(v[3] / n, v[4] / n, v[5] / n, v[6] / n, {v[0], v[1], v[2]}));
    }
    return out;
}

void drawBaseStations(room::Batch& b, const std::vector<room::Mat34>& stations) {
    const room::Rgba wire{0.55F, 0.85F, 1.0F, 0.9F};
    const room::Rgba face{0.3F, 1.0F, 0.75F, 0.95F};
    const double hx = 0.04, hy = 0.04, hz = 0.03; // about the size of a 2.0 base station
    for (const room::Mat34& m : stations) {
        room::Vec3 c[8];
        for (int i = 0; i < 8; i++) {
            c[i] = m.xform({(i & 1) ? hx : -hx, (i & 2) ? hy : -hy, (i & 4) ? hz : -hz});
        }
        for (int i = 0; i < 8; i++) {
            for (int bit : {1, 2, 4}) {
                if ((i & bit) == 0) {
                    b.line(c[i], c[i | bit], wire);
                }
            }
        }
        // the front (-Z): a cross over the face
        b.line(c[0], c[3], face);
        b.line(c[1], c[2], face);
    }
}

} // namespace pibridge::roomsetup
