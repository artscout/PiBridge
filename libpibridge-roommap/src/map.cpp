// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-room/src/map.rs (MIT OR Apache-2.0). See map.h.
#include "pibridge/roommap/map.h"

#include "pibridge/anchor/rustx.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/json.h"
#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

namespace pibridge::roommap::map {

namespace g = cam::geom;
namespace rf = cam::rfmt;
namespace sl = cam::slam;

Mat3 tilt_of(const Quat& q) {
    return sl::mat_of(sl::qmul(sl::qyaw(-sl::yaw_of(q)), q));
}

double yaw_of_mat(const Mat3& r) {
    return sl::yaw_of(sl::quat_of(r));
}

std::pair<double, Vec3> room_from_slam(const Reloc& r, const Pose& slam_from_body) {
    const Pose m = r.room_from_body.compose(slam_from_body.inverse());
    const double yaw = yaw_of_mat(m.r);
    const Mat3 ry = sl::mat_of(sl::qyaw(yaw));
    const Vec3 t = g::sub(r.room_from_body.t, g::mat_vec(ry, slam_from_body.t));
    return {yaw, t};
}

RoomMap RoomMap::make(const std::string& device_uid, std::span<const std::size_t> cams) {
    RoomMap m;
    m.device_uid = device_uid;
    for (std::size_t c : cams) {
        m.cams.push_back(static_cast<std::uint32_t>(c));
    }
    return m;
}

std::size_t RoomMap::points() const {
    std::size_t n = 0;
    for (const auto& k : keyframes) {
        n += k.points.size();
    }
    return n;
}

bool RoomMap::wants_keyframe(const Pose& room_from_body, double min_dist, double min_angle_deg) const {
    return wants_keyframe_in(room_from_body, min_dist, min_angle_deg, std::nullopt);
}

bool RoomMap::wants_keyframe_in(const Pose& room_from_body, double min_dist, double min_angle_deg,
                                std::optional<std::uint32_t> session) const {
    const Vec3 fwd = g::mat_vec(room_from_body.r, {0.0, 0.0, -1.0});
    for (const auto& k : keyframes) {
        if (session && k.session != *session) {
            continue;
        }
        const double d = g::norm(g::sub(k.pose.t, room_from_body.t));
        const Vec3 kf = g::mat_vec(k.pose.r, {0.0, 0.0, -1.0});
        if (d < min_dist && g::angle_deg(fwd, kf) < min_angle_deg) {
            return false;
        }
    }
    return true;
}

std::uint32_t RoomMap::add_keyframe(std::int64_t ts_ns, const Pose& room_from_body,
                                    std::span<const stereo::StereoPoint> pts) {
    const std::uint32_t id = next_id;
    next_id += 1;
    Keyframe k;
    k.id = id;
    k.ts_ns = ts_ns;
    k.session = sessions;
    k.pose = room_from_body;
    k.points.reserve(pts.size());
    for (const auto& s : pts) {
        k.points.push_back(MapPoint{room_from_body.apply(s.p_body), s.desc});
    }
    keyframes.push_back(std::move(k));
    return id;
}

std::optional<Reloc> RoomMap::relocalize(std::span<const stereo::Features> query, const stereo::Rig& rig,
                                         const Mat3& r_tilt, const std::optional<std::pair<Vec3, double>>& prior,
                                         const RelocParams& p) const {
    std::vector<const Keyframe*> cands;
    for (const auto& k : keyframes) {
        if (!prior || g::norm(g::sub(k.pose.t, prior->first)) <= prior->second) {
            cands.push_back(&k);
        }
    }
    if (cands.empty()) {
        return std::nullopt;
    }
    std::vector<const orb::Descriptor*> all_q;
    for (const auto& f : query) {
        for (const auto& d : f.desc) {
            all_q.push_back(&d);
        }
    }
    if (all_q.empty()) {
        return std::nullopt;
    }
    const std::size_t stride = std::max<std::size_t>(all_q.size() / std::max<std::size_t>(p.rank_sample, 1), 1);
    std::vector<const orb::Descriptor*> sample;
    for (std::size_t i = 0; i < all_q.size(); i += stride) {
        sample.push_back(all_q[i]);
    }
    std::vector<std::pair<std::size_t, const Keyframe*>> scored;
    scored.reserve(cands.size());
    for (const Keyframe* k : cands) {
        std::size_t hits = 0;
        for (const orb::Descriptor* q : sample) {
            for (const auto& m : k->points) {
                if (orb::hamming(*q, m.desc) <= p.rank_dist) {
                    hits++;
                    break;
                }
            }
        }
        scored.emplace_back(hits, k);
    }
    std::stable_sort(scored.begin(), scored.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
    std::vector<const Keyframe*> top;
    for (std::size_t i = 0; i < scored.size() && i < std::max<std::size_t>(p.top_k, 1); i++) {
        if (scored[i].first > 0) {
            top.push_back(scored[i].second);
        }
    }
    if (top.empty()) {
        return std::nullopt;
    }
    std::vector<const MapPoint*> pts;
    for (const Keyframe* k : top) {
        for (const auto& m : k->points) {
            pts.push_back(&m);
        }
    }
    std::vector<orb::Descriptor> pdesc;
    pdesc.reserve(pts.size());
    for (const MapPoint* m : pts) {
        pdesc.push_back(m->desc);
    }
    std::vector<pnp::Corr> corrs;
    for (std::size_t c = 0; c < query.size(); c++) {
        for (const auto& [i, j, d] : orb::match_bf(query[c].desc, pdesc, p.max_dist, p.ratio)) {
            (void)d;
            corrs.push_back(pnp::Corr{c, query[c].bearings[i], pts[j]->p});
        }
    }
    if (corrs.size() < p.min_inliers) {
        return std::nullopt;
    }
    const auto bfc = rig.body_from_cams();
    const auto res = pnp::ransac(corrs, bfc, r_tilt, p.pnp);
    if (!res) {
        return std::nullopt;
    }
    if (res->inliers.size() < p.min_inliers ||
        static_cast<double>(res->inliers.size()) < p.min_inlier_frac * static_cast<double>(corrs.size())) {
        return std::nullopt;
    }
    Reloc r;
    r.room_from_body = res->pose.to_pose(r_tilt);
    r.inliers = res->inliers.size();
    r.corrs = corrs.size();
    r.rms_deg = res->rms_deg;
    for (const Keyframe* k : top) {
        r.keyframes.push_back(k->id);
    }
    return r;
}

void RoomMap::set_anchor(const std::string& name, const Vec3& p, const Quat& q) {
    std::erase_if(anchors, [&](const Anchor& a) { return a.name == name; });
    anchors.push_back(Anchor{name, p, q});
}

bool RoomMap::remove_anchor(const std::string& name) {
    const std::size_t n = anchors.size();
    std::erase_if(anchors, [&](const Anchor& a) { return a.name == name; });
    return n != anchors.size();
}

namespace {

template <class T>
void put(std::string& b, T v) {
    char buf[sizeof(T)];
    std::memcpy(buf, &v, sizeof(T));
    b.append(buf, sizeof(T));
}

void put_str(std::string& b, const std::string& s) {
    put<std::uint32_t>(b, static_cast<std::uint32_t>(s.size()));
    b += s;
}

void put_pose(std::string& b, const Pose& p) {
    const Quat q = sl::quat_of(p.r);
    for (double x : p.t) {
        put<double>(b, x);
    }
    for (double x : q) {
        put<double>(b, x);
    }
}

struct Cursor {
    std::string_view d;
    std::size_t i = 0;
    std::expected<std::string_view, std::string> take(std::size_t n) {
        if (i + n > d.size()) {
            return std::unexpected("truncated room map");
        }
        const std::string_view s = d.substr(i, n);
        i += n;
        return s;
    }
    template <class T>
    std::expected<T, std::string> num() {
        auto s = take(sizeof(T));
        if (!s) {
            return std::unexpected(s.error());
        }
        T v;
        std::memcpy(&v, s->data(), sizeof(T));
        return v;
    }
    std::expected<std::string, std::string> string() {
        auto n = num<std::uint32_t>();
        if (!n) {
            return std::unexpected(n.error());
        }
        if (*n > 4096) {
            return std::unexpected("bad string length in room map");
        }
        auto s = take(*n);
        if (!s) {
            return std::unexpected(s.error());
        }
        return rf::utf8_lossy(*s);
    }
    std::expected<Pose, std::string> pose() {
        double v[7];
        for (double& x : v) {
            auto r = num<double>();
            if (!r) {
                return std::unexpected(r.error());
            }
            x = *r;
        }
        return Pose{sl::mat_of(sl::qnorm({v[3], v[4], v[5], v[6]})), {v[0], v[1], v[2]}};
    }
};

#define TRY(var, expr)                                                                                                 \
    auto var##_r = (expr);                                                                                             \
    if (!var##_r) {                                                                                                    \
        return std::unexpected(var##_r.error());                                                                       \
    }                                                                                                                  \
    auto var = *var##_r

} // namespace

std::string RoomMap::encode() const {
    std::string b;
    b.append(MAGIC, 4);
    put<std::uint32_t>(b, VERSION);
    put_str(b, device_uid);
    put<std::uint32_t>(b, static_cast<std::uint32_t>(cams.size()));
    for (std::uint32_t c : cams) {
        put<std::uint32_t>(b, c);
    }
    put<std::uint32_t>(b, sessions);
    put<std::uint32_t>(b, next_id);
    put<std::uint32_t>(b, static_cast<std::uint32_t>(keyframes.size()));
    for (const auto& k : keyframes) {
        put<std::uint32_t>(b, k.id);
        put<std::int64_t>(b, k.ts_ns);
        put<std::uint32_t>(b, k.session);
        put_pose(b, k.pose);
        put<std::uint32_t>(b, static_cast<std::uint32_t>(k.points.size()));
        for (const auto& m : k.points) {
            for (double x : m.p) {
                put<float>(b, static_cast<float>(x));
            }
            b.append(reinterpret_cast<const char*>(m.desc.data()), 32);
        }
    }
    put<std::uint32_t>(b, static_cast<std::uint32_t>(anchors.size()));
    for (const auto& a : anchors) {
        put_str(b, a.name);
        for (double x : a.p) {
            put<double>(b, x);
        }
        for (double x : a.q) {
            put<double>(b, x);
        }
    }
    return b;
}

std::expected<RoomMap, std::string> RoomMap::decode(std::string_view data) {
    Cursor r{data};
    TRY(magic, r.take(4));
    if (magic != std::string_view(MAGIC, 4)) {
        return std::unexpected("not a pimax room map (magic)");
    }
    TRY(v, r.num<std::uint32_t>());
    if (v != VERSION) {
        return std::unexpected("room map version " + std::to_string(v) + ", this build reads " + std::to_string(VERSION));
    }
    RoomMap m;
    TRY(uid, r.string());
    m.device_uid = uid;
    TRY(nc, r.num<std::uint32_t>());
    for (std::uint32_t i = 0; i < std::min<std::uint32_t>(nc, 8); i++) {
        TRY(c, r.num<std::uint32_t>());
        m.cams.push_back(c);
    }
    TRY(sessions, r.num<std::uint32_t>());
    m.sessions = sessions;
    TRY(next_id, r.num<std::uint32_t>());
    m.next_id = next_id;
    TRY(nk, r.num<std::uint32_t>());
    for (std::uint32_t k = 0; k < nk; k++) {
        Keyframe kf;
        TRY(id, r.num<std::uint32_t>());
        kf.id = id;
        TRY(ts, r.num<std::int64_t>());
        kf.ts_ns = ts;
        TRY(session, r.num<std::uint32_t>());
        kf.session = session;
        TRY(pose, r.pose());
        kf.pose = pose;
        TRY(np, r.num<std::uint32_t>());
        kf.points.reserve(std::min<std::uint32_t>(np, 100000));
        for (std::uint32_t i = 0; i < np; i++) {
            MapPoint mp;
            for (double& x : mp.p) {
                TRY(f, r.num<float>());
                x = static_cast<double>(f);
            }
            TRY(desc, r.take(32));
            std::memcpy(mp.desc.data(), desc.data(), 32);
            kf.points.push_back(mp);
        }
        m.keyframes.push_back(std::move(kf));
    }
    TRY(na, r.num<std::uint32_t>());
    for (std::uint32_t k = 0; k < na; k++) {
        TRY(name, r.string());
        double vals[7];
        for (double& x : vals) {
            TRY(f, r.num<double>());
            x = f;
        }
        m.anchors.push_back(Anchor{name, {vals[0], vals[1], vals[2]}, {vals[3], vals[4], vals[5], vals[6]}});
    }
    return m;
}

std::expected<std::string, std::string> RoomMap::save(const std::string& dir) const {
    if (auto r = cam::create_dir_all(dir); !r) {
        return std::unexpected(r.error().text());
    }
    const std::string path = cam::path_join(dir, "map.bin");
    const std::string tmp = cam::path_join(dir, "map.bin.tmp");
    {
        const std::string bytes = encode();
        const int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0666);
        if (fd < 0) {
            return std::unexpected(cam::IoError::last_os().text());
        }
        std::size_t off = 0;
        while (off < bytes.size()) {
            const ssize_t n = ::write(fd, bytes.data() + off, bytes.size() - off);
            if (n < 0) {
                const auto e = cam::IoError::last_os();
                ::close(fd);
                return std::unexpected(e.text());
            }
            off += static_cast<std::size_t>(n);
        }
        if (::fsync(fd) != 0) {
            const auto e = cam::IoError::last_os();
            ::close(fd);
            return std::unexpected(e.text());
        }
        ::close(fd);
    }
    if (std::rename(tmp.c_str(), path.c_str()) != 0) {
        return std::unexpected(cam::IoError::last_os().text());
    }
    std::vector<std::string> al;
    for (const auto& a : anchors) {
        al.push_back("    {\"name\": " + cam::json::quote(a.name) + ", \"p\": [" + rf::fixed(a.p[0], 4) + ", " +
                     rf::fixed(a.p[1], 4) + ", " + rf::fixed(a.p[2], 4) + "], \"q\": [" + rf::fixed(a.q[0], 6) + ", " +
                     rf::fixed(a.q[1], 6) + ", " + rf::fixed(a.q[2], 6) + ", " + rf::fixed(a.q[3], 6) + "]}");
    }
    std::string joined;
    for (std::size_t i = 0; i < al.size(); i++) {
        if (i > 0) {
            joined += ",\n";
        }
        joined += al[i];
    }
    std::string cams_s = "[";
    for (std::size_t i = 0; i < cams.size(); i++) {
        if (i > 0) {
            cams_s += ", ";
        }
        cams_s += std::to_string(cams[i]);
    }
    cams_s += "]";
    const std::string summary = "{\n  \"device_uid\": " + cam::json::quote(device_uid) + ",\n  \"cams\": " + cams_s +
                                ",\n  \"sessions\": " + std::to_string(sessions) + ",\n  \"keyframes\": " +
                                std::to_string(keyframes.size()) + ",\n  \"points\": " + std::to_string(points()) +
                                ",\n  \"anchors\": [\n" + joined + "\n  ]\n}\n";
    if (auto w = cam::write_file(cam::path_join(dir, "room.json"), summary); !w) {
        return std::unexpected(w.error().text());
    }
    return path;
}

std::expected<RoomMap, std::string> RoomMap::load(const std::string& path) {
    const std::string p = cam::is_dir(path) ? cam::path_join(path, "map.bin") : path;
    auto d = cam::read_file(p);
    if (!d) {
        return std::unexpected(p + ": " + d.error().text());
    }
    auto m = decode(*d);
    if (!m) {
        return std::unexpected(p + ": " + m.error());
    }
    return m;
}

std::string room_dir(const std::string& name) {
    const std::string home = cam::env_os("HOME").value_or(".");
    return cam::path_join(cam::path_join(home, ".local/share/omarchyvr/rooms"), name);
}

} // namespace pibridge::roommap::map
