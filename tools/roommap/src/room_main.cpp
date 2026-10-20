// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-room/src/main.rs (MIT OR Apache-2.0): `pibridge-room`, the room-memory prototype tool
// (build a map from a EuRoC recording + its tracker trajectory, relocalise a second recording, info, anchors, the
// synthetic two-session demo). Output and files are the Rust tool's; errors start with "pibridge-room <cmd>:".
#include "pibridge/anchor/rustx.h"
#include "pibridge/cam/calib.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/cam/slam.h"
#include "pibridge/roommap/map.h"
#include "pibridge/roommap/pngread.h"
#include "pibridge/roommap/session.h"
#include "pibridge/roommap/stereo.h"
#include "pibridge/roommap/synth.h"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <dirent.h>
#include <expected>
#include <limits>
#include <map>
#include <numbers>
#include <string>
#include <vector>

using namespace pibridge;
using namespace pibridge::roommap;
namespace g = cam::geom;
namespace rf = cam::rfmt;
namespace sl = cam::slam;
namespace rs = anchor::rs;
using g::Pose;

namespace {

template <class T>
using R = std::expected<T, std::string>;

const sl::Quat SLAM_TO_YUP = {-std::numbers::sqrt2 / 2.0, 0.0, 0.0, std::numbers::sqrt2 / 2.0};

R<std::optional<std::string>> opt(std::vector<std::string>& v, const std::string& name) {
    const auto it = std::find(v.begin(), v.end(), name);
    if (it == v.end()) {
        return std::optional<std::string>{};
    }
    const std::size_t i = static_cast<std::size_t>(it - v.begin());
    if (i + 1 < v.size()) {
        std::string x = v[i + 1];
        v.erase(v.begin() + static_cast<std::ptrdiff_t>(i), v.begin() + static_cast<std::ptrdiff_t>(i) + 2);
        return std::optional<std::string>{x};
    }
    return std::unexpected(name + " needs a value");
}

bool flag(std::vector<std::string>& v, const std::string& name) {
    const auto it = std::find(v.begin(), v.end(), name);
    if (it == v.end()) {
        return false;
    }
    v.erase(it);
    return true;
}

R<std::string> pos(std::vector<std::string>& v, const std::string& what) {
    if (v.empty() || v[0].starts_with("--")) {
        return std::unexpected("missing " + what);
    }
    std::string x = v[0];
    v.erase(v.begin());
    return x;
}

R<cam::DeviceCalib> calib(const std::optional<std::string>& p) {
    auto r = cam::DeviceCalib::load_default(p);
    if (!r) {
        return std::unexpected(r.error().text());
    }
    std::fprintf(stderr, "calibration %s (%s)\n", r->second.c_str(), r->first.uid.c_str());
    return r->first;
}

/// Tracker trajectory, converted to y up.
R<std::vector<std::pair<std::int64_t, Pose>>> load_tracking(const std::string& p) {
    auto t = cam::read_to_string(p);
    if (!t) {
        return std::unexpected(p + ": " + t.error().text());
    }
    std::vector<std::pair<std::int64_t, Pose>> v;
    for (std::string_view l0 : rs::lines(*t)) {
        const std::string_view l = rf::trim(l0);
        if (l.empty() || l.front() == '#') {
            continue;
        }
        std::vector<double> f;
        std::size_t s = 0;
        for (;;) {
            const std::size_t c = l.find(',', s);
            const std::string_view x = rf::trim(l.substr(s, c == std::string_view::npos ? std::string_view::npos : c - s));
            double d = 0;
            if (rf::parse_f64(x, d)) {
                f.push_back(d);
            }
            if (c == std::string_view::npos) {
                break;
            }
            s = c + 1;
        }
        if (f.size() < 8) {
            continue;
        }
        const sl::Quat q = sl::qmul(SLAM_TO_YUP, sl::qnorm({f[5], f[6], f[7], f[4]}));
        const g::Vec3 pp = sl::qrot(SLAM_TO_YUP, {f[1], f[2], f[3]});
        v.emplace_back(rs::to_i64(f[0]), Pose{sl::mat_of(q), pp});
    }
    std::stable_sort(v.begin(), v.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    if (v.empty()) {
        return std::unexpected(p + ": no poses");
    }
    return v;
}

/// Tracker pose at a frame time (nearest within 5 ms).
std::optional<Pose> pose_at(const std::vector<std::pair<std::int64_t, Pose>>& tr, std::int64_t ts) {
    const std::size_t i = static_cast<std::size_t>(
        std::partition_point(tr.begin(), tr.end(), [&](const auto& x) { return x.first < ts; }) - tr.begin());
    std::optional<std::pair<std::int64_t, Pose>> best;
    for (std::size_t j : {i == 0 ? 0 : i - 1, i}) {
        if (j < tr.size()) {
            const std::int64_t d = tr[j].first > ts ? tr[j].first - ts : ts - tr[j].first;
            if (d <= 5000000 && (!best || d < best->first)) {
                best = std::pair{d, tr[j].second};
            }
        }
    }
    if (best) {
        return best->second;
    }
    return std::nullopt;
}

std::vector<std::string> list_dirs_with_mav0(const std::string& d, std::string* err) {
    std::vector<std::string> v;
    DIR* dir = ::opendir(d.c_str());
    if (dir == nullptr) {
        *err = cam::IoError::last_os().text();
        return v;
    }
    while (const dirent* e = ::readdir(dir)) {
        const std::string n = e->d_name;
        if (n == "." || n == "..") {
            continue;
        }
        const std::string p = cam::path_join(d, n);
        if (cam::is_dir(cam::path_join(p, "mav0"))) {
            v.push_back(p);
        }
    }
    ::closedir(dir);
    std::sort(v.begin(), v.end());
    return v;
}

R<std::string> dataset_root(const std::string& d) {
    if (cam::is_dir(cam::path_join(d, "mav0"))) {
        return d;
    }
    std::string err;
    auto v = list_dirs_with_mav0(d, &err);
    if (!err.empty()) {
        return std::unexpected(d + ": " + err);
    }
    if (v.empty()) {
        return std::unexpected(d + ": no EuRoC dataset (mav0)");
    }
    return v.back();
}

struct FrameSet {
    std::int64_t ts;
    std::string a;
    std::string b;
};

R<std::vector<FrameSet>> frames(const std::string& root) {
    auto read = [&](int c) -> R<std::vector<std::pair<std::int64_t, std::string>>> {
        const std::string p = cam::path_join(root, "mav0/cam" + std::to_string(c) + "/data.csv");
        auto t = cam::read_to_string(p);
        if (!t) {
            return std::unexpected(p + ": " + t.error().text());
        }
        std::vector<std::pair<std::int64_t, std::string>> out;
        for (std::string_view l : rs::lines(*t)) {
            if (l.starts_with('#') || rf::trim(l).empty()) {
                continue;
            }
            const auto kv = rs::split_once(l, ',');
            if (!kv) {
                continue;
            }
            const auto ts = rs::parse_i64(rf::trim(kv->first));
            if (!ts) {
                continue;
            }
            out.emplace_back(*ts, std::string(rf::trim(kv->second)));
        }
        return out;
    };
    auto c0 = read(0);
    if (!c0) {
        return std::unexpected(c0.error());
    }
    auto c1 = read(1);
    if (!c1) {
        return std::unexpected(c1.error());
    }
    std::map<std::int64_t, std::string> m1;
    for (auto& [t, f] : *c1) {
        m1[t] = f;
    }
    std::vector<FrameSet> out;
    for (auto& [t, f] : *c0) {
        const auto it = m1.find(t);
        if (it == m1.end()) {
            continue;
        }
        out.push_back(FrameSet{t, cam::path_join(cam::path_join(root, "mav0/cam0/data"), f),
                               cam::path_join(cam::path_join(root, "mav0/cam1/data"), it->second)});
    }
    return out;
}

std::string fixw(double x, int width, int prec, bool plus = false) {
    return rf::fixed_w(x, width, prec, plus);
}

/// Run a session over a recording: (session, per-frame alignments from single relocalisations).
R<std::pair<session::RoomSession, std::vector<std::pair<std::int64_t, session::Alignment>>>> run(
    session::RoomSession s, const std::string& ds, const std::string& tracking, std::size_t every, bool verbose) {
    auto root = dataset_root(ds);
    if (!root) {
        return std::unexpected(root.error());
    }
    auto tr = load_tracking(tracking);
    if (!tr) {
        return std::unexpected(tr.error());
    }
    auto fr = frames(*root);
    if (!fr) {
        return std::unexpected(fr.error());
    }
    std::fprintf(stderr, "%s: %zu frame sets, %zu tracker poses\n", root->c_str(), fr->size(), tr->size());
    std::vector<std::pair<std::int64_t, session::Alignment>> fixes;
    const session::Alignment world{};
    for (std::size_t k = 0; k < fr->size(); k++) {
        if (k % std::max<std::size_t>(every, 1) != 0) {
            continue;
        }
        const auto& f = (*fr)[k];
        const auto sb = pose_at(*tr, f.ts);
        if (!sb) {
            continue;
        }
        auto ia = pngread::read(f.a);
        if (!ia) {
            return std::unexpected(ia.error());
        }
        auto ib = pngread::read(f.b);
        if (!ib) {
            return std::unexpected(ib.error());
        }
        const orb::Image views[2] = {orb::Image{ia->w, ia->h, ia->w, ia->data}, orb::Image{ib->w, ib->h, ib->w, ib->data}};
        for (const auto& e : s.on_frame(f.ts, views, *sb, world, true)) {
            if (e.kind == session::Event::Kind::Keyframe) {
                continue;
            }
            fixes.emplace_back(f.ts, e.align);
            if (verbose) {
                std::printf("%s s %s: yaw %s deg t (%s %s %s)\n",
                            fixw(static_cast<double>(f.ts - (*fr)[0].ts) / 1e9, 8, 2).c_str(),
                            e.kind == session::Event::Kind::Relocalized ? "\"relocalised\"" : "\"refined\"",
                            rf::fixed(rf::to_degrees(e.align.yaw), 2, true).c_str(), rf::fixed(e.align.t[0], 3, true).c_str(),
                            rf::fixed(e.align.t[1], 3, true).c_str(), rf::fixed(e.align.t[2], 3, true).c_str());
            }
        }
    }
    return std::pair{std::move(s), std::move(fixes)};
}

std::size_t parse_every(const std::optional<std::string>& x) {
    if (!x) {
        return 1;
    }
    return rs::parse_usize(*x).value_or(1);
}

R<void> cmd_build(std::vector<std::string> v) {
    auto c = opt(v, "--calib");
    if (!c) {
        return std::unexpected(c.error());
    }
    auto cal = calib(*c);
    if (!cal) {
        return std::unexpected(cal.error());
    }
    auto ev = opt(v, "--every");
    if (!ev) {
        return std::unexpected(ev.error());
    }
    const std::size_t every = parse_every(*ev);
    auto ds = pos(v, "DATASET");
    if (!ds) {
        return std::unexpected(ds.error());
    }
    auto tr = pos(v, "TRACKING.csv");
    if (!tr) {
        return std::unexpected(tr.error());
    }
    auto out = pos(v, "OUT_DIR");
    if (!out) {
        return std::unexpected(out.error());
    }
    const std::size_t ids[2] = {0, 1};
    auto rig = stereo::Rig::from_calib(*cal, ids);
    if (!rig) {
        return std::unexpected(rig.error());
    }
    session::RoomSession s(map::RoomMap::make(cal->uid, ids), *rig, session::SessionParams{});
    auto r = run(std::move(s), *ds, *tr, every, false);
    if (!r) {
        return std::unexpected(r.error());
    }
    const auto st = r->first.stats;
    auto m = std::move(r->first).finish();
    auto p = m.save(*out);
    if (!p) {
        return std::unexpected(p.error());
    }
    std::printf("%s: %zu keyframes, %zu points (%" PRIu64 " keyframe inserts)\n", p->c_str(), m.keyframes.size(),
                m.points(), st.keyframes);
    return {};
}

R<void> cmd_reloc(std::vector<std::string> v) {
    auto c = opt(v, "--calib");
    if (!c) {
        return std::unexpected(c.error());
    }
    auto cal = calib(*c);
    if (!cal) {
        return std::unexpected(cal.error());
    }
    auto ev = opt(v, "--every");
    if (!ev) {
        return std::unexpected(ev.error());
    }
    const std::size_t every = parse_every(*ev);
    const bool save = flag(v, "--save");
    auto mdir = pos(v, "MAP_DIR");
    if (!mdir) {
        return std::unexpected(mdir.error());
    }
    auto ds = pos(v, "DATASET");
    if (!ds) {
        return std::unexpected(ds.error());
    }
    auto tr = pos(v, "TRACKING.csv");
    if (!tr) {
        return std::unexpected(tr.error());
    }
    auto m = map::RoomMap::load(*mdir);
    if (!m) {
        return std::unexpected(m.error());
    }
    std::printf("map: %zu keyframes, %zu points, %u sessions\n", m->keyframes.size(), m->points(), m->sessions);
    const std::size_t ids[2] = {0, 1};
    auto rig = stereo::Rig::from_calib(*cal, ids);
    if (!rig) {
        return std::unexpected(rig.error());
    }
    session::SessionParams sp;
    sp.refine_every_ns = 500000000;
    auto r = run(session::RoomSession(std::move(*m), *rig, sp), *ds, *tr, every, true);
    if (!r) {
        return std::unexpected(r.error());
    }
    const auto st = r->first.stats;
    const auto& fixes = r->second;
    if (st.first_fix_ns) {
        std::printf("first fix after %s s; %" PRIu64 " attempts, %" PRIu64 " found, %" PRIu64 " refinements, %" PRIu64
                    " new keyframes\n",
                    rf::fixed(static_cast<double>(*st.first_fix_ns) / 1e9, 2).c_str(), st.attempts, st.found, st.refined,
                    st.keyframes);
    } else {
        std::printf("room NOT recognised (%" PRIu64 " attempts, %" PRIu64 " single relocalisations)\n", st.attempts,
                    st.found);
    }
    if (fixes.size() >= 2) {
        const double n = static_cast<double>(fixes.size());
        g::Vec3 mt{0.0, 0.0, 0.0};
        for (const auto& f : fixes) {
            mt = g::add(mt, g::scale(f.second.t, 1.0 / n));
        }
        double s2 = -0.0;
        for (const auto& f : fixes) {
            const double e = g::norm(g::sub(f.second.t, mt));
            s2 += e * e;
        }
        const double sd_t = std::sqrt(s2 / n);
        const double y0 = fixes[0].second.yaw;
        std::vector<double> dy;
        for (const auto& f : fixes) {
            dy.push_back(std::atan2(std::sin(f.second.yaw - y0), std::cos(f.second.yaw - y0)));
        }
        double sy = -0.0;
        for (double d : dy) {
            sy += d;
        }
        const double my = sy / n;
        double vy = -0.0;
        for (double d : dy) {
            vy += (d - my) * (d - my);
        }
        const double sd_y = std::sqrt(vy / n);
        std::printf("room_from_slam over %zu fixes: translation spread %s mm RMS, yaw spread %s deg RMS (tracker drift + "
                    "relocalisation noise)\n",
                    fixes.size(), rf::fixed(sd_t * 1000.0, 1).c_str(), rf::fixed(rf::to_degrees(sd_y), 2).c_str());
    }
    if (save) {
        auto mm = std::move(r->first).finish();
        auto p = mm.save(*mdir);
        if (!p) {
            return std::unexpected(p.error());
        }
        std::printf("saved %s (%zu keyframes)\n", p->c_str(), mm.keyframes.size());
    }
    return {};
}

R<void> cmd_info(std::vector<std::string> v) {
    auto d = pos(v, "MAP_DIR");
    if (!d) {
        return std::unexpected(d.error());
    }
    auto m = map::RoomMap::load(*d);
    if (!m) {
        return std::unexpected(m.error());
    }
    g::Vec3 lo{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    g::Vec3 hi{std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
               std::numeric_limits<double>::lowest()};
    for (const auto& k : m->keyframes) {
        for (int i = 0; i < 3; i++) {
            lo[i] = rf::fmin(lo[i], k.pose.t[i]);
            hi[i] = rf::fmax(hi[i], k.pose.t[i]);
        }
    }
    std::string cams = "[";
    for (std::size_t i = 0; i < m->cams.size(); i++) {
        cams += (i > 0 ? ", " : "") + std::to_string(m->cams[i]);
    }
    cams += "]";
    std::printf("device %s cams %s: %u sessions, %zu keyframes, %zu points; keyframe positions x %s..%s y %s..%s z %s..%s m\n",
                m->device_uid.c_str(), cams.c_str(), m->sessions, m->keyframes.size(), m->points(),
                rf::fixed(lo[0], 2).c_str(), rf::fixed(hi[0], 2).c_str(), rf::fixed(lo[1], 2).c_str(),
                rf::fixed(hi[1], 2).c_str(), rf::fixed(lo[2], 2).c_str(), rf::fixed(hi[2], 2).c_str());
    for (const auto& a : m->anchors) {
        std::string name = a.name;
        if (name.size() < 16) {
            name += std::string(16 - name.size(), ' ');
        }
        std::printf("anchor %s p (%s %s %s) yaw %s deg\n", name.c_str(), rf::fixed(a.p[0], 3, true).c_str(),
                    rf::fixed(a.p[1], 3, true).c_str(), rf::fixed(a.p[2], 3, true).c_str(),
                    rf::fixed(rf::to_degrees(sl::yaw_of(a.q)), 1, true).c_str());
    }
    return {};
}

R<void> cmd_anchor(std::vector<std::string> v) {
    auto d = pos(v, "MAP_DIR");
    if (!d) {
        return std::unexpected(d.error());
    }
    auto what = pos(v, "list|set|rm");
    if (!what) {
        return std::unexpected(what.error());
    }
    auto m = map::RoomMap::load(*d);
    if (!m) {
        return std::unexpected(m.error());
    }
    if (*what == "list") {
        for (const auto& a : m->anchors) {
            std::printf("%s %s %s %s %s\n", a.name.c_str(), rf::fixed(a.p[0], 4).c_str(), rf::fixed(a.p[1], 4).c_str(),
                        rf::fixed(a.p[2], 4).c_str(), rf::fixed(rf::to_degrees(sl::yaw_of(a.q)), 2).c_str());
        }
        return {};
    }
    if (*what == "set") {
        auto name = pos(v, "NAME");
        if (!name) {
            return std::unexpected(name.error());
        }
        double n[4] = {0.0, 0.0, 0.0, 0.0};
        for (int i = 0; i < 4; i++) {
            if (i == 3 && v.empty()) {
                break;
            }
            auto x = pos(v, "X Y Z");
            if (!x) {
                return std::unexpected(x.error());
            }
            auto f = rs::parse_f64(*x);
            if (!f) {
                return std::unexpected("bad number");
            }
            n[i] = *f;
        }
        m->set_anchor(*name, {n[0], n[1], n[2]}, sl::qyaw(rf::to_radians(n[3])));
    } else if (*what == "rm") {
        auto name = pos(v, "NAME");
        if (!name) {
            return std::unexpected(name.error());
        }
        if (!m->remove_anchor(*name)) {
            return std::unexpected("no anchor " + rf::debug_str(*name));
        }
    } else {
        return std::unexpected("anchor: list, set or rm");
    }
    auto p = m->save(*d);
    if (!p) {
        return std::unexpected(p.error());
    }
    std::printf("%zu anchors\n", m->anchors.size());
    return {};
}

std::string stats_debug(const session::SessionStats& s) {
    return "SessionStats { attempts: " + std::to_string(s.attempts) + ", found: " + std::to_string(s.found) +
           ", keyframes: " + std::to_string(s.keyframes) + ", refined: " + std::to_string(s.refined) +
           ", first_fix_ns: " + (s.first_fix_ns ? "Some(" + std::to_string(*s.first_fix_ns) + ")" : std::string("None")) +
           " }";
}

R<void> cmd_demo() {
    auto cal = calib(std::nullopt);
    if (!cal) {
        return std::unexpected(cal.error());
    }
    const std::size_t ids[2] = {0, 1};
    auto rig = stereo::Rig::from_calib(*cal, ids);
    if (!rig) {
        return std::unexpected(rig.error());
    }
    std::vector<synth::RayTable> rays;
    for (const auto& c : rig->cams) {
        rays.push_back(synth::RayTable::make(c.intr));
    }
    auto render = [&](const synth::Room& room, const Pose& rb) {
        std::vector<std::vector<std::uint8_t>> v;
        for (std::size_t i = 0; i < rig->cams.size(); i++) {
            v.push_back(room.render_rays(rays[i], rb.compose(rig->cams[i].body_from_cam)));
        }
        return v;
    };
    auto walk = [](std::size_t n, double r, double ph) {
        std::vector<Pose> v;
        for (std::size_t i = 0; i < n; i++) {
            const double a = ph + static_cast<double>(i) / static_cast<double>(n) * (2.0 * std::numbers::pi);
            const sl::Quat q = sl::qmul(sl::qyaw(a + 0.5), {std::sin(-0.1), 0.0, 0.0, std::cos(-0.1)});
            v.push_back(Pose{sl::mat_of(q), {r * std::sin(a), 1.55, r * std::cos(a)}});
        }
        return v;
    };
    const synth::Room room;
    const auto t0 = std::chrono::steady_clock::now();
    session::RoomSession s1(map::RoomMap::make(cal->uid, ids), *rig, session::SessionParams{});
    const session::Alignment id{};
    const auto w1 = walk(30, 0.9, 0.0);
    for (std::size_t k = 0; k < w1.size(); k++) {
        const auto ims = render(room, w1[k]);
        const orb::Image v[2] = {orb::Image{640, 480, 640, ims[0]}, orb::Image{640, 480, 640, ims[1]}};
        s1.on_frame(static_cast<std::int64_t>(k) * 100000000, v, w1[k], id, true);
    }
    auto m = std::move(s1).finish();
    std::printf("session 1: %zu keyframes, %zu points (%s s)\n", m.keyframes.size(), m.points(),
                rf::fixed(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), 1).c_str());
    const Pose slam_from_room{sl::mat_of(sl::qyaw(-0.8)), {-1.0, 0.0, 3.0}};
    const Pose truth = slam_from_room.inverse();
    session::RoomSession s2(std::move(m), *rig, session::SessionParams{});
    synth::Room dim;
    dim.gain = 0.6;
    const auto w2 = walk(30, 0.6, 0.1);
    for (std::size_t k = 0; k < 12; k++) {
        const auto ims = render(dim, w2[k]);
        const orb::Image v[2] = {orb::Image{640, 480, 640, ims[0]}, orb::Image{640, 480, 640, ims[1]}};
        const Pose sb = slam_from_room.compose(w2[k]);
        for (const auto& e : s2.on_frame(static_cast<std::int64_t>(k) * 250000000, v, sb, id, true)) {
            if (e.kind == session::Event::Kind::Keyframe) {
                continue;
            }
            const Pose est = e.align.pose();
            const double dt = g::norm(g::sub(est.t, truth.t));
            const double dy = rf::to_degrees(sl::yaw_of(sl::quat_of(est.r)) - sl::yaw_of(sl::quat_of(truth.r)));
            std::printf("session 2 frame %zu: room found, alignment error %s mm, %s deg\n", k,
                        rf::fixed(dt * 1000.0, 1).c_str(), rf::fixed(dy, 2, true).c_str());
        }
    }
    std::printf("%s\n", stats_debug(s2.stats).c_str());
    return {};
}

const char* HELP = "`omarchyvr-room`: room memory tools (offline prototype of map persistence + relocalisation).\n"
                   "\n"
                   "```text\n"
                   "omarchyvr-room build  DATASET TRACKING.csv OUT_DIR [--calib XML] [--every N]\n"
                   "session 1: keyframes from a EuRoC recording + its tracker trajectory -> OUT_DIR/map.bin\n"
                   "omarchyvr-room reloc  MAP_DIR DATASET TRACKING.csv [--calib XML] [--every N] [--save]\n"
                   "session 2: relocalise a second recording in the saved room; time to the first fix and the\n"
                   "spread of room_from_slam over the recording (consistency = accuracy proxy)\n"
                   "omarchyvr-room info   MAP_DIR\n"
                   "omarchyvr-room anchor MAP_DIR list | set NAME X Y Z [YAW_DEG] | rm NAME\n"
                   "omarchyvr-room demo                synthetic room, two sessions (self-check of the pipeline)\n"
                   "```\n"
                   "TRACKING.csv: `ts,px,py,pz,qw,qx,qy,qz` of the IMU in the tracker world (Basalt: z up; converted to y up),\n"
                   "e.g. `tracking.csv` of `run_slam.sh replay`. DATASET: EuRoC root (mav0) or a recording directory.";

} // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0); // line by line, as Rust's stdout
    std::vector<std::string> v(argv + 1, argv + argc);
    if (v.empty() || v[0] == "-h" || v[0] == "--help") {
        std::fprintf(stderr, "%s\n", HELP);
        std::fprintf(stderr, "default room directory: %s\n", map::room_dir("<name>").c_str());
        return 2;
    }
    const std::string cmd = v[0];
    v.erase(v.begin());
    R<void> r;
    if (cmd == "build") {
        r = cmd_build(v);
    } else if (cmd == "reloc") {
        r = cmd_reloc(v);
    } else if (cmd == "info") {
        r = cmd_info(v);
    } else if (cmd == "anchor") {
        r = cmd_anchor(v);
    } else if (cmd == "demo") {
        r = cmd_demo();
    } else {
        r = std::unexpected("unknown command " + rf::debug_str(cmd) + " (build, reloc, info, anchor, demo)");
    }
    if (!r) {
        std::fprintf(stderr, "pibridge-room %s: %s\n", cmd.c_str(), r.error().c_str());
        return 1;
    }
    return 0;
}
