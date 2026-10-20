// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/rec.rs (MIT OR Apache-2.0). See rec.h.
#include "pibridge/roommap/rec.h"

#include "pibridge/anchor/rustx.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <cmath>
#include <dirent.h>
#include <map>
#include <numbers>
#include <sys/stat.h>

namespace pibridge::roommap::rec {

namespace rf = cam::rfmt;
namespace rs = anchor::rs;
namespace sl = cam::slam;

const sl::Quat SLAM_TO_YUP = {-std::numbers::sqrt2 / 2.0, 0.0, 0.0, std::numbers::sqrt2 / 2.0};

namespace {

/// Entries of a directory (full paths), unsorted; an error text when it cannot be read.
std::vector<std::string> entries(const std::string& d, std::string* err) {
    std::vector<std::string> v;
    DIR* dir = ::opendir(d.c_str());
    if (dir == nullptr) {
        if (err != nullptr) {
            *err = cam::IoError::last_os().text();
        }
        return v;
    }
    while (const dirent* e = ::readdir(dir)) {
        const std::string n = e->d_name;
        if (n != "." && n != "..") {
            v.push_back(cam::path_join(d, n));
        }
    }
    ::closedir(dir);
    return v;
}

std::optional<std::pair<std::int64_t, std::int64_t>> mtime(const std::string& p) {
    struct stat st {};
    if (::stat(p.c_str(), &st) != 0) {
        return std::nullopt;
    }
    return std::pair<std::int64_t, std::int64_t>{st.st_mtim.tv_sec, st.st_mtim.tv_nsec};
}

std::expected<std::vector<std::pair<std::int64_t, std::string>>, std::string> read_cam(const std::string& root, int c) {
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
}

} // namespace

std::expected<std::string, std::string> dataset_root(const std::string& d) {
    if (cam::is_dir(cam::path_join(d, "mav0"))) {
        return d;
    }
    std::string err;
    auto all = entries(d, &err);
    if (!err.empty()) {
        return std::unexpected(d + ": " + err);
    }
    std::vector<std::string> v;
    for (auto& p : all) {
        if (cam::is_dir(cam::path_join(p, "mav0"))) {
            v.push_back(p);
        }
    }
    std::sort(v.begin(), v.end());
    if (v.empty()) {
        return std::unexpected(d + ": no EuRoC dataset (mav0)");
    }
    return v.back();
}

std::optional<std::string> find_tracking(const std::string& d) {
    std::string err;
    auto all = entries(d, &err);
    if (!err.empty()) {
        return std::nullopt;
    }
    std::vector<std::string> v;
    for (auto& p : all) {
        if (rs::file_name(p).starts_with("replay-") && cam::is_file(cam::path_join(p, "tracking.csv"))) {
            v.push_back(p);
        }
    }
    // sort_by_key(modified time): stable; read_dir order first (the directory's own order, as Rust sees it)
    std::stable_sort(v.begin(), v.end(), [](const std::string& a, const std::string& b) {
        return mtime(cam::path_join(a, "tracking.csv")) < mtime(cam::path_join(b, "tracking.csv"));
    });
    if (v.empty()) {
        return std::nullopt;
    }
    return cam::path_join(v.back(), "tracking.csv");
}

Traj parse_tracking(std::string_view text) {
    Traj v;
    for (std::string_view l0 : rs::lines(text)) {
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
        if (f.size() < 8 || std::any_of(f.begin(), f.end(), [](double x) { return !std::isfinite(x); })) {
            continue;
        }
        const sl::Quat q = sl::qmul(SLAM_TO_YUP, sl::qnorm({f[5], f[6], f[7], f[4]}));
        const cam::geom::Vec3 p = sl::qrot(SLAM_TO_YUP, {f[1], f[2], f[3]});
        v.emplace_back(rs::to_i64(f[0]), Pose{sl::mat_of(q), p});
    }
    std::stable_sort(v.begin(), v.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    return v;
}

std::optional<Pose> pose_at(const Traj& tr, std::int64_t ts) {
    const std::size_t i = static_cast<std::size_t>(
        std::partition_point(tr.begin(), tr.end(), [&](const auto& x) { return x.first < ts; }) - tr.begin());
    const Pose* best = nullptr;
    std::int64_t bd = 0;
    for (std::size_t j : {i == 0 ? std::size_t{0} : i - 1, i}) {
        if (j >= tr.size()) {
            continue;
        }
        const std::int64_t d = tr[j].first > ts ? tr[j].first - ts : ts - tr[j].first;
        if (d > 5000000) {
            continue;
        }
        if (best == nullptr || d < bd) {
            best = &tr[j].second;
            bd = d;
        }
    }
    if (best == nullptr) {
        return std::nullopt;
    }
    return *best;
}

std::expected<Recording, std::string> Recording::open(const std::string& dir, const std::optional<std::string>& tracking) {
    auto root = dataset_root(dir);
    if (!root) {
        return std::unexpected(root.error());
    }
    Recording r;
    r.root = *root;
    if (tracking) {
        r.tracking = *tracking;
    } else {
        auto t = find_tracking(dir);
        if (!t) {
            if (auto parent = cam::path_parent(r.root)) {
                t = find_tracking(parent->empty() ? std::string() : *parent);
            }
        }
        if (!t) {
            return std::unexpected(dir + ": no replay-*/tracking.csv; run `analysis/tools/run_slam.sh replay " + dir +
                                   "` first or pass --tracking");
        }
        r.tracking = *t;
    }
    auto text = cam::read_to_string(r.tracking);
    if (!text) {
        return std::unexpected(r.tracking + ": " + text.error().text());
    }
    r.traj = parse_tracking(*text);
    if (r.traj.empty()) {
        return std::unexpected(r.tracking + ": no poses");
    }
    auto c0 = read_cam(r.root, 0);
    if (!c0) {
        return std::unexpected(c0.error());
    }
    auto c1 = read_cam(r.root, 1);
    if (!c1) {
        return std::unexpected(c1.error());
    }
    std::map<std::int64_t, std::string> m1;
    for (auto& [t, f] : *c1) {
        m1[t] = f;
    }
    for (auto& [t, f] : *c0) {
        const auto it = m1.find(t);
        if (it == m1.end()) {
            continue;
        }
        r.frames.push_back(FrameRef{t, cam::path_join(cam::path_join(r.root, "mav0/cam0/data"), f),
                                    cam::path_join(cam::path_join(r.root, "mav0/cam1/data"), it->second)});
    }
    return r;
}

std::string Recording::name() const {
    const std::string fname = rs::file_name(root);
    std::string d = root;
    if (fname.starts_with("euroc_")) {
        d = cam::path_parent(root).value_or(root);
    }
    const std::string n = rs::file_name(d);
    return n.empty() ? "recording" : n;
}

} // namespace pibridge::roommap::rec
