// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/store.rs (MIT OR Apache-2.0). See store.h.
#include "pibridge/roommap/store.h"

#include "pibridge/anchor/backend.h"

#include <cstdio>
#include <filesystem>

#include "pibridge/anchor/rustx.h"
#include "pibridge/anchor/world.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <sys/stat.h>

namespace pibridge::roommap::store {

namespace g = cam::geom;
namespace rf = cam::rfmt;
namespace rs = anchor::rs;

std::string backend_rooms_dir(anchor::Backend b) {
    const std::string home = cam::env_os("HOME").value_or(".");
    return cam::path_join(cam::path_join(home, ".local/share/pibridge/rooms"), std::string(anchor::backend_name(b)));
}

std::string base_dir() {
    for (const char* n : {"PIBRIDGE_ROOMMAP_DIR", "PIMAX_ROOMMAP_DIR"}) {
        if (auto v = cam::env_os(n); v && !v->empty()) {
            return *v;
        }
    }
    const std::string home = cam::env_os("HOME").value_or(".");
    if (const auto b = anchor::backend_from_env()) {
        // Each backend has its own rooms. The rooms saved before the split were all SteamVR's (2.0.1): they are copied
        // there once, the old folder stays as it was (a backup).
        const std::string dir = backend_rooms_dir(*b);
        const std::string old = cam::path_join(home, ".local/share/pibridge/roommap");
        if (*b == anchor::Backend::SteamVr && !cam::is_dir(dir) && cam::is_dir(old)) {
            std::error_code ec;
            std::filesystem::create_directories(dir, ec);
            std::filesystem::copy(old, dir, std::filesystem::copy_options::recursive, ec);
            std::fprintf(stderr, "rooms: the rooms of %s copied to %s (SteamVR backend)%s\n", old.c_str(), dir.c_str(),
                         ec ? (": " + ec.message()).c_str() : "");
        }
        return dir;
    }
    const std::string pibridge = cam::path_join(home, ".local/share/pibridge/roommap");
    const std::string omarchyvr = cam::path_join(home, ".local/share/omarchyvr/roommap");
    if (!cam::is_dir(pibridge) && cam::is_dir(omarchyvr)) {
        return omarchyvr;
    }
    return pibridge;
}

std::string map_dir(const std::string& name) {
    if (name.find('/') != std::string::npos) {
        return name;
    }
    return cam::path_join(base_dir(), name);
}

std::string RelocResult::to_toml() const {
    return "# written by omarchyvr-roommap relocalize: room_from_slam = the Monado world offset (app_from_slam)\nmap = \"" +
           map + "\"\nsource = \"" + source + "\"\nyaw_deg = " + rf::fixed(rf::to_degrees(align.yaw), 6) + "\nt = [" +
           rf::fixed(align.t[0], 6) + ", " + rf::fixed(align.t[1], 6) + ", " + rf::fixed(align.t[2], 6) +
           "]\nframes = " + std::to_string(frames) + "\ninliers = " + std::to_string(inliers) +
           "\nrms_deg = " + rf::fixed(rms_deg, 4) + "\nsigma_deg = " + rf::fixed(sigma_deg, 4) + "\nsigma_m = " +
           rf::fixed(sigma_m, 5) + "\nfix_after_s = " + rf::fixed(fix_after_s, 3) + "\npid = " + std::to_string(pid) + "\n";
}

std::expected<RelocResult, std::string> RelocResult::parse(std::string_view s) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    RelocResult r;
    r.align = Alignment{nan, {nan, nan, nan}};
    r.rms_deg = r.sigma_deg = r.sigma_m = r.fix_after_s = nan;
    for (std::string_view line : rs::lines(s)) {
        const std::size_t h = line.find('#');
        const std::string_view l = rf::trim(h == std::string_view::npos ? line : line.substr(0, h));
        const auto kv = rs::split_once(l, '=');
        if (!kv) {
            continue;
        }
        const std::string_view k = rf::trim(kv->first);
        const std::string_view v = rf::trim(kv->second);
        std::string err;
        auto num = [&](std::string_view x) { return rs::parse_f64(x, &err); };
        auto bad = [&]() { return std::unexpected(std::string(k) + ": " + err); };
        if (k == "map") {
            r.map = std::string(rs::trim_matches(v, "\""));
        } else if (k == "source") {
            r.source = std::string(rs::trim_matches(v, "\""));
        } else if (k == "yaw_deg") {
            auto x = num(v);
            if (!x) {
                return bad();
            }
            r.align.yaw = rf::to_radians(*x);
        } else if (k == "t") {
            std::vector<double> x;
            std::string_view rest = rs::trim_matches(v, "[]");
            for (;;) {
                const std::size_t c = rest.find(',');
                const std::string_view item = rf::trim(c == std::string_view::npos ? rest : rest.substr(0, c));
                std::string e;
                auto d = rs::parse_f64(item, &e);
                if (!d) {
                    return std::unexpected("t: " + e);
                }
                x.push_back(*d);
                if (c == std::string_view::npos) {
                    break;
                }
                rest = rest.substr(c + 1);
            }
            if (x.size() != 3) {
                return std::unexpected("t needs 3 numbers");
            }
            r.align.t = {x[0], x[1], x[2]};
        } else if (k == "frames") {
            auto x = num(v);
            if (!x) {
                return bad();
            }
            r.frames = rs::to_usize(*x);
        } else if (k == "inliers") {
            auto x = num(v);
            if (!x) {
                return bad();
            }
            r.inliers = rs::to_usize(*x);
        } else if (k == "rms_deg") {
            auto x = num(v);
            if (!x) {
                return bad();
            }
            r.rms_deg = *x;
        } else if (k == "sigma_deg") {
            auto x = num(v);
            if (!x) {
                return bad();
            }
            r.sigma_deg = *x;
        } else if (k == "sigma_m") {
            auto x = num(v);
            if (!x) {
                return bad();
            }
            r.sigma_m = *x;
        } else if (k == "fix_after_s") {
            auto x = num(v);
            if (!x) {
                return bad();
            }
            r.fix_after_s = *x;
        } else if (k == "pid") {
            auto x = num(v);
            if (!x) {
                return bad();
            }
            r.pid = rs::to_u32(*x);
        }
    }
    if (!std::isfinite(r.align.yaw) || !std::isfinite(r.align.t[0]) || !std::isfinite(r.align.t[1]) ||
        !std::isfinite(r.align.t[2])) {
        return std::unexpected("yaw_deg / t missing");
    }
    return r;
}

std::expected<RelocResult, std::string> RelocResult::load(const std::string& path) {
    auto t = cam::read_to_string(path);
    if (!t) {
        return std::unexpected(path + ": " + t.error().text());
    }
    return parse(*t);
}

std::expected<void, std::string> RelocResult::save(const std::string& path) const {
    if (auto d = cam::path_parent(path); d && !d->empty()) {
        if (auto r = cam::create_dir_all(*d); !r) {
            return std::unexpected(r.error().text());
        }
    }
    const std::string tmp = rs::with_extension(path, "toml.tmp");
    if (auto w = cam::write_file(tmp, to_toml()); !w) {
        return std::unexpected(w.error().text());
    }
    if (std::rename(tmp.c_str(), path.c_str()) != 0) {
        return std::unexpected(cam::IoError::last_os().text());
    }
    return {};
}

cam::geom::Pose offset_pose(const Alignment& a) {
    return a.pose();
}

cam::Result<std::string> write_offset(const Alignment& a, std::uint32_t pid) {
    const std::string p = anchor::offset_path();
    if (auto w = anchor::write_offset(p, pid, offset_pose(a)); !w) {
        return cam::fail(w.error());
    }
    return p;
}

Alignment compose(const Alignment& a, const Alignment& b) {
    const cam::geom::Pose p = a.pose().compose(b.pose());
    return Alignment{align::wrap(a.yaw + b.yaw), p.t};
}

Alignment inverse(const Alignment& a) {
    const cam::geom::Pose p = a.pose().inverse();
    return Alignment{align::wrap(-a.yaw), p.t};
}

std::string meta_text(const std::string& source, std::size_t keyframes, std::size_t points, std::uint64_t created_unix) {
    return "# omarchyvr-roommap map: room frame = the tracker world of the mapping session (y up, gravity aligned)\n"
           "source = \"" + source + "\"\nkeyframes = " + std::to_string(keyframes) + "\npoints = " + std::to_string(points) +
           "\ncreated = \"unix " + std::to_string(created_unix) + "\"\n";
}

std::expected<void, std::string> save_corrections(const std::string& dir, const std::vector<loopclose::Correction>& c) {
    std::string t = "# ts_ns,yaw_rad,tx,ty,tz: room_from_slam of the mapping session after loop closure\n";
    for (const auto& x : c) {
        t += std::to_string(x.ts_ns) + "," + rf::fixed(x.c.yaw, 9) + "," + rf::fixed(x.c.t[0], 6) + "," +
             rf::fixed(x.c.t[1], 6) + "," + rf::fixed(x.c.t[2], 6) + "\n";
    }
    if (auto w = cam::write_file(cam::path_join(dir, "correction.csv"), t); !w) {
        return std::unexpected(w.error().text());
    }
    return {};
}

namespace {

std::optional<std::pair<std::int64_t, std::int64_t>> mtime(const std::string& p) {
    struct stat st {};
    if (::stat(p.c_str(), &st) != 0) {
        return std::nullopt;
    }
    return std::pair<std::int64_t, std::int64_t>{st.st_mtim.tv_sec, st.st_mtim.tv_nsec};
}

} // namespace

std::optional<Alignment> last_correction(const std::string& dir) {
    const auto c = mtime(cam::path_join(dir, "correction.csv"));
    if (!c) {
        return std::nullopt;
    }
    const auto m = mtime(cam::path_join(dir, "map.bin"));
    if (!m) {
        return std::nullopt;
    }
    // c + 2 s < m: the correction is older than the map
    const std::pair<std::int64_t, std::int64_t> c2{c->first + 2, c->second};
    if (c2 < *m) {
        return std::nullopt;
    }
    const auto all = load_corrections(dir);
    if (all.empty()) {
        return std::nullopt;
    }
    return all.back().c;
}

Alignment fix_after_mapping(const Alignment& offset_at_mapping, const std::optional<Alignment>& correction) {
    if (correction) {
        return compose(*correction, offset_at_mapping);
    }
    return offset_at_mapping;
}

std::vector<loopclose::Correction> load_corrections(const std::string& dir) {
    std::vector<loopclose::Correction> out;
    auto t = cam::read_to_string(cam::path_join(dir, "correction.csv"));
    if (!t) {
        return out;
    }
    for (std::string_view l : rs::lines(*t)) {
        if (l.starts_with('#')) {
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
        if (f.size() == 5) {
            out.push_back(loopclose::Correction{rs::to_i64(f[0]), Alignment{f[1], {f[2], f[3], f[4]}}});
        }
    }
    return out;
}

} // namespace pibridge::roommap::store
