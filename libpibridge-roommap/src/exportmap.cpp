// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/export.rs (MIT OR Apache-2.0). See exportmap.h.
#include "pibridge/roommap/exportmap.h"

#include "pibridge/anchor/rustx.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/cam/slam.h"

#include <cmath>
#include <cstdio>
#include <map>

namespace pibridge::roommap::exportmap {

namespace rf = cam::rfmt;
namespace rs = anchor::rs;

namespace {

std::string num(double v) {
    if (!std::isfinite(v)) {
        return "null";
    }
    std::string s = rf::fixed(v, 3);
    std::string_view t = rs::trim_end_char(s, '0');
    t = rs::trim_end_char(t, '.');
    if (t.empty() || t == "-0") {
        return "0";
    }
    return std::string(t);
}

std::string esc(std::string_view s) {
    std::string o = "\"";
    for (std::size_t i = 0; i < s.size(); i++) {
        const auto c = static_cast<unsigned char>(s[i]);
        if (c == '"') {
            o += "\\\"";
        } else if (c == '\\') {
            o += "\\\\";
        } else if (c == '<') {
            o += "\\u003c";
        } else if (c < 0x20) {
            char buf[8];
            std::snprintf(buf, sizeof buf, "\\u%04x", c);
            o += buf;
        } else {
            o += static_cast<char>(c);
        }
    }
    o += "\"";
    return o;
}

std::string replace_all(std::string s, std::string_view from, std::string_view to) {
    std::string out;
    std::size_t i = 0;
    for (;;) {
        const std::size_t j = s.find(from, i);
        if (j == std::string::npos) {
            out.append(s, i, std::string::npos);
            break;
        }
        out.append(s, i, j - i);
        out += to;
        i = j + from.size();
    }
    return out;
}

} // namespace

std::vector<std::tuple<double, double, double, std::uint32_t>> merged_points(const map::RoomMap& map) {
    struct Cell {
        double s[3];
        std::uint32_t n;
    };
    std::map<std::tuple<std::int64_t, std::int64_t, std::int64_t>, Cell> cells;
    std::vector<std::tuple<std::int64_t, std::int64_t, std::int64_t>> order;
    for (const auto& k : map.keyframes) {
        for (const auto& p : k.points) {
            if (!std::isfinite(p.p[0]) || !std::isfinite(p.p[1]) || !std::isfinite(p.p[2])) {
                continue;
            }
            const std::tuple<std::int64_t, std::int64_t, std::int64_t> key{rs::to_i64(std::floor(p.p[0] / CELL)),
                                                                          rs::to_i64(std::floor(p.p[1] / CELL)),
                                                                          rs::to_i64(std::floor(p.p[2] / CELL))};
            auto it = cells.find(key);
            if (it == cells.end()) {
                order.push_back(key);
                it = cells.emplace(key, Cell{{0.0, 0.0, 0.0}, 0}).first;
            }
            for (int i = 0; i < 3; i++) {
                it->second.s[i] += p.p[i];
            }
            it->second.n++;
        }
    }
    std::vector<std::tuple<double, double, double, std::uint32_t>> out;
    out.reserve(order.size());
    for (const auto& k : order) {
        const Cell& c = cells.at(k);
        const double n = static_cast<double>(c.n);
        out.emplace_back(c.s[0] / n, c.s[1] / n, c.s[2] / n, c.n);
    }
    return out;
}

std::expected<std::string, std::string> export_json(const std::string& dir, const std::string& name) {
    auto m = map::RoomMap::load(dir);
    if (!m) {
        return std::unexpected(m.error());
    }
    std::optional<floor::StoredFloor> fl;
    if (auto s = floor::StoredFloor::load(dir)) {
        fl = *s;
    } else {
        const auto [pts, heads] = floor::map_points(*m, 3.5);
        if (auto f = floor::detect(pts, heads, floor::FloorParams{})) {
            floor::StoredFloor sf;
            sf.y = f->layer.y;
            sf.source = floor::FloorSource::Points;
            sf.confidence = f->confidence;
            sf.sigma_m = 0.0;
            sf.inliers = 0;
            sf.head_above_m = f->head_above_m;
            sf.when = 0;
            fl = sf;
        }
    }
    std::optional<std::string> meta;
    if (auto t = cam::read_to_string(cam::path_join(dir, "roommap.toml"))) {
        meta = *t;
    }
    return map_json(*m, name, fl ? &*fl : nullptr, meta);
}

std::string map_json(const map::RoomMap& map, const std::string& name, const floor::StoredFloor* fl,
                     const std::optional<std::string>& meta_toml) {
    const auto pts = merged_points(map);
    const std::size_t raw = map.points();
    std::string o;
    o.reserve(pts.size() * 28 + map.keyframes.size() * 90 + 512);
    o += "{\n\"format\":\"omarchyvr-roommap-export\",\"version\":1,\"frame\":\"room, y up, metres\",\n";
    o += "\"meta\":{\"name\":" + esc(name) + ",\"device_uid\":" + esc(map.device_uid) + ",\"sessions\":" +
         std::to_string(map.sessions) + ",\"keyframes\":" + std::to_string(map.keyframes.size()) +
         ",\"raw_observations\":" + std::to_string(raw) + ",\"points\":" + std::to_string(pts.size()) +
         ",\"merge_cell_m\":" + num(CELL);
    if (meta_toml) {
        for (std::string_view l : rs::lines(*meta_toml)) {
            if (const auto kv = rs::split_once(l, '=')) {
                const std::string_view k = rf::trim(kv->first);
                const std::string_view v = rs::trim_matches(rf::trim(kv->second), "\"");
                if (k == "source" || k == "created") {
                    o += "," + esc(k) + ":" + esc(v);
                }
            }
        }
    }
    o += "},\n";
    if (fl != nullptr) {
        o += "\"floor\":{\"y\":" + num(fl->y) + ",\"source\":" + esc(floor::source_name(fl->source)) +
             ",\"confidence\":" + num(fl->confidence) + ",\"sigma_m\":" + num(fl->sigma_m) + "},\n";
    } else {
        o += "\"floor\":null,\n";
    }
    o += "\"points\":{\"columns\":[\"x\",\"y\",\"z\",\"n\"],\"data\":[";
    for (std::size_t i = 0; i < pts.size(); i++) {
        if (i > 0) {
            o += ',';
        }
        const auto& [x, y, z, n] = pts[i];
        o += num(x) + "," + num(y) + "," + num(z) + "," + std::to_string(n);
    }
    o += "]},\n\"keyframes\":[";
    for (std::size_t i = 0; i < map.keyframes.size(); i++) {
        if (i > 0) {
            o += ',';
        }
        const auto& k = map.keyframes[i];
        const auto q = cam::slam::quat_of(k.pose.r);
        o += "\n{\"id\":" + std::to_string(k.id) + ",\"session\":" + std::to_string(k.session) + ",\"p\":[" +
             num(k.pose.t[0]) + "," + num(k.pose.t[1]) + "," + num(k.pose.t[2]) + "],\"q\":[" + num(q[0]) + "," +
             num(q[1]) + "," + num(q[2]) + "," + num(q[3]) + "],\"points\":" + std::to_string(k.points.size()) + "}";
    }
    o += "\n],\n\"anchors\":[";
    for (std::size_t i = 0; i < map.anchors.size(); i++) {
        if (i > 0) {
            o += ',';
        }
        const auto& a = map.anchors[i];
        o += "{\"name\":" + esc(a.name) + ",\"p\":[" + num(a.p[0]) + "," + num(a.p[1]) + "," + num(a.p[2]) + "],\"q\":[" +
             num(a.q[0]) + "," + num(a.q[1]) + "," + num(a.q[2]) + "," + num(a.q[3]) + "]}";
    }
    o += "]\n}\n";
    return o;
}

std::string html_page(const std::string& json, const std::string& name) {
    const std::string j = replace_all(json, "</", "<\\/");
    const std::string title = replace_all(std::string(rs::trim_matches(esc(name), "\"")), "\\u003c", "&lt;");
    return replace_all(replace_all(std::string(viewer_html()), "__TITLE__", title), "/*__DATA__*/null", j);
}

} // namespace pibridge::roommap::exportmap
