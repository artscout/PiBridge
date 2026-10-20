// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-lens/src/pvrfit.rs (MIT OR Apache-2.0): parse_dump.
#include "pibridge/lens/pvrdump.h"

#include "pibridge/cam/json.h"
#include "pibridge/lens/fmt.h"

#include <format>

namespace pibridge::lens {

namespace {

using cam::json::Value;

const Value* path(const Value* v, std::string_view p) {
    while (v != nullptr) {
        const std::size_t s = p.find('/');
        v = v->get(p.substr(0, s));
        if (s == std::string_view::npos) {
            break;
        }
        p.remove_prefix(s + 1);
    }
    return v;
}

std::optional<double> f64_of(const Value* v) {
    return v != nullptr ? v->as_f64() : std::nullopt;
}

std::optional<std::vector<double>> floats_of(const Value* v) {
    const auto* a = v != nullptr ? v->as_array() : nullptr;
    if (a == nullptr) {
        return std::nullopt;
    }
    std::vector<double> out;
    out.reserve(a->size());
    for (const auto& x : *a) {
        const auto f = x.as_f64();
        if (!f) {
            return std::nullopt;
        }
        out.push_back(*f);
    }
    return out;
}

std::expected<double, std::string> num(const Value& v, std::string_view p) {
    if (const auto f = f64_of(path(&v, p))) {
        return *f;
    }
    return std::unexpected(std::format("dump: missing number '{}'", p));
}

std::expected<std::vector<double>, std::string> floats(const Value& v, std::string_view p) {
    if (auto f = floats_of(path(&v, p))) {
        return std::move(*f);
    }
    return std::unexpected(std::format("dump: missing number array '{}'", p));
}

} // namespace

std::expected<Dump, std::string> parse_dump(std::string_view text) {
    const auto parsed = cam::json::parse(text);
    if (!parsed) {
        return std::unexpected(parsed.error().text());
    }
    const Value& root = *parsed;
    if (root.get("eyes") == nullptr) {
        return std::unexpected("dump: no 'eyes' array; is this a pimax-distortion-*.json?");
    }
    const auto* eyes_v = root.get("eyes")->as_array();
    if (eyes_v == nullptr) {
        return std::unexpected("dump: 'eyes' is not an array");
    }
    if (eyes_v->size() != 2) {
        return std::unexpected(std::format("dump: expected 2 eyes, found {}", eyes_v->size()));
    }
    Dump d;
    for (std::size_t i = 0; i < 2; i++) {
        const Value& ev = (*eyes_v)[i];
        EyeDump e;
        const auto* nm = ev.get("eye") != nullptr ? ev.get("eye")->as_str() : nullptr;
        e.name = nm != nullptr ? *nm : (i == 0 ? "left" : "right");
#define PIBRIDGE_TRY(var, expr)                                                                                          \
    auto var = (expr);                                                                                                \
    if (!var) {                                                                                                       \
        return std::unexpected(var.error());                                                                          \
    }
        PIBRIDGE_TRY(up, num(ev, "fov_tan/up"));
        PIBRIDGE_TRY(down, num(ev, "fov_tan/down"));
        PIBRIDGE_TRY(left, num(ev, "fov_tan/left"));
        PIBRIDGE_TRY(right, num(ev, "fov_tan/right"));
        e.fov_tan = {*up, *down, *left, *right};
        PIBRIDGE_TRY(q, floats(ev, "hmd_to_eye_pose/orientation_xyzw"));
        PIBRIDGE_TRY(p, floats(ev, "hmd_to_eye_pose/position"));
        if (q->size() != 4 || p->size() != 3) {
            return std::unexpected("dump: bad hmd_to_eye_pose");
        }
        // native panel size of this eye
        const Value* disp = root.get("display");
        disp = disp != nullptr ? disp->get(e.name) : nullptr;
        double w = 0.0;
        double h = 0.0;
        if (disp != nullptr) {
            const double dw = f64_of(disp->get("width")).value_or(0.0);
            const double dh = f64_of(disp->get("height")).value_or(0.0);
            const auto* ed = disp->get("eye_display") != nullptr ? disp->get("eye_display")->as_str() : nullptr;
            const std::string mode = ed != nullptr ? *ed : "";
            if (mode == "horz_side_by_side") {
                w = dw / 2.0;
                h = dh;
            } else if (mode == "vert_side_by_side") {
                w = dw;
                h = dh / 2.0;
            } else {
                w = dw;
                h = dh;
            }
        }
        if (w <= 0.0 || h <= 0.0) {
            w = 3840.0;
            h = 3552.0;
        }
        const Value* dist = ev.get("distortion");
        if (dist == nullptr) {
            return std::unexpected("dump: eye without 'distortion'");
        }
        PIBRIDGE_TRY(gn, num(*dist, "grid_n"));
        const auto grid_n = fmt::sat_cast<std::size_t>(*gn);
        PIBRIDGE_TRY(gr, floats(*dist, "grid_r"));
        PIBRIDGE_TRY(gg, floats(*dist, "grid_g"));
        PIBRIDGE_TRY(gb, floats(*dist, "grid_b"));
        e.grid = {std::move(*gr), std::move(*gg), std::move(*gb)};
        for (const auto& g : e.grid) {
            if (g.size() != grid_n * grid_n * 2) {
                return std::unexpected(std::format("dump: grid arrays do not hold {}x{} points", grid_n, grid_n));
            }
        }
        const std::size_t line_n = fmt::sat_cast<std::size_t>(f64_of(dist->get("line_n")).value_or(0.0));
        const std::pair<const char*, std::array<double, 2>> dirs[] = {
            {"+x", {1.0, 0.0}}, {"-x", {-1.0, 0.0}}, {"+y", {0.0, 1.0}}, {"-y", {0.0, -1.0}}};
        for (const auto& [lname, dir] : dirs) {
            const Value* l = dist->get("lines");
            l = l != nullptr ? l->get(lname) : nullptr;
            if (l == nullptr) {
                continue;
            }
            PIBRIDGE_TRY(r, floats(*l, "r"));
            PIBRIDGE_TRY(g, floats(*l, "g"));
            PIBRIDGE_TRY(b, floats(*l, "b"));
            LineData ld{lname, dir, {std::move(*r), std::move(*g), std::move(*b)}};
            for (const auto& o : ld.out) {
                if (o.size() != line_n * 2) {
                    return std::unexpected(std::format("dump: line {} does not hold {} points", lname, line_n));
                }
            }
            e.lines.push_back(std::move(ld));
        }
#undef PIBRIDGE_TRY
        e.quat = {(*q)[0], (*q)[1], (*q)[2], (*q)[3]};
        e.pos = {(*p)[0], (*p)[1], (*p)[2]};
        e.panel = {w, h};
        e.grid_n = grid_n;
        e.line_n = line_n;
        e.failed_calls = fmt::sat_cast<std::int64_t>(f64_of(dist->get("failed_calls")).value_or(0.0));
        d.eyes[i] = std::move(e);
    }
    const auto* serial = path(&root, "hmd/serial");
    d.serial = serial != nullptr && serial->as_str() != nullptr ? *serial->as_str() : "";
    const auto* product = path(&root, "hmd/product_name");
    d.product = product != nullptr && product->as_str() != nullptr ? *product->as_str() : "";
    if (const auto v = f64_of(path(&root, "config/view_rotation_fix_default_-1"))) {
        d.view_rotation_fix = fmt::sat_cast<std::int64_t>(*v);
    }
    return d;
}

} // namespace pibridge::lens
