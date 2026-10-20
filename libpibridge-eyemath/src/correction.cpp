// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/correction.rs (MIT OR Apache-2.0).
#include "pibridge/eyemath/correction.h"
#include "pibridge_env.h"

#include <cmath>
#include <cstdlib>
#include <format>
#include <vector>

#include "num.h"

namespace pibridge::eyemath {

namespace fs = std::filesystem;
using detail::fmt_fixed;

std::array<double, 6> features(double u, double v) { return {1.0, u, v, u * u, u * v, v * v}; }

std::optional<Tangent> tangent_of(const Vec3& d) {
    if (d[2] < -1e-9 && std::isfinite(d[0]) && std::isfinite(d[1]) && std::isfinite(d[2])) {
        return Tangent{d[0] / -d[2], d[1] / -d[2]};
    }
    return std::nullopt;
}

Vec3 dir_of(double u, double v) {
    const double n = std::sqrt(u * u + v * v + 1.0);
    return {u / n, v / n, -1.0 / n};
}

std::optional<fs::path> home_dir() {
    if (const char* h = std::getenv("HOME")) {
        return fs::path(h);
    }
    return std::nullopt;
}

Correction Correction::identity() { return Correction{}; }

bool Correction::is_identity() const {
    const auto id = identity();
    return cx == id.cx && cy == id.cy;
}

Tangent Correction::map_tangent(double u, double v) const {
    const auto f = features(u, v);
    const auto dot = [&](const std::array<double, 6>& c) {
        double s = 0.0;
        for (std::size_t i = 0; i < 6; i++) {
            s += c[i] * f[i];
        }
        return s;
    };
    return {dot(cx), dot(cy)};
}

Vec3 Correction::apply(const Vec3& d) const {
    if (const auto t = tangent_of(d); t && std::fabs(t->u) <= MAX_TANGENT && std::fabs(t->v) <= MAX_TANGENT) {
        const auto m = map_tangent(t->u, t->v);
        return dir_of(m.u, m.v);
    }
    return d;
}

Vec3f Correction::apply_f32(const Vec3f& d) const {
    const auto o = apply({double{d[0]}, double{d[1]}, double{d[2]}});
    return {static_cast<float>(o[0]), static_cast<float>(o[1]), static_cast<float>(o[2])};
}

void Correction::apply_sample(Sample& s) const {
    if (s.gaze) {
        s.gaze->direction = apply_f32(s.gaze->direction);
    }
    for (auto& e : s.eyes) {
        if (e.direction_valid) {
            e.direction = apply_f32(e.direction);
        }
    }
}

namespace {

std::string arr6(const std::array<double, 6>& c) {
    std::string s = "[";
    for (std::size_t i = 0; i < c.size(); i++) {
        if (i > 0) {
            s += ", ";
        }
        s += fmt_fixed(c[i], 9);
    }
    return s + "]";
}

const char* b01(bool b) { return b ? "1" : "0"; }

/// The default files under `home`, preferred first: PiBridge's folder, the folder of PiBridge's earlier name
/// (pimax-linux), then omarchyvr's.
std::array<fs::path, 3> candidates(const fs::path& home, const char* name) {
    return {home / ".config/pibridge" / name, home / ".config/pimax-linux" / name, home / ".config/omarchyvr" / name};
}

} // namespace

std::string Correction::to_toml(std::span<const std::string> notes) const {
    std::string s = "# pimax eye gaze correction (host side; the eye chip's calibration is not touched)\n";
    for (const auto& n : notes) {
        s += "# " + n + "\n";
    }
    s += "# u' = x . (1, u, v, u*u, u*v, v*v), v' = y . (...) with (u, v) = (x / -z, y / -z) of the head-frame gaze\n";
    s += std::format("model = \"{}\"\n", model);
    s += std::format("x = {}\ny = {}\n", arr6(cx), arr6(cy));
    if (flips) {
        const auto& f = *flips;
        s += std::format("flip = \"{},{},{}\"\n", b01(f[0]), b01(f[1]), b01(f[2]));
    }
    return s;
}

std::expected<Correction, Error> Correction::parse(std::string_view text) {
    std::optional<std::string> model;
    std::optional<std::array<double, 6>> cx;
    std::optional<std::array<double, 6>> cy;
    std::optional<std::array<bool, 3>> flips;
    const auto all = detail::lines(text);
    for (std::size_t n = 0; n < all.size(); n++) {
        const auto raw = all[n];
        const auto line = detail::trim(raw.substr(0, raw.find('#')));
        if (line.empty()) {
            continue;
        }
        const auto eq = line.find('=');
        if (eq == std::string_view::npos) {
            return std::unexpected(Error{std::format("line {}: expected key = value", n + 1)});
        }
        const auto k = detail::trim(line.substr(0, eq));
        const auto v = detail::trim(line.substr(eq + 1));
        const auto nums = [&](std::string_view s) -> std::expected<std::array<double, 6>, Error> {
            if (s.size() < 2 || s.front() != '[' || s.back() != ']') {
                return std::unexpected(Error{std::format("line {}: {} must be [a, b, ...]", n + 1, k)});
            }
            auto inner = s.substr(1, s.size() - 2);
            std::vector<double> vals;
            while (true) {
                const auto comma = inner.find(',');
                const auto p = detail::trim(inner.substr(0, comma));
                if (!p.empty()) {
                    auto x = detail::parse_f64(p);
                    if (!x) {
                        return std::unexpected(Error{std::format("line {}: {}: {}", n + 1, k, x.error())});
                    }
                    vals.push_back(*x);
                }
                if (comma == std::string_view::npos) {
                    break;
                }
                inner.remove_prefix(comma + 1);
            }
            bool finite = true;
            for (double x : vals) {
                finite = finite && std::isfinite(x);
            }
            if (!(vals.size() == 3 || vals.size() == 6) || !finite) {
                return std::unexpected(Error{std::format("line {}: {} needs 3 or 6 finite numbers", n + 1, k)});
            }
            std::array<double, 6> o{};
            for (std::size_t i = 0; i < vals.size(); i++) {
                o[i] = vals[i];
            }
            return o;
        };
        if (k == "model") {
            model = detail::unquote(v);
        } else if (k == "x" || k == "y") {
            auto a = nums(v);
            if (!a) {
                return std::unexpected(a.error());
            }
            (k == "x" ? cx : cy) = *a;
        } else if (k == "flip") {
            const auto u = detail::unquote(v);
            std::vector<bool> p;
            std::string_view rest = u;
            while (true) {
                const auto comma = rest.find(',');
                const auto t = detail::trim(rest.substr(0, comma));
                p.push_back(t == "1" || t == "true");
                if (comma == std::string_view::npos) {
                    break;
                }
                rest.remove_prefix(comma + 1);
            }
            if (p.size() == 3) {
                flips = std::array<bool, 3>{p[0], p[1], p[2]};
            }
        }
    }
    if (!cx) {
        return std::unexpected(Error{"missing x = [...]"});
    }
    if (!cy) {
        return std::unexpected(Error{"missing y = [...]"});
    }
    Correction c;
    c.model = model.value_or("affine");
    c.cx = *cx;
    c.cy = *cy;
    c.flips = flips;
    return c;
}

std::expected<Correction, Error> Correction::load(const fs::path& path) {
    const auto t = detail::read_text(path.c_str());
    if (!t) {
        return std::unexpected(Error{std::format("{}: {}", path.string(), t.error())});
    }
    auto c = parse(*t);
    if (!c) {
        return std::unexpected(Error{std::format("{}: {}", path.string(), c.error().message)});
    }
    return c;
}

std::optional<fs::path> Correction::default_path() {
    const auto h = home_dir();
    if (!h) {
        return std::nullopt;
    }
    return candidates(*h, "eye-correction.toml")[0];
}

std::optional<fs::path> Correction::find_default(const std::optional<fs::path>& home) {
    if (!home) {
        return std::nullopt;
    }
    for (const auto& p : candidates(*home, "eye-correction.toml")) {
        std::error_code ec;
        if (fs::exists(p, ec)) {
            return p;
        }
    }
    return std::nullopt;
}

std::expected<std::optional<Correction>, Error> Correction::select(std::optional<std::string_view> var,
                                                                   const std::optional<fs::path>& home) {
    const auto v = var ? detail::trim(*var) : std::string_view{};
    if (var && (v == "none" || v == "off" || v == "0")) {
        return std::optional<Correction>{};
    }
    if (var && !v.empty()) {
        auto c = load(fs::path(v));
        if (!c) {
            return std::unexpected(c.error());
        }
        return std::optional<Correction>{std::move(*c)};
    }
    if (const auto p = find_default(home)) {
        auto c = load(*p);
        if (!c) {
            return std::unexpected(c.error());
        }
        return std::optional<Correction>{std::move(*c)};
    }
    return std::optional<Correction>{};
}

std::expected<std::optional<Correction>, Error> Correction::from_env() {
    const char* v = pibridge::env("EYE_CORRECTION");
    return select(v ? std::optional<std::string_view>(v) : std::nullopt, home_dir());
}

std::optional<std::string> Correction::flip_warning(const FrameConv& conv) const {
    if (!flips) {
        return std::nullopt;
    }
    const auto& f = *flips;
    const std::array<bool, 3> now{conv.flip_x, conv.flip_y, conv.flip_z};
    if (f == now) {
        return std::nullopt;
    }
    return std::format("eye correction was measured with PIBRIDGE_EYE_FLIP_X/Y/Z = {}/{}/{} but they are {}/{}/{} now: "
                       "the correction is probably wrong, measure again",
                       b01(f[0]), b01(f[1]), b01(f[2]), b01(now[0]), b01(now[1]), b01(now[2]));
}

} // namespace pibridge::eyemath
