// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-lens/src/config.rs (MIT OR Apache-2.0).
#include "pibridge/lens/config.h"

#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"

#include <cmath>
#include <cstdlib>
#include <format>
#include <locale.h>

namespace pibridge::lens {

const char* interp_name(Interp i) {
    return i == Interp::Bilinear ? "Bilinear" : "Bicubic";
}

const char* mask_shape_name(MaskShape s) {
    switch (s) {
    case MaskShape::Off:
        return "Off";
    case MaskShape::Ellipse:
        return "Ellipse";
    case MaskShape::Rect:
        return "Rect";
    case MaskShape::Superellipse:
        return "Superellipse";
    }
    return "Off";
}

EyeConfig EyeConfig::defaults(std::size_t eye) {
    const Poly k = {0.60779f, -0.72489f, 0.58858f, 0.0f, 0.0f};
    EyeConfig e;
    e.fov = {50.0f, 50.0f, 44.5f, 44.5f};
    e.cx = eye == 0 ? 0.0271f : -0.0271f;
    e.cy = 0.009f;
    e.base = {0.65307f, 0.65307f, 0.65307f};
    e.scale = 1.0f;
    e.k = {k, k, k};
    e.ca = {1.0045f, 1.0f, 0.995f};
    e.shift = {};
    e.fit_range = 0.0f;
    e.blackout = {};
    e.mask = Mask::off();
    e.mask_feather = 0.03f;
    e.lut = std::nullopt;
    e.lut_margin = 0.0f;
    e.lut_interp = Interp::Bicubic;
    return e;
}

bool EyeConfig::has_mask() const {
    if (mask.shape != MaskShape::Off) {
        return true;
    }
    for (const float b : blackout) {
        if (b > 0.0f) {
            return true;
        }
    }
    return false;
}

LensConfig LensConfig::defaults(float aspect) {
    LensConfig c;
    c.enabled = true;
    c.aspect = aspect;
    c.cant_deg = 0.0f;
    c.eye = {EyeConfig::defaults(0), EyeConfig::defaults(1)};
    return c;
}

namespace {

bool is_space(char c) {
    return c == ' ' || c == '\t' || c == '\n' || c == '\x0b' || c == '\x0c' || c == '\r';
}

bool is_digit(char c) {
    return c >= '0' && c <= '9';
}

std::string_view trim_space(std::string_view s) {
    while (!s.empty() && is_space(s.front())) {
        s.remove_prefix(1);
    }
    while (!s.empty() && is_space(s.back())) {
        s.remove_suffix(1);
    }
    return s;
}

std::string ascii_lower(std::string_view s) {
    std::string o(s);
    for (char& c : o) {
        if (c >= 'A' && c <= 'Z') {
            c = static_cast<char>(c - 'A' + 'a');
        }
    }
    return o;
}

// Rust `str::parse::<f32>` of a string already known to be [+-]digits[.digits][e[+-]digits] (correctly rounded,
// like glibc's strtof in the C locale).
float parse_decimal_f32(std::string_view s) {
    static const locale_t c_locale = ::newlocale(LC_ALL_MASK, "C", static_cast<locale_t>(nullptr));
    const std::string t(s);
    return ::strtof_l(t.c_str(), nullptr, c_locale);
}

std::optional<Mask> parse_mask(std::string_view val) {
    val = cam::rfmt::trim(val);
    std::string_view word = val;
    std::string_view rest;
    for (std::size_t p = 0; p < val.size(); p++) {
        if (is_space(val[p]) || val[p] == ',') {
            word = val.substr(0, p);
            rest = val.substr(p);
            break;
        }
    }
    const std::string w = ascii_lower(word);
    if (w == "off" || w == "none" || w == "0") {
        return Mask::off();
    }
    if (w == "ellipse" || w == "circle" || w == "rect" || w == "box") {
        const auto f = detail::floats(rest, 1, 2);
        if (!f) {
            return std::nullopt;
        }
        const float a = (*f)[0];
        const float b = f->size() > 1 ? (*f)[1] : (*f)[0];
        if (!(a > 0.0f && b > 0.0f)) {
            return std::nullopt;
        }
        const bool ellipse = w == "ellipse" || w == "circle";
        return Mask{ellipse ? MaskShape::Ellipse : MaskShape::Rect, a, b, 2.0f};
    }
    if (w == "superellipse" || w == "round") {
        const auto f = detail::floats(rest, 3, 3);
        if (!f) {
            return std::nullopt;
        }
        if (!((*f)[0] > 0.0f && (*f)[1] > 0.0f && (*f)[2] >= 1.0f)) {
            return std::nullopt;
        }
        return Mask{MaskShape::Superellipse, (*f)[0], (*f)[1], (*f)[2]};
    }
    return std::nullopt;
}

bool in(float x, float lo, float hi) {
    return lo <= x && x <= hi;
}

std::optional<float> one(std::string_view val) {
    const auto f = detail::floats(val, 1, 1);
    if (!f) {
        return std::nullopt;
    }
    return (*f)[0];
}

std::optional<Poly> poly(std::string_view val) {
    const auto f = detail::floats(val, 3, K_TERMS);
    if (!f) {
        return std::nullopt;
    }
    Poly k{};
    for (std::size_t i = 0; i < f->size(); i++) {
        k[i] = (*f)[i];
    }
    return k;
}

// Apply one `key = value` to one eye; false = "bad key/value".
bool apply_eye_key(EyeConfig& e, const std::string& key, std::string_view val) {
    if (key == "fov") {
        const auto f = detail::floats(val, 4, 4);
        if (!f) {
            return false;
        }
        // tan() of 90 degrees is infinite, and a zero width image has no UV map
        for (const float a : *f) {
            if (!in(a, 0.5f, 89.5f)) {
                return false;
            }
        }
        e.fov = {(*f)[0], (*f)[1], (*f)[2], (*f)[3]};
    } else if (key == "center") {
        const auto f = detail::floats(val, 2, 2);
        if (!f) {
            return false;
        }
        for (const float a : *f) {
            if (std::fabs(a) > 2.0f) {
                return false;
            }
        }
        e.cx = (*f)[0];
        e.cy = (*f)[1];
    } else if (key == "base") {
        const auto b = one(val);
        if (!b || !in(*b, 0.01f, 10.0f)) {
            return false;
        }
        e.base = {*b, *b, *b};
    } else if (key == "scale") {
        const auto v = one(val);
        if (!v || !in(*v, 0.05f, 20.0f)) {
            return false;
        }
        e.scale = *v;
    } else if (key == "k") {
        const auto k = poly(val);
        if (!k) {
            return false;
        }
        e.k = {*k, *k, *k};
    } else if (key == "fit_range") {
        const auto r = one(val);
        if (!r || *r < 0.0f) {
            return false;
        }
        e.fit_range = *r;
    } else if (key == "blackout") {
        const auto f = detail::floats(val, 1, 4);
        if (!f) {
            return false;
        }
        std::array<float, 4> b{};
        if (f->size() == 1) {
            b = {(*f)[0], (*f)[0], (*f)[0], (*f)[0]};
        } else if (f->size() == 4) {
            b = {(*f)[0], (*f)[1], (*f)[2], (*f)[3]};
        } else {
            return false;
        }
        for (const float x : b) {
            if (x < 0.0f) {
                return false;
            }
        }
        e.blackout = b;
    } else if (key == "mask") {
        const auto m = parse_mask(val);
        if (!m) {
            return false;
        }
        e.mask = *m;
    } else if (key == "lut") {
        std::string_view v = val;
        while (!v.empty() && (v.front() == '"' || v.front() == '\'')) {
            v.remove_prefix(1);
        }
        while (!v.empty() && (v.back() == '"' || v.back() == '\'')) {
            v.remove_suffix(1);
        }
        if (v.empty()) {
            return false;
        }
        const std::string l = ascii_lower(v);
        if (l == "none" || l == "off") {
            e.lut = std::nullopt;
        } else {
            e.lut = std::string(v);
        }
    } else if (key == "lut_margin") {
        const auto m = one(val);
        if (!m || !in(*m, 0.0f, 10.0f)) {
            return false;
        }
        e.lut_margin = *m;
    } else if (key == "lut_interp") {
        const std::string l = ascii_lower(val);
        if (l == "bilinear" || l == "linear") {
            e.lut_interp = Interp::Bilinear;
        } else if (l == "bicubic" || l == "cubic") {
            e.lut_interp = Interp::Bicubic;
        } else {
            return false;
        }
    } else if (key == "mask_feather") {
        const auto w = one(val);
        if (!w || *w < 0.0f) {
            return false;
        }
        e.mask_feather = *w;
    } else {
        // Keys with a channel suffix: k_r, base_g, ca_b, shift_r ...
        const std::size_t us = key.rfind('_');
        if (us == std::string::npos) {
            return false;
        }
        const std::string_view stem = std::string_view(key).substr(0, us);
        const std::string_view suffix = std::string_view(key).substr(us + 1);
        std::size_t c = 0;
        if (suffix == "r") {
            c = 0;
        } else if (suffix == "g") {
            c = 1;
        } else if (suffix == "b") {
            c = 2;
        } else {
            return false;
        }
        if (stem == "k") {
            const auto k = poly(val);
            if (!k) {
                return false;
            }
            e.k[c] = *k;
        } else if (stem == "base") {
            const auto b = one(val);
            if (!b || !in(*b, 0.01f, 10.0f)) {
                return false;
            }
            e.base[c] = *b;
        } else if (stem == "ca") {
            // a divisor of the panel radius: zero or negative would turn the picture inside out
            const auto v = one(val);
            if (!v || !in(*v, 0.5f, 2.0f)) {
                return false;
            }
            e.ca[c] = *v;
        } else if (stem == "shift") {
            const auto f = detail::floats(val, 2, 2);
            if (!f) {
                return false;
            }
            e.shift[c] = {(*f)[0], (*f)[1]};
        } else {
            return false;
        }
    }
    return true;
}

// Rust `str::lines`.
std::vector<std::string_view> lines(std::string_view text) {
    std::vector<std::string_view> out;
    while (!text.empty()) {
        const std::size_t nl = text.find('\n');
        std::string_view line;
        if (nl == std::string_view::npos) {
            line = text;
            text = {};
        } else {
            line = text.substr(0, nl);
            text.remove_prefix(nl + 1);
            if (!line.empty() && line.back() == '\r') {
                line.remove_suffix(1);
            }
        }
        out.push_back(line);
    }
    return out;
}

} // namespace

namespace detail {

std::optional<std::pair<float, std::size_t>> scan_float(std::string_view s) {
    std::size_t i = 0;
    while (i < s.size() && is_space(s[i])) {
        i++;
    }
    const std::size_t start = i;
    if (i < s.size() && (s[i] == '+' || s[i] == '-')) {
        i++;
    }
    const std::size_t int_start = i;
    while (i < s.size() && is_digit(s[i])) {
        i++;
    }
    std::size_t digits = i - int_start;
    if (i < s.size() && s[i] == '.') {
        i++;
        const std::size_t frac_start = i;
        while (i < s.size() && is_digit(s[i])) {
            i++;
        }
        digits += i - frac_start;
    }
    if (digits == 0) {
        return std::nullopt;
    }
    if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
        std::size_t j = i + 1;
        if (j < s.size() && (s[j] == '+' || s[j] == '-')) {
            j++;
        }
        const std::size_t exp_start = j;
        while (j < s.size() && is_digit(s[j])) {
            j++;
        }
        if (j > exp_start) {
            i = j;
        }
    }
    const float v = parse_decimal_f32(s.substr(start, i - start));
    if (!std::isfinite(v)) {
        return std::nullopt;
    }
    return std::pair{v, i};
}

std::optional<std::vector<float>> floats(std::string_view s, std::size_t min, std::size_t max) {
    std::vector<float> out;
    std::string_view rest = s;
    while (out.size() < max) {
        const auto r = scan_float(rest);
        if (!r) {
            break;
        }
        out.push_back(r->first);
        rest.remove_prefix(r->second);
        while (!rest.empty() && (rest.front() == ',' || is_space(rest.front()))) {
            rest.remove_prefix(1);
        }
    }
    if (out.size() < min) {
        return std::nullopt;
    }
    return out;
}

} // namespace detail

std::vector<Warning> parse_into(LensConfig& cfg, std::string_view text, std::string_view name) {
    std::vector<Warning> warnings;
    int section = 0; // 0 = common, 1 = left, 2 = right
    std::size_t idx = 0;
    for (const std::string_view raw : lines(text)) {
        const std::size_t lineno = ++idx;
        std::string_view line = raw;
        const std::size_t p = raw.find_first_of("#;");
        if (p != std::string_view::npos) {
            line = raw.substr(0, p);
        }
        const std::string_view s = trim_space(line);
        if (s.empty()) {
            continue;
        }
        if (s.front() == '[') {
            if (s.starts_with("[left]")) {
                section = 1;
            } else if (s.starts_with("[right]")) {
                section = 2;
            } else if (s.starts_with("[common]") || s.starts_with("[both]")) {
                section = 0;
            } else {
                warnings.push_back({lineno, std::format("{}:{} unknown section {}", name, lineno, s)});
            }
            continue;
        }
        const std::size_t eq = s.find('=');
        if (eq == std::string_view::npos) {
            warnings.push_back({lineno, std::format("{}:{} expected key = value", name, lineno)});
            continue;
        }
        const std::string key = ascii_lower(trim_space(s.substr(0, eq)));
        const std::string_view val = trim_space(s.substr(eq + 1));
        if (key == "distortion") {
            const std::string v = ascii_lower(val);
            cfg.enabled = !(v == "off" || v == "0" || v == "false" || v == "no");
            continue;
        }
        if (key == "cant_deg") {
            const auto a = one(val);
            if (a && std::fabs(*a) <= MAX_CANT_DEG) {
                cfg.cant_deg = *a;
            } else {
                warnings.push_back({lineno, std::format("{}:{} bad key/value '{} = {}'", name, lineno, key, val)});
            }
            continue;
        }
        bool ok = true;
        for (std::size_t e = 0; e < 2; e++) {
            if (section == 0 || section == static_cast<int>(e) + 1) {
                ok = apply_eye_key(cfg.eye[e], key, val) && ok;
            }
        }
        if (!ok) {
            warnings.push_back({lineno, std::format("{}:{} bad key/value '{} = {}'", name, lineno, key, val)});
        }
    }
    return warnings;
}

std::pair<LensConfig, std::vector<Warning>> parse(std::string_view text, float aspect) {
    LensConfig cfg = LensConfig::defaults(aspect);
    auto w = parse_into(cfg, text, "lens.conf");
    return {std::move(cfg), std::move(w)};
}

std::string config_path(const Env& env) {
    if (const auto p = env("PIMAX_LENS_CONF"); p && !p->empty()) {
        return *p;
    }
    if (const auto x = env("XDG_CONFIG_HOME"); x && !x->empty()) {
        return cam::path_join(*x, "pibridge/lens.conf");
    }
    const std::string home = env("HOME").value_or(".");
    return cam::path_join(home, ".config/pibridge/lens.conf");
}

std::optional<CantOverride> cant_override(const Env& env) {
    const auto v = env("PIMAX_CANT_DEG");
    if (!v || cam::rfmt::trim(*v).empty()) {
        return std::nullopt;
    }
    const auto a = one(cam::rfmt::trim(*v));
    if (a && std::fabs(*a) <= MAX_CANT_DEG) {
        return CantOverride{true, *a, {}};
    }
    return CantOverride{false, 0.0f, *v};
}

bool reference_allowed(const Env& env) {
    const auto v = env("PIMAX_NO_REFERENCE");
    return !(v && !v->empty() && *v != "0");
}

bool forced_off(const Env& env) {
    const auto v = env("PIMAX_LENS_OFF");
    return v && !v->empty() && v->front() != '0';
}

std::optional<std::string> process_env(std::string_view name) {
    return cam::env_utf8(std::string(name).c_str());
}

} // namespace pibridge::lens
