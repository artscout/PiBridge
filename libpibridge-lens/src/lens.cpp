// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-lens/src/lib.rs (MIT OR Apache-2.0).
#include "pibridge/lens/lens.h"

#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/lens/fmt.h"
#include "pibridge/lens/grid.h"
#include "pibridge/lens/png.h"

#include <cerrno>
#include <fcntl.h>
#include <format>
#include <memory>
#include <sys/stat.h>
#include <unistd.h>

namespace pibridge::lens {

namespace rf = cam::rfmt;
using fmt::display_f32;

namespace {

Message info(std::string text) {
    return Message{Level::Info, std::move(text)};
}

Message warn(std::string text) {
    return Message{Level::Warn, std::move(text)};
}

bool is_dir(const std::string& p) {
    struct stat st {};
    return ::stat(p.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

// PathBuf equality: component-wise (doubled separators, inner "." and a trailing separator do not matter).
std::string components_key(const std::string& p) {
    std::string out = !p.empty() && p.front() == '/' ? "/" : "";
    std::size_t i = 0;
    bool first = true;
    while (i <= p.size()) {
        const std::size_t s = p.find('/', i);
        const std::size_t e = s == std::string::npos ? p.size() : s;
        const std::string_view c(p.data() + i, e - i);
        if (!c.empty() && !(c == "." && !(first && out.empty() && i == 0))) {
            out += c;
            out += '\x01';
        }
        first = false;
        if (s == std::string::npos) {
            break;
        }
        i = s + 1;
    }
    return out;
}

std::string join(const std::string& base, const std::string& rel) {
    if (!rel.empty() && rel.front() == '/') {
        return rel;
    }
    return cam::path_join(base, rel);
}

// std::fs::write of text.
std::string write_file(const std::string& path, std::string_view data) {
    const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0666);
    if (fd < 0) {
        return cam::IoError::last_os().text();
    }
    std::size_t done = 0;
    while (done < data.size()) {
        const ssize_t n = ::write(fd, data.data() + done, data.size() - done);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            const auto e = cam::IoError::last_os().text();
            ::close(fd);
            return e;
        }
        if (n == 0) {
            ::close(fd);
            return "failed to write whole buffer";
        }
        done += static_cast<std::size_t>(n);
    }
    ::close(fd);
    return "";
}

// Load the LUT files named by the config (`lut = FILE`). On success the eye's FOV becomes the LUT's FOV port plus
// `lut_margin` and its centre is reset to the panel middle (the LUT carries the real optical axis). A missing or
// invalid file is a warning and the eye keeps the radial model.
std::array<std::shared_ptr<const Lut>, 2> attach_luts(LensConfig& cfg, const std::optional<std::string>& conf_dir,
                                                      std::vector<Message>& messages) {
    std::array<std::shared_ptr<const Lut>, 2> out{};
    struct Cached {
        std::string path;
        std::string key;
        std::shared_ptr<const Lut> lut;
        std::string err;
    };
    std::vector<Cached> cache;
    for (std::size_t e = 0; e < 2; e++) {
        if (!cfg.eye[e].lut) {
            continue;
        }
        const std::string name = *cfg.eye[e].lut;
        std::string p;
        if (name.starts_with("~/")) {
            p = join(cam::env_os("HOME").value_or(""), name.substr(2));
        } else {
            p = name;
        }
        if (p.empty() || p.front() != '/') {
            if (conf_dir) {
                p = join(*conf_dir, p);
            }
        }
        const std::string key = components_key(p);
        std::size_t idx = cache.size();
        for (std::size_t i = 0; i < cache.size(); i++) {
            if (cache[i].key == key) {
                idx = i;
                break;
            }
        }
        if (idx == cache.size()) {
            auto l = Lut::load(p);
            if (l) {
                cache.push_back({p, key, std::make_shared<const Lut>(std::move(*l)), ""});
            } else {
                cache.push_back({p, key, nullptr, l.error()});
            }
        }
        const Cached& c = cache[idx];
        if (c.lut) {
            EyeConfig& ec = cfg.eye[e];
            const float m = ec.lut_margin;
            const auto f = c.lut->fov_deg(e);
            for (std::size_t i = 0; i < 4; i++) {
                ec.fov[i] = fmt::clampf(f[i] + m, 0.5f, 89.5f);
            }
            ec.cx = 0.0f;
            ec.cy = 0.0f;
            messages.push_back(info(std::format(
                "pimax lens: eye {} LUT {} (serial {}, grid {}x{}, {}) fov L{} R{} U{} D{} (margin {})", e, p,
                c.lut->serial, c.lut->n, c.lut->n, interp_name(ec.lut_interp), rf::fixed(ec.fov[0], 2),
                rf::fixed(ec.fov[1], 2), rf::fixed(ec.fov[2], 2), rf::fixed(ec.fov[3], 2), display_f32(m))));
            out[e] = c.lut;
        } else {
            messages.push_back(warn(std::format(
                "pimax lens: eye {}: cannot use LUT, falling back to the radial model: {}", e, c.err)));
        }
    }
    return out;
}

// The common end of load_file and load_reference: the environment overrides, the LUTs (relative to `lut_dir`), the
// per-eye log lines.
Loaded finish(LensConfig cfg, std::vector<Message> messages, const Env& env, std::string path, bool file_found,
              bool reference, const std::optional<std::string>& lut_dir) {
    if (forced_off(env)) {
        cfg.enabled = false;
    }
    if (const auto c = cant_override(env)) {
        if (c->ok) {
            cfg.cant_deg = c->value;
        } else {
            messages.push_back(warn(std::format(
                "pimax lens: bad PIMAX_CANT_DEG '{}' (number within +-{} expected), ignored", c->raw,
                display_f32(MAX_CANT_DEG))));
        }
    }
    std::array<std::shared_ptr<const Lut>, 2> luts{};
    if (cfg.enabled) {
        luts = attach_luts(cfg, lut_dir, messages);
    }
    for (std::size_t e = 0; e < 2; e++) {
        if (!luts[e]) {
            for (auto& line : eye_log_lines(cfg, e)) {
                messages.push_back(info(std::move(line)));
            }
        }
    }
    const std::string cd = rf::fixed(cfg.cant_deg, 3);
    messages.push_back(
        info(std::format("pimax lens: cant {} deg per eye (left view yawed +{}, right view -{})", cd, cd, cd)));
    return Loaded{Lens(std::move(cfg)).with_luts(std::move(luts)), std::move(path), file_found, reference,
                  std::move(messages)};
}

// load_file with the config the file's keys are applied on (the baseline).
Loaded load_file_on(LensConfig cfg, const Env& env, std::string path) {
    std::vector<Message> messages;
    bool file_found = false;
    if (const auto bytes = cam::read_file(path)) {
        file_found = true;
        messages.push_back(info(std::format("pimax lens: reading {}", rf::utf8_lossy(path))));
        const std::string text = rf::utf8_lossy(*bytes);
        for (const auto& w : parse_into(cfg, text, rf::utf8_lossy(path))) {
            messages.push_back(warn(std::format("pimax lens: {}", w.message)));
        }
    } else {
        messages.push_back(
            info(std::format("pimax lens: no config at {}, using built-in defaults", rf::utf8_lossy(path))));
    }
    const auto dir = detail::path_parent(path);
    return finish(std::move(cfg), std::move(messages), env, std::move(path), file_found, false, dir);
}

} // namespace

namespace detail {

std::optional<std::string> path_parent(const std::string& p) {
    std::string s = p;
    // drop trailing separators and "." components
    while (true) {
        while (s.size() > 1 && s.back() == '/') {
            s.pop_back();
        }
        if (s.size() >= 2 && s.ends_with("/.")) {
            s.resize(s.size() - 2);
            continue;
        }
        break;
    }
    if (s.empty() || s == "/" || s == ".") {
        // "." has the parent "" in Rust; "" and "/" have none
        if (s == ".") {
            return std::string();
        }
        return std::nullopt;
    }
    const std::size_t slash = s.rfind('/');
    if (slash == std::string::npos) {
        return std::string();
    }
    std::string parent = s.substr(0, slash);
    while (parent.size() > 1 && parent.back() == '/') {
        parent.pop_back();
    }
    if (parent.empty()) {
        return std::string("/");
    }
    return parent;
}

std::string create_dir_all(const std::string& path) {
    if (path.empty()) {
        return "";
    }
    if (::mkdir(path.c_str(), 0777) == 0) {
        return "";
    }
    const int e = errno;
    if (e != ENOENT) {
        if (is_dir(path)) {
            return "";
        }
        return cam::IoError::from_errno(e).text();
    }
    const auto parent = path_parent(path);
    if (!parent) {
        return "failed to create whole tree";
    }
    if (auto r = create_dir_all(*parent); !r.empty()) {
        return r;
    }
    if (::mkdir(path.c_str(), 0777) == 0) {
        return "";
    }
    const int e2 = errno;
    if (is_dir(path)) {
        return "";
    }
    return cam::IoError::from_errno(e2).text();
}

} // namespace detail

std::vector<std::string> eye_log_lines(const LensConfig& cfg, std::size_t e) {
    const EyeConfig& l = cfg.eye[e];
    const auto f4 = [](float x) { return rf::fixed(x, 4); };
    std::vector<std::string> v;
    v.push_back(std::format("pimax lens: eye {} {} fov L{} R{} U{} D{} centre {},{} base {} scale {}", e,
                            cfg.enabled ? "distortion on" : "distortion OFF", rf::fixed(l.fov[0], 1),
                            rf::fixed(l.fov[1], 1), rf::fixed(l.fov[2], 1), rf::fixed(l.fov[3], 1), f4(l.cx), f4(l.cy),
                            f4(l.base[1]), rf::fixed(l.scale, 3)));
    v.push_back(std::format("pimax lens: eye {} k_r {} {} {}  k_g {} {} {}  k_b {} {} {}  ca {} {} {}", e,
                            f4(l.k[0][0]), f4(l.k[0][1]), f4(l.k[0][2]), f4(l.k[1][0]), f4(l.k[1][1]), f4(l.k[1][2]),
                            f4(l.k[2][0]), f4(l.k[2][1]), f4(l.k[2][2]), f4(l.ca[0]), f4(l.ca[1]), f4(l.ca[2])));
    bool k45 = false;
    for (const auto& k : l.k) {
        k45 = k45 || k[3] != 0.0f || k[4] != 0.0f;
    }
    if (k45) {
        v.push_back(std::format("pimax lens: eye {} k4 k5: r {} {}  g {} {}  b {} {}", e, f4(l.k[0][3]),
                                f4(l.k[0][4]), f4(l.k[1][3]), f4(l.k[1][4]), f4(l.k[2][3]), f4(l.k[2][4])));
    }
    if (l.base[0] != l.base[1] || l.base[2] != l.base[1]) {
        v.push_back(std::format("pimax lens: eye {} base r {} g {} b {}", e, rf::fixed(l.base[0], 5),
                                rf::fixed(l.base[1], 5), rf::fixed(l.base[2], 5)));
    }
    bool shifted = false;
    for (const auto& s : l.shift) {
        shifted = shifted || s[0] != 0.0f || s[1] != 0.0f;
    }
    if (shifted) {
        const auto p4 = [](float x) { return rf::fixed(x, 4, true); };
        v.push_back(std::format("pimax lens: eye {} shift r {},{} g {},{} b {},{}", e, p4(l.shift[0][0]),
                                p4(l.shift[0][1]), p4(l.shift[1][0]), p4(l.shift[1][1]), p4(l.shift[2][0]),
                                p4(l.shift[2][1])));
    }
    if (l.fit_range > 0.0f) {
        v.push_back(std::format("pimax lens: eye {} fit_range {} (curve continues as a line beyond it)", e,
                                rf::fixed(l.fit_range, 3)));
    }
    if (l.has_mask()) {
        std::string s = std::format("pimax lens: eye {} visibility mask:", e);
        bool any = false;
        for (const float b : l.blackout) {
            any = any || b > 0.0f;
        }
        if (any) {
            s += std::format(" blackout L{} R{} T{} B{}", display_f32(l.blackout[0]), display_f32(l.blackout[1]),
                             display_f32(l.blackout[2]), display_f32(l.blackout[3]));
        }
        switch (l.mask.shape) {
        case MaskShape::Off:
            break;
        case MaskShape::Ellipse:
            s += std::format(" ellipse {} {}", display_f32(l.mask.a), display_f32(l.mask.b));
            break;
        case MaskShape::Rect:
            s += std::format(" rect {} {}", display_f32(l.mask.a), display_f32(l.mask.b));
            break;
        case MaskShape::Superellipse:
            s += std::format(" superellipse {} {} {}", display_f32(l.mask.a), display_f32(l.mask.b),
                             display_f32(l.mask.n));
            break;
        }
        s += std::format(" feather {}", display_f32(l.mask_feather));
        v.push_back(std::move(s));
    }
    return v;
}

Loaded load(float aspect, const Env& env) {
    std::string path = config_path(env);
    const auto lc = env("PIMAX_LENS_CONF");
    const bool explicit_conf = lc && !lc->empty();
    struct stat st {};
    if (!explicit_conf && reference_allowed(env) && ::stat(path.c_str(), &st) != 0) {
        return load_reference(aspect, env, std::move(path));
    }
    // A user file that sets only some keys inherits the rest from the shipped Windows (PVR) fit, not from the old
    // angle-table defaults (2026-10-17).
    LensConfig base = reference_allowed(env) ? reference_config(aspect).first : LensConfig::defaults(aspect);
    return load_file_on(std::move(base), env, std::move(path));
}

std::pair<LensConfig, std::vector<Warning>> reference_config(float aspect) {
    return parse(reference_lens_conf(), aspect);
}

Loaded load_reference(float aspect, const Env& env, std::string path) {
    LensConfig cfg = LensConfig::defaults(aspect);
    std::vector<Message> messages;
    messages.push_back(info(std::format("pimax lens: no config at {}, using the shipped Pimax Dream Air lens profile "
                                        "(reference; the app's lens tools write your own)",
                                        rf::utf8_lossy(path))));
    for (const auto& w : parse_into(cfg, reference_lens_conf(), "reference lens.conf")) {
        messages.push_back(warn(std::format("pimax lens: {}", w.message)));
    }
    return finish(std::move(cfg), std::move(messages), env, std::move(path), false, true, std::nullopt);
}

Loaded load_file(float aspect, const Env& env, std::string path) {
    return load_file_on(LensConfig::defaults(aspect), env, std::move(path));
}

std::vector<Message> write_debug_grid(const Lens& lens, const std::string& dir, std::uint32_t div) {
    std::vector<Message> out;
    if (const auto e = detail::create_dir_all(dir); !e.empty()) {
        out.push_back(warn(std::format("pimax lens: debug grid: cannot create {}: {}", rf::utf8_lossy(dir), e)));
        return out;
    }
    const auto opt = grid::GridOptions::for_panel(div);
    std::string summary;
    for (std::size_t view = 0; view < 2; view++) {
        const auto img = grid::render(lens, view, opt);
        const std::string path = join(dir, std::format("pimax_lens_grid_{}.png", view == 0 ? "left" : "right"));
        if (const auto r = png::write_rgb8(path, opt.width, opt.height, img)) {
            out.push_back(info(std::format("pimax lens: debug grid written to {}", rf::utf8_lossy(path))));
        } else {
            out.push_back(
                warn(std::format("pimax lens: debug grid: cannot write {}: {}", rf::utf8_lossy(path), r.error())));
        }
        summary += grid::summary(lens, view);
    }
    const std::string path = join(dir, "pimax_lens_summary.txt");
    if (const auto e = write_file(path, summary); e.empty()) {
        out.push_back(info(std::format("pimax lens: debug grid summary written to {}", rf::utf8_lossy(path))));
    } else {
        out.push_back(warn(std::format("pimax lens: debug grid: cannot write {}: {}", rf::utf8_lossy(path), e)));
    }
    return out;
}

} // namespace pibridge::lens
