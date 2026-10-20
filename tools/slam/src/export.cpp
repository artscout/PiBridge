// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-slam/src/export.rs, euroc.rs, imu.rs and omarchyvr-config/src/{paths,devicedata}.rs
// (MIT OR Apache-2.0).
#include "export.h"

#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <dirent.h>
#include <format>
#include <sys/stat.h>

namespace pibridge::slamtool {

namespace rf = pibridge::cam::rfmt;
namespace json = pibridge::cam::json;
using cam::env_os;
using cam::env_utf8;
using cam::is_file;
using cam::path_join;

namespace {

std::string path_err(const std::string& p, const std::string& e) {
    return rf::utf8_lossy(p) + ": " + e;
}

// str::trim_matches(c) for an ASCII c
std::string_view trim_char(std::string_view s, char c) {
    while (!s.empty() && s.front() == c) {
        s.remove_prefix(1);
    }
    while (!s.empty() && s.back() == c) {
        s.remove_suffix(1);
    }
    return s;
}

// Expands a leading ~/ against home.
std::string expand(const std::string& home, const std::string& v) {
    if (v.starts_with("~/")) {
        return path_join(home, v.substr(2));
    }
    if (v == "~") {
        return home;
    }
    return v;
}

std::optional<std::string> file_text(const std::string& home) {
    const std::string p = path_join(home, ".config/omarchyvr/paths.toml");
    struct stat st{};
    if (::stat(p.c_str(), &st) != 0) {
        return std::nullopt;
    }
    auto t = cam::read_to_string(p);
    if (!t) {
        return std::nullopt;
    }
    return *t;
}

std::optional<std::string> env_or_file(const std::string& home, const char* env_key, std::string_view key) {
    if (auto v = env_utf8(env_key); v && !rf::trim(*v).empty()) {
        return std::string(rf::trim(*v));
    }
    const auto t = file_text(home);
    if (!t) {
        return std::nullopt;
    }
    return paths_value(*t, key);
}

std::optional<std::string> key_path(const std::string& home, std::string_view key) {
    const auto t = file_text(home);
    if (!t) {
        return std::nullopt;
    }
    const auto v = paths_value(*t, key);
    if (!v) {
        return std::nullopt;
    }
    return expand(home, *v);
}

} // namespace

std::optional<std::string> paths_value(std::string_view text, std::string_view key) {
    std::size_t pos = 0;
    while (pos < text.size()) {
        const std::size_t nl = text.find('\n', pos);
        std::string_view l = text.substr(pos, nl == std::string_view::npos ? std::string_view::npos : nl - pos);
        pos = nl == std::string_view::npos ? text.size() : nl + 1;
        if (!l.empty() && l.back() == '\r') {
            l.remove_suffix(1);
        }
        const std::size_t hash = l.find('#');
        l = rf::trim(l.substr(0, hash));
        const std::size_t eq = l.find('=');
        if (eq == std::string_view::npos) {
            continue;
        }
        if (rf::trim(l.substr(0, eq)) != key) {
            continue;
        }
        const std::string_view v = rf::trim(trim_char(trim_char(rf::trim(l.substr(eq + 1)), '"'), '\''));
        if (!v.empty()) {
            return std::string(v);
        }
        // a blank value: find_map goes on with the next line
    }
    return std::nullopt;
}

std::string src_dir(const std::string& home) {
    const auto v = env_or_file(home, "PIMAX_SRC_DIR", "src_dir");
    return v ? expand(home, *v) : path_join(home, "src");
}

std::string lib_dir(const std::string& home) {
    return key_path(home, "lib_dir").value_or(path_join(home, ".local/lib/omarchyvr"));
}

std::string legacy_gyro_bias_path(const std::string& home,
                                  const std::function<std::optional<std::string>(std::string_view)>& env) {
    std::string root;
    if (auto x = env("XDG_CACHE_HOME"); x && !x->empty()) {
        root = *x;
    } else {
        root = path_join(home, ".cache");
    }
    return path_join(root, "pimax/gyro-bias");
}

std::vector<std::string> base_config_candidates() {
    std::vector<std::string> v;
    if (const auto h = env_os("HOME")) {
        v.push_back(path_join(src_dir(*h), "basalt/data/euroc/euroc_config.json"));
        v.push_back(path_join(*h, ".local/share/basalt/euroc_config.json"));
        v.push_back(path_join(lib_dir(*h), "basalt/share/basalt/euroc_config.json"));
        v.push_back(path_join(*h, ".local/share/basalt/msdmo_config.json"));
    }
    return v;
}

void to_json(const json::Value& v, std::size_t indent, std::string& out) {
    using T = json::Value::Type;
    switch (v.type) {
    case T::Null: out += "null"; break;
    case T::Bool: out += v.b ? "true" : "false"; break;
    case T::Num: {
        const double x = v.num;
        const double fract = x - std::trunc(x);
        if (fract == 0.0 && std::fabs(x) < 1e15) {
            out += std::format("{}", static_cast<long long>(x));
        } else {
            out += json::num(x);
        }
        break;
    }
    case T::Str: out += json::quote(v.str); break;
    case T::Arr:
        out += '[';
        for (std::size_t i = 0; i < v.arr.size(); i++) {
            if (i > 0) {
                out += ", ";
            }
            to_json(v.arr[i], indent, out);
        }
        out += ']';
        break;
    case T::Obj:
        out += "{\n";
        for (std::size_t i = 0; i < v.obj.size(); i++) {
            out += std::string(indent + 2, ' ');
            out += json::quote(v.obj[i].first);
            out += ": ";
            to_json(v.obj[i].second, indent + 2, out);
            if (i + 1 < v.obj.size()) {
                out += ',';
            }
            out += '\n';
        }
        out += std::string(indent, ' ');
        out += '}';
        break;
    }
}

std::expected<std::string, std::string> vio_config(std::string_view base, std::size_t cams) {
    auto v = json::parse(base);
    if (!v) {
        return std::unexpected(v.error().text());
    }
    if (v->type != json::Value::Type::Obj) {
        return std::unexpected(std::string("base config: not an object"));
    }
    auto it = std::find_if(v->obj.begin(), v->obj.end(), [](const auto& kv) { return kv.first == "value0"; });
    if (it == v->obj.end()) {
        return std::unexpected(std::string("base config: no value0"));
    }
    json::Value& inner = it->second;
    if (inner.type != json::Value::Type::Obj) {
        return std::unexpected(std::string("base config: value0 not an object"));
    }
    for (const auto& [k, x] : OVERRIDES) {
        auto m = std::find_if(inner.obj.begin(), inner.obj.end(), [&](const auto& kv) { return kv.first == k; });
        if (m != inner.obj.end()) {
            m->second = json::Value::number(x);
        } else {
            inner.obj.emplace_back(k, json::Value::number(x));
        }
    }
    if (cams > 2) {
        const std::string k(MULTICAM_NONOVERLAP_KEY);
        auto m = std::find_if(inner.obj.begin(), inner.obj.end(), [&](const auto& kv) { return kv.first == k; });
        if (m != inner.obj.end()) {
            m->second = json::Value::boolean(true);
        } else {
            inner.obj.emplace_back(k, json::Value::boolean(true));
        }
    }
    std::string s;
    to_json(*v, 0, s);
    s += '\n';
    return s;
}

std::string toml(const std::string& calib, const std::string& config, bool show_gui) {
    return std::format("# Basalt SLAM tracker options for the Pimax Dream Air (written by omarchyvr-slam export)\n"
                       "# Use: SLAM_CONFIG=<this file> (Monado), or monado-cli slambatch <euroc dir> <this file> <out dir>\n"
                       "show-gui={}\n"
                       "cam-calib=\"{}\"\n"
                       "config-path=\"{}\"\n"
                       "marg-data=\"\"\n"
                       "print-queue=0\n"
                       "use-double=0\n"
                       "deterministic=0\n"
                       "num-threads=0\n",
                       show_gui ? 1 : 0, rf::utf8_lossy(calib), rf::utf8_lossy(config));
}

std::expected<Exported, std::string> export_files(const std::string& dir0, const cam::DeviceCalib& cal,
                                                  const cam::slam::SlamSetup& setup, const cam::slam::ImuNoise& imu,
                                                  const std::optional<std::string>& base, bool show_gui) {
    if (auto r = cam::create_dir_all(dir0); !r) {
        return std::unexpected(path_err(dir0, r.error().text()));
    }
    // Path::canonicalize
    char* real = ::realpath(dir0.c_str(), nullptr);
    if (real == nullptr) {
        return std::unexpected(path_err(dir0, cam::IoError::last_os().text()));
    }
    const std::string dir(real);
    std::free(real);
    // an explicit --base must exist; otherwise the first candidate on disk, else the built-in EuRoC config
    std::string base_path;
    std::string text;
    if (base) {
        auto t = cam::read_to_string(*base);
        if (!t) {
            return std::unexpected(path_err(*base, t.error().text()));
        }
        base_path = *base;
        text = std::move(*t);
    } else {
        std::optional<std::string> found;
        for (const auto& c : base_config_candidates()) {
            if (is_file(c)) {
                found = c;
                break;
            }
        }
        if (found) {
            auto t = cam::read_to_string(*found);
            if (!t) {
                return std::unexpected(path_err(*found, t.error().text()));
            }
            base_path = *found;
            text = std::move(*t);
        } else {
            base_path = std::string(BUILTIN_BASE_NAME);
            text = std::string(builtin_base());
        }
    }
    auto calib = cam::slam::write_basalt_files(dir, cal, setup, imu, {{"vio_config_base", rf::utf8_lossy(base_path)}});
    if (!calib) {
        return std::unexpected(calib.error());
    }
    const std::string config = path_join(dir, "pimax-basalt-config.json");
    auto vc = vio_config(text, setup.cams.size());
    if (!vc) {
        return std::unexpected(vc.error());
    }
    if (auto r = cam::write_file(config, *vc); !r) {
        return std::unexpected(path_err(config, r.error().text()));
    }
    const std::string toml_p = path_join(dir, "pimax-basalt.toml");
    if (auto r = cam::write_file(toml_p, toml(*calib, config, show_gui)); !r) {
        return std::unexpected(path_err(toml_p, r.error().text()));
    }
    return Exported{*calib, config, toml_p, base_path};
}

/*
 * EuRoC (euroc.rs)
 */

std::expected<std::string, std::string> find_root(const std::string& dir) {
    if (cam::is_dir(path_join(dir, "mav0"))) {
        return dir;
    }
    DIR* d = ::opendir(dir.c_str());
    if (d == nullptr) {
        return std::unexpected(path_err(dir, cam::IoError::last_os().text()));
    }
    std::vector<std::string> cands;
    while (const dirent* e = ::readdir(d)) {
        const std::string n = e->d_name;
        if (n == "." || n == "..") {
            continue;
        }
        const std::string p = path_join(dir, n);
        if (cam::is_dir(path_join(p, "mav0"))) {
            cands.push_back(p);
        }
    }
    ::closedir(d);
    // PathBuf ordering of siblings: by the file name bytes
    std::sort(cands.begin(), cands.end());
    if (cands.empty()) {
        return std::unexpected(rf::utf8_lossy(dir) + ": no mav0 directory (not a EuRoC dataset)");
    }
    return cands.back();
}

namespace {

// text.lines().enumerate().map(|(i, l)| (i + 1, l.trim())).filter(non-empty, not '#')
template <class F>
void data_lines(std::string_view text, F&& f) {
    std::size_t pos = 0;
    std::size_t n = 0;
    while (pos < text.size()) {
        const std::size_t nl = text.find('\n', pos);
        std::string_view l = text.substr(pos, nl == std::string_view::npos ? std::string_view::npos : nl - pos);
        pos = nl == std::string_view::npos ? text.size() : nl + 1;
        if (!l.empty() && l.back() == '\r') {
            l.remove_suffix(1);
        }
        n += 1;
        l = rf::trim(l);
        if (l.empty() || l.front() == '#') {
            continue;
        }
        if (!f(n, l)) {
            return;
        }
    }
}

bool parse_i64(std::string_view s, std::int64_t& out) {
    bool neg = false;
    if (!s.empty() && (s[0] == '+' || s[0] == '-')) {
        neg = s[0] == '-';
        s.remove_prefix(1);
    }
    if (s.empty()) {
        return false;
    }
    std::int64_t v = 0;
    for (const char c : s) {
        if (c < '0' || c > '9') {
            return false;
        }
        const int d = c - '0';
        if (__builtin_mul_overflow(v, 10, &v) ||
            (neg ? __builtin_sub_overflow(v, d, &v) : __builtin_add_overflow(v, d, &v))) {
            return false;
        }
    }
    out = v;
    return true;
}

} // namespace

std::expected<std::vector<ImuRow>, std::string> parse_imu(std::string_view text) {
    std::vector<ImuRow> v;
    std::optional<std::string> error;
    data_lines(text, [&](std::size_t n, std::string_view l) {
        std::vector<std::string_view> f;
        std::size_t s = 0;
        for (;;) {
            const std::size_t c = l.find(',', s);
            f.push_back(rf::trim(l.substr(s, c == std::string_view::npos ? std::string_view::npos : c - s)));
            if (c == std::string_view::npos) {
                break;
            }
            s = c + 1;
        }
        if (f.size() < 7) {
            error = std::format("imu line {}: {} fields, want 7", n, f.size());
            return false;
        }
        ImuRow r;
        if (!parse_i64(f[0], r.ts)) {
            error = std::format("imu line {}: bad timestamp {}", n, rf::debug_str(f[0]));
            return false;
        }
        double x[6];
        for (int k = 0; k < 6; k++) {
            if (!rf::parse_f64(f[static_cast<std::size_t>(k) + 1], x[k])) {
                error = std::format("imu line {}: bad number {}", n, rf::debug_str(f[static_cast<std::size_t>(k) + 1]));
                return false;
            }
        }
        r.gyro = {x[0], x[1], x[2]};
        r.accel = {x[3], x[4], x[5]};
        v.push_back(r);
        return true;
    });
    if (error) {
        return std::unexpected(*error);
    }
    return v;
}

std::expected<std::vector<std::pair<std::int64_t, std::string>>, std::string> parse_cam(std::string_view text) {
    std::vector<std::pair<std::int64_t, std::string>> v;
    std::optional<std::string> error;
    data_lines(text, [&](std::size_t n, std::string_view l) {
        const std::size_t c = l.find(',');
        if (c == std::string_view::npos) {
            error = std::format("cam line {}: want ts,filename", n);
            return false;
        }
        const std::string_view a = l.substr(0, c);
        std::int64_t ts = 0;
        if (!parse_i64(rf::trim(a), ts)) {
            error = std::format("cam line {}: bad timestamp {}", n, rf::debug_str(a));
            return false;
        }
        v.emplace_back(ts, std::string(rf::trim(l.substr(c + 1))));
        return true;
    });
    if (error) {
        return std::unexpected(*error);
    }
    return v;
}

std::expected<Dataset, std::string> Dataset::load(const std::string& dir) {
    auto root = find_root(dir);
    if (!root) {
        return std::unexpected(root.error());
    }
    auto read = [](const std::string& p) -> std::expected<std::string, std::string> {
        auto t = cam::read_to_string(p);
        if (!t) {
            return std::unexpected(path_err(p, t.error().text()));
        }
        return *t;
    };
    auto it = read(path_join(*root, "mav0/imu0/data.csv"));
    if (!it) {
        return std::unexpected(it.error());
    }
    auto imu = parse_imu(*it);
    if (!imu) {
        return std::unexpected(imu.error());
    }
    Dataset ds;
    ds.root = *root;
    ds.imu = std::move(*imu);
    for (int i = 0;; i++) {
        const std::string p = path_join(*root, std::format("mav0/cam{}/data.csv", i));
        if (!is_file(p)) {
            break;
        }
        auto t = read(p);
        if (!t) {
            return std::unexpected(t.error());
        }
        auto c = parse_cam(*t);
        if (!c) {
            return std::unexpected(c.error());
        }
        ds.cams.push_back(std::move(*c));
    }
    return ds;
}

/*
 * IMU statistics (imu.rs)
 */

namespace {

template <class F>
std::pair<Vec3, Vec3> stats(const ImuRow* rows, std::size_t count, F&& f) {
    const double n = static_cast<double>(count);
    Vec3 m{0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < count; i++) {
        const Vec3 v = f(rows[i]);
        for (int k = 0; k < 3; k++) {
            m[k] += v[k] / n;
        }
    }
    Vec3 s{0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < count; i++) {
        const Vec3 v = f(rows[i]);
        for (int k = 0; k < 3; k++) {
            // (v - m).powi(2): LLVM turns powi(x, 2) into x * x
            const double d = v[k] - m[k];
            s[k] += d * d / rf::fmax(n - 1.0, 1.0);
        }
    }
    return {m, {std::sqrt(s[0]), std::sqrt(s[1]), std::sqrt(s[2])}};
}

std::size_t clamp_size(std::size_t x, std::size_t lo, std::size_t hi) {
    return std::max(lo, std::min(x, hi));
}

const auto k_gyro = [](const ImuRow& r) { return r.gyro; };
const auto k_accel = [](const ImuRow& r) { return r.accel; };

} // namespace

std::expected<NoiseEstimate, std::string> estimate(const std::vector<ImuRow>& imu, double window_s, double inflate) {
    if (imu.size() < 100) {
        return std::unexpected(std::string("too few IMU samples"));
    }
    const double span = static_cast<double>(imu.back().ts - imu.front().ts) / 1e9;
    const double rate = static_cast<double>(imu.size() - 1) / span;
    const std::size_t n = clamp_size(static_cast<std::size_t>(rf::f64_to_u64(window_s * rate)), 50, imu.size());
    const std::size_t step = std::max<std::size_t>(n / 8, 1);
    std::optional<std::pair<double, std::size_t>> best;
    for (std::size_t i = 0; i + n <= imu.size(); i += step) {
        const auto [m_unused, s] = stats(imu.data() + i, n, k_gyro);
        (void)m_unused;
        const double score = s[0] * s[0] + s[1] * s[1] + s[2] * s[2];
        if (!best || score < best->first) {
            best = std::pair{score, i};
        }
    }
    if (!best) {
        return std::unexpected(std::string("no window"));
    }
    const ImuRow* w = imu.data() + best->second;
    const auto [gm, gs] = stats(w, n, k_gyro);
    const auto [am, as] = stats(w, n, k_accel);
    if (gs[0] > 0.02 || gs[1] > 0.02 || gs[2] > 0.02) {
        return std::unexpected(std::format(
            "no still segment: the quietest {} s still has gyro std [{}, {}, {}] rad/s (put the headset down for a "
            "few seconds)",
            rf::fixed(window_s, 1), rf::fixed(gs[0], 4), rf::fixed(gs[1], 4), rf::fixed(gs[2], 4)));
    }
    const cam::slam::ImuNoise d;
    const double sq = std::sqrt(rate);
    NoiseEstimate e;
    e.noise.accel_noise_std = {as[0] / sq * inflate, as[1] / sq * inflate, as[2] / sq * inflate};
    e.noise.gyro_noise_std = {gs[0] / sq * inflate, gs[1] / sq * inflate, gs[2] / sq * inflate};
    e.noise.accel_bias_std = d.accel_bias_std;
    e.noise.gyro_bias_std = d.gyro_bias_std;
    e.noise.rate_hz = std::round(rate / 100.0) * 100.0;
    e.noise.source.clear();
    e.window = {w[0].ts, w[n - 1].ts};
    e.rate_hz = rate;
    e.gyro_std = gs;
    e.accel_std = as;
    e.gyro_mean = gm;
    e.accel_mean = am;
    return e;
}

std::optional<std::pair<Vec3, std::int64_t>> still_gyro_bias(const std::vector<ImuRow>& imu, double window_s) {
    if (imu.size() < 100) {
        return std::nullopt;
    }
    const double span = static_cast<double>(imu.back().ts - imu.front().ts) / 1e9;
    const double rate = static_cast<double>(imu.size() - 1) / span;
    const std::size_t n = clamp_size(static_cast<std::size_t>(rf::f64_to_u64(window_s * rate)), 50, imu.size());
    const std::size_t step = std::max<std::size_t>(static_cast<std::size_t>(rf::f64_to_u64(0.1 * rate)), 1);
    for (std::size_t i = 0; i + n <= imu.size(); i += step) {
        const ImuRow* w = imu.data() + i;
        const auto [gm, gs] = stats(w, n, k_gyro);
        const auto [am_unused, as] = stats(w, n, k_accel);
        (void)am_unused;
        if (gs[0] < 0.004 && gs[1] < 0.004 && gs[2] < 0.004 && as[0] < 0.03 && as[1] < 0.03 && as[2] < 0.03) {
            return std::pair{gm, w[0].ts};
        }
    }
    return std::nullopt;
}

} // namespace pibridge::slamtool
