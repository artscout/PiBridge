// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-hands/src/fuse.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/fuse.h"
#include "pibridge/ctrl/hint.h"
#include "pibridge/ctrl/parity.h"

#include <limits>
#include <mutex>

#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/ctrl/rust.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <fcntl.h>
#include <format>
#include <unistd.h>

namespace pibridge::ctrl::fuse {

namespace rf = pibridge::cam::rfmt;
namespace io = pibridge::cam;

// ---------- small math ----------

Q qmul(Q a, Q b) {
    return {
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    };
}

Q qconj(Q a) {
    return {-a[0], -a[1], -a[2], a[3]};
}

Q qnorm(Q a) {
    const double n = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3]);
    if (n < 1e-12) {
        return {0.0, 0.0, 0.0, 1.0};
    }
    return {a[0] / n, a[1] / n, a[2] / n, a[3] / n};
}

V3 qrot(Q q, V3 v) {
    const Q p = qmul(qmul(q, {v[0], v[1], v[2], 0.0}), qconj(q));
    return {p[0], p[1], p[2]};
}

Q qexp(V3 w) {
    const double a = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    if (a < 1e-12) {
        return qnorm({w[0] / 2.0, w[1] / 2.0, w[2] / 2.0, 1.0});
    }
    const double s = std::sin(a / 2.0) / a;
    return {w[0] * s, w[1] * s, w[2] * s, std::cos(a / 2.0)};
}

V3 qlog(Q q) {
    if (q[3] < 0.0) {
        q = {-q[0], -q[1], -q[2], -q[3]};
    }
    const double s = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]);
    if (s < 1e-12) {
        return {2.0 * q[0], 2.0 * q[1], 2.0 * q[2]};
    }
    const double a = 2.0 * std::atan2(s, q[3]);
    return {q[0] / s * a, q[1] / s * a, q[2] / s * a};
}

double qangle(Q a, Q b) {
    const V3 v = qlog(qmul(qconj(a), b));
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

namespace {

V3 add(V3 a, V3 b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
V3 sub(V3 a, V3 b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
V3 scale(V3 a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
double norm(V3 a) { return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); }

/// a.partial_cmp(b).unwrap(): -1, 0, 1, or the Rust panic for NaN.
int pcmp(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) {
        rs::panic("called `Option::unwrap()` on a `None` value");
    }
    return a < b ? -1 : (a > b ? 1 : 0);
}

const char* hand_name(std::size_t hand) {
    return hand == 0 ? "left" : "right";
}

/// str::trim_start (Unicode White_Space).
std::string_view trim_start(std::string_view s) {
    std::size_t i = 0;
    while (i < s.size()) {
        const std::size_t n = rs::ws_len(s, i);
        if (n == 0) {
            break;
        }
        i += n;
    }
    return s.substr(i);
}

constexpr std::uint64_t RING_NS = 3000000000ull;

} // namespace

V3 mv(const M3& m, V3 v) {
    return {
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    };
}

double det3(const M3& m) {
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

std::optional<M3> inv3(const M3& m) {
    const double d = det3(m);
    if (std::fabs(d) < 1e-300) {
        return std::nullopt;
    }
    const auto c = [&](std::size_t r0, std::size_t c0, std::size_t r1, std::size_t c1) {
        return m[r0][c0] * m[r1][c1] - m[r0][c1] * m[r1][c0];
    };
    return M3{{
        {c(1, 1, 2, 2) / d, -c(0, 1, 2, 2) / d, c(0, 1, 1, 2) / d},
        {-c(1, 0, 2, 2) / d, c(0, 0, 2, 2) / d, -c(0, 0, 1, 2) / d},
        {c(1, 0, 2, 1) / d, -c(0, 0, 2, 1) / d, c(0, 0, 1, 1) / d},
    }};
}

M3 polar(const M3& m) {
    M3 x = m;
    for (int it = 0; it < 30; it++) {
        const auto inv = inv3(x);
        if (!inv) {
            break;
        }
        M3 n{};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                n[i][j] = 0.5 * (x[i][j] + (*inv)[j][i]);
            }
        }
        x = n;
    }
    return x;
}

Q yaw_only(Q q) {
    const V3 f = qrot(q, {0.0, 0.0, -1.0});
    if (std::fabs(f[0]) + std::fabs(f[2]) < 1e-6) {
        return {0.0, 0.0, 0.0, 1.0};
    }
    const double yaw = std::atan2(-f[0], -f[2]);
    return {0.0, std::sin(yaw / 2.0), 0.0, std::cos(yaw / 2.0)};
}

P P::mul(const P& o) const {
    return P{qnorm(qmul(q, o.q)), add(qrot(q, o.p), p)};
}

P P::inv() const {
    const Q qi = qconj(q);
    return P{qi, scale(qrot(qi, p), -1.0)};
}

// ---------- report decoding ----------

namespace {
std::int16_t be16(const std::uint8_t* r, std::size_t o) {
    return static_cast<std::int16_t>(static_cast<std::uint16_t>((r[o] << 8) | r[o + 1]));
}
} // namespace

std::array<std::array<std::int16_t, 3>, 2> report_accel(const std::uint8_t* raw) {
    return {{{be16(raw, 16), be16(raw, 18), be16(raw, 20)}, {be16(raw, 30), be16(raw, 32), be16(raw, 34)}}};
}

Gyro report_gyro(const std::uint8_t* raw) {
    const std::uint32_t ts = static_cast<std::uint32_t>(raw[8]) | (static_cast<std::uint32_t>(raw[9]) << 8) |
                             (static_cast<std::uint32_t>(raw[10]) << 16) | (static_cast<std::uint32_t>(raw[11]) << 24);
    bool second = false;
    for (std::size_t i = 30; i < 42; i++) {
        second = second || raw[i] != 0;
    }
    return Gyro{{ts, ts + 2000u},
                {{{be16(raw, 22), be16(raw, 24), be16(raw, 26)}, {be16(raw, 36), be16(raw, 38), be16(raw, 40)}}},
                second ? 2u : 1u};
}

// ---------- calibration ----------

double Calib::scale_dps_per_lsb() const {
    return rf::to_degrees(std::cbrt(std::fabs(det3(c))));
}

std::string Calib::axes() const {
    const M3 a = polar(c);
    std::string out;
    for (std::size_t r = 0; r < 3; r++) {
        const auto& row = a[r];
        // max_by on |v| with partial_cmp().unwrap(): the last of equal maxima
        std::size_t j = 0;
        for (std::size_t k = 1; k < 3; k++) {
            if (pcmp(std::fabs(row[j]), std::fabs(row[k])) != 1) {
                j = k;
            }
        }
        if (r > 0) {
            out += ' ';
        }
        out += row[j] >= 0.0 ? "+" : "-";
        out += "xyz"[j];
    }
    return out;
}

double Calib::axis_misalignment_deg() const {
    const M3 a = polar(c);
    double worst = 0.0;
    for (const auto& row : a) {
        double m = 0.0;
        for (double v : row) {
            m = rf::fmax(m, std::fabs(v));
        }
        worst = rf::fmax(worst, rf::to_degrees(std::acos(rf::clamp(m, -1.0, 1.0))));
    }
    return worst;
}

V3 Calib::omega(V3 raw) const {
    return mv(c, sub(raw, bias));
}

M3 Calib::axis_rotation() const {
    return polar(c);
}

V3 Calib::accel(V3 raw) const {
    return scale(mv(axis_rotation(), raw), ACCEL_G / ACCEL_LSB_PER_G);
}

void Acc::add(V3 x, V3 y) {
    const std::array<double, 4> xe = {x[0], x[1], x[2], 1.0};
    for (std::size_t i = 0; i < 4; i++) {
        for (std::size_t j = 0; j < 4; j++) {
            a[i][j] += xe[i] * xe[j];
        }
        for (std::size_t j = 0; j < 3; j++) {
            b[i][j] += xe[i] * y[j];
        }
    }
    yy += y[0] * y[0] + y[1] * y[1] + y[2] * y[2];
    ysum = fuse::add(ysum, y);
    n += 1;
    if (norm(y) > 0.6) {
        moving += 1;
    }
}

std::optional<Acc::Sol> Acc::solve() const {
    if (n < 12) {
        return std::nullopt;
    }
    auto A = a;
    auto B = b;
    for (std::size_t c = 0; c < 4; c++) {
        std::size_t p = c;
        for (std::size_t i = c + 1; i < 4; i++) {
            if (pcmp(std::fabs(A[p][c]), std::fabs(A[i][c])) != 1) {
                p = i;
            }
        }
        if (std::fabs(A[p][c]) < 1e-9) {
            return std::nullopt;
        }
        std::swap(A[c], A[p]);
        std::swap(B[c], B[p]);
        for (std::size_t r = 0; r < 4; r++) {
            if (r != c) {
                const double f = A[r][c] / A[c][c];
                for (std::size_t k = 0; k < 4; k++) {
                    A[r][k] -= f * A[c][k];
                }
                for (std::size_t k = 0; k < 3; k++) {
                    B[r][k] -= f * B[c][k];
                }
            }
        }
    }
    std::array<std::array<double, 3>, 4> w{};
    for (std::size_t r = 0; r < 4; r++) {
        for (std::size_t k = 0; k < 3; k++) {
            w[r][k] = B[r][k] / A[r][r];
        }
    }
    double wtb = 0.0;
    double wtaw = 0.0;
    for (std::size_t k = 0; k < 3; k++) {
        for (std::size_t i = 0; i < 4; i++) {
            wtb += w[i][k] * b[i][k];
            for (std::size_t j = 0; j < 4; j++) {
                wtaw += w[i][k] * a[i][j] * w[j][k];
            }
        }
    }
    const double ssres = rf::fmax(yy - 2.0 * wtb + wtaw, 0.0);
    const double nn = static_cast<double>(n);
    const double sstot = yy - (ysum[0] * ysum[0] + ysum[1] * ysum[1] + ysum[2] * ysum[2]) / nn;
    const double r2 = sstot > 1e-12 ? 1.0 - ssres / sstot : 0.0;
    const M3 c = {{{w[0][0], w[1][0], w[2][0]}, {w[0][1], w[1][1], w[2][1]}, {w[0][2], w[1][2], w[2][2]}}};
    return Sol{c, {w[3][0], w[3][1], w[3][2]}, r2};
}

void Fit::add(const std::array<V3, N_OFF>& xs, V3 y) {
    pairs.emplace_back(xs, y);
    while (pairs.size() > FIT_MAX_PAIRS) {
        pairs.pop_front();
    }
}

std::optional<Fit::Robust> Fit::robust(std::size_t k) const {
    const std::size_t n = pairs.size();
    std::vector<bool> keep(n, true);
    const auto solve = [&](const std::vector<bool>& kp) -> std::optional<Robust> {
        Acc a;
        for (std::size_t i = 0; i < n; i++) {
            if (kp[i]) {
                a.add(pairs[i].first[k], pairs[i].second);
            }
        }
        const auto s = a.solve();
        if (!s) {
            return std::nullopt;
        }
        return Robust{s->c, s->d, s->r2, a.n, a.moving};
    };
    auto cur = solve(keep);
    if (!cur) {
        return std::nullopt;
    }
    for (int round = 0; round < 4; round++) {
        std::vector<double> res(n);
        for (std::size_t i = 0; i < n; i++) {
            res[i] = norm(sub(pairs[i].second, fuse::add(mv(cur->c, pairs[i].first[k]), cur->d)));
        }
        std::vector<double> kept;
        for (std::size_t i = 0; i < n; i++) {
            if (keep[i]) {
                kept.push_back(res[i]);
            }
        }
        if (kept.size() < 24) {
            break;
        }
        std::stable_sort(kept.begin(), kept.end(), [](double x, double y) { return pcmp(x, y) < 0; });
        const double thr =
            rf::fmax(kept[rs::f64_to_usize(static_cast<double>(kept.size()) * 0.85)], 0.3);
        for (std::size_t i = 0; i < n; i++) {
            keep[i] = keep[i] && res[i] <= thr;
        }
        cur = solve(keep);
        if (!cur) {
            return std::nullopt;
        }
    }
    return cur;
}

std::optional<Calib> Fit::best(std::uint32_t min_pairs, std::uint32_t min_moving, double min_r2) const {
    std::optional<Calib> best;
    for (std::size_t k = 0; k < N_OFF; k++) {
        const auto r = robust(k);
        if (!r) {
            continue;
        }
        if (r->kept < min_pairs || r->moving < min_moving || r->r2 < min_r2 || std::fabs(det3(r->c)) < 1e-18) {
            continue;
        }
        const auto ci = inv3(r->c);
        if (!ci) {
            continue;
        }
        const V3 bias = scale(mv(*ci, r->d), -1.0);
        const Calib cal{r->c, bias, FIT_OFFSETS_MS[k], r->kept, r->moving, r->r2};
        if (!best || cal.r2 > best->r2) {
            best = cal;
        }
    }
    return best;
}

Calib default_calib(std::size_t hand) {
    Calib c;
    if (hand == 0) {
        c.c = {{{2.67478985236365e-4, 1.6795515395578024e-4, -2.109750888158635e-3},
                {-2.120013326287931e-3, -6.379925495236746e-5, -3.9738136017879584e-4},
                {1.0217973941656887e-5, 2.0903105813178473e-3, 1.7519832998645835e-4}}};
    } else {
        c.c = {{{2.9903520979486335e-4, 4.4348525459939994e-5, 2.093408299192325e-3},
                {2.1180422764101904e-3, -6.652499863749047e-5, -2.916982133170233e-4},
                {5.9122966786586084e-5, 2.1070131931618533e-3, -4.658550218077466e-5}}};
    }
    c.r2 = 0.98;
    return c;
}

std::optional<Calib> calib_or_default(const std::string& path, std::size_t hand) {
    if (auto c = load_calib(path, hand)) {
        return c;
    }
    const auto v = io::env_utf8("PIMAX_CTRL_IMU_DEFAULT");
    const bool off = v && rf::trim(*v) == "0";
    if (off) {
        return std::nullopt;
    }
    return default_calib(hand);
}

// ---------- calibration file ----------

std::string calib_path() {
    if (auto p = io::env_os("PIMAX_CTRL_IMU_CALIB"); p && !p->empty()) {
        return *p;
    }
    const std::string h = io::env_os("HOME").value_or("");
    return io::path_join(h, ".config/omarchyvr/ctrl-imu.json");
}

std::vector<double> nums(std::string_view s) {
    std::vector<double> out;
    std::size_t start = 0;
    std::size_t i = 0;
    const auto token = [&](std::size_t end) {
        if (end > start) {
            double v = 0.0;
            if (rf::parse_f64(s.substr(start, end - start), v)) {
                out.push_back(v);
            }
        }
    };
    while (i < s.size()) {
        const char ch = s[i];
        std::size_t sep = 0;
        if (ch == ',' || ch == '[' || ch == ']') {
            sep = 1;
        } else {
            sep = rs::ws_len(s, i);
        }
        if (sep > 0) {
            token(i);
            i += sep;
            start = i;
        } else {
            i++;
        }
    }
    token(s.size());
    return out;
}

namespace {

/// Value text after "key": inside obj (up to the matching ] for arrays, else up to , / } / newline).
std::optional<std::string_view> field(std::string_view obj, std::string_view key) {
    const std::string pat = "\"" + std::string(key) + "\"";
    const auto i = obj.find(pat);
    if (i == std::string_view::npos) {
        return std::nullopt;
    }
    std::string_view rest = obj.substr(i + key.size() + 2);
    const auto colon = rest.find(':');
    if (colon == std::string_view::npos) {
        return std::nullopt;
    }
    rest = trim_start(rest.substr(colon + 1));
    if (rest.starts_with('[')) {
        const auto e = rest.find(']');
        if (e == std::string_view::npos) {
            return std::nullopt;
        }
        return rest.substr(0, e + 1);
    }
    auto e = rest.find_first_of(",}\n");
    if (e == std::string_view::npos) {
        e = rest.size();
    }
    return rf::trim(rest.substr(0, e));
}

std::optional<std::string_view> hand_obj(std::string_view text, std::size_t hand) {
    const std::string pat = std::string("\"") + hand_name(hand) + "\"";
    const auto i = text.find(pat);
    if (i == std::string_view::npos) {
        return std::nullopt;
    }
    const std::string_view rest = text.substr(i);
    const auto s = rest.find('{');
    if (s == std::string_view::npos) {
        return std::nullopt;
    }
    const auto e = rest.substr(s).find('}');
    if (e == std::string_view::npos) {
        return std::nullopt;
    }
    return rest.substr(s, e + 1);
}

std::string hand_json(const Calib& c) {
    std::string fl;
    for (std::size_t r = 0; r < 3; r++) {
        for (std::size_t k = 0; k < 3; k++) {
            if (r + k > 0) {
                fl += ", ";
            }
            fl += rs::lower_exp(c.c[r][k]);
        }
    }
    return "{ \"c\": [" + fl + "], \"bias_lsb\": [" + rf::fixed(c.bias[0], 3) + ", " + rf::fixed(c.bias[1], 3) + ", " +
           rf::fixed(c.bias[2], 3) + "], \"time_offset_ms\": " + rf::display_f64(c.time_offset_ms) +
           ", \"pairs\": " + std::to_string(c.pairs) + ", \"moving\": " + std::to_string(c.moving) +
           ", \"r2\": " + rf::fixed(c.r2, 4) + ", \"scale_dps_per_lsb\": " + rf::fixed(c.scale_dps_per_lsb(), 5) +
           ", \"axes\": \"" + c.axes() + "\", \"axis_misalignment_deg\": " + rf::fixed(c.axis_misalignment_deg(), 2) +
           " }";
}

} // namespace

std::optional<Calib> load_calib(const std::string& path, std::size_t hand) {
    const auto t = io::read_to_string(path);
    if (!t) {
        return std::nullopt;
    }
    if (t->find("pimax-ctrl-imu") == std::string::npos) {
        return std::nullopt;
    }
    const auto o = hand_obj(*t, hand);
    if (!o) {
        return std::nullopt;
    }
    const auto fc = field(*o, "c");
    if (!fc) {
        return std::nullopt;
    }
    const auto c = nums(*fc);
    const auto fb = field(*o, "bias_lsb");
    if (!fb) {
        return std::nullopt;
    }
    const auto b = nums(*fb);
    if (c.size() != 9 || b.size() != 3) {
        return std::nullopt;
    }
    const auto f = [&](std::string_view k) -> std::optional<double> {
        const auto v = field(*o, k);
        double x = 0.0;
        if (v && rf::parse_f64(*v, x)) {
            return x;
        }
        return std::nullopt;
    };
    Calib cal;
    cal.c = {{{c[0], c[1], c[2]}, {c[3], c[4], c[5]}, {c[6], c[7], c[8]}}};
    cal.bias = {b[0], b[1], b[2]};
    cal.time_offset_ms = f("time_offset_ms").value_or(0.0);
    cal.pairs = rf::f64_to_u32(f("pairs").value_or(0.0));
    cal.moving = rf::f64_to_u32(f("moving").value_or(0.0));
    cal.r2 = f("r2").value_or(0.0);
    return cal;
}

bool save_calib(const std::string& path, std::size_t hand, const Calib& c) {
    const std::size_t other = 1 - hand;
    const std::string old = io::read_to_string(path).value_or("");
    std::optional<std::string> keep;
    if (old.find("pimax-ctrl-imu") != std::string::npos) {
        if (auto o = hand_obj(old, other)) {
            keep = std::string(*o);
        }
    }
    std::array<std::optional<std::string>, 2> entries;
    entries[hand] = hand_json(c);
    entries[other] = keep;
    std::string s = "{\n  \"format\": \"pimax-ctrl-imu\",\n  \"version\": 1,\n  \"note\": \"controller gyro calibration "
                    "fitted on optical poses (pimax-hands fuse.rs): aim-frame rad/s = c * (raw - bias_lsb)\"";
    for (std::size_t h = 0; h < 2; h++) {
        if (entries[h]) {
            s += ",\n  \"" + std::string(hand_name(h)) + "\": " + *entries[h];
        }
    }
    s += "\n}\n";
    if (const auto d = io::path_parent(path)) {
        if (!io::create_dir_all(*d)) {
            return false;
        }
    }
    const std::string tmp = rs::with_extension(path, "json.tmp" + std::to_string(rs::process_id()));
    if (!io::write_file(tmp, s)) {
        return false;
    }
    return ::rename(tmp.c_str(), path.c_str()) == 0;
}

bool fit_replaces(std::optional<double> old_r2, double new_r2) {
    return !old_r2 || new_r2 >= *old_r2 - 0.02;
}

// ---------- config ----------

void Config::set_legacy_window() {
    hyst = false;
    extrap_s = 0.08;
    gap_s = 0.1;
    confirm_n = 1;
}

Config Config::pibridge() {
    Config c;
    // the inertial position (ins_s) was better on the synthetic IMU but worse on both real takes (overshoot after
    // hard stops 87 -> 57 mm without it, cut errors lower): off; the tracker's IMU-led filter carries the IMU now
    c.ins_s = 0.0;
    c.ins_tau_v = 0.5;
    c.tilt_fix = true;
    c.keep_tau_s = 1.5;
    c.blend_mps = 1.0;
    c.grav_check_deg = 25.0;
    c.ins_jump_m = 0.06;
    return c;
}

Config Config::from_env() {
    Config c = parity_mode() ? Config{} : pibridge();
    if (const auto v = io::env_utf8("PIMAX_CTRL_TRACK_HYST")) {
        const auto t = rf::trim(*v);
        if (t == "0" || t == "off" || t == "false") {
            c.set_legacy_window();
        }
    }
    const auto num = [](const char* k) -> std::optional<double> {
        const auto v = io::env_utf8(k);
        double x = 0.0;
        if (v && rf::parse_f64(rf::trim(*v), x)) {
            return x;
        }
        return std::nullopt;
    };
    if (const auto v = io::env_utf8("PIMAX_CTRL_TRACK_IMU")) {
        const auto t = rf::trim(*v);
        if (t == "0" || t == "off" || t == "false") {
            c.use_imu = false;
        } else if (t == "1" || t == "on" || t == "true") {
            c.use_imu = true;
        }
    }
    if (const auto v = num("PIMAX_CTRL_TRACK_LOST_S"); v && *v > 0.0) {
        c.lost_s = *v;
    }
    if (const auto v = num("PIMAX_CTRL_TRACK_EXTRAP_MS"); v && *v >= 0.0) {
        c.extrap_s = *v / 1000.0;
    }
    if (const auto v = num("PIMAX_CTRL_TRACK_BLEND_MS"); v && *v >= 0.0) {
        c.blend_s = *v / 1000.0;
    }
    if (const auto v = num("PIMAX_CTRL_IMU_FIT"); v && *v == 0.0) {
        c.online_fit = false;
    }
    if (const auto v = num("PIMAX_CTRL_TRACK_FILTER"); v && *v == 0.0) {
        c.filter = false;
    }
    if (const auto v = num("PIMAX_CTRL_TRACK_STILL_LOCK"); v && *v == 0.0) {
        c.still_lock = false;
    }
    if (const auto v = num("PIMAX_CTRL_TRACK_FOLLOW_YAW"); v && 0.0 <= *v && *v <= 1.0) {
        c.follow_yaw = *v;
    }
    if (const auto v = num("PIMAX_CTRL_TRACK_BLEND_MPS"); v && *v >= 0.0) {
        c.blend_mps = *v;
        if (*v == 0.0) {
            c.blend_dps = 0.0;
        }
    }
    if (const auto v = num("PIMAX_CTRL_TRACK_DR_MS"); v && *v >= 0.0) {
        c.dr_tau_s = *v / 1000.0;
    }
    // PiBridge's switches (A/B against the Rust behaviour without PIBRIDGE_CTRL_PARITY)
    if (const auto v = num("PIBRIDGE_CTRL_INS_MS"); v && *v >= 0.0) {
        c.ins_s = *v / 1000.0;
    }
    if (const auto v = num("PIBRIDGE_CTRL_INS_TAU_MS"); v && *v > 0.0) {
        c.ins_tau_v = *v / 1000.0;
    }
    if (const auto v = num("PIBRIDGE_CTRL_INS_FREE_MS"); v && *v >= 0.0) {
        c.ins_free_s = *v / 1000.0;
    }
    if (const auto v = num("PIBRIDGE_CTRL_INS_JUMP_MM"); v && *v >= 0.0) {
        c.ins_jump_m = *v / 1000.0;
    }
    if (!parity_mode()) {
        const auto v = io::env_utf8("PIBRIDGE_CTRL_HINT");
        if (!(v && rf::trim(*v) == "0")) {
            c.hint_path = hint::default_path();
        }
    }
    if (const auto v = num("PIBRIDGE_CTRL_TURN_DROP"); v) {
        c.turn_drop = *v != 0.0;
    }
    if (const auto v = num("PIBRIDGE_CTRL_RAW_ANCHOR"); v) {
        c.raw_anchor = *v != 0.0;
    }
    if (const auto v = num("PIBRIDGE_CTRL_GRAV_DEG"); v && *v >= 0.0) {
        c.grav_check_deg = *v;
    }
    if (const auto v = num("PIBRIDGE_CTRL_TILT"); v) {
        c.tilt_fix = *v != 0.0;
    }
    if (const auto v = num("PIBRIDGE_CTRL_KEEP_S"); v && *v >= 0.0) {
        c.keep_tau_s = *v;
    }
    if (const auto v = io::env_utf8("PIMAX_CTRL_TRACK_FOREARM")) {
        const auto n = nums(*v);
        if (n.size() == 1 && n[0] == 0.0) {
            c.forearm = std::nullopt;
        } else if (n.size() == 3) {
            c.forearm = V3{n[0], n[1], n[2]};
        }
    }
    return c;
}

// ---------- the log ----------

/// BufWriter<File> with 64 KiB, as the Rust log.
struct Fuser::Log {
    int fd = -1;
    std::string buf;
    std::uint64_t span_ns = 0;
    std::optional<std::uint64_t> end;
    std::pair<std::uint64_t, bool> flag{0, false};

    static constexpr std::size_t CAP = 1 << 16;
    void raw_write(const char* p, std::size_t n) {
        while (n > 0) {
            const ssize_t w = ::write(fd, p, n);
            if (w < 0) {
                if (errno == EINTR) {
                    continue;
                }
                return;
            }
            p += w;
            n -= static_cast<std::size_t>(w);
        }
    }
    void flush() {
        if (!buf.empty()) {
            raw_write(buf.data(), buf.size());
            buf.clear();
        }
    }
    void write(std::string_view s) {
        if (buf.size() + s.size() > CAP) {
            flush();
        }
        if (s.size() >= CAP) {
            raw_write(s.data(), s.size());
        } else {
            buf.append(s);
        }
    }
    ~Log() {
        flush();
        if (fd >= 0) {
            ::close(fd);
        }
    }
};

// ---------- the fuser ----------

Fuser::Fuser(std::size_t hand_, Config cfg_, std::optional<Calib> calib_)
    : hand(hand_), cfg(cfg_), calib(calib_), calib_from_file_(calib_.has_value()) {}

Fuser::~Fuser() {
    delete log_;
}

bool Fuser::start_log(const std::string& path, double minutes) {
    const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0666);
    if (fd < 0) {
        return false;
    }
    auto* l = new Log();
    l->fd = fd;
    l->write(std::string("# pimax-ctrlfuse log v1 hand=") + hand_name(hand) + "\n");
    l->write("# I,host_ns,dev_us,gx,gy,gz,ax,ay,az (raw LSB) | O,frame_ns,px,py,pz,qx,qy,qz,qw (world_from_aim),hpx,hpy,"
             "hpz,hqx,hqy,hqz,hqw (world_from_head),rms_px,leds,verdict | P,t_ns,state,px,py,pz,qx,qy,qz,qw (shown aim "
             "pose),hpx,hpy,hpz,hqx,hqy,hqz,hqw (head),other_px,other_py,other_pz (50 Hz)\n");
    l->span_ns = rf::f64_to_u64(minutes * 60e9);
    delete log_;
    log_ = l;
    return true;
}

void Fuser::log_line(std::uint64_t t, const std::string& line) {
    Log* l = log_;
    if (l == nullptr) {
        return;
    }
    if (!l->end) {
        l->end = rs::sat_add(t, l->span_ns);
    }
    const std::uint64_t end = *l->end;
    if (t > end) {
        if (rs::sat_sub(t, l->flag.first) > 1000000000u) {
            bool on = false;
            if (const auto d = io::env_os("XDG_RUNTIME_DIR"); d && !d->empty()) {
                on = io::path_exists(io::path_join(*d, "omarchyvr-imu-log.on"));
            }
            if (l->flag.second && !on) {
                l->flush();
            }
            l->flag = {t, on};
        }
        if (!l->flag.second) {
            return;
        }
    }
    l->write(line);
    l->write("\n");
}

void Fuser::flush_log() {
    if (log_ != nullptr) {
        log_->flush();
    }
}

bool Fuser::gyro_active() const {
    if (cfg.use_imu) {
        return *cfg.use_imu;
    }
    return calib.has_value();
}

std::optional<Calib> Fuser::active_calib() const {
    if (!gyro_active()) {
        return std::nullopt;
    }
    if (calib) {
        return calib;
    }
    // forced on without a fit: Monado's old guess (16.4 LSB per deg/s, raw axes = aim axes)
    const double s = rf::to_radians(1.0 / 16.4);
    Calib c;
    c.c = {{{s, 0.0, 0.0}, {0.0, s, 0.0}, {0.0, 0.0, s}}};
    c.r2 = 0.0;
    return c;
}

void Fuser::report(std::uint64_t host_ns, const std::uint8_t* raw) {
    const Gyro g = report_gyro(raw);
    const auto a = report_accel(raw);
    for (std::size_t k = 0; k < g.n; k++) {
        sample_acc(host_ns, g.ts[k], g.g[k], a[k]);
    }
}

void Fuser::sample(std::uint64_t host_ns, std::uint32_t dev_ts, std::array<std::int16_t, 3> g) {
    sample_acc(host_ns, dev_ts, g, std::nullopt);
}

void Fuser::sample_acc(std::uint64_t host_ns, std::uint32_t dev_ts, std::array<std::int16_t, 3> g,
                       std::optional<std::array<std::int16_t, 3>> acc) {
    // unwrap the u32 us device clock
    std::uint64_t dt_us = 0;
    if (dev_last_) {
        const std::uint32_t d = dev_ts - *dev_last_;
        if (d > 0x80000000u) {
            return; // older than the last sample (reordered): ignore
        }
        dt_us = d;
    }
    dev_last_ = dev_ts;
    dev_us_ += dt_us;
    // host time of the sample: lower envelope of host - device (USB delays only add), drifting up 50 us/s
    const std::int64_t off = rs::wsub(static_cast<std::int64_t>(host_ns), static_cast<std::int64_t>(dev_us_ * 1000u));
    std::int64_t est = off;
    if (clk_off_) {
        const double age_s = static_cast<double>(rs::sat_sub(host_ns, clk_t_)) * 1e-9;
        est = std::min(rs::wadd(*clk_off_, rs::f64_to_i64(age_s * 50000.0)), off);
    }
    clk_off_ = est;
    clk_t_ = host_ns;
    const std::uint64_t t =
        static_cast<std::uint64_t>(std::max<std::int64_t>(rs::wadd(static_cast<std::int64_t>(dev_us_ * 1000u), est), 0));
    const V3 x = {static_cast<double>(g[0]), static_cast<double>(g[1]), static_cast<double>(g[2])};
    stats.imu_samples += 1;
    if (log_ != nullptr) {
        if (acc) {
            log_line(t, std::format("I,{},{},{},{},{},{},{},{}", t, dev_ts, g[0], g[1], g[2], (*acc)[0], (*acc)[1],
                                    (*acc)[2]));
        } else {
            log_line(t, std::format("I,{},{},{},{},{}", t, dev_ts, g[0], g[1], g[2]));
        }
    }
    raw_.emplace_back(t, x);
    if (acc) {
        acc_.emplace_back(t, V3{static_cast<double>((*acc)[0]), static_cast<double>((*acc)[1]),
                                static_cast<double>((*acc)[2])});
        while (!acc_.empty() && rs::sat_sub(t, acc_.front().first) > RING_NS) {
            acc_.pop_front();
        }
    }
    while (!raw_.empty() && rs::sat_sub(t, raw_.front().first) > RING_NS) {
        raw_.pop_front();
    }
    update_rest_bias();
    if (!pending_.empty()) {
        process_pending();
    }
    if (const auto c = active_calib()) {
        const double dt = rf::clamp(static_cast<double>(dt_us) * 1e-6, 0.0, 0.02);
        const V3 w = c->omega(x);
        q_imu_ = qnorm(qmul(q_imu_, qexp(scale(w, dt))));
    }
    q_ring_.emplace_back(t, q_imu_);
    while (!q_ring_.empty() && rs::sat_sub(t, q_ring_.front().first) > RING_NS) {
        q_ring_.pop_front();
    }
    if (!cfg.hint_path.empty() || cfg.imu_state) {
        // the hand-identity state (imu_state): smoothed accelerometer and gyro rate, how long the gyro is quiet
        if (const auto c = active_calib()) {
            const double dt = rf::clamp(static_cast<double>(dt_us) * 1e-6, 0.0, 0.02);
            const double k = rf::fmin(dt / 0.2, 1.0);
            w_lp_ += (norm(c->omega(x)) - w_lp_) * k;
            if (acc) {
                const V3 f = c->accel({static_cast<double>((*acc)[0]), static_cast<double>((*acc)[1]),
                                       static_cast<double>((*acc)[2])});
                acc_lp_ = add(acc_lp_, scale(sub(f, acc_lp_), rf::fmin(dt / 0.1, 1.0)));
                hint_acc_ = add(hint_acc_, f);
                hint_acc_n_ += 1;
            }
            const bool still = w_lp_ < 0.05;
            if (still && !still_now_) {
                still_since_ = t;
            }
            still_now_ = still;
        }
    }
    if (cfg.keep_tau_s > 0.0 && last_) {
        const double age = static_cast<double>(rs::sat_sub(t, last_->t)) * 1e-9;
        if (age > 1.5 && (!qa_ || qa_->fix_t != last_->t || rs::sat_sub(t, qa_->t) > 1000000000u)) {
            if (const auto q = gyro_world(t)) {
                qa_ = Anchor{last_->t, t, *q};
            }
        }
    }
    if (acc && (cfg.ins_s > 0.0 || cfg.tilt_fix)) {
        const V3 a = {static_cast<double>((*acc)[0]), static_cast<double>((*acc)[1]), static_cast<double>((*acc)[2])};
        if (cfg.tilt_fix && last_) {
            if (tilt_t_ != last_->t) {
                tilt_ = {0.0, 0.0, 0.0, 1.0};
                tilt_t_ = last_->t;
            }
            const double age = static_cast<double>(rs::sat_sub(t, last_->t)) * 1e-9;
            const auto c = active_calib();
            const auto q = world_q(t);
            const double dt = rf::clamp(static_cast<double>(dt_us) * 1e-6, 0.0, 0.02);
            if (age > cfg.extrap_s && c && q) {
                const V3 f = c->accel(a);
                const double n = norm(f);
                const double rate = norm(c->omega(x));
                if (n > 0.85 * ACCEL_G && n < 1.15 * ACCEL_G && rate < 1.0) {
                    const V3 up = qrot(*q, scale(f, 1.0 / n));
                    // up x (0, 1, 0): the axis that turns the measured up onto the true one
                    const V3 axis = {-up[2], 0.0, up[0]};
                    tilt_ = qnorm(qmul(qexp(scale(axis, 0.5 * dt)), tilt_));
                }
            }
        }
        if (cfg.ins_s > 0.0) {
            ins_sample(t, static_cast<double>(dt_us) * 1e-6, a);
        }
    }
}

std::optional<Q> Fuser::gyro_world(std::uint64_t t) const {
    if (!last_) {
        return std::nullopt;
    }
    // past about 1.5 s from the fix, from the latest re-anchored orientation (the gyro ring holds 3 s)
    if (qa_ && qa_->fix_t == last_->t && t >= qa_->t) {
        const auto dq = gyro_delta(qa_->t, t);
        return dq ? std::optional<Q>(qnorm(qmul(qa_->q, *dq))) : std::nullopt;
    }
    const auto dq = gyro_delta(last_->t, t);
    if (!dq) {
        return std::nullopt;
    }
    return qnorm(qmul(last_->w.q, *dq));
}

std::optional<Q> Fuser::world_q(std::uint64_t t) const {
    const auto q = gyro_world(t);
    if (!q) {
        return std::nullopt;
    }
    return cfg.tilt_fix && last_ && tilt_t_ == last_->t ? qnorm(qmul(tilt_, *q)) : *q;
}

namespace {
constexpr double INS_SIGMA_A = 0.1;    // m/s^2, accelerometer white noise incl. orientation noise
constexpr double INS_SIGMA_BW = 0.02;  // m/s^2 per sqrt(s), bias random walk
constexpr std::uint64_t INS_HIST_NS = 500000000u;
} // namespace

void Fuser::ins_sample(std::uint64_t t, double dt_dev, V3 acc_raw) {
    if (!ins_.on || !last_) {
        return;
    }
    const auto c = active_calib();
    const auto q = world_q(t);
    const double dt = static_cast<double>(rs::sat_sub(t, ins_.t)) * 1e-9;
    (void)dt_dev;
    if (!c || !q || dt <= 0.0 || dt > 0.05) {
        ins_.t = t;
        return;
    }
    const double age = static_cast<double>(rs::sat_sub(t, last_->t)) * 1e-9;
    const V3 f = qrot(*q, c->accel(acc_raw));
    const V3 a = {f[0], f[1] - ACCEL_G, f[2]};
    const bool frozen = age > cfg.ins_s;
    // zero-velocity update: a controller whose gyro is quiet is not moving either
    const double rate = norm(c->omega(raw_.empty() ? V3{0.0, 0.0, 0.0} : raw_.back().second));
    ins_.w_avg += (rate - ins_.w_avg) * rf::fmin(dt / 0.1, 1.0);
    double damp = age > cfg.ins_free_s ? std::exp(-dt / cfg.ins_tau_v) : 1.0;
    if (ins_.w_avg < 0.1) {
        damp *= std::exp(-dt / 0.05);
    }
    for (std::size_t k = 0; k < 3; k++) {
        Kf& K = ins_.ax[k];
        if (!frozen) {
            const double ak = a[k] - K.x[2];
            K.x[0] += K.x[1] * dt + 0.5 * ak * dt * dt;
            K.x[1] = (K.x[1] + ak * dt) * damp;
        } else {
            K.x[1] = 0.0;
        }
        // P = F P F^T + Q, F = [[1, dt, -dt^2/2], [0, 1, -dt], [0, 0, 1]]
        const double h = -0.5 * dt * dt;
        std::array<std::array<double, 3>, 3> fp{};
        const std::array<std::array<double, 3>, 3> F = {{{1.0, dt, h}, {0.0, 1.0, -dt}, {0.0, 0.0, 1.0}}};
        for (std::size_t i = 0; i < 3; i++) {
            for (std::size_t j = 0; j < 3; j++) {
                for (std::size_t m = 0; m < 3; m++) {
                    fp[i][j] += F[i][m] * K.p[m][j];
                }
            }
        }
        std::array<std::array<double, 3>, 3> np{};
        for (std::size_t i = 0; i < 3; i++) {
            for (std::size_t j = 0; j < 3; j++) {
                for (std::size_t m = 0; m < 3; m++) {
                    np[i][j] += fp[i][m] * F[j][m];
                }
            }
        }
        const double qa = INS_SIGMA_A * INS_SIGMA_A;
        np[0][0] += qa * dt * dt * dt / 3.0;
        np[0][1] += qa * dt * dt / 2.0;
        np[1][0] += qa * dt * dt / 2.0;
        np[1][1] += qa * dt;
        np[2][2] += INS_SIGMA_BW * INS_SIGMA_BW * dt;
        K.p = np;
    }
    // never farther than ins_max_m from the last fix
    V3 p = {ins_.ax[0].x[0], ins_.ax[1].x[0], ins_.ax[2].x[0]};
    const V3 d = sub(p, last_->w.p);
    if (norm(d) > cfg.ins_max_m) {
        p = add(last_->w.p, scale(d, cfg.ins_max_m / norm(d)));
        for (std::size_t k = 0; k < 3; k++) {
            ins_.ax[k].x[0] = p[k];
        }
    }
    ins_.t = t;
    ins_.hist.emplace_back(t, p);
    while (!ins_.hist.empty() && rs::sat_sub(t, ins_.hist.front().first) > INS_HIST_NS) {
        ins_.hist.pop_front();
    }
}

void Fuser::ins_fix(std::uint64_t t, V3 z, std::uint32_t leds, bool gap) {
    const double sm = leds >= 9 ? 0.002 : leds >= 6 ? 0.004 : 0.008;
    const double age = static_cast<double>(rs::sat_sub(t, ins_.fix_t)) * 1e-9;
    const std::uint64_t now = raw_.empty() ? t : std::max(t, raw_.back().first);
    if (!ins_.on || (gap && age > cfg.ins_s)) {
        // (re)start: at the fix, at rest; the learned bias is kept
        const bool had = ins_.on;
        for (std::size_t k = 0; k < 3; k++) {
            Kf& K = ins_.ax[k];
            const double b = had ? K.x[2] : 0.0;
            const double pb = had ? std::max(K.p[2][2], 0.01) : 0.09;
            K.x = {z[k], 0.0, b};
            K.p = {{{sm * sm, 0.0, 0.0}, {0.0, 0.25, 0.0}, {0.0, 0.0, pb}}};
        }
        ins_.on = true;
        ins_.t = now;
        ins_.hist.clear();
        ins_.hist.emplace_back(now, z);
    } else {
        // the filter's position at the fix time (the fix arrives later than its frame)
        V3 then = {ins_.ax[0].x[0], ins_.ax[1].x[0], ins_.ax[2].x[0]};
        for (auto it = ins_.hist.rbegin(); it != ins_.hist.rend(); ++it) {
            if (it->first <= t) {
                then = it->second;
                break;
            }
        }
        for (std::size_t k = 0; k < 3; k++) {
            Kf& K = ins_.ax[k];
            const double y = z[k] - then[k];
            const double S = K.p[0][0] + sm * sm;
            const std::array<double, 3> g = {K.p[0][0] / S, K.p[1][0] / S, K.p[2][0] / S};
            for (std::size_t i = 0; i < 3; i++) {
                K.x[i] += g[i] * y;
            }
            std::array<std::array<double, 3>, 3> np = K.p;
            for (std::size_t i = 0; i < 3; i++) {
                for (std::size_t j = 0; j < 3; j++) {
                    np[i][j] -= g[i] * K.p[0][j];
                }
            }
            K.p = np;
            for (auto& [ht, hp] : ins_.hist) {
                hp[k] += g[0] * y;
            }
        }
    }
    ins_.fix_t = t;
    ins_.handover = std::nullopt;
}

std::optional<double> Fuser::grav_angle(std::uint64_t t, const Q& q) const {
    const auto c = active_calib();
    if (!c || acc_.empty()) {
        return std::nullopt;
    }
    // the accelerometer around the fix time (+-20 ms), shifted by the camera-to-IMU offset of the calibration
    const std::int64_t off = rs::f64_to_i64(c->time_offset_ms * 1e6);
    const std::uint64_t ti = static_cast<std::uint64_t>(std::max<std::int64_t>(rs::wadd(static_cast<std::int64_t>(t), off), 0));
    V3 sum = {0.0, 0.0, 0.0};
    double n = 0.0;
    for (auto it = acc_.rbegin(); it != acc_.rend(); ++it) {
        if (it->first + 20000000u < ti) {
            break;
        }
        if (it->first <= ti + 20000000u) {
            sum = add(sum, it->second);
            n += 1.0;
        }
    }
    if (n < 3.0) {
        return std::nullopt;
    }
    const V3 f = c->accel(scale(sum, 1.0 / n));
    const double m = norm(f);
    if (m < 0.85 * ACCEL_G || m > 1.15 * ACCEL_G) {
        return std::nullopt;
    }
    const V3 up = qrot(q, scale(f, 1.0 / m));
    return rf::to_degrees(std::acos(rf::clamp(up[1], -1.0, 1.0)));
}

std::optional<V3> Fuser::ins_pos(std::uint64_t t) const {
    if (!ins_.on || ins_.hist.empty()) {
        return std::nullopt;
    }
    if (t >= ins_.t) {
        const double dt = rf::fmin(static_cast<double>(t - ins_.t) * 1e-9, 0.05);
        const double age = last_ ? static_cast<double>(rs::sat_sub(t, last_->t)) * 1e-9 : 0.0;
        const double v = age > cfg.ins_s ? 0.0 : 1.0;
        return V3{ins_.ax[0].x[0] + v * ins_.ax[0].x[1] * dt, ins_.ax[1].x[0] + v * ins_.ax[1].x[1] * dt,
                  ins_.ax[2].x[0] + v * ins_.ax[2].x[1] * dt};
    }
    for (auto it = ins_.hist.rbegin(); it != ins_.hist.rend(); ++it) {
        if (it->first <= t) {
            return it->second;
        }
    }
    return ins_.hist.front().second;
}

void Fuser::update_rest_bias() {
    if (raw_.empty()) {
        return;
    }
    const std::uint64_t t = raw_.back().first;
    std::vector<V3> win;
    for (auto it = raw_.rbegin(); it != raw_.rend(); ++it) {
        if (!(t - it->first < 500000000u)) { // wrapping u64 subtraction, as in release Rust
            break;
        }
        win.push_back(it->second);
    }
    if (win.size() < 100) {
        return;
    }
    const double n = static_cast<double>(win.size());
    V3 sum = {0.0, 0.0, 0.0};
    for (const V3& x : win) {
        sum = add(sum, x);
    }
    const V3 mean = scale(sum, 1.0 / n);
    bool near_bias = true;
    if (calib) {
        for (int k = 0; k < 3; k++) {
            near_bias = near_bias && std::fabs(mean[k] - calib->bias[k]) <= 40.0;
        }
    }
    if (!near_bias) {
        return;
    }
    for (const V3& x : win) {
        for (int k = 0; k < 3; k++) {
            if (!(std::fabs(x[k] - mean[k]) <= 6.0)) {
                return;
            }
        }
    }
    rest_n_ += 1;
    rest_sum_ = add(rest_sum_, mean);
    if (rest_n_ % 50 == 0) {
        if (calib) {
            const V3 m = scale(rest_sum_, 1.0 / static_cast<double>(rest_n_));
            calib->bias = add(scale(calib->bias, 0.7), scale(m, 0.3));
        }
        rest_n_ = 0;
        rest_sum_ = {0.0, 0.0, 0.0};
    }
}

std::optional<Q> Fuser::q_at(std::uint64_t t) const {
    const auto& ring = q_ring_;
    if (ring.empty()) {
        return std::nullopt;
    }
    if (t >= ring.back().first) {
        return ring.back().second;
    }
    if (t <= ring.front().first) {
        return ring.front().second;
    }
    // binary search (the ring is in time order: host times only grow)
    const auto it = std::partition_point(ring.begin(), ring.end(), [t](const auto& e) { return e.first <= t; });
    const std::size_t idx = static_cast<std::size_t>(it - ring.begin());
    const auto [t0, q0] = ring[idx - 1];
    auto [t1, q1] = ring[idx];
    const double f = t1 > t0 ? static_cast<double>(t - t0) / static_cast<double>(t1 - t0) : 0.0;
    if (q0[0] * q1[0] + q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3] < 0.0) {
        q1 = {-q1[0], -q1[1], -q1[2], -q1[3]};
    }
    return qnorm({q0[0] + (q1[0] - q0[0]) * f, q0[1] + (q1[1] - q0[1]) * f, q0[2] + (q1[2] - q0[2]) * f,
                  q0[3] + (q1[3] - q0[3]) * f});
}

std::optional<Q> Fuser::gyro_delta(std::uint64_t t0, std::uint64_t t1) const {
    const auto c = active_calib();
    if (!c) {
        return std::nullopt;
    }
    const std::int64_t off = rs::f64_to_i64(c->time_offset_ms * 1e6);
    const auto a = q_at(static_cast<std::uint64_t>(std::max<std::int64_t>(rs::wadd(static_cast<std::int64_t>(t0), off), 0)));
    if (!a) {
        return std::nullopt;
    }
    const auto b = q_at(static_cast<std::uint64_t>(std::max<std::int64_t>(rs::wadd(static_cast<std::int64_t>(t1), off), 0)));
    if (!b) {
        return std::nullopt;
    }
    return qnorm(qmul(qconj(*a), *b));
}

V3 Fuser::shoulder(const P& head) const {
    V3 o = cfg.shoulder;
    if (hand == 1) {
        o[0] = -o[0];
    }
    return add(head.p, qrot(yaw_only(head.q), o));
}

const char* Fuser::implausible(V3 p, const P& head) const {
    if (norm(sub(p, shoulder(head))) > cfg.reach + 0.05) {
        return "reach";
    }
    const V3 local = qrot(qconj(yaw_only(head.q)), sub(p, head.p));
    if ((hand == 0 && local[0] > cfg.cross) || (hand == 1 && local[0] < -cfg.cross)) {
        return "side";
    }
    return nullptr;
}

V3 Fuser::constrain(V3 p, const P& head, std::optional<V3> other) const {
    const V3 sh = shoulder(head);
    const V3 d = sub(p, sh);
    const double n = norm(d);
    if (n > cfg.reach) {
        p = add(sh, scale(d, cfg.reach / n));
    }
    if (other) {
        const V3 o = *other;
        const V3 d2 = sub(p, o);
        const double n2 = norm(d2);
        if (n2 < cfg.min_sep) {
            const V3 dir = n2 > 1e-6 ? scale(d2, 1.0 / n2) : qrot(yaw_only(head.q), {hand == 0 ? -1.0 : 1.0, 0.0, 0.0});
            p = add(o, scale(dir, cfg.min_sep));
        }
    }
    return p;
}

std::optional<V3> Fuser::fresh_fix(std::uint64_t t, double max_age_s) const {
    if (!last_) {
        return std::nullopt;
    }
    const double age =
        static_cast<double>(rs::wabs(rs::wsub(static_cast<std::int64_t>(t), static_cast<std::int64_t>(last_->t)))) * 1e-9;
    if (age <= max_age_s) {
        return last_->w.p;
    }
    return std::nullopt;
}

std::optional<P> Fuser::hint_pose(std::uint64_t t) const {
    if (!last_ || !last_raw_) {
        return std::nullopt;
    }
    const auto dq = gyro_delta(last_->t, t);
    if (!dq) {
        return std::nullopt;
    }
    P p{qnorm(qmul(last_raw_->q, *dq)), last_raw_->p};
    if (cfg.ins_s > 0.0 && ins_.on && ins_.fix_t == last_->t) {
        const auto a = ins_pos(last_->t);
        const auto b = ins_pos(t);
        if (a && b) {
            p.p = add(p.p, sub(*b, *a));
        }
    }
    return p;
}

std::optional<Fuser::ImuState> Fuser::imu_state(std::uint64_t t) const {
    const auto c = active_calib();
    if (!c || q_ring_.empty()) {
        return std::nullopt;
    }
    const std::int64_t off = rs::f64_to_i64(c->time_offset_ms * 1e6);
    const auto q = q_at(static_cast<std::uint64_t>(std::max<std::int64_t>(rs::wadd(static_cast<std::int64_t>(t), off), 0)));
    if (!q) {
        return std::nullopt;
    }
    const double n = norm(acc_lp_);
    ImuState s{*q, n > 1e-6 ? scale(acc_lp_, 1.0 / n) : V3{0.0, 0.0, 0.0}, n > 0.9 * ACCEL_G && n < 1.1 * ACCEL_G,
               still_now_ ? static_cast<double>(rs::sat_sub(t, still_since_)) * 1e-9 : 0.0, calib_from_file_};
    return s;
}

double Fuser::last_fix_age(std::uint64_t t) const {
    return last_ ? static_cast<double>(rs::sat_sub(t, last_->t)) * 1e-9 : std::numeric_limits<double>::infinity();
}

std::optional<V3> Fuser::peek(std::uint64_t t, const P& head) {
    const auto [s, p] = predict(t, head);
    if (s != State::None) {
        return p.p;
    }
    return std::nullopt;
}

bool Fuser::gyro_still(std::uint64_t t) const {
    const std::uint64_t t0 = rs::sat_sub(t, 300000000u);
    std::size_t n = 0;
    double sum = 0.0;
    double maxraw = 0.0;
    for (auto it = raw_.rbegin(); it != raw_.rend(); ++it) {
        const auto& [ts, x] = *it;
        if (ts > t) {
            continue;
        }
        if (ts < t0) {
            break;
        }
        n += 1;
        if (calib) {
            sum += norm(calib->omega(x));
        } else {
            maxraw = rf::fmax(rf::fmax(rf::fmax(maxraw, std::fabs(x[0])), std::fabs(x[1])), std::fabs(x[2]));
        }
    }
    if (n < 50) {
        return false;
    }
    if (calib) {
        return sum / static_cast<double>(n) < 0.2;
    }
    return maxraw < 40.0;
}

double Fuser::gain(std::uint32_t leds, float rms_px, bool smooth) {
    double a = 0.85;
    if (leds <= 4 && smooth) {
        a = 0.12;
    } else if (leds <= 6) {
        a = 0.2;
    } else if (leds <= 9) {
        a = 0.5;
    }
    if (smooth) {
        const double r = rf::fmax(static_cast<double>(rms_px), 1e-3);
        const double w = rf::fmin(1.5 / r, 1.0);
        return a * rf::fmax(w * w, 0.3);
    }
    if (rms_px > 1.5f) {
        return a * 0.5;
    }
    return a;
}

P Fuser::filter_update(const Fix& f, std::uint64_t t, P w, std::uint32_t leds, float rms_px) {
    const double dt = static_cast<double>(t - f.t) * 1e-9;
    const bool still = gyro_still(t) && norm(vel_) < 0.1;
    double a = gain(leds, rms_px, cfg.hyst);
    if (still) {
        a *= 0.35;
    }
    double dt_v = dt;
    if (cfg.hyst && dt > 0.1) {
        const double tau = cfg.extrap_tau;
        dt_v = 0.1 + tau * (1.0 - std::exp(-(dt - 0.1) / tau));
    }
    const V3 pred_p = add(f.w.p, scale(vel_, dt_v));
    const auto dq = gyro_delta(f.t, t);
    const Q pred_q = dq ? qnorm(qmul(f.w.q, *dq)) : f.w.q;
    const V3 e = sub(w.p, pred_p);
    const V3 p = add(pred_p, scale(e, a));
    const double beta = 0.5 * a * a;
    const double vmax = cfg.max_vel;
    V3 v = add(vel_, scale(e, beta / rf::fmax(dt, 1e-3)));
    if (norm(v) > vmax) {
        v = scale(v, vmax / norm(v));
    }
    if (still) {
        v = scale(v, 0.5);
    }
    vel_ = v;
    const double k = dq ? 0.6 * a : a;
    const V3 corr = qlog(qmul(qconj(pred_q), w.q));
    const Q q = qnorm(qmul(pred_q, qexp(scale(corr, k))));
    return P{q, p};
}

std::pair<State, P> Fuser::predict(std::uint64_t t, const P& head) {
    if (!last_) {
        return {State::None, P::id()};
    }
    const Fix f = *last_;
    const double age = static_cast<double>(rs::sat_sub(t, f.t)) * 1e-9;
    const bool streaming = !raw_.empty() && rs::sat_sub(t, raw_.back().first) < 500000000u;
    if (age > cfg.lost_s && !(cfg.keep_tau_s > 0.0 && streaming)) {
        return {State::None, P::id()};
    }
    auto dq = gyro_delta(f.t, t);
    Q q = dq ? qnorm(qmul(f.w.q, *dq)) : f.w.q;
    if (cfg.keep_tau_s > 0.0 && qa_ && qa_->fix_t == f.t && t >= qa_->t) {
        if (const auto gq = gyro_world(t)) {
            q = *gq;
        }
    }
    if (cfg.tilt_fix && dq && tilt_t_ == f.t && age > cfg.extrap_s) {
        q = qnorm(qmul(tilt_, q));
    }
    const double tau = cfg.extrap_tau;
    const auto ex = [tau](double a) { return tau * (1.0 - std::exp(-a / tau)); };
    if (cfg.ins_s > 0.0 && ins_.on && ins_.fix_t == f.t && age <= cfg.ins_s) {
        // PiBridge: the fix moved on by the inertial filter's displacement since its frame time: the prediction to the
        // display time while tracked, the hand carried on out of view (the hold starts where it ends)
        const auto a = ins_pos(f.t);
        const auto b = ins_pos(t);
        if (a && b) {
            // the anchor: the display filter's position, or (raw_anchor) the fix as measured: the inertial filter
            // already smooths it, and the display filter lags a hand that stops hard
            const V3 base = cfg.raw_anchor && last_raw_ ? last_raw_->p : f.w.p;
            const V3 p = add(base, sub(*b, *a));
            hold_ = std::nullopt;
            if (age <= cfg.extrap_s) {
                return {State::Optical, P{q, p}};
            }
            ins_.handover = p;
            return {State::Hold, P{q, p}};
        }
    }
    if (age <= cfg.extrap_s) {
        const V3 p = add(f.w.p, scale(vel_, ex(age)));
        hold_ = std::nullopt;
        return {State::Optical, P{q, p}};
    }
    const P yaw_now{yaw_only(head.q), head.p};
    if (!hold_yaw_) {
        hold_yaw_ = yaw_now;
    }
    const P y0 = *hold_yaw_;
    P yaw;
    if (!hold_) {
        hold_yaw_ = yaw_now;
        yaw = yaw_now;
    } else {
        const V3 dyaw = qlog(qmul(qconj(y0.q), yaw_now.q));
        yaw = P{qnorm(qmul(y0.q, qexp(scale(dyaw, cfg.follow_yaw)))), add(y0.p, scale(sub(yaw_now.p, y0.p), cfg.follow_pos))};
    }
    const bool want_world = cfg.still_lock && gyro_still(t);
    std::optional<V3> arm;
    if (dq && cfg.forearm) {
        V3 fa = *cfg.forearm;
        if (hand == 1) {
            fa[0] = -fa[0];
        }
        arm = fa;
    }
    if (!hold_) {
        V3 p_end = add(f.w.p, scale(vel_, ex(cfg.extrap_s)));
        if (arm) {
            const std::uint64_t t_end = f.t + rf::f64_to_u64(cfg.extrap_s * 1e9);
            const auto d = gyro_delta(f.t, std::min(t_end, t));
            const Q q_end = d ? qnorm(qmul(f.w.q, *d)) : q;
            p_end = sub(p_end, qrot(q_end, *arm));
        }
        if (cfg.ins_s > 0.0 && ins_.handover && ins_.fix_t == f.t) {
            // PiBridge: from where the inertial position left the hand
            p_end = arm ? sub(*ins_.handover, qrot(q, *arm)) : *ins_.handover;
        }
        const P end{q, p_end};
        if (want_world) {
            hold_ = std::pair<P, bool>{end, true};
        } else {
            hold_ = std::pair<P, bool>{yaw.inv().mul(end), false};
        }
    } else if (hold_->second != want_world) {
        const auto [off, world] = *hold_;
        const P now = world ? off : yaw.mul(off);
        if (want_world) {
            hold_ = std::pair<P, bool>{now, true};
        } else {
            hold_ = std::pair<P, bool>{yaw.inv().mul(now), false};
        }
    }
    const auto [off, world] = *hold_;
    const P held = world ? off : yaw.mul(off);
    const Q qq = dq ? q : held.q;
    V3 p = arm ? add(held.p, qrot(qq, *arm)) : held.p;
    if (!world) {
        if (const auto d = dead_reckon(f, t)) {
            p = add(p, *d);
        }
    }
    if (cfg.keep_tau_s > 0.0 && age > cfg.lost_s) {
        // PiBridge: kept past lost_s, gliding to the rest pose instead of jumping there
        V3 r = cfg.rest_pos;
        if (hand == 0) {
            r[0] = -r[0];
        }
        const V3 rest = add(head.p, qrot(yaw_only(head.q), r));
        const double w = 1.0 - std::exp(-(age - cfg.lost_s) / cfg.keep_tau_s);
        p = add(p, scale(sub(rest, p), w));
    }
    return {State::Hold, P{qq, p}};
}

std::optional<V3> Fuser::dead_reckon(const Fix& f, std::uint64_t t) {
    const double tau = cfg.dr_tau_s;
    if (tau <= 0.0) {
        return std::nullopt;
    }
    const auto c = active_calib();
    if (!c) {
        return std::nullopt;
    }
    const std::int64_t off = rs::f64_to_i64(c->time_offset_ms * 1e6);
    const auto q_at_ts = [&](std::uint64_t ts) -> std::optional<Q> {
        const auto a = q_at(static_cast<std::uint64_t>(std::max<std::int64_t>(rs::wadd(static_cast<std::int64_t>(f.t), off), 0)));
        if (!a) {
            return std::nullopt;
        }
        const auto b = q_at(static_cast<std::uint64_t>(std::max<std::int64_t>(rs::wadd(static_cast<std::int64_t>(ts), off), 0)));
        if (!b) {
            return std::nullopt;
        }
        return qnorm(qmul(f.w.q, qmul(qconj(*a), *b)));
    };
    if (!dr_ || dr_->fix_t != f.t) {
        V3 g = {0.0, 0.0, 0.0};
        double n = 0.0;
        for (const auto& [ts, a] : acc_) {
            if (ts + 500000000u < f.t || ts > f.t) {
                continue;
            }
            if (const auto q = q_at_ts(ts)) {
                g = add(g, qrot(*q, c->accel(a)));
                n += 1.0;
            }
        }
        if (n < 50.0) {
            return std::nullopt;
        }
        dr_ = Dr{f.t, scale(g, 1.0 / n), f.t, vel_, {0.0, 0.0, 0.0}, std::nullopt};
    }
    Dr dr = *dr_;
    std::vector<std::pair<std::uint64_t, V3>> samples;
    for (const auto& e : acc_) {
        if (e.first > dr.t && e.first <= t) {
            samples.push_back(e);
        }
    }
    for (const auto& [ts, a] : samples) {
        const double dt = static_cast<double>(ts - dr.t) * 1e-9;
        if (const auto q = q_at_ts(ts)) {
            const V3 aw = sub(qrot(*q, c->accel(a)), dr.g);
            dr.v = add(scale(dr.v, std::exp(-dt / tau)), scale(aw, dt));
        }
        dr.d = add(dr.d, scale(dr.v, dt));
        const double n = norm(dr.d);
        if (n > cfg.dr_max_m) {
            dr.d = scale(dr.d, cfg.dr_max_m / n);
        }
        dr.t = ts;
    }
    if (!dr.d_anchor) {
        dr.d_anchor = dr.d;
    }
    const V3 base = *dr.d_anchor;
    dr_ = dr;
    return sub(dr.d, base);
}

double Fuser::blend_weight(double a, double dur) const {
    const double x = rf::clamp(a / rf::fmax(dur, 1e-6), 0.0, 1.0);
    if (cfg.blend_ease) {
        return 1.0 - x * x * (3.0 - 2.0 * x);
    }
    return 1.0 - x;
}

void Fuser::write_hint(std::uint64_t t, const P& head, State s, const P& /*shown*/) {
    if (t < hint_t_ + 4000000u) {
        return;
    }
    hint_t_ = t;
    // one writer per process for both hands, opened on first use
    static std::mutex mu;
    static hint::Writer* w = nullptr;
    static bool tried = false;
    {
        std::lock_guard lk(mu);
        if (!tried) {
            tried = true;
            w = hint::Writer::open(cfg.hint_path);
        }
    }
    if (w == nullptr) {
        return;
    }
    hint::Entry e;
    e.t_ns = t;
    e.fix_ns = last_ ? last_->t : 0;
    e.state = static_cast<std::uint32_t>(s);
    const auto c = active_calib();
    if (const auto im = imu_state(t)) {
        for (std::size_t i = 0; i < 4; i++) {
            e.qg[i] = static_cast<float>(im->q[i]);
        }
        for (std::size_t i = 0; i < 3; i++) {
            e.up_aim[i] = static_cast<float>(im->up[i]);
        }
        e.still_s = static_cast<float>(im->still_s);
        e.flags = (im->calibrated ? 1u : 0u) | (im->up_ok ? 2u : 0u);
    } else {
        e.flags = 0;
    }
    if (hint_acc_n_ > 0) {
        const V3 fa = scale(hint_acc_, 1.0 / static_cast<double>(hint_acc_n_));
        for (std::size_t i = 0; i < 3; i++) {
            e.f_aim[i] = static_cast<float>(fa[i]);
        }
        hint_acc_ = {0.0, 0.0, 0.0};
        hint_acc_n_ = 0;
    }
    for (std::size_t i = 0; i < 3; i++) {
        e.head_pos[i] = static_cast<float>(head.p[i]);
    }
    for (std::size_t i = 0; i < 4; i++) {
        e.head_quat[i] = static_cast<float>(head.q[i]);
    }
    const V3 uh = qrot(qconj(head.q), {0.0, 1.0, 0.0});
    for (std::size_t i = 0; i < 3; i++) {
        e.up_head[i] = static_cast<float>(uh[i]);
    }
    const auto hp = s != State::None ? hint_pose(t) : std::nullopt;
    if (!hp) {
        e.state = 0;
    } else {
        e.flags |= 4u;
        const P rel = head.inv().mul(*hp);
        for (std::size_t i = 0; i < 3; i++) {
            e.pos[i] = static_cast<float>(rel.p[i]);
        }
        for (std::size_t i = 0; i < 4; i++) {
            e.quat[i] = static_cast<float>(rel.q[i]);
        }
        if (c && !raw_.empty()) {
            const V3 om = c->omega(raw_.back().second);
            for (std::size_t i = 0; i < 3; i++) {
                e.w[i] = static_cast<float>(om[i]);
            }
        }
    }
    w->write(hand, e);
}

std::pair<State, P> Fuser::get_with(std::uint64_t t, const P& head, std::optional<V3> other) {
    auto r = get_inner(t, head, other);
    if (!cfg.hint_path.empty()) {
        write_hint(t, head, r.first, r.second);
    }
    return r;
}

std::pair<State, P> Fuser::get_inner(std::uint64_t t, const P& head, std::optional<V3> other) {
    auto [s, p] = predict(t, head);
    if (s == State::None) {
        return {s, p};
    }
    const double age = last_ ? static_cast<double>(rs::sat_sub(t, last_->t)) * 1e-9 : 0.0;
    if (s == State::Hold || age > 0.05) {
        p.p = constrain(p.p, head, other);
    }
    last_query_t_ = std::max(last_query_t_, t);
    if (blend_) {
        if (!blend_->t0) {
            blend_->t0 = t;
        }
        const std::uint64_t t0 = *blend_->t0;
        const Blend b = *blend_;
        const double a = static_cast<double>(rs::sat_sub(t, t0)) * 1e-9;
        if (a >= b.dur) {
            blend_ = std::nullopt;
        } else {
            const double w = blend_weight(a, b.dur);
            p.p = add(p.p, scale(b.dp, w));
            p.q = qnorm(qmul(qexp(scale(qlog(b.dq), w)), p.q));
        }
    }
    if (log_ != nullptr && t >= last_p_log_ + 20000000u) {
        last_p_log_ = t;
        const V3 o = other ? *other : V3{NAN, NAN, NAN};
        const auto f5 = [](double v) { return rf::fixed(v, 5); };
        const auto f6 = [](double v) { return rf::fixed(v, 6); };
        const auto f4 = [](double v) { return rf::fixed(v, 4); };
        log_line(t, "P," + std::to_string(t) + "," + std::to_string(static_cast<std::uint32_t>(s)) + "," + f5(p.p[0]) +
                        "," + f5(p.p[1]) + "," + f5(p.p[2]) + "," + f6(p.q[0]) + "," + f6(p.q[1]) + "," + f6(p.q[2]) +
                        "," + f6(p.q[3]) + "," + f5(head.p[0]) + "," + f5(head.p[1]) + "," + f5(head.p[2]) + "," +
                        f6(head.q[0]) + "," + f6(head.q[1]) + "," + f6(head.q[2]) + "," + f6(head.q[3]) + "," +
                        f4(o[0]) + "," + f4(o[1]) + "," + f4(o[2]));
    }
    return {s, p};
}

const char* Fuser::optical_with(std::uint64_t t, P w, const P& head, float rms_px, std::uint32_t leds,
                                std::optional<V3> other) {
    last_leds_ = leds;
    const char* verdict = judge(t, w, head, other);
    const std::string_view v(verdict);
    if (log_ != nullptr) {
        const auto f5 = [](double x) { return rf::fixed(x, 5); };
        const auto f6 = [](double x) { return rf::fixed(x, 6); };
        log_line(t, "O," + std::to_string(t) + "," + f5(w.p[0]) + "," + f5(w.p[1]) + "," + f5(w.p[2]) + "," +
                        f6(w.q[0]) + "," + f6(w.q[1]) + "," + f6(w.q[2]) + "," + f6(w.q[3]) + "," + f5(head.p[0]) +
                        "," + f5(head.p[1]) + "," + f5(head.p[2]) + "," + f6(head.q[0]) + "," + f6(head.q[1]) + "," +
                        f6(head.q[2]) + "," + f6(head.q[3]) + "," + rf::fixed(static_cast<double>(rms_px), 3) + "," +
                        std::to_string(leds) + "," + verdict);
    }
    if ((v == "ok" || v == "pos-only") && needs_confirm(t, w)) {
        return "confirm";
    }
    if (v == "ok") {
        accept(t, w, head, leds, rms_px, false);
    } else if (v == "pos-only") {
        const auto pred = predict(t, head).second;
        accept(t, P{pred.q, w.p}, head, leds, rms_px, true);
    }
    return verdict;
}

bool Fuser::needs_confirm(std::uint64_t t, const P& w) {
    if (cfg.confirm_n <= 1) {
        return false;
    }
    if (!last_) {
        return false;
    }
    const double age = static_cast<double>(rs::sat_sub(t, last_->t)) * 1e-9;
    if (age <= cfg.gap_s || age > cfg.lost_s) {
        reentry_ = std::nullopt;
        return false;
    }
    std::uint32_t n = 1;
    if (reentry_ && t > reentry_->t && static_cast<double>(t - reentry_->t) * 1e-9 <= cfg.confirm_s &&
        norm(sub(w.p, reentry_->p)) <= cfg.confirm_m) {
        n = reentry_->n + 1;
    }
    if (n >= cfg.confirm_n) {
        reentry_ = std::nullopt;
        return false;
    }
    reentry_ = Reentry{t, w.p, n};
    return true;
}

const char* Fuser::judge(std::uint64_t t, const P& w, const P& head, std::optional<V3> other) {
    if (!raw_.empty()) {
        const std::uint64_t t_imu = raw_.back().first;
        if (t + 2000000000u < t_imu) {
            stats.rejected_body += 1;
            return "stale";
        }
    }
    if (last_ && t <= last_->t) {
        return "old";
    }
    if (const char* r = implausible(w.p, head)) {
        stats.rejected_body += 1;
        return r;
    }
    if (other && norm(sub(*other, w.p)) < cfg.min_sep) {
        stats.rejected_body += 1;
        return "merged";
    }
    if (cfg.grav_check_deg > 0.0 && !grav_off_ && calib_from_file_) {
        if (const auto a = grav_angle(t, w.q)) {
            const bool bad = *a > cfg.grav_check_deg;
            grav_n_ += 1;
            grav_bad_ += bad ? 1 : 0;
            if (grav_n_ >= 60) {
                // most fixes disagreeing means the axis calibration is wrong, not the fixes
                if (grav_bad_ * 2 > grav_n_) {
                    grav_off_ = true;
                }
                grav_n_ = 0;
                grav_bad_ = 0;
            }
            if (bad && !grav_off_) {
                stats.rejected_tilt += 1;
                return "tilt";
            }
        }
    }
    if (!last_) {
        return "ok";
    }
    const Fix f = *last_;
    const double age = static_cast<double>(rs::sat_sub(t, f.t)) * 1e-9;
    const auto [s, pred] = predict(t, head);
    if (s == State::None) {
        rejects_ = 0;
        return "ok";
    }
    std::string_view why = "ok";
    if (gyro_delta(f.t, t) && age < cfg.lost_s && rf::to_degrees(qangle(pred.q, w.q)) > cfg.max_jump_deg) {
        why = "turn";
    }
    if (age <= cfg.extrap_s + 0.05 && norm(sub(pred.p, w.p)) > cfg.max_jump_m + 3.0 * age) {
        why = "jump";
    }
    // PiBridge: the inertial prediction is good to a few mm over a few frames, so a fix far from it is the other hand
    // or a wrong LED identity even when it is within the wide optical window above
    if (cfg.ins_s > 0.0 && cfg.ins_jump_m > 0.0 && ins_.on && ins_.fix_t == f.t && age <= 0.2 &&
        norm(sub(pred.p, w.p)) > cfg.ins_jump_m + 1.0 * age) {
        why = "jump";
    }
    if (why == "turn" && last_leds_ < 9) {
        stats.rejected += 1;
        // PiBridge (turn_drop): a fix that turned against the gyro has the wrong LEDs (a mirror fit), so its position is
        // wrong too: dropped, as the other rejects (and accepted after the same streak)
        if (cfg.turn_drop && rejects_ < 2) {
            rejects_ += 1;
            return "turn";
        }
        if (!cfg.turn_drop) {
            return "pos-only";
        }
    }
    const std::uint32_t limit = why == "jump" ? 3 : 2;
    if (why != "ok" && rejects_ < limit) {
        rejects_ += 1;
        stats.rejected += 1;
        return why == "jump" ? "jump" : "turn";
    }
    rejects_ = 0;
    return "ok";
}

void Fuser::accept(std::uint64_t t, P w, const P& head, std::uint32_t leds, float rms_px, bool pos_only) {
    const std::optional<Fix> prev = last_;
    if (prev && t <= prev->t) {
        return;
    }
    std::optional<double> age;
    if (prev) {
        age = static_cast<double>(rs::sat_sub(t, prev->t)) * 1e-9;
    }
    const bool gap = !age || *age > cfg.gap_s;
    if (gap && prev) {
        const std::uint64_t qt = std::max(last_query_t_, t);
        auto [s, shown] = predict(qt, head);
        if (blend_ && blend_->t0) {
            const double a = static_cast<double>(rs::sat_sub(qt, *blend_->t0)) * 1e-9;
            if (a < blend_->dur) {
                const double wt = blend_weight(a, blend_->dur);
                shown.p = add(shown.p, scale(blend_->dp, wt));
            }
        }
        if (s != State::None) {
            const V3 dp = sub(shown.p, w.p);
            const Q dq = qnorm(qmul(shown.q, qconj(w.q)));
            double dur = cfg.blend_s;
            if (cfg.blend_mps > 0.0) {
                dur = rf::fmax(dur, norm(dp) / cfg.blend_mps);
            }
            if (cfg.blend_dps > 0.0) {
                dur = rf::fmax(dur, rf::to_degrees(norm(qlog(dq))) / cfg.blend_dps);
            }
            dur = rf::fmin(dur, rf::fmax(cfg.blend_max_s, cfg.blend_s));
            blend_ = Blend{std::nullopt, dur, dp, dq};
            stats.reacquires += 1;
        }
    }
    if (gap) {
        seg_ += 1;
    }
    const P raw_w = w;
    if (cfg.filter && prev && !gap) {
        w = filter_update(*prev, t, w, leds, rms_px);
    }
    if (cfg.filter && !gap) {
        // the filter keeps its own velocity
    } else if (prev && age && *age <= 0.1 && *age > 0.0) {
        V3 v = scale(sub(w.p, prev->w.p), 1.0 / *age);
        const double vmax = cfg.max_vel;
        if (norm(v) > vmax) {
            v = scale(v, vmax / norm(v));
        }
        vel_ = add(scale(vel_, 0.7), scale(v, 0.3));
        if (norm(vel_) > vmax) {
            vel_ = scale(vel_, vmax / norm(vel_));
        }
    } else {
        vel_ = {0.0, 0.0, 0.0};
    }
    if (cfg.ins_s > 0.0) {
        ins_fix(t, raw_w.p, leds, gap);
    }
    last_raw_ = raw_w;
    const Fix fix{t, w, leds, seg_};
    Fix raw_fix = fix;
    raw_fix.w = raw_w;
    if (!pos_only) {
        fit_pair(raw_fix);
    }
    last_ = fix;
    hold_ = std::nullopt;
    reentry_ = std::nullopt;
    stats.fixes += 1;
    if (!pos_only) {
        hist_.push_back(raw_fix);
    }
    while (hist_.size() > 16) {
        hist_.pop_front();
    }
}

void Fuser::fit_pair(const Fix& b) {
    if (b.leds < 9) {
        return;
    }
    const Fix* best = nullptr;
    std::int64_t best_key = 0;
    for (const Fix& a : hist_) {
        if (!(a.seg == b.seg && a.leds >= 9)) {
            continue;
        }
        const double dt = static_cast<double>(rs::sat_sub(b.t, a.t)) * 1e-9;
        if (!(0.08 <= dt && dt <= 0.2)) {
            continue;
        }
        const std::int64_t key = rs::wabs(rs::wsub(static_cast<std::int64_t>(rs::sat_sub(b.t, a.t)), 120000000));
        if (best == nullptr || key < best_key) { // min_by_key: the first of equal minima
            best = &a;
            best_key = key;
        }
    }
    if (best == nullptr) {
        return;
    }
    pending_.emplace_back(*best, b);
    if (pending_.size() > 64) {
        pending_.pop_front();
    }
}

void Fuser::process_pending() {
    if (raw_.empty()) {
        return;
    }
    const std::uint64_t t_last = raw_.back().first;
    const std::uint64_t max_off = rf::f64_to_u64(FIT_OFFSETS_MS[N_OFF - 1] * 1e6) + 10000000u;
    while (!pending_.empty()) {
        const auto [a, b] = pending_.front();
        if (b.t + max_off > t_last) {
            break;
        }
        pending_.pop_front();
        fit_eval(a, b);
    }
}

void Fuser::fit_eval(Fix a, Fix b) {
    const double dt = static_cast<double>(b.t - a.t) * 1e-9;
    const V3 y = scale(qlog(qmul(qconj(a.w.q), b.w.q)), 1.0 / dt);
    if (norm(y) > 20.0) {
        return;
    }
    std::array<V3, N_OFF> xs{};
    for (std::size_t k = 0; k < N_OFF; k++) {
        const std::int64_t o = rs::f64_to_i64(FIT_OFFSETS_MS[k] * 1e6);
        const auto t0 = static_cast<std::uint64_t>(std::max<std::int64_t>(rs::wadd(static_cast<std::int64_t>(a.t), o), 0));
        const auto t1 = static_cast<std::uint64_t>(std::max<std::int64_t>(rs::wadd(static_cast<std::int64_t>(b.t), o), 0));
        V3 sum = {0.0, 0.0, 0.0};
        std::size_t cnt = 0;
        for (const auto& [t, x] : raw_) {
            if (t >= t0 && t < t1) {
                sum = add(sum, x);
                cnt++;
            }
        }
        if (static_cast<double>(cnt) < dt * 400.0 * 0.6) {
            return;
        }
        xs[k] = scale(sum, 1.0 / static_cast<double>(cnt));
    }
    fit.add(xs, y);
    const std::uint32_t n = fit.npairs();
    if (cfg.online_fit && n % 25 == 0) {
        const std::uint32_t need = calib_from_file_ ? 300 : 120;
        if (const auto c = fit.best(need, 40, 0.8)) {
            if (fit_replaces(calib ? std::optional<double>(calib->r2) : std::nullopt, c->r2)) {
                calib = c;
                calib_from_file_ = false;
                fit_dirty_ = true;
                stats.fit_updates += 1;
            }
        }
    }
}

std::optional<bool> Fuser::maybe_save(std::uint64_t now, const std::string& path, double min_interval_s) {
    if (!fit_dirty_ || rs::sat_sub(now, last_save_) < rf::f64_to_u64(min_interval_s * 1e9)) {
        return std::nullopt;
    }
    if (!calib) {
        return std::nullopt;
    }
    const Calib c = *calib;
    fit_dirty_ = false;
    last_save_ = now;
    return save_calib(path, hand, c);
}

} // namespace pibridge::ctrl::fuse
