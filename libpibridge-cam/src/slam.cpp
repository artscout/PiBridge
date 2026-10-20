// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/slam.rs (MIT OR Apache-2.0).
#include "pibridge/cam/slam.h"

#include "pibridge/cam/io.h"
#include "pibridge/cam/json.h"
#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <cmath>
#include <format>

namespace pibridge::cam::slam {

namespace rf = pibridge::cam::rfmt;

Quat qmul(const Quat& a, const Quat& b) {
    const auto [ax, ay, az, aw] = a;
    const auto [bx, by, bz, bw] = b;
    return {
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    };
}

Quat qconj(const Quat& q) {
    return {-q[0], -q[1], -q[2], q[3]};
}

Quat qnorm(const Quat& q) {
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n < 1e-12 || !std::isfinite(n)) {
        return QUAT_ID;
    }
    return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

Vec3 qrot(const Quat& q, const Vec3& v) {
    const Vec3 u{q[0], q[1], q[2]};
    const Vec3 uv = geom::cross(u, v);
    const Vec3 uuv = geom::cross(u, uv);
    return geom::add(v, geom::add(geom::scale(uv, 2.0 * q[3]), geom::scale(uuv, 2.0)));
}

double qangle_deg(const Quat& q0) {
    const Quat q = qnorm(q0);
    return rf::to_degrees(2.0 * std::acos(rf::fmin(std::fabs(q[3]), 1.0)));
}

Quat qyaw(double rad) {
    return {0.0, std::sin(rad / 2.0), 0.0, std::cos(rad / 2.0)};
}

double yaw_of(const Quat& q) {
    const Vec3 d = qrot(q, {0.0, 0.0, -1.0});
    if (std::fabs(d[0]) < 1e-9 && std::fabs(d[2]) < 1e-9) {
        // Looking straight up/down: use the up vector's tilt direction instead.
        const Vec3 u = qrot(q, {0.0, 1.0, 0.0});
        return std::atan2(-u[0], -u[2]);
    }
    return std::atan2(-d[0], -d[2]);
}

Quat quat_of(const Mat3& m) {
    return geom::quat_from_mat(m);
}

Mat3 mat_of(const Quat& q) {
    const Vec3 c0 = qrot(q, {1.0, 0.0, 0.0});
    const Vec3 c1 = qrot(q, {0.0, 1.0, 0.0});
    const Vec3 c2 = qrot(q, {0.0, 0.0, 1.0});
    return geom::from_cols(c0, c1, c2);
}

std::optional<Pose> t_imu_cam(const DeviceCalib& cal, std::size_t i) {
    if (i >= cal.cameras.size()) {
        return std::nullopt;
    }
    return cal.body_from_cam(i, RigConvention::CamFromCam0);
}

namespace {

Vec3 vec3_of(const json::Value* v, const Vec3& d) {
    if (v == nullptr) {
        return d;
    }
    if (v->type == json::Value::Type::Num) {
        return {v->num, v->num, v->num};
    }
    return v->as_f64_array<3>().value_or(d);
}

// "{path}: {e}"
std::string path_err(const std::string& p, const std::string& e) {
    return p + ": " + e;
}

} // namespace

std::optional<std::string> ImuNoise::default_path() {
    if (auto v = env_os("PIMAX_SLAM_IMU"); v && !v->empty()) {
        return *v;
    }
    if (auto h = env_os("HOME")) {
        return path_join(*h, ".config/omarchyvr/slam-imu.json");
    }
    return std::nullopt;
}

std::expected<ImuNoise, std::string> ImuNoise::parse(std::string_view text) {
    auto v = json::parse(text);
    if (!v) {
        return std::unexpected(v.error().text());
    }
    const ImuNoise d;
    ImuNoise n;
    n.accel_noise_std = vec3_of(v->get("accel_noise_std"), d.accel_noise_std);
    n.gyro_noise_std = vec3_of(v->get("gyro_noise_std"), d.gyro_noise_std);
    n.accel_bias_std = vec3_of(v->get("accel_bias_std"), d.accel_bias_std);
    n.gyro_bias_std = vec3_of(v->get("gyro_bias_std"), d.gyro_bias_std);
    const json::Value* r = v->get("imu_update_rate");
    n.rate_hz = (r != nullptr ? r->as_f64() : std::nullopt).value_or(d.rate_hz);
    n.source.clear();
    bool bad = false;
    for (const Vec3* a : {&n.accel_noise_std, &n.gyro_noise_std, &n.accel_bias_std, &n.gyro_bias_std}) {
        for (const double x : *a) {
            if (!std::isfinite(x) || x <= 0.0) {
                bad = true;
            }
        }
    }
    if (bad || std::isnan(n.rate_hz) || n.rate_hz <= 0.0) {
        return std::unexpected(std::string("IMU noise values must be finite and > 0"));
    }
    return n;
}

std::expected<std::optional<ImuNoise>, std::string> ImuNoise::load(const std::optional<std::string>& path) {
    std::optional<std::string> pp = path ? path : default_path();
    if (!pp) {
        return std::optional<ImuNoise>{};
    }
    const std::string& p = *pp;
    if (!path && !is_file(p)) {
        return std::optional<ImuNoise>{};
    }
    auto s = read_to_string(p);
    if (!s) {
        return std::unexpected(path_err(p, s.error().text()));
    }
    auto n = parse(*s);
    if (!n) {
        return std::unexpected(path_err(p, n.error()));
    }
    n->source = rf::utf8_lossy(p);
    return std::optional<ImuNoise>(std::move(*n));
}

std::string ImuNoise::to_json(std::string_view comment) const {
    return std::format(
        "{{\n  \"comment\": {},\n  \"imu_update_rate\": {},\n  \"accel_noise_std\": {},\n  \"gyro_noise_std\": {},\n"
        "  \"accel_bias_std\": {},\n  \"gyro_bias_std\": {}\n}}\n",
        json::quote(comment), json::num(rate_hz), json::num_array(accel_noise_std), json::num_array(gyro_noise_std),
        json::num_array(accel_bias_std), json::num_array(gyro_bias_std));
}

ImuNoise ImuNoise::for_regime(NoiseRegime r) const {
    if (r == NoiseRegime::Still) {
        return *this;
    }
    ImuNoise m = *this;
    for (int k = 0; k < 3; k++) {
        m.accel_noise_std[k] = accel_noise_std[k] * MOVING_ACCEL_RATIO;
        m.gyro_noise_std[k] = gyro_noise_std[k] * MOVING_GYRO_RATIO;
    }
    return m;
}

NoiseRegime NoiseSelector::update(double gyro_max) {
    if (regime_ == NoiseRegime::Still && gyro_max > on) {
        regime_ = NoiseRegime::Moving;
    } else if (regime_ == NoiseRegime::Moving && gyro_max < off) {
        regime_ = NoiseRegime::Still;
    }
    return regime_;
}

std::optional<Vec3> factory_accel_bias(const DeviceCalib& cal) {
    if (!cal.imu) {
        return std::nullopt;
    }
    const auto& raw = cal.imu->raw;
    const auto it = std::find_if(raw.begin(), raw.end(), [](const auto& kv) { return kv.first == "aBias"; });
    if (it == raw.end()) {
        return std::nullopt;
    }
    std::vector<double> x;
    for (const auto w : rf::split_whitespace(it->second)) {
        double v = 0;
        if (!rf::parse_f64(w, v)) {
            return std::nullopt;
        }
        x.push_back(v);
    }
    if (x.size() != 3) {
        return std::nullopt;
    }
    for (const double a : x) {
        if (!std::isfinite(a) || std::fabs(a) > 1.0) {
            return std::nullopt;
        }
    }
    return Vec3{x[0], x[1], x[2]};
}

std::optional<std::string> gyro_bias_cache_path() {
    if (auto x = env_os("XDG_CACHE_HOME"); x && !x->empty()) {
        return path_join(*x, "omarchyvr/gyro-bias");
    }
    if (auto h = env_os("HOME")) {
        return path_join(*h, ".cache/omarchyvr/gyro-bias");
    }
    return std::nullopt;
}

std::optional<Vec3> parse_gyro_bias_cache(std::string_view text) {
    std::vector<double> v;
    for (const auto w : rf::split_whitespace(text)) {
        double x = 0;
        if (!rf::parse_f64(w, x)) {
            return std::nullopt;
        }
        v.push_back(x);
    }
    if (v.size() != 3) {
        return std::nullopt;
    }
    for (const double x : v) {
        if (!std::isfinite(x) || std::fabs(x) > 5.0) {
            return std::nullopt;
        }
    }
    return Vec3{rf::to_radians(-v[0]), rf::to_radians(v[1]), rf::to_radians(-v[2])};
}

std::optional<Vec3> gyro_bias_cache() {
    const auto p = gyro_bias_cache_path();
    if (!p) {
        return std::nullopt;
    }
    auto t = read_to_string(*p);
    if (!t) {
        return std::nullopt;
    }
    return parse_gyro_bias_cache(*t);
}

std::optional<std::string> slam_config_calib_path() {
    const auto cfg = env_os("SLAM_CONFIG");
    if (!cfg || cfg->empty()) {
        return std::nullopt;
    }
    const auto text = read_to_string(*cfg);
    if (!text) {
        return std::nullopt;
    }
    std::string_view t = *text;
    while (!t.empty()) {
        const std::size_t nl = t.find('\n');
        std::string_view ln = rf::trim(t.substr(0, nl));
        t = nl == std::string_view::npos ? std::string_view() : t.substr(nl + 1);
        if (!ln.starts_with("cam-calib")) {
            continue;
        }
        const std::size_t eq = ln.find('=');
        if (eq == std::string_view::npos || rf::trim(ln.substr(0, eq)) != "cam-calib") {
            continue;
        }
        std::string_view v = rf::trim(ln.substr(eq + 1));
        if (v.size() >= 2 && (v.front() == '"' || v.front() == '\'') && v.back() == v.front()) {
            v = v.substr(1, v.size() - 2);
        }
        if (v.empty()) {
            return std::nullopt;
        }
        std::string path(v);
        if (path.front() != '/') {
            const std::size_t slash = cfg->rfind('/');
            path = slash == std::string::npos ? path : cfg->substr(0, slash + 1) + path;
        }
        return path;
    }
    return std::nullopt;
}

CalibBiases calib_json_biases(std::string_view json_text) {
    CalibBiases b;
    const auto doc = json::parse(json_text);
    if (!doc) {
        return b;
    }
    const json::Value* v0 = doc->get("value0");
    if (v0 == nullptr) {
        return b;
    }
    auto three = [&](const char* key, double lim) -> std::optional<Vec3> {
        const json::Value* a = v0->get(key);
        if (a == nullptr || a->as_array() == nullptr || a->as_array()->size() < 3) {
            return std::nullopt;
        }
        Vec3 x{};
        for (std::size_t k = 0; k < 3; k++) {
            const auto f = (*a->as_array())[k].as_f64();
            if (!f || !std::isfinite(*f) || std::fabs(*f) > lim) {
                return std::nullopt;
            }
            x[k] = *f;
        }
        return x;
    };
    b.accel = three("calib_accel_bias", 1.0);
    b.gyro = three("calib_gyro_bias", rf::to_radians(5.0));
    return b;
}

std::expected<std::vector<std::size_t>, std::string> parse_cams(std::string_view s, std::size_t count) {
    std::vector<std::size_t> v;
    std::size_t start = 0;
    for (;;) {
        const std::size_t comma = s.find(',', start);
        const std::string_view part = s.substr(start, comma == std::string_view::npos ? s.npos : comma - start);
        const std::string_view p = rf::trim(part);
        if (!p.empty()) {
            std::uint64_t i = 0;
            if (!rf::parse_u64(p, i)) {
                return std::unexpected(
                    std::format("camera list {}: bad id {}", rf::debug_str(s), rf::debug_str(p)));
            }
            if (i >= count || std::find(v.begin(), v.end(), i) != v.end()) {
                return std::unexpected(
                    std::format("camera list {}: ids must be distinct and < {}", rf::debug_str(s), count));
            }
            v.push_back(static_cast<std::size_t>(i));
        }
        if (comma == std::string_view::npos) {
            break;
        }
        start = comma + 1;
    }
    if (v.empty()) {
        return std::unexpected(std::format("camera list {} is empty", rf::debug_str(s)));
    }
    return v;
}

namespace {

std::string pose_json(const Pose& p, std::string_view indent) {
    const auto q = geom::quat_from_mat(p.r);
    return std::format(
        "{0}{{\n{0}    \"px\": {1},\n{0}    \"py\": {2},\n{0}    \"pz\": {3},\n{0}    \"qx\": {4},\n{0}    \"qy\": {5},\n"
        "{0}    \"qz\": {6},\n{0}    \"qw\": {7}\n{0}}}",
        indent, json::num(p.t[0]), json::num(p.t[1]), json::num(p.t[2]), json::num(q[0]), json::num(q[1]),
        json::num(q[2]), json::num(q[3]));
}

std::string join(const std::vector<std::string>& v, std::string_view sep) {
    std::string o;
    for (std::size_t i = 0; i < v.size(); i++) {
        if (i > 0) {
            o += sep;
        }
        o += v[i];
    }
    return o;
}

std::string_view trim_start(std::string_view s) {
    // str::trim_start (ASCII spaces are all the indentation has)
    const std::string_view t = rf::trim(s);
    if (t.empty()) {
        return t;
    }
    return s.substr(static_cast<std::size_t>(t.data() - s.data()));
}

} // namespace

std::expected<std::string, std::string> basalt_calib_json(const DeviceCalib& cal, const SlamSetup& setup,
                                                          const ImuNoise& imu) {
    std::vector<std::string> tic;
    std::vector<std::string> intr;
    std::vector<std::string> res;
    for (const std::size_t i : setup.cams) {
        auto p = t_imu_cam(cal, i);
        if (!p) {
            return std::unexpected(std::format(
                "camera {}: no camera-IMU extrinsic (ombc/tbc) in the calibration or bad id", i));
        }
        if (setup.cam_translation_scale != 1.0) {
            p->t = geom::scale(p->t, setup.cam_translation_scale);
        }
        tic.push_back(pose_json(*p, "            "));
        const Kb4& k = cal.cameras[i].intr;
        intr.push_back(std::format(
            "            {{\n                \"camera_type\": \"kb4\",\n                \"intrinsics\": {{\n"
            "                    \"fx\": {},\n                    \"fy\": {},\n                    \"cx\": {},\n"
            "                    \"cy\": {},\n                    \"k1\": {},\n                    \"k2\": {},\n"
            "                    \"k3\": {},\n                    \"k4\": {}\n                }}\n            }}",
            json::num(k.fx), json::num(k.fy), json::num(k.cx), json::num(k.cy), json::num(k.k[0]), json::num(k.k[1]),
            json::num(k.k[2]), json::num(k.k[3])));
        res.push_back(std::format("            [{}, {}]", k.width, k.height));
    }
    std::array<double, 9> ab{};
    std::copy(setup.accel_bias.begin(), setup.accel_bias.end(), ab.begin());
    const std::string z9 = json::num_array(ab);
    std::array<double, 12> gb{};
    std::copy(setup.gyro_bias.begin(), setup.gyro_bias.end(), gb.begin());
    const std::string z12 = json::num_array(gb);
    const std::string id = pose_json(Pose{}, "        ");
    return std::format(
        "{{\n    \"value0\": {{\n        \"T_imu_cam\": [\n{}\n        ],\n        \"intrinsics\": [\n{}\n        ],\n"
        "        \"resolution\": [\n{}\n        ],\n        \"calib_accel_bias\": {},\n        \"calib_gyro_bias\": {},\n"
        "        \"imu_update_rate\": {},\n        \"accel_noise_std\": {},\n        \"gyro_noise_std\": {},\n"
        "        \"accel_bias_std\": {},\n        \"gyro_bias_std\": {},\n        \"T_mocap_world\": {},\n"
        "        \"T_imu_marker\": {},\n        \"mocap_time_offset_ns\": 0,\n        \"mocap_to_imu_offset_ns\": 0,\n"
        "        \"cam_time_offset_ns\": {},\n        \"vignette\": []\n    }}\n}}\n",
        join(tic, ",\n"), join(intr, ",\n"), join(res, ",\n"), z9, z12, json::num(imu.rate_hz),
        json::num_array(imu.accel_noise_std), json::num_array(imu.gyro_noise_std),
        json::num_array(imu.accel_bias_std), json::num_array(imu.gyro_bias_std), trim_start(id), trim_start(id),
        setup.cam_time_offset_ns);
}

std::string info_json(const DeviceCalib& cal, const SlamSetup& setup, const ImuNoise& imu,
                      const std::vector<std::pair<std::string, std::string>>& extra) {
    std::vector<std::string> names;
    std::vector<double> ids;
    for (const std::size_t i : setup.cams) {
        names.push_back(json::quote(i < cal.cameras.size() ? std::string_view(cal.cameras[i].name) : "?"));
        ids.push_back(static_cast<double>(i));
    }
    std::string s = std::format(
        "{{\n  \"device_uid\": {},\n  \"factory_cams\": {},\n  \"cam_names\": [{}],\n  \"cam_hz\": {},\n"
        "  \"imu_frame\": \"factory body = OpenXR head axes (x right, y up, z back) = raw IMU with x,z negated\",\n"
        "  \"imu_noise_source\": {},\n  \"cam_time_offset_ns\": {},\n  \"gyro_bias_rad_s\": {},\n"
        "  \"accel_bias_m_s2\": {}",
        json::quote(cal.uid), json::num_array(ids), join(names, ", "), json::num(setup.cam_hz),
        json::quote(imu.source), setup.cam_time_offset_ns, json::num_array(setup.gyro_bias),
        json::num_array(setup.accel_bias));
    for (const auto& [k, v] : extra) {
        s += std::format(",\n  {}: {}", json::quote(k), json::quote(v));
    }
    s += "\n}\n";
    return s;
}

std::expected<std::string, std::string> write_basalt_files(
    const std::string& dir, const DeviceCalib& cal, const SlamSetup& setup, const ImuNoise& imu,
    const std::vector<std::pair<std::string, std::string>>& extra) {
    if (auto r = create_dir_all(dir); !r) {
        return std::unexpected(path_err(rf::utf8_lossy(dir), r.error().text()));
    }
    const std::string calib = path_join(dir, "pimax-basalt-calib.json");
    auto text = basalt_calib_json(cal, setup, imu);
    if (!text) {
        return std::unexpected(text.error());
    }
    if (auto r = write_file(calib, *text); !r) {
        return std::unexpected(path_err(rf::utf8_lossy(calib), r.error().text()));
    }
    const std::string info = path_join(dir, "pimax-slam-info.json");
    if (auto r = write_file(info, info_json(cal, setup, imu, extra)); !r) {
        return std::unexpected(path_err(rf::utf8_lossy(info), r.error().text()));
    }
    return calib;
}

Vec3 default_head_in_imu(const DeviceCalib& cal) {
    const auto a = t_imu_cam(cal, 0);
    const auto b = t_imu_cam(cal, 1);
    if (a && b) {
        const Vec3 m = geom::scale(geom::add(a->t, b->t), 0.5);
        return {m[0], m[1], m[2] + 0.05};
    }
    return {0.0, 0.0, 0.0};
}

/*
 * Supervisor
 */

Supervisor::Supervisor(SupervisorParams p, Vec3 head_in_imu) : p_(p), head_in_imu_(head_in_imu) {}

void Supervisor::set_still(bool still) {
    if (!still) {
        still_anchor_.reset();
        rezero_streak_ = 0;
    }
    still_ = still;
}

void Supervisor::shift_world(const Vec3& d) {
    if (state_ == TrackState::Tracking && std::isfinite(d[0]) && std::isfinite(d[1]) && std::isfinite(d[2])) {
        w_t_ = geom::add(w_t_, d);
    }
}

Vec3 Supervisor::head_world(const Vec3& pos, const Quat& q) const {
    const Vec3 head_slam = geom::add(pos, qrot(q, head_in_imu_));
    return geom::add(qrot(w_q_, head_slam), w_t_);
}

TrackOutput Supervisor::output(std::uint32_t ev) const {
    TrackOutput o;
    o.state = state_;
    o.events = ev;
    o.world_from_slam = {w_t_, w_q_};
    o.fix3dof = fix3dof_;
    o.held_pos = held_pos_;
    o.reason = reason_;
    return o;
}

const char* Supervisor::consistent(const TrackSample& prev, const TrackSample& s) const {
    for (const double x : s.pos) {
        if (!std::isfinite(x)) {
            return "non-finite pose";
        }
    }
    for (const double x : s.q) {
        if (!std::isfinite(x)) {
            return "non-finite pose";
        }
    }
    for (const double x : s.q3dof) {
        if (!std::isfinite(x)) {
            return "non-finite pose";
        }
    }
    const double dt = static_cast<double>(s.ts_ns - prev.ts_ns) / 1e9;
    const double speed = geom::norm(geom::sub(s.pos, prev.pos)) / rf::fmax(dt, 1e-3);
    if (speed > p_.max_speed_m_s) {
        return "position jump";
    }
    const Quat d_slam = qmul(qconj(qnorm(prev.q)), qnorm(s.q));
    const Quat d_imu = qmul(qconj(qnorm(prev.q3dof)), qnorm(s.q3dof));
    if (qangle_deg(qmul(qconj(d_slam), d_imu)) > p_.max_rot_mismatch_deg) {
        return "rotation disagrees with the gyro";
    }
    return nullptr;
}

bool Supervisor::acquire(const TrackSample& s, std::uint64_t now_ns) {
    const Quat target_q = qmul(fix3dof_, qnorm(s.q3dof));
    const Quat new_q = qyaw(yaw_of(qmul(target_q, qconj(qnorm(s.q)))));
    const Vec3 head_slam = geom::add(s.pos, qrot(qnorm(s.q), head_in_imu_));
    const Vec3 new_t = geom::sub(held_pos_, qrot(new_q, head_slam));
    bool restored = false;
    if (resnap_ && lost_w_ && !reset_since_loss_) {
        const Resnap r = *resnap_;
        const auto [old_t, old_q] = *lost_w_;
        double dyaw = yaw_of(new_q) - yaw_of(old_q);
        constexpr double pi = 3.14159265358979323846264338327950288;
        while (dyaw > pi) {
            dyaw -= 2.0 * pi;
        }
        while (dyaw < -pi) {
            dyaw += 2.0 * pi;
        }
        const Vec3 head_old = geom::add(qrot(old_q, head_slam), old_t);
        const std::uint64_t lost_ns = now_ns >= lost_since_ns_ ? now_ns - lost_since_ns_ : 0;
        const double lost_s = static_cast<double>(lost_ns) / 1e9;
        const double allowed = rf::fmax(r.min_m, r.speed_m_s * lost_s);
        if (rf::to_degrees(std::fabs(dyaw)) <= r.yaw_deg && geom::norm(geom::sub(head_old, held_pos_)) <= allowed) {
            w_t_ = old_t;
            w_q_ = old_q;
            restored = true;
        }
    }
    if (!restored) {
        w_q_ = new_q;
        w_t_ = new_t;
    }
    lost_w_.reset();
    state_ = TrackState::Tracking;
    acquisitions += 1;
    reason_ = restored ? "tracking (room restored)" : acquisitions > 1 ? "tracking (re-aligned)" : "tracking";
    return restored;
}

void Supervisor::lose(std::uint64_t now_ns, const char* reason) {
    if (state_ == TrackState::Tracking) {
        lost_w_ = std::pair{w_t_, w_q_};
        reset_since_loss_ = false;
    }
    if (last_) {
        const TrackSample l = *last_;
        held_pos_ = head_world(l.pos, l.q);
        const Quat world_q = qmul(tilt_, qmul(w_q_, qnorm(l.q)));
        fix3dof_ = qyaw(yaw_of(qmul(world_q, qconj(qnorm(l.q3dof)))));
    }
    state_ = TrackState::Lost;
    lost_since_ns_ = now_ns;
    good_run_ = 0;
    losses += 1;
    reason_ = reason;
}

namespace {
std::uint64_t sat_sub(std::uint64_t a, std::uint64_t b) {
    return a >= b ? a - b : 0;
}
} // namespace

TrackOutput Supervisor::update(std::uint64_t now_ns, std::optional<TrackSample> sample) {
    std::uint32_t ev = 0;
    if (recenter_) {
        recenter_ = false;
        ev |= events::RECENTERED;
        if (state_ == TrackState::Tracking && last_) {
            const Vec3 head = head_world(last_->pos, last_->q);
            w_t_ = geom::sub(w_t_, head);
        } else {
            held_pos_ = {0.0, 0.0, 0.0};
        }
    }
    bool fresh = false;
    if (sample && last_) {
        fresh = sample->ts_ns > last_->ts_ns;
    } else if (sample) {
        fresh = true;
    }
    if (fresh) {
        const TrackSample s = *sample;
        last_new_at_ns_ = now_ns;
        const char* check = last_ ? consistent(*last_, s) : nullptr;
        if (state_ == TrackState::Tracking && check == nullptr) {
        } else if (state_ == TrackState::Tracking) {
            rejects += 1;
            lose(now_ns, check);
            ev |= events::LOST;
        } else if (check == nullptr) {
            good_run_ += 1;
            const std::uint32_t need = rezeroing_ ? still_p_.rezero_poses : p_.acquire_poses;
            if (good_run_ >= std::max<std::uint32_t>(need, 1)) {
                if (rezeroing_) {
                    // Silent: the fresh tracker world takes over the held pose, nothing was lost.
                    rezeroing_ = false;
                    acquire(s, now_ns);
                    acquisitions -= 1;
                    reason_ = "tracking (re-zeroed while still)";
                } else {
                    if (acquire(s, now_ns)) {
                        ev |= events::RESNAPPED;
                    }
                    ev |= events::ACQUIRED;
                }
            }
        } else {
            rejects += 1;
            good_run_ = 0;
            if (!rezeroing_) {
                reason_ = check;
            }
        }
        last_ = s;
    } else if (state_ == TrackState::Tracking && sat_sub(now_ns, last_new_at_ns_) > p_.stale_ns) {
        lose(now_ns, "no tracker pose (stale)");
        ev |= events::LOST;
    }
    if (state_ == TrackState::Tracking && still_ && still_p_.hold) {
        ev |= hold_still(now_ns);
    } else {
        still_ref_.reset();
    }
    if (state_ == TrackState::Lost && p_.reset_after_ns > 0 && sat_sub(now_ns, lost_since_ns_) > p_.reset_after_ns &&
        (last_reset_ns_ == 0 || sat_sub(now_ns, last_reset_ns_) > p_.reset_every_ns)) {
        last_reset_ns_ = now_ns;
        ev |= request_reset();
    }
    return output(ev);
}

std::uint32_t Supervisor::request_reset() {
    resets += 1;
    last_.reset();
    good_run_ = 0;
    reset_since_loss_ = true;
    lost_w_.reset();
    still_ref_.reset();
    return events::RESET_WANTED;
}

std::uint32_t Supervisor::rezero(std::uint64_t now_ns) {
    if (last_) {
        const TrackSample l = *last_;
        held_pos_ = head_world(l.pos, l.q);
        const Quat world_q = qmul(tilt_, qmul(w_q_, qnorm(l.q)));
        fix3dof_ = qnorm(qmul(world_q, qconj(qnorm(l.q3dof))));
    }
    state_ = TrackState::Lost;
    lost_since_ns_ = now_ns;
    good_run_ = 0;
    rezeroing_ = true;
    // The driver tells a re-zero by the "re-zeroed" prefix. Three in a row (only with the PiBridge back-off): the
    // cameras see too little to hold a position (dark or plain view), in plain words for the status line.
    reason_ = rezero_streak_ >= 3 ? "re-zeroed: cameras see too little" : "re-zeroed while still";
    return events::REZEROED | request_reset();
}

std::uint32_t Supervisor::hold_still(std::uint64_t now_ns) {
    if (!last_) {
        return 0;
    }
    const TrackSample l = *last_;
    const Vec3 head_slam = geom::add(l.pos, qrot(qnorm(l.q), head_in_imu_));
    if (!still_ref_) {
        // Hold where the head was when the quiet began (the tracker's drift since then is not motion).
        const Vec3 r = still_anchor_.value_or(head_slam);
        still_anchor_.reset();
        still_ref_ = std::pair{r, geom::add(qrot(w_q_, r), w_t_)};
        return 0;
    }
    const auto [ref_slam, target] = *still_ref_;
    w_t_ = geom::sub(target, qrot(w_q_, head_slam));
    const double drift = geom::norm(geom::sub(head_slam, ref_slam));
    // PiBridge back-off (backoff_max 0 = the Rust interval): 1, 2, 4 ... s while the headset stays still.
    const std::uint64_t every = still_p_.reset_every_ns << std::min(rezero_streak_, std::min(still_p_.backoff_max, 16u));
    if (drift > still_p_.reset_m && (last_still_reset_ns_ == 0 || sat_sub(now_ns, last_still_reset_ns_) > every)) {
        last_still_reset_ns_ = now_ns;
        still_resets += 1;
        if (still_p_.backoff_max > 0) {
            rezero_streak_ += 1;
        }
        if (still_p_.rezero) {
            return rezero(now_ns);
        }
        // Old behaviour: held at target (lose() takes the head position with the adjusted alignment), 3DoF until the
        // fresh tracker world is acquired again at that position.
        lose(now_ns, "tracker drifting while still");
        return events::LOST | request_reset();
    }
    return 0;
}

} // namespace pibridge::cam::slam
