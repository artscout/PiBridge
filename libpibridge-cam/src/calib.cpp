// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/calib.rs (MIT OR Apache-2.0).
#include "pibridge/cam/calib.h"

#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <cmath>
#include <format>

namespace pibridge::cam {

namespace {

std::unexpected<CalibError> fmt_err(std::string s) {
    return std::unexpected(CalibError{CalibError::Kind::Format, std::move(s), {}, {}});
}

// The pibridge name of a PIMAX_* variable wins when set (docs/CONVENTIONS.md); both literal names stay in the binary.
std::optional<std::string> env_pair(const char* pibridge_name, const char* pimax_name) {
    if (auto v = env_os(pibridge_name); v && !v->empty()) {
        return v;
    }
    return env_os(pimax_name);
}

std::expected<std::vector<double>, CalibError> nums(const xml::Element& el, const char* attr) {
    const std::string* v = el.attr(attr);
    if (v == nullptr) {
        return fmt_err(std::format("<{}> lacks {}", el.name, attr));
    }
    std::vector<double> out;
    for (const auto t : rfmt::split_whitespace(*v)) {
        double x = 0;
        if (!rfmt::parse_f64(t, x) || !std::isfinite(x)) {
            return fmt_err(std::format("{}: bad number {}", attr, rfmt::debug_str(t)));
        }
        out.push_back(x);
    }
    return out;
}

template <std::size_t N>
std::expected<std::array<double, N>, CalibError> nums_n(const xml::Element& el, const char* attr) {
    auto v = nums(el, attr);
    if (!v) {
        return std::unexpected(v.error());
    }
    if (v->size() < N) {
        return fmt_err(std::format("{}: {} numbers, need {}", attr, v->size(), N));
    }
    std::array<double, N> a{};
    std::copy_n(v->begin(), N, a.begin());
    return a;
}

std::optional<double> parse_trimmed(const std::string* v) {
    if (v == nullptr) {
        return std::nullopt;
    }
    double x = 0;
    if (!rfmt::parse_f64(rfmt::trim(*v), x)) {
        return std::nullopt;
    }
    return x;
}

#define PIBRIDGE_TRY(var, expr)                                                                                           \
    auto var = (expr);                                                                                                 \
    if (!var) {                                                                                                        \
        return std::unexpected(var.error());                                                                           \
    }

} // namespace

const char* convention_name(RigConvention c) {
    return c == RigConvention::CamFromCam0 ? "CamFromCam0" : "Cam0FromCam";
}

bool reference_allowed(const std::function<std::optional<std::string>(std::string_view)>& env) {
    const auto v = env("PIMAX_NO_REFERENCE");
    return !(v && !v->empty() && *v != "0");
}

std::string reference_path(const std::optional<std::string>& home) {
    if (home) {
        const std::string p = path_join(*home, REFERENCE_INSTALLED);
        if (is_file(p)) {
            return p;
        }
    }
    return "(built-in reference) dream-air/reference/headset-calibration.xml";
}

std::string CalibError::text() const {
    switch (kind) {
    case Kind::Io: return std::format("{}: {}", rfmt::utf8_lossy(path), message);
    case Kind::Xml: return message;
    case Kind::Format: return "calibration: " + message;
    case Kind::NotFound: {
        std::string s = "no calibration file found (tried";
        for (const auto& p : tried) {
            s += " " + rfmt::utf8_lossy(p);
        }
        s += "); read one with `omarchyvr calib`";
        return s;
    }
    }
    return message;
}

std::expected<DeviceCalib, CalibError> DeviceCalib::parse(std::string_view doc) {
    auto root = xml::parse(doc);
    if (!root) {
        return std::unexpected(CalibError{CalibError::Kind::Xml, root.error().text(), {}, {}});
    }
    if (root->name != "DeviceConfiguration") {
        return fmt_err(std::format("root element <{}>, expected <DeviceConfiguration>", root->name));
    }
    const auto s = [&](const char* k) {
        const std::string* v = root->attr(k);
        return v != nullptr ? *v : std::string();
    };
    std::vector<CameraCalib> cameras;
    for (const xml::Element* c : root->children_named("Camera")) {
        std::uint32_t id = 0;
        const std::string* ida = c->attr("id");
        if (ida == nullptr || !rfmt::parse_u32(rfmt::trim(*ida), id)) {
            return fmt_err("<Camera> without a numeric id");
        }
        const xml::Element* cal = c->child("Calibration");
        if (cal == nullptr) {
            return fmt_err(std::format("camera {}: no <Calibration>", id));
        }
        const xml::Element* rig = c->child("Rig");
        if (rig == nullptr) {
            return fmt_err(std::format("camera {}: no <Rig>", id));
        }
        PIBRIDGE_TRY(size, nums_n<2>(*cal, "size"));
        PIBRIDGE_TRY(pp, nums_n<2>(*cal, "principal_point"));
        PIBRIDGE_TRY(fl, nums_n<2>(*cal, "focal_length"));
        PIBRIDGE_TRY(dist, nums(*cal, "radial_distortion"));
        if (dist->size() < 4) {
            return fmt_err(std::format("camera {}: {} distortion terms, need 4", id, dist->size()));
        }
        if ((*size)[0] < 1.0 || (*size)[1] < 1.0 || (*fl)[0] <= 0.0 || (*fl)[1] <= 0.0) {
            return fmt_err(std::format("camera {}: bad size or focal length", id));
        }
        PIBRIDGE_TRY(r, nums_n<9>(*rig, "rowMajorRotationMat"));
        const geom::Mat3 rig_r{{{(*r)[0], (*r)[1], (*r)[2]}, {(*r)[3], (*r)[4], (*r)[5]}, {(*r)[6], (*r)[7], (*r)[8]}}};
        if (geom::orthonormality_error(rig_r) > 1e-4 || geom::det(rig_r) < 0.0) {
            return fmt_err(std::format("camera {}: rig rotation is not a rotation", id));
        }
        double time_delta = 0.0;
        if (const xml::Element* ta = c->child("TimeAlignment")) {
            time_delta = parse_trimmed(ta->attr("delta")).value_or(0.0);
        }
        CameraCalib cc;
        cc.id = id;
        const std::string* nm = c->attr("name");
        if (nm == nullptr) {
            nm = c->attr("cam_name");
        }
        cc.name = nm != nullptr ? *nm : "";
        const std::string* model = cal->attr("model");
        cc.model = model != nullptr ? *model : "";
        cc.intr = Kb4::make(rfmt::f64_to_u32((*size)[0]), rfmt::f64_to_u32((*size)[1]), (*fl)[0], (*fl)[1], (*pp)[0],
                            (*pp)[1], {(*dist)[0], (*dist)[1], (*dist)[2], (*dist)[3]},
                            std::vector<double>(dist->begin() + 4, dist->end()));
        cc.rig_r = rig_r;
        PIBRIDGE_TRY(tr, nums_n<3>(*rig, "translation"));
        cc.rig_t = *tr;
        cc.time_delta = time_delta;
        cameras.push_back(std::move(cc));
    }
    if (cameras.empty()) {
        return fmt_err("no <Camera> elements");
    }
    std::stable_sort(cameras.begin(), cameras.end(),
                     [](const CameraCalib& a, const CameraCalib& b) { return a.id < b.id; });
    for (std::size_t i = 0; i < cameras.size(); i++) {
        if (cameras[i].id != i) {
            return fmt_err(std::format("camera ids are not 0..{}", cameras.size() - 1));
        }
    }
    std::optional<ImuExtrinsic> imu;
    if (const xml::Element* sf = root->child("SFConfig")) {
        if (const xml::Element* st = sf->child("Stateinit")) {
            PIBRIDGE_TRY(ombc, nums_n<3>(*st, "ombc"));
            PIBRIDGE_TRY(tbc, nums_n<3>(*st, "tbc"));
            ImuExtrinsic m;
            m.ombc = *ombc;
            m.tbc = *tbc;
            m.delta_s = parse_trimmed(st->attr("delta")).value_or(0.0);
            m.raw = st->attrs;
            imu = std::move(m);
        }
    }
    DeviceCalib d;
    d.uid = s("deviceUID");
    d.sensor = s("cameraModel");
    d.date = s("calibration_date");
    d.version = s("pimax_calibration_version");
    d.cameras = std::move(cameras);
    d.imu = std::move(imu);
    return d;
}

std::expected<DeviceCalib, CalibError> DeviceCalib::load(const std::string& path) {
    auto bytes = read_file(path);
    if (!bytes) {
        return std::unexpected(CalibError{CalibError::Kind::Io, bytes.error().text(), path, {}});
    }
    if (!rfmt::utf8_valid(*bytes)) {
        // std::fs::read_to_string
        return std::unexpected(CalibError{CalibError::Kind::Io, "stream did not contain valid UTF-8", path, {}});
    }
    return parse(*bytes);
}

std::vector<std::string> DeviceCalib::default_candidates() {
    std::vector<std::string> c;
    if (auto v = env_pair("PIBRIDGE_CAM_CALIB", "PIMAX_CAM_CALIB"); v && !v->empty()) {
        c.push_back(*v);
    }
    if (auto v = env_pair("PIBRIDGE_CALIB", "PIMAX_CALIB"); v && !v->empty()) {
        c.push_back(*v);
    }
    if (auto h = env_os("HOME")) {
        // PiBridge's own config folder is the default; the earlier names (pimax-linux, omarchyvr) are read only when
        // the file is there (docs/CONVENTIONS.md).
        c.push_back(path_join(*h, ".config/pibridge/headset-calibration.xml"));
        for (const char* legacy : {".config/pimax-linux/headset-calibration.xml", ".config/omarchyvr/headset-calibration.xml"}) {
            const std::string p = path_join(*h, legacy);
            if (is_file(p)) {
                c.push_back(p);
            }
        }
        // the repo the way omarchyvr_config::paths::repo_dir finds it: $PIMAX_REPO, the installed data copy, else
        // ~/omarchyvr
        std::string repo;
        if (auto r = env_pair("PIBRIDGE_REPO", "PIMAX_REPO"); r && !r->empty()) {
            repo = *r;
        } else {
            const std::string installed = path_join(*h, ".local/share/omarchyvr/data");
            repo = is_file(path_join(installed, "docs/LESSONS.md")) ? installed : path_join(*h, "omarchyvr");
        }
        c.push_back(path_join(repo, "analysis/headset-calibration.xml"));
    }
    return c;
}

std::expected<std::pair<DeviceCalib, std::string>, CalibError> DeviceCalib::load_default(
    const std::optional<std::string>& path) {
    if (path) {
        auto c = load(*path);
        if (!c) {
            return std::unexpected(c.error());
        }
        return std::pair{std::move(*c), *path};
    }
    const auto cands = default_candidates();
    const auto home = env_os("HOME");
    return load_first(cands, reference_allowed([](std::string_view k) { return env_utf8(std::string(k).c_str()); }),
                      home);
}

std::expected<std::pair<DeviceCalib, std::string>, CalibError> DeviceCalib::load_first(
    const std::vector<std::string>& cands, bool reference, const std::optional<std::string>& home) {
    for (const auto& p : cands) {
        if (is_file(p)) {
            auto c = load(p);
            if (!c) {
                return std::unexpected(c.error());
            }
            return std::pair{std::move(*c), p};
        }
    }
    if (reference) {
        auto c = parse(reference_xml());
        if (!c) {
            return std::unexpected(c.error());
        }
        return std::pair{std::move(*c), reference_path(home)};
    }
    return std::unexpected(CalibError{CalibError::Kind::NotFound, {}, {}, cands});
}

geom::Pose DeviceCalib::cam_from_cam0(std::size_t i, RigConvention conv) const {
    const CameraCalib& c = cameras.at(i);
    const geom::Pose p{c.rig_r, c.rig_t};
    return conv == RigConvention::CamFromCam0 ? p : p.inverse();
}

geom::Pose DeviceCalib::stereo(std::size_t a, std::size_t b, RigConvention conv) const {
    return cam_from_cam0(b, conv).compose(cam_from_cam0(a, conv).inverse());
}

std::optional<geom::Pose> DeviceCalib::body_from_cam0() const {
    if (!imu) {
        return std::nullopt;
    }
    return geom::Pose{geom::rodrigues(imu->ombc), imu->tbc};
}

std::optional<geom::Pose> DeviceCalib::body_from_cam(std::size_t i, RigConvention conv) const {
    const auto b = body_from_cam0();
    if (!b) {
        return std::nullopt;
    }
    return b->compose(cam_from_cam0(i, conv).inverse());
}

std::string DeviceCalib::geometry_report(RigConvention conv) const {
    using rfmt::fixed;
    using rfmt::fixed_w;
    std::string s = std::format("device {} sensor {} calibrated {} ({})\n", uid, sensor, date, version);
    const std::size_t n = cameras.size();
    std::vector<geom::Vec3> centres;
    std::vector<geom::Vec3> axes;
    for (std::size_t i = 0; i < n; i++) {
        const CameraCalib& c = cameras[i];
        const Kb4& k = c.intr;
        const auto h = [&](double u, double v) { return k.angle_of_pixel_deg(u, v).value_or(std::nan("")); };
        const double hfov = h(0.0, k.cy) + h(static_cast<double>(k.width) - 1.0, k.cy);
        const double vfov = h(k.cx, 0.0) + h(k.cx, static_cast<double>(k.height) - 1.0);
        const geom::Pose p = cam_from_cam0(i, conv).inverse(); // cam0_from_cam_i
        centres.push_back(p.t);
        axes.push_back(geom::col(p.r, 2));
        s += std::format("cam {} {:15} {}x{} f=({},{}) c=({},{}) k={} extra={} FOV {}x{} deg, model limit {} deg\n", i,
                         c.name, k.width, k.height, fixed(k.fx, 1), fixed(k.fy, 1), fixed(k.cx, 1), fixed(k.cy, 1),
                         rfmt::debug_f64s(k.k), rfmt::debug_f64s(k.extra), fixed(hfov, 0), fixed(vfov, 0),
                         fixed(rfmt::to_degrees(k.theta_limit), 1));
    }
    s += std::format("rig convention {}: camera centres / optical axes in the camera-0 frame (OpenCV axes)\n",
                     convention_name(conv));
    for (std::size_t i = 0; i < n; i++) {
        s += std::format("  cam {}: centre ({},{},{}) mm  axis ({},{},{})  {} deg from cam 0\n", i,
                         fixed_w(centres[i][0] * 1e3, 7, 1), fixed_w(centres[i][1] * 1e3, 7, 1),
                         fixed_w(centres[i][2] * 1e3, 7, 1), fixed(axes[i][0], 3, true), fixed(axes[i][1], 3, true),
                         fixed(axes[i][2], 3, true), fixed(geom::angle_deg(axes[i], axes[0]), 1));
    }
    for (std::size_t i = 0; i < n; i++) {
        for (std::size_t j = i + 1; j < n; j++) {
            s += std::format("  pair {}-{}: baseline {} mm, axes {} deg apart\n", i, j,
                             fixed(geom::norm(geom::sub(centres[j], centres[i])) * 1e3, 1),
                             fixed(geom::angle_deg(axes[i], axes[j]), 1));
        }
    }
    if (body_from_cam0()) {
        s += "body frame (OpenXR head axes: x right, y up, -z forward) from ombc/tbc:\n";
        for (std::size_t i = 0; i < n; i++) {
            const geom::Pose p = *body_from_cam(i, conv);
            const geom::Vec3 z = geom::col(p.r, 2);
            const geom::Vec3 down = geom::col(p.r, 1);
            const double yaw = rfmt::to_degrees(std::atan2(-z[0], -z[2]));
            const double pitch = rfmt::to_degrees(std::asin(z[1]));
            s += std::format("  cam {}: position ({},{},{}) mm, looks yaw {} deg (+ = left) pitch {} deg, "
                             "image-down axis ({},{},{})\n",
                             i, fixed_w(p.t[0] * 1e3, 6, 1), fixed_w(p.t[1] * 1e3, 6, 1), fixed_w(p.t[2] * 1e3, 6, 1),
                             fixed(yaw, 1, true), fixed(pitch, 1, true), fixed(down[0], 2, true),
                             fixed(down[1], 2, true), fixed(down[2], 2, true));
        }
    }
    return s;
}

} // namespace pibridge::cam
