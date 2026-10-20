// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/overlay.rs (MIT OR Apache-2.0).
#include "pibridge/cam/overlay.h"

#include "pibridge/cam/rustfmt.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <format>
#include <unistd.h>

namespace pibridge::cam::overlay {

namespace {

std::unexpected<OverlayError> ferr(std::string s) {
    return std::unexpected(OverlayError{OverlayError::Kind::Format, std::move(s), {}});
}

Result<void> rename_file(const std::string& a, const std::string& b) {
    if (std::rename(a.c_str(), b.c_str()) != 0) {
        return fail(IoError::last_os());
    }
    return {};
}

} // namespace

double CameraRotation::angle_deg() const {
    return rfmt::to_degrees(geom::norm(rotation_vector));
}

std::string OverlayError::text() const {
    switch (kind) {
    case Kind::Io: return std::format("{}: {}", rfmt::utf8_lossy(path), message);
    case Kind::Json: return "overlay: " + message;
    case Kind::Format: return "overlay: " + message;
    }
    return message;
}

CalibOverlay CalibOverlay::for_calib(const DeviceCalib& calib, std::string_view tool) {
    CalibOverlay o;
    o.device_uid = calib.uid;
    o.factory_date = calib.date;
    const std::time_t now = std::time(nullptr);
    o.created_unix = now > 0 ? static_cast<std::uint64_t>(now) : 0;
    o.tool = std::string(tool);
    return o;
}

geom::Vec3 CalibOverlay::rotation(std::size_t id) const {
    for (const auto& r : rotations) {
        if (r.id == id) {
            return r.rotation_vector;
        }
    }
    return {0.0, 0.0, 0.0};
}

double CalibOverlay::max_angle_deg() const {
    double m = 0.0;
    for (const auto& r : rotations) {
        m = rfmt::fmax(m, r.angle_deg());
    }
    return m;
}

std::expected<CalibOverlay, OverlayError> CalibOverlay::parse(std::string_view text) {
    auto pv = json::parse(text);
    if (!pv) {
        return std::unexpected(OverlayError{OverlayError::Kind::Json, pv.error().text(), {}});
    }
    const json::Value& v = *pv;
    const json::Value* f = v.get("format");
    if (f == nullptr || f->as_str() == nullptr || *f->as_str() != FORMAT) {
        return ferr(std::format("\"format\" is not \"{}\"", FORMAT));
    }
    double ver = 0.0;
    if (const json::Value* x = v.get("version"); x != nullptr && x->as_f64()) {
        ver = *x->as_f64();
    }
    if (ver != static_cast<double>(VERSION)) {
        return ferr(std::format("version {}, this reader knows {}", rfmt::display_f64(ver), VERSION));
    }
    const auto s = [&](const char* k) {
        const json::Value* x = v.get(k);
        return x != nullptr && x->as_str() != nullptr ? *x->as_str() : std::string();
    };
    CalibOverlay o;
    if (const json::Value* cams = v.get("cameras"); cams != nullptr && cams->as_array() != nullptr) {
        for (const json::Value& c : *cams->as_array()) {
            const json::Value* idv = c.get("id");
            std::optional<double> idf = idv != nullptr ? idv->as_f64() : std::nullopt;
            if (!idf || std::trunc(*idf) != *idf || !(*idf >= 0.0 && *idf < 16.0)) {
                return ferr("camera without a valid id");
            }
            const auto id = static_cast<std::size_t>(*idf);
            const json::Value* rv = c.get("rotation_vector");
            const auto w = rv != nullptr ? rv->as_f64_array<3>() : std::nullopt;
            if (!w) {
                return ferr(std::format("camera {}: rotation_vector needs 3 numbers", id));
            }
            for (const auto& r : o.rotations) {
                if (r.id == id) {
                    return ferr(std::format("camera {} listed twice", id));
                }
            }
            CameraRotation r;
            r.id = id;
            r.rotation_vector = *w;
            if (const json::Value* sg = c.get("sigma")) {
                r.sigma = sg->as_f64_array<3>();
            }
            o.rotations.push_back(r);
        }
    }
    if (const json::Value* q = v.get("quality"); q != nullptr && q->type == json::Value::Type::Obj) {
        for (const auto& [k, x] : q->obj) {
            if (const auto fx = x.as_f64()) {
                o.quality.emplace_back(k, *fx);
            }
        }
    }
    o.device_uid = s("device_uid");
    o.factory_date = s("factory_calibration_date");
    if (const json::Value* cu = v.get("created_unix"); cu != nullptr && cu->as_f64() && *cu->as_f64() >= 0.0) {
        o.created_unix = rfmt::f64_to_u64(*cu->as_f64());
    }
    o.tool = s("tool");
    if (const json::Value* rc = v.get("reference_camera"); rc != nullptr && rc->as_f64()) {
        o.reference_camera = static_cast<std::size_t>(rfmt::f64_to_u64(*rc->as_f64()));
    }
    o.note = s("note");
    return o;
}

std::string CalibOverlay::to_json() const {
    std::string o = "{\n";
    o += std::format("  \"format\": {},\n", json::quote(FORMAT));
    o += std::format("  \"version\": {},\n", VERSION);
    o += std::format("  \"device_uid\": {},\n", json::quote(device_uid));
    o += std::format("  \"factory_calibration_date\": {},\n", json::quote(factory_date));
    o += std::format("  \"created_unix\": {},\n", created_unix);
    o += std::format("  \"tool\": {},\n", json::quote(tool));
    o += std::format("  \"reference_camera\": {},\n", reference_camera);
    o += "  \"convention\": \"camera rotated about its own centre: cam_from_X' = Exp(rotation_vector) * cam_from_X; "
         "radians, OpenCV camera axes; rig X_i = R X_0 + t\",\n";
    o += "  \"cameras\": [";
    for (std::size_t n = 0; n < rotations.size(); n++) {
        const auto& r = rotations[n];
        o += n == 0 ? "\n" : ",\n";
        o += std::format("    {{ \"id\": {}, \"rotation_vector\": {}, \"angle_deg\": {}", r.id,
                         json::num_array(r.rotation_vector), json::num(std::round(r.angle_deg() * 1e5) / 1e5));
        if (r.sigma) {
            o += std::format(", \"sigma\": {}", json::num_array(*r.sigma));
        }
        o += " }";
    }
    o += rotations.empty() ? "],\n" : "\n  ],\n";
    o += "  \"quality\": {";
    for (std::size_t n = 0; n < quality.size(); n++) {
        o += n == 0 ? "\n" : ",\n";
        o += std::format("    {}: {}", json::quote(quality[n].first), json::num(quality[n].second));
    }
    o += quality.empty() ? "},\n" : "\n  },\n";
    o += std::format("  \"note\": {}\n}}\n", json::quote(note));
    return o;
}

std::expected<CalibOverlay, OverlayError> CalibOverlay::load(const std::string& path) {
    auto t = read_file(path);
    if (!t) {
        return std::unexpected(OverlayError{OverlayError::Kind::Io, t.error().text(), path});
    }
    if (!rfmt::utf8_valid(*t)) {
        return std::unexpected(OverlayError{OverlayError::Kind::Io, "stream did not contain valid UTF-8", path});
    }
    return parse(*t);
}

std::optional<std::string> CalibOverlay::refusal(const DeviceCalib& calib, double max_angle_deg) const {
    if (!device_uid.empty() && device_uid != calib.uid) {
        return std::format("made for headset {}, this calibration is {}", device_uid, calib.uid);
    }
    if (!factory_date.empty() && factory_date != calib.date) {
        return std::format("made against the factory calibration of {}, this one is from {}", factory_date,
                           calib.date);
    }
    for (const auto& r : rotations) {
        if (r.id >= calib.cameras.size()) {
            return std::format("camera {} does not exist", r.id);
        }
        for (const double x : r.rotation_vector) {
            if (!std::isfinite(x)) {
                return std::format("camera {}: not a number", r.id);
            }
        }
        if (r.angle_deg() > max_angle_deg) {
            return std::format("camera {}: correction {} deg exceeds the {} deg limit", r.id,
                               rfmt::fixed(r.angle_deg(), 2), rfmt::display_f64(max_angle_deg));
        }
    }
    return std::nullopt;
}

std::expected<DeviceCalib, CalibError> with_overlay(const DeviceCalib& calib, const CalibOverlay& ov,
                                                    double max_angle_deg) {
    if (auto why = ov.refusal(calib, max_angle_deg)) {
        return std::unexpected(CalibError{CalibError::Kind::Format, "overlay refused: " + *why, {}, {}});
    }
    DeviceCalib c = calib;
    for (const auto& r : ov.rotations) {
        const geom::Mat3 e = geom::rodrigues(r.rotation_vector);
        CameraCalib& cam = c.cameras[r.id];
        cam.rig_r = geom::mat_mul(e, cam.rig_r);
        cam.rig_t = geom::mat_vec(e, cam.rig_t);
    }
    return c;
}

std::optional<std::string> default_path() {
    if (auto v = env_os("PIMAX_CAM_OVERLAY"); v && !v->empty()) {
        return v;
    }
    std::string base;
    if (auto x = env_os("XDG_CONFIG_HOME"); x && !x->empty()) {
        base = *x;
    } else if (auto h = env_os("HOME")) {
        base = path_join(*h, ".config");
    } else {
        return std::nullopt;
    }
    return path_join(base, "omarchyvr/cam-calib-overlay.json");
}

std::string prev_path(const std::string& path) {
    return path + ".prev";
}

Result<void> save(const CalibOverlay& ov, const std::string& path) {
    const std::filesystem::path p(path);
    if (p.has_parent_path() && !p.parent_path().empty()) {
        std::error_code ec;
        std::filesystem::create_directories(p.parent_path(), ec);
        if (ec) {
            return fail(IoError::from_errno(ec.value()));
        }
    }
    const std::string tmp = path + ".tmp";
    {
        std::FILE* f = std::fopen(tmp.c_str(), "wb");
        if (f == nullptr) {
            return fail(IoError::last_os());
        }
        const std::string j = ov.to_json();
        const bool ok = std::fwrite(j.data(), 1, j.size(), f) == j.size();
        const int e = errno;
        if (std::fclose(f) != 0 || !ok) {
            return fail(IoError::from_errno(ok ? errno : e));
        }
    }
    if (path_exists(path)) {
        if (auto r = rename_file(path, prev_path(path)); !r) {
            return r;
        }
    }
    return rename_file(tmp, path);
}

Result<void> rollback(const std::string& path) {
    const std::string prev = prev_path(path);
    if (!path_exists(prev)) {
        return fail(IoError::custom(ErrorKind::NotFound, std::format("no previous overlay {}", rfmt::utf8_lossy(prev))));
    }
    const std::string tmp = path + ".swap";
    const bool had_current = path_exists(path);
    if (had_current) {
        if (auto r = rename_file(path, tmp); !r) {
            return r;
        }
    }
    if (auto r = rename_file(prev, path); !r) {
        return r;
    }
    if (had_current) {
        if (auto r = rename_file(tmp, prev); !r) {
            return r;
        }
    }
    return {};
}

Result<std::vector<std::string>> clear(const std::string& path) {
    std::vector<std::string> gone;
    for (const std::string& p : {path, prev_path(path)}) {
        if (::unlink(p.c_str()) == 0) {
            gone.push_back(p);
        } else if (errno != ENOENT) {
            return fail(IoError::last_os());
        }
    }
    return gone;
}

} // namespace pibridge::cam::overlay
