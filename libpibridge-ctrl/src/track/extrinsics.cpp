// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/extrinsics.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/track/extrinsics.h"

#include "pibridge/cam/io.h"
#include "pibridge/cam/json.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/ctrl/track/rust.h"

#include <cmath>
#include <cstdio>
#include <format>

namespace pibridge::ctrl::track::extrinsics {

namespace g = cam::geom;
namespace rf = cam::rfmt;

namespace {

/// Rust Path::with_extension(ext) for a path with a normal file name.
std::string with_extension(const std::string& path, std::string_view ext) {
    const std::size_t slash = path.rfind('/');
    const std::size_t name_at = slash == std::string::npos ? 0 : slash + 1;
    const std::string name = path.substr(name_at);
    std::size_t stem_len = name.size();
    if (name != "..") {
        const std::size_t dot = name.rfind('.');
        if (dot != std::string::npos && dot > 0) {
            stem_len = dot;
        }
    }
    std::string out = path.substr(0, name_at) + name.substr(0, stem_len);
    if (!ext.empty()) {
        out += '.';
        out += ext;
    }
    return out;
}

/// "[1, 2, 3]" (Rust `{:?}` of a Vec<usize>).
std::string ids(const CamPoses& v) {
    std::string s = "[";
    for (std::size_t i = 0; i < v.size(); i++) {
        s += (i > 0 ? ", " : "") + std::to_string(v[i].first);
    }
    return s + "]";
}

} // namespace

std::string default_path() {
    return cam::path_join(cam::env_os("HOME").value_or(""), ".config/omarchyvr/ctrl-cam-extrinsics.json");
}

std::expected<void, std::string> save(const std::string& path, std::string_view device_uid,
                                      std::span<const std::pair<std::size_t, g::Pose>> cams, std::string_view note) {
    std::string s = "{\n  \"format\": \"pimax-ctrltrack-extrinsics\",\n  \"version\": 1,\n";
    s += std::format("  \"device_uid\": {},\n  \"note\": {},\n  \"cameras\": [\n", cam::json::quote(device_uid),
                     cam::json::quote(note));
    for (std::size_t k = 0; k < cams.size(); k++) {
        const auto& [id, p] = cams[k];
        std::vector<double> r;
        for (const auto& row : p.r) {
            r.insert(r.end(), row.begin(), row.end());
        }
        s += std::format("    {{ \"id\": {}, \"cam_from_body_r\": {}, \"cam_from_body_t\": {} }}{}\n", id,
                         cam::json::num_array(r), cam::json::num_array(p.t), k + 1 < cams.size() ? "," : "");
    }
    s += "  ]\n}\n";
    if (const auto d = cam::path_parent(path)) {
        if (auto c = cam::create_dir_all(*d); !c) {
            return std::unexpected(c.error().text());
        }
    }
    const std::string tmp = with_extension(path, "json.tmp");
    if (auto w = cam::write_file(tmp, s); !w) {
        return std::unexpected(w.error().text());
    }
    if (std::rename(tmp.c_str(), path.c_str()) != 0) {
        return std::unexpected(cam::IoError::last_os().text());
    }
    return {};
}

std::expected<std::pair<std::string, CamPoses>, std::string> load(const std::string& path) {
    const auto t = cam::read_to_string(path);
    if (!t) {
        return std::unexpected(std::format("{}: {}", rf::utf8_lossy(path), t.error().text()));
    }
    auto v = parse(*t);
    if (!v) {
        return std::unexpected(std::format("{}: {}", rf::utf8_lossy(path), v.error()));
    }
    return v;
}

std::expected<std::pair<std::string, CamPoses>, std::string> parse(std::string_view t) {
    const auto v = cam::json::parse(t);
    if (!v) {
        return std::unexpected(
            std::format("JsonError {{ pos: {}, msg: {} }}", v.error().pos, rf::debug_str(v.error().msg)));
    }
    const cam::json::Value* f = v->get("format");
    const std::string* fs = f != nullptr ? f->as_str() : nullptr;
    if (fs == nullptr || *fs != "pimax-ctrltrack-extrinsics") {
        return std::unexpected(std::string("not a pimax-ctrltrack-extrinsics file"));
    }
    const cam::json::Value* u = v->get("device_uid");
    const std::string* us = u != nullptr ? u->as_str() : nullptr;
    std::string uid = us != nullptr ? *us : std::string();
    const cam::json::Value* c = v->get("cameras");
    const auto* arr = c != nullptr ? c->as_array() : nullptr;
    if (arr == nullptr) {
        return std::unexpected(std::string("no cameras"));
    }
    CamPoses out;
    for (const cam::json::Value& cv : *arr) {
        const cam::json::Value* idv = cv.get("id");
        const auto id = idv != nullptr ? idv->as_f64() : std::nullopt;
        if (!id) {
            return std::unexpected(std::string("camera without id"));
        }
        const cam::json::Value* rv = cv.get("cam_from_body_r");
        const auto r = rv != nullptr ? rv->as_f64_array<9>() : std::nullopt;
        if (!r) {
            return std::unexpected(std::string("bad r"));
        }
        const cam::json::Value* tv = cv.get("cam_from_body_t");
        const auto tt = tv != nullptr ? tv->as_f64_array<3>() : std::nullopt;
        if (!tt) {
            return std::unexpected(std::string("bad t"));
        }
        const auto& m = *r;
        out.emplace_back(rs::f64_to_usize(*id),
                         g::Pose{{{{m[0], m[1], m[2]}, {m[3], m[4], m[5]}, {m[6], m[7], m[8]}}}, *tt});
    }
    return std::pair{std::move(uid), std::move(out)};
}

bool same_rig_as_reference(const cam::DeviceCalib& calib) {
    const auto r = cam::DeviceCalib::parse(cam::reference_xml());
    if (!r) {
        return false;
    }
    if (calib.cameras.size() != r->cameras.size()) {
        return false;
    }
    const auto close = [](const g::Pose& a, const g::Pose& b) {
        for (int i = 0; i < 3; i++) {
            if (!(std::fabs(a.t[i] - b.t[i]) < 1e-6)) {
                return false;
            }
            for (int j = 0; j < 3; j++) {
                if (!(std::fabs(a.r[i][j] - b.r[i][j]) < 1e-6)) {
                    return false;
                }
            }
        }
        return true;
    };
    for (std::size_t i = 0; i < r->cameras.size(); i++) {
        const auto a = calib.body_from_cam(i, cam::RigConvention::CamFromCam0);
        const auto b = r->body_from_cam(i, cam::RigConvention::CamFromCam0);
        if (!a || !b || !close(*a, *b)) {
            return false;
        }
    }
    return true;
}

std::optional<CamPoses> reference_for(const cam::DeviceCalib& calib, const Env& env) {
    if (!cam::reference_allowed(env) || !same_rig_as_reference(calib)) {
        return std::nullopt;
    }
    auto p = parse(reference_json());
    if (!p) {
        return std::nullopt;
    }
    return std::move(p->second);
}

std::pair<CamPoses, std::string> resolve(const cam::DeviceCalib& calib, const std::optional<std::string>& path,
                                         const Env& env) {
    const std::string p = path ? *path : default_path();
    if (cam::path_exists(p)) {
        auto l = load(p);
        if (l && (l->first.empty() || l->first == calib.uid)) {
            std::string note = std::format("extrinsics of cams {} from {}", ids(l->second), rf::utf8_lossy(p));
            return {std::move(l->second), std::move(note)};
        }
        if (l) {
            std::fprintf(stderr, "%s: for headset %s, not %s: ignored\n", rf::utf8_lossy(p).c_str(), l->first.c_str(),
                         calib.uid.c_str());
        } else {
            std::fprintf(stderr, "extrinsics not loaded: %s\n", l.error().c_str());
        }
    }
    if (auto v = reference_for(calib, env)) {
        std::string note =
            std::format("extrinsics of cams {} from the shipped Dream Air reference (same headset)", ids(*v));
        return {std::move(*v), std::move(note)};
    }
    return {CamPoses{}, "factory extrinsics"};
}

} // namespace pibridge::ctrl::track::extrinsics
