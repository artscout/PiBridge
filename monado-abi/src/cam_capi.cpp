// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/capi.rs (MIT OR Apache-2.0).
//
// The C ABI of include/pimax_cam.h (verbatim copy of the Rust crate's header) over libpibridge-cam. Every entry point
// is noexcept with a catch-all, like the Rust catch_unwind guard: errors return -1 / NULL / 0 and
// pimax_cam_last_error() gives the thread-local message, with the same texts as the Rust library ("panic inside
// pimax_cam" for anything unexpected). A pimax_cam handle is used by one thread at a time.
//
// Implemented here (milestones M1/M2): version, last_error, open, close, describe, next, copy_tile, get_stats,
// record_start/stop, calib_load/parse/stereo/body_from_cam, dev_to_mono_ns. The SLAM and track entry points are in
// slam_capi.cpp (M4), the AE ones in ae_capi.cpp (M3).
#include "pimax_cam.h"

#include "capi_common.h"

#include "pibridge/cam/calib.h"
#include "pibridge/cam/camera.h"
#include "pibridge/cam/clock.h"
#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <expected>
#include <format>
#include <memory>
#include <string>


using pibridge::cam::Camera;
using pibridge::cam::DeviceCalib;
using pibridge::cam::FrameInfo;
using pibridge::cam::RigConvention;

using namespace pibridge::abi;

namespace {

// The C structs and the library's own types must agree byte for byte (the Rust code had them as one #[repr(C)]).
#define PIBRIDGE_SAME_FIELD(T, U, f) static_assert(offsetof(T, f) == offsetof(U, f), #f)
static_assert(sizeof(FrameInfo) == sizeof(pimax_cam_frame_info));
PIBRIDGE_SAME_FIELD(FrameInfo, pimax_cam_frame_info, host_ts_ns);
PIBRIDGE_SAME_FIELD(FrameInfo, pimax_cam_frame_info, dev_ts_us);
PIBRIDGE_SAME_FIELD(FrameInfo, pimax_cam_frame_info, mono_ts_ns);
PIBRIDGE_SAME_FIELD(FrameInfo, pimax_cam_frame_info, counter);
PIBRIDGE_SAME_FIELD(FrameInfo, pimax_cam_frame_info, sequence);
PIBRIDGE_SAME_FIELD(FrameInfo, pimax_cam_frame_info, dropped);
PIBRIDGE_SAME_FIELD(FrameInfo, pimax_cam_frame_info, kind);
PIBRIDGE_SAME_FIELD(FrameInfo, pimax_cam_frame_info, frame_type);
PIBRIDGE_SAME_FIELD(FrameInfo, pimax_cam_frame_info, stream_index);
PIBRIDGE_SAME_FIELD(FrameInfo, pimax_cam_frame_info, exposure);
PIBRIDGE_SAME_FIELD(FrameInfo, pimax_cam_frame_info, gain);
PIBRIDGE_SAME_FIELD(FrameInfo, pimax_cam_frame_info, y_offset);
PIBRIDGE_SAME_FIELD(FrameInfo, pimax_cam_frame_info, tile_width);
PIBRIDGE_SAME_FIELD(FrameInfo, pimax_cam_frame_info, tile_height);
PIBRIDGE_SAME_FIELD(FrameInfo, pimax_cam_frame_info, flags);
PIBRIDGE_SAME_FIELD(FrameInfo, pimax_cam_frame_info, reserved);
static_assert(sizeof(pibridge::cam::ImuClockRef) == sizeof(pimax_cam_imu_clock_ref));
PIBRIDGE_SAME_FIELD(pibridge::cam::ImuClockRef, pimax_cam_imu_clock_ref, dev_ts_u32);
PIBRIDGE_SAME_FIELD(pibridge::cam::ImuClockRef, pimax_cam_imu_clock_ref, valid);
PIBRIDGE_SAME_FIELD(pibridge::cam::ImuClockRef, pimax_cam_imu_clock_ref, dev_ext_us);
PIBRIDGE_SAME_FIELD(pibridge::cam::ImuClockRef, pimax_cam_imu_clock_ref, offset_ns);
static_assert(PIMAX_CAM_FLAG_META_VALID == pibridge::cam::FLAG_META_VALID);
static_assert(PIMAX_CAM_FLAG_ROWS_SCANNED == pibridge::cam::FLAG_ROWS_SCANNED);
static_assert(PIMAX_CAM_FLAG_REPLAY == pibridge::cam::FLAG_REPLAY);
static_assert(PIMAX_CAM_TILE_W == pibridge::cam::TILE_W && PIMAX_CAM_TILE_H == pibridge::cam::TILE_H);
static_assert(PIMAX_CAM_TILES == pibridge::cam::TILE_COUNT);

void copy_name(char (&dst)[32], const std::string& s) {
    const std::size_t n = std::min<std::size_t>(s.size(), 31);
    for (std::size_t i = 0; i < n; i++) {
        dst[i] = s[i];
    }
}

pimax_cam_calib to_c(const DeviceCalib& c) {
    pimax_cam_calib out;
    std::memset(&out, 0, sizeof out);
    out.camera_count = static_cast<std::uint32_t>(std::min<std::size_t>(c.cameras.size(), 4));
    out.has_imu = c.imu ? 1 : 0;
    for (auto& cam : out.cameras) {
        cam.id = -1;
    }
    if (c.imu) {
        for (int i = 0; i < 3; i++) {
            out.ombc[i] = c.imu->ombc[i];
            out.tbc[i] = c.imu->tbc[i];
        }
        out.imu_cam_delta_s = c.imu->delta_s;
    }
    copy_name(out.device_uid, c.uid);
    for (std::size_t n = 0; n < out.camera_count; n++) {
        const auto& cam = c.cameras[n];
        pimax_cam_camera& o = out.cameras[n];
        copy_name(o.name, cam.name);
        o.id = static_cast<std::int32_t>(cam.id);
        const auto& k = cam.intr;
        o.intr.width = k.width;
        o.intr.height = k.height;
        o.intr.fx = k.fx;
        o.intr.fy = k.fy;
        o.intr.cx = k.cx;
        o.intr.cy = k.cy;
        for (int i = 0; i < 4; i++) {
            o.intr.k[i] = k.k[i];
        }
        o.intr.extra[0] = k.extra.size() > 0 ? k.extra[0] : 0.0;
        o.intr.extra[1] = k.extra.size() > 1 ? k.extra[1] : 0.0;
        for (int r = 0; r < 3; r++) {
            for (int col = 0; col < 3; col++) {
                o.rig_rotation[r * 3 + col] = cam.rig_r[r][col];
            }
            o.rig_translation[r] = cam.rig_t[r];
        }
        o.time_delta_s = cam.time_delta;
    }
    return out;
}

void write_pose(const pibridge::cam::geom::Pose& p, double* r, double* t) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            r[i * 3 + j] = p.r[i][j];
        }
        t[i] = p.t[i];
    }
}

void copy_out(const std::string& s, char* buf, std::size_t len) {
    if (buf != nullptr && len > 0) {
        const std::size_t n = std::min(s.size(), len - 1);
        std::memcpy(buf, s.data(), n);
        buf[n] = 0;
    }
}

} // namespace

namespace pibridge::abi {

thread_local std::string g_last_error;

DeviceCalib from_c(const pimax_cam_calib& c) {
    const std::size_t n = std::min<std::size_t>(c.camera_count, 4);
    DeviceCalib d;
    for (std::size_t i = 0; i < n; i++) {
        const pimax_cam_camera& o = c.cameras[i];
        pibridge::cam::CameraCalib cc;
        cc.id = static_cast<std::uint32_t>(std::max<std::int32_t>(o.id, 0));
        cc.intr = pibridge::cam::Kb4::make(o.intr.width, o.intr.height, o.intr.fx, o.intr.fy, o.intr.cx, o.intr.cy,
                                        {o.intr.k[0], o.intr.k[1], o.intr.k[2], o.intr.k[3]}, {});
        const double* r = o.rig_rotation;
        cc.rig_r = {{{r[0], r[1], r[2]}, {r[3], r[4], r[5]}, {r[6], r[7], r[8]}}};
        cc.rig_t = {o.rig_translation[0], o.rig_translation[1], o.rig_translation[2]};
        cc.time_delta = o.time_delta_s;
        d.cameras.push_back(std::move(cc));
    }
    if (c.has_imu != 0) {
        pibridge::cam::ImuExtrinsic m;
        m.ombc = {c.ombc[0], c.ombc[1], c.ombc[2]};
        m.tbc = {c.tbc[0], c.tbc[1], c.tbc[2]};
        m.delta_s = c.imu_cam_delta_s;
        d.imu = std::move(m);
    }
    return d;
}

} // namespace pibridge::abi

PIBRIDGE_ABI const char* pimax_cam_version(void) {
    return "0.1.0";
}

PIBRIDGE_ABI const char* pimax_cam_last_error(void) {
    return pibridge::abi::g_last_error.c_str();
}

PIBRIDGE_ABI pimax_cam* pimax_cam_open(const char* source, const pimax_cam_open_params* params) {
    return guard<pimax_cam*>(nullptr, [&]() -> std::expected<pimax_cam*, Err> {
        const std::string spec = opt_str(source).value_or("");
        pimax_cam_open_params p{};
        if (params != nullptr) {
            p = *params;
        }
        pibridge::cam::ReplayOptions ro;
        ro.looping = (p.replay_flags & 1u) != 0;
        ro.realtime = (p.replay_flags & 2u) != 0;
        auto cam = Camera::open(spec, p.fps, ro);
        if (!cam) {
            return err(std::format("open {}: {}", pibridge::cam::rfmt::debug_str(spec), cam.error().text()));
        }
        (*cam)->verify_rows = p.verify_rows != 0;
        auto* h = new pimax_cam{std::move(*cam)};
        return h;
    });
}

PIBRIDGE_ABI void pimax_cam_close(pimax_cam* cam) {
    if (cam == nullptr) {
        return;
    }
    (void)guard<int>(0, [&]() -> std::expected<int, Err> {
        std::unique_ptr<pimax_cam> c(cam);
        (void)c->cam->stop_recording();
        return 0;
    });
}

PIBRIDGE_ABI size_t pimax_cam_describe(const pimax_cam* cam, char* buf, size_t len) {
    return guard<std::size_t>(0, [&]() -> std::expected<std::size_t, Err> {
        if (cam == nullptr) {
            return err("null handle");
        }
        const std::string s = cam->cam->describe();
        copy_out(s, buf, len);
        return s.size();
    });
}

PIBRIDGE_ABI int pimax_cam_next(pimax_cam* cam, int timeout_ms, pimax_cam_frame_info* info) {
    return guard<int>(-1, [&]() -> std::expected<int, Err> {
        if (cam == nullptr) {
            return err("null handle");
        }
        auto r = cam->cam->next_frame(timeout_ms);
        if (!r) {
            return err(r.error().text());
        }
        if (*r) {
            if (info != nullptr) {
                std::memcpy(info, &**r, sizeof *info);
            }
            return 1;
        }
        if (cam->cam->source().finished()) {
            return err("end of replay");
        }
        return 0;
    });
}

PIBRIDGE_ABI int pimax_cam_copy_tile(const pimax_cam* cam, uint32_t tile, uint8_t* dst, uint32_t stride) {
    return guard<int>(-1, [&]() -> std::expected<int, Err> {
        if (cam == nullptr) {
            return err("null handle");
        }
        if (dst == nullptr) {
            return err("null destination");
        }
        const std::size_t len = static_cast<std::size_t>(stride) * (pibridge::cam::TILE_H - 1) + pibridge::cam::TILE_W;
        if (auto r = cam->cam->copy_tile(tile, std::span<std::uint8_t>(dst, len), stride); !r) {
            return err(r.error().text());
        }
        return 0;
    });
}

PIBRIDGE_ABI int pimax_cam_get_stats(const pimax_cam* cam, pimax_cam_stats* out) {
    return guard<int>(-1, [&]() -> std::expected<int, Err> {
        if (cam == nullptr) {
            return err("null handle");
        }
        const auto& s = cam->cam->stats;
        pimax_cam_stats v{};
        v.frames = s.frames;
        v.tracking = s.tracking;
        v.led = s.led;
        v.unknown = s.unknown;
        v.dropped = s.dropped;
        v.corrupt = cam->cam->corrupt_frames();
        v.scanned_rows = s.scanned_rows;
        v.row_mismatch = s.row_mismatch;
        v.kind_repeats = s.kind_repeats;
        v.clock_offset_ns = cam->cam->clock_offset_ns();
        if (out == nullptr) {
            return err("null output");
        }
        *out = v;
        return 0;
    });
}

PIBRIDGE_ABI int pimax_cam_record_start(pimax_cam* cam, const char* path, uint32_t zlib_level) {
    return guard<int>(-1, [&]() -> std::expected<int, Err> {
        if (cam == nullptr) {
            return err("null handle");
        }
        const auto p = opt_str(path);
        if (!p) {
            return err("null path");
        }
        std::optional<std::uint8_t> lvl;
        if (zlib_level != 0) {
            lvl = static_cast<std::uint8_t>(std::min<std::uint32_t>(zlib_level, 9));
        }
        if (auto r = cam->cam->start_recording(*p, lvl); !r) {
            return err(std::format("{}: {}", *p, r.error().text()));
        }
        return 0;
    });
}

PIBRIDGE_ABI int64_t pimax_cam_record_stop(pimax_cam* cam) {
    return guard<std::int64_t>(-1, [&]() -> std::expected<std::int64_t, Err> {
        if (cam == nullptr) {
            return err("null handle");
        }
        auto r = cam->cam->stop_recording();
        if (!r) {
            return err(r.error().text());
        }
        return *r ? static_cast<std::int64_t>((*r)->first) : 0;
    });
}

PIBRIDGE_ABI int pimax_cam_calib_load(const char* path, pimax_cam_calib* out, char* used_path, size_t used_len) {
    return guard<int>(-1, [&]() -> std::expected<int, Err> {
        std::optional<std::string> p = opt_str(path);
        if (p && p->empty()) {
            p.reset();
        }
        auto r = DeviceCalib::load_default(p);
        if (!r) {
            return err(r.error().text());
        }
        if (out == nullptr) {
            return err("null output");
        }
        *out = to_c(r->first);
        copy_out(pibridge::cam::rfmt::utf8_lossy(r->second), used_path, used_len);
        return 0;
    });
}

PIBRIDGE_ABI int pimax_cam_calib_parse(const char* xml, size_t len, pimax_cam_calib* out) {
    return guard<int>(-1, [&]() -> std::expected<int, Err> {
        if (xml == nullptr || out == nullptr) {
            return err("null argument");
        }
        const std::string text = pibridge::cam::rfmt::utf8_lossy(std::string_view(xml, len));
        auto cal = DeviceCalib::parse(text);
        if (!cal) {
            return err(cal.error().text());
        }
        *out = to_c(*cal);
        return 0;
    });
}

PIBRIDGE_ABI int pimax_cam_calib_stereo(const pimax_cam_calib* cal, uint32_t a, uint32_t b, double r[9], double t[3]) {
    return guard<int>(-1, [&]() -> std::expected<int, Err> {
        if (cal == nullptr) {
            return err("null calib");
        }
        if (a >= cal->camera_count || b >= cal->camera_count || r == nullptr || t == nullptr) {
            return err("bad camera index or null output");
        }
        const DeviceCalib d = from_c(*cal);
        write_pose(d.stereo(a, b, RigConvention::CamFromCam0), r, t);
        return 0;
    });
}

PIBRIDGE_ABI int pimax_cam_calib_body_from_cam(const pimax_cam_calib* cal, uint32_t i, double r[9], double t[3]) {
    return guard<int>(-1, [&]() -> std::expected<int, Err> {
        if (cal == nullptr) {
            return err("null calib");
        }
        if (i >= cal->camera_count || r == nullptr || t == nullptr) {
            return err("bad camera index or null output");
        }
        const auto p = from_c(*cal).body_from_cam(i, RigConvention::CamFromCam0);
        if (!p) {
            return err("calibration has no ombc/tbc");
        }
        write_pose(*p, r, t);
        return 0;
    });
}

PIBRIDGE_ABI uint64_t pimax_cam_dev_to_mono_ns(uint64_t dev_ts_us, const pimax_cam_imu_clock_ref* ref) {
    if (ref == nullptr || ref->valid == 0) {
        return 0;
    }
    pibridge::cam::ImuClockRef r;
    r.dev_ts_u32 = ref->dev_ts_u32;
    r.valid = ref->valid;
    r.dev_ext_us = ref->dev_ext_us;
    r.offset_ns = ref->offset_ns;
    return pibridge::cam::imu_dev_to_mono_ns(dev_ts_us, r);
}
