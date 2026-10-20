// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/capi_slam.rs (MIT OR Apache-2.0).
//
// The "6DoF / SLAM" section of include/pimax_cam.h over libpibridge-cam's slam, headpose, drift and imulog: the Basalt
// calibration export, the IMU noise file, and the head pose output (pimax_cam_track_*). Same guard, error texts and
// stderr lines as the Rust library (the describe line names the build markers PIMAX_HEAD_TILT, PIMAX_FUSE_AUTO and
// PIMAX_SLAM_DRIFT_HOLD; the "6DoF drift:" report is parsed by the app).
//
// Not here: the auto-exposure entry points (ae_capi.cpp, M3). The head tracker reads the camera brightness the AE
// loop publishes there (headpose::scene, the darker pair's mean of every analysed frame); with AE off nothing
// publishes it and the brightness is "unknown", exactly as in the Rust library.
#include "pimax_cam.h"

#include "capi_common.h"

#include "pibridge/cam/calib.h"
#include "pibridge/cam/headpose.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/imulog.h"
#include "pibridge/cam/slam.h"

#include <cstddef>
#include <cstdio>
#include <cstring>
#include <format>
#include <limits>
#include <memory>

using namespace pibridge::abi;
namespace slam = pibridge::cam::slam;
namespace hp = pibridge::cam::headpose;
namespace rf = pibridge::cam::rfmt;

// The handle: supervisor + predictor + correction blending (headpose::HeadTracker), and the head IMU log
// (PIMAX_HEAD_IMU_LOG, a leaf).
struct pimax_cam_track {
    hp::HeadTracker h;
    std::unique_ptr<pibridge::cam::imulog::HeadImuLog> log;
    std::int64_t log_vio_ts = std::numeric_limits<std::int64_t>::min();

    void log_vio(const std::optional<hp::VioSample>& s) {
        if (log && s && s->ts_ns != log_vio_ts) {
            log_vio_ts = s->ts_ns;
            log->vio(s->ts_ns, s->pos, s->q);
        }
    }
};

namespace {

#define PIBRIDGE_SAME_FIELD(T, U, f) static_assert(offsetof(T, f) == offsetof(U, f), #f)
// pimax_cam_track_params is slam::SupervisorParams (one #[repr(C)] struct in the Rust code).
static_assert(sizeof(slam::SupervisorParams) == sizeof(pimax_cam_track_params));
PIBRIDGE_SAME_FIELD(slam::SupervisorParams, pimax_cam_track_params, max_speed_m_s);
PIBRIDGE_SAME_FIELD(slam::SupervisorParams, pimax_cam_track_params, max_rot_mismatch_deg);
PIBRIDGE_SAME_FIELD(slam::SupervisorParams, pimax_cam_track_params, acquire_poses);
PIBRIDGE_SAME_FIELD(slam::SupervisorParams, pimax_cam_track_params, reserved);
PIBRIDGE_SAME_FIELD(slam::SupervisorParams, pimax_cam_track_params, stale_ns);
PIBRIDGE_SAME_FIELD(slam::SupervisorParams, pimax_cam_track_params, reset_after_ns);
PIBRIDGE_SAME_FIELD(slam::SupervisorParams, pimax_cam_track_params, reset_every_ns);
static_assert(PIMAX_CAM_TRACK_TRACKING == static_cast<unsigned>(slam::TrackState::Tracking));
static_assert(PIMAX_CAM_TRACK_LOST == static_cast<unsigned>(slam::TrackState::Lost));
static_assert(PIMAX_CAM_TRACK_EV_ACQUIRED == slam::events::ACQUIRED && PIMAX_CAM_TRACK_EV_LOST == slam::events::LOST);
static_assert(PIMAX_CAM_TRACK_EV_RESET_WANTED == slam::events::RESET_WANTED);
static_assert(PIMAX_CAM_TRACK_EV_RECENTERED == slam::events::RECENTERED);
static_assert(PIMAX_CAM_TRACK_EV_RESNAPPED == slam::events::RESNAPPED);
static_assert(PIMAX_CAM_TRACK_EV_REZEROED == slam::events::REZEROED);

slam::SupervisorParams params_from_c(const pimax_cam_track_params& c) {
    slam::SupervisorParams p;
    p.max_speed_m_s = c.max_speed_m_s;
    p.max_rot_mismatch_deg = c.max_rot_mismatch_deg;
    p.acquire_poses = c.acquire_poses;
    p.reserved = c.reserved;
    p.stale_ns = c.stale_ns;
    p.reset_after_ns = c.reset_after_ns;
    p.reset_every_ns = c.reset_every_ns;
    return p;
}

template <std::size_t N>
std::array<double, N> arr(const double (&a)[N]) {
    std::array<double, N> o{};
    for (std::size_t i = 0; i < N; i++) {
        o[i] = a[i];
    }
    return o;
}

template <std::size_t N>
void put(double (&dst)[N], const std::array<double, N>& a) {
    for (std::size_t i = 0; i < N; i++) {
        dst[i] = a[i];
    }
}

pimax_cam_slam_imu imu_to_c(const slam::ImuNoise& n) {
    pimax_cam_slam_imu c;
    std::memset(&c, 0, sizeof c);
    put(c.accel_noise_std, n.accel_noise_std);
    put(c.gyro_noise_std, n.gyro_noise_std);
    put(c.accel_bias_std, n.accel_bias_std);
    put(c.gyro_bias_std, n.gyro_bias_std);
    c.rate_hz = n.rate_hz;
    const std::size_t len = std::min<std::size_t>(n.source.size(), 255);
    std::memcpy(c.source, n.source.data(), len);
    return c;
}

slam::ImuNoise imu_from_c(const pimax_cam_slam_imu& c) {
    slam::ImuNoise n;
    n.accel_noise_std = arr(c.accel_noise_std);
    n.gyro_noise_std = arr(c.gyro_noise_std);
    n.accel_bias_std = arr(c.accel_bias_std);
    n.gyro_bias_std = arr(c.gyro_bias_std);
    n.rate_hz = c.rate_hz;
    std::size_t len = 0;
    while (len < sizeof c.source && c.source[len] != 0) {
        len++;
    }
    n.source = rf::utf8_lossy(std::string_view(c.source, len));
    return n;
}

pimax_cam_track_out out_to_c(const pimax_cam_track& t, const slam::TrackOutput& o) {
    pimax_cam_track_out c;
    std::memset(&c, 0, sizeof c);
    c.state = static_cast<std::uint32_t>(o.state);
    c.events = o.events;
    put(c.world_from_slam_pos, o.world_from_slam.first);
    put(c.world_from_slam_q, o.world_from_slam.second);
    put(c.fix3dof_q, o.fix3dof);
    put(c.held_pos, o.held_pos);
    c.acquisitions = t.h.sup.acquisitions;
    c.losses = t.h.sup.losses;
    c.resets = t.h.sup.resets;
    c.rejects = t.h.sup.rejects;
    const std::size_t len = std::min<std::size_t>(std::strlen(o.reason), 47);
    std::memcpy(c.reason, o.reason, len);
    return c;
}

// eprintln!: one line on stderr (Monado's log).
void eprintln(const std::string& s) {
    const std::string line = s + "\n";
    std::fwrite(line.data(), 1, line.size(), stderr);
    std::fflush(stderr);
}

void drift_report(pimax_cam_track& t, std::uint64_t now_ns) {
    if (const auto r = t.h.drift.take_report(static_cast<std::int64_t>(now_ns))) {
        // Monado's log (stderr); the app reads it (omarchyvr-gui-core monlog).
        eprintln(pibridge::cam::drift::report_line(*r));
    }
}

} // namespace

PIBRIDGE_ABI int pimax_cam_slam_imu_load(const char* path, pimax_cam_slam_imu* out) {
    if (out == nullptr) {
        return -1;
    }
    try {
        const auto p = opt_str(path);
        *out = imu_to_c(slam::ImuNoise{});
        return guard<int>(-1, [&]() -> std::expected<int, Err> {
            auto r = slam::ImuNoise::load(p);
            if (!r) {
                return err(r.error());
            }
            if (*r) {
                *out = imu_to_c(**r);
                return 1;
            }
            return 0;
        });
    } catch (...) {
        return -1;
    }
}

PIBRIDGE_ABI int pimax_cam_slam_t_imu_cam(const pimax_cam_calib* cal, uint32_t cam, double out[16]) {
    return guard<int>(-1, [&]() -> std::expected<int, Err> {
        if (cal == nullptr) {
            return err("null calib");
        }
        if (out == nullptr) {
            return err("null output");
        }
        const auto p = slam::t_imu_cam(from_c(*cal), cam);
        if (!p) {
            return err("no camera-IMU extrinsic or bad camera id");
        }
        for (int i = 0; i < 16; i++) {
            out[i] = 0.0;
        }
        for (int col = 0; col < 3; col++) {
            for (int row = 0; row < 3; row++) {
                out[col * 4 + row] = p->r[row][col];
            }
        }
        out[12] = p->t[0];
        out[13] = p->t[1];
        out[14] = p->t[2];
        out[15] = 1.0;
        return 0;
    });
}

PIBRIDGE_ABI int pimax_cam_slam_default_head(const pimax_cam_calib* cal, double out[3]) {
    return guard<int>(-1, [&]() -> std::expected<int, Err> {
        if (cal == nullptr) {
            return err("null calib");
        }
        if (out == nullptr) {
            return err("null output");
        }
        const auto h = slam::default_head_in_imu(from_c(*cal));
        out[0] = h[0];
        out[1] = h[1];
        out[2] = h[2];
        return 0;
    });
}

PIBRIDGE_ABI int pimax_cam_slam_parse_cams(const char* s, uint32_t count, uint32_t* out, uint32_t cap) {
    return guard<int>(-1, [&]() -> std::expected<int, Err> {
        const auto str = opt_str(s);
        if (!str) {
            return err("null camera list");
        }
        auto v = slam::parse_cams(*str, count);
        if (!v) {
            return err(v.error());
        }
        if (out == nullptr || v->size() > cap) {
            return err(std::format("camera list {}: more than {} cameras", rf::debug_str(*str), cap));
        }
        for (std::size_t k = 0; k < v->size(); k++) {
            out[k] = static_cast<std::uint32_t>((*v)[k]);
        }
        return static_cast<int>(v->size());
    });
}

PIBRIDGE_ABI int pimax_cam_slam_write_basalt(const pimax_cam_calib* cal, const uint32_t* cams, uint32_t n, double cam_hz,
                                          int64_t cam_time_offset_ns, const pimax_cam_slam_imu* imu, const char* dir,
                                          const char* note) {
    return guard<int>(-1, [&]() -> std::expected<int, Err> {
        if (cal == nullptr) {
            return err("null calib");
        }
        const slam::ImuNoise noise = imu != nullptr ? imu_from_c(*imu) : slam::ImuNoise{};
        const auto d = opt_str(dir);
        if (!d) {
            return err("null dir");
        }
        if (cams == nullptr || n == 0) {
            return err("no cameras");
        }
        const pibridge::cam::DeviceCalib c = from_c(*cal);
        slam::SlamSetup setup;
        setup.cams.assign(cams, cams + n);
        setup.cam_hz = cam_hz;
        setup.cam_time_offset_ns = cam_time_offset_ns;
        // Same seed as `omarchyvr-slam export` for live runs: the bias the driver cached from its last still window.
        setup.gyro_bias = slam::gyro_bias_cache().value_or(slam::Vec3{0.0, 0.0, 0.0});
        // The C calibration has no raw Stateinit attributes (capi.rs from_c), so this is always zero here: the
        // recorder's calib carries no accel bias (Rust; parity mode).
        setup.accel_bias = slam::factory_accel_bias(c).value_or(slam::Vec3{0.0, 0.0, 0.0});
        const std::string nt = opt_str(note).value_or("");
        // PiBridge (PIMAX_SLAM_RECORD_ACCEL_BIAS, on unless PIBRIDGE_HEAD_PARITY=1): the recording carries the calibration
        // the tracker really uses, so a replay of the take is the session. With a SLAM_CONFIG (Monado then reads the
        // sensor calibration from its cam-calib file, not from this one) that file is copied as it is and the info
        // file names it and its biases; without one the factory aBias is taken from the factory XML itself (the
        // default search, $PIMAX_CAM_CALIB first: the file the driver loaded).
        bool used = !pibridge::cam::headpose::parity_mode();
        if (const auto v = pibridge::cam::env_utf8("PIMAX_SLAM_RECORD_ACCEL_BIAS")) {
            double x = 0;
            if (rf::parse_f64(rf::trim(*v), x)) {
                used = x != 0.0;
            }
        }
        std::vector<std::pair<std::string, std::string>> extra{{"note", nt}};
        std::optional<std::string> used_text;
        if (used) {
            std::string source = "this file (the driver's calibration)";
            if (const auto path = slam::slam_config_calib_path()) {
                if (auto t = pibridge::cam::read_to_string(*path)) {
                    const slam::CalibBiases b = slam::calib_json_biases(*t);
                    setup.accel_bias = b.accel.value_or(slam::Vec3{0.0, 0.0, 0.0});
                    setup.gyro_bias = b.gyro.value_or(setup.gyro_bias);
                    used_text = std::move(*t);
                    source = "copy of the SLAM_CONFIG calibration " + *path;
                }
            }
            if (!used_text && !slam::factory_accel_bias(c)) {
                if (const auto f = pibridge::cam::DeviceCalib::load_default(std::nullopt); f && !f->first.is_reference()) {
                    if (const auto a = slam::factory_accel_bias(f->first)) {
                        setup.accel_bias = *a;
                        source = "the driver's calibration with the factory aBias of " + f->second;
                    }
                }
            }
            extra.emplace_back("calib_source", source);
        }
        auto r = slam::write_basalt_files(*d, c, setup, noise, extra);
        if (!r) {
            return err(r.error());
        }
        if (used_text) {
            // write_basalt_files made the directory and wrote the driver's version; replace it with the one in use.
            if (auto w = pibridge::cam::write_file(*r, *used_text); !w) {
                return err(std::format("{}: {}", *r, w.error().text()));
            }
        }
        return 0;
    });
}

PIBRIDGE_ABI pimax_cam_track* pimax_cam_track_create(const pimax_cam_track_params* params, const double head_in_imu[3]) {
    return guard<pimax_cam_track*>(nullptr, [&]() -> std::expected<pimax_cam_track*, Err> {
        const slam::SupervisorParams p = params != nullptr ? params_from_c(*params) : slam::SupervisorParams{};
        slam::Vec3 h{0.0, 0.0, 0.0};
        if (head_in_imu != nullptr) {
            h = {head_in_imu[0], head_in_imu[1], head_in_imu[2]};
        }
        hp::HeadParams hpar = hp::HeadParams::from_env();
        // Once per session, on stderr (Monado's log): names the automatic output stages (build markers).
        eprintln(hpar.describe());
        auto* t = new pimax_cam_track{hp::HeadTracker(p, std::move(hpar), h), pibridge::cam::imulog::HeadImuLog::from_env(),
                                      std::numeric_limits<std::int64_t>::min()};
        return t;
    });
}

PIBRIDGE_ABI void pimax_cam_track_default_params(pimax_cam_track_params* out) {
    if (out != nullptr) {
        const slam::SupervisorParams p;
        out->max_speed_m_s = p.max_speed_m_s;
        out->max_rot_mismatch_deg = p.max_rot_mismatch_deg;
        out->acquire_poses = p.acquire_poses;
        out->reserved = p.reserved;
        out->stale_ns = p.stale_ns;
        out->reset_after_ns = p.reset_after_ns;
        out->reset_every_ns = p.reset_every_ns;
    }
}

PIBRIDGE_ABI void pimax_cam_track_destroy(pimax_cam_track* t) {
    try {
        delete t;
    } catch (...) {
    }
}

PIBRIDGE_ABI void pimax_cam_track_recenter(pimax_cam_track* t) {
    if (t != nullptr) {
        t->h.sup.request_recenter();
    }
}

PIBRIDGE_ABI int pimax_cam_track_update(pimax_cam_track* t, uint64_t now_ns, const pimax_cam_track_sample* sample,
                                     pimax_cam_track_out* out) {
    return guard<int>(-1, [&]() -> std::expected<int, Err> {
        if (t == nullptr) {
            return err("null tracker");
        }
        // Phase-1 entry point: no velocity (the C side used Monado's own prediction with it).
        std::optional<hp::VioSample> s;
        if (sample != nullptr) {
            s = hp::VioSample{static_cast<std::int64_t>(sample->ts_ns), arr(sample->pos), arr(sample->q),
                              {0.0, 0.0, 0.0}, arr(sample->q3dof)};
        }
        t->log_vio(s);
        const slam::TrackOutput o = t->h.update(static_cast<std::int64_t>(now_ns), s);
        drift_report(*t, now_ns);
        if (out != nullptr) {
            *out = out_to_c(*t, o);
        }
        return 0;
    });
}

PIBRIDGE_ABI void pimax_cam_track_push_imu(pimax_cam_track* t, int64_t ts_ns, const double accel[3], const double gyro[3]) {
    if (t == nullptr || accel == nullptr || gyro == nullptr) {
        return;
    }
    try {
        const slam::Vec3 a{accel[0], accel[1], accel[2]};
        const slam::Vec3 g{gyro[0], gyro[1], gyro[2]};
        if (t->log) {
            t->log->imu(ts_ns, a, g);
        }
        t->h.push_imu(hp::ImuSample{ts_ns, a, g});
    } catch (...) {
    }
}

PIBRIDGE_ABI int pimax_cam_track_update_vio(pimax_cam_track* t, uint64_t now_ns, const pimax_cam_track_vio* sample,
                                         pimax_cam_track_out* out) {
    return guard<int>(-1, [&]() -> std::expected<int, Err> {
        if (t == nullptr) {
            return err("null tracker");
        }
        std::optional<hp::VioSample> s;
        if (sample != nullptr) {
            s = hp::VioSample{static_cast<std::int64_t>(sample->ts_ns), arr(sample->pos), arr(sample->q),
                              arr(sample->vel), arr(sample->q3dof)};
        }
        t->log_vio(s);
        t->h.set_scene_mean(hp::scene::latest(1000));
        const slam::TrackOutput o = t->h.update(static_cast<std::int64_t>(now_ns), s);
        drift_report(*t, now_ns);
        if (out != nullptr) {
            *out = out_to_c(*t, o);
        }
        return 0;
    });
}

PIBRIDGE_ABI int pimax_cam_track_head(pimax_cam_track* t, uint64_t now_ns, int64_t at_ns, const pimax_cam_rel* rel3dof,
                                   pimax_cam_rel* out) {
    return guard<int>(-1, [&]() -> std::expected<int, Err> {
        if (t == nullptr) {
            return err("null tracker");
        }
        if (rel3dof == nullptr) {
            return err("null 3DoF relation");
        }
        if (out == nullptr) {
            return err("null output");
        }
        hp::Rel r;
        r.pos = arr(rel3dof->pos);
        r.q = arr(rel3dof->q);
        r.lin_vel = arr(rel3dof->lin_vel);
        r.ang_vel = arr(rel3dof->ang_vel);
        r.flags = rel3dof->flags;
        const hp::Rel o = t->h.head(static_cast<std::int64_t>(now_ns), at_ns, r);
        pimax_cam_rel c;
        std::memset(&c, 0, sizeof c);
        put(c.pos, o.pos);
        put(c.q, o.q);
        put(c.lin_vel, o.lin_vel);
        put(c.ang_vel, o.ang_vel);
        c.flags = o.flags;
        c.reserved = 0;
        *out = c;
        return static_cast<int>(t->h.output().state);
    });
}

PIBRIDGE_ABI int pimax_cam_track_get_info(const pimax_cam_track* t, pimax_cam_track_info* out) {
    return guard<int>(-1, [&]() -> std::expected<int, Err> {
        if (t == nullptr) {
            return err("null tracker");
        }
        if (out == nullptr) {
            return err("null output");
        }
        const auto [cp, cr] = t->h.correction();
        pimax_cam_track_info c;
        std::memset(&c, 0, sizeof c);
        put(c.gyro_bias, t->h.pred.gyro_bias);
        put(c.corr_pos, cp);
        put(c.corr_rot, cr);
        c.max_corr_pos = t->h.corr.max_pos;
        c.max_corr_rot = t->h.corr.max_rot;
        c.blended = t->h.corr.blended;
        c.snapped = t->h.corr.snapped;
        c.last_imu_ns = t->h.pred.last_imu_ns().value_or(0);
        *out = c;
        return 0;
    });
}

PIBRIDGE_ABI int pimax_cam_track_take_big_corr(pimax_cam_track* t, pimax_cam_track_corr* out) {
    return guard<int>(-1, [&]() -> std::expected<int, Err> {
        if (t == nullptr) {
            return err("null tracker");
        }
        if (out == nullptr) {
            return err("null output");
        }
        const auto e = t->h.take_big();
        if (!e) {
            return 0;
        }
        pimax_cam_track_corr c;
        std::memset(&c, 0, sizeof c);
        c.ts_ns = e->ts_ns;
        put(c.step, e->step);
        put(c.offset, e->offset);
        c.rot = e->rot;
        c.speed = e->speed;
        c.ang_speed = e->ang_speed;
        c.pose_only = e->pose_only ? 1u : 0u;
        c.reserved = 0;
        *out = c;
        return 1;
    });
}
