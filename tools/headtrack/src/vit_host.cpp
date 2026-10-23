// SPDX-License-Identifier: MIT
#include "vit_host.h"

#include "pibridge_winbridge_ext.h"
#include "vit/vit_interface.h"

#include <cstring>
#include <dlfcn.h>

namespace pibridge::headtrack {

#define VIT_SYMS(X)                                                                                                    \
    X(api_get_version) X(tracker_create) X(tracker_destroy) X(tracker_has_image_format)                                 \
    X(tracker_get_supported_extensions) X(tracker_get_enabled_extensions) X(tracker_enable_extension)                   \
    X(tracker_start) X(tracker_stop) X(tracker_reset) X(tracker_is_running) X(tracker_push_imu_sample)                  \
    X(tracker_push_img_sample) X(tracker_add_imu_calibration) X(tracker_add_camera_calibration) X(tracker_pop_pose)     \
    X(tracker_get_timing_titles) X(pose_destroy) X(pose_get_data) X(pose_get_timing) X(pose_get_features)

struct VitHost::Api {
#define VIT_DECL(n) PFN_vit_##n n = nullptr;
    VIT_SYMS(VIT_DECL)
#undef VIT_DECL
};

std::unique_ptr<VitHost> VitHost::open(const std::string& library, std::uint32_t cameras, const std::string& config,
                                       std::string* error) {
    std::unique_ptr<VitHost> h(new VitHost);
    h->lib_ = ::dlopen(library.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (h->lib_ == nullptr) {
        *error = std::string("dlopen: ") + ::dlerror();
        return nullptr;
    }
    h->api_ = std::make_unique<Api>();
#define VIT_LOAD(n)                                                                                                    \
    h->api_->n = reinterpret_cast<PFN_vit_##n>(::dlsym(h->lib_, "vit_" #n));                                          \
    if (h->api_->n == nullptr) {                                                                                       \
        *error = library + ": no vit_" #n;                                                                             \
        return nullptr;                                                                                                \
    }
    VIT_SYMS(VIT_LOAD)
#undef VIT_LOAD
    std::uint32_t major = 0, minor = 0, patch = 0;
    h->api_->api_get_version(&major, &minor, &patch);
    if (major != VIT_HEADER_VERSION_MAJOR) {
        *error = library + ": VIT " + std::to_string(major) + "." + std::to_string(minor) + ", not " +
                 std::to_string(VIT_HEADER_VERSION_MAJOR) + ".x";
        return nullptr;
    }
    vit_config_t cfg{config.empty() ? nullptr : config.c_str(), cameras, 1, false};
    vit_tracker_t* t = nullptr;
    if (h->api_->tracker_create(&cfg, &t) != VIT_SUCCESS || t == nullptr) {
        *error = library + ": tracker_create failed";
        return nullptr;
    }
    h->tracker_ = t;
    if (h->api_->tracker_start(t) != VIT_SUCCESS) {
        *error = library + ": tracker_start failed";
        return nullptr;
    }
    h->cameras_ = cameras;
    // winbridge's controller extension, when the library is libpibridge_vit_winbridge.so
    auto ver = reinterpret_cast<pibridge_wb_ext_version_fn>(::dlsym(h->lib_, "pibridge_wb_ext_version"));
    h->ext_led_ = ::dlsym(h->lib_, "pibridge_wb_push_led");
    h->ext_imu_ = ::dlsym(h->lib_, "pibridge_wb_push_ctrl_imu");
    h->ext_pose_ = ::dlsym(h->lib_, "pibridge_wb_ctrl_pose");
    h->ext_ok_ = ver != nullptr && ver() == PIBRIDGE_WB_EXT_VERSION && h->ext_led_ && h->ext_imu_ && h->ext_pose_;
    return h;
}

VitHost::~VitHost() {
    if (tracker_ != nullptr && api_) {
        auto* t = static_cast<vit_tracker_t*>(tracker_);
        api_->tracker_stop(t);
        api_->tracker_destroy(t);
    }
    // the library stays loaded: a VIT library's worker threads may outlive tracker_destroy by a moment
}

bool VitHost::push_imu(std::int64_t ts_ns, const float accel[3], const float gyro[3]) {
    {
        std::lock_guard g(gate_mu_);
        if (!frame_seen_) {
            return false; // nothing before the first frame set
        }
        std::uint32_t n = since_frame_++;
        bool feed = n < IMU_FULL;
        if (!feed) {
            n -= IMU_FULL;
            feed = n < IMU_SPARSE * IMU_DECIM && n % IMU_DECIM == 0;
        }
        if (!feed) {
            imu_held_++;
            return false;
        }
    }
    vit_imu_sample_t s{};
    s.timestamp = ts_ns;
    s.ax = accel[0];
    s.ay = accel[1];
    s.az = accel[2];
    s.wx = gyro[0];
    s.wy = gyro[1];
    s.wz = gyro[2];
    api_->tracker_push_imu_sample(static_cast<vit_tracker_t*>(tracker_), &s);
    return true;
}

void VitHost::push_frames(std::int64_t ts_ns, const std::vector<const std::uint8_t*>& tiles, std::uint32_t w,
                          std::uint32_t h) {
    for (std::uint32_t c = 0; c < cameras_ && c < tiles.size(); c++) {
        vit_img_sample_t s{};
        s.cam_index = c;
        s.timestamp = ts_ns;
        s.data = const_cast<std::uint8_t*>(tiles[c]);
        s.width = w;
        s.height = h;
        s.stride = w;
        s.size = w * h;
        s.format = VIT_IMAGE_FORMAT_L8;
        api_->tracker_push_img_sample(static_cast<vit_tracker_t*>(tracker_), &s);
    }
    std::lock_guard g(gate_mu_);
    frame_seen_ = true;
    since_frame_ = 0;
}

void VitHost::reset() {
    api_->tracker_reset(static_cast<vit_tracker_t*>(tracker_));
}

std::vector<EnginePose> VitHost::poses() {
    std::vector<EnginePose> out;
    vit_pose_t* p = nullptr;
    while (api_->tracker_pop_pose(static_cast<vit_tracker_t*>(tracker_), &p) == VIT_SUCCESS && p != nullptr) {
        vit_pose_data_t d{};
        api_->pose_get_data(p, &d);
        out.push_back(EnginePose{d.timestamp, {d.px, d.py, d.pz}, {d.ox, d.oy, d.oz, d.ow}, {d.vx, d.vy, d.vz}});
        api_->pose_destroy(p);
        p = nullptr;
    }
    return out;
}

void VitHost::push_led(std::int64_t ts_ns, std::uint32_t shutter_ns, const std::vector<const std::uint8_t*>& tiles) {
    if (!ext_ok_ || tiles.size() != 4) {
        return;
    }
    const std::uint32_t strides[4] = {640, 640, 640, 640};
    reinterpret_cast<pibridge_wb_push_led_fn>(ext_led_)(ts_ns, shutter_ns, 4, tiles.data(), strides);
}

void VitHost::push_controller_imu(std::uint32_t hand, std::int64_t ts_ns, const float accel[3], const float gyro[3]) {
    if (ext_ok_) {
        reinterpret_cast<pibridge_wb_push_ctrl_imu_fn>(ext_imu_)(hand, ts_ns, accel, gyro);
    }
}

std::optional<EngineControllerPose> VitHost::controller_pose(std::uint32_t hand) {
    if (!ext_ok_) {
        return std::nullopt;
    }
    pibridge_wb_ctrl_pose p{};
    if (reinterpret_cast<pibridge_wb_ctrl_pose_fn>(ext_pose_)(hand, &p) != 1) {
        return std::nullopt;
    }
    EngineControllerPose e;
    e.ts_ns = p.ts_ns;
    std::memcpy(e.q, p.q, sizeof e.q);
    std::memcpy(e.p, p.p, sizeof e.p);
    std::memcpy(e.ang_vel, p.ang_vel, sizeof e.ang_vel);
    std::memcpy(e.lin_vel, p.lin_vel, sizeof e.lin_vel);
    std::memcpy(e.head_q, p.head_q, sizeof e.head_q);
    std::memcpy(e.head_p, p.head_p, sizeof e.head_p);
    std::memcpy(e.dev_from_imu, p.dev_from_imu, sizeof e.dev_from_imu);
    return e;
}

} // namespace pibridge::headtrack
