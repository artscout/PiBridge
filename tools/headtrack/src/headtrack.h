// SPDX-License-Identifier: MIT
// pibridge-headtrack: the Crystal Super's / Dream Air's inside-out head tracking without Monado. It owns the headset's
// HID while it runs (wake, keepalive, video, the IMU), takes the tracking cameras from the camera broker, runs a VIT
// SLAM engine (libpibridge_vit_winbridge.so: Pimax Play's engine under Wine) and writes the head pose to the pose file
// CustomHeadsetOpenVR's SLAM headset reads (libpibridge-hmd pose_shm.h, source SOURCE_HEADTRACK).
//
// What Monado's Pimax driver did, here (third_party/monado/src/xrt/drivers/pimax/):
//   pimax_hmd.c sample_time_ns / map_axes  -> ImuTime / on_imu (IMU device time to CLOCK_MONOTONIC, OpenXR axes)
//   pimax_camera.c frame_time / push_slam_frames -> camera_loop (frames on the IMU timeline, one time per set)
//   pimax_camera.c slam_imu_gate_locked    -> VitHost::push_imu
//   pimax_camera.c passthrough_head        -> head_from_engine / lost_head (the engine's pose as is; without a pose
//                                             for 350 ms the 3DoF orientation at the last position)
//   pimax_hmd.c pimax_on_log               -> on_log (the volume buttons to the paused pibridge-keepalived)
// The pose is written with its own sample time and velocities; SteamVR predicts it to the display time
// (CustomHeadsetOpenVR's poseTimeOffset), as driver_monado's poses were predicted by SteamVR too.
#pragma once

#include "vit_host.h"
#include "world.h"

#include "pibridge/cam/clock.h"
#include "pibridge/cam/headpose.h"
#include "pibridge/ctrl/fuse.h"
#include "pibridge/ctrl/track/shm.h"
#include "pibridge/cam/slam.h"
#include "pibridge/hmd/pose_shm.h"
#include "pibridge/hmd/haptics.h"
#include "pibridge/hmd/session.h"

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace pibridge::headtrack {

using cam::slam::Quat;
using cam::slam::Vec3;

struct Options {
    /// The VIT library (the SLAM engine).
    std::string vit_library;
    /// SLAM_CONFIG for engines that read one (Basalt); empty for the Pimax engine.
    std::string slam_config;
    /// The engine: "pimax" (its pose as is, controllers by its LED engine) or "basalt" (our output layer,
    /// libpibridge-cam headpose::HeadTracker; controllers by pibridge-ctrltrack's pose file and libpibridge-ctrl's fuser).
    std::string engine = "pimax";
    /// Frame time = mid-exposure (Basalt; Pimax's engine adds its own camera-IMU delta).
    bool exposure_mid = false;
    /// Camera source spec (libpibridge-cam open_source): "broker" = the camera broker's default socket.
    std::string camera = "broker";
    /// Tiles handed to the engine (PIMAX_SLAM_CAMS).
    std::vector<std::uint32_t> cams{0, 1, 2, 3};
    /// Panel rate (refresh_hz).
    std::uint8_t fps = 90;
    /// Camera-time offset added to every frame set (PIMAX_SLAM_CAM_OFFSET_US), ns.
    std::int64_t cam_offset_ns = 0;
    std::filesystem::path pose_file = hmd::pose_shm::default_path();
    /// Factory calibration (head origin); empty = the default search.
    std::string calibration;
};

/// The IMU's device clock (u32 us) on CLOCK_MONOTONIC: unwrapped, the least delayed offset, relaxed by 1/16384 per
/// sample to follow drift (pimax_hmd.c sample_time_ns). Also the reference the camera frames are mapped with.
class ImuTime {
public:
    std::int64_t map(std::uint32_t dev_us, std::int64_t now_ns);
    cam::ImuClockRef ref() const;

private:
    mutable std::mutex mu_;
    bool have_ = false;
    std::uint64_t ext_us_ = 0;
    std::uint32_t last_dev_ = 0;
    std::int64_t offset_ns_ = 0;
    std::int64_t last_ns_ = 0;
    cam::ImuClockRef ref_;
};

/// 3DoF orientation (the fallback while the engine has no pose): Mahony from the gyro and gravity (y up, the head's
/// OpenXR axes), as tools/slam headreplay; the gyro bias learnt whenever the headset rests.
class Orientation {
public:
    void update(std::int64_t ts_ns, const Vec3& accel, const Vec3& gyro);
    bool valid() const { return init_; }
    Quat q() const { return q_; }
    /// The bias-corrected gyro of the last sample (body frame).
    Vec3 gyro() const { return w_; }

private:
    bool init_ = false;
    Quat q_ = cam::slam::QUAT_ID;
    Vec3 w_{};
    Vec3 bias_{};
    std::int64_t last_ts_ = 0;
    // rest detection for the bias
    Vec3 rest_sum_{};
    int rest_n_ = 0;
};

/// One controller: its last HID report, its own 3DoF (the hold between engine poses), the last engine aim pose.
struct Hand {
    std::mutex mu;
    std::optional<hmd::ControllerReport> report;
    std::int64_t report_ns = 0;
    Orientation imu; // the controller IMU in its raw axes (only its changes are used)
    // output thread only
    bool eng_have = false;
    Vec3 eng_pos{};
    Quat eng_aim = cam::slam::QUAT_ID;
    Quat eng_imu = cam::slam::QUAT_ID; // the 3DoF at the last engine pose
    Quat eng_dfi = cam::slam::QUAT_ID;
    std::int64_t eng_last_ns = 0;
    int logged_state = -1;
    std::int64_t diag_last_ns = 0; // throttles the controller prediction diagnostic (~30 Hz)
    // the Pimax button: a long press (0.8 s) recenters (session thread)
    std::int64_t pimax_down_ns = 0;
    bool recentered = false;
};

class Daemon {
public:
    Daemon(Options o, std::function<void(std::string_view)> say);
    ~Daemon();
    /// Opens the HID session, the engine and the camera; false with the reason said when it cannot.
    bool start();
    void stop();

private:
    class Hid;
    void on_imu(const hmd::ImuSample& s);
    void on_controller(const hmd::ControllerReport& r);
    hmd::pose_shm::Device controller(int side, std::int64_t now_ns);
    /// The controller angle (PIMAX_CTRL_ANGLE, live file) on an aim pose (output thread).
    void aim_out(hmd::pose_shm::Device& d, std::int64_t now_ns);
    double ctrl_angle_deg_ = 30.0;
    // Forward prediction cap of the controller pose, seconds (PIMAX_CTRL_PREDICT_MS, default 25 ms = pimax_ctrl.c).
    // 0 reports it at the sample time and lets SteamVR do all the prediction, as the head does (A/B for the "bending").
    double ctrl_predict_s_ = 0.025;
    std::int64_t angle_polled_ns_ = 0;
    void on_log(std::string_view line);
    void camera_loop();
    void output_loop();
    hmd::pose_shm::Device head_from_engine(const EnginePose& e);
    hmd::pose_shm::Device lost_head(std::int64_t now_ns);

    Options o_;
    std::function<void(std::string_view)> say_;
    std::unique_ptr<hmd::Session> session_;
    std::unique_ptr<VitHost> vit_;
    std::unique_ptr<World> world_;
    // Basalt: the output layer and the optical controllers
    std::mutex ht_mu_;
    std::unique_ptr<cam::headpose::HeadTracker> ht_;
    std::unique_ptr<ctrl::fuse::Fuser> fuser_[2];
    std::unique_ptr<ctrl::track::shm::PoseReader> ctrl_reader_;
    std::uint64_t ctrl_seen_[2] = {0, 0};
    std::int64_t ctrl_open_at_ = 0, ht_poll_at_ = 0;
    hmd::pose_shm::Device basalt_head(std::int64_t now_ns, std::uint32_t& tracking);
    void optical_fixes(std::int64_t now_ns);
    /// Poll the Pimax engine's room state (floor + localization) at a low rate, write room-state.json and log changes
    /// (output thread). This is the observe-and-pin-down step for GetGroundState / GetLocModeState.
    void room_state_poll(std::int64_t now_ns);
    std::int64_t room_poll_at_ = 0;
    bool room_logged_ = false;
    std::int32_t room_loc_ret_ = -3, room_ground_ret_ = -3;
    std::uint8_t room_loc_state_ = 0, room_ground_flag_ = 0;
    // the last published head height (y-up world; y is the engine z, unaffected by the x/z origin shift and the yaw
    // recenter) so room_state_poll can turn the engine-frame floor points into a head-relative drop
    float room_head_y_ = 0;
    bool room_head_valid_ = false;
    /// The room offset and the recenter on a head pose (output thread).
    void to_world(hmd::pose_shm::Device& d, bool position_valid, std::int64_t now_ns);
    std::optional<hmd::pose_shm::Publisher> pub_;
    ImuTime time_;
    std::mutex imu_mu_;
    Orientation orient_;
    std::int64_t last_imu_ns_ = 0;
    Vec3 head_in_imu_{};
    std::atomic<bool> running_{false};
    std::atomic<bool> need_video_{false};
    std::thread camera_;
    std::thread output_;
    std::thread serials_;
    void serials_loop();
    // the engine pose turned into the head pose (output thread only)
    bool zeroed_ = false;
    Quat world_q_ = cam::slam::QUAT_ID;
    Vec3 world_t_{};
    Quat fix_ = cam::slam::QUAT_ID; // head = fix * 3DoF, kept for a loss
    Vec3 held_{};
    bool tracking_ = false;
    std::uint64_t acquired_ = 0, lost_ = 0, sample_ = 0;
    bool own_marker_ = false;
    Hand hands_[2];
    /// The head poses published lately (output thread only): the controllers go under the head at their sample time.
    struct HeadAt {
        std::int64_t ts = 0;
        Vec3 pos{};
        Quat q = cam::slam::QUAT_ID;
    };
    std::vector<HeadAt> head_hist_;
    std::size_t head_next_ = 0;
    std::optional<HeadAt> head_at(std::int64_t ts) const;
    std::uint64_t ctrl_sample_[2] = {0, 0};
    std::uint64_t haptic_requests_ = 0; // vibration requests taken (output thread)
    std::int64_t haptic_logged_ns_ = 0;
    hmd::haptics::Receiver haptic_rx_; // CustomHeadsetOpenVR's pulses
};

} // namespace pibridge::headtrack
