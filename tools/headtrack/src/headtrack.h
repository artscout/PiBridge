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

#include "sysmon.h"
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
    /// Head shadow mode (--shadow-vit): a second engine (Basalt) on the same frames and IMU, its raw poses logged
    /// next to the driving engine's (head-shadow-*.csv); it drives nothing.
    std::string shadow_vit;
    std::string shadow_slam_config;
    std::vector<std::uint32_t> shadow_cams{0, 1};
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
    /// The controller IMU's time for Pimax's LED engine (on_controller): this hand's own device-clock mapping (used when
    /// the report's ts_us is not on the headset IMU's clock), the last time sent, and the source last logged.
    ImuTime dev_time;
    std::int64_t eng_ts_ns = 0;
    int eng_ts_mode = -1;
    /// This controller's IMU sample period (the second sample of a report is one period after the first).
    hmd::ControllerSamplePeriod period;
    std::optional<hmd::ControllerReport> report;
    std::int64_t report_ns = 0;
    std::uint32_t reports = 0; // reports received (the shadow log's IMU rate)
    Orientation imu; // the controller IMU in its raw axes (only its changes are used)
    // output thread only
    bool eng_have = false;
    Vec3 eng_pos{};
    Quat eng_aim = cam::slam::QUAT_ID;
    Quat eng_imu = cam::slam::QUAT_ID; // the 3DoF at the last engine pose
    Quat eng_dfi = cam::slam::QUAT_ID;
    std::int64_t eng_last_ns = 0;
    int logged_state = -1;
    std::uint32_t logged_buttons = 0xffffffffu; // the button bits at the last "buttons" log line (every change is logged)
    std::int64_t diag_last_ns = 0; // throttles the controller prediction diagnostic (~30 Hz)
    // the Pimax button: a long press (0.8 s) recenters (session thread)
    std::int64_t pimax_down_ns = 0;
    bool recentered = false;
};

/// The shadow tracker's pose file: `pibridge-ctrltrack-shadow.pose` next to pibridge-ctrltrack's default one (the app
/// starts `pibridge-ctrltrack live --out` it in shadow mode).
std::string shadow_pose_path();
/// The controller fusers' settings (Config::from_env, the hold as Pimax's).
ctrl::fuse::Config fuse_config();

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
    void start_fuser_logs();
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
    /// The pose file optical_fixes reads: pibridge-ctrltrack's default one, or the shadow tracker's.
    std::string ctrl_pose_path_;
    // Shadow mode (PIBRIDGE_CTRL_SHADOW=1 with the Pimax engine and its LED engine): a second pibridge-ctrltrack runs
    // on the same LED frames and writes shadow_pose_path(); our fusers run on its fixes and the controller IMU exactly
    // as with Basalt (they also write the hint file its IMU-led tracker reads), but only for a comparison log, never
    // for the published pose: each hand's Pimax pose and ours, side by side (shadow_log, ctrl-shadow-*.csv).
    std::unique_ptr<ctrl::fuse::Fuser> shadow_[2];
    std::FILE* shadow_csv_ = nullptr;
    std::string shadow_csv_path_;
    std::int64_t shadow_log_ns_[2] = {0, 0};
    std::uint64_t shadow_rows_ = 0;
    /// The last optical fix of the shadow tracker per hand (world, aim frame), for the log (output thread).
    struct ShadowFix {
        std::uint64_t frame_ns = 0;
        std::uint32_t leds = 0;
        std::uint32_t cams = 0;
        std::uint32_t kind = 0;
        std::array<std::uint8_t, 4> blobs{};
        float rms_px = 0;
        Vec3 p{};
        Quat q = cam::slam::QUAT_ID;
    };
    ShadowFix shadow_fix_[2];
    /// The fuser optical fixes and controller reports go to: Basalt's, else the shadow one, else none.
    ctrl::fuse::Fuser* optical_fuser(std::size_t i) const { return fuser_[i] ? fuser_[i].get() : shadow_[i].get(); }
    /// Shadow mode: our fuser's pose at now next to Pimax's (`dll`, before the controller angle), one row per hand
    /// every 20 ms (output thread).
    void shadow_log(int side, std::int64_t now_ns, const hmd::pose_shm::Device& dll, int dll_state, std::int64_t eng_ns,
                    const std::optional<hmd::ControllerReport>& rep, std::int64_t rep_ns, std::uint32_t reports);
    std::uint32_t shadow_reports_[2] = {0, 0};
    std::array<std::uint8_t, 4> frame_blobs_{}; // the shadow tracker's newest LED frame: blobs per camera
    /// Our controllers' position is reported at the controller IMU (the LED models' ImuToDevice translation, device
    /// frame), the point Pimax's engine reports, not the LED model origin (PIBRIDGE_CTRL_IMU_POINT=0: the origin).
    std::array<Vec3, 2> imu_in_device_{};
    Vec3 imu_point(int side, const Quat& q_aim, const Vec3& p) const;
    std::uint64_t frame_blobs_ns_ = 0;
    std::int64_t shadow_stats_ns_ = 0;
    // head shadow mode (Options::shadow_vit): the second engine, the indices of its cameras in o_.cams, the log
    std::unique_ptr<VitHost> hshadow_;
    std::vector<std::size_t> hshadow_tiles_;
    std::FILE* hshadow_csv_ = nullptr;
    std::string hshadow_csv_path_;
    std::int64_t hshadow_last_[2] = {0, 0};
    std::int64_t hshadow_pose_ns_ = 0;    // the last Basalt pose (monotonic now)
    std::int64_t hshadow_reset_ns_ = 0;   // the last reset
    float hshadow_prev_p_[3] = {0, 0, 0};
    bool hshadow_have_prev_ = false;
    std::uint64_t hshadow_rows_ = 0, hshadow_resets_ = 0;
    std::atomic<std::int64_t> hshadow_frame_ns_{0};
    /// One engine pose to the head shadow log (src 0 the driving engine, 1 the shadow), at most every 5 ms per source.
    void hshadow_log(int src, const EnginePose& e);
    SysMon sysmon_;
    /// Poll the Pimax engine's room state (floor + localization) at a low rate, write room-state.json and log changes
    /// (output thread). This is the observe-and-pin-down step for GetGroundState / GetLocModeState.
    void room_state_poll(std::int64_t now_ns);
    std::int64_t room_poll_at_ = 0;
    /// The recenter: the live anchor and the chaperone commit (the Pimax button's long press, the dashboards' request).
    void recenter_now(const std::string& why);
    /// $XDG_RUNTIME_DIR/pibridge/recenter-request changed: recenter (output thread, ~3 Hz).
    void recenter_request_poll(std::int64_t now_ns);
    std::int64_t recenter_poll_at_ = 0;
    std::optional<std::int64_t> recenter_stamp_;
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
    bool ctrl_dev_ts_ = true; // PIBRIDGE_CTRL_IMU_TS=host: the controller IMU stamped at arrival, as before
    bool ctrl_imu_both_ = true; // PIBRIDGE_CTRL_IMU_BOTH=0: only the first IMU sample of a report to the LED engine
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
    // the right controller's Menu button is held: also sent as the LEFT controller's system click, since SteamVR's
    // Touch profile has /input/system on the left hand only (vrcompositor_bindings_touch.json binds the dashboard to
    // the left one, the right one to nothing), so the right Menu button did nothing in SteamVR
    std::atomic<bool> right_menu_{false};
    /// The head poses published lately (output thread only): the controllers go under the head at their sample time.
    struct HeadAt {
        std::int64_t ts = 0;
        Vec3 pos{};
        Quat q = cam::slam::QUAT_ID;
        /// Before the world transform (room offset, recenter, drift corrections): the frame our controller tracking
        /// lives in, so those never disturb it (Pimax's engine also tracks the controllers in its own world).
        Vec3 raw_pos{};
        Quat raw_q = cam::slam::QUAT_ID;
    };
    /// The world transform of the newest head (world = raw_to_world_ * raw), for our controllers' output.
    RigidPose raw_to_world_{};
    /// Our tracking frame -> the world: a pose of ours (raw frame) as the apps see it.
    ctrl::fuse::P to_world_pose(const ctrl::fuse::P& raw) const;
    void push_head(const hmd::pose_shm::Device& raw, const hmd::pose_shm::Device& world);
    std::vector<HeadAt> head_hist_;
    std::size_t head_next_ = 0;
    std::optional<HeadAt> head_at(std::int64_t ts) const;
    /// The head pose the fusers take: the IMU's, not the head origin's. pibridge-ctrltrack's poses are relative to the
    /// calibration's body (IMU) frame, as the engine's are before `head_in_imu_` is taken off (controller()).
    ctrl::fuse::P fuse_head(const HeadAt& h) const;
    std::uint64_t ctrl_sample_[2] = {0, 0};
    std::uint64_t haptic_requests_ = 0; // vibration requests taken (output thread)
    std::int64_t haptic_logged_ns_ = 0;
    hmd::haptics::Receiver haptic_rx_; // CustomHeadsetOpenVR's pulses
};

} // namespace pibridge::headtrack
