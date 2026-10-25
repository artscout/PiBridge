// SPDX-License-Identifier: MIT
// The SLAM engine of pibridge-headtrack: any VIT 2.0 library (Monado's visual-inertial tracker interface), loaded
// with dlopen as Monado's t_vit_loader does (all 21 symbols, major version 2): libpibridge_vit_winbridge.so (Pimax
// Play's engine under Wine) or Basalt. From winbridge/vit_replay.cpp.
//
// The IMU gate is the Monado driver's (pimax_camera.c slam_imu_gate_locked): nothing before the first frame set, then
// at most IMU_FULL samples per frame set at full rate, every IMU_DECIM-th up to IMU_SPARSE more, then nothing until the
// next frame set. Basalt's IMU queue (300) blocks its pusher when a camera gap fills it; the gate keeps the HID thread
// from ever blocking on it.
#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace pibridge::headtrack {

/// A controller pose of Pimax's LED engine (winbridge's extension, pibridge_winbridge_ext.h): the controller body (LED
/// model frame) and the head IMU pose it was placed with, both in the engine's world.
struct EngineControllerPose {
    std::int64_t ts_ns = 0;
    float q[4] = {0, 0, 0, 1};
    float p[3] = {};
    float ang_vel[3] = {};
    float lin_vel[3] = {};
    float head_q[4] = {0, 0, 0, 1};
    float head_p[3] = {};
    float dev_from_imu[4] = {0, 0, 0, 1}; // ImuToDevice (constant)
};

/// The room state from Pimax's head SLAM engine (winbridge's extension): the floor (GetGroundState) and the
/// localization/recognition state (GetLocModeState). ret: -1 engine not initialised, 0 not ready, 1 valid, -2 the DLL
/// does not export the getter. `ground` is GetGroundState's raw out buffer (layout still being pinned down),
/// `loc_state` is GetLocModeState's single out byte.
struct RoomState {
    std::int32_t ground_ret = 0;
    std::int32_t loc_ret = 0;
    std::uint8_t ground[32] = {};
    std::uint8_t loc_state = 0;
};

/// One engine pose: the IMU in the engine's world (z up), as VIT reports it.
struct EnginePose {
    std::int64_t ts_ns = 0;
    float p[3] = {};
    float q[4] = {0, 0, 0, 1}; // x y z w
    float v[3] = {};
};

class VitHost {
public:
    inline static constexpr std::uint32_t IMU_FULL = 200;
    inline static constexpr std::uint32_t IMU_DECIM = 10;
    inline static constexpr std::uint32_t IMU_SPARSE = 60;

    /// Load `library` and start a tracker for `cameras` cameras (`config`: a SLAM_CONFIG file, empty for none);
    /// nullptr with `error` set when it cannot.
    static std::unique_ptr<VitHost> open(const std::string& library, std::uint32_t cameras, const std::string& config,
                                         std::string* error);
    ~VitHost();
    VitHost(const VitHost&) = delete;
    VitHost& operator=(const VitHost&) = delete;

    /// One IMU sample in the driver's body axes (OpenXR head axes), m/s^2 and rad/s; false when the gate held it back.
    bool push_imu(std::int64_t ts_ns, const float accel[3], const float gyro[3]);
    /// One frame set: `tiles[i]` the 640x480 Y8 image of camera i (i < cameras), one timestamp for all.
    void push_frames(std::int64_t ts_ns, const std::vector<const std::uint8_t*>& tiles, std::uint32_t w, std::uint32_t h);
    /// Every pose the engine produced since the last call (oldest first).
    std::vector<EnginePose> poses();
    /// Start the tracker over (Basalt, when the supervisor asks: RESET_WANTED).
    void reset();
    std::uint32_t cameras() const { return cameras_; }
    /// The library has winbridge's controller extension (Pimax's LED engine).
    bool has_controllers() const { return ext_ok_; }
    /// The four LED-exposure tiles (640x480 Y8), `shutter_ns` the exposure.
    void push_led(std::int64_t ts_ns, std::uint32_t shutter_ns, const std::vector<const std::uint8_t*>& tiles);
    /// One controller IMU sample in its raw axes (m/s^2, rad/s); hand 0 left, 1 right.
    void push_controller_imu(std::uint32_t hand, std::int64_t ts_ns, const float accel[3], const float gyro[3]);
    /// The newest controller pose of `hand`, if the engine has one.
    std::optional<EngineControllerPose> controller_pose(std::uint32_t hand);
    /// The last room state from the engine (floor + localization); nullopt until the first reply. Each call also
    /// queues a fresh request, so poll it at a low rate (the floor and the recognition change slowly).
    std::optional<RoomState> room_state();
    std::uint64_t imu_held() const { return imu_held_; }

private:
    VitHost() = default;
    struct Api;
    void* lib_ = nullptr;
    std::unique_ptr<Api> api_;
    void* tracker_ = nullptr;
    std::uint32_t cameras_ = 0;
    std::mutex gate_mu_;
    bool frame_seen_ = false;
    std::uint32_t since_frame_ = 0;
    std::uint64_t imu_held_ = 0;
    bool ext_ok_ = false;
    void* ext_led_ = nullptr;
    void* ext_imu_ = nullptr;
    void* ext_pose_ = nullptr;
    void* ext_room_ = nullptr;
};

} // namespace pibridge::headtrack
