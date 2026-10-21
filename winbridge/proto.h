// pibridge winbridge: binary protocol Linux client <-> Wine server (TCP localhost).
// Both sides are x86-64 little-endian, so we send structs as-is.
#pragma once
#include <stdint.h>

#define WINBRIDGE_PORT 48291

enum {
    MSG_IMU  = 1,  // client->server: head IMU -> MSG_POSE in reply
    MSG_CAM  = 2,  // client->server: camera frames -> MSG_ACK in reply
    MSG_POSE = 3,  // server->client: head pose
    MSG_ACK  = 4,  // server->client: acknowledgement
    MSG_LED  = 5,  // client->server: controller LED frames (short exposure) -> MSG_ACK
    MSG_CIMU = 6,  // client->server: controller IMU -> MSG_CPOSE
    MSG_CPOSE = 7, // server->client: controller pose
};

#pragma pack(push, 1)
struct MsgHeader { uint32_t type; uint32_t len; };            // len = payload bytes
struct ImuPayload { uint64_t ts; float accel[3]; float gyro[3]; };
// SLAM frames: shutter_ns = exposure (low 32 bits) | gain << 32, like field @8 of a Pimax frame
struct CamHeader { uint64_t ts; uint64_t shutter_ns; uint32_t w, h, ncam; }; // + ncam*w*h bytes Y8
// Poses in the head SLAM engine's world (z up): quat4 (x,y,z,w), pos3, angVel3, linVel3
struct PosePayload { uint64_t ts; float pose[13]; };
struct LedHeader { uint64_t ts; uint32_t shutter_ns, w, h, ncam; };            // + ncam*w*h bytes Y8
// Controller IMU in its raw axes: accel m/s^2, gyro rad/s; hand 0 left, 1 right
struct CtrlImuPayload { uint32_t hand, pad; uint64_t ts; float accel[3]; float gyro[3]; };
// Controller body pose (LED-model frame of controller_params.json) in the head SLAM engine's world;
// tracked = engine tracks the controller optically (otherwise no pose: all zeros). head = headset IMU pose from which
// the LED engine computed the controller (quat4, pos3, same world); dev_from_imu = rotation of the controller IMU into the body
// (ImuToDevice, quat x,y,z,w; constant).
struct CtrlPosePayload { uint32_t hand, tracked; uint64_t ts; float pose[13]; float head[7]; float dev_from_imu[4]; };
#pragma pack(pop)
