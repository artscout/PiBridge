// pibridge winbridge: бинарный протокол Linux-клиент <-> Wine-сервер (TCP localhost).
// Обе стороны x86-64 little-endian, поэтому структуры шлём как есть.
#pragma once
#include <stdint.h>

#define WINBRIDGE_PORT 48291

enum {
    MSG_IMU  = 1,  // клиент->сервер: IMU головы -> в ответ MSG_POSE
    MSG_CAM  = 2,  // клиент->сервер: кадры камер -> в ответ MSG_ACK
    MSG_POSE = 3,  // сервер->клиент: поза головы
    MSG_ACK  = 4,  // сервер->клиент: подтверждение
    MSG_LED  = 5,  // клиент->сервер: LED-кадры контроллеров (короткая выдержка) -> MSG_ACK
    MSG_CIMU = 6,  // клиент->сервер: IMU контроллера -> MSG_CPOSE
    MSG_CPOSE = 7, // сервер->клиент: поза контроллера
};

#pragma pack(push, 1)
struct MsgHeader { uint32_t type; uint32_t len; };            // len = байт полезной нагрузки
struct ImuPayload { uint64_t ts; float accel[3]; float gyro[3]; };
// SLAM-кадры: shutter_ns = выдержка (младшие 32 бита) | усиление << 32, как поле @8 кадра Pimax
struct CamHeader { uint64_t ts; uint64_t shutter_ns; uint32_t w, h, ncam; }; // + ncam*w*h байт Y8
// Позы в мире SLAM движка головы (z вверх): quat4 (x,y,z,w), pos3, angVel3, linVel3
struct PosePayload { uint64_t ts; float pose[13]; };
struct LedHeader { uint64_t ts; uint32_t shutter_ns, w, h, ncam; };            // + ncam*w*h байт Y8
// IMU контроллера в его сырых осях: accel м/с^2, gyro рад/с; hand 0 левый, 1 правый
struct CtrlImuPayload { uint32_t hand, pad; uint64_t ts; float accel[3]; float gyro[3]; };
// Поза корпуса контроллера (система LED-модели controller_params.json) в мире SLAM движка головы;
// tracked = движок ведёт контроллер оптикой (иначе поза не выдаётся: все нули). head = поза IMU шлема, с которой
// LED-движок посчитал контроллер (quat4, pos3, тот же мир); dev_from_imu = поворот IMU контроллера в корпус
// (ImuToDevice, quat x,y,z,w; постоянный).
struct CtrlPosePayload { uint32_t hand, tracked; uint64_t ts; float pose[13]; float head[7]; float dev_from_imu[4]; };
#pragma pack(pop)
