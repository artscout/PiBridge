# pibridge-headtrack

Inside-out tracking of the Crystal Super and the Dream Air: one process tracks the head and the
controllers and writes them to the pose file CustomHeadsetOpenVR's SLAM headset reads, the same way the original
Crystal's own inside-out reaches SteamVR (`pibridge-keepalived`, [keepalived.md](keepalived.md)). SteamVR runs as in
Lighthouse mode: CustomHeadsetOpenVR draws the picture. Sources in `tools/headtrack/`; it is the app's inside-out
session (START VR on the Dashboard; the settings are on the Tracking page, [TRACKING.md](TRACKING.md)).

## What it does

- **The headset**: owns the HID while it runs (`libpibridge-hmd` `Session`: wake, keepalive, video, IMU). It writes the
  inside-out pause marker and waits for `pibridge-keepalived` to pause; the volume buttons are relayed to it.
- **The IMU**: device time (u32 us) unwrapped and mapped to `CLOCK_MONOTONIC` with the least delayed offset; OpenXR
  axes (-x, y, -z), m/s² and rad/s. A 3DoF orientation (Mahony, gyro bias learnt at rest) covers the gaps.
- **The cameras**: frames from the camera broker (`pibridge-cam`) on the IMU timeline. The tracking exposure goes to
  the SLAM engine (a VIT 2.0 library, `vit_host.{h,cpp}`, with the IMU gate 200 / every 10th up to 60 more); the LED
  exposure goes to Pimax's LED engine for the controllers.
- **Engines** (`--engine`):
  - `pimax` (default): Pimax Play's engine under Wine (`libpibridge_vit_winbridge.so`, [WINBRIDGE.md](WINBRIDGE.md)).
    The head pose is taken as it is (zeroed at the start); without a pose for 350 ms the 3DoF orientation at the last
    position. Controllers from the LED engine through winbridge's extension, held by their own gyro for up to 4 s.
  - `basalt`: Basalt with libpibridge-cam's output layer (`headpose::HeadTracker`, mid-exposure frame times, reset when
    the supervisor asks); controllers from `pibridge-ctrltrack`'s pose file fused with their IMU (`libpibridge-ctrl`).
- **Auto-exposure** (`PIMAX_CAM_AE`: unset/0 off, `dry` compute only, `1` send): the same controller as before
  (`libpibridge-cam` `ae.h`), its exposure push sent as HID 0x51 on the headset session; `PIMAX_CAM_AE_MODEL` and
  `PIMAX_CAM_AE_LAW` as on the Tracking page.
- **The room**: the head ring `$XDG_RUNTIME_DIR/pimax-anchor.head` and the offset file `pimax-anchor.offset`
  (`world.{h,cpp}`, the formats of `libpibridge-anchor`), so the room map and RECENTER IN HEADSET work as before.
  A long press (0.8 s) of the Pimax button recenters; the Pimax button is not passed to apps, the menu button is
  SteamVR's system button.
- **Haptics**: CustomHeadsetOpenVR writes `$XDG_RUNTIME_DIR/pimax-haptic-request` ("SLOT AMP MS"), headtrack sends it.
- **Output**: `/dev/shm/pibridge-crystal-pose` (`libpibridge-hmd` `pose_shm.h`, source 2 = headtrack) every 1 ms with
  the sample time and velocities; SteamVR predicts to the display time.

## CustomHeadsetOpenVR

Needs `patches/chovr/chovr-linux-headtrack.patch` on top of the other four ([INSTALL.md](INSTALL.md)): the SLAM
headset stands for the Crystal Super or the Dream Air when the pose file's source is 2, and the controllers' vibration
goes to the request file.

A Monado SteamVR driver (`driver_monado`) left over from an older setup must not be registered with SteamVR
(`external_drivers` in `~/.config/openvr/openvrpaths.vrpath`): it would also claim the headset.

## Realtime priority

Not needed: neither headtrack nor the camera broker asks for it, and the picture is SteamVR's compositor's
(`vrcompositor-launcher` has its own `cap_sys_nice` from SteamVR).

## The old session

`io_headtrack` (default on) is not in the UI; `off` selects the older session through Monado's SteamVR driver, whose
code is still in the app but which PiBridge no longer builds, ships or supports.
