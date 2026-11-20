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
    Their IMU samples carry the report's device timestamp on the head IMU's clock, as in Pimax Play
    (`PIBRIDGE_CTRL_IMU_TS=host`: the arrival time, as before; [WINBRIDGE.md](WINBRIDGE.md)). pibridge-ctrltrack is
    not used with this engine, except in the controller shadow mode (below).
  - `basalt`: Basalt with libpibridge-cam's output layer (`headpose::HeadTracker`, mid-exposure frame times, reset when
    the supervisor asks); controllers from `pibridge-ctrltrack`'s pose file fused with their IMU (`libpibridge-ctrl`).
- **Auto-exposure** (`PIMAX_CAM_AE`: unset/0 off, `dry` compute only, `1` send): the same controller as before
  (`libpibridge-cam` `ae.h`), its exposure push sent as HID 0x51 on the headset session; `PIMAX_CAM_AE_MODEL` and
  `PIMAX_CAM_AE_LAW` as on the Tracking page.
- **The room**: the head ring `$XDG_RUNTIME_DIR/pimax-anchor.head` and the offset file `pimax-anchor.offset`
  (`world.{h,cpp}`, the formats of `libpibridge-anchor`), so the room map and RECENTER IN HEADSET work as before.
  A long press (0.8 s) of the Pimax button recenters; the Pimax button is not passed to apps, the menu button is
  SteamVR's system button.
  The dashboards' **Recenter** (the SteamVR Desktop tab's bar) does the same: a change of
  `$XDG_RUNTIME_DIR/pibridge/recenter-request` is taken as a press (checked ~3 times a second).
- **Haptics**: CustomHeadsetOpenVR writes `$XDG_RUNTIME_DIR/pimax-haptic-request` ("SLOT AMP MS"), headtrack sends it.
- **Output**: `/dev/shm/pibridge-crystal-pose` (`libpibridge-hmd` `pose_shm.h`, source 2 = headtrack) every 1 ms with
  the sample time and velocities; SteamVR predicts to the display time.

## Controller shadow mode

Compares our controller tracking with Pimax's LED engine on the same data and in the same session. Pimax's engine
still drives the controllers in SteamVR, and the shadow mode changes nothing in what SteamVR gets.

Turn it on with `PIBRIDGE_CTRL_SHADOW=1` in the app's extra environment (Tracking page, Advanced; `rec_env=`). It works
with the Pimax engine and its LED engine (`pimaxCtrlEngine`). Then:

- the app also starts `pibridge-ctrltrack live ... --out $XDG_RUNTIME_DIR/pibridge-ctrltrack-shadow.pose` (process
  `ctrltrack-shadow`, log `ctrltrack-shadow.log` with the session's logs; if it ends, the session goes on);
- pibridge-headtrack runs our fusers (`libpibridge-ctrl`) on that tracker's fixes and on the controller IMU, exactly as
  with Basalt. They also write the hint file, so the tracker runs its IMU-led filter. Their pose is only logged.

The log is `~/.local/share/pibridge/ctrl-shadow/ctrl-shadow-YYYYmmdd-HHMMSS.csv`: one row per hand every 20 ms, all
poses in the world and the aim frame (before the controller angle). The columns:

| Columns | Meaning |
|---|---|
| `t_ns`, `hand` | `CLOCK_MONOTONIC` time of the row, 0 left / 1 right |
| `pimax_state` | 0 no pose, 1 Pimax's LED engine, 2 held by the gyro |
| `pimax_t_ns`, `pimax_x..z`, `pimax_qx..qw` | Pimax's pose as published (extrapolated up to 25 ms, to `pimax_t_ns`) |
| `ours_state`, `ours_x..z`, `ours_qx..qw` | our fuser at `t_ns`: 0 none, 1 optical, 2 held |
| `fix_ns`, `fix_leds`, `fix_rms_px`, `fix_x..z`, `fix_qx..qw` | the shadow tracker's last optical fix (frame time, matched LEDs, RMS) |
| `head_x..z`, `head_qx..qw` | the head pose at `t_ns` (the head origin, as published) |
| `pimax_eng_ns`, `pimax_vx..vz`, `pimax_wx..wz` | the time of Pimax's engine pose before the prediction, and the published velocities |
| `imu_age_ms`, `imu_reports` | age of the controller's last report, reports since the previous row |
| `acc_x..z`, `gyro_x..z` | the last report's accelerometer (m/s²) and gyro (rad/s), the controller's raw axes |
| `fix_cams`, `fix_kind`, `blobs0..3` | cameras in the fix; bit 0 few-LED path, bit 1 the IMU filter's pose; blobs per camera in that frame |

Beside it, `ctrl-shadow-…-sys.csv` (`sysmon.{h,cpp}`), once a second:
- the CPU time, run-queue wait and timeslices of every busy thread of winbridge, wineserver, pibridge-ctrltrack, headtrack,
  the broker, vrserver and vrcompositor (`/proc/PID/task/TID/schedstat`);
- the load, CPU pressure, CPU/GPU temperatures, CPU clocks and GPU busy.

pibridge-ctrltrack's poses are relative to the calibration's body (IMU) frame; the fusers take the head pose of the
IMU (`fuse_head`: the published head minus `head_in_imu`), as the Pimax path does. Before this, our controllers sat
`head_in_imu` (~6 cm) off in Basalt and shadow mode.

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
