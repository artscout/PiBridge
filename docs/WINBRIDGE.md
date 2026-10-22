# winbridge: Pimax's own head tracking engine under Wine

`io_slam_engine=pimax` (Tracking page, **Head tracking engine: Pimax**) makes Monado track the head with Pimax Play's
own SLAM engine (`pimax_slam.pi.dll`) and the headset's factory calibration, instead of Basalt. The engine runs under
Wine in its own process; Monado reaches it through a VIT library. Nothing in the Monado Pimax driver changes: the
driver feeds its SLAM tracker as always, only the VIT library behind it is another one.

```
Monado Pimax driver --VIT--> libpibridge_vit_winbridge.so --TCP 127.0.0.1:48291--> winbridge.exe (Wine) --> pimax_slam.pi.dll
   cameras 0,1 + IMU                (pairs the two frames,          (proto.h, lockstep)            Headset* C API
   <-- head pose (T_world_imu)       reconnects every 1 s)
```

## Files

| Path | What |
|---|---|
| `winbridge/server.cpp` -> `winbridge.exe` | The engine host: loads the DLL, `HeadsetInitialImpl` -> `HeadsetSetTrackingMode(1)` -> `HeadsetSetDeviceEnable(1)`, then serves one client. MinGW: `x86_64-w64-mingw32-g++ -O2 -static server.cpp -lws2_32 -o winbridge.exe` |
| `winbridge/vit_winbridge.cpp` -> `libpibridge_vit_winbridge.so` | A VIT 2.0 tracker (all 21 functions Monado's `t_vit_loader` requires). `g++ -O2 -std=c++20 -fPIC -shared vit_winbridge.cpp -o libpibridge_vit_winbridge.so -pthread` |
| `winbridge/proto.h` | The wire format: `MSG_IMU` -> `MSG_POSE`, `MSG_CAM` -> `MSG_ACK` (x86-64 little-endian structs) |
| `winbridge/vit/vit_interface.h` | Copy of Monado's VIT header (BSL-1.0, Collabora); needs `#define VIT_INTERFACE_IMPLEMENTATION` |
| `winbridge/client.cpp`, `winbridge/vit_replay.cpp` | Offline checks: a recording (`replay.bin`) through the raw protocol / through the VIT library loaded like Monado loads it |
| `loader/loader.cpp` | The exploration tool (`smoke`, `init`, `feed`, `play`) the above came from |

The kit (never in this repository, personal use only): `$XDG_DATA_HOME/pibridge/windata/`
- `dlls/`: the DLLs of Pimax Play's `Runtime` folder that `pi_server` loads (the engine needs OpenCV 4.5.3, Ceres and
  the MSVC runtime next to it).
- `slam/`: `voc_GEN_8X4.dbow`, `pimax_database.bin` (Pimax Play's `Runtime\slam`) and the headset's
  `device_calibration_<serial>.xml` (Pimax Play's `ProgramData\Pimax\slam`). The engine writes its logs
  (`6DOF_*.txt`, `controller_*.txt`), its map and `pimax_prior_position.txt` here.

### Copying the Pimax files (once, and again after a Pimax Play update)

The kit comes from the Pimax Play install of the same PC's Windows. Mount the Windows drive (below `/mnt/c`; any mount
point works) and copy:

| From the Windows drive | To |
|---|---|
| `Program Files/Pimax/Runtime/*.dll` | `~/.local/share/pibridge/windata/dlls/` |
| `Program Files/Pimax/Runtime/slam/voc_GEN_8X4.dbow`, `pimax_database.bin` | `~/.local/share/pibridge/windata/slam/` |
| `ProgramData/Pimax/slam/device_calibration_<serial>.xml` (this headset's calibration) | `~/.local/share/pibridge/windata/slam/` |
| `ProgramData/Pimax/slam/H1/` (the whole folder: one `<controller serial>/controller_params.json` per controller) | `~/.local/share/pibridge/windata/slam/H1/` |

```
W=/mnt/c; K=~/.local/share/pibridge/windata
mkdir -p "$K/dlls" "$K/slam"
cp "$W/Program Files/Pimax/Runtime/"*.dll "$K/dlls/"
cp "$W/Program Files/Pimax/Runtime/slam/voc_GEN_8X4.dbow" "$W/Program Files/Pimax/Runtime/slam/pimax_database.bin" "$K/slam/"
cp "$W/ProgramData/Pimax/slam/"device_calibration_*.xml "$K/slam/"
cp -r "$W/ProgramData/Pimax/slam/H1" "$K/slam/"
```

Without `H1/` the head still uses Pimax's engine and the controllers stay with pimax-ctrltrack. The Tracking page
says what is missing. These files are Pimax's: personal use, they stay on this computer and never go into a
repository (`.gitignore` keeps `windll/` and `*.exe` out).

The app looks for `winbridge.exe` and the library in `$XDG_DATA_HOME/pibridge/winbridge/`, then in the source tree
(`rec_winbridge=`, `rec_vit_winbridge=` override), for `wine` in `PATH`, and accepts the kit when the DLL, the
vocabulary and one calibration are there (`pimaxSlamReady`, `core/inside_out.*`).

## Like Pimax Play, not like Basalt

The reference is Pimax Play on Windows, so with this engine PiBridge sets what Pimax Play's service does:

| Driver setting | Why |
|---|---|
| `PIMAX_SLAM_CAMS=0,1,2,3` | Pimax Play hands the engine all four tracking cameras in every frame set. A camera the source does not have gets a black frame in the server (a copy of another camera would be the wrong geometry). |
| `PIMAX_SLAM_EXPOSURE_MID=0` | Raw frame stamps (exposure start). The engine adds its own factory camera-IMU `delta` (about half a typical exposure); the driver's mid-exposure shift is for Basalt and would count twice. |
| `PIMAX_SLAM_PASSTHROUGH=1` | The driver uses the engine's pose as it comes: the newest pose moved on to display time with the engine's linear velocity and the gyro (what Pimax's `PosePredict` does: a constant-velocity step of the same 13-float state), then the head origin and the recenter. No supervisor, still gate, tilt correction, fusion weights or drift hold (Basalt's workarounds; the engine handles loss and relocalisation). With no pose for 350 ms: 3DoF at the last position, orientation continuous. `pimax_camera.c` in the PiBridge Monado tree (`third_party/monado`, marker `PIMAX_SLAM_PASSTHROUGH`). |
| `PIBRIDGE_VIT_POSE_RATE=imu` | Every engine pose (about 1 kHz), as Pimax Play reads one per IMU sample. (`cam` = one per frame set, for the Basalt-style supervisor.) |

## Controllers: Pimax's LED engine (`LedObjectPoseEstimator.dll`)

With the Pimax head engine and optical controller tracking on (`pimaxCtrlEngine`: also needs the controllers' LED
models in the kit), the controllers come from Pimax's LED engine in the same winbridge, as in Pimax Play, and
pimax-ctrltrack is not started. Everything below is from the Frida traces of Pimax Play's service (pass 1-3).

- Init: `ControllerInitialImpl(calib, slamDir, pimax_database.bin, NULL, NULL)`, `ControllerSetDeviceEnable(1)`,
  then per controller `ControllerAddObject(hand 0/1, 15, leds, imuToDevice, 2, 0, 0x34, std::string "\H1\")`:
  `leds` = 15 x `{i32 id; pos[3]; normal[3]}` and `imuToDevice` = `ImuToDevice` as quaternion (x, y, z, w) +
  translation, both from `slam/H1/<controller serial>/controller_params.json` (Pimax Play's
  `ProgramData\Pimax\slam\H1\`). The last argument is an MSVC `std::string` (short-string layout, built in place).
- Frames: the cameras alternate the tracking exposure with a short controller-LED exposure (25 + 25 Hz). The LED
  frames go to `ControllerPutCameraImage(led0..3, headPose, NULL)`: frame `{u64 ts; u32 shutter_ns = 40000; u32 idx;
  w; h; stride; pad; px}`, `headPose` = the head pose at that time. The last argument is a C++ object in Pimax Play;
  NULL works.
- IMU: every controller report (500 Hz per hand) to `ControllerPutControllerImuData(hand, {u64 ts; accel[3];
  gyro[3]}, out, scratch)` in the controller's raw IMU axes (m/s^2, rad/s; the driver's scales 1024 LSB/g and
  16.4 LSB/(deg/s)); the pose comes back in `out` (quaternion, position, angular and linear velocity).
- Axes: the LED engine works in (x, z, -y) of the SLAM world (world rotation Rx(-90 deg), body unchanged): derived
  from one head pose that pass 1 and pass 3 caught in both forms. winbridge turns the head pose into it and the
  controller poses back.
- Path: Monado's Pimax driver (`PIMAX_CTRL_ENGINE=1`) hands the LED frames (camera thread) and every controller IMU
  report to the extension of `libpibridge_vit_winbridge.so` (`pibridge_winbridge_ext.h`, found by dlopen of
  `VIT_SYSTEM_LIBRARY_PATH`), which sends them over the same pipelined socket. The reply carries the controller pose
  and the head pose the engine used; the driver takes the controller relative to the head and puts it under its own
  head pose at the sample time (so recenter and the room anchor apply as to the head), moves it on to display time
  (engine velocity, controller gyro), turns the body into the aim frame (rot_x(-52 deg), as pimax-ctrltrack) and the
  grip (Touch geometry). Not seen by the engine: position held and orientation turned by the gyro for
  `PIMAX_CTRL_TRACK_LOST_S` (4 s), then the rest pose.
- Offline (`ctrl_prep` + `vit-replay --ctrl` on a 30 s take with LED frames and both controllers' IMU, no head data):
  the engine tracks both hands, poses for about 80 % (left) and 76 % (right) of the IMU samples, nothing dropped;
  head-to-controller distance within 5 cm of the take's own tracker (a constant origin offset).

## The session

`RecSession` (`core/rec_session.*`), both START TRACKING (SteamVR + Monado) and Record tracking / Monado directly:
once the camera broker's socket is up, `wine winbridge.exe` starts (`WINEDEBUG=-all`, `PIBRIDGE_WINDATA=Z:\...` = the
kit) before the controller tracker and SteamVR / Monado; the driver gets
`VIT_SYSTEM_LIBRARY_PATH=<libpibridge_vit_winbridge.so>`. Stop ends the processes by PID in reverse order, so the engine
ends after SteamVR / Monado (wine execs into `winbridge.exe`: the started PID is the server's). If the engine ends on
its own, the session goes on; the VIT library loses it and the head is 3DoF until the next start. Another running
`winbridge.exe` blocks the start (it holds the port).

## What the engine needs (found 2026-10-20)

- `HeadsetInitialImpl(calibXml, slamDir, vocab, NULL, NULL)` (from a Frida trace of Pimax Play's service).
- `HeadsetSetTrackingMode(byte)` and `HeadsetSetDeviceEnable(byte)` take one byte each (disassembly: the wrapper
  stores `cl` and returns -1 before init). Mode **1** tracks a new session; mode 0 ("kNormal, load map to reloc")
  only relocalises against the saved map, so an unrelated place never gets a position.
- `HeadsetPutCameraImage(cam0, cam1, cam2, cam3, float)`: 40-byte frames `{u64 ts; u64 shutter_ns; u32 idx, w, h,
  stride; u8* px}`. **`shutter_ns` 0 drops every frame** (log: "image shutter_speed_ns is 0, too short!"). VIT has no
  exposure field: `PIBRIDGE_SHUTTER_NS`, default 10 ms. Cameras 2 and 3 repeat 0 and 1 (the head engine tracks with 0, 1).
- `HeadsetPutHmdImuData(imu, out)`: `{u64 ts; float accel[3]; float gyro[3]}` in, the pose out: quaternion
  (x, y, z, w), position, angular velocity, linear velocity. Before the engine has a position it fills only the
  orientation; the VIT library passes poses on only once a position is there.
- Frames at the recording's pace: the engine works on its own thread.

## Checked

- Offline on a 16 s recording (400 frame pairs, 16 450 IMU samples): 16 441 poses with a position, the same through
  the DLL directly, the raw protocol and the VIT library; repeated runs identical.
- Gravity rotated into the world by the poses: (0.10, -0.03, 10.09) m/s^2, so the engine's world is Z-up like
  Monado expects and no axis change is needed.
- App tests: `pimax_slam_engine_environment`, `session_pimax_engine_runs_winbridge_before_monado_and_stops_it_after`
  (`app/tests/test_record.cpp`).

- `PIBRIDGE_SHUTTER_NS` 0.1 / 10 / 20 ms give the same result: the engine only needs a non-zero value, it does not time
  the frame with it.
- On the two-camera recording the engine's rotation between frames follows the gyro except in fast turns
  (> 100 deg/s: p95 1.2 deg, max 4.5 deg). Black frames for the missing side cameras instead of copies: a little
  better. The four-camera feed (as Pimax Play) is the test that is still open.

## Not done yet

- Monado with `PIMAX_SLAM_PASSTHROUGH` is only compile-checked: not built into the live `pibridge-monado-build`, not run
  with the live headset (needs a session-free moment; the relink drops `cap_sys_nice`, PiBridge's Fix restores it).
- A recording with all four cameras (`PIMAX_SLAM_CAMS=0,1,2,3`) for the offline check, and a Windows take of the same
  motions in Pimax Play to compare with.
- The real exposure is not passed (not needed for timing, see above).
- The controllers through the engine are not run with the live headset yet (Monado built, the driver path only
  compile-checked and tested offline up to the extension).

Settled by Windows pass 5 (`re-dll/README.md`, frida-pass5.json + the USB capture, Beat Saber Extreme):
- Controller IMU: matching the raw USB HID reports with the engine's inputs gave the gyro scale **8.2 LSB/(deg/s)**
  (not the old 16.4 guess; 2x), accel 1024 LSB/g, axes identical. Fixed in the driver (`PIMAX_CTRL_GYRO_LSB_DPS`).
- The pose predictor need not be called: its lead time is a ns timestamp in the `PosePredict` record (head ~16 ms,
  app/controllers clamp at 25 ms). Monado's own predictor stays; the driver caps its horizon at 25 ms like Pimax.
- The floor and play area do **not** come from the engine (`HeadsetGetGroundState` is only SLAM's own ground point,
  unchanged by a room setup). The user's floor and centre live in `chaperone_info.vrchap`; PiBridge's own room setup
  (roomsetup / roommap) owns the zone, not winbridge.