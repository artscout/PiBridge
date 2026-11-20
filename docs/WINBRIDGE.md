# winbridge: Pimax's own head tracking engine under Wine

`io_slam_engine=pimax` (Tracking page, **Tracking engine: Pimax**) makes `pibridge-headtrack`
([pibridge-headtrack.md](pibridge-headtrack.md)) track the head with Pimax Play's own SLAM engine (`pimax_slam.pi.dll`)
and the headset's factory calibration, instead of Basalt. The engine runs under Wine in its own process;
pibridge-headtrack reaches it through a VIT library (`--engine pimax --vit libpibridge_vit_winbridge.so`), the same
interface it loads Basalt through.

```
pibridge-headtrack --VIT--> libpibridge_vit_winbridge.so --TCP 127.0.0.1:48291--> winbridge.exe (Wine) --> pimax_slam.pi.dll
   cameras 0-3 + IMU                (pairs the two frames,          (proto.h, lockstep)            Headset* C API
   <-- head pose (T_world_imu)       reconnects every 1 s)
```

## Files

| Path | What |
|---|---|
| `winbridge/server.cpp` -> `winbridge.exe` | The engine host: loads the DLL, `HeadsetInitialImpl` -> `HeadsetSetTrackingMode(1)` -> `HeadsetSetDeviceEnable(1)`, then serves one client. MinGW: `x86_64-w64-mingw32-g++ -O2 -static server.cpp -lws2_32 -o winbridge.exe` |
| `winbridge/vit_winbridge.cpp` -> `libpibridge_vit_winbridge.so` | A VIT 2.0 tracker (all 21 functions Monado's `t_vit_loader` requires). `g++ -O2 -std=c++20 -fPIC -shared vit_winbridge.cpp -o libpibridge_vit_winbridge.so -pthread` |
| `winbridge/proto.h` | The wire format: `MSG_IMU` -> `MSG_POSE`, `MSG_CAM` -> `MSG_ACK`, `MSG_ROOMREQ` -> `MSG_ROOMSTATE` (x86-64 little-endian structs) |
| `winbridge/vit/vit_interface.h` | Copy of Monado's VIT header (BSL-1.0, Collabora); needs `#define VIT_INTERFACE_IMPLEMENTATION` |
| `winbridge/client.cpp`, `winbridge/vit_replay.cpp` | Offline checks: a recording (`replay.bin`) through the raw protocol / through the VIT library loaded like a VIT host loads it |
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

Without `H1/` the head still uses Pimax's engine, but the controllers have no optical tracking: `pibridge-ctrltrack`'s
poses are used only with the Basalt head engine (pibridge-headtrack has its fuser only there), so Pimax head tracking
with pibridge-ctrltrack controllers is not a working combination. The Tracking page says what is missing. These files are Pimax's: personal use, they stay on this computer and never go into a
repository (`.gitignore` keeps `windll/` and `*.exe` out).

The app looks for `winbridge.exe` and the library in `$XDG_DATA_HOME/pibridge/winbridge/`, then in the source tree
(`rec_winbridge=`, `rec_vit_winbridge=` override), for `wine` in `PATH`, and accepts the kit when the DLL, the
vocabulary and one calibration are there (`pimaxSlamReady`, `core/inside_out.*`).

## Like Pimax Play, not like Basalt

The reference is Pimax Play on Windows, so with this engine pibridge-headtrack does what Pimax Play's service does:

| Setting | Why |
|---|---|
| `--cams 0,1,2,3` | Pimax Play hands the engine all four tracking cameras in every frame set. A camera the source does not have gets a black frame in the server (a copy of another camera would be the wrong geometry). |
| raw frame stamps | Exposure start. The engine adds its own factory camera-IMU `delta` (about half a typical exposure); the mid-exposure shift is for Basalt and would count twice. |
| the pose as it comes | The engine's pose with its velocities, zeroed at the start, then the recenter; SteamVR predicts it to display time. No supervisor, still gate, tilt correction, fusion weights or drift hold (Basalt's workarounds; the engine handles loss and relocalisation). With no pose for 350 ms: 3DoF at the last position, orientation continuous. |
| `PIBRIDGE_VIT_POSE_RATE=imu` | Every engine pose (about 1 kHz), as Pimax Play reads one per IMU sample. (`cam` = one per frame set, for the Basalt-style supervisor.) |

## Controllers: Pimax's LED engine (`LedObjectPoseEstimator.dll`)

With the Pimax head engine and optical controller tracking on (`pimaxCtrlEngine`: also needs the controllers' LED
models in the kit), the controllers come from Pimax's LED engine in the same winbridge, as in Pimax Play, and
pibridge-ctrltrack is not started. Everything below is from the Frida traces of Pimax Play's service (pass 1-3).

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
  8.2 LSB/(deg/s), see below); the pose comes back in `out` (quaternion, position, angular and linear velocity).
  - **What `out` holds** (RE of LedObjectPoseEstimator 2.11.2, 2026-11-20):
    - `out[23]` = 1.0 when the pose is valid: tracked, with a vision update within 66 ms.
    - `out[24]` (byte) = 1 while the controller is in the 6DoF tracking state.
    - `out[20..21]` (u64 ns) = the pose time: the IMU sample time less the engine's 5.4 ms.
  - **Validity.** `out[]` is filled with the IMU-only propagation when the pose is not valid too. winbridge therefore
    takes a pose only with `out[23]`; until 2026-11-20 any non-zero position passed, so stale poses drifted.
    `PIBRIDGE_WB_CTRL_ANY=1` gives back the old test.
- **Head SLAM's saved map** (RE 2026-11-20, pimax_slam 2.0.0.1).
  - **Map mode.** The 5th argument of `HeadsetInitialImpl` (low byte) picks it; `HeadsetSetTrackingMode` is only the
    6DoF / 3DoF switch.
    - 0 = kNormal, as Pimax Play: load `<serial>platMap_orborb_K8L4.{bin,yaml}` to relocalize, save it back.
    - 1 = kLabMap: no load, saves `LabplatMap_*`.
    - 2 = kLabLoc: loads `LabplatMap_*`, never saves.
  - **Bounds (hardcoded).** 1500 keyframes per map (the oldest dropped), at most 3 maps (the one with the fewest
    keyframes deleted), sessions under 50 keyframes thrown away.
  - **Saves.** The whole file is saved once during play, when the session reaches 200 keyframes (relocalization pauses
    meanwhile), and again at shutdown.
  - **Size.** Ours was 207 MB / 3934 keyframes; Pimax Play's on Windows was 65 MB.
  - **`PIBRIDGE_WB_SLAM_MAP`:**
    - `normal` (default);
    - `loc`: kLabLoc on a pinned copy, `<serial>platMap_*` copied to `LabplatMap_*` when the copy is missing or older;
      the map stops growing and no save runs during play;
    - `fresh`: kLabMap.
  - **Pose time.** `PIBRIDGE_WB_CTRL_POSE_TS=1` stamps the pose with the engine's pose time instead of the IMU sample's
    (not yet A/B'd live).
- IMU time: `ts` is the report's device timestamp (`ts_us`, le32 at byte 8), as Pimax Play passes it: in pass 5 the
  head IMU, the controller IMU and the camera frames are on one clock (the headset's, us resolution, the same drift
  against QPC; controller steps are multiples of the ~1978 us sample period). pibridge-headtrack puts `ts_us` through
  the head IMU's clock mapping, the one the LED frames use; if that does not land within 100 ms of the arrival (the
  controller's clock is not the headset's after all) it uses a least-delay mapping of the hand's own, and failing that
  the arrival time. Its log says which: "left controller IMU time for the engine: ...". Until 2026-11-19 the arrival
  time was always used, which adds the radio/USB delay and its jitter to every sample;
  `PIBRIDGE_CTRL_IMU_TS=host` (Advanced > Extra environment) gives that back for an A/B. Not yet checked live which
  mapping a real session picks, nor whether it cures controllers that stick after some minutes (issue #3).
- Axes: the LED engine works in (x, z, -y) of the SLAM world (world rotation Rx(-90 deg), body unchanged): derived
  from one head pose that pass 1 and pass 3 caught in both forms. winbridge turns the head pose into it and the
  controller poses back.
- Both IMU samples of a report go to the engine (`hmd::controller_imu`), as Pimax Play does (twice the calls of one
  per report). Measured on two captures (`ctrl_buttons_hub2`, `p82`): reports come at ~250 Hz per hand; one
  controller samples every ~1978 us with two samples in ~98 % of its reports, the other every 2500 us with one or two;
  s0 (bytes 16) is the earlier sample, s1 (bytes 30) one period later (s1 continues into the next report's s0), and
  `ts_us` is s0's time, stepping by samples x period. So the second sample is sent at the first's time plus this
  hand's period, learnt from its reports (`hmd::ControllerSamplePeriod`, the median of the last 15 steps per sample).
  Until 2026-11-19 only the first sample went in: half the engine's IMU rate (its log: `incomplete IMU interval`).
  `PIBRIDGE_CTRL_IMU_BOTH=0` gives the old behaviour back for an A/B.
- Path: pibridge-headtrack hands the LED frames and every controller IMU report to the extension of
  `libpibridge_vit_winbridge.so` (`pibridge_winbridge_ext.h`, found by dlopen of the VIT library), which sends them over the same pipelined socket. The reply carries the controller pose
  and the head pose the engine used; pibridge-headtrack takes the controller relative to the head and puts it under its own
  head pose at the sample time (so recenter and the room anchor apply as to the head), moves it on to display time
  (engine velocity, controller gyro), turns the body into the aim frame (rot_x(-52 deg), as pimax-ctrltrack) and the
  grip. Not seen by the engine: position held and orientation turned by the gyro for
  `PIMAX_CTRL_TRACK_LOST_S` (4 s), then the rest pose.
- Threads in winbridge: every LED engine call runs on one worker thread of its own (still a single thread for that
  DLL, in arrival order), so a slow controller engine can no longer hold up the head's IMU and camera messages in the
  socket loop (issue #3: the client's queue overflowed, frames were dropped and the engine lost the controllers). The
  socket loop answers at once: an LED frame set goes into a latest-wins mailbox (a set the worker has not started
  when the next one arrives is replaced, counted), a controller IMU sample into a bounded queue (oldest dropped when
  full, counted), and `MSG_CPOSE` carries the latest pose the worker has for that hand (its own timestamp; the client
  keeps the last tracked pose and judges its age anyway). `PIBRIDGE_WB_CTRL_THREAD=0` brings back the inline calls.
  Every 5 s winbridge.log has `timing 5 s, socket loop:` (`HeadsetPutCameraImage`, `HeadsetPutHmdImuData`) and
  `timing 5 s, controller worker:` (`ControllerPutCameraImage`, `ControllerPutControllerImuData`: count, mean, max ms;
  IMU queue depth and drops, replaced LED frame sets). With synthetic black frames under Wine 10
  `ControllerPutCameraImage` returns in ~0.03 ms (it hands the frame to the engine's own tracking thread).
- Offline (`ctrl_prep` + `vit-replay --ctrl` on a 30 s take with LED frames and both controllers' IMU, no head data):
  the engine tracks both hands, poses for about 80 % (left) and 76 % (right) of the IMU samples, nothing dropped;
  head-to-controller distance within 5 cm of the take's own tracker (a constant origin offset).

## Room state (floor and recognition)

The engine's own room getters are carried over the same extension (`pibridge_winbridge_ext.h`, version 2;
`MSG_ROOMREQ` -> `MSG_ROOMSTATE`): `HeadsetGetGroundState` (the floor) and `HeadsetGetLocModeState` (the
localization / recognition state). Each is `int fn(void* out)` returning `-1` (engine not up), `0` (not ready yet) or
`1` (valid); GroundState fills a flag byte then a run of floats (the floor plane/height, layout still being pinned
down), LocModeState fills one byte (the localization state). `pibridge_wb_room_state()` returns the last reply and
queues a fresh request, so pibridge-headtrack polls it at ~5 Hz and writes `room-state.json` (ground_ret, ground_flag,
g0..g3, loc_ret, loc_state) and logs changes -- the observe-and-pin-down step before the room setup uses them for the
floor and a "room recognized" gate. Absent on older DLLs (the getters return `-2`).

## The session

`RecSession` (`core/rec_session.*`), START VR in the inside-out mode: once the camera broker's socket is up,
`wine winbridge.exe` starts (`WINEDEBUG=-all`, `PIBRIDGE_WINDATA=Z:\...` = the kit) before pibridge-headtrack and
SteamVR; pibridge-headtrack gets `--vit <libpibridge_vit_winbridge.so>`. Stop ends the processes by PID in reverse
order, so the engine ends after SteamVR and pibridge-headtrack (wine execs into `winbridge.exe`: the started PID is the
server's). If the engine ends on its own, the session goes on; the VIT library loses it and the head is 3DoF until the
next start. Another running
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
  the VIT interface expects and no axis change is needed.
- App tests: `pimax_slam_engine_environment`, `session_pimax_engine_runs_winbridge_before_monado_and_stops_it_after`
  (`app/tests/test_record.cpp`).

- `PIBRIDGE_SHUTTER_NS` 0.1 / 10 / 20 ms give the same result: the engine only needs a non-zero value, it does not time
  the frame with it.
- On the two-camera recording the engine's rotation between frames follows the gyro except in fast turns
  (> 100 deg/s: p95 1.2 deg, max 4.5 deg). Black frames for the missing side cameras instead of copies: a little
  better. The four-camera feed (as Pimax Play) is the test that is still open.

## Not done yet

- A recording with all four cameras (`PIMAX_SLAM_CAMS=0,1,2,3`) for the offline check, and a Windows take of the same
  motions in Pimax Play to compare with.
- The real exposure is not passed (not needed for timing, see above).
- The controllers through the engine are not run with the live headset yet (tested offline up to the extension).

Settled by Windows pass 5 (`re-dll/README.md`, frida-pass5.json + the USB capture, Beat Saber Extreme):
- Controller IMU: matching the raw USB HID reports with the engine's inputs gave the gyro scale **8.2 LSB/(deg/s)**
  (not the old 16.4 guess; 2x), accel 1024 LSB/g, axes identical. Used by pibridge-headtrack (`PIMAX_CTRL_GYRO_LSB_DPS`).
- The pose predictor need not be called: its lead time is a ns timestamp in the `PosePredict` record (head ~16 ms,
  app/controllers clamp at 25 ms).
- The floor and play area do **not** come from the engine (`HeadsetGetGroundState` is only SLAM's own ground point,
  unchanged by a room setup). The user's floor and centre live in `chaperone_info.vrchap`; PiBridge's own room setup
  (roomsetup / roommap) owns the zone, not winbridge.