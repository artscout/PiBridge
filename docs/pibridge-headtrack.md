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
  the sample time and velocities; SteamVR predicts to the display time. Our controllers too (2026-11-22): the
  tracker's velocity and the gyro's rate while optical, none while held; the driver passes them with the sample's age
  (it zeroed them from 2026-11-20 to 2026-11-22: a frame of lag, the saber running on at swing reversals). The file is written in one `pwrite` (since
  2026-11-21): the earlier four-step seqlock write left it torn for readers whenever headtrack's output thread was
  preempted between the steps, which under full CPU load (Beat Saber + shadow engines, load 8.5 on 8 cores) happened
  250 times in 7 minutes; CustomHeadsetOpenVR then sent SteamVR one untracked head pose for a 2 ms tick and the view
  jumped for a frame (the "slow blink", the floor dropping for a moment; vrserver.txt `hmd: tracking 2 -> 0 (fresh 0,
  state 0, flags 0)`, the overlay's `ft` lines `hmd invalid 1`). The driver now also keeps its last consistent copy for
  up to 100 ms over a torn read and counts them (`hmd: ... torn N` in its per-second line).

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

## Head shadow mode

`--shadow-vit LIB [--shadow-slam-config FILE] [--shadow-cams 0,1]` (the app: `PIBRIDGE_HEAD_SHADOW=1` in the extra
environment with the Pimax engine) runs Basalt on the same frames and IMU beside the driving engine; it drives nothing.
The log is `~/.local/share/pibridge/head-shadow/head-shadow-YYYYmmdd-HHMMSS.csv`, columns
`t_ns,src,px,py,pz,qx,qy,qz,qw,vx,vy,vz,event`, at most one row per 5 ms per source:

- `src 0`: the driving engine's raw pose as it comes (engine world, z up, the pose's own time);
- `src 1`: the shadow engine's raw pose (z up); `event = diverged` when it ran away (3 m/s, a 0.5 m jump or 50 m off)
  and was started over;
- `src 2` (2026-11-20): PiBridge's output layer (`headpose::HeadTracker`, the one `--engine basalt` drives SteamVR
  with) run on the shadow engine's poses and the IMU, asked for the head at `t_ns` = now: y up, the world of the
  layer, `event = tracked` / `3dof`; `event = reset ...` when the layer asked for an engine reset. This is what the
  apps would see with Basalt, beside what they see with Pimax, on one timeline: the latency, the prediction on turns
  and the drift are measured on it (`~/gamedev/ext/slamrec/tools/head_shadow_cmp.py LOG`: lag by angular-speed
  cross-correlation, rotation error still / turning and its sign along the turn axis, position error by time).

Measured so far on the raw engines (src 1 vs src 0, sessions of 2026-11-20 before src 2 existed): no time lag between
the poses' own timestamps; rotation error 2.6-4.6 deg p50 after a constant alignment (Basalt's yaw wanders, the
alignment is from the first 10 s); on turns Basalt is not turned further than Pimax (+0.3 deg along the turn axis), so
an over-turn of the picture, if real, is the output layer's prediction, not the engine's; position error 11 cm in the
first 30 s growing to 23 cm by the 5th minute (lever arm between the two engines' reference points plus Basalt's drift).

### What drifts: yaw, not tilt (2026-11-21)

The 20-minute Beat Saber session of 2026-11-20 23:33 (`head-shadow-20261120-233325.csv`) was first read as a tilt
drift of Basalt (about 0.4 deg/min, "the head sank 20 cm"). That was an axis mix-up of the analysis script
(`head_drift.py` took src 0 as y-up; the raw engine rows, src 0 and 1, are **z-up**, so its "tilt" column held the
rotation about the vertical and its "height" column a horizontal axis). Split along Pimax's real vertical
(`~/gamedev/ext/slamrec/tools/head_drift2.py LOG SRC [WINDOW_S]`, which finds the up axis from the poses), Basalt against
Pimax after one alignment from the first 10 s is:

| minute | yaw error (about the vertical) | tilt error x, y (horizontal axes) | vertical position error | local error (re-aligned per window) |
|---|---|---|---|---|
| 0-2 | +0.6 deg | -0.9, -1.6 deg | +1.5 cm | 0.5 deg, 1.5 cm |
| 6-8 | +0.9 | -1.0, -2.1 | +1.5 | 0.5 deg, 2.1 cm |
| 10-12 | +2.6 | -1.2, -2.1 | -0.6 | 0.5 deg, 2.1 cm |
| 14-16 | +5.7 | -1.3, -2.0 | -3.1 | 0.7 deg, 1.6 cm |
| 18-20 | +8.1 | -1.3, -1.9 | -8.2 | 0.4 deg, 2.4 cm |

So Basalt's gravity (tilt) is stable to a few tenths of a degree over 20 minutes (the constant 1-2 deg is the alignment
offset between the two engines' body frames); what runs away is the **yaw**, 0.1 deg/min for the first 7 minutes, then
0.6 deg/min to the end, and the 25 cm "position error" is that yaw times the ~1.8 m lever arm to the alignment origin
(the horizontal error; the vertical one is 8 cm). The output layer (src 2) shows the same numbers. The yaw drift does
not correlate with the head motion within 10-s windows (correlation with the net head yaw, the summed |yaw rate| or
the path length: |r| <= 0.15): it is a slowly varying yaw-rate error of the engine, not a per-turn scale error. The
other sessions of 2026-11-20 (same configuration) drift in yaw at 0.6-1.7 deg/min with either sign (01:13: -0.8;
03:06: -0.8; 03:23: +0.75; 05:47: -0.6; 06:35: -1.7 deg/min) while their tilt stays within 1 deg: the rate and its
sign change from session to session, as a yaw gyro-bias error does (a MEMS gyro's bias moves by that much,
0.007-0.03 deg/s, while it warms up). Yaw is the one direction a VIO cannot observe from gravity: it comes from the
landmarks alone, through the gyro bias estimate. Which of the inputs lets Basalt lose it is not settled; the suspects
are, in order:

1. the IMU noise model: the live calibration has the measured (x3) densities (`accel_noise_std` 7e-4, `gyro_noise_std`
   8.5e-5 at 1000 Hz; Basalt scales them by sqrt(rate): 0.022 m/s^2 and 0.0027 rad/s per sample). A yaw-bias error of
   0.5 deg/min moves the features by 0.04 px over Basalt's ~1.5-s window (7 keyframes), far under
   `vio_obs_std_dev` 0.5 px, while the gyro-integrated rotation is weighted 0.001 deg: the window follows the gyro and
   the landmarks slide with it. Basalt's EuRoC defaults (0.016, 2.82e-4) and Pimax's Windows values (0.0817, 1.97e-3)
   weigh the gyro 3x and 23x less;
2. `gyro_bias_std` 1e-4 rad/s^2/sqrt(Hz) (EuRoC's) lets the bias estimate walk 0.006 deg/s per sqrt(s), 10-100x the
   bias instability of a MEMS gyro, so the bias follows the window's vision noise instead of its long-term mean;
3. a camera-IMU time offset or a gyro scale error (factory `kg` 0.03-0.3 %): both would give a yaw error that follows
   the turns, which the data does not show (so not the main term);
4. the accel bias and its frame: fine (the factory `aBias` brings the resting accelerometer from 10.11 to 9.77 m/s^2;
   the gravity tilt is stable);
5. `T_imu_cam`: a constant offset only (the 1-2 deg tilt offset above).

**A/B on the first two takes (2026-11-21, Beat Saber, 408 s and 380 s, replayed; `head_drift2.py LOG 1 60 60`
aligns after the settling first minute):** on take 1 `gyro_bias_std` 1e-5 looked best (yaw within +-2.2 deg against
-4.0 for the live calibration, tilt stable, height drift halved; EuRoC and Pimax noise worse, 1e-6 worse), but on take
2 the same setting is clearly worse (-9.3 deg at 360 s against -1.7 deg for 1e-4), and live with 1e-5 it reached
-5.2 deg. The base calibration therefore stays at `gyro_bias_std` 1e-4 (a trial 1e-5 was reverted the same night).
What both takes show instead: the yaw error is not a steady walk but swings by several degrees between minutes with the
same shape for every noise model (so it follows the motion or the scene, not the bias), while the tilt stays within a
degree. The camera-IMU time offset was tried on take 2 (`--imu-offset-ns` -5, -2, +2, +5 ms): the yaw is sensitive to it
(-2 ms doubles the error to 9 deg) but the optimum is at the present offset (0 and +2 ms alike, |yaw| mean 2.2 deg),
so the offset is not the lever. Next: the gyro scale (factory `kg`), and a check whether Pimax itself re-aligns its
yaw against its map (its "error" would then be Pimax's correction, not ours).
A replay is not bit-exact against the live run (threads, `deterministic=0`): the same toml differed by up to 4 deg
at the end of take 2, so compare variants on several takes, not on one window.

The two offline takes with a Basalt replay (the EuRoC recordings of 2026-10-19) carry no Pimax reference and last
55 s, so they cannot rank these; the live log cannot be replayed because it holds no frames or IMU. The next live
session can be, with the take below.

### Head shadow take and replay

`PIBRIDGE_HEAD_SHADOW_TAKE=1` in the extra environment (with `PIBRIDGE_HEAD_SHADOW=1`) records exactly what the shadow
engine is fed, next to the log (a directory instead of `1`, such as `/mnt/g/Devel/pibridge-rec`, puts the two files there
under the same names): `head-shadow-*-take.bin` (the frame sets, `shadow_take.h`: the tiles, the frame time
given to the engine and the push time; raw 15 MB/s, about 18 GB for 20 minutes; `PIBRIDGE_HEAD_SHADOW_TAKE_ZLIB=1`
compresses them roughly 2:1 on a thread of its own) and `head-shadow-*-imu.csv` (the IMU samples as pushed, body axes,
libpibridge-cam imulog `I` lines). The camera thread only copies into a bounded queue; frame sets are dropped and
counted when the disk falls behind (`head shadow take: N frame sets, M dropped` at the end).
`PIBRIDGE_HEAD_SHADOW_TAKE_SEC=300` stops the take that many seconds after its first frame set while the session (and the
shadow log) go on: five minutes is enough for a yaw-drift A/B (the drift is 0.4–1.7°/min) and costs about 2.3 GB zlib.

```
pibridge-headtrack-replay head-shadow-X-take.bin head-shadow-X-imu.csv --vit .../libbasalt.so --slam-config VARIANT.toml \
    --ref head-shadow-X.csv --out run-VARIANT.csv
python3 ~/gamedev/ext/slamrec/tools/head_drift2.py run-VARIANT.csv 1 120
```

The replay pushes the IMU and the frame sets in the live order (a frame set once the IMU has reached its time plus the
take's own median lead), waits for the engine when its poses fall more than `--max-lag-ms` behind, starts it over
when it diverges as the live mode does (`--no-reset` to keep it), and writes the poses as src 1 rows; `--ref` copies
the live log's src 0 rows (Pimax) in front, so `head_drift2.py` compares the replayed Basalt with the Pimax of that
very session, per 2-minute window: the yaw error per minute is the number to drive down. `--imu-offset-ns` /
`--cam-offset-ns` shift one clock for the time-offset experiment. Basalt runs about 4.5x real time with 4 threads
(`num-threads=4` in the toml) and, on the same take and toml, reproduces its poses to the printed precision, so one
run per variant is enough. Variants are a copy of `~/.config/pibridge/slam/pimax-basalt-calib.json` with other
`*_noise_std` / `*_bias_std` values and a toml naming it (the app rewrites the originals at every start). The ones
to run first: the live calibration as it is; `gyro_noise_std` 2.82e-4 + `accel_noise_std` 0.016 (EuRoC); 1.97e-3 +
0.0817 (Pimax); the live one with `gyro_bias_std` 1e-5. On a 55-s offline take these four already differ from each
other by 0.3-0.9 deg of yaw, i.e. the yaw is sensitive to the noise model at the rate seen live.

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

## Stillness check (2026-11-21)

First evening with `--engine basalt` driving SteamVR (Beat Saber, 4 min): playable; the user saw the head "sway a
little" in the SteamVR home and a slightly lagging recovery. The driver saw 0 untracked and 0 invalid head poses
(46 torn reads, all held). What the logs can and cannot say:

- A `PIBRIDGE_CTRL_DUMP` carries the head pose on every controller IMU row (`I,0,...` columns 16-22, ~220 Hz), so a
  dump doubles as a head-output log. `ext/slamrec/tools/head_wobble_dump.py` looks at quiet 2 s windows (head turning
  < 3-6 deg/s, moving < 5 cm/s): Basalt live, detrended position RMS 1.3/0.7/1.7 mm (x/y/z), orientation
  0.05-0.08 deg; the Pimax engine's dump (win4) 0.5-1.7 mm and 0.15-0.3 deg in its few quiet windows. No extra wobble
  of Basalt at 2 s scale, and no holds or steps while worn.
- The head shadow CSVs (`head_wobble_shadow.py`, `head_wobble_shadow_rot.py`, src 0 vs src 2) put Basalt's output and
  Pimax's raw pose side by side: orientation disagreement 0.02-0.03 deg RMS (nothing); position difference 0.7-1.4 mm
  RMS in 2 s windows, residuals barely correlated (each engine's own mm noise, Pimax's is not smaller); in 6 s windows
  Basalt's vertical wanders 2.4 mm RMS (p90 5 mm, ~0.3 Hz) against Pimax's 0.9 mm. That slow vertical breathing (and
  the ~1 cm/min climb seen earlier) is the only measurable candidate for the "sway"; a 2-camera shadow, 4 windows.
- Not in any log: the live session's headtrack stdout (3DoF episodes, corrections, re-snaps) and the output's age.
  Next time: `pibridge-headtrack ... 2>&1 | tee /mnt/g/Devel/pibridge-rec/headtrack-HHMM.log`, and
  `pibridge-posetap --raw --secs 90 > posetap-basalt.txt` while standing still in the SteamVR home, then the same
  minute with `--engine pimax`: the two files compare the actual 1 kHz output (steps, ages) of both engines directly.
- Recovery after a loss is slow by design (`headpose.h`): the re-acquired pose is blended in with tau 0.15 s
  (0.4 s at rest, `PIMAX_SLAM_TAU_REST_MS`), never faster than 0.1 m/s + half the head speed (`v_min`, `k_v`),
  rotation tau 0.1 s; re-snap restores the room when yaw agrees within 3 deg. Whether the lag seen was this slide or
  Basalt's own re-initialisation needs the headtrack log.

**Found the same evening (sessions 21:12 and 21:55):** the "position moves a few centimetres, several times" the user
saw with Basalt live was the room map's `--follow` (every fix's 1-3 cm / 0.3-1.1 deg scatter applied as a drift
correction every 10-16 s, 19 glides in 4 min); retired in favour of `auto --confirm 12` (`docs/pibridge-roommap.md`).
A `PIBRIDGE_ROOM_FOLLOW=0` session of 7 min felt right to the user. Headset on the desk, cameras to the room (take
`head-shadow-20261121-215517`): Pimax's engine stands to 0.2 mm / 0.001 deg/min over 6 min; raw Basalt runs away
at ~3 mm/s in one direction and -6..-12 deg/min of yaw, so the shadow's still-hold reset it 30 times in 6 min and the
held output's yaw drifted -25 deg (each re-zero carries the drifted yaw into the restarted tracker, which repeats its
init transient). Replayed without resets: a 90 s transient after the put-down (15 cm, -8 deg), then +-3 cm; IMU noise
models x10 do not change it. Next for the output layer: do not restart the tracker while still (hold the position
only, or far less often), and hold yaw by the bias-corrected gyro while still rather than by the tracker.
