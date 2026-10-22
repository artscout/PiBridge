# Tracking: the tracking mode and inside-out head tracking (experimental)

The **Tracking** page (called "Inside-out" before; the recorder that was called "Tracking" is now the **Development**
page, [DEVELOPMENT.md](DEVELOPMENT.md)). The saved settings did not change: `tracking_mode`, `io_*` and `rec_*` are
read as before.

PiBridge knows two ways to track the Crystal Super:

| Mode | What tracks | What it needs | State |
|---|---|---|---|
| **Lighthouse** (default) | SteamVR's lighthouse driver: head and Index controllers from the base stations | the Lighthouse faceplate, base stations, CustomHeadsetOpenVR (docs/INSTALL.md) | works |
| **Inside-out** | the Pimax driver in Monado, run by SteamVR's `driver_monado.so` inside vrserver (**SteamVR + Monado**): Basalt SLAM on the headset's two tracking cameras plus the IMU; the Pimax controllers' LEDs found in the cameras (`pimax-ctrltrack`). Games start through SteamVR as usual | a Monado build with the Pimax driver and its `steamvr-monado/` (omarchyvr `forks/monado` + the Crystal patch, or the PiBridge-linked build), `pimax-cam`, `pimax-ctrltrack`, Basalt (`libbasalt.so`), the headset's factory calibration (`rec_cam_calib`), a base Basalt config (`rec_slam_config`), SteamVR and Steam | experimental |

"Monado directly" (monado-service owns the headset, own apps on it) is no longer a choice of this page: it is the
Development page's debug / A-B start ([DEVELOPMENT.md](DEVELOPMENT.md), "Monado directly").

Everything is on the **Tracking** page (left column); nothing has to be typed. The settings are
saved in `~/.config/pibridge/pibridge.conf` and used by every place that starts an inside-out Monado: **START TRACKING**
(SteamVR + Monado), and **Record tracking** and **Monado directly** (Development page, in every mode). Both get their environment from one builder,
`buildIoEnv()` in `app/src/core/inside_out.*`; the recorder's `monadoEnv()` is that builder plus `PIMAX_SLAM_RECORD`.
Changes apply at the next start; a running session keeps the settings it started with.

Why a page of its own: the Development page is the recorder (a protocol, its steps, the recordings list). The mode and
twenty driver settings are a different job and would push the recorder off the screen; both pages share the settings.

## Motion-JPEG instead of YUYV (what that path loses)

The tracking cameras (`34a4:0014`) report themselves as Motion-JPEG instead of YUYV when their USB link comes up at
480 Mbps — a USB 2.0 port, or a 2.0 hub anywhere in the chain. The same headset and the same firmware report YUYV
again on a SuperSpeed port (checked 2026-10-22: `speed=480` gave one 1280x832 MJPEG frame, `speed=5000` gave
1280x832 YUYV with `flags 0x1` and device timestamps), so check the link before blaming the firmware:

```
for f in /sys/bus/usb/devices/*/idVendor; do d=$(dirname "$f")
  [ "$(cat "$f")$(cat "$d/idProduct")" = 34a40014 ] && echo "$d $(cat "$d/speed") Mbps $(cat "$d/product")"
done
```

480 means the format is a link limit, not a firmware one: YUYV 1280x832 is 2.1 MB a frame and even 25 fps needs about
53 MB/s, which a 480 Mbps link cannot carry, so the camera offers JPEG there. Move the cable to an SS port; a USB 2.0
hub caps everything behind it at 480 Mbps.

If it stays at 480 (or the port is 2.0 only), libpibridge-cam negotiates MJPEG, decodes each frame and uses its luma
plane, which holds the two tracking cameras side by side (`StreamProfile::JpegTiles2`,
`docs/libpibridge-cam.md`). The frame goes on in the usual layout, so START TRACKING works the same way — but what
JPEG threw away cannot come back:

| Lost with the metadata | What it means while tracking |
|---|---|
| the device timestamp of the frame | the frame is timed on the host clock, so it sits further from its IMU samples (by the USB latency) and the tracker corrects the head a little later. The mid-exposure shift cannot be computed either (no exposure), so a fixed one can be asked for with `PIMAX_SLAM_CAM_OFFSET_US=1638` (the factory half exposure) |
| the exposure and gain of the frame | no auto-exposure (`io_cam_ae`): the camera keeps what it booted with, there is nothing to measure the result with |
| the two controller cameras | no controller-LED images for `pibridge-ctrltrack`: the Pimax controllers keep their IMU pose only |
| the exact metadata bytes | the frame kind comes from where its tiles start (row 96 for the tracking exposure, row 9 for the short controller one), so SLAM still gets its 30 fps of normal-exposure frames |

`pibridge-cam info --source auto` shows `MJPEG/grey` and `flags 0xa` for such frames, and a `.pxcam` records the
profile, so a replay behaves like the live stream.

## The page

Each setting shows a label, one line of help, a **RECOMMENDED** mark while it has the recommended value, and, in amber,
why it does not apply right now (the control is then disabled), for example "Needs the Basalt library" or
"Auto exposure is off". **Reset to recommended** (Advanced) puts every `io_*` key back; it leaves the mode, the home
scene, the refresh rate, the paths and the extra environment alone.

Screenshots (`pibridge --screenshots DIR`): `09-tracking.png` (Lighthouse mode, the defaults), `09b-tracking-lower.png`
(head tracking, PiBridge's Basalt values, the start environment), `09c-tracking-inside-out-ready.png` (what a start
changes in SteamVR's files), `09d-tracking-running.png`, `09e-dashboard-tracking-running.png`,
`09f-tracking-steamvr-some-off.png` (reasons shown), `06c-apps-inside-out-steamvr-off.png` ("Play starts tracking"),
`08c-room-inside-out.png`; for "Monado directly": `07f-development-monado-directly.png`, `06d-apps-monado-running.png`,
`09g-dashboard-monado-directly.png` (the Controllers card reading Monado, the headset card saying what Monado does).

## Settings, their sources and defaults

`pibridge.conf` key, values (**default**), what the driver gets. "Off only" = the driver's own default is on, so only
the off value is passed (as omarchyvr's launcher passes only the flag of a switch that is off).

| Group | Setting | Key | Values | Environment | Source |
|---|---|---|---|---|---|
| Mode | Head tracking by | `tracking_mode` | **lighthouse**, inside-out | (which setup the Dashboard offers) | |
| (legacy) | Apps run through | `io_apps` | steamvr, monado | still read, chooses nothing: a pibridge.conf with `monado` gets SteamVR + Monado here; "Monado directly" is started on the Development page (test `tracking_runtime_follows_the_sessions`) | `crystal-6dof.sh` (`steamvr` / default branch) |
| Development | Home scene in the headset | `io_home_scene` | **on**, off | pibridge-home while "Monado directly" runs ([home.md](home.md)); the switch is on the Development page | |
| Session | Display connector | `rec_connector` (shared) | **"" = auto** (EDID says Pimax), a connector | `XRT_COMPOSITOR_WAYLAND_CONNECTOR` (monado-service only: Record tracking, Monado directly; SteamVR's compositor picks the display itself) | `crystal-6dof.sh:41` |
| Session | Refresh rate | `refresh_hz` (shared) | 72, **90** | `PIMAX_FPS` | `pimax_hmd.c:81` (driver default 72), `crystal-6dof.sh:23` (90) |
| Head | 6DoF head tracking | `io_slam` | **on**, off | `PIMAX_SLAM=1/0`, `VIT_SYSTEM_LIBRARY_PATH`, `SLAM_CONFIG` | `pimax_camera.c:158`, `t_tracker_slam.cpp:87-88` |
| Head | Head tracking engine | `io_slam_engine` | **basalt**, pimax | pimax: `VIT_SYSTEM_LIBRARY_PATH=libpibridge_vit_winbridge.so` instead of Basalt, no `SLAM_CONFIG`, `PIMAX_SLAM_CAMS=0,1,2,3`, `PIMAX_SLAM_EXPOSURE_MID=0`, `PIMAX_SLAM_PASSTHROUGH=1`, `PIBRIDGE_VIT_POSE_RATE=imu`; with optical controller tracking on also `PIMAX_CTRL_ENGINE=1` (Pimax's LED engine, no pimax-ctrltrack); the session runs `wine winbridge.exe` ([WINBRIDGE.md](WINBRIDGE.md)) | Pimax Play's `pimax_slam.pi.dll`, `LedObjectPoseEstimator.dll` |
| Head | Head tracking cameras | `io_slam_cams` | **2** (lower pair 0, 1), 4 (also the upper side pair 2, 3) | 4: `PIMAX_SLAM_CAMS=0,1,2,3` and a four-camera calibration in PiBridge's Basalt config (below); 2: nothing (the driver's default 0,1). 2 until a live take decides | `pimax_camera.c` `PIMAX_SLAM_CAMS`; ROADMAP 1b |
| Head | Detection grid | `io_slam_grid` | **30**, 40, 50 px | PiBridge's Basalt config: `config.optical_flow_detection_grid_size` | research doc, study 2 (recommendation 3) and study 3 |
| Head | Accelerometer bias | `io_slam_accel_bias` | **factory**, off | PiBridge's Basalt config: `calib_accel_bias = [aBias, 0 x 6]` | research doc "What PiBridge should implement"; pimax-cam `slam.rs` `factory_accel_bias` |
| Head | Gyro bias start value | `io_slam_gyro_bias` | **measured** (driver cache, else factory `wBias`), factory, off | PiBridge's Basalt config: `calib_gyro_bias = [b, 0 x 9]` | research doc; `slam.rs` `parse_gyro_bias_cache`; `pimax_hmd.c:255` (cache path) |
| Head | Drift hold (experimental) | `io_drift_hold` | on, **off** | `PIMAX_SLAM_DRIFT_HOLD=0` when off | `headpose.rs:765`, `drift.rs:75` (driver default on); `crystal-6dof.sh:26` (`=0`); research doc "Third recording" (off for the Crystal Super: mark error 273 mm with it, 74 mm without) |
| Head | Still gate | `io_still_hold` | **on**, off | off only: `PIMAX_SLAM_STILL=0` | `headpose.rs:774`, `slam.rs:578` |
| Head | Tilt correction | `io_head_tilt` | **on**, off | off only: `PIMAX_HEAD_TILT=0` | `headpose.rs:756`, `:700` |
| Head | Automatic fusion | `io_fuse_auto` | **on**, off | off only: `PIMAX_FUSE_AUTO=0` | `headpose.rs:753`, `:707` |
| Head | Smooth corrections | `io_slam_smooth` | **on**, off | off only: `PIMAX_SLAM_SMOOTH=0` | `headpose.rs:730`, `:672` |
| Head | Room anchor | `io_anchor` | on, **off** | `PIMAX_ANCHOR=1` (also passed whenever `pibridge-roommap` is found: the room map, below) | `pimax_anchor.h:209` |
| Head | Room corrections glide | `io_anchor_glide` | **on**, off | off only: `PIMAX_ANCHOR_GLIDE=0` | `pimax_anchor.h:233` |
| Cameras | Auto exposure | `io_cam_ae` | **on**, dry, off | `PIMAX_CAM_AE=1/dry/0` | `pimax_camera.c:163`; `crystal-6dof.sh:26` (1); research doc study 2 (dark images with AE off) |
| Cameras | Exposure model | `io_cam_ae_model` | **crystal-super**, auto | `PIMAX_CAM_AE_MODEL=crystal-super` | `ae.rs:229`, `crystal-6dof.sh:26`. Windows ranges enforced: `ae.rs:137` (2200..10410 us / gain 13..449, 2200..8941 us / 13..439) and the allowlist |
| Cameras | New exposure law | `io_cam_ae_law` | **new**, old | off only: `PIMAX_CAM_AE_LAW=old` | `ae.rs:854`; omarchyvr `trackset.rs` "New camera exposure" |
| Controllers | Optical tracking | `io_ctrl_track` | **on**, off | `PIMAX_CTRL_TRACK=1/0`, and `pimax-ctrltrack live` is started | `pimax_ctrl.c:82`, `crystal-6dof.sh:17` |
| Controllers | Controller angle | `io_ctrl_angle` | -20..70, **30** deg | `PIMAX_CTRL_ANGLE` (always passed) | `pimax_ctrl.c:84`, omarchyvr `ctrl_angle.rs:10-12` |
| Controllers | (no switch) | — | always (driver default) | `PIMAX_CTRL_INHAND` not passed: `auto` marked held controllers as put down and the tracker stopped looking for them (live 2026-10-19) | `pimax_ctrl.c` |
| Controllers | Out-of-view hold | `io_ctrl_lost_s` | 1..10, **4** s | `PIMAX_CTRL_TRACK_LOST_S` when not 4 | `fuse.rs:758`, `:691` |
| Controllers | Steadier controllers | `io_ctrl_steady` | **on**, off | off only: `PIMAX_CTRL_TRACK_HYST=0` | `fuse.rs:743`; omarchyvr `trackset.rs` "Steadier controllers" |
| Advanced | Extra environment | `rec_env` (shared) | `KEY=value` words | appended last (overrides) to monado-service's env and the driver file | |
| Advanced | Programs and files | `rec_monado_service`, `rec_pimax_cam`, `rec_pimax_ctrltrack`, `rec_basalt_lib`, `rec_slam_config`, `rec_lens_conf`, `rec_cam_calib` (shared) | "Choose..." / "Auto" | | docs/DEVELOPMENT.md |

Paths and line numbers: Monado = omarchyvr `forks/monado/src/xrt/drivers/pimax/` (and `auxiliary/tracking/`,
`state_trackers/steamvr_drv/ovrd_driver.cpp`), Rust = omarchyvr `rust/pimax-cam/src/`, `rust/pimax-hands/src/`,
`crystal-6dof.sh` = `~/gamedev/ext/tools/crystal-6dof.sh`, research doc = `docs/research/slam-imu-intrinsics.md`.

Fixed, always passed (from `crystal-6dof.sh` lines 22-29 and 41-42): `PIMAX_LOG=info`, `PIMAX_BIAS_TIMEOUT_MS=200`,
`PIMAX_LENS_CONF`, `PIMAX_CAM_CALIB`, `PIMAX_CAM_SOURCE=broker`; for monado-service also
`XRT_COMPOSITOR_FORCE_WAYLAND_DIRECT=1`, `XRT_COMPOSITOR_FORCE_NVIDIA=0`, `XRT_COMPOSITOR_LOG=info`, `XRT_NO_STDIN=1`,
`IPC_IGNORE_VERSION=1`.

Read but deliberately not offered (test aids or A/B switches in the sources): `PIMAX_SLAM_CAM_OFFSET_US`,
`PIMAX_SLAM_CAMS`, `PIMAX_SLAM_STILL_REZERO`, `PIMAX_SLAM_FUSE`, `PIMAX_SLAM_TAU_MS`, `PIMAX_SLAM_TAU_REST_MS`,
`PIMAX_SLAM_FREEZE`, `PIMAX_SLAM_RESNAP`, `PIMAX_CAM_AE_TARGET` and the other `PIMAX_CAM_AE_*` tuning, the
`PIMAX_CTRL_TRACK_*` fuser tuning, `PIMAX_CTRL_INHAND`, the hand-tracking `PIMAX_HT_*`, `PIMAX_HANDS`. Any of them
can still be set under Advanced > Extra environment.

## The head output's own defaults (libpibridge-cam, in every PiBridge-linked Monado)

Independent of the page's settings, the PiBridge-linked driver's head output layer has measured improvements over the
Rust one (docs/research/head-tracking-improvements.md): drift hold off unless `PIMAX_SLAM_DRIFT_HOLD=1` (the page's
switch), the output's gyro bias seeded from the driver's cache, the tracker's accel bias removed from the output's IMU
samples, the rotation predicted with the angular acceleration, and recordings that carry the calibration the tracker
used. `PIBRIDGE_HEAD_PARITY=1` under Advanced > Extra environment gives the exact omarchyvr behaviour for an A/B; the
single switches are `PIMAX_SLAM_SEED_BIAS`, `PIMAX_SLAM_ACCEL_BIAS`, `PIMAX_SLAM_PREDICT_ACCEL`,
`PIMAX_SLAM_STILL_BACKOFF` (off: measured worse), `PIMAX_SLAM_RECORD_ACCEL_BIAS`, and `PIMAX_SLAM_DRIFT_HOLD=2` (an
experimental hold only while the head is quiet). A Rust-linked Monado ignores all of them.

Not passed yet, recommended once a second take confirms it: `PIMAX_SLAM_CAM_OFFSET_US=-1500` with auto exposure on
(the camera frame times are about 1.5 ms late against the IMU; on the 07:04 take it lowered the mark error after the
turns / nods / shake from 52 to 29 mm and the output's motion error while turning by 16 %). The takes still needed are
listed at the end of that study.

## PiBridge's Basalt config

Values Basalt reads from its own files cannot be passed as environment. At every start (START TRACKING and Record
tracking) PiBridge writes its own copy into `$XDG_CONFIG_HOME/pibridge/slam/`:

- `pimax-basalt.toml`: the base config's lines (the one `rec_slam_config` names, usually `pimax-basalt.toml` from
  `omarchyvr-slam export`; read only, never changed) with `cam-calib` and `config-path` pointing at the two files below;
- `pimax-basalt-config.json`: the base settings, byte for byte, except `config.optical_flow_detection_grid_size`;
- `pimax-basalt-calib.json`: the base calibration, byte for byte, except `calib_accel_bias` = the factory `aBias`
  of `rec_cam_calib` (`<SFConfig><Stateinit aBias>`, used as is) followed by six zeros (the factory scale `ka` and
  misalignment `na` are never applied), and `calib_gyro_bias` = the driver's last bias measured at rest
  (`~/.cache/omarchyvr/gyro-bias`, raw axes deg/s turned into the body frame: x and z negated, rad/s), else the
  factory `wBias` as is, else the base value, followed by nine zeros.

**Four head tracking cameras** (`io_slam_cams=4`): before the copy, PiBridge runs `pibridge-slam export --cams 0,1,2,3
--calib <rec_cam_calib> --base <the base's config-path>` into `slam/cams4/` and takes only its camera arrays
(`T_imu_cam`, `intrinsics`, `resolution`) into the copy; the base's own cameras must be the first two of them
(the same headset), so IMU noise, biases and everything else stay the two-camera values and a comparison 4 vs 2
changes only the cameras. `config.optical_flow_detection_nonoverlap` is set to true in the settings (the upper pair
gets corners of its own; [pibridge-slam.md](pibridge-slam.md)). The driver gets `PIMAX_SLAM_CAMS=0,1,2,3` only when that
worked (or when there is no SLAM_CONFIG at all: the driver then hands Basalt its own four-camera calibration);
otherwise the lower pair is used and the page and the start say why (pibridge-slam not found, export failed, another
headset's cameras). The camera count must match: Basalt aborts the whole monado-service when its calibration has
another count than the cameras it is fed, so the driver also checks the SLAM_CONFIG calibration (fewer cameras: it
uses the first of `PIMAX_SLAM_CAMS`; more: no tracker, 3DoF, VR keeps running).

The frame time of a four-camera set: Basalt wants one timestamp per set, and the upper pair has its own auto
exposure (`ae.h`: pairs 0,1 and 2,3). The driver takes the mean mid-exposure of the tracker cameras (equal to the
pair's own with the lower pair alone, so two cameras are unchanged); each pair is then off by a quarter of the two
pairs' exposure difference (at most (10410 - 2200) / 4 us = 2 ms, usually much less; the AE line of the Monado log
shows both pairs' exposures each second). A recording with four cameras writes `cam0`..`cam3` (the tracker order).

`SLAM_CONFIG` then points at PiBridge's toml. If the base is missing or broken, the start goes on with the base as it is
(or, without any base, with the driver's own calibration and Basalt's built-in settings; the grid cannot be set then),
and the page says why. The page shows the values PiBridge would write (a dry run; nothing is written while you look).

**Recordings keep what the tracker used.** Monado's recorder writes the driver's own calibration into
`<recording>/pimax-basalt-calib.json`, which is not what the live tracker used when `SLAM_CONFIG` is set (study 3: it
showed `calib_accel_bias = 0` although the tracker had the factory bias). When a recording ends, PiBridge moves Monado's
file to `pimax-basalt-calib.driver.json` and copies the calibration and settings the tracker really used (those
`SLAM_CONFIG` names) into the folder as `pimax-basalt-calib.json` and `pimax-basalt-config.json`, so a replay uses
the same files.

## START TRACKING (SteamVR + Monado)

On the Tracking page (Session), on the Dashboard's Tracking card and its **Launch SteamVR** button (inside-out mode),
and through Play on the Apps page (a game or an own app while SteamVR is off). It is the recorder's session
(`RecSession` in `Mode::LiveSteamVr`, `core/io_session.*` with `IoVariant::SteamVr`):

1. PiBridge's Basalt config for the settings (as above); with `io_slam_engine=pimax` instead: once the camera broker's
   socket is up, `wine winbridge.exe` (Pimax's engine, [WINBRIDGE.md](WINBRIDGE.md)) starts before SteamVR / Monado,
   and Stop ends it by PID after them. If it ends on its own, the session goes on with the head in 3DoF.
2. The driver settings file `$XDG_CONFIG_HOME/pibridge/steamvr-driver.env` (`PIMAX_STEAMVR_ENV_FILE`; driver_monado.so
   reads it inside vrserver, which does not get PiBridge's environment; omarchyvr's file is not touched): the same
   `KEY=VALUE` lines monado-service would get (the "driver" list of `buildIoEnv`).
3. Shims for `xdg-open`, `steam`, `gio`, `gtk-launch` in `$XDG_STATE_HOME/pibridge/steamvr-shim/`, first in SteamVR's
   PATH: whatever SteamVR opens starts without SteamVR's variables (omarchyvr incident 2026-10-16: a Steam started
   through vrserver's `xdg-open steam://...` inherited `SteamAppId=250820` and froze).
4. SteamVR's drivers switched for the session (below).
5. The camera broker (socket wait 10 s), `pimax-ctrltrack live --factory` (when optical tracking is on), then
   `<SteamVR runtime>/bin/vrstartup.sh` (`runtime[0]` of `~/.config/openvr/openvrpaths.vrpath`) with
   `VALVE_SKIP_RUNTIME_SAFETY=1` (vrserver on the host: in Steam's sniper container driver_monado.so cannot load its
   host libraries), `QT_QPA_PLATFORM=xcb`, `LD_PRELOAD=<the same build>/steamvr-monado/bin/linux64/pimax_rttime_preload.so`
   (vrserver's RLIMIT_RTTIME would get vrcompositor killed), `PIMAX_STEAMVR_ENV_FILE` and the shims' PATH; output in
   `<session>/logs/steamvr.log`. Just before the launch the clock-tick time after boot is kept as the start mark.

It runs once vrserver runs and the app's SteamVR link reports the headset (an HMD device); 90 s at most, then it
fails in plain words. vrstartup.sh ending by itself after the hand-over is no failure; an error before SteamVR came up
is. Games start through SteamVR as usual. In this mode the Apps page never starts SteamVR through Steam
(`steam://rungameid/250820` would load CustomHeadsetOpenVR): Play starts START TRACKING and waits for "SteamVR fully
ready"; the Dashboard's Launch SteamVR does the same.

It starts only when: the mode is Inside-out; SteamVR is closed and Steam's client runs (a SteamVR started without
Steam starts Steam itself with SteamVR's settings); SteamVR is installed; the Monado build has `steamvr-monado/` and the
rttime preload and all required markers (Development page); no recording and no "Monado directly" runs; the headset,
its cameras and the programs are found (no display connector needed); no restore of an earlier session is pending.

**Stop**: SteamVR first, PID-exactly: `scanSteamVr` (core/steamvr_reset: processes by where their program lives,
never Steam's client) filtered to the ones started at or after the start mark, each checked by its start time before a
signal; SIGTERM, SIGKILL after 5 s ("Closing SteamVR..."). Then the session's own programs in reverse order (the
launcher if still there, the tracker, the broker), then the drivers are put back. **The user quits SteamVR**: what it
left of this start (vrmonitor, vrcompositor) is closed the same way, then the rest ("Stopped: SteamVR was closed.").
**The session ends by itself** (broker died, SteamVR never came up): SteamVR of this start is closed too.
**Closing PiBridge**: the same, blocking (about 7.5 s at most for SteamVR, then the recorder's waits).

### The headset service and the eye tracker step aside (both sessions)

Monado's Pimax driver (in vrserver, or monado-service) owns the headset's HID and the eye chip (PIMAX_EYE) during an
inside-out session, so pibridge-keepalived (it follows vrserver) and pibridge-eyed must not talk to it. Every inside-out
start (START TRACKING and "Monado directly") first writes `$XDG_RUNTIME_DIR/pibridge/inside-out.active` (owner = the app's
pid, the variant; `common/pibridge_pause.h`), before the driver switch and the broker. The daemons check it twice a
second: they close the device (keepalived sends its idle report first when busy and gives the audio back; eyed
unsubscribes and writes nothing to the gaze file), report `state: "paused"` ("paused: inside-out tracking") and take
the headset again when it is gone. The start waits until both say "paused" (or do not run), at most 5 s, then fails
in plain words and removes the marker. Every end removes it (Stop, SteamVR closed, a failure, PiBridge closing; the 2 s
restore check and any refresh also remove a stale one of a crashed PiBridge). No systemctl is used. The Dashboard and
the status bar say "Paused: inside-out tracking". Tests: `pibridge_keepalived` / `pibridge_eyed_test_eyed` (pause, release,
resume, the stale marker), `pibridge_steamvr_monado_tests` (the wait, its timeout, the marker on every path).

The first scan of a monado-service binary for its markers runs on a worker thread (cached by path, size and mtime);
the Development card says "Checking the build (markers)..." meanwhile and the starts wait for it.

### SteamVR's drivers for the session (`core/steamvr_drivers.*`)

What a start changes (the Session card lists it before a start, read only):
- `openvrpaths.vrpath`, `external_drivers`: `<the chosen Monado build>/steamvr-monado` appended (unless there); a
  steamvr-monado dir of another build (a second driver named `monado` by its manifest) taken out for the session and
  put back at its place afterwards.
- `<SteamVR config>/steamvr.vrsettings` (`config[0]` of the vrpath file): `"driver_CustomHeadsetOpenVR": {"enable":
  false}` (SteamVR's per-driver switch; only when CHOVR is registered), and `"driver_monado": {"enable": true}` only
  when it said false.

How: never while vrserver runs (refused with the reason). First the record `$XDG_STATE_HOME/pibridge/steamvr-drivers.json`
(the previous value of each key or that it was absent, whether section and file existed, the entries added and taken
out with their places), then copies `<file>.bak-<yyyyMMdd-HHmmss>` of both files, then the files, each atomically.
Only PiBridge's keys are written; order, other keys and number texts stay (libpibridge-room's Json). **Restore** puts back
exactly the recorded state of those keys (keys SteamVR added meanwhile stay), removes a section or file PiBridge made
when it is empty again, and deletes the record; a second restore does nothing. It runs after the session and whenever
the record exists and vrserver is not running (every 2 s while waiting, and at PiBridge's next start after a crash).
Nothing to change: nothing is written or recorded.

## Apps while "Monado directly" runs (Development page)

While the Development page's "Monado directly" runs, monado-service is the OpenXR runtime (`core/apps_runtime.h`):

- **Own apps** start with `XR_RUNTIME_JSON=<openxr_monado-dev.json of that Monado build>` in their own environment
  only (next to monado-service or up to four folders above, else `<prefix>/share/openxr/1/openxr_monado.json`). The
  system's active runtime stays SteamVR. An app starts once the session runs (Monado up 5 s), its IPC socket
  `$XDG_RUNTIME_DIR/monado_comp_ipc` exists, and the home scene has its session; 60 s at most.
- **Steam games** are off ("Needs SteamVR"): they need an OpenVR-to-OpenXR layer, which is not installed.
- The toolbar chip says TRACKING READY / STARTING TRACKING / TRACKING STOPPED.

When it does not run, everything is the SteamVR path (Lighthouse, or SteamVR + Monado in the inside-out mode).

## The room map at the start (both sessions)

The floor would be at head height (Basalt's origin is the start pose). With a saved room (Room page, **MAP THE ROOM**;
[ROOM-SETUP.md](ROOM-SETUP.md) "Room map (inside-out)") both START TRACKING (SteamVR + Monado) and "Monado directly"
start `pibridge-roommap auto --name <io_room> --wait 120` detached once the runtime is up (Monado's socket, or SteamVR
reporting the headset): it waits for Monado's head ring, relocalises in 15 s windows for up to 120 s and writes floor +
centre + forward through the writer lease; log `<session>/logs/roommap.log`. It is started once per session, and Stop
sends it SIGTERM first (that PID only, SIGKILL after 3 s). `PIMAX_ANCHOR=1` is passed whenever `pibridge-roommap` is
found, so the hook is there for MAP THE ROOM in the running session. Without a saved room nothing is started. The
session line says **Room: Room recognised / Searching for the room / Room not found** from
`$XDG_RUNTIME_DIR/omarchyvr-room.status` (only this session's). The driver's manual recenter (a 1.5 s press of the
menu / Pimax button) is applied on top of the room offset; recenter after "Room recognised" (ROOM-SETUP.md, "The order
of the room offset and the recenter").

## The home scene (`io_home_scene`, default on)

Without an app the headset would show nothing while "Monado directly" runs. Once that session runs, it starts
**pibridge-home** ([home.md](home.md): a dark room with a floor grid, the horizon, the active saved room's boundary, the
controllers and a short text panel) with the same `XR_RUNTIME_JSON`; its output goes to `<session>/logs/home.log`. It is
started once per session and never again (a home that came back while a game runs would take the headset over); on
Stop it gets SIGTERM first (that PID only, SIGKILL after 3 s), then Monado, the tracker and the broker as before.
Monado shows the newest session that began: apps wait until the home's status file (`$XDG_RUNTIME_DIR/pibridge/home.json`)
says it has its session, or 8 s passed, so a game always begins after it and is what the headset shows; when the game
ends, Monado shows the home again. The switch is on the Development page's "Monado directly" card (with the home's
state under it); it applies at the next start. pibridge-home is looked up next to the pibridge binary, in the build tree
(`<build>/tools/home/`), then in PATH.

## The Dashboard while "Monado directly" runs

The headset card: pibridge-keepalived follows vrserver, so it only waits while monado-service owns the headset; the card
then shows Monado's state instead ("Monado: starting / running / stopping / failed") and says that "Monado directly"
(Development page) drives the headset (`monadoHeadsetWords`, `core/io_session.h`).

The Controllers card reads Monado instead of SteamVR (`core/monado_devices.h`):
libmonado (the `mnd_*` API of monado-ctl, a plain IPC client without an OpenXR session) from the session's Monado build
(the runtime file's `MND_libmonado_path`, else `<build>/src/xrt/targets/libmonado/libmonado.so`), loaded with dlopen,
so PiBridge builds and runs without it. A worker thread polls once a second while the session runs and Monado's socket
exists; a lost connection is dropped and tried again after 3 s. The card's chip says MONADO: n/2 ON (MONADO OFF, NO
LIBMONADO, CONNECTING), each controller ON with its battery, OFF ("Turn it on": the Pimax driver reports a battery only
once the controller has sent a report) or LISTED (no battery reported), and the empty state says "Turn the controllers
on." or "Start tracking (Tracking page) to see the controllers."; without libmonado it says so. Monado does not report
whether a controller is tracked, so the card cannot show that. The toolbar's runtime label says Monado.

## Fake mode

`pibridge --fake` / `--screenshots`: the setup is made up, a synthetic base Basalt config and factory file (made-up
numbers) are written into the temporary state dir so the page can show PiBridge's values, both sessions run
FakeRecProcesses (nothing is started; the home scene "runs" there too; SteamVR "comes up" 1.5 s after its launch and
"closes" at Stop), SteamVR's files are never touched (the driver switch only says what it would do) and the
screenshots move a fake clock. The
Controllers card reads a fake libmonado source (left controller on at 64 %, right one off).

## Tests

`app/tests/test_tracking.cpp` (`pibridge_tracking_tests`): defaults == recommendations; the pibridge.conf round trip and
clamping; the environment for the recommended settings and for every switch (SLAM, Basalt or the tracker missing,
camera AE on/dry/off, the SteamVR variant and its driver file); the preload lookup and the connector list over a fake
`/sys`; the factory and cache parsers; PiBridge's Basalt config from synthetic base files (grid, biases, fallbacks,
zeros for scale and misalignment, every other byte kept, the base never written, dry run, broken bases); the
recorder's copy of the tracker's calibration; START TRACKING's readiness, start order, no `PIMAX_SLAM_RECORD`, the
PID-exact reverse stop, without controllers, the lease retry and a dying broker; the home scene (started once Monado
runs with the runtime file, stopped first, never twice in a session, off / not found / no runtime file, SIGKILL when
it ignores SIGTERM, the status file, the program lookup) and the Apps page's runtime over the session, all with
FakeRecProcesses.

`app/tests/test_apps.cpp` (`pibridge_apps_tests`): Monado's readiness steps, the runtime file and libmonado lookup, own
apps on Monado with `XR_RUNTIME_JSON` and never SteamVR, the waits, errors, Cancel, timeout, Steam games refused, and
in the inside-out mode SteamVR started through START TRACKING, never through Steam
(`inside_out_starts_steamvr_through_tracking_never_through_steam`). `app/tests/test_monado.cpp`: the libmonado link.

`app/tests/test_steamvr_monado.cpp` (`pibridge_steamvr_monado_tests`): the driver switch on made-up SteamVR files
(appended dir, CHOVR off, backups, record; byte-for-byte idempotent restore; refused while vrserver runs; a crash and
the next start; SteamVR's own changes kept; previous values, a missing settings file, nothing to do, errors), the
shims, readiness words, start order and environment, the PID-exact close of only this start's SteamVR processes before
the tracker and the broker, SteamVR closed by the user, SIGKILL after 5 s, the failures, PiBridge closing, the restore at
the next start, the headset card words, and the Monado build (markers, kind, choice, driver dir, cap_sys_nice, Fix
permissions with a FakeRunner). `test_tracking.cpp` checks the migration of `io_apps=monado`
(`tracking_runtime_follows_the_sessions`).

## Not verified live

- No inside-out session was started from PiBridge (agents never touch the headset, SteamVR or its files). Both variants
  and their stops are tested with fakes only; the driver switch only on made-up files.
- That SteamVR with `driver_CustomHeadsetOpenVR.enable=false` and the build's steamvr-monado registered picks
  driver_monado as the headset (and the lighthouse driver claims nothing) is expected, not seen yet. Also open:
  whether vrstartup ends after the hand-over, how long SteamVR takes with driver_monado (90 s limit), and
  pibridge-keepalived (it follows vrserver) next to driver_monado's own Pimax driver in vrserver.
- That Basalt loads PiBridge's generated toml/config/calib is not checked against a real Basalt run.
- The Pimax engine (`io_slam_engine=pimax`) ran only offline on a recording (WINBRIDGE.md); not yet inside Monado
  with the live headset, nor with the driver's 6DoF supervisor on its poses.
- Apps on Monado, the home scene with a real game and libmonado are tested only with fakes.
