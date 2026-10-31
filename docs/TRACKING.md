# Tracking: the tracking mode and inside-out head tracking (experimental)

The **Tracking** page holds the tracking mode and the inside-out settings; START VR and STOP VR are on the Dashboard.

PiBridge knows two ways to track the Crystal Super:

| Mode | What tracks | What it needs | State |
|---|---|---|---|
| **Lighthouse** (default) | SteamVR's lighthouse driver: head and Index controllers from the base stations | the Lighthouse faceplate, base stations, CustomHeadsetOpenVR (docs/INSTALL.md) | works |
| **Inside-out** | `pibridge-headtrack` ([pibridge-headtrack.md](pibridge-headtrack.md)): the Pimax engine or Basalt (`io_slam_engine`) on the headset's tracking cameras plus the IMU, the controllers by their LEDs; CustomHeadsetOpenVR shows the poses in SteamVR. Games start through SteamVR as usual | the camera broker (`pibridge-cam`), the headset's factory calibration (`rec_cam_calib`), winbridge for the Pimax engine or Basalt with a base config, CustomHeadsetOpenVR with `chovr-linux-headtrack.patch`, SteamVR and Steam | experimental |

Everything is on the **Tracking** page (left column); nothing has to be typed. The settings are saved in
`~/.config/pibridge/pibridge.conf` and used by **START VR** in the inside-out mode (Dashboard, and Play on the Apps
page). The environment comes from one builder, `buildIoEnv()` in `app/src/core/inside_out.*`; `pibridge-headtrack`
reads the same `PIMAX_*` switches the Pimax driver read, so the settings below keep their meaning. Changes apply at the
next start; a running session keeps the settings it started with.

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
`docs/libpibridge-cam.md`). The frame goes on in the usual layout, so START VR works the same way — but what
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
(head tracking, PiBridge's Basalt values, the start environment), `09c-tracking-inside-out-ready.png`,
`09d-tracking-running.png`, `09e-dashboard-tracking-running.png`, `09f-tracking-steamvr-some-off.png` (reasons shown),
`06c-apps-inside-out-steamvr-off.png` ("Play starts tracking"), `08c-room-inside-out.png`.

## Settings, their sources and defaults

`pibridge.conf` key, values (**default**), what `pibridge-headtrack` gets. "Off only" = the tracker's own default is
on, so only the off value is passed (as omarchyvr's launcher passes only the flag of a switch that is off).

| Group | Setting | Key | Values | Environment / arguments | Source |
|---|---|---|---|---|---|
| Mode | Head tracking by | `tracking_mode` | **lighthouse**, inside-out | (which setup the Dashboard offers) | |
| Session | Refresh rate | `refresh_hz` (shared) | 72, **90** | `--fps`, `PIMAX_FPS` | `pimax_hmd.c:81` (driver default 72), `crystal-6dof.sh:23` (90) |
| Head | 6DoF head tracking | `io_slam` | **on**, off | needed for inside-out (START VR says so when it is off) | `pimax_camera.c:158`, `t_tracker_slam.cpp:87-88` |
| Head | Head tracking engine | `io_slam_engine` | basalt, **pimax** | basalt: `--engine basalt --vit <libbasalt.so> --slam-config <PiBridge's toml>`; pimax: `--engine pimax --vit libpibridge_vit_winbridge.so --cams 0,1,2,3`, `PIBRIDGE_VIT_POSE_RATE=imu`, controllers by Pimax's LED engine (no pibridge-ctrltrack); the session runs `wine winbridge.exe` ([WINBRIDGE.md](WINBRIDGE.md)) | Pimax Play's `pimax_slam.pi.dll`, `LedObjectPoseEstimator.dll` |
| Head | Head tracking cameras | `io_slam_cams` | **2** (lower pair 0, 1), 4 (also the upper side pair 2, 3) | Basalt: `--cams 0,1,2,3` and a four-camera calibration in PiBridge's Basalt config (below), else `--cams 0,1`. 2 until a live take decides | `pimax_camera.c` `PIMAX_SLAM_CAMS`; ROADMAP 1b |
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
| Controllers | Optical tracking | `io_ctrl_track` | **on**, off | `PIMAX_CTRL_TRACK=1/0`; with the Basalt engine `pibridge-ctrltrack live` is started | `pimax_ctrl.c:82`, `crystal-6dof.sh:17` |
| Controllers | Controller angle | `io_ctrl_angle` | -20..70, **30** deg | `PIMAX_CTRL_ANGLE` (always passed) | `pimax_ctrl.c:84`, omarchyvr `ctrl_angle.rs:10-12` |
| Controllers | (no switch) | — | always (driver default) | `PIMAX_CTRL_INHAND` not passed: `auto` marked held controllers as put down and the tracker stopped looking for them (live 2026-10-19) | `pimax_ctrl.c` |
| Controllers | Out-of-view hold | `io_ctrl_lost_s` | 1..10, **4** s | `PIMAX_CTRL_TRACK_LOST_S` when not 4 | `fuse.rs:758`, `:691` |
| Controllers | Steadier controllers | `io_ctrl_steady` | **on**, off | off only: `PIMAX_CTRL_TRACK_HYST=0` | `fuse.rs:743`; omarchyvr `trackset.rs` "Steadier controllers" |
| Controllers | Hands from the controllers | `io_ctrl_hands` | on, **off** | `PIMAX_CTRL_HANDS=1` / `=0` (Monado) | `pimax_ctrl.c`: XR_EXT_hand_tracking from the controllers; off so games that enable hand tracking (Falcon BMS) show the controllers |
| Advanced | Extra environment | `rec_env` (shared) | `KEY=value` words | appended last (overrides) to pibridge-headtrack's environment | |
| Advanced | Programs and files | see "Programs and files" below | "Choose..." / "Auto" | | |

## Programs and files

All optional. Without a key the app looks in the places listed, first match wins (`~` is expanded); `<data>` is
PiBridge's installed data folder. A key that names a file which does not exist is not replaced by a lookup: the page
says the file is missing.

| Key | What | Default lookup |
|---|---|---|
| `rec_pimax_cam` | pibridge-cam (the camera broker) | PATH, `<prefix>/bin/`, the build tree next to the app (`tools/cam/`) |
| `rec_pimax_ctrltrack` | pibridge-ctrltrack | the same (`tools/ctrltrack/`) |
| `rec_pibridge_headtrack` | pibridge-headtrack | the same (`tools/headtrack/`) |
| `rec_basalt_lib` | libbasalt.so | `<prefix>/lib/pibridge/`, `~/gamedev/ovr-build/basalt/lib/`, `/usr/local/lib/`, `/usr/lib/` |
| `rec_winbridge`, `rec_vit_winbridge` | winbridge.exe and libpibridge_vit_winbridge.so (the Pimax engine) | `<prefix>/lib/pibridge/winbridge/`, `~/.local/share/pibridge/winbridge/`, the source tree's `winbridge/` |
| `rec_slam_config` | base Basalt config | `~/.config/pibridge/pimax-basalt.toml`, `<data>/slam/pimax-basalt.toml`, `~/gamedev/ext/slam/pimax-basalt.toml` |
| `rec_lens_conf` | lens file | `~/.config/pibridge/lens.conf`, then the profile chosen on the Display page (`<data>/lens/`) |
| `rec_cam_calib` | camera calibration (factory XML) | `~/.config/pibridge/headset-calibration.xml` (the headset's own), `<data>/calib/crystal-calibration.xml` |
| `rec_cam_device` | camera device | the `/dev/videoN` whose `/sys/class/video4linux/videoN/name` starts with "PimaxSensor" and index is 0 |

Paths and line numbers in the Source column: the Pimax driver of omarchyvr's Monado fork
(`forks/monado/src/xrt/drivers/pimax/`, where these switches come from), Rust = omarchyvr `rust/pimax-cam/src/`,
`rust/pimax-hands/src/`, `crystal-6dof.sh` = `~/gamedev/ext/tools/crystal-6dof.sh`, research doc =
`docs/research/slam-imu-intrinsics.md`.

Fixed, always passed (from `crystal-6dof.sh` lines 22-29): `PIMAX_LOG=info`, `PIMAX_BIAS_TIMEOUT_MS=200`,
`PIMAX_LENS_CONF`, `PIMAX_CAM_CALIB`, `PIMAX_CAM_SOURCE=broker`; pibridge-headtrack gets `--camera broker:<socket>` and
`--calib <rec_cam_calib>`.

Read but deliberately not offered (test aids or A/B switches in the sources): `PIMAX_SLAM_CAM_OFFSET_US`,
`PIMAX_SLAM_CAMS`, `PIMAX_SLAM_STILL_REZERO`, `PIMAX_SLAM_FUSE`, `PIMAX_SLAM_TAU_MS`, `PIMAX_SLAM_TAU_REST_MS`,
`PIMAX_SLAM_FREEZE`, `PIMAX_SLAM_RESNAP`, `PIMAX_CAM_AE_TARGET` and the other `PIMAX_CAM_AE_*` tuning, the
`PIMAX_CTRL_TRACK_*` fuser tuning, `PIMAX_CTRL_INHAND`, the hand-tracking `PIMAX_HT_*`, `PIMAX_HANDS`. Any of them
can still be set under Advanced > Extra environment.

## The head output's own defaults (libpibridge-cam)

Independent of the page's settings, the head output layer (`headpose::HeadTracker`, used with Basalt) has measured improvements over the
Rust one (docs/research/head-tracking-improvements.md): drift hold off unless `PIMAX_SLAM_DRIFT_HOLD=1` (the page's
switch), the output's gyro bias seeded from the gyro bias cache, the tracker's accel bias removed from the output's IMU
samples, and the rotation predicted with the angular acceleration. `PIBRIDGE_HEAD_PARITY=1` under Advanced > Extra environment gives the exact omarchyvr behaviour for an A/B; the
single switches are `PIMAX_SLAM_SEED_BIAS`, `PIMAX_SLAM_ACCEL_BIAS`, `PIMAX_SLAM_PREDICT_ACCEL`,
`PIMAX_SLAM_STILL_BACKOFF` (off: measured worse), and `PIMAX_SLAM_DRIFT_HOLD=2` (an
experimental hold only while the head is quiet).

Not passed yet, recommended once a second take confirms it: `PIMAX_SLAM_CAM_OFFSET_US=-1500` with auto exposure on
(the camera frame times are about 1.5 ms late against the IMU; on the 07:04 take it lowered the mark error after the
turns / nods / shake from 52 to 29 mm and the output's motion error while turning by 16 %). The takes still needed are
listed at the end of that study.

## PiBridge's Basalt config

Values Basalt reads from its own files cannot be passed as environment. At every START VR with the Basalt engine
PiBridge writes its own copy into `$XDG_CONFIG_HOME/pibridge/slam/`:

- `pimax-basalt.toml`: the base config's lines (the one `rec_slam_config` names, usually `pimax-basalt.toml` from
  `omarchyvr-slam export`; read only, never changed) with `cam-calib` and `config-path` pointing at the two files below;
- `pimax-basalt-config.json`: the base settings, byte for byte, except `config.optical_flow_detection_grid_size`;
- `pimax-basalt-calib.json`: the base calibration, byte for byte, except `calib_accel_bias` = the factory `aBias`
  of `rec_cam_calib` (`<SFConfig><Stateinit aBias>`, used as is) followed by six zeros (the factory scale `ka` and
  misalignment `na` are never applied), and `calib_gyro_bias` = the last gyro bias measured at rest
  (`~/.cache/omarchyvr/gyro-bias`, when present; raw axes deg/s turned into the body frame: x and z negated, rad/s), else the
  factory `wBias` as is, else the base value, followed by nine zeros.

**Four head tracking cameras** (`io_slam_cams=4`): before the copy, PiBridge runs `pibridge-slam export --cams 0,1,2,3
--calib <rec_cam_calib> --base <the base's config-path>` into `slam/cams4/` and takes only its camera arrays
(`T_imu_cam`, `intrinsics`, `resolution`) into the copy; the base's own cameras must be the first two of them
(the same headset), so IMU noise, biases and everything else stay the two-camera values and a comparison 4 vs 2
changes only the cameras. `config.optical_flow_detection_nonoverlap` is set to true in the settings (the upper pair
gets corners of its own; [pibridge-slam.md](pibridge-slam.md)). The tracker gets all four cameras only when that
worked; otherwise the lower pair is used and the page and the start say why (pibridge-slam not found, export failed,
another headset's cameras). The camera count must match: Basalt aborts when its calibration has another count than the cameras it is fed.

The frame time of a four-camera set: Basalt wants one timestamp per set, and the upper pair has its own auto
exposure (`ae.h`: pairs 0,1 and 2,3). The tracker takes the mean mid-exposure of the tracker cameras (equal to the
pair's own with the lower pair alone, so two cameras are unchanged); each pair is then off by a quarter of the two
pairs' exposure difference (at most (10410 - 2200) / 4 us = 2 ms, usually much less).

`--slam-config` then points at PiBridge's toml. If the base is missing or broken, the start goes on with the base as it
is (the grid cannot be set then), and the page says why. The page shows the values PiBridge would write (a dry run;
nothing is written while you look).

## START VR (inside-out)

On the Dashboard (inside-out mode), and through Play on the Apps page (a game or an own app while SteamVR is off). It is
`RecSession` in `Mode::LiveSteamVr` (`core/io_session.*`, `IoVariant::SteamVr`):

1. PiBridge's Basalt config for the settings (as above, Basalt engine only).
2. Shims for `xdg-open`, `steam`, `gio`, `gtk-launch` in `$XDG_STATE_HOME/pibridge/steamvr-shim/`, first in SteamVR's
   PATH: whatever SteamVR opens starts without SteamVR's variables (omarchyvr incident 2026-10-16: a Steam started
   through vrserver's `xdg-open steam://...` inherited `SteamAppId=250820` and froze).
3. The camera broker (socket wait 10 s); with `io_slam_engine=pimax` `wine winbridge.exe` (Pimax's engine,
   [WINBRIDGE.md](WINBRIDGE.md)); with Basalt and optical controller tracking on `pibridge-ctrltrack live --factory`;
   then `pibridge-headtrack` ([pibridge-headtrack.md](pibridge-headtrack.md)); then
   `<SteamVR runtime>/bin/vrstartup.sh` (`runtime[0]` of `~/.config/openvr/openvrpaths.vrpath`) with the shims' PATH;
   output in `<session>/logs/`. Just before the launch the clock-tick time after boot is kept as the start mark.

SteamVR's drivers and files are not changed: CustomHeadsetOpenVR (with `chovr-linux-headtrack.patch`, docs/INSTALL.md
5.3) reads the poses from `/dev/shm/pibridge-crystal-pose`.

It runs once vrserver runs and the app's SteamVR link reports the headset (an HMD device); 90 s at most, then it
fails in plain words. vrstartup.sh ending by itself after the hand-over is no failure; an error before SteamVR came up
is. Games start through SteamVR as usual. In this mode the Apps page never starts SteamVR through Steam: Play starts
START VR and waits for "SteamVR fully ready". If winbridge ends on its own, the session goes on with the head in 3DoF;
if pibridge-headtrack ends, the session stops.

It starts only when: the mode is Inside-out; 6DoF head tracking is on; SteamVR is closed and Steam's client runs (a
SteamVR started without Steam starts Steam itself with SteamVR's settings); SteamVR is installed; the headset, its
cameras and the programs are found.

**Stop**: SteamVR first, PID-exactly: `scanSteamVr` (core/steamvr_reset: processes by where their program lives,
never Steam's client) filtered to the ones started at or after the start mark, each checked by its start time before a
signal; SIGTERM, SIGKILL after 5 s ("Closing SteamVR..."). Then the session's own programs in reverse order.
**The user quits SteamVR**: what it left of this start (vrmonitor, vrcompositor) is closed the same way, then the rest
("Stopped: SteamVR was closed."). **The session ends by itself** (broker died, SteamVR never came up): SteamVR of this
start is closed too. **Closing PiBridge**: the same, blocking.

### The headset service and the eye tracker step aside

`pibridge-headtrack` owns the headset's HID during an inside-out session, so pibridge-keepalived (it follows vrserver)
and pibridge-eyed must not talk to it. Every inside-out start first writes `$XDG_RUNTIME_DIR/pibridge/inside-out.active`
(owner = the app's pid, the variant; `common/pibridge_pause.h`), before the broker. The daemons check it twice a
second: they close the device (keepalived sends its idle report first when busy and gives the audio back; eyed
unsubscribes and writes nothing to the gaze file), report `state: "paused"` ("paused: inside-out tracking") and take
the headset again when it is gone. The start waits until both say "paused" (or do not run), at most 5 s, then fails
in plain words and removes the marker. Every end removes it (Stop, SteamVR closed, a failure, PiBridge closing; the 2 s
check and any refresh also remove a stale one of a crashed PiBridge). No systemctl is used. The Dashboard and the
status bar say "Paused: inside-out tracking". Tests: `pibridge_keepalived` / `pibridge_eyed_test_eyed` (pause, release,
resume, the stale marker), `pibridge_steamvr_monado_tests` (the wait, its timeout, the marker on every path).

## The room map at the start

The floor would be at head height (the tracker's origin is the start pose). With a saved room (Room page, **MAP THE
ROOM**; [ROOM-SETUP.md](ROOM-SETUP.md) "Room map (inside-out)") START VR starts `pibridge-roommap auto --name <io_room>
--wait 120` detached once SteamVR reports the headset: it waits for the head ring (`pimax-anchor.head`, written by
pibridge-headtrack), relocalises in 15 s windows for up to 120 s and writes floor + centre + forward through the writer
lease; log `<session>/logs/roommap.log`. It is started once per session, and Stop sends it SIGTERM first (that PID
only, SIGKILL after 3 s). Without a saved room nothing is started. The session line says **Room: Room recognised /
Searching for the room / Room not found** from `$XDG_RUNTIME_DIR/omarchyvr-room.status` (only this session's). A long
press (0.8 s) of the Pimax button recenters on top of the room offset; recenter after "Room recognised" (ROOM-SETUP.md,
"The order of the room offset and the recenter").

On the Monado backend the same long press (or Recenter on a dashboard in the headset) recenters in PiBridge's Monado:
the driver turns the tracking origin so forward is where you look, then the space overseer moves the LOCAL spaces
onto the head as well, each app's own included, so seated games that use LOCAL face forward too
([monado.md](monado.md)). The right controller's Menu button is Touch's `/input/system/click` there (Touch has no
right `menu/click`). Under SteamVR, `pibridge-headtrack` also sends the right Menu button as the left controller's
system click, since SteamVR's Touch bindings open the dashboard from the left one only.

With Lighthouse on the Monado backend and Valve Index controllers (they have one menu-like button each, the system
button): the **left** one is the menu (the apps get it), the **right** one is the recenter button: it never reaches the
apps, held 0.8 s it recenters with the same ring. The recenter moves the Index controllers, trackers and base stations
with the head (steamvr_lh's hooks, [monado.md](monado.md)).

## Fake mode

`pibridge --fake` / `--screenshots`: the setup is made up, a synthetic base Basalt config and factory file (made-up
numbers) are written into the temporary state dir so the page can show PiBridge's values, the session runs
FakeRecProcesses (nothing is started; SteamVR "comes up" 1.5 s after its launch and "closes" at Stop), and the
screenshots move a fake clock.

## Tests

`app/tests/test_tracking.cpp` (`pibridge_tracking_tests`): defaults == recommendations; the pibridge.conf round trip and
clamping; the environment for the recommended settings and for every switch; the factory and cache parsers; PiBridge's
Basalt config from synthetic base files (grid, biases, fallbacks, zeros for scale and misalignment, every other byte
kept, the base never written, dry run, broken bases); START VR's readiness, start order and the PID-exact reverse
stop, all with FakeRecProcesses.

`app/tests/test_apps.cpp` (`pibridge_apps_tests`): in the inside-out mode SteamVR is started through START VR, never
through Steam (`inside_out_starts_steamvr_through_tracking_never_through_steam`).

`app/tests/test_steamvr_monado.cpp` (`pibridge_steamvr_monado_tests`, the name is historical): the shims, readiness
words, start order and environment, the PID-exact close of only this start's SteamVR processes before the session's own
programs, SteamVR closed by the user, SIGKILL after 5 s, the failures, PiBridge closing.

## Not verified live

- That Basalt loads PiBridge's generated toml/config/calib is not checked against a real Basalt run inside
  pibridge-headtrack.
- The controllers with the Basalt engine (pibridge-ctrltrack's poses fused with their IMU) are not checked live in the
  headset yet.
