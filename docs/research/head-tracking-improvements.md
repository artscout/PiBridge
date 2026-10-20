# Head tracking improvements over the Rust port (Crystal Super, offline study)

Date: 2026-10-19. Builds on [slam-imu-intrinsics.md](slam-imu-intrinsics.md) (studies 1-3). The head output layer
(`libpibridge-cam` headpose, slam supervisor, drift governor; the `pimax_cam_track_*` and
`pimax_cam_slam_write_basalt` entry points Monado calls) was byte-exact with omarchyvr's Rust at `5ffbcc5`. This
study changes it where the numbers say so. The exact Rust behaviour stays reachable: **`PIBRIDGE_HEAD_PARITY=1`** turns
every change below off (the difftests against the Rust oracle and the Rust test twins run that way and stay green).
Scratch data, scripts and every run: `~/gamedev/ext/slam/study4/` (`hr.sh`, `vl.sh`, `mk4.py`, `queue4.sh`,
`vitloop/`, `runs/*/README`, `table4*.md`).

## Short answer

What changed in PiBridge (all on by default, all off with `PIBRIDGE_HEAD_PARITY=1`, each with its own switch):

| Change | Switch | Worn take 07:04, before (Rust) -> after | Notes |
|---|---|---|---|
| Drift hold off | `PIMAX_SLAM_DRIFT_HOLD=0` default (1 = Rust hold, 2 = quiet-only experiment) | mark error 2-9 / 2-15 / 9-15: **333 / 264 / 74 -> 63 / 70 / 68 mm**; drift 155 -> 41 mm/min | the app already passed `=0`; now the library default too |
| Accel bias removed in the output layer | `PIMAX_SLAM_ACCEL_BIAS` | steady output-tracker offset median **13.5 -> 6.6 mm** (p95 22.9 -> 15.4); tilt error vs gravity median 0.69 -> 0.32 deg | the bias Basalt subtracts (SLAM_CONFIG calib, else factory aBias) |
| Gyro bias seeded from the driver cache | `PIMAX_SLAM_SEED_BIAS` | rotation error first 10 s median **0.132 -> 0.049 deg** (p95 0.31 -> 0.23), 10-20 s 0.069 -> 0.047 | no 40 s learning; the still gate can engage from the start |
| Rotation prediction with angular acceleration | `PIMAX_SLAM_PREDICT_ACCEL=0.75` | display-time rotation error at 20 ms, head turning 30-100 deg/s p95 **0.83 -> 0.48 deg**, > 100 deg/s 0.88 -> 0.69 (max 1.41 -> 1.13); at 40 ms > 100 deg/s 2.63 -> 1.35 (max 4.7 -> 2.4) | rate of the last 4 ms + 0.75 x the angular acceleration over 24 ms |
| Recorder writes the calibration the tracker used | `PIMAX_SLAM_RECORD_ACCEL_BIAS` | take's `pimax-basalt-calib.json`: `calib_accel_bias` 0 -> the tracker's (copy of the SLAM_CONFIG calib) | a replay of a take is now the session |

Together, closed loop (Basalt in the loop, its own velocity, 07:04 take): mark 330 / 263 / 72 -> 63 / 71 / 71 mm,
dynamic position error p95 11.7 -> 8.1 mm, jitter at rest 0.080 -> 0.046 mm per frame, rotation error > 100 deg/s
p95 0.88 -> 0.69 deg, no losses or resets either way, 10.7 us of CPU per 90 Hz query (0.1 % of one core) either way.

Recommended outside the library (needs the lead / the app, not changed here):

- **Camera-IMU time offset `PIMAX_SLAM_CAM_OFFSET_US=-1500`** with auto exposure on (on top of the driver's
  mid-exposure shift). Measured two ways on the 07:04 take: image rotation against the gyro (`camlag`) -1.37 / -1.47
  ms (cam0 / cam1), and a Basalt sweep: mark error 9-15 (turns, nods, shake) 52 -> 29 mm (tracker) / 68 -> 43 mm
  (output), 2-15 66 -> 63 mm, dynamic error while turning p95 10.9 -> 9.2 mm, tilt 0.97 -> 0.84 deg. Confirm on a
  second take before it becomes the default (below).

Evaluated and **not** adopted (default stays the Rust behaviour, switch kept for A/B):

- Still-gate back-off (`PIMAX_SLAM_STILL_BACKOFF=4`, study 2's patch): closed loop on the desk take, 4 runs each:
  re-zeros 19-24 -> 7-11, but the fresh trackers then run away for longer (tracker path while still 150-600 ->
  830-3760 mm, up to 1.9 m in one still) and one run's output was 1.5 m off the tracker after the pickup; losses are
  the same (0-1 per run). No gain, a real risk.
- Basalt settings (open loop, `slambatch`): optical-flow recall, detection grid 25 / 20, 2 points per cell, 10 key
  frames, 4 pyramid levels: none better than the session's grid 30 config (table below).
- IMU intrinsics (factory `ka` / `na`, a six-pose scale fit): no change; study 3's numbers stand.
- A "quiet-only" drift hold (`PIMAX_SLAM_DRIFT_HOLD=2`): holds only while the head is quiet; mark 67 / 77 / 71 mm
  (vs 63 / 70 / 68 without any hold), creep at the marks 3.7 -> 1.5 mm/s. Not better; kept as an experiment for a
  live A/B.

## Method

Three harnesses, all offline, deterministic where they can be, at `nice -n 19`, guarded so nothing runs while
`monado-service` or `vrserver` runs:

1. **`pibridge-slam headreplay`** (new, `tools/slam/src/headreplay.*`, `docs/pibridge-slam.md`): the output layer exactly
   as Monado drives it (every IMU sample pushed, each tracker pose handed over 25 ms after its frame time, a Mahony
   3DoF with the driver's bias, queries at 90 Hz for `now + 20 ms`, the supervisor's resets emulated by rebasing the
   recorded trajectory, as monado-abi's difftest), on a recorded IMU stream + a recorded Basalt trajectory. The
   switches are read from the environment exactly as in Monado. 0.2 s per run.
2. **`vitloop`** (scratch, `study4/vitloop/`): the same output layer with **Basalt in the loop** through VIT
   (`libbasalt.so` of the live build, read only): images and IMU are pushed in time order, the poses carry Basalt's
   own velocity (live Monado uses it; the recorded replays only have positions), and a reset the supervisor asks for
   really resets Basalt. Without resets it reproduces the `slambatch` trajectory to 2.9 mm (the same as two
   `slambatch` runs of one config: 2.0 mm). 45-75 s per run. With resets (desk take) runs are not repeatable (the
   starved tracker is chaotic), so those numbers are 4 runs each.
3. **`monado-cli slambatch`** (deterministic=1) of the live Rust build `~/gamedev/ovr-build/monado` (read and run
   only) for Basalt-side settings and time shifts (`omarchyvr-slam shift`: camera times moved, images linked), scored
   with study 3's `metrics3.py`.

Metrics (no ground truth anywhere):

- **mark error**: distance between the mean output head positions of the protocol's mark steps (2 still on the mark,
  9 back on the mark, 15 back on the mark at the end); 9-15 brackets 40 s of turns, nods and a shake without leaving
  the mark. Tracker's own numbers in brackets.
- **prediction vs tracker**: the output at display time against the tracker looked at afterwards (positions by cubic
  Hermite between tracker poses, orientation by the gyro between them with the end mismatch spread), mapped into the
  output world with the alignment the output used at that moment; split by head turn rate. **Dynamic** position error
  = the same with its own centred 1 s mean removed (a slowly changing offset is invisible; lag, overshoot and jitter
  are not).
- **jitter**: RMS of the second difference of the 90 Hz output (mm, deg per frame), at rest (still steps) and moving.
- **creep at rest**: least-squares speed of the output in each still step; "moved" = start-to-end distance.
- **tilt error**: angle between the output's up and the accelerometer's (20 ms means, bias removed, |a| within 0.3 of
  g, gyro under 0.15 rad/s) in still and tilt-hold steps. Caveat: the gravity tilt correction pulls towards this same
  reference, so the metric favours it by construction; it shows that the correction's target is less biased with the
  accel bias removed, not that the output is closer to the true vertical.
- losses, acquisitions, resets, re-zeros, drift held, CPU per query.

Recordings: `20261019-070409` (worn, guided 15-step protocol, AE on: the main one; Basalt trajectory study 3 `L` =
the session's calibration and config), `euroc_20261019052737` (desk + six poses, AE off, dark: study 2; trajectory
`GdB` = factory aBias + grid 30, the current config). The gyro bias cache of each session is restored in a scratch
`XDG_CACHE_HOME` (07:04: raw 0.065 0.255 -1.341 deg/s; 05:27: 0.345 0.164 -1.330).

## 1. Gyro bias seed

The output layer learns its own gyro bias from tracker-vs-gyro rotation with a 10 s low-pass, starting at zero; the
Crystal's z bias is 1.3 deg/s, learned after about 40 s (live log). The driver already knows it (its 3DoF uses the
cached rest bias, Basalt gets it as `calib_gyro_bias`). Now `HeadTracker` starts from the same cache
(`slam::gyro_bias_cache`, read once at `track_create`) and keeps learning.

| 07:04, drift hold off, per 10 s window | 0-10 s | 10-20 s | 20-30 s | later |
|---|---|---|---|---|
| rotation error median / p95, deg, no seed | 0.132 / 0.31 | 0.069 / 0.29 | 0.047 / 0.22 | same |
| with the seed | **0.049 / 0.23** | **0.047 / 0.29** | 0.047 / 0.22 | same |

The desk take: the still gate engages earlier (frozen queries 7405 -> 8870 of 17257), because its "mean gyro near
the learned bias" test (after 250 poses) no longer waits for the learning. The still test itself uses only the
spread, so a stale cache (0.3 deg/s off) cannot block it.

## 2. Still-gate back-off (study 2's patch) - not adopted

Ported as `StillParams::backoff_max` (`PIMAX_SLAM_STILL_BACKOFF=N`: up to 2^N s between re-zeros while still; 0 =
Rust; after 3 re-zeros in a row the reason reads "re-zeroed: cameras see too little", which keeps the driver's
"re-zeroed" prefix). It cannot be judged on a recorded trajectory (a reset there only rebases the recorded drift), so
closed loop on the desk take (11 still stretches, a dark start):

| desk 05:27, closed loop, 4 runs | re-zeros | losses | output moved in the stills, sum mm | tracker path in the stills, sum mm (max one still) | position error vs tracker p95 / max mm |
|---|---|---|---|---|---|
| Rust (parity) | 19, 21, 19, 19 | 1, 1, 1, 1 | 40, 41, 39, 40 | 181, 688, 177, 180 (52-449) | 23-24 / 158-393 |
| PiBridge, no back-off (default; before the rotation prediction) | 19, 19, 19, 24 | 1, 1, 0, 1 | 29, 30, 29, 59 | 181, 147, 165, 595 (35-462) | 13-23 / 120-265 |
| PiBridge, back-off 1..16 s | 11, 7, 10, 11 | 1, 1, 1, 0 | 36, 27, 28, 46 | 2697, 826, 3764, 1639 (722-1926) | 10-23 / 164-1482 |

The output is held in every variant (the hold, not the reset, keeps it still). The back-off halves the resets, but a
starved fresh tracker does not settle by being left alone: it runs away for longer, and what it ran away with shows
when the headset is picked up. Keep a re-zero every second; fix the root (light, texture: AE on helps most).

## 3. The recorder's calibration

`pimax_cam_slam_write_basalt` gets the calibration through `pimax_cam_calib`, which has no raw `Stateinit`, so every
take said `calib_accel_bias = 0` while the live tracker used SLAM_CONFIG's calibration with the factory aBias
(study 3). Now (`PIMAX_SLAM_RECORD_ACCEL_BIAS`, default on): with `SLAM_CONFIG` set (Monado then takes the sensor
calibration from its `cam-calib` file) that file is copied into the take as `pimax-basalt-calib.json`, and
`pimax-slam-info.json` carries its biases and `"calib_source": "copy of the SLAM_CONFIG calibration <path>"`; without
one the factory aBias comes from the factory XML itself (`$PIMAX_CAM_CALIB` first; not from the shipped Dream Air
reference). The app's own copy at the end of a recording (docs/TRACKING.md "Recordings keep what the tracker used")
then copies the same content again; harmless, and takes made outside the app are right too.

## 4. Drift hold

The governor holds tracker translation the accelerometer cannot explain. On the Crystal it holds slow steps and leans
as drift: 07:04 mark error 333 / 264 / 74 mm with it (live it held 199 cm in 121 s), 63 / 70 / 68 without. It does
lower the creep in the still steps (1.2-1.6 against 3.2-3.7 mm/s at the marks), which is why it looked useful.

Why there is no correct hold at that level: the creep at rest is 3-5 mm/s (body sway included). The accelerometer
cannot see that: over a 0.5 s window the residual accel bias (0.03 m/s^2) and a 0.5 deg attitude error (0.085 m/s^2
of gravity) already give 4-11 mm, i.e. 10-20 mm/s of uncertainty, and a slow lean or step has hardly more
acceleration spread than standing (protocol steps, 1 s windows: still 1.0 deg/s / 0.07 m/s^2 median; the step away
min 1.2 deg/s / 0.10 m/s^2). The best candidate, a hold only while the head is quiet (gyro under 1.3 deg/s and accel
under 0.09 m/s^2 on every axis over 1 s, `PIMAX_SLAM_DRIFT_HOLD=2`), held 37 cm, cut the creep at two marks to 1.5-1.9
mm/s but left the marks a little worse (67 / 77 / 71 mm). The creep has to be fixed in the tracker (the time offset
below, a map / loop closure), not in the output. Default: off (the app's switch still turns the Rust hold on).

## 5. Camera-IMU time offset

Basalt ignores `cam_time_offset_ns`; the driver shifts frame times by `PIMAX_SLAM_CAM_OFFSET_US` plus half the
exposure (`PIMAX_SLAM_EXPOSURE_MID`, on). On the 07:04 take (AE on, exposures 5.2-6.7 ms, a 5 s head shake):

- `omarchyvr-slam camlag` (image rotation between frames against the integrated gyro): **-1.37 ms (cam0), -1.47 ms
  (cam1)**, residual 0.05 deg per frame pair, a clear minimum. The desk take (05:27, AE off, almost no motion) is too
  flat to tell (+1.05 ms, the residual within 1 % from -4 to +4 ms); study 1 (AE off, worn) said 0.0 with a flat minimum.
- Basalt sweep (camera times moved; L = 0):

| camera time | +1.5 | 0 | -1.0 | -1.4 | -2.2 | -3.0 | -4.0 | -5.5 ms |
|---|---|---|---|---|---|---|---|---|
| tracker mark 2-15 / 9-15 mm | 74 / 70 | 66 / 52 | 61 / 32 | 63 / 29 | 68 / 17 | 71 / 16 | 72 / 22 | 101 / 57 |
| output dynamic error while turning p95 mm | 18.9 | 10.9 | 9.3 | **9.2** | 10.2 | 13.4 | 17.0 | |
| tilt median deg | 0.82 | 0.97 | 0.86 | 0.84 | 0.71 | 0.70 | 0.72 | 0.77 |
| IMU vs trajectory dv rms all / shake m/s^2 | 0.222 / 0.59 | 0.244 / 0.55 | 0.239 / 0.54 | 0.235 / 0.53 | 0.232 / 0.55 | 0.232 / 0.55 | 0.232 / 0.64 | 0.243 / 0.70 |

All metrics agree that the camera times are a little late; the rotation-heavy mark (9-15) likes -2 to -3 ms, the
output's motion error and camlag -1.0 to -1.5 ms. **-1.5 ms** is the safe choice; closed loop with PiBridge's defaults it
gives mark 65 / 63 / 45 mm (against 63 / 71 / 71), dynamic error while turning p95 20.2 -> 13.8 mm.

Is it exposure dependent? If the true image time were `stamp + c` with a constant c (rather than `stamp + exposure / 2`,
which the driver assumes), study 1 (AE off, 3.5 ms power-on exposure, lag 0) and this take (about 5.5 ms, lag -1.4)
would both fit c of about 1.75 ms. Two points with flat minima are not proof; the take below settles it. Until then,
set the offset only with AE on.

## 6. IMU intrinsics

Nothing new to apply: the factory `ka` / `na` stay off (studies 1-2), and study 3 showed the six-pose bias+scale fit
moves the trajectory by less than 1 mm at the marks (IMU agreement 0.244 -> 0.224 m/s^2). The output layer now uses the
same accel bias as Basalt (section 7); if a per-headset six-pose calibration is ever written into the SLAM_CONFIG
calibration (`calib_accel_bias[3..9]`), the output layer reads only its first three terms (the bias) today.

## 7. Output layer: accel bias, prediction, smoothing, jitter, tilt

**Accel bias.** The predictor integrated the raw accelerometer between tracker poses; the Crystal's +0.34 m/s^2 on
the up axis (3 % too much gravity) then pushes every dead-reckoned stretch (pose interval + latency + prediction, about
85 ms) by about 1 mm up, and the blending keeps a standing offset. Removing the bias Basalt uses (SLAM_CONFIG calib,
else factory aBias) from every sample before the output layer sees it:

| 07:04, drift hold off | output-tracker position median / p95 mm | dynamic p95 mm | tilt median / p95 deg | jitter at rest mm |
|---|---|---|---|---|
| raw accel (Rust) | 13.5 / 22.9 | 6.32 | 0.69 / 1.46 | 0.122 |
| bias removed | **6.6 / 15.4** | 6.42 | **0.32 / 1.28** | **0.101** |

A synthetic held head with that bias shows the mechanism alone: 14.4 mm steady offset raw, 0.0 with the bias removed
(`pibridge_cam_headpibridge` `accel_bias_removes_the_steady_lag`).

**Rotation prediction.** Past the newest IMU sample the Rust code holds the last sample's rate. With the angular
acceleration (rate of the last 4 ms, acceleration from it and the 4 ms ending 20 ms earlier, k x that over at most
50 ms):

| display-time rotation error p95 (max), deg | 20 ms: 30-100 deg/s | 20 ms: > 100 deg/s | 40 ms: 30-100 | 40 ms: > 100 |
|---|---|---|---|---|
| rate held (Rust) | 0.83 | 0.88 (1.41) | 2.86 | 2.63 (4.68) |
| k 0.5 | 0.62 | 0.76 (1.27) | 1.58 | 1.62 (2.93) |
| **k 0.75 (default)** | **0.48** | **0.69 (1.13)** | **1.10** | **1.35 (2.42)** |
| k 1.0 | 0.46 | 0.63 (0.99) | 1.13 | 1.34 (2.11) |

k 1.0 overshoots at 40 ms in slow motion (overall p95 0.52 against 0.43); 0.75 is the compromise. Rotation jitter while
moving rises from 0.104 to 0.111 deg per frame (it follows the head's own angular acceleration); at rest unchanged.

**Smoothing and fusion.** Tried, not changed: without the blending (raw prediction) the output is 1 mm from the
tracker but jumps 1.9 mm per frame at rest (the 25 Hz sawtooth); the Rust blending + alpha-beta fusion gives 0.05-0.10
mm per frame. With Basalt's own velocity (closed loop) the jitter halves again (rest 0.046, moving 0.26 mm) while the
dynamic error while turning is larger than with a backward difference (p95 20 against 11 mm): which one is nearer the
truth cannot be told without ground truth (the reference is built from the same positions). The difftests replay a
backward-difference velocity; the live driver passes Basalt's.

**Creep at rest** (3-5 mm/s) is the tracker's (the output follows it within 0.3 mm/s); see section 4.

## 8. Basalt settings (slambatch, 07:04, open loop)

| run | mark 2-9 / 2-15 / 9-15 mm | still drift mm/s (2,7,9,11,13,15) | tilt median / max deg | dv rms all / step / shake |
|---|---|---|---|---|
| L (session: grid 30) | 73 / 66 / 52 | 3.3 9.9 4.2 4.7 3.7 2.5 | 0.97 / 2.63 | 0.244 / 0.34 / 0.55 |
| L rerun | 73 / 65 / 52 | 3.3 9.9 4.2 4.7 3.6 2.5 | 0.97 / 2.63 | 0.244 / 0.34 / 0.55 |
| optical flow recall | 80 / 74 / 51 | 3.3 10.2 4.3 5.2 4.0 2.5 | 0.86 / 2.70 | 0.231 / 0.32 / 0.55 |
| grid 25 | 223 / 165 / 63 | 5.0 12.5 3.8 4.3 3.4 1.8 | 1.23 / 3.27 | 0.382 / 0.67 / 0.59 |
| grid 20 | 85 / 72 / 35 | 3.5 12.1 3.1 5.2 3.3 2.5 | 0.80 / 2.87 | 0.234 / 0.33 / 0.61 |
| 2 points per cell | 160 / 140 / 23 | 4.9 14.7 4.2 5.2 4.9 2.9 | 0.94 / 3.49 | 0.301 / 0.44 / 0.61 |
| 10 key frames | 86 / 82 / 38 | 3.5 9.4 4.9 6.4 4.2 2.5 | 0.90 / 3.85 | 0.466 / 0.86 / 0.57 |
| 4 pyramid levels | 77 / 86 / 66 | 4.4 7.8 4.5 5.0 3.1 4.0 | 0.83 / 2.94 | 0.216 / 0.22 / 0.55 |

Nothing beats the session config on the marks; the time offset (section 5) is the only Basalt-side change that helps.
No Monado or Basalt patch is proposed: everything above goes through existing switches. The camera pairs (the Crystal
has a lower and an upper pair) cannot be compared: every take so far recorded only cameras 0 and 1 (take 4 below).

## Switches (environment Monado reads; `PIMAX_*` names like the rest of libpibridge-cam)

| Switch | PiBridge default | Parity (`PIBRIDGE_HEAD_PARITY=1`) | Values |
|---|---|---|---|
| `PIBRIDGE_HEAD_PARITY` | unset | 1 | 1 / yes / true: everything below at the Rust value unless set explicitly |
| `PIMAX_SLAM_DRIFT_HOLD` | 0 | 1 | 0 off, 1 the Rust hold, 2 quiet-only hold (experiment) |
| `PIMAX_SLAM_SEED_BIAS` | 1 | 0 | start the output's gyro bias at the driver's cache |
| `PIMAX_SLAM_ACCEL_BIAS` | 1 | 0 | remove the tracker's accel bias from the output layer's samples |
| `PIMAX_SLAM_PREDICT_ACCEL` | 0.75 | 0 | 0..1.5, fraction of the angular acceleration in the rotation prediction |
| `PIMAX_SLAM_STILL_BACKOFF` | 0 | 0 | N: up to 2^N s between still re-zeros (measured worse) |
| `PIMAX_SLAM_RECORD_ACCEL_BIAS` | 1 | 0 | the recording carries the calibration the tracker used |

The `6DoF output: ...` line at `track_create` keeps its Rust text and markers and appends `; pibridge: <what is on>`
(nothing in parity mode).

## Live takes still needed (later a guided step on the Development page)

Each with **Record tracking**, the guided protocol, worn, a lit room, AE on, Extra environment as given:

1. **Second worn take, time offset.** `PIMAX_HEAD_IMU_LOG=1` (no offset). Same 15 steps, plus a 10 s fast head shake
   left-right and 10 s up-down at the end. Offline: camlag on both cameras and the Basalt sweep 0 / -1.0 / -1.5 /
   -2.2 ms. Confirms -1.5 ms before it becomes the default. The head IMU log also gives the **real tracker latency**
   (its V lines come in arrival order between the IMU lines), which the replays assume to be 25 ms.
2. **AE off vs on, exposure dependence.** Two short takes (2 min each: the shake steps only), one with auto exposure
   off (Tracking page, Cameras), one with it on, same room and light. If the camlag lag moves by about half the
   exposure difference, the driver's mid-exposure shift should become a constant (+1.75 ms) instead (a Monado
   change, then proposed as a patch under `patches/`).
3. **Desk with AE on.** The headset lying on the desk for 60 s facing the room (not a wall), then picked up and worn
   for 30 s. Tells whether the still re-zeros still fire with good images (the desk take had AE off and dark images).
4. **Four cameras** (ROADMAP 1b). No recording with four cameras exists yet (every take so far has cam0/cam1 only),
   so nothing was compared offline. The take: Tracking page, Head tracking cameras **4 (all)**, everything else as
   recommended (AE on, grid 30); Development page, **Record tracking**, guided, the 15 built-in steps worn, in the
   usual lit room with the usual mark; then one more take the same way with **2 (lower pair)** right after (same
   light, same mark), so the live tracker is compared too, not only replays. Check first in the Monado log: "SLAM
   sensors: cameras 0 (trackingA), 1 (trackingB), 2 (Pi-controllerA), 3 (Pi-controllerB)", "Basalt with
   cam_count=4", the AE line's two pair exposures, and no "SLAM_CONFIG calibration has" error; the recording has
   `cam0`..`cam3` and a four-camera `pimax-basalt-calib.json`.
   Offline, the four-camera take through `monado-cli slambatch` three ways with the same settings and biases (only
   the cameras differ): all four (the recording's calibration and config); the lower pair (`EUROC_CAM_COUNT=2`, the
   same calibration with the last two cameras removed); and, if the lower pair loses the view, the upper pair alone
   (a `--cams 2,3` export with the recording's biases). Compared with `pibridge-slam headreplay` on the steps: mark
   error between the mark steps (2, 9, 15), creep and jitter at rest (2, 7, 9, 11, 13, 15), losses / resets /
   re-zeros (most in the look-up / look-down and shake steps), tilt error, and Basalt's points per frame and per
   camera (more wall and ceiling points is the reason for the upper pair), plus Basalt's time per frame (four
   images of optical flow; the live take shows whether it keeps 25 Hz). The live 2-camera take checks that the
   lower-pair replay of the four-camera take matches what the user saw. 4 becomes the default only if mark error
   and losses are better and nothing gets worse; then the frame-time choice (mean mid-exposure) is checked with
   camlag per pair on that take.
5. **Drift hold A/B.** Two takes of the protocol with `PIMAX_SLAM_DRIFT_HOLD=2` and `=0`, a longer step away (3-4
   steps and back) and a 30 s still on the mark: only a live take shows whether the quiet-only hold leaves a slow
   lean alone.

## Files

- Library: `libpibridge-cam/src/headpose.cpp` (from_env's PiBridge block, `load_biases`, `pibridge_describe`, the predictor's
  extrapolation, the accel bias at `push_imu`, the seed in the constructor, the quiet-only gate),
  `include/pibridge/cam/headpose.h` (`parity_mode`, the new `HeadParams` / `PredictorParams` fields), `slam.cpp/.h`
  (`slam_config_calib_path`, `calib_json_biases`, `StillParams::backoff_max`, `Supervisor::rezero_streak`),
  `drift.h` (`quiet_gyro_std`, `quiet_accel_std`), `monado-abi/src/slam_capi.cpp` (the recorder).
- Tests: `libpibridge-cam/tests/test_headpibridge.cpp` (`pibridge_cam_headpibridge`, 10 cases),
  `monado-abi/tests/test_capi_slam_pibridge.cpp` (`monado_abi_capi_slam_pibridge`, 3 cases); the SLAM difftests,
  `monado_abi_capi_slam` and `monado_abi_diff_ae_scene` run with `PIBRIDGE_HEAD_PARITY=1`.
- Tool: `pibridge-slam headreplay` (`tools/slam/src/headreplay.*`).
- Scratch (not in the repo): `~/gamedev/ext/slam/study4/` (`hr.sh` headreplay on a take, `vl.sh` + `vitloop/` the
  closed loop, `vdrep.sh` the desk repeats in `rep/`, `mk4.py` + `queue4.sh` the slambatch variants in `runs/`,
  `ds/` shifted datasets (links), `table4.md`, `table4-shift.md`).

## Still re-zero while worn? (live session 2026-10-19 21:06)

The still test (every axis: gyro sd < 0.2 deg/s, accel sd < 0.03 m/s^2 over 0.5 s) replayed on every recording:
the worn takes 07:04 and 20:09 (chair) have **no** window that passes; 19:24 and the 18:30 log pass only while the
headset lay on the desk; the desk take passes in its desk stills. In the session's own head IMU log the five re-zeros
fall in one 5.4 s window right after the headset went 0.85 m down (tracker height +0.35 -> -0.50 m), with gyro sd
0.055 deg/s and accel sd 0.010 m/s^2: the bare sensor at rest (a worn head shows 0.3-1 deg/s and 0.05-0.1 m/s^2), so
the headset was lying on something, not worn. The driver's own rest detector agreed ("gyro bias refined at rest").

A stricter gate was tried and not kept: 0.12 deg/s and 0.02 m/s^2 over 2 s moved the output during the desk stills by
261 mm (0.5 s: 87 mm, 1 s: 126 mm, Rust thresholds 52-64 mm), because the dark tracker runs away before the hold
starts. `PIMAX_SLAM_STILL_ENTER_MS` (default 500) stays for A/B. No wear sensor is decoded in any report (omarchyvr
`analysis/auto-mode.md`); `GetVisorStatus` (0x2c) is a request the allowlist does not have.
