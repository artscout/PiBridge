# Optical controller tracking: improvements over the Rust port (2026-10-19)

The M6/M7 port (`libpibridge-ctrl`, `pibridge-ctrltrack`) is byte-exact with omarchyvr's `pimax-ctrltrack` / `pimax-hands`.
This page records what PiBridge now does better, measured offline, and how the exact Rust path stays reachable.

## The parity switch

- `PIBRIDGE_CTRL_PARITY=1` (or `on` / `true`), read once per process (`libpibridge-ctrl/include/pibridge/ctrl/parity.h`):
  blobs, tracker and fuser behave exactly as the Rust code, bit for bit.
- Without it, `pibridge-ctrltrack live` uses `BlobParams::improved()` and `TrackCfg::improved()`, and the fuser behind
  the C ABI (`pimax_ctrlfuse_create` -> `Config::from_env()`) starts from `Config::pibridge()`.
- The library defaults (`BlobParams{}`, `BlobParams::live()`, `TrackCfg{}`, `fuse::Config{}`) are still the Rust
  values, so every C++ twin of a Rust `#[test]` is unchanged.
- The difftests against the Rust oracle (`pibridge_ctrltrack_diff_*`, `pibridge_ctrltrack_csv_*`,
  `pibridge_ctrltrack_live_broker`, `monado_abi_diff_hands_*`, `monado_abi_hands_capi`) set `PIBRIDGE_CTRL_PARITY=1`
  through CTest (`ENVIRONMENT_MODIFICATION`). The test cases themselves are unchanged.
- `track`, `scan`, `show` and `health` are unchanged (no improvements there). Only `live` and the fuser differ.
- The C ABI (`pimax_ctrlfuse.h`, `pimax_hands.h`, `pimax_inhand.h`) and the pose file layout are unchanged.

A/B switches without parity (environment): `PIBRIDGE_CTRL_INS_MS` (0 = no inertial position),
`PIBRIDGE_CTRL_INS_TAU_MS`, `PIBRIDGE_CTRL_INS_FREE_MS`, `PIBRIDGE_CTRL_INS_JUMP_MM` (0 = no inertial jump gate),
`PIBRIDGE_CTRL_TILT` (0/1), `PIBRIDGE_CTRL_KEEP_S` (0 = None after `lost_s` as in Rust), `PIBRIDGE_CTRL_GRAV_DEG` (0 = off).
The Rust `PIMAX_CTRL_TRACK_*` variables still apply on top.

## The harness: `pibridge-ctrlbench`

`tools/ctrltrack/src/bench.cpp`, `bench_fuse.cpp`. Built with `pibridge-ctrltrack`, not installed, no Rust twin.

```
pibridge-ctrlbench FILE.pxcam --calib XML --factory [--parity] [--sim K] [--mask-every S --mask-s D]
                [--fixes OUT.csv] [--leds OUT.csv] [--fuse [--fuse-parity] [--bias-err DPS] [--swap-p P] [--seed N]]
```

It replays the LED frames through the `live` code path (blobs, then `Tracker::step`) as fast as possible. It reports:

- **Tracking:** tracked-frame % per hand, the share of few-LED fixes, gaps, jumps (more than 8 cm between
  consecutive fixes), the p95 of the frame-to-frame step.
- **Jitter:** second differences of three consecutive fixes at under 0.15 m/s, as the noise sigma in mm and deg.
- **Fit quality:** mean rms px and LED count; 5-7 LED fixes compared with the interpolation of 8+ LED neighbours.
- **Hand identity:** left/right swaps, and frames where the left hand is right of the right hand by more than 10 cm.
- **CPU:** ms per LED frame for the blobs and for the tracker (mean and p95).

Options:

- `--sim K`: scales every LED-frame pixel by K, saturating at 255. This is a brighter or darker LED exposure.
- `--mask-every`: occludes one hand at a time. It removes that hand's blobs (the ones the unmasked run matched,
  plus any blob within 25 px of its projected LEDs) for D seconds. It then reports frames to relock and the error
  of the first fix after the occlusion.
- `--leds`: writes every matched LED's blob (peak, area, size) per frame.
- `--fuse`: the fuser bench (section 4).

Recordings used (lab data, outside the repo): `ctrl1.pxcam`, `ctrl2.pxcam` and `first.pxcam` (Crystal Super, both
controllers, 434 / 433 / 72 LED frames, LED exposure 25 / gain 35), with the Crystal calibration and factory
extrinsics, as the session runs. `ctrl2` is the hard one: fast motion, many gaps. `first` has one controller on
the desk (bright) and one dim one beside it. The pibridge recordings of 2026-10-19 (`~/.local/share/pibridge/recordings`)
hold only SLAM frames (EuRoC), so they have no LED frames. No recording has LED frames at 40 / 62: the AE runs only
in sessions, and RECORD TRACKING runs with `PIMAX_CAM_AE=0`.

CPU numbers are from a shared, loaded machine (load average 6-9, other agents building). They are the means of
three runs.

## 1. The 0 % live session, and LED detection robust to exposure

### What explains the 0 %

The log `ctrltrack-20261019052737.log` shows:

- 0 % tracked over the whole session;
- searches in exactly one LED frame out of three (50 of 151 per 5 s), which is the tracker's idle rule after 2 s
  with nothing tracked;
- 0.9-1.3 ms frame work, which is the blob pass alone. A brute-force search with 3 or more blobs costs about 10 ms,
  so the mean would be 3 ms or more if any search had something to work with.

So **no LED frame had LED blobs**: no blob reached the required peak of 120.

The three pibridge recording sessions of 07:02-07:04 show the same signature (0 %, 1 ms). They were guided head-motion
recordings with no controller step.

**The exposure is not the cause.** That session's Monado log has no auto-exposure line (no `auto-exposure ON`, no
`AE on:` statistics), so the LED frames kept the recordings' 25 / 35. The AE's Windows LED pair 40 / 62 would make
the LEDs brighter, never darker. The bench gives:

- simulated 40 / 62 (`--sim 2.83` = 40/25 x 62/35): the Rust detection still tracks 84.6 / 86.4 % on ctrl1;
- the same detection at 0.45x brightness: **0 % on every recording**, with the same 1-in-3 search pattern.

The cause is therefore one of:

- the controllers were out of the cameras' view (the take was on a desk);
- their LEDs were off (controllers asleep, or not linked: Monado logged only the attach bursts and no fixes);
- their LEDs were dimmer than 47 % of full scale: far away, turned away, or lying on the desk.

`first.pxcam` shows the third case: its second controller lies dim on the desk and the Rust detection never finds
it. A `.pxcam` of such a session settles which one (see "Live takes").

### Changes (default without parity)

- **Core blobs** (`BlobParams::core`, `core_frac` 0.5). Each component found at the threshold is re-cut at
  max(threshold, 0.5 x its peak):
  - the centroid is weighted over the core only;
  - LEDs whose halos merged come out as separate blobs;
  - the area and side limits apply to the core;
  - a halo more than 4x the side limit is still rejected as a lamp or window.
- **Dim LEDs** (`BlobParams::dim`, `dim_min` 40). When no pixel of a tile reaches the required peak but the
  brightest pixel is 40 or more, the threshold and the peak test scale with that brightest pixel (80/255 and
  120/255 of it).

| Recording, brightness | Rust L / R % | PiBridge L / R % | Rust jitter L (mm / deg) | PiBridge jitter L (mm / deg) |
|---|---|---|---|---|
| ctrl1 x1 (25/35) | 84.6 / 86.6 | 84.8 / 86.6 | 1.23 / 0.85 | 1.36 / 0.97 |
| ctrl1 x2.83 (as 40/62) | 84.6 / 86.4 | 85.3 / 86.6 | **3.22 / 3.00** | **1.39 / 1.00** |
| ctrl1 x5 | 76.7 / 78.6 | 83.4 / 85.9 | 4.17 / 3.76 | 2.15 / 1.88 |
| ctrl1 x0.45 | **0 / 0** | **84.8 / 86.6** | n/a | 1.36 / 0.97 |
| ctrl1 x0.3 | 0 / 0 | 84.8 / 86.6 | n/a | 1.36 / 0.97 |
| ctrl2 x0.45 / x0.3 | 0 / 0 | 52.4 / 67.0 | n/a | n/a |
| first x5 | 0 / 0 (all merged) | 28 / 97.3 | n/a | n/a |
| first x2.83 | 0 / 97.3 | **93.2 / 97.3** (the dim desk controller found) | n/a | n/a |

- The right-hand jitter at x2.83 is 1.39 -> 1.10 mm. The mean LED count at x2.83 is 9.2 -> 9.7.
- At x1 the PiBridge detection is the same within noise. The left jitter of 1.23 -> 1.36 mm is within the
  run-to-run spread of the core cut (0.35-0.65 gave 1.20-1.36).
- Blob CPU is unchanged (0.55-0.57 ms per LED frame).
- Not done: a local (per-blob) dim rule. In `first`, a dim controller next to a bright one in the same tile stays
  invisible at x1.

## 2. Out of view: IMU dead reckoning, hold, re-acquisition (fuser)

### Measurement

The fuser bench (`--fuse`) builds its inputs this way:

- **Truth:** the tracked trajectory of each hand (fixes smoothed over five).
- **Synthetic controller IMU** at 500 Hz in raw units. This is needed because no recording has the controller IMU
  next to LED frames. It includes:
  - the desk noise of a real controller (omarchyvr fixture `inhand_desk`: gyro sigma 1.1 LSB, accel 1.2 LSB);
  - a gyro bias error the fuser does not know (0.3 or 1.0 deg/s per axis);
  - a 5 LSB accel bias and the 3.6 % accel scale error measured on the desk.
- **Optical fixes:** the tracker's own (noisy) fixes, handed over 30 ms after their frame time.

The run:

- The optical input is cut for 0.5 / 1 / 2 / 5 s at every 0.25 s where the truth covers 1 s before and 0.7 s after.
- The shown pose (`Fuser::get` at 90 Hz) is compared with the truth.
- When the Rust fuser gives up (State::None), the shown pose is Monado's rest pose.

Numbers: ctrl1, 40-76 runs per cut length, three seeds (identical to +-1 mm). ctrl2 and `first` have too few
long tracked spans (1-3 runs).

**Caveat:** the IMU is ideal apart from the modelled noise and biases. Real IMU timing, axis-calibration error and
hand tremor are not modelled. It needs the live take below.

### Changes (`Config::pibridge()`)

1. **Inertial position** (`ins_s` 0.6 s).
   - One Kalman filter per world axis (position, velocity, accelerometer bias) runs on the accelerometer rotated
     by the gyro orientation. Every optical fix corrects it, using a delayed update for the fix latency.
   - The world-frame bias it learns also absorbs the accel scale error on gravity.
   - The shown pose is the last fix plus the filter's displacement since that fix's frame time. This gives the
     **prediction to display time while tracked** (no velocity staircase after a gap) and carries the hand on out
     of view.
   - After `ins_free_s` (0.25 s) the velocity is damped (tau 0.5 s); a quiet gyro means zero velocity (ZUPT); the
     distance from the last fix is capped at 0.35 m.
   - After 0.6 s the Rust arm-model hold takes over from where the filter ended. The elbow is fixed in the body
     frame and the wrist turns with the gyro.
2. **Tilt fix** (`tilt_fix`). Out of view, the gyro orientation's pitch and roll are pulled towards the
   accelerometer's gravity at 0.5/s, only when the accelerometer reads 0.85-1.15 g and the gyro is under 1 rad/s.
3. **No give-up** (`keep_tau_s` 1.5 s).
   - Past `lost_s` (4 s), while the gyro streams, the fuser keeps Hold instead of returning None. The hold glides
     to the rest pose (tau 1.5 s), so there is no snap there or back.
   - Past 1.5 s the gyro orientation is re-anchored every 1 s. The Rust fuser's 3 s gyro ring otherwise loses the
     fix time between 3 and 4 s, and the orientation goes wrong.
4. **Faster glide at relock** (`blend_mps` 0.5 -> 1.0 m/s; the blend is still at least 0.15 s and at most 0.5 s).

| Cut, at | Rust error p50/p90 | PiBridge error p50/p90 |
|---|---|---|
| 0.5 s cut, 0.10 s in | 3 / 47 mm | **2 / 7 mm** |
| 0.5 s cut, 0.25 s in | 5 / 122 mm | **5 / 17 mm** |
| 2 s cut, 0.50 s in | 7 / 126 mm | **10 / 35 mm** |
| 2 s cut, 1.00 s in | 15 / 236 mm | 19-24 / **133-136 mm** |
| 5 s cut, 1.00 s in | 59-61 / 243 mm | **33-38 / 133-135 mm** |
| 5 s cut, 2.00 s in | 63-64 / 243 mm | 71-73 / **196-198 mm** |
| 5 s cut, 4.50 s in | 281 / 367 mm (rest pose, State::None 20 % of the cut) | **90-92 / 233-237 mm** (Hold, never None) |
| Orientation, 5 s cut, 4.5 s in, bias error 0.3 / 1.0 deg/s | n/a (None) | 1.5-1.9 / 4.6-5.7 deg p50 |
| Orientation, 5 s cut, 2 s in, bias error 1.0 deg/s | 3.6-3.7 deg | 2.9-3.1 deg |
| Relock jump, largest step beyond the truth's own, p50/p90 (0.5 s cut) | 6.0 / 37.6 mm | **4.6 / 20.8 mm** |
| Relock jump, 2 s cut | 11.5 / 32.7 mm | 9.7 / 28.0 mm |
| Relock jump, 5 s cut | **280 / 369 mm** (rest pose to optical) | **11.7 / 29.2 mm** |
| Error 0.3 s after relock, 0.5 / 1 / 2 s cuts, p90 | 142 / 144 / 142 mm | **11 / 23-31 / 20 mm** |
| Error while tracked (pose for "now", 30 ms after the frame), p50 | 1.2-1.4 mm | **0.9-1.0 mm** |

Notes on the table:

- The one loss is the median at 1 s into a 2 s cut: 15 -> 19-24 mm. A hand at rest drifts slightly under the
  inertial filter, while the Rust hold freezes it exactly. Every p90 and every other time is better.
- The "1 s / 2 s" p90 tails are hands that moved 15-25 cm during the cut. Neither the hold nor a damped INS can
  know where they went.
- Tuning tried: `ins_s` 0.4 / 0.6 / 0.8 / 1.0 s and damping tau 0.15-0.5 s. 0.6 s / 0.5 s was best at 1-2 s
  without losing the first 0.5 s.
- The Rust dead reckoning (`PIMAX_CTRL_TRACK_DR_MS` 80-300) changed nothing or made it worse (its gravity estimate
  is the mean accelerometer before the fix).

### Re-acquisition in the tracker

`--mask-every`, one hand at a time, 0.5 / 1 / 2 s:

- **ctrl1:** both trackers relock in the first LED frame where the LEDs are back (8/8, 5/5, 4/4 cases). The
  brute-force search runs in every frame while a hand is missing, and in every third frame after 2 s with nothing
  tracked (at most 66 ms at 30 Hz).
- **ctrl2:** the Rust tracker needed 20 frames (664 ms) in one case per cut length. PiBridge relocked at once in all
  of them. Both have one wrong first fix (24-39 cm) among the 0.5 / 1 s cases. The fuser's re-entry confirmation
  and the inertial jump gate (section 3) keep it from being shown.

So the tracker does not need a smaller IMU-predicted search window for speed: relock is already immediate when 5
LEDs are visible in one camera. Where an IMU prior would help is the 3-4 LED (and 1-2 LED) relock, which is
tracked only from a recent prior today. That needs the fuser's orientation in `pibridge-ctrltrack` (see follow-ups).

## 3. Identity: LED modulation, byte 45, left/right swaps

### LEDs are not modulated

The test, on ctrl1 and ctrl2:

- 9673 matched LED observations (threshold 20, so the area is a brightness integral; the peaks are saturated);
- each LED's blob area compared with the median of its own neighbouring frames, as log(area/median).

The result:

- no dependence on frame phase (n mod 2, 3, 4) per controller: all means are within +-0.013, the sd is 0.05-0.25;
- per LED and phase: no z-score above 3, out of 52 and 32 LED x phase cells.

There is no Constellation-style blink code at the LED-frame rate. LED identity stays geometric.

### Byte 45 of the controller report

From the omarchyvr fixtures `ctrl_buttons_hub2`, `p82` and `p83`:

- **Values:** 00/ff, in runs of 5 or 10 reports at 4 ms each (20 / 40 ms, one or two frame periods of the 50 fps
  Dream Air camera).
- **Left/right:** the left slot reads mostly 00 (58 %) and the right mostly ff (87 %). Neither strictly alternates.
- **Interpretation:** it changes on camera-frame boundaries, so it is plausibly an LED/exposure sync flag. It does
  not separate the hands by itself.
- **Not tested:** whether it marks the LED frames. No recording holds controller reports and LED frames together,
  so the correlation cannot be made offline (see "Live takes").

### Wrong LED model (tracker, `TrackCfg::slot_model`)

On ctrl2, 11 % of the Rust fixes (64 of 560) have the other hand's LED model. The two models are nearly mirror
images, and such a fix sits about 10 cm off. The 10 cm jumps alternate with the model label (frames 107-115 of
ctrl2). PiBridge solves each slot with its own model:

- a search fix labelled with the other model is re-fitted with the slot's model (its alt fit, a two-pass match
  from its pose, or a fresh single-camera acquisition on its blobs);
- if none of those fits, the fix is dropped. On ctrl2, 23 were re-fitted and 13 dropped.

| | Rust | PiBridge |
|---|---|---|
| ctrl2 jumps over 8 cm | **14** | **5** |
| ctrl2 tracked L / R | 57.7 / 71.6 % | 52.2 / 67.0 % (dropped wrong-model fixes, short gaps the fuser bridges) |
| ctrl2 left-right-of-right frames | 13 | 10 |
| ctrl2 at x2.83: jumps, swaps | 27, 4 | 16, 0 |
| `first`: the desk controller | left model, 9 LEDs | right model re-fit, 12.5 LEDs |
| ctrl1 | identical | identical |
| Tracker CPU, ctrl1 / ctrl2 | 0.82 / 7.0 ms | 0.82 / 7.55 ms |

### Swaps in the fuser (IMU against the optics)

- **Gravity check** (`grav_check_deg` 25). A fix whose gravity direction disagrees with the controller's
  accelerometer by more than 25 deg is dropped as `tilt`. This is absolute, so it does not drift.
  - It runs only with a gyro calibration and while the accelerometer reads about 1 g.
  - It switches itself off if more than half of 60 fixes fail, which would mean a wrong axis calibration.
- **Inertial jump gate** (`ins_jump_m` 0.06). A fix more than 6 cm + 1 m/s x age from the inertial prediction
  (within 0.2 s of the last fix) is a `jump`, dropped up to three times in a row like the optical jump test.

The bench handed the fuser the other hand's fix in 5 % of the frames (`--swap-p 0.05`, ctrl1). The other-hand
fixes it took:

| | Rust | PiBridge, gravity check only | PiBridge |
|---|---|---|---|
| 0.5 / 1 / 2 / 5 s cut runs | 78/205, 73/180, 71/161, 52/90 (35-58 %) | 54/145 (2 s runs) | **3/205, 0/180, 1/161, 3/90 (0-3 %)** |

Errors without swaps are unchanged. The gravity check alone does little, because both hands are held at similar
tilts. The IMU's position does the work.

## 4. Other weak points found

- **Few-LED tracks:** they ride on a rotation prior extrapolated from the optical fixes (300 px/rad). The gyro
  orientation lives in the fuser (in Monado), not in the tracker, so 1-4 LED fixes cannot use it.
- **The `first` desk controller:** at x1 a dim controller next to a bright one stays invisible (the tile maximum
  rule). A local peak rule would find it.
- **CPU:** the brute-force search is the cost (ctrl2: 7 ms mean, p95 16 ms). It is unchanged.

## 5. The IMU in the tracker: the hint file (built; tracker use off by default)

### The file

The fuser (in Monado) writes, at most every 4 ms per hand, its best estimate of the controller's true pose relative
to the head. It writes to `$XDG_RUNTIME_DIR/pibridge-ctrl-hint`; the exact layout is in
`libpibridge-ctrl/include/pibridge/ctrl/hint.h`. The format:

- 2096 bytes, magic "FCHT", version 1, the writer pid;
- per hand, a seqlock and a ring of 16 entries;
- each entry: time, last fix time, state, flags, head_from_aim position and orientation, and the gyro rate.

The pose in each entry is built this way. It is not the smoothed display pose: that lags 5-10 cm in fast motion,
which made the first try worse.

- **Position:** the last accepted fix as measured, moved by the inertial filter's displacement since.
- **Orientation:** that fix's orientation, turned by the gyro since.

Facts about the writer:

- It is on in `Config::from_env()` unless `PIBRIDGE_CTRL_PARITY=1` or `PIBRIDGE_CTRL_HINT=0`. `Config{}` and
  `Config::pibridge()` never write a file, so the tests write nothing.
- The C ABI is unchanged: the file is written from `Fuser::get_with`, which `pimax_ctrlfuse_get` already calls.

`pibridge-ctrltrack live` reads the file only with `PIBRIDGE_CTRL_TRACK_HINT=1`. It interpolates the ring at each LED
frame's time (or carries the newest entry on by its rate, for up to 60 ms) and converts it to body_from_model (the
inverse of the aim pitch). It skips the file when it is missing, the writer is dead, or the gyro is not calibrated.

### What the tracker does with it (`TrackCfg::use_hint`)

The hint is used only once the slot's last fix is older than `hint_stale_s` (50 ms). Fresh constant-velocity
priors were better: using the hint for fresh frames raised the error of all fixes against the truth from 2.9 to
24 mm p90 on ctrl1. Once the prior is stale:

- **Few-LED fixes:** the hint's pose is the prediction, its orientation is the rotation prior (600 px/rad), and
  2-LED fixes are allowed. For 2 LEDs the fit must have rms 1.0 px or less and stay within 2 cm + 0.5 m/s x age.
- **Relock:** a slot without a prior relocks from the hint when the fuser's last fix is at most 1 s old.
- **Left/right in the search:** a search fix is refused when it turns like the other hand's hint (under 17.5 deg)
  and not like this hand's (over 35 deg).

### Measured

Closed loop on the recordings:

- The fuser of both hands runs on the synthetic IMU (`bench_fuse.cpp` `HintSim`).
- The tracker's own fixes reach the fuser 30 ms after their frame.
- The hint at the frame time goes back to the tracker.
- One hand at a time is cut for 1 s every 2 s.
- The truth is the smoothed reference run.

`pibridge-ctrlbench FILE --hint [--mask-every 2 --mask-s 1]`, seeds 1 and 2 identical:

| | ctrl1 hint off | ctrl1 hint on | ctrl2 hint off | ctrl2 hint on |
|---|---|---|---|---|
| tracked L / R | 84.8 / 86.6 % | 85.0 / 86.6 % | 52.2 / 67.0 % | 52.4 / 70.4 % |
| jumps over 8 cm | 0 | 0 | 5 | **7** |
| fixes with 4 or fewer LEDs: count, error vs truth p50 / p90 | 4, 7.0 / 14.6 mm | 4, 7.0 / 14.6 mm | 24, 10.4 / 25.6 mm | 29, **14.9 / 30.6 mm** |
| with the cuts: tracked L / R | 71.4 / 65.9 % | 71.4 / 65.9 % | 42.5 / 46.4 % | 43.6 / 51.0 % |
| with the cuts: relock, wrong first fix, jumps | 0 ms, 0, 0 | 0 ms, 0, 0 | 0 ms, 1, 3 | 0 ms, 0, 5 |
| swaps | 0 | 0 | 0 | 0 |
| tracker ms per frame (p95) | 0.82 (5.2) | 0.66-0.68 (3.6) | 7.5 (16.8) | 7.2 (16.6) |

What the table says:

- The hint finds a few percent more of the right hand on ctrl2, and one wrong relock less.
- It also makes 2 more jumps, and its few-LED fixes are a third less accurate.
- On ctrl1 it changes nothing.
- So, by the rule "default on only where better", the tracker does not use it by default.

Two limits of this test:

- The synthetic IMU is made from the tracker's own fixes, including their orientation errors (ctrl2 has 180-deg
  flips), so the hint inherits them. The fresh hint agrees with the next fix to 3.7 deg / 14.5 mm p50 but 64 deg /
  49 mm p90 on ctrl2.
- A real IMU has no such flips. The live take below, replayed with `--imu-log` and run with
  `PIBRIDGE_CTRL_TRACK_HINT=1`, is the test that decides.

Variants tried, none better: `hint_few_leds` 2 / 3 / 5 (5 = no few-LED path), rotation prior 300 / 600 px/rad, the
relock age 0 / 1 s, the stale limit 50 / 90 ms, and the left/right refusal against this hand's hint alone (it refused
208 good fixes on ctrl2).

## 6. The fuser on a real IMU log

`pibridge-ctrlbench --imu-log LOG.csv [--calib-json ctrl-imu.json] [--fuse-parity]` reads the log that
`PIMAX_CTRL_IMU_LOG=PREFIX` makes the driver write: `PREFIX-left.csv` / `-right.csv`, `# pimax-ctrlfuse log v1
hand=...`, with lines

- `I,host_ns,dev_us,gx,gy,gz[,ax,ay,az]`
- `O,frame_ns,world_from_aim(7),world_from_head(7),rms_px,leds,verdict`
- `P,t_ns,state,shown(7),head(7),other(3)`

It reports:

1. **Replay check:** the whole log replayed in file order (I samples, O fixes as the driver handed them, a `get` at
   each P line), and how many of the replay's verdicts are the logged ones. With the logging fuser's config and
   calibration that is every one of them: 588 / 588 on the self-test.
2. **Cuts:** cuts of 0.5 / 1 / 2 / 5 s wherever the logged good fixes (`ok`, 8+ LEDs) are dense (every 100 ms
   over the cut and 0.5 s after). The truth is those fixes, interpolated. It reports the error at fixed times into
   the cut, how often the fuser gives up, the relock jump, and the error 0.3 s after the relock.

Facts about the replay:

- The driver queries `get` at the display rate, but P is logged at 50 Hz. The replay's queries are therefore
  sparser than live, so the blend start can differ by up to 20 ms.
- `pibridge_ctrlbench_imu_log` runs `--selftest-imu-log`: a synthetic hand on a 0.1 m circle at 0.5 Hz, logged by the
  Rust fuser, then replayed.

On that self-test log:

| | Rust fuser | PiBridge fuser |
|---|---|---|
| 0.25 s into a cut (p50) | 68 mm | 5 mm |
| 0.5 s into a cut (p50) | 136 mm | 25 mm |

## 7. The take as guided steps

`tools/ctrltrack/ctrl-take-steps.json` holds the "Live takes" item 1 as the app's guided steps:

- the `rec_steps` format of `app/src/core/rec_steps.h` (only "steps" is read; the "about" and "record" keys say
  how to record around it);
- 12 steps, 93 s, kinds `prepare`, `still` and `move`;
- `pibridge_ctrltrack_take_steps` checks that it parses like the app's parser.


## 8. The real take with the controller IMU (2026-10-19 15:42)

The data:

- `ctrl-imu-take.pxcam`: 30 s, 750 LED frames, at the AE's LED pair 40/62.
- The fuser logs `ctrlimu-left.csv` / `ctrlimu-right.csv` on the same clock.
- The script: 0-10 s the right hand only, with hard stops; 10-20 s the hands crossing; 20-30 s each hand behind the
  back.
- In the logs the head pose is the identity (no head tracking in that session), so the tracking world is the head
  frame.

How it is measured:

```
pibridge-ctrlbench TAKE.pxcam --calib XML --factory --imu L.csv,R.csv --calib-json ctrl-imu.json [--imu-id 0|1] [--imutrack]
```

The IMU logs are replayed into a fuser per hand. Its IMU state (hint file v3) goes to the tracker. The check is
against each controller's own gyro, per pair of consecutive fixes that turn at least 5 deg:

- "own": within 15 deg of its own gyro;
- "wrong hand": more than 25 deg off its own gyro, within 15 deg of the other's;
- "neither": more than 25 deg off both, a mirror fit or garbage.

### Hand identity from the IMU in the per-frame tracker (`TrackCfg::imu_id`, default on)

A fix belongs to a hand only if:

- **Turn:** it turned like that hand's gyro since that hand's last fix (25 deg), or, without a recent fix, it is
  tilted like its accelerometer says (30 deg);
- **Rest:** a hand whose gyro is quiet does not move or turn more than 3 cm / 6 deg from its last fix;
- **Parked:** a hand quiet for 5 s and not tracked gets no new fix;
- **Silent:** a hand whose IMU stopped while the other's goes on gets nothing. When both are quiet (no writer), the
  test is off.

The IMU comes from the hint file v3, 5168 bytes, with these fields per entry:

- the gyro-integrated orientation, gravity up in the aim frame, how long the gyro has been quiet;
- the mean specific force since the previous entry;
- the head pose.

| ctrl-imu-take | Rust | PiBridge before | **PiBridge, IMU identity** | IMU-led EKF tracker |
|---|---|---|---|---|
| fix pairs turning against both gyros (mirror / garbage) | 143 | 172 | **0** | 3 |
| wrong-hand pairs | 0 | 0 | 0 | 0 |
| jumps > 5 cm / > 8 cm | 114 / 25 | 171 / 58 | **84 / 9** | 49 / 11 |
| spread at rest (left; the right is never quiet for long) | 25 mm | 116 mm | **3.3 mm** (right 2.0) | 3.3 mm |
| left tracked 0-10 s (the left was not the hand in view) | 95 % | 95 % | **26 %** | 33 % |
| tracked L/R, 10-20 s (crossing) | 74 / 68 % | 74 / 74 % | 64 / 70 % | 54 / 1 % |
| tracked L/R, whole take | 81 / 78 % | 82 / 83 % | 54 / 72 % | 49 / 30 % |

Notes on the table:

- "PiBridge before" is section 1-3's blobs plus the own-model rule. On this take, at the real 40/62 exposure, it jumps
  more than Rust (58 vs 25).
- The IMU identity removes that: every pair turning against the gyro is gone, and 8 cm jumps go from 25 to 9.
- **The cost** is the tracked share (the left most of all). Those were the fixes the gyro calls wrong; the fuser
  holds and carries the hand over the gaps.
- Its own-gyro agreement on good fixes (9+ LEDs) is 1.5 / 3.9 deg p50 (left / right), so a 25 deg gate keeps the
  good fits.

### The IMU-led tracker (`imutrack.h`, built, off: `PIBRIDGE_CTRL_TRACKER=imu`)

The design of Monado's tracking/constellation and OpenHMD's Rift tracker (read for the design; no code copied):

- **State:** per controller, an error-state Kalman filter (orientation, position, velocity, gyro and accelerometer
  biases), turned by the controller's gyro. Optical fits are gated measurement updates.
- **Correspondence:** from the prediction, with a window from the covariance and the predicted orientation as the
  prior. A blob keeps its LED through the prediction.
- **Brute force:** only when the prediction misses. Every hypothesis must agree with that controller's tilt and its
  gyro-carried orientation, and lie within 15 cm + 1 m/s of the prediction.
- **First acquisition:** tentative until 3 updates in a row pass the gate.
- **Identity:** the IMU's.
- **No IMU at all:** the per-frame tracker.

On this take it is worse (table above, tracked 49 / 30 %), so it stays off. Two causes, both measured:

1. **No head pose in the take.** The accelerometer's up and the optics' up, in the head frame, disagree by 18-24 deg
   p50 (35-38 p90), and still by 15-21 deg with the best fixed rotation fitted to them. The head was not level or not
   still. So the accelerometer cannot drive the position in the head frame (the filter's velocity runs off within a
   few frames), and the position runs on constant velocity there (`use_accel = false`).
2. **The saved gyro calibration is 10 deg off a clean axis mapping.** The gyro-carried orientation then drifts
   during gaps, and the acquisition check refuses good fits: 700-1000 refusals on this take.

A take in a session with SLAM (a head pose in the hint file) is needed to tune it, plus a fresh gyro fit.

### The overshoot after a hard stop

The overshoot is the shown pose past where the hand came to rest, along the motion, over 0.6 s after the stop. The
stops are found from the logged good fixes.

| | right, 3 stops (mm) | left, 1 stop (mm) |
|---|---|---|
| shown live (the PiBridge fuser) | -1 / 46 / 104 | 108 |
| replay, Rust fuser | -1 / 30 / 117 | 88 |
| replay, PiBridge fuser | -1 / 47 / 104 (= live) | 108 |
| the optical fixes themselves (per-frame tracker, Rust / PiBridge / PiBridge + IMU identity) | 103 / 113 / 117 (third stop) | 154 / 152 / 152 |

**The overshoot is in the optical fixes, not made by the fuser.** At the third right stop the fixes go 97 mm past
the rest position and come back over 0.3 s. Most of them have 5-8 LEDs with the fuser's `pos-only` verdict: their
orientation turned against the gyro. The fuser adds only about 8 mm to the fixes' peak, and lags them on the way
back (the display filter's low gain for few-LED fixes).

Two fuser changes were tried; neither was better on both hands, so both stay off:

| Change | Right | Left | Default |
|---|---|---|---|
| Drop instead of `pos-only` (`turn_drop`, `PIBRIDGE_CTRL_TURN_DROP=1`) | 104 -> 64 mm | 108 -> 144 mm | off |
| The raw fix as the anchor of the inertial display (`raw_anchor`) | worse | worse | off |

Whether the hand itself swung back cannot be told without a head pose. The next step is the tracker's IMU identity
on a new take: those `pos-only` fits are what it refuses.

## 9. Take 2 with the real head pose (2026-10-19 16:37): the IMU-led tracker is the default

The take: `ctrl-imu-take2.pxcam` and `ctrlimu2-{left,right}.csv`, SteamVR + Monado with SLAM (a real head pose in
the hint file), the same choreography as section 8. The bench feeds the head pose from the logs' O / P lines. The
gyro calibration was not refitted: the filter tracks well with the saved one.

Changes in the IMU-led tracker (`imutrack.cpp`) made on this take:

- the accelerometer drives the position again (`use_accel`, now that there is a head pose), with sigma_a 1.5 and
  measurement sigmas 8 / 16 / 30 mm;
- a tight correspondence window (10 px) is tried before the covariance's;
- the fit is accepted up to the normal 2.5 px rms;
- a search fit for a tracked controller that its prediction missed is a measurement update, not a restart (that
  removed the jumps of section 8: 279 restarts -> 8).

Without a head pose (the hint's stays the identity for 1 s) or without the IMU, `ImuTracker` runs the per-frame
tracker with the IMU hand identity (section 8). `live` uses the IMU-led tracker by default; `PIBRIDGE_CTRL_TRACKER=frame`
turns it off.

| ctrl-imu-take2 | Rust | PiBridge, per-frame + IMU identity | **PiBridge, IMU-led (default)** | IMU-led, its optical fits |
|---|---|---|---|---|
| tracked L / R | 88.5 / 89.6 % | 84.7 / 87.9 % | **86.1 / 91.1 %** | 86.1 / 91.1 % |
| tracked L / R while crossing (10-20 s) | 86 / 84 % | 81 / 86 % | **83 / 88 %** | |
| fix pairs turning against both gyros (mirror / garbage) | 125 | 0 | **0** | 1 |
| wrong-hand pairs | 0 | 0 | 0 | 0 |
| jumps > 5 cm / > 8 cm | 105 / 18 | 75 / 9 | **48 / 4** | 59 / 8 |
| jitter at rest (left), position / orientation | 2.7 mm / 2.5 deg | 3.1 mm / 2.5 deg | **1.9 mm / 0.3 deg** | 3.0 mm / 2.3 deg |
| jitter at rest (right) | 9.3 mm / 7.8 deg | 7.4 mm / 6.7 deg | **2.3 mm / 0.5 deg** | 2.6 mm / 1.6 deg |
| the tracker's output past the rest position after the hard stop (right) | 45 mm | 38 mm | **33 mm** | |

Notes on the table:

- The mirror and wrong-hand rows use the same gyro test the identity filter uses, so read them with that in mind.
  The jumps, the jitter and the left hand's 0-10 s share (section 8) are independent of it.
- "Its optical fits" scores the filter's measurements rather than its output. The output's orientation is partly
  the gyro's.

### The overshoot: the fuser's inertial position, now off

The replay of the real IMU logs (section 6) shows that the inertial position of section 2 (`ins_s` 0.6 s) is worse
on real data than on the synthetic IMU. Without it, on both takes:

| | PiBridge fuser with INS (as live) | PiBridge fuser without INS (new default) | Rust |
|---|---|---|---|
| take 2 right, overshoot at the hard stops | 25 / **87 mm** | 27 / **57 mm** | 27 / 57 mm |
| take 2 right, cut 1 s: error at 0.25 / 0.5 s (p50/p90) | 7/27, 13/60 mm | 3/23, 4/37 mm | 3/23, 5/37 mm |
| take 2 left, cut 2 s: error at 1 s | 13/108 mm | 8/58 mm | 9/58 mm |
| take 1 left, cut 1 s: error at 0.25 s | 12/178 mm | 4/22 mm | 3/20 mm |
| take 1 right, overshoot | -1 / 44 / 105 mm | -1 / 30 / 117 mm | -1 / 30 / 117 mm |

So `Config::pibridge()` now has `ins_s = 0`. The rest of section 2 stays: the tilt fix, no give-up after 4 s, the
1 m/s glide, and the gravity check. The remaining 57 mm is the optical fixes' own swing (45 mm with the old tracker,
33 mm with the IMU-led one) plus the fuser's velocity extrapolation over the 30 ms latency. The IMU now enters the
pose through the tracker's filter instead of the fuser's.

## Follow-ups that need more than offline data

1. **The hint in the tracker** (section 5) is built but off. Decide with a live take: replay its IMU log
   (section 6), and A/B `PIBRIDGE_CTRL_TRACK_HINT=1`.
2. **A closed-loop bench on a real take.** A take with both the IMU log and LED frames can drive `--hint` with the
   real IMU instead of the synthetic one. The `.pxcam` and the log share CLOCK_MONOTONIC.

## Live takes (to become guided steps on the Development page)

The agent records nothing. Each take runs in a normal inside-out session (Monado + `pibridge-ctrltrack live`) with
both controllers awake, and needs about 2.2 GB of free disk per minute.

1. **Controllers with AE on (the real 40 / 62 LED frames).** The session runs with `PIMAX_CAM_AE=1`.
   - Recording: `pibridge-cam record OUT.pxcam --source broker --seconds 60` next to the broker, with
     `PIMAX_CTRL_IMU_LOG=1` (the fuser log of both hands, same clock).
   - Script, 60 s:
     - 0-10 s: both controllers still at chest height in front;
     - 10-25 s: normal pointing with the arms low and to the sides;
     - 25-35 s: fast swings and wrist flicks;
     - 35-45 s: each hand behind the back for about 3 s and back, one after the other;
     - 45-55 s: both controllers put down on the desk in front;
     - 55-60 s: the room light off.
   - Gives:
     - the LED detection at the real AE exposure (section 1);
     - the 0 % case reproduced (desk, dark);
     - a real-IMU fuser replay with true occlusions (section 2);
     - the byte 45 question, if the raw reports are logged as well (a passive report log in the driver, not
       available yet).
2. **The same take with `PIMAX_CAM_AE=0`** (LED frames at 25 / 35): an A/B of the exposure on the same motions.
3. **For a session that shows 0 % again:** 20 s of `pibridge-cam record` from the broker while it happens, then
   `pibridge-ctrltrack scan` on it. It tells "no blobs" (out of view, LEDs off) from "blobs but no fix".
