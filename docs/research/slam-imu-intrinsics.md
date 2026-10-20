# Do the factory IMU intrinsics help Basalt? (Crystal Super, offline study)

Date: 2026-10-19. Offline replays only (Monado `monado-cli slambatch` + Basalt from omarchyvr's fork), one
recording, no ground truth. Scratch data, scripts and every run's config: `~/gamedev/ext/slam/study/` and
`~/gamedev/ext/slam/ab/` (each run folder has a one-line `README`).

## Short answer

(Update 2026-10-19: a second recording with six still poses, and the live session that ran on it, are in
[the section at the end](#second-recording-six-poses-on-the-desk-live-vs-offline-2026-10-19): `ka` is now ruled out on the x axis
too, and the live divergence there came from images with too little to track plus a live build without the accel bias.
A third, worn take with auto exposure on is in [the last section](#third-recording-worn-guided-protocol-auto-exposure-on-2026-10-19-0704):
Basalt now tracks well; the output layer's drift hold is what still costs 20 to 30 cm.)

- **Use the factory accelerometer bias `aBias` as is (body frame, subtract).** It is clearly right: it brings the
  measured gravity from about 3 % too high to within 0.3 % of g, and it is the only term the data asks for.
- **Do not apply `ka` (accel scale) in either convention.** Both `meas/(1+k)` and `meas*(1+k)` make gravity wrong by
  1 to 2 % (9.64 or 9.92 m/s² instead of about 9.8), and a free fit of the scales on this recording gives about 0.
- **Do not apply `na` (accel misalignment).** Every plausible layout doubles or triples the spread of |gravity|
  across head poses; a free fit of the y-z coupling gives 0.
- **`kg`, `ng`, `ombg` (gyro scale, misalignment, gyro-to-accel rotation) are too small to matter** (at most about
  0.3 % and 0.15 degrees). Not worth implementing.
- **`wBias` (gyro bias) is in the body frame as is, no axis flip.** It matches the recording's own gyro bias on y and
  z; on x it is off by about 1 deg/s (the bias moves with temperature). A bias measured at run time beats it.
- **On the VIO trajectory, none of these terms makes a measurable difference** (path, box, start-to-end, drift
  while nearly still, speed outliers all within 1 %). Basalt's online bias estimate absorbs constant errors of
  this size. The terms show up only in how well the IMU agrees with the trajectory (see the table).
- **Side finding: Basalt ignores `cam_time_offset_ns` in `calib.json`** (the line that applies it is commented out
  in `sqrt_keypoint_vio.cpp`). Any camera/IMU time offset has to be applied by shifting timestamps in the driver
  (omarchyvr does this with `PIMAX_SLAM_CAM_OFFSET_US`).

Conclusion in one line: the factory `ka/kg/na/ng/ombg` look like states of Pimax's own filter, or terms the firmware
already applies to the IMU stream; they do not describe the raw stream Basalt sees. Only `aBias` (and, as a
fallback, `wBias`) should be used.

## The recording

`euroc_20261018024922`: Crystal Super with the Lighthouse faceplate, user seated, head moving for 236 s; two
tracking cameras (640x480, 30 Hz in the replay), IMU at 1 kHz in the body frame (OpenXR head axes = raw IMU with
x and z negated). No ground truth.

Is it good enough? **Only partly.** The head was almost never still: there are no stretches of at least 1.5 s
with the gyro below 0.02 rad/s, only five stretches of 1.7 to 3.3 s (11 s in total) at a looser threshold. The
head stayed nearly upright (gravity on the y axis between 83 and 100 %, never on x beyond 12 %), so the x-axis
scale (the biggest `ka`, about 5 %) cannot be observed at all, and y/z only weakly. It does not return to a
marked start point, so start-to-end distance is not a drift number. It is good enough to reject the `ka` and `na`
conventions (gravity magnitude), not good enough to measure a small true scale error.

## Method

1. **Static gravity test (no VIO).** 0.5 s windows with little rotation (295 windows; 132 with the strict
   threshold): the mean specific force should have magnitude g whatever the head pose. Each accel model is judged
   by its mean |a| (should be about 9.78 to 9.83, the range of g on Earth) and by its spread across poses.
2. **Gyro bias.** The whole-recording mean of the gyro (net rotation over 236 s is negligible), compared with
   `wBias` as is and with x, z flipped; also omarchyvr's image-based estimate (`omarchyvr-slam camlag`).
3. **VIO replays**, one at a time at `nice -n 19`, stopped automatically whenever SteamVR or Monado was running.
   Runs are deterministic (`deterministic=1`; a repeat of B gave a byte-identical trajectory), so every
   difference below is real, not noise. Metrics, all without ground truth:
   - path length, bounding box, start-to-end distance, speed p99 / max;
   - drift while nearly still: the VIO's speed inside the quasi-static windows, and its position change across
     the five still stretches;
   - tilt error: angle between the VIO's "up" in the head frame and the measured gravity direction in the
     quasi-static windows (reference: accel minus `aBias`);
   - IMU/trajectory consistency: over 1 s windows, the VIO's change of velocity against the integral of
     (R * calibrated accel - g), with each run's own IMU calibration; the vertical mean shows a gravity/scale error;
   - Basalt's log (no resets, no lost tracking, no errors in any run).

## Static results (accelerometer)

| accel model | mean \|a\| m/s² | spread across poses mm/s² |
|---|---|---|
| raw | 10.11 | 25 |
| **B: a - aBias** | **9.78** | **21** |
| C1: a/(1+ka) - aBias | 9.64 | 21 |
| C2: a*(1+ka) - aBias | 9.92 | 22 |
| (bias before scale: same within 0.005) | | |
| B + na, layouts lower / upper / symmetric | 9.73 to 9.89 | 39 to 78 |

Free fits with g fixed to 9.78 / 9.806 / 9.83: the y bias comes out at 0.35 / 0.32 / 0.30 m/s², i.e. the factory
value (about 0.34) to within 0.03; the y and z scales come out at 0.0 to +0.6 %, not at the factory 1.2 to 1.4 %.
The best y-z coupling term is 0.000.

## Gyro bias

The recording's own gyro bias is about 1.5 deg/s on z, small on x and y. `wBias` as is matches it on y and z to
about 0.2 deg/s and is off by about 1 deg/s on x; with x and z flipped (as the first C1/C2 data sets did) it is off
by about 3 deg/s, i.e. **the flip is wrong**. The factory calibration's body frame is the frame the IMU stream is
already in, for `wBias` exactly as for `aBias` (omarchyvr uses `aBias` the same way).

## VIO results

Variants: A = no accel bias; B = factory `aBias` (omarchyvr today); C1/C2 = lead's data sets (IMU rewritten:
gyro minus `wBias` flipped, scales `ka`, `kg` as /(1+k) or *(1+k); note: the gyro bias was then subtracted twice);
C1f/C2f = the same scales done cleanly as Basalt's `calib_accel_bias` / `calib_gyro_bias` matrices on the original
data; Gw = B with `wBias` as is as the gyro bias prior; G0 = B with no gyro bias prior; Dp/Dm = B with the
camera/IMU time shifted by +/- the factory `delta` (about 1.6 ms). All runs start from B's calibration and the
provisional gyro bias omarchyvr measured, except where stated.

| variant | path m | start-end m | speed p99 / max m/s | speed when nearly still mean / p95 mm/s | still-stretch drift mm (5 stretches) | tilt error median / p95 deg | IMU vs trajectory: dv rms / vertical mean (m/s per s) |
|---|---|---|---|---|---|---|---|
| A (no aBias) | 18.39 | 0.398 | 0.434 / 0.87 | 8.4 / 20.4 | 37.0 | 0.75 / 1.79 | 0.379 / -0.298 |
| **B (aBias)** | 18.43 | 0.404 | 0.434 / 0.87 | 8.4 / 20.1 | 37.6 | 0.82 / 1.37 | **0.184 / +0.034** |
| C1 (lead data, /(1+k)) | 18.41 | 0.399 | 0.438 / 0.87 | 8.5 / 21.3 | 36.3 | 0.73 / 1.21 | 0.243 / +0.176 |
| C2 (lead data, *(1+k)) | 18.36 | 0.396 | 0.433 / 0.85 | 8.6 / 21.0 | 37.4 | 0.90 / 2.80 | 0.255 / -0.110 |
| C1f (B + ka,kg as /(1+k)) | 18.39 | 0.397 | 0.437 / 0.87 | 8.3 / 20.5 | 37.1 | 0.74 / 1.33 | 0.248 / +0.176 |
| C2f (B + ka,kg as *(1+k)) | 18.37 | 0.396 | 0.433 / 0.85 | 8.5 / 20.7 | 37.4 | 0.90 / 2.80 | 0.255 / -0.110 |
| Gw (B, gyro prior = wBias) | 18.37 | 0.394 | 0.434 / 0.87 | 8.4 / 20.2 | 37.2 | 0.80 / 1.95 | 0.191 / +0.034 |
| G0 (B, no gyro prior) | 18.39 | 0.392 | 0.434 / 0.87 | 8.4 / 20.3 | 37.2 | 0.82 / 1.96 | 0.196 / +0.034 |
| Dp (B, IMU earlier by delta) | 18.40 | 0.386 | 0.435 / 0.86 | 8.7 / 21.1 | 37.4 | 0.74 / 1.97 | 0.185 / +0.034 |
| Dm (B, IMU later by delta) | 18.31 | 0.413 | 0.433 / 0.84 | 8.4 / 20.5 | 37.8 | 0.82 / 2.06 | 0.196 / +0.034 |
| B repeat | identical to B (byte for byte) | | | | | | |

How to read it:
- The trajectory columns barely move (path within 0.6 %, start-to-end within 2 cm, drift while still within 1 mm
  per stretch). On this recording none of the intrinsics changes the tracking in a way we can see.
- The last column separates the models: with the right accel model the IMU, rotated by the VIO's attitude, agrees
  with the trajectory. B is best (vertical error 0.03 m/s²). Without `aBias` (A) the IMU disagrees by 0.3 m/s²;
  with `ka` it disagrees by 0.18 (C1) or 0.11 m/s² (C2), the same 1 to 2 % gravity error the static test shows.
  Basalt hides this from the trajectory by moving its own bias estimate, which is why the trajectory does not
  change, but a wrong prior costs convergence time after every start and reset.
- `*(1+k)` (C2/C2f) has the worst tilt tail (2.8 deg p95). `/(1+k)` has a slightly better tilt tail than B (1.2 to
  1.3 against 1.4 deg) but the tilt reference itself uses `aBias` only, so this is within what the metric can tell.
- The lead's C1/C2 (double gyro bias) and the clean C1f/C2f agree closely: Basalt's gyro bias estimate converges
  fast, so a gyro bias prior error of 1 to 3 deg/s does not show on this recording either.
- The gyro bias prior: the measured provisional bias (B) gives the best tilt tail (1.4 deg p95); `wBias` (Gw) and
  no prior at all (G0) are equal to each other (about 2.0 deg) and slightly worse. Small, but it favours a bias
  measured at run time over the factory value.
- The factory `delta` (about 1.6 ms, applied either way by shifting the IMU timestamps): no measurable effect; one
  sign gives a slightly better tilt median, both give a worse tilt tail than B. omarchyvr's camlag also finds the
  camera/IMU lag at 0.0 ms with a flat minimum from -2 to +2 ms, so this recording cannot resolve 1.6 ms.

## What PiBridge should implement

Accelerometer, per sample, in the body frame (raw IMU with x and z negated):

    a_basalt = a_body                       (no scale, no misalignment)
    calib_accel_bias = [aBias_x, aBias_y, aBias_z, 0, 0, 0, 0, 0, 0]   (Basalt subtracts it; aBias used as is)

Gyroscope:

    w_basalt = w_body                       (no kg, ng, ombg)
    calib_gyro_bias = [b_x, b_y, b_z, 0 x 9]
    b = gyro bias measured at run time while the headset is still (omarchyvr's provisional bias / cache);
        fallback when none is known yet: wBias as is (body frame, no flip); last resort 0.

Time: leave the factory `delta` out (see the delta row), and remember that a camera/IMU offset must be applied to
the timestamps before Basalt, never through `cam_time_offset_ns`.

Do not implement `ka`, `kg`, `na`, `ng`, `ombg` now. Keep reading them from the XML and log them, so a later test
can switch them on. If a future still-orientation test shows a real scale error, Basalt can take it without data
rewriting: accel scale and misalignment go into `calib_accel_bias[3..9]` (lower-triangular, `a_cal = (I + S) a - b`),
gyro scale/misalignment into `calib_gyro_bias[3..12]` (full 3x3, column-major, `w_cal = (I + S) w - b`).

Distortion: the factory fisheye model has 6 numbers; Basalt's KB4 takes 4 (k1 to k4). The last two are not used
by any Basalt camera model as such; not tested here (note only). Separately, omarchyvr's camlag reported an
image-to-gyro rotation gain of about 1.09, which is too large to be a gyro scale error (`kg` is at most 0.3 %) and
more likely a property of camlag's image model; worth a look when the intrinsics are checked.

## Next live test (new recording, without the Lighthouse faceplate)

Record with the PiBridge recorder, cameras and IMU as now, in a lit room with texture:

1. **Six still poses, 10 s each**, headset on a table, not on the head: face down, face up, left side, right side,
   top up, top down. This alone settles `aBias` and `ka` per axis (and `na`), without the VIO.
2. **Still on the head, 10 s**, looking straight ahead; mark the spot on the floor and the chair position.
3. **Slow moves** away from the mark and back, as far as the room allows (1 to 2 steps, or a lean of about 0.5 m
   forward, back and to each side), 20 s, eyes on the room (not a wall). The distance does not matter: the loop
   closes at the mark, and the drift is measured there.
4. **Return to the start mark** and hold still 10 s: the start-to-end distance then is a real drift number.
5. **Turns**: slow yaw left and right +/- 90 degrees, then pitch up/down +/- 45 degrees, then roll +/- 30 degrees,
   2 s per sweep, each followed by 3 s still.
6. **Fast head shakes** for 5 s (latency/timing, `delta`), then still 10 s at the start mark again.

Total about 3 minutes. With steps 1, 2, 4 and 6 we get still stretches, gravity in all directions and a closed loop.

## Files

- Scripts (`~/gamedev/ext/slam/study/`): `imustill.py` (still stretches, model helpers), `gravwin.py` (static
  gravity test), `fits.py` (gyro bias, accel fits), `namis.py` (misalignment layouts), `vio_metrics.py` and
  `table.py` (VIO metrics), `mkvariant.py` (variant configs), `run_variant.sh` (guarded slambatch runner),
  `queue.sh`.
- Outputs: `static-gravity.txt`, `static-fits.txt`, `static-na.txt`, `table.md`, per-run `metrics.json`.

---

# Second recording: six poses on the desk, live vs offline (2026-10-19)

Recording `euroc_20261019052737` (192.7 s, Crystal Super **without** the Lighthouse faceplate; 2 cameras at 30 Hz,
IMU at 1012 Hz, `omarchyvr-slam check` OK), made while a live Monado session ran Basalt on the same data (no
picture: the display lease was withdrawn, tracking ran). Scratch, scripts, every run and its config:
`~/gamedev/ext/slam/study2/` (`table.md`, `static-fit6.txt`, `timeline-1s.txt`, `imgstats.txt`, `runs/*/README`).

## Short answer

- **Live and offline agree. The live divergence is not a live-pipeline problem.** Replaying the same data through
  `slambatch` with the live session's exact calibration and config runs away in the same place (1.8 m in the first
  3 s, then 1.2 to 1.8 m per desk still), and the still gate replayed on that trajectory fires 42 times; the live
  log has 36 tracker resets in the same windows. No frames were lost, no IMU was held back, the tracker and the
  recorder got the same samples with the same timestamps, and `deterministic=0` (live) gives the same output as
  `=1` byte for byte.
- **The cause is vision starvation in the desk pose, made worse by a stale live build.** For the first 80 s the
  headset lay on the desk with dark images (mean 14 to 30 of 255; the Crystal runs with auto exposure off) and
  little texture (4 to 5 % of sampled pixels with an edge, against 8 to 12 % in the side poses). Basalt then
  integrates the IMU almost alone. Once the headset was picked up, the live configuration tracks well (1.4 mm/s median drift in
  the still poses, tilt within 0.5 degrees), and no variant runs away there.
- **The live monado-service was built before the factory accel bias went in** (built 2026-10-17 23:05; the
  `calib_accel_bias` commit and the drift hold came on 2026-10-18). The session wrote `calib_accel_bias = 0`, so the
  live run was variant A of the first study. With zero bias the starved stretches run away fastest (1.8 m in 3 s),
  and with zero bias plus a finer detection grid the replay blew up completely (tens of km). The marker
  `PIMAX_SLAM_DRIFT_HOLD` is missing from that binary as well.
- **A finer Basalt detection grid (40 to 30 px) is the one config change that clearly helps** in the starved stretch
  (with a non-zero accel bias): desk drift down from 0.5 to 2.7 m per stretch to 0.002 to 0.8 m, the still gate
  replayed on the trajectory fires 20 to 37 times instead of 41, and the moving part does not change. The FAST threshold (5 to 3) changed nothing (same output
  byte for byte).
- **The six-pose fit settles the intrinsics question.** The x-axis scale is observable now and is about -0.3 %,
  **not** the factory `ka` (about +5 %): applying `ka` in either convention puts gravity 0.4 to 0.5 m/s² off in the
  x poses. The fitted scales are all within -0.5 %. The fitted bias differs from the factory `aBias` by about
  0.1 m/s² on y (the bias moves with temperature and over time). Factory `aBias` alone brings the gravity error
  across the poses from 175 down to 58 mm/s² rms; the full fit gets 3 mm/s². In the VIO the difference between
  factory `aBias` and the fit cannot be seen on this take (the starved stretch is chaotic, see below).
- **Gyro bias**: stable over the whole take (the spread between still windows is about 0.02 deg/s per axis). The
  driver's bias refined at run time matches it exactly. The cached bias the session wrote into Basalt was off by about
  0.3 deg/s on x, and the factory `wBias` by about 1.1 deg/s on x, as in the first study.

## What is in the take (segmented from the IMU)

The user did the protocol "as it came out". It is mostly a desk test:

| time s | what | gravity in body axes | notes |
|---|---|---|---|
| 0 to 80 | lying still on the desk, top up, pitched about 13 degrees | +y 0.97, -z 0.23 | small knocks at 36 to 39 s and 57 to 62 s; at 38.6 s the image goes dark for a moment (a hand or a person in front) |
| 80 to 100 | picked up and moved by hand or on the head, 20 to 40 deg/s | mostly +y | the only "head" part; no clean steps, sweeps or shakes |
| 103 to 111 | face up (lenses to the ceiling), held or resting, not perfectly still | -z | gyro spread 1.5 deg/s |
| 116 to 123 | face down | +z | gyro spread 1.3 deg/s |
| 128 to 138 | on its left side | +x | truly still |
| 143 to 150 | on its right side | -x | truly still |
| 161 to 168 | top down, tilted by about 24 degrees | -y 0.91, +z 0.41 | still |
| 169 to 177 | turned over (fast, up to 115 deg/s) | | |
| 177 to 193 | top up again, nearly still (small vibration) | +y 0.99 | |

There is no marked spot on the head, no step and back, and no yaw, pitch or roll sweep. The protocol's "drift at
the mark" cannot be measured: the closest substitute is the start and end desk poses, which differ by 5 degrees of
tilt, so the headset was not put back exactly in the same place.

## Static fit (no VIO)

One mean per pose (7 poses; the top-up pose twice). Model as Basalt applies it: `a_cal = (I + S) a - b`, S diagonal.
g = 9.81 ± 0.005 m/s² (mid-latitude Europe at low altitude; Basalt uses 9.81). The value of g only moves the three
scales together, by 0.05 % per 0.005 m/s², so it cannot hide a 1 to 5 % `ka`.

| model | gravity error per pose, rms mm/s² | worst pose mm/s² |
|---|---|---|
| raw | 175 | +296 (top up) |
| factory aBias (B) | 58 | +123 (top down) |
| aBias + ka as a/(1+k) (C1) | 264 | -472 (x poses) |
| aBias + ka as a*(1+k) (C2) | 302 | +535 (x poses) |
| B + na, any of 8 layouts | 57 to 118 | no layout helps |
| free bias only (3 terms) | 31 | +56 |
| **free bias + 3 scales (6 terms)** | **3** | 6 |

- Fitted scales: x about -0.3 %, y about -0.5 %, z about -0.2 % (z is the weakest: the face-up and face-down poses
  were not perfectly still). The factory `ka` is +5.0 / +1.4 / +1.2 %. Neither its magnitude nor its sign fits.
- Fitted bias against factory `aBias`: x and z agree within 0.03 m/s²; y is about 0.1 m/s² smaller. The first
  recording's y fit (bias only) came out within 0.03 of the factory value, so the bias has moved (temperature or age).
- Misalignment is **not observable** with these poses: 6 parameters from 7 poses, two of them nearly the same,
  leave no redundancy (leave-one-out fits fall apart when either x pose is dropped). The factory `na` makes nothing
  better in any layout.
- A free fit of g with the scales is degenerate (only g times the scale enters), hence g is fixed.

## Gyro bias in the still windows

The bias is constant within about 0.02 deg/s across 11 still windows over 190 s; z is about 1.35 deg/s. The
driver's bias refined at run time ("gyro bias refined while running") matches it to 0.01 deg/s. The cached bias the
session put into Basalt's `calib_gyro_bias` was about 0.3 deg/s off on x; factory `wBias` was about 1.1 deg/s off on x
(as in the first study). Feeding the measured bias (variant Fsg) did not change the picture on this take.

## Offline replays

Same harness as the first study (`monado-cli slambatch`, `nice -n 19`, guarded so it stops itself if SteamVR or
Monado starts). The base is the **live session's own** `pimax-basalt-calib.json` and `pimax-basalt.toml`.
Metrics without ground truth, times from the first camera frame: drift over each desk still stretch, a simple
emulation of the still gate (moves of more than 3 cm, at least 1 s apart), drift and tilt in the six still poses,
start pose against end pose, and IMU against trajectory (1 s windows, velocity change against the integrated
calibrated accel), for the whole take and for the clean part 100 to 172 s.

| run | what | path m | first 1 s / 3 s mm | desk stills drift mm (0-37 / 40-56 / 64-78 s) | gate trips on the desk | pose drift mm/s median (max) | tilt max deg desk / poses | IMU vs trajectory rms, clean part m/s² |
|---|---|---|---|---|---|---|---|---|
| L | live calib exactly (accel bias 0), grid 40 | 16.4 | 190 / 1805 | 1823 / 1665 / 1156 | 48 | 1.4 (2.3) | 0.16 / 0.52 | 0.280 |
| Lnd | L with deterministic=0 (as live), twice | identical to L, byte for byte | | | | | | |
| B | factory aBias | 17.8 | 106 / 765 | 466 / 1202 / 2713 | 55 | 2.2 (6.2) | 0.64 / 0.53 | 0.149 |
| Fb | fitted bias only | 17.7 | 133 / 249 | 202 / 931 / 2847 | 46 | 4.6 (18.9) | 0.31 / 0.54 | 0.113 |
| Fs | fitted bias + scales | 25.9 | 120 / 86 | 89 / 3090 / 5602 | 44 | 27.8 (56.1) | 0.25 / 0.46 | 0.163 |
| Fsg | Fs + gyro prior = measured bias | 23.3 | 142 / 76 | 1078 / 1305 / 3797 | 55 | 7.9 (21.4) | 0.19 / 0.45 | 0.110 |
| GtB | B + FAST min threshold 3 | identical to B, byte for byte | | | | | | |
| **GdB** | **B + detection grid 30** | 10.4 | 31 / 187 | 120 / 38 / 835 | 32 | 1.7 (3.3) | 0.28 / 0.62 | 0.161 |
| **Gd** | **Fs + detection grid 30** | 6.8 | 14 / 55 | 67 / 76 / 2 | 10 | 1.6 (4.0) | 0.45 / 0.77 | 0.147 |
| GdL | live calib (bias 0) + grid 30 | 45 km | 394 / 3111 | diverged | 72 | diverged | | |

The still gate itself (`omarchyvr-slam stillgate`, current pimax-cam code) replayed on each trajectory: L 42 fires,
B 41, GdB 37, Gd 20, GdL 78 with 23 losses.

How to read it:
- **Every variant fails on the desk, and none runs away after the pick-up** (pose drift 1.4 to 8 mm/s, one
  outlier: Fs at 28 mm/s, which carried the desk divergence over). In the starved stretch Basalt is
  chaotic: small changes (the bias prior, one grid step) flip the result between centimetres and metres, so single
  rows must not be over-read. Two things hold in every pair: zero accel bias is always the worst start (1.8 m in 3 s
  with grid 40, total divergence with grid 30), and the 30 px grid with a non-zero bias always lowers the desk drift
  and the gate trips.
- **The gravity/scale model shows only in the IMU/trajectory column of the clean part**: without accel bias 0.28,
  with any bias 0.11 to 0.16 m/s². Factory aBias against the fit: not separable on this take.
- **The fresh start while still is Basalt's weakest moment.** L moves 0.19 m in its first second with the headset on
  the desk. The live still gate (3 cm, then a reset at most every 1 s) therefore re-zeroed every second from 32 to
  58 s and from 70 to 80 s (32 times): each fresh tracker drifted 3 cm before it had a usable map, was reset, and so
  never left its worst phase. The output was held the whole time (lesson 36: these are not losses), but the tracker
  could not settle until the headset moved.

## Live session against the replay

| | live (Monado log) | offline replay of the same data (L) |
|---|---|---|
| Basalt calib / config | `calib_accel_bias` 0, cached gyro bias, grid 40, deterministic 0 | identical (deterministic 0 and 1 give the same output) |
| frames | camera 60 fps, tracker 30.0 Hz, 0 lost, 0 corrupt, 0 untimed | 5781 frame sets, 33.33 ms steady, no gap |
| IMU | 1012 Hz, 124 samples held before the first frame, 0 camera gaps until shutdown | the same samples (the recorder and the tracker get the same sample and frame objects; the mid-exposure shift is applied before both sinks) |
| 0 to 3 s | re-zeros at 0.7 s and 2.8 s | 0.19 m after 1 s, 1.8 m after 3 s |
| 3 to 32 s | no event (the output layer's own gyro bias starts at 0 and needs about 30 s to learn the 1.35 deg/s; until then the still test does not pass, so the gate stays off) | slow return and about 1 mm/s drift |
| 32 to 80 s | 32 still re-zeros, 3 times "rotation disagrees with the gyro", 1 "position jump" with a 3 s loss | 1.2 to 1.8 m of drift per stretch, worse after the image went dark at 38.6 s |
| 80 to 193 s | 1 re-zero (150.5 s), otherwise tracking | 1 to 2 mm/s in the still poses, tilt within 0.5 degrees |
| 6DoF output corrections (tilt, fusion) | 0.0 mm / 0.00 deg in every stats line | not applicable |

So the output layer (`PIMAX_HEAD_TILT`, `PIMAX_FUSE_AUTO`) played no part here, nor did clocks, frame drops,
queueing or CPU load. The live behaviour is what Basalt does with this data and this calibration, plus the reset loop.

## Recommendation for the live pipeline (most effect first)

1. **Install a build of the current HEAD before the next test.** The running monado-service (2026-10-17 23:05) lacks
   the factory `calib_accel_bias` (pimax-cam 6c4b4c6), the drift hold (8806780; marker `PIMAX_SLAM_DRIFT_HOLD`) and
   the fork commits "gyro bias only from a real rest" and "provisional bias" (54c99958c, 0dff9fa2a; the fork's
   working tree is on 78237e1b3 plus the Crystal patch). Check after the install that the session's
   `pimax-basalt-calib.json` has a non-zero `calib_accel_bias` and that the binary has the marker. Without the
   bias, every stretch with few landmarks runs away about twice as fast, and with a finer grid it can blow up.
2. **Make the images trackable.** The Crystal runs with auto exposure off (`PIMAX_CAM_AE=0`, its exposure command is
   not in the allowlist yet): mean brightness 14 to 30 of 255. The exposure/gain frame from a Windows capture,
   added to the Crystal's allowlist, is the fix at the root. Until then, test in a well-lit room, and do not
   judge the tracking on a headset lying on the desk facing a plain surface.
3. **Basalt detection grid 40 to 30 px** (`omarchyvr-slam` export `OVERRIDES`, `config.optical_flow_detection_grid_size`;
   PiBridge's export the same), only together with step 1. On this take it is the only config change that helped the
   starved stretch. The moving part did not change. Confirm on one more take with head motion before shipping.
4. **Back off the still re-zero** (1, 2, 4 ... 16 s while the headset stays still) and say in the app "the cameras see
   too little to track the position" after 3 re-zeros in a row. A reset every second keeps a starved tracker in its
   weakest phase. Proposal only, not tested (diff in the study's scratch).
5. **Accelerometer intrinsics**: keep factory `aBias`, and never `ka`/`na` (now also ruled out on the x axis). A
   per-headset six-pose calibration (the app asks the user to lay the headset on six sides, 10 s each) would fit bias
   and scale to 3 mm/s² instead of 58. It is cheap and correct, but on this take it gave no visible VIO gain. Low
   priority.
6. **Seed the output layer's gyro bias** with the driver's known bias (cached or refined), so the still test works
   from the first second instead of after about 30 s. Small, mainly a consistency fix.

## Next take (what this one still lacks)

The head part of the protocol: on the head at a marked spot, step or lean about 0.5 m and back, slow yaw, pitch
and roll sweeps, a 5 s shake, 10 s still at the mark, in a well-lit room, after step 1 above. The desk part does
not need repeating; the six poses are done.

---

# Third recording: worn, guided protocol, auto exposure on (2026-10-19 07:04)

Recording `euroc_20261019070409` (121 s, PiBridge's Tracking page, guided, all 15 steps done, none skipped; cameras
25 Hz, IMU 1024 Hz, `check` OK), Crystal Super worn, with camera auto exposure on, Monado at the fork's HEAD plus the
Crystal patch (drift hold on), Basalt with the factory accel bias, the measured gyro bias and detection grid 30.
Segmentation from `steps.json` (CLOCK_MONOTONIC, the dataset's clock); still and tilt-hold windows start after
the step's settle time. Scratch, scripts, every run and its config: `~/gamedev/ext/slam/study3/` (`README`,
`table.md`, `outsteps-L.txt`, `runs/*/README`). No ground truth: "drift at the mark" is the distance between the
mean positions in step 2 (stand on your mark), step 9 and step 15 (back on the mark). Between 9 and 15 the user
did not leave the mark (turn, nod, shake), so 9 to 15 is the cleanest drift number.

## Short answer

- **The images are fixed.** Mean brightness 26 → 64 of 255 (median; range 43 to 81), and the share of pixels
  with an edge on 4x4-binned images (noise removed) went from 2 to 7 % to 22 to 29 %. Pixel noise went up from about
  1 to 3.5 grey levels (more gain), and the signal-to-noise ratio is about the same. AE held lower 5.2 to 6.2 ms / gain
  215 to 249 and upper 6.4 to 6.7 ms / gain 261 to 288, image mean 55 to 72.
- **Basalt itself now tracks a worn headset well.** Live: 1 acquisition, 0 losses, 0 restarts, 0 still re-zeros,
  0 lost or corrupt frames in 121 s. Offline, with the session's config: still drift 2.5 to 4.7 mm/s (including the
  body's own sway), drift at the mark 52 mm between steps 9 and 15 (40 s of turns, nods and a shake), 66 to 73 mm
  from step 2 to steps 9 and 15 (after a step away with a 0.6 x 0.4 m box), no speed outliers (max 1.7 m/s
  while putting the headset on, 1.06 m/s in the shake), tilt error median 1.0 degrees (max 2.6 while the head moved).
- **What breaks now is the output layer's drift hold.** Live it "held" 199 cm of tracker path in 121 s, most of
  it in the step away (39 + 41 cm in 40 to 60 s). Replayed on the offline trajectory it holds 256 cm with the same
  10-second profile and leaves a lasting offset: drift at the mark 273 / 231 mm with the hold, against
  74 / 69 mm without (steps 2 to 9 / 2 to 15). A slow step or lean has too little acceleration spread for the governor,
  so it is held as drift. The hold does help in the stills (back on the mark 10 mm against 35 mm, end 8 against 21 mm),
  but the cost is larger. No parameter setting tried fixes it (see the table).
- **Grid 30 stays.** Grid 40 drifts more at the mark (210 / 170 mm against 73 / 66) and agrees less with the IMU (0.33
  against 0.24 m/s²). The accel bias (0, factory, last take's six-pose fit) does not change the trajectory (to within
  1 mm at the marks), only the IMU/trajectory agreement: 0.37 / 0.24 / 0.22 m/s².
- **The still gate never fired** (live: 0 re-zeros; replay: 0 still periods): the headset was worn, never desk-quiet.
  The back-off patch (`study2/02-still-gate-backoff.diff`) cannot be tested on this take. It matters only for a
  headset lying on a desk.
- **Recorder bug:** the take's own `pimax-basalt-calib.json` and `pimax-slam-info.json` say
  `calib_accel_bias = 0`, but the live tracker used the SLAM_CONFIG calibration with the factory bias (the log
  says "Using sensor calibration provided by the SLAM_CONFIG file"). A replay from the take's folder would silently
  run variant A0. The recorder should copy the calibration the tracker really used.

## Live log per step (10 s drift-hold reports)

| time s | steps | live held cm | replay held cm (L) |
|---|---|---|---|
| 0-10 | 1 put on, 2 start | 27 | 18 |
| 10-20 | 2 still on the mark | 8 | 10 |
| 20-30 | 3-4 tilt left/right | 15 | 27 |
| 30-40 | 5-6 look down/up | 16 | 26 |
| 40-50 | 7 still ahead, 8 step away starts | 39 | 44 |
| 50-60 | 8 step away | 41 | 42 |
| 60-70 | 8 end, 9 back on the mark | 16 | 17 |
| 70-80 | 9 still on the mark | 8 | 12 |
| 80-90 | 10 turn | 8 | 20 |
| 90-100 | 11 hold, 12 nod | 10 | 15 |
| 100-110 | 13 hold, 14 shake | 7 | 15 |
| 110-120 | 15 back on the mark | 4 | 6 |

The 48 log lines that match restart/lost/re-zero are the stats lines ("0 lost"), no event (lesson 36). The gyro bias
of the output layer starts at 0 and takes about 40 s to reach the 1.3 deg/s on z the driver already knows (cached
bias used by Basalt). The driver's rest-window bias refinement found no rest (worn), so it kept the cached value.

## Offline replays (session config = L)

| run | path m | v p99 / max m/s | mark 2-9 / 2-15 / 9-15 mm | still steps mm/s (2,7,9,11,13,15) | tilt holds mm/s (3,4,5,6) | tilt median / max deg | IMU vs trajectory rms: all / step away / shake m/s² |
|---|---|---|---|---|---|---|---|
| **L** (session: factory aBias, grid 30) | 9.22 | 0.68 / 1.69 | 73 / 66 / 52 | 3.3 9.9 4.2 4.7 3.7 2.5 | 32 9 36 4 | 0.97 / 2.63 | 0.244 / 0.34 / 0.55 |
| G40 (grid 40) | 9.00 | 0.68 / 1.73 | 210 / 170 / 41 | 3.7 9.3 4.0 5.3 3.0 3.0 | 37 7 36 3 | 1.00 / 3.02 | 0.326 / 0.55 / 0.59 |
| A0 (accel bias 0) | 9.22 | 0.68 / 1.69 | 73 / 65 / 52 | 3.3 9.9 4.2 4.7 3.5 2.5 | 32 9 36 4 | 0.98 / 2.70 | 0.372 / 0.45 / 0.61 |
| F6 (05:27 six-pose fit) | 9.23 | 0.69 / 1.69 | 73 / 66 / 52 | 3.3 10.0 4.1 4.7 3.5 2.6 | 32 9 36 4 | 0.89 / 2.51 | 0.224 / 0.31 / 0.55 |

The tilt holds 3 (left) and 5 (look down) were not held still (gyro 21 deg/s rms in the hold window), so their
32 to 36 mm/s are real head motion. The shake is the worst IMU/trajectory agreement (0.55 m/s²); a camera/IMU time
offset would show there first. Worth a `camlag` run on this take.

Output layer replayed on L's trajectory (copy of `omarchyvr-slam stillgate` that dumps the output, driver defaults):

| drift hold | held m | mark 2-9 / 2-15 / 9-15 mm | step away: held cm, output start-end mm |
|---|---|---|---|
| off | 0 | 74 / 69 / 71 | 0, 61 |
| **on (default, as live)** | 2.56 | **273 / 231 / 63** | 86, 292 |
| on, floor 3 cm/s | 0.65 | 189 / 148 / 80 | 33, 173 |
| on, noise 0.05 | 2.15 | 302 / 264 / 48 | 80, 308 |
| on, gain 2 s | 2.43 | 250 / 213 / 61 | 81, 272 |
| on, band 0.02-0.06 m/s | 2.56 | 274 / 232 / 63 | 85, 293 |

## Accelerometer (still and tilt-hold windows, worn)

8 usable windows (tilt left and look down dropped: the head moved). Gravity reached x -0.73 (tilt right) and z -0.91
(look up), y 0.4 to 1.0; one-sided on x and z, so the **scales are not observable** (a 6-term fit is degenerate: y scale
+25 %). Bias only, g = 9.81: about (-0.07, +0.31, -0.04) m/s², residual 2 mm/s². The factory `aBias` leaves 31 mm/s² rms
(|g| 30 to 38 mm/s² too low upright: its y is about 0.035 too large here); the 05:27 six-pose fit leaves 23. The y
bias was about 0.24 at 05:27, 0.31 at 07:04 and 0.34 in the factory file: it moves by about 0.1 m/s² (warm-up). It
does not show in the trajectory. Keep factory `aBias`.

## Verdict and next fixes

Inside-out 6DoF on the Crystal Super is **usable when worn, with good light and AE on**, as far as Basalt is
concerned: no losses, no resets, 5 to 7 cm of drift at the mark over 40 to 100 s, 3 to 5 mm/s while standing, and no
jumps. What the user sees live is worse, because the drift hold moves the room by 20 to 30 cm after a slow step or
lean. One take, no ground truth; confirm on a second take with a longer walk.

1. **Drift hold: off by default for the Crystal** (`PIMAX_SLAM_DRIFT_HOLD=0`, the app switch), or switch it on only when
   the tracker is starved (few tracked points or a dark image) instead of always. Replay: mark error 273 → 74 mm. A
   parameter tweak is not enough (best 189 mm with a 3 cm/s floor).
2. **The recorder writes the calibration the tracker used** (non-zero `calib_accel_bias` in the take's
   `pimax-basalt-calib.json` and `pimax-slam-info.json`), so offline replays match the session.
3. **Seed the output layer's gyro bias** with the driver's cached/refined bias (z 1.3 deg/s is learned only after
   about 40 s now). The still-gate back-off stays a desk-only fix, not testable on worn takes; keep it for later.
