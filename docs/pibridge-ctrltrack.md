# pibridge-ctrltrack and the controller tracker (libpibridge-ctrl, tracker half)

The optical tracker of the Pimax controllers, the C++ twin of omarchyvr's `pimax-ctrltrack` (plan
`docs/research/rust-port-plan.md`, M7). The library is `libpibridge-ctrl/` `track.cmake` (namespace
`pibridge::ctrl::track`, targets `pibridge_ctrltrack` / `pibridge_ctrltrack_obj`, on `libpibridge-cam`); the tool is
`tools/ctrltrack/` (`pibridge-ctrltrack`). Nothing is sent to the headset or the controllers: the tracker only reads the
LED frames of the tracking cameras (a `.pxcam` recording or the camera broker) and writes a pose file.

```
pibridge-ctrltrack live   [--source broker|broker:SOCKET|FILE.pxcam] [--out POSEFILE] [--seconds N] [--aim-pitch-deg D]
                       [--calib XML] [--left JSON] [--right JSON] [--extrinsics P | --factory] [--baseline] [--no-body]
                       [--thr T] [--peak P] [--stats S] [--trace FILE]
pibridge-ctrltrack track  FILE.pxcam [--start N] [--count N] [--every N] [--calib XML] [--left JSON] [--right JSON]
                       [--csv OUT] [--no-prior] [--overlay DIR [--overlay-every N]] [--fit-extrinsics
                       [--save-extrinsics P]] [--extrinsics P | --factory]
pibridge-ctrltrack scan   FILE.pxcam [--every N]
pibridge-ctrltrack show   [--out POSEFILE] [--seconds N]
pibridge-ctrltrack health [--out POSEFILE] [--max-age S]   exit 0 fresh, 1 no file / no heartbeat, 2 stale or source down
```

The session runs `live --source broker:$XDG_RUNTIME_DIR/omarchyvr-cam-broker.sock --factory` (PiBridge's
`rec_pimax_ctrltrack`, the Crystal 6DoF script): the same command line works with `pibridge-ctrltrack`. Options, the
usage text, standard output and standard error, exit codes (2 = no calibration / no models: "optical controller
tracking is off"), environment names and files are the Rust tool's, byte for byte; the texts still say
`pimax-ctrltrack` (the usage line and the `pimax-ctrltrack: ...` error prefix), so logs and the launcher's checks read
the same. A bad number or a missing argument ends like the Rust panic (the same message lines, exit 101).

Not ported (`pibridge-ctrltrack <cmd>` prints "is not ported to PiBridge ... use the Rust pimax-ctrltrack <cmd>" and exits
2): the analysis commands `lockrate`, `fusereplay`, `imufit`, `occlusion` (plan M10; three of them need the
pimax-hands fuser) and `dump` (one PNG of an LED frame; `track --overlay` draws the same mosaics).

## What it does

- **Blobs** (`blob.h`): threshold + 8-connected components (union-find), intensity-weighted centroid above the
  background. `track`/`scan` use threshold 60, `live` 80 (`--thr`, `--peak`).
- **LED models** (`model.h`): Pimax's `controller_params.json` (15 LEDs with normals, `ImuToDevice`). Search order:
  `--left/--right`, `$PIMAX_CTRL_MODEL_DIR`, `~/.config/omarchyvr/controllers`, `~/.local/share/omarchyvr/
  controller-models`, the repo's Windows dump; each holds `left.json` + `right.json` or one `<serial>/
  controller_params.json` per controller (folder-name order, the `Hand` field wins). Else the shipped Dream Air
  reference (`devices/dream-air/reference/controllers/{left,right}.json`, embedded; `PIMAX_NO_REFERENCE=1` turns it
  off).
- **Calibration and extrinsics**: the headset XML by libpibridge-cam's default search (`--calib`, `PIMAX_CAM_CALIB`, ...,
  the built-in reference); `cam_from_body` per camera from the factory rig. Without `--factory` the user's own refit
  (`--extrinsics P`, default `~/.config/omarchyvr/ctrl-cam-extrinsics.json`, for this headset's uid) or, for the
  reference headset only (same factory rig), the shipped refit `devices/dream-air/reference/ctrl-cam-extrinsics.json`.
- **Solve** (`p3p.h`, `solve.h`): brute-force acquisition (blob triples x LED triples, Grunert P3P with a
  Durand-Kerner quartic, angular scoring, Levenberg-Marquardt on KB4 pixels), joint refinement over the four
  cameras, tracking from a prior.
- **Rig and frame solve** (`rig.h`): the rig, `solve_frame` (per-camera acquisition, on one thread per camera in
  `live`), blob claims.
- **Tracker** (`tracker.h`, `assign.h`): constant-velocity prediction, wide gate, few-LED track with a rotation
  prior, the loss state machine (30 failed frames), the far cold-start rule, the body model (reach, side, continuity,
  separation, collapse), the in-hand status of Monado's driver, when one runs (`$XDG_RUNTIME_DIR/pimax-ctrl-inhand`: a controller `down`/`off` of a
  live writer is not looked for).
- **Pose file** (`shm.h`): `$XDG_RUNTIME_DIR/pimax-ctrltrack.pose`, 256 bytes, magic `0x54435850`, version 1, seqlock
  on the u32 at 8 (odd while writing; the Rust atomic orders and fences), 96 bytes per hand from 16, the heartbeat at
  208 (beat_ns, led_frames, source_up, reopens, written at least every 0.1 s by `live`). pibridge-headtrack reads it (Basalt engine; Monado's
  driver read it with `pimax_ctrltrack_shm.h`); `show` and `health` read it too.
- **track** (`tools/ctrltrack/src/track.cpp`): pass 1 blobs, pass 2 acquisition on all CPUs (`available_parallelism`
  as Rust computes it: the cgroup CPU quota and the affinity mask; the result does not depend on the thread count),
  pass 3 tracking with priors, the extrinsics refit, CSV, per-camera bias, summary, overlay PNGs.

Numerics are bit-identical to the Rust code: built with `-ffp-contract=off -fwrapv`, the Rust evaluation order in
every expression, Rust's `sort_by` as `std::stable_sort` (also where -0.0 and 0.0 tie), the last maximum of Rust's
`max_by` in the pivot search, `f64::min/max/clamp` semantics, saturating float-to-integer casts, float sums starting
at -0.0. The two `sort_unstable`-like calls (`[usize; 3]` sorts and the deduplicated triple list) sort distinct or
identical values only. The one hash map (`track --overlay`, record number -> frame) is only looked up.

## Tests (`ctest -R ctrltrack`)

The `pibridge_ctrltrack_diff_*` parity tests against the Rust `pimax-ctrltrack` (every command byte for byte) were
removed with the oracle once the port was finished (2026-10-22).

`pibridge_ctrltrack_csv_ctrl{1,2}`: `ctrl1.pxcam`/`ctrl2.pxcam` are Crystal Super recordings, and their reference
CSVs `ctrl1.csv`/`ctrl2.csv` (lab data) are exactly `track REC.pxcam --calib crystal-calibration.xml --csv` of the
Rust tool; `pibridge-ctrltrack` writes the same bytes (with the default reference calibration the output is a different
one, so those CSVs were never a default-run golden). `pibridge_ctrltrack_live_broker`: `live --source broker:SOCK
--factory` as the session runs it, against a `pibridge-cam broker` serving `ctrl1.pxcam`: it runs and exits 0, the
start lines name the broker, LED frames reach the tracker, and (only when a Monado tree is at `third_party/monado`,
which is no longer in the repository) Monado's reader (`ctrltrack_shm_dump`) sees a valid pose file with the heartbeat.
`pibridge_ctrltrack_not_ported_*`: the message and exit 2. Unit twins: the 22 Rust `#[test]`s of the ported modules
(blob 2, assign 3, p3p 2, model 4, extrinsics 4, solve 3, shm 2, tracker 2) plus 3 extra (live blob parameters,
component order of model folders, the pose file bytes).

## PiBridge's improvements (default in `live`; `PIBRIDGE_CTRL_PARITY=1` = the Rust tool)

Measured in `docs/research/ctrl-tracking-improvements.md`. `track`, `scan`, `show` and `health` are unchanged.

- **Core blobs** (`BlobParams::core`):
  - each blob is measured on its core (cut at half its peak), so a saturated halo no longer pulls the centroid;
  - merged LEDs come apart;
  - the result: at a simulated 40/62 LED exposure the jitter is 3.2 -> 1.4 mm and 3.0 -> 1.0 deg.
- **Dim LEDs** (`BlobParams::dim`): when no pixel reaches the peak test, the thresholds scale with the tile's
  brightest pixel. At 0.45x or 0.3x brightness the Rust detection tracks 0 %, PiBridge 85 %.
- **Own LED model per hand** (`TrackCfg::slot_model`): a search fix with the other hand's model is re-fitted or
  dropped. On ctrl2, jumps over 8 cm go from 14 to 5.

- **The fuser's hint** (`TrackCfg::use_hint`; off by default, `PIBRIDGE_CTRL_TRACK_HINT=1` turns it on in `live`):
  - `live` reads `$XDG_RUNTIME_DIR/pibridge-ctrl-hint`, which the fuser writes (format in `pibridge/ctrl/hint.h`);
  - after a gap it takes the IMU pose as the prior and the rotation prior (2-4 LED fixes), relocks from it, and
    refuses left/right swaps in the search;
  - it was not better on the recordings with the synthetic IMU (research doc section 5).

- **Pimax's scheme in the IMU-led tracker** (`imutrack`, 2026-11-20). It follows a reverse engineering of Pimax's LED
  engine (LedObjectPoseEstimator 2.11.2); the reports are in `/mnt/g/omarchyvr-windows/re-dll/led-engine-2026-11-20/`.
  All figures below are from `ctrl-imu-take2`.
  - **Tracking hypotheses (P1P).** The gyro's rotation is kept. For each near (LED, blob) pair, the predicted position
    moves so that the LED lands on its blob: within 150 px (300 px in the first 5 frames), at most 5 cm. The move that
    matches the most LEDs in 4 px replaces the prediction. Result: misses 65 -> 45, jumps over 8 cm 7 -> 1, worst frames
    2.5 -> 0.7 ms. `CT_P1P=0` turns it off; `CT_P1P_M` sets the largest move.
  - **Cold acquisition by hypotheses (Pimax's ControllerDetect).** This applies to a controller with no orientation to go
    by: none, older than `CT_KNOWN_S` (10 s), or lost over 1 m from the head.
    - Every search fit and its refitted twin (the LED ring turned ~180 deg onto itself) becomes a hypothesis.
    - The controller's gyro carries each hypothesis. A new fit within 0.3 rad (`CT_HYP_GYRO`) extends it; 4 frames
      without one (`CT_HYP_MISS`) drop it.
    - From 3 frames the IMU test runs on the change of velocity between the fits, against the accelerometer's specific
      force under the hypothesis' orientation (when still, gravity's tilt too), at most 0.5 m/s (`CT_HYP_M`).
    - A rival on the same blobs that explains the IMU nearly as well delays the acceptance (up to 12 frames).
    - A hypothesis whose turns the other controller's gyro explains clearly better is refused.
    - The hypothesis is accepted as tracked, with its velocity. `CT_HYP=0` gives the old single-frame tentative pick.
    - Cold masks: wrong relocks 4 -> 0, relock p50 160 ms.
  - **The other controller's blobs.**
    - A search fit with a third of its blobs within 4 px of the other tracked controller's projected LEDs is refused,
      as is one within 8 cm of that controller's state. A tracked controller claims its blobs first, as in Pimax.
    - A failed pose fit tries its few-LED updates after both controllers' fits have claimed their blobs.
    - Few-LED-only frames in a row are capped at 15 (Pimax's weak lost threshold; `CT_LED_RUN`).
    - Masked relocks: 15/17 with 1 wrong -> 17/17 with none.
  - **Frame time.** The LED frame's device timestamp goes through a least-delay host mapping, instead of the USB arrival
    and its jitter. `PIBRIDGE_CTRL_FRAME_TS=host` gives back the arrival time. The log's second imutrack line shows the
    mean arrival-minus-capture gap.

- **beat1, a recorded Beat Saber session with Pimax as truth** (2026-11-20). The files are in
  `~/gamedev/ext/slamrec/`: `beat1.pxcam` (180 s), `beat1-imu-{left,right}.csv` (`PIMAX_CTRL_IMU_LOG` from
  pibridge-headtrack), and the shadow log `ctrl-shadow-20261120-063511.csv`.
  - **The comparison script** is `~/gamedev/ext/slamrec/tools/cmp_beat.py SHADOW.csv FIXES.csv`. The shadow log's head pose sits (0.0006, 0.0779,
    -0.0395) m from the tracker's body frame.
  - **Result with the defaults:** fixes for 99 % of LED frames, error against Pimax p50 1.4-1.6 cm, p90 3.3-4.4 cm,
    p99 7 cm.
  - **Without P1P** the right hand is tracked in 55 % of frames.
  - **`CT_SIGMA_A_REL`** (the process noise scaled by the acceleration) defaults to 0, because 0.1 tracked the left
    hand 95.6 % instead of 99.2 %.
  - **Distance guard:** a fix over 1.2 m from the head is not published; the controller is lost instead.
- **Fuser, with the IMU-led tracker** (`track_vel`). In the shadow run, error p90 went 58 -> 6.5 cm, and 74 -> 7 cm on
  swings of 2.5-5 m/s.
  - **No jump / turn tests:** the tracker gates its fits by the IMU already (`trust_tracker`; `PIBRIDGE_CTRL_FUSE_GATE=1`
    keeps the tests).
  - **Carried by the accelerometer** over the fix's age (`track_acc`; `PIBRIDGE_CTRL_TRACK_ACC=0` turns it off).
  - **Snap on re-entry:** a re-entry over 15 cm from the shown pose snaps to the fix instead of blending.
- **Parked controllers** (gyro quiet for 5 s) are searched for every 5th frame instead of never.
- **Shown pose smoothing.**
  - The fuser spreads each fix's step over 15 ms (`PIBRIDGE_CTRL_SMOOTH_MS`; 30 until 2026-11-22, 0 tried the same day: the lag below was the delivery, not the smoothing; 15 played right with the velocity published, low-speed jitter 1.7 mm as with 30): in the shadow logs
    (`ext/slamrec/tools/ctrl_reversal_profile.py`) the shown pose trailed Pimax's by 3-5 cm on the approach to a swing
    reversal, turned 12-24 ms later and ran 1-5 cm past the hand, which the user saw as the saber bending on in the
    direction of the swing. Still 150 ms while the controller is nearly still: gyro under 0.3 rad/s and the tracker's
    speed under 0.12 m/s (`PIBRIDGE_CTRL_SMOOTH_STILL_MS`).
  - **Delivery (2026-11-22).** pibridge-headtrack publishes our controllers with the tracker's velocity carrying the
    fix while optical (`Fuser::shown_velocity`), and CustomHeadsetOpenVR hands it to SteamVR with the sample's age as
    poseTimeOffset, as for the head, so SteamVR predicts the controller's position to the display time. The angular
    velocity is NOT published: the first live try handed SteamVR the gyro's rate through the calibration
    (`Fuser::gyro_rate`) and the saber's blade flapped about the grip (the rate's frame or scale against the published
    orientation is unverified; the orientation already carries the gyro to now). From 2026-11-20 the driver zeroed them (the EKF's optical<->held velocity flickered): a frame of lag behind
    the head. If the 25 Hz step jitter shows again, this is where to look.
  - **Why.** At rest (still1, 2026-11-20) our 0.5-4 Hz bob matched Pimax's (likely the arms' own sway), but our fast
    jitter was about twice Pimax's.
  - **What was rejected.** Weighing fixes less in the tracker itself (Pimax's fast-stop, `CT_QUIET=3`) cost the left
    hand 75 -> 61 % tracking, so it stays off.
- **Fit tolerance by distance** (Pimax's gate: 5 px to 0.4 m, 3 px to 0.7 m).
  - **Why.** Held still at about 0.3 m, the live 16-19 LED fits had 2-2.5 px rms against the fixed 2.5 px. Failing it,
    few-LED updates let the pose wander by 5-10 cm in depth.
  - **The factor.** The fit's rms tolerance is 2.4x under 0.25 m, 1.6x under 0.4 m and 1.2x under 0.55 m. The match
    window stays as it was.
  - **Effect on take2.** rms misses 384 -> 11, few-LED frames L 16 -> 1 %.
  - **Effect on beat1.** Unchanged within 1 cm p99.
  - `CT_NEAR_TOL=0` gives back the fixed tolerance.
  - The LED models we use match Pimax's own kit files exactly.
- **The 1 m rule** of a lost controller counts from where the last update put it, not from its IMU-only position.
  Live, that position ran off on a crossing and forced a slow cold search.

- **The Pimax-scheme tracker** (`wintrack.h`, `window.h`; `PIBRIDGE_CTRL_TRACKER=win`, bench `BENCH_WIN=1`;
  2026-11-20). A rewrite of the tracking core after the reverse-engineered engine, next to the EKF tracker:
  - **A 10-frame sliding window** per controller (Eigen, dense Levenberg-Marquardt, 5 iterations, marginalization of
    the oldest frame into a linear prior): poses, velocities and biases of the last frames solved jointly with the
    IMU preintegrated between them (Pimax's sigma_a 0.25 / sigma_g 0.05 in the VINS form, bias walks 3e-4 / 9.5e-5),
    every matched LED as a reprojection residual (sigma 0.1 px, Huber 3; 2 when the mean |a| exceeds 13 m/s^2),
    keep-static / zero-velocity factors while a stopped controller is still (Pimax's fast-stop and its IMU thresholds).
  - **Pimax's matcher:** LEDs visible within 100 deg; candidate pairs within 999 / 300 / 150 px (first 5 frames / fewer
    than 4 LEDs last time / else); hypotheses: the prediction, P3P on rough matches, P2P and P1P with the predicted
    rotation (at most 30); each scored in all cameras by greedy one-to-one matches within a distance gate (60 / 10 /
    5 / 3 / 1.5 px, scaled 1.5x for our blob noise: beat1 hand 1 95.4 -> 97.2 %); the choice by inliers, error,
    nearness to the prediction and Pimax's alternative rule; 2 inliers suffice while tracking.
  - **Lost:** a budget of 30 frames or 0.5 s (15 with few LEDs or the controllers close in an image, 0 with an LED
    under 5 cm from a camera, 5 right after an acquisition); the IMU prediction is matched every frame and the search
    with the gyro-carried orientation retried; beyond the budget, or over 1 m from the head, a cold acquisition
    (PiBridge's quad index and hypotheses confirmed by the IMU, ported from imutrack).
  - **PiBridge's additions, each measured:** P3P / P2P hypotheses within 15 deg and 0.25 / 0.15 m of the prediction
    (take2: 55 deg picks poisoned the window); a relock after a loss needs 4 inliers and is left to the search after
    6 lost frames, 0.24 s (`WT_RELOCK_FRAMES`; it was 2: masks 6 wrong relocks -> 0, but see the live dump below);
    the accelerometer bias is left free (its prior weight 10 instead of 5e4: the hint's accelerometer carries a
    ~0.6 m/s^2 bias that the pinned bias could not absorb and the velocity ran away); 2 IMU steps suffice for a link
    (Pimax's 8 is for 500 Hz); an IMU sample without an accelerometer reading (the hint writes f_aim = 0 for it,
    live 1-2.5 % of samples, never in the recordings' logs) holds the previous force instead of counting as a free
    fall (`WT_ACC_HOLD=0` to undo; the count is "no acc" in the stats). The hint writer itself (fuse.cpp, 2026-11-20)
    now holds the last accelerometer mean for such an entry instead of writing zero, so new sessions carry none; the
    IMU-led tracker reads the same hint and saw the zeros too.
  - **Dropped after the live dump:** undoing an update whose window rms exceeds 8 px (`WT_REVERT=8` brings it back;
    Pimax has no gate after its solve) and the 1 m rule from the last update instead of the prediction (`WT_FAR_LAST=1`).
  - **Against the EKF tracker (take2 / beat1 vs Pimax / masks):** jitter 1.07 / 1.39 mm vs 1.30 / 1.61; jumps over 8 cm
    1 vs 2; hand 0 against Pimax p50 1.3 cm, p90 3.3, p99 5.2 vs 1.6 / 4.4 / 7.3; hand 1 1.7 / 4.4 / 7.4 vs
    1.4 / 3.4 / 7.7; frames with a fix 95.6 / 97.2 % vs 99.2 / 99.1 (one cold acquisition of 6.5 s on hand 0);
    take2 tracked 90.9 / 90.7 % vs 93.2 / 94.5; wrong relocks 0 / 0 vs 1 / 0; 6.3 ms per frame (two hands) vs 0.6.
  - **Live (2026-11-20, two sessions with Pimax's head):** 4 ms per frame (the live log's "frame work" line was
    wrong for this tracker and is fixed), updates on 100 % of frames for long stretches, but against Pimax the shown
    pose was 20-70 cm off for seconds about 10 % of the time (shadow log 16:01: hand 0 err p50 / p90 / p99 1.4 / 28 /
    70 cm, hand 1 2.0 / 20 / 65) where the EKF tracker's live fixes had been 5 cm p50 / 12 cm p90. The recordings do not
    reproduce it, so the live inputs must: `PIBRIDGE_CTRL_DUMP=FILE` makes pibridge-ctrltrack write every IMU sample
    and LED frame as it sees them, and `pibridge-ctrlbench --dump FILE --factory --imutrack` (with `BENCH_WIN=1` for
    this tracker, `BENCH_DUMP_FIXES=out.csv` for the fixes, `BENCH_STATS=1` for the counters, `WT_DEBUG=1` for a
    per-frame trace, `BENCH_IMU_SHIFT_MS` to move the IMU's clock) replays it exactly.
  - **The live dump `win1` (19:54, 15 min, 23169 LED frames, 2 x 201k IMU samples; Pimax's poses in
    `ctrl-shadow-20261120-195400.csv`; comparison head-relative as `cmp_beat.py`, over the 921 s Pimax tracked):**
    - The replay reproduces the live run: every live fix (20407 on hand 0) has a replay fix at the same frame, 99.7 %
      with the same LED count, positions within 0.2 cm p50 / 1.2 cm p99.
    - **The fixes themselves were good:** hand 0 p50 / p90 / p99 1.5 / 4.2 / 11 cm, hand 1 2.0 / 5.8 / 32; over 15 cm
      only 0.5 / 2.5 % of fixes (2 / 13 s). **The shown-pose errors were the fuser holding through tracker losses:**
      no fix for 88 s (hand 0) and 209 s (hand 1) of Pimax-tracked time, nearly all of it (287 s) in Initing, i.e.
      cold acquisitions of 1.8 s mean after 160 resets (78 "over 1 m", 82 budget). The cold acquisition needs three
      frames with a 5+ LED fit confirmed by the IMU; with an arm hanging (Pimax: the hand 0.77 m below the head,
      tracked throughout) the search finds such a fit only now and then and the hand stayed lost for up to 10 s.
    - **What made the losses (604 loss starts):** 476 were the 8 px revert: at 2.7 m/s the IMU prediction matched 0-1
      LEDs, the P2P hypothesis 7 cm away matched 7, the window would not move that far against the IMU (rms 10-20 px)
      and the update was undone, the next frame's prediction worse still. 122 were a single matched LED (few LEDs).
      Then the matcher was allowed to relock only 2 frames, so 160 of 604 ran into a reset (the prediction's
      velocity ran away to the 1 m rule within 0.3 s in 78 of them).
    - **Ruled out:** the IMU's arrival (the newest hint sample is 12-19 ms ahead of the frame at every frame, behind
      in 0.1 %, no gaps, 9 samples per frame); the blobs (the same as recorded); a camera-IMU clock offset (moving the
      IMU by +4 / +8 / +16 / +24 ms made it worse monotonically; negative shifts crash the bench, see below).
    - **Fixes, measured on the replay (hand 0 / hand 1):** no revert: lost 63 / 93 s, tracked 92 / 88 %; plus the
      6-frame relock: lost 44 / 61 s, tracked 94 / 91 %, error p50 / p90 / p99 1.6 / 4.0 / 10 and 1.9 / 5.3 / 23 cm,
      over 15 cm 1.6 / 12.4 s (before: 88 / 209 s lost, 88 / 72 % tracked, 1.5 / 4.2 / 11 and 2.0 / 5.8 / 32 cm,
      2.1 / 12.8 s). The whole-budget relock gave 35 / 51 s but 2 wrong relocks and 9 jumps on the masks; the 1 m
      rule from the last update only turned far resets into budget resets. The EKF tracker on the same dump: lost
      27 / 65 s, 1.8 / 4.5 / 10 and 1.8 / 4.7 / 27 cm.
    - **Recordings after the change:** beat1 fixes 95.5 / 99.1 %, p50 / p90 / p99 1.4 / 3.2 / 5.5 and 1.7 / 4.4 / 7.2 cm
      (was 95.4 / 95.6 %, 1.4 / 3.2 / 5.6 and 1.6 / 4.3 / 7.5); take2 tracked 91.1 / 93.3 % (was 90.9 / 91.1), jumps
      over 8 cm 1 (was 2), jitter 1.11 / 1.43 mm (was 1.11 / 1.34), window rms 3.0 px (was 2.7: the frames the revert
      dropped are kept); masks relocked 13 / 13, wrong 1, jumps 3, first-fix error p95 88 mm (was 13 / 13, 1, 1,
      68 mm); the 4 window unit tests pass.
  - **The live dumps `win2` (20:49, 13 min) and `win3` (23:33, 20 min), 2026-11-21; Pimax's poses in
    `ctrl-shadow-20261120-204949.csv` / `-233325.csv`; `cmp_dump.py SHADOW FIXES [--episodes]` (the comparison script
    of the win1 study, head-relative as `cmp_beat.py`; "lost" = Pimax-tracked seconds with no fix of ours within
    0.1 s).** The window tracker's fixes agree with Pimax as before (1.4-2.1 cm p50) but it had no fix for 46 / 55 s
    (win2, hand 0 / 1) and 75 / 51 s (win3) where the EKF tracker had 21 / 30 and 66 / 54.
    - **The bench crash (`BENCH_IMU_SHIFT_MS` negative, and a SIGSEGV on win3 in both trackers) was a data fault:**
      the last `F` line of a dump is cut when the session ends mid-write (win3: 2 of 4 cameras). The trackers built
      `obs` / `used` with one entry per camera *in the frame* but index them by the rig's camera, so `used[ci]` ran off
      the end: heap corruption, then bad_alloc in `solve_frame` or a SIGSEGV in `match_pixels`. Fixed in `solve_frame`,
      `Tracker`, `ImuTracker` and `WinTracker` (missing cameras are empty), the bench skips a truncated frame line and
      says so; `frame_short_of_cameras_is_safe` in `test_solve.cpp`.
    - **Where the lost time went (per-frame trace of all 56 win2 episodes; `episodes.py` in the agent's scratch):**
      about 35 of 40 s per hand in cold acquisition after a budget (0.5 s) or far reset. The search *found* the
      controller at the reset (first candidate 0.52 s after the loss, median) but the acquisition came 0.9 s (median) to
      5 s later: the cold hypotheses need three gyro-chained frames, and 5-6 LED fits of a far controller (an arm
      hanging 0.9 m below the head) scatter 20-40 deg frame to frame against the 0.3 rad chain gate (nearest candidate
      p10 25 deg: 3611 chains of length 1, 97 of length 3), and their IMU velocity test is noise-dominated (3 cm of
      position noise is 1.8 m/s of velocity difference at 25 Hz, against the 0.5 m/s threshold). The warm search had
      refused 68 of 92 fits, mostly as "far from the prediction": nearly every loss starts after 2-3 LED updates, after
      which the window's velocity stood at 5-40 m/s and its accelerometer bias at several m/s^2, so the IMU-only
      prediction was metres off within a frame; 11 far resets on hand 1 fired on the first miss (dist 1.0-1.6 m), and
      the 1 m rule fired at 1.00 m on a hanging arm Pimax tracked throughout. The 6-frame relock cut refused matcher
      relocks with 5-8 inliers.
    - **Changes (measured one by one on win2):**
      - *Warm relock in any mode* (imutrack's `known()`): the gyro-carried orientation counts as known for `warm_s`
        10 s after the last update whose place (the last 5+ LED update's) is within the far rule of the head, whether
        the budget has run out or not. A single 5+ LED fit is accepted that agrees with it within 30 deg, passes the
        learnt-up tilt test (20 deg), is over 8 cm from the other controller's prediction, and lies within
        0.15 + 1.0 x age m of the prediction, or has `relock_strong` 7+ LEDs as this hand's own model and is over 15 cm
        from the other controller (the EKF's `CT_FAR_LOOSE`). Only a controller unknown for 10 s is acquired cold.
      - *The restart state*: the prediction's velocity is carried only if the loss is under 0.2 s old and |v| < 6 m/s
        (else zero; the first frame's 0.3 m/s prior lets the window find it), and |ba| is clamped to 1 m/s^2. Without
        this the first version looped relock -> far reset -> relock with |v| 36 m/s (lost 44 / 124 s).
      - *A fit turned against the gyro* (the LED assignment on the ring's twin, or a scattered few-LED fit) is refit
        from the gyro's orientation at its place (12 / 5 / 2.5 px windows) and accepted if 5+ LEDs hold that within
        30 deg (`WT_REFIT=0` refuses instead).
      - *The far rule* at 1.25 m (`WT_FAR_M`): a Beat Saber swing or a hanging arm reaches 0.95-1.0 m from our head
        origin.
      - *Cold chain gate* 0.3 rad for 8+ LED fits, 0.6 rad for fewer (`WT_HYP_FEW_RAD`).
      - *The matcher's relock over the whole budget* with Constellation's with-prior rule
        (`docs/constellation-reference.md`, item 3): beyond 6 lost frames a 4+ inlier match relocks when it leaves at
        most 25 % of the free blobs inside its projected LED boxes unexplained (`relock_box_frac`, `WT_RELOCK_BOX=0`
        off). win2: lost 40.6 / 36.1 s -> 23.4 / 32.4 s, the error unchanged.
      - *Tried and dropped:* accepting any own-model 5+ LED fit while the prediction is not credible (over 1 s old or
        past the far rule): 5-LED fits 0.8-1.1 m above the head were accepted, hand 1 lost 32 -> 43 s with 6 more
        >15 cm episodes.
    - **Result (window tracker before -> after, EKF for reference; err cm p50 / p90 / p99 vs Pimax, >15 cm episodes
      (n / s), lost s (episodes)):**
      - win2 hand 0: 1.4 / 3.5 / 7.3, 1 / 0.3 s, 45.7 s (29) -> 1.4 / 3.6 / 7.7, 2 / 0.6 s, 23.3 s (25); EKF 1.7 / 4.8 / 8.2, 5 / 1.0 s, 20.8 s (25).
      - win2 hand 1: 2.1 / 5.2 / 11.8, 8 / 3.4 s, 54.5 s (25) -> 2.1 / 5.2 / 11.8, 8 / 3.7 s, 34.8 s (28); EKF 1.7 / 4.1 / 18.7, 13 / 8.6 s, 29.7 s (26).
      - win3 hand 0: 1.4 / 3.6 / 9.5, 8 / 2.8 s, 74.5 s (31) -> 1.4 / 3.7 / 8.4, 5 / 1.9 s, 50.2 s (18); EKF 1.6 / 4.5 / 9.0, 14 / 3.9 s, 66.2 s (23).
      - win3 hand 1: 1.9 / 5.2 / 14.1, 15 / 8.8 s, 50.8 s (28) -> 1.9 / 5.2 / 13.4, 15 / 8.2 s, 36.9 s (25); EKF 1.6 / 3.8 / 16.7, 18 / 11.0 s, 54.1 s (29).
      - Counters on win2: far resets 20 -> 6, cold acquisitions 41 -> 4, warm relocks 24 -> 64 (win3: 33 -> 6, 54 -> 4, 24 -> 48). Without the refit of turned fits and the far rule from the last 5+ LED update (box rule on): win2 23.4 / 32.4 s, 2 / 0.6 and 9 / 3.9 s over 15 cm.
      - Both trackers lose both hands for the last 10 s of win2 (771-781 s; the window's rms went to 25-39 px with
        4-6 LEDs just before), not studied.
    - **Recordings after the change:** beat1 fixes 98.9 / 99.6 % (was 95.5 / 99.1), p50 / p90 / p99 1.4 / 3.3 / 9.4
      and 1.7 / 4.4 / 7.3 cm (was 1.4 / 3.2 / 5.5 and 1.7 / 4.4 / 7.2: the 3.4 % of hand-0 frames now covered are
      relock fixes with larger errors, >10 cm 1.0 %); take2 tracked 93.5 / 91.7 % (was 91.1 / 93.3), jumps over 8 cm
      2 (was 1), jitter 1.03 / 1.59 mm; masks (`--mask-every 2.5 --mask-s 1.0`, 11 masks) relocked 11 / 11, wrong 0,
      frames to relock p50 0 p95 30, first-fix error p50 4.8 p95 24.6 mm, jumps 6 (the earlier run, 14 masks: 13 / 13,
      wrong 1, p50 18.6 p95 88 mm, jumps 3); the libpibridge-ctrl unit tests pass (`ctest -R pibridge_ctrl`; the
      `live_broker` test needs a running broker).
    - **Open:** the window's accelerometer bias runs to 0.7 / 1.3 m/s^2 median (up to 8-9) during fast motion, an
      IMU-vision inconsistency not understood (a clock offset was not it; `BENCH_IMU_SHIFT_MS` negative can now be
      tried); on win2 it stood at 5.8 m/s^2 right after the first acquisition (`bg` pinned at 0) and 2.5 m/s^2 after
      others, and it is what drives the runaway: a 5 m/s^2 bias is 0.7 m/s of velocity error per 100 ms, so after
      2-3 LED updates the IMU-only prediction runs off within a frame, which starts the fast-motion losses, the far
      resets and the relock gates' refusals. The truly cold acquisition (nothing known for 10 s, or the session start)
      is still slow for a far controller (chains break on the few-LED fits' scatter; the IMU test is noise-dominated).
      Next step from `docs/constellation-reference.md`: blob label persistence and recovery (item 1), which attacks the
      loss *start* in fast motion where the IMU prediction matches 0-1 LEDs while 7 are in view, and the 4-LED warm
      accept with the expected-visible ratio (item 2; needs `solve_frame` to return 4-LED fits for warm hands). The
      EKF tracker stays the default until a live run confirms the replay.
  - **Bench:** `BENCH_IMU_MS` sets the hint rate it feeds (4 ms, the live rate; it was 10 before 2026-11-20).
  - **Tried and dropped:** the XML's 5th / 6th distortion numbers as Pimax's tangential terms (`CT_TANGENTIAL=1`:
    the fits got worse, and the per-LED residuals fit other coefficients); the engine's -5.4 ms controller IMU shift
    (`CT_IMU_SHIFT_MS`: 0 measured best on our timeline); re-anchoring an Initing controller by its orientation alone.

`pibridge_ctrltrack_csv_*` run with `PIBRIDGE_CTRL_PARITY=1`. `pibridge_ctrltrack_improved` tests the new paths.
`pibridge_ctrltrack_take_steps` checks the controller take's guided steps (`tools/ctrltrack/ctrl-take-steps.json`).
`pibridge_ctrlbench_imu_log` checks the replay of a fuser IMU log (`pibridge-ctrlbench --imu-log`).

`pibridge-ctrlbench` (same directory, not installed) is the offline harness: tracked %, jitter, jumps, swaps, CPU,
synthetic exposure (`--sim`), synthetic occlusion (`--mask-every`), per-LED blobs (`--leds`), and the fuser on a
synthetic IMU (`--fuse`).

## Notes and follow-ups

- The 0 % session (below) is explained in `docs/research/ctrl-tracking-improvements.md` section 1. Its log says no
  blob passed the peak test, and AE was off, so the LED exposure was the recordings' 25/35. The causes left: the
  controllers were out of view, their LEDs were off, or their LEDs were under 47 % brightness.


- The last live Crystal session (`ctrltrack-20261019052737.log`) tracked 0 % of about 150 LED frames per 5 s (30 Hz:
  the Crystal runs 60 fps), searching every third frame (the idle rule: nothing found for 2 s). The tracker itself
  works on the Crystal: replaying the Crystal recordings `ctrl1`/`ctrl2.pxcam` with the Crystal XML and the Dream Air
  reference LED models tracks both controllers in 86 % of the LED frames (`track`) and 97-100 % after the first
  seconds (`live` over the replay, Rust and C++ alike). Candidates, to check with a `.pxcam` recorded during such a
  session (`pibridge-cam record` next to the broker, then `pibridge-ctrltrack scan`): the controllers were not in view or
  their LEDs off (the take is the desk/dark one, Monado reports 0 fixes); the LED-frame exposure differs from the
  recordings (the recordings carry 25/35, with `PIMAX_CAM_AE=1` the AE sends the Windows LED pair 40/62, brighter:
  bigger, merged blobs; the live blob threshold 80 and `max_side` 40 / `max_area` 600 were tuned at 40/62 on the Dream
  Air, blob.rs:44-56); the old tracking recorder ran with `PIMAX_CAM_AE=0`, so its recordings never show the live LED
  exposure.
  No frame-rate assumption in the tracker breaks at 30 Hz (stale prior 300 ms tracker.rs:155, wide gate after 60 ms
  :223, velocity window 5-130 ms :442; "25 Hz" in live.rs:3 is a comment only).
- Not the tracker's: Monado's Pimax camera code feeds SLAM "at 25 Hz" (`pimax_camera.c:725`, the Basalt calib writer
  `:812`) while the Crystal delivers 30 Hz per exposure kind.

- **Relock gate (2026-11-21, `wintrack`):** measured on win1-4 against Pimax with `ext/slamrec/tools/relock_quality.py`
  (every first fix after a gap: its error, LEDs, cameras, the gap, who moved in the next 0.5 s) and the tracker's
  `WT_DEBUG` lines joined to it. Of 35 "wrong" relocks (> 15 cm) 10 were Pimax's own re-acquisition moving
  (its pose steps 20-60 cm right after, ours does not), 13 were 9-10 LED two-camera fits 25-40 cm off that the window
  pulls to the truth within 0.5 s (a far hand, weak depth; not a wrong assignment), and 8-12 were ours: four 2 m
  relocks, all one-camera 5-7 LED warm fits 22-29 deg from the gyro-carried orientation (two of them "strong" 7-LED
  fits that the orientation gate did not touch), three one-camera 5-LED fits 15-23 cm off, three 4-LED matcher relocks
  pushed with 1.7-5.2 px. None was on the other hand. Two rules:
  - *Tentative relock* (`tentative_below` 7, `tentative_one_cam`, `tentative_min_leds` 3, `tentative_rms_px` 2.5;
    `WT_TENTATIVE=0` off): a relock from under 7 LEDs, or seen by one camera, takes the window and claims its blobs
    but publishes no fix until the next frame's matcher update holds it with 3+ inliers under 2.5 px; a miss or a
    weaker update puts the window and the lost state back (counters `tentative / confirmed / undone`). With 4 / 1.5 px
    it undid 66 of 104 on win1, mostly right relocks (the window's rms runs ~2 px): lost time +7 s per hand.
  - *One-camera orientation gate* (`warm_rot_deg` 10 + `warm_rot_deg_per_s` 5 x age, strong or not; `WT_WARM_ROT=0`
    off): a warm fit seen by one camera must agree with the gyro-carried orientation that closely; two-camera fits keep
    the flat 30 deg (their right relocks sit up to 30 deg off after long losses). On the joined data: 3 of 47 right
    one-camera relocks refused, all four 2 m ones caught.
  - *Result* (wrong relocks / lost s per hand, plain -> gated): win1 3 / 21.4, 11 / 44.1 -> 3 / 22.7, 11 / 47.7;
    win2 2 / 15.4, 5 / 29.0 -> 0 / 16.2, 4 / 33.6; win3 3 / 55.3, 9 / 39.5 -> 1 / 42.5, 9 / 34.7; win4 0 / 2.6,
    2 / 3.9 -> 0 / 1.0, 1 / 4.4. One-camera wrong relocks 7 -> 0, the 2 m ones 4 -> 0; the remaining "wrong" are
    Pimax's moves and the far-hand convergence. beat1 masks (70 x 1 s, `--imu ... --imutrack`): relocked 70 / 70 as
    before, wrong 1 -> 0, first-fix error p95 51 -> 40 mm, frames to relock p50 1 -> 2 (the held frame).
  - *The live default, the EKF tracker (imutrack), on the same dumps* (wrong relocks / lost s per hand): win1 6 / 23.7,
    24 / 69.4; win2 9 / 18.1, 15 / 30.4; win3 13 / 72.9, 14 / 56.3; win4 2 / 4.6, 1 / 8.6: 84 wrong relocks against
    the gated window tracker's 29 (plain 35), 40 of them one-camera 5-7 LED fits 1-2 m off, the class the gate removes;
    lost time 284 s against 203 s over the eight hands. The window tracker with the gate is the better tracker on every
    dump: the default since 2026-11-21 and the only tracker the app offers (`PIBRIDGE_CTRL_TRACKER=ekf` in the extra
    environment brings the EKF back for an A/B; `frame` the per-frame tracker). To be confirmed in sessions.
  - *Not done:* the far-hand 25-40 cm first fixes (hold a relock's fixes until the window has 2-3 frames, or weight the
    restart's depth), P2P / P1P fits with the gravity tilt, Hungarian assignment, the accel-bias gate by |w|.
