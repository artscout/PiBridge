# Constellation (Oculus Rift) controller tracking: reference and mapping onto wintrack

Written 2026-11-21 from the sources below, for the acquisition / re-acquisition problem of the window tracker
(`docs/pibridge-ctrltrack.md`, "The Pimax-scheme tracker": our fixes match Pimax while a controller is held, but we have
no fix for 46-55 s of 780 s where Pimax tracks continuously; the EKF tracker loses 21-30 s). Same problem class: IR LED
constellation on the controllers, head-mounted cameras, IMU on the controller.

Sources (all read, line numbers refer to these files):

- **M** = our Monado fork, `third_party/monado/src/xrt/tracking/constellation/` (Jan Schmidt's OpenHMD code ported by
  Beyley Cardellio, 2025-2026; used by the CV1 `rift` driver for the HMD and by `pssense`). Files: `t_rift_blobwatch.c`,
  `led_search_model.c`, `correspondence_search.c`, `pose_metrics.c`, `optimizer/ransac.cpp`, `optimizer/pose_optimize.cpp`,
  `t_constellation_tracker.cpp`, `t_constellation_tracker_internal.hpp`, `src/xrt/include/tracking/t_constellation.h`.
  The fork's Rift S driver (`drivers/rift_s/rift_s_controller.c:205`) is 3DoF only (`m_imu_3dof`); no constellation
  tracking of the Rift S controllers exists in Monado.
- **O** = OpenHMD, Jan Schmidt's `rift-kalman-filter` branch (github.com/thaytan/OpenHMD), cloned read-only to
  `/tmp/claude-1000/-home-darkstar-gamedev-pibridge/e3d7f8d6-79a2-4f32-bccb-56b131dbadec/scratchpad/constellation-src/openhmd-rift-kalman/src/drv_oculus_rift/`:
  `rift-sensor-pose-search.c`, `rift-tracker.c`, `rift-kalman-6dof.{h,c}`, `rift-sensor.c`, `rift-sensor-blobwatch.c`,
  `rift-sensor-flicker.c`, `rift-sensor-pose-helper.c`. This is the only version with the IMU fusion (UKF).
- **B** = Jan Schmidt's blog noraisin.net/diary: "Rift CV1 - Adventures in Kalman filtering" 1 and 2 (p=1004, p=1012),
  "Pose rejection" (p=1031), "Getting close now" (p=1046), "2.5 years of Oculus Rift" (p=1055), "Pulling on a thread"
  (p=1064); and the FossXR 2022 talk "Free drivers for Oculus Rift headsets" (indico.freedesktop.org, PDF).
- **P** = Pimax DLL reverse engineering, `/mnt/g/omarchyvr-windows/re-dll/led-engine-2026-11-20/{acquisition,tracking}.md`.
- **W** = our window tracker: `libpibridge-ctrl/src/track/wintrack.cpp`, `window.cpp`, `rig.cpp`, headers in
  `libpibridge-ctrl/include/pibridge/ctrl/track/`. These files were being edited while this was written; the W line
  numbers are as of 2026-11-21 (wintrack.cpp 1355 lines) and the function names are the stable reference.

## 1. Blob detection and LED identification

**Detector** (M `t_rift_blobwatch.c`): scanline extents above `pixel_threshold` (CV1 0x24 = 36, DK2 0x7f = 127,
`t_rift_blobwatch.h:23-25`), merged by overlap with the previous line (`:449-510`); a blob is kept only if its peak
pixel reaches `blob_required_threshold` 0x40 = 64 (`:389-393`, "collect fainter blobs as long as they have a bright
point"), is not 1x1 (`:396`), and is at most `max_blob_width` 50 px wide and high (`:401`; OpenHMD: 35 px,
`rift-sensor-blobwatch.c:221-224`) - the size cap is their only reflection / window / sun defence. Centroid: grey sum
of (pixel - threshold), 1-based coordinates, no half-pixel centring (`:296-347`). Max 250 blobs per frame
(`t_constellation.h:22`).

**Blob tracking between frames** (`:596-764`): each blob of the previous frame is predicted to `x + vx`, mutual
nearest-neighbour assignment with usurping, rejected beyond `max_match_dist` 50 px (`:671-674`); a match copies
`blob_id`, `led_id`, `age`, and sets the velocity (`:573-581`). So **LED labels and blob identities persist across
frames without any search**. `mark_blob_device` (`:843-920`) is called by the tracker after a (possibly late) pose
solve: it writes the LED labels found on frame k into the *latest* observation by `blob_id`, so a search that took
several frames still labels the current blobs (B p=1046: "labels can be applied even several frames after
identification").

**LED identity**: CV1 / DK2 LEDs carry a 10-bit blink pattern decoded over frames (O `rift-sensor-flicker.c:33-52`,
exact pattern match; `rift-sensor-blobwatch.c:554-555`). The correspondence search does **not** depend on it: in M there
is no flicker code at all; `led_id` on a blob only (a) restricts the search to blobs unlabelled or labelled for this
device (`correspondence_search.c:802-828`) and (b) sets `POSE_MATCH_LED_IDS` when a found pose agrees with the labels
(`pose_metrics.c:334-343`). Rift S / Quest controllers (no IDs) are matched purely by geometry + label persistence.

Expected LED appearance (M `pose_metrics.c:171-271`): LED projected with the full camera model, visible if in front,
inside the image, and `facing_dot <= cos(pi - visibility_angle)` (`:238`; OpenHMD: `RIFT_LED_ANGLE` 82 deg,
`rift-tracker-common.h:1152`; B p=1055 measured ~75 deg), optionally a per-model occlusion callback
(`t_constellation.h:307`); expected radius `led_radius_px = f * radius_m / z` (`:217`). A blob matches an LED only
within `led_radius_px` of the projection and if the blob is not more than 4x the LED size (`:64-70`); among several LEDs
the one nearer the camera wins (`:73-78`).

## 2. Cold acquisition: the correspondence search

M `correspondence_search.c:31-67` (design comment), OpenHMD `correspondence_search.c` is the same algorithm.

- **Model side** (`led_search_model.c:86-160`): for each LED as anchor, the list of neighbour LEDs whose normals are
  within `vis_a + vis_b` of the anchor's (`:114-117`), sorted by 3D distance (`:22`: projected distance is an option,
  off).
- **Image side** (`correspondence_search.c:894-946`): blobs undistorted to the normalised plane, blob size in
  normalised units -> `max_dist` (`:923-925`), every blob's neighbour list sorted by distance (`:937-945`), filtered per
  device to unlabelled / own-label blobs (`:802-828`).
- **Enumeration**: for every anchor LED (`:849-859`) and every 3-combination of its first `max_led_depth` neighbours
  (both orderings of neighbours 1/2: `:698-703`), for every anchor blob (`:654-669`) and every 3-combination of its
  first `max_blob_depth` neighbours (`:570-651`): LambdaTwist P3P on (anchor + 2) (`:395`), the 4th pair as a check.
  Depths (`:782-790`): shallow pass blob depth 4, LED depth 4 from neighbour 1; deep pass blob depth 5
  (`MAX_BLOB_SEARCH_DEPTH`, `.h:24`), LED depth 8 (`MAX_LED_SEARCH_DEPTH`, `:107`) from neighbour 3 (so the deep pass
  does not redo the shallow combinations). With the HMD's ~40 LEDs the shallow pass is 40 x 4 x 2 = 320 LED quads x
  N_blobs x 4 blob triples P3P solves; our 15-LED controller: 120 x N_blobs x 4.
- **Cheap rejections before scoring** (`:416-534`): z outside [0.05, 15] m (`pose_metrics.h:29-31`); the anchor LED
  must not face away from the camera (`facing_dot > 0`, `:454`, "controller LEDs can be visible at that angle");
  the anchor must reproject within `ANCHOR_LED_MAX_DISTANCE_M_SQ` = 0.025^2 normalised units (`.h:27`, LambdaTwist
  spurious failures); the 4th LED must project within its blob's size (`:534`).
- **Gravity prior** (`:195-215`, `:984-996`): with `CS_FLAG_MATCH_GRAVITY` the swing of the candidate orientation
  about the camera's gravity vector must be within `gravity_tolerance_rad` of the prior's swing (yaw free). Enabled
  when the IMU gravity error is < 25 deg (M `t_constellation_tracker.cpp:506-523`; O: < 30 deg, tolerance
  `max(2 sigma, 30 deg)`, `rift-sensor-pose-search.c:293-298`).
- **Full-model verification** (`:174-305` -> `pose_metrics.c:368-499`): project all LEDs, bounding box, greedy blob
  -> LED match within `led_radius_px`; a candidate is **GOOD** without prior if `visible_leds > 4 && matched_blobs > 4
  && error_per_led < 3.0 px^2` and (`unmatched_blobs * 4 <= matched` or `2 * visible <= 3 * matched`) - i.e. at most
  25 % noise blobs inside the box, or 2/3 of the expected LEDs matched (`:467-481`; O requires `> 6` of each,
  `rift-sensor-pose-helper.c:333`; B p=1031 "cold lock: 2/3rds of the expected LEDs must be visible"). **STRONG** =
  GOOD and `error_per_led < 1.5` (`:479`; B p=1046 "within 1.5 pixels"). `POSE_MATCH_DEGENERATE` when
  `matched <= 3` (`:349`) or below the model's `min_leds_for_correspondence_search_{with,without}_prior`
  (`t_constellation.h:266-276`): a P3P-only solve reproduces its own points and never wins (`:501-591`,
  `WORST_REPROJECTION_ERROR` 10).
- **Best-of comparator** (`pose_metrics.c:501-591`): strong beats non-strong; good beats non-good (unless degenerate);
  more matched blobs with lower error/LED; two more blobs with error/LED within 10 %; equal count and lower error;
  with priors, better orientation agreement if error within 20 %.
- **Stop**: `CS_FLAG_STOP_FOR_STRONG_MATCH` ends the search at the first STRONG (`:245`, `:585`, `:856`); else the
  whole depth is searched and the best GOOD returned (`:998-1015`). The long analysis makes two passes per frame over
  all devices: shallow (requires STRONG to accept, O `rift-sensor-pose-search.c:349-351`), then deep (`:253-274`;
  M `t_constellation_tracker.cpp:464-470`).
- **Refinement** (M `pushPose`, `t_constellation_tracker.cpp:693-870`): label the blobs for the pose, Ceres LM over all
  labelled blobs with RANSAC (`optimizer/pose_optimize.cpp:47,414`: max 100 iterations; `ransac.cpp:103-113`: up to
  1000 iterations, 99 % confidence, 4-point P3P+check samples, inlier within 3 px and 1.5x blob size, >= 4 inliers,
  adaptive k `:284-298`), outliers unlabelled (`pose_optimize.cpp:549`), covariance of the pose returned; the sample
  pushed to the device carries matched / visible counts and rms px (`:833-849`).
- **Multi-camera**: each camera is searched independently (its own fast + slow thread and its own
  `correspondence_search`, `:198-222`); poses go to the fusion as separate observations in a common origin
  (`getWorldPose`, `:246-268`). O: the first camera pose is initialised from the HMD's IMU gravity when its error
  <= 25 deg (`rift-sensor-pose-search.c:421-453`). Blobs claimed by a device in one camera are not shared with other
  cameras: there is no joint multi-camera solve (B p=1012 lists "coordinate frame analysis across multiple cameras" as
  future work).
- **Timing** (B FossXR 2022 slide, p=1064): "correspondence search - can be over 100 ms (but more often < 40 ms)";
  blob extraction + RANSAC 1-10 ms; frames every 19.2 ms. The slow path runs on its own thread; a frame arriving while
  it is busy is dropped / replaced (M `deferSampleToSlowThread`, `:271-287`; O `rift-sensor.c:441-456, 571-587`:
  the queue keeps only the newest). Results are applied late through the fusion's delay slots (section 4) and the
  blob labels through `mark_blob_device`.

## 3. Re-acquisition after a loss (the fast path)

M `processSampleFast` (`t_constellation_tracker.cpp:564-691`), O `rift_pose_finder_process_blobs_fast`
(`rift-sensor-pose-search.c:78-239`). Order per device per camera frame:

1. **Prior**: the fusion's pose predicted to the exposure time (O: read back from the UKF delay slot of that exposure,
   refreshed with everything that arrived since, `rift-tracker.c:674-716`, B p=1055), with per-axis error bounds
   floored at `MIN_POS_ERROR` 0.1 m and `MIN_ROT_ERROR` 25 deg (O `rift-tracker.c:60-61`, M 0.10 m / 30 deg
   `t_constellation_tracker_internal.hpp:51-52`; in M the bounds are constants because the fork has no fusion:
   `:367-371` "we don't have any real sensor fusion").
2. **Blob recovery** (M `tryDeviceBlobRecovery`, `:337-442`; O `:151-180`): if >= 4 blobs (O: > 4) still carry this
   device's LED labels from the previous frame, RANSAC-PnP on them directly, starting from the prior (or a default
   pose 1 m in front when there is none, `:366-369`); accepted if GOOD (M currently `kAlwaysTrustRansac`, `:421`).
   No search at all in this case.
3. **Prior check** (M `tryDevicePose`, `:289-335`; O `:135-143`): evaluate the predicted pose against the blobs
   (`prior_must_match` true: position and orientation within the bounds, `pose_metrics.c:421-456`: GOOD needs
   `error_per_led < 2.0` and `matched >= min_leds_with_prior`, the 25 % / 2/3 rule); accept if GOOD and the labels
   agree (`POSE_MATCH_LED_IDS`). Then the **last known camera pose** is tried the same way (M `:637-657`).
4. Otherwise the device's labels are wiped (`:659-668`) and the frame is deferred to the slow search (section 2) with
   the prior and the gravity flag; O only defers when more than 4 blobs are still unused (`:214-221`).

**How long they predict**: B p=1046 / O `rift-tracker.c:49` `POSE_LOST_THRESHOLD` 500 ms: the IMU carries the pose;
after 500 ms the reported position freezes and only the orientation updates (`:630-635`, `:894-897`); `had_pose_lock`
is false from then on (`:1031-1040`), which switches the search to the no-prior rules (more LEDs needed, no gravity
gate from the prior). There is no frame-count budget and no "far from head" rule: the brute force simply keeps running
on every frame that has enough unused blobs, with the prior while it is recent.

**Wrong-lock prevention**:
- A found pose that does not match the prior position is **ignored if a newer observation already exists**
  (`rift-tracker.c:768-773`); orientation is taken only if `POSE_MATCH_ORIENT`, or forced after 100 ms without any
  orientation observation (`POSE_LOST_ORIENT_THRESHOLD`, `:53`, `:779-792`).
- With a prior, a pose outside the bounds can be at most GOOD, never STRONG (`pose_metrics.c:477-480`), so it does not
  stop the search early and loses to any in-bounds candidate.
- B p=1055: with two cameras disagreeing, keep the pose nearest the last position, the IMU gravity and the last
  orientation; "observations outside an expected orientation range are rejected for a period". The author calls this
  outlier rejection "the main source of tracking improvement".
- Degenerate (<= 3 LED) solves cannot be the winner (section 2). Blob labels are only transferred to the blobwatch if no
  later pose has been found (M `:855-863`).
- No N-frame confirmation of a cold lock: a STRONG / GOOD full-model match is accepted on one frame.

## 4. Fusion (OpenHMD only)

- **Filter**: Unscented Kalman filter per device (O `rift-kalman-6dof.h:22-40`): orientation quaternion, position,
  velocity, acceleration (inertial frame), angular velocity (body), accelerometer and gyro bias; plus N **delay slots**
  of (orientation, position). Process model: hybrid constant acceleration / constant velocity (`MOTION_MODEL 2`,
  threshold 3 ms, `.c:22-25`), accel damping 0.9, velocity damping 0.999 (`:34-35`), bias noise 1e-17 / 1e-16
  (`:28-31`), accel clamp 16 g (`:40`). Runs per IMU sample: 1000 Hz HMD, 500 Hz controllers (B FossXR). Cost grew
  O(n^3): 5 slots ~10x slower, 3 slots ~2x; `NUM_POSE_DELAY_SLOTS` 3 (`rift-tracker.c:42`), slots reclaimed when a
  search is slow (`:982-996`).
- **Latency**: at the exposure trigger (the HMD reports the exposure count and its timestamp in the IMU stream,
  `rift_tracker_on_new_exposure`, `:372-452`) the predicted pose is copied into a free slot
  (`rift_kalman_6dof_prepare_delay_slot`, `:1056`); the slot stays fixed while the IMU keeps integrating; when the
  camera pose arrives (20 ms to hundreds of ms later) it is a measurement of the slot, and the cross-covariance
  corrects the current state (B p=1012). A frame's exposure is matched to the history by arrival time within +-10 ms
  (`:467-474`); device timestamps are used, not USB arrival (B p=1012: residual controller IMU vs frame jitter 2-4 ms).
- **Observation**: full pose if orientation matched, position-only otherwise (`:802-808`); the measurement noise is
  not scaled by the score (fixed in `rift_kalman_6dof_pose_update`).
- **Output** (`rift_tracked_device_get_view_pose`, `:610-666`): filter state at the latest IMU time, position frozen
  after 500 ms without an observation, velocity and acceleration zeroed then; angular velocity rotated to the device
  frame, linear velocity gets the lever-arm term `omega x r`; a 1-Euro exponential filter smooths the pose
  (`pose_output_filter`, B p=1046: "reduces jitter but adds lag in fast motion").
- M's fork has none of this: `Device::pushImuSample` is empty (`t_constellation_tracker.cpp:973-975`); the `rift`
  driver keeps a relation history with motion estimation (`drivers/rift/rift_driver.c:933`) and 3DoF IMU orientation.

## 5. Comparison: Constellation vs Pimax DLL vs wintrack

| Step | Constellation (M / O / B) | Pimax DLL (P) | wintrack (W) |
|---|---|---|---|
| Blobs | scanline extents, peak >= 64, <= 50 (O 35) px, grey-sum centroid; blobs tracked frame to frame (50 px gate, velocity) with persistent `blob_id` and `led_id` | detector not RE'd; ROI around predicted LEDs (margin 2 r_max + 15 px, whole image when fast) | union-find components, core at half peak (`blob.h`); no blob-to-blob tracking, no labels carried |
| LED model use | per-LED neighbour lists filtered by normals; expected LED radius px = f r / z; visibility 82 deg (B: 75 deg) | 15 LEDs, 3 mm disk -> 12 rim points, visible within 100 deg | 15 LEDs, visible within 100 deg (`wintrack.cpp:297`), quad index for acquisition (`quads.h`), triples for brute force |
| Cold search | depth-first over blob / LED neighbour lists, LambdaTwist P3P + 4th check, shallow (depth 4) then deep (depth 5 / 8), per camera, every frame with >= 5 unused blobs, stop at STRONG | per camera: 4-blob quad key -> db vote (K 4-6, <= 210 keys), merge to <= 11 sets, SQPnP all solutions, score in all cameras (5 px), top 3x2 refined; 2-blob fallback with the prior | quad index lookup per camera -> P3P -> joint 4-camera refine (`rig.cpp:157-250`), threads per camera; a fit needs >= 5 LEDs and rms <= 2.5 px (`rig.cpp:176`, `wintrack.cpp:1214`) |
| Cold acceptance | one frame: GOOD = > 4 matched, error/LED < 3 px^2, 2/3 of visible or <= 25 % noise; STRONG < 1.5 px | ControllerDetect: hypotheses carried by the gyro (0.3 rad, 2 misses), 3-frame IMU test (velocity change vs preintegration, static tilt 0.3 rad, 5 m/s^2), hot start 2 frames near the prior; accept >= 4 LEDs, LostThreshold 5 | imutrack port (`hyp_step`, `wintrack.cpp:692-818`): 3 frames, 0.5 m/s, 0.3 rad, 4 misses, twin wait, other-hand test; every 3rd frame when idle (`:1160-1161`), every 5th when parked |
| Prior use in search | gravity swing gate (25-30 deg) + per-axis position / rotation bounds from the filter covariance, floored 0.1 m / 25 deg; prior-matching poses need fewer LEDs (`min_leds_with_prior`) | prior pose (+ IMU delta) gives the 2-blob fallback and the hot start; Lost retries the matcher on the IMU prediction every frame | warm relock (`:1264-1275`): orientation within 30 deg of the gyro-carried one (or its twin), position within 0.15 + 1.0 * age m, still >= 5 LEDs / 2.5 px; `OrientPrior` tilt 20 deg (75 before the up is learnt), rot 30 deg (`:1171-1179`); warm for 10 s (`warm_s`) |
| Tracking step | fast path: RANSAC-PnP on still-labelled blobs (>= 4) -> prior pose check -> last known pose -> slow search | matcher: 999/300/150 px candidates, P1P/P2P/P3P <= 30 hypotheses, gate 60-1.5 px by distance, choice by inliers / error / nearness, >= 2 LEDs | same matcher (`match`, `:292-602`, gate x1.5); P3P / P2P hypotheses limited to 15 deg / 0.25 m / 0.15 m of the prediction |
| Loss / budget | IMU coasts 500 ms, then position frozen, orientation keeps updating; search never stops; no far rule | Lost budget min(0.5 s, 30/15/5/0 frames x 33 ms); > 1 m from head -> Initing; Initing searches every frame, no timeout | same budget (`:1125-1151`); relock by the matcher only within 6 lost frames and with >= 4 inliers (`:944-952`); beyond: the search |
| Wrong-lock guards | prior-mismatching position ignored if a newer observation exists; orientation forced only after 100 ms; degenerate solves never win; STRONG only in-bounds; labels not written over newer poses | the 3-frame IMU test; twin dies by gyro inconsistency; a blob claimed by both -> nearer controller | twin + motion test at acquisition; P3P angle gate; min 4 inliers relock; `revert_rms_px` (off); overlap 8 cm with the other controller (`:1218-1224`) |
| Fusion | UKF (O) 22-state + 7 per delay slot, 3 slots, late observations into the slot of their exposure | 10-frame window, IMU preintegration, Ceres, marginalisation, RK4 output to the newest IMU | same window (`window.cpp`), LM 5 iterations; the fuser carries the fix to display time |
| Threading | per camera: fast thread (~1 ms) + slow thread (10-100+ ms), frames dropped while busy, labels applied late | one LED-engine thread | synchronous per frame: 4-6 ms with two hands; `solve_frame` threads per camera |

## 6. What is applicable to our acquisition / re-acquisition, prioritised

Where our time goes (win1 dump, `docs/pibridge-ctrltrack.md`): 287 s of 921 s in Initing after 160 resets; cold
acquisition needs three consecutive frames with a 5+ LED fit confirmed by the IMU and runs every 3rd frame when idle;
with an arm hanging (hand 0.77 m below the head, tracked by Pimax throughout) such a fit comes only now and then. Losses
start in fast motion (prediction off by more than the candidate radius) and with 1 matched LED.

1. **Label persistence + blob recovery (fast-path step 2).** Track blobs frame to frame per camera (nearest neighbour
   with velocity, 50 px gate, `blob_id`) and carry the (controller, LED) label of every inlier of the last update
   (M `t_rift_blobwatch.c:596-764`, `:843-920`). In `match()`, before the prediction-based pairs, add a hypothesis
   from PnP / P3P on the still-labelled blobs (>= 4; M `tryDeviceBlobRecovery`). This is independent of how far the
   IMU prediction drifted, which is exactly the 2.7 m/s case where the prediction matched 0-1 LEDs while 7 LEDs were
   still in view (476 of 604 loss starts before the revert was dropped). Effort: 2-3 days (a small blob tracker in
   `wintrack.cpp` or a new `labels.{h,cpp}`, a hypothesis kind 4 in `match`, bench on win1 / win2 / beat1).
2. **Prior-aware acceptance for the warm search** (Constellation's with-prior rules). Our search accepts only
   `total >= 5 && rms <= 2.5 px` whether cold or warm (`rig.cpp:176`, `wintrack.cpp:1214`). Constellation accepts a
   candidate that lies inside the prior's bounds with fewer LEDs (`min_leds_with_prior`) and judges completeness by
   **expected visible LEDs**: 2/3 of the LEDs that should be visible matched, or <= 25 % unexplained blobs in the
   bounding box, error/LED < 2 px (`pose_metrics.c:421-456`). For a hanging arm with 4 visible LEDs, "5 absolute"
   never fires while "4 of 4 expected, within 30 deg of the gyro orientation and 0.15 m of the prediction" is a safe
   relock. Implement as a second accept path in `wintrack.cpp:1212-1275`: for warm candidates, `total >= 4` with the
   visible-ratio test (use `project_visible`), the existing 30 deg / 0.15 + age gate, and the P3P-degenerate rule
   (never accept total == 3). Effort: 1-2 days. Also use the same ratio in the cold hypothesis candidates
   (`:1214`), keeping the 3-frame IMU confirmation as the guard (Pimax accepts >= 4 there too).
3. **Keep the warm relock open for the whole Lost budget, with Constellation's guard instead of the 6-frame cut.** We
   close the matcher's relock after 6 lost frames because 2-3 blob relocks at a drifted prediction placed the
   controller 10-60 cm off. Constellation's answer is not a frame limit but (a) >= 4 labelled blobs for recovery,
   (b) prior bounds grown from the covariance, floored at 0.1 m / 25 deg, (c) a prior-mismatching position is dropped
   when a newer observation exists, (d) degenerate solves never win. With (1) and (2) in place, `relock_frames` can
   be raised to the budget (the whole-budget relock already gave 35 / 51 s lost vs 44 / 61 s, but with 2 wrong
   relocks; the 4-inlier + visible-ratio + bounds guard should remove those). Effort: 0.5 day of tuning on the dumps
   after 1 and 2.
4. **Asynchronous slow search with late application.** Constellation runs the brute force on its own thread, drops
   frames while busy, and applies the result at the exposure time through the fusion's delay slots and the label
   transfer. Our window already holds 10 frames of states: a search started on frame k and finished at k+2 can reset
   the window at frame k's state and re-push the IMU steps of k+1, k+2 (the `steps()` deque keeps 3 s). That lets us
   search every frame while Initing (today every 3rd when idle, synchronous in the 4-6 ms budget) and spend 20-40 ms
   per search without stalling the matcher of the other hand. Effort: 3-5 days (thread, snapshot of blobs + IMU,
   window re-anchor in `Window`, bench reproducibility).
5. **Gravity + covariance bounds from the window.** Constellation derives the rotation / position tolerance of the
   search from the filter covariance (B p=1031 "the filter variance is used as the tolerance"), floored at 25-30 deg
   and 0.1 m. We use fixed 30 deg and 0.15 + 1.0 * age m. The window's marginal information (`window.cpp:586-593`)
   gives a per-controller covariance for free; using it (with the floors) tightens the relock early in a loss and
   widens it honestly later. Effort: 1 day; gain uncertain, measure on the masks.
6. **Two-pass neighbour-depth search as the fallback when the quad index returns nothing** (few blobs, quantisation
   misses). Constellation's shallow pass is 120 LED quads x N_blobs x 4 blob triples of P3P for 15 LEDs - a few
   thousand P3P solves, well inside 10 ms, and it finds poses from 4 blobs as our index does, without the hash
   tolerance. Effort: 2 days; only worth it if the dumps show quad-index misses on frames where 4+ LEDs are visible
   (count `acquire_quads` empty results vs blobs in view first).

Not applicable or already covered: the UKF with delay slots (our window is the better estimator; Pimax uses the same
window form); the 10-bit LED blink IDs (no IDs on our LEDs, and Constellation's search does not need them); the 500 ms
coast / freeze (our fuser and Lost budget already do this); single-camera-at-a-time search (our joint 4-camera refine
is stronger, keep it).
