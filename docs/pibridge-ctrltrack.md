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
