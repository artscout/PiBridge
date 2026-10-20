# pibridge-slam

Offline 6DoF tools, the C++ port of omarchyvr's `omarchyvr-slam` (`tools/slam/`, milestone M4 of
`docs/research/rust-port-plan.md`). Today it has the subcommand the Crystal setup uses, `export`, and the
auto-exposure dry run `ae` (M3); the analysis subcommands (`camlag`, `stillgate`, `imu-noise`, `timing`, ...) are
M10, and `check` is the app's `core/rec_check`.

```
pibridge-slam export [--out DIR] [--calib XML] [--cams 0,1] [--imu F] [--base CONFIG.json] [--gui] [--cam-offset-ns N]
                  [--gyro-bias X,Y,Z|none] [--bias-from DATASET] [--accel-bias factory|none|X,Y,Z]
```

It writes the same three files as `omarchyvr-slam export`, byte for byte, into `DIR` (default
`~/.config/omarchyvr/slam`, the path the Monado setup and the studies use):

- `pimax-basalt-calib.json` (+ `pimax-slam-info.json`): the Basalt calibration (kb4) of the chosen cameras from the
  factory XML (`libpibridge-cam` `slam::write_basalt_files`, the same code the Monado driver calls through
  `pimax_cam_slam_write_basalt`), with `calib_gyro_bias` and `calib_accel_bias`.
- `pimax-basalt-config.json`: a Basalt VIO base config with our override (`config.optical_flow_detection_grid_size`
  40). The base is `--base`, else the first of `<src_dir>/basalt/data/euroc/euroc_config.json` (`$PIMAX_SRC_DIR`, else
  `src_dir` in `~/.config/omarchyvr/paths.toml`, else `~/src`), `~/.local/share/basalt/euroc_config.json`,
  `<lib_dir>/basalt/share/basalt/euroc_config.json`, `~/.local/share/basalt/msdmo_config.json`, else the built-in copy
  of Basalt's EuRoC config (`tools/slam/data/basalt-euroc_config.json`, BSD-3-Clause, NOTICE).
- `pimax-basalt.toml`: the Monado `SLAM_CONFIG` with absolute paths.

**More than two cameras** (`--cams 0,1,2,3`, the Crystal Super's lower tracking pair plus the upper side pair): the
calibration lists the cameras in that order (the first two exactly as a `--cams 0,1` export has them, test
`four_camera_export_extends_the_two_camera_calibration`). Basalt's VIO config has no per-camera values (intrinsics,
resolution and `T_imu_cam` are all in the calibration); the one setting that depends on the camera count is
`config.optical_flow_detection_nonoverlap`: Basalt finds new corners in cam0 and follows them into every other
camera, and only with this on does a camera that looks where cam0 does not (the upper pair) get corners of its own.
With more than two cameras the export sets it to true (adds it when the base lacks it; Basalt's EuRoC config and
the built-in copy already have it on, so the output stays the Rust tool's there). With two cameras nothing changes
(the parity tests compare every file with `omarchyvr-slam export`, including a `--cams 3,2,1,0` case).

Gyro bias seed (body frame): `--gyro-bias` in deg/s, else the first still second of `--bias-from DATASET` (a EuRoC
root or a recording folder: the newest child with `mav0`), else the driver's cache (`$XDG_CACHE_HOME/omarchyvr/gyro-bias`,
else `~/.cache/omarchyvr/gyro-bias`), else the cache of omarchyvr's earlier names (`<cache>/pimax/gyro-bias`), else the
Dream Air reference value (`devices/dream-air/reference/gyro-bias`). Accel bias: the factory `Stateinit aBias` unless
`--accel-bias none|X,Y,Z`. IMU noise: `--imu`, else `$PIMAX_SLAM_IMU` / `~/.config/omarchyvr/slam-imu.json`, else the
defaults. The text on stdout is the Rust tool's (it prints calibration values: do not paste it from a headset's own
calibration into the repo); errors are `pibridge-slam export: <the Rust message>` on stderr, exit 1.

```
pibridge-slam ae REC.pxcam [--frames N] [--model crystal-super]
```

The auto-exposure controller (`libpibridge-cam` `ae.h`, the code Monado runs through `pimax_cam_ae_update`) open loop
over a recording: for every tracking frame the first step it would request from that frame's own metadata (a
recording cannot respond), one line per device second (`t`, the frame's exposure/gain, mean and clipped fraction of
both pairs, the requested lower and upper setting) and the medians at the end. `--model` takes `dream-air` (default)
or `crystal-super` (its Windows ladder, gain codes and pair ranges). Text and exit status are `omarchyvr-slam ae`'s.

```
pibridge-slam headreplay DATASET TRACKING.csv [--steps steps.json] [--calib basalt-calib.json] [--latency-ms 25]
                      [--predict-ms 20] [--hz 90] [--central-vel] [--head X,Y,Z] [--csv OUT]
```

PiBridge's own (no Rust twin): the head pose output layer (`libpibridge-cam` headpose, what Monado calls through
`pimax_cam_track_*`) replayed on a recording (`DATASET`: a EuRoC root or a recording folder) and a tracker run
(Basalt's `tracking.csv`, e.g. from `monado-cli slambatch`), with the numbers that say whether a change makes head
tracking better: prediction error against the tracker at display time (by head turn rate, and with the slow offset
removed), jitter at rest and moving, mark error between the protocol's mark steps (`--steps`: the recorder's
`steps.json`), creep at rest, tilt error against gravity (`--calib`: the accel bias for that reference and the 3DoF
gyro bias; else the driver's cache), losses / resets / re-zeros, drift held, CPU per query. The replay is the one of
monado-abi's difftest (`slam_difftest replay`); every `PIMAX_SLAM_*` / `PIBRIDGE_HEAD_PARITY` switch is read from the
environment as in Monado. Method and results: `docs/research/head-tracking-improvements.md`.

## Tests

- `pibridge_slam_export` (`tests/test_export.cpp`): twins of the 4 `export.rs` tests (the lab-data one runs on the
  shipped reference), the `imu.rs` test and the load part of the `euroc.rs` test, plus the paths.toml reader.
- `pibridge_slam_export_*`: `pibridge-slam export` against `omarchyvr-slam export` of the pinned commit
  (`${PIBRIDGE_RUST_ORACLE_DIR}/omarchyvr-slam`), stdout + exit status + every file written, through
  `monado-abi/difftest/run_difftest.sh` in a scratch HOME: the reference calibration with 2 and 4 cameras, offsets,
  biases, the GUI flag, the gyro bias from each cache (new, `XDG_CACHE_HOME`, legacy, implausible), a Basalt tree via
  `paths.toml` and via `PIMAX_SRC_DIR`, the msdmo fallback, `--base` / `--imu` / `PIMAX_SLAM_IMU`, 14 error cases,
  `--bias-from` on a recorded dataset (lab data), the default calibration search, and the Crystal's own XML (lab data,
  compared by SHA-256 only, `--hash`).
- `pibridge_slam_export_ae_*`: `pibridge-slam ae` against `omarchyvr-slam ae` on `first`, `ctrl1`, `ctrl2.pxcam` (lab data)
  for both models, `--frames`, model spellings, and 6 error cases.

The files of the session's `~/gamedev/ext/slam/` (written by omarchyvr's working tree on 2026-10-19) match the
port's export of the Crystal XML except where their inputs differ: the gyro bias seed (that day's cache) and the
detection grid (30 in the study's working tree, 40 at the pinned commit); `pimax-slam-info.json` there predates the
`accel_bias_m_s2` field.
