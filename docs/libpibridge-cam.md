# libpibridge-cam

The Pimax tracking cameras (UVC `34a4:0014`, four OV7251 fisheye cameras in one 2560x832 grey stream): frame layout
and metadata, V4L2 capture, `.pxcam` recording and replay, the camera broker (PXCB v1), the self-healing live source,
the factory calibration (KB4 intrinsics, rig and body extrinsics), the device-to-monotonic clock, the capture
statistics, the stream profiles (the grey four-tile stream and the Motion-JPEG two-tile stream of a Crystal Super
firmware), and the 6DoF support the head tracker uses: the Basalt calibration export, the tracker supervisor, the head pose
output layer (IMU-rate prediction, blended corrections, still hold and re-zero, gravity tilt, exact freeze, drift
governor), the head IMU log and the camera auto-exposure (Dream Air and Crystal Super models). Ported from omarchyvr
`rust/pimax-cam` (MIT OR Apache-2.0) at commit `5ffbcc5` (M1, M2 and M4 were ported at
`381eea2`, which `5ffbcc5` changes only in the AE files); the Rust code is the specification, and its tests have C++
twins here. Milestones M1, M2, M3 and M4 of `docs/research/rust-port-plan.md`.

CMake targets `pibridge_cam_obj` (objects) and `pibridge_cam` (static, for PiBridge's tools and tests), namespace
`pibridge::cam::`, headers in `libpibridge-cam/include/pibridge/cam/`. No Qt; `pibridge-cam`, `pibridge-ctrltrack`,
`pibridge-headtrack` and `pibridge-slam` link it. Dependencies: zlib (required: compressed `.pxcam` records),
libjpeg (required: the Motion-JPEG stream profile, `src/jpeg.cpp`), pthreads, `<linux/videodev2.h>`.

Built with `-ffp-contract=off -fno-fast-math -fwrapv` (no fused multiply-add, so every f64 result is the Rust one bit for
bit; both use glibc's libm), `-fvisibility=hidden -fvisibility-inlines-hidden -fno-gnu-unique`. Every arithmetic
expression keeps the Rust evaluation order; Rust's wrapping release-mode integer arithmetic, its saturating float
casts, `f64::min/max/clamp`, `to_radians`, `sum()` starting at -0.0 and its text formatting are reproduced in
`rustfmt.h`.

## Headers

| Header | What | From |
|---|---|---|
| `frame.h` | `FRAME_W/H`, `TILE_*`, `META_*`, `FrameKind`, `Meta::parse` (96-byte metadata row), `tile_rows` (metadata `y_offset` or a scan), `tile_start_consistent`, `copy_tile`, `check_raw` | frame.rs |
| `clock.h` | `ImuClockRef`, `imu_dev_to_mono_ns` (u32 unwrap around the IMU reference, i128 clamp), `HostClockMap` (minimum-delay fallback, `RELAX` = 8), `monotonic_ns` | clock.rs |
| `geom.h` | `Vec3`, `Mat3`, `Pose` (apply/inverse/compose), `rodrigues`, `quat_from_mat`, `angle_deg`, ... | geom.rs |
| `kb4.h` | `Kb4::make` (computes `theta_limit`), `project`, `unproject` (Newton), `angle_of_pixel_deg`, `with_extra_radial` | kb4.rs |
| `xml.h` | the small XML reader of the factory document (`xml::parse`, `Element`, `XmlError` with the Rust byte offsets) | xml.rs |
| `calib.h` | `DeviceCalib::parse/load/load_default/load_first`, `stereo`, `body_from_cam`, `geometry_report` (the `calib` text), `reference_xml()` (embedded), `CalibError` | calib.rs |
| `json.h` | JSON reader, `quote`, `num` (Rust `{:?}` digits) | json.rs |
| `overlay.h` | learned calibration overlay (`CalibOverlay`, `with_overlay`, `save/rollback/clear`) | overlay.rs |
| `io.h` | `IoError` (Rust `io::Error`: `text()` is its Display, "No such file or directory (os error 2)"), `Result<T>`, file helpers | std::io |
| `source.h` | `Source`, `RawFrame`, `ReplayOptions`, `open_source` (spec: "", "auto", "/dev/videoN", "broker[:SOCKET]", a `.pxcam`), `open_once`, `is_broker_spec` | source.rs |
| `record.h` | `.pxcam` `Recorder`, `AsyncRecorder` (writer thread, queue of 64, pool of 8), `Reader`, `Replay` (rebased host times, loop, realtime pacing) | record.rs |
| `v4l2.h` | `V4l2Capture` (raw ioctls, mmap, 6 buffers), `find_device`, `pick_node`; negotiates YUYV and falls back to Motion-JPEG (`DeviceInfo::jpeg`) | v4l2.rs |
| `jpeg.h` | `looks_like_jpeg`, `decode_grey_jpeg` (libjpeg, luma plane only, no error output) | (new in PiBridge) |
| `resilient.h` | `Resilient` (reopen after an error or a 5 s stall, backoff 0.25 s doubling to 2 s / 1 s), `Policy`, `reconnect_enabled` | resilient.rs |
| `camera.h` | `Camera` (source -> metadata -> rows -> clock -> stats, recording), `FrameInfo` (= `pimax_cam_frame_info`), `Stats`, `TileView` | camera.rs |
| `stats.h` | `Series`, `CaptureStats` and its `report()` (the `stats` text) | stats.rs |
| `broker.h` | `Hello`, `Ring`, `serve`, `BrokerSource`, `probe`, `default_socket` | broker.rs |
| `slam.h` | quaternion helpers (`qmul`, `qrot`, `yaw_of`, ...), `t_imu_cam`, `ImuNoise` (`slam-imu.json`, defaults, regimes), `SlamSetup`, `factory_accel_bias`, the gyro bias cache, `parse_cams`, `basalt_calib_json`, `info_json`, `write_basalt_files`, `default_head_in_imu`, `Supervisor` (acquire / lose / re-snap / stale / reset / still hold and re-zero / recenter) | slam.rs |
| `headpose.h` | `Predictor` (IMU ring, bias learning, alpha-beta fusion with automatic weights, still test windows), `HeadParams::from_env` (every `PIMAX_SLAM_*`, `PIMAX_HEAD_TILT`, `PIMAX_FUSE_AUTO` switch), `HeadTracker` (correction blending, snap, exact freeze, gravity tilt, big-correction queue), `scene` (camera brightness from AE) | headpose.rs |
| `drift.h` | `DriftGov` (`PIMAX_SLAM_DRIFT_HOLD`: neck evidence windows, removal band, held shift), `report_line` (the "6DoF drift:" log line) | drift.rs |
| `imulog.h` | `HeadImuLog` (`PIMAX_HEAD_IMU_LOG`, writer thread, bounded queue of 8192, minutes + flag file), `rotate` / keep dir, `parse` | imulog.rs |
| `ae.h` | auto-exposure: `AeModel` (Dream Air / Crystal Super: ladders, gain codes, the Crystal's settings above the soft cap, power-on values, Windows' pair ranges, `model_allows`, `model_detect` from `PIMAX_CAM_AE_MODEL` or USB `34a4:0040`/`0044` in sysfs), `payload` / `parse_payload` (`58 19 51 00 01 32 ...`), `Histogram`, `scene_metric` (masked mean of the unclipped 16x16 cells), `is_good_frame` / `StepWatch`, `AeParams` (`for_model`, `old_law`, `from_env_model`: every `PIMAX_CAM_AE_*` override, `pair_caps`, `split_pair`, `fit_pair`), `LightGov` (extended range, low-light warning, clip-driven low stage), `AutoExposure` (`update`, `frame_stats`), the log texts (`model_line` = `PIMAX_AE_MODEL: ...`, `start_line`, `event_line`, the WayVR toast) | ae.rs, capi_ae.rs |
| `rustfmt.h` | Rust's `{:?}`/`{}`/`{:.N}` of f64, `{:?}` of strings and slices, lossy UTF-8, `trim`/`split_whitespace`, `parse::<f64>()`, saturating casts | (Rust core) |

## Stream profiles

`Source::profile()` says what the frames of that source are packed as; every source hands `Camera` the same canonical
2560x832 grey frame, so the ABI, the `.pxcam` format, the broker and the SLAM side do not change per profile.

- `StreamProfile::GreyTiles4` — what the Rust code documents: the device reports YUYV 1280x832, every byte is one
  grey pixel, so a frame is 2560 bytes wide and holds four 640x480 tiles, rows 0..7 the 96-byte metadata block.
- `StreamProfile::JpegTiles2` — the same cameras over a link that came up at 480 Mbps (a USB 2.0 port or a 2.0 hub
  in the chain): `34a4:0014` then exposes one 1280x832 MJPEG frame at 166666/200000 us and no YUYV, because YUYV is
  2.1 MB a frame and does not fit. `V4l2Capture` negotiates MJPEG, decodes
  each buffer, and copies its luma plane into the canonical frame, so tiles 0 and 1 (the two tracking cameras, side
  by side in the luma plane) are real and tiles 2 and 3 are zero. What JPEG takes away, it cannot give back:
  - the metadata block is gone with its exact bytes (it is picture data now, and 4:2:0 subsampling destroys the
    chroma the outer cameras were folded into), so no device timestamp, no frame counter, no exposure and gain,
    no metadata row offsets (hence `FLAG_NO_DEV_TIME`, and tile rows by scanning);
  - the controller cameras are not in the stream at all.

What does survive is the alternation of the two exposure programs: such a stream runs at 60 fps and the frames
alternate between a tracking window starting at row 96 and a short (controller-LED) window starting at row 9 — the
same pair the raw stream reports as `y_offset` 278 and 10. `Camera` picks the kind from that row
(`LED_WINDOW_MAX_ROW`), which gives SLAM its 30 Hz of normal-exposure frames and leaves the LED half to the
controller tracker. `Recorder` stores the profile in the `.pxcam` header (byte 24, zero in every file the Rust writer
wrote), so a replay reads it back; the broker forwards it to clients in its hello flags (`FLAG_PROFILE_JPEG2`).

## PiBridge's 6DoF improvements (not in the Rust code)

Measured in `docs/research/head-tracking-improvements.md`; each is on by default and off with **`PIBRIDGE_HEAD_PARITY=1`**
(then the head output layer and the recorder behave exactly as the Rust code at the pin; an explicit switch still wins).
`HeadParams()` keeps the Rust defaults (the Rust test twins use it); `HeadParams::from_env()` (what the C ABI calls at
`pimax_cam_track_create`) applies them:

- drift hold off (`PIMAX_SLAM_DRIFT_HOLD=0`; 1 = the Rust hold, 2 = an experimental hold only while the head is quiet,
  `DriftParams::quiet_gyro_std` / `quiet_accel_std`);
- the output's gyro bias starts at the driver's cache (`PIMAX_SLAM_SEED_BIAS`, `HeadParams::seed_gyro_bias`);
- the tracker's accelerometer bias is removed from every sample the output layer uses (`PIMAX_SLAM_ACCEL_BIAS`,
  `HeadParams::accel_bias`: `calib_accel_bias` of the calibration `$SLAM_CONFIG` names, `slam::slam_config_calib_path`
  / `calib_json_biases`, else the factory `aBias` of the headset's own XML);
- the rotation past the newest IMU sample is predicted with 0.75 of the angular acceleration
  (`PIMAX_SLAM_PREDICT_ACCEL`, `PredictorParams::ang_accel_k`);
- the recorder (`pimax_cam_slam_write_basalt`) writes the calibration the tracker used (`PIMAX_SLAM_RECORD_ACCEL_BIAS`).

Evaluated and off by default: the still re-zero back-off (`PIMAX_SLAM_STILL_BACKOFF=N`, `StillParams::backoff_max`). The
`6DoF output: ...` line keeps the Rust text and appends `; pibridge: <what is on>` (`HeadParams::pibridge_describe`).
`pibridge-slam headreplay` (docs/pibridge-slam.md) replays the output layer on a recording with these numbers.

## Files, sockets and environment (unchanged from omarchyvr)

- `.pxcam` v1: 32-byte header "PXCAMRAW", 32-byte record headers "FRAM" + the raw frame (or a zlib stream). Raw
  recordings are byte-identical to the Rust writer's; zlib payloads come from zlib instead of miniz_oxide, so their
  bytes (and sizes) differ while they decompress to the same frames.
- Broker: `$XDG_RUNTIME_DIR/omarchyvr-cam-broker.sock` (else `/tmp/omarchyvr-cam-broker-<uid>.sock`), memfd
  `omarchyvr-cam-broker`, protocol and ring layout in the header comment of `broker.h`. A PiBridge broker serves Rust
  clients (omarchyvr's Monado with `PIMAX_CAM_SOURCE=broker`, `pimax-ctrltrack live --source broker`) and a Rust broker serves PiBridge
  clients; the tests prove both directions.
- Calibration search: `$PIMAX_CAM_CALIB`, `$PIMAX_CALIB` (a `PIBRIDGE_CAM_CALIB` / `PIBRIDGE_CALIB` set wins), then
  `~/.config/pibridge/headset-calibration.xml` and `~/.config/pimax-linux/headset-calibration.xml` **only when the file
  is there** (docs/CONVENTIONS.md; the list and the "not found" text stay the Rust ones otherwise), then
  `~/.config/omarchyvr/headset-calibration.xml`, `<repo>/analysis/headset-calibration.xml` (`$PIMAX_REPO`, the
  installed data copy, `~/omarchyvr`), else the embedded Dream Air reference (`PIMAX_NO_REFERENCE=1` turns it off).
- `PIMAX_CAM_RECONNECT=0` (no self-healing), `PIMAX_CAM_STALL_S` (stall time, default 5), `PIMAX_CAM_OVERLAY`,
  `XDG_CONFIG_HOME`. Log lines keep the Rust prefixes and texts (`pimax-cam: camera lost (...); reopening`,
  `camera broker: ...`).
- 6DoF: `PIMAX_SLAM_IMU` (else `~/.config/omarchyvr/slam-imu.json`), the driver's gyro bias cache
  (`$XDG_CACHE_HOME/omarchyvr/gyro-bias`, else `~/.cache/omarchyvr/gyro-bias`), `PIMAX_SLAM_SMOOTH`, `PIMAX_SLAM_TAU_MS`,
  `PIMAX_SLAM_RESNAP`, `PIMAX_SLAM_FUSE`, `PIMAX_FUSE_AUTO`, `PIMAX_HEAD_TILT`, `PIMAX_SLAM_TAU_REST_MS`,
  `PIMAX_SLAM_BIG_CORR_MM`, `PIMAX_SLAM_DRIFT_HOLD`, `PIMAX_SLAM_FREEZE`, `PIMAX_SLAM_STILL_REZERO`, `PIMAX_SLAM_STILL`,
  `PIMAX_HEAD_IMU_LOG`, `PIMAX_HEAD_IMU_LOG_MIN`, `$XDG_RUNTIME_DIR/omarchyvr-imu-log.on`, the kept logs in
  `$XDG_STATE_HOME/omarchyvr/head-imu` (else `~/.local/state/omarchyvr/head-imu`). Only the `PIMAX_*` names (no
  `PIBRIDGE_*` aliases): the names omarchyvr's and Monado's tools use. The stderr lines are the Rust ones: the
  `6DoF output: PIMAX_HEAD_TILT ... PIMAX_FUSE_AUTO ... PIMAX_SLAM_DRIFT_HOLD ...` line at `track_create` and
  `6DoF drift: head tracking slid N cm without movement in N s; held (session total N cm)` (the app parses it).
- Auto-exposure (the tracker sends it; this library never sends): `PIMAX_CAM_AE_MODEL` (`dream-air` |
  `crystal-super` and a few spellings; else the HID chip on USB: `34a4:0040` alone = Crystal Super, else Dream Air),
  `PIMAX_CAM_AE_LAW` (`old`/`v1`/`off`/`0` = the previous law), `PIMAX_CAM_AE_TARGET`, `_EXP_MIN`, `_EXP_MAX`,
  `_GAIN_MIN`, `_GAIN_MAX` (for the Crystal clamped into its Windows range), `_PERIOD_MS`, `_K`, `_DEADBAND`, `_UPPER`,
  `_QUANTIZE`, `_EXTEND`. stderr at create: `PIMAX_AE_MODEL: <id> (<where from>); <ranges>` and `PIMAX_AE_EXTENDED: ...`
  (build markers); governor events `PIMAX_AE_EXTENDED: ...`, `PIMAX_AE_LIGHT_LOW: ...` (+ the XSOverlay toast to
  `127.0.0.1:42069`); for the Crystal a target outside Windows' ranges is never handed out (`PIMAX_AE_MODEL: crystal-super
  setting [..]/[..] outside the Windows ranges, not sent (N so far)`). The C ABI publishes the darker pair's mean of
  every analysed frame to `headpose::scene` (the head tracker's brightness input, `PIMAX_FUSE_AUTO`). An override that
  makes Rust's `f64::clamp` panic (a NaN cap below its floor, e.g. `_EXP_MAX=nan` with `_EXP_MIN` set) throws here;
  the C ABI turns both into NULL + "panic inside pimax_cam".
- The reference calibration lives in `devices/dream-air/reference/headset-calibration.xml` (verbatim copy of omarchyvr's,
  serial scrubbed: `deviceUID="reference"`), embedded at build time.

## Tests

`ctest -R pibridge_cam_` runs the twins of the Rust unit tests of these modules (frame 6, clock 3, kb4 4, geom 3, xml 2,
calib 7 + 1 PiBridge-only, json 3, overlay 5, record 4 + the 2 of `tests/replay.rs`, stats 1 + 1, camera 1, resilient 6,
v4l2 3, jpeg 5 (JPEG -> grey, junk buffers, the profile through `Camera`, the recorded profile), broker 9 + the PXCB wire layout byte by byte, slam 15, headpose 21, drift 2, imulog 3, ae 27 + the 2 text tests of
capi_ae.rs; plus `pibridge_cam_headpibridge`, 10 PiBridge-only cases of the improvements above). The calibration twins read the embedded
reference (the Rust fixture has the same numbers, only its `deviceUID` differs); so do the slam.rs tests that read
the project headset's own calibration in Rust (the reference is that file with the serial scrubbed). The two
`#[ignore]` head-imu replays of headpose.rs (`replay_head_imu_log`, `replay_drift_hold`) are analysis harnesses, not
checks (`pibridge-slam`'s head replay does that offline, `docs/pibridge-slam.md`). The imulog twin
waits for the keeper thread to re-point the `.prev` link (it removes and re-creates it), which the Rust test reads at
once and so can lose a race with. Tests that need the lab recordings
(`PIBRIDGE_LAB_CAM_DIR`, default `../ext/cam`) report "skipped" without them. No test opens a camera:
sources are recordings, synthetic frames and sockets in the build directory.

## Not ported yet

- Not needed and not planned: `rectify.rs`, `check.rs`, `png.rs` (the `tiles`, `rectify`, `hypotheses`
  subcommands).
- Text differences that remain on purpose: the detail of a corrupt zlib record (`zlib: <miniz_oxide's Debug of its
  error>` in Rust, `zlib: <zlib message>` here).
- `rfmt::debug_str` escapes exactly the code points Rust 1.99's `{:?}` of a `&str` escapes (not printable or grapheme
  extend: `src/rustfmt_escape.inc`, 911 ranges, produced by a probe that
  prints `format!("{:?}", c.to_string())` for every `char` other than the named escapes and keeps those that come
  out as `\u{..}`). A newer Rust
  with newer Unicode tables can differ there; regenerate the table with that toolchain if byte-identical text to a Rust tool matters.
