# Porting the rest of omarchyvr's Rust to PiBridge (C++): inventory and plan

Date: 2026-10-19. Read-only inventory (no builds, no runs). Spec = omarchyvr `main` at `381eea2` plus the AE work
(`pimax-cam/src/ae.rs` +604 lines, `capi_ae.rs`, `include/pimax_cam.h`, `pimax-device/devices/crystal-super/device.toml`
`camera_ae`), committed as `5ffbcc5` ("Autoexposition commit"); the port is pinned to `5ffbcc5` since M3.

Goal: PiBridge contains no Rust. Everything the Crystal Super stack takes from `~/gamedev/omarchyvr/rust/` is C++ in
PiBridge, including the static libraries the Monado Pimax driver links. PiBridge already has keepalived, eyed, the eye
tool, eyecal, libpibridge-hmd (pimax-proto, pimax session, pimax-device allowlist) and the EuRoC check
(`app/src/core/rec_check.*` = `omarchyvr-slam check`).

Two Crystal setups exist (omarchyvr `docs/CRYSTAL-SUPER.md`):
- **Lighthouse faceplate + SteamVR + CustomHeadsetOpenVR** (the daily setup): uses no Monado and, today, only
  keepalived + eyed, both already in PiBridge. Nothing below is needed for it.
- **Without the faceplate / 6DoF experiments / RECORD TRACKING**: Monado (`monado-service`, or `driver_monado.so`
  under SteamVR) + `pimax-cam broker` + `pimax-ctrltrack live --factory`. Everything below is for this setup.

## 1. What the Crystal stack uses today (evidence)

### 1.1 Static libraries Monado links

Build `~/gamedev/ovr-build/monado` (source `~/gamedev/omarchyvr/forks/monado`, RelWithDebInfo), CMakeCache:
`XRT_BUILD_DRIVER_PIMAX=ON`, `PIMAX_USE_RUST_LIBPIMAX=ON`, `PIMAX_CAM=ON`, `XRT_BUILD_DRIVER_HANDTRACKING=ON`,
`XRT_HAVE_ONNXRUNTIME=ON`, `XRT_HAVE_OPENCV=ON`, `PIMAX_RUST_TARGET_DIR=~/gamedev/omarchyvr/rust/target-clean/release`,
`PIMAX_{CAM,EYE,HANDS,LENS}_DIR` = the crates' `include/`.
`src/xrt/drivers/CMakeLists.txt` lines 116-182 link, and `build.ninja` confirms (monado-service, monado-cli,
driver_monado.so, pimax_ht_bench):

| Archive (target-clean) | Crate(s) inside | Header(s) Monado includes |
|---|---|---|
| `libpimax_capi.a` (28.8 MB, 2026-10-19 07:00) | `pimax-capi` = `pimax` session + `pimax-proto` + **re-exports** `pimax-hands` and `pimax-eye` (one Rust staticlib, two would clash on std) | `libpimax/include/pimax.h`, `pimax-hands/include/{pimax_hands,pimax_ctrlfuse,pimax_inhand}.h`, `pimax-eye/include/pimax_eye.h` |
| `libpimax_cam.a` (26.6 MB, 07:00) | `pimax-cam` | `pimax-cam/include/pimax_cam.h` (382 lines) |
| `libpimax_lens.a` (25.2 MB, 2026-10-17) | `pimax-lens` | `pimax-lens/include/pimax_lens_rs.h` |
| (`libpimax_hands.a`) | only for `PIMAX_USE_RUST_LIBPIMAX=OFF` | not used |

C symbols the driver actually calls (header symbols grepped against `drivers/pimax/*.c|*.cpp|*.h`; exported
symbols from `nm -g --defined-only`):

| ABI | Used by the driver | Exported but unused |
|---|---|---|
| `pimax.h` (libpimax) | `pimax_open`, `pimax_close`, `pimax_wait_connected`, `pimax_send`, `pimax_video_prepare`, `pimax_controller_attach`, `pimax_haptic`, `pimax_haptic_poll_request` + callbacks `on_imu`, `on_controller`, `on_controller_status`, `on_state` (session thread) | `pimax_request`, `pimax_read_serial/version`, `pimax_is_connected`, `pimax_set_video_rate`, `pimax_find_interfaces`, `pimax_decode_*`, `pimax_haptic_counts`, `pimax_open_with_fds` |
| `pimax_hands.h` (ABI 1) | `create/destroy/update/update_report/get/abi_version/demo_input/aim_to_grip/tracked_aim_to_grip/grip_to_wrist/controller_poses/controller_describe` | `pimax_hands_reset` |
| `pimax_ctrlfuse.h` | all 7 (`create/destroy/describe/get/link/optical/report`) | none |
| `pimax_inhand.h` | all 10 | none |
| `pimax_eye.h` (ABI 1) | `abi_version/start/stop/state/sample_at` (only with `PIMAX_EYE=1`, **not set** for the Crystal) | `latest`, `observe_target` |
| `pimax_cam.h` | `open/close/next/copy_tile/describe/version/last_error/get_stats/dev_to_mono_ns/record_start/record_stop`, `calib_load/calib_stereo/calib_body_from_cam`, `slam_t_imu_cam/slam_default_head/slam_parse_cams/slam_imu_load/slam_write_basalt`, `track_create/destroy/push_imu/update_vio/head/recenter/get_info/take_big_corr`, `ae_create/ae_update/ae_destroy` (30) | `ae_create_model`, `calib_parse`, `track_default_params`, `track_update` |
| `pimax_lens_rs.h` (ABI 1) | all 8 | none |

Which of them run in the Crystal 6DoF session (`~/gamedev/ext/tools/crystal-6dof.sh`, PiBridge
`app/src/core/rec_session.cpp`): `PIMAX_CAM_SOURCE=broker PIMAX_SLAM=1 PIMAX_CAM_AE=1 PIMAX_CAM_AE_MODEL=crystal-super
PIMAX_CTRL_TRACK=1 PIMAX_LENS_CONF=crystal.conf (lut = crystal-lut.bin) PIMAX_CAM_CALIB=crystal-calibration.xml`
(RECORD TRACKING: `PIMAX_CAM_AE=0`, `PIMAX_SLAM_RECORD=<dir>`). So: libpimax (HID, IMU, controllers, haptics), hands +
ctrlfuse + inhand (`pimax_ctrl.c`, always on with Pimax controllers), pimax-cam broker client + calib + SLAM setup +
Basalt calib writer + 6DoF supervisor (`headpose`) + AE, the lens LUT. **Not active**: eye ABI (`PIMAX_EYE` unset;
the Crystal's eye runs through pibridge-eyed + SteamVR), Mercury hands (`PIMAX_HANDS` unset; and Mercury is Monado's
own C++ `t_ht_mercury` + ONNX, not Rust: it only takes tiles from `pimax_cam_copy_tile`), `.pxcam` recording.

Finding: the C fallback `PIMAX_USE_RUST_LIBPIMAX=OFF` (`omarchyvr/libpimax/src/pimax.c` 402 + `decode.c` 57 lines)
is **stale**: it has no `pimax_haptic*` and no eye code, so it would not link with today's `pimax_ctrl.c`. It is no
way out; the libpimax ABI has to be ported too.

### 1.2 Binaries the session runs, and tools agents use

| Binary / subcommand | Who calls it | Crate, files |
|---|---|---|
| `pimax-cam broker --source /dev/videoN --socket $XDG_RUNTIME_DIR/omarchyvr-cam-broker.sock` | crystal-6dof.sh, PiBridge rec_session (`rec_pimax_cam`) | `pimax-cam` `main.rs` (cmd_broker), `broker.rs`, `resilient.rs`, `v4l2.rs`, `source.rs`, `camera.rs`, `clock.rs`, `frame.rs` |
| `pimax-ctrltrack live --source broker:<sock> --factory` | the same (`rec_pimax_ctrltrack`) | `pimax-ctrltrack` `live.rs`, `tracker.rs`, `track.rs`, `blob.rs`, `assign.rs`, `solve.rs`, `p3p.rs`, `model.rs`, `extrinsics.rs`, `shm.rs`, `main.rs`; pimax-cam frame/geom/calib/source |
| `omarchyvr-slam export` | wrote `~/gamedev/ext/slam/pimax-basalt{.toml,-calib.json,-config.json}` (20 mentions in ext/slam studies) | `omarchyvr-slam` `export.rs`, `main.rs` cmd_export; pimax-cam `slam.rs`, `json.rs`, `calib.rs`; data `data/basalt-euroc_config.json` |
| `omarchyvr-slam check` | after a recording | already ported: PiBridge `rec_check.cpp` |
| `omarchyvr-slam camlag` (4), `stillgate` (1), `imu-noise`, `timing`, `ae`, `shift`, ... | agents, offline studies | `camlag.rs` (+ omarchyvr-room `orb.rs`, `png` crate), `stillgate.rs` (pimax-cam headpose), the rest |
| `pimax-lens make-lut`, `fit-pvr` | one-time: `crystal-lut.bin` from the PVR dump | `pimax-lens` `lut.rs`, `pvrfit.rs`, `bin/pimax-lens.rs` |
| `pimax-ctrltrack track FILE.pxcam --csv` | offline controller replays (`~/gamedev/ext/cam/ctrl{1,2}.csv`) | `track.rs` |
| `pimax-cam info/stats/calib` | occasional diagnosis | `main.rs` |
| `monado-cli slambatch` (24 mentions) | Basalt replays | Monado C/C++, links the same three archives |

PiBridge itself calls (grep of `app/`, `tools/`): `pimax-cam`, `pimax-ctrltrack`, `monado-service` (rec_session),
`pimax-eyecal` (fallback only, already replaced by pibridge-eyecal). Nothing else.

### 1.3 Per piece: size, dependencies, C++ replacement, ABI, IPC, parity references

Line counts: Rust non-test code (up to `#[cfg(test)]`) / `#[test]` count of the crate.

**A. libpimax ABI (`pimax.h`)** -- `pimax-capi/src/lib.rs` 558 (+ `eye.rs` 228), `pimax/src/haptics.rs` 124,
`pimax/src/session.rs` 965 (already ported as `pibridge::hmd::Session`). External crates: libc, log, thiserror -> none.
C++: a thin `extern "C"` adapter over `pibridge::hmd::Session` (callbacks on its session thread, `pimax_request` must
not be called from a callback) + port haptics (OpenXR -> `58 09 07 SS AMP DUR`, request file
`$XDG_RUNTIME_DIR/pimax-haptic-request`, replace/coalesce counters). Parity: `pimax-capi/tests/abi.rs` (532 lines,
8 tests, struct layout + fake device), `tests/hands_export.rs`, `tests/eye_export.rs`; `pimax-difftest`
(C libpimax vs Rust, decode paths); fixtures `rust/fixtures/{p82,p83,if3_log_coldstart,ctrl_buttons_hub2}.rec.gz`,
`status37_golden.txt`, `mahony_golden.txt`.

**B. pimax-hands (hands, ctrlfuse, inhand ABIs)** -- 4,448 / 118 tests. `fuse.rs` 2,463 (optical + IMU fusion,
`PIMAX_CTRL_TRACK_HYST`, writes `~/.config/omarchyvr/ctrl-imu.json` on destroy), `mapping.rs` 1,011, `ffi.rs`,
`fuse_ffi.rs`, `inhand.rs`/`inhand_ffi.rs` (`PIMAX_CTRL_INHAND`, live mode file in XDG_RUNTIME_DIR), `pose.rs`,
`filter.rs`, `rest.rs`, `touch.rs`, `report.rs`, `demo.rs`. No external crates. C++: plain math, own small
quaternion/vec helpers (shared with E). Parity: `tests/{c_abi,driver_glue,inhand_fixtures,properties,replay}.rs`,
C programs `tests/c/{driver_glue,layout,session}.c`, fixtures `rust/fixtures/{ctrl_golden.csv.gz,ctrl_marks.json,
inhand_*.csv.gz}`.

**C. Eye ABI (`pimax_eye.h`)** -- `pimax-capi/src/eye.rs` 228 over pimax-eye `live.rs`. Not used for the Crystal,
but its symbols must exist unless the driver is built with `PIMAX_NO_RUST_EYE`. C++: adapter over `libpibridge-eye`
`live.cpp` (exists). Parity: `pimax-capi/tests/eye_export.rs`.

**D. pimax-lens runtime** -- 2,842 / 87 tests: `capi.rs`, `config.rs` (lens.conf parser), `model.rs` (polynomial,
bit-identical to the old C), `lut.rs` (the Crystal: `PIMAXLUT` v1 file, 129x129 grid per eye/channel, Catmull-Rom,
CRC32), `mask.rs` (visibility mask), `fit.rs`, `grid.rs` + `font.rs` + `png.rs` (debug grid, `PIMAX_LENS_DEBUG_GRID`).
Setup tool `pvrfit.rs` + CLI 1,875 (make-lut, fit-pvr). External: miniz_oxide (PNG deflate) -> zlib. Parity:
`tests/cli.rs`, the unit tests (`round_trip_through_bytes_is_exact`, fixture lines), plus a new grid comparison of
`pimax_lens_rs_compute` over `~/gamedev/ext/lens/crystal.conf` + `crystal-lut.bin`.

**E. pimax-cam library (what Monado links)** -- 11,364 / 149 tests in `capi` 679, `capi_ae` 341, `capi_slam` 789,
`ae` 2,420 (Dream Air + Crystal models, Windows ranges, `PIMAX_CAM_AE_LAW`, `PIMAX_AE_MODEL` log), `headpose` 2,963
(6DoF output: IMU-rate prediction, blended corrections, re-snap, still re-zero, `PIMAX_SLAM_*`, `PIMAX_HEAD_TILT`,
`PIMAX_FUSE_AUTO`), `slam` 1,516 (Basalt calib export, IMU noise, factory accel bias, gyro bias cache), `drift` 415,
`broker` 1,435 (client half for Monado), `resilient` 509 (reconnect/reopen), `source` 142, `v4l2` 734 (raw ioctls,
mmap), `record` 616 (`.pxcam`, zlib), `camera` 419, `frame` 388 (2560x832 frame, 4 tiles 640x480, metadata),
`stats`, `clock` 164 (device us -> CLOCK_MONOTONIC: IMU reference or minimum delay), `calib` 608 + `xml` 301 + `kb4`
244 (factory XML -> KB4 fisheye; embeds the Dream Air reference XML, `PIMAX_NO_REFERENCE`), `overlay` 495 + `json`
406, `geom` 266, `imulog` 397 (`PIMAX_HEAD_IMU_LOG`). External: libc, miniz_oxide -> Linux syscalls directly (as
libpibridge-hmd), zlib (already optional in libpibridge-hmd; make it required here), own JSON: extend
`libpibridge-room/json.*` (449 lines) or a shared `common/` JSON (ask the lead). No Eigen, no OpenCV: the Rust has its
own 3x3/quaternion/KB4 code, port it as is. Not needed for Monado: `rectify` 607, `check` 297, CLI `main` except
broker. Parity: `tests/c_abi.rs` + `tests/c/abi.c` (struct offsets as C sees them + calibration + replay through the C
API), `tests/replay.rs` (lab data), `headpose` `replay_head_imu_log` (ignored test, `PIMAX_REPLAY_HEAD_IMU`), the 149
unit tests, fixture `tests/fixtures/headset-calibration.xml`.

**F. Broker** -- protocol (`broker.rs` header comment): Unix `SOCK_SEQPACKET` at
`$XDG_RUNTIME_DIR/omarchyvr-cam-broker.sock` (fallback `/tmp/omarchyvr-cam-broker-<uid>.sock`); on connect one hello
datagram (magic `PXCB`, version 1, slots, slot_bytes, data_bytes, flags `FLAG_REPLAY`=1, desc_len, desc <= 200 bytes,
u32 LE) with the ring memfd via `SCM_RIGHTS`; then one 8-byte notice per frame (index k, u64 LE, slot `k % slots`).
Ring: 4096-byte header page (latest @64, published @72, camera generation @80, camera state @88 (0/1/2), heartbeat
@96, CLOCK_MONOTONIC ns), slots with a 64-byte header guarded by a seqlock (`2k+1` writing, `2k+2` done), default 8
slots, non-blocking sends (a slow client loses notices, never blocks the server), a second broker on the same socket
is refused, stale socket files replaced. Must stay byte-compatible in both directions while Rust and C++ coexist.
Tests: 9 in `broker.rs` (hello round trip, overwrite, read-only client mapping, two clients, slow client, reopened
camera, restarted broker, replay end).

**G. pimax-ctrltrack live** -- 3,780 / 23 tests (+ analysis `lockrate`, `occlusion`, `fusereplay`, `imufit` 1,507
that use pimax-hands `fuse`). IPC: pose file `$XDG_RUNTIME_DIR/pimax-ctrltrack.pose`, 256 bytes, magic
`0x54435850`, version 1, seqlock, 96 bytes per hand + heartbeat at 208; the reader is Monado's
`pimax_ctrltrack_shm.h` (keep byte-identical). LED models: embedded Dream Air reference
`pimax-device/devices/dream-air/reference/controllers/{left,right}.json` (the Crystal log says it uses those),
extrinsics `--factory` from the calibration XML (`reference/ctrl-cam-extrinsics.json` only for the reference rig).
Parity: `pimax-ctrltrack track ~/gamedev/ext/cam/ctrl1.pxcam --csv` vs `ctrl1.csv` (and ctrl2), unit tests in
assign/blob/extrinsics/model/p3p/shm/solve/tracker, `examples/bench_p3p.rs`.

**H. omarchyvr-slam export** -- `export.rs` 253 + `euroc.rs`/`imu.rs` + cmd_export (~600) on top of E's
`slam`/`json`/`calib`: Basalt `calib.json`, VIO config (Basalt EuRoC base + override
`optical_flow_detection_grid_size=40`, built-in base `data/basalt-euroc_config.json` BSD-3), `SLAM_CONFIG` toml;
options `--calib --cams --imu --base --gui --cam-offset-ns --gyro-bias --bias-from --accel-bias factory|none|X,Y,Z`.
Monado writes the same calib itself through `pimax_cam_slam_write_basalt` (so one implementation serves both).
Parity: the files in `~/gamedev/ext/slam/` and each recording's `pimax-basalt-calib.json` (written by Monado).
Analysis subcommands (5,631 + ORB 1,059 + PNG decoder): later, optional.

## 2. Not needed for the Crystal (leave out)

`omarchyvr-*` product crates (app, gui-core, components, config except the few paths used, doctor, autod, voice*,
hud, voicehud, passthrough, room, roommap, anchor, record, perf, desktest, xrtest, tools except keepalived/eyed
(done), backgrounds, ai), WayVR, `pimax-selfcal` (6,248, learned calibration overlays, opt-in), `pimax-ipd` (motor
not allowed on the Crystal), `pimax-difftest`, `pimax-lens-difftest`, `pimax-lenstest` (eyecal already took what it
needed), `pimax-eye` CLI parts beyond what PiBridge has, `pimax-device` (ported), `pimax-proto` (ported), Mercury hands
(not Rust), `pimax-cam` rectify/check/CLI subcommands other than broker, the ctrltrack analysis subcommands (keep the
Rust binaries as references until the end), Dream Air-only paths (IPD motor, pairing, unpair, sys_reset).

## 3. Port order and milestones (each verifiable without the headset)

Layout (CONVENTIONS.md style; new parts need the lead's OK for the table):
`libpibridge-cam/` (`pibridge::cam::`, target `pibridge_cam`: E + F client), `libpibridge-ctrl/` (`pibridge::ctrl::`: B and G's
solver), `libpibridge-lens/` (D), `monado-abi/` (the `extern "C"` layer: A, B, C, D, E headers copied verbatim, one
archive `libpibridge_pimax.a`, `-fvisibility=hidden`, only `pimax_*` exported, every entry `noexcept` with a catch-all),
tools `pibridge-cam` (broker, info, stats), `pibridge-ctrltrack` (live, track), `pibridge-slam` (export, later analysis),
`pibridge-lens` (make-lut, fit-pvr).

Parity method for every step: the Rust binaries/archives already built in `target-clean` are the oracle. Golden
outputs are produced once from them into a scratch folder (CPU only, `nice -n 19`, never while monado-service or
SteamVR runs); lab inputs (`~/gamedev/ext/cam/*.pxcam`, `~/gamedev/ext/slamrec/euroc_*`,
`~/.local/share/pibridge/recordings/*`) are read-only and the tests that need them are skipped when absent. The
strongest check is a **difftest**: the same C program (the existing `tests/c/*.c` plus new drivers) compiled twice,
against the Rust archive and against `libpibridge_pimax.a`, outputs compared. Compile with `-ffp-contract=off`, no
`-ffast-math`, so f64 results can match bit for bit (both use glibc libm).

| # | Milestone | Done when (no headset) | Size C++ (code + tests) |
|---|---|---|---|
| M0 | Spec freeze + harness: pin the omarchyvr commit (AE committed), copy headers into `monado-abi/include` with a checksum test, the difftest runner (CTest, lab tests skippable) | runner builds the C programs against the Rust archives and passes | 0.5k |
| M1 | `libpibridge-cam` core: frame, xml, calib, kb4, geom, json, overlay, clock, record (.pxcam r/w, zlib), source, v4l2, stats, camera, resilient | `abi.c` struct offsets identical; KB4/extrinsics of `crystal-calibration.xml` and the fixture XML identical to `pimax-cam calib`; `.pxcam` replay of `first.pxcam`, `ctrl1.pxcam`: same frame count, metadata, tile bytes, mono timestamps as `pimax-cam stats`; 149-test twins for these modules | 6k + 2k |
| M2 | Broker server + client, `pibridge-cam broker` | wire/ring layout tests; cross tests Rust server <-> C++ client and C++ server <-> Rust client (replay source `--loop`), slow client, reconnect, second broker refused; then the PiBridge app's `rec_pimax_cam` default points to `pibridge-cam` | 1.5k + 0.6k |
| M3 | AE (`ae`, `capi_ae`) | `omarchyvr-slam ae REC.pxcam --model crystal-super` sequence = C++ payload sequence byte for byte on `first.pxcam`/`ctrl*.pxcam`; property test: no payload outside `CRYSTAL_PAIR_RANGES` (lower 2200..10410 us / 13..449, upper 2200..8941 / 13..439, LED 40/62) for random scenes and every env override; payload also passes PiBridge's allowlist gate for `camera_ae` | 2.5k + 0.8k |
| M4 | SLAM support + 6DoF supervisor (`slam`, `headpose`, `drift`, `imulog`, `capi_slam`) + `pibridge-slam export` | `slam_write_basalt` output and `export` files identical (or within 1e-12) to `~/gamedev/ext/slam/*` for the Crystal XML; headpose replay: a head-imu style input built from `euroc_*/imu0` + a `monado-cli slambatch` trajectory, driven through both archives by one C driver: identical poses/flags/events per sample | 5k + 1.5k |
| M5 | Lens runtime (`libpibridge-lens`) | `pimax_lens_rs_compute` over a dense UV grid, FOV, cant and mask for crystal.conf + crystal-lut.bin and the Dream Air reference lens.conf: identical floats; LUT round trip and corrupt-file tests | 3k + 0.8k |
| M6 | Hands + ctrlfuse + inhand (`libpibridge-ctrl` part 1) | twins of the 118 tests; `driver_glue.c`, `layout.c`, `session.c` difftests; `ctrl_golden.csv.gz`, `inhand_*.csv.gz` replays identical; ctrl-imu.json written the same | 4.8k + 1.5k |
| M7 | Controller tracker (`libpibridge-ctrl` part 2) + `pibridge-ctrltrack live/track` | `track ctrl1.pxcam --csv` / `ctrl2` identical to the Rust CSVs (`~/gamedev/ext/cam/ctrl{1,2}.csv`); pose file bytes read back through Monado's `pimax_ctrltrack_shm.h`; `live --source <pxcam>` over a C++ broker gives the same poses as the Rust live | 4k + 1k |
| M8 | libpimax ABI on `pibridge::hmd::Session` + haptics + eye ABI on libpibridge-eye live | twins of `pimax-capi/tests/abi.rs` with the fake backend; fixture replays (`p82/p83/if3`, `status37_golden`, `mahony_golden`); the allowlist gate sees every command the driver sends (see risk 2) | 1.2k + 0.8k |
| M9 | Monado switch | see below | 0.2k CMake + patch |
| M10 | Optional analysis: `pibridge-slam camlag` (ORB + PNG decode), `stillgate`, `imu-noise`, `timing`; `pibridge-lens make-lut/fit-pvr`; ctrltrack `imufit/fusereplay/occlusion/lockrate` | outputs identical on the study data in `~/gamedev/ext/slam/{study,study2,ab}` | 6-9k |

Total for the Crystal session (M0-M9): about **29k C++ code + 11k tests** (Rust: ~24.5k code lines in scope; C++
runs about 1.1x). M10 another 6-9k. Rough order of effort: M1, M4 and M6 are the big ones; M2, M3, M5, M8 are medium.

### Status (2026-10-19)

| # | Status | Where, and what the tests show |
|---|---|---|
| M0 | **done** | Pin `381eea2` (`monado-abi/spec.txt`; re-pinned to `5ffbcc5` with M3). The seven ABI headers copied verbatim into `monado-abi/include` with `SHA256SUMS`; `monado_abi_headers_pin` compares them with `git show 381eea2:<path>` and notes that `pimax_cam.h` has moved in the working tree (AE). Oracle: `monado-abi/difftest/build-oracle.sh` (git archive of the pin, `CARGO_TARGET_DIR=<omarchyvr>/rust/target-port`). Difftest: `monado-abi/difftest/cam_difftest.c` built against `libpimax_cam.a` and `libpibridge_pimax.a`, `run_difftest.sh` compares stdout byte for byte, lab tests skip (exit 77). Archive `libpibridge_pimax.a` = one object, only `pimax_*` global (`monado_abi_symbols`). |
| M1 | **done** (C ABI part: open/close/describe/next/copy_tile/get_stats/record/calib_*/dev_to_mono_ns, version, last_error) | `libpibridge-cam/` (frame, xml, calib, kb4, geom, json, overlay, clock, record, source, v4l2, stats, camera, resilient, rustfmt), `monado-abi/src/cam_capi.cpp`, `tools/cam` (`pibridge-cam info/stats/record/calib/import`). Bit for bit identical to the Rust archive: struct layouts; 1960 clock mappings; calibration of the reference and the Crystal XML incl. ~4000 broken/mutated documents each (error texts with byte offsets, every pose as hex floats); the default calibration search (27 cases); full replays of `first`/`ctrl1`/`ctrl2.pxcam` (every frame-info field, tile hashes, stats, end-of-replay errors) and a 400-frame loop; recording raw (file bytes) and zlib (frames); 40 error cases. `pimax-cam calib`/`stats`/`record`/`import` text identical. All 57 Rust unit tests of these modules + the 2 of `tests/replay.rs` + the 3 of `capi.rs` have twins (`tests/c_abi.rs` is the difftest `layout`/`replay` modes). |
| M2 | **done** (except the app default) | `libpibridge-cam/src/broker.cpp`, `pibridge-cam broker`/`broker-status`. Wire/ring layout byte for byte; Rust broker -> C++ client and C++ broker -> Rust client (and same-side pairs) on `first.pxcam`: every frame after connecting identical to a direct replay, recorded spacing kept; identical raw hello and ring header; slow client skips without slowing the broker; second broker refused both ways; stale socket replaced; `broker-status` both ways; the 9 Rust broker tests have twins (reconnect after a broker restart, reopened camera, replay end). Open: the app's `rec_pimax_cam` default and its "another pimax-cam runs" check still name `pimax-cam` (app files, another agent's). |
| M3 | **done** | Re-pinned to `5ffbcc5` (`spec.txt`, `SHA256SUMS`: only `pimax_cam.h` changed; oracle rebuilt from a `git archive` of `5ffbcc5` into `rust/target-port2`, the old `target-port` deleted). `libpibridge-cam/` `ae` (`AeModel` Dream Air / Crystal Super, `PIMAX_CAM_AE_MODEL`, model from USB `34a4:0040`, `PIMAX_CAM_AE_LAW`, every override, the governor, the texts), `monado-abi/src/ae_capi.cpp` (all four `pimax_cam_ae_*`; publishes the darker pair's mean to `headpose::scene`, the head tracker's brightness input, as the Rust does), `pibridge-slam ae`; `devices/crystal-super.toml` = omarchyvr's `5ffbcc5` file verbatim (`camera_ae`). Byte for byte against the Rust archive (`ae_difftest.c`, 52 difftests, an in-process PXCB broker so frames are live): every `pimax_cam_ae_out` of `first`/`ctrl1`/`ctrl2.pxcam` for both models, 36 closed loops on a simulated sensor (random rooms, every `PIMAX_CAM_AE_*` override incl. NaN/inf), create/errors texts, the `PIMAX_AE_MODEL` lines, the scene hook through `pimax_cam_track_*`; `pibridge-slam ae` = `omarchyvr-slam ae` on the three recordings x two models + 7 option/error cases. Property: no payload handed out leaves the Crystal pair ranges (difftest exit 3 otherwise; and `monado_abi_ae_gate`: 479 closed loops x every override alone and in 150 random combinations, 31589 payloads all `camera_ae` through PiBridge's gate, 0 targets outside). 27 ae.rs + 4 capi_ae.rs + the allowlist.rs `camera_ae` test have twins. |
| M4 | **done** | `libpibridge-cam/` `slam`, `headpose`, `drift`, `imulog`; `monado-abi/src/slam_capi.cpp` (every `pimax_cam_slam_*` / `pimax_cam_track_*`); `tools/slam` = `pibridge-slam export`. Byte for byte against the Rust archive (`slam_difftest.c`, 17 difftests): struct layouts; `t_imu_cam`/`default_head`/`parse_cams`/`slam_imu_load`/`write_basalt` (files dumped) for the reference and, by hash, the Crystal XML, every error text; 13 synthetic head motions x 7 switch sets (every `PIMAX_SLAM_*`, `PIMAX_HEAD_TILT`, `PIMAX_FUSE_AUTO`; acquire, loss, re-snap, new world, resets, re-zero, recenter, jumps, NaN, phase-1 API, freeze, drift reports); two recorded takes (worn: `recordings/20261019-070409` + its Basalt run; desk: `euroc_20261019052737` + its Basalt run, 42 emulated tracker resets) with the drift hold / re-zero on and off: every update hashed, every pose update, event and 90 Hz query printed in hex; the head IMU log file and its rotation. `pibridge-slam export` = `omarchyvr-slam export` (stdout, exit status, every file; 31 cases incl. caches, `paths.toml`, `PIMAX_SRC_DIR`, `--bias-from` on a recording, the Crystal XML by hash). Unit twins: slam 15, headpose 21 (the 2 `#[ignore]` replays are analysis harnesses), drift 2, imulog 3, capi_slam 3, export 4 + imu 1 + euroc load 1. |
| M5 | **done** | `libpibridge-lens/` (config, model, lut, mask, grid, font, png, lens, pvrdump reader), `monado-abi/src/lens_capi.cpp` (all 8 `pimax_lens_rs_*`). Byte-identical to the Rust `libpimax_lens.a`: 72 synthetic scenarios (every key and warning text, env overrides, valid and 14 kinds of corrupt LUTs, LUT paths, debug grid incl. error paths, NULL handles), `crystal.conf` + `crystal-lut.bin` and the Dream Air reference at 513x513 per eye, masks, debug grid scanlines. 58 Rust unit tests + 3 capi tests have twins. Not ported: the pimax-lens CLI (show/make-lut/fit-pvr, fit.rs, pvrfit fit) = M10. |
| M6 | **done** | `libpibridge-ctrl/` hands half (`hands.cmake`, `pibridge::ctrl`: filter, mapping, report, pose, touch, rest, demo, inhand, fuse), `monado-abi/src/{hands,ctrlfuse,inhand}_capi.cpp`: all 30 functions of `pimax_hands.h`/`pimax_ctrlfuse.h`/`pimax_inhand.h` (the driver calls all but `pimax_hands_reset`, implemented too); doc `docs/libpibridge-ctrl.md`. Byte for byte against the Rust `libpimax_hands.a` (`hands_difftest.c`, 80 difftests): layouts, the Rust `tests/c/session.c`, 4 x 60k randomized hands updates, 17 `PIMAX_CTRL_REST`/`PIMAX_CTRL_AIM` pose sets, replays of `ctrl_buttons_hub2`/`p82`/`p83` through all three ABIs, the five `inhand_*` cuts (14 variants), 7 x 100k in-hand reports (mode file, request texts, every `PIMAX_CTRL_INHAND`), 34 fuser runs (scenarios, 400 s, IMU log, every `PIMAX_CTRL_TRACK_*`/`PIMAX_CTRL_IMU_*` switch, calibration files, the online fit and the `ctrl-imu.json` it saves). Also identical at -O0. 117 of 118 Rust tests have twins (`tests/driver_glue.rs` is `#[ignore]` and needs a Monado tree: M9). |
| M7 | **done** | `libpibridge-ctrl/` tracker half (`track.cmake`, `pibridge::ctrl::track`: blob, assign, p3p, model, extrinsics, solve, shm, rig, tracker), `tools/ctrltrack` = `pibridge-ctrltrack live/track/scan/show/health` (same CLI, usage, texts incl. the `pimax-ctrltrack` prefix, exit codes, env names, pose file + seqlock); reference LED models and extrinsics copied to `devices/dream-air/reference/`; doc `docs/pibridge-ctrltrack.md`. Byte for byte against the Rust tool: `track` on `first`/`ctrl1`/`ctrl2.pxcam` (defaults, `--factory`, windows, model search, `--fit-extrinsics`/`--save-extrinsics`, `--extrinsics`, `--overlay` pixels, the Crystal XML by hash); `ctrl{1,2}.csv` reproduced exactly with `--calib crystal-calibration.xml` (they are Crystal takes); `scan`; 13 real-time `live` replays (`--trace`, the final pose file read through Monado's `pimax_ctrltrack_shm.h`); `show`/`health` cross (Rust file -> C++ and back); 22 error cases; `live --source broker:SOCK --factory` against `pibridge-cam broker`. 22 Rust tests have twins. Not ported (exit 2 with a message): `lockrate`, `fusereplay`, `imufit`, `occlusion`, `dump` (M10). Open: the app's `rec_pimax_ctrltrack` default still names `pimax-ctrltrack`. |
| M8 | **done** | `monado-abi/src/pimax_capi.cpp` (the 17 `pimax.h` functions + `pimax_open_with_fds`), `src/eye_capi.cpp` (the 7 `pimax_eye_*` over libpibridge-eye live; the eye-module enable goes through the gate too), haptics in libpibridge-hmd (`haptics.h`: 8 ms per-hand limiter, request file; `Session::haptic`), `libpimax.cmake`. Byte for byte against the Rust `libpimax_capi.a` (`pimax_difftest.c`, 20 difftests: layouts, decoders over p82/p83/hub2/if3 + noise, a full fake session, replays p83/p82/hub2/if3 (also on IF0), `status37_golden` 28/28, `mahony_golden` within 1e-9 (reference XML), discovery, eye without a chip, commands). Twins of `abi.rs`, `eye.rs`, `eye_export.rs`, `hands_export.rs`; omarchyvr's unmodified `pimax-cli.c` builds against the archive; every ABI import of the live Monado driver objects (80) resolves in `libpibridge_pimax.a`. Every write goes through the model's allowlist gate (`monado_abi_pimax_capi`: every byte that reaches the fake device is listed, 51 Dream Air / 49 Crystal commands; an empty description sends nothing). Crystal toml (user, 2026-10-19, chat): new `haptic` entry; approval lines on the attach-burst and video-prepare entries, which omarchyvr's file already listed with capture sources (every frame the driver sends found byte for byte in the 2026-10-17 takes). Intended difference: an unlisted command returns -EPERM and is never written (Rust writes it). Not proven: the eye ABI against a chip (lab data), the Rust cmake-consumer test (M9). |
| M9 | **offline part done**; live A/B open | Monado switch `PIMAX_USE_PIBRIDGE` (`patches/monado/monado-pimax-use-pibridge.patch`, Monado `src/xrt/drivers/CMakeLists.txt` only, default OFF), built on omarchyvr `forks/monado` `0dff9fa2a` + the Crystal driver diff in a separate tree and build dir (cache = the live one with only paths + the three options changed; recipe in `docs/monado-abi.md`, "The PiBridge-linked Monado"). The marker `PIBRIDGE_PIMAX_ABI 1 spec omarchyvr 5ffbcc5` (`monado-abi/src/abi_id.cpp`, test `monado_abi_marker`). Results 2026-10-19: see "M9 results" below: every omarchyvr marker with the live counts, no Rust symbol or path in any binary, same `ldd`; `monado-cli slambatch` (deterministic) on the three takes byte-identical; `driver_glue.c` over the real `pimax_ctrl.c.o` identical output; `pimax_ht_bench` identical up to the missing Mercury models. Open: the user's live A/B, `cap_sys_nice` on the new binary (user/app), then the default. |
| M10 | not started | |

Known, intended differences: zlib payload bytes (zlib vs miniz_oxide; frames identical); PNG deflate bytes of the
lens debug grid (zlib vs miniz_oxide; scanlines identical); the lens C ABI turns what would be a Rust panic into an
error return; the detail text of a corrupt
zlib record; `stats`' latency line varies in its last digit between any two runs (it depends on the absolute clock
at the start of the replay), so the tool test leaves that line out. `pibridge-slam export` reports errors as
`pibridge-slam export: ...` (the Rust tool says `omarchyvr-slam export: ...`; stderr is not compared). The head IMU log's
writer and keeper threads are detached in both libraries; a test that reads the `.prev` link right after a start can
race the keeper's relink in both (the C++ twin waits for it).

M6/M7 differences (documented in `docs/libpibridge-ctrl.md`, `docs/pibridge-ctrltrack.md`): where Rust panics the C++ aborts / exits 101 with the same message lines except the location (`panicked at pibridge-ctrltrack:`); `*_create` returns NULL on out-of-memory (Rust aborts); the fuser's gyro history lookup (`std::partition_point`) equals Rust's binary search while host times never go backwards (the driver passes monotonic arrival times); `track --overlay` PNG bytes (zlib vs miniz_oxide; pixels identical). The ctrltrack difftests mask only what differs between two Rust runs too (durations, heartbeat/pose ages, pid, the last heartbeat's `led_frames`, thread id).

Follow-ups from M6/M7 (not in the parity path): (1) the Crystal live session's 0 % controller lock (`ext/slamrec/ctrltrack-20261019052737.log`) is not the tracker itself: on the recorded Crystal takes with the Crystal XML and the Dream Air reference models `track` locks both controllers in 86 % of LED frames and `live` 97-100 % after the first seconds (Rust and C++ alike). Likely: that take was the desk/dark one (Monado logs 0 fixes, controllers out of view or LEDs off), and with `PIMAX_CAM_AE=1` the AE sends the window pair 40/62 for LED frames while the recordings have 25/35 (RECORD TRACKING runs with `PIMAX_CAM_AE=0`), so live LED blobs are brighter/larger than what blob.rs's threshold 80, `max_side` 40, `max_area` 600 were tuned on. Check: record a `.pxcam` beside the broker during such a session and run `pibridge-ctrltrack scan`. No 25/50 Hz cadence assumption in the tracker; Monado's `pimax_camera.c:725/812` sets SLAM to 25 Hz while the Crystal gives 30 Hz per exposure kind. (2) Rust's `ctrl-imu.json` parser accepts NaN, after which `pimax_ctrlfuse_describe` panics (would abort monado-service). (3) The fuser's IMU sample-time estimate falls behind silently when device timestamps advance less than the report span (still lock and the gyro then stop working).

Follow-ups found in the SLAM study (`docs/research/slam-imu-intrinsics.md`), deliberately not in the parity path,
each to go behind a switch with today's behaviour as the default (details in `docs/monado-abi.md`, "Follow-ups"):
seed the output layer's gyro bias from the driver's cache (`PIMAX_SLAM_SEED_BIAS`), let the recorder write the
calibration the tracker used (non-zero `calib_accel_bias`: `pimax_cam_slam_write_basalt` gets the calibration through
`pimax_cam_calib`, which has no `Stateinit` raw attributes; `PIMAX_SLAM_RECORD_ACCEL_BIAS`), and drift hold off by
default for the Crystal (the switch `PIMAX_SLAM_DRIFT_HOLD` exists).

### M9: the live switch

- `forks/monado` (local commit only, published as a patch in PiBridge `patches/` like the Crystal prober patch): option
  `PIMAX_USE_PIBRIDGE=ON` + `PIMAX_PIBRIDGE_LIB=<path>/libpibridge_pimax.a` + `PIMAX_PIBRIDGE_INCLUDE=<pibridge>/monado-abi/include`;
  when ON, link that one archive instead of `libpimax_capi.a`, `libpimax_cam.a`, `libpimax_lens.a` and use its
  headers (identical bytes to the Rust crates' headers, enforced by the checksum test of M0). The driver C files do
  not change. Monado already links with the C++ linker, so libstdc++ comes for free.
- A separate build dir whose CMakeCache is a copy of `~/gamedev/ovr-build/monado/CMakeCache.txt` with only the
  three options above changed (omarchyvr lesson 2026-10-15: never a fresh cache).
- Marker check on the new `monado-service` and `driver_monado.so` (`grep -ac`): the omarchyvr NEEDLES list
  (`PIMAX_SLAM\0`, `held from the tracker`, `PIMAX_EYE\0`, `PIMAX_CTRL_TRACK`, `PIMAX_ANCHOR`, `origin_gate_mm`,
  `PIMAX_HT_HZ\0`, `PIMAX_CTRL_INHAND`, `PIMAX_CTRL_ANGLE`, `PIMAX_ANCHOR_GLIDE`, `PIMAX_CAM_AE_LAW`,
  `PIMAX_CTRL_TRACK_HYST`, `PIMAX_SLAM_DRIFT_HOLD`) plus `PIMAX_AE_MODEL` and a new `PIBRIDGE_PIMAX_ABI <version>` string.
  Five of these strings (`PIMAX_CAM_AE_LAW`, `PIMAX_CTRL_TRACK_HYST`, `PIMAX_SLAM_DRIFT_HOLD`, `PIMAX_AE_MODEL`, part of
  `PIMAX_CTRL_INHAND`/`PIMAX_CTRL_TRACK`) live only in the Rust code today: the C++ must carry the same env names and
  log texts. Also `cap_sys_nice=ep` after the relink (restored by the user/app, never by an agent).
- Offline acceptance on the new build: `monado-cli slambatch` on `euroc_20261018024922` and `euroc_20261019052737`
  with `deterministic=1` gives byte-identical trajectories to the Rust build (the study showed repeat runs are
  byte-identical); `pimax_ht_bench` on a `.pxcam` the same.
- Then the user's live A/B (PiBridge RECORD TRACKING with each build; the app's `rec_monado_service` key selects the
  build), and only after that the default changes. Install between sessions only.

#### M9 results (2026-10-19, offline)

Build: source = a local clone of the fork, branch `pibridge-abi` (commit 1: the Crystal diff copied from the live tree;
commit 2: `PIMAX_USE_PIBRIDGE`; local only, unsigned, never pushed; not a `git worktree` because Monado's git-describe
script fails in a worktree of a submodule). Build dir: a separate one, `ninja -j8` at `nice -n 19`, 717 steps, no
errors; about 2.6 GB. `libpibridge_pimax.a` from `build-port`. `cap_sys_nice` is not set on the new `monado-service`
(nor on the live one at the time of the check); restored by the user or the app, never by an agent.

Markers, exact occurrence counts (the needle with its `\0` where omarchyvr's NEEDLES has one), `monado-service` and
`driver_monado.so` alike:

| Needle | Live (Rust) | PiBridge |
|---|---|---|
| `PIMAX_SLAM\0`, `held from the tracker`, `PIMAX_EYE\0`, `PIMAX_HT_HZ\0`, `PIMAX_CAM_AE_LAW`, `PIMAX_CTRL_TRACK_HYST` | 1 each | 1 each |
| `PIMAX_CTRL_TRACK` | 14 | 14 |
| `PIMAX_ANCHOR` | 20 | 20 |
| `PIMAX_CTRL_INHAND` | 5 | 5 |
| `PIMAX_CTRL_ANGLE`, `PIMAX_ANCHOR_GLIDE` | 3 | 3 |
| `PIMAX_SLAM_DRIFT_HOLD`, `PIMAX_AE_MODEL` | 2 | 2 |
| `origin_gate_mm` | 1 | 2 (libpibridge-eyemath's calibration writer and reader each hold the key; the Rust has one string) |
| `PIBRIDGE_PIMAX_ABI` | 0 | 1 |

`grep -ac` (lines, not occurrences) differs for `PIMAX_SLAM`, `PIMAX_EYE` and `PIMAX_SLAM_DRIFT_HOLD` only because
the Rust rodata runs several names together on one line. A scan of every `PIMAX_*` name: the Rust-only ones are the
`PIMAX_EYE_*` tuning names, `PIMAX_SYSFS_ROOT`, `PIMAX_DEV_ROOT`, `PIMAX_REPOS`; PiBridge composes these at run time
(`PIBRIDGE_<name>` first, then `PIMAX_<name>`, `common/pibridge_env.h`), so the full literal is not in the binary; the
behaviour is the same.

Symbols: in `monado-service`, `monado-cli`, `driver_monado.so` and `pimax_ht_bench` of the new build, 0 Rust v0
symbols (`_R...`; live: 3684, 3684, 3684, 2798), 0 `__rust_*`/`rust_eh_personality` (live 37), 0 `.cargo/registry`
or `/rustc/` paths (live 23). No `libpimax_*` or `target-clean` path in `build.ninja`. `ldd` lists are identical.
`driver_monado.so` exports 2034 dynamic symbols (live 4792): the live set minus the 2757 leaked Rust symbols and
`rust_eh_personality`, plus `pimax_open_with_fds_pid` (PiBridge's extension); 137 `pimax_*` functions in
`monado-service` (live 136).

`monado-cli slambatch`, `deterministic=1`, `~/gamedev/ext/slam/pimax-basalt-{calib,config}.json`, the same
`libbasalt.so`, `EUROC_MAX_SPEED=false`, both builds:

| Take | `tracking.csv` | Result |
|---|---|---|
| `ext/slamrec/euroc_20261019052737` (desk) | 5782 lines | byte-identical |
| `ext/slamrec/euroc_20261018024922` | 7086 lines | byte-identical |
| `recordings/20261019-070409/euroc_20261019070409` (worn) | 3019 lines | byte-identical |

(`prediction.csv`, `filtering.csv` and `timing.csv` carry the host clock at query time and differ between any two
runs of one build; they are not trajectories.) Caveat: slambatch drives Basalt through Monado's EuRoC player and does
not call the Pimax archive; it proves the build is otherwise the same, not the C ABI (that is M1-M8's difftests).

Driver glue (`tests/driver_glue.rs`, done by hand): pimax-hands `tests/c/driver_glue.c` linked against each build's
real `drv_pimax.dir/pimax/pimax_ctrl.c.o` + aux libs, once with the Rust `libpimax_capi.a` (`target-port2`) and once
with `libpibridge_pimax.a`, scratch `HOME`/`XDG_RUNTIME_DIR`: stdout and stderr byte-identical. Both report the same one
check failing (`FAIL line 105: left wrist is not grip + (-0.0289, 0.0629, 0.0146)`): the test's expected wrist offset
is older than today's driver, in both libraries alike.

`pimax_ht_bench` on `first.pxcam` and `ctrl1.pxcam`, reference and Crystal calibration: identical output from both
builds, but both stop at "models: .../grayscale_detection_160x160.onnx missing" (no Mercury models on this machine;
they have no licence and are not fetched), so the Mercury part is not compared.

Remaining: the user's live A/B (RECORD TRACKING with each build through `rec_monado_service`), `cap_sys_nice` on the
new binary, then the default; installs between sessions only.

## 4. Risks

1. **Clocks and timestamps.** Camera frames carry device microseconds; `clock.rs` maps them to CLOCK_MONOTONIC from
   the IMU's reference (`pimax_cam_imu_clock_ref`) or a minimum-delay fallback; SLAM uses mid-exposure time
   (`PIMAX_SLAM_EXPOSURE_MID`) and `PIMAX_SLAM_CAM_OFFSET_US` (Basalt ignores `cam_time_offset_ns`). Any rounding
   difference shifts the camera/IMU alignment. Keep the integer arithmetic exactly (u64/i64, same division and
   rounding); the broker must pass the server's host timestamps through unchanged; parity is exact, not toleranced.
2. **HID allowlist.** Today Monado's `pimax_send` (Rust `pimax` session) has **no allowlist gate**: the Crystal AE
   stays inside the Windows ranges only because `ae.rs` clamps (`AeModel::allows`, `pair_caps`). PiBridge's
   `pibridge::hmd::Session` gates every report by `devices/*.toml`. Keep both: the AE clamp (M3 property test) and the
   gate. PiBridge's `devices/crystal-super.toml` has `camera_ae` (copied verbatim from omarchyvr `5ffbcc5`, M3). The gate will then also refuse what the Crystal toml does not list but the driver sends
   (haptics `0x07`, the controller attach burst `16/50/0a/03`, `pimax_video_prepare`'s commands): list them with
   their Windows-capture source, or accept that they are refused (safer) and say so in the docs. Decide before M8.
3. **Moving spec.** The AE rewrite (+604 lines) and `omarchyvr-slam` main.rs are uncommitted in omarchyvr; the
   Crystal driver patch is a patch file. Pin a commit per milestone and re-diff before the switch.
4. **Numerics of the controller tracker and fusion** (P3P, joint solve, LED assignment, fuser). Bit parity needs:
   `-ffp-contract=off`, explicit `std::fma` where Rust calls `mul_add`, Rust `as` casts (float->int saturating,
   NaN -> 0; C++ is UB) through a `sat_cast` helper, `std::stable_sort` where Rust `sort_by` is used (stable) and the
   same `sort_unstable` tie behaviour checked by tests, no hash-order dependence (`track.rs` has one HashMap: check
   it is not iterated into results). Note the last Crystal session tracked 0 % of LED frames
   (`~/gamedev/ext/slamrec/ctrltrack-20261019052737.log`): port for parity first, fix tracking after, in C++.
5. **Threads and shared memory.** Broker ring and pose file use seqlocks with acquire/release and fences; port with
   `std::atomic_ref<std::uint64_t>` and the same fence placement; the libpimax callbacks run on the session thread
   (no `pimax_request` inside a callback). The C ABI must never throw or abort (Rust has panic guards in places):
   `noexcept` + catch-all + `pimax_cam_last_error` thread-local text.
6. **One archive, hidden symbols.** Monado also contains C++ (Mercury, OpenCV); export only `pimax_*`, everything
   else in `pibridge::` namespaces with hidden visibility, so no ODR or symbol clash. The Rust rule "one staticlib"
   disappears, but keep a single archive for a simple CMake switch.
7. **Embedded reference data.** The Rust embeds the Dream Air reference calibration XML, controller LED models and
   extrinsics (`pimax-device/devices/dream-air/reference/`); the Crystal currently falls back to those controller
   models. Copy them into PiBridge `devices/dream-air/reference/` (no serials) and keep `PIMAX_NO_REFERENCE`.
8. **Mercury hands / ONNX.** Not Rust and not active for the Crystal; unaffected except that the frames reach it
   through `pimax_cam_copy_tile` (covered by M1). The Mercury models have no licence: never copied into PiBridge.
9. **Eye.** The eye ABI must not start the chip while pibridge-eyed runs (one owner); for the Crystal it stays off
   (`PIMAX_EYE` unset). Port it as an adapter only, no new eye logic.
10. **Effort concentration.** `headpose.rs` (2,963) and `fuse.rs` (2,463) carry many tuned constants and env
   switches; port line by line with their tests, do not redesign (CONVENTIONS.md).

## 5. Room map and world anchor (R1-R5)

User request 2026-10-19: no Rust left; inside-out needs a persistent world anchor (Basalt starts a new world at every
start). Spec: omarchyvr `5ffbcc5` (the same pin): `rust/omarchyvr-anchor` (world offset, writer lease, head ring,
anchor.toml; AprilTag detect/tag/rig), `rust/omarchyvr-room` (ORB, stereo, PnP, map, session, synth, pngread, the
`omarchyvr-room` CLI) and `rust/omarchyvr-roommap` (align, loopclose, refine, floor, setup, store, light, export,
rec, pipeline, live, the `omarchyvr-roommap` CLI). The Monado side that consumes the files is
`forks/monado/src/xrt/drivers/pimax/pimax_anchor.h` (`PIMAX_ANCHOR`, `PIMAX_ANCHOR_GLIDE`, the head ring
`pimax-anchor.head`): read only, the file formats and the ring layout are matched byte for byte.

Method as for M0-M9: the Rust binaries `omarchyvr-anchor`, `omarchyvr-room`, `omarchyvr-roommap` built from a
`git archive` of `5ffbcc5` into `rust/target-port2` are the oracle; tool-against-tool difftests through
`monado-abi/difftest/run_difftest.sh` (stdout + the files written, byte for byte; run-dependent values masked);
`-ffp-contract=off -fno-fast-math -fwrapv`, saturating `as` casts, stable sorts where Rust sorts stably, `max_by` =
last maximum, `min_by` = first minimum, `Iterator::sum` of floats from -0.0, `powi` as LLVM expands it. Lab inputs
(recordings, the Crystal calibration) are read only and the tests that need them skip (77). Changes that alter
results (user 2026-10-19: "optimise while porting") sit behind a parity switch, are measured on the recordings
against the Rust behaviour and listed in `docs/libpibridge-roommap.md`; R1's files are a format and stay identical.

Layout: `libpibridge-anchor/` (`pibridge::anchor`, target `pibridge_anchor`), `libpibridge-roommap/` (`pibridge::roommap`, target
`pibridge_roommap`; the existing `libpibridge-room/` is the ZoneTweak chaperone port and is not touched),
`tools/roommap/` (`pibridge-roommap`, `pibridge-room`, `pibridge-anchor`), docs `docs/libpibridge-anchor.md`,
`docs/libpibridge-roommap.md`, `docs/pibridge-roommap.md`.

| # | Milestone | Done when (no headset) |
|---|---|---|
| R1 | `libpibridge-anchor`: offset file writer/reader, writer lease + flock, head ring reader, feed check (`feed_now`), `anchor.toml`, pose maths (interp, compute_offset, robust_mean) | files byte-identical to the Rust (offset text, lease text, ring layout = Monado's `pimax_anchor.h`); the 7 world.rs tests have twins; `pibridge-anchor status/clear` = `omarchyvr-anchor status/clear` |
| R2 | `libpibridge-roommap` room half: ORB, stereo, PnP, map (`PXRM` v1), session, synth, pngread; `pibridge-room` | the omarchyvr-room unit tests have twins; `pibridge-room demo/build/reloc/info/anchor` = `omarchyvr-room` (stdout and `map.bin`/`room.json` bytes) on the synthetic room and the recordings |
| R3 | roommap half: rec, pipeline, align, loopclose, refine, floor, setup, store, light, export | `pibridge-roommap build/relocalize/floor/export/view/info/cycle/eval` = `omarchyvr-roommap` on the recordings (EuRoC + Basalt `tracking.csv`): every file written identical |
| R4 | `pibridge-roommap` live parts: `relocalize/build --source broker`, `apply`, `auto`, `setup` (no in-headset panel), `floor follow/touch/set/show`, `light` | `apply`/`setup show`/`light` on replays identical; live paths against a replayed `.pxcam` broker + a synthetic head ring; AprilTag part of omarchyvr-anchor: follow-up (needs the AprilTag C library) |
| R5 | Crystal Super | device description of the rig (which cameras, pairs), evaluation on the Crystal recordings: map built?, second take relocalised?, residual cm/deg; what a good mapping take needs |

### Status (2026-10-19)

| # | Status | Where, and what the tests show |
|---|---|---|
| R1 | **done** | `libpibridge-anchor/` (`world`, `config`, `rustx`), `pibridge-anchor status/clear`; doc `docs/libpibridge-anchor.md`. Probe (4909 lines: offset text/parse, lease, rings + `at`, ring writer, compute_offset, interp, pose_error, robust_mean, 480 lease decisions, 192 feed cases, anchor.toml; f64 bits) byte-identical to the Rust library; the 8 world.rs tests have twins; Monado's own `pimax_anchor.h` (from the checkout, stub headers) parses our offset (placed at once, applied, other pid ignored) and writes the ring we read; `pibridge-anchor status/clear` = `omarchyvr-anchor` (13 difftests). New: `recenter_offset` (for Recenter). Not ported: AprilTag (detect/tag/calibrate/apply: needs the apriltag C library). |
| R2 | **done** | `libpibridge-roommap/` room half (`orb`, `pngread`, `pnp`, `stereo`, `map`, `session`, `synth`), `pibridge-room` (build/reloc/info/anchor/demo). Bit-identical to the Rust: probe of the synthetic room through the reference calibration (8721 lines: BRIEF pattern, virtual pinhole LUTs, ray tables, renders, undistortion, raw ORB, features with the low-light stretch, stereo points; f32/f64 bits) and 2000 ray traces; `pibridge-room` = `omarchyvr-room` in 23 difftests: the demo, `build` on the Crystal desk and worn takes (map.bin and room.json bytes), `reloc` (a second take, the take in its own map with `--save`), `info`, anchors set/list/rm, 11 error cases. Twins: orb 28, pnp 23 (the stress run is `#[ignore]`), tests.rs 6 + pngread 1 + 1 (C++ only: PNG filters, colour, errors), on the reference calibration (the Rust tests need the lab copy). |
| R3 | **done** | `libpibridge-roommap/` roommap half (`rec`, `pipeline`, `align`, `loopclose`, `refine`, `floor`, `setup`, `store`, `light`, `exportmap`, `live`), `pibridge-roommap`; docs `docs/libpibridge-roommap.md`, `docs/pibridge-roommap.md`. Byte-identical to `omarchyvr-roommap` on the Crystal takes (map.bin, correction.csv, floor.toml, preview.txt, room.json, roommap.toml, last-reloc.toml, map.json/html; wall-clock fields masked): build with loop closure + floor, `--refine`, `--no-loops`, relocalize (first fix, whole session, window fits), floor detect/drift/set, export, view, info, apply, cycle, errors. 28 twins (tests.rs 9, floor.rs 6, setup.rs 9, light.rs 4). Speed with identical output: inline casts/rounding, software popcount, the joint fit's rotation once per pose, `-fno-math-errno`: relocalize of the worn take 22.8 -> 13.6 s (Rust 11.9 s). |
| R4 | **done** (offline-tested) | `pibridge-roommap` live commands ported (`build/relocalize --source broker`, `apply`, `auto`, `setup fast|full|show`, `floor touch/set/show/follow`, `light`); difftests: light on a `.pxcam` (3 variants), setup show, auto without map / without Monado (status file), setup without a feed, apply dry/written (offset + lease bytes). Not run against a live Monado (no session touched). `setup` has no in-headset panel (prompts printed, head from the ring; the boundary trace of `setup full` needs the app's panel input). AprilTag part of omarchyvr-anchor: follow-up (needs the apriltag library). |
| R5 | done | Rig: cams 0/1 = lower pair (75 mm, 16 deg apart, 38.5 deg down), in the recordings; no device change. Maps build on all three takes (worn 22 kf / 3611 pts, floor -1.283 m; desk A 38 kf / 5497 pts; desk B 18 kf / 1114 pts, no floor on the desk takes). Worn take split at 60 s: recognised at once (12 frames), 4.4 cm / 0.22 deg first fix vs whole session, windows 2.5 cm RMS (= the tracker's own 5-7 cm return drift). Cross takes: A<->W recognised both ways (A in W after 77 s, W in A after 133 s; yaw +6.75 / -9.49 deg, height +0.32 / -0.33 m: inverse within 2.7 deg and a few cm); B not recognised in A or W, A and W not in B (B: few loops, 5 of 9 accepted). A good mapping take: turn the head and look around the whole room (still marks give few keyframes), look at the floor once, come back past earlier views (loops), light and texture; a seated desk take sees no floor. Follow-up: upper pair 2/3. |
