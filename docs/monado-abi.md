# monado-abi

The C ABI the Monado Pimax driver links, implemented on PiBridge's C++ libraries: one archive
`build/monado-abi/libpibridge_pimax.a` and the headers in `monado-abi/include/`, verbatim copies of omarchyvr's at the
pinned commit (`monado-abi/spec.txt`: `5ffbcc5`, the AE commit on top of `381eea2`; `include/SHA256SUMS`). Plan: `docs/research/rust-port-plan.md`
(M0 harness, M9 the switch in Monado). Today (M0-M7) it implements all of `pimax_cam.h` (with the auto-exposure
entry points, M3), `pimax_lens_rs.h`, and the controller ABIs `pimax_hands.h`, `pimax_ctrlfuse.h` and
`pimax_inhand.h`, and (M8) libpimax `pimax.h` and the eye ABI `pimax_eye.h`: every header the driver includes.

## The archive

- `src/cam_capi.cpp`: the `pimax_cam_*` functions over `libpibridge-cam` (`docs/libpibridge-cam.md`). Every entry point is
  `noexcept` with a catch-all, like the Rust `catch_unwind` guard; errors return -1 / NULL / 0 and
  `pimax_cam_last_error()` is the thread-local text, the same texts as the Rust library's ("open \"x\": No such file
  or directory (os error 2)", "end of replay", "panic inside pimax_cam" for anything unexpected). `pimax_cam_version()`
  says "0.1.0" like the Rust crate. Static asserts check that the library's own structs and the header's agree field
  by field.
- `src/slam_capi.cpp`: the `pimax_cam_slam_*` and `pimax_cam_track_*` functions over libpibridge-cam's `slam`,
  `headpose`, `drift` and `imulog` (`docs/libpibridge-cam.md`), same error texts and stderr lines (`6DoF output:
  PIMAX_HEAD_TILT ...` at create, `6DoF drift: ...` reports). `src/capi_common.h` holds what both camera files share
  (the thread-local error, the guard, `from_c`). `pimax_cam_slam_write_basalt` takes its gyro bias from the driver's
  cache and its accel bias from `from_c`, which (as in Rust) carries no raw `Stateinit` attributes, so the recorder's
  calib has `calib_accel_bias` 0 (see "Follow-ups" below).
- `src/ae_capi.cpp` (M3): `pimax_cam_ae_create`, `pimax_cam_ae_create_model`, `pimax_cam_ae_destroy`,
  `pimax_cam_ae_update` over libpibridge-cam's `ae` (`docs/libpibridge-cam.md`). Same stderr lines as the Rust library
  (`PIMAX_AE_MODEL: <id> (<where from>); ...` and `PIMAX_AE_EXTENDED: ...` at create, the governor's events, the
  "not sent" line), same error texts ("null controller", "null camera", "null output", `unknown headset model "x"
  (dream-air or crystal-super)`, "panic inside pimax_cam" where Rust panics), and every update publishes the darker
  pair's mean to the head tracker (`headpose::scene`, read by `pimax_cam_track_update_vio`). Only live tracking frames
  with metadata drive the loop (replay and LED frames return 0). For the Crystal Super a target outside Windows'
  pair ranges is never handed out; the driver's report then still passes PiBridge's gate (`camera_ae` in
  `devices/crystal-super.toml`, `monado_abi_ae_gate`).
- `src/lens_capi.cpp`: the `pimax_lens_rs_*` functions over libpibridge-lens (`docs/libpibridge-lens.md`); no exception
  leaves the ABI (create returns NULL, the mask call -1).
- `src/hands_capi.cpp`, `src/ctrlfuse_capi.cpp`, `src/inhand_capi.cpp` (M6): every `pimax_hands_*`,
  `pimax_ctrlfuse_*` and `pimax_inhand_*` function over libpibridge-ctrl's hands half (`docs/libpibridge-ctrl.md`),
  including `pimax_hands_reset` (the driver does not call it). NULL is tolerated everywhere exactly as in the Rust
  ffi; one mutex per ctrlfuse / inhand object (reports on the session thread, queries on the compositor), the linked
  other hand is shared like the Rust `Arc<Mutex<Fuser>>` and never locked while the own lock is held;
  `PIMAX_CTRL_REST`/`PIMAX_CTRL_AIM` are read once per process. Same env names, describe lines, status file,
  request file and `ctrl-imu.json` bytes as the Rust library; where the Rust library would panic (NaN reaching a
  `partial_cmp().unwrap()`) the process aborts as it does with Rust.
- `src/pimax_capi.cpp` (M8, `libpimax.cmake`): all 17 functions of `pimax.h` plus the extension
  `pimax_open_with_fds` (as in Rust) and the PiBridge-only `pimax_open_with_fds_pid` (a fake link that claims to be
  the Dream Air or the Crystal Super, for the gate tests), over `pibridge::hmd::Session` (`docs/libpibridge-hmd.md`):
  same struct layouts, callbacks on the session thread (suppressed once `pimax_close` starts), `on_log` lines as
  UTF-8 with invalid bytes replaced and cut at 511 bytes, the same return conventions (0 / length / -errno),
  `pimax_haptic` (OpenXR value -> `58 09 07 SS AMP DUR`, 8 ms per hand, newest wins), `pimax_haptic_poll_request`
  (`$XDG_RUNTIME_DIR/pimax-haptic-request`), `pimax_haptic_counts`, the pure decoders. **The deliberate
  difference: every report goes through the HID allowlist gate of the connected model** (`devices/<id>.toml`: the
  PiBridge devices folders, `PIBRIDGE_DEVICES_DIR` first, else the two descriptions compiled into the archive). A report
  the description does not list is not written and the call returns `-EPERM` (Rust writes it); `pimax_request` of
  an unlisted command returns `-EPERM` at once instead of `-ETIMEDOUT`; `pimax_video_prepare` at an fps other than
  72/90 stops at the refused announce (`-EPERM`; the first two frames went out). Refusals and session lines go to
  stderr as `libpimax: ...` (each distinct line once in a row; Rust's `log` lines are silent in Monado). A NULL
  handle returns `-EINVAL` / 0 instead of crashing.
- `src/eye_capi.cpp` (M8): the 7 `pimax_eye_*` functions over libpibridge-eye's `live::Tracker` (`RealDevice`;
  `PIMAX_EYE_*` read as `PIBRIDGE_EYE_*` first), same return codes and staleness rule (newest sample older than
  300 ms = `PIMAX_EYE_STALE`). Its one HID command, the eye-module enable `eye_power_on`, goes through the same
  allowlist gate (the headset's interface 0, opened for that write). Not used for the Crystal Super (`PIMAX_EYE`
  unset; pibridge-eyed owns the chip).
- `libpibridge_pimax.a` is one relocatable object: `ld -r --force-group-allocation` over the C ABI and every
  `libpibridge-cam`, `libpibridge-lens`, `libpibridge-ctrl` (hands half), `libpibridge-hmd`, `libpibridge-eyemath` and `libpibridge-eye`
  object (the last three compiled once more inside monado-abi with the parity flags and hidden visibility), then `objcopy --keep-global-symbol=pimax_*`. A shared object that links the whole archive
  (Monado's `driver_monado.so`) exports exactly the C ABI: no `std::` template instantiation or libstdc++ inline
  variable leaks (`-fvisibility=hidden` alone would leave ~40 of them), so nothing clashes with Monado's own C++
  (Mercury, OpenCV). Link with the C++ linker plus zlib, libjpeg and pthreads (Monado already links C++).
- `src/abi_id.cpp` (M9): the marker string `PIBRIDGE_PIMAX_ABI 1 spec omarchyvr <commit>` (the commit from
  `spec.txt`), kept by `used`/`retain` although nothing calls it and local after the `objcopy`. A Monado binary that
  links the archive carries it, so `grep -ac PIBRIDGE_PIMAX_ABI` tells a PiBridge-linked build from any other.
  Raise the `1` when PiBridge changes the C ABI.

## Tests (`ctest -R monado_abi`)

| Test | What |
|---|---|
| `monado_abi_headers` | the headers match `include/SHA256SUMS` |
| `monado_abi_symbols` | a shared object built from the whole archive exports only `pimax_*` |
| `monado_abi_marker` | that shared object carries the `PIBRIDGE_PIMAX_ABI 1 spec omarchyvr <commit>` string (M9) |
| `monado_abi_capi` | twins of the Rust `capi.rs` tests through the archive |
| `monado_abi_capi_slam` | twins of the Rust `capi_slam.rs` tests through the archive (reference calibration) |
| `monado_abi_capi_slam_pibridge` | PiBridge's own 6DoF additions through the archive (the recorder's calibration in use) |
| `monado_abi_lens_capi` | twins of the Rust pimax-lens `capi.rs` tests |
| `monado_abi_hands_capi` | twins of the Rust pimax-hands `ffi.rs`, `inhand_ffi.rs` and `tests/c_abi.rs` tests through the archive |
| `monado_abi_ae_capi` | twins of the Rust `capi_ae.rs` ABI tests through the archive |
| `monado_abi_ae_gate` | libpibridge-cam's AE against libpibridge-hmd's gate: twin of the Rust `pimax-device/tests/allowlist.rs` `crystal_super_camera_ae_uses_its_own_windows_ranges`, and the property test (479 closed loops: random rooms x every `PIMAX_CAM_AE_*` override alone with 27 values each, NaN/inf/garbage included, and 150 random combinations): no target leaves the Crystal pair ranges, every payload handed out passes the gate as `camera_ae` (31589 reports) |
| `monado_abi_pimax_capi` | twins of pimax-capi `tests/abi.rs` (full session against a fake headset, close while the caller holds the callback lock), `src/eye.rs`, `tests/eye_export.rs`, `tests/hands_export.rs` (every function of `pimax.h` 17, `pimax_eye.h` 7, `pimax_hands.h` 13 defined in the archive); the gate: `the_gate_sees_every_command` (Dream Air and Crystal Super: rates 0/72/90/60, attach 2/3, video prepare 72/90/60, haptics on slots 1-4 with extreme values, the request file, reads, unlisted sends; every byte that reached the fake device passes that model's `devices/*.toml`, nothing at 60 Hz, no haptic for slots 1/4 or AMP 0), `a_description_that_lists_nothing_sends_nothing` (zero bytes written, every call `-EPERM`), the compiled-in fallback descriptions, the request file |
| `monado_abi_driver_imports` | every ABI function the built Monado Pimax driver objects import (`PIBRIDGE_MONADO_DRIVER_BUILD_DIR`, default `../pibridge-monado-build`, read only; 80 today) is defined in the archive; skipped without the build |

The difftests that compared this archive byte for byte with omarchyvr's Rust archives (the "oracle", built from the
pinned commit) were removed once the port was finished (2026-10-22): PiBridge's C++ is the only implementation now.
Their results are recorded in `docs/research/rust-port-plan.md`.

## The PiBridge-linked Monado (M9)

The Monado build that links `libpibridge_pimax.a` instead of the Rust `libpimax_capi.a`, `libpimax_cam.a` and
`libpimax_lens.a`. It is the only Monado build PiBridge uses (the Rust-linked build was dropped 2026-10-20);
PiBridge's `rec_monado_service` key picks the binary (empty = the usual lookup).

**The switch** (`patches/monado/monado-pimax-use-pibridge.patch`, Monado `src/xrt/drivers/CMakeLists.txt` only, the
driver C files do not change): `PIMAX_USE_PIBRIDGE` (default OFF; OFF builds exactly as before),
`PIMAX_PIBRIDGE_LIB=<pibridge build>/monado-abi/libpibridge_pimax.a`, `PIMAX_PIBRIDGE_INCLUDE=<pibridge>/monado-abi/include`. ON:
one imported archive (C++ link language, plus zlib, pthreads, dl, m, rt) takes the place of the three Rust archives
for `drv_pimax` and `pimax_ht_bench`, and all seven headers come from `PIMAX_PIBRIDGE_INCLUDE` (byte-identical copies,
`monado_abi_headers`). Configure checks that the archive and each header exist.

`pimax_cam.h` carries one PiBridge addition: `PIMAX_CAM_FLAG_NO_DEV_TIME` (8) on the frames of the Motion-JPEG camera
profile (`docs/libpibridge-cam.md`). It is backward compatible both ways: an older archive never sets it, and an
older driver ignores it and keeps such frames untimed (3DoF). The marker `PIBRIDGE_PIMAX_ABI 1` stays as it is.

**Source**: bundled in this repository at [`third_party/monado/`](../third_party/monado) — a snapshot of our Monado
fork (Monado plus the Pimax Crystal Super driver, with the `PIMAX_USE_PIBRIDGE` switch already applied in
`src/xrt/drivers/CMakeLists.txt`), so a user build needs no clone and no separate patch step. The snapshot has no `.git`;
the fork's own history is not carried (PiBridge's repository history is the record). Build it out-of-tree with the three
options below. (Historically this came from omarchyvr's `forks/monado`; we now vendor it so the inside-out driver is
self-contained here.)

**Build dir**: never a fresh cache (omarchyvr lesson 6 and 2026-10-15). Copy the live build's `CMakeCache.txt`,
replace only the source and build paths (`CMAKE_HOME_DIRECTORY`, `CMAKE_CACHEFILE_DIR`, `XRT_SOURCE_DIR`,
`XRT_BINARY_DIR`, `AUX_BINDINGS_DIR`, `CMAKE_FIND_PACKAGE_REDIRECTS_DIR`, the `_OXR_MANIFEST_*` paths, the header
comment), append the three options, then configure and build:

```
sed -e 's|<live build>|<new build>|g' -e 's|<live source>|<new source>|g' <live build>/CMakeCache.txt > <new build>/CMakeCache.txt
cat >> <new build>/CMakeCache.txt <<EOF
PIMAX_USE_PIBRIDGE:BOOL=ON
PIMAX_PIBRIDGE_LIB:FILEPATH=<pibridge build>/monado-abi/libpibridge_pimax.a
PIMAX_PIBRIDGE_INCLUDE:PATH=<pibridge>/monado-abi/include
EOF
nice -n 19 cmake -S <new source> -B <new build> && nice -n 19 ninja -C <new build> -j8
```

Every other option stays the live one (`XRT_BUILD_DRIVER_PIMAX=ON`, `PIMAX_CAM=ON`, `PIMAX_USE_RUST_LIBPIMAX=ON`,
`XRT_HAVE_SYSTEM_CJSON=OFF`, ONNX Runtime, OpenCV, handtracking, `RelWithDebInfo`); a sorted diff of the two caches
after configure shows only the three options (and the compiler entries' type FILEPATH -> STRING, same values).
Rebuild the PiBridge archive first when PiBridge changed (`ninja -C <pibridge build>`): the Monado build links it by path.
A relink drops `cap_sys_nice`; it is restored by the user / the app, never by an agent.

**Offline acceptance** (2026-10-19, results in `docs/research/rust-port-plan.md`, M9): markers (`grep -ac` and exact
counts of the omarchyvr NEEDLES plus `PIMAX_AE_MODEL` and `PIBRIDGE_PIMAX_ABI`) on `monado-service` and
`driver_monado.so` against the live binaries; no Rust symbol (`_R...` v0 mangling, `rust_eh_personality`, `__rust_*`)
and no `.cargo`/`rustc` path in any of the four binaries, the same `ldd`, `driver_monado.so` exports the live set
minus the Rust leaks plus `pimax_open_with_fds_pid`; `monado-cli slambatch` (deterministic Basalt) from both builds
on the recorded takes; pimax-hands `tests/c/driver_glue.c` linked against each build's real `pimax_ctrl.c.o` and
its archive; `pimax_ht_bench` from both builds.

## Allowlist entries the port added (`devices/crystal-super.toml`)

- `camera_ae` (M3): copied verbatim with omarchyvr `5ffbcc5`'s whole file: `58 19 51 00 01 32` + e0..e3, g0..g3, led_e,
  led_g (u16 LE), lower pair 2200..10410 us / gain 13..449, upper pair 2200..8941 / 13..439, LED 40/62; approved by
  the user 2026-10-19; source: all 14 takes of the Windows capture 2026-10-17 (16751 commands, first: take cold2
  t=27.880, 2200/13; `analysis/crystal-super-ae.md`).
- `haptic` (M8, user 2026-10-19 in chat): `58 09 07 {slot:u8} {amp:u8} {dur:u16le} 00 00 00`, slot 2|3, AMP 1..100,
  DUR 1..65535 ms; Crystal source: take `slam`, 8x `58 09 07 03 19 2c 01` (Pimax Play's test pulse, right, 25 %,
  300 ms) at t=31.505..99.452. Slot 2 and the other AMP/DUR values are seen only in the Dream Air capture of
  2026-10-15 (takes 14/15, omarchyvr `docs/STEAMVR.md` section 6; the same controllers).
- Controller attach burst (`58 04 16 SS 1e`, `58 03 50 SS`, `58 07 0a SS 00 00 00 10`, `58 03 03 SS`, slots 2/3)
  and `pimax_video_prepare` (`58 05 5b 05 02 02`, `58 04 29 00 01`, `58 0d 1a 05 02 48|5a`, `58 0d 1a 05 05`): already
  listed in omarchyvr's file with capture sources; the port added `approved = "user, 2026-10-19 (chat): ..."` lines and
  the take times (attach: cold2 t=39.420..39.476, cold1, controllers t=29.598/40.676, lighthouse, lighthouse2; video
  prepare: cold2 t=25.750/25.751/25.962/26.058, cold1 t=48.165, settings, sleep-wake). Every frame the C++ libpimax
  sends was found byte for byte in the decoded takes; nothing the driver sends had to stay refused.

## Follow-ups (not in the parity path; each behind a switch)

Done 2026-10-19 (`docs/research/head-tracking-improvements.md`; on by default, all off with `PIBRIDGE_HEAD_PARITY=1`,
which `monado_abi_capi_slam` sets): 1 (`PIMAX_SLAM_SEED_BIAS`), 2
(`PIMAX_SLAM_RECORD_ACCEL_BIAS`: the SLAM_CONFIG calibration is copied into the take; test `monado_abi_capi_slam_pibridge`)
and 3 (drift hold off by default), plus the accel bias in the output layer and the rotation prediction with angular
acceleration. The list as it was:

From the SLAM study (`docs/research/slam-imu-intrinsics.md`), to do after the switch to the C++ archive:

1. Seed the output layer's gyro bias from the driver's cache (the value `write_basalt` already reads), so the still
   test and the tilt correction work from the first second instead of after about 30-40 s of learning
   (`HeadTracker::pred.gyro_bias` at `track_create`; proposed switch `PIMAX_SLAM_SEED_BIAS=1`).
2. The recorder's `pimax-basalt-calib.json` should carry the calibration the tracker used: today
   `pimax_cam_slam_write_basalt` gets the calibration through `pimax_cam_calib` (`from_c`), which has no raw
   `Stateinit` attributes, so `calib_accel_bias` is 0 while the live tracker's SLAM_CONFIG calib has the factory
   `aBias`. Fix: pass the factory accel bias (a new entry point or a field taken from the loaded XML) behind
   `PIMAX_SLAM_RECORD_ACCEL_BIAS=1`.
3. Drift hold off by default for the Crystal (or on only when the tracker is starved); the switch exists
   (`PIMAX_SLAM_DRIFT_HOLD`), only the default would change.
