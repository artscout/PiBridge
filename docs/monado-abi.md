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
  (Mercury, OpenCV). Link with the C++ linker plus zlib and pthreads (Monado already links C++).
- `src/abi_id.cpp` (M9): the marker string `PIBRIDGE_PIMAX_ABI 1 spec omarchyvr <commit>` (the commit from
  `spec.txt`), kept by `used`/`retain` although nothing calls it and local after the `objcopy`. A Monado binary that
  links the archive carries it, so `grep -ac PIBRIDGE_PIMAX_ABI` tells a PiBridge-linked build from a Rust-linked one.
  Raise the `1` when PiBridge changes the C ABI.

## Tests (`ctest -R monado_abi`)

| Test | What |
|---|---|
| `monado_abi_headers` | the headers match `include/SHA256SUMS` |
| `monado_abi_headers_pin` | each header equals `git show 5ffbcc5:<path>` in the omarchyvr checkout (`PIBRIDGE_OMARCHYVR_DIR`, default `../omarchyvr`); notes headers whose working-tree copy has moved past the pin; skipped without the checkout |
| `monado_abi_symbols` | a shared object built from the whole archive exports only `pimax_*` |
| `monado_abi_marker` | that shared object carries the `PIBRIDGE_PIMAX_ABI 1 spec omarchyvr <commit>` string (M9) |
| `monado_abi_capi` | twins of the Rust `capi.rs` tests through the archive |
| `monado_abi_capi_slam` | twins of the Rust `capi_slam.rs` tests through the archive (reference calibration) |
| `monado_abi_lens_capi` | twins of the Rust pimax-lens `capi.rs` tests |
| `monado_abi_diff_*` | the difftests below |
| `monado_abi_diff_slam_*` | `difftest/slam_difftest.c` against `libpimax_cam.a` (M4), see below |
| `monado_abi_hands_capi` | twins of the Rust pimax-hands `ffi.rs`, `inhand_ffi.rs` and `tests/c_abi.rs` tests through the archive |
| `monado_abi_diff_hands_*` | `difftest/hands_difftest.c` against `libpimax_hands.a` (M6), see below |
| `monado_abi_ae_capi` | twins of the Rust `capi_ae.rs` ABI tests through the archive |
| `monado_abi_ae_gate` | libpibridge-cam's AE against libpibridge-hmd's gate: twin of the Rust `pimax-device/tests/allowlist.rs` `crystal_super_camera_ae_uses_its_own_windows_ranges`, and the property test (479 closed loops: random rooms x every `PIMAX_CAM_AE_*` override alone with 27 values each, NaN/inf/garbage included, and 150 random combinations): no target leaves the Crystal pair ranges, every payload handed out passes the gate as `camera_ae` (31589 reports) |
| `monado_abi_diff_ae_*` | `difftest/ae_difftest.c` against `libpimax_cam.a` (M3), see below |
| `monado_abi_pimax_capi` | twins of pimax-capi `tests/abi.rs` (full session against a fake headset, close while the caller holds the callback lock), `src/eye.rs`, `tests/eye_export.rs`, `tests/hands_export.rs` (every function of `pimax.h` 17, `pimax_eye.h` 7, `pimax_hands.h` 13 defined in the archive); the gate: `the_gate_sees_every_command` (Dream Air and Crystal Super: rates 0/72/90/60, attach 2/3, video prepare 72/90/60, haptics on slots 1-4 with extreme values, the request file, reads, unlisted sends; every byte that reached the fake device passes that model's `devices/*.toml`, nothing at 60 Hz, no haptic for slots 1/4 or AMP 0), `a_description_that_lists_nothing_sends_nothing` (zero bytes written, every call `-EPERM`), the compiled-in fallback descriptions, the request file |
| `monado_abi_diff_pimax_*` | `difftest/pimax_difftest.c` against `libpimax_capi.a` (M8), see below |
| `monado_abi_pimax_cli` | omarchyvr's unmodified `libpimax/tools/pimax-cli.c` built against the archive: usage, unknown subcommand, no headset on an empty sysfs (twin of `unmodified_pimax_cli_non_hardware_paths`) |
| `monado_abi_driver_imports` | every ABI function the built Monado Pimax driver objects import (`PIBRIDGE_MONADO_BUILD_DIR`, default `../ovr-build/monado`, read only; 80 today) is defined in the archive; skipped without the build |
| `monado_abi_diff_lens_{synth,crystal,reference}` | `difftest/lens_difftest.c` against `libpimax_lens.a`: synthetic scenarios, the Crystal conf+LUT (lab data, `../ext/lens`), the Dream Air reference profile |

## The difftest

`difftest/cam_difftest.c` is one C program, compiled twice: against omarchyvr's `libpimax_cam.a` (the oracle) and
against `libpibridge_pimax.a`. `difftest/run_difftest.sh` runs both and compares their standard output byte for byte.
Everything printed is deterministic: doubles as exact hex (`%a`), replay times relative to the first frame (a replay
rebases them onto the clock at its start), tiles as FNV-1a hashes, error texts in full.

| Mode | Covers |
|---|---|
| `layout` | size and offset of every field of the M1 structs as a C compiler sees the header, the version, the initial error |
| `clock` | `pimax_cam_dev_to_mono_ns` over 1960 combinations (u32 wrap, clamps at 0 and u64::MAX, invalid and NULL refs) |
| `calib FILE...` | parse of the file, every prefix (step 7), 9 structural / invalid-UTF-8 bytes at every 13th position, 18 replacements of the first 60 number tokens (`nan`, `+1.5`, `1e999`, `1_0`, U+00A0, ...), 31 synthetic documents (entities, nesting 64..67, comments, CDATA): return code + error text, or a hash of all fields and of every stereo (a, b < 6) and body pose (i < 6) |
| `load WORKDIR REFXML` | the default search in a scratch HOME: built-in reference, `PIMAX_NO_REFERENCE`, installed reference copy, `~/omarchyvr`, data copy with/without its marker, `PIMAX_REPO`, `~/.config/omarchyvr`, `PIMAX_CAM_CALIB`/`PIMAX_CALIB` (missing, set, empty), `used_path` truncation, explicit missing / directory / invalid UTF-8 / wrong root, no HOME, empty HOME |
| `replay FILE [N]` | every frame of a recording (or N frames looped): all `pimax_cam_frame_info` fields, the four tiles, a stride-700 copy, the stats, the end of the replay and the errors after it |
| `record SRC WORKDIR N` | recording through the ABI (raw and zlib 6): the file structure (headers, payload hashes for raw), the file replayed back, recording to a missing directory, NULL path, close while recording |
| `errors WORKDIR` | NULL handles everywhere, missing / quoted / invalid-UTF-8 names, missing broker socket, too long socket path, short / bad magic / bad version / bad stride / longer header / bad record magic / bad lengths / truncated header / truncated payload / short zlib / corrupt zlib recordings, `describe` into small buffers |
| `client SOCKET REF [SLOW_MS]` | frames from a running broker against a replay of the same file (`pibridge_cam_broker_cross`) |
| `hello SOCKET` | the raw PXCB hello, memfd, ring header and slot headers of a running broker, without any library |

### The SLAM / 6DoF difftest (`slam_difftest.c`, `slam.cmake`)

stderr goes into the compared output (the describe line and the drift reports). `-e K=V` / `-u K` set the
environment per run (every `HeadParams` switch is exercised alone or in groups); the driver clears the `PIMAX_SLAM_*`
family first.

| Test | Covers |
|---|---|
| `slam_layout` | sizes and offsets of the eight SLAM/track structs, `pimax_cam_track_default_params` |
| `slam_reference`, `slam_crystal` | `t_imu_cam` for cameras 0..7, `default_head` (also without IMU extrinsic and with one camera), 22 camera lists x 3 counts x 3 capacities (incl. U+00A0 / U+3000, `+1`, overflow), `write_basalt` with 11 gyro-bias cache contents, `XDG_CACHE_HOME` / `HOME` / neither, four cameras, NaN/inf noise, a 256-byte source without NUL, every error (bad / duplicate camera, no extrinsic, NULLs, a file in the way, a directory where a file goes): both files dumped. The Crystal's own XML is compared by hash only (`private`) |
| `slam_imu` | `slam_imu_load`: the default search (`HOME`, `PIMAX_SLAM_IMU` set / empty / missing), explicit missing file, a directory, 25 documents (scalars, wrong lengths, zero, negative, `1e999`, not an object, invalid UTF-8, BOM, duplicate keys) |
| `slam_errors` | NULL handles and outputs of every track call, extreme timestamps (wrapping), odd parameters (0 acquire poses, NaN speed limit, 0 stale time) |
| `slam_imulog` | `PIMAX_HEAD_IMU_LOG`: the file written (3000 IMU + repeated poses), `PIMAX_HEAD_IMU_LOG_MIN` short / bad / negative, the flag file, the kept earlier logs (fixed mtimes, `TZ=UTC`: the stamped names, the `-1`, `-2` collisions, pruning to 3, the `.prev` link), a missing directory, an empty name |
| `slam_synth`, `slam_synth_{raw,nodrift,nofuse,fuse,still,nostill}` | 13 synthetic head motions through the C ABI (walk, noisy velocity, loss in the same world / a new world, long loss with resets, a 6 cm tracker correction, jump + NaN + spin, recenter twice, the phase-1 entry point, desk with a drifting tracker (re-zero), desk pickup, sliding tracker walking and seated (drift reports), 12 acquire poses); every update hashed, every pose update / event and every 90 Hz query printed in full, plus info and big corrections; each with the default switches and six switch sets |
| `slam_replay_worn`, `slam_replay_worn_nodrift` | a recorded worn take (`~/.local/share/pibridge/recordings/20261019-070409`, 122 k IMU samples) with its recorded Basalt trajectory (`../ext/slam/study3/runs/L/out/tracking.csv`): y-up, 25 ms late, a Mahony 3DoF, causal velocities, queries at 90 Hz; drift hold on and off |
| `slam_replay_desk`, `slam_replay_desk_rezero0` | the desk/dark take `../ext/slamrec/euroc_20261019052737` (195 k IMU samples) with `../ext/slam/study2/runs/L/out/tracking.csv`, tracker resets emulated as `omarchyvr-slam stillgate` does (a fresh world at the current pose, another yaw): 42 re-zeros; and with `PIMAX_SLAM_STILL_REZERO=0` (the old dropout) |

### The auto-exposure difftest (`ae_difftest.c`, `ae.cmake`)

`pimax_cam_ae_update` skips replay frames, so the program runs its own small PXCB broker (no `FLAG_REPLAY`) in the
work directory and opens it with `pimax_cam_open("broker:SOCKET")`: every frame reaches the library as a live frame,
published in lock step. stderr is folded into stdout; every update prints the return value, all `pimax_cam_ae_out`
fields (payload in hex, floats as `%a`). A Crystal Super run exits 3 when a payload handed out for sending lies
outside Windows' pair ranges (lower 2200..10410 us / gain 13..449, upper 2200..8941 / 13..439, LED 40/62). 52 tests,
all byte-identical:

| Test | Covers |
|---|---|
| `ae_layout` | `pimax_cam_ae_out` size and offsets, the constants |
| `ae_create*` (7) | `create_model` with 14 names (spellings, invalid UTF-8, unknown), NULL, `create` (detection from this machine's USB); `PIMAX_CAM_AE_MODEL` crystal / dream / unknown; caps overrides; the old law; an override that panics in Rust (the panic text itself is not compared) |
| `ae_errors` | NULL handles and output, a camera before its first frame, an LED frame, a frame without metadata, one tracking frame, a replay broker's frame |
| `ae_scene` | the head tracker's brightness input: the same walk through `pimax_cam_track_*` with bright and with dark AE frames (`PIMAX_FUSE_AUTO=1`); the outputs differ ("yes") in both libraries |
| `ae_pxcam_{first,ctrl1,ctrl2}_{crystal,dream}`, `_first_detect_env`, `_first_old_law` | the recordings (lab data) served live: open loop on their own metadata, now = the recorded host time |
| `ae_synth_crystal_{1..6}`, `ae_synth_dream_{1..3}` | 60 s closed loops on a simulated sensor: random rooms every 2-16 s (radiance 0.03..3000, windows, light blobs, an upper pair of its own), 25 Hz tracking + LED frames, host-clock jitter and camera-restart gaps, every command applied two tracking frames later, random firmware start settings |
| `ae_synth_crystal_env{1..21}`, `_envmix`, `_detect`, `ae_synth_dream_env{1,2}` | each `PIMAX_CAM_AE_*` override (extremes, NaN, inf), a mix, the model from `PIMAX_CAM_AE_MODEL` |

The Rust `omarchyvr-slam ae` dry run is compared with `pibridge-slam ae` in `tools/slam` (`pibridge_slam_export_ae_*`,
`docs/pibridge-slam.md`).

### The libpimax difftest (`pimax_difftest.c`, `libpimax.cmake`)

A fake headset over SOCK_SEQPACKET socketpairs through `pimax_open_with_fds` (both libraries export it); every run
has a scratch `HOME`, `XDG_RUNTIME_DIR`, an empty `PIMAX_SYSFS_ROOT` and `PIBRIDGE_DEVICES_DIR` = this tree's
`devices/` (the C++ gate). Callbacks are logged on the session thread in arrival order (one interface at a time, so
the order is fixed); keepalives are shown only when their rate changes, the wake re-send is left out. Only commands
both libraries write are used (the gate's refusals are tested in `monado_abi_pimax_capi`). 21 tests, all
byte-identical:

| Test | Covers |
|---|---|
| `pimax_layout` | sizes/offsets of the four `pimax.h` structs and `pimax_eye_sample`, constants, the eye ABI version |
| `pimax_decode` | `pimax_decode_imu` / `_controller` over `p83`, `p82`, `ctrl_buttons_hub2`, `if3_log_coldstart` and 200000 noise reports (abi.rs generator): 366129 reports, every field (floats `%a`) |
| `pimax_session` | wake, rate 72, IMU (wrapped clock, extreme values; on IF1 and IF2), two controllers, one- and two-message 0x37, IF3 log lines (split, joined over reports, invalid UTF-8, a 640-byte line cut at 511), serial / version / raw requests with noise replies and small buffers, unanswered and negative timeouts, `-EMSGSIZE`, attach bursts 2/3, video prepare 90/72 and the rate after, 20 OpenXR haptic values (NaN, inf, 0, negative, tiny, huge, `INT64_MAX` ns, bad slots), replace and stop inside the 8 ms gap, 15 request-file texts (Unicode spaces, `+`, overflow, not UTF-8 (left in place), a directory), counts, unplug and every call after it |
| `pimax_replay_{p83,p82,hub2,if3,if3_on_if0}` | the recorded streams into IF2 / IF1 / IF3 / IF0 through the session: every callback (126738 IMU samples, 14723 and 87611 controller reports, 407 log lines, 426 replies) |
| `pimax_status37` | the 28 packets of `status37_golden.txt` into IF0: the status callbacks, each packet's message count matches the golden |
| `pimax_mahony` | `p83` through the session, omarchyvr-tools `orient.rs` (Mahony + factory IMU terms, here from the reference calibration `devices/dream-air/reference/headset-calibration.xml`) on the delivered samples: every 1000th quaternion in `%a`, and all 126 lines of `mahony_golden.txt` within 1e-9 (the Rust test's bound) |
| `pimax_find{,_long,_crystal,_partial,_none}` | `pimax_find_interfaces` over fake sysfs trees (`difftest/pimax_fake_sysfs.sh`): Dream Air, Crystal Super, missing interfaces, a keyboard and the bridge to ignore, paths cut at 63 bytes, NULL buffer |
| `pimax_nodevice`, `pimax_close` | `pimax_open` on an empty sysfs (timeouts, `-ENOTCONN`, prompt close); close while the caller holds the lock the disconnect callback needs; a session without callbacks |
| `pimax_eye`, `pimax_eye_cal` | the eye ABI without a chip (`PIMAX_EYE_USB` = a missing node, no power-on, no USB reset): NULL arguments, start with and without the calibration flag, no sample, stop |
| `pimax_commands` | the full wire of a session (attach, video prepare, rates, haptics on both hands, the request file) |

### The controller difftest (`hands_difftest.c`, `hands.cmake`)

Every run has a scratch `HOME` and `XDG_RUNTIME_DIR` (`@W@`); the files the fuser writes (`ctrl-imu.json`, the
`PIMAX_CTRL_IMU_LOG` logs) are dumped by `difftest/hands_dump.sh` and compared too. Floats as `%a`, long sequences as
FNV-1a hashes of the raw output structs plus a full line every so often. 80 tests, all byte-identical:

| Test | Covers |
|---|---|
| `hands_layout`, `hands_session` | struct sizes/offsets of the three headers, the ABI version, the mode names; pimax-hands `tests/c/session.c` (PASS) |
| `hands_synth_{1..4}` | 60000 updates per seed through `update` / `update_report` / `reset` / `get` on two hands: clean phases, flickering touch bits, NaN / inf / huge / denormal / -0 inputs, raw reports with every byte value and odd stick values, time steps of 0..3.6 s, repeats and steps back; the demo input over 25 s and at odd times; `grip_to_wrist` (any `is_right`), `tracked_aim_to_grip`, NULLs |
| `hands_poses*` (17) | `controller_poses` for 400 head orientations (NULL, zero, NaN, inf, huge, straight up/down, random) x both hands, `aim_to_grip`, `controller_describe` into buffers of 0..11 bytes, with `PIMAX_CTRL_REST`/`PIMAX_CTRL_AIM` default, legacy, numbers (3 and 4, spaces, rounding of `{:.0}`/`{:.2}` at half-way values), every kind of invalid value |
| `hands_replay_{ctrl_buttons_hub2,p82,p83}`, `hands_replay_hub2_log` | the recorded report streams of the pimax-hands fixtures through `hands_update_report`, `inhand_report`/`held`/`describe` and `ctrlfuse_report`/`get`/`describe` (two linked hands); one with the gyro off and the IMU log on (`PIMAX_CTRL_IMU_LOG_MIN` = 3 s) |
| `hands_inhand_*` (14) | the five in-hand fixtures of the session 2026-10-16-1818 through `pimax_inhand_report`: untouched, picked up first, the synthetic typing knocks, silence after half, `PIMAX_CTRL_INHAND_DOWN_S` 2 and " 1.3 " |
| `hands_inhand_synth_{1..7}` | 100000 synthetic reports on two controllers (still, waved, tremor, slow tilt, buttons and analog drift, silence, knocks; one or two IMU samples): every transition with its describe line, `publish` in every mode with NULL controllers (the status file, `pid` masked), `poll_mode` with ten request texts (including invalid UTF-8, which Rust leaves in place), `clear_request`, `mode_from_env` under every `PIMAX_CTRL_INHAND` value, `PIMAX_CTRL_INHAND_DOWN_S` valid / out of range / not a number |
| `hands_fuse_nulls` | NULL handles and pointers of every `pimax_ctrlfuse_*` call, hands -1..3, zero quaternions, extreme times (0, 2^63, u64::MAX) for fixes, reports and queries |
| `hands_fuse_{0..6}`, `_long`, `_fit*` | synthetic sessions (two linked hands, head turning and walking, gyro reports at 250 Hz with one or two samples, 25 Hz fixes with 5..14 LEDs and residuals, flipped few-LED solves, jumps, a hand on top of the other, the wrong side, out of reach, out-of-view gaps and short dropouts, desk periods; jittery host clock, reordered device clock, stale and zero frame times, the hands not linked; scenario 6 clean so the online fit replaces the built-in calibration and saves it): every query state/pose/info hashed at 90 Hz, state changes in full, describe lines |
| `hands_fuse_log*` | `PIMAX_CTRL_IMU_LOG`: every I / O / P line, the 10 min window, a short window with and without `omarchyvr-imu-log.on`, `NaN` minutes |
| `hands_fuse_{noimu,forceimu,nodefault,legacy,switches1..4,dr_desk}` | every `PIMAX_CTRL_TRACK_*` / `PIMAX_CTRL_IMU_*` switch alone or in groups, including invalid values |
| `hands_fuse_calib_*`, `hands_fuse_home_calib` | the calibration file (`PIMAX_CTRL_IMU_CALIB` or the default path): both hands, one hand, damaged, without the format marker, Unicode whitespace and odd number forms, a directory in the way, and `NaN` (both libraries abort: `--exit 134`) |

The oracle is built from the pinned commit, without touching the omarchyvr checkout (`git archive` into a scratch
folder): `monado-abi/difftest/build-oracle.sh <omarchyvr> <omarchyvr>/rust/target-port2 [SCRATCH]` (CPU only,
`nice -n 19`, offline; it builds `pimax-cam`, `pimax-lens`, `omarchyvr-slam`, `pimax-hands`, `pimax-ctrltrack` and
`pimax-capi`: `libpimax_cam.a`, `libpimax_lens.a`, `libpimax_hands.a`, `libpimax_capi.a` and the `pimax-cam`,
`pimax-lens`, `omarchyvr-slam`, `pimax-ctrltrack` tools). CMake finds it at `PIBRIDGE_RUST_ORACLE_DIR` (default
`../omarchyvr/rust/target-port2/release`, built from `5ffbcc5`; the earlier `target-port` of `381eea2` is gone);
without it the difftests say "skipped". The lab inputs (`PIBRIDGE_LAB_CAM_DIR` = `../ext/cam`: `first.pxcam`,
`ctrl1.pxcam`, `ctrl2.pxcam`; `PIBRIDGE_LAB_CALIB_DIR` = `../ext/calib/crystal-calibration.xml`; for M4
`PIBRIDGE_LAB_SLAMREC_DIR` = `../ext/slamrec`, `PIBRIDGE_LAB_SLAM_DIR` = `../ext/slam`, `PIBRIDGE_LAB_RECORDINGS_DIR` =
`~/.local/share/pibridge/recordings`; for M5 `PIBRIDGE_LAB_LENS_DIR` = `../ext/lens`) are read only and optional the same
way.

## The PiBridge-linked Monado (M9)

A second Monado build that links `libpibridge_pimax.a` instead of the Rust `libpimax_capi.a`, `libpimax_cam.a` and
`libpimax_lens.a`. The live build stays as it is; the user A/Bs the two (PiBridge's `rec_monado_service` key picks the
binary).

**The switch** (`patches/monado/monado-pimax-use-pibridge.patch`, Monado `src/xrt/drivers/CMakeLists.txt` only, the
driver C files do not change): `PIMAX_USE_PIBRIDGE` (default OFF; OFF builds exactly as before),
`PIMAX_PIBRIDGE_LIB=<pibridge build>/monado-abi/libpibridge_pimax.a`, `PIMAX_PIBRIDGE_INCLUDE=<pibridge>/monado-abi/include`. ON:
one imported archive (C++ link language, plus zlib, pthreads, dl, m, rt) takes the place of the three Rust archives
for `drv_pimax` and `pimax_ht_bench`, and all seven headers come from `PIMAX_PIBRIDGE_INCLUDE` (byte-identical copies,
`monado_abi_headers`). Configure checks that the archive and each header exist.

**Source**: omarchyvr's `forks/monado` at `0dff9fa2a` plus the Crystal Super driver diff (still uncommitted in
omarchyvr: `pimax_hmd.c`, `pimax_interface.h`, `pimax_prober.c`, `target_lists.c`), then the patch. A local clone of
the fork on a branch `pibridge-abi` (two local commits, never pushed), not a `git worktree`: Monado's
`GetGitRevisionDescription.cmake` cannot find the refs of a worktree of a submodule (it looks in the superproject's
`.git`) and configure fails.

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
which the SLAM difftests, `monado_abi_capi_slam` and `monado_abi_diff_ae_scene` set): 1 (`PIMAX_SLAM_SEED_BIAS`), 2
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
