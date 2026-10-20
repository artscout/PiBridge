# libpibridge-ctrl

The Pimax controllers without hardware access. Two halves, each in its own CMake file of `libpibridge-ctrl/`:

- **hands** (`hands.cmake`, milestone M6 of `docs/research/rust-port-plan.md`): controller-driven hands, the optical
  + IMU pose fuser and the in-hand detector. Ported from omarchyvr `rust/pimax-hands` (MIT OR Apache-2.0) at the
  pinned commit `381eea2` (`monado-abi/spec.txt`); the Rust code is the specification and its tests have C++ twins
  here. The C ABI the Monado Pimax driver calls (`pimax_hands.h`, `pimax_ctrlfuse.h`, `pimax_inhand.h`) is in
  `monado-abi/src/{hands,ctrlfuse,inhand}_capi.cpp` and goes into `libpibridge_pimax.a` (`docs/monado-abi.md`).
- **tracker** (`track.cmake`, M7): see below.

Either half can be left out of a build with `PIBRIDGE_SKIP_PARTS=libpibridge-ctrl/hands` (or `libpibridge-ctrl/track`).

## Hands (M6)

CMake targets `pibridge_ctrl_obj` (objects, in `libpibridge_pimax.a`) and `pibridge_ctrl` (static, for tests and tools),
namespace `pibridge::ctrl` (`pibridge::ctrl::fuse`, `pibridge::ctrl::inhand`, `pibridge::ctrl::touch`), headers in
`libpibridge-ctrl/include/pibridge/ctrl/`. No hardware, no threads of its own, no Qt. Depends on libpibridge-cam's `rustfmt.h`
(Rust number formatting and parsing) and `io.h` (Rust-like file and environment helpers). Built with libpibridge-cam's
parity flags (`-ffp-contract=off -fno-fast-math -fwrapv`, hidden visibility).

| Header | What | From |
|---|---|---|
| `filter.h` | One Euro filter and the critically damped smoother (f32) | filter.rs |
| `mapping.h` | `Input`, `Params`, `HandState` (trigger / grip / touch -> finger curls, the thumb state machine), `to_joints` (Monado's joint layout) | mapping.rs |
| `report.h` | a decoded controller report (`PIMAX_BTN_*`, byte 49 touch bits) -> `Input` | report.rs |
| `pose.h` | `Pose`, `grip_to_wrist` (Monado's `ht_ctrl_emu` grip rule inverted) | pose.rs |
| `touch.h` | grip of a tracked controller from its aim (the Oculus Touch / Quest 2 components SteamVR uses) | touch.rs |
| `rest.h` | where untracked controllers are assumed to be: `Rest` (`PIMAX_CTRL_REST`), `AimMode` (`PIMAX_CTRL_AIM`), `controller_poses` | rest.rs |
| `demo.h` | the scripted demo input (`PIMAX_CTRL_HANDS_DEMO` in the driver) | demo.rs |
| `inhand.h` | `inhand::InHand`: held or lying on the desk, from the controller IMU and inputs | inhand.rs |
| `fuse.h` | `fuse::Fuser`: optical fixes + controller gyro -> the shown aim pose (extrapolation, arm-model hold, still lock, re-entry confirmation, reacquire blend, body model), the online gyro calibration fit and `ctrl-imu.json`, the IMU log | fuse.rs |
| `rust.h` | the Rust semantics the port needs beyond `rustfmt.h`: f32 clamp/min/max, saturating `as` casts, `{:e}`, `char::is_whitespace`, `Path::with_extension`, the panic-abort | Rust core / std |

Environment (the Rust names, unchanged): `PIMAX_CTRL_REST`, `PIMAX_CTRL_AIM` (read once per process);
`PIMAX_CTRL_TRACK_IMU`, `PIMAX_CTRL_TRACK_LOST_S`, `PIMAX_CTRL_TRACK_EXTRAP_MS`, `PIMAX_CTRL_TRACK_BLEND_MS`,
`PIMAX_CTRL_TRACK_HYST` (kill switch of the 2026-10-17 retune), `PIMAX_CTRL_TRACK_FILTER`,
`PIMAX_CTRL_TRACK_STILL_LOCK`, `PIMAX_CTRL_TRACK_FOLLOW_YAW`, `PIMAX_CTRL_TRACK_BLEND_MPS`, `PIMAX_CTRL_TRACK_DR_MS`,
`PIMAX_CTRL_TRACK_FOREARM`, `PIMAX_CTRL_IMU_FIT`, `PIMAX_CTRL_IMU_DEFAULT`, `PIMAX_CTRL_IMU_CALIB`,
`PIMAX_CTRL_IMU_LOG` (+ `PIMAX_CTRL_IMU_LOG_MIN`, `$XDG_RUNTIME_DIR/omarchyvr-imu-log.on`) at each
`pimax_ctrlfuse_create`; `PIMAX_CTRL_INHAND`, `PIMAX_CTRL_INHAND_DOWN_S`.

Files (the Rust paths and bytes): `~/.config/omarchyvr/ctrl-imu.json` (or `PIMAX_CTRL_IMU_CALIB`; read at create,
written atomically through `<name>.json.tmp<pid>` when the online fit improved: every 2000 IMU samples at most every
30 s, and on destroy), `$XDG_RUNTIME_DIR/pimax-ctrl-inhand` (status, through `pimax-ctrl-inhand.tmp`) and
`$XDG_RUNTIME_DIR/pimax-ctrl-inhand-mode` (run-time mode request, read and deleted), `/tmp` when
`XDG_RUNTIME_DIR` is unset or empty. Text written or returned (describe lines, the JSON, the log lines) is
byte-identical to the Rust library's.

Numerics: every expression keeps the Rust order and type (f32 stays `float`: the filters, the curls, the joint
curls; the poses and the fuser are f64); `f32::min/max` and `f64::min/max` take the other operand for NaN
(`std::fmin`/`fmax`); `clamp` keeps NaN; `as` casts from floats saturate (NaN -> 0); `Iterator::sum` of floats
starts at -0.0; `max_by` returns the last of equal maxima, `min_by_key` the first of equal minima, `sort_by` is
`std::stable_sort`; release-mode wrapping integer arithmetic (u64 timestamps, the i64 clock offset). Where the Rust
code would panic (a NaN reaching a `partial_cmp().unwrap()`: a calibration file with `NaN` in it, NaN poses feeding
the fit) the C++ aborts the process, as a Rust panic inside an `extern "C"` function does.

### Tests (`ctest -R pibridge_ctrl`)

| Test | Twins of |
|---|---|
| `pibridge_ctrl_filter`, `_mapping`, `_pose`, `_touch`, `_rest`, `_report`, `_demo`, `_inhand`, `_fuse` | the unit tests of filter.rs (12), mapping.rs (28), pose.rs (8), touch.rs (3), rest.rs (9), report.rs (4), demo.rs (4), inhand.rs (5), fuse.rs (15) |
| `pibridge_ctrl_inhand_fixtures` | tests/inhand_fixtures.rs (7): the real IMU log of the session 2026-10-16-1818 |
| `pibridge_ctrl_properties` | tests/properties.rs (6 proptest properties, as fixed-seed loops over the same value ranges) |
| `pibridge_ctrl_replay` | tests/replay.rs (4): the real button-by-button capture, decoded with libpibridge-hmd's `decode_controller` |
| `monado_abi_hands_capi` | src/ffi.rs (6), src/inhand_ffi.rs (3) and tests/c_abi.rs (3), through `libpibridge_pimax.a` |

The fixtures (`ctrl_buttons_hub2.rec.gz`, `p82.rec.gz`, `p83.rec.gz`, `ctrl_marks.json`, `inhand_*.csv.gz`) are read
from the omarchyvr checkout (`PIBRIDGE_OMARCHYVR_DIR/rust/fixtures`, read only); the tests that need them exit 77
(skipped) without it. Every test runs with `HOME` and `XDG_RUNTIME_DIR` in the build tree. Byte-for-byte parity
with the Rust library is the difftest of `docs/monado-abi.md` (`monado_abi_diff_hands_*`).

Not ported: `tests/driver_glue.rs` (`#[ignore]` in Rust: it links the object `pimax_ctrl.c.o` of a built Monado tree
with the hands patch); its job, the real driver glue over this archive, is the Monado switch (M9).

### Differences from the Rust library (intended)

- A panic: the stderr text differs (the Rust one names the source line); both abort with SIGABRT.
- `*_create` returns NULL when memory runs out (Rust aborts).
- The integrated-gyro ring is searched with `std::partition_point`: identical to the Rust binary search on the
  `VecDeque` as long as the ring is in time order, which it is whenever the report host times do not go backwards
  (the driver passes `CLOCK_MONOTONIC` arrival times). With host times that go backwards the sample time estimate
  can step back and the two binary searches may pick different neighbours.

### PiBridge's fuser improvements (default; `PIBRIDGE_CTRL_PARITY=1` = the Rust fuser)

`Config::pibridge()` (what `Config::from_env()` starts from unless parity) adds, measured in
`docs/research/ctrl-tracking-improvements.md`:

- **Inertial position** (`ins_s` 0.6 s): a Kalman filter per world axis on the gyro-rotated accelerometer,
  corrected by every fix. It predicts to the display time while tracked and carries the hand on for 0.6 s out of
  view (damped, capped), then the arm-model hold takes over.
- **Tilt fix:** gravity pulls the held orientation's pitch and roll.
- **No give-up** (`keep_tau_s`): past `lost_s` the hold glides to the rest pose instead of returning None. The gyro
  orientation is re-anchored past 1.5 s.
- **Faster relock glide:** 1 m/s.
- **Swap rejection:** a gravity check of each fix against the accelerometer, and an inertial jump gate (6 cm).

- **The hint file:** the fuser writes `$XDG_RUNTIME_DIR/pibridge-ctrl-hint` for `pibridge-ctrltrack`. It holds the
  last fix as measured, moved by the IMU since, head-relative, in a seqlock ring of 16 per hand (`pibridge/ctrl/hint.h`,
  `Config::hint_path`). `from_env()` turns it on unless parity or `PIBRIDGE_CTRL_HINT=0`. `pibridge_ctrl_hint` tests it.
  The new API: `Fuser::hint_pose` and `Fuser::last_fix_age`.

New verdicts: `tilt` (gravity check); `jump` now also comes from the inertial gate. New stat: `rejected_tilt`.
A/B switches: `PIBRIDGE_CTRL_INS_MS`, `PIBRIDGE_CTRL_INS_TAU_MS`, `PIBRIDGE_CTRL_INS_FREE_MS`, `PIBRIDGE_CTRL_INS_JUMP_MM`,
`PIBRIDGE_CTRL_TILT`, `PIBRIDGE_CTRL_KEEP_S`, `PIBRIDGE_CTRL_GRAV_DEG`.

The C ABI is unchanged. `Config{}` keeps the Rust defaults, so the Rust twins are unchanged. `pibridge_ctrl_fuse_pibridge`
tests the new paths. The hands difftests and `monado_abi_hands_capi` run with `PIBRIDGE_CTRL_PARITY=1`.

## Tracker (M7)

see docs/pibridge-ctrltrack.md
