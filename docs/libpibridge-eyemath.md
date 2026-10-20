# libpibridge-eyemath: gaze math

`libpibridge-eyemath` (CMake target `pibridge_eyemath`, namespace `pibridge::eyemath::`, headers `include/pibridge/eyemath/*.h`) is the math between the
eye chip's gaze notification and the gaze ray an app uses. It does no I/O apart from reading and writing the
calibration and correction files, and it has no dependencies. Ported from omarchyvr `rust/pimax-eye/src/`
(`gaze.rs`, `track.rs`, `calib.rs`, `correction.rs`, `filter.rs`, `drift.rs`, `clock.rs`, `rng.rs`); the Rust code
is the specification, and the behaviour, constants and tests are the same.

## The live pipeline (what the tracker calls, in order)

```cpp
#include "pibridge/eyemath/calib.h"   // pulls in gaze.h, track.h, correction.h, filter.h
#include "pibridge/eyemath/clock.h"
#include "pibridge/eyemath/drift.h"

// once, at tracker start (omarchyvr live.rs `Pipeline::new`)
auto sel = pibridge::eyemath::Calibration::from_env();          // PIBRIDGE_EYE_CALIBRATION, HOME
// sel: std::expected<std::optional<LoadedCalibration>, Error>; an error means "the file is there but broken"
const pibridge::eyemath::Calibration* cal = (sel && *sel) ? &(*sel)->cal : nullptr;
pibridge::eyemath::FrameConv conv = cal ? cal->conv : pibridge::eyemath::FrameConv::from_env();  // the calibration's conversion wins
pibridge::eyemath::FilterParams fp = cal ? cal->filter : pibridge::eyemath::FilterParams{.enabled = false};  // no filter without one
pibridge::eyemath::ClockMap clock;                               // device us -> CLOCK_MONOTONIC ns
clock.latency_ns = configured_latency_ns;
pibridge::eyemath::CombineState combine;                         // blink hold-off state
pibridge::eyemath::OneEuro one_euro;                             // output smoothing state
pibridge::eyemath::Drift drift;                                  // running recentre offsets (shared with the recentre hook)

// per gaze notification (TTP payload bytes from libpibridge-eye's session, stream 0x0505)
auto g = pibridge::eyemath::parse_payload(payload);              // strict, as live.rs; parse_payload_lenient for captures
if (!g) { parse_errors++; return; }                    // GazeError::message() says why
std::int64_t host = clock.map(g->timestamp_us, arrival_ns);
pibridge::eyemath::Sample s = pibridge::eyemath::convert(*g, conv, seq++, host, arrival_ns);   // device -> OpenXR head frame
if (cal) cal->apply_sample_with(s, combine, drift.offset);   // per-eye maps + combination
else if (corr) corr->apply_sample(s);                         // old single correction (Correction::from_env())
pibridge::eyemath::filter_sample(fp, one_euro, s);               // One-Euro on the combined ray
// s.eyes[0/1] (left/right) and s.gaze (combined ray, std::optional) are what goes to shared memory
// on a new USB link: clock.reset(); one_euro.reset();
```

## Headers and key types

| Header | Contents |
|---|---|
| `types.h` | `Vec3`, `Vec3f`, `Vec2`, `Tangent {u, v}` (the Rust `(f64, f64)`), `Error {message}` |
| `gaze.h` | `GazeSample`, `Eye`, `EyeState`, `FrameFields`, `GazeError`, `parse_payload` (strict), `parse_payload_lenient`, `encode_payload`, `GazeStats`, `format_line`, field numbers `EYE_FIELDS`, `WIRE_ORDER`, `field::*`, `NOMINAL_PERIOD_US` |
| `track.h` | `FrameConv` (+ `from_env`, `slot`), `EyeRay`, `Gaze`, `Sample`, `convert`, `sample_at` (interpolation for a host time) |
| `correction.h` | `features`, `tangent_of`, `dir_of`, `MAX_TANGENT`, `Correction` (map, `to_toml`/`parse`/`load`/`select`/`from_env`, `flip_warning`), `home_dir` |
| `calib.h` | `Calibration`, `EyeCal`, `OriginTerms`, `Combine`, `CombineState`, `LoadedCalibration`; the fit: `EyePoint`, `Ridge`, `fit_eye`, `evaluate`, `select_model`, `Selection`, `ModelFit`, `ModelKind`, `stats`, `tangent_angle_deg`, `point_error_deg`, `raw_ray` |
| `filter.h` | `FilterParams`, `OneEuro`, `MAX_GAP_S`, `filter_sample` (Rust `filter::apply_sample`) |
| `drift.h` | `DriftParams`, `Drift` (`observe`, `apply`, `apply_dir`) |
| `clock.h` | `monotonic_ns`, `ClockMap` |
| `rng.h` | `Rng` (xorshift64*, same sequence as Rust; for simulations and tests) |

Rust `Result<T, String>` is `std::expected<T, pibridge::eyemath::Error>`; `Option<T>` is `std::optional<T>`. Names differ from
Rust only where C++ forces it: `Eye::openness_raw` (field) next to `Eye::openness()` (method),
`ModelKind::name/parse/features/min_points` are the free functions `model_name`, `parse_model`, `model_features`,
`model_min_points`, `Selection::best()` returns a pointer (nullptr for none), `filter::apply_sample` is
`filter_sample`. Rust-private helpers the tests need are in `pibridge::eyemath::detail` (`solve`, `fit_weighted`).

## Gaze payload decoding

`parse_payload(std::span<const std::uint8_t>)` takes the TTP payload of a gaze notification (the bytes after the
24-byte TTP header: `00 00` + one `0x230bb8` compound of 35 `(field number, value)` elements). The transport (bulk
frames, TTP header, stream ids) stays in libpibridge-eye. The few TLV tags the decoder needs are read by a private copy
(`src/tlv_lite.h`) so that libpibridge-eyemath has no dependency; libpibridge-eye has the full TLV layer. `encode_payload`
reproduces the Rust bytes exactly (a test pins the 1210 bytes printed by the Rust code), so a fake chip built on it
sends what the Rust fake sends.

## Device frame and environment

`FrameConv` defaults: flip z only (device +z forward to OpenXR -z forward), eye A is the right eye. Environment:
`PIBRIDGE_EYE_FLIP_X` (0; also swaps which eye is left), `PIBRIDGE_EYE_FLIP_Y` (0), `PIBRIDGE_EYE_FLIP_Z` (1),
`PIBRIDGE_EYE_OFFSET_MM` (`x,y,z`, millimetres), `PIBRIDGE_EYE_SWAP` (0). A flag is on for `1`, `true`, `yes`, `on`.
Each is also read under its older name (`PIMAX_EYE_FLIP_X`, ...) when the `PIBRIDGE_EYE_` one is not set. The
Crystal Super needs `eye_flip_x=1` (the pibridge.conf key; the daemon turns it into `FrameConv::flip_x`).

## `eye-calibration.toml` (versions 1 and 2)

The format is byte-compatible with the Rust `Calibration::to_toml` / `Calibration::parse`: same keys, order,
comments and number formatting. Numbers are written with `std::to_chars` (fixed, correctly rounded, ties to even,
like Rust's `{:.N}`; `NaN`, `inf`, `-inf` spelled as Rust) and read with `std::from_chars` behind a check of Rust's
float grammar (optional `+`/`-`, `inf`, `infinity`, `nan` in any case, out-of-range gives +-inf or 0), so the locale
never matters. Parse error texts are Rust's (`line 3: u: invalid float literal`). Verified: 200 random calibrations
written by C++ parse in Rust and re-serialise to identical bytes, and back; a version-2 file printed by Rust is a
test fixture that round-trips byte for byte.

```toml
# pimax eye calibration: per-eye host-side maps (written by pimax-eyecal; the eye chip's own calibration is not touched)
# <notes, one comment line each>
# per eye: u' = u . (1, u, v, u*u, u*v, v*v), v' = v . (...) on the tangent (x / -z, y / -z) of the raw head-frame direction; then drift and (version 2) origin_u/v . (origin - origin_ref) in mm are added
version = 2                       # 2 when any eye has origin terms, else 1
created_unix = 1791956330
flip = "1,0,1"                    # FrameConv flip_x, flip_y, flip_z
swap = 0
offset_mm = [0.500000000, -1.250000000, 0.000000000]
depth_m = 2.0000                  # clamped to 0.2..100
max_disagree_deg = 3.500          # at least 0.5
holdoff = 2                       # 0..30

[left]                            # and [right]; both must hold at least one key
model = "quadratic"               # offset | affine | quadratic
u = [..6 numbers..]               # 3 or 6 finite numbers (3 are padded with zeros)
v = [..6 numbers..]
origin_m = [-0.032750000, 0.000000000, 0.000000000]
accuracy_deg = 1.0000
precision_deg = 0.5000
drift = [0.001200000, -0.000400000]
# version 2, only when the eye's origin terms are active:
origin_ref_m = [0.026000000, 0.003000000, 0.020000000]
origin_u = [0.100000000, 0.000000000, -0.050000000]
origin_v = [0.000000000, 0.120000000, 0.000000000]
origin_gate_mm = 5.000            # 0..100, 0 = no gate
pupil_ref_mm = 3.3000             # only with pupil terms, 0..20
pupil_uv = [-0.020000000, 0.010000000]

[filter]
kind = "one-euro"                 # or "none"
min_cutoff_hz = 1.2000            # 0.01..100
beta = 0.02500                    # 0..10
d_cutoff_hz = 2.0000              # 0.01..100
snap_deg = 3.000                  # 0..90
```

Notes are comments: `parse` drops them (as in Rust). Unknown keys and sections are ignored.

Selection (`Calibration::select(var, home)` / `from_env()`): `PIBRIDGE_EYE_CALIBRATION=none|off|0` disables it, a
path loads that file (which must load), unset or empty loads the default file when it exists:
`~/.config/pibridge/eye-calibration.toml` first, then `~/.config/pimax-linux/eye-calibration.toml` (PiBridge's earlier
name), then `~/.config/omarchyvr/eye-calibration.toml`.
`Calibration::default_path()` is the pibridge path (where a new file goes); `Calibration::save(path)` writes
through a temporary file and a rename (no `.bak` copy: that is the writer's job, as in the Rust `pimax-eyecal`).
`eye-correction.toml` (`Correction`, `PIBRIDGE_EYE_CORRECTION`) follows the same rules and the same pibridge-first
lookup.

## Tests

`ctest -R eyemath` runs 9 binaries, 53 tests. Every Rust unit test of the ported modules has a C++ twin with the same
cases and tolerances (gaze 9, track 6, correction 8, calib 9, filter 3, drift 1, clock 4, rng 1). C++-only tests: the
Rust byte fixtures (gaze payload and printed line, version-2 calibration file), the number format and parse tables
printed by Rust, a non-"C" locale (skipped with a message when none is installed), `save`/`load`, parse error texts
and clamps, the pibridge, pimax-linux, omarchyvr default lookup, the `PIMAX_EYE_*` fallback names, `from_env` for `FrameConv` and `Calibration`
(HOME pointed at a temporary directory; the user's real files are never read). None of these modules' tests need
private lab data.

The Rust fixtures were printed by a throw-away program linked against omarchyvr `pimax-eye` (not in the tree): it
builds the calibration of the fixture comment, prints `to_toml()`, and prints `encode_payload` / `format_line` of the
canonical gaze sample of the Rust tests.

## Not ported

- `calfit.rs` (sample-level fit of the origin and pupil terms; `ModelFit::org` is carried but never filled here),
  `fixation.rs`, `ipd.rs`, `ipd_watch.rs`, `analysis.rs`: part of the calibration tool and analysis, not the live path.
- `live.rs` (the tracker that runs this pipeline): phase 2, `libpibridge-eye/src/live*`.
- The TLV layer beyond the gaze tags (`tlv.rs` is libpibridge-eye's) and its tests.
- `Rng` exists only as the deterministic source of the tests and simulations; it is not a general RNG.
