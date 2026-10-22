# libpibridge-lens

The lens model of the Monado Pimax driver: the `lens.conf` parser, the per-channel radial polynomial (bit-identical
to the old C model), the exact distortion tables (`lut = FILE`, the Crystal Super's `crystal-lut.bin`), the
visibility mask meshes for `XR_KHR_visibility_mask`, and the debug pictures of `PIMAX_LENS_DEBUG_GRID`. Ported from
omarchyvr `rust/pimax-lens` (MIT OR Apache-2.0) at commit `381eea2` (`monado-abi/spec.txt`); the Rust code is the
specification and its tests have C++ twins here. Milestone M5 of `docs/research/rust-port-plan.md`.

CMake targets `pibridge_lens_obj` (objects, in `libpibridge_pimax.a`) and `pibridge_lens` (static, for tests and tools),
namespace `pibridge::lens::`, headers in `libpibridge-lens/include/pibridge/lens/`. The C ABI `pimax_lens_rs.h` is in
`monado-abi/src/lens_capi.cpp` (`docs/monado-abi.md`). Dependencies: zlib (PNG), libpibridge-cam's `rustfmt.h`
(Rust number formatting), `io.h` and `json.h` (only the dump reader). Built with libpibridge-cam's parity flags
(`-ffp-contract=off -fno-fast-math`, hidden visibility): every f32/f64 expression keeps the Rust order, f32
functions are the glibc ones Rust calls (`tanf`, `atanf`, `powf`, `cosf`, `sinf`, `hypotf`), `powi` is compiler-rt's
square-and-multiply, `to_degrees`/`to_radians` use Rust's f32 constants, `as` casts saturate (`fmt::sat_cast`), numbers
in `lens.conf` are parsed like Rust's `str::parse::<f32>` (correctly rounded, `strtof_l` in the C locale after the
same strtof-like scan).

## Headers

| Header | What | From |
|---|---|---|
| `config.h` | `LensConfig`, `EyeConfig` (fov, centre, base/scale/k1..k5, ca, shift, fit_range, blackout, mask, feather, lut, lut_margin, lut_interp), `parse`/`parse_into` with the warnings of the old C parser, `config_path`, `cant_override` (`PIMAX_CANT_DEG`), `reference_allowed` (`PIMAX_NO_REFERENCE`), `forced_off` (`PIMAX_LENS_OFF`) | config.rs |
| `model.h` | `Lens`: `compute` (the distortion callback), `tangent`, `tangent_to_uv`, `mask_distance`, `hide_amount`, `radius_for_angle`, `first_fold`; `radial_tangent` | model.rs |
| `lut.h` | `Lut`: the `PIMAXLUT` v1 file (`from_bytes`, `to_bytes`, CRC32), the reorientation into the user display, bilinear / Catmull-Rom sampling, `from_dump`, `line_error`, `ORIENTS` | lut.rs, pvrfit.rs `ORIENTS` |
| `mask.h` | `visibility_mesh` (hidden / visible triangle meshes, line loop), `outline_tangent`, `mesh_area` | mask.rs |
| `grid.h`, `font.h`, `png.h` | the debug grid picture and its text summary, the 5x7 font, the PNG writer (zlib) and reader | grid.rs, font.rs, png.rs |
| `lens.h` | `load` (the driver's loading: user file, the shipped Dream Air profile, `PIMAX_LENS_CONF`, LUT attachment, log lines), `load_file`, `load_reference`, `eye_log_lines`, `write_debug_grid`, `reference_lens_conf()` (embedded `devices/dream-air/reference/lens.conf`) | lib.rs |
| `pvrdump.h` | `parse_dump` of a PVR distortion dump (`pimax-distortion-*.json`): what `Lut::from_dump` and the LUT tests need | pvrfit.rs (reader only) |
| `fmt.h` | Rust `{}` of an f32, saturating casts, f32 `powi`/`clamp`/`to_degrees` | Rust core |

Environment (unchanged names): `PIMAX_LENS_CONF`, `XDG_CONFIG_HOME`/`HOME` (`omarchyvr/lens.conf`),
`PIMAX_NO_REFERENCE`, `PIMAX_LENS_OFF`, `PIMAX_CANT_DEG`, `PIMAX_LENS_DEBUG_GRID=<dir>`, `PIMAX_LENS_DEBUG_GRID_DIV`
(1..64, default 4). Log texts are the Rust ones ("pimax lens: ...").

## Tests (`ctest -R lens`)

| Test | What |
|---|---|
| `pibridge_lens_config`, `_model`, `_lut`, `_mask`, `_png`, `_font`, `_grid`, `_lens` | twins of the 58 Rust unit tests of config/model/lut/mask/png/font/grid/lib (same cases, same expected values), plus Path helper checks; the LUT tests read omarchyvr's public fixture `rust/pimax-lens/tests/data/pvr-dump-sub.json` (skipped without the checkout) |
| `monado_abi_lens_capi` | twins of the 3 `capi.rs` tests through `libpibridge_pimax.a` |

The difftests against the Rust `libpimax_lens.a` (synthetic scenarios, the Crystal conf + LUT, the reference
profile; all byte-identical except where noted below) were removed with the oracle once the port was finished
(2026-10-22).

## Differences from the Rust code (intended)

- The PNG deflate stream is zlib's, not miniz_oxide's: the files differ in their IDAT bytes; the scanlines (filter
  bytes and pixels) are identical (what the former difftest compared).
- `png::decode` returns an error on a chunk whose CRC lies past the end (the Rust code panics); `png::encode` throws
  `std::invalid_argument("pixel buffer size")` where the Rust code panics.
- No exception leaves the C ABI (create returns NULL, the mask call -1); the Rust functions would abort on a panic.
- Test twins: `round_trip_through_bytes_is_exact` compares the LUT serial with the dump's own instead of naming it (no
  serials in the tree); `interpolation_error_on_the_real_dump` finds the private full dump through
  `$PIBRIDGE_LENS_REAL_DUMP_DIR` (skipped without, like the Rust test without its lab folder).

## Not ported yet

The setup tools of the crate (plan M10): `pimax-lens show/make-lut/fit-pvr` (`bin/pimax-lens.rs`, `pvrfit.rs` fit and
report, `fit.rs`, `tests/cli.rs`). The runtime the driver links needs none of them.
