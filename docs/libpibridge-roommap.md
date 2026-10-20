# libpibridge-roommap

The persistent room map: Basalt (the VIO in Monado) starts a new world at every start and keeps no map; this library
keeps a map of the room between sessions and recognises it again, so the world origin (and with it the floor, the
play area and every window) comes back to the same real place. Milestones R2 (room half) and R3 (roommap half) of
`docs/research/rust-port-plan.md` section 5. Ported from omarchyvr `rust/omarchyvr-room` and `rust/omarchyvr-roommap`
(MIT OR Apache-2.0) at the pinned commit `5ffbcc5`; the Rust code is the specification and the outputs are
byte-identical (below). The offset it writes goes through `libpibridge-anchor` (`docs/libpibridge-anchor.md`).

Namespace `pibridge::roommap`, targets `pibridge_roommap_obj` / `pibridge_roommap`, headers
`libpibridge-roommap/include/pibridge/roommap/`. Needs libpibridge-cam, libpibridge-anchor, libpibridge-ctrl's tracker half and zlib.
Not to be confused with `libpibridge-room/` (the ZoneTweak chaperone port, untouched). Built with the parity flags plus
`-fno-math-errno` (same libm values; lets the compiler hoist loop-invariant sin/cos as LLVM does for the Rust code).

| Header | What | From |
|---|---|---|
| `orb.h` | ORB: pyramid, FAST-9 per grid cell, Harris, per-level budgets, intensity centroid, steered BRIEF (fixed splitmix pattern), Hamming matching (`match_bf`, `match_gated`) | orb.rs |
| `pngread.h` | grey PNG reader for EuRoC frames (zlib) | pngread.rs |
| `pnp.h` | gravity-aware 4-DoF absolute pose: P2 minimal solver, MSAC RANSAC with local optimisation, Huber LM | pnp.rs |
| `stereo.h` | the rig from the factory calibration, virtual pinhole views (same orientation for every camera), low-light stretch, features with bearings, sub-pixel NCC stereo, triangulation | stereo.rs |
| `map.h` | `RoomMap` (`map.bin` "PXRM" v1, `room.json`), keyframes, anchors, single-frame relocalisation | map.rs |
| `session.h` | `RoomSession`: relocalise at start, refine, add keyframes | session.rs |
| `synth.h` | the synthetic textured room through the KB4 cameras (tests, demo) | synth.rs |
| `rec.h` | recordings (EuRoC + Basalt `tracking.csv`, z up -> y up) | rec.rs |
| `pipeline.h` | map building and per-frame observation over a recording (threads; results in time order) | pipeline.rs |
| `align.h` | pooled relocalisation: per-frame observation, joint 4-DoF fit over a window, first fix, window fits | align.rs |
| `loopclose.h` | loop closure inside the mapping session (4-DoF pose graph, Gauss-Newton, outlier loops dropped) | loopclose.rs |
| `refine.h` | multi-view re-triangulation of the map points (`--refine`) | refine.rs |
| `floor.h` | floor from points (layers, gravity RANSAC), track, touch, `floor.toml`, `Follower`, STAGE shift, preview | floor.rs |
| `setup.h` | room setup: `room-setup.toml`, boundary clean-up, play area, free area, walls, guided steps (`Guide`) | setup.rs |
| `store.h` | map directory, `RelocResult` (`last-reloc.toml`), corrections, offset writer | store.rs |
| `light.h` | brightness/corners/stereo-points quality and advice | light.rs |
| `exportmap.h` | JSON export and the offline HTML viewer (`data/viewer.html`, embedded) | export.rs |
| `live.h` | broker frames + Monado's head ring | live.rs |

Map directory: `PIBRIDGE_ROOMMAP_DIR`, `PIMAX_ROOMMAP_DIR`, else `~/.local/share/pibridge/roommap/<name>`, falling back to
`~/.local/share/omarchyvr/roommap` when only that exists (the Rust tool knows only its own path and
`PIMAX_ROOMMAP_DIR`). Files and their bytes are the Rust ones (`map.bin`, `room.json`, `roommap.toml`,
`correction.csv`, `floor.toml`, `preview.txt`, `room-setup.toml`, `last-reloc.toml`, `reloc-*.toml`, `map.json`,
`map.html`; `$XDG_RUNTIME_DIR/omarchyvr-room.status`, `pimax-stage-bounds`), so a map made by either tool works in the
other.

## Crystal Super

Nothing headset-specific is needed: on both headsets factory cameras 0 and 1 are the lower tracking pair (Crystal:
75 mm baseline, axes 16 deg apart, 38.5 deg down; Dream Air: 111 mm, 54 deg apart), the recordings carry cams 0/1
(`pimax-slam-info.json` `factory_cams`), the broker's tiles 0/1 are those cameras, and the virtual pinhole takes the
pair's mean pitch from the calibration. The Crystal's upper pair (2/3, "Pi-controller", 168 mm, 20 deg up) is not in
the recordings; using it (more texture above the desk) is a follow-up (`live::capture` takes the tile pair). Results
on the Crystal takes: `docs/research/rust-port-plan.md` section 5, R5.

## Numerics and differences

Bit parity rules as in the other ports (`-ffp-contract=off`, saturating `as` casts, stable sorts where Rust sorts
stably, `max_by` = last of equal maxima, `Iterator::sum` from -0.0, `powi` as LLVM expands it, f32 kept f32, the f32
libm calls `sinf`/`cosf`/`atan2f`). Intended differences: the PNG inflate error text (zlib vs miniz_oxide); the map
directory default (above). Pure speed changes with identical output: inline saturating casts and rounding, a
software popcount (no libgcc call), the rotation of a pose computed once per pose in the joint fit (the Rust code
computes sin/cos per ray); `-fno-math-errno`.

## Tests (`ctest -R pibridge_roommap`)

| Test | What |
|---|---|
| `pibridge_roommap_orb` | 28 twins of orb.rs |
| `pibridge_roommap_pnp` | 23 twins of pnp.rs (the `#[ignore]` stress run left out), reference calibration |
| `pibridge_roommap_room` | 6 twins of tests.rs (omarchyvr-room) + pngread, reference calibration |
| `pibridge_roommap_roommap` | 28 twins: omarchyvr-roommap tests.rs (9), floor.rs (6), setup.rs (9), light.rs (4) |
| `pibridge_roommap_probe_room` | probe of the synthetic room through the reference calibration (8721 lines: pattern, LUTs, ray tables, renders, undistortion, raw ORB, features, stereo; float bits) = the oracle's (SHA-256 in `tests/golden/room_probe.sha256`) |
| `pibridge_roommap_probe_trace` | 2000 ray traces of the synthetic room = the oracle's (`tests/golden/trace_probe.txt.gz`) |
| `pibridge_room_diff_*`, `pibridge_roommap_diff_*` (tools/roommap) | the tools against the Rust tools on the Crystal takes, `docs/pibridge-roommap.md` |
