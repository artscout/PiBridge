# libpibridge-anchor

The world anchor files shared with Monado's room-anchor hook, without hardware and without the room map: milestone
R1 of `docs/research/rust-port-plan.md` (section 5). Ported from omarchyvr `rust/omarchyvr-anchor` (MIT OR
Apache-2.0) at the pinned commit `5ffbcc5`: `world.rs` and `config.rs`, and `detect.rs`'s `pose_error`. The Monado
side is `third_party/monado/src/xrt/drivers/pimax/pimax_anchor.h` (`PIMAX_ANCHOR=1`, marker
`PIMAX_ANCHOR_GLIDE`); every file below is byte-compatible with it and with the Rust tools, so PiBridge and omarchyvr
tools can run side by side on one session.

Namespace `pibridge::anchor`, targets `pibridge_anchor_obj` / `pibridge_anchor`, headers `libpibridge-anchor/include/pibridge/anchor/`.
Needs libpibridge-cam (geom, io, rustfmt, slam) and libpibridge-ctrl's tracker half (`quat_to_mat`). Built with the parity
flags (`-ffp-contract=off -fno-fast-math -fwrapv`, hidden visibility).

## Files

| File | Who writes | Format |
|---|---|---|
| `$XDG_RUNTIME_DIR/pimax-anchor.head` (`PIMAX_ANCHOR_HEAD_FILE`) | Monado (raw SLAM head poses, before the offset, at most one per 3 ms) | 16-byte header: u32 magic `0x434E4150`, u32 version 1, u32 seq (odd while writing), u32 writer pid; 128 slots of 40 bytes: u64 ts_ns (CLOCK_MONOTONIC), f32 pos[3], f32 quat[4] (x y z w), u32 flags (bit 0: position valid); round robin, empty slots ts 0; little endian |
| `$XDG_RUNTIME_DIR/pimax-anchor.offset` (`PIMAX_ANCHOR_FILE`) | the tools; Monado deletes it at its start and reads it every 250 ms (by mtime) | `pimax-anchor-offset 1` / `pid N` / `pose px py pz qx qy qz qw` (`{:.6}` position, `{:.7}` quaternion); Monado applies `app_from_slam = offset * slam` only when N is its own pid; the first offset of a session is placed at once, later changes glide |
| `omarchyvr-anchor.offset.owner` next to the offset | the tools (the writer lease) | `roommap <monado pid>` or `anchor <monado pid>` (what the apps read), `lease 2`, `writer <pid>`, `role setup|auto|follow|reloc|apply|tag` |
| `omarchyvr-anchor.offset.lock` next to the offset | the tools | empty; `flock` around every lease decision and offset write |
| `anchor.toml` | the AprilTag calibration (not ported) | `[tag] id size_m`, `[reference] pos quat saved`, `[[window]] name pos quat` |

Every write is atomic (`<file>.tmp` or `.offset.tmp` / `.owner.tmp`, then rename). The environment names are the Rust
and Monado ones (`PIMAX_ANCHOR_FILE`, `PIMAX_ANCHOR_HEAD_FILE`): Monado reads only these, so a `PIBRIDGE_` name could
point a tool at another file than the one Monado reads. `anchor.toml` follows CONVENTIONS.md: `PIBRIDGE_ANCHOR_CONFIG`,
`PIMAX_ANCHOR_CONFIG`, else the first existing of `~/.config/{pibridge,pimax-linux,omarchyvr}/anchor.toml`, else
`~/.config/pibridge/anchor.toml` (the Rust tool knows only `PIMAX_ANCHOR_CONFIG` and the omarchyvr path).

## API (`world.h`, `config.h`)

- Paths: `runtime_dir()`, `ring_path()`, `offset_path()`, `lease_path()`, `lease_path_for(offset)`.
- Head ring: `HeadRing::read(path)` (seqlock read, 8 tries), `HeadRing::parse(bytes)`, `newest_ns()`,
  `at(ts_ns, max_gap_ns)` (interpolated between the valid samples around `ts_ns` when they are at most `max_gap_ns`
  apart, else the nearest valid one within half of it); `ring_bytes(pid, samples)` writes the layout (tests).
- Feed: `feed_now()` / `check_feed(ring, monado_pids, now_ns, alive)`: `Missing`, `NoMonado`, `Foreign` (another
  program replaced the ring: SteamVR's Monado driver), `Stale` (newest sample older than 5 s, or none), `Live`;
  `Feed::plain()` is one sentence for the user, `live_pid()`; `tracker_pids()` = running `monado-service`, else
  `vrserver`; `pids_named`, `pid_alive`.
- Offset: `offset_text`, `parse_offset`, `read_offset(path)` -> (pid, pose); `write_offset(path, pid, pose)` (refused
  with PermissionDenied unless this process holds the lease of that session); `compute_offset(reference, now,
  yaw_only)`; `recenter_offset(slam_from_head)` (not in the Rust code: for PiBridge's Recenter, a yaw-only offset that
  puts the head at x = z = 0 looking along -Z, height unchanged).
- Lease: `take_lease(offset, monado_pid, role)` (returns a log line when it ended another writer), `holds_lease`,
  `read_lease`, `Lease::{parse,text,holder_title}`, `decide_lease` (pure), `Role` with `role_name/kind/title/preempts/
  first_only`. Rules: a user action (`Setup`, `Reloc`, `Apply`, `Tag`) takes the lease from a running background writer
  (`Auto`, `Follow`) and ends it by pid (SIGTERM, only when its `/proc/<pid>/comm` is one of our offset writers); a
  background writer never takes it from a live holder; `Auto` never writes after anyone placed the room in this
  session; the room map and the AprilTag do not mix in one session.
- Maths: `pq`, `from_pq`, `interp` (slerp), `pose_error`, `robust_mean`.
- `Anchor::{parse,to_toml,load,save}`, `anchor_config_path()`.
- `rustx.h`: the Rust semantics the room ports share (`lines`, `split_once`, `trim_matches`, `with_extension`,
  `powi` as LLVM expands it, `total_cmp`, parse error texts, saturating casts, the tool panic).

A Recenter in PiBridge: read the ring (`HeadRing::read(ring_path())`, check `feed_now().live_pid()`), take the newest
valid sample, `take_lease(offset_path(), pid, Role::Apply)`, `write_offset(offset_path(), pid,
recenter_offset(head))`. Monado glides to it (it is not the session's first offset) unless `PIMAX_ANCHOR_GLIDE=0`.

## Differences from the Rust code (intended)

- The writers a user action may end are matched by `comm` `omarchyvr-roomm`, `omarchyvr-ancho` (Rust) and also
  `pibridge-roommap`, `pibridge-anchor`.
- `anchor.toml`'s default path (above).
- A NaN position in `robust_mean` with more than one pose ends the process with the Rust panic text and exit 101.

## Tests (`ctest -R pibridge_anchor`)

| Test | What |
|---|---|
| `pibridge_anchor_world` | twins of the 8 world.rs tests (`lease_tests`, `feed_tests`) + the file names and bytes, the ring layout, recenter |
| `pibridge_anchor_config` | anchor.toml round trip, error texts, the path rule |
| `pibridge_anchor_probe` | `tests/anchor_probe.cpp` against `tests/golden/anchor_probe.txt.gz` (4909 lines, byte for byte): 300 offset texts and their parse, 20 crafted offset files, 14 lease files, 40 rings (corrupt magic/version/size, NaN/inf quaternions, ts 0, duplicates) with 480 `at` queries, 10 ring writes (FNV of the bytes), 60 `compute_offset` x2 / `interp` / `pose_error`, 30 `robust_mean`, 480 `decide_lease` cases, 192 `check_feed` cases with `plain()`, 13 anchor.toml texts; every number as its f64 bits. The golden file is the output of a scratch program that linked omarchyvr-anchor at `5ffbcc5` and ran the same cases (not in the tree: PiBridge has no Rust) |
| `pibridge_anchor_monado` | Monado's own `pimax_anchor.h` (from `third_party/monado`, with stub headers for its math/clock/log helpers): it writes the head ring that `HeadRing::read` parses (128 newest samples, floats exact, pid), deletes the old offset at its start, and parses the offset `write_offset` writes (first offset placed at once, `offset * slam` applied, another pid's file ignored) |

Not ported: the AprilTag detector and pose (`detect.rs`, `tag.rs` printable SVG/PDF, `rig.rs`) and the
`omarchyvr-anchor tag/detect/calibrate/apply` commands: they need the AprilTag C library (apriltag-sys builds it from
its bundled sources; PiBridge downloads nothing and the library is not installed here). Follow-up: port the tag36h11
decoder or link a system libapriltag when present.
