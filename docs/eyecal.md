# pibridge-eyecal

The eye calibration of PiBridge (`tools/eyecal/`, binary `pibridge-eyecal`). Port of omarchyvr's `pimax-eyecal`
(`rust/pimax-eyecal`, MIT OR Apache-2.0) with one addition that is now the default: **the Tobii eye chip calibrates
itself, as Pimax Play does under Windows**. The host-side fit of the Rust tool stays as a fallback (`--host-only`), and
`--recenter` updates the drift offsets of the host map.

It is an OpenXR app (XR_KHR_vulkan_enable2 + Vulkan): it shows the targets in the headset, opens the eye chip itself
(the same start list and profile as `pibridge-eyed`) and reads the raw gaze while the chip calibrates.

```
pibridge-eyecal                 the check grid, the chip's own calibration, a short check, back to the grid (the
                             app's Calibrate)
pibridge-eyecal --recenter      the check grid and the 5-target drift update of eye-calibration.toml (the app's Recenter)
pibridge-eyecal --host-only     the host fit without chip calibration (the Rust tool's calibration)
pibridge-eyecal --simulate      the whole procedure on a simulated wearer and chip (no headset); add --recenter or
                             --host-only to simulate those
```

## What the wearer sees (default: the chip's own calibration)

1. **The check grid** (when the tool starts; the Dashboard's Calibrate runs it): the head-locked grid of omarchyvr's
   `xr_grid` on the dark background: a square grid every 0.2 m at 2 m (every 5th line brighter), a red centre cross,
   circles at 10 / 20 / 30 / 40 / 50 degrees from the view axis (white, yellow, green, cyan, magenta), an orange
   border at the edge of each eye's image. On it **one violet dot with a thin dark outline**: the combined gaze of both
   eyes, as `pibridge-eyed` would publish it now (the chip with the current profile, the host map `pibridge-eyed` loads, its
   filter; see "The gaze dot"). It should sit where you look; the circles tell how far off it is. The bottom line:
   "A: CALIBRATE    B: EXIT" ("WAITING FOR THE EYE TRACKER" until the tool's tracker streams, "NO GAZE" while it
   sends none). **B exits.**
2. **A** starts the chip's calibration (it waits until the tracker streams). Pimax Play's look: a green dot travels
   (1 s) to each of the nine targets. Pass 1 on the dark background: centre and the four diagonals (15 degrees). Pass
   2 on white: left, right, up, down (25 / 23 degrees). At each target the tool waits until the gaze is steady (at
   least 0.45 s after the dot stopped, at most 2 s), then sends the target to the chip, which collects the gaze for
   about 165 ms and answers; the dot dims and moves on. Between the passes the dot stays where it is ("PLEASE WAIT"
   when the chip takes longer than 0.6 s). A head turn of more than 2.5 degrees before a target was sent restarts that
   target ("HEAD MOVED"). **B stops**: the open pass is finished on the chip (0x3fc), nothing is kept, back to the
   grid ("STOPPED: NOT SAVED"; A there waits, "PLEASE WAIT", until the chip has answered the stopped run's last
   requests).
3. **The new profile**: after the last download the chip's new profile is checked and written (see Files), not kept
   yet.
4. **Check**: five targets (centre, +-8 degrees across, +-6 up/down) with the new chip calibration, the grid look. Per
   eye: the raw gaze against the targets, and the constant offset between the chip's frame and the view (the
   2026-10-18 case: 9-14 degrees, mostly vertical, removed by a recentre).
5. **End screen**: "CHIP CALIBRATION DONE", the accuracy per eye after the offset, the offset removed,
   "A: AGAIN    B: SAVE". **A** calibrates again at once. **B saves** (keeps the new profile, writes the host map) and
   **goes back to the check grid** ("SAVED: THE DOT USES THE NEW CALIBRATION", "A: CALIBRATE AGAIN    B: EXIT"): the
   dot now goes through the new host map, and the chip has used its new calibration since its compute (0x42e), so the
   grid shows what apps get from the next `pibridge-eyed` start. **B there exits**; A calibrates again, and that run's
   "previous profile" is the one just saved. A failed run ends on "CALIBRATION FAILED: ...", "A: AGAIN    B: BACK" (B:
   back to the grid, nothing saved).
6. **Leaving without a save** (B on the grid after a run that was not saved, q / Esc on the terminal, the runtime
   ending the session, SIGTERM, an error) puts the profile of the last save back, or the one from before the tool when
   nothing was saved. The host map is only written by B on an end screen.

`--recenter` has the same frame: the check grid ("A: RECENTER    B: EXIT", the dot through the existing host map),
A the five targets (B during them: back to the grid), the end screen "A: AGAIN    B: SAVE"; B writes the recentred
map and goes back to the grid with it ("A: RECENTER AGAIN    B: EXIT"), B there exits. With `--pattern grid` it
keeps the Rust tool's intro and results screen; `--host-only` too (A start, B exit / at the end A again, B save and
exit).

Controllers: Valve Index (SteamVR) A = A (start / next), B = B (stop / back / save / exit), trigger = start; Oculus
Touch bindings as in the Rust tool (A/X next, B/Y back, menu save). Terminal keys (raw stdin): space/a = A, b/r = B,
y save, p pursuit, c cursor, s runtime check, q/Esc quit. The text hangs as a head-locked quad layer 2 m ahead, below
the centre.

`--host-only` and `--recenter --pattern grid` show the procedure of the Rust tool (Pimax path by default, `--pattern
grid` for the 3 x 3 / 13-point grid with rings, validation, live cursor, pursuit, the end-to-end check through the
runtime's `XR_EXT_eye_gaze_interaction`).

### The check grid and the gaze dot

The grid is drawn in the same full-screen fragment pass as the marks (`shaders/marks.frag`, CPU twin `marks.cpp`
`shade`), ray-cast per pixel through each eye's own FOV in that eye's view space, as `xr_grid` does: ported from
omarchyvr `analysis/tools/xr_grid.c` `draw_grid` (MIT OR Apache-2.0). Its uniform block gained three vec4 after the
background (std140): `grid` (on, the grid step 0.1 in tangent units, the 4-pixel border), `gaze` (the dot's
direction in that eye's view space, its radius 0.7 degrees; 0 = no dot) and `gaze_col` (violet, outline 0.15
degrees). The frame cost stays a fraction of a millisecond. Differences from `xr_grid`: one background for both eyes
(the tool's dark grey, which keeps the pupils as during the calibration) instead of a blue / red one per eye, and one
filled dot instead of its two markers (cyan: eyed's per-eye ray, magenta ring: the runtime's gaze).

The dot is the combined gaze `pibridge-eyed` would publish: the tool's tracker is raw (the procedure needs the per-eye
rays), so the procedure runs every raw sample through the same steps as `live::Pipeline`: the host map `pibridge-eyed`
loads (`Calibration::from_env`: `PIBRIDGE_EYE_CALIBRATION`, else `~/.config/pibridge/eye-calibration.toml` and its
fallbacks; `apply_sample`), then its filter (`PIBRIDGE_EYE_FILTER`, else the map's `[filter]`, off without a map;
`filter_sample`), and shows the combined ray at 2 m, projected into each eye like the marks. Without a host map it is
the raw combined gaze. A map measured with another frame conversion than the tool's (flip x / y) is not applied (the
terminal says so). After a save the new map replaces it at once (`Settings::view`); `--recenter` starts with the map it
recentres.

## The chip's own calibration

Captured from Pimax Play (take "calibrate" of 2026-10-18, three runs, byte-identical target requests; the analysis is
omarchyvr `analysis/crystal-super-eye-20261018.md`, "The calibration is the chip's own"). One calibration is two
passes over the bulk pipe; every answer has status 1:

| Pass | Requests (in order) | Answer |
|---|---|---|
| 1 | `0x3f2` start, `0x407` x 5 (centre, (-536, 536), (536, -536), (536, 536), (-536, -536)), `0x42e` compute, `0x44c` download, `0x3fc` finish | 0x407 after ~165 ms, 0x42e after ~500 ms, 0x44c ~80 ms with the profile |
| 2 | `0x3f2`, `0x407` x 4 ((-932, 0), (932, 0), (0, 840), (0, -932)), `0x42e`, `0x44c`, `0x3fc`, `0x44c` again | the last download is the profile Pimax stores |

`0x3f2`, `0x42e`, `0x44c` and `0x3fc` have no payload. `0x407` carries the target in millimetres at z = 2000 (S22.42
fixed point) and the eye mask (3 = both), 50 bytes. The download carries the profile in the 0x456 upload layout
(`00 00 15 | BE len+4 | BE len | body`); the body is what `calibration.setpm` holds after its 16-byte header.

**The chip uses the new calibration from its 0x42e on** (Tobii's log: "computed calibration for both eyes",
"Committed new calibration"); Pimax Play did not upload anything after the calibration in the capture. The check after
the calibration therefore already sees the new calibration. The saved profile is what `pibridge-eyed` uploads at every
chip start (0x456); the app restarts `pibridge-eyed` after the tool, so the next start uploads the new one.

### Where the targets are shown

The table's x, y are taken to be in the chip's own frame, the frame of its gaze origins and directions. The tool turns
them into the head frame with the same `FrameConv` as the gaze (the Crystal Super's `eye_flip_x=1`), so the dot is shown
where the chip's gaze would point at that target: whatever the chip's axes are, its calibration then agrees with the
gaze the tool and `pibridge-eyed` read. With `eye_flip_x=1` the first diagonal (-536, 536) is shown up and right. The
user's memory of the Pimax path (pass 1 first goes bottom-left, pass 2 first goes left) does not settle the sign, so
the check after the calibration looks for it: when an eye's gaze moves *against* the check targets on an axis it says
"the chip's targets may have been shown mirrored (try --chip-mirror x)" (and "THE DOTS LOOK MIRRORED" in the
headset), and no offset is saved for that eye. `--chip-mirror x|y|xy` mirrors the shown targets (the requests stay the
captured bytes).

### The calibration guard

`docs/CONVENTIONS.md`: calibration opcodes are refused unless an explicit feature allows them. The design:

- `frames::Guard` (the start list's gate) is **unchanged**: it still refuses every opcode of `frames::DENIED_OPS`
  (0x3f2, 0x407, 0x42e, 0x3fc, 0x44c), even for a frame added with `allow_frame`, and refuses a start list that
  contains one.
- `calibration::Guard` (`libpibridge-eye/include/pibridge/eye/calibration.h`) is a separate gate, **closed by default**.
  Only `open()` (the calibration session the tool opens once its tracker streams) lets anything out, and then exactly:
  one whole request in one bulk OUT frame (wrapper length = body, header payload length = the bytes after it), a
  request header with status 0 and aux 0, an opcode of the five, an empty payload for 0x3f2 / 0x42e / 0x44c / 0x3fc,
  and for 0x407 one of the nine captured payloads. Anything else is refused before the transport sees it.
- The requests are built only from the captured target table `libpibridge-eye/data/tobii-calibration-targets.txt`,
  compiled in like the base frame list (`calibration::targets()`). Its parser checks every payload's layout and that
  its fixed-point x, y, z are the numbers of its line. The header is the usual request header with a fresh sequence
  number: one more than the highest request the link sent (at least 27, the capture's own numbering).
- Answers are checked: the right opcode and sequence number, status 1, and the last download must be a profile
  (`frames::profile_body` + `frames::check_profile`).

### Inside the live session

The chip has one IN pipe, and the gaze stream must go on while the chip calibrates (the tool needs the gaze to know
when the wearer looks at a target). So the calibration runs inside the tool's own `live::Tracker` session:
`calibration::Channel` is a Transport between the tracker and the chip. It forwards everything, sends the queued
calibration requests from the tracker's own thread (inside its bulk IN reads, so the pipe keeps one user) through the
calibration guard, and picks their answers out of the IN stream for `calibration::Link`, the tool's side (open the
session, submit a step, poll its answer; an answer that does not come in 3 s, 10 s for compute and download, fails).
When the link ends (chip gone, reconnect) the Channel detaches: open requests fail, the session closes, and the tool
shows "CALIBRATION FAILED". `calibration::Run` is the captured sequence as a state machine (the procedure asks it what
to send next and hands it every answer), and `calibration::exchange()` runs one request on a transport nothing else
reads (the tests).

## Files

| File | What |
|---|---|
| eye profile | the chip's new profile. Written where `pibridge-eyed` reads it: `eye_profile` of pibridge.conf when set (also `PIBRIDGE_EYE_PROFILE`, `--profile`), else `~/.config/pibridge/eye-profile-crystal-super.bin`. Checked first (`check_profile`; a bad profile leaves the old file alone), then written through a temporary file and a rename, mode 0600. The previous file is copied to `<name>.bak-<utc>` (0600) at the first save of a run; a calibration after a save (A on the grid) backs the saved one up as `<name>.bak-<utc>-1` (-2, ...). Not kept (the tool ends without B on the end screen of that run): the profile before that run is written back (the last saved one, or the one from before the tool; the `.bak` stays), or the new file is removed when there was none. |
| `~/.config/pibridge/eye-calibration.toml` | the host map (`--calibration` for another file), written at B / menu on the end screen with a `.bak-<utc>` (`-1`, ... for later saves of the same run) of the previous one. After the chip's calibration: an identity map per eye, the eye positions of the view, and the check's constant offset as `drift` for each eye where one was found. `pibridge-eyed` reads it at its next start. |
| `~/.local/share/pibridge/eye-calibration/eyecal-<utc>.json` | results: mode, conversion, eye positions, fits, targets, the chip's targets with the gaze at each request, the check per eye, the calibration text |
| `~/.local/share/pibridge/eye-calibration/eyecal-<utc>-raw.csv` | every raw sample with its phase tag (`--no-raw-log` to skip) |

To go back to the profile before a calibration by hand: copy `eye-profile-crystal-super.bin.bak-<utc>` over
`eye-profile-crystal-super.bin` and restart eye tracking (the app's eye tracking switch off and on).

## Options

| Option | Default | |
|---|---|---|
| `--recenter` / `--host-only` | the chip's calibration | see above |
| `--pattern pimax\|grid`, `--points 9\|13`, `--pursuit` | pimax, 9 | host fit / recentre only |
| `--flip-x 0\|1` | `PIBRIDGE_EYE_FLIP_X`, else `eye_flip_x` of pibridge.conf (1) | device-to-head x |
| `--flip-y 0\|1` | `PIBRIDGE_EYE_FLIP_Y`, else 0 | |
| `--head-limit DEG` | 2.5 | head rotation that restarts a target |
| `--res-div N` | 1 | render at 1/N of the recommended size |
| `--calibration PATH` | `~/.config/pibridge/eye-calibration.toml` (`--recenter` reads the first of pibridge, pimax-linux, omarchyvr) | |
| `--frames FILE` | `PIBRIDGE_EYE_FRAMES`, else `eye_start_list`, else `eye-start-crystal-super.txt` in ~/.config/pibridge, pimax-linux, omarchyvr | the chip's start list (as pibridge-eyed) |
| `--profile FILE` / `--no-profile` | `PIBRIDGE_EYE_PROFILE`, else `eye_profile`, else `eye-profile-crystal-super.bin` in the same folders | uploaded at the start; the new profile replaces it |
| `--chip-mirror none\|x\|y\|xy` | none | mirror the shown chip targets |
| `--settings FILE` | `~/.config/pibridge/pibridge.conf` | |
| `--no-raw-log`, `--simulate`, `--seed N` | | |

Every `PIBRIDGE_EYE_*` variable is also read under its older `PIMAX_EYE_*` name. The tracker's own environment
(`PIBRIDGE_EYE_USB`, `PIBRIDGE_EYE_POWER_ON`, `PIBRIDGE_EYE_USB_RESET`, ...) applies; the tool's tracker is always raw (no host
map, no filter: the procedure needs the per-eye rays).

**One owner**: the tool refuses to start (exit 3, in plain words) while `pibridge-eyed`, `pimax-eyed`, `omarchyvr-eyed` or
`omarchyvr eyed` runs. The app's Calibrate and Recenter stop `pibridge-eyed` first and start it again afterwards (when eye
tracking is on). Exit codes: 0 done, 1 runtime / OpenXR error, 2 bad options or settings, 3 the chip is busy.

## Build and tests

The procedure, fits, report, simulation, HUD texture, chip glue and options are the library `pibridge_eyecal_core`. The
app needs the OpenXR loader (`find_package(OpenXR)`), Vulkan and `glslangValidator` (the marks shaders are compiled to
C arrays at build time, as the quad views layer does); without them `pibridge-eyecal` is built for `--simulate` only and
says so. The tool links `pibridge_eyed_core` for the eye-module enable (`HidHeadsetCommands`) and the start list / profile
loading (`make_config`), so it starts the chip exactly as `pibridge-eyed` does.

`ctest -R eyecal` runs 12 binaries, 64 tests; `pibridge_eye_calibration` (libpibridge-eye) 9 more:

- C++ twins of every Rust test of `pimax-eyecal` except `refit.rs`: layout 3, collect 1, sim 5 (full run, the Pimax
  path and its two buttons, the re-measure that never loops, head turn and glance, mirrored x), gpu 3 (`marks.cpp`
  `block`/`shade`), xrapp 2 (keys and CSV, the gaze forward); of the modules ported with it: `fixation.rs` 3,
  `calfit.rs` 3, `hudquad.rs` 5 and `font.rs` 4 (`hud.cpp`).
- New: the chip mode in the simulation (the captured order and sequence numbers; each 0x407 sent while the simulated
  eyes are within 1 degree of that target; the profile; the check finds and removes the simulated offset; identity +
  drift saved and the profile kept; the end screen; B stops with 0x3fc; a refused 0x42e fails and finishes the pass),
  the offset check (offset, gain error, mirrored axis, too few targets), stale answers, shutdown, the chip scene, the
  stamp and backup writer, the results JSON, the profile store (0600, backup, a bad profile keeps the old file,
  rollback, keep), the owner check on temporary /proc trees, the options over settings and environment, and the
  Runner on a live tracker over FakeChip (saved and kept with B, back to the grid, B exits; put back without; stopped
  mid-run: back to the grid, the finish answered, then B exits).
- The check grid flow: A / B / trigger / menu in the grid, during the run, on the end screen (saved and failed) and in
  the grid after a save; the HUD lines of each; the answers of a stopped run held back before a new run starts; the
  gaze dot through the host map (none: on the point looked at; a 3 degree drift: 3 degrees to the right; no sample: no
  dot); the simulated save-then-grid path (the grid's map is the saved one, the dot within 1 degree of where the
  wearer looks; the simulated chip is 3 degrees off without it) and a second run after the save that keeps it; the
  rollback to the saved profile after a kept save (`ProfileStore`); `--recenter` through the grid (saves, returns, A
  again) and its non-grid variants; the uniform block against the shader's std140 member order (read from
  `marks.frag`); the grid's rings, cross, lines and border, per eye with canted views; the dot's fill, outline,
  position and that it is drawn over the marks.
- libpibridge-eye: the target table, the requests, the guard (refused outside a session, exactly the five opcodes and
  the nine payloads, `frames::Guard` still refusing), the two passes on FakeChip (scripted answers, a multi-frame
  0x44c profile built with `profile_payload`), failed runs, the Link inside a live Tracker, a lost link.

`--simulate` runs the whole flow: the grid, A, the calibration, B (save), the grid with the new calibration for 1 s
(it prints how far the dot is from where the simulated wearer looks), B (exit).

No test opens a device node, starts an OpenXR runtime or touches a file of the user's.

## Not ported / differences

- `--refit RUN.json` (the offline re-fit of a saved run, `refit.rs` `load_run`/`fit_run`/`score`/`refit` and its two
  tests): not ported; the sample-level fit it shares with the live procedure (`eye_data`, `fit_eye`) is.
- The Rust tool's defaults were the grid and `~/.config/omarchyvr/`; here the chip's calibration, the Pimax path for
  `--host-only`, and `~/.config/pibridge/`. Results go to `~/.local/share/pibridge/eye-calibration/`.
- The end-to-end check (S, grid pattern only) needs the runtime's eye gaze, which under SteamVR comes from
  `pibridge-eyed`; while the tool runs, `pibridge-eyed` is stopped, so the check reports no gaze there.
- The tool's tracker stays raw: the check grid applies `pibridge-eyed`'s host map and filter in the procedure, not
  `pibridge-eyed`'s running drift estimate (`Tracker::observe`, zero unless something feeds it).
- The sign of the chip's target table on the Crystal Super is an inference (see "Where the targets are shown"); the
  check and `--chip-mirror` exist for the first test.
