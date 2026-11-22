# pibridge-roommap, pibridge-room, pibridge-anchor

The room-map tools (`tools/roommap/`), C++ ports of omarchyvr's `omarchyvr-roommap`, `omarchyvr-room` and
`omarchyvr-anchor` (MIT OR Apache-2.0, pin `5ffbcc5`). Same subcommands, options, output and files as the Rust tools;
error lines start with the PiBridge name. Libraries: `docs/libpibridge-roommap.md`, `docs/libpibridge-anchor.md`.

```
pibridge-roommap build      (--rec DIR [--tracking CSV] | --source broker [--seconds 60]) [--name NAME | --map DIR]
                         [--every N] [--kf-dist M] [--kf-angle DEG] [--no-loops] [--refine [--keep-unrefined]]
                         [--min-keyframes N --extend S] [--append]
pibridge-roommap relocalize (--rec DIR [--tracking CSV] | --source broker [--seconds 15]) [--name NAME | --map DIR]
                         [--every N] [--window S] [--threads N] [--result FILE] [--apply [--no-floor]]
pibridge-roommap apply      [--name NAME | --map DIR] [--result FILE] [--pid N] [--dry-run] [--no-floor]
pibridge-roommap info | export [--out F.json] | view [--out F.html] [--open]   [--name NAME | --map DIR]
pibridge-roommap floor      detect|drift|touch|set|show|follow|preview ...
pibridge-roommap setup      fast|full|show [--name NAME] [--label TEXT] [--no-hud] [--map-seconds S]
pibridge-roommap auto       [--name NAME] [--wait S] [--status FILE]
pibridge-roommap light      [--source broker|FILE.pxcam] [--seconds 5] [--every N] [--no-stretch] [--darken F [--noise G]]
pibridge-roommap cycle      RESULT.toml RESULT.toml ...
pibridge-roommap eval       --map-a A --rec-a DIR --map-b B --rec-b DIR [--every N] [--window S]
pibridge-room    build DATASET TRACKING.csv OUT_DIR | reloc MAP_DIR DATASET TRACKING.csv [--save] | info | anchor | demo
pibridge-anchor  status | clear
common: --calib XML (default: the headset calibration)
```

The flow the app needs: `build --source broker` (map the room), `relocalize --source broker --apply` (Recognise room),
`auto` (detached at the session start: wait for the head ring, take the writer lease, relocalise in 15 s windows up
to `--wait`, apply floor + centre + forward, status file), `setup fast|full` and `apply`.

`auto --confirm S` (the app adds `--confirm 12` with the Basalt engine, which keeps no map of its own; 2026-11-21): the
start fix is taken within seconds of the engine's start, before Basalt has settled (live: the fix at 2.5 s stood
15 cm / 4.3 deg from the one 16 s later), so S seconds after the room is in place it is recognised once more (6 s of
frames) and, when the new fix differs by over 3 cm or 1 deg, the offset is rewritten as a drift correction (`drift 1`,
`docs/libpibridge-anchor.md`: the tracker glides to it and keeps the user's recenter). A fix counts when it is sure
(sigma <= 1 cm / 0.5 deg, rms <= 1.5 deg); a jump (> 25 cm or 5 deg) only when the next fix agrees; at most four
recognitions, then the start fix stays. The tool is done after that: the room is not touched again in the session.

`auto --follow S` (retired as the app's default, 2026-11-21): the same recognition every S seconds for the whole
session, writing every fix as a drift correction. Live it did harm: a fix scatters 1-3 cm / 0.3-1.1 deg with where
the head stands, every one was applied (19 glides in a 4 min Beat Saber session, the offset's z spread over 10 cm),
and the user saw the world slide by centimetres every 10-16 s; whether Basalt drifts enough over a long session to
need any correction at all is not measured (the vertical "drift" seen in a 7 min session was posture). Still there
for experiments (`PIBRIDGE_ROOM_FOLLOW=0` turns both off):
- `PIBRIDGE_ROOM_FOLLOW_TAU=20` filters the follow: a room fix every ~4 s (1 s pause, 3 s of frames:
  `PIBRIDGE_ROOM_FOLLOW_GAP`, `PIBRIDGE_ROOM_FOLLOW_WIN`), filtered with this time constant, written in 1 mm steps.
- `PIBRIDGE_DRIFT_GLIDE=hidden` makes pibridge-headtrack glide a drift correction mostly while the head turns.
- SteamVR backend only (Monado's offset carries the whole room setup). It ends with the session or when another writer
  takes the lease (a room setup).

Not ported / different:
- PiBridge changes (2026-10-19): under SteamVR (the session pid is pibridge-headtrack, or a vrserver with Monado's
  driver) the floor and the room go into SteamVR's standing frame from `$XDG_RUNTIME_DIR/pibridge/steamvr-stage`
  (written by the app) instead of monado-service's 1.6 m origin; `floor detect --min-below M` (default 0.4 as before); the fast setup rejects a points floor less than
  1.0 m below the head and asks for the controller touch ([ROOM-SETUP.md](ROOM-SETUP.md) "The floor under SteamVR").
- PiBridge (2026-10-19): `setup fast --guide DIR` is the in-headset room setup's half: the prompts and the mapping
  feedback go to `DIR/state`, the floor (yes / touch), the boundary (keep the free area / a drawn path) and forward come
  back from `pibridge-roomsetup --map` in `DIR/answer` (`guide_ipc.h`, [ROOM-SETUP.md](ROOM-SETUP.md) "Map the room in the
  headset"); the files written are the same. Not in `setup`'s help text (the Rust text is kept).
- `setup` has no in-headset panel (the Rust tool opens an omarchyvr-hud OpenXR overlay): the steps are printed and
  the head comes from the tracker's head ring, so `setup fast` works (forward by holding still); `setup full` needs trigger
  input for the boundary, which only the app's in-headset room setup has. The panel note replaces the Rust one.
- `pibridge-anchor tag/detect/calibrate/apply` (AprilTag): not ported (needs the AprilTag library); exit 2.
- The live paths (`--source broker`, `floor touch/follow`, `auto` against a running session) are the same code paths
  as offline but were not run against a live session here (agents do not touch a running session).

## Tests

The difftests against the Rust tools (`pibridge_{anchor,room,roommap}_diff_*`: stdout, stderr and the files written,
byte for byte, on synthetic input and the lab takes) were removed with the oracle once the port was finished
(2026-10-22). The libraries are tested by `ctest -R "pibridge_(anchor|roommap)_"` (`docs/libpibridge-roommap.md`,
`docs/libpibridge-anchor.md`).

