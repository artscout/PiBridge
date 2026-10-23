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
to `--wait`, apply floor + centre + forward, status file), `setup fast|full` and `apply`. The offset goes through the
writer lease (`docs/libpibridge-anchor.md`); the tracker glides to every change after the session's first.

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

