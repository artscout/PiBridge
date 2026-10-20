# pibridge-eyed

Runs the eye tracker of a Pimax headset on its own (no Monado) and publishes the gaze in shared memory for the
SteamVR driver (CustomHeadsetOpenVR with the Linux patch, which hands it to SteamVR's eye tracking component and so to
OpenXR `XR_EXT_eye_gaze_interaction`). Port of `omarchyvr eyed` (`rust/omarchyvr-tools/src/eyed.rs`, MIT OR
Apache-2.0) on libpibridge-eye's live tracker (`pibridge::eye::live`, docs/libpibridge-eye.md) and libpibridge-hmd (the headset command).
Sources in `tools/eyed/`: the state machine is `eyed::Daemon` (`src/eyed.{h,cpp}`), the eye-module enable over
libpibridge-hmd is `eyed::HidHeadsetCommands` (`src/hid_commands.{h,cpp}`), `src/main.cpp` only wires signals and the clock.

## What it does

- Creates the gaze file (default `/dev/shm/omarchyvr-eye-gaze`, 136 bytes, "OVEG", the layout the deployed SteamVR
  driver reads) and writes "no gaze" into it.
- Starts the live tracker: finds the EyeChip (2104:0561), powers the eye module when the headset is there and the
  chip has been missing for 5 s (once per headset enumeration), runs the start list through the guard (with the
  user's eye profile in place of the list's own upload), subscribes, decodes the gaze with libpibridge-eyemath (clock map,
  frame conversion, `eye-calibration.toml`, filter, drift) and recovers a silent chip (eye-module enable at most every
  5 s, USB port reset after two silent links, at most once a minute).
- Every new sample goes into the gaze file at once (the loop runs every 2 ms); without one the file is rewritten twice
  a second with the tracker state (0 searching, 1 connecting, 2 streaming, 3 reconnecting, 4 stopped), so readers
  see that the writer lives.
- Logs "eyes seen" / "no eye seen yet", "eyes lost after X s", "eyes seen again after X s" (lost = 500 ms without
  a single sample with a valid eye; about half the samples carry no eye even when worn), and the sample rate once a
  minute.
- **No eyes for 4 s while streaming** (the Crystal Super switches its eye module off when the headset is taken off
  and not on again when it is put back on; Pimax Play then sends the enable): the captured enable
  `58 05 3c 05 00 01` is sent once for that stretch (`PIBRIDGE_EYE_POWER_ON=0` turns it off).
- **Follow** (`follow=vrserver` by default): the tracker runs only while the process runs; it starts with it and
  stops (unsubscribe 0x4ce, chip released) 5 s after it is gone. The gaze file then says "stopped".
- **SIGHUP**: re-reads `pibridge.conf` (the command line still overrides it). The tracker restarts when the start
  list, the profile, `eye_flip_x` or `eye_tracking` changed (or the last configuration was broken); a new followed
  process is taken in its current state. The gaze file path changes only at the next start.
- **SIGTERM / SIGINT**: the tracker unsubscribes and releases the chip, the gaze file says "no gaze", the status file
  is removed, exit 0.

Only one program may own the eye chip: `omarchyvr-eyed`, Monado's eye tracking and `pimax-eyecal` must not run at the
same time. The unit conflicts with `omarchyvr-eyed.service` and `pimax-eyed.service` (this daemon under PiBridge's earlier name), and the app stops `pibridge-eyed` while `pimax-eyecal`
runs (Calibrate / Recenter) and starts it again afterwards.

## Options

Defaults come from `~/.config/pibridge/pibridge.conf`; flags override them.

| Flag | Setting | Meaning |
|---|---|---|
| `--frames FILE` | `eye_start_list` | the start list made by `pibridge-eye frames` from a Windows capture of this headset (the Crystal Super has no built-in list). Default: `~/.config/pibridge/eye-start-crystal-super.txt`, or `~/.config/omarchyvr/eye-start-crystal-super.txt` when only that one exists; none: the built-in base list (the Dream Air's start). The list's own profile upload is sent (it is this user's Pimax Play eye profile). |
| `--profile FILE` | `eye_profile` | the user's eye profile (`pibridge-eye profile` of Pimax's `calibration.setpm`), uploaded in place of the list's own. Default `eye-profile-crystal-super.bin` in the same folders. A missing file is skipped (the list's own upload). |
| `--no-profile` | | no eye profile upload at all |
| `--shm PATH` | | the gaze file (tests and experiments only: the SteamVR driver reads the default) |
| `--follow PROCESS` / `--no-follow` | `follow` | run only while PROCESS runs (`--follow ''` or `follow=` = always on) |
| | `eye_tracking` | `off`: the daemon stays idle (the app normally stops the unit instead) |
| | `eye_flip_x` | `1` (default): the Crystal Super's mirrored x. The `PIBRIDGE_EYE_FLIP_X` environment wins when set; a loaded `eye-calibration.toml` brings its own conversion |
| `--settings FILE` | | another settings file |

A path in the settings may start with `~/`. The environment of the live tracker applies as well (`PIBRIDGE_EYE_*`, see
`pibridge::eye::live::Config::from_env`: `PIBRIDGE_EYE_CALIBRATION`, `PIBRIDGE_EYE_CORRECTION`, `PIBRIDGE_EYE_FILTER`,
`PIBRIDGE_EYE_LATENCY_MS`, `PIBRIDGE_EYE_USB_RESET`, `PIBRIDGE_EYE_POWER_ON`, `PIBRIDGE_EYE_USB`, ...).

A start list that cannot be read or parsed, or a profile that is not a profile, puts the daemon in the `error` state
with the reason; it keeps running (and keeps the gaze file at "no gaze") until a SIGHUP brings a working setting.

## Status file

`$XDG_RUNTIME_DIR/pibridge/eyed.json` through `pibridge::status::StatusWriter`, at every state change and at least every 2 s:
`state` (`idle` | `searching` | `connecting` | `streaming` | `error`; reconnecting counts as `searching`), `profile`
(`user` = the user's profile is uploaded, `list` = the list's own upload, `none`), `eyes_seen`, `samples_per_s`
(over the last second), `calibration` (the `eye-calibration.toml` in use, or ""), `follow`, `follow_running`, `error`
(the configuration error, or while searching the tracker's last line, e.g. "no EyeChip (2104:0561) on the USB bus";
"" when none), `pid`, `updated_unix_ms`. A clean stop removes the file (the app reads a missing file as not running).

## The eye-module enable

`HidHeadsetCommands` implements `pibridge::eye::power::HeadsetCommands` with libpibridge-hmd: `discovery::find_headset` gives the
model (Crystal Super 34a4:0040 or Dream Air 34a4:0044) and its nodes, `load_allowlist(model)` its `devices/<id>.toml`,
`Allowlist::build("eye_power_on")` the report, `power::headset_command_node` the hidraw node of interface 0, and
`pibridge::hmd::Gate` checks and writes it (`00 58 05 3c 05 00 01`, zero padded to 64). A command the description does not
list is refused before anything is opened. The node is opened for that one write; the keepalive daemon keeps its own
handle (hidraw allows several writers).

## Running

`tools/eyed/pibridge-eyed.service` is the systemd user unit (`ExecStart=%h/.local/bin/pibridge-eyed`, `ExecReload` =
SIGHUP, `Restart=on-failure`, `RestartSec=5`, `Nice=-5`, `Conflicts=omarchyvr-eyed.service`,
`After=pibridge-keepalived.service`; `cmake --install` writes a copy with the installed bindir). There is no
`Environment=PIMAX_EYE_FLIP_X=1` as in `omarchyvr-eyed.service`: `eye_flip_x=1` in the settings covers it. Log lines
go to stdout (`HH:MM:SS message`; the tracker's own lines start with `pibridge-eye: `).

## Tests

`ctest -R pibridge_eyed` (2 binaries, 13 tests). `pibridge_eyed_eyed`: the daemon against FakeChip trackers (gaze in the
temporary gaze file with the left eye under `eye_flip_x=1`, "no gaze" and the status file removed after stop, the
status fields), follow (start with the process, 5 s grace, stop), the eye-module enable after 4 s without eyes (once
per stretch, again after the eyes were seen and lost again, never with power-on off), SIGHUP (restart only on a
changed setting, eye tracking off/on, a new followed process, the gaze file path kept), the error state of a missing
start list and its recovery, the 2 s status rewrite, `make_config` (list, user profile, missing and broken profile,
`--no-profile`, broken list, no list), the options (settings keys, the default files: pibridge, then pimax-linux, then omarchyvr,
flags, errors) and the unit file. `pibridge_eyed_hid_commands`: the enable bytes on interface 0 of a fake Crystal Super
and Dream Air sysfs tree, refusals (no headset, unlisted command, no description) without a write, a failing write.
Everything runs on temporary `HOME`, `XDG_CONFIG_HOME`, `XDG_RUNTIME_DIR` and gaze paths; nothing opens a device.

## Not ported / differences

- The Rust daemon read only its command line and the environment; the settings file, SIGHUP, the status file,
  `--settings`, `--no-follow` and the idle state for `eye_tracking=off` are new (docs/CONVENTIONS.md).
- A start list that cannot be read is an `error` state here; the Rust daemon exited with an error.
- The Rust daemon wrote the enable to the hidraw node directly (`power::send_enable(node)`); here it goes through the
  allowlist gate, and the log names the target from the command sink.

## Paused during inside-out tracking

While `$XDG_RUNTIME_DIR/pibridge/inside-out.active` names a running owner (written by the app's inside-out sessions,
`common/pibridge_pause.h`), pibridge-eyed leaves the headset to Monado's Pimax driver: it closes the device, reports
`"state": "paused"` and `"paused": "paused: inside-out tracking"`, and takes it again when the marker is gone (checked
twice a second; a stale marker of a crashed app does not count). Tests: the pause / resume cases of its test.
