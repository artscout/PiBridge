# pibridge-keepalived

Keeps a Pimax headset awake the way Pimax Play does on Windows, and only while the VR runtime runs. Port of
`omarchyvr keepalived --video <hz> --follow <proc> --audio` (`rust/omarchyvr-tools/src/keepalived.rs`, MIT OR
Apache-2.0) on libpibridge-hmd. Sources in `tools/keepalived/`; the state machine is `keepalived::Daemon`
(`src/keepalived.{h,cpp}`), `src/main.cpp` only wires signals and the clock.

## What it does

- Waits for the headset (Crystal Super 34a4:0040 or Dream Air 34a4:0044), sends the wake, then the keepalive
  `58 04 39 00 RR 00` every 213 ms with RR from the rate file, keeps IF1-IF3 open and drained, and starts over when the
  headset re-enumerates. Every report passes the allowlist of the connected model (`devices/<id>.toml`).
- **Idle** (the followed process does not run): the rate file says `off`, so the headset gets one keepalive with rate
  0 and then none; the session keeps reading it. The LED goes to the green standby.
- **Active** (the process runs): the wake frame, the rate file at FPS, and 2 s later the video frames
  `58 05 5b 05 02 02`, `58 04 29 00 01`, `58 0d 1a 05 02 FPS`, `58 0d 1a 05 05`, then (with audio) the headset's USB
  audio becomes the default output and input. The video frames go out again 2 s after every reconnect while busy.
- **Back to idle** 5 s after the process is gone (SteamVR restarts its server on its own): rate file `off`, the audio
  defaults come back (only those the user did not change meanwhile).
- **SIGTERM / SIGINT**: clean stop. While busy: rate file `off` and one keepalive with rate 0 (the last thing Pimax
  Play sends when it closes); the audio comes back; the session closes; the status file is removed.
- **SIGHUP**: re-reads `pibridge.conf` (the command line still overrides it) and applies follow, rate and audio at
  once (a new followed process is taken in its current state, without the 5 s grace). `--rate-file` and `--no-wake`
  change only at the next start.

## The original Pimax Crystal

The original Crystal (`34a4:0012`, "P3B", `src/crystal.{h,cpp}`) has its own life next to the session above: it is
switched on with its power button and speaks another protocol (1024-byte RPC frames on its HID interface 2,
`libpibridge-hmd` `p3b.h`; only the frames Pimax Play sends, byte for byte from the Windows recording of 2026-10-22).

- **Connect** when its hidraw appears: `screen_state 1`, `control_signal_restart_runtime 0`, `video 120`, the
  Lighthouse properties, `setTobiiEyeDeviceEnable 1` (its eye tracker comes up on USB as `2104:0220`), then
  `getConnectState` every 10 s. The `piPcSwicthIoctl` replies give the video link (`video link 2`: the panel shows the
  PC's picture).
- **Tracking mode** (`crystal_tracking` = `lighthouse` | `slam` | `keep` in `pibridge.conf`, the Dashboard's selector):
  `EnableLighthouse 1` and 8 s later `LighthouseTakeEffect 1`, or `EnableLighthouse 0`. The mode it really runs comes
  from its head reports (byte 113, flags of its own inside-out: 1 orientation, 2 position; 0 in Lighthouse mode).
- **SteamVR and PiBridge** (`crystal_autostart_steamvr`): started once per connection, through `systemd-run --user
  --scope` (out of this service's cgroup), not before 10 s after `video 120` (its DisplayPort replug) and in Lighthouse
  mode not before its Lighthouse module (`28de:2300`) has been on USB for 3 s. A tracking change or the app's
  `restart-steamvr` request (a new refresh rate) closes SteamVR (SIGTERM, SIGKILL after 10 s) and starts it again.
- **Poses**: every head and controller report goes to `/dev/shm/pibridge-crystal-pose` (`libpibridge-hmd`
  `pose_shm.h`), the headset clock mapped to `CLOCK_MONOTONIC`, the head's x-up space turned y up, the controllers'
  buttons, touches, trigger, grip and stick with them. CustomHeadsetOpenVR's SLAM headset reads it.
- **Sound**: while it is on, its microphone and its DisplayPort sound (the GPU audio card's port whose monitor is
  "Pimax", the card profile switched to it and back) are the defaults; the output is looked for every 3 s until
  SteamVR drives its display.
- **The Monado backend** (`vr_backend=monado` in `pibridge.conf`): on connect only the PiBridge app is started (its
  **START VR** runs Monado for the headset), never SteamVR, which would take its Lighthouse module. The poses keep going
  to the shared memory above (Monado's `pibridge_og` builder reads them); the app restarts its Monado session when the
  tracking changes. Its HID stays with this daemon also while it is paused for a Monado session.
- **Volume**: its buttons change its own volume (it plays the DisplayPort sound at it, head report byte 132); a change
  is written to `$XDG_RUNTIME_DIR/pibridge/volume-hud` for the indicator in the headset (`pibridge-xroverlay`).
- **Requests** from the app (`$XDG_RUNTIME_DIR/pibridge/crystal-request`): `shutdown`, `restart-steamvr`.
- **Status**: `crystal_present`, `crystal_stage`, `crystal_lighthouse`, `crystal_tracked`, `crystal_battery`, and its live
  IPD as `lens_ipd_x100`.

## Options

Defaults come from `~/.config/pibridge/pibridge.conf` (`refresh_hz`, `follow`, `audio_to_headset`); flags
override them.

| Flag | Meaning |
|---|---|
| `--video 72\|90` | panel rate (default `refresh_hz`, 90) |
| `--no-video` | plain keepalive, the panel is left alone (the Rust daemon without `--video`); no follow, no audio |
| `--follow PROCESS` | busy only while PROCESS runs (default `follow`, `vrserver`); `--follow ''` or `--no-follow` = always on |
| `--audio` / `--no-audio` | headset audio while busy (default `audio_to_headset`, on) |
| `--no-wake` | skip the wake frame at connect |
| `--rate-file PATH` | default `$XDG_RUNTIME_DIR/pimax-rate` (the omarchyvr path, so both stacks read the same file) |
| `--settings PATH` | another settings file |

As in Rust, `--follow` and `--audio` need the video mode (`--no-video --follow X` is a usage error). New: with
`follow=` (always on) the audio switch also works (to the headset at start, back at stop).

## Status file

`$XDG_RUNTIME_DIR/pibridge/keepalived.json` through `pibridge::status::StatusWriter`, at every state change and at least every
2 s: `state` (`idle` | `waking` | `active` | `no-headset` | `error`), `model`, `headset_present`, `panel_on`,
`rate_hz`, `follow`, `follow_running`, `audio_on_headset`, `error`, `pid`, `updated_unix_ms`. `waking` = busy, the
video frames not sent yet; `error` = no headset and a reason (e.g. no permission to open hidraw, the udev rule);
`error` also carries a failed video-on or rate-file write while the headset is there. A clean stop removes the file.

## Running

`tools/keepalived/pibridge-keepalived.service` is the systemd user unit (`ExecStart=%h/.local/bin/pibridge-keepalived`,
`ExecReload` = SIGHUP, `Restart=always`, `Conflicts=omarchyvr-keepalived.service`; `cmake --install` writes a copy
with the installed bindir). With the default settings it behaves like omarchyvr's unit plus the Crystal Super
Lighthouse drop-in (`--video 90 --follow vrserver --audio`). The app starts, stops and reloads it with
`systemctl --user`. Log lines go to stdout (`HH:MM:SS message`, the journal keeps them).

## Tests

`ctest -R pibridge_keepalived` (29 tests): the whole idle -> active -> idle cycle against the fake Crystal Super (one
rate-0 keepalive then silence; wake + rate at the start; the four video frames exactly after 2 s; audio switched and
given back; status file contents), SIGTERM while busy (rate 0 is the last report, audio back), always-on, reconnect
while busy, the Dream Air at 72 Hz with its own description, SIGHUP reloads, the 2 s status rewrite, the option
parsing, and the original Crystal against a fake link (connect frames, the Lighthouse switch both ways, SteamVR and
the app only after the Lighthouse module and the DisplayPort settle, the restart for a tracking change or on request,
the live tracking flags, its sound taken when the output shows up and given back, the Lighthouse module in a fake sysfs). The followed process, pactl and sysfs are fakes; the status file goes to a temporary `XDG_RUNTIME_DIR`.

## Not ported

- `--no-wake` and `--rate-file` are not re-applied on SIGHUP (they need a new session).
- The Rust daemon printed the hidraw node of IF0 at connect from the real sysfs; here it uses `Deps::roots`
  (the real sysfs by default, a dummy in tests).

## Paused during inside-out tracking

While `$XDG_RUNTIME_DIR/pibridge/inside-out.active` names a running owner (written by the app's inside-out sessions,
`common/pibridge_pause.h`), pibridge-keepalived leaves the headset to `pibridge-headtrack`: it closes the device, reports
`"state": "paused"` and `"paused": "paused: inside-out tracking"`, and takes it again when the marker is gone (checked
twice a second; a stale marker of a crashed app does not count). Tests: the pause / resume cases of its test.

While paused for a Monado session, the Crystal Super's volume buttons come from Monado's Pimax driver
(`$XDG_RUNTIME_DIR/pibridge/volume-button`, `<seq> <dir> <pressed>`): every entry after the one there at the pause
counts, the session's first press too (the driver counts from 1 in each session). Each step of the volume, by a button
in either way, also goes to `volume-hud` (the indicator in the headset).

## Controllers
Pair, Unpair and Test vibration of the Dashboard's Controllers card run here, on the daemon's own session (the only process that
has the headset open while no inside-out session runs). Every frame goes through the allowlist gate.
- **Requests**: `$XDG_RUNTIME_DIR/pibridge/controller-request`, one line `ID pair|unpair left|right` (libpibridge-hmd
  `pairing.h`), and `$XDG_RUNTIME_DIR/pimax-haptic-request`, `SLOT AMP MS` (`haptics.h`). Taken (renamed, read,
  removed) on every tick (at most 0.5 s).
- **Pair**: `58 04 04 00 SS` (allowlist `start_pair`, once per request, never retried), then the firmware log (IF3):
  `pair finished` / `radio_pair_success_process`, or a heartbeat of that controller, = paired;
  `radio_pair_cancel_or_timeout_process` or 35 s = timeout. Refused for a controller heard in the last 10 s
  ("already paired and connected") and while another pairing listens.
- **Unpair**: `58 03 05 SS`, waits 1 s for `4c 04 05 SS 01 ..`.
- **Test vibration**: refused when the controller was not heard in the last 10 s; a controller that is not streaming
  gets the attach burst (`read_serial`, `read_info`, `read_doc_controller`, `read_version`) first, then
  `58 09 07 SS AMP DUR`.
- **Status** (keepalived.json): `ctrl_id`, `ctrl_op`, `ctrl_side`, `ctrl_state` (waiting | paired | timeout |
  unpaired | failed | refused), `ctrl_detail` (plain words), `left_connected`, `right_connected` (report or 0x37
  heartbeat within 10 s), `haptic_seq`, `haptic_result`.
- **Paused** (inside-out): pair and unpair are refused with the reason; the vibration file is left to
  `pibridge-headtrack`.
- Crystal Super: `start_pair` and `unpair` were not in its Windows capture; they are the Dream Air's approved frames
  for the same controllers (devices/crystal-super.toml). Not yet run against a headset.

