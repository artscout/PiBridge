# PiBridge app (`app/`)

The Qt 6 Widgets desktop app, binary `pibridge` (CMake target `pibridge-app`), window title "PiBridge", window icon our gem
(`app/data/pibridge.svg`, drawn the same in code by `ui::appIcon()`), desktop entry `pibridge.desktop`.
It shows the state of the two daemons and changes their shared settings; it never talks to the hardware itself.
The look follows the spirit of the Windows headset software (dark panels, one indigo accent, uppercase letter-spaced
labels, outlined status chips) with our own drawings, icons and names.

```
pibridge                    the app
pibridge --fake             fixed fake state (also PIBRIDGE_FAKE_STATE=1): all XDG dirs point into a temporary
                               directory, no command is ever run, nothing of the user's is read or written
pibridge --screenshots DIR  render every page in fake mode into DIR/*.png and quit (uses the offscreen platform)
```

The app runs no command at start or in fake/screenshot mode. Commands run only when a button or switch is clicked,
except the journal tail of the Logs page, which starts the first time that page is opened (never in fake mode).

## How state flows

| What | Where | Who |
|---|---|---|
| Daemon status | `$XDG_RUNTIME_DIR/pibridge/keepalived.json`, `eyed.json` | read every 1 s (`core/status.*`, Qt JSON). Missing, broken or not rewritten for 6 s = "Service not running" |
| SteamVR running | a process with `/proc/*/comm` = `vrserver` | read every 1 s (`core/system.*`) |
| SteamVR devices | OpenVR, as a background application (see below) | a worker thread, once per second, only while vrserver runs (`core/steamvr*`) |
| Gaze | `/dev/shm/pibridge-eye-gaze` (136 bytes, "OVEG", seqlock) | read only, at ~30 Hz and only while the Dashboard is visible (`core/gazeshm.*`: seq_end, body, seq_begin by separate `pread`s, retried when torn; older than 200 ms = no gaze) |
| Shared settings | `~/.config/pibridge/pibridge.conf` (while it does not exist, `~/.config/pimax-linux/pimax-linux.conf` is read; the first change writes the new file) | written by the app (`core/conf.*`): only the switched key's lines change, every other line, comment and unknown key stays; temp file + rename; numbers with a point in every locale. The daemons re-read it on SIGHUP (sent with `systemctl --user kill --signal=HUP <unit>`) |
| Quad views settings | `~/.config/qv-vulkan/settings.conf` | written through the layer's own code (`qv/settings.*`), so the format is the layer's exactly |
| Quad views layer switch | `~/.local/share/openxr/1/api_layers/implicit.d/XR_APILAYER_ARTSCOUT_quad_views_vulkan.json[.disabled]` | renamed by `qv/layerinstall.*` |
| Last game | `~/.local/state/qv-vulkan/status.txt`, log `qv-vulkan.log` | read only |

### SteamVR devices (`core/steamvr.*`, `core/steamvr_openvr.cpp`)
- **How it connects**: the app vendors only the OpenVR SDK header (`app/third_party/openvr/openvr.h`, SDK 2.15.6,
  BSD-3, Valve) and loads `libopenvr_api.so` of the installed SteamVR with `dlopen`: the first entry of `runtime` in
  `$XDG_CONFIG_HOME/openvr/openvrpaths.vrpath` (default `~/.config/openvr/`), then `<runtime>/bin/linux64/libopenvr_api.so`.
  It resolves `VR_InitInternal2`, `VR_ShutdownInternal`, `VR_GetGenericInterface`, `VR_IsInterfaceVersionValid` (and
  `VR_GetVRInitErrorAsSymbol` for error texts) and does what the header's inline `VR_Init` does (interface check,
  `IVRSystem`, `SetSDKVersion`). No link-time dependency; missing file, runtime, library or symbols = "SteamVR not
  installed". The library stays loaded once loaded.
- **It never starts SteamVR**: it connects only as `VRApplication_Background` (that kind of client fails with
  `NoServerForBackgroundApp` instead of starting SteamVR), and only while the app's vrserver check says vrserver runs.
  A failed connect is retried no more often than every 5 s (also after SteamVR restarts). On `VREvent_Quit`, or when
  vrserver is gone, it calls `VR_ShutdownInternal` and goes back to idle. It calls nothing that changes SteamVR state.
- **What it reads**, once per second, for every connected device index 0..63: class (headset, controller, base
  station, tracker), hand role (left/right), `Prop_ModelNumber_String`, `Prop_ControllerType_String`, battery
  (`Prop_DeviceProvidesBatteryStatus_Bool`, `Prop_DeviceBatteryPercentage_Float`, `Prop_DeviceIsCharging_Bool`), the
  pose in the standing universe (valid + tracking result) and the activity level; and, through `IVRApplications` (same
  generic-interface path, optional), the compositor's process id for the Apps page's readiness check. **Serial numbers** (`Prop_SerialNumber_String`) are
  read for support: shown next to each controller and base station with a Copy button, never logged, never written to
  a file; fake mode and screenshots use made-up ones.
- Layers: `SteamVrSource` (the dlopen backend, or `FakeSteamVrSource` with scripted answers for tests and the fake
  mode), `SteamVrLink` (connect / back off / poll / disconnect, injected clock), `SteamVrWorker` (`std::jthread`, all
  OpenVR calls on it; hands each status to the GUI thread), and `controllersView` / `trackingView` (the plain words).

Every command goes through `core/runner.h` (`ProcessRunner` = QProcess; `FakeRunner` records the calls in tests and
fake mode). What each button does is in `core/actions.*`, tested with the FakeRunner.

## Pages

### Dashboard (`pages/dashboard.*`)
- Toolbar
  - **Runtime: SteamVR**: read-only for now.
  - **Launch SteamVR**: `steam steam://rungameid/250820`, detached (not when SteamVR already runs).
  - **Restart services**: `systemctl --user restart pibridge-keepalived pibridge-eyed`.
  - **Restart headset**: `systemctl --user restart pibridge-keepalived`.
  - **Headset driver settings**: starts CustomHeadsetOpenVR's configurator (sboys3's `custom-headset-gui`) detached.
    Shown only in the Lighthouse/SteamVR mode, i.e. while CustomHeadsetOpenVR is the headset's SteamVR driver: an
    `external_drivers` entry of `openvrpaths.vrpath` ends in `/CustomHeadsetOpenVR` (or `<runtime>/drivers/
    CustomHeadsetOpenVR` exists), and `~/.config/CustomHeadset/settings.json` has an enabled headset (`"enable": true`
    or no `enable` key, CHOVR's default; a file without headset entries runs on the defaults and counts). Re-checked
    whenever the Dashboard is shown. The configurator, first match: `custom-headset-gui` in PATH,
    `~/.local/bin/custom-headset-gui`, the path in `chovr_gui=` (pibridge.conf, optional), the source build
    `~/gamedev/ext/CustomHeadsetOpenVR/output/CustomHeadsetGUI/release/custom-headset-gui`. Not found: disabled,
    tooltip "CustomHeadsetOpenVR's configurator was not found" (`core/chovr.*`).
  - **Reset SteamVR** (`core/steamvr_reset.*`): for a SteamVR that is stuck half-started, or a Steam that claims
    "SteamVR is already running" when nothing runs. A click scans `/proc` and shows a dialog; nothing happens before
    the user confirms.
    - **What counts as SteamVR**: a process whose `/proc/<pid>/exe` (a trailing " (deleted)" ignored) or `argv[0]`
      is under the SteamVR runtime dir (`runtime[0]` of `openvrpaths.vrpath`, else
      `~/.local/share/Steam/steamapps/common/SteamVR`; also its canonical path): `bin/linux64/*` (vrserver,
      vrcompositor, vrmonitor, ...), `bin/vrwebhelper/*`, `tools/steamvr_environments/*` (steamtours); and wrappers
      (pressure-vessel, a shell running `vrstartup.sh`) whose other arguments name that dir. Never by name: a stuck
      vrserver can show the comm "36674: vrstartu". Never Steam's client (`ubuntu12_32/`, `ubuntu12_64/`: steam,
      steamwebhelper, reaper), a look-alike dir (`SteamVR2/`) or the app itself.
    - **Socket holders**: `/proc/net/unix` paths starting with `@/steamvr/` give inodes, the `socket:[inode]` links
      in `/proc/<pid>/fd` give the PIDs. The dialog says "holds the SteamVR socket" (`@/steamvr/SteamVR_Namespace`).
      A holder outside the SteamVR dir is named ("not part of SteamVR: left alone") and never signalled; a socket no
      readable process holds is named too.
    - **Dialog**: one line per process, e.g. "vrserver (PID 36868, running 1 h 24 min, holds the SteamVR socket)"
      (running time from `/proc/<pid>/stat` start time and `/proc/uptime`; "shows as ..." when the comm differs).
      When SteamVR looks healthy (a vrserver with comm `vrserver` runs and the OpenVR background connection is
      connected) the title is "SteamVR looks fine" and it asks "Close SteamVR anyway?". Cancel is the default button.
    - **Close SteamVR**: SIGTERM to exactly the listed PIDs, SIGKILL after 5 s to the ones still alive, then up to
      2 s more; the result in the message line ("Closed: vrcompositor (PID 2), vrserver (PID 1, needed a forced
      stop). SteamVR can be started again."; "Still running: ...", "Could not close: ... (not allowed)"). Before every
      signal the PID's start time is compared with the scan: a reused PID is left alone. PiBridge's own daemons are not
      touched (they follow vrserver).
    - **Nothing of SteamVR runs**: "No SteamVR process is running. If Steam still says SteamVR is running, Steam's
      own state is stuck: restart Steam.", warning "Games started from Steam will close.", **Restart Steam**:
      `steam -shutdown`, wait (max 20 s) until the Steam client (comm `steam` or exe `.../ubuntu12_32/steam`) is gone,
      then `steam` detached. If Steam does not close in 20 s, nothing is started and the line says to exit Steam from
      its menu. Without a running Steam the button is **Start Steam** (just `steam`, detached).
    - Fake mode shows the dialog with a made-up healthy SteamVR (active) or nothing (idle) and never signals or
      starts anything.
  - The result (or the error output) is shown next to the buttons.
- **Headset card**: title = `model` from keepalived.json ("No headset" without it); chip from `state` / `panel_on`:
  SCREEN ON (indigo), SCREEN OFF (amber), NO HEADSET (amber), ERROR (red), SERVICE OFF (grey, no daemon); a hint in
  plain words ("Start SteamVR to wake the headset" when idle); the `error` text in red; state, panel rate, followed
  process; **Refresh rate 72 / 90 Hz** -> `refresh_hz` in the conf + SIGHUP to `pibridge-keepalived`.
- **Controllers** (live from SteamVR): Left / Right rows with the model ("Valve Index Controller"), a chip CONNECTED
  (tracked) / NOT TRACKED (with the reason: "Out of range", "Calibrating", "Not seen"; "standby" when asleep) /
  DISCONNECTED, and the battery (icon, "82 % charging"); header chip "N/2 CONNECTED". Other controllers and trackers
  are listed below. Without SteamVR: STEAMVR OFF and "Start SteamVR to see the controllers."; NO STEAMVR when it is
  not installed; CONNECTING while the connection is retried. While "Monado directly" (Development page) runs the card reads
  Monado through libmonado instead (`core/monado_devices.*`): header MONADO: N/2 ON (MONADO OFF, NO LIBMONADO,
  CONNECTING), each controller ON with its battery, OFF ("Turn it on") or LISTED, and "Turn the controllers on." /
  "Start tracking (Tracking page) to see the controllers."; the toolbar says "Runtime: Monado" (TRACKING.md).
- **Quick settings**
  - **Eye tracking** switch -> `eye_tracking=on|off` + `systemctl --user start|stop pibridge-eyed`.
  - **Calibrate** -> `pibridge-eyecal` (the chip's own calibration, as Pimax Play does it, see docs/eyecal.md);
    **Recenter** -> `pibridge-eyecal --recenter`. Without `pibridge-eyecal` the app falls back to omarchyvr's `pimax-eyecal`.
    The eye chip has one owner: both first run `systemctl --user stop pibridge-eyed`, and when the tool exits
    `systemctl --user start pibridge-eyed` (only while `eye_tracking=on`).
    While one runs, the buttons are disabled and the line below says so; then the result (last output lines).
  - **Import calibration from Windows...** -> file dialog for `calibration.setpm`, then
    `pibridge-eye profile <file> -o ~/.config/pibridge/eye-profile-crystal-super.bin`; on success `eye_profile=<that
    path>` in the conf and SIGHUP to `pibridge-eyed`.
  - Tools are looked up in PATH, next to the app binary, in this build's `tools/*/`, then in the omarchyvr release
    build (`~/gamedev/omarchyvr/rust/target/release`). Not found: the button is disabled and its tooltip says why.
  - **Eye tracker** line: eyed.json `state`, `eyes_seen`, `samples_per_s`, the profile; a live gaze dot.
- **Tracking**: LIGHTHOUSE chip; "N base stations, M tracking" (a station counts as tracking when its pose is
  valid) and one row per station ("Station 1  SteamVR Base Station 2.0  ok / not seen"). Without SteamVR: "Start
  SteamVR to see the base stations."
- **Sound settings**: **Default audio output** switch -> `audio_to_headset=on|off` + SIGHUP to `pibridge-keepalived`.
  **Headset sound test** (`core/sound_test.*`, `pages/sound_tests.*`), like Pimax Play / EVO; the default output and
  input are never changed. The headset's nodes come from `pactl list short sinks|sources` (the prefixes of
  pibridge-keepalived's audio switch, monitors left out).
  - **TEST SPEAKERS**: a generated stereo WAV (0.7 s 440 Hz left only, 0.25 s silence, 0.7 s 660 Hz right only) in
    `$XDG_RUNTIME_DIR/pibridge/`, played with `paplay --device=<headset sink>`, then deleted.
  - **TEST MICROPHONE**: `parec --device=<headset source> --raw` (mono, 16 bit, 48 kHz) for 3 s with a live level
    meter (peak, -60..0 dBFS), then the recording is played back on the headset sink and its temporary WAV deleted;
    the line says the loudest level and warns when it stayed under -40 dB ("muted?").
  - Errors in plain words: headset speakers / microphone not found (turn the headset on), pactl / paplay / parec
    missing, the sound server not answering, playback failed, the recording stopped, no sound from the microphone.
    Fake mode plays and records nothing.
- **Record video** (`pages/record_card.*`, `core/video_record.*`, docs/record.md): records what the headset shows
  into `~/Videos/pimax/pimax-YYYYmmdd-HHMMSS.mp4` with `pibridge-record start` / `pibridge-record stop`. **Eyes** (one eye /
  both), **FPS** (30 / 60 / 72), **Quality** (low / medium / high) and **Sound** are saved at once in
  `~/.config/pibridge/record.toml` (the file pibridge-record reads). The chip: REC (red) while recording, READY, STEAMVR,
  OFF; the line below: the length, size and encoder, or why it cannot record, or the last error in plain words; the
  last file with **Open folder** (`xdg-open`). RECORD is enabled while SteamVR runs (SteamVR's mirror texture through
  pibridge-record-mirror, also for SteamVR + Monado and SteamVR's OpenXR runtime) or Monado's own compositor runs
  ("Monado directly" on the Development page: Monado's tap); the card picks the source by the session. The status is read every
  second while the Dashboard is visible. Fake mode runs nothing.

### Device (`pages/device.*`, `core/device_info.*`)
Like the headset details of Pimax EVO, read at run time and shown only in the app (COPY buttons for serials; never
logged or written to a file; fake mode and screenshots use made-up serials). Files are re-read when the page is
shown; the live values follow the state every second.
- **Headset card** (title "Pimax <model>", chip STEAMVR LIVE / CONNECTING / STEAMVR OFF): model (keepalived.json,
  else SteamVR), **Headset SN** (`Prop_SerialNumber_String` of the HMD; "Start SteamVR to read it."), **IPD** (SteamVR's
  `Prop_UserIpdMeters_Float`, what it renders with; without SteamVR the `ipd` (+ `ipdOffset`) of the enabled
  Crystal Super entry of CustomHeadsetOpenVR's settings.json, marked "not read live"). The headset reports no
  hardware IPD on Linux (CHOVR's `hardwareIpd` reads it through Pimax's Windows runtime only), and the page says so.
  **Measured IPD**: the eye tracker's `ipd_mm` from the gaze shm while it streams.
- **Controllers and base stations**: one row per controller / tracker / station with its serial and model (SteamVR).
- **Pair and test controllers** (`core/controllers.*`, docs/keepalived.md "Controllers"): per hand the state
  (Connected / Not connected / In use by inside-out tracking / No headset / Headset service off) and **Pair**, **Unpair**
  (one-line confirm) and **Test vibration** (60 %, 0.3 s). Pair and Unpair write `ID pair|unpair left|right` to
  `$XDG_RUNTIME_DIR/pibridge/controller-request` and show keepalived's answer (`ctrl_*`, a 30 s countdown while the
  headset listens); refused in plain words during inside-out tracking, without the service or the headset, and Pair
  for a controller that is already connected. Test vibration writes `SLOT 60 300` to
  `$XDG_RUNTIME_DIR/pimax-haptic-request`, taken by keepalived (answer `haptic_*`) or, during inside-out tracking, by
  Monado's Pimax driver. No answer in 3 s: the request is removed and the page says so.
- **Rendering**: render resolution per eye (`IVRSystem::GetRecommendedRenderTargetSize`, supersampling included, plus
  the supersampling of `steamvr.vrsettings`: `supersampleScale` when `supersampleManualOverride`, else Auto); refresh
  rate (`Prop_DisplayFrequency_Float`, else keepalived's panel rate, else `refresh_hz`); quad views on / off / not
  installed (the layer's user manifest); tracking mode (Lighthouse, inside-out SteamVR + Monado, Monado directly,
  "running" while a session runs); the Monado build (Rust / C++ (PiBridge), the markers, scanned on a worker thread).

### Apps (`pages/apps.*`)
Steam games and your own programs as cover tiles, like the library of the Windows headset software.

- **Top bar**: a search box (name contains, any case); **VR only** (on by default: only the Steam games SteamVR lists
  as VR apps or VR overlays; your own apps are always shown; while SteamVR has not written its list yet every game is
  shown and the bar says so); the SteamVR readiness chip (STEAMVR READY / STEAMVR STARTING with what it waits for /
  STEAMVR OFF "Play starts it" / NO STEAMVR); **Add app...**.
- **Tiles** (200 x 396): the cover, a chip over it (RUNNING / STARTING / STARTING STEAMVR / ERROR / READY when
  SteamVR is fully ready), VR badge (Steam), the name, one line (last played: "Played today, 14:02", "Played
  yesterday", "Played 3 days ago", "Played 2 Sep 2026", "Never played"; or "PID 4321", "Exited (code 1)" (red when not
  0), "Crashed", the error in red), and **PLAY** (own apps: **STOP** while running, **CANCEL** while waiting for
  SteamVR). Right click: Play / Stop, Edit..., Quad views profile..., Open log (`xdg-open` of the latest log), Remove.

#### Steam games (read only, no network; `core/apps_vdf.*`, `core/apps_steam.*`)
- Root: the first existing of `$XDG_DATA_HOME/Steam` (`~/.local/share/Steam`), `~/.steam/steam`, `~/.steam/root`
  (canonical paths, duplicates dropped) that has `steamapps/`.
- Libraries: the root, then every `"path"` in `steamapps/libraryfolders.vdf` (fallback `config/libraryfolders.vdf`;
  also the old `"1" "/path"` form); missing folders are skipped.
- Games: `<lib>/steamapps/appmanifest_*.acf` (`AppState`: `appid`, `name`, `installdir`, `StateFlags` (bit 4 =
  installed; others are skipped), `LastPlayed`). Never shown: SteamVR (250820), Proton (every build), the Steam Linux
  Runtimes, Steamworks Common Redistributables (a list of appids plus name rules).
- VR: `config/steamapps.vrmanifest` and every manifest `config/appconfig.json` names (JSON, `applications[].app_key`
  = `steam.app.<id>` or `steam.overlay.<id>`, `last_played_time`). Last played = the later of the manifest and SteamVR.
- Covers from Steam's cache `appcache/librarycache/`: portrait first (`<id>/library_600x900.jpg`,
  `<id>/<hash>/library_capsule.jpg`, `<id>_library_600x900.jpg`), then the wide header (`<id>/header.jpg`,
  `<id>/<hash>/library_header.jpg`, `<id>_header.jpg`; drawn across the middle on a dark enlarged copy); without one
  our own generated art (a gradient from the name and its initials).
- The KeyValues reader: quoted or bare keys/values, `\" \\ \n \t` escapes (other backslashes kept, for Windows
  paths), `//` comments, `#include`/`#base` lines and `[$COND]` tags skipped, keys without regard to case, a depth
  limit; broken text gives what was read plus an error (the game is then skipped and named in the section's tooltip).
- Read when the page is opened (at most every 10 s). Running: a process whose command line has `SteamLaunch
  AppId=<id>` (Steam's `reaper`), checked every 2 s while the page is visible.
- **Play**: `steam steam://rungameid/<id>`, detached (Steam starts SteamVR itself for a VR game). STARTING for up to
  45 s, then RUNNING once the game is seen. Steam games cannot be stopped from here.

#### Own apps (`core/apps_own.*`, `core/apps_launch.*`)
- **Add app... / Edit**: name, program (file dialog), arguments (one line, shell-like quoting), working folder (empty
  = the program's folder), environment (`KEY=value` per line), icon (optional image), **Quad views** on/off,
  the Performance profile name (empty = the program's name) with **Quad views profile...**, Remove.
- Stored in `~/.config/pibridge/apps.toml`, one `[[app]]` table each:
  ```toml
  [[app]]
  id = "falcon-bms"            # from the name; also the log file name
  name = "Falcon BMS"
  exec = "~/tools/bms-log.sh"
  args = ["-xr"]
  workdir = "/games/BMS"
  env = ["DXVK_HUD=fps"]
  icon = "~/Pictures/bms.png"
  quad_views = false
  qv_profile = "Falcon BMS"
  last_played = 1792285852     # written on every start
  ```
  A TOML subset (strings, booleans, integers, string arrays also over several lines, comments). Every line it does
  not know (other keys, comments, other tables, `[app.x]` sub-tables, other value forms) is kept and written back as
  it was. Written atomically (QSaveFile), re-read before every change, so hand edits survive.
- While "Monado directly" (Development page) runs, own apps start on its Monado instead
  (`core/apps_runtime.*`): `XR_RUNTIME_JSON=<openxr_monado-dev.json>`, Play starts tracking when it is off, the tile
  shows STARTING TRACKING, SteamVR is never started; Steam games are off with the reason ([TRACKING.md](TRACKING.md)).
- **Play**: when SteamVR is not fully ready (below), `steam steam://rungameid/250820` first (not when vrserver already
  runs), then the tile shows STARTING STEAMVR with what it waits for, and the program starts once SteamVR is ready
  (60 s at most; else ERROR with the reason: "SteamVR does not see the headset", "the SteamVR compositor did not
  start", "SteamVR is not installed", ...). The program: its arguments, working folder and environment
  (+ `DISABLE_XR_APILAYER_ARTSCOUT_QUAD_VIEWS_VULKAN=1` when quad views are off), stdin `/dev/null`, its own session
  (`setsid`), stdout+stderr into `~/.local/state/pibridge/apps/<id>-<yyyyMMdd-HHmmss>Z.log` with the symlink
  `<id>-latest.log`. The app keeps the process (QProcess) for RUNNING / "Exited (code N)"; **Stop** sends SIGTERM
  to that PID only. Closing PiBridge leaves running programs running.
- **Quad views profile...**: the Performance page opens and selects the profile `exe:<name>` or `app:<name>`
  (any case) when it exists; otherwise add it there with **Add game...**.

#### "SteamVR fully ready" (`core/steamvr_ready.*`)
vrserver running is not enough. In order, every second from the app's state (the same background OpenVR
connection as the dashboard; no second connection):
1. vrserver runs and the background connection is up (the server takes clients);
2. the headset (device 0, class HMD) is connected with a valid pose and `TrackingResult_Running_OK`;
3. the compositor runs: `IVRApplications::GetApplicationProcessId("openvr.component.vrcompositor") != 0` (IVRApplications
   through the same `VR_GetGenericInterface`; reported as `SteamVrStatus::compositorPid`); without IVRApplications,
   a `vrcompositor` process in /proc;
4. the last `SteamVRSystemState_` transition in `~/.local/share/Steam/logs/vrmonitor.txt` (last 64 KB, read only)
   is not one of a start in progress (Startup, Connecting, NotReady, Restart). Ready and Standby pass; a missing log
   or the Shutdown/Off of an earlier run do not block (steps 1-3 already show what runs now);
5. all held for 1.5 s without a break.
`SteamVrReady::waitReady(timeout, callback)` ends with (true) or (false, the reason of the step that never held);
`ReadyTracker` (injected clock) and `evaluateReady()` are tested with fakes.

#### Fake mode
Three made-up Steam games (one with a generated portrait cover, one with a generated wide header, one without art,
written into the fake temp dir) and two made-up own apps, in memory only. Play only says "Fake mode: nothing is
started"; nothing is run or written. Screenshots: `docs/app/06-apps.png` (SteamVR ready, a game and an app running),
`docs/app/06b-apps-steamvr-off.png` (an app waiting for SteamVR, one with an error).

### Performance: Quadview foveated rendering (`pages/performance.*`)
- **Settings for**: "All games" (the global part of settings.conf) or a game profile (`[app:NAME]` / `[exe:NAME]`
  sections); **Add game...** (proposes the last game's name) and **Remove**.
- **Restore default**: the layer's defaults.
- **On/off switch**: the layer switch (renames the user manifest). Disabled when the layer is not installed in the
  home directory (the text next to the title says so).
- **Preset**: CUSTOM / PERFORMANCE / BALANCED / QUALITY / EXTREME (`core/presets.*`). A preset sets the two
  resolutions and the gaze area; CUSTOM is shown when the values match none.

  | Preset | edge (`peripheral_density`) | center (`focus_density`) | gaze area (`gaze_width` = `gaze_height`) |
  |---|---|---|---|
  | PERFORMANCE | 0.10 | 1.00 | 0.30 |
  | BALANCED | 0.20 | 1.00 | 0.35 |
  | QUALITY | 0.30 | 1.25 | 0.40 |
  | EXTREME | 0.40 | 1.50 | 0.45 |
- Sliders (with "-" / "+" and a value box): **Center resolution** (`focus_density`, 50-200 %), **Edge resolution**
  (`peripheral_density`, 5-150 %) - both with the yellow dot: the game must be restarted; **Follow the eyes / Fixed**
  (`tracking`); **Gaze area width/height**, **Fixed area width/height** (10-90 %); **Vertical offset**
  (`offset_y_deg`, -30..30 degrees); **Blend band** (`transition`, 0-50 %); **Edge opacity** (0-100 %); **Sharpen**
  (0-100 %); **Filter** bicubic/bilinear (`peripheral_filter`); **Smooth the gaze** (`gaze_filter`); **Debug
  overlay** (`debug_overlay`). Changes are saved 0.3 s after the last move; a running game picks them up.
- Right: the preview of one eye's field with the fixed and gaze areas to scale (drag the dot), the pixel budget, and
  the last game's status.

### Tracking: tracking mode and inside-out settings (`pages/tracking.*`, `core/inside_out.*`, `core/io_session.*`)
The tracking mode (**Lighthouse** / **Inside-out**, `tracking_mode`) and every setting of the inside-out session (the
Pimax driver in Monado: Basalt SLAM, camera auto exposure, optical controllers), each with one line of help, the
**RECOMMENDED** mark and the reason when it does not apply. All details, the sources and defaults of every knob are in
[TRACKING.md](TRACKING.md) (also the **How tracking works** link). (This page was called "Inside-out"; the
recorder that was called "Tracking" is the Development page.)
- Cards: **Tracking mode**, **Session: SteamVR + Monado** (display connector, refresh rate, what a start changes in
  SteamVR's files, **START TRACKING** / Stop), **Head tracking (SLAM)**, **Cameras**, **Controllers**, **Advanced**
  (extra environment `rec_env`, the programs and files, what a start sets, **Reset to recommended**).
- One environment builder (`buildIoEnv`) for START TRACKING, Record tracking and Monado directly; PiBridge's own Basalt
  config in `~/.config/pibridge/slam/`, generated at every start.
- START TRACKING (inside-out mode) = the broker, the tracker, then SteamVR (vrstartup.sh) with driver_monado.so of
  the chosen Monado build; for the session CustomHeadsetOpenVR is switched off in steamvr.vrsettings and the build's
  steamvr-monado registered in openvrpaths.vrpath (backups and a record first, put back exactly after Stop, SteamVR
  closing, PiBridge closing or at the next start after a crash). Stop closes SteamVR PID-exactly first. Games start
  through SteamVR; Play on the Apps page and Launch SteamVR on the Dashboard start tracking instead of a plain SteamVR.
  Details in [TRACKING.md](TRACKING.md).
- "Monado directly" (monado-service owns the headset, own apps on it, the home scene) moved to the Development page.
- Fake mode: fake processes, synthetic base files, SteamVR's files never touched. Screenshots `09-tracking.png` to
  `09g-dashboard-monado-directly.png`, `06c-apps-inside-out-steamvr-off.png`, `06d-apps-monado-running.png`.

### Room: room setup (`pages/room.*`, `core/roomsetup.*`, libpibridge-room)
The room setup: the boundary (SteamVR's "chaperone"), the play area and its centre, saved rooms. All details, the
controls in the headset and the file formats are in [ROOM-SETUP.md](ROOM-SETUP.md) (also the **How room setup works**
link). The editing is libpibridge-room's `Editor`, the same one the headset tool runs.
- **Room map (inside-out)** (inside-out mode only): **MAP THE ROOM** runs `pibridge-roomsetup --map` in the headset
  against the running session: the room through the headset cameras with the steps over it (look around until the ring
  is green, confirm the floor grid, keep or draw the boundary, look forward and pull the trigger), driving
  `pibridge-roommap setup fast --guide`; the floor, centre and forward are saved and recognised at every start. The card shows Room recognised / Searching / Room
  not found; refused in plain words without a running session ([ROOM-SETUP.md](ROOM-SETUP.md) "Room map").
- **OPEN ROOM SETUP IN HEADSET** starts `pibridge-roomsetup --universe <id>` (detached, through the Runner): the in-headset
  editor, a C++ port of ZoneTweak. Enabled only while SteamVR runs (it is an OpenXR app inside SteamVR) and the tool is
  found; otherwise the reason in plain words.
- **SteamVR universes**: every universe of `chaperone_info*.vrchap` in SteamVR's config folders (corners, area, play
  area, SteamVR's "time"); the one SteamVR tracks in is marked **STEAMVR USES IT** while SteamVR runs (the headset's
  `Prop_CurrentUniverseId_Uint64`, read by the SteamVR status worker). **Show** puts it on the floor plan.
- **Floor plan**: top-down, forward up, grid 0.5 m: the boundary with wall lengths, the warning distance as an orange
  dotted line, the play area (magenta) and the centre with its forward arrow, the saved outline dashed while edited.
  Drag a corner (1 cm steps), double-click a wall to add a corner, right-click a corner to delete it (never below 3).
  **NEW 2 X 2 M**, **FIT PLAY AREA**, **CENTER**, **TURN -15° / +15°** (the forward direction), **SQUARE TO WALLS**,
  **UNDO**.
- **APPLY TO STEAMVR** writes the boundary, play area and centre into the chaperone file (timestamped `.bak-*` copy
  first, atomic write, other universes and unknown fields kept). Enabled only while SteamVR is **not** running:
  SteamVR keeps the boundary in memory and writes it back when it closes. **REVERT** reloads the file, **RESTORE
  ORIGINAL** loads the oldest backup (apply to keep it). **SAVE AS ROOM...** keeps it as a named room.
- **Saved rooms** (`~/.config/pibridge/rooms/*.json`, shared with the headset tool): **Show** (on the floor plan, not
  applied), **Rename**, **Delete** (asks first; SteamVR's boundary is not changed).
- Fake mode: a made-up chaperone file (two universes) and two rooms in the fake temp dir; HOME too points there, so
  `~/.steam` is never read. Screenshots `docs/app/08-room.png` (SteamVR off: Apply enabled),
  `08b-room-steamvr-running.png` (the headset button enabled, the universe in use marked), and the headset tool's
  self-test views `08c-roomsetup-headset-room.png`, `08d-roomsetup-headset-menu.png` (software preview).

### Development: Monado build and Monado directly

- **Monado build**: the A/B selector (Auto / Rust-linked (live) / C++ (PiBridge)) on `rec_monado_service`; the build
  (`PIBRIDGE_PIMAX_ABI` marker), the required markers (a missing one blocks every start), the SteamVR driver dir,
  cap_sys_nice and **Fix permissions** (`pkexec setcap cap_sys_nice=ep`, only after a dialog). [DEVELOPMENT.md](DEVELOPMENT.md).
- **Monado directly (debug / A-B)**: START MONADO / Stop, the home scene switch; while it runs own apps start on it and
  the Dashboard reads Monado (controllers, headset card).

### Development: Record tracking (`pages/development.*`, `core/rec_*`)
Records a EuRoC dataset (tracking cameras + IMU) from a Monado 6DoF session for offline SLAM analysis, with guided
steps read out loud; all details in [DEVELOPMENT.md](DEVELOPMENT.md) (also the **How recordings work** link).
- **RECORD TRACKING** (mode **Guided steps** / **Free recording**): starts `pimax-cam broker`, waits for its socket,
  `pimax-ctrltrack live`, then `monado-service` with the Pimax driver's settings and `PIMAX_SLAM_RECORD=<folder>`
  (`core/rec_session.*`, QProcess through `QtRecProcesses`). The steps start when IMU data arrives. A failed display
  lease (seen in monado.log) is retried once after 4 s. **Stop** / the end of the steps: SIGTERM to exactly those PIDs,
  Monado first, SIGKILL after 10 s (Monado) / 3 s (the others).
- Disabled, with the reasons in plain words, while SteamVR or another Monado / pimax-cam / pimax-ctrltrack runs, without
  the headset or its cameras, without the display connector, or when a program or file is missing (named with its
  pibridge.conf key, `rec_*`). Re-checked every second while the page is visible (`/proc` comm, `/sys` reads only).
- **Steps** card: the planned steps while idle; during a guided recording the step in big letters, a countdown,
  "Next: ...", **Skip step** / **Stop**; each step is spoken (`spd-say`, else a tone through `pw-play`/`paplay`).
  The protocol is data (`core/rec_steps.*`, or a JSON file in `rec_steps=`). `steps.json` in the recording folder gives
  each step's window on the dataset's clock and its kind (prepare / still / tilt-hold / move).
- After the stop: the dataset check (`core/rec_check.*`, ported from `omarchyvr-slam check`) -> "OK: usable" or the
  problem; `check.json`.
- **Recordings**: date, length, size, mode, OK / PROBLEM with the first problem, **Open folder**, **Logs**, **Delete**
  (asks first). **Setup**: every program and file the recording uses, where it was found, and its key.
- Fake mode: fake processes and made-up recordings in the fake temp dir; nothing runs. Screenshots
  `docs/app/07-development.png`, `07b-development-steamvr-running.png`, `07c-development-recording.png`, `07d-development-done.png`.

### VR home (`pages/vrhome.*`, `core/vr_backgrounds.*`, `core/vr_panel.*`)
What pibridge-home shows in the headset ([home.md](home.md)). **In the headset**: the home's state (not running, in the
headset, behind an app) and what it shows; **This window in the headset** (`home_panel`): the PiBridge window itself on a
curved screen in the home, pointed at with a controller (trigger = click, stick = scroll, grip held = bring it in
front); **Passthrough** (the room through the four cameras; off goes back to the last picture); ROOM SETUP and
RECENTER (the Room page's own MAP THE ROOM / RECENTER IN HEADSET, with their checks); SHOW THE HOME IN STEAMVR (starts
pibridge-home as an OpenXR app while SteamVR runs). **Background** (`home_background`): thumbnails of Room (passthrough),
Dusk (built in), Dark grid and 16 Poly Haven CC0 pictures; one not downloaded yet is downloaded on the first tap (curl,
resumable) and turned into the home's picture and a thumbnail, then shown. The credit of the picture in use is under
the grid. Screenshots: `docs/app/11-vr-home.png`, `11b-vr-home-picture.png`.

### Logs (`pages/logs.*`)
- Services: `journalctl --user -u pibridge-keepalived -u pibridge-eyed -n 200 -f` (QProcess, started when the page is
  first opened, killed with the app).
- Quad views layer: the last 200 lines of `~/.local/state/qv-vulkan/qv-vulkan.log`, re-read every second while the
  page is visible.

### About (`pages/about.*`)
The name and tagline ("VR hub for Pimax headsets on Linux. Not affiliated with Pimax."), version, licence MIT, what
PiBridge is built on (omarchyvr, artscout-qv-vulkan-linux, CustomHeadsetOpenVR), the trademark note, and the paths of
the files the app uses.

### Status bar
SteamVR running / not, headset state, eye tracking state; **Help** opens this file (installed docs dir, else the
source tree's `docs/`).

## Code

| Dir | |
|---|---|
| `app/src/core/` | no widgets (library `pibridge_app_core`, tested): paths, conf writer, status readers + plain words, gaze shm reader, presets, runner, system (process check, tool lookup), actions, state source, SteamVR devices (`steamvr.*`, backend `steamvr_openvr.cpp`), SteamVR readiness (`steamvr_ready.*`), Reset SteamVR (`steamvr_reset.*`), Record tracking (`rec_setup.*` lookup and detection, `rec_steps.*` steps and steps.json, `rec_check.*` EuRoC check and recordings list, `rec_session.*` the programs, retry and stop), CustomHeadsetOpenVR detection and configurator lookup (`chovr.*`), the Apps page's logic (`apps_vdf.*` KeyValues reader, `apps_steam.*` library, `apps_own.*` apps.toml, `apps_launch.*` launcher, `apps_runtime.*` SteamVR or Monado), the tracking session (`inside_out.*` settings and env, `io_session.*` START TRACKING and the home scene), Monado's controllers (`monado_devices.*`, libmonado through dlopen), the Device page (`device_info.*`), the sound tests (`sound_test.*`) |
| `app/third_party/openvr/` | `openvr.h` + `LICENSE` of the OpenVR SDK 2.15.6 (BSD-3, Valve), header only |
| `app/src/qv/` | copied from artscout-qv-vulkan-linux (MIT), each file says where from: `settings.*`, `fovmath.*` (layer), `preview.*` (colours adapted), `layerinstall.*` (gui) |
| `app/src/ui/` | theme (style sheet, fonts, colours), widgets (Toggle, Chip, Card, Segmented, TickSlider, SliderRow, RestartDot, HeadsetArt, GazeDot), line icons |
| `app/src/pages/` | the pages (Dashboard, Display, Apps, Device, Performance, Tracking, Room, Recording, Development, Logs, About) and the Dashboard's sound tests (`sound_tests.*`) and record card (`record_card.*`) |
| `app/tests/test_app_core.cpp` | `pibridge_app_tests` (add_test): conf writer (keeps unknown lines and comments, every live duplicate, trailing comments; locale-independent numbers), status JSON parsing and staleness, chips/words, presets, the gaze shm reader against a temp file (the Rust test sample, torn snapshots), every button's command with a FakeRunner, the /proc check; SteamVR: vrpath parsing (temp HOME; missing/broken file), the library path, the real backend without SteamVR (= not installed, never the user's SteamVR), device list -> rows/chips/battery/station counts through the fake source, the 5 s back-off and no attempt without vrserver (injected clock), the worker thread |
| `app/tests/test_apps.cpp` | `pibridge_apps_tests`: the VDF reader (nesting, escapes, comments, tags, broken input, depth limit), the Steam library from a synthetic root in a temp dir (several libraries, tools and unfinished installs skipped, VR list, last played, the three cover layouts, old file forms, symlinked roots), running games from a fake /proc, apps.toml (round trip keeping unknown keys, comments, tables and sub-tables; ids; atomic save), argument quoting, last-played words, the launch spec (env, quad views off, log names) and log link, the readiness steps / settle / timeouts with an injected clock, the launcher with FakeRunner + FakeAppProcesses (start at once, start SteamVR and wait, timeout reason, cancel, stop by PID, exit codes, Steam games), and in "Monado directly" mode (the runtime file, XR_RUNTIME_JSON, waiting for the session, errors, Steam games refused) |
| `app/tests/test_tracking.cpp` | `pibridge_tracking_tests`: the inside-out settings, the env builder, PiBridge's Basalt config, START TRACKING and the home scene with FakeRecProcesses (TRACKING.md) |
| `app/tests/test_monado.cpp` | `pibridge_monado_tests`: the Dashboard's controllers from Monado (link, words, dlopen refusal, worker) with a fake source |

| `app/tests/test_steamvr_reset.cpp` | `pibridge_steamvr_reset_tests`: detection against a fake /proc tree (exe links, argv[0] when exe is unreadable, the odd comm, deleted exe, pressure-vessel wrapper, steamtours, vrwebhelper; Steam, steamwebhelper, reaper, a look-alike dir and the own PID excluded; `/proc/net/unix` + fd links for the socket holders, an outside holder, an unheld socket), running-time words and lines, the dialog words (stuck, healthy, nothing running with and without Steam), the close sequence with fake signals and an injected clock (TERM, KILL at 5 s only to survivors, gone/reused PIDs never signalled, EPERM, a process that survives KILL), the stat identity check, the Steam restart with the FakeRunner (shutdown, wait, start; 20 s timeout; Steam not running) |
| `app/tests/test_record.cpp` | `pibridge_record_tests`: EDID name/vendor and the connector over a fake /sys/class/drm, the camera and headset over fake v4l/usb trees, the lookup order (conf, PATH, fallbacks, the user's calibration first), the /proc check, the readiness words, the built-in steps, steps JSON parsing, the step runner's timing (late ticks, skip, stop), steps.json (integer ns, still windows, clock offset), the EuRoC check (the euroc.rs twins plus gaps, order, scale, overlap, parse errors), the recordings list / check.json / safe delete, the command lines and Monado's environment, the session with FakeRecProcesses and an injected clock (start order, data wait, the lease retry from a log sample and its second failure, the PID-exact TERM-then-KILL stop in reverse order, other failures, skip/stop/shutdown), speech or tone |
| `app/tests/test_room.cpp` | `pibridge_room_page_tests`: the Room page's gates (SteamVR on / off, tool missing, no file), the universe in use from the fake devices, the fake files and the universe / room rows (a point in every locale), the fake HOME, the launch of pibridge-roomsetup with the FakeRunner |
| `app/tests/test_device.cpp` | `pibridge_device_tests`: the Device page's rows live and without SteamVR (fallbacks, words, a point in every locale), modes / quad views / build words, CHOVR's IPD and the supersampling from temp files; the sound tests: node picking (monitors left out), the WAV header and the left/right tone, the meter scale, the speaker and microphone sequences with a scripted runner and a FakeCapture (commands, busy, meter, playback on the headset sink, temp files deleted, every error), never `set-default-*` |
| `app/tests/test_chovr.cpp` | `pibridge_chovr_tests`: CustomHeadsetOpenVR registered (external_drivers, drivers/ dir, broken file), headset enabled rules (missing `enable`, all off, customShader), the configurator lookup order with a temp PATH and home (executables only), the detached start with the FakeRunner |

Build and test (own build dir):
```
cmake -S . -B build-app -G Ninja && nice -n 19 cmake --build build-app -j8 --target pibridge-app pibridge_app_tests
nice -n 19 ctest --test-dir build-app -R pibridge_app
build-app/app/pibridge --screenshots /tmp/pibridge-shots
```

Screenshots of the fake state: `docs/app/*.png` (dashboard active and idle, apps with SteamVR ready and off, quad
views top and lower part, the Tracking page (modes, running, reasons), the Apps page in "Monado directly" mode,
development (recording ready / blocked / recording / done), room with SteamVR off and running, logs, about,
device (`10-device.png`, `10b-device-steamvr-off.png`)).

## Not done yet
- Inside-out: START TRACKING and PiBridge's Basalt config are tested with fakes and synthetic files only; the SteamVR
  (driver_monado) start is not implemented; the Monado build's markers are not checked (see TRACKING.md). Apps on
  Monado, the home scene with a game and the controllers through libmonado are tested with fakes only.
- Room setup: nothing of it has run in a headset or against a running SteamVR yet (see ROOM-SETUP.md, "Not
  verified live"). The Room page's edits are tested through the library; the page itself only through screenshots.
- Record video is tested with a fake tap and a fake ffmpeg only; no recording has been made from a live "Monado
  directly" session yet. The SteamVR mirror has never run against a real SteamVR (whether
  SteamVR's Linux compositor shares its mirror with OpenGL is not known yet).
- Record tracking is tested with fakes only; no recording has been made from the app yet (the display lease retry,
  the voice in the headset, Monado's SIGTERM within 10 s are to be checked live). The steps run at real speed in fake
  mode too.
- Apps: per-game settings for Steam games (their launch options belong to Steam); stopping a Steam game; the
  readiness and launches are tested with fakes only, the live check against a running SteamVR is still to do;
  Standby counts as ready.
- Reset SteamVR and the Steam restart are tested against a fake /proc and fakes only; the live check on a stuck
  SteamVR is still to do. The Reset dialog has no screenshot (the screenshot mode only renders the pages). The
  Dashboard runs `steam` through its own Runner (Actions keeps the app's Runner to itself).
- Runtime selection (only SteamVR). The SteamVR device status has been tested with fakes only; the live check
  against a running SteamVR is still to do.
- The eye tool is looked up as `pibridge-eye` only; omarchyvr's Rust `pimax-eye` is not used (its arguments differ).
  Calibrate and Recenter use `pibridge-eyecal`; omarchyvr's `pimax-eyecal` is only a fallback when it is missing.
- SIGHUP is sent with `systemctl --user kill --signal=HUP`; a daemon started by hand (not by its unit) does not get it.
- No autostart entry yet (the desktop entry `pibridge.desktop` and the icon are installed); no translations; the Help link opens the local doc, not a web page.
