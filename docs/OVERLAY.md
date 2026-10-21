# pibridge-overlay: a room-setup launcher in the SteamVR dashboard (`tools/overlay/`)

**For users:** this is an optional in-VR launcher. It is installed with the app; turn it on with the **SteamVR
dashboard** switch in the **Dashboard → Quick settings** (right under Passthrough). As soon as it is on and SteamVR is
running, open the SteamVR dashboard with the system button, pick the PiBridge tab, and press **ROOM SETUP** or **QUICK
ROOM SETUP** — the play-space setup starts straight from there, no terminal and no manual registration. The rest of this
page is the design detail. (New and not yet verified in a headset.)

When SteamVR is the runtime (the Crystal path), the only thing the user needs
from inside VR is to **start room setup**. `pibridge-overlay` is a small SteamVR dashboard tab that does exactly that:
open the dashboard with the system button, pick the PiBridge tab, and a little panel with two buttons —
**ROOM SETUP** and **QUICK ROOM SETUP** — is there, clickable with the controller laser.

It is deliberately minimal. An earlier version embedded the whole app window (a 2048² texture uploaded ~30 times a
second from `vrpanel.shm`, with a full mouse-event pipeline); that **crashed SteamVR** when the tab was opened. This
version draws a tiny static panel itself and does almost nothing per frame, so it is crash-free.

## What it does

- **A small panel drawn in code** (`tools/overlay/src/overlay_core.cpp`): a 512×320 RGBA card (`kPanelW`×`kPanelH`,
  about 0.6 m wide in the dashboard) with the two buttons, labelled with a built-in 5×7 font. No app window, no
  `vrpanel.shm`.
- **Uploaded once.** `SetOverlayRaw` is called when the overlay is created and again *only* when the hovered button
  changes (a highlight re-draw). There is no per-frame upload.
- **Laser input.** `SetOverlayInputMethod(Mouse)` + `SetOverlayMouseScale` = the panel size. `VREvent_MouseMove` picks
  the hovered button (re-draw on change), `VREvent_MouseButtonUp` (left) on a button fires its action. No scroll, no
  move/press forwarding.
- **The action is a request file, not a launch.** On a click the overlay writes one line —`room-setup` or
  `quick-room-setup`— atomically to `$XDG_RUNTIME_DIR/pibridge/overlay-request`. It never spawns `pibridge-roomsetup`
  itself. The app reads that file and launches the setup with its real machinery (the room name, ring, `--chaperone`,
  `--quick-direct` args).

## Crash-free rules (a leaf)

- **Never `VR_ShutdownInternal`.** That is the crash class the room tool hit. VR is initialised at most once per
  process: `run()` exits the process on `VREvent_Quit` (SteamVR going down) and the app restarts it; the OS cleans up.
- Minimal OpenVR: `CreateDashboardOverlay`, `SetOverlayFromFile` (tab icon), `SetOverlayWidthInMeters`,
  `SetOverlayInputMethod`, `SetOverlayMouseScale`, `SetOverlayRaw`, `PollNextOverlayEvent`. Nothing else.
- If SteamVR is not up yet it waits and retries in-process (no init happened, nothing to clean up). If anything fails
  it just retries or exits; it never takes a VR session down.

## The app side

- **The switch:** Dashboard → Quick settings → "SteamVR dashboard" (right under Passthrough)
  (`app/src/core/overlay_control.h`, `OverlayControl`, owned by the main window; the toggle lives in
  `app/src/pages/dashboard.cpp`). On starts `pibridge-overlay` as a leaf background process, off stops it (SIGTERM). The
  choice is saved in `pibridge.conf` as `steamvr_dashboard` (on|off), so a restart keeps it. A line under the switch
  shows OFF / WAITING (SteamVR not up) / RUNNING / NOT RUNNING + error. If the overlay exits unexpectedly while still
  wanted (e.g. SteamVR restarted), the app restarts it after ~3 s.
- **The request watcher:** `OverlayControl` polls `$XDG_RUNTIME_DIR/pibridge/overlay-request` (~400 ms), consumes a
  request, and emits `roomSetupRequested(bool quick)`. `mainwindow.cpp` connects that to
  `RoomPage::launchSetup()` / `RoomPage::launchQuickSetup()` — the same entry points as the Dashboard's ROOM SETUP
  buttons. A stale request file is cleared at app start, so nothing launches on its own.

## Colour format

`SetOverlayRaw` takes the buffer as **RGBA8, top row first**; the panel is drawn directly in that layout. If a
particular SteamVR/driver build shows the accent blue/red swapped, that is a BGRA/RGBA mismatch in the panel colours
only — swap the `r`/`b` fields in `overlay_core.cpp`; it is cosmetic and never a crash.

## Registering with SteamVR (auto-launch, optional — currently NOT used)

The toggle does not need registration: a running `pibridge-overlay` creates the dashboard tab itself
(`CreateDashboardOverlay`). Auto-launch registration is kept in the binary for later but is **off by default** (the
main thread removed it to stop the earlier crashes). CMake still installs the manifest to
`<datadir>/pibridge/pibridge.vrmanifest` (from `tools/overlay/pibridge.vrmanifest.in`); to turn auto-launch on later,
with SteamVR running: `pibridge-overlay --register` (AddApplicationManifest + SetApplicationAutoLaunch);
`--unregister` reverses it; `--print-manifest` prints the JSON. Enabling both the toggle and auto-launch would have the
app and SteamVR each start an instance (the second `CreateDashboardOverlay` with the same `app_key` fails), so leave
auto-launch off while the app owns the process.

## Command line (the engine; the user never types it)

```
pibridge-overlay                 run the dashboard launcher (needs SteamVR)
pibridge-overlay --register      register + auto-launch (SteamVR must run; optional, off by default)
pibridge-overlay --unregister    remove the registration
pibridge-overlay --print-manifest
pibridge-overlay --selftest      checks the pure parts (panel, hit test, requests, manifest); no runtime or GPU
pibridge-overlay --help
```
