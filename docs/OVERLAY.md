# pibridge-overlay: PiBridge buttons in the SteamVR dashboard bar (`tools/overlay/`)

**For users:** with SteamVR running, open the SteamVR dashboard with the system button. In its bottom bar, next to the
other overlay apps, there are three PiBridge buttons: **Room Setup**, **Quick Room Setup** and **Record Video**. Each
opens one big button — **START ROOM SETUP**, **START QUICK SETUP**, **START RECORDING** / **STOP RECORDING** — clickable
with the controller laser. The record button in the bar is a white ring while nothing records and a solid red dot (named
**Recording**) while a recording runs. It runs with the app by default; the switch is **Dashboard → Quick settings →
SteamVR dashboard** (right under Passthrough). The rest of this page is the design detail.

On the **Monado backend** this panel does not run: the Dashboard's **SteamVR dashboard** switch is off and locked
(`OverlayControl::setBlocked`), and a running panel is stopped when you switch to Monado. The saved choice is kept for
the SteamVR backend. PiBridge's own dashboard over games has the same buttons there ([home.md](home.md)).

## What SteamVR allows (and why it is three tabs)

SteamVR has **no API for a third-party app to put its own buttons into the dashboard bar or the quick menu** — not in
the OpenVR SDK header we build against (`IVROverlay_028`) nor in the installed runtime (`IVROverlay_029`); the bar and
the Quick Access menu are SteamVR's own web UI. What it does: **every dashboard overlay gets one button in that bar**
(its thumbnail as the icon, its name as the label; `renderExternalOverlayDashboardBarButton` in the dashboard UI).
So the overlay creates **three dashboard overlays**, one per action, and each one *is* a button in the SteamVR bar.
A click on a bar button can only *switch to* that overlay's page (SteamVR also switches to the last-used overlay by
itself when the dashboard opens), so the action is not fired on open — that would start a room setup or toggle a
recording every time the dashboard opens. The page is one big button: bar button → big button, no menu.

- **The order** in the bar is SteamVR's (alphabetical by name): Quick Room Setup, Record Video, Room Setup.
- **Inline tabs.** SteamVR shows only `dashboard/maxInlineTabs` external overlays inline (default **2**) and folds the
  rest into an "N more" button. At start the overlay raises it to 5 (our three + two other overlay apps) through
  `IVRSettings`, only if it is lower; it is never lowered. It is a normal SteamVR setting in `steamvr.vrsettings`.
- **The recording state in the bar**: the Record tab's thumbnail is drawn in code and re-uploaded when the state
  changes (the dashboard redraws on `VREvent_DashboardThumbChanged`), and its name becomes "Recording"
  (`SetOverlayName`; if a SteamVR build does not refresh names live, only the icon shows it).

## What it does

- **Pages and icons drawn in code** (`tools/overlay/src/overlay_core.cpp`): each page is a 520×220 RGBA card
  (`kPanelW`×`kPanelH`, about 0.6 m wide) with a header, one big button and a status line, in a built-in 5×7 font;
  each bar icon a 128² RGBA image (a room outline, the quick circle, the record dot). No app window, no `vrpanel.shm`.
- **Uploaded only on a change.** `SetOverlayRaw` when an overlay is created and again only when the hovered state or
  the real state changes what a page / an icon shows. There is no per-frame upload.
- **Laser input.** `SetOverlayInputMethod(Mouse)` + `SetOverlayMouseScale` = the page size. `VREvent_MouseMove` lights
  the button (re-draw on change), `VREvent_MouseButtonUp` (left) on it fires its action.
- **The action is a request file, not a launch.** On a click the overlay writes one line — `room-setup`,
  `quick-room-setup`, `record-start` or `record-stop` — atomically to `$XDG_RUNTIME_DIR/pibridge/overlay-request`. It
  never spawns `pibridge-roomsetup` or `pibridge-record` itself: the app reads the file and does exactly what its own
  buttons do (`RoomPage::launchSetup()` / `launchQuickSetup()` with the room name, ring, `--chaperone`,
  `--quick-direct`, its gate and its status messages; `RecordCard::recordFromVr()` for the recording). One owner, so
  the app always knows a setup runs and the arguments can never drift apart.
- **The state the buttons follow is real, not a flag of the overlay** (read twice a second, one `/proc` pass):
  - *recording* = the capture control file (`record-capture`, `pibridge_record_capture_control.h`) says `active 1`
    **and** a `pibridge-record` process is alive (a recorder that died with SteamVR can leave the file active);
  - *room setup running* = a `pibridge-roomsetup` process with `--map` in its arguments is alive, however it was
    started (overlay, Dashboard, Room page). headtrack's short `--vr-standing` commit and `--recenter` do not count.
- **No second room setup.** While a room setup runs both room buttons read **ROOM SETUP RUNNING** (grey, the room icons
  amber) and a click sends nothing. Right after a click a button reads **STARTING...** / **STOPPING...** and ignores
  clicks until the real state follows (or 8 s for a room setup, 10 s for a recording: the app refused it — its window
  says why). The app's Room page also refuses a second start on its own.

## Crash-free rules (a leaf)

- **Never `VR_ShutdownInternal`.** That is the crash class the room tool hit. VR is initialised at most once per
  process: `run()` exits the process on `VREvent_Quit` (SteamVR going down) and the app restarts it; the OS cleans up.
- Minimal OpenVR: `CreateDashboardOverlay` (×3), `SetOverlayWidthInMeters`, `SetOverlayInputMethod`,
  `SetOverlayMouseScale`, `SetOverlayRaw` (pages and thumbnails), `SetOverlayName`, `PollNextOverlayEvent`, and once
  `IVRSettings::GetInt32/SetInt32` for `dashboard/maxInlineTabs`. Plus the recenter ring overlay
  (`CreateOverlay`, `SetOverlayTransformTrackedDeviceRelative`, `SetOverlayTextureBounds`, `Show/HideOverlay`).
- If SteamVR is not up yet it waits and retries in-process (no init happened, nothing to clean up). If anything fails
  it just retries or exits; it never takes a VR session down.

## The app side

- **The switch:** Dashboard → Quick settings → "SteamVR dashboard" (right under Passthrough)
  (`app/src/core/overlay_control.h`, `OverlayControl`, owned by the main window). On by default; on starts
  `pibridge-overlay` as a leaf background process, off stops it (SIGTERM). Saved in `pibridge.conf` as
  `steamvr_dashboard` (on|off). A line under the switch shows OFF / WAITING (SteamVR not up) / RUNNING / NOT RUNNING +
  error. If the overlay exits while still wanted (e.g. SteamVR restarted), the app restarts it after ~3 s.
- **The request watcher:** `OverlayControl` polls `$XDG_RUNTIME_DIR/pibridge/overlay-request` (~400 ms), consumes a
  request, and emits `roomSetupRequested(bool quick)` / `recordRequested(bool start)`. `mainwindow.cpp` connects them
  to `RoomPage::launchSetup()` / `launchQuickSetup()` (the same entry points as the Dashboard's ROOM SETUP buttons) and
  `RecordCard::recordFromVr()`. A stale request file is cleared at app start, so nothing launches on its own.

## Colour format

`SetOverlayRaw` takes the buffer as **RGBA8, top row first**; the panel is drawn directly in that layout. If a
particular SteamVR/driver build shows the accent blue/red swapped, that is a BGRA/RGBA mismatch in the panel colours
only — swap the `r`/`b` fields in `overlay_core.cpp`; it is cosmetic and never a crash.

## Registering with SteamVR (auto-launch, optional — currently NOT used)

The toggle does not need registration: a running `pibridge-overlay` creates the dashboard buttons itself
(`CreateDashboardOverlay`). Auto-launch registration is kept in the binary for later but is **off by default** (the
main thread removed it to stop the earlier crashes). CMake still installs the manifest to
`<datadir>/pibridge/pibridge.vrmanifest` (from `tools/overlay/pibridge.vrmanifest.in`); to turn auto-launch on later,
with SteamVR running: `pibridge-overlay --register` (AddApplicationManifest + SetApplicationAutoLaunch);
`--unregister` reverses it; `--print-manifest` prints the JSON. Enabling both the toggle and auto-launch would have the
app and SteamVR each start an instance (the second `CreateDashboardOverlay` with the same `app_key` fails), so leave
auto-launch off while the app owns the process.

## Command line (the engine; the user never types it)

```
pibridge-overlay                 run the dashboard buttons (needs SteamVR)
pibridge-overlay --register      register + auto-launch (SteamVR must run; optional, off by default)
pibridge-overlay --unregister    remove the registration
pibridge-overlay --print-manifest
pibridge-overlay --selftest      checks the pure parts (pages, icons, hit test, requests, manifest); no runtime or GPU
pibridge-overlay --help
```
