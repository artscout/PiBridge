# pibridge-home: the VR home in the headset (`tools/home/`)

`pibridge-home` is a small OpenXR + Vulkan program with three jobs in the headset:

- **The dashboard over games** (`--dashboard-overlay`, the Monado backend; section "Over a game" below): PiBridge's
  dashboard as a Monado overlay, opened with the menu button while a game runs.
- **The passthrough window** (`--passthrough-overlay`): PiBridge World's passthrough, the room through the cameras
  with a hole where the world's panel is.
- **The classic VR home** (`vr_home=classic`, or when Godot is missing): the first home before PiBridge World
  ([world.md](world.md)), after omarchyvr's docs/VR-HOME.md and docs/BACKGROUNDS.md. The rest of this page up to "Over a
  game" describes it.

The classic home shows:

- **The PiBridge window** on a curved screen 1.3 m in front of you (a piece of a cylinder around where you stood, 1.5 m
  wide, its height from the window's aspect). It is the app's real main window, so the home's menu is the app: the VR
  home page (room setup, recenter, passthrough on/off, the backgrounds) and every other page. Point a controller at
  it: the ray and a cross show where; the **trigger** is the left mouse button (press above 0.6, release below 0.4; a
  drag keeps going past the edge), the **stick** scrolls (up to 8 notches a second), the **grip button held 0.5 s**
  brings the window in front of you again. The hand whose trigger moved last points.
- **The background** (`home_background` in pibridge.conf, chosen on the app's VR home page): the room through the
  headset's four cameras (`passthrough`, the default; ROOM-SETUP.md "The passthrough"; the saved room's walls and
  ceiling are its surface), a 360-degree picture, the built-in **dusk** sky (drawn in code, always there), or the dark
  home (`none`: floor grid and horizon).
- The active saved room's boundary (libpibridge-room, [ROOM-SETUP.md](ROOM-SETUP.md)) and the controllers.
- While the app is not there (or `home_panel=off`), the old text panel "PIBRIDGE - TRACKING ON / START AN APP FROM
  PIBRIDGE." instead of the window.

Every part is a leaf: no camera picture, no picture file, no GPU pass, no app: the home falls back (dusk sky, the dark
grid, the text panel), says why in its status file (`background_note`, `window`) and keeps running.

SteamVR has its own home and dashboard; SteamVR Home is off for this user, so the app's VR home page has **SHOW THE
HOME IN STEAMVR**: it starts pibridge-home as a normal OpenXR app of SteamVR's (the window, the background and the
passthrough; a game started later takes the headset).

```
pibridge-home [--rooms DIR] [--res-div N] [--camera SPEC]            run (the VR home page starts it)
pibridge-home --selftest [--snapshot DIR [--panel-ppm FILE]]         checks without a runtime or GPU (and pictures)
pibridge-home --dashboard-overlay                                    the dashboard over games (the session starts it)
pibridge-home --passthrough-overlay                                  PiBridge World's passthrough (the world starts it)
```

## The PiBridge window in the headset

The app and pibridge-home share one file, `$XDG_RUNTIME_DIR/pibridge/vrpanel.shm` (`common/pibridge_vrpanel.h`, tmpfs, 16 MB
at most): a header and one BGRA picture of up to 2048 x 2048.

| Side | Writes | Reads |
|---|---|---|
| app (`app/src/core/vr_panel.*`) | `appBeatNs` at every check; while the home's beat is fresh, its main window rendered offscreen (`QWidget::render` into an image, scaled into 2048 px when bigger) under a sequence lock: 30 pictures a second while pointed at in the last 2 s, else 10 | the pointer (x, y in picture pixels, left button) and the wheel's running total; turned into mouse move / press / release and wheel events on the widget under the point (a press keeps its widget until the release) |
| pibridge-home (`tools/home/src/panel.*`, `xrhome.cpp`) | `homeBeatNs` every frame it shows the window; the pointer and the wheel | the newest whole picture (a torn one is skipped), uploaded as an sRGB texture (xrkit `TexPass`, one copy) |

Without pibridge-home the app only checks the file twice a second and renders nothing; without the app pibridge-home shows
the text panel. Neither ever waits for the other. Not shown in the headset: the app's popup windows (a drop-down
list's menu, dialogs): they are separate windows.

### Planned: the same window as a SteamVR dashboard tab (not built)

Under SteamVR the natural place is a "PiBridge" tab in SteamVR's dashboard (IVROverlay `CreateDashboardOverlay`, the
picture with `SetOverlayRaw` / a texture, the dashboard's laser mouse back through `PollNextOverlayEvent`
(`VREvent_MouseMove / ButtonDown / ButtonUp / ScrollSmooth`) into the same injection code of `core/vr_panel.cpp`, only
while `IsDashboardVisible()`). Not built now because the app's OpenVR connection (`core/steamvr_openvr.cpp`) is a
`VRApplication_Background` client on its own worker thread and one process gets one OpenVR context: the overlay needs
an `VRApplication_Overlay` context, so it belongs either in that worker (changing its type for everyone) or in a small
helper process fed by the same `vrpanel.shm` (the cleaner way: a leaf that can crash alone). Until then SHOW THE HOME
IN STEAMVR shows the window as an OpenXR app.

## Backgrounds

| Choice | What |
|---|---|
| `passthrough` (default) | the room through the four cameras; the camera feed runs only while this is the background |
| `dusk` | built in: a dusk sky with stars over a dark plain (`sky.cpp dusk()`) |
| `none` | the dark home: floor grid and horizon |
| a picture id | one of 16 Poly Haven pictures (`tools/home/src/backgrounds.cpp`, all CC0: flowers, ocean, sunset, mountains, forest, lake, desert, night, city; from omarchyvr's `rust/omarchyvr-backgrounds/sources.toml`), drawn as an equirect sky behind everything (`shaders/sky.frag`, one full-screen pass, the picture as a 4096 x 2048 sRGB texture) |

The pictures are not in git (CC0 would allow bundling them, but the 8K panoramas are 10-30 MB each): the app
downloads one the first time it is picked, into `~/.local/share/pibridge/backgrounds` (`app/src/core/vr_backgrounds.*`):
Poly Haven's API names the tonemapped 8K JPG (`api.polyhaven.com/files/<id>`, else its usual URL), curl fetches it
(resumable, 80 MB cap, a named user agent), then a worker thread turns it into `<id>.ppm` (4096 x 2048, what
pibridge-home reads; a picture that is not 2:1 is refused) and `thumbs/<id>.png`. pibridge-home re-reads pibridge.conf once a
second and loads a new picture on a thread (the dusk sky meanwhile). The Wikimedia pictures of omarchyvr's list need
a credit (CC BY / BY-SA) and are left out; the credit of the picture in use is shown on the VR home page anyway.

## How it yields to games

The rules were written for a runtime that shows the client whose session began last and falls back to another running
session when that one ends (Monado's rule, omarchyvr `forks/monado` `ipc_server_process.c`); under SteamVR a game
started later takes the headset the same way. So:

- pibridge-home calls `xrBeginSession` once, at its first READY, right after it starts.
- While another app is shown the home is SYNCHRONIZED: it keeps the frame loop going (`xrWaitFrame` /
  `xrBeginFrame` / `xrEndFrame` for every frame the runtime asks) but draws nothing and submits no layer.
- When the game ends the runtime may show the home again (VISIBLE / FOCUSED); the session is the same one.
- It never begins a second time and never makes a new session: a STOPPING is answered with `xrEndSession` and the
  program leaves. EXITING, LOSS_PENDING and an instance loss: it leaves at once. SIGTERM / SIGINT:
  `xrRequestExitSession`, then the same; after 3 s without an answer it leaves anyway.

The rules are `src/loop.h` (pure, tested); the OpenXR loop is `src/xrhome.cpp`.

## Light on the GPU

About 1 500 lines and a few hundred small triangles, no depth buffer, no MSAA; one full-screen pass for the background
(the passthrough or the sky), the window's 64 textured triangles; the eye size is capped at 2048 px; nothing is drawn
while another app is shown. Uploads: the window picture when the app wrote a new one (about 5.5 MB for 1500 x 920, up
to 30 a second while pointed at), a camera frame at 25 Hz (1.2 MB), a background picture once. Every frame the runtime
asks for is answered (no lowered frame rate). Space: STAGE (floor at y = 0), LOCAL when the runtime has no stage
(the floor then assumed 1.6 m below the start). The panel is placed once, 1.8 m in front of the first head pose, and
stays where it is.

The saved room is drawn in stage numbers through its standing pose (the frame mode of the rooms folder, as the room
setup does). With inside-out tracking the stage is anchored to the room only with a saved room map (TRACKING.md, "The
room map at the start"); without one the boundary is centred on the play centre but can be turned or shifted against
the real walls.


## Status file

`$XDG_RUNTIME_DIR/pibridge/home.json` (common/pibridge_status.h), rewritten at every change and every 2 s: `state`
("starting" before the session began, "running" begun and another app shown, "visible", "focused", "ending"),
`visible`, `focused`, `runtime`, `space` ("stage" | "local"), `room`, `error` (plain words), `pid`, `background`
(passthrough | picture | dusk | grid: what is shown), `background_note` (why it fell back, plain words), `window`
(linked | waiting for the app | off | no GPU pass). The VR home page reads it.

## Code

| File | |
|---|---|
| `src/scene.*` | the scene lists (grid, horizon, room, controllers, the text panel), the room in stage numbers, the panel placement |
| `src/panel.*` | the PiBridge window: the curved screen, the ray hit, the mesh, the pointer rules, the CPU drawing |
| `src/sky.*` | the equirect background on the CPU (snapshots), the PPM reader, the built-in dusk picture |
| `src/backgrounds.*` | the choice, the picture list, the paths, the pibridge.conf keys (also the app's: `pibridge_home_backgrounds`) |
| `src/plan.*` | what is shown from the settings and what is there, with the fallbacks |
| `common/pibridge_vrpanel.h` | the shared file with the app |
| `src/loop.*` | the session rules and the status fields |
| `src/xrhome.*` | the OpenXR side: grip and aim poses (Touch, Index, simple controller and Vive bindings), the frame loop |
| `src/selftest.*`, `src/options.*`, `src/main.cpp` | `--selftest`, the command line |
| `tools/xrkit/` | the OpenXR + Vulkan plumbing shared with pibridge-roomsetup (session, swapchains, the scene pipelines, `renderStereo`; `TexPass`: the sky and the window; the passthrough) |

Built with OpenXR when `pibridge_xrkit` is (the OpenXR loader, Vulkan and glslangValidator are there); otherwise only
`--selftest` works. Installed next to `pibridge` (`bin/pibridge-home`).

## Tests

`tools/home/tests/test_home.cpp` (`pibridge_home_tests`): the command line, the grid and horizon in STAGE and LOCAL, the
saved room in stage numbers, the text panel, the controller markers, the session rules, the status fields.
`pibridge-home --selftest` (`pibridge_home_selftest_binary`) runs those checks and: the background choice and settings, what
is shown with every fallback (`plan.h`), the dusk sky and the equirect mapping, the window's placement, hit, corner
mapping, mesh, the pointer (hover, press, drag, release, the wheel), the shared file (a picture across whole, too big
refused, the beat fresh then stale). `--snapshot DIR` writes `home-1-view.ppm` (the dark home),
`home-2-dusk-window.ppm` (dusk sky + window + ray), `home-3-passthrough-window.ppm` (the synthetic room through all
four cameras on the room prism + window), `home-4..6` (looking up: the camera map, two vs four cameras); with
`--panel-ppm FILE` the window is that picture (a screenshot of the app) instead of a test card. Pictures:
`docs/app/home-*.png`. The app's side: `pibridge_vrhome_tests` (the picture size, the pointer back into the window, Poly
Haven's answer, the conversion and the thumbnail, the fake library, the shared file).

## Not verified live

Nothing of pibridge-home has run against a real runtime (agents never touch a running session). To check in the headset:
the session, the Vulkan drawing (the sky, the window and the passthrough passes), the bindings of the Pimax
controllers (trigger, stick, grip), that a game started after it takes the headset and the home comes back after it, the
window's text at the 2048 px cap, the pointer's clicks and scrolling in the real app, the passthrough's four cameras
(orientation, seams, gains), and the background download from Poly Haven.

## The dashboard (`tools/home/src/dash_*`)

Under Monado nothing else in the headset shows settings, so the home's curved panel starts with PiBridge's own
dashboard (1.9 m wide, a picture of 2400 x 1200 painted with QPainter, only when it changes): a top bar (the runtime,
the room, the clock), the pages on the left, the page's rows in the middle, and a bottom bar with what the last click
did, **PiBridge window** (the app's window as before, when the app is linked) and **Hide**. Point and pull the
trigger to click (down and up on the same box). The **menu button** (Touch: the left menu and the right menu, which
Touch names `system/click`; Valve Index: the left system button, the right one is PiBridge's recenter and never
reaches the apps; simple / Vive: menu), a short press, hides the panel and brings the dashboard back in front of you (a long press stays the driver's
recenter); the grip held 0.5 s brings it in front too. A controller is drawn and points only while the runtime
tracks its position, and 1 s after it loses it (`HandShown`, scene.h): one switched off, a Pimax controller not seen
by the headset yet (its rest pose) or one lost longer is hidden. Without Qt or without the panel pass the home behaves as
before (the window or the text panel).

Every row has a real receiver (`dash_settings.h`); its tag says when it takes effect:

| Page | Row | Receiver | When |
|---|---|---|---|
| Headset | IPD 58-72 mm, 0.5 mm steps | `$XDG_RUNTIME_DIR/pibridge/ipd-request` (lens motors: Monado's Pimax driver or keepalived) + `ipd_mm` (`OXR_DEBUG_IPD_MM`) | lenses now, picture after a restart |
| Headset | Volume, 5 % steps | the headset's PipeWire output (`pactl`, a thread of its own) | now |
| Headset | Refresh rate 72 / 90 Hz | `refresh_hz` (`PIMAX_FPS`) | after a restart |
| Tracking | Head tracking engine Pimax / Basalt | `io_slam_engine` | after a restart |
| Tracking | Optical controller tracking | `io_ctrl_track` | after a restart |
| Tracking | Camera rate 50 / 60 Hz | `io_cam_fps` | after a restart |
| Controllers | Controller angle -20..70 deg | `$XDG_RUNTIME_DIR/pimax-ctrl-angle` + `io_ctrl_angle` | now |
| Controllers | Controllers shown: always / when held / off | `$XDG_RUNTIME_DIR/pimax-ctrl-inhand-mode` (Monado only) | now, this session |
| Controllers | Test vibration left / right | `$XDG_RUNTIME_DIR/pimax-haptic-request` | now |
| Recording | Record video / Stop recording; Picture (one eye / both), Frame rate 30 / 60 / 72, Quality, Resolution 1080p / 1440p / 4K, Codec H.264 / H.265, Sound, Microphone | `pibridge-record start|stop --brief`; the settings into `record.toml` next to pibridge.conf, quoted as pibridge-record writes them | the next recording |
| Room | Room setup / Quick setup | `pibridge-roomsetup --map --room <io_room> [--eye-height M] [--quick-direct] --backend monado`, `PIBRIDGE_BACKEND=monado` (Monado only) | an OpenXR app of the same Monado: it takes the headset, the home comes back when it ends |

"After a restart" means the next start of the Tracking session in PiBridge; a changed one is marked in amber.
`--backend` is passed only when `pibridge-roomsetup --help` names it (the variable always). The status file says what
the panel shows (`panel_shows`: dashboard | window | text | nothing).

## Over a game (`--dashboard-overlay`)

The Monado session starts `pibridge-home --dashboard-overlay` next to PiBridge World. It is a Monado overlay session
(`XR_EXTX_overlay`, placement 8), hidden until the menu button is pressed, and only while a game is the main app:
while PiBridge World is in front, the menu button is the world's.

- **The panel**: the same dashboard pages as above, opened in front of you.
- **The bottom bar**: **Record video** / **Stop recording** (the recording at hand, highlighted while it records;
  the state is the recorder's own, checked once a second while the panel is open), **Quick room setup** (the Room page's quick setup; the dashboard closes, the setup takes the
  headset), **Recenter** (as the Room page's: the driver turns forward to where you look, for every app, seated ones
  too; the dashboard closes) and **Close app** (the game in front ends: SIGTERM, then SIGKILL after 3 s if it is still
  there; its process id comes from libmonado's `mnd_root_get_client_pid`, PiBridge's fork, docs/monado.md). Close app is
  off when the libmonado has no process ids or no game is in front. There is no Hide and no PiBridge window here.
- **The game's inputs** are blocked while the panel is open (libmonado `mnd_root_set_client_io_blocks`), as SteamVR's
  dashboard does, and released when it closes or the overlay ends.

It also brings PiBridge World back after a game. PiBridge World closes itself while a game runs (exit code 75,
[world.md](world.md)). When the game is gone and no app has been in front for 3 s, the overlay:

- writes `$XDG_RUNTIME_DIR/pibridge/world-return`; the PiBridge app starts the world again;
- writes `$XDG_RUNTIME_DIR/pibridge/loading` from the moment the game is gone until an app is in front again (at most
  60 s, refreshed while waiting): `pibridge-xroverlay` shows its loading dots meanwhile ([xroverlay.md](xroverlay.md)).
