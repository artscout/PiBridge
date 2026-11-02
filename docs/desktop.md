# The desktop in the headset (pibridge-desktop, tools/desktop)

**For users:** the desktop view and its bar are made after Pimax Experience's (its look, places and sizes; no
Pimax code or assets are used). In PiBridge World a bar is always there under the dashboard: **Desktop**,
**Record**, **Room setup**, **Recenter**. **Desktop** puts your monitors in front of you in place of the dashboard, and
the bar (now under them) chooses **1**, **2** or **All**; Desktop again, or the menu button, brings the dashboard back.
Over a game the dashboard overlay (menu button) has the same: its Desktop icon, with **Dashboard** and **Close app**. The controller is the mouse: the **trigger** clicks (hold it to drag), the stick scrolls, the right click is **A / X**
in PiBridge World and a **short squeeze** of the grip over a game. Holding the grip brings the screens in front of you
again.

Where the Desktop button is:

| Backend | Where | How it shows |
|---|---|---|
| Monado, over a game | the dashboard over games (menu button), bottom bar, first icon | the monitors as OpenXR quad layers on an arc around you, the bar under them |
| Monado, in PiBridge World | the world's bar, always under the dashboard: **Desktop** | at the frame's places and sizes (the first monitor in the middle, the second on the right, turned to you), with the **Zoom** and **Distance** sliders in the bar; the world places them and its lasers are the mouse, the dashboard overlay draws the pictures as quad layers |
| SteamVR | the **Desktop** tab in SteamVR's dashboard bar (next to Room Setup, Quick Room Setup, Record Video) | the tab is the bar: **Desktop** puts your monitors in the room in front of you (they stay when the dashboard closes; a second press hides them), with **Zoom** and **Distance**; SteamVR's laser is the mouse |

**Once, on the desktop:** KDE has to allow it. On the app's Dashboard, **Quick settings → Desktop in VR → Allow**: KDE
asks whether PiBridge may share the screens and control the pointer; allow it. KDE keeps the answer, so it is never
asked in the headset (a question there would only appear on the monitor). **Ask again** repeats it (after a new
monitor setup, or when KDE forgot it). From a terminal: `pibridge-desktop allow`.

## How it works

```
KDE (xdg-desktop-portal-kde)                      pibridge-desktop run                         the overlay in the headset
 ScreenCast + RemoteDesktop portal  --PipeWire-->  each monitor's picture  --desktop.shm-->   pibridge-home --dashboard-overlay (Monado)
 (one session, restore token)                      the pointer as input   <--desktop.shm--    pibridge-overlay's Desktop tab (SteamVR)
```

- **pibridge-desktop** (`tools/desktop`) opens one portal session with both interfaces: RemoteDesktop (pointer and
  keyboard) and ScreenCast (the monitors, several, the cursor drawn into the pictures). KDE's answer is kept as the
  portal's *restore token* in `~/.local/state/pibridge/desktop-token` (a token is good once: each start writes the new
  one). Without a token it never asks: it says "not allowed yet" and waits for `pibridge-desktop allow`.
- The pictures come over PipeWire (`pw_capture.c`), shared memory only (no DMA-BUF), BGRx/BGRA/RGBx/RGBA, and go to
  `$XDG_RUNTIME_DIR/pibridge/desktop.shm` as BGRA (`common/pibridge_desktop.h`): one slot per monitor (up to 4, at most
  3840 x 2160 each), each under its own sequence lock, at most 120 a second (KWin sends one per change, at the
  monitor's refresh), and only for the monitors the overlay shows.
- The overlay writes its beat, the monitors it shows (`want`), the pointer (the monitor, x, y in its pixels, buttons)
  and the wheel. The service turns them into the portal's absolute pointer moves on that monitor's stream
  (`NotifyPointerMotionAbsolute`), buttons (`NotifyPointerButton`, BTN_LEFT/RIGHT/MIDDLE) and wheel steps
  (`NotifyPointerAxisDiscrete`); leaving the screens releases what is held (`input.cpp`).
- **Nothing runs while nobody looks.** The overlays start `pibridge-desktop run` when the desktop opens (one at a time:
  a lock file). It stops capturing 5 s after the overlay stopped looking and ends itself after 30 s.

### Monado: the dashboard overlay (`pibridge-home --dashboard-overlay`)

- The monitors are OpenXR **quad layers** of their own (`tools/xrkit/quad.*`): each picture is copied into its own
  swapchain when a new one comes, and Monado's compositor samples it straight onto the eyes, so the text is as sharp as
  the headset shows (no second resampling through an eye image).
- `tools/home/src/desk.*`: the monitors on an arc 1.6 m around where you stood, in their desktop order and size (a
  2560-pixel monitor is 1.3 m wide), each turned to face you, a little below the eyes; the ray's hit in that monitor's
  pixels; the pointer rules (a drag stays on the monitor it began on, clamped to its edge).
- The bar is the dashboard painter's `deskLayout` (`dash_view.h`, 2400 x 300) on a curved panel under the screens.
  The quick actions have icons drawn in code (`dash_paint.cpp`), also in the dashboard's own bottom bar.
- An app may also ask the overlay for the desktop by changing `$XDG_RUNTIME_DIR/pibridge/desktop-open` (the overlay
  opens it and blocks that app's inputs until it closes); PiBridge World shows the desktop itself instead (below).

### PiBridge World (`world/scripts/desktop.gd`, `bar.gd`)

- **The layout** (`layout.gd`): one frame at your eyes, turned to where you look, moved and scaled by the interface
  size (Medium: 1.55 m ahead, scale 0.75; Large, Small, Miniature on the World page, `world_ui_scale`). In it the
  dashboard (2.24 x 1.40), the bar (2.24 x 0.336, 1.25 below, tilted 30 degrees up) and the desktop: a monitor 2.19
  wide, the second at x 2.012, 0.727 nearer, turned 40 degrees to you (a third on the left). Zoom (0.5 to 5) scales the
  desktop, Distance (-1.5 to 1.5) moves it (`desktop_zoom`, `desktop_distance` in pibridge.conf). Holding a grip brings
  the frame to where you look.
- **The pictures are drawn by the dashboard overlay**, not by the world: a 2560 x 1440 picture per monitor up to 120
  times a second through Godot's main thread lowered the world's frame rate. The world writes the screens' places in
  the stage and its lasers on them to `$XDG_RUNTIME_DIR/pibridge/world-desktop` every frame
  (`screen <monitor> <pos> <quat> <w> <h>`, `ray <from> <to>`); the overlay (while closed) draws each as a quad layer
  from `desktop.shm` and the lasers over them (`desk::parseWorldDesk`).
- The world writes its side of `desktop.shm` (Godot has no shared memory: a file at the offsets `pibridge_desktop.h`
  checks with `static_assert`): the beat, `want`, the pointer and the wheel. Godot has no CLOCK_MONOTONIC, so the
  service takes a beat that *moves* as the sign of life. Reading it goes through a new handle each time: a buffered
  one returns old bytes after a seek inside what it has read.
- The screens (only their collision shapes) take the world's lasers as the dashboard panel does (`pointer_move`,
  `pointer_button`, `pointer_right` on A / X, `pointer_scroll`, `pointer_leave` when the laser goes). `draw_pictures`
  (the self test) lets the world read and show the pictures itself.
- The bar is always there, at its place under the dashboard (also while it is hidden or the desktop is shown).

### SteamVR: the Desktop tab (`pibridge-overlay`)

- A fourth dashboard overlay, `pibridge.desktop` ("Desktop"), whose page is the bar (the dashboard painter's
  `deskLayout`): **Desktop** shows the monitors and a second press hides them; **1** / **2** / **All**, **Zoom** and
  **Distance** (`-` / `+`, saved as `desktop_zoom` / `desktop_distance`, shared with PiBridge World) while they are
  shown; Record video, Quick room setup, Recenter.
- The monitors are overlays of their own in the room (`pibridge.desktop.monitor<n>`), placed where the head looked when
  Desktop was pressed, at the frame's places and sizes (`tools/home/src/frame_layout.*`, the same numbers as the
  world's `layout.gd`; the interface size `world_ui_scale`). They stay when the dashboard closes, until Desktop hides
  them, and take SteamVR's laser also outside the dashboard (`MakeOverlaysInteractiveIfVisible`; `VREvent_Mouse*`,
  `VREvent_ScrollDiscrete`).
- Each monitor's picture goes to SteamVR as a **Vulkan texture** (`desk_tab.cpp`; a per-frame `SetOverlayRaw` crashed
  SteamVR): copied into one of its two images (SteamVR may still read the last one) on the device SteamVR names with
  `GetOutputDevice`, with the extensions its compositor asks for. The bar is a Vulkan texture too.
- **Recenter** writes `recenter-request`, which `pibridge-headtrack` takes in the inside-out mode (as the Pimax
  button's long press); in the Lighthouse mode SteamVR's own recenter is the one.

## Commands

```
pibridge-desktop allow [--timeout S]   ask KDE once, on the desktop, and keep its answer
pibridge-desktop status                the permission, the service, the monitors
pibridge-desktop forget                drop the permission (KDE asks again at the next allow)
pibridge-desktop run [--idle-exit S] [--fps N]   the service (the overlays start it)
```

## Needs

- KDE Plasma 6 on Wayland with `xdg-desktop-portal-kde` (ScreenCast v4+, RemoteDesktop v2 for the kept permission).
  GNOME's portal has the same interfaces and should work the same; not tried.
- PipeWire (`libpipewire-0.3`), Qt 6 DBus (building: `libpipewire-0.3-dev`; Qt 6 DBus comes with `qt6-base-dev`).

## Not yet

- Typing: the portal session has the keyboard, but nothing sends keys yet (SteamVR's keyboard and PiBridge World's
  keyboard are the next step).
- Tried (2026-11-02, KDE Plasma 6.6, two 2560 x 1440 monitors): KDE's permission once and the restore token after it,
  both monitors' pictures, and in the headset on the Monado backend PiBridge World's desktop at the frame's places with the
  overlay drawing the pictures and the lasers as the mouse. Not tried yet: the SteamVR Desktop tab, the desktop in
  the dashboard over a game.

## Tests

`pibridge_desktop_tests` (the shared file, the pointer -> input events), `pibridge_home_dash_tests` (the bar, the
Desktop box, the screens on the arc, the ray, the pointer), `pibridge_overlay_test` (the SteamVR page and the laser
mouse).
