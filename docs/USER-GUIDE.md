# PiBridge: the user guide

**One app for your Pimax headset on Linux.** PiBridge wakes the headset, keeps it awake while you play, runs its eye
tracker, switches the sound to it, and gives you one window where everything about the headset lives: the display, the
controllers, the room boundary, foveated rendering and recording. SteamVR is the runtime that actually runs your
games; PiBridge is the hub around it.

You never need the command line. Every switch, button and page described here is in the app. The one-time setup (the
driver, the display patch, the SteamVR display driver) is in [INSTALL.md](INSTALL.md); after that, you open PiBridge,
start SteamVR, and play.

This is an unofficial community project. It is not made or supported by Pimax. It is alpha, tested on one PC with one
headset (a Pimax Crystal Super with the Lighthouse faceplate).

<p align="center"><img src="app/01-dashboard.png" alt="The PiBridge app" width="820"></p>

## The idea

Pimax Play on Windows is one program that does everything: it wakes the headset, shows its state, holds the settings,
pairs the controllers, sets up your room. PiBridge is that program for Linux. It talks to two small background helpers
that do the real work with the hardware:

- **`pibridge-keepalived`** wakes the headset, keeps it awake while SteamVR runs, switches the panel on at your chosen
  refresh rate, and makes the headset the default sound device. When SteamVR closes, it hands everything back and the
  headset goes to standby (the LED turns green).
- **`pibridge-eyed`** runs the eye tracker and feeds your gaze to SteamVR and to games.

You do not start or stop these by hand. They follow SteamVR on their own, and the app shows what they are doing. The
app never touches the hardware directly; it reads the helpers' state and changes the settings they share.

## A normal session

1. Open **PiBridge** from your application menu.
2. Press **Launch SteamVR** (or start a VR game from Steam). The headset wakes, the panel turns on, and the sound moves
   to the headset.
3. Put the headset on and play. Pair controllers, set up your room and change settings from the app whenever you like.
4. Close SteamVR. A few seconds later the sound returns to your speakers and the headset goes to standby.

The Dashboard shows the whole state at a glance: whether the screen is on, which controllers are connected and their
battery, the base stations, and whether the eye tracker is streaming.

## The pages

The left menu has one page per part of the headset. Here is what each one is for.

### Dashboard

Your home screen and the state of everything.

- **Launch SteamVR**, **Restart services**, **Restart headset** and **Reset SteamVR** (for a SteamVR that is stuck
  half-started) across the top.
- **START VR**, plus **ROOM SETUP** and **QUICK ROOM SETUP** to set up your play space in the headset.
- The **Headset** card: whether the screen is on, which refresh rate, and a **72 / 90 Hz** switch.
- **Controllers**: left and right with a connected/not-tracked chip, the reason when one is not tracked, and the
  battery. Other controllers and trackers are listed below.
- **Quick settings**: the **Passthrough** switch, the **SteamVR dashboard** switch (shows the PiBridge launcher tab
  inside the SteamVR dashboard, so you can start room setup from in VR), the **Eye tracking** switch, **Calibrate** and
  **Recenter** (the eye tracker's own calibration, the way Pimax Play does it), and **Import calibration from
  Windows...**.
- **Tracking**: your Lighthouse base stations and whether they are seen.
- **Sound settings**: the **Default audio output** switch and buttons to test the headset's speakers and microphone.
- **Record video**: record what the headset shows to a file (see Recording below).

### Display

How the picture looks, independent of how you track. These settings are applied by the headset display driver and take
effect the next time a VR app starts.

- **Refresh rate** (72 / 90 Hz) — the same switch as on the Dashboard.
- **Brightness**, **Contrast**, **Black level**, **Gamma**, **Color saturation**.
- **IPD horizontal offset** — shift the eyes sideways if the lens spacing does not match yours.

### Apps

Your Steam games and your own programs, shown as cover tiles like a game library.

- A search box and a **VR only** filter.
- **Add app...** to add your own program (its path, arguments, working folder, environment and icon), with a per-game
  foveated-rendering (quad views) choice.
- **PLAY** on a tile. If SteamVR is not running yet, PiBridge starts it first and waits until it is fully ready, then
  launches the game. Your own apps show **STOP** while running.

### Device

The headset's details, read live, like the device page in Pimax's own software. Serial numbers have a **COPY** button
and are only ever shown in the app — never logged or saved to a file.

- The headset model and serial, the **IPD** SteamVR renders with, and the **measured IPD** from the eye tracker.
- Each controller, tracker and base station with its serial and model.
- **Pair and test controllers**: pair or unpair a controller and test its vibration.
- **Rendering**: the render resolution per eye, the refresh rate, whether foveated rendering is on, and the tracking
  mode.

### Performance

Foveated rendering (quad views) and render quality.

<p align="center"><img src="app/03-performance-quadviews.png" alt="The Performance page" width="700"></p>

Foveated rendering draws the middle of your view at full detail and the edges at lower detail, which saves a lot of GPU
work. With eye tracking on, the sharp area follows your eyes; without it, it sits in the middle of the field.

- An **on/off** switch for the foveated-rendering layer, and a **preset** (Performance, Balanced, Quality, Extreme, or
  Custom).
- **Settings for** all games or a single game profile; **Add game...** and **Remove**.
- Sliders for the centre and edge resolution, the focus area size, whether it follows your eyes, the blend at the edge,
  sharpening and more. A small preview on the right shows the focus area to scale and the pixel budget.
- The render-quality settings for the headset display driver.

The foveated-rendering layer is installed alongside PiBridge; see [INSTALL.md](INSTALL.md).

### Tracking

How your head and controllers are tracked. For the Crystal Super with the Lighthouse faceplate, the default is
**Lighthouse** (base stations and Index controllers) and there is nothing to change.

<p align="center"><img src="app/09-tracking.png" alt="The Tracking page" width="700"></p>

**Inside-out** tracking (the headset's own cameras, no base stations) is experimental. If you switch to it, this page
has the start button and the camera, head and controller settings, each with a one-line explanation and a recommended
value. See [TRACKING.md](TRACKING.md) for the full detail.

### Room

Set up your play space: the boundary SteamVR shows when you come near a wall, the floor, and the centre and forward
direction games use. See the dedicated [ROOM-SETUP.md](ROOM-SETUP.md) for the step-by-step.

<p align="center"><img src="app/08-room.png" alt="The Room page" width="700"></p>

- **OPEN ROOM SETUP IN HEADSET** runs the setup in the headset, where you point with a controller. (You can also reach
  it from the Dashboard, or from the PiBridge tab in the SteamVR dashboard — see [OVERLAY.md](OVERLAY.md).)
- A top-down **floor plan** you can edit on the desktop: drag a corner, add or delete corners, fit and centre the play
  area, turn the forward direction.
- **Saved rooms** you can switch between (for example "Door open" and "Door closed").
- **APPLY TO STEAMVR** writes your changes into SteamVR's boundary. It is only available while SteamVR is **not**
  running, because SteamVR keeps the boundary in memory and would overwrite a desktop change; the in-headset tool does
  not have that limit.

### Recording

Record what you see in the headset to a video file.

- The same **Record video** card as on the Dashboard: pick one eye or both, the frame rate and the quality, and whether
  to include sound, then press **RECORD**.
- **Encoding**: the codec (H.264 plays everywhere; H.265 makes smaller files) and the output resolution (1080p, 2K or
  4K, capped by what the headset renders).
- **Save location**: where the files go (by default `~/Videos/pimax/`), with **Open folder** to find them.

### Development

Developer and debugging tools. You only need this page if you are helping test PiBridge.

- Choose which build of the Monado driver to use and fix its permissions.
- Run **Monado directly** (without SteamVR) for A/B testing.
- **Record tracking**: record the headset's cameras and motion for offline analysis, with guided steps spoken in the
  headset.

(The **SteamVR dashboard** launcher switch used to be here; it now lives in the Dashboard's **Quick settings** — see
[OVERLAY.md](OVERLAY.md).)

### Logs and About

- **Logs**: the live output of the two background services and of the foveated-rendering layer, in plain text, for when
  something needs looking into.
- **About**: the version, the licence, and what PiBridge is built on.

## Eye tracking

Eye tracking drives the gaze-following foveated rendering and is available to games through SteamVR. To use it you
upload your own eye-tracking profile once; the headset then applies your calibration itself, as it does under Windows.
The **Calibrate** and **Recenter** buttons on the Dashboard run the eye tracker's own calibration. The full setup, and
how to bring your profile over from Windows, is in [INSTALL.md](INSTALL.md).

Your eye profile and calibration files hold your personal eye data. They stay on your computer and are never shared.

## Getting help

- The **Logs** page shows what the services and the layer are doing.
- If a control is greyed out, its tooltip says why (for example, that it needs SteamVR running, or a tool that was not
  found).
- More detail on each part is in the other files under [docs/](.): [INSTALL.md](INSTALL.md),
  [ROOM-SETUP.md](ROOM-SETUP.md), [OVERLAY.md](OVERLAY.md), [TRACKING.md](TRACKING.md) and the per-part pages.

PiBridge is independent and not affiliated with Pimax. Pimax, Crystal Super and Dream Air are trademarks of their
owners, used here only to say which hardware PiBridge works with.
</content>
</invoke>
