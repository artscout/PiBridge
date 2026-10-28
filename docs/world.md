# PiBridge World

The Monado backend's home in the headset: the world around you, the dashboard, your apps. A Godot 4 project
(`world/`, GDScript only, no imported assets) that the Monado session runs instead of pibridge-home
(`vr_home=classic` in pibridge.conf brings pibridge-home back). Godot is MIT licensed: no fees, no account.

## Running

The app finds Godot (`rec_godot=` in pibridge.conf, else the newest `~/gamedev/godot/Godot_v4*_linux.x86_64`, else
`godot4` / `godot` on PATH) and the project (`<prefix>/share/pibridge/world`, else the source tree's `world/`) and runs
`godot --path <world>` with the session's `XR_RUNTIME_JSON` once Monado is up. Godot itself: download the official
Linux build (`Godot_v4.7.2-stable_linux.x86_64.zip` from github.com/godotengine/godot/releases, check it against
`SHA512-SUMS.txt`) and unpack it into `~/gamedev/godot/`; nothing is installed system-wide.

By hand (desktop view without a headset): `godot --path world --xr-mode off`. Self test (no headset, no runtime, temporary
files only): `godot --headless --xr-mode off --path world -- --selftest`. Always pass `--xr-mode off` without the Monado
session: with OpenXR on and SteamVR as the system runtime, Godot would start SteamVR.

## In the headset

- **The world**: a sky with a low sun, a ring of mountains, and terrain around you: flat where you stand (12 m), then
  rolling hills of moss, stone and dry grass (shader noise, no textures). A soft grid shows only near you, so the floor
  reads as a floor. You stand on a dark metal disc with fine rings and a glowing edge at the play circle.
- **Controllers**: the lasers start at the controller's aim point (the model's `openxr_aim` frame when the models are
  there, else the OpenXR aim pose); the trigger clicks on the panel, the thumbstick scrolls. The controllers are drawn
  with the headset's own render models when the kit has them (below), else as plain stand-ins.
- **Dashboard** (the menu button on either controller: show / hide in front of you; hold a grip 0.5 s: bring it in
  front of you; a recenter puts it back in front of you), opening on *Apps*:
  - *Apps*: tiles with **Run** and **Edit**: your apps (PiBridge's `~/.config/pibridge/apps.toml`, the same list as
    the app's Apps page) and the Steam games that use OpenXR (games with only the SteamVR API cannot run on Monado and are not listed). A Steam game starts
    through Steam, with the system's OpenXR runtime pointed at this Monado first (the app's Runtime switch shows it).
    *Add an app*: a name and the program, typed with the keyboard on the panel (Shift lit while on, for one capital;
    the keyboard closes with the page).
  - *Headset*: IPD (the lens motors at once), volume, refresh rate and lens profile (after restart).
  - *Tracking*: Lighthouse / inside-out, the SLAM engine, the camera rate (after restart); the recenter hint.
  - *Controllers*: the pointer angle (at once), when the controllers are shown, test vibration, the models on / off.
  - *Room*: the room setup in the headset (`pibridge-roomsetup --backend monado`: the camera map inside-out, the
    Lighthouse stage with the faceplate).
  - *World*: the background - the PiBridge world, dusk, or one of the 360-degree pictures the app's VR Home page downloads
    (`home_background` in pibridge.conf, shared with the app; a picture replaces the sky, the plain and the mountains).
  - *Recording*: Start / Stop and the video settings of `pibridge-record` (`~/.config/pibridge/record.toml`, the same
    file as the app's Recording page): one eye or both, 30 / 60 / 72 fps, quality, 1080p / 1440p / 4K, H.264 / H.265,
    sound and microphone; used from the next recording.
  - *Quad views*: the quad views layer's settings (`~/.config/qv-vulkan/settings.conf`; the layer re-reads them while a
    game runs, the densities at the next game start).
  - *Passthrough* (top): the room through the tracking cameras over the world (pibridge-home --passthrough-overlay, a
    Monado overlay).
- **Record** (top): a video of what you see (pibridge-record, through Monado's compositor tap); red while it records.
- **Recenter**: hold the Pimax button (pibridge-xroverlay shows the ring over any app), or Recenter on the Tracking /
  Room page. Forward becomes where you look and the centre where you stand; in games too, also seated ones that use
  the LOCAL space ([monado.md](monado.md)).
- **Passthrough**: the world's panel stays visible through a window in the passthrough (the world writes its pose to
  `$XDG_RUNTIME_DIR/pibridge/world-panel`); the menu button brings you back.

## Over a game

The menu button during a game opens PiBridge's dashboard over it (`pibridge-home --dashboard-overlay`, a Monado
overlay started with the session): the settings pages, and along the bottom Record video / Stop recording, Quick room
setup, Recenter and Close app ([home.md](home.md), "Over a game"); the game's inputs are blocked while it is open (libmonado), as
SteamVR's dashboard does. While PiBridge World is the main app, the menu button is the world's.

While a game runs, PiBridge World closes itself so the game gets its GPU memory and CPU back: about 2 s after Monado
moves the world into the background (SYNCHRONIZED: not shown, no input) it ends with exit code 75. Only the overlays
stay, the menu and the recenter ring. Once the game is gone and no app has been left for 3 s, the dashboard overlay
writes `$XDG_RUNTIME_DIR/pibridge/world-return` and the PiBridge app starts the world again. Only a code-75 exit
brings it back; a crash or a plain exit does not. `world_close_for_games = off` in `pibridge.conf` keeps the world
running in the background instead. While the world starts again, `pibridge-xroverlay` shows three pulsing dots in
front of you instead of a black screen ([xroverlay.md](xroverlay.md)).

## Controller models

From the Pimax client on Windows, copy the two folders

    C:\Program Files\Pimax\SteamVRSupport\drivers\aapvr\resources\rendermodels\crystal_controller_left
    C:\Program Files\Pimax\SteamVRSupport\drivers\aapvr\resources\rendermodels\crystal_controller_right

into `~/.local/share/pibridge/windata/steamvr/rendermodels/` (the Windows data kit, docs/WINBRIDGE.md; personal use,
never in this repository). With the Windows drive mounted at `/mnt/c`:

    K=~/.local/share/pibridge/windata/steamvr/rendermodels; mkdir -p "$K"
    cp -r "/mnt/c/Program Files/Pimax/SteamVRSupport/drivers/aapvr/resources/rendermodels/crystal_controller_"{left,right} "$K/"

The world reads `crystal_controller_<side>.json` (the parts and the `openxr_grip` frame) and the OBJ parts with their
textures at start, and puts the model on the OpenXR grip pose the way SteamVR does.

## Files

`world/scripts/`: `main.gd` (XR, the scene, Passthrough, the self test), `environment.gd`, `dashboard.gd` (pages and
every setting's receiver), `vr_panel.gd` (a SubViewport on a quad, its mouse driven by the lasers), `pointer.gd`,
`keyboard.gd`, `apps_model.gd`, `controller_model.gd`, `obj_loader.gd`, `keyconf.gd` (key=value files, comments and
sections kept), `paths.gd`.
