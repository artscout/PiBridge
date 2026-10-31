# PiBridge World

The Monado backend's home in the headset: the world around you, the dashboard, your apps. A Godot 4 project
(`world/`, GDScript only, no imported assets) that the Monado session runs instead of pibridge-home
(`vr_home=classic` in pibridge.conf brings pibridge-home back). Godot is MIT licensed: no fees, no account.

## Running

The app runs the exported world when there is one, `<prefix>/lib/pibridge/pibridge-world` (what the package ships:
Godot's Linux release template with this project embedded, made by `-DPIBRIDGE_EXPORT_WORLD=ON -DPIBRIDGE_GODOT=<godot>`
with `export_presets.cfg`; it needs that Godot version's export templates, docs/PACKAGING.md). It takes the same
arguments as Godot running the project (`pibridge-world --headless --xr-mode off -- --selftest`). `rec_godot=` in
pibridge.conf skips it, for working on the world.

Without the exported world, the app finds Godot (`rec_godot=` in pibridge.conf, else the newest `Godot_v4*_linux.x86_64` in
`~/.local/share/pibridge/godot/`, then in `~/gamedev/godot/`, else `godot4` / `godot` on PATH) and the project
(`<prefix>/share/pibridge/world`, else the source tree's `world/`) and runs `godot --path <world>` with the session's
`XR_RUNTIME_JSON` once Monado is up. Godot itself: `pibridge-get-godot` downloads the official Linux build
(`Godot_v4.7.2-stable_linux.x86_64.zip` from github.com/godotengine/godot/releases), checks it against the SHA-512 of
the release's `SHA512-SUMS.txt` (kept in the script) and unpacks it into `~/.local/share/pibridge/godot/`; nothing is
installed system-wide.

By hand (desktop view without a headset): `godot --path world --xr-mode off`. Self test (no headset, no runtime, temporary
files only): `godot --headless --xr-mode off --path world -- --selftest`. Always pass `--xr-mode off` without the Monado
session: with OpenXR on and SteamVR as the system runtime, Godot would start SteamVR.

## In the headset

- **The world**: a sky with a low sun, a ring of mountains, and terrain around you: flat where you stand (12 m), then
  rolling hills of moss, stone and dry grass (shader noise, no textures). A soft grid shows only near you, so the floor
  reads as a floor. You stand on a dark metal disc with fine rings and a glowing edge at the play circle.
- **Controllers**: the lasers start at the controller's aim point (the model's aim frame when the models are there,
  else the OpenXR aim pose); the trigger clicks on the panel, the thumbstick scrolls. The controllers are drawn with
  their SteamVR render models when they are there (below), else as plain stand-ins: the Pimax Crystal controllers, or
  the Valve Index controllers in the Lighthouse mode (the model follows the controller's OpenXR interaction profile,
  `/interaction_profiles/valve/index_controller`, and changes when the other controllers are picked up). A
  controller is drawn, and points, only while the runtime tracks its position (OpenXR tracking confidence high), and
  for 1 s after it loses it, so a short occlusion does not blink: one switched off, a Pimax controller the headset has
  not seen yet (Monado holds it at a guessed rest pose for games) or one lost longer is hidden (`hand_shown` in
  `main.gd`).
- **Base stations** (the Lighthouse mode only, **Show base stations in VR** on the app's Tracking card,
  `show_base_stations = on` in pibridge.conf): drawn where they stand, as SteamVR does; a file an earlier session left
  (older than 5 s) is not shown. Monado's steamvr_lh writes
  their poses in the stage to `$XDG_RUNTIME_DIR/pibridge/monado/basestations` (`<serial> px py pz qx qy qz qw` a line,
  once a second; [monado.md](monado.md)); the world re-reads it and the setting once a second. The model is SteamVR's
  `lh_basestation_valve_gen2` (else `lh_basestation_vive`, else a plain box).
- **Dashboard** (the menu button: show / hide in front of you; hold a grip 0.5 s: bring it in front of you; a recenter
  puts it back in front of you), opening on *Apps*. The menu button: the left menu or the right system button of the
  Crystal controllers; on Valve Index controllers the **left** system button (the right one is the recenter button),
  or the left B:
  - *Apps*: tiles with **Run** and **Edit**: your apps (PiBridge's `~/.config/pibridge/apps.toml`, the same list as
    the app's Apps page) and the Steam games that use OpenXR (games with only the SteamVR API cannot run on Monado and are not listed). A Steam game starts
    through Steam, with the system's OpenXR runtime pointed at this Monado first (the app's VR backend choice sets it).
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
- **Recenter**: hold the Pimax button, or on Valve Index controllers the **right** system button, ~0.8 s
  (pibridge-xroverlay shows the ring over any app), or Recenter on the Tracking / Room page. With Lighthouse the
  recenter moves the Index controllers, the trackers and the base stations with the head. Forward becomes where you look and the centre where you stand; in games too, also seated ones that use
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

## Controller and base station models

All models go into one folder, `~/.local/share/pibridge/windata/steamvr/rendermodels/` (the Windows data kit,
docs/WINBRIDGE.md; personal use, never in this repository), one sub-folder per model as SteamVR keeps them:

| Model | Copy from | Used for |
|---|---|---|
| `crystal_controller_left`, `crystal_controller_right` | the Pimax client on Windows: `C:\Program Files\Pimax\SteamVRSupport\drivers\aapvr\resources\rendermodels\` | the Crystal controllers |
| `valve_controller_knu_1_0_left`, `valve_controller_knu_1_0_right` | SteamVR on Linux: `<SteamVR>/drivers/indexcontroller/resources/rendermodels/` | the Valve Index controllers (Lighthouse) |
| `lh_basestation_valve_gen2` (and `lh_basestation_vive` for 1.0 base stations) | SteamVR on Linux: `<SteamVR>/resources/rendermodels/` | the base stations (Lighthouse) |

`<SteamVR>` is usually `~/.local/share/Steam/steamapps/common/SteamVR`. With the Windows drive mounted at `/mnt/c`:

    K=~/.local/share/pibridge/windata/steamvr/rendermodels; mkdir -p "$K"
    cp -r "/mnt/c/Program Files/Pimax/SteamVRSupport/drivers/aapvr/resources/rendermodels/crystal_controller_"{left,right} "$K/"
    SV=~/.local/share/Steam/steamapps/common/SteamVR
    cp -r "$SV/drivers/indexcontroller/resources/rendermodels/valve_controller_knu_1_0_"{left,right} "$K/"
    cp -r "$SV/resources/rendermodels/lh_basestation_valve_gen2" "$SV/resources/rendermodels/lh_basestation_vive" "$K/"

The SteamVR ones are also found in the SteamVR install itself (`~/.local/share/Steam` or `~/.steam/steam`) when they
are not copied; the copies win, and keep working when SteamVR moves or is updated.

The world reads `<model>/<model>.json` (the parts and the pose frames) and the OBJ parts with their textures, and puts
the model on the OpenXR grip pose the way SteamVR does: through the `openxr_grip` frame of the Crystal model, the
`grip` frame of the Index model (the same offset Monado gives the Index grip pose); the laser leaves from
`openxr_aim` / `tip`. The Index model's touch dot and its scroll-wheel variants are left out.

## Files

`world/scripts/`: `main.gd` (XR, the scene, Passthrough, the self test), `environment.gd`, `dashboard.gd` (pages and
every setting's receiver), `vr_panel.gd` (a SubViewport on a quad, its mouse driven by the lasers), `pointer.gd`,
`keyboard.gd`, `apps_model.gd`, `controller_model.gd`, `base_stations.gd`, `obj_loader.gd`, `keyconf.gd` (key=value files, comments and
sections kept), `paths.gd`.

The scripts reach each other with `const Name = preload("res://scripts/<file>.gd")`, never `class_name`: a global
class only exists in Godot's `.godot/` cache, which the editor writes and the install leaves out, so the installed
project (and a read-only one under `/usr`) runs without any import step.
