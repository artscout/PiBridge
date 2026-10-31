# Room setup

PiBridge's room setup: the boundary SteamVR shows when you come near a wall (SteamVR calls it the chaperone), the play
area rectangle games use for room-scale, the play centre (where games put you and which way is forward), the warning
distance, and saved rooms ("Door open", "Door closed") to switch between.

## Set up your play space (for users)

Press **ROOM SETUP** on the Dashboard (VR must be running). You can also start it from the **Room** page
(**OPEN ROOM SETUP IN HEADSET**) or, without taking the headset off, from the PiBridge tab in the SteamVR dashboard
(see [OVERLAY.md](OVERLAY.md)). The setup opens **in the headset** and you point at the menu with the controller. If PiBridge doesn't recognise your room yet, it first asks you to **look around slowly** once
so it can find its bearings; if your room is already recognised it skips that.

**Quick setup** — a circle around you, fastest:
1. Pick **Standing** or **Sitting**. The floor is placed automatically (standing ≈ your height, sitting ≈ a chair).
2. Pick a zone size — **Small / Medium / Large** (a 1.0 / 1.5 / 2.0 m circle around you).
3. Press **Finish**. Done.

**Full setup** — draw your own boundary (standing):
1. **Set the floor.** A grid shows where PiBridge thinks your floor is; it reads "the floor is N below your face". Nudge
   it **up / down with the thumbstick** until the grid sits on your real floor, then press **Next**.
2. **Draw the edge.** Hold the **trigger** and sweep the controller along the floor to trace the edge of your space.
   The trigger only draws — nothing is saved until you choose to. Too small a loop is rejected with a hint; just draw
   again.
3. Press **Save**. Done.

The floor, the centre (where you stand), the forward direction (where you look) and the boundary go straight into
**SteamVR's play area / chaperone** — nothing is stored on the tracker's side. It comes back every time you start VR. You
can manage saved rooms and the floor plan later on the app's **Room** page.

---

It is a C++ port of **[ZoneTweak](https://github.com/killdano/ZoneTweak)** by killdano (MIT, Copyright (c) 2026
killdano; the licence is in [NOTICE](../NOTICE)), an in-headset editor for Pimax's safety zone on Windows. PiBridge keeps
its data model and its editing behaviour and points it at SteamVR's chaperone file on Linux. Every ported file says
which ZoneTweak file it comes from in its second line.

Three parts, one editor:

| Part | What |
|---|---|
| `libpibridge-room/` (`pibridge::room::`, target `pibridge_room`) | everything without graphics or runtime: floor geometry, the chaperone file, the zone settings, saved rooms, the editor (input in, edits out), the scene lists the headset draws, a software preview, the pose sources and the tracer |
| `tools/roomsetup/` (`pibridge-roomsetup`) | the OpenXR + Vulkan app that runs the editor in the headset; its OpenVR helper processes; `--selftest`, `--list`, `--use` |
| `tools/xrkit/` (`pibridge_xrkit`) | the OpenXR + Vulkan plumbing (session, swapchains, the two scene pipelines and their shaders) shared with `pibridge-home` ([home.md](home.md)); moved out of `tools/roomsetup/src/xrapp.cpp` unchanged |
| the app's **Room** page (`app/src/pages/room.*`, `app/src/core/roomsetup.*`) | the universes and saved rooms, a floor plan with a few edits, Apply / Revert / Restore original, and the **Open room setup in headset** button (see [app.md](app.md)) |

## Using it

In the app, **Room** in the sidebar:
- **OPEN ROOM SETUP IN HEADSET** (enabled while SteamVR runs) starts `pibridge-roomsetup` for the universe SteamVR
  tracks in. Put the headset on; B opens the menu; **SAVE** writes the file and asks SteamVR to reload it.
- The floor plan edits the same data on the desktop. **APPLY TO STEAMVR** is enabled only while SteamVR is not
  running: SteamVR keeps the boundary in memory and writes it back when it closes, so a desktop change made while it
  runs would be lost. The headset tool does not have that problem: it tells SteamVR to reload the file.

For agents and tests (the user never needs these):
```
pibridge-roomsetup                  the headset editor (SteamVR running, OpenXR runtime = SteamVR)
pibridge-roomsetup --universe ID    edit that universe
pibridge-roomsetup --list           universes of the chaperone file and the saved rooms
pibridge-roomsetup --use NAME       write a saved room without VR (refused while SteamVR runs)
pibridge-roomsetup --selftest [--snapshot DIR]   the editor on made-up files with scripted controllers; DIR/*.ppm views
```
`--config-dir DIR`, `--rooms DIR` and `--settings FILE` point it at other files (the tests use temporary ones).

## Controls in the headset (ZoneTweak's, either hand)

| Input | What it does |
|---|---|
| trigger near a corner post | grab it and move it (FLOOR mode: within 45 cm of the beam's floor point) |
| trigger near a wall | insert a corner there and move it |
| A (Index) / squeeze (Vive) / A or X (Touch) | delete the corner under the beam (never below 3) |
| thumbstick click (trackpad on Vive) | undo |
| B (Index) / menu (Vive) / B or Y (Touch) | open / close the menu |
| trackpad press (Index) / squeeze (Touch) | switch FLOOR beam (straight down) and LASER pointer (One Euro filtered) |
| the padlock next to the yellow play centre | unlock the play area: the centre post moves it, the curved arrows turn it, the edge and corner posts stretch it |

Only one hand is active at a time: the one that pulled its trigger last. In the menu, point the controller tip (or
the laser) into a button and pull the trigger; a tooltip explains the hot button.

Menu: **SAVE**, **REVERT** (to the last save), **UNDO**, **REDO**, **SQUARE UP** (straight walls at right angles),
**SIMPLIFY** (drops points within 8 cm of a straight line), **NEW OUTLINE** (walk to each corner, pull the trigger;
near the first corner it closes), **TRACE OUTLINE** (PiBridge: hold the trigger and carry the controller along the walls;
let go and the path becomes the boundary, Douglas-Peucker 8 cm), **SNAP** (dragged points pull square to their
neighbours), **SAVED SPACES** (tap one: it loads and is written; SAVE AS NEW with the panel keyboard; UPDATE; DELETE
twice), **ZONE SETTINGS** (warning distance -10/-2/+2/+10 cm or SteamVR's default, safety zone on/off, PROXIMITY TEST:
the warning line inside the walls, drag it to set the distance, the walls glow red where SteamVR would show them;
FLOOR FIX WITH SAVE), **FIX FLOOR** (rest a controller on the floor during the 3 s countdown; view only unless FLOOR
FIX WITH SAVE is on), **POINTER**, **UNITS**, **GHOST** (the saved outline in grey while editing), **REALIGN VIEW**
(hold the trigger and drag / twist the whole outline onto SteamVR's own; view only, stick click resets), **UNIVERSE**
(PiBridge: switch the edited universe), **ALIGN** (PiBridge: the frame mode, below), **RESTORE ORIGINAL** (the oldest
backup's boundary; save to keep), **SAVE AND EXIT**, **EXIT** (asks again with unsaved changes). The play area page
(menu while unlocked): **FIT AREA** (the largest rectangle around the centre inside the walls, 10 cm margin),
**CENTER IN ROOM** (the area centroid), **SQUARE TO ROOM** (edges along the dominant wall direction), **RESET**,
**LOCK PLAY AREA**. A recentre while it runs (OpenXR's reference space change) moves the outline with it, so it stays
on the room.

What it looks like: `docs/app/08c-roomsetup-headset-room.png` and `08d-roomsetup-headset-menu.png` (the self test's
software preview of the same scene lists the headset draws). The look is plain: translucent green wall sheets with
corner posts and their lengths, a floor grid, the magenta play area, the yellow centre and forward arrow, ZoneTweak's
5x7 pixel font. ZoneTweak's prison set dressing (laser walls, generators, robots, fire, lights, fog, sound) is not
ported.

## Files

### SteamVR's chaperone file

`chaperone_info*.vrchap` in SteamVR's config folders: the `config` entries of `~/.config/openvr/openvrpaths.vrpath`,
`~/.local/share/Steam/config`, `~/.steam/steam/config` (existing ones, each once). JSON:
```
{ "jsonid" : "chaperone_info", "version" : 5,
  "universes" : [ { "universeID" : "<string>",
                    "collision_bounds" : [ [ [x,0,z], [x,h,z], [x2,h,z2], [x2,0,z2] ], ... ],   one quad per wall
                    "play_area" : [ w, d ],
                    "standing" : { "translation" : [x, y, z], "yaw" : r },
                    "seated" :   { "translation" : [x, y, z], "yaw" : r },
                    "time" : "<text>" } ] }
```
The boundary is the first point of each quad (floor), the wall height the highest y (2.43 m when the quads say
nothing). Writing (ZoneTweak's rules): the edited universe is written into every file that holds it; the
boundary quads, `play_area` and the standing x, z, yaw are replaced (rounded to 1e-6), the standing y only changes by
a floor fix; everything else (other universes, `seated`, `time`, unknown keys, key order, untouched numbers' text)
stays as it was. Before the first write of a session each file is copied to `<file>.bak-YYYYMMDD-HHMMSS`; the write
goes to `<file>.tmp` and is renamed over the file. **RESTORE ORIGINAL** reads the oldest `.bak-*` that has the
universe. To undo a save by hand: close SteamVR, rename the `.bak-*` copy back.

**Frames.** The editor works in OpenXR's STAGE space (SteamVR's standing space). How the file's numbers relate to it
is the frame mode (ZoneTweak's): **A** = raw tracking coordinates, stage = the inverse standing pose applied (right
for Pimax's own universes, ZoneTweak's default); **B** = the same with the yaw negated; **OFF** = the numbers are
standing coordinates already. On this PC's SteamVR file (Lighthouse) the standing origin lies inside the boundary
only when it is read as OFF, so PiBridge uses **OFF** for SteamVR's universes and **A** for one Pimax marked
(`center_calied` / `floor_calied`); **ALIGN** in the menu switches (saved in `rooms/align.txt`). With OFF, moving the
play centre re-expresses the walls in the new standing frame so they stay where they are in the room (ZoneTweak kept
the numbers there, which would move the walls with the centre). The live check is still to do: with the headset on,
the green walls must sit on SteamVR's own grey boundary; if they do not, try ALIGN.

### Saved rooms

`$XDG_CONFIG_HOME/pibridge/rooms/<name>.json` (lower case letters and digits, spaces to `-`), `active.txt` (the room in
use), `align.txt` (the frame mode), `zone-alpha.txt` (the boundary opacity to put back after "zone off"). A room holds
a universe's file numbers, so applying it writes them back unchanged:
```
{ "name" : "Door open", "universe" : "1111111111", "height" : 2.43,
  "points" : [ [x, z], ... ], "frame" : "raw", "saved" : "2026-10-19T14:12:05+02:00", "created" : "...",
  "standing" : { "translation" : [x, y, z], "yaw" : r }, "play_area" : [ w, d ], "warning_distance" : 0.5 }
```
The last four keys are PiBridge's; ZoneTweak's space files (without them) load too.

### Zone settings

ZoneTweak edits Pimax's `profile.json`; under SteamVR the same settings are in `steamvr.vrsettings`, section
`collisionBounds`: the warning distance is `CollisionBoundsFadeDistance` (metres, 0.05..2, 1 cm steps; SteamVR's
default 0.7 when the key is missing), the zone on/off switch is the boundary's opacity `CollisionBoundsColorGammaA`
(off writes 0; on puts back the old value or removes the key). While SteamVR runs they go through IVRSettings (the
helper below), because SteamVR rewrites its settings file itself; otherwise into the file (backup first, atomic).

## How SteamVR is told

`pibridge-roomsetup` never loads a second VR client next to its OpenXR session: the OpenVR calls run in short child
processes of itself (`--vr-universe`, `--vr-reload`, `--vr-zone KEY=VALUE`), each an OpenVR *background* client
(libopenvr_api.so of the installed SteamVR via dlopen, the header in `app/third_party/openvr`). A background client
never starts SteamVR; without SteamVR it fails at once.
- the universe to edit: the headset's `Prop_CurrentUniverseId_Uint64` (the app's SteamVR status reads the same to mark
  the universe in use);
- after a save: `IVRChaperoneSetup::ReloadFromDisk(EChaperoneConfigFile_Live)`, so SteamVR shows the new boundary and
  does not write its old copy back when it closes;
- the zone settings: `IVRSettings::SetFloat / SetInt32 / RemoveKeyInSection`.

**Universe ids.** SteamVR keeps one boundary per tracking universe. Lighthouse universes get a large random id from
SteamVR's room setup. The inside-out headset (CustomHeadsetOpenVR's SLAM headset: pibridge-headtrack for the Crystal
Super / Dream Air, the original Crystal's own tracking; `k_UniverseId`) reports universe **30**, as Pimax's Windows
driver does. (Monado's SteamVR driver used **2**; a universe 2 left in the chaperone file from that time is not used.)

## The inside-out mode

With the tracking mode Inside-out (Tracking page) the Room page works on universe **30** (while SteamVR runs, the
universe SteamVR reports), and says at the top that each start begins a new tracking world which the room map (below)
puts back on the room. The universe list marks it INSIDE-OUT. `pibridge-roomsetup` works on the universe SteamVR
reports.
- The Lighthouse universes and their saved rooms are never changed by it: the page never falls back to a Lighthouse
  universe when universe 30 is missing (it says so and offers **CREATE INSIDE-OUT BOUNDARY**: universe 30 with a 2 x 2 m
  boundary added to the chaperone file, backup first, only while SteamVR is closed); **Save as room** refuses a name
  whose saved room belongs to another universe (in both modes: "Door open" of universe 1111111111 is not overwritten by
  an inside-out "Door open"; it suggests "Door open inside-out"); a saved room of a universe that is not in the file is
  not put on universe 30 (`core/roomsetup.h`: `roomTargetUniverse`, `saveRoomGate`, `showRoomGate`,
  `addInsideOutUniverse`; tests in `pibridge_room_page_tests`).
- Not handled yet: pibridge-roomsetup's own SAVED SPACES in the headset have no such check.

### Room map (inside-out)

Basalt starts its world where the headset is at every start, so without help the floor is at head height and forward
is wherever the headset looked. The room map fixes that: the C++ room-map tool `pibridge-roommap` (`tools/roommap`,
`docs/pibridge-roommap.md`, a port of omarchyvr's `omarchyvr-roommap`) maps the room once and recognises it at every
start. The app only starts it and reads its files (`app/src/core/roommap_link.*`).

- **MAP THE ROOM** (Room page, card "Room map (inside-out)", shown in the inside-out mode) runs the room mapping in
  the headset, `pibridge-roomsetup --map --room <room> --roommap <pibridge-roommap> --ring <head ring>`, against the running
  session (pibridge-headtrack's head ring + the camera broker): the room through the headset cameras with the steps over it
  ("Map the room in the headset" below), driving `pibridge-roommap setup fast --name <room> --guide DIR`. The tool saves the map, the floor and the
  room setup under `~/.local/share/pibridge/roommap/<room>/` (`PIBRIDGE_ROOMMAP_DIR` overrides; omarchyvr's
  `~/.local/share/omarchyvr/roommap` is used when only that exists) and writes the world offset through the writer
  lease (role setup): the floor at y = 0, the centre under you, forward -Z. Refused, in plain words, when the mode is
  not inside-out, the tool is missing, no session with a live head ring runs (START VR first), the head is
  not tracked yet, or a run is going on. The room name is `io_room` in pibridge.conf (default `home`); there is no
  chooser on the page yet.
- **At every start** (START VR in the inside-out mode): the tracker gets `PIMAX_ANCHOR=1` whenever `pibridge-roommap`
  is found (so MAP THE ROOM works in the session that runs; without an offset file nothing changes). When the room has
  a saved map, the session starts `pibridge-roommap auto --name <room> --wait 120` once SteamVR reports the headset,
  output in `<session>/logs/roommap.log`. It waits for the head ring, takes the lease (role auto),
  relocalises in 15 s windows for up to 120 s, applies floor + centre + forward and exits. The first offset of a
  session is placed at once; later ones glide (`PIMAX_ANCHOR_GLIDE`). Stop ends it first, by PID (SIGTERM, SIGKILL
  after 3 s). Without a saved room nothing is started. If it fails, VR keeps running at the start pose.
- **Status**: `pibridge-roommap` writes `$XDG_RUNTIME_DIR/omarchyvr-room.status` (state ok / searching / failed /
  skipped / off, room, pid = the session, text). The Room page's card and the Tracking page's session line show it as
  **Room recognised**, **Searching for the room** or **Room not found** with the tool's sentence; a file from another
  session (another pid, or written before this start) is not shown.
- Tests (`pibridge_tracking_tests`, `pibridge_steamvr_monado_tests`, fake processes only): auto starts once the runtime is
  up and only with a saved room and the tool, once per session, gets SIGTERM before SteamVR, SIGKILL when it
  ignores it, a start failure is shown in words; the status file and the words; the gate; the room name and dir.
  Not run live yet.

### Map the room in the headset (passthrough)

As Pimax's and Meta's room setups: in the headset you see your room through the headset cameras, and the scan, the
floor and the boundary are drawn over that real view. `pibridge-roomsetup --map` (`tools/roomsetup/src/mapflow.*`, the
OpenXR side `runMapXr` in `xrapp.cpp`) shows it and starts `pibridge-roommap setup fast --guide DIR` as its child (output in
`DIR/roommap.log`, printed when it ends); the app's MAP THE ROOM runs it.

1. **Look around.** "Stand in the middle of your play space. Look around slowly." A ring of 12 sectors at eye height,
   one on the floor and one above turns green where you have looked (`Coverage`), with a hint (LOOK LEFT / RIGHT /
   BEHIND YOU / DOWN AT THE FLOOR / UP), the views mapped and the points in view (pibridge-roommap's mapping feedback),
   and how much of the room was seen. A saved map is first relocalised (then mapped on), else mapped anew (30 s,
   `--map-seconds`).
2. **Floor.** With an eye height (Room page: body height x 0.936, or the measured eye height; pibridge.conf
   `room_body_height_m`, `room_eye_height_m`, passed as `--eye-height`) the first method (user decision 2026-10-19:
   controllers are tracked poorly near the floor and the map takes tables and beds for the floor): "Stand up straight,
   look ahead and pull the trigger". The floor goes the eye height below OpenXR's head pose (the eye midpoint),
   averaged over the last second while still (2 cm up/down, 4 cm sideways) and looking ahead (pitch within 15 deg),
   else "stand still and upright, look straight ahead" (answer `height Y EYE`, floor.toml source standing). On the same
   panel: A = the grid is on the floor, a controller on the floor + trigger = a touch. Without an eye height: as follows.
   The floor pibridge-roommap found (standing: at least 1 m below the head, a table is set aside) is drawn as
   a grid of 0.5 m cells on the real floor. Trigger = yes. A trigger with the controller low down (1 m or more below
   the head) is a touch: the floor goes there (`floor.toml`, source touch). Without a floor in the map: "Touch the floor
   with a controller and pull its trigger" (0.5..2.3 m below the head, else "that is not the floor").
3. **Boundary.** The free floor around where you stand (`setup::free_area` of the map points) is drawn as low green
   walls. A short trigger click keeps it; hold the trigger and draw the edge on the floor with the controller's ray (the
   ray's floor point every 5 cm, at least 2 m): `setup::clean_boundary` makes the boundary of it.
4. **Forward.** An arrow on the floor where you look. Trigger (or holding still for 3 s): the centre is where you stand,
   forward where you look. pibridge-roommap writes `room-setup.toml`, the offset (floor at y = 0 of the app space, the centre,
   forward -Z) and the status file exactly as the headless run did (`finish`).

**SET FLOOR FROM MY HEIGHT** (Room page, inside-out, the room recognised in this session) runs `pibridge-roomsetup
--floor-height --room <room> --eye-height M --roommap ... --ring ...`: the same panel over the passthrough with only
that step; the floor goes into the map (`pibridge-roommap floor set --y`) and the room offset (`pibridge-roommap apply`, the
writer lease), log `roomsetup-floor.log` in the session's logs.

The floor is always found again in the map (an older floor.toml is set aside as floor.toml.prev). A touch needs a
tracked controller (a controller out of the cameras' view keeps a held pose: "the headset does not see that
controller"). Logs: `roomsetup-map.log` and `roommap-setup.log` in the newest inside-out session's logs folder.

B twice stops it (pibridge-roommap ends with "stopped in the headset"); the panel follows your gaze when you turn away
from it. When pibridge-roommap ends the result stays on the panel for 5 s and the tool exits with its code.

**The two files** (`libpibridge-roommap/include/pibridge/roommap/guide_ipc.h`): `DIR/state` (pibridge-roommap -> headset: step,
title, text, progress, views mapped, points, `conv` = room_from_raw once the room fix is known, the floor and the
proposed boundary in the room frame) and `DIR/answer` (headset -> pibridge-roommap: one answer per step with an id: yes,
touch Y, path X Z ..., forward CX CZ YAW HEADY, cancel). pibridge-roommap stops when the headset tool is gone, the head
feed is lost for 2 s, or no answer came in 300 s. The headless `setup fast` (without `--guide`) is unchanged.

**Frames.** The headset draws in OpenXR's stage. The map speaks its room frame (`room = conv(raw)`, raw = the tracker's raw
SLAM frame of the head ring). The tool finds stage_from_raw (a yaw and a shift: the room offset and SteamVR's own
standing frame) from the same head pose seen in both: the ring's newest pose (times the
view offset `default_head_in_imu`) against the OpenXR view located at the ring sample's time (`StageLink`, a window of
60 pairs, restarted when 10 pairs in a row jump by more than 10 cm or 4 deg, e.g. a new offset). So nothing depends
on where SteamVR puts its stage, and nothing assumes a right world scale beyond what the map gives (the floor, the
boundary and forward go through the map's own frame).

**The passthrough** (`tools/xrkit/include/pibridge/xrkit/passthrough.h`, shaders `tools/xrkit/shaders/passthrough.*`,
also pibridge-home's: `xrkit::enablePassthrough` + a hook to `renderStereo`): all four cameras (factory ids 0..3, 640x480
grey fisheye, KB4 from the headset's own calibration, else the reference): the lower "tracking" pair and the upper
side "Pi-controller" pair, from the camera broker (`--camera SPEC`: broker, a `.pxcam`, or off). One 640x480x4 R8
array image, one 1.2 MB copy per camera frame (25 Hz) and one full-screen pass per eye. Per eye pixel: the eye ray at
the display time -> the first surface it meets: the saved room as a prism (its outline's walls, the ceiling at the
room's height, the floor; pibridge-home passes it, up to 16 walls), else the floor plane within 8 m, else a sphere of
2.5 m -> that point in each camera at the frame's own capture time (the frame's exposure timestamp, CLOCK_MONOTONIC,
to XrTime through `XR_KHR_convert_timespec_time`; without the extension estimated from the frame times: the time
warp) -> KB4. **Camera choice per pixel** (omarchyvr's `omarchyvr-passthrough` lut.rs, ported): per eye the lower
camera on its side, the other lower one, then the upper camera on its side and the other upper one (sorted by the
optical axis's pitch and the camera's x in the view, not by id). A camera's weight is 1 inside its image and falls to
0 over 24 px towards the tile border or the vignetting circle (330 px); sample A is the first camera with weight > 0,
where its weight wa < 1 the best other camera B is cross-faded in with alpha = wa / (wa + (1 - wa) wb), so there is no
hard seam. Each camera has its own gain (the lower pair's mean over its own, 0.6..1.6, smoothed) so the seams do not
show as a step, then the shared gain for dark rooms (x1..x4 towards mid grey).

Measured on the synthetic room (`pibridge_roomsetup_passthrough`, 80x60 samples, the reference calibration): seen
pixels looking straight ahead 83.5 % with the lower pair (34 % of the top quarter) -> 100 % with all four (100 % of the
top quarter), 10 % of the pixels cross-faded, no step above 0.25 at a camera change along a row; error against the
room traced from the eye over all pixels 3.6 grey levels on the 2.5 m sphere -> 2.2 on the room prism; floor 2.0; with
the head moved between capture and display 1.8 with the capture pose vs 13.9 without. Pictures (pibridge-home's self
test): `docs/app/home-5-passthrough-up-two-cameras.png` (the lower pair: the ceiling is black) vs
`home-6-passthrough-up-four-cameras.png` and the camera map `home-4-passthrough-up-coverage.png` (red / blue the lower
cameras, yellow / green the upper ones, blended at the seams).

What it looks like (the self test's software preview of the synthetic room through the reference calibration):
`docs/app/roommap-1-look-around.png`, `roommap-2-floor.png`, `roommap-3-boundary.png`, `roommap-4-forward.png`
(`pibridge-roomsetup --selftest --snapshot DIR`).

Tests: `pibridge_roomsetup_passthrough` (the reference pair, the eye image against the synthetic room traced from the
eye: about 2 grey levels on the floor; with the head moved between capture and display, 2 with the capture pose vs
14 without; the feed thread on a synthetic `.pxcam`; a lab recording when present), `pibridge_roomsetup_mapflow` (the
stage link and its jump, the coverage hints, floor yes / touch / refused, boundary kept / drawn / too short, forward by
trigger and by holding still with the frame conversions, stop, the end, no answer without the link),
`pibridge_roommap_guide` (the two files), `pibridge_roomsetup_options`, the self test's map steps, `pibridge_tracking_tests`
(the app's arguments and gate).

Not verified live (agents never touch a running session): the passthrough in the headset (the image orientation and
the camera-to-head offset on the real headset, the upper cameras' seams and gains (also with the auto exposure on), the clock conversion and the past-time pose under SteamVR, the
brightness), the stage link under SteamVR with pibridge-headtrack, the controller ray and touch heights, the whole run
against a live session and the broker.

### The floor under SteamVR (fixed 2026-10-19)

Live test (with the Monado SteamVR driver PiBridge used then, Crystal): after MAP THE ROOM the floor was wrong. Two causes, both fixed:
- **The frame.** The room-map tool (as the Rust original) put the floor at app y = -1.6: monado-service's tracking
  origin is 1.6 m above the STAGE floor. Inside vrserver, driver_monado's tracking origin offset is 0 ("use steamvr
  room setup instead", `ovrd_driver.cpp`): the app space is SteamVR's raw space and SteamVR shows its standing frame
  (from universe 2's chaperone "standing" pose, here y -1.52 m and yaw 0.2 rad, made by SteamVR's room setup). Now
  PiBridge's SteamVR link writes SteamVR's `GetRawZeroPoseToStandingAbsoluteTrackingPose` with the vrserver pid to
  `$XDG_RUNTIME_DIR/pibridge/steamvr-stage`; when the session pid is a vrserver, pibridge-roommap uses its standing height
  for the floor and composes its yaw and horizontal origin on top, so the floor lands on SteamVR's standing y = 0
  and the centre / forward on its origin / -Z (libpibridge-roommap `floor.h` `SteamVrStage`; without the file: raw
  y = 0, with a note). Other sessions keep the 1.6 m rule (Rust parity).
- **The table.** The fast setup took a desk 0.76 m below the head as the floor (the Rust gate is 0.4 m). The fast
  setup is done standing, so a points floor less than 1.0 m below the head is set aside (`floor.toml.rejected`), the
  map is searched again with `floor detect --min-below 1.0`, and with no such layer the controller touch is asked
  for ("Lay the RIGHT controller flat on the floor"). On the live map no layer was 1.0 m below the head (the floor was
  not seen), so the touch is what the next live run needs.

RECENTER IN HEADSET's floor touch was refused as "not the floor" because it was checked as |stage y| <= 1.5 m, i.e.
against the stage floor that was being fixed (off by more than a metre). Now the touch must be 0.5..2.3 m below the
head (same frame, `recenterFloorPlausible` in `tools/roomsetup/src/recenter.*`).

pibridge-roomsetup's controllers: the tool drew its box and the FLOOR-mode pointer along the grip pose's -Z. OpenXR's
grip -Z runs along the handle; on the Touch profile the Pimax controllers get it is 60 deg above
the aim ray, plus PIMAX_CTRL_ANGLE 30 deg: straight up. The tool now uses the aim pose for its controller frame (the
grip only without an aim pose). The "generic model" is the tool's own wire box: it draws no render models (SteamVR does
not draw controllers inside an OpenXR scene); the real models are a follow-up.

### The order of the room offset and the recenter

Three things move the inside-out world; pibridge-headtrack applies them in this order (`tools/headtrack/src/world.cpp`,
as the Pimax driver's `pimax_hmd_head_relation`): raw SLAM pose -> the head ring records it -> the **room anchor
offset** (`pimax-anchor.offset`, written by MAP THE ROOM, `auto` or RECENTER IN HEADSET below, one writer at a time
through the lease) -> the **manual recenter** on top (a long press, 0.8 s, of a controller's Pimax button: yaw and
horizontal position, floor kept, for this session only). So the room map puts the
floor and the room in place, and a long press only turns / shifts on top of it; the floor height from the room map
is always kept.

A new room offset (recognised or set up) replaces an earlier recenter in pibridge-headtrack, so a long press made
before the room was recognised does not shift the room afterwards. (Monado's Pimax driver kept its recenter on top of
a later offset; the rule there was to recenter only after "Room recognised".)

### Recenter (inside-out)

Without a mapped room the world starts where the headset is; RECENTER IN HEADSET or the long press puts the
centre and forward right for one session.
**RECENTER IN HEADSET** (Room page, inside-out mode; needs the inside-out session running (START VR) and **Room anchor**
on, Tracking page: `PIMAX_ANCHOR=1`, the offset file and the head ring) starts `pibridge-roomsetup --recenter --recenter-out
$XDG_STATE_HOME/pibridge/recenter.txt --ring $XDG_RUNTIME_DIR/pimax-anchor.head` (`tools/roomsetup/src/recenter.*`):
1. in the headset: "Stand where the play area centre should be. Look forward. Pull the trigger." The newest raw head
   pose of the head ring (before its offset) is taken;
2. "Touch the floor with a controller and pull its trigger, or press B / menu to keep it." (the controller's height
   in the stage; more than 1.5 m off is refused);
3. the tool writes `pibridge-recenter 1 / pid N / head px py pz qx qy qz qw / floor keep|Y` and ends.

The app then writes the room offset (`app/src/core/anchor_offset.*`, one function `writeRecenterOffset`, a private
port of omarchyvr `omarchyvr-anchor/src/world.rs` until libpibridge-anchor (R1) is there): yaw only (gravity stays),
the head's forward onto -Z, the head's floor point to the origin, the height kept from the current offset or moved by
the floor touch. The formats are omarchyvr's byte for byte: `pimax-anchor.offset` ("pimax-anchor-offset 1", "pid N",
"pose ..." with 6 / 7 decimals, written to `.tmp` and renamed), the lease `omarchyvr-anchor.offset.owner`
("roommap <session> / lease 2 / writer <PiBridge's pid> / role setup") under `flock(omarchyvr-anchor.offset.lock)`.
The session is the ring's writer pid (pibridge-headtrack). PiBridge never ends another writer: while a live
one holds the lease (omarchyvr's room setup, auto, the AprilTag) the recenter is refused in plain words, and an
AprilTag offset of the session is not mixed with it. Saved boundaries are in the stage, so after a recenter they land
on the room again. Tests: `pibridge_roomsetup_recenter` (the ring layout of world.rs `ring_bytes`, the steps with
scripted controllers, the result text), `pibridge_steamvr_monado_tests` (offset and lease texts against world.rs, the
math, the writer's lease rules on made-up files), `pibridge_room_page_tests` (the gate). Not run live: the offset in
pibridge-headtrack under SteamVR, the in-headset panel, the floor shift when no offset existed yet (the height then starts from 0).

## The Monado backend in the Lighthouse mode

`pibridge-roomsetup --lh-stage --backend monado` (the Room page, or Room setup on a dashboard in the headset): the
recenter steps in the headset (stand at the centre and look forward, hold the trigger; touch the floor with a
controller, or keep the eye height), then the room's stage (`stage_from_raw`) goes to `<room>/lh-stage` and to
`$XDG_RUNTIME_DIR/pibridge/monado/lh-stage`, which steamvr_lh re-reads at once (`tools/roomsetup/src/lhstage.*`,
[monado.md](monado.md)). Until a room has it, Monado uses the raw Lighthouse space: its origin and floor are where
driver_lighthouse put them (the floor is often above your head), so run this setup once per room.

While it runs, the base stations are drawn where they stand as small wire boxes with a cross on the front (the side
that sweeps the room): steamvr_lh's `$XDG_RUNTIME_DIR/pibridge/monado/basestations`, re-read once a second (only while Monado writes it: a
file older than 5 s is from an earlier session and not drawn), so they
move with the stage as it changes (`parseBaseStations` / `drawBaseStations` in `lhstage.*`).

## Live tracing and pose sources

`pibridge/room/pose.h`: a `PoseSource` gives the head and both controllers (stage space, with the trigger) at a time.
`FakePoseSource` walks a polygon (the tests and the self test trace rooms with it); the headset tool's own OpenXR input
is the real source and TRACE OUTLINE uses it. `TraceRecorder` / `TraceFollow` collect a floor point every 5 cm while
the trigger is held and turn the walk into a simple counter-clockwise polygon (`polygonFromTrace`). Not written yet: a
source that reads the poses without an OpenXR session.

## Inside-out (SLAM) tracking: the world frame

A saved boundary only makes sense in a world frame that is the same at every start. Lighthouse gives that (the base
stations are the frame; SteamVR's universe id names it). Inside-out tracking (pibridge-headtrack, either engine) starts
a new map, and with it a new origin, each session; the room map (above) relocalises against the saved map at the start
and moves the world back onto the room, so the stage (and a boundary saved in it) is the same each time once the room
is recognised.

## Tests and self test

`libpibridge-room/tests` (ctest `pibridge_room_*`): geometry (ZoneTweak's `geom2_test.go` and `store_test.go` cases:
inset, fit, rigid motions; plus simplify, square up, snap, validity, Douglas-Peucker, trace to polygon), JSON (order,
untouched number text, errors), the chaperone store on synthetic files (load and pick, unknown fields and other
universes untouched, one backup per session, restore from the oldest backup, frame mode round trips, moving the centre
keeps the walls in both frame readings, floor delta, applying a room), saved rooms (ZoneTweak's `profiles_test.go`,
rename, full round trip, ZoneTweak's format), zone settings, the editor driven by scripted controller frames (grab and
move, insert, delete, undo, menu clicks, save, play area unlock / move / stretch / save, fit and centre, new outline,
trace by walking, saved spaces and restore, recentre, warning line drag, zone page, laser grab), the fake pose source
and tracer, the scene lists, font and software preview. `tools/roomsetup/tests` (`pibridge_roomsetup_*`): options, the
SteamVR glue with a fake helper runner, the self test (also run as `pibridge-roomsetup --selftest`). The app:
`pibridge_room_page_tests`. All files are in temporary directories; nothing is started.

## Not verified live / not done

- Nothing has run in a headset or against a running SteamVR yet: the OpenXR session, the Vulkan drawing, the
  controller bindings (Index, Vive, Touch, simple), haptics, the reference space change handling, the helper processes
  (universe, ReloadFromDisk, IVRSettings) are written and build, but are untested.
- Which frame mode SteamVR's own universes need is inferred from one file (OFF); check it live (ALIGN).
- Whether ReloadFromDisk alone makes SteamVR show a moved play centre at once, and whether the OpenXR stage then moves
  (the editor follows a reference space change), is to be seen; the fallback is a SteamVR restart.
- The zone on/off switch through the boundary opacity, and the floor fix written into the standing height, follow
  SteamVR's settings and file as documented by the OpenVR header; not checked live.
- Not ported from ZoneTweak: the set dressing and sound, the graphics quality page, the Pimax service restart and its
  live settings, the Lighthouse base station display, the desktop mirror window.
- A pose source outside OpenXR; the room map (MAP THE ROOM, auto at the start) has not run against a live session from the app yet. The app's floor plan has no live tracing of its own: tracing
  is done in the headset.
