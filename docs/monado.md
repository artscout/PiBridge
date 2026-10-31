# PiBridge's Monado fork (third_party/monado)

The Monado backend's OpenXR runtime is a fork of [Monado](https://monado.freedesktop.org/) (Collabora and
contributors, BSL-1.0). It reports itself as **"Monado PiBridge fork from OmarchyVR"**, with the version tag
`v25.1.0-pibridge`. Its lineage:

1. Monado upstream, 25.1.0.
2. omarchyvr's fork, branch `omarchyvr`: the Pimax driver (`src/xrt/drivers/pimax/`) for the Crystal Super and the
   Dream Air, written for omarchyvr (laughtt and contributors). It brings the 6DoF head, the controllers, the room
   anchor, the camera passthrough, haptics and the recorder tap.
3. PiBridge's branch `pibridge`: what the Monado backend needs (below).

How to build it and run it is in [INSTALL.md](INSTALL.md), section 7.2. This page says what the fork changes and how
it is kept.

## Where it lives

The sources are plain files in this repository, `third_party/monado`. The build is out of tree:
`~/gamedev/pibridge-monado-build` is where the app looks for `monado-service`. Another build folder works with
`rec_monado_service=` in `pibridge.conf`.

The fork's own git history is kept outside PiBridge's tree, so PiBridge's repository sees plain files and no nested
repository. On the development machine it is a separate git folder whose work tree is `third_party/monado`:
```
git --git-dir=$HOME/gamedev/pibridge-monado.git status
git --git-dir=$HOME/gamedev/pibridge-monado.git -c commit.gpgsign=false commit -am "..."
```
A fresh clone of PiBridge does not need it: the files in `third_party/monado` are the fork.

Monado's own `.gitignore` files apply inside `third_party/monado`. One generated file is committed on purpose:
`src/xrt/targets/openxr/active_runtime.cmake` (without it a fresh clone cannot configure).

## What the fork changes for PiBridge

| Change | Where | What it does |
|---|---|---|
| `pibridge_lh` builder | `targets/common/target_builder_pibridge_lh.c` | `PIBRIDGE_LH=1`: the Pimax HMD shows the picture, Valve's `driver_lighthouse` (steamvr_lh) poses it by the Lighthouse faceplate and tracks the controllers |
| Live Lighthouse stage | steamvr_lh, `LH_STAGE_FILE` | the room for Monado + Lighthouse, written by `pibridge-roomsetup --lh-stage`, re-read live; SteamVR's chaperone is never read. Without a stage (no room setup yet in this mode) the raw Lighthouse space is used: its floor is not y = 0 (the floor can be above your head), so run the room setup once |
| Hand slots: controllers switched on later | steamvr_lh (`HandSlotDevice`, device.hpp), `PIBRIDGE_LH_HAND_SLOTS` (on by default with `PIBRIDGE_LH=1`) | Monado's device list is fixed when monado-service starts (IPC copies it to every client), so a controller switched on later was refused ("Cannot add device after setup"). Now each hand is a Valve Index shaped device from the start that stands for the Index controller holding that hand now: poses, hand joints, inputs (copied at `update_inputs`), haptics, battery. Controllers coming after the setup are activated inside steamvr_lh and taken by their hand as soon as the driver gives them their role; off, the hand has no pose and its inputs are not active. Its inputs never change, as IPC needs. A hand held at the start by another kind of controller (a Vive wand) stays that controller. Trackers after the start are still refused (Monado has no `XR_HTCX_vive_tracker_interaction` for games anyway). `PIBRIDGE_LH_HAND_SLOTS=0`: upstream's behaviour, the Pimax controllers as the hands when no Lighthouse controller is on at the start |
| Index controllers: menu and recenter | steamvr_lh (`steamvr_lh_set_pibridge_hooks`), `pibridge_lh` builder, `drivers/pimax` (`pimax_hmd_recenter_button`) | the **left** Valve Index system button stays the apps' input (the menu); the **right** one never reaches the apps: held ~0.8 s it recenters, with the recenter ring, as the Pimax button does |
| Recenter moves the Lighthouse devices | steamvr_lh (`world_offset` hook), `drivers/pimax` (`pimax_hmd_get_recenter`) | the Pimax HMD's manual recenter is applied to the Lighthouse controllers, trackers (and their hand joints) and the base stations too, not only to the head |
| Base station poses | steamvr_lh (`PIBRIDGE_LH=1`) | the base stations driver_lighthouse adds (they come after the setup window) are activated for their poses only, never as Monado devices; once a second their poses in the stage (recenter included) go to `$XDG_RUNTIME_DIR/pibridge/monado/basestations`, `<serial> px py pz qx qy qz qw` a line: PiBridge World and `pibridge-roomsetup --lh-stage` draw them |
| Per-backend files | `drivers/pimax/pimax_anchor.h` | `PIBRIDGE_BACKEND=monado`: the room anchor and the head ring under `$XDG_RUNTIME_DIR/pibridge/monado/`, never the SteamVR backend's |
| Eye gaze from PiBridge | `drivers/pimax`, `PIMAX_EYE_SHM` | `XR_EXT_eye_gaze_interaction` from pibridge-eyed's shared memory; Monado never opens the eye chip |
| winbridge source | `drivers/pimax` | head and controller poses from Pimax's own engine (winbridge, extension version 1 and 2) |
| Recenter ring and dashboard recenter | `drivers/pimax` | publishes the Pimax button hold as `$XDG_RUNTIME_DIR/pibridge/recenter-progress` (the ring of `pibridge-xroverlay`); recenters when `recenter-request` changes (the dashboards' Recenter) |
| Recenter for every app | `base/b_space_overseer.c`, `auxiliary/util/u_recenter.c` | after the driver's recenter the space overseer moves the shared LOCAL / LOCAL_FLOOR and every app's own onto the head, so seated apps face where you look too |
| Menus recorded full frame | `compositor/main/comp_recorder.c`, `PIMAX_RECORD_QUAD` | a frame with nothing but quads (a game's main menu) is recorded as its largest quad, fitted into the whole video frame; frames with 3D stay the eye images ([record.md](record.md)) |
| Client process ids | `targets/libmonado` (API 1.9) | `mnd_root_get_client_pid`: the dashboard's **Close app** ends the game in front |
| Runtime name, version tag | `CMakeLists.txt`, `cmake/openxr_manifest.in.json` | "Monado PiBridge fork from OmarchyVR"; without a `.git` of its own the tag is `v<version>-pibridge`, not PiBridge's commit |
| Pimax driver on PiBridge's C++ | `drivers/CMakeLists.txt`, `PIMAX_USE_PIBRIDGE=ON` | links `libpibridge_pimax.a` (monado-abi, [monado-abi.md](monado-abi.md)) instead of omarchyvr's Rust archives |

Each change is a commit of its own on the `pibridge` branch, with the reason in its message.

## Tests

`ctest --test-dir ~/gamedev/pibridge-monado-build` runs Monado's tests plus the fork's:

- `tests_space_recenter`: the LOCAL spaces follow a recenter.
- `tests_pimax_anchor`: the room anchor's files.

PiBridge's own tests read the fork's sources: the recorder's shared-memory layout is checked against
`comp_recorder_shm.h` (`PIBRIDGE_MONADO_SRC_DIR`, default `third_party/monado`), and monado-abi checks the C ABI the
built driver imports.

## The settings PiBridge passes

The app starts `monado-service` with the driver's settings as environment variables; the list for a session is in
its `session.log`. The main ones:

| Variable | From |
|---|---|
| `PIMAX_LENS_CONF` | the lens profile (Display page) |
| `PIMAX_SLAM`, `PIMAX_CTRL_*`, `PIMAX_CAM_*` | the Tracking page ([TRACKING.md](TRACKING.md)) |
| `PIMAX_CTRL_HANDS` | **Hands from the controllers** (off by default: games that enable hand tracking show the controllers) |
| `PIMAX_EYE_SHM` | eye tracking on |
| `PIBRIDGE_LH`, `LH_STAGE_FILE`, `STEAMVR_PATH` | the Lighthouse tracking mode |
| `PIBRIDGE_LH_HAND_SLOTS` | not set (on): the hand slots; `0` only for a test |
| `PIBRIDGE_BACKEND=monado` | always |
| `PIMAX_RECORD`, `PIMAX_RECORD_SHM`, `PIMAX_RECORD_QUAD` | the recorder tap ([record.md](record.md)) |
