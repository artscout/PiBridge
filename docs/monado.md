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
| Live Lighthouse stage | steamvr_lh, `LH_STAGE_FILE` | the room for Monado + Lighthouse, written by `pibridge-roomsetup --lh-stage`, re-read live; SteamVR's chaperone is never read |
| Per-backend files | `drivers/pimax/pimax_anchor.h` | `PIBRIDGE_BACKEND=monado`: the room anchor and the head ring under `$XDG_RUNTIME_DIR/pibridge/monado/`, never the SteamVR backend's |
| Eye gaze from PiBridge | `drivers/pimax`, `PIMAX_EYE_SHM` | `XR_EXT_eye_gaze_interaction` from pibridge-eyed's shared memory; Monado never opens the eye chip |
| winbridge source | `drivers/pimax` | head and controller poses from Pimax's own engine (winbridge, extension version 1 and 2) |
| Recenter ring and dashboard recenter | `drivers/pimax` | publishes the Pimax button hold as `$XDG_RUNTIME_DIR/pibridge/recenter-progress` (the ring of `pibridge-xroverlay`); recenters when `recenter-request` changes (the dashboards' Recenter) |
| Recenter for every app | `base/b_space_overseer.c`, `auxiliary/util/u_recenter.c` | after the driver's recenter the space overseer moves the shared LOCAL / LOCAL_FLOOR and every app's own onto the head, so seated apps face where you look too |
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
| `PIBRIDGE_BACKEND=monado` | always |
| `PIMAX_RECORD`, `PIMAX_RECORD_SHM` | the recorder tap ([record.md](record.md)) |
