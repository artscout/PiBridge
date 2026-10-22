# Development: Record tracking

The **Development** page of the app (called "Tracking" before; the Tracking page is now the tracking mode and the
inside-out session, [TRACKING.md](TRACKING.md)) has one big button, **RECORD TRACKING**. It records the headset's tracking cameras
and its IMU as a EuRoC dataset from a Monado 6DoF session, for offline SLAM analysis (Basalt replays, camera-IMU lag,
IMU noise). While it records, the app reads each step out loud and shows it in big letters, so the user can follow
it with the headset on. Afterwards it checks the dataset and says "OK: usable" or what is wrong.

The headset shows no picture during a recording. That is fine: only the cameras and the IMU matter.

Two more cards on the page: **Monado build** (which Monado every start uses) and **Monado directly** (the debug /
A-B session). Both below.

## Monado build

A selector between **Auto** (the key cleared: the lookup below) and **C++ (PiBridge)**
(`~/gamedev/pibridge-monado-build/...`, built from `third_party/monado`, docs/monado-abi.md "The PiBridge-linked Monado
(M9)"). It writes the existing key
`rec_monado_service`; a path set by hand shows as "chosen by hand". Record tracking, Monado directly and START TRACKING
(SteamVR + Monado: its `steamvr-monado/` and rttime preload are the same build's) use it from their next start.

The card shows (`core/monado_build.*`, read from the binary, cached by path, size and mtime):
- the build: **C++ (PiBridge)** with the whole `PIBRIDGE_PIMAX_ABI` string ("PIBRIDGE_PIMAX_ABI 1 spec omarchyvr 5ffbcc5"), or
  **unknown** (no such marker: a Monado that does not link `libpibridge_pimax.a`);
- the required markers of omarchyvr's preflight: `PIMAX_SLAM`, `PIMAX_ANCHOR`, `PIMAX_EYE`, `PIMAX_CTRL_TRACK`,
  `held from the tracker`, plus `PIMAX_AE_MODEL` and `PIMAX_SLAM_DRIFT_HOLD`. A missing one is said in red and
  blocks every start that uses the build ("lacks a required marker (...)");
- the SteamVR driver dir of the build;
- **cap_sys_nice** of monado-service, read from its `security.capability` attribute (no libcap): "Realtime priority
  allowed (cap_sys_nice=ep)" or the plain reason it is missing (every relink drops it). Missing: an amber note at
  Record tracking and Monado directly (not blocking; it does not matter for SteamVR + Monado).

**Fix permissions** (shown while the capability is missing): a dialog says what realtime priority is for and that
PiBridge will ask for the password to run exactly `pkexec setcap cap_sys_nice=ep <that monado-service>`; only on
"Ask for the password" it runs (through the Runner, never automatically). The result in plain words (done, the prompt
was closed, not allowed, setcap's error), then the state is read again.

## Monado directly (debug / A-B)

START MONADO runs the camera broker, the controller tracker and monado-service with the Tracking page's settings (the
recorder's session in `Mode::Live`, `IoVariant::MonadoDirect`): the display lease retry, the home scene
(`io_home_scene`, the switch is on this card; [home.md](home.md)), the PID-exact reverse stop. While it runs, own
apps on the Apps page start on it (`XR_RUNTIME_JSON`, TRACKING.md "Apps while Monado directly runs"), the Dashboard's
Controllers card reads Monado through libmonado and the headset card shows Monado's state. It works in either
tracking mode. It is blocked while SteamVR, START TRACKING or a recording runs, and blocks them while it runs.

## What runs

In this order (the same as omarchyvr's `crystal-6dof.sh`), each with its log in the recording's `logs/` folder:

| # | Program | Arguments / environment |
|---|---|---|
| 1 | `pimax-cam` | `broker --source <camera> --socket $XDG_RUNTIME_DIR/omarchyvr-cam-broker.sock`; the app waits (10 s at most) for the socket |
| 2 | `pimax-ctrltrack` | `live --source broker:<socket> --factory` |
| 3 | `monado-service` | the inside-out environment of [TRACKING.md](TRACKING.md) (one builder, the Tracking page's settings; with the recommended ones: `PIMAX_FPS=<refresh_hz> PIMAX_LOG=info PIMAX_BIAS_TIMEOUT_MS=200 PIMAX_LENS_CONF=<lens> PIMAX_CAM_CALIB=<calibration> PIMAX_CAM_SOURCE=broker PIMAX_SLAM=1 PIMAX_CAM_AE=1 PIMAX_CAM_AE_MODEL=crystal-super PIMAX_CTRL_TRACK=1 PIMAX_CTRL_ANGLE=30 VIT_SYSTEM_LIBRARY_PATH=<libbasalt.so> SLAM_CONFIG=~/.config/pibridge/slam/pimax-basalt.toml PIMAX_SLAM_DRIFT_HOLD=0 XRT_COMPOSITOR_FORCE_WAYLAND_DIRECT=1 XRT_COMPOSITOR_WAYLAND_CONNECTOR=<connector> XRT_COMPOSITOR_FORCE_NVIDIA=0 XRT_COMPOSITOR_LOG=info XRT_NO_STDIN=1 IPC_IGNORE_VERSION=1`) plus `PIMAX_SLAM_RECORD=<recording folder>`, then the words of `rec_env=` |

Without Basalt (or with head tracking off on the Tracking page), `PIMAX_SLAM=0` and no `VIT_SYSTEM_LIBRARY_PATH` /
`SLAM_CONFIG` (the recorder does not need the live tracker); without a lens file its variable is left out; with optical
controller tracking off, `pimax-ctrltrack` is not started. `SLAM_CONFIG` is PiBridge's copy of `rec_slam_config`
(grid, factory accel bias, measured gyro bias), written at the start. When the recording ends, the calibration and
settings that config names (what the live tracker used) are copied into the folder as `pimax-basalt-calib.json` /
`pimax-basalt-config.json`; Monado's own `pimax-basalt-calib.json` (the driver's calibration) is kept as
`pimax-basalt-calib.driver.json`. `PIMAX_CAM_CALIB` is also given to the broker and the
tracker. The steps start only once IMU data reaches the dataset (30 s at most), so the timeline matches the data.

- **Display lease**: Monado takes the headset's DRM connector from the desktop. That can fail ("Connector ... has been
  withdrawn", "vkGetDrmDisplayEXT failed", "xrt_instance_create_system failed"), and monado-service exits. The app
  sees this in `monado.log` and starts Monado once more after 4 s (broker and tracker stay, the second log is
  `monado-2.log`). A second failure is said in plain words and everything is stopped.
- **Stop** (end of the steps, the Stop button, or an error): SIGTERM to exactly the PIDs the app started, in reverse
  order: Monado first (so it finishes writing), then the tracker, then the broker. A program still there after its wait
  (Monado 10 s, the others 3 s) gets SIGKILL; one that survives that too is named in the message. Nothing else is ever
  signalled. Closing PiBridge during a recording stops it the same way (the app waits for it).
- Monado ends by itself during the recording, or the broker does: the recording stops, the message says so (with the
  last error lines of its log), and what was recorded until then is checked. The controller tracker ending does not stop
  the recording.

## Before it can start

The button is disabled and the page says why, in plain words, when:
- SteamVR (vrserver) runs ("Close SteamVR first"), another `monado-service`, `pimax-cam` or `pimax-ctrltrack` runs;
- no headset is found (a USB device of vendor 34a4 other than the camera), the tracking cameras are not found;
- the display connector is not found;
- a needed program or file is missing: the message names it and its pibridge.conf key.

Missing Basalt, SLAM config or lens file are shown as amber notes; they do not block.

## Settings (pibridge.conf)

All optional. Without a key the app looks in the places listed, first match wins (`~` is expanded).

| Key | What | Default lookup |
|---|---|---|
| `rec_monado_service` | monado-service | PATH, `<prefix>/lib/pibridge/monado/bin/`, `~/gamedev/pibridge-monado-build/src/xrt/targets/service/` |
| `rec_pimax_cam` | pibridge-cam | PATH, `<prefix>/bin/`, the build tree next to the app (`tools/cam/`) |
| `rec_pimax_ctrltrack` | pibridge-ctrltrack | the same (`tools/ctrltrack/`) |
| `rec_basalt_lib` | libbasalt.so | `~/gamedev/ovr-build/basalt/lib/`, `/usr/local/lib/`, `/usr/lib/` |
| `rec_slam_config` | Basalt SLAM config | `~/.config/pibridge/pimax-basalt.toml`, `~/gamedev/ext/slam/pimax-basalt.toml` |
| `rec_lens_conf` | lens file | `~/.config/pibridge/lens.conf`, `~/gamedev/ext/lens/crystal.conf` |
| `rec_cam_calib` | camera calibration (factory XML) | `~/.config/pibridge/headset-calibration.xml` (the headset's own), `~/gamedev/ext/calib/crystal-calibration.xml` |
| `rec_connector` | DRM connector name, e.g. `DP-6` | the connector under `/sys/class/drm` whose EDID monitor name says Pimax (or vendor id PVR); a connected one wins |
| `rec_cam_device` | camera device | the `/dev/videoN` whose `/sys/class/video4linux/videoN/name` starts with "PimaxSensor" and index is 0 |
| `rec_dir` | recordings folder | `$XDG_DATA_HOME/pibridge/recordings` (`~/.local/share/pibridge/recordings`) |
| `rec_steps` | a steps file (below) | the built-in steps |
| `rec_env` | extra `KEY=value` words for monado-service, space separated | none |

A key that names a file which does not exist is not replaced by a lookup: the page says the file is missing.

## Where it goes

```
<rec_dir>/<yyyyMMdd-HHmmss>/
  euroc_<datetime>/mav0/imu0/data.csv        IMU: #timestamp [ns], gyro x y z [rad/s], accel x y z [m/s^2]
  euroc_<datetime>/mav0/cam0/data.csv, data/  camera 0: timestamp [ns], file name; the PNGs
  euroc_<datetime>/mav0/cam1/...              camera 1 (the same timestamps)
  euroc_<datetime>/mav0/cam2/..., cam3/...    with four head tracking cameras (Tracking page): the upper pair
  pimax-basalt-calib.json, pimax-slam-info.json   written by Monado (calibration handed to Basalt)
  steps.json                                  which time window is which step (below)
  check.json                                  the check's result
  logs/broker.log, ctrltrack.log, monado.log, monado-2.log, session.log
```

`session.log` lists what the app did: every start with its PID, command line and environment, the signals, the check.

## steps.json

Rewritten at every step change and at the end, so a crash still leaves the steps so far.

```json
{
  "format": "pibridge-record-steps 1",
  "mode": "guided",
  "clock": "CLOCK_MONOTONIC ns, the dataset's clock (EuRoC timestamps of imu0 and camN)",
  "start_wall": "2026-10-19T14:12:05.123+02:00",
  "record_start_ns": 5123456789012,
  "record_end_ns": 5243456789012,
  "dataset": "euroc_20261019141205",
  "imu_first_ns": 5123401234567, "imu_last_ns": 5243450000000,
  "clock_matches_dataset": true, "offset_ns": 0,
  "stopped_early": false, "end_reason": "finished", "display_lease_retries": 0,
  "steps": [
    {"index": 2, "id": "still-mark", "kind": "still", "text": "Stand on your mark. Look straight ahead. Hold still.",
     "planned_s": 10, "settle_s": 2, "start_ns": 5131456789012, "end_ns": 5141456789012,
     "still_from_ns": 5133456789012, "start_s": 8.05, "end_s": 18.05, "skipped": false, "cut_short": false}
  ],
  "not_reached": []
}
```

- **Times** are integers in ns on CLOCK_MONOTONIC. Monado stamps the IMU and camera samples with the same clock
  (`os_monotonic_get_ns`, the camera frames mapped onto the IMU's timeline), so `start_ns`/`end_ns` compare directly
  with the dataset's timestamps. `clock_matches_dataset` checks that the IMU span overlaps the recording; when it does
  not (a dataset from elsewhere), `offset_ns` maps a step onto the dataset: `dataset_ns = step_ns + offset_ns`.
  `start_s`/`end_s` are seconds after the first IMU sample.
- **kind**: `prepare` (not a measurement), `still` (head still in its normal pose), `tilt-hold` (held still at a large
  tilt: gravity on the other axes), `move`. For `still` and `tilt-hold`, the still window is `still_from_ns..end_ns`
  (the first `settle_s` seconds are for getting into the pose).
- A skipped step (Skip step) ends at the skip and has `skipped: true`; the step running at a Stop has `cut_short: true`;
  steps never reached are listed by id in `not_reached`. A free recording has `"mode": "free"` and no steps.

## The steps

Built in (everything with the headset on the head; laying it on a table in set poses does not work):

| # | id | kind | text | s | settle |
|---|---|---|---|---|---|
| 1 | put-on | prepare | Put the headset on. Get ready. | 8 | |
| 2 | still-mark | still | Stand on your mark. Look straight ahead. Hold still. | 10 | 2 |
| 3 | tilt-left | tilt-hold | Tilt your head to the left shoulder and hold. | 6 | 2 |
| 4 | tilt-right | tilt-hold | Now to the right shoulder and hold. | 6 | 2 |
| 5 | look-down | tilt-hold | Look down at the floor and hold. | 6 | 2 |
| 6 | look-up | tilt-hold | Look up at the ceiling and hold. | 6 | 2 |
| 7 | still-ahead | still | Look straight ahead. Hold still. | 5 | 2 |
| 8 | step-away | move | Lean or step away and come back to the mark. | 20 | |
| 9 | back-on-mark | still | Back on the mark. Hold still. | 10 | 2 |
| 10 | turn | move | Turn your head slowly left and right. | 12 | |
| 11 | hold-after-turn | still | Hold still. | 3 | 1 |
| 12 | nod | move | Nod slowly. | 10 | |
| 13 | hold-after-nod | still | Hold still. | 3 | 1 |
| 14 | shake | move | Shake your head quickly. | 5 | |
| 15 | back-on-mark-end | still | Back on the mark. Hold still. | 10 | 2 |

2 minutes in all; then "Done. Recording stopped." Your own steps: a JSON file named by `rec_steps=`:

```json
{"steps": [{"id": "still", "kind": "still", "text": "Hold still.", "seconds": 10, "settle": 2},
           {"id": "nod", "kind": "move", "text": "Nod slowly.", "seconds": 10}]}
```

A broken file is named on the page and the built-in steps are used.

**Voice**: `spd-say` (priority "text", a little slower than normal), through the default sound output (the headset
when "Default audio output" is on). Without spd-say, a short tone with `pw-play` or `paplay` at each step; without
those, nothing. **Free recording**: no steps; it records until Stop.

## The check

After the stop the app checks the dataset (the C++ twin of `omarchyvr-slam check`, `core/rec_check.*`): IMU and
camera sample counts and rates, the largest gap (an IMU gap above 20 ms is a problem: "USB stall?"), gaps above 1.5
times the median interval, non-increasing timestamps, camera frames outside the IMU span (more than 2: "clock
mismatch?"), how many cam1 timestamps match cam0 (below 90 %: a problem), the median |accel| (more than 0.5 m/s² off
9.807: "scale wrong?"), and missing image files. The result is written to `check.json` and shown in the list of
recordings: date, length (the IMU span), size, OK / PROBLEM and the first problem. **Open folder**, **Logs** and
**Delete** (asks first; only a folder inside the recordings folder that holds steps.json or a euroc_* folder).

## Fake mode

`pibridge --fake` / `--screenshots`: nothing runs (Fix permissions only says what it would run). Three made-up recordings (one with a 35 ms IMU gap) are written into
the fake temporary data dir; RECORD TRACKING plays the start (socket after 0.8 s, data after 2 s), the steps in real
time and the stop with fake processes, and writes a made-up dataset. Screenshots: `07-development.png` (ready),
`07b-development-steamvr-running.png` (blocked), `07c-development-recording.png` (step 3), `07d-development-done.png`,
`07e-development-blocked-by-tracking.png`, `07f-development-monado-directly.png` (C++ build, Monado directly running).

## Not verified live yet

Everything is tested with fakes only (`pibridge_record_tests`, `pibridge_steamvr_monado_tests` for the build card): no real
recording has been made from the app, and Fix permissions has not been run (agents never run pkexec). To check
live: the display lease and its retry against KDE, that spd-say is heard in the headset, Monado's exit on SIGTERM
within 10 s with a complete dataset, and the start of the steps when IMU data arrives.
