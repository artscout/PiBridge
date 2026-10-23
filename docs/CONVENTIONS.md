# PiBridge: conventions (binding for every part and every agent)

PiBridge (VR hub for Pimax headsets on Linux; not affiliated with Pimax) runs a Pimax headset (first: the Crystal Super, Lighthouse faceplate, under SteamVR) on Linux with C++
tools and one Qt app, like Pimax Play / Pimax EVO on Windows. The protocol knowledge comes from omarchyvr
(`~/gamedev/omarchyvr`, MIT OR Apache-2.0, see NOTICE): its Rust code is the **specification**. Port behaviour and
tests faithfully; do not redesign what the Rust code already gets right (timings, retries, guards, edge cases).

## Layout and ownership (one part per directory; never edit another part's files)

| Dir | Part | Ported from (omarchyvr `rust/`) |
|---|---|---|
| `common/` | shared headers: `pibridge_test.h` (tests), `pibridge_status.h` (status files) | owned by the lead; ask before changing |
| `devices/` | device descriptions with the HID allowlist (`crystal-super.toml`, `dream-air.toml`), verbatim copies | `pimax-device/devices/*/device.toml` |
| `libpibridge-hmd/` | HID: discovery, hidraw I/O, protocol frames, allowlist gate, descriptor parser, session (wake, keepalive, panel, rate), audio switch, process follower | `pimax-proto`, `pimax`, `pimax-device` (hid, toml_lite, descriptor), `omarchyvr-tools/src/{audio,follow}.rs` |
| `tools/keepalived/` | `pibridge-keepalived` daemon | `omarchyvr-tools/src/keepalived.rs` |
| `tools/record/` | `pibridge-record` (video recording from SteamVR through the Vulkan capture layer or its mirror, `pibridge-record-mirror`) and `pibridge_record` (the app's Record card links it); doc `docs/record.md` | `omarchyvr-record` (cli, encode, run, settings, shm, status) |
| `libpibridge-eye/` | eye chip protocol: usbfs transport, TTP, TLV, TTP session (subscriptions, gaze notifications), frame lists + guard + profile, power, shm, fake chip, pcap reader and replay plan | `pimax-eye` (usbfs, transport, ttp, tlv, session, frames, power, shm, fake, pcap, replay) |
| `libpibridge-eyemath/` | gaze math (no I/O): gaze samples, device-to-head frame conversion, per-eye calibration map (`eye-calibration.toml`), correction, One-Euro filter, drift, device clock | `pimax-eye` (gaze, track, calib, correction, filter, drift, clock) |
| `libpibridge-eye/src/live*` | the live tracker (phase 2, after both libraries above) | `pimax-eye/src/live.rs` |
| `tools/eyed/` | `pibridge-eyed` daemon | `omarchyvr-tools/src/eyed.rs` |
| `tools/eye-tool/` | `pibridge-eye` CLI: `profile`, `frames`, `pcap`/`plan` | `pimax-eye/src/cli.rs` (those commands) |
| `libpibridge-room/` | the room setup without graphics or runtime: geometry, SteamVR's chaperone file and `steamvr.vrsettings` zone keys, saved rooms, the in-headset editor's logic and scene lists, pose sources; `pibridge::room::`, target `pibridge_room`, doc `docs/ROOM-SETUP.md` | ZoneTweak (Go, MIT, see NOTICE), not omarchyvr |
| `tools/roomsetup/` | `pibridge-roomsetup`: the OpenXR + Vulkan in-headset room setup, its OpenVR helper processes, `--selftest` | ZoneTweak `xr.go`, `main.go` |
| `tools/xrkit/` | `pibridge_xrkit`: the OpenXR + Vulkan plumbing shared by the in-headset tools (session, swapchains, the scene pipelines and shaders); `pibridge::xrkit::`, doc `docs/home.md` | moved out of `tools/roomsetup` |
| `tools/home/` | `pibridge-home`: the VR home scene in the headset (VR home page), `--selftest`; `pibridge::home::`, doc `docs/home.md` | new |
| `app/` | Qt 6 app, Pimax EVO layout | new; quad views page from `~/gamedev/artscout-qv-vulkan-linux/gui` + `layer/src/settings.*`, `fovmath.*` |
| `patches/`, `udev/` | setup files for the Crystal Super (CustomHeadsetOpenVR and NVIDIA patches, EDID and driver install scripts, WirePlumber rule) and our udev rule; see `patches/README.md` | copies from omarchyvr `linux/crystal-super/`, `patches/nvidia/` (as patch sets per GPU generation and driver version) |

Each part has `CMakeLists.txt` (the root adds it when present), `include/` or `src/`, `tests/` (registered with
`add_test`), and a doc `docs/<part>.md`.

## Language and style
- C++23 (GCC 15, Clang), CMake >= 3.22, Ninja. No exceptions across a library boundary: `std::expected<T, Error>`.
- Namespaces `pibridge::hmd::` (libpibridge-hmd), `pibridge::eye::` (libpibridge-eye) and `pibridge::eyemath::` (libpibridge-eyemath). Headers
  `include/pibridge/hmd/*.h`, `include/pibridge/eye/*.h`, `include/pibridge/eyemath/*.h`. CMake targets `pibridge_hmd`, `pibridge_eye`, `pibridge_eyemath`.
  Daemons: `pibridge::keepalived`, `pibridge::eyed`; the app `pibridge::app` / `pibridge::ui`; status files `pibridge::status`; tests
  `pibridge::test` (`PIBRIDGE_TEST_MAIN()`). CMake options and variables start with `PIBRIDGE_`.
- Linux syscalls directly (hidraw, usbfs ioctls, ppoll, eventfd) as the Rust code does: no libusb, no hidapi, no Boost.
  The app may use Qt 6 (Widgets). Nothing is downloaded at build time.
- Comments in English, plain words, explain why; match the density of the Rust originals' doc comments.
- Threads: `std::jthread` + stop tokens or an eventfd; logging through a callback `std::function<void(std::string_view)>`.
- Every source file starts with `// SPDX-License-Identifier: MIT`. A file ported from omarchyvr adds a second line:
  `// Ported from omarchyvr <crate>/<file> (MIT OR Apache-2.0).`

## Safety (the same rules as omarchyvr; no exceptions)
- HID bytes only from the allowlist in `devices/*.toml` (captured from Pimax Play under Windows). The gate refuses any
  other report. Eye chip bytes only from a frame list (captured start) through the guard; calibration opcodes refused
  unless a later, explicit feature allows them.
- **Agents never touch hardware**: do not open `/dev/hidraw*`, `/dev/bus/usb/*`, do not run a daemon or tool against a
  real device, do not stop or start the user's services (`pibridge-*`, `pimax-*`, `omarchyvr-*` keepalived/eyed), never touch a
  running SteamVR. All tests use fakes (in-process fake devices as in the Rust `fake.rs` files).
- No sudo. No installs. No git commits or pushes (the user commits). Builds and tests at `nice -n 19`, `-j8`.
- No serials, keys, e-mail addresses or home paths in the tree. Private lab data (captures, start lists, profiles)
  never enters the repo; tests use the public synthetic data (`pimax-eye/data/base-frames.txt` is public: copy it).

## Files and formats (keep compatible with what is deployed today)
- Gaze shared memory: `/dev/shm/pibridge-eye-gaze`, 136 bytes, magic "OVEG", seqlock, layout of
  `libpibridge-eye/include/pibridge/eye/shm.h` and `patches/chovr/pibridge_eye_gaze.h` (CustomHeadsetOpenVR
  reads it). Byte-identical.
- Frame lists (`eye-start-*.txt`), the eye profile body (`eye-profile-*.bin`), `eye-calibration.toml`: same formats
  as the Rust code. Default paths: `~/.config/pibridge/`, falling back to `~/.config/pimax-linux/` (PiBridge's earlier name)
  and then `~/.config/omarchyvr/` when the file is only there.
- Keepalive rate file and audio behaviour as in `pimax/src/session.rs` and `keepalived.rs`.
- Status files for the app: `pibridge::status::StatusWriter` (`common/pibridge_status.h`) -> `$XDG_RUNTIME_DIR/pibridge/<name>.json`,
  rewritten at least every 2 s while the daemon runs and at every state change. Names and fields:
  - `keepalived.json`: `state` ("idle" | "waking" | "active" | "no-headset" | "error" | "paused"), `paused` ("paused:
    inside-out tracking" or ""), `model`, `headset_present`,
    `panel_on`, `rate_hz`, `follow` (process name or ""), `follow_running`, `audio_on_headset`, `error` (plain words,
    "" when none), `pid`.
  - `eyed.json`: `state` ("idle" | "searching" | "connecting" | "streaming" | "error" | "paused"), `paused`, `profile` ("user" | "list" |
    "none"), `eyes_seen`, `samples_per_s`, `calibration` (path or ""), `follow`, `follow_running`, `error`, `pid`.
  - `home.json` (pibridge-home): `state` ("starting" | "running" | "visible" | "focused" | "ending"), `visible`,
    `focused`, `runtime`, `space` ("stage" | "local"), `room`, `error`, `pid`.
- The inside-out pause marker `$XDG_RUNTIME_DIR/pibridge/inside-out.active` (`common/pibridge_pause.h`): `pibridge-inside-out 1`,
  `owner <app pid>`, `variant steamvr` (`monado` from the old session). While it names a running owner, pibridge-keepalived and pibridge-eyed close
  the headset (HID, eye chip; no gaze written) and report "paused"; a stale one (owner gone) does not count.
- Shared settings (the app writes them, the daemons read them at start and on SIGHUP):
  `~/.config/pibridge/pibridge.conf`, `key=value` lines, `#` comments, unknown keys ignored, numbers with a
  point in every locale. Keys (defaults): `follow=vrserver` (the process whose life the daemons follow; empty = always
  on), `refresh_hz=90` (72 | 90), `audio_to_headset=on`, `eye_tracking=on`, `eye_start_list=` (path; empty = default
  path), `eye_profile=` (path; empty = default path), `eye_flip_x=1`; the app's own keys: `tracking_mode=lighthouse`
  (lighthouse | inside-out; inside-out = pibridge-headtrack + SteamVR), `home_background=passthrough` (the VR home's background: passthrough |
  none (the dark grid) | dusk (built in) | a picture id of tools/home/src/backgrounds.cpp, downloaded to
  `~/.local/share/pibridge/backgrounds/<id>.ppm`; docs/home.md), `home_background_last=` (the app's: the picture the
  passthrough switch goes back to), `home_panel=on` (the PiBridge window in the VR home), the other `io_*` inside-out settings and the
  `rec_*` programs and files (docs/TRACKING.md). Parser: `libpibridge-hmd` `pibridge::hmd::Settings`
  (`include/pibridge/hmd/settings.h`); the app has its own writer that keeps unknown lines.
- Environment overrides are named `PIBRIDGE_*` (`PIBRIDGE_EYE_FLIP_X`, `PIBRIDGE_DEVICES_DIR`, ...); the older `PIMAX_*` name
  of each is still read when the `PIBRIDGE_*` one is not set (omarchyvr and `pimax-eyecal` use them): `common/pibridge_env.h`.
- systemd user units shipped by each daemon part: `pibridge-keepalived.service`, `pibridge-eyed.service` (the app starts,
  stops and restarts them with `systemctl --user`).

## Build and test
```
cmake -S . -B build -G Ninja && nice -n 19 cmake --build build -j8 && nice -n 19 ctest --test-dir build
```
A part is done when it builds without warnings, its tests pass, the ported Rust tests have C++ twins (same cases,
same expected values), and `docs/<part>.md` says what it does, how to run it and what is not ported yet.
