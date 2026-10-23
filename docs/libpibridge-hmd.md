# libpibridge-hmd

HID I/O for Pimax headsets (Crystal Super 34a4:0040, Dream Air 34a4:0044): protocol frames and report decoders,
sysfs discovery, hidraw I/O, the HID allowlist gate, the session thread, the in-process fake headset, the audio
switch, the process follower and the shared settings parser. Ported from omarchyvr `rust/pimax-proto`,
`rust/pimax`, `rust/pimax-device` (hid, toml_lite, descriptor, module) and `rust/omarchyvr-tools/src/{audio,follow}.rs`
(MIT OR Apache-2.0); the Rust code is the specification and its tests have C++ twins here.

CMake target `pibridge_hmd` (static), namespace `pibridge::hmd::`, headers in `libpibridge-hmd/include/pibridge/hmd/`. Optional dependency: zlib
(system, found with `find_package(ZLIB)`) for the calibration document; without it `calib::inflate_blob` returns
`Inflate` and the zlib-based tests say "skipped".

## Headers

| Header | What | From |
|---|---|---|
| `proto.h` | `Report` (64 bytes), ids/constants, `cmd::*`, `Model` + `MODELS` (`DREAM_AIR`, `CRYSTAL_SUPER`), `model_by_pid`; frames `build_command`, `wake`, `keepalive`, `video_prepare_commands`, `attach_commands`, `haptic_command`, `haptic_from_openxr`, `sys_reset`, `unpair`, `to_write_buf`; decoders `parse_reply`, `parse_serial`, `decode_imu`, `parse_status`, `decode_controller` (+ `TouchBits`), `LineSplitter` (IF3 log); `calib::` blocks, `Assembler`, `inflate_blob`, `build_blob` | pimax-proto |
| `sys.h` | `Error`/`Errc`/`Result<T>` (= `std::expected<T, Error>`), owning `Fd`, `ppoll`, `eventfd`, `seqpacket_pair`, `hid::open/read_report/write_report/is_gone`, `proc_cpu_seconds`, `local_hms` | pimax sys.rs, hid.rs, lib.rs |
| `discovery.h` | `Roots` (+ `PIBRIDGE_SYSFS_ROOT`/`PIBRIDGE_DEV_ROOT`), `find_headset` (model + IF0..IF3 nodes), `find_interfaces`, `mask`, `find_bridge_log`, `usb_devnum` | pimax discovery.rs |
| `toml_lite.h` | `toml::parse` for the TOML subset of the device descriptions | pimax-device toml_lite.rs |
| `allowlist.h` | `Allowlist::check` / `build` / `get`, `CommandSpec`, `Denied`, `Refused` (Denied, NotListed, OutOfRange, Padding, Length, UnknownCommand, MissingParam, TooSoon), `Gate::send` (check, then one write of `00 + report`; enforces `min_interval_ms`) | pimax-device hid.rs |
| `descriptor.h` | `Descriptor::parse/load` (`devices/<id>.toml`, validated like Rust), `ipd_report`, `pair_report`, `keepalive_report`, `devices_dirs()`, `load_descriptor(model)`, `load_allowlist(model)`, `detect` | pimax-device descriptor.rs, module.rs, devices/dream_air.rs |
| `session.h` | `Session` (incl. `haptic`, `haptic_counts`), `Config` (incl. `allowlist_for`), `Handler`, `Event`/`EventQueue`, `Stats`, `Backend`/`HidrawBackend`/`Link`, `State` | pimax session.rs |
| `haptics.h` | `haptics::Limiter` (at most one vibration command per hand per 8 ms, newest wins), the test pulse request file (`request_path`, `format_request`, `parse_request`, `take_request`) | pimax haptics.rs |
| `pairing.h` | controller pairing / unpairing: firmware log verdicts (`parse_log`), sides, the app's request file (`format_request`, `parse_request`, `take_request`); frames `start_pair()` / `unpair()` in proto.h | omarchyvr-autod pairing.rs, unpair.rs |
| `fake.h` | `fake::pair()` -> `FakeBackend` + `FakeDevice` (seqpacket socketpairs; `send_report`, `recv_command`, `drain_commands`, `unplug`, `plug`, `set_model`) | pimax fake.rs |
| `audio.h` | `audio::pick`, `to_headset`, `restore`, `Before`; injectable `Runner` (default `run_pactl`, posix_spawn) | omarchyvr-tools audio.rs |
| `follow.h` | `process_running(_in)` (/proc/*/comm, 15 chars), `Follow` (active at once, idle after `IDLE_AFTER` = 5 s) | omarchyvr-tools follow.rs |
| `settings.h` | `Settings::parse/load/default_path` for `pibridge.conf`, `legacy_path`/`load_or_legacy` (the `pimax-linux.conf` of PiBridge's earlier name while `pibridge.conf` is missing) | new (docs/CONVENTIONS.md) |

## The session

```cpp
pibridge::hmd::Config cfg;                       // wake on, keepalive every 213 ms, rate from set_video_rate()
cfg.rate_file = "/run/user/1000/pimax-rate";   // or: re-read the rate on every keepalive
auto s = pibridge::hmd::Session::open(cfg, std::make_unique<MyHandler>());   // real hidraw (HidrawBackend)
s.value()->wait_connected(std::chrono::seconds(5));
s.value()->video_prepare(90);            // panel on: 5b, 29, 1a 02 FPS, 1a 05; keepalive rate 90
```

One `std::jthread` per session: connect (polls the backend every 50 ms, 1-2 s after an error), wake, keepalive,
wake re-send after 500 ms without a `4c .. 1a` ack, optional controller auto-attach with the 0x37 retry logic,
one read per ready fd per cycle on IF0..IF3 (the firmware detaches when nobody drains IF1-IF3), reply matching for
`request`/`request_with`, reconnect after a hang-up. Handler callbacks run on the session thread. The rate file value
`off` means one keepalive with rate 0, then none until the file changes (Pimax Play closed: green standby LED);
missing or invalid = 0; numbers are truncated to a byte like Rust's `as u8`.

**Allowlist gate.** Every write of the session (`send`, `send_report`, `request*`, keepalive, wake, video and attach
frames) goes through a `Gate` built from the connected model's description. `Config::allowlist` sets it explicitly
(tests); otherwise the session loads `<id>.toml` of the link's model from `Config::devices_dirs` or
`pibridge::hmd::devices_dirs()`: `$PIBRIDGE_DEVICES_DIR`, the installed `<prefix>/share/pibridge/devices`, and
`../share/pibridge/devices` next to the executable. A refused report returns `Errc::Refused` (the reason in plain
words) and nothing is written; without a loadable description nothing at all is sent (the session still drains the
interfaces). `Stats::refused` counts refusals. The gate history (`min_interval_ms`) starts over at every connect,
which matches "once per enumeration". `Config::allowlist_for` (a function of the model) replaces the folder search
when set (it was used by the C ABI for Monado, now removed, to fall back to descriptions compiled into its archive).

**Vibration.** `Session::haptic(slot, {amp %, ms})` queues a pulse for the session thread, which writes
`58 09 07 SS AMP DUR` (through the gate, like every write) at most once per 8 ms per hand; a newer pulse replaces a
waiting one, `nullopt` (a stop) drops it and sends nothing. A refused pulse is counted and logged
(`haptic write: refused: ...`). `haptic_counts()` = commands handed to the gate, pulses replaced. The Crystal Super
lists `haptic` since 2026-10-19 (see `devices/crystal-super.toml` below).

**Crystal Super allowlist additions (PiBridge only, user decision 2026-10-19 in chat).** `devices/crystal-super.toml`
is omarchyvr's description at `5ffbcc5` (with `camera_ae`) plus: the `haptic` entry (the Dream Air's frame and
ranges: slot 2|3, AMP 1..100, DUR 1..65535 ms; on this headset the Windows capture of 2026-10-17 has only Pimax
Play's test pulse `58 09 07 03 19 2c 01`, 8 times in take `slam`; slot 2 and the game envelope come from the Dream
Air capture of 2026-10-15, takes 14/15, with the same controllers), and `approved` lines on the commands of
libpimax's attach burst (`read_serial`, `read_info`, `read_doc_controller`, `read_version` for slots 2/3) and video
prepare (`app_unlock`, `panel_on`, `announce_refresh` 72|90, `announce_gpu`). Those were already listed from the
capture (takes cold1, cold2, controllers, lighthouse, lighthouse2, settings, sleep-wake); every frame was found byte
for byte in the decoded takes (IF0 EP 0x01 of 34a4:0040), and the sources now carry the take times.

## Settings file

`~/.config/pibridge/pibridge.conf` (`$XDG_CONFIG_HOME` respected): `key=value` lines, `#` comment lines, unknown
keys ignored, later lines win, values trimmed and taken to the end of the line (paths may contain `#`), numbers via
`std::from_chars` (`90.0` works in every locale). Keys and defaults: `follow=vrserver` (empty = always on),
`refresh_hz=90` (72|90), `audio_to_headset=on`, `eye_tracking=on`, `eye_start_list=`, `eye_profile=`,
`eye_flip_x=1`. Booleans: on/off, 1/0, true/false, yes/no. A bad value keeps the default and adds a line to
`Settings::warnings`.

## Tests

`ctest -R pibridge_hmd_` runs 88 tests in 11 binaries: haptics (the 6 limiter / request file twins of haptics.rs plus
Rust `str::parse` / `split_whitespace` / UTF-8 edge cases), proto (framing, decoders, props twins with a fixed-seed sample of
20000 reports), fixtures (real captures), discovery (fake sysfs tree), toml, allowlist (against
`devices/dream-air.toml` and `devices/crystal-super.toml`), session (fake headset, incl. rate file `off`, the gate,
the model choosing its description), close, audio (fake pactl), follow (fake /proc), settings.

Test data: `libpibridge-hmd/tests/data/` holds the small public files (`status37_golden.txt`, `if3_log_coldstart.rec.gz`,
the synthetic calibration XML). The big public captures (`p82`, `p83`, `ctrl_buttons_hub2`, `ctrl_golden`,
`ctrl_marks`) are read from `PIBRIDGE_FIXTURES_DIR` (CMake cache, default `../omarchyvr/rust/fixtures` next to this
checkout); without them those tests print "skipped" and pass, and the session tests use synthetic IMU/controller
reports. No test opens a device.

## Not ported

- `Session::open_channel` with a `std::sync::mpsc` receiver is `open_channel_with_backend` returning a shared
  `EventQueue` (same bounded, drop-when-full behaviour).
- `sys.rs` signal helpers (`install_term_flag`, `block_term_signals`, `wait_for_term`): the daemons use a signalfd.
- `pimax-device` modules beyond the descriptor: `check.rs`, `checks/`, `autofix.rs`, `onboarding.rs` (only the step
  ids are used to validate descriptions), the compiled-in module registry and the three mock descriptions
  (`dream-air-se`, `crystal-light`, `8kx` are not in `devices/`), so the Rust registry/mock tests
  (`registry_has_two_real_modules_and_three_mocks`, `mocks_refuse_every_dream_air_command`, the
  `Possibly` branch of `usb_detection`) have no twin; `detect` is ported and tested on the two real descriptions.
  `DreamAir::ae_report` is covered through `Allowlist::build("camera_ae", ...)`.
- The private-lab tests (`calibration_reassembly_from_real_windows_capture`, the real-XML calibration round trip) run
  on the public synthetic calibration document instead.
- The C ABI of the Rust crate and the latency benchmark.

Differences on purpose: the session gates every write (the Rust session wrote unchecked; `pimax-device` gated only
module tools), so the Rust timeout test's opcode `0x2c` (refused by the allowlist) is replaced by the allowed read
`0x50`, and a refused wake does not tear the link down.
