# libpibridge-eye

The protocol of the Tobii EyeChip (USB 2104:0561) inside Pimax headsets (Dream Air, Crystal Super), in C++23. Ported
from omarchyvr `rust/pimax-eye` (usbfs, transport, ttp, tlv, session, frames, power, shm, fake, pcap, replay and the
offline part of analysis) and `live.rs` (the live tracker). CMake target `pibridge_eye`, namespace `pibridge::eye::`, headers
`include/pibridge/eye/*.h`. The live tracker (`live.h`, `src/live.cpp`) runs libpibridge-eyemath's gaze math on the chip's stream,
so `pibridge_eye` links `pibridge_eyemath` publicly; it has no dependency on libpibridge-hmd (the headset command comes in through
`power::HeadsetCommands`, implemented over libpibridge-hmd in `tools/eyed/`).

Nothing in this library invents protocol bytes: everything sent to the chip comes from a frame list made from a
Windows capture, through `frames::Guard`, which refuses the calibration opcodes. The only exception is the
calibration module (`calibration.h`): its own gate, closed by default and opened only by `pibridge-eyecal`, lets out
exactly the captured calibration requests (0x3f2, 0x42e, 0x44c, 0x3fc empty; 0x407 with one of the 9 captured
target payloads).

## Layers and headers

| Header | What it does | Key API |
|---|---|---|
| `error.h` | error type for I/O and policy refusals | `Error{code, os_errno, message}`, `Errc`, `Result<T> = std::expected<T, Error>` |
| `transport.h` | the three USB operations | abstract `Transport`: `control(setup, out, timeout)`, `bulk_out(ep, data, timeout)`, `bulk_in(ep, max, timeout) -> optional<Bytes>` (nullopt = timeout) |
| `usbfs.h` | Linux usbfs (ioctls, no libusb) and sysfs discovery | `UsbFs::open(path)`, `claim_interface(n)`, `reset_node(path)`, `scan_sysfs(root, dev)`, `find_eye_chips()`, `SysDevice` |
| `tlv.h` | `tag u8 | len u32 BE | value` items | `Items` (iterator, `next()`, `collect()`), `RawItem::decode() -> Value` (variant), `push_i32/…`, fixed-point conversions |
| `ttp.h` | 8-byte wrapper, 24-byte BE header, messages | `Wrapper`, `Header`, `Message::make/body/items/to_frames`, `StreamDecoder::push`, `captured_start`, `FrameError`, `opcode_name` |
| `session.h` | replay a plan, read the gaze stream | `run_plan(t, plan, opts, dec, log)`, `read_stream(t, dec, duration, GazeHandler)`, `CAPTURED_UNSUBSCRIBE_GAZE`, `OP_SUBSCRIBE` 0x4c4, `OP_UNSUBSCRIBE` 0x4ce, `ReplayError` |
| `frames.h` | start lists, the send gate, eye profiles | `parse(text)`, `select(steps, Skips)`, `Guard::make(steps)` / `Guard::make_with(steps, pin_cal)`, `from_plan(plan, source, keep_auth)`, `check_profile`, `profile_from_setpm`, `profile_body`, `profile_payload`, `with_profile`, `base_list()` |
| `power.h` | eye-module power policy | `HeadsetCommands` (interface), `send_enable(hid)`, `EnableGate::step`, `headset(root)`, `eye_chip_present(root)`, `headset_command_node(class, dev)`, `HEADSET_PIDS` {0x0044, 0x0040} |
| `shm.h` | gaze in shared memory (136 bytes, "OVEG", seqlock) | `Publisher::create(path)`, `publish(sample, state, written_ns)`, `read(path)`, `decode(bytes)`, `body(...)`, `Sample` |
| `fake.h` | in-process chip for tests | `FakeChip` (records `sent`, `status_override`, `silent`, `start_on`, `streaming`, `payload` hook) |
| `pcap.h` | USBPcap reader and writer | `PcapReader::open/from_bytes`, `next_packet`, `read_all`, `UsbPacket::parse/build`, `PcapWriter` |
| `replay.h` | the start plan of a capture | `build_plan(device, packets)`, `Plan` (`is_complete`, `missing_bytes`, `requests`, `auth_steps`, `describe`), `find_eye_devices`, `DeviceFinder` |
| `analysis.h` | offline capture report (used by `pibridge-eye pcap/plan/frames`) | `analyze_file(path, device, GazeCheck)`, `format_report` |
| `live.h` | the live tracker | `Config` (+ `from_env`), `Pipeline`, `run_link`, `Recovery`, `Failure`/`LinkError`, `Device`/`FnDevice`/`RealDevice`, `Tracker` (`start`, `state`, `latest`, `samples_since`, `sample_at`, `observe_target`, `drift`, `stats`, `last_message`, `stop`), `State`, `to_shm`, `fake_sample`/`fake_payload` |

Error style: `std::expected` everywhere. Modules with their own failure taxonomy return their own error struct with a
`kind` and `to_string()` (`ttp::FrameError`, `tlv::TlvError`, `pcap::PcapError`, `session::ReplayError`); parsers of
text formats (`frames::parse`, profiles) return a plain-words `std::string`.

## The live tracker (`live.h`)

```cpp
auto cfg = pibridge::eye::live::Config::from_env();     // PIBRIDGE_EYE_*, eye-calibration.toml / eye-correction.toml
cfg.frames = std::make_shared<const std::string>(start_list_text);   // the Crystal Super's own start list
cfg.profile = std::make_shared<const pibridge::eye::Bytes>(profile_body); // the user's eye profile (implies with_cal)
auto hid = std::make_shared<MyHeadsetCommands>();   // the eye-module enable (tools/eyed: over libpibridge-hmd)
auto tr = pibridge::eye::live::Tracker::start(cfg, std::make_unique<pibridge::eye::live::RealDevice>(hid), log);
if (auto s = tr->latest()) {
    const auto o = pibridge::eye::live::to_shm(*s);         // what the gaze file carries
    publisher->publish(&o, static_cast<std::uint32_t>(tr->state()), pibridge::eyemath::monotonic_ns());
}
tr->stop();                                          // unsubscribe 0x4ce, release the interface
```

One `std::jthread` per tracker, the stop token reaches every wait. The loop (`live.rs` `Tracker::start_device`):
`Device::open` (RealDevice: the first 2104:0561 in sysfs, refused when a kernel driver owns interface 0, usbfs open
and claim; `Config::usb_device` names a node instead). Open failed: `Device::absent` (RealDevice: the eye-module
enable through `power::EnableGate`, once per headset enumeration after 5 s without the chip), wait `retry`, doubling
up to 10 s. Open: `run_link`, then `Device::silent` after a silent failure (RealDevice: the enable again, at most every
5 s), `Recovery::after_link` (USB port reset after `reset_after` silent links, at most every `reset_min_interval`,
then 1.5 s for the re-enumeration), and `Reconnecting` with the back-off. States: `Searching` 0, `Connecting` 1,
`Streaming` 2, `Reconnecting` 3, `Stopped` 4 (the numbers the gaze file carries).

`run_link`: parse the start list (`Config::frames` or the built-in base list), `frames::with_profile` when a profile
is set, `select` (cal left out unless `with_cal` or a profile; vendor write left out; auth on `skip_auth`),
`Guard::make_with` (pinned only for the built-in list without a profile) plus the captured unsubscribe; read 300 ms:
gaze already flowing = adopt the earlier session (nothing is sent, no unsubscribe). Else send every step through the
guard (paced like the capture unless `pace` is off), wait for each response (`response_wait`, at least 10 s for
0x456), refuse a status other than 1 (auth ops excepted). Then read: `Streaming` at the first sample, `Stall` after
`stall` without gaze (10 s for the first one). On the way out, if it subscribed: the captured 0x4ce and 300 ms of
draining. Failures: `NoResponse`, `Stall` (both "silent"), `Status`, `Io`, `Other`.

`Pipeline` is eyemath's pipeline on the IN pipe (docs/libpibridge-eyemath.md): `parse_payload`, `ClockMap`, `convert` with
the calibration's conversion when one is loaded, `apply_sample_with` with the shared drift (or the old correction),
`filter_sample`; a new link resets the framing, the clock map and the filter. `Tracker::observe_target` is the
recentre/drift hook. Log lines go through a callback, each line only once in a row (stderr by default).

## Interfaces between the parts

- **Transport** (`transport.h`): the live tracker talks to `usbfs::UsbFs`; tests use `fake::FakeChip`. Every send to
  the chip should go through `frames::Guard::control` / `Guard::bulk_out` (the gate refuses anything not
  byte-identical to the list, and every denied opcode, even after `allow_frame`).
- **HeadsetCommands** (`power.h`): `Result<void> send(std::string_view id)` sends one named entry of the device
  allowlist. The eye enable is `power::EYE_POWER_COMMAND` = `"eye_power_on"` (bytes `58 05 3c 05 00 01` in
  `devices/dream-air.toml` and `devices/crystal-super.toml`; `power::enable_report()` holds the same bytes for
  checking). Implemented over libpibridge-hmd's allowlist gate by `pibridge::eyed::HidHeadsetCommands` (docs/eyed.md); this
  library never opens hidraw. `live::RealDevice` rate-limits it with `power::EnableGate` (once per headset
  enumeration, after the chip was absent > 5 s) and `SILENT_ENABLE_INTERVAL` (5 s) after a silent link.
- **GazeHandler** (`session.h`): `std::expected<void, std::string>(const ttp::Message&, Duration at)`, called for
  every stream-0x0505 notification by `read_stream`. Without a handler every notification counts as a sample. (The
  live tracker does not use `read_stream`: `live::Pipeline` decodes the stream itself, as `live.rs` does.)
- **GazeCheck** (`analysis.h`): the same hook for offline captures (`payload`, `complete`).
- **fake::FakeChip::payload**: the notification payload generator; `live::fake_payload` (eyemath's `encode_payload`
  of `live::fake_sample`, the Rust `FakeChip::sample`) gives realistic gaze streams.
- **shm::Sample**: the subset of a tracker sample that goes into the shared-memory file (per-eye origin/direction
  with validity, combined ray, sample number, host time, IPD); `live::to_shm` maps eyemath's sample to it.

## Files and formats

- Frame lists: the omarchyvr format (`STEP n | class | description`, `BULK_OUT ep=0x05 … <hex>`), unchanged.
  `frames::base_list()` is the public base list, compiled in from `libpibridge-eye/data/base-frames.txt`
  (a byte array generated by CMake at configure time). `frames::list()` always returns it: no private full list is
  ever built into this tree; users' own lists are files made by `pibridge-eye frames`.
- Eye profile body (`eye-profile-*.bin`): the bytes after the 16-byte `calibration.setpm` header; `with_profile`
  replaces every 0x456 step (header seq/status/aux kept, 4087-byte frame bodies as Windows splits it). A list with a
  replaced profile needs `Guard::make_with(steps, false)` (the pinned guard only accepts the built-in blob).
- Gaze shared memory: byte-identical to `patches/chovr/pibridge_eye_gaze.h` (a test compares
  against a copy of that C struct). `shm::default_path()` is `/dev/shm/pibridge-eye-gaze`, the path the
  deployed daemon and the SteamVR driver use: tests write only to temporary files.

## Build and test

```
cmake -S . -B build -G Ninja -DPL_BUILD_APP=OFF
nice -n 19 cmake --build build -j8 --target pibridge_eye
nice -n 19 ctest --test-dir build -R pibridge_eye_ --output-on-failure
```

12 test binaries (`pibridge_eye_tlv`, `_ttp`, `_pcap`, `_replay`, `_frames`, `_session`, `_usbfs`, `_power`, `_shm`,
`_fake`, `_analysis`, `_live`), 109 cases. `pibridge_eye_live` (22 cases) has a twin of every test of `live.rs`; the nine
Rust tests that needed the private full start list run here on the public base list (it has the same 0x4c4, 0x456 and
0x547 steps) against FakeChip with eyemath-encoded gaze. C++-only live tests: a bad status, a silent request, the
user's profile replacing the list's upload (unpinned guard, the exact payload), an own start list (and a broken one),
`Config::from_env` (variables, a broken profile, defaults, a calibration under the temporary HOME), `to_shm`,
RealDevice on temporary sysfs trees (enable once per enumeration, the 5 s silent interval, a kernel driver owning the
chip) and the log de-duplication. Every unit test of the ported Rust modules has a C++ twin with the same cases and
expected values. Tests that need private lab data in Rust (`cfg_attr(not(lab_data), ignore)`) print `skip …` and
pass; where possible the same behaviour is checked on the public base list instead (for example the replay engine
tests of `tests/replay_fake.rs`). The proptest suites of `tests/props.rs` for TLV, TTP, pcap and the plan builder are
replaced by deterministic pseudo-random loops. No test opens a device node.

## The C ABI for Monado (`pimax_eye.h`)

`monado-abi/src/eye_capi.cpp` (M8, `docs/monado-abi.md`) exports the seven `pimax_eye_*` functions of the Rust
`pimax-capi/src/eye.rs` over `live::Tracker` with a `RealDevice` whose `HeadsetCommands` sends the allowlisted
`eye_power_on` through libpibridge-hmd's gate. The archive compiles this library (and libpibridge-eyemath) once more with
the parity flags and hidden visibility. Not used for the Crystal Super, whose eye runs through pibridge-eyed (one owner
of the chip): Monado starts without `PIMAX_EYE` there.

## Not ported (yet)

- `live.rs` differences: `Tracker::start(cfg)` without a device (the Rust default `RealDevice`) does not exist, the
  caller passes a `RealDevice` with its `HeadsetCommands`; RealDevice's log lines do not name the hidraw node (the
  command sink knows it); `RealDevice::silent` counts an attempt as soon as the headset is on USB (Rust also required
  the hidraw node to be found first; here the sink reports that as a failed send). `Tracker::last_message` is new
  (for the eyed status file). The calibration's frame conversion is printed in words, not as Rust's `{:?}`.
- Gaze decoding and everything about samples (`gaze.rs`, `track.rs`, `clock.rs`, `filter.rs`, `drift.rs`, `calib.rs`,
  `correction.rs`) is libpibridge-eyemath. Hence: `session::StreamSummary` has no `GazeStats`; `analysis` counts gaze
  notifications but has no per-eye states, device-clock rate or CSV export (the `GazeCheck` hook only says
  decodable or not; per-eye statistics need a `GazeStats` in `GazeSummary` and new report lines, and the existing
  analysis tests use undecodable synthetic payloads, so it is not a small change and stays open);
  `FakeChip::sample` is `live::fake_sample`.
- `power::send_enable(node)` that wrote hidraw directly: replaced by the `HeadsetCommands` interface.
- The device commands of the Rust CLI (probe, stream, replay, live, watch, ipd, power-on) and `replaycmd.rs`,
  `ipd*.rs`, `watch.rs`, `fixation.rs`, `calfit.rs`: not part of this library.
- `frames::DATA` / `ListKind::Full`: the private full cold-start list with the auth steps is never compiled in.
- The lab-data tests (fixtures of real captures, the full cold-start list): skipped with a message.
- Stream id 0x0509: not in the Rust specification, so not decoded here; every notification other than 0x0505 counts
  as `other_messages`.
