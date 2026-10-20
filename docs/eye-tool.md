# pibridge-eye (tools/eye-tool)

Offline tools for the eye chip, ported from omarchyvr `pimax-eye/src/cli.rs` (the file commands). Nothing in this
tool talks to a device: it reads Windows captures and calibration files and writes files. The live eye tracker is
`pibridge-eyed`. Built on libpibridge-eye (`docs/libpibridge-eye.md`).

## Commands

```
pibridge-eye profile <calibration.setpm> -o FILE
pibridge-eye frames  <capture.pcap> -o FILE [--device N] [--keep-auth]
pibridge-eye pcap    <capture.pcap> [--device N] [--plan]
pibridge-eye plan    <capture.pcap> [--device N]
pibridge-eye help
```

- `profile`: takes the eye profile body out of the file Pimax's calibration saved under Windows
  (`ProgramData/Tobii/Tobii Platform Runtime/XR5EYECHIP/<headset>/calibration.setpm`). The header and the body's own
  length are checked; a broken file is refused and nothing is written. The output (`eye-profile-*.bin`) is the
  `eye_profile` the eye daemon uploads at every start in place of the start list's own 0x456 upload.
- `frames`: makes the eye chip start list of a Windows capture of the user's own headset: every vendor control
  transfer and TTP request up to the first gaze notification, byte for byte, in capture order; the two auth steps
  (0x76c, 0x777) are left out unless `--keep-auth`. A capture whose snap length cut any byte is refused (a list is
  never padded or guessed). Before writing, the made list is parsed back and must pass `Guard(pin_cal=false)`. When
  the capture holds several sessions, the device whose start reaches the gaze stream with the most steps is used.
  The output is the `eye_start_list` of headsets without a built-in list (the Crystal Super).
- `pcap`: a report of a capture: TTP devices, the message catalogue, gaze notification counts (cut, oversized,
  capture-clock rate) and whether the session start is complete; `--plan` lists every step.
- `plan`: the exact transfers of the session start and which ones the capture cuts.

Values take `--name value` or `--name=value`. Exit codes: 0 ok, 1 usage, 2 file or capture error.

Both written files hold private data of the user's headset (its eye profile): they are created with mode 0600 from
the first byte on (an existing file is narrowed to 0600). Do not share or commit them.

## Build and test

```
nice -n 19 cmake --build build -j8 --target pibridge-eye pibridge_eye_test_cli
nice -n 19 ctest --test-dir build -R pibridge_eye --output-on-failure
```

The tests (`pibridge_eye_cli`) run the commands end to end on a capture made from the public base list and on a
synthetic `calibration.setpm`, check the 0600 mode, and port the Rust CLI tests (`usage_and_errors`,
`args_accept_equals_form`). The `tests/cli.rs` cases on private fixture captures are skipped with a message.

## Not ported (yet)

- `pcap --csv FILE` and `--head N` (they print decoded gaze samples, which needs libpibridge-eyemath's decoder): refused with
  a usage error. The `pcap` report has no per-eye states or device-clock rate for the same reason.
- The device commands (`probe`, `stream`, `replay`, `live`, `watch`, `ipd`, `power-on`): answered with "talks to the
  eye chip or the headset and is not part of this tool"; the daemons and the app do that work.
