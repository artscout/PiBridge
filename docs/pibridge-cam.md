# pibridge-cam

The tracking-camera tool, the C++ twin of omarchyvr's `pimax-cam` (`tools/cam/`, on `libpibridge-cam`). Its standard
output is the Rust tool's text, character for character, and its broker speaks PXCB v1 with Rust clients and
servers. Nothing is ever sent to the headset: the camera is a plain UVC stream.

```
pibridge-cam info      [--source S] [--frames N]                  first frames' metadata (with no source: the device found)
pibridge-cam stats     [--source S] [--seconds N] [--fps F]       intervals, latency, exposure sets, counters
pibridge-cam record    OUT.pxcam [--seconds N] [--fps F] [--zlib L] [--frames N] [--source S]
pibridge-cam calib     [XML] [--conv 1|2] [--overlay F]            parsed calibration and derived geometry
pibridge-cam import    IN.raw OUT.pxcam [--zlib L]                 raw YUYV frames (v4l2-ctl dump) -> recording
pibridge-cam broker    [--source S] [--fps F] [--socket P] [--slots N] [--loop]
pibridge-cam broker-status [--socket P] [--window S] [--json]      exit 0 streaming, 1 no broker, 2 camera down / no frames
```

S: empty or `auto` (the live camera, found under `/sys/class/video4linux`), `/dev/videoN`, `broker[:SOCKET]`, or a
`.pxcam` recording. `broker` serves one camera stream to any number of processes (default socket
`$XDG_RUNTIME_DIR/omarchyvr-cam-broker.sock`, the one Monado's driver and `pimax-ctrltrack` use); a live camera is
reopened after an unplug or a 5 s stall while the broker keeps its socket and its clients. A recording is paced in
real time (`--loop` restarts it). SIGINT/SIGTERM stop `record` and `broker` cleanly. Not ported: `tiles`, `rectify`,
`hypotheses`, `cpu`.

Drop-in for the session: `pibridge-cam broker --source /dev/videoN --socket $XDG_RUNTIME_DIR/omarchyvr-cam-broker.sock`
is the command line PiBridge's RECORD TRACKING and START TRACKING run with `pimax-cam` today (`rec_pimax_cam`). The
app still defaults to `pimax-cam` and looks for a running `pimax-cam` process; switching its default to `pibridge-cam`
is the app's change (plan M2, last step).

## Parity tests (`ctest -R pibridge_cam_tool -R pibridge_cam_broker_cross`)

Each runs the Rust `pimax-cam` of the pinned commit (`PIBRIDGE_RUST_ORACLE_DIR`) and `pibridge-cam` with the same
arguments and compares standard output (`monado-abi/difftest/run_difftest.sh`); skipped without the oracle or the
lab recordings.

| Test | Command | Compared |
|---|---|---|
| `pibridge_cam_tool_calib_reference`, `_conv1`, `_crystal`, `_home` | `calib` of the reference (both rig conventions), of the Crystal Super XML, and the default search in an empty HOME | the whole report |
| `pibridge_cam_tool_stats_{first,ctrl1,ctrl2}` | `stats --source REC.pxcam` | the whole report but the latency line (it depends on the absolute clock at the start of the replay; two Rust runs can differ there in the last digit) |
| `pibridge_cam_tool_record` | `record OUT --source first.pxcam --frames 20` | the report and the written file (headers, payload hashes, times relative to the first record) |
| `pibridge_cam_tool_import` | `import IN.raw OUT --zlib 0` | the report and the written file |
| `pibridge_cam_tool_errors` | `stats --source /nonexistent.pxcam` | exit status 1 |
| `pibridge_cam_broker_cross` | `tools/cam/tests/broker_cross.sh` | see below |

`broker_cross.sh` runs, on `first.pxcam`: every server ({Rust broker, pibridge-cam broker}) with every client (the
difftest C program on `libpimax_cam.a` and on `libpibridge_pimax.a`): every frame after connecting arrives, in order,
identical to a direct replay (metadata, all four tiles) and with the recorded spacing; the raw hello and ring header
of both servers are the same bytes; a slow client (40 ms per frame) skips frames in order while the broker keeps its
pace; a second broker on a live socket is refused, both ways; `broker-status` of each tool reads the other's broker,
and both say the same without a broker; a stale socket file is replaced.

Zlib recordings are compatible both ways but not byte-identical (zlib instead of miniz_oxide), so `record`/`import`
with `--zlib > 0` print other sizes; the frames are the same.
