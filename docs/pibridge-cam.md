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
is the command line PiBridge's RECORD TRACKING and START TRACKING run (`rec_pimax_cam`, default `pibridge-cam`).

## Tests

The tool's parity tests against the Rust `pimax-cam` (and the broker cross tests Rust <-> C++) were removed with the
oracle once the port was finished (2026-10-22). The library behind the tool is tested by `ctest -R pibridge_cam_`
(`docs/libpibridge-cam.md`).

Zlib recordings are compatible both ways but not byte-identical (zlib instead of miniz_oxide), so `record`/`import`
with `--zlib > 0` print other sizes; the frames are the same.
