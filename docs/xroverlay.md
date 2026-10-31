# pibridge-xroverlay: the overlay over every app (`tools/xroverlay/`)

`pibridge-xroverlay` is a small OpenXR overlay session (`XR_EXTX_overlay`, placement 10, above every other overlay)
that the Monado session starts and stops. It draws three things fixed in front of your head, and nothing else:

- **The recenter ring.** While the Pimax button is held, Monado's Pimax driver writes the hold progress (0..1) to
  `$XDG_RUNTIME_DIR/pibridge/recenter-progress`. A ring fills 0.7 m in front of you; when it is full, the driver
  recenters. Under SteamVR, `pibridge-overlay` draws the same ring from `pibridge-headtrack`'s progress
  ([OVERLAY.md](OVERLAY.md)).
- **The volume indicator.** For 1.5 s after a volume button: a speaker and a bar with the headset volume, low in
  front of you (0.9 m, 22 cm down). `pibridge-keepalived` changes the volume (the Crystal Super's buttons; the original
  Crystal changes its own) and writes the new percent to `$XDG_RUNTIME_DIR/pibridge/volume-hud`; the file's age says
  how long it shows (`src/volume.*`: 21 frames, 0 to 100 % by 5).
- **The loading dots.** Three dots pulse one after another, 1.2 m in front of you, while
  `$XDG_RUNTIME_DIR/pibridge/loading` exists and is younger than 30 s. The dashboard overlay writes it when a game has
  ended and PiBridge World is starting again, and removes it once an app is in front ([home.md](home.md), "Over a
  game"). So the headset shows that something is coming instead of a black screen.

All pictures are sprite sheets drawn once at start (the ring: 25 fill levels, supersampled; the dots: 24 frames of a
1.2 s cycle, antialiased) and uploaded once into static swapchains. A frame only picks a cell of a sheet, so nothing is
uploaded or drawn per frame, and nothing at all is submitted while neither is shown. The request files are polled:
the progress every frame, the loading file four times a second.

```
pibridge-xroverlay        run (the Monado session starts it with XR_RUNTIME_JSON set)
```

## Code

| Where | What |
|---|---|
| `src/main.cpp` | the overlay session, the two static swapchains, the frame loop |
| `src/loading.*` | the loading dots: the sheet, the pulse, the request file |
| `src/volume.*` | the volume indicator: the sheet, the frame per percent, the file's age |
| `tools/overlay/src/overlay_core.*` | the ring sheet (shared with the SteamVR overlay) |

## Tests

`pibridge_xroverlay_loading`: the pulse walks across the dots, the sheet is antialiased with transparent corners, the
request file counts only while it is fresh. `pibridge_xroverlay_volume`: the frame per percent, the bar grows with it,
the file shows only while fresh.
