# linux-vulkan-quadviews-openxr-layer (qv-vulkan)

Quad views for Vulkan games on Linux.

An OpenXR API layer that gives Vulkan games **quad views** (`XR_VARJO_quad_views`) and **eye-tracked foveated
rendering** (`XR_VARJO_foveated_rendering`) on SteamVR on Linux. Plus a Qt settings app.

A game with quad views renders four views: a wide view per eye at low pixel density, and a small focus view per eye
at full density where you look. The layer composes the two views of each eye into one image and hands the runtime
an ordinary stereo frame. With eye tracking the focus follows the gaze; without it, it sits in the middle of the
field (fixed foveated rendering), which already saves most of the pixels.

First target: the native Linux build of Falcon BMS (Vulkan, quad views in its OpenXR mode 3).

**Status:** works in a headset: Falcon BMS (native Linux, Vulkan) under SteamVR on a Pimax Crystal Super, with eye
tracking (the gaze through SteamVR's `XR_EXT_eye_gaze_interaction`), October 2026. The end-to-end tests run against a
test runtime (see Tests)

## Why another layer

[Quad-Views-Foveated](https://github.com/mbucchia/Quad-Views-Foveated) (Matthieu Bucchianeri) and
[VectorXR](https://github.com/DienerTech/vectorxr) do this on Windows, and both compose only Direct3D 11 games. 
This layer is written from scratch for Vulkan; the design follows Quad-Views-Foveated, with these differences:

| | qv-vulkan |
|---|---|
| Graphics API | Vulkan (the game's own device and queue) |
| Copies | none: the views are sampled straight from the game's swapchain images, inside their sub-image rectangles |
| Wide views | Catmull-Rom (bicubic) upscaling, or bilinear |
| Focus edge | smooth blend over a configurable band, starting at a configurable share of the focus (as Pimax's "focus transition opacity") |
| Focus at the edge of the field | keeps its size and slides back inside (as VectorXR does), instead of shrinking |
| Gaze | One-Euro filter with a saccade snap: steady at fixation, no lag on saccades; holds the last gaze through blinks |
| Gaze parallax | the gaze point is projected into each eye from its real position and orientation (canted displays) |
| The game's own gaze action | kept: the layer adds its binding to the game's suggestion instead of replacing it |
| Image hand-off | the game's images are released to the runtime only after the composition read them (no use after release) |
| Settings | live: the settings app's changes reach a running game within half a second (except the two densities) |
| Profiles | per game (`[app:NAME]`, `[exe:NAME]` sections) |
| Diagnostics | log, a status file for the settings app, an in-headset overlay of the focus area and the gaze |

## Build and install

Needs CMake 3.22+, a C++20 compiler, the Vulkan and OpenXR headers, `glslangValidator`, and Qt 6 Widgets for the
settings app (`-DQV_BUILD_GUI=OFF` without it).

```sh
git clone git@github.com:artscout/linux-vulkan-quadviews-openxr-layer.git
cd linux-vulkan-quadviews-openxr-layer
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/.local
cmake --build build
ctest --test-dir build
cmake --install build
```

This installs the layer to `~/.local/lib/qv-vulkan/`, its manifest to
`~/.local/share/openxr/1/api_layers/implicit.d/` (an implicit layer: every OpenXR app loads it; it does nothing for
apps that do not ask for quad views) and `qv-vulkan-settings` with a desktop entry.

Turn it off: the switch in the settings app (it renames the manifest), or `DISABLE_XR_APILAYER_ARTSCOUT_QUAD_VIEWS_VULKAN=1`
for one game. Uninstall: delete the three installed files.

## Settings

`~/.config/qv-vulkan/settings.conf`, written by the settings app. All keys, with their defaults, are in
[layer/src/settings.h](layer/src/settings.h). In short:

| Key | Default | |
|---|---|---|
| `peripheral_density` | 0.40 | pixels of the wide views per recommended pixel, 0.05 .. 1.5 (game restart) |
| `focus_density` | 1.00 | pixels of the focus views and of the composed image (game restart) |
| `tracking` | eye | `eye`: follow the gaze when the runtime has eye tracking; `fixed` |
| `gaze_width`, `gaze_height` | 0.35 | focus size with eye tracking, share of the eye's field |
| `fixed_width`, `fixed_height` | 0.50, 0.45 | focus size without eye tracking |
| `offset_x_deg`, `offset_y_deg` | 0 | move the focus, degrees |
| `transition` | 0.20 | blend band at the focus edge, share of the focus size ("Blend band") |
| `edge_opacity` | 0.50 | share of the focus at the outer edge of the band (0 fades from the periphery, 1 hard edge) |
| `sharpen` | 0.60 | contrast-adaptive sharpening of the focus |
| `peripheral_filter` | bicubic | or `bilinear` |
| `gaze_filter`, `gaze_hold_ms` | on, 600 | gaze smoothing, hold through blinks |
| `debug_overlay` | off | tint the focus area and mark the gaze in the headset |

## Eye tracking

The layer reads the gaze through the runtime's `XR_EXT_eye_gaze_interaction`. Monado offers it for headsets whose
driver has eye tracking. SteamVR on Linux offers it only where a driver feeds it (tested: the Crystal Super's eye
tracker through CustomHeadsetOpenVR); without it the layer uses the fixed area. The game's own use of the gaze keeps working.

## On Monado

Monado (PiBridge's Monado backend) has `XR_VARJO_quad_views` itself, emulated in its compositor: the game renders four
views and Monado puts the focus views into the wide ones at the field each was rendered with. It has no
`XR_VARJO_foveated_rendering`, so on its own the focus stays in the middle. With such a runtime the layer runs in its
**native mode**: it does not compose; the four views go to Monado unchanged, and the layer only

- answers `XR_VARJO_foveated_rendering` (supported when the runtime has eye gaze),
- reads the gaze through `XR_EXT_eye_gaze_interaction` (attaching its own action set when the game has none),
- hands out focus views centred on the gaze in `xrLocateViews` (sizes, offsets, the gaze filter: the settings below),
- sizes the four views from the densities in `xrEnumerateViewConfigurationViews`.

Frames, swapchains and masks go straight to Monado. The end-to-end test `native_quad_views_focus_follows_the_gaze`
runs it against the fake runtime in that role (`QV_FAKE_NATIVE_QUAD=1`).

## Logs

`~/.local/state/qv-vulkan/qv-vulkan.log` (one per game process) and `status.txt` next to it (what the settings app
shows under "Last game"). `QV_VULKAN_LOG_STDERR=1` also prints to stderr.

## Tests

`ctest` runs:

- `common`: settings, profiles, the tangent-space geometry, the gaze filter;
- `e2e`: a game that uses quad views like Falcon BMS (Vulkan, foveated rendering, its own gaze action), the real
  OpenXR loader, the layer from the build tree, and a fake runtime with a scripted gaze that reads back what reaches
  it. It checks the views' sizes, the focus position with and without the gaze, and the composed pixels (focus,
  periphery, where the blend is half way), in UNORM and sRGB; and that a stereo game passes through untouched;
- `e2e_validation`: the same under the Khronos validation layer with synchronisation checks; any message fails.

The end-to-end tests run on lavapipe (Mesa's CPU Vulkan), so they never touch the GPU.

## Licence

MIT (see LICENSE). The design follows Quad-Views-Foveated (MIT); no code was copied from it or from VectorXR
(MPL-2.0). The focus sharpening is a contrast-adaptive sharpening in the spirit of AMD FidelityFX CAS.
