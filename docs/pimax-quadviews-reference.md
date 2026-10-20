# Pimax EVO quad views settings: reference

Read from the Pimax EVO 2.0 beta client (Electron, `PimaxClient/pimaxui/resources/app.asar`) and its log
(`AppData/Roaming/PimaxClient/logs/main.log`) on 2026-10-19, for the app's quad views page. Facts only, no code taken.

## Presets
The client stores only the level, `runtime_quadviews_rendering_level` (`globalAppInt`): PERFORMANCE = 0,
BALANCED = 1, QUALITY = 2, EXTREME = 3, CUSTOM = 5 (4 = a legacy custom value). The values of levels 0-3 are not in the
client: the Pimax runtime applies them. Clicking a preset in the client writes nothing but the level.

## Controls, ranges, defaults and stored values (`ui` = the number shown, `raw` = the value in the config)
| Control | Key | Range, default | raw |
|---|---|---|---|
| Quad views on | `runtime_enable_openxr_quadviews_fr` | 0/1 | |
| Center resolution | `runtime_quadviews_gaze_resolution_scale` | 50-300 %, 175 | (ui - 50) / 250 (175 % = 0.5) |
| Edge resolution | `runtime_quadviews_periphery_resolution_scale` | 1-100 %, 10 | (ui - 1) / 99 (10 % = 0.0909) |
| Mode | `piplay_quadviews_fine_type` | 0 quick, 1 detail | |
| Quick horizontal / vertical FOV | `runtime_quadviews_fine_fov_{left,right}_{left,right,up,down}` (+ `piplay_quadviews_quick_*_gaze_fov_scale`) | 5-90 %, 33 | ui / 100 |
| Detail mode, each edge to the centre | the same eight runtime keys (+ `piplay_quadviews_fine_fov_*_to_center`) | 5-50 %, 33 (inner edges 10) | ui / 50, 50 % = 0.99 |
| Vertical offset | `runtime_quadviews_vertical_focus_offset` | -100..100, -10 | -0.5 + (ui + 100) / 200 |
| Blend area | `runtime_quadviews_focus_blend_area` | 1-50 %, 5 | 0.05 |
| Transition type | `runtime_quadviews_focus_transition_type` | off -1, linear 0, alpha 1 | |
| Blend curve | `runtime_quadviews_focus_blend_mode` | Linear 0, Power2 1, Sqrt 2, Sigmoid 3 | |
| Alpha opacity | `runtime_quadviews_focus_transition_opacity_percent` | 5-95 %, 50 | 0.5 |

## Mapping to our layer (`~/.config/qv-vulkan/settings.conf`)
Edge opacity = alpha opacity; blend band = blend area (ours is a share of the focus size on each side); gaze area
width/height = quick FOV; vertical offset = `offset_y_deg` (different unit: ours is degrees). Pimax's center/edge
resolution are slider positions, not pixel multipliers: our `focus_density` / `peripheral_density` stay pixel
multipliers per recommended pixel; the app shows them in its own percent.
