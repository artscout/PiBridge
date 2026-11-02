# SPDX-License-Identifier: MIT
# Where things go in PiBridge World (docs/world.md "Layout"): the whole frame (the "rig") is put at the eyes, turned to
# where you look (yaw only), moved forward and scaled by the interface size; in it the dashboard, the bar under it and
# the desktop's monitors. Units are the frame's own (a metre at scale 1); -z is away from you.
extends RefCounted

# the interface sizes: [forward offset in m, scale]; Medium is the default
const UI_SCALES := {"Large": [2.3, 1.0], "Medium": [1.55, 0.75], "Small": [0.85, 0.5], "Miniature": [0.4, 0.3]}
const UI_SCALE_DEFAULT := "Medium"

# the dashboard and the bar (0.007 per pixel of their pictures)
const MAIN_POS := Vector3(0.0, -0.35, 0.0)        # the dashboard
const MAIN_SIZE := Vector2(2.24, 1.40)
const BAR_POS := Vector3(0.0, -1.25, 0.087)       # the bar: 0.087 nearer, tilted 30 degrees up
const BAR_SIZE := Vector2(2.24, 0.336)
const BAR_TILT_DEG := 30.0

# the desktop (the middle monitor a little above the frame's middle; the sides 0.727 nearer, turned 40 degrees to
# you; a monitor is 2.19 wide, or 1.231 high when it stands upright)
const MON_CENTER := Vector3(0.0, 0.065, 0.0005)
const MON_SIDE_X := 2.012
const MON_SIDE_Z := 0.727
const MON_SIDE_YAW_DEG := 40.0
const MON_W := 2.19
const MON_H := 1.231

# the desktop's sliders: its scale and its distance (times the interface size's scale, + = farther)
const ZOOM_MIN := 0.5
const ZOOM_MAX := 5.0
const DISTANCE_MIN := -1.5
const DISTANCE_MAX := 1.5


static func ui_scale(name: String) -> Array:
	return UI_SCALES.get(name, UI_SCALES[UI_SCALE_DEFAULT])


# The rig at the head: the eyes' place, turned to where they look (level), `forward` metres ahead, scaled.
static func rig_transform(head: Transform3D, scale_name: String) -> Transform3D:
	var s: Array = ui_scale(scale_name)
	var fwd := -head.basis.z
	fwd.y = 0
	if fwd.length() < 0.01:
		fwd = Vector3.FORWARD
	fwd = fwd.normalized()
	var basis := Basis.looking_at(fwd, Vector3.UP).scaled(Vector3.ONE * float(s[1]))
	return Transform3D(basis, head.origin + fwd * float(s[0]))


static func bar_transform() -> Transform3D:
	# its face (+z) turned up towards your eyes, the bottom edge nearer
	return Transform3D(Basis(Vector3.RIGHT, deg_to_rad(-BAR_TILT_DEG)), BAR_POS)


# A monitor's size in rig units from its picture's aspect.
static func monitor_size(w: int, h: int) -> Vector2:
	if w <= 0 or h <= 0:
		return Vector2(MON_W, MON_W * 9.0 / 16.0)
	var a := float(w) / float(h)
	return Vector2(MON_W, MON_W / a) if a > 1.0 else Vector2(MON_H * a, MON_H)


# The places of the shown monitors (`order`: their indices, left to right on the desktop): one
# in the middle; two: the first in the middle, the second on the right; three: the first in the middle, the second on
# the left, the third on the right. [{monitor, transform}]
static func monitor_places(order: Array) -> Array:
	var out := []
	var side := [null, "right", "left"] if order.size() >= 3 else [null, "right"]
	for i in mini(order.size(), 3):
		var t := Transform3D(Basis(), MON_CENTER)
		# turned to you: the right one's face (+z) towards -x, the left one's towards +x
		if side[i] == "right":
			t = Transform3D(Basis(Vector3.UP, deg_to_rad(-MON_SIDE_YAW_DEG)), Vector3(MON_SIDE_X, MON_CENTER.y, MON_SIDE_Z))
		elif side[i] == "left":
			t = Transform3D(Basis(Vector3.UP, deg_to_rad(MON_SIDE_YAW_DEG)), Vector3(-MON_SIDE_X, MON_CENTER.y, MON_SIDE_Z))
		out.append({"monitor": order[i], "transform": t})
	return out


# The desktop's container: scaled about the rig's centre, moved away by the distance slider (times the UI scale).
static func zoom_transform(zoom: float, distance: float, scale_name: String) -> Transform3D:
	var s: Array = ui_scale(scale_name)
	return Transform3D(Basis().scaled(Vector3.ONE * clampf(zoom, ZOOM_MIN, ZOOM_MAX)),
		Vector3(0, 0, -clampf(distance, DISTANCE_MIN, DISTANCE_MAX) * float(s[1])))
