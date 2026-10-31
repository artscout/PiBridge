# SPDX-License-Identifier: MIT
# The controller as SteamVR draws it: a SteamVR render model (<model>/<model>.json lists the parts and the pose frames,
# the parts are OBJ files with their textures). The Pimax Crystal controller's comes from the Windows data kit, the
# Valve Index controller's from SteamVR (Paths.render_model, docs/world.md). Which one: the controller's OpenXR
# interaction profile (set_kind). Placed on the controller's OpenXR grip pose through the model's grip frame, so it sits
# in the hand as in SteamVR. Without the model: a plain stand-in.
extends Node3D

const ObjLoader = preload("res://scripts/obj_loader.gd")
const Paths = preload("res://scripts/paths.gd")

# The render models it knows: the folder (%s = the side), the frames of the OpenXR grip and aim poses in the model
# (the Index model's "grip" and "tip" are the poses Monado gives the Index controller), the parts never drawn
# (state indicators SteamVR shows only now and then).
const KINDS := {
	"crystal": {"model": "crystal_controller_%s", "grip": "openxr_grip", "aim": "openxr_aim", "skip": ["status"]},
	"index": {"model": "valve_controller_knu_1_0_%s", "grip": "grip", "aim": "tip",
		"skip": ["status", "trackpad_touch", "trackpad_scroll_cut", "trackpad_scroll_wheel"]},
}

signal changed # another model (or the stand-in) is in place: the laser follows aim_from_grip

var side := "left"
var kind := ""
var found := false
# The laser's origin and direction relative to the grip pose: the model's aim frame (where SteamVR puts the aim of
# this controller), so the beam leaves the drawn controller's nose; identity until a model is loaded.
var aim_from_grip := Transform3D.IDENTITY


func _init(s: String) -> void:
	side = s
	name = "Model_" + s


func _ready() -> void:
	if kind == "":
		set_kind("crystal")


# The kind for an OpenXR interaction profile ("" while none is bound yet: keep the current one).
static func kind_for_profile(profile: String) -> String:
	if profile == "":
		return ""
	return "index" if profile == "/interaction_profiles/valve/index_controller" else "crystal"


func set_kind(k: String) -> void:
	if k == "" or k == kind:
		return
	kind = k
	for c in get_children():
		remove_child(c) # at once: the new model is the only child from here on
		c.queue_free()
	found = false
	aim_from_grip = Transform3D.IDENTITY
	var spec: Dictionary = KINDS[k]
	var model_name: String = spec["model"] % side
	var dir := Paths.render_model(model_name)
	var j = JSON.parse_string(Paths.read_text(dir + "/" + model_name + ".json")) if dir != "" else null
	if typeof(j) != TYPE_DICTIONARY or not j.has("components"):
		_stand_in()
		changed.emit()
		return
	var comps: Dictionary = j["components"]
	var model := Node3D.new()
	for key in comps:
		var c: Dictionary = comps[key]
		if not c.has("filename") or spec["skip"].has(key):
			continue
		var mesh := ObjLoader.load_mesh(dir + "/" + String(c["filename"]))
		if mesh == null:
			continue
		# the parts are already in the model's space (body.obj is the whole controller, each button sits where it is):
		# component_local is the part's own frame (attach points, animation), never where its mesh goes
		var mi := MeshInstance3D.new()
		mi.mesh = mesh
		model.add_child(mi)
	if model.get_child_count() == 0:
		_stand_in()
		changed.emit()
		return
	# the model's own frame relative to the grip pose: the inverse of where the grip is in the model
	model.transform = _local(comps.get(spec["grip"], {})).affine_inverse()
	if comps.has(spec["aim"]):
		aim_from_grip = model.transform * _local(comps[spec["aim"]])
	add_child(model)
	found = true
	changed.emit()


# A frame of the model ("component_local": origin, rotate_xyz in degrees), composed as Monado composes the same numbers
# for the Index poses (math_quat_from_euler_angles: Rz * Ry * Rx), so the model sits exactly on Monado's grip pose.
static func _local(c: Dictionary) -> Transform3D:
	var l = c.get("component_local", {})
	if typeof(l) != TYPE_DICTIONARY:
		l = {}
	var o: Array = l.get("origin", [0, 0, 0])
	var r: Array = l.get("rotate_xyz", [0, 0, 0])
	var b := Basis.from_euler(Vector3(deg_to_rad(r[0]), deg_to_rad(r[1]), deg_to_rad(r[2])), EULER_ORDER_ZYX)
	return Transform3D(b, Vector3(o[0], o[1], o[2]))


func _stand_in() -> void:
	var mi := MeshInstance3D.new()
	var cap := CapsuleMesh.new()
	cap.radius = 0.022
	cap.height = 0.13
	mi.mesh = cap
	mi.rotation_degrees = Vector3(-70, 0, 0)
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.2, 0.22, 0.26)
	mi.material_override = mat
	add_child(mi)
