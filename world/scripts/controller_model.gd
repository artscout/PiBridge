# SPDX-License-Identifier: MIT
# The controller as the headset's own software draws it: the render model of the Windows data kit
# (<windata>/steamvr/rendermodels/crystal_controller_<side>/: <name>.json lists the parts and the pose frames, the parts
# are OBJ files with their textures). Placed on the controller's OpenXR grip pose through the model's "openxr_grip"
# frame, so it sits in the hand as in SteamVR. Without the kit: a plain stand-in.
extends Node3D

const ObjLoader = preload("res://scripts/obj_loader.gd")
const Paths = preload("res://scripts/paths.gd")

var side := "left"
var found := false
# The laser's origin and direction relative to the grip pose: the model's "openxr_aim" frame (where SteamVR puts the
# aim of this controller), so the beam leaves the drawn controller's nose; identity until a model is loaded.
var aim_from_grip := Transform3D.IDENTITY


func _init(s: String) -> void:
	side = s
	name = "Model_" + s


func _ready() -> void:
	var dir := Paths.controller_models() + "/crystal_controller_" + side
	var j = JSON.parse_string(Paths.read_text(dir + "/crystal_controller_" + side + ".json"))
	if typeof(j) != TYPE_DICTIONARY or not j.has("components"):
		_stand_in()
		return
	var comps: Dictionary = j["components"]
	var model := Node3D.new()
	for key in comps:
		var c: Dictionary = comps[key]
		if not c.has("filename") or key == "status":
			continue
		var mesh := ObjLoader.load_mesh(dir + "/" + String(c["filename"]))
		if mesh == null:
			continue
		var mi := MeshInstance3D.new()
		mi.mesh = mesh
		mi.transform = _local(c)
		model.add_child(mi)
	if model.get_child_count() == 0:
		_stand_in()
		return
	# the model's own frame relative to the grip pose: the inverse of where the grip is in the model
	model.transform = _local(comps.get("openxr_grip", {})).affine_inverse()
	if comps.has("openxr_aim"):
		aim_from_grip = model.transform * _local(comps["openxr_aim"])
	add_child(model)
	found = true


func _local(c: Dictionary) -> Transform3D:
	var l: Dictionary = c.get("component_local", {})
	var o: Array = l.get("origin", [0, 0, 0])
	var r: Array = l.get("rotate_xyz", [0, 0, 0])
	var b := Basis.from_euler(Vector3(deg_to_rad(r[0]), deg_to_rad(r[1]), deg_to_rad(r[2])), EULER_ORDER_XYZ)
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
