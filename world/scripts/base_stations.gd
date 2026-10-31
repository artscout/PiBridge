# SPDX-License-Identifier: MIT
# The Lighthouse base stations where they stand, as SteamVR shows them (the Monado backend in the Lighthouse mode,
# pibridge.conf show_base_stations = on, the PiBridge app's Tracking card; never in the inside-out mode, never from a
# file an earlier session left). Monado's steamvr_lh writes their poses in
# the stage, the space of this world's origin, to $XDG_RUNTIME_DIR/pibridge/monado/basestations ("<serial> px py pz
# qx qy qz qw" a line, once a second); the model is SteamVR's lh_basestation_valve_gen2 (Paths.render_model), else a
# plain box.
extends Node3D

const KeyConf = preload("res://scripts/keyconf.gd")
const ObjLoader = preload("res://scripts/obj_loader.gd")
const Paths = preload("res://scripts/paths.gd")

const MODELS := ["lh_basestation_valve_gen2", "lh_basestation_vive"]

var _stations := {} # serial -> Node3D
var _mesh: Mesh = null
var _next := 0.0


func _init() -> void:
	name = "BaseStations"


static func file_path() -> String:
	return Paths.status_dir() + "/monado/basestations"


# The setting, and only in the Lighthouse mode (the inside-out mode has no base stations to show).
static func wanted() -> bool:
	var conf := KeyConf.new(Paths.pibridge_conf())
	return conf.value("show_base_stations", "off") == "on" and conf.value("tracking_mode", "") == "lighthouse"


# Monado rewrites the file once a second; one left from an earlier Lighthouse session is older and not shown.
static func fresh(path: String, now_unix: float) -> bool:
	return FileAccess.file_exists(path) and now_unix - float(FileAccess.get_modified_time(path)) < 5.0


# "<serial> px py pz qx qy qz qw" lines -> {serial: Transform3D}; malformed lines are left out.
static func parse(text: String) -> Dictionary:
	var out := {}
	for raw in text.split("\n"):
		var p := raw.strip_edges().split(" ", false)
		if p.size() != 8:
			continue
		var q := Quaternion(float(p[4]), float(p[5]), float(p[6]), float(p[7]))
		if not q.is_finite() or q.length() < 0.5:
			continue
		out[p[0]] = Transform3D(Basis(q.normalized()), Vector3(float(p[1]), float(p[2]), float(p[3])))
	return out


func _process(dt: float) -> void:
	_next -= dt
	if _next > 0.0:
		return
	_next = 1.0
	var path := file_path()
	var poses := parse(Paths.read_text(path)) if wanted() and fresh(path, Time.get_unix_time_from_system()) else {}
	for serial in _stations.keys():
		if not poses.has(serial):
			_stations[serial].queue_free()
			_stations.erase(serial)
	for serial in poses:
		if not _stations.has(serial):
			_stations[serial] = _station()
			add_child(_stations[serial])
		_stations[serial].transform = poses[serial]


func _station() -> MeshInstance3D:
	if _mesh == null:
		for m in MODELS:
			var dir := Paths.render_model(m)
			if dir != "":
				_mesh = ObjLoader.load_mesh(dir + "/" + m + ".obj")
				if _mesh != null:
					break
		if _mesh == null:
			var box := BoxMesh.new()
			box.size = Vector3(0.08, 0.08, 0.06)
			_mesh = box
	var mi := MeshInstance3D.new()
	mi.mesh = _mesh
	return mi
