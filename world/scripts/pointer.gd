# SPDX-License-Identifier: MIT
# A controller's laser: a ray from the aim pose; on a VrPanel it moves the panel's mouse, the trigger clicks, the
# thumbstick scrolls. The beam ends at what it hits. A desktop screen (desktop.gd) takes A / X as its right button and
# hears when the laser leaves it.
extends Node3D

var controller: XRController3D
var ray := RayCast3D.new()
var beam := MeshInstance3D.new()
var dot := MeshInstance3D.new()
var _down := false
var _panel = null
var active := true
var tracked := true # main.gd: the controller is tracked (shown); a hidden one does not point or click
var on_screen := false # the laser is on a desktop screen (desktop.gd): main.gd hands it to the overlay to draw
var hit_point := Vector3.ZERO


func _init(c: XRController3D) -> void:
	controller = c
	name = "Pointer"


func _ready() -> void:
	ray.target_position = Vector3(0, 0, -6)
	ray.collide_with_areas = true
	ray.collide_with_bodies = false
	ray.collision_mask = 2
	add_child(ray)
	var bm := BoxMesh.new()
	bm.size = Vector3(0.002, 0.002, 1)
	beam.mesh = bm
	var m := StandardMaterial3D.new()
	m.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	m.albedo_color = Color(0.45, 0.85, 1.0, 0.55)
	beam.material_override = m
	add_child(beam)
	var sm := SphereMesh.new()
	sm.radius = 0.006
	sm.height = 0.012
	dot.mesh = sm
	var dm := StandardMaterial3D.new()
	dm.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	dm.albedo_color = Color(0.6, 0.95, 1.0)
	dot.material_override = dm
	dot.top_level = true
	add_child(dot)
	controller.button_pressed.connect(_on_pressed)
	controller.button_released.connect(_on_released)


func _process(_dt: float) -> void:
	visible = active and tracked and controller.get_has_tracking_data()
	if not visible:
		on_screen = false
		if _panel != null and _down:
			_panel.pointer_button(false)
		_leave(_panel)
		_down = false
		_panel = null
		return
	ray.force_raycast_update()
	var length := 2.5
	var hit = null
	if ray.is_colliding():
		var a = ray.get_collider()
		hit = a.get_meta("panel") if a and a.has_meta("panel") else null
		length = global_position.distance_to(ray.get_collision_point())
	beam.scale = Vector3(1, 1, length)
	beam.position = Vector3(0, 0, -length / 2)
	dot.visible = hit != null
	on_screen = hit != null and hit.has_method("pointer_right")
	if hit != null:
		hit_point = ray.get_collision_point()
		dot.global_position = ray.get_collision_point()
		hit.pointer_move(ray.get_collision_point())
		var s := controller.get_vector2("primary")
		if absf(s.y) > 0.5:
			hit.pointer_scroll(signf(s.y))
	elif _panel != null and _down:
		_panel.pointer_button(false)
		_down = false
	if hit != _panel:
		_leave(_panel)
	_panel = hit


func _leave(p) -> void:
	if p != null and is_instance_valid(p) and p.has_method("pointer_leave"):
		p.pointer_leave()


func _on_pressed(n: String) -> void:
	if n == "trigger_click" and _panel != null:
		_down = true
		_panel.pointer_button(true)
	elif n == "ax_button" and _panel != null and _panel.has_method("pointer_right"):
		_panel.pointer_right(true)


func _on_released(n: String) -> void:
	if n == "ax_button" and _panel != null and _panel.has_method("pointer_right"):
		_panel.pointer_right(false)
	if n == "trigger_click" and _down:
		_down = false
		if _panel != null:
			_panel.pointer_button(false)
