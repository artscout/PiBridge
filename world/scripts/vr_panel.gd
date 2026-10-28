# SPDX-License-Identifier: MIT
# A 2D user interface on a quad in the world: a SubViewport with a transparent background (so rounded corners and gaps
# are see-through), drawn unshaded with its alpha; the lasers (Pointer) drive its mouse.
class_name VrPanel
extends Node3D

var viewport := SubViewport.new()
var mesh := MeshInstance3D.new()
var area := Area3D.new()
var size_px := Vector2i(1600, 1000)
var size_m := Vector2(1.6, 1.0)
var _mouse := Vector2.ZERO
var _scroll_cool := 0.0


func _init(content: Control, px: Vector2i, metres: Vector2) -> void:
	size_px = px
	size_m = metres
	viewport.size = px
	viewport.transparent_bg = true
	viewport.disable_3d = true
	viewport.gui_embed_subwindows = true
	viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	viewport.add_child(content)
	content.position = Vector2.ZERO
	content.size = px


func _ready() -> void:
	add_child(viewport)
	var q := QuadMesh.new()
	q.size = size_m
	mesh.mesh = q
	var m := StandardMaterial3D.new()
	m.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	m.cull_mode = BaseMaterial3D.CULL_DISABLED
	m.albedo_texture = viewport.get_texture()
	m.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR_WITH_MIPMAPS
	mesh.material_override = m
	add_child(mesh)
	var shape := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = Vector3(size_m.x, size_m.y, 0.01)
	shape.shape = box
	area.collision_layer = 2
	area.collision_mask = 0
	area.set_meta("panel", self)
	area.add_child(shape)
	add_child(area)


func _process(dt: float) -> void:
	_scroll_cool = maxf(0.0, _scroll_cool - dt)


func to_pixels(global_point: Vector3) -> Vector2:
	var l := to_local(global_point)
	return Vector2((l.x / size_m.x + 0.5) * size_px.x, (0.5 - l.y / size_m.y) * size_px.y)


func pointer_move(global_point: Vector3) -> void:
	var p := to_pixels(global_point)
	var e := InputEventMouseMotion.new()
	e.position = p
	e.global_position = p
	e.relative = p - _mouse
	_mouse = p
	viewport.push_input(e)


func pointer_button(pressed: bool) -> void:
	var e := InputEventMouseButton.new()
	e.position = _mouse
	e.global_position = _mouse
	e.button_index = MOUSE_BUTTON_LEFT
	e.pressed = pressed
	viewport.push_input(e)


func pointer_scroll(dir: float) -> void:
	if _scroll_cool > 0.0:
		return
	_scroll_cool = 0.08
	for pressed in [true, false]:
		var e := InputEventMouseButton.new()
		e.position = _mouse
		e.global_position = _mouse
		e.button_index = MOUSE_BUTTON_WHEEL_UP if dir > 0 else MOUSE_BUTTON_WHEEL_DOWN
		e.pressed = pressed
		viewport.push_input(e)


# The self test's click: move there, press, release.
func click_px(p: Vector2) -> void:
	var e := InputEventMouseMotion.new()
	e.position = p
	e.global_position = p
	viewport.push_input(e)
	_mouse = p
	pointer_button(true)
	pointer_button(false)
