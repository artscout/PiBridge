# SPDX-License-Identifier: MIT
# The desktop in PiBridge World (docs/desktop.md): your monitors at the frame's places (layout.gd: one in
# the middle, the second on the right, the third on the left; this node is the container main.gd scales and moves with
# the zoom and distance sliders), from pibridge-desktop's
# shared file $XDG_RUNTIME_DIR/pibridge/desktop.shm (common/pibridge_desktop.h: the offsets below are checked there by
# static_assert). The world writes this side of the file (the beat, the monitors shown, the laser's pointer and the
# wheel) and the places of the screens and its lasers to $XDG_RUNTIME_DIR/pibridge/world-desktop every frame: PiBridge's
# dashboard overlay draws the pictures there as quad layers (in its own process: a 2560 x 1440 picture per monitor up to
# 120 times a second would stall the world's main thread). The screens here are only where the lasers hit. With
# `draw_pictures` (the self test, a look without the overlay) a thread reads the pictures and the world shows them. The lasers (pointer.gd) drive a screen as they drive a panel: the trigger is
# the left button, A / X the right one, the stick scrolls.
extends Node3D

const Paths = preload("res://scripts/paths.gd")
const Layout = preload("res://scripts/layout.gd")

const HEADER_BYTES := 8192
const MAX_MONITORS := 4
const SLOT_BYTES := 3840 * 2160 * 4
const OFF_SERVICE_BEAT := 8
const OFF_STATE := 20
const OFF_TEXT_SEQ := 24
const OFF_TEXT := 32
const OFF_LAYOUT_SEQ := 288
const OFF_MONITOR_COUNT := 296
const OFF_MONITORS := 304
const MONITOR_BYTES := 64
const OFF_CLIENT_BEAT := 560
const OFF_WANT := 572
const OFF_POINTER := 576
const OFF_WHEEL := 584
const STATE_RUNNING := 2

signal changed # the monitors or the state's words changed (the bar follows)

var shown := false
var autostart := true             # start pibridge-desktop when it does not answer (off in the self test)
var draw_pictures := false        # the world shows the pictures itself (else the dashboard overlay does)
var rays: Array = []              # [[from, to]]: the lasers on a screen this frame (main.gd), for the overlay to draw
var mask := 0                    # the monitors shown (0: all, until they are known)
var monitors: Array = []         # [{index, x, y, lw, lh, w, h, name}]
var words := ""                  # the state for the bar
var screens := {}                # monitor index -> Screen
var _writer: FileAccess
var _thread: Thread
var _mutex := Mutex.new()
var _run := true
var _want_thread := 0            # what the thread reads (set under the mutex)
var _pending := {}               # monitor -> Image (from the thread)
var _seq := {}                   # monitor -> the last frame sequence read (thread)
var _service_beat := 0
var _service_seen := -1000.0     # when its beat last moved (s, this clock)
var _started_at := -1000.0
var _now := 0.0
var _pointer := 0                # packed, as packPointer
var _wheel := 0


# The pointer as common/pibridge_desktop.h packs it: x + 1 in bits 0..15 (0: off), y in 16..31, buttons 32..39,
# the monitor 40..42.
static func pack_pointer(on: bool, monitor: int, x: int, y: int, buttons: int) -> int:
	var v := (buttons & 0xff) << 32
	if on:
		v |= (mini(x, 0xfffe) + 1) | (mini(y, 0xffff) << 16) | ((monitor & 7) << 40)
	return v


static func shm_path() -> String:
	return Paths.status_dir() + "/desktop.shm"


# The monitors in a header's bytes (the layout's lock is checked by the caller).
static func parse_monitors(h: PackedByteArray) -> Array:
	var out := []
	var n := mini(h.decode_u32(OFF_MONITOR_COUNT), MAX_MONITORS)
	for i in n:
		var b := OFF_MONITORS + i * MONITOR_BYTES
		var name := h.slice(b + 32, b + 64).get_string_from_utf8()
		out.append({"index": i, "w": h.decode_u32(b + 8), "h": h.decode_u32(b + 12), "x": h.decode_s32(b + 16),
			"y": h.decode_s32(b + 20), "lw": h.decode_u32(b + 24), "lh": h.decode_u32(b + 28), "name": name})
	return out


static func world_desktop_path() -> String:
	return Paths.status_dir() + "/world-desktop"


func _ready() -> void:
	if draw_pictures:
		_thread = Thread.new()
		_thread.start(_read_loop)


func _exit_tree() -> void:
	_run = false
	if _thread != null and _thread.is_started():
		_thread.wait_to_finish()


func set_shown(on: bool) -> void:
	shown = on
	for s in screens.values():
		s.set_active(on)
	if on:
		_place()
	else:
		DirAccess.remove_absolute(world_desktop_path()) # the overlay stops drawing them
	changed.emit()


func show_monitors(m: int) -> void:
	mask = m
	_place()
	changed.emit()


func all_mask() -> int:
	var m := 0
	for mi in monitors:
		m |= 1 << int(mi["index"])
	return m


func _process(dt: float) -> void:
	_now += dt
	if not shown:
		return
	if _writer == null:
		if FileAccess.file_exists(shm_path()):
			_writer = FileAccess.open(shm_path(), FileAccess.READ_WRITE)
		if _writer == null:
			_start_service()
			_set_words("Starting the desktop...")
			return
	# the other side's fields, read through a handle of their own each frame: a buffered handle keeps old bytes (a
	# seek inside what it has read does not read again)
	var reader := FileAccess.open(shm_path(), FileAccess.READ)
	var h := reader.get_buffer(600) if reader != null else PackedByteArray()
	reader = null
	# the service: alive while its beat moves; started when it does not answer (it ends itself when nobody looks)
	var beat := h.decode_u64(OFF_SERVICE_BEAT) if h.size() >= 600 else 0
	if beat != _service_beat:
		_service_beat = beat
		_service_seen = _now
	var up := _now - _service_seen < 2.0
	if not up:
		_start_service()
	# this side: the beat (any clock: the service looks at its change), the monitors, the pointer, the wheel
	var want := mask if mask != 0 else (1 << MAX_MONITORS) - 1
	_writer.seek(OFF_CLIENT_BEAT)
	_writer.store_64(Time.get_ticks_usec() * 1000 + 1)
	_writer.seek(OFF_WANT)
	_writer.store_32(want)
	_writer.store_64(_pointer)
	_writer.store_64(_wheel)
	_writer.flush()
	_mutex.lock()
	_want_thread = want
	var pending := _pending
	_pending = {}
	_mutex.unlock()
	# the layout and the words
	var mons := []
	if up and h.size() >= 600 and h.decode_u64(OFF_LAYOUT_SEQ) % 2 == 0:
		mons = parse_monitors(h)
	var state := h.decode_u32(OFF_STATE) if h.size() >= 600 else 0
	var text := h.slice(OFF_TEXT, OFF_TEXT + 256).get_string_from_utf8() if h.size() >= 600 else ""
	if not up:
		_set_words("Starting the desktop...")
	elif state == STATE_RUNNING:
		_set_words("Desktop: %s. Trigger: click, A / X: right click, stick: scroll." % text)
	else:
		_set_words("Desktop: " + text)
	if up and _layout_differs(mons): # the last known monitors stay while the service does not answer
		monitors = mons
		if mask != 0 and (mask & all_mask()) == 0:
			mask = 0
		_place()
		changed.emit()
	for k in pending.keys():
		if screens.has(k):
			screens[k].set_picture(pending[k])
	_write_places()


# The screens' places in the world (= the stage) and the lasers on them, for the overlay that draws the pictures.
func _write_places() -> void:
	var t := ""
	for k in screens.keys():
		var s: Screen = screens[k]
		if not s.is_inside_tree():
			continue
		var g := s.global_transform
		var sc := g.basis.get_scale()
		var q := g.basis.orthonormalized().get_rotation_quaternion()
		t += "screen %d %f %f %f %f %f %f %f %f %f\n" % [k, g.origin.x, g.origin.y, g.origin.z, q.x, q.y, q.z, q.w,
			s.size_m.x * sc.x, s.size_m.y * sc.y]
	for r in rays:
		t += "ray %f %f %f %f %f %f\n" % [r[0].x, r[0].y, r[0].z, r[1].x, r[1].y, r[1].z]
	Paths.write_text(world_desktop_path(), t)


func _set_words(w: String) -> void:
	if w != words:
		words = w
		changed.emit()


func _layout_differs(mons: Array) -> bool:
	if mons.size() != monitors.size():
		return true
	for i in mons.size():
		for key in ["x", "y", "lw", "lh", "w", "h"]:
			if mons[i][key] != monitors[i][key]:
				return true
	return false


func _start_service() -> void:
	if not autostart or _now - _started_at < 5.0:
		return
	_started_at = _now
	var tool := Paths.tool("pibridge-desktop")
	if tool == "":
		_set_words("pibridge-desktop is not installed.")
		return
	# on its own (setsid): it is one at a time by itself and ends itself when nobody looks
	OS.execute("sh", ["-c", "setsid -f '%s' run >/dev/null 2>&1" % tool])


# The shown monitors at the frame's places, in their desktop order (left to right).
func _place() -> void:
	var shown_mons := []
	var m := mask if mask != 0 else all_mask()
	for mi in monitors:
		if (m & (1 << int(mi["index"]))) != 0 and int(mi["lw"]) > 0 and int(mi["lh"]) > 0:
			shown_mons.append(mi)
	shown_mons.sort_custom(func(a, b): return a["x"] < b["x"] if a["x"] != b["x"] else a["y"] < b["y"])
	for k in screens.keys():
		if not shown_mons.any(func(mi): return int(mi["index"]) == k):
			screens[k].queue_free()
			screens.erase(k)
	var by_index := {}
	for mi in shown_mons:
		by_index[int(mi["index"])] = mi
	for place in Layout.monitor_places(shown_mons.map(func(mi): return int(mi["index"]))):
		var k: int = place["monitor"]
		var mi: Dictionary = by_index[k]
		if not screens.has(k):
			var sc := Screen.new(self, k)
			add_child(sc)
			screens[k] = sc
		var s: Screen = screens[k]
		s.set_size(Layout.monitor_size(int(mi["lw"]), int(mi["lh"])), Vector2i(int(mi["w"]), int(mi["h"])))
		s.transform = place["transform"]
		s.set_active(shown)


func set_pointer(on: bool, monitor: int, x: int, y: int, buttons: int) -> void:
	_pointer = pack_pointer(on, monitor, x, y, buttons)


func add_wheel(notches: int) -> void:
	_wheel += 120 * notches


# ---------------------------------------------------------------- the reading thread

func _read_loop() -> void:
	while _run:
		OS.delay_msec(12)
		_mutex.lock()
		var want := _want_thread
		_mutex.unlock()
		if want == 0 or not FileAccess.file_exists(shm_path()):
			continue
		# a new handle for every look: a buffered one returns the old sequence numbers (a seek inside what it has
		# read does not read again), and the pictures never change
		var f := FileAccess.open(shm_path(), FileAccess.READ)
		if f == null:
			continue
		for i in MAX_MONITORS:
			if (want & (1 << i)) == 0:
				continue
			var b := OFF_MONITORS + i * MONITOR_BYTES
			f.seek(b)
			var s1 := f.get_64()
			if s1 == 0 or s1 % 2 == 1 or s1 == _seq.get(i, 0):
				continue
			var w := f.get_32()
			var h := f.get_32()
			if w == 0 or h == 0 or w > 3840 or h > 2160:
				continue
			f.seek(HEADER_BYTES + SLOT_BYTES * i)
			var px := f.get_buffer(w * h * 4)
			f.seek(b)
			if f.get_64() != s1 or px.size() != w * h * 4:
				continue # torn: the next look tries again
			_seq[i] = s1
			var img := Image.create_from_data(w, h, false, Image.FORMAT_RGBA8, px) # BGRA: the shader swaps
			img.generate_mipmaps()
			_mutex.lock()
			_pending[i] = img
			_mutex.unlock()


# ---------------------------------------------------------------- one monitor's screen

class Screen extends Node3D:
	var owner_desk
	var monitor := 0
	var size_m := Vector2(1.3, 0.73)
	var size_px := Vector2i(2560, 1440)
	var mesh := MeshInstance3D.new()
	var area := Area3D.new()
	var shape := CollisionShape3D.new()
	var tex: ImageTexture
	var mat := ShaderMaterial.new()
	var _px := Vector2i.ZERO
	var _left := false
	var _right := false
	var _scroll_cool := 0.0

	func _init(d, m: int) -> void:
		owner_desk = d
		monitor = m
		mesh.visible = d.draw_pictures # else the overlay draws the picture here
		var sh := Shader.new()
		sh.code = """shader_type spatial;
render_mode unshaded, cull_disabled;
uniform sampler2D picture : source_color, filter_linear_mipmap;
uniform bool has_picture = false;
void fragment() {
	vec4 c = texture(picture, UV);
	ALBEDO = has_picture ? c.bgr : vec3(0.02, 0.025, 0.035);
}
"""
		mat.shader = sh
		mesh.material_override = mat
		mesh.mesh = QuadMesh.new()
		add_child(mesh)
		area.collision_layer = 2
		area.collision_mask = 0
		area.set_meta("panel", self)
		shape.shape = BoxShape3D.new()
		area.add_child(shape)
		add_child(area)

	func _process(dt: float) -> void:
		_scroll_cool = maxf(0.0, _scroll_cool - dt)

	# Shown and taking the lasers, or neither: a hidden screen must not catch the laser (the first monitor stands where
	# the dashboard is, a little nearer) nor send clicks to the desktop.
	func set_active(on: bool) -> void:
		visible = on
		area.collision_layer = 2 if on else 0
		if not on:
			pointer_leave()

	func set_size(m: Vector2, px: Vector2i) -> void:
		size_m = m
		if px.x > 0 and px.y > 0:
			size_px = px
		(mesh.mesh as QuadMesh).size = m
		(shape.shape as BoxShape3D).size = Vector3(m.x, m.y, 0.01)

	func set_picture(img: Image) -> void:
		if tex == null or tex.get_width() != img.get_width() or tex.get_height() != img.get_height():
			tex = ImageTexture.create_from_image(img)
			mat.set_shader_parameter("picture", tex)
			mat.set_shader_parameter("has_picture", true)
		else:
			tex.update(img)
		size_px = Vector2i(img.get_width(), img.get_height())

	func _send() -> void:
		owner_desk.set_pointer(true, monitor, _px.x, _px.y, (1 if _left else 0) | (2 if _right else 0))

	# the lasers (pointer.gd), as on a panel
	func pointer_move(global_point: Vector3) -> void:
		var l := to_local(global_point)
		_px = Vector2i(clampi(int((l.x / size_m.x + 0.5) * size_px.x), 0, size_px.x - 1),
			clampi(int((0.5 - l.y / size_m.y) * size_px.y), 0, size_px.y - 1))
		_send()

	func pointer_button(pressed: bool) -> void:
		_left = pressed
		_send()

	func pointer_right(pressed: bool) -> void:
		_right = pressed
		_send()

	func pointer_leave() -> void:
		_left = false
		_right = false
		owner_desk.set_pointer(false, 0, 0, 0, 0)

	func pointer_scroll(dir: float) -> void:
		if _scroll_cool > 0.0:
			return
		_scroll_cool = 0.08
		owner_desk.add_wheel(1 if dir > 0 else -1)
