# SPDX-License-Identifier: MIT
# PiBridge World's bar (docs/world.md): always there under the dashboard, at the frame's place for it (layout.gd BAR_*:
# 2240 x 336 pixels at 0.007 per pixel). Desktop (your monitors, desktop.gd), the monitors' choice 1 / 2 / All and the
# desktop's two sliders, Zoom and Distance, while the desktop is shown;
# Record, Room setup and Recenter. The icons are drawn in code (no picture files), in the dashboard's colours.
extends PanelContainer

const Layout = preload("res://scripts/layout.gd")

signal desktop_toggled(on: bool)
signal monitors_chosen(mask: int)
signal zoom_changed(zoom: float, distance: float)
signal record_pressed
signal room_setup_pressed
signal recenter_pressed

const ICON := 72
const SIZE_PX := Vector2i(2240, 336)

var desktop_btn := Button.new()
var record_btn := Button.new()
var words := Label.new()
var zoom := HSlider.new()
var distance := HSlider.new()
var _mons := HBoxContainer.new()
var _sliders := HBoxContainer.new()
var _mon_count := -1
var _mask := 0
var _recording := false


func _ready() -> void:
	var s := StyleBoxFlat.new()
	s.bg_color = Color(0.04, 0.05, 0.08, 0.9)
	s.set_corner_radius_all(36)
	s.border_color = Color(0.27, 0.36, 0.49, 0.6)
	s.set_border_width_all(3)
	s.content_margin_left = 28
	s.content_margin_right = 28
	s.content_margin_top = 18
	s.content_margin_bottom = 18
	add_theme_stylebox_override("panel", s)
	var v := VBoxContainer.new()
	v.alignment = BoxContainer.ALIGNMENT_CENTER
	v.add_theme_constant_override("separation", 14)
	add_child(v)
	var h := HBoxContainer.new()
	h.add_theme_constant_override("separation", 18)
	v.add_child(h)
	desktop_btn.toggle_mode = true
	_style(desktop_btn, "Desktop", icon("desktop"), 330)
	desktop_btn.toggled.connect(func(on): desktop_toggled.emit(on))
	h.add_child(desktop_btn)
	h.add_child(_mons)
	_mons.add_theme_constant_override("separation", 12)
	var gap := Control.new()
	gap.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	h.add_child(gap)
	_style(record_btn, "Record", icon("record"), 320)
	record_btn.pressed.connect(func(): record_pressed.emit())
	h.add_child(record_btn)
	var room := Button.new()
	_style(room, "Room setup", icon("room"), 380)
	room.pressed.connect(func(): room_setup_pressed.emit())
	h.add_child(room)
	var rc := Button.new()
	_style(rc, "Recenter", icon("recenter"), 340)
	rc.pressed.connect(func(): recenter_pressed.emit())
	h.add_child(rc)
	# the second row while the desktop is shown: the sliders, then the desktop's words
	_sliders.add_theme_constant_override("separation", 18)
	_sliders.visible = false
	v.add_child(_sliders)
	_slider_row(_sliders, "Zoom", zoom, Layout.ZOOM_MIN, Layout.ZOOM_MAX, 1.0)
	_slider_row(_sliders, "Distance", distance, Layout.DISTANCE_MIN, Layout.DISTANCE_MAX, 0.0)
	words.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	words.add_theme_font_size_override("font_size", 30)
	words.add_theme_color_override("font_color", Color(0.65, 0.72, 0.8))
	words.clip_text = true
	words.text_overrun_behavior = TextServer.OVERRUN_TRIM_ELLIPSIS
	_sliders.add_child(words)


func _slider_row(row: HBoxContainer, label: String, sl: HSlider, lo: float, hi: float, v: float) -> void:
	var l := Label.new()
	l.text = label
	l.add_theme_font_size_override("font_size", 34)
	row.add_child(l)
	sl.min_value = lo
	sl.max_value = hi
	sl.step = 0.05
	sl.value = v
	sl.custom_minimum_size = Vector2(460, 90)
	sl.focus_mode = Control.FOCUS_NONE
	sl.value_changed.connect(func(_x): zoom_changed.emit(zoom.value, distance.value))
	row.add_child(sl)


func _style(b: Button, t: String, ic: Texture2D, w: int) -> void:
	b.text = t
	b.icon = ic
	b.expand_icon = false
	b.custom_minimum_size = Vector2(w, 140)
	b.add_theme_font_size_override("font_size", 40)
	b.add_theme_constant_override("h_separation", 16)
	b.focus_mode = Control.FOCUS_NONE


# The desktop's state: on or off, how many monitors (the 1 / 2 / All boxes), which are shown, its words.
func set_desktop(on: bool, count: int, mask: int, text: String) -> void:
	desktop_btn.set_pressed_no_signal(on)
	words.text = text if on else ""
	_sliders.visible = on
	var n := count if on else 0
	if n != _mon_count or mask != _mask:
		_mon_count = n
		_mask = mask
		for c in _mons.get_children():
			c.queue_free()
		var all := (1 << n) - 1
		for i in n:
			_mons.add_child(_mon_button(str(i + 1), 1 << i, mask == (1 << i)))
		if n > 1:
			_mons.add_child(_mon_button("All", all, mask == 0 or mask == all))


# The sliders' values (saved ones at the start), without telling anyone.
func set_zoom(z: float, d: float) -> void:
	zoom.set_value_no_signal(z)
	distance.set_value_no_signal(d)


func _mon_button(t: String, m: int, selected: bool) -> Button:
	var b := Button.new()
	b.text = t
	b.toggle_mode = true
	b.button_pressed = selected
	b.custom_minimum_size = Vector2(130 if t != "All" else 160, 140)
	b.add_theme_font_size_override("font_size", 46)
	b.focus_mode = Control.FOCUS_NONE
	b.pressed.connect(func(): monitors_chosen.emit(m))
	return b


func set_recording(on: bool) -> void:
	if on == _recording:
		return
	_recording = on
	record_btn.text = "Stop" if on else "Record"
	record_btn.icon = icon("stop" if on else "record")


# ---------------------------------------------------------------- the icons, drawn per pixel (ICON x ICON, once each)

static var _icons := {}


static func icon(kind: String) -> ImageTexture:
	if _icons.has(kind):
		return _icons[kind]
	var img := Image.create(ICON, ICON, false, Image.FORMAT_RGBA8)
	var c := Color(0.92, 0.94, 0.97)
	var red := Color(0.93, 0.31, 0.28)
	var s := float(ICON)
	for y in ICON:
		for x in ICON:
			var p := Vector2(x + 0.5, y + 0.5) / s # 0..1
			var a := 0.0
			var col := c
			match kind:
				"desktop": # a monitor on its foot
					a = maxf(_rect_line(p, Vector2(0.08, 0.14), Vector2(0.92, 0.68), 0.05),
						maxf(_seg(p, Vector2(0.5, 0.68), Vector2(0.5, 0.84), 0.05), _seg(p, Vector2(0.3, 0.86), Vector2(0.7, 0.86), 0.05)))
				"record", "stop": # a ring with a red dot (a red square while recording)
					a = _ring(p, Vector2(0.5, 0.5), 0.4, 0.05)
					var inner := (p - Vector2(0.5, 0.5)).length() < 0.24 if kind == "record" else \
						absf(p.x - 0.5) < 0.17 and absf(p.y - 0.5) < 0.17
					if inner:
						a = 1.0
						col = red
				"room": # a floor outline with its centre
					a = maxf(maxf(_seg(p, Vector2(0.5, 0.14), Vector2(0.92, 0.46), 0.045), _seg(p, Vector2(0.92, 0.46), Vector2(0.5, 0.86), 0.045)),
						maxf(maxf(_seg(p, Vector2(0.5, 0.86), Vector2(0.08, 0.46), 0.045), _seg(p, Vector2(0.08, 0.46), Vector2(0.5, 0.14), 0.045)),
						1.0 if (p - Vector2(0.5, 0.5)).length() < 0.07 else 0.0))
				"recenter": # a target: a ring, a dot, four ticks
					a = maxf(_ring(p, Vector2(0.5, 0.5), 0.3, 0.045), 1.0 if (p - Vector2(0.5, 0.5)).length() < 0.08 else 0.0)
					a = maxf(a, maxf(_seg(p, Vector2(0.5, 0.04), Vector2(0.5, 0.2), 0.045), _seg(p, Vector2(0.5, 0.8), Vector2(0.5, 0.96), 0.045)))
					a = maxf(a, maxf(_seg(p, Vector2(0.04, 0.5), Vector2(0.2, 0.5), 0.045), _seg(p, Vector2(0.8, 0.5), Vector2(0.96, 0.5), 0.045)))
			img.set_pixel(x, y, Color(col, clampf(a, 0.0, 1.0)))
	var t := ImageTexture.create_from_image(img)
	_icons[kind] = t
	return t


# coverage (0..1, a pixel-wide soft edge) of a line segment of half width r
static func _seg(p: Vector2, a: Vector2, b: Vector2, r: float) -> float:
	var ab := b - a
	var t := clampf((p - a).dot(ab) / ab.length_squared(), 0.0, 1.0)
	return clampf((r - (p - (a + ab * t)).length()) * ICON + 0.5, 0.0, 1.0)


static func _ring(p: Vector2, c: Vector2, radius: float, r: float) -> float:
	return clampf((r - absf((p - c).length() - radius)) * ICON + 0.5, 0.0, 1.0)


static func _rect_line(p: Vector2, lo: Vector2, hi: Vector2, r: float) -> float:
	return maxf(maxf(_seg(p, lo, Vector2(hi.x, lo.y), r), _seg(p, Vector2(hi.x, lo.y), hi, r)),
		maxf(_seg(p, hi, Vector2(lo.x, hi.y), r), _seg(p, Vector2(lo.x, hi.y), lo, r)))
