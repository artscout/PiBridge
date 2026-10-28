# SPDX-License-Identifier: MIT
# A keyboard on the panel, typed with the lasers: letters, digits, the characters of paths and names, Shift, space,
# backspace, Done. It types into `target` (a LineEdit); its keys never take the focus away from it.
class_name VrKeyboard
extends PanelContainer

signal done

var target: LineEdit
var _shift := false
var _shift_btn: Button
var _letter_buttons: Array[Button] = []

const ROWS := [
	["1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "-", "_"],
	["q", "w", "e", "r", "t", "y", "u", "i", "o", "p", "/", "~"],
	["a", "s", "d", "f", "g", "h", "j", "k", "l", ".", ":", "="],
	["z", "x", "c", "v", "b", "n", "m", ",", "'", "\"", "+", "@"],
]


func _ready() -> void:
	var box := VBoxContainer.new()
	box.add_theme_constant_override("separation", 6)
	add_child(box)
	for row in ROWS:
		var h := HBoxContainer.new()
		h.add_theme_constant_override("separation", 6)
		h.alignment = BoxContainer.ALIGNMENT_CENTER
		for k in row:
			var b := _key(k, 64)
			b.pressed.connect(func(): _type(b.text))
			if k.length() == 1 and k >= "a" and k <= "z":
				_letter_buttons.append(b)
			h.add_child(b)
		box.add_child(h)
	var last := HBoxContainer.new()
	last.add_theme_constant_override("separation", 6)
	last.alignment = BoxContainer.ALIGNMENT_CENTER
	# Shift stays lit while it is on: one letter in capitals, then it lets go by itself
	var shift := _key("⇧ Shift", 140)
	shift.toggle_mode = true
	shift.toggled.connect(_set_shift)
	_shift_btn = shift
	var space := _key("Space", 420)
	space.pressed.connect(func(): _type(" "))
	var back := _key("⌫", 140)
	back.pressed.connect(_backspace)
	var ok := _key("Done", 160)
	ok.pressed.connect(func(): done.emit())
	for b in [shift, space, back, ok]:
		last.add_child(b)
	box.add_child(last)


func _key(t: String, w: int) -> Button:
	var b := Button.new()
	b.text = t
	b.custom_minimum_size = Vector2(w, 64)
	b.focus_mode = Control.FOCUS_NONE
	return b


func _type(s: String) -> void:
	if target == null:
		return
	target.insert_text_at_caret(s)
	if _shift:
		_shift_btn.button_pressed = false


func _backspace() -> void:
	if target == null or target.text.is_empty():
		return
	var c := target.caret_column
	if c > 0:
		target.text = target.text.substr(0, c - 1) + target.text.substr(c)
		target.caret_column = c - 1


## Hides the keyboard and lets go of its LineEdit (the page that held it is about to be rebuilt).
func close() -> void:
	visible = false
	target = null
	if _shift_btn != null:
		_shift_btn.button_pressed = false


func _set_shift(on: bool) -> void:
	_shift = on
	for b in _letter_buttons:
		b.text = b.text.to_upper() if _shift else b.text.to_lower()
