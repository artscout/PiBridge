# SPDX-License-Identifier: MIT
# The dashboard on the panel: pages on the left (Apps, Headset, Tracking, Controllers, Room, Quad views, Recording,
# World; it opens on Apps), the page in the middle, a status line at the bottom, Passthrough and Hide on top. Every
# setting goes where PiBridge's app and tools read it; what only applies at the next start of VR says so ("after
# restart").
#   IPD            $XDG_RUNTIME_DIR/pibridge/ipd-request (the lens motors, at once) + ipd_mm (the rendered eye distance)
#   Volume         the default sink (the headset while VR runs: pibridge-keepalived switches it) via pactl, at once
#   Refresh, lens, tracking mode / engine, camera fps     pibridge.conf, after restart
#   Controller angle   $XDG_RUNTIME_DIR/pimax-ctrl-angle (at once) + io_ctrl_angle
#   Controllers shown  $XDG_RUNTIME_DIR/pimax-ctrl-inhand-mode (at once, this session)
#   Vibrate            $XDG_RUNTIME_DIR/pimax-haptic-request "SLOT AMP MS"
#   Room setup     pibridge-roomsetup --backend monado: --map (inside-out) or --lh-stage (Lighthouse)
#   Quad views     ~/.config/qv-vulkan/settings.conf (the layer re-reads it while a game runs; densities: next game)
class_name Dashboard
extends PanelContainer

signal hide_requested
signal passthrough_toggled(on: bool)
signal models_toggled(on: bool)

var conf := KeyConf.new(Paths.pibridge_conf())
var qv := KeyConf.new(Paths.qv_settings())
var status := Label.new()
var pages := {}
var content := MarginContainer.new()
var keyboard := VrKeyboard.new()
var passthrough_btn := Button.new()
var record_btn := Button.new()
var _record_poll := 0.0
var _recording := false
var _roomsetup_pid := 0
var _steam_games := []
var _steam_thread: Thread
var _apps_box: VBoxContainer
var _volume_lbl := Label.new()
var _started_with := {}

const RESTART := "  (after restart)"


func _ready() -> void:
	theme = _theme()
	add_theme_stylebox_override("panel", _box(Color(0.06, 0.07, 0.09, 0.93), 30))
	var outer := VBoxContainer.new()
	outer.add_theme_constant_override("separation", 14)
	add_child(outer)
	outer.add_child(_top_bar())
	var mid := HBoxContainer.new()
	mid.size_flags_vertical = Control.SIZE_EXPAND_FILL
	mid.add_theme_constant_override("separation", 18)
	outer.add_child(mid)
	var nav := VBoxContainer.new()
	nav.custom_minimum_size = Vector2(250, 0)
	nav.add_theme_constant_override("separation", 8)
	mid.add_child(nav)
	content.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	mid.add_child(content)
	for p in ["Apps", "Headset", "Tracking", "Controllers", "Room", "Quad views", "Recording", "World"]:
		var b := Button.new()
		b.text = p
		b.custom_minimum_size = Vector2(250, 70)
		b.toggle_mode = true
		b.alignment = HORIZONTAL_ALIGNMENT_LEFT
		b.pressed.connect(show_page.bind(p))
		nav.add_child(b)
		pages[p] = {"button": b}
	status.add_theme_color_override("font_color", Color(0.75, 0.8, 0.88))
	status.text = "PiBridge World on %s." % _runtime_word()
	outer.add_child(status)
	keyboard.visible = false
	keyboard.done.connect(keyboard.close)
	outer.add_child(keyboard)
	for k in ["refresh_hz", "lens_profile", "tracking_mode", "io_slam_engine", "io_cam_fps", "ipd_mm"]:
		_started_with[k] = conf.value(k)
	show_page("Apps")


func _process(dt: float) -> void:
	_record_poll -= dt
	if _record_poll <= 0.0:
		_record_poll = 1.5
		_update_record_state()
	if _roomsetup_pid > 0 and not OS.is_process_running(_roomsetup_pid):
		_roomsetup_pid = 0
		say("The room setup ended.")
	if _steam_thread != null and not _steam_thread.is_alive():
		_steam_games = _steam_thread.wait_to_finish()
		_steam_thread = null
		if pages.has("Apps") and _current == "Apps" and _editing == null:
			show_page("Apps")


func say(t: String) -> void:
	status.text = t


func _runtime_word() -> String:
	return "Monado (%s)" % ("Lighthouse" if conf.value("tracking_mode", "lighthouse") != "inside-out" else "inside-out")


# ---------------------------------------------------------------- look


func _box(c: Color, r: int) -> StyleBoxFlat:
	var s := StyleBoxFlat.new()
	s.bg_color = c
	s.set_corner_radius_all(r)
	s.content_margin_left = 22
	s.content_margin_right = 22
	s.content_margin_top = 16
	s.content_margin_bottom = 16
	s.anti_aliasing = true
	return s


func _theme() -> Theme:
	var t := Theme.new()
	t.default_font_size = 30
	var normal := _box(Color(0.14, 0.16, 0.2, 0.95), 14)
	var hover := _box(Color(0.2, 0.3, 0.42, 0.98), 14)
	var down := _box(Color(0.16, 0.42, 0.62, 1.0), 14)
	for st in ["normal", "focus"]:
		t.set_stylebox(st, "Button", normal)
	t.set_stylebox("hover", "Button", hover)
	t.set_stylebox("pressed", "Button", down)
	t.set_stylebox("hover_pressed", "Button", down)
	t.set_color("font_color", "Button", Color(0.9, 0.93, 0.97))
	t.set_stylebox("normal", "LineEdit", _box(Color(0.1, 0.11, 0.14, 1.0), 10))
	t.set_stylebox("focus", "LineEdit", _box(Color(0.12, 0.16, 0.22, 1.0), 10))
	t.set_constant("separation", "HBoxContainer", 12)
	t.set_constant("separation", "VBoxContainer", 12)
	t.set_icon("grabber", "HSlider", _dot(26, Color(0.55, 0.85, 1.0)))
	t.set_icon("grabber_highlight", "HSlider", _dot(30, Color(0.7, 0.92, 1.0)))
	var track := StyleBoxFlat.new()
	track.bg_color = Color(0.25, 0.28, 0.34)
	track.content_margin_top = 4
	track.content_margin_bottom = 4
	track.set_corner_radius_all(4)
	t.set_stylebox("slider", "HSlider", track)
	return t


func _dot(d: int, c: Color) -> ImageTexture:
	var img := Image.create(d, d, false, Image.FORMAT_RGBA8)
	var r := d / 2.0
	for y in d:
		for x in d:
			var dist := Vector2(x + 0.5 - r, y + 0.5 - r).length()
			img.set_pixel(x, y, Color(c, clampf(r - dist, 0.0, 1.0)))
	return ImageTexture.create_from_image(img)


func _top_bar() -> Control:
	var h := HBoxContainer.new()
	var title := Label.new()
	title.text = "PiBridge"
	title.add_theme_font_size_override("font_size", 40)
	h.add_child(title)
	var where := Label.new()
	where.text = "  " + _runtime_word() + "  ·  room \"%s\"" % conf.value("io_room", "home")
	where.add_theme_color_override("font_color", Color(0.65, 0.72, 0.8))
	where.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	h.add_child(where)
	passthrough_btn.text = "◉  Passthrough"
	passthrough_btn.toggle_mode = true
	passthrough_btn.custom_minimum_size = Vector2(300, 64)
	passthrough_btn.toggled.connect(func(on): passthrough_toggled.emit(on))
	record_btn.text = "●  Record"
	record_btn.custom_minimum_size = Vector2(240, 64)
	record_btn.pressed.connect(toggle_record)
	h.add_child(record_btn)
	h.add_child(passthrough_btn)
	var hide := Button.new()
	hide.text = "Hide"
	hide.custom_minimum_size = Vector2(140, 64)
	hide.pressed.connect(func(): hide_requested.emit())
	h.add_child(hide)
	return h


# ---------------------------------------------------------------- building blocks


func _row(label: String, right: Control, note := "") -> HBoxContainer:
	var h := HBoxContainer.new()
	var l := Label.new()
	l.text = label
	l.custom_minimum_size = Vector2(380, 0)
	h.add_child(l)
	right.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	h.add_child(right)
	if note != "":
		var n := Label.new()
		n.text = note
		n.add_theme_color_override("font_color", Color(0.95, 0.72, 0.3))
		h.add_child(n)
	return h


func _choice(options: Array, current: String, on_pick: Callable) -> HBoxContainer:
	var h := HBoxContainer.new()
	var group := ButtonGroup.new()
	for o in options:
		var b := Button.new()
		b.text = o[1]
		b.toggle_mode = true
		b.button_group = group
		b.button_pressed = o[0] == current
		b.custom_minimum_size = Vector2(180, 60)
		b.pressed.connect(func(): on_pick.call(o[0]))
		h.add_child(b)
	return h


func _slider(lo: float, hi: float, step: float, v: float, fmt: String, on_set: Callable) -> HBoxContainer:
	var h := HBoxContainer.new()
	var s := HSlider.new()
	s.min_value = lo
	s.max_value = hi
	s.step = step
	s.value = v
	s.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	s.custom_minimum_size = Vector2(380, 50)
	var l := Label.new()
	l.text = fmt % v
	l.custom_minimum_size = Vector2(150, 0)
	s.value_changed.connect(func(x): l.text = fmt % x)
	s.drag_ended.connect(func(_c): on_set.call(s.value))
	h.add_child(s)
	h.add_child(l)
	return h


func _button(t: String, cb: Callable, w := 260) -> Button:
	var b := Button.new()
	b.text = t
	b.custom_minimum_size = Vector2(w, 64)
	b.pressed.connect(cb)
	return b


func _restart_note(key: String) -> String:
	return RESTART + (" · changed" if conf.value(key) != _started_with.get(key, "") else "")


func _set_conf(key: String, v: String, what: String) -> void:
	say(("%s: saved, used after VR restarts." % what) if conf.set_value(key, v) else "Could not write %s." % conf.path)
	show_page(_current)


# ---------------------------------------------------------------- pages

var _current := "Apps"


func show_page(p: String) -> void:
	_current = p
	keyboard.close() # its LineEdit goes with the old page
	for k in pages:
		pages[k]["button"].button_pressed = k == p
	for c in content.get_children():
		content.remove_child(c)
		c.queue_free()
	var page := VBoxContainer.new()
	page.name = p.replace(" ", "")
	var scroll := ScrollContainer.new()
	scroll.name = p.replace(" ", "")
	scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	scroll.add_child(page)
	page.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	content.add_child(scroll)
	conf.reload()
	match p:
		"Headset":
			_page_headset(page)
		"Tracking":
			_page_tracking(page)
		"Controllers":
			_page_controllers(page)
		"Room":
			_page_room(page)
		"Apps":
			_page_apps(page)
		"Quad views":
			_page_qv(page)
		"Recording":
			_page_recording(page)
		"World":
			_page_world(page)


func _page_headset(page: VBoxContainer) -> void:
	var ipd := float(conf.value("ipd_mm", "63"))
	page.add_child(_row("IPD (lens distance)", _slider(58, 72, 0.5, ipd, "%.1f mm", set_ipd)))
	var vol := HBoxContainer.new()
	vol.add_child(_button("−", func(): set_volume(-5), 100))
	_volume_lbl.text = _volume_now()
	_volume_lbl.custom_minimum_size = Vector2(150, 0)
	_volume_lbl.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	vol.add_child(_volume_lbl)
	vol.add_child(_button("+", func(): set_volume(5), 100))
	page.add_child(_row("Volume", vol))
	page.add_child(_row("Refresh rate", _choice([["72", "72 Hz"], ["90", "90 Hz"]], conf.value("refresh_hz", "90"),
			func(v): _set_conf("refresh_hz", v, "Refresh rate")), _restart_note("refresh_hz")))
	var lens := "chovr" if conf.value("lens_profile", "native") in ["chovr", "default"] else "native"
	page.add_child(_row("Lens profile", _choice([["native", "PiBridge native"], ["chovr", "CHOVR"]], lens,
			func(v): _set_conf("lens_profile", v, "Lens profile")), _restart_note("lens_profile")))


func _page_tracking(page: VBoxContainer) -> void:
	var mode := "inside-out" if conf.value("tracking_mode") == "inside-out" else "lighthouse"
	page.add_child(_row("Head tracking", _choice([["lighthouse", "Lighthouse"], ["inside-out", "Inside-out"]], mode,
			func(v): _set_conf("tracking_mode", v, "Tracking mode")), _restart_note("tracking_mode")))
	if mode == "inside-out":
		var eng := "basalt" if conf.value("io_slam_engine") == "basalt" else "pimax"
		page.add_child(_row("SLAM engine", _choice([["pimax", "Pimax"], ["basalt", "Basalt"]], eng,
				func(v): _set_conf("io_slam_engine", v, "SLAM engine")), _restart_note("io_slam_engine")))
		var fps := "50" if conf.value("io_cam_fps", "60") == "50" else "60"
		page.add_child(_row("Cameras", _choice([["50", "50 fps"], ["60", "60 fps"]], fps,
				func(v): _set_conf("io_cam_fps", v, "Camera rate")), _restart_note("io_cam_fps")))
	page.add_child(_row("Recenter", _button("Recenter now", recenter, 300)))
	var hint := Label.new()
	hint.text = "Or hold the Pimax button on a controller (the ring fills, then the view turns)."
	hint.autowrap_mode = TextServer.AUTOWRAP_WORD
	hint.add_theme_color_override("font_color", Color(0.65, 0.72, 0.8))
	page.add_child(hint)


func _page_controllers(page: VBoxContainer) -> void:
	var angle := float(conf.value("io_ctrl_angle", "52"))
	page.add_child(_row("Pointer angle", _slider(-20, 70, 1, angle, "%+.0f°", set_ctrl_angle)))
	var mode := Paths.read_text(Paths.runtime_dir() + "/pimax-ctrl-inhand").strip_edges()
	if not mode in ["auto", "always", "off"]:
		mode = "always"
	page.add_child(_row("Controllers shown", _choice([["always", "Always"], ["auto", "When held"], ["off", "Off"]], mode,
			set_inhand)))
	var vib := HBoxContainer.new()
	vib.add_child(_button("Left", func(): vibrate(0), 180))
	vib.add_child(_button("Right", func(): vibrate(1), 180))
	page.add_child(_row("Test vibration", vib))
	var models := CheckButton.new()
	models.text = "Draw the controllers' own models"
	models.button_pressed = true
	models.toggled.connect(func(on): models_toggled.emit(on))
	page.add_child(models)
	var src := Label.new()
	src.autowrap_mode = TextServer.AUTOWRAP_WORD
	src.add_theme_color_override("font_color", Color(0.65, 0.72, 0.8))
	src.text = ("Models: " + Paths.controller_models()) if DirAccess.dir_exists_absolute(Paths.controller_models()) \
		else "No controller models: copy them from the Pimax client (docs/world.md, \"Controller models\")."
	page.add_child(src)


func _page_room(page: VBoxContainer) -> void:
	var lh := conf.value("tracking_mode") != "inside-out"
	var info := Label.new()
	info.autowrap_mode = TextServer.AUTOWRAP_WORD
	info.text = ("Lighthouse: stand in the middle of your space, look forward, touch the floor with a controller." if lh
		else "Inside-out: the room is mapped through the cameras; walk and look around when asked.")
	page.add_child(info)
	var h := HBoxContainer.new()
	h.add_child(_button("Recenter", recenter, 240))
	h.add_child(_button("Room setup", func(): start_room_setup(false), 300))
	if not lh:
		h.add_child(_button("Quick setup", func(): start_room_setup(true), 300))
	page.add_child(h)


var _editing = null # the app being edited (Dictionary), else null
var _covers := {}
const TILE := Vector2(250, 375) # 2:3, as Steam's library capsules and the app's Apps page


func _page_apps(page: VBoxContainer) -> void:
	if _editing != null:
		_page_edit(page, _editing)
		return
	var head := HBoxContainer.new()
	var t := Label.new()
	t.text = "Apps"
	t.add_theme_font_size_override("font_size", 36)
	t.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	head.add_child(t)
	if _steam_thread == null:
		head.add_child(_button("Find Steam games" if _steam_games.is_empty() else "Look again", func():
			_steam_thread = Thread.new()
			_steam_thread.start(AppsModel.steam_openxr_games)
			show_page("Apps"), 300))
	head.add_child(_button("+ Add app", func():
		_editing = {"new": true, "name": "", "exec": "", "args": [], "workdir": "", "icon": "", "quad_views": true}
		show_page("Apps"), 220))
	page.add_child(head)
	if _steam_thread != null:
		page.add_child(_dim("Looking through the Steam library..."))
	var grid := GridContainer.new()
	grid.columns = 5
	grid.add_theme_constant_override("h_separation", 22)
	grid.add_theme_constant_override("v_separation", 22)
	page.add_child(grid)
	var all := AppsModel.own_apps()
	all.append_array(_steam_games)
	for a in all:
		grid.add_child(_tile(a))
	if all.is_empty():
		page.add_child(_dim("No apps yet: add one, or let PiBridge find your Steam games with OpenXR (games that only "
			+ "use SteamVR cannot run on Monado and are not listed)."))


func _tile(a: Dictionary) -> Control:
	var tile := PanelContainer.new()
	tile.add_theme_stylebox_override("panel", _box(Color(0.12, 0.14, 0.18, 0.97), 18))
	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", 8)
	tile.add_child(v)
	var cover := _cover(a)
	v.add_child(cover)
	var name_lbl := Label.new()
	name_lbl.text = String(a.get("name", a.get("exec", "")))
	name_lbl.custom_minimum_size = Vector2(TILE.x, 0)
	name_lbl.text_overrun_behavior = TextServer.OVERRUN_TRIM_ELLIPSIS
	name_lbl.clip_text = true
	v.add_child(name_lbl)
	var kind := Label.new()
	kind.text = "Steam · OpenXR" if a.has("appid") else "Your app"
	kind.add_theme_font_size_override("font_size", 22)
	kind.add_theme_color_override("font_color", Color(0.6, 0.67, 0.76))
	v.add_child(kind)
	var buttons := HBoxContainer.new()
	buttons.add_child(_button("Run", func(): run_app(a), 140))
	buttons.add_child(_button("Edit", func():
		_editing = a.duplicate(true)
		show_page("Apps"), 100))
	v.add_child(buttons)
	return tile


func _cover(a: Dictionary) -> Control:
	var path := AppsModel.cover_path(a)
	if path != "":
		if not _covers.has(path):
			var img := Image.load_from_file(path)
			if img:
				img.resize(int(TILE.x), int(TILE.y), Image.INTERPOLATE_LANCZOS)
				_covers[path] = ImageTexture.create_from_image(img)
		if _covers.has(path):
			var tr := TextureRect.new()
			tr.texture = _covers[path]
			tr.custom_minimum_size = TILE
			tr.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
			tr.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_COVERED
			return tr
	# no picture: the first letter on a colour from the name
	var c := Panel.new()
	c.custom_minimum_size = TILE
	var nm := String(a.get("name", "?"))
	var hue := float(abs(nm.hash()) % 360) / 360.0
	c.add_theme_stylebox_override("panel", _box(Color.from_hsv(hue, 0.45, 0.42), 12))
	var l := Label.new()
	l.text = nm.substr(0, 1).to_upper()
	l.add_theme_font_size_override("font_size", 120)
	l.set_anchors_preset(Control.PRESET_FULL_RECT)
	l.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	l.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	c.add_child(l)
	return c


func run_app(a: Dictionary) -> void:
	if a.has("appid"):
		say("%s: %s." % [a["name"], AppsModel.launch_steam(int(a["appid"]))])
	else:
		say("%s: %s." % [a.get("name", ""), "started" if AppsModel.launch_own(a) > 0 else "could not start"])


func _edit_line(page: VBoxContainer, label: String, value: String) -> LineEdit:
	var e := LineEdit.new()
	e.text = value
	e.custom_minimum_size = Vector2(0, 60)
	e.focus_entered.connect(_edit_focused.bind(e))
	page.add_child(_row(label, e))
	return e


func _page_edit(page: VBoxContainer, a: Dictionary) -> void:
	var t := Label.new()
	t.add_theme_font_size_override("font_size", 36)
	page.add_child(t)
	if a.has("appid"):
		# a Steam game: its own quad views settings ([app:<name>] in the layer's file)
		t.text = String(a["name"]) + " · quad views for this game"
		var sec := "app:" + String(a["name"])
		qv.reload()
		page.add_child(_row("Focus follows", _choice([["eye", "Eyes"], ["fixed", "Fixed centre"]],
				qv.section_value(sec, "tracking", qv.value("tracking", "eye")),
				func(v): say("Saved." if qv.set_section_value(sec, "tracking", v) else "Could not write %s." % qv.path))))
		page.add_child(_row("Focus density", _slider(0.5, 1.5, 0.05,
				float(qv.section_value(sec, "focus_density", qv.value("focus_density", "1.00"))), "%.2f",
				func(v): say("Saved." if qv.set_section_value(sec, "focus_density", "%.2f" % v) else "Could not write."))))
		page.add_child(_row("Periphery density", _slider(0.2, 1.0, 0.05,
				float(qv.section_value(sec, "peripheral_density", qv.value("peripheral_density", "0.40"))), "%.2f",
				func(v): say("Saved." if qv.set_section_value(sec, "peripheral_density", "%.2f" % v) else "Could not write."))))
		var back := HBoxContainer.new()
		back.add_child(_button("Run", func(): run_app(a), 200))
		back.add_child(_button("Back", func():
			_editing = null
			show_page("Apps"), 200))
		page.add_child(back)
		return
	t.text = "New app" if a.get("new", false) else "Edit " + String(a.get("name", ""))
	var nm := _edit_line(page, "Name", String(a.get("name", "")))
	var ex := _edit_line(page, "Program", String(a.get("exec", "")))
	var ar := _edit_line(page, "Arguments", " ".join(a.get("args", [])))
	var wd := _edit_line(page, "Folder", String(a.get("workdir", "")))
	var ic := _edit_line(page, "Picture", String(a.get("icon", "")))
	var q := CheckButton.new()
	q.text = "Quad views (the layer) for this app"
	q.button_pressed = a.get("quad_views", true)
	page.add_child(q)
	var row := HBoxContainer.new()
	row.add_child(_button("Save", func(): save_app(a, nm.text, ex.text, ar.text, wd.text, ic.text, q.button_pressed), 200))
	if not a.get("new", false):
		row.add_child(_button("Run", func(): run_app(a), 200))
	row.add_child(_button("Back", func():
		_editing = null
		show_page("Apps"), 200))
	page.add_child(row)


func save_app(a: Dictionary, nm: String, ex: String, args: String, wd: String, icon: String, quad: bool) -> bool:
	nm = nm.strip_edges()
	ex = ex.strip_edges()
	if nm == "" or ex == "":
		say("Give the app a name and its program.")
		return false
	var full := Paths.home() + ex.substr(1) if ex.begins_with("~/") else ex
	if not FileAccess.file_exists(full):
		say("%s does not exist." % ex)
		return false
	var ok := false
	if a.get("new", false):
		ok = AppsModel.add_app(nm, ex)
		if ok:
			var added: Array = AppsModel.own_apps()
			a = added[added.size() - 1]
	if a.has("id"):
		ok = AppsModel.update_app(String(a["id"]), {"name": nm, "exec": ex, "args": args.split(" ", false),
				"workdir": wd.strip_edges(), "icon": icon.strip_edges(), "quad_views": quad})
	say(("%s saved (also on the app's Apps page)." % nm) if ok else "Could not write %s." % Paths.apps_toml())
	if ok:
		_editing = null
		show_page("Apps")
	return ok


func _dim(t: String) -> Label:
	var l := Label.new()
	l.text = t
	l.autowrap_mode = TextServer.AUTOWRAP_WORD
	l.add_theme_color_override("font_color", Color(0.65, 0.72, 0.8))
	return l


func _edit_focused(e: LineEdit) -> void:
	keyboard.target = e
	keyboard.visible = true


func _page_qv(page: VBoxContainer) -> void:
	qv.reload()
	page.add_child(_dim("Quad views: games that render a sharp focus where you look and a lighter periphery (for "
		+ "example Falcon BMS). The focus follows your eyes with eye tracking. Changes reach a running game, the two "
		+ "densities the next game start."))
	page.add_child(_row("Focus follows", _choice([["eye", "Eyes"], ["fixed", "Fixed centre"]], qv.value("tracking", "eye"),
			func(v): set_qv("tracking", v))))
	page.add_child(_row("Periphery density", _slider(0.2, 1.0, 0.05, float(qv.value("peripheral_density", "0.40")), "%.2f",
			func(v): set_qv("peripheral_density", "%.2f" % v))))
	page.add_child(_row("Focus density", _slider(0.5, 1.5, 0.05, float(qv.value("focus_density", "1.00")), "%.2f",
			func(v): set_qv("focus_density", "%.2f" % v))))
	page.add_child(_row("Focus size (eyes)", _slider(0.2, 0.6, 0.01, float(qv.value("gaze_width", "0.35")), "%.2f",
			func(v):
				set_qv("gaze_width", "%.2f" % v)
				set_qv("gaze_height", "%.2f" % v))))
	page.add_child(_row("Focus size (fixed)", _slider(0.3, 0.7, 0.01, float(qv.value("fixed_width", "0.50")), "%.2f",
			func(v):
				set_qv("fixed_width", "%.2f" % v)
				set_qv("fixed_height", "%.2f" % (v * 0.9)))))
	page.add_child(_row("Edge blend", _slider(0.0, 0.5, 0.01, float(qv.value("transition", "0.20")), "%.2f",
			func(v): set_qv("transition", "%.2f" % v))))
	page.add_child(_row("Sharpening", _slider(0.0, 1.0, 0.05, float(qv.value("sharpen", "0.60")), "%.2f",
			func(v): set_qv("sharpen", "%.2f" % v))))
	var dbg := CheckButton.new()
	dbg.text = "Show the focus area (debug)"
	dbg.button_pressed = qv.value("debug_overlay", "off") == "on"
	dbg.toggled.connect(func(on): set_qv("debug_overlay", "on" if on else "off"))
	page.add_child(dbg)


func _page_world(page: VBoxContainer) -> void:
	var cur := conf.value("home_background", "")
	page.add_child(_dim("The world around you. The pictures are the 360-degree backgrounds the app's Home page downloads."))
	var grid := GridContainer.new()
	grid.columns = 4
	grid.add_theme_constant_override("h_separation", 18)
	grid.add_theme_constant_override("v_separation", 18)
	page.add_child(grid)
	var choices := [["", "PiBridge world", ""], ["dusk", "Dusk", ""]]
	var dir := Paths.data_home() + "/pibridge/backgrounds"
	var d := DirAccess.open(dir)
	if d != null:
		for f in d.get_files():
			if f.ends_with(".ppm"):
				var id := f.trim_suffix(".ppm")
				choices.append([id, id.replace("_", " ").capitalize(), dir + "/thumbs/" + id + ".png"])
	for c in choices:
		var b := Button.new()
		b.toggle_mode = true
		b.button_pressed = c[0] == cur or (c[0] == "" and cur in ["passthrough", "none"])
		b.custom_minimum_size = Vector2(330, 200)
		b.text = c[1]
		b.vertical_icon_alignment = VERTICAL_ALIGNMENT_TOP
		b.icon_alignment = HORIZONTAL_ALIGNMENT_CENTER
		b.expand_icon = true
		if c[2] != "" and FileAccess.file_exists(c[2]):
			var img := Image.load_from_file(c[2])
			if img:
				b.icon = ImageTexture.create_from_image(img)
		b.pressed.connect(func(): set_background(c[0]))
		grid.add_child(b)


# ---------------------------------------------------------------- Recording: pibridge-record's settings

const RECORD_STRING_KEYS := ["eyes", "quality", "codec", "audio", "mic", "dir", "encoder", "view"]


# A setting of record.toml without its quotes and comment (the recorder's defaults when absent).
func record_value(key: String, fallback: String) -> String:
	var v := KeyConf.new(Paths.record_settings()).value(key, fallback)
	return v.trim_prefix("\"").trim_suffix("\"")


# Save one setting for the next recording (strings in quotes, as pibridge-record writes them).
func set_record(key: String, v: String) -> bool:
	var rc := KeyConf.new(Paths.record_settings())
	var ok := rc.set_value(key, ("\"%s\"" % v) if key in RECORD_STRING_KEYS else v)
	say(("Recording: saved, used from the next recording." if ok else "Could not write %s." % rc.path))
	return ok


func _page_recording(page: VBoxContainer) -> void:
	var state := HBoxContainer.new()
	var now := Label.new()
	now.text = "Recording now." if _recording else "Not recording."
	now.custom_minimum_size = Vector2(380, 0)
	now.add_theme_color_override("font_color", Color(1.0, 0.45, 0.42) if _recording else Color(0.75, 0.8, 0.88))
	state.add_child(now)
	state.add_child(_button("■  Stop" if _recording else "●  Start recording", func():
		toggle_record()
		await get_tree().create_timer(1.2).timeout
		_update_record_state()
		show_page("Recording"), 360))
	page.add_child(state)
	var pick := func(key: String, label: String, options: Array, fallback: String):
		page.add_child(_row(label, _choice(options, record_value(key, fallback), func(v):
			set_record(key, v)
			show_page("Recording"))))
	pick.call("eyes", "Picture", [["left", "One eye"], ["both", "Both eyes"]], "left")
	pick.call("fps", "Frame rate", [["30", "30 fps"], ["60", "60 fps"], ["72", "72 fps"]], "30")
	pick.call("quality", "Quality", [["low", "Low"], ["medium", "Medium"], ["high", "High"]], "high")
	var mh := int(record_value("max_height", "1080"))
	var res := "2160" if mh >= 2000 else ("1440" if mh >= 1300 else "1080")
	page.add_child(_row("Resolution", _choice([["1080", "1080p"], ["1440", "1440p"], ["2160", "4K"]], res, func(v):
		set_record("max_height", v)
		show_page("Recording"))))
	pick.call("codec", "Codec", [["h264", "H.264"], ["hevc", "H.265"]], "h264")
	pick.call("audio", "Sound", [["on", "On"], ["off", "Off"]], "on")
	pick.call("mic", "Microphone", [["off", "Off"], ["on", "On"]], "off")
	page.add_child(_dim("Saved to %s; the videos go to %s. Changes apply from the next recording." % [
		Paths.record_settings().replace(Paths.home(), "~"), record_value("dir", "~/Videos/pimax")]))


func set_background(id: String) -> void:
	conf.set_value("home_background", id if id != "" else "none")
	say(WorldEnvironmentBuilder.apply_background(id))
	show_page("World")


# ---------------------------------------------------------------- actions (also called by the self test)


func set_ipd(mm: float) -> void:
	var v := "%.1f" % mm
	var motor := Paths.write_text(Paths.status_dir() + "/ipd-request", v + "\n")
	conf.set_value("ipd_mm", v)
	say("IPD %s mm: %s; the rendered eye distance after restart." % [v, "the lenses move" if motor else "could not reach the lens motors"])


func _volume_now() -> String:
	var out := []
	if OS.execute("pactl", ["get-sink-volume", "@DEFAULT_SINK@"], out) == 0 and out.size() > 0:
		var m := RegEx.create_from_string("(\\d+)%").search(String(out[0]))
		if m:
			return m.get_string(1) + " %"
	return "—"


func set_volume(step: int) -> void:
	OS.execute("pactl", ["set-sink-volume", "@DEFAULT_SINK@", ("%+d%%" % step)])
	_volume_lbl.text = _volume_now()
	say("Volume %s." % _volume_lbl.text)


func set_ctrl_angle(deg: float) -> void:
	var v := "%d" % int(round(deg))
	var live := Paths.write_text(Paths.runtime_dir() + "/pimax-ctrl-angle", v + "\n")
	conf.set_value("io_ctrl_angle", v)
	say("Pointer angle %s°%s." % [v, "" if live else " (saved; the driver gets it at the next start)"])


func set_inhand(m: String) -> void:
	say(("Controllers shown: %s." % m) if Paths.write_text(Paths.runtime_dir() + "/pimax-ctrl-inhand-mode", m + "\n")
		else "Could not reach the driver.")


func vibrate(hand: int) -> void:
	var ok := Paths.write_text(Paths.runtime_dir() + "/pimax-haptic-request", "%d 60 200\n" % (2 if hand == 0 else 3))
	say("Vibration: %s." % ("sent" if ok else "could not reach the driver"))


# A video recording runs: pibridge-record's recorder process ("pibridge-record run ..."); under Monado it records
# through the compositor's tap (docs/record.md).
func recording_now() -> bool:
	# the recorder's own word: `pibridge-record status` prints "not recording" unless its recorder runs
	var tool := Paths.tool("pibridge-record")
	var out := []
	if tool == "" or OS.execute(tool, ["status"], out) != 0 or out.is_empty():
		return false
	var first := String(out[0]).strip_edges().split("\n")[0].to_lower()
	return first != "" and not first.begins_with("not recording")


func _update_record_state() -> void:
	_recording = recording_now()
	record_btn.text = "■  Stop recording" if _recording else "●  Record"
	record_btn.add_theme_color_override("font_color", Color(1.0, 0.45, 0.42) if _recording else Color(0.9, 0.93, 0.97))


func toggle_record() -> void:
	var tool := Paths.tool("pibridge-record")
	if tool == "":
		say("pibridge-record was not found.")
		return
	var pid := OS.create_process(tool, ["stop" if _recording else "start", "--brief"])
	say(("Stopping the recording: the video goes to ~/Videos/pimax." if _recording else "Recording what you see.")
		if pid > 0 else "pibridge-record could not start.")
	_record_poll = 1.0


# The Pimax driver recenters when this file changes (as on the button's long press).
func recenter() -> void:
	var ok := Paths.write_text(Paths.status_dir() + "/recenter-request", "%d\n" % Time.get_unix_time_from_system())
	say("Recentered: forward is where you look." if ok else "Could not reach the driver.")


func set_qv(key: String, v: String) -> void:
	say(("Quad views: %s = %s." % [key, v]) if qv.set_value(key, v) else "Could not write %s." % qv.path)


func room_setup_args(quick: bool) -> Array:
	var room := conf.value("io_room", "home")
	var lh := conf.value("tracking_mode") != "inside-out"
	var args := ["--lh-stage" if lh else "--map", "--room", room]
	var eye := float(conf.value("room_eye_height_m", "0"))
	if eye > 3.0:
		eye /= 100.0 # centimetres in older files
	if eye < 1.0:
		var body := float(conf.value("room_body_height_m", "0"))
		eye = body * 0.936 if body > 1.2 else 0.0
	if eye >= 1.0 and eye <= 2.3:
		args += ["--eye-height", "%.3f" % eye]
	if quick and not lh:
		args.append("--quick-direct")
	args += ["--backend", "monado"]
	return args


func start_room_setup(quick: bool) -> void:
	if _roomsetup_pid > 0 and OS.is_process_running(_roomsetup_pid):
		say("The room setup is already running.")
		return
	var tool := Paths.tool("pibridge-roomsetup")
	if tool == "":
		say("pibridge-roomsetup was not found.")
		return
	OS.set_environment("PIBRIDGE_BACKEND", "monado")
	_roomsetup_pid = OS.create_process(tool, room_setup_args(quick))
	say("Room setup started: follow it in the headset." if _roomsetup_pid > 0 else "The room setup could not start.")
