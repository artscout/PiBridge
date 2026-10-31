# SPDX-License-Identifier: MIT
# PiBridge World (docs/world.md): the Monado backend's home in the headset. The world, the dashboard on a panel, the
# controllers with their lasers and models, Passthrough (pibridge-home --passthrough-overlay, Monado's overlay over
# this world). The menu button shows / hides the dashboard in front of you; holding a grip moves it there.
# `-- --selftest` runs the self test without a headset (the settings' files, the apps, the parsers, a click) and quits.
extends Node3D

const AppsModel = preload("res://scripts/apps_model.gd")
const BaseStations = preload("res://scripts/base_stations.gd")
const ControllerModel = preload("res://scripts/controller_model.gd")
const Dashboard = preload("res://scripts/dashboard.gd")
const KeyConf = preload("res://scripts/keyconf.gd")
const ObjLoader = preload("res://scripts/obj_loader.gd")
const Paths = preload("res://scripts/paths.gd")
const Pointer = preload("res://scripts/pointer.gd")
const VrPanel = preload("res://scripts/vr_panel.gd")
const WorldEnvironmentBuilder = preload("res://scripts/environment.gd")

var xr: XRInterface
var origin := XROrigin3D.new()
var camera := XRCamera3D.new()
var panel: VrPanel
var dash: Dashboard
var models: Array[ControllerModel] = []
var _hands := [] # per side: {grip, pointer, shown state}: a controller is drawn and points only while tracked
const LOST_GRACE_S := 1.0
var _profile_check := 0.0
var _passthrough_pid := 0
var _grip_hold := {}
# A game took the headset: Monado moved this session to SYNCHRONIZED (not shown, no input). After a moment the world
# closes itself with YIELD_CODE so the game gets its GPU memory back; the dashboard overlay over the game asks the
# PiBridge app to start it again once no app is left (app/src/core/io_session.h kHomeYieldCode). pibridge.conf
# world_close_for_games = off keeps it running in the background instead.
const YIELD_CODE := 75
const YIELD_AFTER_S := 2.0
var _was_focused := false
var _yield_in := -1.0
var _quitting := false


func _ready() -> void:
	if "--selftest" in OS.get_cmdline_user_args():
		await _selftest()
		return
	xr = XRServer.find_interface("OpenXR")
	if xr and xr.is_initialized():
		get_viewport().use_xr = true
		DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_DISABLED)
		print("PiBridge World: OpenXR on ", xr.get_name())
		# a recenter (the dashboard's, or the Pimax button held) turns the room under the panel: put it back
		# in front (Monado sends XrEventDataReferenceSpaceChangePending once the new origin is in place)
		xr.pose_recentered.connect(_on_recentered)
		if KeyConf.new(Paths.pibridge_conf()).value("world_close_for_games", "on") != "off":
			xr.session_focussed.connect(func(): _was_focused = true; _yield_in = -1.0)
			xr.session_visible.connect(func(): _yield_in = -1.0)
			xr.session_synchronized.connect(_on_session_hidden)
			xr.session_stopping.connect(func(): _quitting = true)
	else:
		push_warning("PiBridge World: no OpenXR session (is the Monado session running?); desktop view")
	WorldEnvironmentBuilder.build(self)
	WorldEnvironmentBuilder.apply_background(KeyConf.new(Paths.pibridge_conf()).value("home_background", ""))
	add_child(origin)
	origin.add_child(camera)
	camera.position.y = 1.7
	for side in ["left", "right"]:
		var aim := XRController3D.new()
		aim.tracker = StringName(side + "_hand")
		aim.pose = &"aim"
		origin.add_child(aim)
		aim.button_pressed.connect(_on_button.bind(side))
		aim.button_released.connect(_on_release.bind(side))
		var grip := XRController3D.new()
		grip.tracker = StringName(side + "_hand")
		grip.pose = &"grip"
		origin.add_child(grip)
		var m := ControllerModel.new(side)
		grip.add_child(m)
		models.append(m)
		# the laser from the model's aim frame (the drawn controller's nose); without a model, the runtime's aim pose
		var ptr := Pointer.new(aim)
		m.changed.connect(_place_pointer.bind(m, ptr, grip, aim))
		_place_pointer(m, ptr, grip, aim)
		_hands.append({"grip": grip, "pointer": ptr, "state": {}})
	origin.add_child(BaseStations.new())
	dash = Dashboard.new()
	dash.hide_requested.connect(func(): panel.visible = false)
	dash.passthrough_toggled.connect(_set_passthrough)
	dash.models_toggled.connect(func(on):
		for m in models:
			m.visible = on)
	panel = VrPanel.new(dash, Vector2i(1800, 1150), Vector2(1.8, 1.15))
	add_child(panel)
	await get_tree().process_frame
	_place_panel()


func _place_pointer(m: ControllerModel, ptr: Node3D, grip: Node3D, aim: Node3D) -> void:
	var parent := grip if m.found else aim
	if ptr.get_parent() != parent:
		if ptr.get_parent() != null:
			ptr.get_parent().remove_child(ptr)
		parent.add_child(ptr)
	ptr.transform = m.aim_from_grip if m.found else Transform3D.IDENTITY


# The model follows the controller in the hand: Valve Index controllers (Lighthouse) or the Crystal's.
func _check_profiles() -> void:
	for m in models:
		var t := XRServer.get_tracker(StringName(m.side + "_hand")) as XRPositionalTracker
		if t != null:
			m.set_kind(ControllerModel.kind_for_profile(t.profile))


func _exit_tree() -> void:
	_set_passthrough(false)


var _panel_write := 0.0


# Whether a controller is drawn: while the runtime tracks its position (confidence HIGH: the Pimax engine sees it, the
# base stations track it), and LOST_GRACE_S after that so a short occlusion does not blink. Never tracked yet (the
# Pimax controllers sit at a guessed rest pose until the headset first sees them), no pose (switched off) or lost
# longer: hidden. `state` keeps the last tracked time.
static func hand_shown(state: Dictionary, located: bool, tracked: bool, now: float) -> bool:
	if located and tracked:
		state["last"] = now
	return located and state.has("last") and now - float(state["last"]) <= LOST_GRACE_S


func _update_hands() -> void:
	var now := Time.get_ticks_msec() / 1000.0
	for h in _hands:
		var grip: XRController3D = h["grip"]
		var pose := grip.get_pose()
		var located := grip.get_has_tracking_data()
		var tracked := pose != null and pose.tracking_confidence == XRPose.XR_TRACKING_CONFIDENCE_HIGH
		var shown := hand_shown(h["state"], located, tracked, now)
		grip.visible = shown
		h["pointer"].tracked = shown


func _process(dt: float) -> void:
	_update_hands()
	_profile_check -= dt
	if _profile_check <= 0.0:
		_profile_check = 0.5
		_check_profiles()
	if _yield_in >= 0.0:
		_yield_in -= dt
		if _yield_in < 0.0 and not _quitting:
			_yield_to_game()
			return
	# while passthrough is on: where the panel is (the stage = this origin), for its window in the passthrough overlay
	if _passthrough_pid > 0:
		if not OS.is_process_running(_passthrough_pid):
			_passthrough_pid = 0
			dash.passthrough_btn.set_pressed_no_signal(false)
		_panel_write -= dt
		if _panel_write <= 0.0 and panel.visible:
			_panel_write = 0.1
			var t := panel.global_transform
			var q := t.basis.get_rotation_quaternion()
			Paths.write_text(Paths.status_dir() + "/world-panel", "%f %f %f %f %f %f %f %f %f\n" % [
				t.origin.x, t.origin.y, t.origin.z, q.x, q.y, q.z, q.w, panel.size_m.x, panel.size_m.y])
	for side in _grip_hold.keys():
		_grip_hold[side] += dt
		if _grip_hold[side] > 0.5:
			_grip_hold.erase(side)
			panel.visible = true
			_place_panel()


func _on_button(n: String, side: String) -> void:
	if (n == "menu_button" or n == "by_button" and side == "left") and _passthrough_pid > 0:
		# in passthrough the menu button brings you back to the world
		dash.passthrough_btn.set_pressed_no_signal(false)
		_set_passthrough(false)
		dash.say("Passthrough off.")
		return
	if n == "menu_button" or n == "by_button" and side == "left":
		panel.visible = not panel.visible
		if panel.visible:
			_place_panel()
	elif n == "grip_click":
		_grip_hold[side] = 0.0


func _on_release(n: String, side: String) -> void:
	if n == "grip_click":
		_grip_hold.erase(side)


# In front of the head, level, at a comfortable height and distance.
func _place_panel() -> void:
	var head := camera.global_transform
	var fwd := -head.basis.z
	fwd.y = 0
	if fwd.length() < 0.01:
		fwd = Vector3.FORWARD
	fwd = fwd.normalized()
	var pos := head.origin + fwd * 1.35
	pos.y = maxf(head.origin.y - 0.15, 0.8)
	panel.global_position = pos
	panel.look_at(pos + fwd, Vector3.UP)


func _on_recentered() -> void:
	if panel == null or not panel.visible:
		return
	# the head pose of the new origin arrives with the next frames
	await get_tree().process_frame
	await get_tree().process_frame
	_place_panel()


func _on_session_hidden() -> void:
	# SYNCHRONIZED also comes on the way to our own exit; only a hidden world after it was focused gives way
	if _was_focused and not _quitting:
		_yield_in = YIELD_AFTER_S


func _yield_to_game() -> void:
	_quitting = true
	_set_passthrough(false)
	print("PiBridge World: a game has the headset; closing to give it the memory (back when it ends)")
	get_tree().quit(YIELD_CODE)


func _set_passthrough(on: bool) -> void:
	if not on:
		if _passthrough_pid > 0:
			OS.kill(_passthrough_pid)
			_passthrough_pid = 0
		DirAccess.remove_absolute(Paths.status_dir() + "/world-panel")
		return
	if _passthrough_pid > 0 and OS.is_process_running(_passthrough_pid):
		return
	var home := Paths.tool("pibridge-home")
	if home == "":
		dash.say("Passthrough: pibridge-home was not found.")
		dash.passthrough_btn.set_pressed_no_signal(false)
		return
	_passthrough_pid = OS.create_process(home, ["--passthrough-overlay"])
	dash.say("Passthrough on: the menu button (or the Passthrough button) brings you back." if _passthrough_pid > 0
		else "Passthrough could not start.")


# ---------------------------------------------------------------- self test

var _fails := 0


func _check(ok: bool, what: String) -> void:
	print(("ok   " if ok else "FAIL ") + what)
	if not ok:
		_fails += 1


func _selftest() -> void:
	var tmp := OS.get_cache_dir() + "/pibridge-world-selftest-%d" % OS.get_process_id()
	DirAccess.make_dir_recursive_absolute(tmp + "/run/pibridge")
	OS.set_environment("HOME", tmp)
	OS.set_environment("XDG_CONFIG_HOME", tmp + "/cfg")
	OS.set_environment("XDG_DATA_HOME", tmp + "/data")
	OS.set_environment("XDG_RUNTIME_DIR", tmp + "/run")
	Paths.write_text(Paths.pibridge_conf(), "# mine\ntracking_mode=inside-out\nroom_eye_height_m=168\nio_room=Office\n")
	Paths.write_text(Paths.qv_settings(), "focus_density=1.00\n[app:Falcon BMS]\nfocus_density=0.80\n")
	dash = Dashboard.new()
	panel = VrPanel.new(dash, Vector2i(1800, 1150), Vector2(1.8, 1.15))
	add_child(panel)
	await get_tree().process_frame
	await get_tree().process_frame

	dash.set_ipd(64.5)
	_check(Paths.read_text(tmp + "/run/pibridge/ipd-request") == "64.5\n", "IPD: the lens motor request")
	_check(KeyConf.new(Paths.pibridge_conf()).value("ipd_mm") == "64.5", "IPD: saved in pibridge.conf")
	_check(Paths.read_text(Paths.pibridge_conf()).begins_with("# mine\n"), "pibridge.conf: comments kept")
	dash.set_ctrl_angle(40.2)
	_check(Paths.read_text(tmp + "/run/pimax-ctrl-angle") == "40\n", "controller angle: live file")
	dash.set_inhand("auto")
	_check(Paths.read_text(tmp + "/run/pimax-ctrl-inhand-mode") == "auto\n", "controllers shown: driver file")
	dash.vibrate(1)
	_check(Paths.read_text(tmp + "/run/pimax-haptic-request") == "3 60 200\n", "vibration: right hand request")
	dash.set_qv("focus_density", "1.20")
	var q := Paths.read_text(Paths.qv_settings())
	_check(q.begins_with("focus_density=1.20\n") and q.contains("[app:Falcon BMS]\nfocus_density=0.80"),
		"quad views: global value set, the app section kept")
	Paths.write_text(Paths.record_settings(), "# pibridge-record settings\neyes = \"left\"        # left | both\nfps = 30\n")
	dash.set_record("eyes", "both")
	dash.set_record("fps", "60")
	dash.set_record("mic", "on")
	var rec := Paths.read_text(Paths.record_settings())
	_check(rec.begins_with("# pibridge-record settings\n") and rec.contains("eyes=\"both\"\n") and rec.contains("fps=60\n")
		and rec.contains("mic=\"on\"\n"), "recording: settings saved as pibridge-record writes them")
	_check(dash.record_value("eyes", "left") == "both" and dash.record_value("max_height", "1080") == "1080",
		"recording: values read back without quotes, defaults when absent")
	dash.show_page("Recording")
	_check(dash.content.get_child_count() > 0, "recording: the page builds")
	dash.set_qv("sharpen", "0.40")
	_check(Paths.read_text(Paths.qv_settings()).contains("sharpen=0.40\n[app:"), "quad views: a new key before the sections")

	dash.recenter()
	_check(FileAccess.file_exists(tmp + "/run/pibridge/recenter-request"), "recenter: the driver's request file")
	_check(dash.room_setup_args(false) == ["--map", "--room", "Office", "--eye-height", "1.680", "--backend", "monado"],
		"room setup: inside-out (eye height from centimetres)")
	_check(dash.room_setup_args(true).has("--quick-direct"), "room setup: quick")
	KeyConf.new(Paths.pibridge_conf()).set_value("tracking_mode", "lighthouse")
	dash.conf.reload()
	_check(dash.room_setup_args(true)[0] == "--lh-stage" and not dash.room_setup_args(true).has("--quick-direct"),
		"room setup: Lighthouse is the stage")

	var exe := tmp + "/bin/my-game"
	Paths.write_text(exe, "#!/bin/sh\n")
	_check(dash.save_app({"new": true}, "My Game", exe, "-xr --fast", "", "", false), "apps: a new app saved")
	var own := AppsModel.own_apps()
	_check(own.size() == 1 and own[0]["name"] == "My Game" and own[0]["exec"] == exe and own[0]["id"] == "my-game"
		and own[0]["args"] == ["-xr", "--fast"] and own[0]["quad_views"] == false, "apps: read back with its arguments")
	AppsModel.add_app("My Game", exe)
	_check(AppsModel.own_apps()[1]["id"] == "my-game-2", "apps: a second one gets its own id")
	_check(dash.save_app(own[0], "Renamed", exe, "", "/games", "", true), "apps: edit saved")
	var ed := AppsModel.own_apps()
	_check(ed[0]["name"] == "Renamed" and ed[0]["workdir"] == "/games" and ed[0]["quad_views"] == true
		and ed[1]["name"] == "My Game", "apps: only that app changed")
	dash.show_page("Apps")
	await get_tree().process_frame
	var tiles := dash.content.find_children("*", "GridContainer", true, false)
	_check(tiles.size() == 1 and tiles[0].get_child_count() == 2, "apps: two tiles")
	dash.qv.set_section_value("app:XR Game", "focus_density", "0.90")
	_check(Paths.read_text(Paths.qv_settings()).contains("[app:XR Game]\nfocus_density=0.90"), "quad views: a game's own section")
	var lib := tmp + "/data/Steam"
	Paths.write_text(lib + "/steamapps/libraryfolders.vdf", "\"libraryfolders\"\n{\n\t\"0\"\n\t{\n\t\t\"path\"\t\t\"%s\"\n\t}\n}\n" % lib)
	Paths.write_text(lib + "/steamapps/appmanifest_10.acf", "\"AppState\"\n{\n\t\"appid\"\t\t\"10\"\n\t\"name\"\t\t\"XR Game\"\n\t\"installdir\"\t\t\"XRGame\"\n}\n")
	Paths.write_text(lib + "/steamapps/common/XRGame/bin/win64/openxr_loader.dll", "x")
	Paths.write_text(lib + "/steamapps/appmanifest_20.acf", "\"AppState\"\n{\n\t\"appid\"\t\t\"20\"\n\t\"name\"\t\t\"VR Old\"\n\t\"installdir\"\t\t\"VROld\"\n}\n")
	Paths.write_text(lib + "/steamapps/common/VROld/openvr_api.dll", "x")
	var games := AppsModel.steam_openxr_games()
	_check(games.size() == 1 and games[0]["name"] == "XR Game" and games[0]["appid"] == 10,
		"Steam: OpenXR games only (the OpenVR-only one left out)")

	Paths.write_text(tmp + "/m/t.obj", "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nvt 0 0\nvt 1 0\nvt 0 1\nf 1/1 2/2 3/3 4/1\n")
	var mesh := ObjLoader.load_mesh(tmp + "/m/t.obj")
	_check(mesh != null and mesh.get_surface_count() == 1 and mesh.surface_get_array_len(0) == 6, "OBJ: a quad, two triangles")

	WorldEnvironmentBuilder.build(self)
	var ppm := PackedByteArray("P6\n# x\n2 1\n255\n".to_ascii_buffer())
	ppm.append_array(PackedByteArray([255, 0, 0, 0, 0, 255]))
	var pf := FileAccess.open(tmp + "/data/pibridge/backgrounds/test_pano.ppm", FileAccess.WRITE) if DirAccess.make_dir_recursive_absolute(tmp + "/data/pibridge/backgrounds") == OK else null
	pf.store_buffer(ppm)
	pf.close()
	var img := WorldEnvironmentBuilder.load_ppm(tmp + "/data/pibridge/backgrounds/test_pano.ppm")
	_check(img != null and img.get_width() == 2 and img.get_pixel(1, 0).b > 0.9, "backgrounds: the PPM reader")
	dash.set_background("test_pano")
	_check(KeyConf.new(Paths.pibridge_conf()).value("home_background") == "test_pano"
		and WorldEnvironmentBuilder.parts["sky"].sky_material is PanoramaSkyMaterial
		and not WorldEnvironmentBuilder.parts["mountains"].visible, "backgrounds: a picture as the sky")
	dash.set_background("")
	_check(WorldEnvironmentBuilder.parts["sky"].sky_material is ProceduralSkyMaterial
		and WorldEnvironmentBuilder.parts["mountains"].visible, "backgrounds: back to the world")
	var b: Button = dash.pages["Apps"]["button"]
	var center := b.get_global_rect().get_center()
	panel.click_px(center)
	await get_tree().process_frame
	await get_tree().process_frame
	_check(dash._current == "Apps", "panel: a click on Apps opens the page")
	dash._editing = {"new": true, "name": "", "exec": "", "args": [], "workdir": "", "icon": "", "quad_views": true}
	dash.show_page("Apps")
	var field: LineEdit = dash.content.find_children("*", "LineEdit", true, false)[0]
	dash._edit_focused(field)
	dash.keyboard._shift_btn.button_pressed = true
	dash.keyboard._letter_buttons[0].pressed.emit()
	_check(field.text == "Q" and not dash.keyboard._shift_btn.button_pressed, "keyboard: Shift types one capital")
	dash.show_page("Quad views")
	_check(not dash.keyboard.visible and dash.keyboard.target == null, "keyboard: closes with the page")
	dash._editing = null
	dash.show_page("Quad views")
	await get_tree().process_frame
	_check(dash.content.get_child(dash.content.get_child_count() - 1).name == "Quadviews", "pages: Quad views builds")

	var hs := {}
	_check(not hand_shown(hs, true, false, 0.0) and hand_shown(hs, true, true, 1.0) and hand_shown(hs, true, false, 1.8)
		and not hand_shown(hs, true, false, 2.2) and not hand_shown(hs, false, false, 3.0),
		"controllers: drawn only while tracked (a second's grace), never at the rest pose before the first sight")
	var bsf := tmp + "/run/pibridge/monado/basestations"
	Paths.write_text(bsf, "LHB-1 0 2 0 0 0 0 1\n")
	KeyConf.new(Paths.pibridge_conf()).set_value("show_base_stations", "on")
	KeyConf.new(Paths.pibridge_conf()).set_value("tracking_mode", "inside-out")
	_check(not BaseStations.wanted(), "base stations: never in the inside-out mode")
	KeyConf.new(Paths.pibridge_conf()).set_value("tracking_mode", "lighthouse")
	var now_unix := Time.get_unix_time_from_system()
	_check(BaseStations.wanted() and BaseStations.fresh(bsf, now_unix) and not BaseStations.fresh(bsf, now_unix + 60),
		"base stations: in the Lighthouse mode, only from a file Monado is writing")
	var bs := BaseStations.parse("LHB-1 1 2.5 -3 0 0 0 1\nbad line\nLHB-2 0 0 0 0 0 0 0\n")
	_check(bs.size() == 1 and bs["LHB-1"].origin == Vector3(1, 2.5, -3), "base stations: the poses file")
	_check(ControllerModel.kind_for_profile("/interaction_profiles/valve/index_controller") == "index"
		and ControllerModel.kind_for_profile("/interaction_profiles/oculus/touch_controller") == "crystal"
		and ControllerModel.kind_for_profile("") == "", "controller models: the kind from the interaction profile")
	var mdir := tmp + "/data/pibridge/windata/steamvr/rendermodels/valve_controller_knu_1_0_left"
	Paths.write_text(mdir + "/body.obj", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
	Paths.write_text(mdir + "/valve_controller_knu_1_0_left.json", JSON.stringify({"components": {
		"body": {"filename": "body.obj", "component_local": {"origin": [0.005, 0.013, 0.107], "rotate_xyz": [93.8, 0, 0]}},
		"grip": {"component_local": {"origin": [0, -0.015, 0.13], "rotate_xyz": [0, 0, 0]}},
		"tip": {"component_local": {"origin": [0, -0.015, 0.02], "rotate_xyz": [0, 0, 0]}}}}))
	var cm := ControllerModel.new("left")
	add_child(cm)
	cm.set_kind("index")
	_check(cm.found and cm.aim_from_grip.origin.is_equal_approx(Vector3(0, 0, -0.11)),
		"controller models: the Index model on its grip frame, the laser from its tip")
	var body: MeshInstance3D = cm.get_child(0).get_child(0)
	_check(body.transform == Transform3D.IDENTITY, "controller models: the parts in the model's space (no component_local)")
	_check(ControllerModel._local({"component_local": {"origin": [0, 0, 0], "rotate_xyz": [15.392, 2.071, -0.303]}}).basis
		.is_equal_approx(Basis(Vector3(0, 0, 1), deg_to_rad(-0.303)) * Basis(Vector3(0, 1, 0), deg_to_rad(2.071))
		* Basis(Vector3(1, 0, 0), deg_to_rad(15.392))), "controller models: frames composed as Monado's (Rz Ry Rx)")

	print("PiBridge World self test: %s" % ("ok" if _fails == 0 else "%d failed" % _fails))
	get_tree().quit(1 if _fails > 0 else 0)
