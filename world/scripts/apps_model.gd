# SPDX-License-Identifier: MIT
# The apps the Monado backend can start: the user's own apps (PiBridge's ~/.config/pibridge/apps.toml, the app's
# Apps page list - the same file, so both show the same) and the installed Steam games that use OpenXR. A game that
# only has the SteamVR API (OpenVR) cannot run on Monado and is left out; so are non-VR games and Steam's own tools.
extends RefCounted

const Paths = preload("res://scripts/paths.gd")

const STEAM_SKIP := [250820, 228980, 1070560, 1391110, 1628350, 1493710, 2180100, 2348590] # SteamVR, runtimes, Proton


# [{id, name, exec, args, workdir}] from apps.toml ([[app]] tables; only the keys used here are read).
static func own_apps() -> Array:
	var out := []
	var cur := {}
	for raw in Paths.read_text(Paths.apps_toml()).split("\n"):
		var line := raw.strip_edges()
		if line == "[[app]]":
			if cur.has("exec"):
				out.append(cur)
			cur = {"args": [], "workdir": ""}
			continue
		if line.begins_with("[") or line.begins_with("#") or not line.contains("=") or cur.is_empty():
			continue
		var k := line.get_slice("=", 0).strip_edges()
		var v := line.substr(line.find("=") + 1).strip_edges()
		if k in ["id", "name", "exec", "workdir", "icon"]:
			cur[k] = _unquote(v)
		elif k == "quad_views":
			cur[k] = v != "false"
		elif k == "args" and v.begins_with("["):
			var a := []
			for part in v.trim_prefix("[").trim_suffix("]").split(","):
				var s := _unquote(part.strip_edges())
				if s != "":
					a.append(s)
			cur["args"] = a
	if cur.has("exec"):
		out.append(cur)
	return out


static func _unquote(v: String) -> String:
	if v.length() >= 2 and ((v.begins_with("\"") and v.ends_with("\"")) or (v.begins_with("'") and v.ends_with("'"))):
		return v.substr(1, v.length() - 2).replace("\\\"", "\"")
	return v


static func _expand(p: String) -> String:
	return Paths.home() + p.substr(1) if p.begins_with("~/") else p


# Append an app to apps.toml (the app reads it as one of its own); the id from the name, made unique.
static func add_app(app_name: String, exec: String) -> bool:
	var taken := {}
	for a in own_apps():
		taken[a.get("id", "")] = true
	var base := app_name.to_lower().replace(" ", "-")
	var id := ""
	for ch in base:
		if (ch >= "a" and ch <= "z") or (ch >= "0" and ch <= "9") or ch == "-":
			id += ch
	if id == "":
		id = "app"
	var unique := id
	var n := 2
	while taken.has(unique):
		unique = "%s-%d" % [id, n]
		n += 1
	var text := Paths.read_text(Paths.apps_toml())
	if text != "" and not text.ends_with("\n"):
		text += "\n"
	text += "\n[[app]]\nid = \"%s\"\nname = \"%s\"\nexec = \"%s\"\nargs = []\n" % [unique, app_name.replace("\"", "'"), exec.replace("\"", "\\\"")]
	return Paths.write_text(Paths.apps_toml(), text)


# Change an app's keys in its [[app]] table (the one with this id); every other line stays as written.
static func update_app(id: String, fields: Dictionary) -> bool:
	var lines := Paths.read_text(Paths.apps_toml()).split("\n")
	var start := -1
	var end := lines.size()
	var in_app := false
	for i in lines.size():
		var line := lines[i].strip_edges()
		if line == "[[app]]" or line.begins_with("["):
			if start >= 0:
				end = i
				break
			in_app = line == "[[app]]"
			continue
		if in_app and line.begins_with("id") and line.contains("=") and _unquote(line.substr(line.find("=") + 1).strip_edges()) == id:
			start = i
			var j := i
			while j > 0 and lines[j].strip_edges() != "[[app]]":
				j -= 1
			start = j
	if start < 0:
		return false
	for k in fields:
		var v = fields[k]
		var text_v: String
		if typeof(v) == TYPE_BOOL:
			text_v = "true" if v else "false"
		elif typeof(v) == TYPE_ARRAY or typeof(v) == TYPE_PACKED_STRING_ARRAY:
			var parts := []
			for a in v:
				parts.append("\"%s\"" % String(a).replace("\"", "\\\""))
			text_v = "[" + ", ".join(parts) + "]"
		else:
			text_v = "\"%s\"" % String(v).replace("\"", "\\\"")
		var done := false
		for i in range(start + 1, end):
			var line := lines[i].strip_edges()
			if line.contains("=") and line.get_slice("=", 0).strip_edges() == k:
				lines[i] = "%s = %s" % [k, text_v]
				done = true
				break
		if not done:
			lines.insert(end, "%s = %s" % [k, text_v])
			end += 1
	return Paths.write_text(Paths.apps_toml(), "\n".join(lines))


# The picture of a tile: the app's own image, the Steam library capsule (2:3), else none.
static func cover_path(app: Dictionary) -> String:
	if app.has("appid"):
		for base in [Paths.data_home() + "/Steam", Paths.home() + "/.steam/steam"]:
			for f in ["library_600x900.jpg", "library_capsule.jpg", "header.jpg"]:
				var p := "%s/appcache/librarycache/%d/%s" % [base, int(app["appid"]), f]
				if FileAccess.file_exists(p):
					return p
		return ""
	var icon := _expand(String(app.get("icon", "")))
	return icon if icon != "" and FileAccess.file_exists(icon) else ""


static func launch_own(app: Dictionary) -> int:
	var exec := _expand(String(app.get("exec", "")))
	var dir := _expand(String(app.get("workdir", "")))
	if dir == "":
		dir = exec.get_base_dir()
	var env := "" if app.get("quad_views", true) else "DISABLE_XR_APILAYER_ARTSCOUT_QUAD_VIEWS_VULKAN=1 "
	var cmd := "cd %s && %sexec %s" % [_q(dir), env, _q(exec)]
	for a in app.get("args", []):
		cmd += " " + _q(String(a))
	return OS.create_process("sh", ["-c", cmd])


static func _q(s: String) -> String:
	return "'" + s.replace("'", "'\\''") + "'"


# The Steam libraries (libraryfolders.vdf "path" lines).
static func steam_libraries() -> Array:
	var roots := []
	for base in [Paths.data_home() + "/Steam", Paths.home() + "/.steam/steam"]:
		var vdf := Paths.read_text(base + "/steamapps/libraryfolders.vdf")
		for raw in vdf.split("\n"):
			var line := raw.strip_edges()
			if line.begins_with("\"path\""):
				var p := line.substr(6).strip_edges().trim_prefix("\"").trim_suffix("\"")
				if not roots.has(p):
					roots.append(p)
		if roots.size() > 0:
			break
	return roots


# [{appid, name, dir, api}] of the installed Steam games with OpenXR (api "openxr"); slow (walks the game folders):
# call it off the main thread.
static func steam_openxr_games() -> Array:
	var out := []
	for lib in steam_libraries():
		var apps := String(lib) + "/steamapps"
		var d := DirAccess.open(apps)
		if d == null:
			continue
		for f in d.get_files():
			if not (f.begins_with("appmanifest_") and f.ends_with(".acf")):
				continue
			var acf := Paths.read_text(apps + "/" + f)
			var appid := int(_acf(acf, "appid"))
			var nm := _acf(acf, "name")
			var dir := apps + "/common/" + _acf(acf, "installdir")
			if appid in STEAM_SKIP or nm.contains("Proton") or nm.contains("Steam Linux Runtime"):
				continue
			var api := _vr_api(dir)
			if api == "openxr":
				out.append({"appid": appid, "name": nm, "dir": dir, "api": api})
	out.sort_custom(func(a, b): return String(a["name"]).naturalnocasecmp_to(String(b["name"])) < 0)
	return out


static func _acf(text: String, key: String) -> String:
	for raw in text.split("\n"):
		var line := raw.strip_edges()
		if line.begins_with("\"" + key + "\""):
			return line.substr(key.length() + 2).strip_edges().trim_prefix("\"").trim_suffix("\"")
	return ""


# "openxr" when the game ships an OpenXR loader (native or Windows), "openvr" when only the SteamVR API, "" neither.
static func _vr_api(dir: String) -> String:
	var openvr := false
	var stack := [[dir, 0]]
	var seen := 0
	while not stack.is_empty() and seen < 20000:
		var e = stack.pop_back()
		var d := DirAccess.open(e[0])
		if d == null:
			continue
		for f in d.get_files():
			seen += 1
			var l := f.to_lower()
			if l.begins_with("libopenxr_loader") or l == "openxr_loader.dll":
				return "openxr"
			if l == "openvr_api.dll" or l.begins_with("libopenvr_api"):
				openvr = true
		if e[1] < 5:
			for sub in d.get_directories():
				stack.append([e[0] + "/" + sub, e[1] + 1])
	return "openvr" if openvr else ""


# Point the system's OpenXR runtime at this session's Monado (as the app's Runtime switch does) and start the game
# through Steam: a game started by Steam never gets this process's environment, only the system runtime.
static func launch_steam(appid: int) -> String:
	var rt := OS.get_environment("XR_RUNTIME_JSON")
	if rt != "":
		var active := Paths.config_home() + "/openxr/1/active_runtime.json"
		DirAccess.make_dir_recursive_absolute(active.get_base_dir())
		var tmp := active + ".world-tmp"
		DirAccess.remove_absolute(tmp)
		var d := DirAccess.open(active.get_base_dir())
		if d != null and d.create_link(rt, tmp) == OK:
			DirAccess.rename_absolute(tmp, active)
	var pid := OS.create_process("steam", ["steam://rungameid/%d" % appid])
	if pid <= 0:
		pid = OS.create_process("xdg-open", ["steam://rungameid/%d" % appid])
	return "started through Steam (OpenXR runtime: Monado)" if pid > 0 else "Steam could not be started"
