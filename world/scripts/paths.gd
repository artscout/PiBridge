# SPDX-License-Identifier: MIT
# Where PiBridge keeps its files (the same ones the app and the C++ tools use).
extends RefCounted


static func home() -> String:
	return OS.get_environment("HOME")


static func runtime_dir() -> String:
	var d := OS.get_environment("XDG_RUNTIME_DIR")
	return d if d != "" else "/tmp"


static func status_dir() -> String:
	return runtime_dir() + "/pibridge"


static func config_home() -> String:
	var d := OS.get_environment("XDG_CONFIG_HOME")
	return d if d != "" else home() + "/.config"


static func data_home() -> String:
	var d := OS.get_environment("XDG_DATA_HOME")
	return d if d != "" else home() + "/.local/share"


static func pibridge_conf() -> String:
	return config_home() + "/pibridge/pibridge.conf"


static func apps_toml() -> String:
	return config_home() + "/pibridge/apps.toml"


static func qv_settings() -> String:
	return config_home() + "/qv-vulkan/settings.conf"


# pibridge-record's settings (tools/record, docs/record.md): key = value, strings in quotes.
static func record_settings() -> String:
	return config_home() + "/pibridge/record.toml"


static func windata() -> String:
	return data_home() + "/pibridge/windata"


static func controller_models() -> String:
	return windata() + "/steamvr/rendermodels"


# The folder of a SteamVR render model (<name>/<name>.json or .obj): the copies in controller_models() first
# (docs/world.md), else the SteamVR install itself (its own models and the Index controllers' driver); "" when none.
static func render_model(model: String) -> String:
	var roots := [controller_models()]
	for base in [data_home() + "/Steam", home() + "/.steam/steam"]:
		var sv: String = base + "/steamapps/common/SteamVR"
		roots.append(sv + "/drivers/indexcontroller/resources/rendermodels")
		roots.append(sv + "/resources/rendermodels")
	for r in roots:
		if DirAccess.dir_exists_absolute(r + "/" + model):
			return r + "/" + model
	return ""


# A PiBridge tool: next to this install (the project …/share/pibridge/world, or the exported …/lib/pibridge/pibridge-world:
# both -> …/bin), then ~/.local/bin, then PATH.
static func tool(name: String) -> String:
	var here := ProjectSettings.globalize_path("res://")
	var bin := here + "/../../../bin/"
	if OS.has_feature("template"):
		here = OS.get_executable_path().get_base_dir()
		bin = here + "/../../bin/"
	for c in [bin + name, home() + "/.local/bin/" + name, here + "/../build/tools/" + name]:
		if FileAccess.file_exists(c):
			return c
	var out := []
	if OS.execute("sh", ["-c", "command -v " + name], out) == 0 and out.size() > 0:
		return String(out[0]).strip_edges()
	return ""


static func read_text(path: String) -> String:
	if not FileAccess.file_exists(path):
		return ""
	var f := FileAccess.open(path, FileAccess.READ)
	return f.get_as_text() if f else ""


# Atomic: a temporary file next to it, then renamed over.
static func write_text(path: String, text: String) -> bool:
	DirAccess.make_dir_recursive_absolute(path.get_base_dir())
	var tmp := path + ".tmp-world"
	var f := FileAccess.open(tmp, FileAccess.WRITE)
	if f == null:
		return false
	f.store_string(text)
	f.close()
	return DirAccess.rename_absolute(tmp, path) == OK
