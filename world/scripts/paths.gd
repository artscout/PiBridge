# SPDX-License-Identifier: MIT
# Where PiBridge keeps its files (the same ones the app and the C++ tools use).
class_name Paths
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


# A PiBridge tool: next to this project's install (…/share/pibridge/world -> …/bin), then ~/.local/bin, then PATH.
static func tool(name: String) -> String:
	var here := ProjectSettings.globalize_path("res://")
	for c in [here + "/../../../bin/" + name, home() + "/.local/bin/" + name, here + "/../build/tools/" + name]:
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
