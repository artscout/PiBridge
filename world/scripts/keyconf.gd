# SPDX-License-Identifier: MIT
# key=value files (pibridge.conf, the quad views settings): read a key, set a key in place. The rules of the app's
# Conf (core/conf.h) and of pibridge-home's confSet: the first uncommented line of the key is replaced, else the line
# goes at the end of the global part (before the first [section], so per-app sections keep their own values);
# comments, other keys and sections are kept as written.
extends RefCounted

const Paths = preload("res://scripts/paths.gd")

var path: String
var text := ""


func _init(p: String) -> void:
	path = p
	reload()


func reload() -> void:
	text = Paths.read_text(path)


func value(key: String, fallback := "") -> String:
	for raw in text.split("\n"):
		var line := raw.strip_edges()
		if line.begins_with("["):
			break # only the global part
		if line.begins_with("#") or not line.contains("="):
			continue
		var k := line.get_slice("=", 0).strip_edges()
		if k == key:
			var v := line.substr(line.find("=") + 1)
			var hash := v.find(" #")
			if hash >= 0:
				v = v.substr(0, hash)
			return v.strip_edges()
	return fallback


# The value of `key` in [section] ("" when absent).
func section_value(section: String, key: String, fallback := "") -> String:
	var inside := false
	for raw in text.split("\n"):
		var line := raw.strip_edges()
		if line.begins_with("["):
			inside = line == "[" + section + "]"
			continue
		if inside and not line.begins_with("#") and line.contains("=") and line.get_slice("=", 0).strip_edges() == key:
			return line.substr(line.find("=") + 1).strip_edges()
	return fallback


# Set `key` in [section] (made at the end when there is none); other sections and lines are kept.
func set_section_value(section: String, key: String, v: String) -> bool:
	reload()
	var lines := text.split("\n")
	if lines.size() > 0 and lines[lines.size() - 1] == "":
		lines.remove_at(lines.size() - 1)
	var head := "[" + section + "]"
	var at := lines.find(head)
	if at < 0:
		lines.append(head)
		lines.append(key + "=" + v)
	else:
		var end := lines.size()
		for i in range(at + 1, lines.size()):
			if lines[i].strip_edges().begins_with("["):
				end = i
				break
		var done := false
		for i in range(at + 1, end):
			var line := lines[i].strip_edges()
			if not line.begins_with("#") and line.contains("=") and line.get_slice("=", 0).strip_edges() == key:
				lines[i] = key + "=" + v
				done = true
				break
		if not done:
			lines.insert(end, key + "=" + v)
	text = "\n".join(lines) + "\n"
	return Paths.write_text(path, text)


func set_value(key: String, v: String) -> bool:
	reload() # another writer (the app) may have changed it
	var lines := text.split("\n")
	if lines.size() > 0 and lines[lines.size() - 1] == "":
		lines.remove_at(lines.size() - 1)
	var section_at := lines.size()
	var done := false
	for i in lines.size():
		var line := lines[i].strip_edges()
		if line.begins_with("["):
			section_at = i
			break
		if line.begins_with("#") or not line.contains("="):
			continue
		if line.get_slice("=", 0).strip_edges() == key:
			lines[i] = key + "=" + v
			done = true
			break
	if not done:
		lines.insert(section_at, key + "=" + v)
	text = "\n".join(lines) + "\n"
	return Paths.write_text(path, text)
