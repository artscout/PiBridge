# SPDX-License-Identifier: MIT
# A Wavefront OBJ (+ MTL, its diffuse texture) as a mesh at run time: the controller render models of the user's
# Windows data kit (docs/world.md). Positions, texture coordinates, normals, polygon faces (fanned into triangles).
class_name ObjLoader
extends RefCounted


static func load_mesh(path: String) -> ArrayMesh:
	var text := Paths.read_text(path)
	if text == "":
		return null
	var pos := PackedVector3Array()
	var uv := PackedVector2Array()
	var nrm := PackedVector3Array()
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	var tex_path := ""
	for raw in text.split("\n"):
		var line := raw.strip_edges()
		if line.begins_with("v "):
			var p := line.split(" ", false)
			pos.append(Vector3(float(p[1]), float(p[2]), float(p[3])))
		elif line.begins_with("vt "):
			var p := line.split(" ", false)
			uv.append(Vector2(float(p[1]), 1.0 - float(p[2]))) # OBJ v grows up, Godot's down
		elif line.begins_with("vn "):
			var p := line.split(" ", false)
			nrm.append(Vector3(float(p[1]), float(p[2]), float(p[3])))
		elif line.begins_with("mtllib ") and tex_path == "":
			tex_path = _diffuse(path.get_base_dir() + "/" + line.substr(7).strip_edges())
		elif line.begins_with("f "):
			var f := line.split(" ", false)
			var corners := []
			for i in range(1, f.size()):
				corners.append(f[i].split("/"))
			for i in range(1, corners.size() - 1):
				# OBJ faces are counter-clockwise; Godot's front faces are clockwise
				for c in [corners[0], corners[i + 1], corners[i]]:
					if c.size() > 2 and c[2] != "":
						st.set_normal(nrm[int(c[2]) - 1])
					if c.size() > 1 and c[1] != "":
						st.set_uv(uv[int(c[1]) - 1])
					st.add_vertex(pos[int(c[0]) - 1])
	var mesh := st.commit()
	if mesh == null:
		return null
	var mat := StandardMaterial3D.new()
	mat.roughness = 0.6
	if tex_path != "" and FileAccess.file_exists(tex_path):
		var img := Image.load_from_file(tex_path)
		if img:
			mat.albedo_texture = ImageTexture.create_from_image(img)
	else:
		mat.albedo_color = Color(0.18, 0.19, 0.22)
	mesh.surface_set_material(0, mat)
	return mesh


static func _diffuse(mtl: String) -> String:
	for raw in Paths.read_text(mtl).split("\n"):
		var line := raw.strip_edges()
		if line.begins_with("map_Kd "):
			return mtl.get_base_dir() + "/" + line.substr(7).strip_edges()
	return ""
