# SPDX-License-Identifier: MIT
# The world around you: a sky with a low sun, a wide plain with a faint grid fading into haze, a ring of distant
# mountains, and the disc you stand on (its edge glows softly). Built in code, no asset files.
extends RefCounted

const Paths = preload("res://scripts/paths.gd")

const GROUND_SHADER := """
shader_type spatial;
render_mode cull_back;
// value noise, a few octaves: the terrain's height far away and its colour everywhere
float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
float noise(vec2 p) {
	vec2 i = floor(p), f = fract(p);
	vec2 u = f * f * (3.0 - 2.0 * f);
	return mix(mix(hash(i), hash(i + vec2(1, 0)), u.x), mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), u.x), u.y);
}
float fbm(vec2 p) {
	float v = 0.0, a = 0.5;
	for (int k = 0; k < 5; k++) { v += a * noise(p); p *= 2.03; a *= 0.5; }
	return v;
}
// flat where you stand (within 12 m), rising into rolling hills towards the mountains
float height(vec2 xz) {
	float r = length(xz);
	float rise = smoothstep(12.0, 70.0, r);
	return rise * (fbm(xz * 0.025) * 14.0 - 3.0) + rise * rise * 6.0;
}
varying vec3 wpos;
void vertex() {
	vec2 xz = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xz;
	VERTEX.y += height(xz);
	// the normal from the height's slope
	float e = 0.5;
	vec3 n = normalize(vec3(height(xz - vec2(e, 0)) - height(xz + vec2(e, 0)), 2.0 * e,
	                        height(xz - vec2(0, e)) - height(xz + vec2(0, e))));
	NORMAL = n;
	wpos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}
void fragment() {
	vec2 xz = wpos.xz;
	float n1 = fbm(xz * 0.35), n2 = fbm(xz * 2.1 + 17.0);
	vec3 moss = vec3(0.10, 0.16, 0.10), stone = vec3(0.22, 0.22, 0.24), dry = vec3(0.28, 0.25, 0.19);
	vec3 c = mix(moss, stone, smoothstep(0.35, 0.75, n1));
	c = mix(c, dry, smoothstep(0.55, 0.8, n2) * 0.5);
	c *= 0.75 + 0.5 * n2;
	// a soft grid only near you, so the floor reads as a floor
	vec2 g = abs(fract(xz) - 0.5) / fwidth(xz);
	float line = 1.0 - min(min(g.x, g.y), 1.0);
	float near = 1.0 - smoothstep(3.0, 9.0, length(xz));
	ALBEDO = mix(c, vec3(0.35, 0.6, 0.8), line * 0.35 * near);
	ROUGHNESS = 0.92;
	SPECULAR = 0.2;
}
"""

const DISC_SHADER := """
shader_type spatial;
render_mode cull_disabled;
varying vec2 local;
void vertex() { local = VERTEX.xz; }
void fragment() {
	float r = length(local);
	if (r > 1.56) { discard; }
	// polished dark metal with fine rings, a glowing edge at the play circle
	float rings = 0.5 + 0.5 * sin(r * 160.0);
	ALBEDO = vec3(0.06, 0.07, 0.09) + vec3(0.02) * rings;
	METALLIC = 0.85;
	ROUGHNESS = 0.25 + 0.1 * rings;
	float edge = smoothstep(1.40, 1.48, r) * (1.0 - smoothstep(1.50, 1.56, r));
	EMISSION = vec3(0.25, 0.65, 1.0) * edge * 1.6;
}
"""


# What build() made, for the background switch: {env, sky, ground, mountains}.
static var parts := {}


static func build(root: Node3D) -> void:
	var env := Environment.new()
	env.background_mode = Environment.BG_SKY
	var sky := Sky.new()
	var sm := ProceduralSkyMaterial.new()
	sm.sky_top_color = Color(0.12, 0.2, 0.36)
	sm.sky_horizon_color = Color(0.62, 0.55, 0.52)
	sm.ground_bottom_color = Color(0.05, 0.06, 0.07)
	sm.ground_horizon_color = Color(0.36, 0.34, 0.36)
	sm.sun_angle_max = 20.0
	sky.sky_material = sm
	env.sky = sky
	env.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	env.ambient_light_energy = 0.7
	env.tonemap_mode = Environment.TONE_MAPPER_FILMIC
	env.fog_enabled = true
	env.fog_light_color = Color(0.5, 0.5, 0.55)
	env.fog_density = 0.006
	env.fog_sky_affect = 0.0
	var we := WorldEnvironment.new()
	we.environment = env
	root.add_child(we)
	parts["env"] = env
	parts["sky"] = sky

	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-14, -35, 0)
	sun.light_color = Color(1.0, 0.86, 0.72)
	sun.light_energy = 1.2
	root.add_child(sun)

	var ground := MeshInstance3D.new()
	var plane := PlaneMesh.new()
	plane.size = Vector2(400, 400)
	plane.subdivide_width = 260
	plane.subdivide_depth = 260
	ground.mesh = plane
	var gm := ShaderMaterial.new()
	var gs := Shader.new()
	gs.code = GROUND_SHADER
	gm.shader = gs
	ground.material_override = gm
	root.add_child(ground)
	parts["ground"] = ground

	var disc := MeshInstance3D.new()
	var dm := PlaneMesh.new()
	dm.size = Vector2(3.2, 3.2)
	disc.mesh = dm
	disc.position.y = 0.004
	var dsm := ShaderMaterial.new()
	var ds := Shader.new()
	ds.code = DISC_SHADER
	dsm.shader = ds
	disc.material_override = dsm
	root.add_child(disc)

	# distant mountains: low-poly cones around the horizon (seeded, so the same every start)
	var hills := Node3D.new()
	hills.name = "Mountains"
	root.add_child(hills)
	parts["mountains"] = hills
	var rng := RandomNumberGenerator.new()
	rng.seed = 2026
	var rock := StandardMaterial3D.new()
	rock.albedo_color = Color(0.2, 0.22, 0.27)
	rock.roughness = 1.0
	for i in 46:
		var a := TAU * i / 46.0 + rng.randf_range(-0.05, 0.05)
		var dist := rng.randf_range(110, 170)
		var h := rng.randf_range(18, 55)
		var cone := CylinderMesh.new()
		cone.top_radius = rng.randf_range(0.0, 3.0)
		cone.bottom_radius = h * rng.randf_range(0.8, 1.4)
		cone.height = h
		cone.radial_segments = rng.randi_range(5, 8)
		cone.rings = 1
		var m := MeshInstance3D.new()
		m.mesh = cone
		m.material_override = rock
		m.position = Vector3(sin(a) * dist, h / 2 - 1.0, cos(a) * dist)
		m.rotation.y = rng.randf() * TAU
		hills.add_child(m)


# The background (pibridge.conf home_background, the app's Home page chooses it; docs/home.md "Backgrounds"):
#   "" / passthrough / none   the world as built above
#   dusk                      the same world under a dusk sky
#   <id>                      a downloaded 360-degree picture (<data>/pibridge/backgrounds/<id>.ppm) as the sky; the
#                             plain and the mountains go (the picture has its own ground), the disc stays
static func apply_background(choice: String) -> String:
	if parts.is_empty():
		return "no world"
	var sky: Sky = parts["sky"]
	var env: Environment = parts["env"]
	var picture := choice not in ["", "passthrough", "none", "dusk"]
	parts["ground"].visible = not picture
	parts["mountains"].visible = not picture
	env.fog_enabled = not picture
	if picture:
		var img := load_ppm(Paths.data_home() + "/pibridge/backgrounds/" + choice + ".ppm")
		if img == null:
			apply_background("")
			return "The picture %s is not downloaded (the app's Home page downloads it)." % choice
		var pm := PanoramaSkyMaterial.new()
		pm.panorama = ImageTexture.create_from_image(img)
		sky.sky_material = pm
		return "Background: " + choice
	var sm := ProceduralSkyMaterial.new()
	if choice == "dusk":
		sm.sky_top_color = Color(0.06, 0.07, 0.18)
		sm.sky_horizon_color = Color(0.85, 0.42, 0.28)
		sm.ground_horizon_color = Color(0.3, 0.2, 0.22)
	else:
		sm.sky_top_color = Color(0.12, 0.2, 0.36)
		sm.sky_horizon_color = Color(0.62, 0.55, 0.52)
		sm.ground_horizon_color = Color(0.36, 0.34, 0.36)
	sm.ground_bottom_color = Color(0.05, 0.06, 0.07)
	sm.sun_angle_max = 20.0
	sky.sky_material = sm
	return "Background: " + ("dusk" if choice == "dusk" else "the PiBridge world")


# A binary PPM (P6, 8-bit RGB): the format pibridge-home's pictures are stored in.
static func load_ppm(path: String) -> Image:
	if not FileAccess.file_exists(path):
		return null
	var f := FileAccess.open(path, FileAccess.READ)
	var fields := []
	var tok := ""
	while fields.size() < 4 and not f.eof_reached():
		var c := char(f.get_8())
		if c == "#":
			while not f.eof_reached() and char(f.get_8()) != "\n":
				pass
		elif c in [" ", "\n", "\r", "\t"]:
			if tok != "":
				fields.append(tok)
				tok = ""
		else:
			tok += c
	if fields.size() < 4 or fields[0] != "P6" or int(fields[3]) != 255:
		return null
	var w := int(fields[1])
	var h := int(fields[2])
	var data := f.get_buffer(w * h * 3)
	if data.size() != w * h * 3:
		return null
	return Image.create_from_data(w, h, false, Image.FORMAT_RGB8, data)
