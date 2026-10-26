extends Node3D
## All instances build the same tiny static map. Later replace its mesh resources.

const MAP_ID := "cub_playground_v1"


func _ready() -> void:
	var environment := WorldEnvironment.new()
	var settings := Environment.new()
	settings.background_mode = Environment.BG_COLOR
	settings.background_color = Color("cbdbe8")
	settings.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	settings.ambient_light_color = Color("dce8f4")
	settings.ambient_light_energy = 0.8
	settings.tonemap_mode = Environment.TONE_MAPPER_LINEAR
	environment.environment = settings
	add_child(environment)
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-55, -30, 0)
	sun.light_energy = 0.75
	sun.shadow_enabled = false
	add_child(sun)
	box(Vector3(50, 0.3, 50), Vector3(0, -0.15, 0), Color("c6d2d3"))
	_grid()
	for x in [-25.0, 25.0]:
		box(Vector3(0.3, 1.1, 50), Vector3(x, 0.55, 0), Color("7a91a1"))
	for z in [-25.0, 25.0]:
		box(Vector3(50, 1.1, 0.3), Vector3(0, 0.55, z), Color("7a91a1"))
	for x in [-14.0, 14.0]:
		box(Vector3(3.0, 1.4, 3.0), Vector3(x, 0.7, 8.0), Color("91a6b2"))
		box(Vector3(2.0, 2.0, 4.0), Vector3(x, 1.0, -17.0), Color("8198a8"))
	# A ramp and a real staircase: no invisible ramp under the steps.
	var ramp := PackedVector3Array([
		Vector3(-10, 0, 0), Vector3(-6, 0, 0), Vector3(-6, 1.2, -8),
		Vector3(-10, 0, 0), Vector3(-6, 1.2, -8), Vector3(-10, 1.2, -8),
	])
	terrain_mesh(ramp, Color("83b9b2"))
	box(Vector3(4, 1.2, 3), Vector3(-8, 0.6, -9.5), Color("83b9b2"))
	for step in range(8):
		var height := float(step + 1) * 0.15
		box(Vector3(3.0, height, 0.6), Vector3(6, height / 2, -float(step) * 0.6 - 0.3), Color("b6c9d4"))
	box(Vector3(3, 1.2, 2), Vector3(6, 0.6, -5.8), Color("b6c9d4"))
	_rough_mesh()
	box(Vector3(2, 1.0, 2), Vector3(11, 0.5, -7), Color("af8878"))
	_label("RAMP", Vector3(-8, 1.7, -10.5))
	_label("STEPS / 15 cm", Vector3(6, 1.7, -6.0))
	_label("ROUGH MESH", Vector3(-2, 0.8, -14))
	_label("CUB / SHARED PLAYGROUND", Vector3(0, 0.03, 9), true)


func box(size: Vector3, at: Vector3, color: Color, solid := true) -> StaticBody3D:
	var body := StaticBody3D.new()
	body.position = at
	body.collision_layer = 1 if solid else 0
	body.collision_mask = 0
	var shape := BoxShape3D.new()
	shape.size = size
	var collision := CollisionShape3D.new()
	collision.shape = shape
	if solid:
		body.add_child(collision)
	var mesh := BoxMesh.new()
	mesh.size = size
	var picture := MeshInstance3D.new()
	picture.mesh = mesh
	picture.material_override = _material(color)
	picture.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	body.add_child(picture)
	add_child(body)
	return body


func terrain_mesh(vertices: PackedVector3Array, color: Color) -> StaticBody3D:
	var surface := SurfaceTool.new()
	surface.begin(Mesh.PRIMITIVE_TRIANGLES)
	for vertex in vertices:
		surface.add_vertex(vertex)
	surface.generate_normals()
	var mesh := surface.commit()
	var picture := MeshInstance3D.new()
	picture.mesh = mesh
	picture.material_override = _material(color)
	picture.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	var body := StaticBody3D.new()
	body.collision_layer = 1
	body.collision_mask = 0
	var shape := ConcavePolygonShape3D.new()
	shape.set_faces(vertices)
	shape.backface_collision = true
	var collision := CollisionShape3D.new()
	collision.shape = shape
	body.add_child(collision)
	body.add_child(picture)
	add_child(body)
	return body


func _rough_mesh() -> void:
	var vertices := PackedVector3Array()
	for x in range(10):
		for z in range(18):
			var a := _rough_point(x, z)
			var b := _rough_point(x + 1, z)
			var c := _rough_point(x + 1, z + 1)
			var d := _rough_point(x, z + 1)
			vertices.append_array(PackedVector3Array([a, b, c, a, c, d]))
	terrain_mesh(vertices, Color("b9ba91"))


func _rough_point(x: int, z: int) -> Vector3:
	var height := 0.09 + 0.045 * sin(float(x) * 1.3) * cos(float(z) * 0.9)
	return Vector3(-4.0 + float(x) * 0.4, height, -7.0 - float(z) * 0.4)


func _material(color: Color) -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.roughness = 1.0
	material.cull_mode = BaseMaterial3D.CULL_DISABLED
	return material


func _grid() -> void:
	var mesh := ImmediateMesh.new()
	mesh.surface_begin(Mesh.PRIMITIVE_LINES)
	for line in range(-24, 25):
		mesh.surface_add_vertex(Vector3(line, 0.005, -24))
		mesh.surface_add_vertex(Vector3(line, 0.005, 24))
		mesh.surface_add_vertex(Vector3(-24, 0.005, line))
		mesh.surface_add_vertex(Vector3(24, 0.005, line))
	mesh.surface_end()
	var picture := MeshInstance3D.new()
	picture.mesh = mesh
	var material := _material(Color("aabdbf"))
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	picture.material_override = material
	add_child(picture)


func _label(text: String, at: Vector3, flat := false) -> void:
	var label := Label3D.new()
	label.text = text
	label.font_size = 48 if flat else 32
	label.pixel_size = 0.012
	label.modulate = Color("506b7b")
	label.outline_size = 0
	label.position = at
	if flat:
		label.rotation.x = -PI * 0.5
	else:
		label.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	add_child(label)
