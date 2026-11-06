extends Node3D
## Common WorldInstance container that delegates to kind-specific loaders.

const MAP_ID := "cub_playground_v1"
const TERRAIN_LAYER := 1
const LIDAR_LAYER := 4
const FLOOR_MAX_ANGLE := deg_to_rad(46.0)
const DEFAULT_CLEARANCE_RADIUS := 0.28
const WorldRegistryScript = preload("res://scripts/world_registry.gd")
const OccupancyLoaderScript = preload("res://scripts/occupancy_loader.gd")
const LidarScript = preload("res://scripts/ros/lidar.gd")

var world_id := MAP_ID
var display_name := "Cub Playground"
var kind := "playground"
var world_hash := ""
var schema_version := 1
var definition: Dictionary = {}
var is_loaded := false
var load_error := ""
var active_spawn_preset := "start_a"
var occupancy_loader: RefCounted
var _environment_initialized := false
var _world_nodes: Array[Node] = []
var _playground_obstacles: Array[AABB] = []


func _ready() -> void:
	_ensure_environment()
	if not is_loaded and load_error.is_empty():
		load_world(world_id)


func _ensure_environment() -> void:
	if _environment_initialized:
		return
	_environment_initialized = true
	var environment := WorldEnvironment.new()
	environment.name = "WorldEnvironment"
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
	sun.name = "Sun"
	sun.rotation_degrees = Vector3(-55, -30, 0)
	sun.light_energy = 0.75
	sun.shadow_enabled = false
	add_child(sun)


func load_world(target_world_id: String, worlds_root := WorldRegistryScript.WORLDS_ROOT) -> bool:
	_ensure_environment()
	var loaded_def := WorldRegistryScript.load_world_definition(target_world_id, worlds_root)
	return load_from_definition(loaded_def)


func load_from_definition(loaded_def: Dictionary) -> bool:
	_ensure_environment()
	_clear_world_nodes()
	occupancy_loader = null
	_playground_obstacles.clear()
	definition = loaded_def.duplicate(true)
	world_id = str(definition.get("world_id", ""))
	display_name = str(definition.get("display_name", world_id))
	kind = str(definition.get("kind", ""))
	world_hash = str(definition.get("world_hash", ""))
	schema_version = int(definition.get("schema_version", 1))
	active_spawn_preset = str(definition.get("default_spawn", "start_a"))

	if not bool(definition.get("ok", false)):
		is_loaded = false
		load_error = str(definition.get("error", "Failed to load world definition."))
		return false

	if kind == "playground":
		_build_playground()
		is_loaded = true
		load_error = ""
		return true
	elif kind == "occupancy_extrusion":
		var loader := OccupancyLoaderScript.new()
		var container := Node3D.new()
		container.name = "OccupancyWorld"
		add_child(container)
		_world_nodes.append(container)
		var err: String = loader.build_into(
			container,
			definition,
			_material(Color("d6dfe2")),
			_material(Color("5d7586"))
		)
		if not err.is_empty():
			_clear_world_nodes()
			is_loaded = false
			load_error = err
			return false
		occupancy_loader = loader
		is_loaded = true
		load_error = ""
		return true

	is_loaded = false
	load_error = "Unsupported world kind '%s'." % kind
	return false


func _clear_world_nodes() -> void:
	for node: Node in _world_nodes:
		if is_instance_valid(node):
			if node.get_parent() == self:
				remove_child(node)
			node.free()
	_world_nodes.clear()


func _build_playground() -> void:
	_track(box(Vector3(50, 0.3, 50), Vector3(0, -0.15, 0), Color("c6d2d3"), true, false))
	_track(_grid())
	for x in [-25.0, 25.0]:
		_track(box(Vector3(0.3, 1.1, 50), Vector3(x, 0.55, 0), Color("7a91a1")))
	for z in [-25.0, 25.0]:
		_track(box(Vector3(50, 1.1, 0.3), Vector3(0, 0.55, z), Color("7a91a1")))
	for x in [-14.0, 14.0]:
		_track(box(Vector3(3.0, 1.4, 3.0), Vector3(x, 0.7, 8.0), Color("91a6b2")))
		_playground_obstacles.append(AABB(Vector3(x - 1.5, 0.0, 8.0 - 1.5), Vector3(3.0, 1.4, 3.0)))
		_track(box(Vector3(2.0, 2.0, 4.0), Vector3(x, 1.0, -17.0), Color("8198a8")))
		_playground_obstacles.append(AABB(Vector3(x - 1.0, 0.0, -17.0 - 2.0), Vector3(2.0, 2.0, 4.0)))
	# A ramp and a real staircase: no invisible ramp under the steps.
	var ramp := PackedVector3Array([
		Vector3(-10, 0, 0), Vector3(-6, 0, 0), Vector3(-6, 1.2, -8),
		Vector3(-10, 0, 0), Vector3(-6, 1.2, -8), Vector3(-10, 1.2, -8),
	])
	_track(terrain_mesh(ramp, Color("83b9b2")))
	_track(box(Vector3(4, 1.2, 3), Vector3(-8, 0.6, -9.5), Color("83b9b2"), true, false))
	for step in range(8):
		var height := float(step + 1) * 0.15
		_track(box(Vector3(3.0, height, 0.6), Vector3(6, height / 2, -float(step) * 0.6 - 0.3), Color("b6c9d4"), true, false))
	_track(box(Vector3(3, 1.2, 2), Vector3(6, 0.6, -5.8), Color("b6c9d4"), true, false))
	_track(_rough_mesh())
	_track(box(Vector3(2, 1.0, 2), Vector3(11, 0.5, -7), Color("af8878")))
	_playground_obstacles.append(AABB(Vector3(11.0 - 1.0, 0.0, -7.0 - 1.0), Vector3(2.0, 1.0, 2.0)))
	_track(_label("RAMP", Vector3(-8, 1.7, -10.5)))
	_track(_label("STEPS / 15 cm", Vector3(6, 1.7, -6.0)))
	_track(_label("ROUGH MESH", Vector3(-2, 0.8, -14)))
	_track(_label("CUB / SHARED PLAYGROUND", Vector3(0, 0.03, 9), true))


func _track(node: Node) -> Node:
	if node != null and node not in _world_nodes:
		_world_nodes.append(node)
	return node


func world_summary() -> Dictionary:
	return {
		"world_id": world_id,
		"display_name": display_name,
		"schema_version": schema_version,
		"kind": kind,
		"world_hash": world_hash,
	}


func get_world_bounds_godot() -> AABB:
	var bounds: Variant = definition.get("bounds", {})
	if bounds is Dictionary and bounds.get("godot_min") is Array and bounds.get("godot_max") is Array:
		var gmin: Array = bounds["godot_min"]
		var gmax: Array = bounds["godot_max"]
		if gmin.size() == 3 and gmax.size() == 3:
			var p_min := Vector3(float(gmin[0]), float(gmin[1]), float(gmin[2]))
			var p_max := Vector3(float(gmax[0]), float(gmax[1]), float(gmax[2]))
			return AABB(p_min, p_max - p_min)
	return AABB(Vector3(-25.0, 0.0, -25.0), Vector3(50.0, 2.0, 50.0))


func get_world_bounds_ros() -> Dictionary:
	var bounds: Variant = definition.get("bounds", {})
	if bounds is Dictionary and bounds.get("ros_min") is Array and bounds.get("ros_max") is Array:
		var rmin: Array = bounds["ros_min"]
		var rmax: Array = bounds["ros_max"]
		if rmin.size() == 3 and rmax.size() == 3:
			return {
				"min": Vector3(float(rmin[0]), float(rmin[1]), float(rmin[2])),
				"max": Vector3(float(rmax[0]), float(rmax[1]), float(rmax[2])),
			}
	return {"min": Vector3(-25.0, -25.0, 0.0), "max": Vector3(25.0, 25.0, 2.0)}


func get_ground_height_ros(ros_x: float, ros_y: float) -> Variant:
	if kind == "occupancy_extrusion" and occupancy_loader != null:
		return occupancy_loader.get_ground_height_ros(ros_x, ros_y)
	if absf(ros_x) <= 24.85 and absf(ros_y) <= 24.85:
		return 0.0
	return null


func validate_spawn_ros(ros_x: float, ros_y: float, ros_z: Variant = null, clearance_radius := DEFAULT_CLEARANCE_RADIUS) -> Dictionary:
	if not is_loaded:
		return {"ok": false, "error": "World '%s' is not loaded." % world_id}
	if not is_finite(ros_x) or not is_finite(ros_y):
		return {"ok": false, "error": "Spawn X/Y must be finite numbers."}
	if kind == "occupancy_extrusion" and occupancy_loader != null:
		return occupancy_loader.validate_spawn_ros(ros_x, ros_y, ros_z, clearance_radius)

	# Playground validation: inside outer walls (+-24.85 m inner face) and outside obstacle boxes.
	var limit := 24.85 - clearance_radius
	if absf(ros_x) > limit or absf(ros_y) > limit:
		return {
			"ok": false,
			"error": "Spawn (%.2f, %.2f) is outside world floor bounds." % [ros_x, ros_y],
		}
	if ros_z != null:
		var z_val := float(ros_z)
		if not is_finite(z_val) or z_val < -0.05 or z_val > 2.5:
			return {
				"ok": false,
				"error": "Spawn Z %.2f is out of valid range." % z_val,
			}
	var godot_xz := LidarScript.ros_to_godot(Vector3(ros_x, ros_y, 0.0))
	for box_aabb: AABB in _playground_obstacles:
		var nearest_x := clampf(godot_xz.x, box_aabb.position.x, box_aabb.end.x)
		var nearest_z := clampf(godot_xz.z, box_aabb.position.z, box_aabb.end.z)
		var dx := godot_xz.x - nearest_x
		var dz := godot_xz.z - nearest_z
		if dx * dx + dz * dz < (clearance_radius - 1e-5) * (clearance_radius - 1e-5):
			return {
				"ok": false,
				"error": "Spawn (%.2f, %.2f) collides with an obstacle." % [ros_x, ros_y],
			}
	return {"ok": true, "ground_z": 0.0}


func resolve_spawn(request: Dictionary = {}, is_primary := true, client_index := 0, fallback_preset := "") -> Dictionary:
	if not is_loaded:
		return {"ok": false, "error": "World '%s' is not loaded: %s" % [world_id, load_error]}
	var has_preset := request.has("spawn") and not str(request["spawn"]).strip_edges().is_empty()
	var has_x := request.has("x") or request.has("spawn-x")
	var has_y := request.has("y") or request.has("spawn-y")
	var has_z := request.has("z") or request.has("spawn-z")
	var has_yaw := request.has("yaw_deg") or request.has("spawn-yaw-deg")
	var has_coords := has_x or has_y or has_z or has_yaw

	if has_preset and has_coords:
		return {"ok": false, "error": "Cannot combine --spawn preset with --spawn-x/--spawn-y/--spawn-z/--spawn-yaw-deg."}

	if has_coords:
		if not (has_x and has_y):
			return {"ok": false, "error": "Coordinate spawn requires both --spawn-x and --spawn-y."}
		var raw_x: Variant = request.get("x", request.get("spawn-x"))
		var raw_y: Variant = request.get("y", request.get("spawn-y"))
		if not _is_finite_number(raw_x) or not _is_finite_number(raw_y):
			return {"ok": false, "error": "Spawn X and Y must be finite numbers."}
		var ros_x := float(raw_x)
		var ros_y := float(raw_y)
		var ros_z_arg: Variant = null
		if has_z:
			var raw_z: Variant = request.get("z", request.get("spawn-z"))
			if not _is_finite_number(raw_z):
				return {"ok": false, "error": "Spawn Z must be a finite number."}
			ros_z_arg = float(raw_z)
		var yaw_deg := 0.0
		if has_yaw:
			var raw_yaw: Variant = request.get("yaw_deg", request.get("spawn-yaw-deg"))
			if not _is_finite_number(raw_yaw):
				return {"ok": false, "error": "Spawn yaw must be a finite number."}
			yaw_deg = float(raw_yaw)

		var check := validate_spawn_ros(ros_x, ros_y, ros_z_arg)
		if not bool(check.get("ok", false)):
			return {"ok": false, "error": str(check.get("error", "Invalid spawn position."))}
		var ground_z := float(check.get("ground_z", 0.0))
		var final_ros_z := float(ros_z_arg) if ros_z_arg != null else (ground_z + 0.1)
		var ros_vec := Vector3(ros_x, ros_y, final_ros_z)
		var godot_pos := LidarScript.ros_to_godot(ros_vec)
		var godot_yaw := deg_to_rad(yaw_deg)
		return {
			"ok": true,
			"mode": "coordinates",
			"preset": "",
			"ros_pos": ros_vec,
			"ros_yaw_deg": yaw_deg,
			"godot_pos": godot_pos,
			"godot_yaw": godot_yaw,
		}

	var preset_name := str(request.get("spawn", "")).strip_edges()
	if preset_name.is_empty():
		preset_name = fallback_preset.strip_edges() if not fallback_preset.strip_edges().is_empty() else active_spawn_preset
	if preset_name.is_empty():
		preset_name = str(definition.get("default_spawn", "start_a"))

	var spawns: Variant = definition.get("spawns", {})
	if not (spawns is Dictionary) or not spawns.has(preset_name):
		return {
			"ok": false,
			"error": "Unknown spawn preset '%s' in world '%s'." % [preset_name, world_id],
		}
	var preset: Dictionary = spawns[preset_name]
	var entry: Dictionary = {}
	if is_primary:
		entry = preset.get("robot", {})
	else:
		var clients: Variant = preset.get("clients", [])
		if clients is Array and not clients.is_empty():
			entry = clients[posmod(client_index, clients.size())]
		else:
			entry = preset.get("robot", {})

	var rx := float(entry.get("x", 0.0))
	var ry := float(entry.get("y", 0.0))
	var rz := float(entry.get("z", 0.0))
	var ryaw_deg := float(entry.get("yaw_deg", 0.0))
	var preset_check := validate_spawn_ros(rx, ry, rz)
	if not bool(preset_check.get("ok", false)):
		return {"ok": false, "error": str(preset_check.get("error", "Invalid preset spawn."))}
	var final_z := rz + 0.1
	var ros_pos := Vector3(rx, ry, final_z)
	return {
		"ok": true,
		"mode": "preset",
		"preset": preset_name,
		"ros_pos": ros_pos,
		"ros_yaw_deg": ryaw_deg,
		"godot_pos": LidarScript.ros_to_godot(ros_pos),
		"godot_yaw": deg_to_rad(ryaw_deg),
	}


static func _is_finite_number(value: Variant) -> bool:
	if value is float or value is int:
		return is_finite(float(value))
	var text := str(value).strip_edges()
	return text.is_valid_float() and is_finite(float(text))


func box(size: Vector3, at: Vector3, color: Color, solid := true, lidar := true) -> StaticBody3D:
	var body := StaticBody3D.new()
	body.position = at
	body.collision_layer = (TERRAIN_LAYER if solid else 0) | (LIDAR_LAYER if solid and lidar else 0)
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
	body.collision_layer = TERRAIN_LAYER
	body.collision_mask = 0
	var shape := ConcavePolygonShape3D.new()
	shape.set_faces(vertices)
	shape.backface_collision = true
	var collision := CollisionShape3D.new()
	collision.shape = shape
	body.add_child(collision)
	var wall_faces := extract_wall_faces(vertices)
	if not wall_faces.is_empty():
		var wall_body := StaticBody3D.new()
		wall_body.name = "LidarWall"
		wall_body.collision_layer = LIDAR_LAYER
		wall_body.collision_mask = 0
		var wall_shape := ConcavePolygonShape3D.new()
		wall_shape.set_faces(wall_faces)
		wall_shape.backface_collision = true
		var wall_collision := CollisionShape3D.new()
		wall_collision.shape = wall_shape
		wall_body.add_child(wall_collision)
		body.add_child(wall_body)
	body.add_child(picture)
	add_child(body)
	return body


static func extract_wall_faces(vertices: PackedVector3Array, max_floor_angle := FLOOR_MAX_ANGLE) -> PackedVector3Array:
	var walls := PackedVector3Array()
	var min_floor_up := cos(max_floor_angle)
	var count := vertices.size() - (vertices.size() % 3)
	for index in range(0, count, 3):
		var a := vertices[index]
		var b := vertices[index + 1]
		var c := vertices[index + 2]
		var cross := (b - a).cross(c - a)
		if cross.length_squared() < 1e-10:
			continue
		if absf(cross.normalized().y) < min_floor_up:
			walls.append(a)
			walls.append(b)
			walls.append(c)
	return walls


func _rough_mesh() -> StaticBody3D:
	var vertices := PackedVector3Array()
	for x in range(10):
		for z in range(18):
			var a := _rough_point(x, z)
			var b := _rough_point(x + 1, z)
			var c := _rough_point(x + 1, z + 1)
			var d := _rough_point(x, z + 1)
			vertices.append_array(PackedVector3Array([a, b, c, a, c, d]))
	return terrain_mesh(vertices, Color("b9ba91"))


func _rough_point(x: int, z: int) -> Vector3:
	var height := 0.09 + 0.045 * sin(float(x) * 1.3) * cos(float(z) * 0.9)
	return Vector3(-4.0 + float(x) * 0.4, height, -7.0 - float(z) * 0.4)


func _material(color: Color) -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.roughness = 1.0
	material.cull_mode = BaseMaterial3D.CULL_DISABLED
	return material


func _grid() -> MeshInstance3D:
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
	return picture


func _label(text: String, at: Vector3, flat := false) -> Label3D:
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
	return label
