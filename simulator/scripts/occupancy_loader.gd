extends RefCounted
## Loader and coordinate/clearance solver for 2D occupancy_extrusion worlds.

const TERRAIN_LAYER := 1
const LIDAR_LAYER := 4
const DEFAULT_CLEARANCE_RADIUS := 0.28

var width := 0
var height := 0
var resolution := 0.2
var origin := Vector3.ZERO  # (origin_x, origin_y, origin_yaw_rad)
var floor_height := 0.0
var floor_thickness := 0.3
var wall_height := 0.5
var clearance_radius := DEFAULT_CLEARANCE_RADIUS
var image: Image
var chunk_count := 0
var rect_count := 0
var triangle_count := 0

var _cos_yaw := 1.0
var _sin_yaw := 0.0
var _base_gx := 0.0
var _base_gz := 0.0
var _du_gx := 0.0
var _du_gz := 0.0
var _dv_gx := 0.0
var _dv_gz := 0.0


func configure(definition: Dictionary) -> String:
	var source: Dictionary = definition.get("source", {})
	var extrusion: Dictionary = definition.get("extrusion", {})
	width = int(source.get("width", 0))
	height = int(source.get("height", 0))
	resolution = float(source.get("resolution", 0.2))
	var raw_origin: Variant = source.get("origin", [0.0, 0.0, 0.0])
	if not (raw_origin is Array) or raw_origin.size() != 3:
		return "Invalid origin array in world definition."
	origin = Vector3(float(raw_origin[0]), float(raw_origin[1]), float(raw_origin[2]))
	if width <= 0 or height <= 0 or resolution <= 0.0:
		return "Invalid occupancy dimensions or resolution."
	floor_height = float(extrusion.get("floor_height", 0.0))
	floor_thickness = float(extrusion.get("floor_thickness", 0.3))
	wall_height = float(extrusion.get("wall_height", 0.5))
	clearance_radius = float(extrusion.get("capsule_clearance_radius_m", DEFAULT_CLEARANCE_RADIUS))

	_cos_yaw = cos(origin.z)
	_sin_yaw = sin(origin.z)
	var map_h := float(height) * resolution
	_base_gx = -(origin.y + _cos_yaw * map_h)
	_base_gz = -(origin.x - _sin_yaw * map_h)
	_du_gx = -resolution * _sin_yaw
	_du_gz = -resolution * _cos_yaw
	_dv_gx = resolution * _cos_yaw
	_dv_gz = -resolution * _sin_yaw

	var base_dir := str(definition.get("base_dir", ""))
	var image_path := "%s/%s" % [base_dir, str(source.get("image", "source.png"))]
	var png_bytes := FileAccess.get_file_as_bytes(image_path)
	if png_bytes.is_empty():
		return "Could not read occupancy image: %s" % image_path
	image = Image.new()
	var err := image.load_png_from_buffer(png_bytes)
	if err != OK:
		return "Failed to decode PNG %s: %s" % [image_path, error_string(err)]
	if image.get_width() != width or image.get_height() != height:
		return "PNG dimensions (%dx%d) do not match world.json (%dx%d)." % [
			image.get_width(), image.get_height(), width, height
		]
	return ""


func build_into(parent: Node3D, definition: Dictionary, floor_material: Material, wall_material: Material) -> String:
	var err := configure(definition)
	if not err.is_empty():
		return err
	var base_dir := str(definition.get("base_dir", ""))
	var appearance: Dictionary = definition.get("appearance", {})
	var textured_floor := appearance.has("floor_texture")
	if textured_floor:
		var texture_path := "%s/%s" % [base_dir, str(appearance["floor_texture"])]
		var floor_image := Image.new()
		var texture_err := floor_image.load_png_from_buffer(FileAccess.get_file_as_bytes(texture_path))
		if texture_err != OK:
			return "Failed to decode floor texture %s: %s" % [texture_path, error_string(texture_err)]
		if floor_image.get_width() != width or floor_image.get_height() != height:
			return "Floor texture dimensions (%dx%d) do not match occupancy image (%dx%d)." % [
				floor_image.get_width(), floor_image.get_height(), width, height
			]
		var texture_material := StandardMaterial3D.new()
		# Decode the original PNG directly: no import-time resizing, lossy compression, or mipmaps.
		texture_material.albedo_texture = ImageTexture.create_from_image(floor_image)
		texture_material.texture_filter = BaseMaterial3D.TEXTURE_FILTER_NEAREST
		texture_material.texture_repeat = false
		texture_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		floor_material = texture_material
	var extrusion: Dictionary = definition.get("extrusion", {})
	var chunks_path := "%s/%s" % [base_dir, str(extrusion.get("chunks_file", "wall_chunks.json"))]
	var parsed_chunks: Variant = JSON.parse_string(FileAccess.get_file_as_string(chunks_path))
	if not (parsed_chunks is Dictionary) or not (parsed_chunks.get("chunks") is Array):
		return "Invalid wall_chunks.json in %s" % chunks_path

	_build_floor(parent, floor_material, textured_floor)
	var chunks_root := Node3D.new()
	chunks_root.name = "WallChunks"
	parent.add_child(chunks_root)

	chunk_count = 0
	rect_count = 0
	triangle_count = 0
	var chunks: Array = parsed_chunks["chunks"]
	for chunk_entry: Variant in chunks:
		if not (chunk_entry is Dictionary):
			continue
		var built := _build_chunk(chunk_entry, wall_material)
		if built != null:
			chunks_root.add_child(built)
			chunk_count += 1
	return ""


func _build_floor(parent: Node3D, material: Material, textured := false) -> StaticBody3D:
	var map_w := float(width) * resolution
	var map_h := float(height) * resolution
	var center_ros := local_to_ros(map_w * 0.5, map_h * 0.5)
	var center_y := floor_height - floor_thickness * 0.5
	var center_godot := Vector3(-center_ros.y, center_y, -center_ros.x)
	var size_godot := Vector3(map_h, floor_thickness, map_w)

	var body := StaticBody3D.new()
	body.name = "OccupancyFloor"
	body.position = center_godot
	body.rotation.y = origin.z
	body.collision_layer = TERRAIN_LAYER
	body.collision_mask = 0

	var shape := BoxShape3D.new()
	shape.size = size_godot
	var collision := CollisionShape3D.new()
	collision.shape = shape
	body.add_child(collision)

	var picture := MeshInstance3D.new()
	picture.name = "FloorSurface"
	if textured:
		picture.mesh = _floor_surface_mesh(size_godot)
	else:
		var mesh := BoxMesh.new()
		mesh.size = size_godot
		picture.mesh = mesh
	picture.material_override = material
	picture.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	body.add_child(picture)

	parent.add_child(body)
	return body


func _floor_surface_mesh(size: Vector3) -> ArrayMesh:
	# Image (u=0,v=0) is map-local (x=0,y=map_height). The body applies origin yaw.
	# A single top quad avoids BoxMesh's face atlas and keeps each pixel at exactly one cell.
	var half := size * 0.5
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = PackedVector3Array([
		Vector3(-half.x, half.y, half.z), Vector3(half.x, half.y, half.z),
		Vector3(half.x, half.y, -half.z), Vector3(-half.x, half.y, -half.z),
	])
	arrays[Mesh.ARRAY_NORMAL] = PackedVector3Array([Vector3.UP, Vector3.UP, Vector3.UP, Vector3.UP])
	arrays[Mesh.ARRAY_TEX_UV] = PackedVector2Array([Vector2(0, 0), Vector2(0, 1), Vector2(1, 1), Vector2(1, 0)])
	# Godot front faces use clockwise winding when viewed from above.
	arrays[Mesh.ARRAY_INDEX] = PackedInt32Array([0, 2, 1, 0, 3, 2])
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return mesh


func _build_chunk(chunk: Dictionary, material: Material) -> StaticBody3D:
	var rects: Array = chunk.get("rects", [])
	if rects.is_empty():
		return null
	var has_h_walls := chunk.has("h_walls")
	var has_v_walls := chunk.has("v_walls")
	var h_walls: Array = chunk.get("h_walls", [])
	var v_walls: Array = chunk.get("v_walls", [])

	if not has_h_walls and not has_v_walls:
		for rect: Variant in rects:
			if rect is Array and rect.size() >= 4:
				var u0 := int(rect[0])
				var v0 := int(rect[1])
				var rw := int(rect[2])
				var rh := int(rect[3])
				h_walls.append([u0, v0, rw, -1])
				h_walls.append([u0, v0 + rh, rw, 1])
				v_walls.append([u0, v0, rh, -1])
				v_walls.append([u0 + rw, v0, rh, 1])

	var quad_count := rects.size() + h_walls.size() + v_walls.size()
	if quad_count == 0:
		return null

	var vertices := PackedVector3Array()
	var normals := PackedVector3Array()
	var indices := PackedInt32Array()
	var collision_faces := PackedVector3Array()
	vertices.resize(quad_count * 4)
	normals.resize(quad_count * 4)
	indices.resize(quad_count * 6)
	collision_faces.resize(quad_count * 6)

	var v_idx := 0
	var i_idx := 0
	var y0 := floor_height
	var y1 := floor_height + wall_height

	var n_up := Vector3.UP
	var n_north := Vector3(-_dv_gx, 0.0, -_dv_gz).normalized()
	var n_south := -n_north
	var n_west := Vector3(-_du_gx, 0.0, -_du_gz).normalized()
	var n_east := -n_west

	# Top faces for each merged rectangle.
	for rect: Variant in rects:
		var u0 := float(rect[0])
		var v0 := float(rect[1])
		var u1 := u0 + float(rect[2])
		var v1 := v0 + float(rect[3])
		var p00 := _grid_point(u0, v0, y1)
		var p10 := _grid_point(u1, v0, y1)
		var p11 := _grid_point(u1, v1, y1)
		var p01 := _grid_point(u0, v1, y1)
		vertices[v_idx] = p00
		vertices[v_idx + 1] = p01
		vertices[v_idx + 2] = p11
		vertices[v_idx + 3] = p10
		normals[v_idx] = n_up
		normals[v_idx + 1] = n_up
		normals[v_idx + 2] = n_up
		normals[v_idx + 3] = n_up
		indices[i_idx] = v_idx
		indices[i_idx + 1] = v_idx + 1
		indices[i_idx + 2] = v_idx + 2
		indices[i_idx + 3] = v_idx
		indices[i_idx + 4] = v_idx + 2
		indices[i_idx + 5] = v_idx + 3
		collision_faces[i_idx] = p00
		collision_faces[i_idx + 1] = p01
		collision_faces[i_idx + 2] = p11
		collision_faces[i_idx + 3] = p00
		collision_faces[i_idx + 4] = p11
		collision_faces[i_idx + 5] = p10
		v_idx += 4
		i_idx += 6
	rect_count += rects.size()

	# Horizontal wall faces (along v = const).
	for seg: Variant in h_walls:
		var u0 := float(seg[0])
		var vg := float(seg[1])
		var u1 := u0 + float(seg[2])
		var dir := int(seg[3])
		var a0 := _grid_point(u0, vg, y0)
		var b0 := _grid_point(u1, vg, y0)
		var a1 := _grid_point(u0, vg, y1)
		var b1 := _grid_point(u1, vg, y1)
		var n := n_north if dir < 0 else n_south
		var c0 := a0 if dir < 0 else b0
		var c1 := a1 if dir < 0 else b1
		var c2 := b1 if dir < 0 else a1
		var c3 := b0 if dir < 0 else a0
		vertices[v_idx] = c0
		vertices[v_idx + 1] = c1
		vertices[v_idx + 2] = c2
		vertices[v_idx + 3] = c3
		normals[v_idx] = n
		normals[v_idx + 1] = n
		normals[v_idx + 2] = n
		normals[v_idx + 3] = n
		indices[i_idx] = v_idx
		indices[i_idx + 1] = v_idx + 1
		indices[i_idx + 2] = v_idx + 2
		indices[i_idx + 3] = v_idx
		indices[i_idx + 4] = v_idx + 2
		indices[i_idx + 5] = v_idx + 3
		collision_faces[i_idx] = c0
		collision_faces[i_idx + 1] = c1
		collision_faces[i_idx + 2] = c2
		collision_faces[i_idx + 3] = c0
		collision_faces[i_idx + 4] = c2
		collision_faces[i_idx + 5] = c3
		v_idx += 4
		i_idx += 6

	# Vertical wall faces (along u = const).
	for seg: Variant in v_walls:
		var ug := float(seg[0])
		var v0 := float(seg[1])
		var v1 := v0 + float(seg[2])
		var dir := int(seg[3])
		var a0 := _grid_point(ug, v0, y0)
		var b0 := _grid_point(ug, v1, y0)
		var a1 := _grid_point(ug, v0, y1)
		var b1 := _grid_point(ug, v1, y1)
		var n := n_west if dir < 0 else n_east
		var c0 := b0 if dir < 0 else a0
		var c1 := b1 if dir < 0 else a1
		var c2 := a1 if dir < 0 else b1
		var c3 := a0 if dir < 0 else b0
		vertices[v_idx] = c0
		vertices[v_idx + 1] = c1
		vertices[v_idx + 2] = c2
		vertices[v_idx + 3] = c3
		normals[v_idx] = n
		normals[v_idx + 1] = n
		normals[v_idx + 2] = n
		normals[v_idx + 3] = n
		indices[i_idx] = v_idx
		indices[i_idx + 1] = v_idx + 1
		indices[i_idx + 2] = v_idx + 2
		indices[i_idx + 3] = v_idx
		indices[i_idx + 4] = v_idx + 2
		indices[i_idx + 5] = v_idx + 3
		collision_faces[i_idx] = c0
		collision_faces[i_idx + 1] = c1
		collision_faces[i_idx + 2] = c2
		collision_faces[i_idx + 3] = c0
		collision_faces[i_idx + 4] = c2
		collision_faces[i_idx + 5] = c3
		v_idx += 4
		i_idx += 6

	triangle_count += quad_count * 2

	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_INDEX] = indices
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)

	var body := StaticBody3D.new()
	body.name = "WallChunk_%d_%d" % [int(chunk.get("cx", 0)), int(chunk.get("cy", 0))]
	body.collision_layer = TERRAIN_LAYER | LIDAR_LAYER
	body.collision_mask = 0

	var shape := ConcavePolygonShape3D.new()
	shape.set_faces(collision_faces)
	shape.backface_collision = true
	var collision := CollisionShape3D.new()
	collision.shape = shape
	body.add_child(collision)

	var picture := MeshInstance3D.new()
	picture.mesh = mesh
	picture.material_override = material
	picture.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	body.add_child(picture)
	return body


func _grid_point(u_edge: float, v_edge: float, y: float) -> Vector3:
	return Vector3(
		_base_gx + u_edge * _du_gx + v_edge * _dv_gx,
		y,
		_base_gz + u_edge * _du_gz + v_edge * _dv_gz
	)


func local_to_ros(local_x: float, local_y: float) -> Vector2:
	return Vector2(
		origin.x + _cos_yaw * local_x - _sin_yaw * local_y,
		origin.y + _sin_yaw * local_x + _cos_yaw * local_y
	)


func ros_to_local(ros_x: float, ros_y: float) -> Vector2:
	var dx := ros_x - origin.x
	var dy := ros_y - origin.y
	return Vector2(
		_cos_yaw * dx + _sin_yaw * dy,
		-_sin_yaw * dx + _cos_yaw * dy
	)


func cell_center_to_ros(u: int, v: int) -> Vector2:
	var local_x := (float(u) + 0.5) * resolution
	var local_y := (float(height - v) - 0.5) * resolution
	return local_to_ros(local_x, local_y)


func ros_to_cell(ros_x: float, ros_y: float) -> Vector2i:
	var local := ros_to_local(ros_x, ros_y)
	var u := int(floor(local.x / resolution))
	var v := int(floor(float(height) - (local.y / resolution)))
	return Vector2i(u, v)


func pixel_value(u: int, v: int) -> int:
	if image == null or u < 0 or u >= width or v < 0 or v >= height:
		return -1
	return int(round(image.get_pixel(u, v).r * 255.0))


func is_cell_occupied(u: int, v: int) -> bool:
	return pixel_value(u, v) == 0


func get_ground_height_ros(ros_x: float, ros_y: float) -> Variant:
	var local := ros_to_local(ros_x, ros_y)
	var map_w := float(width) * resolution
	var map_h := float(height) * resolution
	if local.x < 0.0 or local.y < 0.0 or local.x > map_w or local.y > map_h:
		return null
	return floor_height


func validate_spawn_ros(ros_x: float, ros_y: float, ros_z: Variant = null, radius := -1.0) -> Dictionary:
	if not is_finite(ros_x) or not is_finite(ros_y):
		return {"ok": false, "error": "Spawn X/Y must be finite numbers."}
	var check_radius := clearance_radius if radius < 0.0 else radius
	var local := ros_to_local(ros_x, ros_y)
	var map_w := float(width) * resolution
	var map_h := float(height) * resolution
	if (
		local.x < check_radius
		or local.y < check_radius
		or local.x > map_w - check_radius
		or local.y > map_h - check_radius
	):
		return {
			"ok": false,
			"error": "Spawn (%.2f, %.2f) is outside world floor bounds." % [ros_x, ros_y],
		}
	if ros_z != null:
		var z_val := float(ros_z)
		if not is_finite(z_val) or z_val < floor_height - 0.05 or z_val > floor_height + 2.0:
			return {
				"ok": false,
				"error": "Spawn Z %.2f is out of valid range for floor height %.2f." % [z_val, floor_height],
			}
	if image == null:
		return {"ok": false, "error": "Occupancy image is not loaded."}

	var u_center := local.x / resolution
	var v_center := float(height) - (local.y / resolution)
	var cell_span := int(ceil(check_radius / resolution)) + 1
	var u_min := maxi(0, int(floor(u_center)) - cell_span)
	var u_max := mini(width - 1, int(floor(u_center)) + cell_span)
	var v_min := maxi(0, int(floor(v_center)) - cell_span)
	var v_max := mini(height - 1, int(floor(v_center)) + cell_span)
	var radius_sq := (check_radius - 1e-5) * (check_radius - 1e-5)

	for v in range(v_min, v_max + 1):
		var cell_y0 := float(height - (v + 1)) * resolution
		var cell_y1 := float(height - v) * resolution
		for u in range(u_min, u_max + 1):
			if not is_cell_occupied(u, v):
				continue
			var cell_x0 := float(u) * resolution
			var cell_x1 := float(u + 1) * resolution
			var nearest_x := clampf(local.x, cell_x0, cell_x1)
			var nearest_y := clampf(local.y, cell_y0, cell_y1)
			var dx := local.x - nearest_x
			var dy := local.y - nearest_y
			if dx * dx + dy * dy < radius_sq:
				return {
					"ok": false,
					"error": "Spawn (%.2f, %.2f) collides with or is too close to wall cell (%d, %d)." % [
						ros_x, ros_y, u, v
					],
				}
	return {"ok": true, "ground_z": floor_height}

