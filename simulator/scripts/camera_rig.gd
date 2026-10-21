extends Node3D

var camera: Camera3D
var mode := 0
var yaw := 0.55
var pitch := 0.55
var distance := 9.0
var target: Node3D
var center := Vector3(0, 0.7, 1)
var look_enabled := false
var mouse_sensitivity := 0.003
var _drag_button := 0
var _rotating := false
var _drag_anchor := Vector3.ZERO


func _ready() -> void:
	camera = Camera3D.new()
	camera.current = true
	camera.fov = 62.0
	camera.near = 0.1
	camera.far = 100.0
	add_child(camera)


func select_mode(value: int) -> void:
	var previous := mode
	reset_drag()
	mode = value
	if mode == 0:
		pitch = 0.50
		distance = 8.0
		recenter()
	elif mode == 1:
		pitch = 0.80
		if previous == 0:
			distance = 24.0
			recenter()
	elif previous == 0:
		distance = 26.0
		recenter()
	_place_camera()


func recenter() -> void:
	reset_drag()
	if is_instance_valid(target):
		center = target.global_position + Vector3.UP * 0.8
	_place_camera()


func update(delta: float) -> void:
	if not camera:
		return
	if mode == 0 and is_instance_valid(target):
		center = center.lerp(target.global_position + Vector3.UP * 0.8, 1.0 - exp(-12.0 * delta))
	_place_camera()


func _place_camera() -> void:
	if not camera:
		return
	if mode == 2:
		camera.projection = Camera3D.PROJECTION_ORTHOGONAL
		camera.size = distance
		camera.global_position = center + Vector3.UP * 40.0
		camera.look_at(center, Vector3.FORWARD)
	else:
		camera.projection = Camera3D.PROJECTION_PERSPECTIVE
		var offset := Vector3(sin(yaw) * cos(pitch), sin(pitch), cos(yaw) * cos(pitch)) * distance
		var desired := center + offset
		if mode == 0:
			var query := PhysicsRayQueryParameters3D.create(center, desired, 1)
			var hit := get_world_3d().direct_space_state.intersect_ray(query)
			if not hit.is_empty():
				desired = hit["position"] + hit["normal"] * 0.15
		camera.global_position = desired
		camera.look_at(center, Vector3.UP)


func pan(direction: Vector2, delta: float) -> void:
	if mode == 0 or direction.is_zero_approx():
		return
	var orientation := yaw if mode == 1 else 0.0
	center += Basis(Vector3.UP, orientation) * Vector3(direction.x, 0, direction.y) * distance * delta * 0.55


func _unhandled_input(event: InputEvent) -> void:
	if not look_enabled:
		return
	if event is InputEventMouseButton:
		if event.pressed and event.button_index in [MOUSE_BUTTON_WHEEL_UP, MOUSE_BUTTON_WHEEL_DOWN]:
			var steps: float = event.factor if event.factor > 0.0 else 1.0
			var factor: float = pow(0.8, steps if event.button_index == MOUSE_BUTTON_WHEEL_UP else -steps)
			_zoom_at(event.position, factor)
		elif mode != 0:
			_map_button(event)
	elif event is InputEventMouseMotion:
		if mode == 0:
			# Captured TPS input uses pixels independent of viewport stretch.
			yaw -= event.screen_relative.x * mouse_sensitivity
			pitch = clampf(pitch + event.screen_relative.y * mouse_sensitivity, -0.25, 1.45)
		else:
			_map_motion(event)


func reset_drag() -> void:
	_drag_button = 0
	_rotating = false


func _map_button(event: InputEventMouseButton) -> void:
	if not event.pressed:
		if event.button_index == _drag_button:
			reset_drag()
		return
	if event.button_index == MOUSE_BUTTON_LEFT and event.double_click and not event.ctrl_pressed:
		reset_drag()
		_zoom_at(event.position, 0.5)
		return
	if event.button_index != MOUSE_BUTTON_LEFT and not (mode == 1 and event.button_index == MOUSE_BUTTON_MIDDLE):
		return
	reset_drag()
	if mode == 1 and (event.ctrl_pressed or event.button_index == MOUSE_BUTTON_MIDDLE):
		_rotating = true
	else:
		var anchor: Variant = _terrain_point(event.position)
		if anchor == null:
			return
		_drag_anchor = anchor
	_drag_button = event.button_index


func _map_motion(event: InputEventMouseMotion) -> void:
	if _drag_button == 0:
		return
	var mask := MOUSE_BUTTON_MASK_LEFT if _drag_button == MOUSE_BUTTON_LEFT else MOUSE_BUTTON_MASK_MIDDLE
	if not (event.button_mask & mask):
		reset_drag()
		return
	if _rotating:
		yaw -= event.relative.x * mouse_sensitivity
		pitch = clampf(pitch + event.relative.y * mouse_sensitivity, 0.25, 1.50)
	else:
		# Hold the picked terrain point under the pointer. A fixed plane through
		# that point avoids jumps at mesh seams, steps and the edge of the map.
		var at: Variant = _plane_point(event.position, _drag_anchor.y)
		if at != null:
			center += _drag_anchor - at
	_place_camera()


func _zoom_at(screen_position: Vector2, factor: float) -> void:
	var anchor: Variant = _terrain_point(screen_position) if mode != 0 else null
	distance = clampf(distance * factor, 2.0, 65.0)
	_place_camera()
	if anchor != null:
		var at: Variant = _plane_point(screen_position, anchor.y)
		if at != null:
			center += anchor - at
			_place_camera()
	# A wheel event during a drag starts the next drag segment at this point.
	if _drag_button != 0 and not _rotating:
		var at: Variant = _plane_point(screen_position, _drag_anchor.y)
		if at != null:
			_drag_anchor = at


func _plane_point(screen_position: Vector2, height: float) -> Variant:
	var origin := camera.project_ray_origin(screen_position)
	var at: Variant = Plane(Vector3.UP, height).intersects_ray(origin, camera.project_ray_normal(screen_position))
	# Ignore sky and near-horizon intersections rather than jumping far away.
	return at if at != null and origin.distance_squared_to(at) <= 300.0 * 300.0 else null


func _terrain_point(screen_position: Vector2) -> Variant:
	var origin := camera.project_ray_origin(screen_position)
	var direction := camera.project_ray_normal(screen_position)
	var query := PhysicsRayQueryParameters3D.create(origin, origin + direction * 300.0, 1)
	var hit := get_world_3d().direct_space_state.intersect_ray(query)
	return hit["position"] if not hit.is_empty() else _plane_point(screen_position, center.y)
