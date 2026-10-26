extends CharacterBody3D
## Only the controlling app simulates this body. Remote bodies display snapshots.

const TERRAIN_LAYER := 1
const STEP_HEIGHT := 0.24
const SPEEDS := {"robot": 2.5, "pedestrian": 3.0, "cone": 2.0}
const PoseBufferScript = preload("res://scripts/pose_buffer.gd")
const Cub4Scene = preload("res://assets/cub4/cub4.glb")

var actor_id: int
var owner_id: int
var kind := "robot"
var nickname := ""
var primary := false
var controlled := false
var simulated := false
var target_position := Vector3.ZERO
var target_yaw := 0.0
var body_height := 0.9
var received_updates := 0
var _visual: Node3D
var _shape: CollisionShape3D
var _caption: Label3D
var _color := Color("3d85c6")
var _display_height := 0.0
var _step_snap_cooldown := 0.0
var _poses := PoseBufferScript.new()


func configure(record: Dictionary, local_control: bool) -> void:
	actor_id = int(record["id"])
	owner_id = int(record["owner"])
	kind = str(record["kind"])
	nickname = str(record["nickname"])
	primary = bool(record["primary"])
	_color = Color(record["color"])
	position = record["position"]
	rotation.y = float(record["yaw"])
	target_position = position
	target_yaw = rotation.y
	_poses.reset(position, rotation.y, float(record.get("sample_time", 0.0)))
	collision_layer = 2
	collision_mask = TERRAIN_LAYER
	floor_snap_length = 0.30
	floor_stop_on_slope = true
	floor_constant_speed = true
	floor_max_angle = deg_to_rad(46.0)
	safe_margin = 0.008
	_shape = CollisionShape3D.new()
	var capsule := CapsuleShape3D.new()
	body_height = 1.7 if kind == "pedestrian" else (0.65 if kind == "cone" else 0.9)
	capsule.radius = 0.23 if kind == "pedestrian" else 0.28
	capsule.height = body_height
	_shape.shape = capsule
	_shape.position.y = body_height * 0.5
	add_child(_shape)
	_visual = Node3D.new()
	add_child(_visual)
	_build_visual()
	_caption = Label3D.new()
	_caption.text = "%s%s" % [nickname, " / CUB" if primary else ""]
	_caption.position.y = body_height + 0.35
	_caption.font_size = 34
	_caption.pixel_size = 0.006
	_caption.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	_caption.modulate = Color("25374a")
	_caption.outline_modulate = Color("f7fafc")
	_caption.outline_size = 9
	add_child(_caption)
	set_controlled(local_control)


func set_controlled(value: bool, simulate_locally := false) -> void:
	controlled = value
	simulated = value or simulate_locally
	collision_mask = TERRAIN_LAYER if simulated else 0
	velocity = Vector3.ZERO
	if _caption:
		_caption.text = "%s%s%s" % [nickname, " / CUB" if primary else "", " [YOU]" if value else ""]


func simulate(input: Vector2, camera_yaw: float, fast: bool, delta: float) -> void:
	if primary:
		return  # The primary robot accepts only ROS velocity commands.
	var speed: float = SPEEDS[kind] * (1.6 if fast else 1.0)
	var horizontal := Vector3.ZERO
	if kind == "robot":
		rotation.y -= input.x * 1.9 * delta
		horizontal = -global_basis.z * input.y * speed
	else:
		horizontal = Basis(Vector3.UP, camera_yaw) * Vector3(input.x, 0.0, -input.y) * speed
		if horizontal.length_squared() > 0.01:
			rotation.y = atan2(-horizontal.x, -horizontal.z)
	_move(horizontal, delta)


func simulate_velocity(forward: float, turn: float, delta: float) -> void:
	rotation.y += turn * delta
	_move(-global_basis.z * forward, delta)


func _move(horizontal: Vector3, delta: float) -> void:
	_step_snap_cooldown = maxf(0.0, _step_snap_cooldown - delta)
	floor_snap_length = 0.0 if _step_snap_cooldown > 0.0 else 0.30
	velocity.x = horizontal.x
	velocity.z = horizontal.z
	if is_on_floor():
		velocity.y = -0.1
	else:
		velocity.y -= 18.0 * delta
	# Test the full capsule path before climbing; a ray alone misses low ceilings.
	if is_on_floor() and horizontal.length_squared() > 0.01:
		_try_step(horizontal * delta)
	move_and_slide()
	if position.y < -12.0:
		position = Vector3(0.0, 0.1, 4.0)
		velocity = Vector3.ZERO
	# Smooth only the picture; simulation and network state keep the actual height.
	_display_height = move_toward(_display_height, 0.0, delta * 2.5)
	_visual.position.y = _display_height


func _try_step(motion: Vector3) -> void:
	var obstacle := KinematicCollision3D.new()
	if not test_move(global_transform, motion, obstacle, safe_margin):
		return
	# Ordinary slopes belong to move_and_slide(), not the step-up path.
	if obstacle.get_normal().dot(Vector3.UP) >= cos(floor_max_angle):
		return
	var lifted := global_transform
	var up := Vector3.UP * STEP_HEIGHT
	if test_move(lifted, up, null, safe_margin):
		return
	lifted.origin += up
	if test_move(lifted, motion, null, safe_margin):
		return
	lifted.origin += motion
	var landing := KinematicCollision3D.new()
	if not test_move(lifted, Vector3.DOWN * (STEP_HEIGHT + 0.03), landing, safe_margin):
		return
	# A capsule touching a stair edge has a diagonal contact normal. Sample the
	# actual tread just beyond that edge rather than treating it as a steep wall.
	var probe := obstacle.get_position() + motion.normalized() * 0.04
	probe.y = global_position.y + STEP_HEIGHT + 0.03
	var ground := get_world_3d().direct_space_state.intersect_ray(
		PhysicsRayQueryParameters3D.create(probe, probe + Vector3.DOWN * (STEP_HEIGHT + 0.06), TERRAIN_LAYER))
	if ground.is_empty() or ground["normal"].dot(Vector3.UP) < cos(floor_max_angle):
		return
	var rise: float = ground["position"].y + safe_margin - global_position.y
	if rise < 0.015 or rise > STEP_HEIGHT + 0.001:
		return
	global_position.y += rise
	_display_height -= rise
	velocity.y = 0.0
	# Do not immediately snap back to the lower floor while the rounded capsule
	# crosses the edge. Gravity resumes normally during this short interval.
	_step_snap_cooldown = 0.14
	floor_snap_length = 0.0


func receive_pose(new_position: Vector3, new_yaw: float, sample_time: float) -> void:
	target_position = new_position
	target_yaw = new_yaw
	received_updates += 1
	if not simulated:
		_poses.push(new_position, new_yaw, sample_time)


func interpolate_remote(delta: float) -> void:
	if simulated:
		return
	_poses.advance(delta)
	position = _poses.position
	rotation.y = _poses.yaw


func _material(color: Color) -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.roughness = 1.0
	return material


func _part(mesh: Mesh, at: Vector3, color: Color) -> MeshInstance3D:
	var part := MeshInstance3D.new()
	part.mesh = mesh
	part.material_override = _material(color)
	part.position = at
	part.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	_visual.add_child(part)
	return part


func _box(size: Vector3, at: Vector3, color: Color) -> void:
	var mesh := BoxMesh.new()
	mesh.size = size
	_part(mesh, at, color)


func _build_visual() -> void:
	if primary and kind == "robot":
		var model := Cub4Scene.instantiate()
		model.name = "Cub4"
		_visual.add_child(model)
	elif kind == "pedestrian":
		var torso := CapsuleMesh.new()
		torso.radius = 0.22
		torso.height = 0.85
		torso.radial_segments = 8
		torso.rings = 3
		_part(torso, Vector3(0, 1.0, 0), _color)
		var head := SphereMesh.new()
		head.radius = 0.19
		head.height = 0.38
		head.radial_segments = 8
		head.rings = 4
		_part(head, Vector3(0, 1.51, 0), Color("efc19a"))
		_box(Vector3(0.13, 0.56, 0.16), Vector3(-0.12, 0.30, 0), Color("344559"))
		_box(Vector3(0.13, 0.56, 0.16), Vector3(0.12, 0.30, 0), Color("344559"))
		_box(Vector3(0.23, 0.08, 0.04), Vector3(0, 1.52, -0.18), Color("344559"))
	elif kind == "cone":
		_box(Vector3(0.62, 0.06, 0.62), Vector3(0, 0.03, 0), Color("344559"))
		var cone := CylinderMesh.new()
		cone.top_radius = 0.035
		cone.bottom_radius = 0.24
		cone.height = 0.56
		cone.radial_segments = 10
		_part(cone, Vector3(0, 0.34, 0), Color("f28a36"))
		var stripe := CylinderMesh.new()
		stripe.top_radius = 0.12
		stripe.bottom_radius = 0.16
		stripe.height = 0.10
		stripe.radial_segments = 10
		_part(stripe, Vector3(0, 0.34, 0), Color("f7fafc"))
	else:
		_box(Vector3(0.65, 0.46, 0.75), Vector3(0, 0.43, 0), _color)
		_box(Vector3(0.52, 0.11, 0.62), Vector3(0, 0.72, 0), Color("f3f7fa"))
		_box(Vector3(0.42, 0.12, 0.04), Vector3(0, 0.51, -0.39), Color("76dfce"))
		for x in [-0.37, 0.37]:
			for z in [-0.23, 0.23]:
				var wheel := CylinderMesh.new()
				wheel.top_radius = 0.17
				wheel.bottom_radius = 0.17
				wheel.height = 0.10
				wheel.radial_segments = 10
				var part := _part(wheel, Vector3(x, 0.19, z), Color("344559"))
				part.rotation.z = PI * 0.5
		var lidar := CylinderMesh.new()
		lidar.top_radius = 0.09
		lidar.bottom_radius = 0.09
		lidar.height = 0.14
		lidar.radial_segments = 8
		_part(lidar, Vector3(0, 0.84, -0.1), Color("344559"))
