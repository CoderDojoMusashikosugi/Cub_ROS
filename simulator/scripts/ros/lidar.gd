extends RefCounted
## Host-only 2D horizontal raycast LiDAR using the URDF sensor mount.

const LIDAR_LAYER := 4
const DEFAULT_FRAME_ID := "velodyne"
const DEFAULT_JOINT := "base_to_velodyne"
const DEFAULT_OFFSET := Vector3(0.0, 0.2492, 0.0)
const RAY_COUNT := 360
const ANGLE_MIN := -PI
const ANGLE_INCREMENT := TAU / float(RAY_COUNT)
const ANGLE_MAX := ANGLE_MIN + float(RAY_COUNT - 1) * ANGLE_INCREMENT
const SCAN_INTERVAL := 0.1
const RANGE_MIN := 0.05
const RANGE_MAX := 30.0
const URDF_RELATIVE_PATH := "../ros/cub_description/urdf/cub4.urdf"
const METADATA_PATH := "res://assets/cub4/cub4.json"

var frame_id := DEFAULT_FRAME_ID
var joint_name := DEFAULT_JOINT
var parent_frame := "base_link"
var urdf_xyz := Vector3(0.0, 0.0, 0.2492)
var urdf_rpy := Vector3.ZERO
var offset := DEFAULT_OFFSET
var yaw_offset := 0.0
var range_min := RANGE_MIN
var range_max := RANGE_MAX
var source := "default"
var last_scan: Dictionary = {}
var _query := PhysicsRayQueryParameters3D.new()


func _init(urdf_path := "", metadata_path := METADATA_PATH) -> void:
	_query.collision_mask = LIDAR_LAYER
	_query.hit_from_inside = false
	configure_from_urdf(urdf_path, metadata_path)


func configure_from_urdf(urdf_path := "", metadata_path := METADATA_PATH) -> Dictionary:
	var mount := load_urdf_mount(urdf_path, metadata_path)
	frame_id = str(mount["frame_id"])
	joint_name = str(mount["joint"])
	parent_frame = str(mount["parent"])
	urdf_xyz = mount["xyz"]
	urdf_rpy = mount["rpy"]
	offset = mount["offset"]
	yaw_offset = float(mount["yaw"])
	source = str(mount["source"])
	return mount


func sample(primary: Node3D, actors: Dictionary = {}, stamp: Dictionary = {}) -> Dictionary:
	if not is_instance_valid(primary) or primary.get_world_3d() == null:
		return {}
	if primary.has_method("sync_sensor_pose"):
		primary.sync_sensor_pose()
	for actor: Variant in actors.values():
		if is_instance_valid(actor) and actor != primary and actor.has_method("sync_sensor_pose"):
			actor.sync_sensor_pose()
	var origin: Vector3 = primary.position + Basis(Vector3.UP, primary.rotation.y) * offset
	var base_yaw: float = primary.rotation.y + yaw_offset
	var space := primary.get_world_3d().direct_space_state
	var exclude: Array[RID] = []
	if primary.has_method("sensor_rid"):
		var own_rid: RID = primary.sensor_rid()
		if own_rid.is_valid():
			exclude.append(own_rid)
	_query.exclude = exclude
	_query.collision_mask = LIDAR_LAYER
	var ranges: Array[float] = []
	ranges.resize(RAY_COUNT)
	for index in range(RAY_COUNT):
		var angle := ANGLE_MIN + float(index) * ANGLE_INCREMENT
		var world_yaw := base_yaw + angle
		var direction := Vector3(-sin(world_yaw), 0.0, -cos(world_yaw))
		_query.from = origin
		_query.to = origin + direction * range_max
		var hit := space.intersect_ray(_query)
		if hit.is_empty():
			ranges[index] = range_max
		else:
			ranges[index] = clampf(origin.distance_to(hit["position"]), range_min, range_max)
	var header := {"frame_id": frame_id}
	if not stamp.is_empty():
		header["stamp"] = stamp
	last_scan = {
		"header": header,
		"angle_min": ANGLE_MIN,
		"angle_max": ANGLE_MAX,
		"angle_increment": ANGLE_INCREMENT,
		"time_increment": 0.0,
		"scan_time": SCAN_INTERVAL,
		"range_min": range_min,
		"range_max": range_max,
		"ranges": ranges,
		"intensities": [],
	}
	return last_scan


static func ros_to_godot(ros_xyz: Vector3) -> Vector3:
	return Vector3(-ros_xyz.y, ros_xyz.z, -ros_xyz.x)


static func godot_to_ros(godot_xyz: Vector3) -> Vector3:
	return Vector3(-godot_xyz.z, -godot_xyz.x, godot_xyz.y)


static func ros_time_now() -> Dictionary:
	var now := Time.get_unix_time_from_system()
	var sec := int(floor(now))
	var nanosec := int(round((now - float(sec)) * 1000000000.0))
	if nanosec >= 1000000000:
		sec += 1
		nanosec -= 1000000000
	elif nanosec < 0:
		nanosec = 0
	return {"sec": sec, "nanosec": nanosec}


static func load_urdf_mount(urdf_path := "", metadata_path := METADATA_PATH, target_link := DEFAULT_FRAME_ID) -> Dictionary:
	var candidate := urdf_path
	if candidate.is_empty():
		candidate = ProjectSettings.globalize_path("res://").path_join(URDF_RELATIVE_PATH).simplify_path()
	elif candidate.begins_with("res://") or candidate.begins_with("user://"):
		candidate = ProjectSettings.globalize_path(candidate)
	if FileAccess.file_exists(candidate):
		var parsed := _parse_urdf_file(candidate, target_link)
		if not parsed.is_empty():
			return parsed
	if not metadata_path.is_empty() and FileAccess.file_exists(metadata_path):
		var raw: Variant = JSON.parse_string(FileAccess.get_file_as_string(metadata_path))
		if raw is Dictionary and raw.get("urdf_lidar") is Dictionary:
			var info: Dictionary = raw["urdf_lidar"]
			var xyz_arr: Variant = info.get("xyz", [0.0, 0.0, 0.2492])
			var rpy_arr: Variant = info.get("rpy", [0.0, 0.0, 0.0])
			if xyz_arr is Array and xyz_arr.size() == 3 and rpy_arr is Array and rpy_arr.size() == 3:
				var xyz := Vector3(float(xyz_arr[0]), float(xyz_arr[1]), float(xyz_arr[2]))
				var rpy := Vector3(float(rpy_arr[0]), float(rpy_arr[1]), float(rpy_arr[2]))
				return {
					"joint": str(info.get("joint", DEFAULT_JOINT)),
					"parent": str(info.get("parent", "base_link")),
					"frame_id": str(info.get("frame_id", target_link)),
					"xyz": xyz,
					"rpy": rpy,
					"offset": ros_to_godot(xyz),
					"yaw": rpy.z,
					"source": "metadata",
				}
	var fallback_xyz := Vector3(0.0, 0.0, 0.2492)
	return {
		"joint": DEFAULT_JOINT,
		"parent": "base_link",
		"frame_id": target_link,
		"xyz": fallback_xyz,
		"rpy": Vector3.ZERO,
		"offset": ros_to_godot(fallback_xyz),
		"yaw": 0.0,
		"source": "default",
	}


static func _parse_urdf_file(path: String, target_link: String) -> Dictionary:
	var parser := XMLParser.new()
	if parser.open(path) != OK:
		return {}
	var in_joint := false
	var joint := ""
	var parent := "base_link"
	var child := ""
	var xyz := Vector3.ZERO
	var rpy := Vector3.ZERO
	while parser.read() == OK:
		var node_type := parser.get_node_type()
		if node_type == XMLParser.NODE_ELEMENT:
			var element := parser.get_node_name()
			if element == "joint":
				in_joint = true
				joint = parser.get_named_attribute_value_safe("name")
				parent = "base_link"
				child = ""
				xyz = Vector3.ZERO
				rpy = Vector3.ZERO
			elif in_joint and element == "parent":
				parent = parser.get_named_attribute_value_safe("link")
			elif in_joint and element == "child":
				child = parser.get_named_attribute_value_safe("link")
			elif in_joint and element == "origin":
				xyz = _parse_vec3(parser.get_named_attribute_value_safe("xyz"), Vector3.ZERO)
				rpy = _parse_vec3(parser.get_named_attribute_value_safe("rpy"), Vector3.ZERO)
		elif node_type == XMLParser.NODE_ELEMENT_END and parser.get_node_name() == "joint":
			if in_joint and (child == target_link or joint == DEFAULT_JOINT):
				return {
					"joint": joint,
					"parent": parent,
					"frame_id": child if not child.is_empty() else target_link,
					"xyz": xyz,
					"rpy": rpy,
					"offset": ros_to_godot(xyz),
					"yaw": rpy.z,
					"source": "urdf",
				}
			in_joint = false
	return {}


static func _parse_vec3(text: String, fallback: Vector3) -> Vector3:
	var parts := text.strip_edges().split(" ", false)
	if parts.size() != 3:
		return fallback
	for part in parts:
		if not part.is_valid_float():
			return fallback
	return Vector3(float(parts[0]), float(parts[1]), float(parts[2]))

