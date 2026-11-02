extends RefCounted
## Host-only ground-truth Odometry publisher for the primary robot.

const LidarScript = preload("res://scripts/ros/lidar.gd")
const ODOM_INTERVAL := 0.05
const DEFAULT_FRAME_ID := "odom"
const DEFAULT_CHILD_FRAME_ID := "base_link"

var frame_id := DEFAULT_FRAME_ID
var child_frame_id := DEFAULT_CHILD_FRAME_ID
var last_odom: Dictionary = {}
var _covariance: Array[float] = []


func _init() -> void:
	_covariance.resize(36)
	_covariance.fill(0.0)


func sample(primary: Node3D, stamp: Dictionary = {}) -> Dictionary:
	if not is_instance_valid(primary):
		return {}
	var ros_pos := LidarScript.godot_to_ros(primary.position)
	var yaw := primary.rotation.y
	var half_yaw := yaw * 0.5
	var forward_vel := 0.0
	var lateral_vel := 0.0
	var vertical_vel := 0.0
	if primary is CharacterBody3D:
		var world_vel: Vector3 = primary.velocity
		forward_vel = world_vel.dot(-primary.global_basis.z)
		lateral_vel = world_vel.dot(-primary.global_basis.x)
		vertical_vel = world_vel.y
	var angular_z := 0.0
	var raw_ang: Variant = primary.get("angular_velocity")
	if raw_ang is float or raw_ang is int:
		angular_z = float(raw_ang)
	var header := {"frame_id": frame_id}
	if not stamp.is_empty():
		header["stamp"] = stamp
	last_odom = {
		"header": header,
		"child_frame_id": child_frame_id,
		"pose": {
			"pose": {
				"position": {
					"x": ros_pos.x,
					"y": ros_pos.y,
					"z": ros_pos.z,
				},
				"orientation": {
					"x": 0.0,
					"y": 0.0,
					"z": sin(half_yaw),
					"w": cos(half_yaw),
				},
			},
			"covariance": _covariance.duplicate(),
		},
		"twist": {
			"twist": {
				"linear": {
					"x": forward_vel,
					"y": lateral_vel,
					"z": vertical_vel,
				},
				"angular": {
					"x": 0.0,
					"y": 0.0,
					"z": angular_z,
				},
			},
			"covariance": _covariance.duplicate(),
		},
	}
	return last_odom
