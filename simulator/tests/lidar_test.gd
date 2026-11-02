extends SceneTree

const LidarScript = preload("res://scripts/ros/lidar.gd")
const ActorScript = preload("res://scripts/actor.gd")
const WorldScript = preload("res://scripts/world.gd")

var app: Node3D
var failures := PackedStringArray()
var log_lines := PackedStringArray()
var received_scans: Array[Dictionary] = []
var received_odoms: Array[Dictionary] = []


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	_test_urdf_mount()
	app = load("res://scenes/main.tscn").instantiate()
	root.add_child(app)
	if not app.options.has("integration"):
		app.hud._ros_url.text = "ws://127.0.0.1:1"
	app._host("127.0.0.1", 27352, "LidarTest")
	if not app.session.ready_to_play:
		push_error("LiDAR test could not start its local host")
		quit(1)
		return
	await physics_frame
	await physics_frame
	_test_odometry()
	await _test_scan_geometry()
	await _test_combined_floor_wall_mesh()
	await _test_actors_and_remote_robots()
	if app.options.has("integration"):
		await _test_ros_integration()
	app.session.leave()
	var summary := "LIDAR TESTS PASSED" if failures.is_empty() else "LIDAR TESTS FAILED"
	log_lines.append(summary)
	print(summary)
	var report_file := FileAccess.open("res://.godot/lidar_test_report.txt", FileAccess.WRITE)
	if report_file:
		report_file.store_string("\n".join(log_lines) + "\n")
		report_file.close()
	quit(0 if failures.is_empty() else 1)


func _test_urdf_mount() -> void:
	var lidar := LidarScript.new()
	_check(lidar.source == "urdf" and lidar.frame_id == "velodyne"
		and lidar.joint_name == "base_to_velodyne" and lidar.parent_frame == "base_link"
		and lidar.offset.is_equal_approx(Vector3(0.0, 0.2492, 0.0)),
		"loads velodyne frame_id and 0.2492 m Z offset directly from cub4.urdf")
	var metadata_mount := LidarScript.load_urdf_mount("res://.godot/nonexistent.urdf")
	_check(metadata_mount["source"] == "metadata" and metadata_mount["frame_id"] == "velodyne"
		and metadata_mount["offset"].is_equal_approx(Vector3(0.0, 0.2492, 0.0)),
		"falls back to cub4.json metadata when the ROS URDF file is absent")
	var custom_path := ProjectSettings.globalize_path("res://.godot/test_lidar.urdf")
	var file := FileAccess.open(custom_path, FileAccess.WRITE)
	file.store_string("""<?xml version="1.0"?>
<robot name="test">
  <link name="base_link"/>
  <link name="velodyne"/>
  <joint name="base_to_velodyne" type="fixed">
    <parent link="base_link"/>
    <child link="velodyne"/>
    <origin xyz="0.12 -0.05 0.31" rpy="0 0 0.25"/>
  </joint>
</robot>
""")
	file.close()
	var custom := LidarScript.load_urdf_mount(custom_path)
	DirAccess.remove_absolute(custom_path)
	_check(custom["source"] == "urdf" and custom["offset"].is_equal_approx(Vector3(0.05, 0.31, -0.12))
		and is_equal_approx(float(custom["yaw"]), 0.25),
		"converts URDF ROS (x, y, z, yaw) into Godot (-y, z, -x, yaw)")


func _test_odometry() -> void:
	var primary: Node3D = app.actors[1]
	primary.position = Vector3(-2.5, 0.4, 3.0)
	primary.rotation.y = PI * 0.5
	primary.velocity = Vector3(-1.2, 0.0, 0.0)
	primary.angular_velocity = 0.35
	var odom_msg: Dictionary = app.odom.sample(primary)
	var pos: Dictionary = odom_msg["pose"]["pose"]["position"]
	var ori: Dictionary = odom_msg["pose"]["pose"]["orientation"]
	var lin: Dictionary = odom_msg["twist"]["twist"]["linear"]
	var ang: Dictionary = odom_msg["twist"]["twist"]["angular"]
	_check(odom_msg["header"]["frame_id"] == "odom" and odom_msg["child_frame_id"] == "base_link"
		and is_equal_approx(float(pos["x"]), -3.0)
		and is_equal_approx(float(pos["y"]), 2.5)
		and is_equal_approx(float(pos["z"]), 0.4)
		and is_equal_approx(float(ori["z"]), sin(PI * 0.25))
		and is_equal_approx(float(ori["w"]), cos(PI * 0.25))
		and is_equal_approx(float(lin["x"]), 1.2)
		and is_equal_approx(float(ang["z"]), 0.35),
		"Odometry converts Godot pose and local velocity into ROS odom -> base_link")
	primary.velocity = Vector3.ZERO
	primary.angular_velocity = 0.0
	primary.position = Vector3.ZERO
	primary.rotation.y = 0.0


func _test_scan_geometry() -> void:
	var primary: Node3D = app.actors[1]
	primary.position = Vector3(0.0, 0.0, 0.0)
	primary.rotation.y = 0.0
	await physics_frame
	var scan: Dictionary = app.lidar.sample(primary, app.actors)
	var ranges: Array = scan["ranges"]
	_check(scan["header"]["frame_id"] == "velodyne" and ranges.size() == 360
		and is_equal_approx(float(scan["angle_min"]), -PI)
		and is_equal_approx(float(scan["angle_increment"]), TAU / 360.0)
		and is_equal_approx(float(scan["angle_max"]), PI - TAU / 360.0),
		"LaserScan uses velodyne frame_id and 360 beams across [-pi, pi)")
	var serialized := JSON.stringify(scan)
	_check(not serialized.is_empty() and "null" not in serialized and "inf" not in serialized.to_lower(),
		"LaserScan serializes cleanly to JSON with finite range values")
	var min_range := 999.0
	for value: float in ranges:
		min_range = minf(min_range, value)
	_check(min_range > 2.0, "primary robot excludes its own movement and sensor colliders")
	# At origin facing -Z (ROS +X): outer walls are at x = +-25, z = +-25 (inner face at 24.85 m).
	# Forward (0 rad, index 180) faces z = -25 -> ramp/rough mesh/steps are in between,
	# and must NOT block the horizontal beam at y = 0.2492 m!
	_check(absf(float(ranges[_ray_index(0.0)]) - 24.85) < 0.15
		and absf(float(ranges[_ray_index(PI * 0.5)]) - 24.85) < 0.15
		and absf(float(ranges[_ray_index(-PI * 0.5)]) - 24.85) < 0.15
		and absf(float(ranges[_ray_index(-PI)]) - 24.85) < 0.15,
		"outer walls are detected in all cardinal directions while floor/ramp/rough mesh are ignored")
	# Test stairs exclusion directly in front of the 15 cm staircase at x = 6, z = 1.
	primary.position = Vector3(6.0, 0.0, 1.0)
	primary.rotation.y = 0.0
	await physics_frame
	scan = app.lidar.sample(primary, app.actors)
	_check(float(scan["ranges"][_ray_index(0.0)]) > 20.0,
		"walkable 15 cm staircase and its platform are excluded from LiDAR obstacle hits")
	# Place a temporary wall box 4.0 m ahead (center at z = -4.5, thickness 1.0 -> near face at 4.0 m).
	primary.position = Vector3(0.0, 0.0, 0.0)
	primary.rotation.y = 0.0
	var obstacle: StaticBody3D = app.world.box(Vector3(1.0, 1.2, 1.0), Vector3(0.0, 0.6, -4.5), Color.RED)
	await physics_frame
	scan = app.lidar.sample(primary, app.actors)
	_check(absf(float(scan["ranges"][_ray_index(0.0)]) - 4.0) < 0.08,
		"forward obstacle at Godot -Z is reported at ROS angle 0 rad")
	# Rotate robot +90 deg (left, facing Godot -X); the obstacle at Godot -Z is now on the robot's right (-PI/2).
	primary.rotation.y = PI * 0.5
	scan = app.lidar.sample(primary, app.actors)
	_check(absf(float(scan["ranges"][_ray_index(-PI * 0.5)]) - 4.0) < 0.08,
		"rotating robot left shifts the obstacle to ROS angle -pi/2 (right side)")
	primary.rotation.y = 0.0
	app.world.remove_child(obstacle)
	obstacle.queue_free()
	await physics_frame


func _test_combined_floor_wall_mesh() -> void:
	var primary: Node3D = app.actors[1]
	primary.position = Vector3(0.0, 0.0, 15.0)
	primary.rotation.y = 0.0
	# Build a single mesh containing a flat floor, a 20-deg slope, and a vertical wall at z = 11.5 (3.5 m ahead).
	var combined := PackedVector3Array([
		# Flat floor (y = 0.25, right at beam height, between robot and wall: z = 14 to 12)
		Vector3(-1.5, 0.25, 14.0), Vector3(1.5, 0.25, 14.0), Vector3(1.5, 0.25, 12.0),
		Vector3(-1.5, 0.25, 14.0), Vector3(1.5, 0.25, 12.0), Vector3(-1.5, 0.25, 12.0),
		# Gentle slope (from y = 0.1 to y = 0.4 across z = 14 to 12, normal ~8.5 deg)
		Vector3(-1.5, 0.1, 14.0), Vector3(1.5, 0.1, 14.0), Vector3(1.5, 0.4, 12.0),
		Vector3(-1.5, 0.1, 14.0), Vector3(1.5, 0.4, 12.0), Vector3(-1.5, 0.4, 12.0),
		# Vertical wall at z = 11.5 (3.5 m ahead of z = 15.0)
		Vector3(-1.5, 0.0, 11.5), Vector3(1.5, 0.0, 11.5), Vector3(1.5, 1.2, 11.5),
		Vector3(-1.5, 0.0, 11.5), Vector3(1.5, 1.2, 11.5), Vector3(-1.5, 1.2, 11.5),
	])
	var mesh_body: StaticBody3D = app.world.terrain_mesh(combined, Color.TEAL)
	await physics_frame
	var scan: Dictionary = app.lidar.sample(primary, app.actors)
	var forward_dist := float(scan["ranges"][_ray_index(0.0)])
	_check(absf(forward_dist - 3.5) < 0.08,
		"combined floor+slope+wall mesh ignores floor/slope faces at beam height and detects only the wall (%.2f m)" % forward_dist)
	app.world.remove_child(mesh_body)
	mesh_body.queue_free()
	await physics_frame


func _test_actors_and_remote_robots() -> void:
	var primary: Node3D = app.actors[1]
	primary.position = Vector3(0.0, 0.0, 4.0)
	primary.rotation.y = 0.0
	# Spawn another robot (ahead, 3.0 m center -> 3.0 - 0.28 = 2.72 m surface),
	# a pedestrian (left, 4.0 m center -> 4.0 - 0.23 = 3.77 m surface),
	# and a cone (right, 2.5 m center -> 2.5 - 0.24 = 2.26 m surface).
	app.session._add_record(app.session._record(201, 201, "robot", "OtherRobot", Vector3(0.0, 0.0, 1.0), false, 1))
	app.session._add_record(app.session._record(202, 202, "pedestrian", "Walker", Vector3(-4.0, 0.0, 4.0), false, 2))
	app.session._add_record(app.session._record(203, 203, "cone", "Cone", Vector3(2.5, 0.0, 4.0), false, 3))
	await physics_frame
	var scan: Dictionary = app.lidar.sample(primary, app.actors)
	var ranges: Array = scan["ranges"]
	_check(absf(float(ranges[_ray_index(0.0)]) - (3.0 - ActorScript.SENSOR_RADII["robot"])) < 0.08,
		"detects another robot ahead at its sensor cylinder surface")
	_check(absf(float(ranges[_ray_index(PI * 0.5)]) - (4.0 - ActorScript.SENSOR_RADII["pedestrian"])) < 0.08,
		"detects pedestrian on the left")
	_check(absf(float(ranges[_ray_index(-PI * 0.5)]) - (2.5 - ActorScript.SENSOR_RADII["cone"])) < 0.08,
		"detects low cone on the right via extended sensor cylinder")
	# Simulate a network pose update for the other robot:
	# Its display position (actor.position) remains delayed at z = 1.0, while receive_pose moves target_position to z = -1.0 (5.0 m ahead).
	var other_robot: Node3D = app.actors[201]
	other_robot.receive_pose(Vector3(0.0, 0.0, -1.0), 0.0, 1.0)
	_check(other_robot.position.distance_to(Vector3(0.0, 0.0, 1.0)) < 0.05,
		"remote robot display position remains buffered before interpolation")
	scan = app.lidar.sample(primary, app.actors)
	_check(absf(float(scan["ranges"][_ray_index(0.0)]) - (5.0 - ActorScript.SENSOR_RADII["robot"])) < 0.08,
		"LiDAR uses the latest received network pose of the other robot rather than its delayed display position")
	# Test elevated detection on the ramp top platform (y = 1.2 m).
	primary.position = Vector3(-8.0, 1.2, -8.5)
	primary.rotation.y = 0.0
	other_robot.receive_pose(Vector3(-8.0, 1.2, -10.5), 0.0, 2.0)
	scan = app.lidar.sample(primary, app.actors)
	_check(absf(float(scan["ranges"][_ray_index(0.0)]) - (2.0 - ActorScript.SENSOR_RADII["robot"])) < 0.08,
		"LiDAR detects another robot when both are on an elevated surface (y = 1.2 m)")
	# Remove actors and verify clearing back to open range.
	for id in [201, 202, 203]:
		app.session._remove_record(id)
	primary.position = Vector3(0.0, 0.0, 0.0)
	primary.rotation.y = 0.0
	await physics_frame
	scan = app.lidar.sample(primary, app.actors)
	_check(float(scan["ranges"][_ray_index(0.0)]) > 20.0
		and float(scan["ranges"][_ray_index(PI * 0.5)]) > 20.0
		and float(scan["ranges"][_ray_index(-PI * 0.5)]) > 20.0,
		"removing other actors clears the rays back to the background range")


func _test_ros_integration() -> void:
	var primary: Node3D = app.actors[1]
	primary.position = Vector3(0.0, 0.0, 4.0)
	primary.rotation.y = 0.0
	app.session._add_record(app.session._record(301, 301, "robot", "RosOtherRobot", Vector3(0.0, 0.0, 2.0), false, 1))
	var subscriber := WebSocketPeer.new()
	subscriber.max_queued_packets = 64
	subscriber.connect_to_url(app.ros_bridge.url)
	app.ros_bridge.start()
	var subscribed := false
	var deadline := Time.get_ticks_msec() + 10000
	while (received_scans.size() < 3 or received_odoms.size() < 3) and Time.get_ticks_msec() < deadline:
		subscriber.poll()
		if subscriber.get_ready_state() == WebSocketPeer.STATE_OPEN:
			if not subscribed:
				subscribed = true
				subscriber.send_text(JSON.stringify({
					"op": "subscribe",
					"topic": "/scan",
					"type": "sensor_msgs/msg/LaserScan",
					"queue_length": 1,
				}))
				subscriber.send_text(JSON.stringify({
					"op": "subscribe",
					"topic": "/odom",
					"type": "nav_msgs/msg/Odometry",
					"queue_length": 1,
				}))
			while subscriber.get_available_packet_count() > 0:
				var packet: Variant = JSON.parse_string(subscriber.get_packet().get_string_from_utf8())
				if packet is Dictionary and packet.get("op") == "publish" and packet.get("msg") is Dictionary:
					if packet.get("topic") == "/scan":
						received_scans.append(packet["msg"])
					elif packet.get("topic") == "/odom":
						received_odoms.append(packet["msg"])
		await process_frame
	subscriber.close()
	_check(received_scans.size() >= 3, "ROS receives /scan messages from Godot via rosbridge")
	_check(received_odoms.size() >= 3, "ROS receives /odom messages from Godot via rosbridge")
	if not received_scans.is_empty():
		var latest: Dictionary = received_scans[-1]
		var ranges: Variant = latest.get("ranges", [])
		var expected := 2.0 - ActorScript.SENSOR_RADII["robot"]
		_check(latest.get("header", {}).get("frame_id", "") == "velodyne"
			and ranges is Array and ranges.size() == 360
			and absf(float(ranges[_ray_index(0.0)]) - expected) < 0.08,
			"ROS /scan contains velodyne frame_id, 360 ranges, and the other robot distance")
	if app.options.has("nav-test"):
		var obstacle_pos := Vector2(0.0, 2.0)
		var max_abs_x := 0.0
		var min_clearance := INF
		var nav_deadline := Time.get_ticks_msec() + 25000
		while primary.position.z > 0.5 and Time.get_ticks_msec() < nav_deadline:
			var robot_xz := Vector2(primary.position.x, primary.position.z)
			max_abs_x = maxf(max_abs_x, absf(primary.position.x))
			min_clearance = minf(min_clearance, robot_xz.distance_to(obstacle_pos))
			await process_frame
		log_lines.append("NAV_METRICS: final_godot=(%.2f, %.2f) max_abs_x=%.2f min_clearance=%.2f" % [
			primary.position.x, primary.position.z, max_abs_x, min_clearance
		])
		_check(primary.position.z <= 0.5 and max_abs_x > 0.35 and min_clearance > 0.55,
			"Nav2 detours around LiDAR-detected other robot and reaches past the obstacle")
	app.session._remove_record(301)


func _ray_index(angle: float) -> int:
	return int(round((angle - LidarScript.ANGLE_MIN) / LidarScript.ANGLE_INCREMENT)) % LidarScript.RAY_COUNT


func _check(condition: bool, description: String) -> void:
	var line := "%s: %s" % ["PASS" if condition else "FAIL", description]
	log_lines.append(line)
	print(line)
	if not condition:
		failures.append(description)

