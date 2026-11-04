extends SceneTree
## Automated verification for 2D occupancy extrusion worlds, spawn validation, camera, LiDAR, and online agreement.

const MainScript = preload("res://scripts/main.gd")
const WorldScript = preload("res://scripts/world.gd")
const WorldRegistryScript = preload("res://scripts/world_registry.gd")
const OccupancyLoaderScript = preload("res://scripts/occupancy_loader.gd")
const SessionScript = preload("res://scripts/session.gd")
const LidarScript = preload("res://scripts/ros/lidar.gd")

var app: Node3D
var failures := PackedStringArray()


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	_test_cli_argument_validation()
	await _test_synthetic_rotated_gray_world()
	await _test_gnss2026_world_and_gameplay()
	await _test_online_handshake_and_spawn_enforcement()
	if failures.is_empty():
		print("OCCUPANCY WORLD TESTS PASSED")
		quit(0)
	else:
		for failure in failures:
			push_error(failure)
		print("OCCUPANCY WORLD TESTS FAILED (%d failures)" % failures.size())
		quit(1)


func _test_cli_argument_validation() -> void:
	_check(MainScript.validate_options({"host": true, "world": "gnss2026_mid360_1017_02", "spawn": "start_a"}).is_empty(),
		"accepts --host --world <id> --spawn <preset>")
	_check(MainScript.validate_options({"host": true, "world": "gnss2026_mid360_1017_02", "spawn-x": "62.2", "spawn-y": "30.1", "spawn-yaw-deg": "180"}).is_empty(),
		"accepts --host --spawn-x --spawn-y --spawn-yaw-deg")
	_check(MainScript.validate_options({"join": "127.0.0.1", "spawn-x": "64.0", "spawn-y": "30.1", "spawn-z": "0.2"}).is_empty(),
		"accepts --join with --spawn-x --spawn-y --spawn-z")
	_check(not MainScript.validate_options({"join": "127.0.0.1", "world": "gnss2026_mid360_1017_02"}).is_empty(),
		"rejects --world when combined with --join")
	_check(not MainScript.validate_options({"host": true, "spawn": "start_a", "spawn-x": "0", "spawn-y": "0"}).is_empty(),
		"rejects combining --spawn preset with --spawn-x/--spawn-y")
	_check(not MainScript.validate_options({"host": true, "spawn-x": "10.0"}).is_empty(),
		"rejects --spawn-x without --spawn-y")
	_check(not MainScript.validate_options({"host": true, "spawn-y": "10.0"}).is_empty(),
		"rejects --spawn-y without --spawn-x")
	_check(not MainScript.validate_options({"host": true, "spawn-z": "0.1"}).is_empty(),
		"rejects --spawn-z without --spawn-x and --spawn-y")
	_check(not MainScript.validate_options({"host": true, "spawn-yaw-deg": "90"}).is_empty(),
		"rejects --spawn-yaw-deg without --spawn-x and --spawn-y")
	_check(not MainScript.validate_options({"host": true, "spawn-x": "nan", "spawn-y": "0"}).is_empty(),
		"rejects non-finite --spawn-x")


func _test_synthetic_rotated_gray_world() -> void:
	var temp_root := "res://.godot/test_worlds_synth"
	var world_dir := temp_root + "/rot_gray"
	DirAccess.make_dir_recursive_absolute(ProjectSettings.globalize_path(world_dir))

	# 10x8 grid, resolution 0.2 m/cell, rotated origin (10.0, -5.0, +90 deg).
	var w := 10
	var h := 8
	var r := 0.2
	var img := Image.create(w, h, false, Image.FORMAT_L8)
	# Fill with gray 205 (must be treated as free floor).
	img.fill(Color(205.0 / 255.0, 205.0 / 255.0, 205.0 / 255.0, 1.0))
	# Add another shade of gray (140) on the left side.
	img.set_pixel(2, 4, Color(140.0 / 255.0, 140.0 / 255.0, 140.0 / 255.0, 1.0))
	# Create a 3x3 ring of black (0) around (7, 4), leaving only 1 white cell (254) at (7, 4).
	for dv in [-1, 0, 1]:
		for du in [-1, 0, 1]:
			var val := 254.0 / 255.0 if (du == 0 and dv == 0) else 0.0
			img.set_pixel(7 + du, 4 + dv, Color(val, val, val, 1.0))
	var png_abs := ProjectSettings.globalize_path(world_dir + "/source.png")
	img.save_png(png_abs)

	var chunks_dict := {
		"schema_version": 1,
		"chunk_cells": 64,
		"image_width": w,
		"image_height": h,
		"chunks": [
			{
				"cx": 0,
				"cy": 0,
				"rects": [
					[6, 3, 3, 1],
					[6, 4, 1, 1],
					[8, 4, 1, 1],
					[6, 5, 3, 1],
				],
			}
		],
	}
	var chunks_file := FileAccess.open(world_dir + "/wall_chunks.json", FileAccess.WRITE)
	chunks_file.store_string(JSON.stringify(chunks_dict))
	chunks_file.close()

	var origin := [10.0, -5.0, PI * 0.5]
	# Cell (2, 4) center in local: lx = 2.5 * 0.2 = 0.5, ly = (8 - 4 - 0.5) * 0.2 = 0.7
	# With yaw = +90 deg: ros_x = 10.0 - 0.7 = 9.3, ros_y = -5.0 + 0.5 = -4.5
	var world_def := {
		"schema_version": 1,
		"kind": "occupancy_extrusion",
		"world_id": "rot_gray",
		"display_name": "Rotated Gray Test",
		"world_hash": "synth_rot_gray_hash_01",
		"bounds": {
			"ros_min": [8.4, -5.0, 0.0],
			"ros_max": [10.0, -3.0, 0.5],
			"godot_min": [3.0, 0.0, -10.0],
			"godot_max": [5.0, 0.5, -8.4],
			"size_m": [2.0, 1.6, 0.5],
		},
		"default_spawn": "start_a",
		"spawns": {
			"start_a": {
				"display_name": "Gray Start",
				"robot": {"x": 9.3, "y": -4.5, "z": 0.0, "yaw_deg": 90.0},
				"clients": [{"x": 9.3, "y": -4.5, "z": 0.0, "yaw_deg": 90.0}],
			}
		},
		"source": {
			"image": "source.png",
			"width": w,
			"height": h,
			"resolution": r,
			"origin": origin,
			"occupied_values": [0],
			"free_gray_policy": "free",
		},
		"extrusion": {
			"generator_version": 1,
			"floor_height": 0.0,
			"floor_thickness": 0.3,
			"wall_height": 0.5,
			"chunk_cells": 64,
			"chunks_file": "wall_chunks.json",
			"capsule_clearance_radius_m": 0.28,
		},
	}
	var def_file := FileAccess.open(world_dir + "/world.json", FileAccess.WRITE)
	def_file.store_string(JSON.stringify(world_def))
	def_file.close()

	var test_world := WorldScript.new()
	root.add_child(test_world)
	var loaded := test_world.load_world("rot_gray", temp_root)
	_check(loaded and test_world.is_loaded, "loads synthetic rotated occupancy world")
	var loader: RefCounted = test_world.occupancy_loader
	var c00: Vector2 = loader.cell_center_to_ros(0, 0)
	_check(is_equal_approx(c00.x, 8.5) and is_equal_approx(c00.y, -4.9),
		"rotated origin and vertical flip convert top-left cell (0,0) center to ROS (8.5, -4.9)")
	_check(not loader.is_cell_occupied(0, 0) and not loader.is_cell_occupied(2, 4) and loader.is_cell_occupied(6, 3),
		"pixel value 0 is occupied while gray values 205 and 140 are free")

	var gray_spawn: Dictionary = test_world.resolve_spawn({"spawn": "start_a"}, true)
	_check(bool(gray_spawn.get("ok", false)), "gray region with sufficient clearance is accepted as spawn")

	var hole_ros: Vector2 = loader.cell_center_to_ros(7, 4)
	var hole_spawn: Dictionary = test_world.resolve_spawn({"x": hole_ros.x, "y": hole_ros.y}, true)
	_check(not bool(hole_spawn.get("ok", true)),
		"single free cell (0.2 m) surrounded by black walls is rejected for capsule clearance")

	# An optional image changes only the floor's appearance, with no rotation/mirroring or filtering.
	var floor_image := Image.create(w, h, false, Image.FORMAT_RGB8)
	floor_image.fill(Color.BLUE)
	floor_image.set_pixel(0, 0, Color.RED)
	floor_image.set_pixel(w - 1, 0, Color.GREEN)
	floor_image.set_pixel(0, h - 1, Color.YELLOW)
	floor_image.save_png(ProjectSettings.globalize_path(world_dir + "/floor_texture.png"))
	world_def["appearance"] = {
		"floor_texture": "floor_texture.png",
		"floor_texture_sha256": FileAccess.get_sha256(world_dir + "/floor_texture.png"),
	}
	var textured_def := WorldRegistryScript.validate_definition(world_def, world_dir)
	_check(test_world.load_from_definition(textured_def), "loads optional PNG floor texture")
	var surface: MeshInstance3D = test_world.get_node("OccupancyWorld/OccupancyFloor/FloorSurface")
	var mat: BaseMaterial3D = surface.material_override
	_check(mat.albedo_texture != null and mat.texture_filter == BaseMaterial3D.TEXTURE_FILTER_NEAREST
		and not mat.albedo_texture.get_image().has_mipmaps() and not mat.texture_repeat,
		"floor uses nearest-neighbor filtering without mipmaps or texture repetition")
	_check(mat.albedo_texture.get_image().get_pixel(0, 0).is_equal_approx(Color.RED)
		and mat.albedo_texture.get_image().get_pixel(w - 1, 0).is_equal_approx(Color.GREEN)
		and mat.albedo_texture.get_image().get_pixel(0, h - 1).is_equal_approx(Color.YELLOW),
		"floor image retains distinct corner pixels")
	var arrays := surface.mesh.surface_get_arrays(0)
	var vertices: PackedVector3Array = arrays[Mesh.ARRAY_VERTEX]
	var uvs: PackedVector2Array = arrays[Mesh.ARRAY_TEX_UV]
	for i in range(vertices.size()):
		var ros: Vector2 = test_world.occupancy_loader.local_to_ros(uvs[i].x * w * r, (1.0 - uvs[i].y) * h * r)
		var expected := LidarScript.ros_to_godot(Vector3(ros.x, ros.y, 0.0))
		_check((surface.global_transform * vertices[i]).distance_to(expected) < 0.0001,
			"floor UV corner %d matches map coordinates with rotated origin" % i)
	if DisplayServer.get_name() != "headless":
		await _test_floor_rendering(surface, test_world.occupancy_loader)
	var bad_def := world_def.duplicate(true)
	bad_def["appearance"]["floor_texture"] = "missing.png"
	_check(not WorldRegistryScript.validate_definition(bad_def, world_dir).get("ok", true),
		"rejects missing floor texture before world start")
	bad_def["appearance"]["floor_texture"] = "floor_texture.png"
	bad_def["appearance"]["floor_texture_sha256"] = "incorrect"
	_check(not WorldRegistryScript.validate_definition(bad_def, world_dir).get("ok", true),
		"rejects mismatched floor texture hash")
	floor_image.resize(w - 1, h, Image.INTERPOLATE_NEAREST)
	floor_image.save_png(ProjectSettings.globalize_path(world_dir + "/floor_texture.png"))
	textured_def = WorldRegistryScript.validate_definition(world_def, world_dir, false)
	_check(not test_world.load_from_definition(textured_def) and test_world.load_error.contains("dimensions"),
		"rejects floor texture with a different canvas size")

	root.remove_child(test_world)
	test_world.free()


func _test_floor_rendering(surface: MeshInstance3D, loader: RefCounted) -> void:
	# Render a 4x4 color grid through the production mesh/material. Each texel must
	# cover exactly 32x32 pixels: this catches invisible back faces and filtered edges.
	var viewport := SubViewport.new()
	viewport.size = Vector2i(128, 128)
	viewport.own_world_3d = true
	viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	root.add_child(viewport)
	var texture_image := Image.create(4, 4, false, Image.FORMAT_RGB8)
	var colors := [Color.RED, Color.GREEN, Color.BLUE, Color.YELLOW]
	for v in range(4):
		for u in range(4):
			texture_image.set_pixel(u, v, colors[(u + v) % 4])
	var material: StandardMaterial3D = surface.material_override.duplicate()
	material.albedo_texture = ImageTexture.create_from_image(texture_image)
	var quad := MeshInstance3D.new()
	quad.mesh = loader._floor_surface_mesh(Vector3(4, 0.3, 4))
	quad.position.y = -0.15
	quad.material_override = material
	viewport.add_child(quad)
	var cam := Camera3D.new()
	viewport.add_child(cam)
	cam.position = Vector3(0, 10, 0)
	cam.look_at(Vector3.ZERO, Vector3.LEFT)
	cam.projection = Camera3D.PROJECTION_ORTHOGONAL
	cam.size = 4
	cam.current = true
	for frame in range(3):
		await process_frame
	RenderingServer.force_draw()
	var capture := viewport.get_texture().get_image()
	var mismatches := 0
	for y in range(128):
		for x in range(128):
			var expected: Color = colors[(int(x / 32.0) + int(y / 32.0)) % 4]
			var actual := capture.get_pixel(x, y)
			if absf(actual.r - expected.r) > 0.02 or absf(actual.g - expected.g) > 0.02 or absf(actual.b - expected.b) > 0.02:
				mismatches += 1
	_check(mismatches == 0, "floor renders all 16384 pixels with correct orientation and no blended edges (%d mismatches)" % mismatches)
	viewport.free()


func _test_gnss2026_world_and_gameplay() -> void:
	app = load("res://scenes/main.tscn").instantiate()
	root.add_child(app)
	app.hud._ros_url.text = "ws://127.0.0.1:1"
	await physics_frame
	await process_frame

	_check(app.world.world_id == "cub_playground_v1" and app.world.is_loaded,
		"default startup world remains cub_playground_v1 when --world is omitted")

	var t0 := Time.get_ticks_usec()
	var switched: bool = app._select_world("gnss2026_mid360_1017_02")
	var load_ms := float(Time.get_ticks_usec() - t0) / 1000.0
	var mem_mb := Performance.get_monitor(Performance.MEMORY_STATIC) / 1048576.0
	print("GNSS2026 LOAD METRICS: load_ms=%.2f chunks=%d rects=%d tris=%d static_mem_mb=%.2f" % [
		load_ms,
		app.world.occupancy_loader.chunk_count if app.world.occupancy_loader else -1,
		app.world.occupancy_loader.rect_count if app.world.occupancy_loader else -1,
		app.world.occupancy_loader.triangle_count if app.world.occupancy_loader else -1,
		mem_mb,
	])
	_check(switched and app.world.is_loaded and app.world.world_id == "gnss2026_mid360_1017_02",
		"loads gnss2026_mid360_1017_02 occupancy world")
	_check(app.hud.selected_world_id() == "gnss2026_mid360_1017_02"
		and app.hud.selected_spawn_preset() == "start_a"
		and not app.hud._host_button.disabled,
		"Lobby UI reflects selected world and default preset and enables Host button")
	_check(not app._select_world("nonexistent_world_id") and app.hud._host_button.disabled,
		"selecting an invalid world disables the Host button and shows error info")
	app._select_world("gnss2026_mid360_1017_02")
	_check(app.world.occupancy_loader != null and app.world.occupancy_loader.chunk_count == 304,
		"builds all 304 non-empty 64x64 chunks for gnss2026_mid360_1017_02")
	_check(app.camera_rig.max_zoom_distance > 716.0 and app.camera_rig.camera.far > 1000.0,
		"camera zoom limit and far clip scale to cover the 716 m world")

	# Start host on gnss2026_mid360_1017_02 using default preset (pixel 780, 1469, facing left = 180 deg).
	app._host("127.0.0.1", 27361, "GnssHost")
	_check(app.session.ready_to_play and app.actors.has(1), "host starts session on gnss2026_mid360_1017_02")
	await physics_frame
	await physics_frame

	var primary: Node3D = app.actors[1]
	var expected_ros := Vector3(63.6725, 29.794, 0.1)
	var expected_godot := LidarScript.ros_to_godot(expected_ros)
	_check(Vector2(primary.position.x, primary.position.z).distance_to(Vector2(expected_godot.x, expected_godot.z)) < 0.01
		and is_equal_approx(wrapf(primary.rotation.y - PI, -PI, PI), 0.0),
		"primary robot spawns at pixel (780, 1469) = ROS (63.6725, 29.794) facing left (yaw 180 deg)")

	# Verify 2D camera orientation (KEY_3 / mode 2) matches the PGM image:
	# Image up (-v, ROS +Y) is Godot -X (Vector3.LEFT), Image right (+u, ROS +X) is Godot -Z (Vector3.FORWARD).
	app._select_camera(2)
	var cam: Camera3D = app.camera_rig.camera
	_check(cam.projection == Camera3D.PROJECTION_ORTHOGONAL
		and cam.global_basis.y.is_equal_approx(Vector3.LEFT)
		and cam.global_basis.x.is_equal_approx(Vector3.FORWARD),
		"2D map camera (KEY_3) aligns screen-up with PGM image-up and screen-right with PGM image-right")
	var robot_screen := cam.unproject_position(primary.global_position)
	var forward_screen := cam.unproject_position(primary.global_position + (-primary.global_basis.z) * 2.0)
	_check(forward_screen.x < robot_screen.x - 1.0 and absf(forward_screen.y - robot_screen.y) < 0.5,
		"robot at (780, 1469) with yaw 180 deg faces left on the 2D camera screen")

	# Verify Odometry reports ROS map coordinates directly (since map -> odom is identity).
	var odom_msg: Dictionary = app.odom.sample(primary)
	var odom_pos: Dictionary = odom_msg["pose"]["pose"]["position"]
	var odom_ori: Dictionary = odom_msg["pose"]["pose"]["orientation"]
	_check(absf(float(odom_pos["x"]) - 63.6725) < 0.02
		and absf(float(odom_pos["y"]) - 29.794) < 0.02
		and absf(absf(float(odom_ori["z"])) - 1.0) < 0.001
		and absf(float(odom_ori["w"])) < 0.001,
		"Odometry reports ROS pose (63.6725, 29.794, yaw=pi) at start position")

	# Verify LiDAR scan at start position (780, 1469) facing West (180 deg):
	# North wall (v=1437, south face at v=1438) is 31.5 * 0.2 = 6.30 m to the robot's right (-pi/2).
	# South (+pi/2) has no wall within the 30 m LiDAR range in map_sim.pgm.
	# Forward (0 rad, West along corridor) is open (> 15 m), proving flat floor is ignored.
	var scan_t0 := Time.get_ticks_usec()
	var scan: Dictionary = app.lidar.sample(primary, app.actors)
	var scan_ms := float(Time.get_ticks_usec() - scan_t0) / 1000.0
	print("GNSS2026 LIDAR METRICS: sample_ms=%.2f" % scan_ms)
	var ranges: Array = scan["ranges"]
	var right_dist := float(ranges[_ray_index(-PI * 0.5)])
	var left_dist := float(ranges[_ray_index(PI * 0.5)])
	var forward_dist := float(ranges[_ray_index(0.0)])
	_check(absf(right_dist - 6.30) < 0.12,
		"LiDAR detects North corridor wall on the right at ~6.30 m (actual %.2f m)" % right_dist)
	_check(absf(left_dist - LidarScript.RANGE_MAX) < 0.12,
		"LiDAR reports open space to the South at the 30 m range limit (actual %.2f m)" % left_dist)
	_check(forward_dist > 15.0,
		"LiDAR ignores flat floor and reports open corridor ahead (actual %.2f m)" % forward_dist)

	# Verify collision with 0.5 m wall (including high-speed 5 m/s motion toward the North wall at v=1437):
	# Face North (ROS +Y -> yaw = +90 deg -> Godot -X). North wall inner face is at Godot X = -29.794 - 6.30 = -36.094.
	primary.rotation.y = PI * 0.5
	for frame in range(120):
		await physics_frame
		primary.simulate_velocity(5.0, 0.0, 1.0 / 60.0)
	_check(primary.position.x > -36.094 + 0.20 and primary.position.y < 0.15,
		"primary robot collides with 0.5 m wall without climbing or tunneling at 5 m/s (pos=%s)" % str(primary.position))

	# Verify motion across the 64x64 chunk boundary at u = 768 (Godot Z = -61.1725).
	# map_sim.pgm leaves this former corridor-wall location open.
	primary.position = Vector3(-32.15, 0.05, -61.8)
	primary.rotation.y = PI - 0.12
	for frame in range(60):
		await physics_frame
		primary.simulate_velocity(3.0, 0.0, 1.0 / 60.0)
	_check(primary.position.z > -60.5,
		"robot crosses the open 64x64 chunk boundary without snagging (z=%.2f)" % primary.position.z)

	# Verify falling off or below the floor respawns at the world's validated start point, not (0, 0.1, 4):
	primary.position = Vector3(500.0, -15.0, 500.0)
	primary.simulate_velocity(0.0, 0.0, 1.0 / 60.0)
	_check(Vector2(primary.position.x, primary.position.z).distance_to(Vector2(expected_godot.x, expected_godot.z)) < 0.02
		and is_equal_approx(wrapf(primary.rotation.y - PI, -PI, PI), 0.0),
		"falling out of bounds respawns at the world's safe spawn point and yaw")

	# Verify out-of-bounds and inside-wall spawn rejection on gnss2026_mid360_1017_02:
	var oob_res: Dictionary = app.world.resolve_spawn({"x": 9999.0, "y": 9999.0}, true)
	_check(not bool(oob_res.get("ok", true)) and str(oob_res.get("error", "")).contains("outside"),
		"rejects out-of-bounds spawn coordinates with a clear message")
	# Pixel (773, 1448) is a black wall cell: ROS (62.2725, 33.994)
	var wall_res: Dictionary = app.world.resolve_spawn({"x": 62.2725, "y": 33.994}, true)
	_check(not bool(wall_res.get("ok", true)) and str(wall_res.get("error", "")).contains("wall"),
		"rejects spawn coordinates inside a black wall cell without silently moving to another spot")

	app._leave()
	await process_frame


func _test_online_handshake_and_spawn_enforcement() -> void:
	var port := 27362
	app._select_world("gnss2026_mid360_1017_02")
	app._host("127.0.0.1", port, "OnlineHost")
	_check(app.session.ready_to_play, "host starts online session on gnss2026_mid360_1017_02")

	# Create a client branch with its own MultiplayerAPI in the tree to test the full ENet handshake:
	# 1. Valid client with custom coordinates (--spawn-x 60.0 --spawn-y 30.194 --spawn-yaw-deg -90)
	var client_branch := Node.new()
	client_branch.name = "ClientBranch"
	root.add_child(client_branch)
	var client_mp := MultiplayerAPI.create_default_interface()
	set_multiplayer(client_mp, client_branch.get_path())
	var client_main := Node.new()
	client_main.name = app.name
	client_branch.add_child(client_main)
	var client_world := WorldScript.new()
	client_world.name = "World"
	client_main.add_child(client_world)
	client_world.load_world("cub_playground_v1")  # Client starts on default world before joining!
	var client_session := SessionScript.new()
	client_session.name = "Session"
	client_session.world_node = client_world
	client_main.add_child(client_session)

	var custom_req := {"x": 60.0, "y": 30.194, "yaw_deg": -90.0}
	_check(client_session.join_host("127.0.0.1", port, "pedestrian", "CustomWalker", custom_req) == OK,
		"client initiates join with custom ROS spawn coordinates")
	var joined := await _wait_for(func(): return client_session.ready_to_play and app.session.records.size() == 2, 10.0)
	_check(joined, "client automatically loads host's world (gnss2026_mid360_1017_02), sends Ready, and joins roster")
	_check(client_world.world_id == "gnss2026_mid360_1017_02" and client_world.is_loaded,
		"client switched from cub_playground_v1 to host's gnss2026_mid360_1017_02")
	if joined:
		var cid: int = client_session.local_actor_id
		var crecord: Dictionary = client_session.records[cid]
		var expected_cpos := LidarScript.ros_to_godot(Vector3(60.0, 30.194, 0.1))
		_check(Vector3(crecord["position"]).distance_to(expected_cpos) < 0.02
			and is_equal_approx(float(crecord["yaw"]), deg_to_rad(-90.0)),
			"host validates and broadcasts client's custom spawn coordinates and yaw")
	client_session.leave()
	await _wait_for(func(): return app.session.records.size() == 1, 5.0)

	# 2. Client requesting an invalid spawn inside a wall (62.2725, 33.994) must be rejected by host:
	var bad_statuses := PackedStringArray()
	client_session.status_changed.connect(func(msg: String): bad_statuses.append(msg))
	client_session.join_host("127.0.0.1", port, "cone", "WallCone", {"x": 62.2725, "y": 33.994})
	var rejected := await _wait_for(func():
		for s in bad_statuses:
			if s.contains("Spawn error"):
				return true
		return false, 8.0)
	_check(rejected and not client_session.ready_to_play and app.session.records.size() == 1,
		"host rejects client with wall-colliding spawn coordinates and sends error reason")

	# 3. Client receiving an offer with a mismatched world_hash aborts join with clear message:
	bad_statuses.clear()
	var fake_offer: Dictionary = app.world.world_summary()
	fake_offer["world_hash"] = "0000000000000000000000000000000000000000000000000000000000000000"
	var verify_mismatch: Dictionary = WorldRegistryScript.verify_remote_offer(fake_offer)
	_check(not bool(verify_mismatch.get("ok", true))
		and str(verify_mismatch.get("error", "")).contains("version mismatch")
		and str(verify_mismatch.get("error", "")).contains("gnss2026_mid360_1017_02"),
		"client detects same world_id with different world_hash and reports version mismatch")

	# 4. Multi-process 4-instance test on gnss2026_mid360_1017_02 (1 host + 3 real client processes):
	var report_dir := ProjectSettings.globalize_path("res://.godot/occ-mp-%d" % OS.get_process_id())
	DirAccess.make_dir_recursive_absolute(report_dir)
	var pids := PackedInt32Array()
	var peer_configs := [
		{"name": "MpWalker", "actor": "pedestrian", "extra": ["--spawn", "start_a"]},
		{"name": "MpRobot", "actor": "robot", "extra": ["--spawn-x", "60.0", "--spawn-y", "30.194", "--spawn-yaw-deg", "180"]},
		{"name": "MpCone", "actor": "cone", "extra": []},
	]
	for cfg in peer_configs:
		var pname: String = cfg["name"]
		var rpath := report_dir.path_join(pname + ".json")
		var spath := report_dir.path_join(pname + ".stop")
		var args := PackedStringArray([
			"--path", ProjectSettings.globalize_path("res://"),
			"--audio-driver", "Dummy",
			"--log-file", report_dir.path_join(pname + ".log"),
			"--script", "res://tests/network_peer.gd",
		])
		if DisplayServer.get_name() == "headless":
			args.append("--headless")
		args.append_array(PackedStringArray([
			"--", "--join", "127.0.0.1", "--port", str(port),
			"--name", pname, "--actor", str(cfg["actor"]),
			"--report", rpath, "--exit-after", "25.0", "--stop-file", spath,
		]))
		for item in cfg["extra"]:
			args.append(str(item))
		var pid := OS.create_process(OS.get_executable_path(), args, false)
		if pid > 0:
			pids.append(pid)

	var all_four_joined := await _wait_for(func(): return app.session.records.size() == 4, 20.0)
	_check(all_four_joined, "4 processes (1 host + 3 clients) load gnss2026_mid360_1017_02 and join simultaneously")
	if all_four_joined:
		await create_timer(4.0).timeout
	for cfg in peer_configs:
		var spath := report_dir.path_join(str(cfg["name"]) + ".stop")
		var sf := FileAccess.open(spath, FileAccess.WRITE)
		if sf:
			sf.close()
	for cfg in peer_configs:
		var pname: String = cfg["name"]
		var rpath := report_dir.path_join(pname + ".json")
		await _wait_for(func(): return FileAccess.file_exists(rpath), 10.0)
		if FileAccess.file_exists(rpath):
			var peer_rep: Variant = JSON.parse_string(FileAccess.get_file_as_string(rpath))
			if peer_rep is Dictionary:
				print("GNSS2026 4-PROC PEER %s: max_actors=%s snapshots=%s static_mem_mb=%.2f fps=%s" % [
					pname,
					str(peer_rep.get("maximum_actors", 0)),
					str(peer_rep.get("snapshots", 0)),
					float(peer_rep.get("static_memory_mb", 0.0)),
					str(peer_rep.get("fps_samples", [])),
				])
				_check(int(peer_rep.get("maximum_actors", 0)) == 4 and int(peer_rep.get("snapshots", 0)) > 15,
					"client process %s synchronizes 4 actors on gnss2026_mid360_1017_02" % pname)
	for pid in pids:
		if OS.is_process_running(pid):
			OS.kill(pid)

	# 5. Switch back to cub_playground_v1 after leaving:
	app._leave()
	var back_ok: bool = app._select_world("cub_playground_v1")
	_check(back_ok and app.world.world_id == "cub_playground_v1" and is_equal_approx(app.camera_rig.max_zoom_distance, 65.0),
		"leaving session allows selecting cub_playground_v1 and restores its camera bounds")

	root.remove_child(client_branch)
	client_branch.free()
	root.remove_child(app)
	app.free()


func _ray_index(angle: float) -> int:
	return int(round((angle - LidarScript.ANGLE_MIN) / LidarScript.ANGLE_INCREMENT)) % LidarScript.RAY_COUNT


func _wait_for(condition: Callable, timeout: float) -> bool:
	var deadline := Time.get_ticks_msec() + int(timeout * 1000.0)
	while not condition.call():
		if Time.get_ticks_msec() > deadline:
			return false
		await process_frame
	return true


func _check(condition: bool, description: String) -> void:
	print("%s: %s" % ["PASS" if condition else "FAIL", description])
	if not condition:
		failures.append(description)
