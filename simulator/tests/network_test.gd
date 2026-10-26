extends SceneTree

var app: Node3D
var children := PackedInt32Array()
var reports: Dictionary = {}
var failures := PackedStringArray()
var report_directory := ""
var port := 27341
var spawn_positions: Dictionary = {}


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	report_directory = ProjectSettings.globalize_path("res://.godot/network-test-%d" % OS.get_process_id())
	DirAccess.make_dir_recursive_absolute(report_directory)
	app = load("res://scenes/main.tscn").instantiate()
	root.add_child(app)
	app.ros_bridge.url = "ws://127.0.0.1:1"
	_check(app.session.start_host(port, "TestHost") == OK, "host starts with the primary robot")
	app.session.actor_added.connect(func(record: Dictionary):
		if not spawn_positions.has(record["id"]):
			spawn_positions[record["id"]] = record["position"])
	_spawn_peer("Early1", "pedestrian", 30.0)
	_spawn_peer("Early2", "robot", 30.0)
	var join_timeout := 12.0 if DisplayServer.get_name() == "headless" else 25.0
	var joined := await _wait_for(func(): return app.session.records.size() == 3, join_timeout)
	_check(joined, "two clients join alongside the host's primary robot")
	if not joined:
		await _finish()
		return
	var early_id := _actor_named("Early1")
	var early_start: Vector3 = spawn_positions[early_id]
	await _wait_for(func(): return (app.actors[early_id].received_updates > 20
		and app.actors[early_id].position.distance_to(early_start) > 1.0), 6.0)
	var early_pose: Vector3 = app.session.records[early_id]["position"]
	_check(early_pose.distance_to(early_start) > 1.0, "client movement reaches host")
	_check(app.actors[early_id].received_updates > 10
		and app.actors[early_id].position.distance_to(early_start) > 0.5,
		"host renders the client's timestamped motion")
	_spawn_peer("Late3", "cone", 30.0)
	joined = await _wait_for(func(): return app.session.records.size() == 4, join_timeout)
	_check(joined, "third client joins a running session")
	await create_timer(2.0).timeout
	var primary_robot: Node3D = app.actors[1]
	app.session.change_kind("pedestrian")
	app.session.change_kind("cone")
	_check(app.session.local_actor_id == 1 and app.session.selected_kind == "robot"
		and app.actors[1] == primary_robot and primary_robot.simulated and primary_robot.controlled
		and app.session.records.size() == 4,
		"host actor changes are ignored without creating an extra avatar")
	Input.action_press("move_forward")
	app._has_focus = true
	var host_start: Vector3 = primary_robot.position
	await create_timer(0.6).timeout
	Input.action_release("move_forward")
	_check(Vector2(primary_robot.position.x - host_start.x, primary_robot.position.z - host_start.z).length() < 0.01, "host keyboard cannot bypass ROS velocity control")
	await _wait_for(func(): return _actor_named("Early2") != 0 and app.session.records[_actor_named("Early2")]["kind"] == "cone", 5.0)
	_check(_actor_named("Early2") != 0 and app.session.records[_actor_named("Early2")]["kind"] == "cone",
		"client can change its actor type")
	# Let the host choose when to disconnect, so shader/driver startup time does
	# not make the first client exit before the late joiner finishes loading.
	var stop_file := FileAccess.open(report_directory.path_join("Early1.stop"), FileAccess.WRITE)
	stop_file.close()
	var left := await _wait_for(func(): return not app.session.records.has(early_id), 12.0)
	_check(left and not app.actors.has(early_id), "client disconnect removes its actor")
	var first := _read_report("Early1")
	_check(first.get("maximum_actors", 0) == 4 and first.get("snapshots", 0) > 20,
		"client receives roster updates and continuous snapshots")
	_check(first.get("remote_distance", 0.0) > 0.5 and first.get("remote_updates", 0) > 10,
		"client renders timestamped motion relayed from another client")
	_check(first.get("primary_model", false), "client sees the primary robot as Cub4")
	app.session.leave()
	for nickname in ["Early2", "Late3"]:
		await _wait_for(func(): return FileAccess.file_exists(reports[nickname]), 12.0)
		var report := _read_report(nickname)
		_check(report.get("reason", "") == "server_disconnected", "%s detects host exit" % nickname)
		_check(float(report.get("distance", 0.0)) > 0.5, "%s controls its own actor" % nickname)
		_check(float(report.get("remote_distance", 0.0)) > 0.5 and int(report.get("remote_updates", 0)) > 10,
			"%s renders remote avatars while controlling its own" % nickname)
		_check(report.get("primary_model", false), "%s sees Cub4, including after late joining" % nickname)
	var late := _read_report("Late3")
	var initial: Dictionary = late.get("first_positions", {})
	if initial.has("Early1"):
		var at: Array = initial["Early1"]
		_check(Vector3(at[0], at[1], at[2]).distance_to(early_start) > 1.0,
			"late join receives current positions, not original spawn points")
		_check(float(late.get("first_times", {}).get("Early1", 0.0)) > 0.5,
			"late join initializes interpolation with the existing source timeline")
	else:
		_check(false, "late join receives earlier actors")
	await _finish()


func _spawn_peer(nickname: String, kind: String, lifetime: float) -> void:
	var path := report_directory.path_join(nickname + ".json")
	reports[nickname] = path
	var arguments := PackedStringArray(["--path", ProjectSettings.globalize_path("res://"),
		"--audio-driver", "Dummy", "--log-file", report_directory.path_join(nickname + ".log"),
		"--script", "res://tests/network_peer.gd"])
	if DisplayServer.get_name() == "headless":
		arguments.append("--headless")
	else:
		arguments.append_array(PackedStringArray(["--resolution", "800x480", "--position",
			"%d,%d" % [(children.size() % 2) * 820, (children.size() / 2) * 520 + 40]]))
	arguments.append_array(PackedStringArray(["--", "--join", "127.0.0.1", "--port", str(port),
		"--name", nickname, "--actor", kind, "--report", path, "--exit-after", str(lifetime),
		"--stop-file", report_directory.path_join(nickname + ".stop")]))
	var pid := OS.create_process(OS.get_executable_path(), arguments, false)
	_check(pid > 0, "start process " + nickname)
	children.append(pid)


func _actor_named(nickname: String) -> int:
	for id: int in app.session.records:
		if app.session.records[id]["nickname"] == nickname:
			return id
	return 0


func _wait_for(condition: Callable, timeout: float) -> bool:
	var deadline := Time.get_ticks_msec() + int(timeout * 1000)
	while not condition.call():
		if Time.get_ticks_msec() > deadline:
			return false
		await create_timer(0.05).timeout
	return true


func _read_report(nickname: String) -> Dictionary:
	if not FileAccess.file_exists(reports[nickname]):
		return {}
	var parsed: Variant = JSON.parse_string(FileAccess.get_file_as_string(reports[nickname]))
	return parsed if parsed is Dictionary else {}


func _check(condition: bool, description: String) -> void:
	print("%s: %s" % ["PASS" if condition else "FAIL", description])
	if not condition:
		failures.append(description)


func _finish() -> void:
	Input.action_release("move_forward")
	app.session.leave()
	await create_timer(0.3).timeout
	for pid in children:
		if OS.is_process_running(pid):
			OS.kill(pid)
	if failures.is_empty():
		print("NETWORK TESTS PASSED / one host + three client processes")
		quit(0)
	else:
		for failure in failures:
			push_error(failure)
		print("Logs: ", report_directory)
		quit(1)
