extends SceneTree
## Start real processes with the same arguments used by Customize Run Instances.

const MainScript = preload("res://scripts/main.gd")
const CASES := [
	{"name": "Host", "actor": "robot", "launch_actor": "cone", "camera": "tps", "mode": 0},
	{"name": "Walker", "actor": "pedestrian", "camera": "3d", "mode": 1},
	{"name": "Other Robot", "actor": "robot", "camera": "tps", "mode": 0},
	{"name": "Cone", "actor": "cone", "camera": "2d", "mode": 2},
]
var children := PackedInt32Array()
var failures := PackedStringArray()
var report_directory := ""


func _initialize() -> void:
	if MainScript.parse_options(OS.get_cmdline_user_args()).has("probe-output"):
		_probe.call_deferred()
	else:
		_run.call_deferred()


func _run() -> void:
	Engine.max_fps = 45
	report_directory = ProjectSettings.globalize_path("res://.godot/launch-test-%d" % OS.get_process_id())
	DirAccess.make_dir_recursive_absolute(report_directory)
	# Clients may be scheduled before the host when the editor launches all four.
	for index in [1, 2, 3]:
		_spawn(str(index), _arguments(index))
	await create_timer(0.8).timeout
	_spawn("0", _arguments(0))
	for index in range(4):
		var report := await _read_report(str(index))
		var expected: Dictionary = CASES[index]
		_check(report.get("ready", false) and report.get("host", false) == (index == 0),
			"instance %d starts with its requested host/client role" % index)
		_check(report.get("nickname", "") == expected["name"] and report.get("actor", "") == expected["actor"],
			"instance %d uses the requested name and actor" % index)
		_check(report.get("camera", -1) == expected["mode"] and report.get("target", false),
			"instance %d starts in its requested camera mode targeting its own actor" % index)
		_check(report.get("game_hud", false) and report.get("max_actors", 0) == 4 and report.get("snapshots", 0) > 20,
			"instance %d enters gameplay and synchronizes all four actors" % index)
	_spawn("invalid", PackedStringArray(["--host", "--join", "127.0.0.1"]))
	var invalid := await _read_report("invalid")
	_check(not invalid.get("ready", true) and invalid.get("lobby", false)
		and str(invalid.get("status", "")).begins_with("Launch arguments:"),
		"conflicting startup roles show an error in the lobby without starting a session")
	_spawn("lobby", PackedStringArray(["--actor=cone", "--camera=top", "--name=Setup"]))
	var lobby := await _read_report("lobby")
	_check(not lobby.get("ready", true) and lobby.get("lobby", false)
		and lobby.get("camera", -1) == 2 and lobby.get("selected_actor", "") == "cone",
		"arguments without a role configure the lobby and initial camera without connecting")
	await create_timer(0.3).timeout
	for id in ["0", "1", "2", "3", "invalid", "lobby"]:
		var log_path := report_directory.path_join(id + ".log")
		var log_text := FileAccess.get_file_as_string(log_path) if FileAccess.file_exists(log_path) else "\nERROR: no log"
		_check("\nERROR:" not in log_text and "\nSCRIPT ERROR:" not in log_text,
			"instance %s starts and exits without engine errors" % id)
	for pid in children:
		if OS.is_process_running(pid):
			OS.kill(pid)
	print("LAUNCH TESTS PASSED / editor-style arguments, one host + three clients" if failures.is_empty() else "LAUNCH TESTS FAILED: " + report_directory)
	quit(0 if failures.is_empty() else 1)


func _arguments(index: int) -> PackedStringArray:
	var config: Dictionary = CASES[index]
	var arguments := PackedStringArray(["--camera=" + config["camera"], "--name", config["name"]])
	if index == 0:
		arguments.append("--host")
	else:
		arguments.append_array(PackedStringArray(["--join", "127.0.0.1"]))
	# Also exercise the client actor default, which should match the lobby.
	if index != 1:
		arguments.append_array(PackedStringArray(["--actor", str(config.get("launch_actor", config["actor"]))]))
	return arguments


func _spawn(id: String, arguments: PackedStringArray) -> void:
	var engine_args := PackedStringArray(["--path", ProjectSettings.globalize_path("res://"),
		"--audio-driver", "Dummy", "--quit-after", "900", "--log-file", report_directory.path_join(id + ".log"),
		"--script", "res://tests/launch_test.gd"])
	if DisplayServer.get_name() == "headless":
		engine_args.append("--headless")
	# Common Main Run Args followed by instance Launch Arguments, including a
	# repeated separator to exercise the editor's concatenation behavior.
	engine_args.append_array(PackedStringArray(["--", "--port=27346", "--"]))
	engine_args.append_array(arguments)
	engine_args.append_array(PackedStringArray(["--probe-output", report_directory.path_join(id + ".json")]))
	var pid := OS.create_process(OS.get_executable_path(), engine_args, false)
	_check(pid > 0, "start process " + id)
	if pid > 0:
		children.append(pid)


func _read_report(id: String) -> Dictionary:
	var path := report_directory.path_join(id + ".json")
	var deadline := Time.get_ticks_msec() + 14000
	while not FileAccess.file_exists(path) and Time.get_ticks_msec() < deadline:
		await create_timer(0.1).timeout
	if not FileAccess.file_exists(path):
		return {}
	var data: Variant = JSON.parse_string(FileAccess.get_file_as_string(path))
	return data if data is Dictionary else {}


func _probe() -> void:
	var app: Node3D = load("res://scenes/main.tscn").instantiate()
	root.add_child(app)
	var report: Dictionary = {"ready": false, "max_actors": 0, "snapshots": 0}
	var duration := 6.0 if app.session.ready_to_play or app.session._joining else 0.3
	var deadline := Time.get_ticks_msec() + int(duration * 1000)
	while Time.get_ticks_msec() < deadline:
		if app.session.ready_to_play and app.actors.has(app.session.local_actor_id):
			report["ready"] = true
			report["host"] = app.session.is_host
			report["nickname"] = app.session.nickname
			report["actor"] = app.session.records[app.session.local_actor_id]["kind"]
			report["target"] = app.camera_rig.target == app.actors[app.session.local_actor_id]
			report["game_hud"] = app.hud._bar.is_visible_in_tree() and not app.hud._lobby.is_visible_in_tree()
			report["max_actors"] = maxi(report["max_actors"], app.actors.size())
			report["snapshots"] = maxi(report["snapshots"], app.session.sent_snapshots if app.session.is_host else app.session.received_snapshots)
		await create_timer(0.05).timeout
	report["camera"] = app.camera_rig.mode
	report["lobby"] = app.hud._lobby.is_visible_in_tree()
	report["selected_actor"] = app.hud.KINDS[app.hud._kind.selected]
	report["status"] = app.hud._connection_status
	var output := FileAccess.open(str(app.options["probe-output"]), FileAccess.WRITE)
	output.store_string(JSON.stringify(report))
	output.close()
	app.session.leave()
	quit()


func _check(condition: bool, description: String) -> void:
	print("%s: %s" % ["PASS" if condition else "FAIL", description])
	if not condition:
		failures.append(description)
