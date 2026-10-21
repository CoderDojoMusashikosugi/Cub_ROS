extends SceneTree

var app: Node3D
var elapsed := 0.0
var connected_once := false
var first_positions: Dictionary = {}
var first_times: Dictionary = {}
var remote_starts: Dictionary = {}
var remote_distance := 0.0
var remote_updates := 0
var primary_model := false
var start_position := Vector3.ZERO
var maximum_actors := 0
var changed_kind := false
var report_path := ""
var stop_path := ""
var exit_after := 30.0
var max_snapshots := 0
var own_distance := 0.0
var fps_samples := PackedFloat32Array()
var _sample_elapsed := 0.0
var _finished := false


func _initialize() -> void:
	_start.call_deferred()


func _start() -> void:
	app = load("res://scenes/main.tscn").instantiate()
	root.add_child(app)
	report_path = str(app.options["report"])
	stop_path = str(app.options.get("stop-file", ""))
	exit_after = float(app.options.get("exit-after", 30.0))


func _process(delta: float) -> bool:
	if not app or _finished:
		return false
	elapsed += delta
	_sample_elapsed += delta
	if elapsed > 3.0 and _sample_elapsed >= 0.5:
		_sample_elapsed = 0.0
		fps_samples.append(Engine.get_frames_per_second())
	app._has_focus = true
	var session: Node = app.session
	if session.ready_to_play:
		primary_model = app.actors.has(1) and app.actors[1]._visual.get_node_or_null("Cub4") != null
		maximum_actors = maxi(maximum_actors, session.records.size())
		max_snapshots = maxi(max_snapshots, session.received_snapshots)
		var id: int = session.local_actor_id
		if not connected_once:
			connected_once = true
			start_position = app.actors[id].position
			for record: Dictionary in session.records.values():
				var at: Vector3 = record["position"]
				first_positions[record["nickname"]] = [at.x, at.y, at.z]
				first_times[record["nickname"]] = record["sample_time"]
		for remote_id: int in app.actors:
			var remote: Node3D = app.actors[remote_id]
			if remote.simulated:
				continue
			if not remote_starts.has(remote_id):
				remote_starts[remote_id] = remote.position
			remote_distance = maxf(remote_distance, remote.position.distance_to(remote_starts[remote_id]))
			remote_updates = maxi(remote_updates, remote.received_updates)
		if elapsed < 4.5:
			Input.action_press("move_forward")
		else:
			Input.action_release("move_forward")
		if app.actors.has(id):
			own_distance = maxf(own_distance, app.actors[id].position.distance_to(start_position))
		if str(app.options.get("name", "")) == "Early2" and elapsed > 3.0 and not changed_kind:
			changed_kind = true
			session.change_kind("cone")
	elif connected_once:
		_finish("server_disconnected")
	if elapsed >= exit_after:
		_finish("completed" if connected_once else "connection_failed")
	elif not stop_path.is_empty() and FileAccess.file_exists(stop_path):
		_finish("completed")
	return false


func _finish(reason: String) -> void:
	if _finished:
		return
	_finished = true
	Input.action_release("move_forward")
	var report := {"reason": reason, "connected": connected_once, "maximum_actors": maximum_actors,
		"snapshots": max_snapshots, "distance": own_distance, "first_positions": first_positions,
		"first_times": first_times, "remote_distance": remote_distance, "remote_updates": remote_updates,
		"primary_model": primary_model,
		"fps_samples": Array(fps_samples), "draw_calls": Performance.get_monitor(Performance.RENDER_TOTAL_DRAW_CALLS_IN_FRAME),
		"static_memory_mb": Performance.get_monitor(Performance.MEMORY_STATIC) / 1048576.0}
	var file := FileAccess.open(report_path, FileAccess.WRITE)
	if file:
		file.store_string(JSON.stringify(report))
	print("PEER REPORT: ", JSON.stringify(report))
	app.session.leave()
	quit(0 if connected_once else 1)
