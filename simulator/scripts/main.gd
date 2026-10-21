extends Node3D

const ActorScript = preload("res://scripts/actor.gd")
const WorldScript = preload("res://scripts/world.gd")
const SessionScript = preload("res://scripts/session.gd")
const CameraScript = preload("res://scripts/camera_rig.gd")
const HudScript = preload("res://scripts/hud.gd")
const CAMERA_MODES := {"tps": 0, "3d": 1, "orbit": 1, "2d": 2, "top": 2}

var session: Node
var world: Node3D
var camera_rig: Node3D
var hud: CanvasLayer
var actors: Dictionary = {}
var options: Dictionary = {}
var _has_focus := true
var _publish_elapsed := 0.0
var _simulation_time := 0.0
var _hud_elapsed := 0.0
var _target_id := 0
var _cursor_released := false
var _pending_kind := "robot"
var _playing := false


func _ready() -> void:
	_register_input()
	Engine.max_fps = 45
	world = WorldScript.new()
	world.name = "World"
	add_child(world)
	camera_rig = CameraScript.new()
	camera_rig.name = "CameraRig"
	add_child(camera_rig)
	hud = HudScript.new()
	add_child(hud)
	hud.host_requested.connect(_host)
	hud.join_requested.connect(_join)
	hud.cancel_requested.connect(_leave)
	session = SessionScript.new()
	session.name = "Session"
	session.actor_added.connect(_add_actor)
	session.actor_removed.connect(_remove_actor)
	session.pose_received.connect(_receive_pose)
	session.status_changed.connect(_status_changed)
	session.session_reset.connect(_reset)
	session.roster_changed.connect(_refresh_session)
	add_child(session)
	options = parse_options(OS.get_cmdline_user_args())
	var argument_error := validate_options(options)
	if argument_error.is_empty():
		options["actor"] = "robot" if options.has("host") else str(options.get("actor", "pedestrian" if options.has("join") else "robot"))
		options["name"] = str(options.get("name", "Host" if options.has("host") else ("Guest" if options.has("join") else "Player")))
		_pending_kind = options["actor"]
		hud.set_options(options)
	get_window().focus_entered.connect(func(): _set_window_focus(true))
	get_window().focus_exited.connect(func(): _set_window_focus(false))
	_set_window_focus(DisplayServer.get_name() == "headless" or get_window().has_focus())
	if not argument_error.is_empty():
		_status_changed("Launch arguments: " + argument_error)
		return
	_select_camera(CAMERA_MODES[str(options.get("camera", "tps"))])
	if options.has("host"):
		_host(str(options.get("bind", "127.0.0.1")), int(options.get("port", 27140)), str(options.get("name", "Host")))
	elif options.has("join"):
		_join(str(options["join"]), int(options.get("port", 27140)), str(options.get("actor", "pedestrian")), str(options.get("name", "Guest")))
	_update_hud()
	if options.has("run-seconds") and float(options["run-seconds"]) > 0.0:
		get_tree().create_timer(float(options["run-seconds"])).timeout.connect(func(): get_tree().quit())


func _physics_process(delta: float) -> void:
	if not session.ready_to_play:
		return
	_simulation_time += delta
	var movement := Vector2.ZERO
	var fast := false
	if _has_focus and not _cursor_released:
		movement = Input.get_vector("move_left", "move_right", "move_backward", "move_forward")
		fast = Input.is_action_pressed("move_fast")
		var pan_direction := Vector2(
			float(Input.is_physical_key_pressed(KEY_L)) - float(Input.is_physical_key_pressed(KEY_J)),
			float(Input.is_physical_key_pressed(KEY_K)) - float(Input.is_physical_key_pressed(KEY_I)))
		camera_rig.pan(pan_direction.limit_length(), delta)
	var id: int = session.local_actor_id
	if actors.has(id):
		actors[id].simulate(movement, camera_rig.yaw if camera_rig.mode != 2 else 0.0, fast, delta)
	_publish_elapsed += delta
	if _publish_elapsed >= 0.05:
		_publish_elapsed = fmod(_publish_elapsed, 0.05)
		if actors.has(id):
			session.publish_pose(id, actors[id].position, actors[id].rotation.y, _simulation_time)


func _process(delta: float) -> void:
	for actor: Node3D in actors.values():
		actor.interpolate_remote(delta)
	camera_rig.update(delta)
	_hud_elapsed += delta
	if _hud_elapsed > 0.1:
		_hud_elapsed = 0.0
		_update_hud()


func _notification(what: int) -> void:
	if what == NOTIFICATION_APPLICATION_FOCUS_OUT:
		_set_window_focus(false)
	elif what == NOTIFICATION_APPLICATION_FOCUS_IN:
		_set_window_focus(true)
	elif what == NOTIFICATION_WM_CLOSE_REQUEST:
		Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
		if session:
			session.leave()


func _set_window_focus(value: bool) -> void:
	_has_focus = value
	Engine.max_fps = 45 if value else 30
	_update_mouse_capture()


func _update_mouse_capture() -> void:
	if not camera_rig:
		return
	camera_rig.look_enabled = session != null and session.ready_to_play and _has_focus and not _cursor_released
	if not camera_rig.look_enabled:
		camera_rig.reset_drag()
	if DisplayServer.get_name() != "headless":
		var captured: bool = camera_rig.look_enabled and camera_rig.mode == 0
		var mouse_mode := Input.MOUSE_MODE_CAPTURED if captured else Input.MOUSE_MODE_VISIBLE
		if Input.mouse_mode != mouse_mode:
			Input.mouse_mode = mouse_mode


func _exit_tree() -> void:
	Input.mouse_mode = Input.MOUSE_MODE_VISIBLE


func _unhandled_key_input(event: InputEvent) -> void:
	if not event is InputEventKey or not event.pressed or event.echo:
		return
	if not session.ready_to_play or not _has_focus:
		return
	match event.physical_keycode:
		KEY_ESCAPE:
			if camera_rig.mode == 0:
				_cursor_released = true
			else:
				camera_rig.reset_drag()
			_update_mouse_capture()
		KEY_TAB:
			if camera_rig.mode == 0:
				_cursor_released = not _cursor_released
			_update_mouse_capture()
		KEY_T:
			if session.is_host:
				return
			var index := (SessionScript.KINDS.find(_pending_kind) + 1) % SessionScript.KINDS.size()
			_pending_kind = SessionScript.KINDS[index]
			session.change_kind(_pending_kind)
		KEY_BACKSPACE:
			_leave()
		KEY_1:
			_select_camera(0)
		KEY_2:
			_select_camera(1)
		KEY_3:
			_select_camera(2)
		KEY_F:
			camera_rig.recenter()
		_:
			return
	get_viewport().set_input_as_handled()
	_update_hud()


func _host(address: String, port: int, nickname: String) -> void:
	session.start_host(port, nickname, address)
	_refresh_session()


func _join(address: String, port: int, kind: String, nickname: String) -> void:
	session.join_host(address, port, kind, nickname)
	_refresh_session()


func _leave() -> void:
	session.leave()
	_status_changed("OFFLINE")
	_refresh_session()


func _add_actor(record: Dictionary) -> void:
	var id := int(record["id"])
	if actors.has(id):
		var previous: Node3D = actors[id]
		remove_child(previous)
		previous.queue_free()
	var actor := ActorScript.new()
	actor.name = "Actor_%d" % id
	actor.configure(record, id == session.local_actor_id)
	add_child(actor)
	actors[id] = actor
	if id == _target_id:
		camera_rig.target = actor
	_refresh_session()


func _remove_actor(id: int) -> void:
	if not actors.has(id):
		return
	var actor: Node3D = actors[id]
	actors.erase(id)
	remove_child(actor)
	actor.queue_free()
	_refresh_session()


func _receive_pose(id: int, at: Vector3, yaw: float, sample_time: float) -> void:
	if actors.has(id):
		actors[id].receive_pose(at, yaw, sample_time)


func _reset() -> void:
	camera_rig.target = null
	_target_id = 0
	for actor: Node3D in actors.values():
		remove_child(actor)
		actor.queue_free()
	actors.clear()
	_publish_elapsed = 0.0
	_simulation_time = 0.0
	_refresh_session()


func _refresh_session() -> void:
	var id: int = session.local_actor_id
	for actor_id: int in actors:
		var actor: Node3D = actors[actor_id]
		if actor.controlled != (actor_id == id) or actor.simulated != (actor_id == id):
			actor.set_controlled(actor_id == id)
	if actors.has(id) and (_target_id != id or camera_rig.target != actors[id]):
		_target_id = id
		camera_rig.target = actors[id]
		camera_rig.recenter()
	if session.ready_to_play:
		_pending_kind = session.selected_kind
	if session.ready_to_play and not _playing:
		_cursor_released = false
		var focus := get_viewport().gui_get_focus_owner()
		if focus:
			focus.release_focus()
	_playing = session.ready_to_play
	_update_mouse_capture()
	_update_hud()
	var role := "HOST" if session.is_host else "CLIENT"
	DisplayServer.window_set_title("Cub Simulator | %s | %s" % [role, session.nickname] if session.ready_to_play else "Cub Simulator | Lobby")


func _status_changed(message: String) -> void:
	hud.set_status(message)
	_refresh_session()
	print(message)


func _select_camera(mode: int) -> void:
	camera_rig.select_mode(mode)
	_cursor_released = false
	_update_mouse_capture()
	_update_hud()


func _update_hud() -> void:
	if not session or not hud:
		return
	var id: int = session.local_actor_id
	var actor: Node3D = actors.get(id)
	hud.update_readout({"active": session.ready_to_play, "connecting": session._joining,
		"host": session.is_host, "kind": _pending_kind, "camera": camera_rig.mode,
		"position": actor.position if is_instance_valid(actor) else null,
		"yaw": rad_to_deg(actor.rotation.y) if is_instance_valid(actor) else 0.0,
		"fps": Engine.get_frames_per_second(), "actors": actors.size(), "cursor_free": _cursor_released})


func _register_input() -> void:
	var bindings := {
		"move_forward": [KEY_W, KEY_UP], "move_backward": [KEY_S, KEY_DOWN],
		"move_left": [KEY_A, KEY_LEFT], "move_right": [KEY_D, KEY_RIGHT],
		"move_fast": [KEY_SHIFT],
	}
	for action: String in bindings:
		if not InputMap.has_action(action):
			InputMap.add_action(action)
		for key: int in bindings[action]:
			var event := InputEventKey.new()
			event.physical_keycode = key
			InputMap.action_add_event(action, event)


static func parse_options(arguments: PackedStringArray) -> Dictionary:
	var parsed: Dictionary = {}
	var index := 0
	while index < arguments.size():
		var argument := arguments[index]
		# Godot can combine a common '--' with another one from instance args.
		if argument.begins_with("--") and argument != "--":
			var key := argument.substr(2)
			if "=" in key:
				var pair := key.split("=", true, 1)
				parsed[pair[0]] = pair[1]
			elif index + 1 < arguments.size() and not arguments[index + 1].begins_with("--"):
				index += 1
				parsed[key] = arguments[index]
			else:
				parsed[key] = true
		index += 1
	return parsed


static func validate_options(parsed: Dictionary) -> String:
	if parsed.has("host") and parsed.has("join"):
		return "Use either --host or --join ADDRESS."
	if parsed.has("host") and not parsed["host"] is bool:
		return "--host is a flag; use --bind ADDRESS to set its address."
	for key in ["join", "bind", "port", "actor", "name", "camera", "run-seconds"]:
		if parsed.has(key) and parsed[key] is bool:
			return "--%s requires a value." % key
	if parsed.has("join") and str(parsed["join"]).strip_edges().is_empty():
		return "--join requires a host address."
	if parsed.has("actor") and str(parsed["actor"]) not in SessionScript.KINDS:
		return "--actor must be robot, pedestrian or cone."
	if parsed.has("camera") and not CAMERA_MODES.has(str(parsed["camera"])):
		return "--camera must be tps, 3d or 2d (orbit / top are also accepted)."
	if parsed.has("port"):
		var port := str(parsed["port"])
		if not port.is_valid_int() or int(port) < 1024 or int(port) > 65535:
			return "--port must be an integer from 1024 to 65535."
	if parsed.has("run-seconds"):
		var seconds := str(parsed["run-seconds"])
		if not seconds.is_valid_float() or not is_finite(float(seconds)) or float(seconds) < 0.0:
			return "--run-seconds must be a non-negative number (0 disables the timer)."
	return ""
