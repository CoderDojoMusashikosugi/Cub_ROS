extends Node3D

const ActorScript = preload("res://scripts/actor.gd")
const WorldScript = preload("res://scripts/world.gd")
const WorldRegistryScript = preload("res://scripts/world_registry.gd")
const SessionScript = preload("res://scripts/session.gd")
const CameraScript = preload("res://scripts/camera_rig.gd")
const HudScript = preload("res://scripts/hud.gd")
const RosBridgeScript = preload("res://scripts/ros/ros_bridge.gd")
const JoyInputScript = preload("res://scripts/ros/joy_input.gd")
const LidarScript = preload("res://scripts/ros/lidar.gd")
const OdomScript = preload("res://scripts/ros/odom.gd")
const CAMERA_MODES := {"tps": 0, "3d": 1, "orbit": 1, "2d": 2, "top": 2}
const VALUE_OPTION_KEYS := [
	"join", "bind", "port", "actor", "name", "camera", "run-seconds", "ros-url", "joy-device",
	"world", "spawn", "spawn-x", "spawn-y", "spawn-z", "spawn-yaw-deg",
]

var ros_bridge: Node
var joy_input = JoyInputScript.new()
var lidar = LidarScript.new()
var odom = OdomScript.new()
var session: Node
var world: Node3D
var camera_rig: Node3D
var hud: CanvasLayer
var actors: Dictionary = {}
var options: Dictionary = {}
var available_worlds: Array[Dictionary] = []
var _cli_spawn_request: Dictionary = {}
var _has_focus := true
var _publish_elapsed := 0.0
var _scan_elapsed := 0.0
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
	hud.world_selected.connect(_on_lobby_world_selected)
	hud.spawn_selected.connect(_on_lobby_spawn_selected)
	session = SessionScript.new()
	session.name = "Session"
	session.world_node = world
	session.actor_added.connect(_add_actor)
	session.actor_removed.connect(_remove_actor)
	session.pose_received.connect(_receive_pose)
	session.status_changed.connect(_status_changed)
	session.session_reset.connect(_reset)
	session.roster_changed.connect(_refresh_session)
	session.world_changed.connect(_on_session_world_changed)
	add_child(session)
	ros_bridge = RosBridgeScript.new()
	ros_bridge.teleport_requested.connect(_teleport_primary_from_ros)
	add_child(ros_bridge)

	available_worlds = WorldRegistryScript.list_worlds()
	options = parse_options(OS.get_cmdline_user_args())
	var argument_error := validate_options(options)
	var requested_world_id := str(options.get("world", WorldRegistryScript.DEFAULT_WORLD_ID)).strip_edges()
	if requested_world_id.is_empty():
		requested_world_id = WorldRegistryScript.DEFAULT_WORLD_ID

	hud.set_worlds(available_worlds, requested_world_id)
	_select_world(requested_world_id, str(options.get("spawn", "")))

	if argument_error.is_empty():
		options["actor"] = "robot" if options.has("host") else str(options.get("actor", "pedestrian" if options.has("join") else "robot"))
		options["name"] = str(options.get("name", "Host" if options.has("host") else ("Guest" if options.has("join") else "Player")))
		_pending_kind = options["actor"]
		hud.set_options(options)
		ros_bridge.url = str(options.get("ros-url", "ws://127.0.0.1:9090"))
		joy_input.preferred_device_id = int(options.get("joy-device", -1))
		_cli_spawn_request = extract_spawn_request(options)
		session.local_spawn_request = _cli_spawn_request.duplicate(true)
		# Validate startup world and spawn for lobby or host startup.
		if options.has("world") and not world.is_loaded:
			argument_error = world.load_error
		elif not options.has("join") and not _cli_spawn_request.is_empty():
			var check: Dictionary = world.resolve_spawn(_cli_spawn_request, true, 0)
			if not bool(check.get("ok", false)):
				argument_error = str(check.get("error", "Invalid spawn arguments."))

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
		if session.is_host:
			var command: Vector2 = ros_bridge.command()
			actors[id].simulate_velocity(command.x, command.y, delta)
		else:
			actors[id].simulate(movement, camera_rig.yaw if camera_rig.mode != 2 else camera_rig.TOP_YAW, fast, delta)
	_publish_elapsed += delta
	if _publish_elapsed >= 0.05:
		_publish_elapsed = fmod(_publish_elapsed, 0.05)
		if session.is_host:
			ros_bridge.send_joy(joy_input.sample())
			if actors.has(1):
				ros_bridge.send_odom(odom.sample(actors[1]))
		if actors.has(id):
			session.publish_pose(id, actors[id].position, actors[id].rotation.y, _simulation_time)
	if session.is_host and actors.has(1):
		_scan_elapsed += delta
		if _scan_elapsed >= lidar.SCAN_INTERVAL:
			_scan_elapsed = fmod(_scan_elapsed, lidar.SCAN_INTERVAL)
			ros_bridge.send_scan(lidar.sample(actors[1], actors))


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
	if ros_bridge:
		ros_bridge.stop()


func _unhandled_input(event: InputEvent) -> void:
	if not session.ready_to_play or not _has_focus or camera_rig.mode != 0 or not _cursor_released:
		return
	if event is InputEventMouseButton and event.pressed and event.button_index == MOUSE_BUTTON_LEFT:
		_cursor_released = false
		_update_mouse_capture()
		_update_hud()
		get_viewport().set_input_as_handled()


func _unhandled_key_input(event: InputEvent) -> void:
	if not event is InputEventKey or not event.pressed or event.echo:
		return
	if not session.ready_to_play or not _has_focus:
		return
	match event.physical_keycode:
		KEY_ESCAPE:
			if camera_rig.mode == 0:
				_cursor_released = not _cursor_released
			else:
				camera_rig.reset_drag()
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
		KEY_P:
			if session.is_host:
				ros_bridge.send_proceed_to_next_group()
		_:
			return
	get_viewport().set_input_as_handled()
	_update_hud()


func _select_world(target_world_id: String, preferred_preset := "") -> bool:
	var ok: bool = world.load_world(target_world_id)
	hud.set_worlds(available_worlds, target_world_id)
	if ok:
		camera_rig.configure_world_bounds(world.get_world_bounds_godot())
		hud.set_spawn_presets(world.definition, preferred_preset)
		var preset_id: String = hud.selected_spawn_preset()
		if not preset_id.is_empty():
			world.active_spawn_preset = preset_id
		session.world_info = world.world_summary()
		var size_arr: Variant = world.definition.get("bounds", {}).get("size_m", [50, 50, 2])
		var dim_text := "%.1f x %.1f m" % [float(size_arr[0]), float(size_arr[1])] if (size_arr is Array and size_arr.size() >= 2) else ""
		hud.set_world_status(true, "World: %s [%s] (%s) — READY. Join follows host world." % [
			world.display_name, world.kind, dim_text
		])
	else:
		hud.set_spawn_presets({}, "")
		hud.set_world_status(false, "World error: " + world.load_error)
	_update_hud()
	return ok


func _on_lobby_world_selected(world_id: String) -> void:
	if session.ready_to_play or session._joining:
		return
	_cli_spawn_request.clear()
	_select_world(world_id)


func _on_lobby_spawn_selected(preset_name: String) -> void:
	if session.ready_to_play or session._joining:
		return
	_cli_spawn_request.clear()
	world.active_spawn_preset = preset_name
	session.local_spawn_request = {"spawn": preset_name}


func _on_session_world_changed(_info: Dictionary) -> void:
	if world and world.is_loaded:
		camera_rig.configure_world_bounds(world.get_world_bounds_godot())
		hud.set_worlds(available_worlds, world.world_id)
		hud.set_spawn_presets(world.definition, world.active_spawn_preset)
		hud.set_world_status(true, "World: %s [%s] — READY." % [world.display_name, world.kind])
	_update_hud()


func _host(address: String, port: int, nickname: String) -> void:
	var target_world_id: String = hud.selected_world_id()
	if not target_world_id.is_empty() and (world.world_id != target_world_id or not world.is_loaded):
		_select_world(target_world_id, hud.selected_spawn_preset())
	if not world.is_loaded:
		_status_changed("Cannot host: " + world.load_error)
		return
	var spawn_req := _cli_spawn_request.duplicate(true)
	if spawn_req.is_empty():
		var preset: String = hud.selected_spawn_preset()
		if not preset.is_empty():
			spawn_req = {"spawn": preset}
	session.local_spawn_request = spawn_req
	ros_bridge.url = hud.ros_url()
	joy_input.preferred_device_id = int(options.get("joy-device", -1))
	session.start_host(port, nickname, address, spawn_req)
	_refresh_session()


func _join(address: String, port: int, kind: String, nickname: String) -> void:
	var spawn_req := _cli_spawn_request.duplicate(true)
	session.local_spawn_request = spawn_req
	session.join_host(address, port, kind, nickname, spawn_req)
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
	if ros_bridge:
		ros_bridge.stop()
	camera_rig.target = null
	_target_id = 0
	for actor: Node3D in actors.values():
		remove_child(actor)
		actor.queue_free()
	actors.clear()
	_publish_elapsed = 0.0
	_scan_elapsed = 0.0
	_simulation_time = 0.0
	lidar.last_scan = {}
	odom.last_odom = {}
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
	if ros_bridge:
		if session.ready_to_play and session.is_host:
			ros_bridge.start()
		else:
			ros_bridge.stop()
	_playing = session.ready_to_play
	_update_mouse_capture()
	_update_hud()
	var role := "HOST" if session.is_host else "CLIENT"
	DisplayServer.window_set_title(
		"Cub Simulator | %s | %s | %s" % [role, session.nickname, world.world_id]
		if session.ready_to_play
		else "Cub Simulator | Lobby"
	)


func _status_changed(message: String) -> void:
	hud.set_status(message)
	_refresh_session()
	print(message)


func _select_camera(mode: int) -> void:
	if mode == 0 and camera_rig.mode == 2 and session.ready_to_play:
		if not _warp_local_actor_to_map_center():
			return
	camera_rig.select_mode(mode)
	_cursor_released = false
	_update_mouse_capture()
	_update_hud()


func _warp_local_actor_to_map_center() -> bool:
	var actor: Node3D = actors.get(session.local_actor_id)
	if not is_instance_valid(actor):
		return false
	var at: Variant = camera_rig.map_center_ground_point()
	if at == null:
		push_warning("Cannot warp: place the map center crosshair over terrain.")
		return false
	var ros_position := LidarScript.godot_to_ros(at)
	var check: Dictionary = world.validate_spawn_ros(ros_position.x, ros_position.y)
	if not bool(check.get("ok", false)):
		push_warning("Cannot warp: " + str(check.get("error", "Invalid destination.")))
		return false
	actor.teleport_to(at + Vector3.UP * 0.1)
	# Publish the local actor's new pose on the next physics tick (and odometry for the host).
	_publish_elapsed = 0.05
	return true


## Moves the primary robot on a /sim/teleport request (map frame; ROS yaw equals Godot rotation.y).
func _teleport_primary_from_ros(ros_x: float, ros_y: float, ros_yaw: float) -> void:
	if not session.is_host or not session.ready_to_play:
		push_warning("Cannot teleport: only a running host owns the primary robot.")
		return
	var actor: Node3D = actors.get(session.local_actor_id)
	if not is_instance_valid(actor):
		push_warning("Cannot teleport: primary robot is not spawned.")
		return
	var check: Dictionary = world.validate_spawn_ros(ros_x, ros_y)
	if not bool(check.get("ok", false)):
		push_warning("Cannot teleport: " + str(check.get("error", "Invalid destination.")))
		return
	var ground_z := float(check.get("ground_z", 0.0))
	actor.rotation.y = ros_yaw
	actor.teleport_to(LidarScript.ros_to_godot(Vector3(ros_x, ros_y, ground_z + 0.1)))
	# Publish the new pose and odometry on the next physics tick.
	_publish_elapsed = 0.05
	print("Teleported primary robot to (%.2f, %.2f, yaw %.2f rad)" % [ros_x, ros_y, ros_yaw])


func _update_hud() -> void:
	if not session or not hud:
		return
	var id: int = session.local_actor_id
	var actor: Node3D = actors.get(id)
	hud.update_readout({
		"active": session.ready_to_play,
		"connecting": session._joining,
		"host": session.is_host,
		"kind": _pending_kind,
		"camera": camera_rig.mode,
		"world_name": world.display_name if world else "",
		"world_id": world.world_id if world else "",
		"position": actor.position if is_instance_valid(actor) else null,
		"yaw": rad_to_deg(actor.rotation.y) if is_instance_valid(actor) else 0.0,
		"ros": (ros_bridge.status if ros_bridge else "ROS OFFLINE") + " | " + joy_input.device_name,
		"fps": Engine.get_frames_per_second(),
		"actors": actors.size(),
		"cursor_free": _cursor_released,
	})


func _register_input() -> void:
	var bindings := {
		"move_forward": [KEY_W, KEY_UP], "move_backward": [KEY_S, KEY_DOWN],
		"move_left": [KEY_A, KEY_LEFT], "move_right": [KEY_D, KEY_RIGHT],
		"move_fast": [KEY_SHIFT],
		"manual_stop": [KEY_SPACE],
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


static func extract_spawn_request(parsed: Dictionary) -> Dictionary:
	var req: Dictionary = {}
	if parsed.has("spawn"):
		req["spawn"] = str(parsed["spawn"]).strip_edges()
	if parsed.has("spawn-x"):
		req["x"] = float(str(parsed["spawn-x"]))
	if parsed.has("spawn-y"):
		req["y"] = float(str(parsed["spawn-y"]))
	if parsed.has("spawn-z"):
		req["z"] = float(str(parsed["spawn-z"]))
	if parsed.has("spawn-yaw-deg"):
		req["yaw_deg"] = float(str(parsed["spawn-yaw-deg"]))
	return req


static func validate_options(parsed: Dictionary) -> String:
	if parsed.has("host") and parsed.has("join"):
		return "Use either --host or --join ADDRESS."
	if parsed.has("host") and not parsed["host"] is bool:
		return "--host is a flag; use --bind ADDRESS to set its address."
	for key in VALUE_OPTION_KEYS:
		if parsed.has(key) and parsed[key] is bool:
			return "--%s requires a value." % key
	if parsed.has("join") and str(parsed["join"]).strip_edges().is_empty():
		return "--join requires a host address."
	if parsed.has("join") and parsed.has("world"):
		return "--world cannot be used with --join; clients follow the host's world."
	if parsed.has("world") and str(parsed["world"]).strip_edges().is_empty():
		return "--world requires a world ID."
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
	if parsed.has("ros-url") and not (str(parsed["ros-url"]).begins_with("ws://") or str(parsed["ros-url"]).begins_with("wss://")):
		return "--ros-url must start with ws:// or wss://."
	if parsed.has("input"):
		return "--input has been removed; controller/keyboard selection and manual/autonomous handover are automatic."
	if parsed.has("joy-device") and (not str(parsed["joy-device"]).is_valid_int() or int(parsed["joy-device"]) < -1):
		return "--joy-device must be -1 (automatic selection) or a non-negative device ID to prefer."

	# Spawn argument validation:
	var has_preset := parsed.has("spawn")
	var has_x := parsed.has("spawn-x")
	var has_y := parsed.has("spawn-y")
	var has_z := parsed.has("spawn-z")
	var has_yaw := parsed.has("spawn-yaw-deg")
	var has_coords := has_x or has_y or has_z or has_yaw

	if has_preset and str(parsed["spawn"]).strip_edges().is_empty():
		return "--spawn requires a preset name."
	if has_preset and has_coords:
		return "Cannot combine --spawn preset with --spawn-x/--spawn-y/--spawn-z/--spawn-yaw-deg."
	if has_x != has_y:
		return "Specify both --spawn-x and --spawn-y together."
	if (has_z or has_yaw) and not (has_x and has_y):
		return "--spawn-z and --spawn-yaw-deg require both --spawn-x and --spawn-y."
	for float_key in ["spawn-x", "spawn-y", "spawn-z", "spawn-yaw-deg"]:
		if parsed.has(float_key):
			var text := str(parsed[float_key]).strip_edges()
			if not text.is_valid_float() or not is_finite(float(text)):
				return "--%s must be a finite number." % float_key
	return ""
