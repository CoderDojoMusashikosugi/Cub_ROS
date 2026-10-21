extends SceneTree
## Exercise the actual lobby buttons, session transitions and mouse/key input.

var app: Node3D
var failures := PackedStringArray()


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	app = load("res://scenes/main.tscn").instantiate()
	root.add_child(app)
	await _frames(3)
	if DisplayServer.get_name() == "headless":
		root.notify_mouse_entered()
	app._set_window_focus(true)
	_check(app.hud._lobby.is_visible_in_tree() and not app.hud._bar.is_visible_in_tree(),
		"startup shows mouse-operated lobby without gameplay HUD")
	_check(not app.camera_rig.look_enabled and Input.mouse_mode == Input.MOUSE_MODE_VISIBLE,
		"lobby leaves the cursor free")
	app.hud._nickname.grab_focus()
	var initial_kind: String = app._pending_kind
	_key(KEY_T)
	_check(app._pending_kind == initial_kind and not app.session.ready_to_play,
		"gameplay shortcuts do not affect lobby text editing")
	app.hud._nickname.text = "UiTest"
	app.hud._port.value = 27347
	app.hud._kind.select(1)
	await _click(app.hud._host_button)
	_check(app.session.ready_to_play and app.session.nickname == "UiTest", "mouse Host button starts the configured session")
	if not app.session.ready_to_play:
		_finish()
		return
	var primary_robot: Node3D = app.actors[1]
	_check(app.session.local_actor_id == 1 and app.session.selected_kind == "robot" and app.actors.size() == 1,
		"Host always starts with only the primary robot, regardless of the client actor selector")
	_check(not app.hud._lobby.is_visible_in_tree() and app.hud._bar.is_visible_in_tree(),
		"starting play replaces the lobby with the bottom status line")
	_check(_visible_buttons(app.hud).is_empty() and root.gui_get_focus_owner() == null,
		"gameplay has no visible buttons or lingering form focus")
	_check(app.camera_rig.look_enabled, "starting play enables mouse look")
	_check_mouse_mode(Input.MOUSE_MODE_CAPTURED, "starting play captures the cursor")
	var yaw: float = app.camera_rig.yaw
	_motion(Vector2(30, -10))
	_check(not is_equal_approx(app.camera_rig.yaw, yaw), "mouse motion rotates the TPS camera without holding a button")
	_key(KEY_ESCAPE)
	_check(not app.camera_rig.look_enabled, "Escape releases mouse look")
	_check_mouse_mode(Input.MOUSE_MODE_VISIBLE, "Escape frees the cursor for other windows")
	yaw = app.camera_rig.yaw
	_motion(Vector2(30, 10))
	_check(is_equal_approx(app.camera_rig.yaw, yaw), "released cursor movement leaves the camera still")
	await _frames(15)
	var actor: Node3D = app.actors[app.session.local_actor_id]
	var at: Vector3 = actor.position
	Input.action_press("move_forward")
	await _frames(8)
	Input.action_release("move_forward")
	_check(Vector2(actor.position.x, actor.position.z).distance_to(Vector2(at.x, at.z)) < 0.01,
		"released cursor also pauses movement input")
	_key(KEY_TAB)
	_check(app.camera_rig.look_enabled, "Tab resumes mouse look")
	_check_mouse_mode(Input.MOUSE_MODE_CAPTURED, "Tab captures the cursor again")
	app._set_window_focus(false)
	_check(not app.camera_rig.look_enabled, "unfocused windows stop handling mouse look")
	_check_mouse_mode(Input.MOUSE_MODE_VISIBLE, "losing focus releases the cursor")
	app._set_window_focus(true)
	_check(app.camera_rig.look_enabled, "returning focus resumes active gameplay")
	_key(KEY_ESCAPE)
	app._set_window_focus(false)
	app._set_window_focus(true)
	_check(not app.camera_rig.look_enabled, "manual cursor release survives a focus change")
	_key(KEY_TAB)
	await _test_map_controls()
	_key(KEY_1)
	_check_mouse_mode(Input.MOUSE_MODE_CAPTURED, "switching from map to TPS captures the cursor")
	var yaw_after_map: float = app.camera_rig.yaw
	_motion(Vector2(30, 10))
	_check(not is_equal_approx(app.camera_rig.yaw, yaw_after_map), "TPS resumes button-free mouse look after map navigation")
	_key(KEY_T)
	app.session.change_kind("pedestrian")
	app.session.change_kind("cone")
	_check(app.session.selected_kind == "robot" and app._pending_kind == "robot"
		and app.session.local_actor_id == 1 and app.actors.size() == 1 and app.actors[1] == primary_robot
		and app.camera_rig.target == primary_robot,
		"host keeps the same primary robot across camera changes and attempted actor switches")
	app._update_hud()
	actor = app.actors[app.session.local_actor_id]
	_check(app.hud._position.text.contains("X %.2f" % actor.position.x)
		and app.hud._state.text.contains("HOST | Robot | TPS"),
		"status line shows the controlled actor's coordinates and state")
	_check(app.hud._bar.size.y <= 30.0, "gameplay status occupies one slim bottom row")
	_key(KEY_BACKSPACE)
	await _frames(3)
	_check(not app.session.ready_to_play and app.hud._lobby.is_visible_in_tree()
		and not app.hud._bar.is_visible_in_tree(), "leaving play restores the lobby")
	_check(not app.camera_rig.look_enabled and Input.mouse_mode == Input.MOUSE_MODE_VISIBLE,
		"returning to lobby releases the cursor")
	app.hud._port.value = 27348
	await _click(app.hud._join_button)
	_check(app.session._joining and app.hud._lobby.is_visible_in_tree()
		and app.hud._host_button.disabled and app.hud._cancel_button.is_visible_in_tree(),
		"mouse Join button keeps the lobby visible while connecting")
	_check(not app.camera_rig.look_enabled, "connection attempts leave the cursor free")
	await _click(app.hud._cancel_button)
	_check(not app.session._joining and not app.hud._host_button.disabled,
		"mouse Cancel button returns to the editable lobby")
	_finish()


func _test_map_controls() -> void:
	var rig: Node3D = app.camera_rig
	var camera: Camera3D = rig.camera
	for mode in [1, 2]:
		_key(KEY_2 if mode == 1 else KEY_3)
		var label := "3D map" if mode == 1 else "2D map"
		_check_mouse_mode(Input.MOUSE_MODE_VISIBLE, label + " leaves the cursor visible")
		var center: Vector3 = rig.center
		var yaw: float = rig.yaw
		var pitch: float = rig.pitch
		_motion(Vector2(30, 20))
		_check(rig.center.is_equal_approx(center) and is_equal_approx(rig.yaw, yaw)
			and is_equal_approx(rig.pitch, pitch), label + " ignores pointer motion without a button")
		var anchor := Vector3(-3, 0, 2)
		var start := camera.unproject_position(anchor)
		var destination := start + Vector2(60, 35)
		_button(MOUSE_BUTTON_LEFT, true, start)
		_motion(destination - start, destination, MOUSE_BUTTON_MASK_LEFT)
		_button(MOUSE_BUTTON_LEFT, false, destination)
		_check(camera.unproject_position(anchor).distance_to(destination) < 0.2
			and is_equal_approx(rig.yaw, yaw) and is_equal_approx(rig.pitch, pitch),
			label + " left drag keeps picked ground under the pointer without rotating")
		center = rig.center
		_motion(Vector2(30, 20), destination + Vector2(30, 20))
		_check(rig.center.is_equal_approx(center), label + " stops panning when the button is released")
		var zoom_at := camera.unproject_position(anchor)
		var distance: float = rig.distance
		_button(MOUSE_BUTTON_WHEEL_UP, true, zoom_at)
		_check(rig.distance < distance and camera.unproject_position(anchor).distance_to(zoom_at) < 0.2,
			label + " wheel zooms in around the pointer")
		_button(MOUSE_BUTTON_WHEEL_DOWN, true, zoom_at)
		_check(is_equal_approx(rig.distance, distance) and camera.unproject_position(anchor).distance_to(zoom_at) < 0.2,
			label + " wheel zooms out around the pointer")
		_button(MOUSE_BUTTON_LEFT, true, zoom_at, false, true)
		_button(MOUSE_BUTTON_LEFT, false, zoom_at)
		_check(rig.distance < distance and camera.unproject_position(anchor).distance_to(zoom_at) < 0.2,
			label + " double click zooms in on the selected position")
		# Ctrl-drag rotates only in 3D; the 2D map remains north-up.
		start = camera.unproject_position(anchor)
		_button(MOUSE_BUTTON_LEFT, true, start, true)
		_motion(Vector2(40, 25), start + Vector2(40, 25), MOUSE_BUTTON_MASK_LEFT, true)
		_button(MOUSE_BUTTON_LEFT, false, start + Vector2(40, 25), true)
		if mode == 1:
			_check(not is_equal_approx(rig.yaw, yaw) and not is_equal_approx(rig.pitch, pitch),
				"3D map Ctrl-left drag changes heading and tilt")
			var before_middle: float = rig.yaw
			_button(MOUSE_BUTTON_MIDDLE, true, start)
			_motion(Vector2(20, 0), start + Vector2(20, 0), MOUSE_BUTTON_MASK_MIDDLE)
			_button(MOUSE_BUTTON_MIDDLE, false, start + Vector2(20, 0))
			_check(not is_equal_approx(rig.yaw, before_middle), "3D map also supports middle drag for rotation")
		else:
			_check(is_equal_approx(rig.yaw, yaw) and is_equal_approx(rig.pitch, pitch)
				and camera.projection == Camera3D.PROJECTION_ORTHOGONAL
				and camera.global_basis.y.is_equal_approx(Vector3.FORWARD),
				"2D map stays orthographic and north-up even with Ctrl-drag")
		start = camera.unproject_position(anchor)
		_button(MOUSE_BUTTON_LEFT, true, start)
		app._set_window_focus(false)
		app._set_window_focus(true)
		center = rig.center
		_motion(Vector2(30, 20), start + Vector2(30, 20), MOUSE_BUTTON_MASK_LEFT)
		_check(rig.center.is_equal_approx(center), label + " forgets drag state when focus is lost")
		_button(MOUSE_BUTTON_LEFT, false, start + Vector2(30, 20))
		start = camera.unproject_position(anchor)
		_button(MOUSE_BUTTON_LEFT, true, start)
		_key(KEY_ESCAPE)
		_motion(Vector2(30, 20), start + Vector2(30, 20), MOUSE_BUTTON_MASK_LEFT)
		_check(rig.center.is_equal_approx(center) and rig.look_enabled,
			label + " Escape cancels dragging while leaving map navigation enabled")
		_button(MOUSE_BUTTON_LEFT, false, start + Vector2(30, 20))
		Input.action_press("move_forward")
		var actor: Node3D = app.actors[app.session.local_actor_id]
		var actor_start: Vector3 = actor.position
		await _frames(8)
		Input.action_release("move_forward")
		_check(Vector2(actor.position.x, actor.position.z).distance_to(Vector2(actor_start.x, actor_start.z)) > 0.05,
			label + " still accepts character movement with the visible cursor")
		if mode == 1:
			center = rig.center
			_key(KEY_3)
			_check(rig.center.is_equal_approx(center), "switching between map views preserves the viewed location")
	# Check that picking uses the height of actual terrain, including raised surfaces.
	_key(KEY_2)
	rig.center = Vector3(-8, 1.2, -9)
	rig.distance = 8.0
	rig.update(0.0)
	var raised := Vector3(-8, 1.2, -9.5)
	var screen := camera.unproject_position(raised)
	_button(MOUSE_BUTTON_LEFT, true, screen)
	_motion(Vector2(25, 20), screen + Vector2(25, 20), MOUSE_BUTTON_MASK_LEFT)
	_button(MOUSE_BUTTON_LEFT, false, screen + Vector2(25, 20))
	_check(camera.unproject_position(raised).distance_to(screen + Vector2(25, 20)) < 0.2,
		"3D map drags raised terrain without assuming the ground is at zero height")
	screen = camera.unproject_position(raised)
	_button(MOUSE_BUTTON_WHEEL_UP, true, screen)
	_check(camera.unproject_position(raised).distance_to(screen) < 0.2,
		"3D map zoom preserves the pointer position on raised terrain")


func _frames(count: int) -> void:
	for frame in range(count):
		await process_frame


func _click(button: Button) -> void:
	var event := InputEventMouseButton.new()
	event.position = button.get_global_rect().get_center()
	event.button_index = MOUSE_BUTTON_LEFT
	event.pressed = true
	root.push_input(event, true)
	await process_frame
	event = event.duplicate()
	event.pressed = false
	root.push_input(event, true)
	await _frames(3)


func _button(button: MouseButton, pressed: bool, at: Vector2, ctrl := false, double_click := false) -> void:
	var event := InputEventMouseButton.new()
	event.position = at
	event.button_index = button
	event.pressed = pressed
	event.ctrl_pressed = ctrl
	event.double_click = double_click
	root.push_input(event, true)


func _motion(relative: Vector2, at := Vector2(480, 300), buttons: int = 0, ctrl := false) -> void:
	var event := InputEventMouseMotion.new()
	event.position = at
	event.relative = relative
	event.screen_relative = relative
	event.button_mask = buttons
	event.ctrl_pressed = ctrl
	root.push_input(event, true)


func _key(code: Key) -> void:
	var event := InputEventKey.new()
	event.physical_keycode = code
	event.keycode = code
	event.pressed = true
	root.push_input(event, true)


func _visible_buttons(node: Node) -> Array:
	var buttons: Array = []
	if node is BaseButton and node.is_visible_in_tree():
		buttons.append(node)
	for child in node.get_children():
		buttons.append_array(_visible_buttons(child))
	return buttons


func _check_mouse_mode(expected: int, description: String) -> void:
	if DisplayServer.get_name() != "headless":
		_check(Input.mouse_mode == expected, description)


func _check(condition: bool, description: String) -> void:
	print("%s: %s" % ["PASS" if condition else "FAIL", description])
	if not condition:
		failures.append(description)


func _finish() -> void:
	Input.action_release("move_forward")
	app.session.leave()
	if failures.is_empty():
		print("UI TESTS PASSED")
		quit(0)
	else:
		for failure in failures:
			push_error(failure)
		quit(1)
