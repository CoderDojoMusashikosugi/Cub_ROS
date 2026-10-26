extends SceneTree

const JoyInput = preload("res://scripts/ros/joy_input.gd")
var app: Node3D
var failures := PackedStringArray()
var nav_timer: Timer


# Simulate connection snapshots and SDL samples without requiring physical pads.
class TestInput:
	extends "res://scripts/ros/joy_input.gd"
	var controllers := {}
	var pad := neutral()

	func _connected_controllers() -> Dictionary:
		return controllers

	func _gamepad(_device: int) -> Dictionary:
		return pad.duplicate(true)


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	_test_mapping()
	app = load("res://scenes/main.tscn").instantiate()
	root.add_child(app)
	_test_selection()
	app.joy_input = TestInput.new()
	if not app.options.has("integration"):
		app.hud._ros_url.text = "ws://127.0.0.1:1"
	app._host("127.0.0.1", 27351, "RosControlTest")
	if not app.session.ready_to_play:
		push_error("ROS control test could not start its local host")
		quit(1)
		return
	app._has_focus = true
	await create_timer(0.3).timeout
	var actor: Node3D = app.actors[1]
	app.ros_bridge.stop()
	var start: Vector3 = actor.position
	Input.action_press("move_forward")
	await create_timer(0.5).timeout
	Input.action_release("move_forward")
	_check(_horizontal(actor.position - start) < 0.01, "host WASD never directly moves Cub4 without ROS")
	# Also guard the old actor API against accidental direct control of Cub4.
	actor.simulate(Vector2(1, 1), 0.0, true, 0.1)
	_check(_horizontal(actor.position - start) < 0.01, "primary robot rejects the direct-input movement API")
	if app.options.has("integration"):
		app.ros_bridge.start()
		var deadline := Time.get_ticks_msec() + 10000
		while not app.ros_bridge.command_fresh() and Time.get_ticks_msec() < deadline:
			await process_frame
		_check(app.ros_bridge.command_fresh(), "real rosbridge and cub_commander return velocity commands")
		if app.ros_bridge.command_fresh():
			start = actor.position
			Input.action_press("move_forward")
			await create_timer(0.8).timeout
			Input.action_release("move_forward")
			_check(_horizontal(actor.position - start) > 0.5, "W travels through Joy and cub_commander to move Cub4")
			await create_timer(3.3).timeout
			start = actor.position
			await create_timer(0.3).timeout
			_check(_horizontal(actor.position - start) < 0.01, "without autonomous input, releasing W eventually stops under the original 3-second watchdog")
			var yaw: float = actor.rotation.y
			Input.action_press("move_left")
			await create_timer(0.6).timeout
			Input.action_release("move_left")
			_check(actor.rotation.y > yaw + 0.3, "A produces positive ROS angular.z and turns Cub4 left")
			Input.action_press("move_forward")
			await create_timer(0.3).timeout
			app._set_window_focus(false)
			start = actor.position
			await create_timer(0.4).timeout
			_check(_horizontal(actor.position - start) > 0.2, "focus flag does not replace held keyboard input with a manual stop")
			Input.action_release("move_forward")
			await create_timer(0.3).timeout
			_check(is_zero_approx(app.ros_bridge.command().x), "keyboard release while unfocused stops when no autonomous command exists")
			app._cursor_released = true
			Input.action_press("move_forward")
			await create_timer(0.3).timeout
			_check(is_equal_approx(app.ros_bridge.command().x, 1.2), "cursor release does not suppress host Joy")
			Input.action_release("move_forward")
			app._cursor_released = false
			app._set_window_focus(true)
			app.ros_bridge.stop()
			start = actor.position
			Input.action_press("move_forward")
			await create_timer(0.5).timeout
			Input.action_release("move_forward")
			_check(_horizontal(actor.position - start) < 0.01, "bridge disconnection stops without falling back to WASD")
			app.ros_bridge.start()
			await create_timer(1.5).timeout
			_check(app.ros_bridge.command_fresh() and _horizontal(actor.position - start) < 0.01,
				"reconnect receives fresh commands without replaying held input")
			await _test_handover()

	# Stale and malformed commands must not be accepted as a fresh motion sample.
	app.ros_bridge.stop()
	app.ros_bridge.connected = true
	app.ros_bridge._receive({"op": "publish", "topic": "/cmd_vel_atom", "msg":
		{"linear": {"x": 1.0}, "angular": {"z": 0.0}}})
	_check(app.ros_bridge.command().x == 1.0, "Twist decoder accepts a valid forward command")
	app.ros_bridge._received_at -= 1000
	_check(app.ros_bridge.command().is_zero_approx(), "command timeout returns zero velocity")
	app.ros_bridge._receive({"op": "publish", "topic": "/cmd_vel_atom", "msg":
		{"linear": {"x": "invalid"}, "angular": {"z": 0.0}}})
	_check(app.ros_bridge.command().is_zero_approx(), "malformed Twist cannot refresh a timed-out command")
	app.ros_bridge.connected = false
	app.session.leave()
	print("ROS CONTROL TESTS PASSED" if failures.is_empty() else "ROS CONTROL TESTS FAILED")
	quit(0 if failures.is_empty() else 1)


func _test_mapping() -> void:
	var message := JoyInput.from_keyboard(Vector2(-1, 1))
	_check(message.axes.size() == 8 and message.buttons.size() == 14 and message.buttons[4] == 1
		and message.axes[0] == 1.0 and message.axes[1] == 1.0, "W+A becomes independent DualSense stick axes with virtual L1")
	message = JoyInput.from_keyboard(Vector2.ZERO)
	_check(message.buttons[4] == 0, "idle keyboard releases manual ownership")
	message = JoyInput.from_keyboard(Vector2.ZERO, true)
	_check(message.buttons[4] == 1 and message.axes[1] == 0.0, "Space alone sends a manual zero command")
	message = JoyInput.from_keyboard(Vector2(0, 1), true)
	_check(message.buttons[4] == 1 and message.axes[1] == 1.0, "WASD takes priority over Space without cancelling movement")
	var buttons: Array = []
	buttons.resize(21)
	buttons.fill(0)
	buttons[JOY_BUTTON_LEFT_SHOULDER] = 1
	buttons[JOY_BUTTON_X] = 1
	buttons[JOY_BUTTON_DPAD_UP] = 1
	message = JoyInput.from_gamepad([-0.6, -1.0, 0.0, 0.0, 0.0, 1.0], buttons, 0.1)
	_check(message.axes[0] > 0.5 and message.axes[1] == 1.0 and message.buttons[4] == 1
		and message.buttons[3] == 1 and message.buttons[7] == 1 and message.axes[7] == 1.0
		and message.axes[2] == 1.0 and message.axes[5] == -1.0,
		"SDL DualSense/Xbox axes, face buttons, LB/L1 and triggers map to Linux DualSense Joy")
	message = JoyInput.from_gamepad([0.03, -0.02, 0.0, 0.0, 0.0, 0.0], buttons, 0.1)
	_check(message.axes[0] == 0.0 and message.axes[1] == 0.0, "controller drift is removed by the deadzone")
	buttons.fill(0)
	message = JoyInput.from_gamepad([0.0, -1.0, 0.0, 0.0, 0.0, 0.0], buttons, 0.1)
	_check(message.buttons[4] == 0 and message.buttons[6] == 0 and message.axes[1] == 1.0,
		"stick input without L1/L2 leaves autonomous arbitration to commander")


func _test_selection() -> void:
	var mapper := TestInput.new()
	_check(mapper.sample().buttons[4] == 0 and mapper.source == "keyboard", "no controller automatically selects idle keyboard")
	Input.action_press("move_forward")
	_check(mapper.sample().axes[1] == 1.0, "keyboard selection polls actual Godot actions")
	Input.action_press("move_backward")
	var message := mapper.sample()
	_check(message.buttons[4] == 1 and message.axes[1] == 0.0, "opposing held keys remain a manual zero command")
	Input.action_release("move_backward")
	mapper.controllers = {3: "DualSense"}
	message = mapper.sample()
	_check(mapper.source == "controller" and mapper.device_name == "DualSense" and message.buttons[4] == 0
		and message.axes[1] == 0.0, "hotplug automatically selects the controller and ignores held keyboard input")
	mapper.pad = JoyInput.from_keyboard(Vector2(0, -0.5))
	_check(mapper.sample().axes[1] == -0.5, "selected controller input is forwarded")
	mapper.controllers[4] = "Xbox"
	mapper.sample()
	_check(mapper.device_id == 3, "connecting another controller keeps the current selection")
	mapper.preferred_device_id = 4
	mapper.sample()
	_check(mapper.device_id == 4 and mapper.device_name == "Xbox", "explicit preferred ID selects that connected controller")
	mapper.controllers.erase(4)
	mapper.sample()
	_check(mapper.device_id == 3, "missing preferred controller falls back to another known pad")
	mapper.controllers.clear()
	message = mapper.sample()
	_check(mapper.source == "keyboard" and mapper.device_name == "Keyboard" and message.axes[1] == 1.0,
		"unplug immediately falls back to current WASD rather than retaining stick input")
	Input.action_release("move_forward")
	_check(mapper.sample().buttons[4] == 0, "idle keyboard after unplug permits autonomous operation")
	mapper.controllers = {8: "Xbox reconnected"}
	mapper.pad = JoyInput.neutral()
	_check(mapper.sample().buttons[4] == 0 and mapper.device_id == 8, "reconnect with a new ID selects fresh released input")
	Input.action_press("manual_stop")
	mapper.controllers.clear()
	message = mapper.sample()
	_check(message.buttons[4] == 1 and message.axes[1] == 0.0, "Space action is sampled as a manual stop")
	Input.action_press("move_forward")
	_check(mapper.sample().axes[1] == 1.0, "Space held with W still sends forward input")
	Input.action_release("move_forward")
	Input.action_release("manual_stop")


func _test_handover() -> void:
	app.ros_bridge._send({"op": "advertise", "topic": "/cmd_vel", "type": "geometry_msgs/msg/Twist"})
	nav_timer = Timer.new()
	nav_timer.wait_time = 0.05
	nav_timer.timeout.connect(_publish_nav)
	root.add_child(nav_timer)
	nav_timer.start()
	await create_timer(0.7).timeout
	_check(is_equal_approx(app.ros_bridge.command().x, 0.4), "idle keyboard continuously relays Joy while autonomous cmd_vel controls Cub4")
	Input.action_press("move_forward")
	await create_timer(0.3).timeout
	_check(is_equal_approx(app.ros_bridge.command().x, 1.2), "W overrides a continuously published autonomous command")
	Input.action_release("move_forward")
	Input.action_press("manual_stop")
	await create_timer(0.3).timeout
	_check(is_zero_approx(app.ros_bridge.command().x), "Space overrides autonomous movement with a manual zero")
	Input.action_press("move_forward")
	await create_timer(0.3).timeout
	_check(is_equal_approx(app.ros_bridge.command().x, 1.2), "W overrides Space through the real commander")
	Input.action_release("move_forward")
	await create_timer(0.3).timeout
	_check(is_zero_approx(app.ros_bridge.command().x), "releasing W while Space remains held returns to manual zero")
	Input.action_release("manual_stop")
	app._set_window_focus(false)
	await create_timer(0.3).timeout
	_check(is_equal_approx(app.ros_bridge.command().x, 0.4), "releasing all keys restores autonomy even when unfocused")
	app.joy_input.controllers = {3: "DualSense"}
	Input.action_press("move_forward")
	await create_timer(0.3).timeout
	_check(is_equal_approx(app.ros_bridge.command().x, 0.4), "connected idle controller takes priority over keyboard and permits autonomy")
	Input.action_release("move_forward")
	app.joy_input.pad = JoyInput.neutral(true)
	await create_timer(0.3).timeout
	_check(is_zero_approx(app.ros_bridge.command().x), "unfocused controller L1 with a neutral stick sends manual zero")
	var buttons: Array = []
	buttons.resize(21)
	buttons.fill(0)
	buttons[JOY_BUTTON_LEFT_SHOULDER] = 1
	app.joy_input.pad = JoyInput.from_gamepad([0.0, -1.0, 0.0, 0.0, 0.0, 0.0], buttons, 0.12)
	await create_timer(0.3).timeout
	_check(is_equal_approx(app.ros_bridge.command().x, 1.2), "unfocused controller forwards SDL manual stick input through commander")
	app.joy_input.pad.buttons[4] = 0
	await create_timer(0.3).timeout
	_check(is_equal_approx(app.ros_bridge.command().x, 0.4), "releasing controller L1 resumes autonomy despite a held stick")
	buttons[JOY_BUTTON_LEFT_SHOULDER] = 0
	app.joy_input.pad = JoyInput.from_gamepad([0.0, -1.0, 0.0, 0.0, 1.0, 0.0], buttons, 0.12)
	await create_timer(0.3).timeout
	_check(is_equal_approx(app.ros_bridge.command().x, 1.2), "controller L2/LT also enables manual operation while unfocused")
	app.joy_input.controllers.clear()
	await create_timer(0.3).timeout
	_check(is_equal_approx(app.ros_bridge.command().x, 0.4), "controller unplug with idle keyboard preserves autonomous movement")
	Input.action_press("move_forward")
	await create_timer(0.3).timeout
	_check(is_equal_approx(app.ros_bridge.command().x, 1.2), "WASD takes over after controller unplug while unfocused")
	Input.action_release("move_forward")
	nav_timer.stop()
	nav_timer.queue_free()
	await create_timer(3.3).timeout
	_check(is_zero_approx(app.ros_bridge.command().x), "original 3-second autonomous command timeout stops Cub4")
	app._set_window_focus(true)


func _publish_nav() -> void:
	app.ros_bridge._send({"op": "publish", "topic": "/cmd_vel", "msg":
		{"linear": {"x": 0.4, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}})


func _horizontal(value: Vector3) -> float:
	return Vector2(value.x, value.z).length()


func _check(condition: bool, description: String) -> void:
	print("PASS: " if condition else "FAIL: ", description)
	if not condition:
		failures.append(description)
