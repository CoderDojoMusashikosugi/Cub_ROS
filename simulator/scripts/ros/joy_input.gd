extends RefCounted
## Normalize keyboard, DualSense and Xbox SDL input to the existing Linux DualSense Joy layout.

var source := "keyboard"
var device_id := -1
var preferred_device_id := -1
var deadzone := 0.12
var device_name := "Keyboard"


static func neutral(manual := false) -> Dictionary:
	var axes := [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
	var buttons: Array = []
	buttons.resize(14)
	buttons.fill(0)
	buttons[4] = int(manual)
	# rosbridge stamps a missing header in ROS time for commander liveness checks.
	return {"axes": axes, "buttons": buttons}


func sample() -> Dictionary:
	var controllers := _connected_controllers()
	if controllers.has(preferred_device_id):
		device_id = preferred_device_id
	elif not controllers.has(device_id):
		device_id = int(controllers.keys()[0]) if not controllers.is_empty() else -1
	source = "controller" if device_id >= 0 else "keyboard"
	device_name = str(controllers[device_id]) if device_id >= 0 else "Keyboard"
	if device_id >= 0:
		return _gamepad(device_id)
	# Input is polled independently of camera capture and window focus. Godot/OS
	# key releases become ordinary idle Joy, never a synthetic manual stop.
	var keyboard := Vector2(Input.get_axis("move_left", "move_right"),
		Input.get_axis("move_backward", "move_forward"))
	var held := Input.is_action_pressed("manual_stop")
	for action in ["move_left", "move_right", "move_backward", "move_forward"]:
		held = held or Input.is_action_pressed(action)
	return from_keyboard(keyboard, held)


static func from_keyboard(keyboard: Vector2, held := false) -> Dictionary:
	var message := neutral(held or not keyboard.is_zero_approx())
	message.axes[0] = -keyboard.x
	message.axes[1] = keyboard.y
	return message


func _connected_controllers() -> Dictionary:
	var controllers := {}
	for device in Input.get_connected_joypads():
		if Input.is_joy_known(device):
			controllers[device] = Input.get_joy_name(device)
	return controllers


func _gamepad(device: int) -> Dictionary:
	return from_gamepad([
		Input.get_joy_axis(device, JOY_AXIS_LEFT_X),
		Input.get_joy_axis(device, JOY_AXIS_LEFT_Y),
		Input.get_joy_axis(device, JOY_AXIS_RIGHT_X),
		Input.get_joy_axis(device, JOY_AXIS_RIGHT_Y),
		Input.get_joy_axis(device, JOY_AXIS_TRIGGER_LEFT),
		Input.get_joy_axis(device, JOY_AXIS_TRIGGER_RIGHT)], _buttons(device), deadzone)


func _buttons(device: int) -> Array:
	var result: Array = []
	for button in range(21):
		result.append(int(Input.is_joy_button_pressed(device, button)))
	return result


static func from_gamepad(axes: Array, buttons: Array, zone: float) -> Dictionary:
	var message := neutral(false)
	# Godot stick up/left are negative; ROS forward/left are positive.
	message.axes[0] = -_axis(axes[0], zone)
	message.axes[1] = -_axis(axes[1], zone)
	message.axes[3] = -_axis(axes[2], zone)
	message.axes[4] = -_axis(axes[3], zone)
	message.axes[2] = 1.0 - 2.0 * clampf(axes[4], 0.0, 1.0)
	message.axes[5] = 1.0 - 2.0 * clampf(axes[5], 0.0, 1.0)
	message.axes[6] = float(buttons[JOY_BUTTON_DPAD_LEFT]) - float(buttons[JOY_BUTTON_DPAD_RIGHT])
	message.axes[7] = float(buttons[JOY_BUTTON_DPAD_UP]) - float(buttons[JOY_BUTTON_DPAD_DOWN])
	var mapping := [JOY_BUTTON_A, JOY_BUTTON_B, JOY_BUTTON_Y, JOY_BUTTON_X,
		JOY_BUTTON_LEFT_SHOULDER, JOY_BUTTON_RIGHT_SHOULDER, -1, -1,
		JOY_BUTTON_BACK, JOY_BUTTON_START, JOY_BUTTON_LEFT_STICK,
		JOY_BUTTON_RIGHT_STICK, JOY_BUTTON_GUIDE, JOY_BUTTON_TOUCHPAD]
	for index in range(mapping.size()):
		if mapping[index] >= 0:
			message.buttons[index] = buttons[mapping[index]]
	message.buttons[6] = int(float(axes[4]) > 0.5)
	message.buttons[7] = int(float(axes[5]) > 0.5)
	return message


static func _axis(value: float, zone: float) -> float:
	return signf(value) * maxf(absf(value) - zone, 0.0) / (1.0 - zone)
