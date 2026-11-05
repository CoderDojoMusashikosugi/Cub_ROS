extends SceneTree
## Exercise P through the real input handler and a local rosbridge WebSocket endpoint.

var app: Node3D
var server := TCPServer.new()
var socket: WebSocketPeer
var messages: Array[Dictionary] = []
var failures := PackedStringArray()


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	if server.listen(27352, "127.0.0.1") != OK:
		push_error("Could not start the local rosbridge test endpoint")
		quit(1)
		return
	app = load("res://scenes/main.tscn").instantiate()
	root.add_child(app)
	app.hud._ros_url.text = "ws://127.0.0.1:27352"
	app._host("127.0.0.1", 27353, "ProceedTest")
	await _pump(0.5)
	if DisplayServer.get_name() == "headless":
		root.notify_mouse_entered()
	app._set_window_focus(true)
	_check(app.session.ready_to_play and app.ros_bridge.connected, "host connects to the local rosbridge endpoint")
	var advertised := false
	for message in messages:
		if message.get("op") == "advertise" and message.get("topic") == "/proceed_to_next_group":
			advertised = message.get("type") == "std_msgs/msg/Empty"
	_check(advertised, "bridge advertises the proceed topic as std_msgs/msg/Empty")
	_key(true)
	for repeat in range(3):
		_key(true, true)
	_key(false)
	await _pump(0.15)
	_check(_proceed_count() == 1, "P sends once and ignores key repeats and release")
	for mode in [1, 2]:
		app._select_camera(mode)
		_key(true)
		_key(false)
		await _pump(0.15)
	_check(_proceed_count() == 3, "P also works in both map views")
	app.session.is_host = false
	_key(true)
	app.session.is_host = true
	app._set_window_focus(false)
	_key(true)
	app._set_window_focus(true)
	await _pump(0.15)
	_check(_proceed_count() == 3, "clients and unfocused windows do not send proceed")
	app.ros_bridge.stop()
	_key(true)
	app.ros_bridge.start()
	await _pump(0.5)
	_check(app.ros_bridge.connected and _proceed_count() == 3, "reconnection does not replay P pressed while disconnected")
	_key(true)
	_key(false)
	await _pump(0.15)
	_check(_proceed_count() == 4, "P sends normally after reconnecting")
	app.session.leave()
	_key(true)
	await _pump(0.15)
	_check(_proceed_count() == 4, "lobby does not send proceed")
	var empty_payloads := true
	for message in messages:
		if message.get("op") == "publish" and message.get("topic") == "/proceed_to_next_group":
			empty_payloads = empty_payloads and message.get("msg") == {}
	_check(empty_payloads, "all proceed payloads are empty messages")
	if socket:
		socket.close()
	server.stop()
	print("PROCEED TESTS PASSED" if failures.is_empty() else "PROCEED TESTS FAILED")
	quit(0 if failures.is_empty() else 1)


func _pump(seconds: float) -> void:
	var deadline := Time.get_ticks_msec() + int(seconds * 1000)
	while Time.get_ticks_msec() < deadline:
		if server.is_connection_available():
			socket = WebSocketPeer.new()
			socket.accept_stream(server.take_connection())
		if socket:
			socket.poll()
			while socket.get_available_packet_count() > 0:
				var packet: Variant = JSON.parse_string(socket.get_packet().get_string_from_utf8())
				if packet is Dictionary:
					messages.append(packet)
		await process_frame


func _proceed_count() -> int:
	var count := 0
	for message in messages:
		if message.get("op") == "publish" and message.get("topic") == "/proceed_to_next_group":
			count += 1
	return count


func _key(pressed: bool, repeat := false) -> void:
	var event := InputEventKey.new()
	event.physical_keycode = KEY_P
	event.keycode = KEY_P
	event.pressed = pressed
	event.echo = repeat
	root.push_input(event, true)


func _check(condition: bool, description: String) -> void:
	print("%s: %s" % ["PASS" if condition else "FAIL", description])
	if not condition:
		failures.append(description)
