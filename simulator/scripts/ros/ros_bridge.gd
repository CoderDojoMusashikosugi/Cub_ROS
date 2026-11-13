extends Node
## One host owns this connection. Never substitute local input for a missing ROS command.

signal status_changed(message: String)
## Emitted when ROS asks to move the primary robot (map frame, metres / radians).
signal teleport_requested(ros_x: float, ros_y: float, ros_yaw: float)

const TELEPORT_TOPIC := "/sim/teleport"

var url := "ws://127.0.0.1:9090"
var command_timeout := 0.3
var connected := false
var status := "ROS OFFLINE"
var linear := 0.0
var angular := 0.0
var received_commands := 0
var sent_scans := 0
var sent_odoms := 0
var _socket := WebSocketPeer.new()
var _active := false
var _retry_at := 0
var _connect_started := 0
var _received_at := -1
var _announced := false


func start() -> void:
	if _active:
		return
	_active = true
	_retry_at = 0
	_clear_command()


func stop() -> void:
	if connected:
		send_joy(preload("res://scripts/ros/joy_input.gd").neutral(true))
		_socket.poll()
	_socket.close()
	_socket = WebSocketPeer.new()
	_active = false
	connected = false
	_announced = false
	_clear_command()
	_set_status("ROS OFFLINE")


func _process(_delta: float) -> void:
	if not _active:
		return
	var now := Time.get_ticks_msec()
	if _socket.get_ready_state() == WebSocketPeer.STATE_CLOSED:
		connected = false
		_announced = false
		_clear_command()
		if now >= _retry_at:
			_socket = WebSocketPeer.new()
			_socket.max_queued_packets = 16
			_socket.connect_to_url(url)
			_connect_started = now
			_retry_at = now + 1000
			_set_status("ROS CONNECTING")
		return
	_socket.poll()
	if _socket.get_ready_state() == WebSocketPeer.STATE_OPEN:
		if not _announced:
			connected = true
			_announced = true
			_clear_command()
			_send({"op": "advertise", "topic": "/joy", "type": "sensor_msgs/msg/Joy", "queue_size": 1})
			_send({"op": "advertise", "topic": "/odom", "type": "nav_msgs/msg/Odometry", "queue_size": 1})
			_send({"op": "advertise", "topic": "/scan", "type": "sensor_msgs/msg/LaserScan", "queue_size": 1})
			_send({"op": "advertise", "topic": "/proceed_to_next_group", "type": "std_msgs/msg/Empty", "queue_size": 1})
			_send({"op": "subscribe", "topic": "/cmd_vel_atom", "type": "geometry_msgs/msg/Twist", "queue_length": 1})
			_send({"op": "subscribe", "topic": TELEPORT_TOPIC, "type": "geometry_msgs/msg/PoseStamped", "queue_length": 1})
			# Start with all buttons released; normal input polling resumes next tick.
			_send({"op": "publish", "topic": "/joy", "msg":
				preload("res://scripts/ros/joy_input.gd").neutral()})
		while _socket.get_available_packet_count() > 0:
			var packet: Variant = JSON.parse_string(_socket.get_packet().get_string_from_utf8())
			if packet is Dictionary:
				_receive(packet)
		_set_status("ROS OK" if command_fresh() else "ROS WAIT CMD")
	elif _socket.get_ready_state() == WebSocketPeer.STATE_CONNECTING:
		if now - _connect_started > 3000:
			_socket.close()
	else:
		connected = false
		_clear_command()
		_set_status("ROS DISCONNECTED")


func send_joy(message: Dictionary) -> void:
	if connected:
		# Do not queue old control samples behind an obstructed connection.
		if _socket.get_current_outbound_buffered_amount() > 65536:
			return
		_send({"op": "publish", "topic": "/joy", "msg": message})


func send_scan(message: Dictionary) -> void:
	if connected and not message.is_empty():
		if _socket.get_current_outbound_buffered_amount() > 65536:
			return
		_send({"op": "publish", "topic": "/scan", "msg": message})
		sent_scans += 1


func send_odom(message: Dictionary) -> void:
	if connected and not message.is_empty():
		if _socket.get_current_outbound_buffered_amount() > 65536:
			return
		_send({"op": "publish", "topic": "/odom", "msg": message})
		sent_odoms += 1


func send_proceed_to_next_group() -> void:
	if connected:
		_send({"op": "publish", "topic": "/proceed_to_next_group", "msg": {}})


func _send(message: Dictionary) -> void:
	if _socket.send_text(JSON.stringify(message)) != OK:
		_socket.close()
		connected = false
		_clear_command()


func _receive(packet: Dictionary) -> void:
	if packet.get("op") != "publish":
		return
	if packet.get("topic") == TELEPORT_TOPIC:
		_receive_teleport(packet.get("msg"))
		return
	if packet.get("topic") != "/cmd_vel_atom":
		return
	var message: Variant = packet.get("msg")
	if not message is Dictionary or not message.get("linear") is Dictionary or not message.get("angular") is Dictionary:
		return
	var forward: Variant = message.linear.get("x")
	var turn: Variant = message.angular.get("z")
	if not (forward is float or forward is int) or not (turn is float or turn is int):
		return
	if not is_finite(float(forward)) or not is_finite(float(turn)):
		_clear_command()
		return
	linear = clampf(float(forward), -5.0, 5.0)
	angular = clampf(float(turn), -TAU, TAU)
	_received_at = Time.get_ticks_msec()
	received_commands += 1


func _receive_teleport(message: Variant) -> void:
	if not message is Dictionary or not message.get("pose") is Dictionary:
		return
	var pose: Dictionary = message.pose
	if not pose.get("position") is Dictionary or not pose.get("orientation") is Dictionary:
		return
	var values := [
		pose.position.get("x"), pose.position.get("y"),
		pose.orientation.get("x", 0.0), pose.orientation.get("y", 0.0),
		pose.orientation.get("z", 0.0), pose.orientation.get("w", 1.0),
	]
	for value: Variant in values:
		if not (value is float or value is int) or not is_finite(float(value)):
			push_warning("Ignoring %s: non-numeric pose." % TELEPORT_TOPIC)
			return
	var qx := float(values[2])
	var qy := float(values[3])
	var qz := float(values[4])
	var qw := float(values[5])
	var yaw := atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
	# Drop any velocity command issued before the jump.
	_clear_command()
	teleport_requested.emit(float(values[0]), float(values[1]), yaw)


func command_fresh() -> bool:
	return connected and _received_at >= 0 and Time.get_ticks_msec() - _received_at < command_timeout * 1000.0


func command() -> Vector2:
	return Vector2(linear, angular) if command_fresh() else Vector2.ZERO


func _clear_command() -> void:
	linear = 0.0
	angular = 0.0
	_received_at = -1


func _set_status(message: String) -> void:
	if status != message:
		status = message
		status_changed.emit(message)
