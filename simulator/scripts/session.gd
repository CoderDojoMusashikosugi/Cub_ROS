extends Node
## Server owns lifecycle; each controller owns its own movement results.

signal actor_added(record: Dictionary)
signal actor_removed(actor_id: int)
signal pose_received(actor_id: int, at: Vector3, yaw: float, sample_time: float)
signal status_changed(message: String)
signal session_reset
signal roster_changed

const MAP_ID := "cub_playground_v1"
const PROTOCOL := 2
const MAX_CLIENTS := 3
const KINDS := ["robot", "pedestrian", "cone"]
const COLORS := ["388ac0", "e99b49", "9472b8", "45a88e", "d87387"]

var records: Dictionary = {}
var is_host := false
var ready_to_play := false
var selected_kind := "robot"
var nickname := "Host"
var received_snapshots := 0
var sent_snapshots := 0
var _registered: Dictionary = {}
var _sequences: Dictionary = {}
var _sequence := 0
var _snapshot_elapsed := 0.0
var _joining := false
var _join_elapsed := 0.0
var _spawn_counter := 0

var local_actor_id: int:
	get:
		if not ready_to_play:
			return 0
		if is_host:
			return 1
		return multiplayer.get_unique_id()


func _ready() -> void:
	multiplayer.connected_to_server.connect(_on_connected)
	multiplayer.connection_failed.connect(_on_connection_failed)
	multiplayer.server_disconnected.connect(_on_server_disconnected)
	multiplayer.peer_disconnected.connect(_on_peer_disconnected)
	# Clients never relay their own RPCs to each other.
	multiplayer.server_relay = false


func start_host(port: int, player_name: String, bind_address := "127.0.0.1") -> Error:
	leave()
	var peer := ENetMultiplayerPeer.new()
	peer.set_bind_ip(bind_address if not bind_address.is_empty() else "127.0.0.1")
	var error := peer.create_server(port, MAX_CLIENTS, 3)
	if error != OK:
		status_changed.emit("Cannot host on UDP %d: %s" % [port, error_string(error)])
		return error
	multiplayer.multiplayer_peer = peer
	is_host = true
	ready_to_play = true
	nickname = player_name.strip_edges().left(24)
	if nickname.is_empty():
		nickname = "Host"
	selected_kind = "robot"
	var robot := _record(1, 1, "robot", nickname, Vector3(0, 0.1, 4), true, 0)
	_add_record(robot)
	status_changed.emit("HOST / %s:%d UDP / waiting for clients" % [bind_address, port])
	roster_changed.emit()
	return OK


func join_host(address: String, port: int, actor_kind: String, player_name: String) -> Error:
	leave()
	var peer := ENetMultiplayerPeer.new()
	var error := peer.create_client(address.strip_edges(), port, 3)
	if error != OK:
		status_changed.emit("Cannot connect: %s" % error_string(error))
		return error
	selected_kind = actor_kind if actor_kind in KINDS else "pedestrian"
	nickname = player_name.strip_edges().left(24)
	if nickname.is_empty():
		nickname = "Guest"
	multiplayer.multiplayer_peer = peer
	_joining = true
	_join_elapsed = 0.0
	status_changed.emit("Connecting to %s:%d ..." % [address, port])
	return OK


func leave() -> void:
	ready_to_play = false
	_joining = false
	is_host = false
	if multiplayer.multiplayer_peer:
		multiplayer.multiplayer_peer.close()
	multiplayer.multiplayer_peer = OfflineMultiplayerPeer.new()
	records.clear()
	_registered.clear()
	_sequences.clear()
	_sequence = 0
	_spawn_counter = 0
	_snapshot_elapsed = 0.0
	received_snapshots = 0
	sent_snapshots = 0
	session_reset.emit()


func _physics_process(delta: float) -> void:
	if _joining:
		_join_elapsed += delta
		if _join_elapsed > 12.0:
			leave()
			status_changed.emit("Connection timed out. Check address, UDP port and host capacity.")
	if not ready_to_play or not is_host:
		return
	_snapshot_elapsed += delta
	if _snapshot_elapsed < 0.05:
		return
	_snapshot_elapsed = fmod(_snapshot_elapsed, 0.05)
	if _registered.is_empty():
		return
	var ids := PackedInt32Array()
	var positions := PackedVector3Array()
	var yaws := PackedFloat32Array()
	var times := PackedFloat64Array()
	for id: int in records:
		var record: Dictionary = records[id]
		ids.append(id)
		positions.append(record["position"])
		yaws.append(record["yaw"])
		times.append(record["sample_time"])
	_receive_positions.rpc(ids, positions, yaws, times)
	sent_snapshots += 1


func publish_pose(id: int, at: Vector3, yaw: float, sample_time: float) -> void:
	if not ready_to_play:
		return
	if is_host:
		if id == 1:
			_accept_pose(id, at, yaw, sample_time)
	elif id == local_actor_id:
		if records.has(id):
			records[id]["position"] = at
			records[id]["yaw"] = yaw
			records[id]["sample_time"] = sample_time
		_sequence += 1
		_submit_pose.rpc_id(1, at, yaw, sample_time, _sequence)


func change_kind(new_kind: String) -> void:
	if not ready_to_play or is_host or new_kind not in KINDS:
		return
	_request_kind.rpc_id(1, new_kind)


func _record(id: int, owner: int, actor_kind: String, player_name: String, at: Vector3, primary: bool, color_index: int) -> Dictionary:
	return {"id": id, "owner": owner, "kind": actor_kind, "nickname": player_name,
		"position": at, "yaw": 0.0, "primary": primary, "color": COLORS[color_index % COLORS.size()],
		"sample_time": 0.0}


func _on_connected() -> void:
	_register.rpc_id(1, PROTOCOL, MAP_ID, nickname, selected_kind)


func _on_connection_failed() -> void:
	leave()
	status_changed.emit("Connection failed. Start the host first.")


func _on_server_disconnected() -> void:
	leave()
	status_changed.emit("Host disconnected. The session has ended.")


func _on_peer_disconnected(peer_id: int) -> void:
	if not is_host or not _registered.has(peer_id):
		return
	_registered.erase(peer_id)
	_sequences.erase(peer_id)
	_remove_record(peer_id)
	# ENet may still be processing other disconnections in the same poll.
	# Wait until those peers have also been removed before broadcasting.
	_notify_actor_removed.call_deferred(peer_id, multiplayer.multiplayer_peer)
	status_changed.emit("HOST / %d client(s) connected" % _registered.size())


func _notify_actor_removed(actor_id: int, peer: MultiplayerPeer) -> void:
	if is_host and ready_to_play and multiplayer.multiplayer_peer == peer:
		_remove_actor.rpc(actor_id)


@rpc("any_peer", "call_remote", "reliable", 0)
func _register(protocol: int, map_id: String, player_name: String, actor_kind: String) -> void:
	if not is_host:
		return
	var sender := multiplayer.get_remote_sender_id()
	if sender <= 1 or _registered.has(sender):
		return
	if protocol != PROTOCOL or map_id != MAP_ID:
		multiplayer.multiplayer_peer.disconnect_peer(sender)
		return
	_spawn_counter += 1
	var at := Vector3(float((_spawn_counter - 1) % 3) * 2.0 - 2.0, 0.1, 6.0)
	var actor_kind_safe := actor_kind if actor_kind in KINDS else "pedestrian"
	var safe_name := player_name.strip_edges().left(24)
	if safe_name.is_empty():
		safe_name = "Guest"
	var record := _record(sender, sender, actor_kind_safe, safe_name, at, false, _spawn_counter)
	_add_record(record)
	_receive_roster.rpc_id(sender, records.values())
	for other: int in _registered:
		_spawn_actor.rpc_id(other, record)
	_registered[sender] = true
	status_changed.emit("HOST / %d client(s) connected" % _registered.size())


@rpc("authority", "call_remote", "reliable", 0)
func _receive_roster(roster: Array) -> void:
	_joining = false
	ready_to_play = true
	for record: Dictionary in roster:
		_add_record(record)
	status_changed.emit("CLIENT / connected / %d actors" % records.size())
	roster_changed.emit()


@rpc("authority", "call_remote", "reliable", 0)
func _spawn_actor(record: Dictionary) -> void:
	_add_record(record)
	if int(record["id"]) == local_actor_id:
		selected_kind = str(record["kind"])
	roster_changed.emit()


@rpc("authority", "call_remote", "reliable", 0)
func _remove_actor(id: int) -> void:
	_remove_record(id)


@rpc("any_peer", "call_remote", "unreliable_ordered", 1)
func _submit_pose(at: Vector3, yaw: float, sample_time: float, sequence: int) -> void:
	if not is_host:
		return
	var sender := multiplayer.get_remote_sender_id()
	if not _registered.has(sender) or sequence <= int(_sequences.get(sender, -1)):
		return
	if _accept_pose(sender, at, yaw, sample_time):
		_sequences[sender] = sequence


func _accept_pose(id: int, at: Vector3, yaw: float, sample_time: float) -> bool:
	if not records.has(id) or not at.is_finite() or not is_finite(yaw) or at.length_squared() > 1000000.0:
		return false
	if not is_finite(sample_time) or sample_time <= float(records[id]["sample_time"]):
		return false
	records[id]["position"] = at
	records[id]["yaw"] = yaw
	records[id]["sample_time"] = sample_time
	pose_received.emit(id, at, yaw, sample_time)
	return true


@rpc("authority", "call_remote", "unreliable_ordered", 2)
func _receive_positions(ids: PackedInt32Array, positions: PackedVector3Array, yaws: PackedFloat32Array, times: PackedFloat64Array) -> void:
	if ids.size() != positions.size() or ids.size() != yaws.size() or ids.size() != times.size():
		return
	received_snapshots += 1
	for index in range(ids.size()):
		var id := ids[index]
		if id == local_actor_id:
			continue
		_accept_pose(id, positions[index], yaws[index], times[index])


@rpc("any_peer", "call_remote", "reliable", 0)
func _request_kind(new_kind: String) -> void:
	if not is_host or new_kind not in KINDS:
		return
	var sender := multiplayer.get_remote_sender_id()
	if not _registered.has(sender):
		return
	var record: Dictionary = records[sender].duplicate()
	record["kind"] = new_kind
	_add_record(record)
	_spawn_actor.rpc(record)


func _add_record(record: Dictionary) -> void:
	records[int(record["id"])] = record
	actor_added.emit(record)
	roster_changed.emit()


func _remove_record(id: int) -> void:
	if not records.has(id):
		return
	records.erase(id)
	actor_removed.emit(id)
	roster_changed.emit()
