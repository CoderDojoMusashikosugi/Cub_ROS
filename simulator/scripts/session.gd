extends Node
## Server owns lifecycle and world agreement; each controller owns its own movement results.

signal actor_added(record: Dictionary)
signal actor_removed(actor_id: int)
signal pose_received(actor_id: int, at: Vector3, yaw: float, sample_time: float)
signal status_changed(message: String)
signal session_reset
signal roster_changed
signal world_changed(world_info: Dictionary)

const MAP_ID := "cub_playground_v1"
const PROTOCOL := 3
const MAX_CLIENTS := 3
const KINDS := ["robot", "pedestrian", "cone"]
const COLORS := ["388ac0", "e99b49", "9472b8", "45a88e", "d87387"]
const CONNECT_TIMEOUT_SEC := 12.0
const WORLD_LOAD_TIMEOUT_SEC := 30.0
const WorldRegistryScript = preload("res://scripts/world_registry.gd")

var world_node: Node
var world_info: Dictionary = {
	"world_id": MAP_ID,
	"display_name": "Cub Playground",
	"schema_version": 1,
	"kind": "playground",
	"world_hash": "",
}
var local_spawn_request: Dictionary = {}
var records: Dictionary = {}
var is_host := false
var ready_to_play := false
var selected_kind := "robot"
var nickname := "Host"
var received_snapshots := 0
var sent_snapshots := 0
var _registered: Dictionary = {}
var _pending_peers: Dictionary = {}
var _sequences: Dictionary = {}
var _sequence := 0
var _snapshot_elapsed := 0.0
var _joining := false
var _loading_world := false
var _join_elapsed := 0.0
var _load_elapsed := 0.0
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


func start_host(port: int, player_name: String, bind_address := "127.0.0.1", spawn_override: Dictionary = {}) -> Error:
	leave()
	var spawn_pos := Vector3(0.0, 0.1, 4.0)
	var spawn_yaw := 0.0
	if world_node != null:
		if not bool(world_node.get("is_loaded")):
			var err_msg := str(world_node.get("load_error"))
			status_changed.emit("Cannot host: %s" % (err_msg if not err_msg.is_empty() else "world is not loaded."))
			return ERR_CANT_CREATE
		var req := spawn_override if not spawn_override.is_empty() else local_spawn_request
		var resolved: Dictionary = world_node.resolve_spawn(req, true, 0)
		if not bool(resolved.get("ok", false)):
			status_changed.emit("Spawn error: %s" % str(resolved.get("error", "Invalid spawn position.")))
			return ERR_INVALID_PARAMETER
		spawn_pos = resolved["godot_pos"]
		spawn_yaw = float(resolved["godot_yaw"])
		world_info = world_node.world_summary()

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
	var robot := _record(1, 1, "robot", nickname, spawn_pos, true, 0, spawn_yaw)
	_add_record(robot)
	status_changed.emit("HOST / %s / %s:%d UDP / waiting for clients" % [
		str(world_info.get("world_id", MAP_ID)), bind_address, port
	])
	roster_changed.emit()
	return OK


func join_host(address: String, port: int, actor_kind: String, player_name: String, spawn_override: Dictionary = {}) -> Error:
	leave()
	if not spawn_override.is_empty():
		local_spawn_request = spawn_override.duplicate(true)
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
	_loading_world = false
	_join_elapsed = 0.0
	_load_elapsed = 0.0
	status_changed.emit("Connecting to %s:%d ..." % [address, port])
	return OK


func leave() -> void:
	ready_to_play = false
	_joining = false
	_loading_world = false
	is_host = false
	if multiplayer.multiplayer_peer:
		multiplayer.multiplayer_peer.close()
	multiplayer.multiplayer_peer = OfflineMultiplayerPeer.new()
	records.clear()
	_registered.clear()
	_pending_peers.clear()
	_sequences.clear()
	_sequence = 0
	_spawn_counter = 0
	_snapshot_elapsed = 0.0
	_join_elapsed = 0.0
	_load_elapsed = 0.0
	received_snapshots = 0
	sent_snapshots = 0
	session_reset.emit()


func _physics_process(delta: float) -> void:
	if _joining:
		if _loading_world:
			_load_elapsed += delta
			if _load_elapsed > WORLD_LOAD_TIMEOUT_SEC:
				leave()
				status_changed.emit("World loading timed out while joining host.")
		else:
			_join_elapsed += delta
			if _join_elapsed > CONNECT_TIMEOUT_SEC:
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


func _record(id: int, owner: int, actor_kind: String, player_name: String, at: Vector3, primary: bool, color_index: int, yaw := 0.0) -> Dictionary:
	return {
		"id": id,
		"owner": owner,
		"kind": actor_kind,
		"nickname": player_name,
		"position": at,
		"yaw": yaw,
		"spawn_position": at,
		"spawn_yaw": yaw,
		"primary": primary,
		"color": COLORS[color_index % COLORS.size()],
		"sample_time": 0.0,
	}


func _on_connected() -> void:
	_hello.rpc_id(1, PROTOCOL, nickname, selected_kind, local_spawn_request)


func _on_connection_failed() -> void:
	leave()
	status_changed.emit("Connection failed. Start the host first.")


func _on_server_disconnected() -> void:
	leave()
	status_changed.emit("Host disconnected. The session has ended.")


func _on_peer_disconnected(peer_id: int) -> void:
	_pending_peers.erase(peer_id)
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


func _reject_peer(peer_id: int, reason: String) -> void:
	_join_rejected.rpc_id(peer_id, reason)
	var current_peer := multiplayer.multiplayer_peer
	get_tree().create_timer(0.1).timeout.connect(func():
		if (
			is_host
			and multiplayer.multiplayer_peer == current_peer
			and current_peer != null
			and peer_id in multiplayer.get_peers()
		):
			current_peer.disconnect_peer(peer_id)
	)


@rpc("any_peer", "call_remote", "reliable", 0)
func _hello(protocol: int, player_name: String, actor_kind: String, spawn_request: Dictionary = {}) -> void:
	if not is_host or not ready_to_play:
		return
	var sender := multiplayer.get_remote_sender_id()
	if sender <= 1 or _registered.has(sender) or _pending_peers.has(sender):
		return
	if protocol != PROTOCOL:
		_reject_peer(sender, "Protocol mismatch (host=%d, client=%d)." % [PROTOCOL, protocol])
		return
	if world_node != null and not bool(world_node.get("is_loaded")):
		_reject_peer(sender, "Host world is not ready yet.")
		return
	var actor_kind_safe := actor_kind if actor_kind in KINDS else "pedestrian"
	var safe_name := player_name.strip_edges().left(24)
	if safe_name.is_empty():
		safe_name = "Guest"
	_pending_peers[sender] = {
		"nickname": safe_name,
		"kind": actor_kind_safe,
		"spawn_request": spawn_request if spawn_request is Dictionary else {},
	}
	if world_node != null:
		world_info = world_node.world_summary()
	_world_offer.rpc_id(sender, world_info)


@rpc("authority", "call_remote", "reliable", 0)
func _world_offer(offer: Dictionary) -> void:
	if is_host or not _joining:
		return
	var w_name := str(offer.get("display_name", offer.get("world_id", "world")))
	var w_id := str(offer.get("world_id", ""))
	var w_hash := str(offer.get("world_hash", ""))
	var w_schema := int(offer.get("schema_version", -1))
	if (
		world_node != null
		and w_schema == WorldRegistryScript.SUPPORTED_SCHEMA_VERSION
		and not w_id.is_empty()
		and not w_hash.is_empty()
		and bool(world_node.get("is_loaded"))
		and str(world_node.get("world_id")) == w_id
		and str(world_node.get("world_hash")) == w_hash
	):
		world_info = world_node.world_summary()
		world_changed.emit(world_info)
		_world_ready.rpc_id(1, w_id, w_hash)
		return
	_loading_world = true
	_load_elapsed = 0.0
	status_changed.emit("Loading world '%s' (%s) ..." % [w_name, w_id])
	_prepare_client_world.call_deferred(offer, multiplayer.multiplayer_peer)


func _prepare_client_world(offer: Dictionary, expected_peer: MultiplayerPeer) -> void:
	if not _joining or multiplayer.multiplayer_peer != expected_peer:
		return
	var worlds_root := WorldRegistryScript.WORLDS_ROOT
	if world_node != null and world_node.definition.has("base_dir"):
		var cur_base := str(world_node.definition.get("base_dir", ""))
		if not cur_base.is_empty():
			worlds_root = cur_base.get_base_dir()
	var verified := WorldRegistryScript.verify_remote_offer(offer, worlds_root)
	if not bool(verified.get("ok", false)):
		leave()
		status_changed.emit(str(verified.get("error", "World verification failed.")))
		return
	if world_node != null:
		var need_load := (
			not bool(world_node.get("is_loaded"))
			or str(world_node.get("world_id")) != str(verified.get("world_id", ""))
			or str(world_node.get("world_hash")) != str(verified.get("world_hash", ""))
		)
		if need_load:
			var loaded: bool = world_node.load_from_definition(verified)
			if not loaded:
				var load_err := str(world_node.get("load_error"))
				leave()
				status_changed.emit("Failed to load world '%s' (%s): %s" % [
					str(offer.get("display_name", "")), str(offer.get("world_id", "")), load_err
				])
				return
		world_info = world_node.world_summary()
	else:
		world_info = offer.duplicate(true)

	world_changed.emit(world_info)
	# Ensure physics server registers newly added StaticBody3D shapes before spawning actors.
	await get_tree().physics_frame
	if not _joining or multiplayer.multiplayer_peer != expected_peer:
		return
	_world_ready.rpc_id(1, str(world_info.get("world_id", "")), str(world_info.get("world_hash", "")))


@rpc("any_peer", "call_remote", "reliable", 0)
func _world_ready(client_world_id: String, client_world_hash: String) -> void:
	if not is_host or not ready_to_play:
		return
	var sender := multiplayer.get_remote_sender_id()
	if sender <= 1 or _registered.has(sender) or not _pending_peers.has(sender):
		return
	var peer_info: Dictionary = _pending_peers[sender]
	_pending_peers.erase(sender)
	var expected_id := str(world_info.get("world_id", ""))
	var expected_hash := str(world_info.get("world_hash", ""))
	if client_world_id != expected_id or (not expected_hash.is_empty() and client_world_hash != expected_hash):
		_reject_peer(
			sender,
			"World mismatch: host requires '%s' (%s, hash %s)." % [
				str(world_info.get("display_name", expected_id)),
				expected_id,
				expected_hash.left(12),
			]
		)
		return

	var client_slot := _spawn_counter
	var at := Vector3(float(client_slot % 3) * 2.0 - 2.0, 0.1, 6.0)
	var yaw := 0.0
	if world_node != null:
		var spawn_req: Dictionary = peer_info.get("spawn_request", {})
		var resolved: Dictionary = world_node.resolve_spawn(spawn_req, false, client_slot)
		if not bool(resolved.get("ok", false)):
			_reject_peer(sender, "Spawn error: %s" % str(resolved.get("error", "Invalid client spawn position.")))
			return
		at = resolved["godot_pos"]
		yaw = float(resolved["godot_yaw"])

	_spawn_counter += 1
	var record := _record(
		sender,
		sender,
		str(peer_info.get("kind", "pedestrian")),
		str(peer_info.get("nickname", "Guest")),
		at,
		false,
		_spawn_counter,
		yaw
	)
	_add_record(record)
	_receive_roster.rpc_id(sender, records.values())
	for other: int in _registered:
		_spawn_actor.rpc_id(other, record)
	_registered[sender] = true
	status_changed.emit("HOST / %s / %d client(s) connected" % [expected_id, _registered.size()])


@rpc("authority", "call_remote", "reliable", 0)
func _join_rejected(reason: String) -> void:
	leave()
	status_changed.emit(reason)


@rpc("authority", "call_remote", "reliable", 0)
func _receive_roster(roster: Array) -> void:
	_joining = false
	_loading_world = false
	ready_to_play = true
	for record: Dictionary in roster:
		_add_record(record)
	status_changed.emit("CLIENT / %s / connected / %d actors" % [
		str(world_info.get("world_id", MAP_ID)), records.size()
	])
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
	if not records.has(id) or not at.is_finite() or not is_finite(yaw) or at.length_squared() > 100000000.0:
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
