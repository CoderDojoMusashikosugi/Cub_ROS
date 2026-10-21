extends SceneTree

const ActorScript = preload("res://scripts/actor.gd")
const WorldScript = preload("res://scripts/world.gd")
var world: Node3D
var failures := PackedStringArray()


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	world = WorldScript.new()
	root.add_child(world)
	var actor := _actor(Vector3(-8, 0.1, 1))
	await _walk(actor, Vector2.ZERO, 12)
	await _walk(actor, Vector2(0, 1), 192)
	_check(actor.position.y > 1.15, "ascend mesh ramp", actor.position)
	_check(absf(actor.rotation.x) < 0.001 and absf(actor.rotation.z) < 0.001, "remain upright on ramp", actor.rotation)
	await _walk(actor, Vector2(0, -1), 192)
	_check(actor.position.y < 0.08, "descend ramp and return to floor", actor.position)
	actor.position = Vector3(6, 0.1, 1)
	actor.velocity = Vector3.ZERO
	await _walk(actor, Vector2.ZERO, 12)
	await _walk(actor, Vector2(0, 1), 132)
	_check(actor.position.z < -4.9 and actor.position.y > 1.15, "climb real 15 cm staircase", actor.position)
	await _walk(actor, Vector2(0, -1), 132)
	_check(actor.position.z > 0.5 and actor.position.y < 0.08, "descend staircase", actor.position)
	actor.position = Vector3(-2, 0.1, -6.5)
	actor.velocity = Vector3.ZERO
	await _walk(actor, Vector2.ZERO, 12)
	await _walk(actor, Vector2(0, 1), 138)
	_check(actor.position.z < -13.0 and actor.position.y > 0.03, "cross uneven triangle mesh", actor.position)
	actor.position = Vector3(11, 0.1, -4.5)
	actor.velocity = Vector3.ZERO
	await _walk(actor, Vector2.ZERO, 12)
	await _walk(actor, Vector2(0, 1), 100)
	_check(actor.position.z > -5.85 and actor.position.y < 0.1, "do not climb a one-meter obstacle", actor.position)
	world.box(Vector3(2, 0.15, 1), Vector3(18, 0.075, -0.5), Color.GRAY)
	world.box(Vector3(2, 0.1, 2), Vector3(18, 1.8, -0.5), Color.GRAY)
	actor.position = Vector3(18, 0.1, 0.8)
	actor.velocity = Vector3.ZERO
	await _walk(actor, Vector2.ZERO, 16)
	await _walk(actor, Vector2(0, 1), 60)
	_check(actor.position.z > 0.1 and actor.position.y < 0.1, "step-up respects overhead clearance", actor.position)
	actor.position = Vector3(0, 0.1, 12)
	actor.velocity = Vector3.ZERO
	var other := _actor(actor.position)
	for frame in range(20):
		await physics_frame
		actor.simulate(Vector2.ZERO, 0.0, false, 1.0 / 60.0)
		other.simulate(Vector2.ZERO, 0.0, false, 1.0 / 60.0)
	_check(actor.position.distance_to(other.position) < 0.01, "characters may overlap without pushing", actor.position)
	var robot := _actor(Vector3(6, 0.1, 1), "robot")
	await _walk(robot, Vector2.ZERO, 12)
	await _walk(robot, Vector2(0, 1), 160)
	_check(robot.position.z < -4.9 and robot.position.y > 1.15, "robot also climbs the real staircase", robot.position)
	if failures.is_empty():
		print("TERRAIN TESTS PASSED")
		quit(0)
	else:
		for failure in failures:
			push_error(failure)
		quit(1)


func _actor(at: Vector3, kind := "pedestrian") -> Node3D:
	var actor := ActorScript.new()
	actor.configure({"id": 1, "owner": 1, "kind": kind, "nickname": "Test",
		"primary": false, "color": "388ac0", "position": at, "yaw": 0.0}, true)
	world.add_child(actor)
	return actor


func _walk(actor: Node3D, movement: Vector2, frames: int) -> void:
	for frame in range(frames):
		await physics_frame
		actor.simulate(movement, 0.0, false, 1.0 / 60.0)


func _check(condition: bool, label: String, actual: Variant) -> void:
	print("%s: %s (%s)" % ["PASS" if condition else "FAIL", label, str(actual)])
	if not condition:
		failures.append(label + ": " + str(actual))
