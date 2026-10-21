extends SceneTree

const BufferScript = preload("res://scripts/pose_buffer.gd")
var failures := PackedStringArray()


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	for render_fps in [30, 45, 60]:
		_test_jitter(render_fps)
	_test_boundaries()
	print("SMOOTHING TESTS PASSED" if failures.is_empty() else "SMOOTHING TESTS FAILED")
	quit(0 if failures.is_empty() else 1)


func _test_jitter(render_fps: int) -> void:
	var poses := BufferScript.new()
	var events: Array[Dictionary] = []
	# Stress the old background limit as well: independent 15 FPS sender and
	# relay frames batch 20 Hz samples, with occasional packet loss/duplicates.
	for index in range(161):
		if index > 0 and index % 17 == 0:
			continue
		var source_time := float(index) / 20.0
		var owner_frame := ceilf((source_time + 0.007) * 15.0) / 15.0
		var relay_frame := ceilf((owner_frame + 0.023) * 15.0) / 15.0
		events.append({"arrival": relay_frame + 0.001, "time": source_time})
	var next := 0
	var old_position := Vector3.ZERO
	var old_target := Vector3.ZERO
	var previous := Vector3.ZERO
	var previous_yaw := 3.0
	var new_speeds := PackedFloat64Array()
	var old_speeds := PackedFloat64Array()
	var turn_speeds := PackedFloat64Array()
	var delta := 1.0 / float(render_fps)
	var received := false
	for frame in range(render_fps * 7):
		var now := float(frame) * delta
		while next < events.size() and float(events[next]["arrival"]) <= now:
			var sample_time: float = events[next]["time"]
			old_target = Vector3(sample_time * 3.0, 0, 0)
			poses.push(old_target, wrapf(3.0 + 0.8 * sample_time, -PI, PI), sample_time)
			# Repeated relays must not stretch the source movement timeline.
			poses.push(old_target, wrapf(3.0 + 0.8 * sample_time, -PI, PI), sample_time)
			received = true
			next += 1
		var before := old_position
		old_position = old_position.lerp(old_target, 1.0 - exp(-18.0 * delta))
		poses.advance(delta)
		if now > 1.0 and now < 6.0 and received:
			new_speeds.append((poses.position.x - previous.x) / delta)
			old_speeds.append((old_position.x - before.x) / delta)
			turn_speeds.append(wrapf(poses.yaw - previous_yaw, -PI, PI) / delta)
		previous = poses.position
		previous_yaw = poses.yaw
	var new_error := _deviation(new_speeds)
	var old_error := _deviation(old_speeds)
	print("%d FPS / speed stddev: old %.4f, buffered %.4f m/s" % [render_fps, old_error, new_error])
	_check(new_error < old_error * 0.2 and new_error < 0.15,
		"%d FPS: constant movement stays smooth through batching and packet loss" % render_fps)
	_check(absf(_mean(new_speeds) - 3.0) < 0.05 and absf(_mean(turn_speeds) - 0.8) < 0.03
		and _deviation(turn_speeds) < 0.05,
		"%d FPS: preserve linear/angular speed and rotate across +/-PI without a full turn" % render_fps)
	_check(poses._samples.size() <= BufferScript.MAX_SAMPLES, "buffer history stays bounded")


func _test_boundaries() -> void:
	var poses := BufferScript.new()
	poses.reset(Vector3.ZERO, 0.0, 0.0)
	for index in range(1, 41):
		var time := float(index) * 0.05
		poses.push(Vector3(minf(time, 1.0), 0, 0), 0.0, time)
		poses.advance(0.05)
	_check(poses.position.is_equal_approx(Vector3(1, 0, 0)), "a stopped actor settles at the exact received position")
	poses.push(Vector3(90, 0, 0), 2.0, 1.0)
	poses.advance(1.0)
	_check(poses.position.is_equal_approx(Vector3(1, 0, 0)), "late samples cannot rewind or teleport the actor")
	poses.advance(5.0)
	_check(poses.position.is_equal_approx(Vector3(1, 0, 0)), "connection stalls hold the last position without extrapolation or overshoot")
	poses.push(Vector3(1.2, 0, 0), 0.0, 20.0)
	poses.advance(0.15)
	_check(poses.position.distance_to(Vector3(1.2, 0, 0)) < 0.01, "resume after a long stall without accumulating seconds of delay")
	poses.push(Vector3(20, 0, 0), 1.0, 20.05)
	poses.advance(0.0)
	_check(poses.position.is_equal_approx(Vector3(20, 0, 0)), "respawn-sized teleports snap without sweeping across the map")


func _mean(values: PackedFloat64Array) -> float:
	var total := 0.0
	for value in values:
		total += value
	return total / values.size()


func _deviation(values: PackedFloat64Array) -> float:
	var mean := _mean(values)
	var sum := 0.0
	for value in values:
		sum += (value - mean) * (value - mean)
	return sqrt(sum / values.size())


func _check(condition: bool, description: String) -> void:
	print("%s: %s" % ["PASS" if condition else "FAIL", description])
	if not condition:
		failures.append(description)
