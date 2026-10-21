extends RefCounted
## Replay source-timed snapshots; packet arrival bursts do not change speed.

const DELAY := 0.12
const MAX_SAMPLES := 32
const TELEPORT_DISTANCE := 6.0

var position := Vector3.ZERO
var yaw := 0.0
var _samples: Array[Dictionary] = []
var _play_time := 0.0


func reset(at: Vector3, heading: float, sample_time: float) -> void:
	position = at
	yaw = heading
	_play_time = sample_time - DELAY
	_samples.assign([{"time": sample_time, "position": at, "yaw": heading}])


func push(at: Vector3, heading: float, sample_time: float) -> void:
	if _samples.is_empty():
		reset(at, heading, sample_time)
		return
	var last: Dictionary = _samples.back()
	if sample_time <= float(last["time"]):
		return
	if at.distance_squared_to(last["position"]) > TELEPORT_DISTANCE * TELEPORT_DISTANCE:
		reset(at, heading, sample_time)
		return
	_samples.append({"time": sample_time, "position": at, "yaw": heading})
	while _samples.size() > MAX_SAMPLES:
		_samples.pop_front()
	# Recover from a suspended app or a long connection stall with bounded lag.
	if sample_time - _play_time > 0.5:
		_play_time = sample_time - DELAY


func advance(delta: float) -> void:
	if _samples.is_empty():
		return
	var newest: float = _samples.back()["time"]
	var depth := newest - _play_time
	# A wide dead band avoids following each packet's timing jitter. Small speed
	# corrections handle clock drift without jumps or unbounded display delay.
	var speed := 1.05 if depth > DELAY * 2.0 else (0.95 if depth < DELAY * 0.5 else 1.0)
	_play_time = minf(_play_time + delta * speed, newest)
	while _samples.size() > 2 and float(_samples[1]["time"]) <= _play_time:
		_samples.pop_front()
	var a: Dictionary = _samples[0]
	if _samples.size() == 1 or _play_time <= float(a["time"]):
		position = a["position"]
		yaw = a["yaw"]
		return
	var b: Dictionary = _samples[1]
	var weight := clampf((_play_time - float(a["time"])) / (float(b["time"]) - float(a["time"])), 0.0, 1.0)
	position = a["position"].lerp(b["position"], weight)
	yaw = lerp_angle(float(a["yaw"]), float(b["yaw"]), weight)
