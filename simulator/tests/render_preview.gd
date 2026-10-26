extends SceneTree


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	var app: Node3D = load("res://scenes/main.tscn").instantiate()
	root.add_child(app)
	app.ros_bridge.url = "ws://127.0.0.1:1"
	for frame in range(5):
		await process_frame
	await RenderingServer.frame_post_draw
	root.get_texture().get_image().save_png(ProjectSettings.globalize_path("res://.godot/lobby-preview.png"))
	app.session.start_host(27345, "Host")
	for index in range(3):
		var record: Dictionary = app.session._record(100 + index, 100 + index,
			["pedestrian", "robot", "cone"][index], ["Walker", "Robot", "Cone"][index],
			Vector3(float(index) * 1.7 - 2.0, 0.01, 1.0), false, index + 1)
		app.session._add_record(record)
	for frame in range(30):
		await process_frame
	await RenderingServer.frame_post_draw
	var path := ProjectSettings.globalize_path("res://.godot/preview.png")
	root.get_texture().get_image().save_png(path)
	print("PREVIEW: ", path)
	print("FPS: ", Engine.get_frames_per_second(), "; draw calls: ", Performance.get_monitor(Performance.RENDER_TOTAL_DRAW_CALLS_IN_FRAME))
	quit()
