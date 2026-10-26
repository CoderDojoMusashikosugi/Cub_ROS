extends SceneTree

const ActorScript = preload("res://scripts/actor.gd")
var failures := PackedStringArray()


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	var app: Node3D = load("res://scenes/main.tscn").instantiate()
	root.add_child(app)
	app.ros_bridge.url = "ws://127.0.0.1:1"
	_check(app.session.start_host(27349, "Cub4") == OK, "host starts with Cub4 as the primary robot")
	var primary: Node3D = app.actors[1]
	for index in range(3):
		app.session._add_record(app.session._record(100 + index, 100 + index,
			["pedestrian", "robot", "cone"][index], ["Walker", "Robot", "Cone"][index],
			Vector3(float(index) * 1.7 - 2.0, 0.01, 1.0), false, index + 1))
	await physics_frame
	await process_frame
	var model: Node3D = primary._visual.get_node_or_null("Cub4")
	_check(model != null, "the primary robot uses the imported Cub4 model")
	if model == null:
		app.session.leave()
		quit(1)
		return
	var meshes := _meshes(model)
	_check(meshes.size() == 1 and meshes[0].mesh.get_surface_count() == 1,
		"Cub4 shares one mesh and one material surface")
	var bounds := AABB()
	var first := true
	for instance: MeshInstance3D in meshes:
		var relative: Transform3D = primary._visual.global_transform.affine_inverse() * instance.global_transform
		var part_bounds: AABB = relative * instance.get_aabb()
		bounds = part_bounds if first else bounds.merge(part_bounds)
		first = false
		_check(instance.cast_shadow == GeometryInstance3D.SHADOW_CASTING_SETTING_OFF, "Cub4 does not add shadow rendering")
	print("Cub4 visual bounds in meters: ", bounds)
	_check(absf(bounds.size.x - 0.254) < 0.005 and absf(bounds.size.y - 0.594) < 0.005
		and absf(bounds.size.z - 0.378) < 0.005 and absf(bounds.position.y - 0.0063) < 0.002,
		"Cub4 is upright, at URDF scale, and retains its base_link origin")
	var mesh: Mesh = meshes[0].mesh
	_check((mesh.surface_get_format(0) & Mesh.ARRAY_FORMAT_COLOR) != 0,
		"source material colors are retained as vertex colors")
	var material: BaseMaterial3D = meshes[0].get_active_material(0)
	_check(material != null and material.vertex_color_use_as_albedo,
		"Cub4's material displays the source vertex colors")
	_check(primary._visual.find_children("*", "CollisionObject3D", true, false).is_empty()
		and primary._shape.shape is CapsuleShape3D,
		"the CAD model is display-only and the existing movement collider remains")
	var remote := ActorScript.new()
	remote.configure(app.session.records[1], false)
	root.add_child(remote)
	var remote_meshes := _meshes(remote._visual.get_node("Cub4"))
	_check(remote_meshes.size() == 1 and remote_meshes[0].mesh == mesh and not remote.simulated,
		"a client-side primary robot uses the same Cub4 mesh without local physics")
	root.remove_child(remote)
	remote.queue_free()
	for id in [100, 101, 102]:
		_check(app.actors[id]._visual.get_node_or_null("Cub4") == null,
			"actor %d retains its existing generic model" % id)
	_check(app.actors[101]._shape.shape.height == primary._shape.shape.height
		and app.actors[101]._shape.shape.radius == primary._shape.shape.radius,
		"changing the main robot's appearance does not alter its movement envelope")
	if DisplayServer.get_name() != "headless":
		app._select_camera(0)
		app.camera_rig.yaw = 2.7
		app.camera_rig.pitch = 0.35
		app.camera_rig.distance = 1.1
		# Orbit mode holds a close-up point rather than TPS's 0.8 m follow offset.
		app.camera_rig.mode = 1
		app.camera_rig.center = primary.global_position + Vector3.UP * 0.3
		await _capture("cub4-closeup.png", app)
		app._select_camera(0)
		app.camera_rig.yaw = 0.55
		await _capture("cub4-scene.png", app)
	app.session.leave()
	print("CUB4 MODEL TESTS PASSED" if failures.is_empty() else "CUB4 MODEL TESTS FAILED")
	quit(0 if failures.is_empty() else 1)


func _meshes(node: Node) -> Array:
	var meshes: Array = []
	if node is MeshInstance3D:
		meshes.append(node)
	meshes.append_array(node.find_children("*", "MeshInstance3D", true, false))
	return meshes


func _capture(filename: String, app: Node3D) -> void:
	app.camera_rig.update(0.0)
	for frame in range(12):
		await process_frame
	await RenderingServer.frame_post_draw
	var path := ProjectSettings.globalize_path("res://.godot/" + filename)
	root.get_texture().get_image().save_png(path)
	print("PREVIEW: ", path)


func _check(condition: bool, description: String) -> void:
	print("%s: %s" % ["PASS" if condition else "FAIL", description])
	if not condition:
		failures.append(description)
