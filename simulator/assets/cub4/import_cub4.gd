@tool
extends EditorScenePostImport
## Keep Cub4's vertex colors and the simulator's shadow-free rendering policy.


func _post_import(scene: Node) -> Object:
	_prepare(scene)
	return scene


func _prepare(node: Node) -> void:
	if node is GeometryInstance3D:
		node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	if node is MeshInstance3D:
		for surface in range(node.mesh.get_surface_count()):
			var material := node.get_active_material(surface) as BaseMaterial3D
			if material:
				material.vertex_color_use_as_albedo = true
	for child in node.get_children():
		_prepare(child)
