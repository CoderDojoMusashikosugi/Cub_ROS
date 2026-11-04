extends RefCounted
## Discovers, validates, and loads world definitions from res://worlds/<world_id>/world.json.

const WORLDS_ROOT := "res://worlds"
const DEFAULT_WORLD_ID := "cub_playground_v1"
const KNOWN_WORLD_IDS := ["cub_playground_v1", "gnss2026_mid360_1017_02"]
const SUPPORTED_SCHEMA_VERSION := 1
const SUPPORTED_KINDS := ["playground", "occupancy_extrusion"]


static func list_worlds(worlds_root := WORLDS_ROOT) -> Array[Dictionary]:
	var ids: Array[String] = []
	for known_id in KNOWN_WORLD_IDS:
		var known_path := "%s/%s/world.json" % [worlds_root, known_id]
		if FileAccess.file_exists(known_path):
			ids.append(known_id)
	var dir := DirAccess.open(worlds_root)
	if dir != null:
		dir.list_dir_begin()
		var entry := dir.get_next()
		var extra_ids: Array[String] = []
		while not entry.is_empty():
			if dir.current_is_dir() and not entry.begins_with(".") and entry not in ids:
				var candidate := "%s/%s/world.json" % [worlds_root, entry]
				if FileAccess.file_exists(candidate):
					extra_ids.append(entry)
			entry = dir.get_next()
		dir.list_dir_end()
		extra_ids.sort()
		ids.append_array(extra_ids)
	var results: Array[Dictionary] = []
	for world_id in ids:
		results.append(load_world_definition(world_id, worlds_root, false))
	return results


static func load_world_definition(world_id: String, worlds_root := WORLDS_ROOT, verify_hashes := true) -> Dictionary:
	var clean_id := world_id.strip_edges()
	if clean_id.is_empty():
		return _invalid("", "World ID is empty.")
	var base_dir := "%s/%s" % [worlds_root, clean_id]
	var json_path := "%s/world.json" % base_dir
	if not FileAccess.file_exists(json_path):
		return _invalid(clean_id, "World '%s' not found (%s is missing)." % [clean_id, json_path])
	var raw_text := FileAccess.get_file_as_string(json_path)
	var parsed: Variant = JSON.parse_string(raw_text)
	if not (parsed is Dictionary):
		return _invalid(clean_id, "World '%s' has invalid JSON in %s." % [clean_id, json_path])
	return validate_definition(parsed, base_dir, verify_hashes)


static func validate_definition(raw: Dictionary, base_dir: String, verify_hashes := true) -> Dictionary:
	var definition := raw.duplicate(true)
	var world_id := str(definition.get("world_id", "")).strip_edges()
	if world_id.is_empty():
		return _invalid("", "world.json is missing 'world_id'.")
	var schema_version := int(definition.get("schema_version", -1))
	if schema_version != SUPPORTED_SCHEMA_VERSION:
		return _invalid(world_id, "World '%s' has unsupported schema_version %d (expected %d)." % [
			world_id, schema_version, SUPPORTED_SCHEMA_VERSION
		])
	var kind := str(definition.get("kind", "")).strip_edges()
	if kind not in SUPPORTED_KINDS:
		return _invalid(world_id, "World '%s' has unsupported kind '%s'." % [world_id, kind])
	var display_name := str(definition.get("display_name", world_id)).strip_edges()
	if display_name.is_empty():
		display_name = world_id
	var world_hash := str(definition.get("world_hash", "")).strip_edges()
	if world_hash.is_empty():
		return _invalid(world_id, "World '%s' is missing 'world_hash'." % world_id, display_name, kind)
	if not (definition.get("bounds") is Dictionary):
		return _invalid(world_id, "World '%s' is missing 'bounds'." % world_id, display_name, kind)
	if not (definition.get("spawns") is Dictionary) or (definition["spawns"] as Dictionary).is_empty():
		return _invalid(world_id, "World '%s' has no spawn presets." % world_id, display_name, kind)

	if kind == "occupancy_extrusion":
		var source: Variant = definition.get("source")
		var extrusion: Variant = definition.get("extrusion")
		if not (source is Dictionary) or not (extrusion is Dictionary):
			return _invalid(world_id, "World '%s' is missing occupancy 'source' or 'extrusion' settings." % world_id, display_name, kind)
		var image_name := str(source.get("image", "source.png"))
		var chunks_name := str(extrusion.get("chunks_file", "wall_chunks.json"))
		if image_name.is_absolute_path() or chunks_name.is_absolute_path():
			return _invalid(world_id, "World '%s' must not contain OS absolute paths." % world_id, display_name, kind)
		var image_path := "%s/%s" % [base_dir, image_name]
		var chunks_path := "%s/%s" % [base_dir, chunks_name]
		if not FileAccess.file_exists(image_path):
			return _invalid(world_id, "World '%s' is missing required asset '%s'." % [world_id, image_name], display_name, kind)
		if not FileAccess.file_exists(chunks_path):
			return _invalid(world_id, "World '%s' is missing required asset '%s'." % [world_id, chunks_name], display_name, kind)
		if verify_hashes:
			var expected_png_sha := str(source.get("source_png_sha256", ""))
			if not expected_png_sha.is_empty():
				var actual_png_sha := FileAccess.get_sha256(image_path)
				if not actual_png_sha.is_empty() and actual_png_sha != expected_png_sha:
					return _invalid(world_id, "World '%s' asset '%s' SHA-256 mismatch." % [world_id, image_name], display_name, kind)
			var expected_chunks_sha := str(extrusion.get("chunks_sha256", ""))
			if not expected_chunks_sha.is_empty():
				var actual_chunks_sha := FileAccess.get_sha256(chunks_path)
				if not actual_chunks_sha.is_empty() and actual_chunks_sha != expected_chunks_sha:
					return _invalid(world_id, "World '%s' asset '%s' SHA-256 mismatch." % [world_id, chunks_name], display_name, kind)

	definition["ok"] = true
	definition["error"] = ""
	definition["base_dir"] = base_dir
	definition["display_name"] = display_name
	return definition


static func verify_remote_offer(offer: Dictionary, worlds_root := WORLDS_ROOT) -> Dictionary:
	var world_id := str(offer.get("world_id", "")).strip_edges()
	var remote_name := str(offer.get("display_name", world_id)).strip_edges()
	var remote_hash := str(offer.get("world_hash", "")).strip_edges()
	var remote_schema := int(offer.get("schema_version", -1))
	var remote_kind := str(offer.get("kind", "")).strip_edges()
	if world_id.is_empty() or remote_hash.is_empty():
		return _invalid(world_id, "Host sent incomplete world metadata.", remote_name, remote_kind)
	if remote_schema != SUPPORTED_SCHEMA_VERSION:
		return _invalid(
			world_id,
			"World '%s' (%s) schema version mismatch: host=%d, local=%d." % [
				remote_name, world_id, remote_schema, SUPPORTED_SCHEMA_VERSION
			],
			remote_name,
			remote_kind
		)
	var local_def := load_world_definition(world_id, worlds_root)
	if not bool(local_def.get("ok", false)):
		return _invalid(
			world_id,
			"Missing or invalid world '%s' (%s, required hash %s): %s" % [
				remote_name, world_id, remote_hash.left(12), str(local_def.get("error", "missing files"))
			],
			remote_name,
			remote_kind
		)
	var local_hash := str(local_def.get("world_hash", ""))
	if local_hash != remote_hash:
		return _invalid(
			world_id,
			"World '%s' (%s) version mismatch: host=%s, local=%s. Regenerate or sync simulator/worlds/%s." % [
				remote_name, world_id, remote_hash.left(12), local_hash.left(12), world_id
			],
			remote_name,
			remote_kind
		)
	return local_def


static func spawn_preset_names(definition: Dictionary) -> Array[String]:
	var names: Array[String] = []
	var spawns: Variant = definition.get("spawns", {})
	if not (spawns is Dictionary):
		return names
	var default_spawn := str(definition.get("default_spawn", "start_a"))
	if spawns.has(default_spawn):
		names.append(default_spawn)
	var rest: Array[String] = []
	for key in spawns.keys():
		var name := str(key)
		if name != default_spawn and name != "default":
			rest.append(name)
	rest.sort()
	names.append_array(rest)
	if names.is_empty() and spawns.has("default"):
		names.append("default")
	return names


static func _invalid(world_id: String, error: String, display_name := "", kind := "") -> Dictionary:
	return {
		"ok": false,
		"error": error,
		"world_id": world_id,
		"display_name": display_name if not display_name.is_empty() else world_id,
		"kind": kind,
		"world_hash": "",
		"spawns": {},
	}
