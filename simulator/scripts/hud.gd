extends CanvasLayer
## Mouse-operated lobby with text-only world/preset selection, then a single passive status line during gameplay.

signal host_requested(address: String, port: int, nickname: String)
signal join_requested(address: String, port: int, kind: String, nickname: String)
signal cancel_requested
signal world_selected(world_id: String)
signal spawn_selected(preset_name: String)

const CAMERA_NAMES := ["TPS", "ORBIT", "TOP"]
const BAR_HEIGHT := 28
const KINDS := ["robot", "pedestrian", "cone"]

var _connection_status := "OFFLINE"
var _world_ready := true
var _world_status_text := ""
var _lobby: PanelContainer
var _bar: PanelContainer
var _nickname: LineEdit
var _address: LineEdit
var _ros_url: LineEdit
var _port: SpinBox
var _kind: OptionButton
var _world_option: OptionButton
var _spawn_option: OptionButton
var _world_info: Label
var _host_button: Button
var _join_button: Button
var _cancel_button: Button
var _lobby_status: Label
var _state: Label
var _position: Label
var _performance: Label
var _warp_crosshair: Control


func _ready() -> void:
	var root := Control.new()
	root.name = "Interface"
	root.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	root.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(root)
	_build_lobby(root)
	_build_warp_crosshair(root)
	var bar := PanelContainer.new()
	_bar = bar
	bar.name = "StatusBar"
	bar.set_anchors_and_offsets_preset(Control.PRESET_BOTTOM_WIDE)
	bar.offset_top = -BAR_HEIGHT
	bar.mouse_filter = Control.MOUSE_FILTER_IGNORE
	var style := StyleBoxFlat.new()
	style.bg_color = Color(0.06, 0.10, 0.14, 0.95)
	style.border_width_top = 1
	style.border_color = Color(0.34, 0.50, 0.58, 0.8)
	style.content_margin_left = 10
	style.content_margin_right = 10
	style.content_margin_top = 4
	style.content_margin_bottom = 3
	bar.add_theme_stylebox_override("panel", style)
	root.add_child(bar)
	var row := HBoxContainer.new()
	row.mouse_filter = Control.MOUSE_FILTER_IGNORE
	row.add_theme_constant_override("separation", 16)
	bar.add_child(row)
	_state = _label()
	_state.name = "State"
	_state.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_state.text_overrun_behavior = TextServer.OVERRUN_TRIM_ELLIPSIS
	_state.clip_text = true
	row.add_child(_state)
	_position = _label()
	_position.name = "Coordinates"
	_position.add_theme_color_override("font_color", Color("9cd5d0"))
	row.add_child(_position)
	_performance = _label()
	_performance.name = "Performance"
	row.add_child(_performance)
	bar.hide()


func _build_warp_crosshair(root: Control) -> void:
	_warp_crosshair = Control.new()
	_warp_crosshair.name = "WarpCrosshair"
	_warp_crosshair.mouse_filter = Control.MOUSE_FILTER_IGNORE
	root.add_child(_warp_crosshair)
	_warp_crosshair.set_anchors_and_offsets_preset(Control.PRESET_CENTER)
	_warp_crosshair.offset_left = -14
	_warp_crosshair.offset_top = -14
	_warp_crosshair.offset_right = 14
	_warp_crosshair.offset_bottom = 14
	_warp_crosshair.draw.connect(_draw_warp_crosshair)
	_warp_crosshair.hide()


func _draw_warp_crosshair() -> void:
	for width in [4.0, 2.0]:
		var color := Color("17232e") if width == 4.0 else Color.WHITE
		_warp_crosshair.draw_line(Vector2(2, 14), Vector2(26, 14), color, width)
		_warp_crosshair.draw_line(Vector2(14, 2), Vector2(14, 26), color, width)


func _build_lobby(root: Control) -> void:
	var center := CenterContainer.new()
	center.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	center.mouse_filter = Control.MOUSE_FILTER_IGNORE
	root.add_child(center)
	_lobby = PanelContainer.new()
	_lobby.name = "Lobby"
	_lobby.custom_minimum_size.x = 400
	var style := StyleBoxFlat.new()
	style.bg_color = Color(0.06, 0.10, 0.14, 0.96)
	style.set_content_margin_all(20)
	style.set_corner_radius_all(6)
	_lobby.add_theme_stylebox_override("panel", style)
	center.add_child(_lobby)
	var column := VBoxContainer.new()
	column.add_theme_constant_override("separation", 10)
	_lobby.add_child(column)
	var title := _label()
	title.text = "Cub Simulator"
	title.add_theme_font_size_override("font_size", 22)
	column.add_child(title)
	var form := GridContainer.new()
	form.columns = 2
	form.add_theme_constant_override("h_separation", 14)
	form.add_theme_constant_override("v_separation", 10)
	column.add_child(form)
	_nickname = LineEdit.new()
	_nickname.text = "Player"
	_nickname.max_length = 24
	_add_field(form, "Name", _nickname)
	_kind = OptionButton.new()
	for kind in KINDS:
		_kind.add_item(kind.capitalize())
	_add_field(form, "Client actor", _kind)
	_world_option = OptionButton.new()
	_world_option.name = "WorldOption"
	_world_option.item_selected.connect(_on_world_item_selected)
	_add_field(form, "Host world", _world_option)
	_spawn_option = OptionButton.new()
	_spawn_option.name = "SpawnOption"
	_spawn_option.item_selected.connect(_on_spawn_item_selected)
	_add_field(form, "Start preset", _spawn_option)
	_address = LineEdit.new()
	_address.text = "127.0.0.1"
	_add_field(form, "Address", _address)
	_port = SpinBox.new()
	_port.min_value = 1024
	_port.max_value = 65535
	_port.value = 27140
	_add_field(form, "UDP port", _port)
	_ros_url = LineEdit.new()
	_ros_url.text = "ws://127.0.0.1:9090"
	_add_field(form, "Host ROS URL", _ros_url)
	_world_info = _label()
	_world_info.name = "WorldInfo"
	_world_info.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_world_info.add_theme_color_override("font_color", Color("9cb6c9"))
	column.add_child(_world_info)
	var buttons := HBoxContainer.new()
	buttons.add_theme_constant_override("separation", 10)
	column.add_child(buttons)
	_host_button = Button.new()
	_host_button.text = "Host"
	_host_button.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_host_button.pressed.connect(func(): host_requested.emit(_address.text, int(_port.value), _nickname.text))
	buttons.add_child(_host_button)
	_join_button = Button.new()
	_join_button.text = "Join"
	_join_button.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_join_button.pressed.connect(func(): join_requested.emit(_address.text, int(_port.value), KINDS[_kind.selected], _nickname.text))
	buttons.add_child(_join_button)
	_cancel_button = Button.new()
	_cancel_button.text = "Cancel"
	_cancel_button.pressed.connect(func(): cancel_requested.emit())
	buttons.add_child(_cancel_button)
	_cancel_button.hide()
	_lobby_status = _label()
	_lobby_status.name = "LobbyStatus"
	_lobby_status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	column.add_child(_lobby_status)


func _add_field(form: GridContainer, title: String, field: Control) -> void:
	var label := _label()
	label.text = title
	form.add_child(label)
	field.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	form.add_child(field)


func set_worlds(worlds: Array[Dictionary], current_world_id: String) -> void:
	_world_option.clear()
	var selected_idx := 0
	for index in range(worlds.size()):
		var entry: Dictionary = worlds[index]
		var w_id := str(entry.get("world_id", ""))
		var w_name := str(entry.get("display_name", w_id))
		var w_kind := str(entry.get("kind", "unknown"))
		var ok := bool(entry.get("ok", false))
		var item_text := "%s (%s)%s" % [w_name, w_kind, "" if ok else " [INVALID]"]
		_world_option.add_item(item_text, index)
		_world_option.set_item_metadata(index, w_id)
		if w_id == current_world_id:
			selected_idx = index
	if _world_option.item_count > 0:
		_world_option.select(selected_idx)


func set_spawn_presets(definition: Dictionary, selected_preset := "") -> void:
	_spawn_option.clear()
	var spawns: Variant = definition.get("spawns", {})
	if not (spawns is Dictionary) or (spawns as Dictionary).is_empty():
		_spawn_option.add_item("None", 0)
		_spawn_option.set_item_metadata(0, "")
		_spawn_option.disabled = true
		return
	_spawn_option.disabled = false
	var default_preset := str(definition.get("default_spawn", "start_a"))
	var target := selected_preset if not selected_preset.is_empty() else default_preset
	var names: Array[String] = []
	if spawns.has(default_preset):
		names.append(default_preset)
	var extra: Array[String] = []
	for key in spawns.keys():
		var k := str(key)
		if k != default_preset and k != "default":
			extra.append(k)
	extra.sort()
	names.append_array(extra)
	if names.is_empty() and spawns.has("default"):
		names.append("default")
	var selected_idx := 0
	for index in range(names.size()):
		var preset_id := names[index]
		var preset_dict: Dictionary = spawns.get(preset_id, {})
		var label_text := str(preset_dict.get("display_name", preset_id))
		_spawn_option.add_item("%s [%s]" % [label_text, preset_id], index)
		_spawn_option.set_item_metadata(index, preset_id)
		if preset_id == target or (target == "default" and preset_id == default_preset):
			selected_idx = index
	if _spawn_option.item_count > 0:
		_spawn_option.select(selected_idx)


func set_world_status(ready: bool, info_text: String) -> void:
	_world_ready = ready
	_world_status_text = info_text
	if _world_info:
		_world_info.text = info_text
	if _host_button:
		_host_button.disabled = not ready


func selected_world_id() -> String:
	if not _world_option or _world_option.selected < 0 or _world_option.item_count == 0:
		return ""
	var meta: Variant = _world_option.get_item_metadata(_world_option.selected)
	return str(meta) if meta != null else ""


func selected_spawn_preset() -> String:
	if not _spawn_option or _spawn_option.selected < 0 or _spawn_option.item_count == 0:
		return ""
	var meta: Variant = _spawn_option.get_item_metadata(_spawn_option.selected)
	return str(meta) if meta != null else ""


func _on_world_item_selected(index: int) -> void:
	var meta: Variant = _world_option.get_item_metadata(index)
	if meta != null:
		world_selected.emit(str(meta))


func _on_spawn_item_selected(index: int) -> void:
	var meta: Variant = _spawn_option.get_item_metadata(index)
	if meta != null:
		spawn_selected.emit(str(meta))


func set_options(options: Dictionary) -> void:
	_ros_url.text = str(options.get("ros-url", "ws://127.0.0.1:9090"))
	_nickname.text = str(options.get("name", "Player"))
	_address.text = str(options.get("join", options.get("bind", "127.0.0.1")))
	_port.value = int(options.get("port", 27140))
	var index := KINDS.find(str(options.get("actor", "robot")))
	_kind.select(maxi(0, index))


func ros_url() -> String:
	return _ros_url.text.strip_edges()


func set_status(message: String) -> void:
	_connection_status = message


func update_readout(state: Dictionary) -> void:
	_lobby.visible = not state["active"]
	_bar.visible = state["active"]
	_warp_crosshair.visible = state["active"] and int(state["camera"]) == 2
	if not state["active"]:
		var connecting: bool = state["connecting"]
		_host_button.disabled = connecting or not _world_ready
		_join_button.disabled = connecting
		_cancel_button.visible = connecting
		_nickname.editable = not connecting
		_address.editable = not connecting
		_port.editable = not connecting
		_ros_url.editable = not connecting
		_kind.disabled = connecting
		_world_option.disabled = connecting
		_spawn_option.disabled = connecting or _spawn_option.item_count == 0
		_world_info.text = _world_status_text
		_lobby_status.text = "" if _connection_status == "OFFLINE" else _connection_status
		return
	var connection := "HOST" if state["host"] else "CLIENT"
	var world_label := str(state.get("world_name", "")).strip_edges()
	_state.text = "%s | %s | %s%s%s" % [
		connection,
		str(state["kind"]).capitalize(),
		CAMERA_NAMES[int(state["camera"])],
		" | CURSOR" if state["cursor_free"] else "",
		(" | " + world_label) if not world_label.is_empty() else "",
	]
	if state["host"]:
		_state.text += " | " + str(state.get("ros", "ROS OFFLINE"))
	if state["position"] is Vector3:
		var at: Vector3 = state["position"]
		_position.text = "X %.2f  Y %.2f  Z %.2f  |  %.0f°" % [at.x, at.y, at.z, float(state["yaw"])]
	else:
		_position.text = "X --  Y --  Z --"
	_performance.text = "%d actors | %d FPS" % [int(state["actors"]), int(state["fps"])]


func _label() -> Label:
	var label := Label.new()
	label.add_theme_font_size_override("font_size", 13)
	label.add_theme_color_override("font_color", Color("dce6ee"))
	label.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	return label
