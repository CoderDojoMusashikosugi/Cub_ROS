extends CanvasLayer
## Mouse-operated lobby, then a single passive status line during gameplay.

signal host_requested(address: String, port: int, nickname: String)
signal join_requested(address: String, port: int, kind: String, nickname: String)
signal cancel_requested

const CAMERA_NAMES := ["TPS", "ORBIT", "TOP"]
const BAR_HEIGHT := 28
const KINDS := ["robot", "pedestrian", "cone"]

var _connection_status := "OFFLINE"
var _lobby: PanelContainer
var _bar: PanelContainer
var _nickname: LineEdit
var _address: LineEdit
var _ros_url: LineEdit
var _port: SpinBox
var _kind: OptionButton
var _host_button: Button
var _join_button: Button
var _cancel_button: Button
var _lobby_status: Label
var _state: Label
var _position: Label
var _performance: Label


func _ready() -> void:
	var root := Control.new()
	root.name = "Interface"
	root.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	root.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(root)
	_build_lobby(root)
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


func _build_lobby(root: Control) -> void:
	var center := CenterContainer.new()
	center.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	center.mouse_filter = Control.MOUSE_FILTER_IGNORE
	root.add_child(center)
	_lobby = PanelContainer.new()
	_lobby.name = "Lobby"
	_lobby.custom_minimum_size.x = 360
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
	_lobby_status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	column.add_child(_lobby_status)


func _add_field(form: GridContainer, title: String, field: Control) -> void:
	var label := _label()
	label.text = title
	form.add_child(label)
	field.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	form.add_child(field)


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
	if not state["active"]:
		var connecting: bool = state["connecting"]
		_host_button.disabled = connecting
		_join_button.disabled = connecting
		_cancel_button.visible = connecting
		_nickname.editable = not connecting
		_address.editable = not connecting
		_port.editable = not connecting
		_kind.disabled = connecting
		_lobby_status.text = "" if _connection_status == "OFFLINE" else _connection_status
		return
	var connection := "HOST" if state["host"] else "CLIENT"
	_state.text = "%s | %s | %s%s" % [connection, str(state["kind"]).capitalize(),
		CAMERA_NAMES[int(state["camera"])], " | CURSOR" if state["cursor_free"] else ""]
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
