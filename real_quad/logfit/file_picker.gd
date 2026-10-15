extends Control
## real_quad: a file browser inside the game window, for the model builder's "Add log files...". The
## engine's FileDialog is a window of its own on this platform (an extra OS window: the game's pause menu
## reacts to every window's focus and prints errors for it, and the game hides the mouse cursor over it), so
## the browser is a Control: one overlay over the builder, sized by anchors (it scales with the window), no
## other window anywhere. Several files can be picked (click each; Shift/Ctrl as in any list; "All").
##     open(start_dir)          shows it in start_dir (or the home folder when that is gone)
##     picked(paths)            the Open button, a double-click on a file, or a file path typed in the box
##     canceled                 Cancel, Esc
## exts: the file types shown ("bbl", ...; empty = all files); "All files" shows everything anyway.
## The path box is in the group "zm_typing", which mod_settings' key guard keeps the game's keys away from.

signal picked(paths: PackedStringArray)
signal canceled

const TYPING := "zm_typing"
const DIM := Color(1, 1, 1, 0.62)
const DIR_COLOR := Color(0.75, 0.85, 1.0)

var title := "Pick files"
var exts := PackedStringArray()
var dir := ""

var _items: Array = []                # per list row: {path, dir}
var _list: ItemList = null
var _path: LineEdit = null
var _status: Label = null
var _open: Button = null
var _hidden: CheckBox = null
var _all: CheckBox = null
var _places: HFlowContainer = null
var _title: Label = null
var _place_dirs: Array = []


func _ready() -> void:
	process_mode = Node.PROCESS_MODE_ALWAYS
	mouse_filter = Control.MOUSE_FILTER_STOP
	visible = false
	set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)   # the whole builder window, whatever its size
	var dim := ColorRect.new()
	dim.color = Color(0, 0, 0, 0.55)
	dim.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	add_child(dim)
	var box := PanelContainer.new()
	box.name = "Box"
	box.anchor_left = 0.12
	box.anchor_right = 0.88
	box.anchor_top = 0.08
	box.anchor_bottom = 0.92
	var sb := StyleBoxFlat.new()
	sb.bg_color = Color(0.07, 0.08, 0.11, 1.0)
	sb.set_corner_radius_all(6)
	sb.set_content_margin_all(12)
	box.add_theme_stylebox_override("panel", sb)
	add_child(box)
	var col := VBoxContainer.new()
	col.add_theme_constant_override("separation", 8)
	box.add_child(col)
	_title = Label.new()
	_title.text = title
	_title.add_theme_font_size_override("font_size", int(get_theme_default_font_size() * 1.2))
	col.add_child(_title)
	var nav := HBoxContainer.new()
	nav.add_theme_constant_override("separation", 6)
	col.add_child(nav)
	var up := Button.new()
	up.name = "PickerUp"
	up.text = " Up "
	up.tooltip_text = "The folder above"
	up.pressed.connect(func(): go(dir.get_base_dir()))
	nav.add_child(up)
	_path = LineEdit.new()
	_path.name = "PickerPath"
	_path.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_path.placeholder_text = "Folder (or the path of one file), Enter"
	_path.add_to_group(TYPING)
	_path.text_submitted.connect(_path_entered)
	nav.add_child(_path)
	# plain buttons for the places: a drop-down opens a popup window of its own
	_places = HFlowContainer.new()
	_places.name = "PickerPlaces"
	_places.add_theme_constant_override("h_separation", 6)
	col.add_child(_places)
	_list = ItemList.new()
	_list.name = "PickerList"
	_list.select_mode = ItemList.SELECT_MULTI
	_list.allow_reselect = true
	_list.size_flags_vertical = Control.SIZE_EXPAND_FILL
	_list.auto_height = false
	_list.multi_selected.connect(func(_i, _s): _selection_changed())
	_list.item_activated.connect(_activated)
	col.add_child(_list)
	var opts := HBoxContainer.new()
	opts.add_theme_constant_override("separation", 14)
	col.add_child(opts)
	_all = CheckBox.new()
	_all.name = "PickerAllFiles"
	_all.text = "All files"
	_all.tooltip_text = "Show files of every type, not only the ones this needs"
	_all.toggled.connect(func(_on): _refresh())
	opts.add_child(_all)
	_hidden = CheckBox.new()
	_hidden.name = "PickerHidden"
	_hidden.text = "Hidden"
	_hidden.tooltip_text = "Show hidden files and folders"
	_hidden.toggled.connect(func(_on): _refresh())
	opts.add_child(_hidden)
	var sel := Button.new()
	sel.name = "PickerSelectAll"
	sel.text = " Select all "
	sel.tooltip_text = "Select every file shown in this folder"
	sel.pressed.connect(select_all)
	opts.add_child(sel)
	_status = Label.new()
	_status.name = "PickerStatus"
	_status.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_status.clip_text = true
	_status.modulate = DIM
	opts.add_child(_status)
	var foot := HBoxContainer.new()
	foot.alignment = BoxContainer.ALIGNMENT_END
	foot.add_theme_constant_override("separation", 8)
	col.add_child(foot)
	var cancel := Button.new()
	cancel.name = "PickerCancel"
	cancel.text = " Cancel "
	cancel.pressed.connect(cancel_pick)
	foot.add_child(cancel)
	_open = Button.new()
	_open.name = "PickerOpen"
	_open.text = " Open "
	_open.pressed.connect(_open_pressed)
	foot.add_child(_open)
	_fill_places()


func open(start_dir := "") -> void:
	_title.text = title
	_fill_places()
	var d := start_dir
	if d == "" or not DirAccess.dir_exists_absolute(d):
		d = _home()
	visible = true
	go(d)
	_list.grab_focus()


func cancel_pick() -> void:
	visible = false
	canceled.emit()


func go(to: String) -> void:
	var t := _norm(to)
	if t == "" or not DirAccess.dir_exists_absolute(t):
		_status.text = "No such folder: %s" % to
		return
	dir = t
	_refresh()


## The paths picked in the list now
func selected_paths() -> PackedStringArray:
	var out := PackedStringArray()
	if _list == null:
		return out
	for i in _list.get_selected_items():
		if not _items[i]["dir"]:
			out.append(_items[i]["path"])
	return out


func select_all() -> void:
	for i in _list.item_count:
		if not _items[i]["dir"]:
			_list.select(i, false)
	_selection_changed()


func _refresh() -> void:
	_list.clear()
	_items.clear()
	_path.text = dir
	var d := DirAccess.open(dir)
	if d == null:
		_status.text = "Cannot read this folder"
		_open.disabled = true
		return
	d.include_hidden = _hidden.button_pressed
	var dirs := Array(d.get_directories())
	var files := Array(d.get_files())
	dirs.sort_custom(func(a, b): return a.naturalnocasecmp_to(b) < 0)
	files.sort_custom(func(a, b): return a.naturalnocasecmp_to(b) < 0)
	for n in dirs:
		var i := _list.add_item(n + "/")
		_list.set_item_selectable(i, false)
		_list.set_item_custom_fg_color(i, DIR_COLOR)
		_items.append({"path": dir.path_join(n), "dir": true})
	var shown := 0
	for n in files:
		if not _all.button_pressed and not exts.is_empty() and not exts.has(n.get_extension().to_lower()):
			continue
		_list.add_item(n)
		_items.append({"path": dir.path_join(n), "dir": false})
		shown += 1
	_status.text = "%d files" % shown if shown != 1 else "1 file"
	_selection_changed()


func _selection_changed() -> void:
	var n := selected_paths().size()
	_open.disabled = n == 0
	_open.text = " Open " if n == 0 else " Open (%d) " % n
	if n > 0:
		_status.text = "%d selected" % n


func _activated(i: int) -> void:
	if _items[i]["dir"]:
		go(_items[i]["path"])
		return
	# a double-click toggles the row twice: the row itself is picked, with what was selected before
	var out := selected_paths()
	if not out.has(_items[i]["path"]):
		out.append(_items[i]["path"])
	_finish(out)


func _open_pressed() -> void:
	var out := selected_paths()
	if not out.is_empty():
		_finish(out)


func _finish(out: PackedStringArray) -> void:
	visible = false
	picked.emit(out)


func _path_entered(t: String) -> void:
	var p := _norm(t.strip_edges())
	if p == "":
		return
	if DirAccess.dir_exists_absolute(p):
		go(p)
	elif FileAccess.file_exists(p):
		_finish(PackedStringArray([p]))
	else:
		_status.text = "No such folder or file: %s" % t.strip_edges()


func _fill_places() -> void:
	for c in _places.get_children():
		_places.remove_child(c)
		c.queue_free()
	var seen := {}
	var cands: Array = [["Home", _home()],
		["Documents", OS.get_system_dir(OS.SYSTEM_DIR_DOCUMENTS)],
		["Downloads", OS.get_system_dir(OS.SYSTEM_DIR_DOWNLOADS)],
		["Desktop", OS.get_system_dir(OS.SYSTEM_DIR_DESKTOP)]]
	if OS.get_name() == "Windows":
		for i in DirAccess.get_drive_count():
			var dn := DirAccess.get_drive_name(i)
			cands.append([dn, dn + "/" if dn.ends_with(":") else dn])
	else:
		cands.append(["/", "/"])
		var user := OS.get_environment("USER")
		for m in ["/run/media/" + user, "/media/" + user, "/Volumes"]:
			cands.append(["Drives" if m != "/Volumes" else "Volumes", m])
	for c in cands:
		var p := _norm(c[1])
		if p == "" or seen.has(p) or not DirAccess.dir_exists_absolute(p):
			continue
		seen[p] = true
		var b := Button.new()
		b.text = " %s " % c[0]
		b.tooltip_text = p
		b.pressed.connect(go.bind(p))
		_places.add_child(b)


func _home() -> String:
	var h := OS.get_environment("USERPROFILE") if OS.get_name() == "Windows" else OS.get_environment("HOME")
	return h if h != "" else OS.get_system_dir(OS.SYSTEM_DIR_DOCUMENTS)


static func _norm(p: String) -> String:
	var t := p.replace("\\", "/")
	if t.begins_with("~"):
		var h := OS.get_environment("USERPROFILE") if OS.get_name() == "Windows" else OS.get_environment("HOME")
		t = h + t.substr(1)
	if t == "":
		return ""
	t = t.simplify_path()
	while t.length() > 1 and t.ends_with("/") and not t.ends_with(":/"):
		t = t.left(-1)
	return t


## While this is open the game gets no keys: Esc closes the picker (not the pause menu), the keys that move
## in the list reach the list, typing goes to the path box (mod_settings' guard), everything else is eaten
func _input(event: InputEvent) -> void:
	if not visible or not (event is InputEventKey) or not event.pressed:
		return
	var f := get_viewport().gui_get_focus_owner()
	if f is LineEdit and f.is_in_group(TYPING) and f.has_method("is_editing") and f.call("is_editing"):
		return
	var k := event as InputEventKey
	if k.keycode == KEY_ESCAPE or k.is_action_pressed("ui_cancel"):
		get_viewport().set_input_as_handled()
		cancel_pick()
	elif not (k.keycode in [KEY_UP, KEY_DOWN, KEY_PAGEUP, KEY_PAGEDOWN, KEY_HOME, KEY_END, KEY_ENTER, KEY_KP_ENTER,
			KEY_SPACE, KEY_TAB, KEY_SHIFT, KEY_CTRL]) and not (k.keycode == KEY_A and k.ctrl_pressed):
		get_viewport().set_input_as_handled()
