extends Control
## real_quad: the "Build a model from logs" window - pick Betaflight blackbox logs, read what is
## in them, say what the quad is, build the flight model, fly it. One panel over the pause menu in
## mod_settings' look (its colours, fonts and sizes), made in code; the work is logfit.gd's:
##     inspect(paths: PackedStringArray) -> Dictionary
##         {firmware, sessions, fields, prefill {cells, kv, motor_poles, craft_name}, errors, warnings}
##     build(paths, form: Dictionary, out_dir: String, progress: Callable) -> Dictionary
##         {ok, grade, errors, warnings, report_md, package_dir, model_id}
##     cancel()
## Both run in a Thread (the window polls it each frame, so it works while the game is paused),
## build reports progress(step_name: String, fraction: float). An error or warning is a String, or a
## Dictionary {text, fix} (the fix is a line or a CLI block); "big": true marks a banner.
## The form it gives build(): name, weight_g, cells, pack_type (lipo | lihv | liion), kv, class
## (whoop | micro | mini | five | seven | ten), zone_vehicle_id and, when filled in, capacity_mah,
## prop_in, pitch_in, blades, wheelbase_mm, top_speed_kmh, ducted, same_quad, overwrite.
## host is real_quad's main.gd: user_models_dir(), game_drones(), flown_vehicle(), model_origin(),
## set_drone_model(). Typing goes to the boxes, not the game: the boxes are in the group
## "zm_typing", which mod_settings' key guard keeps the game's keys away from.

signal closed

const DIM := Color(1, 1, 1, 0.62)
const WARN := Color(1.0, 0.78, 0.35)
const GOOD := Color(0.55, 0.9, 0.55)
const BAD := Color(1.0, 0.45, 0.4)
const GROUP := Color(0.75, 0.85, 1.0)
const LOGFIT := "res://real_quad/logfit/logfit.gd"
const CFG := "user://real_quad/logfit.cfg"
const MODELS := "user://real_quad/models"
const EXTS := ["bbl", "bfl", "txt"]
const TYPING := "zm_typing"

# contract 4.2: weight band (g), cells, default prop D / pitch / blades, pack mAh, ducted, Zone drone
const CLASSES := {
	"whoop": {"name": "Whoop (1.2-1.6 in, ducted)", "w": [18, 60], "d": 1.6, "pitch": 1.2, "blades": 3, "mah": 300,
		"ducted": true, "drone": "65mm_freestyle", "note": "Many AIO boards run BLHeli_S without bidirectional DShot (flash Bluejay); brushed whoops cannot be used. Ducts: punch and top speed are approximate. Some tiny motors have 12 poles. Weigh to 0.1 g: 1 g is 4 % at 25 g. 1S sags a lot."},
	"micro": {"name": "Micro (2-2.5 in)", "w": [50, 200], "d": 2.5, "pitch": 1.9, "blades": 3, "mah": 650,
		"ducted": false, "drone": "2.5in_freestyle", "note": "Cinewhoop ducts as for whoops. A camera on or off changes the weight by 20-30 %."},
	"mini": {"name": "Mini (3-3.5 in)", "w": [180, 400], "d": 3.5, "pitch": 2.5, "blades": 3, "mah": 850,
		"ducted": false, "drone": "3.5in_freestyle", "note": "Validated class. A GoPro on or off between logs is the commonest mixed configuration."},
	"five": {"name": "Five (4-5.5 in)", "w": [450, 850], "d": 5.1, "pitch": 4.5, "blades": 3, "mah": 1300,
		"ducted": false, "drone": "5in_freestyle", "note": "Validated class. A GoPro on or off between logs is the commonest mixed configuration."},
	"seven": {"name": "Seven (6-7 in)", "w": [700, 1300], "d": 7.0, "pitch": 4.0, "blades": 3, "mah": 2200,
		"ducted": false, "drone": "5in_freestyle", "note": "Not validated yet. Arm flex can put frame modes near 20-40 Hz. Long-range pilots rarely hover: fly a 10 s hover. No Zone drone is this size: it flies on the 5-inch."},
	"ten": {"name": "Ten (8-10 in)", "w": [1200, 3000], "d": 10.0, "pitch": 5.0, "blades": 2, "mah": 5000,
		"ducted": false, "drone": "5in_freestyle", "note": "Not validated yet. Li-ion packs: say the cell count yourself. No Zone drone is this size: it flies on the 5-inch."},
}
const CLASS_ORDER := ["whoop", "micro", "mini", "five", "seven", "ten"]
const PACKS := {"lipo": "LiPo", "lihv": "LiHV", "liion": "Li-ion"}
const MEANING := {"A": "flies like your quad", "B": "close; small differences", "C": "rough: right size and power, response approximate",
	"D": "a sketch: mostly class defaults"}

# the number boxes: key, label, kind, min, max, required, note
const FIELDS := [
	{"key": "weight_g", "label": "All-up weight with pack (g)", "int": false, "min": 10.0, "max": 5000.0, "req": true,
		"note": "Weigh the quad as flown, with the pack and the camera. Weight, props and pack cannot be checked from the logs: all the logs must be of this quad as entered."},
	{"key": "cells", "label": "Cells (S)", "int": true, "min": 1.0, "max": 12.0, "req": true, "note": ""},
	{"key": "kv", "label": "Motor KV", "int": false, "min": 100.0, "max": 60000.0, "req": true,
		"note": "On the motor's label. 1960 is Betaflight's default: check it."},
	{"key": "capacity_mah", "label": "Pack capacity (mAh)", "int": false, "min": 50.0, "max": 30000.0, "req": false, "note": ""},
	{"key": "prop_in", "label": "Prop diameter (in)", "int": false, "min": 1.0, "max": 13.0, "req": false, "note": ""},
	{"key": "pitch_in", "label": "Prop pitch (in)", "int": false, "min": 0.5, "max": 10.0, "req": false, "note": ""},
	{"key": "blades", "label": "Blades", "int": true, "min": 2.0, "max": 6.0, "req": false, "note": ""},
	{"key": "wheelbase_mm", "label": "Wheelbase (mm, motor to motor)", "int": false, "min": 40.0, "max": 1000.0, "req": false, "note": ""},
	{"key": "top_speed_kmh", "label": "Top speed (km/h)", "int": false, "min": 20.0, "max": 300.0, "req": false,
		"note": "Only used when the accelerometer cannot tell the body drag."},
]

var host = null                       # real_quad's main.gd
var logfit_script: Script = null      # logfit.gd, or a stand-in
var cfg_path := CFG
var out_root := MODELS
var hide_when_unpaused := false       # set by real_quad when opened from the pause menu
var paths := PackedStringArray()
var info := {}                        # inspect()'s answer for paths
var result := {}                      # build()'s answer
var stage := "setup"                  # setup | working | result

var _logfit = null
var _thread: Thread = null
var _mutex := Mutex.new()
var _prog := {"step": "", "frac": 0.0}
var _answer = null
var _done := false
var _kind := ""                       # the job running: inspect | build
var _again := false                   # the files changed while inspecting
var _cancelled := false
var _after := []                      # messages about the last build, shown with the findings
var _t0 := 0
var _was_paused := false
var _touched := {}                    # form key -> the player has set it
var _ctl := {}                        # form key -> its control
var _errs := {}                       # form key -> its error label
var _drones: Array = []               # [{id, name}]
var _dialog: FileDialog = null
var _box: PanelContainer = null
var _title: Label = null
var _pages := {}
var _files_box: VBoxContainer = null
var _msgs: VBoxContainer = null
var _form_box: VBoxContainer = null
var _hint: Label = null
var _build: Button = null
var _step: Label = null
var _bar: ProgressBar = null
var _elapsed: Label = null
var _res_box: VBoxContainer = null
var _report: RichTextLabel = null
var _use: Button = null
var _use_pick: OptionButton = null
var _use_note: Label = null
var _class_note: Label = null
var _overwrite: CheckBox = null
var _same: CheckBox = null
var _same_row: Control = null


func _ready() -> void:
	process_mode = Node.PROCESS_MODE_ALWAYS
	set_process(false)
	mouse_filter = Control.MOUSE_FILTER_STOP
	var dim := ColorRect.new()
	dim.color = Color(0, 0, 0, 0.5)
	dim.set_anchors_preset(Control.PRESET_FULL_RECT)
	add_child(dim)
	_box = PanelContainer.new()
	_box.name = "Box"
	var sb := StyleBoxFlat.new()
	sb.bg_color = Color(0.04, 0.05, 0.07, 0.96)
	sb.set_corner_radius_all(6)
	sb.set_content_margin_all(14)
	_box.add_theme_stylebox_override("panel", sb)
	add_child(_box)
	var col := VBoxContainer.new()
	col.add_theme_constant_override("separation", 8)
	_box.add_child(col)
	var base := get_theme_default_font_size()
	var bar := HBoxContainer.new()
	bar.add_theme_constant_override("separation", 12)
	col.add_child(bar)
	_title = Label.new()
	_title.text = "Build a model from logs"
	_title.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_title.add_theme_font_size_override("font_size", int(base * 1.3))
	bar.add_child(_title)
	var x := Button.new()
	x.name = "Close"
	x.text = " x "
	x.tooltip_text = "Close (a build keeps running; cancel it first to stop it)"
	x.pressed.connect(close)
	bar.add_child(x)
	for s in ["setup", "working", "result"]:
		var p := VBoxContainer.new()
		p.name = s.capitalize()
		p.size_flags_vertical = Control.SIZE_EXPAND_FILL
		p.add_theme_constant_override("separation", 8)
		col.add_child(p)
		_pages[s] = p
	_build_setup(_pages["setup"])
	_build_working(_pages["working"])
	_build_result(_pages["result"])
	get_viewport().size_changed.connect(_size)
	_size()
	_show("setup")
	_refresh()


func _exit_tree() -> void:
	if _thread != null:
		if _logfit != null:
			_logfit.cancel()
		_thread.wait_to_finish()
		_thread = null


## Opens the window on its first page (what was chosen stays)
func open() -> void:
	_was_paused = _paused()
	_drones = host.game_drones() if host != null and host.has_method("game_drones") else []
	_fill_drones()
	visible = true
	_size()
	_refresh()


func close() -> void:
	visible = false
	closed.emit()


func _paused() -> bool:
	var gs := get_node_or_null("/root/ZGamestate")
	return gs != null and gs.get("is_paused") == true


func _size() -> void:
	if _box == null:
		return
	var vs := get_viewport().get_visible_rect().size
	position = Vector2.ZERO
	size = vs
	var m := clampf(vs.y * 0.045, 12.0, 48.0)
	var w := minf(vs.x - 2.0 * m, 1100.0)
	_box.position = Vector2(roundf((vs.x - w) / 2.0), m)
	_box.size = Vector2(w, vs.y - 2.0 * m)


# ---- small pieces ----

func _label(text: String, size_mult := 1.0, color := Color.WHITE) -> Label:
	var l := Label.new()
	l.text = text
	l.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	l.custom_minimum_size = Vector2(120, 0)
	if size_mult != 1.0:
		l.add_theme_font_size_override("font_size", maxi(int(get_theme_default_font_size() * size_mult), 10))
	if color != Color.WHITE:
		l.add_theme_color_override("font_color", color)
	return l


func _note(text: String) -> Label:
	var l := _label(text, 0.78)
	l.modulate = DIM
	return l


func _heading(text: String) -> Label:
	return _label(text, 1.12)


func _typing(c: Node) -> void:
	if c is LineEdit:
		c.add_to_group(TYPING)
	for e in c.find_children("*", "LineEdit", true, false):
		e.add_to_group(TYPING)


func _scroll(into: Control) -> VBoxContainer:
	var sc := ScrollContainer.new()
	sc.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	sc.size_flags_vertical = Control.SIZE_EXPAND_FILL
	sc.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	into.add_child(sc)
	var v := VBoxContainer.new()
	v.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	v.add_theme_constant_override("separation", 6)
	sc.add_child(v)
	return v


func _clear(c: Control) -> void:
	for ch in c.get_children():
		c.remove_child(ch)
		ch.queue_free()


func _show(s: String) -> void:
	stage = s
	for k in _pages:
		(_pages[k] as Control).visible = k == s


# ---- page 1: logs, findings, the quad ----

func _build_setup(page: VBoxContainer) -> void:
	var v := _scroll(page)
	v.add_child(_note("Makes a flight model of your own quad from its Betaflight blackbox logs (Betaflight 2025.12 or newer, 4 motors, bidirectional DShot for the rotor speeds). Pick the logs of this quad: the more of what it does the better, with chirp sweeps best. The model is kept in %s." % _models_path()))
	v.add_child(HSeparator.new())
	v.add_child(_heading("Logs"))
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 10)
	var add := Button.new()
	add.name = "AddFiles"
	add.text = " Add log files... "
	add.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	add.tooltip_text = "Pick .bbl, .bfl or .txt blackbox logs (several at once)"
	add.pressed.connect(_pick_files)
	row.add_child(add)
	var hint := _note("All of the same quad; a card's .bbl or .bfl, or an OpenLog .txt. CSV exports do not work.")
	hint.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	hint.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	row.add_child(hint)
	v.add_child(row)
	_files_box = VBoxContainer.new()
	_files_box.name = "Files"
	_files_box.add_theme_constant_override("separation", 4)
	v.add_child(_files_box)
	_msgs = VBoxContainer.new()
	_msgs.name = "Messages"
	_msgs.add_theme_constant_override("separation", 6)
	v.add_child(_msgs)
	_form_box = VBoxContainer.new()
	_form_box.name = "Form"
	_form_box.add_theme_constant_override("separation", 6)
	v.add_child(_form_box)
	_build_form(_form_box)
	var foot := HBoxContainer.new()
	foot.add_theme_constant_override("separation", 12)
	page.add_child(foot)
	_hint = _label("", 0.9)
	_hint.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	foot.add_child(_hint)
	_build = Button.new()
	_build.name = "Build"
	_build.text = "  Build the model  "
	_build.pressed.connect(start_build)
	foot.add_child(_build)


func _models_path() -> String:
	return ProjectSettings.globalize_path(out_root)


func _build_form(f: VBoxContainer) -> void:
	f.add_child(HSeparator.new())
	f.add_child(_heading("The quad"))
	f.add_child(_note("Fields marked \"from the log\" are read from it: check them. Props and pack are optional: leave them empty to use the class's (shown in grey)."))
	_text_row(f, "name", "Name", "", "A name for the model; its folder is made from it.")
	_options_row(f, "pack_type", "Pack type", PACKS.values(), 0, "")
	for spec in FIELDS:
		if spec["key"] == "capacity_mah":
			_options_row(f, "class", "Craft class", CLASS_ORDER.map(func(c): return CLASSES[c]["name"]), 3, "Sets the defaults below; it is guessed from the weight.")
			_class_note = _note("")
			f.add_child(_class_note)
		_num_row(f, spec)
	_options_row(f, "ducted", "Ducted props", ["class default", "yes", "no"], 0, "")
	_options_row(f, "zone_vehicle_id", "Zone drone", ["(none)"], 0, "The game's drone this model looks like and is flown on. You can pick another when it is built.")
	_same_row = HBoxContainer.new()
	_same = CheckBox.new()
	_same.name = "SameQuad"
	_same.text = "These logs are all from this quad (flight controller replaced or renamed)"
	_same_row.add_child(_same)
	f.add_child(_same_row)
	_same.toggled.connect(func(_on): _refresh())
	_overwrite = CheckBox.new()
	_overwrite.name = "Overwrite"
	_overwrite.visible = false
	_overwrite.toggled.connect(func(_on): _validate())
	f.add_child(_overwrite)


func _row(f: VBoxContainer, key: String, label: String, note: String) -> HBoxContainer:
	var box := VBoxContainer.new()
	box.name = "Row_" + key
	box.add_theme_constant_override("separation", 2)
	f.add_child(box)
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 8)
	box.add_child(row)
	var l := Label.new()
	l.name = "Label"
	l.text = label
	l.custom_minimum_size = Vector2(260, 0)
	row.add_child(l)
	return row


func _after_row(row: HBoxContainer, key: String, note: String) -> void:
	var box := row.get_parent() as VBoxContainer
	var e := _label("", 0.85, BAD)
	e.name = "Error"
	e.visible = false
	box.add_child(e)
	_errs[key] = e
	if note != "":
		box.add_child(_note(note))


func _text_row(f: VBoxContainer, key: String, label: String, value: String, note: String) -> void:
	var row := _row(f, key, label, note)
	var e := LineEdit.new()
	e.name = "Field_" + key
	e.text = value
	e.max_length = 40
	e.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	e.custom_minimum_size = Vector2(160, 0)
	e.text_changed.connect(func(_t): _edited(key))
	row.add_child(e)
	_marker(row, key)
	_typing(e)
	_ctl[key] = e
	_after_row(row, key, note)


func _num_row(f: VBoxContainer, spec: Dictionary) -> void:
	var key: String = spec["key"]
	var row := _row(f, key, spec["label"] + (" *" if spec["req"] else ""), spec["note"])
	var e := LineEdit.new()
	e.name = "Field_" + key
	e.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	e.custom_minimum_size = Vector2(160, 0)
	e.text_changed.connect(func(_t): _edited(key))
	row.add_child(e)
	_marker(row, key)
	_typing(e)
	_ctl[key] = e
	_after_row(row, key, spec["note"])


func _options_row(f: VBoxContainer, key: String, label: String, items: Array, sel: int, note: String) -> void:
	var row := _row(f, key, label, note)
	var o := OptionButton.new()
	o.name = "Field_" + key
	o.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	o.fit_to_longest_item = false
	for s in items:
		o.add_item(str(s))
	o.select(sel)
	o.item_selected.connect(func(_i):
		_touched[key] = true
		if key == "class":
			_class_changed()
		_validate())
	row.add_child(o)
	_marker(row, key)
	_ctl[key] = o
	_after_row(row, key, note)


# "from the log, check" beside a prefilled box
func _marker(row: HBoxContainer, key: String) -> void:
	var m := _label("", 0.78, WARN)
	m.name = "Mark"
	m.autowrap_mode = TextServer.AUTOWRAP_OFF
	m.custom_minimum_size = Vector2.ZERO
	row.add_child(m)


func _mark(key: String, text: String) -> void:
	var c: Control = _ctl[key]
	var m := c.get_parent().get_node_or_null("Mark") as Label
	if m != null:
		m.text = text


func _edited(key: String) -> void:
	_touched[key] = true
	_mark(key, "")
	if key == "weight_g":
		_class_from_weight()
	if key == "prop_in":
		_placeholders()
	_validate()


func _class_key() -> String:
	return CLASS_ORDER[(_ctl["class"] as OptionButton).selected]


## The class whose weight band holds the weight (the nearest band otherwise)
static func class_for_weight(g: float) -> String:
	var best := "five"
	var best_d := INF
	for c in CLASS_ORDER:
		var lo: float = CLASSES[c]["w"][0]
		var hi: float = CLASSES[c]["w"][1]
		var d := 0.0 if g >= lo and g <= hi else INF
		# inside a band: the log distance to its middle; outside: to its edge
		var mid := sqrt(lo * hi)
		d = absf(log(g / mid)) if d == 0.0 else 100.0 + absf(log(g / (lo if g < lo else hi)))
		if d < best_d:
			best_d = d
			best = c
	return best


func _class_from_weight() -> void:
	if _touched.get("class", false):
		return
	var w := _num("weight_g")
	if is_nan(w):
		return
	(_ctl["class"] as OptionButton).select(CLASS_ORDER.find(class_for_weight(w)))
	_class_changed(false)


func _class_changed(user := true) -> void:
	var c: Dictionary = CLASSES[_class_key()]
	_class_note.text = str(c["note"])
	_placeholders()
	if not _touched.get("zone_vehicle_id", false) or user:
		_pick_drone(str(c["drone"]), false)


func _placeholders() -> void:
	var c: Dictionary = CLASSES[_class_key()]
	var d: float = c["d"]
	var pd := _num("prop_in")
	var dd := pd if not is_nan(pd) else d
	(_ctl["capacity_mah"] as LineEdit).placeholder_text = "%d (class default)" % int(c["mah"])
	(_ctl["prop_in"] as LineEdit).placeholder_text = "%s (class default)" % String.num(d, 1)
	(_ctl["pitch_in"] as LineEdit).placeholder_text = "%s (class default)" % String.num(c["pitch"], 1)
	(_ctl["blades"] as LineEdit).placeholder_text = "%d (class default)" % int(c["blades"])
	(_ctl["wheelbase_mm"] as LineEdit).placeholder_text = "%d (from the prop size)" % int(round(80.0 * dd / 5.1 * 2.0 * sqrt(2.0)))
	(_ctl["top_speed_kmh"] as LineEdit).placeholder_text = "(not given)"
	(_ctl["weight_g"] as LineEdit).placeholder_text = "required"
	(_ctl["name"] as LineEdit).placeholder_text = "required"
	(_ctl["kv"] as LineEdit).placeholder_text = "required"
	(_ctl["cells"] as LineEdit).placeholder_text = "required"


func _num(key: String) -> float:
	var t := (_ctl[key] as LineEdit).text.strip_edges()
	return t.to_float() if t.is_valid_float() else NAN


func _fill_drones() -> void:
	var o: OptionButton = _ctl["zone_vehicle_id"]
	var keep := _drone_id()
	o.clear()
	o.add_item("(none)")
	o.set_item_metadata(0, "")
	for d in _drones:
		o.add_item("%s  (%s)" % [d["name"], d["id"]] if str(d["name"]) != str(d["id"]) else str(d["id"]))
		o.set_item_metadata(o.item_count - 1, str(d["id"]))
	if keep != "" or _touched.get("zone_vehicle_id", false):
		_pick_drone(keep, false)
	else:
		_pick_drone(str(CLASSES[_class_key()]["drone"]), false)


func _drone_id() -> String:
	var o: OptionButton = _ctl["zone_vehicle_id"]
	return str(o.get_item_metadata(o.selected)) if o.selected >= 0 and o.item_count > 0 and o.get_item_metadata(o.selected) != null else ""


func _pick_drone(id: String, touched: bool) -> void:
	var o: OptionButton = _ctl["zone_vehicle_id"]
	for i in o.item_count:
		if str(o.get_item_metadata(i)) == id:
			o.select(i)
			if touched:
				_touched["zone_vehicle_id"] = true
			return
	o.select(0)


# ---- files ----

func _pick_files() -> void:
	if _dialog == null:
		_dialog = FileDialog.new()
		_dialog.name = "FileDialog"
		_dialog.process_mode = Node.PROCESS_MODE_ALWAYS
		_dialog.file_mode = FileDialog.FILE_MODE_OPEN_FILES
		_dialog.access = FileDialog.ACCESS_FILESYSTEM
		_dialog.use_native_dialog = false
		_dialog.title = "Pick blackbox logs"
		_dialog.add_filter("*.bbl, *.bfl, *.txt, *.BBL, *.BFL, *.TXT", "Blackbox logs")
		_dialog.files_selected.connect(add_files)
		add_child(_dialog)
		_typing(_dialog)
	var last := str(_cfg().get_value("logfit", "last_dir", ""))
	if last != "" and DirAccess.dir_exists_absolute(last):
		_dialog.current_dir = last
	var vs := get_viewport().get_visible_rect().size
	_dialog.popup(Rect2i(Vector2i(vs * 0.12), Vector2i(vs * 0.76)))


func _cfg() -> ConfigFile:
	var c := ConfigFile.new()
	c.load(cfg_path)
	return c


## Adds logs to the list (a file already there is not added twice) and reads them
func add_files(picked: PackedStringArray) -> void:
	var added := false
	for p in picked:
		if p.get_extension().to_lower() in EXTS and not p in paths:
			paths.append(p)
			added = true
	if not picked.is_empty():
		var c := _cfg()
		c.set_value("logfit", "last_dir", picked[0].get_base_dir())
		DirAccess.make_dir_recursive_absolute(cfg_path.get_base_dir())
		c.save(cfg_path)
	if added:
		_inspect()
	_refresh()


func remove_file(p: String) -> void:
	var i := paths.find(p)
	if i >= 0:
		paths.remove_at(i)
	if paths.is_empty():
		info = {}
		_refresh()
	else:
		_inspect()
	_refresh()


# ---- jobs (a Thread, polled each frame) ----

func _script() -> Script:
	if logfit_script != null:
		return logfit_script
	if FileAccess.file_exists(LOGFIT) or ResourceLoader.exists(LOGFIT):
		var s = load(LOGFIT)
		if s is Script:
			return s
	return null


func available() -> bool:
	return _script() != null


func busy() -> bool:
	return _thread != null


func _start(kind: String, fn: Callable) -> void:
	_kind = kind
	_after = [] if kind == "build" else _after
	_done = false
	_answer = null
	_cancelled = false
	_t0 = Time.get_ticks_msec()
	_prog = {"step": "", "frac": 0.0}
	_thread = Thread.new()
	_thread.start(func():
		var r = fn.call()
		_mutex.lock()
		_answer = r
		_done = true
		_mutex.unlock(), Thread.PRIORITY_LOW)
	set_process(true)


func _inspect() -> void:
	if busy():
		_again = true
		return
	var s := _script()
	if s == null:
		return
	if _logfit == null:
		_logfit = s.new()
	var ps := paths.duplicate()
	_start("inspect", func(): return _logfit.inspect(ps))


func _on_progress(step: String, frac: float) -> void:
	_mutex.lock()
	_prog = {"step": step, "frac": frac}
	_mutex.unlock()


func _process(_d: float) -> void:
	if hide_when_unpaused and visible and _was_paused and not _paused():
		close()
	if _thread == null:
		set_process(false)
		return
	_mutex.lock()
	var done := _done
	var prog := _prog.duplicate()
	var ans = _answer
	_mutex.unlock()
	if stage == "working":
		_step.text = str(prog["step"]) if str(prog["step"]) != "" else "Starting..."
		_bar.value = clampf(float(prog["frac"]), 0.0, 1.0) * 100.0
		_elapsed.text = "%d s" % ((Time.get_ticks_msec() - _t0) / 1000)
	if not done:
		return
	_thread.wait_to_finish()
	_thread = null
	set_process(false)
	var kind := _kind
	_kind = ""
	if kind == "inspect":
		info = ans if ans is Dictionary else {"errors": ["The log reader gave no answer."]}
		if _again:
			_again = false
			_inspect()
		_prefill()
	else:
		_built(ans if ans is Dictionary else {"ok": false, "errors": ["The build gave no answer."]})
	_refresh()


# ---- what inspect found ----

## An error or warning as {text, fix, big}
static func message(m) -> Dictionary:
	if m is Dictionary:
		var t := str(m.get("text", m.get("message", m.get("msg", m.get("error", "")))))
		return {"text": t, "fix": str(m.get("fix", "")), "big": bool(m.get("big", false)) or str(m.get("severity", "")) == "big",
			"overridable": bool(m.get("overridable", false))}
	return {"text": str(m), "fix": "", "big": false, "overridable": false}


static func _kind_of(text: String) -> String:
	var t := text.to_lower()
	if t.contains("chirp"):
		return "chirp"
	if t.contains("accel") or t.contains("accsmooth") or t.contains(" acc ") or t.begins_with("acc ") or t.contains("(acc)"):
		return "acc"
	if t.contains("vbat") or t.contains("battery voltage"):
		return "vbat"
	return ""


## [{text, fix, tag}]: the missing chirp, accelerometer and battery voltage as banners
func banners() -> Array:
	var out := []
	var tags := {}
	for w in info.get("warnings", []):
		var m := message(w)
		var tag := _kind_of(m["text"])
		if m["big"] or tag != "":
			m["tag"] = tag
			out.append(m)
			tags[tag] = true
	var fields: Array = info.get("fields", [])
	if not fields.is_empty():
		if not tags.has("acc") and not fields.any(func(f): return str(f).begins_with("accSmooth")):
			out.append({"text": "No accelerometer in the logs: hover comes from a rougher rule and the drag is not measured. The model is capped at grade C.", "fix": "set blackbox_disable_acc = OFF", "tag": "acc", "big": true})
		if not tags.has("vbat") and not fields.any(func(f): return str(f).begins_with("vbatLatest")):
			out.append({"text": "No battery voltage in the logs: the sag is fitted more weakly. The model is capped at grade B.", "fix": "set blackbox_disable_bat = OFF", "tag": "vbat", "big": true})
	return out


## The errors that stop a build (an override can lift the ones that say so)
func blocking() -> Array:
	var out := []
	for e in info.get("errors", []):
		var m := message(e)
		if m["overridable"] and _same != null and _same.button_pressed:
			continue
		out.append(m)
	return out


func _prefill() -> void:
	var p: Dictionary = info.get("prefill", {})
	var cells = p.get("cells")
	var kv = p.get("kv")
	_prefill_num("cells", cells)
	_prefill_num("kv", kv)
	var nm := str(p.get("craft_name", "")).strip_edges()
	if nm == "" and not paths.is_empty():
		nm = paths[0].get_file().get_basename()
	if nm != "" and not _touched.get("name", false):
		(_ctl["name"] as LineEdit).text = nm.substr(0, 40)
		_mark("name", "from the log, check")
	if p.has("class") and not _touched.get("class", false) and str(p["class"]) in CLASS_ORDER:
		(_ctl["class"] as OptionButton).select(CLASS_ORDER.find(str(p["class"])))
		_class_changed(false)
	if p.has("pack_type") and not _touched.get("pack_type", false):
		var i := PACKS.keys().find(str(p["pack_type"]))
		if i >= 0:
			(_ctl["pack_type"] as OptionButton).select(i)
	_class_changed(false)


# a number, or {value, check: true|false, note}
func _prefill_num(key: String, v) -> void:
	if v == null or _touched.get(key, false):
		return
	var note := ""
	var val = v
	if v is Dictionary:
		val = v.get("value")
		note = str(v.get("note", "check")) if bool(v.get("check", false)) else ""
	if val == null:
		return
	(_ctl[key] as LineEdit).text = String.num(float(val), 0) if float(val) == floorf(float(val)) else String.num(float(val), 3)
	_mark(key, "from the log" + (", " + note if note != "" else ", check"))


# ---- redraw ----

func _refresh() -> void:
	if _files_box == null:
		return
	if host != null and host.has_method("game_drones") and _drones.is_empty():
		_drones = host.game_drones()
		_fill_drones()
	_clear(_files_box)
	_clear(_msgs)
	if not available():
		_msgs.add_child(_label("The model builder is not installed completely: real_quad/logfit/logfit.gd is missing. Update the mod.", 1.0, BAD))
	if paths.is_empty():
		_files_box.add_child(_note("No logs yet."))
	var sessions: Array = info.get("sessions", [])
	for p in paths:
		var row := HBoxContainer.new()
		row.name = "File_" + p.get_file().validate_node_name()
		row.add_theme_constant_override("separation", 8)
		var n := _label(p.get_file())
		n.autowrap_mode = TextServer.AUTOWRAP_OFF
		n.clip_text = true
		n.tooltip_text = p
		n.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		row.add_child(n)
		var rm := Button.new()
		rm.name = "Remove"
		rm.text = " Remove "
		rm.pressed.connect(remove_file.bind(p))
		row.add_child(rm)
		_files_box.add_child(row)
		for s in sessions:
			if s is Dictionary and str(s.get("file", "")).get_file() == p.get_file():
				_files_box.add_child(_session_line(s))
	if busy() and _kind == "inspect":
		_files_box.add_child(_note("Reading the logs..."))
	elif not info.is_empty():
		var ok := 0
		var secs := 0.0
		for s in sessions:
			if s is Dictionary and bool(s.get("usable", s.get("used", true))):
				ok += 1
				secs += float(s.get("armed_s", s.get("flight_s", s.get("duration_s", 0.0))))
		var fw := str(info.get("firmware", ""))
		_files_box.add_child(_label("%d session%s usable%s%s" % [ok, "" if ok == 1 else "s",
			", %.1f min of flight" % (secs / 60.0) if secs > 0.0 else "", ", " + fw if fw != "" else ""], 0.95, GROUP))
	for m in _after:
		_msgs.add_child(_message_box(m, m["color"], m["prefix"]))
	var blocked := blocking()
	for m in blocked:
		_msgs.add_child(_message_box(m, BAD, "Cannot build: "))
	for b in banners():
		_msgs.add_child(_message_box(b, WARN, "WARNING: "))
	for w in info.get("warnings", []):
		var m := message(w)
		if _kind_of(m["text"]) == "" and not m["big"]:
			_msgs.add_child(_message_box(m, DIM, ""))
	if _same_row != null:
		var any_over := false
		for e in info.get("errors", []):
			any_over = any_over or message(e)["overridable"]
		_same_row.visible = paths.size() > 1 or any_over
	_form_box.visible = not paths.is_empty() and not info.is_empty()
	_validate()


func _session_line(s: Dictionary) -> Control:
	var usable := bool(s.get("usable", s.get("used", true)))
	var reason := str(s.get("reason", s.get("note", "")))
	var dur := float(s.get("armed_s", s.get("flight_s", s.get("duration_s", -1.0))))
	var t := "   session %s" % str(s.get("index", s.get("session", "")))
	if dur >= 0.0:
		t += ", %d s armed" % int(round(dur))
	t += ": " + ("ok" if usable else "skipped") + (" - " + reason if reason != "" else "")
	var l := _label(t, 0.88, (GOOD if reason == "" else WARN) if usable else BAD)
	return l


func _message_box(m: Dictionary, color: Color, prefix: String) -> Control:
	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", 2)
	var l := _label(prefix + m["text"], 1.0, color)
	if prefix == "WARNING: ":
		l.add_theme_font_size_override("font_size", int(get_theme_default_font_size() * 1.08))
	v.add_child(l)
	var fix := str(m["fix"])
	if fix != "":
		var r := HBoxContainer.new()
		r.add_theme_constant_override("separation", 8)
		var f := _label("Fix: " + fix, 0.9)
		f.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		r.add_child(f)
		if fix.contains("set "):
			var cp := Button.new()
			cp.name = "CopyCLI"
			cp.text = " Copy CLI "
			cp.tooltip_text = "Puts the commands on the clipboard: paste them into Betaflight Configurator's CLI"
			cp.pressed.connect(func(): DisplayServer.clipboard_set(fix))
			r.add_child(cp)
		v.add_child(r)
	return v


# ---- the form ----

## Checks the boxes: {form, errors {key: text}}; shows the errors and sets Build
func _validate() -> Dictionary:
	var errors := {}
	var form := {}
	if _ctl.is_empty():
		return {"form": form, "errors": errors}
	var nm := (_ctl["name"] as LineEdit).text.strip_edges()
	var id := slug(nm)
	if nm == "":
		errors["name"] = "required"
	elif id == "":
		errors["name"] = "needs some letters or digits"
	else:
		form["name"] = nm
	for spec in FIELDS:
		var key: String = spec["key"]
		var t := (_ctl[key] as LineEdit).text.strip_edges()
		if t == "":
			if spec["req"]:
				errors[key] = "required"
			continue
		if not t.is_valid_float():
			errors[key] = "not a number"
			continue
		var v := t.to_float()
		if spec["int"] and v != floorf(v):
			errors[key] = "a whole number"
		elif v < float(spec["min"]) or v > float(spec["max"]):
			errors[key] = "%s to %s" % [String.num(spec["min"], 1).trim_suffix(".0"), String.num(spec["max"], 1).trim_suffix(".0")]
		else:
			form[key] = int(v) if spec["int"] else v
	form["pack_type"] = PACKS.keys()[(_ctl["pack_type"] as OptionButton).selected]
	form["class"] = _class_key()
	var duct := (_ctl["ducted"] as OptionButton).selected
	if duct > 0:
		form["ducted"] = duct == 1
	form["zone_vehicle_id"] = _drone_id()
	if _same != null and _same.button_pressed:
		form["same_quad"] = true
	# the id must be free: a shipped model's is not, an own one asks first
	var origin: String = host.model_origin(id) if id != "" and host != null and host.has_method("model_origin") else ""
	if origin == "shipped":
		errors["name"] = "\"%s\" is a model that comes with real_quad: pick another name (%s_2?)" % [id, id]
	_overwrite.visible = origin == "user"
	if origin == "user":
		_overwrite.text = "A model \"%s\" of yours exists: replace it" % id
		if not _overwrite.button_pressed:
			errors["name"] = "a model of yours has this name: tick \"replace it\" below, or pick another name"
		else:
			form["overwrite"] = true
	for key in _errs:
		var l: Label = _errs[key]
		l.text = str(errors.get(key, ""))
		l.visible = errors.has(key)
	var why := ""
	if not blocking().is_empty():
		why = "Fix the problems above (or take other logs) to build."
	elif paths.is_empty():
		why = "Add the logs of your quad first."
	elif info.is_empty():
		why = "Reading the logs..."
	elif not errors.is_empty():
		why = "Fill in: " + ", ".join(PackedStringArray(errors.keys().map(func(k): return _label_of(k))))
	elif not available():
		why = "The model builder is not installed completely."
	if _build != null:
		_build.disabled = why != "" or busy()
		_hint.text = why
	return {"form": form, "errors": errors, "id": id}


func _label_of(key: String) -> String:
	for spec in FIELDS:
		if spec["key"] == key:
			return str(spec["label"]).get_slice(" (", 0)
	return "the name" if key == "name" else key


## A name as a folder name: [a-z0-9_], at most 32 characters
static func slug(s: String) -> String:
	var out := ""
	for ch in s.to_lower():
		out += ch if (ch >= "a" and ch <= "z") or (ch >= "0" and ch <= "9") else "_"
	while out.contains("__"):
		out = out.replace("__", "_")
	out = out.strip_edges().trim_prefix("_").trim_suffix("_")
	return out.substr(0, 32).trim_suffix("_")


# ---- building ----

func _build_working(page: VBoxContainer) -> void:
	page.add_child(_heading("Building the model"))
	_step = _label("", 1.12)
	_step.name = "Step"
	page.add_child(_step)
	_bar = ProgressBar.new()
	_bar.name = "Progress"
	_bar.min_value = 0.0
	_bar.max_value = 100.0
	_bar.custom_minimum_size = Vector2(0, 22)
	page.add_child(_bar)
	_elapsed = _note("")
	_elapsed.name = "Elapsed"
	page.add_child(_elapsed)
	page.add_child(_note("This takes a few minutes. The game goes on behind it, and may run slower on a weak computer."))
	var c := Button.new()
	c.name = "Cancel"
	c.text = "  Cancel  "
	c.pressed.connect(cancel)
	var row := HBoxContainer.new()
	row.add_child(c)
	page.add_child(row)


## Builds with what the form says; false when the form or the logs do not allow it
func start_build() -> bool:
	var v := _validate()
	if not v["errors"].is_empty() or not blocking().is_empty() or busy() or not available():
		return false
	if _logfit == null:
		_logfit = _script().new()
	var id: String = v["id"]
	var form: Dictionary = v["form"]
	var out := out_root.path_join(id)
	var ps := paths.duplicate()
	var lf = _logfit
	_start("build", func(): return lf.build(ps, form, out, _on_progress))
	_step.text = "Starting..."
	_bar.value = 0.0
	_elapsed.text = ""
	_show("working")
	return true


func cancel() -> void:
	if busy() and _kind == "build":
		_cancelled = true
		_logfit.cancel()
		_step.text = "Cancelling..."


func _built(r: Dictionary) -> void:
	result = r
	_after = []
	if _cancelled:
		_after.append({"text": "Cancelled.", "fix": "", "color": DIM, "prefix": ""})
		_show("setup")
		return
	if not bool(r.get("ok", false)):
		for e in r.get("errors", []):
			var m := message(e)
			m["color"] = BAD
			m["prefix"] = "The build failed: "
			_after.append(m)
		if _after.is_empty():
			_after.append({"text": "The build failed.", "fix": "", "color": BAD, "prefix": ""})
		_show("setup")
		return
	_show_result()


# ---- the result ----

func _build_result(page: VBoxContainer) -> void:
	var v := _scroll(page)
	_res_box = v
	var foot := HBoxContainer.new()
	foot.add_theme_constant_override("separation", 10)
	page.add_child(foot)
	var l := Label.new()
	l.text = "Fly it on"
	foot.add_child(l)
	_use_pick = OptionButton.new()
	_use_pick.name = "Drone"
	_use_pick.custom_minimum_size = Vector2(220, 0)
	_use_pick.fit_to_longest_item = false
	_use_pick.item_selected.connect(func(_i): _use_label())
	foot.add_child(_use_pick)
	_use = Button.new()
	_use.name = "UseFor"
	_use.pressed.connect(use_for_drone)
	foot.add_child(_use)
	_use_note = _label("", 0.9)
	_use_note.name = "UseNote"
	_use_note.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	foot.add_child(_use_note)
	var folder := Button.new()
	folder.name = "OpenFolder"
	folder.text = " Open folder "
	folder.pressed.connect(func(): OS.shell_open(ProjectSettings.globalize_path(str(result.get("package_dir", out_root)))))
	foot.add_child(folder)
	var back := Button.new()
	back.name = "Another"
	back.text = " Another model "
	back.tooltip_text = "Back to the logs and the form"
	back.pressed.connect(func():
		_show("setup")
		_refresh())
	foot.add_child(back)
	var c := Button.new()
	c.name = "CloseResult"
	c.text = "  Close  "
	c.pressed.connect(close)
	foot.add_child(c)


func _show_result() -> void:
	_clear(_res_box)
	var grade := str(result.get("grade", "?"))
	var head := HBoxContainer.new()
	head.add_theme_constant_override("separation", 16)
	var g := Label.new()
	g.name = "Grade"
	g.text = grade
	g.add_theme_font_size_override("font_size", int(get_theme_default_font_size() * 3.2))
	g.add_theme_color_override("font_color", GOOD if grade in ["A", "B"] else (WARN if grade in ["C", "D"] else BAD))
	head.add_child(g)
	var side := VBoxContainer.new()
	side.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	side.add_child(_label("%s - %s" % [str(result.get("model_id", "")), MEANING.get(grade, "")], 1.3))
	side.add_child(_note("Kept in " + ProjectSettings.globalize_path(str(result.get("package_dir", out_root)))))
	head.add_child(side)
	_res_box.add_child(head)
	var seen := {}
	for w in result.get("warnings", []):
		var m := message(w)
		var big: bool = m["big"] or _kind_of(m["text"]) != ""
		seen[m["text"]] = true
		_res_box.add_child(_message_box(m, WARN if big else DIM, "WARNING: " if big else ""))
	_res_box.add_child(HSeparator.new())
	_report = RichTextLabel.new()
	_report.name = "Report"
	_report.bbcode_enabled = true
	_report.fit_content = true
	_report.scroll_active = false
	_report.selection_enabled = true
	_report.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_report.text = md_to_bbcode(str(result.get("report_md", "")), get_theme_default_font_size())
	_res_box.add_child(_report)
	_use_pick.clear()
	_use_pick.add_item("(none)")
	_use_pick.set_item_metadata(0, "")
	for d in _drones:
		_use_pick.add_item(str(d["name"]))
		_use_pick.set_item_metadata(_use_pick.item_count - 1, str(d["id"]))
	var want := _drone_id()
	for i in _use_pick.item_count:
		if str(_use_pick.get_item_metadata(i)) == want:
			_use_pick.select(i)
	_use_note.text = ""
	_use_label()
	_show("result")


func _use_label() -> void:
	var id := str(_use_pick.get_item_metadata(_use_pick.selected)) if _use_pick.selected >= 0 else ""
	_use.disabled = id == ""
	_use.text = " Use for %s " % _use_pick.get_item_text(_use_pick.selected) if id != "" else " Use for a drone "


## Writes model=<id> into the picked drone's settings and has real_quad take it
func use_for_drone() -> void:
	var vid := str(_use_pick.get_item_metadata(_use_pick.selected))
	var mid := str(result.get("model_id", ""))
	if vid == "" or mid == "" or host == null:
		return
	var err: String = host.set_drone_model(vid, mid)
	_use_note.text = err if err != "" else "Set: %s flies the %s model now." % [_use_pick.get_item_text(_use_pick.selected), mid]
	_use_note.add_theme_color_override("font_color", BAD if err != "" else GOOD)


# ---- the report: minimal Markdown to BBCode ----

## Headings, **bold**, `code`, lists, quotes and tables for a RichTextLabel
static func md_to_bbcode(md: String, base := 16) -> String:
	var lines := md.replace("\r", "").split("\n")
	var out := PackedStringArray()
	var i := 0
	while i < lines.size():
		var line := lines[i]
		var st := line.strip_edges()
		if st.begins_with("```"):
			var block := PackedStringArray()
			i += 1
			while i < lines.size() and not lines[i].strip_edges().begins_with("```"):
				block.append(lines[i])
				i += 1
			out.append("[code]%s[/code]" % _esc("\n".join(block)))
		elif st.begins_with("|"):
			var rows := []
			while i < lines.size() and lines[i].strip_edges().begins_with("|"):
				var cells := lines[i].strip_edges().trim_prefix("|").trim_suffix("|").split("|")
				var plain := PackedStringArray()
				for c in cells:
					plain.append(c.strip_edges().replace("**", "").replace("`", ""))
				if _is_rule(plain):
					rows.append(null)
				else:
					rows.append(plain)
				i += 1
			i -= 1
			out.append(_table(rows))
		elif st.begins_with("#"):
			var lvl := 0
			while lvl < st.length() and st[lvl] == "#":
				lvl += 1
			var mult: float = [1.0, 1.5, 1.3, 1.15, 1.05, 1.0, 1.0][mini(lvl, 6)]
			out.append("[font_size=%d][b]%s[/b][/font_size]" % [int(base * mult), _inline(st.substr(lvl).strip_edges())])
		elif st.begins_with("- ") or st.begins_with("* "):
			var indent := line.length() - line.strip_edges(true, false).length()
			out.append("%s• %s" % ["  ".repeat(indent / 2 + 1), _inline(st.substr(2))])
		elif st.begins_with("> "):
			out.append("[i]%s[/i]" % _inline(st.substr(2)))
		elif st == "---" or st == "***":
			out.append("────────────────")
		else:
			out.append(_inline(line))
		i += 1
	return "\n".join(out)


static func _esc(s: String) -> String:
	return s.replace("[", "[lb]")


static func _is_rule(cells: PackedStringArray) -> bool:
	for c in cells:
		var t := c.strip_edges()
		if t == "" or t.replace("-", "").replace(":", "") != "":
			return false
	return true


static func _inline(s: String) -> String:
	s = _esc(s)
	var bold := RegEx.create_from_string("\\*\\*(.+?)\\*\\*")
	var code := RegEx.create_from_string("`([^`]+)`")
	s = bold.sub(s, "[b]$1[/b]", true)
	s = code.sub(s, "[code]$1[/code]", true)
	return s


static func _table(rows: Array) -> String:
	var cols := 0
	var head := rows.find(null)         # the rows above the rule are the header
	for r in rows:
		if r != null:
			cols = maxi(cols, r.size())
	var out := PackedStringArray(["[table=%d]" % maxi(cols, 1)])
	for n in rows.size():
		if rows[n] == null:
			continue
		for c in cols:
			var t: String = _esc(rows[n][c]) if c < rows[n].size() else ""
			out.append("[cell padding=0,1,14,1]%s[/cell]" % ("[b]%s[/b]" % t if n < head and t != "" else t))
	out.append("[/table]")
	return "".join(out)
