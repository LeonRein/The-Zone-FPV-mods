extends Control
## real_quad: the model manager window - every flight model (shipped and yours), the build that is running,
## a model's report, logs and parameters, and "New model from logs". One panel over the pause menu in
## mod_settings' look (its colours, fonts and sizes), made in code. Three pages:
##   overview  the models (grade, name, source, the drones that fly it, disk use), the running build with
##             its progress, "New model from logs"
##   new       pick Betaflight blackbox logs, read what is in them, say what the quad is, build
##   detail    one model: report, logs (add / remove), parameters (the form), fly it on a drone, duplicate,
##             delete (yours only, asks first), open the folder; a model that comes with real_quad is
##             read-only ("Duplicate and edit")
## The window holds no job: builds belong to the mod's model manager (logfit/model_manager.gd, host.manager),
## so closing and reopening the window keeps them and the pages show their live progress. Only the reading
## of logs on the "new" page (inspect) runs here, in a Thread polled each frame (so it works while the game
## is paused):
##     inspect(paths: PackedStringArray) -> Dictionary
##         {firmware, sessions, fields, prefill {cells, kv, motor_poles, craft_name}, errors, warnings}
## An error or warning is a String, or a Dictionary {text, fix} (the fix is a line or a CLI block);
## "big": true marks a banner. The form it gives the manager: name, weight_g, cells, pack_type (lipo | lihv |
## liion), kv, class (whoop | micro | mini | five | seven | ten), zone_vehicle_id and, when filled in,
## capacity_mah, prop_in, pitch_in, blades, wheelbase_mm, top_speed_kmh, ducted, same_quad (the confirmation).
## host is real_quad's main.gd: manager, game_drones(), set_drone_model(). Typing goes to the boxes, not the
## game: the boxes are in the group "zm_typing", which mod_settings' key guard keeps the game's keys away from.

signal closed

const DIM := Color(1, 1, 1, 0.62)
const WARN := Color(1.0, 0.78, 0.35)
const GOOD := Color(0.55, 0.9, 0.55)
const BAD := Color(1.0, 0.45, 0.4)
const GROUP := Color(0.75, 0.85, 1.0)
const _MM := preload("model_manager.gd")
const LOGFIT := "res://real_quad/logfit/logfit.gd"
const BUILD := "res://real_quad/logfit/fit/build.gd"
const CFG := "user://real_quad/logfit.cfg"
const EXTS := ["bbl", "bfl", "txt"]
const TYPING := "zm_typing"
const CONFIRM_TEXT := "All logs were flown with the same weight, props and battery"
const INPUTS_UNKNOWN := "Inputs unknown - add logs to rebuild"
const STALE_HINT := "Prop wash calibrated by another version of dirty_air or real_quad - rebuild to update."

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
		"note": "Sets the body drag and so the top speed: the logs can rarely tell it (long fast straight passes are needed). Empty = the class default. Rebuild to apply."},
]

var host = null                       # real_quad's main.gd
var mm = null                         # the model manager (host.manager)
var logfit_script: Script = null      # logfit.gd for reading the logs, or a stand-in
var cfg_path := CFG
var hide_when_unpaused := false       # set by real_quad when opened from the pause menu
var paths := PackedStringArray()      # the logs of the new model
var info := {}                        # inspect()'s answer for paths
var stage := "overview"               # overview | new | detail
var detail_id := ""
var detail := {}                      # manager.details(detail_id)
var pend_add := PackedStringArray()   # logs to add to the model in the detail page
var skipped := PackedStringArray()    # picked files that are not blackbox logs (left out, named once)
var unreadable := PackedStringArray()  # picked files that cannot be read (the error texts, named once)
var pend_rm := {}                     # stored log name -> true: to remove

var _logfit = null
var _thread: Thread = null
var _mutex := Mutex.new()
var _answer = null
var _done := false
var _again := false                   # the files changed while inspecting
var _was_paused := false
var _touched := {}                    # form key -> the player has set it
var _ctl := {}                        # form key -> its control
var _errs := {}                       # form key -> its error label
var _mode := "new"                    # what the form is for: new | edit
var _baseline := {}                   # the form as the detail page loaded it
var _drones: Array = []               # [{id, name}]
var _dialog: FileDialog = null
var _pick_for := "new"
var _box: PanelContainer = null
var _title: Label = null
var _pages := {}
var _ov_job: VBoxContainer = null
var _ov_list: VBoxContainer = null
var _job_step: Label = null
var _job_bar: ProgressBar = null
var _job_elapsed: Label = null
var _seen_last = null
var _files_box: VBoxContainer = null
var _msgs: VBoxContainer = null
var _form_box: VBoxContainer = null
var _new_holder: VBoxContainer = null
var _hint: Label = null
var _build: Button = null
var _class_note: Label = null
var _confirm: CheckBox = null
var _d_head: HBoxContainer = null
var _d_job: VBoxContainer = null
var _d_actions: VBoxContainer = null
var _tabs: TabContainer = null
var _d_report: VBoxContainer = null
var _d_logs: VBoxContainer = null
var _d_params: VBoxContainer = null
var _d_hint: Label = null
var _d_foot: HBoxContainer = null
var _d_step: Label = null
var _d_bar: ProgressBar = null
var _use_pick: OptionButton = null
var _use_note: Label = null
var _confirming_delete := false


func _ready() -> void:
	process_mode = Node.PROCESS_MODE_ALWAYS
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
	_title.text = "Models"
	_title.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_title.add_theme_font_size_override("font_size", int(base * 1.3))
	bar.add_child(_title)
	var x := Button.new()
	x.name = "Close"
	x.text = " x "
	x.tooltip_text = "Close (a build keeps running)"
	x.pressed.connect(close)
	bar.add_child(x)
	for s in ["overview", "new", "detail"]:
		var p := VBoxContainer.new()
		p.name = s.capitalize()
		p.size_flags_vertical = Control.SIZE_EXPAND_FILL
		p.add_theme_constant_override("separation", 8)
		col.add_child(p)
		_pages[s] = p
	_build_overview(_pages["overview"])
	_build_new(_pages["new"])
	_build_detail(_pages["detail"])
	get_viewport().size_changed.connect(_size)
	_size()
	_go("overview")


func _exit_tree() -> void:
	shutdown()


## Stops reading the logs and waits for the thread (the window is closed for good, or the game quits)
func shutdown() -> void:
	if _thread != null:
		if _logfit != null:
			_logfit.cancel()
		_thread.wait_to_finish()
		_thread = null


## Opens the window on the overview
func open() -> void:
	_was_paused = _paused()
	if host != null and mm == null:
		mm = host.manager
	_drones = host.game_drones() if host != null and host.has_method("game_drones") else []
	_fill_drones()
	visible = true
	set_process(true)
	_size()
	_go("overview")


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



func _go(s: String) -> void:
	stage = s
	for k in _pages:
		(_pages[k] as Control).visible = k == s
	match s:
		"overview":
			_title.text = "Models"
			_move_form(_new_holder)
			_refresh_overview()
		"new":
			_title.text = "New model from logs"
			if _mode != "new":
				_reset_new()
			_move_form(_new_holder)
			_refresh()
		"detail":
			_title.text = str(detail.get("name", detail_id))


func _card() -> PanelContainer:
	var pc := PanelContainer.new()
	var sb := StyleBoxFlat.new()
	sb.bg_color = Color(1, 1, 1, 0.05)
	sb.set_corner_radius_all(4)
	sb.set_content_margin_all(8)
	pc.add_theme_stylebox_override("panel", sb)
	return pc


func _grade_color(g: String) -> Color:
	return GOOD if g in ["A", "B"] else (WARN if g in ["C", "D"] else DIM)


func _btn(bname: String, text: String, fn: Callable, tip := "") -> Button:
	var b := Button.new()
	b.name = bname
	b.text = text
	b.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	if tip != "":
		b.tooltip_text = tip
	b.pressed.connect(fn)
	return b


# ---- page 1: the overview ----

func _build_overview(page: VBoxContainer) -> void:
	var v := _scroll(page)
	v.add_child(_note("The flight models real_quad can fly: the ones that come with it and the ones you make from your own Betaflight blackbox logs (kept in %s). Open one to see how good it is, what to fly next, its logs and parameters." % ProjectSettings.globalize_path(_root())))
	_ov_job = VBoxContainer.new()
	_ov_job.name = "Job"
	_ov_job.add_theme_constant_override("separation", 6)
	v.add_child(_ov_job)
	_ov_list = VBoxContainer.new()
	_ov_list.name = "List"
	_ov_list.add_theme_constant_override("separation", 6)
	v.add_child(_ov_list)
	var foot := HBoxContainer.new()
	page.add_child(foot)
	foot.add_child(_btn("NewModel", "  New model from logs  ", func(): _go("new")))


func _root() -> String:
	return mm.root if mm != null else "user://real_quad/models"


func _refresh_overview() -> void:
	if _ov_list == null or mm == null:
		return
	_clear(_ov_job)
	_clear(_ov_list)
	_job_step = null
	_job_bar = null
	_job_elapsed = null
	if mm.busy():
		var c: Dictionary = mm.current()
		var card := _card()
		card.name = "JobCard"
		var h := HBoxContainer.new()
		h.add_theme_constant_override("separation", 12)
		card.add_child(h)
		var side := VBoxContainer.new()
		side.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		side.add_child(_label("Building %s" % c.name, 1.1, GROUP))
		_job_step = _label("", 0.95)
		_job_step.name = "JobStep"
		side.add_child(_job_step)
		_job_bar = ProgressBar.new()
		_job_bar.name = "JobBar"
		_job_bar.min_value = 0.0
		_job_bar.max_value = 100.0
		_job_bar.custom_minimum_size = Vector2(0, 20)
		side.add_child(_job_bar)
		_job_elapsed = _note("")
		_job_elapsed.name = "JobElapsed"
		side.add_child(_job_elapsed)
		h.add_child(side)
		if not bool(c.new):
			h.add_child(_btn("JobDetails", " Details ", show_detail.bind(str(c.id))))
		h.add_child(_btn("Cancel", "  Cancel  ", func(): mm.cancel(), "Stops the build; nothing is changed"))
		_ov_job.add_child(card)
		_job_update()
	if not mm.last.is_empty():
		_ov_job.add_child(_last_card())
	var rows: Array = mm.list()
	if rows.is_empty():
		_ov_list.add_child(_note("No models."))
	for r in rows:
		_ov_list.add_child(_model_card(r))
	_seen_last = mm.last


func _last_card() -> Control:
	var rec: Dictionary = mm.last
	var ok := bool(rec.get("ok", false))
	var card := _card()
	card.name = "LastCard"
	var h := HBoxContainer.new()
	h.add_theme_constant_override("separation", 12)
	card.add_child(h)
	var side := VBoxContainer.new()
	side.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	var t: String = _MM.notice_text(rec)
	side.add_child(_label(t, 1.0, GOOD if ok else (DIM if bool(rec.get("cancelled", false)) else BAD)))
	for e in rec.get("errors", []):
		var m := message(e)
		if m.text != "" and not t.ends_with(m.text):
			side.add_child(_label(m.text, 0.9, BAD))
		if m.fix != "":
			side.add_child(_label("Fix: " + m.fix, 0.9))
	for n in rec.get("notes", []):
		side.add_child(_note(str(n)))
	h.add_child(side)
	if ok:
		h.add_child(_btn("OpenLast", " Open ", show_detail.bind(str(rec.id))))
	h.add_child(_btn("Dismiss", " Dismiss ", func():
		mm.dismiss_last()
		_refresh_overview()))
	return card


func _model_card(r: Dictionary) -> Control:
	var card := _card()
	card.name = "Model_" + str(r.id)
	var h := HBoxContainer.new()
	h.add_theme_constant_override("separation", 12)
	card.add_child(h)
	var g := Label.new()
	g.name = "Grade"
	g.text = str(r.grade) if str(r.grade) != "" else "-"
	g.custom_minimum_size = Vector2(44, 0)
	g.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	g.add_theme_font_size_override("font_size", int(get_theme_default_font_size() * 1.8))
	g.add_theme_color_override("font_color", _grade_color(str(r.grade)))
	h.add_child(g)
	var side := VBoxContainer.new()
	side.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	var nm := _label(str(r.name), 1.1)
	nm.name = "Name"
	side.add_child(nm)
	var flies := ", ".join(PackedStringArray((r.drones as Array).map(func(d): return str(d.name))))
	var bits := PackedStringArray(["comes with real_quad" if r.origin == "shipped" else "yours",
		"flies on: " + flies if flies != "" else "not flown on any drone", _MM.human_bytes(int(r.bytes))])
	if r.origin == "user" and not bool(r.inputs):
		bits.append("inputs unknown")
	if bool(r.building):
		bits.append("being rebuilt")
	var sub := _note(" · ".join(bits))
	sub.name = "Sub"
	side.add_child(sub)
	h.add_child(side)
	h.add_child(_btn("Details", " Details ", show_detail.bind(str(r.id))))
	return card


## Live progress of the running build (each frame)
func _job_update() -> void:
	if mm == null or not mm.busy():
		return
	var c: Dictionary = mm.current()
	var txt := ("Cancelling..." if bool(c.cancelling) else (str(c.step) if str(c.step) != "" else "Starting...")) + " %d %%" % int(round(float(c.frac) * 100.0))
	if _job_step != null and is_instance_valid(_job_step):
		_job_step.text = txt
		_job_bar.value = float(c.frac) * 100.0
		_job_elapsed.text = "%d s. This takes a few minutes; the game goes on behind it, and may run slower on a weak computer." % int(c.elapsed_s)
	if stage == "detail" and _d_step != null and is_instance_valid(_d_step):
		_d_step.text = "%s: %s" % [c.name, txt]
		_d_bar.value = float(c.frac) * 100.0



# ---- page 2: new model - logs, findings, the quad ----

func _build_new(page: VBoxContainer) -> void:
	var top := HBoxContainer.new()
	top.add_child(_btn("Back", " < Models ", func(): _go("overview")))
	page.add_child(top)
	var v := _scroll(page)
	v.add_child(_note("Makes a flight model of your own quad from its Betaflight blackbox logs (Betaflight 2025.12 or newer, 4 motors, bidirectional DShot for the rotor speeds). Pick the logs of this quad: the more of what it does the better, with chirp sweeps best. The logs are copied into the model's folder, so it can be rebuilt later without the originals."))
	v.add_child(HSeparator.new())
	v.add_child(_heading("Logs"))
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 10)
	var add := _btn("AddFiles", " Add log files... ", func(): _pick_files("new"), "Pick .bbl, .bfl or .txt blackbox logs (several at once)")
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
	_new_holder = VBoxContainer.new()
	_new_holder.name = "FormHolder"
	v.add_child(_new_holder)
	_form_box = VBoxContainer.new()
	_form_box.name = "Form"
	_form_box.add_theme_constant_override("separation", 6)
	_new_holder.add_child(_form_box)
	_build_form(_form_box)
	var foot := HBoxContainer.new()
	foot.add_theme_constant_override("separation", 12)
	page.add_child(foot)
	_hint = _label("", 0.9)
	_hint.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	foot.add_child(_hint)
	_build = _btn("Build", "  Build the model  ", func(): start_build())
	foot.add_child(_build)


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
	_confirm = CheckBox.new()
	_confirm.name = "ConfirmSame"
	_confirm.text = CONFIRM_TEXT
	_confirm.button_pressed = true
	_confirm.toggled.connect(func(_on):
		_refresh()
		_validate())
	f.add_child(_confirm)
	f.add_child(_note("The logs cannot show this, so the model takes your word. A log flown with other props, a lighter pack or without the camera makes a model that is a blend of both: leave such logs out. (Logs of one quad from different flight controllers are accepted on the same word.)"))



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
		elif key == "pack_type" and not paths.is_empty():
			_inspect()               # the pack check depends on the pack type
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

func _pick_files(target: String) -> void:
	_pick_for = target
	if _dialog == null:
		_dialog = FileDialog.new()
		_dialog.name = "FileDialog"
		_dialog.process_mode = Node.PROCESS_MODE_ALWAYS
		_dialog.file_mode = FileDialog.FILE_MODE_OPEN_FILES
		_dialog.access = FileDialog.ACCESS_FILESYSTEM
		_dialog.use_native_dialog = false
		_dialog.title = "Pick blackbox logs"
		_dialog.add_filter("*.bbl, *.bfl, *.txt, *.BBL, *.BFL, *.TXT", "Blackbox logs")
		_dialog.files_selected.connect(func(p): add_files(p) if _pick_for == "new" else add_detail_logs(p))
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


func _remember_dir(picked: PackedStringArray) -> void:
	if picked.is_empty():
		return
	var c := _cfg()
	c.set_value("logfit", "last_dir", picked[0].get_base_dir())
	DirAccess.make_dir_recursive_absolute(cfg_path.get_base_dir())
	c.save(cfg_path)


## "log" for a Betaflight blackbox log (its first header line is in the first kilobytes: a .txt next to the
## logs may be a CLI dump or notes, which is left out instead of failing the build), "other" for a readable
## file that is none, else the text of the error for a file that cannot be read (permissions, macOS privacy).
static func log_kind(path: String) -> String:
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		var B = load(BUILD)
		return B.unreadable_issue(path.get_file(), B.open_why(path)).message
	var head := f.get_buffer(mini(f.get_length(), 16384)).get_string_from_ascii()
	return "log" if head.contains("H Product:Blackbox flight data recorder") else "other"


## Sorts a picked file after log_kind(): into the logs it is (true) or into skipped / unreadable (false).
func _is_log(p: String) -> bool:
	var k := log_kind(p)
	if k == "other":
		skipped.append(p.get_file())
	elif k != "log":
		unreadable.append(k)
	return k == "log"


func _skipped_note() -> Control:
	var box := VBoxContainer.new()
	if not skipped.is_empty():
		box.add_child(_label("Not blackbox logs, left out: %s" % ", ".join(skipped), 1.0, WARN))
	for u in unreadable:
		box.add_child(_label(u, 1.0, BAD))
	return box


## A log's name in a list: its file name, with its folder when another file in the list has the
## same name (logs of different flights are often all called btfl_001.bbl)
static func shown_name(p: String, all: PackedStringArray) -> String:
	for q in all:
		if q != p and q.get_file() == p.get_file():
			return p.get_base_dir().get_file().path_join(p.get_file())
	return p.get_file()


static func same_file(a: String, b: String) -> bool:
	return ProjectSettings.globalize_path(a).simplify_path() == ProjectSettings.globalize_path(b).simplify_path()


## Adds logs to the new model's list (a file already there is not added twice) and reads them
func add_files(picked: PackedStringArray) -> void:
	var added := false
	skipped = PackedStringArray()
	unreadable = PackedStringArray()
	for p in picked:
		if p.get_extension().to_lower() in EXTS and not p in paths:
			if not _is_log(p):
				continue
			paths.append(p)
			added = true
	_remember_dir(picked)
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


# ---- reading the logs (a Thread, polled each frame) ----

func _script() -> Script:
	if logfit_script != null:
		return logfit_script
	if mm != null and mm.logfit_script != null:
		return mm.logfit_script
	if FileAccess.file_exists(LOGFIT) or ResourceLoader.exists(LOGFIT):
		var s = load(LOGFIT)
		if s is Script:
			return s
	return null


func available() -> bool:
	return _script() != null


## True while the logs are being read
func busy() -> bool:
	return _thread != null


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
	# the pack type decides the voltage band of the pre-build pack check (Li-ion sits lower than LiPo)
	var pack: String = PACKS.keys()[(_ctl["pack_type"] as OptionButton).selected] if _ctl.has("pack_type") and (_ctl["pack_type"] as OptionButton).selected >= 0 else ""
	_done = false
	_answer = null
	_thread = Thread.new()
	_thread.start(func():
		var r = _logfit.inspect(ps, pack)
		_mutex.lock()
		_answer = r
		_done = true
		_mutex.unlock(), Thread.PRIORITY_LOW)
	set_process(true)


func _process(_d: float) -> void:
	# the game hides an idle cursor and shows it again only on mouse motion in its own window,
	# which the file dialog (a window of its own) takes: keep it shown while this is open
	if visible and Input.mouse_mode != Input.MOUSE_MODE_VISIBLE:
		Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
	if hide_when_unpaused and visible and _was_paused and not _paused():
		close()
	if not visible:
		if _thread == null:
			set_process(false)
		return
	if mm != null:
		if mm.busy():
			mm.poll()
		_job_update()
		if mm.last != _seen_last or (stage == "overview" and mm.busy() != (_job_step != null)) or (stage == "detail" and mm.busy() != (_d_step != null)):
			_seen_last = mm.last
			_job_changed()
	if _thread == null:
		return
	_mutex.lock()
	var done := _done
	var ans = _answer
	_mutex.unlock()
	if not done and not _thread.is_alive():
		# the thread is gone: it finished a moment ago (done is set before it ends), or it stopped with an
		# error in the log reader before it could answer
		_mutex.lock()
		done = _done
		ans = _answer
		_mutex.unlock()
		if not done:
			ans = {"errors": ["The log reader stopped unexpectedly (an error in the model builder: see the game's log)."]}
			done = true
	if not done:
		return
	_thread.wait_to_finish()
	_thread = null
	info = ans if ans is Dictionary else {"errors": ["The log reader gave no answer."]}
	if _again:
		_again = false
		_inspect()
	_prefill()
	_refresh()


## A job started or finished
func _job_changed() -> void:
	if stage == "overview":
		_refresh_overview()
	elif stage == "detail":
		# a job of this model that worked brings a new package: the page is read again. Any other change (another
		# model's job, a notice dismissed, a build that failed or was cancelled) leaves the page, and what was
		# typed or marked in it, alone
		if not detail_id.is_empty() and not mm.busy() and str(mm.last.get("id", "")) == detail_id and bool(mm.last.get("ok", false)):
			_reload_detail(false)
		else:
			_fill_job_banner()
			_sync_action_buttons()
			_update_buttons()
	elif stage == "new":
		_validate()


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
## The errors that stop a build (the confirmation lifts the ones that say so)
func blocking() -> Array:
	var out := []
	for e in info.get("errors", []):
		var m := message(e)
		if m["overridable"] and _confirm != null and _confirm.button_pressed:
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



# ---- redraw of the new-model page ----

func _refresh() -> void:
	if _files_box == null:
		return
	if host != null and host.has_method("game_drones") and _drones.is_empty():
		_drones = host.game_drones()
		_fill_drones()
	_clear(_files_box)
	_clear(_msgs)
	if not skipped.is_empty() or not unreadable.is_empty():
		_msgs.add_child(_skipped_note())
	if not available():
		_msgs.add_child(_label("The model builder is not installed completely: real_quad/logfit/logfit.gd is missing. Update the mod.", 1.0, BAD))
	if paths.is_empty():
		_files_box.add_child(_note("No logs yet."))
	var sessions: Array = info.get("sessions", [])
	for i in paths.size():
		var p := paths[i]
		var row := HBoxContainer.new()
		row.name = "File_%d_%s" % [i, p.get_file().validate_node_name()]
		row.add_theme_constant_override("separation", 8)
		var n := _label(shown_name(p, paths))
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
			if s is Dictionary and same_file(str(s.get("file", "")), p):
				_files_box.add_child(_session_line(s))
	if busy():
		_files_box.add_child(_note("Reading the logs..."))
	elif not info.is_empty():
		var ok := 0
		var secs := 0.0
		for s in sessions:
			if s is Dictionary and bool(s.usable):
				ok += 1
				secs += float(s.get("armed_s", 0.0))
		var fw := str(info.get("firmware", ""))
		_files_box.add_child(_label("%d session%s usable%s%s" % [ok, "" if ok == 1 else "s",
			", %.1f min of flight" % (secs / 60.0) if secs > 0.0 else "", ", " + fw if fw != "" else ""], 0.95, GROUP))
	var blocked := blocking()
	for m in blocked:
		_msgs.add_child(_message_box(m, BAD, "Cannot build: "))
	for b in banners():
		_msgs.add_child(_message_box(b, WARN, "WARNING: "))
	for w in info.get("warnings", []):
		var m := message(w)
		if _kind_of(m["text"]) == "" and not m["big"]:
			_msgs.add_child(_message_box(m, DIM, ""))
	# an error the confirmation lifted is a note, not an error
	for e in info.get("errors", []):
		var m := message(e)
		if m["overridable"] and _confirm != null and _confirm.button_pressed:
			m["text"] += " (Used all the same: you confirmed that all logs are of one setup.)"
			_msgs.add_child(_message_box(m, DIM, "Note: "))
	if _mode == "new":
		_form_box.visible = not paths.is_empty() and not info.is_empty()
	_validate()


func _session_line(s: Dictionary) -> Control:
	var usable := bool(s.usable)
	var reason := str(s.get("reason", ""))
	var dur := float(s.get("armed_s", -1.0))
	var t := "   session %s" % str(s.index)
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

## Checks the boxes: {form, errors {key: text}, id}; shows the errors and sets the buttons
func _validate() -> Dictionary:
	var errors := {}
	var form := {}
	if _ctl.is_empty():
		return {"form": form, "errors": errors}
	var nm := (_ctl["name"] as LineEdit).text.strip_edges()
	var id := slug(nm) if _mode == "new" else detail_id
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
	if _confirm != null and _confirm.button_pressed:
		form["same_quad"] = true
	# a new model needs a free name
	if _mode == "new" and id != "" and mm != null:
		var origin: String = mm.origin(id)
		if origin == "shipped":
			errors["name"] = "\"%s\" is a model that comes with real_quad: pick another name (%s_2?)" % [id, id]
		elif origin == "user":
			errors["name"] = "a model of yours has this name: pick another, or open it in the list and rebuild"
	for key in _errs:
		var l: Label = _errs[key]
		l.text = str(errors.get(key, ""))
		l.visible = errors.has(key)
	var res := {"form": form, "errors": errors, "id": id}
	_update_buttons(res)
	return res


## Why the form cannot be used now ("" = it can)
func _form_problem(v: Dictionary) -> String:
	if _confirm != null and not _confirm.button_pressed:
		return "Tick the confirmation: the logs must all be of the quad as entered, or the model is a blend."
	if not v["errors"].is_empty():
		return "Fill in: " + ", ".join(PackedStringArray(v["errors"].keys().map(func(k): return _label_of(k))))
	return ""


func _busy_text() -> String:
	if mm == null or not mm.busy():
		return ""
	var c: Dictionary = mm.current()
	return "%s is being built (%s, %d %%): wait for it to finish, or cancel it in the model list." % [c.name, c.step, int(round(float(c.frac) * 100.0))]


func _update_buttons(v := {}) -> void:
	if _ctl.is_empty():
		return
	if v.is_empty():
		v = _validate()
		return
	if _mode == "new" and _build != null:
		var why := ""
		if not blocking().is_empty():
			why = "Fix the problems above (or take other logs) to build."
		elif paths.is_empty():
			why = "Add the logs of your quad first."
		elif info.is_empty():
			why = "Reading the logs..."
		elif busy():
			why = "Reading the logs..."
		else:
			why = _form_problem(v)
		if why == "" and not available():
			why = "The model builder is not installed completely."
		if why == "":
			why = _busy_text()
		_build.disabled = why != ""
		_hint.text = why
	elif _mode == "edit" and _d_hint != null:
		_update_detail_buttons(v)


func _label_of(key: String) -> String:
	for spec in FIELDS:
		if spec["key"] == key:
			return str(spec["label"]).get_slice(" (", 0)
	return "the name" if key == "name" else key


## A name as a folder name: [a-z0-9_], at most 32 characters
static func slug(s: String) -> String:
	return _MM.slug(s)


## Starts the build of a new model with what the form says; false when the form or the logs do not allow it.
## The manager runs it: the window goes to the overview, where the progress is.
func start_build() -> bool:
	var v := _validate()
	if not v["errors"].is_empty() or not blocking().is_empty() or busy() or not available() or mm == null:
		return false
	if _confirm != null and not _confirm.button_pressed:
		return false
	var err: String = mm.start({"id": v["id"], "form": v["form"], "add": Array(paths), "new": true})
	if err != "":
		_hint.text = err
		return false
	_reset_new()
	_go("overview")
	return true


## The new-model page empty again
func _reset_new() -> void:
	paths = PackedStringArray()
	info = {}
	_touched = {}
	_mode = "new"
	for k in _ctl:
		var c: Control = _ctl[k]
		if c is LineEdit:
			(c as LineEdit).text = ""
		elif c is OptionButton and k != "zone_vehicle_id":
			(c as OptionButton).select(3 if k == "class" else 0)
		_mark(k, "")
	_confirm.button_pressed = true
	_class_changed(false)
	_pick_drone(str(CLASSES[_class_key()]["drone"]), false)


# ---- page 3: one model ----

func _build_detail(page: VBoxContainer) -> void:
	_d_head = HBoxContainer.new()
	_d_head.name = "Head"
	_d_head.add_theme_constant_override("separation", 14)
	page.add_child(_d_head)
	_d_job = VBoxContainer.new()
	_d_job.name = "JobBanner"
	page.add_child(_d_job)
	_d_actions = VBoxContainer.new()
	_d_actions.name = "Actions"
	_d_actions.add_theme_constant_override("separation", 6)
	page.add_child(_d_actions)
	_tabs = TabContainer.new()
	_tabs.name = "Tabs"
	_tabs.size_flags_vertical = Control.SIZE_EXPAND_FILL
	page.add_child(_tabs)
	for t in ["Report", "Logs", "Parameters"]:
		var holder := VBoxContainer.new()
		holder.name = t
		_tabs.add_child(holder)
		var v := _scroll(holder)
		match t:
			"Report": _d_report = v
			"Logs": _d_logs = v
			"Parameters": _d_params = v
	_d_foot = HBoxContainer.new()
	_d_foot.name = "Foot"
	_d_foot.add_theme_constant_override("separation", 12)
	page.add_child(_d_foot)
	_d_hint = _label("", 0.9)
	_d_hint.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_d_foot.add_child(_d_hint)
	_d_foot.add_child(_btn("SaveMeta", "  Save name and drone  ", save_meta, "Writes the name and the Zone drone into the model; no rebuild"))
	_d_foot.add_child(_btn("Rebuild", "  Rebuild the model  ", rebuild, "Builds the model again from its logs and these parameters; the old one flies until the new one is ready"))


## Opens one model's page
func show_detail(id: String) -> void:
	if mm == null:
		return
	var d: Dictionary = mm.details(id)
	if d.is_empty():
		_go("overview")
		return
	detail_id = id
	_reload_detail(true)
	_go("detail")


func _reload_detail(first: bool) -> void:
	var d: Dictionary = mm.details(detail_id)
	if d.is_empty():
		detail_id = ""
		detail = {}
		_go("overview")
		return
	detail = d
	pend_add = PackedStringArray()
	pend_rm = {}
	_confirming_delete = false
	_move_form(_new_holder)
	_title.text = str(d.name)
	var editable: bool = d.origin == "user"
	_fill_head()
	_fill_job_banner()
	_fill_actions()
	_fill_report()
	_fill_logs()
	_tabs.set_tab_hidden(1, not editable)
	_tabs.set_tab_hidden(2, not editable)
	if editable:
		_move_form(_d_params)
		_form_box.visible = true
		set_form(d.form)
		_baseline = _validate()["form"]
	else:
		_mode = "new"
	_tabs.current_tab = 0
	_d_foot.visible = editable
	if editable:
		_update_buttons(_validate())


func _fill_head() -> void:
	_clear(_d_head)
	var d := detail
	var grade := str(d.grade)
	_d_head.add_child(_btn("Back", " < Models ", func(): _go("overview")))
	var g := Label.new()
	g.name = "Grade"
	g.text = grade if grade != "" else "-"
	g.add_theme_font_size_override("font_size", int(get_theme_default_font_size() * 3.2))
	g.add_theme_color_override("font_color", _grade_color(grade))
	_d_head.add_child(g)
	var side := VBoxContainer.new()
	side.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	side.add_child(_label("%s - %s" % [d.name, MEANING.get(grade, "")] if MEANING.has(grade) else str(d.name), 1.3))
	var flies := ", ".join(PackedStringArray((d.drones as Array).map(func(x): return str(x.name))))
	var bits := PackedStringArray(["comes with real_quad (read-only)" if d.origin == "shipped" else "yours",
		"flies on: " + flies if flies != "" else "not flown on any drone",
		"uses %s" % _MM.human_bytes(int(d.bytes)) + (" (logs %s)" % _MM.human_bytes(int(d.logs_bytes)) if int(d.logs_bytes) > 0 else "")])
	var sub := _note(" · ".join(bits) + "\n" + ProjectSettings.globalize_path(str(d.dir)))
	sub.name = "Sub"
	side.add_child(sub)
	_d_head.add_child(side)


## The progress of the running build, when it is this model's
func _fill_job_banner() -> void:
	_clear(_d_job)
	_d_step = null
	_d_bar = null
	if mm == null:
		return
	if not mm.busy():
		# a build of this model that failed: say so here, where the rebuild was started
		var rec: Dictionary = mm.last
		if not rec.is_empty() and str(rec.get("id", "")) == detail_id and not bool(rec.get("ok", false)) and not bool(rec.get("cancelled", false)):
			_d_job.add_child(_last_card())
		return
	var c: Dictionary = mm.current()
	var card := _card()
	var v := VBoxContainer.new()
	card.add_child(v)
	var h := HBoxContainer.new()
	h.add_theme_constant_override("separation", 12)
	v.add_child(h)
	_d_step = _label("", 0.95, GROUP)
	_d_step.name = "JobStep"
	_d_step.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	h.add_child(_d_step)
	h.add_child(_btn("Cancel", "  Cancel  ", func(): mm.cancel()))
	_d_bar = ProgressBar.new()
	_d_bar.name = "JobBar"
	_d_bar.min_value = 0.0
	_d_bar.max_value = 100.0
	_d_bar.custom_minimum_size = Vector2(0, 18)
	v.add_child(_d_bar)
	_d_job.add_child(card)
	_job_update()


func _fill_actions() -> void:
	_clear(_d_actions)
	var d := detail
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 10)
	var l := Label.new()
	l.text = "Fly it on"
	row.add_child(l)
	_use_pick = OptionButton.new()
	_use_pick.name = "Drone"
	_use_pick.custom_minimum_size = Vector2(220, 0)
	_use_pick.fit_to_longest_item = false
	_use_pick.add_item("(pick a drone)")
	_use_pick.set_item_metadata(0, "")
	for x in _drones:
		_use_pick.add_item(str(x["name"]))
		_use_pick.set_item_metadata(_use_pick.item_count - 1, str(x["id"]))
	var want := str(d.zone_vehicle_id)
	if (d.drones as Array).is_empty():
		for i in _use_pick.item_count:
			if str(_use_pick.get_item_metadata(i)) == want:
				_use_pick.select(i)
	else:
		for i in _use_pick.item_count:
			if str(_use_pick.get_item_metadata(i)) == str(d.drones[0].id):
				_use_pick.select(i)
	_use_pick.item_selected.connect(func(_i): _use_label())
	row.add_child(_use_pick)
	var use := _btn("UseFor", "", use_for_drone)
	row.add_child(use)
	_use_note = _label("", 0.9)
	_use_note.name = "UseNote"
	_use_note.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	row.add_child(_use_note)
	_d_actions.add_child(row)
	var row2 := HBoxContainer.new()
	row2.add_theme_constant_override("separation", 10)
	if d.origin == "shipped":
		row2.add_child(_btn("Duplicate", " Duplicate and edit ", duplicate_model, "Makes a copy of yours that you can change"))
	else:
		row2.add_child(_btn("Duplicate", " Duplicate ", duplicate_model, "Makes a copy of this model, with its logs"))
		row2.add_child(_btn("Delete", " Delete ", func():
			_confirming_delete = true
			_fill_actions(), "Deletes the model and its logs"))
	row2.add_child(_btn("OpenFolder", " Open folder ", open_folder))
	_d_actions.add_child(row2)
	if _confirming_delete:
		var cr := HBoxContainer.new()
		cr.name = "DeleteConfirm"
		cr.add_theme_constant_override("separation", 10)
		cr.add_child(_label("Delete %s, its logs and its report for good? The drones set to it fly their default again." % d.name, 1.0, BAD))
		cr.add_child(_btn("DeleteYes", " Yes, delete ", delete_model))
		cr.add_child(_btn("DeleteNo", " Keep it ", func():
			_confirming_delete = false
			_fill_actions()))
		_d_actions.add_child(cr)
	_use_label()
	_sync_action_buttons()


## True while the model of the open page is being built (its folder is about to be replaced)
func _detail_building() -> bool:
	return mm != null and mm.busy() and str(mm.current().get("id", "")) == detail_id


## Duplicate, Delete and Open folder wait while the open model is built
func _sync_action_buttons() -> void:
	var building := _detail_building()
	for n in ["Duplicate", "Delete", "OpenFolder", "DeleteYes"]:
		var b := _d_actions.find_child(n, true, false) as Button
		if b != null:
			b.disabled = building
			if building:
				if not b.has_meta("tip"):
					b.set_meta("tip", b.tooltip_text)
				b.tooltip_text = "%s is being built: wait for it to finish, or cancel it." % detail.get("name", detail_id)
			elif b.has_meta("tip"):
				b.tooltip_text = str(b.get_meta("tip"))


func _use_label() -> void:
	var use := _d_actions.find_child("UseFor", true, false) as Button
	if use == null or _use_pick == null:
		return
	var id := str(_use_pick.get_item_metadata(_use_pick.selected)) if _use_pick.selected >= 0 else ""
	use.disabled = id == ""
	use.text = " Use for %s " % _use_pick.get_item_text(_use_pick.selected) if id != "" else " Use for a drone "


## Writes model=<id> into the picked drone's settings and has real_quad take it
func use_for_drone() -> void:
	var vid := str(_use_pick.get_item_metadata(_use_pick.selected))
	if vid == "" or detail_id == "" or host == null:
		return
	var err: String = host.set_drone_model(vid, detail_id)
	_use_note.text = err if err != "" else "Set: %s flies the %s model now." % [_use_pick.get_item_text(_use_pick.selected), detail_id]
	_use_note.add_theme_color_override("font_color", BAD if err != "" else GOOD)
	if err == "":
		detail = mm.details(detail_id)
		_fill_head()


func duplicate_model() -> void:
	var r: Dictionary = mm.duplicate(detail_id)
	if not bool(r.ok):
		_use_note.text = str(r.error)
		_use_note.add_theme_color_override("font_color", BAD)
		return
	show_detail(str(r.id))


func delete_model() -> void:
	var err: String = mm.delete(detail_id)
	if err != "":
		_confirming_delete = false
		_fill_actions()
		_use_note.text = err
		_use_note.add_theme_color_override("font_color", BAD)
		return
	detail_id = ""
	detail = {}
	_go("overview")


func open_folder() -> void:
	OS.shell_open(ProjectSettings.globalize_path(str(detail.dir)))


func _fill_report() -> void:
	_clear(_d_report)
	var rep := RichTextLabel.new()
	rep.name = "ReportText"
	rep.bbcode_enabled = true
	rep.fit_content = true
	rep.scroll_active = false
	rep.selection_enabled = true
	rep.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	var md := str(detail.report_md)
	rep.text = md_to_bbcode(md, get_theme_default_font_size()) if md != "" else "This model has no report."
	_d_report.add_child(rep)


# ---- the logs of a model ----

func _fill_logs() -> void:
	_clear(_d_logs)
	var d := detail
	if d.origin != "user":
		return
	var stored: Array = (d.logs as Array).filter(func(l): return bool(l.stored))
	_d_logs.add_child(_note("The logs are kept in the model's folder (logs/), so it can be rebuilt without the originals. Add or remove logs, then rebuild. Identical files are kept once."))
	if not skipped.is_empty() or not unreadable.is_empty():
		_d_logs.add_child(_skipped_note())
	if not bool(d.inputs_known) or (stored.is_empty() and not (d.logs as Array).is_empty()):
		var names := ", ".join(PackedStringArray((d.logs as Array).map(func(l): return str(l.name))))
		_d_logs.add_child(_label("%s. This model was built before the logs were kept with it%s." % [INPUTS_UNKNOWN, " (from %d files: %s)" % [d.logs.size(), names] if not (d.logs as Array).is_empty() else ""], 1.0, WARN))
	elif stored.is_empty():
		_d_logs.add_child(_label(INPUTS_UNKNOWN + ".", 1.0, WARN))
	for l in stored:
		var row := HBoxContainer.new()
		row.name = "Log_" + str(l.name).validate_node_name()
		row.add_theme_constant_override("separation", 10)
		var removed := pend_rm.has(str(l.name))
		var n := _label("%s  (%s)%s" % [l.name, _MM.human_bytes(int(l.size)), "  - will be removed" if removed else ""], 1.0, DIM if removed else Color.WHITE)
		n.autowrap_mode = TextServer.AUTOWRAP_OFF
		n.clip_text = true
		n.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		row.add_child(n)
		row.add_child(_btn("Undo" if removed else "Remove", " Undo " if removed else " Remove ", toggle_log.bind(str(l.name))))
		_d_logs.add_child(row)
	for i in pend_add.size():
		var p := pend_add[i]
		var row := HBoxContainer.new()
		row.name = "Add_%d_%s" % [i, p.get_file().validate_node_name()]
		row.add_theme_constant_override("separation", 10)
		var n := _label("%s  - will be added" % shown_name(p, pend_add), 1.0, GOOD)
		n.autowrap_mode = TextServer.AUTOWRAP_OFF
		n.clip_text = true
		n.tooltip_text = p
		n.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		row.add_child(n)
		row.add_child(_btn("Remove", " Remove ", drop_added.bind(p)))
		_d_logs.add_child(row)
	var foot := HBoxContainer.new()
	foot.add_theme_constant_override("separation", 10)
	foot.add_child(_btn("AddLogs", " Add log files... ", func(): _pick_files("detail"), "Pick .bbl, .bfl or .txt blackbox logs of this quad (several at once)"))
	var disk := _note("Disk use: logs %s of %s in all." % [_MM.human_bytes(int(d.logs_bytes)), _MM.human_bytes(int(d.bytes))])
	disk.name = "Disk"
	disk.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	disk.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	foot.add_child(disk)
	_d_logs.add_child(foot)


func toggle_log(log_name: String) -> void:
	if pend_rm.has(log_name):
		pend_rm.erase(log_name)
	else:
		pend_rm[log_name] = true
	_fill_logs()
	_update_buttons(_validate())


func drop_added(p: String) -> void:
	var i := pend_add.find(p)
	if i >= 0:
		pend_add.remove_at(i)
	_fill_logs()
	_update_buttons(_validate())


## Logs to add to the model in the detail page
func add_detail_logs(picked: PackedStringArray) -> void:
	skipped = PackedStringArray()
	unreadable = PackedStringArray()
	for p in picked:
		if p.get_extension().to_lower() in EXTS and not p in pend_add:
			if not _is_log(p):
				continue
			pend_add.append(p)
	_remember_dir(picked)
	_fill_logs()
	_update_buttons(_validate())


# ---- the parameters of a model ----

func _num_text(x) -> String:
	var f := float(x)
	return str(int(f)) if f == floorf(f) else String.num(f, 3)


## Fills the form from a stored form (the detail page)
func set_form(f: Dictionary) -> void:
	_mode = "edit"
	(_ctl["name"] as LineEdit).text = str(f.get("name", ""))
	for spec in FIELDS:
		var k: String = spec["key"]
		(_ctl[k] as LineEdit).text = _num_text(f[k]) if f.get(k) != null else ""
	var pi := PACKS.keys().find(str(f.get("pack_type", "lipo")))
	(_ctl["pack_type"] as OptionButton).select(maxi(pi, 0))
	(_ctl["class"] as OptionButton).select(maxi(CLASS_ORDER.find(str(f.get("class", "five"))), 0))
	(_ctl["ducted"] as OptionButton).select(0 if f.get("ducted") == null else (1 if bool(f.ducted) else 2))
	for k in _ctl:
		_touched[k] = true
		_mark(k, "")
	_confirm.button_pressed = true
	_class_changed(false)
	_pick_drone(str(f.get("zone_vehicle_id", "")), true)


func _move_form(to: Control) -> void:
	if _form_box == null or to == null or _form_box.get_parent() == to:
		return
	if _form_box.get_parent() != null:
		_form_box.get_parent().remove_child(_form_box)
	to.add_child(_form_box)


static func _differs(a: Variant, b: Variant) -> bool:
	if a == null or b == null:
		return a != b
	if (a is int or a is float) and (b is int or b is float):
		return absf(float(a) - float(b)) > 1e-9 * maxf(1.0, absf(float(a)))
	return str(a) != str(b)


## What the detail page would do now: {meta: only name / drone changed, rebuild: more changed or logs changed, why, remaining logs}
func edit_state(v: Dictionary) -> Dictionary:
	var meta := false
	var rebuild := not pend_add.is_empty() or not pend_rm.is_empty()
	var keys := {}
	for k in v["form"]:
		keys[k] = true
	for k in _baseline:
		keys[k] = true
	for k in keys:
		if k == "same_quad":
			continue
		if _differs(v["form"].get(k), _baseline.get(k)):
			if k in ["name", "zone_vehicle_id"]:
				meta = true
			else:
				rebuild = true
	var kept := 0
	for l in detail.get("logs", []):
		if bool(l.stored) and not pend_rm.has(str(l.name)):
			kept += 1
	return {"meta": meta, "rebuild": rebuild, "logs": kept + pend_add.size()}


func _update_detail_buttons(v: Dictionary) -> void:
	var st := edit_state(v)
	var save := _d_foot.find_child("SaveMeta", true, false) as Button
	var reb := _d_foot.find_child("Rebuild", true, false) as Button
	var why := _form_problem(v)
	var busy_t := _busy_text()
	save.visible = bool(st.meta) and not bool(st.rebuild)
	save.disabled = why != "" or busy_t != ""
	var reb_why := why
	if reb_why == "" and int(st.logs) == 0:
		reb_why = INPUTS_UNKNOWN + " (Logs tab)."
	if reb_why == "" and busy_t != "":
		reb_why = busy_t
	reb.disabled = reb_why != ""
	var idle := "Rebuild runs the fit again on the stored logs."
	if str(detail.get("propwash", {}).get("stale", "")) != "" and str(detail.get("origin", "")) == "user":
		idle = STALE_HINT
	_d_hint.text = reb_why if reb_why != "" else ("Changes: the parameters or logs changed - rebuild to use them." if bool(st.rebuild) else
		("The name or the drone changed: save it, no rebuild needed." if bool(st.meta) else idle))


## Builds the model again from its logs and the parameters in the form
func rebuild() -> bool:
	var v := _validate()
	if mm == null or _form_problem(v) != "" or detail_id == "":
		return false
	var keep: Array = []
	for l in detail.logs:
		if bool(l.stored) and not pend_rm.has(str(l.name)):
			keep.append(str(l.name))
	var err: String = mm.start({"id": detail_id, "form": v["form"], "add": Array(pend_add), "keep": keep, "new": false})
	if err != "":
		_d_hint.text = err
		return false
	_go("overview")
	return true


func save_meta() -> bool:
	var v := _validate()
	if mm == null or _form_problem(v) != "":
		return false
	var err: String = mm.apply_meta(detail_id, str(v["form"]["name"]), str(v["form"].get("zone_vehicle_id", "")))
	if err != "":
		_d_hint.text = err
		return false
	_reload_detail(false)
	return true


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

