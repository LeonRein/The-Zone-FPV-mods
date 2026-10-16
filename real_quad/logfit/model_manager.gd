extends RefCounted
## real_quad: the model manager's logic (no UI; the window is logfit/window.gd). Owned by real_quad's
## main.gd, so a build goes on when the window is closed.
##
##   var mm := ModelManager.new(host, "user://real_quad/models")      # host: main.gd (registry, set_drone_model, game_drones)
##   mm.list()                       # [{id, name, origin shipped|user, grade, drones [{id, name}], bytes, inputs, ...}]
##   mm.details(id)                  # + report_md, form (the window's form), logs [{name, size, sha256, stored}]
##   mm.start({id, form, add: [paths], keep: [names] | null, new: bool}) -> "" or why not
##   mm.recalibrate(id) -> "" or why not  # only the prop wash calibration (a job as a build; details(id).propwash
##                                         # {recalibrate, why, text} says when it is due: another dirty_air installed)
##   mm.autotune(id) -> "" or why not     # the tuned twin of a grade A/B model (simulator only): a new model
##                                         # <id>_tuned of yours (the same plant, the tuned fc.json), replaced when tuned again
##   mm.poll()                       # main thread, each frame while busy(): collects a finished job (rescans the models)
##   mm.status()                     # "Building <name>: <step> 45 %" while a job runs, else "" (mod_settings' line)
##   mm.duplicate(id) / mm.delete(id) / mm.apply_meta(id, name, drone)
##
## One job at a time, a start while one runs is refused with a text that names it. A job copies the new
## logs into the model's logs/ folder (identical files, by SHA-256, once), runs logfit.build() in a
## Thread into <work>/building/<id> and, when it worked, copies logs/ and writes inputs.json into the new
## package, checks that it loads, puts the old package aside and moves the new one in place: the old model
## flies until the new one is there, a failed or cancelled job leaves it as it was (logs included: the old
## package keeps its own until the swap has worked). inputs.json records the form and the logs
## (name, size, SHA-256), so a rebuild needs nothing but the model's folder. A package without it
## (made before the manager) shows its form from logfit.json / craft.json and "inputs unknown".

signal finished(record: Dictionary)

const LOGFIT := "res://real_quad/logfit/logfit.gd"
const LOG_EXTS := ["bbl", "bfl", "txt"]
const INPUTS := "inputs.json"
const GRADE_RE := "^## Grade ([A-D])"
const CALIBRATE := "res://real_quad/logfit/fit/calibrate.gd"
const BFModel := preload("../flightmodel/bf_model.gd")

var host = null                      # real_quad's main.gd
var root := "user://real_quad/models"
var work := "user://real_quad/models_work"
var logfit_script: Script = null     # logfit.gd, or a stand-in
var last := {}                       # the record of the last finished job, until dismissed
var notice_until_ms := 0             # the status line shows how the last job went until then

var _job: Job = null


class Job:
	var id := ""
	var name := ""
	var new_model := false
	var dir := ""                     # where the model's package is (for a rebuild)
	var recal := false                # only the prop wash calibration
	var tune := false                 # the autotune (a tuned twin)
	var step := "Starting"
	var frac := 0.0
	var t0 := 0
	var cancelled := false
	var done := false
	var result := {}
	var thread: Thread = null
	var logfit = null
	var mutex := Mutex.new()

	func progress(s: String, f: float) -> void:
		mutex.lock()
		step = s
		frac = f
		mutex.unlock()

	func snapshot() -> Dictionary:
		mutex.lock()
		var d := {"step": step, "frac": frac, "done": done}
		mutex.unlock()
		return d


func _init(p_host = null, p_root := "user://real_quad/models", p_work := "") -> void:
	host = p_host
	root = p_root
	work = p_work if p_work != "" else p_root + "_work"
	recover()


# ------------------------------------------------------------------ files

static func read_json(path: String) -> Dictionary:
	if not FileAccess.file_exists(path):
		return {}
	var d: Variant = JSON.parse_string(FileAccess.get_file_as_string(path))
	return d if d is Dictionary else {}


## Writes a JSON file; false when it could not be written completely (full disk, no rights)
static func write_json(path: String, d: Dictionary) -> bool:
	var f := FileAccess.open(path, FileAccess.WRITE)
	if f == null:
		return false
	var text := JSON.stringify(d, " ", false, true) + "\n"
	var ok := f.store_string(text)
	f.flush()
	ok = ok and f.get_error() == OK
	f.close()
	return ok and FileAccess.get_file_as_bytes(path).size() == text.to_utf8_buffer().size()


static func remove_dir(dir: String) -> void:
	var da := DirAccess.open(dir)
	if da == null:
		return
	for f in da.get_files():
		da.remove(f)
	for d in da.get_directories():
		remove_dir(dir.path_join(d))
	DirAccess.remove_absolute(dir)


static func dir_bytes(dir: String) -> int:
	var da := DirAccess.open(dir)
	if da == null:
		return 0
	var n := 0
	for f in da.get_files():
		var fa := FileAccess.open(dir.path_join(f), FileAccess.READ)
		if fa != null:
			n += fa.get_length()
	for d in da.get_directories():
		n += dir_bytes(dir.path_join(d))
	return n


static func copy_dir(src: String, dst: String) -> bool:
	if DirAccess.make_dir_recursive_absolute(dst) != OK:
		return false
	var da := DirAccess.open(src)
	if da == null:
		return false
	for f in da.get_files():
		if not copy_file(src.path_join(f), dst.path_join(f)):
			return false
	for d in da.get_directories():
		if not copy_dir(src.path_join(d), dst.path_join(d)):
			return false
	return true


static func copy_file(src: String, dst: String) -> bool:
	var a := FileAccess.open(src, FileAccess.READ)
	if a == null:
		return false
	var b := FileAccess.open(dst, FileAccess.WRITE)
	if b == null:
		return false
	var ok := true
	while ok and a.get_position() < a.get_length():
		ok = b.store_buffer(a.get_buffer(4 * 1024 * 1024))
	b.flush()
	ok = ok and b.get_error() == OK
	var want := a.get_length()
	b.close()
	# a full disk can lose the tail without an error on the way: the copy must be as long as the original
	return ok and _length(dst) == want


static func _length(path: String) -> int:
	var f := FileAccess.open(path, FileAccess.READ)
	return f.get_length() if f != null else -1


static func human_bytes(n: int) -> String:
	if n >= 1024 * 1024 * 1024:
		return "%.1f GB" % (n / 1073741824.0)
	if n >= 1024 * 1024:
		return "%.1f MB" % (n / 1048576.0)
	return "%d kB" % maxi(1, int(ceil(n / 1024.0))) if n > 0 else "0 kB"


## Names Windows does not allow for a file or folder (with any extension)
const RESERVED := ["con", "prn", "aux", "nul", "com1", "com2", "com3", "com4", "com5", "com6", "com7", "com8", "com9",
	"lpt1", "lpt2", "lpt3", "lpt4", "lpt5", "lpt6", "lpt7", "lpt8", "lpt9"]


## A name as a folder name: [a-z0-9_], at most 32 characters, not one Windows reserves (con, nul, com1 ...)
static func slug(s: String) -> String:
	var out := ""
	for ch in s.to_lower():
		out += ch if (ch >= "a" and ch <= "z") or (ch >= "0" and ch <= "9") else "_"
	while out.contains("__"):
		out = out.replace("__", "_")
	out = out.strip_edges().trim_prefix("_").trim_suffix("_")
	out = out.substr(0, 32).trim_suffix("_")
	if out in RESERVED:
		out += "_model"
	return out


## A file name that is safe on Windows too: no trailing dots or spaces (Windows drops them), no reserved
## device name (con.txt is one as well)
static func safe_file_name(nm: String) -> String:
	var out := nm.strip_edges()
	while out.ends_with("."):
		out = out.trim_suffix(".").strip_edges()
	if out == "":
		out = "log"
	if out.get_slice(".", 0).strip_edges().to_lower() in RESERVED:
		out = "_" + out
	return out


## A job that was cut short by the game closing: the old package is put back, the leftovers go. The old
## package keeps its logs while a job runs (the new one gets copies), but a crash of an older version
## may have moved some into the new package under construction: those are taken back.
func recover() -> void:
	var da := DirAccess.open(work.path_join("old"))
	if da != null:
		for d in da.get_directories():
			var back := root.path_join(d)
			if not DirAccess.dir_exists_absolute(back):
				if DirAccess.rename_absolute(work.path_join("old").path_join(d), back) == OK:
					_salvage_logs(back, work.path_join("building").path_join(d).path_join("logs"))
	remove_dir(work)


## Copies the logs a package's inputs.json names but its logs/ lacks from another folder
func _salvage_logs(pkg: String, from: String) -> void:
	if not DirAccess.dir_exists_absolute(from):
		return
	for l in read_json(pkg.path_join(INPUTS)).get("logs", []):
		var nm := str(l.get("name", ""))
		var dst := pkg.path_join("logs").path_join(nm)
		if nm != "" and FileAccess.file_exists(from.path_join(nm)) and not FileAccess.file_exists(dst):
			DirAccess.make_dir_recursive_absolute(pkg.path_join("logs"))
			copy_file(from.path_join(nm), dst)


# ------------------------------------------------------------------ the models

func model_dir(id: String) -> String:
	if host != null and host.registry != null and host.registry.has_model(id):
		return str(host.registry.get_entry(id).dir)
	return root.path_join(id)


func origin(id: String) -> String:
	if host != null and host.registry != null and host.registry.has_model(id):
		return "user" if str(host.registry.get_entry(id).root) == root else "shipped"
	return "user" if DirAccess.dir_exists_absolute(root.path_join(id)) else ""


func _grade_of(dir: String) -> String:
	var lf := read_json(dir.path_join("logfit.json"))
	if str(lf.get("grade", "")) != "":
		return str(lf.grade)
	var re := RegEx.create_from_string(GRADE_RE)
	for line in FileAccess.get_file_as_string(dir.path_join("report.md")).split("\n", true, 40):
		var m := re.search(line)
		if m != null:
			return m.get_string(1)
	return ""


func _drones_of(id: String) -> Array:
	var out := []
	if host == null:
		return out
	for d in host.game_drones():
		if host.model_id_for(str(d.id)) == id:
			out.append(d)
	return out


## Every model there is: [{id, name, origin, grade, zone_vehicle_id, drones, bytes, inputs, building}]
func list() -> Array:
	var out := []
	if host == null or host.registry == null:
		return out
	for e in host.registry.list_models():
		out.append(_row(e))
	return out


func _row(e: Dictionary) -> Dictionary:
	var dir := str(e.dir)
	var org := "user" if str(e.root) == root else "shipped"
	return {"id": str(e.id), "name": str(e.name), "origin": org, "grade": _grade_of(dir), "zone_vehicle_id": str(e.zone_vehicle_id),
		"drones": _drones_of(str(e.id)), "bytes": dir_bytes(dir), "dir": dir, "inputs": FileAccess.file_exists(dir.path_join(INPUTS)),
		"building": _job != null and not _job.done and _job.id == str(e.id)}


## One model with its report, form and logs ({} when unknown)
func details(id: String) -> Dictionary:
	if host == null or host.registry == null or not host.registry.has_model(id):
		return {}
	var r := _row(host.registry.get_entry(id))
	var dir: String = r.dir
	var inputs := read_json(dir.path_join(INPUTS))
	var lf := read_json(dir.path_join("logfit.json"))
	var craft := read_json(dir.path_join("craft.json"))
	r["report_md"] = FileAccess.get_file_as_string(dir.path_join("report.md"))
	r["grade_meaning"] = str(lf.get("grade_meaning", ""))
	r["created"] = str(lf.get("created", inputs.get("built", "")))
	r["inputs_known"] = inputs.has("logs") and inputs.has("form")
	var form: Dictionary = (inputs.form as Dictionary).duplicate(true) if inputs.get("form") is Dictionary \
		else form_from_craft(lf.form if lf.get("form") is Dictionary else craft, lf.get("form_sources", {}))
	if str(craft.get("name", "")) != "":
		form["name"] = str(craft.name)
	if craft.has("zone_vehicle_id"):
		form["zone_vehicle_id"] = str(craft.zone_vehicle_id)
	r["form"] = form
	var logs := []
	var stored := {}
	var da := DirAccess.open(dir.path_join("logs"))
	if da != null:
		for f in da.get_files():
			stored[f] = true
	if inputs.get("logs") is Array:
		for l in inputs.logs:
			logs.append({"name": str(l.name), "size": int(l.get("size", 0)), "sha256": str(l.get("sha256", "")), "stored": stored.has(str(l.name))})
	else:
		for f in lf.get("files", []):
			logs.append({"name": str(f.get("name", "")), "size": int(f.get("size", 0)), "sha256": str(f.get("sha256", "")), "stored": false})
	r["logs"] = logs
	r["logs_bytes"] = dir_bytes(dir.path_join("logs"))
	r["propwash"] = propwash_state(dir, str(r.origin), lf)
	return r


## The prop wash calibration of the package in dir: {text (the report's line), recalibrate (offer the button),
## why}; only an own model with a log metric can be recalibrated
func propwash_state(dir: String, org: String, lf: Dictionary) -> Dictionary:
	var plant := read_json(dir.path_join("plant.json"))
	var out := {"text": "", "recalibrate": false, "why": ""}
	if not (FileAccess.file_exists(CALIBRATE) or ResourceLoader.exists(CALIBRATE)):
		return out
	var Cal: Script = load(CALIBRATE)
	if plant.get("dirty_air_calibration") is Dictionary:
		out.text = Cal.summary(plant.dirty_air_calibration)
	var st: Dictionary = Cal.stale(plant)
	out.why = st.why
	out.recalibrate = st.recalibrate and org == "user" and lf.get("propwash") is Dictionary
	return out


## The window's form from a craft.json-layout dictionary (craft.json, or logfit.json's form); keys that
## were class defaults (form_sources "C") stay unset
static func form_from_craft(c: Dictionary, sources := {}) -> Dictionary:
	var f := {}
	var by_class := func(k: String) -> bool: return str(sources.get(k, "")) == "C"
	if c.get("name") != null:
		f["name"] = str(c.name)
	if c.get("mass_kg") != null:
		f["weight_g"] = snappedf(float(c.mass_kg) * 1000.0, 0.01)
	var bat: Dictionary = c.get("battery", {}) if c.get("battery") is Dictionary else {}
	var cells: Variant = c.get("cells", bat.get("cells"))
	if cells != null and not by_class.call("cells"):
		f["cells"] = int(cells)
	f["pack_type"] = "lihv" if bool(bat.get("lihv", false)) else str(c.get("pack", "lipo")) if str(c.get("pack", "lipo")) in ["lipo", "lihv", "liion"] else "lipo"
	if bat.get("capacity_mah") != null and not by_class.call("battery.capacity_mah"):
		f["capacity_mah"] = float(bat.capacity_mah)
	var motor: Dictionary = c.get("motor", {}) if c.get("motor") is Dictionary else {}
	if motor.get("kv") != null:
		f["kv"] = float(motor.kv)
	var prop: Dictionary = c.get("prop", {}) if c.get("prop") is Dictionary else {}
	for pair in [["diameter_in", "prop_in"], ["pitch_in", "pitch_in"], ["blades", "blades"]]:
		if prop.get(pair[0]) != null and not by_class.call("prop." + pair[0]):
			f[pair[1]] = int(prop[pair[0]]) if pair[1] == "blades" else float(prop[pair[0]])
	for k in ["wheelbase_mm", "top_speed_kmh"]:
		if c.get(k) != null and not by_class.call(k):
			f[k] = float(c[k])
	if c.get("ducted") != null and not by_class.call("ducted"):
		f["ducted"] = bool(c.ducted)
	if str(c.get("class", "")) != "":
		f["class"] = str(c["class"])
	f["zone_vehicle_id"] = str(c.get("zone_vehicle_id", ""))
	return f


# ------------------------------------------------------------------ the job

func available() -> bool:
	return _script() != null


func _script() -> Script:
	if logfit_script != null:
		return logfit_script
	if FileAccess.file_exists(LOGFIT) or ResourceLoader.exists(LOGFIT):
		var s = load(LOGFIT)
		if s is Script:
			return s
	return null


func busy() -> bool:
	return _job != null


## The model being built right now: {id, name, step, frac, elapsed_s, new, cancelling, recal} or {}
func current() -> Dictionary:
	if _job == null:
		return {}
	var s := _job.snapshot()
	return {"id": _job.id, "name": _job.name, "step": s.step, "frac": s.frac, "elapsed_s": (Time.get_ticks_msec() - _job.t0) / 1000,
		"new": _job.new_model, "cancelling": _job.cancelled, "recal": _job.recal, "tune": _job.tune}


## mod_settings' line: "Building <name>: <step> 45 %", else how the last job went for a while, else ""
func status() -> String:
	if _job != null:
		var c := current()
		return "%s %s: %s %d %%%s" % ["Recalibrating" if c.recal else "Tuning" if c.tune else "Building", c.name, c.step, int(round(float(c.frac) * 100.0)), " (cancelling)" if c.cancelling else ""]
	if not last.is_empty() and Time.get_ticks_msec() < notice_until_ms:
		return notice_text(last)
	return ""


static func notice_text(rec: Dictionary) -> String:
	if bool(rec.get("tune", false)):
		if bool(rec.get("cancelled", false)):
			return "Tuning %s was cancelled." % rec.name
		if bool(rec.get("ok", false)):
			return "%s: %s." % [rec.name, rec.get("text", "tuned twin made")]
		var w := ""
		for e in rec.get("errors", []):
			w = str(e.get("text", e)) if e is Dictionary else str(e)
			break
		return "Tuning %s failed%s" % [rec.name, ": " + w if w != "" else "."]
	if bool(rec.get("recal", false)):
		if bool(rec.get("cancelled", false)):
			return "Recalibrating the prop wash of %s was cancelled." % rec.name
		if bool(rec.get("ok", false)):
			return "%s: %s." % [rec.name, rec.get("text", "prop wash recalibrated")]
	if bool(rec.get("cancelled", false)):
		return "Building %s was cancelled." % rec.name
	if bool(rec.get("ok", false)):
		return "Model %s is built: grade %s." % [rec.name, rec.get("grade", "?")]
	var why := ""
	for e in rec.get("errors", []):
		why = str(e.get("text", e)) if e is Dictionary else str(e)
		break
	return "Building %s failed%s" % [rec.name, ": " + why if why != "" else "."]


func dismiss_last() -> void:
	last = {}


## Starts a job. spec: {id, form (the window's), add: [paths of logs to copy in], keep: names of the logs
## already in the model to keep (null = all; a new model has none), new: true for a model that does not
## exist yet}. "" = started, else why not.
func start(spec: Dictionary) -> String:
	if _job != null:
		var c := current()
		return "%s is being built (%s, %d %%): wait for it to finish, or cancel it first." % [c.name, c.step, int(round(float(c.frac) * 100.0))]
	if not available():
		return "The model builder is not installed completely: real_quad/logfit/logfit.gd is missing. Update the mod."
	var id := str(spec.get("id", ""))
	var form: Dictionary = (spec.get("form", {}) as Dictionary).duplicate(true)
	if id == "":
		id = slug(str(form.get("name", "")))
	if id == "":
		return "The model needs a name."
	var org := origin(id)
	var is_new: bool = bool(spec.get("new", false))
	if org == "shipped":
		return "\"%s\" is a model that comes with real_quad: duplicate it and edit the copy, or pick another name." % id
	if is_new and org != "":
		return "A model \"%s\" exists already: pick another name, or open it and rebuild." % id
	if not is_new and org == "":
		return "There is no model \"%s\" of yours to rebuild." % id
	var keep: Array = []
	var old_logs: Array = []
	var dir := model_dir(id)
	if not is_new:
		var inputs := read_json(dir.path_join(INPUTS))
		for l in inputs.get("logs", []):
			old_logs.append(l)
		if spec.get("keep") != null:
			keep = spec.keep
		else:
			keep = old_logs.map(func(l): return str(l.name))
	var add: Array = spec.get("add", [])
	var n_logs := keep.size() + add.size()
	if n_logs == 0:
		return "Add the logs of your quad first (the model has none stored)."
	form["name"] = str(form.get("name", id))
	_job = Job.new()
	_job.id = id
	_job.name = str(form.name)
	_job.new_model = is_new
	_job.dir = dir
	_job.t0 = Time.get_ticks_msec()
	_job.logfit = _script().new()
	last = {}
	var job := _job
	var me := self
	var old_dir := dir
	job.thread = Thread.new()
	job.thread.start(func():
		var r := me._run(job, id, form, add, keep, old_logs, old_dir, is_new)
		job.mutex.lock()
		job.result = r
		job.done = true
		job.mutex.unlock(), Thread.PRIORITY_LOW)
	return ""


## Starts a job that runs only the prop wash calibration of an own model (logfit.recalibrate()): its
## plant.json and report change in place, nothing else. "" = started, else why not.
func recalibrate(id: String) -> String:
	if _job != null:
		var c := current()
		return "%s is being built (%s, %d %%): wait for it to finish, or cancel it first." % [c.name, c.step, int(round(float(c.frac) * 100.0))]
	if not available():
		return "The model builder is not installed completely: real_quad/logfit/logfit.gd is missing. Update the mod."
	if origin(id) != "user":
		return "Only your own models can be recalibrated."
	var lf = _script().new()
	if not lf.has_method("recalibrate"):
		return "The model builder is too old to recalibrate: update the mod."
	var dir := model_dir(id)
	_job = Job.new()
	_job.id = id
	_job.name = str(read_json(dir.path_join("craft.json")).get("name", id))
	_job.recal = true
	_job.dir = dir
	_job.t0 = Time.get_ticks_msec()
	_job.logfit = lf
	last = {}
	var job := _job
	job.thread = Thread.new()
	job.thread.start(func():
		var r: Dictionary = job.logfit.recalibrate(dir, func(s: String, f: float): job.progress(s, f))
		if job.cancelled:
			r["cancelled"] = true
		job.mutex.lock()
		job.result = r
		job.done = true
		job.mutex.unlock(), Thread.PRIORITY_LOW)
	return ""


## Starts the autotune of a grade A/B model (logfit.autotune()); when it worked, the tuned twin <id>_tuned is
## written as a model of yours (an older twin of the same id is replaced). "" = started, else why not.
func autotune(id: String) -> String:
	if _job != null:
		var c := current()
		return "%s is being built (%s, %d %%): wait for it to finish, or cancel it first." % [c.name, c.step, int(round(float(c.frac) * 100.0))]
	if not available():
		return "The model builder is not installed completely: real_quad/logfit/logfit.gd is missing. Update the mod."
	var dir := model_dir(id)
	if origin(id) == "":
		return "Unknown model."
	if not _grade_of(dir) in ["A", "B"]:
		return "Only models of grade A or B can be tuned: their plant is identified well enough."
	var lf = _script().new()
	if not lf.has_method("autotune"):
		return "The model builder is too old to tune: update the mod."
	var craft := read_json(dir.path_join("craft.json"))
	_job = Job.new()
	_job.id = id
	_job.name = str(craft.get("name", id))
	_job.tune = true
	_job.dir = dir
	_job.t0 = Time.get_ticks_msec()
	_job.logfit = lf
	last = {}
	var job := _job
	var twin := slug(id).substr(0, 30) + "_tuned"
	job.thread = Thread.new()
	job.thread.start(func():
		var r: Dictionary = job.logfit.autotune(dir, func(s: String, f: float): job.progress(s, f))
		if job.cancelled:
			r["cancelled"] = true
		elif bool(r.get("ok", false)):
			r.merge(_write_twin(dir, twin, craft, r), true)
		job.mutex.lock()
		job.result = r
		job.done = true
		job.mutex.unlock(), Thread.PRIORITY_LOW)
	return ""


## Writes the tuned twin (worker thread): the model's craft.json (new id and name), plant.json and logfit.json,
## the tuned fc.json, report.md (the tune's, then the model's), autotune.json. {ok, twin_id, text} or {ok false, errors}
func _write_twin(src: String, twin: String, craft: Dictionary, r: Dictionary) -> Dictionary:
	var stage := work.path_join("stage").path_join(twin)
	remove_dir(stage)
	DirAccess.make_dir_recursive_absolute(stage)
	var c := craft.duplicate(true)
	c["id"] = twin
	c["name"] = ("%s (tuned)" % str(craft.get("name", twin))).substr(0, 40)
	c["tuned_from"] = str(craft.get("id", ""))
	var ok := write_json(stage.path_join("craft.json"), c) and write_json(stage.path_join("fc.json"), r.fc) \
		and copy_file(src.path_join("plant.json"), stage.path_join("plant.json")) and write_json(stage.path_join("autotune.json"), r.record)
	if FileAccess.file_exists(src.path_join("logfit.json")):
		ok = copy_file(src.path_join("logfit.json"), stage.path_join("logfit.json")) and ok
	var md := str(r.report_md)
	var i := md.find("\n")         # the grade near the top, as a build's report has it (the plant's)
	md = md.substr(0, i) + "\n\n## Grade %s: the plant's, identified from the logs\n" % _grade_of(src) + md.substr(i)
	var f := FileAccess.open(stage.path_join("report.md"), FileAccess.WRITE)
	if f != null:
		f.store_string(md + "\n---\n\n" + FileAccess.get_file_as_string(src.path_join("report.md")))
		f.close()
	var dst := root.path_join(twin)
	if not ok or f == null or not _files_parse(stage):
		remove_dir(stage)
		return {"ok": false, "errors": [{"text": "Cannot write the tuned twin (disk full?).", "fix": ""}]}
	remove_dir(dst)
	DirAccess.make_dir_recursive_absolute(root)
	if DirAccess.rename_absolute(stage, dst) != OK:
		remove_dir(stage)
		return {"ok": false, "errors": [{"text": "Cannot move the tuned twin into %s." % dst, "fix": ""}]}
	var g: Dictionary = r.record.tuned.groups
	var f0: Dictionary = r.record.flown.groups
	var key := "replay" if g.has("replay") else "steps"
	return {"ok": true, "twin_id": twin, "text": "tuned twin %s made (tracking error %s %.0f %%)" % [c.name,
		"of your sticks" if key == "replay" else "in stick steps", 100.0 * (float(g[key]) / maxf(float(f0[key]), 1e-6) - 1.0)]}


func cancel() -> void:
	if _job != null and not _job.cancelled:
		_job.cancelled = true
		_job.logfit.cancel()


# runs in the worker thread
func _run(job: Job, id: String, form: Dictionary, add: Array, keep: Array, old_logs: Array, old_dir: String, is_new: bool) -> Dictionary:
	var fail := func(text: String) -> Dictionary:
		remove_dir(work.path_join("stage").path_join(id))
		remove_dir(work.path_join("building").path_join(id))
		return {"ok": false, "errors": [{"text": text, "fix": ""}], "warnings": []}
	var stage := work.path_join("stage").path_join(id)
	remove_dir(stage)
	remove_dir(work.path_join("building").path_join(id))
	DirAccess.make_dir_recursive_absolute(stage)
	DirAccess.make_dir_recursive_absolute(work.path_join("building"))
	# the logs: kept ones in the model's logs/, new ones copied (staged) unless identical to one there
	job.progress("Copying the logs", 0.0)
	var entries: Array = []          # {name, size, sha256, path, staged}
	var seen := {}
	var notes: Array = []
	for l in old_logs:
		if str(l.name) in keep:
			var p := old_dir.path_join("logs").path_join(str(l.name))
			if not FileAccess.file_exists(p):
				return fail.call("The stored log %s is missing from the model's folder." % l.name)
			seen[str(l.sha256)] = str(l.name)
			entries.append({"name": str(l.name), "size": int(l.size), "sha256": str(l.sha256), "path": p, "staged": false})
	var names := {}                  # lower case: the file system may not tell Log.bbl from log.bbl
	for e in entries:
		names[str(e.name).to_lower()] = true
	for i in add.size():
		if job.cancelled:
			return fail.call("cancelled")
		var src := str(add[i])
		var fa := FileAccess.open(src, FileAccess.READ)
		if fa == null:
			return fail.call("Cannot read %s (%s)." % [src.get_file(), error_string(FileAccess.get_open_error())])
		var size := fa.get_length()
		fa = null
		var sha := FileAccess.get_sha256(src)
		if seen.has(sha):
			notes.append("%s is the same file as %s: used once." % [src.get_file(), seen[sha]])
			continue
		var nm := safe_file_name(src.get_file())
		if names.has(nm.to_lower()):
			nm = "%s_%s" % [sha.substr(0, 8), nm]
		names[nm.to_lower()] = true
		seen[sha] = nm
		if not copy_file(src, stage.path_join(nm)):
			return fail.call("Cannot copy %s into the model's folder (disk full?)." % src.get_file())
		entries.append({"name": nm, "size": size, "sha256": sha, "path": stage.path_join(nm), "staged": true})
		job.progress("Copying the logs", 0.05 * float(i + 1) / maxi(add.size(), 1))
	if entries.is_empty():
		return fail.call("No logs to build from.")
	entries.sort_custom(func(a, b): return str(a.name) < str(b.name))
	var paths := PackedStringArray()
	for e in entries:
		paths.append(ProjectSettings.globalize_path(e.path))
	var out := work.path_join("building").path_join(id)
	var fform := form.duplicate(true)
	fform.erase("overwrite")
	var prog := func(step: String, frac: float): job.progress(step, 0.05 + 0.95 * frac)
	var res: Dictionary = job.logfit.build(paths, fform, out, prog)
	res["notes"] = notes
	if job.cancelled:
		res = fail.call("cancelled")
		res["cancelled"] = true
		return res
	if not bool(res.get("ok", false)):
		remove_dir(stage)
		remove_dir(work.path_join("building").path_join(id))
		return res
	# the package is there: copies of the logs and inputs.json go in, it is checked, then it replaces the old one.
	# The old package keeps its logs until the swap has worked, so whatever goes wrong, nothing is lost.
	job.progress("Saving the model", 0.99)
	var logs_dir := out.path_join("logs")
	if DirAccess.make_dir_recursive_absolute(logs_dir) != OK:
		return fail.call("Cannot make the model's folder (disk full?).")
	for e in entries:
		var target := logs_dir.path_join(e.name)
		# a staged log is the copy this job made: it moves (the copy on disk is the only one needed); a kept one is copied
		var moved: bool = e.staged and DirAccess.rename_absolute(e.path, target) == OK
		if not moved and not copy_file(e.path, target):
			return fail.call("Cannot copy %s into the model's folder (disk full?)." % e.name)
	var inputs := {"inputs_version": 1, "form": fform, "built": Time.get_datetime_string_from_system(true) + "+00:00",
		"logs": entries.map(func(e): return {"name": e.name, "size": e.size, "sha256": e.sha256})}
	if not write_json(out.path_join(INPUTS), inputs):
		return fail.call("Cannot write the model's inputs.json (disk full?).")
	var probe: BFModel = BFModel.new(out) if _files_parse(out) else null
	if probe == null or not probe.valid:
		return fail.call("The new package cannot be loaded (some of its files were not written completely: disk full?). The old model is kept.")
	probe = null
	var aside := work.path_join("old").path_join(id)
	var moved_old := false
	if not is_new and DirAccess.dir_exists_absolute(old_dir):
		DirAccess.make_dir_recursive_absolute(work.path_join("old"))
		remove_dir(aside)
		if DirAccess.rename_absolute(old_dir, aside) != OK:
			return fail.call("Cannot replace the old package (%s)." % old_dir)
		moved_old = true
	elif is_new and DirAccess.dir_exists_absolute(old_dir):
		# somebody made a model of this name meanwhile (Duplicate): leave it alone
		return fail.call("A model \"%s\" appeared while this one was built: pick another name." % id)
	DirAccess.make_dir_recursive_absolute(root)
	if DirAccess.rename_absolute(out, old_dir) != OK:
		# nothing was placed, so there is nothing of the new package to remove at old_dir: put the old one back
		if moved_old:
			DirAccess.rename_absolute(aside, old_dir)
		return fail.call("Cannot put the new package in place (%s)." % old_dir)
	remove_dir(aside)
	remove_dir(stage)
	res["package_dir"] = old_dir
	return res


## The three files of a package are there and JSON objects (without the engine's error messages)
static func _files_parse(dir: String) -> bool:
	for f in ["craft.json", "fc.json", "plant.json"]:
		var p := dir.path_join(f)
		if not FileAccess.file_exists(p):
			return false
		var j := JSON.new()
		if j.parse(FileAccess.get_file_as_string(p)) != OK or not (j.data is Dictionary):
			return false
	return true


## Main thread, each frame while busy(): collects a finished job. True when one finished now.
func poll() -> bool:
	if _job == null:
		return false
	var s := _job.snapshot()
	if not s.done and not _job.thread.is_alive():
		# the worker is gone: it finished a moment ago (done is set before it ends), or it stopped with an
		# error in the builder's script before it could say so
		s = _job.snapshot()
		if not s.done:
			_job.result = _stopped(_job)
			s.done = true
	if not s.done:
		return false
	_job.thread.wait_to_finish()
	var job := _job
	_job = null
	var r: Dictionary = job.result
	var rec := {"id": job.id, "name": job.name, "new": job.new_model, "ok": bool(r.get("ok", false)), "grade": str(r.get("grade", "")),
		"cancelled": bool(r.get("cancelled", false)) or (job.cancelled and not bool(r.get("ok", false))), "errors": r.get("errors", []),
		"warnings": r.get("warnings", []), "notes": r.get("notes", []), "elapsed_s": (Time.get_ticks_msec() - job.t0) / 1000.0,
		"recal": job.recal, "tune": job.tune, "text": str(r.get("text", "")), "twin_id": str(r.get("twin_id", ""))}
	last = rec
	notice_until_ms = Time.get_ticks_msec() + 60000
	if host != null and host.has_method("models_changed"):
		host.models_changed(str(r.get("twin_id", job.id)))
	finished.emit(rec)
	return true


## The result of a job whose thread ended without one (an error in the builder's script): the leftovers go,
## and an old package the job had put aside comes back
func _stopped(job: Job) -> Dictionary:
	var id := job.id
	var aside := work.path_join("old").path_join(id)
	if not job.recal and not job.tune and not job.new_model and DirAccess.dir_exists_absolute(aside) and not DirAccess.dir_exists_absolute(job.dir):
		DirAccess.rename_absolute(aside, job.dir)
	remove_dir(work.path_join("stage").path_join(id))
	remove_dir(work.path_join("building").path_join(id))
	var what := "The prop wash calibration" if job.recal else "The autotune" if job.tune else "The build"
	return {"ok": false, "errors": [{"text": "%s stopped unexpectedly (an error in the model builder: see the game's log). The model is as it was." % what, "fix": ""}], "warnings": []}


## Waits for the job (tests, quitting)
func wait() -> void:
	while _job != null:
		if not poll():
			OS.delay_msec(10)


## The game is closing: cancels a running job and waits for its thread (a Thread must be waited for), without
## the notices of poll(). The job cleans up after itself when it is cancelled.
func shutdown() -> void:
	if _job == null:
		return
	cancel()
	_job.thread.wait_to_finish()
	var job := _job
	_job = null
	if not bool(job.snapshot().done):
		_stopped(job)


# ------------------------------------------------------------------ copy, delete, edit

## A free id near "<id>_copy"
func free_id(base: String) -> String:
	var b := slug(base)
	var cand := b.substr(0, 27) + "_copy"
	var n := 2
	while host.registry.has_model(cand) or DirAccess.dir_exists_absolute(root.path_join(cand)) or (_job != null and _job.id == cand):
		cand = "%s_copy_%d" % [b.substr(0, 25), n]
		n += 1
	return cand


## Copies a model (shipped or yours, with its logs) to a new model of yours. {ok, id, error}
func duplicate(id: String) -> Dictionary:
	if host == null or not host.registry.has_model(id):
		return {"ok": false, "error": "Unknown model."}
	if _job != null and _job.id == id:
		return {"ok": false, "error": "%s is being built: wait for it to finish, or cancel it." % _job.name}
	var src := model_dir(id)
	var nid := free_id(id)
	var dst := root.path_join(nid)
	if not copy_dir(src, dst):
		remove_dir(dst)
		return {"ok": false, "error": "Cannot copy the model's folder (disk full?)."}
	var craft := read_json(dst.path_join("craft.json"))
	var nm := "%s (copy)" % str(craft.get("name", id))
	craft["id"] = nid
	craft["name"] = nm.substr(0, 40)
	var written := write_json(dst.path_join("craft.json"), craft)
	for f in ["logfit.json", INPUTS]:
		var d := read_json(dst.path_join(f))
		if d.get("form") is Dictionary:
			d.form["id"] = nid
			d.form["name"] = craft.name
			written = write_json(dst.path_join(f), d) and written
	if not written:
		remove_dir(dst)
		return {"ok": false, "error": "Cannot write the copy's files (disk full?)."}
	if host.has_method("models_changed"):
		host.models_changed(nid)
	return {"ok": true, "id": nid}


## Deletes one of your models ("" = done, else why not). The drones that were set to it fly their default again.
func delete(id: String) -> String:
	if origin(id) != "user":
		return "Only your own models can be deleted." if origin(id) == "shipped" else "Unknown model."
	if _job != null and _job.id == id:
		return "%s is being built: cancel it first." % _job.name
	remove_dir(model_dir(id))
	if host != null and host.has_method("models_changed"):
		host.models_changed(id)
	# settings.cfg must not name a model that is gone (a model of the same id may still be there: a shipped one)
	if host != null and host.registry != null and not host.registry.has_model(id) and "drones" in host and host.has_method("set_drone_model"):
		for vid in host.drones.keys():
			if str(host.drones[vid].get("model", "")) == id:
				host.set_drone_model(str(vid), "auto")
	return ""


## Name and Zone drone only: written into the package at once, no rebuild. "" = done
func apply_meta(id: String, model_name: String, zone_vehicle_id: String) -> String:
	if origin(id) != "user":
		return "Only your own models can be changed."
	var dir := model_dir(id)
	var craft := read_json(dir.path_join("craft.json"))
	if craft.is_empty():
		return "The model's craft.json cannot be read."
	craft["name"] = model_name
	craft["zone_vehicle_id"] = zone_vehicle_id
	if not write_json(dir.path_join("craft.json"), craft):
		return "Cannot write the model's craft.json."
	var failed := ""
	for f in ["logfit.json", INPUTS]:
		var d := read_json(dir.path_join(f))
		if d.get("form") is Dictionary:
			d.form["name"] = model_name
			d.form["zone_vehicle_id"] = zone_vehicle_id
			if not write_json(dir.path_join(f), d) and failed == "":
				failed = "Cannot write the model's %s." % f
	if host != null and host.has_method("models_changed"):
		host.models_changed(id)
	return failed
