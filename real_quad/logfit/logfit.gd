extends RefCounted
## real_quad logfit: build a flight model from the pilot's own Betaflight blackbox logs (the entry point;
## the work is in fit/, see fit/build.gd for the steps). Runs in a worker Thread: no scene tree.
##
##   var lf = preload("res://real_quad/logfit/logfit.gd").new()
##   var info: Dictionary = lf.inspect(paths, pack_type = "")   # pack_type of the form if known (lipo | lihv | liion)
##       {ok, firmware (text), sessions [{file, index, armed_s, usable, reason}], fields [main field names
##        of the usable sessions], prefill {cells: {value, check, note} | null, kv: {value, check, note} | null,
##        motor_poles, craft_name, pack_type}, errors, warnings}
##   var res: Dictionary = lf.build(paths, form, out_dir, progress)
##       {ok, grade, errors, warnings, report_md, package_dir, model_id} (+ propwash_calibration: the prop
##        wash line of the report)
##   lf.cancel()                    # from any thread: build() returns within about a second, writes nothing
##                                  (a cancel that comes once the package is being moved in place is too late:
##                                  build() then answers ok); a cancel before a job starts cancels that job
##
## form (the window's, logfit/window.gd): name, weight_g, cells, pack_type (lipo | lihv | liion), kv,
## class, zone_vehicle_id and optionally capacity_mah, prop_in, pitch_in, blades, wheelbase_mm,
## top_speed_kmh, ducted, same_quad. The contract's layout (mass_kg, prop {diameter_in,
## pitch_in, blades}, battery {capacity_mah}) is read as well.
## out_dir is the package folder (e.g. user://real_quad/models/<id>); its name is the model id. It is
## written through a temporary folder next to the models root and replaces an existing package.
## Errors and warnings are {text, fix, big, overridable, code, level}; big = a banner (missing
## accelerometer, chirps or battery voltage), overridable = lifted by form.same_quad.
## progress.call(step_name: String, fraction: float) runs on the worker thread.

const Build := preload("fit/build.gd")
const Scan := preload("fit/scan.gd")
const Craft := preload("fit/craft.gd")

var _job: Build = null
var _cancel := false
var _mutex := Mutex.new()


func cancel() -> void:
	_mutex.lock()
	_cancel = true
	if _job != null:
		_job.cancel()
	_mutex.unlock()


## A pending cancel (cancel() before the job started) is not cleared: the new job starts cancelled.
func _new_job(progress: Callable) -> Build:
	_mutex.lock()
	_job = Build.new()
	_job.progress = progress
	if _cancel:
		_job.cancel_flag = true
	var j := _job
	_mutex.unlock()
	return j


## The job is over: true if it was cancelled (the cancel is used up, the next job runs).
func _end_job() -> bool:
	_mutex.lock()
	var was := _cancel or (_job != null and _job.cancel_flag)
	_cancel = false
	_job = null
	_mutex.unlock()
	return was


func inspect(paths: PackedStringArray, pack_type: String = "") -> Dictionary:
	var job := _new_job(Callable())
	var info := Scan.inspect(paths, job, pack_type)
	if _end_job():
		info.ok = false
		info.errors = [{"code": "cancelled", "level": "error", "message": "Cancelled.", "fix": ""}] + info.errors
	info.errors = info.errors.map(func(e): return message(e))
	info.warnings = info.warnings.map(func(e): return message(e))
	return info


func build(paths: PackedStringArray, form: Dictionary, out_dir: String, progress: Callable = Callable()) -> Dictionary:
	var job := _new_job(progress)
	var r := _build(job, paths, form, out_dir)
	var cancelled := _end_job()
	if cancelled and not bool(r.ok):
		r.errors = [{"text": "Cancelled.", "fix": "", "big": false, "overridable": false, "code": "cancelled", "level": "error"}]
		r.warnings = []
		r.report_md = ""
	return r


func _build(job: Build, paths: PackedStringArray, form: Dictionary, out_dir: String) -> Dictionary:
	var c := craft_from_form(form)
	var id := out_dir.get_file() if out_dir != "" else ""
	var re := RegEx.create_from_string("^[a-z0-9_]{1,32}$")
	if id != "" and re.search(id) != null:
		c.id = id
	var r := job.run(c, paths, out_dir, bool(form.get("same_quad", false)))
	var warns: Array = r.get("warnings", []).map(func(e): return message(e))
	for b in r.get("banners", []):
		warns.push_front({"text": b, "fix": "", "big": true, "overridable": false, "code": "banner", "level": "warning"})
	var out := {"ok": bool(r.ok), "grade": r.get("grade", ""), "errors": r.errors.map(func(e): return message(e)), "warnings": warns,
		"report_md": r.get("report_md", ""), "package_dir": r.get("package_dir", "") if r.ok else "", "model_id": str(r.get("model_id", c.get("id", "")))}
	if r.has("propwash_calibration"):
		out["propwash_calibration"] = r.propwash_calibration
	return out


## A pipeline issue as the window's message.
static func message(e: Dictionary) -> Dictionary:
	if e.has("text"):
		return e
	var code := str(e.get("code", ""))
	var big: bool = e.get("big", false) or code in ["no_acc", "no_chirp", "no_vbat"]
	return {"text": str(e.get("message", "")), "fix": str(e.get("fix", "")), "big": big,
		"overridable": code == "board_differs", "code": code, "level": str(e.get("level", "warning"))}


## The window's form -> the craft dictionary of the pipeline (craft.json layout).
static func craft_from_form(form: Dictionary) -> Dictionary:
	var c := {}
	var num := func(k: String) -> Variant:
		var v: Variant = form.get(k)
		if v == null or (v is String and (v as String).strip_edges() == ""):
			return null
		return float(v)
	if form.get("name") != null and str(form.name).strip_edges() != "":
		c.name = str(form.name).strip_edges()
		c.id = Craft.slug(c.name)
	if form.get("mass_kg") != null:
		c.mass_kg = float(form.mass_kg)
	elif num.call("weight_g") != null:
		c.mass_kg = float(num.call("weight_g")) / 1000.0
	var pack := str(form.get("pack_type", form.get("pack", "lipo")))
	c.pack = pack
	var bat: Dictionary = (form.get("battery") as Dictionary).duplicate() if form.get("battery") is Dictionary else {}
	if num.call("cells") != null:
		c.cells = int(num.call("cells"))
		bat.cells = c.cells
	if num.call("capacity_mah") != null:
		bat.capacity_mah = num.call("capacity_mah")
	bat.lihv = pack == "lihv"
	c.battery = bat
	if num.call("kv") != null:
		c.motor = {"kv": num.call("kv")}
	var prop: Dictionary = (form.get("prop") as Dictionary).duplicate() if form.get("prop") is Dictionary else {}
	for pair in [["prop_in", "diameter_in"], ["pitch_in", "pitch_in"], ["blades", "blades"]]:
		if num.call(pair[0]) != null:
			prop[pair[1]] = num.call(pair[0])
	if prop.has("blades"):
		prop.blades = int(prop.blades)
	c.prop = prop
	for k in ["wheelbase_mm", "top_speed_kmh"]:
		if num.call(k) != null:
			c[k] = num.call(k)
	if form.get("ducted") != null:
		c.ducted = bool(form.ducted)
	if form.get("class") != null and str(form["class"]) != "":
		c["class"] = str(form["class"])
	if form.get("zone_vehicle_id") != null and str(form.zone_vehicle_id) != "":
		c.zone_vehicle_id = str(form.zone_vehicle_id)
	return c
