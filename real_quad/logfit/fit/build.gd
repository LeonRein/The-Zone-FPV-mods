extends RefCounted
## real_quad logfit: the pipeline, form + blackbox logs -> model package (contract, all steps):
##   0 files, sessions, checks  every session of every file in canonical order; firmware, required
##                              content, skip rules, consistency, form defaults (class), sensor sanity
##   1 segment                  usable airborne windows, chirp windows; minimum airborne time
##   2 fc_config                Betaflight settings from the newest session's header
##   3 motor                    motor/ESC model per motor, dynamics refinement (eRPM), speed and dead time from the gyro (chirps)
##   4 hover / kT               thrust coefficient from hover with the craft mass
##   5 rotation                 inertia, rotor inertia, yaw direction, rate damping (IV, chirps)
##   6 bias                     torque imbalance per thrust
##   7 battery                  V0 and sag
##   8 drag                     rotor and body drag (ACC), else top speed, else class default
##   9 propwash                 log metric (the calibration's targets)
##  9b prop wash calibration    the own wake's and the calm-air turbulence, flown in dirty_air's air
##                              (calibrate.gd; after the validation, which does not depend on them)
##  10 validate, coverage, grade
##  9c optimized tune           grade A/B: autotune.gd (simulator only), after the grade
##     report, package (craft.json format 2, fc.json, plant.json, report.md, logfit.json; fc_optimized.json
##     and tune.md with an optimized tune)
## run() never touches the scene tree (it runs in a worker Thread); a refusal writes nothing.
## cancel_flag is polled between steps and inside the long loops.

const Num := preload("num.gd")
const Flight := preload("flight.gd")
const Checks := preload("checks.gd")
const Craft := preload("craft.gd")
const Classes := preload("classes.gd")
const Segments := preload("segments.gd")
const FcConfig := preload("fc_config.gd")
const Motor := preload("motor.gd")
const Thrust := preload("thrust.gd")
const Rotation := preload("rotation.gd")
const Bias := preload("bias.gd")
const Battery := preload("battery.gd")
const Drag := preload("drag.gd")
const Propwash := preload("propwash.gd")
const Calibrate := preload("calibrate.gd")
const Autotune := preload("autotune.gd")
const Validate := preload("validate.gd")
const Grade := preload("grade.gd")
const Report := preload("report.gd")
const BBL := preload("../bbl_decoder.gd")
const Controller := preload("../../flightmodel/bf_controller.gd")

const LOGFIT_VERSION := "1.0.0"
const PLANT_KEYS := ["mass", "kv_rad", "motor_a", "motor_b", "motor_c", "motor_delay_s", "kT", "arm", "Ixx", "Iyy", "Izz", "kQ",
	"J_rotor", "yaw_spin", "rate_damping", "pitch_m_per_rad", "cd_area", "rotor_drag", "linear_drag_ref_sum_w",
	"torque_bias_per_thrust", "battery_v0", "battery_r", "prop_radius", "turb_base_ms", "pw_turb_ms", "pw_noise_hz",
	"seed", "_comment"]
# (plant.json also gets "dirty_air_calibration", calibrate.gd's record, after them)
const MIN_CHIRP_S := 30.0
const MIN_AIRBORNE_S := 60.0
const SOFT_AIRBORNE_S := 180.0
const MIN_MOTOR_S := 30.0

var progress := Callable()           # progress.call(step_name: String, fraction: float)
var cancel_flag := false
var steps := {}                      # every step's results (for the parity tests and logfit.json)
var _decoders: Array = []
var _cal: Calibrate = null
var _at: Autotune = null
var _mutex := Mutex.new()


func cancel() -> void:
	cancel_flag = true
	_mutex.lock()
	for d in _decoders:
		d.cancel()
	if _cal != null:
		_cal.cancel()
	if _at != null:
		_at.cancel()
	_mutex.unlock()


func _cancelled() -> bool:
	return cancel_flag


func _say(step: String, frac: float) -> void:
	if progress.is_valid():
		progress.call(step, frac)


static func canonical_order(paths: PackedStringArray) -> PackedStringArray:
	var a: Array = Array(paths)
	a.sort_custom(func(x: String, y: String) -> bool:
		var bx := x.get_file()
		var by := y.get_file()
		if bx != by:
			return bx < by
		return x < y)
	return PackedStringArray(a)


# ------------------------------------------------------------- decoding --

## Decodes the files (in parallel, one task per file), in canonical order:
## [{name, size, sha256, path, sessions: [Flight], error ("" | csv | unreadable | not_blackbox), why}].
## Session ids are made unique (unique_names()).
func load_files(paths: PackedStringArray, from_frac: float = 0.0, to_frac: float = 0.02) -> Array:
	var out: Array = []
	for p in paths:
		out.append({"path": p, "name": p.get_file(), "size": 0, "sha256": "", "sessions": [], "error": "", "why": ""})
	var done := [0]
	Num.parallel_for(paths.size(), func(i: int) -> void:
		_load_one(out[i])
		_mutex.lock()
		done[0] += 1
		_say("Reading the logs", from_frac + (to_frac - from_frac) * done[0] / paths.size())
		_mutex.unlock())
	unique_names(out)
	return out


## Session ids "<stem>#<n>" collide for files of the same name (stem) from different folders: the sessions
## of every such file are named "<stem>-<first 8 hex of the file's SHA-256>#<n>" instead (identical files
## twice: "-1", "-2" in canonical order, i.e. by full path). files: load_files()'s entries in canonical
## order. Deterministic and independent of the input order; ids without a clash stay "<stem>#<n>".
static func unique_names(files: Array) -> void:
	var stems := {}
	for e in files:
		if e.sessions.is_empty():
			continue
		var st: String = str(e.path).get_file().get_basename()
		if not stems.has(st):
			stems[st] = []
		stems[st].append(e)
	for st in stems:
		var group: Array = stems[st]
		if group.size() < 2:
			continue
		var tags: Array = group.map(func(e): return "%s-%s" % [st, str(e.sha256).substr(0, 8)])
		var seen := {}
		for k in group.size():
			var tag: String = tags[k]
			if tags.count(tag) > 1:
				seen[tag] = int(seen.get(tag, 0)) + 1
				tag = "%s-%d" % [tag, seen[tag]]
			for L: Flight in group[k].sessions:
				L.name = "%s#%d" % [tag, L.session]


## The error of a file that cannot be opened (contract 2.1, 5.7): its own text, not "not a blackbox log".
## why: "permission denied", "file not found" or "read error"; a denied read on macOS is its privacy prompt.
static func unreadable_issue(name: String, why: String, macos: Variant = null) -> Dictionary:
	var mac: bool = OS.get_name() == "macOS" if macos == null else bool(macos)
	if mac and why == "permission denied":
		return Checks.issue("unreadable", "error", "%s: macOS did not allow reading this folder: allow it or copy the logs elsewhere" % name,
			"System Settings → Privacy & Security → Files and Folders: allow the game, or copy the logs to another folder")
	return Checks.issue("unreadable", "error", "%s: the file cannot be read (%s)" % [name, why],
		"allow reading the file (its permissions) or copy the logs to another folder")


## The refusal of the files that could not be read or are no blackbox logs ([] if all are fine).
static func file_errors(files: Array) -> Array:
	var errs: Array = []
	for e in files:
		if e.error == "unreadable":
			errs.append(unreadable_issue(e.name, e.why))
		elif e.error == "csv":
			errs.append(Checks.issue("csv_input", "error", "%s: Blackbox Explorer CSV exports are not accepted" % e.name,
				"pick the original .bbl/.bfl/.txt file from the flight controller's flash or SD card"))
		elif e.error == "not_blackbox":
			errs.append(Checks.issue("not_blackbox", "error", "%s: not a Betaflight blackbox log (no log header found)" % e.name,
				"pick the .bbl/.bfl/.txt file from the flight controller's flash or SD card"))
	return errs


## Why a file could not be opened: "permission denied", "file not found" or "read error" (call right after the failed open).
static func open_why(p: String) -> String:
	var oe := FileAccess.get_open_error()
	# (Godot reports a denied open of an existing file as ERR_FILE_CANT_OPEN on Linux)
	if oe == ERR_FILE_NOT_FOUND or not FileAccess.file_exists(p):
		return "file not found"
	if oe == ERR_FILE_NO_PERMISSION or oe == ERR_FILE_CANT_OPEN:
		return "permission denied"
	return "read error"


func _load_one(e: Dictionary) -> void:
	var p: String = e.path
	# readable at all (permissions, macOS privacy): before anything else
	var fa := FileAccess.open(p, FileAccess.READ)
	if fa == null:
		e.error = "unreadable"
		e.why = open_why(p)
		return
	e.size = fa.get_length()
	fa.close()
	if p.to_lower().ends_with(".csv"):
		e.error = "csv"
		return
	if cancel_flag:
		return
	e.sha256 = FileAccess.get_sha256(p)
	var dec := BBL.new()
	_mutex.lock()
	_decoders.append(dec)
	_mutex.unlock()
	if cancel_flag:
		return
	var ss: Array = dec.decode(p)
	_mutex.lock()
	_decoders.erase(dec)
	_mutex.unlock()
	if ss.is_empty():
		e.error = "not_blackbox"
		return
	for k in ss.size():
		if cancel_flag:
			return
		var L := Flight.new()
		L.load(p, ss[k], k)
		L.info.hash = ("%s#%d" % [e.sha256, L.session]).sha256_text()
		ss[k] = null
		e.sessions.append(L)


# ------------------------------------------------------------- the run --

static func _refuse(errs: Array, sessions: Variant = null) -> Dictionary:
	return {"ok": false, "errors": errs, "sessions": sessions}


static func _session_row(L: Flight, used: bool, reason: Variant) -> Dictionary:
	return {"id": L.name, "file": L.file, "path": L.path, "session": L.session, "firmware": Checks.parse_firmware(L.headers)[3],
		"armed_s": Num.pyround(float(L.armed.count(1)) / L.fs, 1) if L.n > 0 else 0.0,
		"used": used, "reason": reason, "acc": L.has_acc(), "vbat": L.has_vbat(),
		"unfilt": bool(L.info.flags.get("unfilt", false)), "log_hz": Num.pyround(L.fs_raw, 1)}


## The pipeline. c: the craft (form, reference layout, see craft.gd); paths: the logs; out_dir: the
## package folder to write (nothing is written if empty). Returns {ok, errors, warnings, grade, ...}.
func run(c: Dictionary, paths: PackedStringArray, out_dir: String, same_quad: bool = false) -> Dictionary:
	var t_start := Time.get_ticks_msec()
	steps = {}
	c = c.duplicate(true)
	var form_errs := Craft.validate_form(c)
	if not form_errs.is_empty():
		return _refuse(form_errs.map(func(e): return Checks.issue(e[0], "error", e[1], e[2])))
	var pack := Craft.pack_type(c)
	c._pack = pack
	var warns: Array = []
	paths = canonical_order(paths)
	if paths.is_empty():
		return _refuse([Checks.issue("no_logs", "error", "no log files found for []", "pick one or more .bbl/.bfl/.txt files")])

	# 0 files and sessions
	_say("Reading the logs", 0.0)
	var files_in := load_files(paths)
	if cancel_flag:
		return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
	var errs: Array = []
	var files: Array = []
	var all_s: Array = []
	for e in files_in:
		files.append({"name": e.name, "size": e.size, "sha256": e.sha256})
		if e.error == "":
			all_s.append_array(e.sessions)
	errs = file_errors(files_in)
	if not errs.is_empty():
		return _refuse(errs)
	_say("Checking the logs", 0.02)
	var reason := {}
	for L: Flight in all_s:
		reason[L.name] = Checks.skip_reason(L)
	var cand: Array = all_s.filter(func(L): return reason[L.name] == null)
	for L: Flight in cand:
		var e: Variant = Checks.firmware_issue(L)
		if e != null:
			errs.append(e)
		var nm: int = L.info.get("n_motors", 0)
		if nm != 0 and nm != 4:
			errs.append(Checks.issue("motor_count", "error", "only quads with 4 motors (%s logs %d motors)" % [L.name, nm], "pick logs of a quad"))
	if not errs.is_empty():
		return _refuse(errs)
	var fails := {}
	for L: Flight in cand:
		fails[L.name] = Checks.required_failures(L)
	for item in Checks.REQUIRED_TEXT:
		var bad: Array = cand.filter(func(L): return fails[L.name].has(item))
		if not cand.is_empty() and bad.size() == cand.size():
			var parts: Array = bad.map(func(L): return "%s: %s" % [L.name, fails[L.name][item]])
			errs.append(Checks.issue("missing_" + item, "error", "%s: %s" % [Checks.REQUIRED_TEXT[item], "; ".join(parts)], Checks.FIX[item]))
		for L: Flight in bad:
			if reason[L.name] == null:
				reason[L.name] = "%s (other sessions have it)" % fails[L.name][item]
	if not errs.is_empty():
		return _refuse(errs)
	var used: Array = cand.filter(func(L): return reason[L.name] == null)
	if used.is_empty():
		var parts: Array = []
		for k in reason:
			parts.append("%s: %s" % [k, str(reason[k])])
		return _refuse([Checks.issue("no_usable_session", "error", "no usable session: " + "; ".join(parts),
			"fly at least one flight of 5 s or more with the quad armed")])
	for L: Flight in used:
		var f: Dictionary = L.info.flags
		if not f.setpoint:
			Checks.rebuild_setpoint(L, Controller, FcConfig.from_headers(L.headers, "")[0])
			warns.append(Checks.issue("setpoint_rebuilt", "warning", "%s: setpoint not logged: rebuilt from rcCommand through the rates (chirps count as missing)" % L.name,
				Checks.FIX.setpoint_rc, L.name))
		elif not f.rc:
			Checks.rebuild_rc(L, Controller, FcConfig.from_headers(L.headers, "")[0])
			warns.append(Checks.issue("rc_rebuilt", "warning", "%s: RC not logged: the validation replay injects the setpoint behind the rates" % L.name,
				Checks.FIX.setpoint_rc, L.name))
	var armed_total := 0.0
	for L: Flight in used:
		armed_total += float(L.armed.count(1)) / L.fs
	if armed_total > Checks.MAX_ARMED_S:
		return _refuse([Checks.issue("too_much", "error", "%d min of armed flight (at most 60 min per run): pick fewer logs" % roundi(armed_total / 60.0), "pick fewer logs")])
	for e in Checks.consistency(used, same_quad, pack):
		(errs if e.level == "error" else warns).append(e)
	if not errs.is_empty():
		return _refuse(errs)

	# form: name, id, KV, cells, class, defaults
	var src_hdr := FcConfig.pick_source(used)
	var sources := {}
	var hdr_name := str(src_hdr.headers.get("Craft name", "")).strip_edges()
	if not Craft._truthy(c.get("name")):
		c.name = hdr_name if hdr_name != "" else used[0].stem
	if not Craft._truthy(c.get("id")):
		c.id = Craft.slug(c.name)
	var kv := 0.0
	if c.get("motor") is Dictionary and Craft._truthy(c.motor.get("kv")):
		kv = float(c.motor.kv)
		sources.kv = "U"
	else:
		kv = float(src_hdr.hint("motor_kv", 0))
		sources.kv = "header"
		if kv == 0.0:
			return _refuse([Checks.issue("form_kv", "error", "motor KV is required (the header has no motor_kv)", "enter the motor KV")])
		warns.append(Checks.issue("kv_from_header", "warning", "motor KV %d taken from the header" % roundi(kv) +
			(" (the Betaflight default 1960: check)" if int(kv) == 1960 else " (check)"), "enter the motor KV"))
	if not c.get("motor") is Dictionary:
		c.motor = {}
	c.motor.kv = kv
	var ncell := Craft.cells(c)
	if ncell > 0:
		sources.cells = "U"
	else:
		var v0 := -1.0
		for L: Flight in used:
			var v := Checks.resting_voltage(L)
			if v > 0.0:
				v0 = v
				break
		if v0 < 0.0:
			return _refuse([Checks.issue("form_cells", "error", "cell count is required (no battery voltage in the logs to infer it)", "enter the cell count")])
		var g := Checks.cells_guess(v0, pack)
		ncell = g[0]
		if ncell == 0:
			return _refuse([Checks.issue("form_cells", "error", "cell count cannot be inferred from the resting voltage %.2f V" % v0, "enter the cell count")])
		sources.cells = "logs"
		warns.append(Checks.issue("cells_inferred", "warning", "%dS inferred from the resting voltage %.2f V" % [ncell, v0] + (" (several cell counts fit: check)" if g[1] else ""),
			"enter the cell count"))
	c._cells = ncell
	c._kv = kv
	var rough_thr := {}
	for L: Flight in used:
		rough_thr[L.name] = Segments.rough_hover_throttle(L)
	var cls := ""
	if c.get("class") != null:
		cls = c["class"]
		sources["class"] = "U"
	else:
		var hz := PackedFloat64Array()
		for L: Flight in used:
			if not L.has_hz():
				continue
			var per := PackedFloat64Array()
			var rt: float = rough_thr[L.name]
			for i in L.n:
				if L.armed[i] != 0 and absf(L.throttle[i] - rt) < 0.03:
					per.append((((L.motor_hz[0][i] + L.motor_hz[1][i]) + L.motor_hz[2][i]) + L.motor_hz[3][i]) / 4.0)
			if per.size() > 100:
				hz.append(Num.median(per))
		cls = Classes.estimate(float(c.mass_kg) * 1000.0, Num.median(hz) if hz.size() > 0 else 0.0)
		sources["class"] = "estimated"
		warns.append(Checks.issue("class_estimated", "info", "craft class '%s' estimated from the weight" % cls + (" and the hover rotor speed" if hz.size() > 0 else "") + ": check", ""))
	Craft.resolve(c, cls, sources)
	var K: Dictionary = Classes.CLASSES[cls]
	var vfull: float = Craft.FULL_V[pack] * ncell
	var erpm_limit := 1.3 * kv * vfull / 60.0

	# sensor plausibility
	var san := {}
	var sane := Num.parallel_for(used.size(), func(k: int) -> Dictionary:
		return Checks.sanity(used[k], rough_thr[used[k].name], ncell, pack, erpm_limit, _cancelled))
	if cancel_flag:
		return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
	for k in used.size():
		var L: Flight = used[k]
		var r: Dictionary = sane[k]
		san[L.name] = r.metrics
		warns.append_array(r.issues)
		if r.drop_acc:
			L.acc = []
		if r.drop_vbat:
			L.vbat = PackedFloat64Array()
		if r.skip != null:
			reason[L.name] = r.skip
		elif r.skip_fit != null:
			reason[L.name] = r.skip_fit + " (not used for the fits)"
	used = used.filter(func(L): return reason[L.name] == null)
	var rows_tbl: Array = all_s.map(func(L): return _session_row(L, reason[L.name] == null, reason[L.name]))
	if used.is_empty():
		var parts: Array = []
		for k in reason:
			if reason[k] != null:
				parts.append("%s: %s" % [k, reason[k]])
		return _refuse([Checks.issue("no_usable_session", "error", "no usable session: " + "; ".join(parts), "check the warnings of each session")], rows_tbl)
	var LL: Array = used
	var rep := Report.new(c)
	rep.sessions = rows_tbl
	var geo := Craft.geometry(c)
	var mass := float(c.mass_kg)
	var kv_rad := kv * 2.0 * PI / 60.0
	var shown := {"prop.diameter_in": "%s\"" % str(c.prop.diameter_in), "prop.pitch_in": "%s\"" % ("%.2f" % (geo.pitch_m_per_rad * 2.0 * PI / 0.0254)),
		"prop.blades": str(c.prop.blades), "battery.capacity_mah": "%s mAh" % str(c.battery.capacity_mah), "ducted": "yes" if c.ducted else "no"}
	for pair in [["prop.diameter_in", "class default"], ["prop.pitch_in", "class pitch ratio × D"], ["prop.blades", "class default"],
			["battery.capacity_mah", "class default"], ["ducted", "class default"]]:
		if sources.get(pair[0]) == "C":
			rep.default(pair[0], shown[pair[0]], pair[1])
	if geo.arm_assumed:
		rep.default("Motor distance (arm)", "%d mm" % roundi(geo.arm * 1e3), "80 mm × D/5.1\" (only the absolute inertias depend on it)")
	steps["00_input"] = {"pack": pack, "cells": ncell, "kv": kv, "class": cls, "sources": sources, "files": files,
		"sessions": all_s.map(func(x): return {"id": x.name, "reason": reason[x.name]}), "erpm_limit_hz": erpm_limit,
		"rough_hover_throttle": rough_thr, "sanity": san}
	if cancel_flag:
		return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])

	# 1 segment
	_say("Finding the usable flight", 0.03)
	var segs := Num.parallel_for(LL.size(), func(k: int) -> Dictionary: return Segments.segment_log(LL[k], erpm_limit, _cancelled))
	if cancel_flag:
		return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
	var uses: Array = []
	for k in LL.size():
		uses.append(Segments.mask_from_windows(LL[k].t, segs[k].use))
	steps["01_segments"] = segs
	rep.segments(LL, segs)
	var air_s := 0.0
	for k in LL.size():
		air_s += float((uses[k] as PackedByteArray).count(1)) / LL[k].fs
	if air_s < MIN_AIRBORNE_S:
		return _refuse([Checks.issue("min_airborne", "error", "only %d s of usable airborne flight (at least 60 s needed)" % roundi(air_s),
			"add logs with more flying (3 min or more recommended)")], rows_tbl)
	if air_s < SOFT_AIRBORNE_S:
		warns.append(Checks.issue("soft_airborne", "warning", "%d s of usable airborne flight (180 s recommended)" % roundi(air_s), "add logs with more flying"))

	# 2 FC config
	_say("Betaflight settings", 0.04)
	var src := FcConfig.pick_source(LL)
	var fr := FcConfig.from_headers(src.headers, c.name)
	var cfg: Dictionary = fr[0]
	var missing: Array = fr[1]
	cfg.motor_kv = int(kv)
	if not missing.is_empty():
		rep.warn("fc_header", "fc.json", "missing in the header: " + ", ".join(missing))
	var diff := FcConfig.tune_differences(LL)
	if not diff.is_empty():
		warns.append(Checks.issue("tune_differs", "info", "the tune differs between the logs (%s): the plant does not depend on the tune; fc.json from %s" % [", ".join(diff.keys()), src.name], ""))
	rep.fc(cfg, src.name, diff)
	steps["02_fc"] = {"source": src.name, "missing": missing, "tune_differences": diff}

	# 3 motor
	_say("Motor and ESC model", 0.04)
	var rows := Num.parallel_for(LL.size(), func(k: int) -> Dictionary: return Motor.prep(LL[k], uses[k], Motor.DELAY_S, _cancelled))
	if cancel_flag:
		return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
	var motor_s := 0.0
	for k in LL.size():
		motor_s += float((rows[k].ok as PackedByteArray).count(1)) / LL[k].fs
	if motor_s < MIN_MOTOR_S:
		return _refuse([Checks.issue("min_motor", "error", "only %d s with all motors running and valid eRPM (at least 30 s needed)" % roundi(motor_s),
			"add logs with more flying; check the eRPM telemetry")], rows_tbl)
	var mf := Motor.fit(rows, kv_rad, 4.0 * ncell, 10, _cancelled)
	if mf.is_empty():
		return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
	var P0: Array = mf[0]
	var V: Dictionary = mf[1]
	var r2: float = mf[2]
	_say("Motor dynamics", 0.065)
	var rd := Motor.refine_dynamics(rows, P0, V, kv_rad, _cancelled)
	if rd.is_empty():
		return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
	var lam: float = rd[0]
	var P := Motor.scaled(P0, lam)
	var sims := Num.parallel_for(rows.size(), func(k: int) -> Array: return Motor.simulate(rows[k], P, V, kv_rad, PackedFloat64Array(), 4, _cancelled))
	if cancel_flag:
		return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
	var frh := {}
	for k in LL.size():
		var e: PackedFloat64Array = Motor.free_run_error_hz(rows[k], sims[k])[0]
		frh[LL[k].name] = Num.mean(e)
	_say("Motor response", 0.07)
	var gain := Motor.response_gain(rows, sims, [5, 10, 20, 30], _cancelled)
	if cancel_flag:
		return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
	var chirp_masks: Array = []
	var chirp_s := 0.0
	for k in LL.size():
		var wins: Array = segs[k].chirps.map(func(ch): return [ch[0], ch[0] + ch[1]])
		var m := Segments.mask_from_windows(LL[k].t, wins)
		chirp_masks.append(m)
		chirp_s += float(m.count(1)) / LL[k].fs
	var use_chirp := chirp_s >= MIN_CHIRP_S
	var delay := Motor.DELAY_S
	var torque: Variant = null
	var R := Motor.torque_path(rows, sims, LL, chirp_masks, Motor.TP_FREQS, _cancelled) if use_chirp else {}
	if cancel_flag:
		return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
	if not R.is_empty() and not (R.re + R.im + R.coh).any(func(c): return Array(c).any(func(v): return is_nan(v))):
		# the speed and dead time from the gyro (the eRPM lags), the steady state stays
		var tau0 := Num.mean(Motor.time_constant(P, R.w, kv_rad))
		var tr := Motor.torque_fit(R, tau0)
		delay = Num.pyround(clampf(Motor.DELAY_S + tr[1], 0.0, Motor.DELAY_MAX_S), 4)
		P = Motor.scaled(P, tr[0])
		sims = Num.parallel_for(rows.size(), func(k: int) -> Array:
			var r: Dictionary = rows[k].duplicate()
			r.u = (LL[k] as Flight).u().map(func(c): return Motor.delayed(c, delay, LL[k].fs))
			return Motor.simulate(r, P, V, kv_rad, PackedFloat64Array(), 4, _cancelled))
		if cancel_flag:
			return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
		torque = {"lambda": tr[0], "delay_s": delay, "tau0_s": tau0, "R": [R.re, R.im], "coh": R.coh, "w": R.w}
	rep.motor(P0, P, lam, rd[1], rd[2], r2, V, gain, frh, torque)
	steps["03_motor"] = {"P0": P0, "V": V, "r2": r2, "lambda": lam, "rms_before": rd[1], "rms_after": rd[2], "P": P, "free_run_hz": frh,
		"gain": gain, "torque": torque, "motor_s": motor_s}
	if cancel_flag:
		return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])

	# 4 hover / kT
	_say("Hover and thrust", 0.1)
	var model_v: Array = rows.map(func(r): return Motor.model_v(r, V))
	var hv: Variant = Thrust.hover(LL, uses, mass, model_v, _cancelled)
	if cancel_flag:
		return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
	if hv == null:
		return _refuse([Checks.issue("min_hover", "error", "no hover found: at least 2 s of steady hover in one session are needed for the thrust coefficient",
			"fly a 10 s hands-off hover in ANGLE")], rows_tbl)
	var h: Dictionary = hv
	var kT: float = h.kT
	var steady := Thrust.steady_estimate(LL, uses, _cancelled)
	rep.hover(h, steady)
	rep.motor_free_run(rd[2], h.hz)
	if rep.hover_spread > 0.06:
		warns.append(Checks.issue("hover_spread", "warning", "hover rotor speed differs by %.1f %% between sessions: weight, props or pack probably differ between logs" % (rep.hover_spread * 100.0),
			"use logs of one configuration (GoPro on/off, props, packs)"))
	steps["04_thrust"] = {"hover": h, "steady": steady}

	# 5 rotation
	_say("Rotation", 0.105)
	var band: Array = Rotation.BAND
	var bcol := PackedFloat64Array()
	for m in 4:
		bcol.append(P[m][1])
	var fa := Rotation.fit_all(LL, sims, bcol, uses, chirp_masks, segs, use_chirp, band, _cancelled)
	if fa.is_empty() or cancel_flag:
		return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
	var fit: Dictionary = fa[0]
	var yaw_src: String = fa[3]
	var miss_ax: Array = Rotation.AX.slice(0, 2).filter(func(ax): return not fit.has(ax))
	if not miss_ax.is_empty():
		return _refuse([Checks.issue("min_rotation", "error", "rotation fit failed: fewer than 100 usable samples in windows of 3 s or more on " + ", ".join(miss_ax),
			"add logs with more flying (chirp sweeps recommended)")], rows_tbl)
	if not fit.has("yaw"):          # no yaw excitation at all (or no data): J_rotor and the direction from the defaults
		fit.yaw = Rotation.no_fit()
	var dflt := Craft.defaults(c, kT)
	var bad_inertia := Rotation.inertia_problem(fit, kT, geo.arm, dflt.Ixx)
	if bad_inertia != "":
		return _refuse([Checks.issue("min_rotation", "error", "rotation fit failed: " + bad_inertia,
			"add logs with more flying (chirp sweeps recommended); check weight and prop size in the form")], rows_tbl)
	var rev_hdr := src.hint("yaw_motors_reversed", -1)      # (not in today's headers: then fc_config's default, props out)
	var rev_default: bool = bool(rev_hdr) if rev_hdr == 0 or rev_hdr == 1 else bool(cfg.yaw_motors_reversed)
	var tp := Rotation.to_params(fit, kT, geo.arm, Craft.REFERENCE.Izz_over_Ixy, use_chirp, dflt.J_rotor, rev_default,
		1 if use_chirp and yaw_src == "chirp" else 0)
	var rp: Dictionary = tp[0]
	var yaw_rev: bool = tp[1]
	var yaw_ok: bool = tp[2]
	if not yaw_ok:
		rep.default("J_rotor (yaw)", "%s kg·m²" % Report.fmt_g(dflt.J_rotor, 3), "yaw fit too uncertain (R² < 0.5 or rel. std. error > 25 %): reference × (D/5.1\")⁴ × blades/3")
		rep.default("yaw_motors_reversed", "ON" if yaw_rev else "OFF", "the yaw fit cannot tell the motor direction: " +
			("from the log header" if rev_hdr == 0 or rev_hdr == 1 else "assumed props out; check it in Betaflight's Motors tab"))
	if use_chirp and yaw_src == "freestyle":
		rep.default("Yaw fit data", "freestyle", "no yaw chirp sweep: yaw from the flying outside the sweeps, yaw rate damping 0")
	var s0 := 1.0 if yaw_rev else -1.0
	var kQ: float = rp.J_rotor * ((((bcol[0] + bcol[1]) + bcol[2]) + bcol[3]) / 4.0)
	cfg.yaw_motors_reversed = yaw_rev
	# the finished torque path against the gyro: measured / sim
	var tq: Variant = null
	if torque != null:
		var Rf := Motor.torque_path(rows, sims, LL, chirp_masks, Motor.TP_FREQS, _cancelled)
		if cancel_flag or Rf.is_empty():
			return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
		tq = []
		for ax in 2:
			var k: float = fit[Rotation.AX[ax]].coef[0] * 1e-6
			var r := PackedFloat64Array()
			var ph := PackedFloat64Array()
			for i in Motor.TP_FREQS.size():
				r.append(sqrt(Rf.re[ax][i] * Rf.re[ax][i] + Rf.im[ax][i] * Rf.im[ax][i]) / k)
				ph.append(rad_to_deg(atan2(Rf.im[ax][i], Rf.re[ax][i])))
			tq.append([r, ph])
	rep.rotation(fit, rp, kQ, yaw_rev, use_chirp, chirp_s, band, tq)
	rep.default("Izz", "%s kg·m²" % Report.fmt_g(rp.Izz, 3), "1.7 × mean roll/pitch inertia (only J_rotor/Izz is measured)")
	steps["05_rotation"] = {"chirp_s": chirp_s, "use_chirp": use_chirp, "band": band, "n_rows": fa[1], "n_rows_yaw": fa[2], "yaw_source": yaw_src,
		"fit": fit, "params": rp, "yaw_reversed": yaw_rev, "yaw_ok": yaw_ok, "yaw_reversed_default": rev_default, "kQ": kQ, "torque_ratio": tq}

	# 6 bias
	_say("Trim", 0.13)
	var b: Variant = Bias.fit(LL, uses, geo.arm, kT, kQ, s0, _cancelled)
	if cancel_flag:
		return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
	rep.bias(b, geo.arm)
	steps["06_bias"] = {"bias": b}

	# 7 battery
	_say("Battery", 0.13)
	var bat_raw: Variant = Battery.fit(rows, LL, P, V, kv_rad, 0.1, _cancelled)
	if cancel_flag:
		return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
	var bat: Dictionary
	if bat_raw == null or not (0.0 <= float(bat_raw.R) and float(bat_raw.R) < 1.0):
		rep.default("Battery sag", "R = %s" % str(dflt.battery_r), "fit not possible (fewer than 10 points or R outside 0-1)")
		bat = {"V0": h.V if h.V != null else 4.2 * ncell, "R": dflt.battery_r, "drain": 0.0, "source": "default", "rms": null, "V0_per_log": {}}
	else:
		bat = bat_raw
	rep.battery(bat)
	steps["07_battery"] = {"fit": bat_raw, "used": bat}

	# the plant (drag filled below)
	var p := {
		"mass": mass, "kv_rad": kv_rad,
		"motor_a": P.map(func(x): return Num.pyround(x[0], 1)), "motor_b": P.map(func(x): return _sig(x[1], 5)),
		"motor_c": P.map(func(x): return Num.pyround(x[2], 1)), "motor_delay_s": delay,
		"kT": _sig(kT, 4), "arm": Num.pyround(geo.arm, 4),
		"Ixx": _sig(rp.Ixx, 4), "Iyy": _sig(rp.Iyy, 4), "Izz": _sig(rp.Izz, 4),
		"kQ": _sig(kQ, 4), "J_rotor": _sig(rp.J_rotor, 4),
		"yaw_spin": Rotation.SPIN.map(func(v): return int(s0 * v)),
		"rate_damping": rp.rate_damping.map(func(v): return Num.pyround(v, 3)),
		"pitch_m_per_rad": geo.pitch_m_per_rad,
		"cd_area": null, "rotor_drag": null, "linear_drag_ref_sum_w": Num.pyround(h.sum_w, 1),
		"torque_bias_per_thrust": (b.beta if b != null else [0.0, 0.0, 0.0]).map(func(v): return _sig(v, 3)),
		"battery_v0": Num.pyround(bat.V0, 2), "battery_r": _sig(bat.R, 4),
		"prop_radius": Num.pyround(geo.prop_radius, 4),
		"turb_base_ms": dflt.turb_base_ms, "pw_turb_ms": dflt.pw_turb_ms, "pw_noise_hz": dflt.pw_noise_hz,
		"seed": 1,
	}
	if b == null:
		rep.default("Torque imbalance", "0", "no hover windows")
	if cancel_flag:
		return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])

	# 8 drag
	_say("Drag", 0.135)
	var shape: Array = dflt.cd_area
	var dfit := Drag.fit(LL, uses, mass, shape, h.sum_w, h.sum_w2, _cancelled)
	if cancel_flag:
		return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
	var rd_default: float = dflt.rotor_drag_per_s * mass / h.sum_w
	var top_given: Variant = c.get("top_speed_kmh") if Craft._truthy(c.get("top_speed_kmh")) else null
	var good: bool = dfit.get("ok", false) and dfit.r2 >= 0.5 and dfit.rel_sd != null
	var rd_ok: bool = good and dfit.rel_sd[1] <= 0.15
	var body_ok: bool = good and dfit.rel_sd[0] <= 0.3
	p.rotor_drag = _sig(dfit.rotor_drag if rd_ok else rd_default, 4)
	var body_src := "default"
	if body_ok:
		p.cd_area = shape.map(func(x): return _sig(x * dfit.scale, 4))
		body_src = "acc"
	elif top_given != null:
		var q := p.duplicate()
		q.cd_area = shape
		var sc := Drag.scale_for_top_speed(q, float(top_given), shape)
		p.cd_area = shape.map(func(x): return _sig(x * sc, 4))
		body_src = "top_speed"
	else:
		p.cd_area = shape.map(func(x): return _sig(x, 4))
	var top := Drag.top_speed(p)
	var top_default: Variant = null
	if body_src != "default":
		var q2 := p.duplicate()
		q2.cd_area = shape
		top_default = Drag.top_speed(q2)[0]
	rep.drag(dfit, rd_ok, body_src, top, top_given, p, top_default)
	steps["08_drag"] = {"fit": dfit, "rd_ok": rd_ok, "body_ok": body_ok, "body_src": body_src, "rotor_drag": p.rotor_drag,
		"cd_area": p.cd_area, "top_speed": top, "top_speed_default_body": top_default}

	# 9 propwash
	_say("Propwash metric", 0.14)
	var pw := Propwash.log_metric(LL, uses, h.throttle, _cancelled)
	if cancel_flag:
		return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
	rep.propwash(pw, p, dflt)
	steps["09_propwash"] = pw

	# 10 validate
	_say("Closed-loop check", 0.15)
	p._comment = "%s: plant identified by real_quad logfit %s from %d blackbox log sessions (%s); see report.md. Body frame FLU = Betaflight gyro frame; motor order RR, FR, RL, FL." % [
		c.name, LOGFIT_VERSION, LL.size(), ", ".join(LL.map(func(x): return x.name))]
	var pp := {}
	for k in PLANT_KEYS:
		pp[k] = p[k]
	p = pp
	var hvv := Validate.hover(p, P, h)
	var ev := Validate.roll_t90(LL, uses, rows, V, p, cfg, 12, _cancelled)
	if cancel_flag:
		return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
	var t90: Variant = Validate.summarize_t90(ev)
	rep.validation(hvv, t90, ev)
	steps["10_validate"] = {"hover": hvv, "events": ev, "t90": t90}

	# 9b prop wash calibration
	_say("Prop wash calibration", 0.155)
	var cal := Calibrate.new()
	_mutex.lock()
	_cal = cal
	_mutex.unlock()
	if cancel_flag:
		cal.cancel()
	cal.progress = func(fr: float) -> void: _say("Prop wash calibration", 0.155 + 0.115 * fr)
	var pwc := cal.run(Craft.package_craft(c), cfg, p, pw)
	_mutex.lock()
	_cal = null
	_mutex.unlock()
	if cancel_flag:
		return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
	Calibrate.apply(p, pwc)
	rep.propwash_calibration(pwc)
	steps["09b_calibration"] = pwc

	# plausibility vs the class band
	_say("Grade and report", 0.27)
	var s: float = geo.scale
	var I0 := 2.0e-3 * (mass / 0.65) * s * s
	var tau := 0.0
	for m in 4:
		tau += 1.0 / (P[m][0] / kv_rad + 2.0 * P[m][1] * 2.0 * PI * h.hz) * 1e3
	tau /= 4.0
	rep.plausible("Weight", mass * 1000.0, K.weight_g[0], K.weight_g[1], "g")
	rep.plausible("Cells", ncell, K.cells[0], K.cells[1], "S")
	rep.plausible("Hover rotor speed", h.hz, K.hover_hz[0], K.hover_hz[1], "Hz")
	rep.plausible("Motor time constant at hover", tau, K.tau_ms[0], K.tau_ms[1], "ms")
	rep.plausible("kT / kT₀", kT / dflt.kT, Classes.BANDS.kT[0], Classes.BANDS.kT[1])
	rep.plausible("Ixx / I₀", rp.Ixx / I0, Classes.BANDS.I[0], Classes.BANDS.I[1])
	rep.plausible("Iyy / I₀", rp.Iyy / I0, Classes.BANDS.I[0], Classes.BANDS.I[1])
	rep.plausible("J_rotor / J₀", rp.J_rotor / dflt.J_ref, Classes.BANDS.J_rotor[0], Classes.BANDS.J_rotor[1])
	rep.plausible("kQ/kT / (0.0117 m × D/5.1)", kQ / kT / dflt.kQ_over_kT, Classes.BANDS.kQ_over_kT[0], Classes.BANDS.kQ_over_kT[1])
	for k in 3:
		rep.plausible("Rate damping %s" % Rotation.AX[k], rp.rate_damping[k], Classes.BANDS.rate_damping[0], Classes.BANDS.rate_damping[1])
	rep.plausible("Top speed (model)", top[0], K.top_kmh[0], K.top_kmh[1], "km/h")
	if bat.source != "default":
		var am := 0.0
		for m in 4:
			am += P[m][0]
		var r_m: float = 1.0 / (kv_rad * rp.J_rotor * (am / 4.0))
		rep.plausible("Pack resistance (battery_r × R_m / cells)", float(bat.R) * r_m / ncell * 1e3, K.batt_mohm_cell[0], K.batt_mohm_cell[1], "mΩ/cell")
	for pl in rep.plaus:
		if pl.state == "WARN":
			warns.append(Checks.issue("implausible", "warning", "%s %s %s is outside the %s class band %s–%s (can be right for an unusual build: check)" % [
				pl.name, Report.fmt_g(pl.value, 3), pl.unit, cls, Report.fmt_g(pl.band[0], 6), Report.fmt_g(pl.band[1], 6)], ""))
	if cls == "seven" or cls == "ten":
		var pk: Variant = _structural_peak(LL, uses)
		if pk != null:
			rep.warn("structural_mode", "Frame resonance below 40 Hz", "gyro spectral peak at %d Hz (inside the 1–30 Hz rotation band region)" % roundi(pk))

	# coverage and grade
	var acc_ok := LL.any(func(x): return x.has_acc())
	var vbat_ok := LL.any(func(x): return x.has_vbat())
	var cov := Grade.coverage(LL, uses, segs, chirp_s, h, dfit, ev, acc_ok, vbat_ok)
	var gr := Grade.grade(cov, rep.gates)
	var letter: String = gr[0]
	var caps: Array = gr[1]
	var rate_ok := LL.all(func(x): return 900.0 <= x.fs_raw and x.fs_raw < 1500.0)
	var fn := Grade.fly_next(cov, cls, src.pid_hz() / 1000.0, rate_ok)
	var fly: Array = fn[0]
	fly.append_array(Grade.propwash_fly(pw))
	if body_src == "default":
		fly.append("Enter the quad's top speed in the model's parameters (%s): the body drag is a class default" % Grade.TOP_SPEED_WHERE)
	rep.grade = letter
	rep.grade_caps = caps
	rep.fly = fly
	rep.cover = [Grade.chirp_text(cov), Grade.drag_text(rd_ok, body_src, Report.fmt_g(float(c.top_speed_kmh), 12) if c.get("top_speed_kmh") else ""),
		Calibrate.summary(pwc)]
	rep.cli = Classes.chirp_cli(cls, src.pid_hz() / 1000.0) if fn[1] else null
	if not acc_ok:
		rep.banners.append("No accelerometer data: hover from the steady rule (about 20 % uncertainty in kT), no drag measurement. Turn on ACC logging: set blackbox_disable_acc = OFF")
	elif not LL.all(func(x): return x.has_acc()):
		warns.append(Checks.issue("acc_partial", "info", "no usable ACC in " + ", ".join(LL.filter(func(x): return not x.has_acc()).map(func(x): return x.name)) + " (hover and drag use the ACC sessions)", Checks.FIX.acc))
	if chirp_s < MIN_CHIRP_S:
		rep.banners.append("No chirp sweeps (%d s, 30 s needed): rotation from freestyle sticks (gain uncertain by 10-40 %%), rate damping 0. Fly CHIRP sweeps (CLI below)" % roundi(chirp_s))
	if not vbat_ok:
		rep.banners.append("No battery voltage: motor voltage estimated per 2-s window, weaker sag fit. Turn on: set blackbox_disable_bat = OFF")
	elif not LL.all(func(x): return x.has_vbat()):
		warns.append(Checks.issue("vbat_partial", "info", "no usable battery voltage in " + ", ".join(LL.filter(func(x): return not x.has_vbat()).map(func(x): return x.name)) + " (motor voltage estimated per 2-s window there)", Checks.FIX.vbat))
	if K.status != "validated":
		rep.banners.append("The %s class is not validated yet: its defaults and plausibility bands are extrapolated" % cls)
	var mixed: Array = warns.filter(func(w): return w.code in ["board_differs", "firmware_differs", "hover_spread"])
	if not mixed.is_empty():
		rep.banners.append("Mixed configurations: " + "; ".join(mixed.map(func(w): return w.message)))
	var fw := Checks.parse_firmware(src.headers)
	if fw[1] != null and not (fw[1][0] == 2026 and fw[1][1] == 6):
		rep.banners.append("fc.json converted from %s: the tune flies through the Betaflight 2026.6 port (small controller differences possible)" % fw[3])
	rep.issues = warns
	steps["11_grade"] = {"coverage": cov, "gates": rep.gates, "plausibility": rep.plaus, "grade": letter, "caps": caps, "fly_next": fly}

	# 9c optimized tune (grade A/B)
	var tune := {}
	if letter in ["A", "B"]:
		_say("Optimizing the tune", 0.27)
		var at := Autotune.new()
		_mutex.lock()
		_at = at
		_mutex.unlock()
		if cancel_flag:
			at.cancel()
		at.progress = func(fr: float) -> void: _say("Optimizing the tune", 0.27 + 0.72 * fr)
		tune = at.run(Craft.package_craft(c), cfg, p, LL, cls)
		_mutex.lock()
		_at = null
		_mutex.unlock()
		if cancel_flag:
			return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
	rep.sec("Optimized tune", ["- " + Autotune.summary(tune.record if bool(tune.get("ok", false)) else tune.get("why", null), letter)])     # (the game's own, as the calibration: not in 'What the logs cover')

	# write
	rep.runtime = (Time.get_ticks_msec() - t_start) / 1000.0
	var pkg_craft := Craft.package_craft(c)
	var form := {}
	for k in c:
		if not str(k).begins_with("_") and not k in ["logs", "segments"]:
			form[k] = c[k]
	var fws := {}
	for r in rows_tbl:
		if r.used:
			fws[r.firmware] = true
	var fwl: Array = fws.keys()
	fwl.sort()
	var lf := {"logfit_version": LOGFIT_VERSION, "package_format": Craft.PACKAGE_FORMAT,
		"created": Time.get_datetime_string_from_system(true) + "+00:00",
		"form": form, "form_sources": sources, "files": files, "sessions": rows_tbl.map(func(r):
			var o: Dictionary = r.duplicate()
			o.erase("path")
			return o),
		"firmware": fwl, "fc_source": src.name,
		"coverage": cov, "gates": rep.gates, "plausibility": rep.plaus, "grade": letter, "grade_meaning": Grade.MEANING[letter],
		"grade_reasons": caps.map(func(x): return {"cap": x[0], "reason": x[1]}), "warnings": warns, "banners": rep.banners,
		"fly_next": fly, "cli": rep.cli, "propwash": pw, "param_sources": _param_sources(rd_ok, body_src, yaw_ok, use_chirp, b, bat, geo, sources),
		"runtime_s": Num.pyround(rep.runtime, 1)}
	for k in pwc.calibrated:
		lf.param_sources[{"calm": "turb_base_ms", "dropcatch": "pw_turb_ms"}[k]] = "M"
	var md := rep.render()
	steps["12_final"] = {"plant": p, "fc": cfg, "craft": pkg_craft, "logfit": lf}
	if out_dir != "":
		# the last chance to cancel: from here on the package is written and the answer is ok
		if cancel_flag:
			return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
		var jsons := {"craft.json": pkg_craft, "fc.json": cfg, "plant.json": p, "logfit.json": lf}
		var texts := {"report.md": md}
		if bool(tune.get("ok", false)):
			jsons["fc_optimized.json"] = Autotune.optimized_json(tune.fc, json_text(cfg), json_text(p), tune.record)
			texts["tune.md"] = Autotune.report_md(str(pkg_craft.get("name", "")), cfg, tune.fc, tune.record)
		var bad := nonfinite(jsons)
		if bad != "":
			return _refuse([Checks.issue("nonfinite_value", "error", "the fit produced a value that is not a finite number (%s): the package would be unusable" % bad,
				"use other logs; if the logs are fine this is a bug in the fit, please report it")], rows_tbl)
		var werr := _write_package(out_dir, jsons, texts, _cancelled)
		if werr == "cancelled":
			return _refuse([Checks.issue("cancelled", "error", "cancelled", "")])
		if werr != "":
			return _refuse([Checks.issue("write_failed", "error", werr, "check that the folder can be written")], rows_tbl)
	_say("Done", 1.0)
	return {"ok": true, "errors": [], "warnings": warns, "banners": rep.banners, "grade": letter, "grade_meaning": Grade.MEANING[letter],
		"grade_reasons": caps, "fly_next": fly, "cli": rep.cli, "report_md": md, "package_dir": out_dir, "model_id": c.id, "propwash_calibration": Calibrate.summary(pwc),
		"plant": p, "fc": cfg, "craft": pkg_craft, "logfit": lf, "sessions": rows_tbl, "runtime_s": rep.runtime}


## float(f"{x:.{n}g}")
static func _sig(x: float, n: int) -> float:
	if x == 0.0 or not is_finite(x):
		return x
	return Report.fmt_g(x, n).to_float()


static func _param_sources(rd_ok: bool, body_src: String, yaw_ok: bool, chirp: bool, b: Variant, bat: Dictionary, geo: Dictionary, form: Dictionary) -> Dictionary:
	var bm := "M" if bat.source != "default" else "C"
	return {"mass": "U", "kv_rad": "U" if form.get("kv") == "U" else "M", "motor_a": "M", "motor_b": "M", "motor_c": "M",
		"motor_delay_s": "M" if chirp else "K", "kT": "M", "arm": "C" if geo.arm_assumed else "U", "Ixx": "D", "Iyy": "D", "Izz": "C",
		"J_rotor": "M" if yaw_ok else "C", "kQ": "D", "yaw_spin": "M" if yaw_ok else "C", "rate_damping": "M" if chirp else "C",
		"pitch_m_per_rad": "C" if geo.pitch_assumed else "U", "prop_radius": "U" if form.get("prop.diameter_in") == "U" else "C",
		"cd_area": {"acc": "M", "top_speed": "D", "default": "C"}[body_src], "rotor_drag": "M" if rd_ok else "C",
		"linear_drag_ref_sum_w": "M", "torque_bias_per_thrust": "M" if b != null else "C",
		"battery_v0": bm, "battery_r": bm, "turb_base_ms": "C", "pw_turb_ms": "C", "pw_noise_hz": "K", "seed": "K"}


## Gyro spectral peak 15-40 Hz standing > 10x above the 15-80 Hz median (seven/ten classes).
static func _structural_peak(logs: Array, uses: Array) -> Variant:
	var acc := PackedFloat64Array()
	var freqs := PackedFloat64Array()
	for k in logs.size():
		var L: Flight = logs[k]
		var r := Num.runs(uses[k])
		for j in range(0, r.size(), 2):
			if r[j + 1] - r[j] < 4096:
				continue
			for ax in 2:
				var w := Num.welch((L.gyro_unfilt[ax] as PackedFloat64Array).slice(r[j], r[j + 1]), L.fs, 2048)
				freqs = w[0]
				var pxx: PackedFloat64Array = w[1]
				if acc.is_empty():
					acc = pxx.duplicate()
				else:
					for i in acc.size():
						acc[i] += pxx[i]
	if acc.is_empty():
		return null
	var band := PackedFloat64Array()
	var best := -1
	for i in acc.size():
		if freqs[i] >= 15.0 and freqs[i] <= 80.0:
			band.append(acc[i])
		if freqs[i] >= 15.0 and freqs[i] <= 40.0 and (best < 0 or acc[i] > acc[best]):
			best = i
	if best >= 0 and acc[best] > 10.0 * Num.median(band):
		return freqs[best]
	return null


## A package's JSON file as written
static func json_text(d: Dictionary) -> String:
	return JSON.stringify(d, " ", false, true) + "\n"


## The path of the first NaN / inf in nested dictionaries and arrays ("plant.json/Ixx"), else "" (JSON cannot hold them:
## Godot writes NaN as null and inf as 1e99999).
static func nonfinite(v: Variant, path: String = "") -> String:
	if v is float:
		return path if is_nan(v) or is_inf(v) else ""
	var keys: Array = v.keys() if v is Dictionary else range(v.size()) if v is Array else []
	for k in keys:
		var bad := nonfinite(v[k], "%s/%s" % [path, k] if path != "" else str(k))
		if bad != "":
			return bad
	return ""


## Writes the package (jsons, and texts: report.md, tune.md) into a temporary folder next to out_dir, then swaps
## it in: an old package is renamed to <id>.bak beside the temporary folder first, put back if the swap fails, and
## deleted after it. Every write is checked. "" = ok, "cancelled" (cancel() before the swap: nothing changed), else
## the error.
static func _write_package(out_dir: String, jsons: Dictionary, md_texts: Dictionary, cancel: Callable = Callable()) -> String:
	var parent := out_dir.get_base_dir()
	var tmp_root := parent.get_base_dir().path_join("logfit_tmp")
	var tmp := tmp_root.path_join(out_dir.get_file())
	var bak := tmp_root.path_join(out_dir.get_file() + ".bak")
	_remove_dir(tmp)
	if DirAccess.make_dir_recursive_absolute(tmp) != OK:
		return "cannot create %s" % tmp
	var texts := md_texts.duplicate()
	for name in jsons:
		texts[name] = json_text(jsons[name])
	for name in texts:
		var err := _write_text(tmp.path_join(name), texts[name])
		if err != "":
			_remove_dir(tmp)
			return err
	if cancel.is_valid() and cancel.call():
		_remove_dir(tmp)
		return "cancelled"
	DirAccess.make_dir_recursive_absolute(parent)
	_remove_dir(bak)
	var had_old := DirAccess.dir_exists_absolute(out_dir)
	if had_old and DirAccess.rename_absolute(out_dir, bak) != OK:
		_remove_dir(tmp)
		return "cannot move the old package %s aside" % out_dir
	if DirAccess.rename_absolute(tmp, out_dir) != OK:
		if had_old:
			DirAccess.rename_absolute(bak, out_dir)
		_remove_dir(tmp)
		return "cannot move %s to %s" % [tmp, out_dir]
	_remove_dir(bak)
	return ""


## Writes text to path, checking the open, the write and the flush ("" = ok).
static func _write_text(path: String, text: String) -> String:
	var f := FileAccess.open(path, FileAccess.WRITE)
	if f == null:
		return "cannot write %s (%s)" % [path, error_string(FileAccess.get_open_error())]
	var ok := f.store_string(text)
	f.flush()
	var err := f.get_error()
	f.close()
	if not ok or err != OK:
		return "cannot write %s (%s)" % [path, error_string(err if err != OK else ERR_FILE_CANT_WRITE)]
	return ""


static func _remove_dir(dir: String) -> void:
	var da := DirAccess.open(dir)
	if da == null:
		return
	for f in da.get_files():
		da.remove(f)
	for d in da.get_directories():
		_remove_dir(dir.path_join(d))
	DirAccess.remove_absolute(dir)
