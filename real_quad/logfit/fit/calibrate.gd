extends RefCounted
## real_quad logfit: prop wash calibration (contract 4.4, step 9b). Sizes two values of the plant so that
## the model, flying through dirty_air's air (dirty_air/air.gd, free_start() / free_tick(): no scene tree,
## no map, high in the open), shakes as much as the logs say:
##   dirty_air_gust  the old wake's turbulence (dirty_air's gust) <- the logs' worst_p99, scenario "dropcatch"
##   turb_base_ms    the model's calm-air turbulence             <- the logs' calm median, scenario "calm"
## dirty_air's wash and wake stay 1 (its physics as its author made it).
##
## The scenarios are those of the propwash test (workspace tests/godot/tests/propwash_test.gd, analysed by
## tests/analyze_propwash.py), flown here without Godot physics: a level hover at 60 m found by a PD
## throttle (2.5 s, wake cleared first), then
##   calm       2 s at hover throttle                   metric: median 0.25-s window
##   dropcatch  0.9 s at zero throttle, 1.2 s at 45 %    metric: worst 0.25-s window
## each window the RMS of the roll and pitch tracking error (setpoint - rate at the 500 Hz tick) band-passed
## 15-78 Hz, every 50 ms; the mean over SEEDS seeds (fixed, the same for every value
## tried, so a value's result is deterministic). The log metric (propwash.gd) is the same band and window
## size on the blackbox log. The scale: the dropcatch scenario's worst window was set by hand to 14.5 deg/s
## for freestyle5 against a worst_p99 of 11.3 in its logs (freestyle35: 13.2 at 10.3, the same ratio), so
## target = DROP_PER_P99 x worst_p99; calm: target = the calm median as it is.
## Search per value: secant steps from the class default (the first step proportional), kept inside the
## bracket found so far (bisection when a step leaves it), at most MAX_EVAL tries, done within TOL.
## Calm first (the gust does not touch a calm hover), then the gust with the calibrated calm turbulence.

const BFModel := preload("../../flightmodel/bf_model.gd")
const Quad := preload("../../flightmodel/bf_quad.gd")
const Num := preload("num.gd")
const Craft := preload("craft.gd")

const AIR := "res://dirty_air/air.gd"
const AIR_CFG := "res://dirty_air/zonemod.cfg"
const DROP_PER_P99 := 14.5 / 11.3
const SEEDS := 32                    # seeds 1..SEEDS by default
const TICK_HZ := 500.0
const HOVER_H := 60.0
const MAX_EVAL := 8
const TOL := 0.02                     # relative
const GUST_MAX := 60.0
const MIN_CALM := 20                 # 0.25-s windows (every 0.125 s)
const MIN_CHOPS := 10
const TURB_MAX := 3.0

var cancel_flag := false
var progress := Callable()           # progress.call(fraction 0..1)
var evals := 0                       # scenario runs (all seeds) so far
var n_seeds := SEEDS                 # seeds 1..n_seeds
var _air_script: Script = null


## dirty_air's version in this game ("" = not installed; "?" = installed without a version)
static func dirty_air_version() -> String:
	if not ResourceLoader.exists(AIR) and not FileAccess.file_exists(AIR):
		return ""
	var c := ConfigFile.new()
	if c.load(AIR_CFG) != OK:
		return "?"
	return str(c.get_value("mod", "version", "?"))


## Targets from the log metric (propwash.gd's log_metric): {dropcatch, calm}, null where the logs lack it
## (fewer than MIN_CALM windows of calm hover, or MIN_CHOPS windows after throttle chops: the worst
## windows of logs without drops are not prop wash)
static func targets(metric: Dictionary) -> Dictionary:
	var n: Array = metric.get("n", [0, 0, 0, 0])
	var p99: Variant = metric.get("worst_p99")
	var calm: Variant = metric.get("calm")
	return {"dropcatch": Num.pyround(DROP_PER_P99 * float(p99), 2) if p99 != null and int(n[1]) >= MIN_CHOPS else null,
		"calm": float(calm) if calm != null and int(n[0]) >= MIN_CALM else null}


## Puts the record's values into the plant (wash and wake 1, the gust and calm turbulence, the record)
static func apply(p: Dictionary, rec: Dictionary) -> void:
	p.dirty_air_wash = 1.0
	p.dirty_air_wake = 1.0
	p.dirty_air_gust = rec.gust
	p.turb_base_ms = rec.turb_base_ms
	p.dirty_air_calibration = rec


## One line for the report and the window: "Prop wash calibrated: drop and catch X °/s (log Y), calm X (log Y)"
## or why not
static func summary(rec: Dictionary) -> String:
	var parts: Array = []
	var sc: Dictionary = rec.scenario
	var tg: Dictionary = rec.targets
	if "dropcatch" in rec.calibrated:
		parts.append("drop and catch %s °/s (log %s)" % [_f(sc.dropcatch), _f(tg.dropcatch)])
	if "calm" in rec.calibrated:
		parts.append("calm %s °/s (log %s)" % [_f(sc.calm), _f(tg.calm)])
	if parts.is_empty():
		return "Prop wash not calibrated: %s; class values (dirty_air gust %s, calm-air turbulence %s m/s)" % [rec.why, _f(rec.gust), _f(rec.turb_base_ms)]
	return "Prop wash calibrated: %s%s" % [", ".join(parts), "; %s" % rec.why if rec.why != "" else ""]


static func _f(x: Variant) -> String:
	if x == null:
		return "?"
	var v := float(x)
	return String.num(v, 2 if absf(v) < 10.0 else 1)


## Whether the package's calibration was made with another dirty_air than the installed one (or none,
## with dirty_air installed): {recalibrate, why}
static func stale(plant: Dictionary) -> Dictionary:
	var ver := dirty_air_version()
	if ver == "":
		return {"recalibrate": false, "why": ""}
	var rec: Variant = plant.get("dirty_air_calibration")
	if not (rec is Dictionary):
		return {"recalibrate": true, "why": "the prop wash was never calibrated in the game (dirty_air %s is installed)" % ver}
	var was := str(rec.get("dirty_air_version", ""))
	if was == "":
		return {"recalibrate": true, "why": "the prop wash was not calibrated (%s); dirty_air %s is installed now" % [str(rec.get("why", "")), ver]}
	if was != ver:
		return {"recalibrate": true, "why": "the prop wash was calibrated with dirty_air %s, %s is installed" % [was, ver]}
	return {"recalibrate": false, "why": ""}


## Calibrates. craft, fc, plant: the package's dictionaries (craft.json, fc.json, plant.json); metric: the
## log metric. Returns the record for plant.json "dirty_air_calibration":
## {state: calibrated | partial | skipped, why, dirty_air_version, targets, scenario {dropcatch, calm},
##  gust, turb_base_ms, seeds, runs, runtime_s} - the values are the class defaults where not calibrated.
func run(craft: Dictionary, fc: Dictionary, plant: Dictionary, metric: Dictionary) -> Dictionary:
	var t0 := Time.get_ticks_msec()
	var ver := dirty_air_version()
	var tg := targets(metric)
	var rec := {"state": "skipped", "why": "", "dirty_air_version": ver, "targets": tg, "scenario": {"dropcatch": null, "calm": null},
		"gust": plant.get("dirty_air_gust", 1.0), "turb_base_ms": plant.get("turb_base_ms", 0.0), "seeds": n_seeds, "runs": 0,
		"calibrated": [], "runtime_s": 0.0}
	if ver == "":
		rec.why = "dirty_air is not installed"
		return rec
	_air_script = load(AIR)
	var probe: Object = _air_script.new() if _air_script is Script and _air_script.can_instantiate() else null
	var fits := probe != null and probe.has_method("free_start")
	if probe != null:
		probe.free()
	if not fits:
		rec.why = "the installed dirty_air is too old (no free_start(): update it)"
		return rec
	if tg.dropcatch == null and tg.calm == null:
		rec.why = "the logs have neither calm hovers nor drops and catches"
		return rec
	# the search always starts from the class defaults (a build's plant has them, a recalibrated one its old values): same result
	var dflt := Craft.defaults(craft)
	plant = plant.duplicate()
	plant.turb_base_ms = dflt.turb_base_ms
	plant.dirty_air_gust = dflt.dirty_air.gust
	rec.turb_base_ms = plant.turb_base_ms
	rec.gust = plant.dirty_air_gust
	var sims: Array = _sims(craft, fc, plant)
	if sims.is_empty():
		rec.why = "the model could not be loaded"
		return rec
	_calibrate(sims, plant, tg, rec)
	for s in sims:
		s.dispose()
	rec.runs = evals
	rec.runtime_s = Num.pyround((Time.get_ticks_msec() - t0) / 1000.0, 1)
	return rec


func _calibrate(sims: Array, plant: Dictionary, tg: Dictionary, rec: Dictionary) -> void:
	var p := plant.duplicate()
	p.dirty_air_wash = 1.0
	p.dirty_air_wake = 1.0
	var n_steps := (MAX_EVAL if tg.calm != null else 1) + (MAX_EVAL if tg.dropcatch != null else 1)
	var done := [0]
	var tick := func() -> void:
		done[0] += 1
		if progress.is_valid():
			progress.call(minf(float(done[0]) / n_steps, 1.0))
	# the hover throttle, once (seed 1, with the class's calm turbulence)
	_apply(sims, p)
	var hover: float = sims[0].find_hover()
	for s in sims:
		s.hover_thr = hover
	# calm air
	if tg.calm != null:
		var r := _search(sims, p, "turb_base_ms", "calm", float(tg.calm), float(p.turb_base_ms), 0.0, TURB_MAX, tick)
		if cancel_flag:
			return
		p.turb_base_ms = Num.pyround(r[0], 3)
		rec.turb_base_ms = p.turb_base_ms
		rec.scenario.calm = Num.pyround(r[1], 2)
		rec.calibrated.append("calm")
	else:
		tick.call()
	# drop and catch
	if tg.dropcatch != null:
		var r := _search(sims, p, "dirty_air_gust", "dropcatch", float(tg.dropcatch), float(p.dirty_air_gust), 0.0, GUST_MAX, tick)
		if cancel_flag:
			return
		p.dirty_air_gust = Num.pyround(r[0], 2)
		rec.gust = p.dirty_air_gust
		rec.scenario.dropcatch = Num.pyround(r[1], 2)
		rec.calibrated.append("dropcatch")
	rec.state = "calibrated" if rec.calibrated.size() == 2 else "partial"
	if tg.calm == null:
		rec.why = "no calm hover in the logs: calm-air turbulence from the class"
	elif tg.dropcatch == null:
		rec.why = "no throttle chops in the logs: the old wake's gusts from the class"


func cancel() -> void:
	cancel_flag = true


# ------------------------------------------------------------------ search

## Secant search of key in [lo, hi] so that scenario's metric is target, from x0: [x, metric] of the try
## closest to the target
func _search(sims: Array, p: Dictionary, key: String, scen: String, target: float, x0: float, lo: float, hi: float, tick: Callable) -> Array:
	var best := [x0, INF]
	var prev: Array = []                          # the last try [x, metric - target]
	var x := clampf(x0, lo, hi) if x0 > 0.0 else 0.1 * hi
	var blo := lo                                 # the bracket: below target at blo, above at bhi (the metric rises with x)
	var bhi := hi
	var tries := 0
	while tries < MAX_EVAL:
		p[key] = x
		var m := _eval(sims, p, scen)
		tries += 1
		tick.call()
		if cancel_flag:
			break
		var e := m - target
		if best[1] == INF or absf(e) < absf(float(best[1]) - target):
			best = [x, m]
		if absf(e) <= TOL * target or (e > 0.0 and x <= lo) or (e < 0.0 and x >= hi):
			break                                 # there, or as close as the bounds allow
		if e < 0.0:
			blo = maxf(blo, x)
		else:
			bhi = minf(bhi, x)
		var nx: float
		if prev.is_empty() or absf(e - float(prev[1])) < 1e-9:
			nx = x * target / m if m > 1e-9 else 2.0 * x   # first: as if the metric were proportional to x
		else:
			nx = x - e * (x - float(prev[0])) / (e - float(prev[1]))
		if is_nan(nx) or nx <= blo or nx >= bhi:
			nx = 0.5 * (blo + bhi) if bhi < hi or e > 0.0 else minf(2.0 * x, hi)
		prev = [x, e]
		x = clampf(nx, lo, hi)
	for i in range(tries, MAX_EVAL):
		tick.call()
	_trace("%s: %s %s -> %.3f (target %.3f, %d runs)" % [scen, key, str(best[0]), best[1], target, tries])
	return best


## The scenario's metric with plant values p, mean over the seeds (each flown by a worker pool task: own model and air)
func _eval(sims: Array, p: Dictionary, scen: String) -> float:
	_apply(sims, p)
	var res := Num.parallel_for(sims.size(), func(i: int) -> float: return sims[i].run(scen))
	evals += 1
	return Num.mean(PackedFloat64Array(res))


func _apply(sims: Array, p: Dictionary) -> void:
	for s in sims:
		s.set_plant(p)


static func _trace(_s: String) -> void:
	if OS.is_debug_build() and OS.get_environment("REAL_QUAD_PW_TRACE") != "":
		print("propwash calibration: ", _s)


## One Sim per seed, each with its own model (loaded from a temporary package) and air
func _sims(craft: Dictionary, fc: Dictionary, plant: Dictionary) -> Array:
	var dir := OS.get_user_data_dir().path_join("real_quad/calibrate_tmp/%d_%d" % [OS.get_process_id(), Time.get_ticks_usec()])
	DirAccess.make_dir_recursive_absolute(dir)
	var out: Array = []
	var ok := true
	for pair in [["craft.json", craft], ["fc.json", fc], ["plant.json", plant]]:
		var f := FileAccess.open(dir.path_join(pair[0]), FileAccess.WRITE)
		ok = f != null
		if not ok:
			break
		f.store_string(JSON.stringify(pair[1]))
		f.close()
	for sd in (range(1, n_seeds + 1) if ok else []):
		var m := BFModel.new(dir)
		ok = m.valid
		if not ok:
			break
		var sim := Sim.new(m, _air_script.new(), int(sd))
		sim.cal = self
		out.append(sim)
	for f in DirAccess.get_files_at(dir):
		DirAccess.remove_absolute(dir.path_join(f))
	DirAccess.remove_absolute(dir)
	if not ok:
		for s in out:
			s.dispose()
		return []
	return out


# ------------------------------------------------------------------ the flight

## Godot's body state as the model sees it in integrate_forces() (custom integrator)
class State extends PhysicsDirectBodyState3DExtension:
	var dt := 1.0 / TICK_HZ
	var xf := Transform3D()
	var lin := Vector3()
	var ang := Vector3()

	func _get_step() -> float:
		return dt

	func _get_transform() -> Transform3D:
		return xf

	func _get_linear_velocity() -> Vector3:
		return lin

	func _set_linear_velocity(v: Vector3) -> void:
		lin = v

	func _get_angular_velocity() -> Vector3:
		return ang

	func _set_angular_velocity(v: Vector3) -> void:
		ang = v


## One seed's flight: the model, its air, a pilot as the propwash test's
class Sim:
	var model: BFModel
	var air: Node
	var seed_n := 1
	var st := State.new()
	var dt := 1.0 / TICK_HZ
	var hover_thr := -0.45
	var msg := {}
	var rc := {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": -1.0}
	var sp := [PackedFloat64Array(), PackedFloat64Array()]
	var gy := [PackedFloat64Array(), PackedFloat64Array()]
	var cal = null                   # the calibration (its cancel_flag)

	func _init(m: BFModel, a: Node, s: int) -> void:
		model = m
		air = a
		seed_n = s

	## Frees the air and the body state (Objects, not reference counted)
	func dispose() -> void:
		air.free()
		st.free()

	func set_plant(p: Dictionary) -> void:
		for k in ["dirty_air_wash", "dirty_air_wake", "dirty_air_gust", "turb_base_ms"]:
			model.plant_params[k] = p[k]
		model.quad.turb_base_ms = float(p.turb_base_ms)

	## The message to dirty_air, as bf_model.apply_air()
	func _msg() -> Dictionary:
		var q := model.quad
		msg["thrust"] = q.thrust[0] + q.thrust[1] + q.thrust[2] + q.thrust[3]
		msg["prop_mm"] = q.prop_radius * 2000.0
		msg["wheelbase_mm"] = q.arm * 2.0 * sqrt(2.0) * 1000.0
		msg["ducted"] = bool(model.craft.get("ducted", false))
		msg["wash"] = model.get_air_wash()
		msg["wake"] = model.get_air_wake()
		msg["gust"] = model.get_air_gust()
		return msg

	func start() -> void:
		model.quad.rng.seed = seed_n
		air.free_start(_msg(), seed_n)

	## One physics tick: dirty_air's air for it (as bf_model.apply_air(), floor and ceiling 1), the model,
	## then the pose integrated as Godot does with the velocities handed back
	func tick() -> void:
		var q := model.quad
		air.free_tick(_msg(), st.xf, st.lin, dt)
		q.thrust_scale.fill(1.0)
		q.has_air = true
		q.air_vel = air.wake_flow
		q.air_mean = air.descent_mean
		var ax: Array = air.prop_axial
		var fl: Array = air.prop_vrs
		for i in 4:
			var j: int = BFModel.DIRTY_AIR_PROP[i]
			q.air_axial[i] = float(ax[j])
			q.air_fluct[i] = float(fl[j])
		model.integrate_forces(null, st, rc, true)
		var b := st.xf.basis
		var a := st.ang * dt
		var an := a.length()
		if an > 1e-12:
			b = Basis(a / an, an) * b
		st.xf = Transform3D(b.orthonormalized(), st.xf.origin + st.lin * dt)

	func rates() -> Vector3:
		return Quad.local_to_flu(st.xf.basis.inverse() * st.ang) * (180.0 / PI)

	func _rec() -> void:
		var r := rates()
		gy[0].append(r.x)
		gy[1].append(r.y)
		sp[0].append(model.fc.setpoint_rate[0])
		sp[1].append(model.fc.setpoint_rate[1])

	func _cancelled() -> bool:
		return cal != null and cal.cancel_flag

	func _fly(seconds: float, thr: float) -> void:
		rc.roll = 0.0
		rc.pitch = 0.0
		rc.throttle = thr
		for i in int(seconds / dt):
			if i % 50 == 0 and _cancelled():
				return
			tick()
			_rec()

	## Teleport to a still, level hover at HOVER_H and settle (wake cleared); returns the mean throttle of its last second
	func hover_at() -> float:
		st.xf = Transform3D(Basis.IDENTITY, Vector3(0, HOVER_H, 0))
		st.lin = Vector3()
		st.ang = Vector3()
		model.reset()
		air.clear_wake()
		var integ := 0.0
		var n := int(2.5 / dt)
		var sum := 0.0
		for i in n:
			if i % 50 == 0 and _cancelled():
				return hover_thr
			var vy := st.lin.y
			integ += -vy * dt + (HOVER_H - st.xf.origin.y) * 0.5 * dt
			var lvl := st.xf.basis.y
			rc.roll = clampf(-lvl.x * 2.0, -1, 1)
			rc.pitch = clampf(lvl.z * 2.0, -1, 1)
			rc.throttle = clampf(hover_thr - vy * 0.3 + integ * 0.5, -1, 1)
			tick()
			if i >= n - int(1.0 / dt):
				sum += rc.throttle
		return sum / int(1.0 / dt)

	func find_hover() -> float:
		start()
		hover_thr = hover_at()
		return hover_thr

	## Flies the scenario from the start (seeded) and returns its metric
	func run(scen: String) -> float:
		start()
		hover_at()
		for k in 2:
			sp[k] = PackedFloat64Array()
			gy[k] = PackedFloat64Array()
		if scen == "calm":
			_fly(2.0, hover_thr)
		else:
			_fly(0.9, -1.0)
			_fly(1.2, -0.1)
		if _cancelled() or sp[0].size() < int(TICK_HZ):
			return 0.0
		var w := windows(sp, gy, TICK_HZ)
		if w.is_empty():
			return 0.0
		if scen == "calm":
			return Num.median(w)
		var mx := 0.0
		for v in w:
			mx = maxf(mx, v)
		return mx

	## RMS of the roll and pitch tracking error, band-passed 15-78 Hz, in 0.25-s windows every 50 ms
	## (tests/analyze_propwash.py)
	static func windows(sp: Array, gy: Array, fs: float) -> PackedFloat64Array:
		var sos := Num.butter(2, [15.0, 78.0], "bandpass", fs)
		var e: Array = []
		for k in 2:
			var x := PackedFloat64Array()
			x.resize(sp[k].size())
			for i in x.size():
				x[i] = sp[k][i] - gy[k][i]
			e.append(Num.sosfiltfilt(sos, x))
		var n := int(0.25 * fs)
		var step := int(0.05 * fs)
		var out := PackedFloat64Array()
		var i0 := 0
		while i0 < e[0].size() - n:
			var s := 0.0
			for i in range(i0, i0 + n):
				s += e[0][i] * e[0][i] + e[1][i] * e[1][i]
			out.append(sqrt(s / (2 * n)))
			i0 += step
		return out


## report.md (md) with the record's calibration in place of the one there: the line under "What the logs
## cover", the rows of the gust and the calm turbulence (Measured or Class defaults), the section
static func report_md(md: String, rec: Dictionary) -> String:
	var rep: RefCounted = load("res://real_quad/logfit/fit/report.gd").new({})
	rep.propwash_calibration(rec)
	var lines := Array(md.split("\n"))
	var out: Array = []
	var i := 0
	var drop_rows := ["| dirty_air gust |", "| Calm-air turbulence |", "| dirty_air wash/wake |"]
	var table := ""
	while i < lines.size():
		var l: String = lines[i]
		if l.begins_with("- Prop wash calibrated") or l.begins_with("- Prop wash not calibrated"):
			out.append("- " + summary(rec))
		elif drop_rows.any(func(r): return l.begins_with(r)):
			pass
		elif l == "## Prop wash calibration":
			while i + 1 < lines.size() and not lines[i + 1].begins_with("## "):
				i += 1
			out.append(l)
			out.append_array(rep.sections[0][1])
			out.append("")
		else:
			if l.begins_with("## "):
				table = l
			# the end of a table: its new rows first
			if l == "" and out.size() > 0 and str(out[-1]).begins_with("|"):
				var rows: Array = rep.measured if table == "## Measured" else rep.defaults if table == "## Class defaults and assumptions" else []
				for r in rows:
					out.append("| %s | %s | %s |" % r)
			out.append(l)
		i += 1
	return "\n".join(out)
