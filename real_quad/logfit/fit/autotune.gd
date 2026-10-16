extends RefCounted
## real_quad logfit: autotune - the optimized tune of a model: the Betaflight settings that fly its plant best in
## the simulator ("how would my quad probably fly when well tuned"), made by the build (build.gd step 9c, grade
## A/B), kept in the package (fc_optimized.json, tune.md) and flown when a drone's tune setting says "optimized".
## Simulator only: nothing here is meant for the real quad (no CLI), and the filters stay as flown.
##
## The flights (calibrate.gd's Sim: bf_model.gd through Godot's integrate_forces() path at 500 Hz ticks,
## no scene tree; Num.parallel_for over its sims, each with its own models; fixed seeds, so every result is deterministic):
##   replay     the pilot's own sticks: the REPLAY_N busiest 4-s windows of the build's logs
##   steps      roll, pitch and yaw stick steps at hover throttle, near idle, and roll steps at full throttle
##   flip       a full-stick roll and a pitch flip at low throttle, then the catch
##   propwash   calibrate.gd's drop and catch through dirty_air's air (if installed), PW_SEEDS seeds
## Each is scored by its tracking error (RMS of setpoint - rate, deg/s; propwash: its worst 0.25-s window
## band-passed 15-78 Hz, as the calibration); the cost is the mean over the groups of tuned / flown.
## Constraints (a candidate that breaks one is not taken):
##   saturation  the share of time the mixer runs out of authority (motor_mix_range > 1) at most the flown
##               tune's + SAT_SLACK
##   robustness  the hover steps again on a perturbed plant (inertia x PERTURB_INERTIA, motor delay x
##               PERTURB_DELAY: more loop gain and more lag at once, for the identification's error): the
##               ringing (RMS of the tracking error band-passed 20-200 Hz on the plateaus after the steps, deg/s)
##               at most RING_SLACK x the flown tune's there, or RING_FLOOR
##   overshoot   of the hover roll and pitch steps at most MAX_OVERSHOOT (or the flown tune's)
##   bounds      the class's (classes.gd TUNE: P, I, FF, highest D, D/P band, TPA, dynamic idle, anti-gravity,
##               thrust linear), widened to a flown value outside them
## Search: staged coordinate search from the flown tune - stage 1 P, D, D-max, FF, I per axis (relative steps
## STEPS1), stage 2 TPA, dynamic idle, anti-gravity, thrust linear (absolute steps STEPS2, halved per round);
## each setting tried up and down, the first move that lowers the cost by MIN_GAIN is kept.
## Not modelled (the report says so): gyro noise and motor heat (instead the class's D bound), frame
## resonances, the feel of the sticks' RC link beyond its rate.

const Calibrate := preload("calibrate.gd")
const Classes := preload("classes.gd")
const BFModel := preload("../../flightmodel/bf_model.gd")
const Flight := preload("flight.gd")
const Num := preload("num.gd")

const TICK_HZ := 500.0
const AX := ["roll", "pitch", "yaw"]
const STAGE1 := [["p", 0], ["d", 0], ["d_max", 0], ["ff", 0], ["i", 0], ["p", 1], ["d", 1], ["d_max", 1], ["ff", 1], ["i", 1],
	["p", 2], ["ff", 2], ["i", 2]]
const STAGE2 := [["tpa_rate", -1], ["tpa_breakpoint", -1], ["dyn_idle_min_rpm", -1], ["anti_gravity_gain", -1], ["thrust_linear", -1]]
const STEPS1 := [0.25, 0.12, 0.06]
const STEPS2 := {"tpa_rate": 10.0, "tpa_breakpoint": 100.0, "dyn_idle_min_rpm": 6.0, "anti_gravity_gain": 20.0, "thrust_linear": 10.0}
const ROUNDS2 := 2
const MIN_GAIN := 0.002
const PW_SEEDS := 2
const SAT_SLACK := 0.005
const RING_SLACK := 1.2
const RING_FLOOR := 0.5              # deg/s
const MAX_OVERSHOOT := 0.1           # of a hover roll or pitch step (or the flown tune's, if more)
const PERTURB_INERTIA := 0.8
const PERTURB_DELAY := 1.5
const REPLAY_N := 3
const REPLAY_S := 4.0
const SETTLE_S := 1.0                # the replay's first second only brings the quad into the flight
const HOVER_S := 1.5                 # the hover before each flight
const MAX_THREADS := 8

var cancel_flag := false
var progress := Callable()           # progress.call(fraction 0..1)
var evals := 0
var _air_script: Script = null


func cancel() -> void:
	cancel_flag = true


# ------------------------------------------------------------------ settings

static func get_v(cfg: Dictionary, k: Array) -> float:
	var a: int = k[1]
	match k[0]:
		"p":
			return float(cfg.pid[AX[a]][0])
		"i":
			return float(cfg.pid[AX[a]][1])
		"d":
			return float(cfg.pid[AX[a]][2])
		"d_max":
			return float(cfg.d_max[a])
		"ff":
			return float(cfg.ff_weight[a])
	return float(cfg[k[0]])


static func set_v(cfg: Dictionary, k: Array, v: float) -> void:
	var a: int = k[1]
	var iv := roundi(v)
	match k[0]:
		"p":
			cfg.pid[AX[a]][0] = iv
		"i":
			cfg.pid[AX[a]][1] = iv
		"d":
			cfg.pid[AX[a]][2] = iv
			cfg.d_max[a] = maxi(int(cfg.d_max[a]), iv)      # D-max is at least D (Betaflight: inactive when equal)
		"d_max":
			cfg.d_max[a] = maxi(iv, int(cfg.pid[AX[a]][2]))
		"ff":
			cfg.ff_weight[a] = iv
		_:
			cfg[k[0]] = iv


static func label(k: Array) -> String:
	var names := {"p": "P", "i": "I", "d": "D", "d_max": "D-max", "ff": "Feedforward", "tpa_rate": "TPA rate (%)",
		"tpa_breakpoint": "TPA breakpoint", "dyn_idle_min_rpm": "Dynamic idle (x100 rpm)", "anti_gravity_gain": "Anti-gravity gain",
		"thrust_linear": "Thrust linear (%)"}
	return "%s %s" % [AX[k[1]].capitalize(), names[k[0]]] if int(k[1]) >= 0 else names[k[0]]


## The highest D of an axis
static func peak_d(cfg: Dictionary, a: int) -> float:
	return maxf(float(cfg.pid[AX[a]][2]), float(cfg.d_max[a]))


## [lo, hi] of a setting for the class (classes.gd TUNE), widened to the flown value
static func bounds(cls: String, flown: Dictionary, k: Array) -> Array:
	var t: Dictionary = Classes.TUNE[cls]
	var rpm := int(flown.get("rpm_filter_harmonics", 0)) > 0
	var lo := 0.0
	var hi := 0.0
	var a: int = k[1]
	match k[0]:
		"p", "i", "ff":
			lo = float(t[k[0]][0]) * float(Classes.TUNE_DEFAULT[k[0]][a])
			hi = float(t[k[0]][1]) * float(Classes.TUNE_DEFAULT[k[0]][a])
		"d", "d_max":
			hi = (float(t.d_max) if rpm else 1.0) * float(Classes.TUNE_DEFAULT.d_max[a])
		_:
			lo = float(t[k[0]][0])
			hi = float(t[k[0]][1])
	var v := get_v(flown, k)
	return [floorf(minf(lo, v)), ceilf(maxf(hi, v))]


## The D/P band of an axis holds (widened to the flown ratio)
static func dp_ok(cls: String, flown: Dictionary, cfg: Dictionary, a: int) -> bool:
	if a == 2 or float(cfg.pid[AX[a]][0]) <= 0.0:
		return true
	var band: Array = Classes.TUNE[cls].dp
	var r0 := peak_d(flown, a) / maxf(float(flown.pid[AX[a]][0]), 1.0)
	var r := peak_d(cfg, a) / float(cfg.pid[AX[a]][0])
	return r >= minf(float(band[0]), r0) - 1e-9 and r <= maxf(float(band[1]), r0) + 1e-9


# ------------------------------------------------------------------ the flights

## Stick programs (4 values a tick: roll, pitch, yaw, throttle as the game's sticks -1..1) of the step and
## flip flights: {name, group, sticks, edges (ticks where a step starts)}
static func programs(hover: float) -> Array:
	var out: Array = []
	var stick := 0.5
	var mk := func(name: String, group: String, segs: Array) -> Dictionary:
		var s := PackedFloat64Array()
		var edges := PackedInt32Array()
		for sg in segs:
			edges.append(s.size() / 4)
			for i in int(float(sg[0]) * TICK_HZ):
				s.append_array([sg[1], sg[2], sg[3], sg[4]])
		return {"name": name, "group": group, "sticks": s, "edges": edges}
	var h := hover
	var segs: Array = []
	for ax in 3:
		for sgn in [1.0, -1.0]:
			var v := [0.0, 0.0, 0.0]
			v[ax] = sgn * stick
			segs.append([0.3, v[0], v[1], v[2], h])
			segs.append([0.4, 0.0, 0.0, 0.0, h])
	out.append(mk.call("steps_hover", "steps", segs))
	var low := -0.9
	out.append(mk.call("steps_idle", "steps", [[0.25, 0.6, 0, 0, low], [0.35, 0, 0, 0, low], [0.25, 0, 0.6, 0, low], [0.35, 0, 0, 0, low]]))
	out.append(mk.call("steps_punch", "steps", [[0.3, 0, 0, 0, 1.0], [0.25, 0.5, 0, 0, 1.0], [0.3, 0, 0, 0, 1.0], [0.25, -0.5, 0, 0, 1.0], [0.3, 0, 0, 0, 1.0]]))
	out.append(mk.call("flip", "flip", [[0.3, 1.0, 0, 0, -0.6], [0.5, 0, 0, 0, h + 0.15], [0.3, 0, 1.0, 0, -0.6], [0.5, 0, 0, 0, h + 0.15]]))
	return out


## The REPLAY_N busiest windows of the logs as stick programs (armed, no chirp, no crash; deterministic)
static func replay_programs(logs: Array, cfg: Dictionary) -> Array:
	var cands: Array = []
	for li in logs.size():
		var L: Flight = logs[li]
		if L.n == 0 or L.rc.size() < 4 or L.setpoint.size() < 3:
			continue
		var w := int((REPLAY_S + SETTLE_S) * L.fs)
		var i0 := 0
		while i0 + w < L.n:
			var ok := true
			var score := 0.0
			for i in range(i0, i0 + w):
				if L.armed[i] == 0 or (L.chirp.size() > i and L.chirp[i] != 0) or absf(L.setpoint[0][i] - L.gyro[0][i]) > 500.0:
					ok = false
					break
				score += absf(L.setpoint[0][i]) + absf(L.setpoint[1][i]) + 0.5 * absf(L.setpoint[2][i])
			if ok:
				cands.append([score / w, li, i0])
			i0 += int(L.fs)
	cands.sort_custom(func(x, y): return x[0] > y[0] or (x[0] == y[0] and (x[1] < y[1] or (x[1] == y[1] and x[2] < y[2]))))
	var picked: Array = []
	for c in cands:
		var clash := false
		for p in picked:
			if p[1] == c[1] and absi(int(p[2]) - int(c[2])) < int((REPLAY_S + SETTLE_S) * logs[c[1]].fs):
				clash = true
		if not clash:
			picked.append(c)
		if picked.size() >= REPLAY_N:
			break
	var mc := float(cfg.get("mincheck", 1050))
	var out: Array = []
	for c in picked:
		var L: Flight = logs[c[1]]
		var s := PackedFloat64Array()
		for k in int((REPLAY_S + SETTLE_S) * TICK_HZ):
			var i := mini(int(c[2]) + roundi(k * L.fs / TICK_HZ), L.n - 1)
			var thr: float = mc + (L.rc[3][i] - 1000.0) * (2000.0 - mc) / 1000.0
			s.append_array([L.rc[0][i] / 500.0, L.rc[1][i] / 500.0, -L.rc[2][i] / 500.0, (thr - 1000.0) / 500.0 - 1.0])
		out.append({"name": "replay %s %.0f s" % [L.name, L.t[int(c[2])]], "group": "replay", "sticks": s, "edges": PackedInt32Array()})
	return out


## Flies a stick program: {err (deg/s), sat (share), ring (deg/s), t90 (s), overshoot} of the part after the
## settle time
static func fly(sim, cfg: Dictionary, prog: Dictionary) -> Dictionary:
	sim.model.fc_cfg = cfg
	sim.start()
	sim.hover_at(HOVER_S)
	var s: PackedFloat64Array = prog.sticks
	var n := s.size() / 4
	var e: Array = [PackedFloat64Array(), PackedFloat64Array(), PackedFloat64Array()]
	var sp: Array = [PackedFloat64Array(), PackedFloat64Array(), PackedFloat64Array()]
	var sat := 0
	var warm := int(SETTLE_S * TICK_HZ) if prog.group == "replay" else 0
	var se := 0.0
	var cnt := 0
	for i in n:
		if i % 50 == 0 and sim._cancelled():
			return {}
		sim.rc.roll = s[4 * i]
		sim.rc.pitch = s[4 * i + 1]
		sim.rc.yaw = s[4 * i + 2]
		sim.rc.throttle = s[4 * i + 3]
		sim.tick()
		var r: Vector3 = sim.rates()
		var fc = sim.model.fc
		for a in 3:
			e[a].append(fc.setpoint_rate[a] - r[a])
			sp[a].append(fc.setpoint_rate[a])
		if i >= warm:
			se += e[0][i] * e[0][i] + e[1][i] * e[1][i] + e[2][i] * e[2][i]
			cnt += 1
			if fc.motor_mix_range > 1.0:
				sat += 1
	if not is_finite(se):
		return {"err": INF, "sat": 1.0, "ring": INF}
	var out := {"err": sqrt(se / maxf(3.0 * cnt, 1.0)), "sat": float(sat) / maxf(cnt, 1.0), "ring": 0.0}
	var edges: PackedInt32Array = prog.edges
	if edges.size() > 0:
		out.ring = _ringing(e, edges, n)
		out.merge(_steps(sp, e, edges))
	return out


## RMS of the tracking error band-passed 20-200 Hz on the plateaus (0.08 s after each step's start to the next)
static func _ringing(e: Array, edges: PackedInt32Array, n: int) -> float:
	var sos := Num.butter(2, [20.0, 200.0], "bandpass", TICK_HZ)
	var ss := 0.0
	var cnt := 0
	for a in 3:
		var f := Num.sosfiltfilt(sos, e[a])
		for k in edges.size():
			var a0 := edges[k] + int(0.08 * TICK_HZ)
			var a1 := edges[k + 1] if k + 1 < edges.size() else n
			for i in range(a0, a1):
				ss += f[i] * f[i]
				cnt += 1
	return sqrt(ss / maxf(cnt, 1.0))


## t90 of the first step and the largest overshoot of the roll and pitch steps: {t90 (s), overshoot (share of
## the plateau)}
static func _steps(sp: Array, e: Array, edges: PackedInt32Array) -> Dictionary:
	var out := {"t90": -1.0, "overshoot": 0.0}
	for k in edges.size():
		var a := edges[k]
		var b: int = edges[k + 1] if k + 1 < edges.size() else (sp[0] as PackedFloat64Array).size()
		for ax in 2:
			var plateau: float = sp[ax][b - 1]
			if absf(plateau) < 50.0:
				continue
			var sg := signf(plateau)
			var peak := 0.0
			for i in range(a, b):
				var g: float = sg * (sp[ax][i] - e[ax][i])
				if k == 0 and out.t90 < 0.0 and g >= 0.9 * absf(plateau):
					out.t90 = float(i - a) / TICK_HZ
				peak = maxf(peak, g)
			out.overshoot = maxf(out.overshoot, peak / absf(plateau) - 1.0)
	return out


# ------------------------------------------------------------------ the evaluation

## {cost, groups {name: metric}, sat, ring_p, ok, why, flights [{name, err}], t90, overshoot}
func evaluate(sims: Array, cfg: Dictionary, progs: Array, ref: Dictionary) -> Dictionary:
	var tasks: Array = []
	for p in progs:
		tasks.append({"prog": p, "variant": "nominal"})
	var hover_steps: Dictionary = progs.filter(func(p): return p.name == "steps_hover")[0]
	tasks.append({"prog": hover_steps, "variant": "perturbed"})
	if sims[0].pw != null:
		for sd in PW_SEEDS:
			tasks.append({"prog": {"name": "propwash seed %d" % (sd + 1), "group": "propwash"}, "variant": "air", "seed": sd + 1})
	var res := []
	res.resize(tasks.size())
	Num.parallel_for(sims.size(), func(t: int) -> void: _worker(sims[t], cfg, tasks, t, sims.size(), res))
	evals += 1
	var groups := {}
	var counts := {}
	var sat := 0.0
	var nsat := 0
	var ring_p := 0.0
	var out := {"flights": [], "ok": true, "why": ""}
	for i in tasks.size():
		var r: Dictionary = res[i] if res[i] is Dictionary else {}
		if r.is_empty():
			return {"cost": INF, "ok": false, "why": "cancelled"}
		var p: Dictionary = tasks[i].prog
		if tasks[i].variant == "perturbed":
			ring_p = float(r.ring)
			continue
		var g: String = p.group
		groups[g] = float(groups.get(g, 0.0)) + float(r.err)
		counts[g] = int(counts.get(g, 0)) + 1
		out.flights.append({"name": p.name, "err": r.err})
		if r.has("sat"):
			sat += float(r.sat)
			nsat += 1
		if p.name == "steps_hover":
			out["t90"] = r.get("t90", -1.0)
			out["overshoot"] = r.get("overshoot", 0.0)
			out["ring"] = r.ring
	for g in groups:
		groups[g] = float(groups[g]) / counts[g]
	out["groups"] = groups
	out["sat"] = sat / maxf(nsat, 1.0)
	out["ring_p"] = ring_p
	if ref.is_empty():
		out["cost"] = 1.0
		return out
	var c := 0.0
	for g in groups:
		c += float(groups[g]) / maxf(float(ref.groups[g]), 1e-6)
	out["cost"] = c / groups.size()
	if not is_finite(float(out.cost)):
		out.ok = false
		out.why = "diverged"
	elif float(out.sat) > float(ref.sat) + SAT_SLACK:
		out.ok = false
		out.why = "saturation"
	elif ring_p > maxf(RING_SLACK * float(ref.ring_p), RING_FLOOR):
		out.ok = false
		out.why = "robustness"
	elif float(out.overshoot) > maxf(float(ref.overshoot), MAX_OVERSHOOT):
		out.ok = false
		out.why = "overshoot"
	return out


static func _worker(sim: Dictionary, cfg: Dictionary, tasks: Array, first: int, nth: int, res: Array) -> void:
	for i in range(first, tasks.size(), nth):
		var t: Dictionary = tasks[i]
		if t.variant == "air":
			var s = sim.pw
			s.model.fc_cfg = cfg
			s.seed_n = int(t.seed)
			var m: float = s.run("dropcatch")
			res[i] = {} if s._cancelled() else {"err": m}
		else:
			res[i] = fly(sim[t.variant], cfg, t.prog)


# ------------------------------------------------------------------ the search

## Tunes. craft, fc, plant: the package's dictionaries; logs: Flight sessions for the stick replay (may be
## empty); cls: the craft class. {ok, why, fc (the tuned fc.json), record (for autotune.json and the report)}
func run(craft: Dictionary, fc: Dictionary, plant: Dictionary, logs: Array, cls: String) -> Dictionary:
	var t0 := Time.get_ticks_msec()
	if not Classes.TUNE.has(cls):
		return {"ok": false, "why": "unknown craft class %s" % cls}
	if ResourceLoader.exists(Calibrate.AIR) or FileAccess.file_exists(Calibrate.AIR):
		var sc: Variant = load(Calibrate.AIR)
		_air_script = sc if sc is Script and (sc as Script).can_instantiate() else null
	var sims := _sims(craft, fc, plant)
	if sims.is_empty():
		return {"ok": false, "why": "the model could not be loaded"}
	var hover: float = sims[0].nominal.find_hover()
	for s in sims:
		for v in ["nominal", "perturbed", "pw"]:
			if s[v] != null:
				s[v].hover_thr = hover
	var progs := programs(hover) + replay_programs(logs, fc)
	var best: Dictionary = fc.duplicate(true)
	var ref := evaluate(sims, best, progs, {})
	var best_e := ref.duplicate(true)
	best_e["cost"] = 1.0
	var why := {}
	var stages: Array = [[STAGE1, STEPS1, true], [STAGE2, [1.0, 0.5].slice(0, ROUNDS2), false]]
	var n_steps := 0
	for st in stages:
		n_steps += (st[0] as Array).size() * 2 * (st[1] as Array).size()
	var done := 0
	for st in stages:
		var knobs: Array = (st[0] as Array).filter(func(k): return _tunable(best, k))
		for step in st[1]:
			for k in knobs:
				var v := get_v(best, k)
				var b := bounds(cls, fc, k)
				var moved := false
				for dir in [1.0, -1.0]:
					var wk := "%s %s" % [label(k), "up" if dir > 0.0 else "down"]     # what stopped the move
					done += 1
					if progress.is_valid():
						progress.call(minf(float(done) / n_steps, 1.0))
					if moved or cancel_flag:
						continue
					var nv: float = v * (1.0 + dir * float(step)) if st[2] else v + dir * float(step) * float(STEPS2[k[0]])
					nv = clampf(roundf(nv), b[0], b[1])
					if nv == v:
						why[wk] = "class bound"
						continue
					var cand: Dictionary = best.duplicate(true)
					set_v(cand, k, nv)
					if k[1] >= 0 and peak_d(cand, k[1]) > b[1] and k[0] in ["d", "d_max"]:
						why[wk] = "class bound (highest D)"
						continue
					if k[1] >= 0 and not dp_ok(cls, fc, cand, k[1]):
						why[wk] = "class bound (D/P band)"
						continue
					var e := evaluate(sims, cand, progs, ref)
					if cancel_flag:
						break
					if not e.ok:
						why[wk] = {"saturation": "motor saturation", "robustness": "ringing on the perturbed plant", "overshoot": "overshoot"}.get(e.why, e.why)
					elif float(e.cost) < float(best_e.cost) - MIN_GAIN:
						best = cand
						best_e = e
						moved = true
						why[wk] = "its last step still gained"
					else:
						why[wk] = "no better flight"
				if cancel_flag:
					break
	for s in sims:
		for v in ["nominal", "perturbed", "pw"]:
			if s[v] != null:
				s[v].dispose()
	if cancel_flag:
		return {"ok": false, "why": "cancelled"}
	var rec := {"class": cls, "flown": ref, "tuned": best_e, "why": why, "evals": evals, "flights": progs.map(func(p): return p.name),
		"runtime_s": Num.pyround((Time.get_ticks_msec() - t0) / 1000.0, 1), "hover_throttle": hover, "propwash": sims[0].pw != null}
	return {"ok": true, "why": "", "fc": best, "record": rec}


## A setting the craft uses: yaw D not at all, dynamic idle only with the RPM filter (bidirectional DShot)
static func _tunable(cfg: Dictionary, k: Array) -> bool:
	if k[0] == "dyn_idle_min_rpm":
		return int(cfg.get("rpm_filter_harmonics", 0)) > 0
	return true


## Per worker (one of parallel_for's): {nominal, perturbed: Sim (clean air), pw: Sim with dirty_air's air or null}
func _sims(craft: Dictionary, fc: Dictionary, plant: Dictionary) -> Array:
	var pert := plant.duplicate(true)
	for k in ["Ixx", "Iyy", "Izz"]:
		pert[k] = float(plant[k]) * PERTURB_INERTIA
	pert["motor_delay_s"] = float(plant.get("motor_delay_s", 0.001)) * PERTURB_DELAY
	var dirs := [_tmp_package(craft, fc, plant), _tmp_package(craft, fc, pert)]
	var out: Array = []
	var nth := clampi(OS.get_processor_count() - 1, 1, MAX_THREADS)
	for t in nth:
		var w := {}
		for v in 2:
			var m := BFModel.new(dirs[v])
			if not m.valid:
				return []
			m.quad.turbulence_enabled = false       # (clean, still air: the ringing is the loop's own)
			var s := Calibrate.Sim.new(m, null, 1)
			s.cal = self
			w["nominal" if v == 0 else "perturbed"] = s
		w["pw"] = null
		if _air_script != null and _air_script.new().has_method("free_start"):
			var s := Calibrate.Sim.new(BFModel.new(dirs[0]), _air_script.new(), 1)
			s.cal = self
			w["pw"] = s
		out.append(w)
	for d in dirs:
		for f in DirAccess.get_files_at(d):
			DirAccess.remove_absolute(d.path_join(f))
		DirAccess.remove_absolute(d)
	return out


static func _tmp_package(craft: Dictionary, fc: Dictionary, plant: Dictionary) -> String:
	var dir := OS.get_user_data_dir().path_join("real_quad/autotune_tmp/%d_%d" % [OS.get_process_id(), Time.get_ticks_usec()])
	DirAccess.make_dir_recursive_absolute(dir)
	for pair in [["craft.json", craft], ["fc.json", fc], ["plant.json", plant]]:
		var f := FileAccess.open(dir.path_join(pair[0]), FileAccess.WRITE)
		f.store_string(JSON.stringify(pair[1]))
		f.close()
	return dir


# ------------------------------------------------------------------ the package

## fc_optimized.json: the tuned settings and, as "_tune", what they were made from (the SHA-256 of the texts of
## fc.json and plant.json: bf_model flies them only with those) and the record
static func optimized_json(tuned: Dictionary, fc_text: String, plant_text: String, rec: Dictionary) -> Dictionary:
	var o := tuned.duplicate(true)
	o["_tune"] = {"fc_sha256": fc_text.sha256_text(), "plant_sha256": plant_text.sha256_text(),
		"created": Time.get_datetime_string_from_system(true) + "+00:00", "record": rec}
	return o


## The line for the build's report: "Optimized tune (tune.md): ..." or why there is none
static func summary(rec: Variant, grade: String) -> String:
	if not (rec is Dictionary):
		return "No optimized tune: grade %s, the plant is not identified well enough for it" % grade if grade in ["C", "D"] else "No optimized tune: %s" % str(rec)
	var parts: Array = []
	var f: Dictionary = rec.flown.groups
	var t: Dictionary = rec.tuned.groups
	for g in ["replay", "steps", "propwash"]:
		if f.has(g):
			parts.append("%s %+.0f %%" % [{"replay": "your sticks", "steps": "stick steps", "propwash": "prop wash"}[g], 100.0 * (float(t[g]) / maxf(float(f[g]), 1e-6) - 1.0)])
	return "Optimized tune (tune.md, simulator only): tracking error %s" % ", ".join(parts)


# ------------------------------------------------------------------ the report

## tune.md: the optimized tune
static func report_md(name: String, flown: Dictionary, tuned: Dictionary, rec: Dictionary) -> String:
	var L: Array = ["# Optimized tune of %s" % name, "",
		"How this quad would probably fly when well tuned: its identified plant flown in the simulator with the Betaflight settings below (a drone flies them when its tune setting in real_quad's settings is \"optimized\"). **Simulator only** - not a tune for the real quad: no noise, motor heat or frame resonances are simulated, and the filters stay as flown.", "",
		"## Settings", "", "| Setting | Flown | Tuned | Stopped by (the last tries) |", "|---|---|---|---|"]
	for k in STAGE1 + STAGE2:
		if not _tunable(flown, k):
			continue
		var a := get_v(flown, k)
		var b := get_v(tuned, k)
		var up := str(rec.why.get(label(k) + " up", "-"))
		var dn := str(rec.why.get(label(k) + " down", "-"))
		var lim := up if b > a else dn if b < a else up if up == dn else "up: %s; down: %s" % [up, dn]
		L.append("| %s | %d | %d | %s |" % [label(k), int(a), int(b), lim])
	var f: Dictionary = rec.flown
	var t: Dictionary = rec.tuned
	L.append_array(["", "## Predicted changes (simulator, the same flights for both)", "", "| Flight | Flown | Tuned | Change |", "|---|---|---|---|"])
	var names := {"replay": "Your stick replay, tracking error (deg/s)", "steps": "Stick steps, tracking error (deg/s)",
		"flip": "Flips, tracking error (deg/s)", "propwash": "Drop and catch, worst prop wash (deg/s)"}
	for g in f.groups:
		L.append("| %s | %.1f | %.1f | %+.0f %% |" % [names.get(g, g), f.groups[g], t.groups[g], 100.0 * (float(t.groups[g]) / maxf(float(f.groups[g]), 1e-6) - 1.0)])
	L.append("| Hover roll step: time to 90 %% (ms) | %.0f | %.0f | |" % [1000.0 * float(f.get("t90", -1)), 1000.0 * float(t.get("t90", -1))])
	L.append("| Hover roll and pitch steps: overshoot (%%) | %.0f | %.0f | at most %.0f |" % [100.0 * float(f.get("overshoot", 0)), 100.0 * float(t.get("overshoot", 0)),
		100.0 * maxf(float(f.get("overshoot", 0)), MAX_OVERSHOOT)])
	L.append("| Out of motor authority (%% of the time) | %.2f | %.2f | at most +%.1f |" % [100.0 * float(f.sat), 100.0 * float(t.sat), 100.0 * SAT_SLACK])
	L.append("| Ringing, plant with %d %% inertia and %d %% motor delay (deg/s) | %.2f | %.2f | at most %.2f |" % [
		roundi(100 * PERTURB_INERTIA), roundi(100 * PERTURB_DELAY), f.ring_p, t.ring_p, maxf(RING_SLACK * float(f.ring_p), RING_FLOOR)])
	L.append_array(["", "## How", "",
		"- Flights: %s." % ", ".join(rec.flights) + ("" if rec.propwash else " No prop wash flight: dirty_air is not installed."),
		"- Search: staged coordinate search from the flown tune (P, D, D-max, feedforward, I per axis; then TPA, dynamic idle, anti-gravity, thrust linear), %d evaluations, %.0f s." % [rec.evals, rec.runtime_s],
		"- Bounds: the %s class's (P, I, feedforward around Betaflight's defaults; highest D at most %.2f x the default D-max with the RPM filter; D/P band), widened to the flown values." % [rec["class"], float(Classes.TUNE[rec["class"]].d_max)],
		"- Constraints: no more time out of motor authority than the flown tune; no more ringing than the flown tune on a plant with less inertia and more motor delay (the identification's error); hover steps overshoot at most %.0f %% (or as flown)." % (100.0 * MAX_OVERSHOOT),
		"", "## Not modelled", "",
		"- Gyro noise, motor heat and D-term noise: a real quad with this much D may run hot; the class's D bound stands in for them.",
		"- Frame resonances, bent props, the receiver link's jitter; the filters are not tuned.",
		"- The plant is the identified one: where the identification is off, so is this.", ""])
	return "\n".join(L)
