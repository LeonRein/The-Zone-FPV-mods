extends RefCounted
## real_quad logfit: input checks (contract 2.1, 2.3, 3.1-3.3, 3.6): firmware, required content,
## session skip rules, consistency across logs, sensor plausibility. Every finding is an issue
## {code, level, message, fix, where} with the reference's exact message and the fix shown to the
## pilot; errors refuse the run (nothing is written).

const Num := preload("num.gd")
const Craft := preload("craft.gd")
const Flight := preload("flight.gd")

const MIN_FW := [2025, 12]
const MAX_ARMED_S := 3600.0
const MIN_ARMED_S := 5.0

const FIX := {
	"gyro": "set blackbox_disable_gyro = OFF",
	"setpoint_rc": "set blackbox_disable_setpoint = OFF and set blackbox_disable_rc = OFF",
	"motors": "set blackbox_disable_motors = OFF",
	"erpm": "set dshot_bidir = ON, set motor_poles = <n> (magnets of your motor), set blackbox_disable_rpm = OFF; the ESC firmware must support bidirectional DShot (BLHeli_32, AM32, Bluejay; BLHeli_S needs Bluejay; brushed: impossible)",
	"firmware": "update the flight controller to Betaflight 2025.12 or newer and log new flights",
	"rate": "set blackbox_sample_rate = 1/<k> with k = the PID loop in kHz (8 kHz: 1/8, 4 kHz: 1/4, 2 kHz: 1/2) for a 1 kHz log",
	"acc": "set blackbox_disable_acc = OFF; ACC must be on (acc_hardware not NONE) and calibrated",
	"chirp": "put CHIRP on a switch, set the chirp settings (CLI block below) and fly 2-3 rounds of roll, pitch and yaw sweeps",
	"vbat": "set blackbox_disable_bat = OFF; calibrate the voltage sensor (vbat_scale)",
	"unfilt": "set blackbox_disable_gyrounfilt = OFF",
	"pid": "set blackbox_disable_pids = OFF",
	"one_quad": "pick logs of this quad only",
}

const REQUIRED_TEXT := {"gyro": "Gyro is not logged", "setpoint_rc": "Setpoint and RC are not logged",
	"motors": "Motor outputs are not logged", "erpm": "eRPM (bidirectional DShot) is not logged",
	"rate": "The log rate is below 500 Hz"}


static func issue(code: String, level: String, message: String, fix: String = "", where: Variant = null) -> Dictionary:
	return {"code": code, "level": level, "message": message, "fix": fix, "where": where}


# ---------------------------------------------------------------- firmware --

## [name, [year/major, minor, patch] or null, target or null, raw] from "Firmware revision".
static func parse_firmware(h: Dictionary) -> Array:
	var raw := str(h.get("Firmware revision", "")).strip_edges()
	var re := RegEx.create_from_string("^(\\S+)\\s+(\\d+)\\.(\\d+)(?:\\.(\\d+))?")
	var m := re.search(raw)
	if m == null:
		return [raw.split(" ")[0] if raw != "" else null, null, null, raw]
	var patch := m.get_string(4)
	var ver := [m.get_string(2).to_int(), m.get_string(3).to_int(), patch.to_int() if patch != "" else 0]
	var parts := raw.split(" ", false)
	var target: Variant = parts[parts.size() - 1] if ")" in raw else null
	return [m.get_string(1), ver, target, raw]


static func firmware_issue(L: Flight) -> Variant:
	var fw := parse_firmware(L.headers)
	var raw: String = fw[3]
	if raw == "":
		return issue("firmware_missing", "error", "%s: the header has no 'Firmware revision'" % L.name, FIX.firmware, L.name)
	if fw[0] != "Betaflight":
		return issue("firmware_not_betaflight", "error",
			"%s: firmware '%s' is not Betaflight; only Betaflight 2025.12 or newer is supported" % [L.name, raw], FIX.firmware, L.name)
	var ver: Variant = fw[1]
	if ver == null or ver[0] < MIN_FW[0] or (ver[0] == MIN_FW[0] and ver[1] < MIN_FW[1]):
		var vs := "%d.%d.%d" % ver if ver != null else "?"
		return issue("firmware_too_old", "error", "%s: Betaflight %s is too old; Betaflight 2025.12 or newer is required" % [L.name, vs],
			FIX.firmware, L.name)
	return null


# ---------------------------------------------------------- content checks --

static func erpm_valid_fraction(L: Flight) -> float:
	if not L.has_hz() or L.armed.count(1) == 0:
		return 0.0
	var good := 0
	var tot := 0
	for i in L.n:
		if L.armed[i] == 0:
			continue
		tot += 1
		var ok := true
		for m in 4:
			var v: float = L.motor_hz[m][i]
			if not (is_finite(v) and v > 0.0):
				ok = false
				break
		if ok:
			good += 1
	return float(good) / tot


## Required items (contract 3.2) this session fails: {item: detail}.
static func required_failures(L: Flight) -> Dictionary:
	var f: Dictionary = L.info.flags
	var out := {}
	if not f.get("gyro", false):
		out.gyro = "no gyroADC[0..2]"
	if not (f.get("setpoint", false) or f.get("rc", false)):
		out.setpoint_rc = "neither setpoint[0..3] nor rcCommand[0..3]"
	if not f.get("motors", false):
		out.motors = "no motor[0..3]"
	if not f.get("erpm", false):
		out.erpm = "no eRPM[0..3] (bidirectional DShot telemetry)"
	else:
		var v := erpm_valid_fraction(L)
		if v < 0.95:
			out.erpm = "eRPM valid in only %.0f %% of the armed samples (> 95 %% needed)" % (v * 100.0)
	if L.fs_raw < 500.0:
		out.rate = "log rate %.0f Hz (at least 500 Hz needed)" % L.fs_raw
	return out


## Session skip rules of contract 2.1 that need no comparison with other sessions (null: usable).
static func skip_reason(L: Flight) -> Variant:
	if L.n == 0:
		return "no main frames after resync"
	if L.armed.count(1) == 0:
		return "never armed"
	var armed_s := float(L.armed.count(1)) / L.fs
	if armed_s < MIN_ARMED_S:
		return "only %.1f s armed (at least 5 s)" % armed_s
	return null


## The rate curve of one axis on a grid of 2001 stick positions -1..1 (Betaflight port's rates).
static func _rate_table(fc, axis: int) -> Array:
	var grid := PackedFloat64Array()
	var tab := PackedFloat64Array()
	for k in 2001:
		var x := -1.0 + 2.0 * k / 2000.0
		grid.append(x)
		tab.append(fc.apply_rates(axis, x, absf(x)))
	return [grid, tab]


## Setpoint missing but RC present: rebuilt from rcCommand through the header's rates (no chirp
## content: chirps count as missing).
static func rebuild_setpoint(L: Flight, Controller: Script, cfg: Dictionary) -> void:
	var fc = Controller.new(cfg, false)
	var sp: Array = []
	for a in 3:
		var db := float(cfg.yaw_deadband if a == 2 else cfg.deadband)
		var gt := _rate_table(fc, a)
		var lim := float(cfg.rate_limits[a])
		var c := PackedFloat64Array()
		c.resize(L.n)
		var rca: PackedFloat64Array = L.rc[a]
		for i in L.n:
			var rcf := clampf(rca[i] / (500.0 - db), -1.0, 1.0)
			c[i] = clampf(Num.interp(rcf, gt[0], gt[1]), -lim, lim)
		sp.append(c)
	L.setpoint = sp
	L.chirp = Num.zeros_mask(L.n)
	L.info.setpoint_rebuilt = true


## RC missing but setpoint present: rcCommand that gives the logged setpoint behind the rates.
static func rebuild_rc(L: Flight, Controller: Script, cfg: Dictionary) -> void:
	var fc = Controller.new(cfg, false)
	var rc: Array = []
	for a in 3:
		var db := float(cfg.yaw_deadband if a == 2 else cfg.deadband)
		var gt := _rate_table(fc, a)
		var c := PackedFloat64Array()
		c.resize(L.n)
		var spa: PackedFloat64Array = L.setpoint[a]
		for i in L.n:
			c[i] = Num.interp(spa[i], gt[1], gt[0]) * (500.0 - db)
		rc.append(c)
	var c3 := PackedFloat64Array()
	c3.resize(L.n)
	for i in L.n:
		c3[i] = 1000.0 + 1000.0 * L.throttle[i]
	rc.append(c3)
	L.rc = rc
	L.info.rc_rebuilt = true


# -------------------------------------------------------------- voltages --

## Pack voltage at the first arm: median of the first 0.3 s of armed samples at low throttle (-1: none).
static func resting_voltage(L: Flight) -> float:
	if not L.has_vbat():
		return -1.0
	var k := maxi(3, int(0.3 * L.fs))
	var v := PackedFloat64Array()
	for i in L.n:
		if L.armed[i] != 0 and L.throttle[i] < 0.1:
			v.append(L.vbat[i])
			if v.size() >= k:
				break
	if v.size() < 3:
		v = L.vbat.slice(0, mini(L.n, k))
	return Num.median(v)


static func cells_candidates(v: float, pack: String = "lipo") -> Array:
	var lo := 2.8 if pack == "liion" else 3.3
	var hi := 4.25 if pack == "liion" else 4.45
	var out: Array = []
	for n in range(1, 13):
		if lo <= v / n and v / n <= hi:
			out.append(n)
	return out


## [cells or 0, ambiguous]
static func cells_guess(v: float, pack: String = "lipo") -> Array:
	var c := cells_candidates(v, pack)
	if c.is_empty():
		return [0, false]
	var ref := 3.7 if pack == "liion" else 4.0
	var best: int = c[0]
	for n in c:
		if absf(v / n - ref) < absf(v / best - ref):
			best = n
	return [best, c.size() > 1]


# -------------------------------------------------------------- sanity --

static func _gmax(L: Flight) -> PackedFloat64Array:
	var o := PackedFloat64Array()
	o.resize(L.n)
	var g0: PackedFloat64Array = L.gyro[0]
	var g1: PackedFloat64Array = L.gyro[1]
	var g2: PackedFloat64Array = L.gyro[2]
	for i in L.n:
		o[i] = maxf(maxf(absf(g0[i]), absf(g1[i])), absf(g2[i]))
	return o


## On the ground without trusting the ACC: quiet gyro at low throttle (or disarmed) for >= 0.3 s
## touching the log start/end, or >= 2 s anywhere.
static func _ground(L: Flight, hover_thr: float) -> PackedByteArray:
	var k := maxi(3, int(0.05 * L.fs))
	var gmax := Num.uniform_filter1d(_gmax(L), k)
	var cand := PackedByteArray()
	cand.resize(L.n)
	for i in L.n:
		cand[i] = 1 if (L.throttle[i] < 0.5 * hover_thr or L.armed[i] == 0) and gmax[i] < 8.0 else 0
	var out := Num.zeros_mask(L.n)
	var r := Num.runs(cand)
	for j in range(0, r.size(), 2):
		var a := r[j]
		var b := r[j + 1]
		var dur := L.t[b - 1] - L.t[a]
		if (dur >= 0.3 and (a == 0 or b >= L.n - 1)) or dur >= 2.0:
			for i in range(a, b):
				out[i] = 1
	return out


## Longest run of a constant value of at least min_s while armed (> 90 %) and the throttle changes.
static func _stuck(x: PackedFloat64Array, armed: PackedByteArray, thr: PackedFloat64Array, fs: float, exclude: Callable = Callable(), min_s: float = 2.0) -> float:
	var n := x.size()
	if n < 2:
		return 0.0
	var worst := 0.0
	var a := 0
	while a < n:
		var b := a + 1
		while b < n and x[b] == x[a]:
			b += 1
		if b - a >= min_s * fs and not (exclude.is_valid() and exclude.call(x[a])):
			var na := 0
			var lo := INF
			var hi := -INF
			for i in range(a, b):
				na += armed[i]
				lo = minf(lo, thr[i])
				hi = maxf(hi, thr[i])
			if float(na) / (b - a) > 0.9 and hi - lo > 0.1:
				worst = maxf(worst, (b - a) / fs)
		a = b
	return worst


## Sensor plausibility of one session (contract 3.6): {issues, drop_acc, drop_vbat, skip, skip_fit,
## metrics}. The caller applies the drops.
static func sanity(L: Flight, hover_thr: float, cells: int, pack: String, erpm_limit_hz: float) -> Dictionary:
	var nm := L.name
	var iss: Array = []
	var res := {"issues": iss, "drop_acc": false, "drop_vbat": false, "skip": null, "skip_fit": null, "metrics": {}}
	var met: Dictionary = res.metrics
	var narm := L.armed.count(1)
	var fl := PackedByteArray()
	fl.resize(L.n)
	for i in L.n:
		fl[i] = 1 if L.armed[i] != 0 and L.throttle[i] > 0.05 else 0
	var ground := _ground(L, hover_thr)
	# ACC
	if L.has_acc():
		var ax: PackedFloat64Array = L.acc[0]
		var ay: PackedFloat64Array = L.acc[1]
		var az: PackedFloat64Array = L.acc[2]
		var ng := ground.count(1)
		if ng >= 0.3 * L.fs:
			var an := PackedFloat64Array()
			var sx := PackedFloat64Array()
			var sy := PackedFloat64Array()
			for i in L.n:
				if ground[i] != 0:
					an.append(sqrt(ax[i] * ax[i] + ay[i] * ay[i] + az[i] * az[i]))
					sx.append(ax[i])
					sy.append(ay[i])
			var g := Num.median(an)
			met.acc_ground_g = Num.pyround(g, 3)
			if absf(g - 1.0) > 0.05:
				res.drop_acc = true
				iss.append(issue("acc_scale", "warning", "%s: ACC not calibrated or wrong scale (|a| on the ground %.2f g, 1 ± 0.05 expected): ACC not used for this session" % [nm, g], FIX.acc, nm))
			else:
				var lx := absf(Num.mean(sx))
				var ly := absf(Num.mean(sy))
				met.acc_level_g = [Num.pyround(lx, 3), Num.pyround(ly, 3)]
				if maxf(lx, ly) >= 0.1:
					iss.append(issue("acc_level", "warning", "%s: ACC not level on the ground (x %.2f g, y %.2f g): FC mounted tilted or ACC not calibrated" % [nm, lx, ly],
						"calibrate the accelerometer on a level surface", nm))
		else:
			met.acc_ground_g = null
		var lim: float = L.info.get("acc_limit_g", 16.0)
		var nfl := fl.count(1)
		if nfl > 0:
			var nc := 0
			for i in L.n:
				if fl[i] != 0 and (absf(ax[i]) >= 0.98 * lim or absf(ay[i]) >= 0.98 * lim or absf(az[i]) >= 0.98 * lim):
					nc += 1
			var clip := float(nc) / nfl
			met.acc_clip = Num.pyround(clip, 5)
			if clip > 0.005:
				iss.append(issue("acc_clip", "warning", "%s: ACC at its range limit in %.1f %% of the flight" % [nm, clip * 100.0], "", nm))
	# gyro
	if narm > 10:
		var sd: Array = []
		var dead: Array = []
		for a in 3:
			var s := Num.std(Num.select(L.gyro[a], L.armed))
			sd.append(Num.pyround(s, 2))
			if s < 0.5:
				dead.append("xyz"[a])
		met.gyro_std = sd
		if not dead.is_empty():
			res.skip = "gyro axis %s dead (std < 0.5 deg/s): sensor fault" % "/".join(dead)
		var nc := 0
		for i in L.n:
			if L.armed[i] != 0 and (absf(L.gyro[0][i]) >= 1900.0 or absf(L.gyro[1][i]) >= 1900.0 or absf(L.gyro[2][i]) >= 1900.0):
				nc += 1
		var clip := float(nc) / narm
		met.gyro_clip = Num.pyround(clip, 5)
		if clip > 0.02:
			iss.append(issue("gyro_clip", "warning", "%s: gyro at 1900 deg/s or more in %.1f %% of the flight (excluded from the rotation fit)" % [nm, clip * 100.0], "", nm))
	# stuck values
	var stuck: Array = []
	for i in 3:
		if _stuck(L.gyro[i], L.armed, L.throttle, L.fs) > 0.0:
			stuck.append("gyroADC[%d]" % i)
	if L.has_hz():
		for i in 4:
			if _stuck(L.motor_hz[i], L.armed, L.throttle, L.fs, func(v: float) -> bool: return v <= 0.0) > 0.0:
				stuck.append("eRPM[%d]" % i)
	for i in 4:
		var x: PackedFloat64Array = L.motor_raw[i]
		var lo := INF
		var hi := -INF
		for v in x:
			lo = minf(lo, v)
			hi = maxf(hi, v)
		if _stuck(x, L.armed, L.throttle, L.fs, func(v: float) -> bool: return v <= lo or v >= hi) > 0.0:
			stuck.append("motor[%d]" % i)
	if not stuck.is_empty() and res.skip == null:
		res.skip = "%s constant for > 2 s while the throttle changes: corrupt log" % ", ".join(stuck)
	# eRPM dropouts and glitches
	if L.has_hz() and narm > 0:
		var idle := float(L.hint("motor_idle", 550) if L.hint("motor_idle", 550) != 0 else 550) / 10000.0
		var bad := PackedFloat64Array([0, 0, 0, 0])
		for i in L.n:
			if L.armed[i] == 0:
				continue
			for m in 4:
				var hz: float = L.motor_hz[m][i]
				var uu := clampf((L.motor_raw[m][i] - 48.0) / 1999.0, 0.0, 1.0)
				if (hz < 30.0 and uu > idle + 0.05) or hz > erpm_limit_hz or hz < -1.0:
					bad[m] += 1.0
		var frac: Array = []
		var worst := 0.0
		var wm := 0
		for m in 4:
			var f := bad[m] / narm
			frac.append(Num.pyround(f, 4))
			if f > worst:
				worst = f
				wm = m
		met.erpm_bad = frac
		if worst > 0.10:
			res.skip_fit = "eRPM dropouts/glitches in %.0f %% of the flight on motor %d" % [worst * 100.0, wm + 1]
		elif worst > 0.01:
			iss.append(issue("erpm_dropouts", "warning", "%s: eRPM dropouts/glitches in %.1f %% of the flight on motor %d" % [nm, worst * 100.0, wm + 1],
				"check the ESC telemetry (bidirectional DShot) and the motor wiring", nm))
	# vbat
	if L.has_vbat():
		var vr := resting_voltage(L)
		met.vbat_rest = Num.pyround(vr, 2) if vr > 0.0 else null
		var why := ""
		var lo := INF
		var hi := -INF
		for v in L.vbat:
			lo = minf(lo, v)
			hi = maxf(hi, v)
		if hi - lo < 0.02:
			why = "constant"
		elif cells > 0:
			var pc := vr / cells
			var src := Num.select(L.vbat, L.armed) if narm > 0 else L.vbat
			var sm := Num.uniform_filter1d(src, maxi(3, int(0.5 * L.fs)))
			var vmin := INF
			for v in sm:
				vmin = minf(vmin, v)
			vmin /= cells
			met.vbat_min_cell = Num.pyround(vmin, 2)
			if not (3.0 <= pc and pc <= float(Craft.FULL_V.get(pack, 4.2)) + 0.1):
				why = "resting %.2f V/cell at %dS (3.0-4.45 expected)" % [pc, cells]
			elif vmin < 2.5:
				why = "%.2f V/cell minimum (< 2.5)" % vmin
		if why != "":
			res.drop_vbat = true
			iss.append(issue("vbat_implausible", "warning", "%s: battery voltage implausible (%s): treated as not logged" % [nm, why], FIX.vbat, nm))
	# corruption
	var st: Dictionary = L.info.get("stats", {})
	var frames: int = st.get("I", 0) + st.get("P", 0)
	var nbad: int = st.get("corrupt", 0) + st.get("desync", 0)
	var cr := float(nbad) / maxi(frames, 1)
	met.corrupt = Num.pyround(cr, 5)
	if cr > 0.10:
		if res.skip == null:
			res.skip = "%.0f %% corrupt frames" % (cr * 100.0)
	elif cr > 0.01:
		iss.append(issue("corrupt", "warning", "%s: %.1f %% corrupt or skipped frames" % [nm, cr * 100.0], "use a faster SD card or the onboard flash; erase the flash before flying", nm))
	# log rate
	met.log_hz = Num.pyround(L.fs_raw, 1)
	if 500.0 <= L.fs_raw and L.fs_raw < 900.0:
		iss.append(issue("rate_low", "warning", "%s: log rate %.0f Hz (1 kHz recommended)" % [nm, L.fs_raw], FIX.rate, nm))
	elif L.fs_raw >= 1500.0:
		iss.append(issue("rate_high", "info", "%s: log rate %.0f Hz, decimated to %.0f Hz (more data than needed: flash fills fast, SD cards drop frames)" % [nm, L.fs_raw, L.fs],
			FIX.rate, nm))
	return res


# --------------------------------------------------------- consistency --

## Contract 2.3 over the used sessions: issues (errors refuse the run).
static func consistency(sessions: Array, same_quad: bool = false, pack: String = "lipo") -> Array:
	var out: Array = []
	var poles := {}
	var pvals := {}
	for L: Flight in sessions:
		poles[L.name] = L.hint("motor_poles", 14)
		pvals[poles[L.name]] = true
	if pvals.size() > 1:
		var parts: Array = []
		for k in poles:
			parts.append("%s %d" % [k, poles[k]])
		out.append(issue("motor_poles_differ", "error", "different motors: motor_poles differs between logs (" + ", ".join(parts) + ")", FIX.one_quad))
	var boards := {}
	var names := {}
	for L: Flight in sessions:
		var fw := parse_firmware(L.headers)
		var b := [fw[2], str(L.headers.get("Board information", "")).strip_edges()]
		boards[str(b)] = b
		var cn := str(L.headers.get("Craft name", "")).strip_edges()
		if cn != "":
			names[cn] = true
	if boards.size() > 1 or names.size() > 1:
		var what: Array = []
		if boards.size() > 1:
			var bl: Array = []
			for b in boards.values():
				bl.append("%s (%s)" % [b[1] if b[1] != "" else "?", b[0] if b[0] != null else "?"])
			bl.sort()
			what.append("board " + " / ".join(bl))
		if names.size() > 1:
			var nl: Array = names.keys()
			nl.sort()
			what.append("craft name " + " / ".join(nl))
		var msg := "logs of different quads? " + "; ".join(what)
		out.append(issue("board_differs", "warning" if same_quad else "error", msg + (" (accepted: 'these logs are all from this quad')" if same_quad else ""),
			"pick logs of this quad only, or confirm 'these logs are all from this quad (FC replaced / renamed)' (craft.json \"same_quad\": true or --same-quad)"))
	var common: Variant = null
	var guesses := {}
	var any := false
	for L: Flight in sessions:
		var v := resting_voltage(L)
		if v > 0.0:
			any = true
			var c := cells_candidates(v, pack)
			if common == null:
				common = c
			else:
				common = common.filter(func(x): return x in c)
			var g: int = cells_guess(v, pack)[0]
			if g > 0:
				guesses[g] = true
	if any and (common as Array).is_empty():
		var gs: Array = guesses.keys()
		gs.sort()
		var txt := " vs ".join(gs.map(func(g): return "%dS" % g)) if not gs.is_empty() else "resting voltages do not fit one cell count"
		out.append(issue("pack_differs", "error", "logs with different packs (%s)" % txt, "pick logs flown with packs of one cell count"))
	var fws := {}
	for L: Flight in sessions:
		fws[parse_firmware(L.headers)[3]] = true
	if fws.size() > 1:
		var fl: Array = fws.keys()
		fl.sort()
		out.append(issue("firmware_differs", "warning", "Betaflight version differs between logs (" + " / ".join(fl) + "); fc.json is taken from the newest log", ""))
	return out
