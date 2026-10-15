extends RefCounted
## real_quad logfit: coverage metrics, the A-D grade and the "what to fly next" list (contract 3.5,
## 5.3). The grade is the worst cap whose condition holds, A if none.

const Num := preload("num.gd")
const Classes := preload("classes.gd")

const CRITICAL := ["motor_free_run", "motor_dynamics", "hover_detection", "rotation_roll_pitch", "hover_throttle", "roll_t90"]
const MEANING := {"A": "flies like your quad", "B": "close; small differences", "C": "rough: right size and power, response approximate",
	"D": "a sketch: mostly class defaults"}
const ORDER := "ABCD"


static func punches(logs: Array, uses: Array, thr: float = 0.8, min_s: float = 0.3) -> int:
	var n := 0
	for li in logs.size():
		var L = logs[li]
		var u: PackedByteArray = uses[li]
		var m := PackedByteArray()
		m.resize(L.n)
		for i in L.n:
			m[i] = 1 if u[i] != 0 and L.throttle[i] >= thr else 0
		var r := Num.runs(m)
		for j in range(0, r.size(), 2):
			if (r[j + 1] - r[j]) / L.fs >= min_s:
				n += 1
	return n


static func coverage(logs: Array, uses: Array, segs: Array, chirp_s: float, hover: Variant, dfit: Dictionary, events: Array, acc_ok: bool, vbat_ok: bool) -> Dictionary:
	var sweeps := {"roll": 0, "pitch": 0, "yaw": 0}
	for li in logs.size():
		var sweep_s := float(logs[li].hint("chirp_time_seconds", 20))
		if sweep_s == 0.0:
			sweep_s = 20.0
		for c in segs[li].chirps:
			if c[1] >= 0.8 * sweep_s:
				sweeps[["roll", "pitch", "yaw"][c[2]]] += 1
	var unfilt := true
	for L in logs:
		if not L.info.flags.get("unfilt", false):
			unfilt = false
	return {"chirp_s": Num.pyround(chirp_s, 1), "sweeps": sweeps, "acc_ok": acc_ok, "vbat_ok": vbat_ok,
		"hover_s": float(hover.n_s) if hover != null else 0.0, "hover_source": hover.source if hover != null else null,
		"punches": punches(logs, uses), "drag_passes": int(dfit.get("n_pass", 0)), "roll_events": events.size(),
		"unfilt": unfilt}


## [letter, [[cap, reason], ...]]; gates: [{id, name, state, ...}].
static func grade(cov: Dictionary, gates: Array) -> Array:
	var caps: Array = []
	var sw: Dictionary = cov.sweeps
	var chirp: bool = cov.chirp_s >= 30.0
	var sweeps_a: bool = sw.roll >= 2 and sw.pitch >= 2 and sw.yaw >= 1
	var crit: Array = []
	for g in gates:
		if g.get("id") in CRITICAL and g.state != "PASS":
			crit.append(g.name)
	var missing: Array = []
	for pair in [["chirp", chirp], ["ACC", cov.acc_ok], ["battery voltage", cov.vbat_ok]]:
		if not pair[1]:
			missing.append(pair[0])
	if missing.size() >= 2:
		caps.append(["D", "%s missing" % " and ".join(missing)])
	if crit.size() >= 2:
		caps.append(["D", "%d critical checks WARN (%s)" % [crit.size(), ", ".join(crit)]])
	if not chirp:
		caps.append(["C", "only %.0f s of chirp sweeps (30 s needed): rotation from freestyle, rate damping 0" % cov.chirp_s])
	if not cov.acc_ok:
		caps.append(["C", "no usable accelerometer: hover from the steady rule, drag from defaults"])
	if crit.size() == 1:
		caps.append(["C", "critical check WARN: %s" % crit[0]])
	if chirp and not sweeps_a:
		if sw.roll >= 1 and sw.pitch >= 1:
			caps.append(["B", "chirp sweeps roll/pitch/yaw %d/%d/%d (A needs 2/2/1)" % [sw.roll, sw.pitch, sw.yaw]])
		else:
			caps.append(["C", "chirp sweeps roll/pitch/yaw %d/%d/%d: no full sweep on roll or pitch" % [sw.roll, sw.pitch, sw.yaw]])
	if not cov.vbat_ok:
		caps.append(["B", "no usable battery voltage: motor voltage estimated per 2-s window"])
	if cov.punches < 2:
		caps.append(["B", "%d punch-outs (2 needed): full-throttle motor response extrapolated" % cov.punches])
	if cov.roll_events == 0:
		caps.append(["B", "no fast roll step: no closed-loop check"])
	elif cov.roll_events < 3:
		caps.append(["B", "%d fast roll steps for the closed-loop check (3 needed)" % cov.roll_events])
	if cov.hover_s < 10.0:
		caps.append(["B", "%.0f s of hover (10 s needed): kT less certain" % cov.hover_s])
	if cov.drag_passes < 3:
		caps.append(["B", "%d drag passes (3 needed): rotor drag default" % cov.drag_passes])
	var letter := "A"
	for c in caps:
		if ORDER.find(c[0]) > ORDER.find(letter):
			letter = c[0]
	return [letter, caps]


## [[lines], cli_needed]
static func fly_next(cov: Dictionary, cls: String, pid_khz: float, log_rate_ok: bool = true) -> Array:
	var items: Array = []
	var cli := false
	var sw: Dictionary = cov.sweeps
	var ch: Dictionary = Classes.CLASSES[cls].chirp
	if cov.chirp_s < 30.0 and sw.roll == 0 and sw.pitch == 0 and sw.yaw == 0:
		items.append("Chirp sweeps: CHIRP mode on a switch, 2-3 rounds of roll, pitch, yaw (%d s each, at least 30 s in total)" % ch.time_s)
		cli = true
	else:
		for pair in [["roll", 2], ["pitch", 2], ["yaw", 1]]:
			var need: int = pair[1] - sw[pair[0]]
			if need > 0:
				items.append("%d more chirp round%s on %s (full %d s sweeps)" % [need, "s" if need > 1 else "", pair[0], ch.time_s])
	if not cov.acc_ok:
		items.append("Turn on ACC logging: `set blackbox_disable_acc = OFF` (ACC on and calibrated)")
		cli = true
	if not cov.vbat_ok:
		items.append("Turn on battery voltage logging: `set blackbox_disable_bat = OFF` (voltage sensor calibrated)")
		cli = true
	if not cov.unfilt:
		items.append("Turn on unfiltered gyro logging: `set blackbox_disable_gyrounfilt = OFF`")
		cli = true
	if not log_rate_ok:
		items.append("Log at 1 kHz: `set blackbox_sample_rate = 1/%d`" % maxi(1, roundi(pid_khz)))
		cli = true
	if cov.hover_s < 10.0:
		items.append("10 s hands-off hover in ANGLE")
	if cov.punches < 2:
		var k: int = 2 - cov.punches
		items.append("%d vertical punch-out%s (full throttle for at least 0.3 s)" % [k, "s" if k > 1 else ""])
	if cov.drag_passes < 3:
		items.append("%d more straight fast passes, each started from a steady hover" % (3 - cov.drag_passes))
	if cov.roll_events < 3:
		items.append("1-2 min of normal flying with fast roll flips (full roll stick and back)")
	return [items, cli]
