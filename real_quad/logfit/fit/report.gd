extends RefCounted
## real_quad logfit: report.md (English, contract 5.3): grade, warning banners, what to fly next,
## checks, measured values, class defaults, plausibility, sessions, then the per-step detail. The
## collectors (gate, default, meas, plausible) also feed logfit.json and the grade.

const Grade := preload("grade.gd")

var c: Dictionary                    # the craft (form)
var sections: Array = []             # [title, [lines]]
var measured: Array = []             # [what, value, how]
var defaults: Array = []             # [what, value, why]
var gates: Array = []                # {id, name, state, detail}
var plaus: Array = []                # {name, value, band, state, unit}
var issues: Array = []
var banners: Array = []
var sessions: Array = []
var notes: Array = []
var runtime := 0.0
var grade := ""
var grade_caps: Array = []
var fly: Array = []
var cover: Array = []                # what the logs cover (chirp sweeps per axis, drag), lines
var cli: Variant = null
var hover_spread := 0.0
var e1 := 0.0
var t90: Variant = null
var hv: Dictionary = {}


func _init(craft: Dictionary) -> void:
	c = craft


## Python's "{x:.{d}g}".
static func fmt_g(x: float, p: int = 3) -> String:
	if is_nan(x):
		return "nan"
	if is_inf(x):
		return "inf" if x > 0 else "-inf"
	if x == 0.0:
		return "0"
	var e := int(floorf(log(absf(x)) / log(10.0)))
	var r := snappedf(x, pow(10.0, e - p + 1)) if e - p + 1 > -300 else x
	if r != 0.0:
		e = int(floorf(log(absf(r)) / log(10.0) + 1e-12))
	if e < -4 or e >= p:
		var m := r / pow(10.0, e)
		var ms := _strip(String.num(m, maxi(p - 1, 0)))
		return "%se%s%02d" % [ms, "-" if e < 0 else "+", absi(e)]
	return _strip(String.num(r, maxi(p - 1 - e, 0)))


static func _strip(s: String) -> String:
	if "." in s:
		while s.ends_with("0"):
			s = s.substr(0, s.length() - 1)
		if s.ends_with("."):
			s = s.substr(0, s.length() - 1)
	return s


static func f(x: Variant, d: int = 3) -> String:
	if x == null:
		return "–"
	if x is Array or x is PackedFloat64Array:
		var parts: Array = []
		for v in x:
			parts.append(f(v, d))
		return "[" + ", ".join(parts) + "]"
	if x is bool:
		return "yes" if x else "no"
	if x is int:
		return str(x)
	return fmt_g(float(x), d)


static func fx(x: float, d: int) -> String:
	return String.num(x, d) if d > 0 else str(roundi(x))


static func _ms(x: Variant) -> String:
	return "–" if x == null else "%d" % roundi(float(x) * 1e3)


func gate(gid: String, name: String, ok: bool, detail: String) -> void:
	gates.append({"id": gid, "name": name, "state": "PASS" if ok else "WARN", "detail": detail})


func warn(gid: String, name: String, detail: String) -> void:
	gates.append({"id": gid, "name": name, "state": "WARN", "detail": detail})


func default(what: String, value: String, why: String) -> void:
	defaults.append([what, value, why])


func meas(what: String, value: String, how: String) -> void:
	measured.append([what, value, how])


func sec(title: String, lines: Array) -> void:
	sections.append([title, lines])


func plausible(name: String, value: Variant, lo: float, hi: float, unit: String = "") -> void:
	var ok: bool = value != null and lo <= float(value) and float(value) <= hi
	plaus.append({"name": name, "value": null if value == null else float(value), "band": [lo, hi],
		"state": "PASS" if ok else "WARN", "unit": unit})


# ---- steps ----

func segments(logs: Array, segs: Array) -> void:
	var lines := ["| Session | Duration | Usable | Source | Chirps | Auto vs. hand (IoU) |", "|---|---|---|---|---|---|"]
	for k in logs.size():
		var L = logs[k]
		var s: Dictionary = segs[k]
		var use_s := 0.0
		for w in s.use:
			use_s += float(w[1]) - float(w[0])
		lines.append("| %s | %d s | %d s | %s | %d | – |" % [L.name, roundi(L.t[L.n - 1] - L.t[0]), roundi(use_s), s.source, s.chirps.size()])
	sec("Sessions and segments", lines)


func fc(cfg: Dictionary, src: String, diff: Dictionary) -> void:
	var lines := ["From the header of **%s**: %s, PID loop %d Hz (looptime %d µs, pid_process_denom %d), RX %d Hz, PID R/P/Y %s/%s/%s." % [
		src, cfg.firmware, roundi(1e6 / (cfg.looptime_us * cfg.pid_process_denom)), cfg.looptime_us, cfg.pid_process_denom,
		roundi(cfg.rx_rate_hz), str(cfg.pid.roll), str(cfg.pid.pitch), str(cfg.pid.yaw)]]
	if not diff.is_empty():
		lines.append("The tune differs between the logs: %s. The plant does not depend on the tune; fc.json is the newest log's tune." % str(diff))
	sec("Betaflight configuration (fc.json)", lines)


static func _col(P: Array, k: int) -> Array:
	return P.map(func(p): return p[k])


func motor(P0: Array, P: Array, lam: float, r0: float, r1: float, r2: float, V: Dictionary, gain: Dictionary, fr: Dictionary, tp: Variant = null) -> void:
	var vs: Variant = V.get("vbat_scale")
	var gl: Array = []
	for k in gain:
		gl.append("%s Hz %s" % [str(k), f(gain[k], 2)])
	var frl: Array = []
	for k in fr:
		frl.append("%s %s" % [k, ("%.1f" % (fr[k]))])
	var lines := ["Model per motor: dω/dt = a·(u·V − ω/Kv) − b·ω² − c, %.1f ms dead time." % ((tp.delay_s if tp != null else 0.001) * 1e3),
		"- Equation-error fit (R² %s): a %s, b %s, c %s" % [f(r2, 3), f(_col(P0, 0), 4), f(_col(P0, 1), 4), f(_col(P0, 2), 3)],
		"- Dynamics refinement λ = %s (a, b, c scaled together, steady state unchanged): free run %s → %s Hz RMS" % [f(lam, 3), f(r0, 3), f(r1, 3)]]
	if tp != null:
		lines.append("- Torque path (chirps: the free run's ω² against the gyro, 5–50 Hz): a, b, c × %s, dead time 1.0 → %.1f ms; time constant at the chirps %s → %s ms (the eRPM telemetry lags the rotors: the eRPM sets the steady state, the gyro the speed)" % [
			f(tp.lambda, 3), tp.delay_s * 1e3, f(tp.tau0_s * 1e3, 3), f(tp.tau0_s / tp.lambda * 1e3, 3)])
	lines.append_array(["- Final: a %s, b %s, c %s" % [f(_col(P, 0), 4), f(_col(P, 1), 4), f(_col(P, 2), 3)],
		"- Response measured/simulated (roll+pitch differential, eRPM fit) at " + ", ".join(gl),
		"- Voltage: " + ("vbat × " + f(vs, 4) if vs != null else "fitted per 2-s window (no vbat in the log)"),
		"- Free run per session (eRPM fit) [Hz RMS]: " + ", ".join(frl)])
	meas("Motor/ESC a, b, c (per motor)", "see Motor/ESC", "eRPM vs. DShot command" + (", speed and dead time from the gyro" if tp != null else ""))
	var g: Array = []
	for k in gain:
		if 5 <= int(k) and int(k) <= 30 and gain[k] != null and float(gain[k]) != 0.0:
			g.append(gain[k])
	var all_ok := not g.is_empty()
	for v in g:
		if not (0.8 <= v and v <= 1.2):
			all_ok = false
	gate("motor_dynamics", "Motor dynamics 5–30 Hz", all_ok, "measured/sim %s (0.8–1.2)" % f(g, 2))
	if vs != null:
		var ok: bool = 0.8 <= vs and vs <= 1.25
		gate("kv_scale", "Motor KV (vbat scale)", ok, "scale %s (0.8–1.25)" % ("%.3f" % (vs)) + ("" if ok else ": KV probably wrong"))
	sec("Motor/ESC", lines)
	e1 = r1


func motor_free_run(r1: float, hover_hz: float) -> void:
	var thr := maxf(12.0, 0.075 * hover_hz)
	gate("motor_free_run", "Motor model free run", r1 <= thr, "%s Hz RMS (threshold max(12, 7.5 %% of %d Hz) = %s)" % [fmt_g(r1, 3), roundi(hover_hz), ("%.1f" % (thr))])


func hover(h: Dictionary, steady: Dictionary) -> void:
	var src := "ACC (|a| = 1 g, level, quiet)" if h.source == "acc" else "quiet steady flight (no ACC: estimate)"
	var pl: Array = []
	for k in h.per_log:
		pl.append("%s %s Hz (%s s)" % [k, str(h.per_log[k].hz), str(h.per_log[k].s)])
	var lines := ["Hover from %s, %s s: rotor %s Hz, motor command %s, throttle %s." % [src, str(h.n_s), ("%.1f" % (h.hz)), ("%.3f" % (h.u)), ("%.3f" % (h.throttle))],
		"kT = m·g / Σω² = %s N/(rad/s)² (mass %s kg from the form)." % [f(h.kT, 4), str(c.mass_kg)],
		"Per session: " + ", ".join(pl)]
	if not steady.is_empty():
		var sl: Array = []
		for k in steady:
			sl.append("%s %s" % [k, str(steady[k])])
		lines.append("The no-ACC rule on the ACC sessions (Hz), as a check of the fallback: " + ", ".join(sl))
	var hz: Array = []
	for v in h.per_log.values():
		hz.append(float(v.hz))
	var spread := 0.0
	if hz.size() > 1:
		var m := 0.0
		for v in hz:
			m += v
		m /= hz.size()
		spread = (hz.max() - hz.min()) / m
	hover_spread = spread
	meas("Thrust coefficient kT", f(h.kT, 4), "hover %d Hz at %s kg (%s)" % [roundi(h.hz), str(c.mass_kg), h.source])
	gate("hover_detection", "Hover detection", h.source == "acc" and float(h.n_s) >= 3.0, "%s, %s s" % [h.source, str(h.n_s)])
	gate("hover_spread", "Hover rotor speed between sessions", spread <= 0.06,
		"spread %s %% (threshold 6 %%)" % ("%.1f" % (spread * 100.0)) + ("" if spread <= 0.06 else ": weight, props or pack probably differ between logs"))
	sec("Hover and thrust", lines)


func rotation(fit: Dictionary, rp: Dictionary, kQ: float, yaw_rev: bool, chirp: bool, chirp_s: float, band: Array, tq: Variant = null) -> void:
	var lines := ["Instrumental-variable fit on %s (%d s of chirp in the logs), band %s–%s Hz." % [
		"chirps" if chirp else "freestyle (no or too little chirp)", roundi(chirp_s), fmt_g(band[0], 6), fmt_g(band[1], 6)],
		"| Axis | Gain | Damping | R² | Rel. std. error (jackknife over windows) |", "|---|---|---|---|---|"]
	for ax in ["roll", "pitch", "yaw"]:
		var ft: Dictionary = fit[ax]
		var se: float = ft.se if ft.se != null else 0.0
		lines.append("| %s | %s | %s | %s | %s %% |" % [ax, f(ft.coef[0], 4), f(ft.coef[1], 3), f(ft.r2, 2), f(se * 100.0, 2)])
	lines.append("Roll/pitch: rad/s² per 10⁶ (rad/s)² → Ixx %s, Iyy %s kg·m² (with kT and arm)." % [f(rp.Ixx, 3), f(rp.Iyy, 3)])
	lines.append("Yaw: −J_rotor/Izz (×10⁻³) → J_rotor %s kg·m², kQ = J_rotor·b = %s; yaw_motors_reversed = %s (sign from the data)." % [f(rp.J_rotor, 3), f(kQ, 3), "ON" if yaw_rev else "OFF"])
	lines.append("Rate damping: %s (%s)." % [f(rp.rate_damping, 3), "from the chirp fit" if chirp else "set to 0: the freestyle fit is too uncertain"])
	meas("Roll/pitch torque gain (→ Ixx, Iyy)", "%s / %s" % [f(fit.roll.coef[0], 3), f(fit.pitch.coef[0], 3)], "IV, " + ("chirp" if chirp else "freestyle"))
	meas("Yaw: J_rotor/Izz, kQ, direction", f(rp.J_rotor / rp.Izz, 3), "IV, motor torque as regressor")
	var r2 := minf(fit.roll.r2, fit.pitch.r2)
	var sp := maxf(fit.roll.se if fit.roll.se != null else 1.0, fit.pitch.se if fit.pitch.se != null else 1.0)
	gate("rotation_roll_pitch", "Rotation roll/pitch", chirp and r2 >= 0.8 and sp <= 0.1,
		"%s, R² ≥ %s, rel. std. error ≤ %s %% (threshold 10 %%)" % ["chirp" if chirp else "freestyle only", f(r2, 2), f(sp * 100.0, 2)])
	var yse: float = fit.yaw.se if fit.yaw.se != null else 1.0
	gate("rotation_yaw", "Rotation yaw", fit.yaw.r2 >= 0.5 and yse <= 0.25,
		"R² %s, rel. std. error %s %% (threshold 25 %%)" % [f(fit.yaw.r2, 2), f((fit.yaw.se if fit.yaw.se != null else 0.0) * 100.0, 2)])
	if tq != null:
		const Motor := preload("motor.gd")
		var parts: Array = []
		var rng: Array = []
		var ok := true
		for ax in 2:
			var vals: Array = []
			var lo := INF
			var hi := -INF
			for i in Motor.TP_FREQS.size():
				if Motor.TP_FREQS[i] >= 10:
					var v: float = tq[ax][i]
					vals.append("%.2f" % v)
					lo = minf(lo, v)
					hi = maxf(hi, v)
					ok = ok and 0.85 <= v and v <= 1.15
			parts.append(["roll", "pitch"][ax] + " " + ", ".join(vals))
			rng.append("%s %.2f–%.2f" % [["roll", "pitch"][ax], lo, hi])
		lines.append("Torque path measured/sim (gain × free run vs. gyro, 10–50 Hz): " + "; ".join(parts))
		gate("torque_path", "Torque path 10–50 Hz", ok, "measured/sim %s (0.85–1.15)" % ", ".join(rng))
	if not chirp:
		default("Rate damping", "0", "not identifiable without chirp")
	sec("Rotational dynamics", lines)


func bias(b: Variant, arm: float) -> void:
	if b == null:
		return
	var beta: Array = b.beta
	var b3 := beta.map(func(v): return v * 1e3)
	var pl: Array = []
	for k in b.per_log_1e3:
		pl.append("%s %s" % [k, str(b.per_log_1e3[k])])
	var lines := ["From the rotor speed differences at hover (%d samples): β = %s ×10⁻³ m (torque = β · total thrust; roll, pitch, yaw)." % [b.n, f(b3, 3)],
		"Per session (×10⁻³): " + ", ".join(pl)]
	meas("Torque imbalance β", f(b3, 2) + " ×10⁻³ m", "eRPM differences at hover")
	if b.spread != null:
		var big := false
		for k in 3:
			if absf(b.spread[k]) > maxf(absf(beta[k]), 3e-4):
				big = true
		gate("bias_consistent", "Imbalance consistent", not big, "spread between sessions %s ×10⁻³" % f((b.spread as Array).map(func(v): return v * 1e3), 2))
	var mx := maxf(absf(beta[0]), absf(beta[1]))
	gate("bias_size", "Imbalance size", mx <= 0.1 * arm, "|β| %s ×10⁻³ m (≤ 0.1 · arm = %s)" % [("%.2f" % (mx * 1e3)), ("%.1f" % (0.1 * arm * 1e3))])
	sec("Trim / imbalance", lines)


func battery(bat: Dictionary) -> void:
	var lines := ["Source: %s. V = V0 − R·I (I = the sim's current proxy). V0 per session: %s." % [bat.source, str(bat.get("V0_per_log"))],
		"battery_v0 = %s V (freshest pack), battery_r = %s, discharge fitted %s V/min (exported as 0: the game simulates only the sag), RMS %s V." % [
			f(bat.V0, 4), f(bat.R, 3), f(float(bat.drain) * 60.0, 3), f(bat.rms, 2)]]
	if bat.source != "default":
		meas("Battery V0, sag R", "%s V, %s" % [f(bat.V0, 4), f(bat.R, 3)], bat.source)
	var rms: float = bat.rms if bat.rms != null else 9.0
	gate("battery_fit", "Battery fit", bat.source == "vbat" and rms <= 0.4, "%s, RMS %s V" % [bat.source, f(bat.rms, 2)])
	sec("Battery", lines)


func drag(d: Dictionary, rd_ok: bool, body_src: String, top: Array, top_given: Variant, p: Dictionary, top_default: Variant) -> void:
	var lines: Array = []
	if d.get("ok", false):
		lines.append("ACC dead reckoning: %d passes from hover (each up to 5 s, start velocity and tilt fitted too: RMS %s m/s, %s°), %d blocks of 0.25 s, speed p50/p90/p99 %s m/s. Fit R² %s: rotor drag %s 1/s (±%s %%), body scale %s (±%s %%)." % [
			d.n_pass, f(d.v0_rms, 2), f(d.tilt_rms_deg, 2), d.n, str(d.v_pct), f(d.r2, 2), f(d.rotor_drag_per_s, 2),
			f(d.rel_sd[1] * 100.0 if d.rel_sd != null else null, 2), f(d.scale, 2), f(d.rel_sd[0] * 100.0 if d.rel_sd != null else null, 2)])
	else:
		lines.append("ACC fit not possible (" + ("no ACC in the logs" if int(d.get("n_pass", 0)) == 0 else "too few passes from hover") + ").")
	var tg := fmt_g(float(top_given), 12) if top_given != null else "None"
	var rd_txt := "rotor drag %s (%s)" % ["from the ACC fit" if rd_ok else "default 0.40 1/s", f(p.rotor_drag, 3)]
	var body_txt: String = {"acc": "body drag from the ACC fit",
		"top_speed": "body drag calibrated to the given top speed %s km/h (the ACC fit does not determine it: too few fast passes)" % tg,
		"default": "body drag class default (reference × (D/5.1\")²)"}[body_src]
	lines.append("Used: %s; %s: cd_area %s m²." % [rd_txt, body_txt, f(p.cd_area, 3)])
	lines.append("Top speed of the model (full throttle, level, fresh pack with sag): **%d km/h** at %d° tilt, %d Hz rotor" % [roundi(top[0]), roundi(top[1]), roundi(top[2])]
		+ ("; given %s km/h" % tg if top_given != null else "; no top speed in the form")
		+ ("; with the default body drag it would be %d km/h." % roundi(top_default) if top_default != null else "."))
	if rd_ok:
		meas("Rotor drag (linear)", "%s 1/s" % f(d.rotor_drag_per_s, 2), "ACC dead reckoning")
	else:
		default("Rotor drag (linear)", "0.40 1/s", "ACC fit missing/uncertain; literature 0.3–0.5")
	if body_src == "acc":
		meas("Body drag", "cd_area " + f(p.cd_area, 3), "ACC dead reckoning")
	elif body_src == "top_speed":
		meas("Body drag", "cd_area " + f(p.cd_area, 3), "given top speed %s km/h" % tg)
	else:
		default("Body drag", "cd_area " + f(p.cd_area, 3), "reference × (D/5.1\")²")
	var rs: Variant = d.get("rel_sd")
	gate("drag_rotor", "ACC rotor drag", rd_ok, "R² %s, uncertainty %s (thresholds 0.5 / 0.15)" % [f(d.get("r2"), 2), f(rs[1] if rs != null else null, 2)])
	gate("top_speed", "Top speed backed", body_src == "acc" or body_src == "top_speed",
		{"acc": "body drag from ACC", "top_speed": "calibrated to the given %s km/h" % tg,
		"default": "neither a given top speed nor an ACC fit: %d km/h is an estimate" % roundi(top[0])}[body_src])
	sec("Drag and top speed", lines)


func propwash(lg: Dictionary, p: Dictionary, dflt: Dictionary) -> void:
	var own: bool = dflt.dirty_air_from == c.get("class")
	var lines := ["Log metric (15–78 Hz roll/pitch tracking error, deg/s, median): calm %s, after throttle chops %s, flips after chops %s; 99th percentile of all usable windows outside chirps %s (windows %s). The targets of the prop wash calibration (below)." % [
		str(lg.calm), str(lg.chops), str(lg.flips), str(lg.worst_p99), str(lg.n)],
		"Class values before the calibration: dirty_air gust %s from the %s class%s, calm-air turbulence %s m/s (reference scaled with the induced velocity, factor %s)." % [
			str(p.dirty_air_gust), dflt.dirty_air_from, "" if own else " (uncalibrated for this class)", str(p.turb_base_ms), f(dflt.v_induced_ratio, 3)],
		"Wake/VRS amplitudes of the Python plant (unused in the game): pw_turb_ms %s, wake_r0 %s m." % [str(p.pw_turb_ms), str(p.wake_r0)]]
	sec("Propwash / turbulence", lines)


## The prop wash calibration (calibrate.gd's record): its own section, the values among the measured ones
## or the class defaults
func propwash_calibration(rec: Dictionary) -> void:
	const Cal := preload("calibrate.gd")
	var sc: Dictionary = rec.scenario
	var tg: Dictionary = rec.targets
	var lines := ["- " + Cal.summary(rec)]
	lines.append("- dirty_air %s, wash and wake 1 (its own figures); scenarios flown in its air without the game (no map, 60 m up), mean of %d seeds, %d runs, %s s." % [
		rec.dirty_air_version if str(rec.dirty_air_version) != "" else "not installed", int(rec.seeds), int(rec.runs), str(rec.runtime_s)])
	lines.append("- Drop and catch: 0.9 s at zero throttle from a hover, caught at 45 %%; its worst 0.25-s window against %s × the logs' 99th percentile (freestyle5's scale: 14.5 deg/s at 11.3)." % fx(Cal.DROP_PER_P99, 3))
	lines.append("- Calm: 2 s of hover; its median 0.25-s window against the logs' calm median.")
	if "dropcatch" in rec.calibrated:
		meas("dirty_air gust", str(rec.gust), "drop and catch %s deg/s (target %s)" % [str(sc.dropcatch), str(tg.dropcatch)])
	else:
		default("dirty_air gust", str(rec.gust), "class value: not calibrated (%s)" % rec.why)
	if "calm" in rec.calibrated:
		meas("Calm-air turbulence", "%s m/s" % str(rec.turb_base_ms), "calm hover %s deg/s (target %s)" % [str(sc.calm), str(tg.calm)])
	else:
		default("Calm-air turbulence", "%s m/s" % str(rec.turb_base_ms), "class value: not calibrated (%s)" % rec.why)
	default("dirty_air wash/wake", "1/1", "dirty_air's own figures")
	sec("Prop wash calibration", lines)


func validation(h: Dictionary, s: Variant, ev: Array) -> void:
	var lines := ["Hover: motor command sim %s / log %s, rotor %d / %d Hz (model voltage %s V)." % [
		("%.3f" % (h.u_sim)), ("%.3f" % (h.u_log)), roundi(h.hz_sim), roundi(h.hz_log), f(h.V, 4)]]
	gate("hover_throttle", "Hover throttle sim vs. log", absf(h.u_sim - h.u_log) <= 0.02, "%s vs. %s (±0.02)" % [("%.3f" % (h.u_sim)), ("%.3f" % (h.u_log))])
	if s != null:
		lines.append("Roll steps (%d, stick replay through the Betaflight port + model): t90 sim %s ms / log %s ms, lag behind the setpoint (at 50 %%) sim %s ms / log %s ms, gyro deviation sim−log %s × log RMS (median)." % [
			s.n, str(s.sim_ms), str(s.log_ms), str(s.lag_sim_ms), str(s.lag_log_ms), ("%.2f" % (s.track))])
		var el: Array = []
		for e in ev:
			el.append("%s %s s %s %s/%s" % [e.log, str(e.t), str(e.plateau), _ms(e.t90_log), _ms(e.t90_sim)])
		lines.append("Events (session, time, plateau deg/s, t90 log/sim ms): " + "; ".join(el))
		gate("roll_t90", "Roll t90 sim vs. log", absf(s.sim_ms - s.log_ms) <= maxf(0.2 * s.log_ms, 10.0), "%s vs. %s ms (±20 %%)" % [str(s.sim_ms), str(s.log_ms)])
		gate("roll_lag", "Roll lag sim vs. log", absf(s.lag_sim_ms - s.lag_log_ms) <= maxf(0.25 * absf(s.lag_log_ms), 2.0),
			"%s vs. %s ms (±25 %%, at least 2 ms)" % [str(s.lag_sim_ms), str(s.lag_log_ms)])
		gate("tracking", "Closed-loop tracking", s.track <= 0.35, "%s (threshold 0.35)" % ("%.2f" % (s.track)))
	else:
		warn("roll_t90", "Roll t90 sim vs. log", "no fast roll steps (≥ 300 deg/s) found: no closed-loop check")
	sec("Validation", lines)
	t90 = s
	hv = h


func render() -> String:
	var n_warn := 0
	for g in gates:
		if g.state == "WARN":
			n_warn += 1
	var out: Array = ["# %s (%s)" % [c.get("name", c.id), c.id], ""]
	if grade != "":
		out += ["## Grade %s: %s" % [grade, Grade.MEANING[grade]], ""]
		if grade_caps.is_empty():
			out.append("- all coverage items and critical checks met")
		for cap in grade_caps:
			out.append("- %s: %s" % [cap[0], cap[1]])
		out.append("")
	if not cover.is_empty():
		out.append("## What the logs cover")
		for x in cover:
			out.append("- " + x)
		out.append("")
	for b in banners:
		out += ["> **%s**" % b, ""]
	if not fly.is_empty():
		out.append("## What to fly next")
		for x in fly:
			out.append("- [ ] " + x)
		out.append("")
		if cli != null:
			out += ["CLI (paste into the Betaflight CLI; take a `diff all` first):", "```", cli, "```", ""]
	var prop: Dictionary = c.prop
	out += ["Made by real_quad logfit from %d sessions in %d s. %d PASS, %d WARN." % [sessions.size(), roundi(runtime), gates.size() - n_warn, n_warn], "",
		"Form: %s kg, %sS %s, %d KV, prop %s\" × %s\" × %s, class %s, top speed %s km/h." % [str(c.mass_kg), str(c.get("_cells", "?")), c.get("_pack", "lipo"),
			roundi(float(c.get("_kv", 0))), str(prop.get("diameter_in")), str(prop.get("pitch_in")) if prop.get("pitch_in") else "?", str(prop.get("blades")),
			str(c.get("class")), fmt_g(float(c.get("top_speed_kmh")), 12) if c.get("top_speed_kmh") else "unknown"], "",
		"## Checks", "| Check | Result | Detail |", "|---|---|---|"]
	for g in gates:
		out.append("| %s | %s | %s |" % [g.name, g.state, g.detail])
	out += ["", "## Measured", "| Quantity | Value | Source |", "|---|---|---|"]
	for m in measured:
		out.append("| %s | %s | %s |" % m)
	out += ["", "## Class defaults and assumptions", "| Quantity | Value | Why |", "|---|---|---|"]
	for d in defaults:
		out.append("| %s | %s | %s |" % d)
	if not plaus.is_empty():
		out += ["", "## Plausibility vs. the %s class band" % str(c.get("class")), "| Quantity | Value | Band | Result |", "|---|---|---|---|"]
		for p in plaus:
			out.append("| %s | %s %s | %s–%s | %s |" % [p.name, f(p.value, 3), p.unit, f(p.band[0], 3), f(p.band[1], 3), p.state])
	out += ["", "## Sessions", "| Session | Firmware | Armed | Used | ACC | vbat | unfilt | Reason |", "|---|---|---|---|---|---|---|---|"]
	for s in sessions:
		out.append("| %s | %s | %d s | %s | %s | %s | %s | %s |" % [s.id, s.firmware, roundi(s.armed_s), "yes" if s.used else "no", f(s.acc), f(s.vbat), f(s.unfilt),
			s.reason if s.reason != null else ""])
	if not issues.is_empty():
		out += ["", "## Warnings and notes"]
		for i in issues:
			out.append("- %s: %s" % [i.level, i.message] + (" (fix: %s)" % i.fix if str(i.get("fix", "")) != "" else ""))
	for s in sections:
		out += ["", "## " + s[0]]
		out += s[1]
	if not notes.is_empty():
		out += ["", "## Notes"]
		for n in notes:
			out.append("- " + n)
	return "\n".join(out) + "\n"
