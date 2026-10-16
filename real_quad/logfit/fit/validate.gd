extends RefCounted
## real_quad logfit: checks of the finished model against the logs, in the code that flies it.
##   hover()      steady state of the motor model: command and rotor speed that carry the weight
##   roll_t90()   step responses: the logged sticks replayed through flightmodel/bf_controller.gd (the
##                Betaflight port) and bf_quad.gd (the plant: clean air, no turbulence, the motor model
##                voltage of the log, rotation only); t90 = time from the start of a fast roll stick
##                move until the roll rate reaches 90 % of the plateau; lag = how much later the roll
##                rate reaches half the plateau than the setpoint. Sim vs log, same events.

const Num := preload("num.gd")
const Flight := preload("flight.gd")
const Motor := preload("motor.gd")
const Controller := preload("../../flightmodel/bf_controller.gd")
const Quad := preload("../../flightmodel/bf_quad.gd")

const G := 9.81
const R2D := 180.0 / PI
const PLANT_HZ := 8000.0


static func hover(p: Dictionary, P: Array, h: Dictionary) -> Dictionary:
	var w := sqrt(float(p.mass) * G / (4.0 * float(p.kT)))
	var V: float = h.V if h.get("V") != null and float(h.V) != 0.0 else float(p.battery_v0)
	var u := Motor.command_for_speed(P, w, V, float(p.kv_rad))
	return {"u_sim": (((u[0] + u[1]) + u[2]) + u[3]) / 4.0, "u_log": h.u, "hz_sim": w / TAU, "hz_log": h.hz, "V": V}


## [[j0, sign, plateau], ...] of a session, the largest first. A step needs 0.7 s of usable flight on both
## sides and its replay window (j0 +- 0.6 s) inside the log: one in the first 0.75 s is skipped.
static func roll_events(L: Flight, use: PackedByteArray, max_events: int = 6, min_plateau: float = 300.0) -> Array:
	var fs := L.fs
	var sp: PackedFloat64Array = L.setpoint[0]
	var n15 := int(0.15 * fs)
	var n3 := int(0.3 * fs)
	var n6 := int(0.6 * fs)
	var n7 := int(0.7 * fs)
	var bad := PackedInt32Array()
	bad.resize(L.n + 1)
	bad[0] = 0
	for i in L.n:
		bad[i + 1] = bad[i] + (1 if use[i] == 0 else 0)
	var ev: Array = []
	var i := n15
	while i < L.n - n3 - 1:
		if i >= n7 and absf(sp[i]) > min_plateau:
			var mn := INF
			for k in range(i - n15, i):
				mn = minf(mn, absf(sp[k]))
			# use[i - 0.7 s : i + 0.7 s] (Python slice semantics at the end)
			var a := i - n7
			var b := mini(i + n7, L.n)
			var all_use := a >= b or bad[b] - bad[a] == 0
			if mn < 100.0 and all_use:
				var sgn := signf(sp[i])
				var plateau := -INF
				for k in range(i, mini(i + n3, L.n)):
					plateau = maxf(plateau, sgn * sp[k])
				var j0 := i - n15
				for k in range(i - n15, i + 1):
					if sgn * sp[k] > 0.1 * plateau:
						j0 = k
						break
				if j0 - n6 < 0 or j0 + n6 > L.n:
					i += 1
					continue
				ev.append([j0, sgn, plateau])
				i += int(0.8 * fs)
				continue
		i += 1
	var order: Array = range(ev.size())
	order.sort_custom(func(x: int, y: int) -> bool: return ev[x][2] > ev[y][2] or (ev[x][2] == ev[y][2] and x < y))
	var out: Array = []
	for k in mini(max_events, order.size()):
		out.append(ev[order[k]])
	return out


static func _t90(t: PackedFloat64Array, g: PackedFloat64Array, j0: int, sgn: float, plateau: float, fs: float, frac: float = 0.9) -> Variant:
	var e := mini(j0 + int(0.5 * fs), g.size())
	for k in range(j0, e):
		if sgn * g[k] >= frac * plateau:
			return t[k] - t[j0]
	return null


static func _rx_from_rc(L: Flight, i: int, mincheck: float) -> PackedFloat64Array:
	var thr: float = mincheck + (L.rc[3][i] - 1000.0) * (2000.0 - mincheck) / 1000.0
	return PackedFloat64Array([1500.0 + L.rc[0][i], 1500.0 + L.rc[1][i], 1500.0 - L.rc[2][i], thr])


## Replays the logged sticks over samples a..b-1: {gyro: [roll, pitch, yaw] arrays, sp: same}.
static func replay(L: Flight, a: int, b: int, p: Dictionary, cfg: Dictionary, vs: PackedFloat64Array) -> Dictionary:
	var fc = Controller.new(cfg, true)
	fc.arm(true)
	fc.throttle_raised = true
	fc.airmode_active = true
	fc.iterm_reset = false
	if not L.pid_i.is_empty():
		for k in 3:
			fc.I[k] = float(L.pid_i[k][a])
	var n_loop := maxi(1, roundi(fc.pid_freq / L.fs))
	var n_sub := maxi(1, roundi(PLANT_HZ / fc.pid_freq))
	var dtp: float = fc.dt / n_sub
	var rx_us := 1e6 / float(cfg.get("rx_rate_hz", 250.0))
	var q = Quad.new(p)
	q.turbulence_enabled = false
	q.src_rp = 0.0
	q.src_ra = 0.0
	q.omega = Vector3(L.gyro[0][a], L.gyro[1][a], L.gyro[2][a]) / R2D
	for m in 4:
		q.w[m] = TAU * maxf(L.motor_hz[m][a], 0.0)
	var delay_n := maxi(1, roundi(float(p.motor_delay_s) / fc.dt))
	var ubuf: Array = []
	var u0 := PackedFloat64Array()
	for m in 4:
		u0.append(clampf((L.motor_raw[m][a] - 48.0) / 1999.0, 0.0, 1.0))
	for k in delay_n:
		ubuf.append(u0)
	var basis := Basis.IDENTITY
	var zero := Vector3()
	var gyro := PackedFloat64Array([0, 0, 0])
	var tel := PackedFloat64Array([0, 0, 0, 0])
	var rg: Array = [PackedFloat64Array(), PackedFloat64Array(), PackedFloat64Array()]
	var rs: Array = [PackedFloat64Array(), PackedFloat64Array(), PackedFloat64Array()]
	var mc := float(cfg.get("mincheck", 1050))
	var last_rc := PackedFloat64Array()
	var last_t := -1e18
	for i in range(a, b):
		var tu := roundi(L.t[i] * 1e6)
		var cur := PackedFloat64Array([L.rc[0][i], L.rc[1][i], L.rc[2][i]])
		if last_rc.is_empty() or cur != last_rc or tu - last_t >= rx_us:
			fc.rx_frame(_rx_from_rc(L, i, mc), tu)
			last_rc = cur
			last_t = tu
		q.src_v = vs[i]
		q.v_batt = vs[i]
		for k in n_loop:
			fc.t_us = tu + int(k * fc.dt * 1e6)
			var om: Vector3 = q.omega * R2D
			gyro[0] = om.x
			gyro[1] = om.y
			gyro[2] = om.z
			for m in 4:
				tel[m] = q.w[m] / TAU
			var mo: PackedFloat64Array = fc.step(gyro, tel)
			var u := PackedFloat64Array([0, 0, 0, 0])
			for m in 4:
				u[m] = clampf((mo[m] - 48.0) / 1999.0, 0.0, 1.0)
			ubuf.append(u)
			var uc: PackedFloat64Array = ubuf.pop_front()
			for _s in n_sub:
				q.step(uc, dtp, basis, zero, zero, true)
		var o: Vector3 = q.omega * R2D
		rg[0].append(o.x)
		rg[1].append(o.y)
		rg[2].append(o.z)
		for k in 3:
			rs[k].append(fc.setpoint_rate[k])
	return {"gyro": rg, "sp": rs}


## Up to max(2, max_total / sessions) candidate steps per session, then the max_total largest plateaus
## over all sessions (ties: session content hash, then time) - independent of the input order.
static func roll_t90(logs: Array, uses: Array, rows: Array, V: Dictionary, p: Dictionary, cfg: Dictionary, max_total: int = 12, cancel: Callable = Callable()) -> Array:
	var per_log := maxi(2, max_total / maxi(1, logs.size()))
	var cand: Array = []
	for k in logs.size():
		var L: Flight = logs[k]
		var key: String = L.info.get("hash", L.name)
		for e in roll_events(L, uses[k], per_log):
			cand.append([-float(e[2]), key, int(e[0]), k, e[1], e[2]])
	cand.sort_custom(func(x, y) -> bool:
		if x[0] != y[0]:
			return x[0] < y[0]
		if x[1] != y[1]:
			return x[1] < y[1]
		return x[2] < y[2])
	var out: Array = []
	for c in cand.slice(0, max_total):
		if cancel.is_valid() and cancel.call():
			return out
		var j0: int = c[2]
		var L: Flight = logs[c[3]]
		var sgn: float = c[4]
		var plateau: float = c[5]
		var vs := Motor.model_v(rows[c[3]], V)
		var a := j0 - int(0.6 * L.fs)
		var b := j0 + int(0.6 * L.fs)
		var rec := replay(L, a, b, p, cfg, vs)
		var ts := L.t.slice(a, b)
		var k0 := j0 - a
		var g0: PackedFloat64Array = L.gyro[0]
		var tl: Variant = _t90(L.t, g0, j0, sgn, plateau, L.fs)
		var tsim: Variant = _t90(ts, rec.gyro[0], k0, sgn, plateau, L.fs)
		var gl: Variant = _t90(L.t, g0, j0, sgn, plateau, L.fs, 0.5)
		var gs: Variant = _t90(ts, rec.gyro[0], k0, sgn, plateau, L.fs, 0.5)
		var spl: Variant = _t90(L.t, L.setpoint[0], j0, sgn, plateau, L.fs, 0.5)
		var sps: Variant = _t90(ts, rec.sp[0], k0, sgn, plateau, L.fs, 0.5)
		var s0 := maxi(k0 - int(0.1 * L.fs), 0)
		var s1 := mini(k0 + int(0.5 * L.fs), b - a)
		var e2 := PackedFloat64Array()
		var l2 := PackedFloat64Array()
		for i in range(s0, s1):
			for ax in 2:
				var lg: float = L.gyro[ax][a + i]
				var d: float = rec.gyro[ax][i] - lg
				e2.append(d * d)
				l2.append(lg * lg)
		out.append({"log": L.name, "t": Num.pyround(L.t[j0], 2), "plateau": int(Num.pyround(plateau)), "t90_log": tl, "t90_sim": tsim,
			"lag_log": null if gl == null or spl == null else gl - spl, "lag_sim": null if gs == null or sps == null else gs - sps,
			"track": sqrt(Num.mean(e2)) / maxf(sqrt(Num.mean(l2)), 1.0)})
	return out


static func summarize_t90(ev: Array) -> Variant:
	var ok: Array = ev.filter(func(e): return e.t90_log != null and e.t90_sim != null)
	if ok.is_empty():
		return null
	var med := func(key: String, src: Array) -> float:
		var v := PackedFloat64Array()
		for e in src:
			v.append(e[key])
		return Num.median(v)
	var tl: float = med.call("t90_log", ok)
	var tsim: float = med.call("t90_sim", ok)
	var lag: Array = ok.filter(func(e): return e.lag_log != null and e.lag_sim != null)
	var ll: float = med.call("lag_log", lag) if not lag.is_empty() else NAN
	var ls: float = med.call("lag_sim", lag) if not lag.is_empty() else NAN
	return {"n": ok.size(), "log_ms": Num.pyround(tl * 1e3, 1), "sim_ms": Num.pyround(tsim * 1e3, 1),
		"lag_log_ms": Num.pyround(ll * 1e3, 1), "lag_sim_ms": Num.pyround(ls * 1e3, 1),
		"track": Num.pyround(med.call("track", ok), 2)}
