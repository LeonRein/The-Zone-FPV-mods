extends RefCounted
## real_quad logfit: battery, model voltage V = V0 - drain * t - R * I with the sim's current proxy
## I = sum_i u_i * max(u_i * V - w_i / Kv, 0) (in volts, so R is dimensionless). V is the motor fit's
## model voltage: the logged vbat times the fitted scale where logged, else the 2-s window voltages.
## One V0 per session (packs differ), a common drain and R; battery_v0 = the freshest pack.

const Num := preload("num.gd")
const Motor := preload("motor.gd")


static func _proxy(r: Dictionary, i: int, v: float, kv_rad: float) -> float:
	var s := 0.0
	for m in 4:
		var u: float = r.u[m][i]
		s += u * maxf(u * v - r.w[m][i] / kv_rad, 0.0)
	return s


## {source, V0_per_log, V0, drain, R, rms, I_range} or null with fewer than 10 points.
static func fit(rows: Array, logs: Array, P: Array, V: Dictionary, kv_rad: float, decim_s: float = 0.1, cancel: Callable = Callable()) -> Variant:
	var vb_rows: Array = rows.filter(func(r): return not (r.vbat as PackedFloat64Array).is_empty())
	var src := "vbat" if not vb_rows.is_empty() else "motor-windows"
	var sel: Array = vb_rows if not vb_rows.is_empty() else rows
	var ns := sel.size()
	var names: Array = sel.map(func(r): return r.name)
	var t_start := {}
	for L in logs:
		var a: int = L.armed.find(1)
		t_start[L.name] = L.t[a] if a >= 0 else 0.0
	var cols: Array = []
	for k in ns + 2:
		cols.append(PackedFloat64Array())
	var y := PackedFloat64Array()
	var add_row := func(j: int, tt: float, ii: float, vv: float) -> void:
		for k in ns:
			cols[k].append(1.0 if k == j else 0.0)
		cols[ns].append(-tt)
		cols[ns + 1].append(-ii)
		y.append(vv)
	for j in ns:
		var r: Dictionary = sel[j]
		var t: PackedFloat64Array = r.t
		var ts: float = t_start[r.name]
		if src == "vbat":
			var vm := Motor.model_v(r, V)
			var dts := PackedFloat64Array()
			for i in t.size() - 1:
				dts.append(t[i + 1] - t[i])
			var step := maxi(1, int(decim_s / Num.median(dts)))
			var ok: PackedByteArray = r.ok
			for i in range(0, ok.size(), step):
				if Num.stop(cancel, i):
					return null
				if ok[i] != 0:
					add_row.call(j, t[i] - ts, _proxy(r, i, vm[i], kv_rad), vm[i])
		else:
			var win: PackedInt32Array = r.win
			var ok: PackedByteArray = r.ok
			for key in V:
				var parts := str(key).rsplit("|", true, 1)
				if parts.size() != 2 or parts[0] != r.name:
					continue
				var k := parts[1].to_int()
				var v: float = V[key]
				var ps := PackedFloat64Array()
				for i in ok.size():
					if Num.stop(cancel, i):
						return null
					if ok[i] != 0 and win[i] == k:
						ps.append(_proxy(r, i, v, kv_rad))
				if ps.size() < 500:
					continue
				add_row.call(j, k * Motor.WIN_S - ts, Num.mean(ps), v)
	if y.size() < 10:
		return null
	var c := Num.lstsq(cols, y)
	var res := PackedFloat64Array()
	res.resize(y.size())
	for i in y.size():
		var p := 0.0
		for k in ns + 2:
			p += cols[k][i] * c[k]
		res[i] = y[i] - p
	var v0 := {}
	var vmax := -INF
	for k in ns:
		if (cols[k] as PackedFloat64Array).has(1.0):
			v0[names[k]] = Num.pyround(c[k], 2)
			vmax = maxf(vmax, v0[names[k]])
	var negi := PackedFloat64Array()
	for v in cols[ns + 1]:
		negi.append(-v)
	return {"source": src, "V0_per_log": v0, "V0": vmax, "drain": c[ns], "R": c[ns + 1], "rms": Num.std(res),
		"I_range": [Num.pyround(Num.percentile(negi, 5), 2), Num.pyround(Num.percentile(negi, 50), 2), Num.pyround(Num.percentile(negi, 95), 2)]}
