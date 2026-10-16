extends RefCounted
## real_quad logfit: motor/ESC/prop dynamics from the DShot command to the rotor speed (eRPM).
##
## Model per motor (rotor speed w [rad/s], command u 0..1, voltage V), command delayed by D (1 ms for the eRPM fit):
##     dw/dt = a * (u * V - w / Kv) - b * w^2 - c
## V is the "model voltage": the logged vbat (low-passed) times one global scale where logged (absorbs
## ESC losses and a rated Kv that differs from the real one), else one value per 2-s window.
## a, b, c, the scale and the window voltages come from alternating least squares (fit(), the static map); then a, b,
## c are scaled together by lambda (refine_dynamics(), golden section on the free-run error against eRPM); with chirps
## the speed and D are then set by the gyro (torque_refine(): the eRPM telemetry lags and smooths the rotor speed,
## so the eRPM alone makes the motors too slow).
##
## A "row" is one session prepared by prep(): {name, t, u[4], w[4], dw[4], ok, win, vbat}. Voltages V:
## {"vbat_scale": s, "<session>|<window>": volts}. P: 4 x [a, b, c] (Array of PackedFloat64Array).

const Num := preload("num.gd")
const Flight := preload("flight.gd")

const WIN_S := 2.0
const DELAY_S := 0.001
const LAMBDA_LO := 0.7
const LAMBDA_HI := 3.5
const SUBSET_BLOCK := 5000
const VBAT_SCALE := "vbat_scale"
const TP_FREQS := [5, 10, 15, 20, 25, 30, 35, 40, 45, 50]   # torque path band [Hz]
const TP_LAMBDA_LO := 0.5      # speed factor range of the torque path refinement
const TP_LAMBDA_HI := 2.0
const DELAY_MAX_S := 0.005


## cancel (optional): polled between the motors and every 10 000 samples of the long loops; when it answers
## true the answer is {} (the caller stops anyway).
static func prep(L: Flight, use: PackedByteArray, delay_s: float = DELAY_S, cancel: Callable = Callable()) -> Dictionary:
	var fs := L.fs
	var cc := cancel.is_valid()
	var n := L.n
	var n_sg := roundi(0.009 * fs) | 1
	var u0 := L.u()
	var w: Array = []
	var dw: Array = []
	var u: Array = []
	var k2 := 2.0 * PI
	for m in 4:
		if cc and cancel.call():
			return {}
		var hz: PackedFloat64Array = L.motor_hz[m]
		var wr := PackedFloat64Array()
		wr.resize(n)
		for i in n:
			var v := hz[i]
			wr[i] = k2 * (0.0 if is_nan(v) else v)
		var dd := Num.savgol(wr, n_sg, 1)
		for i in n:
			dd[i] = dd[i] * fs
		dw.append(dd)
		w.append(Num.savgol(wr, n_sg, 0))
		u.append(delayed(u0[m], delay_s, fs))
	var ok := PackedByteArray()
	ok.resize(n)
	for i in n:
		if cc and i % 10000 == 0 and cancel.call():
			return {}
		var good := use[i] != 0
		if good:
			for m in 4:
				if not (L.motor_raw[m][i] > 50.0 and L.motor_hz[m][i] > 30.0):
					good = false
					break
		ok[i] = 1 if good else 0
	# no derivative across gaps (numpy.roll wraps around)
	for i in range(1, n):
		if L.t[i] - L.t[i - 1] > 3.0 / fs:
			for k in range(-n_sg, n_sg + 1):
				ok[posmod(i + k, n)] = 0
	var win := PackedInt32Array()
	win.resize(n)
	for i in n:
		win[i] = int(Num.floor_div(L.t[i], WIN_S))
	var vb := PackedFloat64Array()
	if L.has_vbat():
		vb = Num.sosfiltfilt(Num.butter(2, 2.0, "lowpass", fs), L.vbat)
	return {"name": L.name, "t": L.t, "u": u, "w": w, "dw": dw, "ok": ok, "win": win, "vbat": vb}


## The commands delayed by delay_s (linear between the samples; the first sample held).
static func delayed(x: PackedFloat64Array, delay_s: float, fs: float) -> PackedFloat64Array:
	var d := delay_s * fs
	var n := floori(d + 1e-9)
	var f := d - n
	var o := PackedFloat64Array()
	o.resize(x.size())
	for i in x.size():
		var v := x[maxi(i - n, 0)]
		o[i] = v if f < 1e-9 else (1.0 - f) * v + f * x[maxi(i - n - 1, 0)]
	return o


static func wkey(name: String, k: int) -> String:
	return "%s|%d" % [name, k]


## The model voltage of a row per sample.
static func model_v(row: Dictionary, V: Dictionary) -> PackedFloat64Array:
	var vb: PackedFloat64Array = row.vbat
	var n: int = (row.t as PackedFloat64Array).size()
	var o := PackedFloat64Array()
	o.resize(n)
	if not vb.is_empty():
		var s: float = V.get(VBAT_SCALE, 1.0)
		for i in n:
			o[i] = vb[i] * s
		return o
	var win: PackedInt32Array = row.win
	var nm: String = row.name
	var last := -2147483648
	var lv := 24.0
	for i in n:
		if win[i] != last:
			last = win[i]
			lv = V.get(wkey(nm, last), 24.0)
		o[i] = lv
	return o


static func _unique(win: PackedInt32Array) -> PackedInt32Array:
	var d := {}
	for k in win:
		d[k] = true
	var o := PackedInt32Array(d.keys())
	o.sort()
	return o


## Alternating least squares: [P, V, r2].
static func fit(rows: Array, kv_rad: float, v_guess: float, iters: int = 10, cancel: Callable = Callable()) -> Array:
	var V := {}
	var any_vb := false
	for r in rows:
		if (r.vbat as PackedFloat64Array).is_empty():
			for k in _unique(r.win):
				V[wkey(r.name, k)] = v_guess
		else:
			any_vb = true
	if any_vb:
		V[VBAT_SCALE] = 1.0
	var P: Array = []
	for m in 4:
		P.append(PackedFloat64Array([0.0, 0.0, 0.0]))
	var R := rows.size()
	for it in iters:
		if cancel.is_valid() and cancel.call():
			return []
		var Vs := Num.parallel_for(R, func(ri: int) -> PackedFloat64Array: return model_v(rows[ri], V))
		# normal equations: the sums of every (motor, row), added up in row order
		var parts := Num.parallel_for(4 * R, func(j: int) -> PackedFloat64Array:
			return PackedFloat64Array() if cancel.is_valid() and cancel.call() else _normal_sums(rows[j % R], j / R, Vs[j % R], kv_rad))
		if cancel.is_valid() and cancel.call():
			return []
		for m in 4:
			var s := Num.zeros(9)
			for ri in R:
				for q in 9:
					s[q] += parts[m * R + ri][q]
			P[m] = Num.solve_normal([[s[0], s[1], s[2]], [s[1], s[3], s[4]], [s[2], s[4], s[5]]], PackedFloat64Array([s[6], s[7], s[8]]))
		# the voltage scale (rows with a logged vbat, per motor) and the window voltages (the others, per row)
		var sc := Num.parallel_for(4 * R, func(j: int) -> Variant:
			var r: Dictionary = rows[j / 4]
			if (r.vbat as PackedFloat64Array).is_empty():
				return _window_volts(r, P, kv_rad) if j % 4 == 0 else null
			return _scale_sums(r, P[j % 4], j % 4, kv_rad))
		var num_s := 0.0
		var den_s := 0.0
		for x in sc:
			if x is Dictionary:
				V.merge(x, true)
			elif x != null:
				num_s += x[0]
				den_s += x[1]
		if den_s > 0.0:
			V[VBAT_SCALE] = num_s / den_s
	var r2 := _r2(rows, P, V, kv_rad)
	return [P, V, r2]


## The 9 sums of the normal equations (s00 s01 s02 s11 s12 s22, then y0 y1 y2) of motor m over a row's ok samples
static func _normal_sums(r: Dictionary, m: int, vv: PackedFloat64Array, kv_rad: float) -> PackedFloat64Array:
	var ok: PackedByteArray = r.ok
	var uu: PackedFloat64Array = r.u[m]
	var ww: PackedFloat64Array = r.w[m]
	var yy: PackedFloat64Array = r.dw[m]
	var s00 := 0.0
	var s01 := 0.0
	var s02 := 0.0
	var s11 := 0.0
	var s12 := 0.0
	var s22 := 0.0
	var y0 := 0.0
	var y1 := 0.0
	var y2 := 0.0
	for i in ok.size():
		if ok[i] == 0:
			continue
		var w := ww[i]
		var x0 := uu[i] * vv[i] - w / kv_rad
		var x1 := -(w * w)
		var y := yy[i]
		s00 += x0 * x0
		s01 += x0 * x1
		s02 -= x0
		s11 += x1 * x1
		s12 -= x1
		s22 += 1.0
		y0 += x0 * y
		y1 += x1 * y
		y2 -= y
	return PackedFloat64Array([s00, s01, s02, s11, s12, s22, y0, y1, y2])


## [numerator, denominator] of the vbat scale for motor m of a row with a logged vbat
static func _scale_sums(r: Dictionary, p: PackedFloat64Array, m: int, kv_rad: float) -> PackedFloat64Array:
	var ok: PackedByteArray = r.ok
	var vb: PackedFloat64Array = r.vbat
	var uu: PackedFloat64Array = r.u[m]
	var ww: PackedFloat64Array = r.w[m]
	var yy: PackedFloat64Array = r.dw[m]
	var a := p[0]
	var b := p[1]
	var c := p[2]
	var ns := 0.0
	var ds := 0.0
	for i in ok.size():
		if ok[i] == 0:
			continue
		var w := ww[i]
		var lhs := yy[i] + a * w / kv_rad + b * w * w + c
		var x := a * uu[i] * vb[i]
		ns += lhs * x
		ds += x * x
	return PackedFloat64Array([ns, ds])


## The voltage of each 2-s window (with at least 200 ok samples) of a row without a logged vbat: {key: volts}
static func _window_volts(r: Dictionary, P: Array, kv_rad: float) -> Dictionary:
	var ok: PackedByteArray = r.ok
	var win: PackedInt32Array = r.win
	var acc := {}
	for i in ok.size():
		if ok[i] == 0:
			continue
		var k := win[i]
		var e: PackedFloat64Array
		if acc.has(k):
			e = acc[k]
		else:
			e = PackedFloat64Array([0.0, 0.0, 0.0])
			acc[k] = e
		e[2] += 1.0
		for m in 4:
			var a: float = P[m][0]
			var w: float = r.w[m][i]
			var au: float = a * r.u[m][i]
			var lhs: float = r.dw[m][i] + a * w / kv_rad + P[m][1] * w * w + P[m][2]
			e[0] += lhs * au
			e[1] += au * au
	var out := {}
	for k in acc:
		var e: PackedFloat64Array = acc[k]
		if e[2] >= 200.0:
			out[wkey(r.name, k)] = e[0] / e[1]
	return out


## R^2 of the equation error over all ok samples and motors: 1 - var(residual) / var(dw), in the order
## row, motor, sample. The residuals and dw are not held as two arrays of 4 x (ok samples): their
## sums are Num.psum's pairwise sums, taken over blocks that _fill() computes again each time (the
## same numbers in the same order, so the same result as variance() of the arrays; about 1 GB less
## at an hour of flight).
static func _r2(rows: Array, P: Array, V: Dictionary, kv_rad: float) -> float:
	var ix := Num.parallel_for(rows.size(), func(ri: int) -> PackedInt32Array:
		var ok: PackedByteArray = rows[ri].ok
		var o := PackedInt32Array()
		for i in ok.size():
			if ok[i] != 0:
				o.append(i)
		return o)
	var off := PackedInt64Array([0])
	for x: PackedInt32Array in ix:
		off.append(off[-1] + 4 * x.size())
	var ctx := {"rows": rows, "P": P, "kv": kv_rad, "ix": ix, "off": off,
		"vv": Num.parallel_for(rows.size(), func(ri: int) -> PackedFloat64Array: return model_v(rows[ri], V))}
	var n: int = off[-1]
	if n == 0:
		return 1.0 - Num.variance(PackedFloat64Array()) / Num.variance(PackedFloat64Array())
	var out := [0.0, 0.0]
	for kind in 2:
		var mean_x := _psum_blocks(n, _fill.bind(ctx, kind, false, 0.0)) / n
		out[kind] = _psum_blocks(n, _fill.bind(ctx, kind, true, mean_x)) / n
	return 1.0 - out[0] / out[1]


## Num.psum over n values that fill(buf, a, count) writes to buf[0..count) for the values a..a+count-1:
## the leaves (blocks of at most 128, as psum's) are summed in parallel, then added in the tree's order.
static func _psum_blocks(n: int, fill: Callable) -> float:
	var leaves := PackedInt64Array()
	_leaves(0, n, leaves)
	var nl := leaves.size() / 2
	var chunks := Num.parallel_for((nl + 255) / 256, func(c: int) -> PackedFloat64Array:
		var buf := Num.zeros(128)
		var vals := PackedFloat64Array()
		for l in range(c * 256, mini(nl, (c + 1) * 256)):
			fill.call(buf, leaves[2 * l], leaves[2 * l + 1])
			vals.append(Num.psum(buf, 0, leaves[2 * l + 1]))
		return vals)
	var vals := PackedFloat64Array()
	for c: PackedFloat64Array in chunks:
		vals.append_array(c)
	return _tree_sum(n, vals, [0])


## Starts and sizes of the leaves of the sum over cnt values from a (left half: half of cnt rounded down to a multiple of 8)
static func _leaves(a: int, cnt: int, out: PackedInt64Array) -> void:
	if cnt <= 128:
		out.append(a)
		out.append(cnt)
		return
	var n2 := cnt / 2
	n2 -= n2 % 8
	_leaves(a, n2, out)
	_leaves(a + n2, cnt - n2, out)


static func _tree_sum(cnt: int, vals: PackedFloat64Array, pos: Array) -> float:
	if cnt <= 128:
		pos[0] += 1
		return vals[pos[0] - 1]
	var n2 := cnt / 2
	n2 -= n2 % 8
	return _tree_sum(n2, vals, pos) + _tree_sum(cnt - n2, vals, pos)


## The values a..a+cnt-1 of r2's sequence: kind 0 the residual dw - model, 1 dw; squared deviation from c if sq
static func _fill(buf: PackedFloat64Array, a: int, cnt: int, ctx: Dictionary, kind: int, sq: bool, c: float) -> void:
	var off: PackedInt64Array = ctx.off
	var rows: Array = ctx.rows
	var P: Array = ctx.P
	var kv_rad: float = ctx.kv
	var ri := 0
	while off[ri + 1] <= a:
		ri += 1
	var m := 0
	var k := 0
	var ix: PackedInt32Array
	var vv: PackedFloat64Array
	var uu: PackedFloat64Array
	var ww: PackedFloat64Array
	var yy: PackedFloat64Array
	var cr := 0
	var pa := 0.0
	var pb := 0.0
	var pc := 0.0
	var loaded := false
	for j in cnt:
		if not loaded:
			ix = ctx.ix[ri]
			cr = ix.size()
			var local := a + j - off[ri]
			m = local / cr
			k = local % cr
			vv = ctx.vv[ri]
			var r: Dictionary = rows[ri]
			uu = r.u[m]
			ww = r.w[m]
			yy = r.dw[m]
			pa = P[m][0]
			pb = P[m][1]
			pc = P[m][2]
			loaded = true
		var i := ix[k]
		var w := ww[i]
		var x := yy[i]
		if kind == 0:
			x = yy[i] - (pa * (uu[i] * vv[i] - w / kv_rad) - pb * w * w - pc)
		if sq:
			var d := x - c
			x = d * d
		buf[j] = x
		k += 1
		if k == cr:
			k = 0
			m += 1
			if m == 4:
				ri += 1
				while ri < rows.size() and ctx.ix[ri].size() == 0:
					ri += 1
				m = 0
				loaded = false
			else:
				var r2: Dictionary = rows[ri]
				uu = r2.u[m]
				ww = r2.w[m]
				yy = r2.dw[m]
				pa = P[m][0]
				pb = P[m][1]
				pc = P[m][2]


static func scaled(P: Array, lam: float) -> Array:
	var o: Array = []
	for m in 4:
		var p: PackedFloat64Array = P[m]
		o.append(PackedFloat64Array([p[0] * lam, p[1] * lam, p[2] * lam]))
	return o


## Free-run rotor speeds from the delayed commands; re-synchronised to the log where not ok.
## cancel (optional): polled every 10 000 samples; when it answers true the rest stays 0 (the caller stops).
static func simulate(row: Dictionary, P: Array, V: Dictionary, kv_rad: float, vv: PackedFloat64Array = PackedFloat64Array(), substeps: int = 4,
		cancel: Callable = Callable()) -> Array:
	var t: PackedFloat64Array = row.t
	var n := t.size()
	if vv.is_empty():
		vv = model_v(row, V)
	var ok: PackedByteArray = row.ok
	var u0: PackedFloat64Array = row.u[0]
	var u1: PackedFloat64Array = row.u[1]
	var u2: PackedFloat64Array = row.u[2]
	var u3: PackedFloat64Array = row.u[3]
	var w0: PackedFloat64Array = row.w[0]
	var w1: PackedFloat64Array = row.w[1]
	var w2: PackedFloat64Array = row.w[2]
	var w3: PackedFloat64Array = row.w[3]
	var a0: float = P[0][0]
	var b0: float = P[0][1]
	var c0: float = P[0][2]
	var a1: float = P[1][0]
	var b1: float = P[1][1]
	var c1: float = P[1][2]
	var a2: float = P[2][0]
	var b2: float = P[2][1]
	var c2: float = P[2][2]
	var a3: float = P[3][0]
	var b3: float = P[3][1]
	var c3: float = P[3][2]
	var s0 := PackedFloat64Array()
	var s1 := PackedFloat64Array()
	var s2 := PackedFloat64Array()
	var s3 := PackedFloat64Array()
	s0.resize(n)
	s1.resize(n)
	s2.resize(n)
	s3.resize(n)
	var dts := PackedFloat64Array()
	dts.resize(n)
	for i in n - 1:
		dts[i] = t[i + 1] - t[i]
	dts[n - 1] = Num.median(dts.slice(0, n - 1)) if n > 1 else 0.001
	var x0 := w0[0]
	var x1 := w1[0]
	var x2 := w2[0]
	var x3 := w3[0]
	var cc := cancel.is_valid()
	for i0 in range(0, n, 10000):
		if cc and cancel.call():
			break
		for i in range(i0, mini(i0 + 10000, n)):
			if ok[i] == 0:
				x0 = w0[i]
				x1 = w1[i]
				x2 = w2[i]
				x3 = w3[i]
			var h := minf(dts[i], 0.005) / substeps
			var v := vv[i]
			var e0 := u0[i] * v
			var e1 := u1[i] * v
			var e2 := u2[i] * v
			var e3 := u3[i] * v
			for _k in substeps:
				x0 = x0 + h * (a0 * (e0 - x0 / kv_rad) - b0 * x0 * x0 - c0)
				x1 = x1 + h * (a1 * (e1 - x1 / kv_rad) - b1 * x1 * x1 - c1)
				x2 = x2 + h * (a2 * (e2 - x2 / kv_rad) - b2 * x2 * x2 - c2)
				x3 = x3 + h * (a3 * (e3 - x3 / kv_rad) - b3 * x3 * x3 - c3)
				if x0 < 0.0:
					x0 = 0.0
				if x1 < 0.0:
					x1 = 0.0
				if x2 < 0.0:
					x2 = 0.0
				if x3 < 0.0:
					x3 = 0.0
			s0[i] = x0
			s1[i] = x1
			s2[i] = x2
			s3[i] = x3
	return [s0, s1, s2, s3]


## [rms per motor, mean per motor] of the free-run error [Hz] over the ok samples.
static func free_run_error_hz(row: Dictionary, sim: Array) -> Array:
	var ok: PackedByteArray = row.ok
	var rms := PackedFloat64Array()
	var mean := PackedFloat64Array()
	for m in 4:
		var s: PackedFloat64Array = sim[m]
		var w: PackedFloat64Array = row.w[m]
		var e := PackedFloat64Array()
		var e2 := PackedFloat64Array()
		for i in ok.size():
			if ok[i] != 0:
				var v := (s[i] - w[i]) / TAU
				e.append(v)
				e2.append(v * v)
		rms.append(sqrt(Num.mean(e2)))
		mean.append(Num.mean(e))
	return [rms, mean]


## The samples the lambda search runs on, independent of the input order: every session gives
## floorf(max_samples * n_i / n_total) samples (all if the total fits) as blocks of ~block samples
## spaced evenly over the session.
static func lambda_subset(rows: Array, max_samples: int = 150000, block: int = SUBSET_BLOCK) -> Array:
	var tot := 0
	for r in rows:
		tot += (r.t as PackedFloat64Array).size()
	var out: Array = []
	for r in rows:
		var n: int = (r.t as PackedFloat64Array).size()
		var share := n if tot <= max_samples else max_samples * n / tot
		var m := maxi(1, Num.pyround_int(float(share) / block))
		var blen := mini(n, share / m)
		for j in m:
			var a := mini(maxi(0, (2 * j + 1) * n / (2 * m) - blen / 2), n - blen)
			out.append(slice_row(r, a, a + blen))
	return out


static func slice_row(r: Dictionary, a: int, b: int) -> Dictionary:
	var o := {"name": r.name}
	for k in ["t", "ok", "win", "vbat"]:
		var v = r[k]
		o[k] = v.slice(a, b) if v.size() > 0 else v
	for k in ["u", "w", "dw"]:
		o[k] = (r[k] as Array).map(func(c): return c.slice(a, b))
	return o


## Scales a, b, c together by lambda (golden section over 0.7..3.5, 9 steps) to the smallest free-run
## error on lambda_subset (RMS over all ok samples and motors). [lambda, rms at 1, rms at lambda] (Hz).
## Drops the rows' "dw" (fit() is its last user) to save memory.
static func refine_dynamics(rows: Array, P: Array, V: Dictionary, kv_rad: float, cancel: Callable = Callable(), max_samples: int = 150000) -> Array:
	var sub := lambda_subset(rows, max_samples)
	var vvs: Array = sub.map(func(r): return model_v(r, V))
	var cost := func(lam: float) -> float:
		if cancel.is_valid() and cancel.call():
			return INF
		var Pl := scaled(P, lam)
		var oks: Array = sub.map(func(r): return (r.ok as PackedByteArray).count(1))
		var errs := Num.parallel_for(sub.size(), func(j: int) -> PackedFloat64Array:
			return free_run_error_hz(sub[j], simulate(sub[j], Pl, V, kv_rad, vvs[j]))[0] if oks[j] > 0 else PackedFloat64Array())
		var se := 0.0
		var nn := 0
		for j in sub.size():
			var k: int = oks[j]
			if k > 0:
				var s2 := 0.0
				for v in errs[j]:
					s2 += v * v
				se += s2 * k
				nn += 4 * k
		return sqrt(se / maxi(nn, 1))
	var lam := golden(cost, LAMBDA_LO, LAMBDA_HI, 9)
	if cancel.is_valid() and cancel.call():
		return []
	var c1: float = cost.call(1.0)
	var cl: float = cost.call(lam)
	if cancel.is_valid() and cancel.call():
		return []
	for r in rows:
		r.erase("dw")      # the derivative of the rotor speed is only for fit(): a quarter of a row's memory
	return [lam, c1, cl]


## Golden section search: the middle of the last bracket.
static func golden(cost: Callable, lo: float, hi: float, steps: int) -> float:
	var g := (sqrt(5.0) - 1.0) / 2.0
	var x1 := hi - g * (hi - lo)
	var x2 := lo + g * (hi - lo)
	var f1: float = cost.call(x1)
	var f2: float = cost.call(x2)
	for _k in steps:
		if f1 < f2:
			hi = x2
			x2 = x1
			f2 = f1
			x1 = hi - g * (hi - lo)
			f1 = cost.call(x1)
		else:
			lo = x1
			x1 = x2
			f1 = f2
			x2 = lo + g * (hi - lo)
			f2 = cost.call(x2)
	return (lo + hi) / 2.0


## Roll / pitch differential of four columns at sample q.
static func _diff(c: Array, q: int, axis: int) -> float:
	return c[2][q] + c[3][q] - c[0][q] - c[1][q] if axis == 0 else c[0][q] + c[2][q] - c[1][q] - c[3][q]


## csd(x, y) / psd(x) summed over pairs as scipy.signal.csd / welch (nperseg 1024, Hann, half overlap) at the bins nearest
## freqs: {bins, cos, sin, win, xy_re, xy_im, xx}; _frf_add() adds a pair, _frf_ratio() gives [re, im] per bin (NAN without power).
static func _frf_new(fs: float, freqs: Array) -> Dictionary:
	var nper := 1024
	var B := {"cos": [], "sin": [], "win": Num.hann(nper), "xy_re": Num.zeros(freqs.size()), "xy_im": Num.zeros(freqs.size()), "xx": Num.zeros(freqs.size())}
	for f in freqs:
		var best := 0
		var bd := INF
		for k in nper / 2 + 1:
			var d := absf(k * fs / nper - float(f))
			if d < bd:
				bd = d
				best = k
		var c := PackedFloat64Array()
		var s := PackedFloat64Array()
		c.resize(nper)
		s.resize(nper)
		for k in nper:
			c[k] = cos(TAU * best * k / nper)
			s[k] = -sin(TAU * best * k / nper)
		B.cos.append(c)
		B.sin.append(s)
	return B


static func _frf_add(B: Dictionary, x: PackedFloat64Array, y: PackedFloat64Array) -> void:
	var nper := 1024
	var step := nper / 2
	var nb: int = B.cos.size()
	var win: PackedFloat64Array = B.win
	var nseg := (x.size() - step) / step
	if nseg <= 0:
		return
	var sxy_re := Num.zeros(nb)
	var sxy_im := Num.zeros(nb)
	var sxx := Num.zeros(nb)
	for sg in nseg:
		var o := sg * step
		var mx := Num.psum(x, o, nper) / nper
		var my := Num.psum(y, o, nper) / nper
		for bi in nb:
			var c: PackedFloat64Array = B.cos[bi]
			var s: PackedFloat64Array = B.sin[bi]
			var xr := 0.0
			var xi := 0.0
			var yr := 0.0
			var yi := 0.0
			for k in nper:
				var vx := (x[o + k] - mx) * win[k]
				var vy := (y[o + k] - my) * win[k]
				xr += vx * c[k]
				xi += vx * s[k]
				yr += vy * c[k]
				yi += vy * s[k]
			# conj(X) * Y
			sxy_re[bi] += xr * yr + xi * yi
			sxy_im[bi] += xr * yi - xi * yr
			sxx[bi] += xr * xr + xi * xi
	for bi in nb:
		B.xy_re[bi] += sxy_re[bi] / nseg
		B.xy_im[bi] += sxy_im[bi] / nseg
		B.xx[bi] += sxx[bi] / nseg


static func _frf_ratio(B: Dictionary) -> Array:
	var re := PackedFloat64Array()
	var im := PackedFloat64Array()
	for bi in (B.xx as PackedFloat64Array).size():
		var xx: float = B.xx[bi]
		re.append(B.xy_re[bi] / xx if xx > 0.0 else NAN)
		im.append(B.xy_im[bi] / xx if xx > 0.0 else NAN)
	return [re, im]


static func _fs(rows: Array) -> float:
	var t0: PackedFloat64Array = rows[0].t
	var dts := PackedFloat64Array()
	for i in t0.size() - 1:
		dts.append(t0[i + 1] - t0[i])
	return 1.0 / Num.median(dts)


## |measured / simulated| roll+pitch rotor-speed differential at a few frequencies (1 = right speed).
static func response_gain(rows: Array, sims: Array, freqs: Array = [5, 10, 20, 30], cancel: Callable = Callable()) -> Dictionary:
	var B := _frf_new(_fs(rows), freqs)
	for ri in rows.size():
		var r: Dictionary = rows[ri]
		var rr := Num.runs(r.ok)
		for j in range(0, rr.size(), 2):
			if rr[j + 1] - rr[j] < 2048:
				continue
			if cancel.is_valid() and cancel.call():
				return {}
			for axis in 2:
				var xs := PackedFloat64Array()
				var xm := PackedFloat64Array()
				for q in range(rr[j], rr[j + 1]):
					xs.append(_diff(sims[ri], q, axis))
					xm.append(_diff(r.w, q, axis))
				_frf_add(B, xs, xm)
	var H := _frf_ratio(B)
	var out := {}
	for bi in freqs.size():
		var re: float = H[0][bi]
		var im: float = H[1][bi]
		out[freqs[bi]] = null if is_nan(re) else Num.pyround(sqrt(re * re + im * im), 2)
	return out


## The torque path of the free run against the gyro over the chirps: per roll / pitch csd(x, y) / psd(x) at freqs with
## x the rotor speed squared differential of the free run and y the logged angular acceleration (gyroUnfilt), over the
## runs of >= 2048 ok samples in masks. (1 = in step: the rotation gain times this is the measured torque path.)
## Returns {re: [roll, pitch], im: [roll, pitch], w: mean free-run rotor speed per motor there [rad/s]}, {} without runs.
static func torque_path(rows: Array, sims: Array, logs: Array, masks: Array, freqs: Array = TP_FREQS, cancel: Callable = Callable()) -> Dictionary:
	var fs := _fs(rows)
	var Bs := [_frf_new(fs, freqs), _frf_new(fs, freqs)]
	var ws := Num.zeros(4)
	var nw := 0
	for ri in rows.size():
		var sim: Array = sims[ri]
		var gu: Array = (logs[ri] as Flight).gyro_unfilt
		var rr := Num.runs(Num.mask_and(rows[ri].ok, masks[ri]))
		for j in range(0, rr.size(), 2):
			var a := rr[j]
			var b := rr[j + 1]
			if b - a < 2048:
				continue
			if cancel.is_valid() and cancel.call():
				return {}
			var W2: Array = []
			for m in 4:
				var c := PackedFloat64Array()
				c.resize(b - a)
				for i in b - a:
					var v: float = sim[m][a + i]
					c[i] = v * v
					ws[m] += v
				W2.append(c)
			nw += b - a
			for axis in 2:
				var x := PackedFloat64Array()
				x.resize(b - a)
				var g := PackedFloat64Array()
				g.resize(b - a)
				for i in b - a:
					x[i] = _diff(W2, i, axis)
					g[i] = deg_to_rad(gu[axis][a + i])
				_frf_add(Bs[axis], x, Num.gradient(g, 1.0 / fs))
	if nw == 0:
		return {}
	for m in 4:
		ws[m] /= nw
	var H0 := _frf_ratio(Bs[0])
	var H1 := _frf_ratio(Bs[1])
	return {"re": [H0[0], H1[0]], "im": [H0[1], H1[1]], "w": ws}


## Speed factor lam and extra dead time d that put the free run in step with the gyro (R: torque_path() of a free run whose
## time constant at the chirps' rotor speed is tau0). Scaling a, b, c by lam makes the time constant tau0 / lam (the steady
## state stays) and R becomes R (1 + j w tau0 / lam) / (1 + j w tau0) e^(j w d): lam makes its magnitude flat over freqs
## (golden section on the variance of ln|R| over TP_LAMBDA, both axes), d its phase zero (least squares through 0).
## Returns [lam, d [s]].
static func torque_refine(R: Dictionary, tau0: float, freqs: Array = TP_FREQS) -> Array:
	var nf := freqs.size()
	var w := PackedFloat64Array()
	for f in freqs:
		w.append(TAU * float(f))
	var lnr: Array = []
	for ax in 2:
		var c := PackedFloat64Array()
		for i in nf:
			var re: float = R.re[ax][i]
			var im: float = R.im[ax][i]
			c.append(log(sqrt(re * re + im * im)) - log(sqrt(1.0 + w[i] * tau0 * w[i] * tau0)))
		lnr.append(c)
	var cost := func(lam: float) -> float:
		var tot := 0.0
		for ax in 2:
			var z := PackedFloat64Array()
			for i in nf:
				var x := w[i] * tau0 / lam
				z.append(lnr[ax][i] + log(sqrt(1.0 + x * x)))
			tot += Num.variance(z)
		return tot
	var lam := golden(cost, TP_LAMBDA_LO, TP_LAMBDA_HI, 12)
	var num := 0.0
	var den := 0.0
	for ax in 2:
		for i in nf:
			var psi := atan2(R.im[ax][i], R.re[ax][i]) - atan(w[i] * tau0) + atan(w[i] * tau0 / lam)
			num += w[i] * psi
			den += w[i] * w[i]
	return [lam, -num / den]


## Small-signal time constant [s] per motor at rotor speed w[m].
static func time_constant(P: Array, w: PackedFloat64Array, kv_rad: float) -> PackedFloat64Array:
	var o := PackedFloat64Array()
	for m in 4:
		o.append(1.0 / (P[m][0] / kv_rad + 2.0 * P[m][1] * w[m]))
	return o


## Rotor speed [rad/s] per motor in steady state for command u at voltage V.
static func steady_speed(P: Array, u: float, V: float, kv_rad: float) -> PackedFloat64Array:
	var o := PackedFloat64Array()
	for m in 4:
		var a: float = P[m][0]
		var b: float = P[m][1]
		var c: float = P[m][2]
		var rhs := a * u * V - c
		o.append(maxf((-(a / kv_rad) + sqrt((a / kv_rad) * (a / kv_rad) + 4.0 * b * maxf(rhs, 0.0))) / (2.0 * b), 0.0))
	return o


static func command_for_speed(P: Array, w: float, V: float, kv_rad: float) -> PackedFloat64Array:
	var o := PackedFloat64Array()
	for m in 4:
		var a: float = P[m][0]
		var b: float = P[m][1]
		var c: float = P[m][2]
		o.append((b * w * w + c + a * w / kv_rad) / (a * V))
	return o
