extends RefCounted
## real_quad logfit: motor/ESC/prop dynamics from the DShot command to the rotor speed (eRPM).
##
## Model per motor (rotor speed w [rad/s], command u 0..1, voltage V), command delayed by 1 ms:
##     dw/dt = a * (u * V - w / Kv) - b * w^2 - c
## V is the "model voltage": the logged vbat (low-passed) times one global scale where logged (absorbs
## ESC losses and a rated Kv that differs from the real one), else one value per 2-s window.
## a, b, c, the scale and the window voltages come from alternating least squares (fit()); then a, b,
## c are scaled together by lambda (refine_dynamics(), golden section on the free-run error).
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


## cancel (optional): polled between the motors and every 10 000 samples of the long loops; when it answers
## true the answer is {} (the caller stops anyway).
static func prep(L: Flight, use: PackedByteArray, delay_s: float = DELAY_S, cancel: Callable = Callable()) -> Dictionary:
	var fs := L.fs
	var cc := cancel.is_valid()
	var n := L.n
	var n_sg := roundi(0.009 * fs) | 1
	var u0 := L.u()
	var d := roundi(delay_s * fs)
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
		var uc: PackedFloat64Array = u0[m]
		if d > 0:
			var ud := PackedFloat64Array()
			ud.resize(n)
			for i in n:
				ud[i] = uc[maxi(i - d, 0)]
			uc = ud
		u.append(uc)
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
	var ikv := 1.0 / kv_rad
	for it in iters:
		if cancel.is_valid() and cancel.call():
			return []
		var Vs: Array = rows.map(func(r): return model_v(r, V))
		for m in 4:
			var s := PackedFloat64Array()
			s.resize(9)
			s.fill(0.0)
			for ri in rows.size():
				if cancel.is_valid() and cancel.call():
					return []
				var r: Dictionary = rows[ri]
				var ok: PackedByteArray = r.ok
				var uu: PackedFloat64Array = r.u[m]
				var ww: PackedFloat64Array = r.w[m]
				var yy: PackedFloat64Array = r.dw[m]
				var vv: PackedFloat64Array = Vs[ri]
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
				s[0] += s00
				s[1] += s01
				s[2] += s02
				s[3] += s11
				s[4] += s12
				s[5] += s22
				s[6] += y0
				s[7] += y1
				s[8] += y2
			P[m] = Num.solve_normal([[s[0], s[1], s[2]], [s[1], s[3], s[4]], [s[2], s[4], s[5]]], PackedFloat64Array([s[6], s[7], s[8]]))
		var num_s := 0.0
		var den_s := 0.0
		for ri in rows.size():
			var r: Dictionary = rows[ri]
			var ok: PackedByteArray = r.ok
			var vb: PackedFloat64Array = r.vbat
			if not vb.is_empty():
				for m in 4:
					var a: float = P[m][0]
					var b: float = P[m][1]
					var c: float = P[m][2]
					var uu: PackedFloat64Array = r.u[m]
					var ww: PackedFloat64Array = r.w[m]
					var yy: PackedFloat64Array = r.dw[m]
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
					num_s += ns
					den_s += ds
				continue
			# per 2-s window
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
			for k in acc:
				var e: PackedFloat64Array = acc[k]
				if e[2] < 200.0:
					continue
				V[wkey(r.name, k)] = e[0] / e[1]
		if den_s > 0.0:
			V[VBAT_SCALE] = num_s / den_s
	var r2 := _r2(rows, P, V, kv_rad)
	return [P, V, r2]


## R^2 of the equation error over all ok samples and motors: 1 - var(residual) / var(dw), in the order
## row, motor, sample. The residuals and dw are not held as two arrays of 4 x (ok samples): their
## sums are Num.psum's pairwise sums, taken over blocks that _fill() computes again each time (the
## same numbers in the same order, so the same result as variance() of the arrays; about 1 GB less
## at an hour of flight).
static func _r2(rows: Array, P: Array, V: Dictionary, kv_rad: float) -> float:
	var ctx := {"rows": rows, "P": P, "kv": kv_rad, "vv": rows.map(func(r): return model_v(r, V)), "ix": [], "off": PackedInt64Array([0])}
	for r in rows:
		var ok: PackedByteArray = r.ok
		var ix := PackedInt32Array()
		for i in ok.size():
			if ok[i] != 0:
				ix.append(i)
		ctx.ix.append(ix)
		ctx.off.append(ctx.off[-1] + 4 * ix.size())
	var n: int = ctx.off[-1]
	if n == 0:
		return 1.0 - Num.variance(PackedFloat64Array()) / Num.variance(PackedFloat64Array())
	var buf := PackedFloat64Array()
	buf.resize(128)
	var out := [0.0, 0.0]
	for kind in 2:
		var mean_x := _psum_blocks(n, buf, _fill.bind(ctx, kind, false, 0.0)) / n
		out[kind] = _psum_blocks(n, buf, _fill.bind(ctx, kind, true, mean_x)) / n
	return 1.0 - out[0] / out[1]


## Num.psum over n values that fill(buf, a, count) writes to buf[0..count) for the values a..a+count-1
## (blocks of at most 128, as psum's leaves)
static func _psum_blocks(n: int, buf: PackedFloat64Array, fill: Callable, a: int = 0, cnt: int = -1) -> float:
	if cnt < 0:
		cnt = n
	if cnt <= 128:
		fill.call(buf, a, cnt)
		return Num.psum(buf, 0, cnt)
	var n2 := cnt / 2
	n2 -= n2 % 8
	return _psum_blocks(n, buf, fill, a, n2) + _psum_blocks(n, buf, fill, a + n2, cnt - n2)


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
		var se := 0.0
		var nn := 0
		var Pl := scaled(P, lam)
		for j in sub.size():
			if cancel.is_valid() and cancel.call():
				return INF
			var r: Dictionary = sub[j]
			var k := (r.ok as PackedByteArray).count(1)
			if k > 0:
				var rms: PackedFloat64Array = free_run_error_hz(r, simulate(r, Pl, V, kv_rad, vvs[j]))[0]
				var s2 := 0.0
				for v in rms:
					s2 += v * v
				se += s2 * k
				nn += 4 * k
		return sqrt(se / maxi(nn, 1))
	var lo := LAMBDA_LO
	var hi := LAMBDA_HI
	var g := (sqrt(5.0) - 1.0) / 2.0
	var x1 := hi - g * (hi - lo)
	var x2 := lo + g * (hi - lo)
	var f1: float = cost.call(x1)
	var f2: float = cost.call(x2)
	for _k in 9:
		if cancel.is_valid() and cancel.call():
			return []
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
	var lam := (lo + hi) / 2.0
	var c1: float = cost.call(1.0)
	var cl: float = cost.call(lam)
	if cancel.is_valid() and cancel.call():
		return []
	for r in rows:
		r.erase("dw")      # the derivative of the rotor speed is only for fit(): a quarter of a row's memory
	return [lam, c1, cl]


## |measured / simulated| roll+pitch rotor-speed differential at a few frequencies (1 = right speed):
## the bins of scipy.signal.csd / welch (nperseg 1024, Hann, half overlap) nearest to them.
static func response_gain(rows: Array, sims: Array, freqs: Array = [5, 10, 20, 30], cancel: Callable = Callable()) -> Dictionary:
	var nper := 1024
	var t0: PackedFloat64Array = rows[0].t
	var dts := PackedFloat64Array()
	for i in t0.size() - 1:
		dts.append(t0[i + 1] - t0[i])
	var fs := 1.0 / Num.median(dts)
	var bins: Array[int] = []
	for f in freqs:
		var best := 0
		var bd := INF
		for k in nper / 2 + 1:
			var d := absf(k * fs / nper - float(f))
			if d < bd:
				bd = d
				best = k
		bins.append(best)
	var nb := bins.size()
	var cosv: Array = []
	var sinv: Array = []
	var win := Num.hann(nper)
	for b in bins:
		var c := PackedFloat64Array()
		var s := PackedFloat64Array()
		c.resize(nper)
		s.resize(nper)
		for k in nper:
			c[k] = cos(TAU * b * k / nper)
			s[k] = -sin(TAU * b * k / nper)
		cosv.append(c)
		sinv.append(s)
	var acc_xy_re := Num.zeros(nb)
	var acc_xy_im := Num.zeros(nb)
	var acc_xx := Num.zeros(nb)
	var xs := PackedFloat64Array()
	var xm := PackedFloat64Array()
	for ri in rows.size():
		var r: Dictionary = rows[ri]
		var sim: Array = sims[ri]
		var rr := Num.runs(r.ok)
		for j in range(0, rr.size(), 2):
			var a := rr[j]
			var b := rr[j + 1]
			if b - a < 2048:
				continue
			if cancel.is_valid() and cancel.call():
				return {}
			for axis in 2:
				var nn := b - a
				xs.resize(nn)
				xm.resize(nn)
				for i in nn:
					var q := a + i
					if axis == 0:
						xm[i] = r.w[2][q] + r.w[3][q] - r.w[0][q] - r.w[1][q]
						xs[i] = sim[2][q] + sim[3][q] - sim[0][q] - sim[1][q]
					else:
						xm[i] = r.w[0][q] + r.w[2][q] - r.w[1][q] - r.w[3][q]
						xs[i] = sim[0][q] + sim[2][q] - sim[1][q] - sim[3][q]
				var step := nper / 2
				var nseg := (nn - step) / step
				var sxy_re := Num.zeros(nb)
				var sxy_im := Num.zeros(nb)
				var sxx := Num.zeros(nb)
				for sg in nseg:
					var o := sg * step
					var ms := Num.psum(xs, o, nper) / nper
					var mm := Num.psum(xm, o, nper) / nper
					for bi in nb:
						var c: PackedFloat64Array = cosv[bi]
						var s: PackedFloat64Array = sinv[bi]
						var sr := 0.0
						var si := 0.0
						var mr := 0.0
						var mi := 0.0
						for k in nper:
							var vs := (xs[o + k] - ms) * win[k]
							var vm := (xm[o + k] - mm) * win[k]
							sr += vs * c[k]
							si += vs * s[k]
							mr += vm * c[k]
							mi += vm * s[k]
						# conj(Xs) * Xm
						sxy_re[bi] += sr * mr + si * mi
						sxy_im[bi] += sr * mi - si * mr
						sxx[bi] += sr * sr + si * si
				if nseg > 0:
					for bi in nb:
						acc_xy_re[bi] += sxy_re[bi] / nseg
						acc_xy_im[bi] += sxy_im[bi] / nseg
						acc_xx[bi] += sxx[bi] / nseg
	var out := {}
	for bi in nb:
		var f: int = freqs[bi]
		if acc_xx[bi] != 0.0:
			out[f] = Num.pyround(sqrt(acc_xy_re[bi] * acc_xy_re[bi] + acc_xy_im[bi] * acc_xy_im[bi]) / acc_xx[bi], 2)
		else:
			out[f] = null
	return out


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
