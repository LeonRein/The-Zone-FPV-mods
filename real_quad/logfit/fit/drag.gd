extends RefCounted
## real_quad logfit: air drag from the accelerometer, and the top speed it implies.
##
## The accelerometer measures the specific force in the body frame; rotor thrust acts along body z
## only, so body x/y specific force is drag:
##     m g a_x = -(0.5 rho |v| CdA_x v_x + rotor_drag * sum(w) * v_x)        (same for y)
## The velocity comes from dead reckoning over short passes (<= 5 s) that start in a hover-like moment
## (velocity ~ 0, attitude from the ACC); each pass's start velocity and start tilt are fitted too (weak
## priors). Fitted globally: one scale on the body drag shape and the linear rotor drag, by a bounded
## Levenberg-Marquardt with the soft-L1 loss (f_scale 0.15) and a forward-difference Jacobian.
## top_speed(): level flight at full throttle with the plant's thrust model and drag.

const Num := preload("num.gd")
const Flight := preload("flight.gd")
const Segments := preload("segments.gd")

const G := 9.81
const RHO := 1.225
const D2R := PI / 180.0
const MAX_THR := 0.85
const MAX_RATE := 500.0
const V0_SD := 3.0
const TILT_SD := 8.0 * PI / 180.0
const PRIOR_W := 0.5
const F_SCALE := 0.15
const MAX_NFEV := 200


# 3x3 matrices as PackedFloat64Array row-major
static func _mm(a: PackedFloat64Array, b: PackedFloat64Array) -> PackedFloat64Array:
	var o := PackedFloat64Array()
	o.resize(9)
	for i in 3:
		for j in 3:
			o[3 * i + j] = a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j]
	return o


static func _skew(x: float, y: float, z: float) -> PackedFloat64Array:
	return PackedFloat64Array([0.0, -z, y, z, 0.0, -x, -y, x, 0.0])


static func _eye() -> PackedFloat64Array:
	return PackedFloat64Array([1, 0, 0, 0, 1, 0, 0, 0, 1])


## Rotation body->world that maps the measured up direction to world z (yaw = 0).
static func _align(up: PackedFloat64Array) -> PackedFloat64Array:
	var nrm := sqrt(up[0] * up[0] + up[1] * up[1] + up[2] * up[2])
	var a := [up[0] / nrm, up[1] / nrm, up[2] / nrm]
	var v := [a[1], -a[0], 0.0]          # a x (0, 0, 1)
	var c: float = a[2]
	if sqrt(v[0] * v[0] + v[1] * v[1]) < 1e-9:
		return _eye()
	var K := _skew(v[0], v[1], v[2])
	var KK := _mm(K, K)
	var o := _eye()
	for i in 9:
		o[i] += K[i] + KK[i] / (1.0 + c)
	return o


## The orthogonal polar factor (the U V^T of the SVD) by Newton's iteration.
static func _polar(r: PackedFloat64Array) -> PackedFloat64Array:
	var x := r.duplicate()
	for _it in 20:
		var a := x
		var det := a[0] * (a[4] * a[8] - a[5] * a[7]) - a[1] * (a[3] * a[8] - a[5] * a[6]) + a[2] * (a[3] * a[7] - a[4] * a[6])
		# inverse transposed = cofactor matrix / det
		var cof := PackedFloat64Array([
			a[4] * a[8] - a[5] * a[7], -(a[3] * a[8] - a[5] * a[6]), a[3] * a[7] - a[4] * a[6],
			-(a[1] * a[8] - a[2] * a[7]), a[0] * a[8] - a[2] * a[6], -(a[0] * a[7] - a[1] * a[6]),
			a[1] * a[5] - a[2] * a[4], -(a[0] * a[5] - a[2] * a[3]), a[0] * a[4] - a[1] * a[3]])
		var nx := PackedFloat64Array()
		nx.resize(9)
		var ch := 0.0
		for i in 9:
			nx[i] = 0.5 * (a[i] + cof[i] / det)
			ch = maxf(ch, absf(nx[i] - a[i]))
		x = nx
		if ch < 1e-16:
			break
	return x


## [[start, end], ...] sample ranges for dead reckoning (see the header).
static func passes(L: Flight, use: PackedByteArray, hover_sum_w2: float, seg_s: float = 5.0, tol: float = 0.15) -> Array:
	if not L.has_acc():
		return []
	var fs := L.fs
	var n := int(0.3 * fs)
	var lw := int(seg_s * fs)
	var an := Segments.norm_col(L.acc)
	var thrust := PackedFloat64Array()
	thrust.resize(L.n)
	for i in L.n:
		var s := 0.0
		for q in 4:
			var w: float = TAU * L.motor_hz[q][i]
			s += w * w
		thrust[i] = s / hover_sum_w2
	var gmax := Segments.max_abs_rows(L.gyro)
	var bad_cum := PackedInt32Array()
	bad_cum.resize(L.n + 1)
	bad_cum[0] = 0
	for i in L.n:
		bad_cum[i + 1] = bad_cum[i] + (1 if use[i] == 0 else 0)
	var ax: PackedFloat64Array = L.acc[0]
	var ay: PackedFloat64Array = L.acc[1]
	var out: Array = []
	var s := 0
	while s + lw < L.n:
		var ok := bad_cum[s + lw] - bad_cum[s] == 0
		if ok:
			ok = absf(Num.psum(an, s, n) / n - 1.0) < tol and absf(Num.psum(ax, s, n) / n) < tol and absf(Num.psum(ay, s, n) / n) < tol
		if ok:
			var gm := 0.0
			for i in range(s, s + n):
				gm = maxf(gm, gmax[i])
			ok = gm < 40.0 and absf(Num.psum(thrust, s, n) / n - 1.0) < 0.2
		if ok:
			var e := s + lw
			for k in lw:
				if L.throttle[s + k] > MAX_THR or gmax[s + k] > MAX_RATE:
					e = s + k
					break
			if e - s > 1.5 * fs:
				out.append([s, e])
			s = maxi(e, s + n)
		else:
			s += n / 3
	return out


## Integrates a pass; per 0.25-s block (centre): [time from start, R (body->world), I = int R f g dt,
## mean a_x, mean a_y, mean sum of rotor speeds].
static func dead_reckon(L: Flight, start: int, end: int, blk_s: float = 0.25) -> Array:
	var fs := L.fs
	var dt := 1.0 / fs
	var n0 := int(0.3 * fs)
	var blk := maxi(1, int(blk_s * fs))
	var h2 := blk / 2
	var ax: PackedFloat64Array = L.acc[0]
	var ay: PackedFloat64Array = L.acc[1]
	var az: PackedFloat64Array = L.acc[2]
	var up := PackedFloat64Array([0, 0, 0])
	for i in range(start, start + n0):
		up[0] += ax[i]
		up[1] += ay[i]
		up[2] += az[i]
	for k in 3:
		up[k] /= n0
	var R := _align(up)
	var I := PackedFloat64Array([0, 0, 0])
	var sw := PackedFloat64Array()
	sw.resize(end - start)
	for i in range(start, end):
		var s := 0.0
		for q in 4:
			s += TAU * L.motor_hz[q][i]
		sw[i - start] = s
	var out: Array = []
	for k in end - start:
		var i := start + k
		if k % blk == h2 and k + h2 < end - start:
			var a := i - h2
			var b := i + blk - h2
			var mx := 0.0
			var my := 0.0
			var ms := 0.0
			for j in range(a, b):
				mx += ax[j]
				my += ay[j]
				ms += sw[j - start]
			out.append([k * dt, R.duplicate(), I.duplicate(), mx / blk, my / blk, ms / blk])
		var f0 := ax[i] * G
		var f1 := ay[i] * G
		var f2 := az[i] * G
		I[0] += (R[0] * f0 + R[1] * f1 + R[2] * f2) * dt
		I[1] += (R[3] * f0 + R[4] * f1 + R[5] * f2) * dt
		I[2] += (R[6] * f0 + R[7] * f1 + R[8] * f2) * dt
		var om := _skew(L.gyro[0][i] * D2R * dt, L.gyro[1][i] * D2R * dt, L.gyro[2][i] * D2R * dt)
		var om2 := _mm(om, om)
		var step := _eye()
		for q in 9:
			step[q] += om[q] + 0.5 * om2[q]
		R = _mm(R, step)
		if k % 100 == 0:
			R = _polar(R)
	return out


## The fit problem (blocks of all passes) and its residuals.
class Problem extends RefCounted:
	var nb := 0
	var npass := 0
	var T := PackedFloat64Array()
	var R := PackedFloat64Array()        # 9 per block
	var I := PackedFloat64Array()        # 3 per block
	var A := PackedFloat64Array()        # 2 per block
	var SW := PackedFloat64Array()
	var seg := PackedInt32Array()
	var shp := PackedFloat64Array()
	var mass := 0.0
	var rd0 := 0.0
	var vb := PackedFloat64Array()       # body velocity per block (3), from the last residuals()

	func n_res() -> int:
		return 2 * nb + 5 * npass

	## Residuals: 2 per block (a_x, a_y), then 3 velocity and 2 tilt priors per pass.
	func residuals(x: PackedFloat64Array) -> PackedFloat64Array:
		var s := x[0]
		var rd := x[1] * rd0
		var out := PackedFloat64Array()
		out.resize(n_res())
		vb.resize(3 * nb)
		var mg := mass * 9.81
		for j in nb:
			var p := 2 + 5 * seg[j]
			var d0 := x[p + 3]
			var d1 := x[p + 4]
			var i0 := I[3 * j]
			var i1 := I[3 * j + 1]
			var i2 := I[3 * j + 2]
			var v0 := x[p] + i0 + (d1 * i2 - 0.0 * i1)
			var v1 := x[p + 1] + i1 + (0.0 * i0 - d0 * i2)
			var v2 := x[p + 2] + i2 + (d0 * i1 - d1 * i0) - 9.81 * T[j] * 1.0
			var r := 9 * j
			# Rf = Rj + skew(d) Rj, skew(d) = [[0, 0, d1], [0, 0, -d0], [-d1, d0, 0]]
			var rf0 := R[r] + d1 * R[r + 6]
			var rf1 := R[r + 1] + d1 * R[r + 7]
			var rf2 := R[r + 2] + d1 * R[r + 8]
			var rf3 := R[r + 3] - d0 * R[r + 6]
			var rf4 := R[r + 4] - d0 * R[r + 7]
			var rf5 := R[r + 5] - d0 * R[r + 8]
			var rf6 := R[r + 6] + (-d1 * R[r] + d0 * R[r + 3])
			var rf7 := R[r + 7] + (-d1 * R[r + 1] + d0 * R[r + 4])
			var rf8 := R[r + 8] + (-d1 * R[r + 2] + d0 * R[r + 5])
			var b0 := rf0 * v0 + rf3 * v1 + rf6 * v2
			var b1 := rf1 * v0 + rf4 * v1 + rf7 * v2
			var b2 := rf2 * v0 + rf5 * v1 + rf8 * v2
			vb[3 * j] = b0
			vb[3 * j + 1] = b1
			vb[3 * j + 2] = b2
			var sp := sqrt(b0 * b0 + b1 * b1 + b2 * b2)
			var fx := -(0.5 * 1.225 * sp * s * shp[0] * b0 + rd * SW[j] * b0)
			var fy := -(0.5 * 1.225 * sp * s * shp[1] * b1 + rd * SW[j] * b1)
			out[2 * j] = fx / mg - A[2 * j]
			out[2 * j + 1] = fy / mg - A[2 * j + 1]
		var o := 2 * nb
		for k in npass:
			for q in 3:
				out[o + 3 * k + q] = x[2 + 5 * k + q] / 3.0 * 0.5
		o += 3 * npass
		for k in npass:
			for q in 2:
				out[o + 2 * k + q] = x[2 + 5 * k + 3 + q] / (8.0 * PI / 180.0) * 0.5
		return out


static func _rho(f: float) -> Array:
	var z := (f / F_SCALE) * (f / F_SCALE)
	var t := 1.0 + z
	return [F_SCALE * F_SCALE * 2.0 * (sqrt(t) - 1.0), 1.0 / sqrt(t), pow(t, -1.5)]


static func _cost(f: PackedFloat64Array) -> float:
	var c := 0.0
	for v in f:
		var z := (v / F_SCALE) * (v / F_SCALE)
		c += F_SCALE * F_SCALE * 2.0 * (sqrt(1.0 + z) - 1.0)
	return 0.5 * c


## Forward-difference Jacobian (SciPy's step), as columns: g0, g1 (all rows) and per pass parameter q
## (0..4) one array over the rows of that pass (fit rows of its blocks; the priors are analytic).
static func _jac(pb: Problem, x: PackedFloat64Array, f0: PackedFloat64Array, lb: PackedFloat64Array, ub: PackedFloat64Array) -> Array:
	var rstep := sqrt(Num.EPS)
	var cols: Array = []
	for g in 2:
		var h := rstep * (1.0 if x[g] >= 0.0 else -1.0) * maxf(1.0, absf(x[g]))
		var xh := x.duplicate()
		xh[g] = x[g] + h
		if xh[g] > ub[g] or xh[g] < lb[g]:
			h = -h
			xh[g] = x[g] + h
		h = xh[g] - x[g]
		var fh := pb.residuals(xh)
		var c := PackedFloat64Array()
		c.resize(2 * pb.nb)
		for i in 2 * pb.nb:
			c[i] = (fh[i] - f0[i]) / h
		cols.append(c)
	var pc: Array = []
	for q in 5:
		var xh := x.duplicate()
		var hs := PackedFloat64Array()
		hs.resize(pb.npass)
		for k in pb.npass:
			var ix := 2 + 5 * k + q
			var h := rstep * (1.0 if x[ix] >= 0.0 else -1.0) * maxf(1.0, absf(x[ix]))
			xh[ix] = x[ix] + h
			hs[k] = xh[ix] - x[ix]
		var fh := pb.residuals(xh)
		var c := PackedFloat64Array()
		c.resize(2 * pb.nb)
		for j in pb.nb:
			var h: float = hs[pb.seg[j]]
			c[2 * j] = (fh[2 * j] - f0[2 * j]) / h
			c[2 * j + 1] = (fh[2 * j + 1] - f0[2 * j + 1]) / h
		pc.append(c)
	pb.residuals(x)
	return [cols, pc]


## Bounded Levenberg-Marquardt with the soft-L1 loss; the normal equations have an arrow structure
## (2 global parameters, 5 per pass) and are solved through the Schur complement.
## Returns {x, fun, jac_g [2 columns, loss-scaled], nfev}.
static func _lm(pb: Problem, x0: PackedFloat64Array, lb: PackedFloat64Array, ub: PackedFloat64Array, cancel: Callable) -> Dictionary:
	var x := x0.duplicate()
	var f := pb.residuals(x)
	var nfev := 1
	var cost := _cost(f)
	var mu := 1e-3
	var nf := 2 * pb.nb
	var npar := x.size()
	var jac: Array = []
	var w := PackedFloat64Array()
	var fs := PackedFloat64Array()
	while nfev < MAX_NFEV:
		if cancel.is_valid() and cancel.call():
			break
		jac = _jac(pb, x, f, lb, ub)
		# loss scaling
		w.resize(f.size())
		fs.resize(f.size())
		for i in f.size():
			var r := _rho(f[i])
			var js := sqrt(maxf(r[2], Num.EPS))
			w[i] = js
			fs[i] = f[i] * r[1] / js
		# normal equations
		var cg: Array = jac[0]
		var cp: Array = jac[1]
		var hgg := PackedFloat64Array([0, 0, 0])
		var gg := PackedFloat64Array([0, 0])
		var hkk: Array = []
		var hgk: Array = []
		var gk: Array = []
		for k in pb.npass:
			hkk.append(Num.zeros(25))
			hgk.append(Num.zeros(10))
			gk.append(Num.zeros(5))
		for i in nf:
			var j := i / 2
			var k: int = pb.seg[j]
			var wi := w[i]
			var a0: float = cg[0][i] * wi
			var a1: float = cg[1][i] * wi
			var fi := fs[i]
			hgg[0] += a0 * a0
			hgg[1] += a0 * a1
			hgg[2] += a1 * a1
			gg[0] += a0 * fi
			gg[1] += a1 * fi
			var e := PackedFloat64Array([cp[0][i] * wi, cp[1][i] * wi, cp[2][i] * wi, cp[3][i] * wi, cp[4][i] * wi])
			var H: PackedFloat64Array = hkk[k]
			var B: PackedFloat64Array = hgk[k]
			var gv: PackedFloat64Array = gk[k]
			for p in 5:
				for q in range(p, 5):
					H[5 * p + q] += e[p] * e[q]
				B[p] += a0 * e[p]
				B[5 + p] += a1 * e[p]
				gv[p] += e[p] * fi
		var o := nf
		for k in pb.npass:
			var H: PackedFloat64Array = hkk[k]
			var gv: PackedFloat64Array = gk[k]
			for q in 5:
				var row := o + (3 * k + q if q < 3 else 3 * pb.npass + 2 * k + q - 3)
				var dv := (0.5 / V0_SD if q < 3 else 0.5 / TILT_SD) * w[row]
				H[5 * q + q] += dv * dv
				gv[q] += dv * fs[row]
			for p in 5:
				for q in range(p + 1, 5):
					H[5 * q + p] = H[5 * p + q]
		# active bounds of the two global parameters: held where the descent points out of the box
		for g in 2:
			var at_lo := x[g] <= lb[g] and gg[g] > 0.0
			var at_hi := x[g] >= ub[g] and gg[g] < 0.0
			if at_lo or at_hi:
				gg[g] = 0.0
				if g == 0:
					hgg[0] = 1.0
					hgg[1] = 0.0
				else:
					hgg[2] = 1.0
					hgg[1] = 0.0
				for k in pb.npass:
					var B: PackedFloat64Array = hgk[k]
					for p in 5:
						B[5 * g + p] = 0.0
		var grad_max := maxf(absf(gg[0]), absf(gg[1]))
		for k in pb.npass:
			for v in gk[k]:
				grad_max = maxf(grad_max, absf(v))
		if grad_max < 1e-14:
			break
		var accepted := false
		while nfev < MAX_NFEV:
			var step := _solve_arrow(pb.npass, hgg, gg, hkk, hgk, gk, mu)
			if step.is_empty():
				mu *= 10.0
				continue
			var xn := x.duplicate()
			var dmax := 0.0
			for i in npar:
				xn[i] = clampf(x[i] + step[i], lb[i], ub[i])
				dmax = maxf(dmax, absf(xn[i] - x[i]) / maxf(absf(x[i]), 1e-3))
			var fn := pb.residuals(xn)
			nfev += 1
			var cn := _cost(fn)
			if cn < cost:
				var rel := (cost - cn) / cost
				x = xn
				f = fn
				cost = cn
				mu = maxf(mu / 3.0, 1e-12)
				accepted = true
				if rel < 1e-14 or dmax < 1e-13:
					nfev = MAX_NFEV
				break
			mu *= 4.0
			if dmax < 1e-13:
				nfev = MAX_NFEV
				break
		if not accepted:
			break
	jac = _jac(pb, x, f, lb, ub)
	var jg: Array = []
	for g in 2:
		var c := PackedFloat64Array()
		c.resize(nf)
		for i in nf:
			c[i] = jac[0][g][i] * sqrt(maxf(_rho(f[i])[2], Num.EPS))
		jg.append(c)
	return {"x": x, "fun": f, "jac_g": jg, "nfev": nfev}


static func _solve_arrow(np_: int, hgg: PackedFloat64Array, gg: PackedFloat64Array, hkk: Array, hgk: Array, gk: Array, mu: float) -> PackedFloat64Array:
	var S := PackedFloat64Array([hgg[0] * (1.0 + mu), hgg[1], hgg[1], hgg[2] * (1.0 + mu)])
	var r := PackedFloat64Array([-gg[0], -gg[1]])
	var minv: Array = []
	for k in np_:
		var H: PackedFloat64Array = hkk[k]
		var M: Array = []
		for p in 5:
			var row := PackedFloat64Array()
			for q in 5:
				row.append(H[5 * p + q] * (1.0 + mu) if p == q else H[5 * p + q])
			M.append(row)
		var B: PackedFloat64Array = hgk[k]
		var gv: PackedFloat64Array = gk[k]
		# M^-1 [B0^T, B1^T, g]
		var y0 := Num.solve(M, B.slice(0, 5))
		var y1 := Num.solve(M, B.slice(5, 10))
		var yg := Num.solve(M, gv)
		if y0.is_empty() or y1.is_empty() or yg.is_empty():
			return PackedFloat64Array()
		minv.append([y0, y1, yg])
		for p in 5:
			S[0] -= B[p] * y0[p]
			S[1] -= B[p] * y1[p]
			S[2] -= B[5 + p] * y0[p]
			S[3] -= B[5 + p] * y1[p]
			r[0] += B[p] * yg[p]
			r[1] += B[5 + p] * yg[p]
	var dg := Num.solve([[S[0], S[1]], [S[2], S[3]]], r)
	if dg.is_empty():
		return dg
	var out := PackedFloat64Array([dg[0], dg[1]])
	for k in np_:
		var m: Array = minv[k]
		for p in 5:
			out.append(-m[2][p] - m[0][p] * dg[0] - m[1][p] * dg[1])
	return out


## Drag scale and rotor drag, jointly with each pass's start velocity (3) and start tilt error (2).
static func fit(logs: Array, uses: Array, mass: float, shape: Array, sum_w_hover: float, sum_w2_hover: float, cancel: Callable = Callable()) -> Dictionary:
	var segs: Array = []
	for li in logs.size():
		var L: Flight = logs[li]
		for ab in passes(L, uses[li], sum_w2_hover):
			var blocks: Array = dead_reckon(L, ab[0], ab[1]).filter(func(b): return b[0] >= 0.4)
			if blocks.size() >= 4:
				segs.append([L.name, L.t[ab[0]], blocks])
	var npass := segs.size()
	var nb := 0
	var plist: Array = []
	for s in segs:
		nb += s[2].size()
		plist.append([s[0], Num.pyround(s[1], 3), s[2].size()])
	if npass < 3 or nb < 40:
		return {"ok": false, "n_pass": npass, "n": nb, "passes": plist}
	var pb := Problem.new()
	pb.nb = nb
	pb.npass = npass
	pb.mass = mass
	pb.shp = PackedFloat64Array(shape)
	pb.rd0 = 0.4 * mass / sum_w_hover
	for k in npass:
		for b in segs[k][2]:
			pb.T.append(b[0])
			pb.R.append_array(b[1])
			pb.I.append_array(b[2])
			pb.A.append(b[3])
			pb.A.append(b[4])
			pb.SW.append(b[5])
			pb.seg.append(k)
	var x0 := Num.zeros(2 + 5 * npass)
	x0[0] = 1.0
	x0[1] = 1.0
	var lb := PackedFloat64Array()
	var ub := PackedFloat64Array()
	lb.resize(x0.size())
	ub.resize(x0.size())
	lb.fill(-INF)
	ub.fill(INF)
	lb[0] = 0.02
	ub[0] = 20.0
	lb[1] = 0.0
	ub[1] = 10.0
	var r := _lm(pb, x0, lb, ub, cancel)
	var x: PackedFloat64Array = r.x
	var fun: PackedFloat64Array = r.fun
	var s := x[0]
	var rd := x[1] * pb.rd0
	var nfit := 2 * nb
	var rf := fun.slice(0, nfit)
	var rf2 := PackedFloat64Array()
	for v in rf:
		rf2.append(v * v)
	var mrf := Num.mean(rf2)
	var rel: Variant = null
	var jg: Array = r.jac_g
	var jtj := [[0.0, 0.0], [0.0, 0.0]]
	for i in nfit:
		var a: float = jg[0][i]
		var b: float = jg[1][i]
		jtj[0][0] += a * a
		jtj[0][1] += a * b
		jtj[1][0] += a * b
		jtj[1][1] += b * b
	var inv := Num.inverse(jtj)
	if not inv.is_empty():
		rel = [sqrt(inv[0][0] * mrf) / s, sqrt(inv[1][1] * mrf) / maxf(x[1], 1e-9)]
	pb.residuals(x)
	var sp := PackedFloat64Array()
	sp.resize(nb)
	for j in nb:
		var b0 := pb.vb[3 * j]
		var b1 := pb.vb[3 * j + 1]
		var b2 := pb.vb[3 * j + 2]
		sp[j] = sqrt(b0 * b0 + b1 * b1 + b2 * b2)
	var ma := [0.0, 0.0]
	for j in nb:
		ma[0] += pb.A[2 * j]
		ma[1] += pb.A[2 * j + 1]
	ma[0] /= nb
	ma[1] /= nb
	var dv := PackedFloat64Array()
	for j in nb:
		dv.append((pb.A[2 * j] - ma[0]) * (pb.A[2 * j] - ma[0]))
		dv.append((pb.A[2 * j + 1] - ma[1]) * (pb.A[2 * j + 1] - ma[1]))
	var r2 := 1.0 - mrf / Num.mean(dv)
	var kmax := 0
	for j in nb:
		if sp[j] > sp[kmax]:
			kmax = j
	var pv := PackedFloat64Array()
	var pt := PackedFloat64Array()
	for k in npass:
		for q in 3:
			pv.append(x[2 + 5 * k + q] * x[2 + 5 * k + q])
		for q in 2:
			pt.append(x[2 + 5 * k + 3 + q] * x[2 + 5 * k + 3 + q])
	var sseg: Array = segs[pb.seg[kmax]]
	return {"ok": true, "scale": s, "rotor_drag": rd, "rotor_drag_per_s": rd * sum_w_hover / mass,
		"rel_sd": rel, "rms_g": sqrt(mrf), "r2": r2, "axy_std_g": Num.std(pb.A),
		"n": nb, "n_pass": npass,
		"v_pct": [Num.pyround(Num.percentile(sp, 50), 1), Num.pyround(Num.percentile(sp, 90), 1), Num.pyround(Num.percentile(sp, 99), 1)],
		"vmax_dr": sp[kmax], "vmax_where": "%s %.1f s" % [sseg[0], float(sseg[1]) + pb.T[kmax]],
		"v0_rms": sqrt(Num.mean(pv)), "tilt_rms_deg": rad_to_deg(sqrt(Num.mean(pt))),
		"passes": plist, "nfev": r.nfev}


# ---------- top speed of the plant (steady level flight, full throttle) ----------

static func _mean4(a: Array) -> float:
	return (((float(a[0]) + float(a[1])) + float(a[2])) + float(a[3])) / 4.0


## [w, thrust of one rotor]
static func _rotor(p: Dictionary, u: float, v_ax: float, V: float) -> PackedFloat64Array:
	var a := _mean4(p.motor_a)
	var b := _mean4(p.motor_b)
	var c := _mean4(p.motor_c)
	var kv: float = p.kv_rad
	var pm: float = p.pitch_m_per_rad
	var w := 2000.0
	for _it in 200:
		var load := minf(maxf(1.0 - v_ax / maxf(w * pm, 1.0), 0.15), 1.6)
		var f := a * (u * V - w / kv) - b * load * w * w - c
		var wn := maxf(w + f / (a / kv + 2.0 * b * load * w) * 0.5, 1.0)
		if wn == w:
			break
		w = wn
	var load2 := minf(maxf(1.0 - v_ax / maxf(w * pm, 1.0), 0.15), 1.6)
	return PackedFloat64Array([w, float(p.kT) * w * w * load2])


## [Fx, Fz, w]
static func _forces(p: Dictionary, v: float, th: float, V: float) -> PackedFloat64Array:
	var vb := [v * cos(th), 0.0, v * sin(th)]
	var rw := _rotor(p, 1.0, vb[2], V)
	var w := rw[0]
	var cd: Array = p.cd_area
	var nv := sqrt(vb[0] * vb[0] + vb[1] * vb[1] + vb[2] * vb[2])
	var fd := [-0.5 * RHO * nv * float(cd[0]) * vb[0], -0.5 * RHO * nv * float(cd[1]) * vb[1], -0.5 * RHO * nv * float(cd[2]) * vb[2]]
	fd[0] += -float(p.rotor_drag) * 4.0 * w * vb[0]
	fd[1] += -float(p.rotor_drag) * 4.0 * w * vb[1]
	var F := [fd[0], fd[1], 4.0 * rw[1] + fd[2]]
	var fx: float = F[0] * cos(th) + F[2] * sin(th)
	var fz: float = -F[0] * sin(th) + F[2] * cos(th) - float(p.mass) * G
	return PackedFloat64Array([fx, fz, w])


## [km/h, tilt deg, rotor Hz, V] for level flight at full throttle (fresh pack with the sag).
static func top_speed(p: Dictionary) -> Array:
	var v0: float = p.battery_v0
	var V := v0
	for _it in 5:
		var w := _rotor(p, 1.0, 0.0, V)[0]
		V = v0 - float(p.battery_r) * 4.0 * maxf(V - w / float(p.kv_rad), 0.0)
	var v_eq := func(th: float) -> float:
		var lo := 0.0
		var hi := 150.0
		for _i in 50:
			var mid := (lo + hi) / 2.0
			if _forces(p, mid, th, V)[0] > 0.0:
				lo = mid
			else:
				hi = mid
		return lo
	var lo := deg_to_rad(5.0)
	var hi := deg_to_rad(89.5)
	var th := 0.0
	for _i in 50:
		th = (lo + hi) / 2.0
		var v: float = v_eq.call(th)
		if _forces(p, v, th, V)[1] > 0.0:
			lo = th
		else:
			hi = th
	var vv: float = v_eq.call(th)
	var f := _forces(p, vv, th, V)
	return [vv * 3.6, rad_to_deg(th), f[2] / TAU, V]


## Body drag scale so the plant tops out at target_kmh (rotor drag kept).
static func scale_for_top_speed(p: Dictionary, target_kmh: float, shape_ref: Array) -> float:
	var q := p.duplicate()
	var lo := 0.05
	var hi := 20.0
	var mid := 1.0
	for _i in 40:
		mid = sqrt(lo * hi)
		q.cd_area = shape_ref.map(func(x): return float(x) * mid)
		if top_speed(q)[0] > target_kmh:
			lo = mid
		else:
			hi = mid
	return mid
