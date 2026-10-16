extends RefCounted
## real_quad logfit: rotational dynamics, angular acceleration vs rotor speeds (instrumental-variable
## regression):
##     roll : dp/dt = k_r * 1e-6 * (W2 + W3 - W0 - W1)          + d_r * 1e-3 * p * S
##     pitch: dq/dt = k_p * 1e-6 * (W0 + W2 - W1 - W3)          + d_p * 1e-3 * q * S
##     yaw  : dr/dt = g_y * 1e-3 * sum_i s_i (b_i W_i + dw_i/dt) + d_y * 1e-3 * r * S
## Wi = wi^2 from the motor model's free run, S = sqrt(sum Wi), s = [1, -1, -1, 1]; g_y = -s0 *
## J_rotor / Izz. The loop is closed, so the instruments are the setpoint and its derivative (external
## to the loop). Chirp windows are the best-conditioned data; freestyle is the fallback.

const Num := preload("num.gd")
const Flight := preload("flight.gd")
const Report := preload("report.gd")

const D2R := PI / 180.0
const AX := ["roll", "pitch", "yaw"]
const SPIN := [1.0, -1.0, -1.0, 1.0]
const BAND := [1.0, 30.0]
const YAW_MIN_R2 := 0.5


## One entry per usable window: {reg {axis: [x0, x1]}, z {axis: [z0, z1]}, y [3], m mask}; reg and z only of
## `axes` (the mask m always checks all three).
static func rows_for(L: Flight, wsim: Array, b: PackedFloat64Array, mask: PackedByteArray, lo: float, hi: float, min_len_s: float = 3.0, edge_s: float = 0.3, axes: Array = AX) -> Array:
	var fs := L.fs
	var n := L.n
	var ok := Num.zeros_mask(n)
	for i in n:
		if mask[i] == 0 or L.armed[i] == 0:
			continue
		var good := true
		for q in 4:
			if not (L.motor_hz[q][i] > 40.0):
				good = false
				break
		if not good:
			continue
		for a in 3:
			if not (absf(L.gyro_unfilt[a][i]) < 1900.0):
				good = false
				break
		if not good:
			continue
		if i > 0 and not (L.t[i] - L.t[i - 1] < 3.0 / fs):
			continue
		ok[i] = 1
	var out: Array = []
	var edge := int(edge_s * fs)
	var lp80 := Num.butter(4, 80.0, "lowpass", fs)
	var lp20 := Num.butter(4, 20.0, "lowpass", fs)
	var lps := Num.butter(4, minf(30.0, 0.4 * hi + 24.0), "lowpass", fs)
	var bp := Num.butter(2, [lo, hi], "bandpass", fs)
	var h := 1.0 / fs
	var r := Num.runs(ok)
	for j in range(0, r.size(), 2):
		var a := r[j]
		var e := r[j + 1]
		if e - a < min_len_s * fs:
			continue
		var nn := e - a
		var gs: Array = []
		var gd: Array = []
		var s: Array = []
		var sd: Array = []
		for k in 3:
			var g := (L.gyro_unfilt[k] as PackedFloat64Array).slice(a, e)
			var sp := (L.setpoint[k] as PackedFloat64Array).slice(a, e)
			for i in nn:
				g[i] = g[i] * D2R
				sp[i] = sp[i] * D2R
			var gf := Num.sosfiltfilt(lp80, g)
			gs.append(gf)
			gd.append(Num.gradient(gf, h))
			var sf := Num.sosfiltfilt(lps, sp)
			s.append(sf)
			sd.append(Num.gradient(sf, h))
		var W2: Array = []
		var wd: Array = []
		for q in 4:
			var w := (wsim[q] as PackedFloat64Array).slice(a, e)
			var w2 := PackedFloat64Array()
			w2.resize(nn)
			for i in nn:
				w2[i] = w[i] * w[i]
			W2.append(w2)
			wd.append(Num.gradient(w, h))
		var S := PackedFloat64Array()
		var xr := PackedFloat64Array()
		var xp := PackedFloat64Array()
		var xq := PackedFloat64Array()
		S.resize(nn)
		xr.resize(nn)
		xp.resize(nn)
		xq.resize(nn)
		for i in nn:
			var v0: float = W2[0][i]
			var v1: float = W2[1][i]
			var v2: float = W2[2][i]
			var v3: float = W2[3][i]
			S[i] = sqrt(v0 + v1 + v2 + v3)
			xr[i] = v2 + v3 - v0 - v1
			xp[i] = v0 + v2 - v1 - v3
			var q := 0.0
			for m in 4:
				q += SPIN[m] * (b[m] * W2[m][i] + wd[m][i])
			xq[i] = q
		var reg := {}
		var z := {}
		var xs := [xr, xp, xq]
		var sc := [1e-6, 1e-6, 1e-3]
		for k in 3:
			if not AX[k] in axes:
				continue
			var c0 := PackedFloat64Array()
			var c1 := PackedFloat64Array()
			var z1 := PackedFloat64Array()
			c0.resize(nn)
			c1.resize(nn)
			z1.resize(nn)
			var xk: PackedFloat64Array = xs[k]
			var gk: PackedFloat64Array = gs[k]
			var sk: PackedFloat64Array = s[k]
			for i in nn:
				c0[i] = xk[i] * sc[k]
				c1[i] = gk[i] * S[i] * 1e-3
				z1[i] = sk[i] * S[i] * 1e-3
			reg[AX[k]] = [Num.sosfiltfilt(bp, c0), Num.sosfiltfilt(bp, c1)]
			z[AX[k]] = [Num.sosfiltfilt(bp, sd[k]), Num.sosfiltfilt(bp, z1)]
		var y: Array = []
		var m := PackedByteArray()
		m.resize(nn)
		m.fill(1)
		for i in mini(edge, nn):
			m[i] = 0
		for i in range(maxi(nn - edge, 0), nn):
			m[i] = 0
		for k in 3:
			y.append(Num.sosfiltfilt(bp, gd[k]))
			var g20 := Num.sosfiltfilt(lp20, gd[k])
			for i in nn:
				if not (absf(g20[i]) < 150.0):
					m[i] = 0
		out.append({"reg": reg, "z": z, "y": y, "m": m})
	return out


static func _solve2(a00: float, a01: float, a10: float, a11: float, b0: float, b1: float) -> PackedFloat64Array:
	return Num.solve([[a00, a01], [a10, a11]], PackedFloat64Array([b0, b1]))


## Per window: [ZtX (4), Zty (2)] for axis k.
static func _window_sums(r: Dictionary, ax: String, k: int) -> PackedFloat64Array:
	var X: Array = r.reg[ax]
	var Z: Array = r.z[ax]
	var y: PackedFloat64Array = r.y[k]
	var m: PackedByteArray = r.m
	var x0: PackedFloat64Array = X[0]
	var x1: PackedFloat64Array = X[1]
	var z0: PackedFloat64Array = Z[0]
	var z1: PackedFloat64Array = Z[1]
	var s := PackedFloat64Array([0, 0, 0, 0, 0, 0])
	for i in m.size():
		if m[i] == 0:
			continue
		s[0] += z0[i] * x0[i]
		s[1] += z0[i] * x1[i]
		s[2] += z1[i] * x0[i]
		s[3] += z1[i] * x1[i]
		s[4] += z0[i] * y[i]
		s[5] += z1[i] * y[i]
	return s


## Per axis of `axes`: {coef, r2, n, windows, se, blocks, spread_blocks}. An axis without excitation (its
## instruments all zero, e.g. no yaw stick at all: a singular system) or without data is left out.
static func iv_fit(rows: Array, axes: Array = AX) -> Dictionary:
	var res := {}
	for k in 3:
		var ax: String = AX[k]
		if not ax in axes:
			continue
		var sel: Array = rows.filter(func(r): return (r.m as PackedByteArray).count(1) > 100)
		if sel.is_empty():
			continue
		var sums: Array = sel.map(func(r): return _window_sums(r, ax, k))
		var tot := PackedFloat64Array([0, 0, 0, 0, 0, 0])
		for s in sums:
			for q in 6:
				tot[q] += s[q]
		var c := _solve2(tot[0], tot[1], tot[2], tot[3], tot[4], tot[5])
		if c.is_empty() or not (is_finite(c[0]) and is_finite(c[1])):
			continue
		# concatenated data for R2 and the 4 blocks
		var Xc0 := PackedFloat64Array()
		var Xc1 := PackedFloat64Array()
		var Zc0 := PackedFloat64Array()
		var Zc1 := PackedFloat64Array()
		var Yc := PackedFloat64Array()
		for r in sel:
			var m: PackedByteArray = r.m
			for i in m.size():
				if m[i] != 0:
					Xc0.append(r.reg[ax][0][i])
					Xc1.append(r.reg[ax][1][i])
					Zc0.append(r.z[ax][0][i])
					Zc1.append(r.z[ax][1][i])
					Yc.append(r.y[k][i])
		var nn := Yc.size()
		var resid := PackedFloat64Array()
		resid.resize(nn)
		for i in nn:
			resid[i] = Yc[i] - (Xc0[i] * c[0] + Xc1[i] * c[1])
		var r2 := 1.0 - Num.variance(resid) / Num.variance(Yc)
		var bl := nn / 4
		var blocks := PackedFloat64Array()
		for j in 4:
			var s := PackedFloat64Array([0, 0, 0, 0, 0, 0])
			for i in range(j * bl, (j + 1) * bl):
				s[0] += Zc0[i] * Xc0[i]
				s[1] += Zc0[i] * Xc1[i]
				s[2] += Zc1[i] * Xc0[i]
				s[3] += Zc1[i] * Xc1[i]
				s[4] += Zc0[i] * Yc[i]
				s[5] += Zc1[i] * Yc[i]
			var cb := _solve2(s[0], s[1], s[2], s[3], s[4], s[5])
			if not cb.is_empty():
				blocks.append(cb[0])
		var jk := PackedFloat64Array()
		for s in sums:
			var cj := _solve2(tot[0] - s[0], tot[1] - s[1], tot[2] - s[2], tot[3] - s[3], tot[4] - s[4], tot[5] - s[5])
			if not cj.is_empty():
				jk.append(cj[0])
		var nj := jk.size()
		var se: Variant = null
		if nj > 1:
			var mj := Num.mean(jk)
			var ss := PackedFloat64Array()
			for v in jk:
				ss.append((v - mj) * (v - mj))
			se = sqrt(float(nj - 1) / nj * Num.psum(ss)) / maxf(absf(c[0]), 1e-9)
		res[ax] = {"coef": [c[0], c[1]], "r2": r2, "n": nn, "windows": sel.size(), "se": se, "blocks": Array(blocks),
			"spread_blocks": Num.std(blocks) / maxf(absf(c[0]), 1e-9) if blocks.size() > 1 else null}
	return res


const YAW_MAX_SE := 0.25


## The yaw entry when the yaw could not be fitted at all (no yaw excitation / no data).
static func no_fit() -> Dictionary:
	return {"coef": [0.0, 0.0], "r2": 0.0, "n": 0, "windows": 0, "se": null, "blocks": [], "spread_blocks": null}


## Sample masks per session for the fit: [roll/pitch masks, yaw masks or null = the same, yaw source]. With
## >= 30 s of chirps roll and pitch use the chirp windows; yaw too if a yaw sweep is among them, else the
## freestyle rest (chirps on roll/pitch only hardly move yaw: its gain and sign would be noise).
static func masks(uses: Array, chirp_masks: Array, segs: Array, use_chirp: bool) -> Array:
	var free: Array = []
	var chp: Array = []
	for k in uses.size():
		free.append(Num.mask_and(uses[k], Num.mask_not(chirp_masks[k])))
		chp.append(Num.mask_and(uses[k], chirp_masks[k]))
	if not use_chirp:
		return [free, null, "freestyle"]
	for s in segs:
		for ch in s.chirps:
			if ch.size() > 2 and ch[2] != null and int(ch[2]) == 2:
				return [chp, null, "chirp"]
	return [chp, free, "freestyle"]


## The IV fit of all axes on the masks of masks(): [fit, windows of the roll/pitch fit, windows of the yaw
## fit, yaw source]; an axis that could not be fitted is missing from fit. Empty if cancelled.
static func fit_all(logs: Array, sims: Array, b: PackedFloat64Array, uses: Array, chirp_masks: Array, segs: Array, use_chirp: bool,
		band: Array, cancel: Callable = Callable()) -> Array:
	var mk := masks(uses, chirp_masks, segs, use_chirp)
	var both: bool = mk[1] == null
	var rr: Array = []
	for k in logs.size():
		rr.append_array(rows_for(logs[k], sims[k], b, mk[0][k], band[0], band[1], 3.0, 0.3, AX if both else AX.slice(0, 2)))
		if cancel.is_valid() and cancel.call():
			return []
	var fit := iv_fit(rr, AX if both else AX.slice(0, 2))
	var n_rp := rr.size()
	var n_y := n_rp
	if not both:
		rr = []
		for k in logs.size():
			rr.append_array(rows_for(logs[k], sims[k], b, mk[1][k], band[0], band[1], 3.0, 0.3, AX.slice(2)))
			if cancel.is_valid() and cancel.call():
				return []
		fit.merge(iv_fit(rr, AX.slice(2)), true)
		n_y = rr.size()
	return [fit, n_rp, n_y, mk[2]]


## The yaw fit is good enough for J_rotor and the motor direction: R2 >= 0.5 and jackknife rel. SE <= 25 %.
static func yaw_measured(fy: Dictionary) -> bool:
	return float(fy.r2) >= YAW_MIN_R2 and fy.se != null and float(fy.se) <= YAW_MAX_SE and float(fy.coef[0]) != 0.0


## Roll/pitch inertia / class inertia outside this: refused (the class band, Classes.BANDS.I, only warns).
const INERTIA_HARD := [0.1, 10.0]


## "" if the roll and pitch gains give a physical inertia, else why not: a gain that is not finite and positive
## (negative / infinite inertia) or an inertia more than a factor 10 from the class inertia i0 (the fit is garbage).
static func inertia_problem(fit: Dictionary, kT: float, arm: float, i0: float) -> String:
	for ax in AX.slice(0, 2):
		var g := float(fit[ax].coef[0])
		if is_nan(g) or is_inf(g) or g <= 0.0:
			return "%s gain %s is not positive: the %s inertia would not be physical" % [ax, Report.fmt_g(g, 3), ax]
		var r := kT * arm / (g * 1e-6) / i0
		if not (INERTIA_HARD[0] <= r and r <= INERTIA_HARD[1]):
			return "%s inertia is %s times the class inertia (plausible: %s to %s)" % [ax, Report.fmt_g(r, 2), Report.fmt_g(INERTIA_HARD[0], 3), Report.fmt_g(INERTIA_HARD[1], 3)]
	return ""


## IV coefficients -> plant parameters: [params {Ixx, Iyy, Izz, J_rotor, rate_damping}, yaw_reversed, yaw_ok].
## Size and direction of the yaw only if the yaw fit is good (yaw_measured()), else J_rotor = j_default and
## the direction rev_default. yaw_damping (-1: as use_damping, 0 / 1): the yaw damping term is used.
static func to_params(fit: Dictionary, kT: float, arm: float, izz_over_ixy: float, use_damping: bool, j_default: float,
		rev_default: bool = true, yaw_damping: int = -1) -> Array:
	var out := {}
	out.Ixx = kT * arm / (float(fit.roll.coef[0]) * 1e-6)
	out.Iyy = kT * arm / (float(fit.pitch.coef[0]) * 1e-6)
	out.Izz = izz_over_ixy * 0.5 * (out.Ixx + out.Iyy)
	var fy: Dictionary = fit.get("yaw", no_fit())
	var gy := float(fy.coef[0]) * 1e-3
	var yaw_ok := yaw_measured(fy)
	var rev := gy < 0.0 if yaw_ok else rev_default
	out.J_rotor = absf(gy) * out.Izz if yaw_ok else j_default
	var yd := use_damping if yaw_damping < 0 else yaw_damping == 1
	out.rate_damping = [maxf(0.0, -float(fit.roll.coef[1])) if use_damping else 0.0,
		maxf(0.0, -float(fit.pitch.coef[1])) if use_damping else 0.0, maxf(0.0, -float(fy.coef[1])) if yd else 0.0]
	return [out, rev, yaw_ok]
