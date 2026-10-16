extends RefCounted
## real_quad logfit: usable airborne windows and chirp windows of a session (contract 3.6).
##   ground   throttle < 0.5 x hover, |acc| ~ 1 g and a quiet gyro for >= 0.3 s (without ACC: quiet
##            gyro at low throttle for >= 0.3 s touching the log start/end, or >= 2 s); upside down
##   crash    motor stall (one rotor < 10 % of the median of the other three for > 50 ms with throttle
##            > 0.5 x hover), lateral specific force > 3 g or |a| spike > 4 g at low throttle, a
##            gyroUnfilt jump > 500 deg/s between two samples, an eRPM glitch; +-0.3 s margins
##   gaps     time step > 3x the median step. Gaps less than 2 s apart form one block (a corrupted stretch
##            of the log); the block is dropped as a whole, from its first gap to its last plus 1 s on both
##            sides; the last 0.3 s
## Windows shorter than 1 s are dropped; chirp windows must be >= 95 % usable.

const Num := preload("num.gd")
const Flight := preload("flight.gd")

const MARGIN := 0.3
const MIN_WINDOW := 1.0
const GAP_MERGE_S := 2.0             # gaps closer than this belong to one corrupted block
const GAP_MARGIN_S := 1.0            # dropped before the first and after the last gap of a block
const STALL_FRAC := 0.10
const STALL_S := 0.05


static func mask_from_windows(t: PackedFloat64Array, windows: Array) -> PackedByteArray:
	var m := Num.zeros_mask(t.size())
	for w in windows:
		var a := Num.lower(t, float(w[0]))
		var b := Num.lower(t, float(w[1]))
		for i in range(a, b):
			m[i] = 1
	return m


static func windows_from_mask(t: PackedFloat64Array, mask: PackedByteArray, min_len: float = MIN_WINDOW) -> Array:
	var out: Array = []
	var r := Num.runs(mask)
	for j in range(0, r.size(), 2):
		var t0 := t[r[j]]
		var t1 := t[r[j + 1] - 1]
		if t1 - t0 >= min_len:
			out.append([Num.pyround(t0, 2), Num.pyround(t1, 2)])
	return out


static func norm3(c: Array, i: int) -> float:
	var x: float = c[0][i]
	var y: float = c[1][i]
	var z: float = c[2][i]
	return sqrt(x * x + y * y + z * z)


static func norm_col(c: Array) -> PackedFloat64Array:
	var n: int = (c[0] as PackedFloat64Array).size()
	var o := PackedFloat64Array()
	o.resize(n)
	var x: PackedFloat64Array = c[0]
	var y: PackedFloat64Array = c[1]
	var z: PackedFloat64Array = c[2]
	for i in n:
		o[i] = sqrt(x[i] * x[i] + y[i] * y[i] + z[i] * z[i])
	return o


static func max_abs_rows(c: Array) -> PackedFloat64Array:
	var n: int = (c[0] as PackedFloat64Array).size()
	var o := PackedFloat64Array()
	o.resize(n)
	o.fill(0.0)
	for col: PackedFloat64Array in c:
		for i in n:
			o[i] = maxf(o[i], absf(col[i]))
	return o


static func rough_hover_throttle(L: Flight) -> float:
	var thr := PackedFloat64Array()
	var an := PackedFloat64Array()
	for i in L.n:
		if L.armed[i] == 0 or L.throttle[i] <= 0.08:
			continue
		if absf(L.gyro[0][i]) >= 60.0 or absf(L.gyro[1][i]) >= 60.0 or absf(L.gyro[2][i]) >= 60.0:
			continue
		if L.has_acc():
			var a := norm3(L.acc, i)
			if not (absf(a - 1.0) < 0.08 and L.acc[2][i] > 0.97 * a):
				continue
		thr.append(L.throttle[i])
	if thr.size() > 100:
		return Num.median(thr)
	return Num.median(Num.select(L.throttle, L.armed))


static func _sum_w2(L: Flight) -> PackedFloat64Array:
	var o := PackedFloat64Array()
	o.resize(L.n)
	var k := 2.0 * PI
	for i in L.n:
		var s := 0.0
		for m in 4:
			var v: float = L.motor_hz[m][i]
			if is_nan(v):
				v = 0.0
			var w := k * v
			s += w * w
		o[i] = s
	return o


static func motor_stall(L: Flight, hover_thr: float) -> PackedByteArray:
	var out := Num.zeros_mask(L.n)
	if not L.has_hz():
		return out
	var cond := Num.zeros_mask(L.n)
	var h := PackedFloat64Array([0, 0, 0, 0])
	for i in L.n:
		if not (L.throttle[i] > 0.5 * hover_thr and L.armed[i] != 0):
			continue
		for m in 4:
			var v: float = L.motor_hz[m][i]
			h[m] = 0.0 if is_nan(v) else v
		for m in 4:
			var o := PackedFloat64Array()
			for q in 4:
				if q != m:
					o.append(h[q])
			o.sort()
			if h[m] < STALL_FRAC * o[1]:
				cond[i] = 1
				break
	var r := Num.runs(cond)
	for j in range(0, r.size(), 2):
		if L.t[r[j + 1] - 1] - L.t[r[j]] >= STALL_S:
			for i in range(r[j], r[j + 1]):
				out[i] = 1
	return out


static func erpm_glitch(L: Flight, limit_hz: float) -> PackedByteArray:
	var out := Num.zeros_mask(L.n)
	if not L.has_hz():
		return out
	for i in L.n:
		for m in 4:
			var v: float = L.motor_hz[m][i]
			if v > limit_hz or v < -1.0:
				out[i] = 1
				break
	return out


static func _dilate(t: PackedFloat64Array, mask: PackedByteArray, before: float, after: float) -> PackedByteArray:
	var out := mask.duplicate()
	var r := Num.runs(mask)
	for j in range(0, r.size(), 2):
		var a := Num.lower(t, t[r[j]] - before)
		var b := Num.upper(t, t[r[j + 1] - 1] + after)
		for i in range(a, b):
			out[i] = 1
	return out


## [blocks, n_gaps]: the stretches [t0, t1] to drop around gaps (time step > 3x the median step). Gaps
## closer than GAP_MERGE_S (end of one to start of the next) make one block, which runs from the start of
## its first gap to the end of its last, plus GAP_MARGIN_S on both sides.
static func gap_blocks(t: PackedFloat64Array) -> Array:
	var n := t.size()
	var blocks: Array = []
	var ngaps := 0
	if n < 2:
		return [blocks, 0]
	var dt := PackedFloat64Array()
	dt.resize(n - 1)
	for i in n - 1:
		dt[i] = t[i + 1] - t[i]
	var lim := 3.0 * Num.median(dt)
	for i in n - 1:
		if dt[i] > lim:
			ngaps += 1
			var a := t[i]
			var b := t[i + 1]
			if not blocks.is_empty() and a - blocks[-1][1] < GAP_MERGE_S:
				blocks[-1][1] = b
			else:
				blocks.append([a, b])
	for blk in blocks:
		blk[0] -= GAP_MARGIN_S
		blk[1] += GAP_MARGIN_S
	return [blocks, ngaps]


static func _nruns(mask: PackedByteArray) -> int:
	return Num.runs(mask).size() / 2


## [use windows, info]
static func auto_segments(L: Flight, hover_thr: float, erpm_limit_hz: float) -> Array:
	var t := L.t
	var n := L.n
	var fs := L.fs
	var info := {}
	var gb := gap_blocks(t)
	var gap := Num.zeros_mask(n)
	for blk in gb[0]:
		for j in range(Num.lower(t, blk[0]), Num.upper(t, blk[1])):
			gap[j] = 1
	info.gaps = gb[1]
	info.gap_blocks = gb[0].size()
	var gap_s := 0.0
	for blk in gb[0]:
		gap_s += blk[1] - blk[0]
	info.gap_s = Num.pyround(gap_s, 1)
	var k := maxi(3, int(0.05 * fs))
	var gmax := Num.uniform_filter1d(max_abs_rows(L.gyro), k)
	var w2 := _sum_w2(L) if L.has_hz() else PackedFloat64Array()
	var ground := Num.zeros_mask(n)
	var thr := L.throttle
	if L.has_acc():
		var an := Num.uniform_filter1d(norm_col(L.acc), k)
		var az: PackedFloat64Array = L.acc[2]
		var hov := PackedFloat64Array()
		for i in n:
			if absf(an[i] - 1.0) < 0.08 and az[i] > 0.97 * an[i] and gmax[i] < 25.0 and absf(thr[i] - hover_thr) < 0.15 * hover_thr:
				hov.append(w2[i] if not w2.is_empty() else 0.0)
		var use_w2 := not w2.is_empty() and hov.size() > 200
		var w2h := Num.median(hov) if use_w2 else 1.0
		var azs := Num.uniform_filter1d(az, k)
		var cand := Num.zeros_mask(n)
		for i in n:
			var tf: float = w2[i] / w2h if use_w2 else thr[i] / hover_thr
			var one_g := absf(an[i] - 1.0) < 0.08
			var c := one_g and gmax[i] < 25.0 and (thr[i] < 0.5 * hover_thr or tf < 0.6)
			c = c or (azs[i] < -0.5 and absf(an[i] - 1.0) < 0.25 and thr[i] < 0.5 * hover_thr)
			cand[i] = 1 if c else 0
		var r := Num.runs(cand)
		for j in range(0, r.size(), 2):
			if t[r[j + 1] - 1] - t[r[j]] >= 0.3:
				for i in range(r[j], r[j + 1]):
					ground[i] = 1
	else:
		var cand := Num.zeros_mask(n)
		for i in n:
			cand[i] = 1 if thr[i] < 0.5 * hover_thr and gmax[i] < 8.0 else 0
		var r := Num.runs(cand)
		for j in range(0, r.size(), 2):
			var a := r[j]
			var b := r[j + 1]
			var dur := t[b - 1] - t[a]
			if (dur >= 0.3 and (a == 0 or b >= n - 1)) or dur >= 2.0:
				for i in range(a, b):
					ground[i] = 1
	info.ground_s = Num.pyround(ground.count(1) / fs, 1)
	ground = _dilate(t, ground, MARGIN, MARGIN)
	# crashes, impacts, telemetry glitches
	var stall := motor_stall(L, hover_thr)
	info.stalls = _nruns(stall)
	var hit := Num.zeros_mask(n)
	var gu := L.gyro_unfilt
	for i in range(1, n):
		var m := 0.0
		for a in 3:
			m = maxf(m, absf(gu[a][i] - gu[a][i - 1]))
		if m > 500.0:
			hit[i] = 1
	if L.has_acc():
		var ks := maxi(3, int(0.5 * fs) | 1)
		var ax: PackedFloat64Array = L.acc[0]
		var ay: PackedFloat64Array = L.acc[1]
		var lat := PackedFloat64Array()
		lat.resize(n)
		for i in n:
			lat[i] = sqrt(ax[i] * ax[i] + ay[i] * ay[i])
		var an2 := norm_col(L.acc)
		for i in n:
			if thr[i] >= 1.3 * hover_thr:
				continue
			# an impact is a short spike; a hard brake from top speed gives > 3 g of smooth drag
			if lat[i] > 3.0 and lat[i] - Num.median_at(lat, i, ks) > 2.5:
				hit[i] = 1
			elif an2[i] > 4.0 and an2[i] - Num.median_at(an2, i, ks) > 4.0:
				hit[i] = 1
	var glitch := erpm_glitch(L, erpm_limit_hz)
	info.erpm_glitches = _nruns(glitch)
	for i in n:
		if glitch[i] != 0 or stall[i] != 0:
			hit[i] = 1
	info.impacts = _nruns(hit)
	var crash := _dilate(t, hit, MARGIN, MARGIN)
	info.crash_s = Num.pyround(crash.count(1) / fs, 1)
	var use := Num.zeros_mask(n)
	var tend := t[n - 1] - MARGIN
	for i in n:
		if L.armed[i] != 0 and gap[i] == 0 and ground[i] == 0 and crash[i] == 0 and not (t[i] > tend):
			use[i] = 1
	return [windows_from_mask(t, use), info]


## [start, duration, axis] for every CHIRP switch activation (the sweep part only, max sweep_s).
static func chirp_windows(L: Flight) -> Array:
	var sweep_s := float(L.hint("chirp_time_seconds", 20))
	if sweep_s == 0.0:
		sweep_s = 20.0
	var out: Array = []
	var sos := Num.butter(2, 8.0, "highpass", L.fs)
	var m := Num.mask_and(L.chirp, L.armed)
	var r := Num.runs(m)
	for j in range(0, r.size(), 2):
		var t0 := L.t[r[j]]
		var dur := minf(L.t[r[j + 1] - 1] - t0, sweep_s + 0.5)
		if dur < 3.0:
			continue
		var s := L.sl(t0, t0 + dur)
		var best := 0
		var bsd := -1.0
		for a in 3:
			var x: PackedFloat64Array = (L.setpoint[a] as PackedFloat64Array).slice(s[0], s[1])
			if x.size() > 50:
				x = Num.sosfiltfilt(sos, x)
			var sd := Num.std(x)
			if sd > bsd:
				bsd = sd
				best = a
		out.append([Num.pyround(t0, 2), Num.pyround(dur, 2), best])
	return out


## {use, chirps, source, auto, auto_info, chirps_auto}
static func segment_log(L: Flight, erpm_limit_hz: float) -> Dictionary:
	var au := auto_segments(L, rough_hover_throttle(L), erpm_limit_hz)
	var ch := chirp_windows(L)
	var res := {"auto": au[0], "auto_info": au[1], "chirps_auto": ch, "use": au[0], "source": "auto"}
	var um := mask_from_windows(L.t, au[0])
	var keep: Array = []
	for c in ch:
		var s := L.sl(c[0], c[0] + c[1])
		var k := 0
		for i in range(s[0], s[1]):
			k += um[i]
		if s[1] > s[0] and float(k) / (s[1] - s[0]) > 0.95:
			keep.append(c)
	res.chirps = keep
	return res
