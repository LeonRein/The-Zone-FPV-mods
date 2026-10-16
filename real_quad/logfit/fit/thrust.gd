extends RefCounted
## real_quad logfit: hover and the thrust coefficient kT = m g / sum(w^2) at hover (mass from the form).
##   with ACC   |a| = 1 g +-0.05, level (a_z > 0.97 |a|), quiet gyro and steady throttle over 1 s: the
##              quad neither accelerates nor tilts, so thrust = weight
##   without    quiet gyro and steady throttle over 1 s only (climbs and cruise mixed in: an estimate)

const Num := preload("num.gd")
const Flight := preload("flight.gd")
const Segments := preload("segments.gd")

const G := 9.81


## cancel (optional): polled every 10 000 samples; when it answers true the result is incomplete (the caller stops).
static func hover_mask(L: Flight, use: PackedByteArray, with_acc: bool = true, cancel: Callable = Callable()) -> PackedByteArray:
	var k := maxi(3, int(1.0 * L.fs))
	var gq := Num.uniform_filter1d(Segments.max_abs_rows(L.gyro), k)
	var t2 := PackedFloat64Array()
	t2.resize(L.n)
	for i in L.n:
		t2[i] = L.throttle[i] * L.throttle[i]
	var m2 := Num.uniform_filter1d(t2, k)
	var m1 := Num.uniform_filter1d(L.throttle, k)
	var an := PackedFloat64Array()
	var az := PackedFloat64Array()
	var acc := with_acc and L.has_acc()
	if acc:
		an = Num.uniform_filter1d(Segments.norm_col(L.acc), k)
		az = L.acc[2]
	var m := Num.zeros_mask(L.n)
	for i in L.n:
		if Num.stop(cancel, i):
			return m
		if use[i] == 0 or not (gq[i] < 30.0):
			continue
		if not (sqrt(maxf(m2[i] - m1[i] * m1[i], 0.0)) < 0.03):
			continue
		var good := true
		for q in 4:
			if not (L.motor_hz[q][i] > 30.0):
				good = false
				break
		if not good:
			continue
		if acc and not (absf(an[i] - 1.0) < 0.05 and az[i] > 0.97 * an[i]):
			continue
		m[i] = 1
	return m


## Pooled hover statistics over the ACC sessions if there are any, else all; model_v: per session the
## model voltage (motor fit) for the hover voltage. null if no session has 2 s of hover.
static func hover(logs: Array, uses: Array, mass: float, model_v: Array, cancel: Callable = Callable()) -> Variant:
	var idx: Array = []
	for i in logs.size():
		if logs[i].has_acc():
			idx.append(i)
	var src := "acc" if not idx.is_empty() else "steady"
	if idx.is_empty():
		idx = range(logs.size())
	var sw2 := PackedFloat64Array()
	var uu := PackedFloat64Array()
	var thr := PackedFloat64Array()
	var VV := PackedFloat64Array()
	var per_log := {}
	for i in idx:
		var L: Flight = logs[i]
		var m := hover_mask(L, uses[i], true, cancel)
		var cnt := m.count(1)
		if cnt < 2.0 * L.fs:
			continue
		var s2 := PackedFloat64Array()
		var su := PackedFloat64Array()
		for j in L.n:
			if Num.stop(cancel, j):
				return null
			if m[j] == 0:
				continue
			var s := 0.0
			var us := 0.0
			for q in 4:
				var w: float = TAU * L.motor_hz[q][j]
				s += w * w
				us += clampf((L.motor_raw[q][j] - 48.0) / 1999.0, 0.0, 1.0)
			s2.append(s)
			su.append(us / 4.0)
			thr.append(L.throttle[j])
			if not (model_v[i] as PackedFloat64Array).is_empty():
				VV.append(model_v[i][j])
		sw2.append_array(s2)
		uu.append_array(su)
		per_log[L.name] = {"s": Num.pyround(cnt / L.fs, 1), "hz": Num.pyround(sqrt(Num.median(s2) / 4.0) / TAU, 1),
			"u": Num.pyround(Num.median(su), 3)}
	if sw2.is_empty():
		return null
	var med := Num.median(sw2)
	var res := {"source": src, "n_s": Num.pyround(sw2.size() / logs[0].fs, 1), "per_log": per_log,
		"sum_w2": med, "u": Num.median(uu), "throttle": Num.median(thr), "hz": sqrt(med / 4.0) / TAU}
	res.kT = mass * G / med
	res.sum_w = 4.0 * sqrt(med / 4.0)
	res.V = Num.median(VV) if not VV.is_empty() else null
	return res


## Hover speed from the no-ACC rule on the ACC sessions, to show how good the fallback is.
static func steady_estimate(logs: Array, uses: Array, cancel: Callable = Callable()) -> Dictionary:
	var out := {}
	for i in logs.size():
		var L: Flight = logs[i]
		if not L.has_acc():
			continue
		var m := hover_mask(L, uses[i], false, cancel)
		if m.count(1) > 2.0 * L.fs:
			var s2 := PackedFloat64Array()
			for j in L.n:
				if Num.stop(cancel, j):
					return out
				if m[j] != 0:
					var s := 0.0
					for q in 4:
						var w: float = TAU * L.motor_hz[q][j]
						s += w * w
					s2.append(s)
			out[L.name] = Num.pyround(sqrt(Num.median(s2) / 4.0) / TAU, 1)
	return out
