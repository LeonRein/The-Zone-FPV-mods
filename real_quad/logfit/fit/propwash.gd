extends RefCounted
## real_quad logfit: the propwash metric of the logs (no fit; contract 4.1: the dirty_air values are the
## class's in v1, this is the target for their calibration). RMS of the roll/pitch tracking error
## (setpoint - gyro) band-passed 15-78 Hz in 0.25-s windows; median over
##   calm   throttle hover..hover+0.25, sticks quiet
##   chops  below hover right after a throttle chop, sticks quiet
##   flips  below hover right after a throttle chop, big stick moves
## and worst_p99 = the 99th percentile of all usable windows outside chirps.

const Num := preload("num.gd")
const Flight := preload("flight.gd")


static func _segments(mask: PackedByteArray, minlen: int) -> Array:
	var out: Array = []
	var r := Num.runs(mask)
	for j in range(0, r.size(), 2):
		if r[j + 1] - r[j] >= minlen:
			out.append([r[j], r[j + 1]])
	return out


## cancel (optional): polled every 100 windows; when it answers true the metric is incomplete (the caller stops).
static func log_metric(logs: Array, uses: Array, thr_h: float, cancel: Callable = Callable()) -> Dictionary:
	var windows := 0
	var calm := PackedFloat64Array()
	var chop := PackedFloat64Array()
	var flip := PackedFloat64Array()
	var every := PackedFloat64Array()
	for li in logs.size():
		var L: Flight = logs[li]
		var use: PackedByteArray = uses[li]
		var fs := L.fs
		var dthr := Num.gradient(L.throttle, 1.0)
		for i in dthr.size():
			dthr[i] = dthr[i] * fs
		var sos := Num.butter(2, [15.0, minf(78.0, 0.45 * fs)], "bandpass", fs)
		var e0 := Num.zeros(L.n)
		var e1 := Num.zeros(L.n)
		for ab in _segments(use, int(0.5 * fs)):
			for k in 2:
				var x := PackedFloat64Array()
				x.resize(ab[1] - ab[0])
				for i in x.size():
					x[i] = L.setpoint[k][ab[0] + i] - L.gyro[k][ab[0] + i]
				var y := Num.sosfiltfilt(sos, x)
				var dst := e0 if k == 0 else e1
				for i in y.size():
					dst[ab[0] + i] = y[i]
		var n := int(0.25 * fs)
		var sq := PackedFloat64Array()
		var spv := PackedFloat64Array()
		sq.resize(2 * n)
		spv.resize(2 * n)
		var sp0: PackedFloat64Array = L.setpoint[0]
		var sp1: PackedFloat64Array = L.setpoint[1]
		for ab in _segments(use, n * 2):
			var a: int = ab[0]
			var b: int = ab[1]
			var s := a + n / 2
			while s < b - n - n / 2:
				windows += 1
				if windows % 100 == 0 and cancel.is_valid() and cancel.call():
					return {"calm": null, "chops": null, "flips": null, "worst_p99": null, "n": [0, 0, 0, 0]}
				for i in n:
					var v0 := e0[s + i]
					var v1 := e1[s + i]
					sq[2 * i] = v0 * v0
					sq[2 * i + 1] = v1 * v1
					spv[2 * i] = sp0[s + i]
					spv[2 * i + 1] = sp1[s + i]
				var rms := sqrt(Num.psum(sq) / (2 * n))
				var chirp := false
				for i in n:
					if L.chirp[s + i] != 0:
						chirp = true
						break
				if not chirp:
					every.append(rms)
				var spm := Num.std(spv)
				var tt := Num.psum(L.throttle, s, n) / n
				var dmin := INF
				for i in range(maxi(a, s - n), s + n):
					dmin = minf(dmin, dthr[i])
				if tt < thr_h and dmin < -1.5:
					if spm < 100.0:
						chop.append(rms)
					elif spm > 200.0:
						flip.append(rms)
				elif thr_h <= tt and tt < thr_h + 0.25 and spm < 30.0:
					calm.append(rms)
				s += n / 2
	var med := func(v: PackedFloat64Array) -> Variant:
		return Num.pyround(Num.median(v), 2) if v.size() > 0 else null
	return {"calm": med.call(calm), "chops": med.call(chop), "flips": med.call(flip),
		"worst_p99": Num.pyround(Num.percentile(every, 99.0), 2) if every.size() > 0 else null,
		"n": [calm.size(), chop.size(), flip.size(), every.size()]}
