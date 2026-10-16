extends RefCounted
## real_quad logfit: torque imbalance (CG offset, frame twist, prop/motor differences) as
## torque = beta * total thrust. In steady flight the torques balance, so the rotor speeds at hover give
##   roll : arm * kT * (W2 + W3 - W0 - W1) + beta_x * kT * sum(W) = 0
##   pitch: arm * kT * (W0 + W2 - W1 - W3) + beta_y * kT * sum(W) = 0
##   yaw  : -s0 * kQ * (W0 + W3 - W1 - W2) + beta_z * kT * sum(W) = 0
## (regression through the origin). Hover samples from the ACC sessions when there are any.

const Num := preload("num.gd")
const Flight := preload("flight.gd")
const Thrust := preload("thrust.gd")


static func _slope(x: PackedFloat64Array, sw: PackedFloat64Array) -> float:
	var a := PackedFloat64Array()
	var b := PackedFloat64Array()
	a.resize(x.size())
	b.resize(x.size())
	for i in x.size():
		a[i] = x[i] * sw[i]
		b[i] = sw[i] * sw[i]
	return Num.psum(a) / Num.psum(b)


## {beta [3], per_log_1e3 {session: [3]}, spread [3] or null, n} or null without hover.
static func fit(logs: Array, uses: Array, arm: float, kT: float, kQ: float, s0: float, cancel: Callable = Callable()) -> Variant:
	var idx: Array = []
	for i in logs.size():
		if logs[i].has_acc():
			idx.append(i)
	if idx.is_empty():
		idx = range(logs.size())
	var xr := PackedFloat64Array()
	var xp := PackedFloat64Array()
	var xy := PackedFloat64Array()
	var sw := PackedFloat64Array()
	var per := {}
	for i in idx:
		var L: Flight = logs[i]
		var m := Thrust.hover_mask(L, uses[i], true, cancel)
		if m.count(1) < 2.0 * L.fs:
			continue
		var r := PackedFloat64Array()
		var p := PackedFloat64Array()
		var y := PackedFloat64Array()
		var s := PackedFloat64Array()
		for j in L.n:
			if Num.stop(cancel, j):
				return null
			if m[j] == 0:
				continue
			var W := PackedFloat64Array([0, 0, 0, 0])
			for q in 4:
				var w: float = TAU * L.motor_hz[q][j]
				W[q] = w * w
			r.append(W[2] + W[3] - W[0] - W[1])
			p.append(W[0] + W[2] - W[1] - W[3])
			y.append(W[0] + W[3] - W[1] - W[2])
			s.append(W[0] + W[1] + W[2] + W[3])
		per[L.name] = [Num.pyround(-arm * _slope(r, s) * 1e3, 2), Num.pyround(-arm * _slope(p, s) * 1e3, 2),
			Num.pyround(s0 * kQ / kT * _slope(y, s) * 1e3, 2)]
		xr.append_array(r)
		xp.append_array(p)
		xy.append_array(y)
		sw.append_array(s)
	if sw.is_empty():
		return null
	var beta := [-arm * _slope(xr, sw), -arm * _slope(xp, sw), s0 * kQ / kT * _slope(xy, sw)]
	var spread: Variant = null
	if per.size() > 1:
		spread = []
		for k in 3:
			var col := PackedFloat64Array()
			for v in per.values():
				col.append(v[k])
			spread.append(Num.std(col) * 1e-3)
	return {"beta": beta, "per_log_1e3": per, "spread": spread, "n": sw.size()}
