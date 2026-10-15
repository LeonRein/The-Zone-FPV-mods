extends RefCounted
## real_quad logfit: the numerics of the pipeline, written to give the numbers of the SciPy / NumPy
## calls the reference implementation (Python, fpvident) makes:
##   butter()          scipy.signal.butter(..., output="sos"): buttap, pre-warp, lp2lp / lp2hp / lp2bp,
##                     bilinear, zpk2sos with "nearest" pairing (the gain in the first section)
##   sosfiltfilt()     scipy.signal.sosfiltfilt: odd extension of 3 * (2 * sections + 1) samples,
##                     sosfilt_zi initial states, forward and backward pass
##   savgol()          scipy.signal.savgol_filter(x, window, 2, deriv, mode="interp")
##   uniform_filter1d  scipy.ndimage.uniform_filter1d (mode "reflect", the C routine's running sum)
##   median_at()       scipy.ndimage.median_filter (mode "nearest") at single samples
##   median, percentile, mean (NumPy's pairwise sum), std, gradient, interp, floor_div
##   lstsq()           least squares by Householder QR; solve() Gaussian elimination with partial
##                     pivoting (LAPACK gesv); solve_normal() equilibrated normal equations
##   fft(), welch()    radix-2 FFT, Welch PSD (Hann, half overlap, constant detrend, density)
## Arrays are PackedFloat64Array (64-bit floats; Vector2/3 are 32-bit and not used here), masks
## PackedByteArray (0/1), 2-D data a list of columns.

const EPS := 2.220446049250313e-16


# ------------------------------------------------------------------ masks --

## Runs of a mask: [a0, b0, a1, b1, ...], each run a..b-1.
static func runs(mask: PackedByteArray) -> PackedInt32Array:
	var out := PackedInt32Array()
	var n := mask.size()
	var on := false
	for i in n:
		if mask[i] != 0:
			if not on:
				out.append(i)
				on = true
		elif on:
			out.append(i)
			on = false
	if on:
		out.append(n)
	return out


static func count(mask: PackedByteArray) -> int:
	return mask.count(1)


static func mask_and(a: PackedByteArray, b: PackedByteArray) -> PackedByteArray:
	var o := a.duplicate()
	for i in o.size():
		if b[i] == 0:
			o[i] = 0
	return o


static func mask_not(a: PackedByteArray) -> PackedByteArray:
	var o := PackedByteArray()
	o.resize(a.size())
	for i in a.size():
		o[i] = 1 if a[i] == 0 else 0
	return o


static func zeros_mask(n: int) -> PackedByteArray:
	var o := PackedByteArray()
	o.resize(n)
	o.fill(0)
	return o


static func select(x: PackedFloat64Array, mask: PackedByteArray) -> PackedFloat64Array:
	var o := PackedFloat64Array()
	o.resize(mask.count(1))
	var k := 0
	for i in mask.size():
		if mask[i] != 0:
			o[k] = x[i]
			k += 1
	return o


static func zeros(n: int) -> PackedFloat64Array:
	var o := PackedFloat64Array()
	o.resize(n)
	o.fill(0.0)
	return o


static func slice(x: PackedFloat64Array, a: int, b: int) -> PackedFloat64Array:
	return x.slice(a, b)


## First index with t >= v (t ascending).
static func lower(t: PackedFloat64Array, v: float) -> int:
	return t.bsearch(v, true)


## First index with t > v (t ascending).
static func upper(t: PackedFloat64Array, v: float) -> int:
	return t.bsearch(v, false)


# ------------------------------------------------------------- statistics --

## NumPy's pairwise summation (blocks of 8 partial sums up to 128 values).
static func psum(x: PackedFloat64Array, a: int = 0, n: int = -1) -> float:
	if n < 0:
		n = x.size() - a
	if n < 8:
		var s := 0.0
		for i in n:
			s += x[a + i]
		return s
	if n <= 128:
		var r0 := x[a]
		var r1 := x[a + 1]
		var r2 := x[a + 2]
		var r3 := x[a + 3]
		var r4 := x[a + 4]
		var r5 := x[a + 5]
		var r6 := x[a + 6]
		var r7 := x[a + 7]
		var i := 8
		while i < n - n % 8:
			r0 += x[a + i]
			r1 += x[a + i + 1]
			r2 += x[a + i + 2]
			r3 += x[a + i + 3]
			r4 += x[a + i + 4]
			r5 += x[a + i + 5]
			r6 += x[a + i + 6]
			r7 += x[a + i + 7]
			i += 8
		var res := ((r0 + r1) + (r2 + r3)) + ((r4 + r5) + (r6 + r7))
		while i < n:
			res += x[a + i]
			i += 1
		return res
	var n2 := n / 2
	n2 -= n2 % 8
	return psum(x, a, n2) + psum(x, a + n2, n - n2)


static func mean(x: PackedFloat64Array) -> float:
	return psum(x) / x.size() if x.size() > 0 else NAN


static func variance(x: PackedFloat64Array) -> float:
	var m := mean(x)
	var d := PackedFloat64Array()
	d.resize(x.size())
	for i in x.size():
		var v := x[i] - m
		d[i] = v * v
	return psum(d) / x.size()


static func std(x: PackedFloat64Array) -> float:
	return sqrt(variance(x))


static func median(x: PackedFloat64Array) -> float:
	var n := x.size()
	if n == 0:
		return NAN
	var s := x.duplicate()
	s.sort()
	if n % 2 == 1:
		return s[n / 2]
	return (s[n / 2 - 1] + s[n / 2]) / 2.0


## numpy.percentile (method "linear").
static func percentile(x: PackedFloat64Array, q: float) -> float:
	var s := x.duplicate()
	s.sort()
	return percentile_sorted(s, q)


static func percentile_sorted(s: PackedFloat64Array, q: float) -> float:
	var n := s.size()
	if n == 0:
		return NAN
	var vi := q / 100.0 * (n - 1)
	var lo := int(floorf(vi))
	var hi := mini(lo + 1, n - 1)
	var g := vi - lo
	var a := s[lo]
	var b := s[hi]
	var d := b - a
	if g >= 0.5:
		return b - d * (1.0 - g)
	return a + d * g


static func max_abs(x: PackedFloat64Array) -> float:
	var m := 0.0
	for v in x:
		m = maxf(m, absf(v))
	return m


## Python's roundf(x, nd): halves to even.
static func pyround(x: float, nd: int = 0) -> float:
	var p := pow(10.0, nd)
	var y := x * p
	var r := roundf(y)
	if absf(y - floorf(y) - 0.5) == 0.0:
		r = 2.0 * roundf(y / 2.0)
	return r / p if nd > 0 else r


static func pyround_int(x: float) -> int:
	return int(pyround(x, 0))


## numpy floor_divide for floats (npy_divmod).
static func floor_div(a: float, b: float) -> float:
	var m := fmod(a, b)
	var div := (a - m) / b
	if m != 0.0 and ((b < 0.0) != (m < 0.0)):
		div -= 1.0
	if div != 0.0:
		var fl := floorf(div)
		if div - fl > 0.5:
			fl += 1.0
		return fl
	return 0.0


## numpy.gradient(x, h) (second order inside, first order at the ends).
static func gradient(x: PackedFloat64Array, h: float) -> PackedFloat64Array:
	var n := x.size()
	var o := PackedFloat64Array()
	o.resize(n)
	if n < 2:
		o.fill(0.0)
		return o
	var h2 := 2.0 * h
	for i in range(1, n - 1):
		o[i] = (x[i + 1] - x[i - 1]) / h2
	o[0] = (x[1] - x[0]) / h
	o[n - 1] = (x[n - 1] - x[n - 2]) / h
	return o


## numpy.interp (x ascending in xp).
static func interp(x: float, xp: PackedFloat64Array, fp: PackedFloat64Array) -> float:
	var n := xp.size()
	if x <= xp[0]:
		return fp[0]
	if x >= xp[n - 1]:
		return fp[n - 1]
	var j := xp.bsearch(x, false) - 1
	if xp[j] == x:
		return fp[j]
	var slope := (fp[j + 1] - fp[j]) / (xp[j + 1] - xp[j])
	return slope * (x - xp[j]) + fp[j]


# -------------------------------------------------------------- smoothing --

## scipy.ndimage.uniform_filter1d(x, size) with mode "reflect".
static func uniform_filter1d(x: PackedFloat64Array, size: int) -> PackedFloat64Array:
	var n := x.size()
	var s1 := size / 2
	var s2 := size - s1 - 1
	var e := PackedFloat64Array()
	e.resize(n + s1 + s2)
	for j in s1:
		e[s1 - 1 - j] = x[mini(j, n - 1)] if n > 0 else 0.0
	for i in n:
		e[s1 + i] = x[i]
	for j in s2:
		e[s1 + n + j] = x[maxi(n - 1 - j, 0)]
	var o := PackedFloat64Array()
	o.resize(n)
	if n == 0:
		return o
	var fs := float(size)
	var tmp := 0.0
	for i in size:
		tmp += e[i]
	o[0] = tmp / fs
	for i in range(1, n):
		tmp += e[i + size - 1] - e[i - 1]
		o[i] = tmp / fs
	return o


## scipy.ndimage.median_filter(x, size, mode="nearest") at sample i (size odd).
static func median_at(x: PackedFloat64Array, i: int, size: int) -> float:
	var h := size / 2
	var n := x.size()
	var w := PackedFloat64Array()
	w.resize(size)
	for k in size:
		w[k] = x[clampi(i - h + k, 0, n - 1)]
	w.sort()
	return w[h]


## scipy.signal.savgol_filter(x, window, 2, deriv) with mode "interp" (window odd, deriv 0 or 1).
static func savgol(x: PackedFloat64Array, window: int, deriv: int) -> PackedFloat64Array:
	var n := x.size()
	var m := window / 2
	var c := PackedFloat64Array()
	c.resize(window)
	var mf := float(m)
	for k in range(-m, m + 1):
		if deriv == 0:
			c[k + m] = (3.0 * (3.0 * mf * mf + 3.0 * mf - 1.0) - 15.0 * k * k) / ((2.0 * mf + 3.0) * (2.0 * mf + 1.0) * (2.0 * mf - 1.0))
		else:
			c[k + m] = 3.0 * k / (mf * (mf + 1.0) * (2.0 * mf + 1.0))
	var o := PackedFloat64Array()
	o.resize(n)
	if n < window:
		o.fill(0.0)
		return o
	for i in range(m, n - m):
		var s := 0.0
		for k in window:
			s += c[k] * x[i - m + k]
		o[i] = s
	_savgol_edge(x, 0, window, 0, m, deriv, o)
	_savgol_edge(x, n - window, n, n - m, n, deriv, o)
	return o


## Quadratic least-squares fit over x[w0:w1] (centred abscissa), evaluated (or its slope) at i0..i1-1.
static func _savgol_edge(x: PackedFloat64Array, w0: int, w1: int, i0: int, i1: int, deriv: int, o: PackedFloat64Array) -> void:
	var nw := w1 - w0
	var c := (nw - 1) / 2.0
	var s := PackedFloat64Array([0, 0, 0, 0, 0])
	var r := PackedFloat64Array([0, 0, 0])
	for k in nw:
		var t := k - c
		var t2 := t * t
		var v := x[w0 + k]
		s[0] += 1.0
		s[1] += t
		s[2] += t2
		s[3] += t2 * t
		s[4] += t2 * t2
		r[0] += v
		r[1] += v * t
		r[2] += v * t2
	var p := solve([[s[0], s[1], s[2]], [s[1], s[2], s[3]], [s[2], s[3], s[4]]], r)
	for i in range(i0, i1):
		var t := (i - w0) - c
		if deriv == 0:
			o[i] = p[0] + p[1] * t + p[2] * t * t
		else:
			o[i] = p[1] + 2.0 * p[2] * t


# --------------------------------------------------------------- filters --

## scipy.signal.butter(order, wn, btype, fs=fs, output="sos"); wn a float or [lo, hi]; returns the
## sections flat (6 per section: b0 b1 b2 a0 a1 a2).
static func butter(order: int, wn: Variant, btype: String, fs: float) -> PackedFloat64Array:
	var w: Array = wn if wn is Array else [wn]
	var warped: Array[float] = []
	for f in w:
		var x := 2.0 * float(f) / fs
		warped.append(2.0 * 2.0 * tan(PI * x / 2.0))
	# analog prototype
	var z: Array = []
	var p: Array = []
	var k := 1.0
	var mm := -order + 1
	while mm < order:
		var th := PI * mm / (2.0 * order)
		p.append(_c(-cos(th), -sin(th)))
		mm += 2
	if btype == "lowpass" or btype == "highpass":
		var wo := warped[0]
		if btype == "lowpass":
			for i in p.size():
				p[i] = _c(p[i][0] * wo, p[i][1] * wo)
			k = k * pow(wo, p.size() - z.size())
		else:
			var pz := _c(1.0, 0.0)
			for q in p:
				pz = _cmul(pz, _c(-q[0], -q[1]))
			k = k * _cdiv(_c(1.0, 0.0), pz)[0]
			for i in p.size():
				p[i] = _cdiv(_c(wo, 0.0), p[i])
			var deg := p.size() - z.size()
			for i in deg:
				z.append(_c(0.0, 0.0))
	else:
		var bw := warped[1] - warped[0]
		var wo := sqrt(warped[0] * warped[1])
		var deg := p.size() - z.size()
		var pp: Array = []
		var pm: Array = []
		for q in p:
			var ql := _c(q[0] * bw / 2.0, q[1] * bw / 2.0)
			var sq := _csqrt(_csub(_cmul(ql, ql), _c(wo * wo, 0.0)))
			pp.append(_cadd(ql, sq))
			pm.append(_csub(ql, sq))
		p = pp + pm
		for i in deg:
			z.append(_c(0.0, 0.0))
		k = k * pow(bw, deg)
	# bilinear (fs = 2 after the pre-warp)
	var fs2 := 4.0
	var nz := _c(1.0, 0.0)
	var dz := _c(1.0, 0.0)
	for q in z:
		nz = _cmul(nz, _csub(_c(fs2, 0.0), q))
	for q in p:
		dz = _cmul(dz, _csub(_c(fs2, 0.0), q))
	var deg2 := p.size() - z.size()
	var zz: Array = []
	var pz2: Array = []
	for q in z:
		zz.append(_cdiv(_cadd(_c(fs2, 0.0), q), _csub(_c(fs2, 0.0), q)))
	for q in p:
		pz2.append(_cdiv(_cadd(_c(fs2, 0.0), q), _csub(_c(fs2, 0.0), q)))
	for i in deg2:
		zz.append(_c(-1.0, 0.0))
	k = k * _cdiv(nz, dz)[0]
	return _zpk2sos(zz, pz2, k)


static func _zpk2sos(z: Array, p: Array, k: float) -> PackedFloat64Array:
	var n_sec := (maxi(p.size(), z.size()) + 1) / 2
	if p.size() % 2 == 1:
		p.append(_c(0.0, 0.0))
		z.append(_c(0.0, 0.0))
	z = _cplxreal(z)
	p = _cplxreal(p)
	var sos := PackedFloat64Array()
	sos.resize(6 * n_sec)
	sos.fill(0.0)
	var si := n_sec - 1
	while si >= 0:
		var i1 := _idx_worst(p)
		var p1: PackedFloat64Array = p[i1]
		p.remove_at(i1)
		var nreal_p := 0
		for q in p:
			if q[1] == 0.0:
				nreal_p += 1
		var nreal_z := 0
		for q in z:
			if q[1] == 0.0:
				nreal_z += 1
		if p1[1] == 0.0 and nreal_p == 0:
			var zi := _nearest(z, p1, "real")
			var z1: PackedFloat64Array = z[zi]
			z.remove_at(zi)
			_put_section(sos, si, [z1], [p1])
		elif p.size() + 1 == z.size() and p1[1] != 0.0 and nreal_p == 1 and nreal_z == 1:
			var zi := _nearest(z, p1, "complex")
			var z1: PackedFloat64Array = z[zi]
			z.remove_at(zi)
			_put_section(sos, si, [z1, _conj(z1)], [p1, _conj(p1)])
		else:
			var p2: PackedFloat64Array
			if p1[1] == 0.0:
				var reals: Array = []
				var ridx: Array[int] = []
				for j in p.size():
					if p[j][1] == 0.0:
						reals.append(p[j])
						ridx.append(j)
				var j2: int = ridx[_idx_worst(reals)]
				p2 = p[j2]
				p.remove_at(j2)
			else:
				p2 = _conj(p1)
			if z.size() > 0:
				var zi := _nearest(z, p1, "any")
				var z1: PackedFloat64Array = z[zi]
				z.remove_at(zi)
				if z1[1] != 0.0:
					_put_section(sos, si, [z1, _conj(z1)], [p1, p2])
				elif z.size() > 0:
					var zj := _nearest(z, p1, "real")
					var z2: PackedFloat64Array = z[zj]
					z.remove_at(zj)
					_put_section(sos, si, [z1, z2], [p1, p2])
				else:
					_put_section(sos, si, [z1], [p1, p2])
			else:
				_put_section(sos, si, [], [p1, p2])
		si -= 1
	for j in 3:
		sos[j] *= k
	return sos


static func _put_section(sos: PackedFloat64Array, si: int, z: Array, p: Array) -> void:
	var b := _poly(z)
	var a := _poly(p)
	for j in b.size():
		sos[si * 6 + 3 - b.size() + j] = b[j]
	for j in a.size():
		sos[si * 6 + 6 - a.size() + j] = a[j]


## Real coefficients of the polynomial with these roots (conjugate pairs or reals).
static func _poly(r: Array) -> PackedFloat64Array:
	var re := PackedFloat64Array([1.0])
	var im := PackedFloat64Array([0.0])
	for q in r:
		var nre := PackedFloat64Array()
		var nim := PackedFloat64Array()
		nre.resize(re.size() + 1)
		nim.resize(re.size() + 1)
		nre.fill(0.0)
		nim.fill(0.0)
		for j in re.size():
			nre[j] += re[j]
			nim[j] += im[j]
			nre[j + 1] += -(re[j] * q[0] - im[j] * q[1])
			nim[j + 1] += -(re[j] * q[1] + im[j] * q[0])
		re = nre
		im = nim
	return re


static func _idx_worst(p: Array) -> int:
	var best := 0
	var bv := INF
	for j in p.size():
		var v := absf(1.0 - _cabs(p[j]))
		if v < bv:
			bv = v
			best = j
	return best


static func _nearest(z: Array, to: PackedFloat64Array, which: String) -> int:
	var order: Array = []
	for j in z.size():
		order.append([_cabs(_csub(z[j], to)), j])
	order.sort_custom(func(a, b): return a[0] < b[0] or (a[0] == b[0] and a[1] < b[1]))
	for o in order:
		var q: PackedFloat64Array = z[o[1]]
		if which == "any" or (which == "real" and q[1] == 0.0) or (which == "complex" and q[1] != 0.0):
			return o[1]
	return 0


## scipy's _cplxreal: one of each conjugate pair (positive imaginary part, sorted by real part), then
## the reals (sorted); pairs averaged.
static func _cplxreal(z: Array) -> Array:
	var tol := 100.0 * EPS
	var s := z.duplicate()
	s.sort_custom(func(a, b): return a[0] < b[0] or (a[0] == b[0] and absf(a[1]) < absf(b[1])))
	var zr: Array = []
	var zp: Array = []
	var zn: Array = []
	for q in s:
		if absf(q[1]) <= tol * _cabs(q):
			zr.append(_c(q[0], 0.0))
		elif q[1] > 0.0:
			zp.append(q)
		else:
			zn.append(q)
	var out: Array = []
	for j in zp.size():
		var a: PackedFloat64Array = zp[j]
		var b: PackedFloat64Array = zn[j] if j < zn.size() else _conj(a)
		out.append(_c((a[0] + b[0]) / 2.0, (a[1] - b[1]) / 2.0))
	return out + zr


static func _c(re: float, im: float) -> PackedFloat64Array:
	return PackedFloat64Array([re, im])


static func _conj(a: PackedFloat64Array) -> PackedFloat64Array:
	return _c(a[0], -a[1])


static func _cadd(a: PackedFloat64Array, b: PackedFloat64Array) -> PackedFloat64Array:
	return _c(a[0] + b[0], a[1] + b[1])


static func _csub(a: PackedFloat64Array, b: PackedFloat64Array) -> PackedFloat64Array:
	return _c(a[0] - b[0], a[1] - b[1])


static func _cmul(a: PackedFloat64Array, b: PackedFloat64Array) -> PackedFloat64Array:
	return _c(a[0] * b[0] - a[1] * b[1], a[0] * b[1] + a[1] * b[0])


## Smith's complex division (as NumPy).
static func _cdiv(a: PackedFloat64Array, b: PackedFloat64Array) -> PackedFloat64Array:
	if absf(b[0]) >= absf(b[1]):
		var rat := b[1] / b[0]
		var scl := 1.0 / (b[0] + b[1] * rat)
		return _c((a[0] + a[1] * rat) * scl, (a[1] - a[0] * rat) * scl)
	var rat2 := b[0] / b[1]
	var scl2 := 1.0 / (b[1] + b[0] * rat2)
	return _c((a[0] * rat2 + a[1]) * scl2, (a[1] * rat2 - a[0]) * scl2)


static func _cabs(a: PackedFloat64Array) -> float:
	var x := absf(a[0])
	var y := absf(a[1])
	var m := maxf(x, y)
	if m == 0.0:
		return 0.0
	var r := minf(x, y) / m
	return m * sqrt(1.0 + r * r)


static func _csqrt(a: PackedFloat64Array) -> PackedFloat64Array:
	var r := _cabs(a)
	if a[0] >= 0.0:
		var t := sqrt((r + a[0]) / 2.0)
		return _c(t, a[1] / (2.0 * t) if t != 0.0 else 0.0)
	var t2 := sqrt((r - a[0]) / 2.0)
	return _c(absf(a[1]) / (2.0 * t2), t2 if a[1] >= 0.0 else -t2)


## Steady-state section states for a unit step (scipy.signal.sosfilt_zi).
static func sosfilt_zi(sos: PackedFloat64Array) -> PackedFloat64Array:
	var ns := sos.size() / 6
	var zi := PackedFloat64Array()
	zi.resize(2 * ns)
	var scale := 1.0
	for s in ns:
		var b0 := sos[6 * s]
		var b1 := sos[6 * s + 1]
		var b2 := sos[6 * s + 2]
		var a1 := sos[6 * s + 4]
		var a2 := sos[6 * s + 5]
		var r := solve([[1.0 + a1, -1.0], [a2, 1.0]], PackedFloat64Array([b1 - a1 * b0, b2 - a2 * b0]))
		zi[2 * s] = scale * r[0]
		zi[2 * s + 1] = scale * r[1]
		scale *= (b0 + b1 + b2) / (1.0 + a1 + a2)
	return zi


## Filters x in place through the sections (direct form II transposed), states zi (changed).
static func sosfilt_inplace(sos: PackedFloat64Array, x: PackedFloat64Array, zi: PackedFloat64Array) -> void:
	var ns := sos.size() / 6
	var n := x.size()
	for s in ns:
		var b0 := sos[6 * s]
		var b1 := sos[6 * s + 1]
		var b2 := sos[6 * s + 2]
		var a1 := sos[6 * s + 4]
		var a2 := sos[6 * s + 5]
		var z0 := zi[2 * s]
		var z1 := zi[2 * s + 1]
		for i in n:
			var xn := x[i]
			var y := b0 * xn + z0
			z0 = b1 * xn - a1 * y + z1
			z1 = b2 * xn - a2 * y
			x[i] = y
		zi[2 * s] = z0
		zi[2 * s + 1] = z1


## scipy.signal.sosfiltfilt(sos, x) (padtype "odd"); x must be longer than the padding.
static func sosfiltfilt(sos: PackedFloat64Array, x: PackedFloat64Array) -> PackedFloat64Array:
	var ns := sos.size() / 6
	var ntaps := 2 * ns + 1
	var z_b := 0
	var z_a := 0
	for s in ns:
		if sos[6 * s + 2] == 0.0:
			z_b += 1
		if sos[6 * s + 5] == 0.0:
			z_a += 1
	ntaps -= mini(z_b, z_a)
	var edge := 3 * ntaps
	var n := x.size()
	if n <= edge:
		push_error("sosfiltfilt: %d samples, more than %d needed" % [n, edge])
		return x.duplicate()
	var ext := PackedFloat64Array()
	ext.resize(n + 2 * edge)
	var x0 := x[0]
	var xl := x[n - 1]
	for j in edge:
		ext[j] = 2.0 * x0 - x[edge - j]
	for i in n:
		ext[edge + i] = x[i]
	for j in edge:
		ext[edge + n + j] = 2.0 * xl - x[n - 2 - j]
	var zi0 := sosfilt_zi(sos)
	var zi := zi0.duplicate()
	var e0 := ext[0]
	for j in zi.size():
		zi[j] = zi0[j] * e0
	sosfilt_inplace(sos, ext, zi)
	ext.reverse()
	var y0 := ext[0]
	for j in zi.size():
		zi[j] = zi0[j] * y0
	sosfilt_inplace(sos, ext, zi)
	ext.reverse()
	return ext.slice(edge, edge + n)


# --------------------------------------------------------- linear algebra --

## Solves a x = b (a: rows) by Gaussian elimination with partial pivoting; [] if singular.
static func solve(a: Array, b: PackedFloat64Array) -> PackedFloat64Array:
	var n := b.size()
	var m: Array[PackedFloat64Array] = []
	for r in a:
		m.append(PackedFloat64Array(r))
	var y := b.duplicate()
	for c in n:
		var piv := c
		var pv := absf(m[c][c])
		for r in range(c + 1, n):
			if absf(m[r][c]) > pv:
				pv = absf(m[r][c])
				piv = r
		if pv == 0.0:
			return PackedFloat64Array()
		if piv != c:
			var tmp := m[c]
			m[c] = m[piv]
			m[piv] = tmp
			var ty := y[c]
			y[c] = y[piv]
			y[piv] = ty
		var rc := m[c]
		for r in range(c + 1, n):
			var rr := m[r]
			var l := rr[c] / rc[c]
			if l != 0.0:
				for j in range(c + 1, n):
					rr[j] -= l * rc[j]
				y[r] -= l * y[c]
			m[r] = rr
	var x := PackedFloat64Array()
	x.resize(n)
	for i in range(n - 1, -1, -1):
		var s := y[i]
		for j in range(i + 1, n):
			s -= m[i][j] * x[j]
		x[i] = s / m[i][i]
	return x


## Normal equations ata x = aty (ata symmetric, rows), equilibrated by the diagonal.
static func solve_normal(ata: Array, aty: PackedFloat64Array) -> PackedFloat64Array:
	var n := aty.size()
	var d := PackedFloat64Array()
	d.resize(n)
	for i in n:
		var v: float = ata[i][i]
		d[i] = 1.0 / sqrt(v) if v > 0.0 else 1.0
	var s: Array = []
	var r := PackedFloat64Array()
	r.resize(n)
	for i in n:
		var row := PackedFloat64Array()
		row.resize(n)
		for j in n:
			row[j] = float(ata[i][j]) * d[i] * d[j]
		s.append(row)
		r[i] = aty[i] * d[i]
	var z := solve(s, r)
	if z.is_empty():
		return z
	for i in n:
		z[i] *= d[i]
	return z


## Least squares min |A x - y| by Householder QR; A as a list of columns.
static func lstsq(cols: Array, y: PackedFloat64Array) -> PackedFloat64Array:
	var nc := cols.size()
	var m := y.size()
	var a: Array[PackedFloat64Array] = []
	for c in cols:
		a.append((c as PackedFloat64Array).duplicate())
	var b := y.duplicate()
	var rdiag := PackedFloat64Array()
	rdiag.resize(nc)
	for k in nc:
		var col := a[k]
		var nrm := 0.0
		var sc := 0.0
		for i in range(k, m):
			sc = maxf(sc, absf(col[i]))
		if sc == 0.0:
			rdiag[k] = 0.0
			continue
		for i in range(k, m):
			var v := col[i] / sc
			nrm += v * v
		nrm = sc * sqrt(nrm)
		if col[k] > 0.0:
			nrm = -nrm
		# v = col[k:] - nrm e1, stored in col[k:]; H = I - v v^T / (v^T v) with v^T v = -nrm * v0 * ...
		col[k] -= nrm
		var vtv := 0.0
		for i in range(k, m):
			vtv += col[i] * col[i]
		for j in range(k + 1, nc):
			var cj := a[j]
			var s := 0.0
			for i in range(k, m):
				s += col[i] * cj[i]
			var f := 2.0 * s / vtv
			for i in range(k, m):
				cj[i] -= f * col[i]
			a[j] = cj
		var sb := 0.0
		for i in range(k, m):
			sb += col[i] * b[i]
		var fb := 2.0 * sb / vtv
		for i in range(k, m):
			b[i] -= fb * col[i]
		a[k] = col
		rdiag[k] = nrm
	var x := PackedFloat64Array()
	x.resize(nc)
	for i in range(nc - 1, -1, -1):
		var s := b[i]
		for j in range(i + 1, nc):
			s -= a[j][i] * x[j]
		x[i] = s / rdiag[i] if rdiag[i] != 0.0 else 0.0
	return x


## Inverse of a small matrix (rows); [] if singular.
static func inverse(a: Array) -> Array:
	var n := a.size()
	var out: Array = []
	for i in n:
		out.append(PackedFloat64Array())
	for j in n:
		var e := zeros(n)
		e[j] = 1.0
		var c := solve(a, e)
		if c.is_empty():
			return []
		for i in n:
			out[i].append(c[i])
	return out


# ------------------------------------------------------------------ spectra --

## In-place radix-2 FFT (size a power of two).
static func fft(re: PackedFloat64Array, im: PackedFloat64Array) -> void:
	var n := re.size()
	var j := 0
	for i in range(1, n):
		var bit := n >> 1
		while j & bit:
			j ^= bit
			bit >>= 1
		j |= bit
		if i < j:
			var t := re[i]
			re[i] = re[j]
			re[j] = t
			t = im[i]
			im[i] = im[j]
			im[j] = t
	var size := 2
	while size <= n:
		var half := size / 2
		var ang := -TAU / size
		for k in half:
			var wr := cos(ang * k)
			var wi := sin(ang * k)
			var s := k
			while s < n:
				var u := s + half
				var tr := wr * re[u] - wi * im[u]
				var ti := wr * im[u] + wi * re[u]
				re[u] = re[s] - tr
				im[u] = im[s] - ti
				re[s] += tr
				im[s] += ti
				s += size
		size *= 2


## Hann window (periodic, as scipy.signal.get_window("hann", n)).
static func hann(n: int) -> PackedFloat64Array:
	var w := PackedFloat64Array()
	w.resize(n)
	for k in n:
		w[k] = 0.5 - 0.5 * cos(TAU * k / n)
	return w


## scipy.signal.welch(x, fs, nperseg) (Hann, half overlap, constant detrend, one-sided density);
## returns [freqs, pxx].
static func welch(x: PackedFloat64Array, fs: float, nperseg: int) -> Array:
	var w := hann(nperseg)
	var wss := 0.0
	for v in w:
		wss += v * v
	var step := nperseg / 2
	var nseg := (x.size() - step) / step
	var nf := nperseg / 2 + 1
	var pxx := zeros(nf)
	var re := PackedFloat64Array()
	var im := PackedFloat64Array()
	re.resize(nperseg)
	im.resize(nperseg)
	for s in nseg:
		var a := s * step
		var m := psum(x, a, nperseg) / nperseg
		for k in nperseg:
			re[k] = (x[a + k] - m) * w[k]
		im.fill(0.0)
		fft(re, im)
		for k in nf:
			pxx[k] += re[k] * re[k] + im[k] * im[k]
	var scale := 1.0 / (fs * wss)
	var freqs := PackedFloat64Array()
	freqs.resize(nf)
	for k in nf:
		var v := pxx[k] * scale / maxf(nseg, 1)
		if k != 0 and not (nperseg % 2 == 0 and k == nf - 1):
			v *= 2.0
		pxx[k] = v
		freqs[k] = k * fs / nperseg
	return [freqs, pxx]
