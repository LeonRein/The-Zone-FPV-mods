extends RefCounted
## real_quad logfit: one blackbox session as the flight arrays of the pipeline (contract 1.2).
##
## load() turns a session of bbl_decoder.gd into the arrays below (the same conversions as the
## reference's decode.py); logs faster than 1.5 kHz are decimated to about 1 kHz (every k-th sample).
##   t            s since the first frame (from `time`, us)
##   gyro         deg/s, gyroADC[0..2] (/10 with blackbox_high_resolution); 3 columns
##   gyro_unfilt  deg/s, gyroUnfilt[0..2]; missing: gyro (info.flags.unfilt false)
##   setpoint     deg/s, setpoint[0..2]
##   rc           rcCommand[0..3] (throttle 1000..2000)
##   throttle     0..1, setpoint[3] / 1000 (missing: from rcCommand[3])
##   motor_raw    DShot command motor[0..3]
##   motor_hz     rotor Hz from eRPM[0..3] and motor_poles; [] without eRPM
##   armed, chirp box 0 / box 6 of flightModeFlags (slow frames, then FLIGHTMODE events); 0/1
##   vbat         V (vbatLatest / 100; [] if not logged or its 95th percentile is <= 1 V)
##   acc          g, accSmooth[0..2] / acc_1G; [] without ACC
##   pid_i        axisI[0..2] ([] if not logged): the replay starts the I-terms from it
## Columns of 2-D data are separate PackedFloat64Array; masks are PackedByteArray.

const BBL := preload("../bbl_decoder.gd")
const Num := preload("num.gd")
const CHIRP_BOX := 6

var name := ""                       # "<file stem>#<n>", n from 1
var path := ""
var file := ""
var stem := ""
var session := 1
var headers: Dictionary = {}
var fs := 0.0                        # log rate after decimation [Hz]
var fs_raw := 0.0
var decimation := 1
var n := 0
var t := PackedFloat64Array()
var gyro: Array = []
var gyro_unfilt: Array = []
var setpoint: Array = []
var rc: Array = []
var throttle := PackedFloat64Array()
var motor_raw: Array = []
var motor_hz: Array = []
var armed := PackedByteArray()
var chirp := PackedByteArray()
var vbat := PackedFloat64Array()
var acc: Array = []
var pid_i: Array = []
var info: Dictionary = {}            # fields, stats, n_frames, n_motors, n_erpm, flags, vbat_p95, acc_limit_g, hash, ...


func has_acc() -> bool:
	return not acc.is_empty()


func has_vbat() -> bool:
	return not vbat.is_empty()


func has_hz() -> bool:
	return not motor_hz.is_empty()


## First integer of a header value ("1,2,3" -> 1), default if missing or not a number.
func hint(key: String, default: int = 0) -> int:
	return header_int(headers, key, default)


static func header_int(h: Dictionary, key: String, default: int) -> int:
	if not h.has(key):
		return default
	var s := str(h[key]).split(",")[0].strip_edges()
	return s.to_int() if s.is_valid_int() else default


func looptime_us() -> int:
	return hint("looptime", 125)


func pid_denom() -> int:
	var d := hint("pid_process_denom", 1)
	return d if d != 0 else 1


func pid_hz() -> float:
	return 1e6 / (looptime_us() * pid_denom())


## Motor command 0..1 over the DShot range, per motor.
func u() -> Array:
	var out: Array = []
	for m in 4:
		var raw: PackedFloat64Array = motor_raw[m]
		var c := PackedFloat64Array()
		c.resize(n)
		for i in n:
			c[i] = clampf((raw[i] - 48.0) / 1999.0, 0.0, 1.0)
		out.append(c)
	return out


## Sample indices with t0 <= t < t1, as [a, b).
func sl(t0: float, t1: float) -> PackedInt32Array:
	return PackedInt32Array([Num.lower(t, t0), Num.lower(t, t1)])


## Fills this from decoded session `s` (index k from 0) of the file at `p`.
func load(p: String, s: Dictionary, k: int) -> void:
	path = p
	file = p.get_file()
	stem = file.get_basename()
	session = k + 1
	name = "%s#%d" % [stem, session]
	headers = s.headers
	var names: PackedStringArray = s.main_names
	var rows: int = s.rows if names.size() > 0 else 0
	var stats := {}
	for key in s.stats:
		stats[key] = int(s.stats[key])
	info = {"headers": headers, "fields": names, "stats": stats, "n_frames": rows,
		"n_motors": _count_prefix(names, "motor["), "n_erpm": _count_prefix(names, "eRPM[")}
	var flags := {}
	for pair in [["gyro", "gyroADC"], ["unfilt", "gyroUnfilt"], ["setpoint", "setpoint"], ["acc", "accSmooth"]]:
		flags[pair[0]] = _all(names, pair[1], 3)
	flags.setpoint = flags.setpoint and "setpoint[3]" in names
	flags.rc = _all(names, "rcCommand", 4)
	flags.motors = _all(names, "motor", 4)
	flags.erpm = _all(names, "eRPM", 4)
	flags.vbat = "vbatLatest" in names
	flags.pid = "axisP[0]" in names
	info.flags = flags
	if rows < 2 or not "time" in names:
		info.n_frames = 0
		n = 0
		return
	# Every column is made at the decimated rate straight from the decoder's column (the dk-th value,
	# then the scaling: the same numbers as scaling all of it and decimating, without the full-length
	# copies of ~30 columns that were the peak of the whole fit for logs faster than 1.5 kHz)
	var src_t: Variant = s.main[names.find("time")]
	var nn: int = src_t.size()
	var tt := PackedFloat64Array()      # full length: its median step decides the decimation
	tt.resize(nn)
	var t0 := float(src_t[0])
	for i in nn:
		tt[i] = (float(src_t[i]) - t0) * 1e-6
	var pos := PackedFloat64Array()
	for i in range(1, nn):
		var d := tt[i] - tt[i - 1]
		if d > 0.0:
			pos.append(d)
	var f := 1.0 / _median_consume(pos) if pos.size() > 0 else 1000.0
	var hr_s := str(headers.get("blackbox_high_resolution", "0")).strip_edges()
	var hr := 10.0 if hr_s == "1" or hr_s == "ON" else 1.0
	fs_raw = f
	var dec := Num.pyround_int(f / 1000.0)
	decimation = maxi(dec, 1)
	fs = fs_raw / decimation
	var dk := decimation
	var no := (nn + dk - 1) / dk
	var col := func(key: String, scale: float = 1.0) -> PackedFloat64Array:
		var src: Variant = s.main[names.find(key)]
		var o := PackedFloat64Array()
		o.resize(no)
		if scale != 1.0:
			for j in no:
				o[j] = float(src[j * dk]) / scale
		else:
			for j in no:
				o[j] = float(src[j * dk])
		return o
	var tri := func(prefix: String, scale: float) -> Array:
		var o: Array = []
		for i in 3:
			var key := "%s[%d]" % [prefix, i]
			o.append(col.call(key, scale) if key in names else Num.zeros(no))
		return o
	var g: Array = tri.call("gyroADC", hr)
	var gu: Array = tri.call("gyroUnfilt", hr) if flags.unfilt else [g[0], g[1], g[2]]    # the same arrays: nothing writes them
	var sp: Array = tri.call("setpoint", hr)
	var rcc: Array = []
	if flags.rc:
		for i in 4:
			rcc.append(col.call("rcCommand[%d]" % i, hr))
		if hr != 1.0:
			# the median is of the whole log: from the full-length column, which is not kept
			var src3: Variant = s.main[names.find("rcCommand[3]")]
			var full := PackedFloat64Array()
			full.resize(nn)
			for j in nn:
				full[j] = float(src3[j]) / hr
			if _median_consume(full) < 500.0:
				var c3: PackedFloat64Array = rcc[3]
				for j in no:
					c3[j] = c3[j] * hr
	else:
		for i in 4:
			var c := Num.zeros(no)
			if i == 3:
				c.fill(1000.0)
			rcc.append(c)
	var thr := PackedFloat64Array()
	if "setpoint[3]" in names:
		thr = col.call("setpoint[3]")
		for j in no:
			thr[j] = thr[j] / 1000.0
	else:
		thr.resize(no)
		var r3: PackedFloat64Array = rcc[3]
		for j in no:
			thr[j] = (r3[j] - 1000.0) / 1000.0
	var mr: Array = []
	var mnames: Array[String] = []
	for i in 4:
		if "motor[%d]" % i in names:
			mnames.append("motor[%d]" % i)
	for i in 4:
		mr.append(col.call(mnames[i]) if i < mnames.size() else Num.zeros(no))
	var hz: Array = []
	if flags.erpm:
		var poles := header_int(headers, "motor_poles", 14)
		if poles == 0:
			poles = 14
		var half := poles / 2.0
		for i in 4:
			var c: PackedFloat64Array = col.call("eRPM[%d]" % i)
			for j in no:
				c[j] = c[j] * 100.0 / 60.0 / half
			hz.append(c)
	var arm_chp := _mode_flags(s, nn, dk)
	var vb := PackedFloat64Array()
	if flags.vbat:
		var vsrc: Variant = s.main[names.find("vbatLatest")]
		var vfull := PackedFloat64Array()      # the percentile is of the whole log
		vfull.resize(nn)
		for j in nn:
			vfull[j] = float(vsrc[j]) / 100.0
		vfull.sort()
		info.vbat_p95 = Num.percentile_sorted(vfull, 95.0)
		if info.vbat_p95 > 1.0:
			vb = col.call("vbatLatest", 100.0)
	var ac: Array = []
	if flags.acc:
		var one_g := float(header_int(headers, "acc_1G", 2048))
		if one_g == 0.0:
			one_g = 2048.0
		for i in 3:
			ac.append(col.call("accSmooth[%d]" % i, one_g))
		info.acc_limit_g = 32767.0 / one_g
	var pi_: Array = tri.call("axisI", 1.0) if flags.pid else []
	t = PackedFloat64Array()
	t.resize(no)
	for j in no:
		t[j] = tt[j * dk]
	gyro = g
	gyro_unfilt = gu
	setpoint = sp
	rc = rcc
	throttle = thr
	motor_raw = mr
	motor_hz = hz
	armed = arm_chp[0]
	chirp = arm_chp[1]
	vbat = vb
	acc = ac
	pid_i = pi_
	n = t.size()


## The median of x, which is sorted in place (Num.median without the copy)
static func _median_consume(x: PackedFloat64Array) -> float:
	var n := x.size()
	if n == 0:
		return NAN
	x.sort()
	if n % 2 == 1:
		return x[n / 2]
	return (x[n / 2 - 1] + x[n / 2]) / 2.0


static func _count_prefix(names: PackedStringArray, prefix: String) -> int:
	var k := 0
	for x in names:
		if x.begins_with(prefix):
			k += 1
	return k


static func _all(names: PackedStringArray, prefix: String, k: int) -> bool:
	for i in k:
		if not "%s[%d]" % [prefix, i] in names:
			return false
	return true


## flightModeFlags per main frame: slow frames, then the FLIGHTMODE events (more precise), in order;
## each sets the flags from its frame on, a later one wins. [armed, chirp] (0 / 1 bytes) of every
## dk-th frame.
static func _mode_flags(s: Dictionary, nn: int, dk: int) -> Array:
	var at := PackedInt64Array()
	var val := PackedInt64Array()
	var sn: PackedStringArray = s.get("slow_names", PackedStringArray())
	var si := sn.find("flightModeFlags")
	if si >= 0:
		var colv = s.slow[si]
		var sat: PackedInt32Array = s.slow_at
		for k in sat.size():
			at.append(sat[k])
			val.append(int(colv[k]))
	for ev in s.get("events", []):
		if int(ev.get("type", -1)) == BBL.EV_FLIGHTMODE:
			at.append(mini(int(ev.at), nn))
			val.append(int(ev.flags))
	var order: Array = range(at.size())
	order.sort_custom(func(a: int, b: int) -> bool: return at[a] < at[b] or (at[a] == at[b] and a < b))
	var no := (nn + dk - 1) / dk
	var arm := PackedByteArray()
	var chp := PackedByteArray()
	arm.resize(no)
	chp.resize(no)
	var best := -1
	var k := 0
	for j in nn:
		while k < order.size() and at[order[k]] <= j:
			best = maxi(best, order[k])
			k += 1
		if j % dk == 0:
			var m := val[best] if best >= 0 else 0
			arm[j / dk] = m & 1
			chp[j / dk] = (m >> CHIRP_BOX) & 1
	return [arm, chp]
