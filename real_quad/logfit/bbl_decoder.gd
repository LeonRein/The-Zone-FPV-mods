extends RefCounted
## real_quad logfit: decoder for Betaflight blackbox logs (.bbl / .bfl).
##
## Written from the public blackbox format (Betaflight's blackbox.c / blackbox_encoding.c /
## blackbox_fielddefs.h): "H name:value" header lines, then I (intra), P (inter), S (slow), G / H (GPS)
## and E (event) frames, each starting with its letter. The header lists, per frame type, the field
## names, signedness, predictor and encoding; a P frame stores only the difference to a prediction
## from the frames before it.
##
##   const BBL := preload("bbl_decoder.gd")
##   var sessions: Array = BBL.new().decode("/path/btfl_001.bbl")
##   for s: Dictionary in sessions:
##       var gyro: PackedInt32Array = BBL.col(s, "gyroADC[0]")
##
## One file can hold several logs (one per arm / blackbox start); each starts with the line
## "H Product:Blackbox flight data recorder ..." and becomes a session (Dictionary):
##   index        position in the file, from 0
##   headers      the "H name:value" lines as strings: headers["Field I name"], headers["rollPID"] ...
##   main_names   field names of the main (I and P) frames, PackedStringArray
##   main         one column per field (column-major), PackedInt32Array; PackedFloat64Array for a column
##                that holds a value outside the int32 range (the time after 35 minutes, unsigned 32 bit)
##   rows         number of main frames (every column has this many entries)
##   slow_names, slow, slow_at   S frames (flight mode flags, failsafe phase ...) in the same column layout;
##                slow_at[k] = index of the main frame that follows the k-th S frame (rows if none does)
##   events       Array of Dictionary {type, at, ...}; at = index of the next main frame. type 0 sync beep
##                (time), 13 in-flight adjustment (func, value), 14 logging resume (iteration, time),
##                15 disarm (reason), 30 flight mode (flags, last_flags), 255 log end
##   gps_names, gps, gps_at      G frames in the same layout, empty without GPS
##   stats        {I, P, S, G, H, E: frames kept; corrupt: frames dropped as damaged (decoding resumes one
##                byte after their start); desync: I / P frames refused because the main stream was
##                broken (P without a good I before it, or time / iteration jumping)}
##   log_end      true if the log ends with its end-of-log event
##   offset, size byte range of the session in the file
##
## Frame validity as in the reference viewers: a frame counts only if the byte after it starts another
## frame (or the log ends there) and it is at most 256 bytes long; otherwise it is dropped. Main frames
## must also move forward from the last good one, iteration by less than 5000 and time by less than 10 s;
## after a bad one P frames are skipped until the next good I frame. A gap longer than that (a lost
## stretch of the log) is only crossed where a logging-resume event announces it, as the firmware writes
## when logging pauses; elsewhere the rest of the session is refused (stats.desync shows it).
## Data versions 1 and 2 (they differ in the tag8_4s16 encoding) are both read; version 1 follows the
## old format as recalled, no log of it was at hand.
## Values are not wrapped to 32 bit: a column with a value outside the int32 range is a PackedFloat64Array.
##
## Thread-friendly: no scene tree, no autoloads. cancel() may be called from another thread; the
## progress callable (done_bytes: int, total_bytes: int) runs on the decoding thread.

const LOG_START := "H Product:Blackbox flight data recorder by Nicholas Sherlock"
const MAX_FRAME_LENGTH := 256
const MAX_TIME_JUMP_US := 10000000
const MAX_ITERATION_JUMP := 5000
const PAD := 1024                    # zero bytes behind the data: a damaged frame can be read to its end

const ENC_SIGNED_VB := 0
const ENC_UNSIGNED_VB := 1
const ENC_NEG_14BIT := 3
const ENC_TAG8_8SVB := 6
const ENC_TAG2_3S32 := 7
const ENC_TAG8_4S16 := 8
const ENC_NULL := 9
const ENC_TAG2_3SVARIABLE := 10

const PRED_ZERO := 0
const PRED_PREVIOUS := 1
const PRED_STRAIGHT_LINE := 2
const PRED_AVERAGE_2 := 3
const PRED_MINTHROTTLE := 4
const PRED_MOTOR_0 := 5
const PRED_INC := 6
const PRED_HOME_COORD := 7
const PRED_1500 := 8
const PRED_VBATREF := 9
const PRED_LAST_MAIN_FRAME_TIME := 10
const PRED_MINMOTOR := 11

const EV_SYNC_BEEP := 0
const EV_INFLIGHT_ADJUSTMENT := 13
const EV_LOGGING_RESUME := 14
const EV_DISARM := 15
const EV_FLIGHTMODE := 30
const EV_LOG_END := 255

const _I := 0x49
const _P := 0x50
const _S := 0x53
const _G := 0x47
const _H := 0x48
const _E := 0x45
const _PROGRESS_EVERY := 8192        # frames between two calls of the progress callable

## Called as progress.call(done_bytes, total_bytes) while decoding; unset = no progress.
var progress := Callable()
## Set by cancel(); decode() then returns what it has so far.
var cancel_requested := false
## True if the last decode was cut off by cancel().
var was_cancelled := false

var _data := PackedByteArray()
var _limit := 0                      # reading stops here (the padding is behind)
var _vp := 0                         # read position after the last _uvb() / _svb() / _read_event()
var _data_version := 2
var _prev := PackedInt64Array()      # last main frame
var _prev2 := PackedInt64Array()     # the one before
var _minthrottle := 0
var _vbatref := 0
var _min_motor := 0
var _motor0 := -1
var _home := PackedInt64Array([0, 0])
var _last_main_time := 0
var _overflow := false               # the frame just predicted has a value outside the int32 range


## A frame type's field definitions from the headers.
class FrameDef extends RefCounted:
	var names := PackedStringArray()
	var signed := PackedByteArray()
	var pred := PackedInt32Array()
	var enc := PackedInt32Array()
	var grp := PackedInt32Array()    # at the first field of a tag8_8svb / tag2_3* / tag8_4s16 group: its size
	var count := 0


## Growing column-major table of decoded frames.
class Table extends RefCounted:
	var cols: Array[PackedInt32Array] = []
	var rows := 0
	var cap := 0
	var wide_cols := {}              # column -> PackedFloat64Array with its true values (from then on)

	func _init(n: int, capacity: int) -> void:
		cap = maxi(capacity, 16)
		for i in n:
			var c := PackedInt32Array()
			c.resize(cap)
			cols.append(c)

	func grow() -> void:
		cap = cap + (cap >> 1) + 16
		for i in cols.size():
			cols[i].resize(cap)
		for i: int in wide_cols:
			wide_cols[i].resize(cap)

	## Stores the row (values as decoded, all columns); a column that is float64 gets the true value.
	func add_row(row: PackedInt64Array) -> void:
		if rows >= cap:
			grow()
		var r := rows
		for i in cols.size():
			cols[i][r] = row[i]
		if not wide_cols.is_empty():
			for i: int in wide_cols:
				wide_cols[i][r] = row[i]
		rows = r + 1

	## add_row() for a row with a value outside the int32 range: such a column continues as a float64 one.
	func add_row_wide(row: PackedInt64Array) -> void:
		for i in cols.size():
			var v := row[i]
			if (v > 2147483647 or v < -2147483648) and not wide_cols.has(i):
				var f := PackedFloat64Array()
				f.resize(cap)
				for k in rows:
					f[k] = cols[i][k]
				wide_cols[i] = f
		add_row(row)

	## The columns cut to the rows used (a PackedFloat64Array where a value did not fit 32 bits).
	func finish() -> Array:
		var out: Array = []
		for i in cols.size():
			if wide_cols.has(i):
				var f: PackedFloat64Array = wide_cols[i]
				f.resize(rows)
				out.append(f)
			else:
				var c := cols[i]
				c.resize(rows)
				out.append(c)
		cols = []
		wide_cols = {}
		return out


## A session's main column by field name (an empty array if there is no such field).
static func col(session: Dictionary, name: String) -> Variant:
	var i: int = (session.main_names as PackedStringArray).find(name)
	if i < 0:
		return PackedInt32Array()
	return session.main[i]


## Reads and decodes a file; returns the sessions ([] if it can't be read or holds no log).
func decode(path: String) -> Array:
	var bytes := FileAccess.get_file_as_bytes(path)
	if bytes.is_empty():
		return []
	return _decode_all(bytes)


## The same for a log in memory (the array is left alone).
func decode_bytes(bytes: PackedByteArray) -> Array:
	return _decode_all(bytes.duplicate())


func cancel() -> void:
	cancel_requested = true


func _decode_all(bytes: PackedByteArray) -> Array:
	cancel_requested = false
	was_cancelled = false
	var total := bytes.size()
	var pad := PackedByteArray()
	pad.resize(PAD)
	bytes.append_array(pad)
	_data = bytes
	_limit = total + PAD - 64
	var starts := _find_starts(total)
	var sessions: Array = []
	for k in starts.size():
		var end: int = starts[k + 1] if k + 1 < starts.size() else total
		sessions.append(_decode_session(k, starts[k], end, total))
		if was_cancelled:
			break
	if progress.is_valid() and not was_cancelled:
		progress.call(total, total)
	_data = PackedByteArray()
	return sessions


## Offsets of the "H Product:Blackbox flight data recorder" lines.
func _find_starts(total: int) -> PackedInt32Array:
	var starts := PackedInt32Array()
	var marker := LOG_START.to_ascii_buffer()
	var n := marker.size()
	var i := _data.find(_H, 0)
	while i >= 0 and i + n <= total:
		if _data[i + 1] == 0x20 and _data[i + 2] == 0x50 and _data.slice(i, i + n) == marker:
			starts.append(i)
			i = _data.find(_H, i + n)
		else:
			i = _data.find(_H, i + 1)
	return starts


static func _header_int(headers: Dictionary, key: String, fallback: int) -> int:
	var v := str(headers.get(key, "")).get_slice(",", 0).strip_edges()
	return v.to_int() if v.is_valid_int() else fallback


static func _int_list(text: String) -> PackedInt32Array:
	var out := PackedInt32Array()
	if text.strip_edges().is_empty():
		return out
	for part in text.split(","):
		out.append(part.strip_edges().to_int())
	return out


## The field definitions of frame type t ("I", "P", "S", "G", "H") from the headers; P uses I's names.
static func _frame_def(headers: Dictionary, t: String, idef: FrameDef) -> FrameDef:
	var fd := FrameDef.new()
	var key := "Field %s " % t
	if t == "P":
		if idef == null:
			return fd
		fd.names = idef.names
		fd.signed = idef.signed
	else:
		var text: String = headers.get(key + "name", "")
		if not text.strip_edges().is_empty():
			fd.names = PackedStringArray(text.split(","))
		fd.signed.resize(fd.names.size())
		var sg := _int_list(headers.get(key + "signed", ""))
		for i in mini(sg.size(), fd.names.size()):
			fd.signed[i] = 1 if sg[i] != 0 else 0
	fd.pred = _int_list(headers.get(key + "predictor", ""))
	fd.enc = _int_list(headers.get(key + "encoding", ""))
	var n := fd.names.size()
	if fd.pred.size() != n or fd.enc.size() != n:
		return FrameDef.new()        # unusable: frames of this type are not recognised
	fd.count = n
	fd.grp.resize(n)
	fd.grp.fill(1)
	var i := 0
	while i < n:
		var g := 1
		match fd.enc[i]:
			ENC_TAG8_8SVB:
				while i + g < n and g < 8 and fd.enc[i + g] == ENC_TAG8_8SVB:
					g += 1
			ENC_TAG2_3S32, ENC_TAG2_3SVARIABLE:
				g = 3
			ENC_TAG8_4S16:
				g = 4
		fd.grp[i] = g
		i += g
	return fd


func _decode_session(index: int, start: int, end: int, total: int) -> Dictionary:
	var data := _data
	var headers := {}
	var pos := start
	while pos + 1 < end and data[pos] == _H and data[pos + 1] == 0x20:
		var nl := data.find(10, pos)
		if nl < 0 or nl >= end:
			pos = end
			break
		var line := data.slice(pos + 2, nl).get_string_from_utf8()
		var colon := line.find(":")
		if colon > 0:
			headers[line.left(colon)] = line.substr(colon + 1)
		pos = nl + 1
	var idef := _frame_def(headers, "I", null)
	var pdef := _frame_def(headers, "P", idef)
	var sdef := _frame_def(headers, "S", null)
	var gdef := _frame_def(headers, "G", null)
	var hdef := _frame_def(headers, "H", null)
	_data_version = _header_int(headers, "Data version", 2)
	_minthrottle = _header_int(headers, "minthrottle", 0)
	_vbatref = _header_int(headers, "vbatref", 0)
	_min_motor = _header_int(headers, "motorOutput", 0)
	_motor0 = idef.names.find("motor[0]")
	var iter_idx := idef.names.find("loopIteration")
	var time_idx := idef.names.find("time")
	_home = PackedInt64Array([0, 0])
	_last_main_time = 0

	# A P frame is logged where (index % I interval + num - 1) % denom < num; "P interval:8" means 1/8
	var interval_i := maxi(_header_int(headers, "I interval", 32), 1)
	var p_num := 1
	var p_denom := 1
	var pint := str(headers.get("P interval", "1"))
	if "/" in pint:
		p_num = maxi(pint.get_slice("/", 0).to_int(), 1)
		p_denom = maxi(pint.get_slice("/", 1).to_int(), 1)
	else:
		p_denom = maxi(pint.to_int(), 1)

	var is_start := PackedByteArray()    # which bytes begin a frame of this log
	is_start.resize(256)
	is_start.fill(0)
	is_start[_E] = 1
	for pair in [[_I, idef], [_P, pdef], [_S, sdef], [_G, gdef], [_H, hdef]]:
		if (pair[1] as FrameDef).count > 0:
			is_start[pair[0]] = 1

	var main_n := idef.count
	var main := Table.new(main_n, (end - pos) / 24)
	var slow := Table.new(sdef.count, 64)
	var gps := Table.new(gdef.count, 16)
	var slow_at := PackedInt32Array()
	var gps_at := PackedInt32Array()
	var events: Array = []
	var stats := {"I": 0, "P": 0, "S": 0, "E": 0, "G": 0, "H": 0, "corrupt": 0, "desync": 0}
	var cur := PackedInt64Array()
	cur.resize(maxi(main_n, 1))
	_prev = cur.duplicate()
	_prev2 = cur.duplicate()
	var sbuf := PackedInt64Array()
	sbuf.resize(maxi(sdef.count, 1))
	var gbuf := PackedInt64Array()
	gbuf.resize(maxi(gdef.count, 1))
	var hbuf := PackedInt64Array()
	hbuf.resize(maxi(hdef.count, 2))
	var main_valid := false
	var last_iter := -1
	var last_time := -1
	var log_end := false
	var next_report := _PROGRESS_EVERY
	var frames_seen := 0

	while pos < end and main_n > 0:
		var c := data[pos]
		if is_start[c] == 0:
			pos += 1
			continue
		frames_seen += 1
		if frames_seen >= next_report:
			next_report += _PROGRESS_EVERY
			if cancel_requested:
				was_cancelled = true
				break
			if progress.is_valid():
				progress.call(pos, total)
		var fstart := pos
		var q := pos + 1
		_overflow = false
		var is_main := false
		var ev := {}
		match c:
			_I:
				is_main = true
				q = _read_raw(idef, q, cur)
				if q >= 0:
					_predict(idef, cur, false, 0)
			_P:
				is_main = true
				var skipped := 0
				if last_iter >= 0:
					skipped = _skipped_frames(last_iter, interval_i, p_num, p_denom)
				q = _read_raw(pdef, q, cur)
				if q >= 0 and main_valid:
					_predict(pdef, cur, true, skipped)
			_S:
				q = _read_raw(sdef, q, sbuf)
				if q >= 0:
					_predict(sdef, sbuf, false, 0)
			_G:
				q = _read_raw(gdef, q, gbuf)
				if q >= 0:
					_predict(gdef, gbuf, false, 0)
			_H:
				q = _read_raw(hdef, q, hbuf)
				if q >= 0:
					_predict(hdef, hbuf, false, 0)
			_E:
				ev = _read_event(q)
				q = _vp if not ev.is_empty() else -1
		var ok := q >= 0 and q <= end and q - fstart <= MAX_FRAME_LENGTH
		if ok and q < end and is_start[data[q]] == 0 and not (c == _E and ev.type == EV_LOG_END):
			ok = false
		if not ok:
			stats.corrupt += 1
			if is_main:
				main_valid = false
			pos = fstart + 1
			continue
		pos = q
		match c:
			_I, _P:
				var iter := cur[iter_idx] if iter_idx >= 0 else 0
				var t := cur[time_idx] if time_idx >= 0 else 0
				var sane := last_iter < 0 or (iter >= last_iter and iter < last_iter + MAX_ITERATION_JUMP and t >= last_time and t < last_time + MAX_TIME_JUMP_US)
				if c == _P and not (main_valid and sane):
					stats.desync += 1
					continue
				if not sane:
					stats.desync += 1
					main_valid = false
					continue
				main.add_row_wide(cur) if _overflow else main.add_row(cur)
				last_iter = iter
				last_time = t
				_last_main_time = t
				main_valid = true
				if c == _I:
					stats.I += 1
					_prev = cur.duplicate()
					_prev2 = cur.duplicate()
				else:
					stats.P += 1
					var spare := _prev2
					_prev2 = _prev
					_prev = cur
					cur = spare
			_S:
				slow.add_row_wide(sbuf) if _overflow else slow.add_row(sbuf)
				slow_at.append(main.rows)
				stats.S += 1
			_G:
				gps.add_row_wide(gbuf) if _overflow else gps.add_row(gbuf)
				gps_at.append(main.rows)
				stats.G += 1
			_H:
				_home[0] = hbuf[0]
				_home[1] = hbuf[1] if hdef.count > 1 else 0
				stats.H += 1
			_E:
				ev["at"] = main.rows
				events.append(ev)
				stats.E += 1
				if ev.type == EV_LOGGING_RESUME:
					last_iter = ev.iteration
					last_time = ev.time
				elif ev.type == EV_LOG_END:
					log_end = true
					break
	return {
		"index": index,
		"headers": headers,
		"main_names": idef.names,
		"rows": main.rows,
		"main": main.finish(),
		"slow_names": sdef.names,
		"slow": slow.finish(),
		"slow_at": slow_at,
		"events": events,
		"gps_names": gdef.names,
		"gps": gps.finish(),
		"gps_at": gps_at,
		"stats": stats,
		"log_end": log_end,
		"offset": start,
		"size": end - start,
	}


## Frames the logger left out between the last main frame and the next one it wrote.
static func _skipped_frames(last_iter: int, interval_i: int, p_num: int, p_denom: int) -> int:
	var n := 0
	var idx := last_iter + 1
	while n < 1000 and (idx % interval_i + p_num - 1) % p_denom >= p_num:
		n += 1
		idx += 1
	return n


## Reads the encoded values of one frame at p into out; returns the position behind it, -1 if the
## definition has an encoding that can't be read.
func _read_raw(fd: FrameDef, p: int, out: PackedInt64Array) -> int:
	var data := _data
	var enc := fd.enc
	var n := fd.count
	var limit := _limit
	var i := 0
	while i < n:
		if p > limit:
			return -1
		var e := enc[i]
		if e <= ENC_UNSIGNED_VB or e == ENC_NEG_14BIT:
			var b := data[p]
			p += 1
			var v := b
			if b >= 0x80:
				v = b & 0x7F
				var shift := 7
				while true:
					b = data[p]
					p += 1
					if b < 0x80:
						v |= b << shift
						break
					v |= (b & 0x7F) << shift
					shift += 7
					if shift > 28:       # more than 5 bytes: damaged
						v = 0
						break
				v &= 0xFFFFFFFF
			if e == ENC_SIGNED_VB:
				v = (v >> 1) ^ -(v & 1)
			elif e == ENC_NEG_14BIT:
				v = -(((v & 0x3FFF) - 0x4000) if (v & 0x2000) != 0 else v)
			out[i] = v
			i += 1
		elif e == ENC_NULL:
			out[i] = 0
			i += 1
		else:
			var g := fd.grp[i]
			match e:
				ENC_TAG8_8SVB:
					p = _tag8_8svb(p, i, g, n, out)
				ENC_TAG2_3S32:
					p = _tag2_3s32(p, i, n, out)
				ENC_TAG2_3SVARIABLE:
					p = _tag2_3svariable(p, i, n, out)
				ENC_TAG8_4S16:
					p = _tag8_4s16(p, i, n, out)
				_:
					return -1
			i += g
	return p


## Unsigned variable-byte integer at p (7 bits per byte, low group first, high bit = more follows).
func _uvb(p: int) -> int:
	var data := _data
	var v := 0
	var shift := 0
	while shift <= 28:
		var b := data[p]
		p += 1
		v |= (b & 0x7F) << shift
		if b < 0x80:
			_vp = p
			return v & 0xFFFFFFFF
		shift += 7
	_vp = p
	return 0


## Zigzag-coded signed variable-byte integer at p.
func _svb(p: int) -> int:
	var v := _uvb(p)
	return (v >> 1) ^ -(v & 1)


static func _sx(v: int, bits: int) -> int:
	var m := 1 << (bits - 1)
	return ((v & ((m << 1) - 1)) ^ m) - m


## One header byte flags the non-zero fields among up to 8 (bit k = field k), then those as signed VB.
func _tag8_8svb(p: int, i: int, g: int, n: int, out: PackedInt64Array) -> int:
	if g == 1:
		out[i] = _svb(p)
		return _vp
	var hdr: int = _data[p]
	p += 1
	for j in g:
		var v := 0
		if (hdr & (1 << j)) != 0:
			v = _svb(p)
			p = _vp
		if i + j < n:
			out[i + j] = v
	return p


## Three signed fields behind a 2 bit selector: 2, 4 or 6 bits each, or 8 to 32 bits each by a
## selector per field.
func _tag2_3s32(p: int, i: int, n: int, out: PackedInt64Array) -> int:
	var data := _data
	var lead: int = data[p]
	p += 1
	var v0 := 0
	var v1 := 0
	var v2 := 0
	match lead >> 6:
		0:
			v0 = _sx(lead >> 4, 2)
			v1 = _sx(lead >> 2, 2)
			v2 = _sx(lead, 2)
		1:
			v0 = _sx(lead, 4)
			var b: int = data[p]
			p += 1
			v1 = _sx(b >> 4, 4)
			v2 = _sx(b, 4)
		2:
			v0 = _sx(lead, 6)
			v1 = _sx(data[p], 6)
			v2 = _sx(data[p + 1], 6)
			p += 2
		_:
			var sizes := lead
			for j in 3:
				var v := 0
				match sizes & 3:
					0:
						v = _sx(data[p], 8)
						p += 1
					1:
						v = _sx(data[p] | (data[p + 1] << 8), 16)
						p += 2
					2:
						v = _sx(data[p] | (data[p + 1] << 8) | (data[p + 2] << 16), 24)
						p += 3
					_:
						v = _sx(data[p] | (data[p + 1] << 8) | (data[p + 2] << 16) | (data[p + 3] << 24), 32)
						p += 4
				sizes >>= 2
				if j == 0:
					v0 = v
				elif j == 1:
					v1 = v
				else:
					v2 = v
	out[i] = v0
	if i + 1 < n:
		out[i + 1] = v1
	if i + 2 < n:
		out[i + 2] = v2
	return p


## Like tag2_3s32, with 5-5-4 bit and 8-7-7 bit fields in place of 4 and 6 bit ones.
func _tag2_3svariable(p: int, i: int, n: int, out: PackedInt64Array) -> int:
	var data := _data
	var lead: int = data[p]
	var sel := lead >> 6
	if sel == 3:
		return _tag2_3s32(p, i, n, out)
	p += 1
	var v0 := 0
	var v1 := 0
	var v2 := 0
	match sel:
		0:
			v0 = _sx(lead >> 4, 2)
			v1 = _sx(lead >> 2, 2)
			v2 = _sx(lead, 2)
		1:
			var b: int = data[p]
			p += 1
			v0 = _sx(lead >> 1, 5)
			v1 = _sx(((lead & 1) << 4) | (b >> 4), 5)
			v2 = _sx(b, 4)
		_:
			var b1: int = data[p]
			var b2: int = data[p + 1]
			p += 2
			v0 = _sx(((lead & 0x3F) << 2) | (b1 >> 6), 8)
			v1 = _sx(((b1 & 0x3F) << 1) | (b2 >> 7), 7)
			v2 = _sx(b2, 7)
	out[i] = v0
	if i + 1 < n:
		out[i + 1] = v1
	if i + 2 < n:
		out[i + 2] = v2
	return p


## Four signed fields behind a selector byte (2 bits per field: absent, 4, 8 or 16 bits). Data version 2
## packs them as a bit stream (a 4 bit field takes half a byte, the next 8 or 16 bit field continues
## in the other half); version 1 keeps 8 and 16 bit fields byte-aligned and 16 bit little-endian.
func _tag8_4s16(p: int, i: int, n: int, out: PackedInt64Array) -> int:
	var data := _data
	var selector: int = data[p]
	p += 1
	var buffer := 0
	var half := false                    # buffer holds a byte whose low nibble is still unused
	var v2 := _data_version >= 2
	for j in 4:
		var v := 0
		match selector & 3:
			1:
				if not half:
					buffer = data[p]
					p += 1
					v = _sx(buffer >> 4, 4)
					half = true
				else:
					v = _sx(buffer, 4)
					half = false
			2:
				if not half or not v2:
					v = _sx(data[p], 8)
					p += 1
				else:
					var b: int = data[p]
					p += 1
					v = _sx(((buffer & 0x0F) << 4) | (b >> 4), 8)
					buffer = b
			3:
				if not v2:
					v = _sx(data[p] | (data[p + 1] << 8), 16)
					p += 2
				elif not half:
					v = _sx((data[p] << 8) | data[p + 1], 16)
					p += 2
				else:
					var b1: int = data[p]
					var b2: int = data[p + 1]
					p += 2
					v = _sx(((buffer & 0x0F) << 12) | (b1 << 4) | (b2 >> 4), 16)
					buffer = b2
		selector >>= 2
		if i + j < n:
			out[i + j] = v
	return p


## Turns the values read into the field values: adds each field's prediction (no wrapping: a column
## with a value outside the int32 range becomes a PackedFloat64Array, see Table).
func _predict(fd: FrameDef, out: PackedInt64Array, has_history: bool, skipped: int) -> void:
	var pred := fd.pred
	var prev := _prev
	var prev2 := _prev2
	for i in fd.count:
		var v := out[i]
		var pr := pred[i]
		if pr != PRED_ZERO:
			if pr == PRED_PREVIOUS:
				if has_history:
					v += prev[i]
			elif pr == PRED_STRAIGHT_LINE:
				if has_history:
					v += 2 * prev[i] - prev2[i]
			elif pr == PRED_AVERAGE_2:
				if has_history:
					v += (prev[i] + prev2[i]) / 2
			elif pr == PRED_INC:
				v = skipped + 1
				if has_history:
					v += prev[i]
			elif pr == PRED_MINTHROTTLE:
				v += _minthrottle
			elif pr == PRED_MOTOR_0:
				if _motor0 >= 0:
					v += out[_motor0]
			elif pr == PRED_1500:
				v += 1500
			elif pr == PRED_VBATREF:
				v += _vbatref
			elif pr == PRED_MINMOTOR:
				v += _min_motor
			elif pr == PRED_LAST_MAIN_FRAME_TIME:
				v += _last_main_time
			elif pr == PRED_HOME_COORD:
				v += _home[1 if fd.names[i].ends_with("[1]") else 0]
		if v > 2147483647 or v < -2147483648:
			_overflow = true
		out[i] = v


## Event frame after its 'E' at p: {type, ...fields}, empty if damaged or of an unknown kind; _vp = end.
func _read_event(p: int) -> Dictionary:
	var data := _data
	var t: int = data[p]
	p += 1
	var ev := {"type": t}
	match t:
		EV_SYNC_BEEP:
			ev["time"] = _uvb(p)
			p = _vp
		EV_FLIGHTMODE:
			ev["flags"] = _uvb(p)
			ev["last_flags"] = _uvb(_vp)
			p = _vp
		EV_DISARM:
			ev["reason"] = _uvb(p)
			p = _vp
		EV_INFLIGHT_ADJUSTMENT:
			var f: int = data[p]
			p += 1
			ev["func"] = f & 127
			if (f & 128) != 0:
				ev["value"] = data.decode_float(p)
				p += 4
			else:
				ev["value"] = _svb(p)
				p = _vp
		EV_LOGGING_RESUME:
			ev["iteration"] = _uvb(p)
			ev["time"] = _uvb(_vp)
			p = _vp
		EV_LOG_END:
			if data.slice(p, p + 10).get_string_from_ascii() != "End of log" or data[p + 10] != 0:
				return {}
			p += 11
		_:
			return {}
	_vp = p
	return ev
