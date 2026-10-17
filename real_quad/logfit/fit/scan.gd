extends RefCounted
## real_quad logfit, step 0: the quick look at the picked files before the form (contract 2, 3.2-3.3, 7):
## sessions with their flight time and why one is skipped, firmware, the fields logged, the form's
## prefill (cells from the resting pack voltage, KV and motor poles from the header, the craft name)
## and the errors / warnings the pilot sees before building. The files are decoded in worker threads
## (one per file); nothing of the arrays is kept. pack: the form's pack type if known (lipo | lihv | liion),
## "" = not known yet: the cell checks then use the widest band (2.8-4.45 V/cell) and a cell count that
## depends on the pack type is only a warning (pack_ambiguous).

const Num := preload("num.gd")
const Flight := preload("flight.gd")
const Checks := preload("checks.gd")
const Build := preload("build.gd")
const FcConfig := preload("fc_config.gd")


static func inspect(paths: PackedStringArray, builder: Build, pack: String = "") -> Dictionary:
	var errs: Array = []
	var warns: Array = []
	var sessions: Array = []
	var fields := {}
	var fws := {}
	paths = Build.canonical_order(paths)
	var files := builder.load_files(paths, 0.0, 1.0)
	var all_s: Array = []
	errs.append_array(Build.file_errors(files))
	for e in files:
		if e.error == "":
			all_s.append_array(e.sessions)
	var reason := {}
	for L: Flight in all_s:
		reason[L.name] = Checks.skip_reason(L)
	var cand: Array = all_s.filter(func(L): return reason[L.name] == null)
	for L: Flight in cand:
		var e: Variant = Checks.firmware_issue(L)
		if e != null:
			errs.append(e)
			reason[L.name] = e.message
		var nm: int = L.info.get("n_motors", 0)
		if nm != 0 and nm != 4:
			errs.append(Checks.issue("motor_count", "error", "only quads with 4 motors (%s logs %d motors)" % [L.name, nm], "pick logs of a quad"))
	var fails := {}
	for L: Flight in cand:
		fails[L.name] = Checks.required_failures(L)
	for item in Checks.REQUIRED_TEXT:
		var bad: Array = cand.filter(func(L): return fails[L.name].has(item))
		if not cand.is_empty() and bad.size() == cand.size():
			var parts: Array = bad.map(func(L): return "%s: %s" % [L.name, fails[L.name][item]])
			errs.append(Checks.issue("missing_" + item, "error", "%s: %s" % [Checks.REQUIRED_TEXT[item], "; ".join(parts)], Checks.FIX[item]))
		for L: Flight in bad:
			if reason[L.name] == null:
				reason[L.name] = "%s (other sessions have it)" % fails[L.name][item]
	var used: Array = cand.filter(func(L): return reason[L.name] == null)
	if not all_s.is_empty() and used.is_empty() and errs.is_empty():
		var parts: Array = []
		for k in reason:
			parts.append("%s: %s" % [k, str(reason[k])])
		errs.append(Checks.issue("no_usable_session", "error", "no usable session: " + "; ".join(parts), "fly at least one flight of 5 s or more with the quad armed"))
	var armed_total := 0.0
	var chirp_s := 0.0
	for L: Flight in all_s:
		var armed_s := float(L.armed.count(1)) / L.fs if L.n > 0 else 0.0
		var ch := 0.0
		for i in L.n:
			if L.armed[i] != 0 and L.chirp[i] != 0:
				ch += 1.0
		ch = ch / L.fs if L.n > 0 else 0.0
		var ok: bool = reason[L.name] == null
		if ok:
			armed_total += armed_s
			chirp_s += ch
			for f in L.info.get("fields", PackedStringArray()):
				fields[f] = true
			fws[Checks.parse_firmware(L.headers)[3]] = true
		sessions.append({"file": L.path, "index": L.session, "armed_s": Num.pyround(armed_s, 1), "usable": ok,
			"reason": reason[L.name] if reason[L.name] != null else ""})
	if armed_total > Checks.MAX_ARMED_S:
		errs.append(Checks.issue("too_much", "error", "%d min of armed flight (at most 60 min per run): pick fewer logs" % roundi(armed_total / 60.0), "pick fewer logs"))
	if not pack in ["lipo", "lihv", "liion"]:
		pack = ""
	if not used.is_empty():
		for e in Checks.consistency(used, false, pack):
			(errs if e.level == "error" else warns).append(e)
	# prefill
	var prefill := {"cells": null, "kv": null, "motor_poles": null, "craft_name": "", "pack_type": pack if pack != "" else "lipo"}
	if not used.is_empty():
		var src: Flight = used[0]
		var newest := -INF
		for L: Flight in used:
			var t := FcConfig.session_time(L)
			if t >= newest:
				newest = t
				src = L
		prefill.craft_name = str(src.headers.get("Craft name", "")).strip_edges()
		var kv := src.hint("motor_kv", 0)
		if kv > 0:
			prefill.kv = {"value": kv, "check": true, "note": "Betaflight default, check" if kv == 1960 else "check"}
		prefill.motor_poles = src.hint("motor_poles", 14)
		for L: Flight in used:
			var v := Checks.resting_voltage(L)
			if v > 0.0:
				var g := Checks.cells_guess(v, pack)
				if g[0] > 0:
					prefill.cells = {"value": g[0], "check": g[1], "note": "several cell counts fit %.2f V: check" % v if g[1] else ""}
				break
		var acc := used.any(func(L): return L.has_acc())
		var vbat := used.any(func(L): return L.has_vbat())
		if not acc:
			warns.append({"code": "no_acc", "level": "warning", "big": true, "fix": Checks.FIX.acc,
				"message": "No accelerometer (accSmooth) in the logs: hover from the steady rule (about 20 % uncertainty in kT), no drag measurement. Grade at most C."})
		if chirp_s < 30.0:
			warns.append({"code": "no_chirp", "level": "warning", "big": true, "fix": Checks.FIX.chirp,
				"message": "No chirp sweeps (%d s of CHIRP mode, 30 s needed): rotation from freestyle sticks (gain uncertain by 10-40 %%), rate damping 0. Grade at most C." % roundi(chirp_s)})
		if not vbat:
			warns.append({"code": "no_vbat", "level": "warning", "big": true, "fix": Checks.FIX.vbat,
				"message": "No battery voltage (vbatLatest) in the logs: motor voltage estimated per 2-s window, weaker sag fit. Grade at most B."})
		if not used.all(func(L): return L.info.flags.get("unfilt", false)):
			warns.append(Checks.issue("no_unfilt", "info", "Unfiltered gyro (gyroUnfilt) is not logged in every session: the rotation fit uses the filtered gyro there", Checks.FIX.unfilt))
	var fwl: Array = fws.keys()
	fwl.sort()
	return {"ok": errs.is_empty() and not used.is_empty(), "firmware": " / ".join(fwl), "sessions": sessions,
		"fields": fields.keys(), "prefill": prefill, "errors": errs, "warnings": warns}
