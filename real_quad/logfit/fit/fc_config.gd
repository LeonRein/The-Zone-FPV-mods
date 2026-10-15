extends RefCounted
## real_quad logfit: the Betaflight configuration flown in the logs -> fc.json (the settings the
## flight controller port needs). Everything comes from the header of the newest session (falling
## back to the next-newest if its header lacks PIDs or rates), except
##   yaw_motors_reversed  not in the header: from the data (rotation.gd, yaw direction)
##   mincheck             not in the header: Betaflight default 1050
##   rx_rate_hz           Betaflight's smoothed RX frame rate (rc_smoothing_rx_smoothed), else 250

const Flight := preload("flight.gd")


static func _ints(h: Dictionary, k: String, default: Variant = null) -> Variant:
	if not h.has(k) or h[k] == null:
		return default
	var out: Array = []
	for x in str(h[k]).split(","):
		var s := x.strip_edges()
		if not s.is_valid_int():
			return default
		out.append(s.to_int())
	return out


static func _i(h: Dictionary, k: String, default: int = 0) -> int:
	var v: Variant = _ints(h, k)
	return v[0] if v != null and not (v as Array).is_empty() else default


## Header "Log start datetime" if its year >= 2000 (UTC seconds), else the file's modification time.
static func session_time(L: Flight) -> float:
	var re := RegEx.create_from_string("^(\\d{4})-(\\d\\d)-(\\d\\d)T(\\d\\d):(\\d\\d):(\\d\\d)")
	var m := re.search(str(L.headers.get("Log start datetime", "")))
	if m != null and m.get_string(1).to_int() >= 2000:
		var d := {"year": m.get_string(1).to_int(), "month": m.get_string(2).to_int(), "day": m.get_string(3).to_int(),
			"hour": m.get_string(4).to_int(), "minute": m.get_string(5).to_int(), "second": m.get_string(6).to_int()}
		return float(Time.get_unix_time_from_datetime_dict(d))
	return float(FileAccess.get_modified_time(L.path))


## Newest session whose header has PIDs and rates (else the newest): the tune the craft flies now.
static func pick_source(logs: Array) -> Flight:
	var order: Array = range(logs.size())
	var times: Array = logs.map(func(L): return session_time(L))
	order.sort_custom(func(a: int, b: int) -> bool: return times[a] > times[b] or (times[a] == times[b] and a > b))
	for i in order:
		if (from_headers(logs[i].headers, "")[1] as Array).is_empty():
			return logs[i]
	return logs[order[0]]


## [fc dict, missing keys]; craft_name: the form's name (not the header's).
static func from_headers(h: Dictionary, craft_name: String, yaw_motors_reversed: bool = true) -> Array:
	var rx := _i(h, "rc_smoothing_rx_smoothed", 0)
	var cfg := {
		"firmware": h.get("Firmware revision", "?"), "craft": craft_name,
		"looptime_us": _i(h, "looptime", 125), "pid_process_denom": _i(h, "pid_process_denom", 1),
		"pid": {"roll": _ints(h, "rollPID"), "pitch": _ints(h, "pitchPID"), "yaw": _ints(h, "yawPID")},
		"ff_weight": _ints(h, "ff_weight", [0, 0, 0]), "d_max": _ints(h, "d_max", [0, 0, 0]),
		"d_max_gain": _i(h, "d_max_gain"), "d_max_advance": _i(h, "d_max_advance"),
		"rates_type": _i(h, "rates_type", 3), "rc_rates": _ints(h, "rc_rates"), "rates": _ints(h, "rates"),
		"rc_expo": _ints(h, "rc_expo"), "rate_limits": _ints(h, "rate_limits", [1998, 1998, 1998]),
		"thr_mid": _i(h, "thr_mid", 50), "thr_expo": _i(h, "thr_expo"), "thr_hover": _i(h, "thr_hover", 50),
		"deadband": _i(h, "deadband"), "yaw_deadband": _i(h, "yaw_deadband"), "mincheck": 1050,
		"rc_smoothing": _i(h, "rc_smoothing", 1), "rc_smoothing_auto_factor": _i(h, "rc_smoothing_auto_factor", 30),
		"rc_smoothing_auto_factor_throttle": _i(h, "rc_smoothing_auto_factor_throttle", 30),
		"rc_smoothing_setpoint_cutoff": _i(h, "rc_smoothing_setpoint_cutoff"),
		"rc_smoothing_throttle_cutoff": _i(h, "rc_smoothing_throttle_cutoff"),
		"rx_rate_hz": float(rx) if 20 <= rx and rx <= 2000 else 250.0, "serialrx_crsf": _i(h, "serialrx_provider") == 9,
		"feedforward_averaging": _i(h, "feedforward_averaging"), "feedforward_smooth_factor": _i(h, "feedforward_smooth_factor"),
		"feedforward_jitter_factor": _i(h, "feedforward_jitter_factor"), "feedforward_boost": _i(h, "feedforward_boost"),
		"feedforward_max_rate_limit": _i(h, "feedforward_max_rate_limit", 100), "feedforward_transition": _i(h, "feedforward_transition"),
		"feedforward_yaw_hold_gain": _i(h, "feedforward_yaw_hold_gain"), "feedforward_yaw_hold_time": _i(h, "feedforward_yaw_hold_time", 100),
		"iterm_relax": _i(h, "iterm_relax", 1), "iterm_relax_type": _i(h, "iterm_relax_type", 1),
		"iterm_relax_cutoff": _i(h, "iterm_relax_cutoff", 15),
		"iterm_windup": _i(h, "iterm_windup", 85), "pidsum_limit": _i(h, "pidsum_limit", 500),
		"pidsum_limit_yaw": _i(h, "pidsum_limit_yaw", 400),
		"anti_gravity_gain": _i(h, "anti_gravity_gain"), "anti_gravity_cutoff_hz": _i(h, "anti_gravity_cutoff_hz", 5),
		"anti_gravity_p_gain": _i(h, "anti_gravity_p_gain"),
		"tpa_mode": _i(h, "tpa_mode"), "tpa_rate": _i(h, "tpa_rate"), "tpa_breakpoint": _i(h, "tpa_breakpoint", 1350),
		"tpa_low_rate": _i(h, "tpa_low_rate"), "tpa_low_breakpoint": _i(h, "tpa_low_breakpoint", 1050),
		"tpa_low_always": _i(h, "tpa_low_always"),
		"throttle_boost": _i(h, "throttle_boost"), "throttle_boost_cutoff": _i(h, "throttle_boost_cutoff", 15),
		"thrust_linear": _i(h, "thrust_linear"), "motor_output_limit": _i(h, "motor_output_limit", 100),
		"motor_idle": _i(h, "motor_idle", 550), "dyn_idle_min_rpm": _i(h, "dyn_idle_min_rpm"),
		"dyn_idle_p_gain": _i(h, "dyn_idle_p_gain"), "dyn_idle_i_gain": _i(h, "dyn_idle_i_gain"),
		"dyn_idle_d_gain": _i(h, "dyn_idle_d_gain"), "dyn_idle_max_increase": _i(h, "dyn_idle_max_increase"),
		"airmode": bool((_i(h, "features") >> 22) & 1), "yaw_motors_reversed": yaw_motors_reversed,
		"gyro_lpf1_static_hz": _i(h, "gyro_lpf1_static_hz"), "gyro_lpf1_type": _i(h, "gyro_lpf1_type"),
		"gyro_lpf2_static_hz": _i(h, "gyro_lpf2_static_hz"), "gyro_lpf2_type": _i(h, "gyro_lpf2_type"),
		"dterm_lpf1_dyn_hz": _ints(h, "dterm_lpf1_dyn_hz", [0, 0]), "dterm_lpf1_dyn_expo": _i(h, "dterm_lpf1_dyn_expo"),
		"dterm_lpf1_static_hz": _i(h, "dterm_lpf1_static_hz"), "dterm_lpf1_type": _i(h, "dterm_lpf1_type"),
		"dterm_lpf2_static_hz": _i(h, "dterm_lpf2_static_hz"), "dterm_lpf2_type": _i(h, "dterm_lpf2_type"),
		"yaw_lowpass_hz": _i(h, "yaw_lowpass_hz"),
		"rpm_filter_harmonics": _i(h, "rpm_filter_harmonics"), "rpm_filter_min_hz": _i(h, "rpm_filter_min_hz", 100),
		"rpm_filter_fade_range_hz": _i(h, "rpm_filter_fade_range_hz", 50), "rpm_filter_q": _i(h, "rpm_filter_q", 500),
		"rpm_filter_lpf_hz": _i(h, "rpm_filter_lpf_hz", 150), "rpm_filter_weights": _ints(h, "rpm_filter_weights", [100, 100, 100]),
		"motor_poles": _i(h, "motor_poles", 14), "motor_kv": _i(h, "motor_kv", 0),
	}
	var missing: Array = []
	for k in ["pid", "rc_rates", "rates", "rc_expo"]:
		if k == "pid":
			if cfg.pid.values().has(null):
				missing.append(k)
		elif cfg[k] == null or (cfg[k] as Array).is_empty():
			missing.append(k)
	return [cfg, missing]


## Header keys that differ between the logs (the tune changed between flights): {key: [values]}.
static func tune_differences(logs: Array) -> Dictionary:
	var out := {}
	for k in ["rollPID", "pitchPID", "yawPID", "rc_rates", "rates", "ff_weight", "d_max", "looptime", "pid_process_denom"]:
		var vals := {}
		for L in logs:
			vals[str(L.headers.get(k)) if L.headers.has(k) else "None"] = true
		if vals.size() > 1:
			var v: Array = vals.keys()
			v.sort()
			out[k] = v
	return out
