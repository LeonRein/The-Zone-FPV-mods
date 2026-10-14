# Betaflight 2026.6.2 rate loop (acro mode, quad X) - GDScript port.
#
# Ported function by function from the firmware sources at commit e0b7bb01b:
#   fc/rc.c (rates, RC smoothing, feedforward), flight/pid.c + pid_init.c (PID, I-term relax,
#   anti-gravity, TPA, D-max), flight/mixer.c + mixer_init.c (airmode mixer, dynamic idle,
#   throttle boost), drivers/dshot.c (endpoints), sensors/gyro + flight/rpm_filter.c (gyro
#   filters, RPM notches), common/filter.c (PT1/PT2/PT3, SVF notch).
# Settings are read from the craft's Betaflight configuration (blackbox header, JSON).
#
# Style note: the code deliberately stays close to the C source - same function boundaries, same
# variable names, filter states as flat packed arrays instead of filter objects - so every line
# can be checked against the firmware, and to keep the 4 kHz loop cheap in GDScript. A Python
# reference implementation of the same port, validated open-loop against logged P/I/D/F terms,
# is compared with this file sample by sample (not part of the mod package).
#
# Units as in the firmware: deg/s, rcCommand -500..500, rcData 1000..2000, DShot 48..2047.
# Axes: Betaflight gyro axes (roll+ right side down, pitch+ nose down, yaw+ nose left).
# Betaflight is GPL-3.0; this port is a derivative work and GPL-3.0 as well.
# (No class_name: the scripts are always loaded by preload(), so the same files can live under
# different res:// paths - mod pack, test project - without clashing global class names.)
extends RefCounted

const PTERM_SCALE := 0.032029
const ITERM_SCALE := 0.244381
const DTERM_SCALE := 0.000529
const FEEDFORWARD_SCALE := 0.013754
const ITERM_RELAX_SETPOINT_THRESHOLD := 40.0
const ANTIGRAVITY_KI := 0.34
const ANTIGRAVITY_KP := 0.0034
const PID_MIXER_SCALING := 1000.0
const DSHOT_MIN := 48.0
const DSHOT_MAX := 2047.0
const DSHOT_RANGE := DSHOT_MAX - DSHOT_MIN
const D_MAX_RANGE_HZ := 85.0
const D_MAX_LOWPASS_HZ := 35.0
const D_MAX_GAIN_FACTOR := 0.00008
const RX_INTERVAL_MIN_US := 800
const RX_INTERVAL_MAX_US := 65500
const CUTOFF_CORRECTION_PT2 := 1.553773974
const CUTOFF_CORRECTION_PT3 := 1.961459177
# quad X mixer: roll, pitch, yaw columns. 0 REAR_R, 1 FRONT_R, 2 REAR_L, 3 FRONT_L
const MIX_ROLL := [-1.0, -1.0, 1.0, 1.0]
const MIX_PITCH := [1.0, -1.0, 1.0, -1.0]
const MIX_YAW := [-1.0, 1.0, 1.0, -1.0]

var cfg: Dictionary
var gyro_dt: float
var dt: float
var pid_freq: float
var t_us: int = 0

# ---- rc
var rc_command := PackedFloat64Array([0, 0, 0, 1000])
var raw_setpoint := PackedFloat64Array([0, 0, 0])
var setpoint_rate := PackedFloat64Array([0, 0, 0])
var rc_deflection := PackedFloat64Array([0, 0, 0])
var rc_deflection_abs := PackedFloat64Array([0, 0, 0])
var max_rc_rate := PackedFloat64Array([0, 0, 0])
var throttle_lut := PackedFloat64Array()
var rx_data_new := false
var last_rx_time_us: int = 0
var current_rx_interval_us: float = 4000.0
var current_rx_rate_hz: float = 250.0
var smoothed_rx_rate_hz: float = 100.0
var rx_rate_valid := false
var _valid_count := 0
var _outlier_count := 0
var _prev_outlier_sign := 0
var rx_to_smooth := PackedFloat64Array([0, 0, 0, 0])
var rc_throttle_smoothed: float = 1000.0
var auto_sp: float
var auto_thr: float
# PT3 states: setpoint (4 channels) and feedforward (3)
var sp_k := PackedFloat64Array([1, 1, 1, 1])
var sp_s1 := PackedFloat64Array([0, 0, 0, 0])
var sp_s2 := PackedFloat64Array([0, 0, 0, 0])
var sp_s := PackedFloat64Array([0, 0, 0, 0])
var ffs_k: float = 1.0
var ffs_s1 := PackedFloat64Array([0, 0, 0])
var ffs_s2 := PackedFloat64Array([0, 0, 0])
var ffs_s := PackedFloat64Array([0, 0, 0])
# feedforward
var ff_smooth_tau: float
var ff_jitter_inv: float
var ff_boost: float
var ff_max_rate_limit: float
var ff_interpolate: bool
var ff_transition: float
var ff_transition_inv: float
var ff_yaw_hold_time: float
var ff_yaw_hold_gain: float
var ff_prev_rc := PackedFloat64Array([0, 0, 0])
var ff_prev_sp := PackedFloat64Array([0, 0, 0])
var ff_prev_sp_speed := PackedFloat64Array([0, 0, 0])
var ff_prev_sp_speed_delta := PackedFloat64Array([0, 0, 0])
var ff_prev_rc_delta_abs := PackedFloat64Array([0, 0, 0])
var ff_prev_dup := [false, false, false]
var ff_prev_rx_interval := PackedFloat64Array([0, 0, 0])
var ff_speed_k: float = 1.0
var ff_speed_s := PackedFloat64Array([0, 0, 0])
var ff_delta_s := PackedFloat64Array([0, 0, 0])
var ff_yaw_hold_s: float = 0.0
var ff_raw := PackedFloat64Array([0, 0, 0])
var ff_smoothed := PackedFloat64Array([0, 0, 0])

# ---- pid
var kp := PackedFloat64Array([0, 0, 0])
var ki := PackedFloat64Array([0, 0, 0])
var kd := PackedFloat64Array([0, 0, 0])
var kf := PackedFloat64Array([0, 0, 0])
var iterm_limit: float
var iterm_limit_yaw: float
var ag_gain: float
var ag_p_gain: float
var ag_k: float
var ag_s1: float = 0.0
var ag_s: float = 0.0
var ag_throttle_d: float = 0.0
var iterm_accel: float = 0.0
var ag_prev_throttle: float = 0.0
var iterm_relax: int
var iterm_relax_type: int
var windup_k: float
var windup_s := PackedFloat64Array([0, 0, 0])
var dmax_percent := PackedFloat64Array([1, 1, 1])
var dmax_gyro_gain: float
var dmax_sp_gain: float
var dmax_range_k: float
var dmax_range_s1 := PackedFloat64Array([0, 0, 0])
var dmax_range_s := PackedFloat64Array([0, 0, 0])
var dmax_lpf_k: float
var dmax_lpf_s1 := PackedFloat64Array([0, 0, 0])
var dmax_lpf_s := PackedFloat64Array([0, 0, 0])
# dterm lowpass 1 (type, k, 3 states per axis) and 2
var dl1_type: int
var dl1_k: float = 1.0
var dl1_on := false
var dl1_s := PackedFloat64Array([0, 0, 0, 0, 0, 0, 0, 0, 0])   # 3 axes x 3 stages
var dl2_type: int
var dl2_k: float = 1.0
var dl2_on := false
var dl2_s := PackedFloat64Array([0, 0, 0, 0, 0, 0, 0, 0, 0])
var yaw_p_lpf_on := false
var yaw_p_lpf_k: float = 1.0
var yaw_p_lpf_s: float = 0.0
var prev_gyro_dterm := PackedFloat64Array([0, 0, 0])
var dyn_lpf_min: float
var dyn_lpf_max: float
var dyn_lpf_expo: float
var dyn_lpf_prev_q: int = -1
var dyn_lpf_last_us: int = -1000000000
# tpa
var tpa_bp: float
var tpa_mult: float
var tpa_low_bp: float
var tpa_low_mult: float
var tpa_low_always: int
var tpa_low_faded := false
var tpa_factor: float = 1.0
var tpa_mode: int
var P := PackedFloat64Array([0, 0, 0])
var I := PackedFloat64Array([0, 0, 0])
var D := PackedFloat64Array([0, 0, 0])
var F := PackedFloat64Array([0, 0, 0])
var pid_sum := PackedFloat64Array([0, 0, 0])
var stabilisation := true
var iterm_reset := true

# ---- gyro
var gl1_type: int
var gl1_on := false
var gl1_k: float = 1.0
var gl1_s := PackedFloat64Array([0, 0, 0, 0, 0, 0, 0, 0, 0])
var gl2_type: int
var gl2_on := false
var gl2_k: float = 1.0
var gl2_s := PackedFloat64Array([0, 0, 0, 0, 0, 0, 0, 0, 0])
var gyro_filtered := PackedFloat64Array([0, 0, 0])
var use_rpm_filter := true
var rpm_harm: int
var rpm_min_hz: float
var rpm_max_hz: float
var rpm_fade: float
var rpm_q: float
var rpm_weights := PackedFloat64Array()
# rpm notches: index n = motor*harm + h ; coefficients and 3-axis states
var rn_a1 := PackedFloat64Array()
var rn_a2 := PackedFloat64Array()
var rn_f := PackedFloat64Array()
var rn_wq := PackedFloat64Array()
var rn_ic1 := PackedFloat64Array()   # n*3 + axis
var rn_ic2 := PackedFloat64Array()
var motor_freq_k: float
var motor_hz := PackedFloat64Array([0, 0, 0, 0])

# ---- mixer
var out_high: float
var out_low: float
var dyn_idle_min_rps: float
var dyn_p: float
var dyn_i_gain: float
var dyn_d: float
var dyn_max_inc: float
var dyn_start_inc: float
var min_rps_delay_k: float
var dyn_i: float = 0.0
var prev_min_rps: float = 0.0
var throttle_boost: float
var throttle_lpf_k: float
var throttle_lpf_s: float = 0.0
var thrust_lin: float
var airmode: bool
var yaw_reversed: bool
var pidsum_limit: float
var pidsum_limit_yaw: float
var motor := PackedFloat64Array([0, 0, 0, 0])
var mixer_throttle: float = 0.0
var motor_mix_range: float = 0.0

var armed := false
var throttle_raised := false
var airmode_active := false


static func pt1_gain(f_cut: float, p_dt: float) -> float:
	var omega := 2.0 * PI * f_cut * p_dt
	return omega / (omega + 1.0)


static func pt1_gain_from_delay(delay: float, p_dt: float) -> float:
	return 1.0 if delay <= 0.0 else p_dt / (p_dt + delay)


static func pt2_gain(f: float, p_dt: float) -> float:
	return pt1_gain(f * CUTOFF_CORRECTION_PT2, p_dt)


static func pt3_gain(f: float, p_dt: float) -> float:
	return pt1_gain(f * CUTOFF_CORRECTION_PT3, p_dt)


## Lowpass setup: returns [enabled, stages, gain]. FILTER_PT1=0, SVF=1 (approximated by PT2), PT2=2, PT3=3.
static func _lp_setup(ftype: int, hz: float, p_dt: float) -> Array:
	if hz <= 0.0:
		return [false, 1, 1.0]
	if ftype == 0:
		return [true, 1, pt1_gain(hz, p_dt)]
	if ftype == 1 or ftype == 2:
		return [true, 2, pt2_gain(hz, p_dt)]
	return [true, 3, pt3_gain(hz, p_dt)]


## Apply a 1-3 stage PT filter on the states s[base..base+2]; s[base] is always the output.
static func _lp_apply(s: PackedFloat64Array, base: int, stages: int, k: float, x: float) -> float:
	if stages == 1:
		s[base] += k * (x - s[base])
		return s[base]
	if stages == 2:
		s[base + 1] += k * (x - s[base + 1])
		s[base] += k * (s[base + 1] - s[base])
		return s[base]
	s[base + 1] += k * (x - s[base + 1])
	s[base + 2] += k * (s[base + 1] - s[base + 2])
	s[base] += k * (s[base + 2] - s[base])
	return s[base]


func _init(p_cfg: Dictionary, p_use_rpm_filter: bool = true) -> void:
	cfg = p_cfg
	gyro_dt = float(cfg.looptime_us) * 1e-6
	dt = gyro_dt * float(cfg.pid_process_denom)
	pid_freq = 1.0 / dt
	for a in 3:
		max_rc_rate[a] = apply_rates(a, 1.0, 1.0)
	_build_throttle_lut()
	auto_sp = 1.5 / (1.0 + float(cfg.rc_smoothing_auto_factor) / 10.0)
	auto_thr = 1.5 / (1.0 + float(cfg.rc_smoothing_auto_factor_throttle) / 10.0)
	var sf := 0.01 * float(cfg.feedforward_smooth_factor)
	ff_smooth_tau = (1.0 / 250.0 * sf) / (1.0 - sf)
	ff_jitter_inv = 1.0 / (1.0 + float(cfg.feedforward_jitter_factor))
	ff_boost = 0.001 * float(cfg.feedforward_boost)
	ff_max_rate_limit = float(cfg.feedforward_max_rate_limit)
	ff_interpolate = not bool(cfg.serialrx_crsf)
	ff_transition = float(cfg.feedforward_transition) / 100.0
	ff_transition_inv = 0.0 if int(cfg.feedforward_transition) == 0 else 100.0 / float(cfg.feedforward_transition)
	ff_yaw_hold_time = 0.001 * float(cfg.feedforward_yaw_hold_time)
	ff_yaw_hold_gain = float(cfg.feedforward_yaw_hold_gain)
	if int(cfg.feedforward_yaw_hold_time) < 100:
		ff_yaw_hold_gain *= 150.0 / (float(cfg.feedforward_yaw_hold_time) + 50.0)
	_update_ff_filters()
	_update_smoothing_cutoffs()
	# pid
	var axes := ["roll", "pitch", "yaw"]
	for a in 3:
		var pid: Array = cfg.pid[axes[a]]
		kp[a] = PTERM_SCALE * float(pid[0])
		ki[a] = ITERM_SCALE * float(pid[1])
		kd[a] = DTERM_SCALE * float(pid[2])
		kf[a] = FEEDFORWARD_SCALE * (float(cfg.ff_weight[a]) * 0.01)
		var dv := float(pid[2])
		var dm := float(cfg.d_max[a])
		dmax_percent[a] = dm / dv if dv > 0.0 and dm > dv else 1.0
	ki[2] *= 2.5
	iterm_limit = 0.01 * float(cfg.iterm_windup) * float(cfg.pidsum_limit)
	iterm_limit_yaw = 0.01 * float(cfg.iterm_windup) * float(cfg.pidsum_limit_yaw)
	ag_gain = float(cfg.anti_gravity_gain)
	ag_p_gain = float(cfg.anti_gravity_p_gain) / 100.0 * ANTIGRAVITY_KP
	ag_k = pt2_gain(float(cfg.anti_gravity_cutoff_hz), dt)
	iterm_relax = int(cfg.iterm_relax)
	iterm_relax_type = int(cfg.iterm_relax_type)
	windup_k = pt1_gain(float(cfg.iterm_relax_cutoff), dt)
	dmax_gyro_gain = D_MAX_GAIN_FACTOR * float(cfg.d_max_gain) / D_MAX_LOWPASS_HZ
	dmax_sp_gain = D_MAX_GAIN_FACTOR * float(cfg.d_max_advance) / D_MAX_LOWPASS_HZ
	dmax_range_k = pt2_gain(D_MAX_RANGE_HZ, dt)
	dmax_lpf_k = pt2_gain(D_MAX_LOWPASS_HZ, dt)
	dyn_lpf_min = float(cfg.dterm_lpf1_dyn_hz[0])
	dyn_lpf_max = float(cfg.dterm_lpf1_dyn_hz[1])
	dyn_lpf_expo = float(cfg.dterm_lpf1_dyn_expo)
	var l1hz := dyn_lpf_min if dyn_lpf_min > 0.0 else float(cfg.dterm_lpf1_static_hz)
	var s1 := _lp_setup(int(cfg.dterm_lpf1_type), l1hz, dt)
	dl1_on = s1[0]; dl1_type = s1[1]; dl1_k = s1[2]
	var s2 := _lp_setup(int(cfg.dterm_lpf2_type), float(cfg.dterm_lpf2_static_hz), dt)
	dl2_on = s2[0]; dl2_type = s2[1]; dl2_k = s2[2]
	var sy := _lp_setup(0, float(cfg.yaw_lowpass_hz), dt)
	yaw_p_lpf_on = sy[0]; yaw_p_lpf_k = sy[2]
	tpa_bp = clampf((float(cfg.tpa_breakpoint) - 1000.0) / 1000.0, 0.0, 0.99)
	tpa_mult = (float(cfg.tpa_rate) / 100.0) / (1.0 - tpa_bp)
	tpa_low_bp = minf(clampf((float(cfg.tpa_low_breakpoint) - 1000.0) / 1000.0, 0.01, 1.0), tpa_bp)
	tpa_low_mult = float(cfg.tpa_low_rate) / (100.0 * tpa_low_bp)
	tpa_low_always = int(cfg.tpa_low_always)
	tpa_mode = int(cfg.tpa_mode)
	# gyro
	var g1 := _lp_setup(int(cfg.gyro_lpf1_type), float(cfg.gyro_lpf1_static_hz), gyro_dt)
	gl1_on = g1[0]; gl1_type = g1[1]; gl1_k = g1[2]
	var g2 := _lp_setup(int(cfg.gyro_lpf2_type), float(cfg.gyro_lpf2_static_hz), gyro_dt)
	gl2_on = g2[0]; gl2_type = g2[1]; gl2_k = g2[2]
	rpm_harm = int(cfg.rpm_filter_harmonics)
	use_rpm_filter = p_use_rpm_filter and rpm_harm > 0
	rpm_min_hz = float(cfg.rpm_filter_min_hz)
	rpm_max_hz = 0.48 / dt
	rpm_fade = float(cfg.rpm_filter_fade_range_hz)
	rpm_q = float(cfg.rpm_filter_q) / 100.0
	for w in cfg.rpm_filter_weights:
		rpm_weights.append(clampf(float(w) / 100.0, 0.0, 1.0))
	var nn := 4 * rpm_harm
	rn_a1.resize(nn); rn_a2.resize(nn); rn_f.resize(nn); rn_wq.resize(nn)
	rn_ic1.resize(nn * 3); rn_ic2.resize(nn * 3)
	rn_ic1.fill(0.0); rn_ic2.fill(0.0)
	for m in 4:
		for h in rpm_harm:
			_rpm_notch_update(m * rpm_harm + h, rpm_min_hz * float(h + 1), 0.0)
	motor_freq_k = pt1_gain(float(cfg.rpm_filter_lpf_hz), dt)
	# mixer
	var lim := float(cfg.motor_output_limit) / 100.0 if int(cfg.motor_output_limit) < 100 else 1.0
	out_high = DSHOT_MAX - DSHOT_RANGE * (1.0 - lim)
	out_low = DSHOT_MIN + 0.01 * (float(cfg.motor_idle) * 0.01) * DSHOT_RANGE
	dyn_idle_min_rps = float(cfg.dyn_idle_min_rpm) * 100.0 / 60.0
	if dyn_idle_min_rps > 0.0:
		out_low = DSHOT_MIN
	dyn_p = float(cfg.dyn_idle_p_gain) * 0.00015
	dyn_i_gain = float(cfg.dyn_idle_i_gain) * 0.01 * dt
	dyn_d = float(cfg.dyn_idle_d_gain) * 0.0000003 * pid_freq
	dyn_max_inc = float(cfg.dyn_idle_max_increase) * 0.001
	dyn_start_inc = float(cfg.motor_idle) * 0.0001
	min_rps_delay_k = 800.0 * dt / 20.0
	throttle_boost = float(cfg.throttle_boost) * 0.1
	throttle_lpf_k = pt1_gain(float(cfg.throttle_boost_cutoff), dt)
	thrust_lin = float(cfg.thrust_linear) / 100.0
	airmode = bool(cfg.airmode)
	yaw_reversed = bool(cfg.yaw_motors_reversed)
	pidsum_limit = float(cfg.pidsum_limit)
	pidsum_limit_yaw = float(cfg.pidsum_limit_yaw)
	motor.fill(0.0)


# ------------------------------------------------------------------ rc.c
func apply_rates(axis: int, rc: float, rc_abs: float) -> float:
	if int(cfg.rates_type) == 3:   # ACTUAL
		var expof := float(cfg.rc_expo[axis]) / 100.0
		expof = rc_abs * (pow(rc, 5) * expof + rc * (1.0 - expof))
		var center := float(cfg.rc_rates[axis]) * 10.0
		var stick_mov := maxf(0.0, float(cfg.rates[axis]) * 10.0 - center)
		return rc * center + stick_mov * expof
	# BETAFLIGHT rates
	if int(cfg.rc_expo[axis]) != 0:
		var e := float(cfg.rc_expo[axis]) / 100.0
		rc = rc * pow(rc_abs, 3) * e + rc * (1.0 - e)
	var rr := float(cfg.rc_rates[axis]) / 100.0
	if rr > 2.0:
		rr += 14.54 * (rr - 2.0)
	var ang := 200.0 * rr * rc
	if int(cfg.rates[axis]) != 0:
		ang *= 1.0 / clampf(1.0 - rc_abs * float(cfg.rates[axis]) / 100.0, 0.01, 1.0)
	return ang


static func _bez(x: float, p0x: float, p1x: float, p2x: float, p0y: float, p1y: float, p2y: float) -> float:
	var a := p0x - 2.0 * p1x + p2x
	var b := 2.0 * p1x - 2.0 * p0x
	var c := p0x - x
	var t := 0.0
	if absf(a) < 1e-6:
		if absf(b) > 1e-6:
			t = -c / b
	else:
		var disc := b * b - 4.0 * a * c
		if disc >= 0.0:
			var s := sqrt(disc)
			var t1 := (-b + s) / (2.0 * a)
			var t2 := (-b - s) / (2.0 * a)
			t = t1 if (t1 >= 0.0 and t1 <= 1.0) else t2
	t = clampf(t, 0.0, 1.0)
	return (1.0 - t) * (1.0 - t) * p0y + 2.0 * (1.0 - t) * t * p1y + t * t * p2y


func _build_throttle_lut() -> void:
	var mid := float(cfg.thr_mid) / 100.0
	var expo := float(cfg.thr_expo) / 100.0
	var hov := float(cfg.thr_hover) / 100.0
	var cp1x := mid * 0.5
	var cp1y := hov * 0.5 * (1.0 + expo)
	var cp2x := (1.0 + mid) * 0.5
	var cp2y := 1.0 + (hov - 1.0) * 0.5 * (1.0 + expo)
	throttle_lut.resize(12)
	for i in 12:
		var x := float(i) / 11.0
		var y := _bez(x, 0.0, cp1x, mid, 0.0, cp1y, hov) if x <= mid else _bez(x, mid, cp2x, 1.0, hov, cp2y, 1.0)
		throttle_lut[i] = float(roundi(1000.0 + y * 1000.0))


func _throttle_lookup(tmp: int) -> float:
	var steps := 11
	var scaled := tmp * steps
	var idx := scaled / 1000
	var rem := float(scaled % 1000)
	if idx >= steps:
		return throttle_lut[steps]
	if idx < 0:
		return throttle_lut[0]
	var lo := throttle_lut[idx]
	var hi := throttle_lut[idx + 1]
	return lo + (hi - lo) * rem / 1000.0


## New RC frame. rc_data = [roll, pitch, yaw, throttle] in 1000..2000 (AETR order).
func rx_frame(rc_data: PackedFloat64Array, frame_t_us: int) -> void:
	var delta: int = frame_t_us - last_rx_time_us if last_rx_time_us != 0 else 0
	last_rx_time_us = frame_t_us
	current_rx_interval_us = clampf(float(delta), RX_INTERVAL_MIN_US, RX_INTERVAL_MAX_US)
	current_rx_rate_hz = 1e6 / current_rx_interval_us
	rx_rate_valid = float(delta) == current_rx_interval_us
	rx_data_new = true
	for a in 3:
		var v := clampf(rc_data[a] - 1500.0, -500.0, 500.0)
		if a == 2:
			v = -v
		rc_command[a] = v
	var mc := float(cfg.mincheck)
	var tmp := clampf(rc_data[3], mc, 2000.0)
	rc_command[3] = _throttle_lookup(int((tmp - mc) * 1000.0 / (2000.0 - mc)))
	var pct := clampf((rc_data[3] - mc) * 100.0 / (2000.0 - mc), 0.0, 100.0)
	if armed:
		if pct >= 25.0:
			throttle_raised = true
		if airmode:
			airmode_active = throttle_raised
	var throttle_active := rc_data[3] >= mc
	if armed and (airmode_active or throttle_active):
		iterm_reset = false
	else:
		iterm_reset = true
	stabilisation = true   # pid_at_min_throttle = 1


func _should_update_smoothing() -> bool:
	if rx_rate_valid:
		var d := current_rx_rate_hz - smoothed_rx_rate_hz
		if absf(d) > smoothed_rx_rate_hz * 0.2:
			var sgn := -1 if d < 0.0 else 1
			if _outlier_count == 0:
				_prev_outlier_sign = sgn
				_outlier_count += 1
			elif sgn != _prev_outlier_sign:
				_outlier_count = 0
				_prev_outlier_sign = sgn
			else:
				_outlier_count += 1
			_valid_count = 0
		else:
			smoothed_rx_rate_hz += 0.1 * (current_rx_rate_hz - smoothed_rx_rate_hz)
			_valid_count += 1
			_outlier_count = 0
		if _valid_count >= 3:
			_valid_count = 0
			return true
		if _outlier_count >= 3:
			smoothed_rx_rate_hz = current_rx_rate_hz
			_outlier_count = 0
	else:
		_valid_count = 0
		_outlier_count = 0
	return false


func _update_ff_filters() -> void:
	ff_speed_k = pt1_gain_from_delay(ff_smooth_tau, 1.0 / smoothed_rx_rate_hz)


func _update_smoothing_cutoffs() -> void:
	var sp: float = float(cfg.rc_smoothing_setpoint_cutoff)
	if sp == 0.0:
		sp = maxf(15.0, smoothed_rx_rate_hz * auto_sp)
	var th: float = float(cfg.rc_smoothing_throttle_cutoff)
	if th == 0.0:
		th = maxf(15.0, smoothed_rx_rate_hz * auto_thr)
	var ksp := pt3_gain(sp, dt)
	for a in 3:
		sp_k[a] = ksp
	ffs_k = ksp
	sp_k[3] = pt3_gain(th, dt)


func _calc_feedforward(axis: int) -> void:
	var rx_interval := current_rx_interval_us * 1e-6
	var rc_delta := rc_command[axis] - ff_prev_rc[axis]
	var rc_delta_abs := absf(rc_delta)
	var is_dup := rc_delta_abs == 0.0
	var sp := raw_setpoint[axis]
	var sp_delta := sp - ff_prev_sp[axis]
	ff_prev_rc[axis] = rc_command[axis]
	ff_prev_sp[axis] = sp
	var rx_rate := current_rx_rate_hz
	var sp_speed := 0.0
	if ff_interpolate:
		var prev_int := ff_prev_rx_interval[axis]
		if not is_dup:
			if ff_prev_dup[axis]:
				rx_rate = 1.0 / (rx_interval + prev_int)
			sp_speed = sp_delta * rx_rate
		else:
			if not ff_prev_dup[axis]:
				if absf(sp) < 0.9 * max_rc_rate[axis]:
					sp_speed = ff_prev_sp_speed[axis] + ff_prev_sp_speed_delta[axis]
					rc_delta_abs = ff_prev_rc_delta_abs[axis]
			else:
				sp_speed = 0.0
				ff_prev_sp_speed[axis] = 0.0
		ff_prev_rx_interval[axis] = rx_interval
	else:
		sp_speed = sp_delta * current_rx_rate_hz
	ff_prev_dup[axis] = is_dup
	var jitter := minf(((rc_delta_abs + ff_prev_rc_delta_abs[axis]) * 0.5 + 1.0) * ff_jitter_inv, 1.0)
	ff_prev_rc_delta_abs[axis] = rc_delta_abs
	ff_speed_s[axis] += ff_speed_k * (sp_speed - ff_speed_s[axis])
	sp_speed = ff_speed_s[axis]
	var sp_speed_delta := sp_speed - ff_prev_sp_speed[axis]
	ff_prev_sp_speed[axis] = sp_speed
	ff_delta_s[axis] += ff_speed_k * (sp_speed_delta - ff_delta_s[axis])
	sp_speed_delta = ff_delta_s[axis]
	ff_prev_sp_speed_delta[axis] = sp_speed_delta
	var boost := sp_speed_delta * rx_rate * ff_boost
	var ff := sp_speed
	if axis < 2:
		ff += boost
		ff *= jitter
		if ff_max_rate_limit != 0.0 and ff * sp > 0.0:
			var limit := (max_rc_rate[axis] - absf(sp)) * ff_max_rate_limit
			ff = clampf(ff, -limit, limit) if limit > 0.0 else 0.0
	else:
		ff *= jitter
		var k := pt1_gain_from_delay(ff_yaw_hold_time, rx_interval)
		ff_yaw_hold_s += k * (sp - ff_yaw_hold_s)
		ff += ff_yaw_hold_gain * (sp - ff_yaw_hold_s)
	if ff_transition != 0.0 and rc_deflection_abs[axis] < ff_transition:
		ff *= rc_deflection_abs[axis] * ff_transition_inv
	ff_raw[axis] = ff


func process_rc_command() -> void:
	if rx_data_new:
		if _should_update_smoothing():
			_update_ff_filters()
			_update_smoothing_cutoffs()
		for a in 3:
			var db: float = float(cfg.yaw_deadband) if a == 2 else float(cfg.deadband)
			var rcf := rc_command[a] / (500.0 - db)
			rc_deflection[a] = rcf
			var rabs := absf(rcf)
			rc_deflection_abs[a] = rabs
			var lim := float(cfg.rate_limits[a])
			raw_setpoint[a] = clampf(apply_rates(a, rcf, rabs), -lim, lim)
			_calc_feedforward(a)
		for i in 4:
			rx_to_smooth[i] = rc_command[3] if i == 3 else raw_setpoint[i]
	# PT3 smoothing of setpoints and throttle
	for i in 4:
		var k := sp_k[i]
		sp_s1[i] += k * (rx_to_smooth[i] - sp_s1[i])
		sp_s2[i] += k * (sp_s1[i] - sp_s2[i])
		sp_s[i] += k * (sp_s2[i] - sp_s[i])
	for i in 3:
		setpoint_rate[i] = sp_s[i]
	rc_throttle_smoothed = sp_s[3]
	for a in 3:
		ffs_s1[a] += ffs_k * (ff_raw[a] - ffs_s1[a])
		ffs_s2[a] += ffs_k * (ffs_s1[a] - ffs_s2[a])
		ffs_s[a] += ffs_k * (ffs_s2[a] - ffs_s[a])
		ff_smoothed[a] = ffs_s[a]
	rx_data_new = false


# ------------------------------------------------------------------ gyro
var _g := PackedFloat64Array([0, 0, 0])     # gyro_update's scratch
var rpm_qi: float = 0.0
func _rpm_notch_update(n: int, freq: float, weight: float) -> void:
	var x := PI * freq * dt
	var f := tan(x)
	var qi := 1.0 / rpm_q
	var a1 := 1.0 / (1.0 + f * (f + qi))
	rn_f[n] = f
	rn_a1[n] = a1
	rn_a2[n] = f * a1
	rn_wq[n] = qi * weight


func gyro_update(rates_dps: PackedFloat64Array, telemetry_hz: PackedFloat64Array) -> void:
	var g := _g
	rpm_qi = 1.0 / rpm_q
	for a in 3:
		if gl2_on:
			_lp_apply(gl2_s, a * 3, gl2_type, gl2_k, rates_dps[a])
			g[a] = _lp_apply(gl2_s, a * 3, gl2_type, gl2_k, rates_dps[a])
		else:
			g[a] = rates_dps[a]
	for m in 4:
		motor_hz[m] += motor_freq_k * (telemetry_hz[m] - motor_hz[m])
	if use_rpm_filter:
		for m in 4:
			for h in rpm_harm:
				var wt := rpm_weights[h]
				if wt <= 0.0:
					continue
				var f := clampf(float(h + 1) * motor_hz[m], rpm_min_hz, rpm_max_hz)
				var margin := f - rpm_min_hz
				var w := margin / rpm_fade if margin < rpm_fade else 1.0
				# _rpm_notch_update(), inlined (the hottest loop of the controller)
				var n := m * rpm_harm + h
				var tf := tan(PI * f * dt)
				var a1 := 1.0 / (1.0 + tf * (tf + rpm_qi))
				rn_f[n] = tf
				rn_a1[n] = a1
				rn_a2[n] = tf * a1
				rn_wq[n] = rpm_qi * w * wt
		for h in rpm_harm:
			for m in 4:
				var n := m * rpm_harm + h
				var a1 := rn_a1[n]
				var a2 := rn_a2[n]
				var f := rn_f[n]
				var wq := rn_wq[n]
				for i in 3:
					var j := n * 3 + i
					var v3 := g[i] - rn_ic2[j]
					var v1 := a1 * rn_ic1[j] + a2 * v3
					var v2 := rn_ic2[j] + f * v1
					rn_ic1[j] = 2.0 * v1 - rn_ic1[j]
					rn_ic2[j] = 2.0 * v2 - rn_ic2[j]
					g[i] -= wq * v1
	for a in 3:
		gyro_filtered[a] = _lp_apply(gl1_s, a * 3, gl1_type, gl1_k, g[a]) if gl1_on else g[a]


# ------------------------------------------------------------------ pid.c
func _tpa_classic(arg: float) -> float:
	var past := arg >= tpa_low_bp or tpa_low_bp <= 0.01
	var rate := 0.0
	if past or tpa_low_faded:
		rate = tpa_mult * maxf(arg - tpa_bp, 0.0)
		if tpa_low_always == 0 and not tpa_low_faded:
			tpa_low_faded = true
	else:
		rate = tpa_low_mult * (tpa_low_bp - arg)
	return 1.0 - rate


func pid_controller() -> void:
	ag_throttle_d *= ag_gain
	iterm_accel = ag_throttle_d * ANTIGRAVITY_KI
	var gdt := PackedFloat64Array([0, 0, 0])
	for a in 3:
		var v := gyro_filtered[a]
		if dl1_on:
			v = _lp_apply(dl1_s, a * 3, dl1_type, dl1_k, v)
		if dl2_on:
			v = _lp_apply(dl2_s, a * 3, dl2_type, dl2_k, v)
		gdt[a] = v
	for a in 3:
		var sp := setpoint_rate[a]
		var gyro := gyro_filtered[a]
		var err := sp - gyro
		var prev_i := I[a]
		var i_err := err
		windup_s[a] += windup_k * (sp - windup_s[a])
		var sp_lpf := windup_s[a]
		var sp_hpf := absf(sp - sp_lpf)
		if iterm_relax != 0 and (a < 2 or iterm_relax == 2 or iterm_relax == 4):
			var factor := maxf(0.0, 1.0 - sp_hpf / ITERM_RELAX_SETPOINT_THRESHOLD)
			var decreasing := (prev_i > 0.0 and i_err < 0.0) or (prev_i < 0.0 and i_err > 0.0)
			if iterm_relax >= 3 and decreasing:
				pass
			elif iterm_relax_type == 1:
				i_err *= factor
			elif iterm_relax_type == 0:
				var e2 := sp_lpf - gyro
				i_err = e2 - clampf(e2, -sp_hpf, sp_hpf) if absf(e2) > sp_hpf else 0.0
			else:
				i_err = 0.0
		var tpa_p := tpa_factor if tpa_mode == 0 else 1.0
		var p_term := kp[a] * err * tpa_p
		if a == 2 and yaw_p_lpf_on:
			yaw_p_lpf_s += yaw_p_lpf_k * (p_term - yaw_p_lpf_s)
			p_term = yaw_p_lpf_s
		var ilim := iterm_limit
		var acc := iterm_accel
		if a == 2:
			ilim = iterm_limit_yaw
			acc = 0.0
			iterm_accel = 0.0
		var i_term := clampf(prev_i + (ki[a] + acc) * dt * i_err, -ilim, ilim)
		var ff_in := ff_smoothed[a]
		var d_term := 0.0
		if kd[a] > 0.0:
			var delta := -(gdt[a] - prev_gyro_dterm[a]) * pid_freq
			var pre := kd[a] * delta
			var mult := 1.0
			if dmax_percent[a] > 1.0:
				dmax_range_s1[a] += dmax_range_k * (delta - dmax_range_s1[a])
				dmax_range_s[a] += dmax_range_k * (dmax_range_s1[a] - dmax_range_s[a])
				var gf := absf(dmax_range_s[a]) * dmax_gyro_gain
				var sf := absf(ff_in) * dmax_sp_gain
				mult += (dmax_percent[a] - 1.0) * maxf(gf, sf)
				dmax_lpf_s1[a] += dmax_lpf_k * (mult - dmax_lpf_s1[a])
				dmax_lpf_s[a] += dmax_lpf_k * (dmax_lpf_s1[a] - dmax_lpf_s[a])
				mult = minf(dmax_lpf_s[a], dmax_percent[a])
			d_term = pre * mult * tpa_factor
		prev_gyro_dterm[a] = gdt[a]
		var f_term := kf[a] * ff_in
		if a != 2:
			var att := maxf(absf(sp) / 50.0, 1.0)
			p_term *= 1.0 + (ag_throttle_d / att) * ag_p_gain
		P[a] = p_term
		I[a] = i_term
		D[a] = d_term
		F[a] = f_term
		pid_sum[a] = p_term + i_term + d_term + f_term
	if not stabilisation:
		for a in 3:
			P[a] = 0.0; I[a] = 0.0; D[a] = 0.0; F[a] = 0.0; pid_sum[a] = 0.0
	elif iterm_reset:
		for a in 3:
			I[a] = 0.0


func _dyn_lpf_update(throttle: float) -> void:
	if t_us - dyn_lpf_last_us < 5000:
		return
	var q := roundi(throttle * 100.0)
	if q == dyn_lpf_prev_q:
		return
	var thr := float(q) / 100.0
	var expof := dyn_lpf_expo / 10.0
	var curve := thr * (1.0 - thr) * expof + thr
	var cutoff := (dyn_lpf_max - dyn_lpf_min) * curve + dyn_lpf_min
	if dyn_lpf_min > 0.0:
		var ft := int(cfg.dterm_lpf1_type)
		if ft == 0:
			dl1_k = pt1_gain(cutoff, dt)
		elif ft == 1 or ft == 2:
			dl1_k = pt2_gain(cutoff, dt)
		else:
			dl1_k = pt3_gain(cutoff, dt)
	dyn_lpf_prev_q = q
	dyn_lpf_last_us = t_us


# ------------------------------------------------------------------ mixer.c
func mix_table() -> void:
	var throttle := (rc_throttle_smoothed - 1000.0) / 1000.0
	var inc := 0.0
	if dyn_idle_min_rps > 0.0:
		var max_inc := dyn_max_inc if throttle_raised else dyn_start_inc
		var min_rps := minf(minf(motor_hz[0], motor_hz[1]), minf(motor_hz[2], motor_hz[3]))
		var err := dyn_idle_min_rps - min_rps
		min_rps = prev_min_rps + min_rps_delay_k * (min_rps - prev_min_rps)
		var dd := (prev_min_rps - min_rps) * dyn_d
		prev_min_rps = min_rps
		var dp := err * dyn_p
		err = maxf(-0.1, err)
		dyn_i = clampf(dyn_i + err * dyn_i_gain, 0.0, max_inc)
		inc = clampf(dp + dyn_i + dd, 0.0, max_inc)
	var range_max := out_high
	var range_min := out_low + inc * (out_high - out_low)
	var out_min := range_min
	var out_range := range_max - range_min
	throttle = clampf(throttle, 0.0, 1.0)
	var roll := clampf(pid_sum[0], -pidsum_limit, pidsum_limit) / PID_MIXER_SCALING
	var pitch := clampf(pid_sum[1], -pidsum_limit, pidsum_limit) / PID_MIXER_SCALING
	var yaw := clampf(pid_sum[2], -pidsum_limit_yaw, pidsum_limit_yaw) / PID_MIXER_SCALING
	if not yaw_reversed:
		yaw = -yaw
	var thr_inv := 1.0 - throttle
	var td := absf(throttle - ag_prev_throttle) * pid_freq
	td *= thr_inv * thr_inv
	if throttle > ag_prev_throttle:
		td *= thr_inv * 0.5
	ag_prev_throttle = throttle
	ag_s1 += ag_k * (td - ag_s1)
	ag_s += ag_k * (ag_s1 - ag_s)
	ag_throttle_d = ag_s
	tpa_factor = _tpa_classic(clampf(throttle, 0.0, 1.0))
	_dyn_lpf_update(throttle)
	if throttle_boost > 0.0:
		throttle_lpf_s += throttle_lpf_k * (throttle - throttle_lpf_s)
		throttle = clampf(throttle + throttle_boost * (throttle - throttle_lpf_s), 0.0, 1.0)
	mixer_throttle = throttle
	if dyn_idle_min_rps > 0.0:
		throttle = maxf(throttle, 0.01)
	if thrust_lin != 0.0:
		throttle *= 1.0 - thrust_lin * (1.0 - throttle)
	var mix := PackedFloat64Array([0, 0, 0, 0])
	var mmax := 0.0
	var mmin := 0.0
	for i in 4:
		var v: float = roll * MIX_ROLL[i] + pitch * MIX_PITCH[i] + yaw * MIX_YAW[i]
		if v > mmax:
			mmax = v
		elif v < mmin:
			mmin = v
		mix[i] = v
	motor_mix_range = mmax - mmin
	var air_pct := 1.0
	if not airmode and throttle < 0.5:
		air_pct = 0.5 + throttle
	var norm := air_pct / motor_mix_range if motor_mix_range > 1.0 else air_pct
	for i in 4:
		mix[i] *= norm
	throttle = clampf(throttle, -mmin * norm, 1.0 - mmax * norm)
	for i in 4:
		var o := mix[i] + throttle
		if thrust_lin != 0.0:
			var inv := 1.0 - o
			o *= 1.0 + thrust_lin * inv * (1.0 + thrust_lin * (inv - o))
		o = out_min + out_range * o
		motor[i] = clampf(o, range_min, range_max)
	if not armed:
		motor.fill(0.0)


# ------------------------------------------------------------------ loop
func arm(p_armed: bool) -> void:
	if p_armed and not armed:
		I.fill(0.0)
	if not p_armed:
		throttle_raised = false
		airmode_active = false
	armed = p_armed


## One PID loop (4 kHz). Returns motor outputs in DShot units (0 = stopped).
func step(gyro_dps: PackedFloat64Array, telemetry_hz: PackedFloat64Array) -> PackedFloat64Array:
	gyro_update(gyro_dps, telemetry_hz)
	process_rc_command()
	pid_controller()
	mix_table()
	t_us += roundi(dt * 1e6)
	return motor
