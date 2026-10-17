extends RefCounted
## real_quad: the Betaflight 2026.6.2 rate loop (acro mode, quad X), a GDScript port.
##
## Ported function by function from the firmware sources at commit e0b7bb01b:
##   fc/rc.c (rates, RC smoothing, feedforward), flight/pid.c + pid_init.c (PID, I-term relax,
##   anti-gravity, TPA, D-max), flight/mixer.c + mixer_init.c (airmode mixer, dynamic idle,
##   throttle boost, battery sag compensation), drivers/dshot.c (endpoints), sensors/gyro +
##   flight/rpm_filter.c + flight/dyn_notch_filter.c + common/sdft.c (gyro filters at the gyro and the PID
##   rate, RPM notches updated round robin, static and dynamic notches, dynamic lowpass), common/filter.c
##   (PT1/PT2/PT3, SVF lowpass and notch).
## Settings are read from the craft's Betaflight configuration (blackbox header, JSON).
##
## Style note: the code deliberately stays close to the C source - same function boundaries, same
## variable names, filter states as flat packed arrays instead of filter objects - so every line
## can be checked against the firmware, and to keep the 4 kHz loop cheap in GDScript. Where the loop
## is hottest (the RPM notches, the PT3 smoothing of the RC input, the mixer) the loops are fused or
## written out and the states are single variables, which gives the same numbers bit for bit. A Python
## reference implementation of the same port, validated open-loop against logged P/I/D/F terms,
## is compared with this file sample by sample (not part of the mod package).
##
## Gyro samples: step() takes the gyro_per_pid samples of one PID loop (oldest first, 3 values each; 3 values
## alone are held for all of them). Optional settings missing in an older fc.json default to off.
##
## Units as in the firmware: deg/s, rcCommand -500..500, rcData 1000..2000, DShot 48..2047.
## Axes: Betaflight gyro axes (roll+ right side down, pitch+ nose down, yaw+ nose left).
## Betaflight is GPL-3.0; this port is a derivative work and GPL-3.0 as well.
## (No class_name: the scripts are always loaded by preload(), so the same files can live under
## different res:// paths - mod pack, test project - without clashing global class names.)

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
const SVF_INV_Q := 1.41421356237    # the SVF lowpass's 1/Q (Butterworth)
const LP_SVF := 4                   # lowpass kind: 1..3 = PT1..PT3 (stages), LP_SVF = state variable filter
const RPM_FILTER_DURATION_S := 0.001
const DYN_LPF_UPDATE_US := 5000
const SDFT_SAMPLE_SIZE := 72
const SDFT_BIN_COUNT := 36
const SDFT_R := 0.9999
const DYN_NOTCH_SMOOTH_HZ := 4.0
const DYN_NOTCH_CALC_TICKS := 12    # 3 axes x 4 steps
const DYN_NOTCH_UPDATE_MIN_HZ := 2000.0
const CELL_VOLTAGE_FULL_CV := 420.0
const SAG_TASK_US := 5000           # the fast voltage task (200 Hz)

var cfg: Dictionary
var gyro_dt: float
var gyro_per_pid: int = 1
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
# cfg's rates as numbers (update_rates()), and the deadbands and rate limits of the axes
var _rates_type: int = 0
var _rc_rates := PackedFloat64Array([0, 0, 0])
var _rates := PackedFloat64Array([0, 0, 0])
var _rc_expo := PackedFloat64Array([0, 0, 0])
var _deadband := PackedFloat64Array([0, 0, 0])
var _rate_limit := PackedFloat64Array([0, 0, 0])
var _mincheck: float = 1050.0
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
# PT3 states: setpoint of roll, pitch, yaw (sp_r, sp_p, sp_y), throttle (sp_t) and the feedforward of the three axes
# (ffs_r, ffs_p, ffs_y), three stages each; single variables because process_rc_command() runs at every PID step
var sp_k: float = 1.0           # the gain of the axes' setpoints and feedforward
var sp_k_thr: float = 1.0       # ... of the throttle
var sp_r1: float = 0.0
var sp_r2: float = 0.0
var sp_r3: float = 0.0
var sp_p1: float = 0.0
var sp_p2: float = 0.0
var sp_p3: float = 0.0
var sp_y1: float = 0.0
var sp_y2: float = 0.0
var sp_y3: float = 0.0
var sp_t1: float = 0.0
var sp_t2: float = 0.0
var sp_t3: float = 0.0
var ffs_r1: float = 0.0
var ffs_r2: float = 0.0
var ffs_r3: float = 0.0
var ffs_p1: float = 0.0
var ffs_p2: float = 0.0
var ffs_p3: float = 0.0
var ffs_y1: float = 0.0
var ffs_y2: float = 0.0
var ffs_y3: float = 0.0
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
var _dl1_raw_type: int      # cfg's dterm_lpf1_type
var dl1_k: float = 1.0
var dl1_on := false
var dl1_s := PackedFloat64Array([0, 0, 0, 0, 0, 0, 0, 0, 0])   # 3 axes x 3 stages (SVF: ic1, ic2)
var dl1_c := PackedFloat64Array([1, 0, 0])                      # SVF coefficients a1, a2, f
var dl2_type: int
var dl2_k: float = 1.0
var dl2_on := false
var dl2_s := PackedFloat64Array([0, 0, 0, 0, 0, 0, 0, 0, 0])
var dl2_c := PackedFloat64Array([1, 0, 0])
var dn_on := false                  # dterm notch (SVF notch: coefficients a1, a2q, fq; states ic1q, ic2 per axis)
var dn_c := PackedFloat64Array([1, 0, 0])
var dn_s := PackedFloat64Array([0, 0, 0, 0, 0, 0])
var iterm_rotation := false
var yaw_p_lpf_on := false
var yaw_p_lpf_k: float = 1.0
var yaw_p_lpf_s: float = 0.0
var prev_gyro_dterm := PackedFloat64Array([0, 0, 0])
var dyn_lpf_min: float
var dyn_lpf_max: float
var dyn_lpf_expo: float
var gdyn_lpf_min: float = 0.0       # the gyro's dynamic lowpass 1
var gdyn_lpf_max: float = 0.0
var gdyn_lpf_expo: float = 0.0
var _gl1_raw_type: int = 0
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
var pid_inject := PackedFloat64Array([0, 0, 0])   # added to the PID sum (the optimizer's margin chirp; not Betaflight)
var stabilisation := true
var iterm_reset := true

# ---- gyro
var gl1_type: int
var gl1_on := false
var gl1_k: float = 1.0
var gl1_s := PackedFloat64Array([0, 0, 0, 0, 0, 0, 0, 0, 0])
var gl1_c := PackedFloat64Array([1, 0, 0])
var gl2_type: int
var gl2_on := false
var gl2_k: float = 1.0
var gl2_s := PackedFloat64Array([0, 0, 0, 0, 0, 0, 0, 0, 0])
var gl2_c := PackedFloat64Array([1, 0, 0])
var gn1_on := false                 # static gyro notches 1 and 2 (SVF notch, as dn_)
var gn1_c := PackedFloat64Array([1, 0, 0])
var gn1_s := PackedFloat64Array([0, 0, 0, 0, 0, 0])
var gn2_on := false
var gn2_c := PackedFloat64Array([1, 0, 0])
var gn2_s := PackedFloat64Array([0, 0, 0, 0, 0, 0])
var gyro_filtered := PackedFloat64Array([0, 0, 0])
var use_rpm_filter := true
var rpm_harm: int
var rpm_min_hz: float
var rpm_max_hz: float
var rpm_fade: float
var rpm_q: float
var rpm_weights := PackedFloat64Array()
# rpm notches: the active ones (harmonic weight > 0) in the order they run, harmonic outer and motor inner
# (n = motor*harm + h in the firmware); the coefficients are worked out in the loop, the states are
# the biquad's two integrators per axis (roll, pitch, yaw), one array each, indexed by the notch
var rn_count: int = 0
var rn_of := PackedInt32Array()       # motor * harmonics + harmonic -> its notch (-1: weight 0, not run)
var rn_a1 := PackedFloat64Array()     # the notches' coefficients (rpmNotchUpdate)
var rn_a2 := PackedFloat64Array()
var rn_f := PackedFloat64Array()
var rn_wq := PackedFloat64Array()
var rn_updates: int = 1               # notch updates per PID loop (all of them within RPM_FILTER_DURATION_S)
var rn_motor_i: int = 0
var rn_harm_i: int = 0
var rn_ic1_r := PackedFloat64Array()
var rn_ic1_p := PackedFloat64Array()
var rn_ic1_y := PackedFloat64Array()
var rn_ic2_r := PackedFloat64Array()
var rn_ic2_p := PackedFloat64Array()
var rn_ic2_y := PackedFloat64Array()
var motor_freq_k: float
var motor_hz := PackedFloat64Array([0, 0, 0, 0])
# dynamic notch (flight/dyn_notch_filter.c, common/sdft.c): per axis dn_count notches, [axis * dn_count + p]
var dyn_count: int = 0
var dyn_q: float = 1.0
var dyn_min_hz: float = 0.0
var dyn_max_hz: float = 0.0
var dyn_center := PackedFloat64Array()
var dyn_a1 := PackedFloat64Array()
var dyn_a2q := PackedFloat64Array()
var dyn_fq := PackedFloat64Array()
var dyn_ic1q := PackedFloat64Array()
var dyn_ic2 := PackedFloat64Array()
var dyn_sample_index: int = 0
var dyn_sample_count: int = 1
var dyn_sample_rcp: float = 1.0
var dyn_acc := PackedFloat64Array([0, 0, 0])
var dyn_avg := PackedFloat64Array([0, 0, 0])
var dyn_tick: int = 0
var dyn_step: int = 0
var dyn_axis: int = 0
var dyn_pt1_dt: float = 0.0
var dyn_threshold: float = 0.0
var sdft_samples := PackedFloat64Array()   # 3 x SDFT_SAMPLE_SIZE
var sdft_idx := PackedInt32Array([0, 0, 0])
var sdft_re := PackedFloat64Array()        # 3 x SDFT_BIN_COUNT
var sdft_im := PackedFloat64Array()
var sdft_tw_re := PackedFloat64Array()
var sdft_tw_im := PackedFloat64Array()
var sdft_rpn: float = 0.0
var sdft_start: int = 1
var sdft_end: int = 1
var sdft_batch: int = 1
var sdft_batches: int = 1
var sdft_res_hz: float = 1.0
var sdft_data := PackedFloat64Array()
var pk_bin := PackedInt32Array()
var pk_val := PackedFloat64Array()

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
var sag_factor: float = 0.0          # vbat_sag_compensation (0 = off)
var sag_full: float = CELL_VOLTAGE_FULL_CV
var sag_range: float = 0.0
var sag_k: float = 1.0
var sag_s: float = 0.0               # the sag-filtered pack voltage [0.01 V], -1 = not seen yet
var sag_last_us: int = -1000000000
var vbat_cv: float = 0.0             # pack voltage [0.01 V] (set_vbat())
var vbat_cells: int = 0
var airmode: bool
var yaw_reversed: bool
var pidsum_limit: float
var pidsum_limit_yaw: float
var motor := PackedFloat64Array([0, 0, 0, 0])
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


## Lowpass setup: returns [enabled, kind, gain, SVF coefficients]. cfg's types FILTER_PT1=0, SVF=1, PT2=2, PT3=3;
## kind 1..3 = PT1..PT3 (stages), LP_SVF. An SVF runs only up to svf_max_hz (the firmware's nyquist checks).
static func _lp_setup(ftype: int, hz: float, p_dt: float, svf_max_hz: float = INF) -> Array:
	if hz <= 0.0 or ftype < 0 or ftype > 3:
		return [false, 1, 1.0, PackedFloat64Array([1, 0, 0])]
	if ftype == 0:
		return [true, 1, pt1_gain(hz, p_dt), PackedFloat64Array([1, 0, 0])]
	if ftype == 2:
		return [true, 2, pt2_gain(hz, p_dt), PackedFloat64Array([1, 0, 0])]
	if ftype == 3:
		return [true, 3, pt3_gain(hz, p_dt), PackedFloat64Array([1, 0, 0])]
	if hz > svf_max_hz:
		return [false, 1, 1.0, PackedFloat64Array([1, 0, 0])]
	var c := PackedFloat64Array([1, 0, 0])
	svf_lowpass_coeffs(c, hz, p_dt)
	return [true, LP_SVF, 1.0, c]


## svfLowpassFilterUpdate: c = [a1, a2, f]
static func svf_lowpass_coeffs(c: PackedFloat64Array, hz: float, p_dt: float) -> void:
	var f := tan(PI * hz * p_dt)
	var inv := 1.0 / (1.0 + f * (f + SVF_INV_Q))
	c[0] = inv
	c[1] = f * inv
	c[2] = f


## svfNotchUpdate: c = [a1, a2q, fq]
static func svf_notch_coeffs(c: PackedFloat64Array, hz: float, p_dt: float, q: float) -> void:
	var f := tan(PI * hz * p_dt)
	var qi := 1.0 / q
	var inv := 1.0 / (1.0 + f * (f + qi))
	c[0] = inv
	c[1] = f * inv * qi
	c[2] = f * q


## filterGetNotchQ
static func notch_q(center: float, cutoff: float) -> float:
	return center * cutoff / (center * center - cutoff * cutoff)


## A static notch (gyro: calculateNyquistAdjustedNotchHz, dterm: pidInitFilters' check): [enabled, coefficients]
static func _notch_setup(hz: float, cutoff: float, nyquist: float, p_dt: float) -> Array:
	if hz > nyquist:
		hz = nyquist if cutoff < nyquist else 0.0
	var c := PackedFloat64Array([1, 0, 0])
	if hz == 0.0 or cutoff == 0.0:
		return [false, c]
	svf_notch_coeffs(c, hz, p_dt, notch_q(hz, cutoff))
	return [true, c]


## The SVF notch on the three axes, in place (states ic1q, ic2 of axis a at s[2a], s[2a + 1])
static func _notch3(s: PackedFloat64Array, c: PackedFloat64Array, x: PackedFloat64Array) -> void:
	var a1 := c[0]
	var a2q := c[1]
	var fq := c[2]
	for a in 3:
		var ic1q := s[2 * a]
		var ic2 := s[2 * a + 1]
		var v := x[a]
		var v1q := a1 * ic1q + a2q * (v - ic2)
		var v2 := ic2 + fq * v1q
		s[2 * a] = 2.0 * v1q - ic1q
		s[2 * a + 1] = 2.0 * v2 - ic2
		x[a] = v - v1q


## Apply a lowpass of the kinds above to the three axes, in place
static func _lpf(s: PackedFloat64Array, kind: int, k: float, c: PackedFloat64Array, x: PackedFloat64Array) -> void:
	if kind != LP_SVF:
		_lp3(s, kind, k, x)
		return
	var a1 := c[0]
	var a2 := c[1]
	var f := c[2]
	for a in 3:
		var b := a * 3
		var ic1 := s[b]
		var ic2 := s[b + 1]
		var v1 := a1 * ic1 + a2 * (x[a] - ic2)
		var v2 := ic2 + f * v1
		s[b] = 2.0 * v1 - ic1
		s[b + 1] = 2.0 * v2 - ic2
		x[a] = v2


## Apply a 1-3 stage PT filter to the three axes, in place: x[a] is filtered by the states s[a*3 .. a*3+2];
## s[a*3] is always the output.
static func _lp3(s: PackedFloat64Array, stages: int, k: float, x: PackedFloat64Array) -> void:
	for a in 3:
		var b := a * 3
		var s0 := s[b]
		if stages == 1:
			s0 += k * (x[a] - s0)
		else:
			var s1 := s[b + 1]
			s1 += k * (x[a] - s1)
			if stages == 2:
				s0 += k * (s1 - s0)
			else:
				var s2 := s[b + 2]
				s2 += k * (s1 - s2)
				s0 += k * (s2 - s0)
				s[b + 2] = s2
			s[b + 1] = s1
		s[b] = s0
		x[a] = s0


func _init(p_cfg: Dictionary, p_use_rpm_filter: bool = true) -> void:
	cfg = p_cfg
	gyro_dt = float(cfg.looptime_us) * 1e-6
	gyro_per_pid = maxi(1, int(cfg.pid_process_denom))
	dt = gyro_dt * float(gyro_per_pid)
	pid_freq = 1.0 / dt
	for a in 3:
		_deadband[a] = float(cfg.yaw_deadband) if a == 2 else float(cfg.deadband)
		_rate_limit[a] = float(cfg.rate_limits[a])
	_mincheck = float(cfg.mincheck)
	update_rates()
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
	_dl1_raw_type = int(cfg.dterm_lpf1_type)
	var pid_nyq := float(int(pid_freq / 2.0 * 0.95))     # (uint32: pidFrequencyNyquist)
	var l1hz := dyn_lpf_min if dyn_lpf_min > 0.0 else float(cfg.dterm_lpf1_static_hz)
	var s1 := _lp_setup(int(cfg.dterm_lpf1_type), l1hz, dt, pid_nyq - 1.0)
	dl1_on = s1[0]; dl1_type = s1[1]; dl1_k = s1[2]; dl1_c = s1[3]
	var s2 := _lp_setup(int(cfg.dterm_lpf2_type), float(cfg.dterm_lpf2_static_hz), dt, pid_nyq - 1.0)
	dl2_on = s2[0]; dl2_type = s2[1]; dl2_k = s2[2]; dl2_c = s2[3]
	var dhz := float(cfg.get("dterm_notch_hz", 0))
	var dcut := float(cfg.get("dterm_notch_cutoff", 0))
	if dhz > pid_nyq:
		dhz = pid_nyq if dcut < pid_nyq else 0.0
	var dns := _notch_setup(dhz, dcut, INF, dt)
	dn_on = dns[0]; dn_c = dns[1]
	iterm_rotation = bool(cfg.get("iterm_rotation", 0))
	var sy := _lp_setup(0, float(cfg.yaw_lowpass_hz), dt)
	yaw_p_lpf_on = sy[0]; yaw_p_lpf_k = sy[2]
	tpa_bp = clampf((float(cfg.tpa_breakpoint) - 1000.0) / 1000.0, 0.0, 0.99)
	tpa_mult = (float(cfg.tpa_rate) / 100.0) / (1.0 - tpa_bp)
	tpa_low_bp = minf(clampf((float(cfg.tpa_low_breakpoint) - 1000.0) / 1000.0, 0.01, 1.0), tpa_bp)
	tpa_low_mult = float(cfg.tpa_low_rate) / (100.0 * tpa_low_bp)
	tpa_low_always = int(cfg.tpa_low_always)
	tpa_mode = int(cfg.tpa_mode)
	# gyro (sensors/gyro_init.c: lowpass 1, the notches and the RPM filter at the PID rate, lowpass 2 at the gyro rate)
	var target_us := roundi(dt * 1e6)
	var sample_us := int(cfg.looptime_us)
	var gyro_nyq := float(int(float(1000000 / 2 / target_us) * 0.95))
	var dyn_g: Array = cfg.get("gyro_lpf1_dyn_hz", [0, 0])
	gdyn_lpf_min = float(dyn_g[0])
	gdyn_lpf_max = float(dyn_g[1])
	gdyn_lpf_expo = float(cfg.get("gyro_lpf1_dyn_expo", 5))
	_gl1_raw_type = int(cfg.gyro_lpf1_type)
	var g1hz := gdyn_lpf_min if gdyn_lpf_min > 0.0 else float(cfg.gyro_lpf1_static_hz)
	var g1 := _lp_setup(int(cfg.gyro_lpf1_type), g1hz, dt, gyro_nyq)
	gl1_on = g1[0]; gl1_type = g1[1]; gl1_k = g1[2]; gl1_c = g1[3]
	var g2 := _lp_setup(int(cfg.gyro_lpf2_type), float(cfg.gyro_lpf2_static_hz), gyro_dt,
			float(int(float(1000000 / 2 / sample_us) * 0.95)))
	gl2_on = g2[0]; gl2_type = g2[1]; gl2_k = g2[2]; gl2_c = g2[3]
	var nhz: Array = cfg.get("gyro_notch_hz", [0, 0])
	var ncut: Array = cfg.get("gyro_notch_cutoff", [0, 0])
	var n1 := _notch_setup(float(nhz[0]), float(ncut[0]), gyro_nyq, dt)
	gn1_on = n1[0]; gn1_c = n1[1]
	var n2 := _notch_setup(float(nhz[1]), float(ncut[1]), gyro_nyq, dt)
	gn2_on = n2[0]; gn2_c = n2[1]
	_dyn_notch_init()
	rpm_harm = int(cfg.rpm_filter_harmonics)
	use_rpm_filter = p_use_rpm_filter and rpm_harm > 0
	rpm_min_hz = float(cfg.rpm_filter_min_hz)
	rpm_max_hz = 0.48 / dt
	rpm_fade = float(cfg.rpm_filter_fade_range_hz)
	rpm_q = float(cfg.rpm_filter_q) / 100.0
	for w in cfg.rpm_filter_weights:
		rpm_weights.append(clampf(float(w) / 100.0, 0.0, 1.0))
	rpm_qi = 1.0 / rpm_q
	rn_of.resize(4 * rpm_harm)
	rn_of.fill(-1)
	for h in rpm_harm:          # (the order of the firmware's loop: harmonic outer, motor inner)
		if rpm_weights[h] <= 0.0:
			continue            # (a notch of weight 0 changes nothing)
		for m in 4:
			rn_of[m * rpm_harm + h] = rn_count
			rn_count += 1
			var tf := tan(PI * rpm_min_hz * float(h + 1) * dt)     # rpmNotchInit: at the minimum, weight 0
			var a1 := 1.0 / (1.0 + tf * (tf + rpm_qi))
			rn_a1.append(a1)
			rn_a2.append(tf * a1)
			rn_f.append(tf)
			rn_wq.append(0.0)
	rn_updates = maxi(1, ceili(float(4 * rpm_harm) / (RPM_FILTER_DURATION_S / dt)))
	for arr in [rn_ic1_r, rn_ic1_p, rn_ic1_y, rn_ic2_r, rn_ic2_p, rn_ic2_y]:
		arr.resize(rn_count)
		arr.fill(0.0)
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
	sag_range = CELL_VOLTAGE_FULL_CV - float(cfg.get("vbat_warning_cell_cv", 350))
	if int(cfg.get("vbat_sag_compensation", 0)) > 0 and sag_range > 0.0:
		sag_factor = float(cfg.vbat_sag_compensation) / 100.0
	sag_k = pt1_gain(1.0 / (float(cfg.get("vbat_sag_lpf_period", 2)) / 10.0), float(SAG_TASK_US) * 1e-6)
	sag_s = -1.0
	airmode = bool(cfg.airmode)
	yaw_reversed = bool(cfg.yaw_motors_reversed)
	pidsum_limit = float(cfg.pidsum_limit)
	pidsum_limit_yaw = float(cfg.pidsum_limit_yaw)
	motor.fill(0.0)


# ------------------------------------------------------------------ rc.c
## The rates of cfg (rates_type, rc_rates, rates, rc_expo) changed: the stick's full rate is worked out again
func update_rates() -> void:
	_rates_type = int(cfg.rates_type)
	for a in 3:
		_rc_rates[a] = float(cfg.rc_rates[a])
		_rates[a] = float(cfg.rates[a])
		_rc_expo[a] = float(cfg.rc_expo[a])
	for a in 3:
		max_rc_rate[a] = apply_rates(a, 1.0, 1.0)


func apply_rates(axis: int, rc: float, rc_abs: float) -> float:
	if _rates_type == 1:   # KISS (as the game's: rc_rates/100 = rate, rates/100 = rate factor, rc_expo/100 = curve)
		var ke := _rc_expo[axis] / 100.0
		var kc := (pow(rc, 3) * ke + rc * (1.0 - ke)) * (_rc_rates[axis] / 100.0)
		return 200.0 * kc / clampf(1.0 - rc_abs * _rates[axis] / 100.0, 0.01, 1.0)
	if _rates_type == 3:   # ACTUAL
		var expof := _rc_expo[axis] / 100.0
		expof = rc_abs * (pow(rc, 5) * expof + rc * (1.0 - expof))
		var center := _rc_rates[axis] * 10.0
		var stick_mov := maxf(0.0, _rates[axis] * 10.0 - center)
		return rc * center + stick_mov * expof
	# BETAFLIGHT rates
	if int(_rc_expo[axis]) != 0:
		var e := _rc_expo[axis] / 100.0
		rc = rc * pow(rc_abs, 3) * e + rc * (1.0 - e)
	var rr := _rc_rates[axis] / 100.0
	if rr > 2.0:
		rr += 14.54 * (rr - 2.0)
	var ang := 200.0 * rr * rc
	if int(_rates[axis]) != 0:
		ang *= 1.0 / clampf(1.0 - rc_abs * _rates[axis] / 100.0, 0.01, 1.0)
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


## New RC frame. rc_data = [roll, pitch, yaw, throttle] in 1000..2000 (AETR order), the sticks with the
## deadband already taken off (Betaflight's rcCommand + 1500: a log has rcCommand, the game's sticks go through
## bf_model.rc_to_rcdata()): process_rc_command() divides by 500 - deadband.
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
	var mc := _mincheck
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
	sp_k = pt3_gain(sp, dt)
	sp_k_thr = pt3_gain(th, dt)


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
			var rcf := rc_command[a] / (500.0 - _deadband[a])
			rc_deflection[a] = rcf
			var rabs := absf(rcf)
			rc_deflection_abs[a] = rabs
			var lim := _rate_limit[a]
			raw_setpoint[a] = clampf(apply_rates(a, rcf, rabs), -lim, lim)
			_calc_feedforward(a)
		for i in 4:
			rx_to_smooth[i] = rc_command[3] if i == 3 else raw_setpoint[i]
	# PT3 smoothing of the setpoints, their feedforward and the throttle
	var k := sp_k
	sp_r1 += k * (rx_to_smooth[0] - sp_r1)
	sp_r2 += k * (sp_r1 - sp_r2)
	sp_r3 += k * (sp_r2 - sp_r3)
	setpoint_rate[0] = sp_r3
	sp_p1 += k * (rx_to_smooth[1] - sp_p1)
	sp_p2 += k * (sp_p1 - sp_p2)
	sp_p3 += k * (sp_p2 - sp_p3)
	setpoint_rate[1] = sp_p3
	sp_y1 += k * (rx_to_smooth[2] - sp_y1)
	sp_y2 += k * (sp_y1 - sp_y2)
	sp_y3 += k * (sp_y2 - sp_y3)
	setpoint_rate[2] = sp_y3
	k = sp_k_thr
	sp_t1 += k * (rx_to_smooth[3] - sp_t1)
	sp_t2 += k * (sp_t1 - sp_t2)
	sp_t3 += k * (sp_t2 - sp_t3)
	rc_throttle_smoothed = sp_t3
	k = sp_k
	ffs_r1 += k * (ff_raw[0] - ffs_r1)
	ffs_r2 += k * (ffs_r1 - ffs_r2)
	ffs_r3 += k * (ffs_r2 - ffs_r3)
	ff_smoothed[0] = ffs_r3
	ffs_p1 += k * (ff_raw[1] - ffs_p1)
	ffs_p2 += k * (ffs_p1 - ffs_p2)
	ffs_p3 += k * (ffs_p2 - ffs_p3)
	ff_smoothed[1] = ffs_p3
	ffs_y1 += k * (ff_raw[2] - ffs_y1)
	ffs_y2 += k * (ffs_y1 - ffs_y2)
	ffs_y3 += k * (ffs_y2 - ffs_y3)
	ff_smoothed[2] = ffs_y3
	rx_data_new = false


# ------------------------------------------------------------------ gyro
var _g := PackedFloat64Array([0, 0, 0])     # gyro_update's scratch
var rpm_qi: float = 0.0
## sensors/gyro.c gyroUpdate (the samples at the gyro rate: lowpass 2 as the downsampling filter, else their
## mean) and gyro_filter_impl.c filterGyro (RPM notches, static notches, lowpass 1, dynamic notch).
## samples: the gyro_per_pid samples of this PID loop, oldest first (3 values: held for all of them).
func gyro_update(samples: PackedFloat64Array, telemetry_hz: PackedFloat64Array) -> void:
	var g := _g
	var held := samples.size() < 3 * gyro_per_pid
	var n := gyro_per_pid
	if gl2_on:
		var k := gl2_k
		var s := gl2_s
		var kind := gl2_type
		for a in 3:
			var b := a * 3
			var s0 := s[b]
			var s1 := s[b + 1]
			var s2 := s[b + 2]
			var out := 0.0
			for j in n:
				var x := samples[a] if held else samples[3 * j + a]
				if kind == 1:
					s0 += k * (x - s0)
					out = s0
				elif kind == 2:
					s1 += k * (x - s1)
					s0 += k * (s1 - s0)
					out = s0
				elif kind == 3:
					s1 += k * (x - s1)
					s2 += k * (s1 - s2)
					s0 += k * (s2 - s0)
					out = s0
				else:       # SVF: s0 = ic1, s1 = ic2
					var v1 := gl2_c[0] * s0 + gl2_c[1] * (x - s1)
					var v2 := s1 + gl2_c[2] * v1
					s0 = 2.0 * v1 - s0
					s1 = 2.0 * v2 - s1
					out = v2
			s[b] = s0
			s[b + 1] = s1
			s[b + 2] = s2
			g[a] = out
	elif held:
		g[0] = samples[0]
		g[1] = samples[1]
		g[2] = samples[2]
	else:
		for a in 3:
			var sum := 0.0
			for j in n:
				sum += samples[3 * j + a]
			g[a] = sum / float(n)
	for m in 4:
		motor_hz[m] += motor_freq_k * (telemetry_hz[m] - motor_hz[m])
	if use_rpm_filter:
		_rpm_filter_update()
		var g0 := g[0]
		var g1 := g[1]
		var g2 := g[2]
		for i in rn_count:
			var a1 := rn_a1[i]
			var a2 := rn_a2[i]
			var tf := rn_f[i]
			var wq := rn_wq[i]
			var c1 := rn_ic1_r[i]
			var c2 := rn_ic2_r[i]
			var v3 := g0 - c2
			var v1 := a1 * c1 + a2 * v3
			var v2 := c2 + tf * v1
			rn_ic1_r[i] = 2.0 * v1 - c1
			rn_ic2_r[i] = 2.0 * v2 - c2
			g0 -= wq * v1
			c1 = rn_ic1_p[i]
			c2 = rn_ic2_p[i]
			v3 = g1 - c2
			v1 = a1 * c1 + a2 * v3
			v2 = c2 + tf * v1
			rn_ic1_p[i] = 2.0 * v1 - c1
			rn_ic2_p[i] = 2.0 * v2 - c2
			g1 -= wq * v1
			c1 = rn_ic1_y[i]
			c2 = rn_ic2_y[i]
			v3 = g2 - c2
			v1 = a1 * c1 + a2 * v3
			v2 = c2 + tf * v1
			rn_ic1_y[i] = 2.0 * v1 - c1
			rn_ic2_y[i] = 2.0 * v2 - c2
			g2 -= wq * v1
		g[0] = g0
		g[1] = g1
		g[2] = g2
	if gn1_on:
		_notch3(gn1_s, gn1_c, g)
	if gn2_on:
		_notch3(gn2_s, gn2_c, g)
	if gl1_on:
		_lpf(gl1_s, gl1_type, gl1_k, gl1_c, g)
	if dyn_count > 0:
		for a in 3:
			dyn_acc[a] += g[a]           # dynNotchPush
			var x := g[a]
			for p in dyn_count:          # dynNotchFilter
				var i := a * dyn_count + p
				var ic1q := dyn_ic1q[i]
				var ic2 := dyn_ic2[i]
				var v1q := dyn_a1[i] * ic1q + dyn_a2q[i] * (x - ic2)
				var v2 := ic2 + dyn_fq[i] * v1q
				dyn_ic1q[i] = 2.0 * v1q - ic1q
				dyn_ic2[i] = 2.0 * v2 - ic2
				x -= v1q
			g[a] = x
	gyro_filtered[0] = g[0]
	gyro_filtered[1] = g[1]
	gyro_filtered[2] = g[2]
	if dyn_count > 0:
		_dyn_notch_update()


## rpm_filter.c rpmFilterUpdate: rn_updates notches a loop, round robin (harmonic inner, motor outer)
func _rpm_filter_update() -> void:
	for u in rn_updates:
		var i := rn_of[rn_motor_i * rpm_harm + rn_harm_i]
		if i >= 0:
			var f := clampf(float(rn_harm_i + 1) * motor_hz[rn_motor_i], rpm_min_hz, rpm_max_hz)
			var margin := f - rpm_min_hz
			var w := margin / rpm_fade if margin < rpm_fade else 1.0
			var tf := tan(PI * f * dt)
			var a1 := 1.0 / (1.0 + tf * (tf + rpm_qi))
			rn_a1[i] = a1
			rn_a2[i] = tf * a1
			rn_f[i] = tf
			rn_wq[i] = rpm_qi * w * rpm_weights[rn_harm_i]
		rn_harm_i = (rn_harm_i + 1) % rpm_harm
		if rn_harm_i == 0:
			rn_motor_i = (rn_motor_i + 1) % 4


## dyn_notch_filter.c dynNotchInit and sdft.c sdftInit
func _dyn_notch_init() -> void:
	dyn_count = 0
	var count := int(cfg.get("dyn_notch_count", 0))
	var looprate := 1.0 / dt
	var nyq := looprate / 2.0
	if count <= 0 or looprate < DYN_NOTCH_UPDATE_MIN_HZ:
		return
	dyn_count = mini(count, 7)
	dyn_q = float(cfg.get("dyn_notch_q", 300)) / 100.0
	dyn_min_hz = float(cfg.get("dyn_notch_min_hz", 100))
	dyn_max_hz = minf(maxf(dyn_min_hz, float(cfg.get("dyn_notch_max_hz", 600))), nyq)
	dyn_sample_count = maxi(1, int(nyq / dyn_max_hz))
	dyn_sample_rcp = 1.0 / float(dyn_sample_count)
	sdft_res_hz = looprate / float(dyn_sample_count) / float(SDFT_SAMPLE_SIZE)
	var start := maxi(1, roundi(dyn_min_hz / sdft_res_hz))
	var end := mini(SDFT_BIN_COUNT - 1, roundi(dyn_max_hz / sdft_res_hz))
	dyn_pt1_dt = float(DYN_NOTCH_CALC_TICKS) / looprate
	sdft_rpn = pow(SDFT_R, SDFT_SAMPLE_SIZE)
	sdft_tw_re.resize(SDFT_BIN_COUNT)
	sdft_tw_im.resize(SDFT_BIN_COUNT)
	for i in SDFT_BIN_COUNT:
		var phi := 2.0 * PI / float(SDFT_SAMPLE_SIZE) * float(i)
		sdft_tw_re[i] = SDFT_R * cos(phi)
		sdft_tw_im[i] = SDFT_R * sin(phi)
	sdft_start = clampi(start, 0, SDFT_BIN_COUNT - 1)
	sdft_end = clampi(end, sdft_start, SDFT_BIN_COUNT - 1)
	sdft_batches = maxi(dyn_sample_count, 1)
	sdft_batch = (sdft_end - sdft_start + 1) / sdft_batches
	sdft_samples.resize(3 * SDFT_SAMPLE_SIZE)
	sdft_samples.fill(0.0)
	sdft_re.resize(3 * SDFT_BIN_COUNT)
	sdft_re.fill(0.0)
	sdft_im.resize(3 * SDFT_BIN_COUNT)
	sdft_im.fill(0.0)
	sdft_data.resize(SDFT_BIN_COUNT)
	sdft_data.fill(0.0)
	pk_bin.resize(dyn_count)
	pk_bin.fill(0)
	pk_val.resize(dyn_count)
	pk_val.fill(0.0)
	for arr in [dyn_center, dyn_a1, dyn_a2q, dyn_fq, dyn_ic1q, dyn_ic2]:
		arr.resize(3 * dyn_count)
		arr.fill(0.0)
	var c := PackedFloat64Array([1, 0, 0])
	for a in 3:
		for p in dyn_count:
			var i := a * dyn_count + p
			dyn_center[i] = (float(p) + 0.5) * (dyn_max_hz - dyn_min_hz) / float(dyn_count) + dyn_min_hz
			svf_notch_coeffs(c, dyn_center[i], dt, dyn_q)
			dyn_a1[i] = c[0]
			dyn_a2q[i] = c[1]
			dyn_fq[i] = c[2]


## dynNotchUpdate: the mean of dyn_sample_count loops goes into the SDFT, spread over as many loops (sdftPushBatch)
func _dyn_notch_update() -> void:
	if dyn_sample_index == dyn_sample_count:
		dyn_sample_index = 0
		for a in 3:
			dyn_avg[a] = dyn_acc[a] * dyn_sample_rcp
			dyn_acc[a] = 0.0
		dyn_tick = DYN_NOTCH_CALC_TICKS
	var bidx := dyn_sample_index
	var bstart := sdft_batch * bidx + sdft_start
	var last := bidx == sdft_batches - 1
	var bend := bstart + (sdft_end - bstart + 1 if last else sdft_batch)
	var xr := sdft_re
	var xi := sdft_im
	var tr := sdft_tw_re
	var ti := sdft_tw_im
	for a in 3:
		var o := a * SDFT_BIN_COUNT
		var si := a * SDFT_SAMPLE_SIZE + sdft_idx[a]
		var delta := dyn_avg[a] - sdft_rpn * sdft_samples[si]
		if last:
			sdft_samples[si] = dyn_avg[a]
			sdft_idx[a] = (sdft_idx[a] + 1) % SDFT_SAMPLE_SIZE
		for i in range(bstart, bend):
			var k := o + i
			var re := xr[k] + delta
			var im := xi[k]
			xr[k] = tr[i] * re - ti[i] * im
			xi[k] = tr[i] * im + ti[i] * re
		# updateEdges: the first bin outside the range on either side
		if sdft_start > 0 and bidx == 0:
			_sdft_bin(o, sdft_start - 1, delta)
		if sdft_end < SDFT_BIN_COUNT - 1 and last:
			_sdft_bin(o, sdft_end + 1, delta)
	dyn_sample_index += 1
	if dyn_tick > 0:
		_dyn_notch_process()
		dyn_tick -= 1


func _sdft_bin(o: int, i: int, delta: float) -> void:
	var re := sdft_re[o + i] + delta
	var im := sdft_im[o + i]
	sdft_re[o + i] = sdft_tw_re[i] * re - sdft_tw_im[i] * im
	sdft_im[o + i] = sdft_tw_re[i] * im + sdft_tw_im[i] * re


## dynNotchProcess: one of four steps (window, peaks, frequencies, filters) for one axis per loop
func _dyn_notch_process() -> void:
	var data := sdft_data
	var st := sdft_start
	var en := sdft_end
	if dyn_step == 0:
		# sdftWinSq: |X[k] - (X[k-1] + X[k+1]) / 2|^2, the Hann window in the frequency domain; at the last bin
		# X[k] - X[k-1] (sdft_start >= 1 always)
		var o := dyn_axis * SDFT_BIN_COUNT
		var xr := sdft_re
		var xi := sdft_im
		for i in range(st, en + 1):
			var l := o + i - 1
			var r := o + i + 1 if i + 1 < SDFT_BIN_COUNT else l
			var re := xr[o + i] - 0.5 * (xr[l] + xr[r])
			var im := xi[o + i] - 0.5 * (xi[l] + xi[r])
			data[i] = re * re + im * im
		dyn_threshold = 0.0
		for i in range(st, en + 1):
			dyn_threshold += data[i]
	elif dyn_step == 1:
		pk_bin.fill(0)
		pk_val.fill(0.0)
		var b := st + 1
		while b < en:
			if data[b] > data[b - 1] and data[b] > data[b + 1]:
				for p in dyn_count:
					if data[b] > pk_val[p]:
						for k in range(dyn_count - 1, p, -1):
							pk_bin[k] = pk_bin[k - 1]
							pk_val[k] = pk_val[k - 1]
						pk_bin[p] = b
						pk_val[p] = data[b]
						break
				b += 1          # the next bin cannot be a peak
			b += 1
		for p in range(dyn_count - 1, 0, -1):     # sort the peaks by bin (empty ones last)
			for k in p:
				if pk_bin[k] > pk_bin[k + 1] and pk_bin[k + 1] != 0:
					var tb := pk_bin[k]
					var tv := pk_val[k]
					pk_bin[k] = pk_bin[k + 1]
					pk_val[k] = pk_val[k + 1]
					pk_bin[k + 1] = tb
					pk_val[k + 1] = tv
	elif dyn_step == 2:
		var peaks := 0
		for p in dyn_count:
			if pk_bin[p] != 0:
				dyn_threshold -= 0.75 * data[pk_bin[p] - 1]
				dyn_threshold -= data[pk_bin[p]]
				dyn_threshold -= 0.75 * data[pk_bin[p] + 1]
				peaks += 1
		dyn_threshold /= float(en - st - peaks + 1)
		dyn_threshold *= 2.0
		for p in dyn_count:
			if pk_bin[p] != 0 and pk_val[p] > dyn_threshold:
				var mean_bin := float(pk_bin[p])
				var y0 := data[pk_bin[p] - 1]
				var y1 := data[pk_bin[p]]
				var y2 := data[pk_bin[p] + 1]
				var denom := 2.0 * (y0 - 2.0 * y1 + y2)
				if denom != 0.0:
					mean_bin += (y0 - y2) / denom
				var center := clampf(mean_bin * sdft_res_hz, dyn_min_hz, dyn_max_hz)
				var mult := clampf(pk_val[p] / dyn_threshold, 1.0, 10.0)
				var gain := pt1_gain(DYN_NOTCH_SMOOTH_HZ * mult, dyn_pt1_dt)
				var i := dyn_axis * dyn_count + p
				dyn_center[i] += gain * (center - dyn_center[i])
	else:
		var c := PackedFloat64Array([1, 0, 0])
		for p in dyn_count:
			if pk_bin[p] != 0 and pk_val[p] > dyn_threshold:
				var i := dyn_axis * dyn_count + p
				svf_notch_coeffs(c, dyn_center[i], dt, dyn_q)
				dyn_a1[i] = c[0]
				dyn_a2q[i] = c[1]
				dyn_fq[i] = c[2]
		dyn_axis = (dyn_axis + 1) % 3
	dyn_step = (dyn_step + 1) % 4


# ------------------------------------------------------------------ pid.c
var _gdt := PackedFloat64Array([0, 0, 0])   # pid_controller's scratch: the gyro for the D term
func pid_controller() -> void:
	ag_throttle_d *= ag_gain
	iterm_accel = ag_throttle_d * ANTIGRAVITY_KI
	var gdt := _gdt
	gdt[0] = gyro_filtered[0]
	gdt[1] = gyro_filtered[1]
	gdt[2] = gyro_filtered[2]
	if dn_on:
		_notch3(dn_s, dn_c, gdt)
	if dl1_on:
		_lpf(dl1_s, dl1_type, dl1_k, dl1_c, gdt)
	if dl2_on:
		_lpf(dl2_s, dl2_type, dl2_k, dl2_c, gdt)
	if iterm_rotation:      # rotateItermAndAxisError: the I vector turns with the craft
		var r := dt * PI / 180.0
		for i in 3:
			var i1 := (i + 1) % 3
			var i2 := (i + 2) % 3
			var rot := gyro_filtered[i] * r
			var nv := I[i1] + I[i2] * rot
			I[i2] -= I[i1] * rot
			I[i1] = nv
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
		pid_sum[a] = p_term + i_term + d_term + f_term + pid_inject[a]
	if not stabilisation:
		for a in 3:
			P[a] = 0.0; I[a] = 0.0; D[a] = 0.0; F[a] = 0.0; pid_sum[a] = 0.0
	elif iterm_reset:
		for a in 3:
			I[a] = 0.0


## dynLpfCutoffFreq, or (expo 0) dynThrottle
static func _dyn_cutoff(thr: float, lo: float, hi: float, expo: float) -> float:
	if expo > 0.0:
		var expof := expo / 10.0
		var curve := thr * (1.0 - thr) * expof + thr
		return (hi - lo) * curve + lo
	return maxf(thr * (1.0 - thr * thr / 3.0) * 1.5 * hi, lo)


## The new cutoff of a dynamic lowpass (dynLpfGyroUpdate, dynLpfDTermUpdate): returns its new gain (PT) or
## updates c (SVF)
func _dyn_lp_gain(raw_type: int, cutoff: float, c: PackedFloat64Array) -> float:
	if raw_type == 0:
		return pt1_gain(cutoff, dt)
	if raw_type == 2:
		return pt2_gain(cutoff, dt)
	if raw_type == 3:
		return pt3_gain(cutoff, dt)
	svf_lowpass_coeffs(c, minf(cutoff, 0.475 / dt), dt)
	return 1.0


func _dyn_lpf_update(throttle: float) -> void:
	if t_us - dyn_lpf_last_us < DYN_LPF_UPDATE_US:
		return
	var q := roundi(throttle * 100.0)
	if q == dyn_lpf_prev_q:
		return
	var thr := float(q) / 100.0
	if gdyn_lpf_min > 0.0 and gl1_on:
		gl1_k = _dyn_lp_gain(_gl1_raw_type, _dyn_cutoff(thr, gdyn_lpf_min, gdyn_lpf_max, gdyn_lpf_expo), gl1_c)
	if dyn_lpf_min > 0.0 and dl1_on:
		dl1_k = _dyn_lp_gain(_dl1_raw_type, _dyn_cutoff(thr, dyn_lpf_min, dyn_lpf_max, dyn_lpf_expo), dl1_c)
	dyn_lpf_prev_q = q
	dyn_lpf_last_us = t_us


# ------------------------------------------------------------------ mixer.c
var _mix := PackedFloat64Array([0, 0, 0, 0])   # mix_table's scratch
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
	if sag_factor > 0.0 and vbat_cells > 0:       # battery sag compensation: less output range on a full pack
		if t_us - sag_last_us >= SAG_TASK_US:
			sag_s = vbat_cv if sag_s < 0.0 else sag_s + sag_k * (vbat_cv - sag_s)
			sag_last_us = t_us
		var cell := float(int(sag_s) / vbat_cells)
		var good := 1.0 - clampf((sag_full - cell) / sag_range, 0.0, 1.0)
		range_max = out_high - (sag_range / sag_full) * good * sag_factor * (out_high - out_low)
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
	# TPA, classic (throttle is within 0..1 here)
	var tpa_rate := 0.0
	if throttle >= tpa_low_bp or tpa_low_bp <= 0.01 or tpa_low_faded:
		tpa_rate = tpa_mult * maxf(throttle - tpa_bp, 0.0)
		if tpa_low_always == 0 and not tpa_low_faded:
			tpa_low_faded = true
	else:
		tpa_rate = tpa_low_mult * (tpa_low_bp - throttle)
	tpa_factor = 1.0 - tpa_rate
	if t_us - dyn_lpf_last_us >= 5000:      # (_dyn_lpf_update() does nothing before)
		_dyn_lpf_update(throttle)
	if throttle_boost > 0.0:
		throttle_lpf_s += throttle_lpf_k * (throttle - throttle_lpf_s)
		throttle = clampf(throttle + throttle_boost * (throttle - throttle_lpf_s), 0.0, 1.0)
	if dyn_idle_min_rps > 0.0:
		throttle = maxf(throttle, 0.01)
	if thrust_lin != 0.0:
		throttle *= 1.0 - thrust_lin * (1.0 - throttle)
	# the quad X mixer, written out (motors 0 REAR_R, 1 FRONT_R, 2 REAR_L, 3 FRONT_L; the signs multiply exactly)
	var mix := _mix
	var v := -roll + pitch - yaw
	var mmax := maxf(v, 0.0)
	var mmin := minf(v, 0.0)
	mix[0] = v
	v = -roll - pitch + yaw
	mmax = maxf(v, mmax)
	mmin = minf(v, mmin)
	mix[1] = v
	v = roll + pitch + yaw
	mmax = maxf(v, mmax)
	mmin = minf(v, mmin)
	mix[2] = v
	v = roll - pitch - yaw
	mmax = maxf(v, mmax)
	mmin = minf(v, mmin)
	mix[3] = v
	motor_mix_range = mmax - mmin
	var air_pct := 1.0
	if not airmode and throttle < 0.5:
		air_pct = 0.5 + throttle
	var norm := air_pct / motor_mix_range if motor_mix_range > 1.0 else air_pct
	throttle = clampf(throttle, -mmin * norm, 1.0 - mmax * norm)
	for i in 4:
		var o := mix[i] * norm + throttle
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


## The pack's voltage, for the battery sag compensation (the firmware's voltage meter, cells detected)
func set_vbat(pack_v: float, cells: int) -> void:
	vbat_cv = pack_v * 100.0
	vbat_cells = cells


## One PID loop (4 kHz). Returns motor outputs in DShot units (0 = stopped).
func step(gyro_dps: PackedFloat64Array, telemetry_hz: PackedFloat64Array) -> PackedFloat64Array:
	gyro_update(gyro_dps, telemetry_hz)
	process_rc_command()
	pid_controller()
	mix_table()
	t_us += roundi(dt * 1e6)
	return motor
