extends RefCounted
## real_quad logfit: craft classes (contract 4.2): form defaults, fallbacks and plausibility bands.
##
## Defaults are what a missing form value falls back to; the bands only produce a WARN with the value
## (an unusual build can be right). "validated" classes are the two with identified reference models.

const ORDER := ["whoop", "micro", "mini", "five", "seven", "ten"]

const CLASSES := {
	"whoop": {"props_in": [1.2, 1.6], "status": "extrapolated", "cells": [1, 2], "weight_g": [18, 60],
		"D": 1.6, "pitch": 1.2, "blades": 4, "capacity": 300, "ducted": true,
		"hover_hz": [450, 900], "tau_ms": [20, 50], "batt_mohm_cell": [40, 200], "top_kmh": [30, 70],
		"chirp": {"end_deci_hz": 3000, "amp_rp": 150, "amp_y": 120, "time_s": 20}, "zone": "65mm_freestyle"},
	"micro": {"props_in": [2.0, 2.5], "status": "extrapolated", "cells": [2, 4], "weight_g": [50, 200],
		"D": 2.5, "pitch": 1.9, "blades": 3, "capacity": 650, "ducted": false,
		"hover_hz": [300, 550], "tau_ms": [12, 35], "batt_mohm_cell": [15, 60], "top_kmh": [60, 110],
		"chirp": {"end_deci_hz": 2000, "amp_rp": 230, "amp_y": 180, "time_s": 20}, "zone": "2.5in_freestyle"},
	"mini": {"props_in": [3.0, 3.5], "status": "validated", "cells": [4, 6], "weight_g": [180, 400],
		"D": 3.5, "pitch": 2.5, "blades": 3, "capacity": 850, "ducted": false,
		"hover_hz": [180, 300], "tau_ms": [12, 35], "batt_mohm_cell": [8, 25], "top_kmh": [100, 150],
		"chirp": {"end_deci_hz": 2000, "amp_rp": 230, "amp_y": 180, "time_s": 20}, "zone": "3.5in_freestyle",
		"dirty_air": {"wash": 1.0, "wake": 1.0, "gust": 2.2}},
	"five": {"props_in": [4.0, 5.5], "status": "validated", "cells": [4, 6], "weight_g": [450, 850],
		"D": 5.1, "pitch": 4.5, "blades": 3, "capacity": 1300, "ducted": false,
		"hover_hz": [130, 220], "tau_ms": [18, 40], "batt_mohm_cell": [2, 12], "top_kmh": [130, 190],
		"chirp": {"end_deci_hz": 2000, "amp_rp": 230, "amp_y": 180, "time_s": 20}, "zone": "5in_freestyle",
		"dirty_air": {"wash": 0.7, "wake": 1.0, "gust": 10.5}},
	"seven": {"props_in": [6.0, 7.0], "status": "extrapolated", "cells": [6, 6], "weight_g": [700, 1300],
		"D": 7.0, "pitch": 4.0, "blades": 3, "capacity": 2200, "ducted": false,
		"hover_hz": [80, 150], "tau_ms": [35, 70], "batt_mohm_cell": [1, 10], "top_kmh": [120, 170],
		"chirp": {"end_deci_hz": 1200, "amp_rp": 150, "amp_y": 120, "time_s": 30}, "zone": "5in_freestyle"},
	"ten": {"props_in": [8.0, 10.0], "status": "extrapolated", "cells": [6, 8], "weight_g": [1200, 3000],
		"D": 10.0, "pitch": 5.0, "blades": 2, "capacity": 5000, "ducted": false,
		"hover_hz": [50, 110], "tau_ms": [50, 100], "batt_mohm_cell": [1, 10], "top_kmh": [100, 160],
		"chirp": {"end_deci_hz": 1200, "amp_rp": 150, "amp_y": 120, "time_s": 30}, "zone": "5in_freestyle"},
}

## Every class: ratio bands of the scaled references (contract 4.2).
const BANDS := {"kT": [0.5, 2.0], "I": [0.4, 2.5], "J_rotor": [0.3, 3.0], "kQ_over_kT": [0.5, 2.0], "rate_damping": [0.0, 5.0]}

const PITFALLS := {
	"whoop": "Many AIO boards run BLHeli_S without bidirectional DShot (flash Bluejay; brushed whoops are impossible). Ducts: kT absorbs the duct's thrust gain at hover, but the prop inflow model and the drag defaults are for open props, so punch and top speed are approximate. Some tiny motors have 12 poles (motor_poles wrong -> every rotor speed and kT wrong). Weigh to 0.1 g. Small flash: 1 kHz without high resolution. 1S sag is large.",
	"micro": "Cinewhoop ducts as for whoops; a GoPro/naked camera on or off changes the weight by 20-30 %.",
	"mini": "Validated class. GoPro on/off between logs is the commonest mixed configuration (the hover spread warning catches it).",
	"five": "Validated class. GoPro on/off between logs is the commonest mixed configuration (the hover spread warning catches it).",
	"seven": "Frame and arm flex can put structural modes near 20-40 Hz, inside the rotation band. Slow motors: the fixed 1 ms motor delay may be short. 2 kHz loops on F411 boards: blackbox_sample_rate = 1/2 for 1 kHz. Long-range pilots rarely hover: fly an explicit 10 s hover. Li-ion: the cell count from voltage is ambiguous.",
	"ten": "Frame and arm flex can put structural modes near 20-40 Hz, inside the rotation band. Slow motors: the fixed 1 ms motor delay may be short. Long-range pilots rarely hover: fly an explicit 10 s hover. Li-ion: the cell count from voltage is ambiguous, sag is large. No Zone drone of this size exists: it is assigned to the 5\".",
}


static func get_class_data(cls: String) -> Dictionary:
	return CLASSES[cls]


## The validated class whose dirty_air values an unvalidated class borrows (contract 4.1, decision 3).
static func nearest_validated(cls: String) -> String:
	var i := ORDER.find(cls)
	var best := ""
	var bd := 1 << 30
	for k in ORDER.size():
		var c: String = ORDER[k]
		if CLASSES[c].status != "validated":
			continue
		var d := absi(k - i)
		if d < bd:
			bd = d
			best = c
	return best


## Class whose weight band contains the weight; ties broken by the hover rotor Hz band; no band: the
## nearest band (in log-weight).
static func estimate(weight_g: float, hover_hz: float = 0.0) -> String:
	var hits: Array = []
	for c in ORDER:
		var w: Array = CLASSES[c].weight_g
		if w[0] <= weight_g and weight_g <= w[1]:
			hits.append(c)
	if hits.size() > 1 and hover_hz > 0.0:
		var inband: Array = []
		for c in hits:
			var h: Array = CLASSES[c].hover_hz
			if h[0] <= hover_hz and hover_hz <= h[1]:
				inband.append(c)
		if not inband.is_empty():
			hits = inband
	if not hits.is_empty():
		return hits[0]
	var best := ""
	var bd := INF
	for c in ORDER:
		var w: Array = CLASSES[c].weight_g
		var d := minf(absf(log(weight_g / w[0])), absf(log(weight_g / w[1])))
		if d < bd:
			bd = d
			best = c
	return best


## The CLI block of contract 3.4 for a class and a PID loop rate in kHz (the log comes out at 1 kHz).
static func chirp_cli(cls: String, pid_khz: float) -> String:
	var ch: Dictionary = CLASSES[cls].chirp
	var k := maxi(1, roundi(pid_khz))
	var lines := ["set blackbox_sample_rate = 1/%d" % k, "set blackbox_mode = NORMAL", "set debug_mode = NONE",
		"set blackbox_disable_pids = ON", "set blackbox_disable_rc = OFF", "set blackbox_disable_setpoint = OFF",
		"set blackbox_disable_gyro = OFF", "set blackbox_disable_gyrounfilt = OFF", "set blackbox_disable_acc = OFF",
		"set blackbox_disable_motors = OFF", "set blackbox_disable_rpm = OFF", "set blackbox_disable_bat = OFF",
		"set blackbox_disable_attitude = ON", "set blackbox_disable_debug = ON", "set blackbox_disable_alt = ON",
		"set blackbox_disable_rssi = ON", "set blackbox_disable_mag = ON", "set blackbox_disable_gps = ON",
		"set blackbox_disable_servos = ON", "set dshot_bidir = ON", "set chirp_frequency_start_deci_hz = 30",
		"set chirp_frequency_end_deci_hz = %d" % ch.end_deci_hz, "set chirp_time_seconds = %d" % ch.time_s,
		"set chirp_amplitude_roll = %d" % ch.amp_rp, "set chirp_amplitude_pitch = %d" % ch.amp_rp,
		"set chirp_amplitude_yaw = %d" % ch.amp_y, "save"]
	return "\n".join(lines)
