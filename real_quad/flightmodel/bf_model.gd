extends RefCounted
## real_quad: the flight model of one craft: the Betaflight rate loop (bf_controller.gd) flying the quad model
## identified from blackbox logs (bf_quad.gd). Loaded from a model package directory
## (models/<id>/ with craft.json, fc.json, plant.json; see real_quad/README.txt).
##
## Runs as a custom integrator on a RigidBody3D: each physics tick (2 ms at 500 Hz) is sub-stepped
## at the firmware's PID rate (fc.json looptime_us * pid_process_denom: 4 kHz for freestyle5, 8 kHz
## for a 3.5" with denom 1). RC frames arrive at the receiver link rate (fc.json rx_rate_hz), motor
## commands reach the motors with the identified latency. Godot only integrates the pose from the mean
## velocities handed back each tick and resolves collisions; when a contact (or a teleport) changed
## those velocities, the internal state is resynchronised from the body.
##
## Interface used by the host (a RigidBody3D script): configure_body(), reset(),
## integrate_forces(), plus settings/telemetry getters. Once a tick, before integrate_forces(), the
## host hands over the air (apply_air(): dirty_air's prop wash, floor and ceiling at each prop, or
## clean air) and the battery (set_pack() from the shared pack, fed with take_charge(); or
## use_identified_sag()).

const Controller := preload("bf_controller.gd")
const Quad := preload("bf_quad.gd")

const USE_RPM_FILTER := true
const CELL_FULL_LIPO := 4.2          # a full cell at rest [V]: the identified battery_v0 is a full pack's
const CELL_FULL_LIHV := 4.35
# Betaflight's motor index (0 rear right, 1 front right, 2 rear left, 3 front left) ->
# dirty_air's prop order (front left, front right, rear left, rear right)
const DIRTY_AIR_PROP := [3, 1, 2, 0]

var package_dir := ""
var valid := false                   # false if the package could not be read (no physics then)
var craft: Dictionary                # craft.json: id, name, zone_vehicle_id, camera, ...
var fc: Controller
var quad: Quad
var fc_cfg: Dictionary               # fc.json
var plant_params: Dictionary         # plant.json
var cpu_us_per_tick: float = 0.0     # smoothed cost of integrate_forces(), for the OSD
## Prop wash strength as a multiple of the calibrated one: scales the wash and wake handed to dirty_air.
var propwash_scale: float = 1.0
var air_wash_level: float = 0.0      # dirty_air's wash level at the quad last tick (telemetry)
var cells: int = 6                   # the pack (craft.json "battery"), see battery_spec()
var lihv := false
var _cell_full: float = CELL_FULL_LIPO   # a full cell of the pack in use, at rest
var _r_cell: float = -1.0            # identified_cell_ohm(), once worked out

# Gyro vibration of props out of balance (prop_damage), chosen by feel, not identified: a tone at each
# damaged rotor's own speed (the RPM filter takes most of it off) and noise above it, as much as a
# fully out-of-balance prop at full speed makes
const VIB_TONE_DPS := 25.0           # peak of the tone on roll and pitch [deg/s]
const VIB_NOISE_DPS := 4.0           # rms of the noise (band limit VIB_NOISE_HZ), roll and pitch; yaw half [deg/s]
const VIB_NOISE_HZ := 300.0

var _imb := PackedFloat64Array([0, 0, 0, 0])     # each prop's out-of-balance (prop_damage), Betaflight's motor order
var _vib := PackedFloat64Array([0, 0, 0, 0])     # ... times its spin, this tick
var _vib_phase := PackedFloat64Array([0, 0, 0, 0])
var _vib_noise := Vector3()
var _vib_on := false
var _full_w := -1.0                  # quad.full_speed(), once worked out
var _armed := false
var _t_us: int = 0
var _next_frame_us: int = 0
var _rx_frame_us: int = 4000         # from fc.json rx_rate_hz
var _mq := PackedFloat64Array()      # motor command latency line: a ring of _mq_n commands (4 each)
var _mq_n: int = 1
var _mq_i: int = 0
var _cd_area0 := Vector3()           # the identified body drag (cd_area), before the game's drag multipliers
var _pkg_rates := {}                 # fc.json's rates (rates_type, rc_rates, rates, rc_expo), for use_package_rates()
var _vel := Vector3()                # world velocity at the end of the last tick (internal state)
var _set_lin := Vector3()            # velocities handed to Godot last tick (to detect contacts)
var _set_ang := Vector3()
var _have_state := false


## package_dir holds craft.json (identity, camera), fc.json (Betaflight settings) and plant.json
## (identified plant). Check `valid` afterwards.
func _init(p_package_dir: String) -> void:
	package_dir = p_package_dir
	craft = _load_json(package_dir.path_join("craft.json"))
	fc_cfg = _load_json(package_dir.path_join("fc.json"))
	plant_params = _load_json(package_dir.path_join("plant.json"))
	if craft.is_empty() or fc_cfg.is_empty() or plant_params.is_empty():
		return
	_rx_frame_us = maxi(1, roundi(1e6 / maxf(float(fc_cfg.get("rx_rate_hz", 250.0)), 1.0)))
	quad = Quad.new(plant_params)
	_cd_area0 = quad.cd_area
	for k in ["rates_type", "rc_rates", "rates", "rc_expo"]:
		_pkg_rates[k] = fc_cfg[k].duplicate() if fc_cfg[k] is Array else fc_cfg[k]
	var bat: Dictionary = craft.get("battery", {}) if craft.get("battery", {}) is Dictionary else {}
	cells = clampi(int(bat.get("cells", craft.get("cells", roundi(quad.batt_v0 / CELL_FULL_LIPO)))), 1, 12)
	lihv = bool(bat.get("lihv", false))
	_cell_full = CELL_FULL_LIHV if lihv else CELL_FULL_LIPO
	quad.volts_per_pack_volt = _k_volts()
	_new_controller()
	valid = true


static func _load_json(path: String) -> Dictionary:
	var d: Variant = JSON.parse_string(FileAccess.get_file_as_string(path)) if FileAccess.file_exists(path) else null
	if not (d is Dictionary):
		push_error("bf_model: cannot read %s" % path)
		return {}
	return d


func _new_controller() -> void:
	fc = Controller.new(fc_cfg, USE_RPM_FILTER)
	var delay_steps := maxi(1, roundi(float(plant_params.get("motor_delay_s", 0.001)) / fc.dt))
	_mq_n = delay_steps
	_mq.resize(4 * _mq_n)
	_mq.fill(0.0)
	_mq_i = 0


# ------------------------------------------------------------------ settings
## In-plane rotor drag expressed as 1/s at hover (the part of the air drag felt at low and
## medium speed). Not measurable from the logs, hence adjustable; top speed barely depends on it.
func get_default_linear_drag() -> float:
	return float(plant_params.rotor_drag) * _drag_per_s_factor()


func get_linear_drag() -> float:
	return quad.rotor_drag * _drag_per_s_factor()


func set_linear_drag(per_s: float) -> void:
	quad.rotor_drag = per_s / _drag_per_s_factor()


func _drag_per_s_factor() -> float:
	return float(plant_params.linear_drag_ref_sum_w) / float(plant_params.mass)


## The drag the package wants by default (craft.json linear_drag, 1/s at hover), else the
## identified one.
func get_package_linear_drag() -> float:
	var d := float(craft.get("linear_drag", 0.0))
	return d if d > 0.0 else get_default_linear_drag()


## The game's drag multipliers (Physics menu, surface_area_x/y/z_multi, percent; 100 = unchanged) scale the
## body drag: the game's z (front/back axis of the body) is FLU x, its x (left/right) FLU y, its y (top/bottom) FLU z.
func set_drag_multipliers(game_x: float, game_y: float, game_z: float) -> void:
	quad.cd_area = _cd_area0 * Vector3(maxf(game_z, 0.0), maxf(game_x, 0.0), maxf(game_y, 0.0)) * 0.01


## The game's wind (ZGamestate.current_wind_effect, world, m/s - its gusts and shelter included) as the air's
## own velocity: the quad flies through it, for the body's drag and the props' inflow.
func set_wind(world: Vector3) -> void:
	quad.wind = world


## The game's own Rates (ZSettings.RATES: {type, roll, pitch, yaw: Vector3}) as the controller's rates. The game's
## three types: "actual" (center, max, expo) -> Betaflight ACTUAL; "betaflight" (RC rate, rate, expo) -> BETAFLIGHT;
## "kiss" (RC rate, rate, curve) -> KISS. Returns false (nothing changed) for a type this does not know.
func set_game_rates(r: Dictionary) -> bool:
	var type := str(r.get("type", ""))
	var rc_rates := []
	var rates := []
	var expo := []
	for axis in ["roll", "pitch", "yaw"]:
		var v: Vector3 = r.get(axis, Vector3.ZERO)
		if type == "actual":
			rc_rates.append(v.x * 0.1)
			rates.append(v.y * 0.1)
			expo.append(v.z * 100.0)
		elif type == "betaflight" or type == "kiss":
			rc_rates.append(v.x * 100.0)
			rates.append(v.y * 100.0)
			expo.append(v.z * 100.0)
		else:
			return false
	fc_cfg["rates_type"] = {"actual": 3, "betaflight": 0, "kiss": 1}[type]
	fc_cfg["rc_rates"] = rc_rates
	fc_cfg["rates"] = rates
	fc_cfg["rc_expo"] = expo
	fc.update_rates()
	return true


## The rates of fc.json again
func use_package_rates() -> void:
	for k in _pkg_rates:
		fc_cfg[k] = _pkg_rates[k].duplicate() if _pkg_rates[k] is Array else _pkg_rates[k]
	fc.update_rates()


## The highest rate the sticks ask for now [deg/s] (the game's OSD readout curr_max_rate)
func current_max_rate_dps(rc: Dictionary) -> float:
	var m := 0.0
	for pair in [["roll", 0], ["pitch", 1], ["yaw", 2]]:
		var c := clampf(float(rc.get(pair[0], 0.0)), -1.0, 1.0)
		m = maxf(m, absf(fc.apply_rates(pair[1], c, absf(c))))
	return m


## Rotor speed at full throttle in clean air on a full pack [rpm] (the top of the sound's rpm scale)
func full_rpm() -> float:
	if _full_w < 0.0:
		_full_w = maxf(quad.full_speed(), 1.0)
	return _full_w * 60.0 / TAU


## Thrust factor of the rotor at Betaflight motor index i (0 rear right, 1 front right, 2 rear
## left, 3 front left): 1 = none. Set by the host every tick.
func set_thrust_scale(i: int, v: float) -> void:
	quad.thrust_scale[i] = v


## The props' condition from prop_damage (its set_model()), each in Betaflight's motor order: thrust factor
## (1 = new, 0 = broken off) into the rotor's thrust, and the out-of-balance (0..1) shaking the gyro
## the flight controller reads - so its PID reacts as on a quad with a bent prop.
func set_props(thrust: Array, imbalance: Array) -> void:
	var any := false
	for i in 4:
		quad.prop_scale[i] = clampf(float(thrust[i]), 0.0, 1.0)
		_imb[i] = clampf(float(imbalance[i]), 0.0, 1.0)
		any = any or _imb[i] > 0.0
	_vib_on = any
	if not any:
		_vib.fill(0.0)
		_vib_noise = Vector3()


## Undamaged props (no prop_damage)
func clear_props() -> void:
	quad.prop_scale.fill(1.0)
	_imb.fill(0.0)
	_vib.fill(0.0)
	_vib_on = false


## How hard each rotor turns, 0..1.2: its thrust against full thrust (speed squared over the full-throttle
## speed squared), Betaflight's motor order. prop_damage wears props by it.
func spin_share() -> PackedFloat64Array:
	if _full_w < 0.0:
		_full_w = maxf(quad.full_speed(), 1.0)
	var out := PackedFloat64Array([0, 0, 0, 0])
	var k := 1.0 / (_full_w * _full_w)
	for i in 4:
		out[i] = minf(quad.w[i] * quad.w[i] * k, 1.2)
	return out


## Propwash strength as a multiplier of the calibrated value (0 = off).
func get_propwash_scale() -> float:
	return propwash_scale


func set_propwash_scale(v: float) -> void:
	propwash_scale = maxf(v, 0.0)


# ------------------------------------------------------------------ air (dirty_air)
## dirty_air's wash and wake for this quad, calibrated on its logs (plant.json dirty_air_wash /
## dirty_air_wake; 1 = dirty_air's own figures), times the drone's propwash setting: wash sizes
## each prop's thrust fluctuation in the ring state (and the gusts), wake the air left behind.
func get_air_wash() -> float:
	return float(plant_params.get("dirty_air_wash", 1.0)) * propwash_scale


func get_air_wake() -> float:
	return float(plant_params.get("dirty_air_wake", 1.0)) * propwash_scale


## The turbulence of dirty_air's old wake for this quad, as a multiple of its own (plant.json
## dirty_air_gust; not scaled by propwash, which scales it through the wash and wake already).
func get_air_gust() -> float:
	return float(plant_params.get("dirty_air_gust", 1.0))


## Takes the air at the props for the coming tick from dirty_air's Air node (dirty_air/air.gd), or
## clean air for null: the old wake's flow and gusts at each prop, the descent curve and vortex ring
## state, and (ground_ceiling) the floor and ceiling effect per prop. Hands dirty_air this quad's
## thrust, props, frame, wash and wake for its next tick, so it works the air out for this quad and
## leaves the body alone (see set_model() in air.gd).
func apply_air(air: Object, ground_ceiling := true) -> void:
	if air == null or not air.has_method("set_model"):
		quad.clear_air()
		air_wash_level = 0.0
		for i in 4:
			quad.thrust_scale[i] = 1.0
		return
	var t := 0.0
	for i in 4:
		t += quad.thrust[i]
	air.call("set_model", {
		"thrust": t, "prop_mm": quad.prop_radius * 2000.0, "wheelbase_mm": quad.arm * 2.0 * sqrt(2.0) * 1000.0,
		"ducted": bool(craft.get("ducted", false)), "wash": get_air_wash(), "wake": get_air_wake(), "gust": get_air_gust(),
	})
	var g = air.get("ground")
	var c = air.get("ceiling")
	var have_gc: bool = ground_ceiling and g is Array and c is Array and g.size() >= 4 and c.size() >= 4
	for i in 4:
		var k: int = DIRTY_AIR_PROP[i]
		quad.thrust_scale[i] = float(g[k]) * float(c[k]) if have_gc else 1.0
	# its air for this tick, worked out for this model (not while disarmed or before its first tick)
	if int(air.get("model_air_frame")) != Engine.get_physics_frames():
		quad.clear_air()
		air_wash_level = 0.0
		return
	quad.has_air = true
	quad.air_vel = air.get("wake_flow")
	quad.air_mean = float(air.get("descent_mean"))
	var ax: Array = air.get("prop_axial")
	var fl: Array = air.get("prop_vrs")
	for i in 4:
		var k: int = DIRTY_AIR_PROP[i]
		quad.air_axial[i] = float(ax[k])
		quad.air_fluct[i] = float(fl[k])
	air_wash_level = float(air.get("wash_level"))


# ------------------------------------------------------------------ battery
## Model volts per pack volt: the identified battery_v0 is a full pack at rest (cells x a full cell).
func _k_volts() -> float:
	return quad.batt_v0 / (float(cells) * _cell_full)


## The identified sag as a cell's internal resistance [ohm]. The identification's sag is
## battery_r * current_proxy (model volts); the model's current is the rotors' power over the drive
## efficiency. Their ratio is taken at full throttle, hovering in still air (where the sag matters;
## at hover the absolute difference is a few hundredths of a volt).
func identified_cell_ohm() -> float:
	var u := 1.0
	var v := quad.batt_v0
	var ws := PackedFloat64Array([0, 0, 0, 0])
	var ip := 0.0
	for it in 60:                    # steady state: a*(u*V - w/Kv) = b*w^2 + c, V = v0 - r*proxy
		ip = 0.0
		for i in 4:
			var bb := quad.ma[i] / quad.kv
			var cc := quad.mc[i] - quad.ma[i] * u * v
			ws[i] = (-bb + sqrt(bb * bb - 4.0 * quad.mb[i] * cc)) / (2.0 * quad.mb[i])
			ip += u * maxf(u * v - ws[i] / quad.kv, 0.0)
		v = quad.batt_v0 - quad.batt_r * ip
	var p_mech := 0.0
	for i in 4:
		p_mech += quad.kQ * ws[i] * ws[i] * ws[i]
	var k := _k_volts()
	var amps := p_mech * k / (quad.drive_efficiency * v)
	return quad.batt_r * ip / maxf(amps, 1e-3) / (k * float(cells))


## The pack this craft flies on, as battery_sag's packs are given (craft.json "battery": cells,
## capacity_mah, lihv; resistance from the identification). capacity_mah 0 = not in the package.
func battery_spec() -> Dictionary:
	var bat: Dictionary = craft.get("battery", {}) if craft.get("battery", {}) is Dictionary else {}
	if _r_cell < 0.0:
		_r_cell = identified_cell_ohm()
	var s := {"name": get_display_name(), "cells": cells, "lihv": lihv,
		"capacity_mah": float(bat.get("capacity_mah", 0.0)), "resistance_mohm": snappedf(_r_cell * 1000.0, 0.01)}
	if bat.has("idle_amps"):
		s["idle_amps"] = float(bat.idle_amps)
	return s


## The motors' voltage from the shared pack for the coming tick: cell_source_v = a cell's voltage
## behind its ohmic resistance (resting voltage minus polarisation), cell_ohm = that resistance,
## cell_full_v = a full cell at rest. The ohmic drop is worked out at every PID step.
func set_pack(cell_source_v: float, cell_ohm: float, cell_full_v: float) -> void:
	_cell_full = cell_full_v
	var k := _k_volts() * float(cells)
	quad.volts_per_pack_volt = _k_volts()
	quad.src_v = k * cell_source_v
	quad.src_rp = 0.0
	quad.src_ra = k * cell_ohm


## The identified sag of a fresh pack, without drain (no shared pack).
func use_identified_sag() -> void:
	quad.volts_per_pack_volt = _k_volts()
	quad.src_v = quad.batt_v0
	quad.src_rp = quad.batt_r
	quad.src_ra = 0.0


## The motors' charge since the last call: Vector3(amp-seconds, seconds, mean cell voltage the
## motors saw).
func take_charge() -> Vector3:
	var t := quad.charge_t
	var c := Vector3(quad.charge_as, t, quad.charge_vs / t / (_k_volts() * float(cells)) if t > 0.0 else quad.v_batt / (_k_volts() * float(cells)))
	quad.charge_as = 0.0
	quad.charge_vs = 0.0
	quad.charge_t = 0.0
	return c


# ------------------------------------------------------------------ host interface
func get_id() -> String:
	return str(craft.get("id", package_dir.get_file()))


func get_display_name() -> String:
	return str(craft.get("name", get_id()))


## Zone vehicle this model is made for (ZGamestate.selected_vehicle_id), "" if unspecified.
func get_zone_vehicle_id() -> String:
	return str(craft.get("zone_vehicle_id", ""))


## PID loop rate of the modelled firmware [Hz].
func get_loop_hz() -> float:
	return 1.0 / fc.dt if valid else 0.0


func uses_custom_integrator() -> bool:
	return true


## The modelled quad's mass and inertia on the body (the host re-applies them every tick: the game sets
## the mass of its Physics menu again whenever it reloads its settings)
func apply_mass(body: RigidBody3D) -> void:
	var m := float(plant_params.mass)
	if not is_equal_approx(body.mass, m):
		body.mass = m
	# FLU inertia -> Godot local axes: X = -left (pitch axis), Y = up (yaw), Z = -forward (roll)
	var i := Vector3(float(plant_params.Iyy), float(plant_params.Izz), float(plant_params.Ixx))
	if body.inertia != i:
		body.inertia = i


func configure_body(body: RigidBody3D) -> void:
	if not valid:
		return
	body.custom_integrator = true
	body.can_sleep = false
	apply_mass(body)
	body.center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
	body.center_of_mass = Vector3.ZERO
	body.linear_damp_mode = RigidBody3D.DAMP_MODE_REPLACE
	body.angular_damp_mode = RigidBody3D.DAMP_MODE_REPLACE
	body.linear_damp = 0.0
	body.angular_damp = 0.0
	body.gravity_scale = 1.0


func reset() -> void:
	if not valid:
		return
	quad.reset()
	_new_controller()
	_armed = false
	_t_us = 0
	_next_frame_us = 0
	_have_state = false
	_vel = Vector3()


func get_camera_angle_deg() -> float:
	return float(craft.get("camera_angle_deg", 20.0))    # camera uptilt of the real quad


func get_camera_fov_deg() -> float:
	return float(craft.get("camera_fov_deg", 123.0))


func get_telemetry() -> Dictionary:
	var rpm := PackedInt32Array()
	for i in 4:
		rpm.append(roundi(quad.w[i] * 60.0 / TAU))
	return {
		"rpm": rpm,
		"vbat": "%.1f V" % (quad.v_batt / _k_volts()),
		"amps": "%.1f A" % quad.amps,
		"propwash": "%.2f" % air_wash_level,
		"pidsum": "%d %d %d" % [roundi(fc.pid_sum[0]), roundi(fc.pid_sum[1]), roundi(fc.pid_sum[2])],
		"cpu": "%.2f ms/tick" % (cpu_us_per_tick / 1000.0),
	}


## Stick dict (-1..1; roll right, pitch forward, yaw right, throttle up positive) -> rcData 1000..2000.
static func rc_to_rcdata(rc: Dictionary) -> PackedFloat64Array:
	var roll := clampf(float(rc.get("roll", 0.0)), -1.0, 1.0)
	var pitch := clampf(float(rc.get("pitch", 0.0)), -1.0, 1.0)
	var yaw := clampf(float(rc.get("yaw", 0.0)), -1.0, 1.0)
	var thr := clampf((float(rc.get("throttle", -1.0)) + 1.0) * 0.5, 0.0, 1.0)
	return PackedFloat64Array([1500.0 + 500.0 * roll, 1500.0 + 500.0 * pitch, 1500.0 + 500.0 * yaw, 1000.0 + 1000.0 * thr])


# Out-of-balance props into the gyro (deg/s): each damaged rotor's tone at its own speed, rotating as the
# imbalance does, and the noise of all of them
func _vibrate(gyro: PackedFloat64Array, dt: float, a: float, sc: float) -> void:
	for i in 4:
		if _vib[i] > 0.0:
			_vib_phase[i] = fmod(_vib_phase[i] + quad.w[i] * dt, TAU)
			gyro[0] += VIB_TONE_DPS * _vib[i] * sin(_vib_phase[i])
			gyro[1] += VIB_TONE_DPS * _vib[i] * cos(_vib_phase[i])
	_vib_noise += (Vector3(quad.rng.randfn(), quad.rng.randfn(), quad.rng.randfn()) * sqrt((2.0 - a) / a) - _vib_noise) * a   # unit variance
	gyro[0] += _vib_noise.x * sc
	gyro[1] += _vib_noise.y * sc
	gyro[2] += _vib_noise.z * sc * 0.5


func integrate_forces(_body: RigidBody3D, state: PhysicsDirectBodyState3D, rc: Dictionary, armed: bool) -> void:
	if not valid:
		return
	var t0 := Time.get_ticks_usec()
	var tick_dt := state.step
	var basis := state.transform.basis.orthonormalized()
	var pos := state.transform.origin
	var pos0 := pos
	# resync when something else (collision, respawn, teleport) changed the body's velocities
	var lin := state.linear_velocity
	var ang := state.angular_velocity
	if not _have_state or lin.distance_to(_set_lin) > 1e-3 or ang.distance_to(_set_ang) > 1e-3:
		_vel = lin
		quad.omega = Quad.local_to_flu(basis.inverse() * ang)
		_have_state = true
	if armed != _armed:
		fc.arm(armed)
		_armed = armed
	var rcdata := rc_to_rcdata(rc)
	var n_sub := maxi(1, roundi(tick_dt / fc.dt))
	var dt := tick_dt / float(n_sub)
	var gyro := PackedFloat64Array([0, 0, 0])
	var telemetry_hz := PackedFloat64Array([0, 0, 0, 0])
	var u := PackedFloat64Array([0, 0, 0, 0])
	var ud := PackedFloat64Array([0, 0, 0, 0])
	var ang_sum := Vector3()
	var vib_lvl := 0.0
	if _vib_on:
		var sp := spin_share()
		for i in 4:
			_vib[i] = _imb[i] * sp[i]
			vib_lvl += _vib[i]
	var vib_a := 1.0 - exp(-TAU * VIB_NOISE_HZ * dt)
	var vib_sc := VIB_NOISE_DPS * sqrt(vib_lvl)
	for k in n_sub:
		if _t_us >= _next_frame_us:
			fc.rx_frame(rcdata, _t_us + 1)
			_next_frame_us += _rx_frame_us
		fc.t_us = _t_us
		var om := quad.omega * (180.0 / PI)
		gyro[0] = om.x
		gyro[1] = om.y
		gyro[2] = om.z
		if vib_lvl > 0.0:
			_vibrate(gyro, dt, vib_a, vib_sc)
		for i in 4:
			telemetry_hz[i] = quad.w[i] / TAU
		var m := fc.step(gyro, telemetry_hz)
		for i in 4:     # DShot 48..2047 -> 0..1; 0 = motor stopped (disarmed)
			u[i] = clampf((m[i] - 48.0) / 1999.0, 0.0, 1.0) if m[i] > 0.0 else 0.0
		var o := _mq_i * 4
		for i in 4:                  # the command of _mq_n steps ago goes to the motors, this one waits
			var c := _mq[o + i]
			_mq[o + i] = u[i]
			ud[i] = c
		_mq_i = (_mq_i + 1) % _mq_n
		var acc := quad.step(ud, dt, basis, _vel, pos, armed)
		# attitude update with the new body rates
		var rot_local := Quad.flu_to_local(quad.omega) * dt
		ang_sum += basis * rot_local
		var angle := rot_local.length()
		if angle > 1e-12:
			basis = basis * Basis(rot_local / angle, angle)
		_vel += acc * dt
		pos += _vel * dt
		_t_us += roundi(dt * 1e6)
	# hand back mean velocities that move the body from its start pose to the integrated pose.
	# (A quaternion log of the tick's rotation would not work: Godot's math is single precision
	# and cos() of a 1e-4 rad rotation rounds to exactly 1.)
	var lin_avg := (pos - pos0) / tick_dt
	var ang_avg := ang_sum / tick_dt
	state.linear_velocity = lin_avg
	state.angular_velocity = ang_avg
	_set_lin = lin_avg
	_set_ang = ang_avg
	cpu_us_per_tick = 0.9 * cpu_us_per_tick + 0.1 * float(Time.get_ticks_usec() - t0)
