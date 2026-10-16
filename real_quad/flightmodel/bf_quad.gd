extends RefCounted
## real_quad: the physical model ("plant") of a quad: ESC/motors, battery, props, rigid body, air.
## All parameters come from a JSON file identified from the quad's blackbox logs; see the keys
## read in _init() for units. A Python reference implementation of the same equations exists and
## is checked against this file sample by sample in clean air (not part of the mod package).
##
## The air: the plant has the prop's own axial inflow (climbing unloads the props, descending loads
## them) and a weak calm-air turbulence. Prop wash - the wake the quad leaves behind, its gusts, the
## descent curve and the vortex ring state - is dirty_air's: bf_model.apply_air() puts its air at each
## prop into has_air / air_* below each physics tick. Without dirty_air the quad flies in clean air.
##
## Battery: the motors see v_batt = src_v - src_rp * current_proxy - src_ra * amps (model volts). By
## default that is the identified sag of a fresh pack (battery_v0, battery_r times the identification's
## current proxy); bf_model sets src_v / src_ra from the shared pack (battery_sag) each tick, src_rp
## then 0. The electrical current is worked out from the rotors' mechanical power over the drive's
## efficiency: amps = sum(kQ*load*w^3 + J*w*dw/dt, each >= 0) / (drive_efficiency * pack volts). (Not
## from the motor model's terms: its fitted offset c makes their electrical power fall below the
## mechanical one on some crafts.)
##
## Frames: body FLU (x forward, y left, z up) = the Betaflight gyro frame. World = Godot (y up).
##   FLU -> Godot local: (x, y, z) -> (-y, z, -x);  Godot local -> FLU: (X, Y, Z) -> (-Z, -X, Y).
## Motor order as in Betaflight: 0 rear-right, 1 front-right, 2 rear-left, 3 front-left.

const RHO := 1.225                   # air density [kg/m^3]
const G := 9.81
const PROP_LOAD_MIN := 0.15          # prop load factor limits (inflow from above / below)
const PROP_LOAD_MAX := 1.6

# --- identified / configured parameters (see _init)
var mass: float
var kv: float                        # motor Kv [rad/s per V]
var ma := PackedFloat64Array()       # motor model per motor: dw/dt = a*(u*V - w/Kv) - b*load*w^2 - c
var mb := PackedFloat64Array()
var mc := PackedFloat64Array()
var kT: float                        # thrust coefficient [N/(rad/s)^2]
var arm: float                       # motor x/y distance from the CG [m]
var inertia := Vector3()             # FLU (Ixx, Iyy, Izz) [kg m^2]
var kQ: float                        # prop drag torque coefficient [N m/(rad/s)^2]
var j_rotor: float                   # rotor (prop + bell) inertia [kg m^2]
var yaw_spin := PackedFloat64Array() # rotation sense of each rotor about +z
var pitch_m_per_rad: float           # prop pitch [m per radian]
var cd_area := Vector3()             # body Cd*A along FLU axes [m^2]
var rotor_drag: float                # in-plane rotor drag [N per (m/s * rad/s)] (adjustable)
var torque_bias := Vector3()         # constant torque imbalance per N of total thrust (FLU) [m]
var rate_damping := Vector3()        # aerodynamic rate damping: d(omega)/dt -= k * sqrt(sum w^2)/1000 * omega [1/(s*krad/s)]
var batt_v0: float                   # identified sag of a fresh pack: V = v0 - r*current_proxy
var batt_r: float
var drive_efficiency: float          # motors + ESCs, mechanical power over electrical
var prop_radius: float
var turb_base_ms: float              # calm-air turbulence [m/s]
var turb_hz: float                   # centre frequency of the turbulence [Hz]

# --- runtime switches
var turbulence_enabled := true
## Per-rotor thrust factor from outside the model (ground and ceiling effect of the host); 1.0 = none.
var thrust_scale := PackedFloat64Array([1.0, 1.0, 1.0, 1.0])
## Per-rotor thrust factor of the props' condition (prop_damage: worn props, 0 = broken off), times the above; 1.0 = new props.
var prop_scale := PackedFloat64Array([1.0, 1.0, 1.0, 1.0])

# --- battery source (bf_model sets it from the shared pack each tick; reset() = the identified sag)
var src_v: float                     # model voltage behind the pack's resistance [V]
var src_rp: float                    # the identified sag [model volts per current-proxy unit]
var src_ra: float = 0.0              # the pack's resistance [model volts per amp]
var volts_per_pack_volt: float = 1.0 # model volts per pack volt (bf_model: battery_v0 is a full pack)

# --- the outside air (dirty_air, through bf_model.apply_air; held for a physics tick)
var has_air := false                 # false: clean air (the prop's own inflow both ways, no prop wash)
var air_vel := Vector3()             # air velocity at the quad (world) [m/s]: the old wake there
var wind := Vector3()                # the game's wind (world) [m/s]: moving air, on every path (with and without dirty_air); kept by clear_air()
var air_mean: float = 1.0            # mean thrust ratio descending (descent curve, vortex ring state)
var air_axial := PackedFloat64Array([0, 0, 0, 0])   # axial air speed at each prop beyond the centre's (wake flow, gusts) [m/s], + = up through the prop
var air_fluct := PackedFloat64Array([0, 0, 0, 0])   # each prop's relative thrust fluctuation (vortex ring state)

# --- state
var w := PackedFloat64Array([0, 0, 0, 0])       # rotor speeds [rad/s]
var wdot := PackedFloat64Array([0, 0, 0, 0])
var omega := Vector3()                          # body rates [rad/s], FLU
var thrust := PackedFloat64Array([0, 0, 0, 0])
var v_batt: float = 0.0
var current_proxy: float = 0.0
var amps: float = 0.0                # battery current of the motors [A]
var charge_as: float = 0.0           # amp-seconds, volt-seconds (v_batt) and seconds since bf_model last took them
var charge_vs: float = 0.0
var charge_t: float = 0.0
var turb_x1 := PackedFloat64Array([0, 0, 0, 0])
var turb_x2 := PackedFloat64Array([0, 0, 0, 0])
var rng := RandomNumberGenerator.new()
# the turbulence's constants (a resonator at turb_hz, damping ratio 0.5), and its noise drive for the dt of the last step
var _turb_w0: float
var _turb_w0sq: float
var _turb_c: float
var _turb_dt: float = -1.0
var _turb_drive: float = 0.0


func _init(p: Dictionary) -> void:
	mass = float(p.mass)
	kv = float(p.kv_rad)
	for i in 4:
		ma.append(float(p.motor_a[i]))
		mb.append(float(p.motor_b[i]))
		mc.append(float(p.motor_c[i]))
		yaw_spin.append(float(p.yaw_spin[i]))
	kT = float(p.kT)
	arm = float(p.arm)
	inertia = Vector3(float(p.Ixx), float(p.Iyy), float(p.Izz))
	kQ = float(p.kQ)
	j_rotor = float(p.J_rotor)
	drive_efficiency = clampf(float(p.get("drive_efficiency", 0.78)), 0.3, 1.0)
	pitch_m_per_rad = float(p.pitch_m_per_rad)
	cd_area = _vec3(p.cd_area)
	rotor_drag = float(p.rotor_drag)
	torque_bias = _vec3(p.torque_bias_per_thrust)
	rate_damping = _vec3(p.get("rate_damping", [0.0, 0.0, 0.0]))
	batt_v0 = float(p.battery_v0)
	batt_r = float(p.battery_r)
	prop_radius = float(p.prop_radius)
	turb_base_ms = float(p.turb_base_ms)
	turb_hz = float(p.pw_noise_hz)
	_turb_w0 = 2.0 * PI * turb_hz
	_turb_w0sq = _turb_w0 * _turb_w0
	_turb_c = 4.0 * 0.5 * _turb_w0 * _turb_w0 * _turb_w0
	rng.seed = int(p.get("seed", 1))
	reset()


static func _vec3(a: Array) -> Vector3:
	return Vector3(float(a[0]), float(a[1]), float(a[2]))


func reset() -> void:
	w.fill(0.0)
	wdot.fill(0.0)
	thrust.fill(0.0)
	omega = Vector3()
	src_v = batt_v0
	src_rp = batt_r
	src_ra = 0.0
	v_batt = batt_v0
	amps = 0.0
	charge_as = 0.0
	charge_vs = 0.0
	charge_t = 0.0
	turb_x1.fill(0.0)
	turb_x2.fill(0.0)


## Clean air: no air from outside (dirty_air gone or not flying).
## Rotor speed [rad/s] at full throttle on a full pack, no inflow (the motor's steady state at u = 1)
func full_speed() -> float:
	var a := mb[0]
	var b := ma[0] / kv
	var c := mc[0] - ma[0] * batt_v0
	return (-b + sqrt(maxf(b * b - 4.0 * a * c, 0.0))) / (2.0 * a) if a > 0.0 else batt_v0 * kv


func clear_air() -> void:
	has_air = false
	air_vel = Vector3()
	air_mean = 1.0
	air_axial.fill(0.0)
	air_fluct.fill(0.0)


static func flu_to_local(v: Vector3) -> Vector3:
	return Vector3(-v.y, v.z, -v.x)


static func local_to_flu(v: Vector3) -> Vector3:
	return Vector3(-v.z, -v.x, v.y)


## Advance the quad by dt. u = motor commands 0..1 (already delayed), basis = body->world (Godot),
## vel = world velocity. Updates the rotor speeds and the body rates (omega)
## and returns the world linear acceleration including gravity.
##
## (Written for speed - this runs at the rate of the firmware's PID loop, 4-8 kHz: the rotors' loops are
## one, the sums run in the order of the rotors, the turbulence constants are worked out once.)
func step(u: PackedFloat64Array, dt: float, basis: Basis, vel: Vector3) -> Vector3:
	# velocity relative to the air (which moves where the quad flies through its old wake)
	var vl := (vel - air_vel - wind) * basis     # (orthonormal: v * basis = basis.transposed() * v), world -> body
	var vb := Vector3(-vl.z, -vl.x, vl.y)        # local_to_flu()
	var v_ax := vb.z
	# battery: the pack's voltage under the motors' current
	var i_proxy := 0.0
	for i in 4:
		var ui := u[i]
		i_proxy += ui * maxf(ui * v_batt - w[i] / kv, 0.0)
	current_proxy = i_proxy
	v_batt = src_v - src_rp * i_proxy - src_ra * amps      # (amps of the last step)
	# calm-air turbulence: an air velocity fluctuation u' along each prop's axis changes its angle
	# of attack, dT = -kT * w * u' / pitch (relatively strongest at low rpm, as real propwash);
	# dirty_air's wake flow and gusts at each prop act the same way
	var u_amp := turb_base_ms if turbulence_enabled else 0.0
	var turb := u_amp > 0.0
	var drive := 0.0
	if turb:
		if dt != _turb_dt:
			_turb_dt = dt
			_turb_drive = sqrt(_turb_c / dt)    # unit-variance band-limited noise
		drive = _turb_drive
	# ESC/motors and rotor thrust with inflow (the air and the turbulence along each prop's axis, with the updated rotor speed)
	var own_load := v_ax != 0.0 and not (has_air and v_ax < 0.0)     # the prop's own inflow load; descending in dirty_air's air is its (nothing counts twice)
	var vbat := v_batt
	var t_total := 0.0
	var wsum := 0.0
	var wsq := 0.0
	var p_mech := 0.0
	var tz := 0.0
	for i in 4:
		var wo := w[i]
		var ui := u[i]
		var load := clampf(1.0 - v_ax / maxf(wo * pitch_m_per_rad, 1.0), PROP_LOAD_MIN, PROP_LOAD_MAX) if own_load else 1.0
		var wd := ma[i] * (ui * vbat - wo / kv) - mb[i] * load * wo * wo - mc[i]
		if ui <= 0.0:
			wd = minf(wd, 0.0)       # a stopped motor never spins itself up (offset c < 0 on two motors)
		var wn := maxf(wo + wd * dt, 0.0)
		var wdi := (wn - wo) / dt
		wdot[i] = wdi
		w[i] = wn
		var dv := 0.0
		if turb:
			var e := rng.randfn(0.0, 1.0) * drive
			var x2 := turb_x2[i]
			var x1 := turb_x1[i]
			x2 += (e - _turb_w0 * x2 - _turb_w0sq * x1) * dt
			x1 += x2 * dt
			turb_x2[i] = x2
			turb_x1[i] = x1
			dv = u_amp * x1      # the prop's extra axial speed through the air
		var den := maxf(wn * pitch_m_per_rad, 1.0)
		var k := clampf(1.0 - v_ax / den, PROP_LOAD_MIN, PROP_LOAD_MAX) if own_load else 1.0
		p_mech += maxf(kQ * k * wn * wn * wn + j_rotor * wn * wdi, 0.0)
		if has_air:
			k *= air_mean * (1.0 + air_fluct[i])
			dv -= air_axial[i]
		if dv != 0.0:                # the load's slope, -1/(w*pitch) per m/s, whatever the load
			k = maxf(k - dv / den, 0.0)
		var t := kT * wn * wn * k * thrust_scale[i] * prop_scale[i]
		thrust[i] = t
		t_total += t
		wsum += wn
		wsq += wn * wn
		tz -= yaw_spin[i] * (kQ * wn * wn + j_rotor * wdi)
	amps = p_mech * volts_per_pack_volt / (drive_efficiency * maxf(v_batt, 1.0))
	charge_as += amps * dt
	charge_vs += v_batt * dt
	charge_t += dt
	# torques (FLU)
	var tx := arm * (thrust[2] + thrust[3] - thrust[0] - thrust[1])
	var ty := arm * (thrust[0] + thrust[2] - thrust[1] - thrust[3])
	var tau := Vector3(tx, ty, tz) + torque_bias * t_total
	var damp := rate_damping * (sqrt(wsq) * 1e-3)
	omega += ((tau - omega.cross(inertia * omega)) / inertia - damp * omega) * dt
	# forces (FLU body): thrust, quadratic body drag, linear in-plane rotor drag
	var f := Vector3(0.0, 0.0, t_total)
	f -= 0.5 * RHO * vb.length() * cd_area * vb
	var rw := rotor_drag * wsum
	f.x -= rw * vb.x
	f.y -= rw * vb.y
	return basis * Vector3(-f.y, f.z, -f.x) / mass + Vector3(0.0, -G, 0.0)     # (flu_to_local)
