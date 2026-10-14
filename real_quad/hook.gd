extends RefCounted
## real_quad: what the player script (player.gd / player_fc.gd) does for the
## quad the game flies. Those two scripts only hand the game's physics step,
## its sound step and the body's integration over to this.
##
## While a model flies the quad (the selected drone has one, the game is
## not paused, no turtle mode) the body runs on a custom integrator: the
## model (flightmodel/bf_model.gd) steps the Betaflight rate loop and the quad's
## motors and props at the firmware's own rate and hands the mean velocities of
## each tick back to the physics engine, which only moves the body and
## resolves its contacts. Jolt ignores forces applied to such a body, so the
## game's own thrust, drag and torque (and those of mods that push the
## quad with forces) have no effect on it - nothing is counted twice.
## Otherwise the game's own physics fly the quad, as without this mod.
##
## The game's own physics step still runs every tick: inputs, arming, OSD,
## respawn, multiplayer packets and stats are the game's.
##
## Each tick, before the model integrates, the hook hands it the props' condition
## (prop_damage: each prop's thrust factor, and its out-of-balance as vibration in the gyro
## the flight controller reads), the air and the battery: dirty_air's air at each prop (its wake, gusts, descent and ring
## state, floor and ceiling; clean air without dirty_air), and the shared
## pack - battery_sag's, driven with the model's motor current, or without
## battery_sag the identified sag of a fresh pack (shown through
## real_quad's zm_battery()).

const ApiGuard := preload("api_guard.gd")
const TRACK_HANDLER_PATH := ^"/root/IngameMain/TrackHandler"
# On the body while a model flies it: the four rotor speeds (shaft rpm, PackedFloat32Array, Betaflight's
# motor order: rear right, front right, rear left, front left), for pilot_audio's motor voices
const RPM_META := "motor_rpm"

var host: RigidBody3D
var mod: Node                       # the real_quad mod (main.gd)
var model: RefCounted = null        # BFModel flying this body right now (null while inactive)
var active := false
var _gs = null                      # the game's ZGamestate autoload
var _orig_body := {}                # body properties restored when switching back to the game's physics
var _gen := -1                      # mod.generation the model was last given its settings for
var _vid := ""                      # vehicle the settings were looked up for
var _bat_key := ""                  # battery_sag's pack this model drives ("vehicle model", "" = none)


static func find_mod(tree: SceneTree) -> Node:
	return tree.root.get_node_or_null("ZoneMods/real_quad") if tree != null else null


func _init(p_host: RigidBody3D, p_mod: Node) -> void:
	host = p_host
	mod = p_mod


func ready() -> void:
	_gs = host.get_node_or_null("/root/ZGamestate")
	if host.has_signal("teleported"):
		host.connect("teleported", func() -> void:
			if model != null:
				model.reset()
				mod.own_pack_reset())


## Before the game's physics step
func before_physics() -> void:
	_guard_race_upload()
	var vid := _vehicle_id()
	var want := _wanted_model_id(vid)
	_set_model(want)
	if model != null and (mod.generation != _gen or vid != _vid):
		_apply_settings(vid)
		_gen = mod.generation
		_vid = vid
	if model != null:
		model.apply_air(mod.air_node(), mod.ground_ceiling_effect)
		_apply_props()
		_apply_battery(vid)


## After the game's physics step
func after_physics() -> void:
	if active:
		host.custom_integrator = true      # the original may have reloaded its settings this tick
		var w: PackedFloat64Array = model.quad.w
		var k := 60.0 / TAU
		host.set_meta(RPM_META, PackedFloat32Array([w[0] * k, w[1] * k, w[2] * k, w[3] * k]))
	elif host.has_meta(RPM_META):
		host.remove_meta(RPM_META)


## Motor sound: the game derives pitch and volume (and the motor speed sent to other players)
## from current_motor_rpm, so feed it the mean of the four modelled rotors.
func before_sound() -> void:
	if active:
		var w: PackedFloat64Array = model.quad.w
		host.set("current_motor_rpm", (w[0] + w[1] + w[2] + w[3]) * 0.25 * 60.0 / TAU)


func integrate_forces(state: PhysicsDirectBodyState3D) -> void:
	if not active or not host.custom_integrator:
		return
	var sp: Vector4 = host.get("stick_pos")     # (roll, pitch, throttle, yaw), after the game's setpoint latency
	var rc := {"roll": sp.x, "pitch": sp.y, "yaw": sp.w, "throttle": sp.z}
	model.integrate_forces(host, state, rc, bool(host.call("is_armed")))


func _vehicle_id() -> String:
	return str(_gs.selected_vehicle_id) if _gs != null else ""


## The model package that should fly the drone right now, "" = the game's physics.
func _wanted_model_id(vid: String) -> String:
	if not mod.on or _gs == null or _gs.is_paused or host.get("turtle_mode_active") == true:
		return ""
	return mod.model_id_for(vid)


## Switches to the model with this id ("" = back to the game's physics). Loads and caches the
## package and applies its mass and inertia.
func _set_model(id: String) -> void:
	var cur := str(model.get_id()) if model != null else ""
	if id == cur:
		return
	var m = mod.registry.load_model(id) if id != "" else null
	if m == null:
		if active:
			_deactivate()
		return
	if not active:                 # a direct model-to-model switch keeps the saved original body
		_orig_body = {
			"inertia": host.inertia, "can_sleep": host.can_sleep,
			"linear_damp_mode": host.linear_damp_mode, "angular_damp_mode": host.angular_damp_mode,
			"linear_damp": host.linear_damp, "angular_damp": host.angular_damp,
		}
	model = m
	_gen = -1
	model.reset()                  # resyncs from the body's current velocities
	model.configure_body(host)     # custom integrator, mass/inertia of the modelled quad
	active = true
	mod.flying(_vehicle_id(), model)


func _deactivate() -> void:
	active = false
	model = null
	_release_battery()
	mod.own_pack.clear()
	host.custom_integrator = false
	for key: String in _orig_body:
		host.set(key, _orig_body[key])
	host.set("_SETTINGS", null)        # the game re-applies its mass and shapes next tick


## The vehicle's own settings (propwash, drag) for the model
func _apply_settings(vid: String) -> void:
	var s: Dictionary = mod.drone_cfg(vid)
	model.set_propwash_scale(float(s["propwash"]))
	var d := float(s["drag"])
	model.set_linear_drag(d if d > 0.0 else model.get_package_linear_drag())


## prop_damage's props: the model gets each prop's thrust factor and out-of-balance (set_model() of
## prop_damage's Props: it hands over how hard each rotor turns, for the wear of a hit, and does not
## push or shake the body itself any more); undamaged props without prop_damage.
func _apply_props() -> void:
	var pd: Node = mod.props_node()
	if pd == null or not pd.has_method("set_model"):
		model.clear_props()
		return
	var st: Dictionary = pd.call("set_model", Array(model.spin_share()))
	model.set_props(st["thrust"], st["imbalance"])


## The shared pack for the coming tick: battery_sag's (driven with the model's mean motor current of
## the last tick; its voltage behind the resistance and the resistance go to the model), or without
## it the model's identified sag, reported by real_quad's zm_battery().
func _apply_battery(vid: String) -> void:
	var c: Vector3 = model.take_charge()
	var amps := c.x / c.y if c.y > 0.0 else 0.0
	var bat: Node = mod.battery_node()
	if bat == null or not bat.has_method("drive"):
		_bat_key = ""
		model.use_identified_sag()
		mod.own_pack_step(model, amps, c.y, c.z)
		return
	var key := vid + " " + str(model.get_id())
	if key != _bat_key or not bool(bat.get("external")):
		bat.call("external_pack", vid, mod.pack_spec(model))
		_bat_key = key
	if c.y > 0.0:
		bat.call("drive", amps, c.y)
	model.set_pack(float(bat.call("source_volts")), float(bat.call("resistance", float(bat.call("soc")))), float(bat.call("ocv", 1.0)))


func _release_battery() -> void:
	if _bat_key == "":
		return
	_bat_key = ""
	var bat: Node = mod.battery_node()
	if bat != null and bat.has_method("external_release"):
		bat.call("external_release")


## This mod flies different physics, so race times must never be posted. The race handler's API
## object is replaced by a variant whose post_race_run() is a no-op (checked every tick because
## the game recreates the handler's state on map changes; the check itself is a type test).
func _guard_race_upload() -> void:
	if not mod.on:
		return
	var handler := host.get_node_or_null(TRACK_HANDLER_PATH)
	if handler == null or not ("api" in handler):
		return
	var api = handler.get("api")
	if api is ApiGuard:
		return
	var safe = ApiGuard.new()
	safe.request_dump = api.request_dump if api != null else handler
	handler.api = safe
	print("[real_quad] race uploads disabled")
