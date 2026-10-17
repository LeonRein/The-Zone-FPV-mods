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
## respawn, multiplayer packets and stats are the game's. Of the game's settings the
## model takes: the Rates menu (rates=game: ZSettings.RATES go into the Betaflight rates, live), the
## Physics menu's surface_area_*_multi (scale the body drag), the wind (ZGamestate.current_wind_effect,
## moving air round the quad), the camera settings and the window focus (an unfocused game has no thrust).
## The weight, thrust and top speed of the Physics menu are the model's, by design.
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
# The game's own top of the motor scale (22.2 V * 1400 KV, player_rigid_body.gd MAX_MOTOR_RPM): its sound's pitch and
# volume and the motor speed other players get are current_motor_rpm / this
const GAME_MAX_RPM := 31080.0
# What the model changes on the body (BFModel.configure_body()), saved when it takes the quad and put back
const BODY_PROPERTIES := ["inertia", "can_sleep", "linear_damp_mode", "angular_damp_mode", "linear_damp", "angular_damp",
	"center_of_mass_mode", "center_of_mass", "gravity_scale"]

var host: RigidBody3D
var mod: Node                       # the real_quad mod (main.gd)
var model: RefCounted = null        # BFModel flying this body right now (null while inactive)
var active := false
var _gs = null                      # the game's ZGamestate autoload
var _zs = null                      # the game's ZSettings autoload (RATES)
var focus_override := 0             # tests: 1 = the game's window has focus, -1 = it has not, 0 = ask the window
var _rates_mode := "game"           # the drone's rates setting
var _rates_key := []                # the game's rates (type, roll, pitch, yaw) last handed to the model
var _drag_key := Vector3(-1, -1, -1)   # the game's drag multipliers last handed to the model (-1: none yet)
var _orig_body := {}                # body properties restored when switching back to the game's physics (BODY_PROPERTIES)
var _gen := -1                      # mod.generation the model was last given its settings for
var _vid := ""                      # vehicle the settings were looked up for
var _bat_key := ""                  # battery_sag's pack this model drives ("vehicle model", "" = none)
var _key_vid := ""                  # what _bat_key's text is made of ...
var _key_model: RefCounted = null
var _key_text := ""                 # ... the text made of them
var _own_key := ""                  # vehicle and model object the real_quad pack (without battery_sag) is for
var _osd_rate_set := false          # curr_max_rate on the OSD is a value written here
# The other mods' nodes, looked for again only when the one found is gone (all of them every tick otherwise)
var _air_n: Node = null
var _props_n: Node = null
var _props_ok := false              # prop_damage's Props has set_model()
var _bat_n: Node = null
var _bat_ok := false                # battery_sag's Battery has drive()
var _handler: Node = null           # the race handler
var _handler_api := false           # ... and it has an api
var _kept: RefCounted = null        # the model put aside for a pause or a turtle flip (its rotors and controller go on after)
var _spin := [0.0, 0.0, 0.0, 0.0]   # the rotors' spin handed to prop_damage


## (Without the mod node, mod stays null and the hook does nothing: the game's own quad.)
func _init(p_host: RigidBody3D) -> void:
	host = p_host


func ready() -> void:
	mod = host.get_tree().root.get_node_or_null("ZoneMods/real_quad")
	_gs = host.get_node_or_null("/root/ZGamestate")
	_zs = host.get_node_or_null("/root/ZSettings")
	if host.has_signal("teleported"):
		host.connect("teleported", func() -> void:
			var m = model if model != null else _kept
			if m != null:
				m.reset()
				# a respawn gives a fresh pack only while disarmed (battery_sag's new_pack_on_respawn default);
				# battery_sag has its own rules for its pack
				if not bool(host.call("is_armed")):
					mod.own_pack_reset())


## Before the game's physics step
func before_physics() -> void:
	if mod == null:
		return
	var vid := _vehicle_id()
	var want := _wanted_model_id(vid)
	_set_model(want, _put_aside())
	_guard_race_upload()
	if model != null and (mod.generation != _gen or vid != _vid):
		_apply_settings(vid)
		_gen = mod.generation
		_vid = vid
	if model != null:
		_apply_game_settings()
		model.apply_air(_air_node(), bool(mod.cfg["ground_ceiling_effect"]))
		_apply_props()
		_apply_battery(vid)


## After the game's physics step
func after_physics() -> void:
	if active:
		host.custom_integrator = true      # the original may have reloaded its settings this tick
		model.apply_mass(host)             # ... and then set the mass of its Physics menu again
		_max_rate_osd()
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
		# the game's scale runs to GAME_MAX_RPM at full throttle, the model's rotors turn faster: pitch and volume span as the game's
		var mean := (w[0] + w[1] + w[2] + w[3]) * 0.25 * 60.0 / TAU
		host.set("current_motor_rpm", mean * GAME_MAX_RPM / maxf(model.full_rpm(), 1.0))


func integrate_forces(state: PhysicsDirectBodyState3D) -> void:
	if not active or not host.custom_integrator:
		return
	var sp: Vector4 = host.get("stick_pos")     # (roll, pitch, throttle, yaw), after the game's setpoint latency
	var rc := {"roll": sp.x, "pitch": sp.y, "yaw": sp.w, "throttle": sp.z}
	# the game gives no thrust while its window has no focus: the motors stop as when disarmed
	model.integrate_forces(host, state, rc, bool(host.call("is_armed")) and _focused())


func _vehicle_id() -> String:
	return str(_gs.selected_vehicle_id) if _gs != null else ""


## The model package that should fly the drone right now, "" = the game's physics.
func _wanted_model_id(vid: String) -> String:
	if not mod.on or _gs == null or _put_aside() or host.get("physics_handler") != null:    # (another handler: the wing)
		return ""
	return mod.model_id_for(vid)


## The game's physics fly for a moment (the game is paused, the quad is on its back in turtle mode): the
## same model goes on after, with the same pack, rotor speeds and controller
func _put_aside() -> bool:
	return _gs != null and (_gs.is_paused or host.get("turtle_mode_active") == true)


## Switches to the model with this id ("" = back to the game's physics). Loads and caches the
## package and applies its mass and inertia. A package rebuilt under the same id is another object: it is
## switched to as well, in mid-flight. aside: the game's physics fly only for a moment (pause, turtle mode),
## so the pack stays.
func _set_model(id: String, aside := false) -> void:
	var m = mod.registry.load_model(id) if id != "" else null
	if m != null and m == model:
		return
	if m == null:
		if active:
			_deactivate(aside)
		return
	if not active:                 # a direct model-to-model switch keeps the saved original body
		_orig_body = {}
		for key: String in BODY_PROPERTIES:
			_orig_body[key] = host.get(key)
	model = m
	_gen = -1
	if m == _kept:
		m.resync()                 # put aside for a moment: the rotors and the controller go on, the velocities are the body's
	else:
		m.reset()                  # from rest, resynced from the body's current velocities
	_kept = null
	model.configure_body(host)     # custom integrator, mass/inertia of the modelled quad
	active = true
	mod.flying(_vehicle_id(), model)


func _deactivate(aside := false) -> void:
	active = false
	_kept = model if aside else null
	model = null
	_release_battery()
	if not aside:
		mod.own_pack.clear()
		_own_key = ""
	host.custom_integrator = false
	for key: String in _orig_body:
		host.set(key, _orig_body[key])
	host.set("_SETTINGS", null)        # the game re-applies its mass and shapes next tick


func _focused() -> bool:
	if focus_override != 0:
		return focus_override > 0
	var vp := host.get_viewport()
	var w := vp.get_window() if vp != null else null
	return w == null or w.has_focus()


## The game's own settings the model follows each tick: its drag multipliers (Physics menu), the wind and, with
## rates=game, the Rates menu's rates (taken again when they change).
func _apply_game_settings() -> void:
	# drag multipliers: the vehicle's adjustable_settings (the game's own copy; null for a tick after a reload)
	var s = host.get("_SETTINGS")
	var adj = null
	if s is Dictionary or s is Object:
		adj = s.get("adjustable_settings")
	if adj is Dictionary:
		var m := Vector3(float(adj.get("surface_area_x_multi", 100.0)), float(adj.get("surface_area_y_multi", 100.0)), float(adj.get("surface_area_z_multi", 100.0)))
		if m != _drag_key:
			_drag_key = m
			model.set_drag_multipliers(m.x, m.y, m.z)
	elif _drag_key.x < 0.0:
		_drag_key = Vector3(100, 100, 100)
		model.set_drag_multipliers(100.0, 100.0, 100.0)
	# wind: relative air velocity (the game's gusts and shelter are in it)
	var wind = _gs.get("current_wind_effect") if _gs != null else null
	model.set_wind(wind if wind is Vector3 else Vector3.ZERO)
	# rates
	if _rates_mode == "game" and _zs != null:
		var r = _zs.get("RATES")
		if r is Dictionary:
			var key := [r.get("type"), r.get("roll"), r.get("pitch"), r.get("yaw")]
			if key != _rates_key:
				_rates_key = key
				model.set_game_rates(r)


## The game's max-rate OSD readout (curr_max_rate, every 60 ticks, when its option is on) from the rates in use.
## With the option off the value written earlier goes (the game, or flight_controller's step it replaced, no longer
## updates it).
func _max_rate_osd() -> void:
	if Engine.get_physics_frames() % 60 != 0 or _gs == null:
		return
	if _gs.get("enable_current_max_rate_osd") != true:
		if _osd_rate_set:
			_osd_rate_set = false
			_gs.OSD_VALUES.erase("curr_max_rate")
		return
	var sp: Vector4 = host.get("stick_pos")
	_gs.OSD_VALUES["curr_max_rate"] = "%.0f" % model.current_max_rate_dps({"roll": sp.x, "pitch": sp.y, "yaw": sp.w})
	_osd_rate_set = true


## The vehicle's own settings (propwash, drag, rates, tune) for the model
func _apply_settings(vid: String) -> void:
	var s: Dictionary = mod.drone_cfg(vid)
	mod.tune_notes[vid] = model.use_tune(str(s.get("tune", "flown")))
	model.set_propwash_scale(float(s["propwash"]))
	var d := float(s["drag"])
	model.set_linear_drag(d if d > 0.0 else model.get_package_linear_drag())
	_rates_mode = str(s.get("rates", "game"))
	_rates_key = []
	_drag_key = Vector3(-1, -1, -1)
	if _rates_mode != "game":
		model.use_package_rates()


## prop_damage's props: the model gets each prop's thrust factor and out-of-balance (set_model() of
## prop_damage's Props: it hands over how hard each rotor turns, for the wear of a hit, and does not
## push or shake the body itself any more); undamaged props without prop_damage.
func _apply_props() -> void:
	var pd := _props_node()
	if pd == null or not _props_ok:
		model.clear_props()
		return
	model.spin_share_into(_spin)
	var st: Dictionary = pd.call("set_model", _spin)
	model.set_props(st["thrust"], st["imbalance"])


## The shared pack for the coming tick: battery_sag's (driven with the model's mean motor current of
## the last tick; its voltage behind the resistance and the resistance go to the model), or without
## it the model's identified sag, reported by real_quad's zm_battery().
func _apply_battery(vid: String) -> void:
	var c: Vector3 = model.take_charge()
	var amps := c.x / c.y if c.y > 0.0 else 0.0
	var bat := _battery_node()
	if bat == null or not _bat_ok:
		_bat_key = ""
		# a new pack when the vehicle or the model changes (not for a pause, a turtle flip or a respawn in flight)
		var own_key := "%s %d" % [vid, model.get_instance_id()]
		if own_key != _own_key:
			_own_key = own_key
			mod.own_pack_reset()
		model.use_identified_sag()
		mod.own_pack_step(model, amps, c.y, c.z)
		return
	if vid != _key_vid or model != _key_model:
		_key_vid = vid
		_key_model = model
		_key_text = vid + " " + str(model.get_id())
	if _key_text != _bat_key or not bool(bat.get("external")):
		bat.call("external_pack", vid, mod.pack_spec(model))
		_bat_key = _key_text
	if c.y > 0.0:
		bat.call("drive", amps, c.y)
	model.set_pack(float(bat.call("source_volts")), float(bat.call("resistance", float(bat.call("soc")))), float(bat.call("ocv", 1.0)))


# The other mods' nodes (mod.air_node() ...), kept while they are in the tree
func _air_node() -> Node:
	if not is_instance_valid(_air_n) or not _air_n.is_inside_tree():
		_air_n = mod.air_node()
	return _air_n


func _props_node() -> Node:
	if not is_instance_valid(_props_n) or not _props_n.is_inside_tree():
		_props_n = mod.props_node()
		_props_ok = _props_n != null and _props_n.has_method("set_model")
	return _props_n


func _battery_node() -> Node:
	if not is_instance_valid(_bat_n) or not _bat_n.is_inside_tree():
		_bat_n = mod.battery_node()
		_bat_ok = _bat_n != null and _bat_n.has_method("drive")
	return _bat_n


func _release_battery() -> void:
	if _bat_key == "":
		return
	_bat_key = ""
	var bat: Node = mod.battery_node()
	if bat != null and bat.has_method("external_release"):
		bat.call("external_release")


## A model flies different physics, so race times must never be posted while it does (or is put aside for a
## pause or a turtle flip). The race handler's API object is then replaced by a variant whose post_race_run()
## is a no-op, and put back when the game's own physics fly (checked every tick because the game recreates the
## handler's state on map changes; the check itself is a type test).
func _guard_race_upload() -> void:
	if not mod.on:
		return
	if not is_instance_valid(_handler) or not _handler.is_inside_tree():
		_handler = host.get_node_or_null(TRACK_HANDLER_PATH)
		_handler_api = _handler != null and "api" in _handler
	var handler := _handler
	if not _handler_api:
		return
	var api = handler.get("api")
	if not (active or _kept != null):
		if api is ApiGuard and is_instance_valid(api.original):
			handler.api = api.original
			print("[real_quad] race uploads as the game's")
		return
	if api is ApiGuard:
		return
	var safe = ApiGuard.new()
	safe.request_dump = api.request_dump if api != null else handler
	safe.original = api
	handler.api = safe
	print("[real_quad] race uploads disabled")
