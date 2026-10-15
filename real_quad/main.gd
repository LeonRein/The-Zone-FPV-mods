extends Node
## real_quad 1.0
##
## Flight models of real quads for The Zone FPV: a Betaflight rate loop (ported
## from the firmware) flying a quad model identified from a real quad's
## blackbox logs - motors, props, battery sag, inertia, drag - in place of the
## game's idealised physics, for the drones that have a model. Which model
## flies which drone is set in settings.cfg, a section per drone. Off until
## turned on (enabled=true). The air (prop wash, floor and ceiling) is
## dirty_air's and the pack battery_sag's, each put into the model; without
## them the model flies in clean air on a pack that sags but does not drain.
## Other mods (fpv_osd) read the pack with zm_battery().
##
## How: the game's quad is player_rigid_body.gd. At every launch this copies it
## out of the game (into real_quad/base) and mounts a small resource pack
## (real_quad/remap.pck) that points the game at player.gd instead - the game's
## own script with the model's physics added through hook.gd. If the game's
## script is not what this expects, nothing is mounted and the game runs as
## without this mod. With flight_controller in effect, player_fc.gd is used
## instead: it extends flight_controller's fc.gd, so flight_controller flies the
## drones without a model here.
##
## Uninstall: delete the real_quad folder.

const VERSION := "1.0"
const DIR := "res://real_quad/"
const GAME := "res://player_rigid_body.gdc"
const COPY := "base/player_rigid_body.gdc"
# The game's player script this was made and tried with
const KNOWN_MD5 := ["b27ff67e1823e52bc519732313f06083"]
# What hook.gd and the player scripts use of the game's script
const NEED_METHODS := ["_physics_process", "play_sound", "is_armed", "calc_motor_rpm", "_handle_rotation", "get_thrust_at_rpm"]
const NEED_PROPERTIES := ["stick_pos", "turtle_mode_active", "current_motor_rpm", "_SETTINGS"]
const ModelRegistry := preload("flightmodel/model_registry.gd")

const DEFAULTS := {
	"ground_ceiling_effect": true,
	"battery_capacity_mah": 1300.0,     # for a model package that does not say
	"unknown_game_version": false,
}
# What each drone's section sets (see settings.default.cfg)
const DRONE_DEFAULTS := {"model": "auto", "propwash": 1.0, "drag": 0.0, "rates": "game"}

var cfg := DEFAULTS.duplicate()
var registry: ModelRegistry = null
var on := false                     # the player script is replaced: drones with a model are flown by it
var generation := 0                 # counts settings.cfg reads, for the hook
var own_pack := {}                  # the pack a model flies on without battery_sag (zm_battery)
var drones := {}                    # vehicle id -> {model, propwash, drag, rates}
var settings_path := DIR + "settings.cfg"
var _core: Node = null
var _copied := false
var _why_not := ""
var _log := PackedStringArray()
var _said := {}
var model_roots := PackedStringArray()   # folders of model packages (a package each: craft.json, fc.json, plant.json)


func zm_init(core: Node, _dir: String) -> void:
	_core = core
	_say("real_quad %s" % VERSION)
	# listed from the real folder: the game's own file list does not know a folder next to its exe
	model_roots = PackedStringArray([_game_dir().path_join("models")])
	_scan_models()
	read_settings()
	_say("ground and ceiling effect %s, %d mAh for a model whose package gives no pack size" % [
		"on" if bool(cfg["ground_ceiling_effect"]) else "off", int(cfg["battery_capacity_mah"])])
	var why := _copy()
	if why != "":
		_why_not = "player_rigid_body.gd - " + why
		_say("not in effect: " + _why_not + ". The game runs as without this mod; an update of it is needed")
	else:
		_copied = true
	_write_status()


## Reads settings.cfg (settings.default.cfg when there is none): [real_quad] and a
## section per drone with model, propwash, drag and rates
func read_settings(path := settings_path) -> void:
	settings_path = path
	var c := ConfigFile.new()
	if c.load(path) != OK and c.load(path.get_base_dir().path_join("settings.default.cfg")) != OK:
		return
	cfg = DEFAULTS.duplicate()
	for k in DEFAULTS:
		if c.has_section_key("real_quad", k):
			var v = c.get_value("real_quad", k)
			cfg[k] = bool(v) if DEFAULTS[k] is bool else float(v)
	cfg["battery_capacity_mah"] = clampf(float(cfg["battery_capacity_mah"]), 50.0, 100000.0)
	drones = {}
	var overrides := ConfigFile.new()
	for sec in c.get_sections():
		if sec == "mod" or sec == "real_quad":
			continue
		var d: Dictionary = DRONE_DEFAULTS.duplicate()
		for k in d:
			if c.has_section_key(sec, k):
				d[k] = c.get_value(sec, k)
		var model := str(d["model"]).strip_edges()
		drones[sec] = {
			"model": model,
			"propwash": clampf(float(d["propwash"]), 0.0, 5.0),
			"drag": clampf(float(d["drag"]), 0.0, 3.0),
			"rates": "model" if str(d["rates"]).strip_edges() == "model" else "game",
		}
		if model == "off":
			overrides.set_value("vehicle_models", sec, ModelRegistry.NONE)
		elif model != "auto" and model != "":
			overrides.set_value("vehicle_models", sec, model)
	if registry != null:
		registry.load_overrides(overrides)
	generation += 1


## Called by mod_settings when settings.cfg has been changed in the game: read again;
## the hook takes the new figures on its next tick
func zm_apply_settings() -> void:
	read_settings()
	_said.clear()
	_write_status()


func zm_restart_keys() -> PackedStringArray:
	return PackedStringArray(["real_quad/unknown_game_version"])


## Shown at the top of the mod's page in mod_settings: the model that flies the
## drone flown right now (so "auto" shows what it picked), or why the settings
## do nothing for now. mod_settings reads it again after each change
func zm_settings_status() -> String:
	if not on:
		return "not in effect: " + (_why_not if _why_not != "" else "the game has not started it yet")
	var gs := get_node_or_null("/root/ZGamestate")
	var vid := str(gs.get("selected_vehicle_id")) if gs != null and gs.get("selected_vehicle_id") != null else ""
	if vid == "":
		return "Flying now: no drone yet."
	var mid := model_id_for(vid)
	var how := str(drone_cfg(vid)["model"])
	return "Flying now: %s%s. A model's weight, thrust and top speed come from the model, not from the game's Physics menu." % [
		mid if mid != "" else "the game's physics", " (%s)" % how if how == "auto" or how == "off" else ""]


func _scan_models() -> void:
	registry = ModelRegistry.new(model_roots)
	registry.scan()
	var names := PackedStringArray()
	for e in registry.list_models():
		var m = registry.load_model(e.id)
		names.append("%s for %s (%d Hz loop%s)" % [e.id, e.zone_vehicle_id if e.zone_vehicle_id != "" else "no drone",
			int(round(m.get_loop_hz())) if m != null else 0, "" if m != null else ", NOT LOADED"])
	_say("models: " + (", ".join(names) if not names.is_empty() else "none"))
	for err in registry.errors:
		_say("model package skipped: " + err)


## The model package that should fly this drone, "" = the game's physics
func model_id_for(vehicle_id: String) -> String:
	return registry.resolve(vehicle_id)


## The drone's own settings: propwash (1 = as calibrated), drag (0 = the package's own) and rates
## ("game" = the game's Rates menu, "model" = the package's)
func drone_cfg(vehicle_id: String) -> Dictionary:
	return drones.get(vehicle_id, DRONE_DEFAULTS)


## dirty_air's air node (the air at the props: prop wash, floor and ceiling), or null
func air_node() -> Node:
	var da = _core.call("get_mod", "dirty_air") if _core != null and _core.has_method("get_mod") else null
	return da.get_node_or_null("Air") if da != null and is_instance_valid(da) else null


## prop_damage's Props node (the props' wear and out-of-balance), or null
func props_node() -> Node:
	var pd = _core.call("get_mod", "prop_damage") if _core != null and _core.has_method("get_mod") else null
	return pd.get_node_or_null("Props") if pd != null and is_instance_valid(pd) else null


## battery_sag's pack node, or null
func battery_node() -> Node:
	var b = _core.call("get_mod", "battery_sag") if _core != null and _core.has_method("get_mod") else null
	return b.get_node_or_null("Battery") if b != null and is_instance_valid(b) else null


## The battery a model's craft flies on (its package's craft.json "battery"), the capacity from
## battery_capacity_mah when the package has none
func pack_spec(model: RefCounted) -> Dictionary:
	var s: Dictionary = model.battery_spec()
	if float(s["capacity_mah"]) <= 0.0:
		s["capacity_mah"] = float(cfg["battery_capacity_mah"])
	return s


## The pack for an OSD (fpv_osd), as battery_sag gives it: battery_sag's own when it is loaded
## (driven by the model where one flies), otherwise the pack a model flies on, or {}
func zm_battery() -> Dictionary:
	var bat := battery_node()
	if bat != null and bat.has_method("report"):
		return bat.call("report")
	return own_pack


## Without battery_sag: one tick of the pack a model flies on - the identified sag of a fresh pack,
## not draining; the mAh used are counted all the same
func own_pack_step(model: RefCounted, amps: float, dt: float, cell_volts: float) -> void:
	if own_pack.is_empty():
		var s := pack_spec(model)
		own_pack = {"cells": int(s["cells"]), "lihv": bool(s["lihv"]), "capacity_mah": float(s["capacity_mah"]),
			"pack": "%dS %d mAh %s" % [int(s["cells"]), int(s["capacity_mah"]), "LiHV" if bool(s["lihv"]) else "LiPo"],
			"mah_used": 0.0, "current": 0.0, "cell_voltage": cell_volts}
	own_pack["current"] = amps
	own_pack["mah_used"] = float(own_pack["mah_used"]) + amps * dt / 3.6
	own_pack["cell_voltage"] = cell_volts


## A respawn: a fresh pack (without battery_sag; battery_sag has its own rules)
func own_pack_reset() -> void:
	own_pack.clear()


## Called by the hook when a model starts flying a drone
func flying(vehicle_id: String, model: RefCounted) -> void:
	var key := vehicle_id + " " + str(model.get_id())
	if _said.has(key):
		return
	_said[key] = true
	var s := drone_cfg(vehicle_id)
	var air := air_node()
	var bat := pack_spec(model)
	var no_cap: bool = float(model.battery_spec()["capacity_mah"]) <= 0.0
	_say("flying %s with %s (%d g, %d Hz loop): drag %s per second; %s; pack %dS %d mAh%s, %.1f mOhm a cell (from the identified sag), %s" % [
		vehicle_id, model.get_display_name(), int(round(float(model.plant_params.mass) * 1000.0)), int(round(model.get_loop_hz())),
		str(snappedf(float(s["drag"]) if float(s["drag"]) > 0.0 else model.get_package_linear_drag(), 0.01)),
		("prop wash from dirty_air, wash %s and wake %s (calibrated x propwash %s)%s" % [str(snappedf(model.get_air_wash(), 0.01)),
			str(snappedf(model.get_air_wake(), 0.01)), str(s["propwash"]), ", with its floor and ceiling effect" if bool(cfg["ground_ceiling_effect"]) else ""])
			if air != null else "no prop wash (dirty_air is not loaded: clean air)",
		int(bat["cells"]), int(bat["capacity_mah"]),
		" (NOT IN THE PACKAGE: battery_capacity_mah from settings.cfg)" if no_cap else "", float(bat["resistance_mohm"]),
		"battery_sag's pack, driven by the model's motor current" if battery_node() != null else "sagging but not draining (battery_sag is not loaded)"])
	_write_status()


# After every mod's zm_init and before the game loads its first scene
func _ready() -> void:
	if not _copied:
		return
	var fc := _flight_controller()
	if _mode_3d():
		_why_not = "mode_3d is in effect (it flies the quad with motors that run both ways, a mixer the Betaflight models do not have)"
		_say("not in effect: " + _why_not + ". The game, flight_controller and mode_3d fly every drone as without this mod; "
			+ "to fly the drones with a model, set enabled=false in mode_3d/settings.cfg (and the other way round)")
		_write_status()
		return
	var why := _check()
	if why == "":
		why = _mount(fc)
	if why != "":
		_why_not = why
		_say("not in effect: " + why)
	else:
		on = true
		_say("in effect: the drones with a model are flown by it" + (", the others by flight_controller" if fc else ", the others by the game's own physics"))
		_coexistence(fc)
	_write_status()


# flight_controller, if it is loaded and in effect (it mounts before this does, as it comes first
# in name order; listed after this in override.cfg's mods=, it finds the game's script already
# loaded and stays out)
func _flight_controller() -> bool:
	var f = _core.call("get_mod", "flight_controller") if _core != null and _core.has_method("get_mod") else null
	return f != null and is_instance_valid(f) and bool(f.get("on"))


# mode_3d, if it is loaded and in effect - or not started yet, as it comes after this in override.cfg's
# mods=, where it takes the quad's script if this does not stay out. Both replace the game's quad script
# (and 3D mode is Betaflight's other mixer: motors both ways), so only one can fly; mode_3d's is
# the one that stays, so the result does not depend on the order the mods start in
func _mode_3d() -> bool:
	var m = _core.call("get_mod", "mode_3d") if _core != null and _core.has_method("get_mod") else null
	return m != null and is_instance_valid(m) and (not m.is_node_ready() or bool(m.get("on")))


func _coexistence(fc: bool) -> void:
	if _core == null or not _core.has_method("get_mods"):
		return
	var lines := PackedStringArray()
	if fc:
		lines.append("flight_controller: in effect; it flies the drones real_quad has no model for, real_quad's own controller replaces it where a model flies")
	elif _core.call("get_mod", "flight_controller") != null:
		lines.append("flight_controller: loaded but not in effect, so not used")
	if air_node() != null:
		lines.append("dirty_air: the prop wash of the modelled quads - its wake, gusts, descent curve and vortex ring state go into each prop of the model, with the wash and wake of the model's package"
			+ ("; and its floor and ceiling effect" if bool(cfg["ground_ceiling_effect"]) else " (its floor and ceiling effect off: ground_ceiling_effect=false)") + "; it pushes and shakes only the other drones")
	else:
		lines.append("dirty_air: not loaded, so the modelled quads fly without prop wash (clean air)")
	if battery_node() != null:
		lines.append("battery_sag: its pack is the modelled quad's - drained by the model's motor current, its voltage under load what the motors get; fresh packs and settings as battery_sag's")
	else:
		lines.append("battery_sag: not loaded, so a modelled quad's pack sags as identified but does not drain (shown on fpv_osd through real_quad)")
	for id in ["rotor_drag", "motor_response"]:
		if _core.call("get_mod", id) != null:
			lines.append("%s: its forces do not reach a modelled quad (the physics engine ignores them while the model flies it): the model has its own %s" % [
				id, {"rotor_drag": "rotor drag (drag= in settings.cfg)", "motor_response": "motor response"}[id]])
	var pd := props_node()
	if pd != null:
		if pd.has_method("set_model"):
			lines.append("prop_damage: its worn props' thrust loss goes into each rotor of the modelled quad (a broken prop gives none), "
				+ "the props' out-of-balance becomes vibration in the gyro the Betaflight loop reads; hits, clips and kicks still push the quad, "
				+ "and it no longer pushes or shakes the model itself")
		else:
			lines.append("prop_damage: too old for real_quad (no set_model()): a modelled quad's props stay new; its hits and kicks still push the quad")
	elif _core.call("get_mod", "prop_damage") != null:
		lines.append("prop_damage: loaded without props, so the modelled quad's props stay new")
	if _core.call("get_mod", "pilot_audio") != null:
		lines.append("pilot_audio: the motor voices follow the model's four rotor speeds")
	for l in lines:
		_say(l)


func _game_dir() -> String:
	return OS.get_executable_path().get_base_dir().path_join("real_quad")


# The game's player script, copied to real_quad/base so that player.gd can extend it
func _copy() -> String:
	if not FileAccess.file_exists(GAME):
		return "the game has no " + GAME
	var d := FileAccess.get_file_as_bytes(GAME)
	if d.size() < 12 or d.slice(0, 4).get_string_from_ascii() != "GDSC":
		return "not a script this knows (compiled GDScript)"
	var md5 := FileAccess.get_md5(GAME)
	if md5 in KNOWN_MD5:
		_say("the game's player script is the one this was made for (md5 %s)" % md5)
	elif bool(cfg["unknown_game_version"]):
		_say("the game's player script (md5 %s) is not the one this was made for: using it anyway (unknown_game_version=true)" % md5)
	else:
		return "the game was updated: its player script (md5 %s) is not the one this was made for (%s); set unknown_game_version=true in settings.cfg to try it anyway" % [md5, ", ".join(KNOWN_MD5)]
	DirAccess.make_dir_recursive_absolute(_game_dir().path_join("base"))
	var f := FileAccess.open(_game_dir().path_join(COPY), FileAccess.WRITE)
	if f == null:
		return "could not write " + _game_dir().path_join(COPY)
	f.store_buffer(d)
	f.close()
	return ""


# The copy has what the player scripts and the hook use, and the scripts compile with it
func _check() -> String:
	# loaded by its .gd name (base/player_rigid_body.gd.remap points it at the .gdc), as a
	# script without a class_name is only found under that name
	var copy = load(DIR + "base/player_rigid_body.gd")
	if not (copy is Script) or not copy.can_instantiate():
		return "the copy of player_rigid_body.gd does not load; an update of this mod is needed"
	var have := {}
	for m in copy.get_script_method_list():
		have[m["name"]] = true
	for m in NEED_METHODS:
		if not have.has(m):
			return "player_rigid_body.gd has no %s() any more; an update of this mod is needed" % m
	var props := {}
	for p in copy.get_script_property_list():
		props[p["name"]] = true
	for p in NEED_PROPERTIES:
		if not props.has(p):
			return "player_rigid_body.gd has no %s any more; an update of this mod is needed" % p
	var has_signal := false
	for s in copy.get_script_signal_list():
		has_signal = has_signal or s["name"] == "teleported"
	if not has_signal:
		return "player_rigid_body.gd has no teleported signal any more; an update of this mod is needed"
	return ""


func _mount(over_fc: bool) -> String:
	var script_name := "player_fc.gd" if over_fc else "player.gd"
	var ours = ResourceLoader.load(DIR + script_name, "", ResourceLoader.CACHE_MODE_IGNORE)
	if not (ours is Script) or not ours.can_instantiate():
		return "%s does not work with this version of the game%s; an update of this mod is needed" % [script_name, " and flight_controller" if over_fc else ""]
	var pck := _game_dir().path_join("remap.pck")
	var pk := PCKPacker.new()
	if pk.pck_start(pck) != OK:
		return "could not write " + pck
	var src := "remap/player_rigid_body_over_fc.gd.remap" if over_fc else "remap/player_rigid_body.gd.remap"
	pk.add_file("res://player_rigid_body.gd.remap", _game_dir().path_join(src))
	if pk.flush() != OK or not ProjectSettings.load_resource_pack(pck, true):
		return "could not mount " + pck
	# a script already loaded under the game's path (flight_controller's own) is read again
	# from the remap, in place
	var got = ResourceLoader.load("res://player_rigid_body.gd", "", ResourceLoader.CACHE_MODE_IGNORE)
	if not (got is Script) or not got.get_script_constant_map().has("ZM_REAL_QUAD"):
		return "the game already had player_rigid_body.gd loaded before this mod started (by another mod?)"
	return ""


func _say(s: String) -> void:
	_log.append(s)
	print("[real_quad] " + s)


func _write_status() -> void:
	var f := FileAccess.open(_game_dir().path_join("status.txt"), FileAccess.WRITE)
	if f != null:
		f.store_string("\n".join(_log) + "\n")
