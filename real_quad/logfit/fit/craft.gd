extends RefCounted
## real_quad logfit: the craft (the form, contract 2.2), its physically scaled class defaults and the
## package craft.json (contract 4.1, package_format 2).
##
## The craft is a Dictionary in the reference's craft.json layout: id, name, mass_kg, cells or
## battery {cells, capacity_mah, lihv}, pack ("lipo" | "lihv" | "liion"), motor {kv},
## prop {diameter_in, pitch_in, blades}, wheelbase_mm, top_speed_kmh, ducted, class,
## zone_vehicle_id, same_quad. Keys starting with "_" are the pipeline's own (_cells, _kv, _pack).
##
## Defaults: where the logs cannot tell a parameter, a reference craft (a 5.1" 650 g quad identified
## from logs) is scaled with simple physics (prop diameter D, mass m, blades); the report flags each.

const Classes := preload("classes.gd")

const G := 9.81
const RHO := 1.225
const IN := 0.0254
const PACKAGE_FORMAT := 2
const PACKS := ["lipo", "lihv", "liion"]
const FULL_V := {"lipo": 4.2, "lihv": 4.35, "liion": 4.2}

const REFERENCE := {
	"D_in": 5.1, "mass": 0.65,
	"arm": 0.080,
	"Ixx": 2.0e-3, "Izz_over_Ixy": 1.7,
	"kT": 1.6e-6,
	"kQ_over_kT": 1.7e-8 / 1.45e-6,
	"J_rotor": 5.0e-6,
	"cd_area": [0.00978, 0.01172, 0.02947],
	"rotor_drag_per_s": 0.40,
	"battery_r": 0.0825,
	"turb_base_ms": 0.31, "pw_turb_ms": 3.8, "pw_noise_hz": 25.0,
}


## [a-z0-9_] runs of other characters as one "_", at most 32 characters.
static func slug(name: String) -> String:
	var s := ""
	var last_us := false
	for ch in name.to_lower():
		var ok := (ch >= "a" and ch <= "z") or (ch >= "0" and ch <= "9") or ch == "_"
		if ok:
			s += ch
			last_us = false
		elif not last_us:
			s += "_"
			last_us = true
	while s.begins_with("_"):
		s = s.substr(1)
	while s.ends_with("_"):
		s = s.substr(0, s.length() - 1)
	if s == "":
		s = "craft"
	return s.substr(0, 32)


static func _dig(c: Dictionary, path: Array) -> Variant:
	var v: Variant = c
	for k in path:
		v = v.get(k) if v is Dictionary else null
	return v


static func _num(c: Dictionary, path: Array, lo: float, hi: float, what: String, errs: Array, code: String = "") -> void:
	var v: Variant = _dig(c, path)
	if v == null:
		return
	var cd := code if code != "" else "form_" + str(path[-1])
	var x := 0.0
	if v is float or v is int:
		x = float(v)
	elif str(v).is_valid_float():
		x = str(v).to_float()
	else:
		errs.append([cd, "%s: '%s' is not a number" % [what, str(v)], "enter a number between %s and %s" % [_g(lo), _g(hi)]])
		return
	if not (lo <= x and x <= hi):
		errs.append([cd, "%s %s is out of range (%s-%s)" % [what, _g(x), _g(lo), _g(hi)], "enter a value between %s and %s" % [_g(lo), _g(hi)]])


static func _g(x: float) -> String:
	return str(int(x)) if x == floorf(x) and absf(x) < 1e15 else str(x)


## Range checks of contract 2.2: [[code, message, fix], ...].
static func validate_form(c: Dictionary) -> Array:
	var errs: Array = []
	if c.get("mass_kg") == null:
		errs.append(["form_mass", "all-up weight with pack is required (no default)", "weigh the quad with its pack and enter mass_kg"])
	else:
		var m: Variant = c.mass_kg
		_num({"m": float(m) * 1000.0 if (m is float or m is int) else m}, ["m"], 10, 5000, "weight [g]", errs, "form_mass")
	_num(c, ["battery", "cells"], 1, 12, "cells", errs)
	_num(c, ["cells"], 1, 12, "cells", errs)
	_num(c, ["battery", "capacity_mah"], 50, 30000, "pack capacity [mAh]", errs)
	_num(c, ["motor", "kv"], 100, 60000, "motor KV", errs)
	_num(c, ["prop", "diameter_in"], 1.0, 13.0, "prop diameter [in]", errs)
	_num(c, ["prop", "pitch_in"], 0.5, 10.0, "prop pitch [in]", errs)
	_num(c, ["prop", "blades"], 2, 6, "prop blades", errs)
	_num(c, ["wheelbase_mm"], 40, 1000, "wheelbase [mm]", errs)
	_num(c, ["top_speed_kmh"], 20, 300, "top speed [km/h]", errs)
	var name: Variant = c.get("name")
	if name != null and not (1 <= str(name).length() and str(name).length() <= 40):
		errs.append(["form_name", "name must have 1-40 characters", "shorten the name"])
	if c.get("id") != null:
		var re := RegEx.create_from_string("^[a-z0-9_]{1,32}$")
		if re.search(str(c.id)) == null:
			errs.append(["form_id", "id '%s' must be 1-32 characters of a-z, 0-9, _" % str(c.id), "use the slug of the name"])
	if c.get("class") != null and not str(c["class"]) in Classes.ORDER:
		errs.append(["form_class", "unknown craft class '%s'" % str(c["class"]), "one of " + ", ".join(Classes.ORDER)])
	if not pack_type(c) in PACKS:
		errs.append(["form_pack", "unknown pack type '%s'" % str(c.get("pack")), "lipo, lihv or liion"])
	return errs


static func pack_type(c: Dictionary) -> String:
	var p: Variant = c.get("pack")
	if p == null:
		var b: Variant = c.get("battery")
		return "lihv" if b is Dictionary and b.get("lihv", false) else "lipo"
	return str(p).to_lower().replace("-", "")


static func _truthy(v: Variant) -> bool:
	return v != null and not (v is float and v == 0.0) and not (v is int and v == 0) and not (v is String and v == "")


## Fills the form defaults of class `cls` into c (in place); sources: key -> "U" (form) or "C" (class).
static func resolve(c: Dictionary, cls: String, sources: Dictionary) -> void:
	var k: Dictionary = Classes.CLASSES[cls]
	c["class"] = cls
	if not c.get("prop") is Dictionary:
		c.prop = {}
	var prop: Dictionary = c.prop
	for pair in [["diameter_in", "D"], ["blades", "blades"]]:
		sources["prop." + pair[0]] = "U" if _truthy(prop.get(pair[0])) else "C"
		if not _truthy(prop.get(pair[0])):
			prop[pair[0]] = k[pair[1]]
	sources["prop.pitch_in"] = "U" if _truthy(prop.get("pitch_in")) else "C"
	if not c.get("battery") is Dictionary:
		c.battery = {}
	var bat: Dictionary = c.battery
	sources["battery.capacity_mah"] = "U" if _truthy(bat.get("capacity_mah")) else "C"
	if not _truthy(bat.get("capacity_mah")):
		bat.capacity_mah = k.capacity
	sources["ducted"] = "U" if c.get("ducted") != null else "C"
	if c.get("ducted") == null:
		c.ducted = k.ducted
	if not c.has("zone_vehicle_id"):
		var z: String = k.zone
		if cls == "whoop":
			z = "65mm_freestyle" if float(prop.diameter_in) <= 1.3 else "85mm_freestyle"
		c.zone_vehicle_id = z
		sources["zone_vehicle_id"] = "C"


static func geometry(c: Dictionary) -> Dictionary:
	var d := float(c.prop.diameter_in)
	var s: float = d / REFERENCE.D_in
	var pitch_in: Variant = c.prop.get("pitch_in")
	var cls: Dictionary = Classes.CLASSES[str(c.get("class")) if c.get("class") != null else "five"]
	var wb: Variant = c.get("wheelbase_mm")
	var arm := 0.0
	if _truthy(c.get("arm_m")):
		arm = float(c.arm_m)
	elif _truthy(wb):
		arm = float(wb) / 1000.0 / (2.0 * sqrt(2.0))
	var pitch := float(pitch_in) if _truthy(pitch_in) else float(cls.pitch) / float(cls.D) * d
	return {"D_in": d, "scale": s, "prop_radius": d * IN / 2.0,
		"pitch_m_per_rad": pitch * IN / (2.0 * PI), "pitch_assumed": not _truthy(pitch_in),
		"arm": arm if arm != 0.0 else float(REFERENCE.arm) * s, "arm_assumed": arm == 0.0}


static func cells(c: Dictionary) -> int:
	var b: Variant = c.get("battery")
	var v: Variant = b.get("cells") if b is Dictionary else null
	if not _truthy(v):
		v = c.get("cells")
	return int(v) if _truthy(v) else 0


## Hover induced velocity of the rotors (momentum theory) [m/s].
static func induced_velocity(mass: float, prop_radius: float) -> float:
	return sqrt(mass * G / (2.0 * RHO * 4.0 * PI * prop_radius * prop_radius))


## Class defaults scaled from REFERENCE by prop diameter, mass and blades (kT: the measured one, for kQ).
static func defaults(c: Dictionary, kT: float = -1.0) -> Dictionary:
	var g := geometry(c)
	var s: float = g.scale
	var m := float(c.mass_kg)
	var mr: float = m / REFERENCE.mass
	var vr: float = induced_velocity(m, g.prop_radius) / induced_velocity(REFERENCE.mass, REFERENCE.D_in * IN / 2.0)
	var blades := float(c.prop.get("blades", 3)) if _truthy(c.prop.get("blades")) else 3.0
	var ixx: float = REFERENCE.Ixx * mr * s * s
	var s4 := s * s * s * s
	var cd: Array = []
	for x in REFERENCE.cd_area:
		cd.append(float(x) * s * s)
	var d := {
		"Ixx": ixx, "Iyy": ixx, "Izz": REFERENCE.Izz_over_Ixy * ixx,
		"kT": REFERENCE.kT * s4,
		"J_rotor": REFERENCE.J_rotor * s4 * blades / 3.0,
		"J_ref": REFERENCE.J_rotor * s4,
		"kQ_over_kT": REFERENCE.kQ_over_kT * s,
		"cd_area": cd,
		"rotor_drag_per_s": REFERENCE.rotor_drag_per_s,
		"battery_r": REFERENCE.battery_r,
		"turb_base_ms": _round(REFERENCE.turb_base_ms * vr, 3), "pw_turb_ms": _round(REFERENCE.pw_turb_ms * vr, 2),
		"v_induced_ratio": vr,
		"pw_noise_hz": REFERENCE.pw_noise_hz,
	}
	if kT >= 0.0:
		d.kQ = REFERENCE.kQ_over_kT * s * kT
	return d


static func _round(x: float, nd: int) -> float:
	var p := pow(10.0, nd)
	return roundf(x * p) / p


## craft.json of the package (contract 4.1): written from the form, not copied.
static func package_craft(c: Dictionary) -> Dictionary:
	var prop: Dictionary = c.prop
	var out := {"id": c.id, "name": c.name, "zone_vehicle_id": c.get("zone_vehicle_id") if c.get("zone_vehicle_id") else "none",
		"mass_kg": float(c.mass_kg), "cells": int(c._cells),
		"battery": {"cells": int(c._cells), "capacity_mah": int(c.battery.capacity_mah), "lihv": pack_type(c) == "lihv"},
		"motor": {"kv": int(c._kv)},
		"prop": {"diameter_in": float(prop.diameter_in),
			"pitch_in": float(prop.pitch_in) if _truthy(prop.get("pitch_in")) else _round(float(geometry(c).pitch_m_per_rad) * 2.0 * PI / IN, 2),
			"blades": int(prop.blades)}}
	if _truthy(c.get("top_speed_kmh")):
		out.top_speed_kmh = float(c.top_speed_kmh)
	out.ducted = bool(c.ducted)
	out["class"] = c["class"]
	out.package_format = PACKAGE_FORMAT
	return out
