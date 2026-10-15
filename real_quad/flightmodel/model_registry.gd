extends RefCounted
## real_quad: the registry of model packages (models/<id>/ with craft.json, fc.json, plant.json).
##
## Pure logic, no UI: scans one or more root folders for packages, lists them, finds the model for a
## Zone vehicle id, keeps an optional override assignment vehicle -> model (persisted in a
## ConfigFile section) and loads/caches BFModel instances. real_quad/main.gd fills the overrides
## from settings.cfg.
##
##   var reg := ModelRegistry.new(["res://real_quad/models"])
##   reg.scan()
##   for e in reg.list_models(): print(e.id, e.name, e.zone_vehicle_id)
##   reg.set_override("5in_freestyle", "my_other_5in")       # or ModelRegistry.NONE = original physics
##   var m := reg.model_for_vehicle("5in_freestyle")          # BFModel or null (no model -> Zone physics)
##
## Roots are scanned in order; a package id found in a later root replaces the same id from an
## earlier one (user packages override the built-in ones). Each replacement is listed in `notes`.
## A package whose craft.json "package_format" (absent = 1) is newer than FORMAT_MAX is not loaded
## and shows in `errors`; the older package of that id, if any, stays.

const BFModel := preload("bf_model.gd")

const PACKAGE_FILES := ["craft.json", "fc.json", "plant.json"]
## Override value meaning "this vehicle keeps the original game physics".
const NONE := "none"
## The newest package format this reads (2 = made by the log fit; it only adds keys to format 1).
const FORMAT_MAX := 2

var roots: PackedStringArray = []
## Problems found by the last scan() ("<dir>: reason"), for a menu or the log.
var errors: PackedStringArray = []
## Things worth a line in the log that are not errors ("id: <dir> replaces <dir>"), by the last scan().
var notes: PackedStringArray = []

var _entries: Dictionary = {}        # id -> {id, name, zone_vehicle_id, dir, root, craft}
var _overrides: Dictionary = {}      # zone vehicle id -> model id (or NONE)
var _cache: Dictionary = {}          # id -> BFModel
var _failed: Dictionary = {}         # ids whose package could not be loaded (not retried until the next scan)


func _init(p_roots: PackedStringArray = PackedStringArray()) -> void:
	roots = p_roots


func add_root(path: String) -> void:
	if not roots.has(path):
		roots.append(path)


# ------------------------------------------------------------------ scanning
## Rescans all roots. Returns the number of packages found. Cached model instances of packages that
## still exist and did not move stay valid; others are dropped.
func scan() -> int:
	var old := _entries
	_entries = {}
	errors = PackedStringArray()
	notes = PackedStringArray()
	_failed.clear()
	for root in roots:
		var da := DirAccess.open(root)
		if da == null:
			continue                  # a missing root (e.g. no user packages yet) is normal
		var subdirs := da.get_directories()
		subdirs.sort()
		for sub in subdirs:
			_register(root, root.path_join(sub), sub)
	for id: String in _cache.keys():
		if not _entries.has(id) or not old.has(id) or old[id].dir != _entries[id].dir:
			_cache.erase(id)
	return _entries.size()


func _register(root: String, dir: String, dir_name: String) -> void:
	for f: String in PACKAGE_FILES:
		if not FileAccess.file_exists(dir.path_join(f)):
			errors.append("%s: %s missing" % [dir, f])
			return
	var craft: Variant = JSON.parse_string(FileAccess.get_file_as_string(dir.path_join("craft.json")))
	if not (craft is Dictionary):
		errors.append("%s: craft.json is not a JSON object" % dir)
		return
	var id := str(craft.get("id", dir_name))
	if id == "" or id == NONE:
		errors.append("%s: invalid id '%s'" % [dir, id])
		return
	var format := int(craft.get("package_format", 1))
	if format > FORMAT_MAX:
		errors.append("%s: package_format %d is newer than this real_quad reads (%d), made by a newer real_quad" % [dir, format, FORMAT_MAX])
		return
	if _entries.has(id):
		notes.append("%s: %s replaces %s" % [id, dir, _entries[id].dir])
	_entries[id] = {
		"id": id,
		"name": str(craft.get("name", id)),
		"zone_vehicle_id": str(craft.get("zone_vehicle_id", "")),
		"dir": dir,
		"root": root,
		"craft": craft,
	}


# ------------------------------------------------------------------ queries
func has_model(id: String) -> bool:
	return _entries.has(id)


## Entry dictionary {id, name, zone_vehicle_id, dir, root, craft} or {} if unknown.
func get_entry(id: String) -> Dictionary:
	return _entries.get(id, {})


## All packages, sorted by display name.
func list_models() -> Array[Dictionary]:
	var out: Array[Dictionary] = []
	for id: String in _entries:
		out.append(_entries[id])
	out.sort_custom(func(a: Dictionary, b: Dictionary) -> bool:
		return str(a.name).naturalnocasecmp_to(str(b.name)) < 0)
	return out


## Packages made for this Zone vehicle (craft.json zone_vehicle_id), sorted by id.
func models_for_vehicle(vehicle_id: String) -> Array[Dictionary]:
	var out: Array[Dictionary] = []
	var ids: Array = _entries.keys()
	ids.sort()
	for id: String in ids:
		if _entries[id].zone_vehicle_id == vehicle_id:
			out.append(_entries[id])
	return out


## Zone vehicle ids that have at least one package.
func vehicle_ids() -> PackedStringArray:
	var out := PackedStringArray()
	for id: String in _entries:
		var v: String = _entries[id].zone_vehicle_id
		if v != "" and not out.has(v):
			out.append(v)
	out.sort()
	return out


## The model id used for a vehicle: the user's override if set (and still installed, or NONE),
## else the first package declaring that vehicle, else "" (no model: original physics).
func resolve(vehicle_id: String) -> String:
	var ov: String = _overrides.get(vehicle_id, "")
	if ov == NONE:
		return ""
	if ov != "" and _entries.has(ov):
		return ov
	var cands := models_for_vehicle(vehicle_id)
	return cands[0].id if not cands.is_empty() else ""


# ------------------------------------------------------------------ override assignment
## Assigns a model to a vehicle (any model to any vehicle). model_id "" removes the override
## (back to the default), NONE keeps the original game physics for that vehicle.
func set_override(vehicle_id: String, model_id: String) -> void:
	if model_id == "":
		_overrides.erase(vehicle_id)
	else:
		_overrides[vehicle_id] = model_id


## The raw override ("" = none set, NONE, or a model id).
func get_override(vehicle_id: String) -> String:
	return _overrides.get(vehicle_id, "")


func save_overrides(cfg: ConfigFile, section: String = "vehicle_models") -> void:
	if cfg.has_section(section):
		cfg.erase_section(section)
	for v: String in _overrides:
		cfg.set_value(section, v, _overrides[v])


func load_overrides(cfg: ConfigFile, section: String = "vehicle_models") -> void:
	_overrides.clear()
	if not cfg.has_section(section):
		return
	for v in cfg.get_section_keys(section):
		_overrides[v] = str(cfg.get_value(section, v, ""))


# ------------------------------------------------------------------ loading
## The (cached) model instance for a package id, or null if unknown or unreadable.
func load_model(id: String) -> BFModel:
	if _cache.has(id):
		return _cache[id]
	if not _entries.has(id) or _failed.has(id):
		return null
	var m := BFModel.new(_entries[id].dir)
	if not m.valid:
		_failed[id] = true
		errors.append("%s: package could not be loaded" % _entries[id].dir)
		return null
	_cache[id] = m
	return m


## Convenience: the model that flies the given vehicle, or null (original physics).
func model_for_vehicle(vehicle_id: String) -> BFModel:
	var id := resolve(vehicle_id)
	return load_model(id) if id != "" else null
