extends "res://real_quad/base/player_rigid_body.gd"
## real_quad: the game's quad (player_rigid_body.gd), flown by a flight model
## when the drone has one. This script takes the place of the game's (a remap
## in remap.pck) and extends the copy of it made at every launch (base/); all
## the work is in hook.gd. Without a model for the drone, everything is the
## game's own.

const ZM_REAL_QUAD := 1
const Hook := preload("hook.gd")

var rq = null            # hook.gd's Hook for this quad


func _ready() -> void:
	var mod = Hook.find_mod(get_tree())
	if mod != null:
		rq = Hook.new(self, mod)
		rq.ready()


func _physics_process(delta: float) -> void:
	if rq != null:
		rq.before_physics()
	super._physics_process(delta)      # the game: inputs, arming, OSD, sound, stats
	if rq != null:
		rq.after_physics()


# the game's sound step, untyped as there
func play_sound():
	if rq != null:
		rq.before_sound()
	super.play_sound()


func _integrate_forces(state: PhysicsDirectBodyState3D) -> void:
	if rq != null:
		rq.integrate_forces(state)
