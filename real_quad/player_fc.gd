extends "res://flight_controller/fc.gd"
## real_quad on top of flight_controller: the game's quad flown by flight_controller's
## flight controller (fc.gd) for the drones real_quad has no model for, and by
## real_quad's models for those it has. This script takes the place of the
## game's (a remap in remap.pck) and extends flight_controller's fc.gd; used
## instead of player.gd when flight_controller is in effect. All the work is
## in hook.gd.

const ZM_REAL_QUAD := 1
const Hook := preload("hook.gd")

var zm_rq = null            # hook.gd's Hook for this quad


func _ready() -> void:
	var mod = Hook.find_mod(get_tree())
	if mod != null:
		zm_rq = Hook.new(self, mod)
		zm_rq.ready()


func _physics_process(delta: float) -> void:
	if zm_rq != null:
		zm_rq.before_physics()
	super._physics_process(delta)
	if zm_rq != null:
		zm_rq.after_physics()


# While a model flies the quad flight_controller's own controller stays out: its
# forces would not reach the body anyway
func _handle_rotation(delta: float, setpoint: Dictionary) -> float:
	if zm_rq != null and zm_rq.active:
		return 0.0
	return super(delta, setpoint)


func play_sound():
	if zm_rq != null:
		zm_rq.before_sound()
	super.play_sound()


func _integrate_forces(state: PhysicsDirectBodyState3D) -> void:
	if zm_rq != null:
		zm_rq.integrate_forces(state)
