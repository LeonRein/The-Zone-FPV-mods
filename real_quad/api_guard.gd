# API variant used while real_quad flies a drone: identical to the game's z_api except that race
# runs are never posted to the leaderboards (the physics here are not the game's, so the times
# would not be comparable). hook.gd puts an instance of it into the race track handler.
extends "res://multiplayer/z_api.gd"

var original = null     # the api that was in the handler before this one, put back when no model flies


func post_race_run(track_id: int, data, callback: Callable):
	print("[real_quad] race run NOT submitted (real_quad physics), track ", track_id)
	return ERR_UNAVAILABLE
