real_quad 1.0
==============

Flight models of real quads. The Zone FPV turns its quads with an
idealised torque that goes wherever the sticks point, and pushes them with
one thrust at their centre. With real_quad, a drone that has a model flies
as the real quad did: a Betaflight rate loop (the firmware's own code,
ported) drives four motors that each take time to change speed, on a quad
worked out from that real quad's blackbox logs - its motors and ESCs,
props, battery, weight, inertia and drag. It turns only as its motors can
turn it, runs out of motor in a punch, sags with its battery and - with
dirty_air - shakes in its own prop wash. The drones without a model are
left as they are. Off until you turn it on. Best with dirty_air (the air:
prop wash, floor and ceiling) and battery_sag (the pack that drains),
whose work goes into the model.

It comes with two models: freestyle5, a 5" freestyle quad (650 g, 2207
1960KV motors, 51499 props, 6S 1500 mAh, 4 kHz loop) for the game's 5"
Freestyle, and freestyle35, a 3.5" quad (300 g, 1604.5 3650KV motors, Gemfan
3525 props, 4S 850 mAh, 8 kHz loop) for the game's 3.5" Freestyle. Each has
the report of how it was worked out (report.md, in German).


TURNING IT ON
  Set enabled=true at the top of real_quad/settings.cfg (made at the first
  launch), save, restart the game. real_quad/status.txt says whether it is
  in effect and which drone it flies with which model.


WHAT CHANGES
  For a drone with a model (the game's 5" Freestyle: freestyle5), each
  physics tick (the game's: 500 a second):
    - the sticks go through the rates and ask for a rotation rate, as on a
      radio. By default the rates are your own, from Pause > Rates in the
      game (Actual, Betaflight or KISS; a change counts at once);
      rates="model" takes those of the real quad
    - Betaflight 2026.6's rate loop, with the settings read from the real
      quad's own blackbox header: rates, RC smoothing, feedforward, PID
      with I-term relax and anti-gravity, D filters and RPM filter, TPA,
      dynamic idle, airmode and the mixer. It runs at the firmware's own
      rate (4000 times a second for freestyle5), several times each tick
    - four motors follow the loop's outputs through the ESC and motor
      response worked out from the logs (speed against throttle, voltage
      and prop load, the delay of the signal and the motor). Each prop
      pushes at its own corner; its speed sets its thrust, its drag torque
      and - as it speeds up or slows down - the kick of its spin on the
      frame, which is what starts and stops a yaw
    - the battery: one pack, battery_sag's, whose voltage under load is
      what the motors get. The motors' current is the rotors' mechanical
      power (prop drag torque and spin-up) over a drive efficiency of 0.78
      (plant.json drive_efficiency): freestyle5 about 5 A hovering (5 g/W)
      and 120-170 A at full throttle, freestyle35 about 3.5 A and 55 A. It
      drains the pack over the flight, and the pack's resistance is the sag
      worked out from the logs (freestyle5 3.4, freestyle35 11.5 mOhm a
      cell). A fresh pack flies as the real one did (freestyle5's
      freshest log was 4.22 V a cell; freestyle35's was 3.99, so its fresh
      pack is a few percent weaker than a real full one). As it drains the
      quad needs more throttle and punches weaker. Fresh packs on respawn
      and the rest are battery_sag's settings; fpv_osd shows the pack.
      Without battery_sag the pack sags as worked out but does not drain
      (fpv_osd shows it through real_quad)
    - the model's weight and inertia (roll, pitch and yaw are not equal),
      its body's drag in the air and the drag of the spinning props. The
      game's own thrust, drag and torque are not used (its drag
      multipliers and wind are: see THE GAME'S OWN SETTINGS)
    - prop wash is dirty_air's: the air it works out at each prop - the
      flow and gusts of the wake the quad left behind, its descent curve
      and vortex ring state - goes into the model's props (descending,
      dirty_air's descent curve replaces the props' own inflow, so nothing
      counts twice). Each model sets dirty_air's wash, wake and gust for
      its quad (plant.json dirty_air_wash, dirty_air_wake, dirty_air_gust;
      freestyle5 0.7, 1 and 10.5; freestyle35 1, 1 and 2.2) and a light
      turbulence in calm air (turb_base_ms), so that the gyro shakes as
      much as the real quad's did in its logs. Dropping straight down and
      punching out is the worst case. Without dirty_air the quad flies in
      clean air: no prop wash, only the light calm-air turbulence
    - the game keeps its inputs and arming, OSD and camera (its camera
      settings apply), crash and respawn, sound (its pitch follows the
      model's motors, scaled to the game's own range), multiplayer and
      stats. The physics engine still moves the quad and finds what it
      hits; when a crash or a respawn changes its speed, the model takes
      that over

  What you notice:
    - the quad turns, stops, yaws, punches and falls as the real one does:
      the 5" reaches 90% of a full roll rate in about 40 ms
    - with dirty_air a hover is calm, a punch shows a couple of degrees a
      second of shake, and a drop and catch about fourteen (calibrated on
      the real quad's logs)
    - the hover throttle on a fresh pack is about 26% (the real quad's
      logs: about 25%; freestyle35 32%, its logs about 30%)

  The CPU: about a quarter of a millisecond a physics tick for the loop at
  4000 Hz (roughly an eighth of a core), about 0.4 ms at 8000 Hz.


MODELS
  A model is a folder in real_quad/models with three files:
    craft.json  who it is: id, name, zone_vehicle_id (the game's drone it
                is made for), weight, linear_drag (the default of the
                drag setting) and a description of the hardware. Its
                camera angle and fov are not used: the game's camera
                settings apply
    fc.json     the real flight controller's settings, from its blackbox
                header: the loop rate, rates, PIDs, filters...
    plant.json  the quad worked out from the logs: motors, thrust and
                torque constants, inertia, drag, battery sag, and
                dirty_air's wash, wake and gust for it (dirty_air_wash,
                dirty_air_wake, dirty_air_gust; 1 each if missing)
  craft.json "battery": {"cells", "capacity_mah", "lihv"} is the pack it
  flies on. Without capacity_mah, battery_capacity_mah in settings.cfg is
  used and status.txt says so.

  Any number of models can be there, for any drone; a folder dropped in is
  found at the next start (or with Rescan models on the Mods page). Models you
  make yourself are kept apart from the game folder, where updates leave
  them alone: in the game's user folder, real_quad/models (the Mods page's
  Build a model button names it). A model there with the id of one that
  came with real_quad takes its place, and status.txt says so. A model
  made by a newer real_quad (craft.json package_format above 2) is skipped,
  and status.txt says so too. settings.cfg says which flies which drone. The
  loop rate comes from fc.json: 4000 Hz for freestyle5; a model with an
  8000 Hz loop works the same way.


WITH THE OTHER MODS
  The game's quad is moved by real_quad itself while a model flies it, and
  the physics engine ignores forces applied to such a body. So the mods
  that push the quad with forces do not reach it and nothing is counted
  twice; those that have a better answer for a model hand it over through
  a small interface (below). Other drones, not flown by a model, are
  exactly as with those mods. real_quad/status.txt says at each launch what
  is used how, and what stands aside and why.

  real_quad has its own flight controller, the Betaflight port; it does not
  use flight_controller's. The two share the drones:
    - flight_controller: if it is in effect, real_quad flies the drones it
      has a model for, with its own controller, and flight_controller flies
      all the others. Where a model flies, flight_controller's controller is
      not run and none of its settings (PIDs, motor times...) apply.
      zonemods loads mods in name order, so flight_controller comes first.
      If you list the mods in override.cfg with real_quad first,
      flight_controller stays out altogether and says so in its
      status.txt: real_quad then flies its drones and the game the rest

  These work with real_quad, and the model uses what they give:
    - dirty_air: the prop wash, and the floor and ceiling. real_quad calls
      its set_model() each tick (the model's thrust, props, frame, wash,
      wake and gust); dirty_air then works out the air at each prop of that
      quad and the model takes it. dirty_air pushes the quad no more and
      does not shake it. Its settings (descent, wake, wash, ground_effect,
      ceiling_effect) apply as for every drone
    - battery_sag: its pack is the model's (see WHAT CHANGES). real_quad
      drives it with external_pack() and drive(), and the motors run on its
      voltage
    - prop_damage: worn props lose thrust in their own rotor (a broken prop
      gives none: the quad usually goes down, as a real one does), and a
      prop out of balance shakes the gyro the Betaflight loop reads - a
      tone at that rotor's speed, which the RPM filter takes most of off,
      and noise above it - so the PIDs react as on a quad with a bent
      prop: busier, hotter motors. real_quad calls prop_damage's
      set_model() each tick (the rotors' spin, for the wear of a hit) and
      gets each prop's thrust factor and out-of-balance back, in
      Betaflight's motor order. prop_damage then neither takes thrust off
      the body with a force nor turns it about to shake it. Hits, clips,
      bounces and kicks still push the quad. The size of the vibration was
      chosen by feel, not worked out from a log. A prop_damage without
      set_model() (an older one) leaves the props new, and status.txt says
      so
    - fpv_osd, mod_settings, drones and pilot_audio work as before: fpv_osd
      shows the pack (through real_quad without battery_sag), mod_settings
      has real_quad's settings, and pilot_audio places the quad's sound.
      real_quad also puts the model's four rotor speeds on the quad
      (metadata "motor_rpm", rpm) for a sound mod that can use them
    - race times are never posted while real_quad is in effect, as the
      physics are not the game's

  These do not reach a modelled quad, as the model has its own:
    - rotor_drag: the model has its own rotor drag (drag= in settings.cfg);
      rotor_drag still sets the body's damping, which real_quad does not
      use
    - motor_response: the model has its own motors

  This stands aside:
    - mode_3d: real_quad does not start while mode_3d is in effect, with or
      without flight_controller. Both replace the game's quad script, and
      3D mode is a different Betaflight mixer (motors turning both ways)
      that the models do not have. mode_3d stays whatever order the mods
      start in, and every drone is flown as without real_quad: by mode_3d
      (through flight_controller if that is in effect) or the game.
      status.txt says "not in effect: mode_3d is in effect", and so does
      the Mods page. To fly the models, set enabled=false in
      mode_3d/settings.cfg

  Nothing to do with the quad's physics, so not touched: analog400mw (the
  video picture and signal), throttle_fix (the radio's throttle reading),
  player_list (the players' names), examples/glb_anim (animated maps) and
  zonemods itself.


HOW IT WORKS
  The game's quad is the script player_rigid_body.gd. At startup real_quad
  copies it out of the game (into real_quad/base), and mounts a small
  resource pack (real_quad/remap.pck) that points the game at real_quad's
  player.gd instead: the game's own script with its physics, sound and body
  steps handed to hook.gd. With flight_controller in effect, player_fc.gd
  is used, which extends flight_controller's fc.gd the same way - only so
  that flight_controller still flies the drones without a model; where a
  model flies, its controller is skipped. Nothing of
  the game's is shipped or changed on disk.

  The models were made and tried with one version of the game (the player
  script's md5 is checked). After a game update real_quad stays out, the
  game runs as without it and status.txt says so, until real_quad is
  updated. unknown_game_version=true tries it anyway, if the script still
  has everything the hook needs.

  The files: flightmodel/ (bf_controller.gd the Betaflight port, bf_quad.gd
  the quad, bf_model.gd the model as the hook uses it, model_registry.gd
  the list of models), hook.gd, player.gd, player_fc.gd, main.gd.


SETTINGS
  Edit real_quad/settings.cfg, save, restart the game - or change them in
  Pause > Mods with mod_settings, and they take effect at once.
    - ground_ceiling_effect: with dirty_air, its floor and ceiling effect on
      each prop
    - battery_capacity_mah: the pack size for a model whose package does not
      give it
    - unknown_game_version: see HOW IT WORKS
    - model_builder: shows the Build a model from logs button on the Mods page
      (the window that makes a model from Betaflight blackbox logs; not
      finished, off by default)
    - a section for each of the game's quads:
        model: "auto" (the model made for it, if any), "off" (the game's
          physics), or a model's name - any model on any drone
        rates: "game" (the default: the sticks' rates are the game's own,
          Pause > Rates, and follow a change at once) or "model" (the
          rates of the model's own Betaflight setup)
        propwash: the strength of the prop wash, 1 = as calibrated, 0 =
          none (times the wash and wake the model hands to dirty_air)
        drag: the props' drag on the quad moving across them, 1/s at hover;
          0 = the model's own


THE GAME'S OWN SETTINGS
  For a quad a model flies:
    These apply: Rates (rates=game; the max-rate readout of the OSD shows
      them), the Physics menu's drag multipliers (surface area x, y and z
      in percent, 100 = the body drag worked out for the model; z scales
      the drag flying forward or back, x sideways, y up or down), the wind
      and its gusts and shelter (the quad flies through moving air: body
      drag and the props' inflow, with or without dirty_air), the camera
      settings, and the game's silence of the motors while its window has
      no focus.
    The model decides, and the Physics menu does nothing for: weight,
      thrust and top speed (the model's mass, inertia, motors, props and
      battery).
    Turtle mode and the pause menu use the game's own physics, as for any
      drone: the model flies again when you are back.


MULTIPLAYER AND RACES
  Only the quad you fly is changed, on your computer; others see it fly
  (its position and attitude). The game sends no motor speeds, so what
  others hear of it is their own game's (or pilot_audio's) idea of it.
  Race times are not submitted.


INSTALL
  1. Steam: right-click The Zone FPV > Manage > Browse local files.
  2. Extract the zip into that folder, so override.cfg and the zonemods,
     drones and real_quad folders sit next to thezone.exe. Replace files
     when asked. (drones is only needed for dirty_air and
     flight_controller.)
  3. Turn it on: enabled=true in real_quad/settings.cfg (see above).

  real_quad is loaded by zonemods, which loads other mods alongside it - see
  zonemods/README.txt.


UNINSTALL
  To turn real_quad off without uninstalling it, set enabled=false at the
  top of real_quad/settings.cfg.

  Delete the real_quad folder. If no other mods use zonemods, delete
  override.cfg and the zonemods folder too.


TROUBLESHOOTING
  real_quad/status.txt is rewritten at every launch: the models found, the
  game's script, whether it is in effect, how the other mods are used, and
  a line each time you fly a drone with a model. zonemods/status.txt lists
  which mods loaded.


LICENSE NOTE
  flightmodel/bf_controller.gd is a port of Betaflight's source (GPL-3.0)
  and so a derivative work of it.
