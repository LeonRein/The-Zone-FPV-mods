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
1855KV motors, 51499 props, 6S 1500 mAh, 4 kHz loop) for the game's 5"
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
      with I-term relax and anti-gravity, the gyro and D filters (RPM
      filter, dynamic and static notches, lowpasses; the gyro's own at the
      gyro rate), TPA, dynamic idle, airmode and the mixer. It runs at the
      firmware's own rate (4000 times a second for freestyle5), several
      times each tick
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
      counts twice). dirty_air's wash and wake stay its own (1); each
      model sets the gusts of the old wake (plant.json dirty_air_gust;
      freestyle5 5.54, freestyle35 4.92) and a light turbulence in calm
      air (turb_base_ms; 0.232 and 0.199 m/s), calibrated so that the gyro
      shakes as much as the real quad's did in its logs: a drop and catch
      as its worst moments, a hover as its calm ones. Dropping straight down and
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
  4000 Hz with the dynamic notch (roughly a ninth of a core), about 0.3 ms
  at 8000 Hz.


MODELS
  A model is a folder in real_quad/models with three files:
    craft.json  who it is: id, name, zone_vehicle_id (the game's drone it
                is made for), weight, linear_drag (the default of the
                drag setting) and a description of the hardware. (The
                camera_angle_deg and camera_fov_deg of older models are
                ignored: the game's camera settings apply)
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
  them alone: in the game's user folder, real_quad/models (the Models...
  button on the Mods page opens the model manager, below). A model there with the id of one that
  came with real_quad takes its place, and status.txt says so. A model
  made by a newer real_quad (craft.json package_format above 2) is skipped,
  and status.txt says so too. settings.cfg says which flies which drone. The
  loop rate comes from fc.json: 4000 Hz for freestyle5; a model with an
  8000 Hz loop works the same way.


MODELS... (THE MODEL MANAGER)
  Pause > Mods > real_quad > Models... lists every model: its grade (A to D,
  how closely it flies like the real quad), whether it came with real_quad or
  is yours, the drones that fly it and the disk it uses. Open one for its
  report (what was measured, what is a class default, what to fly next), its
  logs, and its parameters; there you can fly it on a drone, duplicate it,
  open its folder and, if it is yours, delete it (after asking).
  New model from logs: pick the Betaflight blackbox logs of your quad
  (Betaflight 2025.12 or newer, bidirectional DShot, accelerometer and
  unfiltered gyro logged; chirp sweeps are best), check what was read from
  them, enter weight, cells, KV and the props, and build. The weight, props
  and pack cannot be seen in the logs: the box "All logs were flown with the
  same weight, props and battery" is ticked for you; untick it if they were
  not, and leave those logs out.
  The build is a background job of the mod, not of the window: close the
  window and the build goes on, the Mods page says "Building <name>: <step>
  45 %", a short notice appears when it is done. One build at a time.
  The logs are copied into the model's folder (logs/), identical files once,
  and inputs.json records the form and the logs (name, size, SHA-256): a
  model can be rebuilt without the originals. In a model's page, add or
  remove logs and change the parameters (name, weight, cells, KV, props,
  battery, top speed, the Zone drone...), then Rebuild; the old model flies
  until the new one is ready. A model from before the manager shows "inputs
  unknown": add logs to rebuild it. The top speed is not measurable from
  most logs: enter it in Parameters to set the body drag.
  Prop wash: the build flies the new model through dirty_air's air (no
  map, high up, a few dozen times, in the background: under a minute for
  a 4 kHz quad on most computers, about twice that at 8 kHz) and sizes the
  old wake's gusts to the shake of the logs' drops and catches and the
  calm-air turbulence to their calm hovers; the report says "Prop wash
  calibrated: drop and catch ... (log ...), calm ... (log ...)", or why
  not (dirty_air not installed, no calm hovers or no drops in the logs:
  class values, and what to fly). A model calibrated with another version
  of dirty_air than the one installed says so on its page: rebuild it to
  calibrate again.
  Optimized tune (a build of grade A or B): how would the quad probably
  fly when well tuned? The build then also flies the model in the
  simulator - your own sticks from the logs, stick steps at idle, hover and
  full throttle, flips, a drop and catch through dirty_air's air - and
  searches its Betaflight settings (P, I, D, D-max, feedforward per axis,
  then TPA, dynamic idle, anti-gravity, thrust linear) for the smallest
  tracking error, within bounds for its class (D at most 1.5 x
  Betaflight's default D-max with the RPM filter: the sim has no gyro noise
  or motor heat) and without more motor saturation, overshoot or ringing on
  a plant with less inertia and more motor delay than the flown tune. The
  filters stay as flown. The result stays in the model (fc_optimized.json,
  and tune.md: flown and optimized per setting, what stopped each, the
  predicted changes, what is not modelled; the model's page shows it above
  the report); a drone flies it when its tune setting is "optimized" (see
  SETTINGS; the built-in models come with theirs). A few minutes more for
  the build. Simulator only, not a tune to copy to the real quad.


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

  These work with real_quad, and the model uses what they give. They need
  the versions that came with real_quad (dirty_air 1.1, battery_sag 1.3.1,
  prop_damage 1.0.1 and the like: small additions for flight models, the
  rest unchanged). With a stock version of the same mods - the unpatched
  one from the mod's own download - real_quad still flies, but without
  what that mod would give the model (clean air, a pack that does not
  drain, new props), and status.txt and the Mods page say "a stock
  version" instead of "not loaded":
    - dirty_air: the prop wash, and the floor and ceiling. real_quad calls
      its set_model() each tick (the model's thrust, props, frame, wash,
      wake and gust); dirty_air then works out the air at each prop of that
      quad and the model takes it. dirty_air pushes the quad no more and
      does not shake it. Its settings (descent, wake, wash, ground_effect,
      ceiling_effect) apply as for every drone
    - battery_sag: its pack is the model's (see WHAT CHANGES). real_quad
      drives it with external_pack() and drive(), and the motors run on its
      voltage. hold_cell_volts does nothing for a modelled quad: its motors
      run on the pack's real voltage, and so does the OSD
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
    - race times are never posted while a model flies the quad, as the
      physics are not the game's

  These do not reach a modelled quad, as the model has its own:
    - rotor_drag: the model has its own rotor drag (drag= in settings.cfg);
      rotor_drag still sets the body's damping, which the physics engine
      ignores for such a body (checked: a body on a custom integrator keeps
      its speed whatever its damping), so engine_damping does nothing
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
  has everything the hook needs: it is checked that the script has the
  methods and variables used, no _ready() or _integrate_forces() of its own
  (player.gd replaces those without calling the game's), and that the game's
  ZGamestate, ZSettings (the pause flag, wind, OSD values, rates) and
  z_api.gd still have what the hook reads.

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
    - a section for each of the game's quads:
        model: "auto" (the model made for it, if any), "off" (the game's
          physics), or a model's name - any model on any drone
        rates: "game" (the default: the sticks' rates are the game's own,
          Pause > Rates, and follow a change at once) or "model" (the
          rates of the model's own Betaflight setup)
        tune: "flown" (the default: the real quad's Betaflight tune from
          its logs) or "optimized" (the model's optimized tune, see
          MODELS; without one, or out of date, the flown tune flies and the
          status line says why). A change counts at once
        propwash: the strength of the prop wash, 1 = as calibrated, 0 =
          none (times the wash and wake the model hands to dirty_air)
        drag: the props' drag on the quad moving across them, 1/s at hover;
          0 = the model's own
    - a drone the drones mod adds (zm_vehicle with base=) is flown by the
      game's physics: its id has no model. To fly the model of the drone it
      is built on, add a section of its id to settings.cfg and name the
      model, e.g.
          [meteor75]
          model="freestyle35"
      (a model's page in Models..., Fly it on, does the same). The
      model's weight, thrust and top speed then replace the block's. Until
      then, the Mods page shows the settings of the drone it is built on,
      which do not apply to the added drone; the note at the top says so


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
      drone: the model flies again when you are back, with its rotors,
      controller and pack as they were. It starts from rest again only for
      another drone or model, and after a respawn or a teleport.
    The test wing (and any drone the game flies with another physics
      handler than the quad's) is never flown by a model.


MULTIPLAYER AND RACES
  Only the quad you fly is changed, on your computer; others see it fly
  (its position and attitude). The game sends no motor speeds, so what
  others hear of it is their own game's (or pilot_audio's) idea of it.
  Race times are not submitted while a model flies the quad (nor when it
  is put aside for the pause menu or turtle mode); with the game's own
  physics the game's race upload is as without real_quad.


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
