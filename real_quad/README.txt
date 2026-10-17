real_quad 1.0
==============

Fly the sim like your real quad. real_quad replaces the game's physics for a
drone with a flight model worked out from the blackbox logs of a real quad:
the real Betaflight rate loop (ported, with your tune and filters), four
motors that take time to spin up, the real weight, inertia, drag and battery
sag. It turns, punches, runs out of motor and shakes in its own prop wash
as the real quad did. Drones without a model fly as before.

Two models come with it: freestyle5 (5", 650 g, 6S) for the game's 5"
Freestyle and freestyle35 (3.5", 300 g, 4S) for its 3.5" Freestyle. You can
make your own from your logs (below).


INSTALL AND TURN ON
  1. Copy the mods into the game folder (Steam: right-click The Zone FPV >
     Manage > Browse local files), so override.cfg and the mod folders sit
     next to the game's exe. Replace files when asked.
  2. Start the game once, then set enabled=true at the top of
     real_quad/settings.cfg and restart.
  3. Fly the game's 5" or 3.5" Freestyle.

  Best together with dirty_air (prop wash, floor and ceiling) and battery_sag
  (a pack that sags and drains).


SETTINGS (Pause > Mods > real_quad, or real_quad/settings.cfg)
  For each of the game's drones:
    model     "auto" (the model made for it), "off" (the game's physics) or
              the name of any model - any model can fly any drone
    tune      "flown" (your real Betaflight tune) or "optimized" (a tune the
              simulator found for the model; simulator only, not for the
              real quad - see the model's tune.md)
    rates     "game" (your rates from Pause > Rates) or "model" (the rates
              from the real quad's logs)
    propwash  1 = as measured in the logs, 0 = none
    drag      0 = the model's own
  The game's camera, rates, wind and drag multipliers still apply; weight,
  thrust and top speed are the model's.


YOUR OWN QUAD
  Pause > Mods > real_quad > Models... > New model from logs: pick your
  blackbox logs, check what was read, enter weight, cells, motor KV and
  props, build. The build runs in the background (a few minutes) and gives
  the model a grade:
    A  flies like your quad          C  right size and power, rough response
    B  close, small differences      D  a sketch: mostly class defaults
  and a report that says what is missing and what to fly next. Add more logs
  to a model later and rebuild: more and better data, a better model.

  What you need
    - Betaflight 2025.12 or newer
    - bidirectional DShot (ESC firmware BLHeli_32, AM32 or Bluejay)
    - an accelerometer that is on and calibrated, and a calibrated battery
      voltage sensor
    - logs of one quad with the same weight, props and battery

  Blackbox setup (Betaflight CLI). The log should run at 1 kHz: 1/8 for an
  8 kHz PID loop, 1/4 for 4 kHz, 1/2 for 2 kHz.
      set blackbox_sample_rate = 1/8
      set blackbox_mode = NORMAL
      set debug_mode = NONE
      set dshot_bidir = ON
      set blackbox_disable_gyro = OFF
      set blackbox_disable_gyrounfilt = OFF
      set blackbox_disable_acc = OFF
      set blackbox_disable_rc = OFF
      set blackbox_disable_setpoint = OFF
      set blackbox_disable_motors = OFF
      set blackbox_disable_rpm = OFF
      set blackbox_disable_bat = OFF
      set blackbox_disable_pids = ON
      set blackbox_disable_attitude = ON
      set blackbox_disable_debug = ON
      set blackbox_disable_gps = ON
      save
  Chirp sweeps (strongly recommended: they measure how fast the motors
  really are). Put the CHIRP mode on a switch and set:
      set chirp_frequency_start_deci_hz = 30
      set chirp_frequency_end_deci_hz = 2000
      set chirp_time_seconds = 20
      set chirp_amplitude_roll = 230
      set chirp_amplitude_pitch = 230
      set chirp_amplitude_yaw = 180
      save

  What to fly (a fresh pack, calm air, plenty of space):
    - chirps: hover, flip CHIRP on, keep the sticks still until the sweep
      ends (20 s), CHIRP off. Each sweep does the next axis; fly 2-3 rounds
      of roll, pitch and yaw
    - 10 s of hands-off hover (ANGLE is fine)
    - 2 vertical punch-outs (full throttle for at least half a second)
    - 3 straight fast passes, each from a steady hover
    - about 5 drops and catches: throttle to zero for a second, then catch
      at mid throttle (this sizes the prop wash)
    - 1-2 minutes of normal flying with fast flips and rolls
  Crashes, time on the ground and broken parts of a log are left out
  automatically.


WITH THE OTHER MODS
  dirty_air, battery_sag, prop_damage, fpv_osd, mod_settings, drones and
  pilot_audio work with it and feed the model (use the versions that come
  with real_quad). flight_controller keeps flying the drones that have no
  model. mode_3d: real_quad stays off while 3D mode is on. Race times are
  not posted while a model flies the quad: its physics are not the game's.


TROUBLESHOOTING
  real_quad/status.txt (rewritten at every start) says whether real_quad is
  in effect, which models it found, which drone flies with which model and
  how the other mods are used. After a game update real_quad stays off until
  it is updated (unknown_game_version=true in settings.cfg tries anyway).

  Uninstall: delete the real_quad folder (to only turn it off, set
  enabled=false).


MORE
  Each model folder (real_quad/models; your own ones are in the game's user
  folder) has report.md - how the model was worked out and how well it
  matches the logs - and, with an optimized tune, tune.md.

  flightmodel/bf_controller.gd is a port of Betaflight's source (GPL-3.0)
  and so a derivative work of it.
