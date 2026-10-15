pilot_audio 1.1
=================

Hear the quad from where you stand. The Zone FPV plays the motor sound as if
you were on board - as loud 300 m away as on the pad. pilot_audio puts you at the
spawn point, at head height, facing the way the spawn faces, and works out
what reaches you there - how loud, how muffled, how late and from which side:

  loudness   6 dB quieter each time the distance doubles
  air        duller the further away, as the air soaks up the highs
  walls      quieter and duller with a building or hill in the way
  delay      sound takes a moment to arrive, and the pitch rises as the quad
             comes towards you and drops as it goes away (doppler)
  stereo     left or right by where the quad is, a little duller behind you

In multiplayer, other players' quads get a motor sound too (the game has none
for them), heard the same way.

The spawn point is also where the game's analog video link is measured from,
so the picture and the sound agree. The game's S key moves the spawn to where
the quad is (land first - it keeps the quad's heading); X puts it back.


INSTALL
  1. Steam: right-click The Zone FPV > Manage > Browse local files.
  2. Extract the zip into that folder, so override.cfg and the zonemods and
     pilot_audio folders sit next to thezone.exe. Replace files when asked - your
     settings are kept (see UPDATING in zonemods/README.txt).

  pilot_audio is loaded by zonemods, which loads other mods alongside it - see
  zonemods/README.txt.


UNINSTALL
  To turn pilot_audio off without uninstalling it, set enabled=false at the top
  of pilot_audio/settings.cfg.

  Delete the pilot_audio folder. If no other mods use zonemods, delete override.cfg
  and the zonemods folder too.


SETTINGS
  Edit pilot_audio/settings.cfg (made at the first launch), save, restart the game.
  Each effect can be turned off, and how fast the sound fades, how wide the
  stereo is and which way you face can be set. The game's volume slider still
  sets the overall level.

  listen_from="quad" puts the microphone on your own quad instead: it sounds
  as loud as the game plays it, with no distance, air, walls, delay, doppler
  or stereo (with motor voices too); other players' quads are then heard from
  your quad. "pilot" (the default) is the spawn point as above. Changed in
  mod_settings, it applies at once.


OTHER PLAYERS
  The game sends each player's position and sticks, not their motor speed or
  whether they are armed. Their motor sound follows their throttle the way
  the game's does for your quad, and a quad sat still at zero throttle goes
  quiet. The nearest 8 are heard (multiplayer_max); as players come and go
  or fly nearer and further, sounds fade in and out rather than starting and
  stopping dead.


MOTOR VOICES (a prototype, off by default)
  The game plays one recording for the whole quad. With motor_voices=true in
  settings.cfg, your own quad's sound is made of four voices instead, one per
  motor, each at the speed of that rotor (the sound is modelled on the game's
  own quad recording: the blade pass and its overtones with a little
  turbulence noise around each, over a quiet noise bed). A roll or a punch is
  heard as the four pitches moving apart and together.

  It needs the quad to publish its rotor speeds, which real_quad does for the
  drones that have a model; for any other drone, the game's sound plays as
  before. The voices go through the same distance, air, walls, stereo, delay
  and doppler as the game's sound; motor_voices_db sets their loudness against
  it. Other players' quads are not affected.

  For mod authors: the quad's body (the game's Player node) carries the meta
  "motor_rpm", a PackedFloat32Array of the four rotor speeds in rpm (shaft
  revolutions a minute, not electrical), set every physics tick. The recipe
  is in motor_voices.gd.


TROUBLESHOOTING
  pilot_audio/status.txt is rewritten at every launch and says what was set up;
  zonemods/status.txt lists which mods loaded.
