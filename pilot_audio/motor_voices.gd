extends Node
## pilot_audio: your own quad's motor sound as four voices, one per motor
## (motor_voices, a prototype).
##
## The game plays one recording for the whole quad. Here each motor has a voice
## of its own, played at the speed of that rotor, so rolls, yaws and punches
## are heard as the four pitches moving apart and together.
##
## The sound follows an analysis of the game's own recording (a real 3-blade
## quad). The blade pass (harmonic 3 of the shaft rotation) carries almost
## everything; its octave (6) is at -10 dB, 9 at -22 dB, the shaft rate and the
## rest at -32 dB and below. Each line has a narrow skirt of noise (blade
## loading turbulence), and under it lies a broadband floor 45-60 dB down.
## Loudness goes with the rotor speed as a rotor's does - tones steeper than
## the noise - with a high-pass at the blade pass, so low tones fade; both
## follow the game's own law and equal it at the recording's speed. Relative to
## the game's level at 15k rpm: 4k rpm -24 dB, 11k -7 dB, 27k +14 dB.
## motor_voices_db shifts it all for taste.
##
## A voice is a looped wavetable of TABLE_REVS shaft revolutions: harmonic h is
## Re{(1 + SKIRT z) exp(i (h theta + phi_h))} with z a slow complex noise
## (SKIRT_CTRL points per revolution, low-passed at SKIRT_FC cycles per
## revolution), so the skirts scale with the speed like the lines. A quiet
## noise bed rides with the mean speed. An offline renderer of the same
## recipe, outside this mod, must be kept in step with it.
##
## audio.gd decides when this runs and what it hears: it puts the voices on the
## quad's own audio bus (distance, air, walls and stereo as for the game's
## sound) and hands in, every physics tick, the loudness the game would play,
## the doppler factor and the four rotor speeds (rpm, from the quad body's meta
## "motor_rpm"). Here, every frame, the voice pitches follow their targets
## smoothly.

const RATE := 48000                      # table / noise sample rate [Hz]
const TABLE_N := 128                     # samples per shaft revolution
const TABLE_REVS := 256                  # revolutions in the table (the skirt noise loops after these)
const HARMONICS := [[1, 0.025], [2, 0.009], [3, 1.0], [4, 0.009], [6, 0.30], [9, 0.07], [12, 0.015]]   # [harmonic of f_rev, amplitude]
const TABLE_RMS := 0.3
const SKIRT := 0.17                      # rms of the line modulation noise z, relative to the line
const SKIRT_CTRL := 4                    # points of z per revolution
const SKIRT_FC := 0.3                    # z low-pass corner [cycles per revolution]
const SKIRT_POLES := 6                   # one-pole low-passes in a row
const VOICE_DB := -6.02                  # per voice: four of them share the power of one
const TONE_EXP := 1.3                    # voice amplitude ~ (rpm_i / TONE_REF_RPM) ^ TONE_EXP below the reference ...
const TONE_EXP_HI := 3.5                 # ... and ^ TONE_EXP_HI above it (tones ~ tip speed^7 in power)
const TONE_REF_RPM := 16300.0            # there a voice is at VOICE_DB on the game's loudness (the recording's speed)
const TONE_HP_HZ := 300.0                # a voice's gain also is that of a Butterworth high-pass of TONE_HP_ORDER
const TONE_HP_ORDER := 1                 # at its blade-pass frequency (3 f_rev): low tones fade out at low throttle
const BOOST_MAX_DB := 10.0               # the speed laws lift a voice or the noise at most this much
const NOISE_LEN_S := 4.0
const NOISE_HP_HZ := 40.0
const NOISE_LP_HZ := 2300.0
const NOISE_LP_N := 1                    # one-pole low-passes in a row
const NOISE_RMS := 0.3
const NOISE_DB := -20.5                  # at NOISE_REF_RPM, relative to the four voices together
const NOISE_EXP := 1.0                   # noise amplitude ~ (mean / ref) ^ NOISE_EXP below the reference ...
const NOISE_EXP_HI := 2.5                # ... and ^ NOISE_EXP_HI above it (broadband ~ tip speed^5 in power)
const NOISE_REF_RPM := 16300.0
const NOISE_PITCH_REF_RPM := 15000.0     # noise playback pitch = mean rpm / this
const NOISE_PITCH_MIN := 0.7              # keeps some air in the hiss at idle
const NOISE_PITCH_MAX := 2.5
const SMOOTH_S := 0.01                   # voice pitch follows its target with this time constant
const SILENT_DB := -80.0
const REV_HZ_AT_UNIT_PITCH := float(RATE) / TABLE_N     # f_rev played at pitch_scale 1

var players: Array[AudioStreamPlayer] = []     # the four voices
var noise_player: AudioStreamPlayer = null
var target_pitch := PackedFloat32Array([1.0, 1.0, 1.0, 1.0])
var target_db := PackedFloat32Array([SILENT_DB, SILENT_DB, SILENT_DB, SILENT_DB])
var noise_target_pitch := 1.0
var noise_target_db := SILENT_DB
var _smooth := PackedFloat32Array([1.0, 1.0, 1.0, 1.0])
var _noise_smooth := 1.0
var _fresh := true                       # first targets: take them without gliding
var _want_playing := false

static var _table: AudioStreamWAV = null
static var _noise: AudioStreamWAV = null


## TABLE_REVS shaft revolutions with the skirt noise in them, looped.
static func make_table() -> AudioStreamWAV:
	if _table != null:
		return _table
	var rng := RandomNumberGenerator.new()
	rng.seed = 1
	var nc := TABLE_REVS * SKIRT_CTRL
	var a := 1.0 - exp(-TAU * SKIRT_FC / SKIRT_CTRL)
	var zr := _onepoles_loop(_uniform(rng, nc), a, SKIRT_POLES)
	var zi := _onepoles_loop(_uniform(rng, nc), a, SKIRT_POLES)
	var zsq := 0.0
	for i in nc:
		zsq += zr[i] * zr[i] + zi[i] * zi[i]
	var zk := SKIRT / sqrt(zsq / nc)
	# cos / sin of each harmonic over one revolution, with a random phase each
	var hc: Array[PackedFloat32Array] = []
	var hs: Array[PackedFloat32Array] = []
	var amp := PackedFloat32Array()
	for h in HARMONICS:
		var ph := rng.randf() * TAU
		var c := PackedFloat32Array()
		var s := PackedFloat32Array()
		c.resize(TABLE_N)
		s.resize(TABLE_N)
		for n in TABLE_N:
			c[n] = cos(TAU * float(h[0]) * n / TABLE_N + ph)
			s[n] = sin(TAU * float(h[0]) * n / TABLE_N + ph)
		hc.append(c)
		hs.append(s)
		amp.append(float(h[1]))
	var total := TABLE_N * TABLE_REVS
	var t := PackedFloat32Array()
	t.resize(total)
	var sq := 0.0
	var nh := amp.size()
	for j in total:
		var pc := float(j * SKIRT_CTRL) / TABLE_N
		var i0 := int(pc)
		var fr := pc - i0
		var i1 := (i0 + 1) % nc
		var er := 1.0 + zk * (zr[i0] * (1.0 - fr) + zr[i1] * fr)
		var ei := zk * (zi[i0] * (1.0 - fr) + zi[i1] * fr)
		var n := j % TABLE_N
		var v := 0.0
		for k in nh:
			v += amp[k] * (er * hc[k][n] - ei * hs[k][n])
		t[j] = v
		sq += v * v
	var g := TABLE_RMS / sqrt(sq / total)
	var bytes := PackedByteArray()
	bytes.resize(total * 2)
	for j in total:
		bytes.encode_s16(j * 2, int(clampf(t[j] * g, -1.0, 1.0) * 32767.0))
	_table = _wav(bytes, total)
	return _table


## Looped noise bed: white noise, one-pole high-pass, NOISE_LP_N one-pole low-passes, seamless.
static func make_noise() -> AudioStreamWAV:
	if _noise != null:
		return _noise
	var n := int(NOISE_LEN_S * RATE)
	var rng := RandomNumberGenerator.new()
	rng.seed = 1
	var x := _uniform(rng, n)
	var lo := _onepoles_loop(x, 1.0 - exp(-TAU * NOISE_HP_HZ / RATE), 1)
	for i in n:
		x[i] -= lo[i]
	var y := _onepoles_loop(x, 1.0 - exp(-TAU * NOISE_LP_HZ / RATE), NOISE_LP_N)
	var sq := 0.0
	for i in n:
		sq += y[i] * y[i]
	var k := NOISE_RMS / sqrt(sq / n)
	var bytes := PackedByteArray()
	bytes.resize(n * 2)
	for i in n:
		bytes.encode_s16(i * 2, int(clampf(y[i] * k, -1.0, 1.0) * 32767.0))
	_noise = _wav(bytes, n)
	return _noise


static func _uniform(rng: RandomNumberGenerator, n: int) -> PackedFloat32Array:
	var x := PackedFloat32Array()
	x.resize(n)
	for i in n:
		x[i] = rng.randf_range(-1.0, 1.0)
	return x


## poles one-pole low-passes in a row (coefficient a), seamless on a loop: the buffer is run through twice
## and the second pass is kept, so the filter states at its start are those of its end.
static func _onepoles_loop(x: PackedFloat32Array, a: float, poles: int) -> PackedFloat32Array:
	var n := x.size()
	var st := PackedFloat32Array()
	st.resize(poles)
	st.fill(0.0)
	var y := PackedFloat32Array()
	y.resize(n)
	for pass_i in 2:
		for i in n:
			var v := x[i]
			for k in poles:
				st[k] += a * (v - st[k])
				v = st[k]
			y[i] = v
	return y


static func _wav(bytes: PackedByteArray, samples: int) -> AudioStreamWAV:
	var w := AudioStreamWAV.new()
	w.format = AudioStreamWAV.FORMAT_16_BITS
	w.mix_rate = RATE
	w.stereo = false
	w.data = bytes
	w.loop_mode = AudioStreamWAV.LOOP_FORWARD
	w.loop_begin = 0
	w.loop_end = samples
	return w


## Makes the five players on the bus. Not playing yet.
func start(bus: String) -> void:
	var tab := make_table()
	for i in 4:
		var p := AudioStreamPlayer.new()
		p.name = "Motor%d" % i
		p.stream = tab
		p.bus = bus
		p.volume_db = SILENT_DB
		add_child(p)
		players.append(p)
	noise_player = AudioStreamPlayer.new()
	noise_player.name = "NoiseBed"
	noise_player.stream = make_noise()
	noise_player.bus = bus
	noise_player.volume_db = SILENT_DB
	add_child(noise_player)
	_fresh = true


## Targets from the four rotor speeds [rpm]. base_db: the loudness the game would play (distance is on the
## bus), doppler: pitch factor of the sound reaching the pilot, playing: the game's sound is on.
func set_targets(rpm: PackedFloat32Array, base_db: float, doppler: float, playing: bool) -> void:
	var mean := 0.0
	for i in 4:
		mean += maxf(rpm[i], 0.0)
	mean *= 0.25
	if mean < 1.0:
		playing = false
	if playing and mean >= 1.0:
		for i in 4:
			var r := maxf(rpm[i], 1.0)
			target_pitch[i] = maxf(r / 60.0 / REV_HZ_AT_UNIT_PITCH * doppler, 0.01)
			target_db[i] = maxf(base_db + VOICE_DB + tone_gain_db(r), SILENT_DB)
		noise_target_pitch = clampf(mean / NOISE_PITCH_REF_RPM, NOISE_PITCH_MIN, NOISE_PITCH_MAX)
		noise_target_db = maxf(base_db + NOISE_DB + speed_law_db(mean, NOISE_REF_RPM, NOISE_EXP, NOISE_EXP_HI), SILENT_DB)
	_want_playing = playing
	_apply_play_state()


## (rpm / ref) ^ exp_lo below ref, ^ exp_hi above, in dB, lifting at most BOOST_MAX_DB.
static func speed_law_db(rpm: float, ref: float, exp_lo: float, exp_hi: float) -> float:
	var l := log(maxf(rpm, 1.0) / ref) / log(10.0)
	return minf(20.0 * (exp_lo if l < 0.0 else exp_hi) * l, BOOST_MAX_DB)


## Level of a voice at rotor speed rpm relative to VOICE_DB [dB]: the speed law (at most BOOST_MAX_DB) and
## the blade-pass high-pass.
static func tone_gain_db(rpm: float) -> float:
	var r := maxf(rpm, 1.0)
	var law := speed_law_db(r, TONE_REF_RPM, TONE_EXP, TONE_EXP_HI)
	var x := 3.0 * r / 60.0 / TONE_HP_HZ
	var xn := pow(x, TONE_HP_ORDER)
	return law + 20.0 * log(xn / sqrt(1.0 + xn * xn)) / log(10.0)


## Stops all voices now (before the bus they are on goes).
func silence() -> void:
	_want_playing = false
	_apply_play_state()


func _apply_play_state() -> void:
	for i in players.size():
		var p := players[i]
		if _want_playing and not p.playing:
			_fresh = true
			p.pitch_scale = target_pitch[i]
			# a different start in the turn for each, so they are not in step
			p.play(randf() * TABLE_N * TABLE_REVS / float(RATE))
		elif not _want_playing and p.playing:
			p.stop()
	if noise_player != null:
		if _want_playing and not noise_player.playing:
			noise_player.play(randf() * NOISE_LEN_S)
		elif not _want_playing and noise_player.playing:
			noise_player.stop()


func _process(dt: float) -> void:
	if not _want_playing:
		return
	var k := 1.0 if _fresh else 1.0 - exp(-dt / SMOOTH_S)
	_fresh = false
	for i in players.size():
		_smooth[i] += (target_pitch[i] - _smooth[i]) * k
		players[i].pitch_scale = maxf(_smooth[i], 0.01)
		players[i].volume_db = target_db[i]
	_noise_smooth += (noise_target_pitch - _noise_smooth) * k
	noise_player.pitch_scale = maxf(_noise_smooth, 0.01)
	noise_player.volume_db = noise_target_db


## What the voices are doing, for tests and report().
func report() -> Dictionary:
	var ps := []
	var db := []
	for p in players:
		ps.append(p.pitch_scale)
		db.append(p.volume_db)
	return {"playing": _want_playing, "pitch_scale": ps, "volume_db": db,
		"noise_pitch": noise_player.pitch_scale if noise_player != null else 0.0,
		"noise_db": noise_player.volume_db if noise_player != null else SILENT_DB}
