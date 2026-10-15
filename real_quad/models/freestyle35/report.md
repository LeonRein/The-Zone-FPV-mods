# 3.5" freestyle, 300 g, 4S (freestyle35)

## Grade B: close; small differences

- B: chirp sweeps roll/pitch/yaw 2/1/1 (A needs 2/2/1)

## What to fly next
- [ ] 1 more chirp round on pitch (full 20 s sweeps)
- [ ] Enter the quad's top speed in the form (the body drag is a class default), or fly 3 straight fast passes started from a hover

Made by `fpvident` from 6 sessions in 51 s. 15 PASS, 2 WARN.

Form: 0.3 kg, 4S lipo, 3650 KV, prop 3.5" × 2.5" × 3, class mini, top speed unknown km/h.

## Checks
| Check | Result | Detail |
|---|---|---|
| Segmentation auto = hand | PASS | IoU 1.000 (threshold 0.95) |
| Motor dynamics 5–30 Hz | PASS | measured/sim [1.1, 1.1, 1.2, 1.2] (0.8–1.2) |
| Motor KV (vbat scale) | PASS | scale 0.998 (0.8–1.25) |
| Hover detection | PASS | acc, 22.4 s |
| Hover rotor speed between sessions | PASS | spread 1.7 % (threshold 6 %) |
| Motor model free run | PASS | 6.14 Hz RMS (threshold max(12, 7.5 % of 234 Hz) = 17.6) |
| Rotation roll/pitch | PASS | chirp, R² ≥ 0.9, rel. std. error ≤ 4.6 % (threshold 10 %) |
| Rotation yaw | WARN | R² 0.95, rel. std. error 41 % (threshold 25 %) |
| Imbalance consistent | PASS | spread between sessions [0.39, 0.5, 0.45] ×10⁻³ |
| Imbalance size | PASS | |β| 3.61 ×10⁻³ m (≤ 0.1 · arm = 5.5) |
| Battery fit | PASS | vbat, RMS 0.15 V |
| ACC rotor drag | PASS | R² 0.86, uncertainty 0.072 (thresholds 0.5 / 0.15) |
| Top speed backed | WARN | neither a given top speed nor an ACC fit: 112 km/h is an estimate |
| Hover throttle sim vs. log | PASS | 0.309 vs. 0.303 (±0.02) |
| Roll t90 sim vs. log | PASS | 194.4 vs. 195.3 ms (±20 %) |
| Roll lag sim vs. log | PASS | 2.0 vs. 3.0 ms (±25 %, at least 3 ms) |
| Closed-loop tracking | PASS | 0.03 (threshold 0.35) |

## Measured
| Quantity | Value | Source |
|---|---|---|
| Motor/ESC a, b, c (per motor) | see Motor/ESC | eRPM vs. DShot command |
| Thrust coefficient kT | 3.401e-07 | hover 234 Hz at 0.3 kg (acc) |
| Roll/pitch torque gain (→ Ixx, Iyy) | 60.4 / 45.1 | IV, chirp |
| Yaw: J_rotor/Izz, kQ, direction | 0.00152 | IV, motor torque as regressor |
| Torque imbalance β | [0.71, -3.6, 1.1] ×10⁻³ m | eRPM differences at hover |
| Battery V0, sag R | 15.94 V, 0.141 | vbat |
| Rotor drag (linear) | 0.36 1/s | ACC dead reckoning |

## Class defaults and assumptions
| Quantity | Value | Why |
|---|---|---|
| Motor distance (arm) | 55 mm | 80 mm × D/5.1" (only the absolute inertias depend on it) |
| Izz | 0.000614 kg·m² | 1.7 × mean roll/pitch inertia (only J_rotor/Izz is measured) |
| Body drag | cd_area [0.00461, 0.00552, 0.0139] | reference × (D/5.1")² |
| dirty_air gust | 4.01 | prop wash calibration (real_quad calibrate.gd): drop and catch 13.18 deg/s (target 13.22) |
| Calm-air turbulence | 0.193 m/s | prop wash calibration: calm hover 0.77 deg/s (target 0.77) |
| dirty_air wash/wake | 1/1 | dirty_air's own figures |

## Plausibility vs. the mini class band
| Quantity | Value | Band | Result |
|---|---|---|---|
| Weight | 300 g | 180–400 | PASS |
| Cells | 4 S | 4–6 | PASS |
| Hover rotor speed | 234 Hz | 180–300 | PASS |
| Motor time constant at hover | 25.3 ms | 12–35 | PASS |
| kT / kT₀ | 0.958  | 0.5–2 | PASS |
| Ixx / I₀ | 0.711  | 0.4–2.5 | PASS |
| Iyy / I₀ | 0.952  | 0.4–2.5 | PASS |
| J_rotor / J₀ | 0.843  | 0.3–3 | PASS |
| kQ/kT / (0.0117 m × D/5.1) | 1.24  | 0.5–2 | PASS |
| Rate damping roll | 1.43  | 0–5 | PASS |
| Rate damping pitch | 0.721  | 0–5 | PASS |
| Rate damping yaw | 0.972  | 0–5 | PASS |
| Top speed (model) | 112 km/h | 100–150 | PASS |
| Pack resistance (battery_r × R_m / cells) | 8.95 mΩ/cell | 8–25 | PASS |

## Sessions
| Session | Firmware | Armed | Used | ACC | vbat | unfilt | Reason |
|---|---|---|---|---|---|---|---|
| mini_03_freestyle#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32H743 | 232 s | yes | yes | yes | yes |  |
| mini_04_freestyle_chirp5#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32H743 | 250 s | yes | yes | yes | yes |  |
| mini_05_crash#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32H743 | 6 s | yes | yes | yes | yes |  |
| mini_08_freestyle#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32H743 | 194 s | yes | yes | yes | yes |  |
| mini_09_freestyle#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32H743 | 164 s | yes | yes | yes | yes |  |
| mini_10_freestyle#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32H743 | 219 s | yes | yes | yes | yes |  |

## Sessions and segments
| Session | Duration | Usable | Source | Chirps | Auto vs. hand (IoU) |
|---|---|---|---|---|---|
| mini_03_freestyle#1 | 232 s | 230 s | segments.json | 0 | 1.000 (auto only 0 s, hand only 0.1 s; chirps 0/0) |
| mini_04_freestyle_chirp5#1 | 250 s | 248 s | segments.json | 5 | 1.000 (auto only 0 s, hand only 0 s; chirps 5/5) |
| mini_05_crash#1 | 6 s | 4 s | segments.json | 0 | 0.899 (auto only 0.5 s, hand only 0 s; chirps 0/0) |
| mini_08_freestyle#1 | 193 s | 188 s | segments.json | 0 | 1.000 (auto only 0 s, hand only 0 s; chirps 0/0) |
| mini_09_freestyle#1 | 164 s | 162 s | segments.json | 0 | 1.000 (auto only 0 s, hand only 0 s; chirps 0/0) |
| mini_10_freestyle#1 | 219 s | 217 s | segments.json | 0 | 1.000 (auto only 0 s, hand only 0 s; chirps 0/0) |

Automatic segmentation vs. the hand-checked windows: time-weighted IoU 1.000.

## Betaflight configuration (fc.json)
From the header of **mini_10_freestyle#1**: Betaflight 2026.6.2 (e0b7bb01b) STM32H743, PID loop 8000 Hz (looptime 125 µs, pid_process_denom 1), RX 251 Hz, PID R/P/Y [42, 63, 33]/[49, 71, 44]/[65, 104, 0].

## Motor/ESC
Model per motor: dω/dt = a·(u·V − ω/Kv) − b·ω² − c, 1 ms dead time.
- Equation-error fit (R² 0.403): a [5156, 5001, 5390, 4894], b [0.001592, 0.00165, 0.001785, 0.001702], c [-332, -628, -717, -296]
- Dynamics refinement λ = 2.16 (a, b, c scaled together, steady state unchanged): free run 12.3 → 6.14 Hz RMS
- Final: a [1.113e+04, 1.08e+04, 1.164e+04, 1.057e+04], b [0.003437, 0.003563, 0.003855, 0.003676], c [-717, -1.36e+03, -1.55e+03, -640]
- Response measured/simulated (roll+pitch differential) at 5 Hz 1.1, 10 Hz 1.1, 20 Hz 1.2, 30 Hz 1.2
- Voltage: vbat × 0.9979
- Free run per session [Hz RMS]: mini_03_freestyle#1 5.8, mini_04_freestyle_chirp5#1 6.4, mini_05_crash#1 9.5, mini_08_freestyle#1 5.6, mini_09_freestyle#1 6.8, mini_10_freestyle#1 6.1

## Hover and thrust
Hover from ACC (|a| = 1 g, level, quiet), 22.4 s: rotor 234.1 Hz, motor command 0.303, throttle 0.305.
kT = m·g / Σω² = 3.401e-07 N/(rad/s)² (mass 0.3 kg from the form).
Per session: mini_03_freestyle#1 235.2 Hz (2.5 s), mini_04_freestyle_chirp5#1 232.3 Hz (17.7 s), mini_10_freestyle#1 236.3 Hz (2.2 s)
The no-ACC rule on the ACC sessions (Hz), as a check of the fallback: mini_03_freestyle#1 233.8, mini_04_freestyle_chirp5#1 235.6, mini_08_freestyle#1 339.2, mini_09_freestyle#1 238.2, mini_10_freestyle#1 288.1

## Rotational dynamics
Instrumental-variable fit on chirps (96 s of chirp in the logs), band 1–30 Hz.
| Axis | Gain | Damping | R² | Rel. std. error (jackknife over windows) |
|---|---|---|---|---|
| roll | 60.44 | -1.43 | 0.9 | 0.59 % |
| pitch | 45.13 | -0.721 | 0.97 | 4.6 % |
| yaw | -1.522 | -0.972 | 0.95 | 41 % |
Roll/pitch: rad/s² per 10⁶ (rad/s)² → Ixx 0.000309, Iyy 0.000414 kg·m² (with kT and arm).
Yaw: −J_rotor/Izz (×10⁻³) → J_rotor 9.35e-07 kg·m², kQ = J_rotor·b = 3.4e-09; yaw_motors_reversed = ON (sign from the data).
Rate damping: [1.43, 0.721, 0.972] (from the chirp fit).

## Trim / imbalance
From the rotor speed differences at hover (22769 samples): β = [0.706, -3.61, 1.09] ×10⁻³ m (torque = β · total thrust; roll, pitch, yaw).
Per session (×10⁻³): mini_03_freestyle#1 [0.53, -2.92, 0.39], mini_04_freestyle_chirp5#1 [0.85, -3.85, 1.31], mini_10_freestyle#1 [-0.08, -2.69, 0.33]

## Battery
Source: vbat. V = V0 − R·I (I = the sim's current proxy). V0 per session: {'mini_03_freestyle#1': 15.63, 'mini_04_freestyle_chirp5#1': 15.47, 'mini_05_crash#1': 15.94, 'mini_08_freestyle#1': 15.43, 'mini_09_freestyle#1': 15.56, 'mini_10_freestyle#1': 15.57}.
battery_v0 = 15.94 V (freshest pack), battery_r = 0.141, discharge fitted 0.315 V/min (exported as 0: the game simulates only the sag), RMS 0.15 V.

## Drag and top speed
ACC dead reckoning: 8 passes from hover (each up to 5 s, start velocity and tilt fitted too: RMS 1 m/s, 11°), 111 blocks of 0.25 s, speed p50/p90/p99 [3.9, 18.5, 22.1] m/s. Fit R² 0.86: rotor drag 0.36 1/s (±7.2 %), body scale 0.02 (±1e+03 %).
Used: rotor drag from the ACC fit (1.85e-05); body drag class default (reference × (D/5.1")²): cd_area [0.00461, 0.00552, 0.0139] m².
Top speed of the model (full throttle, level, fresh pack with sag): **112 km/h** at 77° tilt, 705 Hz rotor; no top speed in the form.

## Propwash / turbulence
Log metric (15–78 Hz roll/pitch tracking error, deg/s, median): calm 0.77, after throttle chops 0.9, flips after chops 2.71; 99th percentile of all usable windows outside chirps 10.3 (windows (2865, 595, 62, 7595)). The targets of the prop wash calibration (below).
Wake/turbulence amplitudes of the Python plant (calm-air turbulence before the calibration): reference values scaled with the induced velocity (factor 0.99): turb_base_ms 0.307, pw_turb_ms 3.76, wake_r0 0.0824 m.

## Validation
Hover: motor command sim 0.309 / log 0.303, rotor 234 / 234 Hz (model voltage 14.45 V).
Roll steps (10, stick replay through the Betaflight port + model): t90 sim 194.4 ms / log 195.3 ms, lag behind the setpoint (at 50 %) sim 2.0 ms / log 3.0 ms, gyro deviation sim−log 0.03 × log RMS (median).
Events (session, time, plateau deg/s, t90 log/sim ms): mini_03_freestyle#1 27.52 s 700 193/190; mini_03_freestyle#1 156.7 s 700 128/122; mini_10_freestyle#1 186.96 s 700 110/107; mini_10_freestyle#1 187.93 s 700 120/121; mini_08_freestyle#1 21.55 s 662 199/238; mini_04_freestyle_chirp5#1 50.59 s 587 199/199; mini_09_freestyle#1 5.68 s 525 137/139; mini_08_freestyle#1 27.02 s 510 198/211; mini_04_freestyle_chirp5#1 4.92 s 508 216/223; mini_09_freestyle#1 154.32 s 487 221/223

## Prop wash calibration
- Prop wash calibrated: drop and catch 13.18 °/s (log 13.22), calm 0.77 °/s (log 0.77)
- dirty_air 1.1, wash and wake 1 (its own figures); scenarios flown in its air without the game (no map, 60 m up), mean of 32 seeds, 6 runs, 20.4 s (real_quad logfit/fit/calibrate.gd; replaces the hand calibration).
- Drop and catch: 0.9 s at zero throttle from a hover, caught at 45 %; its worst 0.25-s window against 1.283 × the logs' 99th percentile 10.3 (freestyle5's scale: 14.5 deg/s at 11.3).
- Calm: 2 s of hover; its median 0.25-s window against the logs' calm median.
