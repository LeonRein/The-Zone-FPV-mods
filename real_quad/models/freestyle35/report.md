# 3.5" freestyle, 300 g, 4S (freestyle35)

## Grade A: flies like your quad

- all coverage items and critical checks met

## What the logs cover
- Chirp sweeps found: roll: 6 full sweeps; pitch: 5 full sweeps, 1 aborted after 14 s (needs ≥ 16 s); yaw: 4 full sweeps (the grade A wants 2 roll, 2 pitch, 1 yaw, and 30 s of sweeps in all).
- Drag: rotor drag measured from the accelerometer; body drag / top speed not measurable from these logs (needs long fast straight passes; above ~85 % throttle the accelerometer is unusable from vibration) → class default. Enter the top speed in the model's parameters to set it (Models… → the model → Parameters → Top speed).

## What to fly next
- [ ] Enter the quad's top speed in the model's parameters (Models… → the model → Parameters → Top speed): the body drag is a class default

Made by `fpvident` from 9 sessions in 117 s. 17 PASS, 1 WARN.

Form: 0.3 kg, 4S lipo, 3650 KV, prop 3.5" × 2.5" × 3, class mini, top speed unknown km/h.

## Checks
| Check | Result | Detail |
|---|---|---|
| Segmentation auto = hand | PASS | IoU 1.000 (threshold 0.95) |
| Motor dynamics 5–30 Hz | PASS | measured/sim [1.1, 1.1, 1.1, 1.2] (0.8–1.2) |
| Motor KV (vbat scale) | PASS | scale 0.997 (0.8–1.25) |
| Hover detection | PASS | acc, 77.4 s |
| Hover rotor speed between sessions | PASS | spread 2.8 % (threshold 6 %) |
| Motor model free run | PASS | 6.99 Hz RMS (threshold max(12, 7.5 % of 231 Hz) = 17.3) |
| Rotation roll/pitch | PASS | chirp, R² ≥ 0.9, rel. std. error ≤ 2.7 % (threshold 10 %) |
| Rotation yaw | PASS | R² 0.96, rel. std. error 2.1 % (threshold 25 %) |
| Torque path 10–50 Hz | PASS | measured/sim roll 1.01–1.10, pitch 0.99–1.08 (0.85–1.15), phase 20–50 Hz -3.0…+2.5° (±7°) |
| Imbalance consistent | PASS | spread between sessions [0.35, 0.51, 0.55] ×10⁻³ |
| Imbalance size | PASS | |β| 3.71 ×10⁻³ m (≤ 0.1 · arm = 5.5) |
| Battery fit | PASS | vbat, RMS 0.17 V |
| ACC rotor drag | PASS | R² 0.77, uncertainty 0.049 (thresholds 0.5 / 0.15) |
| Top speed backed | WARN | neither a given top speed nor an ACC fit: 115 km/h is an estimate |
| Hover throttle sim vs. log | PASS | 0.300 vs. 0.295 (±0.02) |
| Roll t90 sim vs. log | PASS | 182.5 vs. 183.7 ms (±20 %) |
| Roll lag sim vs. log | PASS | 2.1 vs. 2.5 ms (±25 %, at least 2 ms) |
| Closed-loop tracking | PASS | 0.03 (threshold 0.35) |

## Measured
| Quantity | Value | Source |
|---|---|---|
| Motor/ESC a, b, c (per motor) | see Motor/ESC | eRPM vs. DShot command, speed and dead time from the gyro |
| Thrust coefficient kT | 3.497e-07 | hover 231 Hz at 0.3 kg (acc) |
| Roll/pitch torque gain (→ Ixx, Iyy) | 49.7 / 38 | IV, chirp |
| Yaw: J_rotor/Izz, kQ, direction | 0.00124 | IV, motor torque as regressor |
| Torque imbalance β | [0.69, -3.7, 1.4] ×10⁻³ m | eRPM differences at hover |
| Battery V0, sag R | 15.95 V, 0.128 | vbat |
| Rotor drag (linear) | 0.33 1/s | ACC dead reckoning |
| Own wake's turbulence | 3.97 m/s | drop and catch 14.87 deg/s (target 14.78) |
| Calm-air turbulence | 0.243 m/s | calm hover 0.79 deg/s (target 0.79) |

## Class defaults and assumptions
| Quantity | Value | Why |
|---|---|---|
| Motor distance (arm) | 55 mm | 80 mm × D/5.1" (only the absolute inertias depend on it) |
| Izz | 0.000758 kg·m² | 1.7 × mean roll/pitch inertia (only J_rotor/Izz is measured) |
| Body drag | cd_area [0.00461, 0.00552, 0.0139] | reference × (D/5.1")² |
| Turbulence | turb 0.307 / 3.76 m/s | reference scaled physically (calibrated in the game) |
| dirty_air wash/wake | 1/1 | dirty_air's own figures |

## Plausibility vs. the mini class band
| Quantity | Value | Band | Result |
|---|---|---|---|
| Weight | 300 g | 180–400 | PASS |
| Cells | 4 S | 4–6 | PASS |
| Hover rotor speed | 231 Hz | 180–300 | PASS |
| Motor time constant at hover | 18.7 ms | 12–35 | PASS |
| kT / kT₀ | 0.985  | 0.5–2 | PASS |
| Ixx / I₀ | 0.888  | 0.4–2.5 | PASS |
| Iyy / I₀ | 1.16  | 0.4–2.5 | PASS |
| J_rotor / J₀ | 0.844  | 0.3–3 | PASS |
| kQ/kT / (0.0117 m × D/5.1) | 1.62  | 0.5–2 | PASS |
| Rate damping roll | 0.37  | 0–5 | PASS |
| Rate damping pitch | 0  | 0–5 | PASS |
| Rate damping yaw | 0.769  | 0–5 | PASS |
| Top speed (model) | 115 km/h | 100–150 | PASS |
| Pack resistance (battery_r × R_m / cells) | 5.92 mΩ/cell | 8–25 | WARN |

## Sessions
| Session | Firmware | Armed | Used | ACC | vbat | unfilt | Reason |
|---|---|---|---|---|---|---|---|
| LOG00001#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32H743 | 389 s | yes | yes | yes | yes |  |
| LOG00002#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32H743 | 175 s | yes | yes | yes | yes |  |
| LOG00003#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32H743 | 152 s | yes | yes | yes | yes |  |
| mini_03_freestyle#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32H743 | 232 s | yes | yes | yes | yes |  |
| mini_04_freestyle_chirp5#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32H743 | 250 s | yes | yes | yes | yes |  |
| mini_05_crash#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32H743 | 6 s | yes | yes | yes | yes |  |
| mini_08_freestyle#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32H743 | 194 s | yes | yes | yes | yes |  |
| mini_09_freestyle#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32H743 | 164 s | yes | yes | yes | yes |  |
| mini_10_freestyle#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32H743 | 219 s | yes | yes | yes | yes |  |

## Warnings and notes
- warning: LOG00001#1: ACC not level on the ground (x 0.10 g, y 0.10 g): FC mounted tilted or ACC not calibrated (fix: calibrate the accelerometer on a level surface)
- warning: LOG00001#1: 2.9 % corrupt or skipped frames (fix: use a faster SD card or the onboard flash; erase the flash before flying)
- warning: Pack resistance (battery_r × R_m / cells) 5.92 mΩ/cell is outside the mini class band 8–25 (can be right for an unusual build: check)

## Sessions and segments
| Session | Duration | Usable | Source | Chirps | Auto vs. hand (IoU) |
|---|---|---|---|---|---|
| LOG00001#1 | 424 s | 360 s | auto | 11 | – |
| LOG00002#1 | 175 s | 173 s | auto | 0 | – |
| LOG00003#1 | 152 s | 150 s | auto | 0 | – |
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
Model per motor: dω/dt = a·(u·V − ω/Kv) − b·ω² − c, 1.4 ms dead time.
- Equation-error fit (R² 0.47): a [5638, 5712, 5776, 5614], b [0.001693, 0.00184, 0.001866, 0.001916], c [-309, -728, -740, -350]
- Dynamics refinement λ = 1.98 (a, b, c scaled together, steady state unchanged): free run 11.4 → 6.99 Hz RMS
- Torque path (chirps: the free run's ω² against the gyro, instrumented by the setpoint, 5–60 Hz; gain, speed and dead time in one fit weighted by the coherence): a, b, c × 1.34, dead time 1.0 → 1.4 ms; time constant at the chirps 24.9 → 18.5 ms (the eRPM telemetry lags the rotors: the eRPM sets the steady state, the gyro the speed)
- Final: a [1.498e+04, 1.518e+04, 1.535e+04, 1.492e+04], b [0.004498, 0.004889, 0.004959, 0.005091], c [-821, -1.93e+03, -1.97e+03, -931]
- Response measured/simulated (roll+pitch differential, eRPM fit) at 5 Hz 1.1, 10 Hz 1.1, 20 Hz 1.1, 30 Hz 1.2
- Voltage: vbat × 0.9966
- Free run per session (eRPM fit) [Hz RMS]: LOG00001#1 3.0, LOG00002#1 7.2, LOG00003#1 10.9, mini_03_freestyle#1 5.9, mini_04_freestyle_chirp5#1 6.2, mini_05_crash#1 8.7, mini_08_freestyle#1 5.8, mini_09_freestyle#1 7.2, mini_10_freestyle#1 6.2

## Hover and thrust
Hover from ACC (|a| = 1 g, level, quiet), 77.4 s: rotor 230.9 Hz, motor command 0.295, throttle 0.297.
kT = m·g / Σω² = 3.497e-07 N/(rad/s)² (mass 0.3 kg from the form).
Per session: LOG00001#1 229.8 Hz (55.0 s), mini_03_freestyle#1 235.2 Hz (2.5 s), mini_04_freestyle_chirp5#1 232.3 Hz (17.7 s), mini_10_freestyle#1 236.3 Hz (2.2 s)
The no-ACC rule on the ACC sessions (Hz), as a check of the fallback: LOG00001#1 234.2, LOG00002#1 429.1, LOG00003#1 330.7, mini_03_freestyle#1 233.8, mini_04_freestyle_chirp5#1 235.6, mini_08_freestyle#1 339.2, mini_09_freestyle#1 238.2, mini_10_freestyle#1 288.1

## Rotational dynamics
Instrumental-variable fit on chirps (320 s of chirp in the logs), band 1–30 Hz.
| Axis | Gain | Damping | R² | Rel. std. error (jackknife over windows) |
|---|---|---|---|---|
| roll | 49.73 | -0.37 | 0.9 | 2.4 % |
| pitch | 37.96 | 0.435 | 0.97 | 2.7 % |
| yaw | -1.235 | -0.769 | 0.96 | 2.1 % |
Roll/pitch: rad/s² per 10⁶ (rad/s)² → Ixx 0.000386, Iyy 0.000506 kg·m² (with kT and arm).
Yaw: −J_rotor/Izz (×10⁻³) → J_rotor 9.36e-07 kg·m², kQ = J_rotor·b = 4.55e-09; yaw_motors_reversed = ON (sign from the data).
Rate damping: [0.37, 0, 0.769] (from the chirp fit).
Torque path measured/sim (gain × free run vs. gyro, 10–50 Hz): roll 1.01, 1.02, 1.02, 1.04, 1.07, 1.08, 1.10, 1.10, 1.10 (phase 20–50 Hz +1.1, +2.1, +2.1, +2.5, +2.0, +1.8, +1.9°); pitch 1.03, 1.03, 1.02, 1.02, 0.99, 1.00, 1.03, 1.04, 1.08 (phase 20–50 Hz -3.0, -2.0, -2.4, -1.7, -2.3, -1.4, -1.9°)

## Trim / imbalance
From the rotor speed differences at hover (78704 samples): β = [0.694, -3.71, 1.37] ×10⁻³ m (torque = β · total thrust; roll, pitch, yaw).
Per session (×10⁻³): LOG00001#1 [0.69, -3.75, 1.35], mini_03_freestyle#1 [0.53, -2.92, 0.51], mini_04_freestyle_chirp5#1 [0.85, -3.85, 1.71], mini_10_freestyle#1 [-0.08, -2.69, 0.43]

## Battery
Source: vbat. V = V0 − R·I (I = the sim's current proxy). V0 per session: {'LOG00001#1': 15.95, 'LOG00002#1': 15.4, 'LOG00003#1': 15.53, 'mini_03_freestyle#1': 15.49, 'mini_04_freestyle_chirp5#1': 15.32, 'mini_05_crash#1': 15.87, 'mini_08_freestyle#1': 15.29, 'mini_09_freestyle#1': 15.43, 'mini_10_freestyle#1': 15.43}.
battery_v0 = 15.95 V (freshest pack), battery_r = 0.128, discharge fitted 0.265 V/min (not exported: the game simulates only the sag), RMS 0.17 V.

## Drag and top speed
ACC dead reckoning: 16 passes from hover (each up to 5 s, start velocity and tilt fitted too: RMS 2.1 m/s, 14°), 244 blocks of 0.25 s, speed p50/p90/p99 [8.3, 19.6, 23.6] m/s. Fit R² 0.77: rotor drag 0.33 1/s (±4.9 %), body scale 0.055 (±2.3e+02 %).
Used: rotor drag from the ACC fit (1.72e-05); body drag class default (reference × (D/5.1")²): cd_area [0.00461, 0.00552, 0.0139] m².
Top speed of the model (full throttle, level, fresh pack with sag): **115 km/h** at 77° tilt, 718 Hz rotor; no top speed in the form.

## Propwash / turbulence
Log metric (15–78 Hz roll/pitch tracking error, deg/s, median): calm 0.79, after throttle chops 0.95, flips after chops 3.87; 99th percentile of all usable windows outside chirps 11.52 (windows (4962, 898, 144, 10940)). Targets of the prop wash calibration in the game (real_quad: drop and catch = 1.283 × p99, calm = the calm median).
Turbulence amplitudes (class values): reference values scaled with the induced velocity (factor 0.99): turb_base_ms 0.307, pw_turb_ms 3.76.

## Validation
Hover: motor command sim 0.300 / log 0.295, rotor 231 / 231 Hz (model voltage 14.61 V).
Roll steps (12, stick replay through the Betaflight port + model): t90 sim 182.5 ms / log 183.7 ms, lag behind the setpoint (at 50 %) sim 2.1 ms / log 2.5 ms, gyro deviation sim−log 0.03 × log RMS (median).
Events (session, time, plateau deg/s, t90 log/sim ms): mini_03_freestyle#1 27.52 s 700 192/190; mini_03_freestyle#1 156.7 s 700 127/122; LOG00002#1 56.81 s 700 297/305; LOG00002#1 90.96 s 700 198/199; mini_10_freestyle#1 186.96 s 700 110/108; mini_10_freestyle#1 187.93 s 700 120/120; LOG00003#1 33.32 s 700 175/175; LOG00003#1 35.89 s 700 75/74; mini_08_freestyle#1 21.55 s 662 199/237; mini_04_freestyle_chirp5#1 50.59 s 587 198/199; mini_09_freestyle#1 5.68 s 525 136/137; mini_08_freestyle#1 27.02 s 510 197/209

## Prop wash calibration
- Prop wash calibrated: drop and catch 14.9 °/s (log 14.8), calm 0.79 °/s (log 0.79)
- dirty_air 1.1, wash and wake 1 (its own figures); scenarios flown in its air without the game (no map, 60 m up), mean of 32 seeds, 4 runs, 28.2 s.
- Drop and catch: 0.9 s at zero throttle from a hover, caught at 45 %; its worst 0.25-s window against 1.283 × the logs' 99th percentile (freestyle5's scale: 14.5 deg/s at 11.3).
- Calm: 2 s of hover; its median 0.25-s window against the logs' calm median.
