# 5" freestyle, 650 g, 6S (freestyle5)

## Grade A: flies like your quad

- all coverage items and critical checks met

## What the logs cover
- Chirp sweeps found: roll: 5 full sweeps; pitch: 5 full sweeps; yaw: 5 full sweeps (the grade A wants 2 roll, 2 pitch, 1 yaw, and 30 s of sweeps in all).
- Drag: rotor drag measured from the accelerometer; body drag set from the top speed given (165 km/h), the accelerometer cannot measure it from these logs.

Made by `fpvident` from 10 sessions in 124 s. 17 PASS, 1 WARN.

Form: 0.65 kg, 6S lipo, 1855 KV, prop 5.1" × 4.99" × 3, class five, top speed 165 km/h.

## Checks
| Check | Result | Detail |
|---|---|---|
| Segmentation auto = hand | PASS | IoU 0.998 (threshold 0.95) |
| Motor dynamics 5–30 Hz | PASS | measured/sim [1.1, 1.1, 1.1, 1.1] (0.8–1.2) |
| Motor KV (vbat scale) | PASS | scale 1.049 (0.8–1.25) |
| Hover detection | PASS | acc, 51.2 s |
| Hover rotor speed between sessions | PASS | spread 3.9 % (threshold 6 %) |
| Motor model free run | PASS | 6.07 Hz RMS (threshold max(12, 7.5 % of 158 Hz) = 12.0) |
| Rotation roll/pitch | PASS | chirp, R² ≥ 0.96, rel. std. error ≤ 4.9 % (threshold 10 %) |
| Rotation yaw | PASS | R² 0.96, rel. std. error 1.3 % (threshold 25 %) |
| Torque path 10–50 Hz | PASS | measured/sim roll 0.99–1.08, pitch 1.05–1.13 (0.85–1.15), phase 20–50 Hz -6.0…+6.0° (±7°) |
| Imbalance consistent | WARN | spread between sessions [1.5, 2.8, 0.55] ×10⁻³ |
| Imbalance size | PASS | |β| 1.10 ×10⁻³ m (≤ 0.1 · arm = 8.0) |
| Battery fit | PASS | vbat, RMS 0.22 V |
| ACC rotor drag | PASS | R² 0.9, uncertainty 0.044 (thresholds 0.5 / 0.15) |
| Top speed backed | PASS | calibrated to the given 165 km/h |
| Hover throttle sim vs. log | PASS | 0.238 vs. 0.245 (±0.02) |
| Roll t90 sim vs. log | PASS | 141.9 vs. 140.9 ms (±20 %) |
| Roll lag sim vs. log | PASS | 4.5 vs. 6.3 ms (±25 %, at least 2 ms) |
| Closed-loop tracking | PASS | 0.04 (threshold 0.35) |

## Measured
| Quantity | Value | Source |
|---|---|---|
| Motor/ESC a, b, c (per motor) | see Motor/ESC | eRPM vs. DShot command, speed and dead time from the gyro |
| Thrust coefficient kT | 1.621e-06 | hover 158 Hz at 0.65 kg (acc) |
| Roll/pitch torque gain (→ Ixx, Iyy) | 67.3 / 58.1 | IV, chirp |
| Yaw: J_rotor/Izz, kQ, direction | 0.00178 | IV, motor torque as regressor |
| Torque imbalance β | [0.51, -1.1, -0.0086] ×10⁻³ m | eRPM differences at hover |
| Battery V0, sag R | 25.39 V, 0.134 | vbat |
| Rotor drag (linear) | 0.29 1/s | ACC dead reckoning |
| Body drag | cd_area [0.00847, 0.0101, 0.0255] | given top speed 165 km/h |
| dirty_air gust | 6.16 | drop and catch 14.57 deg/s (target 14.55) |
| Calm-air turbulence | 0.284 m/s | calm hover 0.75 deg/s (target 0.75) |

## Class defaults and assumptions
| Quantity | Value | Why |
|---|---|---|
| Motor distance (arm) | 80 mm | 80 mm × D/5.1" (only the absolute inertias depend on it) |
| Izz | 0.00353 kg·m² | 1.7 × mean roll/pitch inertia (only J_rotor/Izz is measured) |
| dirty_air wash/wake/gust | 1.0/1.0/6.79 | class values, calibrated in the game: five class |
| Wake/turbulence/VRS | turb 0.31 / 3.8 m/s | reference scaled physically (Python plant only) |
| dirty_air wash/wake | 1/1 | dirty_air's own figures |

## Plausibility vs. the five class band
| Quantity | Value | Band | Result |
|---|---|---|---|
| Weight | 650 g | 450–850 | PASS |
| Cells | 6 S | 4–6 | PASS |
| Hover rotor speed | 158 Hz | 130–220 | PASS |
| Motor time constant at hover | 22 ms | 18–40 | PASS |
| kT / kT₀ | 1.01  | 0.5–2 | PASS |
| Ixx / I₀ | 0.964  | 0.4–2.5 | PASS |
| Iyy / I₀ | 1.12  | 0.4–2.5 | PASS |
| J_rotor / J₀ | 1.26  | 0.3–3 | PASS |
| kQ/kT / (0.0117 m × D/5.1) | 1.52  | 0.5–2 | PASS |
| Rate damping roll | 0.223  | 0–5 | PASS |
| Rate damping pitch | 0  | 0–5 | PASS |
| Rate damping yaw | 0  | 0–5 | PASS |
| Top speed (model) | 165 km/h | 130–190 | PASS |
| Pack resistance (battery_r × R_m / cells) | 2.58 mΩ/cell | 2–12 | PASS |

## Sessions
| Session | Firmware | Armed | Used | ACC | vbat | unfilt | Reason |
|---|---|---|---|---|---|---|---|
| btfl_001#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32F405 | 111 s | yes | no | no | yes |  |
| btfl_011#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32F405 | 108 s | yes | no | no | yes |  |
| btfl_012#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32F405 | 186 s | yes | no | no | yes |  |
| m5_01_chirp6#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32F405 | 321 s | yes | yes | yes | yes |  |
| m5_02_crash#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32F405 | 35 s | yes | yes | yes | yes |  |
| m5_03_hover#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32F405 | 27 s | yes | yes | yes | yes |  |
| m5_04_freestyle#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32F405 | 119 s | yes | yes | yes | yes |  |
| m5_05_freestyle#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32F405 | 59 s | yes | yes | yes | yes |  |
| m5_06_freestyle_abgeschnitten#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32F405 | 39 s | yes | yes | yes | yes |  |
| m5_07_chirp9#1 | Betaflight 2026.6.2 (e0b7bb01b) STM32F405 | 387 s | yes | yes | yes | yes |  |

## Warnings and notes
- warning: m5_01_chirp6#1: ACC not level on the ground (x 0.13 g, y 0.01 g): FC mounted tilted or ACC not calibrated (fix: calibrate the accelerometer on a level surface)
- warning: m5_02_crash#1: ACC not level on the ground (x 0.01 g, y 0.16 g): FC mounted tilted or ACC not calibrated (fix: calibrate the accelerometer on a level surface)
- warning: m5_04_freestyle#1: ACC not level on the ground (x 0.17 g, y 0.01 g): FC mounted tilted or ACC not calibrated (fix: calibrate the accelerometer on a level surface)
- warning: m5_05_freestyle#1: ACC not level on the ground (x 0.11 g, y 0.02 g): FC mounted tilted or ACC not calibrated (fix: calibrate the accelerometer on a level surface)
- warning: m5_06_freestyle_abgeschnitten#1: ACC not level on the ground (x 0.13 g, y 0.02 g): FC mounted tilted or ACC not calibrated (fix: calibrate the accelerometer on a level surface)
- warning: m5_07_chirp9#1: ACC not level on the ground (x 0.15 g, y 0.05 g): FC mounted tilted or ACC not calibrated (fix: calibrate the accelerometer on a level surface)
- info: no usable ACC in btfl_001#1, btfl_011#1, btfl_012#1 (hover and drag use the ACC sessions) (fix: set blackbox_disable_acc = OFF; ACC must be on (acc_hardware not NONE) and calibrated)
- info: no usable battery voltage in btfl_001#1, btfl_011#1, btfl_012#1 (motor voltage estimated per 2-s window there) (fix: set blackbox_disable_bat = OFF; calibrate the voltage sensor (vbat_scale))

## Sessions and segments
| Session | Duration | Usable | Source | Chirps | Auto vs. hand (IoU) |
|---|---|---|---|---|---|
| btfl_001#1 | 111 s | 109 s | auto | 0 | – |
| btfl_011#1 | 107 s | 105 s | auto | 0 | – |
| btfl_012#1 | 186 s | 184 s | auto | 0 | – |
| m5_01_chirp6#1 | 321 s | 318 s | segments.json | 6 | 0.996 (auto only 0.1 s, hand only 1 s; chirps 6/6) |
| m5_02_crash#1 | 35 s | 31 s | segments.json | 0 | 0.998 (auto only 0 s, hand only 0.1 s; chirps 0/0) |
| m5_03_hover#1 | 27 s | 21 s | segments.json | 0 | 0.996 (auto only 0 s, hand only 0.1 s; chirps 0/0) |
| m5_04_freestyle#1 | 119 s | 114 s | segments.json | 0 | 0.998 (auto only 0 s, hand only 0.2 s; chirps 0/0) |
| m5_05_freestyle#1 | 59 s | 56 s | segments.json | 0 | 0.995 (auto only 0 s, hand only 0.2 s; chirps 0/0) |
| m5_06_freestyle_abgeschnitten#1 | 39 s | 37 s | segments.json | 0 | 0.988 (auto only 0.2 s, hand only 0.2 s; chirps 0/0) |
| m5_07_chirp9#1 | 387 s | 384 s | segments.json | 9 | 1.000 (auto only 0 s, hand only 0.1 s; chirps 9/9) |

Automatic segmentation vs. the hand-checked windows: time-weighted IoU 0.998.

## Betaflight configuration (fc.json)
From the header of **m5_07_chirp9#1**: Betaflight 2026.6.2 (e0b7bb01b) STM32F405, PID loop 4000 Hz (looptime 125 µs, pid_process_denom 2), RX 250 Hz, PID R/P/Y [45, 55, 33]/[42, 58, 37]/[50, 80, 0].

## Motor/ESC
Model per motor: dω/dt = a·(u·V − ω/Kv) − b·ω² − c, 1.3 ms dead time.
- Equation-error fit (R² 0.716): a [3762, 3994, 3959, 3823], b [0.002496, 0.002487, 0.002558, 0.002529], c [-326, -214, -177, -251]
- Dynamics refinement λ = 1.28 (a, b, c scaled together, steady state unchanged): free run 6.64 → 6.07 Hz RMS
- Torque path (chirps: the free run's ω² against the gyro, instrumented by the setpoint, 5–60 Hz; gain, speed and dead time in one fit weighted by the coherence): a, b, c × 1.42, dead time 1.0 → 1.3 ms; time constant at the chirps 30.6 → 21.6 ms (the eRPM telemetry lags the rotors: the eRPM sets the steady state, the gyro the speed)
- Final: a [6843, 7265, 7201, 6954], b [0.004541, 0.004525, 0.004652, 0.0046], c [-592, -389, -323, -457]
- Response measured/simulated (roll+pitch differential, eRPM fit) at 5 Hz 1.1, 10 Hz 1.1, 20 Hz 1.1, 30 Hz 1.1
- Voltage: vbat × 1.049
- Free run per session (eRPM fit) [Hz RMS]: btfl_001#1 7.1, btfl_011#1 4.8, btfl_012#1 8.9, m5_01_chirp6#1 6.7, m5_02_crash#1 8.0, m5_03_hover#1 2.4, m5_04_freestyle#1 5.5, m5_05_freestyle#1 4.1, m5_06_freestyle_abgeschnitten#1 4.2, m5_07_chirp9#1 4.5

## Hover and thrust
Hover from ACC (|a| = 1 g, level, quiet), 51.2 s: rotor 157.8 Hz, motor command 0.245, throttle 0.245.
kT = m·g / Σω² = 1.621e-06 N/(rad/s)² (mass 0.65 kg from the form).
Per session: m5_01_chirp6#1 153.7 Hz (11.9 s), m5_03_hover#1 156.0 Hz (4.7 s), m5_04_freestyle#1 159.8 Hz (4.9 s), m5_07_chirp9#1 159.3 Hz (29.7 s)
The no-ACC rule on the ACC sessions (Hz), as a check of the fallback: m5_01_chirp6#1 175.5, m5_03_hover#1 154.7, m5_04_freestyle#1 149.3, m5_05_freestyle#1 160.7, m5_07_chirp9#1 161.3

## Rotational dynamics
Instrumental-variable fit on chirps (307 s of chirp in the logs), band 1–30 Hz.
| Axis | Gain | Damping | R² | Rel. std. error (jackknife over windows) |
|---|---|---|---|---|
| roll | 67.29 | -0.223 | 0.96 | 4.9 % |
| pitch | 58.15 | 1.31 | 0.96 | 2 % |
| yaw | -1.779 | 0.331 | 0.96 | 1.3 % |
Roll/pitch: rad/s² per 10⁶ (rad/s)² → Ixx 0.00193, Iyy 0.00223 kg·m² (with kT and arm).
Yaw: −J_rotor/Izz (×10⁻³) → J_rotor 6.29e-06 kg·m², kQ = J_rotor·b = 2.88e-08; yaw_motors_reversed = ON (sign from the data).
Rate damping: [0.223, 0, 0] (from the chirp fit).
Torque path measured/sim (gain × free run vs. gyro, 10–50 Hz): roll 0.99, 1.00, 1.02, 1.02, 1.02, 1.03, 1.05, 1.05, 1.08 (phase 20–50 Hz +1.5, +2.4, +2.8, +3.1, +3.9, +3.7, +6.0°); pitch 1.05, 1.07, 1.07, 1.08, 1.08, 1.09, 1.11, 1.11, 1.13 (phase 20–50 Hz -3.8, -5.1, -4.9, -6.0, -5.3, -4.7, -5.5°)

## Trim / imbalance
From the rotor speed differences at hover (51227 samples): β = [0.512, -1.1, -0.0086] ×10⁻³ m (torque = β · total thrust; roll, pitch, yaw).
Per session (×10⁻³): m5_01_chirp6#1 [0.27, -2.87, 0.42], m5_03_hover#1 [-2.86, 3.84, -0.98], m5_04_freestyle#1 [0.4, 2.35, -0.6], m5_07_chirp9#1 [1.1, -1.76, 0.08]

## Battery
Source: vbat. V = V0 − R·I (I = the sim's current proxy). V0 per session: {'m5_01_chirp6#1': 24.95, 'm5_02_crash#1': 25.39, 'm5_03_hover#1': 25.17, 'm5_04_freestyle#1': 24.44, 'm5_05_freestyle#1': 23.73, 'm5_06_freestyle_abgeschnitten#1': 23.4, 'm5_07_chirp9#1': 25.12}.
battery_v0 = 25.39 V (freshest pack), battery_r = 0.134, discharge fitted 0.345 V/min (exported as 0: the game simulates only the sag), RMS 0.22 V.

## Drag and top speed
ACC dead reckoning: 16 passes from hover (each up to 5 s, start velocity and tilt fitted too: RMS 1.6 m/s, 14°), 268 blocks of 0.25 s, speed p50/p90/p99 [5.2, 22.8, 35.6] m/s. Fit R² 0.9: rotor drag 0.29 1/s (±4.4 %), body scale 0.02 (±4.4e+02 %).
Used: rotor drag from the ACC fit (4.78e-05); body drag calibrated to the given top speed 165 km/h (the ACC fit does not determine it: too few fast passes): cd_area [0.00847, 0.0101, 0.0255] m².
Top speed of the model (full throttle, level, fresh pack with sag): **165 km/h** at 82° tilt, 582 Hz rotor; given 165 km/h; with the default body drag it would be 157 km/h.

## Propwash / turbulence
Log metric (15–78 Hz roll/pitch tracking error, deg/s, median): calm 0.75, after throttle chops 0.67, flips after chops 4.28; 99th percentile of all usable windows outside chirps 11.34 (windows (4125, 849, 96, 8325)). Targets of the prop wash calibration in the game (real_quad: drop and catch = 1.283 × p99, calm = the calm median).
dirty_air wash/wake/gust 1.0/1.0/6.79 from the five class.
Wake/turbulence amplitudes of the Python plant: reference values scaled with the induced velocity (factor 1): turb_base_ms 0.31, pw_turb_ms 3.8, wake_r0 0.12 m.

## Validation
Hover: motor command sim 0.238 / log 0.245, rotor 158 / 158 Hz (model voltage 23.85 V).
Roll steps (12, stick replay through the Betaflight port + model): t90 sim 141.9 ms / log 140.9 ms, lag behind the setpoint (at 50 %) sim 4.5 ms / log 6.3 ms, gyro deviation sim−log 0.04 × log RMS (median).
Events (session, time, plateau deg/s, t90 log/sim ms): m5_05_freestyle#1 18.24 s 700 188/192; m5_05_freestyle#1 22.13 s 700 126/124; btfl_012#1 44.13 s 700 177/182; btfl_012#1 68.07 s 700 108/102; m5_04_freestyle#1 53.09 s 700 99/99; m5_04_freestyle#1 60.1 s 700 150/151; m5_06_freestyle_abgeschnitten#1 23.79 s 700 125/127; m5_07_chirp9#1 11.25 s 700 132/133; m5_07_chirp9#1 31.31 s 700 104/102; m5_06_freestyle_abgeschnitten#1 13.81 s 662 230/246; btfl_011#1 71.94 s 633 357/361; btfl_011#1 8.63 s 621 218/219

## Prop wash calibration
- Prop wash calibrated: drop and catch 14.6 °/s (log 14.6), calm 0.75 °/s (log 0.75)
- dirty_air 1.1, wash and wake 1 (its own figures); scenarios flown in its air without the game (no map, 60 m up), mean of 32 seeds, 5 runs, 26.9 s.
- Drop and catch: 0.9 s at zero throttle from a hover, caught at 45 %; its worst 0.25-s window against 1.283 × the logs' 99th percentile (freestyle5's scale: 14.5 deg/s at 11.3).
- Calm: 2 s of hover; its median 0.25-s window against the logs' calm median.
