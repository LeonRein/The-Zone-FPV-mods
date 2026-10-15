# 5" freestyle, 650 g, 6S (freestyle5)

## Grade A: flies like your quad

- all coverage items and critical checks met

Made by `fpvident` from 10 sessions in 74 s. 16 PASS, 1 WARN.

Form: 0.65 kg, 6S lipo, 1960 KV, prop 5.1" × 4.99" × 3, class five, top speed 165 km/h.

## Checks
| Check | Result | Detail |
|---|---|---|
| Segmentation auto = hand | PASS | IoU 0.998 (threshold 0.95) |
| Motor dynamics 5–30 Hz | PASS | measured/sim [1.1, 1.1, 1.1, 1.1] (0.8–1.2) |
| Motor KV (vbat scale) | PASS | scale 1.046 (0.8–1.25) |
| Hover detection | PASS | acc, 51.2 s |
| Hover rotor speed between sessions | PASS | spread 3.9 % (threshold 6 %) |
| Motor model free run | PASS | 5.89 Hz RMS (threshold max(12, 7.5 % of 158 Hz) = 12.0) |
| Rotation roll/pitch | PASS | chirp, R² ≥ 0.95, rel. std. error ≤ 4.9 % (threshold 10 %) |
| Rotation yaw | PASS | R² 0.95, rel. std. error 1.3 % (threshold 25 %) |
| Imbalance consistent | WARN | spread between sessions [1.5, 2.8, 0.45] ×10⁻³ |
| Imbalance size | PASS | |β| 1.10 ×10⁻³ m (≤ 0.1 · arm = 8.0) |
| Battery fit | PASS | vbat, RMS 0.22 V |
| ACC rotor drag | PASS | R² 0.9, uncertainty 0.044 (thresholds 0.5 / 0.15) |
| Top speed backed | PASS | calibrated to the given 165 km/h |
| Hover throttle sim vs. log | PASS | 0.238 vs. 0.245 (±0.02) |
| Roll t90 sim vs. log | PASS | 141.4 vs. 141.4 ms (±20 %) |
| Roll lag sim vs. log | PASS | 4.0 vs. 6.5 ms (±25 %, at least 3 ms) |
| Closed-loop tracking | PASS | 0.04 (threshold 0.35) |

## Measured
| Quantity | Value | Source |
|---|---|---|
| Motor/ESC a, b, c (per motor) | see Motor/ESC | eRPM vs. DShot command |
| Thrust coefficient kT | 1.621e-06 | hover 158 Hz at 0.65 kg (acc) |
| Roll/pitch torque gain (→ Ixx, Iyy) | 82 / 69.9 | IV, chirp |
| Yaw: J_rotor/Izz, kQ, direction | 0.00226 | IV, motor torque as regressor |
| Torque imbalance β | [0.51, -1.1, -0.0071] ×10⁻³ m | eRPM differences at hover |
| Battery V0, sag R | 25.34 V, 0.118 | vbat |
| Rotor drag (linear) | 0.29 1/s | ACC dead reckoning |
| Body drag | cd_area [0.0088, 0.0105, 0.0265] | given top speed 165 km/h |

## Class defaults and assumptions
| Quantity | Value | Why |
|---|---|---|
| Motor distance (arm) | 80 mm | 80 mm × D/5.1" (only the absolute inertias depend on it) |
| Izz | 0.00292 kg·m² | 1.7 × mean roll/pitch inertia (only J_rotor/Izz is measured) |
| dirty_air wash/wake/gust | 0.7/1.0/10.5 | not fitted in v1: five class |
| Wake/turbulence/VRS | turb 0.31 / 3.8 m/s | reference scaled physically (Python plant only) |

## Plausibility vs. the five class band
| Quantity | Value | Band | Result |
|---|---|---|---|
| Weight | 650 g | 450–850 | PASS |
| Cells | 6 S | 4–6 | PASS |
| Hover rotor speed | 158 Hz | 130–220 | PASS |
| Motor time constant at hover | 32.1 ms | 18–40 | PASS |
| kT / kT₀ | 1.01  | 0.5–2 | PASS |
| Ixx / I₀ | 0.791  | 0.4–2.5 | PASS |
| Iyy / I₀ | 0.928  | 0.4–2.5 | PASS |
| J_rotor / J₀ | 1.32  | 0.3–3 | PASS |
| kQ/kT / (0.0117 m × D/5.1) | 1.25  | 0.5–2 | PASS |
| Rate damping roll | 2.05  | 0–5 | PASS |
| Rate damping pitch | 0.671  | 0–5 | PASS |
| Rate damping yaw | 0.898  | 0–5 | PASS |
| Top speed (model) | 165 km/h | 130–190 | PASS |
| Pack resistance (battery_r × R_m / cells) | 2.94 mΩ/cell | 2–12 | PASS |

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
Model per motor: dω/dt = a·(u·V − ω/Kv) − b·ω² − c, 1 ms dead time.
- Equation-error fit (R² 0.719): a [3791, 4021, 3983, 3853], b [0.002817, 0.002842, 0.002902, 0.002859], c [372, 496, 531, 453]
- Dynamics refinement λ = 1.26 (a, b, c scaled together, steady state unchanged): free run 6.39 → 5.89 Hz RMS
- Final: a [4778, 5067, 5019, 4856], b [0.003549, 0.003581, 0.003657, 0.003603], c [469, 625, 669, 570]
- Response measured/simulated (roll+pitch differential) at 5 Hz 1.1, 10 Hz 1.1, 20 Hz 1.1, 30 Hz 1.1
- Voltage: vbat × 1.046
- Free run per session [Hz RMS]: btfl_001#1 6.9, btfl_011#1 4.6, btfl_012#1 8.7, m5_01_chirp6#1 6.7, m5_02_crash#1 7.5, m5_03_hover#1 2.4, m5_04_freestyle#1 5.2, m5_05_freestyle#1 3.9, m5_06_freestyle_abgeschnitten#1 4.0, m5_07_chirp9#1 4.4

## Hover and thrust
Hover from ACC (|a| = 1 g, level, quiet), 51.2 s: rotor 157.8 Hz, motor command 0.245, throttle 0.245.
kT = m·g / Σω² = 1.621e-06 N/(rad/s)² (mass 0.65 kg from the form).
Per session: m5_01_chirp6#1 153.7 Hz (11.9 s), m5_03_hover#1 156.0 Hz (4.7 s), m5_04_freestyle#1 159.8 Hz (4.9 s), m5_07_chirp9#1 159.3 Hz (29.7 s)
The no-ACC rule on the ACC sessions (Hz), as a check of the fallback: m5_01_chirp6#1 175.5, m5_03_hover#1 154.7, m5_04_freestyle#1 149.3, m5_05_freestyle#1 160.7, m5_07_chirp9#1 161.3

## Rotational dynamics
Instrumental-variable fit on chirps (307 s of chirp in the logs), band 1–30 Hz.
| Axis | Gain | Damping | R² | Rel. std. error (jackknife over windows) |
|---|---|---|---|---|
| roll | 82.01 | -2.05 | 0.95 | 4.9 % |
| pitch | 69.89 | -0.671 | 0.95 | 2.1 % |
| yaw | -2.256 | -0.898 | 0.95 | 1.3 % |
Roll/pitch: rad/s² per 10⁶ (rad/s)² → Ixx 0.00158, Iyy 0.00186 kg·m² (with kT and arm).
Yaw: −J_rotor/Izz (×10⁻³) → J_rotor 6.59e-06 kg·m², kQ = J_rotor·b = 2.37e-08; yaw_motors_reversed = ON (sign from the data).
Rate damping: [2.05, 0.671, 0.898] (from the chirp fit).

## Trim / imbalance
From the rotor speed differences at hover (51227 samples): β = [0.512, -1.1, -0.00708] ×10⁻³ m (torque = β · total thrust; roll, pitch, yaw).
Per session (×10⁻³): m5_01_chirp6#1 [0.27, -2.87, 0.35], m5_03_hover#1 [-2.86, 3.84, -0.8], m5_04_freestyle#1 [0.4, 2.35, -0.49], m5_07_chirp9#1 [1.1, -1.76, 0.06]

## Battery
Source: vbat. V = V0 − R·I (I = the sim's current proxy). V0 per session: {'m5_01_chirp6#1': 24.91, 'm5_02_crash#1': 25.34, 'm5_03_hover#1': 25.12, 'm5_04_freestyle#1': 24.4, 'm5_05_freestyle#1': 23.7, 'm5_06_freestyle_abgeschnitten#1': 23.36, 'm5_07_chirp9#1': 25.08}.
battery_v0 = 25.34 V (freshest pack), battery_r = 0.118, discharge fitted 0.344 V/min (exported as 0: the game simulates only the sag), RMS 0.22 V.

## Drag and top speed
ACC dead reckoning: 16 passes from hover (each up to 5 s, start velocity and tilt fitted too: RMS 1.6 m/s, 14°), 268 blocks of 0.25 s, speed p50/p90/p99 [5.2, 22.8, 35.6] m/s. Fit R² 0.9: rotor drag 0.29 1/s (±4.4 %), body scale 0.02 (±4.4e+02 %).
Used: rotor drag from the ACC fit (4.78e-05); body drag calibrated to the given top speed 165 km/h (the ACC fit does not determine it: too few fast passes): cd_area [0.0088, 0.0105, 0.0265] m².
Top speed of the model (full throttle, level, fresh pack with sag): **165 km/h** at 82° tilt, 588 Hz rotor; given 165 km/h; with the default body drag it would be 159 km/h.

## Propwash / turbulence
Log metric (15–78 Hz roll/pitch tracking error, deg/s, median): calm 0.75, after throttle chops 0.67, flips after chops 4.28; 99th percentile of all usable windows outside chirps 11.34 (windows (4125, 849, 96, 8325)). Target for the dirty_air calibration in Godot (p99 scaled to freestyle5 = 14.5 deg/s at 11.3).
dirty_air wash/wake/gust 0.7/1.0/10.5 from the five class.
Wake/turbulence amplitudes of the Python plant: reference values scaled with the induced velocity (factor 1): turb_base_ms 0.31, pw_turb_ms 3.8, wake_r0 0.12 m.

## Validation
Hover: motor command sim 0.238 / log 0.245, rotor 158 / 158 Hz (model voltage 23.79 V).
Roll steps (12, stick replay through the Betaflight port + model): t90 sim 141.4 ms / log 141.4 ms, lag behind the setpoint (at 50 %) sim 4.0 ms / log 6.5 ms, gyro deviation sim−log 0.04 × log RMS (median).
Events (session, time, plateau deg/s, t90 log/sim ms): m5_05_freestyle#1 18.24 s 700 189/192; m5_05_freestyle#1 22.13 s 700 127/123; btfl_012#1 44.13 s 700 177/182; btfl_012#1 68.07 s 700 108/102; m5_04_freestyle#1 53.09 s 700 100/98; m5_04_freestyle#1 60.1 s 700 150/151; m5_06_freestyle_abgeschnitten#1 23.79 s 700 125/130; m5_07_chirp9#1 11.25 s 700 133/132; m5_07_chirp9#1 31.31 s 700 105/103; m5_06_freestyle_abgeschnitten#1 13.81 s 662 230/246; btfl_011#1 71.94 s 633 357/359; btfl_011#1 8.63 s 621 219/218
