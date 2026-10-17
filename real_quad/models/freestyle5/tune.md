# Optimized tune of 5" freestyle, 650 g, 6S

How this quad would probably fly when well tuned: its identified plant flown in the simulator with the Betaflight settings below (a drone flies them when its tune setting in real_quad's settings is "optimized"). **Simulator only** - not a tune for the real quad: no noise, motor heat or frame resonances are simulated, and the filters stay as flown.

## Settings

| Setting | Flown | Tuned | Stopped by (the last tries) |
|---|---|---|---|
| Roll P | 45 | 45 | no better flight |
| Roll D | 33 | 27 | trust region |
| Roll D-max | 33 | 39 | trust region |
| Roll Feedforward | 125 | 140 | no better flight |
| Roll I | 55 | 55 | no better flight |
| Pitch P | 42 | 50 | trust region |
| Pitch D | 37 | 37 | no better flight |
| Pitch D-max | 37 | 37 | no better flight |
| Pitch Feedforward | 130 | 130 | no better flight |
| Pitch I | 58 | 58 | no better flight |
| Yaw P | 50 | 50 | no better flight |
| Yaw Feedforward | 140 | 140 | no better flight |
| Yaw I | 80 | 80 | no better flight |
| TPA rate (%) | 45 | 36 | trust region |
| TPA breakpoint | 1350 | 1350 | no better flight |
| Dynamic idle (x100 rpm) | 40 | 37 | still gaining when the search ended |
| Anti-gravity gain | 100 | 80 | trust region |
| Thrust linear (%) | 0 | 0 | up: trust region; down: class bound |

## Predicted changes (simulator, the same flights for both)

| Flight | Flown | Tuned | Change |
|---|---|---|---|
| Stick steps, tracking error (deg/s) | 15.7 | 15.1 | -4 % |
| Flips, tracking error (deg/s) | 76.4 | 75.3 | -1 % |
| Your stick replay, tracking error (deg/s) | 5.4 | 4.9 | -8 % |
| Drop and catch, worst prop wash (deg/s) | 15.0 | 14.3 | -5 % |
| Hover roll step: time to 90 % (ms) | 16 | 16 | |
| Hover roll and pitch steps: overshoot (%) | 4 | 2 | at most 10 |
| Your stick replay: overshoot at the stick stops (%) | 9 | 8 | at most 10 |
| Out of motor authority (% of the time) | 1.09 | 1.27 | at most 1.59 |
| Peak sensitivity Ms roll / pitch / yaw, at hover, identified plant | 1.87 / 1.69 / 1.42 | 1.83 / 1.79 / 1.42 | at most 2.00 / 2.00 / 2.00 |
| Peak sensitivity Ms roll / pitch / yaw, at hover, plant with +15 % loop gain and +0.4 ms dead time | 2.28 / 1.99 / 1.59 | 2.18 / 2.15 / 1.59 | at most 2.50 / 2.50 / 2.50 |
| Peak sensitivity Ms roll / pitch / yaw, at the flown TPA breakpoint, identified plant | 1.92 / 1.74 / 1.37 | 1.85 / 1.83 / 1.37 | at most 2.00 / 2.00 / 2.00 |
| Peak sensitivity Ms roll / pitch / yaw, at the flown TPA breakpoint, plant with +15 % loop gain and +0.4 ms dead time | 2.40 / 2.08 / 1.53 | 2.23 / 2.25 / 1.53 | at most 2.50 / 2.50 / 2.50 |
| Peak sensitivity Ms roll / pitch / yaw, at full throttle, identified plant | 1.63 / 1.53 / 1.16 | 1.68 / 1.71 / 1.17 | at most 2.00 / 2.00 / 2.00 |
| Peak sensitivity Ms roll / pitch / yaw, at full throttle, plant with +15 % loop gain and +0.4 ms dead time | 1.94 / 1.77 / 1.22 | 2.01 / 2.09 / 1.24 | at most 2.50 / 2.50 / 2.50 |

## How

- Flights: steps_hover, steps_idle, steps_punch, flip, replay m5_06_freestyle_abgeschnitten#1 27 s, replay m5_06_freestyle_abgeschnitten#1 19 s, replay btfl_012#1 127 s.
- Cost: the mean over the flight groups of the tuned tune's tracking error / the flown tune's.
- Search: coordinate search from the flown tune, each setting tried up, then (if up was not kept) down; a move is kept when it lowers the cost by more than 0.002. Stage 1: P, D, D-max, feedforward, I per axis (yaw without D; D-max kept at least D), steps of 25, 12, 6 %. Stage 2: TPA rate, TPA breakpoint, dynamic idle (with the RPM filter only), anti-gravity, thrust linear, steps of 10, 100, 6, 20, 10, then half. 83 evaluations, 140 s.
- Limits: every setting at most 20 % from its flown value (rounded inward), and inside the five class's bounds (P, I, feedforward around Betaflight's defaults; highest D at most 1.50 x the default D-max with the RPM filter, 1.0 x without; D/P band), widened to the flown values.
- Constraints, a candidate is not taken when: its share of time out of motor authority (motor mix range above 1), the mean over the stick flights, is more than the flown tune's + 0.5 percentage points; on the identified plant or on the perturbed one (15 % more loop gain and 0.4 ms more dead time, for the identification's error), any axis's peak sensitivity Ms is above the larger of the flown tune's + 0.01 (the estimate's resolution) and 2.0 (2.5 on the perturbed plant: the usual limit of a sound loop); or the overshoot of the hover roll and pitch steps, or at the stick stops of your replay (any axis; a setpoint change of 200 deg/s or more in 0.1 s, then held within 10 % of it for 50 ms; the rate's peak past the setpoint in the 0.1 s after, of the change), is more than 10 % or the flown tune's, whichever is higher.
- Ms: a chirp of +-20 (PID sum units, 5-400 Hz in 2 s) added to one axis's PID sum; S = 1/(1+L) from it and the PID sum the mixer gets, in bands of +-2.5 % from 8 to 300 Hz; Ms the largest |S|. Flown on a pack of 3.8 V per cell at rest (a full one would add 6-10 % loop gain). Always at hover; also at the flown tune's TPA breakpoint and at full throttle for a candidate whose TPA rate, TPA breakpoint or thrust linear differ from flown (the flown tune is always measured there, so the table shows what the tuned one would have to keep).
- Stopped by: "still gaining when the search ended" means its last step was kept: the search's step budget, not the flight, stopped it.

## Not modelled

- Gyro noise, motor heat and D-term noise: a real quad with this much D may run hot; the class's D bound stands in for them.
- Frame resonances, bent props, the receiver link's jitter; the filters are not tuned.
- The plant is the identified one: where the identification is off, so is this.
