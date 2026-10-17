# Optimized tune of 3.5" freestyle, 300 g, 4S

How this quad would probably fly when well tuned: its identified plant flown in the simulator with the Betaflight settings below (a drone flies them when its tune setting in real_quad's settings is "optimized"). **Simulator only** - not a tune for the real quad: no noise, motor heat or frame resonances are simulated, and the filters stay as flown.

## Settings

| Setting | Flown | Tuned | Stopped by (the last tries) |
|---|---|---|---|
| Roll P | 42 | 42 | up: stability margin; down: no better flight |
| Roll D | 33 | 33 | up: stability margin; down: no better flight |
| Roll D-max | 33 | 33 | no better flight |
| Roll Feedforward | 135 | 162 | trust region |
| Roll I | 63 | 63 | no better flight |
| Pitch P | 49 | 55 | no better flight |
| Pitch D | 44 | 36 | trust region |
| Pitch D-max | 44 | 37 | still gaining when the search ended |
| Pitch Feedforward | 165 | 165 | up: overshoot; down: no better flight |
| Pitch I | 71 | 71 | up: no better flight; down: overshoot |
| Yaw P | 65 | 65 | no better flight |
| Yaw Feedforward | 160 | 160 | no better flight |
| Yaw I | 104 | 104 | no better flight |
| TPA rate (%) | 50 | 40 | trust region |
| TPA breakpoint | 1350 | 1450 | no better flight |
| Dynamic idle (x100 rpm) | 45 | 39 | no better flight |
| Anti-gravity gain | 110 | 110 | no better flight |
| Thrust linear (%) | 40 | 40 | up: class bound; down: no better flight |

## Predicted changes (simulator, the same flights for both)

| Flight | Flown | Tuned | Change |
|---|---|---|---|
| Stick steps, tracking error (deg/s) | 16.0 | 15.3 | -4 % |
| Flips, tracking error (deg/s) | 75.8 | 74.3 | -2 % |
| Your stick replay, tracking error (deg/s) | 9.2 | 7.6 | -17 % |
| Drop and catch, worst prop wash (deg/s) | 13.1 | 12.4 | -5 % |
| Hover roll step: time to 90 % (ms) | 16 | 16 | |
| Hover roll and pitch steps: overshoot (%) | 3 | 2 | at most 10 |
| Your stick replay: overshoot at the stick stops (%) | 7 | 6 | at most 10 |
| Out of motor authority (% of the time) | 1.05 | 1.07 | at most 1.55 |
| Peak sensitivity Ms roll / pitch / yaw, at hover, identified plant | 1.96 / 1.92 / 1.67 | 1.96 / 1.82 / 1.67 | at most 2.00 / 2.00 / 2.00 |
| Peak sensitivity Ms roll / pitch / yaw, at hover, plant with +15 % loop gain and +0.4 ms dead time | 2.48 / 2.41 / 1.97 | 2.48 / 2.21 / 1.97 | at most 2.50 / 2.50 / 2.50 |
| Peak sensitivity Ms roll / pitch / yaw, at the flown TPA breakpoint, identified plant | 1.96 / 1.92 / 1.65 | 1.96 / 1.82 / 1.65 | at most 2.00 / 2.00 / 2.00 |
| Peak sensitivity Ms roll / pitch / yaw, at the flown TPA breakpoint, plant with +15 % loop gain and +0.4 ms dead time | 2.48 / 2.41 / 1.95 | 2.48 / 2.21 / 1.95 | at most 2.50 / 2.50 / 2.50 |
| Peak sensitivity Ms roll / pitch / yaw, at full throttle, identified plant | 1.37 / 1.37 / 1.34 | 1.46 / 1.42 / 1.28 | at most 2.00 / 2.00 / 2.00 |
| Peak sensitivity Ms roll / pitch / yaw, at full throttle, plant with +15 % loop gain and +0.4 ms dead time | 1.50 / 1.50 / 1.35 | 1.64 / 1.58 / 1.34 | at most 2.50 / 2.50 / 2.50 |

## How

- Flights: steps_hover, steps_idle, steps_punch, flip, replay LOG00003#1 32 s, replay LOG00003#1 44 s, replay mini_03_freestyle#1 166 s.
- Cost: the mean over the flight groups of the tuned tune's tracking error / the flown tune's.
- Search: coordinate search from the flown tune, each setting tried up, then (if up was not kept) down; a move is kept when it lowers the cost by more than 0.002. Stage 1: P, D, D-max, feedforward, I per axis (yaw without D; D-max kept at least D), steps of 25, 12, 6 %. Stage 2: TPA rate, TPA breakpoint, dynamic idle (with the RPM filter only), anti-gravity, thrust linear, steps of 10, 100, 6, 20, 10, then half. 89 evaluations, 208 s.
- Limits: every setting at most 20 % from its flown value (rounded inward), and inside the mini class's bounds (P, I, feedforward around Betaflight's defaults; highest D at most 1.50 x the default D-max with the RPM filter, 1.0 x without; D/P band), widened to the flown values.
- Constraints, a candidate is not taken when: its share of time out of motor authority (motor mix range above 1), the mean over the stick flights, is more than the flown tune's + 0.5 percentage points; on the identified plant or on the perturbed one (15 % more loop gain and 0.4 ms more dead time, for the identification's error), any axis's peak sensitivity Ms is above the larger of the flown tune's + 0.01 (the estimate's resolution) and 2.0 (2.5 on the perturbed plant: the usual limit of a sound loop); or the overshoot of the hover roll and pitch steps, or at the stick stops of your replay (any axis; a setpoint change of 200 deg/s or more in 0.1 s, then held within 10 % of it for 50 ms; the rate's peak past the setpoint in the 0.1 s after, of the change), is more than 10 % or the flown tune's, whichever is higher.
- Ms: a chirp of +-20 (PID sum units, 5-400 Hz in 2 s) added to one axis's PID sum; S = 1/(1+L) from it and the PID sum the mixer gets, in bands of +-2.5 % from 8 to 300 Hz; Ms the largest |S|. Flown on a pack of 3.8 V per cell at rest (a full one would add 6-10 % loop gain). Always at hover; also at the flown tune's TPA breakpoint and at full throttle for a candidate whose TPA rate, TPA breakpoint or thrust linear differ from flown (the flown tune is always measured there, so the table shows what the tuned one would have to keep).
- Stopped by: "still gaining when the search ended" means its last step was kept: the search's step budget, not the flight, stopped it.

## Not modelled

- Gyro noise, motor heat and D-term noise: a real quad with this much D may run hot; the class's D bound stands in for them.
- Frame resonances, bent props, the receiver link's jitter; the filters are not tuned.
- The plant is the identified one: where the identification is off, so is this.
