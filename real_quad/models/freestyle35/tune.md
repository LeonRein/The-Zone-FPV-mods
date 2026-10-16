# Optimized tune of 3.5" freestyle, 300 g, 4S

How this quad would probably fly when well tuned: its identified plant flown in the simulator with the Betaflight settings below (a drone flies them when its tune setting in real_quad's settings is "optimized"). **Simulator only** - not a tune for the real quad: no noise, motor heat or frame resonances are simulated, and the filters stay as flown.

## Settings

| Setting | Flown | Tuned | Stopped by (the last tries) |
|---|---|---|---|
| Roll P | 42 | 42 | up: stability margin; down: no better flight |
| Roll D | 33 | 33 | stability margin |
| Roll D-max | 33 | 33 | no better flight |
| Roll Feedforward | 135 | 162 | trust region |
| Roll I | 63 | 63 | no better flight |
| Pitch P | 49 | 49 | up: stability margin; down: no better flight |
| Pitch D | 44 | 44 | up: stability margin; down: no better flight |
| Pitch D-max | 44 | 44 | no better flight |
| Pitch Feedforward | 165 | 185 | overshoot |
| Pitch I | 71 | 71 | no better flight |
| Yaw P | 65 | 65 | up: stability margin; down: no better flight |
| Yaw Feedforward | 160 | 160 | no better flight |
| Yaw I | 104 | 104 | up: stability margin; down: no better flight |
| TPA rate (%) | 50 | 40 | trust region |
| TPA breakpoint | 1350 | 1450 | no better flight |
| Dynamic idle (x100 rpm) | 45 | 54 | trust region (reached) |
| Anti-gravity gain | 110 | 90 | no better flight |
| Thrust linear (%) | 40 | 40 | up: class bound; down: no better flight |

## Predicted changes (simulator, the same flights for both)

| Flight | Flown | Tuned | Change |
|---|---|---|---|
| Stick steps, tracking error (deg/s) | 16.1 | 15.6 | -3 % |
| Flips, tracking error (deg/s) | 76.9 | 77.2 | +0 % |
| Your stick replay, tracking error (deg/s) | 9.3 | 7.7 | -16 % |
| Drop and catch, worst prop wash (deg/s) | 14.6 | 13.9 | -5 % |
| Hover roll step: time to 90 % (ms) | 36 | 18 | |
| Hover roll and pitch steps: overshoot (%) | 2 | 2 | at most 10 |
| Your stick replay: overshoot at the stick stops (%) | 5 | 5 | at most 10 |
| Out of motor authority (% of the time) | 1.08 | 1.06 | at most 1.58 |
| Peak sensitivity Ms roll / pitch / yaw, identified plant | 1.82 / 1.79 / 1.61 | 1.82 / 1.79 / 1.61 | at most as flown + 0.01 |
| Phase margin roll / pitch / yaw (deg), identified plant | 46.2 / 47.7 / 54.4 | 46.2 / 47.7 / 54.4 | at least as flown - 0.5 |
| Peak sensitivity Ms roll / pitch / yaw, plant with 80 % inertia and 150 % motor delay | 2.55 / 2.48 / 2.07 | 2.55 / 2.48 / 2.07 | at most as flown + 0.01 |
| Phase margin roll / pitch / yaw (deg), plant with 80 % inertia and 150 % motor delay | 31.7 / 35.2 / 41.6 | 31.7 / 35.2 / 41.6 | at least as flown - 0.5 |

## How

- Flights: steps_hover, steps_idle, steps_punch, flip, replay LOG00003#1 32 s, replay LOG00003#1 44 s, replay mini_03_freestyle#1 166 s.
- Cost: the mean over the flight groups of the tuned tune's tracking error / the flown tune's.
- Search: coordinate search from the flown tune, each setting tried up, then (if up was not kept) down; a move is kept when it lowers the cost by more than 0.002. Stage 1: P, D, D-max, feedforward, I per axis (yaw without D; D-max kept at least D), steps of 25, 12, 6 %. Stage 2: TPA rate, TPA breakpoint, dynamic idle (with the RPM filter only), anti-gravity, thrust linear, steps of 10, 100, 6, 20, 10, then half. 89 evaluations, 210 s.
- Limits: every setting at most 20 % from its flown value (rounded inward), and inside the mini class's bounds (P, I, feedforward around Betaflight's defaults; highest D at most 1.50 x the default D-max with the RPM filter, 1.0 x without; D/P band), widened to the flown values.
- Constraints, a candidate is not taken when: its share of time out of motor authority (motor mix range above 1), the mean over the stick flights, is more than the flown tune's + 0.5 percentage points; on the identified plant or on the one with 80 % inertia and 150 % motor delay, any axis's peak sensitivity Ms is more than 0.01 above or its phase margin more than 0.5 deg below the flown tune's (the estimate's resolution); or the overshoot of the hover roll and pitch steps, or at the stick stops of your replay (any axis; a setpoint change of 200 deg/s or more in 0.1 s, then held within 10 % of it for 50 ms; the rate's peak past the setpoint in the 0.1 s after, of the change), is more than 10 % or the flown tune's, whichever is higher.
- Ms and phase margin: at hover, a chirp of +-20 (PID sum units, 5-400 Hz in 2 s) added to one axis's PID sum; S = 1/(1+L) from it and the PID sum the mixer gets, in bands of +-2.5 % from 8 to 300 Hz; Ms the largest |S|, the phase margin at the first band with |L| below 1.
- Stopped by: "still gaining when the search ended" means its last step was kept: the search's step budget, not the flight, stopped it.

## Not modelled

- Gyro noise, motor heat and D-term noise: a real quad with this much D may run hot; the class's D bound stands in for them.
- Frame resonances, bent props, the receiver link's jitter; the filters are not tuned.
- The plant is the identified one: where the identification is off, so is this.
