# Optimized tune of 3.5" freestyle, 300 g, 4S

How this quad would probably fly when well tuned: its identified plant flown in the simulator with the Betaflight settings below (a drone flies them when its tune setting in real_quad's settings is "optimized"). **Simulator only** - not a tune for the real quad: no noise, motor heat or frame resonances are simulated, and the filters stay as flown.

## Settings

| Setting | Flown | Tuned | Stopped by (the last tries) |
|---|---|---|---|
| Roll P | 42 | 42 | stability margin |
| Roll D | 33 | 33 | stability margin |
| Roll D-max | 33 | 33 | no better flight |
| Roll Feedforward | 135 | 162 | trust region |
| Roll I | 63 | 63 | stability margin |
| Pitch P | 49 | 49 | stability margin |
| Pitch D | 44 | 44 | stability margin |
| Pitch D-max | 44 | 44 | up: stability margin; down: no better flight |
| Pitch Feedforward | 165 | 165 | up: stability margin; down: no better flight |
| Pitch I | 71 | 71 | up: stability margin; down: no better flight |
| Yaw P | 65 | 65 | up: stability margin; down: no better flight |
| Yaw Feedforward | 160 | 160 | no better flight |
| Yaw I | 104 | 104 | stability margin |
| TPA rate (%) | 50 | 40 | trust region |
| TPA breakpoint | 1350 | 1250 | no better flight |
| Dynamic idle (x100 rpm) | 45 | 45 | no better flight |
| Anti-gravity gain | 110 | 90 | no better flight |
| Thrust linear (%) | 40 | 40 | up: class bound; down: no better flight |

## Predicted changes (simulator, the same flights for both)

| Flight | Flown | Tuned | Change |
|---|---|---|---|
| Stick steps, tracking error (deg/s) | 16.6 | 16.3 | -2 % |
| Flips, tracking error (deg/s) | 76.6 | 77.0 | +1 % |
| Your stick replay, tracking error (deg/s) | 9.6 | 8.8 | -8 % |
| Drop and catch, worst prop wash (deg/s) | 14.6 | 14.1 | -3 % |
| Hover roll step: time to 90 % (ms) | 16 | 14 | |
| Hover roll and pitch steps: overshoot (%) | 7 | 10 | at most 10 |
| Your stick replay: overshoot at the stick stops (%) | 10 | 8 | at most 10 |
| Out of motor authority (% of the time) | 1.07 | 1.06 | at most 1.57 |
| Peak sensitivity Ms roll / pitch / yaw, identified plant | 3.22 / 3.09 / 2.26 | 3.13 / 3.02 / 2.24 | at most as flown + 0.01 |
| Phase margin roll / pitch / yaw (deg), identified plant | 27.9 / 29.7 / 41.2 | 27.9 / 29.7 / 43.8 | at least as flown - 0.5 |
| Peak sensitivity Ms roll / pitch / yaw, plant with 80 % inertia and 150 % motor delay | 14.70 / 13.62 / 5.07 | 13.17 / 12.40 / 4.88 | at most as flown + 0.01 |
| Phase margin roll / pitch / yaw (deg), plant with 80 % inertia and 150 % motor delay | 4.4 / 5.8 / 15.5 | 4.5 / 5.9 / 19.2 | at least as flown - 0.5 |

## How

- Flights: steps_hover, steps_idle, steps_punch, flip, replay LOG00003#1 32 s, replay LOG00003#1 44 s, replay mini_03_freestyle#1 166 s.
- Cost: the mean over the flight groups of the tuned tune's tracking error / the flown tune's.
- Search: coordinate search from the flown tune, each setting tried up, then (if up was not kept) down; a move is kept when it lowers the cost by more than 0.002. Stage 1: P, D, D-max, feedforward, I per axis (yaw without D; D-max kept at least D), steps of 25, 12, 6 %. Stage 2: TPA rate, TPA breakpoint, dynamic idle (with the RPM filter only), anti-gravity, thrust linear, steps of 10, 100, 6, 20, 10, then half. 93 evaluations, 194 s.
- Limits: every setting at most 20 % from its flown value (rounded inward), and inside the mini class's bounds (P, I, feedforward around Betaflight's defaults; highest D at most 1.50 x the default D-max with the RPM filter, 1.0 x without; D/P band), widened to the flown values.
- Constraints, a candidate is not taken when: its share of time out of motor authority (motor mix range above 1), the mean over the stick flights, is more than the flown tune's + 0.5 percentage points; on the identified plant or on the one with 80 % inertia and 150 % motor delay, any axis's peak sensitivity Ms is more than 0.01 above or its phase margin more than 0.5 deg below the flown tune's (the estimate's resolution); or the overshoot of the hover roll and pitch steps, or at the stick stops of your replay (any axis; a setpoint change of 200 deg/s or more in 0.1 s, then held within 10 % of it for 50 ms; the rate's peak past the setpoint in the 0.1 s after, of the change), is more than 10 % or the flown tune's, whichever is higher.
- Ms and phase margin: at hover, a chirp of +-20 (PID sum units, 5-400 Hz in 2 s) added to one axis's PID sum; S = 1/(1+L) from it and the PID sum the mixer gets, in bands of +-2.5 % from 8 to 300 Hz; Ms the largest |S|, the phase margin at the first band with |L| below 1.
- Stopped by: "still gaining when the search ended" means its last step was kept: the search's step budget, not the flight, stopped it.

## Not modelled

- Gyro noise, motor heat and D-term noise: a real quad with this much D may run hot; the class's D bound stands in for them.
- Frame resonances, bent props, the receiver link's jitter; the filters are not tuned.
- The plant is the identified one: where the identification is off, so is this.
