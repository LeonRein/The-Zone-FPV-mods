# Optimized tune of 5" freestyle, 650 g, 6S

How this quad would probably fly when well tuned: its identified plant flown in the simulator with the Betaflight settings below (a drone flies them when its tune setting in real_quad's settings is "optimized"). **Simulator only** - not a tune for the real quad: no noise, motor heat or frame resonances are simulated, and the filters stay as flown.

## Settings

| Setting | Flown | Tuned | Stopped by (the last tries) |
|---|---|---|---|
| Roll P | 45 | 45 | up: stability margin; down: overshoot |
| Roll D | 33 | 33 | up: stability margin; down: overshoot |
| Roll D-max | 33 | 33 | up: overshoot; down: no better flight |
| Roll Feedforward | 125 | 125 | overshoot |
| Roll I | 55 | 55 | no better flight |
| Pitch P | 42 | 42 | up: stability margin; down: no better flight |
| Pitch D | 37 | 37 | stability margin |
| Pitch D-max | 37 | 37 | no better flight |
| Pitch Feedforward | 130 | 146 | no better flight |
| Pitch I | 58 | 58 | up: overshoot; down: no better flight |
| Yaw P | 50 | 50 | up: stability margin; down: no better flight |
| Yaw Feedforward | 140 | 140 | no better flight |
| Yaw I | 80 | 80 | up: stability margin; down: no better flight |
| TPA rate (%) | 45 | 36 | trust region |
| TPA breakpoint | 1350 | 1250 | no better flight |
| Dynamic idle (x100 rpm) | 40 | 43 | still gaining when the search ended |
| Anti-gravity gain | 100 | 80 | trust region |
| Thrust linear (%) | 0 | 0 | up: trust region; down: class bound |

## Predicted changes (simulator, the same flights for both)

| Flight | Flown | Tuned | Change |
|---|---|---|---|
| Stick steps, tracking error (deg/s) | 16.4 | 16.1 | -2 % |
| Flips, tracking error (deg/s) | 77.2 | 77.4 | +0 % |
| Your stick replay, tracking error (deg/s) | 5.6 | 5.4 | -3 % |
| Drop and catch, worst prop wash (deg/s) | 14.4 | 12.9 | -10 % |
| Hover roll step: time to 90 % (ms) | 16 | 16 | |
| Hover roll and pitch steps: overshoot (%) | 11 | 11 | at most 11 |
| Your stick replay: overshoot at the stick stops (%) | 10 | 10 | at most 10 |
| Out of motor authority (% of the time) | 1.13 | 1.16 | at most 1.63 |
| Peak sensitivity Ms roll / pitch / yaw, identified plant | 2.37 / 2.07 / 1.63 | 2.37 / 2.07 / 1.63 | at most as flown + 0.01 |
| Phase margin roll / pitch / yaw (deg), identified plant | 35.4 / 44.1 / 64.1 | 35.4 / 44.1 / 64.1 | at least as flown - 0.5 |
| Peak sensitivity Ms roll / pitch / yaw, plant with 80 % inertia and 150 % motor delay | 4.64 / 3.42 / 2.27 | 4.64 / 3.42 / 2.27 | at most as flown + 0.01 |
| Phase margin roll / pitch / yaw (deg), plant with 80 % inertia and 150 % motor delay | 17.6 / 24.8 / 45.8 | 17.6 / 24.8 / 45.8 | at least as flown - 0.5 |

## How

- Flights: steps_hover, steps_idle, steps_punch, flip, replay m5_06_freestyle_abgeschnitten#1 27 s, replay m5_06_freestyle_abgeschnitten#1 19 s, replay btfl_012#1 127 s.
- Cost: the mean over the flight groups of the tuned tune's tracking error / the flown tune's.
- Search: coordinate search from the flown tune, each setting tried up, then (if up was not kept) down; a move is kept when it lowers the cost by more than 0.002. Stage 1: P, D, D-max, feedforward, I per axis (yaw without D; D-max kept at least D), steps of 25, 12, 6 %. Stage 2: TPA rate, TPA breakpoint, dynamic idle (with the RPM filter only), anti-gravity, thrust linear, steps of 10, 100, 6, 20, 10, then half. 91 evaluations, 137 s.
- Limits: every setting at most 20 % from its flown value (rounded inward), and inside the five class's bounds (P, I, feedforward around Betaflight's defaults; highest D at most 1.50 x the default D-max with the RPM filter, 1.0 x without; D/P band), widened to the flown values.
- Constraints, a candidate is not taken when: its share of time out of motor authority (motor mix range above 1), the mean over the stick flights, is more than the flown tune's + 0.5 percentage points; on the identified plant or on the one with 80 % inertia and 150 % motor delay, any axis's peak sensitivity Ms is more than 0.01 above or its phase margin more than 0.5 deg below the flown tune's (the estimate's resolution); or the overshoot of the hover roll and pitch steps, or at the stick stops of your replay (any axis; a setpoint change of 200 deg/s or more in 0.1 s, then held within 10 % of it for 50 ms; the rate's peak past the setpoint in the 0.1 s after, of the change), is more than 10 % or the flown tune's, whichever is higher.
- Ms and phase margin: at hover, a chirp of +-20 (PID sum units, 5-400 Hz in 2 s) added to one axis's PID sum; S = 1/(1+L) from it and the PID sum the mixer gets, in bands of +-2.5 % from 8 to 300 Hz; Ms the largest |S|, the phase margin at the first band with |L| below 1.
- Stopped by: "still gaining when the search ended" means its last step was kept: the search's step budget, not the flight, stopped it.

## Not modelled

- Gyro noise, motor heat and D-term noise: a real quad with this much D may run hot; the class's D bound stands in for them.
- Frame resonances, bent props, the receiver link's jitter; the filters are not tuned.
- The plant is the identified one: where the identification is off, so is this.
