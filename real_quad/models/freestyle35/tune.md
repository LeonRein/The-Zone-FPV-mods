# Optimized tune of 3.5" freestyle, 300 g, 4S

How this quad would probably fly when well tuned: its identified plant flown in the simulator with the Betaflight settings below (a drone flies them when its tune setting in real_quad's settings is "optimized"). **Simulator only** - not a tune for the real quad: no noise, motor heat or frame resonances are simulated, and the filters stay as flown.

## Settings

| Setting | Flown | Tuned | Stopped by (the last tries) |
|---|---|---|---|
| Roll P | 42 | 59 | no better flight |
| Roll D | 33 | 39 | its last step still gained |
| Roll D-max | 33 | 39 | no better flight |
| Roll Feedforward | 135 | 169 | no better flight |
| Roll I | 63 | 63 | no better flight |
| Pitch P | 49 | 72 | its last step still gained |
| Pitch D | 44 | 44 | no better flight |
| Pitch D-max | 44 | 44 | no better flight |
| Pitch Feedforward | 165 | 206 | no better flight |
| Pitch I | 71 | 71 | no better flight |
| Yaw P | 65 | 65 | no better flight |
| Yaw Feedforward | 160 | 160 | no better flight |
| Yaw I | 104 | 104 | no better flight |
| TPA rate (%) | 50 | 35 | its last step still gained |
| TPA breakpoint | 1350 | 1450 | no better flight |
| Dynamic idle (x100 rpm) | 45 | 54 | its last step still gained |
| Anti-gravity gain | 110 | 110 | no better flight |
| Thrust linear (%) | 40 | 40 | up: class bound; down: no better flight |

## Predicted changes (simulator, the same flights for both)

| Flight | Flown | Tuned | Change |
|---|---|---|---|
| Stick steps, tracking error (deg/s) | 16.1 | 14.6 | -9 % |
| Flips, tracking error (deg/s) | 76.9 | 74.1 | -4 % |
| Your stick replay, tracking error (deg/s) | 11.7 | 8.6 | -27 % |
| Drop and catch, worst prop wash (deg/s) | 14.6 | 13.5 | -8 % |
| Hover roll step: time to 90 % (ms) | 36 | 16 | |
| Hover roll and pitch steps: overshoot (%) | 2 | 1 | at most 10 |
| Out of motor authority (% of the time) | 1.16 | 1.39 | at most +0.5 |
| Ringing, plant with 80 % inertia and 150 % motor delay (deg/s) | 2.75 | 2.71 | at most 3.30 |

## How

- Flights: steps_hover, steps_idle, steps_punch, flip, replay LOG00003#1 32 s, replay LOG00003#1 44 s, replay LOG00003#1 38 s.
- Search: staged coordinate search from the flown tune (P, D, D-max, feedforward, I per axis; then TPA, dynamic idle, anti-gravity, thrust linear), 85 evaluations, 148 s.
- Bounds: the mini class's (P, I, feedforward around Betaflight's defaults; highest D at most 1.50 x the default D-max with the RPM filter; D/P band), widened to the flown values.
- Constraints: no more time out of motor authority than the flown tune; no more ringing than the flown tune on a plant with less inertia and more motor delay (the identification's error); hover steps overshoot at most 10 % (or as flown).

## Not modelled

- Gyro noise, motor heat and D-term noise: a real quad with this much D may run hot; the class's D bound stands in for them.
- Frame resonances, bent props, the receiver link's jitter; the filters are not tuned.
- The plant is the identified one: where the identification is off, so is this.
