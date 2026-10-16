# Optimized tune of 5" freestyle, 650 g, 6S

How this quad would probably fly when well tuned: its identified plant flown in the simulator with the Betaflight settings below (a drone flies them when its tune setting in real_quad's settings is "optimized"). **Simulator only** - not a tune for the real quad: no noise, motor heat or frame resonances are simulated, and the filters stay as flown.

## Settings

| Setting | Flown | Tuned | Stopped by (the last tries) |
|---|---|---|---|
| Roll P | 45 | 53 | its last step still gained |
| Roll D | 33 | 37 | no better flight |
| Roll D-max | 33 | 37 | no better flight |
| Roll Feedforward | 125 | 175 | no better flight |
| Roll I | 55 | 55 | no better flight |
| Pitch P | 42 | 63 | its last step still gained |
| Pitch D | 37 | 49 | its last step still gained |
| Pitch D-max | 37 | 49 | no better flight |
| Pitch Feedforward | 130 | 183 | no better flight |
| Pitch I | 58 | 58 | no better flight |
| Yaw P | 50 | 50 | no better flight |
| Yaw Feedforward | 140 | 140 | no better flight |
| Yaw I | 80 | 80 | no better flight |
| TPA rate (%) | 45 | 30 | its last step still gained |
| TPA breakpoint | 1350 | 1450 | no better flight |
| Dynamic idle (x100 rpm) | 40 | 40 | no better flight |
| Anti-gravity gain | 100 | 100 | no better flight |
| Thrust linear (%) | 0 | 15 | its last step still gained |

## Predicted changes (simulator, the same flights for both)

| Flight | Flown | Tuned | Change |
|---|---|---|---|
| Stick steps, tracking error (deg/s) | 15.9 | 15.2 | -5 % |
| Flips, tracking error (deg/s) | 77.9 | 76.6 | -2 % |
| Your stick replay, tracking error (deg/s) | 4.8 | 3.5 | -27 % |
| Drop and catch, worst prop wash (deg/s) | 15.2 | 13.5 | -11 % |
| Hover roll step: time to 90 % (ms) | 18 | 16 | |
| Hover roll and pitch steps: overshoot (%) | 4 | 4 | at most 10 |
| Out of motor authority (% of the time) | 1.10 | 1.40 | at most +0.5 |
| Ringing, plant with 80 % inertia and 150 % motor delay (deg/s) | 3.23 | 3.12 | at most 3.88 |

## How

- Flights: steps_hover, steps_idle, steps_punch, flip, replay m5_04_freestyle#1 60 s, replay m5_04_freestyle#1 50 s, replay m5_04_freestyle#1 32 s.
- Search: staged coordinate search from the flown tune (P, D, D-max, feedforward, I per axis; then TPA, dynamic idle, anti-gravity, thrust linear), 84 evaluations, 116 s.
- Bounds: the five class's (P, I, feedforward around Betaflight's defaults; highest D at most 1.50 x the default D-max with the RPM filter; D/P band), widened to the flown values.
- Constraints: no more time out of motor authority than the flown tune; no more ringing than the flown tune on a plant with less inertia and more motor delay (the identification's error); hover steps overshoot at most 10 % (or as flown).

## Not modelled

- Gyro noise, motor heat and D-term noise: a real quad with this much D may run hot; the class's D bound stands in for them.
- Frame resonances, bent props, the receiver link's jitter; the filters are not tuned.
- The plant is the identified one: where the identification is off, so is this.
