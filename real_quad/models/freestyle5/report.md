# Modell „5" freestyle, 650 g, 6S“ (freestyle5)

Erzeugt von `fpvident` aus 10 Logs in 72 s. 13 PASS, 2 WARN.

Eingaben (craft.json): 0.65 kg, 6S, 1960 KV, Prop 5.1"×4.99", Top-Speed 165 km/h.

## Prüfungen
| Prüfung | Ergebnis | Detail |
|---|---|---|
| Segmentierung auto = Hand | PASS | IoU 1 (Schwelle 0,85) |
| Motormodell freier Lauf | PASS | 8,54 Hz RMS (Schwelle 12) |
| Motordynamik 5–30 Hz | PASS | gemessen/sim [1,1, 1,1, 1,1, 1,1] (0,8–1,2) |
| Hover-Erkennung | PASS | acc, 51.2 s |
| Hover-Drehzahl zwischen Logs | PASS | Streuung 3.9 % (Schwelle 6 %) |
| Rotation Roll/Pitch | PASS | Chirp, R² ≥ 0,95, Streuung ≤ 3,2 % |
| Rotation Yaw | PASS | R² 0,95, Streuung 2,2 % |
| Ungleichgewicht konsistent | WARN | Streuung zwischen Logs [1,5, 2,8, 0,45] ×10⁻³ |
| Akku-Fit | PASS | vbat, RMS 0,22 V |
| ACC Rotor-Drag | PASS | R² 0,9, Unsicherheit 0,044 (Schwellen 0,5 / 0,15) |
| Top-Speed belegt | PASS | auf Vorgabe 165 km/h kalibriert |
| Hover-Gas Sim vs. Log | PASS | 0.238 vs. 0.245 (±0,02) |
| Roll-t90 Sim vs. Log | PASS | 152.9 vs. 153.4 ms (±20 %) |
| Roll-Nachlauf Sim vs. Log | WARN | 2.5 vs. 6.5 ms (±25 %, mind. 3 ms) |
| Closed-Loop-Tracking | PASS | 0.05 (Schwelle 0,35) |

## Gemessen
| Größe | Wert | Woher |
|---|---|---|
| Motor/ESC a, b, c (je Motor) | s. u. | eRPM gegen DShot-Kommando |
| Schubkoeffizient kT | 1,621e-06 | Hover 158 Hz bei 0.65 kg (acc) |
| Roll/Pitch-Momentenverstärkung (→ Ixx, Iyy) | 82,5 / 70,2 | IV, Chirp |
| Yaw: J_rotor/Izz, kQ, Drehrichtung | 0,00227 | IV, Motormoment als Regressor |
| Momenten-Ungleichgewicht β | [0,51, -1,1, -0,007] ×10⁻³ m | eRPM-Differenzen im Hover |
| Akku V0, Sag R | 25,34 V, 0,118 | vbat |
| Rotor-Drag (linear) | 0,29 1/s | ACC-Koppelnavigation |
| Rumpf-Widerstand | cd_area [0,0088, 0,0105, 0,0265] | Top-Speed-Vorgabe 165 km/h |

## Standardwerte / Annahmen
| Größe | Wert | Warum |
|---|---|---|
| Motorabstand (arm) | 80 mm | 80 mm × D/5,1" (nur absolute Trägheiten hängen daran) |
| Izz | 0.00291 kg·m² | 1,7 × mittlere Roll/Pitch-Trägheit (nur J_rotor/Izz ist gemessen) |
| Wake/Turbulenz/VRS | turb 0.31 / 3.8 m/s | zurückgestellt (dirty_air-Kalibrierung), Referenz physikalisch skaliert |

## Logs und Segmente
| Log | Dauer | nutzbar | Quelle | Chirps | Auto vs. Hand (IoU) |
|---|---|---|---|---|---|
| m5_01_chirp6 | 321 s | 318 s | segments.json | 6 | 1 (nur auto 0,1 s, nur Hand 1 s; Chirps 6/6) |
| m5_02_crash | 35 s | 31 s | segments.json | 0 | 1 (nur auto 0 s, nur Hand 0,1 s; Chirps 0/0) |
| m5_03_hover | 27 s | 21 s | segments.json | 0 | 1 (nur auto 0 s, nur Hand 0,1 s; Chirps 0/0) |
| m5_04_freestyle | 119 s | 114 s | segments.json | 0 | 1 (nur auto 0 s, nur Hand 0,2 s; Chirps 0/0) |
| m5_05_freestyle | 59 s | 56 s | segments.json | 0 | 0,99 (nur auto 0 s, nur Hand 0,2 s; Chirps 0/0) |
| m5_06_freestyle_abgeschnitten | 39 s | 37 s | segments.json | 0 | 0,99 (nur auto 0,2 s, nur Hand 0,2 s; Chirps 0/0) |
| m5_07_chirp9 | 387 s | 384 s | segments.json | 9 | 1 (nur auto 0 s, nur Hand 0,1 s; Chirps 9/9) |
| btfl_001 | 111 s | 109 s | auto | 0 | – |
| btfl_011 | 107 s | 105 s | auto | 0 | – |
| btfl_012 | 186 s | 184 s | auto | 0 | – |

Automatische Segmentierung gegen die handgeprüften Fenster: zeitgewichtete IoU 1.

## Betaflight-Konfiguration (fc.json)
Aus dem Header von **m5_07_chirp9**: Betaflight 2026.6.2 (e0b7bb01b) STM32F405, PID-Loop 4000 Hz (looptime 125 µs, pid_process_denom 2), RX 250 Hz, PID R/P/Y [45, 55, 33]/[42, 58, 37]/[50, 80, 0].

## Motor/ESC
Modell je Motor: dω/dt = a·(u·V − ω/Kv) − b·ω² − c, 1 ms Totzeit.
- Gleichungsfehler-Fit (R² 0,719): a [3791, 4021, 3983, 3853], b [0,002817, 0,002842, 0,002902, 0,002859], c [372, 496, 531, 453]
- Dynamik-Korrektur λ = 1,25 (a, b, c gemeinsam skaliert, stationär unverändert): freier Lauf 8,7 → 8,54 Hz RMS
- Endwerte: a [4742, 5030, 4982, 4820], b [0,003523, 0,003554, 0,00363, 0,003576], c [465, 621, 664, 566]
- Antwort gemessen/simuliert (Roll+Pitch-Differenz) bei 5 Hz 1,1, 10 Hz 1,1, 20 Hz 1,1, 30 Hz 1,1
- Spannung: vbat × 1,046
- Freier Lauf je Log [Hz RMS]: m5_01_chirp6 6.7, m5_02_crash 7.5, m5_03_hover 2.4, m5_04_freestyle 5.2, m5_05_freestyle 3.9, m5_06_freestyle_abgeschnitten 4.1, m5_07_chirp9 4.4, btfl_001 7.0, btfl_011 4.6, btfl_012 8.7

## Hover und Schub
Hover aus ACC (|a| = 1 g, waagerecht, ruhig), 51.2 s: 157.8 Hz Rotordrehzahl, Motorkommando 0.245, Gas 0.245.
kT = m·g / Σω² = 1,621e-06 N/(rad/s)² (Masse 0.65 kg aus craft.json).
Je Log: m5_01_chirp6 153.7 Hz (11.9 s), m5_03_hover 156.0 Hz (4.7 s), m5_04_freestyle 159.8 Hz (4.9 s), m5_07_chirp9 159.3 Hz (29.7 s)
Kontrolle der Regel ohne ACC an den ACC-Logs (Hz): m5_01_chirp6 175.5, m5_03_hover 154.7, m5_04_freestyle 149.3, m5_05_freestyle 160.7, m5_07_chirp9 161.3

## Rotationsdynamik
Instrumentvariablen-Schätzung auf Chirps (307 s Chirp im Log), Band 1–30 Hz.
| Achse | Verstärkung | Dämpfung | R² | Streuung (4 Blöcke) |
|---|---|---|---|---|
| roll | 82,46 | -2,1 | 0,95 | 2,5 % |
| pitch | 70,24 | -0,729 | 0,95 | 3,2 % |
| yaw | -2,269 | -0,931 | 0,95 | 2,2 % |
Roll/Pitch: rad/s² pro 10⁶ (rad/s)² → Ixx 0,00157, Iyy 0,00185 kg·m² (mit kT und arm).
Yaw: −J_rotor/Izz (×10⁻³) → J_rotor 6,6e-06 kg·m², kQ = J_rotor·b = 2,36e-08; yaw_motors_reversed = ON (Vorzeichen aus den Daten).
Drehraten-Dämpfung: [2,1, 0,729, 0,931] (aus dem Chirp-Fit).

## Trimm / Ungleichgewicht
Aus den Drehzahlunterschieden im Hover (51227 Samples): β = [0,512, -1,1, -0,00704] ×10⁻³ m (Moment = β · Gesamtschub; Roll, Pitch, Yaw).
Je Log (×10⁻³): m5_01_chirp6 [0.27, -2.87, 0.35], m5_03_hover [-2.86, 3.84, -0.8], m5_04_freestyle [0.4, 2.35, -0.49], m5_07_chirp9 [1.1, -1.76, 0.06]

## Akku
Quelle: vbat. V = V0 − R·I (I = Strom-Proxy des Sims). V0 je Log: {'m5_01_chirp6': 24.91, 'm5_02_crash': 25.34, 'm5_03_hover': 25.12, 'm5_04_freestyle': 24.4, 'm5_05_freestyle': 23.7, 'm5_06_freestyle_abgeschnitten': 23.36, 'm5_07_chirp9': 25.08}.
battery_v0 = 25,34 V (frischester Akku), battery_r = 0,118, Entladung gefittet 0,344 V/min (exportiert 0: das Spiel simuliert nur den Sag), RMS 0,22 V.

## Luftwiderstand und Top-Speed
ACC-Koppelnavigation: 16 Abschnitte ab Hover (je bis 5 s, Startgeschwindigkeit und -neigung mitgeschätzt: RMS 1,6 m/s, 14°), 268 Blöcke à 0,25 s, Tempo p50/p90/p99 [5.2, 22.8, 35.6] m/s. Fit R² 0,9: Rotor-Drag 0,29 1/s (±4,4 %), Rumpf-Skala 0,02 (±4,4e+02 %).
Verwendet: Rotor-Drag aus dem ACC-Fit (4,78e-05); Rumpf-Widerstand auf die Top-Speed-Vorgabe 165 km/h kalibriert (der ACC-Fit bestimmt ihn nicht: zu wenig schnelle Abschnitte): cd_area [0,0088, 0,0105, 0,0265] m².
Top-Speed des Modells (Vollgas, waagerecht, frischer Akku mit Sag): **165 km/h** bei 82° Neigung, 588 Hz Rotor; Vorgabe 165 km/h; mit Standard-Rumpfwiderstand wären es 159 km/h.

## Propwash / Turbulenz
Log-Metrik (15–78 Hz Tracking-Fehler Roll/Pitch, °/s, Median): ruhig 0.75, nach Gas-Cut 0.67, Flips nach Cut 4.28 (Fenster (4125, 849, 96)). Zielwerte für die Kalibrierung.
**Zurückgestellt:** die Wake-/Turbulenz-Amplituden werden nicht hier gefittet, sondern mit dem dirty_air-Wake per Drohne in Godot kalibriert. plant.json behält die Referenzwerte, skaliert mit der induzierten Geschwindigkeit (Faktor 1): turb_base_ms 0.31, pw_turb_ms 3.8, wake_r0 0.12 m.

## Validierung
Hover: Motorkommando Sim 0.238 / Log 0.245, Rotor 158 / 158 Hz (Modellspannung 23,79 V).
Roll-Sprünge (12, Stick-Replay durch Betaflight-Port + Modell): t90 Sim 152.9 ms / Log 153.4 ms, Nachlauf hinter dem Setpoint (bei 50 %) Sim 2.5 ms / Log 6.5 ms, Gyro-Abweichung Sim−Log 0.05 × Log-RMS (Median).
Ereignisse (Log, Zeit, Plateau °/s, t90 Log/Sim ms): m5_01_chirp6 232.32 s 388 198/200; m5_02_crash 20.2 s 590 157/155; m5_02_crash 22.23 s 513 206/192; m5_04_freestyle 53.09 s 700 100/98; m5_04_freestyle 60.1 s 700 150/151; m5_05_freestyle 18.24 s 700 189/192; m5_05_freestyle 22.13 s 700 127/123; m5_06_freestyle_abgeschnitten 23.79 s 700 125/130; m5_06_freestyle_abgeschnitten 13.81 s 662 230/246; m5_07_chirp9 11.25 s 700 133/132; m5_07_chirp9 31.31 s 700 105/103; btfl_001 98.87 s 502 162/164

## Regression gegen Referenzmodell
Gegen `real_quad/models/freestyle5/plant.json`:
| Größe | neu | Referenz | Abweichung |
|---|---|---|---|
| kT | 1,621e-06 | 1,45e-06 | 11,8 % |
| Ixx | 0,001573 | 0,002 | -21,4 % |
| Iyy | 0,001846 | 0,00203 | -9,1 % |
| Izz | 0,002906 | 0,0034 | -14,5 % |
| kQ | 2,355e-08 | 1,7e-08 | 38,5 % |
| J_rotor | 6,596e-06 | 5e-06 | 31,9 % |
| motor_a | [4742, 5030, 4982, 4820] | [3316, 3714, 3703, 3384] | [43, 35,4, 34,5, 42,4] % |
| motor_b | [0,003523, 0,003554, 0,00363, 0,003576] | [0,003235, 0,003425, 0,003493, 0,003333] | [8,9, 3,8, 3,9, 7,3] % |
| motor_c | [465,2, 620,7, 664,3, 566,1] | [-162,6, 67,7, 80, -198,2] | [386, 817, 730, 386] % |
| cd_area | [0,008798, 0,01054, 0,02651] | [0,00978, 0,01172, 0,02947] | [-10, -10,1, -10] % |
| rotor_drag | 4,775e-05 | 6,09e-05 | -21,6 % |
| torque_bias_per_thrust | [0,000512, -0,0011, -7,04e-06] | [-0,00054, -0,00076, 0,0015] | [195, -44,7, -100] % |
| battery_v0 | 25,34 | 26,5 | -4,4 % |
| battery_r | 0,1178 | 0,0825 | 42,8 % |
| turb_base_ms | 0,31 | 0,31 | 0 % |
| pw_turb_ms | 3,8 | 3,8 | 0 % |
| yaw_spin | [1, -1, -1, 1] | [1, -1, -1, 1] | [0, 0, 0, 0] % |
| rate_damping | [2,1, 0,729, 0,931] | [0, 0, 0] | [210, 72,9, 93,1] % |
Schlüssel identisch: ja
