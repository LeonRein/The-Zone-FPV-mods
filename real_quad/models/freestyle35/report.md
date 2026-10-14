# Modell „3.5" freestyle, 300 g, 4S“ (freestyle35)

Erzeugt von `fpvident` aus 6 Logs in 47 s. 11 PASS, 4 WARN.

Eingaben (craft.json): 0.3 kg, 4S, 3650 KV, Prop 3.5"×2.5", Top-Speed unbekannt km/h.

## Prüfungen
| Prüfung | Ergebnis | Detail |
|---|---|---|
| Segmentierung auto = Hand | PASS | IoU 1 (Schwelle 0,85) |
| Motormodell freier Lauf | PASS | 6,29 Hz RMS (Schwelle 12) |
| Motordynamik 5–30 Hz | WARN | gemessen/sim [1,1, 1,2, 1,2, 1,2] (0,8–1,2) |
| Hover-Erkennung | PASS | acc, 22.4 s |
| Hover-Drehzahl zwischen Logs | PASS | Streuung 1.7 % (Schwelle 6 %) |
| Rotation Roll/Pitch | WARN | Chirp, R² ≥ 0,9, Streuung ≤ 96 % |
| Rotation Yaw | WARN | R² 0,95, Streuung 4,1e+02 % |
| Ungleichgewicht konsistent | PASS | Streuung zwischen Logs [0,39, 0,5, 0,43] ×10⁻³ |
| Akku-Fit | PASS | vbat, RMS 0,15 V |
| ACC Rotor-Drag | PASS | R² 0,86, Unsicherheit 0,072 (Schwellen 0,5 / 0,15) |
| Top-Speed belegt | WARN | weder Vorgabe noch ACC-Bestimmung: 112 km/h ist eine Schätzung |
| Hover-Gas Sim vs. Log | PASS | 0.309 vs. 0.303 (±0,02) |
| Roll-t90 Sim vs. Log | PASS | 194.4 vs. 195.3 ms (±20 %) |
| Roll-Nachlauf Sim vs. Log | PASS | 2.0 vs. 3.0 ms (±25 %, mind. 3 ms) |
| Closed-Loop-Tracking | PASS | 0.03 (Schwelle 0,35) |

## Gemessen
| Größe | Wert | Woher |
|---|---|---|
| Motor/ESC a, b, c (je Motor) | s. u. | eRPM gegen DShot-Kommando |
| Schubkoeffizient kT | 3,401e-07 | Hover 234 Hz bei 0.3 kg (acc) |
| Roll/Pitch-Momentenverstärkung (→ Ixx, Iyy) | 62,8 / 46,8 | IV, Chirp |
| Yaw: J_rotor/Izz, kQ, Drehrichtung | 0,00159 | IV, Motormoment als Regressor |
| Momenten-Ungleichgewicht β | [0,71, -3,6, 1] ×10⁻³ m | eRPM-Differenzen im Hover |
| Akku V0, Sag R | 15,94 V, 0,141 | vbat |
| Rotor-Drag (linear) | 0,36 1/s | ACC-Koppelnavigation |

## Standardwerte / Annahmen
| Größe | Wert | Warum |
|---|---|---|
| Motorabstand (arm) | 55 mm | 80 mm × D/5,1" (nur absolute Trägheiten hängen daran) |
| Izz | 0.000592 kg·m² | 1,7 × mittlere Roll/Pitch-Trägheit (nur J_rotor/Izz ist gemessen) |
| Rumpf-Widerstand | cd_area [0,00461, 0,00552, 0,0139] | Referenz × (D/5,1")² |
| Wake/Turbulenz/VRS | turb 0.307 / 3.76 m/s | zurückgestellt (dirty_air-Kalibrierung), Referenz physikalisch skaliert |

## Logs und Segmente
| Log | Dauer | nutzbar | Quelle | Chirps | Auto vs. Hand (IoU) |
|---|---|---|---|---|---|
| mini_03_freestyle | 232 s | 230 s | segments.json | 0 | 1 (nur auto 0 s, nur Hand 0,1 s; Chirps 0/0) |
| mini_04_freestyle_chirp5 | 250 s | 248 s | segments.json | 5 | 1 (nur auto 0 s, nur Hand 0 s; Chirps 5/5) |
| mini_05_crash | 6 s | 4 s | segments.json | 0 | 0,98 (nur auto 0,1 s, nur Hand 0 s; Chirps 0/0) |
| mini_08_freestyle | 193 s | 188 s | segments.json | 0 | 1 (nur auto 0 s, nur Hand 0 s; Chirps 0/0) |
| mini_09_freestyle | 164 s | 162 s | segments.json | 0 | 1 (nur auto 0 s, nur Hand 0 s; Chirps 0/0) |
| mini_10_freestyle | 219 s | 217 s | segments.json | 0 | 1 (nur auto 0 s, nur Hand 0 s; Chirps 0/0) |

Automatische Segmentierung gegen die handgeprüften Fenster: zeitgewichtete IoU 1.

## Betaflight-Konfiguration (fc.json)
Aus dem Header von **mini_10_freestyle**: Betaflight 2026.6.2 (e0b7bb01b) STM32H743, PID-Loop 8000 Hz (looptime 125 µs, pid_process_denom 1), RX 251 Hz, PID R/P/Y [42, 63, 33]/[49, 71, 44]/[65, 104, 0].

## Motor/ESC
Modell je Motor: dω/dt = a·(u·V − ω/Kv) − b·ω² − c, 1 ms Totzeit.
- Gleichungsfehler-Fit (R² 0,403): a [5156, 5001, 5390, 4894], b [0,001592, 0,00165, 0,001785, 0,001702], c [-332, -628, -717, -296]
- Dynamik-Korrektur λ = 2,04 (a, b, c gemeinsam skaliert, stationär unverändert): freier Lauf 12,7 → 6,29 Hz RMS
- Endwerte: a [1,053e+04, 1,022e+04, 1,101e+04, 9996], b [0,003251, 0,00337, 0,003646, 0,003477], c [-678, -1,28e+03, -1,46e+03, -605]
- Antwort gemessen/simuliert (Roll+Pitch-Differenz) bei 5 Hz 1,1, 10 Hz 1,2, 20 Hz 1,2, 30 Hz 1,2
- Spannung: vbat × 0,9979
- Freier Lauf je Log [Hz RMS]: mini_03_freestyle 5.8, mini_04_freestyle_chirp5 6.4, mini_05_crash 9.5, mini_08_freestyle 5.5, mini_09_freestyle 6.9, mini_10_freestyle 6.1

## Hover und Schub
Hover aus ACC (|a| = 1 g, waagerecht, ruhig), 22.4 s: 234.1 Hz Rotordrehzahl, Motorkommando 0.303, Gas 0.305.
kT = m·g / Σω² = 3,401e-07 N/(rad/s)² (Masse 0.3 kg aus craft.json).
Je Log: mini_03_freestyle 235.2 Hz (2.5 s), mini_04_freestyle_chirp5 232.3 Hz (17.7 s), mini_10_freestyle 236.3 Hz (2.2 s)
Kontrolle der Regel ohne ACC an den ACC-Logs (Hz): mini_03_freestyle 233.8, mini_04_freestyle_chirp5 235.6, mini_08_freestyle 339.2, mini_09_freestyle 238.2, mini_10_freestyle 288.1

## Rotationsdynamik
Instrumentvariablen-Schätzung auf Chirps (96 s Chirp im Log), Band 1–30 Hz.
| Achse | Verstärkung | Dämpfung | R² | Streuung (4 Blöcke) |
|---|---|---|---|---|
| roll | 62,77 | -1,77 | 0,9 | 23 % |
| pitch | 46,8 | -1,06 | 0,97 | 96 % |
| yaw | -1,588 | -1,15 | 0,95 | 4,1e+02 % |
Roll/Pitch: rad/s² pro 10⁶ (rad/s)² → Ixx 0,000298, Iyy 0,000399 kg·m² (mit kT und arm).
Yaw: −J_rotor/Izz (×10⁻³) → J_rotor 9,4e-07 kg·m², kQ = J_rotor·b = 3,23e-09; yaw_motors_reversed = ON (Vorzeichen aus den Daten).
Drehraten-Dämpfung: [1,77, 1,06, 1,15] (aus dem Chirp-Fit).

## Trimm / Ungleichgewicht
Aus den Drehzahlunterschieden im Hover (22769 Samples): β = [0,706, -3,61, 1,04] ×10⁻³ m (Moment = β · Gesamtschub; Roll, Pitch, Yaw).
Je Log (×10⁻³): mini_03_freestyle [0.53, -2.92, 0.37], mini_04_freestyle_chirp5 [0.85, -3.85, 1.25], mini_10_freestyle [-0.08, -2.69, 0.32]

## Akku
Quelle: vbat. V = V0 − R·I (I = Strom-Proxy des Sims). V0 je Log: {'mini_03_freestyle': 15.63, 'mini_04_freestyle_chirp5': 15.47, 'mini_05_crash': 15.94, 'mini_08_freestyle': 15.43, 'mini_09_freestyle': 15.56, 'mini_10_freestyle': 15.57}.
battery_v0 = 15,94 V (frischester Akku), battery_r = 0,141, Entladung gefittet 0,315 V/min (exportiert 0: das Spiel simuliert nur den Sag), RMS 0,15 V.

## Luftwiderstand und Top-Speed
ACC-Koppelnavigation: 8 Abschnitte ab Hover (je bis 5 s, Startgeschwindigkeit und -neigung mitgeschätzt: RMS 1 m/s, 11°), 111 Blöcke à 0,25 s, Tempo p50/p90/p99 [3.9, 18.5, 22.1] m/s. Fit R² 0,86: Rotor-Drag 0,36 1/s (±7,2 %), Rumpf-Skala 0,02 (±1e+03 %).
Verwendet: Rotor-Drag aus dem ACC-Fit (1,85e-05); Rumpf-Widerstand Klassen-Standard (Referenz × (D/5,1")²): cd_area [0,00461, 0,00552, 0,0139] m².
Top-Speed des Modells (Vollgas, waagerecht, frischer Akku mit Sag): **112 km/h** bei 77° Neigung, 705 Hz Rotor; keine Vorgabe in craft.json.

## Propwash / Turbulenz
Log-Metrik (15–78 Hz Tracking-Fehler Roll/Pitch, °/s, Median): ruhig 0.77, nach Gas-Cut 0.9, Flips nach Cut 2.71; 99. Perzentil aller nutzbaren Fenster ohne Chirps 10.3 (Fenster (2865, 595, 62, 7595)). Zielwerte für die Kalibrierung (dirty_air in Godot: das p99 skaliert auf freestyle5 = 14,5 °/s bei 11,3).
**Zurückgestellt:** die Wake-/Turbulenz-Amplituden werden nicht hier gefittet, sondern mit dem dirty_air-Wake per Drohne in Godot kalibriert. plant.json behält die Referenzwerte, skaliert mit der induzierten Geschwindigkeit (Faktor 0,99): turb_base_ms 0.307, pw_turb_ms 3.76, wake_r0 0.0824 m.

## Validierung
Hover: Motorkommando Sim 0.309 / Log 0.303, Rotor 234 / 234 Hz (Modellspannung 14,45 V).
Roll-Sprünge (10, Stick-Replay durch Betaflight-Port + Modell): t90 Sim 194.4 ms / Log 195.3 ms, Nachlauf hinter dem Setpoint (bei 50 %) Sim 2.0 ms / Log 3.0 ms, Gyro-Abweichung Sim−Log 0.03 × Log-RMS (Median).
Ereignisse (Log, Zeit, Plateau °/s, t90 Log/Sim ms): mini_03_freestyle 27.52 s 700 193/190; mini_03_freestyle 156.7 s 700 128/122; mini_04_freestyle_chirp5 50.59 s 587 199/199; mini_04_freestyle_chirp5 4.92 s 508 216/223; mini_08_freestyle 21.55 s 662 199/238; mini_08_freestyle 27.02 s 510 198/211; mini_09_freestyle 5.68 s 525 137/140; mini_09_freestyle 154.32 s 487 221/223; mini_10_freestyle 186.96 s 700 110/107; mini_10_freestyle 187.93 s 700 120/122
