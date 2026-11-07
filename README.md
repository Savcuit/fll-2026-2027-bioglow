# FLL 2026–2027 BIOGLOW — robot code

One file: **`main.py`** — the full framework, with the gyroscope, fine angle and distance
correction, PID pivots, stall detection and voltage check, carrying the current mission commands.

## Controls

| Action | Result |
|---|---|
| **LEFT / RIGHT** | choose a program: 0 to 5 on the screen |
| **CENTER** (first press) | run the program shown |
| **CENTER** (while running) | stop the robot and return to the number |
| other buttons | ignored |

The firmware's own stop button is switched off, so CENTER belongs entirely to the program.

## Programs

| Number | What it is |
|---|---|
| **0** | Calibrate — measures gyro noise, motor deadband and top speed, then sets its own tolerances. Run it on the field before a match. |
| **1** | Mission run, 27 steps |
| **2** | Attachment run, 7 steps |
| **3** | Short positioning run, 3 steps |
| **4** | Mission run, 14 steps |
| **5** | Empty — put your next mission here. A function with no body is a syntax error, so it contains `pass`. |

## Calling a movement

```
fd(200)                      200 mm forward, default speed and acceleration
fd(200, .2)                  200 mm at 20% speed
fd(200, .2, .1)              200 mm at 20% speed, 10% acceleration
fd(200, speed=.2, accel=.1)  the same, in keyword form
```

`bk`, `left` and `right` take the same arguments. A value at or below 1 is a share of full
(900 mm/s, 2000 mm/s²); above 1 it is literal. `a_turn` and `f_turn` take degrees and a motor speed.

## Hardware

| Port | Device | Direction |
|---|---|---|
| B | Left drive motor | Counter-clockwise |
| D | Right drive motor | Clockwise |
| A | Attachment motor | Clockwise |
| F | Attachment motor | Clockwise |

- Wheel diameter **62.4 mm**, axle track **120 mm**
- Hub: `PrimeHub(top_side=Axis.Z, front_side=Axis.X)`, gyroscope steering the straights

## The accuracy layer

- `fine_angle()` — PID heading correction (kp 7.0, ki 1.2, kd 0.9) to within 0.4°
- `fine_distance()` — two-pass correction to within 1.2 mm
- `straight()` — `owed_mm` / `owed_deg` bookkeeping, stall detection, 30 s timeout
- `pivot()` — PID pivot (kp 3.0, ki 0.4, kd 0.6) with trim and timeout
- `calibrate()` — measures this robot's gyro noise, deadband and top speed and adapts the tolerances

Pressing CENTER during a run stops it cleanly and returns to the menu.
