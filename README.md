# FLL 2026–2027 BIOGLOW

Robot code for the FIRST LEGO League **BIOGLOW** season, written in Python for
**LEGO SPIKE Prime / Pybricks**.

## Hardware

| Port | Device | Notes |
|---|---|---|
| B | Left drive motor | Counter-clockwise |
| D | Right drive motor | Clockwise |
| A | Attachment motor | Clockwise |
| F | Attachment motor | Clockwise |

- Drive base: wheel diameter **62.4 mm**, axle track **120 mm**
- Hub: `PrimeHub(top_side=Axis.Z, front_side=Axis.X)` — IMU used for straight driving

## How it runs

On start the hub turns **red**, stops all motors, waits 1 s, resets the IMU and
drive base, then turns **green**.

The display shows the selected program number:

- **LEFT / RIGHT** cycle through the programs
- **CENTER** runs the selected one

Pressing any hub button while a program is running raises an exception and stops
the robot (the `check()` guard inside every wait loop).

## Programs

| # | Function | Purpose |
|---|---|---|
| 0 | `run_0` | Calibration only |
| 1 | `run_1` | First Run — drive, attachments, pivots |
| 2 | `run_2` | Second Run — drive, attachments, pivots  |
| 3 | `run_3` |Third Run — drive, attachments, pivots  |

## Helper functions

- `fd(mm, speed)` / `bk(mm, speed)` — drive straight, speed optional
- `left(deg)` / `right(deg)` — gyro turns
- `pivot_left/right(deg)` — pivot on one wheel using `pivot_ratio`
- `a_turn(deg, speed)` / `f_turn(deg, speed)` — attachment motors
- `move_with_tools(d, aa, fa)` — drive while attachments move
- `stop()`, `reset()`, `calibrate()`

## Tuning

`wheel_diameter` and `axle_track` at the top drive distances and turns; adjust
them to your robot before a run. `apply(speed)` sets straight speed,
2 000 mm/s² acceleration and a 790 °/s turn rate.

## Status

**v2.0** — the version used for the current season's missions.
