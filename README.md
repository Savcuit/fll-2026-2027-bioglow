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

## Changelog

### v2.0.1 — fixes
- **Fatal:** `speed` was used in `apply()` but never defined, so every call without an
  explicit speed (`fd(170)`, `bk(200)`, `fd(120)`, `fd(10)`…) raised `NameError` and killed
  the run part-way. `speed = 900` is now defined.
- **Speeds were fractions, not mm/s.** Values such as `fd(420, .3)` were passed straight to
  `drive.settings(straight_speed=...)`, i.e. 0.3 mm/s. `apply()` now treats `s <= 1` as a
  fraction of the default speed, so `.3` means 270 mm/s.
- `gyro` was declared and then ignored (`use_gyro(True)` hard-coded twice). It is now used.
- Buttons were still pressed when a program started, so `check()` aborted it immediately.
  New `wait_release()` called before running a program.
- Magic numbers `2000` / `790` moved to `accel` / `turn_rate`.

### v2.1 — restored working framework
- `main.py` is now the **full framework** (607 lines) that actually runs on the hub:
  fraction-based speed scaling, owed-distance/owed-heading tracking, fine angle and fine
  distance correction, gyro-noise / motor-deadband / top-speed calibration, pivots, arcs,
  tool (A/F) helpers, abort on any button, and a program map selected on the hub display.
- The earlier 236-line "v2.0" was a simplified reconstruction and is **not** the shipped code.
  It is retained in git history only (commit `d856305`).
- Run `run_0` once after flashing: it measures gyro noise and motor deadband, and adapts the
  tolerances and top speed for that session.

## Controls (simple.py)

| Action | Result |
|---|---|
| **LEFT** | previous program |
| **RIGHT** | next program |
| **CENTER** | first press runs the shown program |
| **CENTER again while running** | stops the robot, returns to the number |
| **any other button during a run** | also stops the robot |
