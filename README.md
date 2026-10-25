# FLL 2026–2027 BIOGLOW — robot code

Mission code for the FIRST LEGO League **BIOGLOW** season, written in Python for
**LEGO SPIKE Prime running Pybricks**.

## The files in this repository

| File | What it is | Lines |
|---|---|---|
| **`bioglow_competition.py`** | The competition robot. Full framework: precise distances and headings, gyro and motor calibration, fine correction, every guard. This is the one that races. | 265 |
| **`bioglow_simple.py`** | A small, readable version of the same robot — choose a program on the hub screen and run it. No comments, easy to read and copy. | 317 |
| **`bioglow_simple_explained.py`** | Exactly the same program as `bioglow_simple.py`, but every block is explained in plain English for someone who has never programmed. | 478 |
| **`bioglow_full.py`** | The complete framework: every helper kept (pivots, arcs, all tool calls), the redundant try/except guards stripped, plus a new `move_and_arms()` that drives and moves both arms at once. No comments. | 332 |

All three compile and run on the same hardware. `bioglow_simple.py` and
`bioglow_simple_explained.py` were compared by compiled syntax tree and are identical
in behaviour — they differ only in commentary.

## Hardware

| Port | Device | Direction |
|---|---|---|
| B | Left drive motor | Counter-clockwise |
| D | Right drive motor | Clockwise |
| A | Attachment motor | Clockwise |
| F | Attachment motor | Clockwise |

- Drive base: wheel diameter **62.4 mm**, axle track **120 mm**
- Hub: `PrimeHub(top_side=Axis.Z, front_side=Axis.X)`, gyroscope used for straight driving

## Controls

| Action | Result |
|---|---|
| **LEFT** | previous program |
| **RIGHT** | next program |
| **CENTER** | runs the program shown |
| **CENTER again while running** | stops the robot, returns to the number |
| **any other button during a run** | also stops the robot |
| **Bluetooth button** | emergency stop set by the program |

The hub shows **red** while preparing, **green** when ready to drive, **blue** during a
mission, and blinks **99** in red if the program ever crashes.

## Programs

| # | Name | Purpose |
|---|---|---|
| 0 | `run_0` | Calibration only — measures gyro noise and motor deadband |
| 1 | `run_1` | First mission run |
| 2 | `run_2` | Second mission run |
| 3 | `run_3` | Short alignment move |

Run **program 0 once** after flashing the competition file: it adapts the tolerances and
top speed to the battery and motors of that session.

## Movement helpers

| Call | Meaning |
|---|---|
| `fd(420, .3)` | drive forward 420 mm at 30 % speed |
| `bk(200)` | drive back 200 mm at normal speed |
| `left(90)` / `right(90)` | turn 90° on the spot |
| `a_turn(90)` / `f_turn(-400)` | rotate the arm on port A or F |
| `stop()` / `reset()` / `calibrate()` | halt, re-zero, calibrate |

A speed given as `0.3` means 30 % of the default speed. A speed of `100` means exactly
100 mm per second.

## Tuning

`wheel_diameter` and `axle_track` at the top of each file set distances and turns — check
them against the robot before a competition. The framework file also exposes
`straight_segment_mm`, `move_timeout_ms` and the fine-correction tolerances.

## Status

**v2.2** — competition file condensed to 265 lines; all kept functions verified
identical to v2.1. Mission runs, calibration and controls unchanged.

## Changelog

### v2.2 — condensed competition file (607 → 265 lines)
Nothing that runs has changed. All 45 remaining functions were compared by syntax tree and
are identical to v2.1, and all 78 top-level statements are unchanged; only formatting was
joined and unused helpers removed:

`face`, `shortest`, `pivot*` (5), `arc*` (5), `tool_time/tool_to/tool_run/tool_stop/tool_reset/tool_angle`,
`a_time/a_to/a_run/a_stop/a_reset/a_angle`, `f_time/f_to/f_run/f_stop/f_reset/f_angle`, `wait_s`, `run_5`.

Restore any of them from tag `v2.1` if a run needs them.

### v2.1 — restored working framework
- The full framework that actually runs on the hub is the competition file: fraction-based
  speed scaling, owed-distance and owed-heading tracking, fine angle and fine distance
  correction, gyro-noise / motor-deadband / top-speed calibration, tool helpers, abort on any
  button, and a program map chosen on the hub display.
- The earlier 236-line reconstruction is retained in git history only (commit `d856305`).

### v2.0.1 — fixes to the small version
- **Fatal:** the default `speed` was used but never defined, so any call without an explicit
  speed raised `NameError` and killed the run part-way.
- Speeds such as `.3` were passed to the hub as **0.3 mm/s**; they are now read as a fraction
  of the default speed.
- `gyro` was declared and ignored — now used.
- Buttons were still held when a program started, so the abort guard fired immediately.
  A `wait_release()` now runs before each program.
- **CENTER is the firmware's stop button.** `set_stop_button(Button.BLUETOOTH)` moves the
  emergency stop, freeing CENTER to start a run.
