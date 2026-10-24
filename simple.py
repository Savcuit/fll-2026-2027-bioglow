# =============================================================================
# simple.py - the small, single-file mission runner for the BIOGLOW robot.
# =============================================================================
#
# WHAT THIS PROGRAM DOES, IN PLAIN WORDS
#
#   1. When you switch the hub on and this program starts, it prepares itself:
#      it stops the motors, waits for the gyroscope to wake up, and sets the
#      robot's "zero" position.
#   2. It shows a number on the small screen, e.g. 1.
#   3. LEFT and RIGHT move between programs. CENTER runs the one shown.
#   4. While a program is running, pressing ANY button stops the robot and
#      brings you back to the number, ready to choose again.
#
# Three things are written so mistakes cannot kill the whole program:
#   - The two attachment motors (ports A and F) are optional. If one is
#     unplugged, the robot still starts instead of dying on the first line.
#   - The gyroscope is given time to wake up before it is used. Using it too
#     early is an error on this hub.
#   - "0.3" as a speed means 30% of the normal driving speed, not 0.3 mm per
#     second, which would look like the robot had frozen.
#
# =============================================================================

# ---------------------------------------------------------------- imports ----
# These lines borrow ready-made tools written by the Pybricks project.
# "PrimeHub" is the brain, "Motor" is a motor, "DriveBase" drives two motors
# together like a car, and "wait" pauses, "StopWatch" measures time.
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Axis, Button, Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait

# Create the hub and tell it how it is mounted: the light-up top faces up (Z),
# and the front of the robot is the hub's X side. This lets the hub know which
# way is "forward" and which way is "right".
hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)


# ---------------------------------------------------------------- helpers ----

def stage(n):
    # Shows a number on the hub screen as the program starts.
    # If the program ever dies, the last number stays visible, so we can tell
    # exactly how far it got. Purely for us humans; it changes nothing.
    try:
        hub.display.number(n)
    except Exception:
        pass


stage(1)

# On this hub, the CENTER button is wired by the firmware to STOP the program.
# We need CENTER for ourselves ("run this program"), so we move the emergency
# stop to the Bluetooth button instead. If this hub's firmware refuses that,
# we simply switch the stop button off altogether.
stage(2)
try:
    hub.system.set_stop_button(Button.BLUETOOTH)
except Exception:
    try:
        hub.system.set_stop_button(None)
    except Exception:
        pass

# ------------------------------------------------------------- dimensions ----
# Measurements of the robot. "mm" means millimetres.
wheel_diameter = 62.4   # how tall a drive wheel is (its diameter)
axle_track = 120        # distance between the centres of the left and right wheels

# When the robot spins on one wheel (a "pivot"), the moving wheel has to turn
# further than the angle you want, because it travels around a circle. This
# factor converts the angle you ask for into the angle the wheel must turn.
pivot_ratio = (2 * axle_track) / wheel_diameter

# ---------------------------------------------------------------- settings ---
gyro = True      # True = use the gyroscope to drive straighter and turn accurately
speed = 900      # normal driving speed, in millimetres per second
turn_rate = 790  # how fast the robot is allowed to spin, in degrees per second
accel = 2000     # how quickly it may speed up or slow down

# ------------------------------------------------------------- the motors ----
# Build the two driving motors. Direction.COUNTERCLOCKWISE / CLOCKWISE say they
# are mounted as mirror images of each other, so both push the robot forward.
stage(3)
left_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, Direction.CLOCKWISE)

# Put both motors under one "drive base", which understands distances in mm and
# angles in degrees, instead of wheel rotations.
drive = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

# The attachment motors (the arms) are optional. If a cable is loose or the
# motor is missing, "Motor(...)" raises an error - so we catch it and store
# None instead, meaning "there is no such motor". The rest of the program then
# simply skips any instruction that would use it.
stage(4)
try:
    a = Motor(Port.A, Direction.CLOCKWISE)
except Exception:
    a = None
try:
    f = Motor(Port.F, Direction.CLOCKWISE)
except Exception:
    f = None


# ------------------------------------------------------- basic operations ----

def stop():
    # Cut power to everything, so the robot stops immediately.
    drive.stop()
    left_motor.stop()
    right_motor.stop()
    if a:
        a.stop()
    if f:
        f.stop()


def check():
    # The "panic button". If any button on the hub is down, this deliberately
    # raises an error, which the program catches further down: the robot stops
    # and you return to the program list. This is how CENTER can kill a run.
    if hub.buttons.pressed():
        raise Exception()


def wait_release():
    # Wait until all buttons are let go, then a fraction longer. Without this,
    # the same press could be counted twice (once to start, once to stop).
    while hub.buttons.pressed():
        wait(20)
    wait(50)


# --------------------------------------------------- preparing to drive -----

def imu_ready():
    # The gyroscope ("IMU") needs a moment after power-on before it reports
    # anything. ready() answers True when it is awake. On firmware without that
    # function, we assume it is ready rather than crashing.
    try:
        return hub.imu.ready()
    except Exception:
        return True


def settle_imu():
    # Wait for the gyroscope to wake up, at most 5 seconds, then a short extra
    # pause so the readings are steady.
    t = StopWatch()
    while not imu_ready() and t.time() < 5000:
        wait(10)
    wait(300)


def reset():
    # Put the robot back to a known state: no motion, gyroscope pointing at
    # zero degrees, and the travelled distance set back to zero.
    stop()

    wait(300)
    settle_imu()

    # Each of these can fail on unusual hardware; a failure here must not stop
    # the whole program, so they are wrapped.
    try:
        hub.imu.reset_heading(0)
    except Exception:
        pass

    try:
        drive.reset()
        drive.use_gyro(gyro)
    except Exception:
        pass


def start():
    # The start-up light show: red while preparing, green when ready to drive.
    hub.light.on(Color.RED)
    stop()
    wait(1000)
    reset()
    hub.light.on(Color.GREEN)


# ------------------------------------------------------ moving the robot ----

def apply(s=None):
    # Choose how fast the next movement happens.
    #   no value  -> the normal speed (900 mm per second)
    #   0.3       -> 30% of the normal speed (270 mm per second)
    #   100       -> exactly 100 mm per second
    if s is None:
        v = speed
    elif s <= 1:
        v = speed * s
    else:
        v = s

    drive.settings(
        straight_speed=v,
        straight_acceleration=accel,
        turn_rate=turn_rate,
        turn_acceleration=accel,
    )


def wait_drive():
    # The robot is told to move in the background, and this loop waits for it.
    # Every 5 ms it also checks the buttons, so a press can interrupt the move.
    # If a move somehow takes longer than 30 seconds, it is stopped to prevent
    # the robot driving on for ever.
    t = StopWatch()
    while not drive.done():
        check()
        if t.time() > 30000:
            drive.stop()
            return False
        wait(5)
    return True


def fd(x, s=None):
    # Drive FORWARD by x millimetres. abs() makes sure a negative number still
    # means forward. "then=Stop.HOLD" means brake and hold position at the end.
    apply(s)
    drive.straight(abs(x), then=Stop.HOLD, wait=False)
    wait_drive()


def bk(x, s=None):
    # Drive BACKWARD by x millimetres.
    apply(s)
    drive.straight(-abs(x), then=Stop.HOLD, wait=False)
    wait_drive()


def wait_motor(m):
    # Same idea as wait_drive, but for a single attachment motor: wait for it
    # to finish while still watching for a button press.
    t = StopWatch()
    while not m.done():
        check()
        if t.time() > 30000:
            m.stop()
            return
        wait(5)


def right(x):
    # Turn clockwise (to the robot's right) by x degrees, in place.
    apply()
    drive.turn(abs(x), then=Stop.HOLD, wait=False)
    wait_drive()


def left(x):
    # Turn anticlockwise (to the robot's left) by x degrees, in place.
    apply()
    drive.turn(-abs(x), then=Stop.HOLD, wait=False)
    wait_drive()


def calibrate():
    # Program 0. The robot sits still while we re-zero the gyroscope, so it
    # knows exactly which way "straight ahead" is. Yellow light while working,
    # green when done.
    hub.light.on(Color.YELLOW)
    stop()
    wait(1000)
    settle_imu()
    try:
        hub.imu.reset_heading(0)
    except Exception:
        pass
    wait(1500)
    try:
        drive.reset()
        drive.use_gyro(gyro)
    except Exception:
        pass
    wait_release()
    hub.light.on(Color.GREEN)


def a_turn(x, s=500):
    # Rotate the arm on port A by x degrees. Skipped if that motor is missing.
    if a:
        a.run_angle(s, x, then=Stop.HOLD, wait=False)
        wait_motor(a)


def f_turn(x, s=500):
    # Rotate the arm on port F by x degrees. Skipped if that motor is missing.
    if f:
        f.run_angle(s, x, then=Stop.HOLD, wait=False)
        wait_motor(f)


def pivot(angle, moving, held, sign):
    # Spin around one wheel: the "held" wheel is locked, the "moving" wheel
    # travels, and the robot rotates on the spot. After the turn the gyroscope
    # is re-zeroed, so the next straight line is judged from this new heading.
    held.hold()
    moving.run_angle(500, angle * pivot_ratio * sign, then=Stop.HOLD, wait=True)
    try:
        hub.imu.reset_heading(hub.imu.heading())
    except Exception:
        pass


def pivot_right(x):
    pivot(x, left_motor, right_motor, 1)


def pivot_left(x):
    pivot(x, right_motor, left_motor, -1)


def pivot_right_back(x):
    pivot(x, right_motor, left_motor, -1)


def pivot_left_back(x):
    pivot(x, left_motor, right_motor, 1)


# ------------------------------------------------------------- the runs ------
# Each function below is one mission run: a list of moves in order, read from
# top to bottom. Distances are in millimetres, angles in degrees.
# The second number (when given) is the speed: 0.3 means 30% of normal speed,
# while 100 or 900 mean exact millimetres per second.

def run_0():
    # Program 0: do nothing but calibrate the gyroscope on the field.
    calibrate()


def run_1():
    # Program 1: the long mission run.
    fd(420, .3)     # forward 420 mm at 30% speed
    bk(200, .6)     # back 200 mm at 60%
    right(90)       # turn right 90 degrees
    fd(200, .4)
    left(92)
    fd(440, .5)
    f_turn(-400)    # swing the port F arm by -400 degrees
    left(40)
    fd(200, 100)    # forward 200 mm at exactly 100 mm per second (slow, precise)
    f_turn(270)
    bk(190, .5)
    right(45)
    a_turn(-220, 100)  # port A arm, -220 degrees, speed 100 deg per second
    fd(100, 100)
    a_turn(90, 100)
    fd(170)         # no speed given, so normal speed
    a_turn(-15, 100)
    bk(200)
    a_turn(-50)
    fd(90, 100)
    left(5)         # a tiny 5 degree correction
    a_turn(90, 100)
    fd(120)
    a_turn(-20, 100)
    wait(500)       # pause half a second
    fd(10)
    bk(935, 900)    # a long fast reverse: 935 mm at 900 mm per second
    a_turn(90)


def run_2():
    # Program 2: the second mission run.
    fd(245, .55)
    right(90)
    fd(615)
    right(90)
    bk(200)
    f_turn(-150, 100)
    f_turn(50, 900)
    fd(240)
    left(90)
    bk(615)
    left(90)
    bk(245)


def run_3():
    # Program 3: a short alignment move.
    fd(640, .2)
    left(50)


# --------------------------------------------------------- program list ------
# The choices in the order they appear when you press LEFT / RIGHT.
programs = [run_0, run_1, run_2, run_3]

sel = 1  # the program shown first: 1 = run_1

# ------------------------------------------------------------ the menu -------
stage(5)
start()          # stop, wait, zero the gyroscope, light green
stage(0)
hub.display.number(sel)

try:
    # This loop runs for ever: show the number, react to buttons, repeat.
    while True:
        hub.display.number(sel)

        pressed = hub.buttons.pressed()
        if not pressed:
            wait(20)        # nothing pressed, rest a moment and look again
            continue

        wait(200)           # small delay so one press is not read many times

        if Button.LEFT in pressed:
            # step down the list, wrapping around to the last program
            sel = len(programs) - 1 if sel == 0 else sel - 1

        elif Button.RIGHT in pressed:
            # step up the list, wrapping around to the first program
            sel = 0 if sel == len(programs) - 1 else sel + 1

        elif Button.CENTER in pressed:
            # Run the chosen program. wait_release() makes sure the CENTER
            # press itself is over before the run begins, otherwise the run
            # would stop instantly.
            wait_release()
            try:
                programs[sel]()
                stop()
            except Exception:
                # Stopped by a button press during the run: halt everything and
                # put the number back on the screen, ready for the next choice.
                stop()
                hub.display.number(sel)

            try:
                hub.speaker.beep(1000, 200)   # short confirmation beep
            except Exception:
                pass

        wait_release()

except BaseException:
    # If anything unexpected goes wrong, do not vanish silently: blink 99 in
    # red so it is obvious the program crashed, and which problem to look for.
    while True:
        try:
            hub.display.number(99)
            hub.light.on(Color.RED)
            wait(300)
            hub.light.off()
            wait(300)
        except Exception:
            pass

finally:
    # Whenever the program does end, make sure the motors are off.
    stop()
    hub.light.off()
