# =====================================================================
#  BIOGLOW — robot code, fully explained
#  LEGO SPIKE Prime running Pybricks
#
#  HOW TO USE
#    LEFT / RIGHT on the hub  = choose a program (0 to 5 shown on screen)
#    CENTER                   = run the program shown
#    CENTER again             = stop the robot and go back to the menu
#    0 is always "calibrate"; the others are your mission runs.
#
#  HOW THE NUMBERS IN THE RUNS WORK
#    fd(200, .2, .1)
#        200  distance in millimetres (how far to drive, always positive)
#        .2   speed.  A fraction below 1 means "this share of full speed",
#             so .2 = 20% of 900 mm/s.  A number above 1 is taken literally.
#        .1   acceleration.  Same idea: below 1 it is a share of full
#             acceleration, above 1 it is literal.  Small = gentle start,
#             big = snappy start.
#    bk(x) and left(x) / right(x) take the same three arguments.
#
#  THE ONE CHECK
#    Everything that waits — every straight, every turn, every attachment
#    swing — goes through check().  That single function watches the button,
#    the clock and the motors, so the robot can always be stopped and can
#    never hang waiting for something that will not finish.
# =====================================================================

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Axis, Button, Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait

# The hub, lying flat, with the screen facing the same way as the robot's front.
hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)

# ---- robot measurements -------------------------------------------------
WHEEL = 62.4                       # wheel diameter, millimetres
TRACK = 120.0                      # distance between the two wheels, millimetres
PIVOT_RATIO = (2 * TRACK) / WHEEL  # wheel degrees needed per degree of pivot

# ---- driving defaults ---------------------------------------------------
BASE_SPEED = 900.0        # full speed, mm/s
BASE_ACCEL = 2000.0       # full acceleration, mm/s per second
TURN_RATE = 790.0         # degrees per second while turning
GYRO = True               # let the gyroscope steer straight lines
CORRECT = True            # nudge back onto the compass heading after each move
HEADING_TOL = 0.6         # degrees of error worth correcting
MOVE_TIMEOUT = 30000      # safety: never wait more than 30 s for one move
IMU_TIMEOUT = 5000        # safety: never wait more than 5 s for the gyroscope


def stage(n):
    """Show a number on the hub so a silent crash can still be located."""
    try:
        hub.display.number(n)
    except Exception:
        pass


stage(1)

# The factory stop button is CENTER and we want CENTER for ourselves, so the
# firmware stop is switched off and our own check() watches CENTER instead.
try:
    hub.system.set_stop_button(None)
except Exception:
    try:
        hub.system.set_stop_button(Button.BLUETOOTH)
    except Exception:
        pass

# ---- motors -------------------------------------------------------------
left_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, Direction.CLOCKWISE)

drive = DriveBase(left_motor, right_motor, WHEEL, TRACK)
drive.use_gyro(GYRO)
drive.settings(BASE_SPEED, BASE_ACCEL, TURN_RATE, BASE_ACCEL)

# Attachments may be unplugged; the robot must still start without them.
try:
    tool_a = Motor(Port.A, Direction.CLOCKWISE)
except Exception:
    tool_a = None

try:
    tool_f = Motor(Port.F, Direction.CLOCKWISE)
except Exception:
    tool_f = None


# =====================================================================
#  THE ONE CHECK — buttons, timeouts and waiting, all in one place
# =====================================================================

class Aborted(Exception):
    """Raised when CENTER is pressed during a run."""
    pass


def check(thing=None, timeout=MOVE_TIMEOUT):
    """
    The only checking function in the program.  Call it with nothing to test
    the button, or with a motor to wait for that motor.

      check()          → returns True, or raises Aborted if CENTER is held
      check(drive)     → waits until the drive base has finished moving
      check(tool_a)    → waits until that attachment motor has finished

    With a motor it also gives up after `timeout` milliseconds, stopping the
    motor instead of hanging forever, and returns False in that case.
    """
    if thing is None:
        if Button.CENTER in hub.buttons.pressed():
            raise Aborted()
        return True

    watch = StopWatch()
    while not thing.done():
        if Button.CENTER in hub.buttons.pressed():
            raise Aborted()
        if watch.time() > timeout:
            thing.stop()
            return False
        wait(5)
    return True


def steady():
    """
    Wait for the gyroscope to settle, then pause for a moment so the reading
    is calm before it is used.  If the hub has no IMU, this returns at once.
    """
    watch = StopWatch()
    ready = False
    while not ready and watch.time() < IMU_TIMEOUT:
        try:
            ready = hub.imu.ready()
        except Exception:
            ready = True          # no gyroscope: nothing to wait for
        if not ready:
            wait(10)
    wait(300)


def stop():
    """Stop everything, now."""
    drive.stop()
    left_motor.stop()
    right_motor.stop()
    if tool_a:
        tool_a.stop()
    if tool_f:
        tool_f.stop()


def wait_release():
    """Wait until no button is held, so one press is not read twice."""
    while hub.buttons.pressed():
        wait(20)
    wait(60)


def wait_press():
    """Block until a button is pressed and return which one."""
    pressed = ()
    while not pressed:
        pressed = hub.buttons.pressed()
        wait(20)
    return pressed


# =====================================================================
#  STEERING HELPERS
# =====================================================================

def heading_now():
    """The direction the robot faces, in degrees. 0 means "no error here"."""
    try:
        return hub.imu.heading()
    except Exception:
        return 0.0


def wrap180(value):
    """Fold any angle into -180..180, so 350 becomes -10."""
    while value > 180:
        value -= 360
    while value < -180:
        value += 360
    return value


def tune(s, a):
    """
    Apply this command's own speed and acceleration before it starts.
    Below 1 the number is a share of full; above 1 it is literal.  None
    means "use the default", which is how the old code behaved.
    """
    if s is None:
        speed = BASE_SPEED
    elif s <= 1:
        speed = BASE_SPEED * s
    else:
        speed = s

    if a is None:
        accel = BASE_ACCEL
    elif a <= 1:
        accel = BASE_ACCEL * a
    else:
        accel = a

    drive.settings(speed, accel, TURN_RATE, accel)


def reset():
    """Zero the gyroscope and the wheel counters before a run."""
    stop()
    wait(300)
    steady()
    try:
        hub.imu.reset_heading(0)
    except Exception:
        pass
    try:
        drive.reset()
        drive.use_gyro(GYRO)
    except Exception:
        pass


def start():
    """The sequence at power-on: red light, settle, calibrate, green light."""
    hub.light.on(Color.RED)
    stop()
    wait(1000)
    reset()
    hub.light.on(Color.GREEN)


def hold_heading(target):
    """
    After a straight line the robot may be a degree or two off the compass.
    This nudges it back, which is what keeps the long runs accurate.
    """
    if not CORRECT:
        return
    try:
        error = wrap180(heading_now() - target)
        if abs(error) > HEADING_TOL:
            drive.turn(-error, then=Stop.HOLD, wait=False)
            check(drive)
    except Exception:
        pass


# =====================================================================
#  MOVEMENTS — each one drives, then waits through check()
# =====================================================================

def fd(x, s=None, a=None):
    """Drive forward x millimetres at speed s and acceleration a."""
    target = heading_now()
    tune(s, a)
    drive.straight(abs(x), then=Stop.HOLD, wait=False)
    check(drive)
    hold_heading(target)


def bk(x, s=None, a=None):
    """Drive backward x millimetres. The sign of x never matters."""
    target = heading_now()
    tune(s, a)
    drive.straight(-abs(x), then=Stop.HOLD, wait=False)
    check(drive)
    hold_heading(target)


def right(x, s=None, a=None):
    """Turn right by x degrees."""
    tune(s, a)
    target = wrap180(heading_now() + x)
    drive.turn(abs(x), then=Stop.HOLD, wait=False)
    check(drive)
    hold_heading(target)


def left(x, s=None, a=None):
    """Turn left by x degrees. left(-90) still means "left by 90"."""
    tune(s, a)
    target = wrap180(heading_now() - x)
    drive.turn(-abs(x), then=Stop.HOLD, wait=False)
    check(drive)
    hold_heading(target)


def calibrate():
    """Yellow light: put the robot down still, then it zeroes itself."""
    hub.light.on(Color.YELLOW)
    stop()
    wait(1000)
    steady()
    try:
        hub.imu.reset_heading(0)
    except Exception:
        pass
    wait(1200)
    try:
        drive.reset()
        drive.use_gyro(GYRO)
    except Exception:
        pass
    wait_release()
    hub.light.on(Color.GREEN)


def a_turn(x, s=500):
    """Turn attachment motor A by x degrees."""
    if tool_a:
        tool_a.run_angle(s, x, then=Stop.HOLD, wait=False)
        check(tool_a)


def f_turn(x, s=500):
    """Turn attachment motor F by x degrees."""
    if tool_f:
        tool_f.run_angle(s, x, then=Stop.HOLD, wait=False)
        check(tool_f)


def pivot(angle, moving, held, sign):
    """
    Spin on one wheel. `held` is braked so it cannot roll, `moving` turns,
    and the gyroscope is re-zeroed afterwards so the next move still knows
    which way is forward.
    """
    held.hold()
    moving.run_angle(500, angle * PIVOT_RATIO * sign, then=Stop.HOLD, wait=False)
    check(moving)
    try:
        hub.imu.reset_heading(heading_now())
    except Exception:
        pass


def pivot_right(x):
    pivot(x, left_motor, right_motor, 1)


def pivot_left(x):
    pivot(x, right_motor, left_motor, -1)


# =====================================================================
#  THE PROGRAMS
#
#  Every line is a movement, in the order the robot performs it:
#
#    fd(distance)              drive forward that many millimetres
#    bk(distance)              drive backward
#    left(angle) / right(angle) turn on the spot by that many degrees
#    a_turn(angle, speed)      swing attachment motor A
#    f_turn(angle, speed)      swing attachment motor F
#
#  Each of fd, bk, left and right accepts two optional arguments:
#
#    fd(200, .4, .2)   200 mm, 40% speed, 20% acceleration
#    fd(200, s=.4, a=.2)           the same thing, shortened
#
#  Leave an argument out and it uses the default, exactly as the old code did.
# =====================================================================

def run_0():
    calibrate()


def run_1():
    # The first mission: approach, turn, use both attachments, retreat.
    fd(420, .3)               # out of the start box, gently
    bk(200, .6)
    right(90)
    fd(200, .4)
    left(92)
    fd(440, .5)
    f_turn(-400)
    left(40)
    fd(200, 100)
    f_turn(270)
    bk(190, .5)
    right(45)
    a_turn(-220, 100)
    fd(100, .1)
    a_turn(90, 100)
    fd(170)
    a_turn(-15, 100)
    bk(200)
    a_turn(-50)
    fd(90, 100)
    #left(5)
    a_turn(90, 100)
    fd(120)
    a_turn(-20, 100)
    wait(500)
    fd(10)
    bk(935, 900)
    a_turn(90)


def run_2():
    fd(710, .5)
    f_turn(170, s=200)
    f_turn(-100, s=200)
    f_turn(100, s=500)
    f_turn(-90, s=200)
    bk(-700)
    f_turn(330)


def run_3():
    a_turn(400)
    fd(250, .2)
    a_turn(-2000)


def run_4():
    fd(600, .2)
    right(90)
    fd(275)
    a_turn(290)
    left(-94)
    fd(120, .3)
    a_turn(-60)
    fd(150, .1)
    bk(80)
    fd(100)
    a_turn(-40)
    bk(100, .1)
    right(45)
    a_turn(90)


def run_5():
    """
    Put your next mission here.
    It must contain at least one line: an empty function is a syntax error,
    and that alone stops the whole program from loading.
    """
    pass


programs = [run_0, run_1, run_2, run_3, run_4, run_5]
first_run = 1
sel = first_run


def select_and_run():
    """The menu: show a number, wait for a button, run or change selection."""
    global sel
    wait_release()
    while True:
        hub.display.number(sel)
        pressed = wait_press()
        wait_release()

        if Button.LEFT in pressed:
            sel = len(programs) - 1 if sel == 0 else sel - 1

        elif Button.RIGHT in pressed:
            sel = 0 if sel == len(programs) - 1 else sel + 1

        elif Button.CENTER in pressed:
            hub.light.on(Color.BLUE)
            reset()
            finished = True
            try:
                programs[sel]()
                stop()
                try:
                    hub.speaker.beep(1000, 200)
                except Exception:
                    pass
            except Aborted:
                stop()
                finished = False
            hub.light.on(Color.BLUE)
            wait_release()
            if finished:
                sel = first_run if sel >= len(programs) - 1 else sel + 1


stage(2)
start()
stage(0)

# If anything at all goes wrong, flash 99 in red rather than dying silently.
try:
    select_and_run()
except BaseException:
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
    stop()
    hub.light.off()
