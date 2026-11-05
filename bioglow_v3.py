from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Axis, Button, Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait

hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)

WHEEL = 62.4
TRACK = 120.0
PIVOT_RATIO = (2 * TRACK) / WHEEL

BASE_SPEED = 900.0
BASE_ACCEL = 2000.0
TURN_RATE = 790.0
GYRO = True
CORRECT = True
HEADING_TOL = 0.6
MOVE_TIMEOUT = 30000


def stage(n):
    try:
        hub.display.number(n)
    except Exception:
        pass


def blank():
    pass


stage(1)

try:
    hub.system.set_stop_button(Button.BLUETOOTH)
except Exception:
    try:
        hub.system.set_stop_button(None)
    except Exception:
        pass

left_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, Direction.CLOCKWISE)

drive = DriveBase(left_motor, right_motor, WHEEL, TRACK)
drive.use_gyro(GYRO)
drive.settings(BASE_SPEED, BASE_ACCEL, TURN_RATE, BASE_ACCEL)

try:
    tool_a = Motor(Port.A, Direction.CLOCKWISE)
except Exception:
    tool_a = None

try:
    tool_f = Motor(Port.F, Direction.CLOCKWISE)
except Exception:
    tool_f = None


def stop():
    drive.stop()
    left_motor.stop()
    right_motor.stop()
    if tool_a:
        tool_a.stop()
    if tool_f:
        tool_f.stop()


class Aborted(Exception):
    pass


def check():
    if hub.buttons.pressed():
        raise Aborted()


def wait_release():
    while hub.buttons.pressed():
        wait(20)
    wait(60)


def wait_press():
    pressed = ()
    while not pressed:
        pressed = hub.buttons.pressed()
        wait(20)
    return pressed


def imu_ready():
    try:
        return hub.imu.ready()
    except Exception:
        return True


def settle_imu():
    watch = StopWatch()
    while not imu_ready() and watch.time() < 5000:
        wait(10)
    wait(300)


def heading_now():
    try:
        return hub.imu.heading()
    except Exception:
        return 0.0


def wrap180(value):
    while value > 180:
        value -= 360
    while value < -180:
        value += 360
    return value


def speed_of(s):
    if s is None:
        return BASE_SPEED
    if s <= 1:
        return BASE_SPEED * s
    return s


def accel_of(a):
    if a is None:
        return BASE_ACCEL
    if a <= 1:
        return BASE_ACCEL * a
    return a


def tune(s, a):
    speed = speed_of(s)
    accel = accel_of(a)
    drive.settings(speed, accel, TURN_RATE, accel)


def reset():
    stop()
    wait(300)
    settle_imu()
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
    hub.light.on(Color.RED)
    stop()
    wait(1000)
    reset()
    hub.light.on(Color.GREEN)


def wait_drive():
    watch = StopWatch()
    while not drive.done():
        check()
        if watch.time() > MOVE_TIMEOUT:
            drive.stop()
            return False
        wait(5)
    return True


def wait_motor(motor):
    watch = StopWatch()
    while not motor.done():
        check()
        if watch.time() > MOVE_TIMEOUT:
            motor.stop()
            return
        wait(5)


def hold_heading(target):
    if not CORRECT:
        return
    try:
        error = wrap180(heading_now() - target)
        if abs(error) > HEADING_TOL:
            drive.turn(-error, then=Stop.HOLD, wait=True)
    except Exception:
        pass


def fd(x, s=None, a=None):
    target = heading_now()
    tune(s, a)
    drive.straight(abs(x), then=Stop.HOLD, wait=False)
    wait_drive()
    hold_heading(target)


def bk(x, s=None, a=None):
    target = heading_now()
    tune(s, a)
    drive.straight(-abs(x), then=Stop.HOLD, wait=False)
    wait_drive()
    hold_heading(target)


def right(x, s=None, a=None):
    tune(s, a)
    target = wrap180(heading_now() + x)
    drive.turn(abs(x), then=Stop.HOLD, wait=False)
    wait_drive()
    hold_heading(target)


def left(x, s=None, a=None):
    tune(s, a)
    target = wrap180(heading_now() - x)
    drive.turn(-abs(x), then=Stop.HOLD, wait=False)
    wait_drive()
    hold_heading(target)


def calibrate():
    hub.light.on(Color.YELLOW)
    stop()
    wait(1000)
    settle_imu()
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
    if tool_a:
        tool_a.run_angle(s, x, then=Stop.HOLD, wait=False)
        wait_motor(tool_a)


def f_turn(x, s=500):
    if tool_f:
        tool_f.run_angle(s, x, then=Stop.HOLD, wait=False)
        wait_motor(tool_f)


def pivot(angle, moving, held, sign):
    held.hold()
    moving.run_angle(500, angle * PIVOT_RATIO * sign, then=Stop.HOLD, wait=True)
    try:
        hub.imu.reset_heading(heading_now())
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


def run_0():
    calibrate()


def run_1():
    fd(420, .3, .35)
    bk(200, .6)
    right(90, .5, .4)
    fd(200, .4)
    left(92, .4)
    fd(440, .5)
    f_turn(-400)
    left(40, .3)
    fd(200, 100, .5)
    f_turn(270)
    bk(190, .5)
    right(45, .3)
    a_turn(-220, 100)
    fd(100, .1, .3)
    a_turn(90, 100)
    fd(170)
    a_turn(-15, 100)
    bk(200)
    a_turn(-50)
    fd(90, 100, .3)
    a_turn(90, 100)
    fd(120)
    a_turn(-20, 100)
    wait(500)
    fd(10, .1)
    bk(935, 900)
    a_turn(90)


def run_2():
    fd(710, .5, .4)
    f_turn(170, 200)
    f_turn(-100, 200)
    f_turn(100, 500)
    f_turn(-90, 200)
    bk(700, .6)
    f_turn(330)


def run_3():
    a_turn(400)
    fd(250, .2, .3)
    a_turn(-2000)


def run_4():
    fd(600, .2, .3)
    right(90, .5, .4)
    fd(275)
    a_turn(290)
    left(94, .4)
    fd(120, .3)
    a_turn(-60)
    fd(150, .1, .25)
    bk(80)
    fd(100)
    a_turn(-40)
    bk(100, .1)
    right(45, .3)
    a_turn(90)


def run_5():
    fd(300, .3, .3)
    right(90, .4)
    fd(300, .3)
    bk(300, .3)
    left(90, .4)


programs = [run_0, run_1, run_2, run_3, run_4, run_5]
first_run = 1
sel = first_run


def select_and_run():
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
