from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Axis, Button, Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait

hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)

WHEEL = 62.4
TRACK = 120.0
SPEED = 900.0
ACCEL = 2000.0
TURN_RATE = 790.0
TOL = 0.6
TIMEOUT = 30000


def stage(n):
    try:
        hub.display.number(n)
    except Exception:
        pass


stage(1)

try:
    hub.system.set_stop_button(None)
except Exception:
    try:
        hub.system.set_stop_button(Button.BLUETOOTH)
    except Exception:
        pass

left_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, Direction.CLOCKWISE)
drive = DriveBase(left_motor, right_motor, WHEEL, TRACK)
drive.use_gyro(True)
drive.settings(SPEED, ACCEL, TURN_RATE, ACCEL)

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
    if Button.CENTER in hub.buttons.pressed():
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


def settle():
    watch = StopWatch()
    try:
        while not hub.imu.ready() and watch.time() < 5000:
            wait(10)
    except Exception:
        pass
    wait(300)


def heading():
    try:
        return hub.imu.heading()
    except Exception:
        return 0.0


def wrap180(v):
    while v > 180:
        v -= 360
    while v < -180:
        v += 360
    return v


def reset():
    stop()
    wait(300)
    settle()
    try:
        hub.imu.reset_heading(0)
    except Exception:
        pass
    try:
        drive.reset()
        drive.use_gyro(True)
    except Exception:
        pass


def tune(s, a):
    speed = SPEED if s is None else (SPEED * s if s <= 1 else s)
    accel = ACCEL if a is None else (ACCEL * a if a <= 1 else a)
    drive.settings(speed, accel, TURN_RATE, accel)


def wait_drive():
    watch = StopWatch()
    while not drive.done():
        check()
        if watch.time() > TIMEOUT:
            drive.stop()
            return
        wait(5)


def wait_motor(motor):
    watch = StopWatch()
    while not motor.done():
        check()
        if watch.time() > TIMEOUT:
            motor.stop()
            return
        wait(5)


def hold(target):
    try:
        error = wrap180(heading() - target)
        if abs(error) > TOL:
            drive.turn(-error, then=Stop.HOLD, wait=False)
            wait_drive()
    except Exception:
        pass


def fd(x, s=None, a=None, speed=None, accel=None):
    target = heading()
    tune(speed if speed is not None else s, accel if accel is not None else a)
    drive.straight(abs(x), then=Stop.HOLD, wait=False)
    wait_drive()
    hold(target)


def bk(x, s=None, a=None, speed=None, accel=None):
    target = heading()
    tune(speed if speed is not None else s, accel if accel is not None else a)
    drive.straight(-abs(x), then=Stop.HOLD, wait=False)
    wait_drive()
    hold(target)


def right(x, s=None, a=None, speed=None, accel=None):
    tune(speed if speed is not None else s, accel if accel is not None else a)
    target = wrap180(heading() + x)
    drive.turn(abs(x), then=Stop.HOLD, wait=False)
    wait_drive()
    hold(target)


def left(x, s=None, a=None, speed=None, accel=None):
    tune(speed if speed is not None else s, accel if accel is not None else a)
    target = wrap180(heading() - x)
    drive.turn(-abs(x), then=Stop.HOLD, wait=False)
    wait_drive()
    hold(target)


def a_turn(x, s=500):
    if tool_a:
        tool_a.run_angle(s, x, then=Stop.HOLD, wait=False)
        wait_motor(tool_a)


def f_turn(x, s=500):
    if tool_f:
        tool_f.run_angle(s, x, then=Stop.HOLD, wait=False)
        wait_motor(tool_f)


def calibrate():
    hub.light.on(Color.YELLOW)
    stop()
    wait(1000)
    settle()
    try:
        hub.imu.reset_heading(0)
    except Exception:
        pass
    wait(1200)
    try:
        drive.reset()
        drive.use_gyro(True)
    except Exception:
        pass
    wait_release()
    hub.light.on(Color.GREEN)


def run_0():
    calibrate()


def run_1():
    fd(420, .3)
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
    a_turn(90, 100)
    fd(120)
    a_turn(-20, 100)
    wait(500)
    fd(10)
    bk(935, 900)
    a_turn(90)


def run_2():
    fd(710, .5)
    f_turn(170, 200)
    f_turn(-100, 200)
    f_turn(100, 500)
    f_turn(-90, 200)
    bk(700)
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
    left(94)
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
    pass


programs = [run_0, run_1, run_2, run_3, run_4, run_5]
first_run = 1
sel = first_run


def menu():
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
            done = True
            try:
                programs[sel]()
                stop()
                try:
                    hub.speaker.beep(1000, 200)
                except Exception:
                    pass
            except Aborted:
                stop()
                done = False
            hub.light.on(Color.BLUE)
            wait_release()
            if done:
                sel = first_run if sel >= len(programs) - 1 else sel + 1


stage(2)
hub.light.on(Color.RED)
stop()
wait(1000)
reset()
hub.light.on(Color.GREEN)
stage(0)

try:
    menu()
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
