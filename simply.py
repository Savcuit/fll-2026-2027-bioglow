from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Axis, Button, Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait

hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)

def stage(n):
    try:
        hub.display.number(n)
    except Exception:
        pass

stage(1)

stage(2)
try:
    hub.system.set_stop_button(Button.BLUETOOTH)
except Exception:
    try:
        hub.system.set_stop_button(None)
    except Exception:
        pass

wheel_diameter = 62.4
axle_track = 120

pivot_ratio = (2 * axle_track) / wheel_diameter

gyro = True
speed = 900
turn_rate = 790
accel = 2000

stage(3)
left_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, Direction.CLOCKWISE)

drive = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

stage(4)
try:
    a = Motor(Port.A, Direction.CLOCKWISE)
except Exception:
    a = None
try:
    f = Motor(Port.F, Direction.CLOCKWISE)
except Exception:
    f = None

def stop():
    drive.stop()
    left_motor.stop()
    right_motor.stop()
    if a:
        a.stop()
    if f:
        f.stop()

class Aborted(Exception):
    pass

def check():
    if hub.buttons.pressed():
        raise Aborted()

def wait_release():
    while hub.buttons.pressed():
        wait(20)
    wait(50)

def wait_press():
    p = ()
    while not p:
        p = hub.buttons.pressed()
        wait(20)
    return p

def imu_ready():
    try:
        return hub.imu.ready()
    except Exception:
        return True

def settle_imu():
    t = StopWatch()
    while not imu_ready() and t.time() < 5000:
        wait(10)
    wait(300)

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
        drive.use_gyro(gyro)
    except Exception:
        pass

def start():
    hub.light.on(Color.RED)
    stop()
    wait(1000)
    reset()
    hub.light.on(Color.GREEN)

def apply(s=None):
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
    t = StopWatch()
    while not drive.done():
        check()
        if t.time() > 30000:
            drive.stop()
            return False
        wait(5)
    return True

def fd(x, s=None):
    apply(s)
    drive.straight(abs(x), then=Stop.HOLD, wait=False)
    wait_drive()

def bk(x, s=None):
    apply(s)
    drive.straight(-abs(x), then=Stop.HOLD, wait=False)
    wait_drive()

def wait_motor(m):
    t = StopWatch()
    while not m.done():
        check()
        if t.time() > 30000:
            m.stop()
            return
        wait(5)

def right(x):
    apply()
    drive.turn(abs(x), then=Stop.HOLD, wait=False)
    wait_drive()

def left(x):
    apply()
    drive.turn(-abs(x), then=Stop.HOLD, wait=False)
    wait_drive()

def calibrate():
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
    if a:
        a.run_angle(s, x, then=Stop.HOLD, wait=False)
        wait_motor(a)

def f_turn(x, s=500):
    if f:
        f.run_angle(s, x, then=Stop.HOLD, wait=False)
        wait_motor(f)

def pivot(angle, moving, held, sign):
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
    fd(100, 100)
    a_turn(90, 100)
    fd(170)
    a_turn(-15, 100)
    bk(200)
    a_turn(-50)
    fd(90, 100)
    left(5)
    a_turn(90, 100)
    fd(120)
    a_turn(-20, 100)
    wait(500)
    fd(10)
    bk(935, 900)
    a_turn(90)

def run_2():
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
    fd(640, .2)
    left(50)

programs = [run_0, run_1, run_2, run_3]

first_run = 1

sel = first_run

def select_and_run():
    global sel
    wait_release()
    while True:
        hub.display.number(sel)
        p = wait_press()
        wait_release()

        if Button.LEFT in p:
            sel = len(programs) - 1 if sel == 0 else sel - 1

        elif Button.RIGHT in p:
            sel = 0 if sel == len(programs) - 1 else sel + 1

        elif Button.CENTER in p:
            hub.light.on(Color.BLUE)
            reset()
            ran = True
            try:
                programs[sel]()
                stop()
                try:
                    hub.speaker.beep(1000, 200)
                except Exception:
                    pass
            except Aborted:
                stop()
                ran = False
            hub.light.on(Color.BLUE)
            wait_release()
            if ran:
                sel = first_run if sel >= len(programs) - 1 else sel + 1

stage(5)
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
