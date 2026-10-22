from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import *
from pybricks.robotics import DriveBase
from pybricks.tools import wait,StopWatch

hub=PrimeHub(top_side=Axis.Z,front_side=Axis.X)

l=Motor(Port.B,Direction.COUNTERCLOCKWISE)
r=Motor(Port.D,Direction.CLOCKWISE)
a=Motor(Port.A,Direction.CLOCKWISE)
f=Motor(Port.F,Direction.CLOCKWISE)

drive=DriveBase(l,r,62.4,120)

gyro=True
wheel_diameter=62.4
axle_track=120
pivot_ratio=(2*axle_track)/wheel_diameter

def stop():
    drive.stop()
    l.stop()
    r.stop()
    a.stop()
    f.stop()

def check():
    if hub.buttons.pressed():
        raise Exception()

def reset():
    stop()
    drive.use_gyro(True)

    wait(500)

    hub.imu.reset_heading(0)

    t=StopWatch()
    while not hub.imu.ready() and t.time()<5000:
        wait(10)

    drive.reset()
    drive.use_gyro(True)

def start():
    hub.light.on(Color.RED)

    stop()

    wait(1000)

    reset()

    hub.light.on(Color.GREEN)

def apply(s=None):
    drive.settings(
        straight_speed=s or speed,
        straight_acceleration=2000,
        turn_rate=790,
        turn_acceleration=2000
    )

def wait_drive():
    while not drive.done():
        check()
        wait(5)

def fd(x,s=None):
    apply(s)
    drive.straight(
        abs(x),
        then=Stop.HOLD,
        wait=False
    )
    wait_drive()


def bk(x,s=None):
    apply(s)
    drive.straight(
        -abs(x),
        then=Stop.HOLD,
        wait=False
    )
    wait_drive()

def right(x):
    drive.turn(
        abs(x),
        then=Stop.HOLD,
        wait=True
    )


def left(x):
    drive.turn(
        -abs(x),
        then=Stop.HOLD,
        wait=True
    )

def calibrate():
    hub.light.on(Color.YELLOW)

    stop()

    wait(1000)

    hub.imu.reset_heading(0)

    wait(2000)

    drive.reset()
    drive.use_gyro(True)

    hub.light.on(Color.GREEN)
def a_turn(x,s=500):
    a.run_angle(s,x,then=Stop.HOLD)

def f_turn(x,s=500):
    f.run_angle(s,x,then=Stop.HOLD)

def move_with_tools(d,aa=0,fa=0,aspeed=500,fspeed=500):
    if aa:
        a.run_angle(aspeed,aa,then=Stop.HOLD,wait=False)
    if fa:
        f.run_angle(fspeed,fa,then=Stop.HOLD,wait=False)
    drive.straight(d,then=Stop.HOLD,wait=False)
    while not drive.done() or not a.done() or not f.done():
        check()
        wait(5)

def pivot(angle,moving,held,sign):
    held.hold()

    moving.run_angle(
        500,
        angle*pivot_ratio*sign,
        then=Stop.HOLD,
        wait=True
    )

    hub.imu.reset_heading(hub.imu.heading())

def pivot_right(x):
    pivot(x,l,r,1)

def pivot_left(x):
    pivot(x,r,l,-1)

def pivot_right_back(x):
    pivot(x,r,l,-1)

def pivot_left_back(x):
    pivot(x,l,r,1)

def run_0():
    calibrate()
def run_1():
    fd(420,.3)
    bk(200,.6)
    right(90)
    fd(200,.4)
    left(92)
    fd(440,.5)
    f_turn(-400)
    left(40)
    fd(200,100)
    f_turn(270)
    bk(190,.5)
    right(45)
    a_turn(-220,100)
    fd(100,100)
    a_turn(90,100)
    fd(170)
    a_turn(-15,100)
    bk(200)
    a_turn(-50)
    fd(90,100)
    left(5)
    a_turn(90,100)
    fd(120)
    a_turn(-20,100)
    wait(500)
    fd(10)
    bk(935,900)
    a_turn(90)

def run_2():
    fd(245,.55)
    right(90)
    fd(615)
    right(90)
    bk(200)
    f_turn(-150,100)
    f_turn(50,900)
    fd(240)
    left(90)
    bk(615)
    left(90)
    bk(245)

def run_3():
    fd(640,.2)
    left(50)

programs=[run_0,run_1,run_2,run_3]
sel=1

def choose():
    global sel
    while True:
        hub.display.number(sel)
        p=hub.buttons.pressed()
        if p:
            wait(200)
            if Button.LEFT in p:
                sel-=1
                if sel<0: sel=len(programs)-1
            elif Button.RIGHT in p:
                sel+=1
                if sel>=len(programs): sel=0
            elif Button.CENTER in p:
                programs[sel]()
                stop()

start()

try:
    choose()
finally:
    stop()
    hub.light.off()