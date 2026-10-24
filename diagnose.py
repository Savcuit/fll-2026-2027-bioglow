# diagnose.py - find out why the small 236-line version dies before the display.
#
# Flash this INSTEAD of main.py, with the robot exactly as you race it
# (both attachment motors plugged in). Read the number on the hub:
#
#   1  drive motors B + D created, DriveBase built
#   2  attachment motor on A created
#   3  attachment motor on F created
#   4  IMU reset done immediately, no settle time
#   0  everything passed, hub turns green
#
#   If the display freezes on a number or shows 5-9, that stage failed:
#   6  IMU reset raised because the IMU was not ready yet
#   7  port F empty or motor not detected  -> kills the small version
#   8  port A empty or motor not detected  -> kills the small version
#   9  drive motor B or D not detected

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Axis, Color, Direction, Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)


def halt(code):
    hub.light.on(Color.RED)
    hub.display.number(code)
    while True:
        wait(100)


hub.display.number(1)
try:
    l = Motor(Port.B, Direction.COUNTERCLOCKWISE)
    r = Motor(Port.D, Direction.CLOCKWISE)
    drive = DriveBase(l, r, 62.4, 120)
except Exception:
    halt(9)

hub.display.number(2)
try:
    a = Motor(Port.A, Direction.CLOCKWISE)
except Exception:
    halt(8)

hub.display.number(3)
try:
    f = Motor(Port.F, Direction.CLOCKWISE)
except Exception:
    halt(7)

hub.display.number(4)
try:
    drive.use_gyro(True)
    hub.imu.reset_heading(0)
except Exception:
    halt(6)

hub.display.number(0)
hub.light.on(Color.GREEN)
while True:
    wait(100)
