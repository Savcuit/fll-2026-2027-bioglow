# =============================================================================
# bioglow_full_explained.py - the full BIOGLOW robot code, explained in words.
# =============================================================================
#
# This is the same program as bioglow_full.py, with notes written for someone who
# has never programmed. Nothing about how the robot behaves is different: only
# the explanations are new.
#
# WHAT THE ROBOT IS MADE OF
#   Port B - left wheel motor          Port D - right wheel motor
#   Port A - first arm motor           Port F - second arm motor
#   The hub in the middle holds the screen, the buttons and the gyroscope.
#
# WHAT THE GYROSCOPE DOES
#   It senses which way the robot is facing. The program stores the direction it
#   wants in a number called "owed_deg", and after every move it checks the real
#   heading against that number and corrects the difference.
#
# HOW TO USE THE HUB
#   LEFT  - previous program       RIGHT - next program
#   CENTER - run the program shown. Press CENTER again while it runs: the robot
#            stops and you are back at the number.
#   Red light = preparing, green = ready, blue = running.
#
# THE FIVE PROGRAMS
#   0 calibration   1 first run   2 second run   3 alignment   4 test move
#
# A NOTE ON SPEEDS
#   A speed written as a fraction, such as 0.3, means "30% of the normal speed".
#   A speed written as a bigger number, such as 100, means exactly 100 mm/second.
#
# =============================================================================

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Axis,Color,Direction,Port,Stop,Button
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch,wait

left_motor_port=Port.B
right_motor_port=Port.D
tool_a_port=Port.A
tool_f_port=Port.F
left_motor_direction=Direction.COUNTERCLOCKWISE
right_motor_direction=Direction.CLOCKWISE
tool_a_direction=Direction.CLOCKWISE
tool_f_direction=Direction.CLOCKWISE

wheel_diameter=62.4
axle_track=120.0
heading_correction=1.0

max_straight_speed=900.0
max_turn_rate=790.0
max_motor_speed=1000.0
max_straight_accel=2000.0
max_turn_accel=2000.0
max_motor_accel=5000.0

default_straight_speed=.92
default_straight_accel=.3
default_turn_rate=.076
default_turn_accel=.15
default_pivot_speed=.5
default_pivot_accel=.24
tool_default_speed=.5
tool_default_accel=.4

fine_tune_angle=False
fine_angle_tolerance=.4
fine_angle_kp=7.0
fine_angle_ki=1.2
fine_angle_kd=.9
fine_angle_i_limit=30.0
fine_angle_min_speed=14.0
fine_angle_max_speed=.18
fine_hold_checks=4
fine_timeout_ms=1600

fine_tune_distance=True
fine_dist_tolerance=1.2
fine_dist_min_move=1.0
fine_dist_speed=.045
fine_dist_passes=2

pivot_kp=3.0
pivot_ki=.4
pivot_kd=.6
pivot_i_limit=40.0
pivot_min_speed=22.0
pivot_tolerance=.4
pivot_hold_checks=4
pivot_timeout_ms=12000
pivot_trim_timeout=3000

straight_segment_mm=1000000.0
command_deadband=1.0
move_timeout_ms=30000
gyro_enabled=True
imu_settle_ms=3500
control_loop_ms=2

motor_stall_speed=8
motor_stall_time=200
low_voltage_mv=7200

hub=PrimeHub(top_side=Axis.Z,front_side=Axis.X)

hub.speaker.volume(80)
hub.system.set_stop_button(Button.BLUETOOTH)

left_motor=Motor(left_motor_port,left_motor_direction)
right_motor=Motor(right_motor_port,right_motor_direction)

tool_a=Motor(tool_a_port,tool_a_direction)
tool_f=Motor(tool_f_port,tool_f_direction)

# ---- clamp -------------------------------------------------------
# Keeps a number between a lowest and a highest value. Used so a speed never
# becomes negative or impossibly large.

def clamp(v,l,h):
    return l if v<l else h if v>h else v

# ---- frac --------------------------------------------------------
# Turns a speed written as a fraction (0.5 = half speed) into real motor speed.
# No value means 'use the normal speed'; a value above 1 is treated as an exact speed.

def frac(v,t,d):
    if v is None or float(v)<=0:v=d
    else:v=abs(float(v))
    return t*v if v<=1 else min(v,t)

wheel_deg_per_pivot_deg=2*axle_track/wheel_diameter
drive=DriveBase(left_motor,right_motor,wheel_diameter,axle_track)

owed_mm=0
owed_deg=0

class Aborted(Exception):pass

# ---- abort_check -------------------------------------------------
# The panic check. If any button on the hub is being pressed, it raises 'Aborted',
# which stops the run cleanly and returns to the program list.

def abort_check():
    if hub.buttons.pressed():raise Aborted()

# ---- end_beep ----------------------------------------------------
# Makes a short sound when a run finishes, so the team knows it ended properly.

def end_beep():
    hub.speaker.beep(1000,300)

# ---- shortest ----------------------------------------------------
# Converts any heading into the shortest way to turn to it (never more than 180 degrees).

def shortest(a):
    while a>180:a-=360
    while a<-180:a+=360
    return a

# ---- heading -----------------------------------------------------
# Reads the direction the robot is currently facing, in degrees, from the gyroscope.

def heading():
    return hub.imu.heading()

# ---- battery_ok --------------------------------------------------
# True when the battery is still strong enough for reliable runs.

def battery_ok():
    return hub.battery.voltage()>=low_voltage_mv

# ---- set_stall ---------------------------------------------------
# Tells a motor what counts as 'stuck'. Used to avoid burning motors against the field.

def set_stall(m):
    m.control.stall_tolerances(speed=motor_stall_speed,time=motor_stall_time)

# ---- prepare -----------------------------------------------------
# Applies the stall settings to all four motors before a run.

def prepare():
    for m in(left_motor,right_motor,tool_a,tool_f):
        if m:set_stall(m)

# ---- relax -------------------------------------------------------
# Stops the drive and every motor. Nothing is held, nothing is driven.

def relax():
    drive.stop()
    left_motor.stop()
    right_motor.stop()
    if tool_a:tool_a.stop()
    if tool_f:tool_f.stop()

# ---- shutdown ----------------------------------------------------
# Stops everything and turns the light off. Called when the program ends.

def shutdown():
    relax()
    hub.light.off()

# ---- apply_heading -----------------------------------------------
# Calibrates the gyroscope against its own errors, if the firmware supports it.

def apply_heading():
    hub.imu.settings(heading_correction=heading_correction)

# ---- startup -----------------------------------------------------
# Start-up sequence: red light, stop everything, wait for the gyroscope to settle,
# set the heading and distance to zero, then green light when the battery is good.

def startup():
    global owed_mm,owed_deg
    hub.light.on(Color.RED)
    relax()
    apply_heading()
    wait(imu_settle_ms)
    s=StopWatch()
    while not hub.imu.ready() and s.time()<8000:wait(control_loop_ms)
    hub.imu.reset_heading(0)
    drive.reset()
    drive.use_gyro(gyro_enabled)
    owed_mm=0
    owed_deg=0
    restore()
    prepare()
    hub.light.on(Color.GREEN if battery_ok() else Color.RED)

# ---- reset -------------------------------------------------------
# Puts the robot back to a known state between runs: stopped, gyroscope at zero,
# distance counter at zero.

def reset():
    global owed_mm,owed_deg
    drive.stop()
    left_motor.stop()
    right_motor.stop()
    hub.imu.reset_heading(0)
    drive.reset()
    drive.use_gyro(gyro_enabled)
    restore()
    owed_mm=0
    owed_deg=0

# ---- take_drive --------------------------------------------------
# Hands control back to the drive base after a motor was moved on its own.

def take_drive():
    left_motor.stop()
    right_motor.stop()
    drive.use_gyro(gyro_enabled)

# ---- apply -------------------------------------------------------
# Chooses how fast and how sharply the next movement happens.
# A speed of 0.5 means half speed; 100 means exactly 100 mm per second.

def apply(speed=None,accel=None,rate=None,turn_accel=None):
    drive.stop()
    drive.settings(
    straight_speed=frac(speed,max_straight_speed,default_straight_speed),
    straight_acceleration=frac(accel,max_straight_accel,default_straight_accel),
    turn_rate=frac(rate,max_turn_rate,default_turn_rate),
    turn_acceleration=frac(turn_accel,max_turn_accel,default_turn_accel))

# ---- restore -----------------------------------------------------
# Returns the drive to the normal settings used by the mission runs.

def restore():
    apply()

# ---- wait_drive --------------------------------------------------
# Waits for the robot to finish its current move, checking the buttons 500 times a
# second so CENTER can stop it, and giving up after 30 seconds to be safe.

def wait_drive():
    t=StopWatch()
    while not drive.done():
        abort_check()
        if t.time()>move_timeout_ms:
            drive.stop()
            return False
        wait(control_loop_ms)
    return True

# ---- fine_angle --------------------------------------------------
# The accuracy pass for turning: if the robot is not exactly on its target heading,
# it creeps left or right until it is.

def fine_angle(target):
    if not fine_tune_angle:return
    if abs(target-heading())<=fine_angle_tolerance:return
    while abs(target-heading())>fine_angle_tolerance:
        abort_check()
        c=clamp((target-heading())*fine_angle_kp,fine_angle_min_speed,max_motor_speed)
        left_motor.run(c)
        right_motor.run(-c)
        wait(control_loop_ms)
    left_motor.hold()
    right_motor.hold()
# ---- fine_distance -----------------------------------------------
# The accuracy pass for distance: if the robot stopped a millimetre or two short,
# it takes one more tiny step.

def fine_distance(target,speed=None,accel=None):
    if not fine_tune_distance:return
    for _ in range(fine_dist_passes):
        abort_check()
        error=target-drive.distance()
        if abs(error)<=fine_dist_tolerance or abs(error)<fine_dist_min_move:
            break
        apply(speed,accel)
        drive.straight(error,then=Stop.HOLD,wait=False)
        if not wait_drive():break
    restore()

# ---- straight ----------------------------------------------------
# Drives forward or backward with correction. It drives in pieces, re-checking the
# heading between pieces, then trims the last millimetres.

def straight(distance_mm,speed=None,accel=None,segment=None,
             kp=None,ki=None,kd=None,trim_speed=None):
    global owed_mm
    fine_angle(owed_deg)
    owed_mm+=float(distance_mm)
    seg=straight_segment_mm if segment is None else abs(float(segment))
    take_drive()
    apply(speed,accel)
    while True:
        abort_check()
        remain=owed_mm-drive.distance()
        if abs(remain)<=fine_dist_tolerance:break
        step=max(-seg,min(seg,remain))
        drive.straight(step,then=Stop.HOLD,wait=False)
        if not wait_drive():break
        if abs(owed_deg-heading())>fine_angle_tolerance:
            fine_angle(owed_deg)
            apply(speed,accel)
    restore()
    fine_distance(owed_mm,trim_speed,accel)
    fine_angle(owed_deg)

# ---- fd ----------------------------------------------------------
# Drive FORWARD a given number of millimetres. fd(420) is 420 mm; fd(420,.3) is 30% speed.

def fd(d,*a,**k):
    straight(abs(float(d)),*a,**k)

# ---- bk ----------------------------------------------------------
# Drive BACKWARD a given number of millimetres.

def bk(d,*a,**k):
    straight(-abs(float(d)),*a,**k)

# ---- spin --------------------------------------------------------
# Turn on the spot by a number of degrees. Positive turns right, negative turns left.

def spin(angle,rate=None,turn_accel=None,
         kp=None,ki=None,kd=None,trim_speed=None):
    global owed_deg
    owed_deg+=float(angle)
    take_drive()
    apply(rate=rate,turn_accel=turn_accel)
    turn=owed_deg-heading()
    if abs(turn)>fine_angle_tolerance:
        drive.turn(turn,then=Stop.HOLD,wait=False)
        wait_drive()
    restore()
    fine_angle(owed_deg)

# ---- right -------------------------------------------------------
# Turn clockwise (to the robot's right) by the number of degrees given.

def right(angle,*a,**k):
    spin(abs(float(angle)),*a,**k)

# ---- left --------------------------------------------------------
# Turn anticlockwise (to the robot's left) by the number of degrees given.

def left(angle,*a,**k):
    spin(-abs(float(angle)),*a,**k)

# ---- face --------------------------------------------------------
# Turn to face an absolute direction, e.g. face(90) means 'point east'.

def face(target,*a,**k):
    global owed_deg
    owed_deg=heading()+shortest(float(target)-heading())
    spin(0,*a,**k)

# ---- move_and_arms -----------------------------------------------
# Drive and move the arms AT THE SAME TIME, which saves precious seconds.
# Example: move_and_arms(300, a_deg=-90, f_deg=200) drives 300 mm while both arms turn.
# Distances are millimetres, arm angles are degrees, speeds may be fractions of full speed.

def move_and_arms(distance_mm,a_deg=0,f_deg=0,speed=None,accel=None,aspeed=500,fspeed=500):
    global owed_mm
    fine_angle(owed_deg)
    take_drive()
    apply(speed,accel)
    if a and a_deg:a.run_angle(aspeed,a_deg,then=Stop.HOLD,wait=False)
    if f and f_deg:f.run_angle(fspeed,f_deg,then=Stop.HOLD,wait=False)
    drive.straight(float(distance_mm),then=Stop.HOLD,wait=False)
    while not drive.done() or (a and not a.done()) or (f and not f.done()):
        abort_check()
        wait(control_loop_ms)
    owed_mm=drive.distance()
    restore()
    fine_angle(owed_deg)

# ---- pivot -------------------------------------------------------
# Turn around one locked wheel instead of both. The moving wheel travels further than
# the turn angle, so it is scaled by wheel_deg_per_pivot_deg.

def pivot(angle,moving,held,sign,speed,accel=None):
    global owed_deg
    owed_deg+=angle
    take_drive()    
    held.hold()
    moving.control.limits(
        acceleration=frac(accel,max_motor_accel,default_pivot_accel))
    moving.run_angle(
        frac(speed,max_motor_speed,default_pivot_speed),
        angle*wheel_deg_per_pivot_deg*sign,
        then=Stop.HOLD,
        wait=False)
    while not moving.done():
        abort_check()
        if abs(owed_deg-heading())<=pivot_tolerance:break
        wait(control_loop_ms)
    moving.hold()
    held.hold()
    restore()

# ---- pivot_right -------------------------------------------------
# Pivot clockwise on the right wheel.

def pivot_right(angle,speed=None,accel=None,*x):
    pivot(abs(float(angle)),left_motor,right_motor,1,speed,accel)

# ---- pivot_left --------------------------------------------------
# Pivot anticlockwise on the left wheel.

def pivot_left(angle,speed=None,accel=None,*x):
    pivot(-abs(float(angle)),right_motor,left_motor,-1,speed,accel)

# ---- pivot_right_back --------------------------------------------
# Pivot clockwise using the right wheel, moving backwards.

def pivot_right_back(angle,speed=None,accel=None,*x):
    pivot(abs(float(angle)),right_motor,left_motor,-1,speed,accel)

# ---- pivot_left_back ---------------------------------------------
# Pivot anticlockwise using the left wheel, moving backwards.

def pivot_left_back(angle,speed=None,accel=None,*x):
    pivot(-abs(float(angle)),left_motor,right_motor,1,speed,accel)

# ---- arc ---------------------------------------------------------
# Drive along a curve of a given radius, for a given angle.

def arc(radius,angle,speed=None,accel=None,*x):
    global owed_deg,owed_mm
    owed_deg+=float(angle)
    take_drive()
    apply(speed,accel)
    drive.curve(float(radius),float(angle),then=Stop.HOLD,wait=False)
    wait_drive()
    restore()
    fine_angle(owed_deg)
    owed_mm=drive.distance()

# ---- arc_right ---------------------------------------------------
# Curve to the right.

def arc_right(r,a,*x):
    arc(abs(float(r)),abs(float(a)),*x)

# ---- arc_left ----------------------------------------------------
# Curve to the left.

def arc_left(r,a,*x):
    arc(abs(float(r)),-abs(float(a)),*x)

# ---- arc_back_right ----------------------------------------------
# Reverse along a curve to the right.

def arc_back_right(r,a,*x):
    arc(-abs(float(r)),abs(float(a)),*x)

# ---- arc_back_left -----------------------------------------------
# Reverse along a curve to the left.

def arc_back_left(r,a,*x):
    arc(-abs(float(r)),-abs(float(a)),*x)

# ---- tool --------------------------------------------------------
# Returns the arm motor for port 'A' or 'F'.

def tool(port):
    if str(port).upper()=="A":return tool_a
    if str(port).upper()=="F":return tool_f

# ---- tool_speed --------------------------------------------------
# Converts an arm speed written as a fraction into a real motor speed.

def tool_speed(s):
    return clamp(frac(s,max_motor_speed,tool_default_speed),10,max_motor_speed)

# ---- tool_accel --------------------------------------------------
# Sets how sharply an arm motor may speed up.

def tool_accel(m,a):
    m.control.limits(acceleration=frac(a,max_motor_accel,tool_default_accel))

# ---- tool_turn ---------------------------------------------------
# The general routine for turning an arm by a number of degrees.

def tool_turn(port,angle,speed=None,accel=None,hold=True,block=True):
    m=tool(port)
    if not m:return
    tool_accel(m,accel)
    m.run_angle(tool_speed(speed),float(angle),
                then=Stop.HOLD if hold else Stop.COAST,
                wait=False)
    if block:wait_motor(m)

# ---- wait_motor --------------------------------------------------
# Waits for an arm to finish moving while still watching the buttons.

def wait_motor(m):
    t=StopWatch()
    while not m.done():
        abort_check()
        if t.time()>move_timeout_ms:
            m.stop()
            break
        wait(control_loop_ms)

# ---- tool_time ---------------------------------------------------
# Runs an arm for a number of seconds instead of a number of degrees.

def tool_time(port,time_s,speed=None,accel=None,hold=True,block=True):
    m=tool(port)
    if not m:return
    tool_accel(m,accel)
    m.run_time(tool_speed(speed),
               int(float(time_s)*1000),
               then=Stop.HOLD if hold else Stop.COAST,
               wait=False)
    if block:wait_motor(m)

# ---- tool_to -----------------------------------------------------
# Moves an arm to an absolute angle rather than by a relative amount.

def tool_to(port,target,speed=None,accel=None,hold=True,block=True):
    m=tool(port)
    if not m:return
    tool_accel(m,accel)
    m.run_target(tool_speed(speed),target,
                 then=Stop.HOLD if hold else Stop.COAST,
                 wait=False)
    if block:wait_motor(m)

# ---- tool_run ----------------------------------------------------
# Runs an arm continuously until told to stop.

def tool_run(port,speed=None,accel=None):
    m=tool(port)
    if not m:return
    tool_accel(m,accel)
    m.run(speed or 0)

# ---- tool_stop ---------------------------------------------------
# Stops an arm, optionally holding its position.

def tool_stop(port=None,hold=False):
    motors=[tool_a,tool_f] if port is None else [tool(port)]
    for m in motors:
        if m:
            m.hold() if hold else m.stop()

# ---- tool_reset --------------------------------------------------
# Sets an arm's angle counter back to zero.

def tool_reset(port,angle=0):
    m=tool(port)
    if m:m.reset_angle(angle)

# ---- tool_angle --------------------------------------------------
# Reads how far an arm has turned.

def tool_angle(port):
    m=tool(port)
    return m.angle() if m else 0
# ---- a_turn ------------------------------------------------------
# Turn the arm on port A by a number of degrees. a_turn(90) is a quarter turn.

def a_turn(a,speed=None,accel=None,hold=True,block=True):
    tool_turn("A",a,speed,accel,hold,block)

# ---- a_time ------------------------------------------------------
# Run the port A arm for a number of seconds.

def a_time(t,speed=None,accel=None,hold=True,block=True):
    tool_time("A",t,speed,accel,hold,block)

# ---- a_to --------------------------------------------------------
# Move the port A arm to an absolute angle.

def a_to(t,speed=None,accel=None,hold=True,block=True):
    tool_to("A",t,speed,accel,hold,block)

# ---- a_run -------------------------------------------------------
# Run the port A arm continuously.

def a_run(speed=None,accel=None):
    tool_run("A",speed,accel)

# ---- a_stop ------------------------------------------------------
# Stop the port A arm.

def a_stop(hold=False):
    tool_stop("A",hold)

# ---- a_reset -----------------------------------------------------
# Zero the port A arm's angle counter.

def a_reset(angle=0):
    tool_reset("A",angle)

# ---- a_angle -----------------------------------------------------
# Read the port A arm's angle.

def a_angle():
    return tool_angle("A")

# ---- f_turn ------------------------------------------------------
# Turn the arm on port F by a number of degrees.

def f_turn(a,speed=None,accel=None,hold=True,block=True):
    tool_turn("F",a,speed,accel,hold,block)

# ---- f_time ------------------------------------------------------
# Run the port F arm for a number of seconds.

def f_time(t,speed=None,accel=None,hold=True,block=True):
    tool_time("F",t,speed,accel,hold,block)

# ---- f_to --------------------------------------------------------
# Move the port F arm to an absolute angle.

def f_to(t,speed=None,accel=None,hold=True,block=True):
    tool_to("F",t,speed,accel,hold,block)

# ---- f_run -------------------------------------------------------
# Run the port F arm continuously.

def f_run(speed=None,accel=None):
    tool_run("F",speed,accel)

# ---- f_stop ------------------------------------------------------
# Stop the port F arm.

def f_stop(hold=False):
    tool_stop("F",hold)

# ---- f_reset -----------------------------------------------------
# Zero the port F arm's angle counter.

def f_reset(angle=0):
    tool_reset("F",angle)

# ---- f_angle -----------------------------------------------------
# Read the port F arm's angle.

def f_angle():
    return tool_angle("F")

# ---- wait_s ------------------------------------------------------
# Pause for a number of seconds, while still watching for a button press.

def wait_s(seconds):
    t=StopWatch()
    while t.time()<seconds*1000:
        abort_check()
        wait(control_loop_ms)

# ---- measure_gyro_noise ------------------------------------------
# Calibration step: measures how much the gyroscope drifts while the robot stands still.
# That drift decides how strict the turning accuracy can be.

def measure_gyro_noise():
    relax()
    hub.imu.reset_heading(0)
    low=high=hub.imu.heading()
    t=StopWatch()
    while t.time()<3000:
        abort_check()
        v=hub.imu.heading()
        low=min(low,v)
        high=max(high,v)
        wait(10)
    return high-low

# ---- measure_deadband --------------------------------------------
# Calibration step: finds the smallest motor power that actually moves the robot.
# Below that value the motor buzzes without turning.

def measure_deadband(m):
    c=5
    while c<=200:
        abort_check()
        m.reset_angle(0)
        m.run(c)
        wait(.2)
        moved=abs(m.angle())
        m.stop()
        if moved>3:
            return c
        c+=5
    return 200

# ---- measure_top_speed -------------------------------------------
# Calibration step: measures the real top speed of a motor.

def measure_top_speed(m):
    best=0
    m.run(max_motor_speed)
    t=StopWatch()
    while t.time()<700:
        abort_check()
        if t.time()>350:
            best=max(best,abs(m.speed()))
        wait(10)
    m.stop()
    return best

# ---- calibrate ---------------------------------------------------
# Calibration run. Run this once after switching on: it measures gyro drift and motor
# behaviour, then sets the tolerances and top speed for this session.

def calibrate():
    
    global fine_angle_tolerance, pivot_tolerance
    global fine_angle_min_speed, pivot_min_speed, max_motor_speed
    
    noise = measure_gyro_noise()
    dl = measure_deadband(left_motor)
    dr = measure_deadband(right_motor)
    tl = measure_top_speed(left_motor)
    tr = measure_top_speed(right_motor)
    tol = clamp(noise*1.5, .3, 1.5)
    floor = clamp(max(dl, dr)*1.2, 10, 60)
    top = min(tl, tr)
    
    if top < 300: 
        top = max_motor_speed

    fine_angle_tolerance = tol
    pivot_tolerance = tol
    fine_angle_min_speed = floor
    pivot_min_speed = floor
    max_motor_speed = top
    reset()

# ---- run_0 -------------------------------------------------------
# Program 0: calibration only.

def run_0():
    calibrate()

# ---- run_1 -------------------------------------------------------
# Program 1: the first mission run. Read it top to bottom: each line is one move.

def run_1():
    fd(245,speed=.55)
    right(90)
    fd(615)
    right(90)
    bk(200)
    f_turn(-150,.1)
    f_turn(50,1)
    fd(240)
    left(-90)
    bk(615)
    left(-90)
    bk(245)

        #fd(605,speed=.15,accel=.15) 
        #a_turn(200,speed=1)
        #left(-35,1)
        #a_turn(2000,speed=1)
        #left(-20,1) 
        #right(90,speed=1)
        #bk(800,speed=.55,accel=.15)

# ---- run_2 -------------------------------------------------------
# Program 2: the second mission run.

def run_2():
    fd(1000,speed=1,accel=1)
    bk(1000,speed=1,accel=1)

# ---- run_3 -------------------------------------------------------
# Program 3: a short alignment move.

def run_3():
    f_turn(180,speed=.5)

# ---- run_4 -------------------------------------------------------
# Program 4: a small test move.

def run_4():
    right(90)
    fd(300)

# ---- run_5 -------------------------------------------------------
# Program 5: spare slot.

def run_5():
    left(90)
    bk(300)

programs={
0:run_0,
1:run_1,
2:run_2,
3:run_3,
4:run_4,
}

first_run=1

# ---- wait_release ------------------------------------------------
# Waits until every button is let go, then a fraction longer. This stops one press
# being counted twice.

def wait_release():
    while hub.buttons.pressed():
        wait(20)
    wait(50)

# ---- wait_press --------------------------------------------------
# Waits until a button is pressed and reports which one. Between presses the program
# simply rests, using no battery.

def wait_press():
    p=()
    while not p:
        p=hub.buttons.pressed()
        wait(20)
    return p

# ---- select_and_run ----------------------------------------------
# The menu. It shows the program number, waits for a press, and then:
# LEFT = previous program, RIGHT = next program, CENTER = run it.
# Pressing a button during a run stops the robot and returns here.

def select_and_run():
    selected=first_run
    low=min(programs)
    high=max(programs)
    wait_release()
    while True:
        hub.display.number(selected)
        p=wait_press()
        wait_release()

        if Button.LEFT in p:
            selected=high if selected==low else selected-1

        elif Button.RIGHT in p:
            selected=low if selected==high else selected+1

        elif Button.CENTER in p:
            hub.light.on(Color.BLUE)
            reset()
            try:
                programs[selected]()
                relax()
                end_beep()
                selected=first_run if selected>=high else selected+1
            except Aborted:
                relax()

            hub.light.on(Color.BLUE)
            wait_release()

startup()

try:
    select_and_run()
finally:
    shutdown()
