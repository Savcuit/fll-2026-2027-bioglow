from pybricks.hubs import PrimeHub; from pybricks.pupdevices import Motor; from pybricks.parameters import Axis, Color, Direction, Port, Stop, Button; from pybricks.robotics import DriveBase; from pybricks.tools import StopWatch, wait; left_motor_port = Port.B; right_motor_port = Port.D; tool_a_port = Port.A; tool_f_port = Port.F; left_motor_direction = Direction.COUNTERCLOCKWISE
right_motor_direction = Direction.CLOCKWISE; tool_a_direction = Direction.CLOCKWISE; tool_f_direction = Direction.CLOCKWISE; wheel_diameter = 62.4; axle_track = 120.0; heading_correction = 1.0; max_straight_speed = 900.0; max_turn_rate = 790.0; max_motor_speed = 1000.0; max_straight_accel = 2000.0
max_turn_accel = 2000.0; max_motor_accel = 5000.0; default_straight_speed = 0.92; default_straight_accel = 0.3; default_turn_rate = 0.076; default_turn_accel = 0.15; default_pivot_speed = 0.5; default_pivot_accel = 0.24; tool_default_speed = 0.5; tool_default_accel = 0.4
fine_tune_angle = False; fine_angle_tolerance = 0.4; fine_angle_kp = 7.0; fine_angle_ki = 1.2; fine_angle_kd = 0.9; fine_angle_i_limit = 30.0; fine_angle_min_speed = 14.0; fine_angle_max_speed = 0.18; fine_hold_checks = 4; fine_timeout_ms = 1600
fine_tune_distance = True; fine_dist_tolerance = 1.2; fine_dist_min_move = 1.0; fine_dist_speed = 0.045; fine_dist_passes = 2; pivot_kp = 3.0; pivot_ki = 0.4; pivot_kd = 0.6; pivot_i_limit = 40.0; pivot_min_speed = 22.0
pivot_tolerance = 0.4; pivot_hold_checks = 4; pivot_timeout_ms = 12000; pivot_trim_timeout = 3000; straight_segment_mm = 1000000.0; command_deadband = 1.0; move_timeout_ms = 30000; gyro_enabled = True; imu_settle_ms = 3500; control_loop_ms = 2
motor_stall_speed = 8; motor_stall_time = 200; low_voltage_mv = 7200; hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)
try:
    hub.speaker.volume(80)
except: pass
try:
    hub.system.set_stop_button(Button.BLUETOOTH)
except: pass
left_motor = Motor(left_motor_port, left_motor_direction); right_motor = Motor(right_motor_port, right_motor_direction)
try:
    tool_a = Motor(tool_a_port, tool_a_direction)
except:
    tool_a = None
try:
    tool_f = Motor(tool_f_port, tool_f_direction)
except:
    tool_f = None

def clamp(v, l, h):
    return l if v < l else h if v > h else v

def frac(v, t, d):
    if v is None or float(v) <= 0:
        v = d
    else:
        v = abs(float(v))
    return t * v if v <= 1 else min(v, t)
wheel_deg_per_pivot_deg = 2 * axle_track / wheel_diameter; drive = DriveBase(left_motor, right_motor, wheel_diameter, axle_track); owed_mm = 0; owed_deg = 0

class Aborted(Exception):
    pass

def abort_check():
    if hub.buttons.pressed():
        raise Aborted()

def end_beep():
    try:
        hub.speaker.beep(1000, 300)
    except: pass

def heading():
    return hub.imu.heading()

def battery_ok():
    return hub.battery.voltage() >= low_voltage_mv

def set_stall(m):
    try:
        m.control.stall_tolerances(speed=motor_stall_speed, time=motor_stall_time)
    except: pass

def prepare():
    for m in (left_motor, right_motor, tool_a, tool_f):
        if m:
            set_stall(m)

def relax():
    drive.stop(); left_motor.stop(); right_motor.stop()
    if tool_a:
        tool_a.stop()
    if tool_f:
        tool_f.stop()

def shutdown():
    relax(); hub.light.off()

def apply_heading():
    try:
        hub.imu.settings(heading_correction=heading_correction)
    except: pass

def startup():
    global owed_mm, owed_deg; hub.light.on(Color.RED); relax(); apply_heading(); wait(imu_settle_ms)
    try:
        s = StopWatch()
        while not hub.imu.ready() and s.time() < 8000:
            wait(control_loop_ms)
    except: pass
    hub.imu.reset_heading(0); drive.reset(); drive.use_gyro(gyro_enabled); owed_mm = 0; owed_deg = 0; restore(); prepare(); hub.light.on(Color.GREEN if battery_ok() else Color.RED)

def reset():
    global owed_mm, owed_deg; drive.stop(); left_motor.stop(); right_motor.stop(); hub.imu.reset_heading(0); drive.reset(); drive.use_gyro(gyro_enabled); restore(); owed_mm = 0; owed_deg = 0

def take_drive():
    left_motor.stop(); right_motor.stop(); drive.use_gyro(gyro_enabled)

def apply(speed=None, accel=None, rate=None, turn_accel=None):
    drive.stop(); drive.settings(straight_speed=frac(speed, max_straight_speed, default_straight_speed), straight_acceleration=frac(accel, max_straight_accel, default_straight_accel), turn_rate=frac(rate, max_turn_rate, default_turn_rate), turn_acceleration=frac(turn_accel, max_turn_accel, default_turn_accel))

def restore(): apply()

def wait_drive():
    t = StopWatch()
    while not drive.done():
        abort_check()
        if t.time() > move_timeout_ms:
            drive.stop(); return False
        wait(control_loop_ms)
    return True

def fine_angle(target):
    if not fine_tune_angle:
        return
    if abs(target - heading()) <= fine_angle_tolerance:
        return
    while abs(target - heading()) > fine_angle_tolerance:
        abort_check(); c = clamp((target - heading()) * fine_angle_kp, fine_angle_min_speed, max_motor_speed); left_motor.run(c); right_motor.run(-c); wait(control_loop_ms)
    left_motor.hold(); right_motor.hold()

def fine_distance(target, speed=None, accel=None):
    if not fine_tune_distance:
        return
    for _ in range(fine_dist_passes):
        abort_check(); error = target - drive.distance()
        if abs(error) <= fine_dist_tolerance or abs(error) < fine_dist_min_move:
            break
        apply(speed, accel); drive.straight(error, then=Stop.HOLD, wait=False)
        if not wait_drive():
            break
    restore()

def straight(distance_mm, speed=None, accel=None, segment=None, kp=None, ki=None, kd=None, trim_speed=None):
    global owed_mm; fine_angle(owed_deg); owed_mm += float(distance_mm); seg = straight_segment_mm if segment is None else abs(float(segment)); take_drive(); apply(speed, accel)
    while True:
        abort_check(); remain = owed_mm - drive.distance()
        if abs(remain) <= fine_dist_tolerance:
            break
        step = max(-seg, min(seg, remain)); drive.straight(step, then=Stop.HOLD, wait=False)
        if not wait_drive():
            break
        if abs(owed_deg - heading()) > fine_angle_tolerance:
            fine_angle(owed_deg); apply(speed, accel)
    restore(); fine_distance(owed_mm, trim_speed, accel); fine_angle(owed_deg)

def fd(d, *a, **k): straight(abs(float(d)), *a, **k)

def bk(d, *a, **k): straight(-abs(float(d)), *a, **k)

def spin(angle, rate=None, turn_accel=None, kp=None, ki=None, kd=None, trim_speed=None):
    global owed_deg; owed_deg += float(angle); take_drive(); apply(rate=rate, turn_accel=turn_accel); turn = owed_deg - heading()
    if abs(turn) > fine_angle_tolerance:
        drive.turn(turn, then=Stop.HOLD, wait=False); wait_drive()
    restore(); fine_angle(owed_deg)

def right(angle, *a, **k): spin(abs(float(angle)), *a, **k)

def left(angle, *a, **k): spin(-abs(float(angle)), *a, **k)

def tool(port):
    if str(port).upper() == 'A':
        return tool_a
    if str(port).upper() == 'F':
        return tool_f

def tool_speed(s):
    return clamp(frac(s, max_motor_speed, tool_default_speed), 10, max_motor_speed)

def tool_accel(m, a):
    try:
        m.control.limits(acceleration=frac(a, max_motor_accel, tool_default_accel))
    except: pass

def tool_turn(port, angle, speed=None, accel=None, hold=True, block=True):
    m = tool(port)
    if not m:
        return
    tool_accel(m, accel); m.run_angle(tool_speed(speed), float(angle), then=Stop.HOLD if hold else Stop.COAST, wait=False)
    if block:
        wait_motor(m)

def wait_motor(m):
    t = StopWatch()
    while not m.done():
        abort_check()
        if t.time() > move_timeout_ms:
            m.stop(); break
        wait(control_loop_ms)

def a_turn(a, speed=None, accel=None, hold=True, block=True): tool_turn('A', a, speed, accel, hold, block)

def f_turn(a, speed=None, accel=None, hold=True, block=True): tool_turn('F', a, speed, accel, hold, block)

def measure_gyro_noise():
    relax(); hub.imu.reset_heading(0); low = high = hub.imu.heading(); t = StopWatch()
    while t.time() < 3000:
        abort_check(); v = hub.imu.heading(); low = min(low, v); high = max(high, v); wait(10)
    return high - low

def measure_deadband(m):
    c = 5
    while c <= 200:
        abort_check(); m.reset_angle(0); m.run(c); wait(0.2); moved = abs(m.angle()); m.stop()
        if moved > 3:
            return c
        c += 5
    return 200

def measure_top_speed(m):
    best = 0; m.run(max_motor_speed); t = StopWatch()
    while t.time() < 700:
        abort_check()
        if t.time() > 350:
            best = max(best, abs(m.speed()))
        wait(10)
    m.stop(); return best

def calibrate():
    global fine_angle_tolerance, pivot_tolerance; global fine_angle_min_speed, pivot_min_speed, max_motor_speed; noise = measure_gyro_noise(); dl = measure_deadband(left_motor); dr = measure_deadband(right_motor); tl = measure_top_speed(left_motor); tr = measure_top_speed(right_motor); tol = clamp(noise * 1.5, 0.3, 1.5); floor = clamp(max(dl, dr) * 1.2, 10, 60); top = min(tl, tr)
    if top < 300:
        top = max_motor_speed
    fine_angle_tolerance = tol; pivot_tolerance = tol; fine_angle_min_speed = floor; pivot_min_speed = floor; max_motor_speed = top; reset()

def run_0(): calibrate()

def run_1():
    fd(245, speed=0.55); right(90); fd(615); right(90); bk(200); f_turn(-150, 0.1); f_turn(50, 1); fd(240); left(-90); bk(615)
    left(-90); bk(245)

def run_2():
    fd(1000, speed=1, accel=1); bk(1000, speed=1, accel=1)

def run_3(): f_turn(180, speed=0.5)

def run_4():
    right(90); fd(300)

programs = {0: run_0, 1: run_1, 2: run_2, 3: run_3, 4: run_4}; first_run = 1

def wait_release():
    while hub.buttons.pressed():
        wait(20)
    wait(50)

def wait_press():
    p = ()
    while not p:
        p = hub.buttons.pressed(); wait(20)
    return p

def select_and_run():
    selected = first_run; low = min(programs); high = max(programs); wait_release()
    while True:
        hub.display.number(selected); p = wait_press(); wait_release()
        if Button.LEFT in p:
            selected = high if selected == low else selected - 1
        elif Button.RIGHT in p:
            selected = low if selected == high else selected + 1
        elif Button.CENTER in p:
            hub.light.on(Color.BLUE); reset()
            try:
                programs[selected](); relax(); end_beep(); selected = first_run if selected >= high else selected + 1
            except Aborted:
                relax()
            hub.light.on(Color.BLUE); wait_release()
startup()
try:
    select_and_run()
finally:
    shutdown()
