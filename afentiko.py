from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Axis,Button,Color,Direction,Port,Stop
from pybricks.tools import StopWatch,wait as pwait
 
try:
    from uio import StringIO
    from usys import print_exception
    trace=True
except ImportError:
    try:
        from io import StringIO
        from sys import print_exception
        trace=True
    except ImportError:
        trace=False
 
wheel_diameter=62.4
axle_track=120.0
gyro_scale=1.0
 
max_speed=545.0
max_accel=2000.0
max_rate=520.0
max_turn_accel=2000.0
max_tool_accel=5000.0
top=1000.0
 
min_frac=.03
default_speed=1
default_accel=-.4
default_rate=-.39
default_turn_accel=-.5
tool_speed=0
tool_accel=-.2
 
drive_pid=(5.0,6.0,.15)
turn_pid=(8.0,3.0,.3)
i_zone=5.0
i_limit=10.0
tol= .4
min_dps=14.0
dist_tol=1.2
hold_checks=3
stall_ms=600
loop_ms=5
low_voltage_mv=7200
first_run=1
 
hub=PrimeHub(top_side=Axis.Z,front_side=Axis.X)
left=Motor(Port.B,Direction.COUNTERCLOCKWISE)
right=Motor(Port.D,Direction.CLOCKWISE)
tools={}
for name,port in (("A",Port.A),("F",Port.F)):
    try:tools[name]=Motor(port,Direction.CLOCKWISE)
    except:tools[name]=None
 
mm_per_deg=3.14159265*wheel_diameter/360
k=axle_track/wheel_diameter
owed_mm=owed_deg=vel=integ=prev=0
together=False
clock=StopWatch()
 
class Aborted(Exception):pass
class Probe(Exception):pass
 
script=[]
seen=0
 
def clamp(x,lo,hi):
    return lo if x<lo else hi if x>hi else x
 
def frac(x,limit,default):
    x=float(default if x is None else x)
    if abs(x)>1:return min(abs(x),limit)
    return limit*max(min_frac,(x+1)/2)
 
def tick(ms=loop_ms):
    if Button.CENTER in hub.buttons.pressed():raise Aborted()
    pwait(ms)
 
def halt(hold=True):
    global vel
    vel=0
    if hold:left.hold();right.hold()
    else:
        for m in (left,right,tools["A"],tools["F"]):
            if m:m.stop()
 
def steer(v,err,gains,lim=None,floor=0,ff=0):
    global vel,integ,prev
    dt=clamp(clock.time(),1,20)/1000;clock.reset()
    kp,ki,kd=gains
    integ=clamp(integ+err*dt,-i_limit,i_limit) if abs(err)<i_zone else 0
    w=ff+kp*err+ki*integ+kd*(err-prev)/dt
    prev=err
    if lim is not None:w=clamp(w,-lim,lim)
    if abs(w)<floor:w=floor if err>0 else -floor
    l=v/mm_per_deg+w*k
    r=v/mm_per_deg-w*k
    s=top/max(top,abs(l),abs(r))
    left.run(l*s);right.run(r*s)
    vel=v*s
    return dt
 
def pause(ms):
    t=StopWatch()
    while t.time()<ms:yield
 
def move(mm,speed=None,accel=None,curve=0,stop=True):
    global owed_mm,owed_deg
    owed_mm+=mm
    d=1 if mm>=0 else -1
    length=abs(mm) or 1
    top_v=min(frac(speed,max_speed,default_speed),top*mm_per_deg)
    a=frac(accel,max_accel,default_accel)
    s=vel*d
    dt=loop_ms/1000;mark=1e9;t=StopWatch()
    while True:
        pos=(left.angle()+right.angle())/2*mm_per_deg
        rem=(owed_mm-pos)*d
        if rem<=dist_tol:break
        if abs(pos-mark)>2:mark=pos;t.reset()
        elif t.time()>stall_ms:owed_mm=pos;stop=True;break
        want=min(top_v,(2*a*rem)**.5) if stop else top_v
        s+=clamp(max(want,min_dps*mm_per_deg)-s,-2*a*dt,a*dt)
        goal=owed_deg+curve*clamp(1-rem/length,0,1)
        dt=steer(s*d,goal-hub.imu.heading()*gyro_scale,drive_pid,ff=curve*s/length)
        yield
    owed_deg+=curve
    if stop:halt()
 
def turn(deg,rate=None,accel=None):
    global owed_deg,owed_mm
    owed_deg+=deg
    top_w=frac(rate,max_rate,default_rate)
    aw=frac(accel,max_turn_accel,default_turn_accel)
    brake=2*frac(None,max_accel,default_accel)
    slow=min_dps/k
    fwd=vel
    lim=ok=0;dt=loop_ms/1000;t=StopWatch()
    while (ok<hold_checks or fwd) and t.time()<abs(deg)/top_w*2000+1500:
        err=owed_deg-hub.imu.heading()*gyro_scale
        ok=ok+1 if abs(err)<=tol else 0
        lim=min(lim+aw*dt,top_w,(2*aw*abs(err))**.5+slow)
        fwd-=clamp(fwd,-brake*dt,brake*dt)
        dt=steer(fwd,err,turn_pid,lim,slow if abs(err)>tol else 0)
        yield
    halt()
    owed_mm=(left.angle()+right.angle())/2*mm_per_deg
 
def tool(name,angle,speed=None,accel=None,hold=True,block=True):
    m=tools.get(str(name).upper())
    if not m:return
    if block and vel and not together:halt()
    spd=clamp(frac(speed,top,tool_speed),10,top)
    try:m.control.limits(acceleration=frac(accel,max_tool_accel,tool_accel))
    except:pass
    m.run_angle(spd,angle,Stop.HOLD if hold else Stop.COAST,wait=False)
    t=StopWatch()
    while block and not m.done():
        if t.time()>abs(angle)/spd*1500+800:m.stop();break
        yield
 
def reset():
    global owed_mm,owed_deg,integ,prev
    halt(False)
    hub.imu.reset_heading(0)
    left.reset_angle(0);right.reset_angle(0)
    owed_mm=owed_deg=integ=prev=0
 
def calibrate_now():
    global tol,min_dps,top
    halt();pwait(300)
    lo=hi=hub.imu.heading();t=StopWatch()
    while t.time()<1500:
        h=hub.imu.heading();lo=min(lo,h);hi=max(hi,h)
        tick(10)
    tol=clamp((hi-lo)*1.5,.3,1.5)
    c=0
    while c<200:
        c+=5
        a0=left.angle();b0=right.angle()
        left.run(c);right.run(-c)
        tick(150)
        if min(abs(left.angle()-a0),abs(right.angle()-b0))>=max(2,c*.075):break
    min_dps=clamp(c*1.2,10,60)
    left.run(1000);right.run(-1000)
    best=0;t.reset()
    while t.time()<600:
        if t.time()>300:best=max(best,min(abs(left.speed()),abs(right.speed())))
        tick(10)
    if best>=300:top=best
    print("tol",tol,"min_dps",min_dps,"top",top)
    reset()
 
def command(fn):
    def call(*a,**kw):
        global seen
        if seen<len(script):
            seen+=1
            return
        script.append([fn,a,kw,None])
        raise Probe()
    return call
 
def line_of(e):
    if not trace:return None
    try:
        s=StringIO();print_exception(e,s)
        frames=[x for x in s.getvalue().split("\n") if "File " in x and "line " in x]
        for i in range(1,len(frames)):
            if frames[i].strip().endswith("in call"):
                return int(frames[i-1].split("line ")[1].split(",")[0])
    except Exception:pass
    return None
 
def build(fn):
    global script,seen
    script=[]
    while True:
        seen=0
        try:
            fn()
            break
        except Probe as e:
            script[-1][3]=line_of(e)
    groups=[];last=None
    for f,a,kw,ln in script:
        if groups and ln is not None and ln==last:groups[-1].append((f,a,kw))
        else:groups.append([(f,a,kw)])
        last=ln
    script=[]
    return groups
 
def step(c):
    try:
        next(c)
        return True
    except StopIteration:
        return False
 
def play(groups):
    global together
    for line in groups:
        together=len(line)>1
        try:
            live=[]
            for f,a,kw in line:
                c=f(*a,**kw)
                if c is not None:live.append(c)
            while live:
                live=[c for c in live if step(c)]
                if live:tick()
        finally:
            together=False
 
move=command(move)
turn=command(turn)
tool=command(tool)
wait=command(pause)
calibrate=command(calibrate_now)
 
def menu():
    sel=first_run;lo=min(programs);hi=max(programs)
    while True:
        hub.display.number(sel)
        while hub.buttons.pressed():pwait(20)
        p=()
        while not p:p=hub.buttons.pressed();pwait(20)
        while hub.buttons.pressed():pwait(20)
        if Button.LEFT in p:sel=hi if sel==lo else sel-1
        elif Button.RIGHT in p:sel=lo if sel==hi else sel+1
        elif Button.CENTER in p:
            hub.light.on(Color.ORANGE)
            reset()
            try:
                play(programs[sel])
                halt(False)
                hub.speaker.beep(1000,300)
                sel=first_run if sel>=hi else sel+1
            except Aborted:halt(False)
            hub.light.on(ready)
 
def run_0():
    calibrate()
 
def run_1():
    move(420,.1)
    move(-200,1)
    turn(90)
    move(200,.47)
    turn(-92)
    move(440,.83)
    tool("F",-400)
    turn(-40)
    move(200,-.63)
    tool("F",270)
    move(-190,.83)
    turn(45)
    tool("A",-220,-.8)
    move(100,-.63)
    tool("A",90,-.8)
    move(170,1)
    tool("A",-15,-.8)
    move(-200,1)
    tool("A",-50)
    move(90,-.63)
    tool("A",90,-.8)
    move(120,1)
    tool("A",-20,-.8)
    wait(500)
    move(10,1)
    move(-935,1,1)
    tool("A",90,1,1)
 
def run_2():
    move(710,.83)
    tool("F",170,-.6)
    tool("F",-100,-.6)
    tool("F",100,0)
    tool("F",-90,-.6)
    move(-700,1)
    tool("F",250)
 
def run_3():
    move(100,1)
    turn(50)
    tool("A",310)
    move(315,-.27)
    tool("A",-90)
    move(-10,1)
    tool("A",-60)
    wait(20)
    move(-400,1)
 
def run_4():
    move(600,-.27)
    turn(90)
    move(275,1)
    tool("A",290)
    turn(-94)
    move(150,.1)
    wait(30)
    tool("A",-60)
    move(120,.1)
    move(-80,1)
    move(100,1)
    move(-20,1)
    tool("A",-80)
    move(-80,-.63)
    turn(10)
    tool("A",-70)
    wait(30)
    move(-25,1)
    wait(30)
    move(-120,.1)
    tool("A",-90)
    move(-23,1)
    move(120,1)
 
def run_5():
    move(300,1,-.4), tool("A", -90), tool("F", -90)
    move(-300,1,-.4), tool("A", 90), tool("F", 90)
 
programs={0:build(run_0),1:build(run_1),2:build(run_2),3:build(run_3),4:build(run_4),5:build(run_5)}
 
first_run = 1 
 
try:hub.system.set_stop_button(None)
except:pass
hub.speaker.volume(90)
hub.light.on(Color.RED)
for m in (left,right):
    try:m.control.limits(acceleration=6000)
    except:pass
pwait(500)
clock.reset()
while not hub.imu.ready() and clock.time()<8000:pwait(20)
ready=Color.GREEN if hub.battery.voltage()>=low_voltage_mv else Color.RED
hub.light.on(ready)
try:menu()
finally:
    halt(False)
    hub.light.off()