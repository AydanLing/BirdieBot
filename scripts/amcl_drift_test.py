"""Seed AMCL at ground truth, drive a fixed path, report how far it diverged.

Exists to answer "does this change help localisation?" with a number instead of
an argument. Seeds AMCL at Gazebo truth, drives a three-leg square, and reports
the position and heading error at the end. The turns matter: that is where scan
matching earns its keep, and a straight line would mostly measure odometry.

AMCL is a particle filter, so ONE RUN IS NOT A RESULT. Take at least three per
configuration and compare the spread, not single values. Measured on
/scan_filtered: 67, 70, 57, 46 mm. A single raw-/scan run gave 75 mm, which is
not a baseline and should not be quoted as one.

Needs a quiet machine. nav2's lifecycle manager stalls at "Configuring
map_server" under load, leaving amcl unconfigured and the map->base_link
lookup returning None -- which shows up here as a TypeError on a None pose,
not as a helpful message. If that happens, check `ros2 lifecycle get /amcl`
before suspecting the test.

Run:  python3 amcl_drift_test.py
"""
import math, subprocess, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
import tf2_ros

def truth():
    o=subprocess.run(["gz","model","-m","rosbot","-p"],capture_output=True,text=True,timeout=20).stdout
    import re
    tri=[]
    for g in re.findall(r"\[([-\d.e+ ]+)\]",o):
        p=g.split()
        if len(p)==3:
            try: tri.append([float(v) for v in p])
            except ValueError: pass
    return (tri[0][0],tri[0][1],tri[1][2]) if len(tri)>=2 else None

class D(Node):
    def __init__(s):
        super().__init__("drift")
        s.buf=tf2_ros.Buffer(); tf2_ros.TransformListener(s.buf,s)
        s.cmd=s.create_publisher(TwistStamped,"/cmd_vel",10)
        s.ip=s.create_publisher(PoseWithCovarianceStamped,"/initialpose",10)
    def spin(s,t):
        e=time.time()+t
        while rclpy.ok() and time.time()<e: rclpy.spin_once(s,timeout_sec=0.05)
    def amcl(s):
        try: t=s.buf.lookup_transform("map","base_link",rclpy.time.Time())
        except Exception: return None
        q=t.transform.rotation
        return (t.transform.translation.x,t.transform.translation.y,
                math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    def seed(s,x,y,yaw):
        m=PoseWithCovarianceStamped(); m.header.frame_id="map"
        m.pose.pose.position.x=x; m.pose.pose.position.y=y
        m.pose.pose.orientation.z=math.sin(yaw/2); m.pose.pose.orientation.w=math.cos(yaw/2)
        c=[0.0]*36; c[0]=c[7]=0.15; c[35]=0.05; m.pose.covariance=c
        for _ in range(10):
            m.header.stamp=s.get_clock().now().to_msg(); s.ip.publish(m); s.spin(0.2)
        s.spin(3.0)
    def drive(s,vx,wz,dur):
        e=time.time()+dur
        while rclpy.ok() and time.time()<e:
            t=TwistStamped(); t.header.stamp=s.get_clock().now().to_msg()
            t.header.frame_id="base_link"; t.twist.linear.x=vx; t.twist.angular.z=wz
            s.cmd.publish(t); s.spin(0.05)
        for _ in range(5): s.cmd.publish(TwistStamped()); s.spin(0.05)

rclpy.init(); d=D(); d.spin(3.0)
subprocess.run(["gz","service","-s",f"/world/{WORLD}/set_pose","--reqtype","gz.msgs.Pose",
  "--reptype","gz.msgs.Boolean","--timeout","8000","--req",
  'name: "rosbot", position: {x: 0, y: 0, z: 0}, orientation: {w: 1.0}'],capture_output=True,timeout=20)
time.sleep(4)
d.seed(0.0,0.0,0.0)
t0=truth(); a0=d.amcl()
print(f"  seeded: truth ({t0[0]:+.3f},{t0[1]:+.3f})  amcl ({a0[0]:+.3f},{a0[1]:+.3f})  err {1000*math.hypot(a0[0]-t0[0],a0[1]-t0[1]):.0f} mm")
# a square-ish path: the turns are where scan matching earns its keep
for vx,wz,dur in [(0.18,0.0,7),(0.0,0.6,5),(0.18,0.0,7),(0.0,0.6,5),(0.18,0.0,7),(0.0,0.6,5)]:
    d.drive(vx,wz,dur); d.spin(1.0)
d.spin(3.0)
t1=truth(); a1=d.amcl()
ep=1000*math.hypot(a1[0]-t1[0],a1[1]-t1[1])
ey=math.degrees(abs((a1[2]-t1[2]+math.pi)%(2*math.pi)-math.pi))
print(f"  after path: truth ({t1[0]:+.3f},{t1[1]:+.3f},{math.degrees(t1[2]):+.0f}d)  amcl ({a1[0]:+.3f},{a1[1]:+.3f},{math.degrees(a1[2]):+.0f}d)")
print(f"  DRIFT: {ep:.0f} mm, {ey:.1f} deg")
rclpy.try_shutdown()
